/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/gnss_fusion.hpp>
#include <formal_eskf/runtime/position.hpp>
#include <formal_eskf/runtime/velocity.hpp>

#include <array>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string_view>

#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{

using formal_eskf::Status;
using formal_eskf::runtime::FusionDecision;
using formal_eskf::runtime::GnssFusion;
using formal_eskf::runtime::GnssQualityDecision;
using TestContext = formal_eskf::test::Context;
using Group = GnssFusion::Group;
using StopReason = GnssFusion::StopReason;
constexpr std::string_view lifecycle_profile = "GNSS lifecycle";

struct GroupCase
{
    Group group;
    GnssFusion::GroupState GnssFusion::State::*state;
    GnssFusion::GroupConditions GnssFusion::Conditions::*conditions;
};

constexpr std::array groups{
    GroupCase{Group::horizontal_position, &GnssFusion::State::horizontal_position,
              &GnssFusion::Conditions::horizontal_position},
    GroupCase{Group::vertical_position, &GnssFusion::State::vertical_position,
              &GnssFusion::Conditions::vertical_position},
    GroupCase{Group::velocity, &GnssFusion::State::velocity, &GnssFusion::Conditions::velocity}};

struct Fixture
{
    GnssFusion monitor;
    GnssFusion::Conditions conditions{{true, true}, {true, true}, {true, true}};
    GnssFusion::Parameters parameters{25U, 100U};

    Status update(std::uint64_t now_us, GnssFusion::Sample const & sample = {})
    {
        return monitor.update(now_us, conditions, parameters, sample);
    }
};

bool same_group(GnssFusion::GroupState const & a, GnssFusion::GroupState const & b)
{
    return a.active == b.active && a.fusion_allowed == b.fusion_allowed && a.recovery_required == b.recovery_required &&
           a.stop_reason == b.stop_reason && a.start_time_us == b.start_time_us &&
           a.last_fuse_time_us == b.last_fuse_time_us;
}

void test_start_conditions(TestContext & test)
{
    // All independent enable/readiness combinations, quality outcomes and age boundaries.
    for (unsigned mask = 0U; mask < 64U; ++mask)
    {
        for (auto quality :
             {GnssQualityDecision::accepted, GnssQualityDecision::waiting, GnssQualityDecision::rejected})
        {
            for (std::uint64_t age : {0U, 25U, 26U})
            {
                Fixture fixture;
                unsigned bit = 0U;
                for (auto const & entry : groups)
                {
                    fixture.conditions.*entry.conditions = {(mask & (1U << bit)) != 0U, (mask & (2U << bit)) != 0U};
                    bit += 2U;
                }
                test.expect(fixture.update(1000U, {1000U - age, quality}) == Status::success, lifecycle_profile,
                            "valid startup call");
                for (auto const & entry : groups)
                {
                    auto const conditions = fixture.conditions.*entry.conditions;
                    auto const & state = fixture.monitor.state().*entry.state;
                    auto const reason = !conditions.enabled                        ? StopReason::disabled
                                        : !conditions.ready                        ? StopReason::not_ready
                                        : age > 25U                                ? StopReason::data_timeout
                                        : quality != GnssQualityDecision::accepted ? StopReason::waiting_for_quality
                                                                                   : StopReason::none;
                    bool const active = reason == StopReason::none;
                    test.expect(state.active == active && state.fusion_allowed == active &&
                                    state.stop_reason == reason && !state.recovery_required &&
                                    state.start_time_us == (active ? 1000U : 0U) && state.last_fuse_time_us == 0U,
                                lifecycle_profile, "independent startup, inclusive sample age, no fictitious fusion");
                }
            }
        }
    }

    Fixture empty;
    test.expect(empty.update(1U) == Status::success && !empty.monitor.state().horizontal_position.active &&
                    empty.monitor.state().horizontal_position.stop_reason == StopReason::data_timeout,
                lifecycle_profile, "no sample cannot start a session");
}

void test_group_deadlines(TestContext & test)
{
    for (auto const & entry : groups)
    {
        for (auto decision : {FusionDecision::fused, FusionDecision::rejected, FusionDecision::failed})
        {
            Fixture fixture;
            test.expect(fixture.update(1000U, {999U, GnssQualityDecision::accepted}) == Status::success &&
                            fixture.monitor.record_fusion(entry.group, 999U, decision) == Status::success,
                        lifecycle_profile, "record actual group result with its sample identity");
            auto const & state = fixture.monitor.state().*entry.state;
            test.expect(!state.fusion_allowed &&
                            state.last_fuse_time_us == (decision == FusionDecision::fused ? 1000U : 0U),
                        lifecycle_profile, "only fused records fusion horizon, not measurement timestamp");
            test.expect(fixture.monitor.record_fusion(entry.group, 999U, FusionDecision::fused) == Status::out_of_range,
                        lifecycle_profile, "each group can consume a sample only once");
            for (auto const & other : groups)
            {
                if (other.group != entry.group)
                {
                    test.expect((fixture.monitor.state().*other.state).fusion_allowed &&
                                    (fixture.monitor.state().*other.state).last_fuse_time_us == 0U,
                                lifecycle_profile, "recording one group does not consume another");
                }
            }

            test.expect(fixture.update(1050U, {1050U, GnssQualityDecision::accepted}) == Status::success &&
                            fixture.monitor.record_fusion(entry.group, 1050U, FusionDecision::fused) == Status::success,
                        lifecycle_profile, "successful group makes progress independently");
            test.expect(fixture.update(1100U, {1100U, GnssQualityDecision::accepted}) == Status::success,
                        lifecycle_profile, "fusion deadline equality is still allowed");
            for (auto const & other : groups)
            {
                test.expect((fixture.monitor.state().*other.state).fusion_allowed, lifecycle_profile,
                            "first-fusion deadline uses initial activation");
            }
            test.expect(fixture.update(1101U, {1101U, GnssQualityDecision::accepted}) == Status::success,
                        lifecycle_profile, "new good data cannot hide absence of fusion");
            for (auto const & other : groups)
            {
                auto const & current = fixture.monitor.state().*other.state;
                bool const progressing = other.group == entry.group;
                test.expect(current.active == progressing && current.fusion_allowed == progressing &&
                                current.recovery_required != progressing &&
                                current.stop_reason == (progressing ? StopReason::none : StopReason::fusion_timeout),
                            lifecycle_profile, "only groups without progress require recovery");
            }
            test.expect(fixture.update(1150U, {1150U, GnssQualityDecision::accepted}) == Status::success &&
                            state.fusion_allowed && !state.recovery_required,
                        lifecycle_profile, "last-success deadline equality is allowed");
            test.expect(fixture.update(1151U, {1151U, GnssQualityDecision::accepted}) == Status::success &&
                            !state.active && state.recovery_required,
                        lifecycle_profile, "last-success deadline expires strictly beyond limit");
        }
    }
}

void test_pauses_and_recovery(TestContext & test)
{
    Fixture fixture;
    test.expect(fixture.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success, lifecycle_profile,
                "start all groups");
    test.expect(fixture.update(1001U, {1001U, GnssQualityDecision::rejected}) == Status::success &&
                    fixture.monitor.state().velocity.active && !fixture.monitor.state().velocity.fusion_allowed,
                lifecycle_profile, "one quality rejection suspends this sample, not the active session");
    test.expect(fixture.update(1002U, {1002U, GnssQualityDecision::waiting}) == Status::success &&
                    fixture.monitor.state().velocity.active && !fixture.monitor.state().velocity.fusion_allowed,
                lifecycle_profile, "existing quality monitor owns its recovery wait");
    test.expect(fixture.update(1027U) == Status::success && fixture.monitor.state().velocity.active &&
                    !fixture.monitor.state().velocity.fusion_allowed,
                lifecycle_profile, "sample-less tick at age boundary never re-fuses old data");
    test.expect(fixture.update(1028U) == Status::success && !fixture.monitor.state().velocity.active &&
                    fixture.monitor.state().velocity.stop_reason == StopReason::data_timeout &&
                    !fixture.monitor.state().velocity.recovery_required,
                lifecycle_profile, "data loss stops before the longer fusion deadline");
    test.expect(fixture.update(1050U, {1050U, GnssQualityDecision::accepted}) == Status::success &&
                    fixture.monitor.state().velocity.fusion_allowed &&
                    fixture.monitor.state().velocity.start_time_us == 1000U,
                lifecycle_profile, "short outage can resume without renewing first-fusion deadline");

    fixture.conditions.horizontal_position.enabled = false;
    fixture.conditions.vertical_position.ready = false;
    test.expect(fixture.update(1101U) == Status::success, lifecycle_profile,
                "tick without any new data expires fusion");
    for (auto const & entry : groups)
    {
        auto const & state = fixture.monitor.state().*entry.state;
        test.expect(!state.active && !state.fusion_allowed && state.recovery_required, lifecycle_profile,
                    "disabled and unready groups retain the need for recovery");
    }
    test.expect(fixture.monitor.state().horizontal_position.stop_reason == StopReason::disabled &&
                    fixture.monitor.state().vertical_position.stop_reason == StopReason::not_ready,
                lifecycle_profile, "stop reason reports immediate inhibition without erasing recovery");
    fixture.conditions = {{true, true}, {true, true}, {true, true}};
    test.expect(fixture.update(1102U, {1102U, GnssQualityDecision::accepted}) == Status::success &&
                    !fixture.monitor.state().velocity.fusion_allowed,
                lifecycle_profile, "good data and enable toggles cannot acknowledge recovery");

    for (auto const & entry : groups)
    {
        auto const before = fixture.monitor.state();
        test.expect(fixture.monitor.restart(entry.group) == Status::success &&
                        same_group(fixture.monitor.state().*entry.state, {}),
                    lifecycle_profile, "explicit restart clears only the selected lifecycle");
        test.expect(fixture.monitor.record_fusion(entry.group, 1102U, FusionDecision::fused) == Status::out_of_range,
                    lifecycle_profile, "restart cannot authorize the already consumed sample");
        for (auto const & other : groups)
        {
            if (other.group != entry.group)
            {
                test.expect(same_group(fixture.monitor.state().*other.state, before.*other.state), lifecycle_profile,
                            "restart preserves all other group history");
            }
        }
    }
    test.expect(fixture.update(1103U) == Status::success && !fixture.monitor.state().velocity.active, lifecycle_profile,
                "restart waits for a new accepted sample, not cached quality");
    test.expect(fixture.update(1104U, {1104U, GnssQualityDecision::waiting}) == Status::success &&
                    !fixture.monitor.state().velocity.active,
                lifecycle_profile, "restart does not bypass quality qualification");
    test.expect(fixture.update(1105U, {1105U, GnssQualityDecision::accepted}) == Status::success &&
                    fixture.monitor.state().velocity.fusion_allowed &&
                    fixture.monitor.state().velocity.start_time_us == 1105U &&
                    fixture.monitor.state().velocity.last_fuse_time_us == 0U,
                lifecycle_profile, "explicitly restarted session still has no fictitious fusion");
}

void test_persistent_rejection(TestContext & test)
{
    for (auto decision : {FusionDecision::rejected, FusionDecision::failed})
    {
        Fixture fixture;
        for (std::uint64_t time = 1000U; time <= 1100U; ++time)
        {
            test.expect(fixture.update(time, {time, GnssQualityDecision::accepted}) == Status::success,
                        lifecycle_profile, "quality stays good throughout persistent fusion rejection/failure");
            for (auto const & entry : groups)
            {
                test.expect(fixture.monitor.record_fusion(entry.group, time, decision) == Status::success &&
                                (fixture.monitor.state().*entry.state).last_fuse_time_us == 0U,
                            lifecycle_profile, "repeated unsuccessful fusion never refreshes progress");
            }
        }
        test.expect(fixture.update(1101U, {1101U, GnssQualityDecision::accepted}) == Status::success, lifecycle_profile,
                    "new accepted sample at expired fusion deadline");
        for (auto const & entry : groups)
        {
            test.expect((fixture.monitor.state().*entry.state).recovery_required &&
                            fixture.monitor.record_fusion(entry.group, 1101U, FusionDecision::fused) ==
                                Status::out_of_range,
                        lifecycle_profile, "cannot refresh a timed-out session with unsolicited success");
        }
    }

    for (auto quality : {GnssQualityDecision::waiting, GnssQualityDecision::rejected})
    {
        Fixture fixture;
        test.expect(fixture.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success, lifecycle_profile,
                    "activate before losing quality");
        for (std::uint64_t time = 1001U; time <= 1101U; ++time)
        {
            test.expect(fixture.update(time, {time, quality}) == Status::success &&
                            !fixture.monitor.state().velocity.fusion_allowed,
                        lifecycle_profile, "non-accepted quality cannot authorize a measurement");
        }
        test.expect(fixture.monitor.state().velocity.recovery_required, lifecycle_profile,
                    "persistent quality loss also expires successful-fusion deadline");
    }

    for (auto const & entry : groups)
    {
        for (bool lose_readiness : {false, true})
        {
            Fixture fixture;
            test.expect(fixture.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success,
                        lifecycle_profile, "activate before temporarily losing enable/readiness");
            auto & conditions = fixture.conditions.*entry.conditions;
            conditions = {lose_readiness, !lose_readiness};
            test.expect(fixture.update(1001U, {1001U, GnssQualityDecision::accepted}) == Status::success &&
                            !(fixture.monitor.state().*entry.state).active &&
                            !(fixture.monitor.state().*entry.state).fusion_allowed,
                        lifecycle_profile, "disable or readiness loss stops the group immediately");
            conditions = {true, true};
            test.expect(fixture.update(1002U) == Status::success && !(fixture.monitor.state().*entry.state).active,
                        lifecycle_profile, "re-enable alone cannot reuse a previous sample");
            test.expect(fixture.update(1003U, {1003U, GnssQualityDecision::accepted}) == Status::success &&
                            (fixture.monitor.state().*entry.state).fusion_allowed &&
                            (fixture.monitor.state().*entry.state).start_time_us == 1000U,
                        lifecycle_profile, "short inhibition retains the original first-fusion deadline");
        }
    }
}

void test_invalid_calls(TestContext & test)
{
    Fixture baseline;
    test.expect(baseline.update(1000U, {999U, GnssQualityDecision::accepted}) == Status::success &&
                    baseline.monitor.record_fusion(Group::velocity, 999U, FusionDecision::fused) == Status::success,
                lifecycle_profile, "seed history for invalid-call checks");

    auto check_failure = [&](std::uint64_t now, GnssFusion::Sample const & sample,
                             GnssFusion::Parameters const & parameters, Status expected)
    {
        auto fixture = baseline;
        auto before = fixture.monitor.state();
        test.expect(fixture.monitor.update(now, fixture.conditions, parameters, sample) == expected, lifecycle_profile,
                    "invalid update reports expected status");
        for (auto const & entry : groups)
        {
            (before.*entry.state).fusion_allowed = false;
            test.expect(same_group(fixture.monitor.state().*entry.state, before.*entry.state), lifecycle_profile,
                        "invalid update revokes permissions, preserves other history");
            test.expect(fixture.monitor.record_fusion(entry.group, 999U, FusionDecision::fused) == Status::out_of_range,
                        lifecycle_profile, "failed update cannot leave stale permission");
        }
        test.expect(fixture.update(1001U, {1000U, GnssQualityDecision::accepted}) == Status::success, lifecycle_profile,
                    "invalid update did not consume time or change configuration");
    };
    for (std::uint64_t now : {0U, 999U, 1000U})
    {
        check_failure(now, {}, baseline.parameters, Status::out_of_range);
    }
    for (std::uint64_t sample : {998U, 999U, 1002U})
    {
        check_failure(1001U, {sample, GnssQualityDecision::accepted}, baseline.parameters, Status::out_of_range);
    }
    for (auto quality : {GnssQualityDecision::invalid_call, static_cast<GnssQualityDecision>(99)})
    {
        check_failure(1001U, {1001U, quality}, baseline.parameters, Status::domain_error);
    }
    for (auto parameters : {GnssFusion::Parameters{0U, 100U}, GnssFusion::Parameters{25U, 0U},
                            GnssFusion::Parameters{26U, 100U}, GnssFusion::Parameters{25U, 101U}})
    {
        check_failure(1001U, {}, parameters, Status::domain_error);
    }

    auto before = baseline.monitor.state();
    test.expect(baseline.monitor.record_fusion(static_cast<Group>(99), 999U, FusionDecision::fused) ==
                        Status::domain_error &&
                    baseline.monitor.restart(static_cast<Group>(99)) == Status::domain_error &&
                    baseline.monitor.record_fusion(Group::horizontal_position, 999U, static_cast<FusionDecision>(99)) ==
                        Status::domain_error,
                lifecycle_profile, "invalid enums cannot index storage or mutate history");
    for (std::uint64_t time : {0U, 998U, 1000U})
    {
        test.expect(baseline.monitor.record_fusion(Group::horizontal_position, time, FusionDecision::fused) ==
                        Status::out_of_range,
                    lifecycle_profile, "feedback must identify the currently permitted sample");
    }
    for (auto const & entry : groups)
    {
        test.expect(same_group(baseline.monitor.state().*entry.state, before.*entry.state), lifecycle_profile,
                    "invalid feedback/restart preserves history and valid pending result");
    }
    test.expect(baseline.update(1001U, {1001U, GnssQualityDecision::accepted}) == Status::success &&
                    baseline.monitor.record_fusion(Group::horizontal_position, 999U, FusionDecision::fused) ==
                        Status::out_of_range &&
                    baseline.monitor.record_fusion(Group::horizontal_position, 1001U, FusionDecision::fused) ==
                        Status::success,
                lifecycle_profile, "feedback from a previous tick cannot refresh a new session");
    baseline.monitor.reset();
    baseline.parameters = {1U, 1U};
    test.expect(baseline.update(1U, {1U, GnssQualityDecision::accepted}) == Status::success, lifecycle_profile,
                "full reset permits a new timeline and configuration");

    Fixture limits;
    auto const maximum = std::numeric_limits<std::uint64_t>::max();
    limits.parameters = {1U, 1U};
    test.expect(limits.update(maximum - 2U, {maximum - 2U, GnssQualityDecision::accepted}) == Status::success &&
                    limits.update(maximum - 1U) == Status::success && limits.monitor.state().velocity.active &&
                    limits.update(maximum) == Status::success && limits.monitor.state().velocity.recovery_required,
                lifecycle_profile, "elapsed subtraction is safe through UINT64_MAX");
    test.expect(limits.update(1U) == Status::out_of_range, lifecycle_profile, "wrapped clock requires explicit reset");
    limits.monitor.reset();
    limits.parameters = {maximum, maximum};
    test.expect(limits.update(1U, {1U, GnssQualityDecision::accepted}) == Status::success &&
                    limits.update(maximum) == Status::success && limits.monitor.state().velocity.active &&
                    !limits.monitor.state().velocity.fusion_allowed,
                lifecycle_profile, "maximum duration uses no overflowing deadline addition");
}

template <typename Linalg> void test_quality_to_fusion(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    using Matrix2 = formal_eskf::linalg::Matrix<Linalg, 2U, 2U>;
    using Matrix3 = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    using Vector2 = formal_eskf::linalg::Matrix<Linalg, 2U, 1U>;
    using Vector3 = formal_eskf::linalg::Matrix<Linalg, 3U, 1U>;
    formal_eskf::runtime::GnssQuality<Linalg> quality;
    formal_eskf::runtime::GnssQualityParameters<Scalar> quality_parameters;
    quality_parameters.minimum_health_time_us = 1000000U;
    formal_eskf::runtime::GnssQualitySample<Linalg> sample;
    sample.fix_type = 3U;
    sample.satellites = 10U;
    sample.time_us = 1U;
    auto quality_result = quality.update(sample, {}, sample.time_us, quality_parameters);
    Fixture fixture;
    test.expect(fixture.update(sample.time_us, {sample.time_us, quality_result.decision}) == Status::success &&
                    !fixture.monitor.state().velocity.active,
                profile, "real quality monitor initially waits; lifecycle does not duplicate qualification");
    sample.time_us = 1000002U;
    quality_result = quality.update(sample, {}, sample.time_us, quality_parameters);
    test.expect(quality_result.decision == GnssQualityDecision::accepted &&
                    fixture.update(sample.time_us, {sample.time_us, quality_result.decision}) == Status::success &&
                    fixture.monitor.state().horizontal_position.fusion_allowed &&
                    fixture.monitor.state().vertical_position.fusion_allowed &&
                    fixture.monitor.state().velocity.fusion_allowed,
                profile, "actual quality qualification enables groups");

    formal_eskf::configuration::Ins::NominalState<Linalg> state;
    auto covariance = formal_eskf::linalg::Matrix<Linalg, 15U, 15U>::identity();
    Vector2 position;
    position.set(0U, Scalar{1});
    auto const minimum_norm = static_cast<Scalar>(1e-6);
    auto const horizontal = formal_eskf::runtime::fuse_horizontal_position(
        state, covariance, position, Matrix2::identity(), minimum_norm, Scalar{3}, state, covariance);
    auto const vertical = formal_eskf::runtime::fuse_vertical_position(state, covariance, Scalar{100}, Scalar{1},
                                                                       minimum_norm, Scalar{3}, state, covariance);
    auto const velocity = formal_eskf::runtime::fuse_velocity(state, covariance, Vector3::zero(), Matrix3::identity(),
                                                              minimum_norm, Scalar{0}, state, covariance);
    test.expect(horizontal.decision == FusionDecision::fused && vertical.decision == FusionDecision::rejected &&
                    velocity.decision == FusionDecision::failed,
                profile, "real gated APIs yield distinct fusion, rejection and numerical failure");
    test.expect(fixture.monitor.record_fusion(Group::horizontal_position, sample.time_us, horizontal.decision) ==
                        Status::success &&
                    fixture.monitor.record_fusion(Group::vertical_position, sample.time_us, vertical.decision) ==
                        Status::success &&
                    fixture.monitor.record_fusion(Group::velocity, sample.time_us, velocity.decision) ==
                        Status::success &&
                    fixture.monitor.state().horizontal_position.last_fuse_time_us == sample.time_us &&
                    fixture.monitor.state().vertical_position.last_fuse_time_us == 0U &&
                    fixture.monitor.state().velocity.last_fuse_time_us == 0U,
                profile, "actual results update only successfully fused group's history");
    auto const fused_position = state.p_n;
    auto const fused_covariance = covariance;
    test.expect(fixture.update(sample.time_us + 101U) == Status::success &&
                    fixture.monitor.state().horizontal_position.recovery_required &&
                    formal_eskf::linalg::max_abs(state.p_n - fused_position) == Scalar{0} &&
                    formal_eskf::linalg::max_abs(covariance - fused_covariance) == Scalar{0},
                profile, "no-data tick requests recovery without touching estimate or covariance");
}

} /* end anonymous namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    TestContext test;
    test_start_conditions(test);
    test_group_deadlines(test);
    test_pauses_and_recovery(test);
    test_persistent_rejection(test);
    test_invalid_calls(test);
    test_quality_to_fusion<formal_eskf::test::Backend<float>>(test, "float");
    test_quality_to_fusion<formal_eskf::test::Backend<double>>(test, "double");
    if (test.failures() != 0)
    {
        std::cerr << test.failures() << " GNSS fusion check(s) failed\n";
        return 1;
    }
    std::cout << "GNSS fusion checks passed\n";
    return 0;
}
