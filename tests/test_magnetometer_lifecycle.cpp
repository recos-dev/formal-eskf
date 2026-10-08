/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/magnetometer_fusion.hpp>
#include <formal_eskf/runtime/magnetometer.hpp>

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
using formal_eskf::runtime::MagnetometerFusion;
using StopReason = MagnetometerFusion::StopReason;
using RecoveryReason = MagnetometerFusion::RecoveryReason;
using TestContext = formal_eskf::test::Context;
constexpr std::string_view profile = "magnetometer lifecycle";

struct Fixture
{
    MagnetometerFusion monitor;
    MagnetometerFusion::Parameters parameters{20U, 10U, 50U};
    MagnetometerFusion::Conditions conditions{true, true};
    MagnetometerFusion::Source source{1U, 0U};

    Status sample(std::uint64_t time, bool accepted = true)
    {
        return monitor.update(time, conditions, parameters, {time, source, accepted});
    }

    Status tick(std::uint64_t time) { return monitor.update(time, conditions, parameters, {}); }

    void qualify(TestContext & test, std::uint64_t first = 100U)
    {
        test.expect(sample(first) == Status::success && sample(first + 10U) == Status::success &&
                        sample(first + 20U) == Status::success && monitor.state().fusion_allowed,
                    profile, "three consecutive samples qualify at the inclusive health boundary");
    }
};

bool same_state(MagnetometerFusion::State const & a, MagnetometerFusion::State const & b)
{
    return a.active == b.active && a.fusion_allowed == b.fusion_allowed && a.stop_reason == b.stop_reason &&
           a.recovery_reason == b.recovery_reason && a.health_start_time_us == b.health_start_time_us &&
           a.start_time_us == b.start_time_us && a.last_fuse_time_us == b.last_fuse_time_us;
}

void test_qualification(TestContext & test)
{
    for (unsigned mask = 0U; mask < 4U; ++mask)
    {
        for (bool accepted : {false, true})
        {
            for (std::uint64_t age : {0U, 10U, 11U})
            {
                Fixture fixture;
                fixture.conditions = {(mask & 1U) != 0U, (mask & 2U) != 0U};
                test.expect(fixture.monitor.update(100U, fixture.conditions, fixture.parameters,
                                                   {100U - age, fixture.source, accepted}) == Status::success,
                            profile, "valid first sample");
                auto const & state = fixture.monitor.state();
                auto const expected = !fixture.conditions.enabled ? StopReason::disabled
                                      : !fixture.conditions.ready ? StopReason::not_ready
                                      : age > 10U                 ? StopReason::data_timeout
                                      : !accepted                 ? StopReason::field_rejected
                                                                  : StopReason::waiting_for_health;
                test.expect(!state.active && !state.fusion_allowed && state.stop_reason == expected &&
                                state.start_time_us == 0U && state.last_fuse_time_us == 0U &&
                                state.recovery_reason == RecoveryReason::none &&
                                state.health_start_time_us ==
                                    (expected == StopReason::waiting_for_health ? 100U - age : 0U),
                            profile, "startup conditions, freshness equality, no invented health or fusion");
            }
        }
    }
    Fixture fixture;
    test.expect(fixture.tick(1U) == Status::success && fixture.monitor.state().stop_reason == StopReason::data_timeout,
                profile, "no initial data cannot start");
    test.expect(fixture.monitor.update(2U, fixture.conditions, fixture.parameters, {0U, {99U, 99U}, true}) ==
                        Status::success &&
                    fixture.monitor.state().health_start_time_us == 0U,
                profile, "no-sample metadata cannot bind a source or fabricate a good reading");
    test.expect(fixture.sample(100U) == Status::success && fixture.sample(110U) == Status::success &&
                    fixture.sample(119U) == Status::success && !fixture.monitor.state().fusion_allowed,
                profile, "just before health duration is still waiting");
    test.expect(fixture.sample(120U) == Status::success && fixture.monitor.state().fusion_allowed &&
                    fixture.monitor.state().active && fixture.monitor.state().stop_reason == StopReason::none &&
                    fixture.monitor.state().start_time_us == 120U && fixture.monitor.state().last_fuse_time_us == 0U,
                profile, "health boundary authorizes once but does not claim a fusion");
    test.expect(fixture.tick(121U) == Status::success && fixture.monitor.state().active &&
                    !fixture.monitor.state().fusion_allowed && fixture.monitor.state().last_fuse_time_us == 0U,
                profile, "no new sample revokes old permission even if feedback was omitted");

    Fixture delayed;
    delayed.parameters.maximum_sample_age_us = 100U;
    test.expect(delayed.sample(100U) == Status::success && delayed.tick(130U) == Status::success &&
                    !delayed.monitor.state().active,
                profile, "wall/fusion time alone cannot qualify one good reading");
    test.expect(delayed.monitor.update(131U, delayed.conditions, delayed.parameters, {101U, delayed.source, true}) ==
                        Status::success &&
                    !delayed.monitor.state().active &&
                    delayed.monitor.update(132U, delayed.conditions, delayed.parameters,
                                           {120U, delayed.source, true}) == Status::success &&
                    delayed.monitor.state().fusion_allowed &&
                    delayed.monitor.record_fusion(120U, FusionDecision::fused) == Status::success &&
                    delayed.monitor.state().last_fuse_time_us == 132U,
                profile, "health uses sample span; successful fusion time is the supplied fusion horizon");
}

void test_field_and_data_recovery(TestContext & test)
{
    Fixture fixture;
    fixture.qualify(test);
    test.expect(fixture.monitor.record_fusion(120U, FusionDecision::fused) == Status::success &&
                    fixture.sample(130U, false) == Status::success && !fixture.monitor.state().active &&
                    fixture.monitor.state().stop_reason == StopReason::field_rejected &&
                    fixture.monitor.state().health_start_time_us == 0U,
                profile, "bad field immediately stops fusion and clears health duration");
    test.expect(fixture.sample(140U) == Status::success && fixture.sample(150U) == Status::success &&
                    !fixture.monitor.state().active && fixture.sample(160U) == Status::success &&
                    fixture.monitor.state().fusion_allowed && fixture.monitor.state().start_time_us == 120U &&
                    fixture.monitor.state().last_fuse_time_us == 120U &&
                    fixture.monitor.record_fusion(160U, FusionDecision::fused) == Status::success,
                profile, "short disturbance requires a new good interval without renewing the progress deadline");
    test.expect(fixture.tick(170U) == Status::success && fixture.monitor.state().active &&
                    !fixture.monitor.state().fusion_allowed && fixture.tick(171U) == Status::success &&
                    fixture.monitor.state().stop_reason == StopReason::data_timeout &&
                    !fixture.monitor.state().active && fixture.monitor.state().health_start_time_us == 0U,
                profile, "sample-age equality allowed, one microsecond later stops even without new data");
    test.expect(fixture.sample(180U) == Status::success && fixture.sample(190U) == Status::success &&
                    fixture.sample(200U) == Status::success && fixture.monitor.state().fusion_allowed,
                profile, "short outage can recover by requalification before fusion timeout");

    Fixture gap;
    test.expect(gap.sample(100U) == Status::success && gap.sample(111U) == Status::success &&
                    gap.monitor.state().health_start_time_us == 111U && !gap.monitor.state().active &&
                    gap.sample(121U) == Status::success && gap.sample(131U) == Status::success &&
                    gap.monitor.state().active,
                profile, "gap beyond maximum restarts health even if no intermediate tick detected it");
    Fixture alternating;
    for (std::uint64_t time = 100U; time <= 160U; time += 10U)
    {
        test.expect(alternating.sample(time, time % 20U == 0U) == Status::success &&
                        !alternating.monitor.state().fusion_allowed,
                    profile, "isolated good readings never accumulate across field failures");
    }
}

void test_deadlines_and_feedback(TestContext & test)
{
    for (unsigned mode = 0U; mode < 3U; ++mode)
    {
        Fixture outage;
        outage.qualify(test);
        test.expect(outage.monitor.record_fusion(120U, FusionDecision::fused) == Status::success, profile,
                    "success before a no-data interruption");
        outage.conditions = {mode != 1U, mode != 2U};
        test.expect(outage.tick(170U) == Status::success && !outage.monitor.state().fusion_allowed &&
                        outage.monitor.state().recovery_reason == RecoveryReason::none &&
                        outage.tick(171U) == Status::success &&
                        outage.monitor.state().recovery_reason == RecoveryReason::fusion_timeout &&
                        outage.monitor.state().last_fuse_time_us == 120U,
                    profile, "fusion timeout advances on no-data ticks even while disabled or unready");
        outage.conditions = {true, true};
        test.expect(outage.sample(180U) == Status::success &&
                        outage.monitor.state().stop_reason == StopReason::recovery_required &&
                        !outage.monitor.state().fusion_allowed,
                    profile, "fresh data after a long outage cannot clear recovery");
    }
    for (bool record_first_success : {false, true})
    {
        Fixture fixture;
        fixture.qualify(test);
        test.expect(fixture.monitor.record_fusion(120U, record_first_success
                                                            ? FusionDecision::fused
                                                            : FusionDecision::rejected) == Status::success,
                    profile, "record the actual first result");
        for (std::uint64_t time = 130U; time <= 170U; time += 10U)
        {
            test.expect(fixture.sample(time) == Status::success && fixture.monitor.state().fusion_allowed &&
                            fixture.monitor.record_fusion(time, FusionDecision::rejected) == Status::success &&
                            fixture.monitor.state().last_fuse_time_us == (record_first_success ? 120U : 0U),
                        profile,
                        "gate rejections consume permission without refreshing progress; deadline equality is allowed");
        }
        test.expect(fixture.sample(171U) == Status::success && !fixture.monitor.state().fusion_allowed &&
                        fixture.monitor.state().recovery_reason == RecoveryReason::fusion_timeout,
                    profile, "persistent rejection times out even when every field check is good");
        fixture.conditions.enabled = false;
        test.expect(fixture.sample(172U) == Status::success &&
                        fixture.monitor.state().stop_reason == StopReason::disabled,
                    profile, "disabled display does not erase the recovery cause");
        fixture.conditions.enabled = true;
        test.expect(fixture.sample(173U) == Status::success &&
                        fixture.monitor.state().stop_reason == StopReason::recovery_required &&
                        fixture.monitor.state().recovery_reason == RecoveryReason::fusion_timeout,
                    profile, "enable toggles and fresh good samples cannot clear a fault");
        fixture.monitor.restart();
        test.expect(!fixture.monitor.state().active && fixture.monitor.state().last_fuse_time_us == 0U &&
                        fixture.monitor.state().recovery_reason == RecoveryReason::none &&
                        fixture.monitor.record_fusion(173U, FusionDecision::fused) == Status::out_of_range,
                    profile, "restart acknowledges re-entry without fabricating health, permission or fusion");
        fixture.qualify(test, 180U);
    }

    Fixture feedback;
    feedback.qualify(test);
    auto const before = feedback.monitor.state();
    test.expect(feedback.monitor.record_fusion(119U, FusionDecision::fused) == Status::out_of_range, profile,
                "wrong-sample feedback is rejected");
    test.expect(same_state(feedback.monitor.state(), before), profile, "wrong-sample feedback preserves metadata");
    // Intentionally assert the known result for an invalid enum; retain this regression check.
    // cppcheck-suppress knownConditionTrueFalse
    test.expect(feedback.monitor.record_fusion(120U, static_cast<FusionDecision>(99)) == Status::domain_error, profile,
                "invalid-result feedback is rejected");
    test.expect(same_state(feedback.monitor.state(), before), profile, "invalid-result feedback preserves metadata");
    test.expect(feedback.monitor.record_fusion(120U, FusionDecision::fused) == Status::success &&
                    feedback.monitor.record_fusion(120U, FusionDecision::fused) == Status::out_of_range,
                profile, "feedback is consumed once");
    test.expect(feedback.sample(130U) == Status::success &&
                    feedback.monitor.record_fusion(130U, FusionDecision::failed) == Status::success &&
                    feedback.monitor.state().recovery_reason == RecoveryReason::fusion_failed &&
                    !feedback.monitor.state().active && feedback.monitor.state().last_fuse_time_us == 120U,
                profile, "numerical failure immediately requires caller-directed recovery");
    test.expect(feedback.sample(140U) == Status::success && !feedback.monitor.state().fusion_allowed &&
                    feedback.tick(200U) == Status::success &&
                    feedback.monitor.state().recovery_reason == RecoveryReason::fusion_failed,
                profile, "a later timeout does not erase the original recovery cause");
}

void test_stops_and_source_changes(TestContext & test)
{
    for (bool disable : {false, true})
    {
        Fixture fixture;
        fixture.qualify(test);
        test.expect(fixture.monitor.record_fusion(120U, FusionDecision::fused) == Status::success, profile,
                    "initial success");
        fixture.conditions = {!disable, disable};
        test.expect(fixture.sample(130U) == Status::success && !fixture.monitor.state().active &&
                        fixture.monitor.state().stop_reason == (disable ? StopReason::disabled : StopReason::not_ready),
                    profile, "disable or loss of readiness clears health immediately");
        fixture.conditions = {true, true};
        fixture.qualify(test, 140U);
        test.expect(fixture.monitor.state().start_time_us == 120U &&
                        fixture.monitor.state().last_fuse_time_us == 120U && fixture.sample(171U) == Status::success &&
                        fixture.monitor.state().recovery_reason == RecoveryReason::fusion_timeout,
                    profile, "pause/requalification cannot evade the original unsuccessful-fusion deadline");
    }

    for (bool calibration_change : {false, true})
    {
        Fixture fixture;
        fixture.qualify(test);
        if (calibration_change)
        {
            ++fixture.source.calibration_count;
        }
        else
        {
            ++fixture.source.device_id;
        }
        test.expect(
            fixture.sample(130U) == Status::success && !fixture.monitor.state().fusion_allowed &&
                fixture.monitor.state().recovery_reason == RecoveryReason::source_changed &&
                fixture.monitor.state().health_start_time_us == 0U && fixture.monitor.state().start_time_us == 0U &&
                fixture.monitor.state().last_fuse_time_us == 0U &&
                fixture.monitor.record_fusion(120U, FusionDecision::fused) == Status::out_of_range,
            profile, "source/calibration change drops old history and stale feedback, requiring explicit recovery");
        fixture.monitor.restart();
        test.expect(fixture.sample(130U) == Status::out_of_range && fixture.tick(131U) == Status::success &&
                        !fixture.monitor.state().active,
                    profile, "restart retains clock and sample watermarks; old data cannot requalify");
        fixture.qualify(test, 140U);
        test.expect(fixture.monitor.state().recovery_reason == RecoveryReason::none, profile,
                    "new source remains bound after restart and can qualify normally");
    }
}

void test_invalid_calls(TestContext & test)
{
    for (unsigned invalid_case = 0U; invalid_case < 10U; ++invalid_case)
    {
        Fixture fixture;
        fixture.qualify(test);
        auto parameters = fixture.parameters;
        MagnetometerFusion::Sample sample{130U, fixture.source, true};
        std::uint64_t now = 130U;
        Status expected = Status::domain_error;
        switch (invalid_case)
        {
        case 0U:
            parameters.health_time_us = 0U;
            break;
        case 1U:
            parameters.maximum_sample_age_us = 0U;
            break;
        case 2U:
            parameters.fusion_timeout_us = 0U;
            break;
        case 3U:
            ++parameters.health_time_us;
            break;
        case 4U:
            ++parameters.maximum_sample_age_us;
            break;
        case 5U:
            ++parameters.fusion_timeout_us;
            break;
        case 6U:
            now = 120U;
            expected = Status::out_of_range;
            break;
        case 7U:
            sample.time_us = 120U;
            expected = Status::out_of_range;
            break;
        case 8U:
            sample.time_us = 131U;
            expected = Status::out_of_range;
            break;
        default:
            sample.source.device_id = 0U;
            break;
        }
        test.expect(fixture.monitor.update(now, fixture.conditions, parameters, sample) == expected &&
                        !fixture.monitor.state().active && !fixture.monitor.state().fusion_allowed &&
                        fixture.monitor.state().health_start_time_us == 0U &&
                        fixture.monitor.state().stop_reason == StopReason::invalid_call &&
                        fixture.monitor.state().start_time_us == 120U &&
                        fixture.monitor.record_fusion(120U, FusionDecision::fused) == Status::out_of_range,
                    profile, "invalid calls revoke stale permission and health but preserve session progress");
        fixture.qualify(test, 130U);
        test.expect(fixture.sample(171U) == Status::success &&
                        fixture.monitor.state().recovery_reason == RecoveryReason::fusion_timeout,
                    profile, "invalid calls neither consume valid timestamps nor renew a progress deadline");
    }
    Fixture fixture;
    test.expect(fixture.tick(0U) == Status::out_of_range, profile, "zero current time is invalid");
    fixture.qualify(test);
    fixture.monitor.restart();
    test.expect(fixture.tick(119U) == Status::out_of_range, profile, "restart cannot rewind the timeline");
    fixture.monitor.reset();
    fixture.parameters.health_time_us = 10U;
    test.expect(fixture.sample(1U) == Status::success && fixture.sample(11U) == Status::success &&
                    fixture.monitor.state().fusion_allowed,
                profile, "full reset permits explicit new timeline and duration configuration");

    // Durations are compared by subtraction, never timestamp + duration.
    constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
    Fixture large;
    large.qualify(test, maximum - 30U);
    test.expect(large.monitor.record_fusion(maximum - 10U, FusionDecision::fused) == Status::success &&
                    large.sample(maximum) == Status::success && large.monitor.state().fusion_allowed &&
                    large.tick(0U) == Status::out_of_range,
                profile, "timestamp arithmetic remains bounded near UINT64_MAX and never wraps a clock");
    Fixture long_wait;
    long_wait.parameters = {maximum, maximum, maximum};
    test.expect(long_wait.sample(1U) == Status::success && long_wait.sample(maximum) == Status::success &&
                    !long_wait.monitor.state().active &&
                    long_wait.monitor.state().recovery_reason == RecoveryReason::none,
                profile, "unreachable enormous health duration does not overflow into early qualification");
}

template <typename Matrix> bool same_matrix(Matrix const & a, Matrix const & b)
{
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            if (a(row, column) != b(row, column))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename Linalg, typename Configuration>
void test_real_fusion(TestContext & test, std::string_view scalar_profile)
{
    using value_type = typename Linalg::value_type;
    using vector_type = formal_eskf::linalg::Matrix<Linalg, 3U, 1U>;
    using matrix_type = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    typename Configuration::template NominalState<Linalg> state;
    formal_eskf::linalg::Matrix<Linalg, Configuration::error_state_dimension, Configuration::error_state_dimension> P;
    auto const reference = vector_type::from_row_major({value_type{30}, value_type{0}, value_type{40}});
    auto V = matrix_type::identity();
    formal_eskf::runtime::MagneticFieldParameters<value_type> const field_parameters{
        value_type{5}, static_cast<value_type>(0.2), static_cast<value_type>(0.2), value_type{1}};
    Fixture fixture;
    auto step = [&](std::uint64_t time, vector_type const & measurement)
    {
        // The preliminary check accumulates health without running correction.
        // The gated transaction rechecks the same sample/prior before publication.
        auto const field =
            formal_eskf::runtime::check_magnetic_field(state.q_nb, measurement, reference, field_parameters, false);
        test.expect(fixture.sample(time, formal_eskf::succeeded(field.status) && field.diagnostics_valid &&
                                             field.accepted) == Status::success,
                    scalar_profile, "feed actual field check into lifecycle");
        auto decision = FusionDecision::rejected;
        if (fixture.monitor.state().fusion_allowed)
        {
            auto const result =
                formal_eskf::runtime::fuse_magnetometer(state, P, measurement, reference, V, field_parameters, false,
                                                        static_cast<value_type>(1e-6), value_type{3}, state, P);
            decision = result.fusion.decision;
            test.expect(fixture.monitor.record_fusion(time, decision) == Status::success, scalar_profile,
                        "record actual gated correction outcome");
        }
        return decision;
    };
    test.expect(step(100U, reference) == FusionDecision::rejected &&
                    step(110U, reference) == FusionDecision::rejected &&
                    step(120U, reference) == FusionDecision::fused && fixture.monitor.state().last_fuse_time_us == 120U,
                scalar_profile, "real AHRS/INS correction is reached only after qualification");
    auto const before_q = state.q_nb.coefficients();
    auto const before_P = P;
    test.expect(step(130U, reference * value_type{2}) == FusionDecision::rejected &&
                    fixture.monitor.state().stop_reason == StopReason::field_rejected &&
                    same_matrix(state.q_nb.coefficients(), before_q) && same_matrix(P, before_P),
                scalar_profile, "actual strength disturbance stops the session without changing estimator outputs");
    test.expect(step(140U, reference) == FusionDecision::rejected &&
                    step(150U, reference) == FusionDecision::rejected && step(160U, reference) == FusionDecision::fused,
                scalar_profile, "good data requalifies after short disturbance");
    auto const outlier = vector_type::from_row_major({value_type{0}, value_type{30}, value_type{40}});
    for (std::uint64_t time = 170U; time <= 210U; time += 10U)
    {
        test.expect(step(time, outlier) == FusionDecision::rejected && fixture.monitor.state().active &&
                        fixture.monitor.state().last_fuse_time_us == 160U,
                    scalar_profile, "good field geometry with actual innovation rejection cannot refresh progress");
    }
    test.expect(step(211U, reference) == FusionDecision::rejected &&
                    fixture.monitor.state().recovery_reason == RecoveryReason::fusion_timeout &&
                    same_matrix(state.q_nb.coefficients(), before_q) && same_matrix(P, before_P),
                scalar_profile, "persistent innovation rejection requires recovery even when a good reading returns");
    fixture.monitor.restart();
    test.expect(step(220U, reference) == FusionDecision::rejected &&
                    step(230U, reference) == FusionDecision::rejected && step(240U, reference) == FusionDecision::fused,
                scalar_profile, "explicit recovery still requires new consecutive good samples");
    V(0U, 0U) = value_type{-1};
    test.expect(step(250U, reference) == FusionDecision::failed &&
                    fixture.monitor.state().recovery_reason == RecoveryReason::fusion_failed &&
                    fixture.monitor.state().last_fuse_time_us == 240U &&
                    same_matrix(state.q_nb.coefficients(), before_q) && same_matrix(P, before_P),
                scalar_profile, "actual numerical error propagates to recovery without state/covariance reset");
}

} /* end namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    TestContext test;
    test_qualification(test);
    test_field_and_data_recovery(test);
    test_deadlines_and_feedback(test);
    test_stops_and_source_changes(test);
    test_invalid_calls(test);
    test_real_fusion<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ahrs>(test, "float AHRS");
    test_real_fusion<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ins>(test, "float INS");
    test_real_fusion<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ahrs>(test, "double AHRS");
    test_real_fusion<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ins>(test, "double INS");
    if (test.failures() != 0)
    {
        return 1;
    }
    std::cout << "Magnetometer lifecycle: pass\n";
    return 0;
}
