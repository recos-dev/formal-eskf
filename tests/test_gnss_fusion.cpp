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
    GnssFusion::Conditions conditions{{true, true, true}, {true, true, true}, {true, true, true}};
    GnssFusion::Parameters parameters{100U, 25U, 50U};

    Status update(std::uint64_t now_us, GnssFusion::Sample const & sample = {})
    {
        return monitor.update(now_us, conditions, parameters, sample);
    }
};

bool same_group(GnssFusion::GroupState const & a, GnssFusion::GroupState const & b)
{
    return a.active == b.active && a.fusion_allowed == b.fusion_allowed && a.recovery_required == b.recovery_required &&
           a.stop_reason == b.stop_reason && a.start_time_us == b.start_time_us &&
           a.last_fuse_time_us == b.last_fuse_time_us && a.reset_allowed == b.reset_allowed &&
           a.last_reset_time_us == b.last_reset_time_us;
}

void test_start_conditions(TestContext & test)
{
    for (unsigned mask = 0U; mask < 64U; ++mask)
    {
        for (auto quality :
             {GnssQualityDecision::accepted, GnssQualityDecision::waiting, GnssQualityDecision::rejected})
        {
            for (std::uint64_t age : {0U, 24U, 25U, 99U, 100U})
            {
                Fixture fixture;
                unsigned bit = 0U;
                for (auto const & entry : groups)
                {
                    fixture.conditions.*
                        entry.conditions = {(mask & (1U << bit)) != 0U, (mask & (2U << bit)) != 0U, true};
                    bit += 2U;
                }
                test.expect(fixture.update(1000U, {1000U - age, quality}) == Status::success, lifecycle_profile,
                            "valid startup call");
                for (auto const & entry : groups)
                {
                    auto const condition = fixture.conditions.*entry.conditions;
                    auto const & state = fixture.monitor.state().*entry.state;
                    bool const fresh = age < (entry.group == Group::vertical_position ? 25U : 100U);
                    auto const reason = !condition.enabled                         ? StopReason::disabled
                                        : !condition.ready                         ? StopReason::not_ready
                                        : !fresh                                   ? StopReason::data_timeout
                                        : quality != GnssQualityDecision::accepted ? StopReason::waiting_for_quality
                                                                                   : StopReason::none;
                    bool const active = reason == StopReason::none;
                    test.expect(state.active == active && state.fusion_allowed == active &&
                                    state.stop_reason == reason && !state.recovery_required &&
                                    state.start_time_us == (active ? 1000U : 0U) && state.last_fuse_time_us == 0U,
                                lifecycle_profile, "independent conditions, strict freshness, no fictitious fusion");
                }
            }
        }
    }
    Fixture empty;
    test.expect(empty.update(1U) == Status::success && !empty.monitor.state().velocity.active, lifecycle_profile,
                "no sample cannot start");
    for (auto const & entry : groups)
    {
        Fixture fixture;
        (fixture.conditions.*entry.conditions).start_without_reset = false;
        test.expect(fixture.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success, lifecycle_profile,
                    "uninitialized start is a reset request, not an invalid call");
        auto const & state = fixture.monitor.state().*entry.state;
        test.expect(state.recovery_required && state.reset_allowed && !state.active && !state.fusion_allowed &&
                        state.stop_reason == StopReason::initialization_required,
                    lifecycle_profile, "good quality does not replace state initialization");
    }
}

void test_group_deadlines(TestContext & test)
{
    Fixture fixture;
    test.expect(fixture.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success, lifecycle_profile,
                "start horizontal sessions");
    for (auto const & entry : groups)
    {
        test.expect(fixture.monitor.record_fusion(entry.group, 1000U, FusionDecision::fused) == Status::success,
                    lifecycle_profile, "initial actual progress");
    }
    test.expect(fixture.update(1100U, {1100U, GnssQualityDecision::accepted}) == Status::success &&
                    fixture.monitor.state().velocity.fusion_allowed,
                lifecycle_profile, "horizontal reset deadline equality is allowed");
    test.expect(fixture.monitor.record_fusion(Group::velocity, 1100U, FusionDecision::fused) == Status::success,
                lifecycle_profile, "velocity alone makes progress");
    test.expect(fixture.update(1101U, {1101U, GnssQualityDecision::accepted}) == Status::success &&
                    fixture.monitor.state().horizontal_position.fusion_allowed,
                lifecycle_profile, "position rejection beyond one timeout does not block healthy velocity");
    test.expect(fixture.update(1200U, {1200U, GnssQualityDecision::accepted}) == Status::success &&
                    fixture.monitor.state().velocity.fusion_allowed &&
                    fixture.monitor.record_fusion(Group::velocity, 1200U, FusionDecision::fused) == Status::success,
                lifecycle_profile, "position double-timeout equality is allowed");
    test.expect(fixture.update(1201U, {1201U, GnssQualityDecision::accepted}) == Status::success &&
                    fixture.monitor.state().velocity.recovery_required &&
                    fixture.monitor.state().horizontal_position.recovery_required,
                lifecycle_profile, "stale enabled position beyond double timeout requests coupled recovery");

    Fixture both;
    test.expect(both.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                    both.update(1101U, {1101U, GnssQualityDecision::accepted}) == Status::success &&
                    both.monitor.state().velocity.recovery_required &&
                    both.monitor.state().horizontal_position.recovery_required,
                lifecycle_profile, "no successful horizontal group requests reset before overdue correction");
    test.expect(both.monitor.record_fusion(Group::velocity, 1101U, FusionDecision::fused) == Status::out_of_range,
                lifecycle_profile, "unsolicited success cannot clear reset request");

    Fixture velocity_only;
    velocity_only.conditions.horizontal_position.enabled = false;
    test.expect(
        velocity_only.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
            velocity_only.monitor.record_fusion(Group::velocity, 1000U, FusionDecision::fused) == Status::success &&
            velocity_only.update(1100U, {1100U, GnssQualityDecision::accepted}) == Status::success &&
            velocity_only.monitor.record_fusion(Group::velocity, 1100U, FusionDecision::fused) == Status::success &&
            velocity_only.update(1200U, {1200U, GnssQualityDecision::accepted}) == Status::success &&
            velocity_only.monitor.state().velocity.fusion_allowed,
        lifecycle_profile, "disabled horizontal position does not impose double timeout");

    for (auto decision : {FusionDecision::fused, FusionDecision::rejected, FusionDecision::failed})
    {
        Fixture height;
        test.expect(height.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                        height.monitor.record_fusion(Group::vertical_position, 1000U, FusionDecision::fused) ==
                            Status::success &&
                        height.update(1050U, {1050U, GnssQualityDecision::accepted}) == Status::success &&
                        height.monitor.record_fusion(Group::vertical_position, 1050U, FusionDecision::rejected) ==
                            Status::success &&
                        height.monitor.state().vertical_position.active,
                    lifecycle_profile, "height deadline equality allows rejection without reset");
        test.expect(height.update(1051U, {1051U, GnssQualityDecision::accepted}) == Status::success &&
                        height.monitor.state().vertical_position.fusion_allowed &&
                        height.monitor.record_fusion(Group::vertical_position, 1051U, decision) == Status::success,
                    lifecycle_profile, "height attempts current sample before its own timeout");
        auto const & state = height.monitor.state().vertical_position;
        test.expect(state.recovery_required == (decision != FusionDecision::fused) &&
                        state.reset_allowed == (decision == FusionDecision::rejected) &&
                        state.active == (decision != FusionDecision::failed) &&
                        state.last_fuse_time_us == (decision == FusionDecision::fused ? 1051U : 1000U),
                    lifecycle_profile, "overdue success wins; rejection/failure cannot refresh progress");
        test.expect(height.monitor.state().velocity.fusion_allowed, lifecycle_profile,
                    "height feedback does not consume another group's permission");
    }
}

void test_pauses_and_recovery(TestContext & test)
{
    for (auto const & entry : groups)
    {
        for (bool lose_readiness : {false, true})
        {
            Fixture paused;
            test.expect(paused.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success,
                        lifecycle_profile, "start before disable/readiness loss");
            paused.conditions.*entry.conditions = {lose_readiness, !lose_readiness, true};
            test.expect(paused.update(1001U) == Status::success && !(paused.monitor.state().*entry.state).active &&
                            !(paused.monitor.state().*entry.state).recovery_required,
                        lifecycle_profile, "disable/readiness loss stops without permanent recovery latch");
            paused.conditions.*entry.conditions = {true, true, true};
            test.expect(paused.update(1002U) == Status::success && !(paused.monitor.state().*entry.state).active &&
                            paused.update(1003U, {1003U, GnssQualityDecision::accepted}) == Status::success &&
                            (paused.monitor.state().*entry.state).fusion_allowed,
                        lifecycle_profile, "caller-established re-entry still requires a new quality-accepted sample");
        }
    }
    Fixture fixture;
    test.expect(fixture.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                    fixture.update(1024U) == Status::success && fixture.monitor.state().vertical_position.active &&
                    fixture.monitor.state().vertical_position.stop_reason == StopReason::none &&
                    fixture.update(1025U) == Status::success && !fixture.monitor.state().vertical_position.active &&
                    fixture.monitor.state().velocity.active &&
                    !fixture.monitor.state().vertical_position.recovery_required,
                lifecycle_profile, "height stops at its strict data boundary; horizontal continues");
    test.expect(fixture.update(1099U) == Status::success && fixture.monitor.state().velocity.active &&
                    fixture.update(1100U) == Status::success && !fixture.monitor.state().velocity.active &&
                    fixture.monitor.state().restart_quality && !fixture.monitor.state().velocity.recovery_required,
                lifecycle_profile, "horizontal data stop is ordinary and restarts quality health once");
    test.expect(fixture.update(1101U) == Status::success && !fixture.monitor.state().restart_quality, lifecycle_profile,
                "idle ticks do not repeatedly restart quality");
    test.expect(fixture.update(1102U, {1102U, GnssQualityDecision::waiting}) == Status::success &&
                    !fixture.monitor.state().velocity.active &&
                    fixture.update(1203U, {1203U, GnssQualityDecision::accepted}) == Status::success &&
                    fixture.monitor.state().velocity.fusion_allowed &&
                    fixture.monitor.state().velocity.start_time_us == 1203U,
                lifecycle_profile, "caller-authorized re-entry starts a new session, not a permanent timeout latch");

    // Re-entry can require an actual reset rather than caller-authorized startup.
    fixture.conditions.vertical_position.start_without_reset = false;
    test.expect(fixture.update(1228U) == Status::success &&
                    fixture.update(1229U, {1229U, GnssQualityDecision::accepted}) == Status::success &&
                    fixture.monitor.state().vertical_position.recovery_required,
                lifecycle_profile, "returning data requests initialization when safe re-entry is not established");
    fixture.conditions.vertical_position.start_without_reset = true;
    test.expect(fixture.update(1230U, {1230U, GnssQualityDecision::accepted}) == Status::success &&
                    fixture.monitor.state().vertical_position.recovery_required,
                lifecycle_profile, "condition toggles cannot acknowledge pending reset");
    test.expect(fixture.monitor.restart(Group::vertical_position) == Status::success &&
                    fixture.monitor.record_fusion(Group::vertical_position, 1230U, FusionDecision::fused) ==
                        Status::out_of_range &&
                    fixture.update(1231U) == Status::success && !fixture.monitor.state().vertical_position.active &&
                    fixture.update(1232U, {1232U, GnssQualityDecision::accepted}) == Status::success &&
                    fixture.monitor.state().vertical_position.fusion_allowed,
                lifecycle_profile, "explicit repair acknowledgement still requires a new accepted sample");

    Fixture height;
    height.conditions.horizontal_position.enabled = false;
    height.conditions.velocity.enabled = false;
    test.expect(height.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                    height.monitor.record_fusion(Group::vertical_position, 1000U, FusionDecision::fused) ==
                        Status::success &&
                    height.update(1025U) == Status::success &&
                    height.update(1300U, {1300U, GnssQualityDecision::accepted}) == Status::success &&
                    height.monitor.record_fusion(Group::vertical_position, 1300U, FusionDecision::rejected) ==
                        Status::success &&
                    height.monitor.state().vertical_position.active &&
                    height.monitor.state().vertical_position.last_fuse_time_us == 1000U &&
                    height.monitor.state().vertical_position.start_time_us == 1300U,
                lifecycle_profile, "authorized new session renews its deadline without falsifying last success");
}

void test_persistent_rejection(TestContext & test)
{
    Fixture quality;
    test.expect(quality.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                    quality.update(1100U, {1100U, GnssQualityDecision::rejected}) == Status::success &&
                    quality.monitor.state().velocity.active && !quality.monitor.state().velocity.fusion_allowed &&
                    quality.update(1101U, {1101U, GnssQualityDecision::rejected}) == Status::success &&
                    !quality.monitor.state().velocity.active && quality.monitor.state().restart_quality &&
                    quality.monitor.state().velocity.stop_reason == StopReason::poor_quality &&
                    !quality.monitor.state().velocity.recovery_required &&
                    !quality.monitor.state().vertical_position.active,
                lifecycle_profile, "poor quality stops all groups after strict timeout, without a reset latch");

    Fixture waiting;
    test.expect(waiting.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                    waiting.update(1101U, {1101U, GnssQualityDecision::waiting}) == Status::success &&
                    waiting.monitor.state().velocity.active && !waiting.monitor.state().velocity.fusion_allowed,
                lifecycle_profile, "currently passing checks refresh quality even during recovery wait");
    test.expect(waiting.update(1201U, {1201U, GnssQualityDecision::invalid_call, false}) == Status::success &&
                    waiting.monitor.state().velocity.active &&
                    waiting.update(1202U, {1202U, GnssQualityDecision::invalid_call, false}) == Status::success &&
                    waiting.monitor.state().velocity.stop_reason == StopReason::poor_quality,
                lifecycle_profile, "unavailable quality refreshes receipt, never quality progress");

    for (auto const & entry : groups)
    {
        Fixture fixture;
        test.expect(fixture.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                        fixture.monitor.record_fusion(entry.group, 1000U, FusionDecision::failed) == Status::success,
                    lifecycle_profile, "record numerical failure");
        auto const & state = fixture.monitor.state().*entry.state;
        test.expect(state.recovery_required && state.stop_reason == StopReason::fusion_failed &&
                        state.last_fuse_time_us == 0U,
                    lifecycle_profile, "numerical failure requests repair immediately");
        for (auto const & other : groups)
        {
            if (other.group != entry.group)
            {
                test.expect((fixture.monitor.state().*other.state).fusion_allowed, lifecycle_profile,
                            "one failed group does not erase another pending decision");
            }
        }
    }
}

void test_invalid_calls(TestContext & test)
{
    Fixture baseline;
    test.expect(baseline.update(1000U, {998U, GnssQualityDecision::accepted}) == Status::success &&
                    baseline.monitor.record_fusion(Group::velocity, 998U, FusionDecision::fused) == Status::success,
                lifecycle_profile, "seed history");
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
                        "invalid call revokes permissions without rewriting history");
        }
        test.expect(fixture.update(1001U, {1000U, GnssQualityDecision::accepted}) == Status::success, lifecycle_profile,
                    "invalid call consumed no time/configuration");
    };
    for (auto now : {0U, 999U, 1000U})
    {
        check_failure(now, {}, baseline.parameters, Status::out_of_range);
    }
    for (auto time : {997U, 998U, 1002U})
    {
        check_failure(1001U, {time, GnssQualityDecision::accepted}, baseline.parameters, Status::out_of_range);
    }
    for (auto decision : {GnssQualityDecision::invalid_call, static_cast<GnssQualityDecision>(99)})
    {
        check_failure(1001U, {1001U, decision}, baseline.parameters, Status::domain_error);
    }
    auto const maximum = std::numeric_limits<std::uint64_t>::max();
    for (auto parameters : {GnssFusion::Parameters{0U, 25U, 50U}, GnssFusion::Parameters{100U, 0U, 50U},
                            GnssFusion::Parameters{100U, 25U, 0U}, GnssFusion::Parameters{101U, 25U, 50U},
                            GnssFusion::Parameters{100U, 26U, 50U}, GnssFusion::Parameters{100U, 25U, 51U},
                            GnssFusion::Parameters{maximum, 25U, 50U}})
    {
        check_failure(1001U, {}, parameters, Status::domain_error);
    }
    auto const before = baseline.monitor.state();
    test.expect(baseline.monitor.record_fusion(static_cast<Group>(99), 998U, FusionDecision::fused) ==
                        Status::domain_error &&
                    baseline.monitor.restart(static_cast<Group>(99)) == Status::domain_error &&
                    baseline.monitor.record_fusion(Group::horizontal_position, 998U, static_cast<FusionDecision>(99)) ==
                        Status::domain_error &&
                    baseline.monitor.record_fusion(Group::horizontal_position, 997U, FusionDecision::fused) ==
                        Status::out_of_range,
                lifecycle_profile, "invalid feedback and restart are rejected");
    for (auto const & entry : groups)
    {
        test.expect(same_group(baseline.monitor.state().*entry.state, before.*entry.state), lifecycle_profile,
                    "invalid feedback preserves valid permission");
    }
    test.expect(
        baseline.update(1000U, {999U, GnssQualityDecision::accepted}) == Status::success &&
            baseline.monitor.record_fusion(Group::velocity, 998U, FusionDecision::fused) == Status::out_of_range &&
            baseline.monitor.record_fusion(Group::velocity, 999U, FusionDecision::fused) == Status::success &&
            baseline.monitor.record_fusion(Group::velocity, 999U, FusionDecision::fused) == Status::out_of_range,
        lifecycle_profile, "distinct samples may share a tick, but each feedback belongs to one sample");
    Fixture limits;
    test.expect(limits.update(maximum - 100U, {maximum - 100U, GnssQualityDecision::accepted}) == Status::success &&
                    limits.update(maximum, {maximum, GnssQualityDecision::accepted}) == Status::success &&
                    limits.monitor.state().velocity.fusion_allowed && limits.update(1U) == Status::out_of_range,
                lifecycle_profile, "elapsed subtraction remains safe through UINT64_MAX; wrapped clock is rejected");
    limits.monitor.reset();
    test.expect(limits.update(1U, {1U, GnssQualityDecision::accepted}) == Status::success, lifecycle_profile,
                "explicit full reset permits a new timeline");
}

void test_reset_feedback(TestContext & test)
{
    for (auto const & entry : groups)
    {
        Fixture fixture;
        (fixture.conditions.*entry.conditions).start_without_reset = false;
        test.expect(fixture.update(1000U, {999U, GnssQualityDecision::accepted}) == Status::success, lifecycle_profile,
                    "prepare reset on aligned current sample");
        auto const before = fixture.monitor.state().*entry.state;
        test.expect(before.reset_allowed && !before.active && !before.fusion_allowed &&
                        before.last_reset_time_us == 0U &&
                        fixture.monitor.record_reset(entry.group, 998U, true) == Status::out_of_range &&
                        same_group(before, fixture.monitor.state().*entry.state),
                    lifecycle_profile, "reset requires matching permission; request alone is not progress");
        test.expect(fixture.monitor.record_reset(entry.group, 999U, true) == Status::success, lifecycle_profile,
                    "acknowledge completed state and covariance reset");
        auto const & state = fixture.monitor.state().*entry.state;
        test.expect(state.active && !state.recovery_required && !state.reset_allowed && !state.fusion_allowed &&
                        state.last_reset_time_us == 1000U && state.last_fuse_time_us == 0U &&
                        state.start_time_us == 1000U && !fixture.monitor.state().restart_quality &&
                        fixture.monitor.record_reset(entry.group, 999U, true) == Status::out_of_range &&
                        fixture.monitor.record_fusion(entry.group, 999U, FusionDecision::fused) == Status::out_of_range,
                    lifecycle_profile, "reset advances its own clock once, with no same-sample correction");
        test.expect(fixture.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                        (fixture.monitor.state().*entry.state).fusion_allowed,
                    lifecycle_profile, "distinct receiver sample at the same tick may correct");

        Fixture failed;
        (failed.conditions.*entry.conditions).start_without_reset = false;
        test.expect(failed.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                        failed.monitor.record_reset(entry.group, 1000U, false) == Status::success &&
                        failed.update(1001U, {1001U, GnssQualityDecision::accepted}) == Status::success,
                    lifecycle_profile, "record failed reset and later good sample");
        auto const & failure = failed.monitor.state().*entry.state;
        test.expect(!failure.active && !failure.reset_allowed && failure.recovery_required &&
                        failure.stop_reason == StopReason::reset_failed && failure.last_reset_time_us == 0U &&
                        failure.last_fuse_time_us == 0U && failure.start_time_us == 0U,
                    lifecycle_profile, "failure neither advances progress nor retries automatically");

        for (unsigned inhibition = 0U; inhibition < 6U; ++inhibition)
        {
            Fixture pending;
            (pending.conditions.*entry.conditions).start_without_reset = false;
            test.expect(pending.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success,
                        lifecycle_profile, "pending reset permission");
            auto sample = GnssFusion::Sample{1001U, GnssQualityDecision::accepted};
            if (inhibition == 0U)
            {
                sample = {};
            }
            else if (inhibition == 1U)
            {
                sample.quality = GnssQualityDecision::rejected;
            }
            else if (inhibition == 2U)
            {
                sample.quality_available = false;
            }
            else if (inhibition == 3U)
            {
                (pending.conditions.*entry.conditions).enabled = false;
            }
            else if (inhibition == 4U)
            {
                (pending.conditions.*entry.conditions).ready = false;
            }
            else
            {
                sample.time_us = 1002U; // Invalid future sample also revokes permission.
            }
            auto const expected = inhibition == 5U ? Status::out_of_range : Status::success;
            test.expect(pending.update(1001U, sample) == expected &&
                            !(pending.monitor.state().*entry.state).reset_allowed &&
                            pending.monitor.record_reset(entry.group, 1000U, true) == Status::out_of_range,
                        lifecycle_profile, "stale, unqualified, disabled and invalid calls cannot reset");
            test.expect(pending.update(1003U, {1003U, GnssQualityDecision::accepted}) == Status::success,
                        lifecycle_profile, "advance after revoked permission");
        }
    }
    Fixture timeout;
    test.expect(timeout.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                    timeout.update(1101U, {1101U, GnssQualityDecision::accepted}) == Status::success &&
                    timeout.monitor.state().velocity.reset_allowed && timeout.monitor.state().velocity.active &&
                    timeout.monitor.state().horizontal_position.reset_allowed &&
                    !timeout.monitor.state().restart_quality,
                lifecycle_profile, "coupled timeout requests resets without stopping quality/session");
    test.expect(timeout.monitor.record_reset(Group::velocity, 1101U, true) == Status::success &&
                    timeout.monitor.state().horizontal_position.reset_allowed &&
                    timeout.monitor.record_reset(Group::horizontal_position, 1101U, true) == Status::success &&
                    timeout.update(1201U, {1201U, GnssQualityDecision::accepted}) == Status::success &&
                    timeout.monitor.state().velocity.fusion_allowed &&
                    timeout.monitor.state().horizontal_position.fusion_allowed,
                lifecycle_profile, "first reset cannot cancel second; reset time renews strict deadline");
    test.expect(timeout.update(1202U, {1202U, GnssQualityDecision::accepted}) == Status::success &&
                    timeout.monitor.state().velocity.reset_allowed &&
                    timeout.monitor.state().velocity.start_time_us == 1000U &&
                    timeout.monitor.state().velocity.last_fuse_time_us == 0U,
                lifecycle_profile, "new timeout uses reset progress without rewriting correction/session history");
    test.expect(timeout.monitor.record_reset(Group::vertical_position, 1202U, true) == Status::out_of_range &&
                    timeout.monitor.record_reset(static_cast<Group>(99), 1202U, true) == Status::domain_error,
                lifecycle_profile, "unsolicited height and invalid group feedback are rejected");

    Fixture numerical;
    test.expect(numerical.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                    numerical.monitor.record_fusion(Group::velocity, 1000U, FusionDecision::failed) ==
                        Status::success &&
                    numerical.update(1001U, {1001U, GnssQualityDecision::accepted}) == Status::success &&
                    !numerical.monitor.state().velocity.reset_allowed,
                lifecycle_profile, "numerical correction failure is not automatic reset authorization");

    Fixture reentry;
    reentry.conditions.horizontal_position.start_without_reset = false;
    reentry.conditions.velocity.start_without_reset = false;
    test.expect(reentry.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                    reentry.monitor.record_reset(Group::velocity, 1000U, true) == Status::success &&
                    reentry.monitor.record_reset(Group::horizontal_position, 1000U, true) == Status::success,
                lifecycle_profile, "start reset-backed sessions");
    reentry.conditions.horizontal_position.enabled = false;
    reentry.conditions.velocity.enabled = false;
    test.expect(reentry.update(1001U) == Status::success && reentry.monitor.state().restart_quality, lifecycle_profile,
                "ordinary stop still restarts quality after reset-backed startup");
    reentry.conditions.horizontal_position.enabled = true;
    reentry.conditions.velocity.enabled = true;
    test.expect(reentry.update(1002U, {1002U, GnssQualityDecision::waiting}) == Status::success &&
                    !reentry.monitor.state().velocity.reset_allowed &&
                    reentry.update(1003U, {1003U, GnssQualityDecision::accepted}) == Status::success &&
                    reentry.monitor.state().velocity.reset_allowed &&
                    reentry.monitor.state().horizontal_position.reset_allowed &&
                    reentry.monitor.record_reset(Group::velocity, 1003U, true) == Status::success &&
                    reentry.monitor.record_reset(Group::horizontal_position, 1003U, true) == Status::success &&
                    reentry.monitor.state().velocity.start_time_us == 1003U &&
                    reentry.monitor.state().velocity.last_reset_time_us == 1003U &&
                    !reentry.monitor.state().restart_quality,
                lifecycle_profile, "re-entry waits for quality and completes new resets without a second quality stop");
}

void test_height_reset(TestContext & test)
{
    for (auto const start : {std::uint64_t{1000}, std::numeric_limits<std::uint64_t>::max() - 200U})
    {
        for (bool success : {false, true})
        {
            Fixture fixture;
            test.expect(
                fixture.update(start, {start, GnssQualityDecision::accepted}) == Status::success &&
                    fixture.monitor.record_fusion(Group::vertical_position, start, FusionDecision::fused) ==
                        Status::success &&
                    fixture.update(start + 51U, {start + 51U, GnssQualityDecision::accepted}) == Status::success &&
                    fixture.monitor.record_reset(Group::vertical_position, start + 51U, true) == Status::out_of_range,
                lifecycle_profile, "overdue height must attempt correction before requesting reset");
            auto const horizontal = fixture.monitor.state().horizontal_position;
            auto const velocity = fixture.monitor.state().velocity;
            test.expect(
                fixture.monitor.record_fusion(Group::vertical_position, start + 51U, FusionDecision::rejected) ==
                        Status::success &&
                    fixture.monitor.state().vertical_position.reset_allowed &&
                    fixture.monitor.record_reset(Group::vertical_position, start + 51U, success) == Status::success,
                lifecycle_profile, "rejected overdue height grants one same-sample reset");
            auto const & height = fixture.monitor.state().vertical_position;
            test.expect(height.active == success && height.recovery_required == !success && !height.reset_allowed &&
                            !height.fusion_allowed &&
                            height.stop_reason == (success ? StopReason::none : StopReason::reset_failed) &&
                            height.last_reset_time_us == (success ? start + 51U : 0U) &&
                            height.last_fuse_time_us == start && height.start_time_us == start &&
                            !fixture.monitor.state().restart_quality &&
                            same_group(horizontal, fixture.monitor.state().horizontal_position) &&
                            same_group(velocity, fixture.monitor.state().velocity),
                        lifecycle_profile, "height reset preserves correction, session and horizontal history");
            test.expect(
                fixture.monitor.record_reset(Group::vertical_position, start + 51U, true) == Status::out_of_range &&
                    fixture.monitor.record_fusion(Group::vertical_position, start + 51U, FusionDecision::fused) ==
                        Status::out_of_range,
                lifecycle_profile, "neither duplicate reset nor correction after reset is permitted");
            test.expect(fixture.update(start + 101U, {start + 101U, GnssQualityDecision::accepted}) == Status::success,
                        lifecycle_profile, "advance to reset-based height deadline");
            if (success)
            {
                test.expect(fixture.monitor.record_fusion(Group::vertical_position, start + 101U,
                                                          FusionDecision::rejected) == Status::success &&
                                !height.reset_allowed && !height.recovery_required &&
                                fixture.update(start + 102U, {start + 102U, GnssQualityDecision::accepted}) ==
                                    Status::success &&
                                fixture.monitor.record_fusion(Group::vertical_position, start + 102U,
                                                              FusionDecision::rejected) == Status::success &&
                                height.reset_allowed && fixture.update(start + 103U) == Status::success &&
                                !height.reset_allowed &&
                                fixture.monitor.record_reset(Group::vertical_position, start + 102U, true) ==
                                    Status::out_of_range,
                            lifecycle_profile, "reset renews strict deadline; next update revokes unconsumed reset");
            }
            else
            {
                test.expect(!height.reset_allowed && !height.fusion_allowed && height.recovery_required,
                            lifecycle_profile, "failed timeout reset cannot retry on good data");
            }
        }
    }
    Fixture reentry;
    reentry.conditions.vertical_position.start_without_reset = false;
    test.expect(reentry.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                    reentry.monitor.record_reset(Group::vertical_position, 1000U, true) == Status::success &&
                    reentry.update(1025U) == Status::success && !reentry.monitor.state().vertical_position.active &&
                    reentry.update(1026U, {1026U, GnssQualityDecision::waiting}) == Status::success &&
                    !reentry.monitor.state().vertical_position.reset_allowed &&
                    reentry.update(1027U, {1027U, GnssQualityDecision::accepted}) == Status::success &&
                    reentry.monitor.record_reset(Group::vertical_position, 1027U, true) == Status::success &&
                    reentry.monitor.state().vertical_position.start_time_us == 1027U &&
                    reentry.monitor.state().vertical_position.last_fuse_time_us == 0U &&
                    !reentry.monitor.state().restart_quality,
                lifecycle_profile, "height-only data stop restarts through a qualified reset, not a quality restart");
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
                    !fixture.monitor.state().horizontal_position.active &&
                    formal_eskf::linalg::max_abs(state.p_n - fused_position) == Scalar{0} &&
                    formal_eskf::linalg::max_abs(covariance - fused_covariance) == Scalar{0},
                profile, "no-data tick stops without touching estimate or covariance");
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
    test_reset_feedback(test);
    test_height_reset(test);
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
