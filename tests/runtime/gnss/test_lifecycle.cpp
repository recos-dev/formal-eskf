/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/gnss_lifecycle.hpp>
#include <formal_eskf/runtime/navigation.hpp>
#include <formal_eskf/runtime/vertical_acceleration_health.hpp>

#include <array>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string_view>

#include "test_backend.hpp"
#include "test_cases.hpp"
#include "test_support.hpp"

namespace
{

using formal_eskf::Status;
using formal_eskf::runtime::FusionDecision;
using formal_eskf::runtime::GnssFusion;
using formal_eskf::runtime::GnssQualityDecision;
using TestContext = formal_eskf::test::Context;
using Group = GnssFusion::Group;
using Action = GnssFusion::Action;
using StopReason = GnssFusion::StopReason;

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
    return a.active == b.active && a.action == b.action && a.recovery_required == b.recovery_required &&
           a.stop_reason == b.stop_reason && a.start_time_us == b.start_time_us &&
           a.last_fuse_time_us == b.last_fuse_time_us && a.last_reset_time_us == b.last_reset_time_us;
}

bool same_state(GnssFusion::State const & a, GnssFusion::State const & b)
{
    return same_group(a.horizontal_position, b.horizontal_position) &&
           same_group(a.vertical_position, b.vertical_position) && same_group(a.velocity, b.velocity) &&
           a.vertical_velocity_reset_reason == b.vertical_velocity_reset_reason &&
           a.last_vertical_velocity_reset_time_us == b.last_vertical_velocity_reset_time_us &&
           a.restart_quality == b.restart_quality;
}
void startup(TestContext & test, std::string_view profile)
{
    for (unsigned mask = 0; mask < 512U; ++mask)
    {
        for (auto quality : {GnssQualityDecision::accepted, GnssQualityDecision::waiting, GnssQualityDecision::rejected,
                             GnssQualityDecision::invalid_call})
        {
            Fixture f;
            unsigned bit = 0;
            for (auto const & g : groups)
            {
                f.conditions.*
                    g.conditions = {(mask & (1U << bit)) != 0U, (mask & (2U << bit)) != 0U, (mask & (4U << bit)) != 0U};
                bit += 3U;
            }
            bool const available = quality != GnssQualityDecision::invalid_call;
            test.expect(f.update(1000U, {1000U, quality, available}) == Status::success, profile,
                        "startup condition cube");
            for (auto const & g : groups)
            {
                auto const c = f.conditions.*g.conditions;
                auto const & state = f.monitor.state().*g.state;
                bool const eligible = c.enabled && c.ready && available && quality == GnssQualityDecision::accepted;
                auto const action = !eligible ? Action::none : c.start_without_reset ? Action::fuse : Action::reset;
                auto const reason = !c.enabled               ? StopReason::disabled
                                    : !c.ready               ? StopReason::not_ready
                                    : !eligible              ? StopReason::waiting_for_quality
                                    : !c.start_without_reset ? StopReason::initialization_required
                                                             : StopReason::none;
                test.expect(state.action == action && state.active == (action == Action::fuse) &&
                                state.recovery_required == (action == Action::reset) && state.stop_reason == reason &&
                                state.start_time_us == (state.active ? 1000U : 0U) && state.last_fuse_time_us == 0U &&
                                state.last_reset_time_us == 0U,
                            profile, "eligibility, initialization and permission are separate from actual progress");
            }
        }
    }
    for (std::uint64_t age : {24U, 25U, 26U, 99U, 100U, 101U})
    {
        Fixture f;
        test.expect(f.update(1000U, {1000U - age, GnssQualityDecision::accepted}) == Status::success, profile,
                    "accepted incoming sample can already be stale at fusion horizon");
        for (auto const & g : groups)
        {
            bool const fresh = age < (g.group == Group::vertical_position ? 25U : 100U);
            auto const & state = f.monitor.state().*g.state;
            test.expect(state.active == fresh && (state.action == Action::fuse) == fresh &&
                            state.stop_reason == (fresh ? StopReason::none : StopReason::data_timeout),
                        profile, "startup freshness uses the correct group's strict receipt boundary");
        }
    }
    Fixture empty;
    test.expect(same_state(empty.monitor.state(), GnssFusion::State{}) && empty.update(1U) == Status::success &&
                    !empty.monitor.state().velocity.active,
                profile, "construction and absent sample cannot start a session");
}

void feedback(TestContext & test, std::string_view profile)
{
    for (auto const & g : groups)
    {
        for (auto decision : {FusionDecision::fused, FusionDecision::rejected, FusionDecision::failed})
        {
            Fixture f;
            test.expect(f.update(1000U, {999U, GnssQualityDecision::accepted}) == Status::success, profile,
                        "grant feedback permission");
            auto const before = f.monitor.state();
            for (auto time : {0U, 998U, 1000U})
            {
                test.expect(f.monitor.record_fusion(g.group, time, decision) == Status::out_of_range &&
                                same_state(before, f.monitor.state()),
                            profile, "wrong sample feedback preserves every metadata field and permission");
            }
            test.expect(f.monitor.record_fusion(static_cast<Group>(99), 999U, decision) == Status::domain_error &&
                            same_state(before, f.monitor.state()) &&
                            f.monitor.record_fusion(g.group, 999U, static_cast<FusionDecision>(99)) ==
                                Status::domain_error &&
                            same_state(before, f.monitor.state()),
                        profile, "invalid group or enum does not consume permission");
            test.expect(f.monitor.record_fusion(g.group, 999U, decision) == Status::success, profile,
                        "one matching actual outcome");
            auto const after = f.monitor.state();
            auto const & actual = after.*g.state;
            test.expect(actual.action == Action::none &&
                            actual.last_fuse_time_us == (decision == FusionDecision::fused ? 1000U : 0U) &&
                            actual.recovery_required == (decision == FusionDecision::failed) &&
                            actual.active == (decision != FusionDecision::failed),
                        profile, "only fused advances progress; failure latches repair");
            for (auto const & other : groups)
            {
                if (other.group != g.group)
                {
                    test.expect(same_group(after.*other.state, before.*other.state), profile,
                                "other group's pending permission survives feedback");
                }
            }
            test.expect(f.monitor.record_fusion(g.group, 999U, decision) == Status::out_of_range &&
                            same_state(after, f.monitor.state()),
                        profile, "feedback is single use including rejection and failure");
        }
    }
    Fixture f;
    test.expect(f.update(1000U, {998U, GnssQualityDecision::accepted}) == Status::success &&
                    f.update(1000U, {999U, GnssQualityDecision::accepted}) == Status::success &&
                    f.monitor.record_fusion(Group::velocity, 998U, FusionDecision::fused) == Status::out_of_range &&
                    f.monitor.record_fusion(Group::velocity, 999U, FusionDecision::fused) == Status::success &&
                    f.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                    f.monitor.record_fusion(Group::velocity, 1000U, FusionDecision::fused) == Status::success,
                profile, "new update revokes unconsumed old permission; distinct samples can share one horizon");
}

void expiry(TestContext & test, std::string_view profile)
{
    Fixture fixture;
    test.expect(fixture.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                    fixture.update(1024U) == Status::success && fixture.monitor.state().vertical_position.active &&
                    fixture.monitor.state().vertical_position.stop_reason == StopReason::none &&
                    fixture.update(1025U) == Status::success && !fixture.monitor.state().vertical_position.active &&
                    fixture.monitor.state().velocity.active &&
                    !fixture.monitor.state().vertical_position.recovery_required,
                profile, "height stops at its strict data boundary; horizontal continues");
    test.expect(fixture.update(1099U) == Status::success && fixture.monitor.state().velocity.active &&
                    fixture.update(1100U) == Status::success && !fixture.monitor.state().velocity.active &&
                    fixture.monitor.state().restart_quality && !fixture.monitor.state().velocity.recovery_required,
                profile, "horizontal data stop is ordinary and restarts quality health once");
    test.expect(fixture.update(1101U) == Status::success && !fixture.monitor.state().restart_quality, profile,
                "idle ticks do not repeatedly restart quality");
    test.expect(fixture.update(1102U, {1102U, GnssQualityDecision::waiting}) == Status::success &&
                    !fixture.monitor.state().velocity.active &&
                    fixture.update(1203U, {1203U, GnssQualityDecision::accepted}) == Status::success &&
                    (fixture.monitor.state().velocity.action == Action::fuse) &&
                    fixture.monitor.state().velocity.start_time_us == 1203U,
                profile, "caller-authorized re-entry starts a new session, not a permanent timeout latch");

    Fixture quality;
    test.expect(
        quality.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
            quality.update(1100U, {1100U, GnssQualityDecision::rejected}) == Status::success &&
            quality.monitor.state().velocity.active && (quality.monitor.state().velocity.action != Action::fuse) &&
            quality.update(1101U, {1101U, GnssQualityDecision::rejected}) == Status::success &&
            !quality.monitor.state().velocity.active && quality.monitor.state().restart_quality &&
            quality.monitor.state().velocity.stop_reason == StopReason::poor_quality &&
            !quality.monitor.state().velocity.recovery_required && !quality.monitor.state().vertical_position.active,
        profile, "poor quality stops all groups after strict timeout, without a reset latch");

    Fixture waiting;
    test.expect(waiting.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                    waiting.update(1101U, {1101U, GnssQualityDecision::waiting}) == Status::success &&
                    waiting.monitor.state().velocity.active &&
                    (waiting.monitor.state().velocity.action != Action::fuse),
                profile, "currently passing checks refresh quality even during recovery wait");
    test.expect(waiting.update(1201U, {1201U, GnssQualityDecision::invalid_call, false}) == Status::success &&
                    waiting.monitor.state().velocity.active &&
                    waiting.update(1202U, {1202U, GnssQualityDecision::invalid_call, false}) == Status::success &&
                    waiting.monitor.state().velocity.stop_reason == StopReason::poor_quality,
                profile, "unavailable quality refreshes receipt, never quality progress");
}

void horizontal_progress(TestContext & test, std::string_view profile)
{
    Fixture fixture;
    test.expect(fixture.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success, profile,
                "start horizontal sessions");
    for (auto const & entry : groups)
    {
        test.expect(fixture.monitor.record_fusion(entry.group, 1000U, FusionDecision::fused) == Status::success,
                    profile, "initial actual progress");
    }
    test.expect(fixture.update(1100U, {1100U, GnssQualityDecision::accepted}) == Status::success &&
                    (fixture.monitor.state().velocity.action == Action::fuse),
                profile, "horizontal reset deadline equality is allowed");
    test.expect(fixture.monitor.record_fusion(Group::velocity, 1100U, FusionDecision::fused) == Status::success,
                profile, "velocity alone makes progress");
    test.expect(fixture.update(1101U, {1101U, GnssQualityDecision::accepted}) == Status::success &&
                    (fixture.monitor.state().horizontal_position.action == Action::fuse),
                profile, "position rejection beyond one timeout does not block healthy velocity");
    test.expect(fixture.update(1200U, {1200U, GnssQualityDecision::accepted}) == Status::success &&
                    (fixture.monitor.state().velocity.action == Action::fuse) &&
                    fixture.monitor.record_fusion(Group::velocity, 1200U, FusionDecision::fused) == Status::success,
                profile, "position double-timeout equality is allowed");
    test.expect(fixture.update(1201U, {1201U, GnssQualityDecision::accepted}) == Status::success &&
                    fixture.monitor.state().velocity.recovery_required &&
                    fixture.monitor.state().horizontal_position.recovery_required,
                profile, "stale enabled position beyond double timeout requests coupled recovery");

    Fixture both;
    test.expect(both.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                    both.update(1101U, {1101U, GnssQualityDecision::accepted}) == Status::success &&
                    both.monitor.state().velocity.recovery_required &&
                    both.monitor.state().horizontal_position.recovery_required,
                profile, "no successful horizontal group requests reset before overdue correction");
    test.expect(both.monitor.record_fusion(Group::velocity, 1101U, FusionDecision::fused) == Status::out_of_range,
                profile, "unsolicited success cannot clear reset request");

    Fixture velocity_only;
    velocity_only.conditions.horizontal_position.enabled = false;
    test.expect(
        velocity_only.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
            velocity_only.monitor.record_fusion(Group::velocity, 1000U, FusionDecision::fused) == Status::success &&
            velocity_only.update(1100U, {1100U, GnssQualityDecision::accepted}) == Status::success &&
            velocity_only.monitor.record_fusion(Group::velocity, 1100U, FusionDecision::fused) == Status::success &&
            velocity_only.update(1200U, {1200U, GnssQualityDecision::accepted}) == Status::success &&
            (velocity_only.monitor.state().velocity.action == Action::fuse),
        profile, "disabled horizontal position does not impose double timeout");
}

void height_progress(TestContext & test, std::string_view profile)
{
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
                    profile, "height deadline equality allows rejection without reset");
        test.expect(height.update(1051U, {1051U, GnssQualityDecision::accepted}) == Status::success &&
                        (height.monitor.state().vertical_position.action == Action::fuse) &&
                        height.monitor.record_fusion(Group::vertical_position, 1051U, decision) == Status::success,
                    profile, "height attempts current sample before its own timeout");
        auto const & state = height.monitor.state().vertical_position;
        test.expect(state.recovery_required == (decision != FusionDecision::fused) &&
                        (state.action == Action::reset) == (decision == FusionDecision::rejected) &&
                        state.active == (decision != FusionDecision::failed) &&
                        state.last_fuse_time_us == (decision == FusionDecision::fused ? 1051U : 1000U),
                    profile, "overdue success wins; rejection/failure cannot refresh progress");
        test.expect((height.monitor.state().velocity.action == Action::fuse), profile,
                    "height feedback does not consume another group's permission");
    }
}

void reset_feedback(TestContext & test, std::string_view profile)
{
    enum class Inhibition
    {
        no_sample,
        rejected_quality,
        unavailable_quality,
        disabled_group,
        group_not_ready,
        future_sample
    };
    constexpr std::array inhibitions{Inhibition::no_sample,           Inhibition::rejected_quality,
                                     Inhibition::unavailable_quality, Inhibition::disabled_group,
                                     Inhibition::group_not_ready,     Inhibition::future_sample};
    for (auto const & entry : groups)
    {
        Fixture fixture;
        (fixture.conditions.*entry.conditions).start_without_reset = false;
        test.expect(fixture.update(1000U, {999U, GnssQualityDecision::accepted}) == Status::success, profile,
                    "prepare reset on aligned current sample");
        auto const before = fixture.monitor.state().*entry.state;
        test.expect((before.action == Action::reset) && !before.active && before.last_reset_time_us == 0U &&
                        fixture.monitor.record_reset(entry.group, 998U, true) == Status::out_of_range &&
                        same_group(before, fixture.monitor.state().*entry.state),
                    profile, "reset requires matching permission; request alone is not progress");
        test.expect(fixture.monitor.record_reset(entry.group, 999U, true) == Status::success, profile,
                    "acknowledge completed state and covariance reset");
        auto const & state = fixture.monitor.state().*entry.state;
        test.expect(state.active && !state.recovery_required && state.action == Action::none &&
                        state.last_reset_time_us == 1000U && state.last_fuse_time_us == 0U &&
                        state.start_time_us == 1000U && !fixture.monitor.state().restart_quality &&
                        fixture.monitor.record_reset(entry.group, 999U, true) == Status::out_of_range &&
                        fixture.monitor.record_fusion(entry.group, 999U, FusionDecision::fused) == Status::out_of_range,
                    profile, "reset advances its own clock once, with no same-sample correction");
        test.expect(fixture.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                        ((fixture.monitor.state().*entry.state).action == Action::fuse),
                    profile, "distinct receiver sample at the same tick may correct");

        Fixture failed;
        (failed.conditions.*entry.conditions).start_without_reset = false;
        test.expect(failed.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                        failed.monitor.record_reset(entry.group, 1000U, false) == Status::success &&
                        failed.update(1001U, {1001U, GnssQualityDecision::accepted}) == Status::success,
                    profile, "record failed reset and later good sample");
        auto const & failure = failed.monitor.state().*entry.state;
        test.expect(!failure.active && (failure.action != Action::reset) && failure.recovery_required &&
                        failure.stop_reason == StopReason::reset_failed && failure.last_reset_time_us == 0U &&
                        failure.last_fuse_time_us == 0U && failure.start_time_us == 0U,
                    profile, "failure neither advances progress nor retries automatically");

        for (auto inhibition : inhibitions)
        {
            Fixture pending;
            (pending.conditions.*entry.conditions).start_without_reset = false;
            test.expect(pending.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success, profile,
                        "pending reset permission");
            auto sample = GnssFusion::Sample{1001U, GnssQualityDecision::accepted};
            switch (inhibition)
            {
            case Inhibition::no_sample:
                sample = {};
                break;
            case Inhibition::rejected_quality:
                sample.quality = GnssQualityDecision::rejected;
                break;
            case Inhibition::unavailable_quality:
                sample.quality_available = false;
                break;
            case Inhibition::disabled_group:
                (pending.conditions.*entry.conditions).enabled = false;
                break;
            case Inhibition::group_not_ready:
                (pending.conditions.*entry.conditions).ready = false;
                break;
            case Inhibition::future_sample:
                sample.time_us = 1002U;
                break;
            }
            auto const expected = inhibition == Inhibition::future_sample ? Status::out_of_range : Status::success;
            test.expect(pending.update(1001U, sample) == expected &&
                            ((pending.monitor.state().*entry.state).action != Action::reset) &&
                            pending.monitor.record_reset(entry.group, 1000U, true) == Status::out_of_range,
                        profile, "stale, unqualified, disabled and invalid calls cannot reset");
            test.expect(pending.update(1003U, {1003U, GnssQualityDecision::accepted}) == Status::success, profile,
                        "advance after revoked permission");
        }
    }
    Fixture timeout;
    test.expect(timeout.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                    timeout.update(1101U, {1101U, GnssQualityDecision::accepted}) == Status::success &&
                    (timeout.monitor.state().velocity.action == Action::reset) &&
                    timeout.monitor.state().velocity.active &&
                    (timeout.monitor.state().horizontal_position.action == Action::reset) &&
                    !timeout.monitor.state().restart_quality,
                profile, "coupled timeout requests resets without stopping quality/session");
    test.expect(timeout.monitor.record_reset(Group::velocity, 1101U, true) == Status::success &&
                    (timeout.monitor.state().horizontal_position.action == Action::reset) &&
                    timeout.monitor.record_reset(Group::horizontal_position, 1101U, true) == Status::success &&
                    timeout.update(1201U, {1201U, GnssQualityDecision::accepted}) == Status::success &&
                    (timeout.monitor.state().velocity.action == Action::fuse) &&
                    (timeout.monitor.state().horizontal_position.action == Action::fuse),
                profile, "first reset cannot cancel second; reset time renews strict deadline");
    test.expect(timeout.update(1202U, {1202U, GnssQualityDecision::accepted}) == Status::success &&
                    (timeout.monitor.state().velocity.action == Action::reset) &&
                    timeout.monitor.state().velocity.start_time_us == 1000U &&
                    timeout.monitor.state().velocity.last_fuse_time_us == 0U,
                profile, "new timeout uses reset progress without rewriting correction/session history");
    test.expect(timeout.monitor.record_reset(Group::vertical_position, 1202U, true) == Status::out_of_range &&
                    timeout.monitor.record_reset(static_cast<Group>(99), 1202U, true) == Status::domain_error,
                profile, "unsolicited height and invalid group feedback are rejected");

    Fixture numerical;
    test.expect(numerical.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                    numerical.monitor.record_fusion(Group::velocity, 1000U, FusionDecision::failed) ==
                        Status::success &&
                    numerical.update(1001U, {1001U, GnssQualityDecision::accepted}) == Status::success &&
                    (numerical.monitor.state().velocity.action != Action::reset),
                profile, "numerical correction failure is not automatic reset authorization");

    Fixture reentry;
    reentry.conditions.horizontal_position.start_without_reset = false;
    reentry.conditions.velocity.start_without_reset = false;
    test.expect(reentry.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                    reentry.monitor.record_reset(Group::velocity, 1000U, true) == Status::success &&
                    reentry.monitor.record_reset(Group::horizontal_position, 1000U, true) == Status::success,
                profile, "start reset-backed sessions");
    reentry.conditions.horizontal_position.enabled = false;
    reentry.conditions.velocity.enabled = false;
    test.expect(reentry.update(1001U) == Status::success && reentry.monitor.state().restart_quality, profile,
                "ordinary stop still restarts quality after reset-backed startup");
    reentry.conditions.horizontal_position.enabled = true;
    reentry.conditions.velocity.enabled = true;
    test.expect(reentry.update(1002U, {1002U, GnssQualityDecision::waiting}) == Status::success &&
                    (reentry.monitor.state().velocity.action != Action::reset) &&
                    reentry.update(1003U, {1003U, GnssQualityDecision::accepted}) == Status::success &&
                    (reentry.monitor.state().velocity.action == Action::reset) &&
                    (reentry.monitor.state().horizontal_position.action == Action::reset) &&
                    reentry.monitor.record_reset(Group::velocity, 1003U, true) == Status::success &&
                    reentry.monitor.record_reset(Group::horizontal_position, 1003U, true) == Status::success &&
                    reentry.monitor.state().velocity.start_time_us == 1003U &&
                    reentry.monitor.state().velocity.last_reset_time_us == 1003U &&
                    !reentry.monitor.state().restart_quality,
                profile, "re-entry waits for quality and completes new resets without a second quality stop");

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
                profile, "overdue height must attempt correction before requesting reset");
            auto const horizontal = fixture.monitor.state().horizontal_position;
            auto const velocity = fixture.monitor.state().velocity;
            test.expect(
                fixture.monitor.record_fusion(Group::vertical_position, start + 51U, FusionDecision::rejected) ==
                        Status::success &&
                    (fixture.monitor.state().vertical_position.action == Action::reset) &&
                    fixture.monitor.record_reset(Group::vertical_position, start + 51U, success) == Status::success,
                profile, "rejected overdue height grants one same-sample reset");
            auto const & height = fixture.monitor.state().vertical_position;
            test.expect(
                height.active == success && height.recovery_required == !success && height.action == Action::none &&
                    height.stop_reason == (success ? StopReason::none : StopReason::reset_failed) &&
                    height.last_reset_time_us == (success ? start + 51U : 0U) && height.last_fuse_time_us == start &&
                    height.start_time_us == start && !fixture.monitor.state().restart_quality &&
                    same_group(horizontal, fixture.monitor.state().horizontal_position) &&
                    same_group(velocity, fixture.monitor.state().velocity),
                profile, "height reset preserves correction, session and horizontal history");
            test.expect(
                fixture.monitor.record_reset(Group::vertical_position, start + 51U, true) == Status::out_of_range &&
                    fixture.monitor.record_fusion(Group::vertical_position, start + 51U, FusionDecision::fused) ==
                        Status::out_of_range,
                profile, "neither duplicate reset nor correction after reset is permitted");
            test.expect(fixture.update(start + 101U, {start + 101U, GnssQualityDecision::accepted}) == Status::success,
                        profile, "advance to reset-based height deadline");
            if (success)
            {
                test.expect(fixture.monitor.record_fusion(Group::vertical_position, start + 101U,
                                                          FusionDecision::rejected) == Status::success,
                            profile, "record height rejection at the reset-based deadline");
                test.expect(height.action == Action::none && !height.recovery_required, profile,
                            "deadline equality does not request another reset");
                test.expect(fixture.update(start + 102U, {start + 102U, GnssQualityDecision::accepted}) ==
                                Status::success,
                            profile, "advance beyond the reset-based deadline");
                test.expect(fixture.monitor.record_fusion(Group::vertical_position, start + 102U,
                                                          FusionDecision::rejected) == Status::success,
                            profile, "record overdue height rejection");
                test.expect(height.action == Action::reset, profile, "overdue rejection grants reset");
                test.expect(fixture.update(start + 103U) == Status::success, profile,
                            "advance without a new observation");
                test.expect(height.action == Action::none &&
                                fixture.monitor.record_reset(Group::vertical_position, start + 102U, true) ==
                                    Status::out_of_range,
                            profile, "the next update revokes an unconsumed reset");
            }
            else
            {
                test.expect(height.action == Action::none && height.recovery_required, profile,
                            "failed timeout reset cannot retry on good data");
            }
        }
    }
    Fixture height_reentry;
    height_reentry.conditions.vertical_position.start_without_reset = false;
    test.expect(height_reentry.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                    height_reentry.monitor.record_reset(Group::vertical_position, 1000U, true) == Status::success &&
                    height_reentry.update(1025U) == Status::success &&
                    !height_reentry.monitor.state().vertical_position.active &&
                    height_reentry.update(1026U, {1026U, GnssQualityDecision::waiting}) == Status::success &&
                    (height_reentry.monitor.state().vertical_position.action != Action::reset) &&
                    height_reentry.update(1027U, {1027U, GnssQualityDecision::accepted}) == Status::success &&
                    height_reentry.monitor.record_reset(Group::vertical_position, 1027U, true) == Status::success &&
                    height_reentry.monitor.state().vertical_position.start_time_us == 1027U &&
                    height_reentry.monitor.state().vertical_position.last_fuse_time_us == 0U &&
                    !height_reentry.monitor.state().restart_quality,
                profile, "height-only data stop restarts through a qualified reset, not a quality restart");
}

void start_acceleration_reset_fixture(Fixture & fixture, TestContext & test, std::string_view profile,
                                      std::uint64_t start)
{
    fixture.parameters = {7'000'000U, 1'000'000U, 5'000'000U};
    fixture.conditions.has_vertical_velocity = true;
    fixture.conditions.vertical_acceleration = {start, start};
    test.expect(fixture.update(start, {start, GnssQualityDecision::accepted}) == Status::success, profile,
                "start healthy vertical recovery fixture");
    for (auto group : {Group::velocity, Group::horizontal_position, Group::vertical_position})
    {
        test.expect(fixture.monitor.record_fusion(group, start, FusionDecision::fused) == Status::success, profile,
                    "seed real fusion progress");
    }
}

void acceleration_predicate(TestContext & test, std::string_view profile)
{
    for (auto start : {std::uint64_t{1'000'000}, std::numeric_limits<std::uint64_t>::max() - 1'000'000U})
    {
        for (auto age : {499'999U, 500'000U, 500'001U})
        {
            for (auto velocity : {FusionDecision::fused, FusionDecision::rejected, FusionDecision::failed})
            {
                for (auto height : {FusionDecision::fused, FusionDecision::rejected, FusionDecision::failed})
                {
                    Fixture fixture;
                    start_acceleration_reset_fixture(fixture, test, profile, start);
                    auto const now = start + age;
                    fixture.conditions.vertical_acceleration = {now - 1U, start};
                    test.expect(fixture.update(now, {now, GnssQualityDecision::accepted}) == Status::success, profile,
                                "qualified sample with preceding health times");
                    bool const expired = age > 500'000U;
                    if (expired)
                    {
                        test.expect(
                            fixture.monitor.record_fusion(Group::vertical_position, now, height, {now, start}) ==
                                    Status::out_of_range &&
                                fixture.monitor.record_vertical_velocity_reset(now, true) == Status::out_of_range,
                            profile, "height and Down reset must wait for velocity feedback");
                    }
                    test.expect(fixture.monitor.record_fusion(Group::velocity, now, velocity) == Status::success,
                                profile, "record actual velocity outcome before reset");
                    auto const & state = fixture.monitor.state();
                    bool const velocity_reset = expired && velocity != FusionDecision::failed;
                    test.expect((state.vertical_velocity_reset_reason != StopReason::none) == velocity_reset &&
                                    state.vertical_velocity_reset_reason ==
                                        (velocity_reset ? StopReason::bad_vertical_acceleration : StopReason::none),
                                profile, "500 ms equality is not expired; failed correction grants no reset");
                    if (velocity == FusionDecision::failed)
                    {
                        continue;
                    }
                    if (velocity_reset)
                    {
                        test.expect(fixture.monitor.record_vertical_velocity_reset(now, true) == Status::success &&
                                        state.vertical_velocity_reset_reason == StopReason::none &&
                                        state.last_vertical_velocity_reset_time_us == now,
                                    profile, "one Down reset acknowledges and clears its reason");
                    }
                    test.expect(fixture.monitor.record_fusion(Group::vertical_position, now, height, {now, start}) ==
                                    Status::success,
                                profile, "height uses the current horizon's health after correction");
                    bool const height_reset = expired && height != FusionDecision::failed;
                    test.expect((state.vertical_position.action == Action::reset) == height_reset &&
                                    state.vertical_position.last_fuse_time_us ==
                                        (height == FusionDecision::fused ? now : start),
                                profile, "persistent bad acceleration can reset even successfully fused height");
                    if (height_reset)
                    {
                        test.expect(state.vertical_position.stop_reason == StopReason::bad_vertical_acceleration &&
                                        fixture.monitor.record_reset(Group::vertical_position, now, true) ==
                                            Status::success &&
                                        state.vertical_position.last_reset_time_us == now,
                                    profile, "height reset remains distinct from its correction result");
                    }
                }
            }
        }
    }
    {
        constexpr std::uint64_t start = 1000000U;
        for (bool available : {false, true})
        {
            Fixture fixture;
            start_acceleration_reset_fixture(fixture, test, profile, start);
            fixture.conditions.vertical_acceleration = {available ? start : 0U, 0U};
            test.expect(fixture.update(start + 1U, {start + 1U, GnssQualityDecision::accepted}) == Status::success &&
                            fixture.monitor.record_fusion(Group::velocity, start + 1U, FusionDecision::rejected) ==
                                Status::success &&
                            (fixture.monitor.state().vertical_velocity_reset_reason != StopReason::none) == available,
                        profile, "known never-healthy is expired; absent health is not fault evidence");
            if (available)
            {
                test.expect(fixture.monitor.record_vertical_velocity_reset(start + 1U, true) == Status::success,
                            profile, "acknowledge known-never-healthy Down reset");
            }
            test.expect(fixture.monitor.record_fusion(Group::vertical_position, start + 1U, FusionDecision::fused,
                                                      {available ? start + 1U : 0U, 0U}) == Status::success &&
                            (fixture.monitor.state().vertical_position.action == Action::reset) == available,
                        profile, "known and unavailable zero timestamps have distinct height semantics");
        }
    }
    constexpr std::uint64_t start = 1'000'000U, now = start + 500'001U;
    Fixture baseline;
    start_acceleration_reset_fixture(baseline, test, profile, start);
    baseline.conditions.vertical_acceleration = {now - 1U, start};
    test.expect(baseline.update(now, {now, GnssQualityDecision::accepted}) == Status::success &&
                    baseline.monitor.record_fusion(Group::velocity, now, FusionDecision::fused) == Status::success,
                profile, "seed outstanding Down reset permission");
    for (auto invalid :
         {GnssFusion::VerticalAccelerationTimes{now + 2U, start}, GnssFusion::VerticalAccelerationTimes{now, now + 1U}})
    {
        auto fixture = baseline;
        fixture.conditions.vertical_acceleration = invalid;
        test.expect(fixture.update(now + 1U, {now + 1U, GnssQualityDecision::accepted}) == Status::out_of_range &&
                        fixture.monitor.state().vertical_velocity_reset_reason == StopReason::none &&
                        fixture.monitor.state().velocity.last_fuse_time_us == now &&
                        fixture.monitor.record_vertical_velocity_reset(now, true) == Status::out_of_range,
                    profile, "invalid health times revoke permissions without consuming history");
        fixture.conditions.vertical_acceleration = {now, start};
        test.expect(fixture.update(now + 1U, {now + 1U, GnssQualityDecision::accepted}) == Status::success, profile,
                    "invalid update did not consume timestamp");
    }
    test.expect(baseline.monitor.record_vertical_velocity_reset(now, true) == Status::success, profile,
                "finish Down reset before height timing validation");
    for (auto invalid :
         {GnssFusion::VerticalAccelerationTimes{now - 1U, start},
          GnssFusion::VerticalAccelerationTimes{now + 1U, start}, GnssFusion::VerticalAccelerationTimes{now, now + 1U}})
    {
        auto fixture = baseline;
        auto const before = fixture.monitor.state().vertical_position;
        test.expect(fixture.monitor.record_fusion(Group::vertical_position, now, FusionDecision::fused, invalid) ==
                            Status::out_of_range &&
                        same_group(before, fixture.monitor.state().vertical_position) &&
                        fixture.monitor.record_fusion(Group::vertical_position, now, FusionDecision::fused,
                                                      {now, now}) == Status::success,
                    profile, "height needs current ordered times; invalid feedback preserves permission");
    }

    using Times = GnssFusion::VerticalAccelerationTimes;
    test.expect(!Times{0U, UINT64_MAX}.reset_required(100U) && Times{100U, 0U}.reset_required(100U) &&
                    !Times{101U, 1U}.reset_required(100U) && !Times{100U, 101U}.reset_required(100U),
                profile, "direct predicate distinguishes absent, never-good and unordered snapshots");
}

void down_reset_order(TestContext & test, std::string_view profile)
{
    for (auto const start : {std::uint64_t{1000}, std::numeric_limits<std::uint64_t>::max() - 200U})
    {
        for (auto decision : {FusionDecision::fused, FusionDecision::rejected, FusionDecision::failed})
        {
            for (bool success : {false, true})
            {
                Fixture fixture;
                fixture.conditions.has_vertical_velocity = true;
                test.expect(fixture.update(start, {start, GnssQualityDecision::accepted}) == Status::success &&
                                fixture.monitor.record_fusion(Group::velocity, start, FusionDecision::fused) ==
                                    Status::success &&
                                fixture.monitor.record_fusion(Group::vertical_position, start, FusionDecision::fused) ==
                                    Status::success &&
                                fixture.update(start + 50U, {start + 50U, GnssQualityDecision::accepted}) ==
                                    Status::success &&
                                fixture.monitor.record_fusion(Group::velocity, start + 50U, FusionDecision::rejected) ==
                                    Status::success &&
                                (fixture.monitor.state().vertical_velocity_reset_reason == StopReason::none),
                            profile, "height deadline equality does not permit a Down velocity reset");
                test.expect(
                    fixture.update(start + 51U, {start + 51U, GnssQualityDecision::accepted}) == Status::success &&
                        fixture.monitor.record_vertical_velocity_reset(start + 51U, true) == Status::out_of_range &&
                        fixture.monitor.record_fusion(Group::vertical_position, start + 51U, FusionDecision::fused) ==
                            Status::out_of_range &&
                        fixture.monitor.record_fusion(Group::velocity, start + 51U, decision) == Status::success,
                    profile, "overdue height requires velocity feedback before Down reset and height correction");
                auto const before = fixture.monitor.state();
                if (decision == FusionDecision::failed)
                {
                    test.expect((before.vertical_velocity_reset_reason == StopReason::none) &&
                                    before.velocity.recovery_required &&
                                    fixture.monitor.record_vertical_velocity_reset(start + 51U, true) ==
                                        Status::out_of_range,
                                profile, "numerical correction failure never authorizes automatic reset");
                    continue;
                }
                test.expect(
                    (before.vertical_velocity_reset_reason != StopReason::none) &&
                        fixture.monitor.record_vertical_velocity_reset(start + 50U, true) == Status::out_of_range &&
                        (fixture.monitor.state().vertical_velocity_reset_reason != StopReason::none) &&
                        fixture.monitor.record_fusion(Group::vertical_position, start + 51U, FusionDecision::fused) ==
                            Status::out_of_range &&
                        fixture.monitor.record_vertical_velocity_reset(start + 51U, success) == Status::success,
                    profile, "fused or gate-rejected velocity permits exactly one matching Down reset");
                auto const & state = fixture.monitor.state();
                test.expect(
                    (state.vertical_velocity_reset_reason == StopReason::none) &&
                        state.last_vertical_velocity_reset_time_us == (success ? start + 51U : 0U) &&
                        same_group(before.horizontal_position, state.horizontal_position) &&
                        same_group(before.vertical_position, state.vertical_position) && !state.restart_quality &&
                        state.velocity.last_fuse_time_us == before.velocity.last_fuse_time_us &&
                        state.velocity.last_reset_time_us == before.velocity.last_reset_time_us &&
                        state.velocity.start_time_us == before.velocity.start_time_us &&
                        fixture.monitor.record_vertical_velocity_reset(start + 51U, true) == Status::out_of_range,
                    profile, "Down reset cannot manufacture height or horizontal progress");
                if (success)
                {
                    test.expect(
                        same_group(before.velocity, state.velocity) &&
                            fixture.monitor.record_fusion(Group::vertical_position, start + 51U,
                                                          FusionDecision::fused) == Status::success &&
                            (state.vertical_position.action != Action::reset) &&
                            fixture.update(start + 52U, {start + 52U, GnssQualityDecision::accepted}) ==
                                Status::success &&
                            fixture.monitor.record_fusion(Group::velocity, start + 52U, FusionDecision::fused) ==
                                Status::success &&
                            (state.vertical_velocity_reset_reason == StopReason::none),
                        profile,
                        "later successful height correction prevents another reset, not the one already completed");
                }
                else
                {
                    test.expect(!state.velocity.active && state.velocity.recovery_required &&
                                    state.velocity.stop_reason == StopReason::reset_failed &&
                                    fixture.update(start + 52U, {start + 52U, GnssQualityDecision::accepted}) ==
                                        Status::success &&
                                    state.velocity.action == Action::none &&
                                    (state.vertical_velocity_reset_reason == StopReason::none),
                                profile, "failed Down reset is terminal for velocity until caller repair");
                }
            }
        }
    }

    enum class Inhibition
    {
        no_sample,
        rejected_quality,
        waiting_quality,
        unavailable_quality,
        disabled_velocity,
        velocity_not_ready,
        horizontal_velocity_only,
        disabled_height,
        future_sample
    };
    constexpr std::array inhibitions{Inhibition::no_sample,
                                     Inhibition::rejected_quality,
                                     Inhibition::waiting_quality,
                                     Inhibition::unavailable_quality,
                                     Inhibition::disabled_velocity,
                                     Inhibition::velocity_not_ready,
                                     Inhibition::horizontal_velocity_only,
                                     Inhibition::disabled_height,
                                     Inhibition::future_sample};
    for (auto inhibition : inhibitions)
    {
        Fixture fixture;
        fixture.conditions.has_vertical_velocity = true;
        test.expect(fixture.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                        fixture.update(1051U, {1051U, GnssQualityDecision::accepted}) == Status::success &&
                        fixture.monitor.record_fusion(Group::velocity, 1051U, FusionDecision::rejected) ==
                            Status::success &&
                        (fixture.monitor.state().vertical_velocity_reset_reason != StopReason::none),
                    profile, "prepare one pending Down reset");
        auto sample = GnssFusion::Sample{1052U, GnssQualityDecision::accepted};
        switch (inhibition)
        {
        case Inhibition::no_sample:
            sample = {};
            break;
        case Inhibition::rejected_quality:
            sample.quality = GnssQualityDecision::rejected;
            break;
        case Inhibition::waiting_quality:
            sample.quality = GnssQualityDecision::waiting;
            break;
        case Inhibition::unavailable_quality:
            sample.quality_available = false;
            break;
        case Inhibition::disabled_velocity:
            fixture.conditions.velocity.enabled = false;
            break;
        case Inhibition::velocity_not_ready:
            fixture.conditions.velocity.ready = false;
            break;
        case Inhibition::horizontal_velocity_only:
            fixture.conditions.has_vertical_velocity = false;
            break;
        case Inhibition::disabled_height:
            fixture.conditions.vertical_position.enabled = false;
            break;
        case Inhibition::future_sample:
            sample.time_us = 1053U;
            break;
        }
        test.expect(fixture.update(1052U, sample) ==
                            (inhibition == Inhibition::future_sample ? Status::out_of_range : Status::success) &&
                        (fixture.monitor.state().vertical_velocity_reset_reason == StopReason::none) &&
                        fixture.monitor.record_vertical_velocity_reset(1051U, true) == Status::out_of_range,
                    profile, "every update revokes stale reset permission, including invalid calls");
        if (fixture.monitor.state().velocity.action == Action::fuse)
        {
            test.expect(fixture.monitor.record_fusion(Group::velocity, 1052U, FusionDecision::rejected) ==
                                Status::success &&
                            (fixture.monitor.state().vertical_velocity_reset_reason == StopReason::none),
                        profile, "horizontal-only velocity and disabled height cannot authorize Down reset");
        }
    }
    Fixture timeout;
    timeout.conditions.has_vertical_velocity = true;
    test.expect(
        timeout.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
            timeout.update(1051U, {1051U, GnssQualityDecision::accepted}) == Status::success &&
            timeout.monitor.record_fusion(Group::velocity, 1051U, FusionDecision::rejected) == Status::success &&
            timeout.monitor.record_vertical_velocity_reset(1051U, true) == Status::success &&
            timeout.update(1101U, {1101U, GnssQualityDecision::accepted}) == Status::success &&
            (timeout.monitor.state().velocity.action == Action::reset) &&
            timeout.monitor.record_reset(Group::velocity, 1101U, true) == Status::success &&
            (timeout.monitor.state().vertical_velocity_reset_reason == StopReason::none) &&
            timeout.monitor.record_vertical_velocity_reset(1101U, true) == Status::out_of_range,
        profile, "Down-only reset does not postpone coupled horizontal recovery or duplicate full velocity reset");

    Fixture reentry;
    reentry.conditions.has_vertical_velocity = true;
    reentry.conditions.vertical_position.start_without_reset = false;
    test.expect(reentry.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                    reentry.monitor.record_reset(Group::vertical_position, 1000U, true) == Status::success &&
                    reentry.update(1025U) == Status::success &&
                    reentry.update(1051U, {1051U, GnssQualityDecision::accepted}) == Status::success &&
                    reentry.monitor.record_reset(Group::vertical_position, 1051U, true) == Status::out_of_range &&
                    reentry.monitor.record_fusion(Group::velocity, 1051U, FusionDecision::rejected) ==
                        Status::success &&
                    reentry.monitor.record_reset(Group::vertical_position, 1051U, true) == Status::out_of_range,
                profile, "overdue height re-entry waits for Down reset before height initialization");
    for (auto group : {Group::velocity, Group::vertical_position})
    {
        auto repaired = reentry;
        test.expect(repaired.monitor.restart(group) == Status::success &&
                        (repaired.monitor.state().vertical_velocity_reset_reason == StopReason::none) &&
                        repaired.monitor.record_vertical_velocity_reset(1051U, true) == Status::out_of_range,
                    profile, "external repair revokes pending Down reset permission");
    }
    test.expect(reentry.monitor.record_vertical_velocity_reset(1051U, true) == Status::success &&
                    reentry.monitor.record_reset(Group::vertical_position, 1051U, true) == Status::success &&
                    reentry.monitor.state().vertical_position.start_time_us == 1051U,
                profile, "qualified re-entry performs Down velocity then height reset");
}

void reason_repetition(TestContext & test, std::string_view profile)
{
    constexpr std::uint64_t start = 1'000'000U, now = start + 500'001U;
    for (auto current_good : {std::uint64_t{0}, start, now})
    {
        Fixture fixture;
        start_acceleration_reset_fixture(fixture, test, profile, start);
        fixture.conditions.vertical_acceleration = {now - 1U, start};
        test.expect(fixture.update(now, {now, GnssQualityDecision::accepted}) == Status::success &&
                        fixture.monitor.record_fusion(Group::velocity, now, FusionDecision::fused) == Status::success &&
                        fixture.monitor.record_vertical_velocity_reset(now, true) == Status::success &&
                        fixture.monitor.record_fusion(Group::vertical_position, now, FusionDecision::fused,
                                                      {now, current_good}) == Status::success,
                    profile, "previous bad health resets Down before current health is known");
        auto const & state = fixture.monitor.state();
        test.expect((state.vertical_position.action == Action::reset) == (current_good != now), profile,
                    "current healthy verdict prevents height reset even during the 3 s held-fault interval");
        if (current_good != now)
        {
            test.expect(fixture.monitor.record_reset(Group::vertical_position, now, true) == Status::success, profile,
                        "acknowledge acceleration-triggered height reset");
            fixture.conditions.vertical_acceleration = {now, current_good};
            test.expect(fixture.update(now + 1U, {now + 1U, GnssQualityDecision::accepted}) == Status::success &&
                            fixture.monitor.record_fusion(Group::velocity, now + 1U, FusionDecision::fused) ==
                                Status::success &&
                            (state.vertical_velocity_reset_reason != StopReason::none),
                        profile, "successful reset does not invent a healthy acceleration timestamp or cooldown");
        }
    }

    Fixture priority;
    start_acceleration_reset_fixture(priority, test, profile, 1000000U);
    priority.conditions.vertical_acceleration = {6500000U, 1000000U};
    test.expect(priority.update(6500001U, {6500001U, GnssQualityDecision::accepted}) == Status::success &&
                    priority.monitor.record_fusion(Group::velocity, 6500001U, FusionDecision::rejected) ==
                        Status::success &&
                    priority.monitor.state().vertical_velocity_reset_reason == StopReason::bad_vertical_acceleration &&
                    priority.monitor.record_vertical_velocity_reset(6500001U, true) == Status::success &&
                    priority.monitor.record_fusion(Group::vertical_position, 6500001U, FusionDecision::rejected,
                                                   {6500001U, 1000000U}) == Status::success &&
                    priority.monitor.state().vertical_position.stop_reason == StopReason::bad_vertical_acceleration,
                profile, "acceleration reason takes priority when height timeout is also exceeded");
}

void stop_notification(TestContext & test, std::string_view profile)
{
    for (auto const & entry : groups)
    {
        for (bool lose_readiness : {false, true})
        {
            Fixture paused;
            test.expect(paused.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success, profile,
                        "start before disable/readiness loss");
            paused.conditions.*entry.conditions = {lose_readiness, !lose_readiness, true};
            test.expect(paused.update(1001U) == Status::success && !(paused.monitor.state().*entry.state).active &&
                            !(paused.monitor.state().*entry.state).recovery_required,
                        profile, "disable/readiness loss stops without permanent recovery latch");
            paused.conditions.*entry.conditions = {true, true, true};
            test.expect(paused.update(1002U) == Status::success && !(paused.monitor.state().*entry.state).active &&
                            paused.update(1003U, {1003U, GnssQualityDecision::accepted}) == Status::success &&
                            ((paused.monitor.state().*entry.state).action == Action::fuse),
                        profile, "caller-established re-entry still requires a new quality-accepted sample");
        }
    }

    for (bool readiness : {false, true})
    {
        Fixture f;
        test.expect(f.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success, profile,
                    "start both horizontal and height groups");
        if (readiness)
        {
            f.conditions.horizontal_position.ready = false;
        }
        else
        {
            f.conditions.horizontal_position.enabled = false;
        }
        test.expect(f.update(1001U) == Status::success && !f.monitor.state().restart_quality &&
                        f.monitor.state().velocity.active,
                    profile, "stopping first horizontal group does not restart quality");
        if (readiness)
        {
            f.conditions.velocity.ready = false;
        }
        else
        {
            f.conditions.velocity.enabled = false;
        }
        test.expect(f.update(1002U) == Status::success && f.monitor.state().restart_quality &&
                        f.monitor.state().vertical_position.active,
                    profile, "last horizontal stop notifies once even while height remains active");
        test.expect(f.update(1003U) == Status::success && !f.monitor.state().restart_quality, profile,
                    "notification is not a sticky latch");
    }
}

void invalid_updates(TestContext & test, std::string_view profile)
{
    Fixture baseline;
    test.expect(baseline.update(1000U, {998U, GnssQualityDecision::accepted}) == Status::success &&
                    baseline.monitor.record_fusion(Group::velocity, 998U, FusionDecision::fused) == Status::success,
                profile, "seed history");
    auto check_failure = [&](std::uint64_t now, GnssFusion::Sample const & sample,
                             GnssFusion::Parameters const & parameters, Status expected)
    {
        auto fixture = baseline;
        auto control = baseline;
        auto before = fixture.monitor.state();
        test.expect(fixture.monitor.update(now, fixture.conditions, parameters, sample) == expected, profile,
                    "invalid update reports expected status");
        for (auto const & entry : groups)
        {
            (before.*entry.state).action = Action::none;
            test.expect(same_group(fixture.monitor.state().*entry.state, before.*entry.state), profile,
                        "invalid call revokes permissions without rewriting history");
        }
        test.expect(fixture.update(1001U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                        control.update(1001U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                        same_state(fixture.monitor.state(), control.monitor.state()),
                    profile, "invalid call consumed no time/configuration");
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
}

void restart_frame(TestContext & test, std::string_view profile)
{
    Fixture baseline;
    test.expect(baseline.update(1000U, {998U, GnssQualityDecision::accepted}) == Status::success &&
                    baseline.monitor.record_fusion(Group::velocity, 998U, FusionDecision::fused) == Status::success &&
                    baseline.monitor.record_fusion(Group::horizontal_position, 998U, FusionDecision::rejected) ==
                        Status::success &&
                    baseline.monitor.record_fusion(Group::vertical_position, 998U, FusionDecision::failed) ==
                        Status::success,
                profile, "construct three different group histories");
    auto const before = baseline.monitor.state();
    for (auto const & g : groups)
    {
        auto f = baseline;
        test.expect(f.monitor.restart(g.group) == Status::success, profile, "explicit repaired-group acknowledgment");
        for (auto const & other : groups)
        {
            auto expected = other.group == g.group ? GnssFusion::GroupState{} : before.*other.state;
            test.expect(same_group(f.monitor.state().*other.state, expected), profile,
                        "only selected group history is cleared");
        }
        test.expect(f.update(1000U, {998U, GnssQualityDecision::accepted}) == Status::out_of_range, profile,
                    "restart retains consumed sample watermark");
        auto changed = f.parameters;
        ++changed.horizontal_timeout_us;
        test.expect(f.monitor.update(1001U, f.conditions, changed, {999U, GnssQualityDecision::accepted}) ==
                        Status::domain_error,
                    profile, "restart retains duration contract");
        test.expect(f.update(1001U, {999U, GnssQualityDecision::accepted}) == Status::success, profile,
                    "corrected newer sample succeeds");
    }
    test.expect(baseline.monitor.restart(static_cast<Group>(99)) == Status::domain_error &&
                    same_state(baseline.monitor.state(), before),
                profile, "invalid restart preserves all metadata");
    Fixture down;
    down.conditions.has_vertical_velocity = true;
    test.expect(down.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                    down.update(1051U, {1051U, GnssQualityDecision::accepted}) == Status::success &&
                    down.monitor.record_fusion(Group::velocity, 1051U, FusionDecision::fused) == Status::success &&
                    down.monitor.record_vertical_velocity_reset(1051U, true) == Status::success,
                profile, "establish actual Down-reset history before independent group repairs");
    for (auto const & g : groups)
    {
        auto repaired = down;
        auto const stored = repaired.monitor.state();
        test.expect(repaired.monitor.restart(g.group) == Status::success &&
                        repaired.monitor.state().last_vertical_velocity_reset_time_us ==
                            (g.group == Group::velocity ? 0U : 1051U),
                    profile, "only velocity repair clears its stored Down-reset timestamp");
        for (auto const & other : groups)
        {
            if (g.group != other.group)
            {
                test.expect(same_group(stored.*other.state, repaired.monitor.state().*other.state), profile,
                            "Down-history repair does not change unrelated group metadata");
            }
        }
    }
    baseline.monitor.reset();
    baseline.parameters = {200U, 30U, 60U};
    test.expect(same_state(baseline.monitor.state(), GnssFusion::State{}) &&
                    baseline.update(1U, {1U, GnssQualityDecision::accepted}) == Status::success,
                profile, "full reset permits a fresh timeline and duration configuration");
}

void large_timestamps(TestContext & test, std::string_view profile)
{
    for (std::uint64_t start : {std::uint64_t{1000}, UINT64_MAX - 1000U})
    {
        Fixture f;
        test.expect(f.update(start, {start, GnssQualityDecision::accepted}) == Status::success &&
                        f.monitor.record_fusion(Group::velocity, start, FusionDecision::fused) == Status::success &&
                        f.monitor.record_fusion(Group::horizontal_position, start, FusionDecision::fused) ==
                            Status::success,
                    profile, "translated initial progress");
        test.expect(f.update(start + 100U, {start + 100U, GnssQualityDecision::accepted}) == Status::success &&
                        f.monitor.state().velocity.action == Action::fuse &&
                        f.update(start + 101U, {start + 101U, GnssQualityDecision::accepted}) == Status::success &&
                        f.monitor.state().velocity.action == Action::reset &&
                        f.monitor.state().horizontal_position.action == Action::reset,
                    profile, "relative strict progress boundary is unchanged near integer limit");
        test.expect(f.monitor.record_reset(Group::velocity, start + 101U, true) == Status::success &&
                        f.monitor.state().velocity.last_reset_time_us == start + 101U &&
                        f.update(start + 200U) == Status::success && f.monitor.state().velocity.active &&
                        f.update(start + 201U) == Status::success && !f.monitor.state().velocity.active,
                    profile, "successful reset and receipt expiry use ordered subtraction");
    }
    Fixture f;
    f.parameters.horizontal_timeout_us = UINT64_MAX / 2U;
    test.expect(f.update(1U, {1U, GnssQualityDecision::accepted}) == Status::success, profile,
                "maximum legal horizontal duration accepted");
    Fixture bad;
    bad.parameters.horizontal_timeout_us = UINT64_MAX / 2U + 1U;
    test.expect(bad.update(1U, {1U, GnssQualityDecision::accepted}) == Status::domain_error, profile,
                "double-timeout overflow domain rejected");
}

void partial_groups(TestContext & test, std::string_view profile)
{
    constexpr std::uint64_t start = 1'000'000U, now = start + 500'001U;
    enum class Inhibition
    {
        no_sample,
        rejected_quality,
        waiting_quality,
        unavailable_quality,
        disabled_velocity,
        velocity_not_ready,
        horizontal_velocity_only,
        disabled_height
    };
    constexpr std::array inhibitions{Inhibition::no_sample,
                                     Inhibition::rejected_quality,
                                     Inhibition::waiting_quality,
                                     Inhibition::unavailable_quality,
                                     Inhibition::disabled_velocity,
                                     Inhibition::velocity_not_ready,
                                     Inhibition::horizontal_velocity_only,
                                     Inhibition::disabled_height};
    for (auto inhibition : inhibitions)
    {
        Fixture fixture;
        start_acceleration_reset_fixture(fixture, test, profile, start);
        fixture.conditions.vertical_acceleration = {now - 1U, start};
        auto sample = GnssFusion::Sample{now, GnssQualityDecision::accepted};
        switch (inhibition)
        {
        case Inhibition::no_sample:
            sample = {};
            break;
        case Inhibition::rejected_quality:
            sample.quality = GnssQualityDecision::rejected;
            break;
        case Inhibition::waiting_quality:
            sample.quality = GnssQualityDecision::waiting;
            break;
        case Inhibition::unavailable_quality:
            sample.quality_available = false;
            break;
        case Inhibition::disabled_velocity:
            fixture.conditions.velocity.enabled = false;
            break;
        case Inhibition::velocity_not_ready:
            fixture.conditions.velocity.ready = false;
            break;
        case Inhibition::horizontal_velocity_only:
            fixture.conditions.has_vertical_velocity = false;
            break;
        case Inhibition::disabled_height:
            fixture.conditions.vertical_position.enabled = false;
            break;
        }
        test.expect(fixture.update(now, sample) == Status::success, profile, "update unavailable reset context");
        if (fixture.monitor.state().velocity.action == Action::fuse)
        {
            test.expect(fixture.monitor.record_fusion(Group::velocity, now, FusionDecision::rejected) ==
                            Status::success,
                        profile, "record permitted velocity only");
        }
        test.expect(fixture.monitor.state().vertical_velocity_reset_reason == StopReason::none, profile,
                    "acceleration fault cannot bypass source permission or Down observation availability");
    }
    Fixture fixture;
    start_acceleration_reset_fixture(fixture, test, profile, start);
    fixture.conditions.vertical_acceleration = {now - 1U, now - 1U};
    test.expect(fixture.update(now, {now, GnssQualityDecision::accepted}) == Status::success &&
                    fixture.monitor.record_fusion(Group::vertical_position, now, FusionDecision::fused, {now, start}) ==
                        Status::success &&
                    (fixture.monitor.state().vertical_position.action == Action::reset),
                profile, "current health can independently request height reset");
    auto pending = fixture;
    test.expect(pending.update(now + 1U, {now + 1U, GnssQualityDecision::rejected}) == Status::success &&
                    (pending.monitor.state().vertical_position.action != Action::reset) &&
                    pending.update(now + 2U, {now + 2U, GnssQualityDecision::accepted}) == Status::success &&
                    (pending.monitor.state().vertical_position.action == Action::reset) &&
                    pending.monitor.record_reset(Group::vertical_position, now + 2U, true) == Status::success,
                profile, "pending acceleration reset waits for qualified data before repair");
    test.expect(fixture.monitor.record_reset(Group::vertical_position, now, false) == Status::success &&
                    fixture.update(now + 1U, {now + 1U, GnssQualityDecision::accepted}) == Status::success &&
                    (fixture.monitor.state().vertical_position.action != Action::reset) &&
                    fixture.monitor.state().vertical_position.stop_reason == StopReason::reset_failed,
                profile, "numerical reset failure does not auto-retry");

    Fixture f;
    f.conditions.vertical_position = {};
    f.conditions.has_vertical_velocity = false;
    test.expect(
        f.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
            f.monitor.record_fusion(Group::velocity, 1000U, FusionDecision::fused, {UINT64_MAX, UINT64_MAX}) ==
                Status::success &&
            f.monitor.state().vertical_velocity_reset_reason == StopReason::none &&
            !f.monitor.state().vertical_position.active,
        profile,
        "N/E velocity and absent height cannot invent Down-reset permission; nonheight feedback ignores health fields");
    test.expect(f.update(1001U, {0U, static_cast<GnssQualityDecision>(99), true}) == Status::success, profile,
                "no-sample payload quality is unconsumed");
    test.expect(f.update(1002U, {1002U, static_cast<GnssQualityDecision>(99), false}) == Status::success &&
                    f.monitor.state().velocity.action == Action::none,
                profile, "quality-unavailable receipt ignores verdict but cannot fuse");
}

void repair_persistence(TestContext & test, std::string_view profile)
{
    for (auto const & entry : groups)
    {
        Fixture fixture;
        test.expect(fixture.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                        fixture.monitor.record_fusion(entry.group, 1000U, FusionDecision::failed) == Status::success,
                    profile, "record numerical failure");
        auto const & state = fixture.monitor.state().*entry.state;
        test.expect(state.recovery_required && state.stop_reason == StopReason::fusion_failed &&
                        state.last_fuse_time_us == 0U,
                    profile, "numerical failure requests repair immediately");
        for (auto const & other : groups)
        {
            if (other.group != entry.group)
            {
                test.expect(((fixture.monitor.state().*other.state).action == Action::fuse), profile,
                            "one failed group does not erase another pending decision");
            }
        }
    }

    for (auto const & g : groups)
    {
        Fixture f;
        test.expect(f.update(1000U, {1000U, GnssQualityDecision::accepted}) == Status::success &&
                        f.monitor.record_fusion(g.group, 1000U, FusionDecision::failed) == Status::success,
                    profile, "latch numerical repair");
        for (unsigned step = 0; step < 4; ++step)
        {
            (f.conditions.*g.conditions).enabled = step != 0;
            (f.conditions.*g.conditions).ready = step != 1;
            (f.conditions.*g.conditions).start_without_reset = step == 3;
            auto const before_conditions = f.conditions;
            auto const before_parameters = f.parameters;
            GnssFusion::Sample const sample{1001U + step, GnssQualityDecision::accepted};
            test.expect(f.update(sample.time_us, sample) == Status::success, profile,
                        "later accepted metadata is not repair");
            auto const & state = f.monitor.state().*g.state;
            test.expect(state.recovery_required && !state.active && state.action == Action::none &&
                            state.stop_reason == StopReason::fusion_failed && state.last_fuse_time_us == 0U &&
                            state.last_reset_time_us == 0U,
                        profile, "repair latch survives good quality and all condition toggles");
            for (auto const & other : groups)
            {
                auto const a = f.conditions.*other.conditions;
                auto const b = before_conditions.*other.conditions;
                test.expect(a.enabled == b.enabled && a.ready == b.ready &&
                                a.start_without_reset == b.start_without_reset,
                            profile, "borrowed group conditions unchanged");
            }
            test.expect(f.parameters.horizontal_timeout_us == before_parameters.horizontal_timeout_us &&
                            f.parameters.height_data_timeout_us == before_parameters.height_data_timeout_us &&
                            f.parameters.height_fusion_timeout_us == before_parameters.height_fusion_timeout_us &&
                            sample.time_us == 1001U + step && sample.quality == GnssQualityDecision::accepted &&
                            sample.quality_available,
                        profile, "borrowed timing and sample objects remain unchanged");
        }
    }
}

template <typename Linalg> void quality_to_fusion(TestContext & test, std::string_view profile)
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
                    (fixture.monitor.state().horizontal_position.action == Action::fuse) &&
                    (fixture.monitor.state().vertical_position.action == Action::fuse) &&
                    (fixture.monitor.state().velocity.action == Action::fuse),
                profile, "actual quality qualification enables groups");

    formal_eskf::configuration::Ins::NominalState<Linalg> state;
    auto covariance = formal_eskf::linalg::Matrix<Linalg, 15U, 15U>::identity() * Scalar{3};
    Vector2 position;
    position(0U) = Scalar{1};
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
    test.expect(state.p_n(0) == Scalar{0.75} && state.p_n(1) == Scalar{0} && state.p_n(2) == Scalar{0} &&
                    formal_eskf::linalg::max_abs(state.v_n) == Scalar{0} &&
                    formal_eskf::linalg::max_abs(state.b_a) == Scalar{0} &&
                    formal_eskf::linalg::max_abs(state.b_g) == Scalar{0},
                profile, "independent diagonal model yields three quarters of measured horizontal position only");
    for (std::size_t row = 0; row < 15; ++row)
    {
        for (std::size_t column = 0; column < 15; ++column)
        {
            auto const expected = row != column ? Scalar{0} : row < 2U ? Scalar{0.75} : Scalar{3};
            test.expect(covariance(row, column) == expected, profile,
                        "independent scalar Joseph covariance for the only applied observation");
        }
    }
    auto const fused_position = state.p_n;
    auto const fused_covariance = covariance;
    test.expect(fixture.update(sample.time_us + 101U) == Status::success &&
                    !fixture.monitor.state().horizontal_position.active &&
                    formal_eskf::linalg::max_abs(state.p_n - fused_position) == Scalar{0} &&
                    formal_eskf::linalg::max_abs(covariance - fused_covariance) == Scalar{0},
                profile, "no-data tick stops without touching estimate or covariance");
    test.expect(fixture.monitor.state().restart_quality, profile, "outage emits the caller health-restart instruction");
    quality.restart_health();
    sample.time_us += 102U;
    quality_result = quality.update(sample, {}, sample.time_us, quality_parameters);
    test.expect(quality_result.qualified && quality_result.decision == GnssQualityDecision::waiting &&
                    fixture.update(sample.time_us, {sample.time_us, quality_result.decision}) == Status::success &&
                    fixture.monitor.state().horizontal_position.action == Action::none,
                profile, "outage cannot count as recent healthy time");
    sample.time_us += 1000001U;
    quality_result = quality.update(sample, {}, sample.time_us, quality_parameters);
    test.expect(quality_result.decision == GnssQualityDecision::accepted &&
                    fixture.update(sample.time_us, {sample.time_us, quality_result.decision}) == Status::success &&
                    fixture.monitor.state().horizontal_position.action == Action::fuse &&
                    fixture.monitor.state().velocity.recovery_required,
                profile, "quality recovery reopens ordinary group but cannot repair numerically failed velocity");
    quality.reset();
    fixture.monitor.reset();
    sample.time_us = 1U;
    quality_result = quality.update(sample, {}, sample.time_us, quality_parameters);
    test.expect(!quality_result.qualified && quality_result.decision == GnssQualityDecision::waiting &&
                    fixture.update(1U, {1U, quality_result.decision}) == Status::success &&
                    !fixture.monitor.state().velocity.recovery_required,
                profile, "explicit source reset clears both histories before a new timeline");
}

template <typename Linalg> void vertical_composition(TestContext & test, std::string_view profile)
{
    using T = typename Linalg::value_type;
    using State = formal_eskf::configuration::Ins::NominalState<Linalg>;
    using Covariance = formal_eskf::linalg::Matrix<Linalg, 15U, 15U>;
    using Matrix3 = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    using Vector3 = formal_eskf::linalg::Vector<Linalg, 3U>;
    using Health = formal_eskf::runtime::VerticalAccelerationHealth<Linalg>;
    struct Scenario
    {
        FusionDecision velocity;
        bool down_reset;
        bool current_good;
        FusionDecision height;
        bool height_reset;
    };
    Scenario const scenarios[]{
        {FusionDecision::fused, true, false, FusionDecision::fused, true},
        {FusionDecision::rejected, true, true, FusionDecision::rejected, true},
        {FusionDecision::failed, true, false, FusionDecision::fused, true},
        {FusionDecision::fused, false, false, FusionDecision::fused, true},
        {FusionDecision::fused, true, false, FusionDecision::failed, true},
        {FusionDecision::fused, true, false, FusionDecision::fused, false},
    };
    for (auto const & scenario : scenarios)
    {
        State state;
        Covariance P = Covariance::identity() * T{3};
        T const minimum = static_cast<T>(1e-6);
        Fixture lifecycle;
        lifecycle.parameters = {7000000U, 1000000U, 5000000U};
        lifecycle.conditions.has_vertical_velocity = true;
        formal_eskf::runtime::GnssQuality<Linalg> quality;
        formal_eskf::runtime::GnssQualityParameters<T> quality_parameters;
        quality_parameters.minimum_health_time_us = 1000000U;
        formal_eskf::runtime::GnssQualitySample<Linalg> sample;
        sample.time_us = 1U;
        sample.fix_type = 3U;
        sample.satellites = 8U;
        auto q = quality.update(sample, {}, sample.time_us, quality_parameters);
        test.expect(q.decision == GnssQualityDecision::waiting &&
                        lifecycle.update(sample.time_us, {sample.time_us, q.decision}) == Status::success,
                    profile, "quality wait grants no operation before composition begins");
        constexpr std::uint64_t start = 1000002U;
        sample.time_us = start;
        q = quality.update(sample, {}, start, quality_parameters);
        test.expect(q.decision == GnssQualityDecision::accepted &&
                        lifecycle.update(start, {start, q.decision}) == Status::success,
                    profile, "establish qualified sessions");
        auto velocity = formal_eskf::runtime::fuse_gnss_velocity(state, P, Vector3{}, Matrix3::identity() * T{3},
                                                                 minimum, T{1}, {false, T{0.25}, T{0.5}}, state, P);
        auto height = formal_eskf::runtime::fuse_height(state, P, T{0}, T{3}, minimum, T{1}, false, state, P);
        test.expect(velocity.fusion.decision == FusionDecision::fused &&
                        height.fusion.decision == FusionDecision::fused && P(5, 5) == T{1.5} && P(2, 2) == T{1.5} &&
                        lifecycle.monitor.record_fusion(Group::velocity, start, velocity.fusion.decision) ==
                            Status::success &&
                        lifecycle.monitor.record_fusion(Group::vertical_position, start, height.fusion.decision) ==
                            Status::success,
                    profile, "real initial operations establish actual progress and exact scalar Joseph variances");
        Health health;
        typename Health::Sample imu{start, T{0.125}, true, {false, false, false}};
        typename Health::Observations observations;
        test.expect(health.update(imu, observations) == Status::success && health.state().last_good_time_us == start,
                    profile, "actual preceding health starts good");
        constexpr auto now = start + 600001U;
        imu.time_us = now - 1U;
        observations.barometer_height = {true, T{2}, T{1}};
        observations.gnss_velocity_down = {true, T{8}, T{4}};
        test.expect(health.update(imu, observations) == Status::success && health.state().bad_sample &&
                        health.state().last_good_time_us == start,
                    profile, "independent pressure and GNSS evidence supplies preceding bad health");
        lifecycle.conditions.vertical_acceleration = {health.state().last_update_time_us,
                                                      health.state().last_good_time_us};
        sample.time_us = now;
        q = quality.update(sample, {}, now, quality_parameters);
        test.expect(q.decision == GnssQualityDecision::accepted &&
                        lifecycle.update(now, {now, q.decision}) == Status::success &&
                        lifecycle.monitor.state().velocity.action == Action::fuse &&
                        lifecycle.monitor.state().vertical_position.action == Action::fuse,
                    profile, "update snapshots preceding health before current sensor processing");
        Vector3 const measurement = Vector3::from_row_major({T{0}, T{0}, T{-8}});
        T const gate = scenario.velocity == FusionDecision::failed ? T{0} : T{1};
        bool const limit = scenario.velocity == FusionDecision::fused;
        velocity = formal_eskf::runtime::fuse_gnss_velocity(state, P, measurement, Matrix3::identity() * T{2.5},
                                                            minimum, gate, {limit, T{0.25}, T{0.5}}, state, P);
        test.expect(
            velocity.fusion.decision == scenario.velocity && state.v_n(2) == (limit ? T{-0.75} : T{0}) &&
                lifecycle.monitor.record_fusion(Group::velocity, now, velocity.fusion.decision) == Status::success,
            profile, "only actual velocity operation is reported; selected Down residual uses the current prior");
        if (scenario.velocity == FusionDecision::failed)
        {
            test.expect(lifecycle.monitor.state().vertical_velocity_reset_reason == StopReason::none &&
                            lifecycle.monitor.state().velocity.recovery_required && P(5, 5) == T{1.5},
                        profile, "failed operation cannot authorize or fabricate a numerical reset");
            continue;
        }
        test.expect(velocity.fusion.innovation(2) == T{-8} && velocity.fusion.innovation_variance(2) == T{4}, profile,
                    "raw evidence is from the unchanged pre-correction velocity prior");
        observations = {};
        observations.gnss_velocity_down = {true, -velocity.fusion.innovation(2),
                                           velocity.fusion.innovation_variance(2)};
        if (!scenario.current_good)
        {
            observations.barometer_height = {true, T{2}, T{1}};
        }
        imu.time_us = now;
        test.expect(
            health.update(imu, observations) == Status::success &&
                health.state().bad_sample == !scenario.current_good &&
                health.state().last_good_time_us == (scenario.current_good ? now : start),
            profile,
            "negated raw innovation and S feed actual current health; one GNSS family alone is not high likelihood");
        test.expect(lifecycle.monitor.state().vertical_velocity_reset_reason == StopReason::bad_vertical_acceleration &&
                        lifecycle.monitor.record_fusion(Group::vertical_position, now, FusionDecision::fused,
                                                        {now, now}) == Status::out_of_range,
                    profile, "current healthy verdict cannot cancel snapshot or permit out-of-order height feedback");
        T delta = T{123};
        T const before_down = state.v_n(2);
        auto const before_P = P;
        auto const reset_status = formal_eskf::runtime::reset_vertical_velocity(
            state, P, measurement(2), scenario.down_reset ? T{1} : T{-1}, state, P, delta);
        test.expect(reset_status == (scenario.down_reset ? Status::success : Status::domain_error) &&
                        lifecycle.monitor.record_vertical_velocity_reset(now, formal_eskf::succeeded(reset_status)) ==
                            Status::success,
                    profile, "acknowledge Down reset only after checked numerical operation");
        if (!scenario.down_reset)
        {
            test.expect(state.v_n(2) == before_down && delta == T{123} &&
                            formal_eskf::linalg::max_abs(P - before_P) == T{0} &&
                            lifecycle.monitor.state().velocity.recovery_required &&
                            lifecycle.monitor.state().last_vertical_velocity_reset_time_us == 0U,
                        profile, "failed Down reset preserves outputs and latches repair without progress");
            continue;
        }
        test.expect(state.v_n(2) == T{-8} && delta == T{-8} - before_down && P(5, 5) == T{1} && P(2, 2) == T{1.5} &&
                        lifecycle.monitor.state().last_vertical_velocity_reset_time_us == now,
                    profile, "successful Down reset reports actual delta without changing the height variance");
        bool const height_failure = scenario.height == FusionDecision::failed;
        bool const height_rejection = scenario.height == FusionDecision::rejected;
        T const height_measurement = height_rejection ? T{8} : T{2};
        height = formal_eskf::runtime::fuse_height(state, P, height_measurement, T{2.5}, minimum,
                                                   height_failure ? T{0} : T{1}, false, state, P);
        test.expect(height.fusion.decision == scenario.height &&
                        state.p_n(2) == (scenario.height == FusionDecision::fused ? T{0.75} : T{0}) &&
                        lifecycle.monitor.record_fusion(
                            Group::vertical_position, now, height.fusion.decision,
                            {health.state().last_update_time_us, health.state().last_good_time_us}) == Status::success,
                    profile, "height uses current state and current health after Down-reset acknowledgment");
        bool const reset_height = !scenario.current_good && !height_failure;
        test.expect((lifecycle.monitor.state().vertical_position.action == Action::reset) == reset_height, profile,
                    "current health independently controls height reset; numerical failure grants none");
        if (reset_height)
        {
            T const before_height = state.p_n(2);
            delta = T{123};
            auto const status = formal_eskf::runtime::reset_vertical_position(
                state, P, T{2}, scenario.height_reset ? T{1} : T{-1}, state, P, delta);
            test.expect(status == (scenario.height_reset ? Status::success : Status::domain_error) &&
                            lifecycle.monitor.record_reset(Group::vertical_position, now,
                                                           formal_eskf::succeeded(status)) == Status::success &&
                            state.p_n(2) == (scenario.height_reset ? T{2} : before_height) &&
                            delta == (scenario.height_reset ? T{2} - before_height : T{123}) &&
                            lifecycle.monitor.state().vertical_position.last_reset_time_us ==
                                (scenario.height_reset ? now : 0U),
                        profile, "height reset feedback and delta reflect the actual checked result");
        }
        // Receiver rejection is an admission failure, even if a fault exception could numerically pass.
        ++sample.time_us;
        sample.fix_type = 2U;
        q = quality.update(sample, {}, sample.time_us, quality_parameters);
        test.expect(q.decision == GnssQualityDecision::rejected &&
                        lifecycle.update(sample.time_us, {sample.time_us, q.decision}) == Status::success &&
                        lifecycle.monitor.state().velocity.action == Action::none &&
                        lifecycle.monitor.state().vertical_position.action == Action::none,
                    profile, "fault handling cannot bypass mandatory receiver qualification");
    }
}

} // namespace
int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Case;
    Case const cases[]{
        {"TC-R-GNSS-FUSION-01", "protocol", startup},
        {"TC-R-GNSS-FUSION-02", "protocol", feedback},
        {"TC-R-GNSS-FUSION-03", "protocol", expiry},
        {"TC-R-GNSS-FUSION-04", "protocol", horizontal_progress},
        {"TC-R-GNSS-FUSION-05", "protocol", height_progress},
        {"TC-R-GNSS-FUSION-06", "protocol", reset_feedback},
        {"TC-R-GNSS-FUSION-07", "protocol", acceleration_predicate},
        {"TC-R-GNSS-FUSION-08", "protocol", down_reset_order},
        {"TC-R-GNSS-FUSION-09", "protocol", reason_repetition},
        {"TC-R-GNSS-FUSION-10", "protocol", stop_notification},
        {"TC-R-GNSS-FUSION-11", "protocol", invalid_updates},
        {"TC-R-GNSS-FUSION-12", "protocol", restart_frame},
        {"TC-R-GNSS-FUSION-13", "protocol", large_timestamps},
        {"TC-R-GNSS-FUSION-14", "protocol", partial_groups},
        {"TC-R-GNSS-FUSION-15", "protocol", repair_persistence},
        {"TC-SW-GNSS-01", "binary32", quality_to_fusion<formal_eskf::test::Backend<float>>},
        {"TC-SW-GNSS-01", "binary64", quality_to_fusion<formal_eskf::test::Backend<double>>},
        {"TC-SW-GNSS-02", "binary32", vertical_composition<formal_eskf::test::Backend<float>>},
        {"TC-SW-GNSS-02", "binary64", vertical_composition<formal_eskf::test::Backend<double>>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
