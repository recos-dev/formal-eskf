/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "contracts.hpp"
#include <formal_eskf/runtime/gnss_quality.hpp>
#include <formal_eskf/runtime/measurement_fusion.hpp>
#define private public
#include <formal_eskf/runtime/gnss_lifecycle.hpp>
#undef private
using Controller = formal_eskf::runtime::GnssFusion;
using Group = Controller::Group;
using GroupState = Controller::GroupState;
using Action = Controller::Action;
using Reason = Controller::StopReason;
using Conditions = Controller::Conditions;
using Parameters = Controller::Parameters;
using Sample = Controller::Sample;
using Health = Controller::VerticalAccelerationTimes;
using Status = formal_eskf::Status;
using Decision = formal_eskf::runtime::GnssQualityDecision;
using Fusion = formal_eskf::runtime::FusionDecision;
using Time = std::uint64_t;

bool same_group(GroupState const & a, GroupState const & b)
{
    return a.active == b.active && a.action == b.action && a.recovery_required == b.recovery_required &&
           a.stop_reason == b.stop_reason && a.start_time_us == b.start_time_us &&
           a.last_fuse_time_us == b.last_fuse_time_us && a.last_reset_time_us == b.last_reset_time_us;
}
bool same_parameters(Parameters const & a, Parameters const & b)
{
    return a.horizontal_timeout_us == b.horizontal_timeout_us && a.height_data_timeout_us == b.height_data_timeout_us &&
           a.height_fusion_timeout_us == b.height_fusion_timeout_us;
}
bool same_conditions(Conditions const & a, Conditions const & b)
{
    auto const group_equal = [](Controller::GroupConditions const & x, Controller::GroupConditions const & y)
    { return x.enabled == y.enabled && x.ready == y.ready && x.start_without_reset == y.start_without_reset; };
    return group_equal(a.horizontal_position, b.horizontal_position) &&
           group_equal(a.vertical_position, b.vertical_position) && group_equal(a.velocity, b.velocity) &&
           a.has_vertical_velocity == b.has_vertical_velocity &&
           a.vertical_acceleration.update_time_us == b.vertical_acceleration.update_time_us &&
           a.vertical_acceleration.last_good_time_us == b.vertical_acceleration.last_good_time_us;
}
bool same_controller(Controller const & a, Controller const & b)
{
    return same_group(a.m_state.horizontal_position, b.m_state.horizontal_position) &&
           same_group(a.m_state.vertical_position, b.m_state.vertical_position) &&
           same_group(a.m_state.velocity, b.m_state.velocity) &&
           a.m_state.vertical_velocity_reset_reason == b.m_state.vertical_velocity_reset_reason &&
           a.m_state.last_vertical_velocity_reset_time_us == b.m_state.last_vertical_velocity_reset_time_us &&
           a.m_state.restart_quality == b.m_state.restart_quality && same_parameters(a.m_parameters, b.m_parameters) &&
           a.m_update_time_us == b.m_update_time_us && a.m_sample_time_us == b.m_sample_time_us &&
           a.m_last_quality_pass_us == b.m_last_quality_pass_us &&
           a.m_vertical_velocity_reset_pending == b.m_vertical_velocity_reset_pending;
}
bool valid_parameters(Parameters const & p)
{
    return p.horizontal_timeout_us != 0U && p.horizontal_timeout_us <= std::numeric_limits<Time>::max() / 2U &&
           p.height_data_timeout_us != 0U && p.height_fusion_timeout_us != 0U;
}
bool reset_reason(Reason r)
{
    return r == Reason::initialization_required || r == Reason::fusion_timeout ||
           r == Reason::bad_vertical_acceleration;
}
bool group_invariant(GroupState const & g, Time now, Time sample)
{
    return g.start_time_us <= now && g.last_fuse_time_us <= now && g.last_reset_time_us <= now &&
           (!g.active || g.start_time_us != 0U) &&
           (g.action == Action::none ||
            (now != 0U && sample != 0U &&
             ((g.action == Action::fuse && g.active && !g.recovery_required) ||
              (g.action == Action::reset && g.recovery_required && reset_reason(g.stop_reason)))));
}
bool invariant(Controller const & c)
{
    return c.m_sample_time_us <= c.m_update_time_us && c.m_last_quality_pass_us <= c.m_update_time_us &&
           c.m_state.last_vertical_velocity_reset_time_us <= c.m_update_time_us &&
           (c.m_update_time_us == 0U || valid_parameters(c.m_parameters)) &&
           group_invariant(c.m_state.horizontal_position, c.m_update_time_us, c.m_sample_time_us) &&
           group_invariant(c.m_state.vertical_position, c.m_update_time_us, c.m_sample_time_us) &&
           group_invariant(c.m_state.velocity, c.m_update_time_us, c.m_sample_time_us) &&
           (c.m_vertical_velocity_reset_pending == Reason::none ||
            ((c.m_vertical_velocity_reset_pending == Reason::fusion_timeout ||
              c.m_vertical_velocity_reset_pending == Reason::bad_vertical_acceleration) &&
             c.m_state.velocity.action == Action::fuse && c.m_state.vertical_position.action != Action::none)) &&
           (c.m_state.vertical_velocity_reset_reason == Reason::none ||
            ((c.m_state.vertical_velocity_reset_reason == Reason::fusion_timeout ||
              c.m_state.vertical_velocity_reset_reason == Reason::bad_vertical_acceleration) &&
             c.m_sample_time_us != 0U && c.m_state.velocity.action == Action::none));
}
GroupState * group(Controller & c, Group g)
{
    if (g == Group::horizontal_position)
    {
        return &c.m_state.horizontal_position;
    }
    if (g == Group::vertical_position)
    {
        return &c.m_state.vertical_position;
    }
    if (g == Group::velocity)
    {
        return &c.m_state.velocity;
    }
    return nullptr;
}
void revoke(Controller & c)
{
    c.m_state.horizontal_position.action = Action::none;
    c.m_state.vertical_position.action = Action::none;
    c.m_state.velocity.action = Action::none;
    c.m_vertical_velocity_reset_pending = Reason::none;
    c.m_state.vertical_velocity_reset_reason = Reason::none;
}
void stop(GroupState & g, Reason reason)
{
    g.active = false;
    g.action = Action::none;
    if (!g.recovery_required)
    {
        g.stop_reason = reason;
    }
}
void request_reset(GroupState & g, Reason reason)
{
    if (!g.recovery_required)
    {
        g.stop_reason = reason;
    }
    g.recovery_required = true;
    g.action = Action::none;
}
bool expired(GroupState const & g, Time now, Time duration)
{
    Time progress = g.start_time_us;
    if (g.last_fuse_time_us > progress)
    {
        progress = g.last_fuse_time_us;
    }
    if (g.last_reset_time_us > progress)
    {
        progress = g.last_reset_time_us;
    }
    return progress == 0U || now - progress > duration;
}
bool bad_acceleration(Health h, Time now)
{
    if (h.update_time_us == 0U || h.update_time_us > now || h.last_good_time_us > h.update_time_us)
    {
        return false;
    }
    return h.last_good_time_us == 0U || now - h.last_good_time_us > 500000U;
}
Reason height_reason(Controller const & c, Health health)
{
    return bad_acceleration(health, c.m_update_time_us) ? Reason::bad_vertical_acceleration
           : expired(c.m_state.vertical_position, c.m_update_time_us, c.m_parameters.height_fusion_timeout_us)
               ? Reason::fusion_timeout
               : Reason::none;
}
Status validation(Controller const & c, Time now, Conditions const & conditions, Parameters const & p,
                  Sample const & sample)
{
    if (!valid_parameters(p) || (c.m_update_time_us != 0U && !same_parameters(p, c.m_parameters)))
    {
        return Status::domain_error;
    }
    if (now == 0U || now < c.m_update_time_us || (now == c.m_update_time_us && sample.time_us == 0U) ||
        (sample.time_us != 0U && (sample.time_us <= c.m_sample_time_us || sample.time_us > now)))
    {
        return Status::out_of_range;
    }
    auto const h = conditions.vertical_acceleration;
    if (h.update_time_us != 0U && (h.update_time_us > now || h.last_good_time_us > h.update_time_us))
    {
        return Status::out_of_range;
    }
    if (sample.time_us != 0U && sample.quality_available && sample.quality != Decision::accepted &&
        sample.quality != Decision::waiting && sample.quality != Decision::rejected)
    {
        return Status::domain_error;
    }
    return Status::success;
}
void group_oracle(GroupState & g, Controller::GroupConditions conditions, bool fresh, bool new_sample, bool accepted,
                  Time now)
{
    if (!conditions.enabled || !conditions.ready || !fresh)
    {
        Reason const reason = !conditions.enabled ? Reason::disabled
                              : !conditions.ready ? Reason::not_ready
                                                  : Reason::data_timeout;
        stop(g, reason);
        return;
    }
    if (!new_sample && g.active)
    {
        return;
    }
    if (accepted && !g.active && !g.recovery_required && !conditions.start_without_reset)
    {
        request_reset(g, Reason::initialization_required);
    }
    if (g.recovery_required)
    {
        g.action = accepted && reset_reason(g.stop_reason) ? Action::reset : Action::none;
        return;
    }
    g.stop_reason = accepted ? Reason::none : Reason::waiting_for_quality;
    if (accepted)
    {
        if (!g.active)
        {
            g.start_time_us = now;
        }
        g.active = true;
        g.action = Action::fuse;
    }
}
void update_oracle(Controller & c, Time now, Conditions const & conditions, Parameters const & p, Sample const & sample)
{
    bool const was_horizontal = c.m_state.horizontal_position.active || c.m_state.velocity.active;
    bool const was_velocity = c.m_state.velocity.active;
    c.m_update_time_us = now;
    c.m_parameters = p;
    c.m_state.restart_quality = false;
    bool const new_sample = sample.time_us != 0U;
    bool const accepted = new_sample && sample.quality_available && sample.quality == Decision::accepted;
    if (new_sample)
    {
        c.m_sample_time_us = sample.time_us;
        if (sample.quality_available && sample.quality != Decision::rejected)
        {
            c.m_last_quality_pass_us = now;
        }
    }
    bool const fresh = c.m_sample_time_us != 0U && now - c.m_sample_time_us < p.horizontal_timeout_us;
    bool const fresh_height = c.m_sample_time_us != 0U && now - c.m_sample_time_us < p.height_data_timeout_us;
    bool const poor = new_sample && !accepted && was_horizontal &&
                      (c.m_last_quality_pass_us == 0U || now - c.m_last_quality_pass_us > p.horizontal_timeout_us);
    if (poor || (!new_sample && was_horizontal && !fresh))
    {
        Reason const reason = poor ? Reason::poor_quality : Reason::data_timeout;
        stop(c.m_state.horizontal_position, reason);
        stop(c.m_state.vertical_position, reason);
        stop(c.m_state.velocity, reason);
    }
    else
    {
        bool const both_expired = expired(c.m_state.horizontal_position, now, p.horizontal_timeout_us) &&
                                  expired(c.m_state.velocity, now, p.horizontal_timeout_us);
        bool const position_expired = conditions.horizontal_position.enabled &&
                                      expired(c.m_state.horizontal_position, now, 2U * p.horizontal_timeout_us);
        if (accepted && fresh && was_horizontal && (both_expired || position_expired))
        {
            if (c.m_state.horizontal_position.active && conditions.horizontal_position.enabled &&
                conditions.horizontal_position.ready)
            {
                request_reset(c.m_state.horizontal_position, Reason::fusion_timeout);
            }
            if (c.m_state.velocity.active && conditions.velocity.enabled && conditions.velocity.ready)
            {
                request_reset(c.m_state.velocity, Reason::fusion_timeout);
            }
        }
        group_oracle(c.m_state.horizontal_position, conditions.horizontal_position, fresh, new_sample, accepted, now);
        group_oracle(c.m_state.vertical_position, conditions.vertical_position, fresh_height, new_sample, accepted,
                     now);
        group_oracle(c.m_state.velocity, conditions.velocity, fresh, new_sample, accepted, now);
        if (conditions.has_vertical_velocity && was_velocity && c.m_state.velocity.action == Action::fuse &&
            c.m_state.vertical_position.action != Action::none && c.m_state.vertical_position.start_time_us != 0U)
        {
            c.m_vertical_velocity_reset_pending = height_reason(c, conditions.vertical_acceleration);
        }
    }
    if (was_horizontal && !c.m_state.horizontal_position.active && !c.m_state.velocity.active)
    {
        c.m_state.restart_quality = true;
        c.m_last_quality_pass_us = 0U;
    }
}
void verify_gnss_lifecycle_initialization(Controller c)
{
    Controller initial;
    __ESBMC_assert(invariant(initial) && initial.m_update_time_us == 0U && initial.m_sample_time_us == 0U &&
                       initial.m_last_quality_pass_us == 0U && !initial.m_state.horizontal_position.active &&
                       !initial.m_state.vertical_position.active && !initial.m_state.velocity.active,
                   "GNSS lifecycle: construction establishes empty ordered history and no permissions");
    c.reset();
    __ESBMC_assert(same_controller(c, initial), "GNSS lifecycle: reset restores every persistent field");
}
void verify_gnss_lifecycle_update(Controller c, Time now, Conditions conditions, Parameters p, Sample sample)
{
    __ESBMC_assume(invariant(c));
    auto expected = c;
    auto const old_conditions = conditions;
    auto const old_parameters = p;
    auto const old_sample = sample;
    auto const status = validation(c, now, conditions, p, sample);
    revoke(expected);
    if (status == Status::success)
    {
        update_oracle(expected, now, conditions, p, sample);
    }
    auto const actual = c.update(now, conditions, p, sample);
    __ESBMC_assert(
        actual == status && same_controller(c, expected),
        "GNSS lifecycle: exact validation, freshness, quality/timeout priorities, three-group permissions and history");
    __ESBMC_assert(same_conditions(conditions, old_conditions) && same_parameters(p, old_parameters) &&
                       sample.time_us == old_sample.time_us && sample.quality == old_sample.quality &&
                       sample.quality_available == old_sample.quality_available,
                   "GNSS lifecycle: update preserves all borrowed conditions, parameters and sample fields");
    __ESBMC_assert(invariant(c),
                   "GNSS lifecycle: every public update preserves ordered progress and one-operation permissions");
}

void verify_gnss_lifecycle_fusion(Controller c, Group selected, Time sample_time, Fusion decision, Health health)
{
    __ESBMC_assume(invariant(c));
    auto expected = c;
    auto * target = group(expected, selected);
    auto const old_health = health;
    Status status = Status::success;
    bool const waiting = expected.m_vertical_velocity_reset_pending != Reason::none ||
                         expected.m_state.vertical_velocity_reset_reason != Reason::none;
    if (target == nullptr || (decision != Fusion::fused && decision != Fusion::rejected && decision != Fusion::failed))
    {
        status = Status::domain_error;
    }
    else if (target->action != Action::fuse || sample_time != expected.m_sample_time_us ||
             (selected == Group::vertical_position && waiting) ||
             (selected == Group::vertical_position && health.update_time_us != 0U &&
              (health.update_time_us != expected.m_update_time_us || health.last_good_time_us > health.update_time_us)))
    {
        status = Status::out_of_range;
    }
    else
    {
        target->action = Action::none;
        if (decision == Fusion::fused)
        {
            target->last_fuse_time_us = expected.m_update_time_us;
        }
        if (decision == Fusion::failed)
        {
            stop(*target, Reason::fusion_failed);
            target->recovery_required = true;
        }
        if (selected == Group::velocity)
        {
            expected.m_state.vertical_velocity_reset_reason =
                decision == Fusion::failed ? Reason::none : expected.m_vertical_velocity_reset_pending;
            expected.m_vertical_velocity_reset_pending = Reason::none;
        }
        if (selected == Group::vertical_position && target->active)
        {
            auto const reason = height_reason(expected, health);
            if (reason != Reason::none)
            {
                request_reset(*target, reason);
                target->action = Action::reset;
            }
        }
    }
    auto const actual = c.record_fusion(selected, sample_time, decision, health);
    __ESBMC_assert(actual == status && same_controller(c, expected) && invariant(c) &&
                       health.update_time_us == old_health.update_time_us &&
                       health.last_good_time_us == old_health.last_good_time_us,
                   "GNSS lifecycle: only actual fused feedback advances fusion time; rejection/failure, height health "
                   "and Down-reset handoff are exact");
    if (actual == Status::success)
    {
        auto const * consumed = group(c, selected);
        __ESBMC_assert(consumed != nullptr && consumed->action != Action::fuse,
                       "GNSS lifecycle: successful acknowledgment consumes fusion authority");
    }
}
void verify_gnss_lifecycle_reset_feedback(Controller c, Group selected, Time sample_time, bool success)
{
    __ESBMC_assume(invariant(c));
    auto expected = c;
    auto * target = group(expected, selected);
    bool const waiting = expected.m_vertical_velocity_reset_pending != Reason::none ||
                         expected.m_state.vertical_velocity_reset_reason != Reason::none;
    Status status = Status::success;
    if (target == nullptr)
    {
        status = Status::domain_error;
    }
    else if (target->action != Action::reset || sample_time != expected.m_sample_time_us ||
             (selected == Group::vertical_position && waiting))
    {
        status = Status::out_of_range;
    }
    else
    {
        target->action = Action::none;
        if (!success)
        {
            target->stop_reason = Reason::reset_failed;
            stop(*target, Reason::reset_failed);
            target->recovery_required = true;
        }
        else
        {
            if (!target->active)
            {
                target->start_time_us = expected.m_update_time_us;
            }
            target->active = true;
            target->recovery_required = false;
            target->stop_reason = Reason::none;
            target->last_reset_time_us = expected.m_update_time_us;
        }
    }
    auto const actual = c.record_reset(selected, sample_time, success);
    __ESBMC_assert(
        actual == status && same_controller(c, expected) && invariant(c),
        "GNSS lifecycle: reset acknowledgment is one-use, distinct from fusion progress, with exact recovery failure");
    if (actual == Status::success)
    {
        auto const * consumed = group(c, selected);
        __ESBMC_assert(consumed != nullptr && consumed->action != Action::reset,
                       "GNSS lifecycle: successful acknowledgment consumes reset authority");
    }
}
void verify_gnss_lifecycle_vertical_feedback(Controller c, Time sample_time, bool success)
{
    __ESBMC_assume(invariant(c));
    auto expected = c;
    Status status = Status::out_of_range;
    if (expected.m_state.vertical_velocity_reset_reason != Reason::none && sample_time == expected.m_sample_time_us)
    {
        status = Status::success;
        expected.m_state.vertical_velocity_reset_reason = Reason::none;
        expected.m_vertical_velocity_reset_pending = Reason::none;
        if (success)
        {
            expected.m_state.last_vertical_velocity_reset_time_us = expected.m_update_time_us;
        }
        else
        {
            stop(expected.m_state.velocity, Reason::reset_failed);
            expected.m_state.velocity.recovery_required = true;
        }
    }
    auto const actual = c.record_vertical_velocity_reset(sample_time, success);
    __ESBMC_assert(
        actual == status && same_controller(c, expected) && invariant(c),
        "GNSS lifecycle: Down-only reset acknowledgment preserves 3D/horizontal progress and revokes its permission");
    if (actual == Status::success)
    {
        __ESBMC_assert(c.m_state.vertical_velocity_reset_reason == Reason::none &&
                           c.m_vertical_velocity_reset_pending == Reason::none,
                       "GNSS lifecycle: successful Down acknowledgment consumes its authority");
    }
}
void verify_gnss_lifecycle_restart(Controller c, Group selected)
{
    __ESBMC_assume(invariant(c));
    auto expected = c;
    auto * target = group(expected, selected);
    Status const status = target == nullptr ? Status::domain_error : Status::success;
    if (target != nullptr)
    {
        *target = GroupState{};
        if (selected == Group::vertical_position || selected == Group::velocity)
        {
            expected.m_vertical_velocity_reset_pending = Reason::none;
            expected.m_state.vertical_velocity_reset_reason = Reason::none;
        }
        if (selected == Group::velocity)
        {
            expected.m_state.last_vertical_velocity_reset_time_us = 0U;
        }
    }
    auto const actual = c.restart(selected);
    __ESBMC_assert(
        actual == status && same_controller(c, expected) && invariant(c),
        "GNSS lifecycle: explicit caller-authorized restart resets only selected session and linked recovery metadata");
}
void verify_gnss_lifecycle_health(Health health, Time now)
{
    auto const before = health;
    __ESBMC_assert(health.reset_required(now) == bad_acceleration(health, now) &&
                       health.update_time_us == before.update_time_us &&
                       health.last_good_time_us == before.last_good_time_us,
                   "GNSS lifecycle: known ordered health and strict half-second bad-acceleration threshold; "
                   "unavailable is not failure");
}
int main() { return 0; }
