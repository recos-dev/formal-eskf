/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <limits>
extern void __ESBMC_assume(bool condition);
extern void __ESBMC_assert(bool condition, char const * description);
#include <formal_eskf/runtime/measurement_fusion.hpp>
#define private public
#include <formal_eskf/runtime/magnetometer_lifecycle.hpp>
#undef private
namespace mag_lifecycle_proof
{
using Controller = formal_eskf::runtime::MagnetometerFusion;
using State = Controller::State;
using Source = Controller::Source;
using Sample = Controller::Sample;
using Parameters = Controller::Parameters;
using Conditions = Controller::Conditions;
using Action = Controller::Action;
using Reason = Controller::StopReason;
using Recovery = Controller::RecoveryReason;
using Flight = Controller::FlightState;
using Decision = formal_eskf::runtime::FusionDecision;
using Status = formal_eskf::Status;
using Time = std::uint64_t;
inline bool same_state(State const & a, State const & b)
{
    return a.active == b.active && a.action == b.action && a.filter_reset == b.filter_reset &&
           a.stop_reason == b.stop_reason && a.recovery_reason == b.recovery_reason &&
           a.last_field_failure_time_us == b.last_field_failure_time_us && a.start_time_us == b.start_time_us &&
           a.last_fuse_time_us == b.last_fuse_time_us && a.last_reset_time_us == b.last_reset_time_us;
}
inline bool same_parameters(Parameters const & a, Parameters const & b)
{
    return a.health_time_us == b.health_time_us && a.maximum_interval_us == b.maximum_interval_us &&
           a.fusion_timeout_us == b.fusion_timeout_us;
}
inline bool same_source(Source const & a, Source const & b)
{
    return a.device_id == b.device_id && a.calibration_count == b.calibration_count && a.reference_id == b.reference_id;
}
inline bool same_controller(Controller const & a, Controller const & b)
{
    return same_state(a.m_state, b.m_state) && same_parameters(a.m_parameters, b.m_parameters) &&
           same_source(a.m_source, b.m_source) && a.m_update_time_us == b.m_update_time_us &&
           a.m_sample_time_us == b.m_sample_time_us && a.m_sample_count == b.m_sample_count &&
           a.m_reset_on_timeout == b.m_reset_on_timeout && a.m_reset_sample_qualified == b.m_reset_sample_qualified &&
           a.m_heading_reset_required == b.m_heading_reset_required;
}
inline bool same_conditions(Conditions const & a, Conditions const & b)
{
    return a.enabled == b.enabled && a.ready == b.ready && a.starting_allowed == b.starting_allowed &&
           a.horizontal_aiding == b.horizontal_aiding && a.at_rest == b.at_rest && a.flight_state == b.flight_state &&
           a.heading_consistent == b.heading_consistent && a.heading_aligned == b.heading_aligned;
}
inline bool valid_parameters(Parameters const & p)
{
    return p.maximum_interval_us > 0U && p.maximum_interval_us <= std::numeric_limits<Time>::max() / 2U &&
           p.fusion_timeout_us > 0U;
}
inline bool invariant(Controller const & c)
{
    auto const & s = c.m_state;
    Time const now = c.m_update_time_us;
    bool const action = s.action == Action::none || s.action == Action::fuse || s.action == Action::reset;
    bool const recovery = s.recovery_reason == Recovery::none || s.recovery_reason == Recovery::fusion_timeout ||
                          s.recovery_reason == Recovery::fusion_failed || s.recovery_reason == Recovery::reset_failed ||
                          s.recovery_reason == Recovery::source_changed ||
                          s.recovery_reason == Recovery::reference_changed;
    return c.m_sample_time_us <= now && s.last_field_failure_time_us <= now && s.start_time_us <= now &&
           s.last_fuse_time_us <= now && s.last_reset_time_us <= now && c.m_sample_count <= 4U && action && recovery &&
           (now == 0U || valid_parameters(c.m_parameters)) &&
           (c.m_sample_time_us == 0U || c.m_source.device_id != 0U) &&
           (c.m_sample_count == 0U || c.m_sample_time_us != 0U) &&
           (!s.active || (s.start_time_us != 0U && s.recovery_reason == Recovery::none &&
                          (s.last_fuse_time_us != 0U || s.last_reset_time_us != 0U))) &&
           (s.action == Action::none || (now != 0U && c.m_sample_time_us != 0U)) &&
           (s.action != Action::fuse || s.recovery_reason == Recovery::none) &&
           (s.action != Action::reset ||
            (!s.active && c.m_reset_sample_qualified && s.recovery_reason != Recovery::fusion_failed &&
             s.recovery_reason != Recovery::reset_failed));
}
inline void stop(State & s, Reason reason)
{
    s.active = false;
    s.action = Action::none;
    s.stop_reason = reason;
}
inline void recovery(State & s, Recovery reason)
{
    if (s.recovery_reason == Recovery::none)
    {
        s.recovery_reason = reason;
    }
    stop(s, Reason::recovery_required);
}
inline void timeout(Controller & c)
{
    auto & s = c.m_state;
    Time const progress = s.last_fuse_time_us >= s.last_reset_time_us ? s.last_fuse_time_us : s.last_reset_time_us;
    if (!s.active || c.m_update_time_us - progress <= c.m_parameters.fusion_timeout_us)
    {
        return;
    }
    if (c.m_reset_on_timeout)
    {
        recovery(s, Recovery::fusion_timeout);
        if (c.m_reset_sample_qualified)
        {
            s.action = Action::reset;
        }
    }
    else
    {
        stop(s, Reason::fusion_timeout);
    }
}
inline Status input_status(Controller const & c, Time now, Conditions const & conditions, Parameters const & p,
                           Sample const & sample)
{
    bool const configured = valid_parameters(p) && (c.m_update_time_us == 0U || same_parameters(p, c.m_parameters));
    bool const ordered = now > c.m_update_time_us &&
                         (sample.time_us == 0U || (sample.time_us > c.m_sample_time_us && sample.time_us <= now));
    bool const source = sample.time_us == 0U || sample.source.device_id != 0U;
    bool const flight = conditions.flight_state == Flight::unknown || conditions.flight_state == Flight::on_ground ||
                        conditions.flight_state == Flight::in_air;
    return !configured          ? Status::domain_error
           : !ordered           ? Status::out_of_range
           : !source || !flight ? Status::domain_error
                                : Status::success;
}
inline void source_transition(Controller & c, Source const & source, Conditions const & conditions)
{
    bool const replaced = c.m_source.device_id != 0U && (source.device_id != c.m_source.device_id ||
                                                         source.calibration_count != c.m_source.calibration_count);
    auto & s = c.m_state;
    if (replaced)
    {
        Recovery const retained =
            s.recovery_reason == Recovery::fusion_failed || s.recovery_reason == Recovery::reset_failed
                ? s.recovery_reason
                : Recovery::source_changed;
        Time const last_failure = s.last_field_failure_time_us;
        c.m_heading_reset_required = c.m_heading_reset_required || (s.active && !conditions.horizontal_aiding);
        s = State{};
        s.last_field_failure_time_us = last_failure;
        s.recovery_reason = retained;
        stop(s, Reason::recovery_required);
        c.m_sample_count = 0U;
    }
    else if (c.m_source.device_id != 0U && source.reference_id != c.m_source.reference_id &&
             s.recovery_reason == Recovery::none && s.active && c.m_reset_on_timeout)
    {
        recovery(s, Recovery::reference_changed);
    }
    c.m_source = source;
    if (s.recovery_reason == Recovery::source_changed && conditions.flight_state == Flight::on_ground)
    {
        c.m_heading_reset_required = true;
    }
}
inline void sample_transition(Controller & c, Conditions const & conditions, bool accepted)
{
    auto & s = c.m_state;
    bool const numerical_repair =
        s.recovery_reason == Recovery::fusion_failed || s.recovery_reason == Recovery::reset_failed;
    if (!conditions.ready || numerical_repair)
    {
        stop(s, !conditions.ready ? Reason::not_ready : Reason::recovery_required);
        return;
    }
    if (!accepted || s.last_field_failure_time_us == 0U)
    {
        s.last_field_failure_time_us = c.m_update_time_us;
    }
    bool const healthy = c.m_parameters.health_time_us == 0U ||
                         c.m_update_time_us - s.last_field_failure_time_us > c.m_parameters.health_time_us;
    bool const fresh = c.m_update_time_us - c.m_sample_time_us < c.m_parameters.maximum_interval_us;
    c.m_reset_sample_qualified = conditions.starting_allowed && accepted && c.m_sample_count >= 4U && fresh && healthy;
    if (!conditions.heading_aligned)
    {
        stop(s, Reason::not_ready);
        if (c.m_reset_sample_qualified &&
            (s.recovery_reason != Recovery::source_changed || conditions.flight_state != Flight::unknown))
        {
            s.action = Action::reset;
        }
    }
    else if (s.recovery_reason == Recovery::source_changed)
    {
        stop(s, Reason::recovery_required);
        if (c.m_reset_sample_qualified && conditions.flight_state != Flight::unknown)
        {
            if (c.m_heading_reset_required)
            {
                s.action = Action::reset;
            }
            else
            {
                s.action = Action::fuse;
                s.stop_reason = Reason::none;
                s.recovery_reason = Recovery::none;
            }
        }
    }
    else if (s.recovery_reason == Recovery::fusion_timeout || s.recovery_reason == Recovery::reference_changed)
    {
        if (c.m_reset_on_timeout)
        {
            stop(s, Reason::recovery_required);
            if (c.m_reset_sample_qualified)
            {
                s.action = Action::reset;
            }
        }
        else
        {
            Reason const reason = s.recovery_reason == Recovery::fusion_timeout ? Reason::fusion_timeout : Reason::none;
            s.recovery_reason = Recovery::none;
            stop(s, reason);
        }
    }
    else if (s.active)
    {
        s.action = accepted ? Action::fuse : Action::none;
        s.stop_reason = accepted ? Reason::none : Reason::field_rejected;
        if (!accepted)
        {
            timeout(c);
        }
    }
    else
    {
        Reason const reason = !conditions.starting_allowed        ? Reason::not_ready
                              : !fresh                            ? Reason::data_timeout
                              : !accepted                         ? Reason::field_rejected
                              : c.m_sample_count < 4U || !healthy ? Reason::waiting_for_health
                                                                  : Reason::none;
        if (reason == Reason::none)
        {
            s.action = Action::fuse;
            s.stop_reason = Reason::none;
        }
        else
        {
            stop(s, reason);
        }
    }
}
inline void update_oracle(Controller & c, Time now, Conditions const & conditions, Parameters const & p,
                          Sample const & sample)
{
    c.m_state.action = Action::none;
    c.m_state.filter_reset = false;
    c.m_reset_sample_qualified = false;
    Status const status = input_status(c, now, conditions, p, sample);
    if (status != Status::success)
    {
        stop(c.m_state, Reason::invalid_call);
        return;
    }
    c.m_update_time_us = now;
    c.m_parameters = p;
    c.m_reset_on_timeout = !conditions.horizontal_aiding || (conditions.at_rest && conditions.heading_consistent);
    if (sample.time_us != 0U)
    {
        source_transition(c, sample.source, conditions);
        c.m_sample_time_us = sample.time_us;
    }
    if (!conditions.enabled)
    {
        stop(c.m_state, Reason::disabled);
        return;
    }
    if (sample.time_us == 0U)
    {
        if (c.m_state.recovery_reason != Recovery::none)
        {
            stop(c.m_state, Reason::recovery_required);
        }
        else if (c.m_sample_time_us == 0U || now - c.m_sample_time_us >= p.maximum_interval_us * 2U)
        {
            stop(c.m_state, Reason::data_timeout);
        }
        return;
    }
    if (c.m_sample_count < 4U)
    {
        c.m_state.filter_reset = c.m_sample_count == 0U;
        c.m_sample_count += 1U;
    }
    sample_transition(c, conditions, sample.field_accepted);
    if (c.m_state.action == Action::fuse && conditions.horizontal_aiding && !conditions.heading_consistent)
    {
        c.m_state.action = Action::none;
        c.m_state.stop_reason = Reason::heading_inconsistent;
    }
}
} // namespace mag_lifecycle_proof
void verify_mag_lifecycle_initialization(mag_lifecycle_proof::Controller controller, mag_lifecycle_proof::Source source)
{
    using namespace mag_lifecycle_proof;
    Controller fresh;
    State zero;
    Parameters parameters;
    Source empty;
    __ESBMC_assert(
        same_state(fresh.state(), zero) && same_parameters(fresh.m_parameters, parameters) &&
            same_source(fresh.m_source, empty) && fresh.m_update_time_us == 0U && fresh.m_sample_time_us == 0U &&
            fresh.m_sample_count == 0U && !fresh.m_reset_on_timeout && !fresh.m_reset_sample_qualified &&
            !fresh.m_heading_reset_required && invariant(fresh),
        "R-MAGNETOMETER-LIFECYCLE: constructor establishes every state field and timeline/permission invariants");
    auto reset = controller;
    reset.reset(source);
    auto expected = fresh;
    expected.m_source = source;
    __ESBMC_assert(
        same_controller(reset, expected) && invariant(reset),
        "R-MAGNETOMETER-LIFECYCLE: reset clears every clock/permission and binds only requested initial source");
    __ESBMC_assume(invariant(controller));
    expected = controller;
    expected.m_state = State{};
    expected.m_sample_count = 0U;
    expected.m_heading_reset_required = false;
    controller.restart();
    auto const & state = controller.state();
    __ESBMC_assert(same_controller(controller, expected) && invariant(controller) && &state == &controller.m_state,
                   "R-MAGNETOMETER-LIFECYCLE: explicit restart preserves source/time/configuration watermarks and "
                   "clears session authority");
}
void verify_mag_lifecycle_update(mag_lifecycle_proof::Controller controller, mag_lifecycle_proof::Time now,
                                 mag_lifecycle_proof::Conditions conditions, mag_lifecycle_proof::Parameters parameters,
                                 mag_lifecycle_proof::Sample sample)
{
    using namespace mag_lifecycle_proof;
    __ESBMC_assume(invariant(controller));
    auto const before = controller;
    auto const old_conditions = conditions;
    auto const old_parameters = parameters;
    auto const old_sample = sample;
    auto expected = controller;
    Status const wanted = input_status(controller, now, conditions, parameters, sample);
    update_oracle(expected, now, conditions, parameters, sample);
    auto const status = controller.update(now, conditions, parameters, sample);
    __ESBMC_assert(status == wanted && same_controller(controller, expected),
                   "R-MAGNETOMETER-LIFECYCLE: actual update matches source, qualification, timeout, recovery and "
                   "heading permissions");
    __ESBMC_assert(invariant(controller), "R-MAGNETOMETER-LIFECYCLE: every update preserves ordered clocks, saturated "
                                          "count and legitimate one-sample authority");
    __ESBMC_assert(controller.m_state.last_fuse_time_us == before.m_state.last_fuse_time_us ||
                       controller.m_state.last_fuse_time_us == 0U,
                   "R-MAGNETOMETER-LIFECYCLE: permission never fabricates fused progress");
    __ESBMC_assert(controller.m_state.last_reset_time_us == before.m_state.last_reset_time_us ||
                       controller.m_state.last_reset_time_us == 0U,
                   "R-MAGNETOMETER-LIFECYCLE: permission never fabricates successful reset progress");
    __ESBMC_assert(same_conditions(conditions, old_conditions) && same_parameters(parameters, old_parameters) &&
                       same_source(sample.source, old_sample.source) && sample.time_us == old_sample.time_us &&
                       sample.field_accepted == old_sample.field_accepted,
                   "R-MAGNETOMETER-LIFECYCLE: borrowed configuration, conditions and sample remain unchanged");
}
void verify_mag_lifecycle_fusion(mag_lifecycle_proof::Controller controller, mag_lifecycle_proof::Time sample_time,
                                 mag_lifecycle_proof::Decision decision)
{
    using namespace mag_lifecycle_proof;
    __ESBMC_assume(invariant(controller));
    auto expected = controller;
    bool const valid = decision == Decision::fused || decision == Decision::failed || decision == Decision::rejected;
    bool const permission = controller.m_state.action == Action::fuse && sample_time == controller.m_sample_time_us;
    Status const wanted = !valid ? Status::domain_error : !permission ? Status::out_of_range : Status::success;
    if (wanted == Status::success)
    {
        auto & s = expected.m_state;
        s.action = Action::none;
        if (decision == Decision::fused)
        {
            if (!s.active)
            {
                s.start_time_us = expected.m_update_time_us;
            }
            s.active = true;
            s.last_fuse_time_us = expected.m_update_time_us;
            s.stop_reason = Reason::none;
        }
        else if (decision == Decision::failed)
        {
            recovery(s, Recovery::fusion_failed);
        }
        else
        {
            s.stop_reason = Reason::innovation_rejected;
            timeout(expected);
        }
    }
    Status const status = controller.record_fusion(sample_time, decision);
    __ESBMC_assert(status == wanted, "R-MAGNETOMETER-LIFECYCLE: exact feedback status");
    __ESBMC_assert(same_controller(controller, expected), "R-MAGNETOMETER-LIFECYCLE: exact feedback and timeout frame");
    __ESBMC_assert(invariant(controller),
                   "R-MAGNETOMETER-LIFECYCLE: feedback preserves timeline and authority invariant");
    if (status == Status::success)
    {
        __ESBMC_assert(controller.m_state.action != Action::fuse,
                       "R-MAGNETOMETER-LIFECYCLE: consumed fusion authority cannot authorize repeated feedback");
    }
}
void verify_mag_lifecycle_reset(mag_lifecycle_proof::Controller controller, mag_lifecycle_proof::Time sample_time,
                                bool succeeded)
{
    using namespace mag_lifecycle_proof;
    __ESBMC_assume(invariant(controller));
    auto expected = controller;
    bool const permitted = controller.m_state.action == Action::reset && sample_time == controller.m_sample_time_us;
    Status const wanted = permitted ? Status::success : Status::out_of_range;
    if (permitted)
    {
        auto & s = expected.m_state;
        s.action = Action::none;
        if (succeeded)
        {
            if (s.start_time_us == 0U)
            {
                s.start_time_us = expected.m_update_time_us;
            }
            s.active = true;
            s.recovery_reason = Recovery::none;
            s.stop_reason = Reason::none;
            s.last_reset_time_us = expected.m_update_time_us;
            expected.m_heading_reset_required = false;
        }
        else
        {
            s.recovery_reason = Recovery::reset_failed;
            stop(s, Reason::recovery_required);
        }
    }
    Status const status = controller.record_reset(sample_time, succeeded);
    __ESBMC_assert(status == wanted && same_controller(controller, expected) && invariant(controller),
                   "R-MAGNETOMETER-LIFECYCLE: reset feedback is one-use, atomic on invalid call and never fabricates "
                   "fusion progress");
    if (status == Status::success)
    {
        __ESBMC_assert(controller.m_state.action != Action::reset,
                       "R-MAGNETOMETER-LIFECYCLE: consumed reset authority cannot authorize repeated feedback");
    }
}
int main() { return 0; }
