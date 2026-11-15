/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "contracts.hpp"
#include <algorithm>
#include <formal_eskf/linalg/operations.hpp>
#define private public
#include <formal_eskf/runtime/gnss_quality.hpp>
#undef private

#ifndef FORMAL_ESKF_PROOF_GNSS_BOUNDARY
#define FORMAL_ESKF_PROOF_GNSS_BOUNDARY 0
#endif
using contract_proof::finite_vector;
using contract_proof::Linalg;
using contract_proof::same;
using contract_proof::same_vector;
using contract_proof::Scalar;
using contract_proof::Status;
using contract_proof::Vector3;
using Monitor = formal_eskf::runtime::GnssQuality<Linalg>;
using Parameters = Monitor::parameters_type;
using Sample = Monitor::sample_type;
using Result = Monitor::result_type;
using Checks = formal_eskf::runtime::GnssQualityChecks;
using Decision = formal_eskf::runtime::GnssQualityDecision;
using Motion = formal_eskf::runtime::GnssQualityMotion;
using Vector2 = formal_eskf::linalg::Vector<Linalg, 2U>;

bool same_checks(Checks const & a, Checks const & b)
{
    return a.satellites == b.satellites && a.pdop == b.pdop && a.horizontal_accuracy == b.horizontal_accuracy &&
           a.vertical_accuracy == b.vertical_accuracy && a.speed_accuracy == b.speed_accuracy &&
           a.horizontal_drift == b.horizontal_drift && a.vertical_drift == b.vertical_drift &&
           a.horizontal_speed == b.horizontal_speed && a.vertical_speed == b.vertical_speed && a.spoofed == b.spoofed;
}
bool any_failure(Checks const & failures, Checks const & enabled)
{
    bool const f[]{failures.satellites,          failures.pdop,
                   failures.horizontal_accuracy, failures.vertical_accuracy,
                   failures.speed_accuracy,      failures.horizontal_drift,
                   failures.vertical_drift,      failures.horizontal_speed,
                   failures.vertical_speed,      failures.spoofed};
    bool const e[]{enabled.satellites,          enabled.pdop,
                   enabled.horizontal_accuracy, enabled.vertical_accuracy,
                   enabled.speed_accuracy,      enabled.horizontal_drift,
                   enabled.vertical_drift,      enabled.horizontal_speed,
                   enabled.vertical_speed,      enabled.spoofed};
    unsigned count = 0U;
    for (unsigned i = 0U; i < 10U; ++i)
    {
        count += f[i] && e[i] ? 1U : 0U;
    }
    return count != 0U;
}
bool same_result(Result const & a, Result const & b)
{
    return a.decision == b.decision && a.status == b.status && same_checks(a.failures, b.failures) &&
           a.invalid_fix == b.invalid_fix && a.qualified == b.qualified && a.drift_valid == b.drift_valid &&
           same(a.horizontal_drift, b.horizontal_drift) && same(a.vertical_drift, b.vertical_drift) &&
           same(a.horizontal_speed, b.horizontal_speed) && same(a.vertical_speed, b.vertical_speed) &&
           a.last_fail_us == b.last_fail_us && a.last_pass_us == b.last_pass_us;
}
bool same_monitor(Monitor const & a, Monitor const & b)
{
    return same_vector(a.m_position_rate, b.m_position_rate) && same_vector(a.m_velocity, b.m_velocity) &&
           a.m_sample_time_us == b.m_sample_time_us && a.m_update_time_us == b.m_update_time_us &&
           a.m_last_fail_us == b.m_last_fail_us && a.m_last_pass_us == b.m_last_pass_us &&
           same_checks(a.m_failures, b.m_failures) && a.m_previous_position_valid == b.m_previous_position_valid &&
           a.m_qualified == b.m_qualified;
}
bool same_sample(Sample const & a, Sample const & b)
{
    return a.time_us == b.time_us && a.fix_type == b.fix_type && a.satellites == b.satellites && same(a.pdop, b.pdop) &&
           same(a.horizontal_accuracy, b.horizontal_accuracy) && same(a.vertical_accuracy, b.vertical_accuracy) &&
           same(a.speed_accuracy, b.speed_accuracy) && a.spoofed == b.spoofed &&
           same_vector(a.velocity_n, b.velocity_n) && same_vector(a.position_delta_n, b.position_delta_n) &&
           a.position_reference_time_us == b.position_reference_time_us;
}
bool same_parameters(Parameters const & a, Parameters const & b)
{
    return same_checks(a.enabled_checks, b.enabled_checks) && a.minimum_satellites == b.minimum_satellites &&
           same(a.maximum_pdop, b.maximum_pdop) && same(a.maximum_horizontal_accuracy, b.maximum_horizontal_accuracy) &&
           same(a.maximum_vertical_accuracy, b.maximum_vertical_accuracy) &&
           same(a.maximum_speed_accuracy, b.maximum_speed_accuracy) &&
           same(a.maximum_horizontal_drift, b.maximum_horizontal_drift) &&
           same(a.maximum_vertical_drift, b.maximum_vertical_drift) && same(a.velocity_limit, b.velocity_limit) &&
           a.minimum_health_time_us == b.minimum_health_time_us;
}
bool time_invariant(Monitor const & m)
{
    return m.m_sample_time_us <= m.m_update_time_us && m.m_last_fail_us <= m.m_update_time_us &&
           m.m_last_pass_us <= m.m_update_time_us && (!m.m_previous_position_valid || m.m_sample_time_us != 0U);
}
Status parameters_status(Parameters const & p)
{
    Scalar const limits[]{p.maximum_pdop,           p.maximum_horizontal_accuracy, p.maximum_vertical_accuracy,
                          p.maximum_speed_accuracy, p.maximum_horizontal_drift,    p.maximum_vertical_drift,
                          p.velocity_limit};
    for (auto const limit : limits)
    {
        if (!std::isfinite(limit))
        {
            return Status::non_finite_input;
        }
        if (limit < std::numeric_limits<Scalar>::min())
        {
            return Status::domain_error;
        }
    }
    Scalar const h = Scalar{10} * p.maximum_horizontal_drift, v = Scalar{10} * p.maximum_vertical_drift;
    Scalar const h2 = p.maximum_horizontal_drift * p.maximum_horizontal_drift,
                 speed2 = p.velocity_limit * p.velocity_limit;
    return p.minimum_health_time_us == 0U || !std::isfinite(Scalar{2} * h * h) || !std::isfinite(v) ||
                   !std::isfinite(speed2) || h2 < std::numeric_limits<Scalar>::min() ||
                   speed2 < std::numeric_limits<Scalar>::min()
               ? Status::domain_error
               : Status::success;
}
Status sample_status(Sample const & s, Motion motion, bool prior_position)
{
    Scalar const accuracies[]{s.pdop, s.horizontal_accuracy, s.vertical_accuracy, s.speed_accuracy};
    for (auto const accuracy : accuracies)
    {
        if (!std::isfinite(accuracy))
        {
            return Status::non_finite_input;
        }
        if (accuracy < Scalar{0})
        {
            return Status::domain_error;
        }
    }
    return !finite_vector(s.velocity_n) ||
                   (!motion.in_air && motion.vehicle_at_rest && prior_position && !finite_vector(s.position_delta_n))
               ? Status::non_finite_input
               : Status::success;
}
void health_oracle(Monitor & m, std::uint64_t now, Parameters const & p, Result & r)
{
    bool const failing = r.decision == Decision::rejected;
    auto const last_fail = m.m_last_fail_us == 0U || failing ? now : m.m_last_fail_us;
    auto const elapsed = now - last_fail;
    bool const passing = r.decision == Decision::waiting;
    bool const recovered = elapsed > 1000000U && elapsed > p.minimum_health_time_us / 10U;
    m.m_last_fail_us = last_fail;
    if (passing)
    {
        m.m_last_pass_us = now;
        m.m_qualified = m.m_qualified || (recovered && elapsed > p.minimum_health_time_us);
        r.decision = recovered && m.m_qualified ? Decision::accepted : Decision::waiting;
    }
    r.last_fail_us = m.m_last_fail_us;
    r.last_pass_us = m.m_last_pass_us;
    r.qualified = m.m_qualified;
}
void verify_gnss_initialization(Monitor monitor)
{
    Monitor initial;
    Monitor expected;
    Parameters defaults;
    auto const & enabled = defaults.enabled_checks;
    __ESBMC_assert(
        enabled.satellites && enabled.pdop && enabled.horizontal_accuracy && enabled.vertical_accuracy &&
            enabled.speed_accuracy && enabled.horizontal_drift && enabled.vertical_drift && enabled.horizontal_speed &&
            enabled.vertical_speed && enabled.spoofed && defaults.minimum_satellites == 6U &&
            same(defaults.maximum_pdop, Scalar{2.5}) && same(defaults.maximum_horizontal_accuracy, Scalar{3}) &&
            same(defaults.maximum_vertical_accuracy, Scalar{5}) && same(defaults.maximum_speed_accuracy, Scalar{0.5}) &&
            same(defaults.maximum_horizontal_drift, static_cast<Scalar>(0.1)) &&
            same(defaults.maximum_vertical_drift, static_cast<Scalar>(0.2)) &&
            same(defaults.velocity_limit, Scalar{100}) && defaults.minimum_health_time_us == 10000000U,
        "GNSS quality: specified defaults, units and all ten enabled checks");
    bool const old_invariant = time_invariant(monitor);
    __ESBMC_assert(
        time_invariant(initial) && initial.m_sample_time_us == 0U && initial.m_update_time_us == 0U &&
            initial.m_last_fail_us == 0U && initial.m_last_pass_us == 0U && !initial.m_qualified &&
            !initial.m_previous_position_valid &&
            !any_failure(initial.m_failures, Checks{true, true, true, true, true, true, true, true, true, true}),
        "GNSS quality: constructor establishes empty qualification and ordered clocks");
    __ESBMC_assert(same_vector(initial.m_position_rate, Vector3::zero()) &&
                       same_vector(initial.m_velocity, Vector3::zero()),
                   "GNSS quality: constructor clears both filters");
    expected = monitor;
    expected.m_last_fail_us = expected.m_last_pass_us = 0U;
    monitor.restart_health();
    __ESBMC_assert(same_monitor(monitor, expected),
                   "GNSS quality: restart clears only recent health times, retaining qualification and source history");
    __ESBMC_assert(!old_invariant || time_invariant(monitor),
                   "GNSS quality: restart preserves the initialized clock invariant");
    monitor.reset();
    __ESBMC_assert(same_monitor(monitor, initial), "GNSS quality: reset restores the complete constructor invariant");
}
void verify_gnss_validation(Monitor monitor, Parameters p, Sample sample, Motion motion, Checks enabled,
                            Checks failures)
{
    auto const old_monitor = monitor;
    auto const old_p = p;
    auto const old_sample = sample;
    auto const old_enabled = enabled;
    auto const old_failures = failures;
    bool const expected_enabled = any_failure(failures, enabled);
    __ESBMC_assert(Monitor::validate_parameters(p) == parameters_status(p),
                   "GNSS quality: exact parameter domain and first error");
    __ESBMC_assert(monitor.validate_sample(sample, motion) ==
                       sample_status(sample, motion, monitor.m_previous_position_valid),
                   "GNSS quality: accuracy, mandatory velocity and conditionally consumed position validation");
    __ESBMC_assert(failures.any_enabled(enabled) == expected_enabled && same_checks(enabled, old_enabled) &&
                       same_checks(failures, old_failures),
                   "GNSS quality: each named selection gates only its own failure");
    __ESBMC_assert(same_monitor(monitor, old_monitor) && same_parameters(p, old_p) && same_sample(sample, old_sample),
                   "GNSS quality: validation is pure");
}
void verify_gnss_health(Monitor monitor, std::uint64_t now, Parameters parameters, Result result)
{
    __ESBMC_assume(time_invariant(monitor) && now >= monitor.m_update_time_us && now != 0U &&
                   parameters.minimum_health_time_us != 0U &&
                   (result.decision == Decision::rejected || result.decision == Decision::waiting));
    auto expected_monitor = monitor;
    auto expected_result = result;
    auto const old_parameters = parameters;
    health_oracle(expected_monitor, now, parameters, expected_result);
    monitor.update_health(now, parameters, result);
    __ESBMC_assert(
        same_monitor(monitor, expected_monitor) && same_result(result, expected_result) &&
            same_parameters(parameters, old_parameters),
        "GNSS quality: exact strict initial/recovery timing, sticky qualification and complete health frame");
}

struct MotionResults
{
    inline static Scalar roots[2]{};
    inline static unsigned norm_calls{};
    inline static bool check_norm_inputs{};
    inline static Vector2 norm_position{}, norm_velocity{};
    inline static Vector3 position_rate{}, velocity{};
    inline static Result result{};
    inline static Status status{};
    inline static Sample sample{};
    inline static Parameters parameters{};
    inline static Motion motion{};
    inline static unsigned calls{};
};
#if FORMAL_ESKF_PROOF_GNSS_BOUNDARY == 1
namespace formal_eskf::linalg
{
template <> inline Scalar norm<Linalg, 2U>(Vector2 const & vector)
{
    __ESBMC_assert(MotionResults::norm_calls < 2U, "GNSS motion: at most two pure norm results");
    if (MotionResults::check_norm_inputs)
    {
        __ESBMC_assert(MotionResults::norm_calls == 0U ? same_vector(vector, MotionResults::norm_position)
                                                       : same_vector(vector, MotionResults::norm_velocity),
                       "GNSS motion: horizontal norms consume the two filtered vectors in order");
    }
    return MotionResults::roots[MotionResults::norm_calls++];
}
} // namespace formal_eskf::linalg
#endif
#if FORMAL_ESKF_PROOF_GNSS_BOUNDARY == 2
namespace formal_eskf::runtime
{
template <>
inline Status GnssQuality<Linalg>::update_motion_checks(Sample const & sample, Motion motion,
                                                        Parameters const & parameters, Result & result)
{
    __ESBMC_assert(MotionResults::calls++ == 0U && same_sample(sample, MotionResults::sample) &&
                       same_parameters(parameters, MotionResults::parameters) &&
                       motion.in_air == MotionResults::motion.in_air &&
                       motion.vehicle_at_rest == MotionResults::motion.vehicle_at_rest,
                   "GNSS quality: motion stage receives unchanged sample, configuration and motion");
    __ESBMC_assert(validate_sample(sample, motion) == Status::success &&
                       validate_parameters(parameters) == Status::success && sample.time_us > m_sample_time_us,
                   "GNSS quality: caller establishes motion-stage validity and positive sample interval");
    // Actual producer frame is a required companion profile. No numerical
    // success, finite diagnostic/filter, or particular failure flag is assumed.
    m_position_rate = MotionResults::position_rate;
    m_velocity = MotionResults::velocity;
    result.failures.horizontal_drift = MotionResults::result.failures.horizontal_drift;
    result.failures.vertical_drift = MotionResults::result.failures.vertical_drift;
    result.failures.horizontal_speed = MotionResults::result.failures.horizontal_speed;
    result.failures.vertical_speed = MotionResults::result.failures.vertical_speed;
    result.horizontal_drift = MotionResults::result.horizontal_drift;
    result.vertical_drift = MotionResults::result.vertical_drift;
    result.horizontal_speed = MotionResults::result.horizontal_speed;
    result.vertical_speed = MotionResults::result.vertical_speed;
    result.drift_valid = MotionResults::result.drift_valid;
    return MotionResults::status;
}
} // namespace formal_eskf::runtime
#endif
#if FORMAL_ESKF_PROOF_GNSS_BOUNDARY == 0
void verify_gnss_norm(Vector2 vector, Scalar root)
{
    auto const old = vector;
    contract_proof::OpaqueMath::prepare(root, Scalar{0}, Scalar{0}, Scalar{0});
    auto const result = formal_eskf::linalg::norm(vector);
    __ESBMC_assert(same(result, root) && same_vector(vector, old) && contract_proof::OpaqueMath::sqrt.calls == 1U &&
                       same(contract_proof::OpaqueMath::sqrt.argument, contract_proof::ordered_squared_norm(old)),
                   "GNSS motion: actual same-type norm reduction and pure scalar-return boundary");
}
#endif
#if FORMAL_ESKF_PROOF_GNSS_BOUNDARY == 1
void verify_gnss_motion_frame(Monitor monitor, Sample sample, Parameters p, Motion motion, Result result, Scalar root0,
                              Scalar root1)
{
    // Private callee premises: established by the actual public validation.
    // Keep arbitrary prior filters and all arithmetic-failure outcomes.
    __ESBMC_assume(parameters_status(p) == Status::success &&
                   sample_status(sample, motion, monitor.m_previous_position_valid) == Status::success &&
                   sample.time_us > monitor.m_sample_time_us);

    auto const old_monitor = monitor;
    auto const old_sample = sample;
    auto const old_p = p;
    auto const old_result = result;
    MotionResults::roots[0] = root0;
    MotionResults::roots[1] = root1;
    MotionResults::norm_calls = 0U;
    MotionResults::check_norm_inputs = false;
    auto const status = monitor.update_motion_checks(sample, motion, p, result);
    __ESBMC_assert(status == Status::success || status == Status::non_finite_result,
                   "GNSS motion: producer status range");
    // Restore exactly the producer-owned fields to check the complete frame.
    monitor.m_position_rate = old_monitor.m_position_rate;
    monitor.m_velocity = old_monitor.m_velocity;
    result.failures.horizontal_drift = old_result.failures.horizontal_drift;
    result.failures.vertical_drift = old_result.failures.vertical_drift;
    result.failures.horizontal_speed = old_result.failures.horizontal_speed;
    result.failures.vertical_speed = old_result.failures.vertical_speed;
    result.horizontal_drift = old_result.horizontal_drift;
    result.vertical_drift = old_result.vertical_drift;
    result.horizontal_speed = old_result.horizontal_speed;
    result.vertical_speed = old_result.vertical_speed;
    result.drift_valid = old_result.drift_valid;
    __ESBMC_assert(same_monitor(monitor, old_monitor) && same_result(result, old_result) &&
                       same_sample(sample, old_sample) && same_parameters(p, old_p),
                   "GNSS motion: only filters and motion diagnostics change, including every failure");
}
#endif
#if FORMAL_ESKF_PROOF_GNSS_BOUNDARY == 2
void verify_gnss_transition(Monitor monitor, Sample sample, Parameters parameters, Motion motion, std::uint64_t now,
                            Status motion_status, Result motion_result, Vector3 position_rate, Vector3 velocity)
{
    __ESBMC_assume(time_invariant(monitor));
    auto const before = monitor;
    auto const old_sample = sample;
    auto const old_parameters = parameters;
    auto expected_monitor = monitor;
    Result expected;
    auto invalid = parameters_status(parameters);
    if (invalid == Status::success && (sample.time_us == 0U || sample.time_us <= monitor.m_sample_time_us ||
                                       now < sample.time_us || now < monitor.m_update_time_us ||
                                       (!motion.in_air && motion.vehicle_at_rest && monitor.m_previous_position_valid &&
                                        sample.position_reference_time_us != monitor.m_sample_time_us)))
    {
        invalid = Status::out_of_range;
    }
    unsigned expected_motion_calls = 0U;
    if (invalid != Status::success)
    {
        expected.status = invalid;
    }
    else
    {
        expected.failures = monitor.m_failures;
        expected.status = sample_status(sample, motion, monitor.m_previous_position_valid);
        if (expected.status == Status::success)
        {
            expected_motion_calls = 1U;
            expected.status = motion_status;
            expected_monitor.m_position_rate = position_rate;
            expected_monitor.m_velocity = velocity;
            expected.failures.horizontal_drift = motion_result.failures.horizontal_drift;
            expected.failures.vertical_drift = motion_result.failures.vertical_drift;
            expected.failures.horizontal_speed = motion_result.failures.horizontal_speed;
            expected.failures.vertical_speed = motion_result.failures.vertical_speed;
            expected.horizontal_drift = motion_result.horizontal_drift;
            expected.vertical_drift = motion_result.vertical_drift;
            expected.horizontal_speed = motion_result.horizontal_speed;
            expected.vertical_speed = motion_result.vertical_speed;
            expected.drift_valid = motion_result.drift_valid;
        }
        expected_monitor.m_previous_position_valid = expected.status == Status::success;
        if (expected.status != Status::success)
        {
            Status const rejected_status = expected.status;
            expected = Result{};
            expected.status = rejected_status;
            expected_monitor.m_position_rate = Vector3::zero();
            expected_monitor.m_velocity = Vector3::zero();
        }
        expected.invalid_fix = sample.fix_type < 3U;
        expected.failures.satellites = sample.satellites < parameters.minimum_satellites;
        expected.failures.pdop = sample.pdop > parameters.maximum_pdop;
        expected.failures.horizontal_accuracy = sample.horizontal_accuracy > parameters.maximum_horizontal_accuracy;
        expected.failures.vertical_accuracy = sample.vertical_accuracy > parameters.maximum_vertical_accuracy;
        expected.failures.speed_accuracy = sample.speed_accuracy > parameters.maximum_speed_accuracy;
        expected.failures.spoofed = sample.spoofed;
        expected_monitor.m_failures = expected.failures;
        bool const rejected = expected.status != Status::success || expected.invalid_fix ||
                              any_failure(expected.failures, parameters.enabled_checks);
        expected.decision = rejected ? Decision::rejected : Decision::waiting;
        health_oracle(expected_monitor, now, parameters, expected);
        expected_monitor.m_sample_time_us = sample.time_us;
        expected_monitor.m_update_time_us = now;
    }
    MotionResults::sample = sample;
    MotionResults::parameters = parameters;
    MotionResults::motion = motion;
    MotionResults::status = motion_status;
    MotionResults::result = motion_result;
    MotionResults::position_rate = position_rate;
    MotionResults::velocity = velocity;
    MotionResults::calls = 0U;
    auto const result = monitor.update(sample, motion, now, parameters);
    __ESBMC_assert(same_result(result, expected) && same_monitor(monitor, expected_monitor) &&
                       MotionResults::calls == expected_motion_calls,
                   "GNSS quality: complete public transition, invalid-call rollback, consumed rejection and all "
                   "health/diagnostic fields");
    __ESBMC_assert(time_invariant(monitor) && (!before.m_qualified || monitor.m_qualified),
                   "GNSS quality: every update preserves ordered clocks and historical qualification");
    __ESBMC_assert(same_sample(sample, old_sample) && same_parameters(parameters, old_parameters),
                   "GNSS quality: public update preserves all borrowed inputs");
}
#endif

#if FORMAL_ESKF_PROOF_GNSS_BOUNDARY == 1
Scalar clamp_value(Scalar value, Scalar lower, Scalar upper)
{
    return value < lower ? lower : upper < value ? upper : value;
}
Status motion_oracle(Monitor & monitor, Sample const & sample, Parameters const & p, Motion motion, Result & result,
                     Scalar root0, Scalar root1, unsigned & norm_calls)
{
    if (!motion.in_air && motion.vehicle_at_rest)
    {
        auto const dt =
            clamp_value(static_cast<Scalar>(sample.time_us - monitor.m_sample_time_us) * static_cast<Scalar>(1e-6),
                        static_cast<Scalar>(0.001), Scalar{10});
        auto const alpha = dt / Scalar{10};
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            auto const limit = axis < 2U ? p.maximum_horizontal_drift : p.maximum_vertical_drift;
            auto const delta = monitor.m_previous_position_valid ? sample.position_delta_n(axis) : Scalar{0};
            Scalar const filtered = (delta / dt) * alpha + monitor.m_position_rate(axis) * (Scalar{1} - alpha);
            if (!std::isfinite(filtered))
            {
                return Status::non_finite_result;
            }
            monitor.m_position_rate(axis) = clamp_value(filtered, -Scalar{10} * limit, Scalar{10} * limit);
            Scalar const limited_input = clamp_value(sample.velocity_n(axis), -Scalar{10} * limit, Scalar{10} * limit);
            monitor.m_velocity(axis) = limited_input * alpha + monitor.m_velocity(axis) * (Scalar{1} - alpha);
        }
        norm_calls = 2U;
        result.horizontal_drift = root0;
        result.horizontal_speed = root1;
        result.vertical_drift = std::fabs(monitor.m_position_rate(2U));
        result.vertical_speed = std::fabs(monitor.m_velocity(2U));
        if (!std::isfinite(root0) || !std::isfinite(root1) || !std::isfinite(result.vertical_drift) ||
            !std::isfinite(result.vertical_speed))
        {
            return Status::non_finite_result;
        }
        result.drift_valid = true;
        result.failures.horizontal_drift = root0 > p.maximum_horizontal_drift;
        result.failures.horizontal_speed = root1 > p.maximum_horizontal_drift;
        result.failures.vertical_drift = result.vertical_drift > p.maximum_vertical_drift;
        result.failures.vertical_speed = result.vertical_speed > p.maximum_vertical_drift;
    }
    else
    {
        monitor.m_position_rate = Vector3::zero();
        monitor.m_velocity = Vector3::zero();
        if (motion.in_air)
        {
            result.failures.horizontal_drift = false;
            result.failures.vertical_drift = false;
            result.failures.horizontal_speed = false;
            result.failures.vertical_speed = false;
        }
    }
    Scalar const horizontal =
        (Scalar{0} + sample.velocity_n(0U) * sample.velocity_n(0U)) + sample.velocity_n(1U) * sample.velocity_n(1U);
    Scalar const limit = p.velocity_limit * p.velocity_limit;
    if (!std::isfinite(horizontal) || !std::isfinite(limit))
    {
        return Status::non_finite_result;
    }
    if (horizontal > limit)
    {
        result.failures.horizontal_speed = true;
    }
    if (std::fabs(sample.velocity_n(2U)) > p.velocity_limit)
    {
        result.failures.vertical_speed = true;
    }
    return Status::success;
}
void verify_gnss_motion(Monitor monitor, Sample sample, Parameters p, Motion motion, Result result, Scalar root0,
                        Scalar root1)
{
    // Private callee premises: established by the actual public validation.
    // Keep arbitrary prior filters and all arithmetic-failure outcomes.
    __ESBMC_assume(parameters_status(p) == Status::success &&
                   sample_status(sample, motion, monitor.m_previous_position_valid) == Status::success &&
                   sample.time_us > monitor.m_sample_time_us);

    auto expected_monitor = monitor;
    auto expected_result = result;
    auto const old_sample = sample;
    auto const old_p = p;
    unsigned expected_calls = 0U;
    auto const expected_status =
        motion_oracle(expected_monitor, sample, p, motion, expected_result, root0, root1, expected_calls);
    MotionResults::roots[0] = root0;
    MotionResults::roots[1] = root1;
    MotionResults::norm_calls = 0U;
    MotionResults::check_norm_inputs = true;
    for (std::size_t axis = 0U; axis < 2U; ++axis)
    {
        MotionResults::norm_position(axis) = expected_monitor.m_position_rate(axis);
        MotionResults::norm_velocity(axis) = expected_monitor.m_velocity(axis);
    }
    auto const status = monitor.update_motion_checks(sample, motion, p, result);
    __ESBMC_assert(status == expected_status && MotionResults::norm_calls == expected_calls &&
                       same_monitor(monitor, expected_monitor) && same_result(result, expected_result) &&
                       same_sample(sample, old_sample) && same_parameters(p, old_p),
                   "GNSS motion: ordered IEEE filters, state/input clamp distinction, first-delta omission, "
                   "mode/reset/retained flags and raw speed limits");
}
#endif
int main() { return 0; }
