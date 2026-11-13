/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include <formal_eskf/eskf/accelerometer.hpp>
#include <formal_eskf/eskf/types.hpp>
#include <formal_eskf/linalg/operations.hpp>
#include <formal_eskf/runtime/measurement_fusion.hpp>
#include <formal_eskf/scalar/math.hpp>
#include <formal_eskf/so3/operations.hpp>
#include <formal_eskf/so3/rotation_vector.hpp>
#include <formal_eskf/status.hpp>

namespace formal_eskf::runtime
{

/**
 * Filter body-frame specific force, then check its magnitude against gravity.
 * Input is delta_vel / delta_vel_dt in m/s^2, before estimator bias subtraction.
 * Fixed alpha uses a 10 ms nominal interval and 90 ms time constant; it is NOT
 * recomputed per sample. Update cadence remains the caller's responsibility.
 * This is an amplitude diagnostic, not permission to fuse or evidence of rest.
 */
template <typename Linalg> class AccelerometerMagnitude
{
public:
    using value_type = typename Linalg::value_type;
    using vector_type = linalg::Matrix<Linalg, 3U, 1U>;

    struct Result
    {
        Status status = Status::success;
        bool within_gravity_band = false;
        value_type squared_norm{}; // m^2/s^4; valid only on success.
    };

    /** Clear history on source/calibration/frame changes; next valid sample seeds the filter. */
    void reset() { *this = AccelerometerMagnitude{}; }

    bool initialized() const { return m_initialized; }
    vector_type const & filtered_specific_force_b() const { return m_filtered_specific_force_b; }

    /** First valid sample seeds all axes. Failure preserves history and returns no valid diagnostics. */
    [[nodiscard]] Result update(vector_type const & specific_force_b);

private:
    vector_type m_filtered_specific_force_b{};
    bool m_initialized = false;
}; /* end class AccelerometerMagnitude */

template <typename Linalg>
typename AccelerometerMagnitude<Linalg>::Result
AccelerometerMagnitude<Linalg>::update(vector_type const & specific_force_b)
{
    using math_type = typename Linalg::scalar_math_type;
    constexpr auto sample_interval = static_cast<value_type>(0.01);
    constexpr auto time_constant = static_cast<value_type>(0.09);
    constexpr auto alpha = sample_interval / (time_constant + sample_interval);
    constexpr auto gravity = static_cast<value_type>(9.80665);
    constexpr auto lower_limit = gravity * static_cast<value_type>(0.9);
    constexpr auto upper_limit = gravity * static_cast<value_type>(1.1);

    Result result;
    if (!linalg::all_finite(specific_force_b))
    {
        result.status = Status::non_finite_input;
        return result;
    }
    auto const candidate = m_initialized
                               ? m_filtered_specific_force_b + (specific_force_b - m_filtered_specific_force_b) * alpha
                               : specific_force_b;
    auto const squared_norm = linalg::squared_norm(candidate);
    if (!linalg::all_finite(candidate) || !scalar::is_finite<math_type>(squared_norm))
    {
        result.status = Status::non_finite_result;
        return result;
    }

    m_filtered_specific_force_b = candidate;
    m_initialized = true;
    result.squared_norm = squared_norm;
    result.within_gravity_band = squared_norm > lower_limit * lower_limit && squared_norm < upper_limit * upper_limit;
    return result;
}

/**
 * Non-overlapping, rotation-aware gravity observations for gravity aiding.
 * Prediction still consumes each original sample. The caller resets this
 * history on gaps/source changes and never supplies prediction hold values.
 */
template <typename Linalg, std::uint64_t IntervalUs> class AccelerometerSample
{
    static_assert(IntervalUs >= 1000U && IntervalUs <= 50000U);

public:
    using value_type = typename Linalg::value_type;
    using vector_type = linalg::Vector<Linalg, 3U>;
    using magnitude_type = AccelerometerMagnitude<Linalg>;

    struct Result
    {
        Status status = Status::success;
        bool ready = false;
        vector_type specific_force_b{}; // Current interval, not the LPF output; m/s^2.
        std::uint64_t interval_us = 0U;
        int clipping_mask = -1; // Unknown if any constituent sample is unknown.
        typename magnitude_type::Result magnitude{};
    };

    AccelerometerSample() { clear_interval(); }

    /** Discard partial increments AND LPF history, without fabricating an observation. */
    void reset() { *this = AccelerometerSample{}; }

    /** Failure preserves all history; only ready results contain a new LPF evaluation. */
    [[nodiscard]] Result update(ImuSample<Linalg> const & imu, std::uint64_t period_us, int clipping_mask,
                                value_type minimum_quaternion_norm);

private:
    [[nodiscard]] Status integrate(ImuSample<Linalg> const & imu, value_type dt, value_type minimum_quaternion_norm,
                                   vector_type & accumulated) const;
    void clear_interval();

    magnitude_type m_magnitude{};
    vector_type m_delta_velocity{};
    std::uint64_t m_interval_us = 0U;
    std::uint64_t m_samples = 0U;
    int m_clipping_mask = 0;
    double m_average_period_us = 5000.0;
    std::uint64_t m_required_samples = 1U;
    double m_target_period_us = 0.0;
    double m_minimum_period_us = 0.0;
}; /* end class AccelerometerSample */

template <typename Linalg, std::uint64_t IntervalUs> void AccelerometerSample<Linalg, IntervalUs>::clear_interval()
{
    m_delta_velocity = {};
    m_interval_us = 0U;
    m_samples = 0U;
    m_clipping_mask = 0;
    // Whole input samples, nearest nominal period; not an exact-rate resampler.
    m_required_samples =
        std::max(std::uint64_t{1}, static_cast<std::uint64_t>(std::round(IntervalUs / m_average_period_us)));
    m_target_period_us = static_cast<double>(m_required_samples) * m_average_period_us;
    m_minimum_period_us =
        std::max(static_cast<double>(m_required_samples - 1U) * m_average_period_us, m_average_period_us * 0.5);
}

template <typename Linalg, std::uint64_t IntervalUs>
Status AccelerometerSample<Linalg, IntervalUs>::integrate(ImuSample<Linalg> const & imu, value_type dt,
                                                          value_type minimum_quaternion_norm,
                                                          vector_type & accumulated) const
{
    auto const delta_angle = imu.angular_rate_b * dt;
    auto const delta_velocity = imu.specific_force_b * dt;
    if (!linalg::all_finite(delta_angle) || !linalg::all_finite(delta_velocity))
    {
        return Status::non_finite_result;
    }
    so3::UnitQuaternion<Linalg> delta_q;
    auto const status = so3::exp<Linalg>(delta_angle, minimum_quaternion_norm, delta_q);
    if (!succeeded(status))
    {
        return status;
    }
    // Express previous increments in the new body frame. The newest increment
    // uses a trapezoidal endpoint rotation, not an exact continuous-time integral.
    auto const candidate = so3::inverse_rotate(delta_q, m_delta_velocity) +
                           (delta_velocity + so3::inverse_rotate(delta_q, delta_velocity)) * value_type{0.5};
    if (!linalg::all_finite(candidate))
    {
        return Status::non_finite_result;
    }
    accumulated = candidate;
    return Status::success;
}

template <typename Linalg, std::uint64_t IntervalUs>
typename AccelerometerSample<Linalg, IntervalUs>::Result
AccelerometerSample<Linalg, IntervalUs>::update(ImuSample<Linalg> const & imu, std::uint64_t period_us,
                                                int clipping_mask, value_type minimum_quaternion_norm)
{
    Result result;
    if (!linalg::all_finite(imu.specific_force_b) || !linalg::all_finite(imu.angular_rate_b))
    {
        result.status = Status::non_finite_input;
        return result;
    }
    if (period_us == 0U || period_us > 50000U || clipping_mask < -1 || clipping_mask > 7)
    {
        result.status = Status::domain_error;
        return result;
    }
    auto const dt = static_cast<value_type>(period_us) * static_cast<value_type>(1e-6);
    vector_type accumulated;
    result.status = integrate(imu, dt, minimum_quaternion_norm, accumulated);
    if (!succeeded(result.status))
    {
        return result;
    }
    auto const interval_us = m_interval_us + period_us;
    auto const samples = m_samples + 1U;
    auto const mask = m_clipping_mask < 0 || clipping_mask < 0 ? -1 : m_clipping_mask | clipping_mask;
    bool const ready = (samples >= m_required_samples && static_cast<double>(interval_us) > m_minimum_period_us) ||
                       static_cast<double>(interval_us) > m_target_period_us;
    if (ready)
    {
        auto const specific_force =
            accumulated / (static_cast<value_type>(interval_us) * static_cast<value_type>(1e-6));
        if (!linalg::all_finite(specific_force))
        {
            result.status = Status::non_finite_result;
            return result;
        }
        result.magnitude = m_magnitude.update(specific_force);
        result.status = result.magnitude.status;
        if (!succeeded(result.status))
        {
            return result;
        }
        result.ready = true;
        result.specific_force_b = specific_force;
        result.interval_us = interval_us;
        result.clipping_mask = mask;
    }
    m_average_period_us += 0.1 * (static_cast<double>(period_us) - m_average_period_us);
    if (ready)
    {
        clear_interval();
    }
    else
    {
        m_delta_velocity = accumulated;
        m_interval_us = interval_us;
        m_samples = samples;
        m_clipping_mask = mask;
    }
    return result;
}

struct AccelerometerFusionConditions
{
    bool enabled = false;
    bool tilt_aligned = false;
    bool horizontal_aiding = false; // Actual aiding activity, not availability of a GNSS fix.
    bool at_rest = false;           // Caller detection, not inferred from the magnitude check.
    std::array<bool, 3U> clipped{};
};

struct AccelerometerFusionCheck
{
    Status status = Status::success;
    bool active = false;          // Mode condition, before per-sample clipping/innovation checks.
    bool sample_eligible = false; // Before innovation gating; does not mean a correction was performed.
};

/**
 * Stateless gravity-aiding conditions for AHRS/INS. Supply the actual magnitude
 * result and caller conditions for the same sample/horizon, never stale flags.
 * At rest overrides only the magnitude band, not failed evaluation or clipping.
 * Clipping blocks the sample without clearing an otherwise active mode.
 * No innovation check, correction, lifecycle, or estimator/filter state update.
 */
template <typename Linalg>
[[nodiscard]] constexpr AccelerometerFusionCheck
check_accelerometer_fusion_conditions(AccelerometerFusionConditions const & conditions,
                                      typename AccelerometerMagnitude<Linalg>::Result const & magnitude)
{
    AccelerometerFusionCheck result;
    result.status = magnitude.status;
    if (!succeeded(result.status))
    {
        return result;
    }
    result.active = conditions.enabled && (magnitude.within_gravity_band || conditions.at_rest) &&
                    !conditions.horizontal_aiding && conditions.tilt_aligned;
    result.sample_eligible =
        result.active && !(conditions.clipped[0U] || conditions.clipped[1U] || conditions.clipped[2U]);
    return result;
}

/** Eligibility is not fusion success; only fused publishes state and covariance. */
struct AccelerometerFusionResult
{
    AccelerometerFusionCheck conditions{};
    FusionDecision decision = FusionDecision::failed;
    Status status = Status::success;
};

/**
 * Check gravity-fusion prerequisites, then run the unchanged core correction.
 * Pass the CURRENT specific force (m/s^2), not the LPF vector or an increment.
 * AHRS input is externally bias-calibrated; INS subtracts the prior bias once.
 * gravity_n and effective V retain the core's units and validity premises.
 *
 * The caller updates its magnitude filter once per sample, even when fusion is
 * disabled, then supplies that result with the same sample/horizon conditions.
 * It owns cadence, freshness and reset on source/calibration/frame changes.
 * This function never updates or rolls back that history. Do not reuse a stale
 * successful magnitude result after an update failure.
 *
 * Failed magnitude evaluation returns failed with its status; unmet conditions
 * return rejected + success without validating unused correction inputs.
 * Eligible samples call correction; only its success returns fused. Each output
 * may alias its respective whole prior. Rejection/failure preserves both outputs;
 * other inputs/output storage must be disjoint, as in the core.
 * No innovation gate, normalization of the observation, lifecycle or tuning.
 * Eligibility does not establish negligible acceleration or independent noise.
 */
template <typename Linalg>
[[nodiscard]] AccelerometerFusionResult fuse_accelerometer(
    configuration::Ahrs::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 3U, 3U> const & covariance,
    linalg::Vector<Linalg, 3U> const & specific_force_b, linalg::Vector<Linalg, 3U> const & gravity_n,
    linalg::Matrix<Linalg, 3U, 3U> const & V, AccelerometerFusionConditions const & conditions,
    typename AccelerometerMagnitude<Linalg>::Result const & magnitude,
    typename Linalg::value_type minimum_quaternion_norm, configuration::Ahrs::NominalState<Linalg> & state_output,
    linalg::Matrix<Linalg, 3U, 3U> & covariance_output)
{
    AccelerometerFusionResult result;
    result.conditions = check_accelerometer_fusion_conditions<Linalg>(conditions, magnitude);
    result.status = result.conditions.status;
    if (!succeeded(result.status))
    {
        return result;
    }
    if (!result.conditions.sample_eligible)
    {
        result.decision = FusionDecision::rejected;
        return result;
    }
    result.status = correct_accelerometer(state, covariance, specific_force_b, gravity_n, V, minimum_quaternion_norm,
                                          state_output, covariance_output);
    if (succeeded(result.status))
    {
        result.decision = FusionDecision::fused;
    }
    return result;
}

/** INS forwards all six bias-learning permissions unchanged to the core. */
template <typename Linalg>
[[nodiscard]] AccelerometerFusionResult fuse_accelerometer(
    configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15U, 15U> const & covariance,
    linalg::Vector<Linalg, 3U> const & specific_force_b, linalg::Vector<Linalg, 3U> const & gravity_n,
    linalg::Matrix<Linalg, 3U, 3U> const & V, AccelerometerFusionConditions const & conditions,
    typename AccelerometerMagnitude<Linalg>::Result const & magnitude,
    typename Linalg::value_type minimum_quaternion_norm, configuration::Ins::NominalState<Linalg> & state_output,
    linalg::Matrix<Linalg, 15U, 15U> & covariance_output, configuration::Ins::BiasUpdate const & bias_update = {})
{
    AccelerometerFusionResult result;
    result.conditions = check_accelerometer_fusion_conditions<Linalg>(conditions, magnitude);
    result.status = result.conditions.status;
    if (!succeeded(result.status))
    {
        return result;
    }
    if (!result.conditions.sample_eligible)
    {
        result.decision = FusionDecision::rejected;
        return result;
    }
    result.status = correct_accelerometer(state, covariance, specific_force_b, gravity_n, V, minimum_quaternion_norm,
                                          state_output, covariance_output, bias_update);
    if (succeeded(result.status))
    {
        result.decision = FusionDecision::fused;
    }
    return result;
}

} /* end namespace formal_eskf::runtime */
