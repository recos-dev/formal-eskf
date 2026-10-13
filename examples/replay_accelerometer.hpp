/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <formal_eskf/eskf/types.hpp>
#include <formal_eskf/runtime/accelerometer_magnitude.hpp>
#include <formal_eskf/so3/operations.hpp>
#include <formal_eskf/so3/rotation_vector.hpp>

namespace formal_eskf::example
{

/**
 * Non-overlapping, rotation-aware gravity observations for the host replay.
 * Prediction still consumes each original sample. The caller resets this
 * history on gaps/source changes and never supplies prediction hold values.
 */
template <typename Linalg, std::uint64_t IntervalUs> class ReplayAccelerometer
{
    static_assert(IntervalUs >= 1000U && IntervalUs <= 50000U);

public:
    using value_type = typename Linalg::value_type;
    using vector_type = linalg::Matrix<Linalg, 3U, 1U>;
    using magnitude_type = runtime::AccelerometerMagnitude<Linalg>;

    struct Result
    {
        Status status = Status::success;
        bool ready = false;
        vector_type specific_force_b{}; // Current interval, not the LPF output; m/s^2.
        std::uint64_t interval_us = 0U;
        int clipping_mask = -1; // Unknown if any constituent sample is unknown.
        typename magnitude_type::Result magnitude{};
    };

    ReplayAccelerometer() { clear_interval(); }

    /** Discard partial increments AND LPF history, without fabricating an observation. */
    void reset() { *this = ReplayAccelerometer{}; }

    /** Failure preserves all history; only ready results contain a new LPF evaluation. */
    [[nodiscard]] Result update(ImuSample<Linalg> const & imu, std::uint64_t period_us, int clipping_mask,
                                value_type minimum_quaternion_norm);

private:
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
}; /* end class ReplayAccelerometer */

template <typename Linalg, std::uint64_t IntervalUs> void ReplayAccelerometer<Linalg, IntervalUs>::clear_interval()
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
typename ReplayAccelerometer<Linalg, IntervalUs>::Result
ReplayAccelerometer<Linalg, IntervalUs>::update(ImuSample<Linalg> const & imu, std::uint64_t period_us,
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
    auto const delta_angle = imu.angular_rate_b * dt;
    auto const delta_velocity = imu.specific_force_b * dt;
    if (!linalg::all_finite(delta_angle) || !linalg::all_finite(delta_velocity))
    {
        result.status = Status::non_finite_result;
        return result;
    }
    so3::UnitQuaternion<Linalg> delta_q;
    result.status = so3::try_exp<Linalg>(delta_angle, minimum_quaternion_norm, delta_q);
    if (!succeeded(result.status))
    {
        return result;
    }
    // Express previous increments in the new body frame. The newest increment
    // uses a trapezoidal endpoint rotation, not an exact continuous-time integral.
    auto const accumulated = so3::inverse_rotate(delta_q, m_delta_velocity) +
                             (delta_velocity + so3::inverse_rotate(delta_q, delta_velocity)) * value_type{0.5};
    auto const interval_us = m_interval_us + period_us;
    auto const samples = m_samples + 1U;
    auto const mask = m_clipping_mask < 0 || clipping_mask < 0 ? -1 : m_clipping_mask | clipping_mask;
    if (!linalg::all_finite(accumulated))
    {
        result.status = Status::non_finite_result;
        return result;
    }
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

} /* end namespace formal_eskf::example */
