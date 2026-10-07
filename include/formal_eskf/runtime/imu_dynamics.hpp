/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <limits>

#include <formal_eskf/eskf/types.hpp>
#include <formal_eskf/linalg/operations.hpp>
#include <formal_eskf/scalar/math.hpp>
#include <formal_eskf/status.hpp>

namespace formal_eskf::runtime
{

template <typename Scalar> struct ImuDynamicsParameters
{
    Scalar maximum_specific_force{}; // m/s^2; includes the stationary gravity response.
    Scalar maximum_angular_rate{};   // rad/s. Both thresholds require explicit configuration.
};

template <typename Scalar> struct ImuDynamicsResult
{
    Status status = Status::success;
    bool high_dynamics = true;
    // Diagnostics are valid only on success; zero placeholders on failure.
    Scalar specific_force_squared_norm{}; // m^2/s^4.
    Scalar angular_rate_squared_norm{};   // rad^2/s^2.
};

/**
 * Check instantaneous bias-corrected IMU magnitudes against configured limits.
 * Inputs are body-frame rates, not integrated increments; no gravity removal.
 * Compare squared norms to avoid square roots. Equality is not high dynamics.
 * Positive thresholds must have finite, normal squares in the selected scalar.
 * Failure returns high_dynamics=true, not a successful motion classification.
 *
 * Feed successful high_dynamics to ImuBiasLearning; it owns recovery timing.
 * On failure, do not use old diagnostics or authorize learning: mark both sensor
 * health conditions false for that policy update. Reset the learning monitor on
 * changes to these thresholds, calibration or sensor/frame identity.
 * A low magnitude is not evidence of IMU health, rest, or bias observability.
 */
template <typename Linalg>
[[nodiscard]] ImuDynamicsResult<typename Linalg::value_type>
check_imu_dynamics(ImuSample<Linalg> const & imu, linalg::Matrix<Linalg, 3U, 1U> const & accelerometer_bias_b,
                   linalg::Matrix<Linalg, 3U, 1U> const & gyroscope_bias_b,
                   ImuDynamicsParameters<typename Linalg::value_type> const & parameters) noexcept
{
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    ImuDynamicsResult<value_type> result;
    if (!linalg::all_finite(imu.specific_force_b) || !linalg::all_finite(imu.angular_rate_b) ||
        !linalg::all_finite(accelerometer_bias_b) || !linalg::all_finite(gyroscope_bias_b) ||
        !scalar::is_finite<math_type>(parameters.maximum_specific_force) ||
        !scalar::is_finite<math_type>(parameters.maximum_angular_rate))
    {
        result.status = Status::non_finite_input;
        return result;
    }
    if (!(parameters.maximum_specific_force > value_type{0}) || !(parameters.maximum_angular_rate > value_type{0}))
    {
        result.status = Status::domain_error;
        return result;
    }
    auto const force_limit_squared = parameters.maximum_specific_force * parameters.maximum_specific_force;
    auto const rate_limit_squared = parameters.maximum_angular_rate * parameters.maximum_angular_rate;
    if (!scalar::is_finite<math_type>(force_limit_squared) || !scalar::is_finite<math_type>(rate_limit_squared) ||
        force_limit_squared < std::numeric_limits<value_type>::min() ||
        rate_limit_squared < std::numeric_limits<value_type>::min())
    {
        result.status = Status::domain_error;
        return result;
    }

    auto const specific_force_b = imu.specific_force_b - accelerometer_bias_b;
    auto const angular_rate_b = imu.angular_rate_b - gyroscope_bias_b;
    if (!linalg::all_finite(specific_force_b) || !linalg::all_finite(angular_rate_b))
    {
        result.status = Status::non_finite_result;
        return result;
    }
    auto const force_squared = linalg::squared_norm(specific_force_b);
    auto const rate_squared = linalg::squared_norm(angular_rate_b);
    if (!scalar::is_finite<math_type>(force_squared) || !scalar::is_finite<math_type>(rate_squared))
    {
        result.status = Status::non_finite_result;
        return result;
    }
    result.specific_force_squared_norm = force_squared;
    result.angular_rate_squared_norm = rate_squared;
    result.high_dynamics = force_squared > force_limit_squared || rate_squared > rate_limit_squared;
    return result;
}

} /* end namespace formal_eskf::runtime */
