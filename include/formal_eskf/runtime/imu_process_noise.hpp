/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <array>

#include <formal_eskf/eskf/configuration/ins.hpp>
#include <formal_eskf/scalar/math.hpp>

namespace formal_eskf::runtime
{

struct ImuNoiseConditions
{
    bool bad_vertical_acceleration = false;       // Preceding estimator-horizon verdict.
    std::array<bool, 3U> accelerometer_clipped{}; // Known body-axis flags for this IMU interval.
};

/**
 * Replace sampled accelerometer variance on faulty/clipped body axes.
 * A vertical fault selects every axis; otherwise clipping selects each axis.
 * fault_variance is already squared (m^2/s^4), not a noise density or bias
 * driving intensity. Selection replaces the value, even if the old one is larger.
 * Preserve gyro variance and both bias intensities. Apply bias-noise inhibition
 * afterward, using these selected measurement variances as its thresholds.
 *
 * All noise coefficients and fault_variance must be finite and nonnegative,
 * including coefficients being replaced. Zero is allowed. Failure preserves
 * output; success permits a whole-noise alias. Start each step from configured
 * noise, not the preceding selected output. The caller owns metadata availability.
 */
template <typename Linalg>
[[nodiscard]] Status try_select_imu_process_noise(configuration::Ins::ProcessNoise<Linalg> const & noise,
                                                  ImuNoiseConditions const & conditions,
                                                  typename Linalg::value_type fault_variance,
                                                  configuration::Ins::ProcessNoise<Linalg> & output) noexcept
{
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    if (!scalar::is_finite<math_type>(fault_variance))
    {
        return Status::non_finite_input;
    }
    if (fault_variance < value_type{0})
    {
        return Status::domain_error;
    }
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        value_type const coefficients[] = {noise.specific_force_variance(axis), noise.angular_rate_variance(axis),
                                           noise.accelerometer_bias_random_walk_variance_density(axis),
                                           noise.gyroscope_bias_random_walk_variance_density(axis)};
        for (auto const coefficient : coefficients)
        {
            if (!scalar::is_finite<math_type>(coefficient))
            {
                return Status::non_finite_input;
            }
            if (coefficient < value_type{0})
            {
                return Status::domain_error;
            }
        }
    }
    auto candidate = noise;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        if (conditions.bad_vertical_acceleration || conditions.accelerometer_clipped[axis])
        {
            candidate.specific_force_variance.set(axis, fault_variance);
        }
    }
    output = candidate;
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
