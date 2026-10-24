/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <formal_eskf/eskf/configuration/ins.hpp>
#include <formal_eskf/linalg/operations.hpp>

namespace formal_eskf::runtime
{

/**
 * Select per-step INS bias driving noise using the prior covariance.
 * Keep an axis's intensity only while its prior bias variance is strictly below
 * the corresponding sampled IMU variance. Equality inhibits the increment;
 * this does not clamp P or depend on correction's bias-learning permissions.
 *
 * INS bias transition rows are identity and measurement noise has no bias rows,
 * so these prior diagonals equal their propagated values before bias noise.
 * Covariance must be symmetric PSD; only its six bias diagonals are consumed.
 * All consumed values must be finite and nonnegative, including noise that will
 * be inhibited. The core retains its existing Q_bias = intensity * dt scaling.
 * Success may publish in place over noise; every failure preserves output.
 * Derive each step from the original noise, so inhibition can later be released.
 */
template <typename Linalg>
[[nodiscard]] Status try_inhibit_bias_process_noise(linalg::Matrix<Linalg, 15U, 15U> const & covariance,
                                                    configuration::Ins::ProcessNoise<Linalg> const & noise,
                                                    configuration::Ins::ProcessNoise<Linalg> & output) noexcept
{
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        value_type const coefficients[] = {covariance(9U + axis, 9U + axis),
                                           covariance(12U + axis, 12U + axis),
                                           noise.specific_force_variance(axis),
                                           noise.angular_rate_variance(axis),
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
        if (covariance(9U + axis, 9U + axis) >= noise.specific_force_variance(axis))
        {
            candidate.accelerometer_bias_random_walk_variance_density.set(axis, value_type{0});
        }
        if (covariance(12U + axis, 12U + axis) >= noise.angular_rate_variance(axis))
        {
            candidate.gyroscope_bias_random_walk_variance_density.set(axis, value_type{0});
        }
    }
    output = candidate;
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
