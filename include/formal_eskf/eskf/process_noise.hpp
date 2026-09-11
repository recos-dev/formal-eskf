/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

/**
 * @file
 * Sampled IMU noise and continuous bias random walk, discretized per IMU step.
 */

#include <formal_eskf/eskf/configuration/ahrs.hpp>
#include <formal_eskf/eskf/configuration/ins.hpp>
#include <formal_eskf/linalg/operations.hpp>

namespace formal_eskf
{

namespace detail
{

template <typename Linalg>
[[nodiscard]] Status validate_noise_variances(linalg::Matrix<Linalg, 3U, 1U> const & variances) noexcept
{
    if (!linalg::all_finite(variances))
    {
        return Status::non_finite_input;
    }
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        if (variances(axis) < typename Linalg::value_type{0})
        {
            return Status::domain_error;
        }
    }
    return Status::success;
}

template <typename Math> [[nodiscard]] Status validate_noise_step(typename Math::value_type dt) noexcept
{
    if (!scalar::is_finite<Math>(dt))
    {
        return Status::non_finite_input;
    }
    return dt > typename Math::value_type{0} ? Status::success : Status::out_of_range;
}

} /* end namespace detail */

/**
 * Construct Q_i = diag(angular_rate_variance * dt^2) for rotation-only AHRS.
 *
 * Implements the attitude-noise block of SICE 2023, "A Computationally
 * Efficient GNSS/INS Design of Multirotor based on Error-state Kalman Filter",
 * equation (15). Requires finite, positive dt and finite, nonnegative noise
 * coefficients. Zero noise is valid. Output is unchanged on failure.
 * This operation does not enforce application-specific dt_min/dt_max.
 */
template <typename Linalg>
[[nodiscard]] Status try_discretize_process_noise(configuration::Ahrs::ProcessNoise<Linalg> const & noise,
                                                  typename Linalg::value_type dt,
                                                  linalg::Matrix<Linalg, 3U, 3U> & output) noexcept
{
    Status const step_status = detail::validate_noise_step<typename Linalg::scalar_math_type>(dt);
    if (!succeeded(step_status))
    {
        return step_status;
    }
    Status const noise_status = detail::validate_noise_variances(noise.angular_rate_variance);
    if (!succeeded(noise_status))
    {
        return noise_status;
    }

    linalg::Matrix<Linalg, 3U, 3U> candidate;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        // Evaluate left to right; avoid overflowing dt*dt for zero noise.
        candidate.set(axis, axis, (noise.angular_rate_variance(axis) * dt) * dt);
    }
    if (!linalg::all_finite(candidate))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

/**
 * Construct the INS 12x12 Q_i for body-axis noise impulses in the order
 * [accelerometer measurement, gyroscope measurement, b_a drive, b_g drive].
 * Measurement variances scale by dt^2; bias driving-noise intensities by dt,
 * following SICE 2023 equations (15) and (18).
 *
 * Q_i is defined in noise-source coordinates and is paired with F_i = [B C]
 * from Sola equation (451) and Appendix E.2.1's discussion after (457).
 * Its first block is Sigma_a_b * dt^2, where Sigma_a_b is the diagonal
 * body-frame accelerometer covariance. It is NOT yet the navigation-frame
 * velocity covariance V_i; injection produces V_i = R * Sigma_a_b * R^T * dt^2.
 * Isotropic noise recovers V_i = variance * dt^2 * I without a separate mode.
 * Cross-axis and cross-source noise correlations are not represented.
 * Validation and failure behavior follow the AHRS overload.
 *
 * @see https://arxiv.org/pdf/1711.02508#page=93
 */
template <typename Linalg>
[[nodiscard]] Status try_discretize_process_noise(configuration::Ins::ProcessNoise<Linalg> const & noise,
                                                  typename Linalg::value_type dt,
                                                  linalg::Matrix<Linalg, 12U, 12U> & output) noexcept
{
    Status const step_status = detail::validate_noise_step<typename Linalg::scalar_math_type>(dt);
    if (!succeeded(step_status))
    {
        return step_status;
    }
    using vector3_type = typename Linalg::template vector_type<3U>;
    vector3_type const * variance_groups[] = {&noise.specific_force_variance, &noise.angular_rate_variance,
                                              &noise.accelerometer_bias_random_walk_variance_density,
                                              &noise.gyroscope_bias_random_walk_variance_density};
    for (auto const * variances : variance_groups)
    {
        Status const status = detail::validate_noise_variances(*variances);
        if (!succeeded(status))
        {
            return status;
        }
    }

    linalg::Matrix<Linalg, 12U, 12U> candidate;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        candidate.set(axis, axis, (noise.specific_force_variance(axis) * dt) * dt);
        candidate.set(3U + axis, 3U + axis, (noise.angular_rate_variance(axis) * dt) * dt);
        candidate.set(6U + axis, 6U + axis, noise.accelerometer_bias_random_walk_variance_density(axis) * dt);
        candidate.set(9U + axis, 9U + axis, noise.gyroscope_bias_random_walk_variance_density(axis) * dt);
    }
    if (!linalg::all_finite(candidate))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

} /* end namespace formal_eskf */
