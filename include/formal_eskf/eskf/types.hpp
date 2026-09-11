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
 * Compile-time ESKF configuration abstraction and common sensor types.
 */

#include <cstddef>

namespace formal_eskf
{

/**
 * Uncorrected IMU readings in body coordinates: specific force in m/s^2 and
 * angular rate in rad/s. These are rates, not integrated increments. A level,
 * stationary, bias-free sensor aligned with NED reads [0, 0, -g] and [0, 0, 0].
 */
template <typename Linalg> struct ImuSample
{
    using vector3_type = typename Linalg::template vector_type<3U>;

    vector3_type specific_force_b{};
    vector3_type angular_rate_b{};
};

/**
 * Resolve the fixed types and dimensions supplied by an ESKF configuration.
 *
 * A configuration owns the meanings of its nominal state, local error state,
 * process noise, and runtime parameters. The linear-algebra backend remains an
 * independent compile-time choice.
 */
template <typename Linalg, typename Configuration> struct EskfTypes
{
    using linalg_type = Linalg;
    using configuration_type = Configuration;
    using value_type = typename linalg_type::value_type;

    static constexpr std::size_t nominal_state_dimension = configuration_type::nominal_state_dimension;
    static constexpr std::size_t error_state_dimension = configuration_type::error_state_dimension;
    static constexpr std::size_t process_noise_dimension = configuration_type::process_noise_dimension;

    static_assert(nominal_state_dimension > 0U);
    static_assert(error_state_dimension > 0U);
    static_assert(process_noise_dimension > 0U);

    using nominal_state_type = typename configuration_type::template NominalState<linalg_type>;
    using error_state_type = typename configuration_type::template ErrorState<linalg_type>;
    using error_covariance_type =
        typename linalg_type::template matrix_type<error_state_dimension, error_state_dimension>;
    using imu_sample_type = ImuSample<linalg_type>;
    using process_noise_type = typename configuration_type::template ProcessNoise<linalg_type>;
    using process_noise_covariance_type =
        typename linalg_type::template matrix_type<process_noise_dimension, process_noise_dimension>;
    using parameter_type = typename configuration_type::template Parameters<linalg_type>;
};

} /* end namespace formal_eskf */
