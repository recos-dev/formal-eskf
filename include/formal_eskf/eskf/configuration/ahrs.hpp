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
 * Rotation-only AHRS ESKF configuration.
 */

#include <cstddef>

#include <formal_eskf/so3/unit_quaternion.hpp>

namespace formal_eskf::configuration
{

/**
 * Rotation-only AHRS with a four-coefficient unit-quaternion nominal state and
 * a three-dimensional local attitude error.
 * q_nb maps body to NED. q_true = q_nb * Exp(delta_theta_b), with delta_theta_b
 * in radians. Gyroscope bias must be calibrated externally for this model.
 */
struct Ahrs
{
    static constexpr std::size_t nominal_state_dimension = 4U;
    static constexpr std::size_t error_state_dimension = 3U;
    static constexpr std::size_t process_noise_dimension = 3U;

    /** Nominal state x = [q_nb]. */
    template <typename Linalg> struct NominalState
    {
        using quaternion_type = so3::UnitQuaternion<Linalg>;

        quaternion_type q_nb{};
    };

    /** Local right-multiplicative attitude error delta_x = [delta_theta_b]. */
    template <typename Linalg> struct ErrorState
    {
        using vector3_type = typename Linalg::template vector_type<3U>;

        vector3_type delta_theta_b{};
    };

    /**
     * Independent-axis continuous-time angular-rate noise intensity, rad^2/s.
     * E[w(t)w(s)^T] = diag(intensity) * delta(t-s); this is neither a standard
     * deviation nor a per-sample variance.
     */
    template <typename Linalg> struct ProcessNoise
    {
        using vector3_type = typename Linalg::template vector_type<3U>;

        vector3_type angular_rate_variance_density{};
    };

    /**
     * Runtime parameters; zero defaults require explicit setup before use.
     * Transitions validate consumed fields; dt_min/dt_max are seconds.
     */
    template <typename Linalg> struct Parameters
    {
        using value_type = typename Linalg::value_type;

        ProcessNoise<Linalg> process_noise{};
        value_type minimum_quaternion_norm{};
        value_type dt_min{};
        value_type dt_max{};
    };
};

} /* end namespace formal_eskf::configuration */
