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
 * Full inertial-navigation ESKF configuration.
 */

#include <cstddef>

#include <formal_eskf/so3/unit_quaternion.hpp>

namespace formal_eskf::configuration
{

/**
 * Full inertial-navigation state with position, velocity, attitude, and IMU
 * biases.
 *
 * Navigation coordinates are NED. q_nb maps body to navigation and uses
 * scalar-first [q0, q1, q2, q3] coefficients. The local attitude error is
 * right-multiplicative: q_true = q_nb * Exp(delta_theta_b).
 */
struct Ins
{
    static constexpr std::size_t nominal_state_dimension = 16U;
    static constexpr std::size_t error_state_dimension = 15U;
    static constexpr std::size_t process_noise_dimension = 12U;

    /** Nominal state x = [p_n, v_n, q_nb, b_a, b_g]. */
    template <typename Linalg> struct NominalState
    {
        using vector3_type = typename Linalg::template vector_type<3U>;
        using quaternion_type = so3::UnitQuaternion<Linalg>;

        vector3_type p_n{}; // Navigation position, m.
        vector3_type v_n{}; // Navigation velocity, m/s.
        quaternion_type q_nb{};
        vector3_type b_a{}; // Accelerometer bias in body coordinates, m/s^2.
        vector3_type b_g{}; // Gyroscope bias in body coordinates, rad/s.
    };

    /** Local error state delta_x = [delta_p_n, delta_v_n, delta_theta_b, delta_b_a, delta_b_g]. */
    template <typename Linalg> struct ErrorState
    {
        using vector3_type = typename Linalg::template vector_type<3U>;

        vector3_type delta_p_n{};
        vector3_type delta_v_n{};
        vector3_type delta_theta_b{};
        vector3_type delta_b_a{};
        vector3_type delta_b_g{};
    };

    /**
     * Independent body-axis noise parameters with the time scaling of
     * SICE 2023 equation (15). Axis variances may differ; covariance prediction
     * uses Sola's general noise mapping rather than assuming isotropic noise.
     * Measurement variances describe a sampled error held over one IMU step;
     * they are multiplied by dt^2, not squared again. Bias random-walk
     * intensities drive bias derivatives and are multiplied by dt:
     * E[w(t)w(s)^T] = diag(intensity) * delta(t-s).
     * All coefficients must be finite and nonnegative; zero noise is allowed.
     */
    template <typename Linalg> struct ProcessNoise
    {
        using vector3_type = typename Linalg::template vector_type<3U>;

        vector3_type specific_force_variance{};                         // m^2/s^4, per sample.
        vector3_type angular_rate_variance{};                           // rad^2/s^2, per sample.
        vector3_type accelerometer_bias_random_walk_variance_density{}; // m^2/s^5.
        vector3_type gyroscope_bias_random_walk_variance_density{};     // rad^2/s^3.
    };

    /**
     * Runtime parameters; zero defaults require explicit setup before use.
     * Transitions validate consumed fields. gravity_n is [0, 0, +g] in m/s^2;
     * dt_min/dt_max are seconds. Gravity is fixed, not an estimated state.
     */
    template <typename Linalg> struct Parameters
    {
        using value_type = typename Linalg::value_type;
        using vector3_type = typename Linalg::template vector_type<3U>;

        vector3_type gravity_n{};
        ProcessNoise<Linalg> process_noise{};
        value_type minimum_quaternion_norm{};
        value_type dt_min{};
        value_type dt_max{};
    };
};

} /* end namespace formal_eskf::configuration */
