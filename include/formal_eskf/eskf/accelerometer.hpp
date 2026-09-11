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
 * Accelerometer measurements for AHRS (gravity) and INS (cross-product model).
 */

#include <formal_eskf/eskf/correction.hpp>

namespace formal_eskf
{

namespace detail
{

template <typename Linalg>
[[nodiscard]] linalg::Matrix<Linalg, 3U, 15U>
accelerometer_jacobian_from_terms(linalg::Matrix<Linalg, 3U, 3U> const & Rt, linalg::Matrix<Linalg, 3U, 1U> const & v_b,
                                  linalg::Matrix<Linalg, 3U, 1U> const & omega_b,
                                  linalg::Matrix<Linalg, 3U, 1U> const & g_b) noexcept
{
    auto const W = so3::hat<Linalg>(omega_b);
    auto const U = so3::hat<Linalg>(v_b);
    linalg::Matrix<Linalg, 3U, 15U> H;
    H.template set_block<0U, 3U>(W * Rt);
    H.template set_block<0U, 6U>(W * U - so3::hat<Linalg>(g_b));
    H.template set_block<0U, 9U>(linalg::Matrix<Linalg, 3U, 3U>::identity());
    H.template set_block<0U, 12U>(U);
    return H;
}

} /* end namespace detail */

/**
 * AHRS Jacobian of h = -R(q_nb)^T * gravity_n with respect to the local error
 * in q_true = q_nb * Exp(delta_theta_b). H = hat(h), not the residual derivative.
 * This is an unchecked model evaluation; inputs and arithmetic must be finite.
 */
template <typename Linalg>
[[nodiscard]] linalg::Matrix<Linalg, 3U, 3U>
accelerometer_jacobian(configuration::Ahrs::NominalState<Linalg> const & state,
                       linalg::Matrix<Linalg, 3U, 1U> const & gravity_n) noexcept
{
    return so3::hat<Linalg>(-so3::inverse_rotate(state.q_nb, gravity_n));
}

/**
 * INS Jacobian of h = omega_b x v_b - g_b + b_a, holding measured angular_rate_b
 * and gravity_n fixed. With Rt = R(q_nb)^T, v_b = Rt*v_n,
 * omega_b = angular_rate_b-b_g and g_b = Rt*gravity_n:
 * H = [0, hat(omega_b)*Rt, hat(omega_b)*hat(v_b)-hat(g_b), I, hat(v_b)].
 * Columns follow [delta_p_n, delta_v_n, delta_theta_b, delta_b_a, delta_b_g].
 * In particular, -(delta_b_g x v_b) = hat(v_b)*delta_b_g; the gyro-bias block
 * has a positive sign. The attitude block uses local rotation-vector errors.
 * Inputs and arithmetic must be finite; this evaluation is unchecked.
 */
template <typename Linalg>
[[nodiscard]] linalg::Matrix<Linalg, 3U, 15U>
accelerometer_jacobian(configuration::Ins::NominalState<Linalg> const & state,
                       linalg::Matrix<Linalg, 3U, 1U> const & angular_rate_b,
                       linalg::Matrix<Linalg, 3U, 1U> const & gravity_n) noexcept
{
    auto const Rt = linalg::transpose(so3::to_rotation_matrix(state.q_nb));
    return detail::accelerometer_jacobian_from_terms(Rt, Rt * state.v_n, angular_rate_b - state.b_g, Rt * gravity_n);
}

/**
 * Correct AHRS with externally bias-calibrated body-frame specific_force_b
 * (m/s^2), a fixed NED gravity_n (m/s^2), and effective body-frame measurement
 * covariance V (m^2/s^4). Under negligible navigation-frame translational
 * acceleration, h = -R(q_nb)^T*gravity_n and r = specific_force_b-h.
 * A level stationary sensor reads [0,0,-g] when gravity_n = [0,0,g].
 * Neither vector is normalized. Gravity alone does not observe absolute heading.
 *
 * Calibration, sensor-to-body alignment, IMU-origin/lever-arm compensation,
 * time alignment and deciding when the motion assumption is credible belong
 * to the caller. No motion/innovation gating or acceleration differencing is
 * performed. A zero measured specific force is accepted algebraically, not
 * certified as a valid gravity observation; this is not a free-fall detector.
 * gravity_n must be nonzero and finite; its uncertainty is not a state here.
 * V must account for effective measurement errors under the chosen model and
 * be independent of prior error for the standard correction to be applicable.
 *
 * Uses full three-axis try_correct: Cholesky, Joseph, right-multiplicative Exp
 * injection and covariance reset. P must be symmetric PSD and V symmetric SPD.
 * Non-finite consumed inputs return non_finite_input, zero gravity domain_error,
 * and non-finite model/Jacobian/residual arithmetic non_finite_result. Remaining
 * checks follow try_correct. Outputs commit only on success and may alias their
 * respective inputs. No backend, state augmentation or allocation is added.
 */
template <typename Linalg>
[[nodiscard]] Status try_correct_accelerometer(configuration::Ahrs::NominalState<Linalg> const & state,
                                               linalg::Matrix<Linalg, 3U, 3U> const & covariance,
                                               linalg::Matrix<Linalg, 3U, 1U> const & specific_force_b,
                                               linalg::Matrix<Linalg, 3U, 1U> const & gravity_n,
                                               linalg::Matrix<Linalg, 3U, 3U> const & V,
                                               typename Linalg::value_type minimum_quaternion_norm,
                                               configuration::Ahrs::NominalState<Linalg> & state_output,
                                               linalg::Matrix<Linalg, 3U, 3U> & covariance_output) noexcept
{
    using value_type = typename Linalg::value_type;
    if (!linalg::all_finite(state.q_nb.coefficients()) || !linalg::all_finite(specific_force_b) ||
        !linalg::all_finite(gravity_n))
    {
        return Status::non_finite_input;
    }
    if (!(linalg::max_abs(gravity_n) > value_type{0}))
    {
        return Status::domain_error;
    }
    auto const h = -so3::inverse_rotate(state.q_nb, gravity_n);
    auto const r = specific_force_b - h;
    if (!linalg::all_finite(h) || !linalg::all_finite(r))
    {
        return Status::non_finite_result;
    }
    auto const H = so3::hat<Linalg>(h);
    return try_correct(state, covariance, r, H, V, minimum_quaternion_norm, state_output, covariance_output);
}

/**
 * Correct INS using SICE 2023, "A Computationally Efficient GNSS/INS Design of
 * Multirotor based on Error-state Kalman Filter", (25)--(27). Equation (26) is
 * a_b = dot(v_b) + omega_b x v_b; this model assumes dot(v_b) approximately zero,
 * not zero navigation-frame acceleration. With v_b = R(q_nb)^T*v_n:
 * h = (angular_rate_b-b_g) x v_b - R(q_nb)^T*gravity_n + b_a.
 * We retain raw specific_force_b as the observation, hence +b_a in h. Do not
 * subtract the state's accelerometer or gyroscope bias from the inputs again.
 * Specific force, gravity and V have the AHRS units; angular_rate_b is in rad/s.
 *
 * V is the EFFECTIVE residual-noise covariance, not just accelerometer noise.
 * At first order, n_eff = n_a + hat(v_b)*n_g + dot(v_b). For independent additive
 * accel/gyro noises and the ideal motion assumption, the caller can form
 * V = V_a + hat(v_b)*V_g*hat(v_b)^T. Uncertain velocity and biases already in P
 * enter through H, not as an extra independent noise term. Model/calibration
 * errors and any cross-source correlations require an appropriate effective V.
 * A persistent nonzero dot(v_b) is a model violation, not merely white noise.
 *
 * This first version uses the standard try_correct gain and Joseph update:
 * Cov(prior error,n_eff) is assumed zero. Reusing IMU samples from prediction
 * generally violates that assumption; doing so is an explicit statistical
 * approximation, NOT an implemented correlated-noise update. Supplying V alone
 * does not remove the correlation. No differencing or automatic noise assembly
 * is performed. All state gains are retained; numerical checks, caller-owned
 * motion gating and failure atomicity follow the AHRS overload.
 */
template <typename Linalg>
[[nodiscard]] Status try_correct_accelerometer(
    configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15U, 15U> const & covariance,
    linalg::Matrix<Linalg, 3U, 1U> const & specific_force_b, linalg::Matrix<Linalg, 3U, 1U> const & angular_rate_b,
    linalg::Matrix<Linalg, 3U, 1U> const & gravity_n, linalg::Matrix<Linalg, 3U, 3U> const & V,
    typename Linalg::value_type minimum_quaternion_norm, configuration::Ins::NominalState<Linalg> & state_output,
    linalg::Matrix<Linalg, 15U, 15U> & covariance_output) noexcept
{
    using value_type = typename Linalg::value_type;
    if (!linalg::all_finite(state.q_nb.coefficients()) || !linalg::all_finite(state.v_n) ||
        !linalg::all_finite(state.b_a) || !linalg::all_finite(state.b_g) || !linalg::all_finite(specific_force_b) ||
        !linalg::all_finite(angular_rate_b) || !linalg::all_finite(gravity_n))
    {
        return Status::non_finite_input;
    }
    if (!(linalg::max_abs(gravity_n) > value_type{0}))
    {
        return Status::domain_error;
    }
    auto const Rt = linalg::transpose(so3::to_rotation_matrix(state.q_nb));
    auto const v_b = Rt * state.v_n;
    auto const g_b = Rt * gravity_n;
    auto const omega_b = angular_rate_b - state.b_g;
    if (!linalg::all_finite(v_b) || !linalg::all_finite(g_b) || !linalg::all_finite(omega_b))
    {
        return Status::non_finite_result;
    }
    auto const h = linalg::cross(omega_b, v_b) - g_b + state.b_a;
    auto const r = specific_force_b - h;
    auto const H = detail::accelerometer_jacobian_from_terms(Rt, v_b, omega_b, g_b);
    if (!linalg::all_finite(h) || !linalg::all_finite(r) || !linalg::all_finite(H))
    {
        return Status::non_finite_result;
    }
    return try_correct(state, covariance, r, H, V, minimum_quaternion_norm, state_output, covariance_output);
}

} /* end namespace formal_eskf */
