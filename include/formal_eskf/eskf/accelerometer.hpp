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
 * Accelerometer gravity observations for AHRS and INS.
 */

#include <formal_eskf/eskf/correction.hpp>

namespace formal_eskf
{

namespace detail
{

template <typename Linalg>
[[nodiscard]] linalg::Matrix<Linalg, 3U, 15U>
accelerometer_jacobian_from_prediction(linalg::Matrix<Linalg, 3U, 1U> const & h) noexcept
{
    linalg::Matrix<Linalg, 3U, 15U> H;
    H.template set_block<0U, 6U>(so3::hat<Linalg>(h));
    return H;
}

} /* end namespace detail */

/**
 * AHRS Jacobian of h = -R(q_nb)^T * gravity_n, with respect to the local
 * error in q_true = q_nb * Exp(delta_theta_b). H = hat(h), not the residual derivative.
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
 * INS Jacobian with columns [delta_p_n, delta_v_n, delta_theta_b, delta_b_a,
 * delta_b_g]. For h = -R(q_nb)^T*gravity_n, H = [0, 0, hat(h), 0, 0].
 * Bias preprocessing is held fixed; this is not the joint derivative of h+b_a.
 * Uses the local right-multiplicative attitude error. This is unchecked;
 * the consumed quaternion/gravity inputs and arithmetic must be finite.
 */
template <typename Linalg>
[[nodiscard]] linalg::Matrix<Linalg, 3U, 15U>
accelerometer_jacobian(configuration::Ins::NominalState<Linalg> const & state,
                       linalg::Matrix<Linalg, 3U, 1U> const & gravity_n) noexcept
{
    return detail::accelerometer_jacobian_from_prediction(-so3::inverse_rotate(state.q_nb, gravity_n));
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
 * Correct INS with a gravity observation, assuming negligible navigation-frame
 * translational acceleration. Accept raw specific_force_b and subtract the
 * prior accelerometer bias exactly once:
 * z = specific_force_b - state.b_a, h = -R(q_nb)^T*gravity_n, r = z-h.
 * Neither vector is normalized; units and gravity conventions follow AHRS.
 *
 * Hold bias preprocessing fixed in the linearization:
 * H = [0, 0, hat(h), 0, 0]. Omitting the direct bias derivative is an explicit
 * approximation. Zero bias columns do not freeze bias gains through cross P;
 * the existing bias_update permissions control learning and default to enabled.
 *
 * V is the effective residual-noise covariance. Fixed bias preprocessing does
 * not remove bias uncertainty or its correlation with the prior. Reusing IMU
 * samples from prediction likewise violates the standard correction's
 * independent-noise assumption; supplying V alone does not remove correlation.
 * No dynamic-acceleration compensation, automatic noise assembly, differencing
 * or correlated-noise update is performed. Motion gating, numerical checks and
 * failure atomicity follow the AHRS overload.
 */
template <typename Linalg>
[[nodiscard]] Status try_correct_accelerometer(
    configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15U, 15U> const & covariance,
    linalg::Matrix<Linalg, 3U, 1U> const & specific_force_b, linalg::Matrix<Linalg, 3U, 1U> const & gravity_n,
    linalg::Matrix<Linalg, 3U, 3U> const & V, typename Linalg::value_type minimum_quaternion_norm,
    configuration::Ins::NominalState<Linalg> & state_output, linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
    configuration::Ins::BiasUpdate const & bias_update = {}) noexcept
{
    using value_type = typename Linalg::value_type;
    if (!linalg::all_finite(state.q_nb.coefficients()) || !linalg::all_finite(state.v_n) ||
        !linalg::all_finite(state.b_a) || !linalg::all_finite(state.b_g) || !linalg::all_finite(specific_force_b) ||
        !linalg::all_finite(gravity_n))
    {
        return Status::non_finite_input;
    }
    if (!(linalg::max_abs(gravity_n) > value_type{0}))
    {
        return Status::domain_error;
    }
    auto const h = -so3::inverse_rotate(state.q_nb, gravity_n);
    auto const z = specific_force_b - state.b_a;
    if (!linalg::all_finite(z))
    {
        return Status::non_finite_result;
    }
    auto const r = z - h;
    auto const H = detail::accelerometer_jacobian_from_prediction(h);
    if (!linalg::all_finite(h) || !linalg::all_finite(r) || !linalg::all_finite(H))
    {
        return Status::non_finite_result;
    }
    return try_correct(state, covariance, r, H, V, minimum_quaternion_norm, state_output, covariance_output,
                       bias_update);
}

} /* end namespace formal_eskf */
