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
 * Horizontal and three-dimensional navigation-frame velocity measurements for INS.
 */

#include <formal_eskf/eskf/correction.hpp>

namespace formal_eskf
{

/**
 * Error-state Jacobian of h(state) = [v_N, v_E]^T, not of the residual.
 * Selects columns 3 and 4 of the INS error state
 * [delta_p_n, delta_v_n, delta_theta_b, delta_b_a, delta_b_g].
 * No vertical velocity observation is introduced.
 */
template <typename Linalg> [[nodiscard]] linalg::Matrix<Linalg, 2U, 15U> horizontal_velocity_jacobian() noexcept
{
    linalg::Matrix<Linalg, 2U, 15U> H;
    H.template set_block<0U, 3U>(linalg::Matrix<Linalg, 2U, 2U>::identity());
    return H;
}

/**
 * Error-state Jacobian of h(state) = v_n, not of the residual z_v_n - v_n.
 * H = [0, I, 0, 0, 0] follows the INS ordering
 * [delta_p_n, delta_v_n, delta_theta_b, delta_b_a, delta_b_g].
 * This affine observation has a constant, exact Jacobian.
 */
template <typename Linalg> [[nodiscard]] linalg::Matrix<Linalg, 3U, 15U> velocity_jacobian() noexcept
{
    linalg::Matrix<Linalg, 3U, 15U> H;
    H.template set_block<0U, 3U>(linalg::Matrix<Linalg, 3U, 3U>::identity());
    return H;
}

/**
 * Correct INS with a velocity z_v_n in m/s and its noise covariance V in m^2/s^2.
 * The measurement describes the same body/IMU origin as state.v_n, relative to
 * the same local navigation frame, expressed in NED axes and at the time of
 * the supplied prior state and covariance. It is linear velocity, not body-axis
 * velocity, angular rate, scalar speed or an integrated displacement.
 * Frame conversion, lever-arm compensation and time alignment are caller
 * responsibilities; this operation cannot check them.
 *
 * Forms r = z_v_n - state.v_n and H = [0, I, 0, 0, 0], then delegates to
 * try_correct for Cholesky, Joseph update, error injection and covariance reset.
 * Full V, including cross-axis correlations, is passed through unchanged.
 * Velocity can correct other state components through prior cross-covariances,
 * but this model does not directly observe absolute position.
 *
 * P must be symmetric PSD, V symmetric SPD, and the prior error mean zero, as
 * required by try_correct. Its input checks and failure statuses apply here.
 * Non-finite velocity inputs return non_finite_input; a non-finite residual
 * computed from finite velocities returns non_finite_result. All outputs remain
 * unchanged on failure and may alias their respective state/covariance inputs.
 * No innovation gating, delayed fusion or dynamic allocation is introduced.
 */
template <typename Linalg>
[[nodiscard]] Status
try_correct_velocity(configuration::Ins::NominalState<Linalg> const & state,
                     linalg::Matrix<Linalg, 15U, 15U> const & covariance, linalg::Matrix<Linalg, 3U, 1U> const & z_v_n,
                     linalg::Matrix<Linalg, 3U, 3U> const & V, typename Linalg::value_type minimum_quaternion_norm,
                     configuration::Ins::NominalState<Linalg> & state_output,
                     linalg::Matrix<Linalg, 15U, 15U> & covariance_output) noexcept
{
    if (!linalg::all_finite(state.v_n) || !linalg::all_finite(z_v_n))
    {
        return Status::non_finite_input;
    }
    auto const r = z_v_n - state.v_n;
    if (!linalg::all_finite(r))
    {
        return Status::non_finite_result;
    }
    return try_correct(state, covariance, r, velocity_jacobian<Linalg>(), V, minimum_quaternion_norm, state_output,
                       covariance_output);
}

/**
 * Correct INS with horizontal velocity z_v_ne = [v_N, v_E]^T in m/s and its
 * 2x2 noise covariance V in m^2/s^2. No v_D measurement is required. Frame,
 * origin, time, covariance preconditions and failure/aliasing semantics follow
 * try_correct_velocity. Both components must describe the same measurement time.
 *
 * Forms r = z_v_ne - state.v_n.segment<0, 2>(). Full V is retained, and the
 * full INS state/covariance is corrected through prior cross-covariances;
 * this is not an independent horizontal filter or a gain mask.
 *
 * Use this OR try_correct_velocity for a given velocity sample, not both:
 * fusing the same horizontal observations twice would double-count them.
 * Separate observation calls do not represent cross-source noise correlations.
 */
template <typename Linalg>
[[nodiscard]] Status try_correct_horizontal_velocity(configuration::Ins::NominalState<Linalg> const & state,
                                                     linalg::Matrix<Linalg, 15U, 15U> const & covariance,
                                                     linalg::Matrix<Linalg, 2U, 1U> const & z_v_ne,
                                                     linalg::Matrix<Linalg, 2U, 2U> const & V,
                                                     typename Linalg::value_type minimum_quaternion_norm,
                                                     configuration::Ins::NominalState<Linalg> & state_output,
                                                     linalg::Matrix<Linalg, 15U, 15U> & covariance_output) noexcept
{
    if (!linalg::all_finite(state.v_n) || !linalg::all_finite(z_v_ne))
    {
        return Status::non_finite_input;
    }
    auto const r = z_v_ne - state.v_n.template segment<0U, 2U>();
    if (!linalg::all_finite(r))
    {
        return Status::non_finite_result;
    }
    return try_correct(state, covariance, r, horizontal_velocity_jacobian<Linalg>(), V, minimum_quaternion_norm,
                       state_output, covariance_output);
}

} /* end namespace formal_eskf */
