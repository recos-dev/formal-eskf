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
 * Local navigation-frame position measurement for INS.
 */

#include <formal_eskf/eskf/correction.hpp>

namespace formal_eskf
{

/**
 * Error-state Jacobian of h(state) = p_n, not of the residual z_p_n - p_n.
 * H = [I, 0, 0, 0, 0] follows the INS ordering
 * [delta_p_n, delta_v_n, delta_theta_b, delta_b_a, delta_b_g].
 * This affine observation has a constant, exact Jacobian.
 */
template <typename Linalg> [[nodiscard]] linalg::Matrix<Linalg, 3U, 15U> position_jacobian() noexcept
{
    linalg::Matrix<Linalg, 3U, 15U> H;
    H.template set_block<0U, 0U>(linalg::Matrix<Linalg, 3U, 3U>::identity());
    return H;
}

/**
 * Correct INS with a position z_p_n in meters and its noise covariance V in m^2.
 * The measurement describes the same body/IMU origin as state.p_n, in the same
 * local NED frame and at the time of the supplied prior state and covariance.
 * Frame/origin conversion, lever-arm compensation and time alignment are caller
 * responsibilities; this operation cannot check them.
 *
 * Forms r = z_p_n - state.p_n and H = [I, 0, 0, 0, 0], then delegates to
 * try_correct for Cholesky, Joseph update, error injection and covariance reset.
 * Full V, including cross-axis correlations, is passed through unchanged.
 * Position can correct other state components through prior cross-covariances.
 *
 * P must be symmetric PSD, V symmetric SPD, and the prior error mean zero, as
 * required by try_correct. Its input checks and failure statuses apply here.
 * Non-finite position inputs return non_finite_input; a non-finite residual
 * computed from finite positions returns non_finite_result. All outputs remain
 * unchanged on failure and may alias their respective state/covariance inputs.
 * No innovation gating, delayed fusion or dynamic allocation is introduced.
 */
template <typename Linalg>
[[nodiscard]] Status
try_correct_position(configuration::Ins::NominalState<Linalg> const & state,
                     linalg::Matrix<Linalg, 15U, 15U> const & covariance, linalg::Matrix<Linalg, 3U, 1U> const & z_p_n,
                     linalg::Matrix<Linalg, 3U, 3U> const & V, typename Linalg::value_type minimum_quaternion_norm,
                     configuration::Ins::NominalState<Linalg> & state_output,
                     linalg::Matrix<Linalg, 15U, 15U> & covariance_output) noexcept
{
    if (!linalg::all_finite(state.p_n) || !linalg::all_finite(z_p_n))
    {
        return Status::non_finite_input;
    }
    auto const r = z_p_n - state.p_n;
    if (!linalg::all_finite(r))
    {
        return Status::non_finite_result;
    }
    return try_correct(state, covariance, r, position_jacobian<Linalg>(), V, minimum_quaternion_norm, state_output,
                       covariance_output);
}

} /* end namespace formal_eskf */
