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
 * Horizontal and vertical navigation-frame position measurements for INS.
 */

#include <formal_eskf/eskf/correction.hpp>

namespace formal_eskf
{

/**
 * Error-state Jacobian of h(state) = [p_N, p_E]^T, not of the residual.
 * Selects columns 0 and 1 of the INS error state
 * [delta_p_n, delta_v_n, delta_theta_b, delta_b_a, delta_b_g].
 * No vertical position observation is introduced.
 */
template <typename Linalg> [[nodiscard]] linalg::Matrix<Linalg, 2U, 15U> horizontal_position_jacobian() noexcept
{
    linalg::Matrix<Linalg, 2U, 15U> H;
    H.template set_block<0U, 0U>(linalg::Matrix<Linalg, 2U, 2U>::identity());
    return H;
}

/**
 * Error-state Jacobian of h(state) = p_D, selecting INS error column 2.
 * Vertical position is positive down, not positive-up altitude or distance
 * above terrain. Both position observation Jacobians are constant and exact.
 */
template <typename Linalg> [[nodiscard]] linalg::Matrix<Linalg, 1U, 15U> vertical_position_jacobian() noexcept
{
    linalg::Matrix<Linalg, 1U, 15U> H;
    H.set(0U, 2U, typename Linalg::value_type{1});
    return H;
}

/**
 * Correct INS with horizontal position z_p_ne = [p_N, p_E]^T in meters and
 * its 2x2 noise covariance V in m^2. No p_D measurement is required.
 * The measurement describes the same body/IMU origin as state.p_n, in the same
 * local NED frame and at the time of the supplied prior state and covariance.
 * Frame/origin conversion, lever-arm compensation and time alignment are caller
 * responsibilities; this operation cannot check them.
 *
 * Forms r = z_p_ne - state.p_n.segment<0, 2>(), then delegates to
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
[[nodiscard]] Status try_correct_horizontal_position(configuration::Ins::NominalState<Linalg> const & state,
                                                     linalg::Matrix<Linalg, 15U, 15U> const & covariance,
                                                     linalg::Matrix<Linalg, 2U, 1U> const & z_p_ne,
                                                     linalg::Matrix<Linalg, 2U, 2U> const & V,
                                                     typename Linalg::value_type minimum_quaternion_norm,
                                                     configuration::Ins::NominalState<Linalg> & state_output,
                                                     linalg::Matrix<Linalg, 15U, 15U> & covariance_output) noexcept
{
    if (!linalg::all_finite(state.p_n) || !linalg::all_finite(z_p_ne))
    {
        return Status::non_finite_input;
    }
    auto const r = z_p_ne - state.p_n.template segment<0U, 2U>();
    if (!linalg::all_finite(r))
    {
        return Status::non_finite_result;
    }
    return try_correct(state, covariance, r, horizontal_position_jacobian<Linalg>(), V, minimum_quaternion_norm,
                       state_output, covariance_output);
}

/**
 * Correct INS with vertical position z_p_d in meters (positive down) and its
 * noise variance in m^2, not standard deviation. z_p_d uses the same local
 * NED origin, body/IMU point and time as state.p_n(2); no horizontal position
 * or vertical velocity measurement is required.
 *
 * Forms r = z_p_d - state.p_n(2), then uses the same full-state correction as
 * try_correct_horizontal_position. In particular, position/velocity cross-
 * covariance can correct vertical velocity without a velocity observation.
 * Covariance preconditions, failure statuses and output aliasing follow that
 * operation. The variance must be finite and strictly positive.
 *
 * This is not a rangefinder model: terrain, tilt/lever-arm compensation,
 * altitude datum/sign conversion and sensor bias handling belong to the caller
 * or a separate sensor model. No differenced velocity is generated or fused.
 * Separate observation calls do not represent cross-source noise correlations.
 */
template <typename Linalg>
[[nodiscard]] Status
try_correct_vertical_position(configuration::Ins::NominalState<Linalg> const & state,
                              linalg::Matrix<Linalg, 15U, 15U> const & covariance, typename Linalg::value_type z_p_d,
                              typename Linalg::value_type variance, typename Linalg::value_type minimum_quaternion_norm,
                              configuration::Ins::NominalState<Linalg> & state_output,
                              linalg::Matrix<Linalg, 15U, 15U> & covariance_output) noexcept
{
    using scalar_math_type = typename Linalg::scalar_math_type;
    if (!linalg::all_finite(state.p_n) || !scalar::is_finite<scalar_math_type>(z_p_d) ||
        !scalar::is_finite<scalar_math_type>(variance))
    {
        return Status::non_finite_input;
    }
    auto const r = linalg::Matrix<Linalg, 1U, 1U>::from_row_major({z_p_d - state.p_n(2U)});
    if (!linalg::all_finite(r))
    {
        return Status::non_finite_result;
    }
    auto const V = linalg::Matrix<Linalg, 1U, 1U>::from_row_major({variance});
    return try_correct(state, covariance, r, vertical_position_jacobian<Linalg>(), V, minimum_quaternion_norm,
                       state_output, covariance_output);
}

} /* end namespace formal_eskf */
