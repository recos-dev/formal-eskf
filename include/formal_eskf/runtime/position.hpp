/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <formal_eskf/eskf/position.hpp>
#include <formal_eskf/runtime/measurement_fusion.hpp>

namespace formal_eskf::runtime
{

/**
 * Gated horizontal position, using the core's NED observation and Jacobian.
 * Frames, units, covariance premises and aliases follow try_correct_horizontal_position.
 * Both axes form one rejection group, independent of height and velocity calls.
 * Compute the residual from this call's prior; fuse_measurement owns gate/correction.
 */
template <typename Linalg>
[[nodiscard]] FusionResult<Linalg, 2U> fuse_horizontal_position(
    configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15U, 15U> const & covariance,
    linalg::Matrix<Linalg, 2U, 1U> const & z_p_ne, linalg::Matrix<Linalg, 2U, 2U> const & V,
    typename Linalg::value_type minimum_quaternion_norm, typename Linalg::value_type gate_sigma,
    configuration::Ins::NominalState<Linalg> & state_output, linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
    configuration::Ins::BiasUpdate const & bias_update = {}) noexcept
{
    FusionResult<Linalg, 2U> result;
    if (!linalg::all_finite(state.p_n) || !linalg::all_finite(z_p_ne))
    {
        result.status = Status::non_finite_input;
        return result;
    }
    auto const r = z_p_ne - state.p_n.template segment<0U, 2U>();
    if (!linalg::all_finite(r))
    {
        result.status = Status::non_finite_result;
        return result;
    }
    return fuse_measurement(state, covariance, r, horizontal_position_jacobian<Linalg>(), V, minimum_quaternion_norm,
                            gate_sigma, state_output, covariance_output, bias_update);
}

/** Gated NED down position, not altitude or range above terrain; one rejection group. */
template <typename Linalg>
[[nodiscard]] FusionResult<Linalg, 1U>
fuse_vertical_position(configuration::Ins::NominalState<Linalg> const & state,
                       linalg::Matrix<Linalg, 15U, 15U> const & covariance, typename Linalg::value_type z_p_d,
                       typename Linalg::value_type variance, typename Linalg::value_type minimum_quaternion_norm,
                       typename Linalg::value_type gate_sigma, configuration::Ins::NominalState<Linalg> & state_output,
                       linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
                       configuration::Ins::BiasUpdate const & bias_update = {}) noexcept
{
    FusionResult<Linalg, 1U> result;
    using math_type = typename Linalg::scalar_math_type;
    if (!linalg::all_finite(state.p_n) || !scalar::is_finite<math_type>(z_p_d) ||
        !scalar::is_finite<math_type>(variance))
    {
        result.status = Status::non_finite_input;
        return result;
    }
    auto const r = linalg::Matrix<Linalg, 1U, 1U>::from_row_major({z_p_d - state.p_n(2U)});
    if (!linalg::all_finite(r))
    {
        result.status = Status::non_finite_result;
        return result;
    }
    auto const V = linalg::Matrix<Linalg, 1U, 1U>::from_row_major({variance});
    return fuse_measurement(state, covariance, r, vertical_position_jacobian<Linalg>(), V, minimum_quaternion_norm,
                            gate_sigma, state_output, covariance_output, bias_update);
}

} /* end namespace formal_eskf::runtime */
