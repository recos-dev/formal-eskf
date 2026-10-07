/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <formal_eskf/eskf/velocity.hpp>
#include <formal_eskf/runtime/measurement_fusion.hpp>

namespace formal_eskf::runtime
{

/**
 * Gated 3D velocity, using the core's NED observation and Jacobian.
 * Frames, units, covariance premises and aliases follow try_correct_velocity.
 * All three axes form one rejection group, independent of position calls.
 * Use this OR fuse_horizontal_velocity for a sample, never both.
 */
template <typename Linalg>
[[nodiscard]] FusionResult<Linalg, 3U>
fuse_velocity(configuration::Ins::NominalState<Linalg> const & state,
              linalg::Matrix<Linalg, 15U, 15U> const & covariance, linalg::Matrix<Linalg, 3U, 1U> const & z_v_n,
              linalg::Matrix<Linalg, 3U, 3U> const & V, typename Linalg::value_type minimum_quaternion_norm,
              typename Linalg::value_type gate_sigma, configuration::Ins::NominalState<Linalg> & state_output,
              linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
              configuration::Ins::BiasUpdate const & bias_update = {}) noexcept
{
    FusionResult<Linalg, 3U> result;
    if (!linalg::all_finite(state.v_n) || !linalg::all_finite(z_v_n))
    {
        result.status = Status::non_finite_input;
        return result;
    }
    auto const r = z_v_n - state.v_n;
    if (!linalg::all_finite(r))
    {
        result.status = Status::non_finite_result;
        return result;
    }
    return fuse_measurement(state, covariance, r, velocity_jacobian<Linalg>(), V, minimum_quaternion_norm, gate_sigma,
                            state_output, covariance_output, bias_update);
}

/** Gated horizontal velocity; both axes form one rejection group, with no v_D observation. */
template <typename Linalg>
[[nodiscard]] FusionResult<Linalg, 2U> fuse_horizontal_velocity(
    configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15U, 15U> const & covariance,
    linalg::Matrix<Linalg, 2U, 1U> const & z_v_ne, linalg::Matrix<Linalg, 2U, 2U> const & V,
    typename Linalg::value_type minimum_quaternion_norm, typename Linalg::value_type gate_sigma,
    configuration::Ins::NominalState<Linalg> & state_output, linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
    configuration::Ins::BiasUpdate const & bias_update = {}) noexcept
{
    FusionResult<Linalg, 2U> result;
    if (!linalg::all_finite(state.v_n) || !linalg::all_finite(z_v_ne))
    {
        result.status = Status::non_finite_input;
        return result;
    }
    auto const r = z_v_ne - state.v_n.template segment<0U, 2U>();
    if (!linalg::all_finite(r))
    {
        result.status = Status::non_finite_result;
        return result;
    }
    return fuse_measurement(state, covariance, r, horizontal_velocity_jacobian<Linalg>(), V, minimum_quaternion_norm,
                            gate_sigma, state_output, covariance_output, bias_update);
}

} /* end namespace formal_eskf::runtime */
