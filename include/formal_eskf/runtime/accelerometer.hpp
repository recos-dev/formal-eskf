/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <type_traits>

#include <formal_eskf/eskf/accelerometer.hpp>
#include <formal_eskf/runtime/accelerometer_conditions.hpp>
#include <formal_eskf/runtime/measurement_fusion.hpp>

namespace formal_eskf::runtime
{

/** Eligibility is not fusion success; only fused publishes state and covariance. */
struct AccelerometerFusionResult
{
    AccelerometerFusionCheck conditions{};
    FusionDecision decision = FusionDecision::failed;
    Status status = Status::success;
};

namespace detail
{

template <typename State, typename Linalg, std::size_t Size>
[[nodiscard]] AccelerometerFusionResult
fuse_accelerometer_state(State const & state, linalg::Matrix<Linalg, Size, Size> const & covariance,
                         linalg::Matrix<Linalg, 3U, 1U> const & specific_force_b,
                         linalg::Matrix<Linalg, 3U, 1U> const & gravity_n, linalg::Matrix<Linalg, 3U, 3U> const & V,
                         AccelerometerFusionConditions const & conditions,
                         typename AccelerometerMagnitude<Linalg>::Result const & magnitude,
                         typename Linalg::value_type minimum_quaternion_norm, State & state_output,
                         linalg::Matrix<Linalg, Size, Size> & covariance_output,
                         configuration::Ins::BiasUpdate const & bias_update = {}) noexcept
{
    AccelerometerFusionResult result;
    result.conditions = check_accelerometer_fusion_conditions<Linalg>(conditions, magnitude);
    result.status = result.conditions.status;
    if (!succeeded(result.status))
    {
        return result;
    }
    if (!result.conditions.sample_eligible)
    {
        result.decision = FusionDecision::rejected;
        return result;
    }
    if constexpr (std::is_same_v<State, configuration::Ins::NominalState<Linalg>>)
    {
        result.status =
            try_correct_accelerometer(state, covariance, specific_force_b, gravity_n, V, minimum_quaternion_norm,
                                      state_output, covariance_output, bias_update);
    }
    else
    {
        result.status = try_correct_accelerometer(state, covariance, specific_force_b, gravity_n, V,
                                                  minimum_quaternion_norm, state_output, covariance_output);
    }
    if (succeeded(result.status))
    {
        result.decision = FusionDecision::fused;
    }
    return result;
}

} /* end namespace detail */

/**
 * Check gravity-fusion prerequisites, then run the unchanged core correction.
 * Pass the CURRENT specific force (m/s^2), not the LPF vector or an increment.
 * AHRS input is externally bias-calibrated; INS subtracts the prior bias once.
 * gravity_n and effective V retain the core's units and validity premises.
 *
 * The caller updates its magnitude filter once per sample, even when fusion is
 * disabled, then supplies that result with the same sample/horizon conditions.
 * It owns cadence, freshness and reset on source/calibration/frame changes.
 * This function never updates or rolls back that history. Do not reuse a stale
 * successful magnitude result after an update failure.
 *
 * Failed magnitude evaluation returns failed with its status; unmet conditions
 * return rejected + success without validating unused correction inputs.
 * Eligible samples call correction; only its success returns fused. Each output
 * may alias its respective whole prior. Rejection/failure preserves both outputs;
 * other inputs/output storage must be disjoint, as in the core.
 * No innovation gate, normalization of the observation, lifecycle or tuning.
 * Eligibility does not establish negligible acceleration or independent noise.
 */
template <typename Linalg>
[[nodiscard]] AccelerometerFusionResult fuse_accelerometer(
    configuration::Ahrs::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 3U, 3U> const & covariance,
    linalg::Matrix<Linalg, 3U, 1U> const & specific_force_b, linalg::Matrix<Linalg, 3U, 1U> const & gravity_n,
    linalg::Matrix<Linalg, 3U, 3U> const & V, AccelerometerFusionConditions const & conditions,
    typename AccelerometerMagnitude<Linalg>::Result const & magnitude,
    typename Linalg::value_type minimum_quaternion_norm, configuration::Ahrs::NominalState<Linalg> & state_output,
    linalg::Matrix<Linalg, 3U, 3U> & covariance_output) noexcept
{
    return detail::fuse_accelerometer_state(state, covariance, specific_force_b, gravity_n, V, conditions, magnitude,
                                            minimum_quaternion_norm, state_output, covariance_output);
}

/** INS forwards all six bias-learning permissions unchanged to the core. */
template <typename Linalg>
[[nodiscard]] AccelerometerFusionResult fuse_accelerometer(
    configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15U, 15U> const & covariance,
    linalg::Matrix<Linalg, 3U, 1U> const & specific_force_b, linalg::Matrix<Linalg, 3U, 1U> const & gravity_n,
    linalg::Matrix<Linalg, 3U, 3U> const & V, AccelerometerFusionConditions const & conditions,
    typename AccelerometerMagnitude<Linalg>::Result const & magnitude,
    typename Linalg::value_type minimum_quaternion_norm, configuration::Ins::NominalState<Linalg> & state_output,
    linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
    configuration::Ins::BiasUpdate const & bias_update = {}) noexcept
{
    return detail::fuse_accelerometer_state(state, covariance, specific_force_b, gravity_n, V, conditions, magnitude,
                                            minimum_quaternion_norm, state_output, covariance_output, bias_update);
}

} /* end namespace formal_eskf::runtime */
