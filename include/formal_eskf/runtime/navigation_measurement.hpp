/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <formal_eskf/runtime/position.hpp>
#include <formal_eskf/runtime/velocity.hpp>

namespace formal_eskf::runtime
{

template <typename Value> struct GnssVelocityConditions
{
    bool bad_vertical_acceleration = false;
    Value speed_accuracy{};         // Receiver-reported standard deviation, m/s.
    Value maximum_speed_accuracy{}; // Strict upper bound, independent of enabled quality checks.
};

/** Gate evidence only: passing this check does not grant source permission or fuse. */
template <typename Linalg, std::size_t Size> struct NavigationInnovationCheck
{
    MeasurementCheck<Linalg, Size> gate{};
    linalg::Vector<Linalg, Size> fusion_innovation{};
    bool innovation_limited = false;

    NavigationInnovationCheck() = default;
    explicit NavigationInnovationCheck(MeasurementCheck<Linalg, Size> const & check)
        : gate(check), fusion_innovation(check.innovation)
    {
    }
};

/** Raw gate evidence is retained even when a bounded vertical innovation is fused. */
template <typename Linalg, std::size_t Size> struct NavigationFusionResult
{
    FusionResult<Linalg, Size> fusion{};
    // Valid when fusion.diagnostics_valid; actually applied only when fused.
    linalg::Vector<Linalg, Size> fusion_innovation{};
    bool innovation_limited = false;
};

namespace detail
{

/** Adjust a rejected gate without re-gating a rounded value at its boundary. */
template <typename Linalg, std::size_t Size>
void limit_vertical_innovation(NavigationInnovationCheck<Linalg, Size> & check, std::size_t axis,
                               typename Linalg::value_type gate_sigma, bool allowed)
{
    if (!check.gate.diagnostics_valid || !allowed || !check.gate.rejected)
    {
        return;
    }
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    value_type deviation{};
    check.gate.status = scalar::sqrt<math_type>(check.gate.innovation_variance(axis), deviation);
    if (!succeeded(check.gate.status))
    {
        return;
    }
    auto const limit = gate_sigma * deviation;
    if (!scalar::is_finite<math_type>(limit))
    {
        check.gate.status = Status::non_finite_result;
        return;
    }
    auto const innovation = check.gate.innovation(axis);
    check.fusion_innovation(axis) = innovation < -limit ? -limit : (innovation > limit ? limit : innovation);
    check.innovation_limited = true;
    check.gate.rejected = false;
}

template <typename Linalg, std::size_t Size>
[[nodiscard]] NavigationFusionResult<Linalg, Size> correct_navigation_measurement(
    NavigationInnovationCheck<Linalg, Size> const & check, configuration::Ins::NominalState<Linalg> const & state,
    linalg::Matrix<Linalg, 15U, 15U> const & covariance, linalg::Matrix<Linalg, Size, 15U> const & H,
    linalg::Matrix<Linalg, Size, Size> const & V, typename Linalg::value_type minimum_quaternion_norm,
    configuration::Ins::NominalState<Linalg> & state_output, linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
    configuration::Ins::BiasUpdate const & bias_update)
{
    NavigationFusionResult<Linalg, Size> result{FusionResult<Linalg, Size>{check.gate}, check.fusion_innovation,
                                                check.innovation_limited};
    if (!check.gate.passed())
    {
        return result;
    }
    auto & fusion = result.fusion;
    fusion.status = correct(state, covariance, check.fusion_innovation, H, V, minimum_quaternion_norm, state_output,
                            covariance_output, bias_update);
    if (succeeded(fusion.status))
    {
        fusion.decision = FusionDecision::fused;
    }
    return result;
}

} /* end namespace detail */

/**
 * Compute GNSS velocity evidence without correcting state. Also usable for a
 * quality-rejected sample while the source is still active. Inspect gate.status,
 * gate.diagnostics_valid and gate.rejected; a passing gate is not successful fusion.
 * Raw diagnostics describe z - v and the prior; fusion_innovation includes the
 * permitted Down limit. Conditions require finite accuracy >= 0 and limit > 0.
 */
template <typename Linalg>
[[nodiscard]] NavigationInnovationCheck<Linalg, 3U>
check_gnss_velocity(configuration::Ins::NominalState<Linalg> const & state,
                    linalg::Matrix<Linalg, 15U, 15U> const & covariance, linalg::Vector<Linalg, 3U> const & velocity,
                    linalg::Matrix<Linalg, 3U, 3U> const & V, typename Linalg::value_type minimum_quaternion_norm,
                    typename Linalg::value_type gate_sigma,
                    GnssVelocityConditions<typename Linalg::value_type> const & conditions)
{
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    NavigationInnovationCheck<Linalg, 3U> result;
    if (!linalg::all_finite(state.v_n) || !linalg::all_finite(velocity) ||
        !scalar::is_finite<math_type>(conditions.speed_accuracy) ||
        !scalar::is_finite<math_type>(conditions.maximum_speed_accuracy))
    {
        result.gate.status = Status::non_finite_input;
        return result;
    }
    if (conditions.speed_accuracy < value_type{0} || conditions.maximum_speed_accuracy <= value_type{0})
    {
        result.gate.status = Status::domain_error;
        return result;
    }
    auto const r = velocity - state.v_n;
    if (!linalg::all_finite(r))
    {
        result.gate.status = Status::non_finite_result;
        return result;
    }
    result = NavigationInnovationCheck<Linalg, 3U>{
        check_measurement(state, covariance, r, velocity_jacobian<Linalg>(), V, minimum_quaternion_norm, gate_sigma)};
    auto const & ratio = result.gate.test_ratio;
    bool const allowed = conditions.bad_vertical_acceleration &&
                         conditions.speed_accuracy < conditions.maximum_speed_accuracy && ratio(2U) > value_type{1} &&
                         ratio(0U) < value_type{1} && ratio(1U) < value_type{1};
    detail::limit_vertical_innovation(result, 2U, gate_sigma, allowed);
    return result;
}

/** Caller must establish quality/lifecycle permission; the exception never grants it. */
template <typename Linalg>
[[nodiscard]] NavigationFusionResult<Linalg, 3U> fuse_gnss_velocity(
    configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15U, 15U> const & covariance,
    linalg::Vector<Linalg, 3U> const & velocity, linalg::Matrix<Linalg, 3U, 3U> const & V,
    typename Linalg::value_type minimum_quaternion_norm, typename Linalg::value_type gate_sigma,
    GnssVelocityConditions<typename Linalg::value_type> const & conditions,
    configuration::Ins::NominalState<Linalg> & state_output, linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
    configuration::Ins::BiasUpdate const & bias_update = {})
{
    auto const check =
        check_gnss_velocity(state, covariance, velocity, V, minimum_quaternion_norm, gate_sigma, conditions);
    return detail::correct_navigation_measurement(check, state, covariance, velocity_jacobian<Linalg>(), V,
                                                  minimum_quaternion_norm, state_output, covariance_output,
                                                  bias_update);
}

/**
 * NED Down position from a qualified height source, not altitude or terrain range.
 * The adapter owns source policy and datum conversion. With a current vertical
 * acceleration fault, limit a rejected residual to +/- gate * sqrt(S) and fuse.
 * Preserve raw diagnostics, H, V, bias permissions and checked correction's
 * atomic output/whole-prior alias contracts. No source permission or reset here.
 */
template <typename Linalg>
[[nodiscard]] NavigationFusionResult<Linalg, 1U>
fuse_height(configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15U, 15U> const & covariance,
            typename Linalg::value_type position_down, typename Linalg::value_type variance,
            typename Linalg::value_type minimum_quaternion_norm, typename Linalg::value_type gate_sigma,
            bool bad_vertical_acceleration, configuration::Ins::NominalState<Linalg> & state_output,
            linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
            configuration::Ins::BiasUpdate const & bias_update = {})
{
    NavigationFusionResult<Linalg, 1U> result;
    using math_type = typename Linalg::scalar_math_type;
    if (!linalg::all_finite(state.p_n) || !scalar::is_finite<math_type>(position_down) ||
        !scalar::is_finite<math_type>(variance))
    {
        result.fusion.status = Status::non_finite_input;
        return result;
    }
    auto const r = linalg::Vector<Linalg, 1U>::from_row_major({position_down - state.p_n(2U)});
    if (!linalg::all_finite(r))
    {
        result.fusion.status = Status::non_finite_result;
        return result;
    }
    auto const V = linalg::Matrix<Linalg, 1U, 1U>::from_row_major({variance});
    NavigationInnovationCheck<Linalg, 1U> check{check_measurement(
        state, covariance, r, vertical_position_jacobian<Linalg>(), V, minimum_quaternion_norm, gate_sigma)};
    detail::limit_vertical_innovation(check, 0U, gate_sigma, bad_vertical_acceleration);
    return detail::correct_navigation_measurement(check, state, covariance, vertical_position_jacobian<Linalg>(), V,
                                                  minimum_quaternion_norm, state_output, covariance_output,
                                                  bias_update);
}

} /* end namespace formal_eskf::runtime */
