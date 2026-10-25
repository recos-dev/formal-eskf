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
template <typename Linalg, std::size_t Size> struct GnssInnovationCheck
{
    Status status = Status::success;
    bool diagnostics_valid = false;
    bool rejected = true;
    linalg::Matrix<Linalg, Size, 1U> innovation{};
    linalg::Matrix<Linalg, Size, 1U> innovation_variance{};
    linalg::Matrix<Linalg, Size, 1U> test_ratio{};
    linalg::Matrix<Linalg, Size, 1U> fusion_innovation{};
    bool innovation_limited = false;

    GnssInnovationCheck() = default;
    explicit GnssInnovationCheck(FusionResult<Linalg, Size> const & gate) noexcept
        : status(gate.status), diagnostics_valid(gate.diagnostics_valid),
          rejected(!gate.diagnostics_valid || gate.decision == FusionDecision::rejected), innovation(gate.innovation),
          innovation_variance(gate.innovation_variance), test_ratio(gate.test_ratio), fusion_innovation(gate.innovation)
    {
    }
};

/** Raw gate evidence is retained even when a bounded vertical innovation is fused. */
template <typename Linalg, std::size_t Size> struct GnssMeasurementResult
{
    FusionResult<Linalg, Size> fusion{};
    // Valid when fusion.diagnostics_valid; actually applied only when fused.
    linalg::Matrix<Linalg, Size, 1U> fusion_innovation{};
    bool innovation_limited = false;
};

namespace detail
{

/** Adjust a rejected gate without re-gating a rounded value at its boundary. */
template <typename Linalg, std::size_t Size>
void limit_gnss_innovation(GnssInnovationCheck<Linalg, Size> & check, std::size_t axis,
                           typename Linalg::value_type gate_sigma, bool allowed) noexcept
{
    if (!check.diagnostics_valid || !allowed || !check.rejected)
    {
        return;
    }
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    value_type deviation{};
    check.status = scalar::try_sqrt<math_type>(check.innovation_variance(axis), deviation);
    if (!succeeded(check.status))
    {
        return;
    }
    auto const limit = gate_sigma * deviation;
    if (!scalar::is_finite<math_type>(limit))
    {
        check.status = Status::non_finite_result;
        return;
    }
    auto const innovation = check.innovation(axis);
    check.fusion_innovation.set(axis, innovation < -limit ? -limit : (innovation > limit ? limit : innovation));
    check.innovation_limited = true;
    check.rejected = false;
}

template <typename Linalg, std::size_t Size>
[[nodiscard]] GnssMeasurementResult<Linalg, Size> correct_gnss_measurement(
    GnssInnovationCheck<Linalg, Size> const & check, configuration::Ins::NominalState<Linalg> const & state,
    linalg::Matrix<Linalg, 15U, 15U> const & covariance, linalg::Matrix<Linalg, Size, 15U> const & H,
    linalg::Matrix<Linalg, Size, Size> const & V, typename Linalg::value_type minimum_quaternion_norm,
    configuration::Ins::NominalState<Linalg> & state_output, linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
    configuration::Ins::BiasUpdate const & bias_update) noexcept
{
    GnssMeasurementResult<Linalg, Size> result;
    auto & fusion = result.fusion;
    fusion.status = check.status;
    fusion.diagnostics_valid = check.diagnostics_valid;
    fusion.innovation = check.innovation;
    fusion.innovation_variance = check.innovation_variance;
    fusion.test_ratio = check.test_ratio;
    result.fusion_innovation = check.fusion_innovation;
    result.innovation_limited = check.innovation_limited;
    if (!check.diagnostics_valid || !succeeded(check.status))
    {
        return result;
    }
    if (check.rejected)
    {
        fusion.decision = FusionDecision::rejected;
        return result;
    }
    fusion.status = try_correct(state, covariance, check.fusion_innovation, H, V, minimum_quaternion_norm, state_output,
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
 * quality-rejected sample while the source is still active. Inspect status,
 * diagnostics_valid and rejected; a passing gate is not successful fusion.
 * Raw diagnostics describe z - v and the prior; fusion_innovation includes the
 * permitted Down limit. Conditions require finite accuracy >= 0 and limit > 0.
 */
template <typename Linalg>
[[nodiscard]] GnssInnovationCheck<Linalg, 3U>
check_gnss_velocity(configuration::Ins::NominalState<Linalg> const & state,
                    linalg::Matrix<Linalg, 15U, 15U> const & covariance,
                    linalg::Matrix<Linalg, 3U, 1U> const & velocity, linalg::Matrix<Linalg, 3U, 3U> const & V,
                    typename Linalg::value_type minimum_quaternion_norm, typename Linalg::value_type gate_sigma,
                    GnssVelocityConditions<typename Linalg::value_type> const & conditions) noexcept
{
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    GnssInnovationCheck<Linalg, 3U> result;
    if (!linalg::all_finite(state.v_n) || !linalg::all_finite(velocity) ||
        !scalar::is_finite<math_type>(conditions.speed_accuracy) ||
        !scalar::is_finite<math_type>(conditions.maximum_speed_accuracy))
    {
        result.status = Status::non_finite_input;
        return result;
    }
    if (conditions.speed_accuracy < value_type{0} || conditions.maximum_speed_accuracy <= value_type{0})
    {
        result.status = Status::domain_error;
        return result;
    }
    auto const r = velocity - state.v_n;
    if (!linalg::all_finite(r))
    {
        result.status = Status::non_finite_result;
        return result;
    }
    result = GnssInnovationCheck<Linalg, 3U>{detail::check_fusion_gate(
        state, covariance, r, velocity_jacobian<Linalg>(), V, minimum_quaternion_norm, gate_sigma)};
    auto const & ratio = result.test_ratio;
    bool const allowed = conditions.bad_vertical_acceleration &&
                         conditions.speed_accuracy < conditions.maximum_speed_accuracy && ratio(2U) > value_type{1} &&
                         ratio(0U) < value_type{1} && ratio(1U) < value_type{1};
    detail::limit_gnss_innovation(result, 2U, gate_sigma, allowed);
    return result;
}

/** Caller must establish quality/lifecycle permission; the exception never grants it. */
template <typename Linalg>
[[nodiscard]] GnssMeasurementResult<Linalg, 3U> fuse_gnss_velocity(
    configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15U, 15U> const & covariance,
    linalg::Matrix<Linalg, 3U, 1U> const & velocity, linalg::Matrix<Linalg, 3U, 3U> const & V,
    typename Linalg::value_type minimum_quaternion_norm, typename Linalg::value_type gate_sigma,
    GnssVelocityConditions<typename Linalg::value_type> const & conditions,
    configuration::Ins::NominalState<Linalg> & state_output, linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
    configuration::Ins::BiasUpdate const & bias_update = {}) noexcept
{
    auto const check =
        check_gnss_velocity(state, covariance, velocity, V, minimum_quaternion_norm, gate_sigma, conditions);
    return detail::correct_gnss_measurement(check, state, covariance, velocity_jacobian<Linalg>(), V,
                                            minimum_quaternion_norm, state_output, covariance_output, bias_update);
}

/**
 * GNSS NED Down position, not altitude or terrain range. With a current vertical
 * acceleration fault, limit a rejected residual to +/- gate * sqrt(S) and fuse.
 * Preserve raw diagnostics, H, V, bias permissions and checked correction's
 * atomic output/whole-prior alias contracts. No source permission or reset here.
 */
template <typename Linalg>
[[nodiscard]] GnssMeasurementResult<Linalg, 1U>
fuse_gnss_height(configuration::Ins::NominalState<Linalg> const & state,
                 linalg::Matrix<Linalg, 15U, 15U> const & covariance, typename Linalg::value_type position_down,
                 typename Linalg::value_type variance, typename Linalg::value_type minimum_quaternion_norm,
                 typename Linalg::value_type gate_sigma, bool bad_vertical_acceleration,
                 configuration::Ins::NominalState<Linalg> & state_output,
                 linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
                 configuration::Ins::BiasUpdate const & bias_update = {}) noexcept
{
    GnssMeasurementResult<Linalg, 1U> result;
    using math_type = typename Linalg::scalar_math_type;
    if (!linalg::all_finite(state.p_n) || !scalar::is_finite<math_type>(position_down) ||
        !scalar::is_finite<math_type>(variance))
    {
        result.fusion.status = Status::non_finite_input;
        return result;
    }
    auto const r = linalg::Matrix<Linalg, 1U, 1U>::from_row_major({position_down - state.p_n(2U)});
    if (!linalg::all_finite(r))
    {
        result.fusion.status = Status::non_finite_result;
        return result;
    }
    auto const V = linalg::Matrix<Linalg, 1U, 1U>::from_row_major({variance});
    GnssInnovationCheck<Linalg, 1U> check{detail::check_fusion_gate(
        state, covariance, r, vertical_position_jacobian<Linalg>(), V, minimum_quaternion_norm, gate_sigma)};
    detail::limit_gnss_innovation(check, 0U, gate_sigma, bad_vertical_acceleration);
    return detail::correct_gnss_measurement(check, state, covariance, vertical_position_jacobian<Linalg>(), V,
                                            minimum_quaternion_norm, state_output, covariance_output, bias_update);
}

} /* end namespace formal_eskf::runtime */
