/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <cstddef>

#include <formal_eskf/eskf/configuration/ins.hpp>
#include <formal_eskf/eskf/position.hpp>
#include <formal_eskf/eskf/velocity.hpp>
#include <formal_eskf/linalg/linalg.hpp>
#include <formal_eskf/runtime/measurement_fusion.hpp>

namespace formal_eskf::runtime
{

/**
 * Gated horizontal position, using the core's NED observation and Jacobian.
 * Frames, units, covariance premises and aliases follow correct_horizontal_position.
 * Both axes form one rejection group, independent of height and velocity calls.
 * Compute the residual from this call's prior; fuse_measurement owns gate/correction.
 */
template <typename Linalg>
[[nodiscard]] FusionResult<Linalg, 2U> fuse_horizontal_position(
    configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15U, 15U> const & covariance,
    linalg::Vector<Linalg, 2U> const & z_p_ne, linalg::Matrix<Linalg, 2U, 2U> const & V,
    typename Linalg::value_type minimum_quaternion_norm, typename Linalg::value_type gate_sigma,
    configuration::Ins::NominalState<Linalg> & state_output, linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
    configuration::Ins::BiasUpdate const & bias_update = {})
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
                       configuration::Ins::BiasUpdate const & bias_update = {})
{
    FusionResult<Linalg, 1U> result;
    using math_type = typename Linalg::scalar_math_type;
    if (!linalg::all_finite(state.p_n) || !scalar::is_finite<math_type>(z_p_d) ||
        !scalar::is_finite<math_type>(variance))
    {
        result.status = Status::non_finite_input;
        return result;
    }
    auto const r = linalg::Vector<Linalg, 1U>::from_row_major({z_p_d - state.p_n(2U)});
    if (!linalg::all_finite(r))
    {
        result.status = Status::non_finite_result;
        return result;
    }
    auto const V = linalg::Matrix<Linalg, 1U, 1U>::from_row_major({variance});
    return fuse_measurement(state, covariance, r, vertical_position_jacobian<Linalg>(), V, minimum_quaternion_norm,
                            gate_sigma, state_output, covariance_output, bias_update);
}

/**
 * Gated 3D velocity, using the core's NED observation and Jacobian.
 * Frames, units, covariance premises and aliases follow correct_velocity.
 * All three axes form one rejection group, independent of position calls.
 * Use this OR fuse_horizontal_velocity for a sample, never both.
 */
template <typename Linalg>
[[nodiscard]] FusionResult<Linalg, 3U>
fuse_velocity(configuration::Ins::NominalState<Linalg> const & state,
              linalg::Matrix<Linalg, 15U, 15U> const & covariance, linalg::Vector<Linalg, 3U> const & z_v_n,
              linalg::Matrix<Linalg, 3U, 3U> const & V, typename Linalg::value_type minimum_quaternion_norm,
              typename Linalg::value_type gate_sigma, configuration::Ins::NominalState<Linalg> & state_output,
              linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
              configuration::Ins::BiasUpdate const & bias_update = {})
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
    linalg::Vector<Linalg, 2U> const & z_v_ne, linalg::Matrix<Linalg, 2U, 2U> const & V,
    typename Linalg::value_type minimum_quaternion_norm, typename Linalg::value_type gate_sigma,
    configuration::Ins::NominalState<Linalg> & state_output, linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
    configuration::Ins::BiasUpdate const & bias_update = {})
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

namespace detail
{

template <typename Linalg, std::size_t Size>
[[nodiscard]] Status validate_navigation_reset_inputs(configuration::Ins::NominalState<Linalg> const & state,
                                                      linalg::Matrix<Linalg, 15U, 15U> const & covariance,
                                                      linalg::Vector<Linalg, Size> const & observation,
                                                      linalg::Vector<Linalg, Size> const & variances)
{
    using value_type = typename Linalg::value_type;
    if (!linalg::all_finite(state.p_n) || !linalg::all_finite(state.v_n) ||
        !linalg::all_finite(state.q_nb.coefficients()) || !linalg::all_finite(state.b_a) ||
        !linalg::all_finite(state.b_g) || !linalg::all_finite(covariance) || !linalg::all_finite(observation) ||
        !linalg::all_finite(variances))
    {
        return Status::non_finite_input;
    }
    if (!linalg::is_symmetric(covariance, value_type{0}))
    {
        return Status::domain_error;
    }
    for (std::size_t index = 0U; index < 15U; ++index)
    {
        if (covariance(index, index) < value_type{0})
        {
            return Status::domain_error;
        }
    }
    for (std::size_t axis = 0U; axis < Size; ++axis)
    {
        if (variances(axis) < value_type{0})
        {
            return Status::domain_error;
        }
    }
    return Status::success;
}

/** Decouple one navigation error coordinate, retaining its current variance. */
template <typename Linalg>
void uncorrelate_navigation_axis(linalg::Matrix<Linalg, 15U, 15U> & covariance, std::size_t axis)
{
    for (std::size_t index = 0U; index < 15U; ++index)
    {
        if (index != axis)
        {
            covariance(axis, index) = typename Linalg::value_type{0};
            covariance(index, axis) = typename Linalg::value_type{0};
        }
    }
}

/** Decouple an overwritten coordinate and apply the navigation reset noise floor. */
template <typename Linalg>
void reset_navigation_axis(linalg::Matrix<Linalg, 15U, 15U> & covariance, std::size_t axis,
                           typename Linalg::value_type variance)
{
    using value_type = typename Linalg::value_type;
    constexpr value_type minimum_standard_deviation = static_cast<value_type>(0.01);
    constexpr value_type minimum_variance = minimum_standard_deviation * minimum_standard_deviation;
    uncorrelate_navigation_axis(covariance, axis);
    covariance(axis, axis) = variance < minimum_variance ? minimum_variance : variance;
}

} /* end namespace detail */

/**
 * Replace horizontal INS position with a qualified GNSS observation in local
 * NED meters. Each position variance becomes max(variance_N + variance_E, 0.01^2),
 * in m^2, with its full row/column decorrelated. The sum is intentional; see
 * R-GNSS-RESET. Height, velocity, attitude and biases remain unchanged.
 *
 * Inputs must be finite; P symmetric PSD and the prior quaternion unit. Exact
 * symmetry, nonnegative P diagonals and nonnegative variances are checked, not
 * PSD or quaternion norm. Zero variances receive the explicit floor. Overflow
 * of the variance sum or delta (new minus old) fails before publication.
 * State, P and delta outputs remain unchanged on failure. State/P may each
 * alias their complete prior; all other input/output storage is disjoint.
 * No quality decision, lifecycle acknowledgment or origin change is performed.
 */
template <typename Linalg>
[[nodiscard]] Status reset_gnss_horizontal_position(configuration::Ins::NominalState<Linalg> const & state,
                                                    linalg::Matrix<Linalg, 15U, 15U> const & covariance,
                                                    linalg::Vector<Linalg, 2U> const & position,
                                                    linalg::Vector<Linalg, 2U> const & variances,
                                                    configuration::Ins::NominalState<Linalg> & state_output,
                                                    linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
                                                    linalg::Vector<Linalg, 2U> & delta_output)
{
    using value_type = typename Linalg::value_type;
    Status const status = detail::validate_navigation_reset_inputs(state, covariance, position, variances);
    if (!succeeded(status))
    {
        return status;
    }
    auto const delta = position - state.p_n.template segment<0U, 2U>();
    value_type const horizontal_variance = variances(0U) + variances(1U);
    if (!linalg::all_finite(delta) || !scalar::is_finite<typename Linalg::scalar_math_type>(horizontal_variance))
    {
        return Status::non_finite_result;
    }

    auto candidate = state;
    auto P = covariance;
    for (std::size_t axis = 0U; axis < 2U; ++axis)
    {
        candidate.p_n(axis) = position(axis);
        detail::reset_navigation_axis(P, axis, horizontal_variance);
    }
    state_output = candidate;
    covariance_output = P;
    delta_output = delta;
    return Status::success;
}

/**
 * Replace INS vertical position with a qualified height observation in local
 * NED meters (positive Down), not positive-up altitude. Set P(2,2) to
 * max(variance, 0.01^2) in m^2 and clear the off-diagonal entries in row/column 2.
 * All other nominal fields and covariance entries remain unchanged, including
 * vertical velocity and its variance. Delta is new p_D minus old p_D.
 *
 * Altitude datum/sign/origin conversion, lever arms and variance preparation
 * belong to the caller. Input validation, delta overflow and failure atomicity
 * follow reset_gnss_horizontal_position. State/P outputs may independently
 * alias their complete priors; delta output must be disjoint from state/P.
 * No height-reference initialization, bias adjustment or velocity reset occurs.
 */
template <typename Linalg>
[[nodiscard]] Status
reset_vertical_position(configuration::Ins::NominalState<Linalg> const & state,
                        linalg::Matrix<Linalg, 15U, 15U> const & covariance, typename Linalg::value_type position_d,
                        typename Linalg::value_type variance, configuration::Ins::NominalState<Linalg> & state_output,
                        linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
                        typename Linalg::value_type & delta_output)
{
    using value_type = typename Linalg::value_type;
    using scalar_vector_type = linalg::Vector<Linalg, 1U>;
    Status const status =
        detail::validate_navigation_reset_inputs(state, covariance, scalar_vector_type::from_row_major({position_d}),
                                                 scalar_vector_type::from_row_major({variance}));
    if (!succeeded(status))
    {
        return status;
    }
    value_type const delta = position_d - state.p_n(2U);
    if (!scalar::is_finite<typename Linalg::scalar_math_type>(delta))
    {
        return Status::non_finite_result;
    }

    auto candidate = state;
    candidate.p_n(2U) = position_d;
    auto P = covariance;
    detail::reset_navigation_axis(P, 2U, variance);
    state_output = candidate;
    covariance_output = P;
    delta_output = delta;
    return Status::success;
}

/**
 * Replace only INS Down velocity with a qualified navigation observation in m/s.
 * Set P(5,5) to max(variance, 0.01^2) in m^2/s^2. Decouple both Down position
 * and Down velocity (rows/columns 2 and 5), retaining P(2,2). All other nominal
 * fields and covariance entries remain unchanged. Delta is new v_D minus old.
 *
 * Input validation, aliasing and atomicity follow reset_vertical_position.
 * Variance is already prepared; no vertical-noise scaling is added here.
 * No height or horizontal-velocity reset, clock or fusion acknowledgment occurs.
 */
template <typename Linalg>
[[nodiscard]] Status
reset_vertical_velocity(configuration::Ins::NominalState<Linalg> const & state,
                        linalg::Matrix<Linalg, 15U, 15U> const & covariance, typename Linalg::value_type velocity_d,
                        typename Linalg::value_type variance, configuration::Ins::NominalState<Linalg> & state_output,
                        linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
                        typename Linalg::value_type & delta_output)
{
    using value_type = typename Linalg::value_type;
    using scalar_vector_type = linalg::Vector<Linalg, 1U>;
    Status const status =
        detail::validate_navigation_reset_inputs(state, covariance, scalar_vector_type::from_row_major({velocity_d}),
                                                 scalar_vector_type::from_row_major({variance}));
    if (!succeeded(status))
    {
        return status;
    }
    value_type const delta = velocity_d - state.v_n(2U);
    if (!scalar::is_finite<typename Linalg::scalar_math_type>(delta))
    {
        return Status::non_finite_result;
    }

    auto candidate = state;
    candidate.v_n(2U) = velocity_d;
    auto P = covariance;
    detail::uncorrelate_navigation_axis(P, 2U);
    detail::reset_navigation_axis(P, 5U, variance);
    state_output = candidate;
    covariance_output = P;
    delta_output = delta;
    return Status::success;
}

/**
 * Replace all three NED velocity components with a qualified navigation observation
 * in m/s. Set each velocity variance to max(variance_i, 0.01^2), in m^2/s^2.
 * Decouple all position AND velocity coordinates from each other and the rest
 * of P; preserve position values/diagonals, attitude, biases and their P block.
 * Variances are already prepared per axis; no vertical-noise scaling is added.
 * Input, delta and atomicity contracts follow reset_gnss_horizontal_position.
 * This is neither a height-position reset nor a gated Kalman correction.
 */
template <typename Linalg>
[[nodiscard]] Status
reset_velocity(configuration::Ins::NominalState<Linalg> const & state,
               linalg::Matrix<Linalg, 15U, 15U> const & covariance, linalg::Vector<Linalg, 3U> const & velocity,
               linalg::Vector<Linalg, 3U> const & variances, configuration::Ins::NominalState<Linalg> & state_output,
               linalg::Matrix<Linalg, 15U, 15U> & covariance_output, linalg::Vector<Linalg, 3U> & delta_output)
{
    Status const status = detail::validate_navigation_reset_inputs(state, covariance, velocity, variances);
    if (!succeeded(status))
    {
        return status;
    }
    auto const delta = velocity - state.v_n;
    if (!linalg::all_finite(delta))
    {
        return Status::non_finite_result;
    }

    auto candidate = state;
    candidate.v_n = velocity;
    auto P = covariance;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        detail::uncorrelate_navigation_axis(P, axis);
        detail::reset_navigation_axis(P, 3U + axis, variances(axis));
    }
    state_output = candidate;
    covariance_output = P;
    delta_output = delta;
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
