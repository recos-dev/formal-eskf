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
#include <formal_eskf/linalg/linalg.hpp>

namespace formal_eskf::runtime
{

namespace detail
{

template <typename Linalg, std::size_t Size>
[[nodiscard]] Status validate_gnss_reset_inputs(configuration::Ins::NominalState<Linalg> const & state,
                                                linalg::Matrix<Linalg, 15U, 15U> const & covariance,
                                                linalg::Matrix<Linalg, Size, 1U> const & observation,
                                                linalg::Matrix<Linalg, Size, 1U> const & variances) noexcept
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
void uncorrelate_navigation_axis(linalg::Matrix<Linalg, 15U, 15U> & covariance, std::size_t axis) noexcept
{
    for (std::size_t index = 0U; index < 15U; ++index)
    {
        if (index != axis)
        {
            covariance.set(axis, index, typename Linalg::value_type{0});
            covariance.set(index, axis, typename Linalg::value_type{0});
        }
    }
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
[[nodiscard]] Status try_reset_gnss_horizontal_position(configuration::Ins::NominalState<Linalg> const & state,
                                                        linalg::Matrix<Linalg, 15U, 15U> const & covariance,
                                                        linalg::Matrix<Linalg, 2U, 1U> const & position,
                                                        linalg::Matrix<Linalg, 2U, 1U> const & variances,
                                                        configuration::Ins::NominalState<Linalg> & state_output,
                                                        linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
                                                        linalg::Matrix<Linalg, 2U, 1U> & delta_output) noexcept
{
    using value_type = typename Linalg::value_type;
    Status const status = detail::validate_gnss_reset_inputs(state, covariance, position, variances);
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

    constexpr value_type minimum_standard_deviation = static_cast<value_type>(0.01);
    constexpr value_type minimum_variance = minimum_standard_deviation * minimum_standard_deviation;
    auto candidate = state;
    auto P = covariance;
    for (std::size_t axis = 0U; axis < 2U; ++axis)
    {
        candidate.p_n.set(axis, position(axis));
        detail::uncorrelate_navigation_axis(P, axis);
        P.set(axis, axis, horizontal_variance < minimum_variance ? minimum_variance : horizontal_variance);
    }
    state_output = candidate;
    covariance_output = P;
    delta_output = delta;
    return Status::success;
}

/**
 * Replace INS vertical position with a qualified GNSS observation in local
 * NED meters (positive Down), not positive-up altitude. Set P(2,2) to
 * max(variance, 0.01^2) in m^2 and clear the off-diagonal entries in row/column 2.
 * All other nominal fields and covariance entries remain unchanged, including
 * vertical velocity and its variance. Delta is new p_D minus old p_D.
 *
 * Altitude datum/sign/origin conversion, lever arms and variance preparation
 * belong to the caller. Input validation, delta overflow and failure atomicity
 * follow try_reset_gnss_horizontal_position. State/P outputs may independently
 * alias their complete priors; delta output must be disjoint from state/P.
 * No height-reference initialization, bias adjustment or velocity reset occurs.
 */
template <typename Linalg>
[[nodiscard]] Status try_reset_gnss_vertical_position(configuration::Ins::NominalState<Linalg> const & state,
                                                      linalg::Matrix<Linalg, 15U, 15U> const & covariance,
                                                      typename Linalg::value_type position_d,
                                                      typename Linalg::value_type variance,
                                                      configuration::Ins::NominalState<Linalg> & state_output,
                                                      linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
                                                      typename Linalg::value_type & delta_output) noexcept
{
    using value_type = typename Linalg::value_type;
    using scalar_vector_type = linalg::Matrix<Linalg, 1U, 1U>;
    Status const status =
        detail::validate_gnss_reset_inputs(state, covariance, scalar_vector_type::from_row_major({position_d}),
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

    constexpr value_type minimum_standard_deviation = static_cast<value_type>(0.01);
    constexpr value_type minimum_variance = minimum_standard_deviation * minimum_standard_deviation;
    auto candidate = state;
    candidate.p_n.set(2U, position_d);
    auto P = covariance;
    detail::uncorrelate_navigation_axis(P, 2U);
    P.set(2U, 2U, variance < minimum_variance ? minimum_variance : variance);
    state_output = candidate;
    covariance_output = P;
    delta_output = delta;
    return Status::success;
}

/**
 * Replace all three NED velocity components with a qualified GNSS observation
 * in m/s. Set each velocity variance to max(variance_i, 0.01^2), in m^2/s^2.
 * Decouple all position AND velocity coordinates from each other and the rest
 * of P; preserve position values/diagonals, attitude, biases and their P block.
 * Variances are already prepared per axis; no vertical-noise scaling is added.
 * Input, delta and atomicity contracts follow try_reset_gnss_horizontal_position.
 * This is neither a height-position reset nor a gated Kalman correction.
 */
template <typename Linalg>
[[nodiscard]] Status try_reset_gnss_velocity(configuration::Ins::NominalState<Linalg> const & state,
                                             linalg::Matrix<Linalg, 15U, 15U> const & covariance,
                                             linalg::Matrix<Linalg, 3U, 1U> const & velocity,
                                             linalg::Matrix<Linalg, 3U, 1U> const & variances,
                                             configuration::Ins::NominalState<Linalg> & state_output,
                                             linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
                                             linalg::Matrix<Linalg, 3U, 1U> & delta_output) noexcept
{
    using value_type = typename Linalg::value_type;
    Status const status = detail::validate_gnss_reset_inputs(state, covariance, velocity, variances);
    if (!succeeded(status))
    {
        return status;
    }
    auto const delta = velocity - state.v_n;
    if (!linalg::all_finite(delta))
    {
        return Status::non_finite_result;
    }

    constexpr value_type minimum_standard_deviation = static_cast<value_type>(0.01);
    constexpr value_type minimum_variance = minimum_standard_deviation * minimum_standard_deviation;
    auto candidate = state;
    candidate.v_n = velocity;
    auto P = covariance;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        detail::uncorrelate_navigation_axis(P, axis);
        detail::uncorrelate_navigation_axis(P, 3U + axis);
        P.set(3U + axis, 3U + axis, variances(axis) < minimum_variance ? minimum_variance : variances(axis));
    }
    state_output = candidate;
    covariance_output = P;
    delta_output = delta;
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
