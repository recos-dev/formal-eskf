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
 * Checked error injection and local-error covariance reset for AHRS and INS.
 */

#include <formal_eskf/eskf/configuration/ahrs.hpp>
#include <formal_eskf/eskf/configuration/ins.hpp>
#include <formal_eskf/eskf/covariance.hpp>
#include <formal_eskf/so3/operations.hpp>
#include <formal_eskf/so3/rotation_vector.hpp>
#include <formal_eskf/so3/right_jacobian.hpp>

// Keep this setting identical in every translation unit of an application.
// It selects covariance reset only, independently of ESKF_QUAT_APPROX.
#ifndef ESKF_RESET_APPROX
#define ESKF_RESET_APPROX 0
#endif

static_assert(ESKF_RESET_APPROX == 0 || ESKF_RESET_APPROX == 1, "ESKF_RESET_APPROX must be 0 or 1");

namespace formal_eskf
{

namespace detail
{

template <typename Linalg>
[[nodiscard]] Status
try_inject_attitude(so3::UnitQuaternion<Linalg> const & q_nb, linalg::Matrix<Linalg, 3U, 1U> const & delta_theta_b,
                    typename Linalg::value_type minimum_norm, so3::UnitQuaternion<Linalg> & output) noexcept
{
    using value_type = typename Linalg::value_type;
    using scalar_math_type = typename Linalg::scalar_math_type;

    if (!linalg::all_finite(q_nb.coefficients()) || !linalg::all_finite(delta_theta_b) ||
        !scalar::is_finite<scalar_math_type>(minimum_norm))
    {
        return Status::non_finite_input;
    }
    if (!(minimum_norm > value_type{0}) || minimum_norm > value_type{1})
    {
        return Status::domain_error;
    }
    so3::UnitQuaternion<Linalg> increment;
    Status const exp_status = so3::try_exp(delta_theta_b, minimum_norm, increment);
    if (!succeeded(exp_status))
    {
        return exp_status;
    }
    Status const status = so3::try_compose_normalized(q_nb, increment, minimum_norm, output);
    return status == Status::non_finite_input ? Status::non_finite_result : status;
}

template <std::size_t AttitudeOffset, typename Linalg, std::size_t Size>
[[nodiscard]] Status try_reset_error_covariance(linalg::Matrix<Linalg, 3U, 1U> const & delta_theta_b,
                                                linalg::Matrix<Linalg, Size, Size> const & covariance,
                                                linalg::Matrix<Linalg, Size, Size> & output) noexcept
{
    static_assert(AttitudeOffset + 3U <= Size);
    using matrix3_type = linalg::Matrix<Linalg, 3U, 3U>;

    if (!linalg::all_finite(delta_theta_b) || !linalg::all_finite(covariance))
    {
        return Status::non_finite_input;
    }
    matrix3_type G_theta;
#if ESKF_RESET_APPROX
    // Sola equation (294): first order in the injected attitude correction.
    G_theta = matrix3_type::identity() - so3::hat<Linalg>(delta_theta_b * typename Linalg::value_type{0.5});
#else
    // The differential of Log(Exp(-a)*Exp(a+epsilon)) at epsilon=0 is J_r(a),
    // from Sola equations (182)--(183), consistent with right-multiplied injection.
    Status const jacobian_status = so3::try_right_jacobian(delta_theta_b, G_theta);
    if (!succeeded(jacobian_status))
    {
        return jacobian_status;
    }
#endif
    auto G = linalg::Matrix<Linalg, Size, Size>::identity();
    G.template set_block<AttitudeOffset, AttitudeOffset>(G_theta);
    auto candidate = linalg::sandwich(G, covariance);
    return try_finish_covariance(candidate, output);
}

} /* end namespace detail */

/**
 * Inject the estimated local attitude error: q_next = normalize(q_nb * Exp(delta_theta_b)).
 * Follows Joan Sola, "Quaternion kinematics for the error-state Kalman filter",
 * equation (283c). Unlike SICE 2023 equation (36), the increment uses Exp rather
 * than [1, delta_theta_b/2]. ESKF_QUAT_APPROX selects prediction only.
 *
 * Consumed inputs must be finite and 0 < minimum_quaternion_norm <= 1.
 * Output may alias state and is unchanged on failure. This operation alone
 * does not reset the error mean or covariance; use try_inject_and_reset for
 * an atomic complete injection/reset step.
 *
 * @see https://arxiv.org/abs/1711.02508
 */
template <typename Linalg>
[[nodiscard]] Status try_inject_nominal(configuration::Ahrs::NominalState<Linalg> const & state,
                                        configuration::Ahrs::ErrorState<Linalg> const & error,
                                        typename Linalg::value_type minimum_quaternion_norm,
                                        configuration::Ahrs::NominalState<Linalg> & output) noexcept
{
    auto candidate = state;
    Status const status =
        detail::try_inject_attitude(state.q_nb, error.delta_theta_b, minimum_quaternion_norm, candidate.q_nb);
    if (!succeeded(status))
    {
        return status;
    }
    output = candidate;
    return Status::success;
}

/**
 * INS injection, Sola equations (283a)--(283e), without an estimated gravity state.
 * Position, velocity and bias errors are added in their configured coordinates;
 * attitude uses the same local right-multiplicative Exp as AHRS.
 * Preconditions and output atomicity follow the AHRS overload.
 */
template <typename Linalg>
[[nodiscard]] Status try_inject_nominal(configuration::Ins::NominalState<Linalg> const & state,
                                        configuration::Ins::ErrorState<Linalg> const & error,
                                        typename Linalg::value_type minimum_quaternion_norm,
                                        configuration::Ins::NominalState<Linalg> & output) noexcept
{
    if (!linalg::all_finite(state.p_n) || !linalg::all_finite(state.v_n) || !linalg::all_finite(state.b_a) ||
        !linalg::all_finite(state.b_g) || !linalg::all_finite(error.delta_p_n) ||
        !linalg::all_finite(error.delta_v_n) || !linalg::all_finite(error.delta_b_a) ||
        !linalg::all_finite(error.delta_b_g))
    {
        return Status::non_finite_input;
    }
    auto candidate = state;
    Status const status =
        detail::try_inject_attitude(state.q_nb, error.delta_theta_b, minimum_quaternion_norm, candidate.q_nb);
    if (!succeeded(status))
    {
        return status;
    }
    candidate.p_n = state.p_n + error.delta_p_n;
    candidate.v_n = state.v_n + error.delta_v_n;
    candidate.b_a = state.b_a + error.delta_b_a;
    candidate.b_g = state.b_g + error.delta_b_g;
    if (!linalg::all_finite(candidate.p_n) || !linalg::all_finite(candidate.v_n) ||
        !linalg::all_finite(candidate.b_a) || !linalg::all_finite(candidate.b_g))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

/**
 * Re-express covariance after injection: P_next = G P G^T.
 * By default, G = J_r(delta_theta_b) for AHRS, using Sola equation (183).
 * ESKF_RESET_APPROX=1 selects I - hat(delta_theta_b/2), the first-order
 * small-correction approximation in equation (294). Neither mode uses the
 * identity-only simplification of SICE 2023 equation (40).
 * Even with the closed-form Jacobian, G*P*G^T remains a linearized covariance
 * transformation, not exact nonlinear uncertainty propagation. The approximate
 * mode additionally requires a small injected correction; no runtime angle
 * threshold is imposed. Nominal injection always uses quaternion Exp.
 *
 * Covariance must be symmetric positive semidefinite on entry; this is a
 * caller precondition, not a runtime PSD test. Finite covariance and attitude
 * error are checked. Symmetry cleanup is not a PSD repair or an IEEE-754 PSD
 * guarantee. Output may alias covariance and remains unchanged on failure.
 * The error input is the estimated correction BEFORE resetting its mean.
 */
template <typename Linalg>
[[nodiscard]] Status try_reset_covariance(configuration::Ahrs::ErrorState<Linalg> const & error,
                                          linalg::Matrix<Linalg, 3U, 3U> const & covariance,
                                          linalg::Matrix<Linalg, 3U, 3U> & output) noexcept
{
    return detail::try_reset_error_covariance<0U>(error.delta_theta_b, covariance, output);
}

/**
 * INS reset uses G = diag(I6, G_theta, I6), with G_theta selected as for AHRS.
 * The entire covariance, including attitude cross-covariances, is transformed.
 * Only delta_theta_b is consumed from error; the additive blocks have identity
 * reset derivatives. Other preconditions follow the AHRS overload.
 */
template <typename Linalg>
[[nodiscard]] Status try_reset_covariance(configuration::Ins::ErrorState<Linalg> const & error,
                                          linalg::Matrix<Linalg, 15U, 15U> const & covariance,
                                          linalg::Matrix<Linalg, 15U, 15U> & output) noexcept
{
    return detail::try_reset_error_covariance<6U>(error.delta_theta_b, covariance, output);
}

namespace detail
{

template <typename Linalg, typename State, typename Error, std::size_t Size>
[[nodiscard]] Status
try_inject_state_and_reset(State const & state, linalg::Matrix<Linalg, Size, Size> const & covariance,
                           Error const & error, typename Linalg::value_type minimum_quaternion_norm,
                           State & state_output, linalg::Matrix<Linalg, Size, Size> & covariance_output,
                           Error & error_output) noexcept
{
    State state_candidate;
    Status const nominal_status = try_inject_nominal(state, error, minimum_quaternion_norm, state_candidate);
    if (!succeeded(nominal_status))
    {
        return nominal_status;
    }
    linalg::Matrix<Linalg, Size, Size> covariance_candidate;
    Status const covariance_status = try_reset_covariance(error, covariance, covariance_candidate);
    if (!succeeded(covariance_status))
    {
        return covariance_status;
    }
    state_output = state_candidate;
    covariance_output = covariance_candidate;
    error_output = Error{};
    return Status::success;
}

} /* end namespace detail */

/**
 * Atomically inject the estimated error, reset covariance, and zero the error
 * MEAN (Sola equation (285)), not covariance or the unknown true error.
 * Preconditions are the union of try_inject_nominal and try_reset_covariance.
 * All three outputs commit only on success and may alias their respective
 * inputs. Separate inputs are unchanged. No measurements or solves are used.
 */
template <typename Linalg>
[[nodiscard]] Status try_inject_and_reset(configuration::Ahrs::NominalState<Linalg> const & state,
                                          linalg::Matrix<Linalg, 3U, 3U> const & covariance,
                                          configuration::Ahrs::ErrorState<Linalg> const & error,
                                          typename Linalg::value_type minimum_quaternion_norm,
                                          configuration::Ahrs::NominalState<Linalg> & state_output,
                                          linalg::Matrix<Linalg, 3U, 3U> & covariance_output,
                                          configuration::Ahrs::ErrorState<Linalg> & error_output) noexcept
{
    return detail::try_inject_state_and_reset(state, covariance, error, minimum_quaternion_norm, state_output,
                                              covariance_output, error_output);
}

/** INS overload of the atomic injection/reset step. */
template <typename Linalg>
[[nodiscard]] Status try_inject_and_reset(configuration::Ins::NominalState<Linalg> const & state,
                                          linalg::Matrix<Linalg, 15U, 15U> const & covariance,
                                          configuration::Ins::ErrorState<Linalg> const & error,
                                          typename Linalg::value_type minimum_quaternion_norm,
                                          configuration::Ins::NominalState<Linalg> & state_output,
                                          linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
                                          configuration::Ins::ErrorState<Linalg> & error_output) noexcept
{
    return detail::try_inject_state_and_reset(state, covariance, error, minimum_quaternion_norm, state_output,
                                              covariance_output, error_output);
}

} /* end namespace formal_eskf */
