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
 * Shared fixed-size measurement correction for AHRS and INS.
 */

#include <formal_eskf/eskf/injection.hpp>

namespace formal_eskf
{

namespace detail
{

template <typename Linalg>
void unpack_correction(linalg::Matrix<Linalg, 3U, 1U> const & delta_x,
                       configuration::Ahrs::ErrorState<Linalg> & error) noexcept
{
    error.delta_theta_b = delta_x;
}

template <typename Linalg>
void unpack_correction(linalg::Matrix<Linalg, 15U, 1U> const & delta_x,
                       configuration::Ins::ErrorState<Linalg> & error) noexcept
{
    error.delta_p_n = delta_x.template segment<0U, 3U>();
    error.delta_v_n = delta_x.template segment<3U, 3U>();
    error.delta_theta_b = delta_x.template segment<6U, 3U>();
    error.delta_b_a = delta_x.template segment<9U, 3U>();
    error.delta_b_g = delta_x.template segment<12U, 3U>();
}

/** Input checks only; PSD/PD remain caller preconditions. */
template <typename Linalg, std::size_t Size, std::size_t MeasurementSize>
[[nodiscard]] Status validate_correction_inputs(linalg::Matrix<Linalg, Size, Size> const & covariance,
                                                linalg::Matrix<Linalg, MeasurementSize, 1U> const & r,
                                                linalg::Matrix<Linalg, MeasurementSize, Size> const & H,
                                                linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> const & V,
                                                typename Linalg::value_type minimum_quaternion_norm) noexcept
{
    using value_type = typename Linalg::value_type;
    using scalar_math_type = typename Linalg::scalar_math_type;

    if (!linalg::all_finite(covariance) || !linalg::all_finite(r) || !linalg::all_finite(H) || !linalg::all_finite(V) ||
        !scalar::is_finite<scalar_math_type>(minimum_quaternion_norm))
    {
        return Status::non_finite_input;
    }
    if (!(minimum_quaternion_norm > value_type{0}) || minimum_quaternion_norm > value_type{1} ||
        !linalg::is_symmetric(covariance, value_type{0}) || !linalg::is_symmetric(V, value_type{0}))
    {
        return Status::domain_error;
    }
    // Necessary covariance checks, not a full PSD/PD test. Zero prior variance
    // is allowed; measurement noise is required to be strictly positive definite.
    for (std::size_t index = 0U; index < Size; ++index)
    {
        if (covariance(index, index) < value_type{0})
        {
            return Status::domain_error;
        }
    }
    for (std::size_t index = 0U; index < MeasurementSize; ++index)
    {
        if (!(V(index, index) > value_type{0}))
        {
            return Status::domain_error;
        }
    }

    return Status::success;
}

/**
 * Checked linearized correction before nominal injection and coordinate reset.
 * Outputs are internal scratch objects, disjoint from all inputs and each other.
 * Publish the error vector and Joseph covariance only after every check succeeds.
 * The norm bound is validated here to preserve the public API's validation order.
 */
template <typename Linalg, std::size_t Size, std::size_t MeasurementSize>
[[nodiscard]] Status try_compute_correction(linalg::Matrix<Linalg, Size, Size> const & covariance,
                                            linalg::Matrix<Linalg, MeasurementSize, 1U> const & r,
                                            linalg::Matrix<Linalg, MeasurementSize, Size> const & H,
                                            linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> const & V,
                                            typename Linalg::value_type minimum_quaternion_norm,
                                            linalg::Matrix<Linalg, Size, 1U> & correction_output,
                                            linalg::Matrix<Linalg, Size, Size> & covariance_output) noexcept
{
    using covariance_type = linalg::Matrix<Linalg, Size, Size>;
    Status const input_status = validate_correction_inputs(covariance, r, H, V, minimum_quaternion_norm);
    if (!succeeded(input_status))
    {
        return input_status;
    }

    // Sola (274): K = P H^T (H P H^T + V)^-1. Reuse the cross-covariance
    // and solve K S = P H^T; do not form S^-1.
    auto const PHt = covariance * linalg::transpose(H);
    if (!linalg::all_finite(PHt))
    {
        return Status::non_finite_result;
    }
    auto S = H * PHt + V;
    // P and V have already passed exact-symmetry checks. Remove asymmetry
    // introduced by the products above, not asymmetry in caller-supplied data.
    Status const innovation_status = try_finish_covariance(S, S);
    if (!succeeded(innovation_status))
    {
        return innovation_status;
    }
    linalg::Matrix<Linalg, Size, MeasurementSize> K;
    Status const solve_status = linalg::right_solve_spd(PHt, S, K);
    if (!succeeded(solve_status))
    {
        return solve_status;
    }

    // Sola (275): the prior error mean is zero after the previous reset.
    auto const delta_x = K * r;
    auto const A = covariance_type::identity() - K * H;
    if (!linalg::all_finite(delta_x) || !linalg::all_finite(A))
    {
        return Status::non_finite_result;
    }
    // Sola footnote 26: Joseph form, using the same PRIOR P as in the gain.
    // Keep the complete V, including off-diagonal measurement correlations.
    auto const prior_term = linalg::sandwich(A, covariance);
    auto const noise_term = linalg::sandwich(K, V);
    auto P_corrected = prior_term + noise_term;
    Status const covariance_status = try_finish_covariance(P_corrected, P_corrected);
    if (!succeeded(covariance_status))
    {
        return covariance_status;
    }

    correction_output = delta_x;
    covariance_output = P_corrected;
    return Status::success;
}

template <typename Error, typename Linalg, typename State, std::size_t Size, std::size_t MeasurementSize>
[[nodiscard]] Status
try_correct_state_and_covariance(State const & state, linalg::Matrix<Linalg, Size, Size> const & covariance,
                                 linalg::Matrix<Linalg, MeasurementSize, 1U> const & r,
                                 linalg::Matrix<Linalg, MeasurementSize, Size> const & H,
                                 linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> const & V,
                                 typename Linalg::value_type minimum_quaternion_norm, State & state_output,
                                 linalg::Matrix<Linalg, Size, Size> & covariance_output) noexcept
{
    linalg::Matrix<Linalg, Size, 1U> delta_x;
    linalg::Matrix<Linalg, Size, Size> P_corrected;
    Status const correction_status =
        try_compute_correction(covariance, r, H, V, minimum_quaternion_norm, delta_x, P_corrected);
    if (!succeeded(correction_status))
    {
        return correction_status;
    }
    Error error;
    unpack_correction(delta_x, error);
    Error reset_error;
    // This operation validates the nominal state and commits both caller
    // outputs only after injection and covariance reset have succeeded.
    return try_inject_and_reset(state, P_corrected, error, minimum_quaternion_norm, state_output, covariance_output,
                                reset_error);
}

} /* end namespace detail */

/**
 * Correct AHRS using r = y - h(state), error-state Jacobian H and measurement
 * noise covariance V, all evaluated/prepared for this prior nominal state.
 * The prior error mean is zero. MeasurementSize is a compile-time dimension,
 * independent of the 4 nominal quaternion coefficients and 3 error coordinates.
 *
 * Uses Joan Sola, "Quaternion kinematics for the error-state Kalman filter",
 * (274)--(275), the Joseph update in footnote 26, then the existing local
 * right-multiplicative injection and covariance reset. ESKF_RESET_APPROX selects
 * reset only; ESKF_QUAT_APPROX does not change this correction.
 *
 * P must be symmetric PSD and V symmetric SPD. Full PSD/PD validity is a caller
 * precondition, not established by the runtime checks. Finite matrix inputs,
 * exact input symmetry, P's nonnegative diagonal, V's positive diagonal and
 * 0 < minimum_quaternion_norm <= 1 are checked. Other invalid nominal-state
 * inputs are handled by try_inject_and_reset. All off-diagonal entries of V
 * are retained. Only computed covariance matrices receive symmetry cleanup;
 * cleanup and Joseph form do not constitute a floating-point PSD guarantee.
 *
 * Invalid symmetry, variances or norm bounds return domain_error. Non-finite
 * data/arithmetic and failed innovation LLT are reported via the existing
 * Status values. Injection/reset failures propagate without modifying outputs.
 * Both outputs commit only on success and may alias their respective inputs.
 * No sensor model, residual wrapping, innovation gating or heap allocation is
 * introduced by this operation. The error mean is reset internally.
 *
 * @see https://arxiv.org/abs/1711.02508
 */
template <typename Linalg, std::size_t MeasurementSize>
[[nodiscard]] Status try_correct(configuration::Ahrs::NominalState<Linalg> const & state,
                                 linalg::Matrix<Linalg, 3U, 3U> const & covariance,
                                 linalg::Matrix<Linalg, MeasurementSize, 1U> const & r,
                                 linalg::Matrix<Linalg, MeasurementSize, 3U> const & H,
                                 linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> const & V,
                                 typename Linalg::value_type minimum_quaternion_norm,
                                 configuration::Ahrs::NominalState<Linalg> & state_output,
                                 linalg::Matrix<Linalg, 3U, 3U> & covariance_output) noexcept
{
    return detail::try_correct_state_and_covariance<configuration::Ahrs::ErrorState<Linalg>>(
        state, covariance, r, H, V, minimum_quaternion_norm, state_output, covariance_output);
}

/**
 * INS overload: H's columns follow
 * [delta_p_n, delta_v_n, delta_theta_b, delta_b_a, delta_b_g].
 * Uses the same correction as AHRS with 15 error coordinates. The resulting
 * full covariance, including attitude cross-covariances, is reset after
 * injection. Preconditions and failure atomicity follow the AHRS overload.
 */
template <typename Linalg, std::size_t MeasurementSize>
[[nodiscard]] Status try_correct(configuration::Ins::NominalState<Linalg> const & state,
                                 linalg::Matrix<Linalg, 15U, 15U> const & covariance,
                                 linalg::Matrix<Linalg, MeasurementSize, 1U> const & r,
                                 linalg::Matrix<Linalg, MeasurementSize, 15U> const & H,
                                 linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> const & V,
                                 typename Linalg::value_type minimum_quaternion_norm,
                                 configuration::Ins::NominalState<Linalg> & state_output,
                                 linalg::Matrix<Linalg, 15U, 15U> & covariance_output) noexcept
{
    return detail::try_correct_state_and_covariance<configuration::Ins::ErrorState<Linalg>>(
        state, covariance, r, H, V, minimum_quaternion_norm, state_output, covariance_output);
}

} /* end namespace formal_eskf */
