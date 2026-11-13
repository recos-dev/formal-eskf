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

#include <cstddef>

#include <formal_eskf/eskf/injection.hpp>

namespace formal_eskf
{

namespace detail
{

/** Apply INS bias permissions after the solve, before error and Joseph updates. */
template <typename Linalg, std::size_t MeasurementSize>
void constrain_bias_gain(configuration::Ins::BiasUpdate const & bias_update,
                         linalg::Matrix<Linalg, 15U, MeasurementSize> & K)
{
    using value_type = typename Linalg::value_type;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        for (std::size_t column = 0U; column < MeasurementSize; ++column)
        {
            if (!bias_update.accelerometer[axis])
            {
                K(9U + axis, column) = value_type{0};
            }
            if (!bias_update.gyroscope[axis])
            {
                K(12U + axis, column) = value_type{0};
            }
        }
    }
}

template <typename Linalg>
void unpack_correction(linalg::Vector<Linalg, 3U> const & delta_x, configuration::Ahrs::ErrorState<Linalg> & error)
{
    error.delta_theta_b = delta_x;
}

template <typename Linalg>
void unpack_correction(linalg::Vector<Linalg, 15U> const & delta_x, configuration::Ins::ErrorState<Linalg> & error)
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
                                                linalg::Vector<Linalg, MeasurementSize> const & r,
                                                linalg::Matrix<Linalg, MeasurementSize, Size> const & H,
                                                linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> const & V,
                                                typename Linalg::value_type minimum_quaternion_norm)
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

} /* end namespace detail */

/**
 * Validate a correction's matrix inputs and compute PHt = P H^T and S = H PHt + V.
 * Uses the same validation and arithmetic as correct, without solving for
 * gain or inspecting the nominal state. Suitable for innovation gating before
 * attempting correction; success does not establish PSD P, SPD V or a unit prior.
 * r and the normalization bound are checked in correction order, even though
 * they do not enter these products. Only computed S receives symmetry cleanup.
 * Both outputs publish together on success; outputs are disjoint from each other and from inputs.
 * Results describe this prior and linearization only, not a reusable correction
 * permission. Correction remains independently checked when subsequently called.
 */
template <typename Linalg, std::size_t Size, std::size_t MeasurementSize>
[[nodiscard]] Status
correction_covariances(linalg::Matrix<Linalg, Size, Size> const & covariance,
                       linalg::Vector<Linalg, MeasurementSize> const & r,
                       linalg::Matrix<Linalg, MeasurementSize, Size> const & H,
                       linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> const & V,
                       typename Linalg::value_type minimum_quaternion_norm,
                       linalg::Matrix<Linalg, Size, MeasurementSize> & cross_covariance_output,
                       linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> & innovation_covariance_output)
{
    Status const input_status = detail::validate_correction_inputs(covariance, r, H, V, minimum_quaternion_norm);
    if (!succeeded(input_status))
    {
        return input_status;
    }

    auto const PHt = covariance * linalg::transpose(H);
    if (!linalg::all_finite(PHt))
    {
        return Status::non_finite_result;
    }
    auto S = H * PHt + V;
    Status const innovation_status = detail::finish_covariance(S, S);
    if (!succeeded(innovation_status))
    {
        return innovation_status;
    }
    cross_covariance_output = PHt;
    innovation_covariance_output = S;
    return Status::success;
}

namespace detail
{

/** Checked unconstrained gain; output is disjoint scratch, published only on success. */
template <typename Linalg, std::size_t Size, std::size_t MeasurementSize>
[[nodiscard]] Status compute_correction_gain(linalg::Matrix<Linalg, Size, Size> const & covariance,
                                             linalg::Vector<Linalg, MeasurementSize> const & r,
                                             linalg::Matrix<Linalg, MeasurementSize, Size> const & H,
                                             linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> const & V,
                                             typename Linalg::value_type minimum_quaternion_norm,
                                             linalg::Matrix<Linalg, Size, MeasurementSize> & gain_output)
{
    linalg::Matrix<Linalg, Size, MeasurementSize> PHt;
    linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> S;
    Status const innovation_status = correction_covariances(covariance, r, H, V, minimum_quaternion_norm, PHt, S);
    if (!succeeded(innovation_status))
    {
        return innovation_status;
    }
    // Sola (274): solve K S = P H^T; do not form S^-1.
    linalg::Matrix<Linalg, Size, MeasurementSize> K;
    Status const solve_status = linalg::right_solve_spd(PHt, S, K);
    if (!succeeded(solve_status))
    {
        return solve_status;
    }

    gain_output = K;
    return Status::success;
}

/**
 * Use the same final gain for the error vector and Joseph covariance.
 * Inputs have passed correction validation; K includes all requested constraints.
 * Outputs are separate internal scratch objects, disjoint from inputs.
 * Publish both only after every arithmetic check succeeds.
 */
template <typename Linalg, std::size_t Size, std::size_t MeasurementSize>
[[nodiscard]] Status apply_correction_gain(linalg::Matrix<Linalg, Size, Size> const & covariance,
                                           linalg::Vector<Linalg, MeasurementSize> const & r,
                                           linalg::Matrix<Linalg, MeasurementSize, Size> const & H,
                                           linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> const & V,
                                           linalg::Matrix<Linalg, Size, MeasurementSize> const & K,
                                           linalg::Vector<Linalg, Size> & correction_output,
                                           linalg::Matrix<Linalg, Size, Size> & covariance_output)
{
    using covariance_type = linalg::Matrix<Linalg, Size, Size>;
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
    Status const covariance_status = finish_covariance(P_corrected, P_corrected);
    if (!succeeded(covariance_status))
    {
        return covariance_status;
    }

    correction_output = delta_x;
    covariance_output = P_corrected;
    return Status::success;
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
 * inputs are handled by inject_and_reset. All off-diagonal entries of V
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
[[nodiscard]] Status
correct(configuration::Ahrs::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 3U, 3U> const & covariance,
        linalg::Vector<Linalg, MeasurementSize> const & r, linalg::Matrix<Linalg, MeasurementSize, 3U> const & H,
        linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> const & V,
        typename Linalg::value_type minimum_quaternion_norm, configuration::Ahrs::NominalState<Linalg> & state_output,
        linalg::Matrix<Linalg, 3U, 3U> & covariance_output)
{
    linalg::Matrix<Linalg, 3U, MeasurementSize> K;
    Status const gain_status = detail::compute_correction_gain(covariance, r, H, V, minimum_quaternion_norm, K);
    if (!succeeded(gain_status))
    {
        return gain_status;
    }

    linalg::Vector<Linalg, 3U> delta_x;
    linalg::Matrix<Linalg, 3U, 3U> P_corrected;
    Status const correction_status = detail::apply_correction_gain(covariance, r, H, V, K, delta_x, P_corrected);
    if (!succeeded(correction_status))
    {
        return correction_status;
    }
    configuration::Ahrs::ErrorState<Linalg> error, reset_error;
    detail::unpack_correction(delta_x, error);
    return inject_and_reset(state, P_corrected, error, minimum_quaternion_norm, state_output, covariance_output,
                            reset_error);
}

/**
 * INS overload: H's columns follow
 * [delta_p_n, delta_v_n, delta_theta_b, delta_b_a, delta_b_g].
 * Uses the same correction as AHRS with 15 error coordinates. The resulting
 * full covariance, including attitude cross-covariances, is reset after
 * injection. Preconditions and failure atomicity follow the AHRS overload.
 * Optional bias permissions default to all enabled. Disabled axes retain their
 * bias estimates; the same constrained gain is used for the Joseph covariance.
 * This does not freeze prediction's bias random walk or decide IMU health.
 */
template <typename Linalg, std::size_t MeasurementSize>
[[nodiscard]] Status
correct(configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15U, 15U> const & covariance,
        linalg::Vector<Linalg, MeasurementSize> const & r, linalg::Matrix<Linalg, MeasurementSize, 15U> const & H,
        linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> const & V,
        typename Linalg::value_type minimum_quaternion_norm, configuration::Ins::NominalState<Linalg> & state_output,
        linalg::Matrix<Linalg, 15U, 15U> & covariance_output, configuration::Ins::BiasUpdate const & bias_update = {})
{
    linalg::Matrix<Linalg, 15U, MeasurementSize> K;
    Status const gain_status = detail::compute_correction_gain(covariance, r, H, V, minimum_quaternion_norm, K);
    if (!succeeded(gain_status))
    {
        return gain_status;
    }
    detail::constrain_bias_gain(bias_update, K);

    linalg::Vector<Linalg, 15U> delta_x;
    linalg::Matrix<Linalg, 15U, 15U> P_corrected;
    Status const correction_status = detail::apply_correction_gain(covariance, r, H, V, K, delta_x, P_corrected);
    if (!succeeded(correction_status))
    {
        return correction_status;
    }
    configuration::Ins::ErrorState<Linalg> error, reset_error;
    detail::unpack_correction(delta_x, error);
    return inject_and_reset(state, P_corrected, error, minimum_quaternion_norm, state_output, covariance_output,
                            reset_error);
}

} /* end namespace formal_eskf */
