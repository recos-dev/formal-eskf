/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <formal_eskf/eskf/correction.hpp>
#include <formal_eskf/eskf/innovation.hpp>

namespace formal_eskf::runtime
{

enum class FusionDecision
{
    failed,
    rejected,
    fused,
};

/** Gate evidence for one prior; a passed gate neither corrects nor grants source permission. */
template <typename Linalg, std::size_t Size> struct MeasurementCheck
{
    Status status = Status::success;
    bool diagnostics_valid = false;
    bool rejected = true;
    linalg::Vector<Linalg, Size> innovation{};
    linalg::Vector<Linalg, Size> innovation_variance{};
    linalg::Vector<Linalg, Size> test_ratio{};

    [[nodiscard]] bool passed() const noexcept { return succeeded(status) && diagnostics_valid && !rejected; }
};

/** A fresh per-call result; only fused means state/covariance were published. */
template <typename Linalg, std::size_t Size> struct FusionResult
{
    FusionDecision decision = FusionDecision::failed;
    Status status = Status::success;
    // Remains true if the gate succeeded but correction subsequently failed.
    bool diagnostics_valid = false;
    linalg::Vector<Linalg, Size> innovation{};
    linalg::Vector<Linalg, Size> innovation_variance{};
    linalg::Vector<Linalg, Size> test_ratio{};

    FusionResult() = default;
    explicit FusionResult(MeasurementCheck<Linalg, Size> const & check) noexcept
        : status(check.status), diagnostics_valid(check.diagnostics_valid), innovation(check.innovation),
          innovation_variance(check.innovation_variance), test_ratio(check.test_ratio)
    {
        if (succeeded(status) && diagnostics_valid && check.rejected)
        {
            decision = FusionDecision::rejected;
        }
    }
};

namespace detail
{

template <typename Linalg>
[[nodiscard]] bool finite_fusion_state(configuration::Ahrs::NominalState<Linalg> const & state) noexcept
{
    return linalg::all_finite(state.q_nb.coefficients());
}

template <typename Linalg>
[[nodiscard]] bool finite_fusion_state(configuration::Ins::NominalState<Linalg> const & state) noexcept
{
    return linalg::all_finite(state.q_nb.coefficients()) && linalg::all_finite(state.p_n) &&
           linalg::all_finite(state.v_n) && linalg::all_finite(state.b_a) && linalg::all_finite(state.b_g);
}

/** Shared matrix arithmetic; nominal-state validation belongs to the explicit overloads. */
template <typename Linalg, std::size_t Size, std::size_t MeasurementSize>
[[nodiscard]] MeasurementCheck<Linalg, MeasurementSize> check_measurement_innovation(
    linalg::Matrix<Linalg, Size, Size> const & covariance, linalg::Vector<Linalg, MeasurementSize> const & r,
    linalg::Matrix<Linalg, MeasurementSize, Size> const & H,
    linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> const & V,
    typename Linalg::value_type minimum_quaternion_norm, typename Linalg::value_type gate_sigma) noexcept
{
    MeasurementCheck<Linalg, MeasurementSize> result;
    if (!scalar::is_finite<typename Linalg::scalar_math_type>(gate_sigma))
    {
        result.status = Status::non_finite_input;
        return result;
    }
    linalg::Matrix<Linalg, Size, MeasurementSize> PHt;
    linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> S;
    result.status = try_correction_covariances(covariance, r, H, V, minimum_quaternion_norm, PHt, S);
    if (!succeeded(result.status))
    {
        return result;
    }
    auto const variance = linalg::diagonal(S);
    InnovationCheck<Linalg, MeasurementSize> gate;
    result.status = try_check_innovation(r, variance, gate_sigma, gate);
    if (!succeeded(result.status))
    {
        return result;
    }
    result.diagnostics_valid = true;
    result.innovation = r;
    result.innovation_variance = variance;
    result.test_ratio = gate.test_ratio;
    result.rejected = gate.rejected;
    return result;
}

} /* end namespace detail */

/** Check one AHRS observation group without correction or output publication. */
template <typename Linalg, std::size_t MeasurementSize>
[[nodiscard]] MeasurementCheck<Linalg, MeasurementSize>
check_measurement(configuration::Ahrs::NominalState<Linalg> const & state,
                  linalg::Matrix<Linalg, 3U, 3U> const & covariance, linalg::Vector<Linalg, MeasurementSize> const & r,
                  linalg::Matrix<Linalg, MeasurementSize, 3U> const & H,
                  linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> const & V,
                  typename Linalg::value_type minimum_quaternion_norm, typename Linalg::value_type gate_sigma) noexcept
{
    if (!detail::finite_fusion_state(state))
    {
        MeasurementCheck<Linalg, MeasurementSize> result;
        result.status = Status::non_finite_input;
        return result;
    }
    return detail::check_measurement_innovation(covariance, r, H, V, minimum_quaternion_norm, gate_sigma);
}

/** Check one INS observation group without correction or output publication. */
template <typename Linalg, std::size_t MeasurementSize>
[[nodiscard]] MeasurementCheck<Linalg, MeasurementSize> check_measurement(
    configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15U, 15U> const & covariance,
    linalg::Vector<Linalg, MeasurementSize> const & r, linalg::Matrix<Linalg, MeasurementSize, 15U> const & H,
    linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> const & V,
    typename Linalg::value_type minimum_quaternion_norm, typename Linalg::value_type gate_sigma) noexcept
{
    if (!detail::finite_fusion_state(state))
    {
        MeasurementCheck<Linalg, MeasurementSize> result;
        result.status = Status::non_finite_input;
        return result;
    }
    return detail::check_measurement_innovation(covariance, r, H, V, minimum_quaternion_norm, gate_sigma);
}

/**
 * Gate one observation group, then correct using the same prior, r, H and V.
 * r = z - h(state) and H must be prepared for this prior and fusion horizon.
 * P is symmetric PSD, V symmetric SPD, and the prior quaternion unit, under
 * the existing correction contracts; passing the gate does not prove these.
 * Any component ratio > 1 rejects this entire group, without running correction.
 * Rejection reports success + rejected, not a numerical error or fusion success.
 *
 * Only fused changes outputs. Rejection and failure preserve both outputs,
 * including their independent aliases with the complete input state/covariance.
 * All other input/output storage is disjoint. Diagnostics are returned by value;
 * inspect diagnostics_valid, including on a later correction failure.
 * Finite nominal-state checks precede gating, but normalization/LLT checks remain
 * in correction and are not run for an outlier. Rejected does not certify state
 * health. Gate thresholds are explicit, with no clamp or default.
 *
 * Successive groups use the latest committed state/P and newly evaluated r/H.
 * Rejection affects only this call; it neither rolls back earlier fusion nor
 * blocks another group. Separate calls require independent measurement-noise
 * groups, not zero state cross-covariance. No source lifecycle, clock or retry.
 */
template <typename Linalg, std::size_t MeasurementSize>
[[nodiscard]] FusionResult<Linalg, MeasurementSize>
fuse_measurement(configuration::Ahrs::NominalState<Linalg> const & state,
                 linalg::Matrix<Linalg, 3U, 3U> const & covariance, linalg::Vector<Linalg, MeasurementSize> const & r,
                 linalg::Matrix<Linalg, MeasurementSize, 3U> const & H,
                 linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> const & V,
                 typename Linalg::value_type minimum_quaternion_norm, typename Linalg::value_type gate_sigma,
                 configuration::Ahrs::NominalState<Linalg> & state_output,
                 linalg::Matrix<Linalg, 3U, 3U> & covariance_output) noexcept
{
    auto const check = check_measurement(state, covariance, r, H, V, minimum_quaternion_norm, gate_sigma);
    FusionResult<Linalg, MeasurementSize> result{check};
    if (!check.passed())
    {
        return result;
    }
    result.status = try_correct(state, covariance, r, H, V, minimum_quaternion_norm, state_output, covariance_output);
    if (succeeded(result.status))
    {
        result.decision = FusionDecision::fused;
    }
    return result;
}

/** INS uses the same gate; bias permissions affect correction only, not S. */
template <typename Linalg, std::size_t MeasurementSize>
[[nodiscard]] FusionResult<Linalg, MeasurementSize> fuse_measurement(
    configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15U, 15U> const & covariance,
    linalg::Vector<Linalg, MeasurementSize> const & r, linalg::Matrix<Linalg, MeasurementSize, 15U> const & H,
    linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> const & V,
    typename Linalg::value_type minimum_quaternion_norm, typename Linalg::value_type gate_sigma,
    configuration::Ins::NominalState<Linalg> & state_output, linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
    configuration::Ins::BiasUpdate const & bias_update = {}) noexcept
{
    auto const check = check_measurement(state, covariance, r, H, V, minimum_quaternion_norm, gate_sigma);
    FusionResult<Linalg, MeasurementSize> result{check};
    if (!check.passed())
    {
        return result;
    }
    result.status =
        try_correct(state, covariance, r, H, V, minimum_quaternion_norm, state_output, covariance_output, bias_update);
    if (succeeded(result.status))
    {
        result.decision = FusionDecision::fused;
    }
    return result;
}

} /* end namespace formal_eskf::runtime */
