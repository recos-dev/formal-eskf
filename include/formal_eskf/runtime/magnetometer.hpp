/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <type_traits>

#include <formal_eskf/eskf/magnetometer.hpp>
#include <formal_eskf/runtime/magnetic_field.hpp>
#include <formal_eskf/runtime/measurement_fusion.hpp>

namespace formal_eskf::runtime
{

/** Field diagnostics and innovation diagnostics have separate validity flags. */
template <typename Linalg> struct MagnetometerFusionResult
{
    MagneticFieldCheck<typename Linalg::value_type> field{};
    FusionResult<Linalg, 3U> fusion{}; // The overall outcome, including field rejection/failure.
};

namespace detail
{

template <typename State, typename Linalg, std::size_t Size>
[[nodiscard]] MagnetometerFusionResult<Linalg> fuse_magnetometer_state(
    State const & state, linalg::Matrix<Linalg, Size, Size> const & covariance,
    linalg::Matrix<Linalg, 3U, 1U> const & m_b, linalg::Matrix<Linalg, 3U, 1U> const & m_n,
    linalg::Vector<Linalg, 3U> const * field_reference_n, linalg::Matrix<Linalg, 3U, 3U> const & V,
    MagneticFieldParameters<typename Linalg::value_type> const & field_parameters, bool heading_observable,
    typename Linalg::value_type minimum_quaternion_norm, typename Linalg::value_type gate_sigma, State & state_output,
    linalg::Matrix<Linalg, Size, Size> & covariance_output, configuration::Ins::BiasUpdate const & bias_update = {},
    MagneticFieldChecks const & field_checks = {}) noexcept
{
    MagnetometerFusionResult<Linalg> result;
    if (!finite_fusion_state(state))
    {
        result.fusion.status = Status::non_finite_input;
        return result;
    }
    result.field = check_magnetic_field(state.q_nb, m_b, m_n, field_reference_n, field_parameters, heading_observable,
                                        field_checks);
    result.fusion.status = result.field.status;
    if (!succeeded(result.field.status))
    {
        return result;
    }
    if (!result.field.accepted)
    {
        result.fusion.decision = FusionDecision::rejected;
        return result;
    }
    auto const h_m_b = so3::inverse_rotate(state.q_nb, m_n);
    auto const r = m_b - h_m_b;
    auto const H = magnetometer_jacobian(state, m_n);
    if (!linalg::all_finite(h_m_b) || !linalg::all_finite(r) || !linalg::all_finite(H))
    {
        result.fusion.status = Status::non_finite_result;
        return result;
    }
    auto const check = check_measurement(state, covariance, r, H, V, minimum_quaternion_norm, gate_sigma);
    result.fusion = FusionResult<Linalg, 3U>{check};
    if (!check.passed())
    {
        return result;
    }
    // Use the magnetic correction, not generic fuse_measurement: it owns the
    // compile-time attitude projection and the same constrained-gain Joseph update.
    if constexpr (std::is_same_v<State, configuration::Ins::NominalState<Linalg>>)
    {
        result.fusion.status = try_correct_magnetometer(state, covariance, m_b, m_n, V, minimum_quaternion_norm,
                                                        state_output, covariance_output, bias_update);
    }
    else
    {
        result.fusion.status = try_correct_magnetometer(state, covariance, m_b, m_n, V, minimum_quaternion_norm,
                                                        state_output, covariance_output);
    }
    if (succeeded(result.fusion.status))
    {
        result.fusion.decision = FusionDecision::fused;
    }
    return result;
}

} /* end namespace detail */

/**
 * Check field plausibility, gate all three residual components as one group,
 * then call the existing magnetic correction using this same prior and full V.
 * Only result.fusion.decision == fused publishes state/P. Field rejection leaves
 * innovation diagnostics invalid; a later correction failure retains valid gate
 * diagnostics. Each output may alias its respective whole prior, as in the core.
 * Rejection does not certify unused inputs: covariance/solver checks are deferred
 * until their stage. Unit attitude, PSD P, SPD V and calibration/time alignment
 * remain caller premises. Heading observability follows check_magnetic_field.
 * field_reference_n is borrowed for health checks only (nullptr means absent);
 * m_n owns the innovation and correction, even when no health reference exists.
 * No lifecycle, recovery timer, yaw reset, magnetic states or implicit tuning.
 */
template <typename Linalg>
[[nodiscard]] MagnetometerFusionResult<Linalg> fuse_magnetometer(
    configuration::Ahrs::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 3U, 3U> const & covariance,
    linalg::Matrix<Linalg, 3U, 1U> const & m_b, linalg::Matrix<Linalg, 3U, 1U> const & m_n,
    linalg::Vector<Linalg, 3U> const * field_reference_n, linalg::Matrix<Linalg, 3U, 3U> const & V,
    MagneticFieldParameters<typename Linalg::value_type> const & field_parameters, bool heading_observable,
    typename Linalg::value_type minimum_quaternion_norm, typename Linalg::value_type gate_sigma,
    configuration::Ahrs::NominalState<Linalg> & state_output, linalg::Matrix<Linalg, 3U, 3U> & covariance_output,
    MagneticFieldChecks const & field_checks = {}) noexcept
{
    return detail::fuse_magnetometer_state(state, covariance, m_b, m_n, field_reference_n, V, field_parameters,
                                           heading_observable, minimum_quaternion_norm, gate_sigma, state_output,
                                           covariance_output, {}, field_checks);
}

/** INS forwards per-axis bias permissions unchanged; neither gate uses them. */
template <typename Linalg>
[[nodiscard]] MagnetometerFusionResult<Linalg> fuse_magnetometer(
    configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15U, 15U> const & covariance,
    linalg::Matrix<Linalg, 3U, 1U> const & m_b, linalg::Matrix<Linalg, 3U, 1U> const & m_n,
    linalg::Vector<Linalg, 3U> const * field_reference_n, linalg::Matrix<Linalg, 3U, 3U> const & V,
    MagneticFieldParameters<typename Linalg::value_type> const & field_parameters, bool heading_observable,
    typename Linalg::value_type minimum_quaternion_norm, typename Linalg::value_type gate_sigma,
    configuration::Ins::NominalState<Linalg> & state_output, linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
    configuration::Ins::BiasUpdate const & bias_update = {}, MagneticFieldChecks const & field_checks = {}) noexcept
{
    return detail::fuse_magnetometer_state(state, covariance, m_b, m_n, field_reference_n, V, field_parameters,
                                           heading_observable, minimum_quaternion_norm, gate_sigma, state_output,
                                           covariance_output, bias_update, field_checks);
}

} /* end namespace formal_eskf::runtime */
