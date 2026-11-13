/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

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

/** Qualify the field before consuming covariance/noise; prepare the common residual. */
template <typename Linalg>
[[nodiscard]] MagnetometerFusionResult<Linalg>
check_magnetic_observation(so3::UnitQuaternion<Linalg> const & q_nb, linalg::Vector<Linalg, 3U> const & m_b,
                           linalg::Vector<Linalg, 3U> const & m_n, linalg::Vector<Linalg, 3U> const * field_reference_n,
                           MagneticFieldParameters<typename Linalg::value_type> const & field_parameters,
                           bool heading_observable, MagneticFieldChecks const & field_checks,
                           linalg::Vector<Linalg, 3U> & residual_output) noexcept
{
    MagnetometerFusionResult<Linalg> result;
    result.field =
        check_magnetic_field(q_nb, m_b, m_n, field_reference_n, field_parameters, heading_observable, field_checks);
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
    auto const h_m_b = so3::inverse_rotate(q_nb, m_n);
    auto const r = m_b - h_m_b;
    if (!linalg::all_finite(h_m_b) || !linalg::all_finite(r))
    {
        result.fusion.status = Status::non_finite_result;
        return result;
    }
    residual_output = r;
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
    linalg::Vector<Linalg, 3U> const & m_b, linalg::Vector<Linalg, 3U> const & m_n,
    linalg::Vector<Linalg, 3U> const * field_reference_n, linalg::Matrix<Linalg, 3U, 3U> const & V,
    MagneticFieldParameters<typename Linalg::value_type> const & field_parameters, bool heading_observable,
    typename Linalg::value_type minimum_quaternion_norm, typename Linalg::value_type gate_sigma,
    configuration::Ahrs::NominalState<Linalg> & state_output, linalg::Matrix<Linalg, 3U, 3U> & covariance_output,
    MagneticFieldChecks const & field_checks = {}) noexcept
{
    if (!detail::finite_fusion_state(state))
    {
        MagnetometerFusionResult<Linalg> result;
        result.fusion.status = Status::non_finite_input;
        return result;
    }
    linalg::Vector<Linalg, 3U> r;
    auto result = detail::check_magnetic_observation(state.q_nb, m_b, m_n, field_reference_n, field_parameters,
                                                     heading_observable, field_checks, r);
    if (!succeeded(result.fusion.status) || !result.field.accepted)
    {
        return result;
    }
    auto const H = magnetometer_jacobian(state, m_n);
    if (!linalg::all_finite(H))
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
    // Magnetic correction owns attitude projection and the same-gain Joseph update.
    result.fusion.status = try_correct_magnetometer(state, covariance, m_b, m_n, V, minimum_quaternion_norm,
                                                    state_output, covariance_output);
    if (succeeded(result.fusion.status))
    {
        result.fusion.decision = FusionDecision::fused;
    }
    return result;
}

/** INS correction applies the per-axis bias permissions; neither gate uses them. */
template <typename Linalg>
[[nodiscard]] MagnetometerFusionResult<Linalg> fuse_magnetometer(
    configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15U, 15U> const & covariance,
    linalg::Vector<Linalg, 3U> const & m_b, linalg::Vector<Linalg, 3U> const & m_n,
    linalg::Vector<Linalg, 3U> const * field_reference_n, linalg::Matrix<Linalg, 3U, 3U> const & V,
    MagneticFieldParameters<typename Linalg::value_type> const & field_parameters, bool heading_observable,
    typename Linalg::value_type minimum_quaternion_norm, typename Linalg::value_type gate_sigma,
    configuration::Ins::NominalState<Linalg> & state_output, linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
    configuration::Ins::BiasUpdate const & bias_update = {}, MagneticFieldChecks const & field_checks = {}) noexcept
{
    if (!detail::finite_fusion_state(state))
    {
        MagnetometerFusionResult<Linalg> result;
        result.fusion.status = Status::non_finite_input;
        return result;
    }
    linalg::Vector<Linalg, 3U> r;
    auto result = detail::check_magnetic_observation(state.q_nb, m_b, m_n, field_reference_n, field_parameters,
                                                     heading_observable, field_checks, r);
    if (!succeeded(result.fusion.status) || !result.field.accepted)
    {
        return result;
    }
    auto const H = magnetometer_jacobian(state, m_n);
    if (!linalg::all_finite(H))
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
    // Magnetic correction owns attitude projection and the same-gain Joseph update.
    result.fusion.status = try_correct_magnetometer(state, covariance, m_b, m_n, V, minimum_quaternion_norm,
                                                    state_output, covariance_output, bias_update);
    if (succeeded(result.fusion.status))
    {
        result.fusion.decision = FusionDecision::fused;
    }
    return result;
}

} /* end namespace formal_eskf::runtime */
