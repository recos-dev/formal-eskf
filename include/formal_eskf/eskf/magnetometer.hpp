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
 * Full three-axis magnetic-field measurement for AHRS and INS.
 */

#include <formal_eskf/eskf/correction.hpp>

namespace formal_eskf
{

namespace detail
{

template <std::size_t Size, std::size_t AttitudeOffset, typename Linalg>
[[nodiscard]] linalg::Matrix<Linalg, 3U, Size>
magnetometer_jacobian_from_prediction(linalg::Matrix<Linalg, 3U, 1U> const & h_m_b) noexcept
{
    static_assert(AttitudeOffset + 3U <= Size);
    linalg::Matrix<Linalg, 3U, Size> H;
    H.template set_block<0U, AttitudeOffset>(so3::hat<Linalg>(h_m_b));
    return H;
}

template <std::size_t AttitudeOffset, typename Linalg, typename State, std::size_t Size>
[[nodiscard]] Status try_correct_magnetometer_state_and_covariance(
    State const & state, linalg::Matrix<Linalg, Size, Size> const & covariance,
    linalg::Matrix<Linalg, 3U, 1U> const & m_b, linalg::Matrix<Linalg, 3U, 1U> const & m_n,
    linalg::Matrix<Linalg, 3U, 3U> const & V, typename Linalg::value_type minimum_quaternion_norm, State & state_output,
    linalg::Matrix<Linalg, Size, Size> & covariance_output) noexcept
{
    using value_type = typename Linalg::value_type;
    if (!linalg::all_finite(state.q_nb.coefficients()) || !linalg::all_finite(m_b) || !linalg::all_finite(m_n))
    {
        return Status::non_finite_input;
    }
    // Reject absent field vectors without squaring their magnitudes. Field
    // strength, inclination and disturbance thresholds belong to future gating.
    if (!(linalg::max_abs(m_b) > value_type{0}) || !(linalg::max_abs(m_n) > value_type{0}))
    {
        return Status::domain_error;
    }
    auto const h_m_b = so3::inverse_rotate(state.q_nb, m_n);
    if (!linalg::all_finite(h_m_b))
    {
        return Status::non_finite_result;
    }
    auto const r = m_b - h_m_b;
    if (!linalg::all_finite(r))
    {
        return Status::non_finite_result;
    }
    auto const H = magnetometer_jacobian_from_prediction<Size, AttitudeOffset>(h_m_b);
    return try_correct(state, covariance, r, H, V, minimum_quaternion_norm, state_output, covariance_output);
}

} /* end namespace detail */

/**
 * AHRS error-state Jacobian of h(q_nb) = R(q_nb)^T * m_n, not of m_b - h.
 * With q_true = q_nb * Exp(delta_theta_b),
 * h(q_true) = R(Exp(-delta_theta_b)) * h(q_nb)
 *           = h(q_nb) + hat(h(q_nb)) * delta_theta_b + O(||delta_theta_b||^2).
 * Thus H = hat(h(q_nb)); no factor of two is needed for rotation-vector errors.
 * This is an unchecked model evaluation; inputs and arithmetic must be finite.
 */
template <typename Linalg>
[[nodiscard]] linalg::Matrix<Linalg, 3U, 3U>
magnetometer_jacobian(configuration::Ahrs::NominalState<Linalg> const & state,
                      linalg::Matrix<Linalg, 3U, 1U> const & m_n) noexcept
{
    return detail::magnetometer_jacobian_from_prediction<3U, 0U>(so3::inverse_rotate(state.q_nb, m_n));
}

/**
 * INS Jacobian H = [0, 0, hat(h(q_nb)), 0, 0], following
 * [delta_p_n, delta_v_n, delta_theta_b, delta_b_a, delta_b_g].
 * The reference field is fixed for this update, not an estimated state.
 */
template <typename Linalg>
[[nodiscard]] linalg::Matrix<Linalg, 3U, 15U>
magnetometer_jacobian(configuration::Ins::NominalState<Linalg> const & state,
                      linalg::Matrix<Linalg, 3U, 1U> const & m_n) noexcept
{
    return detail::magnetometer_jacobian_from_prediction<15U, 6U>(so3::inverse_rotate(state.q_nb, m_n));
}

/**
 * Correct AHRS with calibrated body-frame magnetic field m_b in microtesla,
 * navigation-frame reference m_n in microtesla, and measurement covariance V
 * in microtesla^2. The model is m_b = R(q_nb)^T * m_n + noise, as in the magnetic
 * observation of Suh (2010), equation (3), expressed in our frame convention.
 * Neither field is normalized and no magnetic direction is hard-coded.
 *
 * Calibration (including hard/soft iron and sensor-to-body alignment), time
 * alignment and selection of a trustworthy NED reference field are caller
 * responsibilities. Reference/calibration uncertainty is not estimated here;
 * V describes the effective additive body-frame noise under this model and
 * must satisfy try_correct's assumptions, including independence from prior
 * state error. A nonzero field alone does not observe all three attitude DOFs.
 *
 * Uses the full three-axis residual, H and V in one batch try_correct call.
 * No gain projection is applied: magnetic observations can change tilt as well
 * as heading. Cholesky, Joseph, right-multiplicative Exp injection and covariance
 * reset follow try_correct. P must be symmetric PSD and V symmetric SPD.
 * Non-finite fields return non_finite_input, zero field vectors domain_error,
 * and non-finite prediction/residual arithmetic non_finite_result. Other checks
 * and failure statuses follow try_correct. Outputs commit together only on
 * success and may alias their respective inputs. No disturbance/innovation
 * gating, magnetic-bias state, heading-only policy or allocation is introduced.
 *
 * @see https://doi.org/10.1109/TIM.2010.2047157
 */
template <typename Linalg>
[[nodiscard]] Status
try_correct_magnetometer(configuration::Ahrs::NominalState<Linalg> const & state,
                         linalg::Matrix<Linalg, 3U, 3U> const & covariance, linalg::Matrix<Linalg, 3U, 1U> const & m_b,
                         linalg::Matrix<Linalg, 3U, 1U> const & m_n, linalg::Matrix<Linalg, 3U, 3U> const & V,
                         typename Linalg::value_type minimum_quaternion_norm,
                         configuration::Ahrs::NominalState<Linalg> & state_output,
                         linalg::Matrix<Linalg, 3U, 3U> & covariance_output) noexcept
{
    return detail::try_correct_magnetometer_state_and_covariance<0U>(
        state, covariance, m_b, m_n, V, minimum_quaternion_norm, state_output, covariance_output);
}

/**
 * INS overload of full magnetic-field correction. Although H directly observes
 * attitude only, all state components may be corrected through prior cross-
 * covariances. No gain entries are discarded. Preconditions, units and failure
 * atomicity follow the AHRS overload.
 */
template <typename Linalg>
[[nodiscard]] Status
try_correct_magnetometer(configuration::Ins::NominalState<Linalg> const & state,
                         linalg::Matrix<Linalg, 15U, 15U> const & covariance,
                         linalg::Matrix<Linalg, 3U, 1U> const & m_b, linalg::Matrix<Linalg, 3U, 1U> const & m_n,
                         linalg::Matrix<Linalg, 3U, 3U> const & V, typename Linalg::value_type minimum_quaternion_norm,
                         configuration::Ins::NominalState<Linalg> & state_output,
                         linalg::Matrix<Linalg, 15U, 15U> & covariance_output) noexcept
{
    return detail::try_correct_magnetometer_state_and_covariance<6U>(
        state, covariance, m_b, m_n, V, minimum_quaternion_norm, state_output, covariance_output);
}

} /* end namespace formal_eskf */
