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
 * Three-axis magnetic-field measurement with optional attitude tilt protection.
 */

#include <formal_eskf/eskf/correction.hpp>

// Select one magnetic correction policy for the complete build, not per call.
#ifndef ESKF_MAG_TILT
#define ESKF_MAG_TILT 1
#endif

static_assert(ESKF_MAG_TILT == 0 || ESKF_MAG_TILT == 1, "ESKF_MAG_TILT must be 0 or 1");

namespace formal_eskf
{

namespace detail
{

#if ESKF_MAG_TILT
/** Project body-frame attitude gain onto navigation Down; preserve all other rows. */
template <std::size_t AttitudeOffset, typename Linalg, std::size_t Size, std::size_t MeasurementSize>
[[nodiscard]] Status project_magnetic_gain(linalg::Matrix<Linalg, 3U, 3U> const & projector,
                                           linalg::Matrix<Linalg, Size, MeasurementSize> & gain)
{
    static_assert(AttitudeOffset + 3U <= Size);
    auto const attitude_gain = projector * gain.template block<AttitudeOffset, 0U, 3U, MeasurementSize>();
    if (!linalg::all_finite(attitude_gain))
    {
        return Status::non_finite_result;
    }
    gain.template set_block<AttitudeOffset, 0U>(attitude_gain);
    return Status::success;
}
#endif

/** Checked field prediction and residual, independent of the estimator configuration. */
template <typename Linalg>
[[nodiscard]] Status
magnetometer_residual(so3::UnitQuaternion<Linalg> const & q_nb, linalg::Vector<Linalg, 3U> const & m_b,
                      linalg::Vector<Linalg, 3U> const & m_n, linalg::Vector<Linalg, 3U> & prediction_output,
                      linalg::Vector<Linalg, 3U> & residual_output)
{
    using value_type = typename Linalg::value_type;
    if (!linalg::all_finite(q_nb.coefficients()) || !linalg::all_finite(m_b) || !linalg::all_finite(m_n))
    {
        return Status::non_finite_input;
    }
    // Reject absent field vectors without squaring their magnitudes. Field
    // strength, inclination and disturbance thresholds belong to future gating.
    if (!(linalg::max_abs(m_b) > value_type{0}) || !(linalg::max_abs(m_n) > value_type{0}))
    {
        return Status::domain_error;
    }
    auto const h_m_b = so3::inverse_rotate(q_nb, m_n);
    if (!linalg::all_finite(h_m_b))
    {
        return Status::non_finite_result;
    }
    auto const r = m_b - h_m_b;
    if (!linalg::all_finite(r))
    {
        return Status::non_finite_result;
    }
    prediction_output = h_m_b;
    residual_output = r;
    return Status::success;
}

#if ESKF_MAG_TILT
/** Construct the navigation-Down projector before constraining attitude gain rows. */
template <typename Linalg>
[[nodiscard]] Status magnetic_tilt_projector(so3::UnitQuaternion<Linalg> const & q_nb,
                                             linalg::Matrix<Linalg, 3U, 3U> & projector_output)
{
    using value_type = typename Linalg::value_type;
    using vector_type = linalg::Vector<Linalg, 3U>;
    auto const down_n = vector_type::from_row_major({value_type{0}, value_type{0}, value_type{1}});
    auto const down_b = so3::inverse_rotate(q_nb, down_n);
    if (!linalg::all_finite(down_b))
    {
        return Status::non_finite_result;
    }
    // Unit prior remains a caller precondition, as in the full-field model.
    // Do not normalize/repair the prior or replace the observation Jacobian.
    auto const projector = down_b * linalg::transpose(down_b);
    if (!linalg::all_finite(projector))
    {
        return Status::non_finite_result;
    }
    projector_output = projector;
    return Status::success;
}
#endif

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
linalg::Matrix<Linalg, 3U, 3U> magnetometer_jacobian(configuration::Ahrs::NominalState<Linalg> const & state,
                                                     linalg::Vector<Linalg, 3U> const & m_n)
{
    return so3::hat<Linalg>(so3::inverse_rotate(state.q_nb, m_n));
}

/**
 * INS Jacobian H = [0, 0, hat(h(q_nb)), 0, 0], following
 * [delta_p_n, delta_v_n, delta_theta_b, delta_b_a, delta_b_g].
 * The reference field is fixed for this update, not an estimated state.
 */
template <typename Linalg>
linalg::Matrix<Linalg, 3U, 15U> magnetometer_jacobian(configuration::Ins::NominalState<Linalg> const & state,
                                                      linalg::Vector<Linalg, 3U> const & m_n)
{
    linalg::Matrix<Linalg, 3U, 15U> H;
    H.template set_block<0U, 6U>(so3::hat<Linalg>(so3::inverse_rotate(state.q_nb, m_n)));
    return H;
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
 * must satisfy correct's assumptions, including independence from prior
 * state error. A nonzero field alone does not observe all three attitude DOFs.
 *
 * Uses the full three-axis residual, H and V in one batch correction.
 * ESKF_MAG_TILT=0 can change tilt and heading. ESKF_MAG_TILT=1 (default) projects
 * the attitude gain with u_b*u_b^T, where u_b=R(q_nb)^T*[0,0,1]^T. The same constrained
 * gain enters the error and Joseph updates before right-multiplicative Exp
 * injection and covariance reset. Only the attitude update is restricted; this
 * is not a scalar heading observation. The prior quaternion must be unit (not
 * revalidated or repaired here); P must be symmetric PSD and V symmetric SPD.
 * Tilt preservation is exact geometry under these premises, not a certified
 * floating-point residual bound. The policy is compile-time only; no mode
 * argument or runtime selection branch is present.
 * Non-finite fields return non_finite_input, zero field vectors domain_error,
 * and non-finite prediction/residual arithmetic non_finite_result. Other checks
 * and failure statuses follow correct. Outputs commit together only on
 * success and may alias their respective inputs. No disturbance/innovation
 * gating, magnetic-bias state, source lifecycle or allocation is introduced.
 *
 * @see https://doi.org/10.1109/TIM.2010.2047157
 */
template <typename Linalg>
[[nodiscard]] Status correct_magnetometer(
    configuration::Ahrs::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 3U, 3U> const & covariance,
    linalg::Vector<Linalg, 3U> const & m_b, linalg::Vector<Linalg, 3U> const & m_n,
    linalg::Matrix<Linalg, 3U, 3U> const & V, typename Linalg::value_type minimum_quaternion_norm,
    configuration::Ahrs::NominalState<Linalg> & state_output, linalg::Matrix<Linalg, 3U, 3U> & covariance_output)
{
    linalg::Vector<Linalg, 3U> h_m_b, r;
    Status const observation_status = detail::magnetometer_residual(state.q_nb, m_b, m_n, h_m_b, r);
    if (!succeeded(observation_status))
    {
        return observation_status;
    }
    auto const H = so3::hat<Linalg>(h_m_b);
#if ESKF_MAG_TILT
    linalg::Matrix<Linalg, 3U, 3U> projector;
    Status const projection_status = detail::magnetic_tilt_projector(state.q_nb, projector);
    if (!succeeded(projection_status))
    {
        return projection_status;
    }
    linalg::Matrix<Linalg, 3U, 3U> K;
    Status const gain_status = detail::compute_correction_gain(covariance, r, H, V, minimum_quaternion_norm, K);
    if (!succeeded(gain_status))
    {
        return gain_status;
    }
    Status const tilt_status = detail::project_magnetic_gain<0U>(projector, K);
    if (!succeeded(tilt_status))
    {
        return tilt_status;
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
#else
    return correct(state, covariance, r, H, V, minimum_quaternion_norm, state_output, covariance_output);
#endif
}

/**
 * INS overload of full magnetic-field correction. Although H directly observes
 * attitude only, all state components may be corrected through prior cross-
 * covariances. Optional bias permissions follow correct and default to all
 * enabled, independently of ESKF_MAG_TILT. Tilt protection preserves all
 * non-attitude gain rows except those disabled by bias permissions: position,
 * velocity and enabled biases can still change. This does not guarantee tilt
 * protection on later predictions. Preconditions, units and failure atomicity
 * follow the AHRS overload.
 */
template <typename Linalg>
[[nodiscard]] Status correct_magnetometer(
    configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15U, 15U> const & covariance,
    linalg::Vector<Linalg, 3U> const & m_b, linalg::Vector<Linalg, 3U> const & m_n,
    linalg::Matrix<Linalg, 3U, 3U> const & V, typename Linalg::value_type minimum_quaternion_norm,
    configuration::Ins::NominalState<Linalg> & state_output, linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
    configuration::Ins::BiasUpdate const & bias_update = {})
{
    linalg::Vector<Linalg, 3U> h_m_b, r;
    Status const observation_status = detail::magnetometer_residual(state.q_nb, m_b, m_n, h_m_b, r);
    if (!succeeded(observation_status))
    {
        return observation_status;
    }
    linalg::Matrix<Linalg, 3U, 15U> H;
    H.template set_block<0U, 6U>(so3::hat<Linalg>(h_m_b));
#if ESKF_MAG_TILT
    linalg::Matrix<Linalg, 3U, 3U> projector;
    Status const projection_status = detail::magnetic_tilt_projector(state.q_nb, projector);
    if (!succeeded(projection_status))
    {
        return projection_status;
    }
    linalg::Matrix<Linalg, 15U, 3U> K;
    Status const gain_status = detail::compute_correction_gain(covariance, r, H, V, minimum_quaternion_norm, K);
    if (!succeeded(gain_status))
    {
        return gain_status;
    }
    detail::constrain_bias_gain(bias_update, K);
    Status const tilt_status = detail::project_magnetic_gain<6U>(projector, K);
    if (!succeeded(tilt_status))
    {
        return tilt_status;
    }
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
#else
    return correct(state, covariance, r, H, V, minimum_quaternion_norm, state_output, covariance_output, bias_update);
#endif
}

} /* end namespace formal_eskf */
