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
 * Error covariance and atomic nominal/covariance prediction for AHRS and INS.
 */

#include <formal_eskf/eskf/prediction.hpp>
#include <formal_eskf/eskf/process_noise.hpp>
#include <formal_eskf/eskf/covariance.hpp>

namespace formal_eskf
{

namespace detail
{

template <typename Linalg>
[[nodiscard]] Status try_attitude_error_transition(linalg::Matrix<Linalg, 3U, 1U> const & angular_rate_b,
                                                   typename Linalg::value_type dt,
                                                   typename Linalg::value_type minimum_norm,
                                                   linalg::Matrix<Linalg, 3U, 3U> & output) noexcept
{
    auto const rotation_vector = angular_rate_b * dt;
    if (!linalg::all_finite(rotation_vector))
    {
        return Status::non_finite_result;
    }
#if ESKF_QUAT_APPROX
    // First-order local-error transition, not the exact Jacobian of
    // normalized Euler. The minus sign approximates R(Exp(theta))^T.
    (void)minimum_norm;
    output = linalg::Matrix<Linalg, 3U, 3U>::identity() - so3::hat<Linalg>(rotation_vector);
#else
    so3::UnitQuaternion<Linalg> increment;
    Status const status = so3::try_exp(rotation_vector, minimum_norm, increment);
    if (!succeeded(status))
    {
        return status;
    }
    output = linalg::transpose(so3::to_rotation_matrix(increment));
#endif
    return Status::success;
}

} /* end namespace detail */

/**
 * Propagate AHRS covariance using P_next = F_x P F_x^T + Q_i.
 * F_x = R(Exp(angular_rate_b * dt))^T by default; approximation mode uses
 * I - hat(angular_rate_b * dt). Q_i follows SICE 2023 equation (15).
 *
 * Covariance must be symmetric positive semidefinite on entry. This is a
 * caller precondition, not a runtime PSD test or a floating-point PSD proof.
 * Finite consumed inputs, time bounds, and noise parameters are checked.
 * Success restores symmetry; failure leaves output unchanged. Output may
 * alias covariance. AHRS absolute attitude and specific force are not used.
 */
template <typename Linalg>
[[nodiscard]] Status try_predict_covariance(configuration::Ahrs::NominalState<Linalg> const &,
                                            linalg::Matrix<Linalg, 3U, 3U> const & covariance,
                                            ImuSample<Linalg> const & imu, typename Linalg::value_type dt,
                                            configuration::Ahrs::Parameters<Linalg> const & parameters,
                                            linalg::Matrix<Linalg, 3U, 3U> & output) noexcept
{
    Status const parameter_status = detail::validate_prediction_parameters<typename Linalg::scalar_math_type>(
        dt, parameters.dt_min, parameters.dt_max, parameters.minimum_quaternion_norm);
    if (!succeeded(parameter_status))
    {
        return parameter_status;
    }
    if (!linalg::all_finite(covariance) || !linalg::all_finite(imu.angular_rate_b))
    {
        return Status::non_finite_input;
    }

    linalg::Matrix<Linalg, 3U, 3U> Q_i;
    Status const noise_status = try_discretize_process_noise(parameters.process_noise, dt, Q_i);
    if (!succeeded(noise_status))
    {
        return noise_status;
    }
    linalg::Matrix<Linalg, 3U, 3U> F_x;
    Status const transition_status =
        detail::try_attitude_error_transition(imu.angular_rate_b, dt, parameters.minimum_quaternion_norm, F_x);
    if (!succeeded(transition_status))
    {
        return transition_status;
    }
    auto candidate = linalg::sandwich(F_x, covariance) + Q_i;
    return detail::try_finish_covariance(candidate, output);
}

/**
 * Propagate INS covariance following SICE 2023, "A Computationally Efficient
 * GNSS/INS Design of Multirotor based on Error-state Kalman Filter",
 * equations (14), (15), and (18).
 *
 * F_x follows [delta_p_n, delta_v_n, delta_theta_b, delta_b_a, delta_b_g]:
 *   [I, I*dt,             0,     0,     0]
 *   [0,    I, -R*hat(a)*dt, -R*dt,     0]
 *   [0,    0,       R_d^T,     0, -I*dt]
 *   [0,    0,             0,     I,     0]
 *   [0,    0,             0,     0,     I]
 * Here a = specific_force_b - b_a and R_d = R(Exp((angular_rate_b-b_g)*dt)).
 * Approximation mode replaces R_d^T with I-hat((angular_rate_b-b_g)*dt).
 * All linearization quantities use the OLD nominal state.
 *
 * F_i = [B C] follows Sola equation (451) and Appendix E.2.1's discussion
 * after (457), omitting the gravity-error rows because gravity is fixed here.
 * It injects independent body-axis impulses into navigation velocity through
 * -R, local body-frame attitude error through -I, and body-frame biases
 * through I. Its sandwich with diagonal Q_i is assembled in blocks without
 * storing a sparse 15x12 matrix. Only isotropic noise permits the paper's
 * identity velocity injection with the same Q_i; unequal axis variances
 * require retaining R * Sigma_a_b * R^T * dt^2 in the velocity block.
 *
 * This is the paper's truncated covariance model, not the full Jacobian of
 * the constant-acceleration nominal integrator or exact joint discretization.
 * In particular, higher-order position couplings and direct position-noise
 * injection are omitted. Bias coupling remains first order even in Exp mode.
 * Covariance preconditions and failure behavior follow the AHRS overload.
 * Position, velocity, and gravity are not consumed by this operation.
 * The consumed prior attitude is checked against quaternion_squared_norm_tolerance,
 * as in nominal prediction. AHRS covariance consumes neither field.
 */
template <typename Linalg>
[[nodiscard]] Status try_predict_covariance(configuration::Ins::NominalState<Linalg> const & state,
                                            linalg::Matrix<Linalg, 15U, 15U> const & covariance,
                                            ImuSample<Linalg> const & imu, typename Linalg::value_type dt,
                                            configuration::Ins::Parameters<Linalg> const & parameters,
                                            linalg::Matrix<Linalg, 15U, 15U> & output) noexcept
{
    using matrix3_type = linalg::Matrix<Linalg, 3U, 3U>;
    using covariance_type = linalg::Matrix<Linalg, 15U, 15U>;

    Status const parameter_status = detail::validate_prediction_parameters<typename Linalg::scalar_math_type>(
        dt, parameters.dt_min, parameters.dt_max, parameters.minimum_quaternion_norm);
    if (!succeeded(parameter_status))
    {
        return parameter_status;
    }
    Status const quaternion_status =
        detail::validate_prediction_quaternion(state.q_nb, parameters.quaternion_squared_norm_tolerance);
    if (!succeeded(quaternion_status))
    {
        return quaternion_status;
    }
    if (!linalg::all_finite(covariance) || !linalg::all_finite(state.q_nb.coefficients()) ||
        !linalg::all_finite(state.b_a) || !linalg::all_finite(state.b_g) || !linalg::all_finite(imu.specific_force_b) ||
        !linalg::all_finite(imu.angular_rate_b))
    {
        return Status::non_finite_input;
    }
    auto const specific_force_b = imu.specific_force_b - state.b_a;
    auto const angular_rate_b = imu.angular_rate_b - state.b_g;
    if (!linalg::all_finite(specific_force_b) || !linalg::all_finite(angular_rate_b))
    {
        return Status::non_finite_result;
    }

    linalg::Matrix<Linalg, 12U, 12U> Q_i;
    Status const noise_status = try_discretize_process_noise(parameters.process_noise, dt, Q_i);
    if (!succeeded(noise_status))
    {
        return noise_status;
    }
    matrix3_type attitude_transition;
    Status const transition_status = detail::try_attitude_error_transition(
        angular_rate_b, dt, parameters.minimum_quaternion_norm, attitude_transition);
    if (!succeeded(transition_status))
    {
        return transition_status;
    }

    auto const R = so3::to_rotation_matrix(state.q_nb);
    auto const I_dt = matrix3_type::identity() * dt;
    covariance_type F_x = covariance_type::identity();
    F_x.template set_block<0U, 3U>(I_dt);
    F_x.template set_block<3U, 6U>(-R * so3::hat<Linalg>(specific_force_b) * dt);
    F_x.template set_block<3U, 9U>(-R * dt);
    F_x.template set_block<6U, 6U>(attitude_transition);
    F_x.template set_block<6U, 12U>(-I_dt);
    if (!linalg::all_finite(F_x))
    {
        return Status::non_finite_result;
    }

    auto candidate = linalg::sandwich(F_x, covariance);
    candidate.template set_block<3U, 3U>(candidate.template block<3U, 3U, 3U, 3U>() +
                                         linalg::sandwich(R, Q_i.template block<0U, 0U, 3U, 3U>()));
    candidate.template set_block<6U, 6U>(candidate.template block<6U, 6U, 3U, 3U>() +
                                         Q_i.template block<3U, 3U, 3U, 3U>());
    candidate.template set_block<9U, 9U>(candidate.template block<9U, 9U, 3U, 3U>() +
                                         Q_i.template block<6U, 6U, 3U, 3U>());
    candidate.template set_block<12U, 12U>(candidate.template block<12U, 12U, 3U, 3U>() +
                                           Q_i.template block<9U, 9U, 3U, 3U>());
    return detail::try_finish_covariance(candidate, output);
}

namespace detail
{

template <typename Linalg, typename State, typename Parameters, std::size_t Size>
[[nodiscard]] Status try_predict_state_and_covariance(State const & state,
                                                      linalg::Matrix<Linalg, Size, Size> const & covariance,
                                                      ImuSample<Linalg> const & imu, typename Linalg::value_type dt,
                                                      Parameters const & parameters, State & state_output,
                                                      linalg::Matrix<Linalg, Size, Size> & covariance_output) noexcept
{
    auto state_candidate = state;
    Status const nominal_status = try_predict_nominal(state, imu, dt, parameters, state_candidate);
    if (!succeeded(nominal_status))
    {
        return nominal_status;
    }
    linalg::Matrix<Linalg, Size, Size> covariance_candidate;
    Status const covariance_status =
        try_predict_covariance(state, covariance, imu, dt, parameters, covariance_candidate);
    if (!succeeded(covariance_status))
    {
        return covariance_status;
    }
    state_output = state_candidate;
    covariance_output = covariance_candidate;
    return Status::success;
}

} /* end namespace detail */

/**
 * Predict nominal state and covariance from the same old state. Both outputs
 * commit only on success and may alias their respective inputs. Preconditions
 * are the union of try_predict_nominal and try_predict_covariance.
 */
template <typename Linalg>
[[nodiscard]] Status try_predict(configuration::Ahrs::NominalState<Linalg> const & state,
                                 linalg::Matrix<Linalg, 3U, 3U> const & covariance, ImuSample<Linalg> const & imu,
                                 typename Linalg::value_type dt,
                                 configuration::Ahrs::Parameters<Linalg> const & parameters,
                                 configuration::Ahrs::NominalState<Linalg> & state_output,
                                 linalg::Matrix<Linalg, 3U, 3U> & covariance_output) noexcept
{
    return detail::try_predict_state_and_covariance(state, covariance, imu, dt, parameters, state_output,
                                                    covariance_output);
}

/** INS overload of the atomic nominal/covariance prediction step. */
template <typename Linalg>
[[nodiscard]] Status try_predict(configuration::Ins::NominalState<Linalg> const & state,
                                 linalg::Matrix<Linalg, 15U, 15U> const & covariance, ImuSample<Linalg> const & imu,
                                 typename Linalg::value_type dt,
                                 configuration::Ins::Parameters<Linalg> const & parameters,
                                 configuration::Ins::NominalState<Linalg> & state_output,
                                 linalg::Matrix<Linalg, 15U, 15U> & covariance_output) noexcept
{
    return detail::try_predict_state_and_covariance(state, covariance, imu, dt, parameters, state_output,
                                                    covariance_output);
}

} /* end namespace formal_eskf */
