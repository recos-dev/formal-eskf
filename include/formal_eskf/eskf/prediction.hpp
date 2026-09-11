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
 * Checked, backend-independent nominal-state prediction for AHRS and INS.
 */

#include <formal_eskf/eskf/configuration/ahrs.hpp>
#include <formal_eskf/eskf/configuration/ins.hpp>
#include <formal_eskf/eskf/types.hpp>
#include <formal_eskf/so3/operations.hpp>
#include <formal_eskf/so3/rotation_vector.hpp>

// Keep this setting identical in every translation unit of an application.
// It selects nominal and covariance attitude prediction, not so3::try_exp().
#ifndef ESKF_QUAT_APPROX
#define ESKF_QUAT_APPROX 0
#endif

static_assert(ESKF_QUAT_APPROX == 0 || ESKF_QUAT_APPROX == 1, "ESKF_QUAT_APPROX must be 0 or 1");

namespace formal_eskf
{

namespace detail
{

template <typename Math>
[[nodiscard]] Status validate_prediction_parameters(typename Math::value_type dt, typename Math::value_type dt_min,
                                                    typename Math::value_type dt_max,
                                                    typename Math::value_type minimum_quaternion_norm) noexcept
{
    using value_type = typename Math::value_type;

    if (!scalar::is_finite<Math>(dt) || !scalar::is_finite<Math>(dt_min) || !scalar::is_finite<Math>(dt_max) ||
        !scalar::is_finite<Math>(minimum_quaternion_norm))
    {
        return Status::non_finite_input;
    }
    if (!(dt_min > value_type{0}) || dt_max < dt_min || !(minimum_quaternion_norm > value_type{0}) ||
        minimum_quaternion_norm > value_type{1})
    {
        return Status::domain_error;
    }
    if (dt < dt_min || dt > dt_max)
    {
        return Status::out_of_range;
    }
    return Status::success;
}

template <typename Linalg>
[[nodiscard]] Status try_predict_attitude(so3::UnitQuaternion<Linalg> const & q_nb,
                                          typename Linalg::template vector_type<3U> const & angular_rate_b,
                                          typename Linalg::value_type dt, typename Linalg::value_type minimum_norm,
                                          so3::UnitQuaternion<Linalg> & output) noexcept
{
    if (!linalg::all_finite(q_nb.coefficients()) || !linalg::all_finite(angular_rate_b))
    {
        return Status::non_finite_input;
    }

    auto const rotation_vector = angular_rate_b * dt;
    if (!linalg::all_finite(rotation_vector))
    {
        return Status::non_finite_result;
    }

#if ESKF_QUAT_APPROX
    using value_type = typename Linalg::value_type;
    using vector4_type = typename Linalg::template vector_type<4U>;

    // Normalized Euler: normalize(q + 0.5 * q * [0, omega] * dt).
    // Form q * [1, rotation_vector/2] directly, without normalizing the
    // increment first. This branch has one normalization and no sin/cos.
    auto const half_angle = rotation_vector * value_type{0.5};
    value_type const q0 = q_nb.q0();
    value_type const q1 = q_nb.q1();
    value_type const q2 = q_nb.q2();
    value_type const q3 = q_nb.q3();
    value_type const x = half_angle(0U);
    value_type const y = half_angle(1U);
    value_type const z = half_angle(2U);

    vector4_type candidate;
    candidate.set(0U, q0 - q1 * x - q2 * y - q3 * z);
    candidate.set(1U, q1 + q0 * x + q2 * z - q3 * y);
    candidate.set(2U, q2 + q0 * y + q3 * x - q1 * z);
    candidate.set(3U, q3 + q0 * z + q1 * y - q2 * x);
    if (!linalg::all_finite(candidate))
    {
        return Status::non_finite_result;
    }
    return so3::UnitQuaternion<Linalg>::try_from_coefficients(candidate, minimum_norm, output);
#else
    so3::UnitQuaternion<Linalg> increment;
    Status const status = so3::try_exp(rotation_vector, minimum_norm, increment);
    if (!succeeded(status))
    {
        return status;
    }
    Status const compose_status = so3::try_compose_normalized(q_nb, increment, minimum_norm, output);
    return compose_status == Status::non_finite_input ? Status::non_finite_result : compose_status;
#endif
}

} /* end namespace detail */

/**
 * Predict attitude with a body-frame angular rate held constant over dt.
 *
 * The default is q_next = q_nb * Exp(angular_rate_b * dt). Define
 * ESKF_QUAT_APPROX=1 for normalized Euler instead.
 * AHRS assumes external gyroscope-bias calibration. Specific force and process
 * noise are not consumed; this operation does not propagate covariance.
 *
 * Requires finite consumed inputs, 0 < dt_min <= dt_max, dt in the inclusive
 * interval [dt_min, dt_max], and 0 < minimum_quaternion_norm <= 1. Invalid
 * parameters return domain_error; an invalid interval step returns out_of_range.
 * Output may alias state and remains unchanged on failure.
 */
template <typename Linalg>
[[nodiscard]] Status try_predict_nominal(configuration::Ahrs::NominalState<Linalg> const & state,
                                         ImuSample<Linalg> const & imu, typename Linalg::value_type dt,
                                         configuration::Ahrs::Parameters<Linalg> const & parameters,
                                         configuration::Ahrs::NominalState<Linalg> & output) noexcept
{
    Status const parameter_status = detail::validate_prediction_parameters<typename Linalg::scalar_math_type>(
        dt, parameters.dt_min, parameters.dt_max, parameters.minimum_quaternion_norm);
    if (!succeeded(parameter_status))
    {
        return parameter_status;
    }

    auto candidate = state;
    Status const status = detail::try_predict_attitude(state.q_nb, imu.angular_rate_b, dt,
                                                       parameters.minimum_quaternion_norm, candidate.q_nb);
    if (!succeeded(status))
    {
        return status;
    }
    output = candidate;
    return Status::success;
}

/**
 * Predict the INS nominal state with constant navigation-frame acceleration
 * during each step, following SICE 2023, "A Computationally Efficient GNSS/INS
 * Design of Multirotor based on Error-state Kalman Filter", equation (11).
 *
 * a_n = R(q_nb) * (specific_force_b - b_a) + gravity_n
 * p_next = p_n + v_n * dt + 0.5 * a_n * dt^2
 * v_next = v_n + a_n * dt
 * q_next = q_nb * Exp((angular_rate_b - b_g) * dt), or normalized Euler.
 *
 * All right-hand sides use the old state; no rotation-matrix cache is stored.
 * Bias nominal values remain constant. Gravity is supplied in navigation
 * coordinates; the caller is responsible for its physical frame and value.
 * Process noise is not consumed and covariance is not propagated.
 * Parameter validation and output atomicity follow the AHRS overload.
 */
template <typename Linalg>
[[nodiscard]] Status try_predict_nominal(configuration::Ins::NominalState<Linalg> const & state,
                                         ImuSample<Linalg> const & imu, typename Linalg::value_type dt,
                                         configuration::Ins::Parameters<Linalg> const & parameters,
                                         configuration::Ins::NominalState<Linalg> & output) noexcept
{
    using value_type = typename Linalg::value_type;

    Status const parameter_status = detail::validate_prediction_parameters<typename Linalg::scalar_math_type>(
        dt, parameters.dt_min, parameters.dt_max, parameters.minimum_quaternion_norm);
    if (!succeeded(parameter_status))
    {
        return parameter_status;
    }
    if (!linalg::all_finite(state.p_n) || !linalg::all_finite(state.v_n) || !linalg::all_finite(state.b_a) ||
        !linalg::all_finite(state.b_g) || !linalg::all_finite(imu.specific_force_b) ||
        !linalg::all_finite(imu.angular_rate_b) || !linalg::all_finite(parameters.gravity_n))
    {
        return Status::non_finite_input;
    }

    auto const specific_force_b = imu.specific_force_b - state.b_a;
    auto const angular_rate_b = imu.angular_rate_b - state.b_g;
    if (!linalg::all_finite(specific_force_b) || !linalg::all_finite(angular_rate_b))
    {
        return Status::non_finite_result;
    }

    auto candidate = state;
    Status const status = detail::try_predict_attitude(state.q_nb, angular_rate_b, dt,
                                                       parameters.minimum_quaternion_norm, candidate.q_nb);
    if (!succeeded(status))
    {
        return status;
    }

    auto const acceleration_n = so3::rotate(state.q_nb, specific_force_b) + parameters.gravity_n;
    candidate.p_n = state.p_n + state.v_n * dt + acceleration_n * (value_type{0.5} * dt * dt);
    candidate.v_n = state.v_n + acceleration_n * dt;
    if (!linalg::all_finite(acceleration_n) || !linalg::all_finite(candidate.p_n) || !linalg::all_finite(candidate.v_n))
    {
        return Status::non_finite_result;
    }

    output = candidate;
    return Status::success;
}

} /* end namespace formal_eskf */
