/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "kernel.hpp"
using Backend = codegen_test::Backend<SCALAR>;
using Types = codegen_test::Types<SCALAR>;
using S = SCALAR;
Types::nominal_state_type state, state_out;
Types::error_covariance_type P, P_out, F;
Types::imu_sample_type imu;
Types::parameter_type parameters;
formal_eskf::linalg::Matrix<Backend, 3U, 3U> R;
formal_eskf::linalg::Matrix<Backend, 12U, 12U> Q_i;
S dt = S{0.01};

extern "C" int call_predict()
{
    return static_cast<int>(formal_eskf::predict(state, P, imu, dt, parameters, state_out, P_out));
}

extern "C" int call_kernel()
{
    P_out = codegen_test::covariance_kernel<S>(F, P, R, Q_i);
    return 0;
}

extern "C" int setup(unsigned seed)
{
    state = {};
    P = decltype(P)::identity();
    parameters = {};
    parameters.dt_min = S{0.001};
    parameters.dt_max = S{1};
    parameters.minimum_quaternion_norm = S{1e-6};
    parameters.quaternion_squared_norm_tolerance = S{1e-4};
    parameters.gravity_n(2) = S{9.80665};
    formal_eskf::linalg::Vector<Backend, 3U> angle;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        angle(i) = S{0.05} * static_cast<S>(seed * i);
        imu.specific_force_b(i) = S{0.5} * static_cast<S>(seed + i);
        imu.angular_rate_b(i) = S{0.01} * static_cast<S>(seed * i);
        parameters.process_noise.specific_force_variance(i) = S{0.1} * static_cast<S>(i + 1U);
        parameters.process_noise.angular_rate_variance(i) = S{0.001} * static_cast<S>(i + 1U);
        parameters.process_noise.accelerometer_bias_random_walk_variance_density(i) = S{0.0001};
        parameters.process_noise.gyroscope_bias_random_walk_variance_density(i) = S{0.000001};
    }
    if (!formal_eskf::succeeded(formal_eskf::so3::exp(angle, parameters.minimum_quaternion_norm, state.q_nb)))
    {
        return 1;
    }
    if (!formal_eskf::succeeded(formal_eskf::discretize_process_noise(parameters.process_noise, dt, Q_i)))
    {
        return 2;
    }
    R = formal_eskf::so3::to_rotation_matrix(state.q_nb);
    decltype(R) T;
    if (!formal_eskf::succeeded(formal_eskf::detail::attitude_error_transition(imu.angular_rate_b, dt,
                                                                               parameters.minimum_quaternion_norm, T)))
    {
        return 3;
    }
    F = decltype(F)::identity();
    auto I_dt = decltype(R)::identity() * dt;
    F.template set_block<0U, 3U>(I_dt);
    F.template set_block<3U, 6U>(-R * formal_eskf::so3::hat<Backend>(imu.specific_force_b) * dt);
    F.template set_block<3U, 9U>(-R * dt);
    F.template set_block<6U, 6U>(T);
    F.template set_block<6U, 12U>(-I_dt);
    return 0;
}
