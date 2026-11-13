/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/eskf/eskf.hpp>

#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string_view>
#include <type_traits>

#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{

using formal_eskf::Status;
using formal_eskf::configuration::Ahrs;
using formal_eskf::configuration::Ins;
using formal_eskf::test::Context;

// Independent trajectory: fixed position, level attitude, constant positive yaw
// rate. Gravity and magnetic observations follow analytic sine/cosine, without
// using the estimator's rotation functions to manufacture their expected values.
template <typename Backend, typename Configuration> void test_sequence(Context & test, std::string_view profile)
{
    using value_type = typename Backend::value_type;
    using types = formal_eskf::EskfTypes<Backend, Configuration>;
    using vector3_type = formal_eskf::linalg::Vector<Backend, 3U>;
    using matrix3_type = formal_eskf::linalg::Matrix<Backend, 3U, 3U>;
    using covariance_type = typename types::error_covariance_type;
    constexpr value_type dt = static_cast<value_type>(0.02);
    constexpr value_type yaw_rate = static_cast<value_type>(0.2);
    constexpr value_type minimum_norm = static_cast<value_type>(1e-6);
    constexpr std::size_t steps = 50U;
    constexpr value_type initial_error[] = {static_cast<value_type>(0.1), static_cast<value_type>(-0.08),
                                            static_cast<value_type>(0.15)};
    constexpr value_type gravity_values[] = {0, 0, 10};
    constexpr value_type magnetic_values[] = {1, 0, static_cast<value_type>(0.25)};
    auto const gravity = vector3_type::from_row_major(gravity_values);
    auto const magnetic_n = vector3_type::from_row_major(magnetic_values);
    auto const observation_noise = matrix3_type::identity() * static_cast<value_type>(0.02);
    typename types::nominal_state_type state;
    auto P = covariance_type::identity() * static_cast<value_type>(0.2);
    typename types::parameter_type parameters;
    parameters.dt_min = dt;
    parameters.dt_max = dt;
    parameters.minimum_quaternion_norm = minimum_norm;
    parameters.quaternion_squared_norm_tolerance = value_type{32} * std::numeric_limits<value_type>::epsilon();
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        parameters.process_noise.angular_rate_variance(axis) = static_cast<value_type>(1e-4);
    }
    formal_eskf::ImuSample<Backend> imu;
    imu.specific_force_b = -gravity;
    imu.angular_rate_b(2U) = yaw_rate;
    test.expect(formal_eskf::so3::try_exp(vector3_type::from_row_major(initial_error), minimum_norm, state.q_nb) ==
                    Status::success,
                profile, "initialize a nonzero attitude error");
    if constexpr (std::is_same_v<Configuration, Ins>)
    {
        parameters.gravity_n = gravity;
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            state.p_n(axis) = value_type{1};
            state.v_n(axis) = static_cast<value_type>(0.3);
        }
        // Known zero biases isolate the observable navigation/attitude system.
        // Permission masks and bias learning have their own dedicated tests.
        for (std::size_t axis = 9U; axis < types::error_state_dimension; ++axis)
        {
            P(axis, axis) = value_type{0};
        }
    }

    for (std::size_t step = 1U; step <= steps; ++step)
    {
        test.expect(formal_eskf::try_predict(state, P, imu, dt, parameters, state, P) == Status::success, profile,
                    "predict state and covariance from the previous corrected posterior");
        test.expect(formal_eskf::try_correct_accelerometer(state, P, imu.specific_force_b, gravity, observation_noise,
                                                           minimum_norm, state, P) == Status::success,
                    profile, "gravity correction follows prediction");

        value_type const yaw = static_cast<value_type>(step) * dt * yaw_rate;
        value_type const magnetic_b_values[] = {std::cos(yaw), -std::sin(yaw), magnetic_n(2U)};
        auto const magnetic_b = vector3_type::from_row_major(magnetic_b_values);
        test.expect(formal_eskf::try_correct_magnetometer(state, P, magnetic_b, magnetic_n, observation_noise,
                                                          minimum_norm, state, P) == Status::success,
                    profile, "magnetic correction consumes the gravity-corrected posterior");

        if constexpr (std::is_same_v<Configuration, Ins>)
        {
            using vector2_type = formal_eskf::linalg::Vector<Backend, 2U>;
            using matrix2_type = formal_eskf::linalg::Matrix<Backend, 2U, 2U>;
            auto const horizontal_noise = matrix2_type::identity() * observation_noise(0U, 0U);
            test.expect(formal_eskf::try_correct_horizontal_position(state, P, vector2_type{}, horizontal_noise,
                                                                     minimum_norm, state, P) == Status::success,
                        profile, "horizontal position correction follows attitude corrections");
            test.expect(formal_eskf::try_correct_vertical_position(state, P, value_type{0}, observation_noise(2U, 2U),
                                                                   minimum_norm, state, P) == Status::success,
                        profile, "vertical position correction uses the updated posterior");
            test.expect(formal_eskf::try_correct_velocity(state, P, vector3_type{}, observation_noise, minimum_norm,
                                                          state, P) == Status::success,
                        profile, "velocity correction completes the tick");
        }
        test.expect(formal_eskf::linalg::all_finite(P) && formal_eskf::linalg::is_symmetric(P, value_type{0}), profile,
                    "every completed tick leaves finite symmetric covariance");
    }

    value_type const yaw = static_cast<value_type>(steps) * dt * yaw_rate;
    value_type const expected_rotation_values[] = {
        std::cos(yaw), -std::sin(yaw), 0, std::sin(yaw), std::cos(yaw), 0, 0, 0, 1};
    auto const expected_rotation = matrix3_type::from_row_major(expected_rotation_values);
    test.expect(formal_eskf::linalg::max_abs(formal_eskf::so3::to_rotation_matrix(state.q_nb) - expected_rotation) <
                    static_cast<value_type>(0.01),
                profile, "attitude converges to the independently known rotating trajectory");
    test.expect(std::abs(formal_eskf::linalg::squared_norm(state.q_nb.coefficients()) - value_type{1}) <=
                    parameters.quaternion_squared_norm_tolerance,
                profile, "successive injections keep a valid quaternion for the next prediction");
    if constexpr (std::is_same_v<Configuration, Ins>)
    {
        test.expect(formal_eskf::linalg::max_abs(state.p_n) < static_cast<value_type>(0.02) &&
                        formal_eskf::linalg::max_abs(state.v_n) < static_cast<value_type>(0.02),
                    profile, "navigation corrections recover the stationary position and velocity");
        test.expect(formal_eskf::linalg::max_abs(state.b_a) == value_type{0} &&
                        formal_eskf::linalg::max_abs(state.b_g) == value_type{0},
                    profile, "known zero biases remain unchanged");
    }

    auto const before = state;
    auto const P_before = P;
    test.expect(formal_eskf::try_predict(state, P, imu, value_type{0}, parameters, state, P) == Status::out_of_range,
                profile, "an invalid next tick fails after the successful sequence");
    test.expect(formal_eskf::linalg::max_abs(state.q_nb.coefficients() - before.q_nb.coefficients()) == value_type{0} &&
                    formal_eskf::linalg::max_abs(P - P_before) == value_type{0},
                profile, "failed tick preserves the last attitude and covariance");
    if constexpr (std::is_same_v<Configuration, Ins>)
    {
        test.expect(formal_eskf::linalg::max_abs(state.p_n - before.p_n) == value_type{0} &&
                        formal_eskf::linalg::max_abs(state.v_n - before.v_n) == value_type{0} &&
                        formal_eskf::linalg::max_abs(state.b_a - before.b_a) == value_type{0} &&
                        formal_eskf::linalg::max_abs(state.b_g - before.b_g) == value_type{0},
                    profile, "failed tick preserves the last navigation state and biases");
    }
}

} // namespace

int main()
{
    Context test;
    formal_eskf::test::configure_backend_test();
    test_sequence<formal_eskf::test::Backend<float>, Ahrs>(test, "AHRS sequence binary32");
    test_sequence<formal_eskf::test::Backend<double>, Ahrs>(test, "AHRS sequence binary64");
    test_sequence<formal_eskf::test::Backend<float>, Ins>(test, "INS sequence binary32");
    test_sequence<formal_eskf::test::Backend<double>, Ins>(test, "INS sequence binary64");
    if (test.failures() != 0)
    {
        return 1;
    }
    std::cout << "All ESKF sequence tests passed\n";
    return 0;
}
