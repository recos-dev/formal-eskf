/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/eskf/eskf.hpp>

#include <cmath>
#include <array>
#include <bit>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string_view>
#include <type_traits>

#include "test_backend.hpp"
#include "test_support.hpp"
#include "test_cases.hpp"
#include "correction_reference.hpp"

namespace
{

using formal_eskf::Status;
using formal_eskf::configuration::Ahrs;
using formal_eskf::configuration::Ins;
using formal_eskf::test::Context;

// Independent trajectory: fixed position, level attitude, constant positive yaw
// rate. Gravity and magnetic observations follow analytic sine/cosine, without
// using the estimator's rotation functions to manufacture their expected values.
template <typename Backend, typename Configuration>
void test_rotating_regression(Context & test, std::string_view profile)
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
    test.expect(formal_eskf::so3::exp(vector3_type::from_row_major(initial_error), minimum_norm, state.q_nb) ==
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
        test.expect(formal_eskf::predict(state, P, imu, dt, parameters, state, P) == Status::success, profile,
                    "predict state and covariance from the previous corrected posterior");
        test.expect(formal_eskf::correct_accelerometer(state, P, imu.specific_force_b, gravity, observation_noise,
                                                       minimum_norm, state, P) == Status::success,
                    profile, "gravity correction follows prediction");

        value_type const yaw = static_cast<value_type>(step) * dt * yaw_rate;
        value_type const magnetic_b_values[] = {std::cos(yaw), -std::sin(yaw), magnetic_n(2U)};
        auto const magnetic_b = vector3_type::from_row_major(magnetic_b_values);
        test.expect(formal_eskf::correct_magnetometer(state, P, magnetic_b, magnetic_n, observation_noise, minimum_norm,
                                                      state, P) == Status::success,
                    profile, "magnetic correction consumes the gravity-corrected posterior");

        if constexpr (std::is_same_v<Configuration, Ins>)
        {
            using vector2_type = formal_eskf::linalg::Vector<Backend, 2U>;
            using matrix2_type = formal_eskf::linalg::Matrix<Backend, 2U, 2U>;
            auto const horizontal_noise = matrix2_type::identity() * observation_noise(0U, 0U);
            test.expect(formal_eskf::correct_horizontal_position(state, P, vector2_type{}, horizontal_noise,
                                                                 minimum_norm, state, P) == Status::success,
                        profile, "horizontal position correction follows attitude corrections");
            test.expect(formal_eskf::correct_vertical_position(state, P, value_type{0}, observation_noise(2U, 2U),
                                                               minimum_norm, state, P) == Status::success,
                        profile, "vertical position correction uses the updated posterior");
            test.expect(formal_eskf::correct_velocity(state, P, vector3_type{}, observation_noise, minimum_norm, state,
                                                      P) == Status::success,
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
    test.expect(formal_eskf::predict(state, P, imu, value_type{0}, parameters, state, P) == Status::out_of_range,
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

template <std::size_t Rows, std::size_t Columns> struct WideMatrix
{
    static constexpr std::size_t row_count = Rows;
    static constexpr std::size_t column_count = Columns;
    formal_eskf::test::reference::Dense<Rows, Columns> values{};
    long double operator()(std::size_t row, std::size_t column) const { return values[row][column]; }
};

template <typename Backend, typename Configuration, std::size_t MeasurementSize> struct TimelineReference
{
    using value_type = typename Backend::value_type;
    using types = formal_eskf::EskfTypes<Backend, Configuration>;
    using state_type = typename types::nominal_state_type;
    using covariance_type = typename types::error_covariance_type;
    static constexpr bool is_ins = std::is_same_v<Configuration, Ins>;
    static constexpr std::size_t size = types::error_state_dimension;
    static constexpr std::size_t attitude_offset = is_ins ? 6U : 0U;
    static constexpr std::size_t measurement_size = MeasurementSize;
    state_type state;
    WideMatrix<size, size> P;
    formal_eskf::linalg::Matrix<Backend, MeasurementSize, size> H{};
    formal_eskf::linalg::Vector<Backend, MeasurementSize> r{};
    formal_eskf::linalg::Matrix<Backend, MeasurementSize, MeasurementSize> V{};
};

template <typename Backend, typename Configuration>
auto advance_reference_covariance(
    WideMatrix<formal_eskf::EskfTypes<Backend, Configuration>::error_state_dimension,
               formal_eskf::EskfTypes<Backend, Configuration>::error_state_dimension> const & prior,
    long double dt)
{
    constexpr bool ins = std::is_same_v<Configuration, Ins>;
    constexpr std::size_t size = formal_eskf::EskfTypes<Backend, Configuration>::error_state_dimension;
    formal_eskf::test::reference::Dense<size, size> F{};
    for (std::size_t axis = 0U; axis < size; ++axis)
    {
        F[axis][axis] = 1;
    }
    if constexpr (ins)
    {
        constexpr long double R[3][3]{{0, 0, 1}, {1, 0, 0}, {0, 1, 0}};
        constexpr long double force_hat[3][3]{{0, 0, -8}, {0, 0, 0}, {8, 0, 0}};
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            F[row][3U + row] = dt;
            F[6U + row][12U + row] = -dt;
            for (std::size_t column = 0U; column < 3U; ++column)
            {
                F[3U + row][9U + column] = -R[row][column] * dt;
                for (std::size_t k = 0U; k < 3U; ++k)
                {
                    F[3U + row][6U + column] -= R[row][k] * force_hat[k][column] * dt;
                }
            }
        }
    }
    auto result = formal_eskf::test::reference::product(formal_eskf::test::reference::product(F, prior.values),
                                                        formal_eskf::test::reference::transposed(F));
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        if constexpr (ins)
        {
            constexpr long double rotated_force_variance[3]{0.75L, 0.25L, 0.5L};
            result[3U + axis][3U + axis] += rotated_force_variance[axis] * dt * dt;
            result[6U + axis][6U + axis] += static_cast<long double>(axis + 1U) / 16 * dt * dt;
            result[9U + axis][9U + axis] += dt / 64;
            result[12U + axis][12U + axis] += dt / 256;
        }
        else
        {
            result[axis][axis] += static_cast<long double>(axis + 1U) / 16 * dt * dt;
        }
    }
    return WideMatrix<size, size>{result};
}

template <typename Matrix> bool preserved(Matrix const & a, Matrix const & b)
{
    using bytes_type = std::array<std::byte, sizeof(typename Matrix::value_type)>;
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            if (!(std::isnan(a(row, column)) && std::isnan(b(row, column))) &&
                std::bit_cast<bytes_type>(a(row, column)) != std::bit_cast<bytes_type>(b(row, column)))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename State> bool preserved_state(State const & a, State const & b)
{
    bool result = preserved(a.q_nb.coefficients(), b.q_nb.coefficients());
    if constexpr (requires { a.p_n; })
    {
        result = result && preserved(a.p_n, b.p_n) && preserved(a.v_n, b.v_n) && preserved(a.b_a, b.b_a) &&
                 preserved(a.b_g, b.b_g);
    }
    return result;
}

template <typename Reference>
void publish_reference(Context & test, std::string_view profile, Reference const & input, Status status,
                       typename Reference::state_type const & state, typename Reference::covariance_type const & P,
                       WideMatrix<Reference::size, Reference::size> & covariance, bool heading_projection = false)
{
    auto const result =
        formal_eskf::test::reference::reference_correction(input, ESKF_RESET_APPROX != 0, {}, heading_projection);
    using value_type = typename Reference::value_type;
    // A finite-fixture regression allowance for this eight-tick timeline, not
    // a certified cumulative conditioning/roundoff budget.
    constexpr value_type tolerance = static_cast<value_type>(std::is_same_v<value_type, float> ? 2e-4 : 2e-11);
    test.expect(status == Status::success &&
                    formal_eskf::test::reference::matches_reference(input, result, state, P, tolerance),
                profile,
                "every composed observation matches the independently advanced full-state/covariance timeline");
    covariance.values = result.covariance;
}

template <typename Backend, typename Configuration>
void test_reference_timeline(Context & test, std::string_view profile)
{
    using types = formal_eskf::EskfTypes<Backend, Configuration>;
    using value_type = typename types::value_type;
    constexpr bool ins = std::is_same_v<Configuration, Ins>;
    constexpr std::size_t size = types::error_state_dimension;
    constexpr std::size_t offset = ins ? 6U : 0U;
    using full_reference = TimelineReference<Backend, Configuration, 3U>;
    typename types::nominal_state_type state;
    test.expect(formal_eskf::so3::UnitQuaternion<Backend>::from_coefficients(
                    value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.000001},
                    state.q_nb) == Status::success,
                profile, "exact cyclic trajectory attitude is valid");
    typename types::parameter_type parameters;
    parameters.dt_min = value_type{0.125};
    parameters.dt_max = value_type{0.25};
    parameters.minimum_quaternion_norm = value_type{0.000001};
    parameters.quaternion_squared_norm_tolerance = value_type{0.125};
    typename types::imu_sample_type imu;
    imu.specific_force_b(1U) = value_type{-8};
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        parameters.process_noise.angular_rate_variance(axis) = static_cast<value_type>(axis + 1U) / value_type{16};
        if constexpr (ins)
        {
            state.p_n(axis) = static_cast<value_type>(axis + 1U);
            state.b_a(axis) = static_cast<value_type>(axis + 1U) / value_type{8};
            state.b_g(axis) = -static_cast<value_type>(axis + 1U) / value_type{16};
            imu.specific_force_b(axis) += state.b_a(axis);
            imu.angular_rate_b(axis) = state.b_g(axis);
            parameters.process_noise.specific_force_variance(axis) = static_cast<value_type>(axis + 1U) / value_type{4};
            parameters.process_noise.accelerometer_bias_random_walk_variance_density(axis) =
                value_type{1} / value_type{64};
            parameters.process_noise.gyroscope_bias_random_walk_variance_density(axis) =
                value_type{1} / value_type{256};
        }
    }
    if constexpr (ins)
    {
        parameters.gravity_n(2U) = value_type{8};
    }
    auto const nominal_reference = state; // Exact stationary solution of independently stated inputs.
    typename types::error_covariance_type P;
    WideMatrix<size, size> reference_P;
    for (std::size_t row = 0U; row < size; ++row)
    {
        for (std::size_t column = 0U; column < size; ++column)
        {
            long double const u = static_cast<long double>(row + 1U) / 32;
            long double const v = static_cast<long double>(column + 1U) / 32;
            reference_P.values[row][column] = u * v + (row == column ? 1 : 0);
            P(row, column) = static_cast<value_type>(reference_P.values[row][column]);
        }
    }
    using vector3_type = formal_eskf::linalg::Vector<Backend, 3U>;
    using matrix3_type = formal_eskf::linalg::Matrix<Backend, 3U, 3U>;
    auto const gravity = vector3_type::from_row_major({value_type{0}, value_type{0}, value_type{8}});
    auto const magnetic = vector3_type::from_row_major({value_type{2}, value_type{1}, value_type{-3}});
    auto const measured_magnetic = vector3_type::from_row_major({value_type{1}, value_type{-3}, value_type{2}});
    constexpr value_type intervals[]{value_type{0.125}, value_type{0.25}, value_type{0.1875}, value_type{0.125}};
    for (std::size_t tick = 0U; tick < 8U; ++tick)
    {
        value_type const dt = intervals[tick % 4U];
        reference_P = advance_reference_covariance<Backend, Configuration>(reference_P, dt);
        test.expect(formal_eskf::predict(state, P, imu, dt, parameters, state, P) == Status::success &&
                        preserved_state(state, nominal_reference),
                    profile, "variable-time prediction preserves exact stationary nominal solution");
        constexpr value_type tolerance = static_cast<value_type>(std::is_same_v<value_type, float> ? 2e-4 : 2e-11);
        for (std::size_t row = 0U; row < size; ++row)
        {
            for (std::size_t column = 0U; column < size; ++column)
            {
                test.expect(std::abs(static_cast<long double>(P(row, column)) - reference_P.values[row][column]) <=
                                tolerance,
                            profile, "prediction advances full independent prior once with rotated discrete noise");
            }
        }
        full_reference gravity_reference{nominal_reference, reference_P};
        gravity_reference.H(0U, offset + 2U) = value_type{-8};
        gravity_reference.H(2U, offset) = value_type{8};
        gravity_reference.V = matrix3_type::identity() * value_type{4};
        gravity_reference.V(0U, 1U) = gravity_reference.V(1U, 0U) = value_type{0.25};
        if (tick == 2U)
        {
            auto invalid_V = gravity_reference.V;
            invalid_V(0U, 0U) = value_type{0};
            auto const state_before = state;
            auto const P_before = P;
            test.expect(formal_eskf::correct_accelerometer(state, P, imu.specific_force_b, gravity, invalid_V,
                                                           parameters.minimum_quaternion_norm, state,
                                                           P) == Status::domain_error &&
                            preserved_state(state, state_before) && preserved(P, P_before),
                        profile,
                        "failed correction preserves the completed prediction rather than rolling back the tick");
        }
        auto const gravity_status = formal_eskf::correct_accelerometer(
            state, P, imu.specific_force_b, gravity, gravity_reference.V, parameters.minimum_quaternion_norm, state, P);
        publish_reference(test, profile, gravity_reference, gravity_status, state, P, reference_P);
        full_reference magnetic_reference{nominal_reference, reference_P};
        constexpr value_type magnetic_hat[3][3]{{0, -2, -3}, {2, 0, -1}, {3, 1, 0}};
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            for (std::size_t column = 0U; column < 3U; ++column)
            {
                magnetic_reference.H(row, offset + column) = magnetic_hat[row][column];
            }
        }
        magnetic_reference.V = matrix3_type::identity() * value_type{2};
        magnetic_reference.V(0U, 2U) = magnetic_reference.V(2U, 0U) = value_type{0.125};
        auto const magnetic_status = formal_eskf::correct_magnetometer(
            state, P, measured_magnetic, magnetic, magnetic_reference.V, parameters.minimum_quaternion_norm, state, P);
        publish_reference(test, profile, magnetic_reference, magnetic_status, state, P, reference_P,
                          ESKF_MAG_TILT != 0);
        if constexpr (ins)
        {
            TimelineReference<Backend, Configuration, 2U> horizontal{nominal_reference, reference_P};
            horizontal.H(0U, 0U) = horizontal.H(1U, 1U) = value_type{1};
            horizontal.V(0U, 0U) = value_type{1};
            horizontal.V(1U, 1U) = value_type{2};
            horizontal.V(0U, 1U) = horizontal.V(1U, 0U) = value_type{0.25};
            auto const horizontal_measurement =
                formal_eskf::linalg::Vector<Backend, 2U>::from_row_major({value_type{1}, value_type{2}});
            auto const horizontal_status = formal_eskf::correct_horizontal_position(
                state, P, horizontal_measurement, horizontal.V, parameters.minimum_quaternion_norm, state, P);
            publish_reference(test, profile, horizontal, horizontal_status, state, P, reference_P);
            TimelineReference<Backend, Configuration, 1U> vertical{nominal_reference, reference_P};
            vertical.H(0U, 2U) = value_type{1};
            vertical.V(0U, 0U) = value_type{4};
            auto const vertical_status = formal_eskf::correct_vertical_position(
                state, P, value_type{3}, value_type{4}, parameters.minimum_quaternion_norm, state, P);
            publish_reference(test, profile, vertical, vertical_status, state, P, reference_P);
            if (tick % 2U == 0U)
            {
                full_reference velocity{nominal_reference, reference_P};
                for (std::size_t axis = 0U; axis < 3U; ++axis)
                {
                    velocity.H(axis, 3U + axis) = value_type{1};
                    velocity.V(axis, axis) = value_type{2};
                }
                velocity.V(1U, 2U) = velocity.V(2U, 1U) = value_type{-0.125};
                auto const velocity_status = formal_eskf::correct_velocity(
                    state, P, vector3_type{}, velocity.V, parameters.minimum_quaternion_norm, state, P);
                publish_reference(test, profile, velocity, velocity_status, state, P, reference_P);
            }
            else
            {
                TimelineReference<Backend, Configuration, 2U> velocity{nominal_reference, reference_P};
                velocity.H(0U, 3U) = velocity.H(1U, 4U) = value_type{1};
                velocity.V(0U, 0U) = velocity.V(1U, 1U) = value_type{2};
                auto const velocity_status =
                    formal_eskf::correct_horizontal_velocity(state, P, formal_eskf::linalg::Vector<Backend, 2U>{},
                                                             velocity.V, parameters.minimum_quaternion_norm, state, P);
                publish_reference(test, profile, velocity, velocity_status, state, P, reference_P);
            }
        }
    }
}

template <typename Scalar> bool preserved_scalar(Scalar a, Scalar b)
{
    using bytes_type = std::array<std::byte, sizeof(Scalar)>;
    return (std::isnan(a) && std::isnan(b)) || std::bit_cast<bytes_type>(a) == std::bit_cast<bytes_type>(b);
}

template <typename Parameters> bool preserved_parameters(Parameters const & a, Parameters const & b)
{
    bool result = preserved_scalar(a.dt_min, b.dt_min) && preserved_scalar(a.dt_max, b.dt_max) &&
                  preserved_scalar(a.minimum_quaternion_norm, b.minimum_quaternion_norm) &&
                  preserved_scalar(a.quaternion_squared_norm_tolerance, b.quaternion_squared_norm_tolerance) &&
                  preserved(a.process_noise.angular_rate_variance, b.process_noise.angular_rate_variance);
    if constexpr (requires { a.gravity_n; })
    {
        result = result && preserved(a.gravity_n, b.gravity_n) &&
                 preserved(a.process_noise.specific_force_variance, b.process_noise.specific_force_variance) &&
                 preserved(a.process_noise.accelerometer_bias_random_walk_variance_density,
                           b.process_noise.accelerometer_bias_random_walk_variance_density) &&
                 preserved(a.process_noise.gyroscope_bias_random_walk_variance_density,
                           b.process_noise.gyroscope_bias_random_walk_variance_density);
    }
    return result;
}

template <typename Error> bool preserved_error(Error const & a, Error const & b)
{
    bool result = preserved(a.delta_theta_b, b.delta_theta_b);
    if constexpr (requires { a.delta_p_n; })
    {
        result = result && preserved(a.delta_p_n, b.delta_p_n) && preserved(a.delta_v_n, b.delta_v_n) &&
                 preserved(a.delta_b_a, b.delta_b_a) && preserved(a.delta_b_g, b.delta_b_g);
    }
    return result;
}

template <typename Backend, typename Configuration>
void test_independent_frames(Context & test, std::string_view profile)
{
    using types = formal_eskf::EskfTypes<Backend, Configuration>;
    using value_type = typename types::value_type;
    using state_type = typename types::nominal_state_type;
    using covariance_type = typename types::error_covariance_type;
    using error_type = typename types::error_state_type;
    constexpr bool ins = std::is_same_v<Configuration, Ins>;
    constexpr std::size_t offset = ins ? 6U : 0U;
    constexpr value_type minimum_norm = static_cast<value_type>(1e-6);
    constexpr value_type nan = std::numeric_limits<value_type>::quiet_NaN();
    state_type initial;
    state_type sentinel;
    sentinel.q_nb = -sentinel.q_nb;
    if constexpr (ins)
    {
        initial.p_n(0U) = value_type{1};
        initial.v_n(0U) = value_type{0.5};
        sentinel.p_n(0U) = nan;
        sentinel.v_n(1U) = -value_type{0};
        sentinel.b_a(2U) = value_type{7};
        sentinel.b_g(0U) = value_type{-3};
    }
    covariance_type covariance_sentinel;
    covariance_sentinel(0U, 0U) = nan;
    covariance_sentinel(0U, 1U) = -value_type{0};
    error_type error_sentinel;
    error_sentinel.delta_theta_b(0U) = nan;
    error_sentinel.delta_theta_b(1U) = -value_type{0};
    if constexpr (ins)
    {
        error_sentinel.delta_p_n(0U) = value_type{-5};
        error_sentinel.delta_v_n(1U) = nan;
        error_sentinel.delta_b_a(2U) = -value_type{0};
        error_sentinel.delta_b_g(0U) = value_type{7};
    }
    typename types::parameter_type parameters;
    parameters.dt_min = parameters.dt_max = value_type{0.25};
    parameters.minimum_quaternion_norm = minimum_norm;
    parameters.quaternion_squared_norm_tolerance = value_type{0.125};
    parameters.process_noise.angular_rate_variance(0U) = value_type{0.25};
    typename types::imu_sample_type imu;
    if constexpr (!ins)
    {
        imu.specific_force_b(0U) = nan;
        imu.specific_force_b(1U) = -value_type{0};
    }
    auto const imu_before = imu;
    for (bool failure : {false, true})
    {
        auto prediction_parameters = parameters;
        if (failure)
        {
            // The nominal step succeeds before the covariance/noise stage fails.
            prediction_parameters.process_noise.angular_rate_variance(0U) = value_type{-1};
        }
        auto const parameters_before = prediction_parameters;
        for (unsigned aliases = 0U; aliases < 4U; ++aliases)
        {
            auto state = initial;
            auto P = covariance_type::identity();
            auto separate_state = sentinel;
            auto separate_P = covariance_sentinel;
            auto const state_before = state;
            auto const P_before = P;
            auto & state_output = (aliases & 1U) != 0U ? state : separate_state;
            auto & P_output = (aliases & 2U) != 0U ? P : separate_P;
            auto const output_before = state_output;
            auto const covariance_before = P_output;
            auto const status =
                formal_eskf::predict(state, P, imu, value_type{0.25}, prediction_parameters, state_output, P_output);
            test.expect(status == (failure ? Status::domain_error : Status::success), profile,
                        "prediction frame fixture reaches both final success and post-nominal failure");
            test.expect(((aliases & 1U) != 0U || preserved_state(state, state_before)) &&
                            ((aliases & 2U) != 0U || preserved(P, P_before)) &&
                            ((aliases & 1U) == 0U || preserved_state(separate_state, sentinel)) &&
                            ((aliases & 2U) == 0U || preserved(separate_P, covariance_sentinel)) &&
                            preserved_parameters(prediction_parameters, parameters_before) &&
                            preserved(imu.specific_force_b, imu_before.specific_force_b) &&
                            preserved(imu.angular_rate_b, imu_before.angular_rate_b),
                        profile, "prediction preserves every disjoint input and unused output object");
            if (failure)
            {
                test.expect(preserved_state(state_output, output_before) && preserved(P_output, covariance_before),
                            profile, "failed prediction preserves all named outputs including sentinel fields");
            }
        }
        for (unsigned aliases = 0U; aliases < 8U; ++aliases)
        {
            auto state = initial;
            auto P = covariance_type::identity();
            error_type error;
            if constexpr (ins)
            {
                error.delta_p_n(0U) = value_type{0.25};
            }
            if (failure)
            {
                // Nominal injection succeeds; the next covariance reset fails.
                P(0U, 0U) = nan;
            }
            auto separate_state = sentinel;
            auto separate_P = covariance_sentinel;
            auto separate_error = error_sentinel;
            auto const state_before = state;
            auto const P_before = P;
            auto const error_before = error;
            auto & state_output = (aliases & 1U) != 0U ? state : separate_state;
            auto & P_output = (aliases & 2U) != 0U ? P : separate_P;
            auto & error_output = (aliases & 4U) != 0U ? error : separate_error;
            auto const output_before = state_output;
            auto const covariance_before = P_output;
            auto const error_output_before = error_output;
            auto const status =
                formal_eskf::inject_and_reset(state, P, error, minimum_norm, state_output, P_output, error_output);
            test.expect(status == (failure ? Status::non_finite_input : Status::success), profile,
                        "injection/reset frame fixture reaches success and post-injection failure");
            test.expect(((aliases & 1U) != 0U || preserved_state(state, state_before)) &&
                            ((aliases & 2U) != 0U || preserved(P, P_before)) &&
                            ((aliases & 4U) != 0U || preserved_error(error, error_before)) &&
                            ((aliases & 1U) == 0U || preserved_state(separate_state, sentinel)) &&
                            ((aliases & 2U) == 0U || preserved(separate_P, covariance_sentinel)) &&
                            ((aliases & 4U) == 0U || preserved_error(separate_error, error_sentinel)),
                        profile, "injection/reset preserves all disjoint inputs and unused output objects");
            if (failure)
            {
                test.expect(preserved_state(state_output, output_before) && preserved(P_output, covariance_before) &&
                                preserved_error(error_output, error_output_before),
                            profile, "failed injection/reset preserves its three named outputs");
            }
        }
        for (unsigned aliases = 0U; aliases < 4U; ++aliases)
        {
            auto state = initial;
            auto P = covariance_type::identity();
            formal_eskf::linalg::Vector<Backend, 1U> r;
            r(0U) = value_type{0.25};
            formal_eskf::linalg::Matrix<Backend, 1U, types::error_state_dimension> H;
            H(0U, offset) = value_type{1};
            formal_eskf::linalg::Matrix<Backend, 1U, 1U> V;
            V(0U, 0U) = failure ? value_type{0} : value_type{3};
            auto const r_before = r;
            auto const H_before = H;
            auto const V_before = V;
            auto separate_state = sentinel;
            auto separate_P = covariance_sentinel;
            auto const state_before = state;
            auto const P_before = P;
            auto & state_output = (aliases & 1U) != 0U ? state : separate_state;
            auto & P_output = (aliases & 2U) != 0U ? P : separate_P;
            auto const output_before = state_output;
            auto const covariance_before = P_output;
            auto const status = formal_eskf::correct(state, P, r, H, V, minimum_norm, state_output, P_output);
            test.expect(status == (failure ? Status::domain_error : Status::success), profile,
                        "generic correction frame fixture reaches success and failure");
            test.expect(((aliases & 1U) != 0U || preserved_state(state, state_before)) &&
                            ((aliases & 2U) != 0U || preserved(P, P_before)) &&
                            ((aliases & 1U) == 0U || preserved_state(separate_state, sentinel)) &&
                            ((aliases & 2U) == 0U || preserved(separate_P, covariance_sentinel)) &&
                            preserved(r, r_before) && preserved(H, H_before) && preserved(V, V_before),
                        profile, "correction preserves borrowed residual, Jacobian, noise and disjoint prior");
            if (failure)
            {
                test.expect(preserved_state(state_output, output_before) && preserved(P_output, covariance_before),
                            profile, "failed correction preserves all named outputs");
            }
        }
    }
    // These are real-body frame checks. Arbitrarily overwritten failed-callee
    // scratch and skipped-call order require the matching compositional ESBMC
    // harnesses/source review; a native test cannot inject that internal summary.
}

template <typename Backend, typename Configuration> void test_composition(Context & test, std::string_view profile)
{
    test_reference_timeline<Backend, Configuration>(test, profile);
    // Preserve the previous moving/yaw convergence regression as supplemental
    // behavior evidence. Its empirical final trajectory limits are not a
    // certified full-state accuracy or numerical acceptance contract.
    test_rotating_regression<Backend, Configuration>(test, profile);
}

} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using float_backend = formal_eskf::test::Backend<float>;
    using double_backend = formal_eskf::test::Backend<double>;
    formal_eskf::test::Case const cases[] = {
        {"TC-E-STEP-independent-frames", "AHRS.binary32", test_independent_frames<float_backend, Ahrs>},
        {"TC-E-STEP-independent-frames", "AHRS.binary64", test_independent_frames<double_backend, Ahrs>},
        {"TC-E-STEP-independent-frames", "INS.binary32", test_independent_frames<float_backend, Ins>},
        {"TC-E-STEP-independent-frames", "INS.binary64", test_independent_frames<double_backend, Ins>},
        {"TC-SW-CORE-composition", "AHRS.binary32", test_composition<float_backend, Ahrs>},
        {"TC-SW-CORE-composition", "AHRS.binary64", test_composition<double_backend, Ahrs>},
        {"TC-SW-CORE-composition", "INS.binary32", test_composition<float_backend, Ins>},
        {"TC-SW-CORE-composition", "INS.binary64", test_composition<double_backend, Ins>},
    };
    std::cout << "Configuration: quat_approx=" << ESKF_QUAT_APPROX << " reset_approx=" << ESKF_RESET_APPROX
              << " mag_tilt=" << ESKF_MAG_TILT << " codegen=" << ESKF_CODEGEN << '\n';
    return formal_eskf::test::run_cases(cases, argc, argv);
}
