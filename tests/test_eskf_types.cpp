/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cstddef>
#include <iostream>
#include <string_view>

#include <formal_eskf/eskf/eskf.hpp>
#include <formal_eskf/linalg/backend/eigen.hpp>

#include "test_support.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::test::near;

template <typename Vector>
[[nodiscard]] bool vector_near(Vector const & actual, Vector const & expected, typename Vector::value_type tolerance)
{
    for (std::size_t index = 0U; index < Vector::row_count; ++index)
    {
        if (!near(actual(index), expected(index), tolerance))
        {
            return false;
        }
    }
    return true;
}

template <typename Linalg>
void test_nominal_state(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using types = formal_eskf::EskfTypes<Linalg, formal_eskf::configuration::Ins>;
    using state_type = typename types::nominal_state_type;
    using value_type = typename Linalg::value_type;
    using vector3_type = typename Linalg::template vector_type<3U>;

    state_type state;
    vector3_type const zero = vector3_type::zero();
    test.expect(vector_near(state.p_n, zero, tolerance) && vector_near(state.v_n, zero, tolerance) &&
                    vector_near(state.b_a, zero, tolerance) && vector_near(state.b_g, zero, tolerance),
                profile, "default nominal Euclidean components are zero");
    test.expect(state.q_nb.q0() == value_type{1} && state.q_nb.q1() == value_type{0} &&
                    state.q_nb.q2() == value_type{0} && state.q_nb.q3() == value_type{0},
                profile, "default nominal attitude is identity");

    constexpr value_type position_values[3] = {1, 2, 3};
    constexpr value_type velocity_values[3] = {4, 5, 6};
    constexpr value_type accelerometer_bias_values[3] = {7, 8, 9};
    constexpr value_type gyroscope_bias_values[3] = {10, 11, 12};
    state.p_n = vector3_type::from_row_major(position_values);
    state.v_n = vector3_type::from_row_major(velocity_values);
    state.b_a = vector3_type::from_row_major(accelerometer_bias_values);
    state.b_g = vector3_type::from_row_major(gyroscope_bias_values);

    test.expect(state.p_n(0U) == value_type{1} && state.v_n(0U) == value_type{4} && state.b_a(0U) == value_type{7} &&
                    state.b_g(0U) == value_type{10},
                profile, "nominal components are independently addressable by their ESKF symbols");
}

template <typename Linalg>
void test_error_state_and_covariance(TestContext & test, std::string_view profile,
                                     typename Linalg::value_type tolerance)
{
    using types = formal_eskf::EskfTypes<Linalg, formal_eskf::configuration::Ins>;
    using covariance_type = typename types::error_covariance_type;
    using error_state_type = typename types::error_state_type;
    using vector3_type = typename Linalg::template vector_type<3U>;

    static_assert(types::nominal_state_dimension == 16U);
    static_assert(types::error_state_dimension == 15U);
    static_assert(covariance_type::row_count == 15U);
    static_assert(covariance_type::column_count == 15U);

    error_state_type const error;
    vector3_type const zero = vector3_type::zero();
    test.expect(vector_near(error.delta_p_n, zero, tolerance) && vector_near(error.delta_v_n, zero, tolerance) &&
                    vector_near(error.delta_theta_b, zero, tolerance) &&
                    vector_near(error.delta_b_a, zero, tolerance) && vector_near(error.delta_b_g, zero, tolerance),
                profile, "default error state is zero");

    covariance_type const covariance;
    test.expect(formal_eskf::linalg::max_abs(covariance) == typename Linalg::value_type{0}, profile,
                "default error covariance is a fixed 15-by-15 zero matrix");
}

template <typename Linalg>
void test_imu_and_configuration(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using types = formal_eskf::EskfTypes<Linalg, formal_eskf::configuration::Ins>;
    using configuration_type = typename types::parameter_type;
    using imu_sample_type = typename types::imu_sample_type;
    using process_noise_type = typename types::process_noise_type;
    using value_type = typename Linalg::value_type;
    using vector3_type = typename Linalg::template vector_type<3U>;

    constexpr value_type specific_force_values[3] = {1, 2, 3};
    constexpr value_type angular_rate_values[3] = {4, 5, 6};
    imu_sample_type const sample{vector3_type::from_row_major(specific_force_values),
                                 vector3_type::from_row_major(angular_rate_values)};
    test.expect(sample.specific_force_b(2U) == value_type{3} && sample.angular_rate_b(2U) == value_type{6}, profile,
                "IMU sample keeps body-frame specific force and angular rate separate");

    constexpr value_type noise_values[3] = {7, 8, 9};
    vector3_type const noise_vector = vector3_type::from_row_major(noise_values);
    process_noise_type const noise{noise_vector, noise_vector, noise_vector, noise_vector};

    constexpr value_type gravity_values[3] = {0, 0, 10};
    configuration_type const configuration{vector3_type::from_row_major(gravity_values), noise,
                                           static_cast<value_type>(1.0e-6), static_cast<value_type>(1.0e-4),
                                           static_cast<value_type>(1.0e-1)};

    static_assert(types::process_noise_dimension == 12U);
    test.expect(configuration.gravity_n(2U) == value_type{10} &&
                    vector_near(configuration.process_noise.specific_force_variance, noise_vector, tolerance) &&
                    configuration.minimum_quaternion_norm > value_type{0} &&
                    configuration.dt_min < configuration.dt_max,
                profile, "configuration preserves physical and numerical parameters");
}

template <typename Linalg>
void test_ahrs_types(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using types = formal_eskf::EskfTypes<Linalg, formal_eskf::configuration::Ahrs>;
    using state_type = typename types::nominal_state_type;
    using error_type = typename types::error_state_type;
    using covariance_type = typename types::error_covariance_type;
    using value_type = typename Linalg::value_type;
    using vector3_type = typename Linalg::template vector_type<3U>;

    static_assert(types::nominal_state_dimension == 4U);
    static_assert(types::error_state_dimension == 3U);
    static_assert(types::process_noise_dimension == 3U);
    static_assert(covariance_type::row_count == 3U && covariance_type::column_count == 3U);
    static_assert(!requires(state_type STATE) { STATE.p_n; });
    static_assert(!requires(state_type STATE) { STATE.v_n; });
    static_assert(!requires(state_type STATE) { STATE.b_a; });
    static_assert(!requires(state_type STATE) { STATE.b_g; });

    state_type const state;
    auto const coefficients = state.q_nb.coefficients();
    static_assert(decltype(coefficients)::row_count == 4U && decltype(coefficients)::column_count == 1U);
    test.expect(coefficients(0U) == value_type{1} && coefficients(1U) == value_type{0} &&
                    coefficients(2U) == value_type{0} && coefficients(3U) == value_type{0},
                profile, "AHRS nominal state exposes a scalar-first 4x1 identity quaternion");

    error_type const error;
    covariance_type const covariance;
    test.expect(vector_near(error.delta_theta_b, vector3_type::zero(), tolerance) &&
                    formal_eskf::linalg::max_abs(covariance) == value_type{0},
                profile, "AHRS has zero three-dimensional attitude error and 3x3 covariance");

    constexpr value_type noise_values[3] = {1, 2, 3};
    vector3_type const noise_vector = vector3_type::from_row_major(noise_values);
    typename types::parameter_type const parameters{{noise_vector},
                                                    static_cast<value_type>(1.0e-6),
                                                    static_cast<value_type>(1.0e-4),
                                                    static_cast<value_type>(1.0e-1)};
    test.expect(vector_near(parameters.process_noise.angular_rate_variance, noise_vector, tolerance) &&
                    parameters.minimum_quaternion_norm > value_type{0} && parameters.dt_min < parameters.dt_max,
                profile, "AHRS parameters contain angular-rate process noise and time bounds");
}

template <typename Linalg>
void run_type_tests(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    test_nominal_state<Linalg>(test, profile, tolerance);
    test_error_state_and_covariance<Linalg>(test, profile, tolerance);
    test_imu_and_configuration<Linalg>(test, profile, tolerance);
    test_ahrs_types<Linalg>(test, profile, tolerance);
}

} /* end namespace */

int main()
{
    TestContext test;

    Eigen::internal::set_is_malloc_allowed(false);
    run_type_tests<formal_eskf::linalg::EigenBackend<double>>(test, "binary64", 1.0e-12);
    run_type_tests<formal_eskf::linalg::EigenBackend<float>>(test, "binary32", 1.0e-5F);

    if (test.failures() != 0)
    {
        std::cerr << test.failures() << " ESKF type test(s) failed\n";
        return 1;
    }

    std::cout << "All ESKF domain type tests passed\n";
    return 0;
}
