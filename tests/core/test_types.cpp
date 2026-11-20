/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cmath>
#include <cstddef>
#include <limits>
#include <iostream>
#include <string_view>

#include <formal_eskf/eskf/configuration/ahrs.hpp>
#include <formal_eskf/eskf/configuration/ins.hpp>
#include <formal_eskf/eskf/types.hpp>
#include <formal_eskf/eskf/injection.hpp>
#include <formal_eskf/eskf/prediction.hpp>
#include <formal_eskf/linalg/operations.hpp>
#include "test_backend.hpp"

#include "test_support.hpp"
#include "test_cases.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::test::near;

template <typename Vector>
bool vector_near(Vector const & actual, Vector const & expected, typename Vector::value_type tolerance)
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
    using vector3_type = formal_eskf::linalg::Vector<Linalg, 3U>;

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
    using noise_covariance_type = typename types::process_noise_covariance_type;
    using error_state_type = typename types::error_state_type;
    using vector3_type = formal_eskf::linalg::Vector<Linalg, 3U>;

    static_assert(types::nominal_state_dimension == 16U);
    static_assert(types::error_state_dimension == 15U);
    static_assert(covariance_type::row_count == 15U);
    static_assert(covariance_type::column_count == 15U);
    static_assert(noise_covariance_type::row_count == 12U && noise_covariance_type::column_count == 12U);

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
    using vector3_type = formal_eskf::linalg::Vector<Linalg, 3U>;

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
    configuration_type const configuration{vector3_type::from_row_major(gravity_values),
                                           noise,
                                           static_cast<value_type>(1.0e-6),
                                           static_cast<value_type>(1.0e-4),
                                           static_cast<value_type>(1.0e-1),
                                           static_cast<value_type>(1.0e-5)};

    static_assert(types::process_noise_dimension == 12U);
    test.expect(configuration.gravity_n(2U) == value_type{10} &&
                    vector_near(configuration.process_noise.specific_force_variance, noise_vector, tolerance) &&
                    configuration.minimum_quaternion_norm > value_type{0} &&
                    configuration.quaternion_squared_norm_tolerance == static_cast<value_type>(1.0e-5) &&
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
    using noise_covariance_type = typename types::process_noise_covariance_type;
    using value_type = typename Linalg::value_type;
    using vector3_type = formal_eskf::linalg::Vector<Linalg, 3U>;

    static_assert(types::nominal_state_dimension == 4U);
    static_assert(types::error_state_dimension == 3U);
    static_assert(types::process_noise_dimension == 3U);
    static_assert(covariance_type::row_count == 3U && covariance_type::column_count == 3U);
    static_assert(noise_covariance_type::row_count == 3U && noise_covariance_type::column_count == 3U);
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
                                                    static_cast<value_type>(1.0e-1),
                                                    static_cast<value_type>(1.0e-5)};
    test.expect(vector_near(parameters.process_noise.angular_rate_variance, noise_vector, tolerance) &&
                    parameters.minimum_quaternion_norm > value_type{0} && parameters.dt_min < parameters.dt_max &&
                    parameters.quaternion_squared_norm_tolerance == static_cast<value_type>(1.0e-5),
                profile, "AHRS parameters contain angular-rate process noise and time bounds");
}

template <typename Linalg> void test_shape(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    test_nominal_state<Linalg>(test, profile, value_type{0});
    test_error_state_and_covariance<Linalg>(test, profile, value_type{0});
    test_imu_and_configuration<Linalg>(test, profile, value_type{0});
    test_ahrs_types<Linalg>(test, profile, value_type{0});

    using ins_types = formal_eskf::EskfTypes<Linalg, formal_eskf::configuration::Ins>;
    using ahrs_types = formal_eskf::EskfTypes<Linalg, formal_eskf::configuration::Ahrs>;
    static_assert(!requires(typename ins_types::nominal_state_type state, typename ahrs_types::error_state_type error) {
        formal_eskf::inject_nominal(state, error, value_type{0.5}, state);
    });
    static_assert(!requires(typename ahrs_types::nominal_state_type state, typename ins_types::error_state_type error) {
        formal_eskf::inject_nominal(state, error, value_type{0.5}, state);
    });
    static_assert(
        !requires(typename ins_types::error_state_type error, typename ahrs_types::error_covariance_type covariance) {
            formal_eskf::reset_covariance(error, covariance, covariance);
        });
    static_assert(
        !requires(typename ahrs_types::error_state_type error, typename ins_types::error_covariance_type covariance) {
            formal_eskf::reset_covariance(error, covariance, covariance);
        });
    typename ins_types::nominal_state_type prior, output;
    typename ins_types::error_state_type error;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        value_type const i = static_cast<value_type>(axis);
        prior.p_n(axis) = value_type{1} + i;
        prior.v_n(axis) = value_type{4} + i;
        prior.b_a(axis) = value_type{7} + i;
        prior.b_g(axis) = value_type{10} + i;
        error.delta_p_n(axis) = value_type{16} + i;
        error.delta_v_n(axis) = value_type{32} + i;
        error.delta_b_a(axis) = value_type{48} + i;
        error.delta_b_g(axis) = value_type{64} + i;
    }
    auto const status = formal_eskf::inject_nominal(prior, error, value_type{0.5}, output);
    test.expect(status == formal_eskf::Status::success, profile,
                "named error blocks can be injected through public types");
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        value_type const twice_axis = value_type{2} * static_cast<value_type>(axis);
        test.expect(
            output.p_n(axis) == value_type{17} + twice_axis && output.v_n(axis) == value_type{36} + twice_axis &&
                output.b_a(axis) == value_type{55} + twice_axis && output.b_g(axis) == value_type{74} + twice_axis,
            profile, "distinct logical error blocks reach exactly their corresponding nominal fields");
    }
    test.expect(formal_eskf::so3::same_coefficients(output.q_nb, prior.q_nb), profile,
                "zero attitude error preserves identity independently of additive state blocks");
}

template <typename Linalg> void test_frame(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using vector_type = formal_eskf::linalg::Vector<Linalg, 3U>;
    using quaternion_type = formal_eskf::so3::UnitQuaternion<Linalg>;
    quaternion_type permutation;
    test.expect(quaternion_type::from_coefficients(value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5},
                                                   value_type{0.5}, permutation) == formal_eskf::Status::success,
                profile, "exact cyclic body-to-NED attitude is constructible");
    auto const body = vector_type::from_row_major({value_type{1}, value_type{2}, value_type{4}});
    auto const expected_navigation = vector_type::from_row_major({value_type{4}, value_type{1}, value_type{2}});
    test.expect(
        vector_near(formal_eskf::so3::rotate(permutation, body), expected_navigation, value_type{0}) &&
            vector_near(formal_eskf::so3::rotate(-permutation, body), expected_navigation, value_type{0}),
        profile,
        "independent exact cyclic permutation establishes body-to-NED direction and quaternion sign equivalence");

    using ahrs_types = formal_eskf::EskfTypes<Linalg, formal_eskf::configuration::Ahrs>;
    typename ahrs_types::nominal_state_type prior, output;
    typename ahrs_types::error_state_type error;
    test.expect(quaternion_type::from_coefficients(value_type{0}, value_type{1}, value_type{0}, value_type{0},
                                                   value_type{0.5}, prior.q_nb) == formal_eskf::Status::success,
                profile, "exact half-turn prior is constructible");
    // theta is small enough that higher terms round away; the Hamilton product
    // [0,1,0,0] * [1,0,theta/2,0] has positive final component, unlike left multiplication.
    value_type const theta = std::ldexp(value_type{1}, -(std::numeric_limits<value_type>::digits + 2));
    error.delta_theta_b(1U) = theta;
    auto const status = formal_eskf::inject_nominal(prior, error, value_type{0.5}, output);
    test.expect(status == formal_eskf::Status::success && output.q_nb.q0() == value_type{0} &&
                    output.q_nb.q1() == value_type{1} && output.q_nb.q2() == value_type{0} &&
                    output.q_nb.q3() == theta * value_type{0.5},
                profile, "local rotation-vector error right-multiplies with radians and scalar-first Hamilton signs");

    using ins_types = formal_eskf::EskfTypes<Linalg, formal_eskf::configuration::Ins>;
    typename ins_types::nominal_state_type stationary, predicted;
    typename ins_types::imu_sample_type imu;
    typename ins_types::parameter_type parameters;
    parameters.gravity_n(2U) = value_type{10};
    parameters.minimum_quaternion_norm = value_type{0.5};
    parameters.quaternion_squared_norm_tolerance = value_type{0.125};
    parameters.dt_min = parameters.dt_max = value_type{0.5};
    imu.specific_force_b(2U) = value_type{-10};
    test.expect(formal_eskf::predict_nominal(stationary, imu, value_type{0.5}, parameters, predicted) ==
                        formal_eskf::Status::success &&
                    vector_near(predicted.p_n, vector_type::zero(), value_type{0}) &&
                    vector_near(predicted.v_n, vector_type::zero(), value_type{0}),
                profile,
                "positive-Down gravity cancels stationary body specific force without a hidden unit conversion");
}

} /* end namespace */

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Case const cases[] = {
        {"TC-E-STATE-shape", "INS+AHRS.binary32", test_shape<formal_eskf::test::Backend<float>>},
        {"TC-E-STATE-shape", "INS+AHRS.binary64", test_shape<formal_eskf::test::Backend<double>>},
        {"TC-E-STATE-frame", "INS+AHRS.binary32", test_frame<formal_eskf::test::Backend<float>>},
        {"TC-E-STATE-frame", "INS+AHRS.binary64", test_frame<formal_eskf::test::Backend<double>>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
