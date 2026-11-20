/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string_view>
#include <type_traits>

#include <formal_eskf/eskf/prediction.hpp>
#include "test_backend.hpp"

#include "test_support.hpp"
#include "test_cases.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::predict_nominal;
using formal_eskf::Status;
using formal_eskf::test::near;

template <typename Linalg>
auto vector3(typename Linalg::value_type x, typename Linalg::value_type y, typename Linalg::value_type z)
{
    typename Linalg::value_type const values[3] = {x, y, z};
    return formal_eskf::linalg::Vector<Linalg, 3U>::from_row_major(values);
}

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

template <typename Linalg, typename Configuration> struct NominalPredictionFixture
{
    using types = formal_eskf::EskfTypes<Linalg, Configuration>;
    using value_type = typename types::value_type;
    using state_type = typename types::nominal_state_type;

    state_type state{};
    typename types::imu_sample_type imu{};
    typename types::parameter_type parameters{};

    NominalPredictionFixture()
    {
        parameters.minimum_quaternion_norm = static_cast<value_type>(1.0e-6);
        parameters.quaternion_squared_norm_tolerance = value_type{32} * std::numeric_limits<value_type>::epsilon();
        parameters.dt_min = static_cast<value_type>(0.001);
        parameters.dt_max = value_type{1};
        if constexpr (std::is_same_v<Configuration, formal_eskf::configuration::Ins>)
        {
            parameters.gravity_n = vector3<Linalg>(0, 0, 10);
            imu.specific_force_b = vector3<Linalg>(0, 0, -10);
        }
    }
};

template <typename Scalar> bool same_scalar(Scalar left, Scalar right)
{
    if (std::isnan(left) || std::isnan(right))
    {
        return std::isnan(left) && std::isnan(right);
    }
    return left == right && (left != Scalar{0} || std::signbit(left) == std::signbit(right));
}

template <typename Vector> bool same_vector(Vector const & left, Vector const & right)
{
    for (std::size_t axis = 0U; axis < Vector::row_count; ++axis)
    {
        if (!same_scalar(left(axis), right(axis)))
        {
            return false;
        }
    }
    return true;
}

template <typename State> bool same_state(State const & left, State const & right)
{
    bool same = same_vector(left.q_nb.coefficients(), right.q_nb.coefficients());
    if constexpr (requires { left.p_n; })
    {
        same = same && same_vector(left.p_n, right.p_n) && same_vector(left.v_n, right.v_n) &&
               same_vector(left.b_a, right.b_a) && same_vector(left.b_g, right.b_g);
    }
    return same;
}

template <typename Parameters> bool same_parameters(Parameters const & left, Parameters const & right)
{
    bool same = same_scalar(left.minimum_quaternion_norm, right.minimum_quaternion_norm) &&
                same_scalar(left.quaternion_squared_norm_tolerance, right.quaternion_squared_norm_tolerance) &&
                same_scalar(left.dt_min, right.dt_min) && same_scalar(left.dt_max, right.dt_max) &&
                same_vector(left.process_noise.angular_rate_variance, right.process_noise.angular_rate_variance);
    if constexpr (requires { left.gravity_n; })
    {
        same = same && same_vector(left.gravity_n, right.gravity_n) &&
               same_vector(left.process_noise.specific_force_variance, right.process_noise.specific_force_variance) &&
               same_vector(left.process_noise.accelerometer_bias_random_walk_variance_density,
                           right.process_noise.accelerometer_bias_random_walk_variance_density) &&
               same_vector(left.process_noise.gyroscope_bias_random_walk_variance_density,
                           right.process_noise.gyroscope_bias_random_walk_variance_density);
    }
    return same;
}

// Check both legal output arrangements, independently of any numeric oracle.
// Initial output is deliberately different from the prior, including NaN and
// signed zero in the directly constructible Euclidean fields.
template <typename Linalg, typename Configuration>
void expect_prediction_failure(TestContext & test, std::string_view profile,
                               NominalPredictionFixture<Linalg, Configuration> const & fixture,
                               typename Linalg::value_type dt, Status expected)
{
    using value_type = typename Linalg::value_type;
    for (bool const alias : {false, true})
    {
        auto prior = fixture.state;
        auto separate = fixture.state;
        separate.q_nb = -separate.q_nb;
        if constexpr (std::is_same_v<Configuration, formal_eskf::configuration::Ins>)
        {
            separate.p_n =
                vector3<Linalg>(std::numeric_limits<value_type>::quiet_NaN(), -value_type{0}, value_type{17});
            separate.v_n = vector3<Linalg>(value_type{-8}, value_type{4}, value_type{-2});
            separate.b_a = vector3<Linalg>(value_type{1}, value_type{2}, value_type{3});
            separate.b_g = vector3<Linalg>(value_type{4}, value_type{5}, value_type{6});
        }
        auto & output = alias ? prior : separate;
        auto const before = output;
        auto const imu_before = fixture.imu;
        auto const parameters_before = fixture.parameters;
        Status const status = predict_nominal(prior, fixture.imu, dt, fixture.parameters, output);
        test.expect(status == expected, profile, "isolated failure has the specified status");
        test.expect(same_state(output, before) && same_state(prior, fixture.state), profile,
                    "failure preserves full output and borrowed prior for either legal alias");
        test.expect(same_vector(fixture.imu.angular_rate_b, imu_before.angular_rate_b) &&
                        same_vector(fixture.imu.specific_force_b, imu_before.specific_force_b),
                    profile, "prediction does not modify borrowed IMU inputs");
        test.expect(same_parameters(fixture.parameters, parameters_before), profile,
                    "failure does not modify any parameter or process-noise field");
    }
}

template <typename Scalar> Scalar integrated_angle(Scalar angle)
{
#if ESKF_QUAT_APPROX
    // Independent oracle: normalized Euler rotates by 2*atan(theta/2).
    return Scalar{2} * std::atan(angle / Scalar{2});
#else
    return angle;
#endif
}

template <typename Linalg, typename Configuration>
void test_attitude_prediction(TestContext & test, std::string_view profile)
{
    // Retained numerical regression envelope for these bounded fixtures only.
    // It does not close the general libm/roundoff bound in C-NUMERICAL-ACCEPTANCE.
    constexpr auto tolerance = static_cast<typename Linalg::value_type>(
        std::is_same_v<typename Linalg::value_type, float> ? 2.0e-6F : 1.0e-12);
    using value_type = typename Linalg::value_type;
    NominalPredictionFixture<Linalg, Configuration> fixture;

    auto const initial = fixture.state;
    Status status = predict_nominal(fixture.state, fixture.imu, value_type{0.5}, fixture.parameters, fixture.state);
    test.expect(status == Status::success && same_state(fixture.state, initial), profile,
                "stationary prediction preserves the initial state in place");

    // These dyadic increments make all higher Taylor terms round away. This
    // checks the zero-limit branch without a libm accuracy allowance.
    for (int const exponent :
         {-(std::numeric_limits<value_type>::digits + 2), -(std::numeric_limits<value_type>::digits + 4)})
    {
        NominalPredictionFixture<Linalg, Configuration> small;
        value_type const rate = std::ldexp(value_type{1}, exponent);
        small.imu.angular_rate_b = vector3<Linalg>(rate, -rate, value_type{0});
        auto small_output = small.state;
        status = predict_nominal(small.state, small.imu, value_type{0.5}, small.parameters, small_output);
        test.expect(status == Status::success && small_output.q_nb.q0() == value_type{1} &&
                        small_output.q_nb.q1() == rate * value_type{0.25} &&
                        small_output.q_nb.q2() == -rate * value_type{0.25} && small_output.q_nb.q3() == value_type{0},
                    profile, "decreasing exact dyadic increments retain the signed first-order limit");
    }
#if ESKF_QUAT_APPROX
    NominalPredictionFixture<Linalg, Configuration> exact_euler;
    exact_euler.imu.angular_rate_b = vector3<Linalg>(value_type{2}, value_type{2}, value_type{2});
    auto exact_output = exact_euler.state;
    status = predict_nominal(exact_euler.state, exact_euler.imu, value_type{1}, exact_euler.parameters, exact_output);
    test.expect(status == Status::success && exact_output.q_nb.q0() == value_type{0.5} &&
                    exact_output.q_nb.q1() == value_type{0.5} && exact_output.q_nb.q2() == value_type{0.5} &&
                    exact_output.q_nb.q3() == value_type{0.5},
                profile, "Euler candidate [1,1,1,1] has exact norm two and exact normalized coefficients");
#endif

    value_type const half_sqrt_two = std::sqrt(value_type{0.5});
    status = formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(
        half_sqrt_two, half_sqrt_two, value_type{0}, value_type{0}, fixture.parameters.minimum_quaternion_norm,
        fixture.state.q_nb);
    test.expect(status == Status::success, profile, "non-identity attitude fixture is valid");
    // INS uses the same old attitude, so cancel gravity for this tilted fixture.
    if constexpr (std::is_same_v<Configuration, formal_eskf::configuration::Ins>)
    {
        fixture.imu.specific_force_b = vector3<Linalg>(0, -10, 0);
    }
    fixture.imu.angular_rate_b = vector3<Linalg>(0, 1, 0);
    auto output = fixture.state;
    status = predict_nominal(fixture.state, fixture.imu, value_type{1}, fixture.parameters, output);
    value_type const half_angle = integrated_angle(value_type{1}) / value_type{2};
    value_type const c = half_sqrt_two * std::cos(half_angle);
    value_type const s = half_sqrt_two * std::sin(half_angle);
    test.expect(status == Status::success && near(output.q_nb.q0(), c, tolerance) &&
                    near(output.q_nb.q1(), c, tolerance) && near(output.q_nb.q2(), s, tolerance) &&
                    near(output.q_nb.q3(), s, tolerance),
                profile, "selected integrator uses right multiplication by the body-frame increment");

    auto negative = fixture.state;
    negative.q_nb = -negative.q_nb;
    status = predict_nominal(negative, fixture.imu, value_type{1}, fixture.parameters, negative);
    test.expect(status == Status::success &&
                    vector_near(negative.q_nb.coefficients(), -output.q_nb.coefficients(), tolerance),
                profile, "prediction preserves quaternion sign equivalence");

    // A mixed-axis rate exercises every coefficient in the Euler kernel.
    NominalPredictionFixture<Linalg, Configuration> mixed;
    mixed.imu.angular_rate_b = vector3<Linalg>(1, -2, 3);
    value_type const dt = value_type{0.25};
    value_type const rate_norm = std::sqrt(value_type{14});
    value_type const angle = integrated_angle(rate_norm * dt);
    value_type const scale = std::sin(angle / value_type{2}) / rate_norm;
    status = predict_nominal(mixed.state, mixed.imu, dt, mixed.parameters, mixed.state);
    test.expect(status == Status::success && near(mixed.state.q_nb.q0(), std::cos(angle / value_type{2}), tolerance) &&
                    near(mixed.state.q_nb.q1(), scale, tolerance) &&
                    near(mixed.state.q_nb.q2(), value_type{-2} * scale, tolerance) &&
                    near(mixed.state.q_nb.q3(), value_type{3} * scale, tolerance),
                profile, "mixed-axis prediction matches an independent axis-angle oracle");

    status = formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(
        value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5}, mixed.parameters.minimum_quaternion_norm,
        mixed.state.q_nb);
    test.expect(status == Status::success, profile, "four-nonzero-coefficient attitude fixture is valid");
    status = predict_nominal(mixed.state, mixed.imu, dt, mixed.parameters, mixed.state);
    value_type const half_c = value_type{0.5} * std::cos(angle / value_type{2});
    test.expect(status == Status::success && near(mixed.state.q_nb.q0(), half_c - scale, tolerance) &&
                    near(mixed.state.q_nb.q1(), half_c + value_type{3} * scale, tolerance) &&
                    near(mixed.state.q_nb.q2(), half_c - value_type{2} * scale, tolerance) &&
                    near(mixed.state.q_nb.q3(), half_c, tolerance),
                profile, "mixed-axis prediction includes all cross terms for a general initial attitude");

    NominalPredictionFixture<Linalg, Configuration> repeated;
    repeated.imu.angular_rate_b = vector3<Linalg>(0, 0, 1);
    value_type const step = static_cast<value_type>(0.01);
    bool all_succeeded = true;
    for (std::size_t index = 0U; index < 100U; ++index)
    {
        status = predict_nominal(repeated.state, repeated.imu, step, repeated.parameters, repeated.state);
        all_succeeded = all_succeeded && status == Status::success;
    }
    value_type const total_half_angle = value_type{50} * integrated_angle(step);
    test.expect(all_succeeded &&
                    near(repeated.state.q_nb.q0(), std::cos(total_half_angle), tolerance * value_type{10}) &&
                    near(repeated.state.q_nb.q3(), std::sin(total_half_angle), tolerance * value_type{10}) &&
                    near(formal_eskf::linalg::norm(repeated.state.q_nb.coefficients()), value_type{1}, tolerance),
                profile, "repeated constant-rate prediction follows the selected model and preserves unit norm");
}

template <typename Linalg> void test_ins_translation(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    constexpr value_type tolerance = value_type{0}; // All translation fixture intermediates are dyadic.
    NominalPredictionFixture<Linalg, formal_eskf::configuration::Ins> fixture;
    fixture.state.p_n = vector3<Linalg>(1, 2, 3);
    fixture.state.v_n = vector3<Linalg>(4, 5, 6);
    fixture.state.b_a = vector3<Linalg>(1, 2, 3);
    fixture.state.b_g = vector3<Linalg>(value_type{0.25}, value_type{-0.5}, value_type{0.125});
    fixture.imu.specific_force_b = vector3<Linalg>(1, 2, -7);
    fixture.imu.angular_rate_b = fixture.state.b_g;
    auto const initial = fixture.state;
    auto output = initial;

    Status status = predict_nominal(initial, fixture.imu, value_type{0.5}, fixture.parameters, output);
    test.expect(
        status == Status::success && vector_near(output.p_n, vector3<Linalg>(3, value_type{4.5}, 6), tolerance) &&
            vector_near(output.v_n, initial.v_n, tolerance) &&
            formal_eskf::so3::same_coefficients(output.q_nb, initial.q_nb) &&
            vector_near(output.b_a, initial.b_a, value_type{0}) && vector_near(output.b_g, initial.b_g, value_type{0}),
        profile, "INS removes both biases, cancels NED gravity, and preserves constant velocity");

    fixture.imu.specific_force_b = vector3<Linalg>(3, 2, -7);
    status = predict_nominal(initial, fixture.imu, value_type{0.5}, fixture.parameters, output);
    test.expect(status == Status::success &&
                    vector_near(output.p_n, vector3<Linalg>(value_type{3.25}, value_type{4.5}, 6), tolerance) &&
                    vector_near(output.v_n, vector3<Linalg>(5, 5, 6), tolerance),
                profile, "constant acceleration updates position from old velocity with the one-half term");
    auto alias = initial;
    status = predict_nominal(alias, fixture.imu, value_type{0.5}, fixture.parameters, alias);
    test.expect(status == Status::success && same_state(alias, output) && same_state(fixture.state, initial), profile,
                "aliased and separate outputs agree, while separate input remains unchanged");

    NominalPredictionFixture<Linalg, formal_eskf::configuration::Ins> tilted;
    status = formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(
        value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5}, tilted.parameters.minimum_quaternion_norm,
        tilted.state.q_nb);
    test.expect(status == Status::success, profile, "tilted INS fixture is valid");
    tilted.imu.specific_force_b = vector3<Linalg>(0, -10, 0);
    tilted.imu.angular_rate_b = vector3<Linalg>(0, 1, 0);
    auto const old_attitude = tilted.state.q_nb;
    status = predict_nominal(tilted.state, tilted.imu, value_type{0.5}, tilted.parameters, tilted.state);
    test.expect(status == Status::success && vector_near(tilted.state.p_n, vector3<Linalg>(0, 0, 0), tolerance) &&
                    vector_near(tilted.state.v_n, vector3<Linalg>(0, 0, 0), tolerance) &&
                    !formal_eskf::so3::same_rotation(tilted.state.q_nb, old_attitude, tolerance),
                profile, "acceleration uses the current input quaternion, not identity or the predicted attitude");
}

template <typename Linalg, typename Configuration> void test_time_domain(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    NominalPredictionFixture<Linalg, Configuration> fixture;
    fixture.parameters.dt_min = value_type{0.25};
    fixture.parameters.dt_max = value_type{0.5};
    for (value_type const dt : {value_type{0.25}, value_type{0.5}})
    {
        auto output = fixture.state;
        test.expect(predict_nominal(fixture.state, fixture.imu, dt, fixture.parameters, output) == Status::success,
                    profile, "time endpoints are inclusive");
    }
    auto singleton = fixture;
    singleton.parameters.dt_min = singleton.parameters.dt_max;
    singleton.parameters.minimum_quaternion_norm = value_type{1};
    auto output = singleton.state;
    test.expect(predict_nominal(singleton.state, singleton.imu, value_type{0.5}, singleton.parameters, output) ==
                    Status::success,
                profile, "equal time bounds and norm threshold exactly one are valid");

    value_type const nan = std::numeric_limits<value_type>::quiet_NaN();
    value_type const infinity = std::numeric_limits<value_type>::infinity();
    for (value_type const dt : {value_type{0}, value_type{-1}, std::nextafter(value_type{0.25}, value_type{0}),
                                std::nextafter(value_type{0.5}, value_type{1}), nan, infinity})
    {
        expect_prediction_failure(test, profile, fixture, dt,
                                  std::isfinite(dt) ? Status::out_of_range : Status::non_finite_input);
    }
    for (value_type const bad : {nan, infinity, -infinity})
    {
        auto invalid = fixture;
        invalid.parameters.dt_min = bad;
        expect_prediction_failure(test, profile, invalid, value_type{0.5}, Status::non_finite_input);
        invalid = fixture;
        invalid.parameters.dt_max = bad;
        expect_prediction_failure(test, profile, invalid, value_type{0.5}, Status::non_finite_input);
        invalid = fixture;
        invalid.parameters.minimum_quaternion_norm = bad;
        expect_prediction_failure(test, profile, invalid, value_type{0.5}, Status::non_finite_input);
    }
    for (value_type const bad : {value_type{0}, value_type{-1}})
    {
        auto invalid = fixture;
        invalid.parameters.dt_min = bad;
        expect_prediction_failure(test, profile, invalid, value_type{0.5}, Status::domain_error);
    }
    auto invalid = fixture;
    invalid.parameters.dt_max = value_type{0.125};
    expect_prediction_failure(test, profile, invalid, value_type{0.5}, Status::domain_error);
    for (value_type const bad : {value_type{0}, value_type{-1}, std::nextafter(value_type{1}, value_type{2})})
    {
        invalid = fixture;
        invalid.parameters.minimum_quaternion_norm = bad;
        expect_prediction_failure(test, profile, invalid, value_type{0.5}, Status::domain_error);
    }
    // Adjacent validation stages: consumed nonfinite data must not change the
    // documented finite/domain/range ordering of the parameter checks.
    invalid = fixture;
    invalid.imu.angular_rate_b(0U) = nan;
    invalid.parameters.quaternion_squared_norm_tolerance = nan;
    expect_prediction_failure(test, profile, invalid, value_type{0}, Status::out_of_range);
    invalid.parameters.minimum_quaternion_norm = value_type{0};
    expect_prediction_failure(test, profile, invalid, value_type{0}, Status::domain_error);
    invalid.parameters.dt_max = infinity;
    expect_prediction_failure(test, profile, invalid, value_type{0}, Status::non_finite_input);
}

template <typename Linalg, typename Configuration>
void test_consumed_fields(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    NominalPredictionFixture<Linalg, Configuration> fixture;
    value_type const nan = std::numeric_limits<value_type>::quiet_NaN();
    value_type const infinity = std::numeric_limits<value_type>::infinity();
    for (value_type const bad : {nan, infinity, -infinity})
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            auto invalid = fixture;
            invalid.imu.angular_rate_b(axis) = bad;
            expect_prediction_failure(test, profile, invalid, value_type{0.5}, Status::non_finite_input);
            if constexpr (std::is_same_v<Configuration, formal_eskf::configuration::Ins>)
            {
                invalid = fixture;
                invalid.imu.specific_force_b(axis) = bad;
                expect_prediction_failure(test, profile, invalid, value_type{0.5}, Status::non_finite_input);
                invalid = fixture;
                invalid.parameters.gravity_n(axis) = bad;
                expect_prediction_failure(test, profile, invalid, value_type{0.5}, Status::non_finite_input);
                for (auto member : {&Configuration::template NominalState<Linalg>::p_n,
                                    &Configuration::template NominalState<Linalg>::v_n,
                                    &Configuration::template NominalState<Linalg>::b_a,
                                    &Configuration::template NominalState<Linalg>::b_g})
                {
                    invalid = fixture;
                    (invalid.state.*member)(axis) = bad;
                    expect_prediction_failure(test, profile, invalid, value_type{0.5}, Status::non_finite_input);
                }
            }
        }
    }

    fixture.imu.angular_rate_b = vector3<Linalg>(value_type{0.25}, value_type{-0.5}, value_type{1});
    auto reference = fixture.state;
    test.expect(predict_nominal(fixture.state, fixture.imu, value_type{0.5}, fixture.parameters, reference) ==
                        Status::success &&
                    !same_state(reference, fixture.state),
                profile, "unused-input comparison starts from a nontrivial successful update");
    auto unused = fixture;
    unused.parameters.process_noise.angular_rate_variance = vector3<Linalg>(nan, infinity, value_type{-1});
    if constexpr (std::is_same_v<Configuration, formal_eskf::configuration::Ins>)
    {
        unused.parameters.process_noise.specific_force_variance = vector3<Linalg>(infinity, nan, value_type{-1});
        unused.parameters.process_noise.accelerometer_bias_random_walk_variance_density =
            vector3<Linalg>(value_type{-1}, nan, infinity);
        unused.parameters.process_noise.gyroscope_bias_random_walk_variance_density =
            vector3<Linalg>(nan, value_type{-1}, infinity);
    }
    auto output = fixture.state;
    test.expect(predict_nominal(unused.state, unused.imu, value_type{0.5}, unused.parameters, output) ==
                        Status::success &&
                    same_state(output, reference),
                profile, "all process-noise fields are ignored by nominal prediction");
    if constexpr (std::is_same_v<Configuration, formal_eskf::configuration::Ahrs>)
    {
        unused.imu.specific_force_b = vector3<Linalg>(nan, infinity, -infinity);
        test.expect(predict_nominal(unused.state, unused.imu, value_type{0.5}, fixture.parameters, output) ==
                            Status::success &&
                        same_state(output, reference),
                    profile, "AHRS ignores specific force");
        test.expect(predict_nominal(unused.state, unused.imu, value_type{0.5}, unused.parameters, output) ==
                            Status::success &&
                        same_state(output, reference),
                    profile, "AHRS ignores simultaneous unused-field faults");
    }
}

template <typename Linalg, typename Configuration>
void test_arithmetic_failures(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    value_type const maximum = std::numeric_limits<value_type>::max();
    NominalPredictionFixture<Linalg, Configuration> fixture;
    auto overflow = fixture;
    overflow.parameters.dt_max = value_type{2};
    overflow.imu.angular_rate_b(0U) = maximum;
    expect_prediction_failure(test, profile, overflow, value_type{2}, Status::non_finite_result);
    // Finite increment, but its squared length overflows in the selected map.
    overflow = fixture;
    overflow.imu.angular_rate_b(0U) = maximum;
    expect_prediction_failure(test, profile, overflow, value_type{0.5}, Status::non_finite_result);
    if constexpr (std::is_same_v<Configuration, formal_eskf::configuration::Ins>)
    {
        overflow = fixture;
        overflow.state.b_g(0U) = -maximum;
        overflow.imu.angular_rate_b(0U) = maximum;
        expect_prediction_failure(test, profile, overflow, value_type{0.5}, Status::non_finite_result);
        overflow = fixture;
        overflow.state.b_a(0U) = -maximum;
        overflow.imu.specific_force_b(0U) = maximum;
        expect_prediction_failure(test, profile, overflow, value_type{0.5}, Status::non_finite_result);
        overflow = fixture;
        overflow.imu.angular_rate_b(2U) = value_type{1};
        overflow.state.p_n(0U) = maximum;
        overflow.state.v_n(0U) = maximum;
        expect_prediction_failure(test, profile, overflow, value_type{0.5}, Status::non_finite_result);
        overflow = fixture;
        overflow.imu.angular_rate_b(2U) = value_type{1};
        overflow.state.v_n(0U) = maximum;
        overflow.imu.specific_force_b(0U) = maximum;
        expect_prediction_failure(test, profile, overflow, value_type{0.5}, Status::non_finite_result);
        overflow = fixture;
        auto const attitude_status = formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(
            value_type{2}, value_type{0}, value_type{0}, value_type{1}, overflow.parameters.minimum_quaternion_norm,
            overflow.state.q_nb);
        test.expect(attitude_status == Status::success, profile, "nonaxis overflow attitude is constructible");
        // R has first row [3/5,-4/5,0]: the rotated force is 7/5*max,
        // independent of gravity or later integration arithmetic.
        overflow.imu.specific_force_b = vector3<Linalg>(maximum, -maximum, value_type{0});
        overflow.parameters.gravity_n = vector3<Linalg>(0, 0, 0);
        expect_prediction_failure(test, profile, overflow, value_type{0.5}, Status::non_finite_result);
        overflow = fixture;
        overflow.imu.specific_force_b(0U) = maximum;
        overflow.parameters.gravity_n(0U) = maximum;
        expect_prediction_failure(test, profile, overflow, value_type{0.5}, Status::non_finite_result);
    }
}

template <typename Linalg, typename Configuration> void test_alias_frame(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    NominalPredictionFixture<Linalg, Configuration> fixture;
    fixture.imu.angular_rate_b = vector3<Linalg>(value_type{0.5}, value_type{-1}, value_type{2});
    if constexpr (std::is_same_v<Configuration, formal_eskf::configuration::Ins>)
    {
        fixture.state.p_n = vector3<Linalg>(value_type{1}, -value_type{0}, value_type{3});
        fixture.state.v_n = vector3<Linalg>(value_type{4}, value_type{5}, value_type{-6});
        fixture.state.b_a = vector3<Linalg>(value_type{0.5}, value_type{-0.25}, value_type{0.125});
        fixture.state.b_g = vector3<Linalg>(value_type{-0.125}, value_type{0.25}, value_type{-0.5});
    }
    auto const before = fixture.state;
    auto const imu_before = fixture.imu;
    auto const parameters_before = fixture.parameters;
    auto separate = fixture.state;
    separate.q_nb = -separate.q_nb;
    auto alias = fixture.state;
    auto const separate_status =
        predict_nominal(fixture.state, fixture.imu, value_type{0.5}, fixture.parameters, separate);
    auto const alias_status = predict_nominal(alias, fixture.imu, value_type{0.5}, fixture.parameters, alias);
    test.expect(separate_status == Status::success && alias_status == separate_status && same_state(alias, separate) &&
                    same_state(fixture.state, before),
                profile, "whole-state alias equals separate publication and preserves the borrowed prior");
    test.expect(same_vector(fixture.imu.angular_rate_b, imu_before.angular_rate_b) &&
                    same_vector(fixture.imu.specific_force_b, imu_before.specific_force_b) &&
                    same_parameters(fixture.parameters, parameters_before),
                profile, "successful alias and separate calls preserve all IMU and parameter fields");
    auto invalid = fixture;
    invalid.parameters.dt_min = value_type{0};
    expect_prediction_failure(test, profile, invalid, value_type{0.5}, Status::domain_error);
    invalid = fixture;
    invalid.imu.angular_rate_b(0U) = std::numeric_limits<value_type>::quiet_NaN();
    expect_prediction_failure(test, profile, invalid, value_type{0.5}, Status::non_finite_input);
    if constexpr (std::is_same_v<Configuration, formal_eskf::configuration::Ins>)
    {
        invalid = fixture;
        invalid.state.p_n(0U) = std::numeric_limits<value_type>::max();
        invalid.state.v_n(0U) = std::numeric_limits<value_type>::max();
        expect_prediction_failure(test, profile, invalid, value_type{0.5}, Status::non_finite_result);
    }
}

template <typename Linalg, typename Configuration>
void test_prior_validation(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using quaternion_type = formal_eskf::so3::UnitQuaternion<Linalg>;
    NominalPredictionFixture<Linalg, Configuration> fixture;
    // Squaring the smaller component underflows to zero. This constructs an
    // exactly [1, 1/2, 0, 0] non-unit value through the PUBLIC constructor,
    // without private-member access or assuming that the type enforces a bound.
    int const exponent = (std::numeric_limits<value_type>::min_exponent - std::numeric_limits<value_type>::digits) / 2;
    value_type const tiny = std::ldexp(value_type{1}, exponent);
    quaternion_type non_unit;
    auto const construction = quaternion_type::from_coefficients(
        tiny, tiny * value_type{0.5}, value_type{0}, value_type{0}, std::numeric_limits<value_type>::min(), non_unit);
    test.expect(construction == Status::success && non_unit.q0() == value_type{1} && non_unit.q1() == value_type{0.5},
                profile, "public construction exposes a non-unit IEEE fixture");

    for (auto const & q : {non_unit, -non_unit})
    {
        auto prior = fixture.state;
        prior.q_nb = q;
        for (bool alias : {false, true})
        {
            auto state = prior;
            auto separate = fixture.state;
            separate.q_nb = -separate.q_nb;
            auto & output = alias ? state : separate;
            auto const before = output;
            auto const status = predict_nominal(state, fixture.imu, value_type{0.5}, fixture.parameters, output);
            test.expect(status == Status::invalid_quaternion_norm && same_state(output, before) &&
                            same_state(state, prior),
                        profile, "non-unit priors of either sign are rejected atomically");
        }
    }

    for (value_type bound : {value_type{0}, value_type{-1}, value_type{1}, value_type{2},
                             std::numeric_limits<value_type>::infinity(), std::numeric_limits<value_type>::quiet_NaN()})
    {
        auto invalid = fixture;
        invalid.parameters.quaternion_squared_norm_tolerance = bound;
        expect_prediction_failure(test, profile, invalid, value_type{0.5},
                                  std::isfinite(bound) ? Status::domain_error : Status::non_finite_input);
    }

    auto prior = fixture.state;
    prior.q_nb = non_unit;
    auto parameters = fixture.parameters;
    // A deliberately wide TEST tolerance establishes inclusivity, not a
    // suggested deployment tolerance. The accepted old state is not repaired.
    parameters.quaternion_squared_norm_tolerance = value_type{0.25};
    auto output = fixture.state;
    auto status = predict_nominal(prior, fixture.imu, value_type{0.5}, parameters, output);
    test.expect(status == Status::success && formal_eskf::so3::same_coefficients(prior.q_nb, non_unit), profile,
                "the exact residual boundary is inclusive and does not mutate the prior");
    if constexpr (std::is_same_v<Configuration, formal_eskf::configuration::Ins>)
    {
        test.expect(output.p_n(2U) == value_type{0.3125} && output.v_n(2U) == value_type{1.25}, profile,
                    "INS uses the accepted old quaternion without silently normalizing it first");
    }
    parameters.quaternion_squared_norm_tolerance = std::nextafter(value_type{0.25}, value_type{1});
    status = predict_nominal(prior, fixture.imu, value_type{0.5}, parameters, output);
    test.expect(status == Status::success, profile, "a residual just inside the tolerance is accepted");
    parameters.quaternion_squared_norm_tolerance = std::nextafter(value_type{0.25}, value_type{0});
    output = fixture.state;
    status = predict_nominal(prior, fixture.imu, value_type{0.5}, parameters, output);
    test.expect(status == Status::invalid_quaternion_norm && same_state(output, fixture.state), profile,
                "a residual just outside the tolerance is rejected");

    auto large = non_unit;
    for (unsigned step = 0U; step < 32U && std::isfinite(formal_eskf::linalg::squared_norm(large.coefficients()));
         ++step)
    {
        large = large * large;
    }
    test.expect(formal_eskf::linalg::all_finite(large.coefficients()) &&
                    !std::isfinite(formal_eskf::linalg::squared_norm(large.coefficients())),
                profile, "finite quaternion fixture overflows only its squared norm");
    auto invalid = fixture;
    invalid.state.q_nb = large;
    expect_prediction_failure(test, profile, invalid, value_type{0.5}, Status::non_finite_result);
    invalid.state.q_nb = large * large;
    expect_prediction_failure(test, profile, invalid, value_type{0.5}, Status::non_finite_input);
    invalid.parameters.quaternion_squared_norm_tolerance = value_type{0};
    expect_prediction_failure(test, profile, invalid, value_type{0.5}, Status::domain_error);
    invalid.parameters.quaternion_squared_norm_tolerance = std::numeric_limits<value_type>::quiet_NaN();
    expect_prediction_failure(test, profile, invalid, value_type{0.5}, Status::non_finite_input);
}

} /* end namespace */

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using float_backend = formal_eskf::test::Backend<float>;
    using double_backend = formal_eskf::test::Backend<double>;
    using formal_eskf::configuration::Ahrs;
    using formal_eskf::configuration::Ins;
    formal_eskf::test::Case const cases[] = {
        {"TC-E-PRED-ATTITUDE-selected-map", "INS.binary32", test_attitude_prediction<float_backend, Ins>},
        {"TC-E-PRED-ATTITUDE-selected-map", "INS.binary64", test_attitude_prediction<double_backend, Ins>},
        {"TC-E-PRED-ATTITUDE-selected-map", "AHRS.binary32", test_attitude_prediction<float_backend, Ahrs>},
        {"TC-E-PRED-ATTITUDE-selected-map", "AHRS.binary64", test_attitude_prediction<double_backend, Ahrs>},
        {"TC-E-PRED-INS-frozen-acceleration", "INS.binary32", test_ins_translation<float_backend>},
        {"TC-E-PRED-INS-frozen-acceleration", "INS.binary64", test_ins_translation<double_backend>},
        {"TC-E-PRED-time-domain", "INS.binary32", test_time_domain<float_backend, Ins>},
        {"TC-E-PRED-time-domain", "INS.binary64", test_time_domain<double_backend, Ins>},
        {"TC-E-PRED-time-domain", "AHRS.binary32", test_time_domain<float_backend, Ahrs>},
        {"TC-E-PRED-time-domain", "AHRS.binary64", test_time_domain<double_backend, Ahrs>},
        {"TC-E-PRED-prior-validation", "INS.binary32", test_prior_validation<float_backend, Ins>},
        {"TC-E-PRED-prior-validation", "INS.binary64", test_prior_validation<double_backend, Ins>},
        {"TC-E-PRED-prior-validation", "AHRS.binary32", test_prior_validation<float_backend, Ahrs>},
        {"TC-E-PRED-prior-validation", "AHRS.binary64", test_prior_validation<double_backend, Ahrs>},
        {"TC-E-PRED-consumed-fields", "INS.binary32", test_consumed_fields<float_backend, Ins>},
        {"TC-E-PRED-consumed-fields", "INS.binary64", test_consumed_fields<double_backend, Ins>},
        {"TC-E-PRED-consumed-fields", "AHRS.binary32", test_consumed_fields<float_backend, Ahrs>},
        {"TC-E-PRED-consumed-fields", "AHRS.binary64", test_consumed_fields<double_backend, Ahrs>},
        {"TC-E-PRED-arithmetic-failures", "INS.binary32", test_arithmetic_failures<float_backend, Ins>},
        {"TC-E-PRED-arithmetic-failures", "INS.binary64", test_arithmetic_failures<double_backend, Ins>},
        {"TC-E-PRED-arithmetic-failures", "AHRS.binary32", test_arithmetic_failures<float_backend, Ahrs>},
        {"TC-E-PRED-arithmetic-failures", "AHRS.binary64", test_arithmetic_failures<double_backend, Ahrs>},
        {"TC-E-PRED-alias-frame", "INS.binary32", test_alias_frame<float_backend, Ins>},
        {"TC-E-PRED-alias-frame", "INS.binary64", test_alias_frame<double_backend, Ins>},
        {"TC-E-PRED-alias-frame", "AHRS.binary32", test_alias_frame<float_backend, Ahrs>},
        {"TC-E-PRED-alias-frame", "AHRS.binary64", test_alias_frame<double_backend, Ahrs>},
    };
    std::cout << "Configuration: quat_approx=" << ESKF_QUAT_APPROX << '\n';
    return formal_eskf::test::run_cases(cases, argc, argv);
}
