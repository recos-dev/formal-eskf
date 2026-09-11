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

#include <formal_eskf/eskf/eskf.hpp>
#include <formal_eskf/linalg/backend/eigen.hpp>

#include "test_support.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::Status;
using formal_eskf::try_predict_nominal;
using formal_eskf::test::near;

template <typename Linalg>
[[nodiscard]] auto vector3(typename Linalg::value_type x, typename Linalg::value_type y, typename Linalg::value_type z)
{
    typename Linalg::value_type const values[3] = {x, y, z};
    return Linalg::template vector_type<3U>::from_row_major(values);
}

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
        parameters.dt_min = static_cast<value_type>(0.001);
        parameters.dt_max = value_type{1};
        if constexpr (std::is_same_v<Configuration, formal_eskf::configuration::Ins>)
        {
            parameters.gravity_n = vector3<Linalg>(0, 0, 10);
            imu.specific_force_b = vector3<Linalg>(0, 0, -10);
        }
    }
};

template <typename State> [[nodiscard]] bool same_state(State const & left, State const & right)
{
    using value_type = typename State::quaternion_type::value_type;
    bool same = formal_eskf::so3::same_coefficients(left.q_nb, right.q_nb);
    if constexpr (requires { left.p_n; })
    {
        same = same && vector_near(left.p_n, right.p_n, value_type{0}) &&
               vector_near(left.v_n, right.v_n, value_type{0}) && vector_near(left.b_a, right.b_a, value_type{0}) &&
               vector_near(left.b_g, right.b_g, value_type{0});
    }
    return same;
}

template <typename Scalar> [[nodiscard]] Scalar integrated_angle(Scalar angle)
{
#if ESKF_QUAT_APPROX
    // Independent oracle: normalized Euler rotates by 2*atan(theta/2).
    return Scalar{2} * std::atan(angle / Scalar{2});
#else
    return angle;
#endif
}

template <typename Linalg, typename Configuration>
void test_attitude_prediction(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    NominalPredictionFixture<Linalg, Configuration> fixture;

    auto const initial = fixture.state;
    Status status = try_predict_nominal(fixture.state, fixture.imu, value_type{0.5}, fixture.parameters, fixture.state);
    test.expect(status == Status::success && same_state(fixture.state, initial), profile,
                "stationary prediction preserves the initial state in place");

    value_type const half_sqrt_two = std::sqrt(value_type{0.5});
    status = formal_eskf::so3::UnitQuaternion<Linalg>::try_from_coefficients(
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
    status = try_predict_nominal(fixture.state, fixture.imu, value_type{1}, fixture.parameters, output);
    value_type const half_angle = integrated_angle(value_type{1}) / value_type{2};
    value_type const c = half_sqrt_two * std::cos(half_angle);
    value_type const s = half_sqrt_two * std::sin(half_angle);
    test.expect(status == Status::success && near(output.q_nb.q0(), c, tolerance) &&
                    near(output.q_nb.q1(), c, tolerance) && near(output.q_nb.q2(), s, tolerance) &&
                    near(output.q_nb.q3(), s, tolerance),
                profile, "selected integrator uses right multiplication by the body-frame increment");

    auto negative = fixture.state;
    negative.q_nb = -negative.q_nb;
    status = try_predict_nominal(negative, fixture.imu, value_type{1}, fixture.parameters, negative);
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
    status = try_predict_nominal(mixed.state, mixed.imu, dt, mixed.parameters, mixed.state);
    test.expect(status == Status::success && near(mixed.state.q_nb.q0(), std::cos(angle / value_type{2}), tolerance) &&
                    near(mixed.state.q_nb.q1(), scale, tolerance) &&
                    near(mixed.state.q_nb.q2(), value_type{-2} * scale, tolerance) &&
                    near(mixed.state.q_nb.q3(), value_type{3} * scale, tolerance),
                profile, "mixed-axis prediction matches an independent axis-angle oracle");

    status = formal_eskf::so3::UnitQuaternion<Linalg>::try_from_coefficients(
        value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5}, mixed.parameters.minimum_quaternion_norm,
        mixed.state.q_nb);
    test.expect(status == Status::success, profile, "four-nonzero-coefficient attitude fixture is valid");
    status = try_predict_nominal(mixed.state, mixed.imu, dt, mixed.parameters, mixed.state);
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
        status = try_predict_nominal(repeated.state, repeated.imu, step, repeated.parameters, repeated.state);
        all_succeeded = all_succeeded && status == Status::success;
    }
    value_type const total_half_angle = value_type{50} * integrated_angle(step);
    test.expect(all_succeeded &&
                    near(repeated.state.q_nb.q0(), std::cos(total_half_angle), tolerance * value_type{10}) &&
                    near(repeated.state.q_nb.q3(), std::sin(total_half_angle), tolerance * value_type{10}) &&
                    near(formal_eskf::linalg::norm(repeated.state.q_nb.coefficients()), value_type{1}, tolerance),
                profile, "repeated constant-rate prediction follows the selected model and preserves unit norm");
}

template <typename Linalg>
void test_ins_translation(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    NominalPredictionFixture<Linalg, formal_eskf::configuration::Ins> fixture;
    fixture.state.p_n = vector3<Linalg>(1, 2, 3);
    fixture.state.v_n = vector3<Linalg>(4, 5, 6);
    fixture.state.b_a = vector3<Linalg>(1, 2, 3);
    fixture.state.b_g = vector3<Linalg>(value_type{0.25}, value_type{-0.5}, value_type{0.125});
    fixture.imu.specific_force_b = vector3<Linalg>(1, 2, -7);
    fixture.imu.angular_rate_b = fixture.state.b_g;
    auto const initial = fixture.state;
    auto output = initial;

    Status status = try_predict_nominal(initial, fixture.imu, value_type{0.5}, fixture.parameters, output);
    test.expect(
        status == Status::success && vector_near(output.p_n, vector3<Linalg>(3, value_type{4.5}, 6), tolerance) &&
            vector_near(output.v_n, initial.v_n, tolerance) &&
            formal_eskf::so3::same_coefficients(output.q_nb, initial.q_nb) &&
            vector_near(output.b_a, initial.b_a, value_type{0}) && vector_near(output.b_g, initial.b_g, value_type{0}),
        profile, "INS removes both biases, cancels NED gravity, and preserves constant velocity");

    fixture.imu.specific_force_b = vector3<Linalg>(3, 2, -7);
    status = try_predict_nominal(initial, fixture.imu, value_type{0.5}, fixture.parameters, output);
    test.expect(status == Status::success &&
                    vector_near(output.p_n, vector3<Linalg>(value_type{3.25}, value_type{4.5}, 6), tolerance) &&
                    vector_near(output.v_n, vector3<Linalg>(5, 5, 6), tolerance),
                profile, "constant acceleration updates position from old velocity with the one-half term");
    auto alias = initial;
    status = try_predict_nominal(alias, fixture.imu, value_type{0.5}, fixture.parameters, alias);
    test.expect(status == Status::success && same_state(alias, output) && same_state(fixture.state, initial), profile,
                "aliased and separate outputs agree, while separate input remains unchanged");

    NominalPredictionFixture<Linalg, formal_eskf::configuration::Ins> tilted;
    value_type const h = std::sqrt(value_type{0.5});
    status = formal_eskf::so3::UnitQuaternion<Linalg>::try_from_coefficients(
        h, value_type{0}, h, value_type{0}, tilted.parameters.minimum_quaternion_norm, tilted.state.q_nb);
    test.expect(status == Status::success, profile, "tilted INS fixture is valid");
    tilted.imu.specific_force_b = vector3<Linalg>(10, 0, 0);
    tilted.imu.angular_rate_b = vector3<Linalg>(0, 1, 0);
    auto const old_attitude = tilted.state.q_nb;
    status = try_predict_nominal(tilted.state, tilted.imu, value_type{0.5}, tilted.parameters, tilted.state);
    test.expect(status == Status::success && vector_near(tilted.state.p_n, vector3<Linalg>(0, 0, 0), tolerance) &&
                    vector_near(tilted.state.v_n, vector3<Linalg>(0, 0, 0), tolerance) &&
                    !formal_eskf::so3::same_rotation(tilted.state.q_nb, old_attitude, tolerance),
                profile, "acceleration uses the current input quaternion, not identity or the predicted attitude");
}

template <typename Linalg, typename Configuration>
void test_prediction_validation(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    NominalPredictionFixture<Linalg, Configuration> fixture;
    auto const initial = fixture.state;
    value_type const infinity = std::numeric_limits<value_type>::infinity();
    value_type const nan = std::numeric_limits<value_type>::quiet_NaN();

    for (value_type const dt : {fixture.parameters.dt_min, fixture.parameters.dt_max})
    {
        auto output = initial;
        test.expect(try_predict_nominal(initial, fixture.imu, dt, fixture.parameters, output) == Status::success,
                    profile, "both time interval boundaries are inclusive");
    }
    for (value_type const dt :
         {value_type{0}, value_type{-1}, fixture.parameters.dt_min / value_type{2}, value_type{2}, infinity, nan})
    {
        auto alias = initial;
        Status const expected = std::isfinite(dt) ? Status::out_of_range : Status::non_finite_input;
        Status const status = try_predict_nominal(alias, fixture.imu, dt, fixture.parameters, alias);
        test.expect(status == expected && same_state(alias, initial), profile,
                    "invalid dt is rejected without modifying aliased state");
    }

    auto expect_failure = [&](auto const & input, auto const & imu, auto const & parameters, Status expected)
    {
        auto output = initial;
        output.q_nb = -output.q_nb;
        auto const sentinel = output;
        Status const status = try_predict_nominal(input, imu, value_type{0.5}, parameters, output);
        test.expect(status == expected && same_state(output, sentinel), profile,
                    "invalid input or arithmetic failure preserves separate output");
    };

    for (value_type const threshold : {value_type{0}, value_type{-1}, value_type{2}, infinity, nan})
    {
        auto parameters = fixture.parameters;
        parameters.minimum_quaternion_norm = threshold;
        expect_failure(initial, fixture.imu, parameters,
                       std::isfinite(threshold) ? Status::domain_error : Status::non_finite_input);
    }
    auto parameters = fixture.parameters;
    parameters.dt_min = value_type{0};
    expect_failure(initial, fixture.imu, parameters, Status::domain_error);
    parameters = fixture.parameters;
    parameters.dt_max = parameters.dt_min / value_type{2};
    expect_failure(initial, fixture.imu, parameters, Status::domain_error);
    parameters.dt_max = nan;
    expect_failure(initial, fixture.imu, parameters, Status::non_finite_input);

    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        auto imu = fixture.imu;
        imu.angular_rate_b.set(axis, nan);
        expect_failure(initial, imu, fixture.parameters, Status::non_finite_input);
    }
    auto imu = fixture.imu;
    imu.angular_rate_b = vector3<Linalg>(std::numeric_limits<value_type>::max(), 0, 0);
    expect_failure(initial, imu, fixture.parameters, Status::non_finite_result);

    if constexpr (std::is_same_v<Configuration, formal_eskf::configuration::Ins>)
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            imu = fixture.imu;
            imu.specific_force_b.set(axis, infinity);
            expect_failure(initial, imu, fixture.parameters, Status::non_finite_input);
            parameters = fixture.parameters;
            parameters.gravity_n.set(axis, nan);
            expect_failure(initial, fixture.imu, parameters, Status::non_finite_input);

            auto bad_state = initial;
            bad_state.p_n.set(axis, nan);
            expect_failure(bad_state, fixture.imu, fixture.parameters, Status::non_finite_input);
            bad_state = initial;
            bad_state.v_n.set(axis, infinity);
            expect_failure(bad_state, fixture.imu, fixture.parameters, Status::non_finite_input);
            bad_state = initial;
            bad_state.b_a.set(axis, nan);
            expect_failure(bad_state, fixture.imu, fixture.parameters, Status::non_finite_input);
            bad_state = initial;
            bad_state.b_g.set(axis, nan);
            expect_failure(bad_state, fixture.imu, fixture.parameters, Status::non_finite_input);
        }

        // Fail after successfully computing the candidate attitude.
        auto overflow_state = initial;
        overflow_state.p_n.set(0U, std::numeric_limits<value_type>::max());
        overflow_state.v_n.set(0U, std::numeric_limits<value_type>::max());
        imu = fixture.imu;
        imu.angular_rate_b = vector3<Linalg>(0, 0, 1);
        auto const saved = overflow_state;
        Status const status =
            try_predict_nominal(overflow_state, imu, value_type{0.5}, fixture.parameters, overflow_state);
        test.expect(status == Status::non_finite_result && same_state(overflow_state, saved), profile,
                    "late translation overflow rolls back attitude, position, velocity, and biases");

        auto bias_overflow = initial;
        bias_overflow.b_g.set(0U, -std::numeric_limits<value_type>::max());
        imu.angular_rate_b.set(0U, std::numeric_limits<value_type>::max());
        expect_failure(bias_overflow, imu, fixture.parameters, Status::non_finite_result);
    }
    else
    {
        // AHRS does not consume specific force or process noise in this step.
        imu = fixture.imu;
        imu.specific_force_b = vector3<Linalg>(nan, infinity, nan);
        parameters = fixture.parameters;
        parameters.process_noise.angular_rate_variance = vector3<Linalg>(nan, nan, nan);
        auto output = initial;
        test.expect(try_predict_nominal(initial, imu, value_type{0.5}, parameters, output) == Status::success, profile,
                    "AHRS nominal prediction ignores unused specific force and process noise");
    }
}

template <typename Linalg>
void run_prediction_tests(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    test_attitude_prediction<Linalg, formal_eskf::configuration::Ahrs>(test, profile, tolerance);
    test_attitude_prediction<Linalg, formal_eskf::configuration::Ins>(test, profile, tolerance);
    test_ins_translation<Linalg>(test, profile, tolerance);
    test_prediction_validation<Linalg, formal_eskf::configuration::Ahrs>(test, profile);
    test_prediction_validation<Linalg, formal_eskf::configuration::Ins>(test, profile);
}

} /* end namespace */

int main()
{
    TestContext test;
    Eigen::internal::set_is_malloc_allowed(false);
    run_prediction_tests<formal_eskf::linalg::EigenBackend<float>>(test, "binary32", 2.0e-6F);
    run_prediction_tests<formal_eskf::linalg::EigenBackend<double>>(test, "binary64", 1.0e-12);
    if (test.failures() != 0)
    {
        std::cerr << test.failures() << " ESKF prediction test(s) failed\n";
        return 1;
    }
    std::cout << "All ESKF nominal prediction tests passed (" << (ESKF_QUAT_APPROX ? "normalized Euler" : "Exp")
              << ")\n";
    return 0;
}
