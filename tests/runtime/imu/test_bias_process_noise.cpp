/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <array>
#include <bit>
#include <cmath>
#include <iostream>
#include <limits>
#include <string_view>

#include <formal_eskf/eskf/eskf.hpp>
#include <formal_eskf/eskf/velocity.hpp>
#include <formal_eskf/runtime/imu.hpp>
#include <formal_eskf/runtime/vertical_acceleration_health.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{

using formal_eskf::Status;
using TestContext = formal_eskf::test::Context;

template <typename Linalg> struct BiasProcessNoiseFixture
{
    using value_type = typename Linalg::value_type;
    using vector_type = formal_eskf::linalg::Vector<Linalg, 3U>;
    using noise_type = formal_eskf::configuration::Ins::ProcessNoise<Linalg>;
    using covariance_type = formal_eskf::linalg::Matrix<Linalg, 15U, 15U>;
    noise_type noise{vector_type::from_row_major({value_type{1}, value_type{2}, value_type{4}}),
                     vector_type::from_row_major({value_type{0.25}, value_type{0.5}, value_type{1}}),
                     vector_type::from_row_major({value_type{0.125}, value_type{0.25}, value_type{0.5}}),
                     vector_type::from_row_major({value_type{0.03125}, value_type{0.0625}, value_type{0.125}})};
    covariance_type covariance = covariance_type::identity();
};

template <typename Matrix> bool same_bits(Matrix const & a, Matrix const & b)
{
    using bytes_type = std::array<std::byte, sizeof(typename Matrix::value_type)>;
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            if (std::bit_cast<bytes_type>(a(row, column)) != std::bit_cast<bytes_type>(b(row, column)))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename Noise> bool same_noise(Noise const & a, Noise const & b)
{
    return same_bits(a.specific_force_variance, b.specific_force_variance) &&
           same_bits(a.angular_rate_variance, b.angular_rate_variance) &&
           same_bits(a.accelerometer_bias_random_walk_variance_density,
                     b.accelerometer_bias_random_walk_variance_density) &&
           same_bits(a.gyroscope_bias_random_walk_variance_density, b.gyroscope_bias_random_walk_variance_density);
}

template <typename Linalg> void test_selection(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    for (unsigned mask = 0U; mask < 64U; ++mask)
    {
        BiasProcessNoiseFixture<Linalg> fixture;
        auto expected = fixture.noise;
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            bool const accel_inhibit = (mask & (1U << axis)) != 0U;
            bool const gyro_inhibit = (mask & (1U << (3U + axis))) != 0U;
            fixture.covariance(9U + axis, 9U + axis) =
                fixture.noise.specific_force_variance(axis) * (accel_inhibit ? Scalar{2} : Scalar{0.5});
            fixture.covariance(12U + axis, 12U + axis) =
                fixture.noise.angular_rate_variance(axis) * (gyro_inhibit ? Scalar{2} : Scalar{0.5});
            if (accel_inhibit)
            {
                expected.accelerometer_bias_random_walk_variance_density(axis) = Scalar{0};
            }
            if (gyro_inhibit)
            {
                expected.gyroscope_bias_random_walk_variance_density(axis) = Scalar{0};
            }
        }
        auto selected = fixture.noise;
        auto const original = fixture.noise;
        test.expect(formal_eskf::runtime::inhibit_bias_process_noise(fixture.covariance, fixture.noise, selected) ==
                            Status::success &&
                        same_noise(selected, expected) && same_noise(fixture.noise, original),
                    profile, "all 64 independent axis choices preserve measurement variances and configured noise");
        test.expect(formal_eskf::runtime::inhibit_bias_process_noise(fixture.covariance, fixture.noise,
                                                                     fixture.noise) == Status::success &&
                        same_noise(fixture.noise, expected),
                    profile, "whole-noise output alias is supported");
    }
    BiasProcessNoiseFixture<Linalg> fixture;
    for (int side : {-1, 0, 1})
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            auto const a = fixture.noise.specific_force_variance(axis);
            auto const g = fixture.noise.angular_rate_variance(axis);
            auto const target = side < 0 ? Scalar{0} : std::numeric_limits<Scalar>::infinity();
            fixture.covariance(9U + axis, 9U + axis) = side == 0 ? a : std::nextafter(a, target);
            fixture.covariance(12U + axis, 12U + axis) = side == 0 ? g : std::nextafter(g, target);
        }
        auto expected = fixture.noise;
        if (side >= 0)
        {
            expected.accelerometer_bias_random_walk_variance_density = {};
            expected.gyroscope_bias_random_walk_variance_density = {};
        }
        auto selected = fixture.noise;
        test.expect(formal_eskf::runtime::inhibit_bias_process_noise(fixture.covariance, fixture.noise, selected) ==
                            Status::success &&
                        same_noise(selected, expected),
                    profile, "one ULP below selects; exact equality and one ULP above inhibit");
    }
    fixture.noise = {};
    fixture.covariance = {};
    auto selected = fixture.noise;
    test.expect(formal_eskf::runtime::inhibit_bias_process_noise(fixture.covariance, fixture.noise, selected) ==
                        Status::success &&
                    same_noise(selected, fixture.noise),
                profile, "zero variances and zero intensities are valid");
    fixture.noise.accelerometer_bias_random_walk_variance_density(0U) = Scalar{1};
    fixture.noise.gyroscope_bias_random_walk_variance_density(2U) = Scalar{2};
    test.expect(formal_eskf::runtime::inhibit_bias_process_noise(fixture.covariance, fixture.noise, selected) ==
                        Status::success &&
                    selected.accelerometer_bias_random_walk_variance_density(0U) == Scalar{0} &&
                    selected.gyroscope_bias_random_walk_variance_density(2U) == Scalar{0},
                profile,
                "zero variance equality inhibits nonzero intensities, rather than merely preserving zero input");
}

template <typename Linalg> void test_failures(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    for (auto invalid : {Scalar{-1}, std::numeric_limits<Scalar>::infinity(), -std::numeric_limits<Scalar>::infinity(),
                         std::numeric_limits<Scalar>::quiet_NaN()})
    {
        for (std::size_t field = 0U; field < 6U; ++field)
        {
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                BiasProcessNoiseFixture<Linalg> fixture;
                auto output = fixture.noise;
                auto const before = output;
                if (field < 2U)
                {
                    auto const index = 9U + 3U * field + axis;
                    fixture.covariance(index, index) = invalid;
                }
                else
                {
                    std::array<typename BiasProcessNoiseFixture<Linalg>::vector_type *, 4U> const groups{
                        &fixture.noise.specific_force_variance, &fixture.noise.angular_rate_variance,
                        &fixture.noise.accelerometer_bias_random_walk_variance_density,
                        &fixture.noise.gyroscope_bias_random_walk_variance_density};
                    (*groups[field - 2U])(axis) = invalid;
                }
                auto const expected = std::isfinite(invalid) ? Status::domain_error : Status::non_finite_input;
                auto const covariance_before = fixture.covariance;
                auto const input_before = fixture.noise;
                test.expect(formal_eskf::runtime::inhibit_bias_process_noise(fixture.covariance, fixture.noise,
                                                                             output) == expected &&
                                same_noise(output, before) && same_noise(fixture.noise, input_before) &&
                                same_bits(fixture.covariance, covariance_before),
                            profile, "invalid consumed input preserves every input and output coefficient");
                test.expect(formal_eskf::runtime::inhibit_bias_process_noise(fixture.covariance, fixture.noise,
                                                                             fixture.noise) == expected &&
                                same_noise(fixture.noise, input_before) &&
                                same_bits(fixture.covariance, covariance_before),
                            profile, "failure is atomic even with an aliased invalid noise input");
            }
        }
    }
    for (auto unused : {Scalar{-1}, std::numeric_limits<Scalar>::infinity(), std::numeric_limits<Scalar>::quiet_NaN()})
    {
        BiasProcessNoiseFixture<Linalg> fixture;
        for (std::size_t row = 0; row < 15; ++row)
        {
            for (std::size_t column = 0; column < 15; ++column)
            {
                if (row != column || row < 9)
                {
                    fixture.covariance(row, column) = unused;
                }
            }
        }
        auto const prior = fixture.covariance;
        auto expected = fixture.noise;
        expected.accelerometer_bias_random_walk_variance_density(0U) = Scalar{0};
        expected.gyroscope_bias_random_walk_variance_density = {};
        auto output = fixture.noise;
        test.expect(formal_eskf::runtime::inhibit_bias_process_noise(fixture.covariance, fixture.noise, output) ==
                            Status::success &&
                        same_noise(output, expected) && same_bits(fixture.covariance, prior),
                    profile,
                    "only six bias diagonals consumed; arbitrary unused P coefficients stay unvalidated and unchanged");
    }
}

template <typename Linalg> void test_prediction(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    using Ins = formal_eskf::configuration::Ins;
    using Matrix2 = formal_eskf::linalg::Matrix<Linalg, 2U, 2U>;
    using Vector2 = formal_eskf::linalg::Vector<Linalg, 2U>;
    BiasProcessNoiseFixture<Linalg> fixture;
    Ins::NominalState<Linalg> state;
    formal_eskf::ImuSample<Linalg> imu;
    Ins::Parameters<Linalg> parameters;
    parameters.minimum_quaternion_norm = static_cast<Scalar>(1e-6);
    parameters.quaternion_squared_norm_tolerance = static_cast<Scalar>(1e-4);
    parameters.dt_min = static_cast<Scalar>(1e-6);
    parameters.dt_max = Scalar{1};
    // Correlated, positive-definite velocity/bias block. No other state is
    // correlated to it, so a velocity correction can release the inhibited axis.
    fixture.covariance(3U, 3U) = Scalar{2};
    fixture.covariance(9U, 9U) = Scalar{1.5};
    fixture.covariance(3U, 9U) = Scalar{1.5};
    fixture.covariance(9U, 3U) = Scalar{1.5};
    test.expect(formal_eskf::runtime::inhibit_bias_process_noise(fixture.covariance, fixture.noise,
                                                                 parameters.process_noise) == Status::success,
                profile, "select noise before prediction");
    auto predicted = fixture.covariance;
    test.expect(formal_eskf::predict_covariance(state, fixture.covariance, imu, Scalar{0.125}, parameters, predicted) ==
                        Status::success &&
                    predicted(9U, 9U) == Scalar{1.5},
                profile, "inhibited variance above threshold is retained, not clamped; cross-covariance is allowed");
    auto corrected = fixture.covariance;
    test.expect(formal_eskf::correct_horizontal_velocity(
                    state, fixture.covariance, Vector2{}, Matrix2::identity() * Scalar{2},
                    parameters.minimum_quaternion_norm, state, corrected) == Status::success &&
                    corrected(9U, 9U) == Scalar{0.9375},
                profile, "exact scalar Joseph result 3/2-(3/2)^2/4=15/16 releases bias uncertainty");
    test.expect(formal_eskf::runtime::inhibit_bias_process_noise(corrected, fixture.noise, parameters.process_noise) ==
                        Status::success &&
                    parameters.process_noise.accelerometer_bias_random_walk_variance_density(0U) == Scalar{0.125},
                profile, "selection from original configuration restores noise after covariance decreases");
    for (auto dt : {Scalar{0.125}, Scalar{0.25}})
    {
        test.expect(
            formal_eskf::predict_covariance(state, corrected, imu, dt, parameters, predicted) == Status::success &&
                predicted(9U, 9U) == corrected(9U, 9U) + Scalar{0.125} * dt,
            profile, "actual prediction retains intensity times dt, not a squared dt or double-squared variance");
    }
    // One increment may cross the threshold; the next step suppresses its noise.
    fixture.covariance(9U, 9U) = Scalar{0.984375};
    fixture.covariance(3U, 9U) = Scalar{0};
    fixture.covariance(9U, 3U) = Scalar{0};
    test.expect(formal_eskf::runtime::inhibit_bias_process_noise(fixture.covariance, fixture.noise,
                                                                 parameters.process_noise) == Status::success &&
                    formal_eskf::predict_covariance(state, fixture.covariance, imu, Scalar{0.25}, parameters,
                                                    predicted) == Status::success &&
                    predicted(9U, 9U) == Scalar{1.015625},
                profile, "pre-increment threshold permits one-step overshoot");
    test.expect(formal_eskf::runtime::inhibit_bias_process_noise(predicted, fixture.noise, parameters.process_noise) ==
                        Status::success &&
                    parameters.process_noise.accelerometer_bias_random_walk_variance_density(0U) == Scalar{0},
                profile, "overshoot inhibits the next increment without clipping covariance");
}

template <typename Linalg> void ordered_controls(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    using Ins = formal_eskf::configuration::Ins;
    using Health = formal_eskf::runtime::VerticalAccelerationHealth<Linalg>;
    using Policy = formal_eskf::runtime::ImuBiasInhibition<Linalg>;
    BiasProcessNoiseFixture<Linalg> fixture;
    auto P = fixture.covariance * Scalar{2};
    Ins::NominalState<Linalg> state;
    test.expect(decltype(state.q_nb)::from_coefficients(Scalar{0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.5},
                                                        Scalar{0.125}, state.q_nb) == Status::success,
                profile, "exact cyclic body-to-navigation rotation");
    Health health;
    Policy policy({Scalar{25}, Scalar{3}, Scalar{0.5}, true, true});
    Ins::Parameters<Linalg> parameters;
    parameters.minimum_quaternion_norm = Scalar{0.125};
    parameters.quaternion_squared_norm_tolerance = Scalar{0.125};
    parameters.dt_min = Scalar{0.125};
    parameters.dt_max = Scalar{0.125};
    Scalar const dt = Scalar{0.125};
    std::array<std::size_t, 3> const body_axis{2U, 0U, 1U};
    for (unsigned step = 0; step < 3; ++step)
    {
        bool const prior_fault = health.state().bad_vertical_acceleration;
        test.expect(prior_fault == (step == 1), profile, "only preceding health verdict selects current noise");
        formal_eskf::runtime::ImuNoiseConditions conditions{prior_fault, {step == 0, false, false}};
        auto expected = fixture.noise;
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            if (prior_fault || conditions.accelerometer_clipped[axis])
            {
                expected.specific_force_variance(axis) = Scalar{16};
            }
            if (!(P(9U + axis, 9U + axis) < expected.specific_force_variance(axis)))
            {
                expected.accelerometer_bias_random_walk_variance_density(axis) = Scalar{0};
            }
            if (!(P(12U + axis, 12U + axis) < expected.angular_rate_variance(axis)))
            {
                expected.gyroscope_bias_random_walk_variance_density(axis) = Scalar{0};
            }
        }
        test.expect(formal_eskf::runtime::select_imu_process_noise(fixture.noise, conditions, Scalar{16},
                                                                   parameters.process_noise) == Status::success &&
                        formal_eskf::runtime::inhibit_bias_process_noise(P, parameters.process_noise,
                                                                         parameters.process_noise) == Status::success &&
                        same_noise(parameters.process_noise, expected),
                    profile, "measurement variance selection precedes bias threshold and starts from configured noise");
        if (step == 0)
        {
            test.expect(expected.accelerometer_bias_random_walk_variance_density(0U) == Scalar{0.125}, profile,
                        "clipping-raised variance releases X driving noise that original variance would inhibit");
        }
        auto predicted = P;
        test.expect(formal_eskf::predict_covariance(state, P, {}, dt, parameters, predicted) == Status::success,
                    profile, "actual predictor consumes selected process noise");
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            auto const k = body_axis[axis];
            Scalar const velocity_variance = P(3U + axis, 3U + axis) - Scalar{2} * dt * P(3U + axis, 9U + k) +
                                             dt * dt * P(9U + k, 9U + k) +
                                             dt * dt * expected.specific_force_variance(k);
            test.expect(
                predicted(3U + axis, 3U + axis) == velocity_variance &&
                    predicted(9U + axis, 9U + axis) ==
                        P(9U + axis, 9U + axis) + dt * expected.accelerometer_bias_random_walk_variance_density(axis) &&
                    predicted(12U + axis, 12U + axis) ==
                        P(12U + axis, 12U + axis) + dt * expected.gyroscope_bias_random_walk_variance_density(axis),
                profile,
                "independent exact cyclic covariance oracle applies one rotation, dt squared, and intensity times dt");
        }
        P = predicted;
        typename Policy::Conditions learning{true, false, false, prior_fault, step == 0 ? 1 : 0};
        auto const permissions = policy.update(Scalar{0.125}, Scalar{0.25}, state, {}, learning);
        test.expect(permissions.status == Status::success &&
                        permissions.bias_update.gyroscope == std::array<bool, 3>{true, true, true},
                    profile, "policy receives distinct real sensor periods and preceding fault");
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            test.expect(permissions.bias_update.accelerometer[axis] == (!prior_fault && !(step == 0 && axis == 0)),
                        profile, "learning permission is independent of whether driving-noise intensity was selected");
        }
        if (step == 0)
        {
            typename Health::Observations observations;
            observations.barometer_height = {true, Scalar{4}, Scalar{1}};
            observations.gnss_velocity_down = {true, Scalar{2}, Scalar{1}};
            test.expect(health.update({1000U, Scalar{0.125}, true, {}}, observations) == Status::success &&
                            health.state().bad_vertical_acceleration,
                        profile, "new independent falling evidence applies to next prediction, not the completed one");
        }
        else if (step == 1)
        {
            test.expect(health.update({2000U, Scalar{0.125}, true, {}}, {}) == Status::success &&
                            health.state().last_good_time_us == 2000U && health.state().bad_vertical_acceleration,
                        profile, "successful current good evidence updates last-good during held fault");
            test.expect(health.update({3001000U, Scalar{0.125}, true, {}}, {}) == Status::success &&
                            !health.state().bad_vertical_acceleration,
                        profile,
                        "three-second equality releases following-step selection without clearing configured noise");
        }
    }
}

template <typename Linalg> void independent_protections(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    using Ins = formal_eskf::configuration::Ins;
    using Policy = formal_eskf::runtime::ImuBiasInhibition<Linalg>;
    using Vector = formal_eskf::linalg::Vector<Linalg, 3U>;
    using Matrix3 = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    BiasProcessNoiseFixture<Linalg> fixture;
    Ins::NominalState<Linalg> state;
    state.b_a(0U) = Scalar{0.5};
    formal_eskf::ImuSample<Linalg> imu;
    imu.specific_force_b = state.b_a;
    Policy policy({Scalar{25}, Scalar{3}, Scalar{0.5}, true, true});
    auto permissions = policy.update(Scalar{0.125}, Scalar{0.25}, state, imu, {true, false, false, true, 0});
    test.expect(permissions.status == Status::success &&
                    permissions.bias_update.accelerometer == std::array<bool, 3>{false, false, false} &&
                    state.b_a(0U) == Scalar{0.5},
                profile, "held fault disables learning without erasing estimated bias");
    fixture.covariance(9U, 9U) = Scalar{0.5};
    auto selected = fixture.noise;
    test.expect(formal_eskf::runtime::inhibit_bias_process_noise(fixture.covariance, fixture.noise, selected) ==
                        Status::success &&
                    selected.accelerometer_bias_random_walk_variance_density(0U) == Scalar{0.125},
                profile, "disabled correction learning does not disable covariance-driving noise");
    Ins::Parameters<Linalg> parameters;
    parameters.minimum_quaternion_norm = Scalar{0.125};
    parameters.quaternion_squared_norm_tolerance = Scalar{0.125};
    parameters.dt_min = Scalar{0.125};
    parameters.dt_max = Scalar{0.125};
    parameters.process_noise = selected;
    auto predicted = fixture.covariance;
    test.expect(formal_eskf::predict_covariance(state, fixture.covariance, imu, Scalar{0.125}, parameters, predicted) ==
                        Status::success &&
                    predicted(9U, 9U) == Scalar{0.515625} && state.b_a(0U) == Scalar{0.5},
                profile, "nonzero bias held while its selected uncertainty grows by intensity times dt");
    predicted(3U, 9U) = Scalar{0.25};
    predicted(9U, 3U) = Scalar{0.25};
    Vector accepted_velocity;
    accepted_velocity(0U) = Scalar{0.5};
    auto const old_bias = state.b_a;
    test.expect(formal_eskf::correct_velocity(state, predicted, accepted_velocity, Matrix3::identity(), Scalar{0.125},
                                              state, predicted, permissions.bias_update) == Status::success &&
                    same_bits(state.b_a, old_bias) && predicted(9U, 9U) == Scalar{0.515625},
                profile, "actual correction holds inhibited bias gain rows even while driving noise was enabled");
    auto const before_limit_P = predicted;
    test.expect(formal_eskf::runtime::limit_bias<Linalg>(state, {Scalar{0.125}, Scalar{0.125}}) == Status::success &&
                    state.b_a(0U) == Scalar{0.125} && same_bits(predicted, before_limit_P),
                profile, "explicit limiting changes out-of-range bias despite inhibition, without covariance repair");
    imu.specific_force_b = state.b_a;
    auto invalid = imu;
    invalid.specific_force_b(2U) = std::numeric_limits<Scalar>::quiet_NaN();
    auto const failed = policy.update(Scalar{0.125}, Scalar{0.25}, state, invalid, {});
    test.expect(failed.status == Status::non_finite_input &&
                    failed.bias_update.accelerometer == std::array<bool, 3>{false, false, false},
                profile, "failed policy result never represents successful permissions");
    permissions = policy.update(Scalar{0.125}, Scalar{0.25}, state, imu, {true, false, false, false, 0});
    test.expect(permissions.status == Status::success &&
                    permissions.bias_update.accelerometer == std::array<bool, 3>{true, true, true},
                profile, "valid health and actual sample recover permission without extra timer");
    // Independent valid correction prior: S=4I, K_ba,x=1/16 and residual=-1/2.
    auto P = decltype(fixture.covariance)::identity() * Scalar{2};
    P(3U, 9U) = Scalar{0.25};
    P(9U, 3U) = Scalar{0.25};
    Vector measurement = state.v_n;
    measurement(0U) -= Scalar{0.5};
    auto const V = Matrix3::identity() * Scalar{2};
    test.expect(formal_eskf::correct_velocity(state, P, measurement, V, Scalar{0.125}, state, P,
                                              permissions.bias_update) == Status::success &&
                    state.b_a(0U) == Scalar{0.09375},
                profile, "recovered learning makes independent exact inward bias correction");
    auto const corrected_P = P;
    test.expect(formal_eskf::runtime::limit_bias<Linalg>(state, {Scalar{0.125}, Scalar{0.125}}) == Status::success &&
                    state.b_a(0U) == Scalar{0.09375} && same_bits(P, corrected_P),
                profile, "subsequent limit preserves inward correction and posterior covariance");
}

} /* end namespace */
int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Case;
    using Float = formal_eskf::test::Backend<float>;
    using Double = formal_eskf::test::Backend<double>;
    Case const cases[] = {
        {"TC-R-BIAS-PROCESS-NOISE-01", "float", test_selection<Float>},
        {"TC-R-BIAS-PROCESS-NOISE-01", "double", test_selection<Double>},
        {"TC-R-BIAS-PROCESS-NOISE-02", "float", test_failures<Float>},
        {"TC-R-BIAS-PROCESS-NOISE-02", "double", test_failures<Double>},
        {"TC-R-BIAS-PROCESS-NOISE-03", "float", test_prediction<Float>},
        {"TC-R-BIAS-PROCESS-NOISE-03", "double", test_prediction<Double>},
        {"TC-SW-IMU-01", "float", ordered_controls<Float>},
        {"TC-SW-IMU-01", "double", ordered_controls<Double>},
        {"TC-SW-IMU-02", "float", independent_protections<Float>},
        {"TC-SW-IMU-02", "double", independent_protections<Double>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
