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
#include <formal_eskf/runtime/bias_process_noise.hpp>
#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{

using formal_eskf::Status;
using TestContext = formal_eskf::test::Context;
constexpr std::string_view profile = "bias process noise";

template <typename Linalg> struct BiasProcessNoiseFixture
{
    using value_type = typename Linalg::value_type;
    using vector_type = typename Linalg::template vector_type<3U>;
    using noise_type = formal_eskf::configuration::Ins::ProcessNoise<Linalg>;
    using covariance_type = typename Linalg::template matrix_type<15U, 15U>;
    noise_type noise{vector_type::from_row_major({value_type{1}, value_type{2}, value_type{4}}),
                     vector_type::from_row_major({value_type{0.25}, value_type{0.5}, value_type{1}}),
                     vector_type::from_row_major({value_type{0.125}, value_type{0.25}, value_type{0.5}}),
                     vector_type::from_row_major({value_type{0.03125}, value_type{0.0625}, value_type{0.125}})};
    covariance_type covariance = covariance_type::identity();
};

template <typename Matrix> bool same_bits(Matrix const & a, Matrix const & b)
{
    using bytes_type = std::array<std::byte, sizeof(typename Matrix::value_type)>;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        if (std::bit_cast<bytes_type>(a(axis)) != std::bit_cast<bytes_type>(b(axis)))
        {
            return false;
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

template <typename Linalg> void test_selection(TestContext & test)
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
            fixture.covariance.set(9U + axis, 9U + axis,
                                   fixture.noise.specific_force_variance(axis) *
                                       (accel_inhibit ? Scalar{2} : Scalar{0.5}));
            fixture.covariance.set(12U + axis, 12U + axis,
                                   fixture.noise.angular_rate_variance(axis) *
                                       (gyro_inhibit ? Scalar{2} : Scalar{0.5}));
            if (accel_inhibit)
            {
                expected.accelerometer_bias_random_walk_variance_density.set(axis, Scalar{0});
            }
            if (gyro_inhibit)
            {
                expected.gyroscope_bias_random_walk_variance_density.set(axis, Scalar{0});
            }
        }
        auto selected = fixture.noise;
        auto const original = fixture.noise;
        test.expect(formal_eskf::runtime::try_inhibit_bias_process_noise(fixture.covariance, fixture.noise, selected) ==
                            Status::success &&
                        same_noise(selected, expected) && same_noise(fixture.noise, original),
                    profile, "all 64 independent axis choices preserve measurement variances and configured noise");
        test.expect(formal_eskf::runtime::try_inhibit_bias_process_noise(fixture.covariance, fixture.noise,
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
            fixture.covariance.set(9U + axis, 9U + axis, side == 0 ? a : std::nextafter(a, target));
            fixture.covariance.set(12U + axis, 12U + axis, side == 0 ? g : std::nextafter(g, target));
        }
        auto expected = fixture.noise;
        if (side >= 0)
        {
            expected.accelerometer_bias_random_walk_variance_density = {};
            expected.gyroscope_bias_random_walk_variance_density = {};
        }
        auto selected = fixture.noise;
        test.expect(formal_eskf::runtime::try_inhibit_bias_process_noise(fixture.covariance, fixture.noise, selected) ==
                            Status::success &&
                        same_noise(selected, expected),
                    profile, "one ULP below selects; exact equality and one ULP above inhibit");
    }
    fixture.noise = {};
    fixture.covariance = {};
    auto selected = fixture.noise;
    test.expect(formal_eskf::runtime::try_inhibit_bias_process_noise(fixture.covariance, fixture.noise, selected) ==
                        Status::success &&
                    same_noise(selected, fixture.noise),
                profile, "zero variances and zero intensities are valid");
}

template <typename Linalg> void test_failures(TestContext & test)
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
                    fixture.covariance.set(index, index, invalid);
                }
                else
                {
                    std::array<typename BiasProcessNoiseFixture<Linalg>::vector_type *, 4U> const groups{
                        &fixture.noise.specific_force_variance, &fixture.noise.angular_rate_variance,
                        &fixture.noise.accelerometer_bias_random_walk_variance_density,
                        &fixture.noise.gyroscope_bias_random_walk_variance_density};
                    groups[field - 2U]->set(axis, invalid);
                }
                auto const expected = std::isfinite(invalid) ? Status::domain_error : Status::non_finite_input;
                test.expect(formal_eskf::runtime::try_inhibit_bias_process_noise(fixture.covariance, fixture.noise,
                                                                                 output) == expected &&
                                same_noise(output, before),
                            profile, "invalid consumed input preserves every output coefficient");
                auto const input_before = fixture.noise;
                test.expect(formal_eskf::runtime::try_inhibit_bias_process_noise(fixture.covariance, fixture.noise,
                                                                                 fixture.noise) == expected &&
                                same_noise(fixture.noise, input_before),
                            profile, "failure is atomic even with an aliased invalid noise input");
            }
        }
    }
}

template <typename Linalg> void test_prediction(TestContext & test)
{
    using Scalar = typename Linalg::value_type;
    using Ins = formal_eskf::configuration::Ins;
    using Matrix2 = typename Linalg::template matrix_type<2U, 2U>;
    using Vector2 = typename Linalg::template vector_type<2U>;
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
    fixture.covariance.set(3U, 3U, Scalar{2});
    fixture.covariance.set(9U, 9U, Scalar{1.5});
    fixture.covariance.set(3U, 9U, Scalar{1.5});
    fixture.covariance.set(9U, 3U, Scalar{1.5});
    test.expect(formal_eskf::runtime::try_inhibit_bias_process_noise(fixture.covariance, fixture.noise,
                                                                     parameters.process_noise) == Status::success,
                profile, "select noise before prediction");
    auto predicted = fixture.covariance;
    test.expect(formal_eskf::try_predict_covariance(state, fixture.covariance, imu, Scalar{0.125}, parameters,
                                                    predicted) == Status::success &&
                    predicted(9U, 9U) == Scalar{1.5},
                profile, "inhibited variance above threshold is retained, not clamped; cross-covariance is allowed");
    auto corrected = fixture.covariance;
    test.expect(formal_eskf::try_correct_horizontal_velocity(
                    state, fixture.covariance, Vector2{}, Matrix2::identity() * Scalar{0.25},
                    parameters.minimum_quaternion_norm, state, corrected) == Status::success &&
                    corrected(9U, 9U) < Scalar{1},
                profile, "real velocity correction reduces correlated bias uncertainty below its threshold");
    test.expect(formal_eskf::runtime::try_inhibit_bias_process_noise(corrected, fixture.noise,
                                                                     parameters.process_noise) == Status::success &&
                    parameters.process_noise.accelerometer_bias_random_walk_variance_density(0U) == Scalar{0.125},
                profile, "selection from original configuration restores noise after covariance decreases");
    for (auto dt : {Scalar{0.125}, Scalar{0.25}})
    {
        test.expect(
            formal_eskf::try_predict_covariance(state, corrected, imu, dt, parameters, predicted) == Status::success &&
                predicted(9U, 9U) == corrected(9U, 9U) + Scalar{0.125} * dt,
            profile, "actual prediction retains intensity times dt, not a squared dt or double-squared variance");
    }
    // One increment may cross the threshold; the next step suppresses its noise.
    fixture.covariance.set(9U, 9U, Scalar{0.984375});
    fixture.covariance.set(3U, 9U, Scalar{0});
    fixture.covariance.set(9U, 3U, Scalar{0});
    test.expect(formal_eskf::runtime::try_inhibit_bias_process_noise(fixture.covariance, fixture.noise,
                                                                     parameters.process_noise) == Status::success &&
                    formal_eskf::try_predict_covariance(state, fixture.covariance, imu, Scalar{0.25}, parameters,
                                                        predicted) == Status::success &&
                    predicted(9U, 9U) == Scalar{1.015625},
                profile, "PX4-style pre-increment threshold permits one-step overshoot");
    test.expect(formal_eskf::runtime::try_inhibit_bias_process_noise(predicted, fixture.noise,
                                                                     parameters.process_noise) == Status::success &&
                    parameters.process_noise.accelerometer_bias_random_walk_variance_density(0U) == Scalar{0},
                profile, "overshoot inhibits the next increment without clipping covariance");
}

} /* end namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    TestContext test;
    test_selection<formal_eskf::test::Backend<float>>(test);
    test_selection<formal_eskf::test::Backend<double>>(test);
    test_failures<formal_eskf::test::Backend<float>>(test);
    test_failures<formal_eskf::test::Backend<double>>(test);
    test_prediction<formal_eskf::test::Backend<float>>(test);
    test_prediction<formal_eskf::test::Backend<double>>(test);
    if (test.failures() != 0)
    {
        return 1;
    }
    std::cout << "Bias process noise: pass\n";
    return 0;
}
