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
#include <formal_eskf/runtime/imu.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{

using formal_eskf::Status;
using TestContext = formal_eskf::test::Context;
using Conditions = formal_eskf::runtime::ImuNoiseConditions;
using Ins = formal_eskf::configuration::Ins;

template <typename Noise> auto noise_vectors(Noise & noise)
{
    return std::array{&noise.specific_force_variance, &noise.angular_rate_variance,
                      &noise.accelerometer_bias_random_walk_variance_density,
                      &noise.gyroscope_bias_random_walk_variance_density};
}

template <typename Noise> bool same_noise(Noise const & a, Noise const & b)
{
    using bytes_type = std::array<std::byte, sizeof(typename Noise::vector3_type::value_type)>;
    auto const a_vectors = noise_vectors(a);
    auto const b_vectors = noise_vectors(b);
    for (std::size_t field = 0U; field < a_vectors.size(); ++field)
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            if (std::bit_cast<bytes_type>((*a_vectors[field])(axis)) !=
                std::bit_cast<bytes_type>((*b_vectors[field])(axis)))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename Linalg> Ins::ProcessNoise<Linalg> configured_noise()
{
    using Scalar = typename Linalg::value_type;
    using Vector = formal_eskf::linalg::Vector<Linalg, 3U>;
    return {Vector::from_row_major({Scalar{1}, Scalar{32}, Scalar{0}}),
            Vector::from_row_major({Scalar{0.25}, Scalar{0.5}, Scalar{1}}),
            Vector::from_row_major({Scalar{0.125}, Scalar{0.25}, Scalar{0.5}}),
            Vector::from_row_major({Scalar{0.03125}, Scalar{0.0625}, Scalar{0.125}})};
}

template <typename Linalg> void test_selection(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    auto const configured = configured_noise<Linalg>();
    for (bool fault : {false, true})
    {
        for (unsigned mask = 0U; mask < 8U; ++mask)
        {
            Conditions conditions;
            conditions.bad_vertical_acceleration = fault;
            auto expected = configured;
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                conditions.accelerometer_clipped[axis] = (mask & (1U << axis)) != 0U;
                if (fault || conditions.accelerometer_clipped[axis])
                {
                    expected.specific_force_variance(axis) = Scalar{16};
                }
            }
            auto output = configured;
            test.expect(formal_eskf::runtime::select_imu_process_noise(configured, conditions, Scalar{16}, output) ==
                                Status::success &&
                            same_noise(output, expected),
                        profile,
                        "all 16 fault/clipping combinations; replace even a larger baseline; preserve other fields");
            auto alias = configured;
            test.expect(formal_eskf::runtime::select_imu_process_noise(alias, conditions, Scalar{16}, alias) ==
                                Status::success &&
                            same_noise(alias, expected),
                        profile, "whole-noise alias produces the same selection");
            test.expect(formal_eskf::runtime::select_imu_process_noise(configured, {}, Scalar{16}, output) ==
                                Status::success &&
                            same_noise(output, configured),
                        profile, "clear conditions restore configured variances immediately");
        }
    }
    for (auto variance : {Scalar{0}, std::numeric_limits<Scalar>::max()})
    {
        auto output = configured;
        auto expected = configured;
        expected.specific_force_variance(0U) = variance;
        test.expect(formal_eskf::runtime::select_imu_process_noise(configured, {false, {true, false, false}}, variance,
                                                                   output) == Status::success &&
                        same_noise(output, expected),
                    profile, "finite nonnegative variance is copied without squaring or overflow");
    }
}

template <typename Linalg> void test_failures(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    for (auto invalid : {Scalar{-1}, std::numeric_limits<Scalar>::infinity(), -std::numeric_limits<Scalar>::infinity(),
                         std::numeric_limits<Scalar>::quiet_NaN()})
    {
        auto const status = std::isfinite(invalid) ? Status::domain_error : Status::non_finite_input;
        for (bool fault : {false, true})
        {
            Conditions const conditions{fault, {false, false, false}};
            auto noise = configured_noise<Linalg>();
            auto output = noise;
            auto const before = output;
            test.expect(formal_eskf::runtime::select_imu_process_noise(noise, conditions, invalid, output) == status &&
                            same_noise(output, before),
                        profile, "invalid fault variance fails even when unselected and preserves output");
            test.expect(formal_eskf::runtime::select_imu_process_noise(noise, conditions, invalid, noise) == status &&
                            same_noise(noise, before),
                        profile, "invalid parameter preserves aliased noise");
            for (std::size_t field = 0U; field < 4U; ++field)
            {
                for (std::size_t axis = 0U; axis < 3U; ++axis)
                {
                    noise = configured_noise<Linalg>();
                    (*noise_vectors(noise)[field])(axis) = invalid;
                    auto const original = noise;
                    output = before;
                    test.expect(formal_eskf::runtime::select_imu_process_noise(noise, conditions, Scalar{16}, output) ==
                                        status &&
                                    same_noise(output, before) && same_noise(noise, original),
                                profile,
                                "invalid configuration cannot be hidden by replacement; inputs and output preserved");
                    test.expect(formal_eskf::runtime::select_imu_process_noise(noise, conditions, Scalar{16}, noise) ==
                                        status &&
                                    same_noise(noise, original),
                                profile, "invalid aliased input is preserved coefficient by coefficient");
                }
            }
        }
    }
    // Fault parameter validation precedes every configured coefficient.
    for (bool alias : {false, true})
    {
        auto noise = configured_noise<Linalg>();
        noise.specific_force_variance(0U) = std::numeric_limits<Scalar>::quiet_NaN();
        auto const original = noise;
        auto separate = configured_noise<Linalg>();
        separate.angular_rate_variance(2U) = -Scalar{0};
        auto const old_separate = separate;
        auto & output = alias ? noise : separate;
        test.expect(formal_eskf::runtime::select_imu_process_noise(noise, {true, {}}, Scalar{-1}, output) ==
                            Status::domain_error &&
                        same_noise(noise, original) && same_noise(separate, old_separate),
                    profile, "negative fault variance wins before nonfinite configuration, without publication");
        noise = configured_noise<Linalg>();
        noise.specific_force_variance(0U) = Scalar{-1};
        auto const negative_input = noise;
        test.expect(formal_eskf::runtime::select_imu_process_noise(noise, {}, std::numeric_limits<Scalar>::infinity(),
                                                                   output) == Status::non_finite_input &&
                        same_noise(noise, negative_input) && same_noise(separate, old_separate),
                    profile, "nonfinite fault variance wins before negative configuration");
    }
    auto configured = configured_noise<Linalg>();
    auto selected = configured;
    test.expect(formal_eskf::runtime::select_imu_process_noise(configured, {true, {}}, Scalar{16}, selected) ==
                    Status::success,
                profile, "select fault noise before recovery");
    test.expect(formal_eskf::runtime::select_imu_process_noise(configured, {}, Scalar{16}, selected) ==
                        Status::success &&
                    same_noise(configured, selected),
                profile, "restoration uses original configuration, without history or repeated scaling");
}

template <typename Linalg> void test_prediction(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    using Vector = formal_eskf::linalg::Vector<Linalg, 3U>;
    using Covariance = formal_eskf::linalg::Matrix<Linalg, 15U, 15U>;
    auto configured = configured_noise<Linalg>();
    configured.specific_force_variance = Vector::from_row_major({Scalar{1}, Scalar{2}, Scalar{4}});
    auto selected = configured;
    auto covariance = Covariance::identity() * Scalar{2};
    test.expect(formal_eskf::runtime::inhibit_bias_process_noise(covariance, configured, selected) == Status::success &&
                    selected.accelerometer_bias_random_walk_variance_density(0U) == Scalar{0},
                profile, "healthy variance suppresses above-threshold bias noise");
    test.expect(
        formal_eskf::runtime::select_imu_process_noise(configured, {false, {true, false, false}}, Scalar{16},
                                                       selected) == Status::success &&
            formal_eskf::runtime::inhibit_bias_process_noise(covariance, selected, selected) == Status::success &&
            selected.accelerometer_bias_random_walk_variance_density(0U) ==
                configured.accelerometer_bias_random_walk_variance_density(0U) &&
            selected.accelerometer_bias_random_walk_variance_density(1U) == Scalar{0} &&
            selected.gyroscope_bias_random_walk_variance_density(0U) == Scalar{0},
        profile, "bias threshold sees selected measurement variance; independent axes and gyro remain unchanged");

    Ins::NominalState<Linalg> state, output;
    // Exact cyclic body-to-navigation rotation: body X -> NED Y, Y -> Z, Z -> X.
    test.expect(decltype(state.q_nb)::from_coefficients(Scalar{0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.5},
                                                        Scalar{0.125}, state.q_nb) == Status::success,
                profile, "prepare rotated nominal state");
    Ins::Parameters<Linalg> parameters;
    parameters.minimum_quaternion_norm = Scalar{0.125};
    parameters.quaternion_squared_norm_tolerance = Scalar{0.125};
    parameters.dt_min = Scalar{0.125};
    parameters.dt_max = Scalar{0.125};
    parameters.process_noise.specific_force_variance = configured.specific_force_variance;
    test.expect(formal_eskf::runtime::select_imu_process_noise(parameters.process_noise, {false, {true, false, false}},
                                                               Scalar{16}, parameters.process_noise) == Status::success,
                profile, "select only body-X measurement noise");
    Covariance result;
    test.expect(formal_eskf::predict(state, Covariance{}, formal_eskf::ImuSample<Linalg>{}, Scalar{0.125}, parameters,
                                     output, result) == Status::success,
                profile, "actual covariance prediction consumes selected noise");
    std::array<Scalar, 3U> const expected{Scalar{4} / Scalar{64}, Scalar{16} / Scalar{64}, Scalar{2} / Scalar{64}};
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t col = 0U; col < 3U; ++col)
        {
            test.expect(result(3U + row, 3U + col) == (row == col ? expected[row] : Scalar{0}), profile,
                        "sampled variance scales by dt squared and rotates from body to NED");
        }
        test.expect(output.v_n(row) == state.v_n(row) && output.p_n(row) == state.p_n(row) &&
                        output.b_a(row) == state.b_a(row) && output.b_g(row) == state.b_g(row),
                    profile, "noise policy does not change nominal prediction");
    }
}

template <typename Linalg> void replacement(TestContext & test, std::string_view profile)
{
    test_selection<Linalg>(test, profile);
    // Supplementary exact rotated-prediction regression preserves earlier useful evidence.
    test_prediction<Linalg>(test, profile);
}

} /* end namespace */

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Case;
    using Float = formal_eskf::test::Backend<float>;
    using Double = formal_eskf::test::Backend<double>;
    Case const cases[] = {
        {"TC-R-IMU-PROCESS-NOISE-01", "float", replacement<Float>},
        {"TC-R-IMU-PROCESS-NOISE-01", "double", replacement<Double>},
        {"TC-R-IMU-PROCESS-NOISE-02", "float", test_failures<Float>},
        {"TC-R-IMU-PROCESS-NOISE-02", "double", test_failures<Double>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
