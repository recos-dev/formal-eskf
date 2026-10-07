/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/imu_dynamics.hpp>
#include <formal_eskf/runtime/imu_bias_learning.hpp>
#include <formal_eskf/eskf/velocity.hpp>

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <string_view>

#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{

using formal_eskf::Status;
using TestContext = formal_eskf::test::Context;

template <typename Linalg> struct ImuDynamicsFixture
{
    using value_type = typename Linalg::value_type;
    using vector_type = formal_eskf::linalg::Matrix<Linalg, 3U, 1U>;
    formal_eskf::ImuSample<Linalg> imu{};
    vector_type b_a{};
    vector_type b_g{};
    formal_eskf::runtime::ImuDynamicsParameters<value_type> parameters{value_type{4}, value_type{2}};

    [[nodiscard]] auto check() const noexcept
    {
        return formal_eskf::runtime::check_imu_dynamics(imu, b_a, b_g, parameters);
    }
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

template <typename Linalg> void test_magnitudes(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using vector_type = typename ImuDynamicsFixture<Linalg>::vector_type;
    ImuDynamicsFixture<Linalg> fixture;
    static_assert(noexcept(fixture.check()));
    fixture.b_a = vector_type::from_row_major({value_type{1}, value_type{-2}, value_type{0.5}});
    fixture.b_g = vector_type::from_row_major({value_type{0.125}, value_type{-0.25}, value_type{0.5}});
    fixture.imu.specific_force_b = vector_type::from_row_major({value_type{4}, value_type{2}, value_type{12.5}});
    fixture.imu.angular_rate_b = vector_type::from_row_major({value_type{0.875}, value_type{0.75}, value_type{0.5}});
    fixture.parameters = {value_type{13}, value_type{1.25}};
    auto const before = fixture;
    auto result = fixture.check();
    // Independent exact dyadic arithmetic: corrected f=(3,4,12), omega=(.75,1,0).
    test.expect(result.status == Status::success && !result.high_dynamics &&
                    result.specific_force_squared_norm == value_type{169} &&
                    result.angular_rate_squared_norm == value_type{1.5625},
                profile, "subtract both biases, use all three axes, and accept threshold equality");
    test.expect(same_bits(fixture.imu.specific_force_b, before.imu.specific_force_b) &&
                    same_bits(fixture.imu.angular_rate_b, before.imu.angular_rate_b) &&
                    same_bits(fixture.b_a, before.b_a) && same_bits(fixture.b_g, before.b_g) &&
                    fixture.parameters.maximum_specific_force == before.parameters.maximum_specific_force &&
                    fixture.parameters.maximum_angular_rate == before.parameters.maximum_angular_rate,
                profile, "classification preserves all inputs");
    fixture.parameters.maximum_specific_force = value_type{12.5};
    result = fixture.check();
    test.expect(result.status == Status::success && result.high_dynamics, profile,
                "force vector can exceed its limit even when every component is below it");
    fixture.parameters = {value_type{13}, value_type{1}};
    result = fixture.check();
    test.expect(result.status == Status::success && result.high_dynamics, profile,
                "angular-rate vector can exceed its limit without any component exceeding it");
    fixture.parameters = {value_type{12.5}, value_type{1}};
    result = fixture.check();
    test.expect(result.status == Status::success && result.high_dynamics &&
                    result.specific_force_squared_norm == value_type{169} &&
                    result.angular_rate_squared_norm == value_type{1.5625},
                profile, "both excessive magnitudes retain complete diagnostics");

    fixture.parameters = {value_type{13}, value_type{1.25}};
    fixture.imu.specific_force_b = vector_type::from_row_major({value_type{13}, value_type{-5}, value_type{4.5}});
    fixture.imu.angular_rate_b = vector_type::from_row_major({value_type{0.125}, value_type{-1}, value_type{1.5}});
    result = fixture.check();
    test.expect(result.status == Status::success && !result.high_dynamics &&
                    result.specific_force_squared_norm == value_type{169} &&
                    result.angular_rate_squared_norm == value_type{1.5625},
                profile, "coordinate permutation and sign changes preserve the magnitude decision");

    fixture = {};
    fixture.parameters = {value_type{25}, value_type{3}};
    fixture.imu.specific_force_b.set(2U, -static_cast<value_type>(9.80665));
    result = fixture.check();
    auto const gravity = static_cast<value_type>(9.80665);
    test.expect(result.status == Status::success && !result.high_dynamics &&
                    result.specific_force_squared_norm == gravity * gravity,
                profile, "stationary gravity response is not subtracted or normalized");
    fixture.imu.specific_force_b = vector_type::zero();
    result = fixture.check();
    test.expect(result.status == Status::success && !result.high_dynamics &&
                    result.specific_force_squared_norm == value_type{0},
                profile, "zero force passes the upper-magnitude check; this is not free-fall detection");
}

template <typename Linalg> void test_boundaries(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using limits_type = std::numeric_limits<value_type>;
    for (unsigned sensor = 0U; sensor < 2U; ++sensor)
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            for (int direction : {-1, 0, 1})
            {
                for (value_type sign : {value_type{-1}, value_type{1}})
                {
                    ImuDynamicsFixture<Linalg> fixture;
                    auto const limit = sensor == 0U ? fixture.parameters.maximum_specific_force
                                                    : fixture.parameters.maximum_angular_rate;
                    auto const magnitude =
                        direction == 0 ? limit
                                       : std::nextafter(limit, direction < 0 ? value_type{0} : limits_type::infinity());
                    (sensor == 0U ? fixture.imu.specific_force_b : fixture.imu.angular_rate_b)
                        .set(axis, sign * magnitude);
                    auto const result = fixture.check();
                    test.expect(result.status == Status::success && result.high_dynamics == (direction > 0), profile,
                                "below/equal/above each scalar threshold on either sign and every body axis");
                }
            }
        }
    }
    ImuDynamicsFixture<Linalg> small;
    auto const minimum_limit = std::ldexp(value_type{1}, (limits_type::min_exponent - 1) / 2);
    small.parameters = {minimum_limit, minimum_limit};
    small.imu.specific_force_b.set(0U, minimum_limit);
    auto result = small.check();
    test.expect(result.status == Status::success && !result.high_dynamics &&
                    result.specific_force_squared_norm == limits_type::min(),
                profile, "smallest normal squared threshold and its equality are valid");
    small.imu.specific_force_b.set(0U, std::nextafter(minimum_limit, limits_type::infinity()));
    result = small.check();
    test.expect(result.status == Status::success && result.high_dynamics, profile,
                "safe lower threshold still detects the next larger value");
    small.imu.specific_force_b.set(0U, limits_type::min());
    result = small.check();
    test.expect(result.status == Status::success && !result.high_dynamics, profile,
                "squared-norm underflow far below a valid threshold cannot create a high classification");
}

template <typename Linalg> void test_failures(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using limits_type = std::numeric_limits<value_type>;
    auto const fail = [&](ImuDynamicsFixture<Linalg> const & fixture, Status expected)
    {
        auto const before = fixture;
        auto const result = fixture.check();
        test.expect(result.status == expected && result.high_dynamics &&
                        result.specific_force_squared_norm == value_type{0} &&
                        result.angular_rate_squared_norm == value_type{0},
                    profile, "failure is explicit, never low dynamics, and publishes no valid diagnostics");
        test.expect(same_bits(fixture.imu.specific_force_b, before.imu.specific_force_b) &&
                        same_bits(fixture.imu.angular_rate_b, before.imu.angular_rate_b) &&
                        same_bits(fixture.b_a, before.b_a) && same_bits(fixture.b_g, before.b_g),
                    profile, "failed checks preserve even non-finite input coefficients");
    };
    for (value_type invalid : {limits_type::quiet_NaN(), limits_type::infinity(), -limits_type::infinity()})
    {
        for (unsigned field = 0U; field < 14U; ++field)
        {
            ImuDynamicsFixture<Linalg> fixture;
            if (field < 12U)
            {
                std::array<typename ImuDynamicsFixture<Linalg>::vector_type *, 4U> vectors{
                    &fixture.imu.specific_force_b, &fixture.imu.angular_rate_b, &fixture.b_a, &fixture.b_g};
                vectors[field / 3U]->set(field % 3U, invalid);
            }
            else
            {
                (field == 12U ? fixture.parameters.maximum_specific_force : fixture.parameters.maximum_angular_rate) =
                    invalid;
            }
            fail(fixture, Status::non_finite_input);
        }
    }
    auto const minimum_limit = std::ldexp(value_type{1}, (limits_type::min_exponent - 1) / 2);
    for (unsigned sensor = 0U; sensor < 2U; ++sensor)
    {
        for (value_type invalid :
             {value_type{0}, -value_type{0}, value_type{-1}, limits_type::min(), limits_type::denorm_min(),
              std::nextafter(minimum_limit, value_type{0}), limits_type::max()})
        {
            ImuDynamicsFixture<Linalg> fixture;
            (sensor == 0U ? fixture.parameters.maximum_specific_force : fixture.parameters.maximum_angular_rate) =
                invalid;
            fail(fixture, Status::domain_error);
        }
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            ImuDynamicsFixture<Linalg> fixture;
            auto & measurement = sensor == 0U ? fixture.imu.specific_force_b : fixture.imu.angular_rate_b;
            auto & bias = sensor == 0U ? fixture.b_a : fixture.b_g;
            measurement.set(axis, limits_type::max());
            bias.set(axis, -limits_type::max());
            fail(fixture, Status::non_finite_result); // Subtraction overflows.
            bias.set(axis, value_type{0});
            fail(fixture, Status::non_finite_result); // Individual square overflows.
            auto const component = std::ldexp(value_type{1.5}, limits_type::max_exponent / 2 - 1);
            for (std::size_t entry = 0U; entry < 3U; ++entry)
            {
                measurement.set(entry, component);
            }
            fail(fixture, Status::non_finite_result); // Finite squares overflow when summed.
        }
    }
    ImuDynamicsFixture<Linalg> competing;
    competing.imu.specific_force_b.set(0U, value_type{10}); // Already above threshold.
    competing.imu.angular_rate_b.set(2U, limits_type::quiet_NaN());
    fail(competing, Status::non_finite_input);
    competing.parameters.maximum_specific_force = value_type{0};
    fail(competing, Status::non_finite_input); // Non-finite inputs precede domain errors.
    competing.imu.angular_rate_b.set(2U, limits_type::max());
    competing.parameters.maximum_specific_force = value_type{4};
    fail(competing, Status::non_finite_result); // No short circuit after a high force reading.
    competing.parameters = {};
    fail(competing, Status::domain_error); // Invalid configuration precedes norm arithmetic.
}

template <typename Linalg> void test_learning_pipeline(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using Monitor = formal_eskf::runtime::ImuBiasLearning;
    using BiasUpdate = formal_eskf::configuration::Ins::BiasUpdate;
    using state_type = formal_eskf::configuration::Ins::NominalState<Linalg>;
    using covariance_type = formal_eskf::linalg::Matrix<Linalg, 15U, 15U>;
    ImuDynamicsFixture<Linalg> fixture;
    Monitor monitor;
    Monitor::Conditions external;
    external.accelerometer.healthy = true;
    external.gyroscope.healthy = true;
    external.accelerometer.permitted.fill(true);
    external.gyroscope.permitted.fill(true);
    constexpr Monitor::Parameters timing{10U, 5U};
    constexpr BiasUpdate disabled{{false, false, false}, {false, false, false}};
    constexpr BiasUpdate enabled{};
    constexpr BiasUpdate gyro_only{{false, false, false}, {true, true, true}};
    state_type const prior;
    auto P = covariance_type::identity();
    for (std::size_t row = 0U; row < 15U; ++row)
    {
        for (std::size_t column = 0U; column < 15U; ++column)
        {
            P(row, column) += static_cast<value_type>((row + 1U) * (column + 1U)) / value_type{256};
        }
    }
    auto const velocity =
        ImuDynamicsFixture<Linalg>::vector_type::from_row_major({value_type{0.125}, value_type{0.25}, value_type{0.5}});
    auto const V = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>::identity();
    auto const step = [&](std::uint64_t time_us, BiasUpdate const & expected, Status expected_check = Status::success)
    {
        auto const dynamics = fixture.check();
        auto conditions = external; // Do not overwrite independent health, clipping or observability policy.
        conditions.high_dynamics = dynamics.high_dynamics;
        if (dynamics.status != Status::success)
        {
            conditions.accelerometer.healthy = false;
            conditions.gyroscope.healthy = false;
        }
        auto const result = monitor.update(time_us, conditions, timing);
        test.expect(dynamics.status == expected_check && result.status == Status::success &&
                        result.bias_update.accelerometer == expected.accelerometer &&
                        result.bias_update.gyroscope == expected.gyroscope,
                    profile, "detector and existing policy implement immediate inhibition and one recovery interval");
        state_type posterior;
        covariance_type posterior_P;
        test.expect(formal_eskf::try_correct_velocity(prior, P, velocity, V, static_cast<value_type>(1e-6), posterior,
                                                      posterior_P, result.bias_update) == Status::success,
                    profile, "policy permissions reach real INS correction");
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            test.expect((posterior.b_a(axis) != prior.b_a(axis)) == expected.accelerometer[axis] &&
                            (posterior.b_g(axis) != prior.b_g(axis)) == expected.gyroscope[axis],
                        profile, "only expected bias axes change while correcting an independent velocity observation");
        }
        test.expect(formal_eskf::linalg::max_abs(posterior.v_n) > value_type{0} &&
                        formal_eskf::linalg::max_abs(posterior_P - P) > value_type{0},
                    profile, "bias inhibition is not a stop of state or covariance correction");
    };

    step(1U, disabled);
    step(6U, disabled);
    step(11U, enabled);
    fixture.imu.specific_force_b.set(0U, value_type{5});
    step(12U, gyro_only);
    fixture.imu.specific_force_b.set(0U, value_type{0});
    step(13U, gyro_only);
    step(18U, gyro_only);
    step(22U, gyro_only);
    step(23U, enabled);
    fixture.imu.angular_rate_b.set(1U, value_type{3});
    step(24U, gyro_only);
    fixture.imu.angular_rate_b.set(1U, std::numeric_limits<value_type>::quiet_NaN());
    step(25U, disabled, Status::non_finite_input);
    fixture.imu.angular_rate_b.set(1U, value_type{0});
    step(26U, disabled);
    step(31U, disabled);
    step(36U, enabled);
    external.accelerometer.clipped[0U] = true;
    BiasUpdate one_disabled = enabled;
    one_disabled.accelerometer[0U] = false;
    step(37U, one_disabled); // Low dynamics cannot override clipping metadata.
    external.accelerometer.clipped[0U] = false;
    step(38U, one_disabled);
    step(43U, one_disabled);
    step(48U, enabled);
    external.gyroscope.permitted[2U] = false;
    one_disabled = enabled;
    one_disabled.gyroscope[2U] = false;
    step(49U, one_disabled); // Low dynamics cannot establish bias observability.
    external.gyroscope.permitted[2U] = true;
    fixture.parameters.maximum_specific_force = value_type{8};
    monitor.reset(); // Threshold changes invalidate the caller's previous qualification context.
    step(50U, disabled);
    step(55U, disabled);
    step(60U, enabled);
    fixture.parameters.maximum_angular_rate = value_type{0};
    step(61U, disabled, Status::domain_error);
}

template <typename Linalg> void run_conformance(TestContext & test, std::string_view profile)
{
    test_magnitudes<Linalg>(test, profile);
    test_boundaries<Linalg>(test, profile);
    test_failures<Linalg>(test, profile);
    test_learning_pipeline<Linalg>(test, profile);
}

} /* end anonymous namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    TestContext test;
    run_conformance<formal_eskf::test::Backend<float>>(test, "float");
    run_conformance<formal_eskf::test::Backend<double>>(test, "double");
    if (test.failures() != 0)
    {
        std::cerr << test.failures() << " IMU dynamics check(s) failed\n";
        return 1;
    }
    std::cout << "IMU dynamics checks passed\n";
    return 0;
}
