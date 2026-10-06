/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/imu_bias_learning.hpp>
#include <formal_eskf/eskf/velocity.hpp>

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
using BiasUpdate = formal_eskf::configuration::Ins::BiasUpdate;
using Monitor = formal_eskf::runtime::ImuBiasLearning;
using TestContext = formal_eskf::test::Context;

constexpr BiasUpdate disabled{{false, false, false}, {false, false, false}};
constexpr BiasUpdate enabled{};

Monitor::Conditions healthy_conditions()
{
    Monitor::Conditions conditions;
    conditions.accelerometer.healthy = true;
    conditions.gyroscope.healthy = true;
    conditions.accelerometer.permitted.fill(true);
    conditions.gyroscope.permitted.fill(true);
    return conditions;
}

bool same_permissions(BiasUpdate const & a, BiasUpdate const & b)
{
    return a.accelerometer == b.accelerometer && a.gyroscope == b.gyroscope;
}

void expect_permissions(TestContext & test, Monitor::Result const & result, BiasUpdate const & expected,
                        std::string_view description)
{
    test.expect(result.status == Status::success && same_permissions(result.bias_update, expected), "policy",
                description);
}

void test_conditions(TestContext & test)
{
    constexpr Monitor::Parameters parameters{10U, 10U};
    Monitor initial;
    static_assert(noexcept(initial.update(1U, {}, parameters)));
    static_assert(noexcept(initial.reset()));
    expect_permissions(test, initial.update(1U, {}, parameters), disabled,
                       "unknown health and permissions fail closed");
    expect_permissions(test, initial.update(11U, {}, parameters), disabled, "time alone cannot authorize learning");

    // Exhaust the Boolean input space independently of the temporal scenarios.
    for (unsigned permissions = 0U; permissions < 64U; ++permissions)
    {
        for (unsigned clipping = 0U; clipping < 64U; ++clipping)
        {
            for (unsigned health_motion = 0U; health_motion < 8U; ++health_motion)
            {
                Monitor monitor;
                Monitor::Conditions conditions;
                conditions.accelerometer.healthy = (health_motion & 1U) != 0U;
                conditions.gyroscope.healthy = (health_motion & 2U) != 0U;
                conditions.high_dynamics = (health_motion & 4U) != 0U;
                BiasUpdate expected = disabled;
                for (unsigned axis = 0U; axis < 3U; ++axis)
                {
                    conditions.accelerometer.permitted[axis] = (permissions & (1U << axis)) != 0U;
                    conditions.gyroscope.permitted[axis] = (permissions & (1U << (axis + 3U))) != 0U;
                    conditions.accelerometer.clipped[axis] = (clipping & (1U << axis)) != 0U;
                    conditions.gyroscope.clipped[axis] = (clipping & (1U << (axis + 3U))) != 0U;
                    expected.accelerometer[axis] =
                        (permissions & ~clipping & (1U << axis)) != 0U && (health_motion & 5U) == 1U;
                    expected.gyroscope[axis] =
                        (permissions & ~clipping & (1U << (axis + 3U))) != 0U && (health_motion & 2U) != 0U;
                }
                expect_permissions(test, monitor.update(1U, conditions, parameters), disabled, "startup always waits");
                expect_permissions(test, monitor.update(11U, conditions, parameters), expected,
                                   "each qualified axis follows permission, clipping, health and motion truth table");
            }
        }
    }
}

void test_recovery(TestContext & test)
{
    constexpr Monitor::Parameters parameters{100U, 50U};
    for (std::size_t index = 0U; index < 6U; ++index)
    {
        Monitor monitor;
        auto conditions = healthy_conditions();
        auto & sensor = index < 3U ? conditions.accelerometer : conditions.gyroscope;
        auto const axis = index % 3U;
        BiasUpdate one_disabled = enabled;
        (index < 3U ? one_disabled.accelerometer : one_disabled.gyroscope)[axis] = false;

        expect_permissions(test, monitor.update(100U, conditions, parameters), disabled,
                           "first good sample starts recovery");
        expect_permissions(test, monitor.update(150U, conditions, parameters), disabled, "not enough good time yet");
        expect_permissions(test, monitor.update(199U, conditions, parameters), disabled,
                           "one microsecond before recovery");
        expect_permissions(test, monitor.update(200U, conditions, parameters), enabled,
                           "recovery equality is accepted");
        sensor.clipped[axis] = true;
        expect_permissions(test, monitor.update(201U, conditions, parameters), one_disabled,
                           "clipping inhibits immediately");
        sensor.clipped[axis] = false;
        expect_permissions(test, monitor.update(202U, conditions, parameters), one_disabled,
                           "first unclipped sample starts new interval");
        expect_permissions(test, monitor.update(252U, conditions, parameters), one_disabled,
                           "other axes remain qualified");
        expect_permissions(test, monitor.update(301U, conditions, parameters), one_disabled,
                           "bad interval is not credited as good time");
        expect_permissions(test, monitor.update(302U, conditions, parameters), enabled,
                           "only the affected axis needed requalification");

        sensor.permitted[axis] = false;
        expect_permissions(test, monitor.update(303U, conditions, parameters), one_disabled,
                           "permission withdrawal inhibits immediately");
        sensor.permitted[axis] = true;
        expect_permissions(test, monitor.update(304U, conditions, parameters), one_disabled,
                           "permission restoration must qualify again");
        sensor.clipped[axis] = true;
        expect_permissions(test, monitor.update(305U, conditions, parameters), one_disabled,
                           "new clipping restarts recovery");
        sensor.clipped[axis] = false;
        expect_permissions(test, monitor.update(306U, conditions, parameters), one_disabled,
                           "recovery starts after the latest bad sample");
        expect_permissions(test, monitor.update(356U, conditions, parameters), one_disabled,
                           "intermittent good samples do not accumulate");
        expect_permissions(test, monitor.update(405U, conditions, parameters), one_disabled,
                           "repeated fault delays recovery fully");
        expect_permissions(test, monitor.update(406U, conditions, parameters), enabled,
                           "continuous recovery eventually enables learning");

        conditions.high_dynamics = true;
        BiasUpdate gyro_only = disabled;
        gyro_only.gyroscope.fill(true);
        expect_permissions(test, monitor.update(407U, conditions, parameters), gyro_only,
                           "high dynamics inhibits accelerometer bias only");
        conditions.gyroscope.healthy = false;
        expect_permissions(test, monitor.update(408U, conditions, parameters), disabled,
                           "unhealthy gyro inhibits all its axes");
        conditions = healthy_conditions();
        expect_permissions(test, monitor.update(409U, conditions, parameters), disabled,
                           "both sensor groups must recover");
        expect_permissions(test, monitor.update(459U, conditions, parameters), disabled,
                           "recovery is not immediate when faults clear");
        expect_permissions(test, monitor.update(509U, conditions, parameters), enabled,
                           "both sensor groups recover after good interval");
        conditions.accelerometer.healthy = false;
        expect_permissions(test, monitor.update(510U, conditions, parameters), gyro_only,
                           "unhealthy accelerometer leaves gyro policy independent");

        monitor.reset();
        expect_permissions(test, monitor.update(1U, healthy_conditions(), parameters), disabled,
                           "explicit source reset clears qualification and accepts a new timeline");
    }
}

void test_time_and_parameters(TestContext & test)
{
    Monitor monitor;
    auto const conditions = healthy_conditions();
    Monitor::Parameters parameters{100U, 50U};
    expect_permissions(test, monitor.update(1U, conditions, parameters), disabled, "start timing sequence");
    expect_permissions(test, monitor.update(51U, conditions, parameters), disabled,
                       "sample-gap equality preserves recovery");
    expect_permissions(test, monitor.update(101U, conditions, parameters), enabled, "qualify with continuous samples");
    expect_permissions(test, monitor.update(152U, conditions, parameters), disabled,
                       "long gap revokes all authorizations");
    expect_permissions(test, monitor.update(202U, conditions, parameters), disabled,
                       "unobserved gap does not count toward recovery");
    expect_permissions(test, monitor.update(252U, conditions, parameters), enabled, "recovery after gap");
    parameters.recovery_time_us = 50U;
    expect_permissions(test, monitor.update(253U, conditions, parameters), disabled,
                       "shorter configured recovery still restarts history");
    expect_permissions(test, monitor.update(303U, conditions, parameters), enabled,
                       "qualify under new recovery parameter");
    parameters.maximum_sample_interval_us = 40U;
    expect_permissions(test, monitor.update(304U, conditions, parameters), disabled,
                       "sample-interval parameter change also restarts history");
    expect_permissions(test, monitor.update(344U, conditions, parameters), disabled,
                       "new sample-gap equality is accepted");
    expect_permissions(test, monitor.update(354U, conditions, parameters), enabled, "recover after parameter change");

    for (unsigned fault = 0U; fault < 5U; ++fault)
    {
        auto invalid = monitor;
        auto bad_parameters = parameters;
        std::uint64_t const bad_time = fault == 0U ? 0U : (fault == 1U ? 354U : 353U);
        if (fault == 3U)
        {
            bad_parameters.recovery_time_us = 0U;
        }
        if (fault == 4U)
        {
            bad_parameters.maximum_sample_interval_us = 0U;
        }
        auto const result = invalid.update(bad_time, conditions, bad_parameters);
        test.expect(result.status == (fault < 3U ? Status::out_of_range : Status::domain_error) &&
                        same_permissions(result.bias_update, disabled),
                    "time",
                    "invalid time/configuration cannot return an old authorization; configuration has precedence");
        expect_permissions(test, invalid.update(355U, conditions, parameters), disabled,
                           "failure clears all previous good time");
        expect_permissions(test, invalid.update(395U, conditions, parameters), disabled,
                           "old timer cannot shorten recovery after failure");
        expect_permissions(test, invalid.update(405U, conditions, parameters), enabled,
                           "valid samples recover after an invalid call");
    }
    auto const invalid_defaults = monitor.update(355U, conditions, {});
    test.expect(invalid_defaults.status == Status::domain_error &&
                    same_permissions(invalid_defaults.bias_update, disabled),
                "time", "timing parameters require explicit configuration");

    constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
    Monitor large_time;
    parameters = {100U, 50U};
    expect_permissions(test, large_time.update(maximum - 100U, conditions, parameters), disabled,
                       "large timestamp starts normally");
    expect_permissions(test, large_time.update(maximum - 50U, conditions, parameters), disabled,
                       "large timestamp intermediate sample");
    expect_permissions(test, large_time.update(maximum, conditions, parameters), enabled,
                       "elapsed comparison does not overflow at UINT64_MAX");
    auto const wrapped = large_time.update(0U, conditions, parameters);
    test.expect(wrapped.status == Status::out_of_range && same_permissions(wrapped.bias_update, disabled), "time",
                "clock wrap is not forward progress");
    expect_permissions(test, large_time.update(1U, conditions, parameters), disabled, "wrap failure clears history");
    large_time.reset();
    parameters = {maximum, maximum};
    expect_permissions(test, large_time.update(1U, conditions, parameters), disabled,
                       "maximum interval does not overflow");
    expect_permissions(test, large_time.update(maximum, conditions, parameters), disabled,
                       "elapsed maximum minus one does not satisfy maximum duration");
}

template <typename Linalg> void test_correction_integration(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using state_type = formal_eskf::configuration::Ins::NominalState<Linalg>;
    using vector_type = formal_eskf::linalg::Matrix<Linalg, 3U, 1U>;
    using covariance_type = formal_eskf::linalg::Matrix<Linalg, 15U, 15U>;
    state_type const prior;
    auto P = covariance_type::identity();
    for (std::size_t row = 0U; row < 15U; ++row)
    {
        for (std::size_t column = 0U; column < 15U; ++column)
        {
            P(row, column) += static_cast<value_type>((row + 1U) * (column + 1U)) / value_type{256};
        }
    }
    auto const velocity = vector_type::from_row_major({value_type{0.125}, value_type{0.25}, value_type{0.5}});
    auto const V = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>::identity();
    constexpr auto minimum_norm = static_cast<value_type>(1e-6);
    constexpr Monitor::Parameters parameters{10U, 10U};
    for (unsigned permissions = 0U; permissions < 64U; ++permissions)
    {
        auto conditions = healthy_conditions();
        for (unsigned axis = 0U; axis < 3U; ++axis)
        {
            conditions.accelerometer.permitted[axis] = (permissions & (1U << axis)) != 0U;
            conditions.gyroscope.permitted[axis] = (permissions & (1U << (axis + 3U))) != 0U;
        }
        Monitor monitor;
        for (std::uint64_t time_us : {1U, 11U})
        {
            auto const policy = monitor.update(time_us, conditions, parameters);
            state_type posterior;
            covariance_type posterior_P;
            test.expect(policy.status == Status::success &&
                            formal_eskf::try_correct_velocity(prior, P, velocity, V, minimum_norm, posterior,
                                                              posterior_P, policy.bias_update) == Status::success,
                        profile, "runtime permissions feed the real INS correction API");
            test.expect(formal_eskf::linalg::max_abs(posterior.v_n - prior.v_n) > value_type{0} &&
                            formal_eskf::linalg::max_abs(posterior_P - P) > value_type{0},
                        profile, "inhibiting bias learning does not stop velocity or covariance correction");
            for (unsigned axis = 0U; axis < 3U; ++axis)
            {
                bool const accel_changes = time_us == 11U && (permissions & (1U << axis)) != 0U;
                bool const gyro_changes = time_us == 11U && (permissions & (1U << (axis + 3U))) != 0U;
                test.expect((posterior.b_a(axis) != prior.b_a(axis)) == accel_changes &&
                                (posterior.b_g(axis) != prior.b_g(axis)) == gyro_changes,
                            profile, "only temporally qualified body-axis biases change through cross-covariances");
            }
        }
    }
}

} /* end anonymous namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    TestContext test;
    test_conditions(test);
    test_recovery(test);
    test_time_and_parameters(test);
    test_correction_integration<formal_eskf::test::Backend<float>>(test, "float");
    test_correction_integration<formal_eskf::test::Backend<double>>(test, "double");
    if (test.failures() != 0)
    {
        std::cerr << test.failures() << " IMU bias-learning check(s) failed\n";
        return 1;
    }
    std::cout << "IMU bias-learning checks passed\n";
    return 0;
}
