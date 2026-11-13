/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <string_view>

#include <formal_eskf/runtime/imu.hpp>
#include <formal_eskf/runtime/vertical_acceleration_health.hpp>
#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{

using formal_eskf::Status;
using TestContext = formal_eskf::test::Context;
constexpr std::string_view profile = "IMU bias inhibition";
constexpr std::array<bool, 3U> all{true, true, true};
constexpr std::array<bool, 3U> none{false, false, false};

template <typename Linalg> void test_policy(TestContext & test)
{
    using Scalar = typename Linalg::value_type;
    using Policy = formal_eskf::runtime::ImuBiasInhibition<Linalg>;
    using State = typename Policy::state_type;
    using Vector = formal_eskf::linalg::Vector<Linalg, 3U>;
    typename Policy::Parameters const parameters{Scalar{25}, Scalar{3}, Scalar{0.5}, true, true};
    State state;
    formal_eskf::ImuSample<Linalg> imu;
    imu.specific_force_b = Vector::from_row_major({Scalar{0}, Scalar{0}, Scalar{-10}});
    Policy policy(parameters);
    {
        Policy separate_periods(parameters);
        auto spike = imu;
        spike.specific_force_b(2U) = Scalar{-50};
        spike.angular_rate_b(0U) = Scalar{6};
        (void)separate_periods.update(Scalar{0.125}, Scalar{0.25}, state, spike, {});
        auto result = separate_periods.update(Scalar{0.125}, Scalar{0.25}, state, imu, {});
        test.expect(result.specific_force_peak == Scalar{25} && result.angular_rate_peak == Scalar{4.5} &&
                        result.high_dynamics,
                    profile, "each sensor envelope uses its own integration period");
        auto invalid = separate_periods.update(Scalar{0.125}, Scalar{0}, state, imu, {});
        test.expect(invalid.status == Status::domain_error && invalid.bias_update.accelerometer == none, profile,
                    "invalid accel period cannot authorize learning");
        result = separate_periods.update(Scalar{0.125}, Scalar{0.25}, state, imu, {});
        test.expect(result.specific_force_peak == Scalar{12.5} && result.angular_rate_peak == Scalar{3.375}, profile,
                    "failed evaluation preserves both independent envelopes");
    }
    {
        using Health = formal_eskf::runtime::VerticalAccelerationHealth<Linalg>;
        Health health;
        typename Health::Observations observations;
        observations.barometer_height = {true, Scalar{4}, Scalar{1}};
        observations.gnss_velocity_down = {true, Scalar{2}, Scalar{1}};
        test.expect(health.update({1000000U, static_cast<Scalar>(0.01), true, {}}, observations) == Status::success &&
                        health.state().bad_vertical_acceleration,
                    profile, "independent falling references set fault");
        typename Policy::Conditions conditions;
        conditions.bad_vertical_acceleration = health.state().bad_vertical_acceleration;
        auto result = policy.update(static_cast<Scalar>(0.01), static_cast<Scalar>(0.01), state, imu, conditions);
        test.expect(result.bias_update.accelerometer == none && result.bias_update.gyroscope == all, profile,
                    "preceding height health inhibits accel learning only");
        test.expect(health.update({3999999U, static_cast<Scalar>(0.01), true, {}}, {}) == Status::success &&
                        health.state().bad_vertical_acceleration,
                    profile, "fault holds before three seconds");
        test.expect(health.update({4000000U, static_cast<Scalar>(0.01), true, {}}, {}) == Status::success &&
                        !health.state().bad_vertical_acceleration,
                    profile, "fault clears at three-second boundary");
    }
    auto result = policy.update(Scalar{0.125}, Scalar{0.125}, state, imu, {});
    test.expect(result.status == Status::success && result.bias_update.accelerometer == all &&
                    result.bias_update.gyroscope == all,
                profile, "no fixed startup qualification delay");

    auto spike = imu;
    spike.specific_force_b(2U) = Scalar{-50};
    spike.angular_rate_b(0U) = Scalar{6};
    result = policy.update(Scalar{0.125}, Scalar{0.125}, state, spike, {});
    test.expect(result.high_dynamics && result.specific_force_peak == Scalar{50} &&
                    result.angular_rate_peak == Scalar{6} && result.bias_update.accelerometer == none &&
                    result.bias_update.gyroscope == all,
                profile, "spikes latch immediately; gyro remains enabled");
    // Independent exact-binary oracle: 50, 6 decay by 3/4 on each 125 ms interval.
    std::array<Scalar, 3U> const force_peaks{Scalar{37.5}, Scalar{28.125}, Scalar{21.09375}};
    std::array<Scalar, 3U> const rate_peaks{Scalar{4.5}, Scalar{3.375}, Scalar{2.53125}};
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        result = policy.update(Scalar{0.125}, Scalar{0.125}, state, imu, {});
        test.expect(result.specific_force_peak == force_peaks[index] && result.angular_rate_peak == rate_peaks[index] &&
                        result.high_dynamics == (index < 2U) &&
                        result.bias_update.accelerometer == (index < 2U ? none : all),
                    profile, "low samples retain the decaying peak; recovery follows threshold crossing, not 500 ms");
    }
    for (auto dt : {Scalar{0.5}, Scalar{1}})
    {
        (void)policy.update(Scalar{0.125}, Scalar{0.125}, state, spike, {});
        result = policy.update(dt, dt, state, imu, {});
        test.expect(!result.high_dynamics && result.specific_force_peak == Scalar{10} &&
                        result.angular_rate_peak == Scalar{0},
                    profile, "dt at or above tau clamps decay to zero");
    }
    for (bool rate_spike : {false, true})
    {
        Policy independent(parameters);
        auto disturbed = imu;
        if (rate_spike)
        {
            disturbed.angular_rate_b(0U) = Scalar{4};
        }
        else
        {
            disturbed.specific_force_b(2U) = Scalar{-26};
        }
        result = independent.update(Scalar{0.125}, Scalar{0.125}, state, disturbed, {true, false, false, false, 0});
        test.expect(result.high_dynamics && result.bias_update.accelerometer == none &&
                        result.bias_update.gyroscope == all,
                    profile, "either sensor alone inhibits accel bias; rest cannot override high dynamics");
        result = independent.update(Scalar{0.25}, Scalar{0.25}, state, imu, {});
        test.expect(!result.high_dynamics &&
                        (rate_spike ? result.angular_rate_peak == Scalar{2} : result.specific_force_peak == Scalar{13}),
                    profile, "decay uses the current integration period, including variable dt");
    }
    for (bool gyro_enabled : {false, true})
    {
        for (bool accel_enabled : {false, true})
        {
            auto configured = parameters;
            configured.gyroscope_enabled = gyro_enabled;
            configured.accelerometer_enabled = accel_enabled;
            Policy switched(configured);
            result = switched.update(Scalar{0.125}, Scalar{0.125}, state, imu, {});
            test.expect(result.bias_update.accelerometer == (accel_enabled ? all : none) &&
                            result.bias_update.gyroscope == (gyro_enabled ? all : none),
                        profile, "independent configuration switches govern learning immediately");
        }
    }
    for (bool at_rest : {false, true})
    {
        for (bool fake_height : {false, true})
        {
            for (bool fake_position : {false, true})
            {
                typename Policy::Conditions conditions{at_rest, fake_height, fake_position, false, 0};
                result = policy.update(Scalar{0.5}, Scalar{0.5}, state, imu, conditions);
                auto expected = all;
                if (!at_rest && fake_height)
                {
                    expected = none;
                }
                else if (!at_rest && fake_position)
                {
                    expected = {false, false, true};
                }
                test.expect(result.bias_update.accelerometer == expected, profile,
                            "rest overrides fake modes; fake height wins over fake position");
                conditions.bad_vertical_acceleration = true;
                result = policy.update(Scalar{0.5}, Scalar{0.5}, state, imu, conditions);
                test.expect(result.bias_update.accelerometer == none && result.bias_update.gyroscope == all, profile,
                            "bad vertical acceleration inhibits all accel axes, including at rest");
            }
        }
    }
    for (int mask = 0; mask < 8; ++mask)
    {
        result = policy.update(Scalar{0.5}, Scalar{0.5}, state, imu, {true, false, false, false, mask});
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            test.expect(result.bias_update.accelerometer[axis] == ((mask & (1 << axis)) == 0) &&
                            result.bias_update.gyroscope[axis],
                        profile, "accel clipping inhibits only its body axis");
        }
    }
    result = policy.update(Scalar{0.125}, Scalar{0.125}, state, imu, {});
    test.expect(result.bias_update.accelerometer == all, profile, "clipping release has no extra recovery timer");
    test.expect(State::quaternion_type::from_coefficients(Scalar{1}, Scalar{0}, Scalar{1}, Scalar{0},
                                                          static_cast<Scalar>(1e-6), state.q_nb) == Status::success,
                profile, "90-degree pitch fixture");
    result = policy.update(Scalar{0.125}, Scalar{0.125}, state, imu, {false, false, true, false, 0});
    test.expect(result.bias_update.accelerometer == std::array<bool, 3U>{true, false, false}, profile,
                "fake-position observability follows navigation Down in the body frame");
    state.b_a(2U) = Scalar{-25};
    state.b_g(0U) = Scalar{3};
    result = policy.update(Scalar{0.5}, Scalar{0.5}, state, spike, {});
    test.expect(result.specific_force_peak == Scalar{25} && result.angular_rate_peak == Scalar{3} &&
                    !result.high_dynamics && result.bias_update.accelerometer == all,
                profile, "prior biases are subtracted; equality to both magnitude limits is accepted");
}

template <typename Linalg> void test_failures(TestContext & test)
{
    using Scalar = typename Linalg::value_type;
    using Policy = formal_eskf::runtime::ImuBiasInhibition<Linalg>;
    typename Policy::Parameters const parameters{Scalar{25}, Scalar{3}, Scalar{0.5}, true, true};
    typename Policy::state_type state;
    formal_eskf::ImuSample<Linalg> imu;
    imu.specific_force_b(2U) = Scalar{-50};
    Policy policy(parameters);
    (void)policy.update(Scalar{0.125}, Scalar{0.125}, state, imu, {});
    auto bad = imu;
    bad.angular_rate_b(0U) = std::numeric_limits<Scalar>::quiet_NaN();
    auto result = policy.update(Scalar{0.125}, Scalar{0.125}, state, bad, {});
    test.expect(result.status == Status::non_finite_input && result.bias_update.accelerometer == none &&
                    result.bias_update.gyroscope == none,
                profile, "invalid input cannot publish learning permissions");
    for (auto dt : {Scalar{0}, Scalar{-1}, std::numeric_limits<Scalar>::infinity()})
    {
        result = policy.update(dt, dt, state, imu, {});
        test.expect(result.status != Status::success, profile, "invalid integration interval rejected");
    }
    for (int mask : {-1, 8})
    {
        result = policy.update(Scalar{0.125}, Scalar{0.125}, state, imu, {false, false, false, false, mask});
        test.expect(result.status == Status::domain_error, profile, "adapter must resolve missing or invalid clipping");
    }
    bad = imu;
    bad.specific_force_b(0U) = std::numeric_limits<Scalar>::max();
    result = policy.update(Scalar{0.125}, Scalar{0.125}, state, bad, {});
    test.expect(result.status == Status::non_finite_result, profile, "magnitude overflow rejected");
    imu.specific_force_b(2U) = Scalar{-10};
    result = policy.update(Scalar{0.125}, Scalar{0.125}, state, imu, {});
    test.expect(result.specific_force_peak == Scalar{37.5} && result.high_dynamics, profile,
                "all failed evaluations preserve the last measured peak");
    for (auto invalid : {Scalar{0}, Scalar{-1}, std::numeric_limits<Scalar>::infinity()})
    {
        for (std::size_t field = 0U; field < 3U; ++field)
        {
            auto configured = parameters;
            std::array<Scalar *, 3U> const fields{&configured.maximum_specific_force, &configured.maximum_angular_rate,
                                                  &configured.decay_time_constant};
            *fields[field] = invalid;
            Policy invalid_policy(configured);
            result = invalid_policy.update(Scalar{0.125}, Scalar{0.125}, state, imu, {});
            test.expect(result.status != Status::success && result.bias_update.accelerometer == none &&
                            result.bias_update.gyroscope == none,
                        profile, "invalid parameters disable permissions");
        }
    }
}

template <typename Linalg> void test_permissions(TestContext & test)
{
    using Scalar = typename Linalg::value_type;
    using Policy = formal_eskf::runtime::ImuBiasInhibition<Linalg>;
    using Quaternion = typename Policy::state_type::quaternion_type;
    // Exact unit quaternions mapping body X, Y or Z onto navigation Down.
    std::array<std::array<Scalar, 4U>, 3U> const attitudes{{{Scalar{0.5}, Scalar{-0.5}, Scalar{-0.5}, Scalar{-0.5}},
                                                            {Scalar{0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.5}},
                                                            {Scalar{1}, Scalar{0}, Scalar{0}, Scalar{0}}}};
    for (std::size_t down_axis = 0U; down_axis < attitudes.size(); ++down_axis)
    {
        typename Policy::state_type state;
        auto const & q = attitudes[down_axis];
        test.expect(Quaternion::from_coefficients(q[0], q[1], q[2], q[3], Scalar{0.125}, state.q_nb) == Status::success,
                    profile, "exact axis-aligned attitude fixture");
        for (unsigned flags = 0U; flags < 128U; ++flags)
        {
            bool const accel_enabled = (flags & 1U) != 0U;
            bool const gyro_enabled = (flags & 2U) != 0U;
            bool const at_rest = (flags & 4U) != 0U;
            bool const fake_height = (flags & 8U) != 0U;
            bool const fake_position = (flags & 16U) != 0U;
            bool const vertical_fault = (flags & 32U) != 0U;
            bool const high_dynamics = (flags & 64U) != 0U;
            Policy policy({Scalar{25}, Scalar{3}, Scalar{0.5}, accel_enabled, gyro_enabled});
            formal_eskf::ImuSample<Linalg> imu;
            imu.specific_force_b(2U) = high_dynamics ? Scalar{-50} : Scalar{-10};
            for (int clipping = 0; clipping < 8; ++clipping)
            {
                auto const result = policy.update(Scalar{0.5}, Scalar{0.5}, state, imu,
                                                  {at_rest, fake_height, fake_position, vertical_fault, clipping});
                test.expect(result.status == Status::success && result.high_dynamics == high_dynamics, profile,
                            "switches and observability modes do not bypass motion detection");
                for (std::size_t axis = 0U; axis < 3U; ++axis)
                {
                    bool const expected = accel_enabled && !high_dynamics && !vertical_fault &&
                                          (clipping & (1 << axis)) == 0 &&
                                          (at_rest || (!fake_height && (!fake_position || axis == down_axis)));
                    test.expect(result.bias_update.accelerometer[axis] == expected &&
                                    result.bias_update.gyroscope[axis] == gyro_enabled,
                                profile,
                                "complete permission combinations preserve rest precedence and body-axis masks");
                }
            }
        }
    }
}

template <typename Linalg> void test_failed_sample_history(TestContext & test)
{
    using Scalar = typename Linalg::value_type;
    using Policy = formal_eskf::runtime::ImuBiasInhibition<Linalg>;
    typename Policy::state_type state;
    formal_eskf::ImuSample<Linalg> imu;
    imu.specific_force_b(2U) = Scalar{-50};
    imu.angular_rate_b(0U) = Scalar{6};
    Policy original({Scalar{25}, Scalar{3}, Scalar{0.5}, true, true});
    test.expect(original.update(Scalar{0.125}, Scalar{0.25}, state, imu, {}).status == Status::success, profile,
                "seed both envelopes before invalid samples");
    for (bool invalid_angular : {false, true})
    {
        for (auto invalid :
             {Scalar{0}, Scalar{-1}, std::numeric_limits<Scalar>::infinity(), std::numeric_limits<Scalar>::quiet_NaN()})
        {
            auto policy = original;
            auto const result = policy.update(invalid_angular ? invalid : Scalar{0.125},
                                              invalid_angular ? Scalar{0.25} : invalid, state, imu, {});
            auto const expected = std::isfinite(invalid) ? Status::domain_error : Status::non_finite_input;
            test.expect(result.status == expected && result.high_dynamics && result.specific_force_peak == Scalar{0} &&
                            result.angular_rate_peak == Scalar{0} && result.bias_update.accelerometer == none &&
                            result.bias_update.gyroscope == none,
                        profile, "either invalid integration period returns fail-closed placeholder diagnostics");
            auto recovered = policy.update(Scalar{0.125}, Scalar{0.25}, state, {}, {});
            test.expect(recovered.status == Status::success && recovered.specific_force_peak == Scalar{25} &&
                            recovered.angular_rate_peak == Scalar{4.5},
                        profile, "failure never advances either envelope, including when the other period was valid");
        }
    }
}

} /* end namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    TestContext test;
    test_policy<formal_eskf::test::Backend<float>>(test);
    test_policy<formal_eskf::test::Backend<double>>(test);
    test_failures<formal_eskf::test::Backend<float>>(test);
    test_failures<formal_eskf::test::Backend<double>>(test);
    test_permissions<formal_eskf::test::Backend<float>>(test);
    test_permissions<formal_eskf::test::Backend<double>>(test);
    test_failed_sample_history<formal_eskf::test::Backend<float>>(test);
    test_failed_sample_history<formal_eskf::test::Backend<double>>(test);
    if (test.failures() != 0)
    {
        return 1;
    }
    std::cout << "IMU bias inhibition: pass\n";
    return 0;
}
