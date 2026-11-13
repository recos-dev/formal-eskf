// SPDX-License-Identifier: BSD-3-Clause
#include <array>
#include <limits>

#include "../integrations/px4/module/ImuPrediction.hpp"
#include <formal_eskf/runtime/imu_bias_inhibition.hpp>
#include <formal_eskf/runtime/vertical_acceleration_health.hpp>
#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{
using formal_eskf::Status;
using formal_eskf::test::Context;
using Ins = formal_eskf::configuration::Ins;

template <typename Matrix> bool equal(Matrix const & a, Matrix const & b)
{
    for (std::size_t row = 0; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0; column < Matrix::column_count; ++column)
        {
            if (a(row, column) != b(row, column))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename Linalg> void run(Context & test)
{
    using Scalar = typename Linalg::value_type;
    using Vector = formal_eskf::linalg::Vector<Linalg, 3>;
    using Covariance = formal_eskf::linalg::Matrix<Linalg, 15, 15>;
    using Health = formal_eskf::runtime::VerticalAccelerationHealth<Linalg>;
    using Bias = formal_eskf::runtime::ImuBiasInhibition<Linalg>;
    using formal_eskf::px4::predict_imu;
    constexpr Scalar dt = Scalar{0.125};
    constexpr Scalar tolerance = static_cast<Scalar>(1e-6);
    Ins::Parameters<Linalg> parameters;
    parameters.minimum_quaternion_norm = Scalar{0.125};
    parameters.quaternion_squared_norm_tolerance = Scalar{0.125};
    parameters.dt_min = dt;
    parameters.dt_max = dt;
    parameters.process_noise.specific_force_variance = Vector::from_row_major({1, 2, 3});
    parameters.process_noise.angular_rate_variance = Vector::from_row_major({4, 5, 6});
    parameters.process_noise.accelerometer_bias_random_walk_variance_density = Vector::from_row_major({1, 2, 3});
    parameters.process_noise.gyroscope_bias_random_walk_variance_density = Vector::from_row_major({4, 5, 6});
    Ins::NominalState<Linalg> initial;
    // Cyclic rotation: body X -> NED Y, Y -> Z, Z -> X.
    test.expect(decltype(initial.q_nb)::from_coefficients(Scalar{0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.5},
                                                          Scalar{0.125}, initial.q_nb) == Status::success,
                "setup", "unit rotation");
    formal_eskf::ImuSample<Linalg> imu{};
    struct Case
    {
        std::uint8_t clipping;
        bool fault;
        std::array<double, 3> variance;
    };
    std::array const cases{
        Case{0, false, {1, 2, 3}},     Case{1, false, {24.01, 2, 3}},         Case{2, false, {1, 24.01, 3}},
        Case{4, false, {1, 2, 24.01}}, Case{7, false, {24.01, 24.01, 24.01}}, Case{0, true, {24.01, 24.01, 24.01}}};
    for (auto const & item : cases)
    {
        auto state = initial;
        Covariance P;
        test.expect(predict_imu(state, P, imu, dt, parameters, item.clipping, item.fault) == Status::success,
                    "prediction", "real module prediction accepts selected clipped intervals");
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            auto const nav_axis = (axis + 1) % 3;
            test.expect(formal_eskf::test::near(P(3 + nav_axis, 3 + nav_axis),
                                                static_cast<Scalar>(item.variance[axis]) * dt * dt, tolerance),
                        "noise", "body-axis variance rotates to NED and scales exactly once by dt squared");
            test.expect(P(6 + axis, 6 + axis) == Scalar(axis + 4) * dt * dt &&
                            P(9 + axis, 9 + axis) == Scalar(axis + 1) * dt &&
                            P(12 + axis, 12 + axis) == Scalar(axis + 4) * dt,
                        "noise", "gyro variance and both bias driving intensities are unchanged");
        }
        test.expect(equal(state.p_n, initial.p_n) && equal(state.v_n, initial.v_n) &&
                        equal(state.q_nb.coefficients(), initial.q_nb.coefficients()),
                    "prediction", "uncertainty selection does not alter nominal motion");
        // Clearing the flags must restore configured noise, even after all-axis fault selection.
        auto expected_state = state;
        auto expected_P = P;
        test.expect(formal_eskf::predict(expected_state, expected_P, imu, dt, parameters, expected_state, expected_P) ==
                            Status::success &&
                        predict_imu(state, P, imu, dt, parameters, 0, false) == Status::success && equal(P, expected_P),
                    "recovery", "clear sample and configured-noise gap fallback cannot inherit an override");
    }
    {
        auto state = initial;
        Covariance P;
        auto measured = imu;
        measured.specific_force_b = Vector::from_row_major({2, 0, 0});
        test.expect(predict_imu(state, P, measured, dt, parameters, 1, false) == Status::success &&
                        equal(state.v_n, Vector::from_row_major({0, Scalar{0.25}, 0})) &&
                        equal(state.p_n, Vector::from_row_major({0, Scalar{0.015625}, 0})),
                    "clipped motion", "retain measured increments rather than skipping or zeroing the interval");
    }
    for (auto const clipping : {std::uint8_t{0}, std::uint8_t{1}})
    {
        auto state = initial;
        Covariance P;
        std::array<Scalar, 6> const prior{1, 3, Scalar{2.5}, 4, Scalar{4.5}, 7};
        for (std::size_t axis = 0; axis < prior.size(); ++axis)
        {
            P(9 + axis, 9 + axis) = prior[axis];
        }
        test.expect(predict_imu(state, P, imu, dt, parameters, clipping, false) == Status::success, "bias noise",
                    "module and gap-hold predictions select bias driving noise");
        std::array<Scalar, 6> const increment{clipping == 0 ? Scalar{0} : dt, 0, Scalar{3} * dt, 0, Scalar{5} * dt, 0};
        for (std::size_t axis = 0; axis < prior.size(); ++axis)
        {
            test.expect(P(9 + axis, 9 + axis) == prior[axis] + increment[axis], "bias noise",
                        "strict threshold uses selected IMU variance; intensity scales by dt without clamping P");
        }
        P(9, 9) = Scalar{0.5};
        test.expect(predict_imu(state, P, imu, dt, parameters, 0, false) == Status::success &&
                        P(9, 9) == Scalar{0.5} + dt,
                    "bias noise", "lower prior variance restores configured driving noise on a later prediction");
    }
    for (unsigned failure = 0; failure < 4; ++failure)
    {
        auto state = initial;
        state.p_n = Vector::from_row_major({1, 2, 3});
        auto const before = state;
        Covariance P = Covariance::identity();
        if (failure == 3)
        {
            P(9, 9) = Scalar{-1};
        }
        auto const before_P = P;
        auto invalid = parameters;
        if (failure == 0)
        {
            invalid.process_noise.specific_force_variance(0) = std::numeric_limits<Scalar>::quiet_NaN();
        }
        test.expect(predict_imu(state, P, imu, failure == 1 ? -dt : dt, invalid, failure == 2 ? 8 : 1, false) !=
                        Status::success,
                    "failure", "invalid noise, interval, clipping mask or bias variance is rejected");
        test.expect(equal(state.p_n, before.p_n) && equal(state.v_n, before.v_n) && equal(state.b_a, before.b_a) &&
                        equal(state.b_g, before.b_g) && equal(state.q_nb.coefficients(), before.q_nb.coefficients()) &&
                        equal(P, before_P),
                    "failure", "selection or prediction failure preserves the complete estimate");
    }
    // Follow the module order: preceding health -> bias permissions -> prediction -> current health.
    Health health;
    Bias bias{{Scalar{25}, Scalar{3}, Scalar{0.5}, true, true}};
    typename Health::Sample sample{1000000, dt, true, {false, false, true}};
    typename Health::Observations observations;
    observations.barometer_height = {true, Scalar{4}, Scalar{1}}; // One reference above 3 sigma: medium likelihood.
    for (unsigned step = 0; step < 5; ++step)
    {
        bool const preceding_fault = health.state().bad_vertical_acceleration;
        auto state = initial;
        Covariance P;
        typename Bias::Conditions conditions;
        conditions.accelerometer_clipping = 4;
        conditions.bad_vertical_acceleration = preceding_fault;
        auto const permissions = bias.update(dt, dt, state, imu, conditions);
        test.expect(permissions.status == Status::success && !permissions.bias_update.accelerometer[2] &&
                        permissions.bias_update.accelerometer[0] == !preceding_fault &&
                        permissions.bias_update.gyroscope[0],
                    "health", "current clipped axis is inhibited; preceding fault inhibits all accel axes");
        test.expect(
            predict_imu(state, P, imu, dt, parameters, 4, preceding_fault) == Status::success &&
                formal_eskf::test::near(P(4, 4), static_cast<Scalar>(preceding_fault ? 24.01 : 1) * dt * dt, tolerance),
            "health", "all-axis noise follows the preceding verdict, not a future fault");
        test.expect(health.update(sample, observations) == Status::success && health.state().clip_count[2] == step + 1,
                    "health", "accepted clipped samples accumulate detector history");
        sample.time_us += 125000;
    }
    test.expect(health.state().bad_vertical_acceleration, "health",
                "frequent clipping plus apparent falling sets fault");
    sample.clipped = {};
    observations = {};
    sample.time_us = health.state().last_bad_time_us + 3000000;
    test.expect(health.update(sample, observations) == Status::success && !health.state().bad_vertical_acceleration,
                "health", "healthy evidence clears the three-second hold");
    health.reset();
    test.expect(health.state().last_update_time_us == 0 && health.state().clip_count[2] == 0, "gap",
                "missing-data reset drops detector continuity");
}
} /* end namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    Context test;
    struct ClippingCase
    {
        std::uint8_t accel, gyro;
        bool initialized, accepted;
    };
    std::array const cases{ClippingCase{0, 0, false, true},  ClippingCase{1, 0, false, false},
                           ClippingCase{7, 0, false, false}, ClippingCase{1, 0, true, true},
                           ClippingCase{4, 0, true, true},   ClippingCase{7, 0, true, true},
                           ClippingCase{8, 0, true, false},  ClippingCase{255, 0, true, false},
                           ClippingCase{0, 1, true, false},  ClippingCase{7, 4, true, false}};
    for (auto const & item : cases)
    {
        test.expect(formal_eskf::px4::imu_clipping_acceptable(item.accel, item.gyro, item.initialized) == item.accepted,
                    "acceptance", "startup, valid accel bits, unknown bits and gyro clipping");
    }
    run<formal_eskf::test::Backend<float>>(test);
    run<formal_eskf::test::Backend<double>>(test);
    return test.failures() == 0 ? 0 : 1;
}
