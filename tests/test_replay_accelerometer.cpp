/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "../examples/replay_accelerometer.hpp"
#include "../examples/replay_conditions.hpp"
#include <formal_eskf/runtime/accelerometer.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <string_view>

#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{

using formal_eskf::Status;
using TestContext = formal_eskf::test::Context;
template <typename Linalg> using Sampler = formal_eskf::example::ReplayAccelerometer<Linalg, 10000U>;

template <typename Linalg> void test_sampling(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    auto const minimum_norm = static_cast<value_type>(1e-6);
    auto const tolerance = static_cast<value_type>(1e-3);
    formal_eskf::ImuSample<Linalg> imu{};
    imu.specific_force_b(2U) = value_type{-10};
    Sampler<Linalg> sampler;
    auto first = sampler.update(imu, 5000U, 0, minimum_norm);
    test.expect(first.status == Status::success && !first.ready, profile, "partial interval does not update LPF");
    auto second = sampler.update(imu, 5000U, 0, minimum_norm);
    test.expect(second.status == Status::success && second.ready && second.interval_us == 10000U &&
                    std::abs(second.specific_force_b(2U) + value_type{10}) < tolerance &&
                    std::abs(second.magnitude.squared_norm - value_type{100}) < tolerance,
                profile, "constant force is preserved and first complete observation seeds LPF");
    imu.specific_force_b(2U) = value_type{-20};
    test.expect(!sampler.update(imu, 5000U, 0, minimum_norm).ready, profile,
                "no duplicate observation between outputs");
    auto third = sampler.update(imu, 5000U, 0, minimum_norm);
    test.expect(third.ready && std::abs(third.specific_force_b(2U) + value_type{20}) < tolerance &&
                    std::abs(third.magnitude.squared_norm - value_type{121}) < tolerance,
                profile, "current force differs from LPF, which advances exactly once with alpha 0.1");

    sampler.reset();
    imu.specific_force_b(2U) = value_type{-8};
    test.expect(!sampler.update(imu, 2000U, 0, minimum_norm).ready, profile, "short first sample is accumulated");
    imu.specific_force_b(2U) = value_type{-12};
    auto weighted = sampler.update(imu, 9000U, 0, minimum_norm);
    test.expect(weighted.ready && weighted.interval_us == 11000U &&
                    std::abs(weighted.specific_force_b(2U) + value_type{124} / value_type{11}) < tolerance,
                profile, "average is duration-weighted, not the newest sample or sample-count average");

    sampler.reset();
    test.expect(!sampler.update(imu, 10000U, 0, minimum_norm).ready, profile,
                "target equality alone does not bypass the initial required sample count");
    auto const boundary = sampler.update(imu, 1U, 0, minimum_norm);
    test.expect(boundary.ready && boundary.interval_us == 10001U, profile,
                "the next real increment completes the interval without splitting or duplication");
    sampler.reset();
    auto const longest = sampler.update(imu, 50000U, 0, minimum_norm);
    test.expect(longest.ready && longest.interval_us == 50000U, profile,
                "a valid slow input emits once with its actual duration, not repeated 10 ms observations");

    sampler.reset();
    std::uint64_t integrated_us = 0U;
    std::uint64_t pending_us = 0U;
    for (unsigned sample = 0U; sample < 1000U; ++sample)
    {
        pending_us += 4000U;
        auto const result = sampler.update(imu, 4000U, 0, minimum_norm);
        test.expect(result.status == Status::success, profile, "250 Hz sample is valid");
        if (result.ready)
        {
            test.expect(result.interval_us == pending_us && (pending_us == 8000U || pending_us == 12000U), profile,
                        "whole-sample cadence reports 8/12 ms, never a fabricated exact 10 ms");
            integrated_us += result.interval_us;
            pending_us = 0U;
        }
    }
    test.expect(integrated_us + pending_us == 4000000U, profile, "intervals neither overlap nor duplicate samples");
}

template <typename Linalg> void test_rotation_and_flags(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    auto const minimum_norm = static_cast<value_type>(1e-6);
    auto const tolerance = static_cast<value_type>(1e-4);
    formal_eskf::ImuSample<Linalg> imu{};
    Sampler<Linalg> sampler;
    imu.specific_force_b(0U) = value_type{1};
    (void)sampler.update(imu, 5000U, 1, minimum_norm);
    imu.specific_force_b = {};
    imu.angular_rate_b(2U) = std::numbers::pi_v<value_type> * value_type{100};
    auto const rotated = sampler.update(imu, 5000U, 4, minimum_norm);
    test.expect(rotated.ready && std::abs(rotated.specific_force_b(0U)) < tolerance &&
                    std::abs(rotated.specific_force_b(1U) + value_type{0.5}) < tolerance && rotated.clipping_mask == 5,
                profile, "positive quarter-turn moves prior body X increment into new body minus Y");

    sampler.reset();
    imu = {};
    (void)sampler.update(imu, 5000U, 0, minimum_norm);
    imu.specific_force_b(0U) = value_type{1};
    imu.angular_rate_b(2U) = std::numbers::pi_v<value_type> * value_type{100};
    auto const endpoint = sampler.update(imu, 5000U, 0, minimum_norm);
    test.expect(endpoint.ready && std::abs(endpoint.specific_force_b(0U) - value_type{0.25}) < tolerance &&
                    std::abs(endpoint.specific_force_b(1U) + value_type{0.25}) < tolerance,
                profile, "newest increment uses both endpoint rotations, not only the final frame");

    imu = {};
    imu.specific_force_b(2U) = value_type{-10};
    for (int first_mask = -1; first_mask < 8; ++first_mask)
    {
        for (int second_mask = -1; second_mask < 8; ++second_mask)
        {
            sampler.reset();
            (void)sampler.update(imu, 5000U, first_mask, minimum_norm);
            auto const result = sampler.update(imu, 5000U, second_mask, minimum_norm);
            int const expected = first_mask < 0 || second_mask < 0 ? -1 : first_mask | second_mask;
            test.expect(result.ready && result.clipping_mask == expected && result.magnitude.within_gravity_band,
                        profile, "all clipping combinations aggregate across the interval without disabling LPF");
        }
    }
    (void)sampler.update(imu, 5000U, 7, minimum_norm);
    sampler.reset(); // Exactly the gap/source-change action used by the replay.
    imu.specific_force_b(2U) = value_type{-20};
    test.expect(!sampler.update(imu, 5000U, 0, minimum_norm).ready, profile, "reset discards the partial interval");
    auto const restarted = sampler.update(imu, 5000U, 0, minimum_norm);
    test.expect(restarted.ready && restarted.clipping_mask == 0 &&
                    std::abs(restarted.magnitude.squared_norm - value_type{400}) < static_cast<value_type>(1e-3),
                profile, "reset also clears old clipping and LPF; a fresh complete observation seeds it");
}

template <typename Linalg> void test_failure_preservation(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    auto const minimum_norm = static_cast<value_type>(1e-6);
    formal_eskf::ImuSample<Linalg> imu{};
    imu.specific_force_b(2U) = value_type{-10};
    Sampler<Linalg> baseline;
    (void)baseline.update(imu, 5000U, 0, minimum_norm);
    (void)baseline.update(imu, 5000U, 0, minimum_norm);
    (void)baseline.update(imu, 5000U, 1, minimum_norm);
    for (unsigned failure = 0U; failure < 8U; ++failure)
    {
        auto sampler = baseline;
        auto invalid = imu;
        std::uint64_t period = 5000U;
        int mask = 0;
        auto minimum = minimum_norm;
        auto expected_status = Status::domain_error;
        switch (failure)
        {
        case 0U:
            period = 0U;
            break;
        case 1U:
            period = 50001U;
            break;
        case 2U:
            mask = -2;
            break;
        case 3U:
            mask = 8;
            break;
        case 4U:
            minimum = value_type{0};
            break;
        case 5U:
            invalid.specific_force_b(0U) = std::numeric_limits<value_type>::quiet_NaN();
            expected_status = Status::non_finite_input;
            break;
        case 6U:
            invalid.angular_rate_b(0U) = std::numeric_limits<value_type>::max();
            expected_status = Status::non_finite_result;
            break;
        default:
            invalid.specific_force_b(0U) = std::numeric_limits<value_type>::max();
            expected_status = Status::non_finite_result; // LPF squared-norm overflow on the completed interval.
            break;
        }
        auto const rejected = sampler.update(invalid, period, mask, minimum);
        test.expect(rejected.status == expected_status && !rejected.ready, profile,
                    "invalid domain, input or arithmetic does not publish an observation");
        auto const recovered = sampler.update(imu, 5000U, 2, minimum_norm);
        test.expect(recovered.ready && recovered.interval_us == 10000U && recovered.clipping_mask == 3 &&
                        std::abs(recovered.specific_force_b(2U) + value_type{10}) < static_cast<value_type>(1e-4) &&
                        std::abs(recovered.magnitude.squared_norm - value_type{100}) < static_cast<value_type>(1e-3),
                    profile, "failure preserves increments, clipping, cadence and LPF for the next valid sample");
    }
}

template <typename Linalg> void test_fusion(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using matrix_type = typename Linalg::template matrix_type<3U, 3U>;
    using state_type = formal_eskf::configuration::Ins::NominalState<Linalg>;
    using covariance_type = typename Linalg::template matrix_type<15U, 15U>;
    auto const minimum_norm = static_cast<value_type>(1e-6);
    auto const g = static_cast<value_type>(9.80665);
    auto const V = matrix_type::identity() * (g * g);
    typename Linalg::template vector_type<3U> gravity{};
    gravity(2U) = g;
    formal_eskf::ImuSample<Linalg> imu{};
    imu.specific_force_b = -gravity;
    Sampler<Linalg> sampler;
    (void)sampler.update(imu, 5000U, 0, minimum_norm);
    (void)sampler.update(imu, 5000U, 0, minimum_norm);
    imu.specific_force_b(0U) = static_cast<value_type>(0.5);
    (void)sampler.update(imu, 5000U, 0, minimum_norm);
    auto const observation = sampler.update(imu, 5000U, 0, minimum_norm);
    state_type state;
    auto P = covariance_type::identity() * static_cast<value_type>(1e-3);
    formal_eskf::example::ReplayFusionConditions<Linalg> tracker;
    test.expect(tracker.update_tilt(state.q_nb, P.template block<6U, 6U, 3U, 3U>()) == Status::success, profile,
                "test prior has aligned tilt");
    auto const conditions = tracker.conditions(20000U, true, -1, observation.clipping_mask);
    test.expect(observation.ready && conditions.has_value() && conditions->tilt_aligned, profile,
                "complete sampled observation with unknown rest can be eligible");
    if (!conditions)
    {
        return;
    }
    state_type expected;
    covariance_type expected_P;
    test.expect(formal_eskf::try_correct_accelerometer(state, P, observation.specific_force_b, gravity, V, minimum_norm,
                                                       expected, expected_P) == Status::success,
                profile, "direct correction oracle uses the interval measurement and agreed V");
    auto const fused = formal_eskf::runtime::fuse_accelerometer(
        state, P, observation.specific_force_b, gravity, V, *conditions, observation.magnitude, minimum_norm, state, P);
    test.expect(
        fused.decision == formal_eskf::runtime::FusionDecision::fused &&
            formal_eskf::linalg::max_abs(state.q_nb.coefficients() - expected.q_nb.coefficients()) == value_type{0} &&
            formal_eskf::linalg::max_abs(P - expected_P) == value_type{0} && std::abs(state.q_nb.q2()) > value_type{0},
        profile, "sample-to-condition-to-correction path actually updates tilt with current, not LPF, force");
}

template <typename Scalar> void run_conformance(TestContext & test, std::string_view profile)
{
    using Linalg = formal_eskf::test::Backend<Scalar>;
    test_sampling<Linalg>(test, profile);
    test_rotation_and_flags<Linalg>(test, profile);
    test_failure_preservation<Linalg>(test, profile);
    test_fusion<Linalg>(test, profile);
}

} /* end namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    TestContext test;
    run_conformance<float>(test, "float");
    run_conformance<double>(test, "double");
    if (test.failures() != 0)
    {
        return 1;
    }
    std::cout << "Replay accelerometer tests passed\n";
    return 0;
}
