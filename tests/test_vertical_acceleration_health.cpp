/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string_view>

#include <formal_eskf/runtime/velocity.hpp>
#include <formal_eskf/runtime/vertical_acceleration_health.hpp>
#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{

using formal_eskf::Status;
using TestContext = formal_eskf::test::Context;

template <typename Linalg> struct VerticalHealthFixture
{
    using monitor_type = formal_eskf::runtime::VerticalAccelerationHealth<Linalg>;
    using value_type = typename Linalg::value_type;
    monitor_type monitor;
    typename monitor_type::Sample sample{1'000'000U, value_type{0.125}, true, {false, false, false}};
    typename monitor_type::Observations observations;

    auto inputs()
    {
        return std::array<typename monitor_type::Innovation *, 6U>{
            &observations.barometer_height, &observations.gnss_height,   &observations.gnss_velocity_down,
            &observations.range_height,     &observations.vision_height, &observations.vision_velocity_down};
    }

    Status update() { return monitor.update(sample, observations); }
};

template <typename State> bool same_state(State const & a, State const & b)
{
    return a.likelihood == b.likelihood && a.clip_count == b.clip_count && a.bad_clipping == b.bad_clipping &&
           a.bad_sample == b.bad_sample && a.bad_vertical_acceleration == b.bad_vertical_acceleration &&
           a.last_bad_time_us == b.last_bad_time_us && a.last_good_time_us == b.last_good_time_us &&
           a.last_update_time_us == b.last_update_time_us;
}

template <typename Linalg> void test_likelihood(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    using Likelihood = typename VerticalHealthFixture<Linalg>::monitor_type::Likelihood;
    // Every named source can supply either piece of evidence; GNSS pairs and
    // all range/vision pairs must remain one family even with two observations.
    constexpr std::array<unsigned, 6U> families{0U, 1U, 1U, 2U, 2U, 2U};
    for (std::size_t first = 0U; first < families.size(); ++first)
    {
        for (std::size_t second = 0U; second < families.size(); ++second)
        {
            VerticalHealthFixture<Linalg> fixture;
            auto const inputs = fixture.inputs();
            *inputs[second] = {true, Scalar{2}, Scalar{1}};
            *inputs[first] = {true, Scalar{4}, Scalar{1}};
            bool const independent = families[first] != families[second];
            test.expect(fixture.update() == Status::success &&
                            fixture.monitor.state().likelihood ==
                                (independent ? Likelihood::high : Likelihood::medium) &&
                            fixture.monitor.state().bad_sample == independent &&
                            fixture.monitor.state().bad_vertical_acceleration == independent,
                        profile, "all ordered source pairs require distinct reference families for high likelihood");
        }
    }
    for (auto value : {Scalar{-4}, Scalar{0}, Scalar{3}, std::nextafter(Scalar{3}, Scalar{4})})
    {
        VerticalHealthFixture<Linalg> fixture;
        fixture.observations.gnss_height = {true, value, Scalar{1}};
        test.expect(fixture.update() == Status::success &&
                        fixture.monitor.state().likelihood ==
                            (value > Scalar{3} ? Likelihood::medium : Likelihood::low),
                    profile, "one-sided 3-sigma test is strict; large negative residual does not imply falling");
    }
    for (auto value : {Scalar{-2}, Scalar{1}, std::nextafter(Scalar{1}, Scalar{2})})
    {
        VerticalHealthFixture<Linalg> fixture;
        fixture.observations.barometer_height = {true, Scalar{8}, Scalar{4}};
        fixture.observations.gnss_velocity_down = {true, value, Scalar{1}};
        test.expect(fixture.update() == Status::success &&
                        fixture.monitor.state().likelihood ==
                            (value > Scalar{1} ? Likelihood::high : Likelihood::medium),
                    profile, "divide by sqrt(S); corroborating 1-sigma evidence is also strict and directional");
    }
    auto const floor = static_cast<Scalar>(std::numeric_limits<float>::epsilon());
    for (auto variance : {Scalar{0}, std::nextafter(floor, Scalar{0}), floor})
    {
        VerticalHealthFixture<Linalg> fixture;
        fixture.observations.gnss_height = {true, Scalar{1}, variance};
        test.expect(fixture.update() == Status::success &&
                        fixture.monitor.state().likelihood == (variance < floor ? Likelihood::low : Likelihood::medium),
                    profile, "the reference FLT_EPSILON floor applies to both scalar formats, including equality");
    }
}

template <typename Linalg> void test_clipping(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    // N=8: warning at 4; critical at 8; persistent clipping oscillates 8,7,8.
    constexpr std::array<std::uint16_t, 10U> counts{1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 7U, 8U};
    for (unsigned mask = 0U; mask < 8U; ++mask)
    {
        VerticalHealthFixture<Linalg> fixture;
        fixture.observations.gnss_height = {true, Scalar{4}, Scalar{1}};
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            fixture.sample.clipped[axis] = (mask & (1U << axis)) != 0U;
        }
        bool held = false;
        for (auto count : counts)
        {
            bool const bad_clipping = (mask == 7U && count >= 4U) || (mask != 0U && count == 8U);
            bool const bad_now = bad_clipping || ((mask & 4U) != 0U && count >= 4U);
            held = held || bad_now;
            test.expect(fixture.update() == Status::success && fixture.monitor.state().bad_clipping == bad_clipping &&
                            fixture.monitor.state().bad_sample == bad_now &&
                            fixture.monitor.state().bad_vertical_acceleration == held,
                        profile, "Z warning, all-axis warning and any-axis critical use distinct clipping rules");
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                test.expect(fixture.monitor.state().clip_count[axis] == (fixture.sample.clipped[axis] ? count : 0U),
                            profile, "count follows the cap/decrement branch without wraparound");
            }
            fixture.sample.time_us += 125'000U;
        }
        fixture.sample.clipped = {};
        fixture.observations = {};
        for (unsigned step = 0U; step < 10U; ++step)
        {
            auto const expected = step < 8U ? 7U - step : 0U;
            test.expect(fixture.update() == Status::success && !fixture.monitor.state().bad_sample, profile,
                        "clipping alone does not declare apparent falling without innovation evidence");
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                test.expect(fixture.monitor.state().clip_count[axis] == ((mask & (1U << axis)) != 0U ? expected : 0U),
                            profile, "clear samples decay counters to zero, never underflow");
            }
            fixture.sample.time_us += 125'000U;
        }
    }
    VerticalHealthFixture<Linalg> fixture;
    fixture.sample.clipped[0U] = true;
    for (unsigned step = 0U; step < 8U; ++step)
    {
        test.expect(fixture.update() == Status::success, profile, "seed maximum clipping history");
        ++fixture.sample.time_us;
    }
    fixture.sample.average_interval_s = Scalar{0.25}; // N drops from 8 to 4.
    test.expect(fixture.update() == Status::success && fixture.monitor.state().clip_count[0U] == 7U &&
                    fixture.monitor.state().bad_clipping,
                profile, "changed average interval does not clamp existing count to the new limit");
    fixture.monitor.reset();
    fixture.sample.average_interval_s = Scalar{0.1875}; // floor(1/dt)=5, half uses integer division: 2.
    fixture.sample.clipped = {false, false, true};
    fixture.observations.gnss_height = {true, Scalar{4}, Scalar{1}};
    test.expect(fixture.update() == Status::success && !fixture.monitor.state().bad_sample, profile, "odd count limit");
    ++fixture.sample.time_us;
    test.expect(fixture.update() == Status::success && fixture.monitor.state().bad_sample, profile,
                "reciprocal and half-limit truncate as integers");
}

template <typename Linalg> void test_recovery(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    VerticalHealthFixture<Linalg> fixture;
    auto const initial = fixture.monitor.state();
    fixture.observations.barometer_height = {true, Scalar{4}, Scalar{1}};
    fixture.observations.gnss_velocity_down = {true, Scalar{2}, Scalar{1}};
    test.expect(fixture.update() == Status::success && fixture.monitor.state().bad_vertical_acceleration &&
                    fixture.monitor.state().last_bad_time_us == 1'000'000U &&
                    fixture.monitor.state().last_good_time_us == 0U,
                profile, "high likelihood triggers immediately without clipping");
    fixture.observations = {};
    fixture.sample.time_us = 3'999'999U;
    test.expect(fixture.update() == Status::success && !fixture.monitor.state().bad_sample &&
                    fixture.monitor.state().bad_vertical_acceleration &&
                    fixture.monitor.state().last_good_time_us == 3'999'999U,
                profile, "instantaneous good time advances while the fault remains held");
    fixture.sample.time_us = 4'000'000U;
    test.expect(fixture.update() == Status::success && !fixture.monitor.state().bad_vertical_acceleration, profile,
                "fault clears at exactly three seconds, with no extra sample delay");
    fixture.observations.barometer_height = {true, Scalar{4}, Scalar{1}};
    fixture.observations.gnss_velocity_down = {true, Scalar{2}, Scalar{1}};
    fixture.sample.time_us = 5'000'000U;
    test.expect(fixture.update() == Status::success && fixture.monitor.state().bad_vertical_acceleration, profile,
                "fault can recur after recovery");
    fixture.sample.time_us = 7'000'000U;
    test.expect(fixture.update() == Status::success && fixture.monitor.state().last_bad_time_us == 7'000'000U, profile,
                "every bad sample refreshes the hold");
    fixture.observations = {};
    fixture.sample.time_us = 8'000'000U;
    test.expect(fixture.update() == Status::success && fixture.monitor.state().bad_vertical_acceleration, profile,
                "recovery is measured from the latest bad sample, not the first");
    fixture.monitor.reset();
    test.expect(same_state(fixture.monitor.state(), initial), profile, "reset removes all counters and evidence times");
    fixture.sample.time_us = std::numeric_limits<std::uint64_t>::max() - 3'000'000U;
    fixture.observations.barometer_height = {true, Scalar{4}, Scalar{1}};
    fixture.observations.gnss_height = {true, Scalar{2}, Scalar{1}};
    test.expect(fixture.update() == Status::success, profile, "timestamps near uint64 maximum are supported");
    fixture.observations = {};
    fixture.sample.time_us = std::numeric_limits<std::uint64_t>::max() - 1U;
    test.expect(fixture.update() == Status::success && fixture.monitor.state().bad_vertical_acceleration, profile,
                "elapsed-time subtraction avoids overflow");
    ++fixture.sample.time_us;
    test.expect(fixture.update() == Status::success && !fixture.monitor.state().bad_vertical_acceleration, profile,
                "recovery boundary remains correct at uint64 maximum");
}

template <typename Linalg> void test_failures(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    VerticalHealthFixture<Linalg> fixture;
    fixture.sample.clipped = {true, true, true};
    fixture.observations.barometer_height = {true, Scalar{4}, Scalar{1}};
    fixture.observations.gnss_velocity_down = {true, Scalar{2}, Scalar{1}};
    test.expect(fixture.update() == Status::success, profile, "seed nontrivial history before failure checks");
    auto const before = fixture.monitor.state();
    auto expect_failure = [&](Status expected)
    {
        test.expect(fixture.update() == expected && same_state(before, fixture.monitor.state()), profile,
                    "invalid call preserves the entire previous monitor state");
    };
    for (auto time : {std::uint64_t{0}, std::uint64_t{999'999}, std::uint64_t{1'000'000}})
    {
        fixture.sample.time_us = time;
        expect_failure(Status::out_of_range);
    }
    fixture.sample.time_us = 1'125'000U;
    fixture.sample.clipping_available = false;
    expect_failure(Status::domain_error);
    fixture.sample.clipping_available = true;
    for (auto interval : {Scalar{0}, Scalar{-1}, Scalar{1}, Scalar{1} / Scalar{65536},
                          std::numeric_limits<Scalar>::infinity(), std::numeric_limits<Scalar>::quiet_NaN()})
    {
        fixture.sample.average_interval_s = interval;
        expect_failure(!std::isfinite(interval) ? Status::non_finite_input
                       : interval <= Scalar{0}  ? Status::domain_error
                                                : Status::out_of_range);
    }
    fixture.sample.average_interval_s = Scalar{0.125};
    auto const original = fixture.observations;
    for (std::size_t index = 0U; index < 6U; ++index)
    {
        for (auto value : {std::numeric_limits<Scalar>::infinity(), -std::numeric_limits<Scalar>::infinity(),
                           std::numeric_limits<Scalar>::quiet_NaN()})
        {
            fixture.observations = original;
            *fixture.inputs()[index] = {true, value, Scalar{1}};
            expect_failure(Status::non_finite_input);
            *fixture.inputs()[index] = {true, Scalar{1}, value};
            expect_failure(Status::non_finite_input);
        }
        fixture.observations = original;
        *fixture.inputs()[index] = {true, Scalar{1}, Scalar{-1}};
        expect_failure(Status::domain_error);
    }
    fixture.observations = original;
    fixture.observations.vision_height = {true, std::numeric_limits<Scalar>::max(),
                                          static_cast<Scalar>(std::numeric_limits<float>::epsilon())};
    expect_failure(Status::non_finite_result);
    for (auto input : fixture.inputs())
    {
        *input = {false, std::numeric_limits<Scalar>::quiet_NaN(), Scalar{-1}};
    }
    test.expect(fixture.update() == Status::success && !fixture.monitor.state().bad_sample &&
                    fixture.monitor.state().bad_vertical_acceleration,
                profile, "inactive fields are ignored; failed calls do not consume the next valid timestamp");
}

template <typename Linalg> void test_core_residual(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    using Vector = typename Linalg::template vector_type<3U>;
    using Covariance = typename Linalg::template matrix_type<15U, 15U>;
    using Matrix3 = typename Linalg::template matrix_type<3U, 3U>;
    formal_eskf::configuration::Ins::NominalState<Linalg> state;
    state.v_n.set(2U, Scalar{8}); // INS thinks it is falling; GNSS observes zero Down speed.
    auto covariance = Covariance::identity();
    auto const result = formal_eskf::runtime::fuse_velocity(state, covariance, Vector{}, Matrix3::identity(),
                                                            Scalar{0.125}, Scalar{1}, state, covariance);
    test.expect(result.status == Status::success && result.diagnostics_valid &&
                    result.decision == formal_eskf::runtime::FusionDecision::rejected &&
                    result.innovation(2U) == Scalar{-8},
                profile, "a gate rejection still provides signed diagnostic evidence");
    VerticalHealthFixture<Linalg> fixture;
    fixture.observations.gnss_velocity_down = {true, -result.innovation(2U), result.innovation_variance(2U)};
    fixture.observations.barometer_height = {true, Scalar{2}, Scalar{1}};
    test.expect(fixture.update() == Status::success && fixture.monitor.state().bad_sample, profile,
                "negating the production residual connects its convention to falling detection");
}

template <typename Linalg> void run_conformance(TestContext & test, std::string_view profile)
{
    test_likelihood<Linalg>(test, profile);
    test_clipping<Linalg>(test, profile);
    test_recovery<Linalg>(test, profile);
    test_failures<Linalg>(test, profile);
    test_core_residual<Linalg>(test, profile);
}

} /* end namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    TestContext test;
    run_conformance<formal_eskf::test::Backend<float>>(test, "float");
    run_conformance<formal_eskf::test::Backend<double>>(test, "double");
    if (test.failures() != 0)
    {
        return 1;
    }
    std::cout << "Vertical acceleration health: pass\n";
    return 0;
}
