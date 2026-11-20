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

#include <formal_eskf/runtime/navigation.hpp>
#include <formal_eskf/runtime/vertical_acceleration_health.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{

using formal_eskf::Status;
using TestContext = formal_eskf::test::Context;

enum class Reference
{
    pressure,
    gnss,
    ground,
};
constexpr std::array<Reference, 6U> references{Reference::pressure, Reference::gnss,   Reference::gnss,
                                               Reference::ground,   Reference::ground, Reference::ground};

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
    for (std::size_t first = 0U; first < references.size(); ++first)
    {
        for (std::size_t second = 0U; second < references.size(); ++second)
        {
            VerticalHealthFixture<Linalg> fixture;
            auto const inputs = fixture.inputs();
            *inputs[second] = {true, Scalar{2}, Scalar{1}};
            *inputs[first] = {true, Scalar{4}, Scalar{1}};
            bool const independent = references[first] != references[second];
            test.expect(fixture.update() == Status::success &&
                            fixture.monitor.state().likelihood ==
                                (independent ? Likelihood::high : Likelihood::medium) &&
                            fixture.monitor.state().bad_sample == independent &&
                            fixture.monitor.state().bad_vertical_acceleration == independent,
                        profile, "all ordered source pairs require distinct reference families for high likelihood");
        }
    }
    for (auto value :
         {Scalar{-4}, Scalar{0}, std::nextafter(Scalar{3}, Scalar{0}), Scalar{3}, std::nextafter(Scalar{3}, Scalar{4})})
    {
        VerticalHealthFixture<Linalg> fixture;
        fixture.observations.gnss_height = {true, value, Scalar{1}};
        test.expect(fixture.update() == Status::success &&
                        fixture.monitor.state().likelihood ==
                            (value > Scalar{3} ? Likelihood::medium : Likelihood::low),
                    profile, "one-sided 3-sigma test is strict; large negative residual does not imply falling");
    }
    for (auto value :
         {Scalar{-2}, std::nextafter(Scalar{1}, Scalar{0}), Scalar{1}, std::nextafter(Scalar{1}, Scalar{2})})
    {
        VerticalHealthFixture<Linalg> fixture;
        fixture.observations.barometer_height = {true, Scalar{8}, Scalar{4}};
        fixture.observations.gnss_velocity_down = {true, value, Scalar{1}};
        test.expect(fixture.update() == Status::success &&
                        fixture.monitor.state().likelihood ==
                            (value > Scalar{1} ? Likelihood::high : Likelihood::medium),
                    profile, "divide by sqrt(S); corroborating 1-sigma evidence is also strict and directional");
    }
}

template <typename Linalg> void test_combined_evidence(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    using Monitor = typename VerticalHealthFixture<Linalg>::monitor_type;
    using Likelihood = typename Monitor::Likelihood;
    struct EvidenceCase
    {
        typename Monitor::Innovation observation{};
        bool above_minimum = false;
        bool above_limit = false;
    };
    std::array<EvidenceCase, 4U> const cases{
        {{{false, std::numeric_limits<Scalar>::quiet_NaN(), Scalar{-1}}, false, false},
         {{true, Scalar{-4}, Scalar{1}}, false, false},
         {{true, Scalar{2}, Scalar{1}}, true, false},
         {{true, Scalar{4}, Scalar{1}}, true, true}}};
    // All 4^6 assignments: unlike the production grouping, the oracle compares
    // individual source pairs to find independent corroboration.
    for (std::size_t assignment = 0U; assignment < 4096U; ++assignment)
    {
        VerticalHealthFixture<Linalg> fixture;
        auto const inputs = fixture.inputs();
        std::array<std::size_t, 6U> selected{};
        auto remaining = assignment;
        for (std::size_t source = 0U; source < inputs.size(); ++source)
        {
            selected[source] = remaining % cases.size();
            remaining /= cases.size();
            *inputs[source] = cases[selected[source]].observation;
        }
        bool medium = false;
        bool high = false;
        for (std::size_t first = 0U; first < inputs.size(); ++first)
        {
            medium = medium || cases[selected[first]].above_limit;
            for (std::size_t second = 0U; second < inputs.size(); ++second)
            {
                high = high || (references[first] != references[second] && cases[selected[first]].above_limit &&
                                cases[selected[second]].above_minimum);
            }
        }
        auto const expected = high ? Likelihood::high : medium ? Likelihood::medium : Likelihood::low;
        test.expect(fixture.update() == Status::success && fixture.monitor.state().likelihood == expected &&
                        fixture.monitor.state().bad_sample == high,
                    profile, "all source combinations retain independent-reference and inactive-input semantics");
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
    fixture.sample.time_us = 4'000'001U;
    test.expect(fixture.update() == Status::success && !fixture.monitor.state().bad_vertical_acceleration &&
                    fixture.monitor.state().last_good_time_us == 4'000'001U,
                profile, "one microsecond after release remains current good evidence");
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

template <typename Linalg> void clock_failures(TestContext & test, std::string_view profile)
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
    test.expect(fixture.update() == Status::success, profile, "valid retry reuses the previously rejected timestamp");
}

template <typename Linalg> void source_failures(TestContext & test, std::string_view profile)
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
    fixture.sample.time_us = 1'125'000U;
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
    for (std::size_t index = 0; index < 6; ++index)
    {
        fixture.observations = original;
        *fixture.inputs()[index] = {true, std::numeric_limits<Scalar>::max(),
                                    static_cast<Scalar>(std::numeric_limits<float>::epsilon())};
        expect_failure(Status::non_finite_result);
    }
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
    using Vector = formal_eskf::linalg::Vector<Linalg, 3U>;
    using Covariance = formal_eskf::linalg::Matrix<Linalg, 15U, 15U>;
    using Matrix3 = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    formal_eskf::configuration::Ins::NominalState<Linalg> state;
    state.v_n(2U) = Scalar{8}; // INS thinks it is falling; GNSS observes zero Down speed.
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

template <typename Linalg> void test_failure_order(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    using Monitor = typename VerticalHealthFixture<Linalg>::monitor_type;
    typename Monitor::Innovation const negative_variance{true, Scalar{1}, Scalar{-1}};
    typename Monitor::Innovation const non_finite{true, std::numeric_limits<Scalar>::quiet_NaN(), Scalar{1}};
    VerticalHealthFixture<Linalg> fixture;
    fixture.observations.barometer_height = {true, Scalar{4}, Scalar{1}};
    fixture.observations.gnss_height = {true, Scalar{2}, Scalar{1}};
    fixture.sample.clipped = {true, true, true};
    test.expect(fixture.update() == Status::success, profile, "seed history for competing failures");
    auto const before = fixture.monitor.state();
    ++fixture.sample.time_us;
    auto const inputs = fixture.inputs();
    for (std::size_t first = 0U; first < inputs.size(); ++first)
    {
        for (std::size_t second = first + 1U; second < inputs.size(); ++second)
        {
            for (bool negative_variance_first : {false, true})
            {
                fixture.observations = {};
                *inputs[first] = negative_variance_first ? negative_variance : non_finite;
                *inputs[second] = negative_variance_first ? non_finite : negative_variance;
                auto const expected = negative_variance_first ? Status::domain_error : Status::non_finite_input;
                test.expect(fixture.update() == expected && same_state(before, fixture.monitor.state()), profile,
                            "first invalid source determines status, with all history preserved");
            }
        }
    }
    fixture.observations = {};
    fixture.observations.barometer_height = {true, Scalar{4}, Scalar{1}};
    fixture.observations.gnss_height = {true, Scalar{2}, Scalar{1}};
    fixture.observations.vision_velocity_down = {true, Scalar{0}, Scalar{-1}};
    test.expect(fixture.update() == Status::domain_error && same_state(before, fixture.monitor.state()), profile,
                "high likelihood from early sources cannot skip validation of the final source");
    fixture.sample.clipping_available = false;
    fixture.sample.average_interval_s = std::numeric_limits<Scalar>::quiet_NaN();
    test.expect(fixture.update() == Status::non_finite_input && same_state(before, fixture.monitor.state()), profile,
                "interval validation precedes unknown clipping and invalid observations");
    fixture.sample.time_us = before.last_update_time_us;
    test.expect(fixture.update() == Status::out_of_range && same_state(before, fixture.monitor.state()), profile,
                "time validation precedes all consumed values");
}

template <typename Linalg> void initialization(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    using Monitor = typename VerticalHealthFixture<Linalg>::monitor_type;
    using Likelihood = typename Monitor::Likelihood;
    Monitor monitor;
    typename Monitor::State const empty{};
    test.expect(same_state(monitor.state(), empty), profile, "initial state has no counters, verdict or timestamps");
    for (std::size_t source = 0; source < 6; ++source)
    {
        for (auto innovation : {Scalar{-4}, Scalar{4}})
        {
            VerticalHealthFixture<Linalg> fixture;
            for (auto input : fixture.inputs())
            {
                *input = {false, std::numeric_limits<Scalar>::quiet_NaN(), Scalar{-1}};
            }
            *fixture.inputs()[source] = {true, innovation, Scalar{1}};
            test.expect(fixture.update() == Status::success &&
                            fixture.monitor.state().likelihood ==
                                (innovation > Scalar{0} ? Likelihood::medium : Likelihood::low),
                        profile, "each signed source works with arbitrary invalid inactive slots");
            auto const stored = fixture.monitor.state();
            test.expect(same_state(fixture.monitor.state(), stored), profile,
                        "state queries do not advance clocks or counters");
        }
    }
    auto const floor = static_cast<Scalar>(std::numeric_limits<float>::epsilon());
    for (auto variance : {Scalar{0}, std::nextafter(floor, Scalar{0}), floor, std::nextafter(floor, Scalar{1})})
    {
        VerticalHealthFixture<Linalg> fixture;
        fixture.observations.gnss_height = {true, Scalar{1}, variance};
        test.expect(fixture.update() == Status::success &&
                        fixture.monitor.state().likelihood == (variance < floor ? Likelihood::low : Likelihood::medium),
                    profile, "the reference FLT_EPSILON floor applies to both scalar formats, including equality");
    }
    VerticalHealthFixture<Linalg> none;
    test.expect(none.update() == Status::success && none.monitor.state().likelihood == Likelihood::low &&
                    none.monitor.state().last_good_time_us == none.sample.time_us,
                profile, "no active source gives low evidence; this is not proof of physical health");
    test_core_residual<Linalg>(test, profile);
}

template <typename Linalg> void source_grouping(TestContext & test, std::string_view profile)
{
    test_likelihood<Linalg>(test, profile);
    test_combined_evidence<Linalg>(test, profile);
}

template <typename Linalg> void clipping(TestContext & test, std::string_view profile)
{
    test_clipping<Linalg>(test, profile);
    using Scalar = typename Linalg::value_type;
    VerticalHealthFixture<Linalg> fixture;
    fixture.sample.average_interval_s = Scalar{0.5};
    fixture.sample.clipped = {false, false, true};
    fixture.observations.gnss_height = {true, Scalar{4}, Scalar{1}};
    for (std::uint16_t expected : {std::uint16_t{1}, std::uint16_t{2}, std::uint16_t{1}, std::uint16_t{2}})
    {
        test.expect(fixture.update() == Status::success && fixture.monitor.state().clip_count[2U] == expected &&
                        fixture.monitor.state().bad_sample && fixture.monitor.state().bad_clipping == (expected == 2),
                    profile, "minimum N=2 gives warning at1 and decrements at saturated2 even while clipped");
        ++fixture.sample.time_us;
    }
}

template <typename Linalg> void clocks(TestContext & test, std::string_view profile)
{
    clock_failures<Linalg>(test, profile);
    using Scalar = typename Linalg::value_type;
    Scalar const upper_interval = Scalar{0.5};
    Scalar const lower_interval = Scalar{1} / Scalar{65536};
    for (auto interval :
         {std::nextafter(upper_interval, Scalar{0}), upper_interval, std::nextafter(upper_interval, Scalar{1}),
          std::nextafter(lower_interval, Scalar{0}), lower_interval, std::nextafter(lower_interval, Scalar{1}),
          std::numeric_limits<Scalar>::denorm_min()})
    {
        VerticalHealthFixture<Linalg> fixture;
        fixture.sample.average_interval_s = interval;
        Scalar const reciprocal = Scalar{1} / interval;
        Status const expected = !std::isfinite(reciprocal)                              ? Status::non_finite_result
                                : reciprocal >= Scalar{2} && reciprocal < Scalar{65536} ? Status::success
                                                                                        : Status::out_of_range;
        auto const before = fixture.monitor.state();
        test.expect(fixture.update() == expected, profile,
                    "reciprocal finite/range boundary keeps uint16 conversion defined");
        if (expected != Status::success)
        {
            test.expect(same_state(fixture.monitor.state(), before), profile,
                        "reciprocal failure preserves entire history");
            fixture.sample.average_interval_s = Scalar{0.125};
            test.expect(fixture.update() == Status::success, profile,
                        "failed timestamp remains available for corrected retry");
        }
    }
    VerticalHealthFixture<Linalg> fixture;
    fixture.sample.time_us = 0;
    fixture.sample.average_interval_s = std::numeric_limits<Scalar>::quiet_NaN();
    fixture.sample.clipping_available = false;
    fixture.observations.barometer_height = {true, Scalar{0}, Scalar{-1}};
    test.expect(fixture.update() == Status::out_of_range, profile,
                "time errors precede interval, metadata and source errors");
    fixture.sample.time_us = 1;
    test.expect(fixture.update() == Status::non_finite_input, profile,
                "finite interval validation precedes unavailable clipping");
    fixture.sample.average_interval_s = Scalar{0.125};
    test.expect(fixture.update() == Status::domain_error, profile,
                "unavailable clipping is not converted to known clear");
}

template <typename Linalg> void active_source_failures(TestContext & test, std::string_view profile)
{
    source_failures<Linalg>(test, profile);
    test_failure_order<Linalg>(test, profile);
    // Native StandardMath sqrt receives only finite S>=FLT_EPSILON here;
    // injected sqrt-provider failures remain the scalar-contract/formal profile,
    // not a native status reached by fabricating a negative or inactive source.
}

template <typename Linalg> void cached_and_reset(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    using Monitor = typename VerticalHealthFixture<Linalg>::monitor_type;
    VerticalHealthFixture<Linalg> fixture;
    fixture.observations.barometer_height = {true, Scalar{4}, Scalar{1}};
    fixture.observations.gnss_height = {true, Scalar{2}, Scalar{1}};
    for (auto time : {1'000'000ULL, 2'000'000ULL, 20'000'000ULL})
    {
        fixture.sample.time_us = time;
        test.expect(fixture.update() == Status::success && fixture.monitor.state().bad_sample &&
                        fixture.monitor.state().last_bad_time_us == time,
                    profile, "cached active diagnostic stays evidence; no monitor-owned source freshness deadline");
    }
    auto const stored = fixture.monitor.state();
    fixture.sample.time_us += 1;
    fixture.sample.clipping_available = false;
    test.expect(fixture.update() == Status::domain_error && same_state(fixture.monitor.state(), stored), profile,
                "invalid current metadata leaves old history but does not label it a current verdict");
    // Caller chooses to reset at a gap, missing metadata, source or frame interruption.
    fixture.monitor.reset();
    typename Monitor::State const empty{};
    test.expect(same_state(fixture.monitor.state(), empty), profile,
                "explicit reset clears every public history field");
    fixture.observations = {};
    fixture.sample.time_us = 1;
    fixture.sample.clipping_available = true;
    fixture.sample.clipped = {false, false, true};
    test.expect(fixture.update() == Status::success && !fixture.monitor.state().bad_vertical_acceleration &&
                    fixture.monitor.state().clip_count == std::array<std::uint16_t, 3>{0, 0, 1} &&
                    fixture.monitor.state().last_bad_time_us == 0 && fixture.monitor.state().last_good_time_us == 1,
                profile, "new session accepts a new clock origin without inheriting old counters or held fault");
}

} /* end namespace */
int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Case;
    using Float = formal_eskf::test::Backend<float>;
    using Double = formal_eskf::test::Backend<double>;
    Case const cases[] = {
        {"TC-R-VERTICAL-ACCELERATION-01", "float", initialization<Float>},
        {"TC-R-VERTICAL-ACCELERATION-01", "double", initialization<Double>},
        {"TC-R-VERTICAL-ACCELERATION-02", "float", source_grouping<Float>},
        {"TC-R-VERTICAL-ACCELERATION-02", "double", source_grouping<Double>},
        {"TC-R-VERTICAL-ACCELERATION-03", "float", clipping<Float>},
        {"TC-R-VERTICAL-ACCELERATION-03", "double", clipping<Double>},
        {"TC-R-VERTICAL-ACCELERATION-04", "float", test_recovery<Float>},
        {"TC-R-VERTICAL-ACCELERATION-04", "double", test_recovery<Double>},
        {"TC-R-VERTICAL-ACCELERATION-05", "float", clocks<Float>},
        {"TC-R-VERTICAL-ACCELERATION-05", "double", clocks<Double>},
        {"TC-R-VERTICAL-ACCELERATION-06", "float", active_source_failures<Float>},
        {"TC-R-VERTICAL-ACCELERATION-06", "double", active_source_failures<Double>},
        {"TC-R-VERTICAL-ACCELERATION-07", "float", cached_and_reset<Float>},
        {"TC-R-VERTICAL-ACCELERATION-07", "double", cached_and_reset<Double>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
