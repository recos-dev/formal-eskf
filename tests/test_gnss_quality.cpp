/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/gnss_quality.hpp>

#include <cmath>
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
using formal_eskf::runtime::GnssQualityChecks;
using formal_eskf::runtime::GnssQualityDecision;
using formal_eskf::runtime::GnssQualityMotion;
using TestContext = formal_eskf::test::Context;

bool same_checks(GnssQualityChecks const & a, GnssQualityChecks const & b)
{
    return a.satellites == b.satellites && a.pdop == b.pdop && a.horizontal_accuracy == b.horizontal_accuracy &&
           a.vertical_accuracy == b.vertical_accuracy && a.speed_accuracy == b.speed_accuracy &&
           a.horizontal_drift == b.horizontal_drift && a.vertical_drift == b.vertical_drift &&
           a.horizontal_speed == b.horizontal_speed && a.vertical_speed == b.vertical_speed && a.spoofed == b.spoofed;
}

template <typename Scalar>
bool same_result(formal_eskf::runtime::GnssQualityResult<Scalar> const & a,
                 formal_eskf::runtime::GnssQualityResult<Scalar> const & b)
{
    return same_checks(a.failures, b.failures) && a.status == b.status && a.decision == b.decision &&
           a.invalid_fix == b.invalid_fix && a.qualified == b.qualified && a.drift_valid == b.drift_valid &&
           a.horizontal_drift == b.horizontal_drift && a.vertical_drift == b.vertical_drift &&
           a.horizontal_speed == b.horizontal_speed && a.vertical_speed == b.vertical_speed &&
           a.last_fail_us == b.last_fail_us && a.last_pass_us == b.last_pass_us;
}

template <typename Linalg> struct Fixture
{
    using monitor_type = formal_eskf::runtime::GnssQuality<Linalg>;
    using value_type = typename Linalg::value_type;
    monitor_type monitor;
    typename monitor_type::parameters_type parameters;
    typename monitor_type::sample_type sample;
    typename monitor_type::result_type result;

    Fixture()
    {
        sample.time_us = 1000000U;
        sample.fix_type = 3U;
        sample.satellites = parameters.minimum_satellites;
        sample.pdop = parameters.maximum_pdop;
        sample.horizontal_accuracy = parameters.maximum_horizontal_accuracy;
        sample.vertical_accuracy = parameters.maximum_vertical_accuracy;
        sample.speed_accuracy = parameters.maximum_speed_accuracy;
    }

    Status update(GnssQualityMotion motion = {}, std::uint64_t now_us = 0U)
    {
        result = monitor.update(sample, motion, now_us == 0U ? sample.time_us : now_us, parameters);
        if (result.decision != GnssQualityDecision::invalid_call)
        {
            sample.position_reference_time_us = sample.time_us;
        }
        return result.status;
    }
};

template <typename Linalg> void run_conformance(TestContext & context, std::string_view profile)
{
    using fixture_type = Fixture<Linalg>;
    using value_type = typename Linalg::value_type;
    using limits_type = std::numeric_limits<value_type>;
    constexpr GnssQualityMotion stationary{.in_air = false, .vehicle_at_rest = true};
    constexpr GnssQualityMotion moving{};
    constexpr GnssQualityMotion flying{.in_air = true, .vehicle_at_rest = false};
    auto const tolerance = value_type{32} * limits_type::epsilon();

    fixture_type healthy;
    context.expect(healthy.update() == Status::success && same_checks(healthy.result.failures, {}) &&
                       !healthy.result.invalid_fix && healthy.result.decision == GnssQualityDecision::waiting &&
                       !healthy.result.qualified,
                   profile, "inclusive metadata boundaries pass, but first fix is not qualified");

    fixture_type bad_fix;
    bad_fix.sample.fix_type = 2U;
    bad_fix.parameters.enabled_checks = {};
    context.expect(bad_fix.update() == Status::success && bad_fix.result.invalid_fix &&
                       bad_fix.result.decision == GnssQualityDecision::rejected,
                   profile, "fix is mandatory even with all configurable checks disabled");

    auto check_metadata = [&](auto mutate, bool GnssQualityChecks::*check)
    {
        fixture_type fixture;
        GnssQualityChecks expected;
        expected.*check = true;
        mutate(fixture.sample);
        context.expect(fixture.update() == Status::success && same_checks(fixture.result.failures, expected) &&
                           fixture.result.decision == GnssQualityDecision::rejected,
                       profile, "individual metadata failure has its own named diagnostic");
        fixture.monitor.reset();
        fixture.parameters.enabled_checks = {};
        context.expect(fixture.update() == Status::success && same_checks(fixture.result.failures, expected) &&
                           fixture.result.decision == GnssQualityDecision::waiting,
                       profile, "disabled check still reports its diagnostic");
        fixture.monitor.reset();
        fixture.parameters.enabled_checks.*check = true;
        context.expect(fixture.update() == Status::success && fixture.result.decision == GnssQualityDecision::rejected,
                       profile, "individually enabled check rejects sample");
    };
    check_metadata([](auto & sample) { --sample.satellites; }, &GnssQualityChecks::satellites);
    check_metadata([](auto & sample) { sample.pdop = std::nextafter(sample.pdop, limits_type::infinity()); },
                   &GnssQualityChecks::pdop);
    check_metadata(
        [](auto & sample)
        { sample.horizontal_accuracy = std::nextafter(sample.horizontal_accuracy, limits_type::infinity()); },
        &GnssQualityChecks::horizontal_accuracy);
    check_metadata([](auto & sample)
                   { sample.vertical_accuracy = std::nextafter(sample.vertical_accuracy, limits_type::infinity()); },
                   &GnssQualityChecks::vertical_accuracy);
    check_metadata([](auto & sample)
                   { sample.speed_accuracy = std::nextafter(sample.speed_accuracy, limits_type::infinity()); },
                   &GnssQualityChecks::speed_accuracy);
    check_metadata([](auto & sample) { sample.spoofed = true; }, &GnssQualityChecks::spoofed);

    fixture_type drift;
    drift.parameters.maximum_horizontal_drift = value_type{0.5};
    drift.parameters.maximum_vertical_drift = value_type{1};
    drift.sample.time_us = 10000000U;
    drift.sample.position_delta_n.set(0U, limits_type::quiet_NaN());
    context.expect(drift.update(stationary) == Status::success &&
                       drift.result.decision == GnssQualityDecision::waiting && drift.result.drift_valid &&
                       drift.result.horizontal_drift == value_type{0},
                   profile, "first fix ignores unavailable position displacement");
    drift.sample.time_us += 10000000U;
    drift.sample.position_delta_n.set(0U, value_type{5});
    drift.sample.position_delta_n.set(2U, value_type{-10});
    context.expect(drift.update(stationary) == Status::success &&
                       drift.result.decision == GnssQualityDecision::waiting &&
                       drift.result.horizontal_drift == value_type{0.5} && drift.result.vertical_drift == value_type{1},
                   profile, "stationary drift threshold equality passes");
    drift.sample.time_us += 10000000U;
    drift.sample.position_delta_n.set(0U, value_type{10});
    drift.sample.position_delta_n.set(2U, value_type{-20});
    context.expect(drift.update(stationary) == Status::success && drift.result.failures.horizontal_drift &&
                       drift.result.failures.vertical_drift,
                   profile, "horizontal and vertical stationary drift failures");
    drift.sample.time_us += 100000U;
    context.expect(drift.update(moving) == Status::success && !drift.result.drift_valid &&
                       drift.result.failures.horizontal_drift && drift.result.failures.vertical_drift,
                   profile, "ground movement retains failure flags, metrics are unavailable");
    drift.sample.time_us += 100000U;
    context.expect(drift.update(flying) == Status::success && !drift.result.drift_valid &&
                       drift.result.decision == GnssQualityDecision::waiting,
                   profile, "flight clears stationary failures");
    drift.sample.time_us += 100000U;
    drift.sample.position_delta_n = decltype(drift.sample.position_delta_n)::zero();
    context.expect(drift.update(stationary) == Status::success && drift.result.horizontal_drift == value_type{0},
                   profile, "returning to rest uses reset filters");

    fixture_type filter;
    context.expect(filter.update(stationary) == Status::success, profile, "initialize filter sequence");
    filter.sample.time_us += 100000U;
    filter.sample.position_delta_n.set(0U, value_type{4});
    filter.sample.velocity_n.set(0U, value_type{10});
    context.expect(
        filter.update(stationary) == Status::success &&
            formal_eskf::test::near(filter.result.horizontal_drift, static_cast<value_type>(0.4), tolerance) &&
            formal_eskf::test::near(filter.result.horizontal_speed, static_cast<value_type>(0.01), tolerance),
        profile, "position state clamp and velocity input clamp are not interchangeable");
    filter.sample.time_us += 100000U;
    filter.sample.position_delta_n.set(0U, value_type{-4});
    filter.sample.velocity_n.set(0U, value_type{0});
    context.expect(
        filter.update(stationary) == Status::success &&
            formal_eskf::test::near(filter.result.horizontal_drift, static_cast<value_type>(0.004), tolerance),
        profile, "signed filter history retained before drift magnitude");
    filter.sample.time_us += 1U;
    filter.sample.position_delta_n.set(0U, value_type{0});
    context.expect(filter.update(stationary) == Status::success &&
                       formal_eskf::test::near(filter.result.horizontal_drift,
                                               static_cast<value_type>(0.004) * static_cast<value_type>(0.9999),
                                               tolerance),
                   profile, "sample dt lower clamp is 1 ms");
    filter.sample.time_us += 20000000U;
    filter.sample.position_delta_n.set(0U, value_type{1000});
    context.expect(filter.update(stationary) == Status::success && filter.result.horizontal_drift == value_type{1},
                   profile, "dt upper clamp 10 s and ten-times-threshold state anti-windup");

    fixture_type speed;
    speed.sample.time_us = 10000000U;
    speed.sample.velocity_n.set(0U, speed.parameters.maximum_horizontal_drift);
    speed.sample.velocity_n.set(2U, -speed.parameters.maximum_vertical_drift);
    context.expect(speed.update(stationary) == Status::success && speed.result.decision == GnssQualityDecision::waiting,
                   profile, "stationary speed threshold equality passes");
    speed.sample.time_us += 10000000U;
    speed.sample.velocity_n.set(1U, speed.parameters.maximum_horizontal_drift);
    speed.sample.velocity_n.set(2U, -value_type{1});
    context.expect(speed.update(stationary) == Status::success && speed.result.failures.horizontal_speed &&
                       speed.result.failures.vertical_speed,
                   profile, "stationary speed uses horizontal norm and absolute down velocity");
    speed.sample.time_us += 1U;
    speed.sample.velocity_n.set(0U, value_type{60});
    speed.sample.velocity_n.set(1U, value_type{80});
    speed.sample.velocity_n.set(2U, value_type{-100});
    context.expect(speed.update(flying) == Status::success && speed.result.decision == GnssQualityDecision::waiting,
                   profile, "raw 100 m/s boundary passes in flight");
    speed.sample.time_us += 1U;
    speed.sample.velocity_n.set(0U, value_type{80});
    speed.sample.velocity_n.set(2U, value_type{-101});
    context.expect(speed.update(flying) == Status::success && speed.result.failures.horizontal_speed &&
                       speed.result.failures.vertical_speed,
                   profile, "raw speed limits force failure even in flight");
    speed.parameters.enabled_checks = {};
    speed.monitor.reset();
    context.expect(speed.update(flying) == Status::success && speed.result.decision == GnssQualityDecision::waiting &&
                       speed.result.failures.horizontal_speed && speed.result.failures.vertical_speed,
                   profile, "disabled speed checks retain raw-speed diagnostics");

    for (auto const check : {&GnssQualityChecks::horizontal_drift, &GnssQualityChecks::vertical_drift,
                             &GnssQualityChecks::horizontal_speed, &GnssQualityChecks::vertical_speed})
    {
        fixture_type masked;
        masked.parameters.enabled_checks = {};
        masked.parameters.enabled_checks.*check = true;
        masked.sample.time_us = 10000000U;
        context.expect(masked.update(stationary) == Status::success, profile, "prime stationary mask test");
        masked.sample.time_us += 10000000U;
        auto const axis =
            check == &GnssQualityChecks::vertical_drift || check == &GnssQualityChecks::vertical_speed ? 2U : 0U;
        if (check == &GnssQualityChecks::horizontal_drift || check == &GnssQualityChecks::vertical_drift)
        {
            masked.sample.position_delta_n.set(axis, value_type{20});
        }
        else
        {
            masked.sample.velocity_n.set(axis, value_type{2});
        }
        context.expect(masked.update(stationary) == Status::success &&
                           same_checks(masked.result.failures, masked.parameters.enabled_checks) &&
                           masked.result.decision == GnssQualityDecision::rejected,
                       profile, "each selected stationary check independently rejects its own failure");
        masked.parameters.enabled_checks = {};
        masked.monitor.reset();
        // The first fix ignores displacement; repeat it once before testing disabled drift.
        context.expect(masked.update(stationary) == Status::success, profile, "prime disabled stationary checks");
        masked.sample.time_us += 10000000U;
        context.expect(masked.update(stationary) == Status::success && masked.result.failures.*check &&
                           masked.result.decision == GnssQualityDecision::waiting,
                       profile, "disabled stationary checks do not reject sample");
    }

    fixture_type timing;
    context.expect(timing.update() == Status::success, profile, "initialize health wait");
    timing.sample.time_us = 2000000U;
    context.expect(timing.update() == Status::success && timing.result.decision != GnssQualityDecision::accepted,
                   profile, "recovery wait is strict");
    timing.sample.time_us += 1U;
    context.expect(timing.update() == Status::success && timing.result.decision == GnssQualityDecision::waiting &&
                       !timing.result.qualified,
                   profile, "one-second recovery does not replace ten-second initial qualification");
    timing.sample.time_us = 11000000U;
    context.expect(timing.update() == Status::success && !timing.result.qualified, profile,
                   "health boundary is strict");
    timing.sample.time_us += 1U;
    context.expect(timing.update() == Status::success && timing.result.qualified &&
                       timing.result.decision == GnssQualityDecision::accepted,
                   profile, "initial health latches after more than ten seconds");
    timing.sample.time_us = 12000000U;
    timing.sample.fix_type = 2U;
    context.expect(timing.update() == Status::success && timing.result.qualified &&
                       timing.result.decision == GnssQualityDecision::rejected &&
                       timing.result.last_fail_us == 12000000U && timing.result.last_pass_us == 11000001U,
                   profile, "failure records time and blocks sample without clearing historical latch");
    timing.sample.time_us = 13000000U;
    timing.sample.fix_type = 3U;
    context.expect(timing.update() == Status::success && timing.result.decision != GnssQualityDecision::accepted,
                   profile, "recovery boundary after failure");
    timing.sample.time_us += 1U;
    context.expect(timing.update() == Status::success && timing.result.decision == GnssQualityDecision::accepted &&
                       timing.result.qualified,
                   profile, "recovered sample need not redo initial qualification");
    timing.monitor.reset();
    context.expect(timing.update() == Status::success && !timing.result.qualified &&
                       timing.result.decision != GnssQualityDecision::accepted,
                   profile, "explicit reset clears qualification and histories");

    fixture_type long_health;
    long_health.parameters.minimum_health_time_us = 20000000U;
    context.expect(long_health.update() == Status::success, profile, "configurable health interval");
    long_health.sample.time_us += 20000001U;
    context.expect(long_health.update() == Status::success &&
                       long_health.result.decision == GnssQualityDecision::accepted,
                   profile, "configured initial qualification expires");
    long_health.sample.time_us += 1U;
    long_health.sample.fix_type = 2U;
    context.expect(long_health.update() == Status::success &&
                       long_health.result.decision == GnssQualityDecision::rejected,
                   profile, "start configured recovery after qualification");
    long_health.sample.fix_type = 3U;
    long_health.sample.time_us += 2000000U;
    context.expect(long_health.update() == Status::success &&
                       long_health.result.decision != GnssQualityDecision::accepted,
                   profile, "recovery is health/10 above one second");
    long_health.sample.time_us += 1U;
    context.expect(long_health.update() == Status::success &&
                       long_health.result.decision == GnssQualityDecision::accepted,
                   profile, "configured recovery expires");

    fixture_type short_health;
    short_health.parameters.minimum_health_time_us = 1U;
    context.expect(short_health.update() == Status::success, profile, "short health interval");
    short_health.sample.time_us += 1000000U;
    context.expect(short_health.update() == Status::success &&
                       short_health.result.decision == GnssQualityDecision::waiting,
                   profile, "short health setting does not bypass one-second recovery floor");
    short_health.sample.time_us += 1U;
    context.expect(short_health.update() == Status::success &&
                       short_health.result.decision == GnssQualityDecision::accepted,
                   profile, "strict one-second floor expires");

    auto check_bad_data = [&](auto mutate, Status status)
    {
        fixture_type fixture;
        fixture.parameters.enabled_checks = {};
        context.expect(fixture.update(stationary) == Status::success, profile, "prime invalid-data sequence");
        auto valid = fixture.sample;
        fixture.sample.time_us += 11000000U;
        mutate(fixture.sample);
        context.expect(fixture.update(stationary) == status &&
                           fixture.result.decision == GnssQualityDecision::rejected && !fixture.result.qualified &&
                           fixture.result.last_fail_us == fixture.sample.time_us,
                       profile, "bad data cannot be masked or qualify health");
        valid.time_us = fixture.sample.time_us + 1U;
        fixture.sample = valid;
        fixture.sample.position_delta_n.set(0U, limits_type::quiet_NaN());
        context.expect(fixture.update(stationary) == Status::success &&
                           fixture.result.decision == GnssQualityDecision::waiting &&
                           fixture.result.horizontal_drift == value_type{0},
                       profile, "bad data reset position continuity and restart recovery wait");
    };
    for (auto const invalid : {limits_type::quiet_NaN(), limits_type::infinity(), -limits_type::infinity()})
    {
        check_bad_data([&](auto & sample) { sample.pdop = invalid; }, Status::non_finite_input);
        check_bad_data([&](auto & sample) { sample.horizontal_accuracy = invalid; }, Status::non_finite_input);
        check_bad_data([&](auto & sample) { sample.vertical_accuracy = invalid; }, Status::non_finite_input);
        check_bad_data([&](auto & sample) { sample.speed_accuracy = invalid; }, Status::non_finite_input);
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            check_bad_data([&](auto & sample) { sample.velocity_n.set(axis, invalid); }, Status::non_finite_input);
            check_bad_data([&](auto & sample) { sample.position_delta_n.set(axis, invalid); },
                           Status::non_finite_input);
        }
    }
    check_bad_data([](auto & sample) { sample.speed_accuracy = value_type{-1}; }, Status::domain_error);
    check_bad_data([](auto & sample) { sample.velocity_n.set(0U, limits_type::max()); }, Status::non_finite_result);

    fixture_type rate_overflow;
    context.expect(rate_overflow.update(stationary) == Status::success, profile, "prime position-rate overflow");
    rate_overflow.sample.time_us += 1U;
    rate_overflow.sample.position_delta_n.set(0U, limits_type::max());
    context.expect(rate_overflow.update(stationary) == Status::non_finite_result &&
                       rate_overflow.result.decision == GnssQualityDecision::rejected,
                   profile, "rate overflow is rejected before it could be hidden by state clamping");

    auto check_atomic_failure = [&](auto mutate, Status expected)
    {
        fixture_type fixture;
        context.expect(fixture.update(stationary) == Status::success, profile, "prime atomic-failure sequence");
        fixture.sample.time_us += 100000U;
        fixture.sample.position_delta_n.set(0U, static_cast<value_type>(0.2));
        fixture.sample.velocity_n.set(0U, static_cast<value_type>(0.2));
        context.expect(fixture.update(stationary) == Status::success && fixture.result.horizontal_drift > value_type{0},
                       profile, "nonzero filter history before invalid call");
        auto control = fixture;
        fixture.sample.time_us += 100000U;
        mutate(fixture);
        context.expect(fixture.update(stationary) == expected &&
                           fixture.result.decision == GnssQualityDecision::invalid_call,
                       profile, "invalid time/configuration/reference is distinguished from consumed rejection");
        fixture.parameters = control.parameters;
        control.sample.time_us += 200000U;
        control.sample.position_delta_n.set(0U, static_cast<value_type>(0.2));
        fixture.sample = control.sample;
        context.expect(fixture.update(stationary) == Status::success && control.update(stationary) == Status::success &&
                           same_result(fixture.result, control.result),
                       profile, "invalid call leaves filter/timer history unchanged");
    };
    check_atomic_failure([](auto & fixture) { fixture.sample.time_us = 0U; }, Status::out_of_range);
    check_atomic_failure([](auto & fixture) { fixture.sample.time_us = 1000000U; }, Status::out_of_range);
    check_atomic_failure([](auto & fixture) { fixture.sample.time_us = 1100000U; }, Status::out_of_range);
    check_atomic_failure([](auto & fixture) { fixture.sample.time_us = 999999U; }, Status::out_of_range);
    check_atomic_failure([](auto & fixture) { fixture.sample.position_reference_time_us = 0U; }, Status::out_of_range);
    check_atomic_failure([](auto & fixture) { ++fixture.sample.position_reference_time_us; }, Status::out_of_range);
    check_atomic_failure([](auto & fixture) { fixture.parameters.minimum_health_time_us = 0U; }, Status::domain_error);
    check_atomic_failure([](auto & fixture) { fixture.parameters.maximum_pdop = limits_type::quiet_NaN(); },
                         Status::non_finite_input);
    check_atomic_failure([](auto & fixture) { fixture.parameters.maximum_vertical_drift = value_type{0}; },
                         Status::domain_error);
    check_atomic_failure([](auto & fixture) { fixture.parameters.velocity_limit = limits_type::max(); },
                         Status::domain_error);
    check_atomic_failure([](auto & fixture) { fixture.parameters.velocity_limit = limits_type::min(); },
                         Status::domain_error);
    check_atomic_failure([](auto & fixture) { fixture.parameters.maximum_horizontal_drift = limits_type::min(); },
                         Status::domain_error);
    check_atomic_failure([](auto & fixture) { fixture.parameters.maximum_horizontal_drift = limits_type::max(); },
                         Status::domain_error);
    check_atomic_failure([](auto & fixture) { fixture.parameters.maximum_vertical_drift = limits_type::max(); },
                         Status::domain_error);

    fixture_type large_thresholds;
    large_thresholds.parameters.maximum_pdop = limits_type::max();
    large_thresholds.parameters.maximum_horizontal_accuracy = limits_type::max();
    large_thresholds.parameters.maximum_vertical_accuracy = limits_type::max();
    large_thresholds.parameters.maximum_speed_accuracy = limits_type::max();
    context.expect(large_thresholds.update() == Status::success &&
                       large_thresholds.result.decision == GnssQualityDecision::waiting,
                   profile, "comparison-only bounds do not inherit motion arithmetic limits");

    fixture_type clock;
    context.expect(clock.update({}, 2000000U) == Status::success, profile, "separate sample and fusion-horizon clocks");
    auto control_clock = clock;
    clock.sample.time_us += 1U;
    context.expect(clock.update({}, 1999999U) == Status::out_of_range &&
                       clock.result.decision == GnssQualityDecision::invalid_call,
                   profile, "backward fusion horizon rejected atomically");
    context.expect(clock.update({}, 1000000U) == Status::out_of_range &&
                       clock.result.decision == GnssQualityDecision::invalid_call,
                   profile, "future sample rejected atomically");
    clock.sample.time_us = 11000001U;
    control_clock.sample.time_us = clock.sample.time_us;
    context.expect(clock.update({}, 12000001U) == Status::success && clock.result.qualified &&
                       control_clock.update({}, 12000001U) == Status::success &&
                       same_result(clock.result, control_clock.result),
                   profile, "health elapsed uses fusion horizon, not measurement interval");
    fixture_type large_clock;
    large_clock.sample.time_us = std::numeric_limits<std::uint64_t>::max() - 20000000U;
    context.expect(large_clock.update() == Status::success, profile, "large unsigned time accepted");
    large_clock.sample.time_us += 11000000U;
    context.expect(large_clock.update() == Status::success && large_clock.result.qualified, profile,
                   "timer comparison does not overflow by adding duration to timestamp");
}

} /* end anonymous namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    TestContext context;
    run_conformance<formal_eskf::test::Backend<float>>(context, "float");
    run_conformance<formal_eskf::test::Backend<double>>(context, "double");
    if (context.failures() != 0)
    {
        std::cerr << context.failures() << " GNSS quality check(s) failed\n";
        return 1;
    }
    std::cout << "GNSS quality checks passed\n";
    return 0;
}
