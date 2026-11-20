/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/gnss_quality.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <string_view>

#include "test_backend.hpp"
#include "test_cases.hpp"
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

constexpr GnssQualityMotion stationary{.in_air = false, .vehicle_at_rest = true};
constexpr GnssQualityMotion moving{};
constexpr GnssQualityMotion flying{.in_air = true, .vehicle_at_rest = false};
constexpr auto checks = std::array{&GnssQualityChecks::satellites,          &GnssQualityChecks::pdop,
                                   &GnssQualityChecks::horizontal_accuracy, &GnssQualityChecks::vertical_accuracy,
                                   &GnssQualityChecks::speed_accuracy,      &GnssQualityChecks::horizontal_drift,
                                   &GnssQualityChecks::vertical_drift,      &GnssQualityChecks::horizontal_speed,
                                   &GnssQualityChecks::vertical_speed,      &GnssQualityChecks::spoofed};

template <typename Scalar> bool same_scalar(Scalar a, Scalar b)
{
    using Bytes = std::array<std::byte, sizeof(Scalar)>;
    return (std::isnan(a) && std::isnan(b)) || std::bit_cast<Bytes>(a) == std::bit_cast<Bytes>(b);
}

template <typename Linalg, typename Mutation>
void atomic_failure(TestContext & context, std::string_view profile, Mutation mutate, Status expected)
{
    Fixture<Linalg> fixture;
    using T = typename Linalg::value_type;
    context.expect(fixture.update(stationary) == Status::success, profile, "prime monitor");
    fixture.sample.time_us += 100000U;
    fixture.sample.position_delta_n(0) = T{0.25};
    fixture.sample.velocity_n(1) = T{-0.5};
    context.expect(fixture.update(stationary) == Status::success, profile, "nonzero history");
    auto control = fixture;
    fixture.sample.time_us += 100000U;
    auto now = fixture.sample.time_us;
    mutate(fixture, now);
    context.expect(fixture.update(stationary, now) == expected &&
                       fixture.result.decision == GnssQualityDecision::invalid_call,
                   profile, "caller error is not consumed");
    // Retry the exact next valid timestamp, not a later one that could conceal a watermark write.
    control.sample.time_us += 100000U;
    fixture.sample = control.sample;
    fixture.parameters = control.parameters;
    for (unsigned step = 0; step < 3; ++step)
    {
        context.expect(fixture.update(stationary) == Status::success && control.update(stationary) == Status::success &&
                           same_result(fixture.result, control.result),
                       profile, "corrected retry and continuation preserve all history");
        fixture.sample.time_us += 100000U;
        control.sample.time_us += 100000U;
    }
}
template <typename Linalg> void defaults(TestContext & context, std::string_view profile)
{
    using fixture_type = Fixture<Linalg>;
    using value_type = typename Linalg::value_type;
    fixture_type fixture;
    auto const & p = fixture.parameters;
    context.expect(p.minimum_satellites == 6U && p.maximum_pdop == value_type{2.5} &&
                       p.maximum_horizontal_accuracy == value_type{3} && p.maximum_vertical_accuracy == value_type{5} &&
                       p.maximum_speed_accuracy == value_type{0.5} &&
                       p.maximum_horizontal_drift == static_cast<value_type>(0.1) &&
                       p.maximum_vertical_drift == static_cast<value_type>(0.2) &&
                       p.velocity_limit == value_type{100} && p.minimum_health_time_us == 10000000U,
                   profile, "declared defaults including units and health duration");
    for (auto selected : checks)
    {
        context.expect(p.enabled_checks.*selected, profile, "all ten named checks default enabled");
        GnssQualityChecks failure{};
        failure.*selected = true;
        for (auto enabled : checks)
        {
            GnssQualityChecks mask{};
            mask.*enabled = true;
            context.expect(failure.any_enabled(mask) == (selected == enabled), profile,
                           "independent one-hot check selection");
            mask = p.enabled_checks;
            mask.*enabled = false;
            context.expect(failure.any_enabled(mask) == (selected != enabled), profile,
                           "disable only the matching diagnostic");
        }
    }
    GnssQualityChecks combined{};
    combined.pdop = combined.spoofed = true;
    context.expect(combined.any_enabled(p.enabled_checks) && !combined.any_enabled({}) &&
                       !GnssQualityChecks{}.any_enabled(p.enabled_checks),
                   profile, "combined and empty masks");
    for (std::uint8_t fix : {std::uint8_t{0}, std::uint8_t{2}, std::uint8_t{3}, std::uint8_t{4}})
    {
        fixture_type f;
        f.sample.fix_type = fix;
        f.parameters.enabled_checks = {};
        context.expect(f.update() == Status::success && f.result.invalid_fix == (fix < 3U) &&
                           (f.result.decision == GnssQualityDecision::rejected) == (fix < 3U),
                       profile, "mandatory fix cannot be masked");
    }
    for (std::uint8_t satellites : {std::uint8_t{5}, std::uint8_t{6}, std::uint8_t{7}})
    {
        fixture_type f;
        f.sample.satellites = satellites;
        context.expect(f.update() == Status::success && f.result.failures.satellites == (satellites < 6U), profile,
                       "inclusive integer satellite boundary");
    }
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
            masked.sample.position_delta_n(axis) = value_type{20};
        }
        else
        {
            masked.sample.velocity_n(axis) = value_type{2};
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
}

template <typename Linalg> void metadata(TestContext & context, std::string_view profile)
{
    using fixture_type = Fixture<Linalg>;
    using value_type = typename Linalg::value_type;
    using limits_type = std::numeric_limits<value_type>;
    using Sample = typename fixture_type::monitor_type::sample_type;
    std::array members{&Sample::pdop, &Sample::horizontal_accuracy, &Sample::vertical_accuracy,
                       &Sample::speed_accuracy};
    std::array diagnostics{&GnssQualityChecks::pdop, &GnssQualityChecks::horizontal_accuracy,
                           &GnssQualityChecks::vertical_accuracy, &GnssQualityChecks::speed_accuracy};
    for (std::size_t field = 0; field < members.size(); ++field)
    {
        for (auto motion : {stationary, moving, flying})
        {
            for (int side : {-1, 0, 1})
            {
                for (bool enabled : {false, true})
                {
                    fixture_type f;
                    auto & x = f.sample.*members[field];
                    if (side != 0)
                    {
                        x = std::nextafter(x, side < 0 ? value_type{0} : limits_type::infinity());
                    }
                    f.parameters.enabled_checks.*diagnostics[field] = enabled;
                    GnssQualityChecks expected;
                    expected.*diagnostics[field] = side > 0;
                    context.expect(f.update(motion) == Status::success && same_checks(f.result.failures, expected) &&
                                       (f.result.decision == GnssQualityDecision::rejected) == (side > 0 && enabled),
                                   profile, "metadata threshold neighbours, mode and matching enable bit");
                }
            }
        }
    }
    for (bool spoofed : {false, true})
    {
        for (bool enabled : {false, true})
        {
            fixture_type f;
            f.sample.spoofed = spoofed;
            f.parameters.enabled_checks.spoofed = enabled;
            context.expect(f.update() == Status::success && f.result.failures.spoofed == spoofed &&
                               (f.result.decision == GnssQualityDecision::rejected) == (spoofed && enabled),
                           profile, "spoof diagnostic remains visible when disabled");
        }
    }
}

template <typename Linalg> void recurrence(TestContext & context, std::string_view profile)
{
    using fixture_type = Fixture<Linalg>;
    using value_type = typename Linalg::value_type;
    using limits_type = std::numeric_limits<value_type>;
    fixture_type f;
    f.parameters.maximum_horizontal_drift = value_type{0.5};
    f.parameters.maximum_vertical_drift = value_type{1};
    f.parameters.enabled_checks = {};
    std::array<value_type, 3> position{}, velocity{};
    std::uint64_t previous = 0;
    bool continuity = false;
    // Ordered IEEE recurrence is the oracle here, not a universal real-number accuracy claim.
    // Unequal signed components distinguish input clipping from state clipping and axis mistakes.
    std::array<std::uint64_t, 10> intervals{1000000U,  999U,      1000U, 1001U,   9999999U,
                                            10000000U, 10000001U, 1U,    250000U, 250000U};
    for (std::size_t step = 0; step < intervals.size(); ++step)
    {
        f.sample.time_us = previous + intervals[step];
        auto const dt = std::clamp(static_cast<value_type>(intervals[step]) * static_cast<value_type>(1e-6),
                                   static_cast<value_type>(0.001), value_type{10});
        auto const alpha = dt / value_type{10};
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            value_type const sign = step % 2 == 0 ? value_type{1} : value_type{-1};
            auto const displacement = sign * static_cast<value_type>(axis + 1U) * value_type{80};
            auto const speed = -sign * static_cast<value_type>(axis + 1U) * value_type{16};
            f.sample.position_delta_n(axis) = continuity ? displacement : limits_type::quiet_NaN();
            f.sample.velocity_n(axis) = speed;
            auto const limit = axis == 2 ? value_type{10} : value_type{5};
            auto const increment = continuity ? displacement : value_type{0};
            position[axis] =
                std::clamp(increment / dt * alpha + position[axis] * (value_type{1} - alpha), -limit, limit);
            velocity[axis] = std::clamp(speed, -limit, limit) * alpha + velocity[axis] * (value_type{1} - alpha);
        }
        auto const horizontal = [](auto const & x) { return std::sqrt(x[0] * x[0] + x[1] * x[1]); };
        context.expect(
            f.update(stationary) == Status::success && f.result.drift_valid &&
                f.result.horizontal_drift == horizontal(position) && f.result.vertical_drift == std::abs(position[2]) &&
                f.result.horizontal_speed == horizontal(velocity) && f.result.vertical_speed == std::abs(velocity[2]),
            profile, "independent scalar arrays implement dt clamps, signed memory and distinct clipping order");
        previous = f.sample.time_us;
        continuity = true;
    }
}

template <typename Linalg> void motion(TestContext & context, std::string_view profile)
{
    using fixture_type = Fixture<Linalg>;
    using value_type = typename Linalg::value_type;
    using limits_type = std::numeric_limits<value_type>;
    fixture_type drift;
    drift.parameters.maximum_horizontal_drift = value_type{0.5};
    drift.parameters.maximum_vertical_drift = value_type{1};
    drift.sample.time_us = 10000000U;
    drift.sample.position_delta_n(0U) = limits_type::quiet_NaN();
    context.expect(drift.update(stationary) == Status::success &&
                       drift.result.decision == GnssQualityDecision::waiting && drift.result.drift_valid &&
                       drift.result.horizontal_drift == value_type{0},
                   profile, "first fix ignores unavailable position displacement");
    drift.sample.time_us += 10000000U;
    drift.sample.position_delta_n(0U) = value_type{5};
    drift.sample.position_delta_n(2U) = value_type{-10};
    context.expect(drift.update(stationary) == Status::success &&
                       drift.result.decision == GnssQualityDecision::waiting &&
                       drift.result.horizontal_drift == value_type{0.5} && drift.result.vertical_drift == value_type{1},
                   profile, "stationary drift threshold equality passes");
    drift.sample.time_us += 10000000U;
    drift.sample.position_delta_n(0U) = value_type{10};
    drift.sample.position_delta_n(2U) = value_type{-20};
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

    fixture_type speed;
    speed.sample.time_us = 10000000U;
    speed.sample.velocity_n(0U) = speed.parameters.maximum_horizontal_drift;
    speed.sample.velocity_n(2U) = -speed.parameters.maximum_vertical_drift;
    context.expect(speed.update(stationary) == Status::success && speed.result.decision == GnssQualityDecision::waiting,
                   profile, "stationary speed threshold equality passes");
    speed.sample.time_us += 10000000U;
    speed.sample.velocity_n(1U) = speed.parameters.maximum_horizontal_drift;
    speed.sample.velocity_n(2U) = -value_type{1};
    context.expect(speed.update(stationary) == Status::success && speed.result.failures.horizontal_speed &&
                       speed.result.failures.vertical_speed,
                   profile, "stationary speed uses horizontal norm and absolute down velocity");
    speed.sample.time_us += 1U;
    speed.sample.velocity_n(0U) = value_type{60};
    speed.sample.velocity_n(1U) = value_type{80};
    speed.sample.velocity_n(2U) = value_type{-100};
    context.expect(speed.update(flying) == Status::success && speed.result.decision == GnssQualityDecision::waiting,
                   profile, "raw 100 m/s boundary passes in flight");
    speed.sample.time_us += 1U;
    speed.sample.velocity_n(0U) = value_type{80};
    speed.sample.velocity_n(2U) = value_type{-101};
    context.expect(speed.update(flying) == Status::success && speed.result.failures.horizontal_speed &&
                       speed.result.failures.vertical_speed,
                   profile, "raw speed limits force failure even in flight");
    speed.parameters.enabled_checks = {};
    speed.monitor.reset();
    context.expect(speed.update(flying) == Status::success && speed.result.decision == GnssQualityDecision::waiting &&
                       speed.result.failures.horizontal_speed && speed.result.failures.vertical_speed,
                   profile, "disabled speed checks retain raw-speed diagnostics");

    for (auto mode : {stationary, moving, flying})
    {
        for (int side : {-1, 0, 1})
        {
            for (value_type sign : {value_type{-1}, value_type{1}})
            {
                for (bool enabled : {false, true})
                {
                    fixture_type f;
                    f.sample.time_us = 1U; // First stationary filtering cannot itself exceed its drift threshold.
                    f.parameters.enabled_checks = {};
                    f.parameters.enabled_checks.horizontal_speed = enabled;
                    f.parameters.enabled_checks.vertical_speed = enabled;
                    auto raw = value_type{100};
                    if (side != 0)
                    {
                        raw = std::nextafter(raw, side < 0 ? value_type{0} : limits_type::infinity());
                    }
                    f.sample.velocity_n(0) = sign * raw;
                    f.sample.velocity_n(2) = -sign * raw;
                    f.sample.position_delta_n(2) = limits_type::quiet_NaN();
                    f.sample.position_reference_time_us = UINT64_MAX;
                    context.expect(f.update(mode) == Status::success &&
                                       f.result.failures.horizontal_speed == (side > 0) &&
                                       f.result.failures.vertical_speed == (side > 0) &&
                                       (f.result.decision == GnssQualityDecision::rejected) == (side > 0 && enabled),
                                   profile, "raw-speed strict neighbours and signs apply in every motion mode");
                }
            }
        }
    }
    for (auto mode : {moving, flying})
    {
        fixture_type f;
        context.expect(f.update(stationary) == Status::success, profile, "prime ignored-reference context");
        ++f.sample.time_us;
        f.sample.position_delta_n(0) = limits_type::quiet_NaN();
        f.sample.position_reference_time_us = UINT64_MAX;
        context.expect(f.update(mode) == Status::success && !f.result.drift_valid, profile,
                       "nonstationary displacement/reference are unconsumed");
    }
}

template <typename Linalg> void timers(TestContext & context, std::string_view profile)
{
    using fixture_type = Fixture<Linalg>;
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

    for (std::uint64_t health : {1U, 10000000U, 20000000U})
    {
        auto const recovery = std::max(std::uint64_t{1000000}, health / 10U);
        for (auto threshold : {health, recovery})
        {
            for (std::uint64_t elapsed : {threshold - 1U, threshold, threshold + 1U})
            {
                if (elapsed == 0)
                {
                    continue;
                }
                fixture_type f;
                f.parameters.minimum_health_time_us = health;
                auto const initial = f.sample.time_us;
                context.expect(f.update({}, initial + 100U) == Status::success, profile, "horizon baseline");
                ++f.sample.time_us;
                auto const now = initial + 100U + elapsed;
                bool const qualified = elapsed > health && elapsed > recovery;
                context.expect(f.update({}, now) == Status::success && f.result.qualified == qualified &&
                                   f.result.last_fail_us == initial + 100U && f.result.last_pass_us == now &&
                                   (f.result.decision == GnssQualityDecision::accepted) == qualified,
                               profile, "both strict durations use supplied horizon independent of sample spacing");
            }
        }
    }
}

template <typename Linalg> void consumed_data(TestContext & context, std::string_view profile)
{
    using fixture_type = Fixture<Linalg>;
    using value_type = typename Linalg::value_type;
    using limits_type = std::numeric_limits<value_type>;
    auto check_bad_data = [&](auto mutate, Status status)
    {
        for (bool masked : {false, true})
        {
            fixture_type fixture;
            if (masked)
            {
                fixture.parameters.enabled_checks = {};
            }
            context.expect(fixture.update(stationary) == Status::success, profile, "prime invalid-data sequence");
            auto valid = fixture.sample;
            fixture.sample.time_us += 11000000U;
            mutate(fixture.sample);
            context.expect(fixture.update(stationary) == status &&
                               fixture.result.decision == GnssQualityDecision::rejected && !fixture.result.qualified &&
                               fixture.result.last_fail_us == fixture.sample.time_us,
                           profile, "bad data cannot be masked or qualify health");
            auto const consumed = fixture.sample.time_us;
            fixture.sample = valid;
            fixture.sample.time_us = consumed;
            context.expect(fixture.update(stationary) == Status::out_of_range &&
                               fixture.result.decision == GnssQualityDecision::invalid_call,
                           profile, "numerical rejection consumes its timestamp");
            valid.time_us = consumed + 1U;
            fixture.sample = valid;
            fixture.sample.position_delta_n(0U) = limits_type::quiet_NaN();
            context.expect(fixture.update(stationary) == Status::success &&
                               fixture.result.decision == GnssQualityDecision::waiting &&
                               fixture.result.horizontal_drift == value_type{0},
                           profile, "bad data reset position continuity and restart recovery wait");
        }
    };
    for (auto const invalid : {limits_type::quiet_NaN(), limits_type::infinity(), -limits_type::infinity()})
    {
        check_bad_data([&](auto & sample) { sample.pdop = invalid; }, Status::non_finite_input);
        check_bad_data([&](auto & sample) { sample.horizontal_accuracy = invalid; }, Status::non_finite_input);
        check_bad_data([&](auto & sample) { sample.vertical_accuracy = invalid; }, Status::non_finite_input);
        check_bad_data([&](auto & sample) { sample.speed_accuracy = invalid; }, Status::non_finite_input);
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            check_bad_data([&](auto & sample) { sample.velocity_n(axis) = invalid; }, Status::non_finite_input);
            check_bad_data([&](auto & sample) { sample.position_delta_n(axis) = invalid; }, Status::non_finite_input);
        }
    }
    check_bad_data([](auto & sample) { sample.pdop = value_type{-1}; }, Status::domain_error);
    check_bad_data([](auto & sample) { sample.horizontal_accuracy = value_type{-1}; }, Status::domain_error);
    check_bad_data([](auto & sample) { sample.vertical_accuracy = value_type{-1}; }, Status::domain_error);
    check_bad_data([](auto & sample) { sample.speed_accuracy = value_type{-1}; }, Status::domain_error);
    check_bad_data([](auto & sample) { sample.velocity_n(0U) = limits_type::max(); }, Status::non_finite_result);

    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        fixture_type rate_overflow;
        context.expect(rate_overflow.update(stationary) == Status::success, profile, "prime position-rate overflow");
        rate_overflow.sample.time_us += 1U;
        rate_overflow.sample.position_delta_n =
            formal_eskf::linalg::Vector<Linalg, 3U>::from_row_major({value_type{1}, value_type{1}, value_type{1}});
        rate_overflow.sample.velocity_n = rate_overflow.sample.position_delta_n;
        rate_overflow.sample.position_delta_n(axis) = limits_type::max();
        context.expect(rate_overflow.update(stationary) == Status::non_finite_result &&
                           rate_overflow.result.decision == GnssQualityDecision::rejected &&
                           !rate_overflow.result.drift_valid &&
                           rate_overflow.result.horizontal_drift == value_type{0} &&
                           rate_overflow.result.vertical_drift == value_type{0},
                       profile, "overflow on any axis clears partial filter work and diagnostics before clamping");
        ++rate_overflow.sample.time_us;
        rate_overflow.sample.position_delta_n(axis) = limits_type::quiet_NaN();
        rate_overflow.sample.velocity_n = {};
        context.expect(rate_overflow.update(stationary) == Status::success &&
                           rate_overflow.result.decision == GnssQualityDecision::waiting &&
                           rate_overflow.result.horizontal_drift == value_type{0} &&
                           rate_overflow.result.vertical_drift == value_type{0} &&
                           rate_overflow.result.horizontal_speed == value_type{0} &&
                           rate_overflow.result.vertical_speed == value_type{0},
                       profile, "consumed numerical rejection resets every axis and ignores the next position delta");
    }
}

template <typename Linalg> void caller_frame(TestContext & context, std::string_view profile)
{
    using fixture_type = Fixture<Linalg>;
    for (std::uint64_t bad : {0U, 999999U, 1000000U, 1100000U})
    {
        atomic_failure<Linalg>(
            context, profile, [=](auto & f, auto &) { f.sample.time_us = bad; }, Status::out_of_range);
    }
    for (std::uint64_t bad : {0U, 1099999U, 1100001U})
    {
        atomic_failure<Linalg>(
            context, profile, [=](auto & f, auto &) { f.sample.position_reference_time_us = bad; },
            Status::out_of_range);
    }
    atomic_failure<Linalg>(
        context, profile, [](auto & f, auto & now) { now = f.sample.time_us - 1U; }, Status::out_of_range);
    fixture_type f;
    context.expect(f.update(stationary, 2000000U) == Status::success, profile, "delayed source baseline");
    auto control = f;
    ++f.sample.time_us;
    context.expect(f.update(stationary, 1999999U) == Status::out_of_range &&
                       f.result.decision == GnssQualityDecision::invalid_call,
                   profile, "independently backward horizon");
    control.sample = f.sample;
    context.expect(f.update(stationary, 2000000U) == Status::success &&
                       control.update(stationary, 2000000U) == Status::success && same_result(f.result, control.result),
                   profile, "same horizon with newer sample is legal and invalid call preserves reference");
}

template <typename Linalg> void parameters(TestContext & context, std::string_view profile)
{
    using fixture_type = Fixture<Linalg>;
    using value_type = typename Linalg::value_type;
    using limits_type = std::numeric_limits<value_type>;
    using Parameters = typename fixture_type::monitor_type::parameters_type;
    std::array members{&Parameters::maximum_pdop,
                       &Parameters::maximum_horizontal_accuracy,
                       &Parameters::maximum_vertical_accuracy,
                       &Parameters::maximum_speed_accuracy,
                       &Parameters::maximum_horizontal_drift,
                       &Parameters::maximum_vertical_drift,
                       &Parameters::velocity_limit};
    for (auto member : members)
    {
        for (value_type bad : {value_type{0}, -value_type{0}, value_type{-1}, limits_type::denorm_min(),
                               std::nextafter(limits_type::min(), value_type{0}), limits_type::quiet_NaN(),
                               limits_type::infinity(), -limits_type::infinity()})
        {
            atomic_failure<Linalg>(
                context, profile, [=](auto & f, auto &) { f.parameters.*member = bad; },
                std::isfinite(bad) ? Status::domain_error : Status::non_finite_input);
        }
        fixture_type f;
        f.parameters.*member = limits_type::min();
        auto const has_squared_lower_bound =
            member == &Parameters::maximum_horizontal_drift || member == &Parameters::velocity_limit;
        context.expect(f.update() == (has_squared_lower_bound ? Status::domain_error : Status::success), profile,
                       "minimum normal is valid unless the consumed square underflows");
    }
    atomic_failure<Linalg>(
        context, profile, [](auto & f, auto &) { f.parameters.minimum_health_time_us = 0U; }, Status::domain_error);
    for (auto member :
         {&Parameters::maximum_horizontal_drift, &Parameters::maximum_vertical_drift, &Parameters::velocity_limit})
    {
        atomic_failure<Linalg>(
            context, profile, [=](auto & f, auto &) { f.parameters.*member = limits_type::max(); },
            Status::domain_error);
    }
    for (auto member : {&Parameters::maximum_horizontal_drift, &Parameters::velocity_limit})
    {
        // Powers of two make the independently derived squared-normal threshold exact.
        auto const boundary = std::ldexp(value_type{1}, (limits_type::min_exponent - 1) / 2);
        for (auto value :
             {std::nextafter(boundary, value_type{0}), boundary, std::nextafter(boundary, limits_type::infinity())})
        {
            fixture_type f;
            f.parameters.*member = value;
            bool const valid = value * value >= limits_type::min();
            context.expect(f.update() == (valid ? Status::success : Status::domain_error), profile,
                           "square lower-bound neighbours");
        }
    }
    fixture_type large_thresholds;
    large_thresholds.parameters.maximum_pdop = limits_type::max();
    large_thresholds.parameters.maximum_horizontal_accuracy = limits_type::max();
    large_thresholds.parameters.maximum_vertical_accuracy = limits_type::max();
    large_thresholds.parameters.maximum_speed_accuracy = limits_type::max();
    context.expect(large_thresholds.update() == Status::success &&
                       large_thresholds.result.decision == GnssQualityDecision::waiting,
                   profile, "comparison-only bounds do not inherit motion arithmetic limits");
}

template <typename Linalg> void health_reset(TestContext & context, std::string_view profile)
{
    using fixture_type = Fixture<Linalg>;
    using value_type = typename Linalg::value_type;
    using limits_type = std::numeric_limits<value_type>;
    fixture_type f;
    f.parameters.enabled_checks = {};
    context.expect(f.update(stationary) == Status::success, profile, "start health");
    f.sample.time_us += 11000000U;
    f.sample.position_delta_n(0) = value_type{1};
    f.sample.velocity_n(2) = value_type{0.5};
    context.expect(f.update(stationary) == Status::success && f.result.qualified, profile, "qualified nonzero history");
    f.sample.time_us += 100000U;
    f.sample.fix_type = 2U;
    context.expect(f.update(stationary) == Status::success && f.result.qualified &&
                       f.result.decision == GnssQualityDecision::rejected,
                   profile, "metadata rejection preserves filter and historical qualification");
    auto control = f;
    f.monitor.restart_health();
    f.sample.fix_type = 3U;
    control.sample.fix_type = 3U;
    context.expect(f.update(stationary) == Status::out_of_range, profile, "restart does not reopen consumed timestamp");
    ++f.sample.time_us;
    auto saved = f.sample;
    f.sample.position_reference_time_us = 0U;
    context.expect(f.update(stationary) == Status::out_of_range, profile,
                   "restart retains displacement reference continuity");
    f.sample = saved;
    control.sample = saved;
    context.expect(f.update(stationary) == Status::success && control.update(stationary) == Status::success &&
                       f.result.qualified && f.result.last_fail_us == f.sample.time_us &&
                       f.result.last_pass_us == f.sample.time_us &&
                       f.result.horizontal_drift == control.result.horizontal_drift &&
                       f.result.vertical_speed == control.result.vertical_speed &&
                       same_checks(f.result.failures, control.result.failures),
                   profile, "restart changes only health timestamps");
    f.sample.time_us += 1000001U;
    context.expect(f.update(stationary) == Status::success && f.result.decision == GnssQualityDecision::accepted,
                   profile, "historically qualified monitor needs only recent recovery");
    f.sample.time_us += 100000000U;
    context.expect(f.update(stationary) == Status::success && f.result.qualified, profile,
                   "long gap alone does not revoke qualification");
    f.monitor.reset();
    f.sample = typename fixture_type::monitor_type::sample_type{};
    f.sample.time_us = 1U;
    f.sample.fix_type = 3U;
    f.sample.satellites = 6U;
    f.sample.position_delta_n(1) = limits_type::quiet_NaN();
    context.expect(f.update(stationary) == Status::success && !f.result.qualified &&
                       f.result.horizontal_drift == value_type{0} && f.result.vertical_speed == value_type{0},
                   profile, "full reset accepts a new timeline and clears filter and qualification history");
}

template <typename Linalg> void clock_purity(TestContext & context, std::string_view profile)
{
    using fixture_type = Fixture<Linalg>;
    using value_type = typename Linalg::value_type;
    using Sample = typename fixture_type::monitor_type::sample_type;
    using Parameters = typename fixture_type::monitor_type::parameters_type;
    fixture_type small, large;
    std::uint64_t const offset = UINT64_MAX - 50000000U;
    for (std::uint64_t time : {1000000U, 1100000U, 1200000U, 12000001U})
    {
        small.sample.time_us = time;
        large.sample = small.sample;
        large.sample.time_us += offset;
        if (large.sample.position_reference_time_us != 0U)
        {
            large.sample.position_reference_time_us += offset;
        }
        small.sample.velocity_n(0) = large.sample.velocity_n(0) = time == 1000000U ? value_type{0} : value_type{0.125};
        small.sample.position_delta_n(2) = large.sample.position_delta_n(2) = -value_type{0};
        auto const before = large.sample;
        auto const parameters_before = large.parameters;
        // Call production directly, so the fixture's caller-owned reference update is not confused with mutation.
        large.result = large.monitor.update(large.sample, stationary, large.sample.time_us + 99U, large.parameters);
        context.expect(small.update(stationary, time + 99U) == Status::success &&
                           large.result.status == Status::success && small.result.qualified == large.result.qualified &&
                           small.result.decision == large.result.decision &&
                           same_checks(small.result.failures, large.result.failures) &&
                           small.result.horizontal_drift == large.result.horizontal_drift &&
                           small.result.horizontal_speed == large.result.horizontal_speed &&
                           small.result.last_fail_us + offset == large.result.last_fail_us &&
                           small.result.last_pass_us + offset == large.result.last_pass_us,
                       profile, "large ordered timestamps have the same relative-time decisions and filters");
        context.expect(large.sample.time_us == before.time_us && large.sample.fix_type == before.fix_type &&
                           large.sample.satellites == before.satellites && large.sample.spoofed == before.spoofed &&
                           large.sample.position_reference_time_us == before.position_reference_time_us,
                       profile, "borrowed sample discrete fields unchanged");
        for (auto member :
             {&Sample::pdop, &Sample::horizontal_accuracy, &Sample::vertical_accuracy, &Sample::speed_accuracy})
        {
            context.expect(same_scalar(large.sample.*member, before.*member), profile, "borrowed metadata unchanged");
        }
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            context.expect(same_scalar(large.sample.velocity_n(axis), before.velocity_n(axis)) &&
                               same_scalar(large.sample.position_delta_n(axis), before.position_delta_n(axis)),
                           profile, "borrowed vectors preserve representations");
        }
        for (auto member :
             {&Parameters::maximum_pdop, &Parameters::maximum_horizontal_accuracy,
              &Parameters::maximum_vertical_accuracy, &Parameters::maximum_speed_accuracy,
              &Parameters::maximum_horizontal_drift, &Parameters::maximum_vertical_drift, &Parameters::velocity_limit})
        {
            context.expect(same_scalar(large.parameters.*member, parameters_before.*member), profile,
                           "borrowed scalar settings unchanged");
        }
        context.expect(same_checks(large.parameters.enabled_checks, parameters_before.enabled_checks) &&
                           large.parameters.minimum_satellites == parameters_before.minimum_satellites &&
                           large.parameters.minimum_health_time_us == parameters_before.minimum_health_time_us,
                       profile, "borrowed selection and timing settings unchanged");
    }
}

} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using F = formal_eskf::test::Backend<float>;
    using D = formal_eskf::test::Backend<double>;
    using formal_eskf::test::Case;
    Case const cases[]{
        {"TC-R-GNSS-QUALITY-01", "binary32", defaults<F>},      {"TC-R-GNSS-QUALITY-01", "binary64", defaults<D>},
        {"TC-R-GNSS-QUALITY-02", "binary32", metadata<F>},      {"TC-R-GNSS-QUALITY-02", "binary64", metadata<D>},
        {"TC-R-GNSS-QUALITY-03", "binary32", recurrence<F>},    {"TC-R-GNSS-QUALITY-03", "binary64", recurrence<D>},
        {"TC-R-GNSS-QUALITY-04", "binary32", motion<F>},        {"TC-R-GNSS-QUALITY-04", "binary64", motion<D>},
        {"TC-R-GNSS-QUALITY-05", "binary32", timers<F>},        {"TC-R-GNSS-QUALITY-05", "binary64", timers<D>},
        {"TC-R-GNSS-QUALITY-06", "binary32", consumed_data<F>}, {"TC-R-GNSS-QUALITY-06", "binary64", consumed_data<D>},
        {"TC-R-GNSS-QUALITY-07", "binary32", caller_frame<F>},  {"TC-R-GNSS-QUALITY-07", "binary64", caller_frame<D>},
        {"TC-R-GNSS-QUALITY-08", "binary32", parameters<F>},    {"TC-R-GNSS-QUALITY-08", "binary64", parameters<D>},
        {"TC-R-GNSS-QUALITY-09", "binary32", health_reset<F>},  {"TC-R-GNSS-QUALITY-09", "binary64", health_reset<D>},
        {"TC-R-GNSS-QUALITY-10", "binary32", clock_purity<F>},  {"TC-R-GNSS-QUALITY-10", "binary64", clock_purity<D>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
