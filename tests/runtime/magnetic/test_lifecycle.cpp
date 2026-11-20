/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/magnetometer_lifecycle.hpp>
#include <formal_eskf/runtime/magnetometer.hpp>
#include <formal_eskf/runtime/magnetic_reference.hpp>
#include <formal_eskf/eskf/types.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <type_traits>
#include <iostream>
#include <limits>
#include <string_view>

#include "test_backend.hpp"
#include "test_support.hpp"
#include "test_cases.hpp"
#include "correction_reference.hpp"

namespace
{

using formal_eskf::Status;
using formal_eskf::runtime::FusionDecision;
using formal_eskf::runtime::MagnetometerFusion;
using Action = MagnetometerFusion::Action;
using StopReason = MagnetometerFusion::StopReason;
using RecoveryReason = MagnetometerFusion::RecoveryReason;
using TestContext = formal_eskf::test::Context;

struct Fixture
{
    MagnetometerFusion monitor;
    MagnetometerFusion::Parameters parameters{20U, 10U, 50U};
    MagnetometerFusion::Conditions conditions{true, true, true, true, false, MagnetometerFusion::FlightState::unknown,
                                              true};
    MagnetometerFusion::Source source{1U, 0U};

    Status sample(std::uint64_t time, bool accepted = true)
    {
        return monitor.update(time, conditions, parameters, {time, source, accepted});
    }

    Status tick(std::uint64_t time) { return monitor.update(time, conditions, parameters, {}); }

    void qualify(TestContext & test, std::string_view profile, std::uint64_t first = 100U)
    {
        test.expect(sample(first) == Status::success && sample(first + 8U) == Status::success &&
                        sample(first + 16U) == Status::success && sample(first + 21U) == Status::success &&
                        monitor.state().action == Action::fuse && !monitor.state().active &&
                        monitor.state().last_fuse_time_us == 0U,
                    profile, "four samples and strictly elapsed health permit an attempt, not a fictitious fusion");
    }

    void start(TestContext & test, std::string_view profile, std::uint64_t first = 100U)
    {
        qualify(test, profile, first);
        test.expect(monitor.record_fusion(first + 21U, FusionDecision::fused) == Status::success &&
                        monitor.state().active && monitor.state().start_time_us == first + 21U,
                    profile, "actual success activates the session");
    }
};

bool same_state(MagnetometerFusion::State const & a, MagnetometerFusion::State const & b)
{
    return a.active == b.active && a.action == b.action && a.filter_reset == b.filter_reset &&
           a.last_reset_time_us == b.last_reset_time_us && a.stop_reason == b.stop_reason &&
           a.recovery_reason == b.recovery_reason && a.last_field_failure_time_us == b.last_field_failure_time_us &&
           a.start_time_us == b.start_time_us && a.last_fuse_time_us == b.last_fuse_time_us;
}

void test_heading_policy(TestContext & test, std::string_view profile)
{
    Fixture fixture;
    fixture.start(test, profile);
    fixture.conditions.heading_consistent = false;
    fixture.conditions.at_rest = true;
    for (auto time : {130U, 150U, 180U, 200U})
    {
        test.expect(fixture.sample(time) == Status::success && fixture.monitor.state().active &&
                        fixture.monitor.state().action == Action::none &&
                        fixture.monitor.state().stop_reason == StopReason::heading_inconsistent &&
                        fixture.monitor.state().last_fuse_time_us == 121U,
                    profile, "intentional inhibition retains session without fake fusion or at-rest yaw reset");
    }
    test.expect(fixture.monitor.record_fusion(200U, FusionDecision::fused) == Status::out_of_range, profile,
                "inhibited sample cannot report fusion");
    fixture.conditions.heading_consistent = true;
    test.expect(fixture.sample(201U) == Status::success && fixture.monitor.state().action == Action::fuse &&
                    fixture.monitor.record_fusion(201U, FusionDecision::fused) == Status::success,
                profile, "consistency recovery tries the observation before judging timeout");
    fixture.conditions.heading_consistent = false;
    fixture.conditions.horizontal_aiding = false;
    test.expect(fixture.sample(202U) == Status::success && fixture.monitor.state().action == Action::fuse, profile,
                "without horizontal aiding, inconsistency does not veto magnetic heading aiding");
    fixture.conditions.horizontal_aiding = true;
    ++fixture.source.reference_id;
    test.expect(fixture.sample(203U) == Status::success && fixture.monitor.state().action != Action::reset &&
                    fixture.monitor.state().recovery_reason == RecoveryReason::none,
                profile, "reference change at rest cannot reset an inconsistent aided heading");
    test.expect(fixture.sample(260U, false) == Status::success && fixture.monitor.state().action != Action::reset &&
                    fixture.monitor.state().stop_reason == StopReason::fusion_timeout,
                profile, "a subsequent field failure times out without an inconsistent aided yaw reset");

    Fixture startup;
    startup.conditions.heading_consistent = false;
    for (auto time : {100U, 108U, 116U, 121U})
    {
        test.expect(startup.sample(time) == Status::success, profile, "unconfirmed startup consumes healthy samples");
    }
    test.expect(!startup.monitor.state().active && startup.monitor.state().action != Action::fuse &&
                    startup.monitor.state().stop_reason == StopReason::heading_inconsistent &&
                    startup.monitor.state().last_field_failure_time_us == 100U,
                profile, "heading inhibition does not falsify field health");
    startup.conditions.heading_consistent = true;
    test.expect(startup.sample(122U) == Status::success && startup.monitor.state().action == Action::fuse, profile,
                "establishing consistency releases already-qualified startup");
}

void test_startup(TestContext & test, std::string_view profile)
{
    for (unsigned mask = 0U; mask < 4U; ++mask)
    {
        Fixture fixture;
        fixture.conditions.enabled = (mask & 1U) != 0U;
        fixture.conditions.ready = (mask & 2U) != 0U;
        test.expect(fixture.sample(100U) == Status::success, profile, "valid condition combination");
        auto const & state = fixture.monitor.state();
        auto const expected = !fixture.conditions.enabled ? StopReason::disabled
                              : !fixture.conditions.ready ? StopReason::not_ready
                                                          : StopReason::waiting_for_health;
        test.expect(!state.active && state.action != Action::fuse && state.stop_reason == expected &&
                        state.last_field_failure_time_us == (mask == 3U ? 100U : 0U),
                    profile, "health checks run only when enabled and continuing conditions pass");
    }
    Fixture fixture;
    test.expect(fixture.tick(1U) == Status::success && fixture.monitor.state().stop_reason == StopReason::data_timeout,
                profile, "no initial sample cannot start");
    test.expect(fixture.monitor.update(2U, fixture.conditions, fixture.parameters, {0U, {99U, 99U}, true}) ==
                        Status::success &&
                    fixture.monitor.state().last_field_failure_time_us == 0U && !fixture.monitor.state().active,
                profile, "no-sample metadata cannot create health or bind a source");
    test.expect(fixture.sample(100U) == Status::success && fixture.sample(108U) == Status::success &&
                    fixture.sample(116U) == Status::success && fixture.sample(120U) == Status::success &&
                    fixture.monitor.state().action != Action::fuse && fixture.sample(121U) == Status::success &&
                    fixture.monitor.state().action == Action::fuse && !fixture.monitor.state().active,
                profile, "health equality does not qualify; one microsecond later permits an attempt");
    test.expect(fixture.monitor.record_fusion(121U, FusionDecision::rejected) == Status::success &&
                    !fixture.monitor.state().active && fixture.monitor.state().start_time_us == 0U &&
                    fixture.monitor.state().last_fuse_time_us == 0U && fixture.sample(180U) == Status::success &&
                    fixture.monitor.state().action == Action::fuse &&
                    fixture.monitor.record_fusion(180U, FusionDecision::rejected) == Status::success &&
                    fixture.monitor.state().recovery_reason == RecoveryReason::none,
                profile, "rejected startup retries without an invented active-session timeout");

    Fixture count;
    test.expect(count.sample(100U) == Status::success && count.sample(130U) == Status::success &&
                    count.sample(140U) == Status::success && count.monitor.state().action != Action::fuse &&
                    count.sample(150U) == Status::success && count.monitor.state().action == Action::fuse,
                profile, "elapsed time alone cannot bypass the four-sample requirement");
    Fixture failure;
    test.expect(failure.sample(100U, false) == Status::success && failure.sample(108U) == Status::success &&
                    failure.sample(116U) == Status::success && failure.sample(121U) == Status::success &&
                    failure.monitor.state().action == Action::fuse &&
                    failure.monitor.state().last_field_failure_time_us == 100U,
                profile, "health starts at the failed check, not the subsequent first good sample");
    test.expect(failure.sample(122U, false) == Status::success &&
                    failure.monitor.state().last_field_failure_time_us == 122U &&
                    failure.sample(142U) == Status::success && failure.monitor.state().action != Action::fuse &&
                    failure.sample(143U) == Status::success && failure.monitor.state().action == Action::fuse,
                profile, "each failed startup field check restarts the strict health timer");

    for (std::uint64_t age : {9U, 10U, 11U})
    {
        Fixture delayed;
        delayed.qualify(test, profile);
        test.expect(delayed.monitor.update(150U, delayed.conditions, delayed.parameters,
                                           {150U - age, delayed.source, true}) == Status::success &&
                        (delayed.monitor.state().action == Action::fuse) == (age < 10U),
                    profile, "startup age must be strictly less than one maximum interval");
    }
    Fixture horizon;
    test.expect(horizon.monitor.update(100U, horizon.conditions, horizon.parameters, {91U, horizon.source, false}) ==
                        Status::success &&
                    horizon.sample(108U) == Status::success && horizon.sample(116U) == Status::success &&
                    horizon.sample(120U) == Status::success && horizon.monitor.state().action != Action::fuse &&
                    horizon.sample(121U) == Status::success && horizon.monitor.state().action == Action::fuse &&
                    horizon.monitor.state().last_field_failure_time_us == 100U,
                profile, "health uses the check horizon, not the delayed sample timestamp");

    Fixture reset_frame;
    reset_frame.conditions.starting_allowed = false;
    for (auto time : {100U, 108U, 116U, 121U})
    {
        test.expect(reset_frame.sample(time) == Status::success && reset_frame.monitor.state().action != Action::fuse,
                    profile, "no startup on a yaw-alignment/reset frame");
    }
    reset_frame.conditions.starting_allowed = true;
    test.expect(reset_frame.sample(122U) == Status::success && reset_frame.monitor.state().action == Action::fuse,
                profile, "stable following frame may start without discarding field history");
}

void test_active_and_outages(TestContext & test, std::string_view profile)
{
    Fixture fixture;
    fixture.start(test, profile);
    test.expect(fixture.sample(130U, false) == Status::success && fixture.monitor.state().active &&
                    fixture.monitor.state().action != Action::fuse &&
                    fixture.monitor.state().stop_reason == StopReason::field_rejected &&
                    fixture.monitor.state().last_fuse_time_us == 121U && fixture.sample(131U) == Status::success &&
                    fixture.monitor.state().action == Action::fuse &&
                    fixture.monitor.record_fusion(131U, FusionDecision::fused) == Status::success,
                profile, "active disturbance inhibits correction but the next good field need not requalify startup");
    test.expect(fixture.tick(150U) == Status::success && fixture.monitor.state().active &&
                    fixture.monitor.state().action != Action::fuse && fixture.tick(151U) == Status::success &&
                    !fixture.monitor.state().active &&
                    fixture.monitor.state().stop_reason == StopReason::data_timeout &&
                    fixture.monitor.state().last_field_failure_time_us == 130U &&
                    fixture.monitor.state().last_fuse_time_us == 131U,
                profile, "no-data stop occurs at two intervals and preserves field/progress history");
    test.expect(fixture.tick(300U) == Status::success &&
                    fixture.monitor.state().recovery_reason == RecoveryReason::none &&
                    fixture.sample(301U) == Status::success && fixture.monitor.state().action == Action::fuse &&
                    fixture.monitor.record_fusion(301U, FusionDecision::fused) == Status::success &&
                    fixture.monitor.state().active && fixture.monitor.state().start_time_us == 301U,
                profile, "long outage restarts on fresh qualified data without a permanent timeout latch");

    for (bool disable : {false, true})
    {
        Fixture paused;
        paused.start(test, profile);
        paused.conditions.enabled = !disable;
        paused.conditions.ready = disable;
        test.expect(paused.sample(130U) == Status::success && !paused.monitor.state().active &&
                        paused.monitor.state().stop_reason == (disable ? StopReason::disabled : StopReason::not_ready),
                    profile, "disable/continuing-condition failure stops the session");
        paused.conditions.enabled = true;
        paused.conditions.ready = true;
        test.expect(paused.sample(131U) == Status::success && paused.monitor.state().action == Action::fuse &&
                        paused.monitor.record_fusion(131U, FusionDecision::fused) == Status::success,
                    profile, "ordinary stops preserve startup health");
    }
}

void test_timeouts(TestContext & test, std::string_view profile)
{
    for (unsigned mask = 0U; mask < 4U; ++mask)
    {
        Fixture fixture;
        fixture.start(test, profile);
        fixture.conditions.horizontal_aiding = (mask & 1U) != 0U;
        fixture.conditions.at_rest = (mask & 2U) != 0U;
        for (auto time : {131U, 141U, 151U, 161U, 170U, 171U})
        {
            test.expect(fixture.sample(time) == Status::success && fixture.monitor.state().action == Action::fuse &&
                            fixture.monitor.record_fusion(time, FusionDecision::rejected) == Status::success &&
                            fixture.monitor.state().active && fixture.monitor.state().last_fuse_time_us == 121U,
                        profile, "active gate rejections do not refresh progress; timeout equality is allowed");
        }
        test.expect(fixture.sample(172U) == Status::success && fixture.monitor.state().action == Action::fuse &&
                        fixture.monitor.record_fusion(172U, FusionDecision::rejected) == Status::success &&
                        !fixture.monitor.state().active,
                    profile, "attempt fusion before applying the unsuccessful-fusion timeout");
        bool const requires_reset = !fixture.conditions.horizontal_aiding || fixture.conditions.at_rest;
        test.expect(fixture.monitor.state().stop_reason ==
                            (requires_reset ? StopReason::recovery_required : StopReason::fusion_timeout) &&
                        fixture.monitor.state().recovery_reason ==
                            (requires_reset ? RecoveryReason::fusion_timeout : RecoveryReason::none),
                    profile, "timeout requests reset without horizontal aid or at rest; otherwise ordinary stop");
        test.expect(fixture.sample(173U) == Status::success &&
                        (fixture.monitor.state().action == Action::fuse) == !requires_reset,
                    profile, "ordinary timeout can retry; pending reset cannot acknowledge itself");
        if (!requires_reset)
        {
            test.expect(fixture.monitor.record_fusion(173U, FusionDecision::fused) == Status::success &&
                            fixture.monitor.state().active && fixture.monitor.state().start_time_us == 173U,
                        profile, "successful retry reactivates the stopped session");
        }
    }
    Fixture recovered;
    recovered.start(test, profile);
    test.expect(recovered.sample(172U) == Status::success && recovered.monitor.state().action == Action::fuse &&
                    recovered.monitor.record_fusion(172U, FusionDecision::fused) == Status::success &&
                    recovered.monitor.state().active && recovered.monitor.state().last_fuse_time_us == 172U,
                profile, "success at an overdue horizon prevents timeout instead of being discarded beforehand");
    Fixture disturbed;
    disturbed.start(test, profile);
    test.expect(disturbed.sample(172U, false) == Status::success && !disturbed.monitor.state().active &&
                    disturbed.monitor.state().stop_reason == StopReason::fusion_timeout,
                profile, "inhibited correction checks the active progress deadline");
    Fixture absent;
    absent.parameters.fusion_timeout_us = 5U;
    absent.start(test, profile);
    test.expect(absent.tick(130U) == Status::success && absent.monitor.state().active &&
                    absent.monitor.state().recovery_reason == RecoveryReason::none &&
                    absent.tick(141U) == Status::success &&
                    absent.monitor.state().stop_reason == StopReason::data_timeout,
                profile, "no-data branch uses recency, not the fusion-attempt timer");
}

void test_feedback(TestContext & test, std::string_view profile)
{
    Fixture fixture;
    fixture.qualify(test, profile);
    auto const before = fixture.monitor.state();
    test.expect(fixture.monitor.record_fusion(120U, FusionDecision::fused) == Status::out_of_range &&
                    same_state(before, fixture.monitor.state()),
                profile, "wrong-sample feedback is atomic");
    // cppcheck-suppress knownConditionTrueFalse
    test.expect(fixture.monitor.record_fusion(121U, static_cast<FusionDecision>(99)) == Status::domain_error, profile,
                "invalid-result feedback is rejected");
    test.expect(same_state(before, fixture.monitor.state()), profile, "invalid-result feedback preserves metadata");
    test.expect(fixture.monitor.record_fusion(121U, FusionDecision::fused) == Status::success &&
                    fixture.monitor.record_fusion(121U, FusionDecision::fused) == Status::out_of_range,
                profile, "one feedback consumes each permission");
    test.expect(fixture.sample(130U) == Status::success && fixture.tick(131U) == Status::success &&
                    fixture.monitor.record_fusion(130U, FusionDecision::fused) == Status::out_of_range,
                profile, "new tick revokes omitted feedback permission");
    for (bool active : {false, true})
    {
        for (auto decision : {FusionDecision::fused, FusionDecision::rejected, FusionDecision::failed})
        {
            Fixture outcome;
            outcome.qualify(test, profile);
            if (active)
            {
                test.expect(outcome.monitor.record_fusion(121U, FusionDecision::fused) == Status::success &&
                                outcome.sample(122U) == Status::success,
                            profile, "establish active feedback partition");
            }
            auto const time = active ? 122U : 121U;
            test.expect(
                outcome.monitor.record_fusion(time, decision) == Status::success &&
                    outcome.monitor.state().action == Action::none &&
                    outcome.monitor.state().last_fuse_time_us == (decision == FusionDecision::fused ? time
                                                                  : active                          ? 121U
                                                                                                    : 0U) &&
                    outcome.monitor.state().active ==
                        (decision == FusionDecision::fused || (active && decision == FusionDecision::rejected)) &&
                    outcome.monitor.state().recovery_reason ==
                        (decision == FusionDecision::failed ? RecoveryReason::fusion_failed : RecoveryReason::none),
                profile, "startup/active feedback table distinguishes permission, progress and repair");
            auto const consumed = outcome.monitor.state();
            test.expect(outcome.monitor.record_fusion(time, decision) == Status::out_of_range &&
                            same_state(consumed, outcome.monitor.state()),
                        profile, "each result consumes its instruction once");
        }
    }
}

void test_numerical_repair(TestContext & test, std::string_view profile)
{
    Fixture fixture;
    fixture.start(test, profile);
    test.expect(fixture.sample(140U) == Status::success &&
                    fixture.monitor.record_fusion(140U, FusionDecision::failed) == Status::success &&
                    fixture.monitor.state().recovery_reason == RecoveryReason::fusion_failed &&
                    fixture.monitor.state().last_fuse_time_us == 121U && fixture.sample(200U) == Status::success &&
                    fixture.monitor.state().action != Action::fuse,
                profile, "numerical failure requests repair without automatic estimator modification");
    fixture.conditions.enabled = false;
    test.expect(fixture.sample(201U) == Status::success &&
                    fixture.monitor.state().stop_reason == StopReason::disabled &&
                    fixture.monitor.state().recovery_reason == RecoveryReason::fusion_failed,
                profile, "disable cannot erase an outstanding numerical repair request");
    fixture.conditions.enabled = true;
    test.expect(fixture.sample(202U) == Status::success &&
                    fixture.monitor.state().stop_reason == StopReason::recovery_required &&
                    fixture.monitor.state().recovery_reason == RecoveryReason::fusion_failed,
                profile, "enable and fresh healthy input cannot silently acknowledge a reset");
    fixture.monitor.restart();
    fixture.qualify(test, profile, 210U);

    for (bool calibration_change : {false, true})
    {
        Fixture changed;
        changed.start(test, profile);
        if (calibration_change)
        {
            ++changed.source.calibration_count;
        }
        else
        {
            ++changed.source.device_id;
        }
        test.expect(
            changed.sample(130U) == Status::success &&
                changed.monitor.state().recovery_reason == RecoveryReason::source_changed &&
                changed.monitor.state().action != Action::fuse && changed.monitor.state().last_fuse_time_us == 0U &&
                changed.monitor.record_fusion(121U, FusionDecision::fused) == Status::out_of_range,
            profile, "source/calibration change requires reference/state reinitialization, invalidates old feedback");
        changed.monitor.restart();
        test.expect(changed.sample(130U) == Status::out_of_range, profile, "restart retains source/time watermarks");
        changed.qualify(test, profile, 140U);
    }
    for (bool failed_reset : {false, true})
    {
        Fixture repair;
        repair.start(test, profile);
        repair.conditions.horizontal_aiding = false;
        auto const reason = failed_reset ? RecoveryReason::reset_failed : RecoveryReason::fusion_failed;
        test.expect(repair.sample(172U) == Status::success &&
                        repair.monitor.record_fusion(172U, failed_reset ? FusionDecision::rejected
                                                                        : FusionDecision::failed) == Status::success,
                    profile, "establish persistent numerical repair case");
        if (failed_reset)
        {
            test.expect(repair.monitor.record_reset(172U, false) == Status::success, profile,
                        "record actual failed reset");
        }
        ++repair.source.device_id;
        repair.conditions.flight_state = MagnetometerFusion::FlightState::on_ground;
        for (auto time : {180U, 188U, 196U, 204U})
        {
            test.expect(repair.sample(time) == Status::success && repair.monitor.state().recovery_reason == reason &&
                            repair.monitor.state().action == Action::none,
                        profile, "healthy replacement source cannot repair estimator numerical failure");
        }
        repair.monitor.restart();
        test.expect(repair.sample(204U) == Status::out_of_range, profile,
                    "repair acknowledgment preserves consumed time watermark");
        auto changed = repair.parameters;
        ++changed.fusion_timeout_us;
        test.expect(repair.monitor.update(205U, repair.conditions, changed, {205U, repair.source, true}) ==
                        Status::domain_error,
                    profile, "restart preserves configured durations rather than reconfiguring the controller");
        repair.qualify(test, profile, 210U);
        test.expect(repair.monitor.state().recovery_reason == RecoveryReason::none && !repair.monitor.state().active &&
                        repair.monitor.state().last_fuse_time_us == 0U &&
                        repair.monitor.state().last_reset_time_us == 0U,
                    profile, "explicit safe re-entry requires new health/count and creates no estimator progress");
        repair.monitor.reset(repair.source);
        test.expect(same_state(repair.monitor.state(), MagnetometerFusion::State{}) &&
                        repair.sample(1U) == Status::success && repair.monitor.state().filter_reset &&
                        repair.monitor.state().recovery_reason == RecoveryReason::none,
                    profile,
                    "full reset with seeded source clears timeline and repair without treating first sample as source "
                    "replacement");
    }
}

void test_source_reinitialization(TestContext & test, std::string_view profile)
{
    using FlightState = MagnetometerFusion::FlightState;
    for (bool calibration : {false, true})
    {
        for (auto flight : {FlightState::unknown, FlightState::on_ground, FlightState::in_air})
        {
            for (bool aided : {false, true})
            {
                Fixture fixture;
                fixture.start(test, profile);
                fixture.conditions.horizontal_aiding = aided;
                fixture.conditions.flight_state = flight;
                if (calibration)
                {
                    ++fixture.source.calibration_count;
                }
                else
                {
                    ++fixture.source.device_id;
                }
                test.expect(fixture.sample(130U) == Status::success && fixture.monitor.state().filter_reset &&
                                fixture.monitor.state().recovery_reason == RecoveryReason::source_changed &&
                                fixture.monitor.state().last_field_failure_time_us == 100U &&
                                fixture.monitor.state().last_fuse_time_us == 0U &&
                                fixture.monitor.state().action == Action::none &&
                                fixture.monitor.record_fusion(121U, FusionDecision::fused) == Status::out_of_range,
                            profile,
                            "new source reseeds LPF/count/session and revokes old feedback; field health persists");
                for (auto time : {131U, 132U, 133U})
                {
                    test.expect(fixture.sample(time) == Status::success && !fixture.monitor.state().filter_reset,
                                profile, "subsequent source samples retain LPF history");
                }
                bool const reset = flight != FlightState::unknown && (flight == FlightState::on_ground || !aided);
                bool const fuse = flight == FlightState::in_air && aided;
                auto const & state = fixture.monitor.state();
                test.expect((state.action == Action::reset) == reset && (state.action == Action::fuse) == fuse &&
                                !state.active,
                            profile, "four samples: ground/unaided realign, aided flight preserves yaw, unknown waits");
                if (reset)
                {
                    test.expect(fixture.monitor.record_reset(133U, true) == Status::success && state.active &&
                                    state.start_time_us == 133U && state.last_fuse_time_us == 0U &&
                                    state.last_reset_time_us == 133U,
                                profile, "source heading reset starts a new session without claiming fusion");
                }
                else if (fuse)
                {
                    test.expect(fixture.monitor.record_fusion(133U, FusionDecision::fused) == Status::success &&
                                    state.active && state.last_reset_time_us == 0U,
                                profile, "aided in-air source re-enters through actual correction, no yaw reset");
                }
                else
                {
                    fixture.conditions.flight_state = FlightState::on_ground;
                    test.expect(fixture.sample(134U) == Status::success && state.action == Action::reset, profile,
                                "known ground state resolves pending unknown flight context");
                }
            }
        }
    }
    Fixture fixture;
    fixture.start(test, profile);
    fixture.conditions.flight_state = FlightState::on_ground;
    fixture.conditions.ready = false;
    ++fixture.source.device_id;
    test.expect(fixture.sample(130U) == Status::success && fixture.monitor.state().filter_reset, profile,
                "source reseed does not require attitude readiness");
    fixture.conditions.ready = true;
    fixture.conditions.flight_state = FlightState::in_air;
    test.expect(fixture.sample(131U) == Status::success && fixture.sample(132U) == Status::success &&
                    fixture.sample(133U) == Status::success && fixture.monitor.state().action == Action::reset,
                profile, "ground change keeps its alignment requirement after takeoff/readiness");
    test.expect(fixture.monitor.record_reset(133U, false) == Status::success, profile, "record real reset failure");
    ++fixture.source.device_id;
    test.expect(fixture.sample(134U) == Status::success && fixture.sample(200U) == Status::success &&
                    fixture.monitor.state().recovery_reason == RecoveryReason::reset_failed &&
                    fixture.monitor.state().action != Action::reset && fixture.monitor.state().action != Action::fuse,
                profile, "another source cannot silently repair a numerical failure");
}

void test_reference_change(TestContext & test, std::string_view profile)
{
    for (bool aided : {false, true})
    {
        for (bool at_rest : {false, true})
        {
            Fixture fixture;
            fixture.start(test, profile);
            fixture.conditions.horizontal_aiding = aided;
            fixture.conditions.at_rest = at_rest;
            ++fixture.source.reference_id;
            bool const reset = !aided || at_rest;
            test.expect(fixture.sample(130U) == Status::success && !fixture.monitor.state().filter_reset &&
                            (fixture.monitor.state().action == Action::reset) == reset &&
                            (fixture.monitor.state().action == Action::fuse) == !reset &&
                            fixture.monitor.state().last_fuse_time_us == 121U,
                        profile, "reference-only update retains sensor history; aiding/rest selects heading reset");
            if (reset)
            {
                test.expect(fixture.monitor.state().recovery_reason == RecoveryReason::reference_changed &&
                                fixture.monitor.record_reset(130U, true) == Status::success &&
                                fixture.monitor.state().last_reset_time_us == 130U &&
                                fixture.monitor.state().start_time_us == 121U,
                            profile, "reference reset preserves the existing session and actual fusion history");
            }
        }
    }
    Fixture unhealthy;
    unhealthy.start(test, profile);
    unhealthy.conditions.horizontal_aiding = false;
    ++unhealthy.source.reference_id;
    test.expect(unhealthy.sample(130U, false) == Status::success && unhealthy.monitor.state().action != Action::reset &&
                    unhealthy.sample(150U) == Status::success && unhealthy.monitor.state().action != Action::reset &&
                    unhealthy.sample(151U) == Status::success && unhealthy.monitor.state().action == Action::reset,
                profile, "new reference still requires strictly elapsed field health, not just a revision change");
    auto pending = unhealthy;
    pending.conditions.horizontal_aiding = true;
    test.expect(pending.sample(152U) == Status::success && pending.monitor.state().action != Action::reset &&
                    pending.monitor.state().recovery_reason == RecoveryReason::none &&
                    pending.monitor.state().stop_reason == StopReason::none,
                profile, "restored aiding cancels pending reference reset without reporting a timeout");
    enum class Existing
    {
        numerical,
        source,
        timeout
    };
    for (auto existing : {Existing::numerical, Existing::source, Existing::timeout})
    {
        Fixture priority;
        priority.start(test, profile);
        priority.conditions.horizontal_aiding = false;
        std::uint64_t time = 130U;
        RecoveryReason reason = RecoveryReason::fusion_failed;
        if (existing == Existing::numerical)
        {
            test.expect(priority.sample(time) == Status::success &&
                            priority.monitor.record_fusion(time, FusionDecision::failed) == Status::success,
                        profile, "establish numerical failure before reference revision");
        }
        else if (existing == Existing::source)
        {
            ++priority.source.device_id;
            reason = RecoveryReason::source_changed;
            test.expect(priority.sample(time) == Status::success, profile,
                        "establish source recovery before reference revision");
        }
        else
        {
            time = 172U;
            reason = RecoveryReason::fusion_timeout;
            test.expect(priority.sample(time) == Status::success &&
                            priority.monitor.record_fusion(time, FusionDecision::rejected) == Status::success,
                        profile, "establish timeout recovery before reference revision");
        }
        ++priority.source.reference_id;
        test.expect(priority.sample(time + 1U) == Status::success &&
                        priority.monitor.state().recovery_reason == reason && !priority.monitor.state().filter_reset,
                    profile,
                    "reference-only change cannot overwrite an existing numerical/source/timeout recovery reason");
    }
}

void test_reset_feedback(TestContext & test, std::string_view profile)
{
    Fixture fixture;
    fixture.conditions.horizontal_aiding = false;
    fixture.start(test, profile);
    test.expect(fixture.sample(172U) == Status::success &&
                    fixture.monitor.record_fusion(172U, FusionDecision::rejected) == Status::success &&
                    fixture.monitor.state().action == Action::reset,
                profile, "healthy overdue rejection permits actual heading reset");
    auto const before = fixture.monitor.state();
    test.expect(fixture.monitor.record_reset(171U, true) == Status::out_of_range &&
                    same_state(before, fixture.monitor.state()),
                profile, "wrong-sample reset feedback is atomic");
    test.expect(fixture.monitor.record_reset(172U, true) == Status::success && fixture.monitor.state().active &&
                    fixture.monitor.state().last_reset_time_us == 172U &&
                    fixture.monitor.state().last_fuse_time_us == 121U &&
                    fixture.monitor.state().start_time_us == 121U &&
                    fixture.monitor.state().recovery_reason == RecoveryReason::none &&
                    fixture.monitor.record_reset(172U, true) == Status::out_of_range &&
                    fixture.monitor.record_fusion(172U, FusionDecision::fused) == Status::out_of_range,
                profile, "one reset restores the session without fabricated fusion or second correction");
    test.expect(fixture.sample(222U) == Status::success &&
                    fixture.monitor.record_fusion(222U, FusionDecision::rejected) == Status::success &&
                    fixture.monitor.state().action != Action::reset && fixture.monitor.state().active &&
                    fixture.sample(223U) == Status::success &&
                    fixture.monitor.record_fusion(223U, FusionDecision::rejected) == Status::success &&
                    fixture.monitor.state().action == Action::reset &&
                    fixture.monitor.record_reset(223U, false) == Status::success &&
                    fixture.monitor.state().recovery_reason == RecoveryReason::reset_failed &&
                    fixture.monitor.state().last_reset_time_us == 172U && fixture.sample(224U) == Status::success &&
                    fixture.monitor.state().action != Action::reset,
                profile, "reset refreshes timeout; failed reset requires caller repair without retry loop");
    for (bool success : {false, true})
    {
        Fixture source;
        source.start(test, profile);
        source.conditions.flight_state = MagnetometerFusion::FlightState::on_ground;
        ++source.source.device_id;
        for (auto time : {130U, 131U, 132U, 133U})
        {
            test.expect(source.sample(time) == Status::success, profile, "qualify reset for a physically new source");
        }
        test.expect(source.monitor.state().action == Action::reset &&
                        source.monitor.record_reset(133U, success) == Status::success &&
                        source.monitor.state().start_time_us == (success ? 133U : 0U) &&
                        source.monitor.state().last_reset_time_us == (success ? 133U : 0U) &&
                        source.monitor.state().last_fuse_time_us == 0U &&
                        source.monitor.state().recovery_reason ==
                            (success ? RecoveryReason::none : RecoveryReason::reset_failed),
                    profile, "source reset starts new session only on actual success without fabricating fusion");
    }
}

void test_reset_qualification(TestContext & test, std::string_view profile)
{
    Fixture disturbed;
    disturbed.conditions.horizontal_aiding = false;
    disturbed.start(test, profile);
    test.expect(disturbed.sample(172U, false) == Status::success && disturbed.monitor.state().action != Action::reset &&
                    disturbed.sample(191U) == Status::success && disturbed.monitor.state().action != Action::reset &&
                    disturbed.sample(192U) == Status::success && disturbed.monitor.state().action != Action::reset &&
                    disturbed.sample(193U) == Status::success && disturbed.monitor.state().action == Action::reset,
                profile, "bad field must requalify health before recovery reset");
    test.expect(disturbed.tick(194U) == Status::success &&
                    disturbed.monitor.record_reset(193U, true) == Status::out_of_range,
                profile, "no-data tick revokes omitted reset feedback");
    disturbed.conditions.starting_allowed = false;
    test.expect(disturbed.sample(195U) == Status::success && disturbed.monitor.state().action != Action::reset, profile,
                "external reset/yaw-alignment frame cannot reset again");
    disturbed.conditions.starting_allowed = true;
    test.expect(disturbed.monitor.update(206U, disturbed.conditions, disturbed.parameters,
                                         {196U, disturbed.source, true}) == Status::success &&
                    disturbed.monitor.state().action != Action::reset,
                profile, "stale sample cannot authorize recovery reset");
    disturbed.conditions.ready = false;
    test.expect(disturbed.sample(207U) == Status::success && disturbed.monitor.state().action != Action::reset, profile,
                "lost readiness preserves pending recovery without a reset permission");
    disturbed.conditions.ready = true;
    disturbed.conditions.horizontal_aiding = true;
    test.expect(disturbed.sample(208U) == Status::success && disturbed.monitor.state().action != Action::reset &&
                    disturbed.monitor.state().recovery_reason == RecoveryReason::none &&
                    disturbed.monitor.state().stop_reason == StopReason::fusion_timeout,
                profile, "restored horizontal aid cancels timeout reset in favor of ordinary stop");
}

void test_disabled_health_wait(TestContext & test, std::string_view profile)
{
    Fixture fixture;
    fixture.parameters.health_time_us = 0U;
    for (auto time : {100U, 101U, 102U})
    {
        test.expect(fixture.sample(time) == Status::success && fixture.monitor.state().action != Action::fuse, profile,
                    "disabled health wait still requires four samples");
    }
    test.expect(fixture.sample(103U, false) == Status::success && fixture.monitor.state().action != Action::fuse,
                profile, "disabled health wait cannot bypass a current field rejection");
    test.expect(fixture.monitor.update(114U, fixture.conditions, fixture.parameters, {104U, fixture.source, true}) ==
                        Status::success &&
                    fixture.monitor.state().action != Action::fuse &&
                    fixture.monitor.state().stop_reason == StopReason::data_timeout,
                profile, "disabled health wait still requires a fresh sample");
    fixture.conditions.starting_allowed = false;
    test.expect(fixture.sample(115U) == Status::success && fixture.monitor.state().action != Action::fuse, profile,
                "disabled health wait preserves startup inhibition");
    fixture.conditions.starting_allowed = true;
    fixture.conditions.ready = false;
    test.expect(fixture.sample(116U) == Status::success && fixture.monitor.state().action != Action::fuse, profile,
                "disabled health wait preserves alignment prerequisites");
    fixture.conditions.ready = true;
    fixture.conditions.heading_consistent = false;
    test.expect(fixture.sample(117U) == Status::success && fixture.monitor.state().action != Action::fuse, profile,
                "disabled health wait preserves aided heading consistency");
    fixture.conditions.heading_consistent = true;
    test.expect(fixture.sample(118U) == Status::success && fixture.monitor.state().action == Action::fuse &&
                    fixture.monitor.record_fusion(118U, FusionDecision::fused) == Status::success,
                profile, "qualified sample can start without the health delay");
    test.expect(fixture.sample(119U, false) == Status::success && fixture.monitor.state().active &&
                    fixture.monitor.state().action != Action::fuse && fixture.sample(120U) == Status::success &&
                    fixture.monitor.state().action == Action::fuse,
                profile, "brief disturbance retains the session and recovery can retry immediately");

    fixture.conditions.flight_state = MagnetometerFusion::FlightState::on_ground;
    ++fixture.source.device_id;
    for (auto time : {121U, 122U, 123U})
    {
        test.expect(fixture.sample(time) == Status::success && fixture.monitor.state().action != Action::reset, profile,
                    "source change still needs four samples before heading reset");
    }
    test.expect(fixture.sample(124U, false) == Status::success && fixture.monitor.state().action != Action::reset &&
                    fixture.sample(125U) == Status::success && fixture.monitor.state().action == Action::reset,
                profile, "source reset needs a good field but no disabled health delay");

    Fixture alignment;
    alignment.parameters.health_time_us = 0U;
    alignment.conditions.ready = false;
    for (auto time : {100U, 101U, 102U})
    {
        test.expect(alignment.sample(time) == Status::success, profile, "unaligned samples only establish the count");
    }
    alignment.conditions.ready = true;
    test.expect(alignment.sample(103U) == Status::success && alignment.monitor.state().action == Action::fuse &&
                    alignment.monitor.state().last_field_failure_time_us == 103U,
                profile, "zero duration permits the first health check once other prerequisites hold");
}

void test_invalid_calls(TestContext & test, std::string_view profile)
{
    enum class Invalid
    {
        disable_health_wait,
        zero_interval,
        zero_timeout,
        changed_health,
        changed_interval,
        changed_timeout,
        repeated_horizon,
        repeated_sample,
        future_sample,
        zero_source,
        zero_horizon,
        backward_horizon,
        doubled_interval_overflow
    };
    for (auto invalid_case :
         {Invalid::disable_health_wait, Invalid::zero_interval, Invalid::zero_timeout, Invalid::changed_health,
          Invalid::changed_interval, Invalid::changed_timeout, Invalid::repeated_horizon, Invalid::repeated_sample,
          Invalid::future_sample, Invalid::zero_source, Invalid::zero_horizon, Invalid::backward_horizon,
          Invalid::doubled_interval_overflow})
    {
        Fixture fixture;
        fixture.start(test, profile);
        auto parameters = fixture.parameters;
        MagnetometerFusion::Sample sample{130U, fixture.source, true};
        std::uint64_t now = 130U;
        Status expected = Status::domain_error;
        switch (invalid_case)
        {
        case Invalid::disable_health_wait:
            // Zero is valid at initialization, but durations cannot change mid-session.
            parameters.health_time_us = 0U;
            break;
        case Invalid::zero_interval:
            parameters.maximum_interval_us = 0U;
            break;
        case Invalid::zero_timeout:
            parameters.fusion_timeout_us = 0U;
            break;
        case Invalid::changed_health:
            ++parameters.health_time_us;
            break;
        case Invalid::changed_interval:
            ++parameters.maximum_interval_us;
            break;
        case Invalid::changed_timeout:
            ++parameters.fusion_timeout_us;
            break;
        case Invalid::repeated_horizon:
            now = 121U;
            expected = Status::out_of_range;
            break;
        case Invalid::repeated_sample:
            sample.time_us = 121U;
            expected = Status::out_of_range;
            break;
        case Invalid::future_sample:
            sample.time_us = 131U;
            expected = Status::out_of_range;
            break;
        case Invalid::zero_source:
            sample.source.device_id = 0U;
            break;
        case Invalid::zero_horizon:
            now = 0U;
            expected = Status::out_of_range;
            break;
        case Invalid::backward_horizon:
            now = 120U;
            expected = Status::out_of_range;
            break;
        case Invalid::doubled_interval_overflow:
            parameters.maximum_interval_us = std::numeric_limits<std::uint64_t>::max();
            break;
        }
        auto expected_state = fixture.monitor.state();
        expected_state.active = false;
        expected_state.action = Action::none;
        expected_state.filter_reset = false;
        expected_state.stop_reason = StopReason::invalid_call;
        test.expect(fixture.monitor.update(now, fixture.conditions, parameters, sample) == expected &&
                        same_state(fixture.monitor.state(), expected_state),
                    profile,
                    "invalid call revokes permission but does not consume time/config/source or rewrite history");
        test.expect(fixture.sample(130U) == Status::success && fixture.monitor.state().action == Action::fuse, profile,
                    "valid retry can consume the timestamp rejected by an invalid call");
    }
    Fixture combined;
    combined.start(test, profile);
    auto bad_parameters = combined.parameters;
    bad_parameters.maximum_interval_us = 0U;
    test.expect(combined.monitor.update(0U, combined.conditions, bad_parameters, {}) == Status::domain_error &&
                    !combined.monitor.state().filter_reset &&
                    combined.monitor.record_fusion(121U, FusionDecision::fused) == Status::out_of_range &&
                    combined.sample(130U) == Status::success,
                profile, "duration validation precedes invalid time and rejected update consumes no timestamp");
    auto retained = combined.monitor.state();
    retained.active = false;
    retained.action = Action::none;
    retained.filter_reset = false;
    retained.stop_reason = StopReason::invalid_call;
    combined.conditions.flight_state = static_cast<MagnetometerFusion::FlightState>(99);
    test.expect(combined.sample(131U) == Status::domain_error && same_state(combined.monitor.state(), retained),
                profile, "invalid flight context revokes permission but preserves history");
    combined.conditions.flight_state = MagnetometerFusion::FlightState::unknown;
    test.expect(combined.monitor.update(131U, combined.conditions, combined.parameters, {0U, {0U, 99U, 99U}, false}) ==
                        Status::success &&
                    !combined.monitor.state().filter_reset && combined.sample(132U) == Status::success &&
                    !combined.monitor.state().filter_reset,
                profile, "absent sample ignores invalid payload without changing source identity");
}

void test_count_and_time(TestContext & test, std::string_view profile)
{
    constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
    Fixture large;
    large.start(test, profile, maximum - 30U);
    test.expect(large.sample(maximum) == Status::success && large.monitor.state().action == Action::fuse &&
                    large.monitor.record_fusion(maximum, FusionDecision::fused) == Status::success &&
                    large.tick(0U) == Status::out_of_range,
                profile, "elapsed arithmetic remains bounded near UINT64_MAX");
    Fixture duration;
    duration.parameters = {maximum, maximum / 2U, maximum};
    test.expect(duration.sample(1U) == Status::success && duration.tick(maximum) == Status::success &&
                    duration.monitor.state().stop_reason == StopReason::data_timeout &&
                    duration.monitor.state().recovery_reason == RecoveryReason::none,
                profile, "doubled interval is representable and health cannot overflow into permission");
    duration.monitor.reset();
    test.expect(duration.sample(1U) == Status::success && duration.sample(2U) == Status::success &&
                    duration.sample(3U) == Status::success && duration.sample(maximum) == Status::success &&
                    duration.monitor.state().action != Action::fuse,
                profile, "four samples cannot overflow an unreachable health duration into qualification");
    duration.monitor.reset();
    duration.parameters = {20U, 10U, 50U};
    duration.qualify(test, profile, 1U);
}

void test_start_qualification(TestContext & test, std::string_view profile)
{
    test_startup(test, profile);
    test_disabled_health_wait(test, profile);
    for (bool seeded : {false, true})
    {
        Fixture fixture;
        fixture.monitor.reset(seeded ? fixture.source : MagnetometerFusion::Source{});
        test.expect(same_state(fixture.monitor.state(), MagnetometerFusion::State{}), profile,
                    "seeded identity alone creates no session, count or progress");
        for (std::uint64_t index = 0U; index < 5U; ++index)
        {
            test.expect(fixture.sample(100U + index * 8U) == Status::success &&
                            fixture.monitor.state().filter_reset == (index == 0U) && !fixture.monitor.state().active &&
                            fixture.monitor.state().start_time_us == 0U &&
                            fixture.monitor.state().last_fuse_time_us == 0U,
                        profile, "first enabled sample requests one filter seed without fake fusion progress");
        }
    }
}

void test_heading_alignment(TestContext & test, std::string_view profile)
{
    test_heading_policy(test, profile);
    for (bool source_change : {false, true})
    {
        Fixture fixture;
        fixture.start(test, profile);
        fixture.conditions.heading_aligned = false;
        fixture.conditions.heading_consistent = false;
        if (source_change)
        {
            ++fixture.source.device_id;
        }
        for (std::uint64_t time = 130U; time <= 133U; ++time)
        {
            test.expect(fixture.sample(time) == Status::success && fixture.monitor.state().action != Action::fuse,
                        profile, "lost alignment never grants ordinary correction");
        }
        test.expect((fixture.monitor.state().action == Action::reset) == !source_change, profile,
                    "unknown flight inhibits source realignment, not independently lost existing alignment");
        fixture.conditions.flight_state = MagnetometerFusion::FlightState::in_air;
        test.expect(fixture.sample(134U) == Status::success && fixture.monitor.state().action == Action::reset &&
                        fixture.monitor.record_reset(134U, true) == Status::success,
                    profile, "known source context permits explicit qualified alignment reset");
        fixture.conditions.heading_aligned = true;
        fixture.conditions.heading_consistent = true;
        test.expect(fixture.sample(135U) == Status::success && fixture.monitor.state().action == Action::fuse, profile,
                    "caller restores alignment only after actual reset acknowledgment");
        test.expect(fixture.monitor.record_fusion(135U, FusionDecision::failed) == Status::success, profile,
                    "establish numerical repair latch");
        fixture.conditions.heading_aligned = false;
        test.expect(fixture.sample(136U) == Status::success && fixture.monitor.state().action == Action::none &&
                        fixture.monitor.state().recovery_reason == RecoveryReason::fusion_failed,
                    profile, "lost alignment cannot bypass numerical repair");
    }
}

void test_snapshot_recovery(TestContext & test, std::string_view profile)
{
    for (bool reference_change : {false, true})
    {
        Fixture fixture;
        fixture.conditions.horizontal_aiding = false;
        fixture.start(test, profile);
        if (reference_change)
        {
            ++fixture.source.reference_id;
            test.expect(fixture.sample(172U) == Status::success, profile, "grant reference-reset instruction");
        }
        else
        {
            test.expect(fixture.sample(172U) == Status::success &&
                            fixture.monitor.record_fusion(172U, FusionDecision::rejected) == Status::success,
                        profile, "grant timeout-reset instruction");
        }
        auto const reason = reference_change ? RecoveryReason::reference_changed : RecoveryReason::fusion_timeout;
        test.expect(fixture.monitor.state().action == Action::reset &&
                        fixture.monitor.state().recovery_reason == reason,
                    profile, "pending recovery has a distinct instruction and persistent reason");
        auto const before = fixture.monitor.state();
        for (unsigned read = 0U; read < 8U; ++read)
        {
            test.expect(same_state(fixture.monitor.state(), before), profile, "state snapshots are pure");
        }
        enum class Display
        {
            disabled,
            not_ready,
            no_data
        };
        for (auto mode : {Display::disabled, Display::not_ready, Display::no_data})
        {
            auto inhibited = fixture;
            inhibited.conditions.enabled = mode != Display::disabled;
            inhibited.conditions.ready = mode != Display::not_ready;
            auto const status = mode == Display::no_data ? inhibited.tick(173U) : inhibited.sample(173U);
            test.expect(status == Status::success && inhibited.monitor.state().action == Action::none &&
                            inhibited.monitor.state().recovery_reason == reason &&
                            inhibited.monitor.state().last_fuse_time_us == before.last_fuse_time_us &&
                            inhibited.monitor.state().last_reset_time_us == before.last_reset_time_us &&
                            inhibited.monitor.record_reset(172U, true) == Status::out_of_range,
                        profile, "display inhibition revokes permission without erasing pending reason or progress");
        }
    }
}

void test_saturation(TestContext & test, std::string_view profile)
{
    test_count_and_time(test, profile);
    Fixture fixture;
    fixture.parameters.health_time_us = 0U;
    fixture.conditions.enabled = false;
    for (std::uint64_t time = 1U; time <= 8U; ++time)
    {
        test.expect(fixture.sample(time) == Status::success && !fixture.monitor.state().filter_reset &&
                        fixture.monitor.state().action == Action::none,
                    profile, "disabled samples cannot contribute to enabled startup count");
    }
    fixture.conditions.enabled = true;
    for (std::uint64_t count = 1U; count <= 300U; ++count)
    {
        test.expect(fixture.sample(8U + count) == Status::success &&
                        fixture.monitor.state().filter_reset == (count == 1U) &&
                        (fixture.monitor.state().action == Action::fuse) == (count >= 4U),
                    profile, "long sequence retains four-sample eligibility without counter wrap or repeated reseed");
    }
    ++fixture.source.reference_id;
    test.expect(fixture.sample(309U) == Status::success && fixture.monitor.state().action == Action::fuse &&
                    !fixture.monitor.state().filter_reset,
                profile, "reference revision does not discard physical-source count");
    ++fixture.source.device_id;
    fixture.conditions.flight_state = MagnetometerFusion::FlightState::in_air;
    for (std::uint64_t count = 1U; count <= 4U; ++count)
    {
        test.expect(fixture.sample(309U + count) == Status::success &&
                        fixture.monitor.state().filter_reset == (count == 1U) &&
                        (fixture.monitor.state().action == Action::fuse) == (count == 4U),
                    profile, "new physical source requires exactly four new samples");
    }
    // Exact private count saturation is a source/formal property. Native checks
    // observe eligibility and reseed behavior through long public sequences.
}

template <typename Matrix> bool same_matrix(Matrix const & a, Matrix const & b)
{
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            if (a(row, column) != b(row, column))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename Linalg, typename Configuration>
void test_real_fusion(TestContext & test, std::string_view scalar_profile)
{
    using value_type = typename Linalg::value_type;
    using vector_type = formal_eskf::linalg::Matrix<Linalg, 3U, 1U>;
    using matrix_type = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    typename Configuration::template NominalState<Linalg> state;
    formal_eskf::linalg::Matrix<Linalg, Configuration::error_state_dimension, Configuration::error_state_dimension> P;
    auto const reference = vector_type::from_row_major({value_type{30}, value_type{0}, value_type{40}});
    auto V = matrix_type::identity();
    formal_eskf::runtime::MagneticFieldParameters<value_type> const field_parameters{
        value_type{5}, static_cast<value_type>(0.2), static_cast<value_type>(0.2), value_type{1}};
    Fixture fixture;
    auto step = [&](std::uint64_t time, vector_type const & measurement)
    {
        auto const field = formal_eskf::runtime::check_magnetic_field(state.q_nb, measurement, reference, &reference,
                                                                      field_parameters, false);
        test.expect(fixture.sample(time, formal_eskf::succeeded(field.status) && field.diagnostics_valid &&
                                             field.accepted) == Status::success,
                    scalar_profile, "use actual same-prior field check");
        auto decision = FusionDecision::rejected;
        if (fixture.monitor.state().action == Action::fuse)
        {
            auto const result = formal_eskf::runtime::fuse_magnetometer(
                state, P, measurement, reference, &reference, V, field_parameters, false, static_cast<value_type>(1e-6),
                value_type{3}, state, P);
            decision = result.fusion.decision;
            test.expect(fixture.monitor.record_fusion(time, decision) == Status::success, scalar_profile,
                        "use actual gated transaction feedback");
        }
        return decision;
    };
    test.expect(
        step(100U, reference) == FusionDecision::rejected && step(108U, reference) == FusionDecision::rejected &&
            step(116U, reference) == FusionDecision::rejected && step(121U, reference) == FusionDecision::fused &&
            fixture.monitor.state().active && fixture.monitor.state().last_fuse_time_us == 121U,
        scalar_profile, "actual AHRS/INS fusion starts only after qualification");
    auto const before_q = state.q_nb.coefficients();
    auto const before_P = P;
    test.expect(step(130U, reference * value_type{2}) == FusionDecision::rejected && fixture.monitor.state().active &&
                    fixture.monitor.state().action != Action::fuse &&
                    same_matrix(state.q_nb.coefficients(), before_q) && same_matrix(P, before_P) &&
                    step(131U, reference) == FusionDecision::fused,
                scalar_profile, "disturbance preserves outputs; active session resumes on the next good field");
    auto const outlier = vector_type::from_row_major({value_type{0}, value_type{30}, value_type{40}});
    for (auto time : {141U, 151U, 161U, 171U, 181U})
    {
        test.expect(step(time, outlier) == FusionDecision::rejected && fixture.monitor.state().active &&
                        fixture.monitor.state().last_fuse_time_us == 131U,
                    scalar_profile, "actual gate rejection cannot renew progress");
    }
    test.expect(step(182U, outlier) == FusionDecision::rejected && !fixture.monitor.state().active &&
                    fixture.monitor.state().stop_reason == StopReason::fusion_timeout &&
                    fixture.monitor.state().recovery_reason == RecoveryReason::none &&
                    same_matrix(state.q_nb.coefficients(), before_q) && same_matrix(P, before_P),
                scalar_profile, "aided timeout stops without changing state/P or permanently latching");
    test.expect(step(183U, reference) == FusionDecision::fused && fixture.monitor.state().active, scalar_profile,
                "next qualified actual correction reactivates without fabricated reset");
    V(0U, 0U) = value_type{-1};
    test.expect(step(190U, reference) == FusionDecision::failed &&
                    fixture.monitor.state().recovery_reason == RecoveryReason::fusion_failed &&
                    fixture.monitor.state().last_fuse_time_us == 183U &&
                    same_matrix(state.q_nb.coefficients(), before_q) && same_matrix(P, before_P),
                scalar_profile, "actual numerical error requests repair without modifying state/P or progress");
}

template <typename State> bool same_estimator_state(State const & a, State const & b)
{
    bool result = same_matrix(a.q_nb.coefficients(), b.q_nb.coefficients());
    if constexpr (requires { a.p_n; })
    {
        result = result && same_matrix(a.p_n, b.p_n) && same_matrix(a.v_n, b.v_n) && same_matrix(a.b_a, b.b_a) &&
                 same_matrix(a.b_g, b.b_g);
    }
    return result;
}

template <typename Linalg, typename Configuration> struct MagneticCompositionReference
{
    using value_type = typename Linalg::value_type;
    using types = formal_eskf::EskfTypes<Linalg, Configuration>;
    using state_type = typename types::nominal_state_type;
    using covariance_type = typename types::error_covariance_type;
    using vector_type = formal_eskf::linalg::Vector<Linalg, 3U>;
    using matrix3_type = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    static constexpr bool is_ins = std::is_same_v<Configuration, formal_eskf::configuration::Ins>;
    static constexpr std::size_t size = types::error_state_dimension;
    static constexpr std::size_t attitude_offset = is_ins ? 6U : 0U;
    static constexpr std::size_t measurement_size = 3U;
    state_type state;
    covariance_type P;
    formal_eskf::linalg::Matrix<Linalg, 3U, size> H{};
    vector_type r{};
    matrix3_type V;

    MagneticCompositionReference(state_type const & prior, covariance_type const & covariance, vector_type const & raw,
                                 vector_type const & field_n, matrix3_type const & noise)
        : state(prior), P(covariance), V(noise)
    {
        long double const w = state.q_nb.q0(), x = state.q_nb.q1(), y = state.q_nb.q2(), z = state.q_nb.q3();
        long double const R[3][3]{{w * w + x * x - y * y - z * z, 2 * (x * y - w * z), 2 * (x * z + w * y)},
                                  {2 * (x * y + w * z), w * w - x * x + y * y - z * z, 2 * (y * z - w * x)},
                                  {2 * (x * z - w * y), 2 * (y * z + w * x), w * w - x * x - y * y + z * z}};
        std::array<long double, 3U> h{};
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            for (std::size_t column = 0U; column < 3U; ++column)
            {
                h[row] += R[column][row] * field_n(column);
            }
            r(row) = static_cast<value_type>(static_cast<long double>(raw(row)) - h[row]);
        }
        long double const skew[3][3]{{0, -h[2], h[1]}, {h[2], 0, -h[0]}, {-h[1], h[0], 0}};
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            for (std::size_t column = 0U; column < 3U; ++column)
            {
                H(row, attitude_offset + column) = static_cast<value_type>(skew[row][column]);
            }
        }
    }
};

template <typename Linalg, typename Configuration>
void test_stage_composition(TestContext & test, std::string_view profile)
{
    using T = typename Linalg::value_type;
    using Reference = MagneticCompositionReference<Linalg, Configuration>;
    using Vector = typename Reference::vector_type;
    using Matrix = typename Reference::matrix3_type;
    using Heading = formal_eskf::runtime::MagneticHeading<Linalg>;
    enum class Outcome
    {
        fused,
        field_rejected,
        gate_rejected,
        correction_failed
    };
    for (bool health_available : {false, true})
    {
        for (auto outcome :
             {Outcome::fused, Outcome::field_rejected, Outcome::gate_rejected, Outcome::correction_failed})
        {
            typename Reference::state_type state;
            auto P = Reference::covariance_type::identity();
            formal_eskf::runtime::MagneticReference<Linalg> model;
            test.expect(model.update(1U, {T{0}, T{0}, T{32}}, true).changed, profile,
                        "adopt actual observation model before qualification");
            auto const field = model.state().field_n;
            auto const independent_health = Vector::from_row_major({T{0}, T{32}, T{0}});
            auto const * health = health_available ? &independent_health : nullptr;
            formal_eskf::runtime::MagneticFieldParameters<T> const field_parameters{T{1}, T{0.25}, T{0.5}, T{1}};
            typename Heading::Parameters const heading_parameters{T{0.5}, T{0}, T{1}};
            Heading heading;
            test.expect(heading.update_acceleration(Vector::from_row_major({T{8}, T{0}, T{0}}), T{0.25}) ==
                            Status::success,
                        profile, "independent horizontal excitation enables heading observability");
            Fixture lifecycle;
            lifecycle.source.reference_id = model.state().reference_id;
            for (std::uint64_t time : {100U, 108U, 116U})
            {
                auto const check = formal_eskf::runtime::check_magnetic_field(state.q_nb, field, field, health,
                                                                              field_parameters, false);
                test.expect(check.status == Status::success && check.accepted &&
                                heading.update(state.q_nb, field, field, heading_parameters, true, true) ==
                                    Status::success,
                            profile, "same-prior reference and field/heading producers qualify baseline data");
                lifecycle.conditions.heading_consistent = heading.state().consistent;
                test.expect(lifecycle.sample(time, check.accepted) == Status::success &&
                                lifecycle.monitor.state().action == Action::none,
                            profile, "healthy checks do not bypass sample count/time qualification");
            }
            Vector raw = field;
            raw(1U) = T{4};
            if (outcome == Outcome::field_rejected)
            {
                raw = field * T{4};
            }
            if (outcome == Outcome::gate_rejected)
            {
                raw = independent_health;
            }
            auto V = Matrix::identity() * T{3072};
            if (outcome == Outcome::correction_failed)
            {
                // Deliberately outside the caller SPD premise: all preparation
                // checks and diagonal gate pass, then the actual LLT rejects S.
                V(0U, 1U) = V(1U, 0U) = T{8192};
            }
            auto const before = state;
            auto const P_before = P;
            auto const check =
                formal_eskf::runtime::check_magnetic_field(state.q_nb, raw, field, health, field_parameters, false);
            test.expect(check.status == Status::success && check.diagnostics_valid &&
                            check.accepted == (outcome != Outcome::field_rejected) &&
                            check.inclination_checked == health_available &&
                            check.reference_strength == (health_available ? T{32} : T{45}),
                        profile, "optional independent health reference never replaces observation model");
            test.expect(heading.update(state.q_nb, raw, field, heading_parameters, true, true) == Status::success,
                        profile, "heading monitor evaluates the actual sample independently of field/gate verdict");
            lifecycle.conditions.heading_consistent = heading.state().consistent;
            test.expect(lifecycle.sample(121U, check.accepted) == Status::success, profile,
                        "lifecycle consumes field decision");
            if (outcome == Outcome::field_rejected)
            {
                test.expect(lifecycle.monitor.state().action == Action::none &&
                                lifecycle.monitor.record_fusion(121U, FusionDecision::fused) == Status::out_of_range &&
                                same_estimator_state(state, before) && same_matrix(P, P_before),
                            profile, "rejected field grants no numerical attempt or success history");
                continue;
            }
            test.expect(lifecycle.monitor.state().action == Action::fuse, profile,
                        "qualified sample grants one attempt");
            auto const result = formal_eskf::runtime::fuse_magnetometer(
                state, P, raw, field, health, V, field_parameters, false, static_cast<T>(1e-6), T{0.25}, state, P);
            auto const expected = outcome == Outcome::fused           ? FusionDecision::fused
                                  : outcome == Outcome::gate_rejected ? FusionDecision::rejected
                                                                      : FusionDecision::failed;
            test.expect(result.fusion.decision == expected &&
                            lifecycle.monitor.record_fusion(121U, result.fusion.decision) == Status::success &&
                            lifecycle.monitor.state().last_fuse_time_us == (outcome == Outcome::fused ? 121U : 0U),
                        profile, "only actual fusion outcome starts progress; field acceptance is not fusion success");
            if (outcome == Outcome::correction_failed)
            {
                test.expect(result.fusion.status == Status::not_positive_definite && result.fusion.diagnostics_valid &&
                                result.fusion.test_ratio(1U) == T{0.0625},
                            profile,
                            "passing diagonal gate remains valid evidence when correlated solve subsequently fails");
            }
            if (outcome == Outcome::fused)
            {
                Reference const input(before, P_before, raw, field, V);
                auto const independent = formal_eskf::test::reference::reference_correction(
                    input, ESKF_RESET_APPROX != 0, {}, ESKF_MAG_TILT != 0);
                // Finite fixture regression, not a universal nonlinear rounding bound.
                T const tolerance = static_cast<T>(std::is_same_v<T, float> ? 3e-6 : 2e-13);
                test.expect(formal_eskf::test::reference::matches_reference(input, independent, state, P, tolerance),
                            profile,
                            "raw observation and adopted model drive the independently evaluated complete update");
            }
            else
            {
                test.expect(same_estimator_state(state, before) && same_matrix(P, P_before) &&
                                !lifecycle.monitor.state().active,
                            profile, "gate rejection or numerical failure preserves estimates without fake progress");
            }
        }
    }
    // Preserve real active/outage/recovery traces for both numerical configurations.
    test_real_fusion<Linalg, Configuration>(test, profile);
}

template <typename Reference>
void check_yaw_reset(TestContext & test, std::string_view profile, typename Reference::state_type const & before,
                     typename Reference::covariance_type const & P_before, typename Reference::state_type const & state,
                     typename Reference::covariance_type const & P, long double angle, long double yaw_variance)
{
    using T = typename Reference::value_type;
    constexpr std::size_t offset = Reference::attitude_offset;
    // Finite-fixture regression allowance; no general nonlinear error envelope.
    long double const tolerance = std::is_same_v<T, float> ? 8e-6L : 4e-13L;
    test.expect(std::abs(static_cast<long double>(state.q_nb.q0()) - std::cos(angle / 2)) <= tolerance &&
                    std::abs(static_cast<long double>(state.q_nb.q3()) - std::sin(angle / 2)) <= tolerance &&
                    state.q_nb.q1() == T{0} && state.q_nb.q2() == T{0},
                profile, "reset yaw matches independent atan2 geometry and preserves level tilt");
    auto G = formal_eskf::test::reference::Dense<Reference::size, Reference::size>{};
    for (std::size_t axis = 0U; axis < Reference::size; ++axis)
    {
        G[axis][axis] = 1;
    }
    G[offset][offset] = G[offset + 1U][offset + 1U] = std::cos(angle);
    G[offset][offset + 1U] = std::sin(angle);
    G[offset + 1U][offset] = -std::sin(angle);
    G[offset + 2U][offset + 2U] = 0;
    auto expected = formal_eskf::test::reference::product(
        formal_eskf::test::reference::product(G, formal_eskf::test::reference::copied(P_before)),
        formal_eskf::test::reference::transposed(G));
    expected[offset + 2U][offset + 2U] += yaw_variance;
    for (std::size_t row = 0U; row < Reference::size; ++row)
    {
        for (std::size_t column = 0U; column < Reference::size; ++column)
        {
            test.expect(std::abs(static_cast<long double>(P(row, column)) - expected[row][column]) <= tolerance,
                        profile, "reset covariance matches independent navigation-yaw replacement and local transport");
        }
    }
    if constexpr (Reference::is_ins)
    {
        test.expect(same_matrix(state.p_n, before.p_n) && same_matrix(state.v_n, before.v_n) &&
                        same_matrix(state.b_a, before.b_a) && same_matrix(state.b_g, before.b_g),
                    profile, "heading reset preserves unrelated nominal INS coordinates");
    }
}

template <typename Linalg, typename Configuration>
void test_source_filter_composition(TestContext & test, std::string_view profile)
{
    using T = typename Linalg::value_type;
    using Reference = MagneticCompositionReference<Linalg, Configuration>;
    using Vector = typename Reference::vector_type;
    using Matrix = typename Reference::matrix3_type;
    using Heading = formal_eskf::runtime::MagneticHeading<Linalg>;
    enum class Outcome
    {
        success,
        failed_reseed,
        failed_reset,
        unknown_flight
    };
    for (bool calibration_change : {false, true})
    {
        for (auto outcome : {Outcome::success, Outcome::failed_reseed, Outcome::failed_reset, Outcome::unknown_flight})
        {
            typename Reference::state_type state;
            auto P = Reference::covariance_type::identity();
            auto const field = Vector::from_row_major({T{32}, T{0}, T{0}});
            auto const changed_raw = Vector::from_row_major({T{32}, T{32}, T{0}});
            auto const V = Matrix::identity() * T{3072};
            formal_eskf::runtime::MagneticFieldParameters<T> const field_parameters{T{16}, T{0.5}, T{1}, T{1}};
            typename Heading::Parameters const heading_parameters{T{1}, T{0}, T{1}};
            Fixture lifecycle;
            lifecycle.qualify(test, profile);
            auto const first = formal_eskf::runtime::fuse_magnetometer(
                state, P, field, field, &field, V, field_parameters, false, static_cast<T>(1e-6), T{3}, state, P);
            test.expect(first.fusion.decision == FusionDecision::fused &&
                            lifecycle.monitor.record_fusion(121U, first.fusion.decision) == Status::success,
                        profile, "actual zero-residual baseline establishes the old-source session");
            formal_eskf::runtime::MagnetometerFilter<Linalg> filter;
            auto const old_field = Vector::from_row_major({T{0}, T{32}, T{0}});
            test.expect(filter.update(old_field) == Status::success, profile,
                        "establish distinct old-source LPF history");
            Heading heading;
            lifecycle.conditions.horizontal_aiding = false;
            lifecycle.conditions.heading_aligned = false;
            lifecycle.conditions.flight_state = outcome == Outcome::unknown_flight
                                                    ? MagnetometerFusion::FlightState::unknown
                                                    : MagnetometerFusion::FlightState::on_ground;
            if (calibration_change)
            {
                ++lifecycle.source.calibration_count;
            }
            else
            {
                ++lifecycle.source.device_id;
            }
            Vector expected_filter = old_field;
            constexpr T interval = static_cast<T>(0.01), time_constant = static_cast<T>(0.09);
            constexpr T alpha = interval / (time_constant + interval);
            std::uint64_t reset_time = 133U;
            for (std::uint64_t time = 130U; time <= 133U; ++time)
            {
                auto raw = time == 133U ? changed_raw : field;
                if (outcome == Outcome::failed_reseed && time == 130U)
                {
                    raw(1U) = std::numeric_limits<T>::quiet_NaN();
                }
                auto const check =
                    formal_eskf::runtime::check_magnetic_field(state.q_nb, raw, field, &field, field_parameters, false);
                bool const accepted = check.status == Status::success && check.accepted;
                if (accepted)
                {
                    test.expect(heading.update(state.q_nb, raw, field, heading_parameters, false, false) ==
                                    Status::success,
                                profile, "unaligned source reports unavailable heading consistency");
                }
                lifecycle.conditions.heading_consistent = heading.state().consistent;
                test.expect(lifecycle.sample(time, accepted) == Status::success &&
                                lifecycle.monitor.state().filter_reset == (time == 130U),
                            profile, "source/calibration revision instructs one physical-filter reseed");
                auto const filter_status = filter.update(raw, lifecycle.monitor.state().filter_reset);
                if (outcome == Outcome::failed_reseed)
                {
                    test.expect(filter_status == Status::non_finite_input && same_matrix(filter.field_b(), old_field) &&
                                    lifecycle.monitor.state().action == Action::none,
                                profile,
                                "failed reseed preserves old history but cannot authorize its use for the new sensor");
                    lifecycle.monitor.restart();
                    filter.reset();
                    test.expect(lifecycle.sample(131U) == Status::success && lifecycle.monitor.state().filter_reset &&
                                    lifecycle.monitor.state().action == Action::none &&
                                    filter.update(field, true) == Status::success &&
                                    same_matrix(filter.field_b(), field),
                                profile,
                                "host abort/restart starts a fresh qualification instead of mixing old sensor history");
                    break;
                }
                expected_filter = time == 130U ? raw : expected_filter + (raw - expected_filter) * alpha;
                test.expect(
                    filter_status == Status::success && same_matrix(filter.field_b(), expected_filter), profile,
                    "one recurrence per consumed source sample agrees with independent scalar-format recurrence");
                if (time < 133U)
                {
                    test.expect(lifecycle.monitor.state().action == Action::none, profile, "four new samples required");
                }
            }
            if (outcome == Outcome::failed_reseed)
            {
                continue;
            }
            if (outcome == Outcome::unknown_flight)
            {
                test.expect(lifecycle.monitor.state().action == Action::none && !heading.state().consistent, profile,
                            "unknown flight state does not authorize alignment even with qualified source data");
                lifecycle.conditions.flight_state = MagnetometerFusion::FlightState::on_ground;
                reset_time = 134U;
                auto const release_check = formal_eskf::runtime::check_magnetic_field(state.q_nb, changed_raw, field,
                                                                                      &field, field_parameters, false);
                test.expect(release_check.status == Status::success && release_check.accepted &&
                                lifecycle.sample(reset_time, release_check.accepted) == Status::success &&
                                !lifecycle.monitor.state().filter_reset &&
                                filter.update(changed_raw) == Status::success,
                            profile, "known ground state releases a freshly checked qualified reset");
                expected_filter = expected_filter + (changed_raw - expected_filter) * alpha;
            }
            test.expect(lifecycle.monitor.state().action == Action::reset && !heading.state().consistent &&
                            lifecycle.monitor.state().last_reset_time_us == 0U,
                        profile, "reset instruction alone cannot create alignment or successful-reset time");
            auto const before = state;
            auto const P_before = P;
            formal_eskf::so3::UnitQuaternion<Linalg> delta;
            delta = -delta;
            auto const delta_before = delta.coefficients();
            T const variance = outcome == Outcome::failed_reset ? T{-1} : T{0.25};
            auto const reset_status = formal_eskf::runtime::reset_magnetic_heading(
                state, P, filter.field_b(), field, variance, T{1}, static_cast<T>(1e-6), state, P, delta);
            test.expect(reset_status == (outcome == Outcome::failed_reset ? Status::domain_error : Status::success) &&
                            lifecycle.monitor.record_reset(reset_time, formal_eskf::succeeded(reset_status)) ==
                                Status::success,
                        profile, "controller acknowledges only the actual checked reset result");
            if (outcome == Outcome::failed_reset)
            {
                test.expect(
                    same_estimator_state(state, before) && same_matrix(P, P_before) &&
                        same_matrix(delta.coefficients(), delta_before) && !heading.state().consistent &&
                        lifecycle.monitor.state().recovery_reason == RecoveryReason::reset_failed &&
                        lifecycle.monitor.state().last_reset_time_us == 0U,
                    profile,
                    "failed reset changes controller recovery but preserves estimator outputs and heading latch");
                continue;
            }
            long double const angle = -std::atan2(static_cast<long double>(expected_filter(1U)),
                                                  static_cast<long double>(expected_filter(0U)));
            check_yaw_reset<Reference>(test, profile, before, P_before, state, P, angle, 0.25L);
            test.expect(std::abs(angle + std::atan2(32.0L, 32.0L)) > 0.25L, profile,
                        "filtered reset oracle differs materially from the raw-sample negative control");
            heading.reset_heading();
            test.expect(heading.state().consistent && !heading.state().innovation_valid &&
                            lifecycle.monitor.state().last_reset_time_us == reset_time &&
                            lifecycle.monitor.state().last_fuse_time_us == 0U,
                        profile,
                        "actual alignment precedes heading notification and is distinct from ordinary fusion progress");
            lifecycle.conditions.heading_aligned = true;
            lifecycle.conditions.heading_consistent = heading.state().consistent;
            auto const raw = changed_raw;
            auto const next_check =
                formal_eskf::runtime::check_magnetic_field(state.q_nb, raw, field, &field, field_parameters, false);
            test.expect(heading.update(state.q_nb, raw, field, heading_parameters, true, false) == Status::success,
                        profile, "post-reset heading monitor consumes the new raw sample");
            lifecycle.conditions.heading_consistent = heading.state().consistent;
            test.expect(next_check.status == Status::success && next_check.accepted &&
                            lifecycle.sample(reset_time + 1U, next_check.accepted) == Status::success &&
                            lifecycle.monitor.state().action == Action::fuse && filter.update(raw) == Status::success,
                        profile, "fresh field/heading checks permit ordinary correction with retained LPF history");
            Reference const raw_input(state, P, raw, field, V);
            Reference const wrong_filtered_input(state, P, filter.field_b(), field, V);
            auto const expected = formal_eskf::test::reference::reference_correction(raw_input, ESKF_RESET_APPROX != 0,
                                                                                     {}, ESKF_MAG_TILT != 0);
            auto const wrong = formal_eskf::test::reference::reference_correction(
                wrong_filtered_input, ESKF_RESET_APPROX != 0, {}, ESKF_MAG_TILT != 0);
            auto const result = formal_eskf::runtime::fuse_magnetometer(
                state, P, raw, field, &field, V, field_parameters, false, static_cast<T>(1e-6), T{100}, state, P);
            T const tolerance = static_cast<T>(std::is_same_v<T, float> ? 8e-6 : 4e-13);
            test.expect(
                result.fusion.decision == FusionDecision::fused &&
                    formal_eskf::test::reference::matches_reference(raw_input, expected, state, P, tolerance) &&
                    std::abs(expected.delta_x[Reference::attitude_offset + 2U][0] -
                             wrong.delta_x[Reference::attitude_offset + 2U][0]) > 0.001L &&
                    lifecycle.monitor.record_fusion(reset_time + 1U, result.fusion.decision) == Status::success,
                profile,
                "ordinary fusion uses raw data; independent filtered-input negative control gives a different update");
        }
    }
}

template <typename Linalg, typename Configuration>
void test_reference_composition(TestContext & test, std::string_view profile)
{
    using T = typename Linalg::value_type;
    using Reference = MagneticCompositionReference<Linalg, Configuration>;
    using Vector = typename Reference::vector_type;
    using Matrix = typename Reference::matrix3_type;
    using Heading = formal_eskf::runtime::MagneticHeading<Linalg>;
    for (bool aided : {false, true})
    {
        for (bool at_rest : {false, true})
        {
            for (bool health_available : {false, true})
            {
                typename Reference::state_type state;
                auto P = Reference::covariance_type::identity();
                formal_eskf::runtime::MagneticReference<Linalg> model;
                test.expect(model.update(1U, {T{0}, T{0}, T{32}}, true).changed, profile, "adopt initial model");
                auto const original = model.state().field_n;
                auto const health_field = Vector::from_row_major({T{0}, T{32}, T{0}});
                auto const * health = health_available ? &health_field : nullptr;
                auto const V = Matrix::identity() * T{3072};
                formal_eskf::runtime::MagneticFieldParameters<T> const field_parameters{T{8}, T{0.5}, T{1}, T{1}};
                typename Heading::Parameters const heading_parameters{T{0.5}, T{0}, T{1}};
                Heading heading;
                test.expect(heading.update_acceleration(Vector::from_row_major({T{8}, T{0}, T{0}}), T{0.25}) ==
                                    Status::success &&
                                heading.update(state.q_nb, original, original, heading_parameters, true, true) ==
                                    Status::success &&
                                heading.state().consistent,
                            profile, "actual independent excitation establishes consistent heading");
                Fixture lifecycle;
                lifecycle.source.reference_id = model.state().reference_id;
                lifecycle.qualify(test, profile);
                auto result =
                    formal_eskf::runtime::fuse_magnetometer(state, P, original, original, health, V, field_parameters,
                                                            false, static_cast<T>(1e-6), T{3}, state, P);
                test.expect(result.fusion.decision == FusionDecision::fused &&
                                lifecycle.monitor.record_fusion(121U, result.fusion.decision) == Status::success,
                            profile, "actual baseline correction starts the reference-aware session");
                formal_eskf::runtime::MagnetometerFilter<Linalg> filter;
                test.expect(filter.update(original) == Status::success, profile, "seed physical-source filter");
                auto const id = model.state().reference_id;
                auto const small = model.update(130U, {T{0}, T{0}, T{32.5}}, true, true);
                test.expect(small.status == Status::success && small.checked && !small.changed &&
                                model.state().reference_id == id && same_matrix(model.state().field_n, original),
                            profile,
                            "checked subthreshold model is not adopted and cannot revise the observation field");
                auto const raw = Vector::from_row_major({T{36}, T{0}, T{0}});
                auto field_check = formal_eskf::runtime::check_magnetic_field(state.q_nb, raw, model.state().field_n,
                                                                              health, field_parameters, false);
                lifecycle.source.reference_id = model.state().reference_id;
                test.expect(field_check.accepted && lifecycle.sample(130U, field_check.accepted) == Status::success &&
                                !lifecycle.monitor.state().filter_reset && filter.update(raw) == Status::success,
                            profile, "unchanged reference preserves source count and LPF");
                result = formal_eskf::runtime::fuse_magnetometer(state, P, raw, model.state().field_n, health, V,
                                                                 field_parameters, false, static_cast<T>(1e-6), T{3},
                                                                 state, P);
                test.expect(result.fusion.decision == FusionDecision::fused &&
                                lifecycle.monitor.record_fusion(130U, result.fusion.decision) == Status::success,
                            profile, "ordinary actual feedback precedes reference revision");
                auto const old_filter = filter.field_b();
                auto const old_metadata = lifecycle.monitor.state();
                auto const adopted = model.update(140U, {T{0.125}, T{0}, T{32}}, true, true);
                test.expect(adopted.status == Status::success && adopted.changed &&
                                model.state().reference_id == id + 1U,
                            profile, "larger explicit declination update adopts a new observation revision");
                auto const field = model.state().field_n;
                lifecycle.conditions.horizontal_aiding = aided;
                lifecycle.conditions.at_rest = at_rest;
                test.expect(heading.update(state.q_nb, raw, field, heading_parameters, true, aided) == Status::success,
                            profile, "heading monitor uses the newly adopted observation vector");
                lifecycle.conditions.heading_consistent = heading.state().consistent;
                lifecycle.source.reference_id = model.state().reference_id;
                field_check =
                    formal_eskf::runtime::check_magnetic_field(state.q_nb, raw, field, health, field_parameters, false);
                bool const reset = !aided || at_rest;
                test.expect(field_check.status == Status::success && field_check.accepted &&
                                field_check.inclination_checked == health_available &&
                                lifecycle.sample(140U, field_check.accepted) == Status::success &&
                                lifecycle.monitor.state().action == (reset ? Action::reset : Action::fuse) &&
                                !lifecycle.monitor.state().filter_reset &&
                                lifecycle.monitor.state().last_fuse_time_us == old_metadata.last_fuse_time_us &&
                                lifecycle.monitor.state().last_field_failure_time_us ==
                                    old_metadata.last_field_failure_time_us &&
                                same_matrix(filter.field_b(), old_filter),
                            profile,
                            "reference adoption keeps sensor history and optional-health availability while choosing "
                            "the required action");
                constexpr T alpha = static_cast<T>(0.01) / (static_cast<T>(0.09) + static_cast<T>(0.01));
                auto const expected_filter = old_filter + (raw - old_filter) * alpha;
                test.expect(filter.update(raw) == Status::success && same_matrix(filter.field_b(), expected_filter),
                            profile,
                            "reference-only update continues the existing LPF recurrence instead of reseeding");
                auto const before = state;
                auto const P_before = P;
                if (reset)
                {
                    formal_eskf::so3::UnitQuaternion<Linalg> delta;
                    auto const status = formal_eskf::runtime::reset_magnetic_heading(
                        state, P, filter.field_b(), field, T{0.25}, T{1}, static_cast<T>(1e-6), state, P, delta);
                    test.expect(status == Status::success &&
                                    lifecycle.monitor.record_reset(140U, true) == Status::success,
                                profile, "actual filtered heading reset precedes reference recovery acknowledgment");
                    long double const angle =
                        std::atan2(static_cast<long double>(field(1U)), static_cast<long double>(field(0U)));
                    check_yaw_reset<Reference>(test, profile, before, P_before, state, P, angle, 0.25L);
                    heading.reset_heading();
                    test.expect(
                        lifecycle.monitor.state().start_time_us == old_metadata.start_time_us &&
                            lifecycle.monitor.state().last_fuse_time_us == 130U &&
                            lifecycle.monitor.state().last_reset_time_us == 140U && heading.state().consistent,
                        profile,
                        "reference reset retains session and fusion clocks without relabeling it as correction");
                }
                else
                {
                    Reference const input(before, P_before, raw, field, V);
                    auto const independent = formal_eskf::test::reference::reference_correction(
                        input, ESKF_RESET_APPROX != 0, {}, ESKF_MAG_TILT != 0);
                    result = formal_eskf::runtime::fuse_magnetometer(state, P, raw, field, health, V, field_parameters,
                                                                     false, static_cast<T>(1e-6), T{3}, state, P);
                    T const tolerance = static_cast<T>(std::is_same_v<T, float> ? 8e-6 : 4e-13);
                    test.expect(
                        result.fusion.decision == FusionDecision::fused &&
                            formal_eskf::test::reference::matches_reference(input, independent, state, P, tolerance) &&
                            lifecycle.monitor.record_fusion(140U, result.fusion.decision) == Status::success &&
                            lifecycle.monitor.state().last_reset_time_us == 0U,
                        profile,
                        "aided motion fuses the new model through the independent correction equation without a reset");
                }
            }
        }
    }
}

} /* end namespace */

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using Float = formal_eskf::test::Backend<float>;
    using Double = formal_eskf::test::Backend<double>;
    using Ahrs = formal_eskf::configuration::Ahrs;
    using Ins = formal_eskf::configuration::Ins;
    formal_eskf::test::Case const cases[]{
        {"TC-R-MAGNETOMETER-LIFECYCLE-01", "protocol", test_start_qualification},
        {"TC-R-MAGNETOMETER-LIFECYCLE-02", "protocol", test_feedback},
        {"TC-R-MAGNETOMETER-LIFECYCLE-03", "protocol", test_active_and_outages},
        {"TC-R-MAGNETOMETER-LIFECYCLE-04", "protocol", test_timeouts},
        {"TC-R-MAGNETOMETER-LIFECYCLE-05", "protocol", test_reset_qualification},
        {"TC-R-MAGNETOMETER-LIFECYCLE-06", "protocol", test_reset_feedback},
        {"TC-R-MAGNETOMETER-LIFECYCLE-07", "protocol", test_heading_alignment},
        {"TC-R-MAGNETOMETER-LIFECYCLE-08", "protocol", test_source_reinitialization},
        {"TC-R-MAGNETOMETER-LIFECYCLE-09", "protocol", test_reference_change},
        {"TC-R-MAGNETOMETER-LIFECYCLE-10", "protocol", test_numerical_repair},
        {"TC-R-MAGNETOMETER-LIFECYCLE-11", "protocol", test_invalid_calls},
        {"TC-R-MAGNETOMETER-LIFECYCLE-12", "protocol", test_saturation},
        {"TC-R-MAGNETOMETER-LIFECYCLE-13", "protocol", test_snapshot_recovery},
        {"TC-SW-MAGNETIC-01", "AHRS.binary32", test_stage_composition<Float, Ahrs>},
        {"TC-SW-MAGNETIC-01", "AHRS.binary64", test_stage_composition<Double, Ahrs>},
        {"TC-SW-MAGNETIC-01", "INS.binary32", test_stage_composition<Float, Ins>},
        {"TC-SW-MAGNETIC-01", "INS.binary64", test_stage_composition<Double, Ins>},
        {"TC-SW-MAGNETIC-02", "AHRS.binary32", test_source_filter_composition<Float, Ahrs>},
        {"TC-SW-MAGNETIC-02", "AHRS.binary64", test_source_filter_composition<Double, Ahrs>},
        {"TC-SW-MAGNETIC-02", "INS.binary32", test_source_filter_composition<Float, Ins>},
        {"TC-SW-MAGNETIC-02", "INS.binary64", test_source_filter_composition<Double, Ins>},
        {"TC-SW-MAGNETIC-03", "AHRS.binary32", test_reference_composition<Float, Ahrs>},
        {"TC-SW-MAGNETIC-03", "AHRS.binary64", test_reference_composition<Double, Ahrs>},
        {"TC-SW-MAGNETIC-03", "INS.binary32", test_reference_composition<Float, Ins>},
        {"TC-SW-MAGNETIC-03", "INS.binary64", test_reference_composition<Double, Ins>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
