/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/magnetometer_fusion.hpp>
#include <formal_eskf/runtime/magnetometer.hpp>

#include <cstdint>
#include <iostream>
#include <limits>
#include <string_view>

#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{

using formal_eskf::Status;
using formal_eskf::runtime::FusionDecision;
using formal_eskf::runtime::MagnetometerFusion;
using StopReason = MagnetometerFusion::StopReason;
using RecoveryReason = MagnetometerFusion::RecoveryReason;
using TestContext = formal_eskf::test::Context;
constexpr std::string_view profile = "magnetometer lifecycle";

struct Fixture
{
    MagnetometerFusion monitor;
    MagnetometerFusion::Parameters parameters{20U, 10U, 50U};
    MagnetometerFusion::Conditions conditions{true, true, true, true, false};
    MagnetometerFusion::Source source{1U, 0U};

    Status sample(std::uint64_t time, bool accepted = true)
    {
        return monitor.update(time, conditions, parameters, {time, source, accepted});
    }

    Status tick(std::uint64_t time) { return monitor.update(time, conditions, parameters, {}); }

    void qualify(TestContext & test, std::uint64_t first = 100U)
    {
        test.expect(sample(first) == Status::success && sample(first + 8U) == Status::success &&
                        sample(first + 16U) == Status::success && sample(first + 21U) == Status::success &&
                        monitor.state().fusion_allowed && !monitor.state().active &&
                        monitor.state().last_fuse_time_us == 0U,
                    profile, "four samples and strictly elapsed health permit an attempt, not a fictitious fusion");
    }

    void start(TestContext & test, std::uint64_t first = 100U)
    {
        qualify(test, first);
        test.expect(monitor.record_fusion(first + 21U, FusionDecision::fused) == Status::success &&
                        monitor.state().active && monitor.state().start_time_us == first + 21U,
                    profile, "actual success activates the session");
    }
};

bool same_state(MagnetometerFusion::State const & a, MagnetometerFusion::State const & b)
{
    return a.active == b.active && a.fusion_allowed == b.fusion_allowed && a.reset_allowed == b.reset_allowed &&
           a.filter_reset == b.filter_reset && a.last_reset_time_us == b.last_reset_time_us &&
           a.stop_reason == b.stop_reason && a.recovery_reason == b.recovery_reason &&
           a.last_field_failure_time_us == b.last_field_failure_time_us && a.start_time_us == b.start_time_us &&
           a.last_fuse_time_us == b.last_fuse_time_us;
}

void test_startup(TestContext & test)
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
        test.expect(!state.active && !state.fusion_allowed && state.stop_reason == expected &&
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
                    !fixture.monitor.state().fusion_allowed && fixture.sample(121U) == Status::success &&
                    fixture.monitor.state().fusion_allowed && !fixture.monitor.state().active,
                profile, "health equality does not qualify; one microsecond later permits an attempt");
    test.expect(fixture.monitor.record_fusion(121U, FusionDecision::rejected) == Status::success &&
                    !fixture.monitor.state().active && fixture.monitor.state().start_time_us == 0U &&
                    fixture.monitor.state().last_fuse_time_us == 0U && fixture.sample(180U) == Status::success &&
                    fixture.monitor.state().fusion_allowed &&
                    fixture.monitor.record_fusion(180U, FusionDecision::rejected) == Status::success &&
                    fixture.monitor.state().recovery_reason == RecoveryReason::none,
                profile, "rejected startup retries without an invented active-session timeout");

    Fixture count;
    test.expect(count.sample(100U) == Status::success && count.sample(130U) == Status::success &&
                    count.sample(140U) == Status::success && !count.monitor.state().fusion_allowed &&
                    count.sample(150U) == Status::success && count.monitor.state().fusion_allowed,
                profile, "elapsed time alone cannot bypass the four-sample requirement");
    Fixture failure;
    test.expect(failure.sample(100U, false) == Status::success && failure.sample(108U) == Status::success &&
                    failure.sample(116U) == Status::success && failure.sample(121U) == Status::success &&
                    failure.monitor.state().fusion_allowed &&
                    failure.monitor.state().last_field_failure_time_us == 100U,
                profile, "health starts at the failed check, not the subsequent first good sample");
    test.expect(failure.sample(122U, false) == Status::success &&
                    failure.monitor.state().last_field_failure_time_us == 122U &&
                    failure.sample(142U) == Status::success && !failure.monitor.state().fusion_allowed &&
                    failure.sample(143U) == Status::success && failure.monitor.state().fusion_allowed,
                profile, "each failed startup field check restarts the strict health timer");

    for (std::uint64_t age : {9U, 10U, 11U})
    {
        Fixture delayed;
        delayed.qualify(test);
        test.expect(delayed.monitor.update(150U, delayed.conditions, delayed.parameters,
                                           {150U - age, delayed.source, true}) == Status::success &&
                        delayed.monitor.state().fusion_allowed == (age < 10U),
                    profile, "startup age must be strictly less than one maximum interval");
    }
    Fixture horizon;
    test.expect(horizon.monitor.update(100U, horizon.conditions, horizon.parameters, {91U, horizon.source, false}) ==
                        Status::success &&
                    horizon.sample(108U) == Status::success && horizon.sample(116U) == Status::success &&
                    horizon.sample(120U) == Status::success && !horizon.monitor.state().fusion_allowed &&
                    horizon.sample(121U) == Status::success && horizon.monitor.state().fusion_allowed &&
                    horizon.monitor.state().last_field_failure_time_us == 100U,
                profile, "health uses the check horizon, not the delayed sample timestamp");

    Fixture reset_frame;
    reset_frame.conditions.starting_allowed = false;
    for (auto time : {100U, 108U, 116U, 121U})
    {
        test.expect(reset_frame.sample(time) == Status::success && !reset_frame.monitor.state().fusion_allowed, profile,
                    "no startup on a yaw-alignment/reset frame");
    }
    reset_frame.conditions.starting_allowed = true;
    test.expect(reset_frame.sample(122U) == Status::success && reset_frame.monitor.state().fusion_allowed, profile,
                "stable following frame may start without discarding field history");
}

void test_active_and_outages(TestContext & test)
{
    Fixture fixture;
    fixture.start(test);
    test.expect(fixture.sample(130U, false) == Status::success && fixture.monitor.state().active &&
                    !fixture.monitor.state().fusion_allowed &&
                    fixture.monitor.state().stop_reason == StopReason::field_rejected &&
                    fixture.monitor.state().last_fuse_time_us == 121U && fixture.sample(131U) == Status::success &&
                    fixture.monitor.state().fusion_allowed &&
                    fixture.monitor.record_fusion(131U, FusionDecision::fused) == Status::success,
                profile, "active disturbance inhibits correction but the next good field need not requalify startup");
    test.expect(fixture.tick(150U) == Status::success && fixture.monitor.state().active &&
                    !fixture.monitor.state().fusion_allowed && fixture.tick(151U) == Status::success &&
                    !fixture.monitor.state().active &&
                    fixture.monitor.state().stop_reason == StopReason::data_timeout &&
                    fixture.monitor.state().last_field_failure_time_us == 130U &&
                    fixture.monitor.state().last_fuse_time_us == 131U,
                profile, "no-data stop occurs at two intervals and preserves field/progress history");
    test.expect(fixture.tick(300U) == Status::success &&
                    fixture.monitor.state().recovery_reason == RecoveryReason::none &&
                    fixture.sample(301U) == Status::success && fixture.monitor.state().fusion_allowed &&
                    fixture.monitor.record_fusion(301U, FusionDecision::fused) == Status::success &&
                    fixture.monitor.state().active && fixture.monitor.state().start_time_us == 301U,
                profile, "long outage restarts on fresh qualified data without a permanent timeout latch");

    for (bool disable : {false, true})
    {
        Fixture paused;
        paused.start(test);
        paused.conditions.enabled = !disable;
        paused.conditions.ready = disable;
        test.expect(paused.sample(130U) == Status::success && !paused.monitor.state().active &&
                        paused.monitor.state().stop_reason == (disable ? StopReason::disabled : StopReason::not_ready),
                    profile, "disable/continuing-condition failure stops the session");
        paused.conditions.enabled = true;
        paused.conditions.ready = true;
        test.expect(paused.sample(131U) == Status::success && paused.monitor.state().fusion_allowed &&
                        paused.monitor.record_fusion(131U, FusionDecision::fused) == Status::success,
                    profile, "ordinary stops preserve startup health");
    }
}

void test_timeouts(TestContext & test)
{
    for (unsigned mask = 0U; mask < 4U; ++mask)
    {
        Fixture fixture;
        fixture.start(test);
        fixture.conditions.horizontal_aiding = (mask & 1U) != 0U;
        fixture.conditions.at_rest = (mask & 2U) != 0U;
        for (auto time : {131U, 141U, 151U, 161U, 171U})
        {
            test.expect(fixture.sample(time) == Status::success && fixture.monitor.state().fusion_allowed &&
                            fixture.monitor.record_fusion(time, FusionDecision::rejected) == Status::success &&
                            fixture.monitor.state().active && fixture.monitor.state().last_fuse_time_us == 121U,
                        profile, "active gate rejections do not refresh progress; timeout equality is allowed");
        }
        test.expect(fixture.sample(172U) == Status::success && fixture.monitor.state().fusion_allowed &&
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
                        fixture.monitor.state().fusion_allowed == !requires_reset,
                    profile, "ordinary timeout can retry; pending reset cannot acknowledge itself");
        if (!requires_reset)
        {
            test.expect(fixture.monitor.record_fusion(173U, FusionDecision::fused) == Status::success &&
                            fixture.monitor.state().active && fixture.monitor.state().start_time_us == 173U,
                        profile, "successful retry reactivates the stopped session");
        }
    }
    Fixture recovered;
    recovered.start(test);
    test.expect(recovered.sample(172U) == Status::success && recovered.monitor.state().fusion_allowed &&
                    recovered.monitor.record_fusion(172U, FusionDecision::fused) == Status::success &&
                    recovered.monitor.state().active && recovered.monitor.state().last_fuse_time_us == 172U,
                profile, "success at an overdue horizon prevents timeout instead of being discarded beforehand");
    Fixture disturbed;
    disturbed.start(test);
    test.expect(disturbed.sample(172U, false) == Status::success && !disturbed.monitor.state().active &&
                    disturbed.monitor.state().stop_reason == StopReason::fusion_timeout,
                profile, "inhibited correction checks the active progress deadline");
    Fixture absent;
    absent.parameters.fusion_timeout_us = 5U;
    absent.start(test);
    test.expect(absent.tick(130U) == Status::success && absent.monitor.state().active &&
                    absent.monitor.state().recovery_reason == RecoveryReason::none &&
                    absent.tick(141U) == Status::success &&
                    absent.monitor.state().stop_reason == StopReason::data_timeout,
                profile, "no-data branch uses recency, not the fusion-attempt timer");
}

void test_feedback_and_recovery(TestContext & test)
{
    Fixture fixture;
    fixture.qualify(test);
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
    test.expect(fixture.sample(140U) == Status::success &&
                    fixture.monitor.record_fusion(140U, FusionDecision::failed) == Status::success &&
                    fixture.monitor.state().recovery_reason == RecoveryReason::fusion_failed &&
                    fixture.monitor.state().last_fuse_time_us == 121U && fixture.sample(200U) == Status::success &&
                    !fixture.monitor.state().fusion_allowed,
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
    fixture.qualify(test, 210U);

    for (bool calibration_change : {false, true})
    {
        Fixture changed;
        changed.start(test);
        if (calibration_change)
        {
            ++changed.source.calibration_count;
        }
        else
        {
            ++changed.source.device_id;
        }
        test.expect(changed.sample(130U) == Status::success &&
                        changed.monitor.state().recovery_reason == RecoveryReason::source_changed &&
                        !changed.monitor.state().fusion_allowed && changed.monitor.state().last_fuse_time_us == 0U &&
                        changed.monitor.record_fusion(121U, FusionDecision::fused) == Status::out_of_range,
                    profile,
                    "source/calibration change requires reference/state reinitialization, invalidates old feedback");
        changed.monitor.restart();
        test.expect(changed.sample(130U) == Status::out_of_range, profile, "restart retains source/time watermarks");
        changed.qualify(test, 140U);
    }
}

void test_source_reinitialization(TestContext & test)
{
    using FlightState = MagnetometerFusion::FlightState;
    for (bool calibration : {false, true})
    {
        for (auto flight : {FlightState::unknown, FlightState::on_ground, FlightState::in_air})
        {
            for (bool aided : {false, true})
            {
                Fixture fixture;
                fixture.start(test);
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
                                !fixture.monitor.state().fusion_allowed && !fixture.monitor.state().reset_allowed &&
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
                test.expect(state.reset_allowed == reset && state.fusion_allowed == fuse && !state.active, profile,
                            "four samples: ground/unaided realign, aided flight preserves yaw, unknown waits");
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
                    test.expect(fixture.sample(134U) == Status::success && state.reset_allowed, profile,
                                "known ground state resolves pending unknown flight context");
                }
            }
        }
    }
    Fixture fixture;
    fixture.start(test);
    fixture.conditions.flight_state = FlightState::on_ground;
    fixture.conditions.ready = false;
    ++fixture.source.device_id;
    test.expect(fixture.sample(130U) == Status::success && fixture.monitor.state().filter_reset, profile,
                "source reseed does not require attitude readiness");
    fixture.conditions.ready = true;
    fixture.conditions.flight_state = FlightState::in_air;
    test.expect(fixture.sample(131U) == Status::success && fixture.sample(132U) == Status::success &&
                    fixture.sample(133U) == Status::success && fixture.monitor.state().reset_allowed,
                profile, "ground change keeps its alignment requirement after takeoff/readiness");
    test.expect(fixture.monitor.record_reset(133U, false) == Status::success, profile, "record real reset failure");
    ++fixture.source.device_id;
    test.expect(fixture.sample(134U) == Status::success && fixture.sample(200U) == Status::success &&
                    fixture.monitor.state().recovery_reason == RecoveryReason::reset_failed &&
                    !fixture.monitor.state().reset_allowed && !fixture.monitor.state().fusion_allowed,
                profile, "another source cannot silently repair a numerical failure");
}

void test_reference_change(TestContext & test)
{
    for (bool aided : {false, true})
    {
        for (bool at_rest : {false, true})
        {
            Fixture fixture;
            fixture.start(test);
            fixture.conditions.horizontal_aiding = aided;
            fixture.conditions.at_rest = at_rest;
            ++fixture.source.reference_id;
            bool const reset = !aided || at_rest;
            test.expect(fixture.sample(130U) == Status::success && !fixture.monitor.state().filter_reset &&
                            fixture.monitor.state().reset_allowed == reset &&
                            fixture.monitor.state().fusion_allowed == !reset &&
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
    unhealthy.start(test);
    unhealthy.conditions.horizontal_aiding = false;
    ++unhealthy.source.reference_id;
    test.expect(unhealthy.sample(130U, false) == Status::success && !unhealthy.monitor.state().reset_allowed &&
                    unhealthy.sample(150U) == Status::success && !unhealthy.monitor.state().reset_allowed &&
                    unhealthy.sample(151U) == Status::success && unhealthy.monitor.state().reset_allowed,
                profile, "new reference still requires strictly elapsed field health, not just a revision change");
    auto pending = unhealthy;
    pending.conditions.horizontal_aiding = true;
    test.expect(pending.sample(152U) == Status::success && !pending.monitor.state().reset_allowed &&
                    pending.monitor.state().recovery_reason == RecoveryReason::none &&
                    pending.monitor.state().stop_reason == StopReason::none,
                profile, "restored aiding cancels pending reference reset without reporting a timeout");
    Fixture invalid;
    invalid.start(test);
    auto const before = invalid.monitor.state();
    invalid.conditions.flight_state = static_cast<MagnetometerFusion::FlightState>(99);
    test.expect(invalid.sample(130U) == Status::domain_error &&
                    invalid.monitor.state().last_fuse_time_us == before.last_fuse_time_us,
                profile, "invalid flight-state enum cannot authorize a source reset");
}

void test_reset_feedback(TestContext & test)
{
    Fixture fixture;
    fixture.conditions.horizontal_aiding = false;
    fixture.start(test);
    test.expect(fixture.sample(172U) == Status::success &&
                    fixture.monitor.record_fusion(172U, FusionDecision::rejected) == Status::success &&
                    fixture.monitor.state().reset_allowed,
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
                    !fixture.monitor.state().reset_allowed && fixture.monitor.state().active &&
                    fixture.sample(223U) == Status::success &&
                    fixture.monitor.record_fusion(223U, FusionDecision::rejected) == Status::success &&
                    fixture.monitor.state().reset_allowed &&
                    fixture.monitor.record_reset(223U, false) == Status::success &&
                    fixture.monitor.state().recovery_reason == RecoveryReason::reset_failed &&
                    fixture.monitor.state().last_reset_time_us == 172U && fixture.sample(224U) == Status::success &&
                    !fixture.monitor.state().reset_allowed,
                profile, "reset refreshes timeout; failed reset requires caller repair without retry loop");

    Fixture disturbed;
    disturbed.conditions.horizontal_aiding = false;
    disturbed.start(test);
    test.expect(disturbed.sample(172U, false) == Status::success && !disturbed.monitor.state().reset_allowed &&
                    disturbed.sample(192U) == Status::success && !disturbed.monitor.state().reset_allowed &&
                    disturbed.sample(193U) == Status::success && disturbed.monitor.state().reset_allowed,
                profile, "bad field must requalify health before recovery reset");
    test.expect(disturbed.tick(194U) == Status::success &&
                    disturbed.monitor.record_reset(193U, true) == Status::out_of_range,
                profile, "no-data tick revokes omitted reset feedback");
    disturbed.conditions.starting_allowed = false;
    test.expect(disturbed.sample(195U) == Status::success && !disturbed.monitor.state().reset_allowed, profile,
                "external reset/yaw-alignment frame cannot reset again");
    disturbed.conditions.starting_allowed = true;
    test.expect(disturbed.monitor.update(206U, disturbed.conditions, disturbed.parameters,
                                         {196U, disturbed.source, true}) == Status::success &&
                    !disturbed.monitor.state().reset_allowed,
                profile, "stale sample cannot authorize recovery reset");
    disturbed.conditions.ready = false;
    test.expect(disturbed.sample(207U) == Status::success && !disturbed.monitor.state().reset_allowed, profile,
                "lost readiness preserves pending recovery without a reset permission");
    disturbed.conditions.ready = true;
    disturbed.conditions.horizontal_aiding = true;
    test.expect(disturbed.sample(208U) == Status::success && !disturbed.monitor.state().reset_allowed &&
                    disturbed.monitor.state().recovery_reason == RecoveryReason::none &&
                    disturbed.monitor.state().stop_reason == StopReason::fusion_timeout,
                profile, "restored horizontal aid cancels timeout reset in favor of ordinary stop");
}

void test_invalid_calls(TestContext & test)
{
    for (unsigned invalid_case = 0U; invalid_case < 13U; ++invalid_case)
    {
        Fixture fixture;
        fixture.start(test);
        auto parameters = fixture.parameters;
        MagnetometerFusion::Sample sample{130U, fixture.source, true};
        std::uint64_t now = 130U;
        Status expected = Status::domain_error;
        switch (invalid_case)
        {
        case 0U:
            parameters.health_time_us = 0U;
            break;
        case 1U:
            parameters.maximum_interval_us = 0U;
            break;
        case 2U:
            parameters.fusion_timeout_us = 0U;
            break;
        case 3U:
            ++parameters.health_time_us;
            break;
        case 4U:
            ++parameters.maximum_interval_us;
            break;
        case 5U:
            ++parameters.fusion_timeout_us;
            break;
        case 6U:
            now = 121U;
            expected = Status::out_of_range;
            break;
        case 7U:
            sample.time_us = 121U;
            expected = Status::out_of_range;
            break;
        case 8U:
            sample.time_us = 131U;
            expected = Status::out_of_range;
            break;
        case 9U:
            sample.source.device_id = 0U;
            break;
        case 10U:
            now = 0U;
            expected = Status::out_of_range;
            break;
        case 11U:
            now = 120U;
            expected = Status::out_of_range;
            break;
        default:
            parameters.maximum_interval_us = std::numeric_limits<std::uint64_t>::max();
            break;
        }
        test.expect(fixture.monitor.update(now, fixture.conditions, parameters, sample) == expected &&
                        !fixture.monitor.state().active && !fixture.monitor.state().fusion_allowed &&
                        fixture.monitor.state().stop_reason == StopReason::invalid_call &&
                        fixture.monitor.state().last_fuse_time_us == 121U &&
                        fixture.monitor.state().last_field_failure_time_us == 100U,
                    profile,
                    "invalid call revokes permission but does not consume time/config/source or rewrite history");
        test.expect(fixture.sample(130U) == Status::success && fixture.monitor.state().fusion_allowed, profile,
                    "valid retry can consume the timestamp rejected by an invalid call");
    }
    constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
    Fixture large;
    large.start(test, maximum - 30U);
    test.expect(large.sample(maximum) == Status::success && large.monitor.state().fusion_allowed &&
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
                    !duration.monitor.state().fusion_allowed,
                profile, "four samples cannot overflow an unreachable health duration into qualification");
    duration.monitor.reset();
    duration.parameters = {20U, 10U, 50U};
    duration.qualify(test, 1U);
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
        auto const field =
            formal_eskf::runtime::check_magnetic_field(state.q_nb, measurement, reference, field_parameters, false);
        test.expect(fixture.sample(time, formal_eskf::succeeded(field.status) && field.diagnostics_valid &&
                                             field.accepted) == Status::success,
                    scalar_profile, "use actual same-prior field check");
        auto decision = FusionDecision::rejected;
        if (fixture.monitor.state().fusion_allowed)
        {
            auto const result =
                formal_eskf::runtime::fuse_magnetometer(state, P, measurement, reference, V, field_parameters, false,
                                                        static_cast<value_type>(1e-6), value_type{3}, state, P);
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
                    !fixture.monitor.state().fusion_allowed && same_matrix(state.q_nb.coefficients(), before_q) &&
                    same_matrix(P, before_P) && step(131U, reference) == FusionDecision::fused,
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

} /* end namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    TestContext test;
    test_startup(test);
    test_active_and_outages(test);
    test_timeouts(test);
    test_feedback_and_recovery(test);
    test_source_reinitialization(test);
    test_reference_change(test);
    test_reset_feedback(test);
    test_invalid_calls(test);
    test_real_fusion<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ahrs>(test, "float AHRS");
    test_real_fusion<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ins>(test, "float INS");
    test_real_fusion<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ahrs>(test, "double AHRS");
    test_real_fusion<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ins>(test, "double INS");
    if (test.failures() != 0)
    {
        return 1;
    }
    std::cout << "Magnetometer lifecycle: pass\n";
    return 0;
}
