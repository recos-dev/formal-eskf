/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "height_state_support.hpp"
#ifndef FORMAL_ESKF_PROOF_HEIGHT_SOURCE
#define FORMAL_ESKF_PROOF_HEIGHT_SOURCE 0
#endif
static_assert(FORMAL_ESKF_PROOF_HEIGHT_SOURCE < 2);
using height_proof::altitude_valid;
using height_proof::Bias;
using height_proof::Manager;
using height_proof::Reason;
using height_proof::Result;
using height_proof::same;
using height_proof::same_bias;
using height_proof::same_manager;
using height_proof::same_matrix;
using height_proof::same_result;
using height_proof::same_sample;
using height_proof::same_session;
using height_proof::Sample;
using height_proof::Scalar;
using height_proof::Session;
using height_proof::Source;
using height_proof::Status;
using height_proof::stopped;
namespace setting = formal_eskf::px4::profile::height;
bool empty_session(Session const & aid)
{
    auto const & bias = aid.bias;
    return aid.ratio.time_us == 0U && std::isnan(aid.ratio.filtered) && same(bias.value, Scalar{0}) &&
           same(bias.variance, Scalar{0}) && same(bias.maximum_variance, Scalar{2}) &&
           same(bias.signed_ratio, Scalar{0}) && same(bias.positive_time, Scalar{0}) &&
           same(bias.negative_time, Scalar{0}) && same(bias.dt, static_cast<Scalar>(0.01)) &&
           aid.fusion.status == Status::success &&
           aid.fusion.decision == formal_eskf::runtime::FusionDecision::failed && !aid.fusion.diagnostics_valid &&
           same(aid.fusion.innovation(0U), Scalar{0}) && same(aid.fusion.innovation_variance(0U), Scalar{0}) &&
           same(aid.fusion.test_ratio(0U), Scalar{0}) && same(aid.fusion_innovation, Scalar{0}) &&
           !aid.innovation_limited && aid.sample_time == 0U && aid.data_time == 0U && aid.progress_time == 0U &&
           aid.fused_time == 0U && aid.device_id == 0U && aid.calibration_count == 0U && aid.generation == 0U &&
           !aid.active && !aid.fusion_reset && !aid.fault && aid.stop_reason == Reason::none;
}
bool empty_history(Manager const & manager, std::uint64_t started, Scalar altitude)
{
    return empty_session(manager.m_sessions[0U]) && empty_session(manager.m_sessions[1U]) &&
           manager.m_reference == Source::none && same(manager.m_origin_altitude, altitude) &&
           same(manager.m_barometer_lpf, Scalar{0}) && manager.m_barometer_samples == 0U &&
           manager.m_started == started && manager.m_tick == 0U && manager.m_last_height_progress == 0U;
}
void verify_height_initialization(Manager manager, std::uint64_t now, Scalar altitude)
{
    Manager expected;
    expected = manager;
    bool const valid = now != 0U && altitude_valid(altitude);
    if (valid)
    {
        expected = Manager{};
        expected.m_origin_altitude = altitude;
        expected.m_started = now;
    }
    Manager fresh;
    __ESBMC_assert(empty_history(fresh, 0U, Scalar{0}) && height_proof::control_domain(fresh),
                   "R-HEIGHT-FUSION: constructor establishes every specified empty field");
    auto const status = manager.initialize(now, altitude);
    __ESBMC_assert(!valid || (height_proof::history_domain(manager, now) && height_proof::control_domain(manager)),
                   "R-HEIGHT-FUSION: initialize establishes the public owner induction domain");
    __ESBMC_assert(!valid || empty_history(manager, now, altitude),
                   "R-HEIGHT-FUSION: successful initialize establishes exact defaults");
    __ESBMC_assert(
        status == (valid ? Status::success : Status::domain_error) && same_manager(manager, expected),
        "R-HEIGHT-FUSION: initialization resets every session or preserves the entire owner on invalid time/altitude");
}
void verify_height_stop(Manager manager, Reason reason)
{
    constexpr Source source = static_cast<Source>(FORMAL_ESKF_PROOF_HEIGHT_SOURCE);
    Manager expected;
    expected = manager;
    bool const control = height_proof::control_domain(manager);
    stopped(expected, source, reason);
    manager.stop(source, reason);
    __ESBMC_assert(!control || height_proof::control_domain(manager),
                   "Height stop: owner origin, filter, intervals and active-reference invariant preserved");
    __ESBMC_assert(
        same_manager(manager, expected),
        "R-HEIGHT-FUSION: stopping clears only selected authority/diagnostics and its reference, retaining history");
}
void verify_height_admission(Manager manager, std::uint64_t now, Sample sample, Result result)
{
    constexpr Source source = static_cast<Source>(FORMAL_ESKF_PROOF_HEIGHT_SOURCE);
    // Time subtraction requires the owner's chronological transition invariant.
    // This private lemma does not claim that public update establishes it.
    for (auto const & aid : manager.m_sessions)
    {
        __ESBMC_assume(aid.data_time <= aid.sample_time && aid.sample_time <= now);
    }
    Manager expected;
    expected = manager;
    auto expected_result = result;
    bool const domain = height_proof::history_domain(manager, now);
    bool const control = height_proof::control_domain(manager);
    auto const before_sample = sample;
    bool const barometer = source == Source::barometer;
    auto & aid = expected.m_sessions[barometer ? 0U : 1U];
    auto const deadline = barometer ? setting::barometer_timeout_us : setting::gnss_timeout_us;
    auto const age = barometer ? setting::barometer_maximum_age_us : setting::gnss_maximum_age_us;
    bool accepted = false;
    if (sample.time_us == 0U || sample.time_us <= aid.sample_time || sample.time_us > now)
    {
        if (aid.active && now - aid.data_time >= deadline)
        {
            stopped(expected, source, Reason::no_data);
        }
    }
    else
    {
        aid.sample_time = sample.time_us;
        if (now - sample.time_us >= age)
        {
            ++expected_result.rejected;
            if (aid.active && now - aid.data_time >= deadline)
            {
                stopped(expected, source, Reason::no_data);
            }
        }
        else if (sample.device_id == 0U || !altitude_valid(sample.altitude) || !std::isfinite(sample.variance) ||
                 sample.variance <= Scalar{0})
        {
            stopped(expected, source, Reason::invalid_sample);
            ++expected_result.rejected;
        }
        else
        {
            if (aid.device_id != sample.device_id || aid.calibration_count != sample.calibration_count ||
                aid.generation != sample.generation)
            {
                stopped(expected, source, Reason::source_changed);
                aid.generation = sample.generation;
                aid.device_id = sample.device_id;
                aid.calibration_count = sample.calibration_count;
                aid.bias = {};
                aid.ratio = {};
                aid.fault = false;
                if (barometer)
                {
                    expected.m_barometer_samples = 0U;
                }
            }
            if (!sample.qualified)
            {
                stopped(expected, source, Reason::quality);
                ++expected_result.rejected;
            }
            else
            {
                aid.data_time = sample.time_us;
                accepted = true;
            }
        }
    }
    bool const actual = manager.admit(source, now, sample, result);
    __ESBMC_assert(
        actual == accepted && same_manager(manager, expected) && same_result(result, expected_result) &&
            same_sample(sample, before_sample),
        "R-HEIGHT-FUSION: exact source admission, strict ages, source/calibration/generation clearing, rejection "
        "counts and full frame");
    __ESBMC_assert(!domain || height_proof::history_domain(manager, now),
                   "R-HEIGHT-FUSION: admission preserves the complete owner induction domain");
    __ESBMC_assert(!control || height_proof::control_domain(manager),
                   "Height admission: source retirement/reset preserves owner control invariants");
    auto const & after = manager.session(source);
    __ESBMC_assert(
        after.data_time <= after.sample_time && after.sample_time <= now &&
            (!actual ||
             (after.data_time == sample.time_us && sample.qualified && sample.device_id != 0U &&
              altitude_valid(sample.altitude) && std::isfinite(sample.variance) && sample.variance > Scalar{0})),
        "R-HEIGHT-FUSION: admission preserves chronological watermarks and requires fresh qualified finite data");
}
void verify_height_queries(Manager manager, std::uint64_t now,
                           formal_eskf::runtime::GnssFusion::VerticalAccelerationTimes acceleration)
{
    constexpr Source source = static_cast<Source>(FORMAL_ESKF_PROOF_HEIGHT_SOURCE);
    Manager before;
    before = manager;
    auto const progress = manager.m_last_height_progress == 0U ? manager.m_started : manager.m_last_height_progress;
    bool const acceleration_reset =
        acceleration.update_time_us != 0U && acceleration.update_time_us <= now &&
        acceleration.last_good_time_us <= acceleration.update_time_us &&
        (acceleration.last_good_time_us == 0U || now - acceleration.last_good_time_us > 500000U);
    bool const reset =
        acceleration_reset || (progress != 0U && now > progress && now - progress > setting::fusion_timeout_us);
    bool const valid = manager.m_last_height_progress != 0U && now >= manager.m_last_height_progress &&
                       now - manager.m_last_height_progress <= setting::fusion_timeout_us;
    auto const actual = manager.vertical_innovation(source);
    auto const & aid = manager.m_sessions[source == Source::barometer ? 0U : 1U];
    auto const & selected_session = manager.session(source);
    __ESBMC_assert(&selected_session == &aid, "R-HEIGHT-FUSION: source accessor selects the required session");
    bool const visible = aid.active && !aid.fusion_reset && aid.fusion.diagnostics_valid;
    __ESBMC_assert(manager.reset_required(now, acceleration) == reset && manager.valid(now) == valid &&
                       manager.reference() == manager.m_reference &&
                       manager.progress_time() == manager.m_last_height_progress && actual.active == visible &&
                       same(actual.innovation, visible ? -aid.fusion_innovation : Scalar{0}) &&
                       same(actual.variance, visible ? aid.fusion.innovation_variance(0U) : Scalar{0}) &&
                       same_manager(manager, before),
                   "R-HEIGHT-FUSION: exact timeout/health query, selected innovation sign and suppression after reset "
                   "without mutation");
}
int main() { return 0; }
