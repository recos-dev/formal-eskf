/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once
#include "contracts.hpp"
#include "height_support.hpp"
#include "step_correction_support.hpp"
#include <formal_eskf/runtime/gnss_lifecycle.hpp>
#include <formal_eskf/runtime/navigation.hpp>
#include <formal_eskf/runtime/vertical_acceleration_health.hpp>
#include "../../../integrations/px4/module/DeploymentProfile.hpp"
// Expose private producer state only after dependencies are loaded. ESBMC
// mistakes a member named finite for its C intrinsic; alpha-renaming retains
// the actual body. The bounded library models are checked separately.
#include "height_sessions.hpp"
#define array FormalEskfHeightSessions
#define private public
#define max formal_eskf_proof_max
#define min formal_eskf_proof_min
#define any_of formal_eskf_proof_any_of
#define finite formal_eskf_proof_finite
#include "../../../integrations/px4/module/HeightAiding.hpp"
#undef finite
#undef any_of
#undef min
#undef max
#undef private
#undef array
namespace height_proof
{
using contract_proof::Scalar;
using correction_proof::same;
using correction_proof::same_matrix;
using formal_eskf::Status;
using Manager = formal_eskf::px4::HeightAiding<step_correction_proof::Backend>;
using Source = Manager::Source;
using Reason = Manager::StopReason;
using Session = Manager::Session;
using Sample = Manager::Sample;
using Result = Manager::Result;
using Bias = formal_eskf::px4::HeightBias<Scalar>;
namespace setting = formal_eskf::px4::profile::height;
static_assert(setting::fusion_timeout_us == 5000000U && setting::barometer_timeout_us == 400000U &&
                  setting::gnss_timeout_us == 1000000U && setting::barometer_maximum_age_us == 200000U &&
                  setting::gnss_maximum_age_us == 500000U && setting::minimum_altitude_m == -12000.0 &&
                  setting::maximum_altitude_m == 100000.0 && setting::barometer_warmup_samples == 20U &&
                  setting::barometer_lpf_gain == 0.1 && setting::maximum_aided_gnss_variance == 100.0 &&
                  setting::gate_sigma == 5.0 && setting::reset_velocity_variance == 10.0,
              "height deployment defaults match their specified constants");
inline bool same_bias(Bias const & a, Bias const & b)
{
    return same(a.value, b.value) && same(a.variance, b.variance) && same(a.maximum_variance, b.maximum_variance) &&
           same(a.signed_ratio, b.signed_ratio) && same(a.positive_time, b.positive_time) &&
           same(a.negative_time, b.negative_time) && same(a.dt, b.dt);
}
// Scratch may contain nonfinite arithmetic; update() rejects it before commit.
// The ceiling is always ordered, so the standard clamp precondition holds.
inline bool bias_domain(Bias const & bias)
{
    return std::isfinite(bias.maximum_variance) &&
           bias.maximum_variance >= static_cast<Scalar>(formal_eskf::px4::profile::height_bias::minimum_variance) &&
           (!std::isfinite(bias.variance) || bias.variance >= Scalar{0});
}
inline bool same_owner_constants(Manager const & a, Manager const & b)
{
    return same(a.m_origin_altitude, b.m_origin_altitude) && a.m_started == b.m_started && a.m_tick == b.m_tick;
}
inline bool history_domain(Manager const & owner, std::uint64_t epoch)
{
    if (owner.m_started > epoch || owner.m_tick > epoch || owner.m_last_height_progress > epoch ||
        owner.m_barometer_samples > setting::barometer_warmup_samples + 1U)
    {
        return false;
    }
    for (auto const & aid : owner.m_sessions)
    {
        if (aid.data_time > aid.sample_time || aid.sample_time > epoch || aid.progress_time > epoch ||
            aid.fused_time > epoch || aid.ratio.time_us > epoch || !bias_domain(aid.bias))
        {
            return false;
        }
    }
    return true;
}
inline bool same_ratio(formal_eskf::px4::InnovationRatio const & a, formal_eskf::px4::InnovationRatio const & b)
{
    double const left = a.filtered;
    double const right = b.filtered;
    return a.time_us == b.time_us &&
           (__builtin_bit_cast(std::uint64_t, left) == __builtin_bit_cast(std::uint64_t, right) ||
            (std::isnan(left) && std::isnan(right)));
}
inline bool same_session(Session const & a, Session const & b)
{
    auto const & x = a.fusion;
    auto const & y = b.fusion;
    return same_bias(a.bias, b.bias) && same_ratio(a.ratio, b.ratio) && x.status == y.status &&
           x.decision == y.decision && x.diagnostics_valid == y.diagnostics_valid &&
           same_matrix(x.innovation, y.innovation) && same_matrix(x.innovation_variance, y.innovation_variance) &&
           same_matrix(x.test_ratio, y.test_ratio) && same(a.fusion_innovation, b.fusion_innovation) &&
           a.innovation_limited == b.innovation_limited && a.sample_time == b.sample_time &&
           a.data_time == b.data_time && a.progress_time == b.progress_time && a.fused_time == b.fused_time &&
           a.device_id == b.device_id && a.calibration_count == b.calibration_count && a.generation == b.generation &&
           a.active == b.active && a.fusion_reset == b.fusion_reset && a.fault == b.fault &&
           a.stop_reason == b.stop_reason;
}
inline bool same_manager(Manager const & a, Manager const & b)
{
    return same_session(a.m_sessions[0U], b.m_sessions[0U]) && same_session(a.m_sessions[1U], b.m_sessions[1U]) &&
           a.m_reference == b.m_reference && same(a.m_origin_altitude, b.m_origin_altitude) &&
           same(a.m_barometer_lpf, b.m_barometer_lpf) && a.m_barometer_samples == b.m_barometer_samples &&
           a.m_started == b.m_started && a.m_tick == b.m_tick && a.m_last_height_progress == b.m_last_height_progress;
}
inline bool same_result(Result const & a, Result const & b)
{
    return a.status == b.status && a.fused == b.fused && a.rejected == b.rejected &&
           a.height_resets == b.height_resets && a.velocity_resets == b.velocity_resets &&
           same(a.height_delta, b.height_delta) && same(a.velocity_delta, b.velocity_delta);
}
inline bool same_sample(Sample const & a, Sample const & b)
{
    return a.time_us == b.time_us && a.device_id == b.device_id && a.calibration_count == b.calibration_count &&
           same(a.altitude, b.altitude) && same(a.variance, b.variance) && a.qualified == b.qualified &&
           a.generation == b.generation;
}
inline bool altitude_valid(Scalar altitude)
{
    return std::isfinite(altitude) && altitude > static_cast<Scalar>(setting::minimum_altitude_m) &&
           altitude < static_cast<Scalar>(setting::maximum_altitude_m);
}
inline void stopped(Manager & expected, Source source, Reason reason)
{
    auto & aid = expected.m_sessions[source == Source::barometer ? 0U : 1U];
    aid.active = false;
    aid.stop_reason = reason;
    aid.fusion = {};
    if (expected.m_reference == source)
    {
        expected.m_reference = Source::none;
    }
}

} // namespace height_proof
