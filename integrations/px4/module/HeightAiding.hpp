// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include <formal_eskf/runtime/gnss_fusion.hpp>
#include <formal_eskf/runtime/navigation_measurement.hpp>
#include <formal_eskf/runtime/navigation_reset.hpp>
#include <formal_eskf/runtime/position.hpp>
#include <formal_eskf/runtime/vertical_acceleration_health.hpp>

#include "BiasLimits.hpp"
#include "HeightBias.hpp"

namespace formal_eskf::px4
{

/** GNSS-reference height management for the two-source PX4 deployment profile. */
template <typename Linalg> class HeightAiding
{
public:
    using value_type = typename Linalg::value_type;
    using state_type = configuration::Ins::NominalState<Linalg>;
    using covariance_type = linalg::Matrix<Linalg, 15U, 15U>;
    using fusion_type = runtime::FusionResult<Linalg, 1U>;

    enum class Source : std::uint8_t
    {
        barometer,
        gnss,
        none
    };

    enum class StopReason
    {
        none,
        no_data,
        invalid_sample,
        quality,
        source_changed,
        fusion_timeout
    };

    struct Sample
    {
        std::uint64_t time_us{};
        std::uint32_t device_id{};
        std::uint8_t calibration_count{};
        value_type altitude{}; // Up-positive MSL metres; same datum as origin_altitude.
        value_type variance{};
        bool qualified = true;
    };

    struct Acceleration
    {
        bool bad = false; // Held fault, independent of last-good reset timing.
        runtime::GnssFusion::VerticalAccelerationTimes times{};
    };

    struct Session
    {
        HeightBias<value_type> bias{};
        fusion_type fusion{}; // Raw gate diagnostics.
        value_type fusion_innovation{};
        bool innovation_limited = false;
        std::uint64_t sample_time{}, data_time{}, progress_time{}, fused_time{};
        std::uint32_t device_id{};
        std::uint8_t calibration_count{};
        bool active = false;
        bool fusion_reset = false; // Raw diagnostics remain visible, but cannot drive health after a reset.
        bool fault = false;        // Barometer rejection latch; source/calibration change clears it.
        StopReason stop_reason = StopReason::none;
    };

    struct Result
    {
        Status status = Status::success;
        unsigned fused{}, rejected{}, height_resets{}, velocity_resets{};
        value_type velocity_delta{};
    };

    static constexpr std::uint64_t fusion_timeout_us = 5000000U;
    static constexpr value_type barometer_variance = static_cast<value_type>(3.5 * 3.5);
    static constexpr value_type gate_sigma = value_type{5};
    static constexpr unsigned barometer_warmup_samples = 20U; // 200 ms / 10 ms pinned profile.

    [[nodiscard]] Status initialize(std::uint64_t now_us, value_type origin_altitude) noexcept
    {
        if (now_us == 0U || !valid_altitude(origin_altitude))
        {
            return Status::domain_error;
        }
        *this = HeightAiding{};
        m_origin_altitude = origin_altitude;
        m_started = now_us;
        return Status::success;
    }

    /** One increasing estimator tick: barometer before GNSS; future samples wait. */
    [[nodiscard]] Result update(std::uint64_t now_us, Sample const & barometer, Sample const & gnss,
                                bool vertical_velocity_aiding, state_type & state, covariance_type & covariance,
                                configuration::Ins::BiasUpdate const & bias_update = {},
                                Acceleration const & acceleration = {false, {0U, 0U}}) noexcept;

    /** Inspect one sensor session; source must be barometer or gnss. */
    [[nodiscard]] Session const & session(Source source) const noexcept { return m_sessions[index(source)]; }
    [[nodiscard]] typename runtime::VerticalAccelerationHealth<Linalg>::Innovation
    vertical_innovation(Source source) const noexcept
    {
        auto const & aid = session(source);
        if (!aid.active || aid.fusion_reset || !aid.fusion.diagnostics_valid)
        {
            return {};
        }
        return {true, -aid.fusion_innovation, aid.fusion.innovation_variance(0U)};
    }
    [[nodiscard]] Source reference() const noexcept { return m_reference; }
    [[nodiscard]] std::uint64_t progress_time() const noexcept { return m_last_height_progress; }
    [[nodiscard]] std::uint8_t reset_counter() const noexcept { return m_reset_counter; }
    [[nodiscard]] value_type reset_delta() const noexcept { return m_reset_delta; }
    [[nodiscard]] bool reset_required(std::uint64_t now_us,
                                      runtime::GnssFusion::VerticalAccelerationTimes const & acceleration = {
                                          0U, 0U}) const noexcept
    {
        auto const progress = m_last_height_progress == 0U ? m_started : m_last_height_progress;
        return acceleration.reset_required(now_us) ||
               (progress != 0U && now_us > progress && now_us - progress > fusion_timeout_us);
    }
    [[nodiscard]] bool valid(std::uint64_t now_us) const noexcept
    {
        return m_last_height_progress != 0U && now_us >= m_last_height_progress &&
               now_us - m_last_height_progress <= fusion_timeout_us;
    }

private:
    [[nodiscard]] static constexpr std::size_t index(Source source) noexcept
    {
        return source == Source::barometer ? 0U : 1U;
    }
    [[nodiscard]] static bool valid_altitude(value_type altitude) noexcept
    {
        return std::isfinite(altitude) && altitude > value_type{-12000} && altitude < value_type{100000};
    }
    void stop(Source source, StopReason reason) noexcept;
    void process(Source source, std::uint64_t now_us, Sample const & sample, bool vertical_velocity_aiding,
                 state_type & state, covariance_type & covariance, Result & result,
                 configuration::Ins::BiasUpdate const & bias_update, Acceleration const & acceleration) noexcept;
    void reset_height(Source source, std::uint64_t now_us, value_type altitude, value_type variance, state_type & state,
                      covariance_type & covariance, Result & result) noexcept;

    std::array<Session, 2U> m_sessions{};
    Source m_reference = Source::none;
    value_type m_origin_altitude{}, m_barometer_lpf{}, m_reset_delta{};
    unsigned m_barometer_samples{};
    std::uint8_t m_reset_counter{};
    std::uint64_t m_started{}, m_tick{}, m_last_height_progress{};
}; /* end class HeightAiding */

template <typename Linalg> void HeightAiding<Linalg>::stop(Source source, StopReason reason) noexcept
{
    auto & aid = m_sessions[index(source)];
    aid.active = false;
    aid.stop_reason = reason;
    aid.fusion = {};
    if (m_reference == source)
    {
        m_reference = Source::none;
    }
}

template <typename Linalg>
void HeightAiding<Linalg>::reset_height(Source source, std::uint64_t now_us, value_type altitude, value_type variance,
                                        state_type & state, covariance_type & covariance, Result & result) noexcept
{
    value_type delta{};
    // The existing checked primitive replaces one Down coordinate regardless of sensor origin.
    result.status = runtime::try_reset_vertical_position(state, covariance, m_origin_altitude - altitude, variance,
                                                         state, covariance, delta);
    if (!succeeded(result.status))
    {
        return;
    }
    for (auto & aid : m_sessions)
    {
        aid.bias.value += delta; // Preserve each corrected altitude relative to the reset state.
    }
    m_reset_delta = result.height_resets == 0U ? delta : m_reset_delta + delta;
    ++m_reset_counter;
    ++result.height_resets;
    m_last_height_progress = now_us;
    auto & aid = m_sessions[index(source)];
    aid.progress_time = now_us;
    aid.fusion_reset = true;
}

template <typename Linalg>
void HeightAiding<Linalg>::process(Source source, std::uint64_t now_us, Sample const & sample,
                                   bool vertical_velocity_aiding, state_type & state, covariance_type & covariance,
                                   Result & result, configuration::Ins::BiasUpdate const & bias_update,
                                   Acceleration const & acceleration) noexcept
{
    auto & aid = m_sessions[index(source)];
    bool const barometer = source == Source::barometer;
    auto const data_timeout = barometer ? 400000U : 1000000U;
    auto const maximum_age = barometer ? 200000U : 500000U;
    if (sample.time_us == 0U || sample.time_us <= aid.sample_time || sample.time_us > now_us)
    {
        if (aid.active && now_us - aid.data_time >= data_timeout)
        {
            stop(source, StopReason::no_data);
        }
        return;
    }
    aid.sample_time = sample.time_us;
    if (now_us - sample.time_us >= maximum_age)
    {
        ++result.rejected;
        if (aid.active && now_us - aid.data_time >= data_timeout)
        {
            stop(source, StopReason::no_data);
        }
        return;
    }
    if (sample.device_id == 0U || !valid_altitude(sample.altitude) || !std::isfinite(sample.variance) ||
        sample.variance <= value_type{0})
    {
        stop(source, StopReason::invalid_sample);
        ++result.rejected;
        return;
    }
    if (aid.device_id != sample.device_id || aid.calibration_count != sample.calibration_count)
    {
        stop(source, StopReason::source_changed);
        aid.device_id = sample.device_id;
        aid.calibration_count = sample.calibration_count;
        aid.bias = {};
        aid.fault = false;
        if (barometer)
        {
            m_barometer_samples = 0U;
        }
    }
    if (!sample.qualified)
    {
        stop(source, StopReason::quality);
        ++result.rejected;
        return;
    }
    aid.data_time = sample.time_us;
    value_type const altitude = m_origin_altitude - state.p_n(2U);
    if (barometer)
    {
        m_barometer_lpf = m_barometer_samples == 0U
                              ? sample.altitude
                              : m_barometer_lpf + static_cast<value_type>(0.1) * (sample.altitude - m_barometer_lpf);
        m_barometer_samples = std::min(m_barometer_samples + 1U, barometer_warmup_samples + 1U);
        if (m_barometer_samples <= barometer_warmup_samples)
        {
            aid.bias.value = m_barometer_lpf - altitude;
            return;
        }
        if (aid.fault)
        {
            ++result.rejected;
            return;
        }
    }
    value_type const variance = !barometer && m_sessions[index(Source::barometer)].active
                                    ? std::min(sample.variance, value_type{100})
                                    : sample.variance;
    // Freeze the observation at the PRIOR bias; this sample's bias update serves the next sample.
    value_type const observation = m_origin_altitude - (sample.altitude - aid.bias.value);
    value_type const observation_variance = variance + aid.bias.variance;
    if (aid.active && m_reference != source)
    {
        aid.bias.fuse(sample.altitude - altitude, variance + covariance(2U, 2U), variance);
    }
    if (!aid.active)
    {
        if (source == Source::gnss)
        {
            reset_height(source, now_us, sample.altitude, variance, state, covariance, result);
            if (!succeeded(result.status))
            {
                return;
            }
            aid.bias = {};
            m_reference = source;
        }
        else
        {
            aid.bias.value = m_barometer_lpf - altitude;
        }
        aid.active = true;
        aid.stop_reason = StopReason::none;
        aid.progress_time = now_us; // Startup grace, distinct from a successful correction.
        return;
    }
    aid.fusion_reset = false;
    // Both qualified sources observe scalar NED Down; source policy and
    // altitude conversion stay here.
    auto const measurement =
        runtime::fuse_height(state, covariance, observation, observation_variance, static_cast<value_type>(1e-6),
                             gate_sigma, acceleration.bad, state, covariance, bias_update);
    aid.fusion = measurement.fusion;
    aid.fusion_innovation = measurement.fusion_innovation(0U);
    aid.innovation_limited = measurement.innovation_limited;
    result.status = aid.fusion.status;
    if (!succeeded(result.status))
    {
        return;
    }
    if (aid.fusion.decision == runtime::FusionDecision::fused)
    {
        result.status = try_limit_imu_bias(state);
        if (!succeeded(result.status))
        {
            return;
        }
        aid.fused_time = aid.progress_time = m_last_height_progress = now_us;
        ++result.fused;
    }
    else
    {
        ++result.rejected;
    }
    if (m_reference == source && reset_required(now_us, acceleration.times))
    {
        value_type const reset_altitude = barometer ? m_barometer_lpf - aid.bias.value : sample.altitude;
        reset_height(source, now_us, reset_altitude, variance, state, covariance, result);
        if (!succeeded(result.status))
        {
            return;
        }
        aid.bias.value = (barometer ? m_barometer_lpf : sample.altitude) - (m_origin_altitude - state.p_n(2U));
        if (barometer && !vertical_velocity_aiding)
        {
            result.status = runtime::try_reset_vertical_velocity(state, covariance, value_type{0}, value_type{10},
                                                                 state, covariance, result.velocity_delta);
            if (succeeded(result.status))
            {
                ++result.velocity_resets;
            }
        }
    }
    else if (now_us - aid.progress_time > fusion_timeout_us)
    {
        stop(source, StopReason::fusion_timeout);
        aid.fault = barometer && !reset_required(now_us, acceleration.times);
    }
}

template <typename Linalg>
typename HeightAiding<Linalg>::Result HeightAiding<Linalg>::update(std::uint64_t now_us, Sample const & barometer,
                                                                   Sample const & gnss, bool vertical_velocity_aiding,
                                                                   state_type & state, covariance_type & covariance,
                                                                   configuration::Ins::BiasUpdate const & bias_update,
                                                                   Acceleration const & acceleration) noexcept
{
    Result result;
    if (m_started == 0U || now_us < m_started || now_us <= m_tick)
    {
        result.status = Status::out_of_range;
        return result;
    }
    if (acceleration.times.update_time_us > now_us ||
        acceleration.times.last_good_time_us > acceleration.times.update_time_us ||
        (acceleration.bad && acceleration.times.update_time_us == 0U))
    {
        result.status = Status::out_of_range;
        return result;
    }
    if (!std::isfinite(state.p_n(2U)) || !std::isfinite(covariance(2U, 2U)))
    {
        result.status = Status::non_finite_input;
        return result;
    }
    if (covariance(2U, 2U) < value_type{0})
    {
        result.status = Status::domain_error;
        return result;
    }
    auto candidate = *this;
    auto next_state = state;
    auto next_covariance = covariance;
    value_type const dt =
        m_tick == 0U ? value_type{0} : static_cast<value_type>(now_us - m_tick) * static_cast<value_type>(1e-6);
    for (Source source : {Source::barometer, Source::gnss})
    {
        auto & aid = candidate.m_sessions[index(source)];
        if (aid.active && candidate.m_reference != source)
        {
            aid.bias.predict(dt);
        }
        candidate.process(source, now_us, source == Source::barometer ? barometer : gnss, vertical_velocity_aiding,
                          next_state, next_covariance, result, bias_update, acceleration);
        if (!succeeded(result.status))
        {
            return Result{result.status};
        }
    }
    if (candidate.m_reference == Source::none)
    {
        candidate.m_reference = candidate.session(Source::gnss).active        ? Source::gnss
                                : candidate.session(Source::barometer).active ? Source::barometer
                                                                              : Source::none;
    }
    if (std::any_of(candidate.m_sessions.begin(), candidate.m_sessions.end(),
                    [](Session const & aid) { return !aid.bias.finite(); }))
    {
        return Result{Status::non_finite_result};
    }
    candidate.m_tick = now_us;
    *this = candidate;
    state = next_state;
    covariance = next_covariance;
    return result;
}

} /* end namespace formal_eskf::px4 */
