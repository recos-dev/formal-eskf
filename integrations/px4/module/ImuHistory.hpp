// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include <formal_eskf/eskf/types.hpp>
#include <formal_eskf/runtime/ring_buffer.hpp>

#include "DeploymentProfile.hpp"
#include "ImuPrediction.hpp"

namespace formal_eskf::px4
{

struct ImuSource
{
    std::uint32_t accelerometer_id{};
    std::uint32_t gyroscope_id{};
    std::uint8_t accelerometer_calibration{};
    std::uint8_t gyroscope_calibration{};

    bool matches(ImuSource const & other) const
    {
        return other.accelerometer_id == accelerometer_id && other.gyroscope_id == gyroscope_id &&
               other.accelerometer_calibration == accelerometer_calibration &&
               other.gyroscope_calibration == gyroscope_calibration;
    }
};

/** Integrated body-FRD input shared by immediate and delayed prediction. */
template <typename Linalg> struct ImuInput
{
    using value_type = typename Linalg::value_type;
    using vector3_type = linalg::Vector<Linalg, 3>;

    vector3_type delta_angle_b{};    // rad
    vector3_type delta_velocity_b{}; // m/s
    std::uint64_t time_us{};         // Integration endpoint, PX4 monotonic microseconds.
    std::uint32_t angular_period_us{};
    std::uint32_t acceleration_period_us{};
    std::uint8_t accelerometer_clipping{};
    std::uint8_t gyroscope_clipping{};
    ImuSource source{};

    value_type angular_interval_s() const
    {
        return static_cast<value_type>(angular_period_us) * static_cast<value_type>(profile::seconds_per_microsecond);
    }
    value_type acceleration_interval_s() const
    {
        return static_cast<value_type>(acceleration_period_us) *
               static_cast<value_type>(profile::seconds_per_microsecond);
    }
    /** Requires admitted positive periods; callers check the resulting finite rates. */
    ImuSample<Linalg> mean_rates() const;
};

template <typename Linalg> ImuSample<Linalg> ImuInput<Linalg>::mean_rates() const
{
    auto const dt = angular_interval_s();
    auto const accel_dt = acceleration_interval_s();
    return {vector3_type::from_row_major(
                {delta_velocity_b(0) / accel_dt, delta_velocity_b(1) / accel_dt, delta_velocity_b(2) / accel_dt}),
            vector3_type::from_row_major({delta_angle_b(0) / dt, delta_angle_b(1) / dt, delta_angle_b(2) / dt})};
}

/** Startup values, matching the eventual adapter's signed integer parameter domain. */
struct TimingParameters
{
    std::int32_t filter_interval_us = profile::history::filter_interval_us;
    std::int32_t fusion_delay_ms = profile::history::fusion_delay_ms;
    std::int32_t gnss_delay_ms{};
    std::int32_t magnetic_delay_ms{};
    std::int32_t barometer_delay_ms{};
};

struct TimingConfiguration
{
    std::uint64_t minimum_lag_us{};
    std::uint64_t gnss_delay_us{};
    std::uint64_t magnetic_delay_us{};
    std::uint64_t barometer_delay_us{};
    std::size_t imu_length{}; // Output history must use the same length.
};

/** Validate the implemented 250 Hz mode. Failure preserves output; no clamping. */
[[nodiscard]] inline bool configure_timing(TimingParameters const & parameters, TimingConfiguration & output)
{
    if (parameters.filter_interval_us != profile::history::filter_interval_us || parameters.fusion_delay_ms <= 0 ||
        parameters.gnss_delay_ms < 0 || parameters.magnetic_delay_ms < 0 || parameters.barometer_delay_ms < 0)
    {
        return false;
    }
    // Widen before converting milliseconds; all nonnegative int32 values fit.
    TimingConfiguration candidate;
    candidate.minimum_lag_us = static_cast<std::uint64_t>(parameters.fusion_delay_ms) * 1000;
    candidate.gnss_delay_us = static_cast<std::uint64_t>(parameters.gnss_delay_ms) * 1000;
    candidate.magnetic_delay_us = static_cast<std::uint64_t>(parameters.magnetic_delay_ms) * 1000;
    candidate.barometer_delay_us = static_cast<std::uint64_t>(parameters.barometer_delay_ms) * 1000;
    auto const latest_arrival = std::max({profile::history::gnss_arrival_age_us + candidate.gnss_delay_us,
                                          profile::history::magnetic_arrival_age_us + candidate.magnetic_delay_us,
                                          profile::history::barometer_arrival_age_us + candidate.barometer_delay_us});
    if (candidate.minimum_lag_us < latest_arrival + profile::history::maximum_period_us / 2 ||
        candidate.minimum_lag_us > profile::history::maximum_lag_us)
    {
        return false;
    }
    auto const intervals = candidate.minimum_lag_us / profile::history::minimum_period_us +
                           (candidate.minimum_lag_us % profile::history::minimum_period_us != 0);
    if (intervals >= profile::history::imu_capacity)
    {
        return false;
    }
    candidate.imu_length = static_cast<std::size_t>(intervals) + 1;
    output = candidate;
    return true;
}

/**
 * Single-owner retained IMU timeline for PX4-TIME-IMU. No clocks or PX4 effects.
 * Configure, seed once, push admitted arrivals, then acknowledge successful
 * delayed prediction. The module owns current-output propagation and fusion ordering.
 */
template <typename Linalg> class ImuHistory
{
public:
    using input_type = ImuInput<Linalg>;
    using imu_type = ImuSample<Linalg>;

    struct Interval
    {
        std::uint64_t time_us{}; // Ring tag equals sample.time_us; assigned only by push().
        std::uint64_t start_time_us{};
        input_type sample{};
        imu_type held_rate{};
        std::uint32_t hold_period_us{}; // Zero means there is no synthetic segment.
    };

    enum class Fault
    {
        none,
        invalid_sample,
        discontinuity,
        history_loss,
        lag_out_of_range,
        invalid_completion
    };

    [[nodiscard]] bool configure(TimingParameters const & parameters);
    [[nodiscard]] bool seed(input_type const & sample, std::uint64_t now_us);
    [[nodiscard]] bool push(input_type const & sample, std::uint64_t now_us);

    /** Latest admitted interval for current-time output propagation; view expires on mutation. */
    Interval const * latest() const;
    /** Null during warmup, after completion, or on fault. View expires on mutation. */
    Interval const * delayed() const;
    /** Call only after the selected interval's numerical prediction has succeeded. */
    [[nodiscard]] bool complete(std::uint64_t time_us);
    /** Explicit session restart: preserve configuration, clear all temporal state. */
    void reset();

    TimingConfiguration const & configuration() const { return m_configuration; }
    ImuSource const & source() const { return m_previous.source; }
    Fault fault() const { return m_fault; }
    std::size_t entries() const { return m_samples.entries(); }
    std::uint64_t latest_time_us() const { return m_previous.time_us; }
    std::uint64_t fusion_time_us() const { return m_fusion_time_us; }

private:
    static bool valid_input(input_type const & sample, std::uint64_t now_us);
    bool fail(Fault reason);

    runtime::RingBuffer<Interval, profile::history::imu_capacity> m_samples{0};
    TimingConfiguration m_configuration{};
    input_type m_previous{};
    std::uint64_t m_fusion_time_us{};
    Fault m_fault = Fault::none;
}; /* end class ImuHistory */

template <typename Linalg> bool ImuHistory<Linalg>::configure(TimingParameters const & parameters)
{
    TimingConfiguration candidate;
    if (m_previous.time_us != 0 || m_fault != Fault::none || !configure_timing(parameters, candidate) ||
        !m_samples.allocate(candidate.imu_length))
    {
        return false;
    }
    m_configuration = candidate;
    return true;
}

template <typename Linalg> bool ImuHistory<Linalg>::valid_input(input_type const & sample, std::uint64_t now_us)
{
    if (sample.time_us == 0 || sample.time_us > now_us || now_us - sample.time_us > profile::output::maximum_age_us ||
        sample.angular_period_us < profile::history::minimum_period_us ||
        sample.angular_period_us > profile::history::maximum_period_us ||
        sample.acceleration_period_us < profile::history::minimum_period_us ||
        sample.acceleration_period_us > profile::history::maximum_period_us ||
        !imu_clipping_acceptable(sample.accelerometer_clipping, sample.gyroscope_clipping, true))
    {
        return false;
    }
    auto const rates = sample.mean_rates();
    return linalg::all_finite(rates.specific_force_b) && linalg::all_finite(rates.angular_rate_b);
}

template <typename Linalg> bool ImuHistory<Linalg>::seed(input_type const & sample, std::uint64_t now_us)
{
    if (!m_samples.valid() || m_previous.time_us != 0 || m_fault != Fault::none || !valid_input(sample, now_us))
    {
        return false;
    }
    m_previous = sample;
    m_fusion_time_us = sample.time_us;
    return true;
}

template <typename Linalg> bool ImuHistory<Linalg>::fail(Fault reason)
{
    m_fault = reason;
    return false;
}

template <typename Linalg> bool ImuHistory<Linalg>::push(input_type const & sample, std::uint64_t now_us)
{
    if (m_previous.time_us == 0 || m_fault != Fault::none)
    {
        return false;
    }
    if (!valid_input(sample, now_us) || sample.time_us <= m_previous.time_us)
    {
        return fail(Fault::invalid_sample);
    }
    auto const elapsed = sample.time_us - m_previous.time_us;
    if (elapsed < profile::history::minimum_period_us || elapsed > profile::imu::maximum_interval_us ||
        !m_previous.source.matches(sample.source))
    {
        return fail(Fault::discontinuity);
    }
    auto const gap = static_cast<std::int64_t>(elapsed) - static_cast<std::int64_t>(sample.angular_period_us);
    if (gap < -profile::imu::gap_tolerance_us)
    {
        return fail(Fault::discontinuity);
    }
    bool const full = m_samples.entries() == m_samples.get_length();
    if (full && m_samples.get_oldest().time_us > m_fusion_time_us)
    {
        return fail(Fault::history_loss);
    }
    // Check the retained span that would result from this push, before mutation.
    if (m_samples.entries() + 1 >= m_samples.get_length())
    {
        auto index = m_samples.get_oldest_index();
        if (full)
        {
            index = (index + 1) % m_samples.get_length();
        }
        auto const lag = sample.time_us - m_samples[index].time_us;
        if (lag < m_configuration.minimum_lag_us || lag > profile::history::maximum_lag_us)
        {
            return fail(Fault::lag_out_of_range);
        }
    }
    Interval interval;
    interval.time_us = sample.time_us;
    interval.start_time_us = m_previous.time_us;
    interval.sample = sample;
    if (gap > profile::imu::gap_tolerance_us)
    {
        interval.hold_period_us = static_cast<std::uint32_t>(gap);
        interval.held_rate = m_previous.mean_rates();
    }
    m_samples.push(interval);
    m_previous = sample;
    return true;
}

template <typename Linalg> typename ImuHistory<Linalg>::Interval const * ImuHistory<Linalg>::latest() const
{
    return m_fault == Fault::none && m_samples.entries() != 0 ? &m_samples.get_newest() : nullptr;
}

template <typename Linalg> typename ImuHistory<Linalg>::Interval const * ImuHistory<Linalg>::delayed() const
{
    if (m_fault != Fault::none || !m_samples.valid() || m_samples.entries() != m_samples.get_length())
    {
        return nullptr;
    }
    auto const & interval = m_samples.get_oldest();
    return interval.time_us > m_fusion_time_us ? &interval : nullptr;
}

template <typename Linalg> bool ImuHistory<Linalg>::complete(std::uint64_t time_us)
{
    if (m_fault != Fault::none)
    {
        return false;
    }
    auto const * interval = delayed();
    if (interval == nullptr || time_us != interval->time_us || interval->start_time_us != m_fusion_time_us)
    {
        return fail(Fault::invalid_completion);
    }
    m_fusion_time_us = time_us;
    return true;
}

template <typename Linalg> void ImuHistory<Linalg>::reset()
{
    m_samples.reset();
    m_previous = {};
    m_fusion_time_us = 0;
    m_fault = Fault::none;
}

} /* end namespace formal_eskf::px4 */
