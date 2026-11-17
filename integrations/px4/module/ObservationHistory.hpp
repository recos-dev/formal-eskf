// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstddef>
#include <cstdint>

#include "ImuHistory.hpp"

namespace formal_eskf::px4
{

/**
 * Single-owner observation/context histories for PX4-TIME-OBSERVATIONS.
 * Inputs have PX4 monotonic sample timestamps; stored observation time_us is
 * the effective fusion epoch. No uORB, clock, projection or quality policy is
 * owned here. A queued event is not a quality pass or permission to fuse.
 */
template <typename Linalg> class ObservationHistory
{
    static constexpr auto retention_us = profile::history::maximum_lag_us +
                                         profile::history::observation_maximum_age_us + profile::output::maximum_age_us;
    static_assert(profile::history::gnss_capacity >=
                  (retention_us + profile::history::gnss_spacing_us - 1) / profile::history::gnss_spacing_us + 2);
    static_assert(profile::history::magnetic_capacity >=
                  (retention_us + profile::history::magnetic_spacing_us - 1) / profile::history::magnetic_spacing_us +
                      2);
    static_assert(profile::history::barometer_capacity >=
                  (retention_us + profile::history::barometer_spacing_us - 1) / profile::history::barometer_spacing_us +
                      2);
    static_assert((profile::history::context_capacity - 1) * profile::history::minimum_period_us >=
                  profile::history::maximum_lag_us + profile::history::observation_maximum_age_us +
                      profile::imu::maximum_interval_us);

public:
    using value_type = typename Linalg::value_type;
    using vector3_type = linalg::Vector<Linalg, 3>;
    using interval_type = typename ImuHistory<Linalg>::Interval;

    struct Source
    {
        std::uint32_t device_id{};        // Zero is an explicit unavailable-source indication.
        std::uint8_t calibration_count{}; // GNSS uses zero.
        bool matches(Source const & other) const
        {
            return device_id == other.device_id && calibration_count == other.calibration_count;
        }
    };

    struct SourceStamp
    {
        std::uint64_t time_us{}; // Original sample time, never the arrival clock.
        Source source{};
    };

    struct GnssSample
    {
        SourceStamp stamp{};
        double latitude_deg{}, longitude_deg{}, altitude_msl_m{}, altitude_ellipsoid_m{};
        vector3_type velocity_n{};                                                       // NED, m/s.
        value_type horizontal_accuracy_m{}, vertical_accuracy_m{}, speed_accuracy_m_s{}; // Standard deviations.
        value_type hdop{}, vdop{};
        std::uint8_t fix_type{}, satellites{};
        bool velocity_valid{}, spoofed{};
    };

    struct MagneticSample
    {
        SourceStamp stamp{};
        vector3_type field_b{}; // Calibrated body FRD, microtesla.
    };

    struct BarometerSample
    {
        SourceStamp stamp{};
        value_type altitude_m{}; // Producer's pressure altitude, before height-bias estimation.
    };

    struct GnssObservation
    {
        std::uint64_t time_us{}, arrival_time_us{}, residual_delay_us{};
        std::uint32_t half_period_us{};
        GnssSample sample{};
    };

    struct MagneticObservation
    {
        std::uint64_t time_us{}, arrival_time_us{}, residual_delay_us{};
        std::uint32_t half_period_us{};
        MagneticSample sample{};
    };

    struct BarometerObservation
    {
        std::uint64_t time_us{}, arrival_time_us{}, residual_delay_us{};
        std::uint32_t half_period_us{};
        BarometerSample sample{};
    };

    struct LandState
    {
        std::uint64_t time_us{}; // Original land-detector message time.
        bool known{}, landed{}, at_rest{};
    };

    struct VehicleState
    {
        std::uint64_t time_us{}; // Original vehicle-status message time.
        bool known{}, armed{};
    };

    struct Context
    {
        std::uint64_t time_us{}; // Real IMU endpoint when these values were known.
        LandState land{};
        VehicleState vehicle{};
    };

    struct Seed
    {
        Context context{};
        SourceStamp gnss{}, magnetic{}, barometer{}; // Transfer alignment watermarks and identities.
    };

    enum class Admission
    {
        queued,
        not_ready,
        invalid_time,
        out_of_order,
        invalid_source,
        stale,
        imu_stale,
        offset_underflow,
        before_seed,
        too_fast
    };

    struct PushResult
    {
        Admission decision = Admission::not_ready;
        bool source_changed{};
        std::size_t source_discarded{};
        bool overwritten{};
    };

    enum class Selection
    {
        not_ready,
        repeated,
        empty,
        selected
    };
    struct PopResult
    {
        Selection decision = Selection::not_ready;
        // Older superseded records on selection; expired records on an empty result.
        std::size_t retired{};
    };

    [[nodiscard]] bool configure(TimingParameters const & parameters);
    [[nodiscard]] bool seed(Seed const & initial);
    /** Exactly once per newly admitted real IMU interval; no synthetic context samples. */
    [[nodiscard]] bool record_imu(interval_type const & interval, LandState const & land, VehicleState const & vehicle);
    /** After successful delayed prediction, before selection/policies; at most once per new IMU endpoint. */
    [[nodiscard]] bool begin_fusion(interval_type const & interval);

    [[nodiscard]] PushResult push_gnss(GnssSample const & sample, std::uint64_t now_us);
    [[nodiscard]] PushResult push_magnetic(MagneticSample const & sample, std::uint64_t now_us);
    [[nodiscard]] PushResult push_barometer(BarometerSample const & sample, std::uint64_t now_us);
    /** Once per source per begun fusion step. Share the single GNSS event among all its consumers. */
    [[nodiscard]] PopResult pop_gnss(GnssObservation & output);
    [[nodiscard]] PopResult pop_magnetic(MagneticObservation & output);
    [[nodiscard]] PopResult pop_barometer(BarometerObservation & output);

    /** Non-consuming; missing/stale context is unknown. Use the caller's source-specific age limit. */
    Context context(std::uint64_t time_us, std::uint64_t maximum_age_us) const;
    std::uint64_t fusion_time_us() const { return m_fusion_time_us; }
    std::uint32_t half_period_us() const { return static_cast<std::uint32_t>(m_average_period_us * 0.5); }
    void reset(); // Retain startup configuration; clear histories, watermarks, context and timing average.

private:
    struct Timeline
    {
        Source source{};
        std::uint64_t sample_time_us{}, arrival_time_us{}, admitted_time_us{}, selection_time_us{};
    };
    struct Limits
    {
        std::uint64_t delay_us{}, arrival_age_us{}, spacing_us{};
    };

    static bool valid_seed_source(SourceStamp const & source, std::uint64_t epoch);
    static Context snapshot(Context const & previous, std::uint64_t time_us, LandState const & land,
                            VehicleState const & vehicle);
    PushResult admit(Timeline & timeline, SourceStamp const & stamp, std::uint64_t now_us, Limits const & limits,
                     std::uint64_t & effective_time_us);
    PopResult select(Timeline & timeline);

    runtime::RingBuffer<GnssObservation, profile::history::gnss_capacity> m_gnss{profile::history::gnss_capacity};
    runtime::RingBuffer<MagneticObservation, profile::history::magnetic_capacity> m_magnetic{
        profile::history::magnetic_capacity};
    runtime::RingBuffer<BarometerObservation, profile::history::barometer_capacity> m_barometer{
        profile::history::barometer_capacity};
    runtime::RingBuffer<Context, profile::history::context_capacity> m_context{profile::history::context_capacity};
    Timeline m_gnss_time{}, m_magnetic_time{}, m_barometer_time{};
    TimingConfiguration m_configuration{};
    std::uint64_t m_seed_time_us{}, m_latest_time_us{}, m_fusion_time_us{}, m_advanced_at_us{};
    double m_average_period_us = profile::history::filter_interval_us;
    bool m_configured{};
}; /* end class ObservationHistory */

template <typename Linalg> bool ObservationHistory<Linalg>::configure(TimingParameters const & parameters)
{
    TimingConfiguration candidate;
    if (m_seed_time_us != 0 || !configure_timing(parameters, candidate))
    {
        return false;
    }
    m_configuration = candidate;
    m_configured = true;
    return true;
}

template <typename Linalg>
bool ObservationHistory<Linalg>::valid_seed_source(SourceStamp const & source, std::uint64_t epoch)
{
    return source.time_us <= epoch && (source.source.device_id == 0 || source.time_us != 0);
}

template <typename Linalg>
typename ObservationHistory<Linalg>::Context
ObservationHistory<Linalg>::snapshot(Context const & previous, std::uint64_t time_us, LandState const & land,
                                     VehicleState const & vehicle)
{
    auto result = previous;
    result.time_us = time_us;
    if (land.time_us > previous.land.time_us && land.time_us <= time_us)
    {
        result.land = land;
    }
    if (vehicle.time_us > previous.vehicle.time_us && vehicle.time_us <= time_us)
    {
        result.vehicle = vehicle;
    }
    return result;
}

template <typename Linalg> bool ObservationHistory<Linalg>::seed(Seed const & initial)
{
    auto const epoch = initial.context.time_us;
    if (!m_configured || m_seed_time_us != 0 || epoch == 0 || !valid_seed_source(initial.gnss, epoch) ||
        !valid_seed_source(initial.magnetic, epoch) || !valid_seed_source(initial.barometer, epoch))
    {
        return false;
    }
    m_context.push(snapshot({}, epoch, initial.context.land, initial.context.vehicle));
    m_gnss_time = {initial.gnss.source, initial.gnss.time_us};
    m_magnetic_time = {initial.magnetic.source, initial.magnetic.time_us};
    m_barometer_time = {initial.barometer.source, initial.barometer.time_us};
    m_seed_time_us = epoch;
    m_latest_time_us = epoch;
    m_fusion_time_us = epoch;
    return true;
}

template <typename Linalg>
bool ObservationHistory<Linalg>::record_imu(interval_type const & interval, LandState const & land,
                                            VehicleState const & vehicle)
{
    if (m_seed_time_us == 0 || interval.start_time_us != m_latest_time_us ||
        interval.time_us != interval.sample.time_us || interval.time_us <= m_latest_time_us)
    {
        return false;
    }
    auto const elapsed = interval.time_us - m_latest_time_us;
    if (elapsed < profile::history::minimum_period_us || elapsed > profile::imu::maximum_interval_us)
    {
        return false;
    }
    m_context.push(snapshot(m_context.get_newest(), interval.time_us, land, vehicle));
    m_latest_time_us = interval.time_us;
    return true;
}

template <typename Linalg> bool ObservationHistory<Linalg>::begin_fusion(interval_type const & interval)
{
    auto const & sample = interval.sample;
    if (m_seed_time_us == 0 || m_advanced_at_us == m_latest_time_us || interval.start_time_us != m_fusion_time_us ||
        interval.time_us != sample.time_us || interval.time_us <= m_fusion_time_us ||
        interval.time_us > m_latest_time_us || sample.angular_period_us < profile::history::minimum_period_us ||
        sample.angular_period_us > profile::history::maximum_period_us ||
        sample.acceleration_period_us < profile::history::minimum_period_us ||
        sample.acceleration_period_us > profile::history::maximum_period_us)
    {
        return false;
    }
    auto const lag = m_latest_time_us - interval.time_us;
    if (lag < m_configuration.minimum_lag_us || lag > profile::history::maximum_lag_us)
    {
        return false;
    }
    // Match the main-filter period reference: mean of the two REAL integration periods, never the held gap.
    double const period_us = 0.5 * (static_cast<double>(sample.angular_period_us) + sample.acceleration_period_us);
    m_average_period_us =
        profile::imu::interval_history_weight * m_average_period_us + profile::imu::interval_sample_weight * period_us;
    m_fusion_time_us = interval.time_us;
    m_advanced_at_us = m_latest_time_us;
    return true;
}

template <typename Linalg>
typename ObservationHistory<Linalg>::PushResult
ObservationHistory<Linalg>::admit(Timeline & timeline, SourceStamp const & stamp, std::uint64_t now_us,
                                  Limits const & limits, std::uint64_t & effective_time_us)
{
    PushResult result;
    if (m_seed_time_us == 0)
    {
        return result;
    }
    if (now_us < m_latest_time_us || now_us < timeline.arrival_time_us || stamp.time_us == 0 || stamp.time_us > now_us)
    {
        result.decision = Admission::invalid_time;
        return result;
    }
    timeline.arrival_time_us = now_us;
    if (stamp.time_us <= timeline.sample_time_us)
    {
        result.decision = Admission::out_of_order;
        return result;
    }
    // Even a rate/staleness rejection must expose a new source or an unavailable indication.
    result.source_changed = !stamp.source.matches(timeline.source);
    timeline.source = stamp.source;
    timeline.sample_time_us = stamp.time_us;
    if (stamp.source.device_id == 0)
    {
        result.decision = Admission::invalid_source;
    }
    else if (now_us - stamp.time_us >= limits.arrival_age_us)
    {
        result.decision = Admission::stale;
    }
    else if (now_us - m_latest_time_us >= profile::output::maximum_age_us)
    {
        result.decision = Admission::imu_stale;
    }
    else if (stamp.time_us <= limits.delay_us || stamp.time_us - limits.delay_us <= half_period_us())
    {
        result.decision = Admission::offset_underflow;
    }
    else
    {
        effective_time_us = stamp.time_us - limits.delay_us - half_period_us();
        if (effective_time_us <= m_seed_time_us)
        {
            result.decision = Admission::before_seed;
        }
        else if (timeline.admitted_time_us != 0 && (effective_time_us <= timeline.admitted_time_us ||
                                                    effective_time_us - timeline.admitted_time_us < limits.spacing_us))
        {
            result.decision = Admission::too_fast;
        }
        else
        {
            timeline.admitted_time_us = effective_time_us;
            result.decision = Admission::queued;
        }
    }
    return result;
}

template <typename Linalg>
typename ObservationHistory<Linalg>::PushResult ObservationHistory<Linalg>::push_gnss(GnssSample const & sample,
                                                                                      std::uint64_t now_us)
{
    std::uint64_t time{};
    auto const delay = m_configuration.gnss_delay_us;
    auto result = admit(m_gnss_time, sample.stamp, now_us,
                        {delay, profile::history::gnss_arrival_age_us, profile::history::gnss_spacing_us}, time);
    if (result.source_changed)
    {
        result.source_discarded = m_gnss.entries();
        m_gnss.reset();
    }
    if (result.decision == Admission::queued)
    {
        result.overwritten = m_gnss.entries() == m_gnss.get_length();
        m_gnss.push({time, now_us, delay, half_period_us(), sample});
    }
    return result;
}

template <typename Linalg>
typename ObservationHistory<Linalg>::PushResult ObservationHistory<Linalg>::push_magnetic(MagneticSample const & sample,
                                                                                          std::uint64_t now_us)
{
    std::uint64_t time{};
    auto const delay = m_configuration.magnetic_delay_us;
    auto result =
        admit(m_magnetic_time, sample.stamp, now_us,
              {delay, profile::history::magnetic_arrival_age_us, profile::history::magnetic_spacing_us}, time);
    if (result.source_changed)
    {
        result.source_discarded = m_magnetic.entries();
        m_magnetic.reset();
    }
    if (result.decision == Admission::queued)
    {
        result.overwritten = m_magnetic.entries() == m_magnetic.get_length();
        m_magnetic.push({time, now_us, delay, half_period_us(), sample});
    }
    return result;
}

template <typename Linalg>
typename ObservationHistory<Linalg>::PushResult
ObservationHistory<Linalg>::push_barometer(BarometerSample const & sample, std::uint64_t now_us)
{
    std::uint64_t time{};
    auto const delay = m_configuration.barometer_delay_us;
    auto result =
        admit(m_barometer_time, sample.stamp, now_us,
              {delay, profile::history::barometer_arrival_age_us, profile::history::barometer_spacing_us}, time);
    if (result.source_changed)
    {
        result.source_discarded = m_barometer.entries();
        m_barometer.reset();
    }
    if (result.decision == Admission::queued)
    {
        result.overwritten = m_barometer.entries() == m_barometer.get_length();
        m_barometer.push({time, now_us, delay, half_period_us(), sample});
    }
    return result;
}

template <typename Linalg>
typename ObservationHistory<Linalg>::PopResult ObservationHistory<Linalg>::select(Timeline & timeline)
{
    if (m_advanced_at_us == 0)
    {
        return {};
    }
    if (timeline.selection_time_us == m_fusion_time_us)
    {
        return {Selection::repeated};
    }
    timeline.selection_time_us = m_fusion_time_us;
    return {Selection::empty};
}

template <typename Linalg>
typename ObservationHistory<Linalg>::PopResult ObservationHistory<Linalg>::pop_gnss(GnssObservation & output)
{
    auto result = select(m_gnss_time);
    if (result.decision == Selection::empty)
    {
        auto const count = m_gnss.entries();
        bool const selected =
            m_gnss.pop_first_older_than(m_fusion_time_us, &output, profile::history::observation_maximum_age_us);
        result.decision = selected ? Selection::selected : Selection::empty;
        result.retired = count - m_gnss.entries() - static_cast<std::size_t>(selected);
    }
    return result;
}

template <typename Linalg>
typename ObservationHistory<Linalg>::PopResult ObservationHistory<Linalg>::pop_magnetic(MagneticObservation & output)
{
    auto result = select(m_magnetic_time);
    if (result.decision == Selection::empty)
    {
        auto const count = m_magnetic.entries();
        bool const selected =
            m_magnetic.pop_first_older_than(m_fusion_time_us, &output, profile::history::observation_maximum_age_us);
        result.decision = selected ? Selection::selected : Selection::empty;
        result.retired = count - m_magnetic.entries() - static_cast<std::size_t>(selected);
    }
    return result;
}

template <typename Linalg>
typename ObservationHistory<Linalg>::PopResult ObservationHistory<Linalg>::pop_barometer(BarometerObservation & output)
{
    auto result = select(m_barometer_time);
    if (result.decision == Selection::empty)
    {
        auto const count = m_barometer.entries();
        bool const selected =
            m_barometer.pop_first_older_than(m_fusion_time_us, &output, profile::history::observation_maximum_age_us);
        result.decision = selected ? Selection::selected : Selection::empty;
        result.retired = count - m_barometer.entries() - static_cast<std::size_t>(selected);
    }
    return result;
}

template <typename Linalg>
typename ObservationHistory<Linalg>::Context ObservationHistory<Linalg>::context(std::uint64_t time_us,
                                                                                 std::uint64_t maximum_age_us) const
{
    if (m_seed_time_us == 0 || time_us < m_seed_time_us || time_us > m_fusion_time_us)
    {
        return {};
    }
    auto index = (m_context.get_oldest_index() + m_context.entries()) % m_context.get_length();
    for (std::size_t count = 0; count < m_context.entries(); ++count)
    {
        index = index == 0 ? m_context.get_length() - 1 : index - 1;
        auto const & sample = m_context[index];
        if (sample.time_us > time_us)
        {
            continue;
        }
        if (time_us - sample.time_us >= profile::imu::maximum_interval_us)
        {
            return {};
        }
        auto result = sample;
        result.land.known = result.land.known && time_us - result.land.time_us < maximum_age_us;
        result.vehicle.known = result.vehicle.known && time_us - result.vehicle.time_us < maximum_age_us;
        return result;
    }
    return {};
}

template <typename Linalg> void ObservationHistory<Linalg>::reset()
{
    m_gnss.reset();
    m_magnetic.reset();
    m_barometer.reset();
    m_context.reset();
    m_gnss_time = {};
    m_magnetic_time = {};
    m_barometer_time = {};
    m_seed_time_us = 0;
    m_latest_time_us = 0;
    m_fusion_time_us = 0;
    m_advanced_at_us = 0;
    m_average_period_us = profile::history::filter_interval_us;
}

} /* end namespace formal_eskf::px4 */
