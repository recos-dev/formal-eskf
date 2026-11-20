// SPDX-License-Identifier: BSD-3-Clause

#include "observation_support.hpp"

using observation_proof::Admission;
using observation_proof::Limits;
using observation_proof::Observations;
using observation_proof::same_context;
using observation_proof::same_double;
using observation_proof::same_interval;
using observation_proof::same_land;
using observation_proof::same_timeline;
using observation_proof::same_vehicle;
using observation_proof::Stamp;
namespace profile = observation_proof::profile;

#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
#define SELECTED_TIME m_gnss_time
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
#define SELECTED_TIME m_magnetic_time
#else
#define SELECTED_TIME m_barometer_time
#endif

void verify_observation_admission(Observations owner, Stamp stamp, std::uint64_t now, Limits limits,
                                  std::uint64_t effective)
{
    constexpr unsigned source = FORMAL_ESKF_PROOF_OBSERVATION_SOURCE;
    static_assert(FORMAL_ESKF_PROOF_OBSERVATION_SOURCE < 3);
    auto const timeline = owner.SELECTED_TIME;
    auto const old_gnss = owner.m_gnss_time;
    auto const old_magnetic = owner.m_magnetic_time;
    auto const old_barometer = owner.m_barometer_time;
    auto const old_stamp = stamp;
    auto const old_limits = limits;
    auto const old_configuration = owner.m_configuration;
    auto const old_configured = owner.m_configured;
    auto const old_fusion = owner.m_fusion_time_us;
    auto const old_advanced = owner.m_advanced_at_us;
    // Established by construction/reset and the actual begin_fusion recurrence.
    __ESBMC_assume(owner.m_average_period_us >= 3800. && owner.m_average_period_us <= 4200.);
    bool const generation_domain = timeline.generation <= timeline.sample_time_us;
    auto const previous_generation = timeline.generation;
    auto expected = timeline;
    auto expected_effective = effective;
    auto const seed = owner.m_seed_time_us;
    auto const latest = owner.m_latest_time_us;
    auto const mean = owner.m_average_period_us;
    auto const half = static_cast<std::uint32_t>(mean * 0.5);
    Admission decision = Admission::not_ready;
    bool changed = false;
    if (seed != 0U)
    {
        if (now < latest || now < timeline.arrival_time_us || stamp.time_us == 0U || stamp.time_us > now)
        {
            decision = Admission::invalid_time;
        }
        else
        {
            expected.arrival_time_us = now;
            if (stamp.time_us <= timeline.sample_time_us)
            {
                decision = Admission::out_of_order;
            }
            else
            {
                changed = stamp.source.device_id != timeline.source.device_id ||
                          stamp.source.calibration_count != timeline.source.calibration_count;
                expected.generation += static_cast<std::uint64_t>(changed);
                expected.source = stamp.source;
                expected.sample_time_us = stamp.time_us;
                if (stamp.source.device_id == 0U)
                {
                    decision = Admission::invalid_source;
                }
                else if (now - stamp.time_us >= limits.arrival_age_us)
                {
                    decision = Admission::stale;
                }
                else if (now - latest >= profile::output::maximum_age_us)
                {
                    decision = Admission::imu_stale;
                }
                else if (stamp.time_us <= limits.delay_us || stamp.time_us - limits.delay_us <= half)
                {
                    decision = Admission::offset_underflow;
                }
                else
                {
                    expected_effective = stamp.time_us - limits.delay_us - half;
                    if (expected_effective <= seed)
                    {
                        decision = Admission::before_seed;
                    }
                    else if (timeline.admitted_time_us != 0U &&
                             (expected_effective <= timeline.admitted_time_us ||
                              expected_effective - timeline.admitted_time_us < limits.spacing_us))
                    {
                        decision = Admission::too_fast;
                    }
                    else
                    {
                        decision = Admission::queued;
                        expected.admitted_time_us = expected_effective;
                    }
                }
            }
        }
    }
    auto const result = owner.admit(owner.SELECTED_TIME, stamp, now, limits, effective);
    __ESBMC_assert((source == 0 || same_timeline(owner.m_gnss_time, old_gnss)) &&
                       (source == 1 || same_timeline(owner.m_magnetic_time, old_magnetic)) &&
                       (source == 2 || same_timeline(owner.m_barometer_time, old_barometer)),
                   "Admission preserves unrelated timelines");
    __ESBMC_assert(imu_proof::same_configuration(owner.m_configuration, old_configuration) &&
                       owner.m_configured == old_configured && owner.m_fusion_time_us == old_fusion &&
                       owner.m_advanced_at_us == old_advanced,
                   "Admission preserves configuration and fusion clocks");
    __ESBMC_assert(stamp.time_us == old_stamp.time_us && stamp.source.device_id == old_stamp.source.device_id &&
                       stamp.source.calibration_count == old_stamp.source.calibration_count &&
                       limits.delay_us == old_limits.delay_us && limits.arrival_age_us == old_limits.arrival_age_us &&
                       limits.spacing_us == old_limits.spacing_us,
                   "Admission preserves all inputs");
    __ESBMC_assert(result.decision == decision && result.source_changed == changed && result.source_discarded == 0U &&
                       !result.overwritten && same_timeline(owner.SELECTED_TIME, expected) &&
                       effective == expected_effective,
                   "Observation admission: exact first rejection, watermark/generation protocol and effective epoch");
    __ESBMC_assert(owner.m_seed_time_us == seed && owner.m_latest_time_us == latest &&
                       owner.m_average_period_us == mean,
                   "Observation admission: admission does not advance IMU or fusion clocks");
    __ESBMC_assert(
        !generation_domain || (owner.SELECTED_TIME.generation >= previous_generation &&
                               owner.SELECTED_TIME.generation <= owner.SELECTED_TIME.sample_time_us &&
                               (!result.source_changed || owner.SELECTED_TIME.generation > previous_generation)),
        "Observation admission: generation cannot wrap from the seed invariant because every change advances raw time");
    if (result.decision == Admission::queued)
    {
        __ESBMC_assert(
            effective > seed && effective <= stamp.time_us && effective < now && stamp.source.device_id != 0U &&
                now - stamp.time_us < limits.arrival_age_us && now - latest < profile::output::maximum_age_us,
            "Observation admission: queued epochs are after seed, not future, and use fresh nonzero sources");
    }
}

void verify_source_identity(Observations::Source a, Observations::Source b)
{
    auto const old_a = a;
    auto const old_b = b;
    bool const expected = a.device_id == b.device_id && a.calibration_count == b.calibration_count;
    __ESBMC_assert(a.matches(b) == expected && a.matches(a), "Source identity compares both fields, including self");
    __ESBMC_assert(a.device_id == old_a.device_id && a.calibration_count == old_a.calibration_count &&
                       b.device_id == old_b.device_id && b.calibration_count == old_b.calibration_count,
                   "Source comparison preserves inputs");
}
void verify_seed_source(Stamp stamp, std::uint64_t epoch)
{
    auto const old = stamp;
    bool const expected = stamp.time_us <= epoch && (stamp.source.device_id == 0 || stamp.time_us != 0);
    __ESBMC_assert(Observations::valid_seed_source(stamp, epoch) == expected, "Seed source exact boundary");
    __ESBMC_assert(stamp.time_us == old.time_us && stamp.source.device_id == old.source.device_id &&
                       stamp.source.calibration_count == old.source.calibration_count,
                   "Seed source input frame");
}
void verify_snapshot(Observations::Context previous, std::uint64_t epoch, Observations::LandState land,
                     Observations::VehicleState vehicle)
{
    auto const old = previous;
    auto const old_land = land;
    auto const old_vehicle = vehicle;
    bool const replace_land = land.time_us > previous.land.time_us && land.time_us <= epoch;
    bool const replace_vehicle = vehicle.time_us > previous.vehicle.time_us && vehicle.time_us <= epoch;
    auto const result = Observations::snapshot(previous, epoch, land, vehicle);
    __ESBMC_assert(result.time_us == epoch, "Snapshot records the supplied IMU endpoint");
    if (replace_land)
    {
        __ESBMC_assert(same_land(result.land, old_land), "Snapshot copies new available land metadata");
    }
    else
    {
        __ESBMC_assert(same_land(result.land, old.land), "Snapshot retains old land metadata otherwise");
    }
    if (replace_vehicle)
    {
        __ESBMC_assert(same_vehicle(result.vehicle, old_vehicle), "Snapshot copies new available vehicle metadata");
    }
    else
    {
        __ESBMC_assert(same_vehicle(result.vehicle, old.vehicle), "Snapshot retains old vehicle metadata otherwise");
    }
    __ESBMC_assert(same_context(previous, old) && same_land(land, old_land) && same_vehicle(vehicle, old_vehicle),
                   "Snapshot preserves every input");
    if (old.land.time_us <= epoch && old.vehicle.time_us <= epoch)
    {
        __ESBMC_assert(result.land.time_us <= epoch && result.vehicle.time_us <= epoch,
                       "Snapshot cannot introduce future knowledge");
    }
}
void verify_period_range(Observations owner)
{
    __ESBMC_assume(owner.m_average_period_us >= 3800. && owner.m_average_period_us <= 4200.);
    auto const mean = owner.m_average_period_us;
    auto const half = owner.half_period_us();
    __ESBMC_assert(half == static_cast<std::uint32_t>(mean * 0.5) && half >= 1900 && half <= 2100,
                   "Half-period offset is finite, representable, and within the declared range");
}
void verify_observation_clock(Observations owner, imu_proof::Interval interval)
{
    __ESBMC_assume(owner.m_average_period_us >= 3800. && owner.m_average_period_us <= 4200.);
    auto const old_gnss = owner.m_gnss_time;
    auto const old_magnetic = owner.m_magnetic_time;
    auto const old_barometer = owner.m_barometer_time;
    auto const old_configuration = owner.m_configuration;
    auto const old_configured = owner.m_configured;
    auto const seed = owner.m_seed_time_us;
    auto const latest = owner.m_latest_time_us;
    auto const old_interval = interval;
    auto const mean = owner.m_average_period_us;
    auto const fusion = owner.m_fusion_time_us;
    auto const advanced = owner.m_advanced_at_us;
    bool const expected = owner.m_seed_time_us != 0 && owner.m_advanced_at_us != owner.m_latest_time_us &&
                          interval.start_time_us == owner.m_fusion_time_us &&
                          interval.time_us == interval.sample.time_us && interval.time_us > owner.m_fusion_time_us &&
                          interval.time_us <= owner.m_latest_time_us && interval.sample.angular_period_us >= 3800 &&
                          interval.sample.angular_period_us <= 4200 && interval.sample.acceleration_period_us >= 3800 &&
                          interval.sample.acceleration_period_us <= 4200 &&
                          owner.m_latest_time_us - interval.time_us >= owner.m_configuration.minimum_lag_us &&
                          owner.m_latest_time_us - interval.time_us <= 300000;
    __ESBMC_assert(owner.next_fusion(interval) == expected, "Next fusion exact eligibility");
    __ESBMC_assert(same_timeline(owner.m_gnss_time, old_gnss) && same_timeline(owner.m_magnetic_time, old_magnetic) &&
                       same_timeline(owner.m_barometer_time, old_barometer) &&
                       imu_proof::same_configuration(owner.m_configuration, old_configuration) &&
                       owner.m_configured == old_configured && owner.m_seed_time_us == seed &&
                       owner.m_latest_time_us == latest && owner.m_fusion_time_us == fusion &&
                       owner.m_advanced_at_us == advanced && same_double(owner.m_average_period_us, mean) &&
                       same_interval(interval, old_interval),
                   "Next fusion preserves complete inputs and owner metadata before begin");
    bool const result = owner.begin_fusion(interval);
    __ESBMC_assert(same_timeline(owner.m_gnss_time, old_gnss) && same_timeline(owner.m_magnetic_time, old_magnetic) &&
                       same_timeline(owner.m_barometer_time, old_barometer) &&
                       imu_proof::same_configuration(owner.m_configuration, old_configuration) &&
                       owner.m_configured == old_configured && owner.m_seed_time_us == seed &&
                       owner.m_latest_time_us == latest && same_interval(interval, old_interval),
                   "Fusion preserves complete inputs and other owner metadata");
    __ESBMC_assert(result == expected, "Begin fusion exact eligibility");
    if (result)
    {
        double const period =
            0.5 * (static_cast<double>(interval.sample.angular_period_us) + interval.sample.acceleration_period_us);
        __ESBMC_assert(owner.m_average_period_us == 0.99 * mean + 0.01 * period &&
                           owner.m_fusion_time_us == interval.time_us &&
                           owner.m_advanced_at_us == owner.m_latest_time_us,
                       "Begin fusion exact mean and clock publication");
        __ESBMC_assert(owner.m_average_period_us >= 3800. && owner.m_average_period_us <= 4200.,
                       "Real-period recurrence preserves its interval under binary64 rounding");
        __ESBMC_assert(!owner.next_fusion(interval), "One delayed step per real endpoint");
    }
    else
    {
        __ESBMC_assert(owner.m_average_period_us == mean && owner.m_fusion_time_us == fusion &&
                           owner.m_advanced_at_us == advanced,
                       "Rejected begin leaves timing unchanged");
    }
}

void verify_observation_selection(Observations owner)
{
    constexpr unsigned source = FORMAL_ESKF_PROOF_OBSERVATION_SOURCE;
    static_assert(source < 3);
    auto expected = owner.SELECTED_TIME;
    auto const old_gnss = owner.m_gnss_time;
    auto const old_magnetic = owner.m_magnetic_time;
    auto const old_barometer = owner.m_barometer_time;
    auto const configuration = owner.m_configuration;
    auto const configured = owner.m_configured;
    auto const seed = owner.m_seed_time_us;
    auto const latest = owner.m_latest_time_us;
    auto const fusion = owner.m_fusion_time_us;
    auto const advanced = owner.m_advanced_at_us;
    auto const average = owner.m_average_period_us;
    auto decision = Observations::Selection::not_ready;
    if (advanced != 0)
    {
        decision =
            expected.selection_time_us == fusion ? Observations::Selection::repeated : Observations::Selection::empty;
        expected.selection_time_us = fusion;
    }
    auto const result = owner.select(owner.SELECTED_TIME);
    __ESBMC_assert(result.decision == decision && result.retired == 0 && same_timeline(owner.SELECTED_TIME, expected),
                   "Selection exact eligibility, single-consumption watermark and result initialization");
    __ESBMC_assert((source == 0 || same_timeline(owner.m_gnss_time, old_gnss)) &&
                       (source == 1 || same_timeline(owner.m_magnetic_time, old_magnetic)) &&
                       (source == 2 || same_timeline(owner.m_barometer_time, old_barometer)),
                   "Selection preserves unrelated source timelines");
    __ESBMC_assert(imu_proof::same_configuration(owner.m_configuration, configuration) &&
                       owner.m_configured == configured && owner.m_seed_time_us == seed &&
                       owner.m_latest_time_us == latest && owner.m_fusion_time_us == fusion &&
                       owner.m_advanced_at_us == advanced && same_double(owner.m_average_period_us, average),
                   "Selection preserves all configuration and clock fields");
}

void verify_observation_configuration(Observations owner, formal_eskf::px4::TimingParameters parameters)
{
    auto const old_gnss = owner.m_gnss_time;
    auto const old_magnetic = owner.m_magnetic_time;
    auto const old_barometer = owner.m_barometer_time;
    auto const configuration = owner.m_configuration;
    auto const configured = owner.m_configured;
    auto const seed = owner.m_seed_time_us;
    auto const latest = owner.m_latest_time_us;
    auto const fusion = owner.m_fusion_time_us;
    auto const advanced = owner.m_advanced_at_us;
    auto const average = owner.m_average_period_us;
    auto const input = parameters;
    formal_eskf::px4::TimingConfiguration candidate;
    // Actual shared timing producer: accepted independently in delayed-input
    // and delayed-history. The owner must publish it only before seeding.
    bool const eligible = seed == 0 && formal_eskf::px4::configure_timing(parameters, candidate);
    bool const result = owner.configure(parameters);
    __ESBMC_assert(result == eligible, "Configuration exact pre-seed timing admission");
    if (result)
    {
        __ESBMC_assert(owner.m_configured && imu_proof::same_configuration(owner.m_configuration, candidate),
                       "Configuration publishes the entire validated candidate");
    }
    else
    {
        __ESBMC_assert(owner.m_configured == configured &&
                           imu_proof::same_configuration(owner.m_configuration, configuration),
                       "Configuration failure preserves the previous setting");
    }
    __ESBMC_assert(same_timeline(owner.m_gnss_time, old_gnss) && same_timeline(owner.m_magnetic_time, old_magnetic) &&
                       same_timeline(owner.m_barometer_time, old_barometer) && owner.m_seed_time_us == seed &&
                       owner.m_latest_time_us == latest && owner.m_fusion_time_us == fusion &&
                       owner.m_advanced_at_us == advanced && same_double(owner.m_average_period_us, average),
                   "Configuration preserves complete timelines and clocks");
    __ESBMC_assert(parameters.filter_interval_us == input.filter_interval_us &&
                       parameters.fusion_delay_ms == input.fusion_delay_ms &&
                       parameters.gnss_delay_ms == input.gnss_delay_ms &&
                       parameters.magnetic_delay_ms == input.magnetic_delay_ms &&
                       parameters.barometer_delay_ms == input.barometer_delay_ms,
                   "Configuration preserves every input field");
}

void verify_observation_views(Observations owner)
{
    __ESBMC_assume(owner.m_average_period_us >= 3800. && owner.m_average_period_us <= 4200.);
    auto const gnss = owner.m_gnss_time;
    auto const magnetic = owner.m_magnetic_time;
    auto const barometer = owner.m_barometer_time;
    auto const configuration = owner.m_configuration;
    auto const configured = owner.m_configured;
    auto const seed = owner.m_seed_time_us;
    auto const latest = owner.m_latest_time_us;
    auto const fusion = owner.m_fusion_time_us;
    auto const advanced = owner.m_advanced_at_us;
    auto const average = owner.m_average_period_us;
    __ESBMC_assert(&owner.gnss_source() == &owner.m_gnss_time.source &&
                       &owner.magnetic_source() == &owner.m_magnetic_time.source &&
                       &owner.barometer_source() == &owner.m_barometer_time.source,
                   "Source getters borrow the exact source subobject");
    __ESBMC_assert(owner.gnss_generation() == gnss.generation && owner.magnetic_generation() == magnetic.generation &&
                       owner.barometer_generation() == barometer.generation && owner.fusion_time_us() == fusion &&
                       owner.half_period_us() == static_cast<std::uint32_t>(average * 0.5),
                   "Observation getters return the correct generation and fusion metadata");
    __ESBMC_assert(same_timeline(owner.m_gnss_time, gnss) && same_timeline(owner.m_magnetic_time, magnetic) &&
                       same_timeline(owner.m_barometer_time, barometer) &&
                       imu_proof::same_configuration(owner.m_configuration, configuration) &&
                       owner.m_configured == configured && owner.m_seed_time_us == seed &&
                       owner.m_latest_time_us == latest && owner.m_fusion_time_us == fusion &&
                       owner.m_advanced_at_us == advanced && same_double(owner.m_average_period_us, average),
                   "Observation getters preserve all owner metadata");
}

void verify_double_frame_equality(double a, double b)
{
    bool const expected =
        (a == b && (a != 0. || std::signbit(a) == std::signbit(b))) || (std::isnan(a) && std::isnan(b));
    __ESBMC_assert(same_double(a, b) == expected && !same_double(0., -0.),
                   "Double frame equality preserves values, signed zero and NaN classification");
}
