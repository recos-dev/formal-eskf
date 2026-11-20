// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include "imu_owner_support.hpp"
#define private public
#include "../../../integrations/px4/module/ObservationHistory.hpp"
#undef private
namespace observation_proof
{
using Observations = formal_eskf::px4::ObservationHistory<imu_proof::Backend>;
using Timeline = Observations::Timeline;
using Stamp = Observations::SourceStamp;
using Limits = Observations::Limits;
using Admission = Observations::Admission;
namespace profile = formal_eskf::px4::profile;
static_assert(profile::history::filter_interval_us == 4000 && profile::history::minimum_period_us == 3800 &&
                  profile::history::maximum_period_us == 4200 && profile::history::maximum_lag_us == 300000 &&
                  profile::history::gnss_capacity == 16 && profile::history::magnetic_capacity == 64 &&
                  profile::history::barometer_capacity == 128 && profile::history::context_capacity == 128 &&
                  profile::history::gnss_arrival_age_us == 150000 &&
                  profile::history::magnetic_arrival_age_us == 100000 &&
                  profile::history::barometer_arrival_age_us == 100000 &&
                  profile::history::observation_maximum_age_us == 100000 &&
                  profile::history::gnss_spacing_us == 50000 && profile::history::magnetic_spacing_us == 10000 &&
                  profile::history::barometer_spacing_us == 5000 && profile::output::maximum_age_us == 100000 &&
                  profile::imu::maximum_interval_us == 50000 && profile::imu::interval_history_weight == 0.99 &&
                  profile::imu::interval_sample_weight == 0.01,
              "Observation contracts pin the deployment profile");

inline bool same_timeline(Timeline const & a, Timeline const & b)
{
    return a.source.device_id == b.source.device_id && a.source.calibration_count == b.source.calibration_count &&
           a.sample_time_us == b.sample_time_us && a.arrival_time_us == b.arrival_time_us &&
           a.admitted_time_us == b.admitted_time_us && a.selection_time_us == b.selection_time_us &&
           a.generation == b.generation;
}

inline bool same_land(Observations::LandState const & a, Observations::LandState const & b)
{
    return a.time_us == b.time_us && a.known == b.known && a.landed == b.landed && a.at_rest == b.at_rest &&
           a.in_ground_effect == b.in_ground_effect;
}
inline bool same_vehicle(Observations::VehicleState const & a, Observations::VehicleState const & b)
{
    return a.time_us == b.time_us && a.known == b.known && a.armed == b.armed && a.fixed_wing == b.fixed_wing;
}
inline bool same_context(Observations::Context const & a, Observations::Context const & b)
{
    return a.time_us == b.time_us && same_land(a.land, b.land) && same_vehicle(a.vehicle, b.vehicle);
}
inline bool same_double(double a, double b)
{
    return (__builtin_bit_cast(std::uint64_t, a) == __builtin_bit_cast(std::uint64_t, b)) |
           (std::isnan(a) & std::isnan(b));
}
inline bool same_interval(imu_proof::Interval const & a, imu_proof::Interval const & b)
{
    bool equal = a.time_us == b.time_us && a.start_time_us == b.start_time_us && a.hold_period_us == b.hold_period_us &&
                 imu_proof::same_input(a.sample, b.sample);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        equal = imu_proof::same(a.held_rate.specific_force_b(axis), b.held_rate.specific_force_b(axis)) &
                imu_proof::same(a.held_rate.angular_rate_b(axis), b.held_rate.angular_rate_b(axis)) & equal;
    }
    return equal;
}
} // namespace observation_proof
