// SPDX-License-Identifier: BSD-3-Clause

// Exact output_validity body, with the owner/callee results left independent.
// This covers temporal and source-generation eligibility, not sensor health
// accuracy or freshness after the eligibility check returns.
#include <cstdint>

extern bool nondet_bool();
extern std::uint64_t nondet_time();
extern std::uint32_t nondet_id();
extern void __ESBMC_assert(bool, char const *);

namespace
{
#include "module_constants.inc"
struct Output
{
    struct Sample
    {
        std::uint64_t time_us{};
    } sample;
    bool tracking{};
    Sample const & current() const { return sample; }
    bool ready() const { return tracking; }
};
struct Observations
{
    struct Source
    {
        std::uint32_t device_id{};
    } gnss, magnetic, barometer;
    std::uint64_t gnss_epoch{}, magnetic_epoch{}, barometer_epoch{};
    Source gnss_source() const { return gnss; }
    Source magnetic_source() const { return magnetic; }
    Source barometer_source() const { return barometer; }
    std::uint64_t gnss_generation() const { return gnss_epoch; }
    std::uint64_t magnetic_generation() const { return magnetic_epoch; }
    std::uint64_t barometer_generation() const { return barometer_epoch; }
};
struct Height
{
    enum class Source
    {
        barometer,
        gnss
    };
    struct Session
    {
        bool active{};
        std::uint64_t generation{};
    } barometer, gnss;
    std::uint64_t expected_epoch{};
    bool usable{};
    Session const & session(Source source) const
    {
        if (source == Source::barometer)
        {
            return barometer;
        }
        return gnss;
    }
    bool valid(std::uint64_t time) const
    {
        __ESBMC_assert(time == expected_epoch, "Height lifecycle validity is evaluated at delayed main time");
        return usable;
    }
};
struct Flow
{
    using height_type = Height;
    struct OutputValidity
    {
        bool healthy{}, gps_position{}, gps_velocity{}, gps{}, gps_height{};
        bool magnetic{}, magnetic_aiding{}, barometer{};
        bool height{}, vertical_velocity{}, dead_reckoning{};
    };
    bool m_initialized{}, m_fault{};
    std::uint64_t m_time_us{};
    Output m_output;
    Observations m_observations;
    Height m_height_aiding;
    struct Source
    {
        std::uint64_t generation{};
    } m_gnss, m_magnetic;
    bool position_valid{}, velocity_valid{}, position_recent{}, velocity_recent{}, heading_valid{}, heading_aiding{};
    bool gps_position_valid() const { return position_valid; }
    bool gps_velocity_valid() const { return velocity_valid; }
    bool gps_position_valid(std::uint64_t age) const
    {
        __ESBMC_assert(age == profile::output::dead_reckoning_age_us, "Position recency uses dead-reckoning age");
        return position_recent;
    }
    bool gps_velocity_valid(std::uint64_t age) const
    {
        __ESBMC_assert(age == profile::output::dead_reckoning_age_us, "Velocity recency uses dead-reckoning age");
        return velocity_recent;
    }
    bool magnetic_heading_valid() const { return heading_valid; }
    bool magnetic_heading_aiding() const { return heading_aiding; }
    OutputValidity output_validity(std::uint64_t now) const;
};

#include "module_validity.inc"
} // namespace

extern "C" void verify_module_validity()
{
    Flow flow;
    flow.m_initialized = nondet_bool();
    flow.m_fault = nondet_bool();
    flow.m_output.tracking = nondet_bool();
    flow.m_output.sample.time_us = nondet_time();
    flow.m_time_us = nondet_time();
    flow.m_height_aiding.expected_epoch = flow.m_time_us;
    flow.m_observations.gnss.device_id = nondet_id();
    flow.m_observations.magnetic.device_id = nondet_id();
    flow.m_observations.barometer.device_id = nondet_id();
    flow.m_observations.gnss_epoch = nondet_time();
    flow.m_observations.magnetic_epoch = nondet_time();
    flow.m_observations.barometer_epoch = nondet_time();
    flow.m_gnss.generation = nondet_time();
    flow.m_magnetic.generation = nondet_time();
    flow.m_height_aiding.barometer.generation = nondet_time();
    flow.m_height_aiding.gnss.generation = nondet_time();
    flow.m_height_aiding.barometer.active = nondet_bool();
    flow.m_height_aiding.gnss.active = nondet_bool();
    flow.m_height_aiding.usable = nondet_bool();
    flow.position_valid = nondet_bool();
    flow.velocity_valid = nondet_bool();
    flow.position_recent = nondet_bool();
    flow.velocity_recent = nondet_bool();
    flow.heading_valid = nondet_bool();
    flow.heading_aiding = nondet_bool();
    auto const now = nondet_time();
    auto const result = flow.output_validity(now);
    bool const healthy = flow.m_initialized && !flow.m_fault && flow.m_output.tracking &&
                         flow.m_output.sample.time_us <= now &&
                         now - flow.m_output.sample.time_us < profile::output::maximum_age_us;
    bool const gps =
        flow.m_observations.gnss.device_id != 0 && flow.m_observations.gnss_epoch == flow.m_gnss.generation;
    bool const magnetic =
        flow.m_observations.magnetic.device_id != 0 && flow.m_observations.magnetic_epoch == flow.m_magnetic.generation;
    bool const barometer = flow.m_observations.barometer.device_id != 0 && flow.m_height_aiding.barometer.active &&
                           flow.m_observations.barometer_epoch == flow.m_height_aiding.barometer.generation;
    bool const gps_height = gps && flow.m_height_aiding.gnss.active &&
                            flow.m_observations.gnss_epoch == flow.m_height_aiding.gnss.generation;
    __ESBMC_assert(result.healthy == healthy,
                   "Output health requires alignment, tracking, no fault and strict freshness");
    __ESBMC_assert(result.gps_position == (healthy && gps && flow.position_valid) &&
                       result.gps_velocity == (healthy && gps && flow.velocity_valid) &&
                       result.gps == (result.gps_position || result.gps_velocity),
                   "Navigation validity cannot cross a GNSS source generation");
    __ESBMC_assert(result.magnetic == (healthy && magnetic && flow.heading_valid) &&
                       result.magnetic_aiding == (healthy && magnetic && flow.heading_aiding),
                   "Magnetic validity and aiding use the current selected generation");
    __ESBMC_assert(result.gps_height == (healthy && gps_height) && result.barometer == (healthy && barometer) &&
                       result.height == (healthy && flow.m_height_aiding.usable && (barometer || gps_height)) &&
                       result.vertical_velocity == (result.gps_velocity || result.height),
                   "Height requires a usable owner and a current active source; it can support vertical velocity");
    __ESBMC_assert(result.dead_reckoning == !(result.gps && (flow.position_recent || flow.velocity_recent)),
                   "Dead reckoning reflects recent valid horizontal aiding");
}
