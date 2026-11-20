// SPDX-License-Identifier: BSD-3-Clause

// Actual GNSS consumed-watermark and source-change bodies. Geographic motion
// construction and the quality monitor are typed call boundaries, not invented
// passing data. Invalid metadata, repeated samples and all decisions remain live.
#include "../contracts.hpp"
#include <array>
#include <formal_eskf/runtime/gnss_quality.hpp>
#include <formal_eskf/runtime/gnss_lifecycle.hpp>

extern bool nondet_bool();
extern double nondet_double();
extern std::uint64_t nondet_time();
extern std::uint32_t nondet_id();
extern unsigned nondet_value();
#define PX4_ERR(...) ((void)0)

namespace
{
#include "module_constants.inc"
using Vector = formal_eskf::linalg::Vector<contract_proof::Linalg, 3>;
using Decision = formal_eskf::runtime::GnssQualityDecision;
using QualityResult = formal_eskf::runtime::GnssQualityResult<double>;
using Status = formal_eskf::Status;
struct Raw
{
    struct Stamp
    {
        struct Source
        {
            std::uint32_t device_id{};
        } source;
    } stamp;
    double latitude_deg{}, longitude_deg{}, altitude_msl_m{}, altitude_ellipsoid_m{}, hdop{}, vdop{};
    bool velocity_valid{};
};
struct Observation
{
    std::uint64_t time_us{}, generation{};
    Raw sample;
};
struct QualitySample
{
    std::uint64_t time_us{};
};
unsigned const * expected_parameters;
struct Quality
{
    QualityResult response;
    unsigned resets{}, updates{};
    std::uint64_t expected_time{}, expected_horizon{};
    void reset() { ++resets; }
    QualityResult update(QualitySample sample, unsigned motion, std::uint64_t now, unsigned const & settings)
    {
        __ESBMC_assert(sample.time_us == expected_time, "Quality monitor receives the selected effective sample epoch");
        __ESBMC_assert(now == expected_horizon, "Quality monitor receives the delayed main horizon");
        __ESBMC_assert(motion == 1, "Quality monitor receives the selected motion producer result");
        __ESBMC_assert(&settings == expected_parameters, "Quality monitor borrows the configured parameter owner");
        ++updates;
        return response;
    }
};
struct Lifecycle
{
    unsigned resets{};
    void reset() { ++resets; }
};
struct Projection
{
    std::uint64_t time{};
    double latitude{}, longitude{};
    void initReference(double lat, double lon, std::uint64_t epoch)
    {
        time = epoch;
        latitude = lat;
        longitude = lon;
    }
};
struct Flow
{
    using status_type = Status;
    static Vector vector3(double x, double y, double z) { return Vector::from_row_major({x, y, z}); }
    struct Gnss
    {
        Quality quality;
        QualityResult quality_result;
        Lifecycle lifecycle;
        formal_eskf::runtime::GnssFusion::Sample sample;
        Observation input;
        Projection previous_fix;
        bool metadata_valid{}, position_rejected{}, velocity_rejected{};
        std::array<unsigned, 2> position_ratios{};
        std::array<unsigned, 3> velocity_ratios{};
        double position_test_ratio{}, previous_altitude{};
        Vector velocity_test_ratios;
        std::uint32_t device_id{}, corrections{};
        std::uint64_t generation{}, sample_time_us{};
        unsigned quality_parameters{};
        void change_source(std::uint32_t new_device_id, std::uint64_t new_generation);
    } m_gnss;
    std::uint64_t m_time_us{};
    unsigned m_rejections{}, m_errors{}, sample_calls{}, motion_calls{};
    bool m_fault{};
    bool check(Status status);
    bool recent(std::uint64_t sample, std::uint64_t max_age) const;
    QualitySample gnss_quality_sample()
    {
        __ESBMC_assert(m_gnss.sample.time_us == m_gnss.input.time_us && m_gnss.metadata_valid,
                       "Quality sample construction requires fresh selected data and valid geographic metadata");
        ++sample_calls;
        return {m_gnss.input.time_us};
    }
    unsigned gnss_motion()
    {
        ++motion_calls;
        return 1; // Origin label; all motion outcomes are in the monitor contract.
    }
    void update_gps_quality();
};
#include "module_status_check.inc"
#include "module_recent.inc"
#include "module_gnss.inc"
} // namespace

extern "C" void verify_module_gnss()
{
    Flow flow;
    auto & gnss = flow.m_gnss;
    gnss.input.time_us = nondet_time();
    gnss.input.generation = nondet_time();
    gnss.input.sample.stamp.source.device_id = nondet_id();
    auto & gps = gnss.input.sample;
    gps.latitude_deg = nondet_double();
    gps.longitude_deg = nondet_double();
    gps.altitude_msl_m = nondet_double();
    gps.altitude_ellipsoid_m = nondet_double();
    gps.hdop = nondet_double();
    gps.vdop = nondet_double();
    gps.velocity_valid = nondet_bool();
    gnss.sample_time_us = nondet_time();
    gnss.device_id = nondet_id();
    gnss.generation = nondet_time();
    gnss.metadata_valid = nondet_bool();
    gnss.corrections = nondet_id();
    gnss.previous_fix.time = nondet_time();
    gnss.sample.time_us = nondet_time();
    flow.m_time_us = nondet_time();
    flow.m_errors = nondet_value();
    flow.m_rejections = nondet_value();
    unsigned const decision = nondet_value();
    __ESBMC_assume(decision <= unsigned(Decision::accepted));
    gnss.quality.response.decision = static_cast<Decision>(decision);
    gnss.quality.response.status = nondet_bool() ? Status::success : Status::domain_error;
    gnss.quality.expected_time = gnss.input.time_us;
    gnss.quality.expected_horizon = flow.m_time_us;
    expected_parameters = &gnss.quality_parameters;
    auto const consumed = gnss.sample_time_us;
    auto const generation = gnss.generation;
    auto const source = gnss.device_id;
    auto const corrections = gnss.corrections;
    auto const rejections = flow.m_rejections;
    bool const new_event = gnss.input.time_us > consumed && gnss.input.time_us <= flow.m_time_us;
    bool const changed = new_event && (source != gps.stamp.source.device_id || generation != gnss.input.generation);
    bool const fresh =
        new_event && gps.velocity_valid && flow.m_time_us - gnss.input.time_us <= profile::gnss::maximum_sample_age_us;
    bool const metadata = std::isfinite(gps.latitude_deg) && gps.latitude_deg >= -90. && gps.latitude_deg <= 90. &&
                          std::isfinite(gps.longitude_deg) && gps.longitude_deg >= -180. && gps.longitude_deg <= 180. &&
                          std::isfinite(gps.altitude_msl_m) && std::isfinite(gps.altitude_ellipsoid_m) &&
                          std::isfinite(gps.hdop) && gps.hdop >= 0. && std::isfinite(gps.vdop) && gps.vdop >= 0.;
    bool const evaluated = fresh && metadata;
    flow.update_gps_quality();
    __ESBMC_assert(gnss.sample_time_us == (new_event ? gnss.input.time_us : consumed) &&
                       gnss.sample.time_us == (fresh ? gnss.input.time_us : 0),
                   "Consumed watermark advances once, including invalid payloads; no new event has zero fusion time");
    __ESBMC_assert(
        gnss.device_id == (changed ? gps.stamp.source.device_id : source) &&
            gnss.generation == (changed ? gnss.input.generation : generation) &&
            gnss.lifecycle.resets == unsigned(changed) && gnss.corrections == corrections,
        "Only a selected new device/generation resets qualification and lifecycle; correction count is retained");
    __ESBMC_assert(gnss.quality.updates == unsigned(evaluated) && flow.sample_calls == unsigned(evaluated) &&
                       flow.motion_calls == unsigned(evaluated) &&
                       gnss.quality.resets == unsigned(changed) + unsigned(fresh && !metadata),
                   "Invalid velocity, stale or invalid geographic input cannot qualify through the monitor");
    __ESBMC_assert(!fresh || gnss.sample.quality_available == metadata,
                   "Missing required metadata is not represented as a passing quality result");
    __ESBMC_assert(flow.m_rejections == rejections + unsigned(fresh && !metadata) +
                                            unsigned(evaluated && gnss.quality.response.decision == Decision::rejected),
                   "Rejected metadata and consumed rejected monitor results are counted once");
    auto const updates = gnss.quality.updates;
    auto const resets = gnss.quality.resets;
    auto const rejection_count = flow.m_rejections;
    auto const watermark = gnss.sample_time_us;
    flow.update_gps_quality();
    __ESBMC_assert(gnss.sample.time_us == 0 && gnss.sample_time_us == watermark && gnss.quality.updates == updates &&
                       gnss.quality.resets == resets && flow.m_rejections == rejection_count,
                   "A retained or future GNSS event cannot be consumed a second time at this horizon");
}
