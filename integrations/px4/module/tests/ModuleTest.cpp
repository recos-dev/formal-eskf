// SPDX-License-Identifier: BSD-3-Clause
#include "../FormalEskf.hpp"

#include <array>
#include <atomic>
#include <thread>
#include <new>
#include <cerrno>
#include <bit>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <platforms/posix/apps.h>
#include <uORB/uORBManager.hpp>
#include <px4_platform_common/param.h>

// This standalone executable has no PX4 shell command registry.
void init_app_map(apps_map_type &) {}
void list_builtins(apps_map_type &) {}

namespace
{
std::uint64_t clock_us = 1000000U;
std::uint64_t watchdog_calls{};
std::uint32_t watchdog_delay{};
unsigned failures{};
unsigned scenarios{};
unsigned publications{};
FILE * trace_file{};
pthread_mutex_t * failed_init_mutex{};
pthread_mutex_t * failed_lock_mutex{};
pthread_mutex_t * failed_trylock_mutex{};
orb_id_t failed_publication{};
bool failed_allocation{}, failed_perf{};

void expect(bool condition, char const * description)
{
    if (!condition)
    {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", description);
    }
}

void trace_bytes(void const * data, std::size_t size)
{
    auto const * bytes = static_cast<unsigned char const *>(data);
    for (std::size_t i = 0; i < size; ++i)
    {
        std::fprintf(trace_file, "%02x", bytes[i]);
    }
    std::fputc('\n', trace_file);
}

template <typename Matrix> bool same_bits(Matrix const & a, Matrix const & b)
{
    using bytes_type = std::array<std::byte, sizeof(typename Matrix::value_type)>;
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            if (std::bit_cast<bytes_type>(a(row, column)) != std::bit_cast<bytes_type>(b(row, column)))
            {
                return false;
            }
        }
    }
    return true;
}

} // namespace

// Time/timer dispatch and explicitly injected resource failures are controlled. Numerical code, messages,
// uORB storage/subscriptions and the production Run() implementation are real.
// The work-queue manager is intentionally stopped: PX4 reports its unavailable
// queue during construction, and this fixture invokes each tick synchronously.
// Real scheduler startup and forced-timeout shutdown remain outside this fixture.
// Separate cases exercise the snapshot protocol and the normal module lifecycle lock.
extern "C" hrt_abstime __wrap_hrt_absolute_time() { return clock_us; }
extern "C" void __wrap_hrt_call_after(hrt_call *, hrt_abstime delay, hrt_callout, void *)
{
    ++watchdog_calls;
    watchdog_delay = static_cast<std::uint32_t>(delay);
}
extern "C" void __wrap_hrt_cancel(hrt_call *) {}
// Resource tests inject returned dependency errors; all normal calls use the
// actual PX4 implementation. Injection is disabled during concurrent tests.
extern "C" int __real_pthread_mutex_init(pthread_mutex_t *, pthread_mutexattr_t const *);
extern "C" int __wrap_pthread_mutex_init(pthread_mutex_t * mutex, pthread_mutexattr_t const * attr)
{
    return mutex == failed_init_mutex ? ENOMEM : __real_pthread_mutex_init(mutex, attr);
}
extern "C" int __real_pthread_mutex_lock(pthread_mutex_t *);
extern "C" int __wrap_pthread_mutex_lock(pthread_mutex_t * mutex)
{
    return mutex == failed_lock_mutex ? EINVAL : __real_pthread_mutex_lock(mutex);
}
extern "C" int __real_pthread_mutex_trylock(pthread_mutex_t *);
extern "C" int __wrap_pthread_mutex_trylock(pthread_mutex_t * mutex)
{
    return mutex == failed_trylock_mutex ? EINVAL : __real_pthread_mutex_trylock(mutex);
}
extern "C" int __real__ZN4uORB7Manager11orb_publishEPK12orb_metadataPvPKv(orb_id_t, orb_advert_t, void const *);
extern "C" int __wrap__ZN4uORB7Manager11orb_publishEPK12orb_metadataPvPKv(orb_id_t topic, orb_advert_t handle,
                                                                          void const * data)
{
    return topic == failed_publication
               ? PX4_ERROR
               : __real__ZN4uORB7Manager11orb_publishEPK12orb_metadataPvPKv(topic, handle, data);
}
extern "C" void * __real__ZnwmRKSt9nothrow_t(std::size_t, std::nothrow_t const &);
extern "C" void * __wrap__ZnwmRKSt9nothrow_t(std::size_t size, std::nothrow_t const & tag)
{
    return failed_allocation ? nullptr : __real__ZnwmRKSt9nothrow_t(size, tag);
}
extern "C" perf_counter_t __real_perf_alloc(perf_counter_type, char const *);
extern "C" perf_counter_t __wrap_perf_alloc(perf_counter_type type, char const * name)
{
    return failed_perf ? nullptr : __real_perf_alloc(type, name);
}

class FormalEskfModuleTest
{
public:
    FormalEskfModuleTest()
    {
        ++scenarios;
        clock_us += 100000000U;
        m_sample.timestamp_sample = clock_us;
        m_sample.delta_angle_dt = 4000U;
        m_sample.delta_velocity_dt = 4000U;
        m_sample.delta_velocity[2] = static_cast<float>(-9.80665 * 0.004);
        m_sample.accel_device_id = 101U;
        m_sample.gyro_device_id = 102U;
        expect(m_filter.init(), "module startup parameters and callback registration");
        for (std::size_t i = 0U; i < m_topics.size(); ++i)
        {
            m_outputs[i] = orb_subscribe(m_topics[i]);
            expect(m_outputs[i] >= 0, "subscribe to estimator output");
        }
        capture_outputs(false);
    }

    ~FormalEskfModuleTest()
    {
        for (int subscription : m_outputs)
        {
            orb_unsubscribe(subscription);
        }
    }

    void capture_outputs(bool record = true)
    {
        for (std::size_t i = 0U; i < m_topics.size(); ++i)
        {
            bool updated = false;
            expect(orb_check(m_outputs[i], &updated) == 0, "read output publication status");
            if (!updated)
            {
                continue;
            }
            std::array<unsigned char, 512U> message{};
            expect(m_topics[i]->o_size <= message.size(), "topic fits bounded capture storage");
            if (m_topics[i]->o_size > message.size())
            {
                std::abort();
            }
            expect(orb_copy(m_topics[i], m_outputs[i], message.data()) == 0, "copy actual uORB output");
            if (record)
            {
                ++publications;
                if (trace_file != nullptr)
                {
                    std::fprintf(trace_file, "%s ", m_topics[i]->o_name);
                    trace_bytes(message.data(), m_topics[i]->o_size_no_padding);
                }
            }
        }
    }

    void tick()
    {
        clock_us = m_sample.timestamp_sample;
        m_sample.timestamp = clock_us;
        sensor_gps_s gps{};
        gps.timestamp = clock_us;
        gps.timestamp_sample = clock_us;
        gps.device_id = m_gps_device;
        gps.fix_type = 3U;
        gps.satellites_used = 12U;
        gps.latitude_deg = 47.397742;
        gps.longitude_deg = 8.545594;
        gps.altitude_msl_m = 488.f;
        gps.altitude_ellipsoid_m = 535.f;
        gps.hdop = 0.7f;
        gps.vdop = 0.9f;
        gps.eph = 0.7f;
        gps.epv = 1.f;
        gps.s_variance_m_s = 0.2f;
        gps.vel_ned_valid = m_gps_velocity_valid;
        if (m_send_gps)
        {
            m_gps_pub.publish(gps);
        }
        vehicle_magnetometer_s mag{};
        mag.timestamp = clock_us;
        mag.timestamp_sample = clock_us;
        mag.device_id = m_mag_device;
        mag.calibration_count = m_mag_calibration;
        mag.magnetometer_ga[0] = 0.22f;
        mag.magnetometer_ga[1] = 0.01f;
        mag.magnetometer_ga[2] = 0.43f;
        m_mag_pub.publish(mag);
        vehicle_air_data_s baro{};
        baro.timestamp = clock_us;
        baro.timestamp_sample = clock_us;
        baro.baro_device_id = m_baro_device;
        baro.baro_alt_meter = m_baro_altitude;
        m_baro_pub.publish(baro);
        vehicle_land_detected_s land{};
        land.timestamp = clock_us;
        land.landed = m_landed;
        land.at_rest = m_at_rest;
        land.in_ground_effect = m_in_ground_effect;
        if (m_send_land)
        {
            m_land_pub.publish(land);
        }
        vehicle_status_s vehicle{};
        vehicle.timestamp = clock_us;
        vehicle.arming_state = m_armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
        vehicle.vehicle_type =
            m_fixed_wing ? vehicle_status_s::VEHICLE_TYPE_FIXED_WING : vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
        if (m_send_status)
        {
            m_status_pub.publish(vehicle);
        }
        m_imu_pub.publish(m_sample);
        auto const calls = watchdog_calls;
        m_filter.Run();
        capture_outputs();
        expect(watchdog_calls == calls + 1U && watchdog_delay == 20000U, "Run schedules the 20 ms watchdog");
    }

    void warmup()
    {
        for (unsigned i = 0; i < 3000U; ++i)
        {
            m_sample.timestamp_sample += 4000U;
            tick();
        }
        expect(m_filter.m_initialized && !m_filter.m_fault,
               "stationary real sensor sequence initializes the estimator");
        expect(m_filter.m_initialization.imu_samples == 1000U,
               "startup caps IMU accumulation without fabricating new samples");
        expect(m_filter.m_predictions > 0U, "post-startup ticks run prediction");
        expect(m_filter.m_gnss.corrections > 0U, "post-startup ticks run GNSS correction");
        snapshot("initialized");
    }

    void snapshot(char const * label)
    {
        if (trace_file == nullptr)
        {
            return;
        }
        std::fprintf(trace_file, "%s %llu %u %u %u %u %u %u %u %u %u %u\n", label,
                     static_cast<unsigned long long>(m_filter.m_time_us), m_filter.m_initialized, m_filter.m_fault,
                     m_filter.m_initialization.imu_samples, m_filter.m_initialization.magnetic_samples,
                     m_filter.m_predictions, m_filter.m_rejections, m_filter.m_errors, m_filter.m_gaps,
                     m_filter.m_position_reset_counter, m_filter.m_velocity_reset_counter);
        std::array<double, 16U + 225U> values{};
        auto const & state = m_filter.m_state;
        for (std::size_t i = 0; i < 3U; ++i)
        {
            values[i] = state.p_n(i);
            values[3U + i] = state.v_n(i);
            values[10U + i] = state.b_a(i);
            values[13U + i] = state.b_g(i);
        }
        for (std::size_t i = 0; i < 4U; ++i)
        {
            values[6U + i] = state.q_nb.coefficients()(i);
        }
        for (std::size_t row = 0; row < 15U; ++row)
        {
            for (std::size_t col = 0; col < 15U; ++col)
            {
                values[16U + 15U * row + col] = m_filter.m_covariance(row, col);
            }
        }
        trace_bytes(values.data(), sizeof(values));
        std::fprintf(trace_file, "gravity %u %u %u %u %llu\n", unsigned(m_filter.m_tilt_alignment.aligned()),
                     unsigned(m_filter.m_gravity.active), m_filter.m_gravity.samples, m_filter.m_gravity.corrections,
                     static_cast<unsigned long long>(m_filter.m_gravity.interval_us));
        auto const & magnetic = m_filter.m_magnetic.lifecycle.state();
        auto const & heading = m_filter.m_magnetic.heading.state();
        auto const & observation = m_filter.m_magnetic.observation;
        std::fprintf(trace_file, "magnetic %u %u %u %u %u %u %u %u %u %llu %llu %llu %llu %llu %llu\n",
                     unsigned(magnetic.active), unsigned(magnetic.action), unsigned(magnetic.stop_reason),
                     unsigned(magnetic.recovery_reason), unsigned(m_filter.m_heading_aligned),
                     unsigned(m_filter.m_quaternion_reset_counter), m_filter.m_odometry_reset_counter,
                     m_filter.m_magnetic.corrections, unsigned(observation.fusion.decision),
                     static_cast<unsigned long long>(m_filter.m_magnetic.sample_time_us),
                     static_cast<unsigned long long>(magnetic.start_time_us),
                     static_cast<unsigned long long>(magnetic.last_fuse_time_us),
                     static_cast<unsigned long long>(magnetic.last_reset_time_us),
                     static_cast<unsigned long long>(m_filter.m_magnetic_model.lookup_count()),
                     static_cast<unsigned long long>(m_filter.m_mag_model_origin_time));
        std::array<double, 8U> magnetic_values{heading.innovation, heading.filtered_innovation,
                                               heading.horizontal_acceleration,
                                               formal_eskf::linalg::max_abs(observation.fusion.test_ratio)};
        for (std::size_t i = 0U; i < 4U; ++i)
        {
            magnetic_values[4U + i] = m_filter.m_quaternion_reset_delta.coefficients()(i);
        }
        trace_bytes(magnetic_values.data(), sizeof(magnetic_values));
        auto const & gnss = m_filter.m_gnss.lifecycle.state();
        for (auto const & group : {gnss.velocity, gnss.horizontal_position})
        {
            std::fprintf(trace_file, "gnss %u %u %u %u %llu %llu %llu\n", unsigned(group.active),
                         unsigned(group.action), unsigned(group.recovery_required), unsigned(group.stop_reason),
                         static_cast<unsigned long long>(group.last_fuse_time_us),
                         static_cast<unsigned long long>(group.last_reset_time_us),
                         static_cast<unsigned long long>(group.start_time_us));
        }
        std::fprintf(trace_file, "navigation %u %u %u %u %u %llu %llu\n", m_filter.m_gnss.corrections,
                     m_filter.m_baro_updates, unsigned(m_filter.m_height_aiding.reference()),
                     unsigned(m_filter.m_height_reset_counter), unsigned(m_filter.m_vertical_velocity_reset_counter),
                     static_cast<unsigned long long>(m_filter.m_gnss.sample_time_us),
                     static_cast<unsigned long long>(m_filter.m_height_aiding.progress_time()));
        std::array<double, 9U> navigation_values{
            m_filter.m_gnss.position_test_ratio,
            m_filter.m_gnss.velocity_test_ratios(0),
            m_filter.m_gnss.velocity_test_ratios(1),
            m_filter.m_gnss.velocity_test_ratios(2),
            m_filter.m_height_aiding.session(FormalEskf::height_type::Source::barometer).fusion.test_ratio(0),
            m_filter.m_height_aiding.session(FormalEskf::height_type::Source::gnss).fusion.test_ratio(0),
            m_filter.m_height_reset_delta,
            m_filter.m_vertical_velocity_reset_delta,
            m_filter.m_gnss.previous_altitude};
        trace_bytes(navigation_values.data(), sizeof(navigation_values));
        for (auto source : {FormalEskf::height_type::Source::barometer, FormalEskf::height_type::Source::gnss})
        {
            auto const & aid = m_filter.m_height_aiding.session(source);
            std::fprintf(
                trace_file, "height %u %u %u %u %u %u %llu %llu %llu %llu\n", unsigned(aid.active), unsigned(aid.fault),
                unsigned(aid.fusion_reset), unsigned(aid.stop_reason), aid.device_id, unsigned(aid.calibration_count),
                static_cast<unsigned long long>(aid.sample_time), static_cast<unsigned long long>(aid.data_time),
                static_cast<unsigned long long>(aid.progress_time), static_cast<unsigned long long>(aid.fused_time));
            std::array<double, 9U> height_values{
                aid.bias.value,        aid.bias.variance,      aid.bias.maximum_variance,
                aid.bias.signed_ratio, aid.bias.positive_time, aid.bias.negative_time,
                aid.bias.dt,           aid.fusion_innovation,  aid.fusion.test_ratio(0)};
            trace_bytes(height_values.data(), sizeof(height_values));
        }
    }

    void select_navigation_inputs()
    {
        auto & f = m_filter;
        f.m_gnss.input = {f.m_gps.timestamp_sample, 0, 0, 0, FormalEskf::decode_gnss(f.m_gps)};
        f.m_gnss.context = f.startup_context(f.m_gnss.input.time_us, 3000000U);
        f.m_barometer = {f.m_baro.timestamp_sample, 0, 0, 0, FormalEskf::decode_barometer(f.m_baro)};
    }

    void prepare_navigation()
    {
        auto & f = m_filter;
        f.m_initialized = true;
        f.m_time_us = clock_us;
        f.m_covariance = FormalEskf::covariance_type::identity();
        f.m_origin.projection.initReference(47.397742, 8.545594, clock_us);
        f.m_origin.time_us = clock_us;
        f.m_origin.altitude_msl_m = 488.;
        f.m_origin.altitude_ellipsoid_m = 535.;
        f.m_gps.device_id = 201U;
        f.m_gps.latitude_deg = 47.397742;
        f.m_gps.longitude_deg = 8.545594;
        f.m_gps.altitude_msl_m = 488.;
        f.m_gps.altitude_ellipsoid_m = 535.;
        f.m_gps.fix_type = 3U;
        f.m_gps.satellites_used = 12U;
        f.m_gps.hdop = 0.7f;
        f.m_gps.vdop = 0.9f;
        f.m_gps.eph = 0.7f;
        f.m_gps.epv = 1.f;
        f.m_gps.s_variance_m_s = 0.2f;
        f.m_gps.vel_ned_valid = true;
        f.m_gnss.metadata_valid = true;
        f.m_gnss.quality_result.decision = formal_eskf::runtime::GnssQualityDecision::accepted;
        f.m_baro.baro_device_id = 401U;
        f.m_baro.baro_alt_meter = 538.f;
        expect(formal_eskf::succeeded(f.m_height_aiding.initialize(clock_us, 488.)), "initialize height datum");
    }

    void navigation_tick(bool gnss = true, bool barometer = true, std::uint64_t dt = 20000U)
    {
        auto & f = m_filter;
        clock_us += dt;
        f.m_time_us = clock_us;
        f.m_gnss.sample = {};
        if (gnss)
        {
            f.m_gps.timestamp_sample = clock_us;
            f.m_gnss.sample = {clock_us, f.m_gnss.quality_result.decision};
        }
        if (barometer)
        {
            f.m_baro.timestamp_sample = clock_us;
        }
        select_navigation_inputs();
        f.fuse_gps({});
        if (!f.m_fault)
        {
            select_navigation_inputs();
            f.fuse_height();
        }
        f.publish(FormalEskf::imu_type{});
        f.publish_status(clock_us);
        capture_outputs();
        snapshot("navigation");
    }

    void start_navigation()
    {
        prepare_navigation();
        for (unsigned i = 0U; i < 23U; ++i)
        {
            navigation_tick();
        }
        expect(!m_filter.m_fault && m_filter.m_height_aiding.session(FormalEskf::height_type::Source::barometer).active,
               "two-source navigation startup");
    }

    static void navigation_fusion(bool inhibit)
    {
        FormalEskfModuleTest test;
        test.prepare_navigation();
        test.navigation_tick();
        auto & f = test.m_filter;
        expect(f.m_position_reset_counter == 1U && f.m_velocity_reset_counter == 1U && f.m_height_reset_counter == 1U &&
                   f.m_gnss.corrections == 0U,
               "first qualified sample resets three groups without counting correction");
        f.m_covariance = FormalEskf::covariance_type::identity();
        f.m_covariance(3, 9) = f.m_covariance(9, 3) = 0.2;
        f.m_covariance(0, 12) = f.m_covariance(12, 0) = 0.2;
        f.m_gps.vel_n_m_s = 1.f;
        f.m_gps.latitude_deg += 0.000005;
        f.m_bias_learning_result.bias_update = {{!inhibit, !inhibit, !inhibit}, {!inhibit, !inhibit, !inhibit}};
        test.navigation_tick();
        expect(!f.m_fault && f.m_gnss.corrections == 3U && f.m_state.v_n(0) > 0.,
               "velocity then position then height each count a correction");
        expect((f.m_state.b_a(0) == 0.) == inhibit && (f.m_state.b_g(0) == 0.) == inhibit,
               "permissions reach velocity and horizontal gain rows");
        auto const position_progress = f.m_gnss.lifecycle.state().horizontal_position.last_fuse_time_us;
        f.m_gps.latitude_deg += 1.;
        f.m_gps.vel_n_m_s = 1.1f;
        test.navigation_tick();
        expect(!f.m_fault && f.m_gnss.lifecycle.state().velocity.last_fuse_time_us == clock_us &&
                   f.m_gnss.lifecycle.state().horizontal_position.last_fuse_time_us == position_progress,
               "position rejection does not undo successful velocity fusion");
    }

    enum class GnssInputCase
    {
        duplicate,
        future,
        stale,
        velocity_invalid,
        metadata_invalid,
        receiver,
        quality
    };

    static void navigation_admission(GnssInputCase scenario)
    {
        FormalEskfModuleTest test;
        test.prepare_navigation();
        auto & f = test.m_filter;
        f.m_land.timestamp = clock_us;
        f.m_land.landed = true;
        f.m_land.at_rest = true;
        for (unsigned i = 0U; i < 502U; ++i)
        {
            clock_us += 20000U;
            f.m_time_us = clock_us;
            f.m_gps.timestamp_sample = clock_us;
            f.m_land.timestamp = clock_us;
            test.select_navigation_inputs();
            f.update_gps_quality();
        }
        expect(f.gps_good(), "real monitor qualifies after ten seconds");
        auto const seen = f.m_gnss.sample_time_us;
        auto const previous = f.m_gnss.previous_fix.getProjectionReferenceTimestamp();
        clock_us += 600000U;
        f.m_time_us = clock_us;
        f.m_gps.timestamp_sample = clock_us;
        switch (scenario)
        {
        case GnssInputCase::duplicate:
            f.m_gps.timestamp_sample = seen;
            break;
        case GnssInputCase::future:
            f.m_gps.timestamp_sample = clock_us + 1U;
            break;
        case GnssInputCase::stale:
            f.m_gps.timestamp_sample = clock_us - 500001U;
            break;
        case GnssInputCase::velocity_invalid:
            f.m_gps.vel_ned_valid = false;
            break;
        case GnssInputCase::metadata_invalid:
            f.m_gps.latitude_deg = std::numeric_limits<double>::quiet_NaN();
            break;
        case GnssInputCase::receiver:
            ++f.m_gps.device_id;
            break;
        case GnssInputCase::quality:
            f.m_gps.satellites_used = 1U;
            break;
        }
        test.select_navigation_inputs();
        f.update_gps_quality();
        bool const ignored = scenario == GnssInputCase::duplicate || scenario == GnssInputCase::future ||
                             scenario == GnssInputCase::stale || scenario == GnssInputCase::velocity_invalid;
        expect((f.m_gnss.sample.time_us == 0U) == ignored && !f.m_fault, "admission precedes qualification");
        if (ignored)
        {
            expect(f.m_gnss.previous_fix.getProjectionReferenceTimestamp() == previous,
                   "ignored arrival does not advance drift reference");
        }
        if (scenario == GnssInputCase::metadata_invalid)
        {
            expect(!f.m_gnss.metadata_valid && !f.m_gnss.sample.quality_available && f.m_rejections == 1U,
                   "malformed coordinates revoke metadata, not a bad-fix substitute");
        }
        if (scenario == GnssInputCase::receiver || scenario == GnssInputCase::quality)
        {
            expect(!f.gps_good(), "changed receiver and failed checks cannot retain qualification");
        }
        if (scenario == GnssInputCase::receiver)
        {
            expect(f.m_gnss.position_ratios[0].time_us == 0 && f.m_gnss.velocity_ratios[0].time_us == 0 &&
                       std::isnan(f.m_gnss.velocity_ratios[0].filtered),
                   "receiver replacement clears innovation reporting history before requalification");
        }
        test.snapshot("GNSS-admission");
    }

    static void navigation_outage()
    {
        FormalEskfModuleTest test;
        test.start_navigation();
        auto & f = test.m_filter;
        auto const resets = f.m_height_reset_counter;
        test.navigation_tick(false, true, 999999U);
        expect(f.m_height_aiding.reference() == FormalEskf::height_type::Source::gnss, "height data grace");
        test.navigation_tick(false, true, 1U);
        expect(f.m_height_aiding.reference() == FormalEskf::height_type::Source::barometer &&
                   f.m_height_reset_counter == resets,
               "fallback switches reference without reset");
        test.navigation_tick(false, true, 6000001U);
        expect(!f.m_gnss.lifecycle.state().velocity.active && !f.gps_velocity_valid(), "seven-second GNSS stop");
        f.m_gps.altitude_msl_m += 5.;
        test.navigation_tick();
        expect(!f.m_fault && f.m_height_reset_counter == std::uint8_t(resets + 1U) &&
                   f.m_height_aiding.reference() == FormalEskf::height_type::Source::gnss,
               "returning qualified GNSS reclaims height with real reset");
    }

    static void navigation_failure(bool reset)
    {
        FormalEskfModuleTest test;
        test.prepare_navigation();
        if (!reset)
        {
            test.navigation_tick();
        }
        auto & f = test.m_filter;
        auto const velocity_resets = f.m_velocity_reset_counter;
        auto const position_resets = f.m_position_reset_counter;
        auto const corrections = f.m_gnss.corrections;
        // Fault injection after metadata qualification: exercise partial failure of
        // the second operation, not external-coordinate admission.
        f.m_gps.latitude_deg = std::numeric_limits<double>::quiet_NaN();
        f.m_gps.vel_n_m_s = 1.f;
        test.navigation_tick();
        expect(f.m_fault && f.m_errors == 1U && f.m_state.v_n(0) > 0.,
               "position failure preserves preceding velocity result and faults once");
        expect(f.m_position_reset_counter == position_resets &&
                   f.m_velocity_reset_counter == velocity_resets + unsigned(reset) &&
                   f.m_gnss.corrections == corrections + unsigned(!reset),
               "only the completed velocity operation receives success bookkeeping");
    }

    enum class HeightInputCase
    {
        duplicate,
        future,
        stale,
        invalid,
        calibration,
        unqualified
    };

    static void height_admission(HeightInputCase scenario)
    {
        FormalEskfModuleTest test;
        test.start_navigation();
        auto & f = test.m_filter;
        auto const progress = f.m_height_aiding.progress_time();
        auto const sample_time = f.m_height_aiding.session(FormalEskf::height_type::Source::barometer).sample_time;
        clock_us += scenario == HeightInputCase::stale ? 220000U : 20000U;
        f.m_time_us = clock_us;
        f.m_gnss.sample = {};
        f.m_baro.timestamp_sample = clock_us;
        switch (scenario)
        {
        case HeightInputCase::duplicate:
            f.m_baro.timestamp_sample = sample_time;
            break;
        case HeightInputCase::future:
            f.m_baro.timestamp_sample = clock_us + 1U;
            break;
        case HeightInputCase::stale:
            f.m_baro.timestamp_sample = clock_us - 200000U;
            break;
        case HeightInputCase::invalid:
            f.m_baro.baro_alt_meter = std::numeric_limits<float>::quiet_NaN();
            break;
        case HeightInputCase::calibration:
            ++f.m_baro.calibration_count;
            break;
        case HeightInputCase::unqualified:
            f.m_gnss.sample = {clock_us, formal_eskf::runtime::GnssQualityDecision::rejected};
            f.m_gnss.quality_result.decision = formal_eskf::runtime::GnssQualityDecision::rejected;
            f.m_gps.timestamp_sample = clock_us;
            break;
        }
        test.select_navigation_inputs();
        f.fuse_height();
        expect(!f.m_fault, "height admission rejection is not an estimator fault");
        if (scenario != HeightInputCase::unqualified)
        {
            expect(f.m_height_aiding.progress_time() == progress, "invalid/unready source cannot report progress");
        }
        else
        {
            expect(!f.m_height_aiding.session(FormalEskf::height_type::Source::gnss).active,
                   "unqualified GNSS cannot claim height fusion");
        }
        if (scenario == HeightInputCase::calibration)
        {
            expect(f.m_height_aiding.session(FormalEskf::height_type::Source::barometer).ratio.time_us == 0,
                   "barometer calibration clears diagnostic filtering with its bias history");
        }
        test.snapshot("height-admission");
    }

    static void height_transaction_failure()
    {
        FormalEskfModuleTest test;
        test.start_navigation();
        auto & f = test.m_filter;
        auto const prior = f.m_state;
        auto const covariance = f.m_covariance;
        auto const progress = f.m_height_aiding.progress_time();
        auto const bias = f.m_height_aiding.session(FormalEskf::height_type::Source::barometer).bias.value;
        auto const resets = f.m_height_reset_counter;
        clock_us += 20000U;
        f.m_time_us = clock_us;
        f.m_gnss.sample = {clock_us, formal_eskf::runtime::GnssQualityDecision::accepted};
        f.m_gps.timestamp_sample = clock_us;
        f.m_baro.timestamp_sample = clock_us;
        f.m_baro.baro_alt_meter += 1.f;
        // Invalid covariance fails the first numerical operation but must also roll
        // back the preceding tentative bias update and all source watermarks.
        f.m_covariance(0, 0) = std::numeric_limits<double>::quiet_NaN();
        test.select_navigation_inputs();
        f.fuse_height();
        expect(f.m_fault && f.m_errors == 1U && f.m_height_aiding.progress_time() == progress &&
                   f.m_height_reset_counter == resets &&
                   f.m_height_aiding.session(FormalEskf::height_type::Source::barometer).bias.value == bias &&
                   formal_eskf::linalg::max_abs(f.m_state.p_n - prior.p_n) == 0. &&
                   f.m_covariance(2, 2) == covariance(2, 2),
               "failed height transaction leaves estimator/history intact");
        test.snapshot("height-rollback");
    }

    static void navigation_rejected_quality()
    {
        FormalEskfModuleTest test;
        test.start_navigation();
        auto & f = test.m_filter;
        auto const progress = f.m_gnss.lifecycle.state().velocity.last_fuse_time_us;
        auto const corrections = f.m_gnss.corrections;
        f.m_gnss.quality_result.decision = formal_eskf::runtime::GnssQualityDecision::rejected;
        f.m_gps.vel_d_m_s = 1.f;
        test.navigation_tick();
        expect(!f.m_fault && f.m_gnss.vertical_innovation.active && f.m_gnss.vertical_innovation.innovation < 0. &&
                   formal_eskf::linalg::dot(f.m_gnss.velocity_test_ratios, f.m_gnss.velocity_test_ratios) > 0. &&
                   f.m_gnss.lifecycle.state().velocity.last_fuse_time_us == progress &&
                   f.m_gnss.corrections == corrections,
               "active rejected-quality source supplies health evidence without fusion");
        f.m_gnss.metadata_valid = false;
        test.navigation_tick();
        expect(!f.m_gnss.vertical_innovation.active, "invalid metadata clears velocity evidence");
    }

    static void navigation_timeout()
    {
        FormalEskfModuleTest test;
        test.start_navigation();
        auto & f = test.m_filter;
        auto const position_resets = f.m_position_reset_counter;
        auto const velocity_resets = f.m_velocity_reset_counter;
        f.m_gps.vel_n_m_s = 100.f;
        f.m_gps.latitude_deg += 0.01;
        for (unsigned i = 0U; i < 350U; ++i)
        {
            test.navigation_tick();
        }
        expect(f.m_gnss.position_rejected && f.m_gnss.velocity_rejected,
               "rejected production navigation records actual decisions");
        expect(f.m_position_reset_counter == position_resets && f.m_velocity_reset_counter == velocity_resets,
               "coupled timeout is strict at seven seconds");
        test.navigation_tick();
        expect(!f.m_fault && f.m_position_reset_counter == position_resets + 1U &&
                   f.m_velocity_reset_counter == velocity_resets + 1U,
               "both horizontal decisions survive velocity-first reset feedback");
        expect(!f.m_gnss.position_rejected && !f.m_gnss.velocity_rejected,
               "successful resets clear prior navigation rejection diagnostics");
        expect(f.m_gnss.position_ratios[0].filtered == 0. && f.m_gnss.velocity_ratios[2].filtered == 0. &&
                   f.m_gnss.velocity_ratios[2].time_us == f.m_gnss.sample.time_us,
               "navigation resets zero filtered innovation at the reset observation epoch");
    }

    static void height_combined_resets()
    {
        FormalEskfModuleTest test;
        test.start_navigation();
        auto & f = test.m_filter;
        f.m_gnss.quality_result.decision = formal_eskf::runtime::GnssQualityDecision::rejected;
        test.navigation_tick();
        f.m_baro.baro_alt_meter += 1000.f;
        for (unsigned i = 0U; i < 250U; ++i)
        {
            test.navigation_tick(false);
        }
        auto const position = f.m_state.p_n(2);
        auto const height_resets = f.m_height_reset_counter;
        auto const vertical_resets = f.m_vertical_velocity_reset_counter;
        f.m_gps.altitude_msl_m += 3.;
        f.m_gnss.quality_result.decision = formal_eskf::runtime::GnssQualityDecision::accepted;
        test.navigation_tick();
        expect(!f.m_fault && f.m_height_reset_counter == std::uint8_t(height_resets + 2U) &&
                   std::abs(f.m_height_reset_delta - (-3. - position)) < 1e-12 &&
                   f.m_vertical_velocity_reset_counter == std::uint8_t(vertical_resets + 1U),
               "pre-height Down reset and two height resets retain distinct counters and aggregate height delta");
    }

    static void navigation_context(bool fresh_land, bool armed)
    {
        FormalEskfModuleTest test;
        test.prepare_navigation();
        auto & f = test.m_filter;
        f.m_vehicle_status.timestamp = clock_us;
        f.m_vehicle_status.arming_state =
            armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
        f.m_land.timestamp = fresh_land ? clock_us : clock_us - 3000000U;
        f.m_land.landed = true;
        f.m_land.at_rest = true;
        f.m_gps.vel_n_m_s = 10.f;
        f.m_gps.timestamp_sample = clock_us;
        test.select_navigation_inputs();
        f.update_gps_quality();
        // Ground/rest checks reject velocity; fresh land overrides armed state.
        expect(!f.m_fault && f.m_gnss.quality_result.failures.horizontal_speed == fresh_land,
               "fresh land context overrides arming; exactly three-second-old land is unavailable");
        test.snapshot("GNSS-context");
    }

    static void output_mapping(bool stale, bool fault)
    {
        FormalEskfModuleTest test;
        test.warmup();
        auto & f = test.m_filter;
        auto const expected_position = FormalEskf::vector3(1.23456789, -2.34567891, -3.45678912);
        auto const expected_velocity = FormalEskf::vector3(4., 5., 6.);
        expect(f.reset_output({}, expected_velocity - f.m_output.current().v_n,
                              expected_position - f.m_output.current().p_n),
               "set distinct current output for mapping");
        f.m_state.p_n = FormalEskf::vector3(100., 200., 300.);
        f.m_state.v_n = FormalEskf::vector3(-4., -5., -6.);
        f.m_state.b_a = FormalEskf::vector3(0.01, 0.02, 0.03);
        f.m_state.b_g = FormalEskf::vector3(0.04, 0.05, 0.06);
        f.m_position_reset_delta = FormalEskf::vector2_type::from_row_major({0.1, 0.2});
        f.m_velocity_reset_delta = FormalEskf::vector3(0.3, 0.4, 0.5);
        f.m_height_reset_delta = -0.6;
        f.m_vertical_velocity_reset_delta = 0.7;
        f.m_position_reset_counter = 253U;
        f.m_height_reset_counter = 254U;
        f.m_velocity_reset_counter = 255U;
        f.m_vertical_velocity_reset_counter = 252U;
        f.m_odometry_reset_counter = 251U;
        f.m_fault = fault;
        f.m_last_output = f.m_output.current().time_us - 10000U;
        if (stale)
        {
            clock_us += 100000U;
        }
        FormalEskf::imu_type imu{FormalEskf::vector3(0.1, 0.2, -9.7), FormalEskf::vector3(0.4, 0.5, 0.6)};
        f.publish(imu);
        f.publish_status(clock_us);
        estimator_status_s status{};
        estimator_status_flags_s flags{};
        estimator_sensor_bias_s bias{};
        orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status);
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        orb_copy(ORB_ID(estimator_sensor_bias), test.m_outputs[6], &bias);
        expect(status.timestamp == clock_us && status.timestamp_sample == f.m_time_us &&
                   bias.timestamp_sample == f.m_time_us &&
                   bool(status.solution_status_flags & 1U) == (!stale && !fault) &&
                   bias.accel_bias_valid == (!stale && !fault),
               "status/bias keep the delayed epoch and revoke validity at wall-age/fault boundary");
        expect(status.reset_count_pod_d == 254U && status.reset_count_vel_d == 252U &&
                   flags.cs_inertial_dead_reckoning == (stale || fault),
               "status reset and validity mapping");
        bool updated{};
        orb_check(test.m_outputs[1], &updated);
        expect(updated == (!stale && !fault), "stale or faulted output cannot publish navigation");
        if (updated)
        {
            vehicle_local_position_s local{};
            vehicle_global_position_s global{};
            vehicle_odometry_s odometry{};
            orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local);
            orb_copy(ORB_ID(vehicle_global_position), test.m_outputs[2], &global);
            orb_copy(ORB_ID(vehicle_odometry), test.m_outputs[3], &odometry);
            expect(local.x == float(expected_position(0)) && local.vx == float(expected_velocity(0)) &&
                       local.delta_z == float(-0.6) && global.delta_alt == -local.delta_z &&
                       local.z_reset_counter == global.alt_reset_counter && odometry.reset_counter == 251U &&
                       odometry.q[0] == float(f.m_output.current().q_nb.coefficients()(0)),
                   "navigation reads current output, with reset signs/counters and published rounding");
            double latitude{}, longitude{};
            f.m_origin.projection.reproject(local.x, local.y, latitude, longitude);
            expect(global.lat == latitude && global.lon == longitude && local.xy_valid && local.z_valid &&
                       global.alt == float(f.m_origin.altitude_msl_m - f.m_output.current().p_n(2)),
                   "global mapping uses current local output rather than the delayed position");
            expect(local.timestamp_sample == f.m_output.current().time_us &&
                       global.timestamp_sample == local.timestamp_sample &&
                       odometry.timestamp_sample == local.timestamp_sample &&
                       local.timestamp_sample - status.timestamp_sample == 212000U &&
                       local.heading_var == float(f.rotation_variance_n()(2)),
                   "current navigation and delayed covariance proxy have explicitly different epochs");
        }
        test.capture_outputs();
        test.snapshot("output-mapping");
    }

    static void output_cadence()
    {
        FormalEskfModuleTest test;
        test.warmup();
        auto & f = test.m_filter;
        f.m_last_output = clock_us;
        f.m_last_status = clock_us;
        auto const last = f.m_last_output;
        for (unsigned tick = 1; tick <= 3; ++tick)
        {
            auto const count = publications;
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
            expect(publications == count + (tick == 3 ? 4U : 1U),
                   "attitude publishes every real interval; navigation only when ten milliseconds elapsed");
            expect(f.m_last_output == (tick == 3 ? clock_us : last),
                   "250 Hz input realizes the navigation limit at twelve milliseconds");
        }
        f.m_initialized = false;
        f.publish_status(clock_us);
        bool updated{};
        orb_check(test.m_outputs[6], &updated);
        expect(!updated, "uninitialized status does not publish bias estimates");
        test.capture_outputs();
        test.snapshot("output-cadence");
    }

    static void publication_preparation()
    {
        FormalEskfModuleTest test;
        vehicle_attitude_s attitude{};
        vehicle_local_position_s local{};
        vehicle_global_position_s global{};
        vehicle_odometry_s odometry{};
        estimator_status_s status{};
        estimator_status_flags_s flags{};
        estimator_sensor_bias_s bias{};
        expect(orb_copy(ORB_ID(vehicle_attitude), test.m_outputs[0], &attitude) == 0 &&
                   orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local) == 0 &&
                   orb_copy(ORB_ID(vehicle_global_position), test.m_outputs[2], &global) == 0 &&
                   orb_copy(ORB_ID(vehicle_odometry), test.m_outputs[3], &odometry) == 0 &&
                   orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status) == 0 &&
                   orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags) == 0 &&
                   orb_copy(ORB_ID(estimator_sensor_bias), test.m_outputs[6], &bias) == 0,
               "startup prepares actual storage for all seven topics");
        expect(attitude.timestamp == 0U && attitude.timestamp_sample == 0U && std::isnan(attitude.q[0]) &&
                   local.timestamp == 0U && !local.xy_valid && !local.z_valid && !local.v_xy_valid &&
                   !local.v_z_valid && global.timestamp == 0U && !global.lat_lon_valid && !global.alt_valid &&
                   odometry.timestamp == 0U && odometry.quality == 0 && std::isnan(odometry.q[0]) &&
                   status.timestamp == 0U && status.solution_status_flags == 0U && flags.timestamp == 0U &&
                   !flags.cs_tilt_align && !flags.cs_yaw_align && bias.timestamp == 0U && !bias.accel_bias_valid &&
                   !bias.gyro_bias_valid,
               "preparation marks no estimate, never a fabricated valid sample");
        test.warmup();
        auto & f = test.m_filter;
        f.m_last_output = clock_us - 10000U;
        auto const state = f.m_state;
        failed_publication = ORB_ID(vehicle_attitude);
        f.publish_attitude(clock_us);
        failed_publication = nullptr;
        expect(f.m_publication_failures == 1U && !f.m_fault && same_bits(state.p_n, f.m_state.p_n),
               "publication error is counted without pretending estimator failure or delivery success");
        f.update_diagnostics(clock_us);
        FormalEskf::Diagnostics view;
        expect(f.read_diagnostics(view) && view.publication_failures == 1U,
               "publication failure reaches shell snapshot");
    }

    static void resource_failures()
    {
        ++scenarios;
        {
            FormalEskf filter;
            failed_init_mutex = &filter.m_diagnostics_mutex;
            expect(!filter.init() && !filter.m_diagnostics_ready, "mutex setup failure prevents callback startup");
            failed_init_mutex = nullptr;
        }
        {
            failed_perf = true;
            FormalEskf filter;
            failed_perf = false;
            expect(!filter.init(), "perf allocation failure prevents callback startup");
        }
        for (orb_id_t topic : {ORB_ID(vehicle_attitude), ORB_ID(vehicle_local_position),
                               ORB_ID(vehicle_global_position), ORB_ID(vehicle_odometry), ORB_ID(estimator_status),
                               ORB_ID(estimator_status_flags), ORB_ID(estimator_sensor_bias)})
        {
            FormalEskf filter;
            failed_publication = topic;
            expect(!filter.init(), "each failed topic preparation prevents startup");
            failed_publication = nullptr;
        }
        failed_allocation = true;
        expect(FormalEskf::task_spawn(0, nullptr) == PX4_ERROR && FormalEskf::get_instance() == nullptr,
               "module allocation failure returns an error without constructing a session");
        failed_allocation = false;
    }

    static void diagnostic_contention()
    {
        FormalEskfModuleTest test;
        test.warmup();
        auto & f = test.m_filter;
        FormalEskf::Diagnostics before, after;
        expect(f.read_diagnostics(before), "read coherent pre-contention snapshot");
        auto const predictions = f.m_predictions;
        expect(pthread_mutex_lock(&f.m_diagnostics_mutex) == 0, "hold snapshot exchange");
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        expect(f.m_predictions == predictions + 1U && f.m_diagnostics.sequence == before.sequence,
               "busy diagnostics skip only snapshot, never block a real Run update");
        expect(pthread_mutex_unlock(&f.m_diagnostics_mutex) == 0, "release snapshot exchange");
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        expect(f.read_diagnostics(after) && after.sequence == before.sequence + 2U &&
                   after.skipped == before.skipped + 1U && after.sample_time_us == f.m_time_us,
               "later snapshot exposes missed sequence and resumes with current state");
        failed_lock_mutex = &f.m_diagnostics_mutex;
        auto const old_sequence = after.sequence;
        expect(!f.read_diagnostics(after) && after.sequence == old_sequence,
               "failed reader does not expose live state");
        failed_lock_mutex = nullptr;
        failed_trylock_mutex = &f.m_diagnostics_mutex;
        f.update_diagnostics(clock_us);
        failed_trylock_mutex = nullptr;
        f.update_diagnostics(clock_us);
        expect(f.read_diagnostics(after) && after.exchange_errors == 1U && !f.m_fault,
               "exchange error is observable and does not stop estimation");
    }

    static void diagnostic_concurrency()
    {
        FormalEskfModuleTest test;
        auto & f = test.m_filter;
        std::atomic<bool> ready{false}, done{false};
        std::atomic<unsigned> incoherent{0U}, reads{0U};
        std::thread reader(
            [&]
            {
                FormalEskf::Diagnostics view;
                while (!done.load())
                {
                    if (f.read_diagnostics(view))
                    {
                        ++reads;
                        if (view.position[0] != double(view.predictions) || view.velocity[0] != -view.position[0] ||
                            view.sample_time_us != view.predictions)
                        {
                            ++incoherent;
                        }
                    }
                    ready.store(true);
                }
            });
        while (!ready.load())
        {
            std::this_thread::yield();
        }
        for (unsigned i = 1U; i <= 20000U; ++i)
        {
            f.m_predictions = i;
            f.m_time_us = i;
            f.m_state.p_n(0) = i;
            f.m_state.v_n(0) = -double(i);
            f.update_diagnostics(i);
        }
        done.store(true);
        reader.join();
        expect(reads.load() > 0U && incoherent.load() == 0U,
               "concurrent reader never sees mixed diagnostic generations");
        f.update_diagnostics(clock_us);
        FormalEskf::Diagnostics final;
        expect(f.read_diagnostics(final) && final.predictions == 20000U,
               "final snapshot remains readable after contention");
    }

    static void diagnostic_shutdown()
    {
        ++scenarios;
        auto * instance = new FormalEskf();
        FormalEskf::_object.store(instance);
        FormalEskf::_task_id = FormalEskf::task_id_is_work_queue;
        expect(instance->init(), "initialize session for normal module shutdown");
        // ModuleBase::status_command holds this lifecycle guard through print_status.
        expect(pthread_mutex_lock(&px4_modules_mutex) == 0, "hold the real module reader lifetime guard");
        FormalEskf::Diagnostics view;
        expect(instance->read_diagnostics(view), "read snapshot before shutdown");
        instance->request_stop();
        std::atomic<bool> stop_requested{false};
        std::thread owner(
            [&]
            {
                stop_requested.store(true);
                instance->Run(); // unregisters, cancels and deletes under ModuleBase's lifecycle lock.
            });
        while (!stop_requested.load())
        {
            std::this_thread::yield();
        }
        expect(FormalEskf::get_instance() == instance, "reader lifecycle guard prevents concurrent destruction");
        expect(pthread_mutex_unlock(&px4_modules_mutex) == 0, "release module reader lifetime guard");
        owner.join();
        expect(FormalEskf::get_instance() == nullptr && FormalEskf::_task_id == -1,
               "normal shutdown releases session after diagnostic reader exits");
    }

    void set_horizontal_aiding(bool enabled)
    {
        // Establish the preceding GNSS phase's lifecycle state. Magnetic tests
        // exercise fuse_mag(), not GNSS qualification or its numerical update.
        using Gnss = formal_eskf::runtime::GnssFusion;
        m_filter.m_gnss.lifecycle.reset();
        if (enabled)
        {
            Gnss::Conditions conditions;
            conditions.horizontal_position = {true, true, true};
            expect(formal_eskf::succeeded(m_filter.m_gnss.lifecycle.update(
                       clock_us, conditions, formal_eskf::px4::profile::gnss::timing,
                       {clock_us, formal_eskf::runtime::GnssQualityDecision::accepted})),
                   "prepare horizontal aiding context");
            expect(formal_eskf::succeeded(m_filter.m_gnss.lifecycle.record_fusion(
                       Gnss::Group::horizontal_position, clock_us, formal_eskf::runtime::FusionDecision::fused)),
                   "prepare preceding fusion feedback");
        }
    }

    void prepare_magnetic(bool aided = true)
    {
        auto & filter = m_filter;
        filter.m_time_us = clock_us;
        filter.m_covariance = FormalEskf::covariance_type::identity() * 0.01;
        filter.m_initialized = true;
        filter.m_origin.projection.initReference(47.397742, 8.545594, clock_us);
        filter.m_origin.time_us = clock_us;
        filter.m_mag_model_origin_time = clock_us;
        filter.m_mag.device_id = 301U;
        filter.m_mag.magnetometer_ga[0] = 0.3f;
        filter.m_mag.magnetometer_ga[2] = 0.4f;
        auto const field = FormalEskf::vector3(100. * double(filter.m_mag.magnetometer_ga[0]), 0.,
                                               100. * double(filter.m_mag.magnetometer_ga[2]));
        expect(formal_eskf::succeeded(filter.m_magnetic_model.initialize(clock_us, {0U, 0.}, field, {})),
               "seed magnetic north without an independent health model");
        filter.m_magnetic.sample_time_us = clock_us;
        filter.initialize_magnetic_aiding({301U, 0U, 1U}, clock_us);
        filter.m_land.landed = true;
        filter.m_land.at_rest = true;
        set_horizontal_aiding(aided);
    }

    void magnetic_tick(bool new_sample = true, std::uint64_t interval = 10000U, bool fresh_context = true)
    {
        clock_us += interval;
        m_filter.m_time_us = clock_us;
        if (new_sample)
        {
            m_filter.m_mag.timestamp_sample = clock_us;
        }
        if (fresh_context)
        {
            m_filter.m_land.timestamp = clock_us;
        }
        m_filter.m_magnetic.input = {m_filter.m_mag.timestamp_sample, 0, 0, 0,
                                     FormalEskf::decode_magnetic(m_filter.m_mag)};
        m_filter.m_magnetic.context = m_filter.startup_context(m_filter.m_time_us, 3000000U);
        m_filter.fuse_mag();
        snapshot("magnetic-phase");
    }

    void start_magnetic()
    {
        for (unsigned i = 0U; i < 101U; ++i)
        {
            magnetic_tick();
            expect(m_filter.m_magnetic.corrections == 0U &&
                       std::max(m_filter.m_magnetic.lifecycle.state().last_fuse_time_us,
                                m_filter.m_magnetic.lifecycle.state().last_reset_time_us) == 0U,
                   "strict startup health does not invent fusion");
        }
        magnetic_tick();
        expect(!m_filter.m_fault && m_filter.m_magnetic.corrections == 1U && m_filter.magnetic_heading_aiding(),
               "first qualified sample executes real magnetic correction");
    }

    void magnetic_outputs()
    {
        if (!m_filter.m_fault)
        {
            m_filter.publish({});
        }
        m_filter.publish_status(clock_us);
        capture_outputs();
    }

    static void magnetic_outage(bool aided)
    {
        FormalEskfModuleTest test;
        test.prepare_magnetic(aided);
        test.start_magnetic();
        auto & filter = test.m_filter;
        auto const progress = std::max(filter.m_magnetic.lifecycle.state().last_fuse_time_us,
                                       filter.m_magnetic.lifecycle.state().last_reset_time_us);
        test.magnetic_tick(false, 1000000U);
        expect(!filter.m_magnetic.lifecycle.state().active && filter.m_heading_aligned == aided &&
                   std::max(filter.m_magnetic.lifecycle.state().last_fuse_time_us,
                            filter.m_magnetic.lifecycle.state().last_reset_time_us) == progress,
               "data timeout preserves progress and clears only unaided heading alignment");
        test.magnetic_outputs();
        test.magnetic_tick();
        expect(!filter.m_fault && filter.m_quaternion_reset_counter == (aided ? 0U : 1U) &&
                   filter.m_magnetic.corrections == (aided ? 2U : 1U),
               "returning data resumes aided fusion or performs one unaided heading reset");
        expect(filter.m_odometry_reset_counter == filter.m_quaternion_reset_counter,
               "heading reset reaches odometry bookkeeping exactly once");
        test.magnetic_outputs();
        test.magnetic_tick();
        expect(filter.m_magnetic.corrections == (aided ? 3U : 2U), "ordinary fusion follows reset on the next sample");
    }

    enum class MagneticInputCase
    {
        stale,
        future,
        duplicate,
        non_finite,
        zero_field,
        invalid_device
    };

    static void magnetic_input(MagneticInputCase scenario)
    {
        FormalEskfModuleTest test;
        test.prepare_magnetic();
        test.start_magnetic();
        auto & filter = test.m_filter;
        auto const before = filter.m_covariance;
        auto const seen = filter.m_magnetic.sample_time_us;
        auto const lookups = filter.m_magnetic_model.lookup_count();
        bool received = true;
        switch (scenario)
        {
        case MagneticInputCase::stale:
            clock_us += 600000U;
            filter.m_mag.timestamp_sample = clock_us - 500000U;
            received = false;
            break;
        case MagneticInputCase::future:
            filter.m_mag.timestamp_sample = clock_us + 20000U;
            received = false;
            break;
        case MagneticInputCase::duplicate:
            received = false;
            break;
        case MagneticInputCase::non_finite:
            filter.m_mag.magnetometer_ga[0] = std::numeric_limits<float>::quiet_NaN();
            break;
        case MagneticInputCase::zero_field:
            filter.m_mag.magnetometer_ga[0] = 0.f;
            filter.m_mag.magnetometer_ga[2] = 0.f;
            break;
        case MagneticInputCase::invalid_device:
            filter.m_mag.device_id = 0U;
            break;
        }
        test.magnetic_tick(received, scenario == MagneticInputCase::stale ? 0U : 10000U);
        expect(!filter.m_fault && filter.m_magnetic.corrections == 1U && same_bits(before, filter.m_covariance),
               "inadmissible magnetic sample does not alter state or latch a numerical fault");
        expect(filter.m_magnetic_model.lookup_count() == lookups,
               "inadmissible magnetic data cannot trigger geographic lookup");
        bool const consumed = scenario != MagneticInputCase::future && scenario != MagneticInputCase::duplicate;
        expect((filter.m_magnetic.sample_time_us > seen) == consumed && filter.m_rejections == (consumed ? 1U : 0U),
               "admission watermark and rejection count distinguish arrivals from pending/duplicate data");
        test.magnetic_outputs();
        if (scenario == MagneticInputCase::future)
        {
            test.magnetic_tick(false);
            expect(filter.m_magnetic.corrections == 2U, "future observation becomes eligible at its timestamp");
            test.magnetic_tick(false);
            expect(filter.m_magnetic.corrections == 2U, "eligible observation is not reused");
        }
        if (scenario == MagneticInputCase::stale)
        {
            test.magnetic_tick(false, 400000U);
            expect(!filter.m_magnetic.lifecycle.state().active, "late arrival does not postpone no-data timeout");
        }
    }

    enum class MagneticSourceCase
    {
        ground,
        aided_air,
        unaided_air,
        unknown,
        stale_land,
        land_overrides_arming
    };

    static void magnetic_source(MagneticSourceCase scenario)
    {
        FormalEskfModuleTest test;
        test.prepare_magnetic(scenario != MagneticSourceCase::unaided_air);
        test.start_magnetic();
        auto & filter = test.m_filter;
        ++filter.m_mag.calibration_count;
        filter.m_land.landed =
            scenario == MagneticSourceCase::ground || scenario == MagneticSourceCase::land_overrides_arming;
        bool const fresh_land = scenario != MagneticSourceCase::unknown && scenario != MagneticSourceCase::stale_land;
        if (!fresh_land)
        {
            filter.m_land.timestamp = clock_us - 3000000U;
        }
        if (scenario == MagneticSourceCase::stale_land || scenario == MagneticSourceCase::land_overrides_arming)
        {
            filter.m_vehicle_status.timestamp = clock_us;
            filter.m_vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
        }
        for (unsigned i = 0U; i < 4U; ++i)
        {
            test.magnetic_tick(true, 10000U, fresh_land);
        }
        bool const reset = scenario == MagneticSourceCase::ground || scenario == MagneticSourceCase::unaided_air ||
                           scenario == MagneticSourceCase::land_overrides_arming;
        bool const pending = scenario == MagneticSourceCase::unknown;
        expect(!filter.m_fault && filter.m_quaternion_reset_counter == (reset ? 1U : 0U) &&
                   filter.m_magnetic.corrections == ((!reset && !pending) ? 2U : 1U),
               "source recovery uses ground/air/unknown context with fresh land overriding arming");
        if (reset)
        {
            expect(filter.m_magnetic.ratios[0].filtered == 0. &&
                       filter.m_magnetic.ratios[0].time_us == filter.m_magnetic.lifecycle.state().last_reset_time_us,
                   "magnetic heading reset clears signed diagnostic history");
        }
        test.magnetic_outputs();
        if (pending)
        {
            test.magnetic_tick();
            expect(filter.m_magnetic.corrections == 2U, "known in-air context releases aided source recovery");
        }
    }

    static void magnetic_bias_and_field()
    {
        FormalEskfModuleTest test;
        test.prepare_magnetic();
        test.start_magnetic();
        auto & filter = test.m_filter;
        filter.m_covariance = FormalEskf::covariance_type::identity();
        filter.m_covariance(8U, 9U) = filter.m_covariance(9U, 8U) = 0.2;
        filter.m_covariance(8U, 12U) = filter.m_covariance(12U, 8U) = 0.2;
        filter.m_bias_learning_result.bias_update = {{false, false, false}, {true, true, true}};
        filter.m_mag.magnetometer_ga[1] = 0.01f;
        test.magnetic_tick();
        expect(filter.m_magnetic.corrections == 2U && filter.m_state.b_a(0U) == 0. && filter.m_state.b_g(0U) != 0. &&
                   filter.m_covariance(9U, 9U) == 1.,
               "magnetic correction forwards independent bias permissions");
        filter.m_state.b_a(0U) = -1.;
        filter.m_state.b_g(1U) = 1.;
        test.magnetic_tick();
        expect(filter.m_magnetic.corrections == 3U && filter.m_state.b_a(0U) == -0.4 && filter.m_state.b_g(1U) == 0.15,
               "successful correction applies deployment bias limits");
        filter.m_state.b_a(0U) = -1.;
        auto const before = filter.m_covariance;
        filter.m_mag.magnetometer_ga[2] = 1.f;
        test.magnetic_tick();
        expect(filter.m_magnetic.corrections == 3U && filter.m_state.b_a(0U) == -1. &&
                   same_bits(before, filter.m_covariance) && filter.m_magnetic.lifecycle.state().active &&
                   !filter.magnetic_heading_aiding(),
               "field rejection retains session and estimate without invoking bias limits");
        test.magnetic_outputs();
    }

    static void magnetic_timeout(bool disturbance)
    {
        FormalEskfModuleTest test;
        test.prepare_magnetic(false);
        test.start_magnetic();
        auto & filter = test.m_filter;
        filter.m_land.at_rest = false;
        filter.m_mag.magnetometer_ga[0] = 0.f;
        filter.m_mag.magnetometer_ga[1] = disturbance ? 3.f : 0.3f;
        filter.m_mag.magnetometer_ga[2] = disturbance ? 4.f : 0.4f;
        for (unsigned i = 0U; i < 701U; ++i)
        {
            test.magnetic_tick();
        }
        expect(!filter.m_fault && filter.m_quaternion_reset_counter == (disturbance ? 0U : 1U),
               "overdue fusion resets only while the field is qualified");
        if (disturbance)
        {
            expect(!filter.m_heading_aligned, "unqualified timeout clears unaided alignment");
            test.set_horizontal_aiding(true);
            filter.m_mag.magnetometer_ga[1] = 0.3f;
            filter.m_mag.magnetometer_ga[2] = 0.4f;
            for (unsigned i = 0U; i < 102U; ++i)
            {
                test.magnetic_tick();
            }
            expect(filter.m_quaternion_reset_counter == 1U,
                   "returning horizontal aiding does not bypass lost heading alignment");
        }
        expect(filter.m_heading_aligned && filter.m_odometry_reset_counter == 1U,
               "successful heading reset restores alignment and notifies odometry");
        test.magnetic_outputs();
    }

    static void magnetic_failure(bool reset)
    {
        FormalEskfModuleTest test;
        test.prepare_magnetic(false);
        test.start_magnetic();
        auto & filter = test.m_filter;
        filter.m_covariance(0U, 0U) = -1.;
        auto const before = filter.m_covariance;
        if (reset)
        {
            ++filter.m_mag.calibration_count;
            for (unsigned i = 0U; i < 3U; ++i)
            {
                test.magnetic_tick();
            }
        }
        test.magnetic_tick();
        using Recovery = formal_eskf::runtime::MagnetometerFusion::RecoveryReason;
        expect(filter.m_fault && filter.m_errors == 1U && filter.m_magnetic.corrections == 1U &&
                   filter.m_quaternion_reset_counter == 0U && same_bits(before, filter.m_covariance),
               "numerical failure preserves estimate and success counters, recording one fault");
        expect(filter.m_magnetic.lifecycle.state().recovery_reason ==
                   (reset ? Recovery::reset_failed : Recovery::fusion_failed),
               "actual numerical result reaches lifecycle repair latch");
        test.magnetic_outputs();
    }

    static void magnetic_heading_history()
    {
        FormalEskfModuleTest test;
        test.prepare_magnetic();
        test.start_magnetic();
        auto & filter = test.m_filter;
        filter.m_land.at_rest = false;
        filter.m_mag.magnetometer_ga[0] = 0.f;
        filter.m_mag.magnetometer_ga[1] = 0.3f;
        for (unsigned i = 0U; i < 30U; ++i)
        {
            test.magnetic_tick();
        }
        expect(!filter.m_magnetic.heading.state().consistent && !filter.magnetic_heading_aiding(),
               "filtered heading inconsistency inhibits aided navigation updates");
        auto const updates = filter.m_magnetic.corrections;
        filter.m_mag.magnetometer_ga[0] = 0.3f;
        filter.m_mag.magnetometer_ga[1] = 0.f;
        for (unsigned i = 0U; i < 100U; ++i)
        {
            test.magnetic_tick();
        }
        expect(filter.m_magnetic.corrections == updates && filter.m_quaternion_reset_counter == 0U,
               "small innovation without excitation cannot restore heading consistency");
        for (unsigned i = 0U; i < 200U; ++i)
        {
            expect(formal_eskf::succeeded(
                       filter.m_magnetic.heading.update_acceleration(FormalEskf::vector3(1., 0., 0.), 0.01)),
                   "feed prediction acceleration to maneuver history");
        }
        test.magnetic_tick();
        expect(filter.m_magnetic.corrections == updates + 1U && filter.m_magnetic.heading.state().consistent,
               "maneuver evidence permits consistency recovery");
        filter.m_magnetic.heading.reset_acceleration();
        expect(filter.m_magnetic.heading.state().horizontal_acceleration == 0. &&
                   filter.m_magnetic.heading.state().consistent,
               "gap reset clears acceleration without heading loss");
        test.snapshot("magnetic-history");
        test.magnetic_outputs();
    }

    static void magnetic_model_lookup()
    {
        FormalEskfModuleTest test;
        test.prepare_magnetic();
        test.start_magnetic();
        auto & filter = test.m_filter;
        auto const field = filter.m_magnetic_model.observation_n();
        auto const lookups = filter.m_magnetic_model.lookup_count();
        filter.m_origin.time_us = clock_us;
        filter.m_mag.magnetometer_ga[0] = std::numeric_limits<float>::quiet_NaN();
        test.magnetic_tick();
        expect(filter.m_magnetic_model.lookup_count() == lookups &&
                   filter.m_mag_model_origin_time != filter.m_origin.time_us,
               "invalid input cannot consume pending origin lookup");
        filter.m_mag.magnetometer_ga[0] = 0.3f;
        test.magnetic_tick();
        expect(!filter.m_fault && filter.m_magnetic_model.lookup_count() == lookups + 1U &&
                   filter.m_mag_model_origin_time == filter.m_origin.time_us && filter.m_magnetic_model.available() &&
                   same_bits(field, filter.m_magnetic_model.observation_n()),
               "eligible sample invokes real WMM once, retaining separately selected observation field");
        test.magnetic_outputs();
    }

    static void magnetic_reference_change(bool geographic)
    {
        FormalEskfModuleTest test;
        test.prepare_magnetic();
        auto & filter = test.m_filter;
        auto const field = filter.m_magnetic_model.observation_n();
        FormalEskf::magnetic_model_type::sample_type model{0., std::atan2(field(2U), field(0U)),
                                                           formal_eskf::linalg::norm(field)};
        expect(formal_eskf::succeeded(
                   filter.m_magnetic_model.initialize(clock_us, {geographic ? 3U : 2U, 0.}, field, model)),
               "seed independent geographic reference");
        test.start_magnetic();
        model.declination = 0.05;
        expect(filter.m_magnetic_model.update(clock_us + 10000U, model, true, true).changed,
               "adopt a changed reference before magnetic control");
        test.magnetic_tick();
        expect(!filter.m_fault && filter.m_quaternion_reset_counter == (geographic ? 1U : 0U) &&
                   filter.m_magnetic.corrections == (geographic ? 1U : 2U),
               "geographic revision resets resting heading; health-only revision retains correction reference");
        if (!geographic)
        {
            model.strength = 80.;
            expect(filter.m_magnetic_model.update(clock_us + 10000U, model, true, true).changed,
                   "adopt independent health strength");
            auto const before = filter.m_covariance;
            test.magnetic_tick();
            expect(filter.m_magnetic.corrections == 2U && filter.m_quaternion_reset_counter == 0U &&
                       filter.m_magnetic.observation.field.failures.strength && same_bits(before, filter.m_covariance),
                   "independent health model can reject a seed-matching observation without resetting yaw");
        }
        test.magnetic_outputs();
    }

    void prepare_gravity()
    {
        m_filter.m_time_us = m_sample.timestamp_sample;
        m_filter.m_covariance = FormalEskf::covariance_type::identity() * 0.0001;
        m_filter.m_land.timestamp = m_filter.m_time_us;
    }

    // Exercise the production gravity phase independently of earlier aiding.
    // Inputs satisfy IMU admission; invalid input periods are tested through Run().
    void gravity_sample(FormalEskf::imu_type const & imu, std::uint32_t angular_period = 5000U,
                        std::uint32_t acceleration_period = 5000U, std::uint8_t clipping = 0U)
    {
        FormalEskf::ImuInput input;
        input.angular_period_us = angular_period;
        input.acceleration_period_us = acceleration_period;
        input.accelerometer_clipping = clipping;
        input.time_us = m_filter.m_time_us;
        m_filter.m_imu_context = m_filter.startup_context(m_filter.m_time_us, 2000000U);
        m_filter.fuse_gravity(input, imu);
        snapshot("gravity-phase");
    }

    enum class GravityCase
    {
        eligible,
        disabled,
        aided,
        high_force,
        rest_override,
        stale_rest,
        clipped,
        unaligned
    };

    static void gravity_conditions(GravityCase scenario)
    {
        FormalEskfModuleTest test;
        test.prepare_gravity();
        auto & filter = test.m_filter;
        FormalEskf::imu_type imu{-filter.m_parameters.gravity_n, {}};
        std::uint8_t clipping = 0U;
        switch (scenario)
        {
        case GravityCase::eligible:
            expect(formal_eskf::succeeded(formal_eskf::so3::exp<FormalEskf::backend_type>(
                       FormalEskf::vector3(0.02, 0., 0.), 1e-6, filter.m_state.q_nb)),
                   "construct tilted prior for production gravity correction");
            break;
        case GravityCase::disabled:
            filter.m_gravity_enabled = false;
            break;
        case GravityCase::aided:
            test.warmup();
            filter.m_gravity.reset_samples();
            break;
        case GravityCase::high_force:
            imu.specific_force_b = imu.specific_force_b * 2.;
            break;
        case GravityCase::rest_override:
        case GravityCase::stale_rest:
            imu.specific_force_b = imu.specific_force_b * 2.;
            filter.m_land.at_rest = true;
            if (scenario == GravityCase::stale_rest)
            {
                filter.m_land.timestamp = filter.m_time_us - 2000001U;
            }
            break;
        case GravityCase::clipped:
            clipping = 1U;
            break;
        case GravityCase::unaligned:
            filter.m_covariance = FormalEskf::covariance_type::identity();
            break;
        }
        auto const before = filter.m_covariance;
        auto const before_q = filter.m_state.q_nb.coefficients();
        auto const count = filter.m_gravity.corrections;
        for (unsigned i = 0U; i < 4U; ++i)
        {
            test.gravity_sample(imu, 5000U, 5000U, clipping);
        }
        bool const eligible = scenario == GravityCase::eligible || scenario == GravityCase::rest_override;
        expect(!filter.m_fault && (filter.m_gravity.corrections > count) == eligible,
               "production gravity conditions admit only eligible samples");
        bool const active = eligible || scenario == GravityCase::clipped;
        expect(filter.m_gravity.active == active, "mode activity is distinct from clipped sample rejection");
        if (!eligible)
        {
            expect(same_bits(before, filter.m_covariance),
                   "ineligible gravity observations leave covariance unchanged");
        }
        if (scenario == GravityCase::eligible)
        {
            expect(filter.m_state.q_nb.q1() < before_q(1U), "gravity correction reduces the injected roll error");
        }
        expect(filter.m_tilt_alignment.aligned() == (scenario != GravityCase::unaligned),
               "alignment evaluation remains independent of gravity enablement and sample eligibility");
    }

    static void gravity_history()
    {
        FormalEskfModuleTest test;
        test.prepare_gravity();
        auto & filter = test.m_filter;
        FormalEskf::imu_type imu{-filter.m_parameters.gravity_n, {}};
        filter.m_gravity_enabled = false;
        auto high_force = imu;
        high_force.specific_force_b = high_force.specific_force_b * 2.;
        test.gravity_sample(high_force);
        test.gravity_sample(high_force);
        expect(filter.m_gravity.samples == 1U && filter.m_tilt_alignment.aligned(),
               "disabled gravity still samples, filters magnitude and evaluates alignment");
        filter.m_gravity_enabled = true;
        test.gravity_sample(imu);
        test.gravity_sample(imu);
        expect(filter.m_gravity.corrections == 0U && !filter.m_gravity.active,
               "re-enabling retains LPF history and rejects the first gravity-sized observation");
        filter.reset_imu_observations();
        expect(filter.m_tilt_alignment.aligned(), "sample-history reset preserves estimator alignment");
        test.gravity_sample(imu);
        test.gravity_sample(imu);
        expect(filter.m_gravity.corrections == 1U, "reset discards old LPF history and permits the new observation");
        auto const emitted = filter.m_gravity.samples;
        test.gravity_sample(imu);
        filter.reset_imu_observations();
        test.gravity_sample(imu);
        expect(filter.m_gravity.samples == emitted, "partial intervals cannot cross a history reset");
        filter.m_gravity_enabled = false;
        test.gravity_sample(imu);
        expect(!filter.m_gravity.active, "disabled mode clears previously active gravity");
    }

    static void gravity_periods_and_bias()
    {
        FormalEskfModuleTest a, b;
        a.prepare_gravity();
        b.prepare_gravity();
        FormalEskf::imu_type imu{-a.m_filter.m_parameters.gravity_n, FormalEskf::vector3(0.5, 0., 0.)};
        auto equivalent = imu;
        equivalent.angular_rate_b(0) = 0.4;
        for (unsigned i = 0U; i < 2U; ++i)
        {
            a.gravity_sample(imu, 4000U, 5000U);
            b.gravity_sample(equivalent);
        }
        expect(same_bits(a.m_filter.m_covariance, b.m_filter.m_covariance) &&
                   same_bits(a.m_filter.m_state.q_nb.coefficients(), b.m_filter.m_state.q_nb.coefficients()) &&
                   same_bits(a.m_filter.m_state.p_n, b.m_filter.m_state.p_n) &&
                   same_bits(a.m_filter.m_state.v_n, b.m_filter.m_state.v_n) &&
                   same_bits(a.m_filter.m_state.b_a, b.m_filter.m_state.b_a) &&
                   same_bits(a.m_filter.m_state.b_g, b.m_filter.m_state.b_g),
               "independent angle/velocity periods retain the same measured angle integral");
        auto & filter = a.m_filter;
        filter.m_state.b_a(0) = 1.;
        filter.m_state.b_g(1) = -1.;
        filter.m_bias_learning_result.bias_update = {{false, false, false}, {false, false, false}};
        auto const count = filter.m_gravity.corrections;
        a.gravity_sample(imu);
        a.gravity_sample(imu);
        expect(filter.m_gravity.corrections == count + 1U && filter.m_state.b_a(0) == 0.4 &&
                   filter.m_state.b_g(1) == -0.15,
               "gravity forwards bias inhibition and applies deployment limits after successful fusion");
    }

    enum class GravityFailure
    {
        tilt,
        sample,
        correction
    };

    static void gravity_failure(GravityFailure failure)
    {
        FormalEskfModuleTest test;
        test.prepare_gravity();
        auto & filter = test.m_filter;
        FormalEskf::imu_type imu{-filter.m_parameters.gravity_n, {}};
        if (failure == GravityFailure::tilt)
        {
            filter.m_covariance(6U, 6U) = std::numeric_limits<double>::quiet_NaN();
        }
        else if (failure == GravityFailure::sample)
        {
            imu.specific_force_b(0U) = std::numeric_limits<double>::quiet_NaN();
        }
        else
        {
            test.gravity_sample(imu);
            filter.m_parameters.gravity_n = {};
        }
        auto const before = filter.m_covariance;
        test.gravity_sample(imu);
        expect(filter.m_fault && filter.m_errors == 1U && filter.m_gravity.samples == 0U,
               "gravity phase failure is recorded once and does not publish a successful sample");
        expect(same_bits(before, filter.m_covariance),
               "failed alignment/correction preserves covariance, including NaN payloads");
        expect(filter.m_tilt_alignment.aligned() == (failure != GravityFailure::tilt) &&
                   filter.m_gravity.active == (failure == GravityFailure::correction),
               "failed correction preserves already evaluated alignment and mode activity");
        filter.publish_status(clock_us);
        test.capture_outputs();
        expect(filter.m_gravity.corrections == 0U, "failure cannot count as a correction");
    }

    static void gravity_in_run(bool enabled)
    {
        FormalEskfModuleTest test;
        test.m_filter.m_gravity_enabled = enabled;
        test.warmup();
        auto const count = test.m_filter.m_gravity.corrections;
        test.m_send_gps = false;
        for (unsigned i = 0U; i < 2000U; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
        }
        expect(!test.m_filter.m_fault && test.m_filter.m_tilt_alignment.aligned() &&
                   (test.m_filter.m_gravity.corrections > count) == enabled,
               "Run releases gravity after GNSS aiding stops, respecting startup enablement");
        test.snapshot("gravity-after-gnss-stop");
        test.m_sample.timestamp_sample += 4000U;
        test.m_sample.delta_velocity_clipping = 8U;
        auto const horizon = test.m_filter.m_time_us;
        test.tick();
        expect(test.m_filter.m_time_us == horizon && test.m_filter.m_tilt_alignment.aligned() && test.m_filter.m_fault,
               "invalid IMU metadata faults before gravity without advancing the delayed horizon");
        test.snapshot("gravity-invalid-metadata");
    }

    static void ground_effect_in_run()
    {
        FormalEskfModuleTest test;
        test.warmup();
        auto & f = test.m_filter;
        auto advance = [&](unsigned ticks)
        {
            for (unsigned i = 0; i < ticks; ++i)
            {
                test.m_sample.timestamp_sample += 4000U;
                test.tick();
            }
        };
        test.m_in_ground_effect = true;
        advance(54);
        expect(!f.m_ground_effect.active, "ground-effect request cannot activate while landed");
        test.m_landed = test.m_at_rest = false;
        test.m_armed = true;
        for (unsigned i = 1; i <= 54; ++i)
        {
            advance(1);
            expect(f.m_ground_effect.active == (i == 54),
                   "ground-effect request applies at the delayed flight-context epoch, never on arrival");
        }
        test.m_baro_altitude -= 3.f;
        advance(60);
        auto const & baro = f.m_height_aiding.session(FormalEskf::height_type::Source::barometer);
        expect(!f.m_fault && baro.fusion.decision == formal_eskf::runtime::FusionDecision::fused &&
                   baro.fusion.innovation(0) > 1. && baro.fusion.innovation(0) < 4. && baro.fusion_innovation == 0. &&
                   !baro.innovation_limited,
               "production Run applies the 4 m deadzone to a selected low barometric altitude");
        f.publish_status(clock_us);
        estimator_status_s status{};
        estimator_status_flags_s flags{};
        orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status);
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        expect(flags.cs_gnd_effect && (status.control_mode_flags & (1ULL << status.CS_GND_EFFECT)) != 0,
               "both PX4 status outputs report the active ground-effect protection");
        test.m_in_ground_effect = false;
        advance(54);
        auto const last_detection = f.m_ground_effect.last_detection_us;
        for (unsigned i = 0; i < 2500 && f.m_time_us - last_detection < 10000000U; ++i)
        {
            advance(1);
        }
        expect(f.m_ground_effect.active && f.m_time_us - last_detection == 10000000U,
               "negative requests do not clear the latch; exactly ten delayed seconds still held");
        advance(1);
        expect(!f.m_ground_effect.active, "ground-effect hold expires strictly after ten delayed seconds");
        test.m_in_ground_effect = true;
        advance(54);
        expect(f.m_ground_effect.active, "new positive request reactivates after expiry");
        test.m_fixed_wing = true;
        advance(54);
        expect(!f.m_ground_effect.active, "historical fixed-wing state disables protection");
        test.m_fixed_wing = false;
        test.m_in_ground_effect = false;
        advance(54);
        expect(!f.m_ground_effect.active, "cleared latch cannot reactivate from its old detection timestamp");
        test.m_in_ground_effect = true;
        advance(54);
        test.m_landed = true;
        advance(54);
        expect(!f.m_ground_effect.active && !f.m_fault, "landing clears protection at the delayed epoch");
    }

    static void ground_effect_missing_context()
    {
        FormalEskfModuleTest test;
        test.warmup();
        auto & f = test.m_filter;
        test.m_in_ground_effect = true;
        test.m_armed = true;
        test.m_landed = test.m_at_rest = false;
        for (unsigned i = 0; i < 54; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
        }
        expect(f.m_ground_effect.active, "setup: known rotary-wing flight activates protection");
        test.m_send_status = test.m_send_land = false;
        for (unsigned i = 0; i < 54; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
        }
        auto const last_detection = f.m_ground_effect.last_detection_us;
        for (unsigned i = 0; i < 10; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
        }
        expect(f.m_ground_effect.active && f.m_ground_effect.last_detection_us == last_detection,
               "repeated context snapshots cannot renew detection without a new source message");
        for (unsigned i = 0; i < 805; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
        }
        expect(!f.m_ground_effect.active && !f.m_fault,
               "stale vehicle status is unknown, not permission to keep ground-effect protection active");
    }

    static void delayed_pipeline()
    {
        FormalEskfModuleTest test;
        auto & f = test.m_filter;
        for (unsigned i = 0; i < 3000 && !f.m_initialized; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
        }
        expect(f.m_initialized && !f.m_fault && !f.m_output.ready(),
               "alignment seeds, but does not fabricate output history");
        auto const seed_time = f.m_time_us;
        for (unsigned i = 1; i <= 54; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
            expect(!f.m_fault && f.m_output.current().time_us == seed_time + i * 4000U,
                   "every real interval advances current output during warmup");
            expect(f.m_time_us == seed_time + (i == 54 ? 4000U : 0U) && f.m_predictions == (i == 54 ? 1U : 0U),
                   "exactly 54 real records admit the first delayed prediction");
            if (i < 54)
            {
                expect(!f.output_validity(clock_us).healthy, "warmup never grants valid current output");
                vehicle_local_position_s local{};
                orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local);
                expect(local.timestamp_sample == 0U, "warmup does not publish a current navigation estimate");
            }
        }
        expect(f.m_output.ready() && f.m_output.fusion_time_us() == f.m_time_us &&
                   f.m_imu_history.fusion_time_us() == f.m_time_us,
               "feedback acknowledges the exact main/output epoch");
        test.m_landed = test.m_at_rest = false;
        test.m_armed = true;
        for (unsigned i = 0; i < 54; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
            expect(!f.m_fault && f.m_output.current().time_us - f.m_time_us == 212000U,
                   "healthy steps maintain two horizons without skipped or duplicated prediction");
            expect(f.m_imu_context.land.known && f.m_imu_context.land.at_rest == (i < 53),
                   "future takeoff context never grants or removes historical rest permission");
            if (f.m_gnss.sample.time_us != 0)
            {
                expect(f.m_gnss.sample.time_us == f.m_gnss.input.sample.stamp.time_us - 2000U &&
                           f.m_height_aiding.session(FormalEskf::height_type::Source::gnss).sample_time ==
                               f.m_gnss.sample.time_us,
                       "one offset GNSS event is shared by velocity, horizontal position and height");
            }
        }
        auto const fusion_time = f.m_time_us;
        auto const output_time = f.m_output.current().time_us;
        auto const predictions = f.m_predictions;
        clock_us += 100000U;
        f.Run();
        expect(f.m_time_us == fusion_time && f.m_output.current().time_us == output_time &&
                   f.m_predictions == predictions && !f.output_validity(clock_us).healthy,
               "backup expires output without advancing either timeline");
    }

    static void startup()
    {
        FormalEskfModuleTest test;
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        expect(!test.m_filter.m_initialized && test.m_filter.m_initialization.imu_samples == 1U,
               "startup collects, does not initialize early");
        auto const mag_time = test.m_filter.m_initialization.magnetic_time_us;
        auto const source = test.m_filter.m_initialization.magnetic_source;
        test.m_armed = true;
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        expect(test.m_filter.m_initialization.imu_samples == 0U &&
                   test.m_filter.m_initialization.magnetic_samples == 0U,
               "armed startup clears both accumulated sample sets");
        expect(test.m_filter.m_initialization.magnetic_time_us == mag_time &&
                   test.m_filter.m_initialization.magnetic_source.device_id == source.device_id,
               "startup motion reset preserves magnetic watermark and source identity");
        test.m_armed = false;
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        test.m_mag_device = 302U;
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        expect(test.m_filter.m_initialization.imu_samples == 2U &&
                   test.m_filter.m_initialization.magnetic_samples == 1U,
               "magnetometer source change resets only the magnetic accumulation");
        test.m_sample.timestamp_sample += 4000U;
        test.m_sample.delta_velocity[0] = std::numeric_limits<float>::quiet_NaN();
        auto const previous = test.m_filter.m_imu_history.latest_time_us();
        test.tick();
        expect(!test.m_filter.m_fault && test.m_filter.m_time_us < test.m_sample.timestamp_sample &&
                   test.m_filter.m_imu_history.latest_time_us() == previous,
               "non-finite alignment input preserves the prior epoch and does not latch a fault");
        test.snapshot("startup");
    }

    static void sensor_position_parameters()
    {
        FormalEskfModuleTest test;
        char const * const names[] = {"FESKF_IMU_POS_X", "FESKF_IMU_POS_Y", "FESKF_IMU_POS_Z",
                                      "FESKF_GPS_POS_X", "FESKF_GPS_POS_Y", "FESKF_GPS_POS_Z"};
        for (auto name : names)
        {
            auto const handle = param_find(name);
            float saved{};
            expect(param_get(handle, &saved) == 0, "sensor installation parameter exists");
            for (float value : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
                                -std::numeric_limits<float>::infinity(), -10.01f, 10.01f})
            {
                param_set_no_notification(handle, &value);
                expect(!test.m_filter.init(), "invalid installation coordinate prevents startup");
            }
            for (float value : {-10.f, 10.f})
            {
                param_set_no_notification(handle, &value);
                expect(test.m_filter.init(), "installation bounds are inclusive");
            }
            param_set_no_notification(handle, &saved);
        }
        auto const imu = param_find("FESKF_IMU_POS_X");
        auto const gps = param_find("FESKF_GPS_POS_Z");
        float imu_saved{}, gps_saved{};
        param_get(imu, &imu_saved);
        param_get(gps, &gps_saved);
        float imu_value = 0.25f, gps_value = -0.5f;
        param_set_no_notification(imu, &imu_value);
        param_set_no_notification(gps, &gps_value);
        expect(test.m_filter.init() && test.m_filter.m_sensor_positions.imu_b(0) == 0.25 &&
                   test.m_filter.m_sensor_positions.gnss_b(2) == -0.5,
               "actual PX4 parameters bind metres in body FRD");
        test.warmup();
        float changed = 9.f;
        param_set_no_notification(imu, &changed);
        param_set_no_notification(gps, &changed);
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        expect(!test.m_filter.m_fault && test.m_filter.m_sensor_positions.imu_b(0) == 0.25 &&
                   test.m_filter.m_sensor_positions.gnss_b(2) == -0.5,
               "live parameter updates cannot silently move the sensor or output point");
        param_set_no_notification(imu, &imu_saved);
        param_set_no_notification(gps, &gps_saved);
    }

    static void rotating_sensor_geometry()
    {
        for (double yaw : {0., 0.7, -1.2})
        {
            FormalEskfModuleTest test;
            test.prepare_navigation();
            auto & f = test.m_filter;
            f.m_sensor_positions.imu_b = FormalEskf::vector3(0.2, -0.1, 0.3);
            f.m_sensor_positions.gnss_b = FormalEskf::vector3(1., 0.4, -0.2);
            f.m_state.b_g = FormalEskf::vector3(0.01, -0.02, 0.03);
            f.m_gps.s_variance_m_s = 0.6f; // Above the unchanged 0.3 m/s receiver-noise floor.
            matrix::Euler<double> const angles(0.3, -0.4, yaw);
            matrix::Quaternion<double> const q(angles);
            matrix::Dcm<double> const R(angles);
            expect(formal_eskf::succeeded(
                       FormalEskf::quaternion_type::from_coefficients(q(0), q(1), q(2), q(3), 1e-6, f.m_state.q_nb)),
                   "rotating-rig attitude");
            // Independent rig oracle: differentiate point positions, without a cross product.
            double const dt = 1e-6;
            matrix::Vector3<double> const rotation_rate_n(0.6, -0.7, 2.);
            matrix::Dcm<double> const plus(matrix::Quaternion<double>(matrix::AxisAngle<double>(rotation_rate_n * dt)) *
                                           q);
            matrix::Dcm<double> const minus(
                matrix::Quaternion<double>(matrix::AxisAngle<double>(rotation_rate_n * -dt)) * q);
            matrix::Vector3<double> const imu_b(0.2, -0.1, 0.3), gps_b(1., 0.4, -0.2);
            matrix::Vector3<double> const imu_velocity = (plus * imu_b - minus * imu_b) / (2. * dt);
            matrix::Vector3<double> const gps_velocity = (plus * gps_b - minus * gps_b) / (2. * dt);
            matrix::Vector3<double> const imu_position =
                R * (imu_b - gps_b); // Geographic origin is the first antenna fix.
            matrix::Vector3<double> const rate = R.transpose() * rotation_rate_n;
            expect(std::abs(gps_velocity(2) - imu_velocity(2)) > 0.01,
                   "rig exercises vertical rotational velocity as well as horizontal motion");
            FormalEskf::imu_type imu;
            imu.angular_rate_b = FormalEskf::vector3(rate(0), rate(1), rate(2)) + f.m_state.b_g;
            f.m_gps.vel_n_m_s = static_cast<float>(gps_velocity(0));
            f.m_gps.vel_e_m_s = static_cast<float>(gps_velocity(1));
            f.m_gps.vel_d_m_s = static_cast<float>(gps_velocity(2));
            f.initialize_navigation(imu);
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                expect(std::abs(f.m_state.p_n(axis) - imu_position(axis)) < 1e-12 &&
                           std::abs(f.m_state.v_n(axis) - imu_velocity(axis)) < 2e-7,
                       "startup position and velocity are at the IMU, with attitude and gyro bias respected");
            }
            f.m_state.p_n = {};
            f.m_state.v_n = {};
            clock_us += 20000U;
            f.m_time_us = clock_us;
            f.m_gps.timestamp_sample = clock_us;
            f.m_gnss.sample = {clock_us, formal_eskf::runtime::GnssQualityDecision::accepted};
            test.select_navigation_inputs();
            auto const raw = f.m_gnss.input.sample.velocity_n;
            auto const observation = f.gnss_velocity_observation(imu);
            auto const expected_variance = double(f.m_gps.s_variance_m_s) * double(f.m_gps.s_variance_m_s);
            expect(observation.variances(0) == expected_variance && same_bits(raw, f.m_gnss.input.sample.velocity_n),
                   "compensation preserves raw receiver data and its existing noise policy");
            f.fuse_gps(imu);
            f.fuse_height();
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                expect(!f.m_fault && std::abs(f.m_state.p_n(axis) - imu_position(axis)) < 1e-10 &&
                           std::abs(f.m_state.v_n(axis) - imu_velocity(axis)) < 2e-7,
                       "qualified GNSS horizontal/height/velocity resets consume compensated observations");
            }
            auto const corrections = f.m_gnss.corrections;
            clock_us += 20000U;
            f.m_time_us = clock_us;
            f.m_gps.timestamp_sample = clock_us;
            f.m_gnss.sample.time_us = clock_us;
            test.select_navigation_inputs();
            f.fuse_gps(imu);
            f.fuse_height();
            expect(!f.m_fault && f.m_gnss.corrections == corrections + 3U &&
                       formal_eskf::linalg::norm(f.m_gnss.velocity_test_ratios) < 1e-12 &&
                       f.m_gnss.position_test_ratio < 1e-12,
                   "normal velocity, horizontal and height corrections accept the compensated rotating rig");
            f.m_sensor_positions.gnss_b = f.m_sensor_positions.imu_b;
            expect(formal_eskf::linalg::squared_norm(f.gnss_position_offset()) == 0. &&
                       formal_eskf::linalg::squared_norm(f.gnss_velocity_offset(imu)) == 0.,
                   "coincident IMU and antenna need no GNSS compensation, even away from the body origin");
        }
    }

    static void body_reference_output()
    {
        FormalEskfModuleTest test;
        test.warmup();
        auto & f = test.m_filter;
        f.m_sensor_positions.imu_b = FormalEskf::vector3(0.3, -0.2, 0.1);
        f.m_sensor_positions.gnss_b = f.m_sensor_positions.imu_b;
        test.m_sample.delta_angle[0] = 0.2f * 0.004f;
        test.m_sample.delta_angle[1] = -0.3f * 0.004f;
        test.m_sample.delta_angle[2] = 0.4f * 0.004f;
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        auto const current = f.m_output.current();
        expect(current.time_us > f.m_time_us &&
                   formal_eskf::linalg::norm(current.q_nb.coefficients() - f.m_state.q_nb.coefficients()) > 1e-5,
               "body output oracle uses a genuinely different current attitude from the delayed filter");
        auto const & c = current.q_nb.coefficients();
        matrix::Quaternion<double> const q(c(0), c(1), c(2), c(3));
        matrix::Dcm<double> const R(q);
        matrix::Vector3<double> const imu_b(0.3, -0.2, 0.1);
        auto const & rate = f.m_output.angular_rate_b();
        double const dt = 1e-6;
        matrix::Vector3<double> const angle(rate(0) * dt, rate(1) * dt, rate(2) * dt);
        matrix::Dcm<double> const plus(q * matrix::Quaternion<double>(matrix::AxisAngle<double>(angle)));
        matrix::Dcm<double> const minus(q * matrix::Quaternion<double>(matrix::AxisAngle<double>(-angle)));
        matrix::Vector3<double> const position_offset = R * imu_b;
        matrix::Vector3<double> const velocity_offset = (plus * imu_b - minus * imu_b) / (2. * dt);
        f.m_last_output = current.time_us - 10000U;
        f.publish(FormalEskf::decode_imu(test.m_sample).mean_rates());
        vehicle_local_position_s local{};
        vehicle_global_position_s global{};
        vehicle_odometry_s odometry{};
        orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local);
        orb_copy(ORB_ID(vehicle_global_position), test.m_outputs[2], &global);
        orb_copy(ORB_ID(vehicle_odometry), test.m_outputs[3], &odometry);
        auto const body = f.body_output();
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            expect(std::abs(body.position_n(axis) - (current.p_n(axis) - position_offset(axis))) < 1e-12 &&
                       std::abs(body.velocity_n(axis) - (current.v_n(axis) - velocity_offset(axis))) < 1e-9 &&
                       odometry.position[axis] == float(body.position_n(axis)) &&
                       odometry.velocity[axis] == float(body.velocity_n(axis)),
                   "current body output agrees with an independent rotating-point derivative");
            expect(odometry.position_variance[axis] == float(f.m_covariance(axis, axis)),
                   "reported uncertainty retains the documented delayed IMU-state proxy");
        }
        expect(local.x == odometry.position[0] && local.y == odometry.position[1] && local.z == odometry.position[2] &&
                   local.vx == odometry.velocity[0] && local.vy == odometry.velocity[1] &&
                   local.vz == odometry.velocity[2] && local.z_deriv == local.vz &&
                   global.alt == float(f.m_origin.altitude_msl_m - body.position_n(2)) &&
                   global.alt_ellipsoid == float(f.m_origin.altitude_ellipsoid_m - body.position_n(2)) &&
                   same_bits(current.p_n, f.m_output.current().p_n) && same_bits(current.v_n, f.m_output.current().v_n),
               "local/global/odometry share one body point without moving the retained IMU state");
        FormalEskf::quaternion_type yaw_delta;
        expect(formal_eskf::succeeded(formal_eskf::so3::exp(FormalEskf::vector3(0., 0., 0.7), 1e-6, yaw_delta)),
               "heading reset setup");
        auto const positions = f.m_position_reset_counter;
        auto const velocities = f.m_velocity_reset_counter;
        auto const heights = f.m_height_reset_counter;
        auto const vertical = f.m_vertical_velocity_reset_counter;
        // A prior translation in the same delayed step must not be hidden by the heading reset.
        auto const translation = FormalEskf::vector2_type::from_row_major({0.1, -0.2});
        auto const velocity = FormalEskf::vector3(-0.3, 0.4, 0.5);
        f.record_position_reset(translation);
        f.record_velocity_reset(velocity);
        f.record_height_reset(0.6, 1);
        f.record_vertical_velocity_reset(velocity(2));
        expect(f.reset_output(yaw_delta, {}, {}), "attitude reset transforms output history");
        auto const after = f.body_output();
        auto const dp = after.position_n - body.position_n;
        auto const dv = after.velocity_n - body.velocity_n;
        expect(f.m_position_reset_counter == std::uint8_t(positions + 2U) &&
                   f.m_velocity_reset_counter == std::uint8_t(velocities + 2U) &&
                   f.m_height_reset_counter == std::uint8_t(heights + 2U) &&
                   f.m_vertical_velocity_reset_counter == std::uint8_t(vertical + 2U) &&
                   std::abs(f.m_position_reset_delta(0) - translation(0) - dp(0)) < 1e-12 &&
                   std::abs(f.m_velocity_reset_delta(1) - velocity(1) - dv(1)) < 1e-12 &&
                   std::abs(f.m_height_reset_delta - 0.6 - dp(2)) < 1e-12 &&
                   std::abs(f.m_vertical_velocity_reset_delta - velocity(2) - dv(2)) < 1e-12,
               "heading-induced body point changes accumulate with same-step translation reset notifications");
    }

    enum class CapabilityCase
    {
        all_aiding,
        height_only,
        horizontal_only,
        velocity_without_height,
        source_changed,
        no_aiding,
        no_tilt,
        stale,
        fault
    };

    static void confidence_capabilities(CapabilityCase scenario)
    {
        FormalEskfModuleTest test;
        test.warmup();
        auto & f = test.m_filter;
        bool horizontal = true, height = true, vertical_velocity = true, attitude = true;
        if (scenario == CapabilityCase::height_only || scenario == CapabilityCase::no_aiding)
        {
            f.m_gnss.lifecycle.reset();
            horizontal = false;
        }
        if (scenario == CapabilityCase::horizontal_only || scenario == CapabilityCase::velocity_without_height ||
            scenario == CapabilityCase::no_aiding)
        {
            f.m_height_aiding = {};
            height = false;
            vertical_velocity = scenario == CapabilityCase::velocity_without_height;
        }
        if (scenario == CapabilityCase::horizontal_only)
        {
            using Fusion = formal_eskf::runtime::GnssFusion;
            f.m_gnss.lifecycle.reset();
            Fusion::Conditions conditions;
            conditions.horizontal_position = {true, true, true};
            expect(formal_eskf::succeeded(
                       f.m_gnss.lifecycle.update(f.m_time_us, conditions, formal_eskf::px4::profile::gnss::timing,
                                                 {f.m_time_us, formal_eskf::runtime::GnssQualityDecision::accepted})),
                   "position-only lifecycle setup");
            expect(formal_eskf::succeeded(f.m_gnss.lifecycle.record_fusion(
                       Fusion::Group::horizontal_position, f.m_time_us, formal_eskf::runtime::FusionDecision::fused)),
                   "position-only successful progress");
        }
        if (scenario == CapabilityCase::source_changed)
        {
            ++f.m_gnss.generation; // Delayed source no longer matches the selected receiver generation.
            horizontal = false;
            height = f.m_height_aiding.session(FormalEskf::height_type::Source::barometer).active;
            vertical_velocity = height;
        }
        if (scenario == CapabilityCase::no_tilt)
        {
            f.m_tilt_alignment = {};
            attitude = false;
        }
        if (scenario == CapabilityCase::stale || scenario == CapabilityCase::fault)
        {
            f.m_fault = scenario == CapabilityCase::fault;
            clock_us += scenario == CapabilityCase::stale ? 100000U : 0U;
            horizontal = height = vertical_velocity = attitude = false;
        }
        f.publish_status(clock_us);
        estimator_status_s status{};
        estimator_status_flags_s flags{};
        orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status);
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        const unsigned capabilities = unsigned(attitude) | (unsigned(horizontal) << 1) |
                                      (unsigned(vertical_velocity) << 2) | (unsigned(horizontal) << 3) |
                                      (unsigned(horizontal) << 4) | (unsigned(height) << 5);
        expect((status.solution_status_flags & 0x3fU) == capabilities,
               "solution bits independently describe attitude, horizontal, height and vertical velocity");
        if (!attitude)
        {
            expect(status.pre_flt_fail_innov_heading,
                   "missing tilt alignment, stale output and estimator fault retain the Commander readiness veto");
        }
        expect(status.filter_fault_flags == 0 && !flags.fs_bad_acc_vertical && !flags.fs_bad_acc_clipping,
               "a generic fault or unavailable aiding is not an accelerometer fault");
        expect(flags.cs_inertial_dead_reckoning == !horizontal,
               "source changes and horizontal loss cannot report fresh horizontal aiding");
        if (scenario != CapabilityCase::stale && scenario != CapabilityCase::fault)
        {
            f.m_covariance(0, 0) = 9.;
            f.m_covariance(1, 1) = 16.;
            f.m_covariance(3, 3) = 36.;
            f.m_covariance(4, 4) = 64.;
            f.m_last_output = f.m_output.current().time_us - 10000U;
            f.publish({});
            vehicle_local_position_s local{};
            vehicle_odometry_s odometry{};
            vehicle_global_position_s global{};
            orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local);
            orb_copy(ORB_ID(vehicle_global_position), test.m_outputs[2], &global);
            orb_copy(ORB_ID(vehicle_odometry), test.m_outputs[3], &odometry);
            expect(global.alt_valid == height, "global altitude validity requires height progress, not velocity alone");
            expect(local.xy_valid == horizontal && local.v_xy_valid == horizontal &&
                       local.z_valid == vertical_velocity && local.v_z_valid == vertical_velocity &&
                       local.dead_reckoning == !horizontal,
                   "local validity agrees with the solution capabilities");
            expect(local.eph == 5.f && local.evh == 10.f && odometry.quality == 0,
                   "horizontal confidence uses both axes; odometry does not invent a percentage score");
        }
    }

    static void bias_confidence()
    {
        FormalEskfModuleTest test;
        test.warmup();
        auto & f = test.m_filter;
        struct Case
        {
            float variance[3];
            bool valid;
        };
        for (auto const & sample :
             {Case{{0.f, 0.f, 0.f}, false}, Case{{0.1f, 0.f, 0.f}, true},
              Case{{std::nextafter(0.1f, INFINITY), 0.f, 0.f}, false}, Case{{0.05f, 0.05f, 0.05f}, true},
              Case{{0.06f, 0.06f, 0.06f}, false}, Case{{-0.01f, 0.01f, 0.f}, false}, Case{{NAN, 0.f, 0.f}, false},
              Case{{INFINITY, 0.f, 0.f}, false}})
        {
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                f.m_covariance(9 + axis, 9 + axis) = sample.variance[axis];
                f.m_covariance(12 + axis, 12 + axis) = sample.variance[axis];
            }
            auto const before = f.m_covariance;
            f.m_bias_learning_result.bias_update = {{false, false, false}, {false, false, false}};
            f.publish_sensor_bias(clock_us, true);
            estimator_sensor_bias_s bias{};
            orb_copy(ORB_ID(estimator_sensor_bias), test.m_outputs[6], &bias);
            expect(
                bias.accel_bias_valid == sample.valid && bias.gyro_bias_valid == sample.valid &&
                    !bias.accel_bias_stable && !bias.gyro_bias_stable && !bias.mag_bias_valid &&
                    same_bits(before, f.m_covariance),
                "bias covariance threshold is independent of temporary learning inhibition and calibration stability");
            f.publish_sensor_bias(clock_us, false);
            orb_copy(ORB_ID(estimator_sensor_bias), test.m_outputs[6], &bias);
            expect(!bias.accel_bias_valid && !bias.gyro_bias_valid, "unhealthy output revokes both bias-valid flags");
        }
    }

    static void innovation_reporting()
    {
        FormalEskfModuleTest test;
        using formal_eskf::px4::InnovationRatio;
        InnovationRatio ratio;
        expect(std::isnan(ratio.filtered), "unobserved ratio is unavailable");
        ratio.update(1000000U, -1., 1.);
        expect(ratio.filtered == 1., "first sample seeds unsigned ratio as in PX4");
        ratio.update(1500000U, -1., 1.);
        expect(ratio.filtered == 0., "signed opposite innovation cancels at alpha=0.5");
        ratio.update(1500000U, -1., 1.);
        expect(ratio.filtered == 1., "repeated sample epoch reseeds rather than subtracting unsigned time");
        ratio.update(1400000U, -1., 2.);
        expect(ratio.filtered == 2., "regressed diagnostic epoch reseeds");
        ratio.reset(2000000U);
        ratio.update(2000001U, 1., 1.);
        expect(std::abs(ratio.filtered - 1. / 501.) < 1e-14, "minimum interval is one millisecond");
        ratio.reset(3000000U);
        ratio.update(13000000U, -1., 1.);
        expect(std::abs(ratio.filtered + 2. / 3.) < 1e-14, "maximum interval is one second");
        ratio.update(14000000U, 1., 100.);
        expect(ratio.filtered == 4., "large positive filtered outlier is limited");
        ratio.update(15000000U, -1., 100.);
        expect(ratio.filtered == -4., "large negative filtered outlier is limited");
        ratio.update(16000000U, 1., NAN);
        expect(std::isnan(ratio.filtered), "missing diagnostics cannot become a perfect residual");
        ratio.update(17000000U, 1., 100.);
        expect(ratio.filtered == 4., "recovered diagnostics are also limited");
        ratio.reset(18000000U);
        expect(ratio.filtered == 0. && ratio.time_us == 18000000U, "successful state reset clears reporting history");
    }

    static void heading_readiness(bool calibration_change)
    {
        FormalEskfModuleTest test;
        test.warmup();
        auto & f = test.m_filter;
        auto const resets = f.m_quaternion_reset_counter;
        if (calibration_change)
        {
            ++test.m_mag_calibration;
        }
        else
        {
            ++test.m_mag_device;
        }
        bool unavailable = false, alignment_lost = false, unsafe = false;
        bool horizontal_retained = true;
        estimator_status_s status{};
        for (unsigned i = 0; i < 500U; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
            auto const validity = f.output_validity(clock_us);
            f.publish_status(clock_us);
            orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status);
            if (!validity.magnetic)
            {
                unavailable = true;
                alignment_lost |= !f.m_heading_aligned;
                horizontal_retained &= validity.gps;
                unsafe |= !status.pre_flt_fail_innov_heading;
            }
        }
        expect(unavailable && alignment_lost && horizontal_retained && !f.m_fault,
               "real ground magnetic source/calibration replacement loses heading while GNSS remains valid");
        expect(!unsafe, "Commander heading veto must remain set throughout source replacement and realignment");
        expect(f.m_quaternion_reset_counter > resets && f.output_validity(clock_us).magnetic &&
                   !status.pre_flt_fail_innov_heading,
               "successful magnetic realignment restores preflight readiness without changing GNSS fusion");
    }

    static void diagnostic_flags()
    {
        FormalEskfModuleTest test;
        test.warmup();
        auto & f = test.m_filter;
        f.m_gnss.position_test_ratio = 0.25;
        f.m_gnss.velocity_test_ratios = FormalEskf::vector3(0.25, 0.25, 0.25);
        for (auto & ratio : f.m_gnss.position_ratios)
        {
            ratio = {};
            ratio.update(f.m_time_us, -1., 0.25);
        }
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            f.m_gnss.velocity_ratios[axis] = {};
            f.m_gnss.velocity_ratios[axis].update(f.m_time_us, -1., 0.25);
            f.m_magnetic.ratios[axis] = {};
            f.m_magnetic.ratios[axis].update(f.m_time_us, -1., 0.25);
        }
        f.publish_status(clock_us);
        estimator_status_s status{};
        estimator_status_flags_s flags{};
        orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status);
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        expect(!status.pre_flt_fail_innov_pos_horiz && !status.pre_flt_fail_innov_vel_horiz &&
                   !status.pre_flt_fail_innov_heading,
               "preflight threshold equality is accepted");
        expect(status.pos_test_ratio == 0.5f && status.vel_test_ratio == 0.5f && std::isnan(status.tas_test_ratio) &&
                   std::isnan(status.hagl_test_ratio),
               "summary ratio is amplitude-scaled; absent estimators do not report perfect zero residual");
        auto const initial = flags;
        auto const ratio_before = f.m_magnetic.ratios[0];
        f.publish_status(clock_us);
        expect(f.m_magnetic.ratios[0].filtered == ratio_before.filtered &&
                   f.m_magnetic.ratios[0].time_us == ratio_before.time_us,
               "publication does not advance innovation filtering");
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        expect(flags.control_status_changes == initial.control_status_changes &&
                   flags.fault_status_changes == initial.fault_status_changes &&
                   flags.innovation_fault_status_changes == initial.innovation_fault_status_changes,
               "repeated status publication is not a state change");
        f.m_gnss.position_test_ratio = 0.75;
        f.m_gnss.velocity_test_ratios = FormalEskf::vector3(0.75, 0.75, 0.01);
        for (auto & ratio : f.m_gnss.position_ratios)
        {
            ratio.update(f.m_time_us, -1., 0.75);
        }
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            f.m_gnss.velocity_ratios[axis].update(f.m_time_us, -1., axis == 2 ? 0.01 : 0.75);
            f.m_magnetic.ratios[axis].update(f.m_time_us, -1., 0.75);
        }
        f.publish_status(clock_us);
        orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status);
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        expect(status.pre_flt_fail_innov_pos_horiz && status.pre_flt_fail_innov_vel_horiz &&
                   status.pre_flt_fail_innov_heading && !status.pre_flt_fail_innov_vel_vert && !flags.reject_hor_pos &&
                   !flags.reject_hor_vel,
               "preflight warning at 0.5 is distinct from measurement rejection at 1");
        f.m_gnss.position_test_ratio = 2.;
        f.m_gnss.position_rejected = true;
        f.m_magnetic.observation.field.diagnostics_valid = true;
        f.m_magnetic.observation.field.failures.strength = true;
        f.m_gravity.active = true;
        f.publish_status(clock_us);
        orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status);
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        expect(flags.reject_hor_pos && (status.solution_status_flags & (1U << 10)) != 0 &&
                   flags.innovation_fault_status_changes == initial.innovation_fault_status_changes + 1,
               "GNSS rejection is diagnosed without being classified as numerical failure");
        expect(status.pre_flt_fail_mag_field_disturbed && (status.control_mode_flags & (1ULL << 16)) != 0 &&
                   (status.control_mode_flags & (1ULL << 34)) != 0 && flags.cs_gravity_vector &&
                   flags.control_status_changes == initial.control_status_changes + 1,
               "packed and named control flags share disturbance and gravity state");
        FormalEskf::vertical_health_type::Sample sample;
        f.m_vertical_health.reset();
        sample.time_us = f.m_time_us - 500000U;
        sample.average_interval_s = 0.004;
        sample.clipping_available = true;
        sample.clipped = {true, true, true};
        FormalEskf::vertical_health_type::Observations observations;
        observations.gnss_velocity_down = {true, 10., 1.};
        observations.barometer_height = {true, 10., 1.};
        for (unsigned i = 0; i < 125U; ++i)
        {
            sample.time_us += 4000U;
            expect(formal_eskf::succeeded(f.m_vertical_health.update(sample, observations)), "vertical fault setup");
        }
        f.publish_status(clock_us);
        orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status);
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        expect(flags.fs_bad_acc_vertical && flags.fs_bad_acc_clipping &&
                   status.filter_fault_flags == ((1U << 10) | (1U << 11)) &&
                   (status.solution_status_flags & (1U << 11)) != 0 &&
                   flags.fault_status_changes == initial.fault_status_changes + 1,
               "actual vertical/clipping faults use PX4's current bits, not obsolete bit 16");
        f.m_vertical_health.reset();
        f.publish_status(clock_us);
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        expect(!flags.fs_bad_acc_vertical && !flags.fs_bad_acc_clipping &&
                   flags.fault_status_changes == initial.fault_status_changes + 2,
               "fault recovery is also a status transition");
    }

    static void navigation_covariance_frame()
    {
        for (double roll : {0., 0.7, std::numbers::pi / 2.})
        {
            FormalEskfModuleTest test;
            test.warmup();
            auto & f = test.m_filter;
            matrix::Euler<double> const angles(roll, 0., 0.4);
            matrix::Dcm<double> const R(angles);
            matrix::Quaternion<double> const q(angles);
            expect(formal_eskf::succeeded(
                       FormalEskf::quaternion_type::from_coefficients(q(0), q(1), q(2), q(3), 1e-6, f.m_state.q_nb)),
                   "delayed covariance attitude");
            matrix::SquareMatrix<double, 3> P;
            P.setZero();
            P(0, 0) = 0.02;
            P(1, 1) = 0.25;
            P(2, 2) = 0.001;
            P(0, 1) = P(1, 0) = 0.001;
            P(1, 2) = P(2, 1) = 0.003;
            for (std::size_t row = 0; row < 3; ++row)
            {
                for (std::size_t column = 0; column < 3; ++column)
                {
                    f.m_covariance(6 + row, 6 + column) = P(row, column);
                }
            }
            auto const before = f.m_covariance;
            auto const expected = R * P * R.transpose();
            auto const variance = f.rotation_variance_n();
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                expect(std::abs(variance(axis) - expected(axis, axis)) < 1e-12,
                       "NED variance rotates the complete body covariance, including correlations");
            }
            expect(f.magnetic_heading_aiding(), "heading fixture has active aiding");
            expect(f.magnetic_heading_valid() == (expected(2, 2) < 0.1),
                   "heading validity uses navigation Down variance, not body Z variance");
            f.m_last_output = f.m_output.current().time_us - 10000U;
            f.publish(FormalEskf::decode_imu(test.m_sample).mean_rates());
            vehicle_local_position_s local{};
            vehicle_odometry_s odometry{};
            orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local);
            orb_copy(ORB_ID(vehicle_odometry), test.m_outputs[3], &odometry);
            expect(std::abs(double(local.heading_var) - expected(2, 2)) < 2e-8 &&
                       std::abs(double(local.tilt_var) - expected(0, 0) - expected(1, 1)) < 2e-8 &&
                       local.heading_good_for_control == (expected(2, 2) < 0.1),
                   "local heading/tilt and validity share the delayed navigation covariance");
            expect(odometry.orientation_variance[0] == float(P(0, 0)) &&
                       odometry.orientation_variance[1] == float(P(1, 1)) &&
                       odometry.orientation_variance[2] == float(P(2, 2)) && same_bits(before, f.m_covariance),
                   "odometry retains body rotation variances without mutating the core covariance");
        }
    }

    static void dateline_output()
    {
        for (double direction : {-1., 1.})
        {
            FormalEskfModuleTest test;
            test.warmup();
            auto & f = test.m_filter;
            f.m_origin.projection.initReference(0., direction * 179.999, f.m_origin.time_us);
            f.m_state.p_n = FormalEskf::vector3(0., direction * 2000., 0.);
            expect(f.m_output.apply_reset({}, {}, f.m_state.p_n), "dateline output setup");
            auto const state_before = f.m_state;
            auto const position_counter = f.m_position_reset_counter;
            double latitude{}, longitude{};
            f.m_origin.projection.reproject(f.m_state.p_n(0), f.m_state.p_n(1), latitude, longitude);
            expect(std::abs(longitude) > 180., "raw projection crosses the longitude convention boundary");
            double const expected_longitude = longitude - direction * 360.;
            auto const lookups = f.m_magnetic_model.lookup_count();
            auto const unavailable = f.m_mag_model_unavailable;
            f.m_mag_model_origin_time = 0; // Request a lookup at the current estimated position.
            f.m_origin.time_us = clock_us;
            expect(f.gps_position_valid(), "dateline fixture has qualified horizontal aiding");
            f.update_magnetic_model();
            expect(f.m_magnetic_model.lookup_count() == lookups + 1U && f.m_mag_model_unavailable == unavailable &&
                       !f.m_fault,
                   "crossing the dateline preserves a valid WMM lookup");
            f.m_last_output = f.m_output.current().time_us - 10000U;
            f.publish(FormalEskf::decode_imu(test.m_sample).mean_rates());
            vehicle_global_position_s global{};
            vehicle_local_position_s local{};
            orb_copy(ORB_ID(vehicle_global_position), test.m_outputs[2], &global);
            orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local);
            expect(global.lat_lon_valid && global.lon >= -180. && global.lon < 180. &&
                       std::abs(global.lon - expected_longitude) < 1e-7 && local.ref_lon == direction * 179.999 &&
                       double(local.y) * direction > 1900. && f.m_position_reset_counter == position_counter &&
                       same_bits(state_before.p_n, f.m_state.p_n),
                   "global longitude wraps without changing local origin, state or reset history");
        }
    }

    static void timing_parameters()
    {
        FormalEskfModuleTest test;
        struct Setting
        {
            char const * name;
            std::int32_t rejected;
        };
        for (auto const & setting :
             {Setting{"FESKF_DT_US", 10000}, Setting{"FESKF_DELAY_MS", 240}, Setting{"FESKF_GPS_DLY", -1},
              Setting{"FESKF_MAG_DLY", 101}, Setting{"FESKF_BARO_DLY", 101}})
        {
            auto const handle = param_find(setting.name);
            std::int32_t saved{};
            expect(param_get(handle, &saved) == 0, "timing parameter is present in the PX4 parameter table");
            param_set_no_notification(handle, &setting.rejected);
            expect(!test.m_filter.init(), "unsupported timing combination prevents callback startup");
            param_set_no_notification(handle, &saved);
            expect(test.m_filter.init(), "restoring timing parameters restores startup");
        }
        auto const handle = param_find("FESKF_DELAY_MS");
        std::int32_t maximum = 239;
        param_set_no_notification(handle, &maximum);
        expect(test.m_filter.init() && test.m_filter.m_imu_history.configuration().imu_length == 64,
               "largest supported millisecond delay uses all 64 fixed slots");
        std::int32_t normal = 200;
        param_set_no_notification(handle, &normal);
        expect(test.m_filter.init(), "restore the default history length before warmup");
        test.warmup();
        param_set_no_notification(handle, &maximum);
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        expect(!test.m_filter.m_fault && test.m_filter.m_imu_history.configuration().imu_length == 54 &&
                   test.m_filter.m_output.current().time_us - test.m_filter.m_time_us == 212000U,
               "running parameter changes cannot resize or retime histories");
        param_set_no_notification(handle, &normal);
    }

    static void delayed_heading_reset()
    {
        FormalEskfModuleTest test;
        test.warmup();
        auto & f = test.m_filter;
        auto const counter = f.m_quaternion_reset_counter;
        f.m_heading_aligned = false;
        for (unsigned i = 0; i < 20 && f.m_quaternion_reset_counter == counter; ++i)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
        }
        vehicle_attitude_s attitude{};
        orb_copy(ORB_ID(vehicle_attitude), test.m_outputs[0], &attitude);
        FormalEskf::quaternion_type before;
        expect(formal_eskf::succeeded(FormalEskf::quaternion_type::from_coefficients(
                   attitude.q[0], attitude.q[1], attitude.q[2], attitude.q[3], 1e-6, before)),
               "fast-phase attitude is a valid pre-reset output");
        auto const transformed = f.m_quaternion_reset_delta * before;
        expect(!f.m_fault && f.m_quaternion_reset_counter == std::uint8_t(counter + 1) &&
                   attitude.quat_reset_counter == counter &&
                   formal_eskf::linalg::norm(transformed.coefficients() - f.m_output.current().q_nb.coefficients()) <
                       1e-6,
               "qualified delayed heading reset applies the common rotation after the already-published fast attitude");
        test.m_sample.timestamp_sample += 4000;
        test.tick();
        orb_copy(ORB_ID(vehicle_attitude), test.m_outputs[0], &attitude);
        expect(!f.m_fault && attitude.quat_reset_counter == std::uint8_t(counter + 1) &&
                   f.m_quaternion_reset_counter == attitude.quat_reset_counter,
               "the next attitude carries the reset exactly once; ordinary feedback does not reset again");
    }

    static void delayed_source_changes()
    {
        FormalEskfModuleTest test;
        test.warmup();
        auto & f = test.m_filter;
        auto const gps_progress = f.m_gnss.lifecycle.state().velocity.last_fuse_time_us;
        auto const mag_corrections = f.m_magnetic.corrections;
        // Both switches arrive before either can reach the delayed horizon.
        ++test.m_gps_device;
        ++test.m_mag_device;
        ++test.m_baro_device;
        test.m_sample.timestamp_sample += 4000;
        test.tick();
        --test.m_gps_device;
        --test.m_mag_device;
        --test.m_baro_device;
        test.m_sample.timestamp_sample += 4000;
        test.tick();
        test.m_gps_velocity_valid = false;
        auto const validity = f.output_validity(clock_us);
        expect(!f.m_fault && !validity.gps && !validity.magnetic && !validity.barometer &&
                   f.m_observations.gnss_generation() == 2 && f.m_observations.magnetic_generation() == 2 &&
                   f.m_observations.barometer_generation() == 2,
               "switch away and back immediately vetoes old-generation aiding, even if rate-limited");
        for (unsigned i = 0; i < 75; ++i)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
        }
        expect(!f.m_fault && f.m_gnss.generation == 2 && !f.m_gnss.quality_result.qualified &&
                   f.m_magnetic.generation == 2 && !f.m_magnetic.lifecycle.state().active &&
                   f.m_magnetic.corrections == mag_corrections &&
                   f.m_height_aiding.session(FormalEskf::height_type::Source::barometer).generation == 2,
               "newly selected same-ID generations requalify GNSS/magnetic and restart height-bias history");
        expect(!f.output_validity(clock_us).gps_height &&
                   f.m_height_aiding.session(FormalEskf::height_type::Source::gnss).generation == 0,
               "new receiver with invalid velocity cannot lend old-generation height activity current validity");
        expect(f.m_gnss.lifecycle.state().velocity.last_fuse_time_us <= gps_progress,
               "queueing replacement data cannot claim a successful GNSS fusion");
        expect(std::isnan(f.m_gnss.velocity_ratios[0].filtered) && std::isnan(f.m_magnetic.ratios[0].filtered),
               "switch-away/back cannot retain old generation innovation summaries");
        auto const rejected = f.m_arrival_rejections;
        auto const fusion_time = f.m_time_us;
        auto gps = f.m_gps;
        gps.timestamp_sample = clock_us + 1;
        gps.device_id = 0;
        test.m_gps_pub.publish(gps);
        f.Run();
        expect(f.m_arrival_rejections == rejected + 1 && f.m_observations.gnss_generation() == 2 &&
                   f.m_observations.gnss_source().device_id == 201 && f.m_time_us == fusion_time,
               "future-clock source invalidation is rejected without revoking trusted source identity or advancing "
               "fusion");
        gps.timestamp_sample = f.m_gps.timestamp_sample - 2;
        test.m_gps_pub.publish(gps);
        f.Run();
        expect(f.m_arrival_rejections == rejected + 2 && f.m_observations.gnss_generation() == 2,
               "out-of-order source data cannot retroactively switch the selected generation");
    }

    static void delayed_resets_and_failure()
    {
        FormalEskfModuleTest test;
        test.warmup();
        auto & f = test.m_filter;
        // Force legitimate reacquisition actions through the existing controller;
        // keep the live output history, then inspect the main reset transforms.
        f.m_gnss.lifecycle.reset();
        f.m_gnss.input.time_us = f.m_time_us;
        f.m_gnss.input.sample.velocity_n = FormalEskf::vector3(1., 2., 3.);
        f.m_gnss.input.sample.latitude_deg += 0.00001;
        f.m_gnss.sample = {f.m_time_us, formal_eskf::runtime::GnssQualityDecision::accepted};
        auto const before = f.m_output.current();
        auto const main_before = f.m_state;
        auto const position_counter = f.m_position_reset_counter;
        auto const velocity_counter = f.m_velocity_reset_counter;
        f.fuse_gps({});
        expect(!f.m_fault && f.m_position_reset_counter == std::uint8_t(position_counter + 1) &&
                   f.m_velocity_reset_counter == std::uint8_t(velocity_counter + 1),
               "delayed GNSS reacquisition executes both production reset paths");
        expect(formal_eskf::linalg::norm((f.m_output.current().v_n - before.v_n) - (f.m_state.v_n - main_before.v_n)) <
                       1e-12 &&
                   formal_eskf::linalg::norm((f.m_output.current().p_n - before.p_n) -
                                             (f.m_state.p_n - main_before.p_n)) < 1e-12 &&
                   f.m_output.current().time_us == before.time_us,
               "GNSS reset translates the current output by the main delta without retiming it");
        // Failure in delayed covariance occurs after the current attitude phase.
        auto const output_epoch = f.m_output.current().time_us;
        auto const fusion_epoch = f.m_time_us;
        auto const last_navigation = f.m_last_output;
        f.m_covariance(0, 0) = std::numeric_limits<double>::quiet_NaN();
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        vehicle_attitude_s attitude{};
        orb_copy(ORB_ID(vehicle_attitude), test.m_outputs[0], &attitude);
        expect(f.m_fault && attitude.timestamp_sample == output_epoch + 4000U && f.m_time_us == fusion_epoch &&
                   f.m_last_output == last_navigation,
               "fast attitude retains its current epoch when subsequent delayed prediction fails; no navigation is "
               "published");
    }

    static void alignment_interval_admission()
    {
        FormalEskfModuleTest test;
        for (std::uint32_t period : {0U, 3799U, 4201U, 50000U})
        {
            test.m_sample.timestamp_sample += 4000;
            test.m_sample.delta_angle_dt = period;
            test.tick();
            expect(!test.m_filter.m_initialized && !test.m_filter.m_fault && test.m_filter.m_time_us == 0 &&
                       test.m_filter.m_initialization.imu_samples == 0,
                   "alignment rejects unsupported IMU periods before accumulation or seeding");
        }
        test.m_sample.delta_angle_dt = 4000;
        test.m_sample.timestamp_sample += 4000;
        test.tick();
        expect(test.m_filter.m_initialization.imu_samples == 1 && !test.m_filter.m_fault,
               "valid profile input can begin alignment after rejected pre-start samples");
    }

    static void startup_parameters()
    {
        FormalEskfModuleTest test;
        auto const gravity = param_find("FESKF_GRAV_EN");
        auto const source = param_find("FESKF_DECL_TYPE");
        auto const declination = param_find("FESKF_MAG_DECL");
        std::int32_t saved_gravity{}, saved_source{};
        float saved_declination{};
        expect(param_get(gravity, &saved_gravity) == 0 && param_get(source, &saved_source) == 0 &&
                   param_get(declination, &saved_declination) == 0,
               "read actual PX4 startup settings");
        std::int32_t invalid_gravity = -1;
        param_set_no_notification(gravity, &invalid_gravity);
        expect(!test.m_filter.init(), "invalid gravity selection prevents startup");
        std::int32_t disabled_gravity = 0;
        std::int32_t invalid_source = 4;
        param_set_no_notification(gravity, &disabled_gravity);
        param_set_no_notification(source, &invalid_source);
        expect(!test.m_filter.init() && !test.m_filter.m_gravity_enabled,
               "declination rejection preserves the preceding gravity setting assignment");
        param_set_no_notification(source, &saved_source);
        float invalid_declination = std::numeric_limits<float>::quiet_NaN();
        param_set_no_notification(declination, &invalid_declination);
        expect(!test.m_filter.init(), "non-finite saved declination prevents startup");
        invalid_declination = 181.f;
        param_set_no_notification(declination, &invalid_declination);
        expect(!test.m_filter.init(), "out-of-range saved declination prevents startup");
        param_set_no_notification(gravity, &saved_gravity);
        param_set_no_notification(source, &saved_source);
        param_set_no_notification(declination, &saved_declination);
        expect(test.m_filter.init(), "restored PX4 settings permit startup");
        test.snapshot("startup-parameters");
    }

    static void admission()
    {
        enum class Input
        {
            duplicate,
            zero_angle_period,
            long_angle_period,
            zero_velocity_period,
            short_velocity_period,
            gyro_clipping,
            clipping_bits,
            acceleration_clipping
        };
        for (auto scenario :
             {Input::duplicate, Input::zero_angle_period, Input::long_angle_period, Input::zero_velocity_period,
              Input::short_velocity_period, Input::gyro_clipping, Input::clipping_bits, Input::acceleration_clipping})
        {
            FormalEskfModuleTest test;
            test.warmup();
            auto const fusion_time = test.m_filter.m_time_us;
            auto const output_time = test.m_filter.m_output.current().time_us;
            if (scenario != Input::duplicate)
            {
                test.m_sample.timestamp_sample += 4000U;
            }
            switch (scenario)
            {
            case Input::duplicate:
                break; // Duplicate endpoint.
            case Input::zero_angle_period:
                test.m_sample.delta_angle_dt = 0;
                break;
            case Input::long_angle_period:
                test.m_sample.delta_angle_dt = 4201;
                break;
            case Input::zero_velocity_period:
                test.m_sample.delta_velocity_dt = 0;
                break;
            case Input::short_velocity_period:
                test.m_sample.delta_velocity_dt = 3799;
                break;
            case Input::gyro_clipping:
                test.m_sample.delta_angle_clipping = 1;
                break;
            case Input::clipping_bits:
                test.m_sample.delta_velocity_clipping = 8;
                break;
            case Input::acceleration_clipping:
                test.m_sample.delta_velocity_clipping = 1;
                break;
            }
            test.tick();
            bool const accepted = scenario == Input::acceleration_clipping;
            expect(test.m_filter.m_fault != accepted,
                   "invalid initialized IMU faults; supported acceleration clipping is admitted");
            expect(test.m_filter.m_time_us == fusion_time + (accepted ? 4000 : 0) &&
                       test.m_filter.m_output.current().time_us == output_time + (accepted ? 4000 : 0),
                   "admission faults preserve both horizons; accepted input advances each exactly once");
        }
    }

    static void gaps_and_periods()
    {
        FormalEskfModuleTest test;
        test.warmup();
        test.m_sample.timestamp_sample += 4200U;
        test.tick();
        expect(!test.m_filter.m_fault && test.m_filter.m_gaps == 0U,
               "200 us positive gap is tolerated without synthetic prediction");
        test.m_sample.timestamp_sample += 4201U;
        test.tick();
        expect(!test.m_filter.m_fault && test.m_filter.m_imu_history.latest()->hold_period_us == 201U,
               "201 us positive gap is retained for identical output/main propagation");
        test.m_sample.timestamp_sample += 3800U;
        test.tick();
        expect(!test.m_filter.m_fault, "minus 200 us overlap is accepted");
        test.m_sample.timestamp_sample += 4000U;
        test.m_sample.delta_velocity_dt = 4200U;
        test.m_sample.delta_angle[0] = 0.00004f;
        test.m_sample.delta_angle[1] = -0.00008f;
        test.m_sample.delta_angle[2] = 0.00012f;
        test.m_sample.delta_velocity[0] = 0.0002f;
        test.m_sample.delta_velocity[1] = -0.0003f;
        test.m_sample.delta_velocity[2] = static_cast<float>(-9.80665 * 0.0042);
        test.tick();
        expect(!test.m_filter.m_fault &&
                   test.m_filter.m_imu_history.latest()->sample.mean_rates().specific_force_b(2U) ==
                       static_cast<double>(test.m_sample.delta_velocity[2]) / (4200. * 1e-6),
               "IMU conversion uses the acceleration integration period independently");
        expect(test.m_filter.m_imu_history.latest()->sample.mean_rates().angular_rate_b(0U) ==
                   static_cast<double>(test.m_sample.delta_angle[0]) / (4000. * 1e-6),
               "nonzero angular increment uses the gyro integration period");
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            expect(test.m_filter.m_imu_history.latest()->sample.mean_rates().specific_force_b(axis) ==
                           static_cast<double>(test.m_sample.delta_velocity[axis]) / (4200. * 1e-6) &&
                       test.m_filter.m_imu_history.latest()->sample.mean_rates().angular_rate_b(axis) ==
                           static_cast<double>(test.m_sample.delta_angle[axis]) / (4000. * 1e-6),
                   "input conversion preserves every FRD axis, sign and independent measured period");
        }
        test.snapshot("periods");
        test.m_sample.timestamp_sample += 5000U;
        test.m_sample.delta_velocity_dt = 4000U;
        test.m_sample.delta_angle[0] = 0.f;
        test.m_sample.delta_angle[1] = 0.f;
        test.m_sample.delta_angle[2] = 0.f;
        test.m_sample.delta_velocity[0] = 0.f;
        test.m_sample.delta_velocity[1] = 0.f;
        test.m_sample.delta_velocity[2] = static_cast<float>(-9.80665 * 0.004);
        test.tick();
        expect(!test.m_filter.m_fault && test.m_filter.m_imu_history.latest()->hold_period_us == 1000U,
               "gap with distinct previous and current rates is retained");
        test.snapshot("held-measured-rate");
        auto const predictions = test.m_filter.m_predictions;
        for (unsigned i = 0; i < 54; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
        }
        expect(!test.m_filter.m_fault && test.m_filter.m_gaps == 2U &&
                   test.m_filter.m_predictions == predictions + 54U && test.m_filter.m_output.ready(),
               "retained held segments reach the main filter exactly once while matching current-output history");
        auto const horizon = test.m_filter.m_time_us;
        auto const count = publications;
        clock_us += 100000U;
        test.m_filter.Run();
        test.capture_outputs();
        expect(test.m_filter.m_time_us == horizon && publications == count + 3U,
               "watchdog publishes status without an IMU prediction or estimate publication");
        test.snapshot("watchdog");
    }

    static void interval_boundaries()
    {
        FormalEskfModuleTest test;
        test.warmup();
        for (std::uint32_t period : {3800U, 4200U})
        {
            auto const predictions = test.m_filter.m_predictions;
            test.m_sample.timestamp_sample += period;
            test.m_sample.delta_angle_dt = period;
            test.m_sample.delta_velocity_dt = period;
            test.m_sample.delta_velocity[2] = static_cast<float>(-9.80665 * static_cast<double>(period) * 1e-6);
            test.tick();
            expect(!test.m_filter.m_fault && test.m_filter.m_predictions == predictions + 1U &&
                       test.m_filter.m_imu_history.latest_time_us() == test.m_sample.timestamp_sample,
                   "profile minimum/maximum intervals advance one delayed step without a synthetic gap");
            expect(test.m_filter.m_gaps == 0U, "matching elapsed and measured periods need no gap hold");
            test.snapshot("interval-boundary");
        }
    }

    enum class Fault
    {
        non_finite,
        source_change,
        gyro_source_change,
        calibration_change,
        accel_calibration_change,
        overlap,
        outage,
        prediction,
        quality
    };

    static void fault(Fault kind)
    {
        FormalEskfModuleTest test;
        test.warmup();
        auto const previous = test.m_filter.m_imu_history.latest_time_us();
        auto const predictions = test.m_filter.m_predictions;
        test.m_sample.timestamp_sample += 4000U;
        switch (kind)
        {
        case Fault::non_finite:
            test.m_sample.delta_angle[0] = std::numeric_limits<float>::infinity();
            break;
        case Fault::source_change:
            ++test.m_sample.accel_device_id;
            break;
        case Fault::gyro_source_change:
            ++test.m_sample.gyro_device_id;
            break;
        case Fault::accel_calibration_change:
            ++test.m_sample.accel_calibration_count;
            break;
        case Fault::calibration_change:
            ++test.m_sample.gyro_calibration_count;
            break;
        case Fault::overlap:
            test.m_sample.timestamp_sample -= 201U;
            break;
        case Fault::outage:
            test.m_sample.timestamp_sample += 46001U;
            break;
        case Fault::prediction:
            test.m_filter.m_covariance(0U, 0U) = std::numeric_limits<double>::quiet_NaN();
            break;
        case Fault::quality:
            test.m_filter.m_gnss.quality_parameters.velocity_limit = std::numeric_limits<double>::quiet_NaN();
            break;
        }
        auto const fusion_before = test.m_filter.m_time_us;
        for (unsigned i = 0; i < 16 && !test.m_filter.m_fault; ++i)
        {
            test.tick();
            if (!test.m_filter.m_fault)
            {
                test.m_sample.timestamp_sample += 4000U;
            }
        }
        bool const delayed_failure = kind == Fault::prediction || kind == Fault::quality;
        expect(test.m_filter.m_fault, "numerical and timing failures latch a restart-required fault");
        test.m_filter.publish_status(clock_us);
        estimator_status_s status{};
        estimator_status_flags_s flags{};
        estimator_sensor_bias_s bias{};
        orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status);
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        orb_copy(ORB_ID(estimator_sensor_bias), test.m_outputs[6], &bias);
        expect((status.solution_status_flags & 0x3fU) == 0 && !flags.cs_tilt_align && !bias.accel_bias_valid &&
                   !bias.gyro_bias_valid && status.filter_fault_flags == 0,
               "real numerical/history failures revoke capabilities without inventing an IMU sensor fault");
        expect(delayed_failure || (test.m_filter.m_time_us == fusion_before &&
                                   test.m_filter.m_imu_history.latest_time_us() == previous),
               "IMU admission faults cannot consume either horizon");
        auto const completed = test.m_filter.m_predictions;
        expect(delayed_failure || completed == predictions, "admission faults do not predict the main filter");
        test.snapshot("fault");
        test.m_sample.delta_angle[0] = 0.f;
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        expect(test.m_filter.m_fault && test.m_filter.m_predictions == completed,
               "a following valid sample does not clear a latched fault");
        test.snapshot("fault-latched");
    }

private:
    FormalEskf m_filter;
    std::array<orb_id_t, 7U> m_topics{ORB_ID(vehicle_attitude),        ORB_ID(vehicle_local_position),
                                      ORB_ID(vehicle_global_position), ORB_ID(vehicle_odometry),
                                      ORB_ID(estimator_status),        ORB_ID(estimator_status_flags),
                                      ORB_ID(estimator_sensor_bias)};
    std::array<int, 7U> m_outputs{};
    vehicle_imu_s m_sample{};
    bool m_armed = false;
    bool m_landed = true, m_at_rest = true;
    bool m_in_ground_effect = false, m_fixed_wing = false, m_send_status = true, m_send_land = true;
    float m_baro_altitude = 488.f;
    bool m_send_gps = true;
    bool m_gps_velocity_valid = true;
    std::uint32_t m_gps_device = 201U, m_mag_device = 301U, m_baro_device = 401U;
    std::uint8_t m_mag_calibration{};
    uORB::Publication<vehicle_imu_s> m_imu_pub{ORB_ID(vehicle_imu)};
    uORB::Publication<sensor_gps_s> m_gps_pub{ORB_ID(vehicle_gps_position)};
    uORB::Publication<vehicle_magnetometer_s> m_mag_pub{ORB_ID(vehicle_magnetometer)};
    uORB::Publication<vehicle_air_data_s> m_baro_pub{ORB_ID(vehicle_air_data)};
    uORB::Publication<vehicle_land_detected_s> m_land_pub{ORB_ID(vehicle_land_detected)};
    uORB::Publication<vehicle_status_s> m_status_pub{ORB_ID(vehicle_status)};
}; /* end class FormalEskfModuleTest */

int main(int argc, char ** argv)
{
    if (argc > 2)
    {
        std::fprintf(stderr, "Usage: %s [trace-file]\n", argv[0]);
        return 2;
    }
    if (argc == 2)
    {
        trace_file = std::fopen(argv[1], "wb");
        if (trace_file == nullptr)
        {
            return 2;
        }
    }
    uORB::Manager::initialize();
    param_init();
    FormalEskfModuleTest::timing_parameters();
    FormalEskfModuleTest::delayed_pipeline();
    FormalEskfModuleTest::delayed_resets_and_failure();
    FormalEskfModuleTest::delayed_source_changes();
    FormalEskfModuleTest::delayed_heading_reset();
    FormalEskfModuleTest::alignment_interval_admission();
    FormalEskfModuleTest::startup_parameters();
    FormalEskfModuleTest::startup();
    FormalEskfModuleTest::admission();
    FormalEskfModuleTest::gaps_and_periods();
    FormalEskfModuleTest::interval_boundaries();
    using GravityCase = FormalEskfModuleTest::GravityCase;
    for (auto scenario :
         {GravityCase::eligible, GravityCase::disabled, GravityCase::aided, GravityCase::high_force,
          GravityCase::rest_override, GravityCase::stale_rest, GravityCase::clipped, GravityCase::unaligned})
    {
        FormalEskfModuleTest::gravity_conditions(scenario);
    }
    FormalEskfModuleTest::gravity_history();
    FormalEskfModuleTest::gravity_periods_and_bias();
    FormalEskfModuleTest::gravity_failure(FormalEskfModuleTest::GravityFailure::tilt);
    FormalEskfModuleTest::gravity_failure(FormalEskfModuleTest::GravityFailure::sample);
    FormalEskfModuleTest::gravity_failure(FormalEskfModuleTest::GravityFailure::correction);
    FormalEskfModuleTest::gravity_in_run(true);
    FormalEskfModuleTest::gravity_in_run(false);
    FormalEskfModuleTest::magnetic_outage(false);
    FormalEskfModuleTest::magnetic_outage(true);
    using MagneticInputCase = FormalEskfModuleTest::MagneticInputCase;
    for (auto scenario :
         {MagneticInputCase::stale, MagneticInputCase::future, MagneticInputCase::duplicate,
          MagneticInputCase::non_finite, MagneticInputCase::zero_field, MagneticInputCase::invalid_device})
    {
        FormalEskfModuleTest::magnetic_input(scenario);
    }
    using MagneticSourceCase = FormalEskfModuleTest::MagneticSourceCase;
    for (auto scenario :
         {MagneticSourceCase::ground, MagneticSourceCase::aided_air, MagneticSourceCase::unaided_air,
          MagneticSourceCase::unknown, MagneticSourceCase::stale_land, MagneticSourceCase::land_overrides_arming})
    {
        FormalEskfModuleTest::magnetic_source(scenario);
    }
    FormalEskfModuleTest::magnetic_bias_and_field();
    FormalEskfModuleTest::magnetic_timeout(false);
    FormalEskfModuleTest::magnetic_timeout(true);
    FormalEskfModuleTest::magnetic_failure(false);
    FormalEskfModuleTest::magnetic_failure(true);
    FormalEskfModuleTest::magnetic_heading_history();
    FormalEskfModuleTest::magnetic_model_lookup();
    FormalEskfModuleTest::magnetic_reference_change(false);
    FormalEskfModuleTest::magnetic_reference_change(true);
    FormalEskfModuleTest::navigation_fusion(false);
    FormalEskfModuleTest::navigation_fusion(true);
    using GnssInputCase = FormalEskfModuleTest::GnssInputCase;
    for (auto scenario :
         {GnssInputCase::duplicate, GnssInputCase::future, GnssInputCase::stale, GnssInputCase::velocity_invalid,
          GnssInputCase::metadata_invalid, GnssInputCase::receiver, GnssInputCase::quality})
    {
        FormalEskfModuleTest::navigation_admission(scenario);
    }
    FormalEskfModuleTest::navigation_outage();
    FormalEskfModuleTest::navigation_failure(false);
    FormalEskfModuleTest::navigation_failure(true);
    using HeightInputCase = FormalEskfModuleTest::HeightInputCase;
    for (auto scenario : {HeightInputCase::duplicate, HeightInputCase::future, HeightInputCase::stale,
                          HeightInputCase::invalid, HeightInputCase::calibration, HeightInputCase::unqualified})
    {
        FormalEskfModuleTest::height_admission(scenario);
    }
    FormalEskfModuleTest::height_transaction_failure();
    FormalEskfModuleTest::navigation_rejected_quality();
    FormalEskfModuleTest::navigation_timeout();
    FormalEskfModuleTest::height_combined_resets();
    FormalEskfModuleTest::ground_effect_in_run();
    FormalEskfModuleTest::ground_effect_missing_context();
    FormalEskfModuleTest::navigation_context(true, true);
    FormalEskfModuleTest::navigation_context(false, true);
    FormalEskfModuleTest::navigation_context(false, false);
    FormalEskfModuleTest::output_mapping(false, false);
    FormalEskfModuleTest::output_mapping(true, false);
    FormalEskfModuleTest::output_mapping(false, true);
    FormalEskfModuleTest::output_cadence();
    using Fault = FormalEskfModuleTest::Fault;
    for (auto fault :
         {Fault::non_finite, Fault::source_change, Fault::gyro_source_change, Fault::calibration_change,
          Fault::accel_calibration_change, Fault::overlap, Fault::outage, Fault::prediction, Fault::quality})
    {
        FormalEskfModuleTest::fault(fault);
    }
    FILE * comparison_trace = trace_file;
    trace_file = nullptr; // Deliberate diagnostic/startup changes have independent assertions.
    FormalEskfModuleTest::publication_preparation();
    FormalEskfModuleTest::resource_failures();
    FormalEskfModuleTest::diagnostic_contention();
    FormalEskfModuleTest::diagnostic_concurrency();
    FormalEskfModuleTest::diagnostic_shutdown();
    // New installation scenarios have independent oracles; preserve the prior comparison trace.
    FormalEskfModuleTest::sensor_position_parameters();
    FormalEskfModuleTest::rotating_sensor_geometry();
    FormalEskfModuleTest::body_reference_output();
    using CapabilityCase = FormalEskfModuleTest::CapabilityCase;
    for (auto scenario :
         {CapabilityCase::all_aiding, CapabilityCase::height_only, CapabilityCase::horizontal_only,
          CapabilityCase::velocity_without_height, CapabilityCase::source_changed, CapabilityCase::no_aiding,
          CapabilityCase::no_tilt, CapabilityCase::stale, CapabilityCase::fault})
    {
        FormalEskfModuleTest::confidence_capabilities(scenario);
    }
    FormalEskfModuleTest::bias_confidence();
    FormalEskfModuleTest::innovation_reporting();
    FormalEskfModuleTest::heading_readiness(false);
    FormalEskfModuleTest::heading_readiness(true);
    FormalEskfModuleTest::diagnostic_flags();
    FormalEskfModuleTest::navigation_covariance_frame();
    FormalEskfModuleTest::dateline_output();
    trace_file = comparison_trace;
    if (trace_file != nullptr)
    {
        bool const written = std::ferror(trace_file) == 0;
        int const closed = std::fclose(trace_file);
        expect(written && closed == 0, "write complete comparison trace");
    }
    std::printf("PX4 module: %u scenarios, %u failures\n", scenarios, failures);
    return failures == 0U ? 0 : 1;
}
