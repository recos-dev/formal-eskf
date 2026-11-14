// SPDX-License-Identifier: BSD-3-Clause
#include "../FormalEskf.hpp"

#include <array>
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

// Only platform time/timer dispatch are controlled. Numerical code, messages,
// uORB storage/subscriptions and the production Run() implementation are real.
// The work-queue manager is intentionally stopped: PX4 reports its unavailable
// queue during construction, and this fixture invokes each tick synchronously.
// Scheduler startup, concurrency and shutdown are not tested here.
extern "C" hrt_abstime __wrap_hrt_absolute_time() { return clock_us; }
extern "C" void __wrap_hrt_call_after(hrt_call *, hrt_abstime delay, hrt_callout, void *)
{
    ++watchdog_calls;
    watchdog_delay = static_cast<std::uint32_t>(delay);
}
extern "C" void __wrap_hrt_cancel(hrt_call *) {}
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
        gps.device_id = 201U;
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
        gps.vel_ned_valid = true;
        if (m_send_gps)
        {
            m_gps_pub.publish(gps);
        }
        vehicle_magnetometer_s mag{};
        mag.timestamp = clock_us;
        mag.timestamp_sample = clock_us;
        mag.device_id = m_mag_device;
        mag.magnetometer_ga[0] = 0.22f;
        mag.magnetometer_ga[1] = 0.01f;
        mag.magnetometer_ga[2] = 0.43f;
        m_mag_pub.publish(mag);
        vehicle_air_data_s baro{};
        baro.timestamp = clock_us;
        baro.timestamp_sample = clock_us;
        baro.baro_device_id = 401U;
        baro.baro_alt_meter = 488.f;
        m_baro_pub.publish(baro);
        vehicle_land_detected_s land{};
        land.timestamp = clock_us;
        land.landed = true;
        land.at_rest = true;
        m_land_pub.publish(land);
        vehicle_status_s vehicle{};
        vehicle.timestamp = clock_us;
        vehicle.arming_state = m_armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
        m_status_pub.publish(vehicle);
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
        expect(m_filter.m_gps_updates > 0U, "post-startup ticks run GNSS correction");
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
                                               heading.horizontal_acceleration, m_filter.m_magnetic.test_ratio};
        for (std::size_t i = 0U; i < 4U; ++i)
        {
            magnetic_values[4U + i] = m_filter.m_quaternion_reset_delta.coefficients()(i);
        }
        trace_bytes(magnetic_values.data(), sizeof(magnetic_values));
    }

    void set_horizontal_aiding(bool enabled)
    {
        // Establish the preceding GNSS phase's lifecycle state. Magnetic tests
        // exercise fuse_mag(), not GNSS qualification or its numerical update.
        using Gnss = formal_eskf::runtime::GnssFusion;
        m_filter.m_gnss_fusion.reset();
        if (enabled)
        {
            Gnss::Conditions conditions;
            conditions.horizontal_position = {true, true, true};
            expect(formal_eskf::succeeded(
                       m_filter.m_gnss_fusion.update(clock_us, conditions, FormalEskf::gnss_fusion_parameters,
                                                     {clock_us, formal_eskf::runtime::GnssQualityDecision::accepted})),
                   "prepare horizontal aiding context");
            expect(formal_eskf::succeeded(m_filter.m_gnss_fusion.record_fusion(
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
        expect(test.m_filter.m_time_us == horizon && test.m_filter.m_tilt_alignment.aligned() &&
                   !test.m_filter.m_gravity.active,
               "invalid IMU metadata rejects before gravity and clears only observation history");
        test.snapshot("gravity-invalid-metadata");
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
        auto const previous = test.m_filter.m_imu_history.time_us;
        test.tick();
        expect(!test.m_filter.m_fault && test.m_filter.m_time_us == test.m_sample.timestamp_sample &&
                   test.m_filter.m_imu_history.time_us == previous,
               "non-finite uninitialized sample consumes time, preserves metadata and does not latch a fault");
        test.snapshot("startup");
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
        FormalEskfModuleTest test;
        test.warmup();
        auto const previous_time = test.m_filter.m_time_us;
        auto const previous_sample = test.m_filter.m_imu_history.time_us;
        auto const rejects = test.m_filter.m_rejections;
        test.tick();
        expect(test.m_filter.m_rejections == rejects + 1U && test.m_filter.m_time_us == previous_time &&
                   test.m_filter.m_imu_history.time_us == previous_sample && !test.m_filter.m_fault,
               "duplicate IMU rejects without consuming time or metadata");
        test.m_sample.timestamp_sample += 4000U;
        test.m_sample.delta_angle_dt = 0U;
        test.m_sample.delta_angle[0] = std::numeric_limits<float>::infinity();
        test.tick();
        expect(test.m_filter.m_time_us == previous_time && !test.m_filter.m_fault,
               "zero gyro period rejects before rate conversion and non-finite fault handling");
        test.m_sample.delta_angle[0] = 0.f;
        test.m_sample.delta_angle_dt = 50001U;
        test.tick();
        expect(test.m_filter.m_time_us == previous_time && !test.m_filter.m_fault,
               "oversized gyro period rejects before timeline consumption");
        test.m_sample.delta_angle_dt = 4000U;
        test.m_sample.delta_velocity_dt = 0U;
        test.tick();
        expect(test.m_filter.m_time_us == previous_time && !test.m_filter.m_fault,
               "zero acceleration period rejects before rate conversion");
        test.m_sample.delta_velocity_dt = 50001U;
        test.tick();
        expect(test.m_filter.m_time_us == previous_time && !test.m_filter.m_fault,
               "oversized acceleration period rejects before timeline consumption");
        test.m_sample.delta_velocity_dt = 4000U;
        test.m_sample.delta_angle_clipping = 1U;
        test.tick();
        expect(test.m_filter.m_time_us == previous_time && !test.m_filter.m_fault,
               "gyro clipping rejects before timeline consumption");
        test.m_sample.delta_angle_clipping = 0U;
        test.m_sample.delta_velocity_clipping = 8U;
        test.tick();
        expect(test.m_filter.m_time_us == previous_time && !test.m_filter.m_fault,
               "unsupported clipping bits reject before timeline consumption");
        test.m_sample.delta_velocity_clipping = 1U;
        test.tick();
        expect(test.m_filter.m_time_us == test.m_sample.timestamp_sample && !test.m_filter.m_fault,
               "valid accelerometer clipping is admitted after initialization");
        test.snapshot("admission");
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
        expect(!test.m_filter.m_fault && test.m_filter.m_gaps == 1U,
               "201 us positive gap predicts the held previous rate");
        test.m_sample.timestamp_sample += 3800U;
        test.tick();
        expect(!test.m_filter.m_fault, "minus 200 us overlap is accepted");
        test.m_sample.timestamp_sample += 4000U;
        test.m_sample.delta_velocity_dt = 5000U;
        test.m_sample.delta_angle[0] = 0.00004f;
        test.m_sample.delta_angle[1] = -0.00008f;
        test.m_sample.delta_angle[2] = 0.00012f;
        test.m_sample.delta_velocity[0] = 0.0002f;
        test.m_sample.delta_velocity[1] = -0.0003f;
        test.m_sample.delta_velocity[2] = static_cast<float>(-9.80665 * 0.005);
        test.tick();
        expect(!test.m_filter.m_fault && test.m_filter.m_imu_history.measurement.specific_force_b(2U) ==
                                             static_cast<double>(test.m_sample.delta_velocity[2]) / (5000. * 1e-6),
               "IMU conversion uses the acceleration integration period independently");
        expect(test.m_filter.m_imu_history.measurement.angular_rate_b(0U) ==
                   static_cast<double>(test.m_sample.delta_angle[0]) / (4000. * 1e-6),
               "nonzero angular increment uses the gyro integration period");
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            expect(test.m_filter.m_imu_history.measurement.specific_force_b(axis) ==
                           static_cast<double>(test.m_sample.delta_velocity[axis]) / (5000. * 1e-6) &&
                       test.m_filter.m_imu_history.measurement.angular_rate_b(axis) ==
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
        expect(!test.m_filter.m_fault && test.m_filter.m_gaps == 2U,
               "gap with distinct previous and current rates is admitted");
        test.snapshot("held-measured-rate");
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
        for (std::uint32_t period : {1U, 50000U})
        {
            auto const predictions = test.m_filter.m_predictions;
            test.m_sample.timestamp_sample += period;
            test.m_sample.delta_angle_dt = period;
            test.m_sample.delta_velocity_dt = period;
            test.m_sample.delta_velocity[2] = static_cast<float>(-9.80665 * static_cast<double>(period) * 1e-6);
            test.tick();
            expect(!test.m_filter.m_fault && test.m_filter.m_predictions == predictions + 1U &&
                       test.m_filter.m_imu_history.time_us == test.m_sample.timestamp_sample,
                   "inclusive minimum/maximum IMU periods reach prediction without a synthetic gap");
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
        auto const previous = test.m_filter.m_imu_history.time_us;
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
            test.m_filter.m_gnss_quality_parameters.velocity_limit = std::numeric_limits<double>::quiet_NaN();
            break;
        }
        test.tick();
        expect(test.m_filter.m_fault && test.m_filter.m_time_us == test.m_sample.timestamp_sample &&
                   test.m_filter.m_predictions == predictions,
               "fault consumes the admitted horizon and prevents a successful prediction");
        bool const early_failure = kind == Fault::non_finite || kind == Fault::quality;
        expect(test.m_filter.m_imu_history.time_us == (early_failure ? previous : test.m_sample.timestamp_sample),
               "only entry into initialized prediction consumes previous-IMU metadata on failure");
        test.snapshot("fault");
        test.m_sample.delta_angle[0] = 0.f;
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        expect(test.m_filter.m_fault && test.m_filter.m_predictions == predictions,
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
    bool m_send_gps = true;
    std::uint32_t m_mag_device = 301U;
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
    using Fault = FormalEskfModuleTest::Fault;
    for (auto fault :
         {Fault::non_finite, Fault::source_change, Fault::gyro_source_change, Fault::calibration_change,
          Fault::accel_calibration_change, Fault::overlap, Fault::outage, Fault::prediction, Fault::quality})
    {
        FormalEskfModuleTest::fault(fault);
    }
    if (trace_file != nullptr)
    {
        bool const written = std::ferror(trace_file) == 0;
        int const closed = std::fclose(trace_file);
        expect(written && closed == 0, "write complete comparison trace");
    }
    std::printf("PX4 module: %u scenarios, %u failures\n", scenarios, failures);
    return failures == 0U ? 0 : 1;
}
