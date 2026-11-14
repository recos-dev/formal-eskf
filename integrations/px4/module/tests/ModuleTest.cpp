// SPDX-License-Identifier: BSD-3-Clause
#include "../FormalEskf.hpp"

#include <array>
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
        m_gps_pub.publish(gps);
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
