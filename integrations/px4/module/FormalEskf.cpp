// SPDX-License-Identifier: BSD-3-Clause
#include <algorithm>
#include <cmath>
#include "FormalEskf.hpp"
#include <lib/world_magnetic_model/geo_mag_declination.h>
#include <px4_platform_common/param.h>

using time_literals::operator""_ms;
// Double precision dense 15-state operations need more stack than EKF2's
// generated float expressions. SITL-only; measured .su files accompany build.
static constexpr px4::wq_config_t formal_eskf_queue{"wq:formal_eskf", 60 * 1024, -15};

FormalEskf::FormalEskf() : ScheduledWorkItem(MODULE_NAME, formal_eskf_queue)
{
    // Register output topics before logger starts; publish only valid samples.
    m_attitude_pub.advertise();
    m_local_pub.advertise();
    m_global_pub.advertise();
    m_odometry_pub.advertise();
    m_bias_pub.advertise();
    m_status_pub.advertise();
    m_flags_pub.advertise();
    m_parameters.gravity_n = vector3(0., 0., 9.80665);
    m_parameters.minimum_quaternion_norm = minimum_quaternion_norm;
    m_parameters.quaternion_squared_norm_tolerance = 1e-10;
    m_parameters.dt_min = 1e-6;
    m_parameters.dt_max = 0.05;
    m_parameters.process_noise.specific_force_variance = vector3(1., 1., 1.);
    m_parameters.process_noise.angular_rate_variance = vector3(0.0025, 0.0025, 0.0025);
    m_parameters.process_noise.accelerometer_bias_random_walk_variance_density = vector3(1e-6, 1e-6, 1e-6);
    m_parameters.process_noise.gyroscope_bias_random_walk_variance_density = vector3(1e-8, 1e-8, 1e-8);
}

FormalEskf::~FormalEskf()
{
    m_imu_sub.unregisterCallback();
    perf_free(m_cycle_perf);
}

bool FormalEskf::init()
{
    int32_t gravity_enabled{};
    if (param_get(param_find("FESKF_GRAV_EN"), &gravity_enabled) != 0 || gravity_enabled < 0 || gravity_enabled > 1)
    {
        PX4_ERR("invalid gravity aiding setting");
        return false;
    }
    m_gravity_enabled = gravity_enabled != 0;
    int32_t declination_source{};
    float saved_declination{};
    if (param_get(param_find("FESKF_DECL_TYPE"), &declination_source) != 0 ||
        param_get(param_find("FESKF_MAG_DECL"), &saved_declination) != 0 || declination_source < 0 ||
        declination_source > 3 || !std::isfinite(saved_declination) || std::abs(saved_declination) > 180.f)
    {
        PX4_ERR("invalid magnetic declination settings");
        return false;
    }
    m_magnetic_settings = {uint32_t(declination_source), math::radians(double(saved_declination))};
    if (!m_imu_sub.registerCallback())
    {
        return false;
    }
    ScheduleNow();
    return true;
}

bool FormalEskf::recent(uint64_t sample, uint64_t max_age) const
{
    return sample != 0 && sample <= m_time && m_time - sample <= max_age;
}

void FormalEskf::update_gps_quality()
{
    using namespace formal_eskf::runtime;
    m_gnss_sample = {};
    if (m_gps.timestamp_sample <= m_gps_seen || m_gps.timestamp_sample > m_time)
    {
        return;
    }
    m_gps_seen = m_gps.timestamp_sample;
    // Match the source wrapper: invalid velocity is not a quality pass or a fresh aiding sample.
    if (!m_gps.vel_ned_valid || !recent(m_gps.timestamp_sample, 500000))
    {
        return;
    }
    if (m_gps.device_id != m_gps_device_id)
    {
        m_gnss_quality.reset();
        m_gnss_fusion.reset();
        m_gps_previous_fix = {};
        m_gps_device_id = m_gps.device_id;
    }
    m_gnss_sample.time_us = m_gps.timestamp_sample;
    m_gps_metadata_valid = std::isfinite(m_gps.latitude_deg) && std::abs(m_gps.latitude_deg) <= 90. &&
                           std::isfinite(m_gps.longitude_deg) && std::abs(m_gps.longitude_deg) <= 180. &&
                           std::isfinite(m_gps.altitude_msl_m) && std::isfinite(m_gps.altitude_ellipsoid_m) &&
                           std::isfinite(m_gps.hdop) && m_gps.hdop >= 0.f && std::isfinite(m_gps.vdop) &&
                           m_gps.vdop >= 0.f;
    if (!m_gps_metadata_valid)
    {
        m_gnss_quality.reset();
        m_gps_previous_fix = {};
        m_gnss_quality_result = {};
        m_gnss_sample.quality_available = false;
        ++m_rejections;
        return;
    }
    GnssQualitySample<backend_type> sample;
    sample.time_us = m_gps.timestamp_sample;
    sample.fix_type = m_gps.fix_type;
    sample.satellites = m_gps.satellites_used;
    sample.pdop = std::hypot(double(m_gps.hdop), double(m_gps.vdop));
    sample.horizontal_accuracy = m_gps.eph;
    sample.vertical_accuracy = m_gps.epv;
    sample.speed_accuracy = m_gps.s_variance_m_s;
    sample.spoofed = m_gps.spoofing_state == sensor_gps_s::SPOOFING_STATE_MULTIPLE;
    sample.velocity_n = vector3(m_gps.vel_n_m_s, m_gps.vel_e_m_s, m_gps.vel_d_m_s);
    sample.position_reference_time_us = m_gps_previous_fix.getProjectionReferenceTimestamp();
    if (sample.position_reference_time_us != 0)
    {
        float north{}, east{};
        m_gps_previous_fix.project(m_gps.latitude_deg, m_gps.longitude_deg, north, east);
        sample.position_delta_n = vector3(north, east, m_gps_previous_alt - m_gps.altitude_msl_m);
    }
    GnssQualityMotion motion;
    uint64_t const now = hrt_absolute_time();
    if (m_vehicle_status.timestamp != 0 && m_vehicle_status.timestamp <= now &&
        now - m_vehicle_status.timestamp < 3000000)
    {
        motion.in_air = m_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
    }
    if (m_land.timestamp != 0 && m_land.timestamp <= now && now - m_land.timestamp < 3000000)
    {
        motion.in_air = !m_land.landed;
        motion.vehicle_at_rest = m_land.at_rest;
    }
    m_gnss_quality_result = m_gnss_quality.update(sample, motion, m_time, m_gnss_quality_parameters);
    if (m_gnss_quality_result.decision == GnssQualityDecision::invalid_call)
    {
        check(m_gnss_quality_result.status);
        return;
    }
    m_gnss_sample.quality = m_gnss_quality_result.decision;
    m_gps_previous_fix.initReference(m_gps.latitude_deg, m_gps.longitude_deg, sample.time_us);
    m_gps_previous_alt = m_gps.altitude_msl_m;
    if (m_gnss_sample.quality == GnssQualityDecision::rejected)
    {
        ++m_rejections;
    }
}

bool FormalEskf::gps_good() const
{
    return m_gps_metadata_valid && m_gps.vel_ned_valid && recent(m_gps.timestamp_sample, 500000) &&
           m_gnss_quality_result.decision == formal_eskf::runtime::GnssQualityDecision::accepted;
}

uint16_t FormalEskf::gps_failure_flags() const
{
    auto const & result = m_gnss_quality_result;
    auto const & failed = result.failures;
    return (uint16_t(result.invalid_fix) << estimator_status_s::GPS_CHECK_FAIL_GPS_FIX) |
           (uint16_t(failed.satellites) << estimator_status_s::GPS_CHECK_FAIL_MIN_SAT_COUNT) |
           (uint16_t(failed.pdop) << estimator_status_s::GPS_CHECK_FAIL_MAX_PDOP) |
           (uint16_t(failed.horizontal_accuracy) << estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_ERR) |
           (uint16_t(failed.vertical_accuracy) << estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_ERR) |
           (uint16_t(failed.speed_accuracy) << estimator_status_s::GPS_CHECK_FAIL_MAX_SPD_ERR) |
           (uint16_t(failed.horizontal_drift) << estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_DRIFT) |
           (uint16_t(failed.vertical_drift) << estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_DRIFT) |
           (uint16_t(failed.horizontal_speed) << estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_SPD_ERR) |
           (uint16_t(failed.vertical_speed) << estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_SPD_ERR) |
           (uint16_t(failed.spoofed) << estimator_status_s::GPS_CHECK_FAIL_SPOOFED);
}

bool FormalEskf::gps_position_valid(uint64_t max_age) const
{
    auto const & group = m_gnss_fusion.state().horizontal_position;
    return group.active && !group.recovery_required &&
           recent(std::max(group.last_fuse_time_us, group.last_reset_time_us), max_age);
}

bool FormalEskf::gps_velocity_valid(uint64_t max_age) const
{
    auto const & group = m_gnss_fusion.state().velocity;
    return group.active && !group.recovery_required &&
           recent(std::max(group.last_fuse_time_us, group.last_reset_time_us), max_age);
}

bool FormalEskf::check(status_type status)
{
    if (formal_eskf::succeeded(status))
    {
        return true;
    }
    ++m_errors;
    m_fault = true; // Latch numerical failures; no silent reset or fallback.
    PX4_ERR("numerical failure %d at %llu", int(status), (unsigned long long)m_time);
    return false;
}

void FormalEskf::initialize(vehicle_imu_s const & sample, imu_type const & imu)
{
    const double accel_norm = formal_eskf::linalg::norm(imu.specific_force_b);
    if (m_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED || std::abs(accel_norm - 9.80665) > 0.5 ||
        formal_eskf::linalg::norm(imu.angular_rate_b) > 0.1)
    {
        m_init_samples = 0;
        m_accel_sum = {};
        m_gyro_sum = {};
        m_mag_sum = {};
        m_init_mag_samples = 0;
        return;
    }
    if (m_init_samples < 1000)
    {
        m_accel_sum = m_accel_sum + imu.specific_force_b;
        m_gyro_sum = m_gyro_sum + imu.angular_rate_b;
        ++m_init_samples;
    }
    if (m_mag.timestamp_sample > m_mag_init_seen && recent(m_mag.timestamp_sample, 500000))
    {
        if (m_mag.device_id != m_mag_init_source.device_id ||
            m_mag.calibration_count != m_mag_init_source.calibration_count)
        {
            m_mag_sum = {};
            m_init_mag_samples = 0;
        }
        m_mag_init_source = {m_mag.device_id, m_mag.calibration_count, 1U};
        const auto m = vector3(m_mag.magnetometer_ga[0], m_mag.magnetometer_ga[1], m_mag.magnetometer_ga[2]);
        if (m_mag.device_id != 0 && formal_eskf::linalg::all_finite(m) && formal_eskf::linalg::norm(m) > 0.1)
        {
            m_mag_sum = m_mag_sum + m;
            ++m_init_mag_samples;
        }
        m_mag_init_seen = m_mag.timestamp_sample;
    }
    if (m_init_samples < 250 || m_init_mag_samples < 20 || !gps_good() || !recent(m_baro.timestamp_sample, 500000) ||
        !std::isfinite(m_baro.baro_alt_meter) || std::hypot(m_gps.vel_n_m_s, m_gps.vel_e_m_s) > 0.8f ||
        std::abs(m_gps.vel_d_m_s) > 0.5f)
    {
        return;
    }

    // Invalid external altitude delays startup; it is not a numerical-core fault.
    if (!formal_eskf::succeeded(m_height_aiding.initialize(m_time, m_gps.altitude_msl_m)))
    {
        return;
    }

    const auto accel = m_accel_sum / double(m_init_samples);
    const auto mag = m_mag_sum / double(m_init_mag_samples);
    const double roll = std::atan2(-accel(1), -accel(2));
    const double pitch = std::atan2(accel(0), std::hypot(accel(1), accel(2)));
    const matrix::Dcm<double> tilt{matrix::Euler<double>(roll, pitch, 0.)};
    const matrix::Vector3<double> level_mag = tilt * matrix::Vector3<double>(mag(0), mag(1), mag(2));
    if (100. * std::hypot(level_mag(0), level_mag(1)) < magnetic_type::field_parameters.minimum_horizontal_field)
    {
        return;
    }
    auto const model = lookup_magnetic_model(m_gps.latitude_deg, m_gps.longitude_deg);
    m_mag_model_unavailable += !model.has_value();
    if (!check(m_magnetic_model.initialize(m_time, m_magnetic_settings,
                                           vector3(100. * level_mag(0), 100. * level_mag(1), 100. * level_mag(2)),
                                           model)))
    {
        return;
    }
    const double yaw = m_magnetic_model.declination() - std::atan2(level_mag(1), level_mag(0));
    const matrix::Quaternion<double> q{matrix::Euler<double>(roll, pitch, yaw)};
    if (!check(state_type::quaternion_type::try_from_coefficients(q(0), q(1), q(2), q(3), minimum_quaternion_norm,
                                                                  m_state.q_nb)))
    {
        return;
    }
    m_state.b_g = m_gyro_sum / double(m_init_samples);
    if (!check(formal_eskf::px4::try_limit_imu_bias(m_state)))
    {
        return;
    }
    m_state.v_n = vector3(m_gps.vel_n_m_s, m_gps.vel_e_m_s, m_gps.vel_d_m_s);
    const double diagonal[15] = {1., 1., 2.25, 0.09, 0.09, 0.09, 0.01, 0.01, 0.04, 0.04, 0.04, 0.04, 1e-4, 1e-4, 1e-4};
    for (size_t i = 0; i < 15; ++i)
    {
        m_covariance(i, i) = diagonal[i];
    }
    m_origin_time = m_time;
    m_projection.initReference(m_gps.latitude_deg, m_gps.longitude_deg, m_origin_time);
    m_mag_model_origin_time = m_origin_time;
    m_origin_alt = m_gps.altitude_msl_m;
    m_origin_ellipsoid = m_gps.altitude_ellipsoid_m;

    m_mag_seen = m_mag.timestamp_sample;
    m_mag_init_source.reference_id = m_magnetic_model.reference_id();
    m_magnetic_aiding.initialize(m_mag_init_source, m_mag_seen);
    m_last_imu = sample;
    m_previous_imu = imu;
    m_initialized = true;
    if (!update_imu_bias(sample, imu))
    {
        return;
    }
    fuse_gps();
    if (!m_fault && update_imu_health(sample))
    {
        fuse_height();
    }
    PX4_INFO("initialized from IMU + magnetometer + GNSS; yaw=%.2f deg, EKF2 replacement", math::degrees(yaw));
}

void FormalEskf::Run()
{
    if (should_exit())
    {
        m_imu_sub.unregisterCallback();
        ScheduleClear();
        exit_and_cleanup();
        return;
    }
    std::lock_guard<std::mutex> guard(m_state_mutex);
    ScheduleDelayed(20_ms); // Watchdog also runs if IMU publication stops.
    perf_begin(m_cycle_perf);
    m_gps_sub.update(&m_gps);
    m_mag_sub.update(&m_mag);
    m_baro_sub.update(&m_baro);
    m_land_sub.update(&m_land);
    m_vehicle_status_sub.update(&m_vehicle_status);
    vehicle_imu_s sample{};
    if (m_imu_sub.update(&sample))
    {
        process_imu(sample);
    }
    uint64_t const now = hrt_absolute_time();
    if (now - m_last_status >= 100000)
    {
        publish_status(now);
        m_last_status = now;
    }
    perf_end(m_cycle_perf);
}

void FormalEskf::process_imu(vehicle_imu_s const & sample)
{
    double const dt = double(sample.delta_angle_dt) * 1e-6;
    double const accel_dt = double(sample.delta_velocity_dt) * 1e-6;
    if (sample.timestamp_sample <= m_time || dt < 1e-6 || dt > 0.05 || accel_dt < 1e-6 || accel_dt > 0.05 ||
        !formal_eskf::px4::imu_clipping_acceptable(sample.delta_velocity_clipping, sample.delta_angle_clipping,
                                                   m_initialized))
    {
        ++m_rejections;
        // Admission failure revokes learning/health, but does not consume the timeline.
        m_bias_learning_result = {};
        m_vertical_health.reset();
        m_gravity_aiding.reset_samples();
        m_average_imu_interval = nominal_imu_interval;
        return;
    }

    imu_type const imu{vector3(double(sample.delta_velocity[0]) / accel_dt, double(sample.delta_velocity[1]) / accel_dt,
                               double(sample.delta_velocity[2]) / accel_dt),
                       vector3(double(sample.delta_angle[0]) / dt, double(sample.delta_angle[1]) / dt,
                               double(sample.delta_angle[2]) / dt)};
    uint64_t const previous_time = m_time;
    m_time = sample.timestamp_sample;
    if (!formal_eskf::linalg::all_finite(imu.specific_force_b) || !formal_eskf::linalg::all_finite(imu.angular_rate_b))
    {
        ++m_rejections;
        m_fault = m_initialized;
        return;
    }
    if (m_fault)
    {
        return;
    }
    update_gps_quality();
    if (m_fault)
    {
        return;
    }
    if (!m_initialized)
    {
        initialize(sample, imu);
    }
    else
    {
        predict_and_fuse(sample, imu, dt, accel_dt, previous_time);
        // An admitted initialized sample consumes these watermarks even on fault.
        m_previous_imu = imu;
        m_last_imu = sample;
    }
    if (m_initialized && !m_fault)
    {
        publish(imu);
    }
}

void FormalEskf::predict_and_fuse(vehicle_imu_s const & sample, imu_type const & imu, double dt, double accel_dt,
                                  uint64_t previous_time)
{
    int64_t const gap = int64_t(m_time - previous_time) - int64_t(sample.delta_angle_dt);
    if (m_time - previous_time > 50000 || gap < -200 || sample.accel_device_id != m_last_imu.accel_device_id ||
        sample.gyro_device_id != m_last_imu.gyro_device_id ||
        sample.accel_calibration_count != m_last_imu.accel_calibration_count ||
        sample.gyro_calibration_count != m_last_imu.gyro_calibration_count)
    {
        m_fault = true;
        ++m_errors;
        PX4_ERR("IMU discontinuity; restart required (gap=%lld us)", (long long)gap);
        return;
    }
    if (gap > 200)
    {
        ++m_gaps;
        m_vertical_health.reset();
        m_gravity_aiding.reset_samples();
        m_average_imu_interval = nominal_imu_interval;
        m_magnetic_aiding.reset_acceleration();
        // Synthetic intervals use the previous rate, never a fresh sensor observation.
        if (!check(formal_eskf::px4::try_predict_imu(m_state, m_covariance, m_previous_imu, double(gap) * 1e-6,
                                                     m_parameters, 0, false)))
        {
            return;
        }
    }
    if (!update_imu_bias(sample, imu) ||
        !check(formal_eskf::px4::try_predict_imu(m_state, m_covariance, imu, dt, m_parameters,
                                                 sample.delta_velocity_clipping,
                                                 m_vertical_health.state().bad_vertical_acceleration)))
    {
        return;
    }
    ++m_predictions;
    auto const acceleration_n = formal_eskf::so3::rotate(m_state.q_nb, imu.specific_force_b - m_state.b_a);
    if (!check(m_magnetic_aiding.update_acceleration(acceleration_n, accel_dt)))
    {
        return;
    }
    fuse_gps();
    // Current GNSS velocity and preceding height evidence feed the health check.
    if (m_fault || !update_imu_health(sample))
    {
        return;
    }
    fuse_height();
    if (m_fault)
    {
        return;
    }
    fuse_gravity(sample, imu);
    if (m_fault)
    {
        return;
    }
    fuse_mag();
}

void FormalEskf::fuse_gps()
{
    using namespace formal_eskf::runtime;
    using Group = GnssFusion::Group;
    GnssFusion::Conditions conditions;
    conditions.horizontal_position = {.enabled = true, .ready = m_initialized};
    conditions.velocity = {.enabled = true, .ready = m_initialized};
    // HeightAiding manages the two height sources separately from horizontal aiding.
    if (!check(m_gnss_fusion.update(m_time, conditions, gnss_fusion_parameters, m_gnss_sample)))
    {
        return;
    }
    if (m_gnss_fusion.state().restart_quality)
    {
        m_gnss_quality.restart_health();
    }
    if (!m_gnss_fusion.state().velocity.active)
    {
        m_gps_velocity_innovation = {};
    }
    if (m_gnss_sample.time_us == 0)
    {
        return;
    }
    if (!m_gps_metadata_valid || !formal_eskf::succeeded(m_gnss_quality_result.status))
    {
        m_gps_velocity_innovation = {};
        return;
    }
    const auto velocity = vector3(m_gps.vel_n_m_s, m_gps.vel_e_m_s, m_gps.vel_d_m_s);
    // v1.16.2 defaults: 0.3 m/s velocity; Down variance is 2.25x.
    const double velocity_variance = square(std::max(double(m_gps.s_variance_m_s), 0.3));
    const auto velocity_variances = vector3(velocity_variance, velocity_variance, 2.25 * velocity_variance);
    auto Rv = matrix3_type::identity() * velocity_variance;
    Rv(2, 2) = velocity_variances(2);
    auto const & session = m_gnss_fusion.state();
    auto const & health = m_vertical_health.state();
    GnssVelocityConditions<double> const velocity_conditions{health.bad_vertical_acceleration,
                                                             double(m_gps.s_variance_m_s),
                                                             m_gnss_quality_parameters.maximum_speed_accuracy};
    if (!gps_good())
    {
        // A still-active source may update health evidence without receiving fusion permission.
        if (session.velocity.active)
        {
            auto const evidence = check_gnss_velocity(m_state, m_covariance, velocity, Rv, minimum_quaternion_norm, 5.,
                                                      velocity_conditions);
            if (!check(evidence.gate.status))
            {
                return;
            }
            m_gps_velocity_innovation = {evidence.gate.diagnostics_valid, -evidence.fusion_innovation(2),
                                         evidence.gate.innovation_variance(2)};
            m_vel_ratio =
                evidence.gate.diagnostics_valid
                    ? std::max({evidence.gate.test_ratio(0), evidence.gate.test_ratio(1), evidence.gate.test_ratio(2)})
                    : double(INFINITY);
        }
        return;
    }
    float north{}, east{};
    m_projection.project(m_gps.latitude_deg, m_gps.longitude_deg, north, east);
    const auto xy = formal_eskf::linalg::Vector<backend_type, 2>::from_row_major({double(north), double(east)});
    const double position_variance = square(std::min(std::max(double(m_gps.eph), 0.5), 10.));
    const auto Rxy = formal_eskf::linalg::Matrix<backend_type, 2, 2>::identity() * position_variance;
    // Snapshot the coupled reset decision before either horizontal group advances.
    if (session.velocity.action == GnssFusion::Action::reset)
    {
        vector3_type delta;
        auto const status =
            try_reset_velocity(m_state, m_covariance, velocity, velocity_variances, m_state, m_covariance, delta);
        if (!check(
                m_gnss_fusion.record_reset(Group::velocity, m_gnss_sample.time_us, formal_eskf::succeeded(status))) ||
            !check(status))
        {
            return;
        }
        m_velocity_reset_delta = delta;
        ++m_velocity_reset_counter;
        record_vertical_velocity_reset(delta(2));
        ++m_odometry_reset_counter;
        m_vel_ratio = 0.;
        m_gps_velocity_innovation = {true, 0., velocity_variances(2)};
    }
    else if (session.velocity.action == GnssFusion::Action::fuse)
    {
        auto const result =
            fuse_gnss_velocity(m_state, m_covariance, velocity, Rv, minimum_quaternion_norm, 5., velocity_conditions,
                               m_state, m_covariance, m_bias_learning_result.bias_update);
        if (!record_gps_fusion(Group::velocity, result.fusion, m_vel_ratio))
        {
            return;
        }
        m_gps_velocity_innovation = {result.fusion.diagnostics_valid, -result.fusion_innovation(2),
                                     result.fusion.innovation_variance(2)};
        // Use preceding height progress, after normal velocity correction and before height control.
        if (m_height_aiding.reset_required(m_time, {health.last_update_time_us, health.last_good_time_us}))
        {
            double delta{};
            if (!check(try_reset_vertical_velocity(m_state, m_covariance, velocity(2), velocity_variances(2), m_state,
                                                   m_covariance, delta)))
            {
                return;
            }
            record_vertical_velocity_reset(delta);
            ++m_odometry_reset_counter;
            // A Down-only reset preserves this attempt's selected velocity diagnostics.
        }
    }
    if (session.horizontal_position.action == GnssFusion::Action::reset)
    {
        formal_eskf::linalg::Vector<backend_type, 2> delta;
        auto const variances =
            formal_eskf::linalg::Vector<backend_type, 2>::from_row_major({position_variance, position_variance});
        auto const status =
            try_reset_gnss_horizontal_position(m_state, m_covariance, xy, variances, m_state, m_covariance, delta);
        if (!check(m_gnss_fusion.record_reset(Group::horizontal_position, m_gnss_sample.time_us,
                                              formal_eskf::succeeded(status))) ||
            !check(status))
        {
            return;
        }
        m_position_reset_delta = delta;
        ++m_position_reset_counter;
        ++m_odometry_reset_counter;
        m_pos_ratio = 0.;
    }
    else if (session.horizontal_position.action == GnssFusion::Action::fuse)
    {
        auto const result = fuse_horizontal_position(m_state, m_covariance, xy, Rxy, minimum_quaternion_norm, 5.,
                                                     m_state, m_covariance, m_bias_learning_result.bias_update);
        if (!record_gps_fusion(Group::horizontal_position, result, m_pos_ratio))
        {
            return;
        }
    }
    m_max_fusion_age = std::max(m_max_fusion_age, m_time - m_gnss_sample.time_us);
}

void FormalEskf::fuse_gravity(vehicle_imu_s const & sample, imu_type const & imu)
{
    auto const & gnss = m_gnss_fusion.state();
    formal_eskf::px4::GravityAiding<backend_type>::Context const context{
        m_gravity_enabled, recent(m_land.timestamp, 2000000) && m_land.at_rest,
        gnss.horizontal_position.active || gnss.velocity.active};
    auto const result = m_gravity_aiding.update(imu, sample.delta_angle_dt, sample.delta_velocity_dt,
                                                sample.delta_velocity_clipping, context, m_parameters.gravity_n,
                                                m_state, m_covariance, m_bias_learning_result.bias_update);
    if (!check(result.status) || !result.ready)
    {
        return;
    }
    ++m_gravity_samples;
    m_gravity_interval = result.interval_us;
    m_gravity_updates += result.fusion.decision == formal_eskf::runtime::FusionDecision::fused;
}

bool FormalEskf::magnetic_heading_valid() const
{
    return m_magnetic_aiding.heading_aiding() && recent(m_magnetic_aiding.progress_time()) && m_covariance(8, 8) < 0.1;
}

std::optional<FormalEskf::magnetic_model_type::sample_type> FormalEskf::lookup_magnetic_model(double latitude,
                                                                                              double longitude) const
{
    if (!std::isfinite(latitude) || !std::isfinite(longitude) || std::abs(latitude) > 90. || std::abs(longitude) > 180.)
    {
        return std::nullopt;
    }
    magnetic_model_type::sample_type const local{
        math::radians(double(get_mag_declination_degrees(latitude, longitude))),
        math::radians(double(get_mag_inclination_degrees(latitude, longitude))),
        100. * double(get_mag_strength_gauss(latitude, longitude))};
    magnetic_model_type::sample_type model;
    double const origin_lat = m_projection.isInitialized() ? m_projection.getProjectionReferenceLat() : latitude;
    double const origin_lon = m_projection.isInitialized() ? m_projection.getProjectionReferenceLon() : longitude;
    if (!formal_eskf::succeeded(
            magnetic_model_type::to_origin_frame(local, latitude, longitude, origin_lat, origin_lon, model)))
    {
        return std::nullopt;
    }
    return model;
}

void FormalEskf::update_magnetic_model()
{
    bool const position_valid = gps_position_valid();
    bool const origin_updated = m_origin_time > m_mag_model_origin_time;
    if (!m_magnetic_model.lookup_due(m_time, position_valid, origin_updated))
    {
        return;
    }
    double latitude = m_projection.getProjectionReferenceLat();
    double longitude = m_projection.getProjectionReferenceLon();
    if (position_valid)
    {
        m_projection.reproject(m_state.p_n(0), m_state.p_n(1), latitude, longitude);
    }
    auto const model = lookup_magnetic_model(latitude, longitude);
    m_mag_model_unavailable += !model.has_value();
    m_mag_model_origin_time = m_origin_time;
    auto const result = m_magnetic_model.update(m_time, model, position_valid, origin_updated);
    check(result.status);
}

void FormalEskf::fuse_mag()
{
    using FlightState = magnetic_type::lifecycle_type::FlightState;
    magnetic_type::Context context;
    // Activity is distinct from the 5 s output-validity and 1 s dead-reckoning flags.
    context.horizontal_aiding =
        m_gnss_fusion.state().horizontal_position.active || m_gnss_fusion.state().velocity.active;
    uint64_t const now = hrt_absolute_time();
    if (m_vehicle_status.timestamp != 0 && m_vehicle_status.timestamp <= now &&
        now - m_vehicle_status.timestamp < 3000000)
    {
        context.flight_state = m_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
                                   ? FlightState::in_air
                                   : FlightState::on_ground;
    }
    if (m_land.timestamp != 0 && m_land.timestamp <= now && now - m_land.timestamp < 3000000)
    {
        context.flight_state = m_land.landed ? FlightState::on_ground : FlightState::in_air;
        context.at_rest = m_land.at_rest;
    }
    magnetic_type::Sample sample;
    if (m_mag.timestamp_sample > m_mag_seen && m_mag.timestamp_sample <= m_time)
    {
        m_mag_seen = m_mag.timestamp_sample;
        sample = {m_mag.timestamp_sample,
                  {m_mag.device_id, m_mag.calibration_count, 0U},
                  vector3(100. * double(m_mag.magnetometer_ga[0]), 100. * double(m_mag.magnetometer_ga[1]),
                          100. * double(m_mag.magnetometer_ga[2]))};
        if (m_time - sample.time_us < magnetic_type::timing.maximum_interval_us && sample.source.device_id != 0U &&
            formal_eskf::linalg::all_finite(sample.field_b) &&
            formal_eskf::linalg::dot(sample.field_b, sample.field_b) > 0.)
        {
            update_magnetic_model();
        }
        sample.source.reference_id = m_magnetic_model.reference_id();
    }
    if (m_fault)
    {
        return;
    }
    magnetic_type::Reference const reference{m_magnetic_model.observation_n(), m_magnetic_model.health_n(),
                                             m_magnetic_model.available()};
    auto const result = m_magnetic_aiding.update(m_time, sample, context, reference, m_state, m_covariance,
                                                 m_bias_learning_result.bias_update);
    if (!check(result.status))
    {
        return;
    }
    m_mag_updates += result.fused;
    m_rejections += result.rejected;
    m_odometry_reset_counter += result.reset;
    if (sample.time_us != 0)
    {
        auto const & fusion = m_magnetic_aiding.observation().fusion;
        m_mag_ratio = fusion.diagnostics_valid ? formal_eskf::linalg::max_abs(fusion.test_ratio) : double(INFINITY);
    }
}

void FormalEskf::record_vertical_velocity_reset(double delta)
{
    m_vertical_velocity_reset_delta =
        m_vertical_velocity_reset_time == m_time ? m_vertical_velocity_reset_delta + delta : delta;
    m_vertical_velocity_reset_time = m_time;
    ++m_vertical_velocity_reset_counter;
}

void FormalEskf::fuse_height()
{
    using Source = height_type::Source;
    height_type::Sample const barometer{m_baro.timestamp_sample, m_baro.baro_device_id, m_baro.calibration_count,
                                        double(m_baro.baro_alt_meter), height_type::barometer_variance};
    height_type::Sample gnss;
    if (m_gnss_sample.time_us != 0)
    {
        gnss = {m_gnss_sample.time_us,
                m_gps.device_id,
                0,
                m_gps.altitude_msl_m,
                square(std::max(double(m_gps.epv), 0.75)),
                gps_good()};
    }
    auto const & health = m_vertical_health.state();
    height_type::Acceleration const acceleration{health.bad_vertical_acceleration,
                                                 {health.last_update_time_us, health.last_good_time_us}};
    auto const result = m_height_aiding.update(m_time, barometer, gnss, m_gnss_fusion.state().velocity.active, m_state,
                                               m_covariance, m_bias_learning_result.bias_update, acceleration);
    if (!check(result.status))
    {
        return;
    }
    m_rejections += result.rejected;
    m_odometry_reset_counter += result.height_resets + result.velocity_resets;
    if (result.velocity_resets != 0)
    {
        record_vertical_velocity_reset(result.velocity_delta);
    }
    auto const & baro = m_height_aiding.session(Source::barometer);
    auto const & gps = m_height_aiding.session(Source::gnss);
    m_baro_updates += baro.fused_time == m_time;
    m_gps_updates += gps.fused_time == m_time;
    m_height_ratio = baro.fusion.diagnostics_valid ? baro.fusion.test_ratio(0) : double(INFINITY);
    m_gps_height_ratio = gps.fusion.diagnostics_valid ? gps.fusion.test_ratio(0) : double(INFINITY);
}

bool FormalEskf::update_imu_bias(vehicle_imu_s const & sample, imu_type const & imu)
{
    const double angular_interval = double(sample.delta_angle_dt) * 1e-6;
    const double force_interval = double(sample.delta_velocity_dt) * 1e-6;
    m_average_imu_interval =
        0.99 * m_average_imu_interval + 0.01 * std::clamp(0.5 * (angular_interval + force_interval),
                                                          0.5 * nominal_imu_interval, 2. * nominal_imu_interval);
    bias_learning_type::Conditions conditions;
    conditions.at_rest = recent(m_land.timestamp, 2000000) && m_land.at_rest;
    conditions.accelerometer_clipping = sample.delta_velocity_clipping;
    conditions.bad_vertical_acceleration = m_vertical_health.state().bad_vertical_acceleration;
    // This module has no fake-position/height modes. A GNSS outage is not a fake-aiding mode.
    m_bias_learning_result = m_bias_learning.update(angular_interval, force_interval, m_state, imu, conditions);
    return check(m_bias_learning_result.status);
}

bool FormalEskf::update_imu_health(vehicle_imu_s const & sample)
{
    vertical_health_type::Observations observations;
    observations.barometer_height = m_height_aiding.vertical_innovation(height_type::Source::barometer);
    observations.gnss_height = m_height_aiding.vertical_innovation(height_type::Source::gnss);
    observations.gnss_velocity_down = m_gps_velocity_innovation;
    observations.gnss_velocity_down.active &= m_gnss_fusion.state().velocity.active;
    vertical_health_type::Sample health_sample;
    health_sample.time_us = m_time;
    health_sample.average_interval_s = m_average_imu_interval;
    health_sample.clipping_available = true;
    for (size_t axis = 0; axis < 3; ++axis)
    {
        health_sample.clipped[axis] = (sample.delta_velocity_clipping & (1U << axis)) != 0;
    }
    // Current GNSS velocity, preceding height attempts: evaluate before current height correction.
    return check(m_vertical_health.update(health_sample, observations));
}

void FormalEskf::publish(imu_type const & imu)
{
    const uint64_t now = hrt_absolute_time();
    const bool nav_valid = (gps_position_valid() || gps_velocity_valid()) && now < m_time + 100000;
    const bool heading_valid = magnetic_heading_valid();
    vehicle_attitude_s attitude{};
    attitude.timestamp_sample = m_time;
    attitude.timestamp = now;
    for (size_t i = 0; i < 4; ++i)
    {
        attitude.q[i] = float(m_state.q_nb.coefficients()(i));
        attitude.delta_q_reset[i] = float(m_magnetic_aiding.reset_delta().coefficients()(i));
    }
    attitude.quat_reset_counter = m_magnetic_aiding.reset_counter();
    m_attitude_pub.publish(attitude);
    if (m_time - m_last_output < 10000)
    {
        return;
    } // Navigation/odometry <= 100 Hz.
    m_last_output = m_time;
    vehicle_local_position_s local{};
    local.timestamp_sample = m_time;
    local.timestamp = now;
    local.x = m_state.p_n(0);
    local.y = m_state.p_n(1);
    local.z = m_state.p_n(2);
    local.vx = m_state.v_n(0);
    local.vy = m_state.v_n(1);
    local.vz = m_state.v_n(2);
    local.z_deriv = local.vz;
    const auto accel_n =
        formal_eskf::so3::rotate(m_state.q_nb, imu.specific_force_b - m_state.b_a) + m_parameters.gravity_n;
    local.ax = accel_n(0);
    local.ay = accel_n(1);
    local.az = accel_n(2);
    local.xy_valid = local.v_xy_valid = nav_valid;
    local.z_valid = m_height_aiding.valid(m_time);
    local.v_z_valid = gps_velocity_valid() || local.z_valid;
    local.delta_z = m_height_aiding.reset_delta();
    local.z_reset_counter = m_height_aiding.reset_counter();
    for (size_t axis = 0; axis < 2; ++axis)
    {
        local.delta_xy[axis] = m_position_reset_delta(axis);
        local.delta_vxy[axis] = m_velocity_reset_delta(axis);
    }
    local.delta_vz = m_vertical_velocity_reset_delta;
    local.xy_reset_counter = m_position_reset_counter;
    local.vxy_reset_counter = m_velocity_reset_counter;
    local.vz_reset_counter = m_vertical_velocity_reset_counter;
    local.heading = matrix::Eulerf(matrix::Quatf(attitude.q)).psi();
    local.unaided_heading = NAN; // No separate gyro-only heading estimator.
    local.heading_var = m_covariance(8, 8);
    local.tilt_var = m_covariance(6, 6) + m_covariance(7, 7);
    local.heading_good_for_control = heading_valid;
    auto const & delta = m_magnetic_aiding.reset_delta().coefficients();
    local.delta_heading = 2. * std::atan2(delta(3), delta(0));
    local.heading_reset_counter = m_magnetic_aiding.reset_counter();
    local.xy_global = local.z_global = true;
    local.ref_timestamp = m_origin_time;
    local.ref_lat = m_projection.getProjectionReferenceLat();
    local.ref_lon = m_projection.getProjectionReferenceLon();
    local.ref_alt = m_origin_alt;
    local.eph = std::sqrt(std::max(m_covariance(0, 0), m_covariance(1, 1)));
    local.epv = std::sqrt(m_covariance(2, 2));
    local.evh = std::sqrt(std::max(m_covariance(3, 3), m_covariance(4, 4)));
    local.evv = std::sqrt(m_covariance(5, 5));
    local.dead_reckoning = !(gps_position_valid(1000000) || gps_velocity_valid(1000000));
    local.dist_bottom = NAN;
    local.dist_bottom_var = NAN; // No terrain estimate.
    local.vxy_max = local.vz_max = local.hagl_min = local.hagl_max_z = local.hagl_max_xy = INFINITY;
    m_local_pub.publish(local);
    vehicle_global_position_s global{};
    global.timestamp_sample = m_time;
    global.timestamp = now;
    m_projection.reproject(local.x, local.y, global.lat, global.lon);
    global.alt = m_origin_alt - m_state.p_n(2);
    global.alt_ellipsoid = m_origin_ellipsoid - m_state.p_n(2);
    global.lat_lon_valid = local.xy_valid;
    global.lat_lon_reset_counter = m_position_reset_counter;
    global.alt_valid = local.z_valid;
    global.delta_alt = -local.delta_z;
    global.alt_reset_counter = local.z_reset_counter;
    global.eph = local.eph;
    global.epv = local.epv;
    global.dead_reckoning = local.dead_reckoning;
    global.terrain_alt = NAN;
    m_global_pub.publish(global);
    vehicle_odometry_s odometry{};
    odometry.timestamp_sample = m_time;
    odometry.timestamp = now;
    odometry.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
    odometry.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
    for (size_t i = 0; i < 4; ++i)
    {
        odometry.q[i] = attitude.q[i];
    }
    for (size_t i = 0; i < 3; ++i)
    {
        odometry.position[i] = m_state.p_n(i);
        odometry.velocity[i] = m_state.v_n(i);
        odometry.angular_velocity[i] = imu.angular_rate_b(i) - m_state.b_g(i);
        odometry.position_variance[i] = m_covariance(i, i);
        odometry.velocity_variance[i] = m_covariance(i + 3, i + 3);
        odometry.orientation_variance[i] = m_covariance(i + 6, i + 6);
    }
    odometry.reset_counter = m_odometry_reset_counter;
    odometry.quality = nav_valid && heading_valid ? 100 : 0;
    m_odometry_pub.publish(odometry);
}

void FormalEskf::publish_status(uint64_t now)
{
    const bool healthy = m_initialized && !m_fault && now < m_time + 100000;
    const bool gps_pos_valid = healthy && gps_position_valid();
    const bool gps_vel_valid = healthy && gps_velocity_valid();
    const bool gps_valid = gps_pos_valid || gps_vel_valid;
    const bool gps_height_active = healthy && m_height_aiding.session(height_type::Source::gnss).active;
    const bool mag_valid = healthy && magnetic_heading_valid();
    const bool mag_aiding = healthy && m_magnetic_aiding.heading_aiding();
    const bool baro_active = healthy && m_height_aiding.session(height_type::Source::barometer).active;
    estimator_status_s status{};
    status.timestamp = now;
    status.timestamp_sample = m_time;
    if (healthy)
    {
        if (m_gravity_aiding.tilt_aligned())
        {
            status.control_mode_flags |= (1ULL << status.CS_TILT_ALIGN);
        }
        if (m_magnetic_aiding.aligned())
        {
            status.control_mode_flags |= (1ULL << status.CS_YAW_ALIGN);
        }
    }
    if (mag_aiding)
    {
        status.control_mode_flags |= (1ULL << status.CS_MAG_HDG);
    }
    if (healthy && m_gnss_fusion.state().horizontal_position.active)
    {
        status.control_mode_flags |= (1ULL << status.CS_GNSS_POS);
    }
    if (healthy && m_gnss_fusion.state().velocity.active)
    {
        status.control_mode_flags |= (1ULL << status.CS_GNSS_VEL);
    }
    if (gps_height_active)
    {
        status.control_mode_flags |= (1ULL << status.CS_GPS_HGT);
    }
    if (baro_active)
    {
        status.control_mode_flags |= (1ULL << status.CS_BARO_HGT);
    }
    if (!m_land.landed)
    {
        status.control_mode_flags |= (1ULL << status.CS_IN_AIR);
    }
    status.gps_check_fail_flags = gps_failure_flags();
    status.reset_count_quat = m_magnetic_aiding.reset_counter();
    status.reset_count_pos_ne = m_position_reset_counter;
    status.reset_count_vel_ne = m_velocity_reset_counter;
    status.reset_count_vel_d = m_vertical_velocity_reset_counter;
    status.reset_count_pod_d = m_height_aiding.reset_counter();
    status.filter_fault_flags = (m_fault || m_vertical_health.state().bad_vertical_acceleration) ? (1U << 16) : 0U;
    status.hdg_test_ratio = m_mag_ratio;
    status.vel_test_ratio = m_vel_ratio;
    status.pos_test_ratio = m_pos_ratio;
    status.hgt_test_ratio =
        m_height_aiding.reference() == height_type::Source::gnss ? m_gps_height_ratio : m_height_ratio;
    status.pre_flt_fail_innov_heading = !mag_valid || m_mag_ratio > 1.;
    status.pre_flt_fail_innov_vel_horiz = status.pre_flt_fail_innov_vel_vert = !gps_vel_valid || m_vel_ratio > 1.;
    status.pre_flt_fail_innov_pos_horiz = !gps_pos_valid || m_pos_ratio > 1.;
    status.pre_flt_fail_innov_height =
        !m_height_aiding.valid(m_time) ||
        !((baro_active && m_height_ratio <= 1.) || (gps_height_active && m_gps_height_ratio <= 1.));
    status.pos_horiz_accuracy = std::sqrt(std::max(m_covariance(0, 0), m_covariance(1, 1)));
    status.pos_vert_accuracy = std::sqrt(m_covariance(2, 2));
    status.solution_status_flags = healthy ? 1 : 0;
    if (gps_valid)
    {
        status.solution_status_flags |= 0x3e;
    }
    status.accel_device_id = m_last_imu.accel_device_id;
    status.gyro_device_id = m_last_imu.gyro_device_id;
    status.mag_device_id = m_mag.device_id;
    status.baro_device_id = m_baro.baro_device_id;
    m_status_pub.publish(status);
    estimator_status_flags_s flags{};
    flags.timestamp = now;
    flags.timestamp_sample = m_time;
    flags.cs_tilt_align = healthy && m_gravity_aiding.tilt_aligned();
    flags.cs_gravity_vector = healthy && m_gravity_aiding.active();
    flags.cs_yaw_align = healthy && m_magnetic_aiding.aligned();
    flags.cs_gnss_pos = healthy && m_gnss_fusion.state().horizontal_position.active;
    flags.cs_gnss_vel = healthy && m_gnss_fusion.state().velocity.active;
    flags.cs_gps_hgt = gps_height_active;
    flags.cs_mag = healthy && m_magnetic_aiding.lifecycle().active;
    flags.cs_mag_hdg = mag_aiding;
    flags.cs_mag_heading_consistent = healthy && m_magnetic_aiding.heading().consistent;
    auto const & magnetic_field = m_magnetic_aiding.observation().field;
    flags.cs_mag_field_disturbed =
        magnetic_field.diagnostics_valid && (magnetic_field.failures.strength || magnetic_field.failures.inclination);
    flags.cs_baro_hgt = baro_active;
    flags.cs_baro_fault = m_height_aiding.session(height_type::Source::barometer).fault;
    flags.cs_in_air = !m_land.landed;
    flags.cs_inertial_dead_reckoning = !(healthy && (gps_position_valid(1000000) || gps_velocity_valid(1000000)));
    flags.fs_bad_acc_vertical = m_vertical_health.state().bad_vertical_acceleration;
    flags.reject_hor_pos = m_pos_ratio > 1.;
    flags.reject_hor_vel = flags.reject_ver_vel = m_vel_ratio > 1.;
    flags.reject_ver_pos = status.hgt_test_ratio > 1.f;
    flags.reject_yaw = m_magnetic_aiding.observation().fusion.diagnostics_valid && m_mag_ratio > 1.;
    m_flags_pub.publish(flags);
    if (!m_initialized)
    {
        return;
    }
    estimator_sensor_bias_s bias{};
    bias.timestamp = now;
    bias.timestamp_sample = m_time;
    bias.accel_device_id = m_last_imu.accel_device_id;
    bias.gyro_device_id = m_last_imu.gyro_device_id;
    bias.accel_bias_valid = bias.gyro_bias_valid = healthy;
    bias.accel_bias_limit = formal_eskf::px4::imu_bias_limits.accelerometer;
    bias.gyro_bias_limit = formal_eskf::px4::imu_bias_limits.gyroscope;
    for (size_t i = 0; i < 3; ++i)
    {
        bias.accel_bias[i] = m_state.b_a(i);
        bias.gyro_bias[i] = m_state.b_g(i);
        bias.accel_bias_variance[i] = m_covariance(i + 9, i + 9);
        bias.gyro_bias_variance[i] = m_covariance(i + 12, i + 12);
    }
    m_bias_pub.publish(bias);
}

int FormalEskf::task_spawn(int argc, char * argv[])
{
    FormalEskf * instance = new FormalEskf();
    if (instance)
    {
        _object.store(instance);
        _task_id = task_id_is_work_queue;
        if (instance->init())
        {
            return PX4_OK;
        }
    }
    delete instance;
    _object.store(nullptr);
    _task_id = -1;
    return PX4_ERROR;
}

int FormalEskf::print_status()
{
    std::lock_guard<std::mutex> guard(m_state_mutex);
    PX4_INFO("formal-eskf INS / PX4 matrix / binary64 / C++20 / single instance");
    PX4_INFO("initialized=%d fault=%d predictions=%u gps=%u mag=%u baro=%u", m_initialized, m_fault, m_predictions,
             m_gps_updates, m_mag_updates, m_baro_updates);
    PX4_INFO("gnss_quality=%d qualified=%d metadata_valid=%d fail_flags=0x%x pos_active=%d vel_active=%d resets=%u/%u",
             int(m_gnss_quality_result.decision), m_gnss_quality_result.qualified, m_gps_metadata_valid,
             unsigned(gps_failure_flags()), m_gnss_fusion.state().horizontal_position.active,
             m_gnss_fusion.state().velocity.active, unsigned(m_position_reset_counter),
             unsigned(m_velocity_reset_counter));
    PX4_INFO("rejections=%u numerical_errors=%u gap_holds=%u max_gps_age_us=%llu", m_rejections, m_errors, m_gaps,
             (unsigned long long)m_max_fusion_age);
    PX4_INFO("p=[%.3f %.3f %.3f] v=[%.3f %.3f %.3f]", m_state.p_n(0), m_state.p_n(1), m_state.p_n(2), m_state.v_n(0),
             m_state.v_n(1), m_state.v_n(2));
    PX4_INFO("mag: active=%u aligned=%u consistent=%u stop=%u recovery=%u resets=%u",
             unsigned(m_magnetic_aiding.lifecycle().active), unsigned(m_magnetic_aiding.aligned()),
             unsigned(m_magnetic_aiding.heading().consistent), unsigned(m_magnetic_aiding.lifecycle().stop_reason),
             unsigned(m_magnetic_aiding.lifecycle().recovery_reason), unsigned(m_magnetic_aiding.reset_counter()));
    PX4_INFO("mag model: available=%u checks=%llu model_revision=%u heading_revision=%u source=%u unavailable=%u",
             unsigned(m_magnetic_model.available()), (unsigned long long)m_magnetic_model.lookup_count(),
             unsigned(m_magnetic_model.model_revision()), unsigned(m_magnetic_model.reference_id()),
             unsigned(m_magnetic_model.heading_source()), unsigned(m_mag_model_unavailable));
    PX4_INFO("mag declination: %.3f deg", math::degrees(m_magnetic_model.declination()));
    auto const & baro = m_height_aiding.session(height_type::Source::barometer);
    auto const & gps = m_height_aiding.session(height_type::Source::gnss);
    PX4_INFO("height: ref=%u baro=%u gnss=%u baro_fault=%u resets=%u vz_resets=%u",
             unsigned(m_height_aiding.reference()), unsigned(baro.active), unsigned(gps.active), unsigned(baro.fault),
             unsigned(m_height_aiding.reset_counter()), unsigned(m_vertical_velocity_reset_counter));
    PX4_INFO("height bias: baro=%.3f gnss=%.3f m", baro.bias.value, gps.bias.value);
    unsigned accel_mask{}, gyro_mask{};
    for (size_t axis = 0; axis < 3; ++axis)
    {
        accel_mask |= unsigned(m_bias_learning_result.bias_update.accelerometer[axis]) << axis;
        gyro_mask |= unsigned(m_bias_learning_result.bias_update.gyroscope[axis]) << axis;
    }
    PX4_INFO("bias learning: accel=%u gyro=%u high_dynamics=%u", accel_mask, gyro_mask,
             unsigned(m_bias_learning_result.high_dynamics));
    PX4_INFO("vertical IMU health: available=%u bad=%u", unsigned(m_vertical_health.state().last_update_time_us != 0),
             unsigned(m_vertical_health.state().bad_vertical_acceleration));
    PX4_INFO("gravity: enabled=%u tilt=%u active=%u samples=%u fused=%u interval_us=%llu", unsigned(m_gravity_enabled),
             unsigned(m_gravity_aiding.tilt_aligned()), unsigned(m_gravity_aiding.active()), m_gravity_samples,
             m_gravity_updates, (unsigned long long)m_gravity_interval);
    perf_print_counter(m_cycle_perf);
    return 0;
}

int FormalEskf::custom_command(int argc, char * argv[]) { return print_usage("unknown command"); }

int FormalEskf::print_usage(char const * reason)
{
    if (reason)
    {
        PX4_WARN("%s", reason);
    }
    PRINT_MODULE_DESCRIPTION(
        "Single-instance formal-eskf INS adapter. SITL POC: IMU, GNSS, barometer and magnetometer.");
    PRINT_MODULE_USAGE_NAME("formal_eskf", "estimator");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

extern "C" __EXPORT int formal_eskf_main(int argc, char * argv[]) { return FormalEskf::main(argc, argv); }
