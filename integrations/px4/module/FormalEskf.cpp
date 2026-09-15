// SPDX-License-Identifier: BSD-3-Clause
#include <algorithm>
#include <cmath>
#include "FormalEskf.hpp"
#include <lib/world_magnetic_model/geo_mag_declination.h>

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

bool FormalEskf::gps_good() const
{
    return recent(m_gps.timestamp_sample, 500000) && m_gps.fix_type >= 3 && m_gps.satellites_used >= 6 &&
           m_gps.vel_ned_valid && std::isfinite(m_gps.latitude_deg) && std::isfinite(m_gps.longitude_deg) &&
           std::abs(m_gps.latitude_deg) <= 90. && std::abs(m_gps.longitude_deg) <= 180. &&
           std::isfinite(m_gps.altitude_msl_m) && std::isfinite(m_gps.altitude_ellipsoid_m) &&
           std::isfinite(m_gps.vel_n_m_s) && std::isfinite(m_gps.vel_e_m_s) && std::isfinite(m_gps.vel_d_m_s) &&
           std::isfinite(m_gps.eph) && m_gps.eph > 0.f && m_gps.eph < 5.f && std::isfinite(m_gps.epv) &&
           m_gps.epv > 0.f && m_gps.epv < 8.f && std::isfinite(m_gps.s_variance_m_s) && m_gps.s_variance_m_s > 0.f &&
           m_gps.s_variance_m_s < 2.f && m_gps.jamming_state != sensor_gps_s::JAMMING_STATE_CRITICAL &&
           m_gps.spoofing_state < sensor_gps_s::SPOOFING_STATE_INDICATED;
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
        const auto m = vector3(m_mag.magnetometer_ga[0], m_mag.magnetometer_ga[1], m_mag.magnetometer_ga[2]);
        if (formal_eskf::linalg::all_finite(m) && formal_eskf::linalg::norm(m) > 0.1)
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

    const auto accel = m_accel_sum / double(m_init_samples);
    const auto mag = m_mag_sum / double(m_init_mag_samples);
    const double roll = std::atan2(-accel(1), -accel(2));
    const double pitch = std::atan2(accel(0), std::hypot(accel(1), accel(2)));
    const matrix::Dcm<double> tilt{matrix::Euler<double>(roll, pitch, 0.)};
    const matrix::Vector3<double> level_mag = tilt * matrix::Vector3<double>(mag(0), mag(1), mag(2));
    const double decl = math::radians(get_mag_declination_degrees(m_gps.latitude_deg, m_gps.longitude_deg));
    const double incl = math::radians(get_mag_inclination_degrees(m_gps.latitude_deg, m_gps.longitude_deg));
    const double strength = 100. * double(get_mag_strength_gauss(m_gps.latitude_deg, m_gps.longitude_deg));
    const double yaw = decl - std::atan2(level_mag(1), level_mag(0));
    const matrix::Quaternion<double> q{matrix::Euler<double>(roll, pitch, yaw)};
    if (!check(state_type::quaternion_type::try_from_coefficients(q(0), q(1), q(2), q(3), minimum_quaternion_norm,
                                                                  m_state.q_nb)))
    {
        return;
    }
    m_state.b_g = m_gyro_sum / double(m_init_samples);
    m_state.v_n = vector3(m_gps.vel_n_m_s, m_gps.vel_e_m_s, m_gps.vel_d_m_s);
    m_mag_n = vector3(strength * std::cos(incl) * std::cos(decl), strength * std::cos(incl) * std::sin(decl),
                      strength * std::sin(incl));
    const double diagonal[15] = {1., 1., 2.25, 0.09, 0.09, 0.09, 0.01, 0.01, 0.04, 0.04, 0.04, 0.04, 1e-4, 1e-4, 1e-4};
    for (size_t i = 0; i < 15; ++i)
    {
        m_covariance.set(i, i, diagonal[i]);
    }
    m_origin_time = m_time;
    m_projection.initReference(m_gps.latitude_deg, m_gps.longitude_deg, m_origin_time);
    m_origin_alt = m_gps.altitude_msl_m;
    m_origin_ellipsoid = m_gps.altitude_ellipsoid_m;
    m_baro_origin = m_baro.baro_alt_meter;
    m_gps_seen = m_gps_fused = m_gps.timestamp_sample;
    m_mag_seen = m_mag_fused = m_mag.timestamp_sample;
    m_baro_seen = m_baro_fused = m_baro.timestamp_sample;
    m_last_imu = sample;
    m_previous_imu = imu;
    m_initialized = true;
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
        const double dt = double(sample.delta_angle_dt) * 1e-6;
        const double accel_dt = double(sample.delta_velocity_dt) * 1e-6;
        if (sample.timestamp_sample <= m_time || dt < 1e-6 || dt > 0.05 || accel_dt < 1e-6 || accel_dt > 0.05 ||
            sample.delta_angle_clipping || sample.delta_velocity_clipping)
        {
            ++m_rejections;
        }
        else
        {
            const imu_type imu{vector3(double(sample.delta_velocity[0]) / accel_dt,
                                       double(sample.delta_velocity[1]) / accel_dt,
                                       double(sample.delta_velocity[2]) / accel_dt),
                               vector3(double(sample.delta_angle[0]) / dt, double(sample.delta_angle[1]) / dt,
                                       double(sample.delta_angle[2]) / dt)};
            const uint64_t previous_time = m_time;
            m_time = sample.timestamp_sample;
            if (!formal_eskf::linalg::all_finite(imu.specific_force_b) ||
                !formal_eskf::linalg::all_finite(imu.angular_rate_b))
            {
                ++m_rejections;
                m_fault = m_initialized;
            }
            else if (!m_initialized && !m_fault)
            {
                initialize(sample, imu);
            }
            else if (!m_fault)
            {
                const int64_t gap = int64_t(m_time - previous_time) - int64_t(sample.delta_angle_dt);
                if (m_time - previous_time > 50000 || gap < -200 ||
                    sample.accel_device_id != m_last_imu.accel_device_id ||
                    sample.gyro_device_id != m_last_imu.gyro_device_id ||
                    sample.accel_calibration_count != m_last_imu.accel_calibration_count ||
                    sample.gyro_calibration_count != m_last_imu.gyro_calibration_count)
                {
                    m_fault = true;
                    ++m_errors;
                    PX4_ERR("IMU discontinuity; restart required (gap=%lld us)", (long long)gap);
                }
                else
                {
                    // Small missing intervals use the PREVIOUS measured rate explicitly.
                    if (gap > 200)
                    {
                        ++m_gaps;
                        check(formal_eskf::try_predict(m_state, m_covariance, m_previous_imu, double(gap) * 1e-6,
                                                       m_parameters, m_state, m_covariance));
                    }
                    if (!m_fault && check(formal_eskf::try_predict(m_state, m_covariance, imu, dt, m_parameters,
                                                                   m_state, m_covariance)))
                    {
                        ++m_predictions;
                        fuse_gps();
                        if (!m_fault)
                        {
                            fuse_baro();
                        }
                        if (!m_fault)
                        {
                            fuse_mag();
                        }
                    }
                }
                m_previous_imu = imu;
                m_last_imu = sample;
            }
            if (m_initialized && !m_fault)
            {
                publish(imu);
            }
        }
    }
    const uint64_t now = hrt_absolute_time();
    if (now - m_last_status >= 100000)
    {
        publish_status(now);
        m_last_status = now;
    }
    perf_end(m_cycle_perf);
}

void FormalEskf::fuse_gps()
{
    if (m_gps.timestamp_sample <= m_gps_seen || m_gps.timestamp_sample > m_time)
    {
        return;
    }
    m_gps_seen = m_gps.timestamp_sample;
    if (!gps_good())
    {
        ++m_rejections;
        return;
    }
    float north{}, east{};
    m_projection.project(m_gps.latitude_deg, m_gps.longitude_deg, north, east);
    const auto xy = backend_type::vector_type<2>::from_row_major({double(north), double(east)});
    const double z = m_origin_alt - m_gps.altitude_msl_m;
    const auto velocity = vector3(m_gps.vel_n_m_s, m_gps.vel_e_m_s, m_gps.vel_d_m_s);
    const auto Rxy = backend_type::matrix_type<2, 2>::identity() * square(std::max(double(m_gps.eph), 1.));
    const double Rz = square(std::max(double(m_gps.epv), 1.5));
    const auto Rv = matrix3_type::identity() * square(std::max(double(m_gps.s_variance_m_s), 0.2));
    const auto Hxy = formal_eskf::horizontal_position_jacobian<backend_type>();
    m_pos_ratio = gate(xy - backend_type::vector_type<2>::from_row_major({m_state.p_n(0), m_state.p_n(1)}), Hxy, Rxy);
    m_vel_ratio = gate(velocity - m_state.v_n, formal_eskf::velocity_jacobian<backend_type>(), Rv);
    const double height_ratio = square(z - m_state.p_n(2)) / (25. * (m_covariance(2, 2) + Rz));
    if (m_pos_ratio > 1. || m_vel_ratio > 1. || height_ratio > 1.)
    {
        ++m_rejections;
        return;
    }
    // Commit an entire GNSS event atomically, even if a later correction fails.
    state_type candidate = m_state;
    covariance_type P = m_covariance;
    if (check(formal_eskf::try_correct_horizontal_position(candidate, P, xy, Rxy, minimum_quaternion_norm, candidate,
                                                           P)) &&
        check(formal_eskf::try_correct_vertical_position(candidate, P, z, Rz, minimum_quaternion_norm, candidate, P)) &&
        check(formal_eskf::try_correct_velocity(candidate, P, velocity, Rv, minimum_quaternion_norm, candidate, P)))
    {
        m_state = candidate;
        m_covariance = P;
        m_gps_fused = m_gps.timestamp_sample;
        m_max_fusion_age = std::max(m_max_fusion_age, m_time - m_gps_fused);
        ++m_gps_updates;
    }
}

void FormalEskf::fuse_mag()
{
    if (m_mag.timestamp_sample <= m_mag_seen || m_mag.timestamp_sample > m_time)
    {
        return;
    }
    m_mag_seen = m_mag.timestamp_sample;
    const auto mag = vector3(100. * double(m_mag.magnetometer_ga[0]), 100. * double(m_mag.magnetometer_ga[1]),
                             100. * double(m_mag.magnetometer_ga[2]));
    if (!recent(m_mag.timestamp_sample, 200000) || !formal_eskf::linalg::all_finite(mag) ||
        std::abs(formal_eskf::linalg::norm(mag) - formal_eskf::linalg::norm(m_mag_n)) > 20.)
    {
        ++m_rejections;
        return;
    }
    const auto R = matrix3_type::identity() * 9.; // microtesla^2; PX4 gauss -> microtesla above.
    m_mag_ratio = gate(mag - formal_eskf::so3::inverse_rotate(m_state.q_nb, m_mag_n),
                       formal_eskf::magnetometer_jacobian(m_state, m_mag_n), R);
    if (m_mag_ratio > 1.)
    {
        ++m_rejections;
        return;
    }
    if (check(formal_eskf::try_correct_magnetometer(m_state, m_covariance, mag, m_mag_n, R, minimum_quaternion_norm,
                                                    m_state, m_covariance)))
    {
        m_mag_fused = m_mag.timestamp_sample;
        ++m_mag_updates;
    }
}

void FormalEskf::fuse_baro()
{
    if (m_baro.timestamp_sample <= m_baro_seen || m_baro.timestamp_sample > m_time)
    {
        return;
    }
    m_baro_seen = m_baro.timestamp_sample;
    if (!recent(m_baro.timestamp_sample, 200000) || !std::isfinite(m_baro.baro_alt_meter))
    {
        ++m_rejections;
        return;
    }
    const double z = m_baro_origin - double(m_baro.baro_alt_meter);
    m_height_ratio = square(z - m_state.p_n(2)) / (25. * (m_covariance(2, 2) + 1.));
    if (m_height_ratio > 1.)
    {
        ++m_rejections;
        return;
    }
    if (check(formal_eskf::try_correct_vertical_position(m_state, m_covariance, z, 1., minimum_quaternion_norm, m_state,
                                                         m_covariance)))
    {
        m_baro_fused = m_baro.timestamp_sample;
        ++m_baro_updates;
    }
}

void FormalEskf::publish(imu_type const & imu)
{
    const uint64_t now = hrt_absolute_time();
    const bool nav_valid = recent(m_gps_fused) && now < m_time + 100000;
    const bool heading_valid = recent(m_mag_fused) && m_covariance(8, 8) < 0.1;
    vehicle_attitude_s attitude{};
    attitude.timestamp_sample = m_time;
    attitude.timestamp = now;
    for (size_t i = 0; i < 4; ++i)
    {
        attitude.q[i] = float(m_state.q_nb.coefficients()(i));
    }
    attitude.delta_q_reset[0] = 1.f;
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
    local.z_valid = local.v_z_valid = nav_valid || recent(m_baro_fused);
    local.heading = matrix::Eulerf(matrix::Quatf(attitude.q)).psi();
    local.unaided_heading = NAN; // No separate gyro-only heading estimator.
    local.heading_var = m_covariance(8, 8);
    local.tilt_var = m_covariance(6, 6) + m_covariance(7, 7);
    local.heading_good_for_control = heading_valid;
    local.xy_global = local.z_global = true;
    local.ref_timestamp = m_origin_time;
    local.ref_lat = m_projection.getProjectionReferenceLat();
    local.ref_lon = m_projection.getProjectionReferenceLon();
    local.ref_alt = m_origin_alt;
    local.eph = std::sqrt(std::max(m_covariance(0, 0), m_covariance(1, 1)));
    local.epv = std::sqrt(m_covariance(2, 2));
    local.evh = std::sqrt(std::max(m_covariance(3, 3), m_covariance(4, 4)));
    local.evv = std::sqrt(m_covariance(5, 5));
    local.dead_reckoning = !nav_valid;
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
    global.lat_lon_valid = nav_valid;
    global.alt_valid = local.z_valid;
    global.eph = local.eph;
    global.epv = local.epv;
    global.dead_reckoning = !nav_valid;
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
    odometry.quality = nav_valid && heading_valid ? 100 : 0;
    m_odometry_pub.publish(odometry);
}

void FormalEskf::publish_status(uint64_t now)
{
    const bool healthy = m_initialized && !m_fault && now < m_time + 100000;
    const bool gps_valid = healthy && recent(m_gps_fused);
    const bool mag_valid = healthy && recent(m_mag_fused);
    const bool baro_valid = healthy && recent(m_baro_fused);
    estimator_status_s status{};
    status.timestamp = now;
    status.timestamp_sample = m_time;
    if (healthy)
    {
        status.control_mode_flags |= (1ULL << status.CS_TILT_ALIGN);
    }
    if (mag_valid)
    {
        status.control_mode_flags |= (1ULL << status.CS_YAW_ALIGN) | (1ULL << status.CS_MAG_3D);
    }
    if (gps_valid)
    {
        status.control_mode_flags |=
            (1ULL << status.CS_GNSS_POS) | (1ULL << status.CS_GNSS_VEL) | (1ULL << status.CS_GPS_HGT);
    }
    if (baro_valid)
    {
        status.control_mode_flags |= (1ULL << status.CS_BARO_HGT);
    }
    if (!m_land.landed)
    {
        status.control_mode_flags |= (1ULL << status.CS_IN_AIR);
    }
    status.gps_check_fail_flags = gps_good() ? 0 : (1 << status.GPS_CHECK_FAIL_GPS_FIX);
    status.filter_fault_flags = m_fault ? (1 << 16) : 0;
    status.hdg_test_ratio = m_mag_ratio;
    status.vel_test_ratio = m_vel_ratio;
    status.pos_test_ratio = m_pos_ratio;
    status.hgt_test_ratio = m_height_ratio;
    status.pre_flt_fail_innov_heading = !mag_valid || m_mag_ratio > 1.;
    status.pre_flt_fail_innov_vel_horiz = status.pre_flt_fail_innov_vel_vert = !gps_valid || m_vel_ratio > 1.;
    status.pre_flt_fail_innov_pos_horiz = !gps_valid || m_pos_ratio > 1.;
    status.pre_flt_fail_innov_height = !(baro_valid || gps_valid) || m_height_ratio > 1.;
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
    flags.cs_tilt_align = healthy;
    flags.cs_yaw_align = mag_valid;
    flags.cs_gnss_pos = flags.cs_gnss_vel = flags.cs_gps_hgt = gps_valid;
    flags.cs_mag_3d = flags.cs_mag = mag_valid;
    flags.cs_baro_hgt = baro_valid;
    flags.cs_in_air = !m_land.landed;
    flags.cs_inertial_dead_reckoning = !gps_valid;
    flags.fs_bad_acc_vertical = m_fault;
    flags.reject_hor_pos = m_pos_ratio > 1.;
    flags.reject_hor_vel = flags.reject_ver_vel = m_vel_ratio > 1.;
    flags.reject_ver_pos = m_height_ratio > 1.;
    flags.reject_yaw = m_mag_ratio > 1.;
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
    bias.accel_bias_limit = 3.;
    bias.gyro_bias_limit = 0.5;
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
    PX4_INFO("rejections=%u numerical_errors=%u gap_holds=%u max_gps_age_us=%llu", m_rejections, m_errors, m_gaps,
             (unsigned long long)m_max_fusion_age);
    PX4_INFO("p=[%.3f %.3f %.3f] v=[%.3f %.3f %.3f]", m_state.p_n(0), m_state.p_n(1), m_state.p_n(2), m_state.v_n(0),
             m_state.v_n(1), m_state.v_n(2));
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
