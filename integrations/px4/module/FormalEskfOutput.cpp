// SPDX-License-Identifier: BSD-3-Clause
#include "FormalEskf.hpp"

#include <algorithm>
#include <cmath>

namespace profile = formal_eskf::px4::profile;

void FormalEskf::publish(imu_type const & imu)
{
    const uint64_t now = hrt_absolute_time();
    const bool nav_valid = (gps_position_valid() || gps_velocity_valid()) && now < m_time_us + 100000;
    const bool heading_valid = magnetic_heading_valid();
    vehicle_attitude_s attitude{};
    attitude.timestamp_sample = m_time_us;
    attitude.timestamp = now;
    for (size_t i = 0; i < 4; ++i)
    {
        attitude.q[i] = float(m_state.q_nb.coefficients()(i));
        attitude.delta_q_reset[i] = float(m_magnetic_aiding.reset_delta().coefficients()(i));
    }
    attitude.quat_reset_counter = m_magnetic_aiding.reset_counter();
    m_attitude_pub.publish(attitude);
    if (m_time_us - m_last_output < 10000)
    {
        return;
    } // Navigation/odometry <= 100 Hz.
    m_last_output = m_time_us;
    vehicle_local_position_s local{};
    local.timestamp_sample = m_time_us;
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
    local.z_valid = m_height_aiding.valid(m_time_us);
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
    local.ref_timestamp = m_origin.time_us;
    local.ref_lat = m_origin.projection.getProjectionReferenceLat();
    local.ref_lon = m_origin.projection.getProjectionReferenceLon();
    local.ref_alt = m_origin.altitude_msl_m;
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
    global.timestamp_sample = m_time_us;
    global.timestamp = now;
    m_origin.projection.reproject(local.x, local.y, global.lat, global.lon);
    global.alt = m_origin.altitude_msl_m - m_state.p_n(2);
    global.alt_ellipsoid = m_origin.altitude_ellipsoid_m - m_state.p_n(2);
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
    odometry.timestamp_sample = m_time_us;
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
    const bool healthy = m_initialized && !m_fault && now < m_time_us + 100000;
    const bool gps_pos_valid = healthy && gps_position_valid();
    const bool gps_vel_valid = healthy && gps_velocity_valid();
    const bool gps_valid = gps_pos_valid || gps_vel_valid;
    const bool gps_height_active = healthy && m_height_aiding.session(height_type::Source::gnss).active;
    const bool mag_valid = healthy && magnetic_heading_valid();
    const bool mag_aiding = healthy && m_magnetic_aiding.heading_aiding();
    const bool baro_active = healthy && m_height_aiding.session(height_type::Source::barometer).active;
    estimator_status_s status{};
    status.timestamp = now;
    status.timestamp_sample = m_time_us;
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
        !m_height_aiding.valid(m_time_us) ||
        !((baro_active && m_height_ratio <= 1.) || (gps_height_active && m_gps_height_ratio <= 1.));
    status.pos_horiz_accuracy = std::sqrt(std::max(m_covariance(0, 0), m_covariance(1, 1)));
    status.pos_vert_accuracy = std::sqrt(m_covariance(2, 2));
    status.solution_status_flags = healthy ? 1 : 0;
    if (gps_valid)
    {
        status.solution_status_flags |= 0x3e;
    }
    status.accel_device_id = m_imu_history.source.accelerometer_id;
    status.gyro_device_id = m_imu_history.source.gyroscope_id;
    status.mag_device_id = m_mag.device_id;
    status.baro_device_id = m_baro.baro_device_id;
    m_status_pub.publish(status);
    estimator_status_flags_s flags{};
    flags.timestamp = now;
    flags.timestamp_sample = m_time_us;
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
    bias.timestamp_sample = m_time_us;
    bias.accel_device_id = m_imu_history.source.accelerometer_id;
    bias.gyro_device_id = m_imu_history.source.gyroscope_id;
    bias.accel_bias_valid = bias.gyro_bias_valid = healthy;
    bias.accel_bias_limit = profile::bias_limits<double>().accelerometer;
    bias.gyro_bias_limit = profile::bias_limits<double>().gyroscope;
    for (size_t i = 0; i < 3; ++i)
    {
        bias.accel_bias[i] = m_state.b_a(i);
        bias.gyro_bias[i] = m_state.b_g(i);
        bias.accel_bias_variance[i] = m_covariance(i + 9, i + 9);
        bias.gyro_bias_variance[i] = m_covariance(i + 12, i + 12);
    }
    m_bias_pub.publish(bias);
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
