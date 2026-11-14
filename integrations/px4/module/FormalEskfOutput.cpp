// SPDX-License-Identifier: BSD-3-Clause
#include "FormalEskf.hpp"

#include <algorithm>
#include <cmath>
#include <cerrno>

namespace profile = formal_eskf::px4::profile;

bool FormalEskf::prepare_publications()
{
    // PX4 allocates topic data on first write, not advertise(nullptr). Zero epochs
    // and invalid flags/attitude represent no estimate during startup/restart.
    vehicle_attitude_s attitude{};
    std::fill(std::begin(attitude.q), std::end(attitude.q), NAN);
    vehicle_local_position_s local{};
    vehicle_global_position_s global{};
    vehicle_odometry_s odometry{};
    std::fill(std::begin(odometry.q), std::end(odometry.q), NAN);
    estimator_status_s status{};
    estimator_status_flags_s flags{};
    estimator_sensor_bias_s bias{};
    return m_attitude_pub.publish(attitude) && m_local_pub.publish(local) && m_global_pub.publish(global) &&
           m_odometry_pub.publish(odometry) && m_status_pub.publish(status) && m_flags_pub.publish(flags) &&
           m_bias_pub.publish(bias);
}

void FormalEskf::record_publication(bool delivered)
{
    if (!delivered)
    {
        ++m_publication_failures;
    }
}

void FormalEskf::publish(imu_type const & imu)
{
    const uint64_t now = hrt_absolute_time();
    const bool nav_valid =
        (gps_position_valid() || gps_velocity_valid()) && now < m_time_us + profile::output::maximum_age_us;
    const bool heading_valid = magnetic_heading_valid();
    auto const attitude = publish_attitude(now);
    if (m_time_us - m_last_output < profile::output::navigation_interval_us)
    {
        return;
    }
    m_last_output = m_time_us;
    auto const local = publish_local_position(now, imu, attitude, nav_valid, heading_valid);
    publish_global_position(now, local);
    publish_odometry(now, imu, attitude, nav_valid && heading_valid);
}

vehicle_attitude_s FormalEskf::publish_attitude(std::uint64_t now)
{
    vehicle_attitude_s attitude{};
    attitude.timestamp_sample = m_time_us;
    attitude.timestamp = now;
    for (size_t i = 0; i < 4; ++i)
    {
        attitude.q[i] = float(m_state.q_nb.coefficients()(i));
        attitude.delta_q_reset[i] = float(m_quaternion_reset_delta.coefficients()(i));
    }
    attitude.quat_reset_counter = m_quaternion_reset_counter;
    record_publication(m_attitude_pub.publish(attitude));
    return attitude;
}

vehicle_local_position_s FormalEskf::publish_local_position(std::uint64_t now, imu_type const & imu,
                                                            vehicle_attitude_s const & attitude, bool navigation_valid,
                                                            bool heading_valid)
{
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
    local.xy_valid = local.v_xy_valid = navigation_valid;
    local.z_valid = m_height_aiding.valid(m_time_us);
    local.v_z_valid = gps_velocity_valid() || local.z_valid;
    local.delta_z = m_height_reset_delta;
    local.z_reset_counter = m_height_reset_counter;
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
    auto const & delta = m_quaternion_reset_delta.coefficients();
    local.delta_heading = 2. * std::atan2(delta(3), delta(0));
    local.heading_reset_counter = m_quaternion_reset_counter;
    local.xy_global = local.z_global = true;
    local.ref_timestamp = m_origin.time_us;
    local.ref_lat = m_origin.projection.getProjectionReferenceLat();
    local.ref_lon = m_origin.projection.getProjectionReferenceLon();
    local.ref_alt = m_origin.altitude_msl_m;
    local.eph = std::sqrt(std::max(m_covariance(0, 0), m_covariance(1, 1)));
    local.epv = std::sqrt(m_covariance(2, 2));
    local.evh = std::sqrt(std::max(m_covariance(3, 3), m_covariance(4, 4)));
    local.evv = std::sqrt(m_covariance(5, 5));
    local.dead_reckoning = !(gps_position_valid(profile::output::dead_reckoning_age_us) ||
                             gps_velocity_valid(profile::output::dead_reckoning_age_us));
    local.dist_bottom = NAN;
    local.dist_bottom_var = NAN; // No terrain estimate.
    local.vxy_max = local.vz_max = local.hagl_min = local.hagl_max_z = local.hagl_max_xy = INFINITY;
    record_publication(m_local_pub.publish(local));
    return local;
}

void FormalEskf::publish_global_position(std::uint64_t now, vehicle_local_position_s const & local)
{
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
    record_publication(m_global_pub.publish(global));
}

void FormalEskf::publish_odometry(std::uint64_t now, imu_type const & imu, vehicle_attitude_s const & attitude,
                                  bool valid)
{
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
    odometry.quality = valid ? 100 : 0;
    record_publication(m_odometry_pub.publish(odometry));
}

FormalEskf::OutputValidity FormalEskf::output_validity(std::uint64_t now) const
{
    const bool healthy = m_initialized && !m_fault && now < m_time_us + profile::output::maximum_age_us;
    const bool gps_pos_valid = healthy && gps_position_valid();
    const bool gps_vel_valid = healthy && gps_velocity_valid();
    const bool gps_valid = gps_pos_valid || gps_vel_valid;
    const bool gps_height_active = healthy && m_height_aiding.session(height_type::Source::gnss).active;
    const bool mag_valid = healthy && magnetic_heading_valid();
    const bool mag_aiding = healthy && magnetic_heading_aiding();
    const bool baro_active = healthy && m_height_aiding.session(height_type::Source::barometer).active;
    return {healthy, gps_pos_valid, gps_vel_valid, gps_valid, gps_height_active, mag_valid, mag_aiding, baro_active};
}

void FormalEskf::publish_status(uint64_t now)
{
    auto const validity = output_validity(now);
    auto const status = publish_estimator_status(now, validity);
    publish_status_flags(now, validity, status);
    if (m_initialized)
    {
        publish_sensor_bias(now, validity.healthy);
    }
}

estimator_status_s FormalEskf::publish_estimator_status(std::uint64_t now, OutputValidity const & validity)
{
    estimator_status_s status{};
    status.timestamp = now;
    status.timestamp_sample = m_time_us;
    if (validity.healthy)
    {
        if (m_tilt_alignment.aligned())
        {
            status.control_mode_flags |= (1ULL << status.CS_TILT_ALIGN);
        }
        if (m_heading_aligned)
        {
            status.control_mode_flags |= (1ULL << status.CS_YAW_ALIGN);
        }
    }
    if (validity.magnetic_aiding)
    {
        status.control_mode_flags |= (1ULL << status.CS_MAG_HDG);
    }
    if (validity.healthy && m_gnss.lifecycle.state().horizontal_position.active)
    {
        status.control_mode_flags |= (1ULL << status.CS_GNSS_POS);
    }
    if (validity.healthy && m_gnss.lifecycle.state().velocity.active)
    {
        status.control_mode_flags |= (1ULL << status.CS_GNSS_VEL);
    }
    if (validity.gps_height)
    {
        status.control_mode_flags |= (1ULL << status.CS_GPS_HGT);
    }
    if (validity.barometer)
    {
        status.control_mode_flags |= (1ULL << status.CS_BARO_HGT);
    }
    if (!m_land.landed)
    {
        status.control_mode_flags |= (1ULL << status.CS_IN_AIR);
    }
    status.gps_check_fail_flags = gps_failure_flags();
    status.reset_count_quat = m_quaternion_reset_counter;
    status.reset_count_pos_ne = m_position_reset_counter;
    status.reset_count_vel_ne = m_velocity_reset_counter;
    status.reset_count_vel_d = m_vertical_velocity_reset_counter;
    status.reset_count_pod_d = m_height_reset_counter;
    status.filter_fault_flags = (m_fault || m_vertical_health.state().bad_vertical_acceleration) ? (1U << 16) : 0U;
    status.hdg_test_ratio = m_magnetic.test_ratio;
    status.vel_test_ratio = m_gnss.velocity_test_ratio;
    status.pos_test_ratio = m_gnss.position_test_ratio;
    status.hgt_test_ratio =
        m_height_aiding.reference() == height_type::Source::gnss ? m_gps_height_ratio : m_height_ratio;
    status.pre_flt_fail_innov_heading = !validity.magnetic || m_magnetic.test_ratio > 1.;
    status.pre_flt_fail_innov_vel_horiz = status.pre_flt_fail_innov_vel_vert =
        !validity.gps_velocity || m_gnss.velocity_test_ratio > 1.;
    status.pre_flt_fail_innov_pos_horiz = !validity.gps_position || m_gnss.position_test_ratio > 1.;
    status.pre_flt_fail_innov_height =
        !m_height_aiding.valid(m_time_us) ||
        !((validity.barometer && m_height_ratio <= 1.) || (validity.gps_height && m_gps_height_ratio <= 1.));
    status.pos_horiz_accuracy = std::sqrt(std::max(m_covariance(0, 0), m_covariance(1, 1)));
    status.pos_vert_accuracy = std::sqrt(m_covariance(2, 2));
    status.solution_status_flags = validity.healthy ? 1 : 0;
    if (validity.gps)
    {
        status.solution_status_flags |= 0x3e;
    }
    status.accel_device_id = m_imu_history.source.accelerometer_id;
    status.gyro_device_id = m_imu_history.source.gyroscope_id;
    status.mag_device_id = m_mag.device_id;
    status.baro_device_id = m_baro.baro_device_id;
    record_publication(m_status_pub.publish(status));
    return status;
}

void FormalEskf::publish_status_flags(std::uint64_t now, OutputValidity const & validity,
                                      estimator_status_s const & status)
{
    estimator_status_flags_s flags{};
    flags.timestamp = now;
    flags.timestamp_sample = m_time_us;
    flags.cs_tilt_align = validity.healthy && m_tilt_alignment.aligned();
    flags.cs_gravity_vector = validity.healthy && m_gravity.active;
    flags.cs_yaw_align = validity.healthy && m_heading_aligned;
    flags.cs_gnss_pos = validity.healthy && m_gnss.lifecycle.state().horizontal_position.active;
    flags.cs_gnss_vel = validity.healthy && m_gnss.lifecycle.state().velocity.active;
    flags.cs_gps_hgt = validity.gps_height;
    flags.cs_mag = validity.healthy && m_magnetic.lifecycle.state().active;
    flags.cs_mag_hdg = validity.magnetic_aiding;
    flags.cs_mag_heading_consistent = validity.healthy && m_magnetic.heading.state().consistent;
    auto const & magnetic_field = m_magnetic.observation.field;
    flags.cs_mag_field_disturbed =
        magnetic_field.diagnostics_valid && (magnetic_field.failures.strength || magnetic_field.failures.inclination);
    flags.cs_baro_hgt = validity.barometer;
    flags.cs_baro_fault = m_height_aiding.session(height_type::Source::barometer).fault;
    flags.cs_in_air = !m_land.landed;
    flags.cs_inertial_dead_reckoning =
        !(validity.healthy && (gps_position_valid(profile::output::dead_reckoning_age_us) ||
                               gps_velocity_valid(profile::output::dead_reckoning_age_us)));
    flags.fs_bad_acc_vertical = m_vertical_health.state().bad_vertical_acceleration;
    flags.reject_hor_pos = m_gnss.position_test_ratio > 1.;
    flags.reject_hor_vel = flags.reject_ver_vel = m_gnss.velocity_test_ratio > 1.;
    flags.reject_ver_pos = status.hgt_test_ratio > 1.f;
    flags.reject_yaw = m_magnetic.observation.fusion.diagnostics_valid && m_magnetic.test_ratio > 1.;
    record_publication(m_flags_pub.publish(flags));
}

void FormalEskf::publish_sensor_bias(std::uint64_t now, bool healthy)
{
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
    record_publication(m_bias_pub.publish(bias));
}

void FormalEskf::update_diagnostics(std::uint64_t now)
{
    ++m_diagnostics_sequence;
    int const locked = pthread_mutex_trylock(&m_diagnostics_mutex);
    if (locked != 0)
    {
        ++m_diagnostics_skipped;
        m_diagnostics_errors += locked != EBUSY;
        return;
    }
    auto & snapshot = m_diagnostics;
    snapshot.time_us = now;
    snapshot.sample_time_us = m_time_us;
    snapshot.sequence = m_diagnostics_sequence;
    snapshot.skipped = m_diagnostics_skipped;
    snapshot.exchange_errors = m_diagnostics_errors;
    snapshot.publication_failures = m_publication_failures;
    auto const & baro = m_height_aiding.session(height_type::Source::barometer);
    auto const & gps = m_height_aiding.session(height_type::Source::gnss);
    snapshot.initialized = m_initialized;
    snapshot.fault = m_fault;
    snapshot.predictions = m_predictions;
    snapshot.gps_updates = m_gnss.corrections;
    snapshot.mag_updates = m_magnetic.corrections;
    snapshot.baro_updates = m_baro_updates;
    snapshot.gnss_quality = static_cast<int>(m_gnss.quality_result.decision);
    snapshot.qualified = m_gnss.quality_result.qualified;
    snapshot.metadata_valid = m_gnss.metadata_valid;
    snapshot.gps_failure_flags = gps_failure_flags();
    snapshot.gps_position_active = m_gnss.lifecycle.state().horizontal_position.active;
    snapshot.gps_velocity_active = m_gnss.lifecycle.state().velocity.active;
    snapshot.position_resets = m_position_reset_counter;
    snapshot.velocity_resets = m_velocity_reset_counter;
    snapshot.rejections = m_rejections;
    snapshot.errors = m_errors;
    snapshot.gaps = m_gaps;
    snapshot.maximum_gps_age_us = m_max_fusion_age;
    snapshot.magnetic_active = m_magnetic.lifecycle.state().active;
    snapshot.heading_aligned = m_heading_aligned;
    snapshot.heading_consistent = m_magnetic.heading.state().consistent;
    snapshot.magnetic_stop = static_cast<unsigned>(m_magnetic.lifecycle.state().stop_reason);
    snapshot.magnetic_recovery = static_cast<unsigned>(m_magnetic.lifecycle.state().recovery_reason);
    snapshot.quaternion_resets = m_quaternion_reset_counter;
    snapshot.model_available = m_magnetic_model.available();
    snapshot.model_checks = m_magnetic_model.lookup_count();
    snapshot.model_revision = m_magnetic_model.model_revision();
    snapshot.heading_revision = m_magnetic_model.reference_id();
    snapshot.heading_source = static_cast<unsigned>(m_magnetic_model.heading_source());
    snapshot.model_unavailable = m_mag_model_unavailable;
    snapshot.declination = m_magnetic_model.declination();
    snapshot.height_reference = static_cast<unsigned>(m_height_aiding.reference());
    snapshot.barometer_active = baro.active;
    snapshot.gnss_height_active = gps.active;
    snapshot.barometer_fault = baro.fault;
    snapshot.height_resets = m_height_reset_counter;
    snapshot.vertical_velocity_resets = m_vertical_velocity_reset_counter;
    snapshot.barometer_bias = baro.bias.value;
    snapshot.gnss_height_bias = gps.bias.value;
    snapshot.high_dynamics = m_bias_learning_result.high_dynamics;
    snapshot.health_available = m_vertical_health.state().last_update_time_us != 0;
    snapshot.bad_acceleration = m_vertical_health.state().bad_vertical_acceleration;
    snapshot.gravity_enabled = m_gravity_enabled;
    snapshot.tilt_aligned = m_tilt_alignment.aligned();
    snapshot.gravity_active = m_gravity.active;
    snapshot.gravity_samples = m_gravity.samples;
    snapshot.gravity_corrections = m_gravity.corrections;
    snapshot.gravity_interval_us = m_gravity.interval_us;
    snapshot.accelerometer_mask = snapshot.gyroscope_mask = 0U;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        snapshot.position[axis] = m_state.p_n(axis);
        snapshot.velocity[axis] = m_state.v_n(axis);
        snapshot.accelerometer_mask |= unsigned(m_bias_learning_result.bias_update.accelerometer[axis]) << axis;
        snapshot.gyroscope_mask |= unsigned(m_bias_learning_result.bias_update.gyroscope[axis]) << axis;
    }
    snapshot.cycle_events = perf_event_count(m_cycle_perf);
    snapshot.cycle_mean_s = perf_mean(m_cycle_perf);
    m_diagnostics_errors += pthread_mutex_unlock(&m_diagnostics_mutex) != 0;
}

bool FormalEskf::read_diagnostics(Diagnostics & snapshot)
{
    if (!m_diagnostics_ready || pthread_mutex_lock(&m_diagnostics_mutex) != 0)
    {
        return false;
    }
    snapshot = m_diagnostics;
    return pthread_mutex_unlock(&m_diagnostics_mutex) == 0;
}

int FormalEskf::print_status()
{
    Diagnostics snapshot;
    if (!read_diagnostics(snapshot))
    {
        PX4_ERR("diagnostics unavailable");
        return PX4_ERROR;
    }
    PX4_INFO("diagnostics: sequence=%llu sample_us=%llu snapshot_us=%llu", (unsigned long long)snapshot.sequence,
             (unsigned long long)snapshot.sample_time_us, (unsigned long long)snapshot.time_us);
    PX4_INFO("diagnostics: skipped=%llu errors=%llu publication_failures=%llu", (unsigned long long)snapshot.skipped,
             (unsigned long long)snapshot.exchange_errors, (unsigned long long)snapshot.publication_failures);
    PX4_INFO("formal-eskf INS / PX4 matrix / binary64 / C++20 / single instance");
    PX4_INFO("initialized=%d fault=%d predictions=%u gps=%u mag=%u baro=%u", snapshot.initialized, snapshot.fault,
             snapshot.predictions, snapshot.gps_updates, snapshot.mag_updates, snapshot.baro_updates);
    PX4_INFO("gnss_quality=%d qualified=%d metadata_valid=%d fail_flags=0x%x pos_active=%d vel_active=%d resets=%u/%u",
             int(snapshot.gnss_quality), snapshot.qualified, snapshot.metadata_valid,
             unsigned(snapshot.gps_failure_flags), snapshot.gps_position_active, snapshot.gps_velocity_active,
             unsigned(snapshot.position_resets), unsigned(snapshot.velocity_resets));
    PX4_INFO("rejections=%u numerical_errors=%u gap_holds=%u max_gps_age_us=%llu", snapshot.rejections, snapshot.errors,
             snapshot.gaps, (unsigned long long)snapshot.maximum_gps_age_us);
    PX4_INFO("p=[%.3f %.3f %.3f] v=[%.3f %.3f %.3f]", snapshot.position[0], snapshot.position[1], snapshot.position[2],
             snapshot.velocity[0], snapshot.velocity[1], snapshot.velocity[2]);
    PX4_INFO("mag: active=%u aligned=%u consistent=%u stop=%u recovery=%u resets=%u",
             unsigned(snapshot.magnetic_active), unsigned(snapshot.heading_aligned),
             unsigned(snapshot.heading_consistent), unsigned(snapshot.magnetic_stop),
             unsigned(snapshot.magnetic_recovery), unsigned(snapshot.quaternion_resets));
    PX4_INFO("mag model: available=%u checks=%llu model_revision=%u heading_revision=%u source=%u unavailable=%u",
             unsigned(snapshot.model_available), (unsigned long long)snapshot.model_checks,
             unsigned(snapshot.model_revision), unsigned(snapshot.heading_revision), unsigned(snapshot.heading_source),
             unsigned(snapshot.model_unavailable));
    PX4_INFO("mag declination: %.3f deg", math::degrees(snapshot.declination));
    PX4_INFO("height: ref=%u baro=%u gnss=%u baro_fault=%u resets=%u vz_resets=%u", unsigned(snapshot.height_reference),
             unsigned(snapshot.barometer_active), unsigned(snapshot.gnss_height_active),
             unsigned(snapshot.barometer_fault), unsigned(snapshot.height_resets),
             unsigned(snapshot.vertical_velocity_resets));
    PX4_INFO("height bias: baro=%.3f gnss=%.3f m", snapshot.barometer_bias, snapshot.gnss_height_bias);
    PX4_INFO("bias learning: accel=%u gyro=%u high_dynamics=%u", snapshot.accelerometer_mask, snapshot.gyroscope_mask,
             unsigned(snapshot.high_dynamics));
    PX4_INFO("vertical IMU health: available=%u bad=%u", unsigned(snapshot.health_available),
             unsigned(snapshot.bad_acceleration));
    PX4_INFO("gravity: enabled=%u tilt=%u active=%u samples=%u fused=%u interval_us=%llu",
             unsigned(snapshot.gravity_enabled), unsigned(snapshot.tilt_aligned), unsigned(snapshot.gravity_active),
             snapshot.gravity_samples, snapshot.gravity_corrections, (unsigned long long)snapshot.gravity_interval_us);
    PX4_INFO("cycle: %llu events, %.3f us mean (PX4 clock)", (unsigned long long)snapshot.cycle_events,
             double(snapshot.cycle_mean_s) * 1e6);
    return 0;
}
