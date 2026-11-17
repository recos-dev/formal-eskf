// SPDX-License-Identifier: BSD-3-Clause
#include "FormalEskf.hpp"

#include <algorithm>
#include <cmath>
#include <cerrno>

namespace profile = formal_eskf::px4::profile;

namespace
{
// Wire layouts follow pinned PX4 v1.16.2 EKF/common.h and its named status flags.
// EstimatorStatus.msg retains an obsolete fault-layout comment (vertical: 16).
std::uint64_t control_flags(estimator_status_flags_s const & flags)
{
    return (std::uint64_t(flags.cs_tilt_align) << 0) | (std::uint64_t(flags.cs_yaw_align) << 1) |
           (std::uint64_t(flags.cs_gnss_pos) << 2) | (std::uint64_t(flags.cs_mag_hdg) << 4) |
           (std::uint64_t(flags.cs_in_air) << 7) | (std::uint64_t(flags.cs_baro_hgt) << 9) |
           (std::uint64_t(flags.cs_gps_hgt) << 11) | (std::uint64_t(flags.cs_mag_field_disturbed) << 16) |
           (std::uint64_t(flags.cs_gnd_effect) << 20) | (std::uint64_t(flags.cs_vehicle_at_rest) << 26) |
           (std::uint64_t(flags.cs_inertial_dead_reckoning) << 29) | (std::uint64_t(flags.cs_gravity_vector) << 34) |
           (std::uint64_t(flags.cs_mag) << 35) | (std::uint64_t(flags.cs_mag_heading_consistent) << 37) |
           (std::uint64_t(flags.cs_baro_fault) << 43) | (std::uint64_t(flags.cs_gnss_vel) << 44);
}

std::uint32_t fault_flags(estimator_status_flags_s const & flags)
{
    return (std::uint32_t(flags.fs_bad_acc_vertical) << 10) | (std::uint32_t(flags.fs_bad_acc_clipping) << 11);
}

std::uint16_t innovation_flags(estimator_status_flags_s const & flags)
{
    return static_cast<std::uint16_t>((unsigned(flags.reject_hor_vel) << 0) | (unsigned(flags.reject_ver_vel) << 1) |
                                      (unsigned(flags.reject_hor_pos) << 2) | (unsigned(flags.reject_ver_pos) << 3) |
                                      (unsigned(flags.reject_yaw) << 7));
}

float innovation_ratio(double squared_ratio, bool active)
{
    // PX4 summary scale: magnitude of the signed, filtered squared ratio.
    return active && std::isfinite(squared_ratio) ? float(std::sqrt(std::abs(squared_ratio))) : NAN;
}

bool bias_valid(matrix::Vector3f const & value, matrix::Vector3f const & variance)
{
    for (size_t axis = 0; axis < 3; ++axis)
    {
        if (!std::isfinite(value(axis)) || !std::isfinite(variance(axis)) || variance(axis) < 0.f)
        {
            return false;
        }
    }
    return variance.longerThan(0.f) && !variance.longerThan(profile::output::maximum_bias_variance_norm);
}
} // namespace

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

FormalEskf::BodyOutput FormalEskf::body_output() const
{
    auto const & output = m_output.current();
    auto const position_offset_n = formal_eskf::so3::rotate(output.q_nb, m_sensor_positions.imu_b);
    auto const velocity_offset_n = formal_eskf::so3::rotate(
        output.q_nb, formal_eskf::linalg::cross(m_output.angular_rate_b(), m_sensor_positions.imu_b));
    return {output.p_n - position_offset_n, output.v_n - velocity_offset_n};
}

void FormalEskf::publish(imu_type const & imu)
{
    const uint64_t now = hrt_absolute_time();
    auto const validity = output_validity(now);
    auto const time = m_output.current().time_us;
    if (!validity.healthy || time - m_last_output < profile::output::navigation_interval_us)
    {
        return;
    }
    auto const body = body_output();
    if (!check(formal_eskf::linalg::all_finite(body.position_n) && formal_eskf::linalg::all_finite(body.velocity_n)
                   ? status_type::success
                   : status_type::non_finite_result))
    {
        return;
    }
    m_last_output = time;
    auto const local = publish_local_position(now, imu, body, validity.gps, validity.magnetic);
    publish_global_position(now, local, body);
    publish_odometry(now, imu, body);
}

vehicle_attitude_s FormalEskf::publish_attitude(std::uint64_t now)
{
    vehicle_attitude_s attitude{};
    attitude.timestamp_sample = m_output.current().time_us;
    attitude.timestamp = now;
    for (size_t i = 0; i < 4; ++i)
    {
        attitude.q[i] = float(m_output.current().q_nb.coefficients()(i));
        attitude.delta_q_reset[i] = float(m_quaternion_reset_delta.coefficients()(i));
    }
    attitude.quat_reset_counter = m_quaternion_reset_counter;
    record_publication(m_attitude_pub.publish(attitude));
    return attitude;
}

vehicle_local_position_s FormalEskf::publish_local_position(std::uint64_t now, imu_type const & imu,
                                                            BodyOutput const & body, bool navigation_valid,
                                                            bool heading_valid)
{
    vehicle_local_position_s local{};
    local.timestamp_sample = m_output.current().time_us;
    local.timestamp = now;
    local.x = body.position_n(0);
    local.y = body.position_n(1);
    local.z = body.position_n(2);
    local.vx = body.velocity_n(0);
    local.vy = body.velocity_n(1);
    local.vz = body.velocity_n(2);
    local.z_deriv = local.vz;
    const auto accel_n =
        formal_eskf::so3::rotate(m_output.current().q_nb, imu.specific_force_b - m_state.b_a) + m_parameters.gravity_n;
    local.ax = accel_n(0);
    local.ay = accel_n(1);
    local.az = accel_n(2);
    local.xy_valid = local.v_xy_valid = navigation_valid;
    auto const validity = output_validity(now);
    // PX4 controller compatibility: local z/vz share vertical aiding; global altitude stays separate.
    local.z_valid = local.v_z_valid = validity.vertical_velocity;
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
    auto const & q = m_output.current().q_nb.coefficients();
    local.heading = matrix::Eulerf(matrix::Quatf(float(q(0)), float(q(1)), float(q(2)), float(q(3)))).psi();
    local.unaided_heading = NAN; // No separate gyro-only heading estimator.
    auto const rotation_variance = rotation_variance_n();
    local.heading_var = rotation_variance(2);
    local.tilt_var = rotation_variance(0) + rotation_variance(1);
    local.heading_good_for_control = heading_valid;
    auto const & delta = m_quaternion_reset_delta.coefficients();
    local.delta_heading = 2. * std::atan2(delta(3), delta(0));
    local.heading_reset_counter = m_quaternion_reset_counter;
    local.xy_global = local.z_global = true;
    local.ref_timestamp = m_origin.time_us;
    local.ref_lat = m_origin.projection.getProjectionReferenceLat();
    local.ref_lon = m_origin.projection.getProjectionReferenceLon();
    local.ref_alt = m_origin.altitude_msl_m;
    local.eph = std::sqrt(m_covariance(0, 0) + m_covariance(1, 1));
    local.epv = std::sqrt(m_covariance(2, 2));
    local.evh = std::sqrt(m_covariance(3, 3) + m_covariance(4, 4));
    local.evv = std::sqrt(m_covariance(5, 5));
    local.dead_reckoning = validity.dead_reckoning;
    local.dist_bottom = NAN;
    local.dist_bottom_var = NAN; // No terrain estimate.
    local.vxy_max = local.vz_max = local.hagl_min = local.hagl_max_z = local.hagl_max_xy = INFINITY;
    record_publication(m_local_pub.publish(local));
    return local;
}

void FormalEskf::publish_global_position(std::uint64_t now, vehicle_local_position_s const & local,
                                         BodyOutput const & body)
{
    vehicle_global_position_s global{};
    global.timestamp_sample = m_output.current().time_us;
    global.timestamp = now;
    m_origin.projection.reproject(local.x, local.y, global.lat, global.lon);
    global.lon = matrix::wrap(global.lon, -180., 180.);
    global.alt = m_origin.altitude_msl_m - body.position_n(2);
    global.alt_ellipsoid = m_origin.altitude_ellipsoid_m - body.position_n(2);
    global.lat_lon_valid = local.xy_valid;
    global.lat_lon_reset_counter = m_position_reset_counter;
    global.alt_valid = output_validity(now).height;
    global.delta_alt = -local.delta_z;
    global.alt_reset_counter = local.z_reset_counter;
    global.eph = local.eph;
    global.epv = local.epv;
    global.dead_reckoning = local.dead_reckoning;
    global.terrain_alt = NAN;
    record_publication(m_global_pub.publish(global));
}

void FormalEskf::publish_odometry(std::uint64_t now, imu_type const & imu, BodyOutput const & body)
{
    vehicle_odometry_s odometry{};
    odometry.timestamp_sample = m_output.current().time_us;
    odometry.timestamp = now;
    odometry.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
    odometry.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
    for (size_t i = 0; i < 4; ++i)
    {
        odometry.q[i] = float(m_output.current().q_nb.coefficients()(i));
    }
    for (size_t i = 0; i < 3; ++i)
    {
        odometry.position[i] = body.position_n(i);
        odometry.velocity[i] = body.velocity_n(i);
        odometry.angular_velocity[i] = imu.angular_rate_b(i) - m_state.b_g(i);
        odometry.position_variance[i] = m_covariance(i, i);
        odometry.velocity_variance[i] = m_covariance(i + 3, i + 3);
        odometry.orientation_variance[i] = m_covariance(i + 6, i + 6);
    }
    odometry.reset_counter = m_odometry_reset_counter;
    odometry.quality = 0; // Like EKF2: no calibrated percentage-quality score.
    record_publication(m_odometry_pub.publish(odometry));
}

FormalEskf::OutputValidity FormalEskf::output_validity(std::uint64_t now) const
{
    auto const time = m_output.current().time_us;
    const bool healthy =
        m_initialized && !m_fault && m_output.ready() && now >= time && now - time < profile::output::maximum_age_us;
    const bool gps_current =
        m_observations.gnss_source().device_id != 0 && m_observations.gnss_generation() == m_gnss.generation;
    const bool magnetic_current = m_observations.magnetic_source().device_id != 0 &&
                                  m_observations.magnetic_generation() == m_magnetic.generation;
    const bool gps_pos_valid = healthy && gps_current && gps_position_valid();
    const bool gps_vel_valid = healthy && gps_current && gps_velocity_valid();
    const bool gps_valid = gps_pos_valid || gps_vel_valid;
    auto const & gnss_height = m_height_aiding.session(height_type::Source::gnss);
    const bool gps_height_active =
        healthy && gps_current && gnss_height.active && gnss_height.generation == m_observations.gnss_generation();
    const bool mag_valid = healthy && magnetic_current && magnetic_heading_valid();
    const bool mag_aiding = healthy && magnetic_current && magnetic_heading_aiding();
    auto const & barometer = m_height_aiding.session(height_type::Source::barometer);
    const bool baro_active = healthy && m_observations.barometer_source().device_id != 0 &&
                             m_observations.barometer_generation() == barometer.generation && barometer.active;
    const bool height_valid = m_height_aiding.valid(m_time_us) && (baro_active || gps_height_active);
    const bool dead_reckoning = !(gps_valid && (gps_position_valid(profile::output::dead_reckoning_age_us) ||
                                                gps_velocity_valid(profile::output::dead_reckoning_age_us)));
    return {healthy,       gps_pos_valid, gps_vel_valid, gps_valid,    gps_height_active,
            mag_valid,     mag_aiding,    baro_active,   height_valid, gps_vel_valid || height_valid,
            dead_reckoning};
}

void FormalEskf::publish_status(uint64_t now)
{
    auto const validity = output_validity(now);
    auto flags = status_flags(now, validity);
    auto const status = publish_estimator_status(now, validity, flags);
    auto const innovation = innovation_flags(flags);
    m_status_changes.control_count += status.control_mode_flags != m_status_changes.control;
    m_status_changes.fault_count += status.filter_fault_flags != m_status_changes.fault;
    m_status_changes.innovation_count += innovation != m_status_changes.innovation;
    m_status_changes.control = status.control_mode_flags;
    m_status_changes.fault = status.filter_fault_flags;
    m_status_changes.innovation = innovation;
    flags.control_status_changes = m_status_changes.control_count;
    flags.fault_status_changes = m_status_changes.fault_count;
    flags.innovation_fault_status_changes = m_status_changes.innovation_count;
    record_publication(m_flags_pub.publish(flags));
    if (m_initialized)
    {
        publish_sensor_bias(now, validity.healthy);
    }
}

estimator_status_s FormalEskf::publish_estimator_status(std::uint64_t now, OutputValidity const & validity,
                                                        estimator_status_flags_s const & flags)
{
    estimator_status_s status{};
    status.timestamp = now;
    status.timestamp_sample = m_time_us;
    auto const & tracking = m_output.tracking_error();
    status.output_tracking_error[0] = formal_eskf::linalg::norm(tracking.attitude_b);
    status.output_tracking_error[1] = formal_eskf::linalg::norm(tracking.velocity_n);
    status.output_tracking_error[2] = formal_eskf::linalg::norm(tracking.position_n);
    status.control_mode_flags = control_flags(flags);
    status.gps_check_fail_flags = gps_failure_flags();
    status.reset_count_quat = m_quaternion_reset_counter;
    status.reset_count_pos_ne = m_position_reset_counter;
    status.reset_count_vel_ne = m_velocity_reset_counter;
    status.reset_count_vel_d = m_vertical_velocity_reset_counter;
    status.reset_count_pod_d = m_height_reset_counter;
    status.filter_fault_flags = fault_flags(flags);
    // These diagnostics have no producer in this module; zero would imply a measurement.
    status.time_slip = NAN;
    status.mag_inclination_deg = status.mag_inclination_ref_deg = NAN;
    status.mag_strength_gs = status.mag_strength_ref_gs = NAN;
    status.hdg_test_ratio =
        innovation_ratio(std::max({std::abs(m_magnetic.ratios[0].filtered), std::abs(m_magnetic.ratios[1].filtered),
                                   std::abs(m_magnetic.ratios[2].filtered)}),
                         flags.cs_mag_hdg);
    const float horizontal_velocity_ratio = innovation_ratio(
        std::max(std::abs(m_gnss.velocity_ratios[0].filtered), std::abs(m_gnss.velocity_ratios[1].filtered)),
        flags.cs_gnss_vel);
    const float vertical_velocity_ratio = innovation_ratio(m_gnss.velocity_ratios[2].filtered, flags.cs_gnss_vel);
    status.vel_test_ratio = std::fmax(horizontal_velocity_ratio, vertical_velocity_ratio);
    status.pos_test_ratio = innovation_ratio(
        std::max(std::abs(m_gnss.position_ratios[0].filtered), std::abs(m_gnss.position_ratios[1].filtered)),
        flags.cs_gnss_pos);
    const float barometer_ratio =
        innovation_ratio(m_height_aiding.session(height_type::Source::barometer).ratio.filtered, flags.cs_baro_hgt);
    const float gnss_height_ratio =
        innovation_ratio(m_height_aiding.session(height_type::Source::gnss).ratio.filtered, flags.cs_gps_hgt);
    status.hgt_test_ratio = std::isfinite(barometer_ratio) && std::isfinite(gnss_height_ratio)
                                ? (barometer_ratio + gnss_height_ratio) / 2.f
                                : std::fmax(barometer_ratio, gnss_height_ratio);
    status.tas_test_ratio = status.hagl_test_ratio = status.beta_test_ratio = NAN;
    const float preflight_limit = profile::output::preflight_innovation_ratio;
    // Commander does not check our tilt/yaw capability flags. Retain the module's
    // heading-readiness veto; unlike EKF2, our GNSS policy has no alignment prerequisite.
    status.pre_flt_fail_innov_heading =
        !flags.cs_tilt_align || !validity.magnetic || status.hdg_test_ratio > preflight_limit;
    status.pre_flt_fail_innov_vel_horiz = horizontal_velocity_ratio > preflight_limit;
    status.pre_flt_fail_innov_vel_vert = vertical_velocity_ratio > preflight_limit;
    status.pre_flt_fail_innov_pos_horiz = status.pos_test_ratio > preflight_limit;
    status.pre_flt_fail_innov_height = status.hgt_test_ratio > preflight_limit;
    status.pre_flt_fail_mag_field_disturbed = flags.cs_mag_field_disturbed;
    status.pos_horiz_accuracy = m_initialized ? float(std::sqrt(m_covariance(0, 0) + m_covariance(1, 1))) : NAN;
    status.pos_vert_accuracy = m_initialized ? float(std::sqrt(m_covariance(2, 2))) : NAN;
    const bool gnss_glitch = m_gnss.position_test_ratio > 1. || m_gnss.velocity_test_ratios(0) > 1. ||
                             m_gnss.velocity_test_ratios(1) > 1. || m_gnss.velocity_test_ratios(2) > 1.;
    // MAVLink/PX4 solution capabilities are independent; horizontal aiding cannot certify height.
    status.solution_status_flags = static_cast<std::uint16_t>(
        (unsigned(flags.cs_tilt_align) << 0) | (unsigned(validity.gps) << 1) |
        (unsigned(validity.vertical_velocity) << 2) | (unsigned(validity.gps) << 3) | (unsigned(validity.gps) << 4) |
        (unsigned(validity.height) << 5) | (unsigned(flags.cs_vehicle_at_rest) << 7) |
        (unsigned(flags.cs_gnss_pos || flags.cs_gnss_vel) << 8) | (unsigned(flags.cs_gnss_pos) << 9) |
        (unsigned(gnss_glitch) << 10) | (unsigned(flags.fs_bad_acc_vertical || flags.fs_bad_acc_clipping) << 11));
    status.accel_device_id = m_imu_history.source().accelerometer_id;
    status.gyro_device_id = m_imu_history.source().gyroscope_id;
    status.mag_device_id = m_magnetic.source.device_id;
    status.baro_device_id = m_height_aiding.session(height_type::Source::barometer).device_id;
    record_publication(m_status_pub.publish(status));
    return status;
}

estimator_status_flags_s FormalEskf::status_flags(std::uint64_t now, OutputValidity const & validity) const
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
    flags.cs_gnd_effect = validity.healthy && m_ground_effect.active;
    flags.cs_baro_fault = m_height_aiding.session(height_type::Source::barometer).fault;
    flags.cs_in_air = m_imu_context.land.known && !m_imu_context.land.landed;
    flags.cs_vehicle_at_rest = validity.healthy && m_imu_context.land.known && m_imu_context.land.at_rest;
    flags.cs_inertial_dead_reckoning = validity.dead_reckoning;
    flags.fs_bad_acc_vertical = m_vertical_health.state().bad_vertical_acceleration;
    flags.fs_bad_acc_clipping = m_vertical_health.state().bad_clipping;
    flags.reject_hor_pos = m_gnss.position_rejected;
    // The public velocity operation accepts/rejects all three axes together.
    flags.reject_hor_vel = flags.reject_ver_vel = m_gnss.velocity_rejected;
    auto const reference = m_height_aiding.reference();
    if (reference != height_type::Source::none)
    {
        auto const & height = m_height_aiding.session(reference);
        flags.reject_ver_pos =
            !height.fusion_reset && height.fusion.decision == formal_eskf::runtime::FusionDecision::rejected;
    }
    flags.reject_yaw = m_magnetic.lifecycle.state().last_reset_time_us < m_magnetic.sample_time_us &&
                       m_magnetic.observation.fusion.decision == formal_eskf::runtime::FusionDecision::rejected;
    return flags;
}

void FormalEskf::publish_sensor_bias(std::uint64_t now, bool healthy)
{
    estimator_sensor_bias_s bias{};
    bias.timestamp = now;
    bias.timestamp_sample = m_time_us;
    bias.accel_device_id = m_imu_history.source().accelerometer_id;
    bias.gyro_device_id = m_imu_history.source().gyroscope_id;
    bias.accel_bias_limit = profile::bias_limits<double>().accelerometer;
    bias.gyro_bias_limit = profile::bias_limits<double>().gyroscope;
    for (size_t i = 0; i < 3; ++i)
    {
        bias.accel_bias[i] = m_state.b_a(i);
        bias.gyro_bias[i] = m_state.b_g(i);
        bias.accel_bias_variance[i] = m_covariance(i + 9, i + 9);
        bias.gyro_bias_variance[i] = m_covariance(i + 12, i + 12);
    }
    bias.accel_bias_valid = healthy && bias.accel_device_id != 0 &&
                            bias_valid(matrix::Vector3f(bias.accel_bias), matrix::Vector3f(bias.accel_bias_variance));
    bias.gyro_bias_valid = healthy && bias.gyro_device_id != 0 &&
                           bias_valid(matrix::Vector3f(bias.gyro_bias), matrix::Vector3f(bias.gyro_bias_variance));
    // Temporary learning inhibition is not loss of the stored estimate. Stable stays false:
    // this module has no PX4 calibration-learning/convergence monitor.
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
    snapshot.output_time_us = m_output.current().time_us;
    snapshot.lag_us = snapshot.output_time_us >= m_time_us ? snapshot.output_time_us - m_time_us : 0;
    snapshot.output_ready = m_output.ready();
    snapshot.imu_entries = m_imu_history.entries();
    snapshot.arrival_rejections = m_arrival_rejections;
    snapshot.rate_rejections = m_rate_rejections;
    snapshot.stale_arrivals = m_stale_arrivals;
    snapshot.source_discards = m_source_discards;
    snapshot.overflows = m_overflows;
    snapshot.retired_observations = m_retired_observations;
    snapshot.generation_loss = m_generation_loss;
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
    snapshot.output_mean_s = perf_mean(m_output_perf);
    snapshot.fusion_mean_s = perf_mean(m_fusion_perf);
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
    PX4_INFO("timing: fusion_us=%llu output_us=%llu lag_us=%llu ready=%u imu_entries=%u",
             (unsigned long long)snapshot.sample_time_us, (unsigned long long)snapshot.output_time_us,
             (unsigned long long)snapshot.lag_us, unsigned(snapshot.output_ready), unsigned(snapshot.imu_entries));
    PX4_INFO("arrivals: rejected=%llu rate=%llu stale=%llu source_discards=%llu overflow=%llu retired=%llu lost=%llu",
             (unsigned long long)snapshot.arrival_rejections, (unsigned long long)snapshot.rate_rejections,
             (unsigned long long)snapshot.stale_arrivals, (unsigned long long)snapshot.source_discards,
             (unsigned long long)snapshot.overflows, (unsigned long long)snapshot.retired_observations,
             (unsigned long long)snapshot.generation_loss);
    PX4_INFO("formal-eskf INS / PX4 matrix / binary64 / C++20 / single instance");
    PX4_INFO("initialized=%d fault=%d predictions=%u gps=%u mag=%u baro=%u", snapshot.initialized, snapshot.fault,
             snapshot.predictions, snapshot.gps_updates, snapshot.mag_updates, snapshot.baro_updates);
    PX4_INFO("gnss_quality=%d qualified=%d metadata_valid=%d fail_flags=0x%x pos_active=%d vel_active=%d resets=%u/%u",
             int(snapshot.gnss_quality), snapshot.qualified, snapshot.metadata_valid,
             unsigned(snapshot.gps_failure_flags), snapshot.gps_position_active, snapshot.gps_velocity_active,
             unsigned(snapshot.position_resets), unsigned(snapshot.velocity_resets));
    PX4_INFO("rejections=%u estimator_errors=%u gap_holds=%u max_gps_age_us=%llu", snapshot.rejections, snapshot.errors,
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
    PX4_INFO("phases: output=%.3f fusion=%.3f us mean (PX4 clock, not wall-time WCET)",
             double(snapshot.output_mean_s) * 1e6, double(snapshot.fusion_mean_s) * 1e6);
    return 0;
}
