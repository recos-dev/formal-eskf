// SPDX-License-Identifier: BSD-3-Clause
#include "FormalEskf.hpp"

#include <algorithm>
#include <cmath>

namespace profile = formal_eskf::px4::profile;

template <std::size_t N>
bool FormalEskf::record_gps_fusion(formal_eskf::runtime::GnssFusion::Group group,
                                   formal_eskf::runtime::FusionResult<backend_type, N> const & result, double & ratio)
{
    ratio = result.diagnostics_valid ? 0. : double(INFINITY);
    for (size_t axis = 0; result.diagnostics_valid && axis < N; ++axis)
    {
        ratio = std::max(ratio, result.test_ratio(axis));
    }
    if (!check(m_gnss_fusion.record_fusion(group, m_gnss_sample.time_us, result.decision)) || !check(result.status))
    {
        return false;
    }
    if (result.decision == formal_eskf::runtime::FusionDecision::rejected)
    {
        ++m_rejections;
    }
    else if (result.decision == formal_eskf::runtime::FusionDecision::fused)
    {
        if (!check(formal_eskf::runtime::limit_bias(m_state, profile::bias_limits<double>())))
        {
            return false;
        }
        ++m_gps_updates;
    }
    return true;
}

void FormalEskf::update_gps_quality()
{
    using namespace formal_eskf::runtime;
    m_gnss_sample = {};
    if (m_gps.timestamp_sample <= m_gps_seen || m_gps.timestamp_sample > m_time_us)
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
    m_gnss_quality_result = m_gnss_quality.update(sample, motion, m_time_us, m_gnss_quality_parameters);
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

void FormalEskf::fuse_gps()
{
    using namespace formal_eskf::runtime;
    using Group = GnssFusion::Group;
    GnssFusion::Conditions conditions;
    conditions.horizontal_position = {.enabled = true, .ready = m_initialized};
    conditions.velocity = {.enabled = true, .ready = m_initialized};
    // HeightAiding manages the two height sources separately from horizontal aiding.
    if (!check(m_gnss_fusion.update(m_time_us, conditions, gnss_fusion_parameters, m_gnss_sample)))
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
            auto const evidence = check_gnss_velocity(m_state, m_covariance, velocity, Rv,
                                                      profile::minimum_quaternion_norm, 5., velocity_conditions);
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
    m_origin.projection.project(m_gps.latitude_deg, m_gps.longitude_deg, north, east);
    const auto xy = formal_eskf::linalg::Vector<backend_type, 2>::from_row_major({double(north), double(east)});
    const double position_variance = square(std::min(std::max(double(m_gps.eph), 0.5), 10.));
    const auto Rxy = formal_eskf::linalg::Matrix<backend_type, 2, 2>::identity() * position_variance;
    // Snapshot the coupled reset decision before either horizontal group advances.
    if (session.velocity.action == GnssFusion::Action::reset)
    {
        vector3_type delta;
        auto const status =
            reset_velocity(m_state, m_covariance, velocity, velocity_variances, m_state, m_covariance, delta);
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
            fuse_gnss_velocity(m_state, m_covariance, velocity, Rv, profile::minimum_quaternion_norm, 5.,
                               velocity_conditions, m_state, m_covariance, m_bias_learning_result.bias_update);
        if (!record_gps_fusion(Group::velocity, result.fusion, m_vel_ratio))
        {
            return;
        }
        m_gps_velocity_innovation = {result.fusion.diagnostics_valid, -result.fusion_innovation(2),
                                     result.fusion.innovation_variance(2)};
        // Use preceding height progress, after normal velocity correction and before height control.
        if (m_height_aiding.reset_required(m_time_us, {health.last_update_time_us, health.last_good_time_us}))
        {
            double delta{};
            if (!check(reset_vertical_velocity(m_state, m_covariance, velocity(2), velocity_variances(2), m_state,
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
            reset_gnss_horizontal_position(m_state, m_covariance, xy, variances, m_state, m_covariance, delta);
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
        auto const result = fuse_horizontal_position(m_state, m_covariance, xy, Rxy, profile::minimum_quaternion_norm,
                                                     5., m_state, m_covariance, m_bias_learning_result.bias_update);
        if (!record_gps_fusion(Group::horizontal_position, result, m_pos_ratio))
        {
            return;
        }
    }
    m_max_fusion_age = std::max(m_max_fusion_age, m_time_us - m_gnss_sample.time_us);
}

void FormalEskf::record_vertical_velocity_reset(double delta)
{
    m_vertical_velocity_reset_delta =
        m_vertical_velocity_reset_time == m_time_us ? m_vertical_velocity_reset_delta + delta : delta;
    m_vertical_velocity_reset_time = m_time_us;
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
    auto const result = m_height_aiding.update(m_time_us, barometer, gnss, m_gnss_fusion.state().velocity.active,
                                               m_state, m_covariance, m_bias_learning_result.bias_update, acceleration);
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
    m_baro_updates += baro.fused_time == m_time_us;
    m_gps_updates += gps.fused_time == m_time_us;
    m_height_ratio = baro.fusion.diagnostics_valid ? baro.fusion.test_ratio(0) : double(INFINITY);
    m_gps_height_ratio = gps.fusion.diagnostics_valid ? gps.fusion.test_ratio(0) : double(INFINITY);
}
