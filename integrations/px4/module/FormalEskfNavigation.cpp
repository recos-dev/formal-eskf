// SPDX-License-Identifier: BSD-3-Clause
#include "FormalEskf.hpp"

#include <algorithm>
#include <cmath>

namespace profile = formal_eskf::px4::profile;

FormalEskf::observation_history_type::GnssSample FormalEskf::decode_gnss(sensor_gps_s const & sample)
{
    observation_history_type::GnssSample result;
    result.stamp = {sample.timestamp_sample, {sample.device_id, 0}};
    result.latitude_deg = sample.latitude_deg;
    result.longitude_deg = sample.longitude_deg;
    result.altitude_msl_m = sample.altitude_msl_m;
    result.altitude_ellipsoid_m = sample.altitude_ellipsoid_m;
    result.velocity_n = vector3(sample.vel_n_m_s, sample.vel_e_m_s, sample.vel_d_m_s);
    result.horizontal_accuracy_m = sample.eph;
    result.vertical_accuracy_m = sample.epv;
    result.speed_accuracy_m_s = sample.s_variance_m_s;
    result.hdop = sample.hdop;
    result.vdop = sample.vdop;
    result.fix_type = sample.fix_type;
    result.satellites = sample.satellites_used;
    result.velocity_valid = sample.vel_ned_valid;
    result.spoofed = sample.spoofing_state == sensor_gps_s::SPOOFING_STATE_MULTIPLE;
    return result;
}

FormalEskf::observation_history_type::BarometerSample FormalEskf::decode_barometer(vehicle_air_data_s const & sample)
{
    return {{sample.timestamp_sample, {sample.baro_device_id, sample.calibration_count}}, sample.baro_alt_meter};
}

template <std::size_t N>
bool FormalEskf::record_gps_fusion(formal_eskf::runtime::GnssFusion::Group group,
                                   formal_eskf::runtime::FusionResult<backend_type, N> const & result, double & ratio)
{
    ratio = result.diagnostics_valid ? 0. : double(INFINITY);
    for (size_t axis = 0; result.diagnostics_valid && axis < N; ++axis)
    {
        ratio = std::max(ratio, result.test_ratio(axis));
    }
    if (!check(m_gnss.lifecycle.record_fusion(group, m_gnss.sample.time_us, result.decision)) || !check(result.status))
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
        ++m_gnss.corrections;
    }
    return true;
}

void FormalEskf::update_gps_quality()
{
    auto const & gps = m_gnss.input.sample;
    using namespace formal_eskf::runtime;
    m_gnss.sample = {};
    if (m_gnss.input.time_us <= m_gnss.sample_time_us || m_gnss.input.time_us > m_time_us)
    {
        return;
    }
    m_gnss.sample_time_us = m_gnss.input.time_us;
    if (gps.stamp.source.device_id != m_gnss.device_id || m_gnss.input.generation != m_gnss.generation)
    {
        m_gnss.quality.reset();
        m_gnss.quality_result = {};
        m_gnss.metadata_valid = false;
        m_gnss.lifecycle.reset();
        m_gnss.previous_fix = {};
        m_gnss.device_id = gps.stamp.source.device_id;
        m_gnss.generation = m_gnss.input.generation;
    }
    // Invalid velocity cannot qualify or refresh successful aiding.
    if (!gps.velocity_valid || !recent(m_gnss.input.time_us, profile::gnss::maximum_sample_age_us))
    {
        return;
    }
    m_gnss.sample.time_us = m_gnss.input.time_us;
    m_gnss.metadata_valid = std::isfinite(gps.latitude_deg) && std::abs(gps.latitude_deg) <= 90. &&
                            std::isfinite(gps.longitude_deg) && std::abs(gps.longitude_deg) <= 180. &&
                            std::isfinite(gps.altitude_msl_m) && std::isfinite(gps.altitude_ellipsoid_m) &&
                            std::isfinite(gps.hdop) && gps.hdop >= 0. && std::isfinite(gps.vdop) && gps.vdop >= 0.;
    if (!m_gnss.metadata_valid)
    {
        m_gnss.quality.reset();
        m_gnss.previous_fix = {};
        m_gnss.quality_result = {};
        m_gnss.sample.quality_available = false;
        ++m_rejections;
        return;
    }
    auto const sample = gnss_quality_sample();
    auto const motion = gnss_motion();
    m_gnss.quality_result = m_gnss.quality.update(sample, motion, m_time_us, m_gnss.quality_parameters);
    if (m_gnss.quality_result.decision == GnssQualityDecision::invalid_call)
    {
        check(m_gnss.quality_result.status);
        return;
    }
    m_gnss.sample.quality = m_gnss.quality_result.decision;
    m_gnss.previous_fix.initReference(gps.latitude_deg, gps.longitude_deg, sample.time_us);
    m_gnss.previous_altitude = gps.altitude_msl_m;
    if (m_gnss.sample.quality == GnssQualityDecision::rejected)
    {
        ++m_rejections;
    }
}

formal_eskf::runtime::GnssQualitySample<FormalEskf::backend_type> FormalEskf::gnss_quality_sample() const
{
    auto const & gps = m_gnss.input.sample;
    formal_eskf::runtime::GnssQualitySample<backend_type> sample;
    sample.time_us = m_gnss.input.time_us;
    sample.fix_type = gps.fix_type;
    sample.satellites = gps.satellites;
    sample.pdop = std::hypot(double(gps.hdop), double(gps.vdop));
    sample.horizontal_accuracy = gps.horizontal_accuracy_m;
    sample.vertical_accuracy = gps.vertical_accuracy_m;
    sample.speed_accuracy = gps.speed_accuracy_m_s;
    sample.spoofed = gps.spoofed;
    sample.velocity_n = gps.velocity_n;
    sample.position_reference_time_us = m_gnss.previous_fix.getProjectionReferenceTimestamp();
    if (sample.position_reference_time_us != 0)
    {
        float north{}, east{};
        m_gnss.previous_fix.project(gps.latitude_deg, gps.longitude_deg, north, east);
        sample.position_delta_n = vector3(north, east, m_gnss.previous_altitude - gps.altitude_msl_m);
    }
    return sample;
}

formal_eskf::runtime::GnssQualityMotion FormalEskf::gnss_motion() const
{
    formal_eskf::runtime::GnssQualityMotion motion;
    if (m_gnss.context.vehicle.known)
    {
        motion.in_air = m_gnss.context.vehicle.armed;
    }
    if (m_gnss.context.land.known)
    {
        motion.in_air = !m_gnss.context.land.landed;
        motion.vehicle_at_rest = m_gnss.context.land.at_rest;
    }
    return motion;
}

bool FormalEskf::gps_good() const
{
    auto const & gps = m_gnss.input.sample;
    return m_gnss.metadata_valid && gps.velocity_valid &&
           recent(m_gnss.input.time_us, profile::gnss::maximum_sample_age_us) &&
           m_gnss.quality_result.decision == formal_eskf::runtime::GnssQualityDecision::accepted;
}

uint16_t FormalEskf::gps_failure_flags() const
{
    auto const & result = m_gnss.quality_result;
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
    auto const & group = m_gnss.lifecycle.state().horizontal_position;
    return group.active && !group.recovery_required &&
           recent(std::max(group.last_fuse_time_us, group.last_reset_time_us), max_age);
}

bool FormalEskf::gps_velocity_valid(uint64_t max_age) const
{
    auto const & group = m_gnss.lifecycle.state().velocity;
    return group.active && !group.recovery_required &&
           recent(std::max(group.last_fuse_time_us, group.last_reset_time_us), max_age);
}

void FormalEskf::fuse_gps()
{
    using namespace formal_eskf::runtime;
    GnssFusion::Conditions conditions;
    bool const source_current =
        m_output.current().time_us == 0 ||
        (m_observations.gnss_source().device_id != 0 && m_observations.gnss_generation() == m_gnss.generation);
    conditions.horizontal_position = {.enabled = true, .ready = m_initialized && source_current};
    conditions.velocity = conditions.horizontal_position;
    // HeightAiding manages the two height sources separately from horizontal aiding.
    if (!check(m_gnss.lifecycle.update(m_time_us, conditions, profile::gnss::timing, m_gnss.sample)))
    {
        return;
    }
    if (m_gnss.lifecycle.state().restart_quality)
    {
        m_gnss.quality.restart_health();
    }
    if (!m_gnss.lifecycle.state().velocity.active)
    {
        m_gnss.vertical_innovation = {};
    }
    if (m_gnss.sample.time_us == 0)
    {
        return;
    }
    if (!m_gnss.metadata_valid || !formal_eskf::succeeded(m_gnss.quality_result.status))
    {
        m_gnss.vertical_innovation = {};
        return;
    }
    auto const observation = gnss_velocity_observation();
    if (!gps_good())
    {
        // A still-active source may supply health evidence without fusion permission.
        if (m_gnss.lifecycle.state().velocity.active)
        {
            check_gnss_velocity_observation(observation);
        }
        return;
    }
    // The lifecycle has decided both actions before either operation advances progress.
    if (!fuse_gps_velocity(observation) || !fuse_gps_position())
    {
        return;
    }
    m_max_fusion_age = std::max(m_max_fusion_age, m_time_us - m_gnss.sample.time_us);
}

FormalEskf::GnssVelocityObservation FormalEskf::gnss_velocity_observation() const
{
    auto const & gps = m_gnss.input.sample;
    GnssVelocityObservation observation;
    observation.velocity_n = gps.velocity_n;
    double const variance = square(std::max(double(gps.speed_accuracy_m_s), profile::gnss::minimum_velocity_noise_m_s));
    observation.variances = vector3(variance, variance, profile::gnss::vertical_variance_multiplier * variance);
    observation.covariance = matrix3_type::identity() * variance;
    observation.covariance(2, 2) = observation.variances(2);
    observation.conditions = {m_vertical_health.state().bad_vertical_acceleration, double(gps.speed_accuracy_m_s),
                              m_gnss.quality_parameters.maximum_speed_accuracy};
    return observation;
}

void FormalEskf::check_gnss_velocity_observation(GnssVelocityObservation const & observation)
{
    auto const evidence = formal_eskf::runtime::check_gnss_velocity(
        m_state, m_covariance, observation.velocity_n, observation.covariance, profile::minimum_quaternion_norm,
        profile::gnss::gate_sigma, observation.conditions);
    if (!check(evidence.gate.status))
    {
        return;
    }
    m_gnss.vertical_innovation = {evidence.gate.diagnostics_valid, -evidence.fusion_innovation(2),
                                  evidence.gate.innovation_variance(2)};
    m_gnss.velocity_test_ratio =
        evidence.gate.diagnostics_valid
            ? std::max({evidence.gate.test_ratio(0), evidence.gate.test_ratio(1), evidence.gate.test_ratio(2)})
            : double(INFINITY);
}

bool FormalEskf::fuse_gps_velocity(GnssVelocityObservation const & observation)
{
    using namespace formal_eskf::runtime;
    using Group = gnss_lifecycle_type::Group;
    auto const & velocity = observation.velocity_n;
    auto const & velocity_variances = observation.variances;
    auto const & Rv = observation.covariance;
    auto const & velocity_conditions = observation.conditions;
    auto const & health = m_vertical_health.state();
    if (m_gnss.lifecycle.state().velocity.action == GnssFusion::Action::reset)
    {
        vector3_type delta;
        auto const status =
            reset_velocity(m_state, m_covariance, velocity, velocity_variances, m_state, m_covariance, delta);
        if (!check(m_gnss.lifecycle.record_reset(Group::velocity, m_gnss.sample.time_us,
                                                 formal_eskf::succeeded(status))) ||
            !check(status))
        {
            return false;
        }
        if (!reset_output({}, delta, {}))
        {
            return false;
        }
        m_velocity_reset_delta = delta;
        ++m_velocity_reset_counter;
        record_vertical_velocity_reset(delta(2));
        ++m_odometry_reset_counter;
        m_gnss.velocity_test_ratio = 0.;
        m_gnss.vertical_innovation = {true, 0., velocity_variances(2)};
    }
    else if (m_gnss.lifecycle.state().velocity.action == GnssFusion::Action::fuse)
    {
        auto const result = fuse_gnss_velocity(m_state, m_covariance, velocity, Rv, profile::minimum_quaternion_norm,
                                               profile::gnss::gate_sigma, velocity_conditions, m_state, m_covariance,
                                               m_bias_learning_result.bias_update);
        if (!record_gps_fusion(Group::velocity, result.fusion, m_gnss.velocity_test_ratio))
        {
            return false;
        }
        m_gnss.vertical_innovation = {result.fusion.diagnostics_valid, -result.fusion_innovation(2),
                                      result.fusion.innovation_variance(2)};
        // Use preceding height progress, after normal velocity correction and before height control.
        if (m_height_aiding.reset_required(m_time_us, {health.last_update_time_us, health.last_good_time_us}))
        {
            double delta{};
            if (!check(reset_vertical_velocity(m_state, m_covariance, velocity(2), velocity_variances(2), m_state,
                                               m_covariance, delta)))
            {
                return false;
            }
            if (!reset_output({}, vector3(0., 0., delta), {}))
            {
                return false;
            }
            record_vertical_velocity_reset(delta);
            ++m_odometry_reset_counter;
            // A Down-only reset preserves this attempt's selected velocity diagnostics.
        }
    }
    return true;
}

bool FormalEskf::fuse_gps_position()
{
    auto const & gps = m_gnss.input.sample;
    using namespace formal_eskf::runtime;
    using Group = gnss_lifecycle_type::Group;
    float north{}, east{};
    m_origin.projection.project(gps.latitude_deg, gps.longitude_deg, north, east);
    const auto xy = vector2_type::from_row_major({double(north), double(east)});
    const double position_variance =
        square(std::min(std::max(double(gps.horizontal_accuracy_m), profile::gnss::minimum_position_noise_m),
                        profile::gnss::maximum_position_noise_m));
    const auto Rxy = matrix2_type::identity() * position_variance;
    if (m_gnss.lifecycle.state().horizontal_position.action == GnssFusion::Action::reset)
    {
        vector2_type delta;
        auto const variances = vector2_type::from_row_major({position_variance, position_variance});
        auto const status =
            reset_gnss_horizontal_position(m_state, m_covariance, xy, variances, m_state, m_covariance, delta);
        if (!check(m_gnss.lifecycle.record_reset(Group::horizontal_position, m_gnss.sample.time_us,
                                                 formal_eskf::succeeded(status))) ||
            !check(status))
        {
            return false;
        }
        if (!reset_output({}, {}, vector3(delta(0), delta(1), 0.)))
        {
            return false;
        }
        m_position_reset_delta = delta;
        ++m_position_reset_counter;
        ++m_odometry_reset_counter;
        m_gnss.position_test_ratio = 0.;
    }
    else if (m_gnss.lifecycle.state().horizontal_position.action == GnssFusion::Action::fuse)
    {
        auto const result = fuse_horizontal_position(m_state, m_covariance, xy, Rxy, profile::minimum_quaternion_norm,
                                                     profile::gnss::gate_sigma, m_state, m_covariance,
                                                     m_bias_learning_result.bias_update);
        if (!record_gps_fusion(Group::horizontal_position, result, m_gnss.position_test_ratio))
        {
            return false;
        }
    }
    return true;
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
    auto const & gps = m_gnss.input.sample;
    using Source = height_type::Source;
    height_type::Sample const barometer{m_barometer.time_us,
                                        m_barometer.sample.stamp.source.device_id,
                                        m_barometer.sample.stamp.source.calibration_count,
                                        m_barometer.sample.altitude_m,
                                        profile::height::barometer_variance,
                                        true,
                                        m_barometer.generation};
    height_type::Sample gnss;
    if (m_gnss.sample.time_us != 0)
    {
        gnss = {m_gnss.sample.time_us,
                gps.stamp.source.device_id,
                0,
                gps.altitude_msl_m,
                square(std::max(double(gps.vertical_accuracy_m), profile::height::minimum_gnss_noise_m)),
                gps_good(),
                m_gnss.input.generation};
    }
    auto const & health = m_vertical_health.state();
    height_type::Acceleration const acceleration{health.bad_vertical_acceleration,
                                                 {health.last_update_time_us, health.last_good_time_us}};
    auto const result = m_height_aiding.update(m_time_us, barometer, gnss, m_gnss.lifecycle.state().velocity.active,
                                               m_state, m_covariance, m_bias_learning_result.bias_update, acceleration);
    if (!check(result.status))
    {
        return;
    }
    if ((result.height_resets != 0 || result.velocity_resets != 0) &&
        !reset_output({}, vector3(0., 0., result.velocity_delta), vector3(0., 0., result.height_delta)))
    {
        return;
    }
    if (result.height_resets != 0U)
    {
        m_height_reset_delta = result.height_delta;
        m_height_reset_counter = static_cast<std::uint8_t>(m_height_reset_counter + result.height_resets);
    }
    m_rejections += result.rejected;
    m_odometry_reset_counter += result.height_resets + result.velocity_resets;
    if (result.velocity_resets != 0)
    {
        record_vertical_velocity_reset(result.velocity_delta);
    }
    auto const & baro = m_height_aiding.session(Source::barometer);
    auto const & gnss_session = m_height_aiding.session(Source::gnss);
    m_baro_updates += baro.fused_time == m_time_us;
    m_gnss.corrections += gnss_session.fused_time == m_time_us;
    m_height_ratio = baro.fusion.diagnostics_valid ? baro.fusion.test_ratio(0) : double(INFINITY);
    m_gps_height_ratio = gnss_session.fusion.diagnostics_valid ? gnss_session.fusion.test_ratio(0) : double(INFINITY);
}
