// SPDX-License-Identifier: BSD-3-Clause
#include "FormalEskf.hpp"

#include <algorithm>
#include <cmath>
#include <lib/world_magnetic_model/geo_mag_declination.h>

namespace profile = formal_eskf::px4::profile;

void FormalEskf::Gravity::reset_samples()
{
    sampler.reset();
    active = false;
}

void FormalEskf::fuse_gravity(ImuInput const & sample, imu_type const & imu)
{
    // Only admitted, real IMU intervals reach this phase. Alignment uses the
    // post-navigation/height prior, even when gravity fusion is disabled.
    if (!check(m_tilt_alignment.update(m_state.q_nb, m_covariance.block<6U, 6U, 3U, 3U>())))
    {
        return;
    }
    // The sampler uses the acceleration period; retain the independently
    // measured angle integral when the two integration periods differ.
    auto interval_imu = imu;
    interval_imu.angular_rate_b =
        imu.angular_rate_b * (static_cast<double>(sample.angular_period_us) / sample.acceleration_period_us);
    auto const observation = m_gravity.sampler.update(interval_imu, sample.acceleration_period_us,
                                                      sample.accelerometer_clipping, profile::minimum_quaternion_norm);
    if (!check(observation.status))
    {
        return;
    }

    auto const & gnss = m_gnss.lifecycle.state();
    formal_eskf::runtime::AccelerometerFusionConditions const conditions{
        m_gravity_enabled,
        m_tilt_alignment.aligned(),
        gnss.horizontal_position.active || gnss.velocity.active,
        m_imu_context.land.known && m_imu_context.land.at_rest,
        {(observation.clipping_mask & 1) != 0, (observation.clipping_mask & 2) != 0,
         (observation.clipping_mask & 4) != 0}};
    if (!conditions.enabled || conditions.horizontal_aiding || !conditions.tilt_aligned)
    {
        m_gravity.active = false;
    }
    if (!observation.ready)
    {
        return;
    }

    // Existing unnormalized observation weight, not calibrated sensor noise.
    auto const V = matrix3_type::identity() * formal_eskf::linalg::dot(m_parameters.gravity_n, m_parameters.gravity_n);
    auto const result = formal_eskf::runtime::fuse_accelerometer(
        m_state, m_covariance, observation.specific_force_b, m_parameters.gravity_n, V, conditions,
        observation.magnitude, profile::minimum_quaternion_norm, m_state, m_covariance,
        m_bias_learning_result.bias_update);
    m_gravity.active = result.conditions.active;
    if (!check(result.status))
    {
        return;
    }
    bool const fused = result.decision == formal_eskf::runtime::FusionDecision::fused;
    if (fused && !check(formal_eskf::runtime::limit_bias(m_state, profile::bias_limits<double>())))
    {
        // Sampling and correction have already committed; a tick is not atomic.
        return;
    }
    ++m_gravity.samples;
    m_gravity.interval_us = observation.interval_us;
    m_gravity.corrections += fused;
}

void FormalEskf::initialize_magnetic_aiding(magnetic_lifecycle_type::Source const & source,
                                            std::uint64_t sample_time_us)
{
    // Reset sensor history and estimator heading bookkeeping after real startup
    // alignment. Retain module reporting totals; invent no fusion timestamp.
    m_magnetic.heading = {};
    m_magnetic.filter.reset();
    m_magnetic.lifecycle.reset(source);
    m_magnetic.observation = {};
    m_magnetic.source = source;
    m_magnetic.sample_time_us = sample_time_us;
    m_magnetic.horizontal_aiding = false;
    m_magnetic.heading.reset_heading();
    m_heading_aligned = true;
    m_quaternion_reset_delta = {};
    m_quaternion_reset_counter = 0U;
}

bool FormalEskf::magnetic_heading_aiding() const
{
    return m_magnetic.lifecycle.state().active && m_heading_aligned && m_magnetic.observation.field.accepted &&
           (!m_magnetic.horizontal_aiding || m_magnetic.heading.state().consistent);
}

FormalEskf::vector3_type FormalEskf::rotation_variance_n() const
{
    // The core uses right-local body errors; PX4 heading/tilt proxies use NED errors.
    // Use the delayed attitude that belongs to this covariance, not current output.
    auto const R = formal_eskf::so3::to_rotation_matrix(m_state.q_nb);
    auto const covariance_n = formal_eskf::linalg::sandwich(R, m_covariance.block<6U, 6U, 3U, 3U>());
    return vector3(covariance_n(0, 0), covariance_n(1, 1), covariance_n(2, 2));
}

bool FormalEskf::magnetic_heading_valid() const
{
    auto const & lifecycle = m_magnetic.lifecycle.state();
    return magnetic_heading_aiding() &&
           recent(std::max(lifecycle.last_fuse_time_us, lifecycle.last_reset_time_us),
                  profile::magnetic::maximum_heading_age_us) &&
           rotation_variance_n()(2) < profile::magnetic::maximum_heading_variance;
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
        profile::microtesla_per_gauss * double(get_mag_strength_gauss(latitude, longitude))};
    magnetic_model_type::sample_type model;
    double const origin_lat =
        m_origin.projection.isInitialized() ? m_origin.projection.getProjectionReferenceLat() : latitude;
    double const origin_lon =
        m_origin.projection.isInitialized() ? m_origin.projection.getProjectionReferenceLon() : longitude;
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
    bool const origin_updated = m_origin.time_us > m_mag_model_origin_time;
    if (!m_magnetic_model.lookup_due(m_time_us, position_valid, origin_updated))
    {
        return;
    }
    double latitude = m_origin.projection.getProjectionReferenceLat();
    double longitude = m_origin.projection.getProjectionReferenceLon();
    if (position_valid)
    {
        m_origin.projection.reproject(m_state.p_n(0), m_state.p_n(1), latitude, longitude);
        longitude = matrix::wrap(longitude, -180., 180.);
    }
    auto const model = lookup_magnetic_model(latitude, longitude);
    m_mag_model_unavailable += !model.has_value();
    m_mag_model_origin_time = m_origin.time_us;
    auto const result = m_magnetic_model.update(m_time_us, model, position_valid, origin_updated);
    check(result.status);
}

FormalEskf::magnetic_lifecycle_type::Conditions FormalEskf::magnetic_conditions() const
{
    using FlightState = magnetic_lifecycle_type::FlightState;
    magnetic_lifecycle_type::Conditions conditions;
    conditions.enabled = true;
    // Session activity is distinct from output validity and dead reckoning.
    conditions.horizontal_aiding =
        m_gnss.lifecycle.state().horizontal_position.active || m_gnss.lifecycle.state().velocity.active;
    if (m_magnetic.context.vehicle.known)
    {
        conditions.flight_state = m_magnetic.context.vehicle.armed ? FlightState::in_air : FlightState::on_ground;
    }
    if (m_magnetic.context.land.known)
    {
        conditions.flight_state = m_magnetic.context.land.landed ? FlightState::on_ground : FlightState::in_air;
        conditions.at_rest = m_magnetic.context.land.at_rest;
    }
    if (m_output.current().time_us != 0)
    {
        conditions.enabled = m_observations.magnetic_source().device_id != 0 &&
                             m_observations.magnetic_generation() == m_magnetic.generation;
    }
    return conditions;
}

FormalEskf::observation_history_type::MagneticSample FormalEskf::decode_magnetic(vehicle_magnetometer_s const & sample)
{
    return {{sample.timestamp_sample, {sample.device_id, sample.calibration_count}},
            vector3(profile::microtesla_per_gauss * double(sample.magnetometer_ga[0]),
                    profile::microtesla_per_gauss * double(sample.magnetometer_ga[1]),
                    profile::microtesla_per_gauss * double(sample.magnetometer_ga[2]))};
}

FormalEskf::MagneticSample FormalEskf::consume_magnetic_sample()
{
    MagneticSample sample;
    auto const & input = m_magnetic.input;
    if (input.time_us <= m_magnetic.sample_time_us || input.time_us > m_time_us)
    {
        return sample;
    }
    // If a sensor switched away and back before selection, its identity alone
    // cannot reveal the interruption. Discard its old qualification/filter too.
    if (input.generation != m_magnetic.generation &&
        input.sample.stamp.source.device_id == m_magnetic.source.device_id &&
        input.sample.stamp.source.calibration_count == m_magnetic.source.calibration_count)
    {
        m_magnetic.lifecycle.restart();
        m_magnetic.filter.reset();
        m_magnetic.heading.reset_heading();
        m_heading_aligned = false;
    }
    m_magnetic.generation = input.generation;
    m_magnetic.sample_time_us = input.time_us;
    sample.time_us = input.time_us;
    sample.source = {input.sample.stamp.source.device_id, input.sample.stamp.source.calibration_count, 0U};
    sample.field_b = input.sample.field_b;
    sample.fresh = m_time_us - sample.time_us < profile::magnetic::timing.maximum_interval_us;
    sample.usable = sample.fresh && sample.source.device_id != 0U && formal_eskf::linalg::all_finite(sample.field_b) &&
                    formal_eskf::linalg::dot(sample.field_b, sample.field_b) > 0.;
    return sample;
}

FormalEskf::status_type FormalEskf::prepare_magnetic_observation(MagneticSample const & sample,
                                                                 magnetic_lifecycle_type::Conditions const & conditions,
                                                                 magnetic_lifecycle_type::Sample & event)
{
    // Only fresh arrivals enter this phase. Invalid fields with a valid source
    // still reach lifecycle readiness; a zero source supplies no new event.
    m_magnetic.observation = {};
    auto const & previous = m_magnetic.lifecycle.state();
    bool const source_changed = sample.source.device_id != m_magnetic.source.device_id ||
                                sample.source.calibration_count != m_magnetic.source.calibration_count;
    bool const on_ground = conditions.flight_state == magnetic_lifecycle_type::FlightState::on_ground;
    if ((source_changed && sample.source.device_id != 0U &&
         (on_ground || (previous.active && !conditions.horizontal_aiding))) ||
        (previous.recovery_reason == magnetic_lifecycle_type::RecoveryReason::source_changed && on_ground))
    {
        m_heading_aligned = false;
    }
    if (sample.source.device_id != 0U)
    {
        m_magnetic.source = sample.source;
        event = {sample.time_us, sample.source, false};
    }
    if (!sample.usable)
    {
        return status_type::success;
    }
    formal_eskf::runtime::MagneticHeading<backend_type>::Parameters const parameters{
        profile::magnetic::heading_noise_rad, profile::magnetic::acceleration_limit_m_s2,
        profile::magnetic::field.minimum_horizontal_field};
    auto const status = m_magnetic.heading.update(m_state.q_nb, sample.field_b, m_magnetic_model.observation_n(),
                                                  parameters, m_heading_aligned, conditions.horizontal_aiding);
    if (!formal_eskf::succeeded(status))
    {
        return status;
    }
    auto const * health = m_magnetic_model.available() ? &m_magnetic_model.health_n() : nullptr;
    m_magnetic.observation.field =
        formal_eskf::runtime::check_magnetic_field(m_state.q_nb, sample.field_b, m_magnetic_model.observation_n(),
                                                   health, profile::magnetic::field, false, profile::magnetic::checks);
    event.field_accepted = m_magnetic.observation.field.accepted;
    return m_magnetic.observation.field.status;
}

bool FormalEskf::fuse_magnetic_observation(MagneticSample const & sample)
{
    auto const V = matrix3_type::identity() * profile::magnetic::noise_variance;
    auto const * health = m_magnetic_model.available() ? &m_magnetic_model.health_n() : nullptr;
    m_magnetic.observation = formal_eskf::runtime::fuse_magnetometer(
        m_state, m_covariance, sample.field_b, m_magnetic_model.observation_n(), health, V, profile::magnetic::field,
        false, profile::minimum_quaternion_norm, profile::magnetic::gate_sigma, m_state, m_covariance,
        m_bias_learning_result.bias_update, profile::magnetic::checks);
    auto const & fusion = m_magnetic.observation.fusion;
    // Feedback precedes numerical status handling and post-correction limiting.
    if (!check(m_magnetic.lifecycle.record_fusion(sample.time_us, fusion.decision)) || !check(fusion.status))
    {
        return false;
    }
    return fusion.decision != formal_eskf::runtime::FusionDecision::fused ||
           check(formal_eskf::runtime::limit_bias(m_state, profile::bias_limits<double>()));
}

FormalEskf::status_type FormalEskf::reset_magnetic_heading(std::uint64_t sample_time_us)
{
    quaternion_type delta;
    auto const status = formal_eskf::runtime::reset_magnetic_heading(
        m_state, m_covariance, m_magnetic.filter.field_b(), m_magnetic_model.observation_n(),
        profile::magnetic::heading_noise_rad * profile::magnetic::heading_noise_rad,
        profile::magnetic::field.minimum_horizontal_field, profile::minimum_quaternion_norm, m_state, m_covariance,
        delta);
    bool const reset = formal_eskf::succeeded(status);
    auto const feedback = m_magnetic.lifecycle.record_reset(sample_time_us, reset);
    if (!formal_eskf::succeeded(feedback))
    {
        return feedback;
    }
    if (reset)
    {
        if (!reset_output(delta, {}, {}))
        {
            return status_type::out_of_range;
        }
        m_quaternion_reset_delta = delta;
        ++m_quaternion_reset_counter;
        m_heading_aligned = true;
        m_magnetic.heading.reset_heading();
    }
    return status;
}

void FormalEskf::fuse_mag()
{
    auto sample = consume_magnetic_sample();
    auto conditions = magnetic_conditions();
    if (sample.usable)
    {
        update_magnetic_model();
    }
    sample.source.reference_id = m_magnetic_model.reference_id();
    if (m_fault)
    {
        return;
    }

    auto const previous = m_magnetic.lifecycle.state();
    m_magnetic.horizontal_aiding = conditions.horizontal_aiding;
    magnetic_lifecycle_type::Sample event;
    bool rejected = sample.time_us != 0U && !sample.fresh;
    if (sample.fresh)
    {
        if (!check(prepare_magnetic_observation(sample, conditions, event)))
        {
            return;
        }
        rejected = !sample.usable || !event.field_accepted;
    }
    conditions.ready = !sample.fresh || sample.usable;
    conditions.heading_consistent = m_magnetic.heading.state().consistent;
    conditions.heading_aligned = m_heading_aligned;
    if (!check(m_magnetic.lifecycle.update(m_time_us, conditions, profile::magnetic::timing, event)))
    {
        return;
    }
    auto const & lifecycle = m_magnetic.lifecycle.state();
    if (lifecycle.filter_reset)
    {
        m_magnetic.filter.reset();
    }
    if (previous.active && !lifecycle.active && !conditions.horizontal_aiding)
    {
        m_heading_aligned = false;
    }
    if (sample.usable && !check(m_magnetic.filter.update(sample.field_b)))
    {
        return;
    }

    bool fused = false;
    if (lifecycle.action == magnetic_lifecycle_type::Action::fuse)
    {
        if (!fuse_magnetic_observation(sample))
        {
            return;
        }
        fused = m_magnetic.observation.fusion.decision == formal_eskf::runtime::FusionDecision::fused;
        rejected = m_magnetic.observation.fusion.decision == formal_eskf::runtime::FusionDecision::rejected;
    }
    bool reset = false;
    status_type reset_status = status_type::success;
    // Rejected fusion feedback can request a reset in the same tick.
    if (lifecycle.action == magnetic_lifecycle_type::Action::reset)
    {
        reset_status = reset_magnetic_heading(event.time_us);
        reset = formal_eskf::succeeded(reset_status);
    }
    // Preserve the separate post-feedback transition, including reset failure.
    if (previous.active && !lifecycle.active && !conditions.horizontal_aiding)
    {
        m_heading_aligned = false;
    }
    if (!check(reset_status))
    {
        return;
    }
    m_magnetic.corrections += fused;
    m_rejections += rejected;
    m_odometry_reset_counter += reset;
    if (sample.time_us != 0U)
    {
        auto const & fusion = m_magnetic.observation.fusion;
        m_magnetic.test_ratio =
            fusion.diagnostics_valid ? formal_eskf::linalg::max_abs(fusion.test_ratio) : double(INFINITY);
    }
}
