// SPDX-License-Identifier: BSD-3-Clause
#include "FormalEskf.hpp"

#include <algorithm>
#include <cmath>
#include <lib/world_magnetic_model/geo_mag_declination.h>

namespace profile = formal_eskf::px4::profile;

void FormalEskf::fuse_gravity(ImuInput const & sample, imu_type const & imu)
{
    auto const & gnss = m_gnss_fusion.state();
    formal_eskf::px4::GravityAiding<backend_type>::Context const context{
        m_gravity_enabled, recent(m_land.timestamp, profile::imu::rest_maximum_age_us) && m_land.at_rest,
        gnss.horizontal_position.active || gnss.velocity.active};
    auto const result = m_gravity_aiding.update(imu, sample.angular_period_us, sample.acceleration_period_us,
                                                sample.accelerometer_clipping, context, m_parameters.gravity_n, m_state,
                                                m_covariance, m_bias_learning_result.bias_update);
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
    }
    auto const model = lookup_magnetic_model(latitude, longitude);
    m_mag_model_unavailable += !model.has_value();
    m_mag_model_origin_time = m_origin.time_us;
    auto const result = m_magnetic_model.update(m_time_us, model, position_valid, origin_updated);
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
    if (m_mag.timestamp_sample > m_mag_seen && m_mag.timestamp_sample <= m_time_us)
    {
        m_mag_seen = m_mag.timestamp_sample;
        sample = {m_mag.timestamp_sample,
                  {m_mag.device_id, m_mag.calibration_count, 0U},
                  vector3(profile::microtesla_per_gauss * double(m_mag.magnetometer_ga[0]),
                          profile::microtesla_per_gauss * double(m_mag.magnetometer_ga[1]),
                          profile::microtesla_per_gauss * double(m_mag.magnetometer_ga[2]))};
        if (m_time_us - sample.time_us < magnetic_type::timing.maximum_interval_us && sample.source.device_id != 0U &&
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
    auto const result = m_magnetic_aiding.update(m_time_us, sample, context, reference, m_state, m_covariance,
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
