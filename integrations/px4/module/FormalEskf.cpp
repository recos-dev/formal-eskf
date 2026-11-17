// SPDX-License-Identifier: BSD-3-Clause
#include "FormalEskf.hpp"

#include <algorithm>
#include <cmath>
#include <new>
#include <px4_platform_common/param.h>

namespace profile = formal_eskf::px4::profile;

// Dense binary64 operations use a dedicated SITL stack; target budgets are separate.
static constexpr px4::wq_config_t formal_eskf_queue{"wq:formal_eskf", profile::work_queue_stack_bytes,
                                                    profile::work_queue_relative_priority};

FormalEskf::types_type::parameter_type FormalEskf::estimator_parameters()
{
    types_type::parameter_type result;
    result.gravity_n = vector3(0., 0., profile::gravity_m_s2);
    result.minimum_quaternion_norm = profile::minimum_quaternion_norm;
    result.quaternion_squared_norm_tolerance = profile::quaternion_squared_norm_tolerance;
    result.dt_min = profile::imu::minimum_interval_s;
    result.dt_max = profile::imu::maximum_interval_s;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        result.process_noise.specific_force_variance(axis) = profile::imu::specific_force_variance;
        result.process_noise.angular_rate_variance(axis) = profile::imu::angular_rate_variance;
        result.process_noise.accelerometer_bias_random_walk_variance_density(axis) =
            profile::imu::accelerometer_bias_variance_density;
        result.process_noise.gyroscope_bias_random_walk_variance_density(axis) =
            profile::imu::gyroscope_bias_variance_density;
    }
    return result;
}

FormalEskf::bias_learning_type::Parameters FormalEskf::bias_learning_parameters()
{
    bias_learning_type::Parameters result;
    result.maximum_specific_force = profile::imu::maximum_specific_force_m_s2;
    result.maximum_angular_rate = profile::imu::maximum_angular_rate_rad_s;
    result.decay_time_constant = profile::imu::bias_learning_decay_s;
    result.accelerometer_enabled = profile::imu::accelerometer_bias_learning;
    result.gyroscope_enabled = profile::imu::gyroscope_bias_learning;
    return result;
}

FormalEskf::FormalEskf() : ScheduledWorkItem(MODULE_NAME, formal_eskf_queue) {}

FormalEskf::~FormalEskf()
{
    m_imu_sub.unregisterCallback();
    perf_free(m_cycle_perf);
    perf_free(m_output_perf);
    perf_free(m_fusion_perf);
    if (m_diagnostics_ready)
    {
        pthread_mutex_destroy(&m_diagnostics_mutex);
    }
}

bool FormalEskf::load_timing_parameters()
{
    formal_eskf::px4::TimingParameters timing;
    if (param_get(param_find("FESKF_DT_US"), &timing.filter_interval_us) != 0 ||
        param_get(param_find("FESKF_DELAY_MS"), &timing.fusion_delay_ms) != 0 ||
        param_get(param_find("FESKF_GPS_DLY"), &timing.gnss_delay_ms) != 0 ||
        param_get(param_find("FESKF_MAG_DLY"), &timing.magnetic_delay_ms) != 0 ||
        param_get(param_find("FESKF_BARO_DLY"), &timing.barometer_delay_ms) != 0 || !m_imu_history.configure(timing) ||
        !m_observations.configure(timing) || !m_output.configure(timing))
    {
        PX4_ERR("unsupported delayed-fusion timing settings");
        return false;
    }
    return true;
}

bool FormalEskf::load_magnetic_parameters()
{
    int32_t declination_source{};
    float saved_declination{};
    if (param_get(param_find("FESKF_DECL_TYPE"), &declination_source) != 0 ||
        param_get(param_find("FESKF_MAG_DECL"), &saved_declination) != 0 || declination_source < 0 ||
        declination_source > 3 || !std::isfinite(saved_declination) || std::abs(saved_declination) > 180.f)
    {
        PX4_ERR("invalid magnetic declination settings");
        return false;
    }
    m_magnetic_settings = {static_cast<std::uint32_t>(declination_source),
                           math::radians(static_cast<double>(saved_declination))};
    return true;
}

bool FormalEskf::load_sensor_positions()
{
    char const * const imu_names[] = {"FESKF_IMU_POS_X", "FESKF_IMU_POS_Y", "FESKF_IMU_POS_Z"};
    char const * const gnss_names[] = {"FESKF_GPS_POS_X", "FESKF_GPS_POS_Y", "FESKF_GPS_POS_Z"};
    SensorPositions positions;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        float imu{}, gnss{};
        if (param_get(param_find(imu_names[axis]), &imu) != 0 || param_get(param_find(gnss_names[axis]), &gnss) != 0 ||
            !std::isfinite(imu) || !std::isfinite(gnss) || std::abs(imu) > profile::maximum_sensor_position_m ||
            std::abs(gnss) > profile::maximum_sensor_position_m)
        {
            PX4_ERR("invalid sensor positions");
            return false;
        }
        positions.imu_b(axis) = imu;
        positions.gnss_b(axis) = gnss;
    }
    m_sensor_positions = positions;
    return true;
}

bool FormalEskf::load_startup_parameters()
{
    if (!load_timing_parameters())
    {
        return false;
    }
    int32_t gravity_enabled{};
    if (param_get(param_find("FESKF_GRAV_EN"), &gravity_enabled) != 0 || gravity_enabled < 0 || gravity_enabled > 1)
    {
        PX4_ERR("invalid gravity aiding setting");
        return false;
    }
    m_gravity_enabled = profile::gravity::enabled && gravity_enabled != 0;
    return load_magnetic_parameters() && load_sensor_positions();
}

bool FormalEskf::init()
{
    if (!m_diagnostics_ready)
    {
        m_diagnostics_ready = pthread_mutex_init(&m_diagnostics_mutex, nullptr) == 0;
    }
    if (!m_diagnostics_ready || m_cycle_perf == nullptr || m_output_perf == nullptr || m_fusion_perf == nullptr ||
        !load_startup_parameters() || !prepare_publications())
    {
        return false;
    }
    // Complete the initial snapshot before a sensor callback can schedule Run().
    update_diagnostics(hrt_absolute_time());
    if (!m_imu_sub.registerCallback())
    {
        return false;
    }
    ScheduleNow();
    return true;
}

bool FormalEskf::recent(uint64_t sample, uint64_t max_age) const
{
    return sample != 0 && sample <= m_time_us && m_time_us - sample <= max_age;
}

bool FormalEskf::check(status_type status)
{
    if (formal_eskf::succeeded(status))
    {
        return true;
    }
    ++m_errors;
    m_fault = true; // Latch numerical failures; no silent reset or fallback.
    PX4_ERR("numerical failure %d at %llu", static_cast<int>(status), static_cast<unsigned long long>(m_time_us));
    return false;
}

void FormalEskf::Initialization::reset_samples()
{
    imu_samples = 0U;
    accelerometer_sum = {};
    gyroscope_sum = {};
    magnetic_sum_gauss = {};
    magnetic_samples = 0U;
}

bool FormalEskf::collect_initialization_samples(imu_type const & imu)
{
    double const accel_norm = formal_eskf::linalg::norm(imu.specific_force_b);
    if (m_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED ||
        std::abs(accel_norm - profile::gravity_m_s2) > profile::startup::gravity_tolerance_m_s2 ||
        formal_eskf::linalg::norm(imu.angular_rate_b) > profile::startup::maximum_angular_rate_rad_s)
    {
        m_initialization.reset_samples();
        return false;
    }
    if (m_initialization.imu_samples < profile::startup::maximum_imu_samples)
    {
        m_initialization.accelerometer_sum = m_initialization.accelerometer_sum + imu.specific_force_b;
        m_initialization.gyroscope_sum = m_initialization.gyroscope_sum + imu.angular_rate_b;
        ++m_initialization.imu_samples;
    }
    if (m_mag.timestamp_sample > m_initialization.magnetic_time_us &&
        recent(m_mag.timestamp_sample, profile::startup::maximum_sample_age_us))
    {
        if (m_mag.device_id != m_initialization.magnetic_source.device_id ||
            m_mag.calibration_count != m_initialization.magnetic_source.calibration_count)
        {
            m_initialization.magnetic_sum_gauss = {};
            m_initialization.magnetic_samples = 0;
        }
        m_initialization.magnetic_source = {m_mag.device_id, m_mag.calibration_count, 1U};
        auto const m = vector3(m_mag.magnetometer_ga[0], m_mag.magnetometer_ga[1], m_mag.magnetometer_ga[2]);
        if (m_mag.device_id != 0 && formal_eskf::linalg::all_finite(m) &&
            formal_eskf::linalg::norm(m) > profile::startup::minimum_magnetic_field_gauss)
        {
            m_initialization.magnetic_sum_gauss = m_initialization.magnetic_sum_gauss + m;
            ++m_initialization.magnetic_samples;
        }
        m_initialization.magnetic_time_us = m_mag.timestamp_sample;
    }
    return true;
}

bool FormalEskf::initialization_ready() const
{
    if (m_initialization.imu_samples < profile::startup::minimum_imu_samples ||
        m_initialization.magnetic_samples < profile::startup::minimum_magnetic_samples || !gps_good() ||
        !recent(m_baro.timestamp_sample, profile::startup::maximum_sample_age_us) ||
        !std::isfinite(m_baro.baro_alt_meter) ||
        std::hypot(m_gps.vel_n_m_s, m_gps.vel_e_m_s) > profile::startup::maximum_horizontal_speed_m_s ||
        std::abs(m_gps.vel_d_m_s) > profile::startup::maximum_vertical_speed_m_s)
    {
        return false;
    }
    return true;
}

bool FormalEskf::initialize_attitude(double & yaw)
{
    auto const accel = m_initialization.accelerometer_sum / static_cast<double>(m_initialization.imu_samples);
    auto const mag = m_initialization.magnetic_sum_gauss / static_cast<double>(m_initialization.magnetic_samples);
    double const roll = std::atan2(-accel(1), -accel(2));
    double const pitch = std::atan2(accel(0), std::hypot(accel(1), accel(2)));
    const matrix::Dcm<double> tilt{matrix::Euler<double>(roll, pitch, 0.)};
    const matrix::Vector3<double> level_mag = tilt * matrix::Vector3<double>(mag(0), mag(1), mag(2));
    if (profile::microtesla_per_gauss * std::hypot(level_mag(0), level_mag(1)) <
        profile::magnetic::field.minimum_horizontal_field)
    {
        return false;
    }
    auto const model = lookup_magnetic_model(m_gps.latitude_deg, m_gps.longitude_deg);
    m_mag_model_unavailable += !model.has_value();
    if (!check(m_magnetic_model.initialize(m_time_us, m_magnetic_settings,
                                           vector3(profile::microtesla_per_gauss * level_mag(0),
                                                   profile::microtesla_per_gauss * level_mag(1),
                                                   profile::microtesla_per_gauss * level_mag(2)),
                                           model)))
    {
        return false;
    }
    yaw = m_magnetic_model.declination() - std::atan2(level_mag(1), level_mag(0));
    const matrix::Quaternion<double> q{matrix::Euler<double>(roll, pitch, yaw)};
    if (!check(state_type::quaternion_type::from_coefficients(q(0), q(1), q(2), q(3), profile::minimum_quaternion_norm,
                                                              m_state.q_nb)))
    {
        return false;
    }
    m_state.b_g = m_initialization.gyroscope_sum / static_cast<double>(m_initialization.imu_samples);
    if (!check(formal_eskf::runtime::limit_bias(m_state, profile::bias_limits<double>())))
    {
        return false;
    }
    return true;
}

void FormalEskf::initialize_navigation(imu_type const & imu)
{
    // The geographic reference remains the first antenna fix; the state is at the IMU.
    m_state.p_n = -gnss_position_offset();
    m_state.v_n = vector3(m_gps.vel_n_m_s, m_gps.vel_e_m_s, m_gps.vel_d_m_s) - gnss_velocity_offset(imu);
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        m_covariance(axis, axis) = profile::startup::position_variance[axis];
        m_covariance(3U + axis, 3U + axis) = profile::startup::velocity_variance[axis];
        m_covariance(6U + axis, 6U + axis) = profile::startup::attitude_variance[axis];
        m_covariance(9U + axis, 9U + axis) = profile::startup::accelerometer_bias_variance;
        m_covariance(12U + axis, 12U + axis) = profile::startup::gyroscope_bias_variance;
    }
    m_origin.time_us = m_time_us;
    m_origin.projection.initReference(m_gps.latitude_deg, m_gps.longitude_deg, m_origin.time_us);
    m_mag_model_origin_time = m_origin.time_us;
    m_origin.altitude_msl_m = m_gps.altitude_msl_m;
    m_origin.altitude_ellipsoid_m = m_gps.altitude_ellipsoid_m;
}

void FormalEskf::initialize(ImuInput const & sample, imu_type const & imu)
{
    if (!collect_initialization_samples(imu) || !initialization_ready())
    {
        return;
    }
    // Invalid external altitude delays startup; it is not a numerical-core fault.
    if (!formal_eskf::succeeded(m_height_aiding.initialize(m_time_us, m_gps.altitude_msl_m)))
    {
        return;
    }

    double yaw{};
    if (!initialize_attitude(yaw))
    {
        return;
    }
    initialize_navigation(imu);
    m_initialization.magnetic_source.reference_id = m_magnetic_model.reference_id();
    initialize_magnetic_aiding(m_initialization.magnetic_source, m_mag.timestamp_sample);
    m_initialized = true;
    if (!update_imu_bias(sample, imu))
    {
        return;
    }
    fuse_gps(imu);
    if (!m_fault && update_imu_health(sample.accelerometer_clipping))
    {
        fuse_height();
    }
    observation_history_type::Seed seed;
    seed.context = startup_context(m_time_us, profile::gnss::context_maximum_age_us);
    seed.gnss = m_gnss.input.sample.stamp;
    seed.magnetic = decode_magnetic(m_mag).stamp;
    seed.barometer = m_barometer.sample.stamp;
    if (m_fault || !check_timing(m_imu_history.seed(sample, hrt_absolute_time()) && m_observations.seed(seed) &&
                                 m_output.seed(m_state, m_time_us)))
    {
        return;
    }
    PX4_INFO("initialized from IMU + magnetometer + GNSS; yaw=%.2f deg, EKF2 replacement", math::degrees(yaw));
}

FormalEskf::InputUpdates FormalEskf::read_inputs()
{
    InputUpdates updates;
    auto const gps_generation = m_gps_sub.get_last_generation();
    auto const mag_generation = m_mag_sub.get_last_generation();
    auto const baro_generation = m_baro_sub.get_last_generation();
    auto const imu_generation = m_imu_sub.get_last_generation();
    updates.gps = m_gps_sub.update(&m_gps);
    updates.magnetic = m_mag_sub.update(&m_mag);
    updates.barometer = m_baro_sub.update(&m_baro);
    m_land_sub.update(&m_land);
    m_vehicle_status_sub.update(&m_vehicle_status);

    updates.imu = m_imu_sub.update(&updates.sample);
    m_generation_loss += updates.gps ? unsigned(m_gps_sub.get_last_generation() - gps_generation - 1) : 0;
    m_generation_loss += updates.magnetic ? unsigned(m_mag_sub.get_last_generation() - mag_generation - 1) : 0;
    m_generation_loss += updates.barometer ? unsigned(m_baro_sub.get_last_generation() - baro_generation - 1) : 0;
    m_generation_loss += updates.imu ? unsigned(m_imu_sub.get_last_generation() - imu_generation - 1) : 0;
    return updates;
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
    ScheduleDelayed(profile::backup_interval_us); // Status timeout, not an independent watchdog.
    perf_begin(m_cycle_perf);
    auto const updates = read_inputs();
    bool const was_initialized = m_initialized;
    if (updates.imu)
    {
        // Admission/current propagation and fast attitude precede delayed work.
        perf_begin(m_output_perf);
        process_imu(decode_imu(updates.sample));
        perf_end(m_output_perf);
    }
    uint64_t const now = hrt_absolute_time();
    if (was_initialized && !m_fault)
    {
        if (updates.gps)
        {
            record_admission(m_observations.push_gnss(decode_gnss(m_gps), now));
        }
        if (updates.magnetic)
        {
            record_admission(m_observations.push_magnetic(decode_magnetic(m_mag), now));
        }
        if (updates.barometer)
        {
            record_admission(m_observations.push_barometer(decode_barometer(m_baro), now));
        }
        if (updates.imu)
        {
            advance_fusion();
        }
    }
    if (now - m_last_status >= profile::status_interval_us)
    {
        publish_status(now);
        m_last_status = now;
    }
    perf_end(m_cycle_perf);
    update_diagnostics(now);
}

FormalEskf::ImuInput FormalEskf::decode_imu(vehicle_imu_s const & sample)
{
    ImuInput result;
    result.delta_angle_b =
        vector3(static_cast<double>(sample.delta_angle[0]), static_cast<double>(sample.delta_angle[1]),
                static_cast<double>(sample.delta_angle[2]));
    result.delta_velocity_b =
        vector3(static_cast<double>(sample.delta_velocity[0]), static_cast<double>(sample.delta_velocity[1]),
                static_cast<double>(sample.delta_velocity[2]));
    result.time_us = sample.timestamp_sample;
    result.angular_period_us = sample.delta_angle_dt;
    result.acceleration_period_us = sample.delta_velocity_dt;
    result.accelerometer_clipping = sample.delta_velocity_clipping;
    result.gyroscope_clipping = sample.delta_angle_clipping;
    result.source.accelerometer_id = sample.accel_device_id;
    result.source.gyroscope_id = sample.gyro_device_id;
    result.source.accelerometer_calibration = sample.accel_calibration_count;
    result.source.gyroscope_calibration = sample.gyro_calibration_count;
    return result;
}

bool FormalEskf::check_timing(bool succeeded)
{
    if (!succeeded)
    {
        ++m_errors;
        m_fault = true;
        PX4_ERR("history/timing failure; restart required");
    }
    return succeeded;
}

void FormalEskf::record_admission(observation_history_type::PushResult const & result)
{
    using Admission = observation_history_type::Admission;
    m_arrival_rejections += result.decision != Admission::queued;
    m_rate_rejections += result.decision == Admission::too_fast;
    m_stale_arrivals += result.decision == Admission::stale || result.decision == Admission::imu_stale;
    m_source_discards += result.source_discarded;
    m_overflows += result.overwritten;
}

FormalEskf::observation_history_type::LandState FormalEskf::land_state() const
{
    return {m_land.timestamp, m_land.timestamp != 0, m_land.landed, m_land.at_rest, m_land.in_ground_effect};
}

FormalEskf::observation_history_type::VehicleState FormalEskf::vehicle_state() const
{
    return {m_vehicle_status.timestamp, m_vehicle_status.timestamp != 0,
            m_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED,
            m_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING};
}

FormalEskf::observation_history_type::Context FormalEskf::startup_context(std::uint64_t epoch,
                                                                          std::uint64_t maximum_age) const
{
    observation_history_type::Context context{epoch, land_state(), vehicle_state()};
    context.land.known &= context.land.time_us <= epoch && epoch - context.land.time_us < maximum_age;
    context.vehicle.known &= context.vehicle.time_us <= epoch && epoch - context.vehicle.time_us < maximum_age;
    return context;
}

bool FormalEskf::accept_imu(ImuInput const & sample)
{
    auto const now = hrt_absolute_time();
    if (sample.time_us <= m_time_us || sample.time_us > now || now - sample.time_us > profile::output::maximum_age_us ||
        sample.angular_period_us < profile::history::minimum_period_us ||
        sample.angular_period_us > profile::history::maximum_period_us ||
        sample.acceleration_period_us < profile::history::minimum_period_us ||
        sample.acceleration_period_us > profile::history::maximum_period_us ||
        !formal_eskf::px4::imu_clipping_acceptable(sample.accelerometer_clipping, sample.gyroscope_clipping, false))
    {
        ++m_rejections;
        // Revoke permissions and partial observations without consuming the horizon.
        m_bias_learning_result = {};
        reset_imu_observations();
        return false;
    }
    return true;
}

void FormalEskf::reset_imu_observations()
{
    m_vertical_health.reset();
    m_gravity.reset_samples();
    m_average_imu_interval = profile::imu::nominal_interval_s;
}

void FormalEskf::process_imu(ImuInput const & sample)
{
    if (m_fault)
    {
        return;
    }
    if (!m_initialized)
    {
        if (!accept_imu(sample))
        {
            return;
        }
        auto const imu = sample.mean_rates();
        if (!formal_eskf::linalg::all_finite(imu.specific_force_b) ||
            !formal_eskf::linalg::all_finite(imu.angular_rate_b))
        {
            ++m_rejections;
            return;
        }
        m_time_us = sample.time_us;
        m_imu_context = startup_context(m_time_us, profile::imu::rest_maximum_age_us);
        m_gnss.input = {m_gps.timestamp_sample, 0, 0, 0, decode_gnss(m_gps)};
        m_gnss.context = startup_context(m_gnss.input.time_us, profile::gnss::context_maximum_age_us);
        m_barometer = {m_baro.timestamp_sample, 0, 0, 0, decode_barometer(m_baro)};
        update_gps_quality();
        if (!m_fault)
        {
            initialize(sample, imu);
        }
        return;
    }
    if (!check_timing(m_imu_history.push(sample, hrt_absolute_time())))
    {
        return;
    }
    auto const & interval = *m_imu_history.latest();
    if (!check_timing(m_observations.record_imu(interval, land_state(), vehicle_state()) && m_output.predict(interval)))
    {
        return;
    }
    if (output_validity(hrt_absolute_time()).healthy)
    {
        publish_attitude(hrt_absolute_time());
    }
}

void FormalEskf::advance_fusion()
{
    auto const * interval = m_imu_history.delayed();
    if (interval == nullptr || m_fault)
    {
        return;
    }
    perf_begin(m_fusion_perf);
    predict_and_fuse(*interval);
    if (!m_fault && check_timing(m_output.correct(m_time_us, m_state) && m_imu_history.complete(m_time_us)))
    {
        publish(m_imu_history.latest()->sample.mean_rates());
    }
    perf_end(m_fusion_perf);
}

void FormalEskf::select_observations()
{
    // A missed pop must not reuse an old event. GNSS data may be retained for
    // diagnostics, but update_gps_quality's watermark prevents a second use.
    auto const gps = m_observations.pop_gnss(m_gnss.input);
    m_magnetic.input = {};
    auto const magnetic = m_observations.pop_magnetic(m_magnetic.input);
    m_barometer = {};
    auto const barometer = m_observations.pop_barometer(m_barometer);
    m_retired_observations += gps.retired + magnetic.retired + barometer.retired;
    m_gnss.context = m_observations.context(m_gnss.input.time_us, profile::gnss::context_maximum_age_us);
    m_magnetic.context = m_observations.context(m_magnetic.input.time_us != 0 ? m_magnetic.input.time_us : m_time_us,
                                                profile::magnetic::context_maximum_age_us);
    update_gps_quality();
}

void FormalEskf::predict_and_fuse(imu_history_type::Interval const & interval)
{
    auto const & sample = interval.sample;
    auto const imu = sample.mean_rates();
    m_imu_context = m_observations.prediction_context(interval, profile::imu::rest_maximum_age_us);
    if (interval.hold_period_us != 0)
    {
        ++m_gaps;
        reset_imu_observations();
        m_magnetic.heading.reset_acceleration();
        if (!check(formal_eskf::px4::predict_imu(m_state, m_covariance, interval.held_rate,
                                                 interval.hold_period_us * profile::seconds_per_microsecond,
                                                 m_parameters, 0, false)))
        {
            return;
        }
    }
    if (!update_imu_bias(sample, imu) ||
        !check(formal_eskf::px4::predict_imu(m_state, m_covariance, imu, sample.angular_interval_s(), m_parameters,
                                             sample.accelerometer_clipping,
                                             m_vertical_health.state().bad_vertical_acceleration)))
    {
        return;
    }
    m_time_us = sample.time_us;
    ++m_predictions;
    if (!check_timing(m_observations.begin_fusion(interval)))
    {
        return;
    }
    select_observations();
    auto const acceleration_n = formal_eskf::so3::rotate(m_state.q_nb, imu.specific_force_b - m_state.b_a);
    if (m_fault || !check(m_magnetic.heading.update_acceleration(acceleration_n, sample.acceleration_interval_s())))
    {
        return;
    }
    fuse_gps(imu);
    // Current delayed GNSS velocity and preceding height evidence feed health.
    if (m_fault || !update_imu_health(sample.accelerometer_clipping))
    {
        return;
    }
    update_ground_effect();
    fuse_height();
    if (m_fault)
    {
        return;
    }
    fuse_gravity(sample, imu);
    if (!m_fault)
    {
        fuse_mag();
    }
}

bool FormalEskf::reset_output(quaternion_type const & delta_q, vector3_type const & delta_v,
                              vector3_type const & delta_p)
{
    // Alignment corrections precede output seeding. Afterwards every state
    // reset must transform the current output and its retained history.
    if (m_output.current().time_us == 0)
    {
        return true;
    }
    auto const before = body_output();
    if (!check_timing(m_output.apply_reset(delta_q, delta_v, delta_p)))
    {
        return false;
    }
    auto const & q = delta_q.coefficients();
    if ((q(1) != 0. || q(2) != 0. || q(3) != 0.) && formal_eskf::linalg::squared_norm(m_sensor_positions.imu_b) > 0.)
    {
        // A heading reset also moves an offset output point. Translation resets
        // are recorded by their callers; add only the reference-point change here.
        auto const after = body_output();
        auto const position_delta = after.position_n - before.position_n - delta_p;
        auto const velocity_delta = after.velocity_n - before.velocity_n - delta_v;
        // Count the point reset once even when an individual component is zero.
        m_resets.record_position(m_time_us, vector2_type::from_row_major({position_delta(0), position_delta(1)}));
        m_resets.record_height(m_time_us, position_delta(2), 1);
        m_resets.record_velocity(m_time_us, velocity_delta);
        m_resets.record_vertical_velocity(m_time_us, velocity_delta(2));
    }
    return true;
}

bool FormalEskf::update_imu_bias(ImuInput const & sample, imu_type const & imu)
{
    double const angular_interval = sample.angular_interval_s();
    double const force_interval = sample.acceleration_interval_s();
    m_average_imu_interval = profile::imu::interval_history_weight * m_average_imu_interval +
                             profile::imu::interval_sample_weight *
                                 std::clamp(0.5 * (angular_interval + force_interval),
                                            profile::imu::minimum_interval_ratio * profile::imu::nominal_interval_s,
                                            profile::imu::maximum_interval_ratio * profile::imu::nominal_interval_s);
    bias_learning_type::Conditions conditions;
    conditions.at_rest = m_imu_context.land.known && m_imu_context.land.at_rest;
    conditions.accelerometer_clipping = sample.accelerometer_clipping;
    conditions.bad_vertical_acceleration = m_vertical_health.state().bad_vertical_acceleration;
    // This module has no fake-position/height modes. A GNSS outage is not a fake-aiding mode.
    m_bias_learning_result = m_bias_learning.update(angular_interval, force_interval, m_state, imu, conditions);
    return check(m_bias_learning_result.status);
}

bool FormalEskf::update_imu_health(std::uint8_t accelerometer_clipping)
{
    vertical_health_type::Observations observations;
    observations.barometer_height = m_height_aiding.vertical_innovation(height_type::Source::barometer);
    observations.gnss_height = m_height_aiding.vertical_innovation(height_type::Source::gnss);
    observations.gnss_velocity_down = m_gnss.vertical_innovation;
    observations.gnss_velocity_down.active &= m_gnss.lifecycle.state().velocity.active;
    vertical_health_type::Sample health_sample;
    health_sample.time_us = m_time_us;
    health_sample.average_interval_s = m_average_imu_interval;
    health_sample.clipping_available = true;
    for (size_t axis = 0; axis < 3; ++axis)
    {
        health_sample.clipped[axis] = (accelerometer_clipping & (1U << axis)) != 0;
    }
    // Current GNSS velocity, preceding height attempts: evaluate before current height correction.
    return check(m_vertical_health.update(health_sample, observations));
}

int FormalEskf::task_spawn(int argc, char * argv[])
{
    FormalEskf * instance = new (std::nothrow) FormalEskf();
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
