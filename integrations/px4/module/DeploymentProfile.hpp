// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <numbers>

#include <formal_eskf/runtime/imu.hpp>
#include <formal_eskf/runtime/gnss_lifecycle.hpp>
#include <formal_eskf/runtime/gnss_quality.hpp>

#include "DeploymentConfig.hpp"
#include <formal_eskf/runtime/magnetic_field.hpp>
#include <formal_eskf/runtime/magnetometer_lifecycle.hpp>

namespace formal_eskf::px4::profile
{

// Build-time deployment settings, with explicitly bound FESKF startup parameters.
// Live EKF2 parameter values are not consumed by this module.
inline constexpr double gravity_m_s2 = 9.80665;
inline constexpr double minimum_quaternion_norm = 1e-6;
inline constexpr double quaternion_squared_norm_tolerance = 1e-10;
inline constexpr double seconds_per_microsecond = 1e-6;
inline constexpr double microtesla_per_gauss = 100.;
inline constexpr float maximum_sensor_position_m = 10.f; // Per-axis small-vehicle installation envelope.

namespace startup
{
inline constexpr double gravity_tolerance_m_s2 = 0.5;
inline constexpr double maximum_angular_rate_rad_s = 0.1;
inline constexpr double minimum_magnetic_field_gauss = 0.1;
inline constexpr unsigned minimum_imu_samples = 250U;
inline constexpr unsigned maximum_imu_samples = 1000U;
inline constexpr unsigned minimum_magnetic_samples = 20U;
inline constexpr std::uint64_t maximum_sample_age_us = 500000U;
inline constexpr float maximum_horizontal_speed_m_s = 0.8f;
inline constexpr float maximum_vertical_speed_m_s = 0.5f;
// Diagonal error-state variances: p_n, v_n, theta_b, b_a, b_g.
inline constexpr std::array<double, 3U> position_variance{1., 1., 2.25};
inline constexpr std::array<double, 3U> velocity_variance{0.09, 0.09, 0.09};
inline constexpr std::array<double, 3U> attitude_variance{0.01, 0.01, 0.04};
inline constexpr double accelerometer_bias_variance = 0.04;
inline constexpr double gyroscope_bias_variance = 1e-4;
} // namespace startup

namespace imu
{
inline constexpr double minimum_interval_s = 1e-6;
inline constexpr double maximum_interval_s = 0.05;
inline constexpr double nominal_interval_s = 0.004; // vehicle_imu: 250 Hz.
inline constexpr std::uint64_t maximum_interval_us = 50000U;
inline constexpr std::int64_t gap_tolerance_us = 200;
inline constexpr std::uint64_t rest_maximum_age_us = 2000000U;
inline constexpr double interval_history_weight = 0.99;
inline constexpr double interval_sample_weight = 0.01;
inline constexpr double minimum_interval_ratio = 0.5;
inline constexpr double maximum_interval_ratio = 2.;
inline constexpr double specific_force_variance = ESKF_CFG_ACCEL_VARIANCE; // m^2/s^4, per sample.
inline constexpr double angular_rate_variance = ESKF_CFG_GYRO_VARIANCE;    // rad^2/s^2, per sample.
inline constexpr double faulty_specific_force_variance = ESKF_CFG_FAULT_ACCEL_STD * ESKF_CFG_FAULT_ACCEL_STD;
inline constexpr double accelerometer_bias_variance_density = ESKF_CFG_ACCEL_BIAS_DENSITY;
inline constexpr double gyroscope_bias_variance_density = ESKF_CFG_GYRO_BIAS_DENSITY;
inline constexpr double maximum_specific_force_m_s2 = ESKF_CFG_BIAS_MAX_FORCE;
inline constexpr double maximum_angular_rate_rad_s = ESKF_CFG_BIAS_MAX_RATE;
inline constexpr double bias_learning_decay_s = ESKF_CFG_BIAS_DECAY_S;
inline constexpr bool accelerometer_bias_learning = ESKF_CFG_ACCEL_BIAS_LEARNING;
inline constexpr bool gyroscope_bias_learning = ESKF_CFG_GYRO_BIAS_LEARNING;
} // namespace imu

// First delayed-fusion profile: one real SIH IMU interval per main-filter step.
namespace history
{
inline constexpr std::int32_t filter_interval_us = 4000;
inline constexpr std::int32_t fusion_delay_ms = 200;
inline constexpr std::uint32_t minimum_period_us = 3800;
inline constexpr std::uint32_t maximum_period_us = 4200;
inline constexpr std::uint64_t maximum_lag_us = 300000;
inline constexpr std::uint64_t gnss_arrival_age_us = 150000;
inline constexpr std::uint64_t magnetic_arrival_age_us = 100000;
inline constexpr std::uint64_t barometer_arrival_age_us = 100000;
inline constexpr std::size_t imu_capacity = 64;
inline constexpr std::size_t context_capacity = 128;
inline constexpr std::size_t gnss_capacity = 16;
inline constexpr std::size_t magnetic_capacity = 64;
inline constexpr std::size_t barometer_capacity = 128;
inline constexpr std::uint64_t observation_maximum_age_us = 100000;
inline constexpr std::uint64_t gnss_spacing_us = 50000;
inline constexpr std::uint64_t magnetic_spacing_us = 10000;
inline constexpr std::uint64_t barometer_spacing_us = 5000;
inline constexpr std::size_t storage_budget_bytes = 64 * 1024;
} // namespace history

namespace gravity
{
inline constexpr bool enabled = ESKF_CFG_GRAVITY_AIDING;
inline constexpr std::uint64_t observation_interval_us = 10000U;
} // namespace gravity

namespace magnetic
{
inline constexpr bool use_model = ESKF_CFG_WMM;
inline constexpr runtime::MagneticFieldParameters<double> field{ESKF_CFG_MAG_STRENGTH_TOL, ESKF_CFG_MAG_INCLINATION_TOL,
                                                                0., ESKF_CFG_MAG_MIN_HORIZONTAL};
inline constexpr runtime::MagneticFieldChecks checks{ESKF_CFG_MAG_CHECK_STRENGTH, ESKF_CFG_MAG_CHECK_INCLINATION,
                                                     ESKF_CFG_MAG_REQUIRE_WMM};
inline constexpr runtime::MagnetometerFusion::Parameters timing{std::uint64_t{ESKF_CFG_MAG_HEALTH_MS} * 1000U,
                                                                std::uint64_t{ESKF_CFG_MAG_INTERVAL_MS} * 1000U,
                                                                std::uint64_t{ESKF_CFG_MAG_FUSION_TIMEOUT_MS} * 1000U};
inline constexpr double heading_noise_rad = ESKF_CFG_MAG_HEADING_NOISE;
inline constexpr double acceleration_limit_m_s2 = ESKF_CFG_MAG_ACCEL_LIMIT;
inline constexpr double noise_variance = ESKF_CFG_MAG_VARIANCE; // microtesla^2.
inline constexpr double gate_sigma = ESKF_CFG_MAG_GATE;
inline constexpr double maximum_heading_variance = 0.1;
inline constexpr std::uint64_t maximum_heading_age_us = 1000000U;
inline constexpr std::uint64_t context_maximum_age_us = 3000000U;
inline constexpr std::uint64_t model_interval_us = std::uint64_t{ESKF_CFG_WMM_INTERVAL_MS} * 1000U;
} // namespace magnetic

namespace gnss
{
inline constexpr runtime::GnssQualityParameters<double> quality{
    .enabled_checks = {.satellites = ESKF_CFG_GNSS_CHECK_SATELLITES,
                       .pdop = ESKF_CFG_GNSS_CHECK_PDOP,
                       .horizontal_accuracy = ESKF_CFG_GNSS_CHECK_HORIZONTAL_ACCURACY,
                       .vertical_accuracy = ESKF_CFG_GNSS_CHECK_VERTICAL_ACCURACY,
                       .speed_accuracy = ESKF_CFG_GNSS_CHECK_SPEED_ACCURACY,
                       .horizontal_drift = ESKF_CFG_GNSS_CHECK_HORIZONTAL_DRIFT,
                       .vertical_drift = ESKF_CFG_GNSS_CHECK_VERTICAL_DRIFT,
                       .horizontal_speed = ESKF_CFG_GNSS_CHECK_HORIZONTAL_SPEED,
                       .vertical_speed = ESKF_CFG_GNSS_CHECK_VERTICAL_SPEED,
                       .spoofed = ESKF_CFG_GNSS_CHECK_SPOOFED},
    .minimum_satellites = ESKF_CFG_GNSS_MIN_SATELLITES,
    .maximum_pdop = ESKF_CFG_GNSS_MAX_PDOP,
    .maximum_horizontal_accuracy = ESKF_CFG_GNSS_MAX_HACC,
    .maximum_vertical_accuracy = ESKF_CFG_GNSS_MAX_VACC,
    .maximum_speed_accuracy = ESKF_CFG_GNSS_MAX_SACC,
    .maximum_horizontal_drift = ESKF_CFG_GNSS_MAX_HDRIFT,
    .maximum_vertical_drift = ESKF_CFG_GNSS_MAX_VDRIFT,
    .velocity_limit = ESKF_CFG_GNSS_VELOCITY_LIMIT,
    .minimum_health_time_us = std::uint64_t{ESKF_CFG_GNSS_HEALTH_MS} * 1000U};
// Horizontal lifecycle only; the height coordinator arbitrates GNSS/barometer.
inline constexpr runtime::GnssFusion::Parameters timing{std::uint64_t{ESKF_CFG_GNSS_TIMEOUT_MS} * 1000U, 1000000U,
                                                        5000000U};
inline constexpr std::uint64_t maximum_sample_age_us = 500000U;
inline constexpr std::uint64_t context_maximum_age_us = 3000000U;
inline constexpr std::uint64_t maximum_aiding_age_us = 5000000U;
inline constexpr double minimum_velocity_noise_m_s = ESKF_CFG_GNSS_MIN_VEL_NOISE;
inline constexpr double vertical_variance_multiplier = 2.25;
inline constexpr double minimum_position_noise_m = ESKF_CFG_GNSS_MIN_POS_NOISE;
inline constexpr double maximum_position_noise_m = ESKF_CFG_GNSS_MAX_POS_NOISE;
inline constexpr double gate_sigma = ESKF_CFG_GNSS_GATE;
} // namespace gnss

namespace height
{
inline constexpr std::uint64_t fusion_timeout_us = std::uint64_t{ESKF_CFG_HEIGHT_TIMEOUT_MS} * 1000U;
inline constexpr std::uint64_t barometer_timeout_us = std::uint64_t{ESKF_CFG_BARO_TIMEOUT_MS} * 1000U;
inline constexpr std::uint64_t gnss_timeout_us = std::uint64_t{ESKF_CFG_GNSS_HEIGHT_TIMEOUT_MS} * 1000U;
inline constexpr std::uint64_t barometer_maximum_age_us = 200000U;
inline constexpr std::uint64_t gnss_maximum_age_us = 500000U;
inline constexpr double minimum_altitude_m = -12000.;
inline constexpr double maximum_altitude_m = 100000.;
inline constexpr double barometer_variance = ESKF_CFG_BARO_VARIANCE;
inline constexpr double minimum_gnss_noise_m = ESKF_CFG_GNSS_MIN_HEIGHT_NOISE;
inline constexpr double maximum_aided_gnss_variance = ESKF_CFG_GNSS_MAX_HEIGHT_VARIANCE;
inline constexpr double gate_sigma = ESKF_CFG_HEIGHT_GATE;
inline constexpr double barometer_lpf_gain = 0.1;
inline constexpr unsigned barometer_warmup_samples = 20U;
inline constexpr double reset_velocity_variance = 10.;
inline constexpr bool ground_effect_enabled = ESKF_CFG_GROUND_EFFECT;
inline constexpr double ground_effect_deadzone_m = ESKF_CFG_GROUND_DEADZONE;
inline constexpr std::uint64_t ground_effect_timeout_us = std::uint64_t{ESKF_CFG_GROUND_HOLD_MS} * 1000U;
inline constexpr std::uint64_t context_maximum_age_us = 3000000U;
} // namespace height

namespace height_bias
{
inline constexpr double initial_maximum_variance = 2.;
inline constexpr double initial_interval_s = 0.01;
inline constexpr double process_variance_density = 0.13 * 0.13;
inline constexpr double minimum_variance = 1e-8;
inline constexpr double measurement_variance_floor = 1e-4;
inline constexpr double gate_squared = 9.;
inline constexpr double offset_ratio = 0.2;
inline constexpr double offset_duration_s = 10.;
inline constexpr double offset_variance_multiplier = 1000.;
inline constexpr double ratio_filter_time_s = 10.;
} // namespace height_bias

namespace output
{
// PX4 v1.16.2 reporting thresholds; bias variance-vector norm, not per-axis sigma.
inline constexpr float maximum_bias_variance_norm = 0.1f;
inline constexpr float preflight_innovation_ratio = 0.5f;
inline constexpr double innovation_time_constant_s = 0.5;
inline constexpr double innovation_minimum_interval_s = 0.001;
inline constexpr double innovation_maximum_interval_s = 1.;
inline constexpr double innovation_ratio_limit = 4.;
inline constexpr double velocity_time_constant_s = ESKF_CFG_OUTPUT_VEL_TAU;
inline constexpr double position_time_constant_s = ESKF_CFG_OUTPUT_POS_TAU;
inline constexpr double interval_history_weight = 0.8;
inline constexpr double interval_sample_weight = 0.2;
inline constexpr double attitude_tracking_gain = 0.5;
inline constexpr double integral_tracking_gain = 0.1;
inline constexpr std::uint64_t navigation_interval_us = 10000U;
inline constexpr std::uint64_t maximum_age_us = 100000U;
inline constexpr std::uint64_t dead_reckoning_age_us = 1000000U;
} // namespace output

// SITL scheduling profile; target resource acceptance is separate.
inline constexpr std::uint16_t work_queue_stack_bytes = 60U * 1024U;
inline constexpr std::int8_t work_queue_relative_priority = -15;
inline constexpr std::uint32_t backup_interval_us = 20000U;
inline constexpr std::uint64_t status_interval_us = 100000U;

template <typename Scalar> constexpr runtime::BiasLimits<Scalar> bias_limits()
{
    return {static_cast<Scalar>(ESKF_CFG_ACCEL_BIAS_LIMIT),
            static_cast<Scalar>(ESKF_CFG_GYRO_BIAS_LIMIT)}; // m/s^2, rad/s.
}

} /* end namespace formal_eskf::px4::profile */
