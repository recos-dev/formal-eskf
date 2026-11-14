// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <cstdint>
#include <numbers>

#include <formal_eskf/runtime/imu.hpp>
#include <formal_eskf/runtime/gnss_lifecycle.hpp>
#include <formal_eskf/runtime/magnetic_field.hpp>
#include <formal_eskf/runtime/magnetometer_lifecycle.hpp>

namespace formal_eskf::px4::profile
{

// Fixed deployment settings. PX4 startup parameters override only gravity
// enablement and magnetic declination selection; these are not EKF2 parameters.
inline constexpr double gravity_m_s2 = 9.80665;
inline constexpr double minimum_quaternion_norm = 1e-6;
inline constexpr double quaternion_squared_norm_tolerance = 1e-10;
inline constexpr double seconds_per_microsecond = 1e-6;
inline constexpr double microtesla_per_gauss = 100.;

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
inline constexpr double specific_force_variance = 1.;   // m^2/s^4, per sample.
inline constexpr double angular_rate_variance = 0.0025; // rad^2/s^2, per sample.
inline constexpr double faulty_specific_force_variance = 4.9 * 4.9;
inline constexpr double accelerometer_bias_variance_density = 1e-6;
inline constexpr double gyroscope_bias_variance_density = 1e-8;
inline constexpr double maximum_specific_force_m_s2 = 25.;
inline constexpr double maximum_angular_rate_rad_s = 3.;
inline constexpr double bias_learning_decay_s = 0.5;
} // namespace imu

namespace gravity
{
inline constexpr std::uint64_t observation_interval_us = 10000U;
} // namespace gravity

namespace magnetic
{
inline constexpr runtime::MagneticFieldParameters<double> field{20., std::numbers::pi / 9., 0., 5.};
inline constexpr runtime::MagneticFieldChecks checks{true, false, false};
inline constexpr runtime::MagnetometerFusion::Parameters timing{1000000U, 500000U, 7000000U};
inline constexpr double heading_noise_rad = 0.3;
inline constexpr double acceleration_limit_m_s2 = 0.5;
inline constexpr double noise_variance = 25.; // (0.05 gauss)^2 in microtesla^2.
inline constexpr double gate_sigma = 3.;
inline constexpr double maximum_heading_variance = 0.1;
inline constexpr std::uint64_t maximum_heading_age_us = 1000000U;
inline constexpr std::uint64_t context_maximum_age_us = 3000000U;
inline constexpr std::uint64_t model_interval_us = 10000000U;
} // namespace magnetic

namespace gnss
{
// Horizontal lifecycle only; the height coordinator arbitrates GNSS/barometer.
inline constexpr runtime::GnssFusion::Parameters timing{7000000U, 1000000U, 5000000U};
inline constexpr std::uint64_t maximum_sample_age_us = 500000U;
inline constexpr std::uint64_t context_maximum_age_us = 3000000U;
inline constexpr std::uint64_t maximum_aiding_age_us = 5000000U;
inline constexpr double minimum_velocity_noise_m_s = 0.3;
inline constexpr double vertical_variance_multiplier = 2.25;
inline constexpr double minimum_position_noise_m = 0.5;
inline constexpr double maximum_position_noise_m = 10.;
inline constexpr double gate_sigma = 5.;
} // namespace gnss

namespace height
{
inline constexpr std::uint64_t fusion_timeout_us = 5000000U;
inline constexpr std::uint64_t barometer_timeout_us = 400000U;
inline constexpr std::uint64_t gnss_timeout_us = 1000000U;
inline constexpr std::uint64_t barometer_maximum_age_us = 200000U;
inline constexpr std::uint64_t gnss_maximum_age_us = 500000U;
inline constexpr double minimum_altitude_m = -12000.;
inline constexpr double maximum_altitude_m = 100000.;
inline constexpr double barometer_variance = 3.5 * 3.5;
inline constexpr double minimum_gnss_noise_m = 0.75;
inline constexpr double maximum_aided_gnss_variance = 100.;
inline constexpr double gate_sigma = 5.;
inline constexpr double barometer_lpf_gain = 0.1;
inline constexpr unsigned barometer_warmup_samples = 20U;
inline constexpr double reset_velocity_variance = 10.;
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
    return {static_cast<Scalar>(0.4), static_cast<Scalar>(0.15)}; // m/s^2, rad/s.
}

} /* end namespace formal_eskf::px4::profile */
