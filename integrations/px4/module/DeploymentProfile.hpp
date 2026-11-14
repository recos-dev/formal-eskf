// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <cstdint>
#include <numbers>

#include <formal_eskf/runtime/imu.hpp>
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
inline constexpr std::uint64_t context_maximum_age_us = 3000000U;
inline constexpr std::uint64_t model_interval_us = 10000000U;
} // namespace magnetic

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
