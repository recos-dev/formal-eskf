// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstdint>

#include <formal_eskf/eskf/eskf.hpp>
#include <formal_eskf/runtime/bias_process_noise.hpp>
#include <formal_eskf/runtime/imu_process_noise.hpp>

namespace formal_eskf::px4
{

/** Only an initialized filter may propagate accelerometer-clipped intervals. */
[[nodiscard]] inline bool imu_clipping_acceptable(std::uint8_t accelerometer, std::uint8_t gyroscope,
                                                  bool initialized) noexcept
{
    return accelerometer <= 7U && gyroscope == 0U && (initialized || accelerometer == 0U);
}

/** One prediction; gap holds pass zero clipping/false fault and retain configured IMU noise. */
template <typename Linalg>
[[nodiscard]] Status try_predict_imu(configuration::Ins::NominalState<Linalg> & state,
                                     linalg::Matrix<Linalg, 15U, 15U> & covariance, ImuSample<Linalg> const & imu,
                                     typename Linalg::value_type dt,
                                     configuration::Ins::Parameters<Linalg> const & parameters,
                                     std::uint8_t accelerometer_clipping, bool bad_vertical_acceleration) noexcept
{
    using value_type = typename Linalg::value_type;
    if (accelerometer_clipping > 7U)
    {
        return Status::domain_error;
    }
    runtime::ImuNoiseConditions conditions;
    conditions.bad_vertical_acceleration = bad_vertical_acceleration;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        conditions.accelerometer_clipped[axis] = (accelerometer_clipping & (1U << axis)) != 0U;
    }
    // Start from configured noise each time; selection must not latch an override.
    auto step_parameters = parameters;
    constexpr value_type fault_variance = static_cast<value_type>(4.9 * 4.9); // m^2/s^4, sampled acceleration.
    auto status = runtime::try_select_imu_process_noise(parameters.process_noise, conditions, fault_variance,
                                                        step_parameters.process_noise);
    if (!succeeded(status))
    {
        return status;
    }
    status = runtime::try_inhibit_bias_process_noise(covariance, step_parameters.process_noise,
                                                     step_parameters.process_noise);
    if (!succeeded(status))
    {
        return status;
    }
    return try_predict(state, covariance, imu, dt, step_parameters, state, covariance);
}

} /* end namespace formal_eskf::px4 */
