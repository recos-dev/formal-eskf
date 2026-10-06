/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include <formal_eskf/eskf/configuration/ins.hpp>
#include <formal_eskf/status.hpp>

namespace formal_eskf::runtime
{

/**
 * Per-axis INS bias-learning qualification, driven by caller-supplied IMU time.
 * Health, motion detection and observability belong to the caller; this monitor
 * only applies immediate inhibition and continuous-good recovery timing.
 * No estimator state, covariance or process-noise parameter is modified.
 */
class ImuBiasLearning
{
public:
    struct SensorConditions
    {
        bool healthy = false;             // Explicit caller judgment: safe for bias learning.
        std::array<bool, 3U> permitted{}; // Configuration AND aiding/observability permission, in body axes.
        std::array<bool, 3U> clipped{};   // Clipping observed since the preceding policy update.
    };

    struct Conditions
    {
        SensorConditions accelerometer{};
        SensorConditions gyroscope{};
        bool high_dynamics = false; // Inhibits all accelerometer bias axes, not gyroscope bias by itself.
    };

    struct Parameters
    {
        std::uint64_t recovery_time_us{}; // Both intervals must be explicitly set to positive values.
        std::uint64_t maximum_sample_interval_us{};
    };

    struct Result
    {
        Status status = Status::success; // A valid policy evaluation, not permission to fuse a measurement.
        configuration::Ins::BiasUpdate bias_update{{false, false, false}, {false, false, false}};
    };

    /** Also required on IMU/source/calibration or body-frame changes. */
    void reset() noexcept { *this = ImuBiasLearning{}; }

    /**
     * Call once per new IMU sample, before corrections at that fusion horizon.
     * Startup, a long sample gap or changed parameters restart qualification.
     * Invalid parameters/time return disabled permissions and clear all history:
     * an invalid call must not carry an old learning authorization forward.
     * Do not reuse the result at later horizons without updating this monitor.
     */
    [[nodiscard]] Result update(std::uint64_t time_us, Conditions const & conditions,
                                Parameters const & parameters) noexcept;

private:
    [[nodiscard]] static bool update_axis(bool eligible, std::uint64_t time_us, std::uint64_t recovery_time_us,
                                          std::uint64_t & good_since_us) noexcept;

    std::array<std::uint64_t, 3U> m_accelerometer_good_since{};
    std::array<std::uint64_t, 3U> m_gyroscope_good_since{};
    std::uint64_t m_last_sample_time{};
    Parameters m_parameters{};
}; /* end class ImuBiasLearning */

inline bool ImuBiasLearning::update_axis(bool eligible, std::uint64_t time_us, std::uint64_t recovery_time_us,
                                         std::uint64_t & good_since_us) noexcept
{
    if (!eligible)
    {
        good_since_us = 0U;
        return false;
    }
    if (good_since_us == 0U)
    {
        good_since_us = time_us;
    }
    return time_us - good_since_us >= recovery_time_us;
}

inline ImuBiasLearning::Result ImuBiasLearning::update(std::uint64_t time_us, Conditions const & conditions,
                                                       Parameters const & parameters) noexcept
{
    Result result;
    if (parameters.recovery_time_us == 0U || parameters.maximum_sample_interval_us == 0U)
    {
        reset();
        result.status = Status::domain_error;
        return result;
    }
    if (time_us == 0U || time_us <= m_last_sample_time)
    {
        reset();
        result.status = Status::out_of_range;
        return result;
    }

    bool const sample_gap =
        m_last_sample_time != 0U && time_us - m_last_sample_time > parameters.maximum_sample_interval_us;
    if (sample_gap || parameters.recovery_time_us != m_parameters.recovery_time_us ||
        parameters.maximum_sample_interval_us != m_parameters.maximum_sample_interval_us)
    {
        reset();
    }
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        bool const accelerometer_eligible = conditions.accelerometer.healthy && !conditions.high_dynamics &&
                                            conditions.accelerometer.permitted[axis] &&
                                            !conditions.accelerometer.clipped[axis];
        bool const gyroscope_eligible =
            conditions.gyroscope.healthy && conditions.gyroscope.permitted[axis] && !conditions.gyroscope.clipped[axis];
        result.bias_update.accelerometer[axis] =
            update_axis(accelerometer_eligible, time_us, parameters.recovery_time_us, m_accelerometer_good_since[axis]);
        result.bias_update.gyroscope[axis] =
            update_axis(gyroscope_eligible, time_us, parameters.recovery_time_us, m_gyroscope_good_since[axis]);
    }
    m_last_sample_time = time_us;
    m_parameters = parameters;
    return result;
}

} /* end namespace formal_eskf::runtime */
