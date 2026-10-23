/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <algorithm>

#include <formal_eskf/eskf/configuration/ins.hpp>
#include <formal_eskf/eskf/types.hpp>
#include <formal_eskf/linalg/operations.hpp>
#include <formal_eskf/so3/operations.hpp>

namespace formal_eskf::example
{

struct ReplayImuFlags
{
    int at_rest = -1;
    int accelerometer_clipping = -1;
};

/** Decaying IMU peaks and per-axis learning permissions for one replay session. */
template <typename Linalg> class ReplayBiasLearning
{
public:
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    using state_type = configuration::Ins::NominalState<Linalg>;

    struct Parameters
    {
        value_type maximum_specific_force{}; // m/s^2, including gravity.
        value_type maximum_angular_rate{};   // rad/s.
        value_type decay_time_constant{};    // s; not a recovery deadline.
        bool accelerometer_enabled = true;
        bool gyroscope_enabled = true;
    };

    struct Conditions
    {
        bool at_rest = false;
        bool fake_height = false;
        bool fake_position = false;
        bool bad_vertical_acceleration = false;
        int accelerometer_clipping = 0; // Three body-axis bits; adapter resolves missing metadata.
    };

    struct Result
    {
        Status status = Status::success;
        bool high_dynamics = true;
        value_type specific_force_peak{};
        value_type angular_rate_peak{};
        configuration::Ins::BiasUpdate bias_update{{false, false, false}, {false, false, false}};
    };

    explicit ReplayBiasLearning(Parameters const & parameters) : m_parameters(parameters) {}

    /**
     * Evaluate before prediction, using the prior unit q_nb and biases. The
     * replay input has synchronized accel/gyro periods in seconds. All current
     * corrections receive this result; no separate continuous-good timer.
     * Conditions describe actual modes/faults, not time since aiding. Failure
     * preserves both envelopes and returns disabled permissions. A new session
     * starts with zero peaks; missing IMU intervals never erase a measured peak.
     */
    [[nodiscard]] Result update(value_type dt, state_type const & state, ImuSample<Linalg> const & imu,
                                Conditions const & conditions) noexcept
    {
        Result result;
        if (!scalar::is_finite<math_type>(dt) || !scalar::is_finite<math_type>(m_parameters.maximum_specific_force) ||
            !scalar::is_finite<math_type>(m_parameters.maximum_angular_rate) ||
            !scalar::is_finite<math_type>(m_parameters.decay_time_constant) ||
            !linalg::all_finite(imu.specific_force_b) || !linalg::all_finite(imu.angular_rate_b) ||
            !linalg::all_finite(state.b_a) || !linalg::all_finite(state.b_g))
        {
            result.status = Status::non_finite_input;
            return result;
        }
        if (!(dt > value_type{0}) || !(m_parameters.maximum_specific_force > value_type{0}) ||
            !(m_parameters.maximum_angular_rate > value_type{0}) ||
            !(m_parameters.decay_time_constant > value_type{0}) || conditions.accelerometer_clipping < 0 ||
            conditions.accelerometer_clipping > 7)
        {
            result.status = Status::domain_error;
            return result;
        }
        auto const force = linalg::norm(imu.specific_force_b - state.b_a);
        auto const rate = linalg::norm(imu.angular_rate_b - state.b_g);
        auto const R = so3::to_rotation_matrix(state.q_nb);
        if (!scalar::is_finite<math_type>(force) || !scalar::is_finite<math_type>(rate) || !linalg::all_finite(R))
        {
            result.status = Status::non_finite_result;
            return result;
        }
        // Equivalent to 1 - clamp(dt / tau, 0, 1), without overflowing dt / tau.
        auto const decay = dt >= m_parameters.decay_time_constant
                               ? value_type{0}
                               : value_type{1} - dt / m_parameters.decay_time_constant;
        result.specific_force_peak = std::max(force, decay * m_specific_force_peak);
        result.angular_rate_peak = std::max(rate, decay * m_angular_rate_peak);
        result.high_dynamics = result.specific_force_peak > m_parameters.maximum_specific_force ||
                               result.angular_rate_peak > m_parameters.maximum_angular_rate;
        bool const accel_enabled =
            m_parameters.accelerometer_enabled && !result.high_dynamics && !conditions.bad_vertical_acceleration;
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            bool const observable =
                conditions.at_rest ||
                (!conditions.fake_height && (!conditions.fake_position || scalar::absolute<math_type>(R(2U, axis)) >
                                                                              static_cast<value_type>(0.966)));
            result.bias_update.accelerometer[axis] =
                accel_enabled && observable && (conditions.accelerometer_clipping & (1 << axis)) == 0;
            result.bias_update.gyroscope[axis] = m_parameters.gyroscope_enabled;
        }
        m_specific_force_peak = result.specific_force_peak;
        m_angular_rate_peak = result.angular_rate_peak;
        return result;
    }

private:
    Parameters m_parameters;
    value_type m_specific_force_peak{};
    value_type m_angular_rate_peak{};
}; /* end class ReplayBiasLearning */

} /* end namespace formal_eskf::example */
