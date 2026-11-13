/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <algorithm>
#include <array>

#include <formal_eskf/eskf/configuration/ins.hpp>
#include <formal_eskf/eskf/types.hpp>
#include <formal_eskf/linalg/operations.hpp>
#include <formal_eskf/scalar/math.hpp>
#include <formal_eskf/so3/operations.hpp>

namespace formal_eskf::runtime
{

/** Decaying IMU peaks and per-axis learning permissions for one estimator session. */
template <typename Linalg> class ImuBiasInhibition
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

    explicit ImuBiasInhibition(Parameters const & parameters) : m_parameters(parameters) {}

    /**
     * Evaluate before prediction, using the prior unit q_nb and biases. The
     * accel/gyro integration periods are supplied separately in seconds. All current
     * corrections receive this result; no separate continuous-good timer.
     * Conditions describe actual modes/faults, not time since aiding. Failure
     * preserves both envelopes and returns disabled permissions. A new session
     * starts with zero peaks; missing IMU intervals never erase a measured peak.
     */
    [[nodiscard]] Result update(value_type angular_interval, value_type force_interval, state_type const & state,
                                ImuSample<Linalg> const & imu, Conditions const & conditions);

private:
    value_type decayed_peak(value_type magnitude, value_type previous_peak, value_type interval) const;
    configuration::Ins::BiasUpdate learning_permissions(linalg::Matrix<Linalg, 3U, 3U> const & R, bool high_dynamics,
                                                        Conditions const & conditions) const;

    Parameters m_parameters;
    value_type m_specific_force_peak{};
    value_type m_angular_rate_peak{};
}; /* end class ImuBiasInhibition */

template <typename Linalg>
typename ImuBiasInhibition<Linalg>::value_type
ImuBiasInhibition<Linalg>::decayed_peak(value_type magnitude, value_type previous_peak, value_type interval) const
{
    // Validated positive intervals/tau: avoid overflowing interval / tau.
    auto const decay = interval >= m_parameters.decay_time_constant
                           ? value_type{0}
                           : value_type{1} - interval / m_parameters.decay_time_constant;
    return std::max(magnitude, decay * previous_peak);
}

template <typename Linalg>
configuration::Ins::BiasUpdate ImuBiasInhibition<Linalg>::learning_permissions(linalg::Matrix<Linalg, 3U, 3U> const & R,
                                                                               bool high_dynamics,
                                                                               Conditions const & conditions) const
{
    configuration::Ins::BiasUpdate permissions;
    bool const accel_enabled =
        m_parameters.accelerometer_enabled && !high_dynamics && !conditions.bad_vertical_acceleration;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        bool observable = true;
        if (conditions.at_rest)
        {
            observable = true;
        }
        else if (conditions.fake_height)
        {
            observable = false;
        }
        else if (conditions.fake_position)
        {
            observable = scalar::absolute<math_type>(R(2U, axis)) > static_cast<value_type>(0.966);
        }
        permissions.accelerometer[axis] =
            accel_enabled && observable && (conditions.accelerometer_clipping & (1 << axis)) == 0;
        permissions.gyroscope[axis] = m_parameters.gyroscope_enabled;
    }
    return permissions;
}

template <typename Linalg>
typename ImuBiasInhibition<Linalg>::Result
ImuBiasInhibition<Linalg>::update(value_type angular_interval, value_type force_interval, state_type const & state,
                                  ImuSample<Linalg> const & imu, Conditions const & conditions)
{
    Result result;
    if (!scalar::is_finite<math_type>(angular_interval) || !scalar::is_finite<math_type>(force_interval) ||
        !scalar::is_finite<math_type>(m_parameters.maximum_specific_force) ||
        !scalar::is_finite<math_type>(m_parameters.maximum_angular_rate) ||
        !scalar::is_finite<math_type>(m_parameters.decay_time_constant) || !linalg::all_finite(imu.specific_force_b) ||
        !linalg::all_finite(imu.angular_rate_b) || !linalg::all_finite(state.b_a) || !linalg::all_finite(state.b_g))
    {
        result.status = Status::non_finite_input;
        return result;
    }
    if (!(angular_interval > value_type{0}) || !(force_interval > value_type{0}) ||
        !(m_parameters.maximum_specific_force > value_type{0}) ||
        !(m_parameters.maximum_angular_rate > value_type{0}) || !(m_parameters.decay_time_constant > value_type{0}) ||
        conditions.accelerometer_clipping < 0 || conditions.accelerometer_clipping > 7)
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
    result.specific_force_peak = decayed_peak(force, m_specific_force_peak, force_interval);
    result.angular_rate_peak = decayed_peak(rate, m_angular_rate_peak, angular_interval);
    result.high_dynamics = result.specific_force_peak > m_parameters.maximum_specific_force ||
                           result.angular_rate_peak > m_parameters.maximum_angular_rate;
    result.bias_update = learning_permissions(R, result.high_dynamics, conditions);

    m_specific_force_peak = result.specific_force_peak;
    m_angular_rate_peak = result.angular_rate_peak;
    return result;
}

struct ImuNoiseConditions
{
    bool bad_vertical_acceleration = false;       // Preceding estimator-horizon verdict.
    std::array<bool, 3U> accelerometer_clipped{}; // Known body-axis flags for this IMU interval.
};

/**
 * Replace sampled accelerometer variance on faulty/clipped body axes.
 * A vertical fault selects every axis; otherwise clipping selects each axis.
 * fault_variance is already squared (m^2/s^4), not a noise density or bias
 * driving intensity. Selection replaces the value, even if the old one is larger.
 * Preserve gyro variance and both bias intensities. Apply bias-noise inhibition
 * afterward, using these selected measurement variances as its thresholds.
 *
 * All noise coefficients and fault_variance must be finite and nonnegative,
 * including coefficients being replaced. Zero is allowed. Failure preserves
 * output; success permits a whole-noise alias. Start each step from configured
 * noise, not the preceding selected output. The caller owns metadata availability.
 */
template <typename Linalg>
[[nodiscard]] Status
select_imu_process_noise(configuration::Ins::ProcessNoise<Linalg> const & noise, ImuNoiseConditions const & conditions,
                         typename Linalg::value_type fault_variance, configuration::Ins::ProcessNoise<Linalg> & output)
{
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    if (!scalar::is_finite<math_type>(fault_variance))
    {
        return Status::non_finite_input;
    }
    if (fault_variance < value_type{0})
    {
        return Status::domain_error;
    }
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        value_type const coefficients[] = {noise.specific_force_variance(axis), noise.angular_rate_variance(axis),
                                           noise.accelerometer_bias_random_walk_variance_density(axis),
                                           noise.gyroscope_bias_random_walk_variance_density(axis)};
        for (auto const coefficient : coefficients)
        {
            if (!scalar::is_finite<math_type>(coefficient))
            {
                return Status::non_finite_input;
            }
            if (coefficient < value_type{0})
            {
                return Status::domain_error;
            }
        }
    }
    auto candidate = noise;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        if (conditions.bad_vertical_acceleration || conditions.accelerometer_clipped[axis])
        {
            candidate.specific_force_variance(axis) = fault_variance;
        }
    }
    output = candidate;
    return Status::success;
}

/**
 * Select per-step INS bias driving noise using the prior covariance.
 * Keep an axis's intensity only while its prior bias variance is strictly below
 * the corresponding sampled IMU variance. Equality inhibits the increment;
 * this does not clamp P or depend on correction's bias-learning permissions.
 *
 * INS bias transition rows are identity and measurement noise has no bias rows,
 * so these prior diagonals equal their propagated values before bias noise.
 * Covariance must be symmetric PSD; only its six bias diagonals are consumed.
 * All consumed values must be finite and nonnegative, including noise that will
 * be inhibited. The core retains its existing Q_bias = intensity * dt scaling.
 * Success may publish in place over noise; every failure preserves output.
 * Derive each step from the original noise, so inhibition can later be released.
 */
template <typename Linalg>
[[nodiscard]] Status inhibit_bias_process_noise(linalg::Matrix<Linalg, 15U, 15U> const & covariance,
                                                configuration::Ins::ProcessNoise<Linalg> const & noise,
                                                configuration::Ins::ProcessNoise<Linalg> & output)
{
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        value_type const coefficients[] = {covariance(9U + axis, 9U + axis),
                                           covariance(12U + axis, 12U + axis),
                                           noise.specific_force_variance(axis),
                                           noise.angular_rate_variance(axis),
                                           noise.accelerometer_bias_random_walk_variance_density(axis),
                                           noise.gyroscope_bias_random_walk_variance_density(axis)};
        for (auto const coefficient : coefficients)
        {
            if (!scalar::is_finite<math_type>(coefficient))
            {
                return Status::non_finite_input;
            }
            if (coefficient < value_type{0})
            {
                return Status::domain_error;
            }
        }
    }
    auto candidate = noise;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        if (covariance(9U + axis, 9U + axis) >= noise.specific_force_variance(axis))
        {
            candidate.accelerometer_bias_random_walk_variance_density(axis) = value_type{0};
        }
        if (covariance(12U + axis, 12U + axis) >= noise.angular_rate_variance(axis))
        {
            candidate.gyroscope_bias_random_walk_variance_density(axis) = value_type{0};
        }
    }
    output = candidate;
    return Status::success;
}

/** Symmetric per-axis bounds, not vector-norm bounds. Zero forces zero bias. */
template <typename Scalar> struct BiasLimits
{
    Scalar accelerometer{}; // m/s^2.
    Scalar gyroscope{};     // rad/s.
};

/**
 * Explicitly limit the six INS bias coefficients in place after a correction.
 * Finite, nonnegative limits and finite biases are required. Validate all inputs
 * before changing state; failure preserves it. Other state fields are untouched.
 * This operation neither changes covariance nor consults learning permissions:
 * an already out-of-range bias is limited even when its learning is inhibited.
 */
template <typename Linalg>
[[nodiscard]] Status limit_bias(configuration::Ins::NominalState<Linalg> & state,
                                BiasLimits<typename Linalg::value_type> limits)
{
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    if (!scalar::is_finite<math_type>(limits.accelerometer) || !scalar::is_finite<math_type>(limits.gyroscope) ||
        !linalg::all_finite(state.b_a) || !linalg::all_finite(state.b_g))
    {
        return Status::non_finite_input;
    }
    if (limits.accelerometer < value_type{0} || limits.gyroscope < value_type{0})
    {
        return Status::domain_error;
    }
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        state.b_a(axis) = std::clamp(state.b_a(axis), -limits.accelerometer, limits.accelerometer);
        state.b_g(axis) = std::clamp(state.b_g(axis), -limits.gyroscope, limits.gyroscope);
    }
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
