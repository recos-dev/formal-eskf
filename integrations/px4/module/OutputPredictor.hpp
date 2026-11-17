// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include <formal_eskf/eskf/prediction.hpp>

#include "ImuHistory.hpp"

namespace formal_eskf::px4
{

/**
 * Current-time INS output with a matching, fixed-capacity history. No covariance,
 * sensors, clock or publication is owned here. See PX4-TIME-OUTPUT for the
 * tracking equations, reset convention and differences from the pinned PX4.
 * Use the same configuration and seed epoch as ImuHistory. Predict from latest(),
 * then correct from the successfully completed main state at delayed().
 */
template <typename Linalg> class OutputPredictor
{
public:
    using value_type = typename Linalg::value_type;
    using vector3_type = linalg::Vector<Linalg, 3>;
    using quaternion_type = so3::UnitQuaternion<Linalg>;
    using state_type = configuration::Ins::NominalState<Linalg>;
    using interval_type = typename ImuHistory<Linalg>::Interval;

    struct Sample
    {
        std::uint64_t time_us{};
        std::uint64_t start_time_us{};
        std::uint32_t angular_period_us{};
        quaternion_type q_nb{};
        vector3_type v_n{};
        vector3_type p_n{};
    };

    struct TrackingError
    {
        vector3_type attitude_b{}; // Shortest-sign -2*q_error.vector, a small-angle proxy.
        vector3_type velocity_n{}; // m/s
        vector3_type position_n{}; // m
    };

    enum class Fault
    {
        none,
        epoch_mismatch,
        history_loss,
        lag_out_of_range,
        invalid_state,
        numerical_failure
    };

    [[nodiscard]] bool configure(TimingParameters const & parameters);
    [[nodiscard]] bool seed(state_type const & state, std::uint64_t time_us);
    /** Requires an admitted real interval from ImuHistory::latest(), exactly once. */
    [[nodiscard]] bool predict(interval_type const & interval);
    /** Requires successful main prediction/fusion at the exact oldest output epoch. */
    [[nodiscard]] bool correct(std::uint64_t time_us, state_type const & state);
    /** q_new = q_delta * q_old; additive NED deltas. Caller owns reset counters. */
    [[nodiscard]] bool apply_reset(quaternion_type const & q_delta, vector3_type const & velocity_delta_n,
                                   vector3_type const & position_delta_n);
    /** Session restart clears history, feedback and fault, retaining configuration. */
    void reset();

    Sample const & current() const { return m_current; }
    /** Body rate used by the latest propagation, including bias/tracking feedback. */
    vector3_type const & angular_rate_b() const { return m_output_angular_rate_b; }
    /** Non-consuming view, null during warmup, after correction or on fault. */
    Sample const * delayed() const;
    TrackingError const & tracking_error() const { return m_error; }
    Fault fault() const { return m_fault; }
    std::size_t entries() const { return m_history.entries(); }
    std::uint64_t fusion_time_us() const { return m_fusion_time_us; }
    /** Readiness does not establish wall-clock freshness or controller acceptance. */
    bool ready() const { return m_tracking && m_fault == Fault::none; }

private:
    static bool valid_quaternion(quaternion_type const & q);
    static bool valid_state(state_type const & state);
    bool advance(Sample & output, ImuSample<Linalg> imu, value_type dt) const;
    bool stage_translation(vector3_type const & velocity_delta, vector3_type const & position_delta);
    void commit_history();
    bool fail(Fault reason);

    runtime::RingBuffer<Sample, profile::history::imu_capacity> m_history{0};
    // Persistent workspace keeps failure-atomic history updates off the callback stack.
    std::array<Sample, profile::history::imu_capacity> m_staged{};
    TimingConfiguration m_configuration{};
    Sample m_current{};
    vector3_type m_accel_bias{};
    vector3_type m_gyro_bias{};
    vector3_type m_angular_rate_correction{};
    vector3_type m_output_angular_rate_b{};
    vector3_type m_velocity_error_sum{};
    vector3_type m_position_error_sum{};
    TrackingError m_error{};
    value_type m_filter_interval_s = static_cast<value_type>(profile::imu::nominal_interval_s);
    std::uint64_t m_fusion_time_us{};
    bool m_tracking = false;
    Fault m_fault = Fault::none;
}; /* end class OutputPredictor */

template <typename Linalg> bool OutputPredictor<Linalg>::configure(TimingParameters const & parameters)
{
    TimingConfiguration candidate;
    if (m_current.time_us != 0 || m_fault != Fault::none || !configure_timing(parameters, candidate) ||
        !m_history.allocate(candidate.imu_length))
    {
        return false;
    }
    m_configuration = candidate;
    return true;
}

template <typename Linalg> bool OutputPredictor<Linalg>::valid_quaternion(quaternion_type const & q)
{
    auto const residual = linalg::squared_norm(q.coefficients()) - value_type{1};
    auto const tolerance = static_cast<value_type>(profile::quaternion_squared_norm_tolerance);
    return residual >= -tolerance && residual <= tolerance;
}

template <typename Linalg> bool OutputPredictor<Linalg>::valid_state(state_type const & state)
{
    return valid_quaternion(state.q_nb) && linalg::all_finite(state.v_n) && linalg::all_finite(state.p_n) &&
           linalg::all_finite(state.b_a) && linalg::all_finite(state.b_g);
}

template <typename Linalg> bool OutputPredictor<Linalg>::seed(state_type const & state, std::uint64_t time_us)
{
    if (!m_history.valid() || m_current.time_us != 0 || m_fault != Fault::none || time_us == 0 || !valid_state(state))
    {
        return false;
    }
    m_current = {time_us, time_us, 0, state.q_nb, state.v_n, state.p_n};
    m_accel_bias = state.b_a;
    m_gyro_bias = state.b_g;
    m_fusion_time_us = time_us;
    return true;
}

template <typename Linalg> bool OutputPredictor<Linalg>::fail(Fault reason)
{
    m_fault = reason;
    return false;
}

template <typename Linalg>
bool OutputPredictor<Linalg>::advance(Sample & output, ImuSample<Linalg> imu, value_type dt) const
{
    state_type state{output.p_n, output.v_n, output.q_nb, m_accel_bias, m_gyro_bias};
    configuration::Ins::Parameters<Linalg> parameters;
    parameters.gravity_n(2) = static_cast<value_type>(profile::gravity_m_s2);
    parameters.minimum_quaternion_norm = static_cast<value_type>(profile::minimum_quaternion_norm);
    parameters.quaternion_squared_norm_tolerance = static_cast<value_type>(profile::quaternion_squared_norm_tolerance);
    parameters.dt_min = static_cast<value_type>(profile::imu::minimum_interval_s);
    parameters.dt_max = static_cast<value_type>(profile::imu::maximum_interval_s);
    imu.angular_rate_b = imu.angular_rate_b + m_angular_rate_correction;
    if (!succeeded(predict_nominal(state, imu, dt, parameters, state)))
    {
        return false;
    }
    output.q_nb = state.q_nb;
    output.v_n = state.v_n;
    output.p_n = state.p_n;
    return true;
}

template <typename Linalg> bool OutputPredictor<Linalg>::predict(interval_type const & interval)
{
    if (m_current.time_us == 0 || m_fault != Fault::none)
    {
        return false;
    }
    if (interval.start_time_us != m_current.time_us || interval.time_us != interval.sample.time_us ||
        interval.time_us <= interval.start_time_us)
    {
        return fail(Fault::epoch_mismatch);
    }
    bool const full = m_history.entries() == m_history.get_length();
    if (full && m_history.get_oldest().time_us > m_fusion_time_us)
    {
        return fail(Fault::history_loss);
    }
    if (m_history.entries() + 1 >= m_history.get_length())
    {
        auto index = m_history.get_oldest_index();
        if (full)
        {
            index = (index + 1) % m_history.get_length();
        }
        auto const lag = interval.time_us - m_history[index].time_us;
        if (lag < m_configuration.minimum_lag_us || lag > profile::history::maximum_lag_us)
        {
            return fail(Fault::lag_out_of_range);
        }
    }
    auto candidate = m_current;
    if (interval.hold_period_us != 0 && !advance(candidate, interval.held_rate,
                                                 static_cast<value_type>(interval.hold_period_us) *
                                                     static_cast<value_type>(profile::seconds_per_microsecond)))
    {
        return fail(Fault::numerical_failure);
    }
    auto const imu = interval.sample.mean_rates();
    if (!advance(candidate, imu, interval.sample.angular_interval_s()))
    {
        return fail(Fault::numerical_failure);
    }
    candidate.time_us = interval.time_us;
    candidate.start_time_us = interval.start_time_us;
    candidate.angular_period_us = interval.sample.angular_period_us;
    m_history.push(candidate);
    m_current = candidate;
    m_output_angular_rate_b = imu.angular_rate_b + m_angular_rate_correction - m_gyro_bias;
    return true;
}

template <typename Linalg> typename OutputPredictor<Linalg>::Sample const * OutputPredictor<Linalg>::delayed() const
{
    if (m_fault != Fault::none || !m_history.valid() || m_history.entries() != m_history.get_length())
    {
        return nullptr;
    }
    auto const & sample = m_history.get_oldest();
    return sample.time_us > m_fusion_time_us ? &sample : nullptr;
}

template <typename Linalg>
bool OutputPredictor<Linalg>::stage_translation(vector3_type const & velocity_delta,
                                                vector3_type const & position_delta)
{
    auto index = m_history.get_oldest_index();
    for (std::size_t count = 0; count < m_history.entries(); ++count)
    {
        auto & candidate = m_staged[index];
        candidate = m_history[index];
        candidate.v_n = candidate.v_n + velocity_delta;
        candidate.p_n = candidate.p_n + position_delta;
        if (!linalg::all_finite(candidate.v_n) || !linalg::all_finite(candidate.p_n))
        {
            return false;
        }
        index = (index + 1) % m_history.get_length();
    }
    return true;
}

template <typename Linalg> void OutputPredictor<Linalg>::commit_history()
{
    auto index = m_history.get_oldest_index();
    for (std::size_t count = 0; count < m_history.entries(); ++count)
    {
        m_history[index] = m_staged[index];
        index = (index + 1) % m_history.get_length();
    }
    m_current = m_history.get_newest();
}

template <typename Linalg> bool OutputPredictor<Linalg>::correct(std::uint64_t time_us, state_type const & state)
{
    if (m_current.time_us == 0 || m_fault != Fault::none)
    {
        return false;
    }
    auto const * sample = delayed();
    if (sample == nullptr || sample->time_us != time_us || sample->start_time_us != m_fusion_time_us)
    {
        return fail(Fault::epoch_mismatch);
    }
    if (!valid_state(state))
    {
        return fail(Fault::invalid_state);
    }
    quaternion_type q_error;
    auto const difference = state.q_nb.inverse() * sample->q_nb;
    if (!succeeded(quaternion_type::from_coefficients(
            difference.coefficients(), static_cast<value_type>(profile::minimum_quaternion_norm), q_error)))
    {
        return fail(Fault::numerical_failure);
    }
    value_type const sign_scale = q_error.q0() < value_type{0} ? value_type{2} : value_type{-2};
    TrackingError error{vector3_type::from_row_major({q_error.q1(), q_error.q2(), q_error.q3()}) * sign_scale,
                        state.v_n - sample->v_n, state.p_n - sample->p_n};
    auto const lag_s = static_cast<value_type>(m_current.time_us - time_us) *
                       static_cast<value_type>(profile::seconds_per_microsecond);
    // A rate, not a per-callback angle: held and real segments receive their own dt.
    auto const angular_rate_correction =
        error.attitude_b * (static_cast<value_type>(profile::output::attitude_tracking_gain) / lag_s);
    auto const interval_s = static_cast<value_type>(profile::output::interval_history_weight) * m_filter_interval_s +
                            static_cast<value_type>(profile::output::interval_sample_weight) *
                                static_cast<value_type>(sample->angular_period_us) *
                                static_cast<value_type>(profile::seconds_per_microsecond);
    // The supported 3.8--4.2 ms profile stays below both fixed 0.25 s time constants.
    auto const velocity_gain = interval_s / static_cast<value_type>(profile::output::velocity_time_constant_s);
    auto const position_gain = interval_s / static_cast<value_type>(profile::output::position_time_constant_s);
    auto const velocity_sum = m_velocity_error_sum + error.velocity_n;
    auto const position_sum = m_position_error_sum + error.position_n;
    auto const velocity_delta = error.velocity_n * velocity_gain +
                                velocity_sum * (velocity_gain * velocity_gain *
                                                static_cast<value_type>(profile::output::integral_tracking_gain));
    auto const position_delta = error.position_n * position_gain +
                                position_sum * (position_gain * position_gain *
                                                static_cast<value_type>(profile::output::integral_tracking_gain));
    if (!linalg::all_finite(angular_rate_correction) || !linalg::all_finite(velocity_sum) ||
        !linalg::all_finite(position_sum) || !stage_translation(velocity_delta, position_delta))
    {
        return fail(Fault::numerical_failure);
    }
    commit_history();
    m_accel_bias = state.b_a;
    m_gyro_bias = state.b_g;
    m_angular_rate_correction = angular_rate_correction;
    m_velocity_error_sum = velocity_sum;
    m_position_error_sum = position_sum;
    m_filter_interval_s = interval_s;
    m_error = error;
    m_fusion_time_us = time_us;
    m_tracking = true;
    return true;
}

template <typename Linalg>
bool OutputPredictor<Linalg>::apply_reset(quaternion_type const & q_delta, vector3_type const & velocity_delta_n,
                                          vector3_type const & position_delta_n)
{
    if (m_current.time_us == 0 || m_fault != Fault::none)
    {
        return false;
    }
    if (!valid_quaternion(q_delta) || !linalg::all_finite(velocity_delta_n) || !linalg::all_finite(position_delta_n))
    {
        return fail(Fault::invalid_state);
    }
    // Before the first interval, stage the seed in one workspace slot instead.
    bool const seed_only = m_history.entries() == 0;
    auto const count = seed_only ? std::size_t{1} : m_history.entries();
    auto index = m_history.get_oldest_index();
    for (std::size_t record = 0; record < count; ++record)
    {
        auto & candidate = m_staged[index];
        candidate = seed_only ? m_current : m_history[index];
        candidate.v_n = candidate.v_n + velocity_delta_n;
        candidate.p_n = candidate.p_n + position_delta_n;
        auto const rotated = q_delta * candidate.q_nb;
        if (!succeeded(quaternion_type::from_coefficients(
                rotated.coefficients(), static_cast<value_type>(profile::minimum_quaternion_norm), candidate.q_nb)) ||
            !linalg::all_finite(candidate.v_n) || !linalg::all_finite(candidate.p_n))
        {
            return fail(Fault::numerical_failure);
        }
        index = (index + 1) % m_history.get_length();
    }
    if (!seed_only)
    {
        commit_history();
    }
    else
    {
        m_current = m_staged[m_history.get_oldest_index()];
    }
    // Main and output receive the same transform: tracking errors and their
    // integrals are invariant. Bias feedback changes only at correct().
    return true;
}

template <typename Linalg> void OutputPredictor<Linalg>::reset()
{
    m_history.reset();
    m_staged.fill(Sample{});
    m_current = {};
    m_accel_bias = {};
    m_gyro_bias = {};
    m_angular_rate_correction = {};
    m_output_angular_rate_b = {};
    m_velocity_error_sum = {};
    m_position_error_sum = {};
    m_error = {};
    m_filter_interval_s = static_cast<value_type>(profile::imu::nominal_interval_s);
    m_fusion_time_us = 0;
    m_tracking = false;
    m_fault = Fault::none;
}

} /* end namespace formal_eskf::px4 */
