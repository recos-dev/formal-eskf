// SPDX-License-Identifier: BSD-3-Clause

#include "boundary.hpp"
#include <feedback_steps.hpp>
using namespace output_boundary;
void verify_output_feedback_numerics(Output object, State state, Sample sample, Quaternion q, std::uint64_t time,
                                     output_steps::FeedbackValues result)
{
    owner = &object;
    capture();
    auto const before_state = state;
    auto const before_sample = sample;
    auto const before_q = q;
    auto const scale = q.q0() < Scalar{0} ? Scalar{2} : Scalar{-2};
    auto const lag = static_cast<Scalar>(integers[field::time] - time) * static_cast<Scalar>(1e-6);
    auto const period = static_cast<Scalar>(profile::output::interval_history_weight) * values[field::interval_mean] +
                        static_cast<Scalar>(profile::output::interval_sample_weight) *
                            static_cast<Scalar>(sample.angular_period_us) * static_cast<Scalar>(1e-6);
    auto const velocity_gain = period / static_cast<Scalar>(profile::output::velocity_time_constant_s);
    auto const position_gain = period / static_cast<Scalar>(profile::output::position_time_constant_s);
    auto const coefficients = q.coefficients();
    Scalar angle[3], rate[3], velocity_error[3], position_error[3], velocity_sum[3], position_sum[3];
    Scalar velocity_delta[3], position_delta[3];
    for (std::size_t i = 0; i < 3; ++i)
    {
        angle[i] = coefficients(i + 1) * scale;
        rate[i] = angle[i] * (static_cast<Scalar>(profile::output::attitude_tracking_gain) / lag);
        velocity_error[i] = state.v_n(i) - sample.v_n(i);
        position_error[i] = state.p_n(i) - sample.p_n(i);
        velocity_sum[i] = values[field::velocity_sum_x + i] + velocity_error[i];
        position_sum[i] = values[field::position_sum_x + i] + position_error[i];
        velocity_delta[i] = velocity_error[i] * velocity_gain +
                            velocity_sum[i] * (velocity_gain * velocity_gain *
                                               static_cast<Scalar>(profile::output::integral_tracking_gain));
        position_delta[i] = position_error[i] * position_gain +
                            position_sum[i] * (position_gain * position_gain *
                                               static_cast<Scalar>(profile::output::integral_tracking_gain));
    }
    output_steps::correction_numerics(object, state, &sample, q, time, result);
    __ESBMC_assert(same(result.interval_s, period), "Output feedback exact IEEE period recurrence");
    for (std::size_t i = 0; i < 3; ++i)
    {
        __ESBMC_assert(same(result.error.attitude_b(i), angle[i]) && same(result.angular_rate_correction(i), rate[i]),
                       "Output feedback shortest-sign attitude error and exact rate scaling");
        __ESBMC_assert(same(result.error.velocity_n(i), velocity_error[i]) &&
                           same(result.error.position_n(i), position_error[i]),
                       "Output feedback uses main-minus-delayed NED residuals");
        __ESBMC_assert(same(result.velocity_sum(i), velocity_sum[i]) && same(result.position_sum(i), position_sum[i]),
                       "Output feedback exact velocity and position integral updates");
        __ESBMC_assert(same(result.velocity_delta(i), velocity_delta[i]) &&
                           same(result.position_delta(i), position_delta[i]),
                       "Output feedback exact ordered PI translation coefficients");
        __ESBMC_assert(same(state.v_n(i), before_state.v_n(i)) && same(state.p_n(i), before_state.p_n(i)) &&
                           same(state.b_a(i), before_state.b_a(i)) && same(state.b_g(i), before_state.b_g(i)),
                       "Output feedback arithmetic preserves the external main state vectors");
    }
    __ESBMC_assert(same_sample(sample, before_sample), "Output feedback arithmetic preserves the delayed record");
    auto const old_state_q = before_state.q_nb.coefficients();
    auto const state_q = state.q_nb.coefficients();
    auto const old_q = before_q.coefficients();
    auto const after_q = q.coefficients();
    for (std::size_t i = 0; i < 4; ++i)
    {
        __ESBMC_assert(same(old_state_q(i), state_q(i)) && same(old_q(i), after_q(i)),
                       "Output feedback arithmetic preserves both external quaternions");
    }
    check();
}
