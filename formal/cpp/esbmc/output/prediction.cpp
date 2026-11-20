// SPDX-License-Identifier: BSD-3-Clause

#include "boundary.hpp"
namespace
{
using namespace output_boundary;
using Interval = imu_proof::Interval;
using Rates = formal_eskf::ImuSample<Backend>;
Interval expected_interval;
Rates mean;
Sample held_result, real_result, pending;
bool held_ok, real_ok;
unsigned advances, means, stores;
bool holding;
} // namespace
namespace formal_eskf::px4
{
template <> ImuSample<output_proof::Backend> ImuInput<output_proof::Backend>::mean_rates() const
{
    check();
    __ESBMC_assert(means == 0 && advances == static_cast<unsigned>(holding),
                   "Output prediction requests mean rates once after successful hold");
    __ESBMC_assert(imu_proof::same_input(*this, expected_interval.sample),
                   "Output prediction forwards every integrated input field to mean rates");
    ++means;
    return mean;
}
template <>
bool OutputPredictor<output_proof::Backend>::advance(Sample & candidate, ImuSample<output_proof::Backend> imu,
                                                     value_type dt) const
{
    check();
    __ESBMC_assert(this == owner && advances < 2 && same_sample(candidate, pending),
                   "Output prediction advances the complete staged candidate on its owner");
    bool const held = holding && advances == 0;
    auto const duration =
        held ? static_cast<Scalar>(expected_interval.hold_period_us) * static_cast<Scalar>(1e-6)
             : static_cast<Scalar>(expected_interval.sample.angular_period_us) * static_cast<Scalar>(1e-6);
    for (std::size_t i = 0; i < 3; ++i)
    {
        __ESBMC_assert(same(imu.specific_force_b(i),
                            (held ? expected_interval.held_rate.specific_force_b(i) : mean.specific_force_b(i))) &&
                           same(imu.angular_rate_b(i),
                                (held ? expected_interval.held_rate.angular_rate_b(i) : mean.angular_rate_b(i))),
                       "Output prediction passes the selected held or real rate unchanged");
    }
    __ESBMC_assert(same(dt, duration) && (held ? means == 0 : means == 1),
                   "Output prediction forwards segment duration in order");
    bool const ok = held ? held_ok : real_ok;
    if (ok)
    {
        Sample result;
        if (held)
        {
            result = held_result;
        }
        else
        {
            result = real_result;
        }
        candidate.q_nb = result.q_nb;
        candidate.v_n = result.v_n;
        candidate.p_n = result.p_n;
        pending = candidate;
    }
    ++advances;
    return ok;
}
} // namespace formal_eskf::px4
template <> void formal_eskf::runtime::RingBuffer<output_proof::Sample, 64>::push(Sample const & sample)
{
    check();
    __ESBMC_assert(this == &owner->m_history && stores == 0 && means == 1 && advances == (holding ? 2U : 1U) &&
                       real_ok && (!holding || held_ok),
                   "Output prediction publishes only after every propagation succeeds");
    pending.time_us = expected_interval.time_us;
    pending.start_time_us = expected_interval.start_time_us;
    pending.angular_period_us = expected_interval.sample.angular_period_us;
    __ESBMC_assert(same_sample(sample, pending), "Output prediction sends the full endpoint to typed storage");
    auto const length = integers[field::length];
    bool const full = integers[field::count] == length;
    integers[field::count] = full ? integers[field::count] : integers[field::count] + 1;
    integers[field::oldest] =
        full ? (integers[field::oldest] + 1 == length ? 0 : integers[field::oldest] + 1) : integers[field::oldest];
    integers[field::write] = integers[field::write] + 1 == length ? 0 : integers[field::write] + 1;
    __ESBMC_init_object(owner);
    restore_outer();
    capture_storage();
    ++stores;
}
void verify_output_prediction(Output object, Interval interval, Sample hold, Sample real, Rates rates,
                              bool hold_success, bool real_success)
{
    owner = &object;
    metadata(object);
    __ESBMC_assume(interval.sample.angular_period_us >= 3800 && interval.sample.angular_period_us <= 4200 &&
                   interval.sample.acceleration_period_us >= 3800 && interval.sample.acceleration_period_us <= 4200);
    capture();
    expected_interval = interval;
    held_result = hold;
    real_result = real;
    held_ok = hold_success;
    real_ok = real_success;
    mean = rates;
    pending = object.m_current;
    holding = interval.hold_period_us != 0;
    advances = means = stores = 0;
    auto expected_fault = fault;
    bool eligible = integers[field::time] != 0 && fault == Output::Fault::none;
    if (eligible && (interval.start_time_us != integers[field::time] || interval.time_us != interval.sample.time_us ||
                     interval.time_us <= interval.start_time_us))
    {
        eligible = false;
        expected_fault = Output::Fault::epoch_mismatch;
    }
    bool const full = integers[field::count] == integers[field::length];
    if (eligible && full && object.m_history.m_samples[integers[field::oldest]].time_us > integers[field::fusion_time])
    {
        eligible = false;
        expected_fault = Output::Fault::history_loss;
    }
    if (eligible && integers[field::count] + 1 >= integers[field::length])
    {
        auto const index =
            full ? (integers[field::oldest] + 1 == integers[field::length] ? 0 : integers[field::oldest] + 1)
                 : integers[field::oldest];
        auto const lag = interval.time_us - object.m_history.m_samples[index].time_us;
        if (lag < integers[field::minimum_lag] || lag > profile::history::maximum_lag_us)
        {
            eligible = false;
            expected_fault = Output::Fault::lag_out_of_range;
        }
    }
    bool const run_real = eligible && (!holding || held_ok);
    bool const expected = run_real && real_ok;
    if (eligible && !expected)
    {
        expected_fault = Output::Fault::numerical_failure;
    }
    bool const accepted = object.predict(interval);
    __ESBMC_assert(accepted == expected &&
                       advances == static_cast<unsigned>(eligible && holding) + static_cast<unsigned>(run_real) &&
                       means == static_cast<unsigned>(run_real) && stores == static_cast<unsigned>(expected),
                   "Output prediction exact guard precedence, segment order and publication count");
    fault = expected_fault;
    if (expected)
    {
        expect_current(pending);
        values[field::propagated_rate_x] =
            mean.angular_rate_b(0) + values[field::angular_correction_x] - values[field::gyro_bias_x];
        values[field::propagated_rate_y] =
            mean.angular_rate_b(1) + values[field::angular_correction_y] - values[field::gyro_bias_y];
        values[field::propagated_rate_z] =
            mean.angular_rate_b(2) + values[field::angular_correction_z] - values[field::gyro_bias_z];
    }
    check();
    __ESBMC_assert(imu_proof::same_input(interval.sample, expected_interval.sample) &&
                       interval.time_us == expected_interval.time_us &&
                       interval.start_time_us == expected_interval.start_time_us &&
                       interval.hold_period_us == expected_interval.hold_period_us,
                   "Output prediction preserves integrated input and interval epochs");
    for (std::size_t i = 0; i < 3; ++i)
    {
        __ESBMC_assert(same(interval.held_rate.specific_force_b(i), expected_interval.held_rate.specific_force_b(i)) &&
                           same(interval.held_rate.angular_rate_b(i), expected_interval.held_rate.angular_rate_b(i)),
                       "Output prediction preserves held input rates");
    }
}
