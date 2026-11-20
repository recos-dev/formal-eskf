// SPDX-License-Identifier: BSD-3-Clause

#include "selected_frame.hpp"
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
template <> ImuSample<output_proof::Backend> ImuInput<output_proof::Backend>::mean_rates() const { return mean; }
template <>
bool OutputPredictor<output_proof::Backend>::advance(Sample & candidate, ImuSample<output_proof::Backend> imu,
                                                     value_type dt) const
{
    bool const held = holding && advances == 0;
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
    check_selected();
    auto const length = integers[field::length];
    bool const full = integers[field::count] == length;
    integers[field::count] = full ? integers[field::count] : integers[field::count] + 1;
    integers[field::oldest] =
        full ? (integers[field::oldest] + 1 == length ? 0 : integers[field::oldest] + 1) : integers[field::oldest];
    integers[field::write] = integers[field::write] + 1 == length ? 0 : integers[field::write] + 1;
    __ESBMC_init_object(owner);
    restore_outer();
    restore_selected_workspace();
    capture_selected_history();
    ++stores;
}
void verify_output_prediction_frame(Output object, Interval interval, Sample hold, Sample real, Rates rates,
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
    capture_selected();
    (void)object.predict(interval);
    check_selected();
}
