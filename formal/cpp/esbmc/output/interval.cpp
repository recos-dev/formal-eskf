// SPDX-License-Identifier: BSD-3-Clause

#include <interval_step.hpp>
using namespace output_proof;
void verify_output_interval_range(Scalar previous, std::uint16_t period)
{
    auto const initial = static_cast<Scalar>(profile::imu::nominal_interval_s);
    __ESBMC_assert(initial > Scalar{0} && initial <= static_cast<Scalar>(0.005),
                   "Output interval initialization establishes the finite positive envelope");
    __ESBMC_assume(previous >= Scalar{0} && previous <= static_cast<Scalar>(0.005));
    __ESBMC_assume(period >= 3800 && period <= 4200);
    Sample sample;
    sample.angular_period_us = period;
    auto const mean = output_steps::interval_average(previous, &sample);
    __ESBMC_assert(mean > Scalar{0} && mean <= static_cast<Scalar>(0.005),
                   "Output interval average preserves a positive finite 5 ms envelope");
    auto const velocity_gain = mean / static_cast<Scalar>(profile::output::velocity_time_constant_s);
    auto const position_gain = mean / static_cast<Scalar>(profile::output::position_time_constant_s);
    __ESBMC_assert(std::isfinite(velocity_gain) && velocity_gain > Scalar{0} && std::isfinite(position_gain) &&
                       position_gain > Scalar{0},
                   "Output feedback gains are positive and finite in the configured profile");
}
