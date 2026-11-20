// SPDX-License-Identifier: BSD-3-Clause

#define private public
#include "../contracts.hpp"
#undef private
// Keep the actual timing producer available under a distinct name. The output
// caller binds to the summary below; the actual producer has separate evidence.
#define configure_timing output_verified_timing
#include "../imu_owner_support.hpp"
#undef configure_timing
namespace formal_eskf::px4
{
bool configure_timing(TimingParameters const &, TimingConfiguration &);
}
#include "boundary.hpp"
namespace
{
using namespace output_boundary;
unsigned resets, timing_calls;
std::size_t requested_length;
formal_eskf::px4::TimingParameters expected_parameters;
formal_eskf::px4::TimingConfiguration timing_result;
bool timing_success;
} // namespace
namespace formal_eskf::px4
{
bool configure_timing(TimingParameters const & parameters, TimingConfiguration & result)
{
    check();
    __ESBMC_assert(timing_calls == 0 && resets == 0,
                   "Output configuration checks timing before allocation exactly once");
    __ESBMC_assert(parameters.filter_interval_us == expected_parameters.filter_interval_us &&
                       parameters.fusion_delay_ms == expected_parameters.fusion_delay_ms &&
                       parameters.gnss_delay_ms == expected_parameters.gnss_delay_ms &&
                       parameters.magnetic_delay_ms == expected_parameters.magnetic_delay_ms &&
                       parameters.barometer_delay_ms == expected_parameters.barometer_delay_ms,
                   "Output configuration forwards every signed timing parameter");
    ++timing_calls;
    if (timing_success)
    {
        result = timing_result;
    }
    return timing_success;
}
} // namespace formal_eskf::px4
template <> void formal_eskf::runtime::RingBuffer<output_proof::Sample, 64>::reset()
{
    using namespace output_boundary;
    __ESBMC_assert(this == &owner->m_history && resets == 0 && m_length == requested_length && requested_length >= 2 &&
                       requested_length <= 64,
                   "Output configure resets the actual allocated ring exactly once");
    integers[field::length] = requested_length;
    check();
    __ESBMC_init_object(owner);
    integers[field::count] = 0;
    integers[field::oldest] = 0;
    integers[field::write] = 0;
    restore_outer();
    capture_storage();
    ++resets;
}
void verify_output_configuration(Output input_owner, formal_eskf::px4::TimingParameters parameters,
                                 formal_eskf::px4::TimingConfiguration output, bool success)
{
    owner = &input_owner;
    __ESBMC_assume(!success || (output.imu_length >= 2 && output.imu_length <= 64));
    expected_parameters = parameters;
    timing_result = output;
    timing_success = success;
    requested_length = output.imu_length;
    resets = timing_calls = 0;
    capture();
    bool const can_call = integers[field::time] == 0 && fault == Output::Fault::none;
    bool const expected = can_call && success;
    bool const changed = expected && output.imu_length != integers[field::length];
    bool const accepted = input_owner.configure(parameters);
    __ESBMC_assert(accepted == expected && timing_calls == static_cast<unsigned>(can_call) &&
                       resets == static_cast<unsigned>(changed),
                   "Output configuration preserves eligibility and exact timing/reset dispatch");
    if (expected)
    {
        integers[field::minimum_lag] = output.minimum_lag_us;
        integers[field::gnss_delay] = output.gnss_delay_us;
        integers[field::magnetic_delay] = output.magnetic_delay_us;
        integers[field::barometer_delay] = output.barometer_delay_us;
        integers[field::configured_length] = output.imu_length;
    }
    check();
    __ESBMC_assert(parameters.filter_interval_us == expected_parameters.filter_interval_us &&
                       parameters.fusion_delay_ms == expected_parameters.fusion_delay_ms &&
                       parameters.gnss_delay_ms == expected_parameters.gnss_delay_ms &&
                       parameters.magnetic_delay_ms == expected_parameters.magnetic_delay_ms &&
                       parameters.barometer_delay_ms == expected_parameters.barometer_delay_ms,
                   "Output configuration preserves every signed timing parameter");
}
