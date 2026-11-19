// SPDX-License-Identifier: BSD-3-Clause

#include "imu_owner_support.hpp"

using imu_proof::Imu;
using imu_proof::Input;
using imu_proof::Interval;
using imu_proof::same;
using imu_proof::same_configuration;
using imu_proof::same_input;
using TimingParameters = formal_eskf::px4::TimingParameters;

namespace
{
Imu * expected_owner;
unsigned reset_calls;
std::size_t original_count;
std::size_t original_oldest;
std::size_t original_write;
#if FORMAL_ESKF_PROOF_FRAME
constexpr std::size_t observed_slot = FORMAL_ESKF_PROOF_SLOT;
static_assert(observed_slot < 64);
Interval expected_slot;
bool same_interval(Interval const & a, Interval const & b)
{
    bool equal = a.time_us == b.time_us && a.start_time_us == b.start_time_us && a.hold_period_us == b.hold_period_us &&
                 same_input(a.sample, b.sample);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        equal = same(a.held_rate.specific_force_b(axis), b.held_rate.specific_force_b(axis)) &
                same(a.held_rate.angular_rate_b(axis), b.held_rate.angular_rate_b(axis)) & equal;
    }
    return equal;
}

#endif
} // namespace

// Actual allocate() and configure() execute. Only the independently proved typed
// reset is summarized; all payloads are fresh, not assumed zero or unchanged.
// Matching reset evidence supplies zero values. Every frame profile checks that
// configure preserves that arbitrary producer poststate, including inactive slots.
template <> void formal_eskf::runtime::RingBuffer<Interval, 64>::reset()
{
    __ESBMC_assert(this == &expected_owner->m_samples && reset_calls == 0 && m_length >= 2 && m_length <= 64,
                   "IMU configuration invokes reset on its allocated ring");
    __ESBMC_assert(m_count == original_count && m_oldest == original_oldest && m_write == original_write,
                   "IMU configuration preserves indices until reset");
#if FORMAL_ESKF_PROOF_FRAME
    __ESBMC_assert(same_interval(m_samples[observed_slot], expected_slot), "IMU configuration pre-reset payload frame");
#endif
    ++reset_calls;
    auto const previous = expected_owner->m_previous;
    auto const configuration = expected_owner->m_configuration;
    auto const fusion = expected_owner->m_fusion_time_us;
    auto const fault = expected_owner->m_fault;
    auto const length = m_length;
    __ESBMC_init_object(expected_owner);
    expected_owner->m_previous = previous;
    expected_owner->m_configuration = configuration;
    expected_owner->m_fusion_time_us = fusion;
    expected_owner->m_fault = fault;
    m_length = length;
    m_count = 0;
    m_oldest = 0;
    m_write = 0;
#if FORMAL_ESKF_PROOF_FRAME
    expected_slot = m_samples[observed_slot];
#endif
}

#if FORMAL_ESKF_PROOF_FRAME
void verify_imu_configuration_frame(Imu owner, TimingParameters parameters)
#else
void verify_imu_configuration_contract(Imu owner, TimingParameters parameters)
#endif
{
    imu_proof::metadata(owner.m_samples);
    original_count = owner.m_samples.m_count;
    original_oldest = owner.m_samples.m_oldest;
    original_write = owner.m_samples.m_write;
    expected_owner = &owner;
    reset_calls = 0;
#if FORMAL_ESKF_PROOF_FRAME
    expected_slot = owner.m_samples.m_samples[observed_slot];
    (void)owner.configure(parameters);
    __ESBMC_assert(same_interval(owner.m_samples.m_samples[observed_slot], expected_slot),
                   "IMU configuration preserves the reset producer poststate");
#else
    auto const previous = owner.m_previous;
    auto const config = owner.m_configuration;
    auto const fusion = owner.m_fusion_time_us;
    auto const fault = owner.m_fault;
    auto const length = owner.m_samples.m_length;
    auto const before = parameters;
    imu_proof::Configuration candidate;
    bool const eligible =
        previous.time_us == 0 && fault == Imu::Fault::none && formal_eskf::px4::configure_timing(parameters, candidate);
    bool const changed = eligible && candidate.imu_length != length;
    bool const result = owner.configure(parameters);
    __ESBMC_assert(result == eligible && reset_calls == (changed ? 1U : 0U),
                   "IMU configuration exact eligibility and reset call count");
    __ESBMC_assert(same_input(owner.m_previous, previous) && owner.m_fusion_time_us == fusion && owner.m_fault == fault,
                   "IMU configuration preserves previous input, cursor and fault");
    if (result)
    {
        __ESBMC_assert(same_configuration(owner.m_configuration, candidate), "IMU configuration complete publication");
    }
    else
    {
        __ESBMC_assert(same_configuration(owner.m_configuration, config), "IMU configuration failure frame");
    }
    __ESBMC_assert(owner.m_samples.m_length == (changed ? candidate.imu_length : length) &&
                       owner.m_samples.m_count == (changed ? 0 : original_count) &&
                       owner.m_samples.m_oldest == (changed ? 0 : original_oldest) &&
                       owner.m_samples.m_write == (changed ? 0 : original_write),
                   "IMU configuration exact ring metadata");
    __ESBMC_assert(parameters.filter_interval_us == before.filter_interval_us &&
                       parameters.fusion_delay_ms == before.fusion_delay_ms &&
                       parameters.gnss_delay_ms == before.gnss_delay_ms &&
                       parameters.magnetic_delay_ms == before.magnetic_delay_ms &&
                       parameters.barometer_delay_ms == before.barometer_delay_ms,
                   "IMU configuration caller input frame");
#endif
}
