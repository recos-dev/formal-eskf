// SPDX-License-Identifier: BSD-3-Clause

#include "imu_owner_support.hpp"

using imu_proof::Backend;
using imu_proof::Imu;
using imu_proof::Input;
using imu_proof::Interval;
using imu_proof::same;
using imu_proof::same_configuration;
using imu_proof::same_input;
using imu_proof::same_source;
using imu_proof::Scalar;

namespace
{
Imu * expected_owner;
Input const * expected_input;
std::uint64_t expected_now;
bool admission_result;
unsigned admission_calls;
unsigned rate_calls;
unsigned storage_calls;
Imu::imu_type mean_rate_result;
Interval forwarded;
Input original_input;
Input original_previous;
std::size_t original_length;
std::size_t original_count;
std::size_t original_oldest;
std::size_t original_write;
#if FORMAL_ESKF_PROOF_FRAME
constexpr std::size_t observed_slot = FORMAL_ESKF_PROOF_SLOT;
static_assert(observed_slot < 64);
Interval expected_slot;
#endif

#if FORMAL_ESKF_PROOF_FRAME
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

// These are caller contracts, not substitute producer proofs. Acceptance requires
// matching delayed-input, mean-rate and every delayed-storage producer profile.
template <> bool formal_eskf::px4::ImuHistory<Backend>::valid_input(input_type const & sample, std::uint64_t now)
{
    __ESBMC_assert(&sample == expected_input && same_input(sample, original_input) && now == expected_now &&
                       admission_calls == 0 && rate_calls == 0 && storage_calls == 0,
                   "IMU push admission receives the actual input and clock before any mutation producer");
    ++admission_calls;
    return admission_result;
}

template <> formal_eskf::ImuSample<Backend> formal_eskf::px4::ImuInput<Backend>::mean_rates() const
{
    __ESBMC_assert(this == &expected_owner->m_previous && same_input(*this, original_previous) &&
                       angular_period_us >= 3800 && angular_period_us <= 4200 && acceleration_period_us >= 3800 &&
                       acceleration_period_us <= 4200 && rate_calls == 0 && storage_calls == 0,
                   "IMU held rate comes from the previous admitted input before storage");
    ++rate_calls;
    return mean_rate_result;
}

template <> void formal_eskf::runtime::RingBuffer<Interval, 64>::push(Interval const & sample)
{
    __ESBMC_assert(this == &expected_owner->m_samples && m_length >= 2 && m_length <= 64 && m_count <= m_length &&
                       m_oldest < m_length && m_write < m_length && storage_calls == 0,
                   "IMU push meets typed storage receiver and metadata domain");
    for (std::size_t i = 0; i < 64; ++i)
    {
        __ESBMC_assert(&sample != &m_samples[i], "IMU push supplies an external interval to storage");
    }
    __ESBMC_assert(m_length == original_length && m_count == original_count && m_oldest == original_oldest &&
                       m_write == original_write,
                   "IMU push preserves metadata until the storage producer");

#if FORMAL_ESKF_PROOF_FRAME
    __ESBMC_assert(same_interval(m_samples[observed_slot], expected_slot), "IMU pre-storage payload frame");
#endif
    forwarded = sample;
    ++storage_calls;
    // Overapproximate every payload the actual typed producer could publish.
    // Every physical slot is arbitrary, including inactive storage. Frame profiles
    // cover all observation positions; none restricts the producer poststate.
    // Capturing this fresh poststate checks that the remaining caller preserves
    // it; leaving the ring unchanged would not establish that frame.
    auto const previous = expected_owner->m_previous;
    auto const configuration = expected_owner->m_configuration;
    auto const fusion = expected_owner->m_fusion_time_us;
    auto const fault = expected_owner->m_fault;
    // The intrinsic requires a complete variable. Restore every non-buffer
    // field and metadata, leaving exactly the backing payload unconstrained.
    __ESBMC_init_object(expected_owner);
    expected_owner->m_previous = previous;
    expected_owner->m_configuration = configuration;
    expected_owner->m_fusion_time_us = fusion;
    expected_owner->m_fault = fault;
    m_length = original_length;
    m_count = original_count;
    m_oldest = original_oldest;
    m_write = original_write;

#if FORMAL_ESKF_PROOF_FRAME
    expected_slot = m_samples[observed_slot];
#endif
    m_write = m_write + 1 == m_length ? 0 : m_write + 1;
    if (m_count == m_length)
    {
        m_oldest = m_oldest + 1 == m_length ? 0 : m_oldest + 1;
    }
    else
    {
        ++m_count;
    }
}

#if FORMAL_ESKF_PROOF_FRAME
void verify_imu_push_frame(Imu owner, Input input, std::uint64_t now, bool admitted, Imu::imu_type rates)
{
    imu_proof::metadata(owner.m_samples);
    // Reachable active owners are configured and retain an admitted previous input.
    // Constructor/configure/seed/reset and the timeline induction discharge this domain.
    __ESBMC_assume(owner.m_previous.time_us == 0 ||
                   (owner.m_samples.m_length >= 2 && owner.m_previous.angular_period_us >= 3800 &&
                    owner.m_previous.angular_period_us <= 4200 && owner.m_previous.acceleration_period_us >= 3800 &&
                    owner.m_previous.acceleration_period_us <= 4200));
    expected_slot = owner.m_samples.m_samples[observed_slot];
    auto const previous = owner.m_previous;
    auto const count = owner.m_samples.m_count;
    auto const length = owner.m_samples.m_length;
    auto const oldest = owner.m_samples.m_oldest;
    auto const write = owner.m_samples.m_write;
    original_input = input;
    original_previous = previous;
    original_length = length;
    original_count = count;
    original_oldest = oldest;
    original_write = write;
    expected_owner = &owner;
    expected_input = &input;
    expected_now = now;
    admission_result = admitted;
    admission_calls = 0;
    rate_calls = 0;
    storage_calls = 0;
    mean_rate_result = rates;
    (void)owner.push(input, now);
    __ESBMC_assert(same_interval(owner.m_samples.m_samples[observed_slot], expected_slot),
                   "IMU caller preserves the storage producer poststate");
}
#else
void verify_imu_push_contract(Imu owner, Input input, std::uint64_t now, bool admitted, Imu::imu_type rates)
{
    imu_proof::metadata(owner.m_samples);
    // Reachable active owners are configured and retain an admitted previous input.
    // Constructor/configure/seed/reset and the timeline induction discharge this domain.
    __ESBMC_assume(owner.m_previous.time_us == 0 ||
                   (owner.m_samples.m_length >= 2 && owner.m_previous.angular_period_us >= 3800 &&
                    owner.m_previous.angular_period_us <= 4200 && owner.m_previous.acceleration_period_us >= 3800 &&
                    owner.m_previous.acceleration_period_us <= 4200));

    auto const previous = owner.m_previous;
    auto const before_input = input;
    auto const config = owner.m_configuration;
    auto const fusion = owner.m_fusion_time_us;
    auto const fault = owner.m_fault;
    auto const count = owner.m_samples.m_count;
    auto const length = owner.m_samples.m_length;
    auto const oldest = owner.m_samples.m_oldest;
    auto const write = owner.m_samples.m_write;
    bool eligible = previous.time_us != 0 && fault == Imu::Fault::none;
    auto expected_fault = fault;
    if (eligible && (!admitted || input.time_us <= previous.time_us))
    {
        expected_fault = Imu::Fault::invalid_sample;
        eligible = false;
    }
    if (eligible)
    {
        auto const elapsed = input.time_us - previous.time_us;
        if (elapsed < 3800 || elapsed > 50000 || !same_source(input.source, previous.source))
        {
            expected_fault = Imu::Fault::discontinuity;
            eligible = false;
        }
        else if (static_cast<std::int64_t>(elapsed) - static_cast<std::int64_t>(input.angular_period_us) < -200)
        {
            expected_fault = Imu::Fault::discontinuity;
            eligible = false;
        }
    }
    if (eligible && count == length && owner.m_samples.m_samples[oldest].time_us > fusion)
    {
        expected_fault = Imu::Fault::history_loss;
        eligible = false;
    }
    if (eligible && count + 1 >= length)
    {
        auto retained = oldest;
        if (count == length)
        {
            retained = (oldest + 1) % length;
        }
        auto const lag = input.time_us - owner.m_samples.m_samples[retained].time_us;
        if (lag < config.minimum_lag_us || lag > 300000)
        {
            expected_fault = Imu::Fault::lag_out_of_range;
            eligible = false;
        }
    }
    original_input = input;
    original_previous = previous;
    original_length = length;
    original_count = count;
    original_oldest = oldest;
    original_write = write;
    expected_owner = &owner;
    expected_input = &input;
    expected_now = now;
    admission_result = admitted;
    admission_calls = 0;
    rate_calls = 0;
    storage_calls = 0;
    mean_rate_result = rates;
    bool const result = owner.push(input, now);
    __ESBMC_assert(result == eligible && owner.m_fault == expected_fault, "IMU push exact status and first fault");
    __ESBMC_assert(admission_calls == (previous.time_us != 0 && fault == Imu::Fault::none ? 1U : 0U) &&
                       storage_calls == (result ? 1U : 0U),
                   "IMU push exact admission and storage call counts");
    __ESBMC_assert(same_input(input, before_input) && same_configuration(owner.m_configuration, config) &&
                       owner.m_fusion_time_us == fusion,
                   "IMU push caller input, configuration and cursor frame");
    if (result)
    {
        auto const gap = static_cast<std::int64_t>(input.time_us - previous.time_us) -
                         static_cast<std::int64_t>(input.angular_period_us);
        bool const held = gap > 200;
        __ESBMC_assert(same_input(owner.m_previous, input) && forwarded.time_us == input.time_us &&
                           forwarded.start_time_us == previous.time_us && same_input(forwarded.sample, input) &&
                           forwarded.hold_period_us == (held ? static_cast<std::uint32_t>(gap) : 0U) &&
                           rate_calls == (held ? 1U : 0U),
                       "IMU push complete interval forwarding and previous-input publication");
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            if (held)
            {
                __ESBMC_assert(same(forwarded.held_rate.specific_force_b(axis), rates.specific_force_b(axis)) &&
                                   same(forwarded.held_rate.angular_rate_b(axis), rates.angular_rate_b(axis)),
                               "IMU push preserves held-rate producer coefficients");
            }
            else
            {
                __ESBMC_assert(same(forwarded.held_rate.specific_force_b(axis), Scalar{0}) &&
                                   same(forwarded.held_rate.angular_rate_b(axis), Scalar{0}),
                               "IMU push initializes unused held-rate coefficients");
            }
        }
    }
    else
    {
        __ESBMC_assert(same_input(owner.m_previous, previous) && rate_calls == 0,
                       "IMU rejected push preserves previous input without rate evaluation");
    }
    __ESBMC_assert(owner.m_samples.m_length == length &&
                       owner.m_samples.m_count == (result && count < length ? count + 1 : count) &&
                       owner.m_samples.m_oldest ==
                           (result && count == length ? (oldest + 1 == length ? 0 : oldest + 1) : oldest) &&
                       owner.m_samples.m_write == (result ? (write + 1 == length ? 0 : write + 1) : write),
                   "IMU push preserves exact storage producer metadata");
}
#endif
