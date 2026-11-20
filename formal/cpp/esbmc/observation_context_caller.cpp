// SPDX-License-Identifier: BSD-3-Clause
#include "observation_frame_support.hpp"
static_assert(FORMAL_ESKF_PROOF_FRAME == 4);
using observation_proof::Observations;
using observation_proof::same_context;
using observation_proof::same_interval;
using Context = Observations::Context;
namespace
{
Context returned_context;
std::uint64_t expected_query, expected_age;
unsigned calls;
} // namespace
template <>
Context formal_eskf::px4::ObservationHistory<imu_proof::Backend>::lookup_context(std::uint64_t time_us,
                                                                                 std::uint64_t maximum_age_us) const
{
    __ESBMC_assert(this == expected_owner && time_us == expected_query && maximum_age_us == expected_age && calls == 0,
                   "Context caller forwards the original receiver, query and age once");
    check_metadata();
    check_storage();
    __ESBMC_assert(m_context.m_length == 128 && m_context.m_count <= 128 && m_context.m_oldest < 128 &&
                       m_context.m_write == (m_context.m_oldest + m_context.m_count) % 128,
                   "Context caller meets the actual lookup storage domain");
    ++calls;
    return returned_context;
}

void verify_context_caller(Observations owner, std::uint64_t query, std::uint64_t age, imu_proof::Interval interval)
{
    static_assert(FORMAL_ESKF_PROOF_PREDICTION == 0 || FORMAL_ESKF_PROOF_PREDICTION == 1);
    auto const count = imu_proof::nondet_index();
    auto const oldest = imu_proof::nondet_index();
    __ESBMC_assume(count <= 128 && oldest < 128);
    owner.m_context.m_length = 128;
    owner.m_context.m_count = count;
    owner.m_context.m_oldest = oldest;
    owner.m_context.m_write = (static_cast<std::size_t>(oldest) + count) % 128;
    auto const old_interval = interval;
    expected_owner = &owner;
    capture_metadata();
    capture_storage();
    expected_age = age;
    calls = 0;
    __ESBMC_init_object(&returned_context);
#if FORMAL_ESKF_PROOF_PREDICTION
    expected_query = interval.time_us;
    bool const sequence = expected_seed != 0 && expected_advanced != expected_latest &&
                          interval.start_time_us == expected_fusion && interval.time_us == interval.sample.time_us &&
                          interval.time_us > expected_fusion && interval.time_us <= expected_latest &&
                          interval.sample.angular_period_us >= 3800 && interval.sample.angular_period_us <= 4200 &&
                          interval.sample.acceleration_period_us >= 3800 &&
                          interval.sample.acceleration_period_us <= 4200;
    bool const eligible = sequence && expected_latest - interval.time_us >= expected_configuration.minimum_lag_us &&
                          expected_latest - interval.time_us <= 300000;
    auto const result = owner.prediction_context(interval, age);
#else
    expected_query = query;
    bool const eligible = expected_seed != 0 && expected_seed <= query && query <= expected_fusion;
    auto const result = owner.context(query, age);
#endif
    __ESBMC_assert(calls == static_cast<unsigned>(eligible), "Context caller exact eligibility");
    if (eligible)
    {
        __ESBMC_assert(same_context(result, returned_context), "Context caller preserves the complete producer value");
    }
    else
    {
        __ESBMC_assert(same_context(result, Context{}), "Rejected context query returns completely empty context");
    }
    check_metadata();
    check_storage();
    __ESBMC_assert(same_interval(interval, old_interval), "Context caller preserves complete IMU interval");
}
