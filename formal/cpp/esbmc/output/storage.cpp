// SPDX-License-Identifier: BSD-3-Clause

#include "support.hpp"
using namespace output_proof;
using Ring = formal_eskf::runtime::RingBuffer<Sample, 64>;
void verify_output_typed_storage(Ring ring, Sample sample)
{
    auto const length = imu_proof::nondet_index();
    auto const count = imu_proof::nondet_index();
    auto const oldest = imu_proof::nondet_index();
    __ESBMC_assume(length > 0 && length <= 64 && count <= length && oldest < length);
    ring.m_length = length;
    ring.m_count = count;
    ring.m_oldest = oldest;
    auto const sum = static_cast<std::size_t>(oldest) + count;
    auto const expected_write = sum < length ? sum : sum - length;
    constexpr std::size_t write = FORMAL_ESKF_PROOF_WRITE;
    static_assert(write < 64);
    __ESBMC_assume(expected_write == write);
    ring.m_write = write;
    std::uint64_t stored_integers[64][3], input_integers[3];
    Scalar stored_values[64][10], input_values[10];
    input_integers[0] = sample.time_us;
    input_integers[1] = sample.start_time_us;
    input_integers[2] = sample.angular_period_us;
    input_values[0] = sample.q_nb.q0();
    input_values[1] = sample.q_nb.q1();
    input_values[2] = sample.q_nb.q2();
    input_values[3] = sample.q_nb.q3();
    input_values[4] = sample.v_n.m_storage.values.head;
    input_values[5] = sample.v_n.m_storage.values.tail.head;
    input_values[6] = sample.v_n.m_storage.values.tail.tail.head;
    input_values[7] = sample.p_n.m_storage.values.head;
    input_values[8] = sample.p_n.m_storage.values.tail.head;
    input_values[9] = sample.p_n.m_storage.values.tail.tail.head;
    for (std::size_t i = 0; i < 64; ++i)
    {
        stored_integers[i][0] = ring.m_samples[i].time_us;
        stored_integers[i][1] = ring.m_samples[i].start_time_us;
        stored_integers[i][2] = ring.m_samples[i].angular_period_us;
        stored_values[i][0] = ring.m_samples[i].q_nb.q0();
        stored_values[i][1] = ring.m_samples[i].q_nb.q1();
        stored_values[i][2] = ring.m_samples[i].q_nb.q2();
        stored_values[i][3] = ring.m_samples[i].q_nb.q3();
        stored_values[i][4] = ring.m_samples[i].v_n.m_storage.values.head;
        stored_values[i][5] = ring.m_samples[i].v_n.m_storage.values.tail.head;
        stored_values[i][6] = ring.m_samples[i].v_n.m_storage.values.tail.tail.head;
        stored_values[i][7] = ring.m_samples[i].p_n.m_storage.values.head;
        stored_values[i][8] = ring.m_samples[i].p_n.m_storage.values.tail.head;
        stored_values[i][9] = ring.m_samples[i].p_n.m_storage.values.tail.tail.head;
    }
    ring.push(sample);
    __ESBMC_assert(ring.m_length == length && ring.m_count == (count == length ? count : count + 1) &&
                       ring.m_oldest == (count == length ? (oldest + 1) % length : oldest) &&
                       ring.m_write == (write + 1) % length,
                   "Typed output storage preserves length and updates count/cursors exactly");
    for (std::size_t i = 0; i < 64; ++i)
    {
        __ESBMC_assert(ring.m_samples[i].time_us == (i == write ? input_integers[0] : stored_integers[i][0]),
                       "Output typed store complete slot field: time_us");
        __ESBMC_assert(ring.m_samples[i].start_time_us == (i == write ? input_integers[1] : stored_integers[i][1]),
                       "Output typed store complete slot field: start_time_us");
        __ESBMC_assert(ring.m_samples[i].angular_period_us == (i == write ? input_integers[2] : stored_integers[i][2]),
                       "Output typed store complete slot field: angular_period_us");
        __ESBMC_assert(same(ring.m_samples[i].q_nb.q0(), (i == write ? input_values[0] : stored_values[i][0])),
                       "Output typed store complete slot field: q_nb.q0()");
        __ESBMC_assert(same(ring.m_samples[i].q_nb.q1(), (i == write ? input_values[1] : stored_values[i][1])),
                       "Output typed store complete slot field: q_nb.q1()");
        __ESBMC_assert(same(ring.m_samples[i].q_nb.q2(), (i == write ? input_values[2] : stored_values[i][2])),
                       "Output typed store complete slot field: q_nb.q2()");
        __ESBMC_assert(same(ring.m_samples[i].q_nb.q3(), (i == write ? input_values[3] : stored_values[i][3])),
                       "Output typed store complete slot field: q_nb.q3()");
        __ESBMC_assert(
            same(ring.m_samples[i].v_n.m_storage.values.head, (i == write ? input_values[4] : stored_values[i][4])),
            "Output typed store complete slot field: v_n.m_storage.values.head");
        __ESBMC_assert(same(ring.m_samples[i].v_n.m_storage.values.tail.head,
                            (i == write ? input_values[5] : stored_values[i][5])),
                       "Output typed store complete slot field: v_n.m_storage.values.tail.head");
        __ESBMC_assert(same(ring.m_samples[i].v_n.m_storage.values.tail.tail.head,
                            (i == write ? input_values[6] : stored_values[i][6])),
                       "Output typed store complete slot field: v_n.m_storage.values.tail.tail.head");
        __ESBMC_assert(
            same(ring.m_samples[i].p_n.m_storage.values.head, (i == write ? input_values[7] : stored_values[i][7])),
            "Output typed store complete slot field: p_n.m_storage.values.head");
        __ESBMC_assert(same(ring.m_samples[i].p_n.m_storage.values.tail.head,
                            (i == write ? input_values[8] : stored_values[i][8])),
                       "Output typed store complete slot field: p_n.m_storage.values.tail.head");
        __ESBMC_assert(same(ring.m_samples[i].p_n.m_storage.values.tail.tail.head,
                            (i == write ? input_values[9] : stored_values[i][9])),
                       "Output typed store complete slot field: p_n.m_storage.values.tail.tail.head");
    }
    __ESBMC_assert(sample.time_us == input_integers[0], "Output typed store preserves complete input: time_us");
    __ESBMC_assert(sample.start_time_us == input_integers[1],
                   "Output typed store preserves complete input: start_time_us");
    __ESBMC_assert(sample.angular_period_us == input_integers[2],
                   "Output typed store preserves complete input: angular_period_us");
    __ESBMC_assert(same(sample.q_nb.q0(), input_values[0]), "Output typed store preserves complete input: q_nb.q0()");
    __ESBMC_assert(same(sample.q_nb.q1(), input_values[1]), "Output typed store preserves complete input: q_nb.q1()");
    __ESBMC_assert(same(sample.q_nb.q2(), input_values[2]), "Output typed store preserves complete input: q_nb.q2()");
    __ESBMC_assert(same(sample.q_nb.q3(), input_values[3]), "Output typed store preserves complete input: q_nb.q3()");
    __ESBMC_assert(same(sample.v_n.m_storage.values.head, input_values[4]),
                   "Output typed store preserves complete input: v_n.m_storage.values.head");
    __ESBMC_assert(same(sample.v_n.m_storage.values.tail.head, input_values[5]),
                   "Output typed store preserves complete input: v_n.m_storage.values.tail.head");
    __ESBMC_assert(same(sample.v_n.m_storage.values.tail.tail.head, input_values[6]),
                   "Output typed store preserves complete input: v_n.m_storage.values.tail.tail.head");
    __ESBMC_assert(same(sample.p_n.m_storage.values.head, input_values[7]),
                   "Output typed store preserves complete input: p_n.m_storage.values.head");
    __ESBMC_assert(same(sample.p_n.m_storage.values.tail.head, input_values[8]),
                   "Output typed store preserves complete input: p_n.m_storage.values.tail.head");
    __ESBMC_assert(same(sample.p_n.m_storage.values.tail.tail.head, input_values[9]),
                   "Output typed store preserves complete input: p_n.m_storage.values.tail.tail.head");
}
void verify_output_typed_reset(Ring ring)
{
    auto const length = ring.m_length;
    ring.reset();
    __ESBMC_assert(ring.m_length == length && ring.m_count == 0 && ring.m_oldest == 0 && ring.m_write == 0,
                   "Output typed reset preserves length and clears metadata");
    for (std::size_t i = 0; i < 64; ++i)
    {
        zero_sample(ring.m_samples[i]);
    }
}
