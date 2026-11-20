// SPDX-License-Identifier: BSD-3-Clause
#include "observation_frame_support.hpp"
#include "context_step.hpp"
static_assert(FORMAL_ESKF_PROOF_FRAME == 4);

void verify_context_step(observation_proof::Observations owner, std::uint64_t query, std::uint64_t age,
                         observation_proof::Observations::Context result)
{
    constexpr std::size_t chosen = FORMAL_ESKF_PROOF_SLOT;
    static_assert(chosen < 128);
    std::size_t cursor = chosen == 127 ? 0 : chosen + 1;
    auto const old_output = result;
    auto const count = imu_proof::nondet_index();
    auto const oldest = imu_proof::nondet_index();
    __ESBMC_assume(count <= 128 && oldest < 128);
    owner.m_context.m_length = 128;
    owner.m_context.m_count = count;
    owner.m_context.m_oldest = oldest;
    owner.m_context.m_write = (static_cast<std::size_t>(oldest) + count) % 128;
    auto const old_query = query;
    auto const old_age = age;
    expected_owner = &owner;
    capture_metadata();
    capture_storage();
    bool const stop = context_integers[chosen][0] <= query;
    bool const fresh = stop && query - context_integers[chosen][0] < 50000;
    auto const expected_integers_0 = stop ? (fresh ? context_integers[chosen][0] : 0) : old_output.time_us;
    auto const expected_integers_1 = stop ? (fresh ? context_integers[chosen][1] : 0) : old_output.land.time_us;
    auto const expected_integers_2 = stop ? (fresh ? context_integers[chosen][2] : 0) : old_output.vehicle.time_us;
    bool const expected_flags_0 =
        stop ? fresh && context_flags[chosen][0] && query - context_integers[chosen][1] < age : old_output.land.known;
    bool const expected_flags_1 = stop ? fresh && context_flags[chosen][1] : old_output.land.landed;
    bool const expected_flags_2 = stop ? fresh && context_flags[chosen][2] : old_output.land.at_rest;
    bool const expected_flags_3 = stop ? fresh && context_flags[chosen][3] : old_output.land.in_ground_effect;
    bool const expected_flags_4 = stop ? fresh && context_flags[chosen][4] && query - context_integers[chosen][2] < age
                                       : old_output.vehicle.known;
    bool const expected_flags_5 = stop ? fresh && context_flags[chosen][5] : old_output.vehicle.armed;
    bool const expected_flags_6 = stop ? fresh && context_flags[chosen][6] : old_output.vehicle.fixed_wing;
    auto const stopped = context_step(owner, cursor, query, age, result);
    __ESBMC_assert(stopped == stop && cursor == chosen, "Context step exact match and reverse cursor");
    __ESBMC_assert(result.time_us == expected_integers_0, "Context exact lookup/publication time_us");
    __ESBMC_assert(result.land.time_us == expected_integers_1, "Context exact lookup/publication land.time_us");
    __ESBMC_assert(result.vehicle.time_us == expected_integers_2, "Context exact lookup/publication vehicle.time_us");
    __ESBMC_assert(result.land.known == expected_flags_0, "Context exact lookup/publication land.known");
    __ESBMC_assert(result.land.landed == expected_flags_1, "Context exact lookup/publication land.landed");
    __ESBMC_assert(result.land.at_rest == expected_flags_2, "Context exact lookup/publication land.at_rest");
    __ESBMC_assert(result.land.in_ground_effect == expected_flags_3,
                   "Context exact lookup/publication land.in_ground_effect");
    __ESBMC_assert(result.vehicle.known == expected_flags_4, "Context exact lookup/publication vehicle.known");
    __ESBMC_assert(result.vehicle.armed == expected_flags_5, "Context exact lookup/publication vehicle.armed");
    __ESBMC_assert(result.vehicle.fixed_wing == expected_flags_6,
                   "Context exact lookup/publication vehicle.fixed_wing");
    __ESBMC_assert(query == old_query && age == old_age, "Context complete external input frame");
    check_metadata();
    check_storage();
}

void verify_context_empty(observation_proof::Observations owner, std::uint64_t query, std::uint64_t age)
{
    auto const oldest = imu_proof::nondet_index();
    __ESBMC_assume(oldest < 128);
    owner.m_context.m_length = 128;
    owner.m_context.m_count = 0;
    owner.m_context.m_oldest = oldest;
    owner.m_context.m_write = oldest;
    auto const old_query = query;
    auto const old_age = age;
    expected_owner = &owner;
    capture_metadata();
    capture_storage();
    auto const result = owner.lookup_context(query, age);
    __ESBMC_assert(observation_proof::same_context(result, observation_proof::Observations::Context{}),
                   "Empty lookup returns complete unknown context");
    __ESBMC_assert(query == old_query && age == old_age, "Empty lookup preserves query and age");
    check_metadata();
    check_storage();
}

using observation_proof::Observations;
using Context = Observations::Context;
using Ring = formal_eskf::runtime::RingBuffer<Context, 128>;
void verify_context_views(Ring ring)
{
    auto const count = imu_proof::nondet_index();
    auto const oldest = imu_proof::nondet_index();
    __ESBMC_assume(count > 0 && count <= 128 && oldest < 128);
    ring.m_length = 128;
    ring.m_count = count;
    ring.m_oldest = oldest;
    ring.m_write = (static_cast<std::size_t>(oldest) + count) % 128;
    auto const write = ring.m_write;
    std::uint64_t old_integers[128][3];
    for (std::size_t i = 0; i < 128; ++i)
    {
        old_integers[i][0] = ring.m_samples[i].time_us;
        old_integers[i][1] = ring.m_samples[i].land.time_us;
        old_integers[i][2] = ring.m_samples[i].vehicle.time_us;
    }
    bool old_flags[128][7];
    for (std::size_t i = 0; i < 128; ++i)
    {
        old_flags[i][0] = ring.m_samples[i].land.known;
        old_flags[i][1] = ring.m_samples[i].land.landed;
        old_flags[i][2] = ring.m_samples[i].land.at_rest;
        old_flags[i][3] = ring.m_samples[i].land.in_ground_effect;
        old_flags[i][4] = ring.m_samples[i].vehicle.known;
        old_flags[i][5] = ring.m_samples[i].vehicle.armed;
        old_flags[i][6] = ring.m_samples[i].vehicle.fixed_wing;
    }
    __ESBMC_assert(ring.get_length() == 128 && ring.entries() == count && ring.get_oldest_index() == oldest,
                   "Context metadata views return the actual ring fields");
    __ESBMC_assert((ring.get_oldest_index() + ring.entries()) % ring.get_length() == write,
                   "Context lookup initial cursor equals the coherent write cursor");
    auto const & oldest_view = ring.get_oldest();
    __ESBMC_assert(&oldest_view == &ring.m_samples[oldest], "Oldest context retains its actual owner alias");
    auto const & actual = ring.get_newest();
    auto const index = (static_cast<std::size_t>(oldest) + count - 1) % 128;
    __ESBMC_assert(&actual == &ring.m_samples[index], "Newest context is the last logical active slot");
    __ESBMC_assert(ring.m_length == 128 && ring.m_count == count && ring.m_oldest == oldest && ring.m_write == write,
                   "Newest view preserves ring metadata");
    for (std::size_t i = 0; i < 128; ++i)
    {
        __ESBMC_assert(ring.m_samples[i].time_us == old_integers[i][0], "Newest view preserves time_us");
        __ESBMC_assert(ring.m_samples[i].land.time_us == old_integers[i][1], "Newest view preserves land.time_us");
        __ESBMC_assert(ring.m_samples[i].vehicle.time_us == old_integers[i][2],
                       "Newest view preserves vehicle.time_us");
        __ESBMC_assert(ring.m_samples[i].land.known == old_flags[i][0], "Newest view preserves land.known");
        __ESBMC_assert(ring.m_samples[i].land.landed == old_flags[i][1], "Newest view preserves land.landed");
        __ESBMC_assert(ring.m_samples[i].land.at_rest == old_flags[i][2], "Newest view preserves land.at_rest");
        __ESBMC_assert(ring.m_samples[i].land.in_ground_effect == old_flags[i][3],
                       "Newest view preserves land.in_ground_effect");
        __ESBMC_assert(ring.m_samples[i].vehicle.known == old_flags[i][4], "Newest view preserves vehicle.known");
        __ESBMC_assert(ring.m_samples[i].vehicle.armed == old_flags[i][5], "Newest view preserves vehicle.armed");
        __ESBMC_assert(ring.m_samples[i].vehicle.fixed_wing == old_flags[i][6],
                       "Newest view preserves vehicle.fixed_wing");
    }
}
