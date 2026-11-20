// SPDX-License-Identifier: BSD-3-Clause

#include "boundary.hpp"
#include <source_steps.hpp>
using namespace output_boundary;
void verify_output_translation_step(Output object, Vector velocity, Vector position)
{
    constexpr std::size_t slot = FORMAL_ESKF_PROOF_SLOT;
    static_assert(slot < 64);
    owner = &object;
    metadata(object);
    __ESBMC_assume(slot < object.m_history.m_length);
    capture();
    auto const velocity_before = velocity;
    auto const position_before = position;
    auto index = slot;
    bool expected = true;
    for (std::size_t i = 0; i < 3; ++i)
    {
        staged_values[slot][4 + i] = history_values[slot][4 + i] + velocity(i);
        staged_values[slot][7 + i] = history_values[slot][7 + i] + position(i);
        expected = std::isfinite(staged_values[slot][4 + i]) & std::isfinite(staged_values[slot][7 + i]) & expected;
        staged_integers[slot][i] = history_integers[slot][i];
    }
    for (std::size_t i = 0; i < 4; ++i)
    {
        staged_values[slot][i] = history_values[slot][i];
    }
    bool const result = output_steps::translation(object, index, velocity, position);
    __ESBMC_assert(result == expected &&
                       index == (expected ? (slot + 1 == integers[field::length] ? 0 : slot + 1) : slot),
                   "Output translation step exact finite result and first-failure cursor");
    check();
    for (std::size_t i = 0; i < 3; ++i)
    {
        __ESBMC_assert(same(velocity(i), velocity_before(i)) && same(position(i), position_before(i)),
                       "Output translation preserves both external deltas");
    }
}
void verify_output_commit_step(Output object)
{
    constexpr std::size_t slot = FORMAL_ESKF_PROOF_SLOT;
    static_assert(slot < 64);
    owner = &object;
    metadata(object);
    __ESBMC_assume(slot < object.m_history.m_length);
    capture();
    for (std::size_t i = 0; i < 3; ++i)
    {
        history_integers[slot][i] = staged_integers[slot][i];
    }
    for (std::size_t i = 0; i < 10; ++i)
    {
        history_values[slot][i] = staged_values[slot][i];
    }
    auto index = slot;
    output_steps::commit(object, index);
    __ESBMC_assert(index == (slot + 1 == integers[field::length] ? 0 : slot + 1),
                   "Output commit step advances the physical cursor exactly");
    check();
}
void verify_output_commit_finish(Output object)
{
    constexpr std::size_t slot = FORMAL_ESKF_PROOF_SLOT;
    static_assert(slot < 64);
    owner = &object;
    metadata(object);
    __ESBMC_assume(object.m_history.m_count > 0 &&
                   (object.m_history.m_write == 0 ? object.m_history.m_length - 1 : object.m_history.m_write - 1) ==
                       slot);
    capture();
    expect_current(object.m_history.m_samples[slot]);
    output_steps::finish(object);
    check();
}
