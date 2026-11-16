// SPDX-License-Identifier: BSD-3-Clause

#include "../../formal/cpp/esbmc/ring_buffer.cpp"

#ifndef FORMAL_ESKF_TEST_HISTORY
#define FORMAL_ESKF_TEST_HISTORY 0
#endif

// Deliberate counterexamples check that induction premises admit full, empty
// and partial wrapped histories. These are verifier regressions, not proofs.
void verify_buffer_seed_reachable()
{
    static_assert(capacity > 1);
    Buffer buffer(0);
    seed_history(buffer);
    constexpr auto count = FORMAL_ESKF_TEST_HISTORY == 0 ? capacity : FORMAL_ESKF_TEST_HISTORY == 1 ? 0 : capacity - 1;
    __ESBMC_assume(buffer.m_length == capacity && buffer.m_count == count && buffer.m_oldest == capacity - 1);
    __ESBMC_assert(false, "buffer seed: requested wrapped history is reachable");
}
