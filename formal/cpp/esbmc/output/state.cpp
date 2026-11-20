// SPDX-License-Identifier: BSD-3-Clause

#include "support.hpp"
using namespace output_proof;
void verify_output_validation(State state)
{
    Scalar previous[16];
    previous[0] = state.q_nb.q0();
    previous[1] = state.q_nb.q1();
    previous[2] = state.q_nb.q2();
    previous[3] = state.q_nb.q3();
    previous[4] = state.v_n.m_storage.values.head;
    previous[5] = state.v_n.m_storage.values.tail.head;
    previous[6] = state.v_n.m_storage.values.tail.tail.head;
    previous[7] = state.p_n.m_storage.values.head;
    previous[8] = state.p_n.m_storage.values.tail.head;
    previous[9] = state.p_n.m_storage.values.tail.tail.head;
    previous[10] = state.b_a.m_storage.values.head;
    previous[11] = state.b_a.m_storage.values.tail.head;
    previous[12] = state.b_a.m_storage.values.tail.tail.head;
    previous[13] = state.b_g.m_storage.values.head;
    previous[14] = state.b_g.m_storage.values.tail.head;
    previous[15] = state.b_g.m_storage.values.tail.tail.head;
    Scalar squared{0};
    for (std::size_t i = 0; i < 4; ++i)
    {
        squared = squared + previous[i] * previous[i];
    }
    Scalar const residual = squared - Scalar{1};
    Scalar const tolerance = static_cast<Scalar>(profile::quaternion_squared_norm_tolerance);
    bool expected = residual >= -tolerance && residual <= tolerance;
    for (std::size_t i = 4; i < 16; ++i)
    {
        expected = std::isfinite(previous[i]) && expected;
    }
    bool const actual = Output::valid_state(state);
    __ESBMC_assert(actual == expected, "Output state validation: ordered norm residual and all finite vectors");
    __ESBMC_assert(same(state.q_nb.q0(), previous[0]), "Validation preserves input q_nb.q0()");
    __ESBMC_assert(same(state.q_nb.q1(), previous[1]), "Validation preserves input q_nb.q1()");
    __ESBMC_assert(same(state.q_nb.q2(), previous[2]), "Validation preserves input q_nb.q2()");
    __ESBMC_assert(same(state.q_nb.q3(), previous[3]), "Validation preserves input q_nb.q3()");
    __ESBMC_assert(same(state.v_n.m_storage.values.head, previous[4]),
                   "Validation preserves input v_n.m_storage.values.head");
    __ESBMC_assert(same(state.v_n.m_storage.values.tail.head, previous[5]),
                   "Validation preserves input v_n.m_storage.values.tail.head");
    __ESBMC_assert(same(state.v_n.m_storage.values.tail.tail.head, previous[6]),
                   "Validation preserves input v_n.m_storage.values.tail.tail.head");
    __ESBMC_assert(same(state.p_n.m_storage.values.head, previous[7]),
                   "Validation preserves input p_n.m_storage.values.head");
    __ESBMC_assert(same(state.p_n.m_storage.values.tail.head, previous[8]),
                   "Validation preserves input p_n.m_storage.values.tail.head");
    __ESBMC_assert(same(state.p_n.m_storage.values.tail.tail.head, previous[9]),
                   "Validation preserves input p_n.m_storage.values.tail.tail.head");
    __ESBMC_assert(same(state.b_a.m_storage.values.head, previous[10]),
                   "Validation preserves input b_a.m_storage.values.head");
    __ESBMC_assert(same(state.b_a.m_storage.values.tail.head, previous[11]),
                   "Validation preserves input b_a.m_storage.values.tail.head");
    __ESBMC_assert(same(state.b_a.m_storage.values.tail.tail.head, previous[12]),
                   "Validation preserves input b_a.m_storage.values.tail.tail.head");
    __ESBMC_assert(same(state.b_g.m_storage.values.head, previous[13]),
                   "Validation preserves input b_g.m_storage.values.head");
    __ESBMC_assert(same(state.b_g.m_storage.values.tail.head, previous[14]),
                   "Validation preserves input b_g.m_storage.values.tail.head");
    __ESBMC_assert(same(state.b_g.m_storage.values.tail.tail.head, previous[15]),
                   "Validation preserves input b_g.m_storage.values.tail.tail.head");
}
