// SPDX-License-Identifier: BSD-3-Clause

#pragma once
#define FORMAL_ESKF_PROOF_FRAME_FIRST 0
#define FORMAL_ESKF_PROOF_FRAME_END 0
#include "boundary.hpp"
namespace output_boundary
{
constexpr std::size_t selected_slot = FORMAL_ESKF_PROOF_SLOT;
static_assert(selected_slot < 64);
Scalar selected_history_values[10], selected_workspace_values[10];
std::uint64_t selected_history_integers[3], selected_workspace_integers[3];
void capture_selected_history()
{
    selected_history_integers[0] = owner->m_history.m_samples[selected_slot].time_us;
    selected_history_integers[1] = owner->m_history.m_samples[selected_slot].start_time_us;
    selected_history_integers[2] = owner->m_history.m_samples[selected_slot].angular_period_us;
    selected_history_values[0] = owner->m_history.m_samples[selected_slot].q_nb.m_q0;
    selected_history_values[1] = owner->m_history.m_samples[selected_slot].q_nb.m_q1;
    selected_history_values[2] = owner->m_history.m_samples[selected_slot].q_nb.m_q2;
    selected_history_values[3] = owner->m_history.m_samples[selected_slot].q_nb.m_q3;
    selected_history_values[4] = owner->m_history.m_samples[selected_slot].v_n.m_storage.values.head;
    selected_history_values[5] = owner->m_history.m_samples[selected_slot].v_n.m_storage.values.tail.head;
    selected_history_values[6] = owner->m_history.m_samples[selected_slot].v_n.m_storage.values.tail.tail.head;
    selected_history_values[7] = owner->m_history.m_samples[selected_slot].p_n.m_storage.values.head;
    selected_history_values[8] = owner->m_history.m_samples[selected_slot].p_n.m_storage.values.tail.head;
    selected_history_values[9] = owner->m_history.m_samples[selected_slot].p_n.m_storage.values.tail.tail.head;
}
void capture_selected_workspace()
{
    selected_workspace_integers[0] = owner->m_staged[selected_slot].time_us;
    selected_workspace_integers[1] = owner->m_staged[selected_slot].start_time_us;
    selected_workspace_integers[2] = owner->m_staged[selected_slot].angular_period_us;
    selected_workspace_values[0] = owner->m_staged[selected_slot].q_nb.m_q0;
    selected_workspace_values[1] = owner->m_staged[selected_slot].q_nb.m_q1;
    selected_workspace_values[2] = owner->m_staged[selected_slot].q_nb.m_q2;
    selected_workspace_values[3] = owner->m_staged[selected_slot].q_nb.m_q3;
    selected_workspace_values[4] = owner->m_staged[selected_slot].v_n.m_storage.values.head;
    selected_workspace_values[5] = owner->m_staged[selected_slot].v_n.m_storage.values.tail.head;
    selected_workspace_values[6] = owner->m_staged[selected_slot].v_n.m_storage.values.tail.tail.head;
    selected_workspace_values[7] = owner->m_staged[selected_slot].p_n.m_storage.values.head;
    selected_workspace_values[8] = owner->m_staged[selected_slot].p_n.m_storage.values.tail.head;
    selected_workspace_values[9] = owner->m_staged[selected_slot].p_n.m_storage.values.tail.tail.head;
}
void capture_selected()
{
    capture_selected_history();
    capture_selected_workspace();
}
void restore_selected_workspace()
{
    owner->m_staged[selected_slot].time_us = selected_workspace_integers[0];
    owner->m_staged[selected_slot].start_time_us = selected_workspace_integers[1];
    owner->m_staged[selected_slot].angular_period_us = selected_workspace_integers[2];
    owner->m_staged[selected_slot].q_nb.m_q0 = selected_workspace_values[0];
    owner->m_staged[selected_slot].q_nb.m_q1 = selected_workspace_values[1];
    owner->m_staged[selected_slot].q_nb.m_q2 = selected_workspace_values[2];
    owner->m_staged[selected_slot].q_nb.m_q3 = selected_workspace_values[3];
    owner->m_staged[selected_slot].v_n.m_storage.values.head = selected_workspace_values[4];
    owner->m_staged[selected_slot].v_n.m_storage.values.tail.head = selected_workspace_values[5];
    owner->m_staged[selected_slot].v_n.m_storage.values.tail.tail.head = selected_workspace_values[6];
    owner->m_staged[selected_slot].p_n.m_storage.values.head = selected_workspace_values[7];
    owner->m_staged[selected_slot].p_n.m_storage.values.tail.head = selected_workspace_values[8];
    owner->m_staged[selected_slot].p_n.m_storage.values.tail.tail.head = selected_workspace_values[9];
}
void check_selected()
{
    __ESBMC_assert(owner->m_history.m_samples[selected_slot].time_us == selected_history_integers[0],
                   "Output selected history frame: time_us");
    __ESBMC_assert(owner->m_history.m_samples[selected_slot].start_time_us == selected_history_integers[1],
                   "Output selected history frame: start_time_us");
    __ESBMC_assert(owner->m_history.m_samples[selected_slot].angular_period_us == selected_history_integers[2],
                   "Output selected history frame: angular_period_us");
    __ESBMC_assert(same(owner->m_history.m_samples[selected_slot].q_nb.m_q0, selected_history_values[0]),
                   "Output selected history frame: q_nb.m_q0");
    __ESBMC_assert(same(owner->m_history.m_samples[selected_slot].q_nb.m_q1, selected_history_values[1]),
                   "Output selected history frame: q_nb.m_q1");
    __ESBMC_assert(same(owner->m_history.m_samples[selected_slot].q_nb.m_q2, selected_history_values[2]),
                   "Output selected history frame: q_nb.m_q2");
    __ESBMC_assert(same(owner->m_history.m_samples[selected_slot].q_nb.m_q3, selected_history_values[3]),
                   "Output selected history frame: q_nb.m_q3");
    __ESBMC_assert(
        same(owner->m_history.m_samples[selected_slot].v_n.m_storage.values.head, selected_history_values[4]),
        "Output selected history frame: v_n.m_storage.values.head");
    __ESBMC_assert(
        same(owner->m_history.m_samples[selected_slot].v_n.m_storage.values.tail.head, selected_history_values[5]),
        "Output selected history frame: v_n.m_storage.values.tail.head");
    __ESBMC_assert(
        same(owner->m_history.m_samples[selected_slot].v_n.m_storage.values.tail.tail.head, selected_history_values[6]),
        "Output selected history frame: v_n.m_storage.values.tail.tail.head");
    __ESBMC_assert(
        same(owner->m_history.m_samples[selected_slot].p_n.m_storage.values.head, selected_history_values[7]),
        "Output selected history frame: p_n.m_storage.values.head");
    __ESBMC_assert(
        same(owner->m_history.m_samples[selected_slot].p_n.m_storage.values.tail.head, selected_history_values[8]),
        "Output selected history frame: p_n.m_storage.values.tail.head");
    __ESBMC_assert(
        same(owner->m_history.m_samples[selected_slot].p_n.m_storage.values.tail.tail.head, selected_history_values[9]),
        "Output selected history frame: p_n.m_storage.values.tail.tail.head");
    __ESBMC_assert(owner->m_staged[selected_slot].time_us == selected_workspace_integers[0],
                   "Output selected workspace frame: time_us");
    __ESBMC_assert(owner->m_staged[selected_slot].start_time_us == selected_workspace_integers[1],
                   "Output selected workspace frame: start_time_us");
    __ESBMC_assert(owner->m_staged[selected_slot].angular_period_us == selected_workspace_integers[2],
                   "Output selected workspace frame: angular_period_us");
    __ESBMC_assert(same(owner->m_staged[selected_slot].q_nb.m_q0, selected_workspace_values[0]),
                   "Output selected workspace frame: q_nb.m_q0");
    __ESBMC_assert(same(owner->m_staged[selected_slot].q_nb.m_q1, selected_workspace_values[1]),
                   "Output selected workspace frame: q_nb.m_q1");
    __ESBMC_assert(same(owner->m_staged[selected_slot].q_nb.m_q2, selected_workspace_values[2]),
                   "Output selected workspace frame: q_nb.m_q2");
    __ESBMC_assert(same(owner->m_staged[selected_slot].q_nb.m_q3, selected_workspace_values[3]),
                   "Output selected workspace frame: q_nb.m_q3");
    __ESBMC_assert(same(owner->m_staged[selected_slot].v_n.m_storage.values.head, selected_workspace_values[4]),
                   "Output selected workspace frame: v_n.m_storage.values.head");
    __ESBMC_assert(same(owner->m_staged[selected_slot].v_n.m_storage.values.tail.head, selected_workspace_values[5]),
                   "Output selected workspace frame: v_n.m_storage.values.tail.head");
    __ESBMC_assert(
        same(owner->m_staged[selected_slot].v_n.m_storage.values.tail.tail.head, selected_workspace_values[6]),
        "Output selected workspace frame: v_n.m_storage.values.tail.tail.head");
    __ESBMC_assert(same(owner->m_staged[selected_slot].p_n.m_storage.values.head, selected_workspace_values[7]),
                   "Output selected workspace frame: p_n.m_storage.values.head");
    __ESBMC_assert(same(owner->m_staged[selected_slot].p_n.m_storage.values.tail.head, selected_workspace_values[8]),
                   "Output selected workspace frame: p_n.m_storage.values.tail.head");
    __ESBMC_assert(
        same(owner->m_staged[selected_slot].p_n.m_storage.values.tail.tail.head, selected_workspace_values[9]),
        "Output selected workspace frame: p_n.m_storage.values.tail.tail.head");
}
} // namespace output_boundary
