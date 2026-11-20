// SPDX-License-Identifier: BSD-3-Clause

#include "selected_frame.hpp"
namespace
{
using namespace output_boundary;
bool translation_ok;
Sample committed;
void restore_selected_history()
{
    owner->m_history.m_samples[selected_slot].time_us = selected_history_integers[0];
    owner->m_history.m_samples[selected_slot].start_time_us = selected_history_integers[1];
    owner->m_history.m_samples[selected_slot].angular_period_us = selected_history_integers[2];
    owner->m_history.m_samples[selected_slot].q_nb.m_q0 = selected_history_values[0];
    owner->m_history.m_samples[selected_slot].q_nb.m_q1 = selected_history_values[1];
    owner->m_history.m_samples[selected_slot].q_nb.m_q2 = selected_history_values[2];
    owner->m_history.m_samples[selected_slot].q_nb.m_q3 = selected_history_values[3];
    owner->m_history.m_samples[selected_slot].v_n.m_storage.values.head = selected_history_values[4];
    owner->m_history.m_samples[selected_slot].v_n.m_storage.values.tail.head = selected_history_values[5];
    owner->m_history.m_samples[selected_slot].v_n.m_storage.values.tail.tail.head = selected_history_values[6];
    owner->m_history.m_samples[selected_slot].p_n.m_storage.values.head = selected_history_values[7];
    owner->m_history.m_samples[selected_slot].p_n.m_storage.values.tail.head = selected_history_values[8];
    owner->m_history.m_samples[selected_slot].p_n.m_storage.values.tail.tail.head = selected_history_values[9];
}
} // namespace
namespace formal_eskf::px4
{
template <> bool OutputPredictor<output_proof::Backend>::stage_translation(vector3_type const &, vector3_type const &)
{
    check_selected();
    __ESBMC_init_object(owner);
    capture_selected_workspace();
    restore_outer();
    restore_selected_history();
    return translation_ok;
}
template <> void OutputPredictor<output_proof::Backend>::commit_history()
{
    check_selected();
    __ESBMC_init_object(owner);
    restore_outer();
    restore_selected_workspace();
    m_current = committed;
    capture_selected_history();
}
} // namespace formal_eskf::px4
#include <feedback_steps.hpp>
void verify_output_feedback_frame(Output object, State state, std::uint64_t time, output_steps::FeedbackValues feedback,
                                  bool staged, Sample newest)
{
    owner = &object;
    metadata(object);
    __ESBMC_assume(object.m_history.m_count > 0);
    capture();
    capture_selected();
    translation_ok = staged;
    committed = newest;
    (void)output_steps::correction_finish(object, state, time, feedback);
    check_selected();
}
