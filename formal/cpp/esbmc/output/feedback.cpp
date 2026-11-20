// SPDX-License-Identifier: BSD-3-Clause

#include "boundary.hpp"
// Behavior and outer-owner fields are proved here. feedback_frame.cpp supplies
// every history/workspace frame over the same inputs and producer outcomes.
static_assert(FORMAL_ESKF_PROOF_FRAME_FIRST == 0 && FORMAL_ESKF_PROOF_FRAME_END == 0);
namespace
{
using namespace output_boundary;
State requested_state;
Vector velocity_translation, position_translation;
Sample committed;
bool translation_ok;
unsigned translations, commits;
void check_state(State const & state)
{
    auto const q = state.q_nb.coefficients();
    auto const original = requested_state.q_nb.coefficients();
    for (std::size_t i = 0; i < 4; ++i)
    {
        __ESBMC_assert(same(q(i), original(i)), "Output correction forwards all input quaternion coefficients");
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
        __ESBMC_assert(same(state.p_n(i), requested_state.p_n(i)) && same(state.v_n(i), requested_state.v_n(i)) &&
                           same(state.b_a(i), requested_state.b_a(i)) && same(state.b_g(i), requested_state.b_g(i)),
                       "Output correction forwards position, velocity and both biases");
    }
}
} // namespace
namespace formal_eskf::px4
{
template <>
bool OutputPredictor<output_proof::Backend>::stage_translation(vector3_type const & velocity,
                                                               vector3_type const & position)
{
    check();
    __ESBMC_assert(this == owner && translations == 0 && commits == 0,
                   "Output correction tail stages exactly once before publication");
    for (std::size_t i = 0; i < 3; ++i)
    {
        __ESBMC_assert(same(velocity(i), velocity_translation(i)) && same(position(i), position_translation(i)),
                       "Output correction forwards both complete PI translations");
    }
    __ESBMC_init_object(owner);
    // This behavioral overapproximation leaves all storage arbitrary. The
    // companion frame proof establishes the narrower actual storage contract.
    restore_outer();
    ++translations;
    return translation_ok;
}
template <> void OutputPredictor<output_proof::Backend>::commit_history()
{
    check();
    __ESBMC_assert(this == owner && translations == 1 && translation_ok && commits == 0 && m_history.m_count > 0,
                   "Output correction publishes only fully validated nonempty history");
    __ESBMC_init_object(owner);
    restore_outer();
    // The actual commit producer supplies the per-slot copy and newest equations.
    // Its caller must retain any returned visible trajectory unchanged.
    m_current = committed;
    expect_current(committed);
    ++commits;
}
} // namespace formal_eskf::px4
#include <feedback_steps.hpp>
void verify_output_feedback_finish(Output object, State state, std::uint64_t time,
                                   output_steps::FeedbackValues feedback, bool staged, Sample newest)
{
    owner = &object;
    metadata(object);
    __ESBMC_assume(object.m_history.m_count > 0);
    capture();
    auto const before_feedback = feedback;
    requested_state = state;
    translation_ok = staged;
    committed = newest;
    velocity_translation = feedback.velocity_delta;
    position_translation = feedback.position_delta;
    translations = commits = 0;
    bool finite = true;
    for (std::size_t i = 0; i < 3; ++i)
    {
        finite = std::isfinite(feedback.angular_rate_correction(i)) & std::isfinite(feedback.velocity_sum(i)) &
                 std::isfinite(feedback.position_sum(i)) & finite;
    }
    bool const accepted = finite && staged;
    bool const result = output_steps::correction_finish(object, state, time, feedback);
    __ESBMC_assert(result == accepted && translations == static_cast<unsigned>(finite) &&
                       commits == static_cast<unsigned>(accepted),
                   "Output feedback tail checks finiteness and complete staging before commit");
    if (accepted)
    {
        for (std::size_t i = 0; i < 3; ++i)
        {
            values[field::accel_bias_x + i] = state.b_a(i);
            values[field::gyro_bias_x + i] = state.b_g(i);
            values[field::angular_correction_x + i] = feedback.angular_rate_correction(i);
            values[field::velocity_sum_x + i] = feedback.velocity_sum(i);
            values[field::position_sum_x + i] = feedback.position_sum(i);
            values[field::attitude_error_x + i] = feedback.error.attitude_b(i);
            values[field::velocity_error_x + i] = feedback.error.velocity_n(i);
            values[field::position_error_x + i] = feedback.error.position_n(i);
        }
        values[field::interval_mean] = feedback.interval_s;
        integers[field::fusion_time] = time;
        tracking = true;
    }
    else
    {
        fault = Output::Fault::numerical_failure;
    }
    check();
    check_state(state);
    __ESBMC_assert(same(feedback.interval_s, before_feedback.interval_s),
                   "Output feedback tail preserves interval input");
    for (std::size_t i = 0; i < 3; ++i)
    {
        __ESBMC_assert(same(feedback.error.attitude_b(i), before_feedback.error.attitude_b(i)),
                       "Output feedback tail preserves input error.attitude_b");
        __ESBMC_assert(same(feedback.error.velocity_n(i), before_feedback.error.velocity_n(i)),
                       "Output feedback tail preserves input error.velocity_n");
        __ESBMC_assert(same(feedback.error.position_n(i), before_feedback.error.position_n(i)),
                       "Output feedback tail preserves input error.position_n");
        __ESBMC_assert(same(feedback.angular_rate_correction(i), before_feedback.angular_rate_correction(i)),
                       "Output feedback tail preserves input angular_rate_correction");
        __ESBMC_assert(same(feedback.velocity_sum(i), before_feedback.velocity_sum(i)),
                       "Output feedback tail preserves input velocity_sum");
        __ESBMC_assert(same(feedback.position_sum(i), before_feedback.position_sum(i)),
                       "Output feedback tail preserves input position_sum");
        __ESBMC_assert(same(feedback.velocity_delta(i), before_feedback.velocity_delta(i)),
                       "Output feedback tail preserves input velocity_delta");
        __ESBMC_assert(same(feedback.position_delta(i), before_feedback.position_delta(i)),
                       "Output feedback tail preserves input position_delta");
    }
}
