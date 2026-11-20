// SPDX-License-Identifier: BSD-3-Clause

#pragma once
#define private public
#include "../contracts.hpp"
#undef private
#include "frames.hpp"
namespace output_boundary
{
using namespace output_frames;
// Actual typed storage writes no enclosing-owner fields. Restore that exact
// frame after whole-root havoc; arbitrary producer payloads remain unconstrained.
void restore_outer()
{
    owner->m_current.time_us = integers[field::time];
    owner->m_current.start_time_us = integers[field::start_time];
    owner->m_current.angular_period_us = integers[field::period];
    owner->m_fusion_time_us = integers[field::fusion_time];
    owner->m_history.m_length = integers[field::length];
    owner->m_history.m_count = integers[field::count];
    owner->m_history.m_oldest = integers[field::oldest];
    owner->m_history.m_write = integers[field::write];
    owner->m_configuration.minimum_lag_us = integers[field::minimum_lag];
    owner->m_configuration.gnss_delay_us = integers[field::gnss_delay];
    owner->m_configuration.magnetic_delay_us = integers[field::magnetic_delay];
    owner->m_configuration.barometer_delay_us = integers[field::barometer_delay];
    owner->m_configuration.imu_length = integers[field::configured_length];
    owner->m_current.q_nb.m_q0 = values[field::q0];
    owner->m_current.q_nb.m_q1 = values[field::q1];
    owner->m_current.q_nb.m_q2 = values[field::q2];
    owner->m_current.q_nb.m_q3 = values[field::q3];
    owner->m_current.v_n.m_storage.values.head = values[field::velocity_x];
    owner->m_current.v_n.m_storage.values.tail.head = values[field::velocity_y];
    owner->m_current.v_n.m_storage.values.tail.tail.head = values[field::velocity_z];
    owner->m_current.p_n.m_storage.values.head = values[field::position_x];
    owner->m_current.p_n.m_storage.values.tail.head = values[field::position_y];
    owner->m_current.p_n.m_storage.values.tail.tail.head = values[field::position_z];
    owner->m_accel_bias.m_storage.values.head = values[field::accel_bias_x];
    owner->m_accel_bias.m_storage.values.tail.head = values[field::accel_bias_y];
    owner->m_accel_bias.m_storage.values.tail.tail.head = values[field::accel_bias_z];
    owner->m_gyro_bias.m_storage.values.head = values[field::gyro_bias_x];
    owner->m_gyro_bias.m_storage.values.tail.head = values[field::gyro_bias_y];
    owner->m_gyro_bias.m_storage.values.tail.tail.head = values[field::gyro_bias_z];
    owner->m_angular_rate_correction.m_storage.values.head = values[field::angular_correction_x];
    owner->m_angular_rate_correction.m_storage.values.tail.head = values[field::angular_correction_y];
    owner->m_angular_rate_correction.m_storage.values.tail.tail.head = values[field::angular_correction_z];
    owner->m_output_angular_rate_b.m_storage.values.head = values[field::propagated_rate_x];
    owner->m_output_angular_rate_b.m_storage.values.tail.head = values[field::propagated_rate_y];
    owner->m_output_angular_rate_b.m_storage.values.tail.tail.head = values[field::propagated_rate_z];
    owner->m_velocity_error_sum.m_storage.values.head = values[field::velocity_sum_x];
    owner->m_velocity_error_sum.m_storage.values.tail.head = values[field::velocity_sum_y];
    owner->m_velocity_error_sum.m_storage.values.tail.tail.head = values[field::velocity_sum_z];
    owner->m_position_error_sum.m_storage.values.head = values[field::position_sum_x];
    owner->m_position_error_sum.m_storage.values.tail.head = values[field::position_sum_y];
    owner->m_position_error_sum.m_storage.values.tail.tail.head = values[field::position_sum_z];
    owner->m_error.attitude_b.m_storage.values.head = values[field::attitude_error_x];
    owner->m_error.attitude_b.m_storage.values.tail.head = values[field::attitude_error_y];
    owner->m_error.attitude_b.m_storage.values.tail.tail.head = values[field::attitude_error_z];
    owner->m_error.velocity_n.m_storage.values.head = values[field::velocity_error_x];
    owner->m_error.velocity_n.m_storage.values.tail.head = values[field::velocity_error_y];
    owner->m_error.velocity_n.m_storage.values.tail.tail.head = values[field::velocity_error_z];
    owner->m_error.position_n.m_storage.values.head = values[field::position_error_x];
    owner->m_error.position_n.m_storage.values.tail.head = values[field::position_error_y];
    owner->m_error.position_n.m_storage.values.tail.tail.head = values[field::position_error_z];
    owner->m_filter_interval_s = values[field::interval_mean];
    owner->m_tracking = tracking;
    owner->m_fault = fault;
    for (std::size_t i = FORMAL_ESKF_PROOF_FRAME_FIRST; i < FORMAL_ESKF_PROOF_FRAME_END; ++i)
    {
        owner->m_staged[i].time_us = staged_integers[i][0];
        owner->m_staged[i].start_time_us = staged_integers[i][1];
        owner->m_staged[i].angular_period_us = staged_integers[i][2];
        owner->m_staged[i].q_nb.m_q0 = staged_values[i][0];
        owner->m_staged[i].q_nb.m_q1 = staged_values[i][1];
        owner->m_staged[i].q_nb.m_q2 = staged_values[i][2];
        owner->m_staged[i].q_nb.m_q3 = staged_values[i][3];
        owner->m_staged[i].v_n.m_storage.values.head = staged_values[i][4];
        owner->m_staged[i].v_n.m_storage.values.tail.head = staged_values[i][5];
        owner->m_staged[i].v_n.m_storage.values.tail.tail.head = staged_values[i][6];
        owner->m_staged[i].p_n.m_storage.values.head = staged_values[i][7];
        owner->m_staged[i].p_n.m_storage.values.tail.head = staged_values[i][8];
        owner->m_staged[i].p_n.m_storage.values.tail.tail.head = staged_values[i][9];
    }
}
void capture_storage()
{
    for (std::size_t i = FORMAL_ESKF_PROOF_FRAME_FIRST; i < FORMAL_ESKF_PROOF_FRAME_END; ++i)
    {
        history_integers[i][0] = owner->m_history.m_samples[i].time_us;
        history_integers[i][1] = owner->m_history.m_samples[i].start_time_us;
        history_integers[i][2] = owner->m_history.m_samples[i].angular_period_us;
        history_values[i][0] = owner->m_history.m_samples[i].q_nb.q0();
        history_values[i][1] = owner->m_history.m_samples[i].q_nb.q1();
        history_values[i][2] = owner->m_history.m_samples[i].q_nb.q2();
        history_values[i][3] = owner->m_history.m_samples[i].q_nb.q3();
        history_values[i][4] = owner->m_history.m_samples[i].v_n.m_storage.values.head;
        history_values[i][5] = owner->m_history.m_samples[i].v_n.m_storage.values.tail.head;
        history_values[i][6] = owner->m_history.m_samples[i].v_n.m_storage.values.tail.tail.head;
        history_values[i][7] = owner->m_history.m_samples[i].p_n.m_storage.values.head;
        history_values[i][8] = owner->m_history.m_samples[i].p_n.m_storage.values.tail.head;
        history_values[i][9] = owner->m_history.m_samples[i].p_n.m_storage.values.tail.tail.head;
    }
}
void expect_current(Sample const & s)
{
    integers[field::time] = s.time_us;
    integers[field::start_time] = s.start_time_us;
    integers[field::period] = s.angular_period_us;
    values[field::q0] = s.q_nb.q0();
    values[field::q1] = s.q_nb.q1();
    values[field::q2] = s.q_nb.q2();
    values[field::q3] = s.q_nb.q3();
    values[field::velocity_x] = s.v_n.m_storage.values.head;
    values[field::velocity_y] = s.v_n.m_storage.values.tail.head;
    values[field::velocity_z] = s.v_n.m_storage.values.tail.tail.head;
    values[field::position_x] = s.p_n.m_storage.values.head;
    values[field::position_y] = s.p_n.m_storage.values.tail.head;
    values[field::position_z] = s.p_n.m_storage.values.tail.tail.head;
}
bool same_sample(Sample const & a, Sample const & b)
{
    bool result = true;
    result = (a.time_us == b.time_us) & result;
    result = (a.start_time_us == b.start_time_us) & result;
    result = (a.angular_period_us == b.angular_period_us) & result;
    result = same(a.q_nb.q0(), b.q_nb.q0()) & result;
    result = same(a.q_nb.q1(), b.q_nb.q1()) & result;
    result = same(a.q_nb.q2(), b.q_nb.q2()) & result;
    result = same(a.q_nb.q3(), b.q_nb.q3()) & result;
    result = same(a.v_n.m_storage.values.head, b.v_n.m_storage.values.head) & result;
    result = same(a.v_n.m_storage.values.tail.head, b.v_n.m_storage.values.tail.head) & result;
    result = same(a.v_n.m_storage.values.tail.tail.head, b.v_n.m_storage.values.tail.tail.head) & result;
    result = same(a.p_n.m_storage.values.head, b.p_n.m_storage.values.head) & result;
    result = same(a.p_n.m_storage.values.tail.head, b.p_n.m_storage.values.tail.head) & result;
    result = same(a.p_n.m_storage.values.tail.tail.head, b.p_n.m_storage.values.tail.tail.head) & result;
    return result;
}
void metadata(Output & object)
{
    auto const length = imu_proof::nondet_index();
    auto const count = imu_proof::nondet_index();
    auto const oldest = imu_proof::nondet_index();
    __ESBMC_assume(length <= 64 && length != 1 && count <= length && (length == 0 ? oldest == 0 : oldest < length));
    __ESBMC_assume(object.m_current.time_us == 0 || length >= 2);
    auto const sum = static_cast<std::size_t>(oldest) + count;
    object.m_history.m_length = length;
    object.m_history.m_count = count;
    object.m_history.m_oldest = oldest;
    object.m_history.m_write = sum < length ? sum : sum - length;
}
} // namespace output_boundary
