// SPDX-License-Identifier: BSD-3-Clause

#pragma once
#include "support.hpp"
#ifndef FORMAL_ESKF_PROOF_FRAME_FIRST
#define FORMAL_ESKF_PROOF_FRAME_FIRST 0
#define FORMAL_ESKF_PROOF_FRAME_END 64
#endif
static_assert(0 <= FORMAL_ESKF_PROOF_FRAME_FIRST && FORMAL_ESKF_PROOF_FRAME_FIRST <= FORMAL_ESKF_PROOF_FRAME_END &&
              FORMAL_ESKF_PROOF_FRAME_END <= 64);
namespace output_frames
{
using namespace output_proof;
namespace field
{
inline constexpr std::size_t q0 = 0;
inline constexpr std::size_t q1 = 1;
inline constexpr std::size_t q2 = 2;
inline constexpr std::size_t q3 = 3;
inline constexpr std::size_t velocity_x = 4;
inline constexpr std::size_t velocity_y = 5;
inline constexpr std::size_t velocity_z = 6;
inline constexpr std::size_t position_x = 7;
inline constexpr std::size_t position_y = 8;
inline constexpr std::size_t position_z = 9;
inline constexpr std::size_t accel_bias_x = 10;
inline constexpr std::size_t accel_bias_y = 11;
inline constexpr std::size_t accel_bias_z = 12;
inline constexpr std::size_t gyro_bias_x = 13;
inline constexpr std::size_t gyro_bias_y = 14;
inline constexpr std::size_t gyro_bias_z = 15;
inline constexpr std::size_t angular_correction_x = 16;
inline constexpr std::size_t angular_correction_y = 17;
inline constexpr std::size_t angular_correction_z = 18;
inline constexpr std::size_t propagated_rate_x = 19;
inline constexpr std::size_t propagated_rate_y = 20;
inline constexpr std::size_t propagated_rate_z = 21;
inline constexpr std::size_t velocity_sum_x = 22;
inline constexpr std::size_t velocity_sum_y = 23;
inline constexpr std::size_t velocity_sum_z = 24;
inline constexpr std::size_t position_sum_x = 25;
inline constexpr std::size_t position_sum_y = 26;
inline constexpr std::size_t position_sum_z = 27;
inline constexpr std::size_t attitude_error_x = 28;
inline constexpr std::size_t attitude_error_y = 29;
inline constexpr std::size_t attitude_error_z = 30;
inline constexpr std::size_t velocity_error_x = 31;
inline constexpr std::size_t velocity_error_y = 32;
inline constexpr std::size_t velocity_error_z = 33;
inline constexpr std::size_t position_error_x = 34;
inline constexpr std::size_t position_error_y = 35;
inline constexpr std::size_t position_error_z = 36;
inline constexpr std::size_t interval_mean = 37;
inline constexpr std::size_t time = 0;
inline constexpr std::size_t start_time = 1;
inline constexpr std::size_t period = 2;
inline constexpr std::size_t fusion_time = 3;
inline constexpr std::size_t length = 4;
inline constexpr std::size_t count = 5;
inline constexpr std::size_t oldest = 6;
inline constexpr std::size_t write = 7;
inline constexpr std::size_t minimum_lag = 8;
inline constexpr std::size_t gnss_delay = 9;
inline constexpr std::size_t magnetic_delay = 10;
inline constexpr std::size_t barometer_delay = 11;
inline constexpr std::size_t configured_length = 12;
} // namespace field
Output * owner;
// Record scalars: q0..q3, NED velocity x/y/z, NED position x/y/z.
// Record integers: endpoint, start, angular period. Flat snapshots avoid
// unsupported nested snapshot copies in the verifier. Every field is checked.
Scalar history_values[64][10], staged_values[64][10];
std::uint64_t history_integers[64][3], staged_integers[64][3];
Scalar values[38];
std::uint64_t integers[13];
bool tracking;
Output::Fault fault;
void capture()
{
    integers[field::time] = owner->m_current.time_us;
    integers[field::start_time] = owner->m_current.start_time_us;
    integers[field::period] = owner->m_current.angular_period_us;
    integers[field::fusion_time] = owner->m_fusion_time_us;
    integers[field::length] = owner->m_history.m_length;
    integers[field::count] = owner->m_history.m_count;
    integers[field::oldest] = owner->m_history.m_oldest;
    integers[field::write] = owner->m_history.m_write;
    integers[field::minimum_lag] = owner->m_configuration.minimum_lag_us;
    integers[field::gnss_delay] = owner->m_configuration.gnss_delay_us;
    integers[field::magnetic_delay] = owner->m_configuration.magnetic_delay_us;
    integers[field::barometer_delay] = owner->m_configuration.barometer_delay_us;
    integers[field::configured_length] = owner->m_configuration.imu_length;
    values[field::q0] = owner->m_current.q_nb.q0();
    values[field::q1] = owner->m_current.q_nb.q1();
    values[field::q2] = owner->m_current.q_nb.q2();
    values[field::q3] = owner->m_current.q_nb.q3();
    values[field::velocity_x] = owner->m_current.v_n.m_storage.values.head;
    values[field::velocity_y] = owner->m_current.v_n.m_storage.values.tail.head;
    values[field::velocity_z] = owner->m_current.v_n.m_storage.values.tail.tail.head;
    values[field::position_x] = owner->m_current.p_n.m_storage.values.head;
    values[field::position_y] = owner->m_current.p_n.m_storage.values.tail.head;
    values[field::position_z] = owner->m_current.p_n.m_storage.values.tail.tail.head;
    values[field::accel_bias_x] = owner->m_accel_bias.m_storage.values.head;
    values[field::accel_bias_y] = owner->m_accel_bias.m_storage.values.tail.head;
    values[field::accel_bias_z] = owner->m_accel_bias.m_storage.values.tail.tail.head;
    values[field::gyro_bias_x] = owner->m_gyro_bias.m_storage.values.head;
    values[field::gyro_bias_y] = owner->m_gyro_bias.m_storage.values.tail.head;
    values[field::gyro_bias_z] = owner->m_gyro_bias.m_storage.values.tail.tail.head;
    values[field::angular_correction_x] = owner->m_angular_rate_correction.m_storage.values.head;
    values[field::angular_correction_y] = owner->m_angular_rate_correction.m_storage.values.tail.head;
    values[field::angular_correction_z] = owner->m_angular_rate_correction.m_storage.values.tail.tail.head;
    values[field::propagated_rate_x] = owner->m_output_angular_rate_b.m_storage.values.head;
    values[field::propagated_rate_y] = owner->m_output_angular_rate_b.m_storage.values.tail.head;
    values[field::propagated_rate_z] = owner->m_output_angular_rate_b.m_storage.values.tail.tail.head;
    values[field::velocity_sum_x] = owner->m_velocity_error_sum.m_storage.values.head;
    values[field::velocity_sum_y] = owner->m_velocity_error_sum.m_storage.values.tail.head;
    values[field::velocity_sum_z] = owner->m_velocity_error_sum.m_storage.values.tail.tail.head;
    values[field::position_sum_x] = owner->m_position_error_sum.m_storage.values.head;
    values[field::position_sum_y] = owner->m_position_error_sum.m_storage.values.tail.head;
    values[field::position_sum_z] = owner->m_position_error_sum.m_storage.values.tail.tail.head;
    values[field::attitude_error_x] = owner->m_error.attitude_b.m_storage.values.head;
    values[field::attitude_error_y] = owner->m_error.attitude_b.m_storage.values.tail.head;
    values[field::attitude_error_z] = owner->m_error.attitude_b.m_storage.values.tail.tail.head;
    values[field::velocity_error_x] = owner->m_error.velocity_n.m_storage.values.head;
    values[field::velocity_error_y] = owner->m_error.velocity_n.m_storage.values.tail.head;
    values[field::velocity_error_z] = owner->m_error.velocity_n.m_storage.values.tail.tail.head;
    values[field::position_error_x] = owner->m_error.position_n.m_storage.values.head;
    values[field::position_error_y] = owner->m_error.position_n.m_storage.values.tail.head;
    values[field::position_error_z] = owner->m_error.position_n.m_storage.values.tail.tail.head;
    values[field::interval_mean] = owner->m_filter_interval_s;
    tracking = owner->m_tracking;
    fault = owner->m_fault;
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
        staged_integers[i][0] = owner->m_staged[i].time_us;
        staged_integers[i][1] = owner->m_staged[i].start_time_us;
        staged_integers[i][2] = owner->m_staged[i].angular_period_us;
        staged_values[i][0] = owner->m_staged[i].q_nb.q0();
        staged_values[i][1] = owner->m_staged[i].q_nb.q1();
        staged_values[i][2] = owner->m_staged[i].q_nb.q2();
        staged_values[i][3] = owner->m_staged[i].q_nb.q3();
        staged_values[i][4] = owner->m_staged[i].v_n.m_storage.values.head;
        staged_values[i][5] = owner->m_staged[i].v_n.m_storage.values.tail.head;
        staged_values[i][6] = owner->m_staged[i].v_n.m_storage.values.tail.tail.head;
        staged_values[i][7] = owner->m_staged[i].p_n.m_storage.values.head;
        staged_values[i][8] = owner->m_staged[i].p_n.m_storage.values.tail.head;
        staged_values[i][9] = owner->m_staged[i].p_n.m_storage.values.tail.tail.head;
    }
}
void check()
{
    __ESBMC_assert(owner->m_current.time_us == integers[field::time], "Output complete owner frame: m_current.time_us");
    __ESBMC_assert(owner->m_current.start_time_us == integers[field::start_time],
                   "Output complete owner frame: m_current.start_time_us");
    __ESBMC_assert(owner->m_current.angular_period_us == integers[field::period],
                   "Output complete owner frame: m_current.angular_period_us");
    __ESBMC_assert(owner->m_fusion_time_us == integers[field::fusion_time],
                   "Output complete owner frame: m_fusion_time_us");
    __ESBMC_assert(owner->m_history.m_length == integers[field::length],
                   "Output complete owner frame: m_history.m_length");
    __ESBMC_assert(owner->m_history.m_count == integers[field::count],
                   "Output complete owner frame: m_history.m_count");
    __ESBMC_assert(owner->m_history.m_oldest == integers[field::oldest],
                   "Output complete owner frame: m_history.m_oldest");
    __ESBMC_assert(owner->m_history.m_write == integers[field::write],
                   "Output complete owner frame: m_history.m_write");
    __ESBMC_assert(owner->m_configuration.minimum_lag_us == integers[field::minimum_lag],
                   "Output complete owner frame: m_configuration.minimum_lag_us");
    __ESBMC_assert(owner->m_configuration.gnss_delay_us == integers[field::gnss_delay],
                   "Output complete owner frame: m_configuration.gnss_delay_us");
    __ESBMC_assert(owner->m_configuration.magnetic_delay_us == integers[field::magnetic_delay],
                   "Output complete owner frame: m_configuration.magnetic_delay_us");
    __ESBMC_assert(owner->m_configuration.barometer_delay_us == integers[field::barometer_delay],
                   "Output complete owner frame: m_configuration.barometer_delay_us");
    __ESBMC_assert(owner->m_configuration.imu_length == integers[field::configured_length],
                   "Output complete owner frame: m_configuration.imu_length");
    __ESBMC_assert(same(owner->m_current.q_nb.q0(), values[field::q0]),
                   "Output complete owner frame: m_current.q_nb.q0()");
    __ESBMC_assert(same(owner->m_current.q_nb.q1(), values[field::q1]),
                   "Output complete owner frame: m_current.q_nb.q1()");
    __ESBMC_assert(same(owner->m_current.q_nb.q2(), values[field::q2]),
                   "Output complete owner frame: m_current.q_nb.q2()");
    __ESBMC_assert(same(owner->m_current.q_nb.q3(), values[field::q3]),
                   "Output complete owner frame: m_current.q_nb.q3()");
    __ESBMC_assert(same(owner->m_current.v_n.m_storage.values.head, values[field::velocity_x]),
                   "Output complete owner frame: m_current.v_n.m_storage.values.head");
    __ESBMC_assert(same(owner->m_current.v_n.m_storage.values.tail.head, values[field::velocity_y]),
                   "Output complete owner frame: m_current.v_n.m_storage.values.tail.head");
    __ESBMC_assert(same(owner->m_current.v_n.m_storage.values.tail.tail.head, values[field::velocity_z]),
                   "Output complete owner frame: m_current.v_n.m_storage.values.tail.tail.head");
    __ESBMC_assert(same(owner->m_current.p_n.m_storage.values.head, values[field::position_x]),
                   "Output complete owner frame: m_current.p_n.m_storage.values.head");
    __ESBMC_assert(same(owner->m_current.p_n.m_storage.values.tail.head, values[field::position_y]),
                   "Output complete owner frame: m_current.p_n.m_storage.values.tail.head");
    __ESBMC_assert(same(owner->m_current.p_n.m_storage.values.tail.tail.head, values[field::position_z]),
                   "Output complete owner frame: m_current.p_n.m_storage.values.tail.tail.head");
    __ESBMC_assert(same(owner->m_accel_bias.m_storage.values.head, values[field::accel_bias_x]),
                   "Output complete owner frame: m_accel_bias.m_storage.values.head");
    __ESBMC_assert(same(owner->m_accel_bias.m_storage.values.tail.head, values[field::accel_bias_y]),
                   "Output complete owner frame: m_accel_bias.m_storage.values.tail.head");
    __ESBMC_assert(same(owner->m_accel_bias.m_storage.values.tail.tail.head, values[field::accel_bias_z]),
                   "Output complete owner frame: m_accel_bias.m_storage.values.tail.tail.head");
    __ESBMC_assert(same(owner->m_gyro_bias.m_storage.values.head, values[field::gyro_bias_x]),
                   "Output complete owner frame: m_gyro_bias.m_storage.values.head");
    __ESBMC_assert(same(owner->m_gyro_bias.m_storage.values.tail.head, values[field::gyro_bias_y]),
                   "Output complete owner frame: m_gyro_bias.m_storage.values.tail.head");
    __ESBMC_assert(same(owner->m_gyro_bias.m_storage.values.tail.tail.head, values[field::gyro_bias_z]),
                   "Output complete owner frame: m_gyro_bias.m_storage.values.tail.tail.head");
    __ESBMC_assert(same(owner->m_angular_rate_correction.m_storage.values.head, values[field::angular_correction_x]),
                   "Output complete owner frame: m_angular_rate_correction.m_storage.values.head");
    __ESBMC_assert(
        same(owner->m_angular_rate_correction.m_storage.values.tail.head, values[field::angular_correction_y]),
        "Output complete owner frame: m_angular_rate_correction.m_storage.values.tail.head");
    __ESBMC_assert(
        same(owner->m_angular_rate_correction.m_storage.values.tail.tail.head, values[field::angular_correction_z]),
        "Output complete owner frame: m_angular_rate_correction.m_storage.values.tail.tail.head");
    __ESBMC_assert(same(owner->m_output_angular_rate_b.m_storage.values.head, values[field::propagated_rate_x]),
                   "Output complete owner frame: m_output_angular_rate_b.m_storage.values.head");
    __ESBMC_assert(same(owner->m_output_angular_rate_b.m_storage.values.tail.head, values[field::propagated_rate_y]),
                   "Output complete owner frame: m_output_angular_rate_b.m_storage.values.tail.head");
    __ESBMC_assert(
        same(owner->m_output_angular_rate_b.m_storage.values.tail.tail.head, values[field::propagated_rate_z]),
        "Output complete owner frame: m_output_angular_rate_b.m_storage.values.tail.tail.head");
    __ESBMC_assert(same(owner->m_velocity_error_sum.m_storage.values.head, values[field::velocity_sum_x]),
                   "Output complete owner frame: m_velocity_error_sum.m_storage.values.head");
    __ESBMC_assert(same(owner->m_velocity_error_sum.m_storage.values.tail.head, values[field::velocity_sum_y]),
                   "Output complete owner frame: m_velocity_error_sum.m_storage.values.tail.head");
    __ESBMC_assert(same(owner->m_velocity_error_sum.m_storage.values.tail.tail.head, values[field::velocity_sum_z]),
                   "Output complete owner frame: m_velocity_error_sum.m_storage.values.tail.tail.head");
    __ESBMC_assert(same(owner->m_position_error_sum.m_storage.values.head, values[field::position_sum_x]),
                   "Output complete owner frame: m_position_error_sum.m_storage.values.head");
    __ESBMC_assert(same(owner->m_position_error_sum.m_storage.values.tail.head, values[field::position_sum_y]),
                   "Output complete owner frame: m_position_error_sum.m_storage.values.tail.head");
    __ESBMC_assert(same(owner->m_position_error_sum.m_storage.values.tail.tail.head, values[field::position_sum_z]),
                   "Output complete owner frame: m_position_error_sum.m_storage.values.tail.tail.head");
    __ESBMC_assert(same(owner->m_error.attitude_b.m_storage.values.head, values[field::attitude_error_x]),
                   "Output complete owner frame: m_error.attitude_b.m_storage.values.head");
    __ESBMC_assert(same(owner->m_error.attitude_b.m_storage.values.tail.head, values[field::attitude_error_y]),
                   "Output complete owner frame: m_error.attitude_b.m_storage.values.tail.head");
    __ESBMC_assert(same(owner->m_error.attitude_b.m_storage.values.tail.tail.head, values[field::attitude_error_z]),
                   "Output complete owner frame: m_error.attitude_b.m_storage.values.tail.tail.head");
    __ESBMC_assert(same(owner->m_error.velocity_n.m_storage.values.head, values[field::velocity_error_x]),
                   "Output complete owner frame: m_error.velocity_n.m_storage.values.head");
    __ESBMC_assert(same(owner->m_error.velocity_n.m_storage.values.tail.head, values[field::velocity_error_y]),
                   "Output complete owner frame: m_error.velocity_n.m_storage.values.tail.head");
    __ESBMC_assert(same(owner->m_error.velocity_n.m_storage.values.tail.tail.head, values[field::velocity_error_z]),
                   "Output complete owner frame: m_error.velocity_n.m_storage.values.tail.tail.head");
    __ESBMC_assert(same(owner->m_error.position_n.m_storage.values.head, values[field::position_error_x]),
                   "Output complete owner frame: m_error.position_n.m_storage.values.head");
    __ESBMC_assert(same(owner->m_error.position_n.m_storage.values.tail.head, values[field::position_error_y]),
                   "Output complete owner frame: m_error.position_n.m_storage.values.tail.head");
    __ESBMC_assert(same(owner->m_error.position_n.m_storage.values.tail.tail.head, values[field::position_error_z]),
                   "Output complete owner frame: m_error.position_n.m_storage.values.tail.tail.head");
    __ESBMC_assert(same(owner->m_filter_interval_s, values[field::interval_mean]),
                   "Output complete owner frame: m_filter_interval_s");
    __ESBMC_assert(owner->m_tracking == tracking && owner->m_fault == fault, "Output complete tracking/fault frame");
    for (std::size_t i = FORMAL_ESKF_PROOF_FRAME_FIRST; i < FORMAL_ESKF_PROOF_FRAME_END; ++i)
    {
        __ESBMC_assert(owner->m_history.m_samples[i].time_us == history_integers[i][0],
                       "Output complete history frame: time_us");
        __ESBMC_assert(owner->m_history.m_samples[i].start_time_us == history_integers[i][1],
                       "Output complete history frame: start_time_us");
        __ESBMC_assert(owner->m_history.m_samples[i].angular_period_us == history_integers[i][2],
                       "Output complete history frame: angular_period_us");
        __ESBMC_assert(same(owner->m_history.m_samples[i].q_nb.q0(), history_values[i][0]),
                       "Output complete history frame: q_nb.q0()");
        __ESBMC_assert(same(owner->m_history.m_samples[i].q_nb.q1(), history_values[i][1]),
                       "Output complete history frame: q_nb.q1()");
        __ESBMC_assert(same(owner->m_history.m_samples[i].q_nb.q2(), history_values[i][2]),
                       "Output complete history frame: q_nb.q2()");
        __ESBMC_assert(same(owner->m_history.m_samples[i].q_nb.q3(), history_values[i][3]),
                       "Output complete history frame: q_nb.q3()");
        __ESBMC_assert(same(owner->m_history.m_samples[i].v_n.m_storage.values.head, history_values[i][4]),
                       "Output complete history frame: v_n.m_storage.values.head");
        __ESBMC_assert(same(owner->m_history.m_samples[i].v_n.m_storage.values.tail.head, history_values[i][5]),
                       "Output complete history frame: v_n.m_storage.values.tail.head");
        __ESBMC_assert(same(owner->m_history.m_samples[i].v_n.m_storage.values.tail.tail.head, history_values[i][6]),
                       "Output complete history frame: v_n.m_storage.values.tail.tail.head");
        __ESBMC_assert(same(owner->m_history.m_samples[i].p_n.m_storage.values.head, history_values[i][7]),
                       "Output complete history frame: p_n.m_storage.values.head");
        __ESBMC_assert(same(owner->m_history.m_samples[i].p_n.m_storage.values.tail.head, history_values[i][8]),
                       "Output complete history frame: p_n.m_storage.values.tail.head");
        __ESBMC_assert(same(owner->m_history.m_samples[i].p_n.m_storage.values.tail.tail.head, history_values[i][9]),
                       "Output complete history frame: p_n.m_storage.values.tail.tail.head");
        __ESBMC_assert(owner->m_staged[i].time_us == staged_integers[i][0], "Output complete staged frame: time_us");
        __ESBMC_assert(owner->m_staged[i].start_time_us == staged_integers[i][1],
                       "Output complete staged frame: start_time_us");
        __ESBMC_assert(owner->m_staged[i].angular_period_us == staged_integers[i][2],
                       "Output complete staged frame: angular_period_us");
        __ESBMC_assert(same(owner->m_staged[i].q_nb.q0(), staged_values[i][0]),
                       "Output complete staged frame: q_nb.q0()");
        __ESBMC_assert(same(owner->m_staged[i].q_nb.q1(), staged_values[i][1]),
                       "Output complete staged frame: q_nb.q1()");
        __ESBMC_assert(same(owner->m_staged[i].q_nb.q2(), staged_values[i][2]),
                       "Output complete staged frame: q_nb.q2()");
        __ESBMC_assert(same(owner->m_staged[i].q_nb.q3(), staged_values[i][3]),
                       "Output complete staged frame: q_nb.q3()");
        __ESBMC_assert(same(owner->m_staged[i].v_n.m_storage.values.head, staged_values[i][4]),
                       "Output complete staged frame: v_n.m_storage.values.head");
        __ESBMC_assert(same(owner->m_staged[i].v_n.m_storage.values.tail.head, staged_values[i][5]),
                       "Output complete staged frame: v_n.m_storage.values.tail.head");
        __ESBMC_assert(same(owner->m_staged[i].v_n.m_storage.values.tail.tail.head, staged_values[i][6]),
                       "Output complete staged frame: v_n.m_storage.values.tail.tail.head");
        __ESBMC_assert(same(owner->m_staged[i].p_n.m_storage.values.head, staged_values[i][7]),
                       "Output complete staged frame: p_n.m_storage.values.head");
        __ESBMC_assert(same(owner->m_staged[i].p_n.m_storage.values.tail.head, staged_values[i][8]),
                       "Output complete staged frame: p_n.m_storage.values.tail.head");
        __ESBMC_assert(same(owner->m_staged[i].p_n.m_storage.values.tail.tail.head, staged_values[i][9]),
                       "Output complete staged frame: p_n.m_storage.values.tail.tail.head");
    }
}
} // namespace output_frames
