// SPDX-License-Identifier: BSD-3-Clause

#include "frames.hpp"
namespace
{
output_proof::State expected_state;
bool validation_result;
unsigned validation_calls;
void check_input(output_proof::State const & state)
{
    __ESBMC_assert(imu_proof::same(state.q_nb.q0(), expected_state.q_nb.q0()),
                   "Seed preserves/forwards input: q_nb.q0()");
    __ESBMC_assert(imu_proof::same(state.q_nb.q1(), expected_state.q_nb.q1()),
                   "Seed preserves/forwards input: q_nb.q1()");
    __ESBMC_assert(imu_proof::same(state.q_nb.q2(), expected_state.q_nb.q2()),
                   "Seed preserves/forwards input: q_nb.q2()");
    __ESBMC_assert(imu_proof::same(state.q_nb.q3(), expected_state.q_nb.q3()),
                   "Seed preserves/forwards input: q_nb.q3()");
    __ESBMC_assert(imu_proof::same(state.v_n.m_storage.values.head, expected_state.v_n.m_storage.values.head),
                   "Seed preserves/forwards input: v_n.m_storage.values.head");
    __ESBMC_assert(imu_proof::same(state.v_n.m_storage.values.tail.head, expected_state.v_n.m_storage.values.tail.head),
                   "Seed preserves/forwards input: v_n.m_storage.values.tail.head");
    __ESBMC_assert(
        imu_proof::same(state.v_n.m_storage.values.tail.tail.head, expected_state.v_n.m_storage.values.tail.tail.head),
        "Seed preserves/forwards input: v_n.m_storage.values.tail.tail.head");
    __ESBMC_assert(imu_proof::same(state.p_n.m_storage.values.head, expected_state.p_n.m_storage.values.head),
                   "Seed preserves/forwards input: p_n.m_storage.values.head");
    __ESBMC_assert(imu_proof::same(state.p_n.m_storage.values.tail.head, expected_state.p_n.m_storage.values.tail.head),
                   "Seed preserves/forwards input: p_n.m_storage.values.tail.head");
    __ESBMC_assert(
        imu_proof::same(state.p_n.m_storage.values.tail.tail.head, expected_state.p_n.m_storage.values.tail.tail.head),
        "Seed preserves/forwards input: p_n.m_storage.values.tail.tail.head");
    __ESBMC_assert(imu_proof::same(state.b_a.m_storage.values.head, expected_state.b_a.m_storage.values.head),
                   "Seed preserves/forwards input: b_a.m_storage.values.head");
    __ESBMC_assert(imu_proof::same(state.b_a.m_storage.values.tail.head, expected_state.b_a.m_storage.values.tail.head),
                   "Seed preserves/forwards input: b_a.m_storage.values.tail.head");
    __ESBMC_assert(
        imu_proof::same(state.b_a.m_storage.values.tail.tail.head, expected_state.b_a.m_storage.values.tail.tail.head),
        "Seed preserves/forwards input: b_a.m_storage.values.tail.tail.head");
    __ESBMC_assert(imu_proof::same(state.b_g.m_storage.values.head, expected_state.b_g.m_storage.values.head),
                   "Seed preserves/forwards input: b_g.m_storage.values.head");
    __ESBMC_assert(imu_proof::same(state.b_g.m_storage.values.tail.head, expected_state.b_g.m_storage.values.tail.head),
                   "Seed preserves/forwards input: b_g.m_storage.values.tail.head");
    __ESBMC_assert(
        imu_proof::same(state.b_g.m_storage.values.tail.tail.head, expected_state.b_g.m_storage.values.tail.tail.head),
        "Seed preserves/forwards input: b_g.m_storage.values.tail.tail.head");
}
} // namespace
template <> bool formal_eskf::px4::OutputPredictor<output_proof::Backend>::valid_state(state_type const & state)
{
    check_input(state);
    output_frames::check();
    ++validation_calls;
    return validation_result;
}
void verify_output_seed(output_proof::Output input_owner, output_proof::State state, std::uint64_t time, bool valid)
{
    using namespace output_frames;
    owner = &input_owner;
    expected_state = state;
    validation_result = valid;
    validation_calls = 0;
    capture();
    bool const checked = input_owner.m_history.m_length != 0 && input_owner.m_current.time_us == 0 &&
                         input_owner.m_fault == Output::Fault::none && time != 0;
    bool const expected = checked && valid;
    bool const accepted = input_owner.seed(state, time);
    __ESBMC_assert(accepted == expected && validation_calls == static_cast<unsigned>(checked),
                   "Seed exact startup/state guard and validation call count");
    if (expected)
    {
        integers[field::time] = time;
        integers[field::start_time] = time;
        integers[field::period] = 0;
        integers[field::fusion_time] = time;
        values[field::q0] = expected_state.q_nb.q0();
        values[field::q1] = expected_state.q_nb.q1();
        values[field::q2] = expected_state.q_nb.q2();
        values[field::q3] = expected_state.q_nb.q3();
        values[field::velocity_x] = expected_state.v_n.m_storage.values.head;
        values[field::velocity_y] = expected_state.v_n.m_storage.values.tail.head;
        values[field::velocity_z] = expected_state.v_n.m_storage.values.tail.tail.head;
        values[field::position_x] = expected_state.p_n.m_storage.values.head;
        values[field::position_y] = expected_state.p_n.m_storage.values.tail.head;
        values[field::position_z] = expected_state.p_n.m_storage.values.tail.tail.head;
        values[field::accel_bias_x] = expected_state.b_a.m_storage.values.head;
        values[field::accel_bias_y] = expected_state.b_a.m_storage.values.tail.head;
        values[field::accel_bias_z] = expected_state.b_a.m_storage.values.tail.tail.head;
        values[field::gyro_bias_x] = expected_state.b_g.m_storage.values.head;
        values[field::gyro_bias_y] = expected_state.b_g.m_storage.values.tail.head;
        values[field::gyro_bias_z] = expected_state.b_g.m_storage.values.tail.tail.head;
    }
    check();
    check_input(state);
}
