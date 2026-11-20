// SPDX-License-Identifier: BSD-3-Clause

// Same-type output-owner proof backend; production bodies remain unchanged.
#pragma once
#include "../imu_owner_support.hpp"
#define private public
#include "../../../../integrations/px4/module/OutputPredictor.hpp"
#undef private

namespace output_proof
{
using imu_proof::Backend;
using imu_proof::same;
using imu_proof::same_configuration;
using imu_proof::Scalar;
using Output = formal_eskf::px4::OutputPredictor<Backend>;
using Sample = Output::Sample;
using State = Output::state_type;
using Vector = Output::vector3_type;
using Quaternion = Output::quaternion_type;
namespace profile = formal_eskf::px4::profile;
static_assert(profile::history::imu_capacity == 64);

void zero_vector(Vector const & vector)
{
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        __ESBMC_assert(same(vector(axis), Scalar{0}), "Empty output vector retains positive zero");
    }
}
void zero_sample(Sample const & sample)
{
    __ESBMC_assert(sample.time_us == 0 && sample.start_time_us == 0 && sample.angular_period_us == 0,
                   "Empty output sample has zero epochs and period");
    __ESBMC_assert(same(sample.q_nb.q0(), Scalar{1}) && same(sample.q_nb.q1(), Scalar{0}) &&
                       same(sample.q_nb.q2(), Scalar{0}) && same(sample.q_nb.q3(), Scalar{0}),
                   "Empty output sample has identity attitude");
    zero_vector(sample.v_n);
    zero_vector(sample.p_n);
}
void empty_owner(Output const & owner)
{
    __ESBMC_assert(owner.m_history.m_count == 0 && owner.m_history.m_oldest == 0 && owner.m_history.m_write == 0,
                   "Empty output history has zero cursors and count");
    zero_sample(owner.m_current);
    zero_vector(owner.m_accel_bias);
    zero_vector(owner.m_gyro_bias);
    zero_vector(owner.m_angular_rate_correction);
    zero_vector(owner.m_output_angular_rate_b);
    zero_vector(owner.m_velocity_error_sum);
    zero_vector(owner.m_position_error_sum);
    zero_vector(owner.m_error.attitude_b);
    zero_vector(owner.m_error.velocity_n);
    zero_vector(owner.m_error.position_n);
    __ESBMC_assert(same(owner.m_filter_interval_s, static_cast<Scalar>(profile::imu::nominal_interval_s)) &&
                       owner.m_fusion_time_us == 0 && !owner.m_tracking && owner.m_fault == Output::Fault::none,
                   "Empty output owner clears feedback time, readiness and fault");
    for (std::size_t i = 0; i < 64; ++i)
    {
        zero_sample(owner.m_history.m_samples[i]);
        zero_sample(owner.m_staged[i]);
    }
}
} // namespace output_proof
