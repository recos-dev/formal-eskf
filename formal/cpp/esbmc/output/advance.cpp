// SPDX-License-Identifier: BSD-3-Clause

#include "frames.hpp"
namespace
{
using namespace output_proof;
State returned_state;
formal_eskf::Status returned_status;
Sample expected_sample;
formal_eskf::ImuSample<Backend> expected_imu;
Scalar expected_dt;
unsigned calls;
} // namespace
namespace formal_eskf
{
template <>
Status predict_nominal(output_proof::State const & state, ImuSample<output_proof::Backend> const & imu,
                       output_proof::Scalar dt,
                       configuration::Ins::Parameters<output_proof::Backend> const & parameters,
                       output_proof::State & output)
{
    using namespace output_frames;
    check();
    __ESBMC_assert(&state == &output && calls == 0, "Output advance calls in-place nominal prediction exactly once");
    ++calls;
    auto const q = state.q_nb.coefficients();
    auto const previous_q = expected_sample.q_nb.coefficients();
    for (std::size_t i = 0; i < 4; ++i)
    {
        __ESBMC_assert(same(q(i), previous_q(i)), "Output advance forwards complete prior attitude");
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
        __ESBMC_assert(same(state.v_n(i), expected_sample.v_n(i)) && same(state.p_n(i), expected_sample.p_n(i)) &&
                           same(state.b_a(i), owner->m_accel_bias(i)) && same(state.b_g(i), owner->m_gyro_bias(i)),
                       "Output advance forwards prior translation and current learned biases");
        __ESBMC_assert(
            same(imu.specific_force_b(i), expected_imu.specific_force_b(i)) &&
                same(imu.angular_rate_b(i), expected_imu.angular_rate_b(i) + owner->m_angular_rate_correction(i)),
            "Output advance forwards force and feedback-corrected angular rate");
        __ESBMC_assert(same(parameters.gravity_n(i), i == 2 ? static_cast<Scalar>(profile::gravity_m_s2) : Scalar{0}),
                       "Output advance forwards configured gravity vector");
    }
    __ESBMC_assert(
        same(dt, expected_dt) &&
            same(parameters.minimum_quaternion_norm, static_cast<Scalar>(profile::minimum_quaternion_norm)) &&
            same(parameters.quaternion_squared_norm_tolerance,
                 static_cast<Scalar>(profile::quaternion_squared_norm_tolerance)) &&
            same(parameters.dt_min, static_cast<Scalar>(profile::imu::minimum_interval_s)) &&
            same(parameters.dt_max, static_cast<Scalar>(profile::imu::maximum_interval_s)),
        "Output advance forwards exact duration and numerical profile");
    if (returned_status == Status::success)
    {
        output = returned_state;
    }
    return returned_status;
}
} // namespace formal_eskf
void verify_output_advance(output_proof::Output input_owner, output_proof::Sample sample,
                           formal_eskf::ImuSample<output_proof::Backend> imu, output_proof::Scalar dt,
                           output_proof::State candidate, formal_eskf::Status status)
{
    using namespace output_frames;
    owner = &input_owner;
    capture();
    expected_sample = sample;
    expected_imu = imu;
    expected_dt = dt;
    returned_state = candidate;
    returned_status = status;
    calls = 0;
    bool const accepted = input_owner.advance(sample, imu, dt);
    __ESBMC_assert(accepted == (status == formal_eskf::Status::success) && calls == 1,
                   "Output advance propagates exact nominal status");
    auto const q = sample.q_nb.coefficients();
    auto const candidate_q = candidate.q_nb.coefficients();
    auto const prior_q = expected_sample.q_nb.coefficients();
    for (std::size_t i = 0; i < 4; ++i)
    {
        __ESBMC_assert(same(q(i), accepted ? candidate_q(i) : prior_q(i)),
                       "Output advance publishes full attitude or retains the prior");
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
        __ESBMC_assert(same(sample.v_n(i), accepted ? candidate.v_n(i) : expected_sample.v_n(i)) &&
                           same(sample.p_n(i), accepted ? candidate.p_n(i) : expected_sample.p_n(i)),
                       "Output advance publishes both translation vectors or retains both");
        __ESBMC_assert(same(imu.specific_force_b(i), expected_imu.specific_force_b(i)) &&
                           same(imu.angular_rate_b(i), expected_imu.angular_rate_b(i)),
                       "Output advance preserves the complete external IMU input");
    }
    __ESBMC_assert(sample.time_us == expected_sample.time_us && sample.start_time_us == expected_sample.start_time_us &&
                       sample.angular_period_us == expected_sample.angular_period_us && same(dt, expected_dt),
                   "Output advance preserves epochs, period and external duration");
    check();
}
