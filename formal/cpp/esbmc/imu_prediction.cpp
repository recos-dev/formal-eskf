/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "covariance_oracle.hpp"
#include <formal_eskf/eskf/eskf.hpp>
#include "../../../integrations/px4/module/ImuPrediction.hpp"

using contract_proof::Linalg;
using contract_proof::same;
using contract_proof::Scalar;
using contract_proof::Status;
using covariance_oracle::same_matrix;
using State = formal_eskf::configuration::Ins::NominalState<Linalg>;
using Parameters = formal_eskf::configuration::Ins::Parameters<Linalg>;
using Noise = formal_eskf::configuration::Ins::ProcessNoise<Linalg>;
using Matrix = formal_eskf::linalg::Matrix<Linalg, 15U, 15U>;
using Imu = formal_eskf::ImuSample<Linalg>;
#ifndef FORMAL_ESKF_PROOF_IMU_PREDICTION_CALLER
#define FORMAL_ESKF_PROOF_IMU_PREDICTION_CALLER 0
#endif

bool same_state(State const & a, State const & b)
{
    return same_matrix(a.q_nb.coefficients(), b.q_nb.coefficients()) && same_matrix(a.p_n, b.p_n) &&
           same_matrix(a.v_n, b.v_n) && same_matrix(a.b_a, b.b_a) && same_matrix(a.b_g, b.b_g);
}
bool same_noise(Noise const & a, Noise const & b)
{
    return same_matrix(a.specific_force_variance, b.specific_force_variance) &&
           same_matrix(a.angular_rate_variance, b.angular_rate_variance) &&
           same_matrix(a.accelerometer_bias_random_walk_variance_density,
                       b.accelerometer_bias_random_walk_variance_density) &&
           same_matrix(a.gyroscope_bias_random_walk_variance_density, b.gyroscope_bias_random_walk_variance_density);
}
bool same_parameters(Parameters const & a, Parameters const & b)
{
    return same_noise(a.process_noise, b.process_noise) && same_matrix(a.gravity_n, b.gravity_n) &&
           same(a.minimum_quaternion_norm, b.minimum_quaternion_norm) && same(a.dt_min, b.dt_min) &&
           same(a.dt_max, b.dt_max) && same(a.quaternion_squared_norm_tolerance, b.quaternion_squared_norm_tolerance);
}
bool same_imu(Imu const & a, Imu const & b)
{
    return same_matrix(a.specific_force_b, b.specific_force_b) && same_matrix(a.angular_rate_b, b.angular_rate_b);
}
Status variance_status(Scalar value)
{
    return !std::isfinite(value) ? Status::non_finite_input
           : value < Scalar{0}   ? Status::domain_error
                                 : Status::success;
}
void first_failure(Status & status, Scalar value)
{
    if (status == Status::success)
    {
        status = variance_status(value);
    }
}
#if !FORMAL_ESKF_PROOF_IMU_PREDICTION_CALLER
void verify_prediction_noise(Noise noise, formal_eskf::runtime::ImuNoiseConditions conditions, Scalar fault,
                             Noise output)
{
    auto const before = noise;
    auto const old_output = output;
    Status expected_status = variance_status(fault);
    auto expected = noise;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        first_failure(expected_status, noise.specific_force_variance(axis));
        first_failure(expected_status, noise.angular_rate_variance(axis));
        first_failure(expected_status, noise.accelerometer_bias_random_walk_variance_density(axis));
        first_failure(expected_status, noise.gyroscope_bias_random_walk_variance_density(axis));
        if (conditions.bad_vertical_acceleration || conditions.accelerometer_clipped[axis])
        {
            expected.specific_force_variance(axis) = fault;
        }
    }
    auto const result = formal_eskf::runtime::select_imu_process_noise(noise, conditions, fault, output);
    __ESBMC_assert(result == expected_status && same_noise(noise, before) &&
                       (result == Status::success ? same_noise(output, expected) : same_noise(output, old_output)),
                   "IMU prediction producer: actual same-type selection, ordered validation and pure/atomic frame");
}
void verify_prediction_bias(Matrix covariance, Noise noise)
{
    auto const before = noise;
    auto const old_covariance = covariance;
    Status expected_status = Status::success;
    auto expected = noise;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        first_failure(expected_status, covariance(9U + axis, 9U + axis));
        first_failure(expected_status, covariance(12U + axis, 12U + axis));
        first_failure(expected_status, noise.specific_force_variance(axis));
        first_failure(expected_status, noise.angular_rate_variance(axis));
        first_failure(expected_status, noise.accelerometer_bias_random_walk_variance_density(axis));
        first_failure(expected_status, noise.gyroscope_bias_random_walk_variance_density(axis));
        if (covariance(9U + axis, 9U + axis) >= noise.specific_force_variance(axis))
        {
            expected.accelerometer_bias_random_walk_variance_density(axis) = Scalar{0};
        }
        if (covariance(12U + axis, 12U + axis) >= noise.angular_rate_variance(axis))
        {
            expected.gyroscope_bias_random_walk_variance_density(axis) = Scalar{0};
        }
    }
    auto const result = formal_eskf::runtime::inhibit_bias_process_noise(covariance, noise, noise);
    __ESBMC_assert(result == expected_status && same_matrix(covariance, old_covariance) &&
                       (result == Status::success ? same_noise(noise, expected) : same_noise(noise, before)),
                   "IMU prediction producer: actual same-type in-place inhibition and exact threshold/atomic frame");
}
#endif
struct Prediction
{
    inline static State before{}, candidate{};
    inline static Matrix covariance{}, candidate_covariance{};
    inline static Imu imu{};
    inline static Parameters parameters{};
    inline static Scalar dt{};
    inline static Status status{};
    inline static unsigned calls{}, selections{}, inhibitions{};
    inline static Noise selected{}, inhibited{};
    inline static Status selection_status{}, inhibition_status{};
    inline static std::uint8_t clipping{};
    inline static bool fault{};
};
#if FORMAL_ESKF_PROOF_IMU_PREDICTION_CALLER
namespace formal_eskf::runtime
{
template <>
inline Status select_imu_process_noise<Linalg>(Noise const & noise, ImuNoiseConditions const & conditions, Scalar fault,
                                               Noise & output)
{
    __ESBMC_assert(
        Prediction::selections++ == 0U && Prediction::inhibitions == 0U && Prediction::calls == 0U &&
            same_noise(noise, Prediction::parameters.process_noise) && same_noise(output, noise) && &noise != &output &&
            conditions.bad_vertical_acceleration == Prediction::fault &&
            same(fault, static_cast<Scalar>(px4::profile::imu::faulty_specific_force_variance)),
        "IMU prediction: selection starts from the complete configured noise and fixed squared fault variance");
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        __ESBMC_assert(conditions.accelerometer_clipped[axis] == (((Prediction::clipping >> axis) & 1U) != 0U),
                       "IMU prediction: decode each current clipping bit exactly");
    }
    if (Prediction::selection_status == Status::success)
    {
        output = Prediction::selected;
    }
    return Prediction::selection_status;
}
template <>
inline Status inhibit_bias_process_noise<Linalg>(Matrix const & covariance, Noise const & noise, Noise & output)
{
    __ESBMC_assert(Prediction::selections == 1U && Prediction::selection_status == Status::success &&
                       Prediction::inhibitions++ == 0U && Prediction::calls == 0U && &noise == &output &&
                       same_noise(noise, Prediction::selected) && same_matrix(covariance, Prediction::covariance),
                   "IMU prediction: bias noise uses the selected variance and unchanged prior covariance in-place");
    if (Prediction::inhibition_status == Status::success)
    {
        output = Prediction::inhibited;
    }
    return Prediction::inhibition_status;
}
} // namespace formal_eskf::runtime
namespace formal_eskf
{
// Same-type E-STEP producer: failure-atomic in-place state/P, pure IMU and
// parameters. No success, finite, PSD or particular numerical value assumed.
template <>
inline Status predict<Linalg>(State const & state, Matrix const & covariance, Imu const & imu, Scalar dt,
                              Parameters const & parameters, State & state_output, Matrix & covariance_output)
{
    auto expected_parameters = Prediction::parameters;
    expected_parameters.process_noise = Prediction::inhibited;
    __ESBMC_assert(
        Prediction::selections == 1U && Prediction::inhibitions == 1U &&
            Prediction::inhibition_status == Status::success && Prediction::calls++ == 0U && &state == &state_output &&
            &covariance == &covariance_output && same_state(state, Prediction::before) &&
            same_matrix(covariance, Prediction::covariance) && same_imu(imu, Prediction::imu) &&
            same(dt, Prediction::dt) && same_parameters(parameters, expected_parameters),
        "IMU prediction: forward the unchanged prior, IMU, period and selected parameters exactly once in-place");
    if (Prediction::status == Status::success)
    {
        state_output = Prediction::candidate;
        covariance_output = Prediction::candidate_covariance;
    }
    return Prediction::status;
}
} // namespace formal_eskf
#endif

void verify_imu_admission(std::uint8_t accel, std::uint8_t gyro, bool initialized)
{
    bool const expected = (accel & 0xf8U) == 0U && gyro == 0U && (initialized || accel == 0U);
    __ESBMC_assert(formal_eskf::px4::imu_clipping_acceptable(accel, gyro, initialized) == expected,
                   "IMU admission: gyro/unknown clipping always refused; accel clipping only after initialization");
}
#if FORMAL_ESKF_PROOF_IMU_PREDICTION_CALLER
void verify_imu_prediction(State state, Matrix covariance, Imu imu, Scalar dt, Parameters parameters,
                           std::uint8_t clipping, bool fault, Status selection_status, Status inhibition_status,
                           Status prediction_status, Noise selected, Noise inhibited, State candidate,
                           Matrix candidate_covariance)
{
    auto const old_state = state;
    auto const old_covariance = covariance;
    auto const old_imu = imu;
    auto const old_parameters = parameters;
    auto const expected_status = clipping > 7U                          ? Status::domain_error
                                 : selection_status != Status::success  ? selection_status
                                 : inhibition_status != Status::success ? inhibition_status
                                                                        : prediction_status;
    bool const selected_stage = clipping <= 7U;
    bool const inhibited_stage = selected_stage && selection_status == Status::success;
    bool const predicted_stage = inhibited_stage && inhibition_status == Status::success;
    Prediction::before = state;
    Prediction::covariance = covariance;
    Prediction::imu = imu;
    Prediction::parameters = parameters;
    Prediction::dt = dt;
    Prediction::candidate = candidate;
    Prediction::candidate_covariance = candidate_covariance;
    Prediction::status = prediction_status;
    Prediction::selected = selected;
    Prediction::inhibited = inhibited;
    Prediction::selection_status = selection_status;
    Prediction::inhibition_status = inhibition_status;
    Prediction::clipping = clipping;
    Prediction::fault = fault;
    Prediction::calls = Prediction::selections = Prediction::inhibitions = 0U;
    auto const result = formal_eskf::px4::predict_imu(state, covariance, imu, dt, parameters, clipping, fault);
    __ESBMC_assert(result == expected_status && Prediction::selections == (selected_stage ? 1U : 0U) &&
                       Prediction::inhibitions == (inhibited_stage ? 1U : 0U) &&
                       Prediction::calls == (predicted_stage ? 1U : 0U),
                   "IMU prediction: first failure propagation and exact stage admission");
    __ESBMC_assert(result == Status::success
                       ? same_state(state, candidate) && same_matrix(covariance, candidate_covariance)
                       : same_state(state, old_state) && same_matrix(covariance, old_covariance),
                   "IMU prediction: all failure stages preserve both outputs; success commits the same prediction");
    __ESBMC_assert(same_imu(imu, old_imu) && same_parameters(parameters, old_parameters),
                   "IMU prediction: configured noise and borrowed inputs never latch step-specific overrides");
}
#endif
int main() { return 0; }
