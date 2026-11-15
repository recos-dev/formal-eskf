/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "step_correction_support.hpp"
#include <formal_eskf/runtime/imu.hpp>

using namespace step_correction_proof;
using ProcessNoise = formal_eskf::configuration::Ins::ProcessNoise<Backend>;
using Conditions = formal_eskf::runtime::ImuNoiseConditions;
using Limits = formal_eskf::runtime::BiasLimits<Scalar>;
static_assert(state_size == 15U && boundaries_clear && FORMAL_ESKF_PROOF_STEP_ALIAS >= 0 &&
              FORMAL_ESKF_PROOF_STEP_ALIAS <= 1);

bool same_noise(ProcessNoise const & a, ProcessNoise const & b)
{
    return same_matrix(a.specific_force_variance, b.specific_force_variance) &&
           same_matrix(a.angular_rate_variance, b.angular_rate_variance) &&
           same_matrix(a.accelerometer_bias_random_walk_variance_density,
                       b.accelerometer_bias_random_walk_variance_density) &&
           same_matrix(a.gyroscope_bias_random_walk_variance_density, b.gyroscope_bias_random_walk_variance_density);
}
Scalar noise_coefficient(ProcessNoise const & noise, std::size_t index)
{
    auto const axis = index / 4U;
    switch (index % 4U)
    {
    case 0U:
        return noise.specific_force_variance(axis);
    case 1U:
        return noise.angular_rate_variance(axis);
    case 2U:
        return noise.accelerometer_bias_random_walk_variance_density(axis);
    default:
        return noise.gyroscope_bias_random_walk_variance_density(axis);
    }
}
Status coefficient_status(Scalar value)
{
    return !std::isfinite(value) ? Status::non_finite_input
           : value < Scalar{0}   ? Status::domain_error
                                 : Status::success;
}
void verify_imu_noise(ProcessNoise noise, Conditions conditions, Scalar fault_variance, ProcessNoise output)
{
    auto const before = noise;
    auto const old_output = output;
    auto const old_conditions = conditions;
    Status expected_status = coefficient_status(fault_variance);
    // Classify every coefficient; retain the first failing field's status.
    for (std::size_t i = 0U; i < 12U; ++i)
    {
        Status const next = coefficient_status(noise_coefficient(noise, i));
        if (expected_status == Status::success)
        {
            expected_status = next;
        }
    }
    ProcessNoise expected = before;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        if (conditions.bad_vertical_acceleration || conditions.accelerometer_clipped[axis])
        {
            expected.specific_force_variance(axis) = fault_variance;
        }
    }
    ProcessNoise * target = &output;
    if constexpr (FORMAL_ESKF_PROOF_STEP_ALIAS == 1)
    {
        target = &noise;
    }
    auto const old_target = *target;
    auto const status = formal_eskf::runtime::select_imu_process_noise(noise, conditions, fault_variance, *target);
    __ESBMC_assert(status == expected_status &&
                       (status == Status::success ? same_noise(*target, expected) : same_noise(*target, old_target)),
                   "R-IMU-PROCESS-NOISE: exact first error, per-axis replacement "
                   "and atomic output");
    __ESBMC_assert(FORMAL_ESKF_PROOF_STEP_ALIAS ? same_noise(output, old_output) : same_noise(noise, before),
                   "R-IMU-PROCESS-NOISE: preserve distinct input or unused output");
    __ESBMC_assert(conditions.bad_vertical_acceleration == old_conditions.bad_vertical_acceleration &&
                       conditions.accelerometer_clipped == old_conditions.accelerometer_clipped,
                   "R-IMU-PROCESS-NOISE: selection never modifies the caller's conditions");
}
void verify_bias_noise(Covariance covariance, ProcessNoise noise, ProcessNoise output)
{
    auto const old_covariance = covariance;
    auto const before = noise;
    auto const old_output = output;
    Status expected_status = Status::success;
    for (std::size_t i = 0U; i < 18U; ++i)
    {
        auto const axis = i / 6U;
        auto const field = i % 6U;
        Scalar const value = field < 2U ? covariance(9U + field * 3U + axis, 9U + field * 3U + axis)
                                        : noise_coefficient(noise, axis * 4U + field - 2U);
        Status const next = coefficient_status(value);
        if (expected_status == Status::success)
        {
            expected_status = next;
        }
    }
    ProcessNoise expected = before;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        if (covariance(9U + axis, 9U + axis) >= before.specific_force_variance(axis))
        {
            expected.accelerometer_bias_random_walk_variance_density(axis) = Scalar{0};
        }
        if (covariance(12U + axis, 12U + axis) >= before.angular_rate_variance(axis))
        {
            expected.gyroscope_bias_random_walk_variance_density(axis) = Scalar{0};
        }
    }
    ProcessNoise * target = &output;
    if constexpr (FORMAL_ESKF_PROOF_STEP_ALIAS == 1)
    {
        target = &noise;
    }
    auto const old_target = *target;
    auto const status = formal_eskf::runtime::inhibit_bias_process_noise(covariance, noise, *target);
    __ESBMC_assert(status == expected_status &&
                       (status == Status::success ? same_noise(*target, expected) : same_noise(*target, old_target)),
                   "R-BIAS-PROCESS-NOISE: equality inhibits only bias "
                   "intensities, with exact first error and rollback");
    __ESBMC_assert(same_matrix(covariance, old_covariance) &&
                       (FORMAL_ESKF_PROOF_STEP_ALIAS ? same_noise(output, old_output) : same_noise(noise, before)),
                   "R-BIAS-PROCESS-NOISE: preserve covariance and distinct input "
                   "or unused output");
}
void verify_bias_limits(State state, Limits limits)
{
    auto const before = state;
    auto const old_limits = limits;
    bool const finite = std::isfinite(limits.accelerometer) && std::isfinite(limits.gyroscope) &&
                        correction_proof::finite_matrix(state.b_a) && correction_proof::finite_matrix(state.b_g);
    Status const expected_status = !finite ? Status::non_finite_input
                                   : limits.accelerometer < Scalar{0} || limits.gyroscope < Scalar{0}
                                       ? Status::domain_error
                                       : Status::success;
    auto const status = formal_eskf::runtime::limit_bias(state, limits);
    __ESBMC_assert(status == expected_status, "R-BIAS-LIMITS: validate every consumed value before changing any bias");
    if (status == Status::success)
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            Scalar const a = before.b_a(axis), g = before.b_g(axis);
            Scalar const expected_a = a < -limits.accelerometer  ? -limits.accelerometer
                                      : limits.accelerometer < a ? limits.accelerometer
                                                                 : a;
            Scalar const expected_g = g < -limits.gyroscope  ? -limits.gyroscope
                                      : limits.gyroscope < g ? limits.gyroscope
                                                             : g;
            __ESBMC_assert(same(state.b_a(axis), expected_a) && same(state.b_g(axis), expected_g),
                           "R-BIAS-LIMITS: symmetric coefficient bounds preserve "
                           "in-range representations");
            __ESBMC_assert(state.b_a(axis) >= -limits.accelerometer && state.b_a(axis) <= limits.accelerometer &&
                               state.b_g(axis) >= -limits.gyroscope && state.b_g(axis) <= limits.gyroscope,
                           "R-BIAS-LIMITS: every successful bias is finite and "
                           "within its bound");
        }
        __ESBMC_assert(same_matrix(state.q_nb.coefficients(), before.q_nb.coefficients()) &&
                           same_matrix(state.p_n, before.p_n) && same_matrix(state.v_n, before.v_n),
                       "R-BIAS-LIMITS: preserve all non-bias state, even when those unused "
                       "fields are nonfinite");
    }
    else
    {
        __ESBMC_assert(same_state(state, before), "R-BIAS-LIMITS: any failure preserves the complete state");
    }
    __ESBMC_assert(same(limits.accelerometer, old_limits.accelerometer) && same(limits.gyroscope, old_limits.gyroscope),
                   "R-BIAS-LIMITS: caller limits are unchanged");
}
int main() { return 0; }
