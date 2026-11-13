/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define main observation_unused_main
#if FORMAL_ESKF_PROOF_OBSERVATION_CONTRACT
#include "../../formal/cpp/esbmc/observations.cpp"
#else
#include "../../formal/cpp/esbmc/observation_models.cpp"
#endif
#undef main

#include <array>
#include <cstdio>
#include <cstdlib>

enum class TestCase
{
    baseline,
    nan_measurement,
    infinite_reference,
    zero_reference,
    zero_measurement,
    nan_rotation,
    max_finite_inputs,
    negative_zero,
    accelerometer_bias_overflow,
};

void __ESBMC_assert(bool condition, char const * description)
{
    if (!condition)
    {
        std::fprintf(stderr, "%s\n", description);
        std::abort();
    }
}
void __ESBMC_assume(bool condition) { __ESBMC_assert(condition, "unexpected infeasible native witness"); }
float nondet_float() { return 0.0F; }

// Deliberately distinct coordinates, off-diagonals and summary outputs check
// oracle sensitivity. Summary success is NOT a numerical correction witness.
int main()
{
    using namespace observation_proof;
    constexpr std::array test_cases{
        TestCase::baseline,          TestCase::nan_measurement,  TestCase::infinite_reference,
        TestCase::zero_reference,    TestCase::zero_measurement, TestCase::nan_rotation,
        TestCase::max_finite_inputs, TestCase::negative_zero,    TestCase::accelerometer_bias_overflow,
    };
    for (auto const test_case : test_cases)
    {
        state_type state;
        vector_type reference;
        rotation_type R;
        residual_type measurement;
        for (std::size_t i = 0U; i < 3U; ++i)
        {
            reference(i) = static_cast<value_type>(i + 1U);
            for (std::size_t j = 0U; j < 3U; ++j)
            {
                R(i, j) = static_cast<value_type>(i * 3U + j + 1U) / value_type{8};
            }
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
            state.p_n(i) = static_cast<value_type>(i + 3U);
            state.v_n(i) = static_cast<value_type>(i + 4U);
            state.b_a(i) = static_cast<value_type>(i + 5U);
            state.b_g(i) = static_cast<value_type>(i + 6U);
#endif
        }
        for (std::size_t i = 0U; i < measurement_size; ++i)
        {
            measurement(i) = static_cast<value_type>(i + 7U);
        }
        switch (test_case)
        {
        case TestCase::baseline:
            break;
        case TestCase::nan_measurement:
            measurement(measurement_size - 1U) = std::numeric_limits<value_type>::quiet_NaN();
            break;
        case TestCase::infinite_reference:
            reference(2U) = std::numeric_limits<value_type>::infinity();
            break;
        case TestCase::zero_reference:
            reference = vector_type{};
            break;
        case TestCase::zero_measurement:
            measurement = residual_type{};
            break;
        case TestCase::nan_rotation:
            R(2U, 2U) = std::numeric_limits<value_type>::quiet_NaN();
            break;
        case TestCase::max_finite_inputs:
            reference(2U) = std::numeric_limits<value_type>::max();
            R(2U, 2U) = std::numeric_limits<value_type>::max();
            break;
        case TestCase::negative_zero:
            R(2U, 1U) = -value_type{0};
            measurement(0U) = -value_type{0};
            break;
        case TestCase::accelerometer_bias_overflow:
#if FORMAL_ESKF_PROOF_OBSERVATION == 7 && FORMAL_ESKF_PROOF_STATE_SIZE == 15
            state.b_a(0U) = -std::numeric_limits<value_type>::max();
            measurement(0U) = std::numeric_limits<value_type>::max();
#endif
            break;
        }
#if FORMAL_ESKF_PROOF_OBSERVATION_CONTRACT
        covariance_type covariance, covariance_output;
        noise_type V;
        state_type state_output;
        state_output.q_nb = -state.q_nb;
        Result result;
        result.state = state_output;
        for (std::size_t i = 0U; i < state_size; ++i)
        {
            for (std::size_t j = 0U; j < state_size; ++j)
            {
                covariance(i, j) = static_cast<value_type>(i * state_size + j + 1U);
                covariance_output(i, j) = value_type{-2};
                result.covariance(i, j) = value_type{-3};
            }
        }
        for (std::size_t i = 0U; i < measurement_size; ++i)
        {
            for (std::size_t j = 0U; j < measurement_size; ++j)
            {
                V(i, j) = static_cast<value_type>(i * measurement_size + j + 1U);
            }
        }
        result.covariance(state_size - 1U, state_size - 1U) = std::numeric_limits<value_type>::quiet_NaN();
        constexpr std::array statuses{
            Status::success,
            Status::non_finite_input,
            Status::domain_error,
            Status::out_of_range,
            Status::zero_or_unsafe_divisor,
            Status::invalid_quaternion_norm,
            Status::not_positive_definite,
            Status::ill_conditioned,
            Status::non_finite_result,
        };
        for (auto const status : statuses)
        {
            result.status = status;
            verify_observation(state, covariance, measurement, V, reference, value_type{0.125}, state_output,
                               covariance_output, result, R);
        }
#else
#if FORMAL_ESKF_TEST_PRODUCER == 1
        verify_observation_rotation(state.q_nb);
#else
        verify_observation_jacobian(state, reference, R);
#endif
#endif
    }
    std::puts("E-OBS native oracle/flow witnesses: pass");
}
