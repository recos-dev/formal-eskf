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

#include <cstdio>
#include <cstdlib>

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
    for (unsigned test_case = 0U; test_case < 8U; ++test_case)
    {
        state_type state;
        vector_type reference, angular_rate;
        rotation_type R;
        ins_jacobian_type terms;
        terms.set(2U, 14U, value_type{-7});
        residual_type measurement;
        for (std::size_t i = 0U; i < 3U; ++i)
        {
            reference.set(i, static_cast<value_type>(i + 1U));
            angular_rate.set(i, static_cast<value_type>(i + 2U));
            for (std::size_t j = 0U; j < 3U; ++j)
            {
                R.set(i, j, static_cast<value_type>(i * 3U + j + 1U) / value_type{8});
            }
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
            state.p_n.set(i, static_cast<value_type>(i + 3U));
            state.v_n.set(i, static_cast<value_type>(i + 4U));
            state.b_a.set(i, static_cast<value_type>(i + 5U));
            state.b_g.set(i, static_cast<value_type>(i + 6U));
#endif
        }
        for (std::size_t i = 0U; i < measurement_size; ++i)
        {
            measurement.set(i, static_cast<value_type>(i + 7U));
        }
        switch (test_case)
        {
        case 1U:
            measurement.set(measurement_size - 1U, std::numeric_limits<value_type>::quiet_NaN());
            break;
        case 2U:
            reference.set(2U, std::numeric_limits<value_type>::infinity());
            break;
        case 3U:
            reference = vector_type{};
            break;
        case 4U:
            measurement = residual_type{};
            break;
        case 5U:
            R.set(2U, 2U, std::numeric_limits<value_type>::quiet_NaN());
            break;
        case 6U:
            reference.set(2U, std::numeric_limits<value_type>::max());
            R.set(2U, 2U, std::numeric_limits<value_type>::max());
            break;
        case 7U:
            R.set(2U, 1U, -value_type{0});
            measurement.set(0U, -value_type{0});
            break;
        default:
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
                covariance.set(i, j, static_cast<value_type>(i * state_size + j + 1U));
                covariance_output.set(i, j, value_type{-2});
                result.covariance.set(i, j, value_type{-3});
            }
        }
        for (std::size_t i = 0U; i < measurement_size; ++i)
        {
            for (std::size_t j = 0U; j < measurement_size; ++j)
            {
                V.set(i, j, static_cast<value_type>(i * measurement_size + j + 1U));
            }
        }
        result.covariance.set(state_size - 1U, state_size - 1U, std::numeric_limits<value_type>::quiet_NaN());
        for (unsigned status = 0U; status < 9U; ++status)
        {
            result.status = static_cast<Status>(status);
            verify_observation(state, covariance, measurement, V, reference, angular_rate, value_type{0.125},
                               state_output, covariance_output, result, R, terms);
        }
#else
#if FORMAL_ESKF_TEST_PRODUCER == 1
        verify_observation_rotation(state.q_nb);
#elif FORMAL_ESKF_TEST_PRODUCER == 2
        verify_observation_terms(R, reference, angular_rate, reference);
#else
        verify_observation_jacobian(state, reference, angular_rate, R, terms);
#endif
#endif
    }
    std::puts("E-OBS native oracle/flow witnesses: pass");
}
