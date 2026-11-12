/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define main step_correction_unused_main
#include "../../formal/cpp/esbmc/step_correction.cpp"
#undef main

#include <cstdio>
#include <cstdlib>

// Feasible, distinguishable data for control-flow/oracle regressions. The
// arbitrary success outputs are summary values, NOT numerical ESKF witnesses.
void verify_correction_step_regression(Status gain_status, Status correction_status, Status injection_status,
                                       Status reset_status)
{
    State state{}, state_output{};
    Covariance covariance{}, covariance_output{};
    Residual residual{};
    Jacobian H{};
    Noise V{};
    Results results{};
    state_output.q_nb = -state.q_nb;
    results.injected.q_nb = state_output.q_nb;
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        state.p_n(i) = Scalar{1};
        state.v_n(i) = Scalar{2};
        state.b_a(i) = Scalar{3};
        state.b_g(i) = Scalar{4};
        results.injected.p_n(i) = Scalar{-1};
        results.injected.v_n(i) = Scalar{-2};
        results.injected.b_a(i) = Scalar{-3};
        results.injected.b_g(i) = Scalar{-4};
    }
#endif
    for (std::size_t i = 0U; i < state_size; ++i)
    {
        results.delta(i) = static_cast<Scalar>(i + 1U);
        for (std::size_t j = 0U; j < state_size; ++j)
        {
            covariance(i, j) = Scalar{1};
            covariance_output(i, j) = Scalar{2};
            results.joseph(i, j) = Scalar{3};
            results.reset(i, j) = Scalar{4};
        }
    }
    results.delta(0U) = -Scalar{0};
    covariance(0U, 0U) = std::numeric_limits<Scalar>::infinity();
    results.reset(state_size - 1U, state_size - 1U) = std::numeric_limits<Scalar>::quiet_NaN();
    for (std::size_t row = 0U; row < state_size; ++row)
    {
        for (std::size_t column = 0U; column < measurement_size; ++column)
        {
            results.gain(row, column) = static_cast<Scalar>(row + column + 1U);
        }
    }
    results.gain(0U, 0U) = -Scalar{0};
    results.gain(state_size - 1U, measurement_size - 1U) = std::numeric_limits<Scalar>::quiet_NaN();
    results.gain_status = gain_status;
    results.correction_status = correction_status;
    results.injection_status = injection_status;
    results.reset_status = reset_status;
    verify_correction_step(state, covariance, residual, H, V, Scalar{0.125}, state_output, covariance_output, results);
}

#if FORMAL_ESKF_TEST_NATIVE
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

int main()
{
    for (unsigned gain = 0U; gain < 9U; ++gain)
        for (unsigned correction = 0U; correction < 9U; ++correction)
            for (unsigned injection = 0U; injection < 9U; ++injection)
                for (unsigned reset = 0U; reset < 9U; ++reset)
                    verify_correction_step_regression(static_cast<Status>(gain), static_cast<Status>(correction),
                                                      static_cast<Status>(injection), static_cast<Status>(reset));
    std::puts("E-STEP-CORRECT native witnesses: pass (6561 status combinations)");
}
#else
int main() { return 0; }
#endif
