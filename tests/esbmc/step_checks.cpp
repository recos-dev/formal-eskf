/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define main step_unused_main
#include "../../formal/cpp/esbmc/step.cpp"
#undef main

#include <cstdio>
#include <cstdlib>

// Concrete feasibility/oracle regression, not general-domain proof coverage.
template <unsigned Aliases> void step_fixture(Status nominal_status, Status covariance_status)
{
    State state{}, state_output{};
    Error error{}, error_output{};
    Matrix covariance{}, covariance_output{};
    Results results{};
    Imu imu{};
    Parameters parameters{};
    state_output.q_nb = -state.q_nb;
    OpaqueMath::prepare(Scalar{1}, Scalar{0}, Scalar{1}, Scalar{0});
    auto const initialization = Quaternion::try_from_coefficients(Scalar{0}, Scalar{1}, Scalar{0}, Scalar{0},
                                                                  Scalar{0.125}, results.nominal.q_nb);
    __ESBMC_assert(initialization == Status::success, "E-STEP regression: construct distinct nominal output");
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        error.delta_theta_b.set(i, Scalar{4});
        error_output.delta_theta_b.set(i, Scalar{5});
#if FORMAL_ESKF_PROOF_INS
        state.p_n.set(i, Scalar{1});
        state.v_n.set(i, Scalar{2});
        state.b_a.set(i, Scalar{3});
        state.b_g.set(i, Scalar{4});
        error.delta_p_n.set(i, Scalar{5});
        error.delta_v_n.set(i, Scalar{6});
        error.delta_b_a.set(i, Scalar{7});
        error.delta_b_g.set(i, Scalar{8});
        error_output = error;
        results.nominal.v_n.set(i, Scalar{-1});
#endif
    }
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            covariance.set(row, column, Scalar{1});
            covariance_output.set(row, column, Scalar{2});
            results.covariance.set(row, column, Scalar{3});
        }
    covariance.set(0U, 0U, std::numeric_limits<Scalar>::infinity());
    results.covariance.set(Matrix::row_count - 1U, Matrix::column_count - 1U, std::numeric_limits<Scalar>::quiet_NaN());
    results.nominal_status = nominal_status;
    results.covariance_status = covariance_status;
    prediction_transaction<(Aliases & 3U)>(state, covariance, imu, Scalar{0}, parameters, state_output,
                                           covariance_output, results);
    injection_reset_transaction<Aliases>(state, covariance, error, Scalar{0}, state_output, covariance_output,
                                         error_output, results);
}

void verify_step_regression(Status nominal_status, Status covariance_status)
{
    step_fixture<FORMAL_ESKF_PROOF_STEP_ALIAS>(nominal_status, covariance_status);
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
    unsigned count = 0U;
    for (unsigned nominal = 0U; nominal < 9U; ++nominal)
        for (unsigned covariance = 0U; covariance < 9U; ++covariance)
        {
            auto const a = static_cast<Status>(nominal);
            auto const b = static_cast<Status>(covariance);
            step_fixture<0U>(a, b);
            step_fixture<1U>(a, b);
            step_fixture<2U>(a, b);
            step_fixture<3U>(a, b);
            step_fixture<4U>(a, b);
            step_fixture<5U>(a, b);
            step_fixture<6U>(a, b);
            step_fixture<7U>(a, b);
            count += 8U;
        }
    std::printf("E-STEP native witnesses: pass (%u alias/status combinations, both transactions)\n", count);
}
#else
int main() { return 0; }
#endif
