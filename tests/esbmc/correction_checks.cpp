/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "../../formal/cpp/esbmc/correction_support.hpp"
#include <limits>

#ifndef FORMAL_ESKF_PROOF_CHECK_FAULT
#define FORMAL_ESKF_PROOF_CHECK_FAULT 0
#endif

using namespace correction_proof;

// Coupled, dyadic witness through all actual pre-injection code, including LLT.
// This is a reachability/negative regression, not the arbitrary-input proof.
void verify_correction_regression()
{
    static_assert(measurement_size == 1U && !FORMAL_ESKF_PROOF_CORRECTION_CONTRACT &&
                  !FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY);
    Covariance prior, output;
    Jacobian H;
    Noise V;
    Residual residual;
    Correction delta;
    for (std::size_t i = 0U; i < state_size; ++i)
        prior.set(i, i, Scalar{3});
    prior.set(0U, state_size - 1U, Scalar{1});
    prior.set(state_size - 1U, 0U, Scalar{1});
    H.set(0U, state_size - 1U, Scalar{1});
    V.set(0U, 0U, Scalar{1});
    residual.set(0U, Scalar{4});
    solve_proof::Math::sqrt = {};
    solve_proof::Math::sqrt.results.head = Scalar{2};
    auto const status = formal_eskf::detail::try_compute_correction(prior, residual, H, V, Scalar{0.5}, delta, output);
    __ESBMC_assert(status == Status::success && solve_proof::Math::sqrt.calls == 1U,
                   "Regression: coupled correction reaches actual solve and successful Joseph publication");
#if FORMAL_ESKF_PROOF_CHECK_FAULT == 1
    output.set(state_size - 1U, state_size - 1U, Scalar{1});
#elif FORMAL_ESKF_PROOF_CHECK_FAULT == 2
    delta.set(state_size - 1U, Scalar{1});
#endif
    auto const & corrected = output;
    auto const & correction = delta;
    for (std::size_t i = 0U; i < state_size; ++i)
    {
        Scalar const expected_delta = i == 0U ? Scalar{1} : i == state_size - 1U ? Scalar{3} : Scalar{0};
        __ESBMC_assert(same(correction(i), expected_delta),
                       "Regression: every correction coordinate, including the last");
        for (std::size_t j = 0U; j < state_size; ++j)
        {
            Scalar expected{0};
            if (i == j)
                expected = i == 0U ? Scalar{2.75} : i == state_size - 1U ? Scalar{0.75} : Scalar{3};
            else if ((i == 0U && j == state_size - 1U) || (j == 0U && i == state_size - 1U))
                expected = Scalar{0.25};
            __ESBMC_assert(same(corrected(i, j), expected),
                           "Regression: full correlated Joseph covariance, including last-cell faults");
        }
    }
    auto const saved_delta = delta;
    auto const saved_output = output;
    residual.set(0U, std::numeric_limits<Scalar>::infinity());
    solve_proof::Math::sqrt.calls = 0U;
    auto const rejected =
        formal_eskf::detail::try_compute_correction(prior, residual, H, V, Scalar{0.5}, delta, output);
    __ESBMC_assert(rejected == Status::non_finite_input && solve_proof::Math::sqrt.calls == 0U &&
                       same_matrix(delta, saved_delta) && same_matrix(output, saved_output),
                   "Regression: invalid residual fails before solve and preserves both outputs");
}

int main() { return 0; }
