/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "../../formal/cpp/esbmc/covariance_prediction.cpp"

#ifndef FORMAL_ESKF_TEST_FAULT
#define FORMAL_ESKF_TEST_FAULT 0
#endif

// Verifier/oracle regression, not an additional formal-coverage profile.
// E-RESET's shared-oracle regressions additionally check every valid coordinate.
void verify_pcov_regression()
{
    __ESBMC_assert(ESKF_QUAT_APPROX == 0, "E-PCOV regression: default prediction uses Exp");
    Results results{};
    results.propagated.set(size - 1U, size - 1U, std::numeric_limits<Scalar>::quiet_NaN());
    results.propagated.set(0U, size - 1U, std::numeric_limits<Scalar>::infinity());
    auto candidate = results.propagated;
    Calls::result = &results;
#if FORMAL_ESKF_TEST_FAULT
    candidate.set(size - 1U, size - 1U, Scalar{0});
#endif
    check_candidate(candidate);
}
