/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "../../formal/cpp/esbmc/reset.cpp"

#ifndef FORMAL_ESKF_TEST_FAULT
#define FORMAL_ESKF_TEST_FAULT 0
#endif

// Verifier regressions, not extra formal-coverage profiles. In particular,
// scalar-by-scalar covariance copies must preserve non-finite classifications
// and signed zero, and a changed LAST cell must not be silently ignored.
void verify_reset_regression()
{
    __ESBMC_assert(ESKF_RESET_APPROX == 0, "E-RESET regression: the default is the closed-form right Jacobian");
    Matrix source;
    source.set(0U, 0U, -Scalar{0});
    source.set(0U, 1U, std::numeric_limits<Scalar>::quiet_NaN());
    source.set(0U, 2U, std::numeric_limits<Scalar>::infinity());
    source.set(1U, 0U, -std::numeric_limits<Scalar>::infinity());
    source.set(1U, 1U, std::numeric_limits<Scalar>::denorm_min());
    source.set(size - 1U, size - 1U, std::numeric_limits<Scalar>::max());
    auto const constructed = source;
    Matrix assigned;
    assigned = constructed;
#if FORMAL_ESKF_TEST_FAULT
    assigned.set(size - 1U, size - 1U, Scalar{0});
#endif
    __ESBMC_assert(same_matrix(constructed, source) && same_matrix(assigned, source),
                   "E-RESET regression: copied IEEE values and the last covariance cell are checked");
}

// Check the checker: a change at ANY valid coordinate must be observable.
// This guards against an oracle accidentally omitting an entire row/field.
void verify_reset_comparison(std::size_t row, std::size_t column)
{
    __ESBMC_assume(row < size && column < size);
    Matrix original;
    auto changed = original;
    changed.set(row, column, Scalar{1});
    __ESBMC_assert(!same_matrix(changed, original), "E-RESET regression: comparison observes every covariance cell");
}
