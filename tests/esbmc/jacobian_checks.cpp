/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "../../formal/cpp/esbmc/right_jacobian.cpp"

#ifndef FORMAL_ESKF_PROOF_CHECK_FAULT
#define FORMAL_ESKF_PROOF_CHECK_FAULT 0
#endif

// Verifier regression, not additional domain coverage. Exercise the same
// caller and nine-entry comparison path with one deliberately wrong entry.
void verify_jacobian_checks()
{
    Vector3 phi;
    phi(1U) = Scalar{0.03125};
    Matrix3 output;
    verify_right_jacobian(phi, output, Scalar{0.0009765625}, Scalar{0}, Scalar{0}, Scalar{0}, true);
    auto const actual = FiniteContract::received;
    auto expected = actual;
#if FORMAL_ESKF_PROOF_CHECK_FAULT
    expected(0U, 2U) = expected(0U, 2U) + Scalar{0.25};
#endif
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            __ESBMC_assert(same(actual(row, column), expected(row, column)),
                           "Regression: a changed Jacobian entry must be rejected");
        }
    }
}
