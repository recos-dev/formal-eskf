/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "solve_support.hpp"

using namespace solve_proof;

void verify_cholesky_arithmetic(Scalar a, Scalar b, Scalar c)
{
    __ESBMC_assert(!FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT, "Runner error: arithmetic producer must be actual");
    __ESBMC_assert(same(Kernel::subtract_product(a, b, c), a - b * c),
                   "F-SOLVE: actual ordered multiply then subtract");
    __ESBMC_assert(same(Kernel::quotient(a, b), a / b), "F-SOLVE: actual IEEE quotient");
}

int main() { return 0; }
