/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <limits>

#define main solve_harness_main
#include "../../formal/cpp/esbmc/solve.cpp"
#undef main

#ifndef FORMAL_ESKF_PROOF_CHECK_FAULT
#define FORMAL_ESKF_PROOF_CHECK_FAULT 0
#endif

// Reachability/solver regression, not the arbitrary-input proof. Exact dyadic
// roots make this independent 2x2 coupled example representable in both formats.
void verify_solve_regression()
{
    static_assert(size == 2U && columns == 3U);
    Factor system, factor;
    system.values[0U] = Scalar{4};
    system.values[1U] = Scalar{2};
    system.values[2U] = Scalar{2};
    system.values[3U] = Scalar{5};
    Math::sqrt = {};
    Math::sqrt.results.head = Scalar{2};
    Math::sqrt.results.tail.head = Scalar{2};
    auto status = Kernel::factorize<size>(system, factor);
    __ESBMC_assert(status == Status::success && Math::sqrt.calls == 2U && factor.values[0U] == Scalar{2} &&
                       factor.values[2U] == Scalar{1} && factor.values[3U] == Scalar{2},
                   "Regression: non-diagonal LLT success is reachable");
    Right rhs, output;
    for (std::size_t column = 0U; column < columns; ++column)
    {
        Scalar const j = static_cast<Scalar>(column);
        rhs.values[column] = Scalar{10} + Scalar{6} * j;
        rhs.values[columns + column] = Scalar{17} + Scalar{7} * j;
        status = Kernel::forward_substitute<size, columns>(factor, rhs, output, column);
        __ESBMC_assert(status == Status::success, "Regression: forward success is reachable");
        status = Kernel::backward_substitute<size, columns>(factor, output, column);
        __ESBMC_assert(status == Status::success, "Regression: backward success is reachable");
    }
#if FORMAL_ESKF_PROOF_CHECK_FAULT
    output.values[size * columns - 1U] += Scalar{1};
#endif
    for (std::size_t column = 0U; column < columns; ++column)
    {
        Scalar const j = static_cast<Scalar>(column);
        __ESBMC_assert(output.values[column] == Scalar{1} + j && output.values[columns + column] == Scalar{3} + j,
                       "Regression: all solution coefficients, including last-cell faults");
    }

    system.values[0U] = Scalar{0};
    Factor const before = factor;
    Math::sqrt.calls = 0U;
    status = Kernel::factorize<size>(system, factor);
    __ESBMC_assert(status == Status::not_positive_definite && Math::sqrt.calls == 0U &&
                       equal_cells(factor.values, before.values),
                   "Regression: zero pivot fails before root/write");
    rhs.values[0U] = std::numeric_limits<Scalar>::infinity();
    status = Kernel::forward_substitute<size, columns>(factor, rhs, output, 0U);
    __ESBMC_assert(status == Status::non_finite_result, "Regression: non-finite forward result rejected");
    status = Kernel::backward_substitute<size, columns>(factor, output, 0U);
    __ESBMC_assert(status == Status::non_finite_result, "Regression: non-finite backward result rejected");
}

int main() { return 0; }
