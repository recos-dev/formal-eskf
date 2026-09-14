/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "solve_support.hpp"

namespace solve_proof
{
// Assertions are checked before their use as cutpoints. The observer reads
// actual workspace but never modifies it. Each scalar primitive returns an
// unconstrained IEEE value; verify_cholesky_arithmetic proves its actual body.
// This checks every call argument, recurrence step, write and failure order,
// without expanding preceding floating-point arithmetic a second time.
void confirm(Scalar value, Scalar expected)
{
    bool const match = same(value, expected);
    __ESBMC_assert(match, "F-SOLVE: ordered IEEE coefficient operand/result");
    __ESBMC_assume(match);
}
struct Trace
{
    inline static unsigned operation{}, row{}, column{}, inner{};
    inline static bool output_phase{}, root_seen{}, quotient_seen{}, finished{};
    inline static Scalar residual{}, root_result{}, quotient_result{};
    inline static Status status{};
    inline static Factor const * system = nullptr;
    inline static Factor const * factor = nullptr;
    inline static Right const * rhs = nullptr;
    inline static Right const * workspace = nullptr;
    inline static Factor expected_factor{};
    inline static ScalarArray<Scalar, 3U> produced{};
    inline static unsigned written{};
    inline static Right const * initial = nullptr;

    // Recurrence operands come from independent input snapshots and previously
    // checked results, never from the mutable workspace being checked.
    static Scalar l(unsigned i, unsigned j)
    {
        return operation == 1U ? expected_factor.values[i * size + j] : factor->values[i * size + j];
    }
    static Scalar x(unsigned i)
    {
        __ESBMC_assert(i < size && (written & (1U << i)) != 0U,
                       "F-SOLVE: substitution consumes only previously computed rows");
        return produced[i];
    }

    static unsigned terms() { return operation == 1U ? column : operation == 2U ? row : size - row - 1U; }
    static Scalar initial_residual()
    {
        return operation == 1U   ? system->values[row * size + column]
               : operation == 2U ? rhs->values[row * columns + column]
                                 : initial->values[row * columns + column];
    }
    static Scalar accumulated() { return inner == 0U ? initial_residual() : residual; }
    static Scalar fresh();
    static Scalar subtract_product(Scalar value, Scalar left, Scalar right)
    {
        __ESBMC_assert(!finished && !output_phase && !quotient_seen && inner < terms(),
                       "F-SOLVE: exact multiply-subtract eligibility/order");
        confirm(value, accumulated());
        unsigned const index = operation == 3U ? row + 1U + inner : inner;
        confirm(left, operation == 3U ? l(index, row) : l(row, index));
        confirm(right, operation == 1U ? l(column, index) : x(index));
        residual = fresh();
        ++inner;
        return residual;
    }
    static Scalar quotient(Scalar value, Scalar divisor)
    {
        __ESBMC_assert(!finished && !quotient_seen && inner == terms() &&
                           (operation != 1U || (output_phase && row != column && !root_seen)),
                       "F-SOLVE: exact quotient eligibility/order");
        confirm(value, accumulated());
        confirm(divisor, l(operation == 1U ? column : row, operation == 1U ? column : row));
        quotient_seen = true;
        quotient_result = fresh();
        return quotient_result;
    }
    static bool inspect(Scalar value)
    {
        __ESBMC_assert(!finished, "F-SOLVE: no computation after failure/completion");
        __ESBMC_assume(!finished);
        __ESBMC_assert(row < size && column < (operation == 1U ? size : columns), "F-SOLVE: oracle stage in bounds");
        bool const finite_value = formal_eskf::scalar::StandardMath<Scalar>::is_finite(value);
        if (operation == 1U && !output_phase)
        {
            __ESBMC_assert(inner == terms() && !quotient_seen, "F-SOLVE: complete residual before inspection");
            confirm(value, accumulated());
            residual = value;
            if (!finite_value)
            {
                status = Status::non_finite_result;
                finished = true;
            }
            else if (row == column && value <= Scalar{0})
            {
                status = Status::not_positive_definite;
                finished = true;
            }
            else
            {
                output_phase = true;
                root_seen = false;
            }
            return finite_value;
        }
        if (operation == 1U)
        {
            __ESBMC_assert(root_seen == (row == column), "F-SOLVE: root only for diagonal");
            __ESBMC_assert(quotient_seen == (row != column), "F-SOLVE: quotient only off diagonal");
            confirm(value, row == column ? root_result : quotient_result);
            confirm(factor->values[row * size + column], value);
            expected_factor.values[row * size + column] = value;
            output_phase = false;
        }
        else
        {
            __ESBMC_assert(quotient_seen && inner == terms(), "F-SOLVE: complete substitution before inspection");
            confirm(value, quotient_result);
            confirm(workspace->values[row * columns + column], value);
            produced[row] = value;
            written |= 1U << row;
        }
        if (!finite_value)
        {
            status = Status::non_finite_result;
            finished = true;
        }
        else if (operation == 1U)
        {
            ++row;
            if (row == size)
            {
                ++column;
                row = column;
            }
            finished = column == size;
        }
        else if (operation == 2U)
        {
            ++row;
            finished = row == size;
        }
        else
        {
            if (row == 0U)
                finished = true;
            else
                --row;
        }
        inner = 0U;
        quotient_seen = false;
        return finite_value;
    }
};
extern double nondet_double();
Scalar Trace::fresh()
{
#if FORMAL_ESKF_PROOF_BINARY64
    return nondet_double();
#else
    return nondet_float();
#endif
}
#if FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT
bool Math::is_finite(Scalar value) { return Trace::inspect(value); }
void Math::record_root(Scalar input, Scalar result, unsigned index)
{
    __ESBMC_assert(Trace::operation == 1U && Trace::output_phase && !Trace::root_seen && !Trace::finished &&
                       Trace::row == Trace::column && index == Trace::column,
                   "F-SOLVE: exact root call eligibility and order");
    confirm(input, Trace::residual);
    Trace::root_seen = true;
    Trace::root_result = result;
}
#endif
} // namespace solve_proof
#if FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT
namespace formal_eskf::linalg
{
template <>
inline solve_proof::Scalar Cholesky<solve_proof::Backend>::subtract_product(solve_proof::Scalar residual,
                                                                            solve_proof::Scalar left,
                                                                            solve_proof::Scalar right) noexcept
{
    return solve_proof::Trace::subtract_product(residual, left, right);
}
template <>
inline solve_proof::Scalar Cholesky<solve_proof::Backend>::quotient(solve_proof::Scalar numerator,
                                                                    solve_proof::Scalar denominator) noexcept
{
    return solve_proof::Trace::quotient(numerator, denominator);
}
} // namespace formal_eskf::linalg
#endif
using namespace solve_proof;
void verify_factor_equations(Factor system, Factor initial, ScalarArray<Scalar, 3U> roots)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT == 1, "Runner error: equations require arithmetic contracts");
    Trace::status = Status::success;
    Trace::inner = 0U;
    Trace::output_phase = false;
    Trace::root_seen = false;
    Trace::quotient_seen = false;
    Trace::finished = false;
    Factor const before = system;
    Factor output = initial;
    Math::sqrt = {roots, 0U};
    Trace::operation = 1U;
    Trace::row = 0U;
    Trace::column = 0U;
    Trace::system = &before;
    Trace::factor = &output;
    Trace::expected_factor = initial;
    Status const status = Kernel::factorize<size>(system, output);
    __ESBMC_assert(Trace::finished && status == Trace::status,
                   "F-SOLVE: exact factor completion and failure precedence");
    __ESBMC_assert(equal_cells(output.values, Trace::expected_factor.values),
                   "F-SOLVE: factor output/partial-write frame");
    __ESBMC_assert(equal_cells(system.values, before.values), "F-SOLVE: factor input frame");
    if (status == Status::success)
    {
        __ESBMC_assert(Math::sqrt.calls == size, "F-SOLVE: success evaluates every diagonal root");
        for (std::size_t row = 0U; row < size; ++row)
        {
            for (std::size_t column = 0U; column <= row; ++column)
                __ESBMC_assert(std::isfinite(output.values[row * size + column]),
                               "F-SOLVE: successful lower triangle finite");
            __ESBMC_assert(same(output.values[row * size + row], roots[row]),
                           "F-SOLVE: diagonal publishes its root result");
        }
    }
}
void verify_forward_equations(Factor factor, Right rhs, Right initial, unsigned inspected_row,
                              unsigned inspected_column)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT == 1, "Runner error: equations require arithmetic contracts");
    Trace::status = Status::success;
    Trace::inner = 0U;
    Trace::output_phase = false;
    Trace::root_seen = false;
    Trace::quotient_seen = false;
    Trace::finished = false;
    constexpr unsigned column = FORMAL_ESKF_PROOF_COLUMN;
    static_assert(column < columns);
    __ESBMC_assume(inspected_row < size && inspected_column < columns);
    Factor const old_factor = factor;
    Right const old_rhs = rhs;
    Right output = initial;
    Trace::operation = 2U;
    Trace::row = 0U;
    Trace::column = column;
    Trace::factor = &old_factor;
    Trace::rhs = &old_rhs;
    Trace::workspace = &output;
    Trace::initial = &initial;
    Trace::written = 0U;
    Status const status = Kernel::forward_substitute<size, columns>(factor, rhs, output, column);
    __ESBMC_assert(Trace::finished && status == Trace::status,
                   "F-SOLVE: exact forward completion and failure precedence");
    std::size_t const cell = inspected_row * columns + inspected_column;
    Scalar const expected = inspected_column == column && (Trace::written & (1U << inspected_row)) != 0U
                                ? Trace::produced[inspected_row]
                                : initial.values[cell];
    __ESBMC_assert(same(output.values[cell], expected), "F-SOLVE: forward output/partial-write frame");
    __ESBMC_assert(status != Status::success || std::isfinite(output.values[inspected_row * columns + column]),
                   "F-SOLVE: successful forward column finite");
    __ESBMC_assert(equal_cells(factor.values, old_factor.values) && equal_cells(rhs.values, old_rhs.values),
                   "F-SOLVE: forward input frame");
}
void verify_backward_equations(Factor factor, Right initial, unsigned inspected_row, unsigned inspected_column)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT == 1, "Runner error: equations require arithmetic contracts");
    Trace::status = Status::success;
    Trace::inner = 0U;
    Trace::output_phase = false;
    Trace::root_seen = false;
    Trace::quotient_seen = false;
    Trace::finished = false;
    constexpr unsigned column = FORMAL_ESKF_PROOF_COLUMN;
    static_assert(column < columns);
    __ESBMC_assume(inspected_row < size && inspected_column < columns);
    Factor const old_factor = factor;
    Right output = initial;
    Trace::operation = 3U;
    Trace::row = size - 1U;
    Trace::column = column;
    Trace::factor = &old_factor;
    Trace::workspace = &output;
    Trace::initial = &initial;
    Trace::written = 0U;
    Status const status = Kernel::backward_substitute<size, columns>(factor, output, column);
    __ESBMC_assert(Trace::finished && status == Trace::status,
                   "F-SOLVE: exact backward completion and failure precedence");
    std::size_t const cell = inspected_row * columns + inspected_column;
    Scalar const expected = inspected_column == column && (Trace::written & (1U << inspected_row)) != 0U
                                ? Trace::produced[inspected_row]
                                : initial.values[cell];
    __ESBMC_assert(same(output.values[cell], expected), "F-SOLVE: backward output/partial-write frame");
    __ESBMC_assert(status != Status::success || std::isfinite(output.values[inspected_row * columns + column]),
                   "F-SOLVE: successful backward column finite");
    __ESBMC_assert(equal_cells(factor.values, old_factor.values), "F-SOLVE: backward input frame");
}
int main() { return 0; }
