/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "correction_support.hpp"

#ifndef FORMAL_ESKF_PROOF_ROWS
#define FORMAL_ESKF_PROOF_ROWS 3
#endif
#ifndef FORMAL_ESKF_PROOF_INNER
#define FORMAL_ESKF_PROOF_INNER 3
#endif
#ifndef FORMAL_ESKF_PROOF_PRODUCT_COLUMNS
#define FORMAL_ESKF_PROOF_PRODUCT_COLUMNS 3
#endif

namespace correction_proof
{
constexpr std::size_t rows = FORMAL_ESKF_PROOF_ROWS;
constexpr std::size_t inner = FORMAL_ESKF_PROOF_INNER;
constexpr std::size_t columns = FORMAL_ESKF_PROOF_PRODUCT_COLUMNS;
using Left = Backend::matrix_type<rows, inner>;
using Right = Backend::matrix_type<inner, columns>;
using Product = Backend::matrix_type<rows, columns>;
using Square = Backend::matrix_type<rows, rows>;
using covariance_oracle::flatten;
struct FiniteCall
{
    inline static Square const * input = nullptr;
    inline static bool result{};
    inline static unsigned count{};
};
} // namespace correction_proof

#if FORMAL_ESKF_PROOF_FINITE_CONTRACT
namespace formal_eskf::linalg
{
template <>
inline bool all_finite<correction_proof::Backend, correction_proof::rows, correction_proof::rows>(
    correction_proof::Square const & input) noexcept
{
    using namespace correction_proof;
    __ESBMC_assert(FiniteCall::count++ == 0U && same_matrix(input, *FiniteCall::input),
                   "E-CORRECT finalizer: check every original candidate coefficient before mutation");
    return FiniteCall::result;
}
} // namespace formal_eskf::linalg
#endif

using namespace correction_proof;

void verify_correction_equality(Scalar a, Scalar b)
{
    __ESBMC_assert(same(a, b) == contract_proof::same(a, b),
                   "E-CORRECT oracle: representation comparison preserves value, signed zero and NaN classification");
}

void verify_correction_zero_difference(Scalar a, Scalar b)
{
    __ESBMC_assert(!std::isfinite(a) || !std::isfinite(b) || ((a - b == Scalar{0}) == (a == b)),
                   "E-CORRECT symmetry scalar lemma: finite IEEE difference is zero iff equal");
}

template <bool Subtract> void correction_elementwise(Square left, Square right)
{
    constexpr bool configured = !FORMAL_ESKF_PROOF_CORRECTION_CONTRACT && !FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT &&
                                !FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY && !FORMAL_ESKF_PROOF_FINITE_CONTRACT;
    __ESBMC_assert(configured, "Runner error: correction elementwise producer must be actual");
    if constexpr (!configured)
        return;
    auto const old_left = left;
    auto const old_right = right;
    Scalar a[rows * rows]{}, b[rows * rows]{}, c[rows * rows]{};
    flatten(old_left, a);
    flatten(old_right, b);
    auto const result = Subtract ? left - right : left + right;
    flatten(result, c);
    for (std::size_t i = 0U; i < rows * rows; ++i)
        __ESBMC_assert(same(c[i], Subtract ? a[i] - b[i] : a[i] + b[i]),
                       "E-CORRECT elementwise: every actual ordered IEEE add/subtract coefficient");
    __ESBMC_assert(same_matrix(left, old_left) && same_matrix(right, old_right),
                   "E-CORRECT elementwise: complete input frames");
}
void verify_correction_add(Square left, Square right) { correction_elementwise<false>(left, right); }
void verify_correction_subtract(Square left, Square right) { correction_elementwise<true>(left, right); }

void verify_correction_finite(Product candidate)
{
    constexpr bool configured = !FORMAL_ESKF_PROOF_CORRECTION_CONTRACT && !FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT &&
                                !FORMAL_ESKF_PROOF_FINITE_CONTRACT && !FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY;
    __ESBMC_assert(configured, "Runner error: correction finite predicate must be actual");
    if constexpr (!configured)
        return;
    auto const before = candidate;
    __ESBMC_assert(formal_eskf::linalg::all_finite(candidate) == finite_matrix(before),
                   "E-CORRECT finite: exactly every raw coefficient is finite");
    __ESBMC_assert(same_matrix(candidate, before), "E-CORRECT finite: complete input frame");
}

// Every position is checked, not a representative row/column. The actual entry
// and dot producers close the two boundaries without imposing numeric bounds.
void verify_correction_product(Left left, Right right, Product entries)
{
    constexpr bool configured = !FORMAL_ESKF_PROOF_CORRECTION_CONTRACT && !FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT &&
                                FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY == 1;
    __ESBMC_assert(configured, "Runner error: correction product requires only entry summaries");
    if constexpr (!configured)
        return;
    auto const old_left = left;
    auto const old_right = right;
    auto const left_storage = MatrixAccess::storage(old_left);
    auto const right_storage = MatrixAccess::storage(old_right);
    auto const entry_storage = MatrixAccess::storage(entries);
    using Call = EntryCalls;
    // Own snapshots, not addresses of a deduced-reference accessor return;
    // ESBMC materializes that return as a temporary. Check values at entry.
    Call::left = &left_storage;
    Call::right = &right_storage;
    Call::entries = &entry_storage;
    Call::received_left = nullptr;
    Call::received_right = nullptr;
    Call::count = 0U;
    auto const result = left * right;
    __ESBMC_assert(Call::count == rows * columns && same_matrix(result, entries),
                   "E-CORRECT product: publish all returned entries at their complete requested positions");
    __ESBMC_assert(same_matrix(left, old_left) && same_matrix(right, old_right),
                   "E-CORRECT product: both complete inputs survive");
}

void verify_correction_entry(Left left, Right right, std::size_t row, std::size_t column, Scalar dot)
{
    constexpr bool configured = !FORMAL_ESKF_PROOF_CORRECTION_CONTRACT && !FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT &&
                                FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY == 2;
    __ESBMC_assert(configured, "Runner error: correction entry requires only dot summary");
    if constexpr (!configured)
        return;
    __ESBMC_assume(row < rows && column < columns);
    auto const old_left = left;
    auto const old_right = right;
    DotCalls<inner>::count = 0U;
    DotCalls<inner>::result = dot;
    auto const result =
        product_entry<rows, inner, columns>(MatrixAccess::storage(left), MatrixAccess::storage(right), row, column);
    Scalar a[rows * inner]{}, b[inner * columns]{}, actual_left[inner]{}, actual_right[inner]{};
    flatten(old_left, a);
    flatten(old_right, b);
    flatten(DotCalls<inner>::left, actual_left);
    flatten(DotCalls<inner>::right, actual_right);
    for (std::size_t k = 0U; k < inner; ++k)
        __ESBMC_assert(same(actual_left[k], a[row * inner + k]) && same(actual_right[k], b[k * columns + column]),
                       "E-CORRECT entry: complete selected input row and column, in ascending inner-index order");
    __ESBMC_assert(DotCalls<inner>::count == 1U && same(result, dot), "E-CORRECT entry: return the sole dot result");
    __ESBMC_assert(same_matrix(left, old_left) && same_matrix(right, old_right),
                   "E-CORRECT entry: complete input frames");
}

using DotVector = Backend::vector_type<inner>;
void verify_correction_dot(DotVector left, DotVector right)
{
    constexpr bool configured = !FORMAL_ESKF_PROOF_CORRECTION_CONTRACT && !FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT &&
                                FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY == 0;
    __ESBMC_assert(configured, "Runner error: correction dot must be actual");
    if constexpr (!configured)
        return;
    auto const old_left = left;
    auto const old_right = right;
    Scalar a[inner]{}, b[inner]{};
    flatten(old_left, a);
    flatten(old_right, b);
    Scalar expected{0};
    for (std::size_t k = 0U; k < inner; ++k)
        expected += a[k] * b[k];
    auto const result = formal_eskf::linalg::dot(left, right);
    __ESBMC_assert(same(result, expected),
                   "E-CORRECT dot: actual ordered IEEE multiply/add, including non-finite operands");
    __ESBMC_assert(same_matrix(left, old_left) && same_matrix(right, old_right),
                   "E-CORRECT dot: complete input frames");
}

void correction_finish_case(Square candidate, bool finite)
{
    constexpr bool configured = !FORMAL_ESKF_PROOF_CORRECTION_CONTRACT && !FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT &&
                                FORMAL_ESKF_PROOF_FINITE_CONTRACT == 1 && !FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY;
    __ESBMC_assert(configured, "Runner error: correction finalizer must be actual");
    if constexpr (!configured)
        return;
    auto const before = candidate;
    FiniteCall::input = &before;
    FiniteCall::result = finite;
    FiniteCall::count = 0U;
    auto const status = formal_eskf::detail::try_finish_covariance(candidate, candidate);
    __ESBMC_assert(FiniteCall::count == 1U && status == (finite ? Status::success : Status::non_finite_result),
                   "E-CORRECT finalizer: status is exactly the complete candidate finite predicate");
    Scalar old[rows * rows]{}, result[rows * rows]{};
    flatten(before, old);
    flatten(candidate, result);
    for (std::size_t i = 0U; i < rows; ++i)
        for (std::size_t j = 0U; j < rows; ++j)
        {
            Scalar expected = old[i * rows + j];
            if (finite && i != j)
            {
                auto const lo = i < j ? i : j;
                auto const hi = i < j ? j : i;
                expected = Scalar{0.5} * old[lo * rows + hi] + Scalar{0.5} * old[hi * rows + lo];
            }
            __ESBMC_assert(same(result[i * rows + j], expected),
                           "E-CORRECT finalizer: every paired mean, diagonal and failure rollback");
        }
}

void verify_correction_finish(Square candidate)
{
    // Exhaust both predicate outcomes, never assume the successful one.
    // verify_correction_finite closes the sole boundary on this exact type.
    correction_finish_case(candidate, false);
    correction_finish_case(candidate, true);
}

void verify_correction_symmetry(Square matrix)
{
    constexpr bool configured = !FORMAL_ESKF_PROOF_CORRECTION_CONTRACT && !FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT &&
                                !FORMAL_ESKF_PROOF_FINITE_CONTRACT && !FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY;
    __ESBMC_assert(configured, "Runner error: correction symmetry producer must be actual");
    if constexpr (!configured)
        return;
    auto const before = matrix;
    Scalar input[rows * rows]{};
    flatten(before, input);
    bool symmetric = finite_matrix(before);
    for (std::size_t i = 0U; i < rows; ++i)
        for (std::size_t j = 0U; j < rows; ++j)
        {
            auto const a = input[i * rows + j];
            auto const b = input[j * rows + i];
            bool const zero_difference = !std::isfinite(a) || !std::isfinite(b) || ((a - b == Scalar{0}) == (a == b));
            // Universal scalar producer verify_correction_zero_difference is
            // mandatory for this exact format. Instantiate it per pair without
            // reproving the same IEEE lemma 225 times; no symmetry is assumed.
            __ESBMC_assume(zero_difference);
            symmetric = (a == b) && symmetric;
        }
    __ESBMC_assert(formal_eskf::linalg::is_symmetric(matrix, Scalar{0}) == symmetric,
                   "E-CORRECT symmetry: exact finite pairwise equality, not positive definiteness");
    __ESBMC_assert(same_matrix(matrix, before), "E-CORRECT symmetry: all input entries survive");
}

int main() { return 0; }
