/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "linalg_support.hpp"

using namespace linalg_proof;
namespace la = formal_eskf::linalg;

void assert_stored_values(Matrix const & matrix, Scalar const (&expected)[count])
{
    Scalar observed[count]{};
    flatten(matrix, observed);
    for (std::size_t i = 0U; i < count; ++i)
        __ESBMC_assert(same(observed[i], expected[i]), "F-LINALG: every raw storage coefficient matches its snapshot");
}

void verify_linalg_storage(Matrix input, Matrix output)
{
    __ESBMC_assert(actual && storage_part <= 3U, "Runner error: linalg storage requires actual operations");
    if constexpr (!actual)
        return;
    Scalar before[count]{}, observed[count]{};
    flatten(input, before);
    // The flat 144-cell type uses three independent executions to avoid
    // repeatedly expanding recursive indexing in a single formula. Part zero
    // runs all postconditions for the other shapes; no producer is summarized.
    if constexpr (storage_part == 0U || storage_part == 1U)
    {
        auto const zero = Matrix::zero();
        flatten(zero, observed);
        for (std::size_t i = 0U; i < count; ++i)
            __ESBMC_assert(same(observed[i], Scalar{0}), "F-LINALG: every initialized raw cell is positive zero");
        Matrix copy(input);
        assert_stored_values(copy, before);
        Matrix moved(std::move(copy));
        assert_stored_values(moved, before);
        auto & assigned = (output = moved);
        __ESBMC_assert(&assigned == &output, "F-LINALG: assignment returns its target");
        assert_stored_values(output, before);
        output = std::move(moved);
        assert_stored_values(output, before);
        output = std::move(output);
        assert_stored_values(output, before);
        output = output;
        flatten(output, observed);
        for (std::size_t i = 0U; i < count; ++i)
            __ESBMC_assert(same(observed[i], before[i]), "F-LINALG: copy/move and self assignment preserve all values");
        assert_stored_values(input, before);
    }
    if constexpr (storage_part == 0U || storage_part == 2U)
    {
        // Local owning coefficients remain arbitrary via the symbolic input.
        auto const constructed = Matrix::from_row_major(before);
        assert_stored_values(constructed, before);
        assert_stored_values(input, before);
    }
    if constexpr (rows == columns && (storage_part == 0U || storage_part == 3U))
    {
        auto const identity = Matrix::identity();
        flatten(identity, observed);
        for (std::size_t i = 0U; i < count; ++i)
            __ESBMC_assert(same(observed[i], i / columns == i % columns ? Scalar{1} : Scalar{0}),
                           "F-LINALG: identity has precisely the diagonal ones");
    }
}

void verify_linalg_access(Matrix input, std::size_t row, std::size_t column, Scalar replacement)
{
    __ESBMC_assert(actual, "Runner error: linalg access requires actual operations");
    if constexpr (!actual)
        return;
    // Public coefficient access is unchecked; bounds are its caller premise.
    __ESBMC_assume(row < rows && column < columns);
    auto const copy = input;
    Scalar before[count]{}, observed[count]{};
    flatten(input, before);
    __ESBMC_assert(same(input(row, column), before[row * columns + column]) &&
                       same(copy(row, column), before[row * columns + column]),
                   "F-LINALG: mutable and const access select the requested raw coefficient");
    assert_stored_values(input, before);
    assert_stored_values(copy, before);
    input.set(row, column, replacement);
    flatten(input, observed);
    for (std::size_t i = 0U; i < count; ++i)
        __ESBMC_assert(same(observed[i], i == row * columns + column ? replacement : before[i]),
                       "F-LINALG: setter changes only the requested coefficient");
    input(row, column) = -replacement;
    if constexpr (columns == 1U)
    {
        __ESBMC_assert(same(input(row), -replacement) && same(copy(row), before[row]),
                       "F-LINALG: vector access agrees with column-zero access");
        input.set(row, replacement);
        input(row) = -replacement;
    }
    flatten(input, observed);
    for (std::size_t i = 0U; i < count; ++i)
        __ESBMC_assert(same(observed[i], i == row * columns + column ? -replacement : before[i]),
                       "F-LINALG: mutable reference writes only the selected cell");
    flatten(copy, observed);
    for (std::size_t i = 0U; i < count; ++i)
        __ESBMC_assert(same(observed[i], before[i]), "F-LINALG: copied matrices do not share storage");
}

void verify_linalg_elementwise(Matrix left, Matrix right, Scalar scalar)
{
    __ESBMC_assert(actual, "Runner error: linalg arithmetic requires actual operations");
    if constexpr (!actual)
        return;
    auto const old_left = left;
    auto const old_right = right;
    Scalar a[count]{}, b[count]{}, observed[count]{};
    flatten(old_left, a);
    flatten(old_right, b);
    auto const add = left + right;
    flatten(add, observed);
    for (std::size_t i = 0U; i < count; ++i)
        __ESBMC_assert(same(observed[i], a[i] + b[i]), "F-LINALG: eager addition coefficients");
    auto const subtract = left - right;
    flatten(subtract, observed);
    for (std::size_t i = 0U; i < count; ++i)
        __ESBMC_assert(same(observed[i], a[i] - b[i]), "F-LINALG: eager subtraction coefficients");
    auto const negate = -left;
    flatten(negate, observed);
    for (std::size_t i = 0U; i < count; ++i)
        __ESBMC_assert(same(observed[i], -a[i]), "F-LINALG: eager negation coefficients");
    auto const scale = left * scalar;
    flatten(scale, observed);
    for (std::size_t i = 0U; i < count; ++i)
        __ESBMC_assert(same(observed[i], a[i] * scalar), "F-LINALG: eager scaling coefficients");
    __ESBMC_assert(same_matrix(scalar * left, scale),
                   "F-LINALG: scalar-left overload delegates to the same ordered scale");
    // Raw division has no Status and performs IEEE division, including zero
    // divisors. Checked normalization owns the safe-divisor precondition.
    auto const divide = left / scalar;
    flatten(divide, observed);
    for (std::size_t i = 0U; i < count; ++i)
        __ESBMC_assert(same(observed[i], a[i] / scalar), "F-LINALG: eager division coefficients");
    __ESBMC_assert(same_matrix(left, old_left) && same_matrix(right, old_right),
                   "F-LINALG: arithmetic preserves both input matrices");
}

void verify_linalg_transpose(Matrix input)
{
    __ESBMC_assert(actual, "Runner error: linalg transpose requires actual operations");
    if constexpr (!actual)
        return;
    auto const before = input;
    Scalar source[count]{}, observed[count]{};
    flatten(before, source);
    auto const result = la::transpose(input);
    flatten(result, observed);
    for (std::size_t r = 0U; r < rows; ++r)
        for (std::size_t c = 0U; c < columns; ++c)
            __ESBMC_assert(same(observed[c * rows + r], source[r * columns + c]),
                           "F-LINALG: transpose swaps mathematical row/column and preserves every value");
    __ESBMC_assert(same_matrix(input, before), "F-LINALG: transpose preserves input");
}

constexpr std::size_t start_row = FORMAL_ESKF_PROOF_START_ROW;
constexpr std::size_t start_column = FORMAL_ESKF_PROOF_START_COLUMN;
constexpr std::size_t block_rows = FORMAL_ESKF_PROOF_BLOCK_ROWS;
constexpr std::size_t block_columns = FORMAL_ESKF_PROOF_BLOCK_COLUMNS;
using Block = Backend::matrix_type<block_rows, block_columns>;

void verify_linalg_block(Matrix input, Block replacement)
{
    __ESBMC_assert(actual, "Runner error: linalg block requires actual operations");
    if constexpr (!actual)
        return;
    auto const before = input;
    auto const old_replacement = replacement;
    Scalar a[count]{}, b[block_rows * block_columns]{}, observed[count]{};
    flatten(before, a);
    flatten(old_replacement, b);
    auto const block = input.template block<start_row, start_column, block_rows, block_columns>();
    flatten(block, observed);
    for (std::size_t r = 0U; r < block_rows; ++r)
        for (std::size_t c = 0U; c < block_columns; ++c)
            __ESBMC_assert(same(observed[r * block_columns + c], a[(start_row + r) * columns + start_column + c]),
                           "F-LINALG: extraction uses both offsets and the source row stride");
#if FORMAL_ESKF_PROOF_PRODUCT_COLUMNS == 1 && FORMAL_ESKF_PROOF_BLOCK_COLUMNS == 1
    __ESBMC_assert(same_matrix(input.template segment<start_row, block_rows>(), block),
                   "F-LINALG: vector segment selects the same coordinates as its block");
#endif
    __ESBMC_assert(same_matrix(input, before), "F-LINALG: block and segment extraction preserve their source");
    input.template set_block<start_row, start_column>(replacement);
#if FORMAL_ESKF_PROOF_PRODUCT_COLUMNS == 1 && FORMAL_ESKF_PROOF_BLOCK_COLUMNS == 1
    {
        auto via_segment = before;
        via_segment.template set_segment<start_row>(replacement);
        __ESBMC_assert(same_matrix(input, via_segment), "F-LINALG: segment replacement agrees with block replacement");
    }
#endif
    flatten(input, observed);
    for (std::size_t r = 0U; r < rows; ++r)
        for (std::size_t c = 0U; c < columns; ++c)
        {
            bool const inside =
                r >= start_row && r < start_row + block_rows && c >= start_column && c < start_column + block_columns;
            __ESBMC_assert(same(observed[r * columns + c],
                                inside ? b[(r - start_row) * block_columns + c - start_column] : a[r * columns + c]),
                           "F-LINALG: block replacement changes exactly the selected rectangle");
        }
    __ESBMC_assert(same_matrix(replacement, old_replacement), "F-LINALG: block source is unchanged");
    if constexpr (rows == block_rows && columns == block_columns)
    {
        auto const snapshot = input;
        input.template set_block<0U, 0U>(input);
#if FORMAL_ESKF_PROOF_PRODUCT_COLUMNS == 1
        input.template set_segment<0U>(input);
#endif
        __ESBMC_assert(same_matrix(input, snapshot), "F-LINALG: whole-object self replacement preserves values");
    }
}

void verify_linalg_reductions(Matrix input)
{
    __ESBMC_assert(actual, "Runner error: linalg reductions require actual operations");
    if constexpr (!actual)
        return;
    auto const before = input;
    Scalar a[count]{};
    flatten(before, a);
    bool finite = true;
    Scalar maximum{0};
    for (std::size_t i = 0U; i < count; ++i)
        finite = std::isfinite(a[i]) && finite;
    for (std::size_t i = 0U; i < count; ++i)
    {
        Scalar const candidate = magnitude(a[i]);
        if (!std::isfinite(candidate))
        {
            maximum = candidate;
            break;
        }
        if (candidate > maximum)
            maximum = candidate;
    }
    __ESBMC_assert(la::all_finite(input) == finite, "F-LINALG: finite predicate inspects all raw cells");
    __ESBMC_assert(same(la::max_abs(input), maximum),
                   "F-LINALG: max_abs returns maximum or first row-major non-finite magnitude");
#if FORMAL_ESKF_PROOF_ROWS == FORMAL_ESKF_PROOF_PRODUCT_COLUMNS
    {
        Scalar expected{0};
        for (std::size_t i = 0U; i < rows; ++i)
            expected += a[i * columns + i];
        __ESBMC_assert(same(la::trace(input), expected), "F-LINALG: trace uses ascending diagonal IEEE addition");
        auto const diagonal = la::diagonal(input);
        Scalar d[rows]{};
        flatten(diagonal, d);
        for (std::size_t i = 0U; i < rows; ++i)
            __ESBMC_assert(same(d[i], a[i * columns + i]),
                           "F-LINALG: diagonal extracts exactly the diagonal coefficients");
    }
#endif
    __ESBMC_assert(same_matrix(input, before), "F-LINALG: reductions preserve input");
}

void verify_linalg_symmetry(Matrix input, Scalar tolerance)
{
    __ESBMC_assert(actual && rows == columns, "Runner error: linalg symmetry requires actual square operations");
    if constexpr (!actual)
        return;
#if FORMAL_ESKF_PROOF_ROWS == FORMAL_ESKF_PROOF_PRODUCT_COLUMNS
    auto const before = input;
    Scalar a[count]{}, observed[count]{};
    flatten(before, a);
    auto const symmetric = la::symmetrize(input);
    flatten(symmetric, observed);
    bool valid = std::isfinite(tolerance) && tolerance >= Scalar{0};
    Scalar differences[count]{};
    for (std::size_t r = 0U; r < rows; ++r)
        for (std::size_t c = 0U; c < columns; ++c)
        {
            Scalar const sum = a[r * columns + c] + a[c * columns + r];
            __ESBMC_assert(same(observed[r * columns + c], sum * Scalar{0.5}),
                           "F-LINALG: symmetrize averages each transposed pair in the declared IEEE order");
            differences[r * columns + c] = a[r * columns + c] - a[c * columns + r];
            valid = std::isfinite(a[r * columns + c]) && valid;
        }
    // Independent raw-array residual, with the same specified reduction order.
    // Avoid asking the solver to rediscover an ordering theorem for 225 IEEE
    // comparisons; no assumption or restriction is added to the input domain.
    Scalar maximum{0};
    for (std::size_t i = 0U; i < count; ++i)
    {
        Scalar const candidate = magnitude(differences[i]);
        if (!std::isfinite(candidate))
        {
            maximum = candidate;
            break;
        }
        if (candidate > maximum)
            maximum = candidate;
    }
    bool const expected = valid && maximum <= tolerance;
    __ESBMC_assert(la::is_symmetric(input, tolerance) == expected,
                   "F-LINALG: symmetry uses finite inputs, valid tolerance and every transposed-pair residual");
    __ESBMC_assert(same_matrix(input, before), "F-LINALG: symmetry operations preserve input");
#endif
}

using Vector3 = Backend::vector_type<3U>;
void verify_linalg_vector(Vector3 left, Vector3 right, Scalar root)
{
    __ESBMC_assert(actual, "Runner error: linalg vector requires actual operations");
    if constexpr (!actual)
        return;
    auto const old_left = left;
    auto const old_right = right;
    Scalar a[3]{}, b[3]{}, observed[3]{};
    flatten(old_left, a);
    flatten(old_right, b);
    auto const cross = la::cross(left, right);
    flatten(cross, observed);
    for (std::size_t i = 0U; i < 3U; ++i)
        __ESBMC_assert(same(observed[i], a[(i + 1U) % 3U] * b[(i + 2U) % 3U] - a[(i + 2U) % 3U] * b[(i + 1U) % 3U]),
                       "F-LINALG: cross product follows right-handed cyclic coefficient equations");
    Scalar dot{0}, squared{0};
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        dot += a[i] * b[i];
        squared += a[i] * a[i];
    }
    __ESBMC_assert(same(la::dot(left, right), dot) && same(la::squared_norm(left), squared),
                   "F-LINALG: dot and squared norm use ordered products and additions");
#if FORMAL_ESKF_PROOF_LINALG_BACKEND == 0
    // The exact existing norm backend with an arbitrary scalar-root return.
    // No root accuracy or finite result is assumed; F-SCALAR owns that contract.
    contract_proof::OpaqueMath::prepare(root, Scalar{0}, Scalar{0}, Scalar{0});
    auto const norm = la::norm(left);
    __ESBMC_assert(contract_proof::OpaqueMath::sqrt.calls == 1U &&
                       same(contract_proof::OpaqueMath::sqrt.argument, squared) && same(norm, root),
                   "F-LINALG: norm forwards the complete squared norm and returns the scalar result");
#else
    (void)root; // solve_proof::Math's positive-pivot observer is not a general norm model.
#endif
    __ESBMC_assert(same_matrix(left, old_left) && same_matrix(right, old_right),
                   "F-LINALG: vector operations preserve both inputs");
}

// The missing small product shapes for contract_proof::Linalg. Large products
// and solve_proof::Backend use their already proved entry/dot decompositions.
void verify_linalg_product(Matrix left, Backend::matrix_type<columns, FORMAL_ESKF_PROOF_OTHER_COLUMNS> right)
{
    __ESBMC_assert(actual, "Runner error: linalg product requires actual operations");
    if constexpr (!actual)
        return;
    auto const old_left = left;
    auto const old_right = right;
    constexpr std::size_t other_columns = FORMAL_ESKF_PROOF_OTHER_COLUMNS;
    Scalar a[count]{}, b[columns * other_columns]{}, observed[rows * other_columns]{};
    flatten(old_left, a);
    flatten(old_right, b);
    auto const result = left * right;
    flatten(result, observed);
    for (std::size_t r = 0U; r < rows; ++r)
        for (std::size_t c = 0U; c < other_columns; ++c)
        {
            Scalar expected{0};
            for (std::size_t k = 0U; k < columns; ++k)
                expected += a[r * columns + k] * b[k * other_columns + c];
            __ESBMC_assert(same(observed[r * other_columns + c], expected),
                           "F-LINALG: every product entry uses the complete ordered inner product");
        }
    __ESBMC_assert(same_matrix(left, old_left) && same_matrix(right, old_right),
                   "F-LINALG: matrix product preserves both inputs");
}

int main() { return 0; }
