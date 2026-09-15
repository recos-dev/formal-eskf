/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

// Included after correction_support's types. This is a proof backend, not a
// deployment implementation: factor ordered products into an entry operation
// and the existing actual dot operation. The order of IEEE operations is kept.
#ifndef FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY
#define FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY 0
#endif

namespace correction_proof
{
struct EntryCalls
{
    inline static void const * left = nullptr;
    inline static void const * right = nullptr;
    inline static void const * entries = nullptr;
    inline static void const * received_left = nullptr;
    inline static void const * received_right = nullptr;
    inline static std::size_t count{};
};
template <std::size_t K> struct DotCalls
{
    inline static Backend::vector_type<K> left{}, right{};
    inline static Scalar result{};
    inline static unsigned count{};
};
} // namespace correction_proof

#if FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY == 2
namespace formal_eskf::linalg
{
#define CORRECTION_DOT(K)                                                                                              \
    template <>                                                                                                        \
    inline correction_proof::Scalar dot<correction_proof::Backend, K>(                                                 \
        Matrix<correction_proof::Backend, K, 1U> const & left,                                                         \
        Matrix<correction_proof::Backend, K, 1U> const & right) noexcept                                               \
    {                                                                                                                  \
        using Call = correction_proof::DotCalls<K>;                                                                    \
        __ESBMC_assert(Call::count++ == 0U, "E-CORRECT entry: exactly one dot product");                               \
        Call::left = left;                                                                                             \
        Call::right = right;                                                                                           \
        return Call::result;                                                                                           \
    }
CORRECTION_DOT(1U)
CORRECTION_DOT(2U)
CORRECTION_DOT(3U)
CORRECTION_DOT(15U)
#undef CORRECTION_DOT
} // namespace formal_eskf::linalg
#endif

namespace correction_proof
{
template <std::size_t R, std::size_t K, std::size_t C>
Scalar product_entry(Backend::storage_type<R, K> const & left, Backend::storage_type<K, C> const & right,
                     std::size_t row, std::size_t column)
{
#if FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY == 1
    using Call = EntryCalls;
    if (Call::count == 0U)
    {
        auto const & expected_left = *static_cast<Backend::storage_type<R, K> const *>(Call::left);
        auto const & expected_right = *static_cast<Backend::storage_type<K, C> const *>(Call::right);
        __ESBMC_assert(same_cells(left.values, expected_left.values) && same_cells(right.values, expected_right.values),
                       "E-CORRECT product: complete original operand values");
        Call::received_left = &left;
        Call::received_right = &right;
    }
    __ESBMC_assert(&left == Call::received_left && &right == Call::received_right,
                   "E-CORRECT product: every entry uses the same read-only operands");
    __ESBMC_assert(row == Call::count / C && column == Call::count % C && Call::count < R * C,
                   "E-CORRECT product: every output position exactly once");
    ++Call::count;
    return static_cast<Backend::storage_type<R, C> const *>(Call::entries)->values[row * C + column];
#else
    Backend::vector_type<K> row_values, column_values;
    for (std::size_t k = 0U; k < K; ++k)
    {
        row_values.set(k, Backend::coefficient(left, row, k));
        column_values.set(k, Backend::coefficient(right, k, column));
    }
    return formal_eskf::linalg::dot(row_values, column_values);
#endif
}
template <std::size_t R, std::size_t K, std::size_t C>
void ordered_product(Backend::storage_type<R, K> const & left, Backend::storage_type<K, C> const & right,
                     Backend::storage_type<R, C> & output)
{
    for (std::size_t row = 0U; row < R; ++row)
        for (std::size_t column = 0U; column < C; ++column)
            output.values[row * C + column] = product_entry<R, K, C>(left, right, row, column);
}
} // namespace correction_proof

namespace formal_eskf::verification
{
// Only shapes actually consumed by correction and its Joseph sandwiches.
// Cholesky performs no matrix multiplication; its proved bodies are unchanged.
#define CORRECTION_MULTIPLY(R, K, C)                                                                                   \
    template <>                                                                                                        \
    template <>                                                                                                        \
    inline void FixedArrayLinalg<correction_proof::Scalar, NoNormObserver<correction_proof::Scalar>,                   \
                                 solve_proof::Math>::multiply<R, K, C>(storage_type<R, K> const & left,                \
                                                                       storage_type<K, C> const & right,               \
                                                                       storage_type<R, C> & output) noexcept           \
    {                                                                                                                  \
        correction_proof::ordered_product<R, K, C>(left, right, output);                                               \
    }
CORRECTION_MULTIPLY(15U, 15U, 1U)
CORRECTION_MULTIPLY(15U, 15U, 15U)
CORRECTION_MULTIPLY(15U, 15U, 2U)
CORRECTION_MULTIPLY(15U, 15U, 3U)
CORRECTION_MULTIPLY(15U, 1U, 1U)
CORRECTION_MULTIPLY(15U, 1U, 15U)
CORRECTION_MULTIPLY(15U, 2U, 1U)
CORRECTION_MULTIPLY(15U, 2U, 15U)
CORRECTION_MULTIPLY(15U, 2U, 2U)
CORRECTION_MULTIPLY(15U, 3U, 1U)
CORRECTION_MULTIPLY(15U, 3U, 15U)
CORRECTION_MULTIPLY(15U, 3U, 3U)
CORRECTION_MULTIPLY(1U, 15U, 1U)
CORRECTION_MULTIPLY(1U, 3U, 1U)
CORRECTION_MULTIPLY(2U, 15U, 2U)
CORRECTION_MULTIPLY(2U, 3U, 2U)
CORRECTION_MULTIPLY(3U, 15U, 3U)
CORRECTION_MULTIPLY(3U, 1U, 1U)
CORRECTION_MULTIPLY(3U, 1U, 3U)
CORRECTION_MULTIPLY(3U, 2U, 1U)
CORRECTION_MULTIPLY(3U, 2U, 2U)
CORRECTION_MULTIPLY(3U, 2U, 3U)
CORRECTION_MULTIPLY(3U, 3U, 1U)
CORRECTION_MULTIPLY(3U, 3U, 2U)
CORRECTION_MULTIPLY(3U, 3U, 3U)
#undef CORRECTION_MULTIPLY
} // namespace formal_eskf::verification
