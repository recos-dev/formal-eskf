/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include "solve_support.hpp"
#include "covariance_oracle.hpp"
#include <cstdint>
#include <type_traits>

#ifndef FORMAL_ESKF_PROOF_STATE_SIZE
#define FORMAL_ESKF_PROOF_STATE_SIZE 3
#endif
#ifndef FORMAL_ESKF_PROOF_MEASUREMENT_SIZE
#define FORMAL_ESKF_PROOF_MEASUREMENT_SIZE 3
#endif
#ifndef FORMAL_ESKF_PROOF_CORRECTION_CONTRACT
#define FORMAL_ESKF_PROOF_CORRECTION_CONTRACT 0
#endif
#ifndef FORMAL_ESKF_PROOF_FINITE_CONTRACT
#define FORMAL_ESKF_PROOF_FINITE_CONTRACT 0
#endif

// Reuse the named 15x15 storage representation, now for the exact backend
// instantiated by F-SOLVE. No F-SOLVE shape uses 15x15 storage, and no existing
// scalar/solver operation or smaller storage specialization is changed.
namespace formal_eskf::verification
{
template <>
template <>
struct FixedArrayLinalg<contract_proof::Scalar, NoNormObserver<contract_proof::Scalar>,
                        solve_proof::Math>::storage_type<15U, 15U>
{
    CovarianceCells<contract_proof::Scalar> values;
};
} // namespace formal_eskf::verification

#include <formal_eskf/eskf/correction.hpp>

namespace correction_proof
{
using covariance_oracle::cells;
using covariance_oracle::finite_matrix;
using formal_eskf::linalg::detail::MatrixAccess;
using solve_proof::Backend;
using solve_proof::Scalar;
using solve_proof::Status;
// Representation equality makes identical IEEE expressions cheap to compare.
// As in the existing oracle, preserve signed zero but do not claim NaN payloads.
inline bool same(Scalar a, Scalar b)
{
    using Bits = std::conditional_t<sizeof(Scalar) == 4U, std::uint32_t, std::uint64_t>;
    return __builtin_bit_cast(Bits, a) == __builtin_bit_cast(Bits, b) || (std::isnan(a) && std::isnan(b));
}
template <typename Cell> bool same_cells(Cell const & a, Cell const & b)
{
    if constexpr (std::is_same_v<Cell, Scalar>)
        return same(a, b);
    else if constexpr (requires { a.rows; })
        return same_cells(a.rows, b.rows);
    else if constexpr (requires { a.head; })
    {
        bool valid = same_cells(a.head, b.head);
        if constexpr (requires { a.tail; })
            valid = same_cells(a.tail, b.tail) && valid;
        return valid;
    }
    else
    {
        bool valid = true;
#define CORRECTION_CELL(I) valid = same_cells(a.c##I, b.c##I) && valid
        CORRECTION_CELL(0);
        CORRECTION_CELL(1);
        CORRECTION_CELL(2);
        CORRECTION_CELL(3);
        CORRECTION_CELL(4);
        CORRECTION_CELL(5);
        CORRECTION_CELL(6);
        CORRECTION_CELL(7);
        CORRECTION_CELL(8);
        CORRECTION_CELL(9);
        CORRECTION_CELL(10);
        CORRECTION_CELL(11);
        CORRECTION_CELL(12);
        CORRECTION_CELL(13);
        CORRECTION_CELL(14);
#undef CORRECTION_CELL
        return valid;
    }
}
template <typename M> bool same_matrix(M const & a, M const & b) { return same_cells(cells(a), cells(b)); }
constexpr std::size_t state_size = FORMAL_ESKF_PROOF_STATE_SIZE;
constexpr std::size_t measurement_size = FORMAL_ESKF_PROOF_MEASUREMENT_SIZE;
static_assert(state_size == 3U || state_size == 15U);
// The square prior sandwich additionally uses inner size 15; actual correction
// caller profiles are restricted to measurement sizes 1/2/3 by their guard.
static_assert(measurement_size >= 1U && (measurement_size <= 3U || measurement_size == state_size));
using Covariance = Backend::matrix_type<state_size, state_size>;
using Noise = Backend::matrix_type<measurement_size, measurement_size>;
using Jacobian = Backend::matrix_type<measurement_size, state_size>;
using Gain = Backend::matrix_type<state_size, measurement_size>;
using Residual = Backend::vector_type<measurement_size>;
using Correction = Backend::vector_type<state_size>;

template <typename M> bool nonnegative_diagonal(M const & matrix)
{
    bool valid = true;
    for (std::size_t i = 0U; i < M::row_count; ++i)
        valid = valid && matrix(i, i) >= Scalar{0};
    return valid;
}
template <typename M> bool positive_diagonal(M const & matrix)
{
    bool valid = true;
    for (std::size_t i = 0U; i < M::row_count; ++i)
        valid = valid && matrix(i, i) > Scalar{0};
    return valid;
}
template <typename M> bool identity_matrix(M const & matrix)
{
    bool valid = true;
    for (std::size_t i = 0U; i < M::row_count; ++i)
        for (std::size_t j = 0U; j < M::column_count; ++j)
            valid = same(matrix(i, j), i == j ? Scalar{1} : Scalar{0}) && valid;
    return valid;
}
} // namespace correction_proof

#include "correction_product.hpp"
