/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include "covariance_support.hpp"

namespace covariance_oracle
{
using namespace contract_proof;

inline bool same_cells(Scalar a, Scalar b) { return same(a, b); }
inline bool finite_cells(Scalar value) { return std::isfinite(value); }

template <typename Cell>
bool same_cells(formal_eskf::verification::CovarianceRow<Cell> const & a,
                formal_eskf::verification::CovarianceRow<Cell> const & b)
{
    bool valid = true;
    valid = same_cells(a.c0, b.c0) && valid;
    valid = same_cells(a.c1, b.c1) && valid;
    valid = same_cells(a.c2, b.c2) && valid;
    valid = same_cells(a.c3, b.c3) && valid;
    valid = same_cells(a.c4, b.c4) && valid;
    valid = same_cells(a.c5, b.c5) && valid;
    valid = same_cells(a.c6, b.c6) && valid;
    valid = same_cells(a.c7, b.c7) && valid;
    valid = same_cells(a.c8, b.c8) && valid;
    valid = same_cells(a.c9, b.c9) && valid;
    valid = same_cells(a.c10, b.c10) && valid;
    valid = same_cells(a.c11, b.c11) && valid;
    valid = same_cells(a.c12, b.c12) && valid;
    valid = same_cells(a.c13, b.c13) && valid;
    valid = same_cells(a.c14, b.c14) && valid;
    return valid;
}
template <typename Cell> bool finite_cells(formal_eskf::verification::CovarianceRow<Cell> const & a)
{
    bool valid = true;
    valid = finite_cells(a.c0) && valid;
    valid = finite_cells(a.c1) && valid;
    valid = finite_cells(a.c2) && valid;
    valid = finite_cells(a.c3) && valid;
    valid = finite_cells(a.c4) && valid;
    valid = finite_cells(a.c5) && valid;
    valid = finite_cells(a.c6) && valid;
    valid = finite_cells(a.c7) && valid;
    valid = finite_cells(a.c8) && valid;
    valid = finite_cells(a.c9) && valid;
    valid = finite_cells(a.c10) && valid;
    valid = finite_cells(a.c11) && valid;
    valid = finite_cells(a.c12) && valid;
    valid = finite_cells(a.c13) && valid;
    valid = finite_cells(a.c14) && valid;
    return valid;
}

template <typename Cell, std::size_t Count>
bool same_cells(formal_eskf::verification::ScalarArray<Cell, Count> const & a,
                formal_eskf::verification::ScalarArray<Cell, Count> const & b)
{
    bool valid = same_cells(a.head, b.head);
    if constexpr (Count > 1U)
    {
        valid = same_cells(a.tail, b.tail) && valid;
    }
    return valid;
}

template <typename Cell, std::size_t Count>
bool finite_cells(formal_eskf::verification::ScalarArray<Cell, Count> const & a)
{
    bool valid = finite_cells(a.head);
    if constexpr (Count > 1U)
    {
        valid = finite_cells(a.tail) && valid;
    }
    return valid;
}

template <typename M> auto const & cells(M const & matrix)
{
    auto const & storage = formal_eskf::linalg::detail::MatrixAccess::storage(matrix).values;
    if constexpr (M::row_count == 15U && M::column_count == 15U)
    {
        return storage.rows;
    }
    else
    {
        return storage;
    }
}

template <typename M> bool same_matrix(M const & a, M const & b) { return same_cells(cells(a), cells(b)); }
template <typename M> bool finite_matrix(M const & a) { return finite_cells(cells(a)); }

// Independent row-major oracle traversal. Production still executes its
// actual coefficient lookup, storage and arithmetic, without storage summaries.
inline void flatten_cells(Scalar value, Scalar *& output) { *output++ = value; }
template <typename Cell> void flatten_cells(formal_eskf::verification::CovarianceRow<Cell> const & a, Scalar *& output)
{
    flatten_cells(a.c0, output);
    flatten_cells(a.c1, output);
    flatten_cells(a.c2, output);
    flatten_cells(a.c3, output);
    flatten_cells(a.c4, output);
    flatten_cells(a.c5, output);
    flatten_cells(a.c6, output);
    flatten_cells(a.c7, output);
    flatten_cells(a.c8, output);
    flatten_cells(a.c9, output);
    flatten_cells(a.c10, output);
    flatten_cells(a.c11, output);
    flatten_cells(a.c12, output);
    flatten_cells(a.c13, output);
    flatten_cells(a.c14, output);
}
template <typename Cell, std::size_t Count>
void flatten_cells(formal_eskf::verification::ScalarArray<Cell, Count> const & a, Scalar *& output)
{
    flatten_cells(a.head, output);
    if constexpr (Count > 1U)
    {
        flatten_cells(a.tail, output);
    }
}
template <typename M> void flatten(M const & matrix, Scalar * output) { flatten_cells(cells(matrix), output); }

} // namespace covariance_oracle
