/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <formal_eskf/linalg/linalg.hpp>

namespace formal_eskf::detail
{

/** Shared finite-result check and symmetry cleanup for covariance transitions. */
template <typename Linalg, std::size_t Size>
[[nodiscard]] Status try_finish_covariance(linalg::Matrix<Linalg, Size, Size> & candidate,
                                           linalg::Matrix<Linalg, Size, Size> & output) noexcept
{
    if (!linalg::all_finite(candidate))
    {
        return Status::non_finite_result;
    }
    // Remove roundoff asymmetry, not an indefinite covariance's negative
    // eigenvalues. Halve before adding to avoid overflow in a + a^T.
    for (std::size_t row = 0U; row < Size; ++row)
    {
        for (std::size_t column = row + 1U; column < Size; ++column)
        {
            auto const mean = typename Linalg::value_type{0.5} * candidate(row, column) +
                              typename Linalg::value_type{0.5} * candidate(column, row);
            candidate.set(row, column, mean);
            candidate.set(column, row, mean);
        }
    }
    output = candidate;
    return Status::success;
}

} /* end namespace formal_eskf::detail */
