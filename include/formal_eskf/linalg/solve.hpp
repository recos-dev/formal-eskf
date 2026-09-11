/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

/**
 * @file
 * Fallible fixed-size linear algebra operations.
 */

#include <cstddef>

#include <formal_eskf/linalg/operations.hpp>
#include <formal_eskf/status.hpp>

namespace formal_eskf::linalg
{

/**
 * Solve system * solution = right_hand_side for a finite, exactly symmetric,
 * positive-definite system matrix.
 *
 * The output is unchanged on failure.  A configurable finite-precision
 * symmetry and conditioning policy will replace the exact symmetry check once
 * that numerical profile is defined.
 */
template <typename Linalg, std::size_t Size, std::size_t RightColumns>
[[nodiscard]] Status solve_spd(Matrix<Linalg, Size, Size> const & system,
                               Matrix<Linalg, Size, RightColumns> const & right_hand_side,
                               Matrix<Linalg, Size, RightColumns> & solution) noexcept
{
    using value_type = typename Linalg::value_type;

    if (!all_finite(system) || !all_finite(right_hand_side))
    {
        return Status::non_finite_input;
    }
    if (!is_symmetric(system, value_type{0}))
    {
        return Status::not_positive_definite;
    }

    Matrix<Linalg, Size, RightColumns> candidate;
    Status const status = Linalg::template solve_spd<Size, RightColumns>(detail::MatrixAccess::storage(system),
                                                                         detail::MatrixAccess::storage(right_hand_side),
                                                                         detail::MatrixAccess::storage(candidate));
    if (!succeeded(status))
    {
        return status;
    }
    if (!all_finite(candidate))
    {
        return Status::non_finite_result;
    }

    solution = candidate;
    return Status::success;
}

template <typename Linalg, std::size_t Rows, std::size_t Size>
[[nodiscard]] Status right_solve_spd(Matrix<Linalg, Rows, Size> const & right_hand_side,
                                     Matrix<Linalg, Size, Size> const & system,
                                     Matrix<Linalg, Rows, Size> & solution) noexcept
{
    Matrix<Linalg, Size, Rows> const transposed_right_hand_side = transpose(right_hand_side);
    Matrix<Linalg, Size, Rows> transposed_solution;
    Status const status = solve_spd(system, transposed_right_hand_side, transposed_solution);
    if (!succeeded(status))
    {
        return status;
    }

    solution = transpose(transposed_solution);
    return Status::success;
}

} /* end namespace formal_eskf::linalg */
