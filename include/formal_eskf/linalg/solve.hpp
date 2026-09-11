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
 * The backends use Cholesky (LLT), sharing one factorization across all
 * right-hand-side columns. No inverse is formed. The caller is responsible for
 * cleaning up roundoff asymmetry when constructing a covariance system.
 *
 * Non-finite inputs return non_finite_input. Asymmetric inputs or a failed
 * positive-definite factorization return not_positive_definite. Non-finite
 * factors or solutions return non_finite_result. Success does not certify a
 * condition number or forward-error bound; no conditioning cutoff is imposed.
 *
 * Failure leaves output unchanged. Output may alias either input when their
 * dimensions match.
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

/**
 * Solve solution * system = right_hand_side for the same SPD domain.
 * Uses system * solution^T = right_hand_side^T because system is symmetric.
 * The finite-input, failure and output-alias rules of solve_spd also apply.
 */
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
