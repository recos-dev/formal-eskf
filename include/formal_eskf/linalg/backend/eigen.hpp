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
 * Eigen implementation of the fixed-size linear algebra backend.
 */

#include <cstddef>
#include <type_traits>

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <formal_eskf/status.hpp>
#include <formal_eskf/scalar/backend/standard.hpp>

namespace formal_eskf::linalg
{

/**
 * Fixed-size Eigen backend.
 *
 * Eigen storage and expression templates remain private implementation
 * details.  Public matrix operations always return eager Matrix values.
 */
template <typename Scalar> class EigenBackend
{

    static_assert(std::is_floating_point_v<Scalar>);

public:
    using value_type = Scalar;
    using scalar_math_type = scalar::StandardMath<value_type>;

    template <std::size_t Rows, std::size_t Columns>
    using storage_type = Eigen::Matrix<value_type, static_cast<int>(Rows), static_cast<int>(Columns)>;

    template <std::size_t Rows, std::size_t Columns> static void set_zero(storage_type<Rows, Columns> & matrix)
    {
        matrix.setZero();
    }

    template <std::size_t Rows, std::size_t Columns>
    static value_type const & coefficient(storage_type<Rows, Columns> const & matrix, std::size_t row,
                                          std::size_t column)
    {
        return matrix(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column));
    }

    template <std::size_t Rows, std::size_t Columns>
    static value_type & coefficient(storage_type<Rows, Columns> & matrix, std::size_t row, std::size_t column)
    {
        return matrix(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column));
    }

    template <std::size_t Rows, std::size_t Columns>
    static void add(storage_type<Rows, Columns> const & left, storage_type<Rows, Columns> const & right,
                    storage_type<Rows, Columns> & result)
    {
        result = left + right;
    }

    template <std::size_t Rows, std::size_t Columns>
    static void subtract(storage_type<Rows, Columns> const & left, storage_type<Rows, Columns> const & right,
                         storage_type<Rows, Columns> & result)
    {
        result = left - right;
    }

    template <std::size_t Rows, std::size_t Columns>
    static void negate(storage_type<Rows, Columns> const & matrix, storage_type<Rows, Columns> & result)
    {
        result = -matrix;
    }

    template <std::size_t Rows, std::size_t Columns>
    static void scale(storage_type<Rows, Columns> const & matrix, value_type scalar,
                      storage_type<Rows, Columns> & result)
    {
        result = matrix * scalar;
    }

    template <std::size_t Rows, std::size_t Columns>
    static void divide(storage_type<Rows, Columns> const & matrix, value_type scalar,
                       storage_type<Rows, Columns> & result)
    {
        result = matrix / scalar;
    }

    template <std::size_t Rows, std::size_t Inner, std::size_t Columns>
    static void multiply(storage_type<Rows, Inner> const & left, storage_type<Inner, Columns> const & right,
                         storage_type<Rows, Columns> & result)
    {
        result.noalias() = left * right;
    }

    template <std::size_t Rows, std::size_t Columns>
    static void transpose(storage_type<Rows, Columns> const & matrix, storage_type<Columns, Rows> & result)
    {
        result = matrix.transpose();
    }

    template <std::size_t Size>
    static value_type dot(storage_type<Size, 1U> const & left, storage_type<Size, 1U> const & right)
    {
        return left.dot(right);
    }

    template <std::size_t Size> static value_type squared_norm(storage_type<Size, 1U> const & vector)
    {
        return vector.squaredNorm();
    }

    template <std::size_t Size> static value_type norm(storage_type<Size, 1U> const & vector) { return vector.norm(); }

    template <std::size_t Size, std::size_t RightColumns>
    [[nodiscard]] static Status solve_spd(storage_type<Size, Size> const & system,
                                          storage_type<Size, RightColumns> const & right_hand_side,
                                          storage_type<Size, RightColumns> & solution);

}; /* end class EigenBackend */

template <typename Scalar>
template <std::size_t Size, std::size_t RightColumns>
Status EigenBackend<Scalar>::solve_spd(storage_type<Size, Size> const & system,
                                       storage_type<Size, RightColumns> const & right_hand_side,
                                       storage_type<Size, RightColumns> & solution)
{
    using system_storage_type = storage_type<Size, Size>;
    Eigen::LLT<system_storage_type> const factorization(system);
    if (factorization.info() != Eigen::Success)
    {
        return Status::not_positive_definite;
    }
    if (!factorization.matrixLLT().allFinite())
    {
        return Status::non_finite_result;
    }

    solution = factorization.solve(right_hand_side);
    return Status::success;
}

} /* end namespace formal_eskf::linalg */
