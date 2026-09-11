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
 * PX4 matrix implementation of the fixed-size linear algebra backend.
 */

#include <cstddef>
#include <type_traits>

#include <matrix/Vector.hpp>

#include <formal_eskf/linalg/linalg.hpp>
#include <formal_eskf/scalar/backend/standard.hpp>

namespace formal_eskf::linalg
{

/**
 * Fixed-size backend using the host's PX4 matrix headers.
 *
 * This adapter does not bundle or download PX4. The embedding build supplies
 * matrix and its platform/generated-header dependencies.
 *
 * Arithmetic uses PX4 storage and operators, except coefficient-wise division
 * and checked Cholesky solve, which preserve the public linalg contracts.
 */
template <typename Scalar> class Px4MatrixBackend
{

    static_assert(std::is_floating_point_v<Scalar>);

public:
    using value_type = Scalar;
    using scalar_math_type = scalar::StandardMath<value_type>;

    template <std::size_t Rows, std::size_t Columns>
    using matrix_type = Matrix<Px4MatrixBackend<Scalar>, Rows, Columns>;

    template <std::size_t Size> using vector_type = matrix_type<Size, 1U>;

    template <std::size_t Rows, std::size_t Columns> using storage_type = ::matrix::Matrix<value_type, Rows, Columns>;

    template <std::size_t Rows, std::size_t Columns>
    static void set_zero(storage_type<Rows, Columns> & matrix) noexcept;

    template <std::size_t Rows, std::size_t Columns>
    [[nodiscard]] static value_type const & coefficient(storage_type<Rows, Columns> const & matrix, std::size_t row,
                                                        std::size_t column) noexcept;

    template <std::size_t Rows, std::size_t Columns>
    [[nodiscard]] static value_type & coefficient(storage_type<Rows, Columns> & matrix, std::size_t row,
                                                  std::size_t column) noexcept;

    template <std::size_t Rows, std::size_t Columns>
    static void set_coefficient(storage_type<Rows, Columns> & matrix, std::size_t row, std::size_t column,
                                value_type value) noexcept;

    template <std::size_t Rows, std::size_t Columns>
    static void add(storage_type<Rows, Columns> const & left, storage_type<Rows, Columns> const & right,
                    storage_type<Rows, Columns> & result) noexcept;

    template <std::size_t Rows, std::size_t Columns>
    static void subtract(storage_type<Rows, Columns> const & left, storage_type<Rows, Columns> const & right,
                         storage_type<Rows, Columns> & result) noexcept;

    template <std::size_t Rows, std::size_t Columns>
    static void negate(storage_type<Rows, Columns> const & matrix, storage_type<Rows, Columns> & result) noexcept;

    template <std::size_t Rows, std::size_t Columns>
    static void scale(storage_type<Rows, Columns> const & matrix, value_type scalar,
                      storage_type<Rows, Columns> & result) noexcept;

    template <std::size_t Rows, std::size_t Columns>
    static void divide(storage_type<Rows, Columns> const & matrix, value_type scalar,
                       storage_type<Rows, Columns> & result) noexcept;

    template <std::size_t Rows, std::size_t Inner, std::size_t Columns>
    static void multiply(storage_type<Rows, Inner> const & left, storage_type<Inner, Columns> const & right,
                         storage_type<Rows, Columns> & result) noexcept;

    template <std::size_t Rows, std::size_t Columns>
    static void transpose(storage_type<Rows, Columns> const & matrix, storage_type<Columns, Rows> & result) noexcept;

    template <std::size_t Size>
    [[nodiscard]] static value_type dot(storage_type<Size, 1U> const & left,
                                        storage_type<Size, 1U> const & right) noexcept;

    template <std::size_t Size>
    [[nodiscard]] static value_type squared_norm(storage_type<Size, 1U> const & vector) noexcept;

    template <std::size_t Size> [[nodiscard]] static value_type norm(storage_type<Size, 1U> const & vector) noexcept;

    template <std::size_t Size, std::size_t RightColumns>
    [[nodiscard]] static Status solve_spd(storage_type<Size, Size> const & system,
                                          storage_type<Size, RightColumns> const & right_hand_side,
                                          storage_type<Size, RightColumns> & solution) noexcept;

}; /* end class Px4MatrixBackend */

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void Px4MatrixBackend<Scalar>::set_zero(storage_type<Rows, Columns> & matrix) noexcept
{
    matrix.setZero();
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
typename Px4MatrixBackend<Scalar>::value_type const &
Px4MatrixBackend<Scalar>::coefficient(storage_type<Rows, Columns> const & matrix, std::size_t row,
                                      std::size_t column) noexcept
{
    return matrix(row, column);
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
typename Px4MatrixBackend<Scalar>::value_type &
Px4MatrixBackend<Scalar>::coefficient(storage_type<Rows, Columns> & matrix, std::size_t row,
                                      std::size_t column) noexcept
{
    return matrix(row, column);
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void Px4MatrixBackend<Scalar>::set_coefficient(storage_type<Rows, Columns> & matrix, std::size_t row,
                                               std::size_t column, value_type value) noexcept
{
    matrix(row, column) = value;
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void Px4MatrixBackend<Scalar>::add(storage_type<Rows, Columns> const & left, storage_type<Rows, Columns> const & right,
                                   storage_type<Rows, Columns> & result) noexcept
{
    result = left + right;
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void Px4MatrixBackend<Scalar>::subtract(storage_type<Rows, Columns> const & left,
                                        storage_type<Rows, Columns> const & right,
                                        storage_type<Rows, Columns> & result) noexcept
{
    result = left - right;
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void Px4MatrixBackend<Scalar>::negate(storage_type<Rows, Columns> const & matrix,
                                      storage_type<Rows, Columns> & result) noexcept
{
    result = -matrix;
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void Px4MatrixBackend<Scalar>::scale(storage_type<Rows, Columns> const & matrix, value_type scalar,
                                     storage_type<Rows, Columns> & result) noexcept
{
    result = matrix * scalar;
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void Px4MatrixBackend<Scalar>::divide(storage_type<Rows, Columns> const & matrix, value_type scalar,
                                      storage_type<Rows, Columns> & result) noexcept
{
    // PX4's operator/ multiplies by a reciprocal, which can overflow even
    // when every coefficient-wise quotient is representable.
    for (std::size_t row = 0U; row < Rows; ++row)
    {
        for (std::size_t column = 0U; column < Columns; ++column)
        {
            result(row, column) = matrix(row, column) / scalar;
        }
    }
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Inner, std::size_t Columns>
void Px4MatrixBackend<Scalar>::multiply(storage_type<Rows, Inner> const & left,
                                        storage_type<Inner, Columns> const & right,
                                        storage_type<Rows, Columns> & result) noexcept
{
    result = left * right;
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void Px4MatrixBackend<Scalar>::transpose(storage_type<Rows, Columns> const & matrix,
                                         storage_type<Columns, Rows> & result) noexcept
{
    result = matrix.transpose();
}

template <typename Scalar>
template <std::size_t Size>
typename Px4MatrixBackend<Scalar>::value_type
Px4MatrixBackend<Scalar>::dot(storage_type<Size, 1U> const & left, storage_type<Size, 1U> const & right) noexcept
{
    return ::matrix::Vector<value_type, Size>(left).dot(right);
}

template <typename Scalar>
template <std::size_t Size>
typename Px4MatrixBackend<Scalar>::value_type
Px4MatrixBackend<Scalar>::squared_norm(storage_type<Size, 1U> const & vector) noexcept
{
    return ::matrix::Vector<value_type, Size>(vector).norm_squared();
}

template <typename Scalar>
template <std::size_t Size>
typename Px4MatrixBackend<Scalar>::value_type
Px4MatrixBackend<Scalar>::norm(storage_type<Size, 1U> const & vector) noexcept
{
    return ::matrix::Vector<value_type, Size>(vector).norm();
}

template <typename Scalar>
template <std::size_t Size, std::size_t RightColumns>
Status Px4MatrixBackend<Scalar>::solve_spd(storage_type<Size, Size> const & system,
                                           storage_type<Size, RightColumns> const & right_hand_side,
                                           storage_type<Size, RightColumns> & solution) noexcept
{
    // Checked LLT on PX4 storage. Do not use PX4's choleskyInv: this API
    // neither forms an inverse nor imposes an absolute pivot cutoff.
    // All accumulation remains in value_type (including binary64).
    storage_type<Size, Size> factor;
    for (std::size_t column = 0U; column < Size; ++column)
    {
        for (std::size_t row = column; row < Size; ++row)
        {
            value_type residual = system(row, column);
            for (std::size_t inner = 0U; inner < column; ++inner)
            {
                residual -= factor(row, inner) * factor(column, inner);
            }
            if (!scalar_math_type::is_finite(residual))
            {
                return Status::non_finite_result;
            }
            if (row == column)
            {
                if (residual <= value_type{0})
                {
                    return Status::not_positive_definite;
                }
                factor(row, column) = scalar_math_type::sqrt(residual);
            }
            else
            {
                factor(row, column) = residual / factor(column, column);
            }
            if (!scalar_math_type::is_finite(factor(row, column)))
            {
                return Status::non_finite_result;
            }
        }
    }

    // The public solve_spd wrapper passes a separate candidate and provides
    // rollback and alias safety. Reuse that candidate for both substitutions.
    for (std::size_t column = 0U; column < RightColumns; ++column)
    {
        for (std::size_t row = 0U; row < Size; ++row)
        {
            value_type residual = right_hand_side(row, column);
            for (std::size_t inner = 0U; inner < row; ++inner)
            {
                residual -= factor(row, inner) * solution(inner, column);
            }
            solution(row, column) = residual / factor(row, row);
            if (!scalar_math_type::is_finite(solution(row, column)))
            {
                return Status::non_finite_result;
            }
        }
        for (std::size_t remaining = Size; remaining > 0U; --remaining)
        {
            std::size_t const row = remaining - 1U;
            value_type residual = solution(row, column);
            for (std::size_t inner = row + 1U; inner < Size; ++inner)
            {
                residual -= factor(inner, row) * solution(inner, column);
            }
            solution(row, column) = residual / factor(row, row);
            if (!scalar_math_type::is_finite(solution(row, column)))
            {
                return Status::non_finite_result;
            }
        }
    }
    return Status::success;
}

} /* end namespace formal_eskf::linalg */
