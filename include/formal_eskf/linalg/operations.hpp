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
 * Backend-independent fixed-size linear algebra operations.
 */

#include <cstddef>

#include <formal_eskf/linalg/matrix.hpp>
#include <formal_eskf/scalar/math.hpp>
#include <formal_eskf/status.hpp>

namespace formal_eskf::linalg
{

template <typename Linalg, std::size_t Rows, std::size_t Columns>
Matrix<Linalg, Columns, Rows> transpose(Matrix<Linalg, Rows, Columns> const & matrix)
{
    Matrix<Linalg, Columns, Rows> result;
    Linalg::template transpose<Rows, Columns>(detail::MatrixAccess::storage(matrix),
                                              detail::MatrixAccess::storage(result));
    return result;
}

template <typename Linalg, std::size_t Size>
typename Linalg::value_type dot(Vector<Linalg, Size> const & left, Vector<Linalg, Size> const & right)
{
    return Linalg::template dot<Size>(detail::MatrixAccess::storage(left), detail::MatrixAccess::storage(right));
}

template <typename Linalg, std::size_t Size>
typename Linalg::value_type squared_norm(Vector<Linalg, Size> const & vector)
{
    return Linalg::template squared_norm<Size>(detail::MatrixAccess::storage(vector));
}

template <typename Linalg, std::size_t Size> typename Linalg::value_type norm(Vector<Linalg, Size> const & vector)
{
    return Linalg::template norm<Size>(detail::MatrixAccess::storage(vector));
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
bool all_finite(Matrix<Linalg, Rows, Columns> const & matrix)
{
    using scalar_math_type = typename Linalg::scalar_math_type;

    for (std::size_t row = 0U; row < Rows; ++row)
    {
        for (std::size_t column = 0U; column < Columns; ++column)
        {
            if (!scalar::is_finite<scalar_math_type>(matrix(row, column)))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename Linalg, std::size_t Size>
[[nodiscard]] Status normalize(Vector<Linalg, Size> const & input, typename Linalg::value_type minimum_norm,
                               Vector<Linalg, Size> & output)
{
    using value_type = typename Linalg::value_type;
    using scalar_math_type = typename Linalg::scalar_math_type;

    if (!all_finite(input) || !scalar::is_finite<scalar_math_type>(minimum_norm))
    {
        return Status::non_finite_input;
    }

    value_type const input_norm = norm(input);
    if (!scalar::is_finite<scalar_math_type>(input_norm))
    {
        return Status::non_finite_result;
    }
    if (!(minimum_norm > value_type{0}) || input_norm < minimum_norm)
    {
        return Status::zero_or_unsafe_divisor;
    }

    Vector<Linalg, Size> const candidate = input / input_norm;
    if (!all_finite(candidate))
    {
        return Status::non_finite_result;
    }

    for (std::size_t index = 0U; index < Size; ++index)
    {
        output(index) = candidate(index);
    }
    return Status::success;
}

template <typename Linalg> Vector<Linalg, 3U> cross(Vector<Linalg, 3U> const & left, Vector<Linalg, 3U> const & right)
{
    Vector<Linalg, 3U> result;
    result(0U) = left(1U) * right(2U) - left(2U) * right(1U);
    result(1U) = left(2U) * right(0U) - left(0U) * right(2U);
    result(2U) = left(0U) * right(1U) - left(1U) * right(0U);
    return result;
}

template <typename Linalg, std::size_t Size>
typename Linalg::value_type trace(Matrix<Linalg, Size, Size> const & matrix)
{
    typename Linalg::value_type result{0};
    for (std::size_t index = 0U; index < Size; ++index)
    {
        result += matrix(index, index);
    }
    return result;
}

template <typename Linalg, std::size_t Size> Vector<Linalg, Size> diagonal(Matrix<Linalg, Size, Size> const & matrix)
{
    Vector<Linalg, Size> result;
    for (std::size_t index = 0U; index < Size; ++index)
    {
        result(index) = matrix(index, index);
    }
    return result;
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
typename Linalg::value_type max_abs(Matrix<Linalg, Rows, Columns> const & matrix)
{
    using value_type = typename Linalg::value_type;
    using scalar_math_type = typename Linalg::scalar_math_type;
    value_type result{0};
    for (std::size_t row = 0U; row < Rows; ++row)
    {
        for (std::size_t column = 0U; column < Columns; ++column)
        {
            value_type const magnitude = scalar::absolute<scalar_math_type>(matrix(row, column));
            if (!scalar::is_finite<scalar_math_type>(magnitude))
            {
                return magnitude;
            }
            if (magnitude > result)
            {
                result = magnitude;
            }
        }
    }
    return result;
}

template <typename Linalg, std::size_t Size>
Matrix<Linalg, Size, Size> symmetrize(Matrix<Linalg, Size, Size> const & matrix)
{
    return typename Linalg::value_type{0.5} * (matrix + transpose(matrix));
}

template <typename Linalg, std::size_t Size>
bool is_symmetric(Matrix<Linalg, Size, Size> const & matrix, typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    using scalar_math_type = typename Linalg::scalar_math_type;
    if (!all_finite(matrix) || !scalar::is_finite<scalar_math_type>(tolerance) || tolerance < value_type{0})
    {
        return false;
    }
    return max_abs(matrix - transpose(matrix)) <= tolerance;
}

template <typename Linalg, std::size_t Rows, std::size_t Inner>
Matrix<Linalg, Rows, Rows> sandwich(Matrix<Linalg, Rows, Inner> const & transform,
                                    Matrix<Linalg, Inner, Inner> const & matrix)
{
    return transform * matrix * transpose(transform);
}

} /* end namespace formal_eskf::linalg */
