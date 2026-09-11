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

namespace detail
{

/** Pure coefficient step used after a checked norm has been established. */
template <typename Linalg, std::size_t Size>
[[nodiscard]] Matrix<Linalg, Size, 1U>
normalization_candidate(Matrix<Linalg, Size, 1U> const & input, typename Linalg::value_type input_norm) noexcept
{
    return input / input_norm;
}

} /* end namespace detail */

template <typename Linalg, std::size_t Rows, std::size_t Columns>
[[nodiscard]] Matrix<Linalg, Columns, Rows> transpose(Matrix<Linalg, Rows, Columns> const & matrix) noexcept
{
    Matrix<Linalg, Columns, Rows> result;
    Linalg::template transpose<Rows, Columns>(detail::MatrixAccess::storage(matrix),
                                              detail::MatrixAccess::storage(result));
    return result;
}

template <typename Linalg, std::size_t Size>
[[nodiscard]] typename Linalg::value_type dot(Matrix<Linalg, Size, 1U> const & left,
                                              Matrix<Linalg, Size, 1U> const & right) noexcept
{
    return Linalg::template dot<Size>(detail::MatrixAccess::storage(left), detail::MatrixAccess::storage(right));
}

template <typename Linalg, std::size_t Size>
[[nodiscard]] typename Linalg::value_type squared_norm(Matrix<Linalg, Size, 1U> const & vector) noexcept
{
    return Linalg::template squared_norm<Size>(detail::MatrixAccess::storage(vector));
}

template <typename Linalg, std::size_t Size>
[[nodiscard]] typename Linalg::value_type norm(Matrix<Linalg, Size, 1U> const & vector) noexcept
{
    return Linalg::template norm<Size>(detail::MatrixAccess::storage(vector));
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
[[nodiscard]] bool all_finite(Matrix<Linalg, Rows, Columns> const & matrix) noexcept
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
[[nodiscard]] Status try_normalize(Matrix<Linalg, Size, 1U> const & input, typename Linalg::value_type minimum_norm,
                                   Matrix<Linalg, Size, 1U> & output) noexcept
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

    Matrix<Linalg, Size, 1U> const candidate = detail::normalization_candidate(input, input_norm);
    if (!all_finite(candidate))
    {
        return Status::non_finite_result;
    }

    for (std::size_t index = 0U; index < Size; ++index)
    {
        output.set(index, candidate(index));
    }
    return Status::success;
}

template <typename Linalg>
[[nodiscard]] Matrix<Linalg, 3U, 1U> cross(Matrix<Linalg, 3U, 1U> const & left,
                                           Matrix<Linalg, 3U, 1U> const & right) noexcept
{
    Matrix<Linalg, 3U, 1U> result;
    result(0U) = left(1U) * right(2U) - left(2U) * right(1U);
    result(1U) = left(2U) * right(0U) - left(0U) * right(2U);
    result(2U) = left(0U) * right(1U) - left(1U) * right(0U);
    return result;
}

template <typename Linalg, std::size_t Size>
[[nodiscard]] typename Linalg::value_type trace(Matrix<Linalg, Size, Size> const & matrix) noexcept
{
    typename Linalg::value_type result{0};
    for (std::size_t index = 0U; index < Size; ++index)
    {
        result += matrix(index, index);
    }
    return result;
}

template <typename Linalg, std::size_t Size>
[[nodiscard]] Matrix<Linalg, Size, 1U> diagonal(Matrix<Linalg, Size, Size> const & matrix) noexcept
{
    Matrix<Linalg, Size, 1U> result;
    for (std::size_t index = 0U; index < Size; ++index)
    {
        result(index) = matrix(index, index);
    }
    return result;
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
[[nodiscard]] typename Linalg::value_type max_abs(Matrix<Linalg, Rows, Columns> const & matrix) noexcept
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
[[nodiscard]] Matrix<Linalg, Size, Size> symmetrize(Matrix<Linalg, Size, Size> const & matrix) noexcept
{
    return typename Linalg::value_type{0.5} * (matrix + transpose(matrix));
}

template <typename Linalg, std::size_t Size>
[[nodiscard]] bool is_symmetric(Matrix<Linalg, Size, Size> const & matrix,
                                typename Linalg::value_type tolerance) noexcept
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
[[nodiscard]] Matrix<Linalg, Rows, Rows> sandwich(Matrix<Linalg, Rows, Inner> const & transform,
                                                  Matrix<Linalg, Inner, Inner> const & matrix) noexcept
{
    return transform * matrix * transpose(transform);
}

} /* end namespace formal_eskf::linalg */
