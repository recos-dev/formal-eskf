#pragma once

/**
 * @file
 * Eigen implementation of the fixed-size linear algebra backend.
 */

#include <cstddef>
#include <type_traits>

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <formal_eskf/linalg/linalg.hpp>
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

    template <std::size_t Rows, std::size_t Columns> using matrix_type = Matrix<EigenBackend<Scalar>, Rows, Columns>;

    template <std::size_t Size> using vector_type = matrix_type<Size, 1U>;

    template <std::size_t Rows, std::size_t Columns>
    using storage_type = Eigen::Matrix<value_type, static_cast<int>(Rows), static_cast<int>(Columns)>;

    template <std::size_t Rows, std::size_t Columns>
    static void set_zero(storage_type<Rows, Columns> & matrix) noexcept;

    template <std::size_t Rows, std::size_t Columns>
    [[nodiscard]] static value_type const & coefficient(storage_type<Rows, Columns> const & matrix, std::size_t row,
                                                        std::size_t column) noexcept;

    template <std::size_t Rows, std::size_t Columns>
    [[nodiscard]] static value_type & coefficient(storage_type<Rows, Columns> & matrix, std::size_t row,
                                                  std::size_t column) noexcept;

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

}; /* end class EigenBackend */

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void EigenBackend<Scalar>::set_zero(storage_type<Rows, Columns> & matrix) noexcept
{
    matrix.setZero();
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
typename EigenBackend<Scalar>::value_type const &
EigenBackend<Scalar>::coefficient(storage_type<Rows, Columns> const & matrix, std::size_t row,
                                  std::size_t column) noexcept
{
    return matrix(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column));
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
typename EigenBackend<Scalar>::value_type &
EigenBackend<Scalar>::coefficient(storage_type<Rows, Columns> & matrix, std::size_t row, std::size_t column) noexcept
{
    return matrix(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column));
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void EigenBackend<Scalar>::add(storage_type<Rows, Columns> const & left, storage_type<Rows, Columns> const & right,
                               storage_type<Rows, Columns> & result) noexcept
{
    result = left + right;
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void EigenBackend<Scalar>::subtract(storage_type<Rows, Columns> const & left, storage_type<Rows, Columns> const & right,
                                    storage_type<Rows, Columns> & result) noexcept
{
    result = left - right;
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void EigenBackend<Scalar>::negate(storage_type<Rows, Columns> const & matrix,
                                  storage_type<Rows, Columns> & result) noexcept
{
    result = -matrix;
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void EigenBackend<Scalar>::scale(storage_type<Rows, Columns> const & matrix, value_type scalar,
                                 storage_type<Rows, Columns> & result) noexcept
{
    result = matrix * scalar;
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void EigenBackend<Scalar>::divide(storage_type<Rows, Columns> const & matrix, value_type scalar,
                                  storage_type<Rows, Columns> & result) noexcept
{
    result = matrix / scalar;
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Inner, std::size_t Columns>
void EigenBackend<Scalar>::multiply(storage_type<Rows, Inner> const & left, storage_type<Inner, Columns> const & right,
                                    storage_type<Rows, Columns> & result) noexcept
{
    result.noalias() = left * right;
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void EigenBackend<Scalar>::transpose(storage_type<Rows, Columns> const & matrix,
                                     storage_type<Columns, Rows> & result) noexcept
{
    result = matrix.transpose();
}

template <typename Scalar>
template <std::size_t Size>
typename EigenBackend<Scalar>::value_type EigenBackend<Scalar>::dot(storage_type<Size, 1U> const & left,
                                                                    storage_type<Size, 1U> const & right) noexcept
{
    return left.dot(right);
}

template <typename Scalar>
template <std::size_t Size>
typename EigenBackend<Scalar>::value_type
EigenBackend<Scalar>::squared_norm(storage_type<Size, 1U> const & vector) noexcept
{
    return vector.squaredNorm();
}

template <typename Scalar>
template <std::size_t Size>
typename EigenBackend<Scalar>::value_type EigenBackend<Scalar>::norm(storage_type<Size, 1U> const & vector) noexcept
{
    return vector.norm();
}

template <typename Scalar>
template <std::size_t Size, std::size_t RightColumns>
Status EigenBackend<Scalar>::solve_spd(storage_type<Size, Size> const & system,
                                       storage_type<Size, RightColumns> const & right_hand_side,
                                       storage_type<Size, RightColumns> & solution) noexcept
{
    using system_storage_type = storage_type<Size, Size>;
    Eigen::LLT<system_storage_type> const factorization(system);
    if (factorization.info() != Eigen::Success)
    {
        return Status::not_positive_definite;
    }

    solution = factorization.solve(right_hand_side);
    return Status::success;
}

} /* end namespace formal_eskf::linalg */
