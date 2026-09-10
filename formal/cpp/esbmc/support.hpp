#pragma once

/**
 * Minimal fixed-array backend and symbolic-input helpers for ESBMC.
 *
 * This is proof infrastructure, not a deployment backend.  It instantiates
 * the production quaternion and SO(3) templates without asking ESBMC to model
 * Eigen internals.
 */

#include <cstddef>

#include <formal_eskf/linalg/linalg.hpp>
#include <formal_eskf/scalar/backend/standard.hpp>
#include <formal_eskf/so3/operations.hpp>
#include <formal_eskf/so3/unit_quaternion.hpp>

extern void __ESBMC_assume(bool condition);
extern void __ESBMC_assert(bool condition, char const * description);

namespace formal_eskf::verification
{

template <typename Scalar> class ArrayLinalg
{
public:
    using value_type = Scalar;
    using scalar_math_type = scalar::StandardMath<value_type>;

    template <std::size_t Rows, std::size_t Columns>
    using matrix_type = linalg::Matrix<ArrayLinalg<Scalar>, Rows, Columns>;

    template <std::size_t Size> using vector_type = matrix_type<Size, 1U>;

    template <std::size_t Rows, std::size_t Columns> struct storage_type
    {
        value_type values[Rows * Columns];
    };

    template <std::size_t Rows, std::size_t Columns> static void set_zero(storage_type<Rows, Columns> & matrix) noexcept
    {
        for (std::size_t index = 0U; index < Rows * Columns; ++index)
        {
            matrix.values[index] = value_type{0};
        }
    }

    template <std::size_t Rows, std::size_t Columns>
    [[nodiscard]] static value_type const & coefficient(storage_type<Rows, Columns> const & matrix, std::size_t row,
                                                        std::size_t column) noexcept
    {
        return matrix.values[row * Columns + column];
    }

    template <std::size_t Rows, std::size_t Columns>
    [[nodiscard]] static value_type & coefficient(storage_type<Rows, Columns> & matrix, std::size_t row,
                                                  std::size_t column) noexcept
    {
        return matrix.values[row * Columns + column];
    }

    template <std::size_t Rows, std::size_t Columns>
    static void add(storage_type<Rows, Columns> const & left, storage_type<Rows, Columns> const & right,
                    storage_type<Rows, Columns> & result) noexcept
    {
        for (std::size_t index = 0U; index < Rows * Columns; ++index)
        {
            result.values[index] = left.values[index] + right.values[index];
        }
    }

    template <std::size_t Rows, std::size_t Columns>
    static void subtract(storage_type<Rows, Columns> const & left, storage_type<Rows, Columns> const & right,
                         storage_type<Rows, Columns> & result) noexcept
    {
        for (std::size_t index = 0U; index < Rows * Columns; ++index)
        {
            result.values[index] = left.values[index] - right.values[index];
        }
    }

    template <std::size_t Rows, std::size_t Columns>
    static void divide(storage_type<Rows, Columns> const & matrix, value_type scalar,
                       storage_type<Rows, Columns> & result) noexcept
    {
        for (std::size_t index = 0U; index < Rows * Columns; ++index)
        {
            result.values[index] = matrix.values[index] / scalar;
        }
    }

    template <std::size_t Size>
    [[nodiscard]] static value_type dot(storage_type<Size, 1U> const & left,
                                        storage_type<Size, 1U> const & right) noexcept
    {
        value_type result{0};
        for (std::size_t index = 0U; index < Size; ++index)
        {
            result += left.values[index] * right.values[index];
        }
        return result;
    }

    template <std::size_t Size>
    [[nodiscard]] static value_type squared_norm(storage_type<Size, 1U> const & vector) noexcept
    {
        return dot(vector, vector);
    }

    template <std::size_t Size> [[nodiscard]] static value_type norm(storage_type<Size, 1U> const & vector) noexcept
    {
        return scalar_math_type::sqrt(squared_norm(vector));
    }
};

using Linalg = ArrayLinalg<float>;
using Quaternion = so3::UnitQuaternion<Linalg>;
using Scalar = Linalg::value_type;
using Vector3 = Linalg::vector_type<3U>;
using Matrix3 = Linalg::matrix_type<3U, 3U>;

inline void assume_scalar(Scalar value) { __ESBMC_assume(value >= Scalar{-1} && value <= Scalar{1}); }

inline void assume_quaternion(Quaternion const & quaternion)
{
    assume_scalar(quaternion.q0());
    assume_scalar(quaternion.q1());
    assume_scalar(quaternion.q2());
    assume_scalar(quaternion.q3());
}

[[nodiscard]] inline bool same_coefficients(Quaternion const & left, Quaternion const & right) noexcept
{
    return left.q0() == right.q0() && left.q1() == right.q1() && left.q2() == right.q2() && left.q3() == right.q3();
}

[[nodiscard]] inline bool is_negative_identity(Quaternion const & quaternion) noexcept
{
    return quaternion.q0() == Scalar{-1} && quaternion.q1() == Scalar{0} && quaternion.q2() == Scalar{0} &&
           quaternion.q3() == Scalar{0};
}

} /* end namespace formal_eskf::verification */
