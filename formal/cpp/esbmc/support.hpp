/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
#include <formal_eskf/so3/rotation_vector.hpp>
#include <formal_eskf/so3/unit_quaternion.hpp>

extern void __ESBMC_assume(bool condition);
extern void __ESBMC_assert(bool condition, char const * description);
extern float nondet_float();

/**
 * Contract model for the first-quadrant atan2 calls made by principal Log.
 * ESBMC 8.4 has no atan2f body.  Interior results remain nondeterministic over
 * the specified range; only the two exact axis values are fixed.
 */
extern "C" float atan2f(float y, float x) noexcept
{
    constexpr float half_pi = 1.57079632679489661923F;
    if (y == 0.0F && x > 0.0F)
    {
        return 0.0F;
    }
    if (y > 0.0F && x == 0.0F)
    {
        return half_pi;
    }

    float const result = nondet_float();
    __ESBMC_assume(result >= 0.0F && result <= half_pi);
    return result;
}

namespace formal_eskf::verification
{

// Scalar fields avoid bytewise memcpy/type-punning of floating arrays in the
// verifier. This is fixed-size value storage, with no heap or shared cells.
template <typename Number, std::size_t Size> struct ScalarArray
{
    Number head;
    ScalarArray<Number, Size - 1U> tail;
    ScalarArray() : head{}, tail{} {}
    ScalarArray(ScalarArray const & other) : head(other.head), tail(other.tail) {}
    ScalarArray & operator=(ScalarArray const & other)
    {
        head = other.head;
        tail = other.tail;
        return *this;
    }
    Number & operator[](std::size_t index)
    {
        __ESBMC_assert(index < Size, "proof storage index is in bounds");
        if (index == 0U)
        {
            return head;
        }
        return tail[index - 1U];
    }
    Number const & operator[](std::size_t index) const
    {
        __ESBMC_assert(index < Size, "proof storage index is in bounds");
        if (index == 0U)
        {
            return head;
        }
        return tail[index - 1U];
    }
};

template <typename Number> struct ScalarArray<Number, 1U>
{
    Number head;
    ScalarArray() : head{} {}
    ScalarArray(ScalarArray const & other) : head(other.head) {}
    ScalarArray & operator=(ScalarArray const & other)
    {
        head = other.head;
        return *this;
    }
    Number & operator[](std::size_t index)
    {
        __ESBMC_assert(index == 0U, "proof storage last index is zero");
        return head;
    }
    Number const & operator[](std::size_t index) const
    {
        __ESBMC_assert(index == 0U, "proof storage last index is zero");
        return head;
    }
};

template <typename Number> struct NoNormObserver
{
    template <std::size_t Size, typename Values> static void input(Values const &) noexcept {}
    static void result(Number) noexcept {}
    template <std::size_t Rows, std::size_t Inner, std::size_t Columns, typename Left, typename Right, typename Result>
    static void product(Left const &, Right const &, Result const &) noexcept
    {
    }
    template <std::size_t Size, typename Left, typename Right, typename Result>
    static void addition(Left const &, Right const &, Result const &) noexcept
    {
    }
    template <std::size_t Size, typename Values, typename Result>
    static void quotient(Values const &, Number, Result const &) noexcept
    {
    }
    template <std::size_t Size, typename Values, typename Result>
    static void scaling(Values const &, Number, Result const &) noexcept
    {
    }
};

template <typename Number, typename Observer = NoNormObserver<Number>, typename Math = scalar::StandardMath<Number>>
class FixedArrayLinalg
{
public:
    using value_type = Number;
    using scalar_math_type = Math;

    template <std::size_t Rows, std::size_t Columns>
    using matrix_type = linalg::Matrix<FixedArrayLinalg, Rows, Columns>;

    template <std::size_t Size> using vector_type = matrix_type<Size, 1U>;

    template <std::size_t Rows, std::size_t Columns> struct storage_type
    {
        ScalarArray<value_type, Rows * Columns> values;
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
    static void set_coefficient(storage_type<Rows, Columns> & matrix, std::size_t row, std::size_t column,
                                value_type value) noexcept
    {
        matrix.values[row * Columns + column] = value;
    }

    template <std::size_t Rows, std::size_t Columns>
    static void add(storage_type<Rows, Columns> const & left, storage_type<Rows, Columns> const & right,
                    storage_type<Rows, Columns> & result) noexcept
    {
        for (std::size_t index = 0U; index < Rows * Columns; ++index)
        {
            result.values[index] = left.values[index] + right.values[index];
        }
        Observer::template addition<Rows * Columns>(left.values, right.values, result.values);
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
    static void negate(storage_type<Rows, Columns> const & matrix, storage_type<Rows, Columns> & result) noexcept
    {
        for (std::size_t index = 0U; index < Rows * Columns; ++index)
        {
            result.values[index] = -matrix.values[index];
        }
    }

    template <std::size_t Rows, std::size_t Columns>
    static void scale(storage_type<Rows, Columns> const & matrix, value_type scalar,
                      storage_type<Rows, Columns> & result) noexcept
    {
        for (std::size_t index = 0U; index < Rows * Columns; ++index)
        {
            result.values[index] = matrix.values[index] * scalar;
        }
        Observer::template scaling<Rows * Columns>(matrix.values, scalar, result.values);
    }

    template <std::size_t Rows, std::size_t Columns>
    static void divide(storage_type<Rows, Columns> const & matrix, value_type scalar,
                       storage_type<Rows, Columns> & result) noexcept
    {
        for (std::size_t index = 0U; index < Rows * Columns; ++index)
        {
            result.values[index] = matrix.values[index] / scalar;
        }
        Observer::template quotient<Rows * Columns>(matrix.values, scalar, result.values);
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
        Observer::template input<Size>(vector.values);
        value_type const result = scalar_math_type::sqrt(squared_norm(vector));
        Observer::result(result);
        return result;
    }

    template <std::size_t Rows, std::size_t Inner, std::size_t Columns>
    static void multiply(storage_type<Rows, Inner> const & left, storage_type<Inner, Columns> const & right,
                         storage_type<Rows, Columns> & result) noexcept
    {
        for (std::size_t row = 0U; row < Rows; ++row)
        {
            for (std::size_t column = 0U; column < Columns; ++column)
            {
                value_type sum{0};
                for (std::size_t inner = 0U; inner < Inner; ++inner)
                {
                    sum += left.values[row * Inner + inner] * right.values[inner * Columns + column];
                }
                result.values[row * Columns + column] = sum;
            }
        }
        Observer::template product<Rows, Inner, Columns>(left.values, right.values, result.values);
    }

    template <std::size_t Rows, std::size_t Columns>
    static void transpose(storage_type<Rows, Columns> const & matrix, storage_type<Columns, Rows> & result) noexcept
    {
        for (std::size_t row = 0U; row < Rows; ++row)
        {
            for (std::size_t column = 0U; column < Columns; ++column)
            {
                result.values[column * Rows + row] = matrix.values[row * Columns + column];
            }
        }
    }
};

using ArrayLinalg = FixedArrayLinalg<float>;
using Linalg = ArrayLinalg;
using Quaternion = so3::UnitQuaternion<Linalg>;
using Scalar = Linalg::value_type;
using Vector3 = Linalg::vector_type<3U>;
using Vector4 = Linalg::vector_type<4U>;
using Matrix3 = Linalg::matrix_type<3U, 3U>;

inline void assume_scalar(Scalar value) { __ESBMC_assume(value >= Scalar{-1} && value <= Scalar{1}); }

inline void assume_vector(Vector3 const & vector)
{
    assume_scalar(vector(0U));
    assume_scalar(vector(1U));
    assume_scalar(vector(2U));
}

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

[[nodiscard]] inline bool same_vector(Vector3 const & left, Vector3 const & right) noexcept
{
    return left(0U) == right(0U) && left(1U) == right(1U) && left(2U) == right(2U);
}

} /* end namespace formal_eskf::verification */
