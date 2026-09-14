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
 * CMSIS-DSP implementation of the fixed-size linear algebra backend.
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

#include <dsp/basic_math_functions.h>
#include <dsp/matrix_functions.h>

#include <formal_eskf/linalg/linalg.hpp>
#include <formal_eskf/scalar/backend/standard.hpp>

namespace formal_eskf::linalg
{

namespace detail
{

// Precision dispatch only. CMSIS-DSP selects scalar/MVE/Neon kernels in its
// own build; the adapter never reimplements that platform selection.
template <typename Scalar> struct CmsisDspApi;

template <> struct CmsisDspApi<float>
{
    using matrix_type = arm_matrix_instance_f32;
    static constexpr auto add = arm_add_f32;
    static constexpr auto subtract = arm_sub_f32;
    static constexpr auto negate = arm_negate_f32;
    static constexpr auto scale = arm_scale_f32;
    static constexpr auto multiply = arm_mat_mult_f32;
    static constexpr auto transpose = arm_mat_trans_f32;
    static constexpr auto dot = arm_dot_prod_f32;
    static constexpr auto cholesky = arm_mat_cholesky_f32;
    static constexpr auto solve_lower = arm_mat_solve_lower_triangular_f32;
    static constexpr auto solve_upper = arm_mat_solve_upper_triangular_f32;
};

template <> struct CmsisDspApi<double>
{
    using matrix_type = arm_matrix_instance_f64;
    static constexpr auto add = arm_add_f64;
    static constexpr auto subtract = arm_sub_f64;
    static constexpr auto negate = arm_negate_f64;
    static constexpr auto scale = arm_scale_f64;
    static constexpr auto multiply = arm_mat_mult_f64;
    static constexpr auto transpose = arm_mat_trans_f64;
    static constexpr auto dot = arm_dot_prod_f64;
    static constexpr auto cholesky = arm_mat_cholesky_f64;
    static constexpr auto solve_lower = arm_mat_solve_lower_triangular_f64;
    static constexpr auto solve_upper = arm_mat_solve_upper_triangular_f64;
};

} /* end namespace detail */

/**
 * Fixed-size backend using externally supplied CMSIS-DSP C kernels.
 *
 * The embedding build owns CMSIS-DSP, CPU/FPU/ABI flags and SIMD selection.
 * It must qualify that build against the selected floating-point profile;
 * enabling this adapter does not enable fast-math or certify SIMD kernels.
 *
 * Storage owns its coefficients. Non-owning CMSIS descriptors are created
 * only for a call, so copying/moving a Matrix never retains source pointers.
 * Arithmetic and LLT solves remain in the selected float/double precision.
 */
template <typename Scalar> class CmsisDspBackend
{
    static_assert(std::is_same_v<Scalar, float> || std::is_same_v<Scalar, double>);
    using api_type = detail::CmsisDspApi<Scalar>;
    using descriptor_type = typename api_type::matrix_type;

public:
    using value_type = Scalar;
    using scalar_math_type = scalar::StandardMath<value_type>;

    template <std::size_t Rows, std::size_t Columns> using matrix_type = Matrix<CmsisDspBackend<Scalar>, Rows, Columns>;

    template <std::size_t Size> using vector_type = matrix_type<Size, 1U>;

    template <std::size_t Rows, std::size_t Columns> struct alignas(16) storage_type
    {
        static_assert(Rows > 0U && Rows <= std::numeric_limits<std::uint16_t>::max());
        static_assert(Columns > 0U && Columns <= std::numeric_limits<std::uint16_t>::max());
        // Matrix kernels also use signed int for flattened indices.
        static_assert(Rows * Columns <= static_cast<std::size_t>(std::numeric_limits<int>::max()));

        // CMSIS-DSP documents up to three extra 32-bit words read by vector
        // kernels. Keep this padding inside the initialized scalar array, not
        // merely adjacent memory. Layout is independent of consumer macros.
        static constexpr std::size_t padding_size =
            (3U * sizeof(std::uint32_t) + sizeof(value_type) - 1U) / sizeof(value_type);
        std::array<value_type, Rows * Columns + padding_size> values{};
    };

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

private:
    template <std::size_t Rows, std::size_t Columns>
    [[nodiscard]] static descriptor_type descriptor(storage_type<Rows, Columns> & matrix) noexcept;

    template <std::size_t Rows, std::size_t Columns>
    [[nodiscard]] static descriptor_type descriptor(storage_type<Rows, Columns> const & matrix) noexcept;

    template <std::size_t Rows, std::size_t Columns>
    [[nodiscard]] static bool finite(storage_type<Rows, Columns> const & matrix) noexcept;

}; /* end class CmsisDspBackend */

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void CmsisDspBackend<Scalar>::set_zero(storage_type<Rows, Columns> & matrix) noexcept
{
    matrix.values.fill(value_type{0});
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
typename CmsisDspBackend<Scalar>::value_type const &
CmsisDspBackend<Scalar>::coefficient(storage_type<Rows, Columns> const & matrix, std::size_t row,
                                     std::size_t column) noexcept
{
    return matrix.values[row * Columns + column];
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
typename CmsisDspBackend<Scalar>::value_type &
CmsisDspBackend<Scalar>::coefficient(storage_type<Rows, Columns> & matrix, std::size_t row, std::size_t column) noexcept
{
    return matrix.values[row * Columns + column];
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void CmsisDspBackend<Scalar>::set_coefficient(storage_type<Rows, Columns> & matrix, std::size_t row, std::size_t column,
                                              value_type value) noexcept
{
    coefficient(matrix, row, column) = value;
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void CmsisDspBackend<Scalar>::add(storage_type<Rows, Columns> const & left, storage_type<Rows, Columns> const & right,
                                  storage_type<Rows, Columns> & result) noexcept
{
    api_type::add(left.values.data(), right.values.data(), result.values.data(),
                  static_cast<std::uint32_t>(Rows * Columns));
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void CmsisDspBackend<Scalar>::subtract(storage_type<Rows, Columns> const & left,
                                       storage_type<Rows, Columns> const & right,
                                       storage_type<Rows, Columns> & result) noexcept
{
    api_type::subtract(left.values.data(), right.values.data(), result.values.data(),
                       static_cast<std::uint32_t>(Rows * Columns));
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void CmsisDspBackend<Scalar>::negate(storage_type<Rows, Columns> const & matrix,
                                     storage_type<Rows, Columns> & result) noexcept
{
    api_type::negate(matrix.values.data(), result.values.data(), static_cast<std::uint32_t>(Rows * Columns));
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void CmsisDspBackend<Scalar>::scale(storage_type<Rows, Columns> const & matrix, value_type scalar,
                                    storage_type<Rows, Columns> & result) noexcept
{
    api_type::scale(matrix.values.data(), scalar, result.values.data(), static_cast<std::uint32_t>(Rows * Columns));
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void CmsisDspBackend<Scalar>::divide(storage_type<Rows, Columns> const & matrix, value_type scalar,
                                     storage_type<Rows, Columns> & result) noexcept
{
    // Scaling by 1/scalar can overflow even when each quotient is finite.
    for (std::size_t index = 0U; index < Rows * Columns; ++index)
    {
        result.values[index] = matrix.values[index] / scalar;
    }
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Inner, std::size_t Columns>
void CmsisDspBackend<Scalar>::multiply(storage_type<Rows, Inner> const & left,
                                       storage_type<Inner, Columns> const & right,
                                       storage_type<Rows, Columns> & result) noexcept
{
    auto const left_descriptor = descriptor(left);
    auto const right_descriptor = descriptor(right);
    auto result_descriptor = descriptor(result);
    // The only error is a size mismatch, excluded by these template dimensions.
    // Shared Matrix arithmetic supplies a distinct result (no input alias).
    (void)api_type::multiply(&left_descriptor, &right_descriptor, &result_descriptor);
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
void CmsisDspBackend<Scalar>::transpose(storage_type<Rows, Columns> const & matrix,
                                        storage_type<Columns, Rows> & result) noexcept
{
    auto const input_descriptor = descriptor(matrix);
    auto result_descriptor = descriptor(result);
    // Matching dimensions and separate output are supplied by the shared layer.
    (void)api_type::transpose(&input_descriptor, &result_descriptor);
}

template <typename Scalar>
template <std::size_t Size>
typename CmsisDspBackend<Scalar>::value_type CmsisDspBackend<Scalar>::dot(storage_type<Size, 1U> const & left,
                                                                          storage_type<Size, 1U> const & right) noexcept
{
    value_type result{0};
    api_type::dot(left.values.data(), right.values.data(), static_cast<std::uint32_t>(Size), &result);
    return result;
}

template <typename Scalar>
template <std::size_t Size>
typename CmsisDspBackend<Scalar>::value_type
CmsisDspBackend<Scalar>::squared_norm(storage_type<Size, 1U> const & vector) noexcept
{
    return dot(vector, vector);
}

template <typename Scalar>
template <std::size_t Size>
typename CmsisDspBackend<Scalar>::value_type
CmsisDspBackend<Scalar>::norm(storage_type<Size, 1U> const & vector) noexcept
{
    return scalar_math_type::sqrt(squared_norm(vector));
}

template <typename Scalar>
template <std::size_t Size, std::size_t RightColumns>
Status CmsisDspBackend<Scalar>::solve_spd(storage_type<Size, Size> const & system,
                                          storage_type<Size, RightColumns> const & right_hand_side,
                                          storage_type<Size, RightColumns> & solution) noexcept
{
    // CMSIS Cholesky only writes the lower triangle; storage initializes the
    // rest to zero. Factor once for all RHS, then solve L*Y=B and L^T*X=Y.
    storage_type<Size, Size> factor;
    auto const system_descriptor = descriptor(system);
    auto factor_descriptor = descriptor(factor);
    arm_status const factor_status = api_type::cholesky(&system_descriptor, &factor_descriptor);
    // CMSIS success alone does not exclude overflow/NaN. Check this even on
    // decomposition failure, before mapping a finite nonpositive pivot.
    if (!finite(factor))
    {
        return Status::non_finite_result;
    }
    if (factor_status != ARM_MATH_SUCCESS)
    {
        return Status::not_positive_definite;
    }
    for (std::size_t index = 0U; index < Size; ++index)
    {
        if (coefficient(factor, index, index) <= value_type{0})
        {
            return Status::not_positive_definite;
        }
    }

    storage_type<Size, RightColumns> intermediate;
    auto const rhs_descriptor = descriptor(right_hand_side);
    auto intermediate_descriptor = descriptor(intermediate);
    arm_status const lower_status =
        api_type::solve_lower(&factor_descriptor, &rhs_descriptor, &intermediate_descriptor);
    if (!finite(intermediate))
    {
        return Status::non_finite_result;
    }
    if (lower_status != ARM_MATH_SUCCESS)
    {
        return Status::not_positive_definite;
    }

    storage_type<Size, Size> upper;
    transpose(factor, upper);
    auto const upper_descriptor = descriptor(upper);
    auto solution_descriptor = descriptor(solution);
    arm_status const upper_status =
        api_type::solve_upper(&upper_descriptor, &intermediate_descriptor, &solution_descriptor);
    if (!finite(solution))
    {
        return Status::non_finite_result;
    }
    // The public wrapper provides finite-input validation, rollback and alias
    // safety by passing a separate solution candidate.
    return upper_status == ARM_MATH_SUCCESS ? Status::success : Status::not_positive_definite;
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
typename CmsisDspBackend<Scalar>::descriptor_type
CmsisDspBackend<Scalar>::descriptor(storage_type<Rows, Columns> & matrix) noexcept
{
    return {static_cast<std::uint16_t>(Rows), static_cast<std::uint16_t>(Columns), matrix.values.data()};
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
typename CmsisDspBackend<Scalar>::descriptor_type
CmsisDspBackend<Scalar>::descriptor(storage_type<Rows, Columns> const & matrix) noexcept
{
    // CMSIS input descriptors have shallow constness: pData is Scalar* even
    // for read-only inputs. Only pass this view to the read-only source
    // parameters of multiply, transpose, Cholesky and triangular solves.
    return {static_cast<std::uint16_t>(Rows), static_cast<std::uint16_t>(Columns),
            const_cast<value_type *>(matrix.values.data())};
}

template <typename Scalar>
template <std::size_t Rows, std::size_t Columns>
bool CmsisDspBackend<Scalar>::finite(storage_type<Rows, Columns> const & matrix) noexcept
{
    for (std::size_t index = 0U; index < Rows * Columns; ++index)
    {
        if (!scalar_math_type::is_finite(matrix.values[index]))
        {
            return false;
        }
    }
    return true;
}

} /* end namespace formal_eskf::linalg */
