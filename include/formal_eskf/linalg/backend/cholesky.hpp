/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <cstddef>

#include <formal_eskf/status.hpp>

namespace formal_eskf::linalg
{

/**
 * Checked LLT implementation helpers, not the public solve interface.
 *
 * Read-only inputs and workspace are disjoint. Only the lower triangle of the factor
 * is consumed or written. Substitution requires a successfully computed
 * factor and column < RightColumns; backward follows forward on that column.
 * Workspace may be partially written on failure. The public solve wrapper
 * owns validation, rollback and alias safety.
 *
 * Accumulations stay in value_type, with no inverse or pivot-size cutoff.
 */
template <typename Backend> class Cholesky
{
    using value_type = typename Backend::value_type;
    using scalar_math_type = typename Backend::scalar_math_type;
    template <std::size_t Rows, std::size_t Columns>
    using storage_type = typename Backend::template storage_type<Rows, Columns>;

public:
    // Explicit scalar steps keep the recurrence's evaluation order visible.
    // In particular, subtract_product is not a fused multiply-add.
    [[nodiscard]] static value_type subtract_product(value_type residual, value_type left, value_type right) noexcept
    {
        return residual - left * right;
    }

    [[nodiscard]] static value_type quotient(value_type numerator, value_type denominator) noexcept
    {
        return numerator / denominator;
    }

    template <std::size_t Size, std::size_t RightColumns>
    [[nodiscard]] static Status solve(storage_type<Size, Size> const & system,
                                      storage_type<Size, RightColumns> const & right_hand_side,
                                      storage_type<Size, RightColumns> & solution) noexcept
    {
        // Keep each column's forward/backward pair together: failure order and
        // partially written scratch storage must not depend on this decomposition.
        storage_type<Size, Size> factor;
        Status const factor_status = factorize<Size>(system, factor);
        if (!succeeded(factor_status))
        {
            return factor_status;
        }
        for (std::size_t column = 0U; column < RightColumns; ++column)
        {
            Status const forward_status =
                forward_substitute<Size, RightColumns>(factor, right_hand_side, solution, column);
            if (!succeeded(forward_status))
            {
                return forward_status;
            }
            Status const backward_status = backward_substitute<Size, RightColumns>(factor, solution, column);
            if (!succeeded(backward_status))
            {
                return backward_status;
            }
        }
        return Status::success;
    }

    template <std::size_t Size>
    [[nodiscard]] static Status factorize(storage_type<Size, Size> const & system,
                                          storage_type<Size, Size> & factor) noexcept
    {
        for (std::size_t column = 0U; column < Size; ++column)
        {
            for (std::size_t row = column; row < Size; ++row)
            {
                value_type residual = Backend::coefficient(system, row, column);
                for (std::size_t inner = 0U; inner < column; ++inner)
                {
                    residual = subtract_product(residual, Backend::coefficient(factor, row, inner),
                                                Backend::coefficient(factor, column, inner));
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
                    Backend::coefficient(factor, row, column) = scalar_math_type::sqrt(residual);
                }
                else
                {
                    Backend::coefficient(factor, row, column) =
                        quotient(residual, Backend::coefficient(factor, column, column));
                }
                if (!scalar_math_type::is_finite(Backend::coefficient(factor, row, column)))
                {
                    return Status::non_finite_result;
                }
            }
        }
        return Status::success;
    }

    template <std::size_t Size, std::size_t RightColumns>
    [[nodiscard]] static Status forward_substitute(storage_type<Size, Size> const & factor,
                                                   storage_type<Size, RightColumns> const & right_hand_side,
                                                   storage_type<Size, RightColumns> & solution,
                                                   std::size_t column) noexcept
    {
        for (std::size_t row = 0U; row < Size; ++row)
        {
            value_type residual = Backend::coefficient(right_hand_side, row, column);
            for (std::size_t inner = 0U; inner < row; ++inner)
            {
                residual = subtract_product(residual, Backend::coefficient(factor, row, inner),
                                            Backend::coefficient(solution, inner, column));
            }
            Backend::coefficient(solution, row, column) = quotient(residual, Backend::coefficient(factor, row, row));
            if (!scalar_math_type::is_finite(Backend::coefficient(solution, row, column)))
            {
                return Status::non_finite_result;
            }
        }
        return Status::success;
    }

    template <std::size_t Size, std::size_t RightColumns>
    [[nodiscard]] static Status backward_substitute(storage_type<Size, Size> const & factor,
                                                    storage_type<Size, RightColumns> & solution,
                                                    std::size_t column) noexcept
    {
        for (std::size_t remaining = Size; remaining > 0U; --remaining)
        {
            std::size_t const row = remaining - 1U;
            value_type residual = Backend::coefficient(solution, row, column);
            for (std::size_t inner = row + 1U; inner < Size; ++inner)
            {
                residual = subtract_product(residual, Backend::coefficient(factor, inner, row),
                                            Backend::coefficient(solution, inner, column));
            }
            Backend::coefficient(solution, row, column) = quotient(residual, Backend::coefficient(factor, row, row));
            if (!scalar_math_type::is_finite(Backend::coefficient(solution, row, column)))
            {
                return Status::non_finite_result;
            }
        }
        return Status::success;
    }
};

} // namespace formal_eskf::linalg
