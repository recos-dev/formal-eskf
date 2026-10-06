/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <cstddef>
#include <limits>

#include <formal_eskf/linalg/operations.hpp>
#include <formal_eskf/scalar/math.hpp>
#include <formal_eskf/status.hpp>

namespace formal_eskf
{

/** Diagnostics are valid only after a successful try_check_innovation call. */
template <typename Linalg, std::size_t Size> struct InnovationCheck
{
    linalg::Matrix<Linalg, Size, 1U> test_ratio{};
    bool rejected = true;
};

/**
 * Check r[i]^2 / (gate_sigma^2 * S[i,i]) against one, independently per axis.
 * The caller supplies diag(S), where S = H P H^T + V, not measurement noise V.
 * This is a componentwise gate, not a joint Mahalanobis test or an SPD check.
 *
 * Status::success means diagnostics were computed, not that fusion is allowed:
 * any ratio > 1 rejects the entire observation; equality is accepted.
 * No state or covariance is changed and correction does not call this implicitly.
 * On failure, output is unchanged. Its vector may alias either complete input.
 * See E-INNOV for numerical bounds and failure semantics.
 */
template <typename Linalg, std::size_t Size>
[[nodiscard]] Status try_check_innovation(linalg::Matrix<Linalg, Size, 1U> const & innovation,
                                          linalg::Matrix<Linalg, Size, 1U> const & innovation_variance,
                                          typename Linalg::value_type gate_sigma,
                                          InnovationCheck<Linalg, Size> & output) noexcept
{
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    constexpr auto minimum_normal = std::numeric_limits<value_type>::min();

    if (!scalar::is_finite<math_type>(gate_sigma) || !linalg::all_finite(innovation) ||
        !linalg::all_finite(innovation_variance))
    {
        return Status::non_finite_input;
    }
    if (gate_sigma <= value_type{0})
    {
        return Status::domain_error;
    }
    for (std::size_t axis = 0U; axis < Size; ++axis)
    {
        if (innovation_variance(axis) <= value_type{0})
        {
            return Status::domain_error;
        }
        if (innovation_variance(axis) < minimum_normal)
        {
            return Status::zero_or_unsafe_divisor;
        }
    }

    auto const gate_squared = gate_sigma * gate_sigma;
    if (!scalar::is_finite<math_type>(gate_squared))
    {
        return Status::non_finite_result;
    }
    if (gate_squared < minimum_normal)
    {
        return Status::zero_or_unsafe_divisor;
    }

    InnovationCheck<Linalg, Size> candidate;
    candidate.rejected = false;
    for (std::size_t axis = 0U; axis < Size; ++axis)
    {
        auto const innovation_squared = innovation(axis) * innovation(axis);
        auto const denominator = gate_squared * innovation_variance(axis);
        if (!scalar::is_finite<math_type>(innovation_squared) || !scalar::is_finite<math_type>(denominator))
        {
            return Status::non_finite_result;
        }
        if (denominator < minimum_normal)
        {
            return Status::zero_or_unsafe_divisor;
        }
        auto const ratio = innovation_squared / denominator;
        if (!scalar::is_finite<math_type>(ratio))
        {
            return Status::non_finite_result;
        }
        candidate.test_ratio.set(axis, ratio);
        candidate.rejected = candidate.rejected || ratio > value_type{1};
    }
    output = candidate;
    return Status::success;
}

} /* end namespace formal_eskf */
