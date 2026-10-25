/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <algorithm>

#include <formal_eskf/eskf/configuration/ins.hpp>
#include <formal_eskf/linalg/operations.hpp>

namespace formal_eskf::runtime
{

/** Symmetric per-axis bounds, not vector-norm bounds. Zero forces zero bias. */
template <typename Scalar> struct BiasLimits
{
    Scalar accelerometer{}; // m/s^2.
    Scalar gyroscope{};     // rad/s.
};

/**
 * Explicitly limit the six INS bias coefficients in place after a correction.
 * Finite, nonnegative limits and finite biases are required. Validate all inputs
 * before changing state; failure preserves it. Other state fields are untouched.
 * This operation neither changes covariance nor consults learning permissions:
 * an already out-of-range bias is limited even when its learning is inhibited.
 */
template <typename Linalg>
[[nodiscard]] Status try_limit_bias(configuration::Ins::NominalState<Linalg> & state,
                                    BiasLimits<typename Linalg::value_type> limits) noexcept
{
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    if (!scalar::is_finite<math_type>(limits.accelerometer) || !scalar::is_finite<math_type>(limits.gyroscope) ||
        !linalg::all_finite(state.b_a) || !linalg::all_finite(state.b_g))
    {
        return Status::non_finite_input;
    }
    if (limits.accelerometer < value_type{0} || limits.gyroscope < value_type{0})
    {
        return Status::domain_error;
    }
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        state.b_a.set(axis, std::clamp(state.b_a(axis), -limits.accelerometer, limits.accelerometer));
        state.b_g.set(axis, std::clamp(state.b_g(axis), -limits.gyroscope, limits.gyroscope));
    }
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
