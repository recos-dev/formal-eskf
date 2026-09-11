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
 * Standard-library implementation of scalar-math primitives.
 */

#include <cmath>
#include <limits>
#include <numbers>
#include <type_traits>

namespace formal_eskf::scalar
{

template <typename Scalar> class StandardMath
{

    static constexpr bool is_binary32 = std::is_same_v<Scalar, float> && std::numeric_limits<Scalar>::radix == 2 &&
                                        std::numeric_limits<Scalar>::digits == 24 &&
                                        std::numeric_limits<Scalar>::max_exponent == 128;
    static constexpr bool is_binary64 = std::is_same_v<Scalar, double> && std::numeric_limits<Scalar>::radix == 2 &&
                                        std::numeric_limits<Scalar>::digits == 53 &&
                                        std::numeric_limits<Scalar>::max_exponent == 1024;

    static_assert(std::numeric_limits<Scalar>::is_iec559);
    static_assert(is_binary32 || is_binary64);

public:
    using value_type = Scalar;

    [[nodiscard]] static constexpr value_type pi() noexcept { return std::numbers::pi_v<value_type>; }

    [[nodiscard]] static bool is_finite(value_type value) noexcept { return std::isfinite(value); }

    [[nodiscard]] static value_type absolute(value_type value) noexcept { return std::abs(value); }

    [[nodiscard]] static value_type sqrt(value_type value) noexcept { return std::sqrt(value); }

    [[nodiscard]] static value_type sin(value_type value) noexcept { return std::sin(value); }

    [[nodiscard]] static value_type cos(value_type value) noexcept { return std::cos(value); }

    [[nodiscard]] static value_type atan2(value_type y, value_type x) noexcept { return std::atan2(y, x); }

}; /* end class StandardMath */

} /* end namespace formal_eskf::scalar */
