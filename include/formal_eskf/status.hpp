/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

namespace formal_eskf
{

enum class Status
{
    success,
    non_finite_input,
    domain_error,
    out_of_range,
    zero_or_unsafe_divisor,
    invalid_quaternion_norm,
    not_positive_definite,
    ill_conditioned,
    non_finite_result,
};

[[nodiscard]] constexpr bool succeeded(Status status) noexcept { return status == Status::success; }

} /* end namespace formal_eskf */
