/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

#include <algorithm>

// ESBMC 8.4.0's embedded <algorithm> returns the second equal argument from
// max, and adds non-standard by-value overloads. The production two-argument
// std::max contract (and inspected libstdc++ implementation) retains the first
// reference. In particular, max(-0,+0) must retain -0. Alpha-rename only calls
// in the selected production header AFTER loading its dependencies. This is
// a library contract model, not a change to production or a proof of target STL.
namespace std
{
template <typename Value> constexpr Value const & formal_eskf_proof_max(Value const & first, Value const & second)
{
    if (first < second)
    {
        return second;
    }
    return first;
}
} // namespace std
