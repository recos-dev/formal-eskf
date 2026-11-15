/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once
#include "algorithm_support.hpp"

// ESBMC 8.4.0 has no any_of and its double min overload returns int.
// These scoped standard-library contracts are used only while parsing
// HeightAiding, after its dependencies. No production body is replaced.
namespace std
{
template <typename Value> constexpr Value const & formal_eskf_proof_min(Value const & first, Value const & second)
{
    // Explicit returns avoid ESBMC's conditional-reference conversion defect.
    if (second < first)
    {
        return second;
    }
    return first;
}
template <typename Iterator, typename Predicate>
bool formal_eskf_proof_any_of(Iterator first, Iterator last, Predicate predicate)
{
    for (; first != last; ++first)
    {
        if (predicate(*first))
        {
            return true;
        }
    }
    return false;
}
} // namespace std
