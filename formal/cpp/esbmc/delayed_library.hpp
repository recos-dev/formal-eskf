// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "algorithm_support.hpp"
#include <initializer_list>

// ESBMC 8.4.0 omits max(initializer_list). Only the unsigned three-value
// startup timing call is redirected; the owner body is unchanged. The model
// is checked against a relational maximum oracle in delayed_library.cpp.
namespace std
{
template <typename Value> Value formal_eskf_proof_max(initializer_list<Value> values)
{
    __ESBMC_assert(values.size() != 0U, "delayed library: maximum requires a nonempty list");
    auto position = values.begin();
    Value result = *position++;
    for (; position != values.end(); ++position)
    {
        if (result < *position)
        {
            result = *position;
        }
    }
    return result;
}
} // namespace std
