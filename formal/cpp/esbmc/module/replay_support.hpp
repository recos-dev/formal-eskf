// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <optional>

extern void __ESBMC_assert(bool, char const *);

// Scoped library contract for trivial replay values: ESBMC's bundled optional
// lacks emplace(). This does not establish target STL construction/destruction.
namespace std
{
template <typename Value> struct formal_eskf_replay_optional
{
    bool engaged{};
    Value value{};
    formal_eskf_replay_optional() = default;
    formal_eskf_replay_optional(nullopt_t) {}
    formal_eskf_replay_optional(Value const & input) : engaged(true), value(input) {}
    bool has_value() const { return engaged; }
    explicit operator bool() const { return engaged; }
    Value const * operator->() const
    {
        __ESBMC_assert(engaged, "Replay optional access requires an engaged value");
        return &value;
    }
    Value const & operator*() const
    {
        __ESBMC_assert(engaged, "Replay optional access requires an engaged value");
        return value;
    }
    Value & emplace()
    {
        value = {};
        engaged = true;
        return value;
    }
};
} // namespace std
