/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once
#include "contracts.hpp"
// Scoped value-storage model for HeightAiding's two sessions. ESBMC 8.4.0
// both zeroes nonzero defaults in its std::array model and crashes converting
// that model's copy constructor for Session. Named element storage retains
// actual element constructors/copy operations. Only indexing, value copying
// and bounded iteration are used by this source; no target layout claim.
namespace std
{
template <typename Value, std::size_t Size> class FormalEskfHeightSessions
{
    static_assert(Size == 2U);
    formal_eskf::verification::ScalarArray<Value, Size> m_values;

public:
    struct Iterator
    {
        FormalEskfHeightSessions * owner;
        std::size_t index;
        Value & operator*() const { return (*owner)[index]; }
        Iterator & operator++()
        {
            ++index;
            return *this;
        }
        bool operator!=(Iterator const & other) const { return owner != other.owner || index != other.index; }
    };
    struct ConstIterator
    {
        FormalEskfHeightSessions const * owner;
        std::size_t index;
        Value const & operator*() const { return (*owner)[index]; }
        ConstIterator & operator++()
        {
            ++index;
            return *this;
        }
        bool operator!=(ConstIterator const & other) const { return owner != other.owner || index != other.index; }
    };
    Value & operator[](std::size_t i) { return m_values[i]; }
    Value const & operator[](std::size_t i) const { return m_values[i]; }
    Iterator begin() { return {this, 0U}; }
    Iterator end() { return {this, Size}; }
    ConstIterator begin() const { return {this, 0U}; }
    ConstIterator end() const { return {this, Size}; }
};
} // namespace std
