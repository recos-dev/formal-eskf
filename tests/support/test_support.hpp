/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <cmath>
#include <cstddef>
#include <iostream>
#include <string_view>

namespace formal_eskf::test
{

class Context
{
public:
    void expect(bool condition, std::string_view profile, std::string_view description)
    {
        ++m_assertions;
        if (!condition)
        {
            ++m_failures;
            std::cerr << "FAIL [" << profile << "]: " << description << '\n';
        }
    }

    int failures() const { return m_failures; }
    std::size_t assertions() const { return m_assertions; }

private:
    int m_failures = 0;
    std::size_t m_assertions = 0;
};

template <typename T> bool near(T actual, T expected, T tolerance)
{
    using std::abs;
    return abs(actual - expected) <= tolerance;
}

} /* end namespace formal_eskf::test */
