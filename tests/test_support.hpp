/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <cmath>
#include <iostream>
#include <string_view>

namespace formal_eskf::test
{

class Context
{
public:
    void expect(bool condition, std::string_view profile, std::string_view description)
    {
        if (!condition)
        {
            ++m_failures;
            std::cerr << "FAIL [" << profile << "]: " << description << '\n';
        }
    }

    [[nodiscard]] int failures() const noexcept { return m_failures; }

private:
    int m_failures = 0;
};

template <typename T> [[nodiscard]] bool near(T actual, T expected, T tolerance)
{
    using std::abs;
    return abs(actual - expected) <= tolerance;
}

} /* end namespace formal_eskf::test */
