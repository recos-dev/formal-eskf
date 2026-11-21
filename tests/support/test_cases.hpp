/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <ostream>
#include <span>
#include <string_view>

#include "test_support.hpp"

namespace formal_eskf::test
{

struct Case
{
    std::string_view id;
    std::string_view profile;
    void (*run)(Context &, std::string_view) = nullptr;
};

// No arguments runs the suite. A selection must identify exactly one case/profile.
inline int run_cases(std::span<Case const> cases, int argc, char const * const * argv,
                     std::ostream & output = std::cout)
{
    bool const list = argc == 2 && std::string_view(argv[1]) == "--list";
    bool const selected =
        argc == 5 && std::string_view(argv[1]) == "--case" && std::string_view(argv[3]) == "--profile";
    if (argc != 1 && !list && !selected)
    {
        output << "Usage: " << argv[0] << " [--list | --case ID --profile PROFILE]\n";
        return 2;
    }
    if (cases.empty())
    {
        output << "ERROR: empty case registry\n";
        return 2;
    }
    std::size_t selected_index = cases.size();
    for (std::size_t index = 0U; index < cases.size(); ++index)
    {
        auto const & entry = cases[index];
        if (entry.id.empty() || entry.profile.empty() || entry.run == nullptr)
        {
            output << "ERROR: incomplete case registry entry\n";
            return 2;
        }
        auto const previous = cases.first(index);
        if (std::any_of(previous.begin(), previous.end(),
                        [&entry](Case const & other)
                        { return entry.id == other.id && entry.profile == other.profile; }))
        {
            output << "ERROR: duplicate case/profile: " << entry.id << ' ' << entry.profile << '\n';
            return 2;
        }
        if (selected && entry.id == argv[2] && entry.profile == argv[4])
        {
            selected_index = index;
        }
    }
    if (selected && selected_index == cases.size())
    {
        output << "ERROR: unknown case/profile: " << argv[2] << ' ' << argv[4] << '\n';
        return 2;
    }

    Context test;
    for (std::size_t index = 0U; index < cases.size(); ++index)
    {
        auto const & entry = cases[index];
        if (list)
        {
            output << entry.id << ' ' << entry.profile << '\n';
        }
        else if (!selected || index == selected_index)
        {
            output << "RUN " << entry.id << ' ' << entry.profile << '\n';
            int const before = test.failures();
            std::size_t const assertions_before = test.assertions();
            entry.run(test, entry.profile);
            if (test.assertions() == assertions_before)
            {
                output << "ERROR: case performed no assertions: " << entry.id << ' ' << entry.profile << '\n';
                test.expect(false, entry.profile, "case performed no assertions");
            }
            output << (test.failures() == before ? "PASS " : "FAIL ") << entry.id << ' ' << entry.profile << '\n';
        }
    }
    return test.failures() == 0 ? 0 : 1;
}

} /* end namespace formal_eskf::test */
