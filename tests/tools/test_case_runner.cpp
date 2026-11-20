/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <sstream>
#include <string_view>

#include "test_cases.hpp"

namespace
{

using formal_eskf::test::Case;
using formal_eskf::test::Context;
using formal_eskf::test::run_cases;

void passing_case(Context & test, std::string_view profile)
{
    test.expect(profile == "A" || profile == "B", profile, "runner passes the selected profile to the callback");
}

void failing_case(Context & test, std::string_view profile)
{
    test.expect(false, profile, "intentional failure for runner result propagation");
}

void empty_case(Context &, std::string_view) {}

void conditional_case(Context & test, std::string_view profile)
{
    if (profile == "A")
    {
        test.expect(true, profile, "supported profile executes an assertion");
    }
}

} /* end namespace */

int main()
{
    Context test;
    std::ostringstream output;
    Case const cases[] = {{"first", "A", passing_case}, {"first", "B", passing_case}, {"second", "A", passing_case}};
    char const * const all[] = {"test"};
    int result = run_cases(cases, 1, all, output);
    test.expect(result == 0 &&
                    output.str() ==
                        "RUN first A\nPASS first A\nRUN first B\nPASS first B\nRUN second A\nPASS second A\n",
                "runner", "default selection executes every registered case/profile once");

    output.str("");
    char const * const selected[] = {"test", "--case", "first", "--profile", "B"};
    result = run_cases(cases, 5, selected, output);
    test.expect(result == 0 && output.str() == "RUN first B\nPASS first B\n", "runner",
                "explicit selection executes only the exact case/profile");

    output.str("");
    char const * const listed[] = {"test", "--list"};
    result = run_cases(cases, 2, listed, output);
    test.expect(result == 0 && output.str() == "first A\nfirst B\nsecond A\n", "runner",
                "list returns the registry without execution results");

    char const * const invalid[][5] = {{"test", "--case", "unknown", "--profile", "A"},
                                       {"test", "--case", "first", "--profile", "unknown"},
                                       {"test", "--case", "", "--profile", "A"},
                                       {"test", "--case", "first", "--profile", ""},
                                       {"test", "--case", "first", "--case", "first"}};
    for (auto const & arguments : invalid)
    {
        output.str("");
        result = run_cases(cases, 5, arguments, output);
        test.expect(result == 2 && output.str().find("RUN ") == std::string::npos, "runner",
                    "unknown, empty and duplicate selectors fail before execution");
    }
    output.str("");
    result = run_cases(cases, 3, selected, output);
    test.expect(result == 2 && output.str().find("RUN ") == std::string::npos, "runner",
                "incomplete selection is rejected before execution");

    Case const duplicate[] = {{"first", "A", passing_case}, {"first", "A", passing_case}};
    output.str("");
    result = run_cases(duplicate, 1, all, output);
    test.expect(result == 2 && output.str().find("RUN ") == std::string::npos, "runner",
                "duplicate case/profile registry entries are rejected before execution");
    test.expect(run_cases({}, 1, all, output) == 2, "runner", "empty required suite cannot pass");
    Case const incomplete[] = {{"first", "A", nullptr}};
    test.expect(run_cases(incomplete, 1, all, output) == 2, "runner", "missing implementation cannot pass");

    Case const failed[] = {{"failed", "A", failing_case}, {"passed", "A", passing_case}};
    std::ostringstream expected_error;
    auto * original_error = std::cerr.rdbuf(expected_error.rdbuf());
    output.str("");
    result = run_cases(failed, 1, all, output);
    std::cerr.rdbuf(original_error);
    test.expect(result == 1 && output.str() == "RUN failed A\nFAIL failed A\nRUN passed A\nPASS passed A\n" &&
                    expected_error.str().find("intentional failure") != std::string::npos,
                "runner", "failure is reported per case and cannot be masked by a later pass");
    Case const no_checks[] = {
        {"empty", "A", empty_case}, {"skipped", "B", conditional_case}, {"passed", "A", passing_case}};
    output.str("");
    expected_error.str("");
    original_error = std::cerr.rdbuf(expected_error.rdbuf());
    result = run_cases(no_checks, 1, all, output);
    std::cerr.rdbuf(original_error);
    test.expect(result == 1 && output.str().find("FAIL empty A") != std::string::npos &&
                    output.str().find("FAIL skipped B") != std::string::npos &&
                    output.str().find("PASS empty") == std::string::npos &&
                    output.str().find("PASS skipped") == std::string::npos &&
                    output.str().find("PASS passed A") != std::string::npos,
                "runner", "empty and silently skipped callbacks fail independently of a later passing case");

    output.str("");
    expected_error.str("");
    original_error = std::cerr.rdbuf(expected_error.rdbuf());
    result = run_cases(failed, 2, listed, output);
    std::cerr.rdbuf(original_error);
    test.expect(result == 0 && output.str() == "failed A\npassed A\n" && expected_error.str().empty(), "runner",
                "listing never invokes even a deliberately failing callback");

    Case const missing_id[] = {{"", "A", passing_case}};
    Case const missing_profile[] = {{"first", "", passing_case}};
    for (auto const * entry : {missing_id, missing_profile})
    {
        output.str("");
        result = run_cases(std::span<Case const>(entry, 1U), 1, all, output);
        test.expect(result == 2 && output.str().find("RUN ") == std::string::npos, "runner",
                    "an incomplete registry identity is rejected before any callback");
    }
    return test.failures() == 0 ? 0 : 1;
}
