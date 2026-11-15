/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "contracts.hpp"
#include "height_support.hpp"
using contract_proof::Scalar;
void verify_height_min(Scalar first, Scalar second)
{
    auto const & result = std::formal_eskf_proof_min(first, second);
    __ESBMC_assert(
        &result == (second < first ? &second : &first),
        "height library contract: scalar min preserves the first reference on ties and unordered comparisons");
}
void verify_height_min_unsigned(unsigned first, unsigned second)
{
    auto const & result = std::formal_eskf_proof_min(first, second);
    __ESBMC_assert(&result == (second < first ? &second : &first),
                   "height library contract: unsigned min preserves the selected reference without conversion");
}
void verify_height_any_of(bool first, bool second, unsigned length)
{
    __ESBMC_assume(length <= 2U);
    bool const values[2] = {first, second};
    unsigned calls = 0U;
    bool const result = std::formal_eskf_proof_any_of(values, values + length,
                                                      [&calls](bool value)
                                                      {
                                                          ++calls;
                                                          return value;
                                                      });
    bool const expected = (length > 0U && first) || (length > 1U && second);
    __ESBMC_assert(
        result == expected && calls == (length > 0U && first ? 1U : length),
        "height library contract: any_of checks the bounded session range and stops on the first true result");
}
int main() { return 0; }
