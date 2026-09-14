/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "contracts.hpp"

using namespace contract_proof;
using CovarianceVector = Linalg::vector_type<15U>;

namespace
{

// Read the actual fields independently of coefficient/operator[] indexing.
// The entry-gathering contract must be able to represent 15 independent
// values: a shared getter/setter indexing defect must not constrain that domain.
template <std::size_t Count>
void flatten(formal_eskf::verification::ScalarArray<Scalar, Count> const & source, Scalar * output)
{
    *output = source.head;
    if constexpr (Count > 1U)
    {
        flatten(source.tail, output + 1U);
    }
}

void flatten(CovarianceVector const & source, Scalar * output)
{
    flatten(formal_eskf::linalg::detail::MatrixAccess::storage(source).values, output);
}

} // namespace

void verify_covariance_vector_storage(CovarianceVector source, CovarianceVector output, std::size_t index,
                                      Scalar replacement)
{
    __ESBMC_assume(index < 15U);
    Scalar before[15U]{}, constructed[15U]{}, assigned[15U]{}, initialized[15U]{}, after[15U]{};
    flatten(source, before);
    auto const copy = source;
    output = source;
    flatten(copy, constructed);
    flatten(output, assigned);
    auto const zero = CovarianceVector::zero();
    flatten(zero, initialized);
    for (std::size_t k = 0U; k < 15U; ++k)
    {
        __ESBMC_assert(same(constructed[k], before[k]) && same(assigned[k], before[k]),
                       "E-RESET vector storage: actual copy construction and assignment preserve every field");
        __ESBMC_assert(same(initialized[k], Scalar{0}),
                       "E-RESET vector storage: every initialized field is positive zero");
    }
    auto const & const_source = source;
    __ESBMC_assert(same(source(index), before[index]) && same(const_source(index), before[index]),
                   "E-RESET vector storage: actual mutable and const access select the independent requested field");
    source.set(index, replacement);
    flatten(source, after);
    flatten(copy, constructed);
    flatten(output, assigned);
    for (std::size_t k = 0U; k < 15U; ++k)
    {
        __ESBMC_assert(same(after[k], k == index ? replacement : before[k]),
                       "E-RESET vector storage: actual setter changes only the requested field");
        __ESBMC_assert(same(constructed[k], before[k]) && same(assigned[k], before[k]),
                       "E-RESET vector storage: copied vectors do not share mutable cells");
    }
}

int main() { return 0; }
