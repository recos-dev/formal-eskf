/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "solve_support.hpp"
#include "scalar_support.hpp"
#include <formal_eskf/eskf/innovation.hpp>

#ifndef FORMAL_ESKF_PROOF_MEASUREMENT_SIZE
#define FORMAL_ESKF_PROOF_MEASUREMENT_SIZE 1
#endif

namespace
{
using formal_eskf::Status;
using scalar_proof::same;
using scalar_proof::Scalar;
using backend_type = solve_proof::Backend;
constexpr std::size_t size = FORMAL_ESKF_PROOF_MEASUREMENT_SIZE;
constexpr unsigned alias = FORMAL_ESKF_PROOF_ALIAS;
using vector_type = formal_eskf::linalg::Vector<backend_type, size>;
using check_type = formal_eskf::InnovationCheck<backend_type, size>;
static_assert(size >= 1U && size <= 3U && alias <= 4U);

bool equal_diagnostics(vector_type const & a, vector_type const & b)
{
    bool equal = true;
    for (std::size_t i = 0U; i < size; ++i)
    {
        equal = same(a(i), b(i)) && equal;
    }
    return equal;
}

// Specification oracle: classify every axis, then select the first error.
// Unlike the implementation, aggregation is a count and never short-circuits.
// No finite/bounded/success assumptions restrict inputs or old diagnostics.
Status expected_result(vector_type const & r, vector_type const & variance, Scalar sigma, check_type & expected)
{
    constexpr Scalar minimum_normal = std::numeric_limits<Scalar>::min();
    bool inputs_finite = scalar_proof::finite(sigma);
    Status domain = sigma <= Scalar{0} ? Status::domain_error : Status::success;
    for (std::size_t i = 0U; i < size; ++i)
    {
        inputs_finite = scalar_proof::finite(r(i)) && scalar_proof::finite(variance(i)) && inputs_finite;
        auto const invalid = variance(i) <= Scalar{0}       ? Status::domain_error
                             : variance(i) < minimum_normal ? Status::zero_or_unsafe_divisor
                                                            : Status::success;
        if (domain == Status::success)
        {
            domain = invalid;
        }
    }
    if (!inputs_finite)
    {
        return Status::non_finite_input;
    }
    if (domain != Status::success)
    {
        return domain;
    }
    Scalar const squared_sigma = sigma * sigma;
    if (!scalar_proof::finite(squared_sigma))
    {
        return Status::non_finite_result;
    }
    if (squared_sigma < minimum_normal)
    {
        return Status::zero_or_unsafe_divisor;
    }
    Status status = Status::success;
    unsigned outliers = 0U;
    for (std::size_t i = 0U; i < size; ++i)
    {
        Scalar const numerator = r(i) * r(i);
        Scalar const denominator = squared_sigma * variance(i);
        auto invalid = !scalar_proof::finite(numerator) || !scalar_proof::finite(denominator)
                           ? Status::non_finite_result
                       : denominator < minimum_normal ? Status::zero_or_unsafe_divisor
                                                      : Status::success;
        if (invalid == Status::success)
        {
            expected.test_ratio(i) = numerator / denominator;
            if (!scalar_proof::finite(expected.test_ratio(i)))
            {
                invalid = Status::non_finite_result;
            }
            if (expected.test_ratio(i) > Scalar{1})
            {
                ++outliers;
            }
        }
        if (status == Status::success)
        {
            status = invalid;
        }
    }
    expected.rejected = outliers != 0U;
    return status;
}
} /* end anonymous namespace */

void verify_innovation(vector_type residual, vector_type variance, Scalar sigma, check_type output)
{
    // All five partitions of the two input vectors and output vector.
    auto const & r = alias == 1U || alias == 4U ? output.test_ratio : residual;
    auto const & v = alias == 2U || alias == 4U ? output.test_ratio : (alias == 3U ? residual : variance);
    auto const old_r = r;
    auto const old_v = v;
    auto const old_residual = residual;
    auto const old_variance = variance;
    auto const before = output;
    check_type expected;
    auto const expected_status = expected_result(old_r, old_v, sigma, expected);
    auto const status = formal_eskf::check_innovation(r, v, sigma, output);
    __ESBMC_assert(status == expected_status, "E-INNOV: ordered IEEE validation and arithmetic statuses");
    if (status == Status::success)
    {
        __ESBMC_assert(equal_diagnostics(output.test_ratio, expected.test_ratio) &&
                           output.rejected == expected.rejected,
                       "E-INNOV: publish every ordered ratio and reject iff any ratio exceeds one");
        for (std::size_t i = 0U; i < size; ++i)
        {
            __ESBMC_assert(scalar_proof::finite(output.test_ratio(i)) && output.test_ratio(i) >= Scalar{0},
                           "E-INNOV: successful diagnostics are finite and nonnegative");
        }
    }
    else
    {
        __ESBMC_assert(equal_diagnostics(output.test_ratio, before.test_ratio) && output.rejected == before.rejected,
                       "E-INNOV: all failures preserve the complete diagnostics, including aliases");
    }
    __ESBMC_assert(equal_diagnostics(residual, old_residual) && equal_diagnostics(variance, old_variance),
                   "E-INNOV: separate input storage is never modified");
}

int main() { return 0; }
