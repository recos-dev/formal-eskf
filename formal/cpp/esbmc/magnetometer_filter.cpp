/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "solve_support.hpp"
#include <formal_eskf/eskf/magnetometer.hpp>
#include <formal_eskf/runtime/magnetic_field.hpp>
#include <formal_eskf/runtime/measurement_fusion.hpp>
#define private public
#include <formal_eskf/runtime/magnetometer.hpp>
#undef private
namespace magnetic_filter_proof
{
using contract_proof::finite_vector;
using contract_proof::same;
using contract_proof::same_vector;
using contract_proof::Scalar;
using formal_eskf::Status;
using solve_proof::Backend;
using Filter = formal_eskf::runtime::MagnetometerFilter<Backend>;
using Vector = Filter::vector_type;
} // namespace magnetic_filter_proof
void verify_magnetic_filter_initialization(magnetic_filter_proof::Filter filter)
{
    using namespace magnetic_filter_proof;
    Filter fresh;
    Vector zero;
    __ESBMC_assert(!fresh.initialized() && same_vector(fresh.field_b(), zero),
                   "R-MAGNETOMETER-FILTER: default filter has zero data and no accepted sample");
    filter.reset();
    __ESBMC_assert(!filter.initialized() && same_vector(filter.field_b(), zero),
                   "R-MAGNETOMETER-FILTER: reset discards initialized or invalid prior history");
}
void verify_magnetic_filter_update(magnetic_filter_proof::Filter filter, magnetic_filter_proof::Vector sample,
                                   bool reseed)
{
    using namespace magnetic_filter_proof;
    auto const before = filter;
    auto const old_sample = sample;
    constexpr Scalar interval = static_cast<Scalar>(0.01), time_constant = static_cast<Scalar>(0.09);
    constexpr Scalar alpha = interval / (time_constant + interval);
    Vector expected;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        expected(axis) = !before.m_initialized || reseed
                             ? sample(axis)
                             : before.m_field_b(axis) + (sample(axis) - before.m_field_b(axis)) * alpha;
    }
    Status const expected_status = !finite_vector(sample)     ? Status::non_finite_input
                                   : !finite_vector(expected) ? Status::non_finite_result
                                                              : Status::success;
    auto const status = filter.update(sample, reseed);
    __ESBMC_assert(status == expected_status,
                   "R-MAGNETOMETER-FILTER: finite sample and computed candidate determine exact failure priority");
    if (status == Status::success)
    {
        __ESBMC_assert(filter.initialized() && same_vector(filter.field_b(), expected) &&
                           finite_vector(filter.field_b()),
                       "R-MAGNETOMETER-FILTER: first/reseed sample replaces history; ordinary update uses fixed alpha");
    }
    else
    {
        __ESBMC_assert(filter.initialized() == before.m_initialized && same_vector(filter.field_b(), before.m_field_b),
                       "R-MAGNETOMETER-FILTER: every failure, including requested reseed, preserves all history");
    }
    auto const & reference = filter.field_b();
    __ESBMC_assert(&reference == &filter.m_field_b && same_vector(sample, old_sample),
                   "R-MAGNETOMETER-FILTER: accessor returns owned data and borrowed sample stays unchanged");
    __ESBMC_assert(!finite_vector(before.m_field_b) || finite_vector(filter.field_b()),
                   "R-MAGNETOMETER-FILTER: finite field history is an initialized inductive invariant");
}
int main() { return 0; }
