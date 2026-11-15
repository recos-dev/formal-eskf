/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "contracts.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <formal_eskf/eskf/accelerometer.hpp>
#include <formal_eskf/eskf/types.hpp>
#include <formal_eskf/runtime/measurement_fusion.hpp>
#include <formal_eskf/so3/rotation_vector.hpp>
#define private public
#include <formal_eskf/runtime/accelerometer.hpp>
#undef private
namespace accel_proof
{
using contract_proof::Linalg;
using contract_proof::same;
using contract_proof::Scalar;
using formal_eskf::Status;
using Magnitude = formal_eskf::runtime::AccelerometerMagnitude<Linalg>;
using Vector = Magnitude::vector_type;
inline bool same_vector(Vector const & a, Vector const & b)
{
    return same(a(0U), b(0U)) && same(a(1U), b(1U)) && same(a(2U), b(2U));
}
inline bool finite_vector(Vector const & value)
{
    return std::isfinite(value(0U)) && std::isfinite(value(1U)) && std::isfinite(value(2U));
}
} // namespace accel_proof
void verify_accelerometer_magnitude_initialization(accel_proof::Magnitude history)
{
    using namespace accel_proof;
    Magnitude fresh;
    Vector zero;
    __ESBMC_assert(!fresh.initialized() && same_vector(fresh.filtered_specific_force_b(), zero),
                   "R-ACCELEROMETER-MAGNITUDE: constructor establishes empty, zero filter history");
    history.reset();
    __ESBMC_assert(!history.initialized() && same_vector(history.filtered_specific_force_b(), zero),
                   "R-ACCELEROMETER-MAGNITUDE: reset clears arbitrary prior history and initialization");
}
void verify_accelerometer_magnitude(accel_proof::Magnitude history, accel_proof::Vector sample)
{
    using namespace accel_proof;
    auto const before = history;
    auto const old_sample = sample;
    constexpr Scalar interval = static_cast<Scalar>(0.01);
    constexpr Scalar time_constant = static_cast<Scalar>(0.09);
    constexpr Scalar alpha = interval / (time_constant + interval);
    constexpr Scalar gravity = static_cast<Scalar>(9.80665);
    constexpr Scalar lower = gravity * static_cast<Scalar>(0.9);
    constexpr Scalar upper = gravity * static_cast<Scalar>(1.1);
    Vector expected;
    Scalar norm{0};
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        expected(axis) = before.m_initialized ? before.m_filtered_specific_force_b(axis) +
                                                    (sample(axis) - before.m_filtered_specific_force_b(axis)) * alpha
                                              : sample(axis);
        norm += expected(axis) * expected(axis);
    }
    Status const status = !finite_vector(sample)                             ? Status::non_finite_input
                          : !finite_vector(expected) || !std::isfinite(norm) ? Status::non_finite_result
                                                                             : Status::success;
    auto const result = history.update(sample);
    __ESBMC_assert(result.status == status, "R-ACCELEROMETER-MAGNITUDE: exact input/result failure classification");
    if (status == Status::success)
    {
        __ESBMC_assert(history.initialized() && same_vector(history.filtered_specific_force_b(), expected) &&
                           same(result.squared_norm, norm) &&
                           result.within_gravity_band == (norm > lower * lower && norm < upper * upper),
                       "R-ACCELEROMETER-MAGNITUDE: seed or fixed-alpha filter, actual norm and strict gravity band");
        __ESBMC_assert(finite_vector(history.filtered_specific_force_b()) && std::isfinite(result.squared_norm) &&
                           result.squared_norm >= Scalar{0},
                       "R-ACCELEROMETER-MAGNITUDE: every committed filter and diagnostic are finite");
    }
    else
    {
        __ESBMC_assert(history.initialized() == before.m_initialized &&
                           same_vector(history.filtered_specific_force_b(), before.m_filtered_specific_force_b) &&
                           !result.within_gravity_band && same(result.squared_norm, Scalar{0}),
                       "R-ACCELEROMETER-MAGNITUDE: failure preserves all history and publishes no valid diagnostic");
    }
    __ESBMC_assert(!finite_vector(before.m_filtered_specific_force_b) ||
                       finite_vector(history.filtered_specific_force_b()),
                   "R-ACCELEROMETER-MAGNITUDE: finite history is inductive across every public update");
    auto const & filtered = history.filtered_specific_force_b();
    __ESBMC_assert(&filtered == &history.m_filtered_specific_force_b && same_vector(sample, old_sample),
                   "R-ACCELEROMETER-MAGNITUDE: accessor identity and sample purity");
}

using contract_proof::Linalg;
using contract_proof::same;
using formal_eskf::Status;
using Conditions = formal_eskf::runtime::AccelerometerFusionConditions;
using Magnitude = formal_eskf::runtime::AccelerometerMagnitude<Linalg>::Result;

void verify_accelerometer_conditions(Conditions conditions, Magnitude magnitude)
{
    auto const before = conditions;
    auto const old_magnitude = magnitude;
    auto const actual = formal_eskf::runtime::check_accelerometer_fusion_conditions<Linalg>(conditions, magnitude);
    // Exhaustive truth-table rows, independently expressed from the conjunction
    // in production. All eight flags, every status and unused IEEE diagnostic
    // are symbolic; no success or finite-data premise is assumed.
    unsigned const row = static_cast<unsigned>(conditions.enabled) + 2U * conditions.tilt_aligned +
                         4U * conditions.horizontal_aiding + 8U * conditions.at_rest +
                         16U * magnitude.within_gravity_band;
    bool const active = magnitude.status == Status::success && (row == 11U || row == 19U || row == 27U);
    unsigned const clipping_count =
        static_cast<unsigned>(conditions.clipped[0U]) + conditions.clipped[1U] + conditions.clipped[2U];
    __ESBMC_assert(actual.status == magnitude.status && actual.active == active &&
                       actual.sample_eligible == (active && clipping_count == 0U),
                   "R-ACCELEROMETER-CONDITIONS: exact status, active mode and per-sample eligibility truth table");
    __ESBMC_assert(!actual.sample_eligible ||
                       (actual.active && magnitude.status == Status::success && conditions.enabled &&
                        conditions.tilt_aligned && !conditions.horizontal_aiding),
                   "R-ACCELEROMETER-CONDITIONS: eligibility cannot bypass a failed evaluation or required mode");
    __ESBMC_assert(conditions.enabled == before.enabled && conditions.tilt_aligned == before.tilt_aligned &&
                       conditions.horizontal_aiding == before.horizontal_aiding &&
                       conditions.at_rest == before.at_rest && conditions.clipped == before.clipped &&
                       magnitude.status == old_magnitude.status &&
                       magnitude.within_gravity_band == old_magnitude.within_gravity_band &&
                       same(magnitude.squared_norm, old_magnitude.squared_norm),
                   "R-ACCELEROMETER-CONDITIONS: every input, including unused diagnostics, remains unchanged");
}
int main() { return 0; }
