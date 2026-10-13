/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <array>

#include <formal_eskf/runtime/accelerometer_magnitude.hpp>

namespace formal_eskf::runtime
{

struct AccelerometerFusionConditions
{
    bool enabled = false;
    bool tilt_aligned = false;
    bool horizontal_aiding = false; // Actual aiding activity, not availability of a GNSS fix.
    bool at_rest = false;           // Caller detection, not inferred from the magnitude check.
    std::array<bool, 3U> clipped{};
};

struct AccelerometerFusionCheck
{
    Status status = Status::success;
    bool active = false;          // Mode condition, before per-sample clipping/innovation checks.
    bool sample_eligible = false; // Before innovation gating; does not mean a correction was performed.
};

/**
 * Stateless gravity-aiding conditions for AHRS/INS. Supply the actual magnitude
 * result and caller conditions for the same sample/horizon, never stale flags.
 * At rest overrides only the magnitude band, not failed evaluation or clipping.
 * Clipping blocks the sample without clearing an otherwise active mode.
 * No innovation check, correction, lifecycle, or estimator/filter state update.
 */
template <typename Linalg>
[[nodiscard]] constexpr AccelerometerFusionCheck
check_accelerometer_fusion_conditions(AccelerometerFusionConditions const & conditions,
                                      typename AccelerometerMagnitude<Linalg>::Result const & magnitude) noexcept
{
    AccelerometerFusionCheck result;
    result.status = magnitude.status;
    if (!succeeded(result.status))
    {
        return result;
    }
    result.active = conditions.enabled && (magnitude.within_gravity_band || conditions.at_rest) &&
                    !conditions.horizontal_aiding && conditions.tilt_aligned;
    result.sample_eligible =
        result.active && !(conditions.clipped[0U] || conditions.clipped[1U] || conditions.clipped[2U]);
    return result;
}

} /* end namespace formal_eskf::runtime */
