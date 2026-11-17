// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "DeploymentProfile.hpp"

namespace formal_eskf::px4
{

/** Reporting-only signed squared ratio; update once per measurement, never on publication. */
struct InnovationRatio
{
    std::uint64_t time_us{};
    double filtered = NAN;

    // Innovation uses PX4's prediction-minus-observation convention.
    void update(std::uint64_t sample_time_us, double innovation, double ratio)
    {
        if (time_us != 0U && sample_time_us > time_us)
        {
            const double dt =
                std::clamp(double(sample_time_us - time_us) * 1e-6, profile::output::innovation_minimum_interval_s,
                           profile::output::innovation_maximum_interval_s);
            const double alpha = dt / (dt + profile::output::innovation_time_constant_s);
            const double signed_ratio = innovation > 0. ? ratio : (innovation < 0. ? -ratio : 0.);
            filtered = std::isfinite(filtered) ? filtered + alpha * (signed_ratio - filtered) : ratio;
            if (ratio > profile::output::innovation_ratio_limit)
            {
                filtered = std::clamp(filtered, -profile::output::innovation_ratio_limit,
                                      profile::output::innovation_ratio_limit);
            }
        }
        else
        {
            // PX4 seeds with the unsigned ratio, including a repeated/regressed timestamp.
            filtered = ratio;
        }
        if (!std::isfinite(innovation) || !std::isfinite(ratio) || ratio < 0.)
        {
            filtered = NAN;
        }
        time_us = sample_time_us;
    }

    void reset(std::uint64_t sample_time_us)
    {
        time_us = sample_time_us;
        filtered = 0.;
    }
};

} // namespace formal_eskf::px4
