// SPDX-License-Identifier: BSD-3-Clause

#include "contracts.hpp"
#include <cstdint>
#include "../../../integrations/px4/module/InnovationRatio.hpp"

// Independent reporting oracle. The reporting filter cannot authorize fusion.
formal_eskf::px4::InnovationRatio ratio_update(formal_eskf::px4::InnovationRatio old, std::uint64_t time,
                                               double innovation, double ratio)
{
    namespace output = formal_eskf::px4::profile::output;
    double value = ratio;
    if (old.time_us != 0U && time > old.time_us)
    {
        double dt = static_cast<double>(time - old.time_us) * 1e-6;
        dt = dt < output::innovation_minimum_interval_s   ? output::innovation_minimum_interval_s
             : dt > output::innovation_maximum_interval_s ? output::innovation_maximum_interval_s
                                                          : dt;
        double const alpha = dt / (dt + output::innovation_time_constant_s);
        double const signed_ratio = innovation > 0. ? ratio : innovation < 0. ? -ratio : 0.;
        value = std::isfinite(old.filtered) ? old.filtered + alpha * (signed_ratio - old.filtered) : ratio;
        if (ratio > output::innovation_ratio_limit)
        {
            value = value < -output::innovation_ratio_limit  ? -output::innovation_ratio_limit
                    : value > output::innovation_ratio_limit ? output::innovation_ratio_limit
                                                             : value;
        }
    }
    if (!std::isfinite(innovation) || !std::isfinite(ratio) || ratio < 0.)
    {
        value = NAN;
    }
    return {time, value};
}

void verify_innovation_reporting(formal_eskf::px4::InnovationRatio report, std::uint64_t time, double innovation,
                                 double ratio)
{
    auto const expected = ratio_update(report, time, innovation, ratio);
    report.update(time, innovation, ratio);
    __ESBMC_assert(report.time_us == time && (__builtin_bit_cast(std::uint64_t, report.filtered) ==
                                                  __builtin_bit_cast(std::uint64_t, expected.filtered) ||
                                              (std::isnan(report.filtered) && std::isnan(expected.filtered))),
                   "Innovation reporting: exact double-precision seed/filter/clamp and invalid-data behavior");
    report.reset(time);
    __ESBMC_assert(report.time_us == time && report.filtered == 0.,
                   "Innovation reporting: reset retains the supplied epoch with a zero diagnostic");
}
int main() { return 0; }
