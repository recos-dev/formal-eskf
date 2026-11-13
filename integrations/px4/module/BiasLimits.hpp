// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <formal_eskf/runtime/bias_limits.hpp>

namespace formal_eskf::px4
{

inline constexpr runtime::BiasLimits<double> imu_bias_limits{0.4, 0.15};

/** Fixed deployment bounds, applied after each successful measurement group. */
template <typename Linalg> [[nodiscard]] Status limit_imu_bias(configuration::Ins::NominalState<Linalg> & state)
{
    using value_type = typename Linalg::value_type;
    return runtime::limit_bias(state, {static_cast<value_type>(imu_bias_limits.accelerometer),
                                       static_cast<value_type>(imu_bias_limits.gyroscope)});
}

} /* end namespace formal_eskf::px4 */
