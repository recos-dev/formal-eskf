// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <algorithm>
#include <cmath>

namespace formal_eskf::px4
{

/** Scalar altitude offset outside the INS state; callers freeze the reference source. */
template <typename Scalar> struct HeightBias
{
    Scalar value{}, variance{};
    Scalar maximum_variance = Scalar{2};
    Scalar signed_ratio{}, positive_time{}, negative_time{};
    Scalar dt = static_cast<Scalar>(0.01);

    void predict(Scalar interval) noexcept
    {
        dt = interval;
        bool const offset = std::abs(signed_ratio) > static_cast<Scalar>(0.2) || positive_time > Scalar{10} ||
                            negative_time > Scalar{10};
        variance = std::clamp(variance + static_cast<Scalar>(0.13 * 0.13) * dt * (offset ? Scalar{1000} : Scalar{1}),
                              static_cast<Scalar>(1e-8), maximum_variance);
    }

    void fuse(Scalar measurement, Scalar measurement_variance, Scalar sensor_variance) noexcept
    {
        maximum_variance = std::max(sensor_variance, static_cast<Scalar>(1e-4));
        Scalar const residual = measurement - value;
        Scalar const S = variance + std::max(measurement_variance, static_cast<Scalar>(1e-4));
        Scalar const ratio = residual * residual / (Scalar{9} * S);
        if (ratio < Scalar{1})
        {
            Scalar const K = variance / S;
            value += K * residual;
            variance = std::clamp((Scalar{1} - K) * variance, static_cast<Scalar>(1e-8), maximum_variance);
        }
        Scalar const sign = residual > Scalar{0} ? Scalar{1} : (residual < Scalar{0} ? Scalar{-1} : Scalar{0});
        signed_ratio += dt / (dt + Scalar{10}) * (std::clamp(sign * ratio, Scalar{-1}, Scalar{1}) - signed_ratio);
        positive_time = residual > Scalar{0} ? Scalar{0} : positive_time + dt;
        negative_time = residual > Scalar{0} ? negative_time + dt : Scalar{0};
    }

    [[nodiscard]] bool finite() const noexcept
    {
        return std::isfinite(value) && std::isfinite(variance) && std::isfinite(signed_ratio) &&
               std::isfinite(positive_time) && std::isfinite(negative_time);
    }
}; /* end struct HeightBias */

} /* end namespace formal_eskf::px4 */
