/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

#include <formal_eskf/runtime/accelerometer_conditions.hpp>
#include <formal_eskf/runtime/tilt_alignment.hpp>

namespace formal_eskf::example
{

/**
 * Attitude and aiding conditions for the replay's single navigation source.
 * Sensor quality/lifecycle decisions remain in their respective controllers.
 * Times are monotonic replay microseconds; initialization is not a fusion event.
 */
template <typename Linalg> class ReplayFusionConditions
{
public:
    using value_type = typename Linalg::value_type;
    static constexpr std::uint64_t horizontal_timeout_us = 7'000'000U;
    static constexpr value_type tilt_limit = runtime::TiltAlignment<Linalg>::tilt_limit;

    [[nodiscard]] Status update_tilt(so3::UnitQuaternion<Linalg> const & q_nb,
                                     linalg::Matrix<Linalg, 3U, 3U> const & P_theta_b)
    {
        return m_tilt.update(q_nb, P_theta_b);
    }

    /** Call only after actual horizontal position/velocity correction succeeds. */
    void record_horizontal_fusion(std::uint64_t time_us) { m_last_horizontal_fusion_us = time_us; }

    [[nodiscard]] bool horizontal_aiding(std::uint64_t now_us) const
    {
        // A future timestamp is not a reason to release gravity aiding.
        return m_last_horizontal_fusion_us != 0U &&
               (now_us < m_last_horizontal_fusion_us || now_us - m_last_horizontal_fusion_us < horizontal_timeout_us);
    }
    [[nodiscard]] bool tilt_aligned() const { return m_tilt.aligned(); }
    [[nodiscard]] value_type tilt_variance() const { return m_tilt.variance(); }

    /**
     * -1 means unavailable. Unknown rest disables only its override; unknown
     * clipping cannot authorize a sample, so no complete conditions are returned.
     * enabled requests gravity aiding, not permission to run correction here.
     */
    [[nodiscard]] std::optional<runtime::AccelerometerFusionConditions> conditions(std::uint64_t now_us, bool enabled,
                                                                                   int at_rest, int clipping_mask) const
    {
        if (now_us == 0U || now_us < m_last_horizontal_fusion_us || at_rest < -1 || at_rest > 1 || clipping_mask < 0 ||
            clipping_mask > 7)
        {
            return std::nullopt;
        }
        return runtime::AccelerometerFusionConditions{
            enabled,
            m_tilt.aligned(),
            horizontal_aiding(now_us),
            at_rest == 1,
            {(clipping_mask & 1) != 0, (clipping_mask & 2) != 0, (clipping_mask & 4) != 0}};
    }

private:
    runtime::TiltAlignment<Linalg> m_tilt{};
    std::uint64_t m_last_horizontal_fusion_us{};
}; /* end class ReplayFusionConditions */

} /* end namespace formal_eskf::example */
