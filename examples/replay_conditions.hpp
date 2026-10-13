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
#include <numbers>
#include <optional>

#include <formal_eskf/runtime/accelerometer_conditions.hpp>
#include <formal_eskf/so3/operations.hpp>

namespace formal_eskf::example
{

/**
 * Conditions for the replay's single, fail-fast navigation source. No sensor
 * quality/lifecycle controller is added: a navigation failure ends the replay.
 * Times are monotonic replay microseconds; initialization is not a fusion event.
 */
template <typename Linalg> class ReplayFusionConditions
{
public:
    using value_type = typename Linalg::value_type;
    static constexpr std::uint64_t horizontal_timeout_us = 7'000'000U;
    static constexpr value_type tilt_limit = std::numbers::pi_v<value_type> / value_type{60};

    /** Unit q_nb and symmetric PSD body-local attitude covariance are caller premises. */
    [[nodiscard]] Status update_tilt(so3::UnitQuaternion<Linalg> const & q_nb,
                                     linalg::Matrix<Linalg, 3U, 3U> const & P_theta_b) noexcept
    {
        if (!linalg::all_finite(q_nb.coefficients()) || !linalg::all_finite(P_theta_b))
        {
            return Status::non_finite_input;
        }
        auto const R_nb = so3::to_rotation_matrix(q_nb);
        auto const P_theta_n = R_nb * P_theta_b * linalg::transpose(R_nb);
        auto const variance = P_theta_n(0U, 0U) + P_theta_n(1U, 1U);
        if (!linalg::all_finite(P_theta_n) || !std::isfinite(variance))
        {
            return Status::non_finite_result;
        }
        if (P_theta_n(0U, 0U) < value_type{0} || P_theta_n(1U, 1U) < value_type{0})
        {
            return Status::domain_error;
        }
        m_tilt_variance = variance;
        m_tilt_aligned = m_tilt_aligned || variance < tilt_limit * tilt_limit;
        return Status::success;
    }

    /** Call only after actual horizontal position/velocity correction succeeds. */
    void record_horizontal_fusion(std::uint64_t time_us) noexcept { m_last_horizontal_fusion_us = time_us; }

    [[nodiscard]] bool horizontal_aiding(std::uint64_t now_us) const noexcept
    {
        // A future timestamp is not a reason to release gravity aiding.
        return m_last_horizontal_fusion_us != 0U &&
               (now_us < m_last_horizontal_fusion_us || now_us - m_last_horizontal_fusion_us < horizontal_timeout_us);
    }
    [[nodiscard]] bool tilt_aligned() const noexcept { return m_tilt_aligned; }
    [[nodiscard]] value_type tilt_variance() const noexcept { return m_tilt_variance; }

    /**
     * -1 means unavailable. Unknown rest disables only its override; unknown
     * clipping cannot authorize a sample, so no complete conditions are returned.
     * enabled requests gravity aiding, not permission to run correction here.
     */
    [[nodiscard]] std::optional<runtime::AccelerometerFusionConditions>
    conditions(std::uint64_t now_us, bool enabled, int at_rest, int clipping_mask) const noexcept
    {
        if (now_us == 0U || now_us < m_last_horizontal_fusion_us || at_rest < -1 || at_rest > 1 || clipping_mask < 0 ||
            clipping_mask > 7)
        {
            return std::nullopt;
        }
        return runtime::AccelerometerFusionConditions{
            enabled,
            m_tilt_aligned,
            horizontal_aiding(now_us),
            at_rest == 1,
            {(clipping_mask & 1) != 0, (clipping_mask & 2) != 0, (clipping_mask & 4) != 0}};
    }

private:
    value_type m_tilt_variance{};
    bool m_tilt_aligned = false;
    std::uint64_t m_last_horizontal_fusion_us{};
}; /* end class ReplayFusionConditions */

} /* end namespace formal_eskf::example */
