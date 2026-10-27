/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <limits>

#include <formal_eskf/so3/operations.hpp>

namespace formal_eskf::runtime
{

/** Heading consistency history, separate from field plausibility and innovation gating. */
template <typename Linalg> class MagneticHeading
{
public:
    using value_type = typename Linalg::value_type;
    using vector_type = linalg::Matrix<Linalg, 3U, 1U>;
    using horizontal_type = linalg::Matrix<Linalg, 2U, 1U>;

    struct Parameters
    {
        value_type heading_noise{};            // Positive radians, at most pi; strict consistency threshold.
        value_type acceleration_limit{};       // Nonnegative m/s^2; strict observability threshold.
        value_type minimum_horizontal_field{}; // Positive normal microtesla.
    };

    struct State
    {
        value_type innovation{}; // Wrapped predicted-minus-measured heading, radians.
        value_type filtered_innovation{};
        value_type horizontal_acceleration{}; // Norm of the filtered N/E acceleration, m/s^2.
        bool innovation_valid = false;
        bool observable = false; // Current aiding + maneuver condition, not a mathematical proof.
        bool consistent = false; // Retained through low excitation until inconsistency is observed.
    };

    [[nodiscard]] State const & state() const noexcept { return m_state; }
    void reset() noexcept { *this = MagneticHeading{}; }
    /** Missing IMU intervals provide no maneuver evidence. Preserve the heading latch. */
    void reset_acceleration() noexcept
    {
        m_acceleration = horizontal_type{};
        m_state.horizontal_acceleration = value_type{0};
        m_state.observable = false;
    }
    /** Call only after an actual magnetic heading alignment/reset succeeds. */
    void reset_heading() noexcept
    {
        m_state.innovation = value_type{0};
        m_state.filtered_innovation = value_type{0};
        m_state.innovation_valid = false;
        m_state.consistent = true;
    }

    /** Bias-corrected specific force rotated into NED at prediction; only N/E are filtered. */
    [[nodiscard]] Status update_acceleration(vector_type const & acceleration_n, value_type dt) noexcept;

    /**
     * Unit q_nb, calibrated m_b and trustworthy m_n at one horizon. Update once
     * per consumed magnetic sample, even while navigation fusion is inhibited.
     * No learned magnetic bias is subtracted. Low horizontal field clears the
     * latch without inventing a heading. Unaligned heading clears its history.
     * Invalid inputs/arithmetic preserve history; callers must handle failure.
     */
    [[nodiscard]] Status update(so3::UnitQuaternion<Linalg> const & q_nb, vector_type const & m_b,
                                vector_type const & m_n, Parameters const & parameters, bool heading_aligned,
                                bool horizontal_aiding) noexcept;

private:
    horizontal_type m_acceleration{};
    State m_state{};
}; /* end class MagneticHeading */

template <typename Linalg>
Status MagneticHeading<Linalg>::update_acceleration(vector_type const & acceleration_n, value_type dt) noexcept
{
    using math_type = typename Linalg::scalar_math_type;
    if (!linalg::all_finite(acceleration_n) || !scalar::is_finite<math_type>(dt))
    {
        return Status::non_finite_input;
    }
    if (dt < std::numeric_limits<value_type>::min())
    {
        return Status::domain_error;
    }
    // A one-second vector LPF: opposing accelerations cancel before taking the norm.
    auto const alpha = dt / (value_type{1} + dt);
    auto const candidate = m_acceleration + (acceleration_n.template block<0U, 0U, 2U, 1U>() - m_acceleration) * alpha;
    auto const squared_norm = linalg::dot(candidate, candidate);
    if (!linalg::all_finite(candidate) || !scalar::is_finite<math_type>(squared_norm))
    {
        return Status::non_finite_result;
    }
    value_type magnitude{};
    auto const status = scalar::try_sqrt<math_type>(squared_norm, magnitude);
    if (succeeded(status))
    {
        m_acceleration = candidate;
        m_state.horizontal_acceleration = magnitude;
        m_state.observable = false; // Recomputed with aiding on the next magnetic sample.
    }
    return status;
}

template <typename Linalg>
Status MagneticHeading<Linalg>::update(so3::UnitQuaternion<Linalg> const & q_nb, vector_type const & m_b,
                                       vector_type const & m_n, Parameters const & parameters, bool heading_aligned,
                                       bool horizontal_aiding) noexcept
{
    using math_type = typename Linalg::scalar_math_type;
    if (!linalg::all_finite(q_nb.coefficients()) || !linalg::all_finite(m_b) || !linalg::all_finite(m_n) ||
        !scalar::is_finite<math_type>(parameters.heading_noise) ||
        !scalar::is_finite<math_type>(parameters.acceleration_limit) ||
        !scalar::is_finite<math_type>(parameters.minimum_horizontal_field))
    {
        return Status::non_finite_input;
    }
    auto const pi = scalar::pi<math_type>();
    if (!(parameters.heading_noise > value_type{0}) || parameters.heading_noise > pi ||
        parameters.acceleration_limit < value_type{0} ||
        parameters.minimum_horizontal_field < std::numeric_limits<value_type>::min())
    {
        return Status::domain_error;
    }
    auto candidate = m_state;
    candidate.observable = horizontal_aiding && m_state.horizontal_acceleration > parameters.acceleration_limit;
    candidate.innovation_valid = false;
    candidate.innovation = value_type{0};
    if (!heading_aligned)
    {
        candidate.filtered_innovation = value_type{0};
        candidate.consistent = false;
        m_state = candidate;
        return Status::success;
    }
    auto const measured_n = so3::rotate(q_nb, m_b);
    auto const measured_horizontal = linalg::norm(measured_n.template block<0U, 0U, 2U, 1U>());
    auto const reference_horizontal = linalg::norm(m_n.template block<0U, 0U, 2U, 1U>());
    if (!linalg::all_finite(measured_n) || !scalar::is_finite<math_type>(measured_horizontal) ||
        !scalar::is_finite<math_type>(reference_horizontal))
    {
        return Status::non_finite_result;
    }
    if (measured_horizontal < parameters.minimum_horizontal_field ||
        reference_horizontal < parameters.minimum_horizontal_field)
    {
        candidate.consistent = false;
        m_state = candidate;
        return Status::success;
    }
    value_type measured_heading{}, reference_heading{};
    auto status = scalar::try_atan2<math_type>(measured_n(1U), measured_n(0U), measured_heading);
    if (!succeeded(status))
    {
        return status;
    }
    status = scalar::try_atan2<math_type>(m_n(1U), m_n(0U), reference_heading);
    if (!succeeded(status))
    {
        return status;
    }
    auto innovation = measured_heading - reference_heading;
    if (innovation > pi)
    {
        innovation -= value_type{2} * pi;
    }
    else if (innovation < -pi)
    {
        innovation += value_type{2} * pi;
    }
    // Nominal 10 ms / 90 ms LPF, applied once per magnetic sample, not per IMU tick.
    constexpr auto interval = static_cast<value_type>(0.01);
    constexpr auto time_constant = static_cast<value_type>(0.09);
    candidate.innovation = innovation;
    candidate.innovation_valid = true;
    candidate.filtered_innovation +=
        (innovation - candidate.filtered_innovation) * (interval / (time_constant + interval));
    if (scalar::absolute<math_type>(candidate.filtered_innovation) >= parameters.heading_noise)
    {
        candidate.consistent = false;
    }
    else if (candidate.observable)
    {
        candidate.consistent = true;
    }
    m_state = candidate;
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
