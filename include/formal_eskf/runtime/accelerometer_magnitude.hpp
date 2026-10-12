/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <formal_eskf/linalg/operations.hpp>
#include <formal_eskf/scalar/math.hpp>
#include <formal_eskf/status.hpp>

namespace formal_eskf::runtime
{

/**
 * Filter body-frame specific force, then check its magnitude against gravity.
 * Input is delta_vel / delta_vel_dt in m/s^2, before estimator bias subtraction.
 * Fixed alpha uses a 10 ms nominal interval and 90 ms time constant; it is NOT
 * recomputed per sample. Update cadence remains the caller's responsibility.
 * This is an amplitude diagnostic, not permission to fuse or evidence of rest.
 */
template <typename Linalg> class AccelerometerMagnitude
{
public:
    using value_type = typename Linalg::value_type;
    using vector_type = linalg::Matrix<Linalg, 3U, 1U>;

    struct Result
    {
        Status status = Status::success;
        bool within_gravity_band = false;
        value_type squared_norm{}; // m^2/s^4; valid only on success.
    };

    /** Clear history on source/calibration/frame changes; next valid sample seeds the filter. */
    void reset() noexcept { *this = AccelerometerMagnitude{}; }

    [[nodiscard]] bool initialized() const noexcept { return m_initialized; }
    [[nodiscard]] vector_type const & filtered_specific_force_b() const noexcept { return m_filtered_specific_force_b; }

    /** First valid sample seeds all axes. Failure preserves history and returns no valid diagnostics. */
    [[nodiscard]] Result update(vector_type const & specific_force_b) noexcept;

private:
    vector_type m_filtered_specific_force_b{};
    bool m_initialized = false;
}; /* end class AccelerometerMagnitude */

template <typename Linalg>
typename AccelerometerMagnitude<Linalg>::Result
AccelerometerMagnitude<Linalg>::update(vector_type const & specific_force_b) noexcept
{
    using math_type = typename Linalg::scalar_math_type;
    constexpr auto sample_interval = static_cast<value_type>(0.01);
    constexpr auto time_constant = static_cast<value_type>(0.09);
    constexpr auto alpha = sample_interval / (time_constant + sample_interval);
    constexpr auto gravity = static_cast<value_type>(9.80665);
    constexpr auto lower_limit = gravity * static_cast<value_type>(0.9);
    constexpr auto upper_limit = gravity * static_cast<value_type>(1.1);

    Result result;
    if (!linalg::all_finite(specific_force_b))
    {
        result.status = Status::non_finite_input;
        return result;
    }
    auto const candidate = m_initialized
                               ? m_filtered_specific_force_b + (specific_force_b - m_filtered_specific_force_b) * alpha
                               : specific_force_b;
    auto const squared_norm = linalg::squared_norm(candidate);
    if (!linalg::all_finite(candidate) || !scalar::is_finite<math_type>(squared_norm))
    {
        result.status = Status::non_finite_result;
        return result;
    }

    m_filtered_specific_force_b = candidate;
    m_initialized = true;
    result.squared_norm = squared_norm;
    result.within_gravity_band = squared_norm > lower_limit * lower_limit && squared_norm < upper_limit * upper_limit;
    return result;
}

} /* end namespace formal_eskf::runtime */
