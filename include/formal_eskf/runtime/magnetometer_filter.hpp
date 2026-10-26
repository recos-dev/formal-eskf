/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <formal_eskf/linalg/operations.hpp>
#include <formal_eskf/status.hpp>

namespace formal_eskf::runtime
{

/**
 * Body-field LPF for heading reset, not normal magnetic correction.
 * Fixed alpha uses a nominal 10 ms interval and 90 ms time constant.
 * Update once per consumed magnetic sample; source/calibration changes reseed
 * from that sample. Reference-only changes retain measurement history.
 */
template <typename Linalg> class MagnetometerFilter
{
public:
    using value_type = typename Linalg::value_type;
    using vector_type = linalg::Matrix<Linalg, 3U, 1U>;

    [[nodiscard]] bool initialized() const noexcept { return m_initialized; }
    [[nodiscard]] vector_type const & field_b() const noexcept { return m_field_b; }
    void reset() noexcept { *this = MagnetometerFilter{}; }

    /** Invalid input/arithmetic preserves history, including a requested reseed. */
    [[nodiscard]] Status update(vector_type const & sample, bool reseed = false) noexcept;

private:
    vector_type m_field_b{};
    bool m_initialized = false;
}; /* end class MagnetometerFilter */

template <typename Linalg> Status MagnetometerFilter<Linalg>::update(vector_type const & sample, bool reseed) noexcept
{
    if (!linalg::all_finite(sample))
    {
        return Status::non_finite_input;
    }
    constexpr auto interval = static_cast<value_type>(0.01);
    constexpr auto time_constant = static_cast<value_type>(0.09);
    constexpr auto alpha = interval / (time_constant + interval);
    auto const candidate = !m_initialized || reseed ? sample : m_field_b + (sample - m_field_b) * alpha;
    if (!linalg::all_finite(candidate))
    {
        return Status::non_finite_result;
    }
    m_field_b = candidate;
    m_initialized = true;
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
