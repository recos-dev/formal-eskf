/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <numbers>

#include <formal_eskf/scalar/math.hpp>
#include <formal_eskf/so3/operations.hpp>

namespace formal_eskf::runtime
{

/** Latch alignment from NED tilt variance; no clock or aiding-source ownership. */
template <typename Linalg> class TiltAlignment
{
public:
    using value_type = typename Linalg::value_type;
    static constexpr value_type tilt_limit = std::numbers::pi_v<value_type> / value_type{60};

    /** Unit q_nb and symmetric PSD body-local attitude covariance are caller premises. */
    [[nodiscard]] Status update(so3::UnitQuaternion<Linalg> const & q_nb,
                                linalg::Matrix<Linalg, 3U, 3U> const & P_theta_b);

    bool aligned() const { return m_aligned; }
    value_type variance() const { return m_variance; }

private:
    value_type m_variance{};
    bool m_aligned = false;
}; /* end class TiltAlignment */

template <typename Linalg>
Status TiltAlignment<Linalg>::update(so3::UnitQuaternion<Linalg> const & q_nb,
                                     linalg::Matrix<Linalg, 3U, 3U> const & P_theta_b)
{
    if (!linalg::all_finite(q_nb.coefficients()) || !linalg::all_finite(P_theta_b))
    {
        return Status::non_finite_input;
    }
    auto const R_nb = so3::to_rotation_matrix(q_nb);
    auto const P_theta_n = R_nb * P_theta_b * linalg::transpose(R_nb);
    auto const tilt_variance = P_theta_n(0U, 0U) + P_theta_n(1U, 1U);
    if (!linalg::all_finite(P_theta_n) || !scalar::is_finite<typename Linalg::scalar_math_type>(tilt_variance))
    {
        return Status::non_finite_result;
    }
    if (P_theta_n(0U, 0U) < value_type{0} || P_theta_n(1U, 1U) < value_type{0})
    {
        return Status::domain_error;
    }
    m_variance = tilt_variance;
    m_aligned = m_aligned || tilt_variance < tilt_limit * tilt_limit;
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
