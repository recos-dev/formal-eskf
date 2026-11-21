/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this file.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <formal_eskf/geodesy/ecef.hpp>

namespace formal_eskf::geodesy
{

/** F-GEO-NED: one owned fixed-origin frame. Changing it does not rebase an estimator. */
template <typename Linalg> class LocalNed
{
    static_assert(detail::binary64<Linalg>, "Geodesy requires IEEE-754 binary64 double");

public:
    using vector_type = linalg::Vector<Linalg, 3U>;
    using matrix_type = linalg::Matrix<Linalg, 3U, 3U>;

    [[nodiscard]] Status set_origin(GeodeticPosition const & origin);
    bool initialized() const { return m_initialized; }
    [[nodiscard]] Status ecef_to_ned(vector_type const & position_e, vector_type & position_n) const;
    [[nodiscard]] Status ned_to_ecef(vector_type const & position_n, vector_type & position_e) const;
    [[nodiscard]] Status rotation_from_ecef(matrix_type & C_ne) const;

private:
    vector_type m_origin_e{};
    matrix_type m_C_ne{};
    bool m_initialized = false;
}; /* end class LocalNed */

template <typename Linalg> Status LocalNed<Linalg>::set_origin(GeodeticPosition const & origin)
{
    vector_type origin_e;
    Status const status = geodetic_to_ecef<Linalg>(origin, origin_e);
    if (!succeeded(status))
    {
        return status;
    }
    using math_type = typename Linalg::scalar_math_type;
    auto const latitude = detail::sin_cos_degrees<math_type>(origin.latitude_deg);
    auto const longitude = detail::sin_cos_degrees<math_type>(origin.longitude_deg);
    auto const C_ne = matrix_type::from_row_major({-latitude.sine * longitude.cosine, -latitude.sine * longitude.sine,
                                                   latitude.cosine, -longitude.sine, longitude.cosine, 0.0,
                                                   -latitude.cosine * longitude.cosine,
                                                   -latitude.cosine * longitude.sine, -latitude.sine});
    if (!linalg::all_finite(C_ne))
    {
        return Status::non_finite_result;
    }
    m_origin_e = origin_e;
    m_C_ne = C_ne;
    m_initialized = true;
    return Status::success;
}

template <typename Linalg>
Status LocalNed<Linalg>::ecef_to_ned(vector_type const & position_e, vector_type & position_n) const
{
    if (!linalg::all_finite(position_e))
    {
        return Status::non_finite_input;
    }
    if (!m_initialized)
    {
        return Status::domain_error;
    }
    vector_type const candidate = m_C_ne * (position_e - m_origin_e);
    if (!linalg::all_finite(candidate))
    {
        return Status::non_finite_result;
    }
    position_n = candidate;
    return Status::success;
}

template <typename Linalg>
Status LocalNed<Linalg>::ned_to_ecef(vector_type const & position_n, vector_type & position_e) const
{
    if (!linalg::all_finite(position_n))
    {
        return Status::non_finite_input;
    }
    if (!m_initialized)
    {
        return Status::domain_error;
    }
    vector_type const candidate = m_origin_e + linalg::transpose(m_C_ne) * position_n;
    if (!linalg::all_finite(candidate))
    {
        return Status::non_finite_result;
    }
    position_e = candidate;
    return Status::success;
}

template <typename Linalg> Status LocalNed<Linalg>::rotation_from_ecef(matrix_type & C_ne) const
{
    if (!m_initialized)
    {
        return Status::domain_error;
    }
    C_ne = m_C_ne;
    return Status::success;
}

} /* end namespace formal_eskf::geodesy */
