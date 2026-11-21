/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this file.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <limits>
#include <type_traits>

#include <formal_eskf/linalg/operations.hpp>
#include <formal_eskf/scalar/math.hpp>

namespace formal_eskf::geodesy
{

/** WGS84 geodetic angles in degrees and ellipsoidal (not MSL) height in metres. */
struct GeodeticPosition
{
    double latitude_deg{};
    double longitude_deg{};
    double height_ellipsoid_m{};
};

namespace detail
{

inline constexpr double semi_major_axis_m = 6378137.0;
inline constexpr double flattening = 1.0 / 298.257223563;
inline constexpr double eccentricity_squared = flattening * (2.0 - flattening);
inline constexpr double semi_minor_axis_m = semi_major_axis_m * (1.0 - flattening);
inline constexpr double minimum_height_m = -12000.0;
inline constexpr double maximum_height_m = 100000.0;

/** Caller supplies a validated angle in [-180, 180] degrees. Keep cardinal axes exact. */
template <typename Math> scalar::SinCos<double> sin_cos_degrees(double angle)
{
    if (angle == 0.0)
    {
        return {angle, 1.0};
    }
    if (angle == 90.0 || angle == -90.0)
    {
        return {angle / 90.0, 0.0};
    }
    if (angle == 180.0 || angle == -180.0)
    {
        return {0.0, -1.0};
    }
    double const radians = angle * (scalar::pi<Math>() / 180.0);
    return {Math::sin(radians), Math::cos(radians)};
}

template <typename Linalg>
inline constexpr bool binary64 =
    std::is_same_v<typename Linalg::value_type, double> && std::numeric_limits<double>::is_iec559 &&
            std::numeric_limits<double>::radix == 2 &&
    std::numeric_limits<double>::digits == 53 && std::numeric_limits<double>::max_exponent == 1024;

} /* end namespace detail */

/** F-GEO-ECEF: checked forward conversion; failure leaves position_e unchanged. */
template <typename Linalg>
[[nodiscard]] Status geodetic_to_ecef(GeodeticPosition const & position, linalg::Vector<Linalg, 3U> & position_e)
{
    static_assert(detail::binary64<Linalg>, "Geodesy requires IEEE-754 binary64 double");
    using math_type = typename Linalg::scalar_math_type;
    if (!scalar::is_finite<math_type>(position.latitude_deg) || !scalar::is_finite<math_type>(position.longitude_deg) ||
        !scalar::is_finite<math_type>(position.height_ellipsoid_m))
    {
        return Status::non_finite_input;
    }
    if (position.latitude_deg < -90.0 || position.latitude_deg > 90.0 || position.longitude_deg < -180.0 ||
        position.longitude_deg > 180.0)
    {
        return Status::domain_error;
    }
    if (position.height_ellipsoid_m < detail::minimum_height_m ||
        position.height_ellipsoid_m > detail::maximum_height_m)
    {
        return Status::out_of_range;
    }

    linalg::Vector<Linalg, 3U> candidate;
    if (position.latitude_deg == 90.0 || position.latitude_deg == -90.0)
    {
        candidate(2U) = (position.latitude_deg / 90.0) * (detail::semi_minor_axis_m + position.height_ellipsoid_m);
    }
    else
    {
        auto const latitude = detail::sin_cos_degrees<math_type>(position.latitude_deg);
        auto const longitude = detail::sin_cos_degrees<math_type>(position.longitude_deg);
        double const N = detail::semi_major_axis_m /
                         math_type::sqrt(1.0 - detail::eccentricity_squared * latitude.sine * latitude.sine);
        double const radial = (N + position.height_ellipsoid_m) * latitude.cosine;
        candidate(0U) = radial * longitude.cosine;
        candidate(1U) = radial * longitude.sine;
        candidate(2U) = (N * (1.0 - detail::eccentricity_squared) + position.height_ellipsoid_m) * latitude.sine;
    }
    if (!linalg::all_finite(candidate))
    {
        return Status::non_finite_result;
    }
    position_e = candidate;
    return Status::success;
}

/**
 * F-GEO-ECEF: bounded near-surface inverse, with longitude in [-180, 180).
 * From the forward equations, phi = atan2(z + e2*N(phi)*sin(phi), hypot(x,y)).
 * Perform eight fixed-point refinements. Project onto the ellipsoid normal for
 * height; dividing by cos(phi) or sin(phi) would be ill-conditioned at an axis.
 * Failure leaves all geodetic fields unchanged.
 */
template <typename Linalg>
[[nodiscard]] Status ecef_to_geodetic(linalg::Vector<Linalg, 3U> const & position_e, GeodeticPosition & position)
{
    static_assert(detail::binary64<Linalg>, "Geodesy requires IEEE-754 binary64 double");
    using math_type = typename Linalg::scalar_math_type;
    if (!linalg::all_finite(position_e))
    {
        return Status::non_finite_input;
    }
    double const x = position_e(0U), y = position_e(1U), z = position_e(2U);
    if (x == 0.0 && y == 0.0 && z == 0.0)
    {
        return Status::domain_error;
    }

    // Necessary near-surface bounds reject remote inputs before squaring.
    // They also exclude the interior region where geodetic inverses are ambiguous.
    constexpr double maximum_radius = detail::semi_major_axis_m + detail::maximum_height_m;
    constexpr double minimum_radius = detail::semi_minor_axis_m + detail::minimum_height_m;
    if (linalg::max_abs(position_e) > maximum_radius)
    {
        return Status::out_of_range;
    }
    double const horizontal_squared = x * x + y * y;
    double const radius_squared = horizontal_squared + z * z;
    if (radius_squared < minimum_radius * minimum_radius || radius_squared > maximum_radius * maximum_radius)
    {
        return Status::out_of_range;
    }

    GeodeticPosition candidate;
    if (x == 0.0 && y == 0.0)
    {
        candidate.latitude_deg = z > 0.0 ? 90.0 : -90.0;
        candidate.height_ellipsoid_m = scalar::absolute<math_type>(z) - detail::semi_minor_axis_m;
    }
    else
    {
        double const horizontal = math_type::sqrt(horizontal_squared);
        double latitude = math_type::atan2(z, horizontal);
        if (!scalar::is_finite<math_type>(horizontal) || !scalar::is_finite<math_type>(latitude))
        {
            return Status::non_finite_result;
        }
        for (unsigned iteration = 0U; iteration < 8U; ++iteration)
        {
            double const sine = math_type::sin(latitude);
            double const N =
                detail::semi_major_axis_m / math_type::sqrt(1.0 - detail::eccentricity_squared * sine * sine);
            double const vertical = z + detail::eccentricity_squared * N * sine;
            if (!scalar::is_finite<math_type>(N) || !scalar::is_finite<math_type>(vertical))
            {
                return Status::non_finite_result;
            }
            latitude = math_type::atan2(vertical, horizontal);
        }
        double const sine = math_type::sin(latitude), cosine = math_type::cos(latitude);
        double const surface_normal =
            detail::semi_major_axis_m * math_type::sqrt(1.0 - detail::eccentricity_squared * sine * sine);
        candidate.latitude_deg = latitude * (180.0 / scalar::pi<math_type>());
        candidate.longitude_deg = math_type::atan2(y, x) * (180.0 / scalar::pi<math_type>());
        candidate.height_ellipsoid_m = horizontal * cosine + z * sine - surface_normal;
        if (candidate.longitude_deg >= 180.0)
        {
            candidate.longitude_deg -= 360.0;
        }
    }
    if (!scalar::is_finite<math_type>(candidate.latitude_deg) ||
        !scalar::is_finite<math_type>(candidate.longitude_deg) ||
        !scalar::is_finite<math_type>(candidate.height_ellipsoid_m))
    {
        return Status::non_finite_result;
    }
    if (candidate.height_ellipsoid_m < detail::minimum_height_m ||
        candidate.height_ellipsoid_m > detail::maximum_height_m)
    {
        return Status::out_of_range;
    }
    position = candidate;
    return Status::success;
}

} /* end namespace formal_eskf::geodesy */
