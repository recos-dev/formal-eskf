/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this file.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>

#include <formal_eskf/geodesy/ecef.hpp>

#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{
namespace geo = formal_eskf::geodesy;
using formal_eskf::Status;
using formal_eskf::test::Context;
using Backend = formal_eskf::test::Backend<double>;
using Vector = formal_eskf::linalg::Vector<Backend, 3U>;

constexpr double a = 6378137.0;
constexpr double b = 6356752.3142451793;
constexpr double tolerance = 1e-6;

struct Fixture
{
    geo::GeodeticPosition geodetic;
    std::array<double, 3U> ecef{};
};

// GeographicLib v2.5 CartConvert -p 9, WGS84. Metre values rounded to 1 nm.
// These fixtures are independent of the implementation and its inverse.
constexpr Fixture fixtures[]{
    {{25.033, 121.5654, 42.5}, {-3026977.303695336, 4926945.614071296, 2682405.021553170}},
    {{-33.8688, 151.2093, -340.0}, {-4645803.862133477, 2553070.379749518, -3534182.908276805}},
    {{48.8566, 2.3522, 100000.0}, {4266653.944407487, 175260.124533985, 4855387.865190784}},
    {{-45.0, -135.0, -12000.0}, {-3188419.145060575, -3188419.145060575, -4478863.127491682}},
    {{89.999999, 43.0, 75.0}, {0.081688763, 0.076176003, 6356827.314245178}},
    {{-89.999999, -173.0, 125.0}, {-0.110863595, -0.013612338, -6356877.314245178}},
    {{0.01, 179.999999, 1000.0}, {-6379136.903490510, 0.111336942, 1105.917285639}},
    {{-0.01, -179.999999, 20.0}, {-6378156.903505436, -0.111319838, -1105.746243373}},
    {{10.0, -80.0, 98765.0}, {1107725.578923823, -6282223.936876179, 1117398.910002636}},
    {{-20.0, 60.0, -11999.0}, {2992280.506069768, 5182781.867010750, -2163592.888128993}},
};

bool same(double left, double right)
{
    return std::bit_cast<std::uint64_t>(left) == std::bit_cast<std::uint64_t>(right);
}

bool same(geo::GeodeticPosition const & left, geo::GeodeticPosition const & right)
{
    return same(left.latitude_deg, right.latitude_deg) && same(left.longitude_deg, right.longitude_deg) &&
           same(left.height_ellipsoid_m, right.height_ellipsoid_m);
}

bool same(Vector const & left, Vector const & right)
{
    return same(left(0U), right(0U)) && same(left(1U), right(1U)) && same(left(2U), right(2U));
}

double distance(Vector const & point, std::array<double, 3U> const & reference)
{
    return std::hypot(point(0U) - reference[0U], point(1U) - reference[1U], point(2U) - reference[2U]);
}

// Long-double normal geometry is a secondary sampled oracle, not a uniform bound.
std::array<double, 3U> forward_reference(geo::GeodeticPosition const & point)
{
    long double const phi = static_cast<long double>(point.latitude_deg) * std::acos(-1.0L) / 180.0L;
    long double const lambda = static_cast<long double>(point.longitude_deg) * std::acos(-1.0L) / 180.0L;
    constexpr long double f = 1.0L / 298.257223563L;
    constexpr long double e2 = f * (2.0L - f);
    long double const sin_phi = std::sin(phi);
    long double const N = 6378137.0L / std::sqrt(1.0L - e2 * sin_phi * sin_phi);
    long double const radial = (N + point.height_ellipsoid_m) * std::cos(phi);
    return {static_cast<double>(radial * std::cos(lambda)), static_cast<double>(radial * std::sin(lambda)),
            static_cast<double>((N * (1.0L - e2) + point.height_ellipsoid_m) * sin_phi)};
}

void axes(Context & test, std::string_view profile)
{
    Fixture const axes[]{
        {{0.0, 0.0, 0.0}, {a, 0.0, 0.0}},
        {{0.0, 90.0, 100.0}, {0.0, a + 100.0, 0.0}},
        {{0.0, -90.0, -20.0}, {0.0, -a + 20.0, 0.0}},
        {{0.0, 180.0, 0.0}, {-a, 0.0, 0.0}},
        {{0.0, -180.0, 0.0}, {-a, 0.0, 0.0}},
        {{90.0, 123.0, 80.0}, {0.0, 0.0, b + 80.0}},
        {{-90.0, -73.0, -60.0}, {0.0, 0.0, -b + 60.0}},
    };
    for (auto const & fixture : axes)
    {
        Vector output;
        test.expect(geo::geodetic_to_ecef<Backend>(fixture.geodetic, output) == Status::success, profile,
                    "axis conversion succeeds");
        test.expect(distance(output, fixture.ecef) == 0.0, profile, "cardinal axis is exact");
        geo::GeodeticPosition result;
        test.expect(geo::ecef_to_geodetic<Backend>(output, result) == Status::success, profile,
                    "axis inverse succeeds");
        test.expect(std::abs(result.latitude_deg - fixture.geodetic.latitude_deg) < 1e-13 &&
                        std::abs(result.height_ellipsoid_m - fixture.geodetic.height_ellipsoid_m) <= tolerance,
                    profile, "axis latitude and height");
        double const longitude = std::abs(fixture.geodetic.latitude_deg) == 90.0 ? 0.0
                                 : fixture.geodetic.longitude_deg == 180.0       ? -180.0
                                                                                 : fixture.geodetic.longitude_deg;
        test.expect(result.longitude_deg == longitude, profile, "pole longitude and antimeridian convention");
    }
}

void reference(Context & test, std::string_view profile)
{
    for (auto const & fixture : fixtures)
    {
        Vector output;
        test.expect(geo::geodetic_to_ecef<Backend>(fixture.geodetic, output) == Status::success &&
                        distance(output, fixture.ecef) <= tolerance,
                    profile, "forward matches independent GeographicLib fixture");
        // Domain-boundary round trips need not preserve admission (F-GEO-ECEF).
        if (fixture.geodetic.height_ellipsoid_m == -12000.0 || fixture.geodetic.height_ellipsoid_m == 100000.0)
        {
            continue;
        }
        geo::GeodeticPosition result;
        Vector const input = Vector::from_row_major({fixture.ecef[0U], fixture.ecef[1U], fixture.ecef[2U]});
        test.expect(geo::ecef_to_geodetic<Backend>(input, result) == Status::success, profile,
                    "inverse independently supplied Cartesian fixture");
        test.expect(std::abs(result.height_ellipsoid_m - fixture.geodetic.height_ellipsoid_m) <= tolerance &&
                        distance(input, forward_reference(result)) <= tolerance,
                    profile, "inverse height and independent reconstruction");
        test.expect(result.longitude_deg >= -180.0 && result.longitude_deg < 180.0, profile,
                    "inverse longitude has canonical range");
    }
}

void grid(Context & test, std::string_view profile)
{
    for (double const latitude :
         {-89.999999999, -89.0, -60.0, -30.0, -0.001, 0.0, 0.001, 30.0, 60.0, 89.0, 89.999999999})
    {
        for (double const longitude : {-180.0, -179.999999999, -120.0, -45.0, 0.0, 45.0, 120.0, 179.999999999, 180.0})
        {
            for (double const height : {-11999.99, -430.0, 0.0, 8000.0, 99999.99})
            {
                geo::GeodeticPosition const input{latitude, longitude, height};
                auto const expected = forward_reference(input);
                Vector output;
                test.expect(geo::geodetic_to_ecef<Backend>(input, output) == Status::success &&
                                distance(output, expected) <= tolerance,
                            profile, "grid forward Cartesian error");
                geo::GeodeticPosition reverse;
                test.expect(
                    geo::ecef_to_geodetic<Backend>(Vector::from_row_major({expected[0U], expected[1U], expected[2U]}),
                                                   reverse) == Status::success &&
                        std::abs(reverse.height_ellipsoid_m - height) <= tolerance &&
                        distance(Vector::from_row_major({expected[0U], expected[1U], expected[2U]}),
                                 forward_reference(reverse)) <= tolerance,
                    profile, "grid inverse normal height and independently reconstructed position");
                test.expect(reverse.longitude_deg >= -180.0 && reverse.longitude_deg < 180.0, profile,
                            "grid canonical longitude");
            }
        }
    }
}

void forward_failure(Context & test, std::string_view profile)
{
    double const nan = std::numeric_limits<double>::quiet_NaN();
    double const inf = std::numeric_limits<double>::infinity();
    struct Invalid
    {
        geo::GeodeticPosition input;
        Status expected = Status::success;
    };
    Invalid const invalid[]{
        {{nan, 0.0, 0.0}, Status::non_finite_input},
        {{0.0, inf, 0.0}, Status::non_finite_input},
        {{0.0, 0.0, -inf}, Status::non_finite_input},
        {{91.0, 181.0, nan}, Status::non_finite_input},
        {{91.0, 181.0, 100001.0}, Status::domain_error},
        {{std::nextafter(90.0, inf), 0.0, 0.0}, Status::domain_error},
        {{std::nextafter(-90.0, -inf), 0.0, 0.0}, Status::domain_error},
        {{0.0, std::nextafter(180.0, inf), 0.0}, Status::domain_error},
        {{0.0, std::nextafter(-180.0, -inf), 0.0}, Status::domain_error},
        {{0.0, 0.0, std::nextafter(-12000.0, -inf)}, Status::out_of_range},
        {{0.0, 0.0, std::nextafter(100000.0, inf)}, Status::out_of_range},
    };
    for (auto const & fixture : invalid)
    {
        Vector const before = Vector::from_row_major({nan, -0.0, inf});
        Vector output = before;
        auto input = fixture.input;
        test.expect(geo::geodetic_to_ecef<Backend>(input, output) == fixture.expected, profile,
                    "forward invalid precedence");
        test.expect(same(output, before) && same(input, fixture.input), profile,
                    "forward failure preserves outputs and inputs bitwise");
    }
    for (double const height : {-12000.0, std::nextafter(-12000.0, 0.0), std::nextafter(100000.0, 0.0), 100000.0})
    {
        Vector output;
        test.expect(geo::geodetic_to_ecef<Backend>({0.0, 0.0, height}, output) == Status::success, profile,
                    "forward height endpoints and adjacent interior accepted");
    }
}

void reverse_failure(Context & test, std::string_view profile)
{
    double const nan = std::numeric_limits<double>::quiet_NaN();
    double const inf = std::numeric_limits<double>::infinity();
    double const max = std::numeric_limits<double>::max();
    struct Invalid
    {
        std::array<double, 3U> input{};
        Status expected = Status::success;
    };
    Invalid const invalid[]{
        {{nan, 0.0, 0.0}, Status::non_finite_input},
        {{0.0, inf, 0.0}, Status::non_finite_input},
        {{max, 0.0, -inf}, Status::non_finite_input},
        {{0.0, -0.0, 0.0}, Status::domain_error},
        {{1.0, 2.0, 3.0}, Status::out_of_range},
        {{max, max, max}, Status::out_of_range},
        {{std::nextafter(a - 12000.0, 0.0), 0.0, 0.0}, Status::out_of_range},
        {{std::nextafter(a + 100000.0, inf), 0.0, 0.0}, Status::out_of_range},
        {{0.0, 0.0, std::nextafter(b - 12000.0, 0.0)}, Status::out_of_range},
        {{0.0, 0.0, std::nextafter(b + 100000.0, inf)}, Status::out_of_range},
    };
    for (auto const & fixture : invalid)
    {
        Vector const before = Vector::from_row_major({fixture.input[0U], fixture.input[1U], fixture.input[2U]});
        Vector input = before;
        geo::GeodeticPosition const sentinel{nan, -0.0, inf};
        auto output = sentinel;
        test.expect(geo::ecef_to_geodetic<Backend>(input, output) == fixture.expected, profile,
                    "reverse invalid precedence and axis height admission");
        test.expect(same(output, sentinel) && same(input, before), profile,
                    "reverse failure preserves outputs and inputs bitwise");
    }
    for (double const height : {-12000.0, 100000.0})
    {
        for (auto const & input :
             {Vector::from_row_major({a + height, 0.0, 0.0}), Vector::from_row_major({0.0, 0.0, b + height})})
        {
            geo::GeodeticPosition output;
            test.expect(geo::ecef_to_geodetic<Backend>(input, output) == Status::success &&
                            output.height_ellipsoid_m == height,
                        profile, "exact axis height endpoints accepted");
        }
    }
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Case;
    Case const cases[]{
        {"TC-F-GEO-ECEF-AXES", "binary64", axes},
        {"TC-F-GEO-ECEF-REFERENCE", "partial.binary64", reference},
        {"TC-F-GEO-ECEF-GRID", "partial.binary64", grid},
        {"TC-F-GEO-ECEF-FORWARD-FAILURE", "binary64", forward_failure},
        {"TC-F-GEO-ECEF-REVERSE-FAILURE", "binary64", reverse_failure},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
