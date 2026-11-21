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

#include <formal_eskf/geodesy/local_ned.hpp>

#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{
namespace geo = formal_eskf::geodesy;
namespace la = formal_eskf::linalg;
using formal_eskf::Status;
using formal_eskf::test::Context;
using Backend = formal_eskf::test::Backend<double>;
using Frame = geo::LocalNed<Backend>;
using Vector = Frame::vector_type;
using Matrix = Frame::matrix_type;

bool same(double left, double right)
{
    return std::bit_cast<std::uint64_t>(left) == std::bit_cast<std::uint64_t>(right);
}

template <typename M> bool same(M const & left, M const & right)
{
    for (std::size_t row = 0U; row < M::row_count; ++row)
    {
        for (std::size_t column = 0U; column < M::column_count; ++column)
        {
            if (!same(left(row, column), right(row, column)))
            {
                return false;
            }
        }
    }
    return true;
}

void axes(Context & test, std::string_view profile)
{
    Frame frame;
    test.expect(frame.set_origin({0.0, 0.0, 0.0}) == Status::success, profile, "equatorial origin");
    Matrix C;
    test.expect(frame.rotation_from_ecef(C) == Status::success &&
                    la::max_abs(C - Matrix::from_row_major({0, 0, 1, 0, 1, 0, -1, 0, 0})) == 0.0,
                profile, "ECEF z is North, y is East, x is Up");
    Vector const ecef = Vector::from_row_major({6378140.0, 5.0, 7.0});
    Vector local;
    test.expect(frame.ecef_to_ned(ecef, local) == Status::success &&
                    la::max_abs(local - Vector::from_row_major({7.0, 5.0, -3.0})) == 0.0,
                profile, "position translates before rotating");
    Vector result;
    test.expect(frame.ned_to_ecef(local, result) == Status::success && same(result, ecef), profile,
                "inverse applies transpose and origin");
    Vector aliased = ecef;
    test.expect(frame.ecef_to_ned(aliased, aliased) == Status::success && same(aliased, local), profile,
                "forward input/output may alias");
    test.expect(frame.ned_to_ecef(aliased, aliased) == Status::success && same(aliased, ecef), profile,
                "reverse input/output may alias");

    test.expect(frame.set_origin({90.0, 90.0, 0.0}) == Status::success &&
                    frame.rotation_from_ecef(C) == Status::success &&
                    la::max_abs(C - Matrix::from_row_major({0, -1, 0, -1, 0, 0, 0, 0, -1})) == 0.0,
                profile, "north polar origin retains caller longitude");
    test.expect(frame.set_origin({-90.0, 90.0, 0.0}) == Status::success &&
                    frame.rotation_from_ecef(C) == Status::success &&
                    la::max_abs(C - Matrix::from_row_major({0, 1, 0, -1, 0, 0, 0, 0, 1})) == 0.0,
                profile, "south polar handedness");
}

void geometry(Context & test, std::string_view profile)
{
    for (geo::GeodeticPosition const origin : {geo::GeodeticPosition{25.033, 121.5654, 42.5},
                                               {-33.8688, 151.2093, -340.0},
                                               {89.999999, -179.999999, 500.0},
                                               {-89.999999, 179.999999, -11999.0}})
    {
        // Independent long-double construction: Up is the ellipsoid normal,
        // East is the longitude tangent, North = Up x East.
        long double const phi = origin.latitude_deg * std::acos(-1.0L) / 180.0L;
        long double const lambda = origin.longitude_deg * std::acos(-1.0L) / 180.0L;
        std::array<long double, 3U> const up{std::cos(phi) * std::cos(lambda), std::cos(phi) * std::sin(lambda),
                                             std::sin(phi)};
        std::array<long double, 3U> const east{-std::sin(lambda), std::cos(lambda), 0.0L};
        std::array<long double, 3U> const north{up[1U] * east[2U] - up[2U] * east[1U],
                                                up[2U] * east[0U] - up[0U] * east[2U],
                                                up[0U] * east[1U] - up[1U] * east[0U]};
        std::array<std::array<long double, 3U>, 3U> const basis{north, east, {-up[0U], -up[1U], -up[2U]}};
        constexpr long double f = 1.0L / 298.257223563L;
        constexpr long double e2 = f * (2.0L - f);
        long double const N = 6378137.0L / std::sqrt(1.0L - e2 * up[2U] * up[2U]);
        std::array<long double, 3U> const origin_e{(N + origin.height_ellipsoid_m) * up[0U],
                                                   (N + origin.height_ellipsoid_m) * up[1U],
                                                   (N * (1.0L - e2) + origin.height_ellipsoid_m) * up[2U]};
        Frame frame;
        Matrix C;
        test.expect(frame.set_origin(origin) == Status::success && frame.rotation_from_ecef(C) == Status::success,
                    profile, "nontrivial origin initialization");
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            for (std::size_t column = 0U; column < 3U; ++column)
            {
                test.expect(std::abs(C(row, column) - basis[row][column]) <= 1e-14L, profile,
                            "basis matches independent normal/tangent construction");
            }
        }
        test.expect(la::max_abs(C * la::transpose(C) - Matrix::identity()) <= 3e-14, profile, "orthogonality residual");
        double const determinant = C(0, 0) * (C(1, 1) * C(2, 2) - C(1, 2) * C(2, 1)) -
                                   C(0, 1) * (C(1, 0) * C(2, 2) - C(1, 2) * C(2, 0)) +
                                   C(0, 2) * (C(1, 0) * C(2, 1) - C(1, 1) * C(2, 0));
        test.expect(std::abs(determinant - 1.0) <= 3e-14, profile, "proper rotation, no reflection");
        for (Vector const & input :
             {Vector::from_row_major({6378137.0, 0.0, 0.0}),
              Vector::from_row_major({-3026977.303695336, 4926945.614071296, 2682405.021553170})})
        {
            Vector local;
            test.expect(frame.ecef_to_ned(input, local) == Status::success, profile, "affine forward succeeds");
            long double error_squared = 0.0L;
            for (std::size_t row = 0U; row < 3U; ++row)
            {
                long double reference = 0.0L;
                for (std::size_t column = 0U; column < 3U; ++column)
                {
                    reference += basis[row][column] * (input(column) - origin_e[column]);
                }
                long double const error = local(row) - reference;
                error_squared += error * error;
            }
            test.expect(std::sqrt(error_squared) <= 1e-6L, profile, "represented ECEF input: affine forward budget");
            Vector output;
            test.expect(frame.ned_to_ecef(local, output) == Status::success, profile, "affine inverse succeeds");
            error_squared = 0.0L;
            for (std::size_t row = 0U; row < 3U; ++row)
            {
                long double reference = origin_e[row];
                for (std::size_t column = 0U; column < 3U; ++column)
                {
                    reference += basis[column][row] * local(column);
                }
                long double const error = output(row) - reference;
                error_squared += error * error;
            }
            test.expect(std::sqrt(error_squared) <= 1e-6L, profile, "represented NED input: affine inverse budget");
        }
    }
}

void curvature_and_transport(Context & test, std::string_view profile)
{
    Frame origin, sample;
    test.expect(origin.set_origin({0.0, 0.0, 0.0}) == Status::success &&
                    sample.set_origin({0.0, 90.0, 0.0}) == Status::success,
                profile, "two distinct tangent frames");
    Vector local;
    test.expect(origin.ecef_to_ned(Vector::from_row_major({0.0, 6378137.0, 0.0}), local) == Status::success &&
                    local(2U) == 6378137.0,
                profile, "equal ellipsoidal heights do not imply zero Down");
    Matrix C0, Cs;
    test.expect(origin.rotation_from_ecef(C0) == Status::success && sample.rotation_from_ecef(Cs) == Status::success,
                profile, "retrieve both bases");
    Matrix const T = C0 * la::transpose(Cs);
    Vector const velocity = Vector::from_row_major({2.0, 3.0, 4.0});
    test.expect(la::max_abs(T * velocity - Vector::from_row_major({2.0, -4.0, 3.0})) == 0.0, profile,
                "sample NED vector needs basis transport without translation");
    Matrix const P = Matrix::from_row_major({4, 1, 2, 1, 9, 3, 2, 3, 16});
    Matrix const expected = Matrix::from_row_major({4, -2, 1, -2, 16, -3, 1, -3, 9});
    test.expect(la::max_abs(T * P * la::transpose(T) - expected) == 0.0, profile,
                "correlated covariance rotates both indices");
}

void lifecycle(Context & test, std::string_view profile)
{
    Frame frame;
    double const nan = std::numeric_limits<double>::quiet_NaN();
    Vector const sentinel = Vector::from_row_major({nan, -0.0, 7.0});
    Vector output = sentinel;
    Matrix basis = Matrix::from_row_major({nan, -0.0, 1, 2, 3, 4, 5, 6, 7});
    Matrix const saved_basis = basis;
    test.expect(!frame.initialized() && frame.rotation_from_ecef(basis) == Status::domain_error &&
                    same(basis, saved_basis),
                profile, "default frame rejects basis retrieval without mutation");
    test.expect(frame.ecef_to_ned(Vector{}, output) == Status::domain_error && same(output, sentinel) &&
                    frame.ned_to_ecef(Vector{}, output) == Status::domain_error && same(output, sentinel),
                profile, "default frame rejects finite positions atomically");
    test.expect(frame.ecef_to_ned(sentinel, output) == Status::non_finite_input && same(output, sentinel) &&
                    frame.ned_to_ecef(sentinel, output) == Status::non_finite_input && same(output, sentinel),
                profile, "nonfinite input precedes initialization");
    test.expect(frame.set_origin({91.0, 0.0, 0.0}) == Status::domain_error && !frame.initialized(), profile,
                "invalid first origin preserves uninitialized state");
    test.expect(frame.set_origin({25.0, 120.0, 100.0}) == Status::success && frame.initialized(), profile,
                "successful initialization");
    Frame copy = frame;
    Vector const input = Vector::from_row_major({6378137.0, 1.0, 2.0});
    Vector before;
    test.expect(frame.ecef_to_ned(input, before) == Status::success &&
                    frame.rotation_from_ecef(basis) == Status::success,
                profile, "save observable cached state");
    for (geo::GeodeticPosition const invalid :
         {geo::GeodeticPosition{nan, 0.0, 0.0}, {91.0, 0.0, 0.0}, {0.0, 0.0, 100001.0}})
    {
        test.expect(!formal_eskf::succeeded(frame.set_origin(invalid)) && frame.initialized(), profile,
                    "rejected replacement retains initialized state");
        Matrix after;
        test.expect(frame.ecef_to_ned(input, output) == Status::success && same(output, before) &&
                        frame.rotation_from_ecef(after) == Status::success && same(after, basis),
                    profile, "rejected replacement retains both origin and basis");
    }
    test.expect(frame.set_origin({0.0, 90.0, 0.0}) == Status::success &&
                    frame.ecef_to_ned(input, output) == Status::success && !same(output, before),
                profile, "successful replacement changes original frame");
    test.expect(copy.ecef_to_ned(input, output) == Status::success && same(output, before), profile,
                "copy retains independent cached values");
}

void failure(Context & test, std::string_view profile)
{
    Frame frame;
    test.expect(frame.set_origin({45.0, 45.0, 0.0}) == Status::success, profile, "overflow fixture origin");
    double const nan = std::numeric_limits<double>::quiet_NaN();
    double const inf = std::numeric_limits<double>::infinity();
    double const max = std::numeric_limits<double>::max();
    Vector const sentinel = Vector::from_row_major({nan, -0.0, inf});
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        for (double const invalid : {nan, inf, -inf})
        {
            Vector input;
            input(axis) = invalid;
            Vector const before = input;
            Vector output = sentinel;
            test.expect(frame.ecef_to_ned(input, output) == Status::non_finite_input && same(output, sentinel) &&
                            frame.ned_to_ecef(input, output) == Status::non_finite_input && same(output, sentinel) &&
                            same(input, before),
                        profile, "each nonfinite position field rejects atomically");
        }
    }
    Vector input = Vector::from_row_major({max, max, max});
    Vector output = sentinel;
    test.expect(frame.ecef_to_ned(input, output) == Status::non_finite_result && same(output, sentinel), profile,
                "finite ECEF input can overflow rotation; preserve output");
    input = Vector::from_row_major({-max, -max, -max});
    test.expect(frame.ned_to_ecef(input, output) == Status::non_finite_result && same(output, sentinel), profile,
                "finite NED input can overflow rotation; preserve output");
    auto const before = input;
    test.expect(frame.ned_to_ecef(input, input) == Status::non_finite_result && same(input, before), profile,
                "overflow preserves aliased input/output");
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Case;
    Case const cases[]{
        {"TC-F-GEO-NED-AXES", "binary64", axes},
        {"TC-F-GEO-NED-GEOMETRY", "partial.binary64", geometry},
        {"TC-F-GEO-NED-TRANSPORT", "binary64", curvature_and_transport},
        {"TC-F-GEO-NED-LIFECYCLE", "binary64", lifecycle},
        {"TC-F-GEO-NED-FAILURE", "binary64", failure},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
