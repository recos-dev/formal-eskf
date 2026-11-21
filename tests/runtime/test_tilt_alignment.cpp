/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <array>
#include <cmath>
#include <limits>
#include <numbers>

#include <formal_eskf/runtime/tilt_alignment.hpp>

#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{
using formal_eskf::Status;
using formal_eskf::test::Case;
using formal_eskf::test::Context;

// R-TILT-ALIGNMENT owns these cases; retain their established test IDs.
// Running these tests does not approve the Draft requirement.
template <typename Scalar> void geometry(Context & test, std::string_view profile)
{
    using Backend = formal_eskf::test::Backend<Scalar>;
    using Alignment = formal_eskf::runtime::TiltAlignment<Backend>;
    using Matrix = formal_eskf::linalg::Matrix<Backend, 3, 3>;
    using Quaternion = formal_eskf::so3::UnitQuaternion<Backend>;
    Alignment alignment;
    test.expect(!alignment.aligned() && alignment.variance() == Scalar{0}, profile,
                "default zero diagnostic does not establish alignment");
    Matrix const P =
        Matrix::from_row_major({Scalar{0.0625}, Scalar{0.015625}, Scalar{0.0078125}, Scalar{0.015625}, Scalar{0.125},
                                Scalar{0.015625}, Scalar{0.0078125}, Scalar{0.015625}, Scalar{0.25}});
    auto const before = P;
    test.expect(alignment.update(Quaternion::identity(), P) == Status::success &&
                    alignment.variance() == Scalar{0.1875} && !alignment.aligned(),
                profile, "identity N/E sum of anisotropic positive covariance");
    Quaternion cyclic;
    test.expect(Quaternion::from_coefficients(Scalar{0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.125},
                                              cyclic) == Status::success,
                profile, "exact unit quaternion for cyclic-axis rotation");
    // This exact rotation sends body (x,y,z) to navigation (z,x,y).
    // It is a tilted attitude; the independent N/E variance is Pzz + Pxx.
    test.expect(alignment.update(cyclic, P) == Status::success && alignment.variance() == Scalar{0.3125}, profile,
                "navigation tilt variance uses rotated covariance, not body x/y sum");
    for (std::size_t row = 0; row < 3; ++row)
    {
        for (std::size_t column = 0; column < 3; ++column)
        {
            test.expect(P(row, column) == before(row, column), profile, "covariance input remains untouched");
        }
    }
    for (int repeat = 0; repeat < 3; ++repeat)
    {
        test.expect(alignment.variance() == Scalar{0.3125} && !alignment.aligned(), profile, "queries are pure");
    }
    test.expect(alignment.update(-cyclic, P) == Status::success && alignment.variance() == Scalar{0.3125}, profile,
                "quaternion sign does not change physical tilt variance");
    Quaternion oblique;
    test.expect(Quaternion::from_coefficients(Scalar{1}, Scalar{0}, Scalar{0.5}, Scalar{0}, Scalar{0.125}, oblique) ==
                    Status::success,
                profile, "oblique pitch makes X/Z cross covariance contribute to N/E variance");
    // A checked coefficient box avoids assuming a libm normalization error budget.
    // Its exact-real rotation has c=a*a-b*b, d=2*a*b and s=a*a+b*b.
    long double const a_low = 0.8944L, a_high = 0.8945L;
    long double const b_low = 0.4472L, b_high = 0.4473L;
    test.expect(oblique.q0() >= a_low && oblique.q0() <= a_high && oblique.q2() >= b_low && oblique.q2() <= b_high &&
                    oblique.q1() == Scalar{0} && oblique.q3() == Scalar{0},
                profile, "public quaternion lies in the independently selected coefficient enclosure");
    long double const epsilon = std::numeric_limits<Scalar>::epsilon();
    auto const c_low = a_low * a_low - b_high * b_high - 3 * epsilon;
    auto const c_high = a_high * a_high - b_low * b_low + 3 * epsilon;
    auto const d_low = 2 * a_low * b_low - 3 * epsilon;
    auto const d_high = 2 * a_high * b_high + 3 * epsilon;
    auto const s_low = a_low * a_low + b_low * b_low - 3 * epsilon;
    auto const s_high = a_high * a_high + b_high * b_high + 3 * epsilon;
    // Both selected diagonals depend on fewer than 64 rounded operations with
    // magnitudes <1; coefficient errors are enclosed above. This coarse bound
    // includes the independent long-double endpoint arithmetic, not observed error.
    long double const rounding = 64 * (epsilon + std::numeric_limits<long double>::epsilon());
    auto const lower = c_low * c_low / 16 + c_low * d_low / 64 + d_low * d_low / 4 + s_low * s_low / 8 - rounding;
    auto const upper =
        c_high * c_high / 16 + c_high * d_high / 64 + d_high * d_high / 4 + s_high * s_high / 8 + rounding;
    test.expect(alignment.update(oblique, P) == Status::success && alignment.variance() >= lower &&
                    alignment.variance() <= upper,
                profile, "independent positive polynomial enclosure detects discarded cross covariance");
    // At the ideal coefficients the answer is 63/200; omitting Pxz gives 123/400.
    test.expect(lower > 123.0L / 400.0L && lower < 63.0L / 200.0L && upper > 63.0L / 200.0L, profile,
                "enclosure separates the known wrong diagonal-only oracle");
}

template <typename Scalar> void threshold(Context & test, std::string_view profile)
{
    using Backend = formal_eskf::test::Backend<Scalar>;
    using Alignment = formal_eskf::runtime::TiltAlignment<Backend>;
    using Matrix = formal_eskf::linalg::Matrix<Backend, 3, 3>;
    auto const q = formal_eskf::so3::UnitQuaternion<Backend>::identity();
    Scalar const limit = std::numbers::pi_v<Scalar> / Scalar{60};
    Scalar const squared_limit = limit * limit;
    test.expect(Alignment::tilt_limit == limit, profile, "public tilt limit is three degrees in radians");
    std::array const values{std::nextafter(squared_limit, Scalar{0}), squared_limit,
                            std::nextafter(squared_limit, std::numeric_limits<Scalar>::infinity())};
    for (std::size_t index = 0; index < values.size(); ++index)
    {
        Alignment alignment;
        Matrix P{};
        P(0, 0) = values[index];
        test.expect(alignment.update(q, P) == Status::success && alignment.variance() == values[index] &&
                        alignment.aligned() == (index == 0),
                    profile, "identity fixture reaches strict represented boundary exactly");
    }
    Alignment alignment;
    for (auto index : {2U, 0U, 2U})
    {
        Matrix P{};
        P(0, 0) = values[index];
        test.expect(alignment.update(q, P) == Status::success && alignment.variance() == values[index], profile,
                    "successful updates always refresh variance diagnostic");
    }
    test.expect(alignment.aligned(), profile, "acquired alignment remains latched after high variance");
    Alignment const new_session;
    test.expect(!new_session.aligned() && new_session.variance() == Scalar{0}, profile,
                "reconstruction starts a new alignment session; there is no reset API");
}

template <typename Scalar> void failures(Context & test, std::string_view profile)
{
    using Backend = formal_eskf::test::Backend<Scalar>;
    using Alignment = formal_eskf::runtime::TiltAlignment<Backend>;
    using Matrix = formal_eskf::linalg::Matrix<Backend, 3, 3>;
    using Quaternion = formal_eskf::so3::UnitQuaternion<Backend>;
    auto const q = Quaternion::identity();
    int const exponent = (std::numeric_limits<Scalar>::min_exponent - std::numeric_limits<Scalar>::digits) / 2;
    Scalar const tiny = std::ldexp(Scalar{1}, exponent);
    Quaternion large;
    test.expect(Quaternion::from_coefficients(tiny, tiny * Scalar{0.5}, Scalar{0}, Scalar{0},
                                              std::numeric_limits<Scalar>::min(), large) == Status::success &&
                    large.q0() == Scalar{1} && large.q1() == Scalar{0.5},
                profile, "gradual underflow produces a public non-unit fault fixture");
    for (unsigned step = 0; step < 32 && std::isfinite(formal_eskf::linalg::squared_norm(large.coefficients())); ++step)
    {
        large = large * large;
    }
    auto const nonfinite = large * large;
    test.expect(formal_eskf::linalg::all_finite(large.coefficients()) &&
                    !formal_eskf::linalg::all_finite(nonfinite.coefficients()),
                profile, "public multiplication reaches finite-overflow and nonfinite quaternion faults");
    for (bool initially_aligned : {false, true})
    {
        Alignment alignment;
        Matrix seed{};
        seed(0, 0) = initially_aligned ? Scalar{0.0009765625} : Scalar{1};
        test.expect(alignment.update(q, seed) == Status::success && alignment.aligned() == initially_aligned, profile,
                    "establish a reachable prior history");
        Scalar const before = alignment.variance();
        test.expect(alignment.update(nonfinite, seed) == Status::non_finite_input && alignment.variance() == before &&
                        alignment.aligned() == initially_aligned,
                    profile, "publicly reachable nonfinite quaternion is rejected without history mutation");
        test.expect(alignment.update(large, seed) == Status::non_finite_result && alignment.variance() == before &&
                        alignment.aligned() == initially_aligned,
                    profile, "finite non-unit fault overflows rotation and preserves history");
        for (std::size_t slot = 0; slot < 9; ++slot)
        {
            for (Scalar invalid : {std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity(),
                                   -std::numeric_limits<Scalar>::infinity()})
            {
                Matrix P = seed;
                P(slot / 3, slot % 3) = invalid;
                test.expect(alignment.update(q, P) == Status::non_finite_input && alignment.variance() == before &&
                                alignment.aligned() == initially_aligned,
                            profile, "all nonfinite covariance positions preserve both owner fields");
                test.expect(std::isnan(invalid) ? std::isnan(P(slot / 3, slot % 3)) : P(slot / 3, slot % 3) == invalid,
                            profile, "failed input remains untouched");
            }
        }
        for (std::size_t axis : {0U, 1U})
        {
            Matrix P = seed;
            P(axis, axis) = Scalar{-1};
            test.expect(alignment.update(q, P) == Status::domain_error && alignment.variance() == before &&
                            alignment.aligned() == initially_aligned,
                        profile, "explicit negative-variance guard; outside PSD success premise");
        }
        Matrix overflow{};
        overflow(0, 0) = std::numeric_limits<Scalar>::max();
        overflow(1, 1) = std::numeric_limits<Scalar>::max();
        test.expect(alignment.update(q, overflow) == Status::non_finite_result && alignment.variance() == before &&
                        alignment.aligned() == initially_aligned,
                    profile, "finite PSD matrix with unrepresentable N/E sum rolls back");
        Quaternion tilted;
        test.expect(Quaternion::from_coefficients(Scalar{1}, Scalar{0}, Scalar{0}, Scalar{0.5}, Scalar{0.125},
                                                  tilted) == Status::success,
                    profile, "construct a valid non-axis rotation for product overflow");
        for (std::size_t row = 0; row < 3; ++row)
        {
            for (std::size_t column = 0; column < 3; ++column)
            {
                overflow(row, column) = std::numeric_limits<Scalar>::max();
            }
        }
        test.expect(alignment.update(tilted, overflow) == Status::non_finite_result && alignment.variance() == before &&
                        alignment.aligned() == initially_aligned,
                    profile, "finite rank-one PSD input with rotation-product overflow rolls back");
        test.expect(alignment.update(q, seed) == Status::success && alignment.variance() == before &&
                        alignment.aligned() == initially_aligned,
                    profile, "valid continuation remains usable after failures");
    }
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    constexpr std::array cases{
        Case{"TC-R-ACCELEROMETER-CONDITIONS-03", "candidate-float", geometry<float>},
        Case{"TC-R-ACCELEROMETER-CONDITIONS-03", "candidate-double", geometry<double>},
        Case{"TC-R-ACCELEROMETER-CONDITIONS-04", "candidate-float", threshold<float>},
        Case{"TC-R-ACCELEROMETER-CONDITIONS-04", "candidate-double", threshold<double>},
        Case{"TC-R-ACCELEROMETER-CONDITIONS-05", "candidate-float", failures<float>},
        Case{"TC-R-ACCELEROMETER-CONDITIONS-05", "candidate-double", failures<double>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
