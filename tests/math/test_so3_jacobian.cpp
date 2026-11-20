/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string_view>
#include <type_traits>

#include <formal_eskf/so3/right_jacobian.hpp>
#include <formal_eskf/linalg/linalg.hpp>

#include "test_backend.hpp"

#include "test_cases.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::Status;
using formal_eskf::test::near;

// Independent oracle: integrate the matrix-exponential power series,
// J_r(phi) = sum_{k=0}^infinity (-hat(phi))^k / (k+1)!.
// The tests using this reference have norm(phi) < 7; 64 terms give an
// adequate truncation margin, and long double reduces reference roundoff.
template <typename Vector> auto jacobian_series_reference(Vector const & phi)
{
    long double const x = static_cast<long double>(phi(0U));
    long double const y = static_cast<long double>(phi(1U));
    long double const z = static_cast<long double>(phi(2U));
    std::array<long double, 9U> const minus_W{0, z, -y, -z, 0, x, y, -x, 0};
    std::array<long double, 9U> term{1, 0, 0, 0, 1, 0, 0, 0, 1};
    auto sum = term;
    for (std::size_t order = 1U; order <= 64U; ++order)
    {
        std::array<long double, 9U> next{};
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            for (std::size_t column = 0U; column < 3U; ++column)
            {
                for (std::size_t inner = 0U; inner < 3U; ++inner)
                {
                    next[row * 3U + column] += term[row * 3U + inner] * minus_W[inner * 3U + column];
                }
                next[row * 3U + column] /= static_cast<long double>(order + 1U);
                sum[row * 3U + column] += next[row * 3U + column];
            }
        }
        term = next;
    }
    return sum;
}

template <typename Linalg>
void test_jacobian_values(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    using vector3_type = formal_eskf::linalg::Vector<Linalg, 3U>;
    using matrix3_type = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    constexpr value_type limit = value_type{0.0625};
    value_type const pi = formal_eskf::scalar::pi<typename Linalg::scalar_math_type>();
    std::array<value_type, 13U> const angles{value_type{0},
                                             std::numeric_limits<value_type>::denorm_min(),
                                             static_cast<value_type>(1.0e-12),
                                             static_cast<value_type>(1.0e-4),
                                             std::nextafter(limit, value_type{0}),
                                             limit,
                                             std::nextafter(limit, value_type{1}),
                                             value_type{0.125},
                                             value_type{0.75},
                                             pi,
                                             value_type{2} * pi - value_type{0.125},
                                             value_type{2} * pi,
                                             value_type{2} * pi + value_type{0.125}};
    for (value_type const angle : angles)
    {
        for (std::size_t direction = 0U; direction < 4U; ++direction)
        {
            vector3_type phi;
            if (direction < 3U)
            {
                phi(direction) = angle;
            }
            else
            {
                // Unit mixed axis [2, -3, 6]/7, with all cross terms nonzero.
                phi(0U) = angle * (value_type{2} / value_type{7});
                phi(1U) = angle * (value_type{-3} / value_type{7});
                phi(2U) = angle * (value_type{6} / value_type{7});
            }
            matrix3_type jacobian;
            Status const status = formal_eskf::so3::right_jacobian(phi, jacobian);
            auto const expected = jacobian_series_reference(phi);
            bool correct = status == Status::success;
            for (std::size_t row = 0U; row < 3U; ++row)
            {
                for (std::size_t column = 0U; column < 3U; ++column)
                {
                    correct = correct && near(jacobian(row, column),
                                              static_cast<value_type>(expected[row * 3U + column]), tolerance);
                }
            }
            test.expect(correct, profile,
                        "closed form agrees with independent series through zero, cutoff, pi and 2*pi");
            matrix3_type negative;
            Status const negative_status = formal_eskf::so3::right_jacobian(-phi, negative);
            test.expect(negative_status == Status::success &&
                            formal_eskf::linalg::max_abs(negative - formal_eskf::linalg::transpose(jacobian)) <=
                                tolerance,
                        profile, "J_r(-phi) equals J_r(phi)^T");
            test.expect(formal_eskf::linalg::max_abs(jacobian * phi - phi) <= value_type{8} * tolerance, profile,
                        "right Jacobian leaves the rotation-axis direction unchanged");
        }
    }

    vector3_type phi;
    matrix3_type jacobian;
    test.expect(formal_eskf::so3::right_jacobian(phi, jacobian) == Status::success &&
                    formal_eskf::linalg::max_abs(jacobian - matrix3_type::identity()) == value_type{0},
                profile, "zero Jacobian is exactly identity without division by zero");
    phi(0U) = value_type{0.75};
    test.expect(formal_eskf::so3::right_jacobian(phi, jacobian) == Status::success &&
                    jacobian(1U, 1U) < value_type{0.9375},
                profile, "closed form is not silently replaced by I-hat(phi)/2");

    // Very large but representable squared norms must not form theta^3 or
    // multiply an unscaled hat(phi)^2 by a tiny coefficient.
    phi(0U) = static_cast<value_type>(1.0e10);
    test.expect(formal_eskf::so3::right_jacobian(phi, jacobian) == Status::success &&
                    formal_eskf::linalg::all_finite(jacobian) && near(jacobian(0U, 0U), value_type{1}, tolerance),
                profile, "large finite angle uses a bounded unit-axis expression");
}

template <typename Linalg> void test_jacobian_failures(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using vector3_type = formal_eskf::linalg::Vector<Linalg, 3U>;
    using matrix3_type = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    auto const sentinel = matrix3_type::identity() * value_type{2};
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        for (value_type const invalid :
             {std::numeric_limits<value_type>::quiet_NaN(), std::numeric_limits<value_type>::infinity(),
              -std::numeric_limits<value_type>::infinity()})
        {
            vector3_type phi;
            phi(axis) = invalid;
            auto output = sentinel;
            Status const status = formal_eskf::so3::right_jacobian(phi, output);
            test.expect(status == Status::non_finite_input &&
                            formal_eskf::linalg::max_abs(output - sentinel) == value_type{0},
                        profile, "non-finite input preserves output");
        }
        vector3_type phi;
        phi(axis) = std::numeric_limits<value_type>::max();
        auto output = sentinel;
        Status const status = formal_eskf::so3::right_jacobian(phi, output);
        test.expect(status == Status::non_finite_result &&
                        formal_eskf::linalg::max_abs(output - sentinel) == value_type{0},
                    profile, "squared-norm overflow preserves output");
    }
}

// Independent quaternion Exp in wider arithmetic for a right-perturbation
// differential. No production Exp/Log/R/hat participates in this oracle.
std::array<long double, 4> reference_exp(std::array<long double, 3> const & phi)
{
    long double const theta = std::sqrt(phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2]);
    long double const scale = theta == 0 ? 0.5L : std::sin(theta / 2) / theta;
    return {std::cos(theta / 2), scale * phi[0], scale * phi[1], scale * phi[2]};
}

template <typename B> void derivative_sweep(TestContext & test, std::string_view profile)
{
    auto const phi = formal_eskf::linalg::Vector<B, 3>::from_row_major({0.25, -0.375, 0.5});
    formal_eskf::linalg::Matrix<B, 3, 3> J;
    test.expect(formal_eskf::so3::right_jacobian(phi, J) == Status::success, profile,
                "right-Jacobian derivative fixture");
    std::array<long double, 3> const base{0.25L, -0.375L, 0.5L};
    auto const q = reference_exp(base);
    long double first_error = 0, previous = std::numeric_limits<long double>::infinity();
    for (long double step : {0.25L, 0.125L, 0.0625L, 0.03125L, 0.015625L})
    {
        long double error = 0;
        for (std::size_t column = 0; column < 3; ++column)
        {
            auto plus = base, minus = base;
            plus[column] += step;
            minus[column] -= step;
            auto const qp = reference_exp(plus), qm = reference_exp(minus);
            std::array<long double, 4> dq{};
            for (std::size_t index = 0; index < 4; ++index)
            {
                dq[index] = (qp[index] - qm[index]) / (2 * step);
            }
            for (std::size_t row = 0; row < 3; ++row)
            {
                auto const next = (row + 1) % 3, last = (row + 2) % 3;
                // 2*vec(conj(q)*dq) is the local/right infinitesimal action.
                long double const derivative = 2 * (q[0] * dq[row + 1] - q[row + 1] * dq[0] -
                                                    q[next + 1] * dq[last + 1] + q[last + 1] * dq[next + 1]);
                error = std::max(error, std::abs(derivative - static_cast<long double>(J(row, column))));
            }
        }
        test.expect(std::isfinite(error) && error < previous, profile,
                    "halving step reduces independent right-perturbation truncation residual");
        if (step == 0.25L)
        {
            first_error = error;
        }
        previous = error;
        std::cout << "RIGHT-DERIVATIVE step=" << step << " max_error=" << error << '\n';
    }
    test.expect(previous < first_error / 32, profile,
                "step sweep approaches J_r rather than the inverse or left Jacobian");
}

template <typename Scalar> struct BranchMath : formal_eskf::scalar::StandardMath<Scalar>
{
    using Base = formal_eskf::scalar::StandardMath<Scalar>;
    enum class Fault
    {
        none,
        root,
        sine,
        cosine,
        zero_root
    };
    static inline Fault fault = Fault::none;
    static inline unsigned trig_calls{};
    static Scalar sqrt(Scalar value)
    {
        return fault == Fault::root ? std::numeric_limits<Scalar>::quiet_NaN()
                                    : (fault == Fault::zero_root ? Scalar{0} : Base::sqrt(value));
    }
    static Scalar sin(Scalar value)
    {
        ++trig_calls;
        return fault == Fault::sine ? std::numeric_limits<Scalar>::infinity() : Base::sin(value);
    }
    static Scalar cos(Scalar value)
    {
        ++trig_calls;
        return fault == Fault::cosine ? std::numeric_limits<Scalar>::quiet_NaN() : Base::cos(value);
    }
};

template <typename Base> struct BranchBackend : Base
{
    using Scalar = typename Base::value_type;
    using scalar_math_type = BranchMath<Scalar>;
    template <std::size_t R, std::size_t C> using Storage = typename Base::template storage_type<R, C>;
    static inline bool override_squared{}, bad_candidate{};
    static inline Scalar squared_value{};
    template <std::size_t N> static Scalar squared_norm(Storage<N, 1> const & input)
    {
        return override_squared ? squared_value : Base::template squared_norm<N>(input);
    }
    template <std::size_t R, std::size_t C>
    static void scale(Storage<R, C> const & input, Scalar scale, Storage<R, C> & output)
    {
        Base::template scale<R, C>(input, scale, output);
        if (bad_candidate)
        {
            Base::template coefficient<R, C>(output, R - 1, C - 1) = std::numeric_limits<Scalar>::infinity();
        }
    }
    static void reset()
    {
        override_squared = bad_candidate = false;
        scalar_math_type::fault = scalar_math_type::Fault::none;
        scalar_math_type::trig_calls = 0;
    }
};

template <typename B> void map(TestContext & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    // Historical sampled series envelope; not a verified uniform truncation,
    // reference-roundoff or deployed-libm error bound. Keep MAP numerical scope Partial.
    Scalar const tolerance = std::is_same_v<Scalar, float> ? static_cast<Scalar>(5e-7) : static_cast<Scalar>(1e-12);
    test_jacobian_values<B>(test, profile, tolerance);
    derivative_sweep<B>(test, profile);
}

template <typename Base> void branch(TestContext & test, std::string_view profile)
{
    using B = BranchBackend<Base>;
    using Scalar = typename B::value_type;
    using Vector = formal_eskf::linalg::Vector<B, 3>;
    for (int side : {-1, 0, 1})
    {
        B::reset();
        Vector phi;
        phi(0) = Scalar{0.0625};
        if (side != 0)
        {
            phi(0) = std::nextafter(phi(0), side < 0 ? Scalar{0} : Scalar{1});
        }
        Scalar const square = phi(0) * phi(0);
        test.expect(side == 0 ? square == Scalar{0.00390625}
                              : (side < 0 ? square < Scalar{0.00390625} : square > Scalar{0.00390625}),
                    profile, "represented vector reaches exact cutoff or its intended adjacent side");
        formal_eskf::linalg::Matrix<B, 3, 3> result;
        test.expect(formal_eskf::so3::right_jacobian(phi, result) == Status::success &&
                        B::scalar_math_type::trig_calls == (side > 0 ? 2U : 0U),
                    profile, "passive scalar observation distinguishes inclusive Taylor versus half-angle branch");
        if (side <= 0)
        {
            Scalar const A = Scalar{0.5} - square * (Scalar{1} / Scalar{24} -
                                                     square * (Scalar{1} / Scalar{720} - square / Scalar{40320}));
            Scalar const C =
                Scalar{1} / Scalar{6} -
                square * (Scalar{1} / Scalar{120} - square * (Scalar{1} / Scalar{5040} - square / Scalar{362880}));
            test.expect(result(0, 0) == Scalar{1} && result(1, 1) == Scalar{1} - square * C &&
                            result(2, 2) == result(1, 1) && result(1, 2) == phi(0) * A && result(2, 1) == -phi(0) * A,
                        profile, "selected polynomial coefficients agree with independent axis reduction");
        }
    }
    B::reset();
    Vector mixed = Vector::from_row_major({Scalar{0.015625}, Scalar{-0.03125}, Scalar{0.015625}});
    formal_eskf::linalg::Matrix<B, 3, 3> output;
    test.expect(formal_eskf::so3::right_jacobian(mixed, output) == Status::success &&
                    B::scalar_math_type::trig_calls == 0,
                profile, "nonaxis vector below cutoff also selects the polynomial");
    B::reset();
    // Native dispatch/polynomial checks do not close the numerical continuity budget.
}

template <typename Base> void failures(TestContext & test, std::string_view profile)
{
    test_jacobian_failures<Base>(test, profile);
    using B = BranchBackend<Base>;
    using Scalar = typename B::value_type;
    using Matrix = formal_eskf::linalg::Matrix<B, 3, 3>;
    using Vector = formal_eskf::linalg::Vector<B, 3>;
    auto const phi = Vector::from_row_major({1, -2, 3});
    auto check = [&](Status expected)
    {
        Matrix output =
            Matrix::from_row_major({-Scalar{0}, 2, 3, 4, std::numeric_limits<Scalar>::quiet_NaN(), 6, 7, 8, 9});
        auto const before = output;
        test.expect(formal_eskf::so3::right_jacobian(phi, output) == expected, profile,
                    "controlled Jacobian stage status");
        for (std::size_t row = 0; row < 3; ++row)
        {
            for (std::size_t column = 0; column < 3; ++column)
            {
                using Bytes = std::array<std::byte, sizeof(Scalar)>;
                test.expect((std::isnan(output(row, column)) && std::isnan(before(row, column))) ||
                                std::bit_cast<Bytes>(output(row, column)) == std::bit_cast<Bytes>(before(row, column)),
                            profile,
                            "all nine output fields remain unchanged, including signed zero and invalid sentinels");
            }
        }
    };
    B::reset();
    B::override_squared = true;
    B::squared_value = Scalar{-1};
    check(Status::domain_error);
    B::squared_value = std::numeric_limits<Scalar>::infinity();
    check(Status::non_finite_result);
    B::reset();
    using Fault = typename B::scalar_math_type::Fault;
    for (auto fault : {Fault::root, Fault::sine, Fault::cosine, Fault::zero_root})
    {
        B::scalar_math_type::fault = fault;
        check(Status::non_finite_result);
        B::reset();
    }
    B::bad_candidate = true;
    check(Status::non_finite_result);
    B::reset();
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using F = formal_eskf::test::Backend<float>;
    using D = formal_eskf::test::Backend<double>;
    using formal_eskf::test::Case;
    Case const cases[]{
        {"TC-F-JR-MAP", "partial.binary32", map<F>},
        {"TC-F-JR-MAP", "partial.binary64", map<D>},
        {"TC-F-JR-BRANCH", "partial.binary32", branch<F>},
        {"TC-F-JR-BRANCH", "partial.binary64", branch<D>},
        {"TC-F-JR-FAILURE", "controlled+native.binary32", failures<F>},
        {"TC-F-JR-FAILURE", "controlled+native.binary64", failures<D>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
