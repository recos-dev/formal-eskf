/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string_view>
#include <type_traits>

#include <formal_eskf/linalg/linalg.hpp>

#include "test_backend.hpp"
#include <formal_eskf/so3/so3.hpp>

#include "test_cases.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::test::near;

template <typename Matrix>
bool matrix_near(Matrix const & actual, Matrix const & expected, typename Matrix::value_type tolerance)
{
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            if (!near(actual(row, column), expected(row, column), tolerance))
            {
                return false;
            }
        }
    }
    return true;
}

namespace so = formal_eskf::so3;
namespace la = formal_eskf::linalg;
using formal_eskf::Status;
using Context = TestContext;

template <typename Scalar> bool same_scalar(Scalar a, Scalar b)
{
    using bytes = std::array<std::byte, sizeof(Scalar)>;
    return (std::isnan(a) && std::isnan(b)) || std::bit_cast<bytes>(a) == std::bit_cast<bytes>(b);
}
template <typename Matrix> bool same_fields(Matrix const & a, Matrix const & b)
{
    for (std::size_t row = 0; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0; column < Matrix::column_count; ++column)
        {
            if (!same_scalar(a(row, column), b(row, column)))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename B> auto quaternion(Context & test, std::string_view profile, std::array<typename B::value_type, 4> c)
{
    using Scalar = typename B::value_type;
    so::UnitQuaternion<B> q;
    test.expect(so::UnitQuaternion<B>::from_coefficients(c[0], c[1], c[2], c[3], Scalar{0.125}, q) == Status::success,
                profile, "public fixture construction");
    return q;
}

template <typename B> auto fixtures(Context & test, std::string_view profile)
{
    return std::array{so::UnitQuaternion<B>::identity(), quaternion<B>(test, profile, {0, 1, 0, 0}),
                      quaternion<B>(test, profile, {0, 0, 1, 0}), quaternion<B>(test, profile, {0, 0, 0, 1}),
                      quaternion<B>(test, profile, {0.5, 0.5, 0.5, 0.5})};
}

template <typename Quaternion> auto hamilton(Quaternion const & a, Quaternion const & b)
{
    using Scalar = typename Quaternion::value_type;
    auto const x = a.coefficients(), y = b.coefficients();
    std::array<Scalar, 4> result{};
    result[0] = x(0) * y(0);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        result[0] -= x(axis + 1) * y(axis + 1);
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        auto const next = (axis + 1) % 3, last = (axis + 2) % 3;
        result[axis + 1] =
            x(0) * y(axis + 1) + y(0) * x(axis + 1) + x(next + 1) * y(last + 1) - x(last + 1) * y(next + 1);
    }
    return result;
}

template <typename Q> bool coefficients_equal(Q const & q, std::array<typename Q::value_type, 4> const & expected)
{
    return q.q0() == expected[0] && q.q1() == expected[1] && q.q2() == expected[2] && q.q3() == expected[3];
}

template <typename B> void identity(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    so::UnitQuaternion<B> q;
    test.expect(coefficients_equal(q, {1, 0, 0, 0}) &&
                    coefficients_equal(so::UnitQuaternion<B>::identity(), {1, 0, 0, 0}),
                profile, "default/static identity scalar-first coefficients");
    auto values = q.coefficients();
    values(0) = Scalar{-3};
    test.expect(q.q0() == Scalar{1}, profile, "coefficient copy is owning");
    auto const nonidentity = quaternion<B>(test, profile, {0.5, 0.5, 0.5, 0.5});
    test.expect(so::same_coefficients(q * nonidentity, nonidentity) &&
                    so::same_coefficients(nonidentity * q, nonidentity),
                profile, "exact left/right multiplicative identity");
}

template <typename B> void multiplication(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    auto const samples = fixtures<B>(test, profile);
    for (auto const & a : samples)
    {
        for (auto const & b : samples)
        {
            test.expect(coefficients_equal(a * b, hamilton(a, b)) && coefficients_equal((-a) * b, hamilton(-a, b)),
                        profile, "independent scalar/vector Hamilton determinant formulation");
        }
    }
    auto q = samples[4];
    for (unsigned repeat = 0; repeat < 12; ++repeat)
    {
        auto const expected = hamilton(q, samples[4]);
        q = q * samples[4];
        test.expect(coefficients_equal(q, expected), profile, "repeated exact products preserve coefficient signs");
    }
    // Public normalization can produce this out-of-unit value through squared
    // norm underflow. It is used only to distinguish raw multiplication from repair.
    Scalar const tiny = std::ldexp(Scalar{1}, std::is_same_v<Scalar, float> ? -74 : -537);
    test.expect(so::UnitQuaternion<B>::from_coefficients(tiny, tiny / Scalar{2}, Scalar{0}, Scalar{0},
                                                         std::numeric_limits<Scalar>::min(), q) == Status::success &&
                    coefficients_equal(q, {1, 0.5, 0, 0}),
                profile, "public nonunit arithmetic guard fixture");
    test.expect(coefficients_equal(q * q, {0.75, 1, 0, 0}), profile,
                "raw multiplication does not normalize or canonicalize");
}

template <typename B> void inverse(Context & test, std::string_view profile)
{
    for (auto const & value : fixtures<B>(test, profile))
    {
        for (auto const q : {value, -value})
        {
            auto const result = q.inverse();
            test.expect(coefficients_equal(result, {q.q0(), -q.q1(), -q.q2(), -q.q3()}) &&
                            so::same_coefficients(result.inverse(), q) &&
                            coefficients_equal(q * result, {1, 0, 0, 0}) &&
                            coefficients_equal(result * q, {1, 0, 0, 0}),
                        profile, "conjugate and both exact inverse identities");
        }
    }
}

template <typename B> void normalization(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    using Q = so::UnitQuaternion<B>;
    auto const basics = fixtures<B>(test, profile);
    std::array const units{basics[0], basics[1], basics[2],
                           basics[3], basics[4], quaternion<B>(test, profile, {0.5, -0.5, 0.5, -0.5})};
    for (auto const & unit : units)
    {
        for (Scalar scale : {Scalar{-4}, Scalar{0.5}, Scalar{8}})
        {
            auto const c = unit.coefficients() * scale;
            Q scalar, vector;
            test.expect(Q::from_coefficients(c(0), c(1), c(2), c(3), Scalar{0.125}, scalar) == Status::success &&
                            Q::from_coefficients(c, Scalar{0.125}, vector) == Status::success &&
                            so::same_coefficients(scalar, vector) &&
                            so::same_coefficients(vector, scale < Scalar{0} ? -unit : unit),
                        profile, "both overloads normalize exactly scaled dyadic coefficients without preferred sign");
            Scalar norm{};
            for (std::size_t axis = 0; axis < 4; ++axis)
            {
                norm += vector.coefficients()(axis) * vector.coefficients()(axis);
            }
            test.expect(norm == Scalar{1}, profile, "independently summed exact fixture unit residual");
            for (bool alias : {false, true})
            {
                auto input = vector;
                Q separate = -unit;
                auto & output = alias ? input : separate;
                test.expect(so::normalize(input, Scalar{0.125}, output) == Status::success &&
                                so::same_coefficients(output, vector),
                            profile, "public quaternion normalize disjoint/in-place");
            }
        }
    }
}

template <typename B> void normalization_domain(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    using Q = so::UnitQuaternion<B>;
    using Vector = la::Vector<B, 4>;
    auto check = [&](Vector const & input, Scalar threshold, Status expected)
    {
        for (bool scalar_overload : {false, true})
        {
            Q output = -Q::identity();
            auto const before = output;
            auto const status = scalar_overload
                                    ? Q::from_coefficients(input(0), input(1), input(2), input(3), threshold, output)
                                    : Q::from_coefficients(input, threshold, output);
            test.expect(status == expected && same_fields(output.coefficients(), before.coefficients()), profile,
                        "factory failure precedence and complete quaternion rollback");
        }
    };
    auto const good = Vector::from_row_major({4, 0, 0, 0});
    for (Scalar threshold : {std::nextafter(Scalar{4}, Scalar{0}), Scalar{4}})
    {
        Q result;
        test.expect(Q::from_coefficients(good, threshold, result) == Status::success &&
                        coefficients_equal(result, {1, 0, 0, 0}),
                    profile, "threshold above one is allowed and equality is inclusive");
    }
    check(good, std::nextafter(Scalar{4}, Scalar{5}), Status::invalid_quaternion_norm);
    check(Vector{}, Scalar{1}, Status::invalid_quaternion_norm);
    for (Scalar invalid : {std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity(),
                           -std::numeric_limits<Scalar>::infinity()})
    {
        check(good, invalid, Status::non_finite_input);
        for (std::size_t axis = 0; axis < 4; ++axis)
        {
            auto bad = good;
            bad(axis) = invalid;
            check(bad, Scalar{1}, Status::non_finite_input);
            check(bad, Scalar{0}, Status::domain_error);
        }
    }
    check(good, Scalar{-1}, Status::domain_error);
    auto large = good;
    large(1) = std::numeric_limits<Scalar>::max();
    large(2) = std::numeric_limits<Scalar>::max();
    check(large, Scalar{1}, Status::non_finite_result);
    check(large, Scalar{0}, Status::domain_error);
    for (Scalar invalid : {Scalar{0}, Scalar{-1}, std::numeric_limits<Scalar>::quiet_NaN()})
    {
        Q q = -Q::identity();
        auto const before = q;
        test.expect(so::normalize(q, invalid, q) ==
                            (std::isfinite(invalid) ? Status::domain_error : Status::non_finite_input) &&
                        same_fields(q.coefficients(), before.coefficients()),
                    profile, "in-place normalization failure preserves sign/fields");
    }
}

enum class MathFault
{
    none,
    root,
    sine,
    cosine,
    atan,
    zero_root
};
template <typename Scalar> struct ProbeMath : formal_eskf::scalar::StandardMath<Scalar>
{
    using Base = formal_eskf::scalar::StandardMath<Scalar>;
    static inline MathFault fault = MathFault::none;
    static inline unsigned roots{}, sines{}, cosines{}, atans{};
    static Scalar sqrt(Scalar x)
    {
        ++roots;
        return fault == MathFault::root ? std::numeric_limits<Scalar>::quiet_NaN()
                                        : (fault == MathFault::zero_root ? Scalar{0} : Base::sqrt(x));
    }
    static Scalar sin(Scalar x)
    {
        ++sines;
        return fault == MathFault::sine ? std::numeric_limits<Scalar>::infinity() : Base::sin(x);
    }
    static Scalar cos(Scalar x)
    {
        ++cosines;
        return fault == MathFault::cosine ? std::numeric_limits<Scalar>::quiet_NaN() : Base::cos(x);
    }
    static Scalar atan2(Scalar y, Scalar x)
    {
        ++atans;
        return fault == MathFault::atan ? std::numeric_limits<Scalar>::infinity() : Base::atan2(y, x);
    }
    static void reset()
    {
        fault = MathFault::none;
        roots = sines = cosines = atans = 0;
    }
};

template <typename Base> struct ProbeBackend : Base
{
    using Scalar = typename Base::value_type;
    using scalar_math_type = ProbeMath<Scalar>;
    template <std::size_t R, std::size_t C> using Storage = typename Base::template storage_type<R, C>;
    static inline bool override_squared{}, override_norm{}, bad_division{}, bad_scale{};
    static inline Scalar squared_value{}, norm_value{};
    template <std::size_t N> static Scalar squared_norm(Storage<N, 1> const & input)
    {
        return override_squared ? squared_value : Base::template squared_norm<N>(input);
    }
    template <std::size_t N> static Scalar norm(Storage<N, 1> const & input)
    {
        return override_norm ? norm_value : Base::template norm<N>(input);
    }
    template <std::size_t R, std::size_t C>
    static void divide(Storage<R, C> const & input, Scalar divisor, Storage<R, C> & output)
    {
        Base::template divide<R, C>(input, divisor, output);
        if (bad_division)
        {
            Base::template coefficient<R, C>(output, R - 1, C - 1) = std::numeric_limits<Scalar>::quiet_NaN();
        }
    }
    template <std::size_t R, std::size_t C>
    static void scale(Storage<R, C> const & input, Scalar factor, Storage<R, C> & output)
    {
        Base::template scale<R, C>(input, factor, output);
        if (bad_scale)
        {
            Base::template coefficient<R, C>(output, R - 1, C - 1) = std::numeric_limits<Scalar>::infinity();
        }
    }
    static void reset()
    {
        override_squared = override_norm = bad_division = bad_scale = false;
        scalar_math_type::reset();
    }
};

template <typename Base> void normalization_result(Context & test, std::string_view profile)
{
    using B = ProbeBackend<Base>;
    using Scalar = typename B::value_type;
    using Q = so::UnitQuaternion<B>;
    B::reset();
    for (Scalar norm : {Scalar{0}, std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity()})
    {
        B::override_norm = true;
        B::norm_value = norm;
        for (bool vector_overload : {false, true})
        {
            Q output = -Q::identity();
            auto const before = output;
            auto const status =
                vector_overload
                    ? Q::from_coefficients(la::Vector<B, 4>::from_row_major({1, 0, 0, 0}), Scalar{0.125}, output)
                    : Q::from_coefficients(Scalar{1}, Scalar{0}, Scalar{0}, Scalar{0}, Scalar{0.125}, output);
            test.expect(status == (std::isfinite(norm) ? Status::invalid_quaternion_norm : Status::non_finite_result) &&
                            same_fields(output.coefficients(), before.coefficients()),
                        profile, "controlled norm failure mapping and publication");
        }
    }
    B::reset();
    B::bad_division = true;
    Q output = -Q::identity();
    auto const before = output;
    test.expect(Q::from_coefficients(Scalar{1}, Scalar{0}, Scalar{0}, Scalar{0}, Scalar{0.125}, output) ==
                        Status::non_finite_result &&
                    same_fields(output.coefficients(), before.coefficients()),
                profile, "nonfinite final coefficient prevents all quaternion writes");
    B::reset();
}

template <typename B> void composition(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    using Q = so::UnitQuaternion<B>;
    auto const a = quaternion<B>(test, profile, {0, 1, 0, 0});
    auto const b = quaternion<B>(test, profile, {0.5, 0.5, 0.5, 0.5});
    auto const expected = hamilton(a, b);
    for (unsigned alias = 0; alias < 3; ++alias)
    {
        auto left = a, right = b;
        Q separate;
        auto & output = alias == 1 ? left : (alias == 2 ? right : separate);
        test.expect(so::compose_normalized(left, right, Scalar{0.125}, output) == Status::success &&
                        coefficients_equal(output, expected),
                    profile, "noncommuting checked composition applies right operand first with every legal alias");
        auto const v = la::Vector<B, 3>::from_row_major({1, 2, 4});
        auto const rotated = so::rotate(output, v);
        test.expect(rotated(0) == Scalar{4} && rotated(1) == Scalar{-1} && rotated(2) == Scalar{-2}, profile,
                    "independent cyclic permutation followed by x-half-turn action");
        left = a;
        right = b;
        separate = -Q::identity();
        auto const before = output;
        test.expect(so::compose_normalized(left, right, Scalar{0}, output) == Status::domain_error &&
                        same_fields(output.coefficients(), before.coefficients()),
                    profile, "composition threshold failure is atomic");
    }
    Scalar const tiny = std::ldexp(Scalar{1}, std::is_same_v<Scalar, float> ? -74 : -537);
    Q raw, normalized;
    test.expect(Q::from_coefficients(tiny, tiny / Scalar{2}, Scalar{0}, Scalar{0}, std::numeric_limits<Scalar>::min(),
                                     raw) == Status::success,
                profile, "construct public nonunit value to distinguish normalized publication");
    test.expect(so::compose_normalized(raw, raw, Scalar{0.125}, normalized) == Status::success &&
                    normalized.q0() == Scalar{3} / Scalar{5} && normalized.q1() == Scalar{4} / Scalar{5},
                profile, "checked composition publishes normalized product rather than raw product");
    using Probe = ProbeBackend<B>;
    using PQ = so::UnitQuaternion<Probe>;
    Probe::reset();
    Probe::override_norm = true;
    Probe::norm_value = Scalar{0};
    auto p = -PQ::identity();
    test.expect(so::compose_normalized(PQ::identity(), PQ::identity(), Scalar{0.125}, p) ==
                        Status::invalid_quaternion_norm &&
                    coefficients_equal(p, {-1, 0, 0, 0}),
                profile, "checked composition consumes normalization and propagates its failure");
    Probe::reset();
}

template <typename B> void sign_compare(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    auto const q = quaternion<B>(test, profile, {0.5, 0.5, 0.5, 0.5});
    auto const other = so::UnitQuaternion<B>::identity();
    test.expect(!so::same_coefficients(q, -q) && so::same_rotation(q, -q, Scalar{0}) && so::same_coefficients(q, q),
                profile, "coefficient identity differs from physical sign equivalence");
    for (Scalar tolerance :
         {std::nextafter(Scalar{0.5}, Scalar{0}), Scalar{0.5}, std::nextafter(Scalar{0.5}, Scalar{1})})
    {
        test.expect(so::same_rotation(q, other, tolerance) == (tolerance >= Scalar{0.5}) &&
                        so::same_rotation(-q, other, tolerance) == (tolerance >= Scalar{0.5}),
                    profile, "inclusive minimum of plus/minus coefficient residual");
    }
    for (Scalar invalid :
         {Scalar{-1}, std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity()})
    {
        test.expect(!so::same_rotation(q, q, invalid), profile, "invalid tolerance is never equivalence evidence");
    }
}

template <typename B> void matrix_coefficients(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    Scalar const expected[5][9]{{1, 0, 0, 0, 1, 0, 0, 0, 1},
                                {1, 0, 0, 0, -1, 0, 0, 0, -1},
                                {-1, 0, 0, 0, 1, 0, 0, 0, -1},
                                {-1, 0, 0, 0, -1, 0, 0, 0, 1},
                                {0, 0, 1, 1, 0, 0, 0, 1, 0}};
    auto const samples = fixtures<B>(test, profile);
    for (std::size_t index = 0; index < samples.size(); ++index)
    {
        for (auto const q : {samples[index], -samples[index]})
        {
            auto const R = so::to_rotation_matrix(q);
            for (std::size_t row = 0; row < 3; ++row)
            {
                for (std::size_t column = 0; column < 3; ++column)
                {
                    test.expect(R(row, column) == expected[index][row * 3 + column], profile,
                                "all nine independent exact rotation coefficients");
                }
            }
        }
    }
    auto const general = quaternion<B>(test, profile, {2, 1, -3, 4});
    int const numerators[9]{-20, -22, -4, 10, -4, -28, 20, -20, 10};
    // Input squared norm is exactly 30. Rounded sqrt/division perturb each
    // normalized coefficient by <= about 2u; the quadratic coefficient has
    // at most seven further rounded operations. 16*epsilon safely encloses
    // this fixed, unit-scale calculation plus the wider rational reference.
    auto const R = so::to_rotation_matrix(general);
    for (std::size_t index = 0; index < 9; ++index)
    {
        long double const expected = static_cast<long double>(numerators[index]) / 30;
        test.expect(std::abs(static_cast<long double>(R(index / 3, index % 3)) - expected) <=
                        Scalar{16} * std::numeric_limits<Scalar>::epsilon(),
                    profile, "independent rational oblique rotation checks every coefficient and cross term");
    }
}

template <typename B> void action(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    using Vector = la::Vector<B, 3>;
    auto const q = quaternion<B>(test, profile, {0.5, 0.5, 0.5, 0.5});
    for (auto const & v : {Vector::from_row_major({1, 0, 0}), Vector::from_row_major({0, 1, 0}),
                           Vector::from_row_major({0, 0, 1}), Vector::from_row_major({1, -2, 4})})
    {
        for (auto const sign : {q, -q})
        {
            auto const forward = so::rotate(sign, v), backward = so::inverse_rotate(sign, v);
            test.expect(forward(0) == v(2) && forward(1) == v(0) && forward(2) == v(1) && backward(0) == v(1) &&
                            backward(1) == v(2) && backward(2) == v(0),
                        profile, "independent cyclic coordinate action distinguishes R from transpose");
            test.expect(matrix_near(so::inverse_rotate(sign, forward), v, Scalar{0}), profile, "exact action recovery");
            Scalar input_norm{}, result_norm{};
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                input_norm += v(axis) * v(axis);
                result_norm += forward(axis) * forward(axis);
            }
            test.expect(input_norm == result_norm, profile, "independent exact length preservation");
        }
    }
    auto const general = quaternion<B>(test, profile, {2, 1, -3, 4});
    auto const v = Vector::from_row_major({1, -2, 4});
    auto const forward = so::rotate(general, v), backward = so::inverse_rotate(general, v);
    int const forward_numerators[3]{8, -94, 100}, backward_numerators[3]{40, -94, 92};
    // Propagate the fixed rational-matrix enclosure through three products
    // and two sums, with ||v||_1=7. This is a fixture-specific error budget.
    long double const bound = 32.0L * 7 * std::numeric_limits<Scalar>::epsilon();
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        test.expect(std::abs(static_cast<long double>(forward(axis)) -
                             static_cast<long double>(forward_numerators[axis]) / 30) <= bound &&
                        std::abs(static_cast<long double>(backward(axis)) -
                                 static_cast<long double>(backward_numerators[axis]) / 30) <= bound,
                    profile, "oblique action agrees with independent rational coordinate oracle");
    }
}

template <typename B> void hat(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    using Vector = la::Vector<B, 3>;
    auto const b = Vector::from_row_major({2, -3, 5});
    for (auto const & a : {Vector{}, Vector::from_row_major({1, 0, 0}), Vector::from_row_major({0, 1, 0}),
                           Vector::from_row_major({0, 0, 1}), Vector::from_row_major({2, -1, 3})})
    {
        Scalar const expected[9]{0, -a(2), a(1), a(2), 0, -a(0), -a(1), a(0), 0};
        auto const W = so::hat(a);
        for (std::size_t row = 0; row < 3; ++row)
        {
            for (std::size_t column = 0; column < 3; ++column)
            {
                test.expect(W(row, column) == expected[row * 3 + column] && W(row, column) == -W(column, row), profile,
                            "independent skew entries and transpose sign");
            }
        }
        auto const product = W * b;
        test.expect(product(0) == a(1) * b(2) - a(2) * b(1) && product(1) == a(2) * b(0) - a(0) * b(2) &&
                        product(2) == a(0) * b(1) - a(1) * b(0),
                    profile, "hat action equals independent determinant, not production cross");
    }
}

// General-angle checks are supplemental regression samples with a declared
// 64-epsilon coefficient envelope. They do not close the outstanding uniform
// libm, normalization, inverse or branch-transition error-budget requirements.
template <typename Scalar> constexpr Scalar sample_envelope()
{
    return Scalar{64} * std::numeric_limits<Scalar>::epsilon();
}

template <typename B> void exp_map(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    using Vector = la::Vector<B, 3>;
    using Q = so::UnitQuaternion<B>;
    test.expect(std::numeric_limits<long double>::digits > std::numeric_limits<Scalar>::digits, profile,
                "general-angle sampled reference uses wider arithmetic");
    Scalar const pi = static_cast<Scalar>(3.141592653589793238462643383279502884L);
    for (auto const & phi :
         {Vector{}, Vector::from_row_major({1, 0, 0}), Vector::from_row_major({0, -2, 0}),
          Vector::from_row_major({0, 0, 4}), Vector::from_row_major({8, 0, 0}), Vector::from_row_major({pi, 0, 0}),
          Vector::from_row_major({std::nextafter(pi, Scalar{4}), 0, 0}), Vector::from_row_major({0.25, -0.5, 1})})
    {
        long double square{};
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            auto const x = static_cast<long double>(phi(axis));
            square += x * x;
        }
        long double const theta = std::sqrt(square), scalar = std::cos(theta / 2),
                          scale = theta == 0 ? 0.5L : std::sin(theta / 2) / theta;
        Q output = -Q::identity();
        auto const status = so::exp(phi, Scalar{0.125}, output);
        test.expect(status == Status::success &&
                        std::abs(static_cast<long double>(output.q0()) - scalar) <= sample_envelope<Scalar>(),
                    profile, "independent long-double half-angle scalar reference (sample envelope only)");
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            test.expect(std::abs(static_cast<long double>(output.coefficients()(axis + 1)) - scale * phi(axis)) <=
                            sample_envelope<Scalar>(),
                        profile, "independent half-angle vector reference including nonprincipal input angles");
        }
        auto const R = so::to_rotation_matrix(output);
        std::array<long double, 3> axis{};
        if (theta != 0)
        {
            for (std::size_t index = 0; index < 3; ++index)
            {
                axis[index] = static_cast<long double>(phi(index)) / theta;
            }
        }
        long double const sine = std::sin(theta), cosine = std::cos(theta);
        long double const skew[3][3]{{0, -axis[2], axis[1]}, {axis[2], 0, -axis[0]}, {-axis[1], axis[0], 0}};
        for (std::size_t row = 0; row < 3; ++row)
        {
            for (std::size_t column = 0; column < 3; ++column)
            {
                long double const expected =
                    (row == column ? cosine : 0) + (1 - cosine) * axis[row] * axis[column] + sine * skew[row][column];
                test.expect(std::abs(static_cast<long double>(R(row, column)) - expected) <= sample_envelope<Scalar>(),
                            profile, "Exp rotation agrees with independent Rodrigues matrix (sample envelope)");
            }
        }
        if (square == 0)
        {
            test.expect(coefficients_equal(output, {1, 0, 0, 0}), profile, "Exp zero is exact identity");
        }
    }
}

template <typename Base> void exp_failure(Context & test, std::string_view profile)
{
    using B = ProbeBackend<Base>;
    using Scalar = typename B::value_type;
    using Vector = la::Vector<B, 3>;
    using Q = so::UnitQuaternion<B>;
    auto const good = Vector::from_row_major({1, 2, 3});
    auto check = [&](Vector const & input, Scalar threshold, Status expected)
    {
        Q output = -Q::identity();
        auto const before = output;
        auto const saved = input;
        test.expect(so::exp(input, threshold, output) == expected &&
                        same_fields(output.coefficients(), before.coefficients()) && same_fields(input, saved),
                    profile, "Exp ordered failure preserves all quaternion/input fields");
    };
    B::reset();
    for (Scalar invalid : {std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity(),
                           -std::numeric_limits<Scalar>::infinity()})
    {
        check(good, invalid, Status::non_finite_input);
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            auto bad = good;
            bad(axis) = invalid;
            check(bad, Scalar{0}, Status::non_finite_input);
        }
    }
    check(good, Scalar{0}, Status::domain_error);
    check(good, Scalar{-1}, Status::domain_error);
    auto large = good;
    large(0) = std::numeric_limits<Scalar>::max();
    check(large, Scalar{0.125}, Status::non_finite_result);
    B::override_squared = true;
    B::squared_value = Scalar{-1};
    check(good, Scalar{0.125}, Status::domain_error);
    B::squared_value = std::numeric_limits<Scalar>::infinity();
    check(good, Scalar{0.125}, Status::non_finite_result);
    B::reset();
    for (auto fault : {MathFault::root, MathFault::sine, MathFault::cosine, MathFault::zero_root})
    {
        B::scalar_math_type::fault = fault;
        check(good, Scalar{0.125}, Status::non_finite_result);
        B::reset();
    }
    B::override_norm = true;
    B::norm_value = Scalar{0};
    check(good, Scalar{0.125}, Status::invalid_quaternion_norm);
    B::reset();
    B::bad_division = true;
    check(good, Scalar{0.125}, Status::non_finite_result);
    B::reset();
    check(Vector{}, Scalar{2}, Status::invalid_quaternion_norm);
}

template <typename B> auto boundary_vector()
{
    using Scalar = typename B::value_type;
    la::Vector<B, 3> v;
    if constexpr (std::is_same_v<Scalar, float>)
    {
        v(0) = std::ldexp(Scalar{1}, -12);
        v(1) = v(0);
    }
    else
    {
        v(0) = std::ldexp(Scalar{1}, -26);
    }
    return v;
}

template <typename Base> void small_exp(Context & test, std::string_view profile)
{
    using B = ProbeBackend<Base>;
    using Scalar = typename B::value_type;
    using Q = so::UnitQuaternion<B>;
    auto const boundary = boundary_vector<B>();
    for (int side : {-1, 0, 1})
    {
        B::reset();
        auto v = boundary;
        if (side != 0)
        {
            v(0) = std::nextafter(v(0), side < 0 ? Scalar{0} : Scalar{1});
        }
        Scalar const square = v(0) * v(0) + v(1) * v(1) + v(2) * v(2);
        test.expect(side == 0 ? square == std::numeric_limits<Scalar>::epsilon()
                              : (side < 0 ? square < std::numeric_limits<Scalar>::epsilon()
                                          : square > std::numeric_limits<Scalar>::epsilon()),
                    profile, "encoded vector squared norm reaches the intended side of epsilon");
        Q output;
        auto const status = so::exp(v, Scalar{0.125}, output);
        test.expect(status == Status::success && B::scalar_math_type::sines == (side > 0 ? 1U : 0U) &&
                        B::scalar_math_type::cosines == (side > 0 ? 1U : 0U),
                    profile, "passive scalar seam observes Taylor/half-angle dispatch");
        if (side <= 0)
        {
            Scalar const fourth = square * square;
            Scalar const q0 = Scalar{1} - square / Scalar{8} + fourth / Scalar{384};
            Scalar const scale = Scalar{0.5} - square / Scalar{48} + fourth / Scalar{3840};
            // In these epsilon-scale fixtures the normalized candidate has
            // exactly unit computed norm; no general Taylor error claim follows.
            test.expect(output.q0() == q0 && output.q1() == scale * v(0) && output.q2() == scale * v(1) &&
                            output.q3() == scale * v(2),
                        profile, "selected scalar-format Taylor candidate in exactly rounded boundary fixture");
        }
    }
    B::reset();
    Q output;
    test.expect(so::exp(la::Vector<B, 3>{}, Scalar{0.125}, output) == Status::success &&
                    coefficients_equal(output, {1, 0, 0, 0}) && B::scalar_math_type::sines == 0,
                profile, "zero uses nonsingular branch without trig");
    B::reset();
}

template <typename B> void log_map(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    using Vector = la::Vector<B, 3>;
    auto const general = quaternion<B>(test, profile, {2, 1, -3, 4});
    auto const near_cut = quaternion<B>(test, profile, {std::numeric_limits<Scalar>::epsilon(), 1, 0, 0});
    for (auto const value :
         {so::UnitQuaternion<B>::identity(), general, near_cut, quaternion<B>(test, profile, {0, 1, 0, 0})})
    {
        for (auto const q : {value, -value})
        {
            Vector output = Vector::from_row_major({7, 8, 9});
            auto const status = so::log(q, output);
            long double const sign = q.q0() < Scalar{0} ? -1 : 1;
            long double norm =
                std::sqrt(static_cast<long double>(q.q1()) * q.q1() + static_cast<long double>(q.q2()) * q.q2() +
                          static_cast<long double>(q.q3()) * q.q3());
            long double const scale = norm == 0 ? 0 : 2 * std::atan2(norm, sign * q.q0()) / norm;
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                long double const expected = scale * sign * q.coefficients()(axis + 1);
                test.expect(
                    status == Status::success &&
                        std::abs(static_cast<long double>(output(axis)) - expected) <= sample_envelope<Scalar>(),
                    profile,
                    "independent principal coefficient model; at pi either signed axis represents the rotation");
            }
            if (norm == 0)
            {
                test.expect(output(0) == Scalar{0} && output(1) == Scalar{0} && output(2) == Scalar{0}, profile,
                            "both identity signs map exactly to zero");
            }
        }
    }
}

template <typename Base> void small_log(Context & test, std::string_view profile)
{
    using B = ProbeBackend<Base>;
    using Scalar = typename B::value_type;
    using Q = so::UnitQuaternion<B>;
    using Vector = la::Vector<B, 3>;
    auto const boundary = boundary_vector<B>();
    // The dyadic seed has a rounded total squared norm of one in both formats:
    // q0*q0 rounds to 1-epsilon and the vector contributes epsilon. Adjacent
    // vector steps perturb that total only by O(epsilon^2), so either backend
    // reduction order leaves normalization unchanged at the requested boundary.
    Scalar const q0 = Scalar{1} - std::numeric_limits<Scalar>::epsilon() / Scalar{2};
    for (int side : {-1, 0, 1})
    {
        B::reset();
        auto v = boundary;
        if (side != 0)
        {
            v(0) = std::nextafter(v(0), side < 0 ? Scalar{0} : Scalar{1});
        }
        Q q;
        test.expect(Q::from_coefficients(q0, v(0), v(1), v(2), Scalar{0.125}, q) == Status::success, profile,
                    "public small-vector quaternion fixture");
        test.expect(q.q0() == q0 && q.q1() == v(0) && q.q2() == v(1) && q.q3() == v(2), profile,
                    "normalization preserves the independently constructed boundary coefficients");
        Scalar const square = q.q1() * q.q1() + q.q2() * q.q2() + q.q3() * q.q3();
        test.expect(side == 0 ? square == std::numeric_limits<Scalar>::epsilon()
                              : (side < 0 ? square < std::numeric_limits<Scalar>::epsilon()
                                          : square > std::numeric_limits<Scalar>::epsilon()),
                    profile, "actual normalized coefficient squared norm reaches branch side");
        B::scalar_math_type::reset();
        Vector output;
        auto const status = so::log(q, output);
        test.expect(status == Status::success && B::scalar_math_type::atans == (side > 0 ? 1U : 0U), profile,
                    "passive atan2 seam identifies Log branch");
        if (side <= 0)
        {
            Scalar const scale = Scalar{2} / q.q0() * (Scalar{1} - square / (Scalar{3} * (q.q0() * q.q0())));
            test.expect(output(0) == q.q1() * scale && output(1) == q.q2() * scale && output(2) == q.q3() * scale,
                        profile, "selected IEEE Taylor scale without asserting real unit residual");
        }
    }
    B::reset();
    Vector output = Vector::from_row_major({7, 8, 9});
    test.expect(so::log(Q::identity(), output) == Status::success && output(0) == Scalar{0} &&
                    B::scalar_math_type::atans == 0,
                profile, "zero vector branch precedes scale division");
    auto const half_turn = quaternion<B>(test, profile, {0, 1, 0, 0});
    B::override_squared = true;
    B::squared_value = std::numeric_limits<Scalar>::epsilon();
    output = Vector::from_row_major({7, 8, 9});
    auto const before = output;
    test.expect(so::log(half_turn, output) == Status::zero_or_unsafe_divisor && same_fields(output, before), profile,
                "controlled inconsistent norm reaches q0-squared unsafe guard, not deployed unit-input evidence");
    B::reset();
}

template <typename B> void log_inverse(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    using Vector = la::Vector<B, 3>;
    using Q = so::UnitQuaternion<B>;
    for (auto const & phi : {Vector{}, Vector::from_row_major({0.25, -0.5, 1}), Vector::from_row_major({2, -1, 0.5}),
                             Vector::from_row_major({8, 0, 0})})
    {
        Q q;
        Vector recovered;
        test.expect(so::exp(phi, Scalar{0.125}, q) == Status::success && so::log(q, recovered) == Status::success,
                    profile, "actual Exp and Log composition succeeds on bounded finite fixtures");
        long double squared{};
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            squared += static_cast<long double>(phi(axis)) * phi(axis);
        }
        if (squared < 9)
        {
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                test.expect(std::abs(static_cast<long double>(recovered(axis)) - phi(axis)) <=
                                sample_envelope<Scalar>(),
                            profile, "local inverse inside open principal domain (sample envelope)");
            }
        }
        else
        {
            long double const expected = 8.0L - 2 * 3.141592653589793238462643383279502884L;
            test.expect(
                std::abs(static_cast<long double>(recovered(0)) - expected) <= sample_envelope<Scalar>() &&
                    recovered(1) == Scalar{0} && recovered(2) == Scalar{0},
                profile,
                "outside principal domain use independently wrapped angle rather than original-vector equality");
        }
    }
}

template <typename Base> void log_failure(Context & test, std::string_view profile)
{
    using B = ProbeBackend<Base>;
    using Scalar = typename B::value_type;
    using Q = so::UnitQuaternion<B>;
    using Vector = la::Vector<B, 3>;
    B::reset();
    auto const good = quaternion<B>(test, profile, {0.5, 0.5, 0.5, 0.5});
    auto check = [&](Q const & q, Status expected)
    {
        auto output = Vector::from_row_major({-Scalar{0}, std::numeric_limits<Scalar>::quiet_NaN(), 7});
        auto const before = output;
        auto const saved = q.coefficients();
        test.expect(so::log(q, output) == expected && same_fields(output, before) &&
                        same_fields(q.coefficients(), saved),
                    profile, "Log stage failure preserves all vector/input fields");
    };
    B::override_squared = true;
    B::squared_value = Scalar{-1};
    check(good, Status::domain_error);
    B::squared_value = std::numeric_limits<Scalar>::infinity();
    check(good, Status::non_finite_result);
    B::reset();
    for (auto fault : {MathFault::root, MathFault::atan, MathFault::zero_root})
    {
        B::scalar_math_type::fault = fault;
        check(good, Status::non_finite_result);
        B::reset();
    }
    B::bad_scale = true;
    check(good, Status::non_finite_result);
    B::reset();
    Scalar const tiny = std::ldexp(Scalar{1}, std::is_same_v<Scalar, float> ? -74 : -537);
    Q q;
    test.expect(Q::from_coefficients(tiny, tiny / Scalar{2}, Scalar{0}, Scalar{0}, std::numeric_limits<Scalar>::min(),
                                     q) == Status::success,
                profile, "public norm-underflow seed for nonfinite arithmetic state");
    for (unsigned step = 0; step < 64 && la::all_finite(q.coefficients()); ++step)
    {
        q = q * q;
    }
    test.expect(!la::all_finite(q.coefficients()), profile,
                "public raw multiplication reaches malformed coefficients without layout writes");
    check(q, Status::non_finite_input);
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
        {"TC-Q-IDENTITY-VALUE", "binary32", identity<F>},
        {"TC-Q-IDENTITY-VALUE", "binary64", identity<D>},
        {"TC-Q-MUL-HAMILTON", "binary32", multiplication<F>},
        {"TC-Q-MUL-HAMILTON", "binary64", multiplication<D>},
        {"TC-Q-INVERSE-CONJUGATE", "binary32", inverse<F>},
        {"TC-Q-INVERSE-CONJUGATE", "binary64", inverse<D>},
        {"TC-Q-NORMALIZE-COEFFICIENTS", "binary32", normalization<F>},
        {"TC-Q-NORMALIZE-COEFFICIENTS", "binary64", normalization<D>},
        {"TC-Q-NORMALIZE-DOMAIN", "binary32", normalization_domain<F>},
        {"TC-Q-NORMALIZE-DOMAIN", "binary64", normalization_domain<D>},
        {"TC-Q-NORMALIZE-RESULT", "controlled.binary32", normalization_result<F>},
        {"TC-Q-NORMALIZE-RESULT", "controlled.binary64", normalization_result<D>},
        {"TC-Q-ROT-COMPOSE-ORDER", "binary32", composition<F>},
        {"TC-Q-ROT-COMPOSE-ORDER", "binary64", composition<D>},
        {"TC-Q-SIGN-COMPARE", "binary32", sign_compare<F>},
        {"TC-Q-SIGN-COMPARE", "binary64", sign_compare<D>},
        {"TC-Q-ROT-MATRIX-COEFFICIENTS", "binary32", matrix_coefficients<F>},
        {"TC-Q-ROT-MATRIX-COEFFICIENTS", "binary64", matrix_coefficients<D>},
        {"TC-Q-ROTATE-ACTION", "binary32", action<F>},
        {"TC-Q-ROTATE-ACTION", "binary64", action<D>},
        {"TC-SO3-HAT-COEFFICIENTS", "binary32", hat<F>},
        {"TC-SO3-HAT-COEFFICIENTS", "binary64", hat<D>},
        {"TC-SO3-EXP-MAP", "partial.binary32", exp_map<F>},
        {"TC-SO3-EXP-MAP", "partial.binary64", exp_map<D>},
        {"TC-SO3-EXP-FAILURE", "controlled+native.binary32", exp_failure<F>},
        {"TC-SO3-EXP-FAILURE", "controlled+native.binary64", exp_failure<D>},
        {"TC-SO3-SMALL-ANGLE-EXP", "partial.binary32", small_exp<F>},
        {"TC-SO3-SMALL-ANGLE-EXP", "partial.binary64", small_exp<D>},
        {"TC-SO3-LOG-MAP", "partial.binary32", log_map<F>},
        {"TC-SO3-LOG-MAP", "partial.binary64", log_map<D>},
        {"TC-SO3-SMALL-ANGLE-LOG", "partial.binary32", small_log<F>},
        {"TC-SO3-SMALL-ANGLE-LOG", "partial.binary64", small_log<D>},
        {"TC-SO3-LOG-INVERSE", "partial.binary32", log_inverse<F>},
        {"TC-SO3-LOG-INVERSE", "partial.binary64", log_inverse<D>},
        {"TC-SO3-LOG-FAILURE", "controlled+native.binary32", log_failure<F>},
        {"TC-SO3-LOG-FAILURE", "controlled+native.binary64", log_failure<D>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
