/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this file.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <limits>
#include <type_traits>

#include <formal_eskf/scalar/backend/standard.hpp>
#include <formal_eskf/scalar/math.hpp>

#include "test_cases.hpp"

namespace
{
using formal_eskf::Status;
using formal_eskf::test::Context;

// Outward-rounded intervals give a reference independent of deployed libm.
// RN arithmetic, gradual underflow and strict compilation are profile premises.
struct Interval
{
    long double low;
    long double high;
};
long double down(long double value) { return std::nextafter(value, -std::numeric_limits<long double>::infinity()); }
long double up(long double value) { return std::nextafter(value, std::numeric_limits<long double>::infinity()); }
Interval add(Interval a, Interval b) { return {down(a.low + b.low), up(a.high + b.high)}; }
Interval multiply(Interval a, Interval b)
{
    std::array const products{a.low * b.low, a.low * b.high, a.high * b.low, a.high * b.high};
    return {down(*std::min_element(products.begin(), products.end())),
            up(*std::max_element(products.begin(), products.end()))};
}
Interval divide(Interval value, long double positive)
{
    return {down(value.low / positive), up(value.high / positive)};
}
Interval pi_interval()
{
    constexpr long double pi = 3.141592653589793238462643383279502884L;
    return {down(pi), up(pi)};
}
std::array<Interval, 2> trig_reference(long double angle, int turns)
{
    Interval x = add({angle, angle}, multiply({-2.0L * turns, -2.0L * turns}, pi_interval()));
    Interval const negative_square = multiply({-1, -1}, multiply(x, x));
    Interval sine = x, cosine{1, 1}, sine_term = x, cosine_term{1, 1};
    for (int term = 1; term <= 32; ++term)
    {
        sine_term = divide(multiply(sine_term, negative_square), (2.0L * term) * (2.0L * term + 1));
        cosine_term = divide(multiply(cosine_term, negative_square), (2.0L * term - 1) * (2.0L * term));
        sine = add(sine, sine_term);
        cosine = add(cosine, cosine_term);
    }
    // Every fixture has |reduced x| <= 4. The omitted alternating tail is
    // smaller than its next term, hence conservatively < 2*last-term magnitude.
    auto widen = [](Interval sum, Interval last)
    {
        long double const bound = up(2 * std::max(std::abs(last.low), std::abs(last.high)));
        return add(sum, {-bound, bound});
    };
    return {widen(sine, sine_term), widen(cosine, cosine_term)};
}

template <typename Scalar> bool contains_fixture_result(Scalar actual, Interval reference)
{
    // A one-adjacent-value allowance around the reference enclosure is a
    // declared finite-fixture regression target, not a uniform libm bound or
    // approval of the outstanding F-SCALAR numerical acceptance criteria.
    auto const low = std::nextafter(static_cast<Scalar>(reference.low), -std::numeric_limits<Scalar>::infinity());
    auto const high = std::nextafter(static_cast<Scalar>(reference.high), std::numeric_limits<Scalar>::infinity());
    return std::isfinite(actual) && actual >= low && actual <= high;
}

template <typename Scalar> bool same(Scalar a, Scalar b)
{
    using bytes = std::array<std::byte, sizeof(Scalar)>;
    return (std::isnan(a) && std::isnan(b)) || std::bit_cast<bytes>(a) == std::bit_cast<bytes>(b);
}

template <typename Scalar> void basic(Context & test, std::string_view profile)
{
    using Math = formal_eskf::scalar::StandardMath<Scalar>;
    using limits = std::numeric_limits<Scalar>;
    for (Scalar magnitude : {Scalar{0}, limits::denorm_min(), limits::min(), Scalar{3}, limits::max()})
    {
        for (Scalar sign : {Scalar{-1}, Scalar{1}})
        {
            Scalar const input = sign * magnitude;
            auto const result = formal_eskf::scalar::absolute<Math>(input);
            test.expect(formal_eskf::scalar::is_finite<Math>(input) && result == magnitude && !std::signbit(result),
                        profile, "finite classification and unsigned magnitude including negative zero/subnormal");
        }
    }
    for (Scalar input : {limits::infinity(), -limits::infinity(), limits::quiet_NaN()})
    {
        auto const result = formal_eskf::scalar::absolute<Math>(input);
        test.expect(!formal_eskf::scalar::is_finite<Math>(input) &&
                        (std::isnan(input) ? std::isnan(result) : result == limits::infinity()),
                    profile, "exceptional classification and magnitude");
    }
    // Both endpoints round to the independently known correctly rounded pi.
    auto const interval = pi_interval();
    test.expect(static_cast<Scalar>(interval.low) == static_cast<Scalar>(interval.high) &&
                    formal_eskf::scalar::pi<Math>() == static_cast<Scalar>(interval.low),
                profile, "pi agrees with independent enclosing decimal constant");
}

template <typename Scalar> void root(Context & test, std::string_view profile)
{
    using Math = formal_eskf::scalar::StandardMath<Scalar>;
    using limits = std::numeric_limits<Scalar>;
    for (Scalar expected : {Scalar{0}, -Scalar{0}, Scalar{0.5}, Scalar{1}, Scalar{2}, Scalar{16}})
    {
        Scalar const input = expected == Scalar{0} ? expected : expected * expected;
        for (bool alias : {false, true})
        {
            Scalar argument = input, separate = -Scalar{0};
            Scalar & output = alias ? argument : separate;
            auto const status = formal_eskf::scalar::sqrt<Math>(argument, output);
            test.expect(status == Status::success && same(output, expected) && (alias || same(argument, input)),
                        profile, "exact nonnegative root and both zero signs, separate/in-place output");
        }
    }
    for (Scalar input : {-Scalar{1}, -limits::min(), -limits::denorm_min(), limits::infinity(), -limits::infinity(),
                         limits::quiet_NaN()})
    {
        for (bool alias : {false, true})
        {
            Scalar argument = input, separate = -Scalar{0};
            Scalar & output = alias ? argument : separate;
            Scalar const before = output;
            auto const status = formal_eskf::scalar::sqrt<Math>(argument, output);
            test.expect(status == (std::isfinite(input) ? Status::domain_error : Status::non_finite_input) &&
                            same(output, before) && (alias || same(argument, input)),
                        profile, "root input validation preserves complete output/input frame");
        }
        test.expect(std::isnan(Math::sqrt(input)) || input == limits::infinity(), profile,
                    "standard adapter preserves invalid raw-root classification");
    }
    test.expect(Math::sqrt(limits::infinity()) == limits::infinity(), profile, "raw positive-infinity root");
}

template <typename Scalar> void sin_cos(Context & test, std::string_view profile)
{
    using Math = formal_eskf::scalar::StandardMath<Scalar>;
    using Pair = formal_eskf::scalar::SinCos<Scalar>;
    for (Scalar angle : {Scalar{0}, -Scalar{0}, Scalar{-0.5}, Scalar{0.5}, Scalar{-1}, Scalar{1}, Scalar{-2}, Scalar{2},
                         Scalar{-4}, Scalar{4}, Scalar{-1024}, Scalar{1024}})
    {
        int const turns = angle == Scalar{1024} ? 163 : (angle == Scalar{-1024} ? -163 : 0);
        auto const reference = trig_reference(static_cast<long double>(angle), turns);
        for (unsigned alias = 0; alias < 3; ++alias)
        {
            Pair output{Scalar{7}, Scalar{-8}};
            if (alias == 1)
            {
                output.sine = angle;
            }
            if (alias == 2)
            {
                output.cosine = angle;
            }
            auto const status = formal_eskf::scalar::sin_cos<Math>(
                alias == 1 ? output.sine : (alias == 2 ? output.cosine : angle), output);
            test.expect(status == Status::success && contains_fixture_result(output.sine, reference[0]) &&
                            contains_fixture_result(output.cosine, reference[1]),
                        profile, "both fields match independent interval Taylor/reference reduction");
            if (angle == Scalar{0})
            {
                test.expect(same(output.sine, angle) && output.cosine == Scalar{1}, profile,
                            "exact signed-zero sine and unit cosine");
            }
        }
    }
    for (Scalar angle : {std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity(),
                         -std::numeric_limits<Scalar>::infinity()})
    {
        for (unsigned alias = 0; alias < 3; ++alias)
        {
            Pair output{-Scalar{0}, Scalar{-8}};
            if (alias == 1)
            {
                output.sine = angle;
            }
            if (alias == 2)
            {
                output.cosine = angle;
            }
            auto const before = output;
            auto const status = formal_eskf::scalar::sin_cos<Math>(
                alias == 1 ? output.sine : (alias == 2 ? output.cosine : angle), output);
            test.expect(status == Status::non_finite_input && same(output.sine, before.sine) &&
                            same(output.cosine, before.cosine),
                        profile, "nonfinite angle preserves both fields for every supported input alias");
        }
    }
}

template <typename Scalar> void arctangent(Context & test, std::string_view profile)
{
    using Math = formal_eskf::scalar::StandardMath<Scalar>;
    for (Scalar y : {-Scalar{1}, -Scalar{0}, Scalar{0}, Scalar{1}})
    {
        for (Scalar x : {-Scalar{1}, -Scalar{0}, Scalar{0}, Scalar{1}})
        {
            for (unsigned alias = 0; alias < 3; ++alias)
            {
                Scalar a = y, b = x, separate = Scalar{7};
                Scalar & output = alias == 1 ? a : (alias == 2 ? b : separate);
                Scalar const before = output;
                auto const status = formal_eskf::scalar::atan2<Math>(a, b, output);
                if (y == Scalar{0} && x == Scalar{0})
                {
                    test.expect(status == Status::domain_error && same(output, before), profile,
                                "all signed-zero origins are invalid without publication");
                }
                else if (y == Scalar{0} && x > Scalar{0})
                {
                    test.expect(status == Status::success && same(output, y), profile, "positive-axis signed zero");
                }
                else
                {
                    long double fraction = x == Scalar{0} ? 0.5L : (y == Scalar{0} ? 1.0L : (x > 0 ? 0.25L : 0.75L));
                    if (std::signbit(y))
                    {
                        fraction = -fraction;
                    }
                    auto const reference = multiply(pi_interval(), {fraction, fraction});
                    test.expect(status == Status::success && contains_fixture_result(output, reference), profile,
                                "independent axis/diagonal quadrant and argument-order oracle");
                }
                test.expect((alias == 1 || same(a, y)) && (alias == 2 || same(b, x)), profile,
                            "unaliased coordinates remain unchanged");
            }
        }
    }
    for (Scalar invalid : {std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity(),
                           -std::numeric_limits<Scalar>::infinity()})
    {
        for (bool bad_y : {false, true})
        {
            for (unsigned alias = 0; alias < 3; ++alias)
            {
                Scalar y = bad_y ? invalid : -Scalar{0}, x = bad_y ? Scalar{0} : invalid, separate = Scalar{7};
                Scalar & output = alias == 1 ? y : (alias == 2 ? x : separate);
                Scalar const before = output;
                auto const status = formal_eskf::scalar::atan2<Math>(y, x, output);
                test.expect(status == Status::non_finite_input && same(output, before), profile,
                            "either nonfinite coordinate precedes zero handling and preserves aliased output");
            }
        }
    }
    Scalar shared = Scalar{1};
    test.expect(formal_eskf::scalar::atan2<Math>(shared, shared, shared) == Status::success &&
                    contains_fixture_result(shared, divide(pi_interval(), 4)),
                profile, "both coordinates and output may share one scalar");
}

template <typename Scalar> struct ControlledMath
{
    using value_type = Scalar;
    static inline Scalar sqrt_result{}, sine_result{}, cosine_result{}, atan_result{}, first{}, second{};
    static inline unsigned calls{};
    static bool is_finite(Scalar value) { return std::isfinite(value); }
    static Scalar sqrt(Scalar value)
    {
        ++calls;
        first = value;
        return sqrt_result;
    }
    static Scalar sin(Scalar value)
    {
        ++calls;
        first = value;
        return sine_result;
    }
    static Scalar cos(Scalar value)
    {
        ++calls;
        second = value;
        return cosine_result;
    }
    static Scalar atan2(Scalar y, Scalar x)
    {
        ++calls;
        first = y;
        second = x;
        return atan_result;
    }
};

template <typename Scalar> void controlled_results(Context & test, std::string_view profile)
{
    using Math = ControlledMath<Scalar>;
    for (Scalar invalid : {std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity(),
                           -std::numeric_limits<Scalar>::infinity()})
    {
        Math::sqrt_result = Math::atan_result = invalid;
        Math::calls = 0;
        Scalar output = -Scalar{0};
        test.expect(formal_eskf::scalar::sqrt<Math>(Scalar{4}, output) == Status::non_finite_result &&
                        same(output, -Scalar{0}) && Math::calls == 1 && Math::first == Scalar{4},
                    profile, "controlled root result rejected with correct forwarding and no publication");
        Math::calls = 0;
        test.expect(formal_eskf::scalar::atan2<Math>(Scalar{2}, Scalar{-3}, output) == Status::non_finite_result &&
                        same(output, -Scalar{0}) && Math::calls == 1 && Math::first == Scalar{2} &&
                        Math::second == Scalar{-3},
                    profile, "controlled atan2 result rejected with y/x forwarding and no publication");
        for (bool failed_cosine : {false, true})
        {
            Math::sine_result = failed_cosine ? Scalar{0.25} : invalid;
            Math::cosine_result = failed_cosine ? invalid : Scalar{0.5};
            Math::calls = 0;
            formal_eskf::scalar::SinCos<Scalar> pair{Scalar{2}, -Scalar{0}};
            test.expect(formal_eskf::scalar::sin_cos<Math>(pair.sine, pair) == Status::non_finite_result &&
                            pair.sine == Scalar{2} && same(pair.cosine, -Scalar{0}) &&
                            (Math::calls == 2 || (!failed_cosine && Math::calls == 1)) && Math::first == Scalar{2} &&
                            (Math::calls == 1 || Math::second == Scalar{2}),
                        profile, "either controlled pair field fails atomically after correct forwarding");
        }
    }
    Math::sqrt_result = Scalar{-3};
    Math::atan_result = Scalar{9};
    Math::sine_result = Scalar{2};
    Math::cosine_result = Scalar{-4};
    Scalar output{};
    formal_eskf::scalar::SinCos<Scalar> pair;
    test.expect(formal_eskf::scalar::sqrt<Math>(Scalar{4}, output) == Status::success && output == Scalar{-3} &&
                    formal_eskf::scalar::atan2<Math>(Scalar{1}, Scalar{1}, output) == Status::success &&
                    output == Scalar{9} && formal_eskf::scalar::sin_cos<Math>(Scalar{1}, pair) == Status::success &&
                    pair.sine == Scalar{2} && pair.cosine == Scalar{-4},
                profile, "frontend publishes finite backend values unchanged; this is not backend conformance");
    Math::calls = 0;
    test.expect(formal_eskf::scalar::sqrt<Math>(Scalar{-1}, output) == Status::domain_error &&
                    formal_eskf::scalar::sin_cos<Math>(std::numeric_limits<Scalar>::infinity(), pair) ==
                        Status::non_finite_input &&
                    formal_eskf::scalar::atan2<Math>(Scalar{0}, Scalar{0}, output) == Status::domain_error &&
                    Math::calls == 0,
                profile, "frontend input rejection skips the elementary backend");
}
} // namespace

int main(int argc, char ** argv)
{
    using formal_eskf::test::Case;
    Case const cases[]{
        {"TC-F-SCALAR-BASIC", "binary32", basic<float>},
        {"TC-F-SCALAR-BASIC", "binary64", basic<double>},
        {"TC-F-SCALAR-SQRT", "binary32", root<float>},
        {"TC-F-SCALAR-SQRT", "binary64", root<double>},
        {"TC-F-SCALAR-SINCOS", "binary32", sin_cos<float>},
        {"TC-F-SCALAR-SINCOS", "binary64", sin_cos<double>},
        {"TC-F-SCALAR-ATAN2", "binary32", arctangent<float>},
        {"TC-F-SCALAR-ATAN2", "binary64", arctangent<double>},
        {"TC-F-SCALAR-RESULT", "controlled.binary32", controlled_results<float>},
        {"TC-F-SCALAR-RESULT", "controlled.binary64", controlled_results<double>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
