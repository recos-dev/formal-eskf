/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <bit>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string_view>
#include <type_traits>

#include <formal_eskf/scalar/backend/standard.hpp>
#include <formal_eskf/scalar/math.hpp>

#include "test_support.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::test::near;

template <typename Math>
void test_primitives(TestContext & test, std::string_view profile, typename Math::value_type tolerance)
{
    using value_type = typename Math::value_type;

    test.expect(formal_eskf::scalar::is_finite<Math>(value_type{1}), profile, "finite value is accepted");
    test.expect(!formal_eskf::scalar::is_finite<Math>(std::numeric_limits<value_type>::infinity()), profile,
                "infinity is rejected");
    test.expect(!formal_eskf::scalar::is_finite<Math>(std::numeric_limits<value_type>::quiet_NaN()), profile,
                "NaN is rejected");
    test.expect(near(formal_eskf::scalar::absolute<Math>(value_type{-3}), value_type{3}, tolerance), profile,
                "absolute value");
    test.expect(near(formal_eskf::scalar::pi<Math>(), static_cast<value_type>(3.14159265358979323846), tolerance),
                profile, "pi constant");
}

template <typename Math> void test_backend_sqrt(TestContext & test, std::string_view profile)
{
    using value_type = typename Math::value_type;

    test.expect(Math::sqrt(value_type{4}) == value_type{2}, profile, "backend sqrt preserves valid results");
    value_type const zero = Math::sqrt(value_type{0});
    value_type const negative_zero = Math::sqrt(-value_type{0});
    test.expect(zero == value_type{0} && !std::signbit(zero) && negative_zero == value_type{0} &&
                    std::signbit(negative_zero),
                profile, "backend sqrt preserves both signs of zero");
    value_type const infinity = std::numeric_limits<value_type>::infinity();
    test.expect(Math::sqrt(infinity) == infinity &&
                    std::isnan(Math::sqrt(std::numeric_limits<value_type>::quiet_NaN())),
                profile, "backend sqrt preserves infinity and NaN classifications");
    test.expect(std::isnan(Math::sqrt(value_type{-1})) && std::isnan(Math::sqrt(-infinity)), profile,
                "negative backend sqrt inputs return NaN without an unchecked libm call");
}

template <typename Math> void test_ieee_edges(TestContext & test, std::string_view profile)
{
    using value_type = typename Math::value_type;
    using bits_type = std::conditional_t<std::is_same_v<value_type, float>, std::uint32_t, std::uint64_t>;
    constexpr bits_type pi_bits = []
    {
        if constexpr (std::is_same_v<value_type, float>)
            return bits_type{0x40490fdbU};
        else
            return bits_type{0x400921fb54442d18ULL};
    }();
    test.expect(std::bit_cast<bits_type>(Math::pi()) == pi_bits &&
                    std::bit_cast<bits_type>(formal_eskf::scalar::pi<Math>()) == pi_bits,
                profile, "native standard pi matches the independently specified IEEE encoding");
    value_type const subnormal = std::numeric_limits<value_type>::denorm_min();
    test.expect(Math::is_finite(subnormal) && Math::is_finite(-subnormal) && Math::absolute(-subnormal) == subnormal &&
                    !std::signbit(Math::absolute(-value_type{0})),
                profile, "finite and absolute preserve subnormals and clear negative zero");
    value_type const infinity = std::numeric_limits<value_type>::infinity();
    test.expect(!Math::is_finite(-infinity) && Math::absolute(-infinity) == infinity &&
                    std::isnan(Math::absolute(std::numeric_limits<value_type>::quiet_NaN())),
                profile, "negative infinity and NaN classification");
    for (value_type y : {value_type{0}, -value_type{0}})
    {
        for (value_type x : {value_type{0}, -value_type{0}})
        {
            value_type output{7};
            auto const status = formal_eskf::scalar::try_atan2<Math>(y, x, output);
            test.expect(status == formal_eskf::Status::domain_error && output == value_type{7}, profile,
                        "atan2 rejects all four signed-zero origins without publication");
        }
        value_type output{7};
        auto const status = formal_eskf::scalar::try_atan2<Math>(y, value_type{-1}, output);
        test.expect(status == formal_eskf::Status::success && std::signbit(output) == std::signbit(y) &&
                        Math::absolute(output) == Math::pi(),
                    profile, "native atan2 preserves both sides of the negative-axis branch cut");
    }
}

template <typename Math> void test_aliases(TestContext & test, std::string_view profile)
{
    using value_type = typename Math::value_type;
    using pair_type = formal_eskf::scalar::SinCos<value_type>;
    value_type root{4};
    auto status = formal_eskf::scalar::try_sqrt<Math>(root, root);
    test.expect(status == formal_eskf::Status::success && root == value_type{2}, profile, "in-place sqrt");
    root = -value_type{0};
    status = formal_eskf::scalar::try_sqrt<Math>(root, root);
    test.expect(status == formal_eskf::Status::success && std::signbit(root), profile, "in-place negative-zero sqrt");
    root = value_type{-1};
    status = formal_eskf::scalar::try_sqrt<Math>(root, root);
    test.expect(status == formal_eskf::Status::domain_error && root == value_type{-1}, profile,
                "failed in-place sqrt preserves input");
    for (bool sine_alias : {false, true})
    {
        pair_type output{value_type{1}, value_type{2}};
        value_type const input = sine_alias ? output.sine : output.cosine;
        status = formal_eskf::scalar::try_sin_cos<Math>(sine_alias ? output.sine : output.cosine, output);
        test.expect(status == formal_eskf::Status::success && output.sine == Math::sin(input) &&
                        output.cosine == Math::cos(input),
                    profile, "sincos copies an aliased angle before publishing either component");
    }
    for (bool y_alias : {false, true})
    {
        value_type y{1}, x{-2};
        value_type const expected = Math::atan2(y, x);
        status = formal_eskf::scalar::try_atan2<Math>(y, x, y_alias ? y : x);
        test.expect(status == formal_eskf::Status::success && (y_alias ? y : x) == expected &&
                        (y_alias ? x == value_type{-2} : y == value_type{1}),
                    profile, "atan2 can publish to either input while preserving the other");
    }
    value_type shared{1};
    status = formal_eskf::scalar::try_atan2<Math>(shared, shared, shared);
    test.expect(status == formal_eskf::Status::success && shared == Math::atan2(value_type{1}, value_type{1}), profile,
                "atan2 permits both inputs and output to share one object");
}

template <typename Math>
void test_checked_sqrt(TestContext & test, std::string_view profile, typename Math::value_type tolerance)
{
    using value_type = typename Math::value_type;

    value_type output{9};
    formal_eskf::Status const success = formal_eskf::scalar::try_sqrt<Math>(value_type{4}, output);
    test.expect(success == formal_eskf::Status::success && near(output, value_type{2}, tolerance), profile,
                "sqrt succeeds on its domain");

    output = value_type{9};
    formal_eskf::Status const negative = formal_eskf::scalar::try_sqrt<Math>(value_type{-1}, output);
    test.expect(negative == formal_eskf::Status::domain_error && output == value_type{9}, profile,
                "negative sqrt fails without changing output");

    formal_eskf::Status const non_finite =
        formal_eskf::scalar::try_sqrt<Math>(std::numeric_limits<value_type>::infinity(), output);
    test.expect(non_finite == formal_eskf::Status::non_finite_input && output == value_type{9}, profile,
                "non-finite sqrt input fails without changing output");
}

template <typename Math>
void test_checked_trigonometry(TestContext & test, std::string_view profile, typename Math::value_type tolerance)
{
    using value_type = typename Math::value_type;
    using sin_cos_type = formal_eskf::scalar::SinCos<value_type>;

    value_type const pi_value = formal_eskf::scalar::pi<Math>();
    sin_cos_type sin_cos{value_type{7}, value_type{8}};
    formal_eskf::Status const sin_cos_status =
        formal_eskf::scalar::try_sin_cos<Math>(pi_value / value_type{6}, sin_cos);
    test.expect(sin_cos_status == formal_eskf::Status::success && near(sin_cos.sine, value_type{0.5}, tolerance) &&
                    near(sin_cos.cosine, static_cast<value_type>(0.86602540378443864676), tolerance),
                profile, "sine and cosine succeed together");

    sin_cos = sin_cos_type{value_type{7}, value_type{8}};
    formal_eskf::Status const non_finite_sin_cos =
        formal_eskf::scalar::try_sin_cos<Math>(std::numeric_limits<value_type>::quiet_NaN(), sin_cos);
    test.expect(non_finite_sin_cos == formal_eskf::Status::non_finite_input && sin_cos.sine == value_type{7} &&
                    sin_cos.cosine == value_type{8},
                profile, "non-finite angle fails without changing output");

    value_type angle{7};
    formal_eskf::Status const atan2_status = formal_eskf::scalar::try_atan2<Math>(value_type{1}, value_type{0}, angle);
    test.expect(atan2_status == formal_eskf::Status::success && near(angle, pi_value / value_type{2}, tolerance),
                profile, "atan2 preserves quadrant");

    angle = value_type{7};
    formal_eskf::Status const undefined = formal_eskf::scalar::try_atan2<Math>(value_type{0}, value_type{0}, angle);
    test.expect(undefined == formal_eskf::Status::domain_error && angle == value_type{7}, profile,
                "atan2 at the zero pair fails without changing output");
}

template <typename Scalar, bool CosineFails = false> class NonFiniteResultMath
{
public:
    using value_type = Scalar;

    [[nodiscard]] static bool is_finite(value_type value) noexcept { return std::isfinite(value); }
    [[nodiscard]] static value_type sqrt(value_type) noexcept { return std::numeric_limits<value_type>::quiet_NaN(); }
    [[nodiscard]] static value_type sin(value_type) noexcept
    {
        return CosineFails ? value_type{1} : std::numeric_limits<value_type>::quiet_NaN();
    }
    [[nodiscard]] static value_type cos(value_type) noexcept
    {
        return CosineFails ? std::numeric_limits<value_type>::quiet_NaN() : value_type{1};
    }
    [[nodiscard]] static value_type atan2(value_type, value_type) noexcept
    {
        return std::numeric_limits<value_type>::quiet_NaN();
    }
};

void test_non_finite_results(TestContext & test)
{
    using math_type = NonFiniteResultMath<double>;

    double scalar_output = 7.0;
    formal_eskf::Status const sqrt_status = formal_eskf::scalar::try_sqrt<math_type>(4.0, scalar_output);
    test.expect(sqrt_status == formal_eskf::Status::non_finite_result && scalar_output == 7.0, "fault math",
                "sqrt rejects a non-finite math result");

    formal_eskf::scalar::SinCos<double> sin_cos{7.0, 8.0};
    formal_eskf::Status const sin_cos_status = formal_eskf::scalar::try_sin_cos<math_type>(1.0, sin_cos);
    test.expect(sin_cos_status == formal_eskf::Status::non_finite_result && sin_cos.sine == 7.0 &&
                    sin_cos.cosine == 8.0,
                "fault math", "sine and cosine reject a non-finite math result");

    formal_eskf::Status const atan2_status = formal_eskf::scalar::try_atan2<math_type>(1.0, 1.0, scalar_output);
    test.expect(atan2_status == formal_eskf::Status::non_finite_result && scalar_output == 7.0, "fault math",
                "atan2 rejects a non-finite math result");
}

template <typename Math>
void run_conformance_tests(TestContext & test, std::string_view profile, typename Math::value_type tolerance)
{
    test_primitives<Math>(test, profile, tolerance);
    test_backend_sqrt<Math>(test, profile);
    test_ieee_edges<Math>(test, profile);
    test_aliases<Math>(test, profile);
    test_checked_sqrt<Math>(test, profile, tolerance);
    test_checked_trigonometry<Math>(test, profile, tolerance);
    using value_type = typename Math::value_type;
    formal_eskf::scalar::SinCos<value_type> output{value_type{1}, value_type{2}};
    auto const status = formal_eskf::scalar::try_sin_cos<NonFiniteResultMath<value_type, true>>(output.sine, output);
    test.expect(status == formal_eskf::Status::non_finite_result && output.sine == value_type{1} &&
                    output.cosine == value_type{2},
                profile, "failed cosine preserves the entire pair even when the angle aliases its sine");
}

} /* end namespace */

int main()
{
    TestContext test;

    run_conformance_tests<formal_eskf::scalar::StandardMath<double>>(test, "binary64", 1.0e-12);
    run_conformance_tests<formal_eskf::scalar::StandardMath<float>>(test, "binary32", 1.0e-5F);
    test_non_finite_results(test);

    if (test.failures() != 0)
    {
        std::cerr << test.failures() << " scalar math test(s) failed\n";
        return 1;
    }

    std::cout << "All standard scalar-math conformance tests passed\n";
    return 0;
}
