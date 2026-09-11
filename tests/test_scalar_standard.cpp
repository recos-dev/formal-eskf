/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cmath>
#include <iostream>
#include <limits>
#include <string_view>

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

template <typename Scalar> class NonFiniteResultMath
{
public:
    using value_type = Scalar;

    [[nodiscard]] static bool is_finite(value_type value) noexcept { return std::isfinite(value); }
    [[nodiscard]] static value_type sqrt(value_type) noexcept { return std::numeric_limits<value_type>::quiet_NaN(); }
    [[nodiscard]] static value_type sin(value_type) noexcept { return std::numeric_limits<value_type>::quiet_NaN(); }
    [[nodiscard]] static value_type cos(value_type) noexcept { return value_type{1}; }
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
    test_checked_sqrt<Math>(test, profile, tolerance);
    test_checked_trigonometry<Math>(test, profile, tolerance);
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
