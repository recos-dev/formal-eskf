/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string_view>
#include <type_traits>

#include <formal_eskf/so3/right_jacobian.hpp>
#include <formal_eskf/so3/rotation_vector.hpp>
#include <formal_eskf/linalg/backend/eigen.hpp>

#include "test_support.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::Status;
using formal_eskf::test::near;

// Independent oracle: integrate the matrix-exponential power series,
// J_r(phi) = sum_{k=0}^infinity (-hat(phi))^k / (k+1)!.
// The tests using this reference have norm(phi) < 7; 64 terms give an
// adequate truncation margin, and long double reduces reference roundoff.
template <typename Vector> [[nodiscard]] auto jacobian_series_reference(Vector const & phi)
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
    using vector3_type = typename Linalg::template vector_type<3U>;
    using matrix3_type = typename Linalg::template matrix_type<3U, 3U>;
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
                phi.set(direction, angle);
            }
            else
            {
                // Unit mixed axis [2, -3, 6]/7, with all cross terms nonzero.
                phi.set(0U, angle * (value_type{2} / value_type{7}));
                phi.set(1U, angle * (value_type{-3} / value_type{7}));
                phi.set(2U, angle * (value_type{6} / value_type{7}));
            }
            matrix3_type jacobian;
            Status const status = formal_eskf::so3::try_right_jacobian(phi, jacobian);
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
            Status const negative_status = formal_eskf::so3::try_right_jacobian(-phi, negative);
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
    test.expect(formal_eskf::so3::try_right_jacobian(phi, jacobian) == Status::success &&
                    formal_eskf::linalg::max_abs(jacobian - matrix3_type::identity()) == value_type{0},
                profile, "zero Jacobian is exactly identity without division by zero");
    phi.set(0U, value_type{0.75});
    test.expect(formal_eskf::so3::try_right_jacobian(phi, jacobian) == Status::success &&
                    jacobian(1U, 1U) < value_type{0.9375},
                profile, "closed form is not silently replaced by I-hat(phi)/2");

    // Very large but representable squared norms must not form theta^3 or
    // multiply an unscaled hat(phi)^2 by a tiny coefficient.
    phi.set(0U, static_cast<value_type>(1.0e10));
    test.expect(formal_eskf::so3::try_right_jacobian(phi, jacobian) == Status::success &&
                    formal_eskf::linalg::all_finite(jacobian) && near(jacobian(0U, 0U), value_type{1}, tolerance),
                profile, "large finite angle uses a bounded unit-axis expression");
}

template <typename Linalg> void test_reset_differential(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using quaternion_type = formal_eskf::so3::UnitQuaternion<Linalg>;
    using vector3_type = typename Linalg::template vector_type<3U>;
    using matrix3_type = typename Linalg::template matrix_type<3U, 3U>;
    constexpr value_type minimum_norm = static_cast<value_type>(1.0e-6);
    value_type const step =
        std::is_same_v<value_type, float> ? static_cast<value_type>(0.005) : static_cast<value_type>(1.0e-5);
    value_type const tolerance =
        std::is_same_v<value_type, float> ? static_cast<value_type>(5.0e-5) : static_cast<value_type>(3.0e-10);
    vector3_type phi;
    phi.set(0U, value_type{0.25});
    phi.set(1U, value_type{-0.375});
    phi.set(2U, value_type{0.5});
    quaternion_type nominal;
    matrix3_type jacobian;
    bool correct = formal_eskf::so3::try_exp(phi, minimum_norm, nominal) == Status::success &&
                   formal_eskf::so3::try_right_jacobian(phi, jacobian) == Status::success;
    for (std::size_t column = 0U; column < 3U; ++column)
    {
        auto plus = phi;
        auto minus = phi;
        plus.set(column, plus(column) + step);
        minus.set(column, minus(column) - step);
        quaternion_type q_plus;
        quaternion_type q_minus;
        vector3_type log_plus;
        vector3_type log_minus;
        correct = correct && formal_eskf::so3::try_exp(plus, minimum_norm, q_plus) == Status::success &&
                  formal_eskf::so3::try_exp(minus, minimum_norm, q_minus) == Status::success &&
                  formal_eskf::so3::try_log(nominal.inverse() * q_plus, log_plus) == Status::success &&
                  formal_eskf::so3::try_log(nominal.inverse() * q_minus, log_minus) == Status::success;
        auto const derivative = (log_plus - log_minus) / (value_type{2} * step);
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            correct = correct && near(derivative(row), jacobian(row, column), tolerance);
        }
    }
    test.expect(correct, profile, "J_r is the reset differential, not its inverse or the left Jacobian");
}

template <typename Linalg> void test_jacobian_failures(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using vector3_type = typename Linalg::template vector_type<3U>;
    using matrix3_type = typename Linalg::template matrix_type<3U, 3U>;
    auto const sentinel = matrix3_type::identity() * value_type{2};
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        for (value_type const invalid :
             {std::numeric_limits<value_type>::quiet_NaN(), std::numeric_limits<value_type>::infinity(),
              -std::numeric_limits<value_type>::infinity()})
        {
            vector3_type phi;
            phi.set(axis, invalid);
            auto output = sentinel;
            Status const status = formal_eskf::so3::try_right_jacobian(phi, output);
            test.expect(status == Status::non_finite_input &&
                            formal_eskf::linalg::max_abs(output - sentinel) == value_type{0},
                        profile, "non-finite input preserves output");
        }
        vector3_type phi;
        phi.set(axis, std::numeric_limits<value_type>::max());
        auto output = sentinel;
        Status const status = formal_eskf::so3::try_right_jacobian(phi, output);
        test.expect(status == Status::non_finite_result &&
                        formal_eskf::linalg::max_abs(output - sentinel) == value_type{0},
                    profile, "squared-norm overflow preserves output");
    }
}

template <typename Linalg>
void run_jacobian_tests(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    test_jacobian_values<Linalg>(test, profile, tolerance);
    test_reset_differential<Linalg>(test, profile);
    test_jacobian_failures<Linalg>(test, profile);
}

} /* end namespace */

int main()
{
    TestContext test;
    Eigen::internal::set_is_malloc_allowed(false);
    run_jacobian_tests<formal_eskf::linalg::EigenBackend<float>>(test, "binary32", 5.0e-7F);
    run_jacobian_tests<formal_eskf::linalg::EigenBackend<double>>(test, "binary64", 1.0e-12);
    if (test.failures() != 0)
    {
        std::cerr << test.failures() << " SO(3) right-Jacobian test(s) failed\n";
        return 1;
    }
    std::cout << "All SO(3) right-Jacobian tests passed\n";
    return 0;
}
