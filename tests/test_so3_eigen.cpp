/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string_view>

#include <formal_eskf/linalg/backend/eigen.hpp>
#include <formal_eskf/so3/so3.hpp>

#include "test_support.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::test::near;

template <typename Matrix>
[[nodiscard]] bool matrix_near(Matrix const & actual, Matrix const & expected, typename Matrix::value_type tolerance)
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

template <typename Linalg>
void test_construction(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using quaternion_type = formal_eskf::so3::UnitQuaternion<Linalg>;
    using value_type = typename Linalg::value_type;

    quaternion_type const identity;
    test.expect(identity.q0() == value_type{1} && identity.q1() == value_type{0} && identity.q2() == value_type{0} &&
                    identity.q3() == value_type{0},
                profile, "default construction produces identity");

    quaternion_type normalized;
    formal_eskf::Status const status = quaternion_type::try_from_coefficients(
        value_type{2}, value_type{0}, value_type{0}, value_type{0}, tolerance, normalized);
    test.expect(status == formal_eskf::Status::success && same_coefficients(identity, normalized), profile,
                "construction normalizes coefficients");

    quaternion_type negative_identity;
    formal_eskf::Status const negative_status = quaternion_type::try_from_coefficients(
        value_type{-2}, value_type{0}, value_type{0}, value_type{0}, tolerance, negative_identity);
    test.expect(negative_status == formal_eskf::Status::success && !same_coefficients(identity, negative_identity) &&
                    same_rotation(identity, negative_identity, tolerance),
                profile, "normalization preserves sign while rotation comparison accepts either sign");

    quaternion_type unchanged = negative_identity;
    formal_eskf::Status const zero_status = quaternion_type::try_from_coefficients(
        value_type{0}, value_type{0}, value_type{0}, value_type{0}, tolerance, unchanged);
    test.expect(zero_status == formal_eskf::Status::invalid_quaternion_norm &&
                    same_coefficients(unchanged, negative_identity),
                profile, "zero quaternion fails without changing output");

    formal_eskf::Status const non_finite_status =
        quaternion_type::try_from_coefficients(std::numeric_limits<value_type>::quiet_NaN(), value_type{0},
                                               value_type{0}, value_type{0}, tolerance, unchanged);
    test.expect(non_finite_status == formal_eskf::Status::non_finite_input &&
                    same_coefficients(unchanged, negative_identity),
                profile, "non-finite quaternion fails without changing output");

    formal_eskf::Status const invalid_threshold_status = quaternion_type::try_from_coefficients(
        value_type{1}, value_type{0}, value_type{0}, value_type{0}, value_type{0}, unchanged);
    test.expect(invalid_threshold_status == formal_eskf::Status::domain_error &&
                    same_coefficients(unchanged, negative_identity),
                profile, "invalid minimum norm fails without changing output");

    test.expect(near(formal_eskf::linalg::norm(normalized.coefficients()), value_type{1}, tolerance), profile,
                "successful construction produces unit norm");
}

template <typename Linalg>
void test_multiplication_and_inverse(TestContext & test, std::string_view profile,
                                     typename Linalg::value_type tolerance)
{
    using quaternion_type = formal_eskf::so3::UnitQuaternion<Linalg>;
    using value_type = typename Linalg::value_type;

    value_type const half_sqrt_two = static_cast<value_type>(0.70710678118654752440);
    quaternion_type qx;
    quaternion_type qy;
    quaternion_type expected;
    formal_eskf::Status const qx_status = quaternion_type::try_from_coefficients(
        half_sqrt_two, half_sqrt_two, value_type{0}, value_type{0}, tolerance, qx);
    formal_eskf::Status const qy_status = quaternion_type::try_from_coefficients(
        half_sqrt_two, value_type{0}, half_sqrt_two, value_type{0}, tolerance, qy);
    formal_eskf::Status const expected_status = quaternion_type::try_from_coefficients(
        value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5}, tolerance, expected);
    test.expect(qx_status == formal_eskf::Status::success && qy_status == formal_eskf::Status::success &&
                    expected_status == formal_eskf::Status::success,
                profile, "quarter-turn fixtures are valid");

    quaternion_type const product = qx * qy;
    test.expect(same_rotation(product, expected, tolerance), profile, "Hamilton product follows q_ac=q_ab*q_bc");

    quaternion_type normalized_product;
    formal_eskf::Status const compose_status = try_compose_normalized(qx, qy, tolerance, normalized_product);
    test.expect(compose_status == formal_eskf::Status::success &&
                    same_rotation(normalized_product, expected, tolerance) &&
                    near(formal_eskf::linalg::norm(normalized_product.coefficients()), value_type{1}, tolerance),
                profile, "normalized composition preserves the rotation and unit norm");

    quaternion_type const identity = quaternion_type::identity();
    test.expect(same_rotation(qx * qx.inverse(), identity, tolerance) &&
                    same_rotation(qx.inverse() * qx, identity, tolerance),
                profile, "inverse is a two-sided inverse");

    auto const composed_matrix = to_rotation_matrix(product);
    auto const matrix_product = to_rotation_matrix(qx) * to_rotation_matrix(qy);
    test.expect(matrix_near(composed_matrix, matrix_product, tolerance), profile,
                "quaternion and rotation-matrix composition orders agree");
}

template <typename Linalg>
void test_exp_and_log(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using quaternion_type = formal_eskf::so3::UnitQuaternion<Linalg>;
    using scalar_math_type = typename Linalg::scalar_math_type;
    using value_type = typename Linalg::value_type;
    using vector3_type = typename Linalg::template vector_type<3U>;

    vector3_type const zero = vector3_type::zero();
    quaternion_type zero_exp;
    formal_eskf::Status const zero_exp_status = try_exp(zero, tolerance, zero_exp);
    test.expect(zero_exp_status == formal_eskf::Status::success &&
                    same_coefficients(zero_exp, quaternion_type::identity()),
                profile, "Exp_q(0) is exactly identity");

    value_type const pi = formal_eskf::scalar::pi<scalar_math_type>();
    value_type const half_sqrt_two = static_cast<value_type>(0.70710678118654752440);
    value_type const quarter_turn_values[3] = {value_type{0.5} * pi, value_type{0}, value_type{0}};
    vector3_type const quarter_turn = vector3_type::from_row_major(quarter_turn_values);

    quaternion_type quarter_turn_exp;
    formal_eskf::Status const quarter_turn_status = try_exp(quarter_turn, tolerance, quarter_turn_exp);
    quaternion_type expected_quarter_turn;
    formal_eskf::Status const expected_status = quaternion_type::try_from_coefficients(
        half_sqrt_two, half_sqrt_two, value_type{0}, value_type{0}, tolerance, expected_quarter_turn);
    test.expect(quarter_turn_status == formal_eskf::Status::success &&
                    expected_status == formal_eskf::Status::success &&
                    same_rotation(quarter_turn_exp, expected_quarter_turn, tolerance),
                profile, "Exp_q maps a pi/2 x-axis rotation to [cos(pi/4), sin(pi/4), 0, 0]");

    value_type const rotation_values[3] = {static_cast<value_type>(0.2), static_cast<value_type>(-0.3),
                                           static_cast<value_type>(0.4)};
    vector3_type const rotation_vector = vector3_type::from_row_major(rotation_values);
    quaternion_type quaternion;
    formal_eskf::Status const exp_status = try_exp(rotation_vector, tolerance, quaternion);

    vector3_type recovered;
    formal_eskf::Status const log_status = try_log(quaternion, recovered);
    vector3_type recovered_from_negative;
    formal_eskf::Status const negative_log_status = try_log(-quaternion, recovered_from_negative);
    test.expect(exp_status == formal_eskf::Status::success && log_status == formal_eskf::Status::success &&
                    negative_log_status == formal_eskf::Status::success &&
                    matrix_near(recovered, rotation_vector, tolerance) &&
                    matrix_near(recovered_from_negative, rotation_vector, tolerance),
                profile, "Log_q locally inverts Exp_q and gives q and -q the same principal result");

    value_type const small = std::numeric_limits<value_type>::epsilon();
    value_type const small_values[3] = {small, -value_type{2} * small, value_type{3} * small};
    vector3_type const small_rotation = vector3_type::from_row_major(small_values);
    quaternion_type small_exp;
    formal_eskf::Status const small_exp_status = try_exp(small_rotation, tolerance, small_exp);
    vector3_type small_recovered;
    formal_eskf::Status const small_log_status = try_log(small_exp, small_recovered);
    test.expect(small_exp_status == formal_eskf::Status::success && small_log_status == formal_eskf::Status::success &&
                    small_exp.q1() != value_type{0} && near(small_exp.q0(), value_type{1}, tolerance) &&
                    near(small_exp.q1() / small, value_type{0.5}, tolerance) &&
                    near(small_exp.q2() / small, value_type{-1}, tolerance) &&
                    near(small_exp.q3() / small, value_type{1.5}, tolerance) &&
                    near(small_recovered(0U) / small, value_type{1}, tolerance) &&
                    near(small_recovered(1U) / small, value_type{-2}, tolerance) &&
                    near(small_recovered(2U) / small, value_type{3}, tolerance),
                profile, "Exp_q and Log_q follow Sola's small-angle developments");

    vector3_type zero_log;
    formal_eskf::Status const identity_log_status = try_log(-quaternion_type::identity(), zero_log);
    test.expect(identity_log_status == formal_eskf::Status::success && matrix_near(zero_log, zero, tolerance), profile,
                "Log_q maps both identity representations to zero");

    value_type const long_rotation_values[3] = {value_type{1.5} * pi, value_type{0}, value_type{0}};
    vector3_type const long_rotation = vector3_type::from_row_major(long_rotation_values);
    quaternion_type long_rotation_exp;
    formal_eskf::Status const long_exp_status = try_exp(long_rotation, tolerance, long_rotation_exp);
    vector3_type principal_rotation;
    formal_eskf::Status const principal_log_status = try_log(long_rotation_exp, principal_rotation);
    value_type const expected_principal_values[3] = {-value_type{0.5} * pi, value_type{0}, value_type{0}};
    vector3_type const expected_principal = vector3_type::from_row_major(expected_principal_values);
    test.expect(long_exp_status == formal_eskf::Status::success &&
                    principal_log_status == formal_eskf::Status::success &&
                    matrix_near(principal_rotation, expected_principal, tolerance),
                profile, "Log_q returns a principal rotation vector with norm no greater than pi");
}

template <typename Linalg>
void test_exp_rotation_matrix(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using quaternion_type = formal_eskf::so3::UnitQuaternion<Linalg>;
    using matrix3_type = typename Linalg::template matrix_type<3U, 3U>;
    using value_type = typename Linalg::value_type;
    using vector3_type = typename Linalg::template vector_type<3U>;

    value_type const rotation_values[3] = {static_cast<value_type>(0.2), static_cast<value_type>(-0.3),
                                           static_cast<value_type>(0.4)};
    vector3_type const rotation_vector = vector3_type::from_row_major(rotation_values);
    value_type const theta_squared = formal_eskf::linalg::squared_norm(rotation_vector);
    value_type const theta = std::sqrt(theta_squared);

    quaternion_type quaternion;
    formal_eskf::Status const status = try_exp(rotation_vector, tolerance, quaternion);

    matrix3_type const rotation_hat = formal_eskf::so3::hat<Linalg>(rotation_vector);
    matrix3_type const expected = matrix3_type::identity() + rotation_hat * (std::sin(theta) / theta) +
                                  (rotation_hat * rotation_hat) * ((value_type{1} - std::cos(theta)) / theta_squared);
    test.expect(status == formal_eskf::Status::success &&
                    matrix_near(to_rotation_matrix(quaternion), expected, tolerance),
                profile, "R(Exp_q(phi)) matches the Rodrigues closed form");
}

template <typename Linalg>
void test_exp_failure_atomicity(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using quaternion_type = formal_eskf::so3::UnitQuaternion<Linalg>;
    using value_type = typename Linalg::value_type;
    using vector3_type = typename Linalg::template vector_type<3U>;

    quaternion_type unchanged;
    quaternion_type const expected = unchanged;

    value_type const non_finite_values[3] = {std::numeric_limits<value_type>::quiet_NaN(), value_type{0},
                                             value_type{0}};
    vector3_type const non_finite = vector3_type::from_row_major(non_finite_values);
    formal_eskf::Status const non_finite_status = try_exp(non_finite, tolerance, unchanged);
    test.expect(non_finite_status == formal_eskf::Status::non_finite_input && same_coefficients(unchanged, expected),
                profile, "Exp_q rejects non-finite input without changing output");

    value_type const large_values[3] = {std::numeric_limits<value_type>::max(), std::numeric_limits<value_type>::max(),
                                        value_type{0}};
    vector3_type const large = vector3_type::from_row_major(large_values);
    formal_eskf::Status const overflow_status = try_exp(large, tolerance, unchanged);
    test.expect(overflow_status == formal_eskf::Status::non_finite_result && same_coefficients(unchanged, expected),
                profile, "Exp_q reports norm overflow without changing output");

    formal_eskf::Status const invalid_threshold_status = try_exp(vector3_type::zero(), value_type{0}, unchanged);
    test.expect(invalid_threshold_status == formal_eskf::Status::domain_error && same_coefficients(unchanged, expected),
                profile, "Exp_q rejects an invalid quaternion-norm threshold without changing output");
}

template <typename Linalg>
void test_rotation_action(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using quaternion_type = formal_eskf::so3::UnitQuaternion<Linalg>;
    using value_type = typename Linalg::value_type;
    using vector3_type = typename Linalg::template vector_type<3U>;

    value_type const half_sqrt_two = static_cast<value_type>(0.70710678118654752440);
    quaternion_type qz;
    formal_eskf::Status const status = quaternion_type::try_from_coefficients(
        half_sqrt_two, value_type{0}, value_type{0}, half_sqrt_two, tolerance, qz);

    constexpr value_type x_values[3] = {1, 0, 0};
    constexpr value_type y_values[3] = {0, 1, 0};
    vector3_type const x_axis = vector3_type::from_row_major(x_values);
    vector3_type const y_axis = vector3_type::from_row_major(y_values);
    vector3_type const rotated = rotate(qz, x_axis);
    vector3_type const recovered = inverse_rotate(qz, rotated);

    test.expect(status == formal_eskf::Status::success && matrix_near(rotated, y_axis, tolerance), profile,
                "positive z quarter-turn rotates x to y");
    test.expect(matrix_near(recovered, x_axis, tolerance), profile, "inverse rotation recovers the input vector");
    test.expect(near(formal_eskf::linalg::norm(rotated), formal_eskf::linalg::norm(x_axis), tolerance), profile,
                "rotation preserves vector norm");
    test.expect(matrix_near(to_rotation_matrix(qz), to_rotation_matrix(-qz), tolerance), profile,
                "q and -q produce the same rotation matrix");
}

template <typename Linalg>
void test_reference_rotation_matrix(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using quaternion_type = formal_eskf::so3::UnitQuaternion<Linalg>;
    using matrix3_type = typename Linalg::template matrix_type<3U, 3U>;
    using value_type = typename Linalg::value_type;

    quaternion_type quaternion;
    formal_eskf::Status const status = quaternion_type::try_from_coefficients(
        value_type{1}, value_type{2}, value_type{3}, value_type{4}, tolerance, quaternion);

    value_type const expected_values[9] = {
        value_type{-2} / value_type{3}, value_type{2} / value_type{15},  value_type{11} / value_type{15},
        value_type{2} / value_type{3},  value_type{-1} / value_type{3},  value_type{2} / value_type{3},
        value_type{1} / value_type{3},  value_type{14} / value_type{15}, value_type{2} / value_type{15},
    };
    matrix3_type const expected = matrix3_type::from_row_major(expected_values);

    test.expect(status == formal_eskf::Status::success &&
                    matrix_near(to_rotation_matrix(quaternion), expected, tolerance),
                profile, "R(q) matches the q0, q1, q2, q3 form of Sola Equation (115)");
}

template <typename Linalg>
void test_hat(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    using vector3_type = typename Linalg::template vector_type<3U>;

    constexpr value_type left_values[3] = {1, 2, 3};
    constexpr value_type right_values[3] = {-4, 5, -6};
    vector3_type const left = vector3_type::from_row_major(left_values);
    vector3_type const right = vector3_type::from_row_major(right_values);
    auto const skew = formal_eskf::so3::hat<Linalg>(left);

    test.expect(matrix_near(skew * right, formal_eskf::linalg::cross(left, right), tolerance), profile,
                "hat(a)*b equals cross(a,b)");
    test.expect(formal_eskf::linalg::max_abs(formal_eskf::linalg::transpose(skew) + skew) <= tolerance, profile,
                "hat matrix is skew-symmetric");
}

template <typename Linalg>
void run_conformance_tests(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    test_construction<Linalg>(test, profile, tolerance);
    test_multiplication_and_inverse<Linalg>(test, profile, tolerance);
    test_rotation_action<Linalg>(test, profile, tolerance);
    test_reference_rotation_matrix<Linalg>(test, profile, tolerance);
    test_exp_and_log<Linalg>(test, profile, tolerance);
    test_exp_rotation_matrix<Linalg>(test, profile, tolerance);
    test_exp_failure_atomicity<Linalg>(test, profile, tolerance);
    test_hat<Linalg>(test, profile, tolerance);
}

} /* end namespace */

int main()
{
    TestContext test;

    Eigen::internal::set_is_malloc_allowed(false);
    run_conformance_tests<formal_eskf::linalg::EigenBackend<double>>(test, "binary64", 1.0e-12);
    run_conformance_tests<formal_eskf::linalg::EigenBackend<float>>(test, "binary32", 1.0e-5F);

    if (test.failures() != 0)
    {
        std::cerr << test.failures() << " SO(3) test(s) failed\n";
        return 1;
    }

    std::cout << "All SO(3) unit-quaternion conformance tests passed\n";
    return 0;
}
