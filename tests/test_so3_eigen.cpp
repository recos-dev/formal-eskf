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
void test_paper_rotation_matrix(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
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
                profile, "R(q) matches the q0, q1, q2, q3 form of paper Equation (4)");
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
    test_paper_rotation_matrix<Linalg>(test, profile, tolerance);
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
