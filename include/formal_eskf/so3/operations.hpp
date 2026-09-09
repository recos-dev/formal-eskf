#pragma once

/**
 * @file
 * Backend-independent SO(3) and unit-quaternion operations.
 */

#include <formal_eskf/linalg/linalg.hpp>
#include <formal_eskf/scalar/math.hpp>
#include <formal_eskf/so3/unit_quaternion.hpp>

namespace formal_eskf::so3
{

template <typename Linalg>
[[nodiscard]] bool same_coefficients(UnitQuaternion<Linalg> const & q_a, UnitQuaternion<Linalg> const & q_b) noexcept
{
    return q_a.q0() == q_b.q0() && q_a.q1() == q_b.q1() && q_a.q2() == q_b.q2() && q_a.q3() == q_b.q3();
}

/**
 * Compare rotations while accounting for q and -q equivalence.
 *
 * Tolerance is the maximum absolute coefficient residual for either sign.
 */
template <typename Linalg>
[[nodiscard]] bool same_rotation(UnitQuaternion<Linalg> const & q_a, UnitQuaternion<Linalg> const & q_b,
                                 typename Linalg::value_type tolerance) noexcept
{
    using scalar_math_type = typename Linalg::scalar_math_type;
    using value_type = typename Linalg::value_type;

    if (!scalar::is_finite<scalar_math_type>(tolerance) || tolerance < value_type{0})
    {
        return false;
    }

    auto const q_a_coefficients = q_a.coefficients();
    auto const q_b_coefficients = q_b.coefficients();
    return linalg::max_abs(q_a_coefficients - q_b_coefficients) <= tolerance ||
           linalg::max_abs(q_a_coefficients + q_b_coefficients) <= tolerance;
}

/**
 * Convert q = [q0, q1, q2, q3] to R(q) using Equation (4) of the
 * project paper.  q0 is the scalar coefficient.
 */
template <typename Linalg>
[[nodiscard]] typename Linalg::template matrix_type<3U, 3U>
to_rotation_matrix(UnitQuaternion<Linalg> const & quaternion) noexcept
{
    using matrix3_type = typename Linalg::template matrix_type<3U, 3U>;
    using value_type = typename Linalg::value_type;

    value_type const q0 = quaternion.q0();
    value_type const q1 = quaternion.q1();
    value_type const q2 = quaternion.q2();
    value_type const q3 = quaternion.q3();

    value_type const q0q0 = q0 * q0;
    value_type const q0q1 = q0 * q1;
    value_type const q0q2 = q0 * q2;
    value_type const q0q3 = q0 * q3;
    value_type const q1q1 = q1 * q1;
    value_type const q1q2 = q1 * q2;
    value_type const q1q3 = q1 * q3;
    value_type const q2q2 = q2 * q2;
    value_type const q2q3 = q2 * q3;
    value_type const q3q3 = q3 * q3;

    matrix3_type result;
    result(0U, 0U) = q0q0 + q1q1 - q2q2 - q3q3;
    result(0U, 1U) = value_type{2} * (q1q2 - q0q3);
    result(0U, 2U) = value_type{2} * (q1q3 + q0q2);
    result(1U, 0U) = value_type{2} * (q1q2 + q0q3);
    result(1U, 1U) = q0q0 - q1q1 + q2q2 - q3q3;
    result(1U, 2U) = value_type{2} * (q2q3 - q0q1);
    result(2U, 0U) = value_type{2} * (q1q3 - q0q2);
    result(2U, 1U) = value_type{2} * (q2q3 + q0q1);
    result(2U, 2U) = q0q0 - q1q1 - q2q2 + q3q3;
    return result;
}

template <typename Linalg>
[[nodiscard]] typename Linalg::template vector_type<3U>
rotate(UnitQuaternion<Linalg> const & quaternion, typename Linalg::template vector_type<3U> const & vector) noexcept
{
    return to_rotation_matrix(quaternion) * vector;
}

template <typename Linalg>
[[nodiscard]] typename Linalg::template vector_type<3U>
inverse_rotate(UnitQuaternion<Linalg> const & quaternion,
               typename Linalg::template vector_type<3U> const & vector) noexcept
{
    return linalg::transpose(to_rotation_matrix(quaternion)) * vector;
}

template <typename Linalg>
[[nodiscard]] typename Linalg::template matrix_type<3U, 3U>
hat(typename Linalg::template vector_type<3U> const & vector) noexcept
{
    using matrix3_type = typename Linalg::template matrix_type<3U, 3U>;
    using value_type = typename Linalg::value_type;

    matrix3_type result;
    result(0U, 0U) = value_type{0};
    result(0U, 1U) = -vector(2U);
    result(0U, 2U) = vector(1U);
    result(1U, 0U) = vector(2U);
    result(1U, 1U) = value_type{0};
    result(1U, 2U) = -vector(0U);
    result(2U, 0U) = -vector(1U);
    result(2U, 1U) = vector(0U);
    result(2U, 2U) = value_type{0};
    return result;
}

} /* end namespace formal_eskf::so3 */
