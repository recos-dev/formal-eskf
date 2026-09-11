/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

/**
 * @file
 * Backend-independent maps between rotation vectors and unit quaternions.
 */

#include <limits>

#include <formal_eskf/linalg/linalg.hpp>
#include <formal_eskf/scalar/math.hpp>
#include <formal_eskf/so3/unit_quaternion.hpp>

namespace formal_eskf::so3
{

namespace detail
{

template <typename Scalar> struct PrincipalQuaternionCoefficients
{
    Scalar q0;
    Scalar q1;
    Scalar q2;
    Scalar q3;
};

template <typename Linalg>
[[nodiscard]] PrincipalQuaternionCoefficients<typename Linalg::value_type>
principal_quaternion_coefficients(UnitQuaternion<Linalg> const & quaternion) noexcept
{
    using value_type = typename Linalg::value_type;
    value_type const sign = quaternion.q0() < value_type{0} ? value_type{-1} : value_type{1};
    return {sign * quaternion.q0(), sign * quaternion.q1(), sign * quaternion.q2(), sign * quaternion.q3()};
}

template <typename Scalar>
[[nodiscard]] Scalar log_taylor_scale(Scalar q0, Scalar q0_squared, Scalar qv_squared_norm) noexcept
{
    return Scalar{2} / q0 * (Scalar{1} - qv_squared_norm / (Scalar{3} * q0_squared));
}

template <typename Scalar> [[nodiscard]] Scalar log_closed_form_scale(Scalar qv_norm, Scalar half_angle) noexcept
{
    return Scalar{2} * half_angle / qv_norm;
}

template <typename Linalg>
[[nodiscard]] typename Linalg::template vector_type<3U>
log_candidate(typename Linalg::template vector_type<3U> const & qv, typename Linalg::value_type scale) noexcept
{
    return qv * scale;
}

} /* end namespace detail */

/**
 * Compute the quaternion exponential of a rotation vector.
 *
 * Implements the capitalized quaternion Exp in Joan Sola, "Quaternion
 * kinematics for the error-state Kalman filter", equations (97), (98), and
 * (101).  The small-angle branch follows the Taylor development in equations
 * (41) and (44), after substituting the half-angle phi/2, and is selected using
 * the scalar format's machine epsilon.  Project coefficients [q0, q1, q2, q3]
 * correspond to Sola's [qw, qx, qy, qz].
 *
 * @see https://arxiv.org/abs/1711.02508
 */
template <typename Linalg>
[[nodiscard]] Status try_exp(typename Linalg::template vector_type<3U> const & rotation_vector,
                             typename Linalg::value_type minimum_quaternion_norm,
                             UnitQuaternion<Linalg> & output) noexcept
{
    using scalar_math_type = typename Linalg::scalar_math_type;
    using value_type = typename Linalg::value_type;

    if (!linalg::all_finite(rotation_vector) || !scalar::is_finite<scalar_math_type>(minimum_quaternion_norm))
    {
        return Status::non_finite_input;
    }
    if (!(minimum_quaternion_norm > value_type{0}))
    {
        return Status::domain_error;
    }

    value_type const theta_squared = linalg::squared_norm(rotation_vector);
    if (!scalar::is_finite<scalar_math_type>(theta_squared))
    {
        return Status::non_finite_result;
    }
    if (theta_squared < value_type{0})
    {
        return Status::domain_error;
    }

    value_type q0;
    value_type vector_scale;
    if (theta_squared <= std::numeric_limits<value_type>::epsilon())
    {
        value_type const theta_fourth = theta_squared * theta_squared;
        q0 = value_type{1} - theta_squared / value_type{8} + theta_fourth / value_type{384};
        vector_scale = value_type{0.5} - theta_squared / value_type{48} + theta_fourth / value_type{3840};
    }
    else
    {
        value_type theta;
        Status const sqrt_status = scalar::try_sqrt<scalar_math_type>(theta_squared, theta);
        if (!succeeded(sqrt_status))
        {
            return sqrt_status;
        }

        scalar::SinCos<value_type> half_angle;
        Status const sin_cos_status = scalar::try_sin_cos<scalar_math_type>(value_type{0.5} * theta, half_angle);
        if (!succeeded(sin_cos_status))
        {
            return sin_cos_status;
        }

        q0 = half_angle.cosine;
        vector_scale = half_angle.sine / theta;
    }

    if (!scalar::is_finite<scalar_math_type>(q0) || !scalar::is_finite<scalar_math_type>(vector_scale))
    {
        return Status::non_finite_result;
    }

    return UnitQuaternion<Linalg>::try_from_coefficients(
        q0, vector_scale * rotation_vector(0U), vector_scale * rotation_vector(1U), vector_scale * rotation_vector(2U),
        minimum_quaternion_norm, output);
}

/**
 * Compute the principal rotation vector of a unit quaternion.
 *
 * Implements the capitalized quaternion Log in Joan Sola, "Quaternion
 * kinematics for the error-state Kalman filter", equations (103)--(106).  The
 * A q0 >= 0 representative is selected first, giving q and -q the same result
 * away from q0 == 0 and a norm no greater than pi.  At q0 == 0, the two
 * opposite axis signs are both valid principal results for the pi rotation.
 * This sign selection is an SO(3) policy applied before Sola's S^3 Log
 * formula.
 *
 * @see https://arxiv.org/abs/1711.02508
 */
template <typename Linalg>
[[nodiscard]] Status try_log(UnitQuaternion<Linalg> const & quaternion,
                             typename Linalg::template vector_type<3U> & output) noexcept
{
    using scalar_math_type = typename Linalg::scalar_math_type;
    using value_type = typename Linalg::value_type;
    using vector3_type = typename Linalg::template vector_type<3U>;

    if (!linalg::all_finite(quaternion.coefficients()))
    {
        return Status::non_finite_input;
    }

    auto const principal = detail::principal_quaternion_coefficients(quaternion);
    value_type const q0 = principal.q0;
    vector3_type qv;
    qv.set(0U, principal.q1);
    qv.set(1U, principal.q2);
    qv.set(2U, principal.q3);

    value_type const qv_squared_norm = linalg::squared_norm(qv);
    if (!scalar::is_finite<scalar_math_type>(qv_squared_norm))
    {
        return Status::non_finite_result;
    }
    if (qv_squared_norm < value_type{0})
    {
        return Status::domain_error;
    }
    if (qv_squared_norm == value_type{0})
    {
        output.set(0U, value_type{0});
        output.set(1U, value_type{0});
        output.set(2U, value_type{0});
        return Status::success;
    }

    value_type scale;
    if (qv_squared_norm <= std::numeric_limits<value_type>::epsilon())
    {
        value_type const q0_squared = q0 * q0;
        if (!(q0_squared > value_type{0}))
        {
            return Status::zero_or_unsafe_divisor;
        }
        scale = detail::log_taylor_scale(q0, q0_squared, qv_squared_norm);
    }
    else
    {
        value_type qv_norm;
        Status const sqrt_status = scalar::try_sqrt<scalar_math_type>(qv_squared_norm, qv_norm);
        if (!succeeded(sqrt_status))
        {
            return sqrt_status;
        }

        value_type half_angle;
        Status const atan2_status = scalar::try_atan2<scalar_math_type>(qv_norm, q0, half_angle);
        if (!succeeded(atan2_status))
        {
            return atan2_status;
        }

        scale = detail::log_closed_form_scale(qv_norm, half_angle);
    }

    if (!scalar::is_finite<scalar_math_type>(scale))
    {
        return Status::non_finite_result;
    }

    vector3_type const candidate = detail::log_candidate<Linalg>(qv, scale);
    if (!linalg::all_finite(candidate))
    {
        return Status::non_finite_result;
    }

    output.set(0U, candidate(0U));
    output.set(1U, candidate(1U));
    output.set(2U, candidate(2U));
    return Status::success;
}

} /* end namespace formal_eskf::so3 */
