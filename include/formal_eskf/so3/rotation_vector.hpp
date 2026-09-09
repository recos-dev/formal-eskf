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
 * q0 >= 0 representative is selected first so that q and -q have the same
 * project-defined principal result with norm no greater than pi; that sign
 * selection is an SO(3) policy applied before Sola's S^3 Log formula.
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

    value_type const sign = quaternion.q0() < value_type{0} ? value_type{-1} : value_type{1};
    value_type const q0 = sign * quaternion.q0();
    vector3_type qv;
    qv(0U) = sign * quaternion.q1();
    qv(1U) = sign * quaternion.q2();
    qv(2U) = sign * quaternion.q3();

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
        output = vector3_type::zero();
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
        scale = value_type{2} / q0 * (value_type{1} - qv_squared_norm / (value_type{3} * q0_squared));
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

        scale = value_type{2} * half_angle / qv_norm;
    }

    if (!scalar::is_finite<scalar_math_type>(scale))
    {
        return Status::non_finite_result;
    }

    vector3_type const candidate = qv * scale;
    if (!linalg::all_finite(candidate))
    {
        return Status::non_finite_result;
    }

    output = candidate;
    return Status::success;
}

} /* end namespace formal_eskf::so3 */
