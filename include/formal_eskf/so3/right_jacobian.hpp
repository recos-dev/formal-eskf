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
 * Checked, backend-independent SO(3) right Jacobian.
 */

#include <formal_eskf/so3/operations.hpp>

namespace formal_eskf::so3
{

/**
 * Evaluate Joan Sola, "Quaternion kinematics for the error-state Kalman
 * filter", equation (183):
 *   J_r(phi) = I - (1-cos(theta))/theta^2 * hat(phi)
 *                + (theta-sin(theta))/theta^3 * hat(phi)^2,
 * where theta = norm(phi). This is J_r, not J_r^{-1} or a rotation matrix.
 *
 * At zero J_r = I. Near zero, cubic polynomials in theta^2 evaluate the
 * removable singularities. Away from zero a unit-axis/half-angle form avoids
 * cancellation in 1-cos(theta), overflow in theta^3 and a matrix square.
 * The closed form is the exact-real Jacobian; finite-precision evaluation
 * and the local Taylor branch do not claim exact floating-point results.
 *
 * Requires finite phi and a representable squared norm. Failure leaves
 * output unchanged. No inversion, factorization or dynamic allocation is used.
 * This mathematical operation is independent of ESKF approximation macros.
 *
 * @see https://arxiv.org/abs/1711.02508
 */
template <typename Linalg>
[[nodiscard]] Status try_right_jacobian(linalg::Matrix<Linalg, 3U, 1U> const & phi,
                                        linalg::Matrix<Linalg, 3U, 3U> & output) noexcept
{
    using value_type = typename Linalg::value_type;
    using scalar_math_type = typename Linalg::scalar_math_type;

    if (!linalg::all_finite(phi))
    {
        return Status::non_finite_input;
    }
    value_type const theta_squared = linalg::squared_norm(phi);
    if (!scalar::is_finite<scalar_math_type>(theta_squared))
    {
        return Status::non_finite_result;
    }
    if (theta_squared < value_type{0})
    {
        return Status::domain_error;
    }

    auto axis = phi;
    value_type identity_scale;
    value_type outer_scale;
    value_type skew_scale;
    // theta <= 1/16. The first omitted exact-real coefficient terms are
    // theta^8/10! < 6.42e-17 and theta^8/11! < 5.84e-18, respectively.
    // This conservative binary64 cutoff also works for binary32. It is
    // specific to these coefficients, not copied from quaternion Exp.
    constexpr value_type taylor_limit_squared = value_type{0.00390625};
    if (theta_squared <= taylor_limit_squared)
    {
        skew_scale =
            value_type{0.5} -
            theta_squared * (value_type{1} / value_type{24} -
                             theta_squared * (value_type{1} / value_type{720} - theta_squared / value_type{40320}));
        outer_scale =
            value_type{1} / value_type{6} -
            theta_squared * (value_type{1} / value_type{120} -
                             theta_squared * (value_type{1} / value_type{5040} - theta_squared / value_type{362880}));
        // hat(phi)^2 = phi*phi^T - theta^2*I.
        identity_scale = value_type{1} - theta_squared * outer_scale;
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
        Status const sin_cos_status = scalar::try_sin_cos<scalar_math_type>(theta * value_type{0.5}, half_angle);
        if (!succeeded(sin_cos_status))
        {
            return sin_cos_status;
        }
        axis = phi / theta;
        // With u = phi/theta:
        // J_r = sinc(theta)*I + (1-sinc(theta))*u*u^T
        //       - 2*sin(theta/2)^2/theta * hat(u).
        identity_scale = (value_type{2} * half_angle.sine * half_angle.cosine) / theta;
        outer_scale = value_type{1} - identity_scale;
        skew_scale = (value_type{2} * half_angle.sine * half_angle.sine) / theta;
    }

    auto candidate = (axis * linalg::transpose(axis)) * outer_scale - hat<Linalg>(axis) * skew_scale;
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        candidate.set(index, index, candidate(index, index) + identity_scale);
    }
    if (!linalg::all_finite(candidate))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

} /* end namespace formal_eskf::so3 */
