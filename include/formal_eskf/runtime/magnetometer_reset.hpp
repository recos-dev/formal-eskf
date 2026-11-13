/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <cstddef>
#include <limits>

#include <formal_eskf/eskf/configuration/ahrs.hpp>
#include <formal_eskf/eskf/configuration/ins.hpp>
#include <formal_eskf/runtime/magnetic_field.hpp>
#include <formal_eskf/so3/rotation_vector.hpp>

namespace formal_eskf::runtime
{

namespace detail
{

/** Reset navigation yaw uncertainty and re-express tilt/cross terms in the new local frame. */
template <std::size_t AttitudeOffset, typename Linalg, std::size_t Size>
[[nodiscard]] Status reset_heading_covariance(linalg::Matrix<Linalg, Size, Size> const & covariance,
                                              linalg::Matrix<Linalg, 3U, 3U> const & R_before,
                                              linalg::Matrix<Linalg, 3U, 3U> const & R_after,
                                              typename Linalg::value_type yaw_variance,
                                              linalg::Matrix<Linalg, Size, Size> & output)
{
    static_assert(AttitudeOffset + 3U <= Size);
    using value_type = typename Linalg::value_type;
    using vector_type = linalg::Vector<Linalg, 3U>;
    using matrix_type = linalg::Matrix<Linalg, 3U, 3U>;
    // Reset the navigation-frame yaw variance, then express the retained tilt
    // and cross-covariances in the new body-local attitude coordinates.
    auto const R_after_transpose = linalg::transpose(R_after);
    auto const down_n = vector_type::from_row_major({value_type{0}, value_type{0}, value_type{1}});
    auto const down_b = R_after_transpose * down_n;
    auto const tilt_n = matrix_type::identity() - down_n * linalg::transpose(down_n);
    auto const A = R_after_transpose * tilt_n * R_before;
    auto const cross = A * covariance.template block<AttitudeOffset, 0U, 3U, Size>();
    constexpr value_type minimum_std = static_cast<value_type>(0.01);
    value_type const variance = yaw_variance < minimum_std * minimum_std ? minimum_std * minimum_std : yaw_variance;
    auto attitude = cross.template block<0U, AttitudeOffset, 3U, 3U>() * linalg::transpose(A) +
                    (down_b * linalg::transpose(down_b)) * variance;
    if (!linalg::all_finite(cross) || !linalg::all_finite(attitude))
    {
        return Status::non_finite_result;
    }
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        if (attitude(row, row) < value_type{0})
        {
            return Status::not_positive_definite;
        }
        for (std::size_t column = row; column < 3U; ++column)
        {
            // Divide before adding: keep the established overflow/rounding behavior.
            value_type const value = attitude(row, column) / value_type{2} + attitude(column, row) / value_type{2};
            attitude(row, column) = value;
            attitude(column, row) = value;
        }
    }
    output = covariance;
    output.template set_block<AttitudeOffset, 0U>(cross);
    output.template set_block<0U, AttitudeOffset>(linalg::transpose(cross));
    output.template set_block<AttitudeOffset, AttitudeOffset>(attitude);
    return Status::success;
}

/**
 * Compute only quaternion/covariance candidates. Outputs are disjoint scratch
 * storage and may be partial on failure; the INS/AHRS entry points publish them
 * together only after success.
 */
template <std::size_t AttitudeOffset, typename Linalg, std::size_t Size>
[[nodiscard]] Status compute_magnetic_heading_reset(
    so3::UnitQuaternion<Linalg> const & q_nb, linalg::Matrix<Linalg, Size, Size> const & covariance,
    linalg::Vector<Linalg, 3U> const & m_b, linalg::Vector<Linalg, 3U> const & m_n,
    typename Linalg::value_type yaw_variance, typename Linalg::value_type minimum_horizontal_field,
    typename Linalg::value_type minimum_quaternion_norm, so3::UnitQuaternion<Linalg> & attitude_output,
    linalg::Matrix<Linalg, Size, Size> & covariance_output, so3::UnitQuaternion<Linalg> & delta_output)
{
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    using vector_type = linalg::Vector<Linalg, 3U>;
    if (!linalg::all_finite(q_nb.coefficients()) || !linalg::all_finite(covariance) || !linalg::all_finite(m_b) ||
        !linalg::all_finite(m_n) || !scalar::is_finite<math_type>(yaw_variance) ||
        !scalar::is_finite<math_type>(minimum_horizontal_field) ||
        !scalar::is_finite<math_type>(minimum_quaternion_norm))
    {
        return Status::non_finite_input;
    }
    if (yaw_variance < value_type{0} || minimum_horizontal_field < std::numeric_limits<value_type>::min() ||
        !(minimum_quaternion_norm > value_type{0}) || minimum_quaternion_norm > value_type{1} ||
        !linalg::is_symmetric(covariance, value_type{0}))
    {
        return Status::domain_error;
    }
    for (std::size_t index = 0U; index < Size; ++index)
    {
        if (covariance(index, index) < value_type{0})
        {
            return Status::domain_error;
        }
    }

    auto const R_before = so3::to_rotation_matrix(q_nb);
    auto const measured_n = R_before * m_b;
    if (!linalg::all_finite(measured_n))
    {
        return Status::non_finite_result;
    }
    MagneticFieldGeometry<value_type> measured;
    MagneticFieldGeometry<value_type> reference;
    Status status = magnetic_field_geometry(measured_n, measured);
    if (!succeeded(status))
    {
        return status;
    }
    status = magnetic_field_geometry(m_n, reference);
    if (!succeeded(status))
    {
        return status;
    }
    if (measured.horizontal < minimum_horizontal_field || reference.horizontal < minimum_horizontal_field)
    {
        return Status::domain_error;
    }
    value_type measured_heading{};
    value_type reference_heading{};
    status = scalar::atan2<math_type>(measured_n(1U), measured_n(0U), measured_heading);
    if (!succeeded(status))
    {
        return status;
    }
    status = scalar::atan2<math_type>(m_n(1U), m_n(0U), reference_heading);
    if (!succeeded(status))
    {
        return status;
    }
    value_type heading_change = reference_heading - measured_heading;
    auto const pi = scalar::pi<math_type>();
    if (heading_change > pi)
    {
        heading_change -= value_type{2} * pi;
    }
    else if (heading_change < -pi)
    {
        heading_change += value_type{2} * pi;
    }
    status = so3::exp<Linalg>(vector_type::from_row_major({value_type{0}, value_type{0}, heading_change}),
                              minimum_quaternion_norm, delta_output);
    if (!succeeded(status))
    {
        return status;
    }
    status = so3::compose_normalized(delta_output, q_nb, minimum_quaternion_norm, attitude_output);
    if (!succeeded(status))
    {
        return status;
    }

    return reset_heading_covariance<AttitudeOffset>(covariance, R_before, so3::to_rotation_matrix(attitude_output),
                                                    yaw_variance, covariance_output);
}

} /* end namespace detail */

/**
 * Align the horizontal component of R(q_nb)*m_b with a trusted fixed NED m_n.
 * Fields are calibrated microtesla; yaw_variance is rad^2, floored at 0.01^2.
 * Left-multiply by a navigation-Down rotation: tilt is preserved, including at
 * Euler singularities. delta_output is this navigation-frame rotation.
 *
 * For local attitude errors, A = R_after^T*(I-e_D*e_D^T)*R_before.
 * Apply A to attitude covariance and cross-covariances, then add yaw variance
 * along R_after^T*e_D. Non-attitude state/P blocks remain unchanged.
 *
 * Prior quaternion must be unit and P symmetric PSD. Checks cover finite inputs,
 * exact symmetry, nonnegative diagonals/variance, normal positive horizontal
 * threshold, 0 < minimum_quaternion_norm <= 1 and sufficient horizontal field.
 * PSD and prior norm are caller premises. Field health, tilt alignment, freshness
 * and reset authorization are caller responsibilities; this is not fusion.
 * All outputs remain unchanged on failure. State/P may independently alias their
 * entire priors; all other storage, including delta_output, must be disjoint.
 */
template <typename Linalg>
[[nodiscard]] Status reset_magnetic_heading(
    configuration::Ahrs::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 3U, 3U> const & covariance,
    linalg::Vector<Linalg, 3U> const & m_b, linalg::Vector<Linalg, 3U> const & m_n,
    typename Linalg::value_type yaw_variance, typename Linalg::value_type minimum_horizontal_field,
    typename Linalg::value_type minimum_quaternion_norm, configuration::Ahrs::NominalState<Linalg> & state_output,
    linalg::Matrix<Linalg, 3U, 3U> & covariance_output, so3::UnitQuaternion<Linalg> & delta_output)
{
    auto candidate = state;
    linalg::Matrix<Linalg, 3U, 3U> P;
    so3::UnitQuaternion<Linalg> delta;
    auto const status = detail::compute_magnetic_heading_reset<0U>(state.q_nb, covariance, m_b, m_n, yaw_variance,
                                                                   minimum_horizontal_field, minimum_quaternion_norm,
                                                                   candidate.q_nb, P, delta);
    if (!succeeded(status))
    {
        return status;
    }
    state_output = candidate;
    covariance_output = P;
    delta_output = delta;
    return Status::success;
}

/** INS overload; position, velocity and bias values are retained exactly. */
template <typename Linalg>
[[nodiscard]] Status reset_magnetic_heading(
    configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15U, 15U> const & covariance,
    linalg::Vector<Linalg, 3U> const & m_b, linalg::Vector<Linalg, 3U> const & m_n,
    typename Linalg::value_type yaw_variance, typename Linalg::value_type minimum_horizontal_field,
    typename Linalg::value_type minimum_quaternion_norm, configuration::Ins::NominalState<Linalg> & state_output,
    linalg::Matrix<Linalg, 15U, 15U> & covariance_output, so3::UnitQuaternion<Linalg> & delta_output)
{
    if (!linalg::all_finite(state.p_n) || !linalg::all_finite(state.v_n) || !linalg::all_finite(state.b_a) ||
        !linalg::all_finite(state.b_g))
    {
        return Status::non_finite_input;
    }
    auto candidate = state;
    linalg::Matrix<Linalg, 15U, 15U> P;
    so3::UnitQuaternion<Linalg> delta;
    auto const status = detail::compute_magnetic_heading_reset<6U>(state.q_nb, covariance, m_b, m_n, yaw_variance,
                                                                   minimum_horizontal_field, minimum_quaternion_norm,
                                                                   candidate.q_nb, P, delta);
    if (!succeeded(status))
    {
        return status;
    }
    state_output = candidate;
    covariance_output = P;
    delta_output = delta;
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
