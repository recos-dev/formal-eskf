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
#include <formal_eskf/eskf/magnetometer.hpp>
#include <formal_eskf/linalg/operations.hpp>
#include <formal_eskf/runtime/magnetic_field.hpp>
#include <formal_eskf/runtime/measurement_fusion.hpp>
#include <formal_eskf/so3/rotation_vector.hpp>
#include <formal_eskf/status.hpp>

namespace formal_eskf::runtime
{

/** Heading consistency history, separate from field plausibility and innovation gating. */
template <typename Linalg> class MagneticHeading
{
public:
    using value_type = typename Linalg::value_type;
    using vector_type = linalg::Vector<Linalg, 3U>;
    using horizontal_type = linalg::Vector<Linalg, 2U>;

    struct Parameters
    {
        value_type heading_noise{};            // Positive radians, at most pi; strict consistency threshold.
        value_type acceleration_limit{};       // Nonnegative m/s^2; strict observability threshold.
        value_type minimum_horizontal_field{}; // Positive normal microtesla.
    };

    struct State
    {
        value_type innovation{}; // Wrapped predicted-minus-measured heading, radians.
        value_type filtered_innovation{};
        value_type horizontal_acceleration{}; // Norm of the filtered N/E acceleration, m/s^2.
        bool innovation_valid = false;
        bool observable = false; // Current aiding + maneuver condition, not a mathematical proof.
        bool consistent = false; // Retained through low excitation until inconsistency is observed.
    };

    State const & state() const { return m_state; }
    void reset() { *this = MagneticHeading{}; }
    /** Missing IMU intervals provide no maneuver evidence. Preserve the heading latch. */
    void reset_acceleration()
    {
        m_acceleration = horizontal_type{};
        m_state.horizontal_acceleration = value_type{0};
        m_state.observable = false;
    }
    /** Call only after an actual magnetic heading alignment/reset succeeds. */
    void reset_heading()
    {
        m_state.innovation = value_type{0};
        m_state.filtered_innovation = value_type{0};
        m_state.innovation_valid = false;
        m_state.consistent = true;
    }

    /** Bias-corrected specific force rotated into NED at prediction; only N/E are filtered. */
    [[nodiscard]] Status update_acceleration(vector_type const & acceleration_n, value_type dt);

    /**
     * Unit q_nb, calibrated m_b and trustworthy m_n at one horizon. Update once
     * per consumed magnetic sample, even while navigation fusion is inhibited.
     * No learned magnetic bias is subtracted. Low horizontal field clears the
     * latch without inventing a heading. Unaligned heading clears its history.
     * Invalid inputs/arithmetic preserve history; callers must handle failure.
     */
    [[nodiscard]] Status update(so3::UnitQuaternion<Linalg> const & q_nb, vector_type const & m_b,
                                vector_type const & m_n, Parameters const & parameters, bool heading_aligned,
                                bool horizontal_aiding);

private:
    [[nodiscard]] static Status validate_inputs(so3::UnitQuaternion<Linalg> const & q_nb, vector_type const & m_b,
                                                vector_type const & m_n, Parameters const & parameters);

    horizontal_type m_acceleration{};
    State m_state{};
}; /* end class MagneticHeading */

template <typename Linalg>
Status MagneticHeading<Linalg>::update_acceleration(vector_type const & acceleration_n, value_type dt)
{
    using math_type = typename Linalg::scalar_math_type;
    if (!linalg::all_finite(acceleration_n) || !scalar::is_finite<math_type>(dt))
    {
        return Status::non_finite_input;
    }
    if (dt < std::numeric_limits<value_type>::min())
    {
        return Status::domain_error;
    }
    // A one-second vector LPF: opposing accelerations cancel before taking the norm.
    auto const alpha = dt / (value_type{1} + dt);
    auto const candidate = m_acceleration + (acceleration_n.template segment<0U, 2U>() - m_acceleration) * alpha;
    auto const squared_norm = linalg::dot(candidate, candidate);
    if (!linalg::all_finite(candidate) || !scalar::is_finite<math_type>(squared_norm))
    {
        return Status::non_finite_result;
    }
    value_type magnitude{};
    auto const status = scalar::sqrt<math_type>(squared_norm, magnitude);
    if (succeeded(status))
    {
        m_acceleration = candidate;
        m_state.horizontal_acceleration = magnitude;
        m_state.observable = false; // Recomputed with aiding on the next magnetic sample.
    }
    return status;
}

template <typename Linalg>
Status MagneticHeading<Linalg>::update(so3::UnitQuaternion<Linalg> const & q_nb, vector_type const & m_b,
                                       vector_type const & m_n, Parameters const & parameters, bool heading_aligned,
                                       bool horizontal_aiding)
{
    using math_type = typename Linalg::scalar_math_type;
    auto status = validate_inputs(q_nb, m_b, m_n, parameters);
    if (!succeeded(status))
    {
        return status;
    }
    auto candidate = m_state;
    candidate.observable = horizontal_aiding && m_state.horizontal_acceleration > parameters.acceleration_limit;
    candidate.innovation_valid = false;
    candidate.innovation = value_type{0};
    if (!heading_aligned)
    {
        candidate.filtered_innovation = value_type{0};
        candidate.consistent = false;
        m_state = candidate;
        return Status::success;
    }
    auto const measured_n = so3::rotate(q_nb, m_b);
    auto const measured_horizontal = linalg::norm(measured_n.template segment<0U, 2U>());
    auto const reference_horizontal = linalg::norm(m_n.template segment<0U, 2U>());
    if (!linalg::all_finite(measured_n) || !scalar::is_finite<math_type>(measured_horizontal) ||
        !scalar::is_finite<math_type>(reference_horizontal))
    {
        return Status::non_finite_result;
    }
    if (measured_horizontal < parameters.minimum_horizontal_field ||
        reference_horizontal < parameters.minimum_horizontal_field)
    {
        candidate.consistent = false;
        m_state = candidate;
        return Status::success;
    }
    value_type innovation{};
    status = detail::magnetic_heading_error(measured_n, m_n, innovation);
    if (!succeeded(status))
    {
        return status;
    }
    // Nominal 10 ms / 90 ms LPF, applied once per magnetic sample, not per IMU tick.
    constexpr auto interval = static_cast<value_type>(0.01);
    constexpr auto time_constant = static_cast<value_type>(0.09);
    candidate.innovation = innovation;
    candidate.innovation_valid = true;
    candidate.filtered_innovation +=
        (innovation - candidate.filtered_innovation) * (interval / (time_constant + interval));
    if (scalar::absolute<math_type>(candidate.filtered_innovation) >= parameters.heading_noise)
    {
        candidate.consistent = false;
    }
    else if (candidate.observable)
    {
        candidate.consistent = true;
    }
    m_state = candidate;
    return Status::success;
}

template <typename Linalg>
Status MagneticHeading<Linalg>::validate_inputs(so3::UnitQuaternion<Linalg> const & q_nb, vector_type const & m_b,
                                                vector_type const & m_n, Parameters const & parameters)
{
    using math_type = typename Linalg::scalar_math_type;
    if (!linalg::all_finite(q_nb.coefficients()) || !linalg::all_finite(m_b) || !linalg::all_finite(m_n) ||
        !scalar::is_finite<math_type>(parameters.heading_noise) ||
        !scalar::is_finite<math_type>(parameters.acceleration_limit) ||
        !scalar::is_finite<math_type>(parameters.minimum_horizontal_field))
    {
        return Status::non_finite_input;
    }
    auto const pi = scalar::pi<math_type>();
    if (!(parameters.heading_noise > value_type{0}) || parameters.heading_noise > pi ||
        parameters.acceleration_limit < value_type{0} ||
        parameters.minimum_horizontal_field < std::numeric_limits<value_type>::min())
    {
        return Status::domain_error;
    }
    return Status::success;
}

/** Field diagnostics and innovation diagnostics have separate validity flags. */
template <typename Linalg> struct MagnetometerFusionResult
{
    MagneticFieldCheck<typename Linalg::value_type> field{};
    FusionResult<Linalg, 3U> fusion{}; // The overall outcome, including field rejection/failure.
};

namespace detail
{

/** Qualify the field before consuming covariance/noise; prepare the common residual. */
template <typename Linalg>
[[nodiscard]] MagnetometerFusionResult<Linalg>
check_magnetic_observation(so3::UnitQuaternion<Linalg> const & q_nb, linalg::Vector<Linalg, 3U> const & m_b,
                           linalg::Vector<Linalg, 3U> const & m_n, linalg::Vector<Linalg, 3U> const * field_reference_n,
                           MagneticFieldParameters<typename Linalg::value_type> const & field_parameters,
                           bool heading_observable, MagneticFieldChecks const & field_checks,
                           linalg::Vector<Linalg, 3U> & residual_output)
{
    MagnetometerFusionResult<Linalg> result;
    result.field =
        check_magnetic_field(q_nb, m_b, m_n, field_reference_n, field_parameters, heading_observable, field_checks);
    result.fusion.status = result.field.status;
    if (!succeeded(result.field.status))
    {
        return result;
    }
    if (!result.field.accepted)
    {
        result.fusion.decision = FusionDecision::rejected;
        return result;
    }
    auto const h_m_b = so3::inverse_rotate(q_nb, m_n);
    auto const r = m_b - h_m_b;
    if (!linalg::all_finite(h_m_b) || !linalg::all_finite(r))
    {
        result.fusion.status = Status::non_finite_result;
        return result;
    }
    residual_output = r;
    return result;
}

} /* end namespace detail */

/**
 * Check field plausibility, gate all three residual components as one group,
 * then call the existing magnetic correction using this same prior and full V.
 * Only result.fusion.decision == fused publishes state/P. Field rejection leaves
 * innovation diagnostics invalid; a later correction failure retains valid gate
 * diagnostics. Each output may alias its respective whole prior, as in the core.
 * Rejection does not certify unused inputs: covariance/solver checks are deferred
 * until their stage. Unit attitude, PSD P, SPD V and calibration/time alignment
 * remain caller premises. Heading observability follows check_magnetic_field.
 * field_reference_n is borrowed for health checks only (nullptr means absent);
 * m_n owns the innovation and correction, even when no health reference exists.
 * No lifecycle, recovery timer, yaw reset, magnetic states or implicit tuning.
 */
template <typename Linalg>
[[nodiscard]] MagnetometerFusionResult<Linalg>
fuse_magnetometer(configuration::Ahrs::NominalState<Linalg> const & state,
                  linalg::Matrix<Linalg, 3U, 3U> const & covariance, linalg::Vector<Linalg, 3U> const & m_b,
                  linalg::Vector<Linalg, 3U> const & m_n, linalg::Vector<Linalg, 3U> const * field_reference_n,
                  linalg::Matrix<Linalg, 3U, 3U> const & V,
                  MagneticFieldParameters<typename Linalg::value_type> const & field_parameters,
                  bool heading_observable, typename Linalg::value_type minimum_quaternion_norm,
                  typename Linalg::value_type gate_sigma, configuration::Ahrs::NominalState<Linalg> & state_output,
                  linalg::Matrix<Linalg, 3U, 3U> & covariance_output, MagneticFieldChecks const & field_checks = {})
{
    if (!detail::finite_fusion_state(state))
    {
        MagnetometerFusionResult<Linalg> result;
        result.fusion.status = Status::non_finite_input;
        return result;
    }
    linalg::Vector<Linalg, 3U> r;
    auto result = detail::check_magnetic_observation(state.q_nb, m_b, m_n, field_reference_n, field_parameters,
                                                     heading_observable, field_checks, r);
    if (!succeeded(result.fusion.status) || !result.field.accepted)
    {
        return result;
    }
    auto const H = magnetometer_jacobian(state, m_n);
    if (!linalg::all_finite(H))
    {
        result.fusion.status = Status::non_finite_result;
        return result;
    }
    auto const check = check_measurement(state, covariance, r, H, V, minimum_quaternion_norm, gate_sigma);
    result.fusion = FusionResult<Linalg, 3U>{check};
    if (!check.passed())
    {
        return result;
    }
    // Magnetic correction owns attitude projection and the same-gain Joseph update.
    result.fusion.status =
        correct_magnetometer(state, covariance, m_b, m_n, V, minimum_quaternion_norm, state_output, covariance_output);
    if (succeeded(result.fusion.status))
    {
        result.fusion.decision = FusionDecision::fused;
    }
    return result;
}

/** INS correction applies the per-axis bias permissions; neither gate uses them. */
template <typename Linalg>
[[nodiscard]] MagnetometerFusionResult<Linalg> fuse_magnetometer(
    configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15U, 15U> const & covariance,
    linalg::Vector<Linalg, 3U> const & m_b, linalg::Vector<Linalg, 3U> const & m_n,
    linalg::Vector<Linalg, 3U> const * field_reference_n, linalg::Matrix<Linalg, 3U, 3U> const & V,
    MagneticFieldParameters<typename Linalg::value_type> const & field_parameters, bool heading_observable,
    typename Linalg::value_type minimum_quaternion_norm, typename Linalg::value_type gate_sigma,
    configuration::Ins::NominalState<Linalg> & state_output, linalg::Matrix<Linalg, 15U, 15U> & covariance_output,
    configuration::Ins::BiasUpdate const & bias_update = {}, MagneticFieldChecks const & field_checks = {})
{
    if (!detail::finite_fusion_state(state))
    {
        MagnetometerFusionResult<Linalg> result;
        result.fusion.status = Status::non_finite_input;
        return result;
    }
    linalg::Vector<Linalg, 3U> r;
    auto result = detail::check_magnetic_observation(state.q_nb, m_b, m_n, field_reference_n, field_parameters,
                                                     heading_observable, field_checks, r);
    if (!succeeded(result.fusion.status) || !result.field.accepted)
    {
        return result;
    }
    auto const H = magnetometer_jacobian(state, m_n);
    if (!linalg::all_finite(H))
    {
        result.fusion.status = Status::non_finite_result;
        return result;
    }
    auto const check = check_measurement(state, covariance, r, H, V, minimum_quaternion_norm, gate_sigma);
    result.fusion = FusionResult<Linalg, 3U>{check};
    if (!check.passed())
    {
        return result;
    }
    // Magnetic correction owns attitude projection and the same-gain Joseph update.
    result.fusion.status = correct_magnetometer(state, covariance, m_b, m_n, V, minimum_quaternion_norm, state_output,
                                                covariance_output, bias_update);
    if (succeeded(result.fusion.status))
    {
        result.fusion.decision = FusionDecision::fused;
    }
    return result;
}

/**
 * Body-field LPF for heading reset, not normal magnetic correction.
 * Fixed alpha uses a nominal 10 ms interval and 90 ms time constant.
 * Update once per consumed magnetic sample; source/calibration changes reseed
 * from that sample. Reference-only changes retain measurement history.
 */
template <typename Linalg> class MagnetometerFilter
{
public:
    using value_type = typename Linalg::value_type;
    using vector_type = linalg::Vector<Linalg, 3U>;

    bool initialized() const { return m_initialized; }
    vector_type const & field_b() const { return m_field_b; }
    void reset() { *this = MagnetometerFilter{}; }

    /** Invalid input/arithmetic preserves history, including a requested reseed. */
    [[nodiscard]] Status update(vector_type const & sample, bool reseed = false);

private:
    vector_type m_field_b{};
    bool m_initialized = false;
}; /* end class MagnetometerFilter */

template <typename Linalg> Status MagnetometerFilter<Linalg>::update(vector_type const & sample, bool reseed)
{
    if (!linalg::all_finite(sample))
    {
        return Status::non_finite_input;
    }
    constexpr auto interval = static_cast<value_type>(0.01);
    constexpr auto time_constant = static_cast<value_type>(0.09);
    constexpr auto alpha = interval / (time_constant + interval);
    auto const candidate = !m_initialized || reseed ? sample : m_field_b + (sample - m_field_b) * alpha;
    if (!linalg::all_finite(candidate))
    {
        return Status::non_finite_result;
    }
    m_field_b = candidate;
    m_initialized = true;
    return Status::success;
}

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
