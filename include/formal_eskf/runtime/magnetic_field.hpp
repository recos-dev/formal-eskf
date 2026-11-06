/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <limits>

#include <formal_eskf/so3/operations.hpp>

namespace formal_eskf::runtime
{

/** Per-call field-check selection and availability of an independent reference. */
struct MagneticFieldChecks
{
    bool strength = true;
    bool inclination = true;
    bool require_reference = false;
    bool reference_available = true; // A measured seed alone does not establish this.
};

template <typename Scalar> struct MagneticFieldParameters
{
    Scalar strength_tolerance{};       // Absolute difference, microtesla.
    Scalar inclination_tolerance{};    // Radians, in [0, pi].
    Scalar heading_tolerance{};        // Radians, in [0, pi].
    Scalar minimum_horizontal_field{}; // Positive normal microtesla; explicit configuration required.
};

struct MagneticFieldFailures
{
    bool reference = false;
    bool strength = false;
    bool inclination = false;
    bool horizontal_field = false;
    bool heading = false;
};

template <typename Scalar> struct MagneticFieldCheck
{
    Status status = Status::success;
    bool accepted = false;
    bool diagnostics_valid = false;
    bool strength_checked = false;
    bool inclination_checked = false;
    bool heading_checked = false;
    MagneticFieldFailures failures{};
    Scalar strength{}; // Microtesla, measured field rotated into NED.
    Scalar reference_strength{};
    Scalar inclination_error{}; // Radians; measured minus reference, positive Down.
    Scalar heading_error{};     // Wrapped radians; meaningful only when heading_checked.
};

namespace detail
{

template <typename Scalar> struct MagneticFieldGeometry
{
    Scalar strength{};
    Scalar horizontal{};
    Scalar inclination{};
};

/** Bounded geometry for a finite NED field; reject overflow/underflow, never clamp. */
template <typename Linalg>
[[nodiscard]] Status magnetic_field_geometry(linalg::Matrix<Linalg, 3U, 1U> const & field,
                                             MagneticFieldGeometry<typename Linalg::value_type> & output) noexcept
{
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    auto const horizontal_squared = field(0U) * field(0U) + field(1U) * field(1U);
    auto const squared_strength = horizontal_squared + field(2U) * field(2U);
    if (!scalar::is_finite<math_type>(squared_strength))
    {
        return Status::non_finite_result;
    }
    if (squared_strength < std::numeric_limits<value_type>::min())
    {
        return Status::zero_or_unsafe_divisor;
    }
    MagneticFieldGeometry<value_type> candidate;
    auto status = scalar::try_sqrt<math_type>(squared_strength, candidate.strength);
    if (!succeeded(status))
    {
        return status;
    }
    status = scalar::try_sqrt<math_type>(horizontal_squared, candidate.horizontal);
    if (!succeeded(status))
    {
        return status;
    }
    status = scalar::try_atan2<math_type>(field(2U), candidate.horizontal, candidate.inclination);
    if (succeeded(status))
    {
        output = candidate;
    }
    return status;
}

} /* end namespace detail */

/**
 * Stateless field plausibility, not sensor-health qualification or yaw alignment.
 * q_nb is unit and maps calibrated body m_b into NED; m_n is a trustworthy fixed
 * observation reference. field_reference_n supplies independent health geometry.
 * All fields use microtesla in the same navigation frame and fusion horizon.
 * No model lookup, bias estimate, clock or filtering is owned here.
 *
 * Selected strength/inclination checks use field_reference_n when marked as an
 * independent reference. Without one, require_reference rejects selected checks;
 * otherwise strength uses 45 +/- 40 microtesla and inclination is skipped.
 * This fallback is only a health test: it never replaces m_n for correction.
 * Default selection preserves the supplied-reference contract of existing calls.
 * Always require sufficient horizontal field in the measured and observation
 * vectors. The health reference never supplies the heading check or correction.
 * Apply the wrapped horizontal-direction check only when
 * heading_observable is true: the caller must establish aligned, independently
 * aided heading observability. Do not infer it from this magnetometer itself.
 * False skips that one check, not the later innovation gate. All upper-threshold
 * equalities and minimum-horizontal equality pass in computed arithmetic.
 *
 * Only success + accepted permits the next gate. Failures leave diagnostics as
 * invalid zero placeholders. A rejection is success + !accepted with named
 * failures. A passed check is not proof of an undisturbed field or valid attitude.
 */
template <typename Linalg>
[[nodiscard]] MagneticFieldCheck<typename Linalg::value_type>
check_magnetic_field(so3::UnitQuaternion<Linalg> const & q_nb, linalg::Matrix<Linalg, 3U, 1U> const & m_b,
                     linalg::Matrix<Linalg, 3U, 1U> const & m_n,
                     linalg::Matrix<Linalg, 3U, 1U> const & field_reference_n,
                     MagneticFieldParameters<typename Linalg::value_type> const & parameters, bool heading_observable,
                     MagneticFieldChecks const & checks = {}) noexcept
{
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    MagneticFieldCheck<value_type> result;
    if (!linalg::all_finite(q_nb.coefficients()) || !linalg::all_finite(m_b) || !linalg::all_finite(m_n) ||
        !scalar::is_finite<math_type>(parameters.strength_tolerance) ||
        !scalar::is_finite<math_type>(parameters.inclination_tolerance) ||
        !scalar::is_finite<math_type>(parameters.heading_tolerance) ||
        !scalar::is_finite<math_type>(parameters.minimum_horizontal_field))
    {
        result.status = Status::non_finite_input;
        return result;
    }
    auto const pi = scalar::pi<math_type>();
    if (parameters.strength_tolerance < value_type{0} || parameters.inclination_tolerance < value_type{0} ||
        parameters.inclination_tolerance > pi || parameters.heading_tolerance < value_type{0} ||
        parameters.heading_tolerance > pi ||
        parameters.minimum_horizontal_field < std::numeric_limits<value_type>::min() ||
        !(linalg::max_abs(m_b) > value_type{0}) || !(linalg::max_abs(m_n) > value_type{0}))
    {
        result.status = Status::domain_error;
        return result;
    }
    auto const measured_n = so3::rotate(q_nb, m_b);
    if (!linalg::all_finite(measured_n))
    {
        result.status = Status::non_finite_result;
        return result;
    }
    detail::MagneticFieldGeometry<value_type> measured;
    detail::MagneticFieldGeometry<value_type> reference;
    result.status = detail::magnetic_field_geometry(measured_n, measured);
    if (!succeeded(result.status))
    {
        return result;
    }
    result.status = detail::magnetic_field_geometry(m_n, reference);
    if (!succeeded(result.status))
    {
        return result;
    }
    detail::MagneticFieldGeometry<value_type> field_reference;
    if (checks.reference_available && (checks.strength || checks.inclination))
    {
        if (!linalg::all_finite(field_reference_n))
        {
            result.status = Status::non_finite_input;
            return result;
        }
        if (!(linalg::max_abs(field_reference_n) > value_type{0}))
        {
            result.status = Status::domain_error;
            return result;
        }
        // The single-reference overload has already computed this geometry.
        field_reference = reference;
        if (&field_reference_n != &m_n)
        {
            result.status = detail::magnetic_field_geometry(field_reference_n, field_reference);
        }
        if (!succeeded(result.status))
        {
            return result;
        }
    }
    auto const inclination_error = measured.inclination - field_reference.inclination;
    bool const horizontal_failure = measured.horizontal < parameters.minimum_horizontal_field ||
                                    reference.horizontal < parameters.minimum_horizontal_field;
    value_type heading_error{};
    bool const heading_checked = heading_observable && !horizontal_failure;
    if (heading_checked)
    {
        value_type measured_heading{};
        value_type reference_heading{};
        result.status = scalar::try_atan2<math_type>(measured_n(1U), measured_n(0U), measured_heading);
        if (!succeeded(result.status))
        {
            return result;
        }
        result.status = scalar::try_atan2<math_type>(m_n(1U), m_n(0U), reference_heading);
        if (!succeeded(result.status))
        {
            return result;
        }
        heading_error = measured_heading - reference_heading;
        if (heading_error > pi)
        {
            heading_error -= value_type{2} * pi;
        }
        else if (heading_error < -pi)
        {
            heading_error += value_type{2} * pi;
        }
    }
    result.strength = measured.strength;
    result.reference_strength = checks.reference_available ? field_reference.strength : value_type{45};
    result.strength_checked = checks.strength && (checks.reference_available || !checks.require_reference);
    result.inclination_checked = checks.inclination && checks.reference_available;
    result.inclination_error = result.inclination_checked ? inclination_error : value_type{0};
    result.heading_error = heading_error;
    result.heading_checked = heading_checked;
    result.failures.reference =
        checks.require_reference && !checks.reference_available && (checks.strength || checks.inclination);
    result.failures.strength =
        result.strength_checked &&
        (checks.reference_available ? scalar::absolute<math_type>(measured.strength - result.reference_strength) >
                                          parameters.strength_tolerance
                                    : measured.strength < value_type{5} || measured.strength > value_type{85});
    result.failures.inclination =
        result.inclination_checked && scalar::absolute<math_type>(inclination_error) > parameters.inclination_tolerance;
    result.failures.horizontal_field = horizontal_failure;
    result.failures.heading =
        heading_checked && scalar::absolute<math_type>(heading_error) > parameters.heading_tolerance;
    result.accepted = !result.failures.reference && !result.failures.strength && !result.failures.inclination &&
                      !result.failures.horizontal_field && !result.failures.heading;
    result.diagnostics_valid = true;
    return result;
}

/** Use the observation field for health checks when no separate reference is supplied. */
template <typename Linalg>
[[nodiscard]] MagneticFieldCheck<typename Linalg::value_type>
check_magnetic_field(so3::UnitQuaternion<Linalg> const & q_nb, linalg::Matrix<Linalg, 3U, 1U> const & m_b,
                     linalg::Matrix<Linalg, 3U, 1U> const & m_n,
                     MagneticFieldParameters<typename Linalg::value_type> const & parameters, bool heading_observable,
                     MagneticFieldChecks const & checks = {}) noexcept
{
    return check_magnetic_field(q_nb, m_b, m_n, m_n, parameters, heading_observable, checks);
}

} /* end namespace formal_eskf::runtime */
