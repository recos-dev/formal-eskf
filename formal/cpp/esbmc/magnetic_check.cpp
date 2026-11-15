/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "contracts.hpp"
#include <limits>
#include <formal_eskf/runtime/magnetic_field.hpp>
#ifndef FORMAL_ESKF_PROOF_MAG_CHECK_CONTRACT
#define FORMAL_ESKF_PROOF_MAG_CHECK_CONTRACT 0
#endif
#ifndef FORMAL_ESKF_PROOF_MAG_HEALTH_ALIAS
#define FORMAL_ESKF_PROOF_MAG_HEALTH_ALIAS 0
#endif
#ifndef FORMAL_ESKF_PROOF_MAG_CHECK_VALUES
#define FORMAL_ESKF_PROOF_MAG_CHECK_VALUES 0
#endif
namespace field_proof
{
using contract_proof::finite_vector;
using contract_proof::Linalg;
using contract_proof::Quaternion;
using contract_proof::same;
using contract_proof::same_vector;
using contract_proof::Scalar;
using formal_eskf::Status;
using Vector = contract_proof::Vector3;
using Geometry = formal_eskf::runtime::detail::MagneticFieldGeometry<Scalar>;
using Parameters = formal_eskf::runtime::MagneticFieldParameters<Scalar>;
using Checks = formal_eskf::runtime::MagneticFieldChecks;
using Result = formal_eskf::runtime::MagneticFieldCheck<Scalar>;
struct GeometryResult
{
    Geometry value;
    Status status;
};
inline bool finite_geometry(Geometry const & v)
{
    return std::isfinite(v.strength) && std::isfinite(v.horizontal) && std::isfinite(v.inclination);
}
inline bool nonzero(Vector const & v) { return v(0U) != Scalar{0} || v(1U) != Scalar{0} || v(2U) != Scalar{0}; }
inline bool same_parameters(Parameters a, Parameters b)
{
    return same(a.strength_tolerance, b.strength_tolerance) && same(a.inclination_tolerance, b.inclination_tolerance) &&
           same(a.heading_tolerance, b.heading_tolerance) &&
           same(a.minimum_horizontal_field, b.minimum_horizontal_field);
}
inline bool same_result(Result const & a, Result const & b)
{
    return a.status == b.status && a.accepted == b.accepted && a.diagnostics_valid == b.diagnostics_valid &&
           a.strength_checked == b.strength_checked && a.inclination_checked == b.inclination_checked &&
           a.heading_checked == b.heading_checked && a.failures.reference == b.failures.reference &&
           a.failures.strength == b.failures.strength && a.failures.inclination == b.failures.inclination &&
           a.failures.horizontal_field == b.failures.horizontal_field && a.failures.heading == b.failures.heading &&
           same(a.strength, b.strength) && same(a.reference_strength, b.reference_strength) &&
           same(a.inclination_error, b.inclination_error) && same(a.heading_error, b.heading_error);
}
struct Calls
{
    inline static Quaternion q;
    inline static Vector body, reference, measured, health;
    inline static GeometryResult geometry0{}, geometry1{}, geometry2{};
    inline static Scalar heading{};
    inline static Status heading_status{};
    inline static unsigned rotations{}, geometries{}, headings{};
};
} // namespace field_proof
#if FORMAL_ESKF_PROOF_MAG_CHECK_CONTRACT
// Scalar/geometry/heading frames are proved in magnetic_field.cpp for this
// exact backend. Forward-action input purity is checked below on its actual body.
namespace formal_eskf::so3
{
template <>
inline field_proof::Vector rotate<field_proof::Linalg>(field_proof::Quaternion const & q, field_proof::Vector const & v)
{
    using namespace field_proof;
    __ESBMC_assert(Calls::rotations++ == 0U && Calls::geometries == 0U && Calls::headings == 0U &&
                       same_vector(q.coefficients(), Calls::q.coefficients()) && same_vector(v, Calls::body),
                   "R-MAGNETIC-FIELD: original body field is rotated with the original quaternion before geometry");
    return Calls::measured;
}
} // namespace formal_eskf::so3
namespace formal_eskf::runtime::detail
{
template <>
inline Status magnetic_field_geometry<field_proof::Linalg>(field_proof::Vector const & v,
                                                           field_proof::Geometry & output)
{
    using namespace field_proof;
    __ESBMC_assert(Calls::rotations == 1U && Calls::geometries < 3U && Calls::headings == 0U,
                   "R-MAGNETIC-FIELD: measured, observation and optional health geometry precede heading check");
    unsigned const index = Calls::geometries++;
    GeometryResult result;
    if (index == 0U)
    {
        __ESBMC_assert(same_vector(v, Calls::measured),
                       "R-MAGNETIC-FIELD: measured geometry receives rotated measurement");
        result = Calls::geometry0;
    }
    else if (index == 1U)
    {
        __ESBMC_assert(same_vector(v, Calls::reference),
                       "R-MAGNETIC-FIELD: observation geometry receives original NED reference");
        result = Calls::geometry1;
    }
    else
    {
        __ESBMC_assert(same_vector(v, Calls::health),
                       "R-MAGNETIC-FIELD: independent health geometry receives only selected health reference");
        result = Calls::geometry2;
    }
#if FORMAL_ESKF_PROOF_MAG_CHECK_VALUES
    __ESBMC_assert(result.status == Status::success,
                   "R-MAGNETIC-FIELD: diagnostic path discharges each geometry success premise");
    output = result.value;
    return Status::success;
#else
    if (result.status == Status::success)
    {
        output = result.value;
    }
    return result.status;
#endif
}
template <>
inline Status magnetic_heading_error<field_proof::Linalg>(field_proof::Vector const & measured,
                                                          field_proof::Vector const & reference,
                                                          field_proof::Scalar & output)
{
    using namespace field_proof;
    __ESBMC_assert(
        Calls::rotations == 1U && Calls::geometries >= 2U && Calls::headings++ == 0U &&
            same_vector(measured, Calls::measured) && same_vector(reference, Calls::reference),
        "R-MAGNETIC-FIELD: heading uses measured and observation fields, never the independent health field");
#if FORMAL_ESKF_PROOF_MAG_CHECK_VALUES
    __ESBMC_assert(Calls::heading_status == Status::success,
                   "R-MAGNETIC-FIELD: diagnostic path discharges heading success premise");
    output = Calls::heading;
    return Status::success;
#else
    if (Calls::heading_status == Status::success)
    {
        output = Calls::heading;
    }
    return Calls::heading_status;
#endif
}
} // namespace formal_eskf::runtime::detail
#endif
void verify_magnetic_forward_frame(field_proof::Quaternion q, field_proof::Vector vector)
{
    using namespace field_proof;
    __ESBMC_assert(!FORMAL_ESKF_PROOF_MAG_CHECK_CONTRACT, "Runner error: actual forward action required");
    auto const old_q = q;
    auto const old_vector = vector;
    // Same-type matrix coefficients are proved by verify_imu_rotation. The
    // independent ordered dot oracle binds all forward-action coordinates.
    auto const rotation = formal_eskf::so3::to_rotation_matrix(q);
    Vector expected;
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            expected(row) += rotation(row, column) * vector(column);
        }
    }
    auto const result = formal_eskf::so3::rotate(q, vector);
    __ESBMC_assert(same_vector(result, expected),
                   "R-MAGNETIC-FIELD: actual forward action uses R times body field with ordered IEEE dots");
    __ESBMC_assert(same_vector(q.coefficients(), old_q.coefficients()) && same_vector(vector, old_vector),
                   "R-MAGNETIC-FIELD: actual forward rotation preserves both borrowed inputs for every IEEE value");
}
void verify_magnetic_check(field_proof::Quaternion q, field_proof::Vector body, field_proof::Vector reference,
                           field_proof::Vector health, field_proof::Parameters parameters, field_proof::Checks checks,
                           bool observable, field_proof::Vector measured, field_proof::GeometryResult geometry0,
                           field_proof::GeometryResult geometry1, field_proof::GeometryResult geometry2,
                           field_proof::Status heading_status, field_proof::Scalar heading)
{
    using namespace field_proof;
    __ESBMC_assert(FORMAL_ESKF_PROOF_MAG_CHECK_CONTRACT, "Runner error: field caller requires proved callee frames");
    static_assert(FORMAL_ESKF_PROOF_MAG_HEALTH_ALIAS < 4);
    // Producer success guarantees these finite fields. No premise constrains
    // public inputs, failure statuses, check flags or observed heading result.
    __ESBMC_assume(geometry0.status != Status::success || finite_geometry(geometry0.value));
    __ESBMC_assume(geometry1.status != Status::success || finite_geometry(geometry1.value));
    __ESBMC_assume(geometry2.status != Status::success || finite_geometry(geometry2.value));
    auto const old_q = q;
    auto const old_body = body;
    auto const old_reference = reference;
    auto const old_health = health;
    auto const old_parameters = parameters;
    auto const old_checks = checks;
    Vector const * health_pointer = nullptr;
    if constexpr (FORMAL_ESKF_PROOF_MAG_HEALTH_ALIAS == 1)
    {
        health_pointer = &reference;
    }
    if constexpr (FORMAL_ESKF_PROOF_MAG_HEALTH_ALIAS == 2)
    {
        health_pointer = &health;
    }
    if constexpr (FORMAL_ESKF_PROOF_MAG_HEALTH_ALIAS == 3)
    {
        health_pointer = &body;
    }
    Calls::q = q;
    Calls::body = body;
    Calls::reference = reference;
    Calls::measured = measured;
    if (health_pointer)
    {
        Calls::health = *health_pointer;
    }
    Calls::geometry0 = geometry0;
    Calls::geometry1 = geometry1;
    Calls::geometry2 = geometry2;
    Calls::heading = heading;
    Calls::heading_status = heading_status;
    Calls::rotations = Calls::geometries = Calls::headings = 0U;
    Result expected;
    unsigned rotations = 0U, geometries = 0U, headings = 0U;
    constexpr Scalar pi = std::numbers::pi_v<Scalar>;
    bool const finite =
        finite_vector(q.coefficients()) && finite_vector(body) && finite_vector(reference) &&
        std::isfinite(parameters.strength_tolerance) && std::isfinite(parameters.inclination_tolerance) &&
        std::isfinite(parameters.heading_tolerance) && std::isfinite(parameters.minimum_horizontal_field);
    bool const domain = parameters.strength_tolerance >= Scalar{0} && parameters.inclination_tolerance >= Scalar{0} &&
                        parameters.inclination_tolerance <= pi && parameters.heading_tolerance >= Scalar{0} &&
                        parameters.heading_tolerance <= pi &&
                        parameters.minimum_horizontal_field >= std::numeric_limits<Scalar>::min() && nonzero(body) &&
                        nonzero(reference);
    expected.status = !finite ? Status::non_finite_input : !domain ? Status::domain_error : Status::success;
    if (expected.status == Status::success)
    {
        rotations = 1U;
        if (!finite_vector(measured))
        {
            expected.status = Status::non_finite_result;
        }
    }
    if (expected.status == Status::success)
    {
        geometries = 1U;
        expected.status = geometry0.status;
    }
    if (expected.status == Status::success)
    {
        geometries = 2U;
        expected.status = geometry1.status;
    }
    bool const available = health_pointer != nullptr;
    Geometry health_geometry;
    if (expected.status == Status::success && available && (checks.strength || checks.inclination))
    {
        if (!finite_vector(*health_pointer))
        {
            expected.status = Status::non_finite_input;
        }
        else if (!nonzero(*health_pointer))
        {
            expected.status = Status::domain_error;
        }
        else if constexpr (FORMAL_ESKF_PROOF_MAG_HEALTH_ALIAS == 1)
        {
            health_geometry = geometry1.value;
        }
        else
        {
            geometries = 3U;
            expected.status = geometry2.status;
            health_geometry = geometry2.value;
        }
    }
    bool const horizontal_failure = geometry0.value.horizontal < parameters.minimum_horizontal_field ||
                                    geometry1.value.horizontal < parameters.minimum_horizontal_field;
    bool const heading_checked = observable && !horizontal_failure;
    if (expected.status == Status::success && heading_checked)
    {
        headings = 1U;
        expected.status = heading_status;
    }
    if (expected.status == Status::success)
    {
        expected.diagnostics_valid = true;
        expected.strength = geometry0.value.strength;
        expected.reference_strength = available ? health_geometry.strength : Scalar{45};
        expected.strength_checked = checks.strength && (available || !checks.require_reference);
        expected.inclination_checked = checks.inclination && available;
        expected.heading_checked = heading_checked;
        expected.inclination_error =
            expected.inclination_checked ? geometry0.value.inclination - health_geometry.inclination : Scalar{0};
        expected.heading_error = heading_checked ? heading : Scalar{0};
        auto & f = expected.failures;
        f.reference = checks.require_reference && !available && (checks.strength || checks.inclination);
        f.horizontal_field = horizontal_failure;
        if (expected.strength_checked)
        {
            if (available)
            {
                f.strength =
                    std::fabs(geometry0.value.strength - health_geometry.strength) > parameters.strength_tolerance;
            }
            else
            {
                f.strength = geometry0.value.strength < Scalar{5} || geometry0.value.strength > Scalar{85};
            }
        }
        f.inclination =
            expected.inclination_checked && std::fabs(expected.inclination_error) > parameters.inclination_tolerance;
        f.heading = heading_checked && std::fabs(heading) > parameters.heading_tolerance;
        unsigned const failures =
            static_cast<unsigned>(f.reference) + f.strength + f.inclination + f.horizontal_field + f.heading;
        expected.accepted = failures == 0U;
    }
    // Complementary obligations: every failure/status/call frame is checked
    // unconditionally; numeric diagnostics are checked on the validated
    // successful-computation path. Rejection is still allowed in this path.
    if constexpr (FORMAL_ESKF_PROOF_MAG_CHECK_VALUES)
    {
        __ESBMC_assume(expected.status == Status::success);
    }
    auto const actual = formal_eskf::runtime::check_magnetic_field<Linalg>(q, body, reference, health_pointer,
                                                                           parameters, observable, checks);
    __ESBMC_assert(actual.status == expected.status && Calls::rotations == rotations &&
                       Calls::geometries == geometries && Calls::headings == headings,
                   "R-MAGNETIC-FIELD: optional-reference selection and reuse preserve exact status and call priority");
    if constexpr (FORMAL_ESKF_PROOF_MAG_CHECK_VALUES)
    {
        __ESBMC_assert(
            same_result(actual, expected),
            "R-MAGNETIC-FIELD: all physical checks, threshold equalities and complete successful diagnostics match");
    }
    else
    {
        __ESBMC_assert(actual.status == Status::success ? actual.diagnostics_valid : same_result(actual, expected),
                       "R-MAGNETIC-FIELD: all failure paths publish only invalid zero diagnostics");
    }
    __ESBMC_assert(same_vector(q.coefficients(), old_q.coefficients()) && same_vector(body, old_body) &&
                       same_vector(reference, old_reference) && same_vector(health, old_health) &&
                       same_parameters(parameters, old_parameters) && checks.strength == old_checks.strength &&
                       checks.inclination == old_checks.inclination &&
                       checks.require_reference == old_checks.require_reference,
                   "R-MAGNETIC-FIELD: every borrowed input and all pointer alias modes remain unchanged");
}
int main() { return 0; }
