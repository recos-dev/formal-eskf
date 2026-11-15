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
#ifndef FORMAL_ESKF_PROOF_MAG_GEOMETRY_MODE
#define FORMAL_ESKF_PROOF_MAG_GEOMETRY_MODE 0
#endif
namespace geometry_proof
{
using contract_proof::finite_vector;
using contract_proof::Linalg;
using contract_proof::OpaqueMath;
using contract_proof::Quaternion;
using contract_proof::same;
using contract_proof::same_vector;
using contract_proof::Scalar;
using formal_eskf::Status;
using Vector = contract_proof::Vector3;
using Geometry = formal_eskf::runtime::detail::MagneticFieldGeometry<Scalar>;
using Parameters = formal_eskf::runtime::MagneticFieldParameters<Scalar>;
inline Status root_status(Scalar input, Scalar value)
{
    return !std::isfinite(input)   ? Status::non_finite_input
           : input < Scalar{0}     ? Status::domain_error
           : !std::isfinite(value) ? Status::non_finite_result
                                   : Status::success;
}
inline Status angle_status(Scalar y, Scalar x, Scalar value)
{
    return !std::isfinite(y) || !std::isfinite(x) ? Status::non_finite_input
           : y == Scalar{0} && x == Scalar{0}     ? Status::domain_error
           : !std::isfinite(value)                ? Status::non_finite_result
                                                  : Status::success;
}
inline bool same_geometry(Geometry a, Geometry b)
{
    return same(a.strength, b.strength) && same(a.horizontal, b.horizontal) && same(a.inclination, b.inclination);
}
inline bool same_parameters(Parameters a, Parameters b)
{
    return same(a.strength_tolerance, b.strength_tolerance) && same(a.inclination_tolerance, b.inclination_tolerance) &&
           same(a.heading_tolerance, b.heading_tolerance) &&
           same(a.minimum_horizontal_field, b.minimum_horizontal_field);
}
struct Root
{
    Scalar argument, value;
};
struct Angle
{
    Scalar y, x, value;
};
struct Calls
{
    inline static Root root0{}, root1{};
    inline static Angle angle0{}, angle1{};
    inline static unsigned roots{}, angles{};
};
} // namespace geometry_proof
#if FORMAL_ESKF_PROOF_MAG_GEOMETRY_MODE == 1
namespace formal_eskf::scalar
{
template <>
inline Status sqrt<geometry_proof::OpaqueMath>(geometry_proof::Scalar input, geometry_proof::Scalar & output)
{
    using namespace geometry_proof;
    __ESBMC_assert(Calls::roots < 2U && Calls::angles == 0U, "R-MAGNETIC-FIELD: at most two roots precede inclination");
    bool const first = Calls::roots++ == 0U;
    Scalar const argument = first ? Calls::root0.argument : Calls::root1.argument;
    Scalar const value = first ? Calls::root0.value : Calls::root1.value;
    __ESBMC_assert(same(input, argument),
                   "R-MAGNETIC-FIELD: total strength then horizontal squared magnitude reach checked roots");
    auto const status = root_status(input, value);
    if (status == Status::success)
    {
        output = value;
    }
    return status;
}
} // namespace formal_eskf::scalar
#endif
#if FORMAL_ESKF_PROOF_MAG_GEOMETRY_MODE >= 1
namespace formal_eskf::scalar
{
template <>
inline Status atan2<geometry_proof::OpaqueMath>(geometry_proof::Scalar y, geometry_proof::Scalar x,
                                                geometry_proof::Scalar & output)
{
    using namespace geometry_proof;
    __ESBMC_assert(Calls::angles < 2U, "R-MAGNETIC-FIELD: bounded angle calls");
    bool const first = Calls::angles++ == 0U;
    Scalar const expected_y = first ? Calls::angle0.y : Calls::angle1.y;
    Scalar const expected_x = first ? Calls::angle0.x : Calls::angle1.x;
    Scalar const value = first ? Calls::angle0.value : Calls::angle1.value;
    __ESBMC_assert(same(y, expected_y) && same(x, expected_x),
                   "R-MAGNETIC-FIELD: exact y,x order reaches checked atan2");
    if constexpr (FORMAL_ESKF_PROOF_MAG_GEOMETRY_MODE == 1)
    {
        __ESBMC_assert(first && Calls::roots == 2U, "R-MAGNETIC-FIELD: one inclination follows both successful roots");
    }
#if FORMAL_ESKF_PROOF_MAG_GEOMETRY_MODE >= 3
    __ESBMC_assert(angle_status(y, x, value) == Status::success,
                   "R-MAGNETIC-HEADING: numeric path satisfies the checked-angle producer's successful domain");
    output = value;
    return Status::success;
#else
    auto const status = angle_status(y, x, value);
    if (status == Status::success)
    {
        output = value;
    }
    return status;
#endif
}
} // namespace formal_eskf::scalar
#endif
void verify_magnetic_root(geometry_proof::Scalar input, geometry_proof::Scalar output, geometry_proof::Scalar root)
{
    using namespace geometry_proof;
    __ESBMC_assert(FORMAL_ESKF_PROOF_MAG_GEOMETRY_MODE == 0, "Runner error: actual checked root required");
    auto const before = output;
    OpaqueMath::prepare(root, Scalar{0}, Scalar{0}, Scalar{0});
    auto const status = formal_eskf::scalar::sqrt<OpaqueMath>(input, output);
    auto const expected = root_status(input, root);
    bool const called = std::isfinite(input) && input >= Scalar{0};
    __ESBMC_assert(status == expected && same(output, status == Status::success ? root : before) &&
                       OpaqueMath::sqrt.calls == (called ? 1U : 0U) &&
                       (!called || same(OpaqueMath::sqrt.argument, input)),
                   "R-MAGNETIC-FIELD: actual checked root domain, argument, result and atomic output contract");
}
void verify_magnetic_angle(geometry_proof::Scalar y, geometry_proof::Scalar x, geometry_proof::Scalar output,
                           geometry_proof::Scalar angle)
{
    using namespace geometry_proof;
    __ESBMC_assert(FORMAL_ESKF_PROOF_MAG_GEOMETRY_MODE == 0, "Runner error: actual checked atan2 required");
    auto const before = output;
    OpaqueMath::prepare(Scalar{0}, Scalar{0}, Scalar{0}, angle);
    auto const status = formal_eskf::scalar::atan2<OpaqueMath>(y, x, output);
    bool const called = std::isfinite(y) && std::isfinite(x) && (y != Scalar{0} || x != Scalar{0});
    __ESBMC_assert(
        status == angle_status(y, x, angle) && same(output, status == Status::success ? angle : before) &&
            OpaqueMath::atan2.calls == (called ? 1U : 0U) &&
            (!called || (same(OpaqueMath::atan2.y, y) && same(OpaqueMath::atan2.x, x))),
        "R-MAGNETIC-FIELD: actual atan2 origin/finite checks, argument order, result and atomic output contract");
}
void verify_magnetic_geometry(geometry_proof::Vector field, geometry_proof::Geometry output,
                              geometry_proof::Scalar strength, geometry_proof::Scalar horizontal,
                              geometry_proof::Scalar inclination)
{
    using namespace geometry_proof;
    __ESBMC_assert(FORMAL_ESKF_PROOF_MAG_GEOMETRY_MODE == 1, "Runner error: geometry requires proved scalar frames");
    auto const before = output;
    auto const old_field = field;
    Scalar const h2 = field(0U) * field(0U) + field(1U) * field(1U), b2 = h2 + field(2U) * field(2U);
    Calls::root0 = {b2, strength};
    Calls::root1 = {h2, horizontal};
    Calls::angle0 = {field(2U), horizontal, inclination};
    Calls::roots = Calls::angles = 0U;
    Status expected = Status::success;
    unsigned roots = 0U, angles = 0U;
    if (!std::isfinite(b2))
    {
        expected = Status::non_finite_result;
    }
    else if (b2 < std::numeric_limits<Scalar>::min())
    {
        expected = Status::zero_or_unsafe_divisor;
    }
    else
    {
        roots = 1U;
        expected = root_status(b2, strength);
        if (expected == Status::success)
        {
            roots = 2U;
            expected = root_status(h2, horizontal);
            if (expected == Status::success)
            {
                angles = 1U;
                expected = angle_status(field(2U), horizontal, inclination);
            }
        }
    }
    auto const actual = formal_eskf::runtime::detail::magnetic_field_geometry<Linalg>(field, output);
    __ESBMC_assert(
        actual == expected && Calls::roots == roots && Calls::angles == angles,
        "R-MAGNETIC-FIELD: total squared magnitude guards precede ordered scalar calls with exact failure status");
    if (actual == Status::success)
    {
        __ESBMC_assert(same(output.strength, strength) && same(output.horizontal, horizontal) &&
                           same(output.inclination, inclination) && std::isfinite(output.strength) &&
                           std::isfinite(output.horizontal) && std::isfinite(output.inclination),
                       "R-MAGNETIC-FIELD: successful geometry publishes all finite scalar results together");
    }
    else
    {
        __ESBMC_assert(same_geometry(output, before),
                       "R-MAGNETIC-FIELD: any geometry failure preserves the complete output");
    }
    __ESBMC_assert(same_vector(field, old_field), "R-MAGNETIC-FIELD: geometry never changes its borrowed field");
}
void verify_magnetic_heading_error(geometry_proof::Vector measured, geometry_proof::Vector reference,
                                   geometry_proof::Scalar output, geometry_proof::Scalar measured_angle,
                                   geometry_proof::Scalar reference_angle)
{
    using namespace geometry_proof;
    __ESBMC_assert(FORMAL_ESKF_PROOF_MAG_GEOMETRY_MODE == 2,
                   "Runner error: heading error requires proved checked angles");
    auto const before = output;
    auto const old_measured = measured;
    auto const old_reference = reference;
    Calls::angle0 = {measured(1U), measured(0U), measured_angle};
    Calls::angle1 = {reference(1U), reference(0U), reference_angle};
    Calls::angles = Calls::roots = 0U;
    auto const first = angle_status(measured(1U), measured(0U), measured_angle),
               second = angle_status(reference(1U), reference(0U), reference_angle);
    Status const expected = first == Status::success ? second : first;
    auto const actual = formal_eskf::runtime::detail::magnetic_heading_error<Linalg>(measured, reference, output);
    __ESBMC_assert(actual == expected && Calls::angles == (first == Status::success ? 2U : 1U) &&
                       (actual == Status::success || same(output, before)),
                   "R-MAGNETIC-HEADING: checked-angle failure propagation, call count and output atomicity");
    __ESBMC_assert(same_vector(measured, old_measured) && same_vector(reference, old_reference),
                   "R-MAGNETIC-HEADING: both borrowed fields, including unused Down coefficients, remain unchanged");
}
void verify_magnetic_heading_value(geometry_proof::Vector measured, geometry_proof::Vector reference,
                                   geometry_proof::Scalar measured_angle, geometry_proof::Scalar reference_angle)
{
    using namespace geometry_proof;
    static_assert(FORMAL_ESKF_PROOF_MAG_GEOMETRY_MODE <= 4);
    __ESBMC_assert(FORMAL_ESKF_PROOF_MAG_GEOMETRY_MODE >= 3,
                   "Runner error: numeric path uses successful checked angles");
    __ESBMC_assume(angle_status(measured(1U), measured(0U), measured_angle) == Status::success &&
                   angle_status(reference(1U), reference(0U), reference_angle) == Status::success);
    constexpr Scalar pi = std::numbers::pi_v<Scalar>;
    if constexpr (FORMAL_ESKF_PROOF_MAG_GEOMETRY_MODE == 4)
    {
        // Numerical principal-range premise, not inferred from finiteness.
        __ESBMC_assume(measured_angle >= -pi && measured_angle <= pi && reference_angle >= -pi &&
                       reference_angle <= pi);
    }
    Calls::angle0 = {measured(1U), measured(0U), measured_angle};
    Calls::angle1 = {reference(1U), reference(0U), reference_angle};
    Calls::angles = Calls::roots = 0U;
    Scalar expected = measured_angle - reference_angle;
    if (expected > pi)
    {
        expected -= Scalar{2} * pi;
    }
    else if (expected < -pi)
    {
        expected += Scalar{2} * pi;
    }
    Scalar output{};
    auto const status = formal_eskf::runtime::detail::magnetic_heading_error<Linalg>(measured, reference, output);
    __ESBMC_assert(
        status == Status::success && Calls::angles == 2U && same(output, expected),
        "R-MAGNETIC-HEADING: successful scalar paths publish the exact IEEE measured-minus-reference single wrap");
    if constexpr (FORMAL_ESKF_PROOF_MAG_GEOMETRY_MODE == 4)
    {
        __ESBMC_assert(std::isfinite(output) && output >= -pi && output <= pi,
                       "R-MAGNETIC-HEADING: principal primitive headings imply a finite principal wrapped result");
    }
}

void verify_magnetic_validation(geometry_proof::Quaternion q, geometry_proof::Vector body,
                                geometry_proof::Vector reference, geometry_proof::Parameters parameters)
{
    using namespace geometry_proof;
    auto const old_q = q;
    auto const old_body = body;
    auto const old_reference = reference;
    auto const old_parameters = parameters;
    bool const finite =
        finite_vector(q.coefficients()) && finite_vector(body) && finite_vector(reference) &&
        std::isfinite(parameters.strength_tolerance) && std::isfinite(parameters.inclination_tolerance) &&
        std::isfinite(parameters.heading_tolerance) && std::isfinite(parameters.minimum_horizontal_field);
    constexpr Scalar pi = std::numbers::pi_v<Scalar>;
    bool const domain = parameters.strength_tolerance >= Scalar{0} && parameters.inclination_tolerance >= Scalar{0} &&
                        parameters.inclination_tolerance <= pi && parameters.heading_tolerance >= Scalar{0} &&
                        parameters.heading_tolerance <= pi &&
                        parameters.minimum_horizontal_field >= std::numeric_limits<Scalar>::min() &&
                        (body(0U) != Scalar{0} || body(1U) != Scalar{0} || body(2U) != Scalar{0}) &&
                        (reference(0U) != Scalar{0} || reference(1U) != Scalar{0} || reference(2U) != Scalar{0});
    auto const actual = formal_eskf::runtime::detail::validate_magnetic_field<Linalg>(q, body, reference, parameters);
    __ESBMC_assert(actual == (!finite   ? Status::non_finite_input
                              : !domain ? Status::domain_error
                                        : Status::success),
                   "R-MAGNETIC-FIELD: finite-input priority, inclusive angular bounds, positive-normal horizontal "
                   "floor and nonzero fields");
    __ESBMC_assert(same_vector(q.coefficients(), old_q.coefficients()) && same_vector(body, old_body) &&
                       same_vector(reference, old_reference) && same_parameters(parameters, old_parameters),
                   "R-MAGNETIC-FIELD: validation preserves all input fields and parameters");
}
int main() { return 0; }
