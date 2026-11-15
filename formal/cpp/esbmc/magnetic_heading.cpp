/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "contracts.hpp"
#include <formal_eskf/eskf/configuration/ahrs.hpp>
#include <formal_eskf/eskf/configuration/ins.hpp>
#include <formal_eskf/eskf/magnetometer.hpp>
#include <formal_eskf/runtime/magnetic_field.hpp>
#include <formal_eskf/runtime/measurement_fusion.hpp>
#include <formal_eskf/so3/rotation_vector.hpp>
#define private public
#include <formal_eskf/runtime/magnetometer.hpp>
#undef private
#ifndef FORMAL_ESKF_PROOF_HEADING_CALLER
#define FORMAL_ESKF_PROOF_HEADING_CALLER 0
#endif
#ifndef FORMAL_ESKF_PROOF_HEADING_MODE
#define FORMAL_ESKF_PROOF_HEADING_MODE 0
#endif
namespace heading_proof
{
using contract_proof::finite_vector;
using contract_proof::Linalg;
using contract_proof::OpaqueMath;
using contract_proof::Quaternion;
using contract_proof::same;
using contract_proof::same_vector;
using contract_proof::Scalar;
using formal_eskf::Status;
using Heading = formal_eskf::runtime::MagneticHeading<Linalg>;
using State = Heading::State;
using Parameters = Heading::Parameters;
using Vector = Heading::vector_type;
using Horizontal = Heading::horizontal_type;
inline bool same_state(State const & a, State const & b)
{
    return same(a.innovation, b.innovation) && same(a.filtered_innovation, b.filtered_innovation) &&
           same(a.horizontal_acceleration, b.horizontal_acceleration) && a.innovation_valid == b.innovation_valid &&
           a.observable == b.observable && a.consistent == b.consistent;
}
inline bool same_parameters(Parameters const & a, Parameters const & b)
{
    return same(a.heading_noise, b.heading_noise) && same(a.acceleration_limit, b.acceleration_limit) &&
           same(a.minimum_horizontal_field, b.minimum_horizontal_field);
}
inline bool finite_acceleration(Heading const & h)
{
    return finite_vector(h.m_acceleration) && std::isfinite(h.m_state.horizontal_acceleration);
}
inline bool bounded_heading(State const & s)
{
    constexpr Scalar pi = std::numbers::pi_v<Scalar>;
    return std::isfinite(s.innovation) && std::fabs(s.innovation) <= pi && std::isfinite(s.filtered_innovation) &&
           std::fabs(s.filtered_innovation) <= Scalar{2} * pi;
}
inline Status input_status(Quaternion const & q, Vector const & b, Vector const & n, Parameters const & p)
{
    bool const finite = finite_vector(q.coefficients()) && finite_vector(b) && finite_vector(n) &&
                        std::isfinite(p.heading_noise) && std::isfinite(p.acceleration_limit) &&
                        std::isfinite(p.minimum_horizontal_field);
    bool const domain = p.heading_noise > Scalar{0} && p.heading_noise <= std::numbers::pi_v<Scalar> &&
                        p.acceleration_limit >= Scalar{0} &&
                        p.minimum_horizontal_field >= std::numeric_limits<Scalar>::min();
    return !finite ? Status::non_finite_input : !domain ? Status::domain_error : Status::success;
}
struct Calls
{
    inline static Quaternion q;
    inline static Vector body, reference, measured;
    inline static Scalar measured_horizontal{}, reference_horizontal{}, innovation{};
    inline static Status status{};
    inline static unsigned rotations{}, norms{}, headings{};
};
} // namespace heading_proof
#if FORMAL_ESKF_PROOF_HEADING_CALLER
// Exact-type forward purity and 2D norm producers are checked separately.
// Every IEEE norm return is allowed, including negative, infinite and NaN.
namespace formal_eskf::so3
{
template <>
inline heading_proof::Vector rotate<heading_proof::Linalg>(heading_proof::Quaternion const & q,
                                                           heading_proof::Vector const & v)
{
    using namespace heading_proof;
    __ESBMC_assert(Calls::rotations++ == 0U && Calls::norms == 0U && Calls::headings == 0U &&
                       same_vector(q.coefficients(), Calls::q.coefficients()) && same_vector(v, Calls::body),
                   "R-MAGNETIC-HEADING: rotate the original field before norms and heading");
    return Calls::measured;
}
} // namespace formal_eskf::so3
namespace formal_eskf::linalg
{
template <> inline heading_proof::Scalar norm<heading_proof::Linalg, 2U>(heading_proof::Horizontal const & v)
{
    using namespace heading_proof;
    __ESBMC_assert(Calls::rotations == 1U && Calls::norms < 2U && Calls::headings == 0U,
                   "R-MAGNETIC-HEADING: both horizontal norms precede heading");
    unsigned const index = Calls::norms++;
    if (index == 0U)
    {
        __ESBMC_assert(same(v(0U), Calls::measured(0U)) && same(v(1U), Calls::measured(1U)),
                       "R-MAGNETIC-HEADING: measured norm receives exactly North and East");
        return Calls::measured_horizontal;
    }
    __ESBMC_assert(same(v(0U), Calls::reference(0U)) && same(v(1U), Calls::reference(1U)),
                   "R-MAGNETIC-HEADING: reference norm receives exactly North and East");
    return Calls::reference_horizontal;
}
} // namespace formal_eskf::linalg
namespace formal_eskf::runtime::detail
{
template <>
inline Status magnetic_heading_error<heading_proof::Linalg>(heading_proof::Vector const & measured,
                                                            heading_proof::Vector const & reference,
                                                            heading_proof::Scalar & output)
{
    using namespace heading_proof;
    __ESBMC_assert(Calls::rotations == 1U && Calls::norms == 2U && Calls::headings++ == 0U &&
                       same_vector(measured, Calls::measured) && same_vector(reference, Calls::reference),
                   "R-MAGNETIC-HEADING: wrapped innovation uses original reference and rotated field exactly once");
#if FORMAL_ESKF_PROOF_HEADING_MODE == 1
    __ESBMC_assert(Calls::status == Status::success,
                   "R-MAGNETIC-HEADING: numeric path discharges the wrapped-heading success premise");
    output = Calls::innovation;
    return Status::success;
#else
    if (Calls::status == Status::success)
    {
        output = Calls::innovation;
    }
    return Calls::status;
#endif
}
} // namespace formal_eskf::runtime::detail
#endif
void verify_heading_initialization(heading_proof::Heading heading, unsigned action)
{
    using namespace heading_proof;
    Heading fresh;
    State zero;
    Horizontal zero_acceleration;
    __ESBMC_assert(same_state(fresh.state(), zero) && same_vector(fresh.m_acceleration, zero_acceleration) &&
                       finite_acceleration(fresh) && bounded_heading(fresh.state()),
                   "R-MAGNETIC-HEADING: construction establishes zero history and both inductive domains");
    __ESBMC_assume(action < 3U);
    auto const before = heading;
    auto expected = before.m_state;
    auto acceleration = before.m_acceleration;
    if (action == 0U)
    {
        heading.reset();
        expected = zero;
        acceleration = zero_acceleration;
    }
    else if (action == 1U)
    {
        heading.reset_acceleration();
        acceleration = zero_acceleration;
        expected.horizontal_acceleration = Scalar{0};
        expected.observable = false;
    }
    else
    {
        heading.reset_heading();
        expected.innovation = Scalar{0};
        expected.filtered_innovation = Scalar{0};
        expected.innovation_valid = false;
        expected.consistent = true;
    }
    auto const & state = heading.state();
    __ESBMC_assert(same_state(state, expected) && same_vector(heading.m_acceleration, acceleration) &&
                       &state == &heading.m_state,
                   "R-MAGNETIC-HEADING: full, IMU-gap and successful-alignment resets have distinct complete frames");
    __ESBMC_assert((!finite_acceleration(before) || finite_acceleration(heading)) &&
                       (!bounded_heading(before.state()) || bounded_heading(heading.state())),
                   "R-MAGNETIC-HEADING: every reset preserves both initialized history invariants");
}
void verify_heading_norm(heading_proof::Horizontal value, heading_proof::Scalar root)
{
    using namespace heading_proof;
    __ESBMC_assert(!FORMAL_ESKF_PROOF_HEADING_CALLER, "Runner error: actual 2D norm required");
    auto const before = value;
    Scalar const squared = (Scalar{0} + value(0U) * value(0U)) + value(1U) * value(1U);
    OpaqueMath::prepare(root, Scalar{0}, Scalar{0}, Scalar{0});
    Scalar const result = formal_eskf::linalg::norm(value);
    __ESBMC_assert(OpaqueMath::sqrt.calls == 1U && same(OpaqueMath::sqrt.argument, squared) && same(result, root) &&
                       same_vector(value, before),
                   "R-MAGNETIC-HEADING: actual 2D norm binds ordered squared sum, arbitrary root and pure input");
}
void verify_heading_validation(heading_proof::Quaternion q, heading_proof::Vector body, heading_proof::Vector reference,
                               heading_proof::Parameters parameters)
{
    using namespace heading_proof;
    auto const old_q = q;
    auto const old_body = body;
    auto const old_reference = reference;
    auto const old_parameters = parameters;
    __ESBMC_assert(Heading::validate_inputs(q, body, reference, parameters) ==
                       input_status(q, body, reference, parameters),
                   "R-MAGNETIC-HEADING: finite inputs and exact threshold domains determine validation; zero fields "
                   "are permitted");
    __ESBMC_assert(same_vector(q.coefficients(), old_q.coefficients()) && same_vector(body, old_body) &&
                       same_vector(reference, old_reference) && same_parameters(parameters, old_parameters),
                   "R-MAGNETIC-HEADING: actual validation preserves every borrowed input");
}
void verify_heading_acceleration(heading_proof::Heading heading, heading_proof::Vector acceleration,
                                 heading_proof::Scalar dt, heading_proof::Scalar root)
{
    using namespace heading_proof;
    auto const before = heading;
    auto const old_input = acceleration;
    auto expected = before.m_state;
    Horizontal candidate;
    Scalar const alpha = dt / (Scalar{1} + dt);
    for (std::size_t axis = 0U; axis < 2U; ++axis)
    {
        candidate(axis) = before.m_acceleration(axis) + (acceleration(axis) - before.m_acceleration(axis)) * alpha;
    }
    Scalar const squared = (Scalar{0} + candidate(0U) * candidate(0U)) + candidate(1U) * candidate(1U);
    Status wanted = !finite_vector(acceleration) || !std::isfinite(dt)     ? Status::non_finite_input
                    : dt < std::numeric_limits<Scalar>::min()              ? Status::domain_error
                    : !finite_vector(candidate) || !std::isfinite(squared) ? Status::non_finite_result
                    : squared < Scalar{0}                                  ? Status::domain_error
                    : !std::isfinite(root)                                 ? Status::non_finite_result
                                                                           : Status::success;
    unsigned const calls = finite_vector(acceleration) && std::isfinite(dt) &&
                                   dt >= std::numeric_limits<Scalar>::min() && finite_vector(candidate) &&
                                   std::isfinite(squared) && squared >= Scalar{0}
                               ? 1U
                               : 0U;
    OpaqueMath::prepare(root, Scalar{0}, Scalar{0}, Scalar{0});
    Status const status = heading.update_acceleration(acceleration, dt);
    __ESBMC_assert(status == wanted && OpaqueMath::sqrt.calls == calls &&
                       (calls == 0U || same(OpaqueMath::sqrt.argument, squared)),
                   "R-MAGNETIC-HEADING: actual vector LPF and squared norm preserve exact guards and root request");
    if (status == Status::success)
    {
        expected.horizontal_acceleration = root;
        expected.observable = false;
        __ESBMC_assert(same_vector(heading.m_acceleration, candidate),
                       "R-MAGNETIC-HEADING: opposing N/E accelerations cancel before norm; Down is not filtered");
    }
    else
    {
        __ESBMC_assert(same_vector(heading.m_acceleration, before.m_acceleration),
                       "R-MAGNETIC-HEADING: acceleration failure preserves vector history");
    }
    __ESBMC_assert(same_state(heading.state(), expected) && same_vector(acceleration, old_input),
                   "R-MAGNETIC-HEADING: acceleration update preserves heading history/latch and borrowed input");
    __ESBMC_assert(!finite_acceleration(before) || finite_acceleration(heading),
                   "R-MAGNETIC-HEADING: every acceleration update preserves initialized finite history");
}
void verify_heading_update(heading_proof::Heading heading, heading_proof::Quaternion q, heading_proof::Vector body,
                           heading_proof::Vector reference, heading_proof::Parameters parameters, bool aligned,
                           bool aiding, heading_proof::Vector measured, heading_proof::Scalar measured_horizontal,
                           heading_proof::Scalar reference_horizontal, heading_proof::Status angle_status,
                           heading_proof::Scalar innovation)
{
    using namespace heading_proof;
    __ESBMC_assert(FORMAL_ESKF_PROOF_HEADING_CALLER, "Runner error: heading caller requires closed producer frames");
    auto const before = heading;
    auto const old_q = q;
    auto const old_body = body;
    auto const old_reference = reference;
    auto const old_parameters = parameters;
    Calls::q = q;
    Calls::body = body;
    Calls::reference = reference;
    Calls::measured = measured;
    Calls::measured_horizontal = measured_horizontal;
    Calls::reference_horizontal = reference_horizontal;
    Calls::innovation = innovation;
    Calls::status = angle_status;
    Calls::rotations = Calls::norms = Calls::headings = 0U;
    Status wanted = input_status(q, body, reference, parameters);
    unsigned rotations = 0U, norms = 0U, headings = 0U;
    auto expected = before.m_state;
    if (wanted == Status::success)
    {
        expected.observable = aiding && before.m_state.horizontal_acceleration > parameters.acceleration_limit;
        expected.innovation_valid = false;
        expected.innovation = Scalar{0};
        if (!aligned)
        {
            expected.filtered_innovation = Scalar{0};
            expected.consistent = false;
        }
        else
        {
            rotations = 1U;
            norms = 2U;
            if (!finite_vector(measured) || !std::isfinite(measured_horizontal) || !std::isfinite(reference_horizontal))
            {
                wanted = Status::non_finite_result;
            }
            else if (measured_horizontal < parameters.minimum_horizontal_field ||
                     reference_horizontal < parameters.minimum_horizontal_field)
            {
                expected.consistent = false;
            }
            else
            {
                headings = 1U;
                wanted = angle_status;
                if (wanted == Status::success)
                {
                    expected.innovation = innovation;
                    expected.innovation_valid = true;
                    constexpr Scalar interval = static_cast<Scalar>(0.01), time_constant = static_cast<Scalar>(0.09);
                    expected.filtered_innovation =
                        before.m_state.filtered_innovation +
                        (innovation - before.m_state.filtered_innovation) * (interval / (time_constant + interval));
                    if (std::fabs(expected.filtered_innovation) >= parameters.heading_noise)
                    {
                        expected.consistent = false;
                    }
                    else if (expected.observable)
                    {
                        expected.consistent = true;
                    }
                }
            }
        }
    }
#if FORMAL_ESKF_PROOF_HEADING_MODE == 1
    __ESBMC_assume(wanted == Status::success);
#elif FORMAL_ESKF_PROOF_HEADING_MODE == 2
    // This premise is closed by the wrapped-heading range producer only under
    // the stated principal-atan2 contract, not by finite scalar returns alone.
    __ESBMC_assume(bounded_heading(before.state()));
    __ESBMC_assume(angle_status != Status::success ||
                   (std::isfinite(innovation) && std::fabs(innovation) <= std::numbers::pi_v<Scalar>));
#endif
    Status const status = heading.update(q, body, reference, parameters, aligned, aiding);
    __ESBMC_assert(
        status == wanted && Calls::rotations == rotations && Calls::norms == norms && Calls::headings == headings,
        "R-MAGNETIC-HEADING: validation, unaligned/weak-field admission and complete producer call priority");
#if FORMAL_ESKF_PROOF_HEADING_MODE == 1
    __ESBMC_assert(
        same_state(heading.state(), expected),
        "R-MAGNETIC-HEADING: full successful heading history, IEEE LPF, strict observability and latch threshold");
#else
    __ESBMC_assert(status == Status::success || same_state(heading.state(), before.state()),
                   "R-MAGNETIC-HEADING: every validation, geometry or angular failure preserves complete state");
#endif
#if FORMAL_ESKF_PROOF_HEADING_MODE == 2
    __ESBMC_assert(
        bounded_heading(heading.state()),
        "R-MAGNETIC-HEADING: principal innovations preserve initialized bounded heading history for every transition");
#endif
    __ESBMC_assert(same_vector(heading.m_acceleration, before.m_acceleration) &&
                       same(heading.state().horizontal_acceleration, before.state().horizontal_acceleration) &&
                       same_vector(q.coefficients(), old_q.coefficients()) && same_vector(body, old_body) &&
                       same_vector(reference, old_reference) && same_parameters(parameters, old_parameters),
                   "R-MAGNETIC-HEADING: heading update preserves acceleration history and all borrowed inputs");
}
int main() { return 0; }
