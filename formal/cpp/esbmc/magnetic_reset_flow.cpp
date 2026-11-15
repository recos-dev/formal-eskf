/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "covariance_oracle.hpp"
#include <formal_eskf/runtime/magnetometer.hpp>
#ifndef FORMAL_ESKF_PROOF_RESET_VALUES
#define FORMAL_ESKF_PROOF_RESET_VALUES 0
#endif
#ifndef FORMAL_ESKF_PROOF_RESET_PUBLIC
#define FORMAL_ESKF_PROOF_RESET_PUBLIC 0
#endif
#ifndef FORMAL_ESKF_PROOF_STATE_SIZE
#define FORMAL_ESKF_PROOF_STATE_SIZE 3
#endif
namespace reset_flow_proof
{
using contract_proof::Linalg;
using contract_proof::OpaqueMath;
using contract_proof::Quaternion;
using contract_proof::same;
using contract_proof::Scalar;
using covariance_oracle::finite_matrix;
using covariance_oracle::same_matrix;
using formal_eskf::Status;
constexpr std::size_t size = FORMAL_ESKF_PROOF_STATE_SIZE;
constexpr std::size_t offset = size == 15U ? 6U : 0U;
static_assert(size == 3U || size == 15U);
using Matrix = formal_eskf::linalg::Matrix<Linalg, size, size>;
using Matrix3 = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
using Vector = contract_proof::Vector3;
using Configuration =
    std::conditional_t<size == 15U, formal_eskf::configuration::Ins, formal_eskf::configuration::Ahrs>;
using State = Configuration::NominalState<Linalg>;
using Geometry = formal_eskf::runtime::detail::MagneticFieldGeometry<Scalar>;
struct Replies
{
    Matrix3 before, after;
    Matrix covariance;
    Vector rotated;
    Geometry measured, reference;
    Quaternion delta, attitude;
    Scalar measured_heading, reference_heading;
    Status measured_status, reference_status, measured_angle_status, reference_angle_status;
    Status exp_status, compose_status, covariance_status;
};
struct Calls
{
    inline static Replies replies;
    inline static Quaternion q;
    inline static Matrix P;
    inline static Vector body, reference, rotated;
    inline static Scalar variance, horizontal, minimum, angle;
    inline static unsigned rotations, products, symmetries, geometries, angles, exps, compositions, covariances,
        helpers;
    inline static Status helper_status;
};
inline bool finite_extra(State const & state)
{
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
    return finite_matrix(state.p_n) && finite_matrix(state.v_n) && finite_matrix(state.b_a) && finite_matrix(state.b_g);
#else
    (void)state;
    return true;
#endif
}
inline bool same_extra(State const & a, State const & b)
{
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
    return same_matrix(a.p_n, b.p_n) && same_matrix(a.v_n, b.v_n) && same_matrix(a.b_a, b.b_a) &&
           same_matrix(a.b_g, b.b_g);
#else
    (void)a;
    (void)b;
    return true;
#endif
}
inline bool same_q(Quaternion const & a, Quaternion const & b)
{
    return same_matrix(a.coefficients(), b.coefficients());
}
inline Status validation(Quaternion const & q, Matrix const & P, Vector const & body, Vector const & reference,
                         Scalar variance, Scalar horizontal, Scalar minimum)
{
    if (!finite_matrix(q.coefficients()) || !finite_matrix(P) || !finite_matrix(body) || !finite_matrix(reference) ||
        !std::isfinite(variance) || !std::isfinite(horizontal) || !std::isfinite(minimum))
    {
        return Status::non_finite_input;
    }
    if (variance < Scalar{0} || horizontal < std::numeric_limits<Scalar>::min() || !(minimum > Scalar{0}) ||
        minimum > Scalar{1})
    {
        return Status::domain_error;
    }
    for (std::size_t i = 0; i < size; ++i)
    {
        if (P(i, i) < Scalar{0})
        {
            return Status::domain_error;
        }
        for (std::size_t j = 0; j < size; ++j)
        {
            // Exact finite symmetry: the scalar subtraction equivalence is
            // independently discharged by verify_navigation_symmetry_pair.
            if (P(i, j) != P(j, i))
            {
                return Status::domain_error;
            }
        }
    }
    return Status::success;
}
inline Scalar wrapped(Scalar difference)
{
    Scalar const pi = std::numbers::pi_v<Scalar>;
    if (difference > pi)
    {
        difference -= Scalar{2} * pi;
    }
    else if (difference < -pi)
    {
        difference += Scalar{2} * pi;
    }
    return difference;
}
inline void prepare(Quaternion const & q, Matrix const & P, Vector const & body, Vector const & reference,
                    Scalar variance, Scalar horizontal, Scalar minimum, Replies const & replies)
{
    Calls::q = q;
    Calls::P = P;
    Calls::body = body;
    Calls::reference = reference;
    Calls::variance = variance;
    Calls::horizontal = horizontal;
    Calls::minimum = minimum;
    Calls::replies = replies;
    Calls::rotated = replies.rotated;
    Calls::angle = wrapped(replies.reference_heading - replies.measured_heading);
    Calls::products = Calls::symmetries = Calls::rotations = Calls::geometries = Calls::angles = Calls::exps =
        Calls::compositions = Calls::covariances = Calls::helpers = 0U;
}
} // namespace reset_flow_proof
#if !FORMAL_ESKF_PROOF_RESET_PUBLIC
// Same-type producers: imu_inhibition.cpp (rotation matrix), magnetic_field.cpp
// (geometry/atan2), maps.cpp (Exp), normalization.cpp (composition), and
// magnetic_reset.cpp / magnetic_reset_ins.cpp (AHRS / INS covariance).
// Replies overapproximate all statuses and values. No successful, finite, unit
// or physically correct result is assumed. F-LINALG closes the 3x3-by-3x1
// product and 3x3/15x15 symmetry operations for this exact Linalg type.
// verify_navigation_symmetry_pair supplies the zero-residual equivalence;
// the caller establishes finite inputs.
namespace formal_eskf::linalg
{
template <>
template <>
inline reset_flow_proof::Vector Matrix<reset_flow_proof::Linalg, 3U, 3U>::operator*
    <1U>(reset_flow_proof::Vector const & v) const
{
    using namespace reset_flow_proof;
    __ESBMC_assert(Calls::products++ == 0U && Calls::rotations == 1U && same_matrix(*this, Calls::replies.before) &&
                       same_matrix(v, Calls::body),
                   "R-MAGNETOMETER-RESET: original body field is multiplied by the original attitude frame");
    return Calls::rotated;
}
template <>
inline bool is_symmetric<reset_flow_proof::Linalg, reset_flow_proof::size>(reset_flow_proof::Matrix const & P,
                                                                           reset_flow_proof::Scalar tolerance)
{
    using namespace reset_flow_proof;
    __ESBMC_assert(Calls::symmetries++ == 0U && finite_matrix(P) && tolerance == Scalar{0} && same_matrix(P, Calls::P),
                   "R-MAGNETOMETER-RESET: caller establishes finite original covariance and zero symmetry tolerance");
    bool equal = true;
    for (std::size_t i = 0U; i < size; ++i)
        for (std::size_t j = 0U; j < size; ++j)
        {
            equal = (P(i, j) == P(j, i)) && equal;
        }
    return equal;
}
} // namespace formal_eskf::linalg
namespace formal_eskf::so3
{
template <>
inline reset_flow_proof::Matrix3 to_rotation_matrix<reset_flow_proof::Linalg>(reset_flow_proof::Quaternion const & q)
{
    using namespace reset_flow_proof;
    __ESBMC_assert(Calls::rotations < 2U, "R-MAGNETOMETER-RESET: bounded rotation requests");
    if (Calls::rotations++ == 0U)
    {
        __ESBMC_assert(same_q(q, Calls::q), "R-MAGNETOMETER-RESET: original attitude supplies measured field frame");
        return Calls::replies.before;
    }
    __ESBMC_assert(Calls::compositions == 1U && Calls::replies.compose_status == Status::success &&
                       same_q(q, Calls::replies.attitude),
                   "R-MAGNETOMETER-RESET: new attitude supplies covariance frame only after successful composition");
    return Calls::replies.after;
}
template <>
inline Status exp<reset_flow_proof::Linalg>(reset_flow_proof::Vector const & v, reset_flow_proof::Scalar minimum,
                                            reset_flow_proof::Quaternion & output)
{
    using namespace reset_flow_proof;
    __ESBMC_assert(Calls::exps++ == 0U && Calls::angles == 2U && same(v(0U), Scalar{0}) && same(v(1U), Scalar{0}) &&
                       same(minimum, Calls::minimum),
                   "R-MAGNETOMETER-RESET: Down axis and original threshold reach Exp");
#if FORMAL_ESKF_PROOF_RESET_VALUES
    __ESBMC_assert(same(v(2U), Calls::angle),
                   "R-MAGNETOMETER-RESET: exact wrapped reference-minus-measured angle reaches Exp");
#endif
    if (Calls::replies.exp_status == Status::success)
    {
        output = Calls::replies.delta;
    }
    return Calls::replies.exp_status;
}
template <>
inline Status compose_normalized<reset_flow_proof::Linalg>(reset_flow_proof::Quaternion const & a,
                                                           reset_flow_proof::Quaternion const & b,
                                                           reset_flow_proof::Scalar minimum,
                                                           reset_flow_proof::Quaternion & output)
{
    using namespace reset_flow_proof;
    __ESBMC_assert(Calls::compositions++ == 0U && Calls::exps == 1U && Calls::replies.exp_status == Status::success &&
                       same_q(a, Calls::replies.delta) && same_q(b, Calls::q) && same(minimum, Calls::minimum),
                   "R-MAGNETOMETER-RESET: successful navigation Down delta left-multiplies original attitude");
    if (Calls::replies.compose_status == Status::success)
    {
        output = Calls::replies.attitude;
    }
    return Calls::replies.compose_status;
}
} // namespace formal_eskf::so3
namespace formal_eskf::runtime::detail
{
template <>
inline Status magnetic_field_geometry<reset_flow_proof::Linalg>(reset_flow_proof::Vector const & v,
                                                                reset_flow_proof::Geometry & output)
{
    using namespace reset_flow_proof;
    __ESBMC_assert(Calls::rotations == 1U && Calls::geometries < 2U && Calls::angles == 0U,
                   "R-MAGNETOMETER-RESET: measured then reference geometry precedes heading");
    if (Calls::geometries++ == 0U)
    {
        __ESBMC_assert(same_matrix(v, Calls::rotated),
                       "R-MAGNETOMETER-RESET: measured geometry uses rotated body field");
#if FORMAL_ESKF_PROOF_RESET_VALUES
        __ESBMC_assert(Calls::replies.measured_status == Status::success,
                       "R-MAGNETOMETER-RESET: numeric profile establishes producer success");
        output = Calls::replies.measured;
        return Status::success;
#else
        if (Calls::replies.measured_status == Status::success)
        {
            output = Calls::replies.measured;
        }
        return Calls::replies.measured_status;
#endif
    }
    __ESBMC_assert(same_matrix(v, Calls::reference),
                   "R-MAGNETOMETER-RESET: reference geometry uses original NED field");
#if FORMAL_ESKF_PROOF_RESET_VALUES
    __ESBMC_assert(Calls::replies.reference_status == Status::success,
                   "R-MAGNETOMETER-RESET: numeric profile establishes producer success");
    output = Calls::replies.reference;
    return Status::success;
#else
    if (Calls::replies.reference_status == Status::success)
    {
        output = Calls::replies.reference;
    }
    return Calls::replies.reference_status;
#endif
}
template <>
inline Status reset_heading_covariance<reset_flow_proof::offset, reset_flow_proof::Linalg, reset_flow_proof::size>(
    reset_flow_proof::Matrix const & P, reset_flow_proof::Matrix3 const & before,
    reset_flow_proof::Matrix3 const & after, reset_flow_proof::Scalar variance, reset_flow_proof::Matrix & output)
{
    using namespace reset_flow_proof;
    __ESBMC_assert(Calls::covariances++ == 0U && Calls::rotations == 2U && same_matrix(P, Calls::P) &&
                       same_matrix(before, Calls::replies.before) && same_matrix(after, Calls::replies.after) &&
                       same(variance, Calls::variance),
                   "R-MAGNETOMETER-RESET: original covariance, old/new frames and yaw variance reach covariance reset");
    if (Calls::replies.covariance_status == Status::success)
    {
        output = Calls::replies.covariance;
    }
    return Calls::replies.covariance_status;
}
} // namespace formal_eskf::runtime::detail
namespace formal_eskf::scalar
{
template <>
inline Status atan2<reset_flow_proof::OpaqueMath>(reset_flow_proof::Scalar y, reset_flow_proof::Scalar x,
                                                  reset_flow_proof::Scalar & output)
{
    using namespace reset_flow_proof;
    __ESBMC_assert(Calls::angles < 2U && Calls::geometries == 2U && Calls::exps == 0U,
                   "R-MAGNETOMETER-RESET: two headings follow geometry");
    if (Calls::angles++ == 0U)
    {
        __ESBMC_assert(same(y, Calls::rotated(1U)) && same(x, Calls::rotated(0U)),
                       "R-MAGNETOMETER-RESET: measured atan2 y,x order");
#if FORMAL_ESKF_PROOF_RESET_VALUES
        __ESBMC_assert(Calls::replies.measured_angle_status == Status::success,
                       "R-MAGNETOMETER-RESET: numeric profile establishes producer success");
        output = Calls::replies.measured_heading;
        return Status::success;
#else
        if (Calls::replies.measured_angle_status == Status::success)
        {
            output = Calls::replies.measured_heading;
        }
        return Calls::replies.measured_angle_status;
#endif
    }
    __ESBMC_assert(same(y, Calls::reference(1U)) && same(x, Calls::reference(0U)),
                   "R-MAGNETOMETER-RESET: reference atan2 y,x order");
#if FORMAL_ESKF_PROOF_RESET_VALUES
    __ESBMC_assert(Calls::replies.reference_angle_status == Status::success,
                   "R-MAGNETOMETER-RESET: numeric profile establishes producer success");
    output = Calls::replies.reference_heading;
    return Status::success;
#else
    if (Calls::replies.reference_angle_status == Status::success)
    {
        output = Calls::replies.reference_heading;
    }
    return Calls::replies.reference_angle_status;
#endif
}
} // namespace formal_eskf::scalar
#else
namespace formal_eskf::runtime::detail
{
template <>
inline Status
compute_magnetic_heading_reset<reset_flow_proof::offset, reset_flow_proof::Linalg, reset_flow_proof::size>(
    reset_flow_proof::Quaternion const & q, reset_flow_proof::Matrix const & P, reset_flow_proof::Vector const & body,
    reset_flow_proof::Vector const & reference, reset_flow_proof::Scalar variance, reset_flow_proof::Scalar horizontal,
    reset_flow_proof::Scalar minimum, reset_flow_proof::Quaternion & attitude, reset_flow_proof::Matrix & covariance,
    reset_flow_proof::Quaternion & delta)
{
    using namespace reset_flow_proof;
    __ESBMC_assert(Calls::helpers++ == 0U && same_q(q, Calls::q) && same_matrix(P, Calls::P) &&
                       same_matrix(body, Calls::body) && same_matrix(reference, Calls::reference) &&
                       same(variance, Calls::variance) && same(horizontal, Calls::horizontal) &&
                       same(minimum, Calls::minimum),
                   "R-MAGNETOMETER-RESET: public wrapper passes the complete original reset request once");
    // The actual helper allows partial scratch outputs on failure. Write all
    // scratch outputs for every status, a deliberate overapproximation.
    attitude = Calls::replies.attitude;
    covariance = Calls::replies.covariance;
    delta = Calls::replies.delta;
    return Calls::helper_status;
}
} // namespace formal_eskf::runtime::detail
#endif
#if !FORMAL_ESKF_PROOF_RESET_PUBLIC
void verify_magnetic_reset_flow(reset_flow_proof::Quaternion q, reset_flow_proof::Matrix P,
                                reset_flow_proof::Vector body, reset_flow_proof::Vector reference,
                                reset_flow_proof::Scalar variance, reset_flow_proof::Scalar horizontal,
                                reset_flow_proof::Scalar minimum, reset_flow_proof::Quaternion attitude,
                                reset_flow_proof::Matrix covariance, reset_flow_proof::Quaternion delta,
                                reset_flow_proof::Replies replies)
{
    using namespace reset_flow_proof;
    auto const old_q = q, old_attitude = attitude, old_delta = delta;
    auto const old_P = P, old_covariance = covariance;
    auto const old_body = body, old_reference = reference;
    prepare(q, P, body, reference, variance, horizontal, minimum, replies);
    Status wanted = validation(q, P, body, reference, variance, horizontal, minimum);
#if FORMAL_ESKF_PROOF_RESET_VALUES
    // Complementary to the unconditional flow/frame profile. Producer-success
    // premises describe the path reaching the wrapped-angle/Exp request.
    __ESBMC_assume(wanted == Status::success && finite_matrix(replies.rotated) &&
                   replies.measured_status == Status::success && replies.reference_status == Status::success &&
                   replies.measured.horizontal >= horizontal && replies.reference.horizontal >= horizontal &&
                   replies.measured_angle_status == Status::success &&
                   replies.reference_angle_status == Status::success && std::isfinite(replies.measured_heading) &&
                   std::isfinite(replies.reference_heading));
#endif

    unsigned rotations = 0U, geometries = 0U, angles = 0U, exps = 0U, compositions = 0U, covariances = 0U;
    if (wanted == Status::success)
    {
        rotations = 1U;
        if (!finite_matrix(Calls::rotated))
        {
            wanted = Status::non_finite_result;
        }
        else
        {
            geometries = 1U;
            wanted = replies.measured_status;
        }
    }
    if (wanted == Status::success)
    {
        geometries = 2U;
        wanted = replies.reference_status;
    }
    if (wanted == Status::success &&
        (replies.measured.horizontal < horizontal || replies.reference.horizontal < horizontal))
    {
        wanted = Status::domain_error;
    }
    if (wanted == Status::success)
    {
        angles = 1U;
        wanted = replies.measured_angle_status;
    }
    if (wanted == Status::success)
    {
        angles = 2U;
        wanted = replies.reference_angle_status;
    }
    if (wanted == Status::success)
    {
        exps = 1U;
        wanted = replies.exp_status;
    }
    if (wanted == Status::success)
    {
        compositions = 1U;
        wanted = replies.compose_status;
    }
    if (wanted == Status::success)
    {
        rotations = 2U;
        covariances = 1U;
        wanted = replies.covariance_status;
    }
    auto const status = formal_eskf::runtime::detail::compute_magnetic_heading_reset<offset>(
        q, P, body, reference, variance, horizontal, minimum, attitude, covariance, delta);
    __ESBMC_assert(status == wanted && Calls::rotations == rotations && Calls::products == (rotations ? 1U : 0U) &&
                       Calls::geometries == geometries && Calls::angles == angles && Calls::exps == exps &&
                       Calls::compositions == compositions && Calls::covariances == covariances,
                   "R-MAGNETOMETER-RESET: complete validation, threshold, callee failure priority and stop order");
    if (exps && replies.exp_status == Status::success)
    {
        __ESBMC_assert(same_q(delta, replies.delta), "R-MAGNETOMETER-RESET: successful Exp scratch delta");
    }
    else
    {
        __ESBMC_assert(same_q(delta, old_delta), "R-MAGNETOMETER-RESET: untouched delta until successful Exp");
    }
    if (compositions && replies.compose_status == Status::success)
    {
        __ESBMC_assert(same_q(attitude, replies.attitude),
                       "R-MAGNETOMETER-RESET: successful composition scratch attitude");
    }
    else
    {
        __ESBMC_assert(same_q(attitude, old_attitude),
                       "R-MAGNETOMETER-RESET: untouched attitude until successful composition");
    }
    if (covariances && replies.covariance_status == Status::success)
    {
        __ESBMC_assert(same_matrix(covariance, replies.covariance),
                       "R-MAGNETOMETER-RESET: successful covariance scratch output");
    }
    else
    {
        __ESBMC_assert(same_matrix(covariance, old_covariance),
                       "R-MAGNETOMETER-RESET: failed covariance scratch unchanged");
    }
    __ESBMC_assert(same_q(q, old_q) && same_matrix(P, old_P) && same_matrix(body, old_body) &&
                       same_matrix(reference, old_reference),
                   "R-MAGNETOMETER-RESET: helper preserves every borrowed input on all paths");
}
#else
void verify_magnetic_reset_transaction(reset_flow_proof::State state, reset_flow_proof::Matrix P,
                                       reset_flow_proof::Vector body, reset_flow_proof::Vector reference,
                                       reset_flow_proof::Scalar variance, reset_flow_proof::Scalar horizontal,
                                       reset_flow_proof::Scalar minimum, reset_flow_proof::State output,
                                       reset_flow_proof::Matrix covariance, reset_flow_proof::Quaternion delta,
                                       reset_flow_proof::Replies replies, reset_flow_proof::Status requested)
{
    using namespace reset_flow_proof;
    static_assert(FORMAL_ESKF_PROOF_ALIAS < 4);
    State * state_target = &output;
    Matrix * covariance_target = &covariance;
    if constexpr (FORMAL_ESKF_PROOF_ALIAS & 1U)
    {
        state_target = &state;
    }
    if constexpr (FORMAL_ESKF_PROOF_ALIAS & 2U)
    {
        covariance_target = &P;
    }
    State & target = *state_target;
    Matrix & target_P = *covariance_target;
    auto const old_state = state, old_output = target;
    auto const old_P = P, old_covariance = target_P;
    auto const old_body = body, old_reference = reference;
    auto const old_delta = delta;
    prepare(state.q_nb, P, body, reference, variance, horizontal, minimum, replies);
    Calls::helper_status = requested;
    auto const status = formal_eskf::runtime::reset_magnetic_heading(state, P, body, reference, variance, horizontal,
                                                                     minimum, target, target_P, delta);
    __ESBMC_assert(status == (finite_extra(old_state) ? requested : Status::non_finite_input) &&
                       Calls::helpers == unsigned(finite_extra(old_state)),
                   "R-MAGNETOMETER-RESET: helper status propagates exactly");
    if (status == Status::success)
    {
        __ESBMC_assert(same_extra(target, old_state) && same_q(target.q_nb, replies.attitude) &&
                           same_matrix(target_P, replies.covariance) && same_q(delta, replies.delta),
                       "R-MAGNETOMETER-RESET: all three successful outputs publish together, including prior aliases");
    }
    else
    {
        __ESBMC_assert(same_extra(target, old_output) && same_q(target.q_nb, old_output.q_nb) &&
                           same_matrix(target_P, old_covariance) && same_q(delta, old_delta),
                       "R-MAGNETOMETER-RESET: failed helper scratch never leaks to any output or prior alias");
    }
    if constexpr (!(FORMAL_ESKF_PROOF_ALIAS & 1))
    {
        __ESBMC_assert(same_extra(state, old_state) && same_q(state.q_nb, old_state.q_nb),
                       "R-MAGNETOMETER-RESET: separate state prior unchanged");
    }
    if constexpr (!(FORMAL_ESKF_PROOF_ALIAS & 2))
    {
        __ESBMC_assert(same_matrix(P, old_P), "R-MAGNETOMETER-RESET: separate covariance prior unchanged");
    }
    __ESBMC_assert(same_matrix(body, old_body) && same_matrix(reference, old_reference),
                   "R-MAGNETOMETER-RESET: public field inputs unchanged");
}
#endif
int main() { return 0; }
