/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "bias_support.hpp"
#include "step_correction_support.hpp"
#include <formal_eskf/runtime/magnetometer.hpp>

namespace magnetic_fusion_proof
{
using correction_proof::finite_matrix;
using step_correction_proof::Backend;
using step_correction_proof::boundaries_clear;
using step_correction_proof::Covariance;
using step_correction_proof::Jacobian;
using step_correction_proof::measurement_size;
using step_correction_proof::Noise;
using step_correction_proof::Quaternion;
using step_correction_proof::same;
using step_correction_proof::same_matrix;
using step_correction_proof::same_state;
using step_correction_proof::Scalar;
using step_correction_proof::State;
using step_correction_proof::state_size;
using step_correction_proof::Status;
using step_correction_proof::Vector3;
using Vector = Vector3;
using Permissions = bias_proof::permissions_type;
using Parameters = formal_eskf::runtime::MagneticFieldParameters<Scalar>;
using Checks = formal_eskf::runtime::MagneticFieldChecks;
using Field = formal_eskf::runtime::MagneticFieldCheck<Scalar>;
using Check = formal_eskf::runtime::MeasurementCheck<Backend, 3U>;
using Result = formal_eskf::runtime::MagnetometerFusionResult<Backend>;
using Decision = formal_eskf::runtime::FusionDecision;
static_assert(boundaries_clear && measurement_size == 3U && FORMAL_ESKF_PROOF_STEP_ALIAS < 4);
struct Replies
{
    Field field;
    Vector predicted, residual;
    Status observation_status;
    Check gate;
    State state;
    Covariance covariance;
    Status correction_status;
};
struct Calls
{
    inline static State const *prior{}, *state_before{};
    inline static Covariance const *P{}, *P_before{};
    inline static State * state_output{};
    inline static Covariance * P_output{};
    inline static Vector const *body{}, *reference{}, *health{};
    inline static Noise const * V{};
    inline static Parameters const * parameters{};
    inline static Checks const * checks{};
    inline static Permissions const * permissions{};
    inline static Replies const * replies{};
    inline static Scalar minimum{}, sigma{};
    inline static bool observable{};
    inline static unsigned observations{}, gates{}, corrections{};
};
inline bool same_field(Field const & a, Field const & b)
{
    return a.status == b.status && a.accepted == b.accepted && a.diagnostics_valid == b.diagnostics_valid &&
           a.strength_checked == b.strength_checked && a.inclination_checked == b.inclination_checked &&
           a.heading_checked == b.heading_checked && a.failures.reference == b.failures.reference &&
           a.failures.strength == b.failures.strength && a.failures.inclination == b.failures.inclination &&
           a.failures.horizontal_field == b.failures.horizontal_field && a.failures.heading == b.failures.heading &&
           same(a.strength, b.strength) && same(a.reference_strength, b.reference_strength) &&
           same(a.inclination_error, b.inclination_error) && same(a.heading_error, b.heading_error);
}
inline bool same_parameters(Parameters const & a, Parameters const & b)
{
    return same(a.strength_tolerance, b.strength_tolerance) && same(a.inclination_tolerance, b.inclination_tolerance) &&
           same(a.heading_tolerance, b.heading_tolerance) &&
           same(a.minimum_horizontal_field, b.minimum_horizontal_field);
}
inline bool same_checks(Checks const & a, Checks const & b)
{
    return a.strength == b.strength && a.inclination == b.inclination && a.require_reference == b.require_reference;
}
inline bool finite_state(State const & state)
{
    bool valid = finite_matrix(state.q_nb.coefficients());
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
    valid = finite_matrix(state.p_n) && finite_matrix(state.v_n) && finite_matrix(state.b_a) &&
            finite_matrix(state.b_g) && valid;
#endif
    return valid;
}
inline void unpublished()
{
    __ESBMC_assert(same_state(*Calls::state_output, *Calls::state_before) &&
                       same_matrix(*Calls::P_output, *Calls::P_before),
                   "R-MAGNETOMETER-FUSION: outputs remain unchanged before successful correction");
}
inline bool jacobian_matches(Jacobian const & H)
{
    auto const & h = Calls::replies->predicted;
    Scalar block[9]{0, -h(2U), h(1U), h(2U), 0, -h(0U), -h(1U), h(0U), 0};
    constexpr std::size_t offset = state_size == 15U ? 6U : 0U;
    bool valid = true;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < state_size; ++j)
        {
            Scalar const wanted = j >= offset && j < offset + 3U ? block[3U * i + j - offset] : Scalar{0};
            valid = same(H(i, j), wanted) && valid;
        }
    return valid;
}
inline Result observation_result()
{
    Result result;
    result.field = Calls::replies->field;
    result.fusion.status = result.field.status;
    if (result.field.status == Status::success)
    {
        if (!result.field.accepted)
            result.fusion.decision = Decision::rejected;
        else
            result.fusion.status = Calls::replies->observation_status;
    }
    return result;
}
} // namespace magnetic_fusion_proof

// Actual same-type producers: magnetic observation, measurement checks and
// M0/M1 magnetic correction. Outputs/statuses remain arbitrary except for
// their established non-fused observation and atomic correction frames.
namespace formal_eskf::runtime::detail
{
template <>
inline magnetic_fusion_proof::Result check_magnetic_observation<magnetic_fusion_proof::Backend>(
    magnetic_fusion_proof::Quaternion const & q, magnetic_fusion_proof::Vector const & body,
    magnetic_fusion_proof::Vector const & reference, magnetic_fusion_proof::Vector const * health,
    magnetic_fusion_proof::Parameters const & parameters, bool observable, magnetic_fusion_proof::Checks const & checks,
    magnetic_fusion_proof::Vector & predicted, magnetic_fusion_proof::Vector & residual)
{
    using namespace magnetic_fusion_proof;
    __ESBMC_assert(Calls::observations++ == 0U && Calls::gates == 0U && Calls::corrections == 0U &&
                       finite_state(*Calls::prior),
                   "R-MAGNETOMETER-FUSION: finite prior precedes exactly one observation stage");
    __ESBMC_assert(same_matrix(q.coefficients(), Calls::prior->q_nb.coefficients()) &&
                       same_matrix(body, *Calls::body) && same_matrix(reference, *Calls::reference) &&
                       health == Calls::health && same_parameters(parameters, *Calls::parameters) &&
                       same_checks(checks, *Calls::checks) && observable == Calls::observable,
                   "R-MAGNETOMETER-FUSION: observation receives original prior, distinct references and policy");
    unpublished();
    predicted = Calls::replies->predicted;
    residual = Calls::replies->residual;
    return observation_result();
}
} // namespace formal_eskf::runtime::detail
namespace formal_eskf::runtime
{
template <>
inline magnetic_fusion_proof::Check check_measurement<magnetic_fusion_proof::Backend, 3U>(
    magnetic_fusion_proof::State const & state, magnetic_fusion_proof::Covariance const & P,
    magnetic_fusion_proof::Vector const & r, magnetic_fusion_proof::Jacobian const & H,
    magnetic_fusion_proof::Noise const & V, magnetic_fusion_proof::Scalar minimum, magnetic_fusion_proof::Scalar sigma)
{
    using namespace magnetic_fusion_proof;
    __ESBMC_assert(Calls::observations == 1U && Calls::gates++ == 0U && Calls::corrections == 0U &&
                       observation_result().fusion.status == Status::success && Calls::replies->field.accepted,
                   "R-MAGNETOMETER-FUSION: gate follows accepted observation exactly once");
    __ESBMC_assert(same_state(state, *Calls::prior) && same_matrix(P, *Calls::P) &&
                       same_matrix(r, Calls::replies->residual) && jacobian_matches(H) && finite_matrix(H) &&
                       same_matrix(V, *Calls::V) && same(minimum, Calls::minimum) && same(sigma, Calls::sigma),
                   "R-MAGNETOMETER-FUSION: gate uses original prior, full V, exact attitude Jacobian and residual");
    unpublished();
    return Calls::replies->gate;
}
} // namespace formal_eskf::runtime
namespace formal_eskf
{
template <>
inline Status correct_magnetometer<magnetic_fusion_proof::Backend>(
    magnetic_fusion_proof::State const & state, magnetic_fusion_proof::Covariance const & P,
    magnetic_fusion_proof::Vector const & body, magnetic_fusion_proof::Vector const & reference,
    magnetic_fusion_proof::Noise const & V, magnetic_fusion_proof::Scalar minimum,
    magnetic_fusion_proof::State & output, magnetic_fusion_proof::Covariance & covariance
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
    ,
    configuration::Ins::BiasUpdate const & permissions
#endif
)
{
    using namespace magnetic_fusion_proof;
    __ESBMC_assert(Calls::observations == 1U && Calls::gates == 1U && Calls::corrections++ == 0U &&
                       Calls::replies->gate.passed(),
                   "R-MAGNETOMETER-FUSION: only a successful non-rejected gate reaches correction");
    __ESBMC_assert(
        same_state(state, *Calls::prior) && same_matrix(P, *Calls::P) && same_matrix(body, *Calls::body) &&
            same_matrix(reference, *Calls::reference) && same_matrix(V, *Calls::V) && same(minimum, Calls::minimum) &&
            &output == Calls::state_output && &covariance == Calls::P_output,
        "R-MAGNETOMETER-FUSION: correction uses unchanged raw fields/prior/noise and original output aliases");
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
    __ESBMC_assert(bias_proof::same_permissions(permissions, *Calls::permissions),
                   "R-MAGNETOMETER-FUSION: all six INS bias permissions reach correction");
#endif
    unpublished();
    if (Calls::replies->correction_status == Status::success)
    {
        output = Calls::replies->state;
        covariance = Calls::replies->covariance;
    }
    return Calls::replies->correction_status;
}
} // namespace formal_eskf
void verify_magnetic_fusion(magnetic_fusion_proof::State state, magnetic_fusion_proof::Covariance P,
                            magnetic_fusion_proof::Vector body, magnetic_fusion_proof::Vector reference,
                            magnetic_fusion_proof::Vector health, unsigned health_source,
                            magnetic_fusion_proof::Noise V, magnetic_fusion_proof::Parameters parameters,
                            magnetic_fusion_proof::Checks checks, bool observable,
                            magnetic_fusion_proof::Scalar minimum, magnetic_fusion_proof::Scalar sigma,
                            magnetic_fusion_proof::Permissions permissions, magnetic_fusion_proof::State state_output,
                            magnetic_fusion_proof::Covariance P_output, magnetic_fusion_proof::Replies replies)
{
    using namespace magnetic_fusion_proof;
    solve_proof::Math::observe_pivots = false;
    __ESBMC_assume(health_source < 4U);
    Vector const * selected = health_source == 0U   ? nullptr
                              : health_source == 1U ? &health
                              : health_source == 2U ? &body
                                                    : &reference;
    auto const old_state = state;
    auto const old_P = P;
    auto const old_body = body, old_reference = reference, old_health = health;
    auto const old_V = V;
    auto const old_parameters = parameters;
    auto const old_checks = checks;
    auto const old_permissions = permissions;
    auto const old_state_output = state_output;
    auto const old_P_output = P_output;
    State * state_target = &state_output;
    Covariance * P_target = &P_output;
    if constexpr (FORMAL_ESKF_PROOF_STEP_ALIAS & 1U)
        state_target = &state;
    if constexpr (FORMAL_ESKF_PROOF_STEP_ALIAS & 2U)
        P_target = &P;
    auto const before_state = *state_target;
    auto const before_P = *P_target;
    Calls::prior = &old_state;
    Calls::P = &old_P;
    Calls::body = &old_body;
    Calls::reference = &old_reference;
    Calls::health = selected;
    Calls::V = &old_V;
    Calls::parameters = &old_parameters;
    Calls::checks = &old_checks;
    Calls::permissions = &old_permissions;
    Calls::state_before = &before_state;
    Calls::P_before = &before_P;
    Calls::state_output = state_target;
    Calls::P_output = P_target;
    Calls::replies = &replies;
    Calls::minimum = minimum;
    Calls::sigma = sigma;
    Calls::observable = observable;
    Calls::observations = Calls::gates = Calls::corrections = 0;
    bool const finite = finite_state(old_state);
    Result expected;
    if (!finite)
        expected.fusion.status = Status::non_finite_input;
    else
        expected = observation_result();
    bool const qualified = finite && expected.fusion.status == Status::success && replies.field.accepted;
    bool const finite_H = finite_matrix(replies.predicted);
    bool const gated = qualified && finite_H;
    bool const corrected =
        gated && replies.gate.status == Status::success && replies.gate.diagnostics_valid && !replies.gate.rejected;
    bool const fused = corrected && replies.correction_status == Status::success;
    if (qualified && !finite_H)
        expected.fusion.status = Status::non_finite_result;
    if (gated)
    {
        expected.fusion.status = corrected ? replies.correction_status : replies.gate.status;
        expected.fusion.diagnostics_valid = replies.gate.diagnostics_valid;
        expected.fusion.innovation = replies.gate.innovation;
        expected.fusion.innovation_variance = replies.gate.innovation_variance;
        expected.fusion.test_ratio = replies.gate.test_ratio;
        expected.fusion.decision =
            fused ? Decision::fused
            : replies.gate.status == Status::success && replies.gate.diagnostics_valid && replies.gate.rejected
                ? Decision::rejected
                : Decision::failed;
    }
    auto const actual = formal_eskf::runtime::fuse_magnetometer(state, P, body, reference, selected, V, parameters,
                                                                observable, minimum, sigma, *state_target, *P_target
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
                                                                ,
                                                                permissions
#endif
                                                                ,
                                                                checks);
    __ESBMC_assert(
        Calls::observations == unsigned(finite) && Calls::gates == unsigned(gated) &&
            Calls::corrections == unsigned(corrected),
        "R-MAGNETOMETER-FUSION: complete validation, field-stop, finite-Jacobian and gate/correction call ordering");
    __ESBMC_assert(same_field(actual.field, expected.field) && actual.fusion.status == expected.fusion.status &&
                       actual.fusion.decision == expected.fusion.decision &&
                       actual.fusion.diagnostics_valid == expected.fusion.diagnostics_valid &&
                       same_matrix(actual.fusion.innovation, expected.fusion.innovation) &&
                       same_matrix(actual.fusion.innovation_variance, expected.fusion.innovation_variance) &&
                       same_matrix(actual.fusion.test_ratio, expected.fusion.test_ratio),
                   "R-MAGNETOMETER-FUSION: exact field/gate diagnostics, error priority and fusion verdict");
    __ESBMC_assert(fused ? (same_state(*state_target, replies.state) && same_matrix(*P_target, replies.covariance))
                         : (same_state(*state_target, before_state) && same_matrix(*P_target, before_P)),
                   "R-MAGNETOMETER-FUSION: only successful correction publishes both outputs for all aliases");
    if constexpr (!(FORMAL_ESKF_PROOF_STEP_ALIAS & 1U))
        __ESBMC_assert(same_state(state, old_state), "magnetic fusion: separate state input frame");
    else
        __ESBMC_assert(same_state(state_output, old_state_output), "magnetic fusion: unused state output frame");
    if constexpr (!(FORMAL_ESKF_PROOF_STEP_ALIAS & 2U))
        __ESBMC_assert(same_matrix(P, old_P), "magnetic fusion: separate covariance input frame");
    else
        __ESBMC_assert(same_matrix(P_output, old_P_output), "magnetic fusion: unused covariance output frame");
    __ESBMC_assert(same_matrix(body, old_body) && same_matrix(reference, old_reference) &&
                       same_matrix(health, old_health) && same_matrix(V, old_V) &&
                       same_parameters(parameters, old_parameters) && same_checks(checks, old_checks) &&
                       bias_proof::same_permissions(permissions, old_permissions),
                   "R-MAGNETOMETER-FUSION: measurements, both references, full noise and policies are unchanged");
}
int main() { return 0; }
