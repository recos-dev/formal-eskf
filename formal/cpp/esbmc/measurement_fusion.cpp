/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bias_support.hpp"
#include "step_correction_support.hpp"
#include <formal_eskf/runtime/measurement_fusion.hpp>

namespace fusion_proof
{
using namespace step_correction_proof;
using correction_proof::Gain;
using Gate = formal_eskf::InnovationCheck<Backend, measurement_size>;
using Check = formal_eskf::runtime::MeasurementCheck<Backend, measurement_size>;
using Result = formal_eskf::runtime::FusionResult<Backend, measurement_size>;
using Decision = formal_eskf::runtime::FusionDecision;
static_assert(boundaries_clear && measurement_size <= 3U && FORMAL_ESKF_PROOF_STEP_ALIAS >= 0 &&
              FORMAL_ESKF_PROOF_STEP_ALIAS < 4);
struct Results
{
    Gain PHt;
    Noise S;
    Gate gate;
    State state;
    Covariance covariance;
    Status covariance_status, gate_status, correction_status;
};
struct Calls
{
    inline static Results const * results = nullptr;
    inline static State const * state = nullptr;
    inline static Covariance const * covariance = nullptr;
    inline static Residual const * residual = nullptr;
    inline static Jacobian const * H = nullptr;
    inline static Noise const * V = nullptr;
    inline static State * state_target = nullptr;
    inline static Covariance * covariance_target = nullptr;
    inline static State const * state_before = nullptr;
    inline static Covariance const * covariance_before = nullptr;
    inline static bias_proof::permissions_type const * permissions = nullptr;
    inline static Scalar minimum{}, sigma{};
    inline static unsigned preparation{}, gate{}, correction{};
};
void unpublished()
{
    __ESBMC_assert(same_state(*Calls::state_target, *Calls::state_before) &&
                       same_matrix(*Calls::covariance_target, *Calls::covariance_before),
                   "R-MEASUREMENT-FUSION: outputs are unchanged before successful "
                   "correction");
}
bool finite_state(State const & state)
{
    bool finite = correction_proof::finite_matrix(state.q_nb.coefficients());
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
    finite = correction_proof::finite_matrix(state.p_n) && correction_proof::finite_matrix(state.v_n) &&
             correction_proof::finite_matrix(state.b_a) && correction_proof::finite_matrix(state.b_g) && finite;
#endif
    return finite;
}
} // namespace fusion_proof

// Same-backend producer dependencies: correction_covariances frame/equations,
// E-INNOV's actual body, then E-STEP-CORRECT/E-BIAS-UPDATE. Summaries use no
// finite, positive-definite, unit-prior or successful-input assumptions.
namespace formal_eskf
{
template <>
inline Status correction_covariances<fusion_proof::Backend, fusion_proof::state_size, fusion_proof::measurement_size>(
    fusion_proof::Covariance const & P, fusion_proof::Residual const & r, fusion_proof::Jacobian const & H,
    fusion_proof::Noise const & V, fusion_proof::Scalar minimum, fusion_proof::Gain & PHt, fusion_proof::Noise & S)
{
    using namespace fusion_proof;
    __ESBMC_assert(Calls::preparation++ == 0U && Calls::gate == 0U && Calls::correction == 0U &&
                       finite_state(*Calls::state) && std::isfinite(Calls::sigma),
                   "R-MEASUREMENT-FUSION: finite state/sigma precede covariance "
                   "preparation exactly once");
    __ESBMC_assert(same_matrix(P, *Calls::covariance) && same_matrix(r, *Calls::residual) &&
                       same_matrix(H, *Calls::H) && same_matrix(V, *Calls::V) && same(minimum, Calls::minimum),
                   "R-MEASUREMENT-FUSION: covariance preparation receives the "
                   "complete original prior and observation");
    __ESBMC_assert(static_cast<void const *>(&PHt) != &S && static_cast<void const *>(&PHt) != &P &&
                       static_cast<void const *>(&S) != &P && &S != &V &&
                       static_cast<void const *>(&PHt) != Calls::covariance_target &&
                       static_cast<void const *>(&S) != Calls::covariance_target,
                   "R-MEASUREMENT-FUSION: covariance preparation uses disjoint "
                   "scratch outputs");
    __ESBMC_assert(static_cast<void const *>(&PHt) != &r && static_cast<void const *>(&PHt) != &H &&
                       static_cast<void const *>(&PHt) != &V && static_cast<void const *>(&S) != &r &&
                       static_cast<void const *>(&S) != &H,
                   "R-MEASUREMENT-FUSION: preparation outputs are disjoint from "
                   "every observation input");
    unpublished();
    // A stronger producer is atomic; admitting scratch writes on failure tests
    // that the runtime caller never consumes failed preparation results.
    PHt = Calls::results->PHt;
    S = Calls::results->S;
    return Calls::results->covariance_status;
}
template <>
inline Status check_innovation<fusion_proof::Backend, fusion_proof::measurement_size>(
    fusion_proof::Residual const & r, fusion_proof::Residual const & variance, fusion_proof::Scalar sigma,
    fusion_proof::Gate & output)
{
    using namespace fusion_proof;
    __ESBMC_assert(Calls::preparation == 1U && Calls::results->covariance_status == Status::success &&
                       Calls::gate++ == 0U && Calls::correction == 0U,
                   "R-MEASUREMENT-FUSION: gate follows only successful "
                   "covariance preparation");
    __ESBMC_assert(same_matrix(r, *Calls::residual) && same(sigma, Calls::sigma),
                   "R-MEASUREMENT-FUSION: gate receives unchanged residual and "
                   "explicit sigma");
    for (std::size_t i = 0U; i < measurement_size; ++i)
    {
        __ESBMC_assert(same(variance(i), Calls::results->S(i, i)),
                       "R-MEASUREMENT-FUSION: gate uses diag(S), never measurement "
                       "covariance diag(V)");
    }
    unpublished();
    output = Calls::results->gate;
    return Calls::results->gate_status;
}
template <>
inline Status correct<fusion_proof::Backend, fusion_proof::measurement_size>(
    fusion_proof::State const & state, fusion_proof::Covariance const & P, fusion_proof::Residual const & r,
    fusion_proof::Jacobian const & H, fusion_proof::Noise const & V, fusion_proof::Scalar minimum,
    fusion_proof::State & state_output, fusion_proof::Covariance & covariance_output
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
    ,
    configuration::Ins::BiasUpdate const & permissions
#endif
)
{
    using namespace fusion_proof;
    __ESBMC_assert(Calls::gate == 1U && Calls::results->gate_status == Status::success &&
                       !Calls::results->gate.rejected && Calls::correction++ == 0U,
                   "R-MEASUREMENT-FUSION: correction is attempted exactly once, only on an "
                   "accepted gate");
    __ESBMC_assert(same_state(state, *Calls::state) && same_matrix(P, *Calls::covariance) &&
                       same_matrix(r, *Calls::residual) && same_matrix(H, *Calls::H) && same_matrix(V, *Calls::V) &&
                       same(minimum, Calls::minimum),
                   "R-MEASUREMENT-FUSION: correction uses the SAME prior, residual, H, V "
                   "and threshold");
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
    __ESBMC_assert(bias_proof::same_permissions(permissions, *Calls::permissions),
                   "R-MEASUREMENT-FUSION: bias permissions reach correction "
                   "unchanged and do not affect gating");
#endif
    __ESBMC_assert(&state_output == Calls::state_target && &covariance_output == Calls::covariance_target,
                   "R-MEASUREMENT-FUSION: correction receives actual caller "
                   "outputs and legal aliases");
    unpublished();
    if (Calls::results->correction_status == Status::success)
    {
        state_output = Calls::results->state;
        covariance_output = Calls::results->covariance;
    }
    return Calls::results->correction_status;
}
} // namespace formal_eskf

using namespace fusion_proof;
void verify_measurement_fusion(State state, Covariance P, Residual r, Jacobian H, Noise V, Scalar minimum, Scalar sigma,
                               State state_output, Covariance covariance_output, Results results,
                               bias_proof::permissions_type permissions, bool check_only)
{
    constexpr bool state_alias = (FORMAL_ESKF_PROOF_STEP_ALIAS & 1) != 0;
    constexpr bool covariance_alias = (FORMAL_ESKF_PROOF_STEP_ALIAS & 2) != 0;
    auto const old_state = state;
    auto const old_P = P;
    auto const old_r = r;
    auto const old_H = H;
    auto const old_V = V;
    auto const old_permissions = permissions;
    auto const old_state_output = state_output;
    auto const old_covariance_output = covariance_output;
    State * state_pointer = &state_output;
    Covariance * covariance_pointer = &covariance_output;
    if constexpr (state_alias)
    {
        state_pointer = &state;
    }
    if constexpr (covariance_alias)
    {
        covariance_pointer = &P;
    }
    auto const before_state = *state_pointer;
    auto const before_covariance = *covariance_pointer;
    Calls::results = &results;
    Calls::state = &old_state;
    Calls::covariance = &old_P;
    Calls::residual = &old_r;
    Calls::H = &old_H;
    Calls::V = &old_V;
    Calls::minimum = minimum;
    Calls::sigma = sigma;
    Calls::state_target = state_pointer;
    Calls::covariance_target = covariance_pointer;
    Calls::state_before = &before_state;
    Calls::covariance_before = &before_covariance;
    Calls::permissions = &old_permissions;
    Calls::preparation = Calls::gate = Calls::correction = 0U;
    bool const eligible = finite_state(state) && std::isfinite(sigma);
    bool const prepared = eligible && results.covariance_status == Status::success;
    bool const diagnosed = prepared && results.gate_status == Status::success;
    bool const accepted = diagnosed && !results.gate.rejected;
    Status const gate_status = !eligible   ? Status::non_finite_input
                               : !prepared ? results.covariance_status
                                           : results.gate_status;
    Result result;
    if (check_only)
    {
        auto const check = formal_eskf::runtime::check_measurement(state, P, r, H, V, minimum, sigma);
        __ESBMC_assert(check.passed() == accepted && check.rejected == (!diagnosed || results.gate.rejected),
                       "R-MEASUREMENT-FUSION: check-only eligibility requires "
                       "successful valid non-rejected diagnostics");
        result = Result{check};
    }
    else
    {
        result = formal_eskf::runtime::fuse_measurement(state, P, r, H, V, minimum, sigma, *state_pointer,
                                                        *covariance_pointer
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
                                                        ,
                                                        permissions
#endif
        );
    }
    bool const corrected = !check_only && accepted;
    bool const fused = corrected && results.correction_status == Status::success;
    Status const expected_status = corrected ? results.correction_status : gate_status;
    Decision const expected_decision = fused                                ? Decision::fused
                                       : diagnosed && results.gate.rejected ? Decision::rejected
                                                                            : Decision::failed;
    __ESBMC_assert(result.status == expected_status && result.decision == expected_decision &&
                       result.diagnostics_valid == diagnosed,
                   "R-MEASUREMENT-FUSION: distinguish rejected, failed and "
                   "fused; retain gate diagnostics after "
                   "correction failure");
    for (std::size_t i = 0U; i < measurement_size; ++i)
    {
        __ESBMC_assert(same(result.innovation(i), diagnosed ? old_r(i) : Scalar{0}) &&
                           same(result.innovation_variance(i), diagnosed ? results.S(i, i) : Scalar{0}) &&
                           same(result.test_ratio(i), diagnosed ? results.gate.test_ratio(i) : Scalar{0}),
                       "R-MEASUREMENT-FUSION: return this prior's complete diagnostics or "
                       "fresh zero placeholders");
    }
    __ESBMC_assert(Calls::preparation == (eligible ? 1U : 0U) && Calls::gate == (prepared ? 1U : 0U) &&
                       Calls::correction == (corrected ? 1U : 0U),
                   "R-MEASUREMENT-FUSION: no skipped stages or correction after "
                   "rejection/failure");
    __ESBMC_assert((fused ? same_state(*state_pointer, results.state) : same_state(*state_pointer, before_state)) &&
                       (fused ? same_matrix(*covariance_pointer, results.covariance)
                              : same_matrix(*covariance_pointer, before_covariance)),
                   "R-MEASUREMENT-FUSION: only fused publishes both outputs; check-only, "
                   "rejection and failure preserve them");
    __ESBMC_assert(
        (state_alias ? same_state(state_output, old_state_output) : same_state(state, old_state)) &&
            (covariance_alias ? same_matrix(covariance_output, old_covariance_output) : same_matrix(P, old_P)) &&
            same_matrix(r, old_r) && same_matrix(H, old_H) && same_matrix(V, old_V) &&
            bias_proof::same_permissions(permissions, old_permissions),
        "R-MEASUREMENT-FUSION: preserve distinct inputs and unused outputs");
}
int main() { return 0; }
