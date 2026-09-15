/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "step_correction_support.hpp"

namespace step_correction_proof
{
// Summaries write only their separate scratch outputs, even on failure.
// No finite/unit/PSD/success premise is imposed. Their same-type input frames
// are established by E-CORRECT and step_correction_frames.cpp, not inferred
// from the older contract_proof::Linalg injection/reset instantiations.
struct Results
{
    Correction delta;
    Covariance joseph, reset;
    State injected;
    Status correction_status, injection_status, reset_status;
};
struct Calls
{
    inline static Results const * results = nullptr;
    inline static State const * state = nullptr;
    inline static Covariance const * covariance = nullptr;
    inline static Residual const * residual = nullptr;
    inline static Jacobian const * H = nullptr;
    inline static Noise const * V = nullptr;
    inline static State const * state_target = nullptr;
    inline static Covariance const * covariance_target = nullptr;
    inline static State const * state_before = nullptr;
    inline static Covariance const * covariance_before = nullptr;
    inline static Scalar minimum{};
    inline static unsigned correction{}, injection{}, reset{};
};
void unpublished()
{
    __ESBMC_assert(same_state(*Calls::state_target, *Calls::state_before) &&
                       same_matrix(*Calls::covariance_target, *Calls::covariance_before),
                   "E-STEP-CORRECT: neither caller output is published before all three stages succeed");
}
} // namespace step_correction_proof

#if FORMAL_ESKF_PROOF_STEP_CONTRACT
namespace formal_eskf::detail
{
template <>
inline Status try_compute_correction<step_correction_proof::Backend, step_correction_proof::state_size,
                                     step_correction_proof::measurement_size>(
    step_correction_proof::Covariance const & P, step_correction_proof::Residual const & r,
    step_correction_proof::Jacobian const & H, step_correction_proof::Noise const & V,
    step_correction_proof::Scalar minimum, step_correction_proof::Correction & delta,
    step_correction_proof::Covariance & output) noexcept
{
    using namespace step_correction_proof;
    __ESBMC_assert(Calls::correction++ == 0U && Calls::injection == 0U && Calls::reset == 0U,
                   "E-STEP-CORRECT: linear correction executes first and once");
    __ESBMC_assert(same_matrix(P, *Calls::covariance) && same_matrix(r, *Calls::residual) &&
                       same_matrix(H, *Calls::H) && same_matrix(V, *Calls::V) && same(minimum, Calls::minimum),
                   "E-STEP-CORRECT: correction receives all original measurement/prior inputs");
    __ESBMC_assert(&output != &P && &output != Calls::covariance_target && static_cast<void const *>(&delta) != &r &&
                       static_cast<void const *>(&delta) != &output,
                   "E-STEP-CORRECT: linear results use separate scratch objects");
    unpublished();
    delta = Calls::results->delta;
    output = Calls::results->joseph;
    return Calls::results->correction_status;
}
} // namespace formal_eskf::detail

namespace formal_eskf
{
template <>
inline Status try_inject_nominal<step_correction_proof::Backend>(step_correction_proof::State const & state,
                                                                 step_correction_proof::Error const & error,
                                                                 step_correction_proof::Scalar minimum,
                                                                 step_correction_proof::State & output) noexcept
{
    using namespace step_correction_proof;
    __ESBMC_assert(Calls::correction == 1U && Calls::results->correction_status == Status::success &&
                       Calls::injection++ == 0U && Calls::reset == 0U,
                   "E-STEP-CORRECT: injection occurs once only after successful correction");
    __ESBMC_assert(same_state(state, *Calls::state) && unpacked(error, Calls::results->delta) &&
                       same(minimum, Calls::minimum),
                   "E-STEP-CORRECT: injection receives OLD nominal state and every unpacked error coordinate");
    __ESBMC_assert(&output != &state && &output != Calls::state_target,
                   "E-STEP-CORRECT: nominal injection writes separate scratch");
    unpublished();
    output = Calls::results->injected;
    return Calls::results->injection_status;
}

template <>
inline Status try_reset_covariance<step_correction_proof::Backend>(step_correction_proof::Error const & error,
                                                                   step_correction_proof::Covariance const & covariance,
                                                                   step_correction_proof::Covariance & output) noexcept
{
    using namespace step_correction_proof;
    __ESBMC_assert(Calls::injection == 1U && Calls::results->injection_status == Status::success &&
                       Calls::reset++ == 0U,
                   "E-STEP-CORRECT: reset occurs once only after successful injection");
    __ESBMC_assert(unpacked(error, Calls::results->delta) && same_matrix(covariance, Calls::results->joseph),
                   "E-STEP-CORRECT: reset uses the complete Joseph covariance and error BEFORE zeroing");
    __ESBMC_assert(&output != &covariance && &output != Calls::covariance_target,
                   "E-STEP-CORRECT: covariance reset writes separate scratch");
    unpublished();
    output = Calls::results->reset;
    return Calls::results->reset_status;
}
} // namespace formal_eskf
#endif

using namespace step_correction_proof;

void verify_correction_step(State state, Covariance covariance, Residual residual, Jacobian H, Noise V, Scalar minimum,
                            State state_output, Covariance covariance_output, Results results)
{
    constexpr bool configured = FORMAL_ESKF_PROOF_STEP_CONTRACT == 1 && !FORMAL_ESKF_PROOF_STEP_FRAME &&
                                boundaries_clear && measurement_size <= 3U && FORMAL_ESKF_PROOF_STEP_ALIAS >= 0 &&
                                FORMAL_ESKF_PROOF_STEP_ALIAS < 4;
    __ESBMC_assert(configured,
                   "Runner error: correction step requires only three component summaries and a valid alias");
    if constexpr (!configured)
        return;
    constexpr bool state_alias = (FORMAL_ESKF_PROOF_STEP_ALIAS & 1) != 0;
    constexpr bool covariance_alias = (FORMAL_ESKF_PROOF_STEP_ALIAS & 2) != 0;
    auto const old_state = state;
    auto const old_covariance = covariance;
    auto const old_residual = residual;
    auto const old_H = H;
    auto const old_V = V;
    auto const old_state_output = state_output;
    auto const old_covariance_output = covariance_output;
    State * state_pointer = &state_output;
    Covariance * covariance_pointer = &covariance_output;
    if constexpr (state_alias)
        state_pointer = &state;
    if constexpr (covariance_alias)
        covariance_pointer = &covariance;
    State & target_state = *state_pointer;
    Covariance & target_covariance = *covariance_pointer;
    auto const before_state = target_state;
    auto const before_covariance = target_covariance;
    Calls::results = &results;
    Calls::state = &old_state;
    Calls::covariance = &old_covariance;
    Calls::residual = &old_residual;
    Calls::H = &old_H;
    Calls::V = &old_V;
    Calls::minimum = minimum;
    Calls::state_target = &target_state;
    Calls::covariance_target = &target_covariance;
    Calls::state_before = &before_state;
    Calls::covariance_before = &before_covariance;
    Calls::correction = Calls::injection = Calls::reset = 0U;
    auto const status =
        formal_eskf::try_correct(state, covariance, residual, H, V, minimum, target_state, target_covariance);
    Status const expected = results.correction_status != Status::success  ? results.correction_status
                            : results.injection_status != Status::success ? results.injection_status
                                                                          : results.reset_status;
    __ESBMC_assert(status == expected, "E-STEP-CORRECT: exact first failure or success");
    __ESBMC_assert(
        Calls::correction == 1U && Calls::injection == (results.correction_status == Status::success ? 1U : 0U) &&
            Calls::reset ==
                (results.correction_status == Status::success && results.injection_status == Status::success ? 1U : 0U),
        "E-STEP-CORRECT: no skipped, repeated or post-failure stage");
    __ESBMC_assert(status == Status::success
                       ? same_state(target_state, results.injected) && same_matrix(target_covariance, results.reset)
                       : same_state(target_state, before_state) && same_matrix(target_covariance, before_covariance),
                   "E-STEP-CORRECT: complete state/reset-covariance publication or complete rollback");
    __ESBMC_assert(state_alias ? same_state(state_output, old_state_output) : same_state(state, old_state),
                   "E-STEP-CORRECT: distinct state input or unused output is unchanged");
    __ESBMC_assert(covariance_alias ? same_matrix(covariance_output, old_covariance_output)
                                    : same_matrix(covariance, old_covariance),
                   "E-STEP-CORRECT: distinct covariance input or unused output is unchanged");
    __ESBMC_assert(same_matrix(residual, old_residual) && same_matrix(H, old_H) && same_matrix(V, old_V),
                   "E-STEP-CORRECT: all measurement inputs remain unchanged");
}

int main() { return 0; }
