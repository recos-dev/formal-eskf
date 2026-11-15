/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "bias_support.hpp"
#include "step_correction_support.hpp"
#include <formal_eskf/runtime/accelerometer.hpp>

namespace accel_fusion_proof
{
using step_correction_proof::Backend;
using step_correction_proof::Covariance;
using step_correction_proof::Noise;
using step_correction_proof::same;
using step_correction_proof::same_matrix;
using step_correction_proof::same_state;
using step_correction_proof::Scalar;
using step_correction_proof::State;
using step_correction_proof::Status;
using Vector = step_correction_proof::Vector3;
using Permissions = bias_proof::permissions_type;
using Conditions = formal_eskf::runtime::AccelerometerFusionConditions;
using Magnitude = formal_eskf::runtime::AccelerometerMagnitude<Backend>::Result;
using Decision = formal_eskf::runtime::FusionDecision;
static_assert(step_correction_proof::boundaries_clear && correction_proof::measurement_size == 3U &&
              (correction_proof::state_size == 3U || correction_proof::state_size == 15U) &&
              FORMAL_ESKF_PROOF_STEP_ALIAS < 4);
struct Output
{
    State state;
    Covariance P;
    Status status;
};
struct Call
{
    inline static State const *state{}, *before_state{};
    inline static Covariance const *P{}, *before_P{};
    inline static Vector const *sample{}, *gravity{};
    inline static Noise const * V{};
    inline static Permissions const * permissions{};
    inline static State * state_target{};
    inline static Covariance * P_target{};
    inline static Output const * output{};
    inline static Scalar minimum{};
    inline static unsigned count{};
};
inline Status correct(State const & state, Covariance const & P, Vector const & sample, Vector const & gravity,
                      Noise const & V, Scalar minimum, State & output, Covariance & next_P)
{
    __ESBMC_assert(
        Call::count++ == 0U && same_state(state, *Call::state) && same_matrix(P, *Call::P) &&
            same_matrix(sample, *Call::sample) && same_matrix(gravity, *Call::gravity) && same_matrix(V, *Call::V) &&
            same(minimum, Call::minimum),
        "R-ACCELEROMETER-FUSION: correct receives the original sample, prior, gravity, noise and threshold once");
    __ESBMC_assert(&output == Call::state_target && &next_P == Call::P_target &&
                       same_state(output, *Call::before_state) && same_matrix(next_P, *Call::before_P),
                   "R-ACCELEROMETER-FUSION: original output aliases and sentinel values reach correction unchanged");
    // E-OBS (models 6/7) and E-BIAS-UPDATE prove this same-type atomic frame.
    // Successful outputs/status are otherwise unconstrained; no numeric
    // validity, success, unit attitude or finite-input premise is assumed.
    if (Call::output->status == Status::success)
    {
        output = Call::output->state;
        next_P = Call::output->P;
    }
    return Call::output->status;
}
inline bool same_conditions(Conditions const & a, Conditions const & b)
{
    return a.enabled == b.enabled && a.tilt_aligned == b.tilt_aligned && a.horizontal_aiding == b.horizontal_aiding &&
           a.at_rest == b.at_rest && a.clipped == b.clipped;
}
} // namespace accel_fusion_proof
namespace formal_eskf
{
template <>
inline Status correct_accelerometer<accel_fusion_proof::Backend>(
    accel_fusion_proof::State const & state, accel_fusion_proof::Covariance const & P,
    accel_fusion_proof::Vector const & sample, accel_fusion_proof::Vector const & gravity,
    accel_fusion_proof::Noise const & V, accel_fusion_proof::Scalar minimum, accel_fusion_proof::State & output,
    accel_fusion_proof::Covariance & next_P
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
    ,
    configuration::Ins::BiasUpdate const & permissions
#endif
)
{
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
    __ESBMC_assert(bias_proof::same_permissions(permissions, *accel_fusion_proof::Call::permissions),
                   "R-ACCELEROMETER-FUSION: every INS bias permission reaches the core unchanged");
#endif
    return accel_fusion_proof::correct(state, P, sample, gravity, V, minimum, output, next_P);
}
} // namespace formal_eskf
void verify_accelerometer_fusion(accel_fusion_proof::State state, accel_fusion_proof::Covariance P,
                                 accel_fusion_proof::Vector sample, accel_fusion_proof::Vector gravity,
                                 accel_fusion_proof::Noise V, accel_fusion_proof::Scalar minimum,
                                 accel_fusion_proof::Conditions conditions, accel_fusion_proof::Magnitude magnitude,
                                 accel_fusion_proof::Permissions permissions, accel_fusion_proof::State state_output,
                                 accel_fusion_proof::Covariance P_output, accel_fusion_proof::Output delegated)
{
    using namespace accel_fusion_proof;
    constexpr bool state_alias = (FORMAL_ESKF_PROOF_STEP_ALIAS & 1U) != 0U;
    constexpr bool covariance_alias = (FORMAL_ESKF_PROOF_STEP_ALIAS & 2U) != 0U;
    auto const old_state = state;
    auto const old_P = P;
    auto const old_sample = sample;
    auto const old_gravity = gravity;
    auto const old_V = V;
    auto const old_conditions = conditions;
    auto const old_magnitude = magnitude;
    auto const old_permissions = permissions;
    auto const old_state_output = state_output;
    auto const old_P_output = P_output;
    State * state_target = &state_output;
    Covariance * P_target = &P_output;
    if constexpr (state_alias)
    {
        state_target = &state;
    }
    if constexpr (covariance_alias)
    {
        P_target = &P;
    }
    auto const before_state = *state_target;
    auto const before_P = *P_target;
    Call::state = &old_state;
    Call::P = &old_P;
    Call::sample = &old_sample;
    Call::gravity = &old_gravity;
    Call::V = &old_V;
    Call::permissions = &old_permissions;
    Call::before_state = &before_state;
    Call::before_P = &before_P;
    Call::state_target = state_target;
    Call::P_target = P_target;
    Call::minimum = minimum;
    Call::output = &delegated;
    Call::count = 0U;
    unsigned const row = static_cast<unsigned>(conditions.enabled) + 2U * conditions.tilt_aligned +
                         4U * conditions.horizontal_aiding + 8U * conditions.at_rest +
                         16U * magnitude.within_gravity_band;
    bool const active = magnitude.status == Status::success && (row == 11U || row == 19U || row == 27U);
    bool const eligible = active && !conditions.clipped[0U] && !conditions.clipped[1U] && !conditions.clipped[2U];
    bool const fused = eligible && delegated.status == Status::success;
    Status const expected_status = eligible ? delegated.status : magnitude.status;
    Decision const expected_decision = magnitude.status != Status::success ? Decision::failed
                                       : !eligible                         ? Decision::rejected
                                       : fused                             ? Decision::fused
                                                                           : Decision::failed;
    auto const result = formal_eskf::runtime::fuse_accelerometer(state, P, sample, gravity, V, conditions, magnitude,
                                                                 minimum, *state_target, *P_target
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
                                                                 ,
                                                                 permissions
#endif
    );
    __ESBMC_assert(
        Call::count == static_cast<unsigned>(eligible) && result.status == expected_status &&
            result.decision == expected_decision && result.conditions.status == magnitude.status &&
            result.conditions.active == active && result.conditions.sample_eligible == eligible,
        "R-ACCELEROMETER-FUSION: every prerequisite, status and decision is distinct from actual correction success");
    __ESBMC_assert(
        fused ? same_state(*state_target, delegated.state) && same_matrix(*P_target, delegated.P)
              : same_state(*state_target, before_state) && same_matrix(*P_target, before_P),
        "R-ACCELEROMETER-FUSION: only successful correction publishes, rejection and failure preserve all outputs");
    __ESBMC_assert(state_alias ? same_state(state_output, old_state_output) : same_state(state, old_state),
                   "R-ACCELEROMETER-FUSION: disjoint prior or unused nominal output remains unchanged");
    __ESBMC_assert(covariance_alias ? same_matrix(P_output, old_P_output) : same_matrix(P, old_P),
                   "R-ACCELEROMETER-FUSION: disjoint prior or unused covariance output remains unchanged");
    __ESBMC_assert(same_matrix(sample, old_sample) && same_matrix(gravity, old_gravity) && same_matrix(V, old_V) &&
                       same_conditions(conditions, old_conditions) && magnitude.status == old_magnitude.status &&
                       magnitude.within_gravity_band == old_magnitude.within_gravity_band &&
                       same(magnitude.squared_norm, old_magnitude.squared_norm) &&
                       bias_proof::same_permissions(permissions, old_permissions),
                   "R-ACCELEROMETER-FUSION: all borrowed observation, condition, diagnostic and permission inputs "
                   "remain unchanged");
}
int main() { return 0; }
