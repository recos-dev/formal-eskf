/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
// Reuse the existing root observer hook; finite classification remains the
// actual standard primitive. No arithmetic result is assumed by the observer.
#define FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT 1
#include "step_correction_support.hpp"
#include "bias_support.hpp"
#include <formal_eskf/runtime/navigation.hpp>

#ifndef FORMAL_ESKF_PROOF_RECOVERY_BOUNDARY
#define FORMAL_ESKF_PROOF_RECOVERY_BOUNDARY 0
#endif
namespace recovery_proof
{
using formal_eskf::Status;
using step_correction_proof::Backend;
using step_correction_proof::Covariance;
using step_correction_proof::Jacobian;
using step_correction_proof::Noise;
using step_correction_proof::Residual;
using step_correction_proof::same;
using step_correction_proof::same_matrix;
using step_correction_proof::same_state;
using step_correction_proof::Scalar;
using step_correction_proof::State;
constexpr std::size_t width = correction_proof::measurement_size;
using Check = formal_eskf::runtime::MeasurementCheck<Backend, width>;
using NavigationCheck = formal_eskf::runtime::NavigationInnovationCheck<Backend, width>;
using Result = formal_eskf::runtime::NavigationFusionResult<Backend, width>;
using Conditions = formal_eskf::runtime::GnssVelocityConditions<Scalar>;
using Permissions = formal_eskf::configuration::Ins::BiasUpdate;
static_assert(step_correction_proof::state_size == 15U && (width == 1U || width == 3U));
bool same_check(Check const & a, Check const & b)
{
    return a.status == b.status && a.diagnostics_valid == b.diagnostics_valid && a.rejected == b.rejected &&
           same_matrix(a.innovation, b.innovation) && same_matrix(a.innovation_variance, b.innovation_variance) &&
           same_matrix(a.test_ratio, b.test_ratio);
}
bool same_navigation(NavigationCheck const & a, NavigationCheck const & b)
{
    return same_check(a.gate, b.gate) && same_matrix(a.fusion_innovation, b.fusion_innovation) &&
           a.innovation_limited == b.innovation_limited;
}
bool diagnostic_contract(Check const & check, Scalar sigma)
{
    if (!check.diagnostics_valid)
    {
        return true;
    }
    bool valid = check.status == Status::success && std::isfinite(sigma) && sigma > Scalar{0};
    for (std::size_t i = 0U; i < width; ++i)
    {
        valid = valid && std::isfinite(check.innovation(i)) && std::isfinite(check.innovation_variance(i)) &&
                check.innovation_variance(i) > Scalar{0} && std::isfinite(check.test_ratio(i)) &&
                check.test_ratio(i) >= Scalar{0};
    }
    return valid;
}
Scalar clamp_innovation(Scalar innovation, Scalar bound)
{
    return innovation < -bound ? -bound : (innovation > bound ? bound : innovation);
}

NavigationCheck limited(NavigationCheck check, std::size_t axis, Scalar sigma, bool allowed, Scalar root)
{
    if (!check.gate.diagnostics_valid || !allowed || !check.gate.rejected)
    {
        return check;
    }
    if (!std::isfinite(root))
    {
        check.gate.status = Status::non_finite_result;
        return check;
    }
    Scalar const bound = sigma * root;
    if (!std::isfinite(bound))
    {
        check.gate.status = Status::non_finite_result;
        return check;
    }
    Scalar const raw = check.gate.innovation(axis);
    check.fusion_innovation(axis) = clamp_innovation(raw, bound);
    check.gate.status = Status::success;
    check.gate.rejected = false;
    check.innovation_limited = true;
    return check;
}
struct Preparation
{
    inline static Noise S;
    inline static Status status;
    inline static unsigned count{};
};
struct Call
{
    inline static State const * state{};
    inline static Covariance const * P{};
    inline static Residual const * raw{};
    inline static Residual const * selected{};
    inline static Noise const * V{};
    inline static Permissions const * permissions{};
    inline static State * state_target{};
    inline static Covariance * covariance_target{};
    inline static State const * state_before{};
    inline static Covariance const * covariance_before{};
    inline static State const * next_state{};
    inline static Covariance const * next_P{};
    inline static Check check;
    inline static Scalar minimum{}, sigma{};
    inline static unsigned checks{}, corrections{}, limits{};
    inline static NavigationCheck const * limit_input{};
    inline static Scalar root{};
    inline static bool limit_allowed{};
    inline static Status correction_status{};
};
void assert_H(Jacobian const & H)
{
    for (std::size_t i = 0; i < width; ++i)
    {
        for (std::size_t j = 0; j < 15U; ++j)
        {
            Scalar const expected = j == (width == 3U ? 3U + i : 2U) ? Scalar{1} : Scalar{0};
            __ESBMC_assert(same(H(i, j), expected),
                           "R-GNSS-VERTICAL-RECOVERY: exact velocity or Down selector Jacobian");
        }
    }
}
} // namespace recovery_proof

namespace recovery_proof
{
inline Scalar expected_root_argument{}, expected_root_result{};
}
namespace solve_proof
{
bool Math::is_finite(Scalar value) { return std::isfinite(value); }
void Math::record_root(Scalar input, Scalar result, unsigned index)
{
    __ESBMC_assert(index == 0U && same(input, recovery_proof::expected_root_argument) &&
                       same(result, recovery_proof::expected_root_result),
                   "R-GNSS-VERTICAL-RECOVERY: sqrt receives the selected innovation variance and forwards its return");
}
} // namespace solve_proof

#if FORMAL_ESKF_PROOF_RECOVERY_BOUNDARY == 1
namespace formal_eskf
{
template <>
inline Status correction_covariances<recovery_proof::Backend, 15U, recovery_proof::width>(
    recovery_proof::Covariance const &, recovery_proof::Residual const &, recovery_proof::Jacobian const &,
    recovery_proof::Noise const &, recovery_proof::Scalar, correction_proof::Gain &, recovery_proof::Noise & S)
{
    __ESBMC_assert(recovery_proof::Preparation::count++ == 0U, "R-GNSS-VERTICAL-RECOVERY: one covariance preparation");
    S = recovery_proof::Preparation::S;
    return recovery_proof::Preparation::status;
}
} // namespace formal_eskf
#endif

#if FORMAL_ESKF_PROOF_RECOVERY_BOUNDARY == 2
namespace formal_eskf::runtime::detail
{
template <>
inline void limit_vertical_innovation<recovery_proof::Backend, recovery_proof::width>(
    recovery_proof::NavigationCheck & check, std::size_t axis, recovery_proof::Scalar sigma, bool allowed)
{
    using namespace recovery_proof;
    __ESBMC_assert(
        Call::limits++ == 0U && Call::checks == 1U && Call::corrections == 0U && axis == (width == 3U ? 2U : 0U) &&
            same(sigma, Call::sigma) && allowed == Call::limit_allowed && same_navigation(check, *Call::limit_input) &&
            diagnostic_contract(check.gate, sigma),
        "R-GNSS-VERTICAL-RECOVERY: limiter follows the gate with exact vertical axis, policy and raw diagnostics");
    // Exact functional summary established by verify_navigation_limit for
    // this backend/shape, including arbitrary root returns and failures within the diagnostic contract.
    check = limited(check, axis, sigma, allowed, Call::root);
}
} // namespace formal_eskf::runtime::detail
namespace formal_eskf::runtime
{
template <>
inline recovery_proof::Check check_measurement<recovery_proof::Backend, recovery_proof::width>(
    recovery_proof::State const & state, recovery_proof::Covariance const & P, recovery_proof::Residual const & r,
    recovery_proof::Jacobian const & H, recovery_proof::Noise const & V, recovery_proof::Scalar minimum,
    recovery_proof::Scalar sigma)
{
    using namespace recovery_proof;
    __ESBMC_assert(Call::checks++ == 0U && Call::corrections == 0U && same_state(state, *Call::state) &&
                       same_matrix(P, *Call::P) && same_matrix(r, *Call::raw) && same_matrix(V, *Call::V) &&
                       same(minimum, Call::minimum) && same(sigma, Call::sigma),
                   "R-GNSS-VERTICAL-RECOVERY: gate forwards original prior, residual, noise and thresholds once");
    assert_H(H);
    return Call::check;
}
} // namespace formal_eskf::runtime
namespace formal_eskf
{
template <>
inline Status correct<recovery_proof::Backend, recovery_proof::width>(
    recovery_proof::State const & state, recovery_proof::Covariance const & P, recovery_proof::Residual const & r,
    recovery_proof::Jacobian const & H, recovery_proof::Noise const & V, recovery_proof::Scalar minimum,
    recovery_proof::State & state_output, recovery_proof::Covariance & covariance_output,
    configuration::Ins::BiasUpdate const & permissions)
{
    using namespace recovery_proof;
    __ESBMC_assert(
        Call::corrections++ == 0U && same_state(state, *Call::state) && same_matrix(P, *Call::P) &&
            same_matrix(r, *Call::selected) && same_matrix(V, *Call::V) && same(minimum, Call::minimum) &&
            bias_proof::same_permissions(permissions, *Call::permissions),
        "R-GNSS-VERTICAL-RECOVERY: correction forwards selected residual with unchanged prior, H, V and permissions");
    __ESBMC_assert(&state_output == Call::state_target && &covariance_output == Call::covariance_target &&
                       same_state(state_output, *Call::state_before) &&
                       same_matrix(covariance_output, *Call::covariance_before),
                   "R-GNSS-VERTICAL-RECOVERY: outputs remain unpublished before checked correction");
    assert_H(H);
    if (Call::correction_status == Status::success)
    {
        state_output = *Call::next_state;
        covariance_output = *Call::next_P;
    }
    return Call::correction_status;
}
} // namespace formal_eskf
#endif

using recovery_proof::Backend;
using recovery_proof::Call;
using recovery_proof::Check;
using recovery_proof::Conditions;
using recovery_proof::Covariance;
using recovery_proof::diagnostic_contract;
using recovery_proof::Jacobian;
using recovery_proof::limited;
using recovery_proof::NavigationCheck;
using recovery_proof::Noise;
using recovery_proof::Permissions;
using recovery_proof::Preparation;
using recovery_proof::Residual;
using recovery_proof::Result;
using recovery_proof::same;
using recovery_proof::same_matrix;
using recovery_proof::same_navigation;
using recovery_proof::same_state;
using recovery_proof::Scalar;
using recovery_proof::State;
using recovery_proof::Status;
using recovery_proof::width;
#if FORMAL_ESKF_PROOF_RECOVERY_BOUNDARY == 1
void verify_navigation_diagnostics(State state, Covariance P, Residual r, Jacobian H, Noise V, Scalar minimum,
                                   Scalar sigma, Noise S, Status preparation_status)
{
    auto const before_state = state;
    auto const before_P = P;
    auto const before_r = r;
    auto const before_H = H;
    auto const before_V = V;
    Preparation::S = S;
    Preparation::status = preparation_status;
    Preparation::count = 0U;
    auto const check = formal_eskf::runtime::check_measurement(state, P, r, H, V, minimum, sigma);
    __ESBMC_assert(diagnostic_contract(check, sigma), "R-GNSS-VERTICAL-RECOVERY: actual gate producer establishes "
                                                      "finite positive S, sigma and finite raw diagnostics");
    __ESBMC_assert(!check.diagnostics_valid || same_matrix(check.innovation, r),
                   "R-GNSS-VERTICAL-RECOVERY: valid gate retains raw residual");
    __ESBMC_assert(same_state(state, before_state) && same_matrix(P, before_P) && same_matrix(r, before_r) &&
                       same_matrix(H, before_H) && same_matrix(V, before_V),
                   "R-GNSS-VERTICAL-RECOVERY: actual gate producer preserves all inputs");
}
#endif
void verify_navigation_limit_product(Scalar sigma, Scalar root)
{
    __ESBMC_assume(std::isfinite(sigma) && sigma > Scalar{0} && std::isfinite(root) && root >= Scalar{0});
    Scalar const bound = sigma * root;
    __ESBMC_assert(
        !std::isfinite(bound) || bound >= Scalar{0},
        "R-GNSS-VERTICAL-RECOVERY: a finite limit is nonnegative under the explicit nonnegative-root premise");
}
void verify_navigation_clamp(Scalar innovation, Scalar bound)
{
    __ESBMC_assume(std::isfinite(innovation) && std::isfinite(bound) && bound >= Scalar{0});
    Scalar const result = recovery_proof::clamp_innovation(innovation, bound);
    __ESBMC_assert(result >= -bound && result <= bound && (innovation < Scalar{0} || result >= Scalar{0}) &&
                       (innovation > Scalar{0} || result <= Scalar{0}),
                   "R-GNSS-VERTICAL-RECOVERY: scalar clamp obeys the bound and preserves numerical sign");
    if (innovation >= -bound && innovation <= bound)
    {
        __ESBMC_assert(same(result, innovation),
                       "R-GNSS-VERTICAL-RECOVERY: in-bound innovation is retained including signed zero");
    }
}

#if FORMAL_ESKF_PROOF_RECOVERY_BOUNDARY == 0
void verify_navigation_limit(NavigationCheck check, std::size_t axis, Scalar sigma, bool allowed, Scalar root)
{
    __ESBMC_assume(axis < width && diagnostic_contract(check.gate, sigma));
    auto const expected = limited(check, axis, sigma, allowed, root);
    bool const attempt = check.gate.diagnostics_valid && allowed && check.gate.rejected;
    recovery_proof::expected_root_argument = check.gate.innovation_variance(axis);
    recovery_proof::expected_root_result = root;
    solve_proof::Math::sqrt.results[0U] = root;
    solve_proof::Math::sqrt.calls = 0U;
    formal_eskf::runtime::detail::limit_vertical_innovation(check, axis, sigma, allowed);
    __ESBMC_assert(
        same_navigation(check, expected) && solve_proof::Math::sqrt.calls == (attempt ? 1U : 0U),
        "R-GNSS-VERTICAL-RECOVERY: exact limiting trigger, scalar errors, selected axis and untouched raw diagnostics");
}
#endif

#if FORMAL_ESKF_PROOF_RECOVERY_BOUNDARY == 2
void verify_navigation_recovery(State state, Covariance P, Residual observation, Noise V, Scalar minimum, Scalar sigma,
                                Conditions conditions, bool bad_acceleration, Permissions permissions, Check gate,
                                Scalar root, State state_output, Covariance covariance_output, State successful_state,
                                Covariance successful_P, Status correction_status)
{
    __ESBMC_assume(diagnostic_contract(gate, sigma));
    auto const old_state = state;
    auto const old_P = P;
    auto const old_observation = observation;
    auto const old_V = V;
    auto const old_permissions = permissions;
    auto const old_conditions = conditions;
    auto const old_state_output = state_output;
    auto const old_covariance_output = covariance_output;
    State * state_target = &state_output;
    Covariance * P_target = &covariance_output;
    if constexpr ((FORMAL_ESKF_PROOF_STEP_ALIAS & 1U) != 0U)
    {
        state_target = &state;
    }
    if constexpr ((FORMAL_ESKF_PROOF_STEP_ALIAS & 2U) != 0U)
    {
        P_target = &P;
    }
    auto const before_state = *state_target;
    auto const before_P = *P_target;
    Residual raw;
    for (std::size_t i = 0; i < width; ++i)
    {
        if constexpr (width == 3U)
        {
            raw(i) = observation(i) - state.v_n(i);
        }
        else
        {
            raw(i) = observation(i) - state.p_n(2U);
        }
    }
    Status prefix = Status::success;
    if constexpr (width == 3U)
    {
        if (!correction_proof::finite_matrix(state.v_n) || !correction_proof::finite_matrix(observation) ||
            !std::isfinite(conditions.speed_accuracy) || !std::isfinite(conditions.maximum_speed_accuracy))
        {
            prefix = Status::non_finite_input;
        }
        else if (conditions.speed_accuracy < Scalar{0} || conditions.maximum_speed_accuracy <= Scalar{0})
        {
            prefix = Status::domain_error;
        }
    }
    else
    {
        if (!correction_proof::finite_matrix(state.p_n) || !std::isfinite(observation(0U)) || !std::isfinite(V(0U, 0U)))
        {
            prefix = Status::non_finite_input;
        }
    }
    if (prefix == Status::success && !correction_proof::finite_matrix(raw))
    {
        prefix = Status::non_finite_result;
    }
    bool allowed = bad_acceleration;
    if constexpr (width == 3U)
    {
        allowed = conditions.bad_vertical_acceleration &&
                  conditions.speed_accuracy < conditions.maximum_speed_accuracy && gate.test_ratio(2U) > Scalar{1} &&
                  gate.test_ratio(0U) < Scalar{1} && gate.test_ratio(1U) < Scalar{1};
    }
    NavigationCheck expected;
    if (prefix != Status::success)
    {
        expected.gate.status = prefix;
    }
    else
    {
        expected.gate = gate;
        expected.fusion_innovation = gate.innovation;
        expected = limited(expected, width == 3U ? 2U : 0U, sigma, allowed, root);
    }
    bool const corrected =
        expected.gate.status == Status::success && expected.gate.diagnostics_valid && !expected.gate.rejected;
    Call::state = &old_state;
    Call::P = &old_P;
    Call::raw = &raw;
    Call::selected = &expected.fusion_innovation;
    Call::V = &old_V;
    Call::minimum = minimum;
    Call::sigma = sigma;
    Call::permissions = &old_permissions;
    Call::check = gate;
    Call::checks = 0U;
    Call::corrections = 0U;
    Call::limits = 0U;
    NavigationCheck limit_input;
    limit_input.gate = gate;
    limit_input.fusion_innovation = gate.innovation;
    Call::limit_input = &limit_input;
    Call::root = root;
    Call::limit_allowed = allowed;
    Call::state_target = state_target;
    Call::covariance_target = P_target;
    Call::state_before = &before_state;
    Call::covariance_before = &before_P;
    Call::next_state = &successful_state;
    Call::next_P = &successful_P;
    Call::correction_status = correction_status;
    recovery_proof::expected_root_argument = gate.innovation_variance(width == 3U ? 2U : 0U);
    recovery_proof::expected_root_result = root;
    solve_proof::Math::sqrt.results[0U] = root;
    solve_proof::Math::sqrt.calls = 0U;
#if FORMAL_ESKF_PROOF_MEASUREMENT_SIZE == 3
    auto const actual = formal_eskf::runtime::fuse_gnss_velocity(state, P, observation, V, minimum, sigma, conditions,
                                                                 *state_target, *P_target, permissions);
#else
    auto const actual = formal_eskf::runtime::fuse_height(state, P, observation(0U), V(0U, 0U), minimum, sigma,
                                                          bad_acceleration, *state_target, *P_target, permissions);
#endif
    using Decision = formal_eskf::runtime::FusionDecision;
    bool const fused = corrected && correction_status == Status::success;
    Status const status = corrected ? correction_status : expected.gate.status;
    Decision const decision =
        fused ? Decision::fused
        : expected.gate.status == Status::success && expected.gate.diagnostics_valid && expected.gate.rejected
            ? Decision::rejected
            : Decision::failed;
    __ESBMC_assert(
        actual.fusion.status == status && actual.fusion.decision == decision &&
            actual.fusion.diagnostics_valid == expected.gate.diagnostics_valid &&
            same_matrix(actual.fusion.innovation, expected.gate.innovation) &&
            same_matrix(actual.fusion.innovation_variance, expected.gate.innovation_variance) &&
            same_matrix(actual.fusion.test_ratio, expected.gate.test_ratio) &&
            same_matrix(actual.fusion_innovation, expected.fusion_innovation) &&
            actual.innovation_limited == expected.innovation_limited,
        "R-GNSS-VERTICAL-RECOVERY: exact public validation, raw/selected diagnostics, limit policy and final decision");
    __ESBMC_assert(Call::checks == (prefix == Status::success ? 1U : 0U) &&
                       Call::corrections == (corrected ? 1U : 0U) &&
                       Call::limits == (prefix == Status::success ? 1U : 0U) && solve_proof::Math::sqrt.calls == 0U,
                   "R-GNSS-VERTICAL-RECOVERY: no skipped gate, re-gating or correction after rejection/error");
    __ESBMC_assert(
        (fused ? same_state(*state_target, successful_state) : same_state(*state_target, before_state)) &&
            (fused ? same_matrix(*P_target, successful_P) : same_matrix(*P_target, before_P)),
        "R-GNSS-VERTICAL-RECOVERY: only actual fused result publishes both outputs in every legal alias mode");
    __ESBMC_assert(((FORMAL_ESKF_PROOF_STEP_ALIAS & 1U) != 0U ? same_state(state_output, old_state_output)
                                                              : same_state(state, old_state)) &&
                       ((FORMAL_ESKF_PROOF_STEP_ALIAS & 2U) != 0U
                            ? same_matrix(covariance_output, old_covariance_output)
                            : same_matrix(P, old_P)) &&
                       same_matrix(observation, old_observation) && same_matrix(V, old_V) &&
                       bias_proof::same_permissions(permissions, old_permissions) &&
                       conditions.bad_vertical_acceleration == old_conditions.bad_vertical_acceleration &&
                       same(conditions.speed_accuracy, old_conditions.speed_accuracy) &&
                       same(conditions.maximum_speed_accuracy, old_conditions.maximum_speed_accuracy),
                   "R-GNSS-VERTICAL-RECOVERY: preserve all borrowed inputs and unused outputs");
}
#endif
int main() { return 0; }
