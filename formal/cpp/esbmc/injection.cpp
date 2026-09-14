/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "contracts.hpp"

#include <type_traits>

#include <formal_eskf/eskf/injection.hpp>

#ifndef FORMAL_ESKF_PROOF_INJECTION_CONTRACT
#define FORMAL_ESKF_PROOF_INJECTION_CONTRACT 0
#endif

namespace injection_proof
{
using namespace contract_proof;
using AhrsState = formal_eskf::configuration::Ahrs::NominalState<Linalg>;
using AhrsError = formal_eskf::configuration::Ahrs::ErrorState<Linalg>;
using InsState = formal_eskf::configuration::Ins::NominalState<Linalg>;
using InsError = formal_eskf::configuration::Ins::ErrorState<Linalg>;

// Pure, failure-atomic summaries of already-proved public quaternion callees.
// Results and statuses are independent symbolic inputs: no successful, finite
// or unit-norm return is assumed. maps.cpp and normalization.cpp use exactly
// contract_proof::Linalg/Scalar and discharge these boundaries separately.
struct ExpContract
{
    inline static Quaternion result{};
    inline static Status status{};
    inline static Vector3 received{};
    inline static Scalar minimum{};
    inline static unsigned calls = 0U;
};

struct CompositionContract
{
    inline static Quaternion result{}, q_a{}, q_b{};
    inline static Status status{};
    inline static Scalar minimum{};
    inline static unsigned calls = 0U;
};
} // namespace injection_proof

#if FORMAL_ESKF_PROOF_INJECTION_CONTRACT
namespace formal_eskf::so3
{
template <>
inline Status try_exp<contract_proof::Linalg>(contract_proof::Vector3 const & theta, contract_proof::Scalar minimum,
                                              contract_proof::Quaternion & output) noexcept
{
    using injection_proof::ExpContract;
    __ESBMC_assert(ExpContract::calls == 0U, "E-INJECT contract: one Exp request");
    ++ExpContract::calls;
    ExpContract::received = theta;
    ExpContract::minimum = minimum;
    if (ExpContract::status == Status::success)
    {
        output = ExpContract::result;
    }
    return ExpContract::status;
}

template <>
inline Status try_compose_normalized<contract_proof::Linalg>(contract_proof::Quaternion const & q_a,
                                                             contract_proof::Quaternion const & q_b,
                                                             contract_proof::Scalar minimum,
                                                             contract_proof::Quaternion & output) noexcept
{
    using injection_proof::CompositionContract;
    __ESBMC_assert(CompositionContract::calls == 0U && injection_proof::ExpContract::calls == 1U,
                   "E-INJECT contract: one normalized composition after Exp");
    ++CompositionContract::calls;
    CompositionContract::q_a = q_a;
    CompositionContract::q_b = q_b;
    CompositionContract::minimum = minimum;
    if (CompositionContract::status == Status::success)
    {
        output = CompositionContract::result;
    }
    return CompositionContract::status;
}
} // namespace formal_eskf::so3
#endif

namespace injection_proof
{

// Explicit semantic field order, independent of production vector addition.
Vector3 const & additive_state(InsState const & state, std::size_t group)
{
    switch (group)
    {
    case 0U:
        return state.p_n;
    case 1U:
        return state.v_n;
    case 2U:
        return state.b_a;
    default:
        __ESBMC_assert(group == 3U, "E-INJECT oracle: four additive state groups");
        return state.b_g;
    }
}

Vector3 const & additive_error(InsError const & error, std::size_t group)
{
    switch (group)
    {
    case 0U:
        return error.delta_p_n;
    case 1U:
        return error.delta_v_n;
    case 2U:
        return error.delta_b_a;
    default:
        __ESBMC_assert(group == 3U, "E-INJECT oracle: four additive error groups");
        return error.delta_b_g;
    }
}

template <typename State> bool same_state(State const & a, State const & b)
{
    bool valid = same_vector(a.q_nb.coefficients(), b.q_nb.coefficients());
    if constexpr (std::is_same_v<State, InsState>)
    {
        for (std::size_t group = 0U; group < 4U; ++group)
        {
            valid = same_vector(additive_state(a, group), additive_state(b, group)) && valid;
        }
    }
    return valid;
}

template <typename Error> bool same_error(Error const & a, Error const & b)
{
    bool valid = same_vector(a.delta_theta_b, b.delta_theta_b);
    if constexpr (std::is_same_v<Error, InsError>)
    {
        for (std::size_t group = 0U; group < 4U; ++group)
        {
            valid = same_vector(additive_error(a, group), additive_error(b, group)) && valid;
        }
    }
    return valid;
}

template <typename State, typename Error>
void verify_injection(State state, Error error, Scalar minimum, State output, Quaternion increment, Status exp_status,
                      Quaternion composed, Status composition_status)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_INJECTION_CONTRACT && !FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT,
                   "Runner error: injection requires Exp/composition summaries and the matching constructor type");
    static_assert(FORMAL_ESKF_PROOF_ALIAS == 0 || FORMAL_ESKF_PROOF_ALIAS == 1);
    (void)output;
    auto const state_before = state;
    auto const error_before = error;
#if FORMAL_ESKF_PROOF_ALIAS
    auto & target = state;
#else
    auto & target = output;
#endif
    auto const before = target;

    bool finite_inputs = finite_vector(state_before.q_nb.coefficients()) && finite_vector(error_before.delta_theta_b) &&
                         std::isfinite(minimum);
    if constexpr (std::is_same_v<State, InsState>)
    {
        for (std::size_t group = 0U; group < 4U; ++group)
        {
            finite_inputs = finite_inputs && finite_vector(additive_state(state_before, group)) &&
                            finite_vector(additive_error(error_before, group));
        }
    }
    bool const threshold_valid = minimum > Scalar{0} && minimum <= Scalar{1};
    bool const call_exp = finite_inputs && threshold_valid;
    bool const call_composition = call_exp && exp_status == Status::success;
    Status expected = !finite_inputs                                   ? Status::non_finite_input
                      : !threshold_valid                               ? Status::domain_error
                      : exp_status != Status::success                  ? exp_status
                      : composition_status == Status::non_finite_input ? Status::non_finite_result
                                                                       : composition_status;

    Scalar sums[4U][3U]{};
    if constexpr (std::is_same_v<State, InsState>)
    {
        if (expected == Status::success)
        {
            for (std::size_t group = 0U; group < 4U; ++group)
            {
                for (std::size_t axis = 0U; axis < 3U; ++axis)
                {
                    sums[group][axis] =
                        additive_state(state_before, group)(axis) + additive_error(error_before, group)(axis);
                    if (!std::isfinite(sums[group][axis]))
                    {
                        expected = Status::non_finite_result;
                    }
                }
            }
        }
    }

    ExpContract::result = increment;
    ExpContract::status = exp_status;
    ExpContract::calls = 0U;
    CompositionContract::result = composed;
    CompositionContract::status = composition_status;
    CompositionContract::calls = 0U;

    // Execute the public overload, its actual attitude helper, finite checks,
    // all twelve INS additions and actual state publication/storage.
    auto const actual = formal_eskf::try_inject_nominal(state, error, minimum, target);
    __ESBMC_assert(actual == expected,
                   "E-INJECT: validation precedence, Exp status, composition remapping and additive overflow");
    __ESBMC_assert(ExpContract::calls == (call_exp ? 1U : 0U) &&
                       CompositionContract::calls == (call_composition ? 1U : 0U),
                   "E-INJECT: callees execute exactly after their prerequisites");
    if (call_exp)
    {
        __ESBMC_assert(same_vector(ExpContract::received, error_before.delta_theta_b) &&
                           same(ExpContract::minimum, minimum),
                       "E-INJECT: Exp receives the complete local attitude error and unchanged threshold");
    }
    if (call_composition)
    {
        __ESBMC_assert(same_vector(CompositionContract::q_a.coefficients(), state_before.q_nb.coefficients()) &&
                           same_vector(CompositionContract::q_b.coefficients(), increment.coefficients()) &&
                           same(CompositionContract::minimum, minimum),
                       "E-INJECT: normalized composition is old q_nb times the returned Exp increment");
    }
    if (actual == Status::success)
    {
        bool valid = same_vector(target.q_nb.coefficients(), composed.coefficients());
        if constexpr (std::is_same_v<State, InsState>)
        {
            for (std::size_t group = 0U; group < 4U; ++group)
            {
                for (std::size_t axis = 0U; axis < 3U; ++axis)
                {
                    valid = same(additive_state(target, group)(axis), sums[group][axis]) && valid;
                }
            }
        }
        __ESBMC_assert(valid, "E-INJECT: every attitude and Euclidean output coefficient is published correctly");
    }
    else
    {
        __ESBMC_assert(same_state(target, before), "E-INJECT: every prior output coefficient is preserved on failure");
    }
    __ESBMC_assert(same_error(error, error_before),
                   "E-INJECT: error mean is read-only, not reset by nominal injection");
#if !FORMAL_ESKF_PROOF_ALIAS
    __ESBMC_assert(same_state(state, state_before), "E-INJECT: a distinct prior state is unchanged on every return");
#endif
}
} // namespace injection_proof

void verify_ahrs_injection(injection_proof::AhrsState state, injection_proof::AhrsError error,
                           contract_proof::Scalar minimum, injection_proof::AhrsState output,
                           contract_proof::Quaternion increment, contract_proof::Status exp_status,
                           contract_proof::Quaternion composed, contract_proof::Status composition_status)
{
    injection_proof::verify_injection(state, error, minimum, output, increment, exp_status, composed,
                                      composition_status);
}

void verify_ins_injection(injection_proof::InsState state, injection_proof::InsError error,
                          contract_proof::Scalar minimum, injection_proof::InsState output,
                          contract_proof::Quaternion increment, contract_proof::Status exp_status,
                          contract_proof::Quaternion composed, contract_proof::Status composition_status)
{
    injection_proof::verify_injection(state, error, minimum, output, increment, exp_status, composed,
                                      composition_status);
}

int main() { return 0; }
