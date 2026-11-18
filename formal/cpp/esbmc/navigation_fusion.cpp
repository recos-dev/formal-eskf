/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bias_support.hpp"
#include "observations_support.hpp"
#include <formal_eskf/runtime/navigation.hpp>

namespace navigation_proof
{
using namespace observation_proof;
using Result = formal_eskf::runtime::FusionResult<backend_type, measurement_size>;
using Decision = formal_eskf::runtime::FusionDecision;
static_assert(configured && model < 4U && !FORMAL_ESKF_PROOF_OBSERVATION_ROTATION &&
              !FORMAL_ESKF_PROOF_OBSERVATION_CONTRACT && FORMAL_ESKF_PROOF_ALIAS < 4U);
struct Output
{
    state_type state;
    covariance_type covariance;
    Result result;
};
struct Call
{
    inline static Output const * output = nullptr;
    inline static state_type const * state = nullptr;
    inline static covariance_type const * covariance = nullptr;
    inline static residual_type const * measurement = nullptr;
    inline static noise_type const * V = nullptr;
    inline static Model const * model = nullptr;
    inline static state_type * state_target = nullptr;
    inline static covariance_type * covariance_target = nullptr;
    inline static state_type const * state_before = nullptr;
    inline static covariance_type const * covariance_before = nullptr;
    inline static bias_proof::permissions_type const * permissions = nullptr;
    inline static value_type minimum{}, sigma{};
    inline static unsigned count{};
};
bool same_result(Result const & a, Result const & b)
{
    return a.decision == b.decision && a.status == b.status && a.diagnostics_valid == b.diagnostics_valid &&
           same_matrix(a.innovation, b.innovation) && same_matrix(a.innovation_variance, b.innovation_variance) &&
           same_matrix(a.test_ratio, b.test_ratio);
}
} // namespace navigation_proof
namespace formal_eskf::runtime
{
template <>
inline navigation_proof::Result fuse_measurement<navigation_proof::backend_type, navigation_proof::measurement_size,
                                                 navigation_proof::correction_model>(
    navigation_proof::state_type const & state, navigation_proof::covariance_type const & P,
    navigation_proof::residual_type const & r, navigation_proof::jacobian_type const & H,
    navigation_proof::noise_type const & V, navigation_proof::value_type minimum, navigation_proof::value_type sigma,
    navigation_proof::state_type & state_output, navigation_proof::covariance_type & covariance_output,
    configuration::Ins::BiasUpdate const & permissions)
{
    using namespace navigation_proof;
    __ESBMC_assert(Call::count++ == 0U && same_state(state, *Call::state) && same_matrix(P, *Call::covariance) &&
                       same_matrix(V, *Call::V) && same(minimum, Call::minimum) && same(sigma, Call::sigma) &&
                       bias_proof::same_permissions(permissions, *Call::permissions),
                   "R-MEASUREMENT-FUSION: navigation wrapper forwards original prior, full "
                   "V, thresholds and permissions once");
    __ESBMC_assert(&state_output == Call::state_target && &covariance_output == Call::covariance_target &&
                       same_state(state_output, *Call::state_before) &&
                       same_matrix(covariance_output, *Call::covariance_before),
                   "R-MEASUREMENT-FUSION: wrappers preserve caller outputs until "
                   "delegated fusion");
    for (std::size_t i = 0U; i < measurement_size; ++i)
    {
        __ESBMC_assert(same(r(i), (*Call::measurement)(i)-Call::model->prediction[i]),
                       "R-MEASUREMENT-FUSION: correct NED residual and selected "
                       "position/velocity axes");
        for (std::size_t j = 0U; j < state_size; ++j)
        {
            __ESBMC_assert(same(H(i, j), Call::model->H[i][j]),
                           "R-MEASUREMENT-FUSION: exact selector Jacobian, including "
                           "all unobserved zeros");
        }
    }
    if (Call::output->result.decision == Decision::fused)
    {
        state_output = Call::output->state;
        covariance_output = Call::output->covariance;
    }
    return Call::output->result;
}
} // namespace formal_eskf::runtime
using namespace navigation_proof;
void verify_navigation_fusion(state_type state, covariance_type P, residual_type measurement, noise_type V,
                              value_type minimum, value_type sigma, state_type state_output,
                              covariance_type covariance_output, bias_proof::permissions_type permissions,
                              Output output)
{
    constexpr bool state_alias = (FORMAL_ESKF_PROOF_ALIAS & 1U) != 0;
    constexpr bool covariance_alias = (FORMAL_ESKF_PROOF_ALIAS & 2U) != 0;
    auto const old_state = state;
    auto const old_P = P;
    auto const old_measurement = measurement;
    auto const old_V = V;
    auto const old_permissions = permissions;
    auto const old_state_output = state_output;
    auto const old_covariance_output = covariance_output;
    state_type * state_pointer = &state_output;
    covariance_type * covariance_pointer = &covariance_output;
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
    Model expected_model_value;
    vector_type unused_reference;
    expected_model(state, unused_reference, expected_model_value);
    bool finite = finite_vector(measurement);
    if constexpr (model <= 1U)
    {
        finite = finite_vector(state.p_n) && finite;
    }
    else
    {
        finite = finite_vector(state.v_n) && finite;
    }
    if constexpr (model == 1U)
    {
        finite = std::isfinite(V(0U, 0U)) && finite;
    }
    bool finite_residual = true;
    for (std::size_t i = 0U; i < measurement_size; ++i)
    {
        finite_residual = std::isfinite(measurement(i) - expected_model_value.prediction[i]) && finite_residual;
    }
    Status const prefix = !finite            ? Status::non_finite_input
                          : !finite_residual ? Status::non_finite_result
                                             : Status::success;
    Call::output = &output;
    Call::state = &old_state;
    Call::covariance = &old_P;
    Call::measurement = &old_measurement;
    Call::V = &old_V;
    Call::model = &expected_model_value;
    Call::minimum = minimum;
    Call::sigma = sigma;
    Call::permissions = &old_permissions;
    Call::state_target = state_pointer;
    Call::covariance_target = covariance_pointer;
    Call::state_before = &before_state;
    Call::covariance_before = &before_covariance;
    Call::count = 0U;
#if FORMAL_ESKF_PROOF_OBSERVATION == 0
    auto const actual = formal_eskf::runtime::fuse_horizontal_position(
        state, P, measurement, V, minimum, sigma, *state_pointer, *covariance_pointer, permissions);
#elif FORMAL_ESKF_PROOF_OBSERVATION == 1
    auto const actual = formal_eskf::runtime::fuse_vertical_position(
        state, P, measurement(0U), V(0U, 0U), minimum, sigma, *state_pointer, *covariance_pointer, permissions);
#elif FORMAL_ESKF_PROOF_OBSERVATION == 2
    auto const actual = formal_eskf::runtime::fuse_horizontal_velocity(
        state, P, measurement, V, minimum, sigma, *state_pointer, *covariance_pointer, permissions);
#else
    auto const actual = formal_eskf::runtime::fuse_velocity(state, P, measurement, V, minimum, sigma, *state_pointer,
                                                            *covariance_pointer, permissions);
#endif
    Result expected;
    expected.status = prefix;
    if (prefix == Status::success)
    {
        expected = output.result;
    }
    bool const fused = prefix == Status::success && output.result.decision == Decision::fused;
    __ESBMC_assert(Call::count == (prefix == Status::success ? 1U : 0U) && same_result(actual, expected),
                   "R-MEASUREMENT-FUSION: wrappers return precise prefix errors "
                   "or every delegated diagnostic/decision unchanged");
    __ESBMC_assert((fused ? same_state(*state_pointer, output.state) : same_state(*state_pointer, before_state)) &&
                       (fused ? same_matrix(*covariance_pointer, output.covariance)
                              : same_matrix(*covariance_pointer, before_covariance)),
                   "R-MEASUREMENT-FUSION: only fused changes outputs, for all four legal "
                   "alias arrangements");
    __ESBMC_assert(
        (state_alias ? same_state(state_output, old_state_output) : same_state(state, old_state)) &&
            (covariance_alias ? same_matrix(covariance_output, old_covariance_output) : same_matrix(P, old_P)) &&
            same_matrix(measurement, old_measurement) && same_matrix(V, old_V) &&
            bias_proof::same_permissions(permissions, old_permissions),
        "R-MEASUREMENT-FUSION: wrappers preserve measurements, permissions, "
        "distinct prior and unused outputs");
}
int main() { return 0; }
