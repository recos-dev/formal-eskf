/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "observations_support.hpp"

namespace observation_proof
{

struct Result
{
    state_type state;
    covariance_type covariance;
    Status status;
};

struct Call
{
    inline static Result const * result = nullptr;
    inline static Model const * model = nullptr;
    inline static state_type const * state = nullptr;
    inline static covariance_type const * covariance = nullptr;
    inline static residual_type const * measurement = nullptr;
    inline static noise_type const * V = nullptr;
    inline static state_type * state_target = nullptr;
    inline static covariance_type * covariance_target = nullptr;
    inline static state_type const * state_before = nullptr;
    inline static covariance_type const * covariance_before = nullptr;
    inline static value_type minimum{};
    inline static unsigned count{};
};

} /* end namespace observation_proof */

#if FORMAL_ESKF_PROOF_OBSERVATION_CONTRACT
namespace formal_eskf
{
// The same backend/state/measurement instantiations are covered by E-CORRECT,
// F-SOLVE and E-STEP-CORRECT. Only public success publication, failure atomicity
// and distinct-input purity are used here; no numerical result is assumed.
template <>
inline Status try_correct<observation_proof::backend_type, observation_proof::measurement_size>(
    observation_proof::state_type const & state, observation_proof::covariance_type const & covariance,
    observation_proof::residual_type const & r, observation_proof::jacobian_type const & H,
    observation_proof::noise_type const & V, observation_proof::value_type minimum,
    observation_proof::state_type & state_output, observation_proof::covariance_type & covariance_output) noexcept
{
    using namespace observation_proof;
    __ESBMC_assert(Call::count++ == 0U, "E-OBS: exactly one correction delegation");
    __ESBMC_assert(same_state(state, *Call::state) && same_matrix(covariance, *Call::covariance) &&
                       same_matrix(V, *Call::V) && same(minimum, Call::minimum),
                   "E-OBS: original state, complete P/V and norm threshold reach correction unchanged");
    __ESBMC_assert(&state_output == Call::state_target && &covariance_output == Call::covariance_target,
                   "E-OBS: correction receives the caller's actual output objects");
    __ESBMC_assert(same_state(state_output, *Call::state_before) &&
                       same_matrix(covariance_output, *Call::covariance_before),
                   "E-OBS: no caller output is published before correction");
    for (std::size_t row = 0U; row < measurement_size; ++row)
    {
        __ESBMC_assert(same(r(row), (*Call::measurement)(row)-Call::model->prediction[row]),
                       "E-OBS: residual is measurement minus the specified prediction, in every coordinate");
        for (std::size_t column = 0U; column < state_size; ++column)
        {
            __ESBMC_assert(same(H(row, column), Call::model->H[row][column]),
                           "E-OBS: every Jacobian coefficient and unobserved zero column matches the model");
        }
    }
    if (Call::result->status == Status::success)
    {
        state_output = Call::result->state;
        covariance_output = Call::result->covariance;
    }
    return Call::result->status;
}
} /* end namespace formal_eskf */
#endif

using observation_proof::covariance_type;
using observation_proof::noise_type;
using observation_proof::residual_type;
using observation_proof::state_type;
using observation_proof::value_type;
using observation_proof::vector_type;

void verify_observation(state_type state, covariance_type covariance, residual_type measurement, noise_type V,
                        vector_type reference, vector_type angular_rate, value_type minimum, state_type state_output,
                        covariance_type covariance_output, observation_proof::Result result,
                        observation_proof::rotation_type rotation_result,
                        observation_proof::ins_jacobian_type terms_result)
{
    using namespace observation_proof;
    constexpr bool valid = configured && model_boundaries && FORMAL_ESKF_PROOF_OBSERVATION_CONTRACT == 1 &&
                           FORMAL_ESKF_PROOF_ALIAS >= 0 && FORMAL_ESKF_PROOF_ALIAS < 4;
    __ESBMC_assert(valid, "Runner error: observation requires exact model/dimensions and only the correction contract");
    if constexpr (!valid)
    {
        return;
    }
    constexpr bool state_alias = (FORMAL_ESKF_PROOF_ALIAS & 1) != 0;
    constexpr bool covariance_alias = (FORMAL_ESKF_PROOF_ALIAS & 2) != 0;
    auto const old_state = state;
    auto const old_covariance = covariance;
    auto const old_measurement = measurement;
    auto const old_V = V;
    auto const old_reference = reference;
    auto const old_angular_rate = angular_rate;
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
        covariance_pointer = &covariance;
    }
    auto const state_before = *state_pointer;
    auto const covariance_before = *covariance_pointer;
    RotationCall::result = rotation_result;
    RotationCall::argument = state.q_nb;
    RotationCall::count = 0U;
    TermsCall::result = terms_result;
    TermsCall::count = 0U;
    Model expected;
    expected_model(old_state, reference, angular_rate, expected);
    TermsCall::expected = &expected;
    Status prefix = input_status(old_state, measurement, V, reference, angular_rate);
    bool const model_eligible = prefix == Status::success;
    if (prefix == Status::success)
    {
        bool finite = expected.intermediate_finite;
        for (std::size_t row = 0U; row < measurement_size; ++row)
        {
            finite = std::isfinite(expected.prediction[row]) &&
                     std::isfinite(measurement(row) - expected.prediction[row]) && finite;
            for (std::size_t column = 0U; column < state_size; ++column)
            {
                finite = std::isfinite(expected.H[row][column]) && finite;
            }
        }
        if (!finite)
        {
            prefix = Status::non_finite_result;
        }
    }
    Call::result = &result;
    Call::model = &expected;
    Call::state = &old_state;
    Call::covariance = &old_covariance;
    Call::measurement = &old_measurement;
    Call::V = &old_V;
    Call::minimum = minimum;
    Call::state_target = state_pointer;
    Call::covariance_target = covariance_pointer;
    Call::state_before = &state_before;
    Call::covariance_before = &covariance_before;
    Call::count = 0U;

#if FORMAL_ESKF_PROOF_OBSERVATION == 0
    Status const status = formal_eskf::try_correct_horizontal_position(state, covariance, measurement, V, minimum,
                                                                       *state_pointer, *covariance_pointer);
#elif FORMAL_ESKF_PROOF_OBSERVATION == 1
    Status const status = formal_eskf::try_correct_vertical_position(state, covariance, measurement(0U), V(0U, 0U),
                                                                     minimum, *state_pointer, *covariance_pointer);
#elif FORMAL_ESKF_PROOF_OBSERVATION == 2
    Status const status = formal_eskf::try_correct_horizontal_velocity(state, covariance, measurement, V, minimum,
                                                                       *state_pointer, *covariance_pointer);
#elif FORMAL_ESKF_PROOF_OBSERVATION == 3
    Status const status = formal_eskf::try_correct_velocity(state, covariance, measurement, V, minimum, *state_pointer,
                                                            *covariance_pointer);
#elif FORMAL_ESKF_PROOF_OBSERVATION == 4 || FORMAL_ESKF_PROOF_OBSERVATION == 5
    Status const status = formal_eskf::try_correct_magnetometer(state, covariance, measurement, reference, V, minimum,
                                                                *state_pointer, *covariance_pointer);
#elif FORMAL_ESKF_PROOF_OBSERVATION == 6
    Status const status = formal_eskf::try_correct_accelerometer(state, covariance, measurement, reference, V, minimum,
                                                                 *state_pointer, *covariance_pointer);
#else
    Status const status = formal_eskf::try_correct_accelerometer(
        state, covariance, measurement, angular_rate, reference, V, minimum, *state_pointer, *covariance_pointer);
#endif
    __ESBMC_assert(status == (prefix == Status::success ? result.status : prefix),
                   "E-OBS: exact validation failure or unchanged correction status");
    __ESBMC_assert(Call::count == (prefix == Status::success ? 1U : 0U),
                   "E-OBS: correction is neither skipped on eligible inputs nor called after model failure");
    __ESBMC_assert(RotationCall::count == (model >= 4U && model_eligible ? 1U : 0U),
                   "E-OBS: rotation executes only after model inputs are validated");
    __ESBMC_assert(TermsCall::count == (model == 7U && model_eligible && expected.intermediate_finite ? 1U : 0U),
                   "E-OBS: INS Jacobian assembly executes only after body-frame terms are finite");
    __ESBMC_assert(status == Status::success
                       ? same_state(*state_pointer, result.state) && same_matrix(*covariance_pointer, result.covariance)
                       : same_state(*state_pointer, state_before) &&
                             same_matrix(*covariance_pointer, covariance_before),
                   "E-OBS: exact correction publication or complete rollback");
    __ESBMC_assert(state_alias ? same_state(state_output, old_state_output) : same_state(state, old_state),
                   "E-OBS: distinct nominal input or unused output stays unchanged");
    __ESBMC_assert(covariance_alias ? same_matrix(covariance_output, old_covariance_output)
                                    : same_matrix(covariance, old_covariance),
                   "E-OBS: distinct prior covariance or unused output stays unchanged");
    __ESBMC_assert(same_matrix(measurement, old_measurement) && same_matrix(V, old_V) &&
                       same_matrix(reference, old_reference) && same_matrix(angular_rate, old_angular_rate),
                   "E-OBS: all sensor/reference inputs remain unchanged");
}

int main() { return 0; }
