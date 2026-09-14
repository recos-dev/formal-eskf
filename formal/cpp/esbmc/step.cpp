/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "covariance_oracle.hpp"

#include <type_traits>

#include <formal_eskf/eskf/covariance_prediction.hpp>
#include <formal_eskf/eskf/injection.hpp>

#ifndef FORMAL_ESKF_PROOF_INS
#define FORMAL_ESKF_PROOF_INS 0
#endif
#ifndef FORMAL_ESKF_PROOF_STEP_CONTRACT
#define FORMAL_ESKF_PROOF_STEP_CONTRACT 0
#endif
#ifndef FORMAL_ESKF_PROOF_STEP_FRAME
#define FORMAL_ESKF_PROOF_STEP_FRAME 0
#endif
#ifndef FORMAL_ESKF_PROOF_STEP_ALIAS
#define FORMAL_ESKF_PROOF_STEP_ALIAS 0
#endif

namespace step_proof
{
using namespace contract_proof;
using namespace covariance_oracle;
static_assert(FORMAL_ESKF_PROOF_INS == 0 || FORMAL_ESKF_PROOF_INS == 1);
using Configuration =
    std::conditional_t<FORMAL_ESKF_PROOF_INS, formal_eskf::configuration::Ins, formal_eskf::configuration::Ahrs>;
using State = Configuration::NominalState<Linalg>;
using Error = Configuration::ErrorState<Linalg>;
using Parameters = Configuration::Parameters<Linalg>;
using Imu = formal_eskf::ImuSample<Linalg>;
using Matrix = Linalg::matrix_type<Configuration::error_state_dimension, Configuration::error_state_dimension>;

bool same_state(State const & a, State const & b)
{
    bool equal = same_matrix(a.q_nb.coefficients(), b.q_nb.coefficients());
#if FORMAL_ESKF_PROOF_INS
    equal = same_matrix(a.p_n, b.p_n) && same_matrix(a.v_n, b.v_n) && same_matrix(a.b_a, b.b_a) &&
            same_matrix(a.b_g, b.b_g) && equal;
#endif
    return equal;
}

bool same_error(Error const & a, Error const & b)
{
    bool equal = same_matrix(a.delta_theta_b, b.delta_theta_b);
#if FORMAL_ESKF_PROOF_INS
    equal = same_matrix(a.delta_p_n, b.delta_p_n) && same_matrix(a.delta_v_n, b.delta_v_n) &&
            same_matrix(a.delta_b_a, b.delta_b_a) && same_matrix(a.delta_b_g, b.delta_b_g) && equal;
#endif
    return equal;
}

bool same_parameters(Parameters const & a, Parameters const & b)
{
    bool equal = same_matrix(a.process_noise.angular_rate_variance, b.process_noise.angular_rate_variance) &&
                 same(a.minimum_quaternion_norm, b.minimum_quaternion_norm) && same(a.dt_min, b.dt_min) &&
                 same(a.dt_max, b.dt_max) &&
                 same(a.quaternion_squared_norm_tolerance, b.quaternion_squared_norm_tolerance);
#if FORMAL_ESKF_PROOF_INS
    equal = same_matrix(a.gravity_n, b.gravity_n) &&
            same_matrix(a.process_noise.specific_force_variance, b.process_noise.specific_force_variance) &&
            same_matrix(a.process_noise.accelerometer_bias_random_walk_variance_density,
                        b.process_noise.accelerometer_bias_random_walk_variance_density) &&
            same_matrix(a.process_noise.gyroscope_bias_random_walk_variance_density,
                        b.process_noise.gyroscope_bias_random_walk_variance_density) &&
            equal;
#endif
    return equal;
}

bool same_imu(Imu const & a, Imu const & b)
{
    return same_matrix(a.specific_force_b, b.specific_force_b) && same_matrix(a.angular_rate_b, b.angular_rate_b);
}

bool zero_vector(Vector3 const & v)
{
    auto const & raw = cells(v);
    return same(raw.head, Scalar{0}) && same(raw.tail.head, Scalar{0}) && same(raw.tail.tail.head, Scalar{0});
}

bool zero_error(Error const & error)
{
    bool zero = zero_vector(error.delta_theta_b);
#if FORMAL_ESKF_PROOF_INS
    zero = zero_vector(error.delta_p_n) && zero_vector(error.delta_v_n) && zero_vector(error.delta_b_a) &&
           zero_vector(error.delta_b_g) && zero;
#endif
    return zero;
}

// Universal output-only callees. Even FAILURE writes arbitrary candidate data:
// these transactions must not depend on the callee preserving its own output.
// No success, finite, unit, PSD or numerical-result assumption is made.
// Matching input-purity evidence: verify_nominal_prediction_frame, E-PCOV,
// E-INJECT and E-RESET, all with contract_proof::Linalg and actual storage.
struct Results
{
    State nominal;
    Matrix covariance;
    Status nominal_status, covariance_status;
};

struct Calls
{
    inline static Results const * results = nullptr;
    inline static State const * state = nullptr;
    inline static Matrix const * covariance = nullptr;
    inline static Error const * error = nullptr;
    inline static Imu const * imu = nullptr;
    inline static Parameters const * parameters = nullptr;
    inline static Scalar scalar{};
    inline static State const * state_target = nullptr;
    inline static Matrix const * covariance_target = nullptr;
    inline static Error const * error_target = nullptr;
    inline static State const * state_before = nullptr;
    inline static Matrix const * covariance_before = nullptr;
    inline static Error const * error_before = nullptr;
    inline static unsigned nominal = 0U, covariance_calls = 0U;
};

void check_unpublished()
{
    __ESBMC_assert(same_state(*Calls::state_target, *Calls::state_before) &&
                       same_matrix(*Calls::covariance_target, *Calls::covariance_before),
                   "E-STEP: no state or covariance publication before both callees succeed");
    if (Calls::error_target)
        __ESBMC_assert(same_error(*Calls::error_target, *Calls::error_before),
                       "E-STEP: no error mean reset before both callees succeed");
}

void nominal_request(State const & state, State & output)
{
    __ESBMC_assert(Calls::nominal++ == 0U && Calls::covariance_calls == 0U,
                   "E-STEP: nominal callee executes first and exactly once");
    __ESBMC_assert(same_state(state, *Calls::state), "E-STEP: nominal callee receives the complete old state");
    __ESBMC_assert(&output != &state && &output != Calls::state_target,
                   "E-STEP: nominal callee writes a separate scratch object");
    check_unpublished();
}

void covariance_request(Matrix const & covariance, Matrix & output)
{
    __ESBMC_assert(Calls::nominal == 1U && Calls::results->nominal_status == Status::success &&
                       Calls::covariance_calls++ == 0U,
                   "E-STEP: covariance callee executes once, only after nominal success");
    __ESBMC_assert(same_matrix(covariance, *Calls::covariance), "E-STEP: covariance callee receives complete old P");
    __ESBMC_assert(&output != &covariance && &output != Calls::covariance_target,
                   "E-STEP: covariance callee writes a separate scratch object");
    check_unpublished();
}

constexpr bool boundaries_clear = !FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT && !FORMAL_ESKF_PROOF_ACCESS_CONTRACT &&
                                  !FORMAL_ESKF_PROOF_PRODUCT_ENTRY_CONTRACT && !FORMAL_ESKF_PROOF_DOT_CONTRACT;
} // namespace step_proof

#if FORMAL_ESKF_PROOF_STEP_CONTRACT
namespace formal_eskf
{
template <>
inline Status try_predict_nominal<contract_proof::Linalg>(step_proof::State const & state, step_proof::Imu const & imu,
                                                          contract_proof::Scalar dt,
                                                          step_proof::Parameters const & parameters,
                                                          step_proof::State & output) noexcept
{
    using namespace step_proof;
    nominal_request(state, output);
    __ESBMC_assert(same_imu(imu, *Calls::imu) && same_parameters(parameters, *Calls::parameters) &&
                       same(dt, Calls::scalar),
                   "E-STEP: nominal prediction receives unchanged IMU, parameters and dt");
    output = Calls::results->nominal;
    return Calls::results->nominal_status;
}

template <>
inline Status try_predict_covariance<contract_proof::Linalg>(step_proof::State const & state,
                                                             step_proof::Matrix const & covariance,
                                                             step_proof::Imu const & imu, contract_proof::Scalar dt,
                                                             step_proof::Parameters const & parameters,
                                                             step_proof::Matrix & output) noexcept
{
    using namespace step_proof;
    covariance_request(covariance, output);
    __ESBMC_assert(same_state(state, *Calls::state) && same_imu(imu, *Calls::imu) &&
                       same_parameters(parameters, *Calls::parameters) && same(dt, Calls::scalar),
                   "E-STEP: covariance prediction receives OLD state and the same IMU, parameters and dt");
    output = Calls::results->covariance;
    return Calls::results->covariance_status;
}

template <>
inline Status
try_inject_nominal<contract_proof::Linalg>(step_proof::State const & state, step_proof::Error const & error,
                                           contract_proof::Scalar minimum, step_proof::State & output) noexcept
{
    using namespace step_proof;
    nominal_request(state, output);
    __ESBMC_assert(same_error(error, *Calls::error) && same(minimum, Calls::scalar),
                   "E-STEP: injection receives the complete pre-reset error and unchanged threshold");
    output = Calls::results->nominal;
    return Calls::results->nominal_status;
}

template <>
inline Status try_reset_covariance<contract_proof::Linalg>(step_proof::Error const & error,
                                                           step_proof::Matrix const & covariance,
                                                           step_proof::Matrix & output) noexcept
{
    using namespace step_proof;
    covariance_request(covariance, output);
    __ESBMC_assert(same_error(error, *Calls::error), "E-STEP: covariance reset receives the error BEFORE zeroing");
    output = Calls::results->covariance;
    return Calls::results->covariance_status;
}
} // namespace formal_eskf
#endif

namespace step_proof
{
struct LeafResults
{
    Quaternion exponential, composition, constructor;
    Status exponential_status, composition_status, constructor_status;
};
inline LeafResults const * leaves = nullptr;
} // namespace step_proof

#if FORMAL_ESKF_PROOF_STEP_FRAME
namespace formal_eskf::so3
{
// Leaf summaries need only output-local writes, not mathematical correctness.
// The same-type actual vector constructor, Exp and composition producers are
// retained dependencies (normalization.cpp and maps.cpp). No attitude helper,
// nominal wrapper, validation or translation arithmetic is summarized here.
template <>
inline Status UnitQuaternion<contract_proof::Linalg>::try_from_coefficients(vector4_type const &, value_type,
                                                                            UnitQuaternion & output) noexcept
{
    output = step_proof::leaves->constructor;
    return step_proof::leaves->constructor_status;
}
template <>
inline Status try_exp<contract_proof::Linalg>(contract_proof::Vector3 const &, contract_proof::Scalar,
                                              contract_proof::Quaternion & output) noexcept
{
    output = step_proof::leaves->exponential;
    return step_proof::leaves->exponential_status;
}
template <>
inline Status try_compose_normalized<contract_proof::Linalg>(contract_proof::Quaternion const &,
                                                             contract_proof::Quaternion const &, contract_proof::Scalar,
                                                             contract_proof::Quaternion & output) noexcept
{
    output = step_proof::leaves->composition;
    return step_proof::leaves->composition_status;
}
} // namespace formal_eskf::so3
#endif

using namespace step_proof;

template <unsigned Aliases>
void prediction_transaction(State state, Matrix covariance, Imu imu, Scalar dt, Parameters parameters,
                            State state_output, Matrix covariance_output, Results results)
{
    static_assert(Aliases < 4U);
    constexpr bool state_alias = (Aliases & 1U) != 0U;
    constexpr bool covariance_alias = (Aliases & 2U) != 0U;
    constexpr bool configured = FORMAL_ESKF_PROOF_STEP_CONTRACT && !FORMAL_ESKF_PROOF_STEP_FRAME && boundaries_clear;
    __ESBMC_assert(configured, "Runner error: step caller requires only component summaries");
    if constexpr (!configured)
        return;
    auto const old_state = state;
    auto const old_covariance = covariance;
    auto const old_imu = imu;
    auto const old_parameters = parameters;
    auto const old_state_output = state_output;
    auto const old_covariance_output = covariance_output;
    State * state_pointer = &state_output;
    Matrix * covariance_pointer = &covariance_output;
    if (state_alias)
        state_pointer = &state;
    if (covariance_alias)
        covariance_pointer = &covariance;
    State & target_state = *state_pointer;
    Matrix & target_covariance = *covariance_pointer;
    auto const before_state = target_state;
    auto const before_covariance = target_covariance;
    Calls::results = &results;
    Calls::state = &old_state;
    Calls::covariance = &old_covariance;
    Calls::imu = &old_imu;
    Calls::parameters = &old_parameters;
    Calls::scalar = dt;
    Calls::state_target = &target_state;
    Calls::covariance_target = &target_covariance;
    Calls::error_target = nullptr;
    Calls::state_before = &before_state;
    Calls::covariance_before = &before_covariance;
    Calls::nominal = Calls::covariance_calls = 0U;

    auto const status =
        formal_eskf::try_predict(state, covariance, imu, dt, parameters, target_state, target_covariance);
    auto const expected =
        results.nominal_status == Status::success ? results.covariance_status : results.nominal_status;
    __ESBMC_assert(status == expected, "E-STEP: prediction returns the first failure or success");
    __ESBMC_assert(Calls::nominal == 1U &&
                       Calls::covariance_calls == (results.nominal_status == Status::success ? 1U : 0U),
                   "E-STEP: prediction calls neither skip nor repeat a stage");
    if (expected == Status::success)
        __ESBMC_assert(same_state(target_state, results.nominal) && same_matrix(target_covariance, results.covariance),
                       "E-STEP: prediction publishes every state and covariance coefficient on success");
    else
        __ESBMC_assert(same_state(target_state, before_state) && same_matrix(target_covariance, before_covariance),
                       "E-STEP: prediction preserves every output on either failure");
    if (state_alias)
        __ESBMC_assert(same_state(state_output, old_state_output), "E-STEP: unused state output is unchanged");
    else
        __ESBMC_assert(same_state(state, old_state), "E-STEP: distinct input state is unchanged");
    if (covariance_alias)
        __ESBMC_assert(same_matrix(covariance_output, old_covariance_output),
                       "E-STEP: unused covariance output is unchanged");
    else
        __ESBMC_assert(same_matrix(covariance, old_covariance), "E-STEP: distinct input covariance is unchanged");
    __ESBMC_assert(same_imu(imu, old_imu) && same_parameters(parameters, old_parameters),
                   "E-STEP: prediction preserves every IMU and parameter field");
}

template <unsigned Aliases>
void injection_reset_transaction(State state, Matrix covariance, Error error, Scalar minimum, State state_output,
                                 Matrix covariance_output, Error error_output, Results results)
{
    static_assert(Aliases < 8U);
    constexpr bool state_alias = (Aliases & 1U) != 0U;
    constexpr bool covariance_alias = (Aliases & 2U) != 0U;
    constexpr bool error_alias = (Aliases & 4U) != 0U;
    constexpr bool configured = FORMAL_ESKF_PROOF_STEP_CONTRACT && !FORMAL_ESKF_PROOF_STEP_FRAME && boundaries_clear;
    __ESBMC_assert(configured, "Runner error: step caller requires only component summaries");
    if constexpr (!configured)
        return;
    auto const old_state = state;
    auto const old_covariance = covariance;
    auto const old_error = error;
    auto const old_state_output = state_output;
    auto const old_covariance_output = covariance_output;
    auto const old_error_output = error_output;
    State * state_pointer = &state_output;
    Matrix * covariance_pointer = &covariance_output;
    if (state_alias)
        state_pointer = &state;
    if (covariance_alias)
        covariance_pointer = &covariance;
    State & target_state = *state_pointer;
    Matrix & target_covariance = *covariance_pointer;
    Error * error_pointer = &error_output;
    if (error_alias)
        error_pointer = &error;
    Error & target_error = *error_pointer;
    auto const before_state = target_state;
    auto const before_covariance = target_covariance;
    auto const before_error = target_error;
    Calls::results = &results;
    Calls::state = &old_state;
    Calls::covariance = &old_covariance;
    Calls::error = &old_error;
    Calls::scalar = minimum;
    Calls::state_target = &target_state;
    Calls::covariance_target = &target_covariance;
    Calls::error_target = &target_error;
    Calls::state_before = &before_state;
    Calls::covariance_before = &before_covariance;
    Calls::error_before = &before_error;
    Calls::nominal = Calls::covariance_calls = 0U;

    auto const status = formal_eskf::try_inject_and_reset(state, covariance, error, minimum, target_state,
                                                          target_covariance, target_error);
    auto const expected =
        results.nominal_status == Status::success ? results.covariance_status : results.nominal_status;
    __ESBMC_assert(status == expected, "E-STEP: injection/reset returns the first failure or success");
    __ESBMC_assert(Calls::nominal == 1U &&
                       Calls::covariance_calls == (results.nominal_status == Status::success ? 1U : 0U),
                   "E-STEP: injection/reset calls neither skip nor repeat a stage");
    if (expected == Status::success)
        __ESBMC_assert(
            same_state(target_state, results.nominal) && same_matrix(target_covariance, results.covariance) &&
                zero_error(target_error),
            "E-STEP: injection/reset publishes every state/covariance coefficient and zeros every error mean");
    else
        __ESBMC_assert(same_state(target_state, before_state) && same_matrix(target_covariance, before_covariance) &&
                           same_error(target_error, before_error),
                       "E-STEP: injection/reset preserves all three outputs on either failure");
    if (state_alias)
        __ESBMC_assert(same_state(state_output, old_state_output), "E-STEP: unused state output is unchanged");
    else
        __ESBMC_assert(same_state(state, old_state), "E-STEP: distinct input state is unchanged");
    if (covariance_alias)
        __ESBMC_assert(same_matrix(covariance_output, old_covariance_output),
                       "E-STEP: unused covariance output is unchanged");
    else
        __ESBMC_assert(same_matrix(covariance, old_covariance), "E-STEP: distinct input covariance is unchanged");
    if (error_alias)
        __ESBMC_assert(same_error(error_output, old_error_output), "E-STEP: unused error output is unchanged");
    else
        __ESBMC_assert(same_error(error, old_error), "E-STEP: distinct input error is unchanged");
}

void verify_prediction_step(State state, Matrix covariance, Imu imu, Scalar dt, Parameters parameters,
                            State state_output, Matrix covariance_output, Results results)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_STEP_ALIAS >= 0 && FORMAL_ESKF_PROOF_STEP_ALIAS < 4,
                   "Runner error: prediction has exactly four alias arrangements");
    prediction_transaction<(FORMAL_ESKF_PROOF_STEP_ALIAS & 3)>(state, covariance, imu, dt, parameters, state_output,
                                                               covariance_output, results);
}

void verify_injection_reset_step(State state, Matrix covariance, Error error, Scalar minimum, State state_output,
                                 Matrix covariance_output, Error error_output, Results results)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_STEP_ALIAS >= 0 && FORMAL_ESKF_PROOF_STEP_ALIAS < 8,
                   "Runner error: injection/reset has exactly eight alias arrangements");
    injection_reset_transaction<(FORMAL_ESKF_PROOF_STEP_ALIAS & 7)>(state, covariance, error, minimum, state_output,
                                                                    covariance_output, error_output, results);
}

void verify_nominal_prediction_frame(State state, Imu imu, Scalar dt, Parameters parameters, State output,
                                     LeafResults leaf_results)
{
    constexpr bool configured = FORMAL_ESKF_PROOF_STEP_FRAME && !FORMAL_ESKF_PROOF_STEP_CONTRACT && boundaries_clear;
    __ESBMC_assert(configured, "Runner error: nominal frame producer requires only quaternion leaf summaries");
    if constexpr (!configured)
        return;
#if FORMAL_ESKF_PROOF_STEP_FRAME
    auto const old_state = state;
    auto const old_imu = imu;
    auto const old_parameters = parameters;
    leaves = &leaf_results;
    (void)formal_eskf::try_predict_nominal(state, imu, dt, parameters, output);
    __ESBMC_assert(same_state(state, old_state) && same_imu(imu, old_imu) &&
                       same_parameters(parameters, old_parameters),
                   "E-STEP producer: actual same-type nominal prediction preserves every input field");
#else
    (void)state;
    (void)imu;
    (void)dt;
    (void)parameters;
    (void)output;
    (void)leaf_results;
#endif
}

int main() { return 0; }
