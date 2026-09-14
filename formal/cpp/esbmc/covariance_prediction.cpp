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

#ifndef FORMAL_ESKF_PROOF_INS
#define FORMAL_ESKF_PROOF_INS 0
#endif
#ifndef FORMAL_ESKF_PROOF_PCOV_CONTRACT
#define FORMAL_ESKF_PROOF_PCOV_CONTRACT 0
#endif

namespace pcov_proof
{
using namespace contract_proof;
using namespace covariance_oracle;
static_assert(FORMAL_ESKF_PROOF_INS == 0 || FORMAL_ESKF_PROOF_INS == 1);
static_assert(FORMAL_ESKF_PROOF_ALIAS == 0 || FORMAL_ESKF_PROOF_ALIAS == 1);
using Configuration =
    std::conditional_t<FORMAL_ESKF_PROOF_INS, formal_eskf::configuration::Ins, formal_eskf::configuration::Ahrs>;
using State = Configuration::NominalState<Linalg>;
using Parameters = Configuration::Parameters<Linalg>;
using Noise = Configuration::ProcessNoise<Linalg>;
using Imu = formal_eskf::ImuSample<Linalg>;
constexpr std::size_t size = FORMAL_ESKF_PROOF_INS ? 15U : 3U;
constexpr std::size_t noise_size = FORMAL_ESKF_PROOF_INS ? 12U : 3U;
using Matrix = Linalg::matrix_type<size, size>;
using Matrix3 = Linalg::matrix_type<3U, 3U>;
using NoiseMatrix = Linalg::matrix_type<noise_size, noise_size>;

bool same_noise(Noise const & a, Noise const & b)
{
    bool equal = same_vector(a.angular_rate_variance, b.angular_rate_variance);
#if FORMAL_ESKF_PROOF_INS
    equal = same_vector(a.specific_force_variance, b.specific_force_variance) &&
            same_vector(a.accelerometer_bias_random_walk_variance_density,
                        b.accelerometer_bias_random_walk_variance_density) &&
            same_vector(a.gyroscope_bias_random_walk_variance_density, b.gyroscope_bias_random_walk_variance_density) &&
            equal;
#endif
    return equal;
}

bool same_state(State const & a, State const & b)
{
    bool equal = same_vector(a.q_nb.coefficients(), b.q_nb.coefficients());
#if FORMAL_ESKF_PROOF_INS
    equal = same_vector(a.p_n, b.p_n) && same_vector(a.v_n, b.v_n) && same_vector(a.b_a, b.b_a) &&
            same_vector(a.b_g, b.b_g) && equal;
#endif
    return equal;
}

bool same_parameters(Parameters const & a, Parameters const & b)
{
    bool equal = same_noise(a.process_noise, b.process_noise) && same(a.dt_min, b.dt_min) && same(a.dt_max, b.dt_max) &&
                 same(a.minimum_quaternion_norm, b.minimum_quaternion_norm) &&
                 same(a.quaternion_squared_norm_tolerance, b.quaternion_squared_norm_tolerance);
#if FORMAL_ESKF_PROOF_INS
    equal = same_vector(a.gravity_n, b.gravity_n) && equal;
#endif
    return equal;
}

Status parameter_status(Scalar dt, Parameters const & p)
{
    if (!std::isfinite(dt) || !std::isfinite(p.dt_min) || !std::isfinite(p.dt_max) ||
        !std::isfinite(p.minimum_quaternion_norm))
    {
        return Status::non_finite_input;
    }
    if (p.dt_min <= Scalar{0} || p.dt_max < p.dt_min || p.minimum_quaternion_norm <= Scalar{0} ||
        p.minimum_quaternion_norm > Scalar{1})
    {
        return Status::domain_error;
    }
    return dt < p.dt_min || dt > p.dt_max ? Status::out_of_range : Status::success;
}

// Pure same-type summaries, paired with covariance_transition.cpp,
// process_noise.cpp (OpaqueMath), maps.cpp and E-RESET matrix producers.
// Every return/status is arbitrary; no finite/PSD/success premise is assumed.
// Check arguments AT the call boundary, before control-flow joins duplicate
// floating-point expressions. This is not an assumption or a mocked success.
struct Results
{
    bool covariance_finite, transition_finite;
    Status quaternion_status, noise_status, transition_status, finish_status;
    NoiseMatrix noise;
    Matrix3 transition, rotation, force_product, rotated_noise;
    Matrix propagated, finished;
};
struct Calls
{
    inline static Results const * result = nullptr;
    inline static State const * state = nullptr;
    inline static Parameters const * parameters = nullptr;
    inline static Imu const * imu = nullptr;
    inline static Matrix const * covariance = nullptr;
    inline static Scalar dt{};
    inline static unsigned quaternion = 0U, finite = 0U, noise = 0U, transition = 0U, rotation = 0U, product = 0U,
                           sandwich = 0U, noise_sandwich = 0U, finish = 0U;
    inline static Matrix checked_transition{};
};

void check_candidate(Matrix const & actual)
{
    Scalar candidate[size * size]{}, propagated[size * size]{}, noise[noise_size * noise_size]{};
    flatten(actual, candidate);
    flatten(Calls::result->propagated, propagated);
    flatten(Calls::result->noise, noise);
#if FORMAL_ESKF_PROOF_INS
    Scalar rotated[9]{};
    flatten(Calls::result->rotated_noise, rotated);
#endif
    bool valid = true;
    for (std::size_t row = 0U; row < size; ++row)
    {
        for (std::size_t column = 0U; column < size; ++column)
        {
            Scalar entry = propagated[row * size + column];
#if FORMAL_ESKF_PROOF_INS
            if (row >= 3U && row < 6U && column >= 3U && column < 6U)
                entry += rotated[(row - 3U) * 3U + column - 3U];
            else if (row >= 6U && row / 3U == column / 3U)
                entry += noise[(row - 3U) * noise_size + column - 3U];
#else
            entry += noise[row * size + column];
#endif
            valid = same(candidate[row * size + column], entry) && valid;
        }
    }
    __ESBMC_assert(valid, "E-PCOV: all noise blocks and every unaffected covariance entry reach finalization");
}

#if FORMAL_ESKF_PROOF_INS
void check_transition(Matrix const & actual)
{
    Scalar fx[225]{}, r[9]{}, product[9]{}, transition[9]{};
    flatten(actual, fx);
    flatten(Calls::result->rotation, r);
    flatten(Calls::result->force_product, product);
    flatten(Calls::result->transition, transition);
    Scalar const dt = Calls::dt;
    bool valid = true;
    for (std::size_t row = 0U; row < 15U; ++row)
    {
        for (std::size_t column = 0U; column < 15U; ++column)
        {
            Scalar entry = row == column ? Scalar{1} : Scalar{0};
            if (row < 3U && column >= 3U && column < 6U)
                entry = (row == column - 3U ? Scalar{1} : Scalar{0}) * dt;
            else if (row >= 3U && row < 6U && column >= 6U && column < 9U)
                entry = product[(row - 3U) * 3U + column - 6U] * dt;
            else if (row >= 3U && row < 6U && column >= 9U && column < 12U)
                entry = (-r[(row - 3U) * 3U + column - 9U]) * dt;
            else if (row >= 6U && row < 9U && column >= 6U && column < 9U)
                entry = transition[(row - 6U) * 3U + column - 6U];
            else if (row >= 6U && row < 9U && column >= 12U)
                entry = -((row - 6U == column - 12U ? Scalar{1} : Scalar{0}) * dt);
            valid = same(fx[row * 15U + column], entry) && valid;
        }
    }
    __ESBMC_assert(valid, "E-PCOV: every F_x block, sign, dt evaluation order and untouched zero/identity");
}
#endif
} // namespace pcov_proof

#if FORMAL_ESKF_PROOF_PCOV_CONTRACT
namespace formal_eskf::verification
{
// Same-type E-NOISE storage producer closes this initialization postcondition.
template <>
template <>
inline void FixedArrayLinalg<contract_proof::Scalar, NoNormObserver<contract_proof::Scalar>,
                             contract_proof::OpaqueMath>::set_zero<12U, 12U>(storage_type<12U, 12U> & matrix) noexcept
{
    matrix = {};
}

} // namespace formal_eskf::verification

namespace formal_eskf::detail
{
template <>
inline Status validate_prediction_quaternion<contract_proof::Linalg>(contract_proof::Quaternion const & q,
                                                                     contract_proof::Scalar tolerance) noexcept
{
    using namespace pcov_proof;
    __ESBMC_assert(Calls::quaternion++ == 0U, "E-PCOV contract: one prior quaternion check");
    __ESBMC_assert(same_vector(q.coefficients(), Calls::state->q_nb.coefficients()) &&
                       same(tolerance, Calls::parameters->quaternion_squared_norm_tolerance),
                   "E-PCOV: validate the old quaternion with the requested tolerance");
    return Calls::result->quaternion_status;
}
template <>
inline Status try_attitude_error_transition<contract_proof::Linalg>(contract_proof::Vector3 const & omega,
                                                                    contract_proof::Scalar dt,
                                                                    contract_proof::Scalar minimum,
                                                                    pcov_proof::Matrix3 & output) noexcept
{
    using namespace pcov_proof;
    __ESBMC_assert(Calls::transition++ == 0U, "E-PCOV contract: one attitude transition");
    __ESBMC_assert(same(dt, Calls::dt) && same(minimum, Calls::parameters->minimum_quaternion_norm),
                   "E-PCOV: actual dt and minimum reach the attitude transition");
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        Scalar expected = Calls::imu->angular_rate_b(i);
#if FORMAL_ESKF_PROOF_INS
        expected -= Calls::state->b_g(i);
#endif
        __ESBMC_assert(same(omega(i), expected), "E-PCOV: transition consumes the bias-corrected angular rate");
    }
    if (Calls::result->transition_status == Status::success)
        output = Calls::result->transition;
    return Calls::result->transition_status;
}
template <> inline Status try_finish_covariance(pcov_proof::Matrix & candidate, pcov_proof::Matrix & output) noexcept
{
    using namespace pcov_proof;
    __ESBMC_assert(Calls::finish++ == 0U, "E-PCOV contract: one covariance finalizer");
    check_candidate(candidate);
    if (Calls::result->finish_status == Status::success)
    {
        candidate = Calls::result->finished;
        output = Calls::result->finished;
    }
    return Calls::result->finish_status;
}
} // namespace formal_eskf::detail

namespace formal_eskf
{
template <>
inline Status try_discretize_process_noise<contract_proof::Linalg>(pcov_proof::Noise const & noise,
                                                                   contract_proof::Scalar dt,
                                                                   pcov_proof::NoiseMatrix & output) noexcept
{
    using namespace pcov_proof;
    __ESBMC_assert(Calls::noise++ == 0U, "E-PCOV contract: one process noise discretization");
    __ESBMC_assert(same_noise(noise, Calls::parameters->process_noise) && same(dt, Calls::dt),
                   "E-PCOV: every process noise group and actual dt reach discretization");
    if (Calls::result->noise_status == Status::success)
        output = Calls::result->noise;
    return Calls::result->noise_status;
}
} // namespace formal_eskf

namespace formal_eskf::so3
{
template <> inline pcov_proof::Matrix3 to_rotation_matrix(contract_proof::Quaternion const & q) noexcept
{
    using namespace pcov_proof;
    __ESBMC_assert(Calls::rotation++ == 0U, "E-PCOV contract: one old-state rotation");
    __ESBMC_assert(same_vector(q.coefficients(), Calls::state->q_nb.coefficients()),
                   "E-PCOV: navigation rotation uses the OLD nominal attitude");
    return Calls::result->rotation;
}
} // namespace formal_eskf::so3

namespace formal_eskf::linalg
{
template <> inline bool all_finite(pcov_proof::Matrix const & matrix) noexcept
{
    using namespace pcov_proof;
    __ESBMC_assert(Calls::finite < (FORMAL_ESKF_PROOF_INS ? 2U : 1U),
                   "E-PCOV contract: bounded complete matrix checks");
    if (Calls::finite++ == 0U)
    {
        __ESBMC_assert(same_matrix(matrix, *Calls::covariance), "E-PCOV: check the complete OLD covariance");
        return Calls::result->covariance_finite;
    }
#if FORMAL_ESKF_PROOF_INS
    check_transition(matrix);
    Calls::checked_transition = matrix;
#endif
    return Calls::result->transition_finite;
}
template <>
template <>
inline pcov_proof::Matrix3 Matrix<contract_proof::Linalg, 3U, 3U>::operator*
    <3U>([[maybe_unused]] pcov_proof::Matrix3 const & right) const noexcept
{
    using namespace pcov_proof;
    __ESBMC_assert(Calls::product++ == 0U, "E-PCOV contract: one force Jacobian product");
#if FORMAL_ESKF_PROOF_INS
    Scalar const x = Calls::imu->specific_force_b(0U) - Calls::state->b_a(0U);
    Scalar const y = Calls::imu->specific_force_b(1U) - Calls::state->b_a(1U);
    Scalar const z = Calls::imu->specific_force_b(2U) - Calls::state->b_a(2U);
    Scalar const hat[9] = {Scalar{0}, -z, y, z, Scalar{0}, -x, -y, x, Scalar{0}};
    Scalar r[9]{}, a[9]{}, b[9]{};
    flatten(Calls::result->rotation, r);
    flatten(*this, a);
    flatten(right, b);
    for (std::size_t i = 0U; i < 9U; ++i)
        __ESBMC_assert(same(a[i], -r[i]) && same(b[i], hat[i]),
                       "E-PCOV: force product operands are negative OLD R and hat(a_m-b_a)");
#endif
    return Calls::result->force_product;
}
template <>
inline pcov_proof::Matrix sandwich(pcov_proof::Matrix const & transform, pcov_proof::Matrix const & p) noexcept
{
    using namespace pcov_proof;
    __ESBMC_assert(Calls::sandwich++ == 0U, "E-PCOV contract: one complete covariance sandwich");
    __ESBMC_assert(same_matrix(p, *Calls::covariance), "E-PCOV: propagate the complete OLD covariance");
#if FORMAL_ESKF_PROOF_INS
    __ESBMC_assert(same_matrix(transform, Calls::checked_transition), "E-PCOV: propagate with the checked F_x");
#else
    __ESBMC_assert(same_matrix(transform, Calls::result->transition),
                   "E-PCOV: AHRS propagates with the returned transition");
#endif
    return Calls::result->propagated;
}
#if FORMAL_ESKF_PROOF_INS
template <>
inline pcov_proof::Matrix3 sandwich(pcov_proof::Matrix3 const & transform, pcov_proof::Matrix3 const & p) noexcept
{
    using namespace pcov_proof;
    __ESBMC_assert(Calls::noise_sandwich++ == 0U, "E-PCOV contract: one complete anisotropic noise sandwich");
    __ESBMC_assert(same_matrix(transform, Calls::result->rotation), "E-PCOV: rotate velocity noise with OLD R");
    Scalar actual[9]{}, noise[144]{};
    flatten(p, actual);
    flatten(Calls::result->noise, noise);
    for (std::size_t row = 0U; row < 3U; ++row)
        for (std::size_t column = 0U; column < 3U; ++column)
            __ESBMC_assert(same(actual[row * 3U + column], noise[row * 12U + column]),
                           "E-PCOV: rotate the entire body acceleration noise block without assuming isotropy");
    return Calls::result->rotated_noise;
}
#endif
} // namespace formal_eskf::linalg
#endif

using namespace pcov_proof;

void verify_predict_covariance(State state, Matrix covariance, Imu imu, Scalar dt, Parameters parameters, Matrix output,
                               Results results)
{
    constexpr bool configured = FORMAL_ESKF_PROOF_PCOV_CONTRACT && !FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT &&
                                !FORMAL_ESKF_PROOF_ACCESS_CONTRACT && !FORMAL_ESKF_PROOF_PRODUCT_ENTRY_CONTRACT &&
                                !FORMAL_ESKF_PROOF_DOT_CONTRACT;
    __ESBMC_assert(configured, "Runner error: E-PCOV requires only its explicit caller summaries");
    if constexpr (!configured)
        return;
    auto const state_before = state;
    auto const imu_before = imu;
    auto const parameters_before = parameters;
    auto const covariance_before = covariance;
    Calls::result = &results;
    Calls::state = &state_before;
    Calls::imu = &imu_before;
    Calls::parameters = &parameters_before;
    Calls::covariance = &covariance_before;
    Calls::dt = dt;
#if FORMAL_ESKF_PROOF_ALIAS
    auto & target = covariance;
    (void)output;
#else
    auto & target = output;
#endif
    auto const before = target;
    auto const status = formal_eskf::try_predict_covariance(state, covariance, imu, dt, parameters, target);

    Status expected = parameter_status(dt, parameters_before);
    unsigned quaternion_calls = 0U, finite_calls = 0U, noise_calls = 0U, transition_calls = 0U, rotation_calls = 0U,
             product_calls = 0U, sandwich_calls = 0U, noise_sandwich_calls = 0U, finish_calls = 0U;
#if FORMAL_ESKF_PROOF_INS
    if (expected == Status::success)
    {
        quaternion_calls = 1U;
        expected = results.quaternion_status;
    }
#endif
    if (expected == Status::success)
    {
        finite_calls = 1U;
        bool finite = results.covariance_finite && finite_vector(imu_before.angular_rate_b);
#if FORMAL_ESKF_PROOF_INS
        finite = finite_vector(state_before.q_nb.coefficients()) && finite_vector(state_before.b_a) &&
                 finite_vector(state_before.b_g) && finite_vector(imu_before.specific_force_b) && finite;
#endif
        if (!finite)
            expected = Status::non_finite_input;
    }
#if FORMAL_ESKF_PROOF_INS
    if (expected == Status::success)
    {
        bool finite = true;
        for (std::size_t i = 0U; i < 3U; ++i)
            finite = std::isfinite(imu_before.angular_rate_b(i) - state_before.b_g(i)) &&
                     std::isfinite(imu_before.specific_force_b(i) - state_before.b_a(i)) && finite;
        if (!finite)
            expected = Status::non_finite_result;
    }
#endif
    if (expected == Status::success)
    {
        noise_calls = 1U;
        expected = results.noise_status;
    }
    if (expected == Status::success)
    {
        transition_calls = 1U;
        expected = results.transition_status;
    }
#if FORMAL_ESKF_PROOF_INS
    if (expected == Status::success)
    {
        rotation_calls = 1U;
        product_calls = 1U;
        finite_calls = 2U;
        if (!results.transition_finite)
            expected = Status::non_finite_result;
    }
#endif
    if (expected == Status::success)
    {
        sandwich_calls = 1U;
        finish_calls = 1U;
        noise_sandwich_calls = FORMAL_ESKF_PROOF_INS ? 1U : 0U;
        expected = results.finish_status;
    }
    __ESBMC_assert(status == expected, "E-PCOV: ordered validation, overflow and callee-status propagation");
    __ESBMC_assert(Calls::quaternion == quaternion_calls && Calls::finite == finite_calls &&
                       Calls::noise == noise_calls && Calls::transition == transition_calls &&
                       Calls::rotation == rotation_calls && Calls::product == product_calls &&
                       Calls::sandwich == sandwich_calls && Calls::noise_sandwich == noise_sandwich_calls &&
                       Calls::finish == finish_calls,
                   "E-PCOV: no skipped, repeated or premature callee calls");
    if (status == Status::success)
        __ESBMC_assert(same_matrix(target, results.finished), "E-PCOV: publish every finalized coefficient on success");
    else
        __ESBMC_assert(same_matrix(target, before), "E-PCOV: preserve every output coefficient on any failure");
#if !FORMAL_ESKF_PROOF_ALIAS
    __ESBMC_assert(same_matrix(covariance, covariance_before), "E-PCOV: distinct input covariance is unchanged");
#endif
    __ESBMC_assert(same_state(state, state_before) && same_parameters(parameters, parameters_before) &&
                       same_vector(imu.specific_force_b, imu_before.specific_force_b) &&
                       same_vector(imu.angular_rate_b, imu_before.angular_rate_b),
                   "E-PCOV: complete nominal state, parameters and IMU frame, including unused IEEE fields");
}

int main() { return 0; }
