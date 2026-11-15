/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "contracts.hpp"
#include "algorithm_support.hpp"
#include <formal_eskf/eskf/accelerometer.hpp>
#include <formal_eskf/eskf/types.hpp>
#include <formal_eskf/runtime/measurement_fusion.hpp>
#define private public
#define max formal_eskf_proof_max
#include <formal_eskf/runtime/accelerometer.hpp>
#undef max
#undef private
#ifndef FORMAL_ESKF_PROOF_SAMPLE_CONTRACT
#define FORMAL_ESKF_PROOF_SAMPLE_CONTRACT 0
#endif
namespace sample_proof
{
using contract_proof::finite_vector;
using contract_proof::Linalg;
using contract_proof::Quaternion;
using contract_proof::same;
using contract_proof::same_vector;
using contract_proof::Scalar;
using formal_eskf::Status;
using Sampler = formal_eskf::runtime::AccelerometerSample<Linalg, 10000U>;
using Vector = Sampler::vector_type;
using Imu = formal_eskf::ImuSample<Linalg>;
inline bool same_double(double a, double b)
{
    return (a == b && (a != 0.0 || std::signbit(a) == std::signbit(b))) || (std::isnan(a) && std::isnan(b));
}
inline bool same_history(Sampler const & a, Sampler const & b)
{
    return a.m_magnitude.initialized() == b.m_magnitude.initialized() &&
           same_vector(a.m_magnitude.filtered_specific_force_b(), b.m_magnitude.filtered_specific_force_b()) &&
           same_vector(a.m_delta_velocity, b.m_delta_velocity) && a.m_interval_us == b.m_interval_us &&
           a.m_samples == b.m_samples && a.m_clipping_mask == b.m_clipping_mask &&
           same_double(a.m_average_period_us, b.m_average_period_us) && a.m_required_samples == b.m_required_samples &&
           same_double(a.m_target_period_us, b.m_target_period_us) &&
           same_double(a.m_minimum_period_us, b.m_minimum_period_us);
}
struct Calls
{
    inline static Vector angle, increment, rotated_prior, rotated_increment;
    inline static Vector const * prior{};
    inline static Quaternion q;
    inline static Scalar minimum{};
    inline static Status status{};
    inline static unsigned exps{}, prior_rotations{}, increment_rotations{};
};
} // namespace sample_proof
#if FORMAL_ESKF_PROOF_SAMPLE_CONTRACT
// SO3-EXP supplies the pure-input frame for this exact backend. The actual
// inverse action's input frame is proved below in the non-summary profile.
// Arbitrary returns overapproximate numerical results, including all failures.
namespace formal_eskf::so3
{
template <>
inline Status exp<sample_proof::Linalg>(sample_proof::Vector const & angle, sample_proof::Scalar minimum,
                                        sample_proof::Quaternion & output)
{
    using namespace sample_proof;
    __ESBMC_assert(Calls::exps++ == 0U && Calls::prior_rotations == 0U && Calls::increment_rotations == 0U &&
                       same_vector(angle, Calls::angle) && same(minimum, Calls::minimum),
                   "R-ACCELEROMETER-FUSION: integrate forwards dt-scaled angular rate and threshold to Exp once");
    output = Calls::q;
    return Calls::status;
}
template <>
inline sample_proof::Vector inverse_rotate<sample_proof::Linalg>(sample_proof::Quaternion const & q,
                                                                 sample_proof::Vector const & vector)
{
    using namespace sample_proof;
    __ESBMC_assert(Calls::exps == 1U && Calls::status == Status::success &&
                       same_vector(q.coefficients(), Calls::q.coefficients()),
                   "R-ACCELEROMETER-FUSION: both inverse actions use the successful Exp quaternion");
    // C++ does not specify the evaluation order of the two additions' operands.
    // Distinguish the two vector objects without requiring either call first.
    if (&vector == Calls::prior)
    {
        __ESBMC_assert(Calls::prior_rotations++ == 0U,
                       "R-ACCELEROMETER-FUSION: prior increment is inverse-rotated exactly once");
        return Calls::rotated_prior;
    }
    __ESBMC_assert(Calls::increment_rotations++ == 0U && same_vector(vector, Calls::increment),
                   "R-ACCELEROMETER-FUSION: newest dt-scaled increment is inverse-rotated exactly once");
    return Calls::rotated_increment;
}
} // namespace formal_eskf::so3
#endif
void verify_sample_initialization(sample_proof::Sampler history)
{
    using namespace sample_proof;
    Sampler fresh;
    Vector zero;
    __ESBMC_assert(
        !fresh.m_magnitude.initialized() && same_vector(fresh.m_magnitude.filtered_specific_force_b(), zero) &&
            same_vector(fresh.m_delta_velocity, zero) && fresh.m_interval_us == 0U && fresh.m_samples == 0U &&
            fresh.m_clipping_mask == 0 && fresh.m_average_period_us == 5000.0 && fresh.m_required_samples == 2U &&
            fresh.m_target_period_us == 10000.0 && fresh.m_minimum_period_us == 5000.0,
        "R-ACCELEROMETER-FUSION: constructor establishes empty increment/filter history and default sampling "
        "thresholds");
    history.reset();
    __ESBMC_assert(
        same_history(history, fresh),
        "R-ACCELEROMETER-FUSION: reset discards every partial interval, average and magnitude history field");
}
void verify_sample_inverse_frame(sample_proof::Quaternion q, sample_proof::Vector vector)
{
    using namespace sample_proof;
    __ESBMC_assert(!FORMAL_ESKF_PROOF_SAMPLE_CONTRACT, "Runner error: inverse action frame must use the actual body");
    auto const old_q = q;
    auto const old_vector = vector;
    auto const result = formal_eskf::so3::inverse_rotate(q, vector);
    (void)result;
    __ESBMC_assert(same_vector(q.coefficients(), old_q.coefficients()) && same_vector(vector, old_vector),
                   "R-ACCELEROMETER-FUSION: actual inverse action preserves both inputs for every IEEE representation");
}
void verify_sample_integration(sample_proof::Sampler history, sample_proof::Imu imu, sample_proof::Scalar dt,
                               sample_proof::Scalar minimum, sample_proof::Vector output, sample_proof::Quaternion q,
                               sample_proof::Status exp_status, sample_proof::Vector rotated_prior,
                               sample_proof::Vector rotated_increment)
{
    using namespace sample_proof;
    __ESBMC_assert(FORMAL_ESKF_PROOF_SAMPLE_CONTRACT, "Runner error: integration requires proved pure callee frames");
    auto const before = history;
    auto const old_imu = imu;
    auto const old_output = output;
    Calls::prior = &history.m_delta_velocity;
    Calls::q = q;
    Calls::status = exp_status;
    Calls::minimum = minimum;
    Calls::rotated_prior = rotated_prior;
    Calls::rotated_increment = rotated_increment;
    Calls::exps = Calls::prior_rotations = Calls::increment_rotations = 0U;
    Vector expected;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        Calls::angle(axis) = imu.angular_rate_b(axis) * dt;
        Calls::increment(axis) = imu.specific_force_b(axis) * dt;
        expected(axis) = rotated_prior(axis) + (Calls::increment(axis) + rotated_increment(axis)) * Scalar{0.5};
    }
    bool const finite = finite_vector(Calls::angle) && finite_vector(Calls::increment);
    Status const status = !finite                         ? Status::non_finite_result
                          : exp_status != Status::success ? exp_status
                          : !finite_vector(expected)      ? Status::non_finite_result
                                                          : Status::success;
    auto const actual = history.integrate(imu, dt, minimum, output);
    __ESBMC_assert(
        actual == status && Calls::exps == (finite ? 1U : 0U) &&
            Calls::prior_rotations == (finite && exp_status == Status::success ? 1U : 0U) &&
            Calls::increment_rotations == Calls::prior_rotations,
        "R-ACCELEROMETER-FUSION: integration validates increments, propagates Exp failure and checks rotated result");
    __ESBMC_assert(
        status == Status::success ? same_vector(output, expected) && finite_vector(output)
                                  : same_vector(output, old_output),
        "R-ACCELEROMETER-FUSION: only finite trapezoidal increments publish; failure preserves caller output");
    __ESBMC_assert(same_history(history, before) && same_vector(imu.specific_force_b, old_imu.specific_force_b) &&
                       same_vector(imu.angular_rate_b, old_imu.angular_rate_b),
                   "R-ACCELEROMETER-FUSION: integration preserves all sample history and both IMU vectors");
}
int main() { return 0; }
