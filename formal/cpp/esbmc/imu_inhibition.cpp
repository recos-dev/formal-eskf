/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "contracts.hpp"
#include "algorithm_support.hpp"
#include <algorithm>
#include <formal_eskf/eskf/configuration/ins.hpp>
#include <formal_eskf/eskf/types.hpp>
// All dependencies are loaded first; expose only the target's private producer
// and state for inductive checks, without altering any production body/layout.
#define private public
#define max formal_eskf_proof_max
#include <formal_eskf/runtime/imu.hpp>
#undef max
#undef private
#ifndef FORMAL_ESKF_PROOF_IMU_CALLER
#define FORMAL_ESKF_PROOF_IMU_CALLER 0
#endif
#ifndef FORMAL_ESKF_PROOF_IMU_PEAK_PROPERTY
#define FORMAL_ESKF_PROOF_IMU_PEAK_PROPERTY 0
#endif
using contract_proof::finite_vector;
using contract_proof::Linalg;
using contract_proof::OpaqueMath;
using contract_proof::ordered_squared_norm;
using contract_proof::Quaternion;
using contract_proof::same;
using contract_proof::same_vector;
using contract_proof::Scalar;
using contract_proof::Status;
using contract_proof::Vector3;
using Policy = formal_eskf::runtime::ImuBiasInhibition<Linalg>;
using State = Policy::state_type;
using Matrix3 = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
using Imu = formal_eskf::ImuSample<Linalg>;
using Permissions = formal_eskf::configuration::Ins::BiasUpdate;

bool same_matrix(Matrix3 const & a, Matrix3 const & b)
{
    bool equal = true;
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        for (std::size_t j = 0U; j < 3U; ++j)
        {
            equal = same(a(i, j), b(i, j)) && equal;
        }
    }
    return equal;
}
bool finite_matrix(Matrix3 const & a)
{
    bool finite = true;
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        for (std::size_t j = 0U; j < 3U; ++j)
        {
            finite = std::isfinite(a(i, j)) && finite;
        }
    }
    return finite;
}
bool same_parameters(Policy::Parameters const & a, Policy::Parameters const & b)
{
    return same(a.maximum_specific_force, b.maximum_specific_force) &&
           same(a.maximum_angular_rate, b.maximum_angular_rate) && same(a.decay_time_constant, b.decay_time_constant) &&
           a.accelerometer_enabled == b.accelerometer_enabled && a.gyroscope_enabled == b.gyroscope_enabled;
}
bool same_state(State const & a, State const & b)
{
    return same_vector(a.q_nb.coefficients(), b.q_nb.coefficients()) && same_vector(a.p_n, b.p_n) &&
           same_vector(a.v_n, b.v_n) && same_vector(a.b_a, b.b_a) && same_vector(a.b_g, b.b_g);
}
bool valid_peak(Scalar value) { return std::isfinite(value) && value >= Scalar{0}; }
bool permissions_match(Permissions const & result, Policy::Parameters const & parameters, Matrix3 const & rotation,
                       bool high, Policy::Conditions const & conditions)
{
    bool valid = true;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        bool observable =
            conditions.at_rest ||
            (!conditions.fake_height && (!conditions.fake_position || rotation(2U, axis) > static_cast<Scalar>(0.966) ||
                                         rotation(2U, axis) < -static_cast<Scalar>(0.966)));
        bool const expected = parameters.accelerometer_enabled && !high && !conditions.bad_vertical_acceleration &&
                              observable &&
                              ((static_cast<unsigned>(conditions.accelerometer_clipping) >> axis) & 1U) == 0U;
        valid =
            valid && result.accelerometer[axis] == expected && result.gyroscope[axis] == parameters.gyroscope_enabled;
    }
    return valid;
}
struct Calls
{
    inline static State state{};
    inline static Imu imu{};
    inline static Matrix3 rotation{};
    inline static Scalar force{}, rate{}, force_peak{}, rate_peak{}, old_force{}, old_rate{}, force_interval{},
        rate_interval{};
    inline static unsigned norms{}, rotations{}, peaks{};
};
#if FORMAL_ESKF_PROOF_IMU_CALLER
namespace formal_eskf::linalg
{
template <> inline Scalar norm<Linalg, 3U>(Vector3 const & input)
{
    __ESBMC_assert(Calls::norms < 2U && Calls::rotations == 0U && Calls::peaks == 0U,
                   "IMU inhibition: two corrected magnitudes precede rotation and peak updates");
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        if (Calls::norms == 0U)
        {
            __ESBMC_assert(same(input(i), Calls::imu.specific_force_b(i) - Calls::state.b_a(i)),
                           "IMU inhibition: force subtracts the prior accelerometer bias exactly once");
        }
        else
        {
            __ESBMC_assert(same(input(i), Calls::imu.angular_rate_b(i) - Calls::state.b_g(i)),
                           "IMU inhibition: rate subtracts the prior gyro bias exactly once");
        }
    }
    return Calls::norms++ == 0U ? Calls::force : Calls::rate;
}
} // namespace formal_eskf::linalg
namespace formal_eskf::so3
{
template <> inline Matrix3 to_rotation_matrix<Linalg>(Quaternion const & input)
{
    __ESBMC_assert(Calls::norms == 2U && Calls::rotations++ == 0U && Calls::peaks == 0U &&
                       same_vector(input.coefficients(), Calls::state.q_nb.coefficients()),
                   "IMU inhibition: one rotation uses the unchanged prior quaternion");
    return Calls::rotation;
}
} // namespace formal_eskf::so3
namespace formal_eskf::runtime
{
template <>
inline Scalar ImuBiasInhibition<Linalg>::decayed_peak(Scalar magnitude, Scalar previous, Scalar interval) const
{
    __ESBMC_assert(Calls::norms == 2U && Calls::rotations == 1U && Calls::peaks < 2U && std::isfinite(magnitude) &&
                       valid_peak(previous) && std::isfinite(interval) && interval > Scalar{0} &&
                       std::isfinite(m_parameters.decay_time_constant) && m_parameters.decay_time_constant > Scalar{0},
                   "IMU inhibition: caller establishes every peak-producer premise");
    bool const force = Calls::peaks++ == 0U;
    __ESBMC_assert(force ? (same(magnitude, Calls::force) && same(previous, Calls::old_force) &&
                            same(interval, Calls::force_interval))
                         : (same(magnitude, Calls::rate) && same(previous, Calls::old_rate) &&
                            same(interval, Calls::rate_interval)),
                   "IMU inhibition: each sensor uses its own prior peak and integration period");
    return force ? Calls::force_peak : Calls::rate_peak;
}
} // namespace formal_eskf::runtime
#endif

#if !FORMAL_ESKF_PROOF_IMU_CALLER
void verify_imu_norm(Vector3 vector, Scalar root)
{
    auto const before = vector;
    OpaqueMath::prepare(root, Scalar{0}, Scalar{0}, Scalar{0});
    auto const result = formal_eskf::linalg::norm(vector);
    __ESBMC_assert(OpaqueMath::sqrt.calls == 1U && same(OpaqueMath::sqrt.argument, ordered_squared_norm(before)) &&
                       same(result, root) && same_vector(vector, before),
                   "IMU inhibition: actual norm producer, ordered reduction, root boundary and input frame");
}
void verify_imu_rotation(Quaternion q)
{
    auto const before = q;
    Scalar const q0 = q.q0(), q1 = q.q1(), q2 = q.q2(), q3 = q.q3();
    Matrix3 expected;
    expected(0U, 0U) = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    expected(0U, 1U) = Scalar{2} * (q1 * q2 - q0 * q3);
    expected(0U, 2U) = Scalar{2} * (q1 * q3 + q0 * q2);
    expected(1U, 0U) = Scalar{2} * (q1 * q2 + q0 * q3);
    expected(1U, 1U) = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
    expected(1U, 2U) = Scalar{2} * (q2 * q3 - q0 * q1);
    expected(2U, 0U) = Scalar{2} * (q1 * q3 - q0 * q2);
    expected(2U, 1U) = Scalar{2} * (q2 * q3 + q0 * q1);
    expected(2U, 2U) = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    auto const result = formal_eskf::so3::to_rotation_matrix(q);
    __ESBMC_assert(same_matrix(result, expected) && same_vector(q.coefficients(), before.coefficients()),
                   "IMU inhibition: actual same-backend rotation producer and input frame");
}
void verify_imu_peak(Policy::Parameters parameters, Scalar previous, Scalar magnitude, Scalar interval)
{
    __ESBMC_assume(valid_peak(previous) && std::isfinite(magnitude) && std::isfinite(interval) &&
                   interval > Scalar{0} && std::isfinite(parameters.decay_time_constant) &&
                   parameters.decay_time_constant > Scalar{0});
    Policy policy(parameters);
    auto const result = policy.decayed_peak(magnitude, previous, interval);
    Scalar const factor =
        interval < parameters.decay_time_constant ? Scalar{1} - interval / parameters.decay_time_constant : Scalar{0};
    Scalar const carried = factor * previous;
    Scalar const expected = carried > magnitude ? carried : magnitude;
    static_assert(FORMAL_ESKF_PROOF_IMU_PEAK_PROPERTY == 0 || FORMAL_ESKF_PROOF_IMU_PEAK_PROPERTY == 1);
    if constexpr (FORMAL_ESKF_PROOF_IMU_PEAK_PROPERTY == 0)
    {
        __ESBMC_assert(same(result, expected), "IMU inhibition: exact clamped decay/max");
    }
    else
    {
        __ESBMC_assert(valid_peak(result), "IMU inhibition: inductive finite nonnegative peak");
        __ESBMC_assert(result >= magnitude, "IMU inhibition: peak retains current magnitude");
    }
    __ESBMC_assert(same_parameters(policy.m_parameters, parameters), "IMU inhibition: peak preserves parameters");
}
#endif
void verify_imu_max_model(Scalar first, Scalar second, bool alias)
{
    auto const before_first = first, before_second = second;
    auto const & right = alias ? first : second;
    auto const & result = std::formal_eskf_proof_max(first, right);
    __ESBMC_assert(first < right ? &result == &right : &result == &first,
                   "IMU max model: select second only on strict less-than, otherwise retain first reference");
    __ESBMC_assert(same(first, before_first) && same(second, before_second),
                   "IMU max model: preserve both inputs, including signed zero and NaN classification");
}
void verify_imu_permissions(Policy::Parameters parameters, Matrix3 rotation, bool high, Policy::Conditions conditions)
{
    Policy policy(parameters);
    auto const before = rotation;
    auto const actual = policy.learning_permissions(rotation, high, conditions);
    __ESBMC_assert(
        permissions_match(actual, parameters, rotation, high, conditions) && same_matrix(rotation, before) &&
            same_parameters(policy.m_parameters, parameters),
        "IMU inhibition: all mode priorities, clipping bits, signed alignment threshold and independent gyro enable");
}
#if FORMAL_ESKF_PROOF_IMU_CALLER
void verify_imu_inhibition(Policy::Parameters parameters, Scalar previous_force, Scalar previous_rate,
                           Scalar angular_interval, Scalar force_interval, State state, Imu imu,
                           Policy::Conditions conditions, Scalar force, Scalar rate, Matrix3 rotation,
                           Scalar next_force, Scalar next_rate)
{
    Policy policy(parameters);
    __ESBMC_assert(same(policy.m_specific_force_peak, Scalar{0}) && same(policy.m_angular_rate_peak, Scalar{0}) &&
                       same_parameters(policy.m_parameters, parameters),
                   "IMU inhibition: constructor establishes base invariant");
    __ESBMC_assume(valid_peak(previous_force) && valid_peak(previous_rate));
    policy.m_specific_force_peak = previous_force;
    policy.m_angular_rate_peak = previous_rate;
    auto const old_state = state;
    auto const old_imu = imu;
    auto const old_conditions = conditions;
    Calls::state = state;
    Calls::imu = imu;
    Calls::rotation = rotation;
    Calls::force = force;
    Calls::rate = rate;
    Calls::old_force = previous_force;
    Calls::old_rate = previous_rate;
    Calls::force_interval = force_interval;
    Calls::rate_interval = angular_interval;
    Calls::norms = Calls::rotations = Calls::peaks = 0U;
    // Postcondition of the actual peak producer, proved in both scalar formats.
    __ESBMC_assume(valid_peak(next_force) && valid_peak(next_rate));
    Calls::force_peak = next_force;
    Calls::rate_peak = next_rate;
    bool const finite = std::isfinite(angular_interval) && std::isfinite(force_interval) &&
                        std::isfinite(parameters.maximum_specific_force) &&
                        std::isfinite(parameters.maximum_angular_rate) &&
                        std::isfinite(parameters.decay_time_constant) && finite_vector(imu.specific_force_b) &&
                        finite_vector(imu.angular_rate_b) && finite_vector(state.b_a) && finite_vector(state.b_g);
    bool const domain = angular_interval > Scalar{0} && force_interval > Scalar{0} &&
                        parameters.maximum_specific_force > Scalar{0} && parameters.maximum_angular_rate > Scalar{0} &&
                        parameters.decay_time_constant > Scalar{0} && conditions.accelerometer_clipping >= 0 &&
                        conditions.accelerometer_clipping <= 7;
    Status const expected = !finite   ? Status::non_finite_input
                            : !domain ? Status::domain_error
                            : !std::isfinite(force) || !std::isfinite(rate) || !finite_matrix(rotation)
                                ? Status::non_finite_result
                                : Status::success;
    auto const result = policy.update(angular_interval, force_interval, state, imu, conditions);
    __ESBMC_assert(result.status == expected && Calls::norms == (finite && domain ? 2U : 0U) &&
                       Calls::rotations == (finite && domain ? 1U : 0U) &&
                       Calls::peaks == (expected == Status::success ? 2U : 0U),
                   "IMU inhibition: complete input validation, exact delegation order and failure status");
    if (expected == Status::success)
    {
        bool const high =
            !(next_force <= parameters.maximum_specific_force && next_rate <= parameters.maximum_angular_rate);
        __ESBMC_assert(
            same(result.specific_force_peak, next_force) && same(result.angular_rate_peak, next_rate) &&
                same(policy.m_specific_force_peak, next_force) && same(policy.m_angular_rate_peak, next_rate) &&
                result.high_dynamics == high &&
                permissions_match(result.bias_update, parameters, rotation, high, conditions),
            "IMU inhibition: publish both new peaks and same-horizon learning permissions; equality permits learning");
    }
    else
    {
        __ESBMC_assert(same(policy.m_specific_force_peak, previous_force) &&
                           same(policy.m_angular_rate_peak, previous_rate) &&
                           same(result.specific_force_peak, Scalar{0}) && same(result.angular_rate_peak, Scalar{0}) &&
                           result.high_dynamics,
                       "IMU inhibition: all failures preserve history and return fresh invalid peak diagnostics");
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            __ESBMC_assert(!result.bias_update.accelerometer[axis] && !result.bias_update.gyroscope[axis],
                           "IMU inhibition: failure disables every learning permission");
        }
    }
    __ESBMC_assert(valid_peak(policy.m_specific_force_peak) && valid_peak(policy.m_angular_rate_peak),
                   "IMU inhibition: every public update preserves the inductive peak invariant");
    __ESBMC_assert(same_state(state, old_state) && same_vector(imu.specific_force_b, old_imu.specific_force_b) &&
                       same_vector(imu.angular_rate_b, old_imu.angular_rate_b) &&
                       same_parameters(policy.m_parameters, parameters) &&
                       conditions.at_rest == old_conditions.at_rest &&
                       conditions.fake_height == old_conditions.fake_height &&
                       conditions.fake_position == old_conditions.fake_position &&
                       conditions.bad_vertical_acceleration == old_conditions.bad_vertical_acceleration &&
                       conditions.accelerometer_clipping == old_conditions.accelerometer_clipping,
                   "IMU inhibition: all borrowed inputs and configuration unchanged");
}
#endif
int main() { return 0; }
