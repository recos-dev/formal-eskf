/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "contracts.hpp"

#include <formal_eskf/eskf/covariance_prediction.hpp>

#ifndef FORMAL_ESKF_PROOF_TRANSITION_CONTRACT
#define FORMAL_ESKF_PROOF_TRANSITION_CONTRACT 0
#endif

namespace transition_proof
{
using namespace contract_proof;
using Matrix = Linalg::matrix_type<3U, 3U>;

struct ExpContract
{
    inline static Quaternion result{};
    inline static Status status{};
    inline static Vector3 argument{};
    inline static Scalar minimum{};
    inline static unsigned calls = 0U;
};
struct RotationContract
{
    inline static Matrix result{};
    inline static Quaternion argument{};
    inline static unsigned calls = 0U;
};
} // namespace transition_proof

#if FORMAL_ESKF_PROOF_TRANSITION_CONTRACT
namespace formal_eskf::so3
{
// Actual same-type Exp is discharged by maps.cpp and its normalization/math
// producers. No accuracy/unit-norm/finite-success assumption is made here.
template <>
inline Status try_exp<contract_proof::Linalg>(contract_proof::Vector3 const & theta, contract_proof::Scalar minimum,
                                              contract_proof::Quaternion & output) noexcept
{
    using transition_proof::ExpContract;
    __ESBMC_assert(ExpContract::calls++ == 0U, "E-PCOV transition contract: one Exp request");
    ExpContract::argument = theta;
    ExpContract::minimum = minimum;
    if (ExpContract::status == Status::success)
        output = ExpContract::result;
    return ExpContract::status;
}
template <> inline transition_proof::Matrix to_rotation_matrix(contract_proof::Quaternion const & q) noexcept
{
    using transition_proof::RotationContract;
    __ESBMC_assert(RotationContract::calls++ == 0U, "E-PCOV transition contract: one increment rotation");
    RotationContract::argument = q;
    return RotationContract::result;
}
} // namespace formal_eskf::so3
#endif

using namespace transition_proof;

void verify_covariance_transition(Vector3 omega, Scalar dt, Scalar minimum, Matrix output, Quaternion increment,
                                  Status exp_status, Matrix rotation)
{
    constexpr bool configured = FORMAL_ESKF_PROOF_TRANSITION_CONTRACT && !FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT;
    __ESBMC_assert(configured, "Runner error: transition producer requires only Exp and rotation summaries");
    if constexpr (!configured)
        return;
    auto const omega_before = omega;
    auto const before = output;
    ExpContract::result = increment;
    ExpContract::status = exp_status;
    RotationContract::result = rotation;
    auto const status = formal_eskf::detail::try_attitude_error_transition(omega, dt, minimum, output);
    Scalar const x = omega_before(0U) * dt, y = omega_before(1U) * dt, z = omega_before(2U) * dt;
    bool const finite = std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    Status expected = finite ? Status::success : Status::non_finite_result;
    unsigned exp_calls = 0U, rotation_calls = 0U;
#if !ESKF_QUAT_APPROX
    if (finite)
    {
        exp_calls = 1U;
        __ESBMC_assert(same(ExpContract::argument(0U), x) && same(ExpContract::argument(1U), y) &&
                           same(ExpContract::argument(2U), z) && same(ExpContract::minimum, minimum),
                       "E-PCOV transition: full rotation vector and minimum reach Exp");
        expected = exp_status;
        if (exp_status == Status::success)
        {
            rotation_calls = 1U;
            __ESBMC_assert(same_vector(RotationContract::argument.coefficients(), increment.coefficients()),
                           "E-PCOV transition: rotation uses the returned increment");
        }
    }
#endif
    __ESBMC_assert(status == expected && ExpContract::calls == exp_calls && RotationContract::calls == rotation_calls,
                   "E-PCOV transition: overflow, callee failures and mode-specific call counts");
    [[maybe_unused]] Scalar const skew[9] = {Scalar{0}, -z, y, z, Scalar{0}, -x, -y, x, Scalar{0}};
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            Scalar entry = before(row, column);
            if (expected == Status::success)
            {
#if ESKF_QUAT_APPROX
                entry = (row == column ? Scalar{1} : Scalar{0}) - skew[row * 3U + column];
#else
                entry = rotation(column, row);
#endif
            }
            __ESBMC_assert(same(output(row, column), entry),
                           "E-PCOV transition: transpose of R(Exp) or I-hat, with complete failure rollback");
        }
    }
    __ESBMC_assert(same_vector(omega, omega_before), "E-PCOV transition: angular rate input frame");
}

// Separate exact-type producer. No quaternion/unit/finite premise excludes
// IEEE inputs accepted by the unchecked rotation-matrix operation.
void verify_covariance_rotation(Quaternion q)
{
    constexpr bool configured = !FORMAL_ESKF_PROOF_TRANSITION_CONTRACT && !FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT;
    __ESBMC_assert(configured, "Runner error: covariance rotation producer must execute the actual implementation");
    if constexpr (!configured)
        return;
    auto const before = q;
    auto const actual = formal_eskf::so3::to_rotation_matrix(q);
    Scalar const q0 = before.q0(), q1 = before.q1(), q2 = before.q2(), q3 = before.q3();
    Scalar const expected[9] = {q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3, Scalar{2} * (q1 * q2 - q0 * q3),
                                Scalar{2} * (q1 * q3 + q0 * q2),       Scalar{2} * (q1 * q2 + q0 * q3),
                                q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3, Scalar{2} * (q2 * q3 - q0 * q1),
                                Scalar{2} * (q1 * q3 - q0 * q2),       Scalar{2} * (q2 * q3 + q0 * q1),
                                q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3};
    for (std::size_t row = 0U; row < 3U; ++row)
        for (std::size_t column = 0U; column < 3U; ++column)
            __ESBMC_assert(same(actual(row, column), expected[row * 3U + column]),
                           "E-PCOV rotation: all nine scalar-first Hamilton rotation coefficients");
    __ESBMC_assert(same_vector(q.coefficients(), before.coefficients()), "E-PCOV rotation: quaternion input frame");
}

void verify_covariance_quaternion_validation(Quaternion q, Scalar tolerance)
{
    constexpr bool configured = !FORMAL_ESKF_PROOF_TRANSITION_CONTRACT && !FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT;
    __ESBMC_assert(configured, "Runner error: covariance quaternion validation must execute the actual implementation");
    if constexpr (!configured)
        return;
    auto const before = q;
    auto const status = formal_eskf::detail::validate_prediction_quaternion(q, tolerance);
    Status expected = Status::success;
    if (!std::isfinite(tolerance))
        expected = Status::non_finite_input;
    else if (tolerance <= Scalar{0} || tolerance >= Scalar{1})
        expected = Status::domain_error;
    else if (!finite_vector(before.coefficients()))
        expected = Status::non_finite_input;
    else
    {
        Scalar norm{0};
        for (std::size_t i = 0U; i < 4U; ++i)
            norm += before.coefficients()(i) * before.coefficients()(i);
        if (!std::isfinite(norm))
            expected = Status::non_finite_result;
        else if (std::fabs(norm - Scalar{1}) > tolerance)
            expected = Status::invalid_quaternion_norm;
    }
    __ESBMC_assert(status == expected,
                   "E-PCOV validation: tolerance precedence, complete norm arithmetic and acceptance");
    __ESBMC_assert(same_vector(q.coefficients(), before.coefficients()),
                   "E-PCOV validation: no normalization or mutation");
}

int main() { return 0; }
