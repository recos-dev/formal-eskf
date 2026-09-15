/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include "correction_support.hpp"

#ifndef FORMAL_ESKF_PROOF_STEP_CONTRACT
#define FORMAL_ESKF_PROOF_STEP_CONTRACT 0
#endif
#ifndef FORMAL_ESKF_PROOF_STEP_ALIAS
#define FORMAL_ESKF_PROOF_STEP_ALIAS 0
#endif
#ifndef FORMAL_ESKF_PROOF_STEP_FRAME
#define FORMAL_ESKF_PROOF_STEP_FRAME 0
#endif

// F-SOLVE's root observer deliberately requires a positive pivot. A vector
// norm also accepts zero and overflowed squared norms, so use the standard
// scalar-library sqrt boundary for this additional primitive. Neither F-SOLVE nor
// E-CORRECT consumes norm<4>; their actual bodies/storage remain unchanged.
namespace formal_eskf::verification
{
template <>
template <>
inline correction_proof::Scalar
FixedArrayLinalg<correction_proof::Scalar, NoNormObserver<correction_proof::Scalar>, solve_proof::Math>::norm<4U>(
    storage_type<4U, 1U> const & vector) noexcept
{
    return formal_eskf::scalar::StandardMath<correction_proof::Scalar>::sqrt(squared_norm(vector));
}
} // namespace formal_eskf::verification

namespace step_correction_proof
{
using correction_proof::Backend;
using correction_proof::Correction;
using correction_proof::Covariance;
using correction_proof::Jacobian;
using correction_proof::measurement_size;
using correction_proof::Noise;
using correction_proof::Residual;
using correction_proof::same;
using correction_proof::same_matrix;
using correction_proof::Scalar;
using correction_proof::state_size;
using correction_proof::Status;
using Configuration =
    std::conditional_t<state_size == 15U, formal_eskf::configuration::Ins, formal_eskf::configuration::Ahrs>;
using State = Configuration::NominalState<Backend>;
using Error = Configuration::ErrorState<Backend>;
using Quaternion = formal_eskf::so3::UnitQuaternion<Backend>;
using Vector3 = Backend::vector_type<3U>;
using Vector4 = Backend::vector_type<4U>;
using Matrix3 = Backend::matrix_type<3U, 3U>;

inline bool same_state(State const & a, State const & b)
{
    bool valid = same_matrix(a.q_nb.coefficients(), b.q_nb.coefficients());
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
    valid = same_matrix(a.p_n, b.p_n) && same_matrix(a.v_n, b.v_n) && same_matrix(a.b_a, b.b_a) &&
            same_matrix(a.b_g, b.b_g) && valid;
#endif
    return valid;
}

inline bool same_error(Error const & a, Error const & b)
{
    bool valid = same_matrix(a.delta_theta_b, b.delta_theta_b);
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
    valid = same_matrix(a.delta_p_n, b.delta_p_n) && same_matrix(a.delta_v_n, b.delta_v_n) &&
            same_matrix(a.delta_b_a, b.delta_b_a) && same_matrix(a.delta_b_g, b.delta_b_g) && valid;
#endif
    return valid;
}

// Independent raw-coordinate oracle, not another call to unpack_correction.
inline bool unpacked(Error const & error, Correction const & delta)
{
    Scalar expected[state_size]{};
    covariance_oracle::flatten(delta, expected);
    bool valid = true;
    for (std::size_t i = 0U; i < 3U; ++i)
    {
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
        valid = same(error.delta_p_n(i), expected[i]) && same(error.delta_v_n(i), expected[3U + i]) &&
                same(error.delta_theta_b(i), expected[6U + i]) && same(error.delta_b_a(i), expected[9U + i]) &&
                same(error.delta_b_g(i), expected[12U + i]) && valid;
#else
        valid = same(error.delta_theta_b(i), expected[i]) && valid;
#endif
    }
    return valid;
}

constexpr bool boundaries_clear = !FORMAL_ESKF_PROOF_CORRECTION_CONTRACT && !FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT &&
                                  !FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY && !FORMAL_ESKF_PROOF_FINITE_CONTRACT &&
                                  !FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT && !FORMAL_ESKF_PROOF_ACCESS_CONTRACT &&
                                  !FORMAL_ESKF_PROOF_PRODUCT_ENTRY_CONTRACT && !FORMAL_ESKF_PROOF_DOT_CONTRACT;
} // namespace step_correction_proof
