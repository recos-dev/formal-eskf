/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "support.hpp"

#ifndef FORMAL_ESKF_PROOF_BINARY64
#define FORMAL_ESKF_PROOF_BINARY64 0
#endif
#ifndef FORMAL_ESKF_PROOF_AXIS
#define FORMAL_ESKF_PROOF_AXIS 0
#endif
#ifndef FORMAL_ESKF_PROOF_ROTATION_CONTRACT
#define FORMAL_ESKF_PROOF_ROTATION_CONTRACT 0
#endif

namespace rotation_proof
{

#if FORMAL_ESKF_PROOF_BINARY64
using Scalar = double;
#else
using Scalar = float;
#endif
using Linalg = formal_eskf::verification::FixedArrayLinalg<Scalar>;
using Quaternion = formal_eskf::so3::UnitQuaternion<Linalg>;
using Matrix = Linalg::matrix_type<3U, 3U>;
using Vector = Linalg::vector_type<3U>;

// Preserve IEEE value and zero sign; NaN payloads are outside ESBMC's model.
bool same_scalar(Scalar a, Scalar b)
{
    return (a == b && (a != Scalar{0} || std::signbit(a) == std::signbit(b))) || (std::isnan(a) && std::isnan(b));
}

// The exact model is SO3.rotate / inverseRotate. The executable contract fixes
// left-to-right evaluation, including initial +0; no floating reassociation.
Scalar ordered_dot(Scalar a, Scalar b, Scalar c, Vector const & v)
{
    Scalar result{0};
    result += a * v(0U);
    result += b * v(1U);
    result += c * v(2U);
    return result;
}

// Pure-callee summary: the actual matrix producer is proved separately for
// every row and both scalars. No coefficient/finite/unit-norm property is
// assumed here. An arbitrary matrix overapproximates every actual return.
struct RotationContract
{
    inline static Matrix result{};
    inline static Quaternion received_q{};
    inline static bool called = false;

    static void prepare(Matrix const & candidate)
    {
        result = candidate;
        called = false;
    }
};

} // namespace rotation_proof

#if FORMAL_ESKF_PROOF_ROTATION_CONTRACT
namespace formal_eskf::so3
{
template <>
rotation_proof::Matrix to_rotation_matrix<rotation_proof::Linalg>(rotation_proof::Quaternion const & q) noexcept
{
    using namespace rotation_proof;
    __ESBMC_assert(!RotationContract::called, "Q-ROTATE contract: action requests exactly one rotation matrix");
    RotationContract::called = true;
    RotationContract::received_q = q;
    return RotationContract::result;
}
} // namespace formal_eskf::so3
#endif

using namespace rotation_proof;

// Actual callee, not the summary. Arbitrary IEEE quaternion representations;
// no unit-norm premise, no assumed finite intermediate/result. This establishes
// source correspondence, not real orthogonality or a numerical error bound.
void verify_rotation_matrix(Quaternion q)
{
    __ESBMC_assert(!FORMAL_ESKF_PROOF_ROTATION_CONTRACT, "Runner error: verify the actual matrix producer");
    constexpr std::size_t i = FORMAL_ESKF_PROOF_AXIS;
    static_assert(i < 3U);
    Scalar const q0 = q.q0();
    Scalar const q1 = q.q1();
    Scalar const q2 = q.q2();
    Scalar const q3 = q.q3();
    // Independent row equations from SO3.rotationMatrix (Sola (115)).
    Vector expected;
#if FORMAL_ESKF_PROOF_AXIS == 0
    expected(0U) = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    expected(1U) = Scalar{2} * (q1 * q2 - q0 * q3);
    expected(2U) = Scalar{2} * (q1 * q3 + q0 * q2);
#elif FORMAL_ESKF_PROOF_AXIS == 1
    expected(0U) = Scalar{2} * (q1 * q2 + q0 * q3);
    expected(1U) = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
    expected(2U) = Scalar{2} * (q2 * q3 - q0 * q1);
#else
    expected(0U) = Scalar{2} * (q1 * q3 - q0 * q2);
    expected(1U) = Scalar{2} * (q2 * q3 + q0 * q1);
    expected(2U) = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
#endif
    auto const actual = formal_eskf::so3::to_rotation_matrix(q);
    bool const bounded = q0 >= Scalar{-1} && q0 <= Scalar{1} && q1 >= Scalar{-1} && q1 <= Scalar{1} &&
                         q2 >= Scalar{-1} && q2 <= Scalar{1} && q3 >= Scalar{-1} && q3 <= Scalar{1};
    for (std::size_t j = 0U; j < 3U; ++j)
    {
        __ESBMC_assert(same_scalar(actual(i, j), expected(j)),
                       "Q-ROT-MATRIX: row obeys ordered IEEE coefficient equations");
        // Discharge the finite-matrix premise used to recover every original
        // bounded-q basis claim. This is asserted, not assumed.
        __ESBMC_assert(!bounded || std::isfinite(actual(i, j)),
                       "Q-ROT-MATRIX: bounded quaternion has finite matrix entries");
    }
    __ESBMC_assert(same_scalar(q.q0(), q0) && same_scalar(q.q1(), q1) && same_scalar(q.q2(), q2) &&
                       same_scalar(q.q3(), q3),
                   "Q-ROT-MATRIX: actual producer preserves its quaternion input");
}

// Actual action/transpose/matvec code under an opaque matrix-producer contract.
// Arbitrary IEEE q, matrix and vector; no success/finite assumptions. Separate
// actual-producer proofs above close the only callee boundary.
void verify_rotation_action(Quaternion q, Vector v, Matrix matrix)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_ROTATION_CONTRACT, "Runner error: action requires its explicit matrix summary");
    auto const q_before = q.coefficients();
    auto const v_before = v;
    RotationContract::prepare(matrix);
    auto const forward = formal_eskf::so3::rotate(q, v);
    __ESBMC_assert(RotationContract::called, "Q-ROTATE: forward action obtains the matrix");
    for (std::size_t j = 0U; j < 4U; ++j)
    {
        __ESBMC_assert(same_scalar(RotationContract::received_q.coefficients()(j), q_before(j)) &&
                           same_scalar(q.coefficients()(j), q_before(j)),
                       "Q-ROTATE: forward action passes and preserves the original quaternion");
    }
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        __ESBMC_assert(same_scalar(v(i), v_before(i)), "Q-ROTATE: forward action preserves its vector input");
    }
    RotationContract::prepare(matrix);
    auto const inverse = formal_eskf::so3::inverse_rotate(q, v);
    __ESBMC_assert(RotationContract::called, "Q-ROTATE: inverse action obtains the matrix");
    for (std::size_t j = 0U; j < 4U; ++j)
    {
        __ESBMC_assert(same_scalar(RotationContract::received_q.coefficients()(j), q_before(j)) &&
                           same_scalar(q.coefficients()(j), q_before(j)),
                       "Q-ROTATE: inverse action passes the original quaternion; both actions preserve it");
    }
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        Scalar const expected_forward = ordered_dot(matrix(i, 0U), matrix(i, 1U), matrix(i, 2U), v_before);
        Scalar const expected_inverse = ordered_dot(matrix(0U, i), matrix(1U, i), matrix(2U, i), v_before);
        __ESBMC_assert(same_scalar(forward(i), expected_forward),
                       "Q-ROTATE: every forward component is the ordered IEEE row dot product");
        __ESBMC_assert(same_scalar(inverse(i), expected_inverse),
                       "Q-ROTATE: every inverse component is the ordered IEEE column dot product");
        __ESBMC_assert(same_scalar(v(i), v_before(i)), "Q-ROTATE: both actions preserve their vector input");
    }
}

// Basis specialization is a separate backend lemma, not an inference of IEEE
// linearity from a few examples. Symbolic index covers all three basis inputs;
// all matrix coefficients are arbitrary finite IEEE values. Together with the
// producer's bounded-q finiteness and the action proof this retains all 18
// original forward/inverse basis properties, and extends them to binary64.
void verify_matvec_basis(Matrix matrix, unsigned index)
{
    // State the premise independently of the production all_finite wrapper;
    // an erroneous wrapper must not make this backend lemma vacuous.
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            __ESBMC_assume(std::isfinite(matrix(row, column)));
        }
    }
    __ESBMC_assume(index < 3U);
    Vector basis;
    basis(index) = Scalar{1};
    auto const forward = matrix * basis;
    auto const inverse = formal_eskf::linalg::transpose(matrix) * basis;
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        __ESBMC_assert(forward(i) == matrix(i, index),
                       "Q-ROTATE basis: finite-matrix forward action selects the column");
        __ESBMC_assert(inverse(i) == matrix(index, i), "Q-ROTATE basis: finite-matrix inverse action selects the row");
    }
}
