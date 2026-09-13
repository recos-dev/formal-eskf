/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "contracts.hpp"

#ifndef FORMAL_ESKF_PROOF_AXIS
#define FORMAL_ESKF_PROOF_AXIS 0
#endif

using namespace contract_proof;

// Q-IDENTITY/Q-INVERSE: representations are arbitrary IEEE values. Identity
// multiplication laws have a finite-input premise (0*Inf is NaN); equality of
// those arithmetic results is value equality, not signed-zero preservation.
void verify_representation(Quaternion q)
{
    auto const before = q.coefficients();
    Quaternion const identity;
    __ESBMC_assert(same(identity.q0(), Scalar{1}) && same(identity.q1(), Scalar{0}) && same(identity.q2(), Scalar{0}) &&
                       same(identity.q3(), Scalar{0}) &&
                       same_vector(identity.coefficients(), Quaternion::identity().coefficients()),
                   "Q-IDENTITY: default and named identity are scalar-first [1,+0,+0,+0]");
    auto const negative = -q;
    auto const conjugate = q.inverse();
    __ESBMC_assert(same(negative.q0(), -q.q0()) && same(negative.q1(), -q.q1()) && same(negative.q2(), -q.q2()) &&
                       same(negative.q3(), -q.q3()),
                   "Q-SIGN: negation changes all four signs");
    __ESBMC_assert(same(conjugate.q0(), q.q0()) && same(conjugate.q1(), -q.q1()) && same(conjugate.q2(), -q.q2()) &&
                       same(conjugate.q3(), -q.q3()),
                   "Q-INVERSE: conjugation changes only vector signs");
    __ESBMC_assert(same_vector((-negative).coefficients(), before) &&
                       same_vector(conjugate.inverse().coefficients(), before),
                   "Q-INVERSE: sign and conjugation involutions preserve values and signed zeros");
    if (finite_vector(before))
    {
        auto const left = identity * q;
        auto const right = q * identity;
        for (std::size_t i = 0U; i < 4U; ++i)
        {
            __ESBMC_assert(left.coefficients()(i) == before(i) && right.coefficients()(i) == before(i),
                           "Q-IDENTITY: two-sided arithmetic identity on every finite coefficient");
        }
    }
    __ESBMC_assert(same_vector(q.coefficients(), before), "Q-IDENTITY/Q-INVERSE: input is unchanged");
}

void verify_product(Quaternion a, Quaternion b)
{
    auto const a_before = a.coefficients();
    auto const b_before = b.coefficients();
    // Sola (12), Quaternion.hamiltonMul: no float associativity is asserted.
    Vector4 expected;
    expected(0U) = a.q0() * b.q0() - a.q1() * b.q1() - a.q2() * b.q2() - a.q3() * b.q3();
    expected(1U) = a.q0() * b.q1() + a.q1() * b.q0() + a.q2() * b.q3() - a.q3() * b.q2();
    expected(2U) = a.q0() * b.q2() - a.q1() * b.q3() + a.q2() * b.q0() + a.q3() * b.q1();
    expected(3U) = a.q0() * b.q3() + a.q1() * b.q2() - a.q2() * b.q1() + a.q3() * b.q0();
    auto const result = a * b;
    __ESBMC_assert(same_vector(result.coefficients(), expected), "Q-MUL: every ordered Hamilton coefficient");
    __ESBMC_assert(same_vector(a.coefficients(), a_before) && same_vector(b.coefficients(), b_before),
                   "Q-MUL: both operands are unchanged");
}

void verify_hat_entries(Vector3 v)
{
    auto const before = v;
    auto const matrix = formal_eskf::so3::hat<Linalg>(v);
    __ESBMC_assert(same(matrix(0U, 0U), Scalar{0}) && same(matrix(1U, 1U), Scalar{0}) &&
                       same(matrix(2U, 2U), Scalar{0}),
                   "SO3-HAT: all diagonal entries are positive zero");
    __ESBMC_assert(same(matrix(0U, 1U), -v(2U)) && same(matrix(0U, 2U), v(1U)) && same(matrix(1U, 0U), v(2U)) &&
                       same(matrix(1U, 2U), -v(0U)) && same(matrix(2U, 0U), -v(1U)) && same(matrix(2U, 1U), v(0U)),
                   "SO3-HAT: all six off-diagonal entries follow the cross-product sign convention");
    __ESBMC_assert(same_vector(v, before), "SO3-HAT: input is unchanged");
}

// Finite coefficients, without the previous [-1,1] fixture bound. Overflowed
// matrix entries may be Inf/NaN; equality here includes NaN classification.
void verify_matrix_sign(Quaternion q)
{
    __ESBMC_assume(finite_vector(q.coefficients()));
    constexpr std::size_t row = FORMAL_ESKF_PROOF_AXIS;
    static_assert(row < 3U);
    auto const positive = formal_eskf::so3::to_rotation_matrix(q);
    auto const negative = formal_eskf::so3::to_rotation_matrix(-q);
    for (std::size_t column = 0U; column < 3U; ++column)
    {
        __ESBMC_assert(positive(row, column) == negative(row, column) ||
                           (std::isnan(positive(row, column)) && std::isnan(negative(row, column))),
                       "Q-SIGN: either quaternion sign yields the same matrix values/NaN classes");
    }
}

// Independent max-residual decision, not a second call to max_abs. Invalid
// coefficients cannot produce a true comparison with a finite tolerance.
void verify_rotation_comparison(Quaternion a, Quaternion b, Scalar tolerance)
{
    auto const a_before = a.coefficients();
    auto const b_before = b.coefficients();
    bool difference_ok = true;
    bool sum_ok = true;
    bool coefficients_equal = true;
    for (std::size_t i = 0U; i < 4U; ++i)
    {
        Scalar const x = a_before(i);
        Scalar const y = b_before(i);
        difference_ok = difference_ok && std::fabs(x - y) <= tolerance;
        sum_ok = sum_ok && std::fabs(x + y) <= tolerance;
        coefficients_equal = coefficients_equal && x == y;
    }
    bool const expected = std::isfinite(tolerance) && tolerance >= Scalar{0} && (difference_ok || sum_ok);
    __ESBMC_assert(formal_eskf::so3::same_rotation(a, b, tolerance) == expected,
                   "Q-SIGN: finite nonnegative tolerance bounds all coefficients for either sign");
    __ESBMC_assert(formal_eskf::so3::same_coefficients(a, b) == coefficients_equal,
                   "Q-SIGN: same_coefficients compares representation without sign equivalence");
    __ESBMC_assert(same_vector(a.coefficients(), a_before) && same_vector(b.coefficients(), b_before),
                   "Q-SIGN: comparisons preserve both inputs");
}

void verify_principal_representative(Quaternion q)
{
    auto const before = q.coefficients();
    auto const principal = formal_eskf::so3::detail::principal_quaternion_coefficients(q);
    Scalar const sign = q.q0() < Scalar{0} ? Scalar{-1} : Scalar{1};
    __ESBMC_assert(same(principal.q0, sign * q.q0()) && same(principal.q1, sign * q.q1()) &&
                       same(principal.q2, sign * q.q2()) && same(principal.q3, sign * q.q3()),
                   "SO3-LOG: representative negates exactly when scalar coefficient is negative");
    if (finite_vector(before) && q.q0() != Scalar{0})
    {
        auto const opposite = formal_eskf::so3::detail::principal_quaternion_coefficients(-q);
        __ESBMC_assert(principal.q0 == opposite.q0 && principal.q1 == opposite.q1 && principal.q2 == opposite.q2 &&
                           principal.q3 == opposite.q3 && principal.q0 > Scalar{0},
                       "SO3-LOG: both signs select one positive-scalar representative away from pi");
    }
    if (q.q0() == Scalar{0})
    {
        __ESBMC_assert(same(principal.q1, q.q1()) && same(principal.q2, q.q2()) && same(principal.q3, q.q3()),
                       "SO3-LOG: either signed-zero scalar retains its original pi-axis sign");
    }
    __ESBMC_assert(same_vector(q.coefficients(), before), "SO3-LOG: representative selection preserves input");
}

int main() { return 0; }
