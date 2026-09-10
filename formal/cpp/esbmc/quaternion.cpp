/**
 * ESBMC checks for the implemented quaternion and SO(3) semantics.
 *
 * Each entry point below maps directly to one requirement in
 * docs/so3-unit-quaternion-semantics.md.  All symbolic coefficients are
 * IEEE-754 binary32 values in [-1, 1]; unit norm is not assumed unless the
 * entry point calls the public checked constructor.
 */

#include <limits>

#include "support.hpp"

namespace proof = formal_eskf::verification;

using proof::Linalg;
using proof::Matrix3;
using proof::Quaternion;
using proof::Scalar;
using proof::Vector3;

/* Q-IDENTITY, Q-NEGATE, and Q-INVERSE. */
void verify_basic_representation(Quaternion quaternion)
{
    proof::assume_quaternion(quaternion);

    Quaternion const identity = Quaternion::identity();
    __ESBMC_assert(identity.q0() == Scalar{1} && identity.q1() == Scalar{0} && identity.q2() == Scalar{0} &&
                       identity.q3() == Scalar{0},
                   "Q-IDENTITY: identity is [1, 0, 0, 0]");

    Quaternion const negative = -quaternion;
    __ESBMC_assert(negative.q0() == -quaternion.q0() && negative.q1() == -quaternion.q1() &&
                       negative.q2() == -quaternion.q2() && negative.q3() == -quaternion.q3(),
                   "Q-NEGATE: negation changes every coefficient sign");

    Quaternion const inverse = quaternion.inverse();
    __ESBMC_assert(inverse.q0() == quaternion.q0() && inverse.q1() == -quaternion.q1() &&
                       inverse.q2() == -quaternion.q2() && inverse.q3() == -quaternion.q3(),
                   "Q-INVERSE: unit inverse is the Hamilton conjugate");

    __ESBMC_assert(proof::same_coefficients(identity * quaternion, quaternion) &&
                       proof::same_coefficients(quaternion * identity, quaternion),
                   "Q-IDENTITY-LAWS: identity acts on both sides");
    __ESBMC_assert(proof::same_coefficients(-negative, quaternion) &&
                       proof::same_coefficients(inverse.inverse(), quaternion),
                   "Q-INVOLUTIONS: negation and inverse are involutive");
}

/* Q-MUL. */
void verify_hamilton_product(Quaternion left, Quaternion right)
{
    proof::assume_quaternion(left);
    proof::assume_quaternion(right);
    Quaternion const result = left * right;

    Scalar const q0 = left.q0() * right.q0() - left.q1() * right.q1() - left.q2() * right.q2() - left.q3() * right.q3();
    Scalar const q1 = left.q0() * right.q1() + left.q1() * right.q0() + left.q2() * right.q3() - left.q3() * right.q2();
    Scalar const q2 = left.q0() * right.q2() - left.q1() * right.q3() + left.q2() * right.q0() + left.q3() * right.q1();
    Scalar const q3 = left.q0() * right.q3() + left.q1() * right.q2() - left.q2() * right.q1() + left.q3() * right.q0();

    __ESBMC_assert(result.q0() == q0 && result.q1() == q1 && result.q2() == q2 && result.q3() == q3,
                   "Q-MUL: multiplication implements the scalar-first Hamilton product");
}

/* Q-NORMALIZE: checked-construction status and failure atomicity. */
void verify_construction_success(Scalar q0, Scalar q1, Scalar q2, Scalar q3)
{
    proof::assume_scalar(q0);
    proof::assume_scalar(q1);
    proof::assume_scalar(q2);
    proof::assume_scalar(q3);
    __ESBMC_assume(q0 <= Scalar{-0.25} || q0 >= Scalar{0.25} || q1 <= Scalar{-0.25} || q1 >= Scalar{0.25} ||
                   q2 <= Scalar{-0.25} || q2 >= Scalar{0.25} || q3 <= Scalar{-0.25} || q3 >= Scalar{0.25});

    Quaternion output;
    formal_eskf::Status const status = Quaternion::try_from_coefficients(q0, q1, q2, q3, Scalar{0.125}, output);
    __ESBMC_assert(status == formal_eskf::Status::success, "Q-NORMALIZE-SUCCESS: profiled nonzero input is accepted");
}

void verify_construction_failures()
{
    Quaternion zero_output = -Quaternion::identity();
    formal_eskf::Status const zero_status =
        Quaternion::try_from_coefficients(Scalar{0}, Scalar{0}, Scalar{0}, Scalar{0}, Scalar{0.5}, zero_output);
    __ESBMC_assert(zero_status == formal_eskf::Status::invalid_quaternion_norm &&
                       proof::is_negative_identity(zero_output),
                   "Q-NORMALIZE-ZERO: zero input is rejected without changing output");

    Quaternion minimum_output = -Quaternion::identity();
    formal_eskf::Status const minimum_status =
        Quaternion::try_from_coefficients(Scalar{1}, Scalar{0}, Scalar{0}, Scalar{0}, Scalar{0}, minimum_output);
    __ESBMC_assert(minimum_status == formal_eskf::Status::domain_error && proof::is_negative_identity(minimum_output),
                   "Q-NORMALIZE-MINIMUM: non-positive minimum norm is rejected without changing output");

    Scalar const nan = std::numeric_limits<Scalar>::quiet_NaN();
    Quaternion nan_output = -Quaternion::identity();
    formal_eskf::Status const nan_status =
        Quaternion::try_from_coefficients(nan, Scalar{0}, Scalar{0}, Scalar{0}, Scalar{0.5}, nan_output);
    __ESBMC_assert(nan_status == formal_eskf::Status::non_finite_input && proof::is_negative_identity(nan_output),
                   "Q-NORMALIZE-NAN: non-finite input is rejected without changing output");

    Scalar const maximum = std::numeric_limits<Scalar>::max();
    Quaternion overflow_output = -Quaternion::identity();
    formal_eskf::Status const overflow_status =
        Quaternion::try_from_coefficients(maximum, maximum, Scalar{0}, Scalar{0}, Scalar{0.5}, overflow_output);
    __ESBMC_assert(overflow_status == formal_eskf::Status::non_finite_result &&
                       proof::is_negative_identity(overflow_output),
                   "Q-NORMALIZE-OVERFLOW: norm overflow is reported without changing output");
}

/* Q-ROT-MATRIX.  Rows are separate only to keep solver memory bounded. */
void verify_rotation_matrix_row_0(Quaternion quaternion)
{
    proof::assume_quaternion(quaternion);
    Matrix3 const rotation = formal_eskf::so3::to_rotation_matrix(quaternion);
    Scalar const q0 = quaternion.q0();
    Scalar const q1 = quaternion.q1();
    Scalar const q2 = quaternion.q2();
    Scalar const q3 = quaternion.q3();
    __ESBMC_assert(rotation(0U, 0U) == q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3 &&
                       rotation(0U, 1U) == Scalar{2} * (q1 * q2 - q0 * q3) &&
                       rotation(0U, 2U) == Scalar{2} * (q1 * q3 + q0 * q2),
                   "Q-ROT-MATRIX-ROW-0: R(q) row 0 matches the specified map");
}

void verify_rotation_matrix_row_1(Quaternion quaternion)
{
    proof::assume_quaternion(quaternion);
    Matrix3 const rotation = formal_eskf::so3::to_rotation_matrix(quaternion);
    Scalar const q0 = quaternion.q0();
    Scalar const q1 = quaternion.q1();
    Scalar const q2 = quaternion.q2();
    Scalar const q3 = quaternion.q3();
    __ESBMC_assert(rotation(1U, 0U) == Scalar{2} * (q1 * q2 + q0 * q3) &&
                       rotation(1U, 1U) == q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3 &&
                       rotation(1U, 2U) == Scalar{2} * (q2 * q3 - q0 * q1),
                   "Q-ROT-MATRIX-ROW-1: R(q) row 1 matches the specified map");
}

void verify_rotation_matrix_row_2(Quaternion quaternion)
{
    proof::assume_quaternion(quaternion);
    Matrix3 const rotation = formal_eskf::so3::to_rotation_matrix(quaternion);
    Scalar const q0 = quaternion.q0();
    Scalar const q1 = quaternion.q1();
    Scalar const q2 = quaternion.q2();
    Scalar const q3 = quaternion.q3();
    __ESBMC_assert(rotation(2U, 0U) == Scalar{2} * (q1 * q3 - q0 * q2) &&
                       rotation(2U, 1U) == Scalar{2} * (q2 * q3 + q0 * q1) &&
                       rotation(2U, 2U) == q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3,
                   "Q-ROT-MATRIX-ROW-2: R(q) row 2 matches the specified map");
}

[[nodiscard]] bool rotation_sign_equal(Quaternion const & quaternion, std::size_t row, std::size_t column)
{
    return formal_eskf::so3::to_rotation_matrix(quaternion)(row, column) ==
           formal_eskf::so3::to_rotation_matrix(-quaternion)(row, column);
}

#define FORMAL_ESKF_ROTATION_SIGN_CHECK(ROW, COLUMN)                                                                   \
    void verify_rotation_sign_##ROW##COLUMN(Quaternion quaternion)                                                     \
    {                                                                                                                  \
        proof::assume_quaternion(quaternion);                                                                          \
        __ESBMC_assert(rotation_sign_equal(quaternion, ROW##U, COLUMN##U), "Q-SIGN: R(q) equals R(-q)");               \
    }

FORMAL_ESKF_ROTATION_SIGN_CHECK(0, 0)
FORMAL_ESKF_ROTATION_SIGN_CHECK(0, 1)
FORMAL_ESKF_ROTATION_SIGN_CHECK(0, 2)
FORMAL_ESKF_ROTATION_SIGN_CHECK(1, 0)
FORMAL_ESKF_ROTATION_SIGN_CHECK(1, 1)
FORMAL_ESKF_ROTATION_SIGN_CHECK(1, 2)
FORMAL_ESKF_ROTATION_SIGN_CHECK(2, 0)
FORMAL_ESKF_ROTATION_SIGN_CHECK(2, 1)
FORMAL_ESKF_ROTATION_SIGN_CHECK(2, 2)

#undef FORMAL_ESKF_ROTATION_SIGN_CHECK

void verify_same_rotation_sign(Quaternion quaternion)
{
    proof::assume_quaternion(quaternion);
    __ESBMC_assert(formal_eskf::so3::same_rotation(quaternion, -quaternion, Scalar{0}),
                   "Q-SIGN: same_rotation accepts q and -q at zero tolerance");
}

/* SO3-HAT. */
void verify_hat(Scalar x, Scalar y, Scalar z)
{
    proof::assume_scalar(x);
    proof::assume_scalar(y);
    proof::assume_scalar(z);
    Vector3 vector;
    vector(0U) = x;
    vector(1U) = y;
    vector(2U) = z;

    Matrix3 const result = formal_eskf::so3::hat<Linalg>(vector);
    __ESBMC_assert(result(0U, 0U) == Scalar{0} && result(0U, 1U) == -z && result(0U, 2U) == y && result(1U, 0U) == z &&
                       result(1U, 1U) == Scalar{0} && result(1U, 2U) == -x && result(2U, 0U) == -y &&
                       result(2U, 1U) == x && result(2U, 2U) == Scalar{0},
                   "SO3-HAT: hat(v) matches the specified skew-symmetric matrix");
}

int main() { return 0; }
