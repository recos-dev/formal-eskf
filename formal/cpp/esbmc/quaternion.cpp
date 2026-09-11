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
using proof::Vector4;

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

    Quaternion output = -Quaternion::identity();
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

void verify_construction_basis()
{
    Quaternion output = -Quaternion::identity();
    formal_eskf::Status const status =
        Quaternion::try_from_coefficients(Scalar{0}, Scalar{0}, Scalar{1}, Scalar{0}, Scalar{0.125}, output);
    __ESBMC_assert(status == formal_eskf::Status::success && output.q0() == Scalar{0} && output.q1() == Scalar{0} &&
                       output.q2() == Scalar{1} && output.q3() == Scalar{0},
                   "Q-NORMALIZE-BASIS: an exact unit basis quaternion is unchanged");
}

#define FORMAL_ESKF_NORMALIZATION_COEFFICIENT_CHECK(INDEX)                                                             \
    void verify_normalization_candidate_coefficient_##INDEX(Scalar q0, Scalar q1, Scalar q2, Scalar q3, Scalar norm)  \
    {                                                                                                                  \
        proof::assume_scalar(q0);                                                                                      \
        proof::assume_scalar(q1);                                                                                      \
        proof::assume_scalar(q2);                                                                                      \
        proof::assume_scalar(q3);                                                                                      \
        __ESBMC_assume(norm >= Scalar{0.125} && norm <= Scalar{1});                                                    \
        Vector4 input;                                                                                                  \
        input.set(0U, q0);                                                                                              \
        input.set(1U, q1);                                                                                              \
        input.set(2U, q2);                                                                                              \
        input.set(3U, q3);                                                                                              \
        Vector4 const output = formal_eskf::linalg::detail::normalization_candidate(input, norm);                      \
        __ESBMC_assert(output(INDEX##U) == input(INDEX##U) / norm,                                                     \
                       "Q-NORMALIZE-COEFFICIENT: candidate coefficient equals input divided by checked norm");        \
    }

FORMAL_ESKF_NORMALIZATION_COEFFICIENT_CHECK(0)
FORMAL_ESKF_NORMALIZATION_COEFFICIENT_CHECK(1)
FORMAL_ESKF_NORMALIZATION_COEFFICIENT_CHECK(2)
FORMAL_ESKF_NORMALIZATION_COEFFICIENT_CHECK(3)

#undef FORMAL_ESKF_NORMALIZATION_COEFFICIENT_CHECK

void verify_construction_below_threshold()
{
    Quaternion output = -Quaternion::identity();
    formal_eskf::Status const status =
        Quaternion::try_from_coefficients(Scalar{0.25}, Scalar{0}, Scalar{0}, Scalar{0}, Scalar{0.5}, output);
    __ESBMC_assert(status == formal_eskf::Status::invalid_quaternion_norm && proof::is_negative_identity(output),
                   "Q-NORMALIZE-THRESHOLD: a finite nonzero norm below the threshold is rejected atomically");
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

/* Q-ROT-COMPOSE: production multiplication followed by checked normalization. */
void verify_composition(Quaternion left, Quaternion right)
{
    proof::assume_quaternion(left);
    proof::assume_quaternion(right);

    Quaternion const product = left * right;
    Scalar const product_norm_squared = product.q0() * product.q0() + product.q1() * product.q1() +
                                        product.q2() * product.q2() + product.q3() * product.q3();
    __ESBMC_assume(product_norm_squared >= Scalar{0.25});

    Quaternion output = -Quaternion::identity();
    formal_eskf::Status const status = formal_eskf::so3::try_compose_normalized(left, right, Scalar{0.125}, output);

    Quaternion expected = Quaternion::identity();
    formal_eskf::Status const expected_status = Quaternion::try_from_coefficients(
        product.q0(), product.q1(), product.q2(), product.q3(), Scalar{0.125}, expected);

    __ESBMC_assert(status == formal_eskf::Status::success && expected_status == formal_eskf::Status::success,
                   "Q-ROT-COMPOSE-STATUS: a profiled safe product is accepted");
    __ESBMC_assert(proof::same_coefficients(output, expected),
                   "Q-ROT-COMPOSE: composition is Hamilton multiplication followed by normalization");
}

/* Q-ROTATE: basis actions cover all coefficients without one large query. */
[[nodiscard]] Vector3 basis_vector(std::size_t index)
{
    Vector3 result;
    result(index) = Scalar{1};
    return result;
}

#define FORMAL_ESKF_ROTATE_BASIS_CHECK(ROW, COLUMN)                                                                    \
    void verify_rotate_basis_##ROW##COLUMN(Quaternion quaternion)                                                      \
    {                                                                                                                  \
        proof::assume_quaternion(quaternion);                                                                          \
        Matrix3 const rotation = formal_eskf::so3::to_rotation_matrix(quaternion);                                     \
        Vector3 const result = formal_eskf::so3::rotate(quaternion, basis_vector(COLUMN##U));                          \
        __ESBMC_assert(result(ROW##U) == rotation(ROW##U, COLUMN##U),                                                  \
                       "Q-ROTATE: rotating a basis vector returns the matching R(q) coefficient");                     \
    }                                                                                                                  \
    void verify_inverse_rotate_basis_##ROW##COLUMN(Quaternion quaternion)                                              \
    {                                                                                                                  \
        proof::assume_quaternion(quaternion);                                                                          \
        Matrix3 const rotation = formal_eskf::so3::to_rotation_matrix(quaternion);                                     \
        Vector3 const result = formal_eskf::so3::inverse_rotate(quaternion, basis_vector(ROW##U));                     \
        __ESBMC_assert(result(COLUMN##U) == rotation(ROW##U, COLUMN##U),                                               \
                       "Q-INVERSE-ROTATE: inverse rotating a basis vector returns the matching R(q) coefficient");     \
    }

FORMAL_ESKF_ROTATE_BASIS_CHECK(0, 0)
FORMAL_ESKF_ROTATE_BASIS_CHECK(0, 1)
FORMAL_ESKF_ROTATE_BASIS_CHECK(0, 2)
FORMAL_ESKF_ROTATE_BASIS_CHECK(1, 0)
FORMAL_ESKF_ROTATE_BASIS_CHECK(1, 1)
FORMAL_ESKF_ROTATE_BASIS_CHECK(1, 2)
FORMAL_ESKF_ROTATE_BASIS_CHECK(2, 0)
FORMAL_ESKF_ROTATE_BASIS_CHECK(2, 1)
FORMAL_ESKF_ROTATE_BASIS_CHECK(2, 2)

#undef FORMAL_ESKF_ROTATE_BASIS_CHECK

void verify_rotate_zero(Quaternion quaternion)
{
    proof::assume_quaternion(quaternion);
    __ESBMC_assert(
        proof::same_vector(formal_eskf::so3::rotate(quaternion, Vector3::zero()), Vector3::zero()) &&
            proof::same_vector(formal_eskf::so3::inverse_rotate(quaternion, Vector3::zero()), Vector3::zero()),
        "Q-ROTATE-ZERO: forward and inverse actions preserve the zero vector");
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

/* SO3-EXP and SO3-SMALL-ANGLE. */
void verify_exp_zero()
{
    Vector3 const zero = Vector3::zero();
    Quaternion output = -Quaternion::identity();
    formal_eskf::Status const status = formal_eskf::so3::try_exp(zero, Scalar{0.125}, output);

    __ESBMC_assert(status == formal_eskf::Status::success && proof::same_coefficients(output, Quaternion::identity()),
                   "SO3-EXP-ZERO: Exp(0) returns identity");
}

void verify_exp_taylor_branch()
{
    Scalar const angle = Scalar{0.0001};

    Vector3 rotation_vector;
    rotation_vector(0U) = angle;
    rotation_vector(1U) = Scalar{0};
    rotation_vector(2U) = Scalar{0};

    Scalar const theta_squared = angle * angle;
    __ESBMC_assert(theta_squared <= std::numeric_limits<Scalar>::epsilon(),
                   "SO3-SMALL-ANGLE-DOMAIN: profiled input selects the Taylor branch");
    Scalar const theta_fourth = theta_squared * theta_squared;
    Scalar const q0 = Scalar{1} - theta_squared / Scalar{8} + theta_fourth / Scalar{384};
    Scalar const vector_scale = Scalar{0.5} - theta_squared / Scalar{48} + theta_fourth / Scalar{3840};

    Quaternion output = -Quaternion::identity();
    formal_eskf::Status const status = formal_eskf::so3::try_exp(rotation_vector, Scalar{0.125}, output);
    Quaternion expected;
    formal_eskf::Status const expected_status =
        Quaternion::try_from_coefficients(q0, vector_scale * angle, Scalar{0}, Scalar{0}, Scalar{0.125}, expected);

    __ESBMC_assert(status == formal_eskf::Status::success && expected_status == formal_eskf::Status::success,
                   "SO3-EXP-TAYLOR-STATUS: profiled small input is accepted");
    __ESBMC_assert(proof::same_coefficients(output, expected),
                   "SO3-EXP-TAYLOR: the production Taylor coefficients are normalized and returned");
}

void verify_exp_closed_form()
{
    Scalar const angle = Scalar{0.5};

    Vector3 rotation_vector;
    rotation_vector(0U) = angle;
    rotation_vector(1U) = Scalar{0};
    rotation_vector(2U) = Scalar{0};

    Scalar const theta = std::sqrt(angle * angle);
    Scalar const q0 = std::cos(Scalar{0.5} * theta);
    Scalar const vector_scale = std::sin(Scalar{0.5} * theta) / theta;

    Quaternion output = -Quaternion::identity();
    formal_eskf::Status const status = formal_eskf::so3::try_exp(rotation_vector, Scalar{0.125}, output);
    Quaternion expected;
    formal_eskf::Status const expected_status =
        Quaternion::try_from_coefficients(q0, vector_scale * angle, Scalar{0}, Scalar{0}, Scalar{0.125}, expected);

    __ESBMC_assert(status == formal_eskf::Status::success && expected_status == formal_eskf::Status::success,
                   "SO3-EXP-CLOSED-STATUS: profiled regular input is accepted");
    __ESBMC_assert(proof::same_coefficients(output, expected),
                   "SO3-EXP-CLOSED: the production closed-form coefficients are normalized and returned");
}

void verify_exp_failures()
{
    Vector3 zero = Vector3::zero();
    Quaternion output = -Quaternion::identity();
    formal_eskf::Status const minimum_status = formal_eskf::so3::try_exp(zero, Scalar{0}, output);
    __ESBMC_assert(minimum_status == formal_eskf::Status::domain_error && proof::is_negative_identity(output),
                   "SO3-EXP-MINIMUM: a non-positive minimum norm is rejected atomically");

    zero(0U) = std::numeric_limits<Scalar>::quiet_NaN();
    formal_eskf::Status const nan_status = formal_eskf::so3::try_exp(zero, Scalar{0.125}, output);
    __ESBMC_assert(nan_status == formal_eskf::Status::non_finite_input && proof::is_negative_identity(output),
                   "SO3-EXP-NAN: non-finite input is rejected atomically");
}

/* SO3-LOG. */
void verify_log_zero()
{
    Vector3 output;
    output(0U) = Scalar{1};
    output(1U) = Scalar{1};
    output(2U) = Scalar{1};

    formal_eskf::Status const identity_status = formal_eskf::so3::try_log(Quaternion::identity(), output);
    __ESBMC_assert(identity_status == formal_eskf::Status::success && proof::same_vector(output, Vector3::zero()),
                   "SO3-LOG-IDENTITY: Log(identity) returns zero");

    output(0U) = Scalar{1};
    output(1U) = Scalar{1};
    output(2U) = Scalar{1};
    formal_eskf::Status const negative_status = formal_eskf::so3::try_log(-Quaternion::identity(), output);
    __ESBMC_assert(negative_status == formal_eskf::Status::success && proof::same_vector(output, Vector3::zero()),
                   "SO3-LOG-NEGATIVE-IDENTITY: Log(-identity) returns zero");
}

void verify_log_taylor_branch(Scalar vector_coefficient)
{
    __ESBMC_assume(vector_coefficient >= Scalar{0.0001} && vector_coefficient <= Scalar{0.0002});

    Quaternion quaternion;
    formal_eskf::Status const construction_status = Quaternion::try_from_coefficients(
        Scalar{1}, vector_coefficient, Scalar{0}, Scalar{0}, Scalar{0.125}, quaternion);
    __ESBMC_assert(construction_status == formal_eskf::Status::success,
                   "SO3-LOG-TAYLOR-CONSTRUCTION: profiled quaternion construction succeeds");

    Scalar const qv_squared_norm = quaternion.q1() * quaternion.q1();
    __ESBMC_assert(qv_squared_norm > Scalar{0} && qv_squared_norm <= std::numeric_limits<Scalar>::epsilon(),
                   "SO3-LOG-TAYLOR-DOMAIN: profiled quaternion selects the Taylor branch");
    Scalar const q0_squared = quaternion.q0() * quaternion.q0();
    Scalar const scale = Scalar{2} / quaternion.q0() * (Scalar{1} - qv_squared_norm / (Scalar{3} * q0_squared));

    Vector3 output;
    formal_eskf::Status const status = formal_eskf::so3::try_log(quaternion, output);
    __ESBMC_assert(status == formal_eskf::Status::success, "SO3-LOG-TAYLOR-STATUS: Taylor Log succeeds");
    __ESBMC_assert(output(0U) == quaternion.q1() * scale && output(1U) == Scalar{0} && output(2U) == Scalar{0},
                   "SO3-LOG-TAYLOR: production Log evaluates its documented small-vector scale");
}

void verify_log_closed_form()
{
    Scalar const vector_coefficient = Scalar{0.5};

    Quaternion quaternion;
    formal_eskf::Status const construction_status = Quaternion::try_from_coefficients(
        Scalar{0.75}, vector_coefficient, Scalar{0}, Scalar{0}, Scalar{0.125}, quaternion);
    __ESBMC_assert(construction_status == formal_eskf::Status::success,
                   "SO3-LOG-CLOSED-CONSTRUCTION: profiled quaternion construction succeeds");

    Vector3 output;
    formal_eskf::Status const status = formal_eskf::so3::try_log(quaternion, output);
    __ESBMC_assert(status == formal_eskf::Status::success, "SO3-LOG-CLOSED-STATUS: closed-form Log succeeds");
    __ESBMC_assert(output(1U) == Scalar{0} && output(2U) == Scalar{0},
                   "SO3-LOG-CLOSED: zero vector coefficients remain zero through the regular path");
}

void verify_log_closed_form_scale(Scalar qv_norm, Scalar half_angle)
{
    __ESBMC_assume(qv_norm >= Scalar{0.125} && qv_norm <= Scalar{1});
    __ESBMC_assume(half_angle >= Scalar{0} && half_angle <= Scalar{1.57079632679489661923});
    Scalar const scale = formal_eskf::so3::detail::log_closed_form_scale(qv_norm, half_angle);
    __ESBMC_assert(scale == Scalar{2} * half_angle / qv_norm,
                   "SO3-LOG-CLOSED-SCALE: the observed atan2 half-angle is divided by vector norm");
}

#define FORMAL_ESKF_LOG_CANDIDATE_COEFFICIENT_CHECK(INDEX)                                                            \
    void verify_log_candidate_coefficient_##INDEX(Scalar q1, Scalar q2, Scalar q3, Scalar scale)                     \
    {                                                                                                                  \
        proof::assume_scalar(q1);                                                                                      \
        proof::assume_scalar(q2);                                                                                      \
        proof::assume_scalar(q3);                                                                                      \
        proof::assume_scalar(scale);                                                                                   \
        Vector3 qv;                                                                                                    \
        qv.set(0U, q1);                                                                                                \
        qv.set(1U, q2);                                                                                                \
        qv.set(2U, q3);                                                                                                \
        Vector3 const output = formal_eskf::so3::detail::log_candidate<Linalg>(qv, scale);                            \
        __ESBMC_assert(output(INDEX##U) == qv(INDEX##U) * scale,                                                       \
                       "SO3-LOG-CANDIDATE: each vector coefficient is multiplied by the checked scale");            \
    }

FORMAL_ESKF_LOG_CANDIDATE_COEFFICIENT_CHECK(0)
FORMAL_ESKF_LOG_CANDIDATE_COEFFICIENT_CHECK(1)
FORMAL_ESKF_LOG_CANDIDATE_COEFFICIENT_CHECK(2)

#undef FORMAL_ESKF_LOG_CANDIDATE_COEFFICIENT_CHECK

void verify_log_principal_sign(Quaternion quaternion)
{
    proof::assume_quaternion(quaternion);
    __ESBMC_assume(quaternion.q0() != Scalar{0});

    auto const principal = formal_eskf::so3::detail::principal_quaternion_coefficients(quaternion);
    auto const negative_principal = formal_eskf::so3::detail::principal_quaternion_coefficients(-quaternion);
    __ESBMC_assert(principal.q0 == negative_principal.q0 && principal.q1 == negative_principal.q1 &&
                       principal.q2 == negative_principal.q2 && principal.q3 == negative_principal.q3,
                   "SO3-LOG-SIGN: q and -q select the same principal representative away from pi");
}

void verify_log_pi_boundary()
{
    Quaternion positive_axis;
    formal_eskf::Status const positive_construction =
        Quaternion::try_from_coefficients(Scalar{0}, Scalar{1}, Scalar{0}, Scalar{0}, Scalar{0.125}, positive_axis);
    Quaternion negative_axis = -positive_axis;

    Vector3 positive_output;
    Vector3 negative_output;
    formal_eskf::Status const positive_status = formal_eskf::so3::try_log(positive_axis, positive_output);
    formal_eskf::Status const negative_status = formal_eskf::so3::try_log(negative_axis, negative_output);

    __ESBMC_assert(positive_construction == formal_eskf::Status::success &&
                       positive_status == formal_eskf::Status::success &&
                       negative_status == formal_eskf::Status::success,
                   "SO3-LOG-PI-STATUS: both pi-axis representatives are accepted");
    __ESBMC_assert(positive_output(0U) == -negative_output(0U) && positive_output(1U) == Scalar{0} &&
                       positive_output(2U) == Scalar{0} && negative_output(1U) == Scalar{0} &&
                       negative_output(2U) == Scalar{0},
                   "SO3-LOG-PI: opposite axis signs remain valid at the pi boundary");
    constexpr Scalar half_pi = Scalar{1.57079632679489661923};
    __ESBMC_assert(positive_output(0U) == Scalar{2} * half_pi &&
                       negative_output(0U) == -(Scalar{2} * half_pi),
                   "SO3-LOG-PI-MAGNITUDE: both boundary results have the contracted pi magnitude");
}

int main() { return 0; }
