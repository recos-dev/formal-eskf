/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * Retained binary32 execution witnesses and historical bounded success checks.
 * General-domain source proofs live in algebra.cpp, normalization.cpp,
 * rotation.cpp and maps.cpp. These witnesses do not substitute for them.
 */

#include <limits>

#include "support.hpp"

namespace proof = formal_eskf::verification;
using proof::Linalg;
using proof::Quaternion;
using proof::Scalar;
using proof::Vector3;

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

void verify_construction_below_threshold()
{
    Quaternion output = -Quaternion::identity();
    formal_eskf::Status const status =
        Quaternion::try_from_coefficients(Scalar{0.25}, Scalar{0}, Scalar{0}, Scalar{0}, Scalar{0.5}, output);
    __ESBMC_assert(status == formal_eskf::Status::invalid_quaternion_norm && proof::is_negative_identity(output),
                   "Q-NORMALIZE-THRESHOLD: a finite nonzero norm below the threshold is rejected atomically");
}

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

void verify_rotate_zero(Quaternion quaternion)
{
    proof::assume_quaternion(quaternion);
    auto const forward = formal_eskf::so3::rotate(quaternion, Vector3::zero());
    auto const inverse = formal_eskf::so3::inverse_rotate(quaternion, Vector3::zero());
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        __ESBMC_assert(forward(i) == Scalar{0}, "Q-ROTATE-ZERO: forward action preserves each zero component");
        __ESBMC_assert(inverse(i) == Scalar{0}, "Q-ROTATE-ZERO: inverse action preserves each zero component");
    }
}

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
    __ESBMC_assert(positive_output(0U) == Scalar{2} * half_pi && negative_output(0U) == -(Scalar{2} * half_pi),
                   "SO3-LOG-PI-MAGNITUDE: both boundary results have the contracted pi magnitude");
}

int main() { return 0; }
