/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include "linalg_support.hpp"
#include <formal_eskf/eskf/position.hpp>
#include <formal_eskf/eskf/velocity.hpp>
#include <formal_eskf/eskf/magnetometer.hpp>
#include <formal_eskf/eskf/accelerometer.hpp>

#ifndef FORMAL_ESKF_PROOF_OBSERVATION
#define FORMAL_ESKF_PROOF_OBSERVATION 0
#endif
#ifndef FORMAL_ESKF_PROOF_OBSERVATION_CONTRACT
#define FORMAL_ESKF_PROOF_OBSERVATION_CONTRACT 0
#endif
#ifndef FORMAL_ESKF_PROOF_OBSERVATION_ROTATION
#define FORMAL_ESKF_PROOF_OBSERVATION_ROTATION 0
#endif
#ifndef FORMAL_ESKF_PROOF_OBSERVATION_TERMS
#define FORMAL_ESKF_PROOF_OBSERVATION_TERMS 0
#endif

namespace observation_proof
{

using value_type = correction_proof::Scalar;
using backend_type = solve_proof::Backend;
using configuration_type = std::conditional_t<FORMAL_ESKF_PROOF_STATE_SIZE == 15, formal_eskf::configuration::Ins,
                                              formal_eskf::configuration::Ahrs>;
using state_type = configuration_type::NominalState<backend_type>;
using covariance_type = correction_proof::Covariance;
using residual_type = correction_proof::Residual;
using jacobian_type = correction_proof::Jacobian;
using noise_type = correction_proof::Noise;
using vector_type = backend_type::vector_type<3U>;
using quaternion_type = formal_eskf::so3::UnitQuaternion<backend_type>;
using rotation_type = backend_type::matrix_type<3U, 3U>;
using ins_jacobian_type = backend_type::matrix_type<3U, 15U>;
template <std::size_t Size> using coordinates_type = formal_eskf::verification::ScalarArray<value_type, Size>;
template <std::size_t Columns> using rows_type = formal_eskf::verification::ScalarArray<coordinates_type<Columns>, 3U>;
using correction_proof::same;
using correction_proof::same_matrix;
using formal_eskf::Status;

// Four INS selectors, then AHRS/INS magnetic and AHRS/INS acceleration models.
constexpr unsigned model = FORMAL_ESKF_PROOF_OBSERVATION;
constexpr std::size_t state_size = FORMAL_ESKF_PROOF_STATE_SIZE;
constexpr std::size_t measurement_size = FORMAL_ESKF_PROOF_MEASUREMENT_SIZE;
constexpr bool configured = model < 8U && state_size == ((model == 4U || model == 6U) ? 3U : 15U) &&
                            measurement_size == (model == 1U ? 1U : (model == 0U || model == 2U ? 2U : 3U)) &&
                            linalg_proof::actual;
constexpr std::size_t attitude_offset = state_size == 15U ? 6U : 0U;
constexpr bool model_boundaries = FORMAL_ESKF_PROOF_OBSERVATION_ROTATION == (model >= 4U ? 1 : 0) &&
                                  FORMAL_ESKF_PROOF_OBSERVATION_TERMS == (model == 7U ? 1 : 0);

// Pure rotation-matrix boundary. The same-type actual producer below closes
// this summary; no finite, unit, orthogonal or numerical-accuracy premise.
struct RotationCall
{
    inline static rotation_type result;
    inline static quaternion_type argument;
    inline static unsigned count{};
};

inline bool same_state(state_type const & left, state_type const & right)
{
    bool equal = same_matrix(left.q_nb.coefficients(), right.q_nb.coefficients());
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
    equal = same_matrix(left.p_n, right.p_n) && same_matrix(left.v_n, right.v_n) && same_matrix(left.b_a, right.b_a) &&
            same_matrix(left.b_g, right.b_g) && equal;
#endif
    return equal;
}

template <typename Vector> bool finite_vector(Vector const & vector)
{
    bool finite = true;
    for (std::size_t index = 0U; index < Vector::row_count; ++index)
    {
        finite = std::isfinite(vector(index)) && finite;
    }
    return finite;
}

template <typename Vector> bool nonzero(Vector const & vector)
{
    bool nonzero = false;
    for (std::size_t index = 0U; index < Vector::row_count; ++index)
    {
        nonzero = vector(index) != value_type{0} || nonzero;
    }
    return nonzero;
}

// Raw scalar equations, not calls to the production rotation/hat/model helpers.
// Scalar-field coordinate storage avoids ESBMC's bytewise floating-array
// copies. The equations below do not call backend or production model helpers.
inline void rotation(quaternion_type const & q, rows_type<3U> & R)
{
    value_type const q0 = q.q0(), q1 = q.q1(), q2 = q.q2(), q3 = q.q3();
    R[0][0] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    R[0][1] = value_type{2} * (q1 * q2 - q0 * q3);
    R[0][2] = value_type{2} * (q1 * q3 + q0 * q2);
    R[1][0] = value_type{2} * (q1 * q2 + q0 * q3);
    R[1][1] = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
    R[1][2] = value_type{2} * (q2 * q3 - q0 * q1);
    R[2][0] = value_type{2} * (q1 * q3 - q0 * q2);
    R[2][1] = value_type{2} * (q2 * q3 + q0 * q1);
    R[2][2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}

inline void skew(coordinates_type<3U> const & vector, rows_type<3U> & matrix)
{
    matrix[0][0] = value_type{0};
    matrix[0][1] = -vector[2];
    matrix[0][2] = vector[1];
    matrix[1][0] = vector[2];
    matrix[1][1] = value_type{0};
    matrix[1][2] = -vector[0];
    matrix[2][0] = -vector[1];
    matrix[2][1] = vector[0];
    matrix[2][2] = value_type{0};
}

inline void inverse_action(rows_type<3U> const & R, vector_type const & vector, coordinates_type<3U> & output)
{
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        value_type sum{0};
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            sum += R[column][row] * vector(column);
        }
        output[row] = sum;
    }
}

struct Model
{
    coordinates_type<3U> prediction;
    rows_type<state_size> H;
    rows_type<3U> R;
    coordinates_type<3U> v_b, omega_b, g_b;
    bool intermediate_finite = true;
};

struct TermsCall
{
    inline static ins_jacobian_type result;
    inline static Model const * expected = nullptr;
    inline static unsigned count{};
};

inline void ins_jacobian(rows_type<3U> const & R, coordinates_type<3U> const & v_b,
                         coordinates_type<3U> const & omega_b, coordinates_type<3U> const & g_b, rows_type<15U> & H)
{
    rows_type<3U> W, U, G;
    skew(omega_b, W);
    skew(v_b, U);
    skew(g_b, G);
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            value_type velocity{0}, attitude{0};
            for (std::size_t inner = 0U; inner < 3U; ++inner)
            {
                velocity += W[row][inner] * R[column][inner];
                attitude += W[row][inner] * U[inner][column];
            }
            H[row][column] = value_type{0};
            H[row][3U + column] = velocity;
            H[row][6U + column] = attitude - G[row][column];
            H[row][9U + column] = row == column ? value_type{1} : value_type{0};
            H[row][12U + column] = U[row][column];
        }
    }
}

inline void expected_model(state_type const & state, vector_type const & reference, vector_type const & angular_rate,
                           Model & result)
{
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        result.prediction[row] = value_type{0};
        for (std::size_t column = 0U; column < state_size; ++column)
        {
            result.H[row][column] = value_type{0};
        }
    }
    result.intermediate_finite = true;
#if FORMAL_ESKF_PROOF_OBSERVATION < 4 && FORMAL_ESKF_PROOF_STATE_SIZE == 15
    for (std::size_t row = 0U; row < measurement_size; ++row)
    {
        std::size_t const axis = model == 1U ? 2U : row;
        result.prediction[row] = model < 2U ? state.p_n(axis) : state.v_n(axis);
        result.H[row][(model < 2U ? 0U : 3U) + axis] = value_type{1};
    }
#else
    auto & R = result.R;
    auto & reference_b = result.g_b;
#if FORMAL_ESKF_PROOF_OBSERVATION_ROTATION
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            R[row][column] = RotationCall::result(row, column);
        }
    }
#else
    rotation(state.q_nb, R);
#endif
    inverse_action(R, reference, reference_b);
#if FORMAL_ESKF_PROOF_OBSERVATION == 7 && FORMAL_ESKF_PROOF_STATE_SIZE == 15
    auto & v_b = result.v_b;
    auto & omega_b = result.omega_b;
    inverse_action(R, state.v_n, v_b);
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        omega_b[row] = angular_rate(row) - state.b_g(row);
        result.intermediate_finite = std::isfinite(v_b[row]) && std::isfinite(reference_b[row]) &&
                                     std::isfinite(omega_b[row]) && result.intermediate_finite;
    }
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        std::size_t const next = (row + 1U) % 3U, last = (row + 2U) % 3U;
        result.prediction[row] =
            (omega_b[next] * v_b[last] - omega_b[last] * v_b[next]) - reference_b[row] + state.b_a(row);
    }
#if FORMAL_ESKF_PROOF_OBSERVATION_TERMS
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < state_size; ++column)
        {
            result.H[row][column] = TermsCall::result(row, column);
        }
    }
#else
    ins_jacobian(R, v_b, omega_b, reference_b, result.H);
#endif
#else
    (void)angular_rate;
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        result.prediction[row] = model == 6U ? -reference_b[row] : reference_b[row];
    }
    rows_type<3U> H;
    skew(result.prediction, H);
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            result.H[row][attitude_offset + column] = H[row][column];
        }
    }
#endif

#endif
    (void)state;
    (void)reference;
    (void)angular_rate;
}

inline Status input_status(state_type const & state, residual_type const & measurement, noise_type const & V,
                           vector_type const & reference, vector_type const & angular_rate)
{
#if FORMAL_ESKF_PROOF_OBSERVATION < 4 && FORMAL_ESKF_PROOF_STATE_SIZE == 15
    bool const finite = finite_vector(measurement) &&
                        (model < 2U ? finite_vector(state.p_n) : finite_vector(state.v_n)) &&
                        (model != 1U || std::isfinite(V(0U, 0U)));
#else
    bool finite = finite_vector(state.q_nb.coefficients()) && finite_vector(measurement) && finite_vector(reference);
#if FORMAL_ESKF_PROOF_OBSERVATION == 7 && FORMAL_ESKF_PROOF_STATE_SIZE == 15
    finite = finite_vector(state.v_n) && finite_vector(state.b_a) && finite_vector(state.b_g) &&
             finite_vector(angular_rate) && finite;
#endif
#endif
    (void)V;
    (void)reference;
    (void)angular_rate;
    if (!finite)
    {
        return Status::non_finite_input;
    }
    if constexpr (model >= 4U)
    {
        if (!nonzero(reference) || (model < 6U && !nonzero(measurement)))
        {
            return Status::domain_error;
        }
    }
    return Status::success;
}

} /* end namespace observation_proof */

#if FORMAL_ESKF_PROOF_OBSERVATION_ROTATION
namespace formal_eskf::so3
{
template <>
inline observation_proof::rotation_type
to_rotation_matrix<observation_proof::backend_type>(observation_proof::quaternion_type const & q) noexcept
{
    using namespace observation_proof;
    __ESBMC_assert(RotationCall::count++ == 0U && same_matrix(q.coefficients(), RotationCall::argument.coefficients()),
                   "E-OBS: exactly one rotation-matrix call with the original quaternion");
    return RotationCall::result;
}
} /* end namespace formal_eskf::so3 */
#endif

#if FORMAL_ESKF_PROOF_OBSERVATION_TERMS
namespace formal_eskf::detail
{
// Pure arbitrary result; verify_observation_terms checks the actual producer.
template <>
inline observation_proof::ins_jacobian_type accelerometer_jacobian_from_terms<observation_proof::backend_type>(
    observation_proof::rotation_type const & Rt, observation_proof::vector_type const & v_b,
    observation_proof::vector_type const & omega_b, observation_proof::vector_type const & g_b) noexcept
{
    using namespace observation_proof;
    __ESBMC_assert(TermsCall::count++ == 0U, "E-OBS: exactly one INS Jacobian assembly");
    auto const & expected = *TermsCall::expected;
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        __ESBMC_assert(same(v_b(row), expected.v_b[row]) && same(omega_b(row), expected.omega_b[row]) &&
                           same(g_b(row), expected.g_b[row]),
                       "E-OBS: INS Jacobian receives rotated velocity/gravity and bias-corrected angular rate");
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            __ESBMC_assert(same(Rt(row, column), expected.R[column][row]),
                           "E-OBS: INS Jacobian receives the transposed rotation matrix");
        }
    }
    return TermsCall::result;
}
} /* end namespace formal_eskf::detail */
#endif
