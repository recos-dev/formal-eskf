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

static_assert(ESKF_MAG_TILT == 0, "E-OBS covers unrestricted magnetic correction; E-MAG-TILT remains pending");

#ifndef FORMAL_ESKF_PROOF_OBSERVATION
#define FORMAL_ESKF_PROOF_OBSERVATION 0
#endif
#ifndef FORMAL_ESKF_PROOF_OBSERVATION_CONTRACT
#define FORMAL_ESKF_PROOF_OBSERVATION_CONTRACT 0
#endif
#ifndef FORMAL_ESKF_PROOF_OBSERVATION_ROTATION
#define FORMAL_ESKF_PROOF_OBSERVATION_ROTATION 0
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
constexpr bool model_boundaries = FORMAL_ESKF_PROOF_OBSERVATION_ROTATION == (model >= 4U ? 1 : 0);

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
};

inline void expected_model(state_type const & state, vector_type const & reference, Model & result)
{
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        result.prediction[row] = value_type{0};
        for (std::size_t column = 0U; column < state_size; ++column)
        {
            result.H[row][column] = value_type{0};
        }
    }
#if FORMAL_ESKF_PROOF_OBSERVATION < 4 && FORMAL_ESKF_PROOF_STATE_SIZE == 15
    for (std::size_t row = 0U; row < measurement_size; ++row)
    {
        std::size_t const axis = model == 1U ? 2U : row;
        result.prediction[row] = model < 2U ? state.p_n(axis) : state.v_n(axis);
        result.H[row][(model < 2U ? 0U : 3U) + axis] = value_type{1};
    }
#else
    rows_type<3U> R;
    coordinates_type<3U> reference_b;
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
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        result.prediction[row] = model >= 6U ? -reference_b[row] : reference_b[row];
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
    (void)state;
    (void)reference;
}

inline value_type corrected_measurement(state_type const & state, residual_type const & measurement, std::size_t row)
{
#if FORMAL_ESKF_PROOF_OBSERVATION == 7 && FORMAL_ESKF_PROOF_STATE_SIZE == 15
    return measurement(row) - state.b_a(row);
#else
    (void)state;
    return measurement(row);
#endif
}

inline Status input_status(state_type const & state, residual_type const & measurement, noise_type const & V,
                           vector_type const & reference)
{
#if FORMAL_ESKF_PROOF_OBSERVATION < 4 && FORMAL_ESKF_PROOF_STATE_SIZE == 15
    bool const finite = finite_vector(measurement) &&
                        (model < 2U ? finite_vector(state.p_n) : finite_vector(state.v_n)) &&
                        (model != 1U || std::isfinite(V(0U, 0U)));
#else
    bool finite = finite_vector(state.q_nb.coefficients()) && finite_vector(measurement) && finite_vector(reference);
#if FORMAL_ESKF_PROOF_OBSERVATION == 7 && FORMAL_ESKF_PROOF_STATE_SIZE == 15
    finite = finite_vector(state.v_n) && finite_vector(state.b_a) && finite_vector(state.b_g) && finite;
#endif
#endif
    (void)V;
    (void)reference;
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
