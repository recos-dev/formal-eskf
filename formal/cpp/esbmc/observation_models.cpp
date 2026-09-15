/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "observations_support.hpp"

void verify_observation_jacobian(observation_proof::state_type state, observation_proof::vector_type reference,
                                 observation_proof::vector_type angular_rate,
                                 observation_proof::rotation_type rotation_result,
                                 observation_proof::ins_jacobian_type terms_result)
{
    using namespace observation_proof;
    constexpr bool valid = configured && model_boundaries && !FORMAL_ESKF_PROOF_OBSERVATION_CONTRACT;
    __ESBMC_assert(valid,
                   "Runner error: observation Jacobian requires actual producer bodies and exact model/dimensions");
    if constexpr (!valid)
    {
        return;
    }
    auto const state_before = state;
    auto const reference_before = reference;
    auto const angular_before = angular_rate;
    RotationCall::result = rotation_result;
    RotationCall::argument = state.q_nb;
    RotationCall::count = 0U;
    TermsCall::result = terms_result;
    TermsCall::count = 0U;
    Model expected;
    expected_model(state, reference, angular_rate, expected);
    TermsCall::expected = &expected;
#if FORMAL_ESKF_PROOF_OBSERVATION == 0
    auto const actual = formal_eskf::horizontal_position_jacobian<backend_type>();
#elif FORMAL_ESKF_PROOF_OBSERVATION == 1
    auto const actual = formal_eskf::vertical_position_jacobian<backend_type>();
#elif FORMAL_ESKF_PROOF_OBSERVATION == 2
    auto const actual = formal_eskf::horizontal_velocity_jacobian<backend_type>();
#elif FORMAL_ESKF_PROOF_OBSERVATION == 3
    auto const actual = formal_eskf::velocity_jacobian<backend_type>();
#elif FORMAL_ESKF_PROOF_OBSERVATION == 4 || FORMAL_ESKF_PROOF_OBSERVATION == 5
    auto const actual = formal_eskf::magnetometer_jacobian(state, reference);
#elif FORMAL_ESKF_PROOF_OBSERVATION == 6
    auto const actual = formal_eskf::accelerometer_jacobian(state, reference);
#else
    auto const actual = formal_eskf::accelerometer_jacobian(state, angular_rate, reference);
#endif
    for (std::size_t row = 0U; row < measurement_size; ++row)
    {
        for (std::size_t column = 0U; column < state_size; ++column)
        {
            __ESBMC_assert(
                same(actual(row, column), expected.H[row][column]),
                "E-OBS: unchecked Jacobian matches ordered IEEE model equations, including non-finite inputs");
        }
    }
    __ESBMC_assert(RotationCall::count == (model >= 4U ? 1U : 0U), "E-OBS: Jacobian consumes its rotation producer");
    __ESBMC_assert(TermsCall::count == (model == 7U ? 1U : 0U), "E-OBS: INS Jacobian consumes its terms producer");
    __ESBMC_assert(same_state(state, state_before) && same_matrix(reference, reference_before) &&
                       same_matrix(angular_rate, angular_before),
                   "E-OBS: unchecked Jacobian preserves every input");
}

void verify_observation_rotation(observation_proof::quaternion_type q)
{
    using namespace observation_proof;
    constexpr bool valid = configured && model == 4U && !FORMAL_ESKF_PROOF_OBSERVATION_ROTATION &&
                           !FORMAL_ESKF_PROOF_OBSERVATION_TERMS && !FORMAL_ESKF_PROOF_OBSERVATION_CONTRACT;
    __ESBMC_assert(valid, "Runner error: observation rotation requires the actual same-type producer");
    if constexpr (!valid)
    {
        return;
    }
    auto const before = q;
    rows_type<3U> expected;
    rotation(q, expected);
    auto const actual = formal_eskf::so3::to_rotation_matrix(q);
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            __ESBMC_assert(same(actual(row, column), expected[row][column]),
                           "E-OBS: actual rotation producer matches all ordered scalar equations");
        }
    }
    __ESBMC_assert(same_matrix(q.coefficients(), before.coefficients()), "E-OBS: rotation producer preserves input");
}

void verify_observation_terms(observation_proof::rotation_type Rt, observation_proof::vector_type v_b,
                              observation_proof::vector_type omega_b, observation_proof::vector_type g_b)
{
    using namespace observation_proof;
    constexpr bool valid = configured && model == 7U && !FORMAL_ESKF_PROOF_OBSERVATION_ROTATION &&
                           !FORMAL_ESKF_PROOF_OBSERVATION_TERMS && !FORMAL_ESKF_PROOF_OBSERVATION_CONTRACT;
    __ESBMC_assert(valid, "Runner error: observation terms require the actual same-type producer");
    if constexpr (!valid)
    {
        return;
    }
    auto const old_Rt = Rt;
    auto const old_v = v_b;
    auto const old_omega = omega_b;
    auto const old_g = g_b;
    rows_type<3U> R;
    coordinates_type<3U> v, omega, g;
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        v[row] = v_b(row);
        omega[row] = omega_b(row);
        g[row] = g_b(row);
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            R[column][row] = Rt(row, column);
        }
    }
    rows_type<15U> expected;
    ins_jacobian(R, v, omega, g, expected);
    auto const actual = formal_eskf::detail::accelerometer_jacobian_from_terms(Rt, v_b, omega_b, g_b);
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 15U; ++column)
        {
            __ESBMC_assert(same(actual(row, column), expected[row][column]),
                           "E-OBS: all five INS Jacobian blocks match ordered scalar equations");
        }
    }
    __ESBMC_assert(same_matrix(Rt, old_Rt) && same_matrix(v_b, old_v) && same_matrix(omega_b, old_omega) &&
                       same_matrix(g_b, old_g),
                   "E-OBS: actual INS Jacobian assembly preserves every input");
}

int main() { return 0; }
