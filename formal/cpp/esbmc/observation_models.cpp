/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "observations_support.hpp"

void verify_observation_jacobian(observation_proof::state_type state, observation_proof::vector_type reference,
                                 observation_proof::rotation_type rotation_result)
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
    RotationCall::result = rotation_result;
    RotationCall::argument = state.q_nb;
    RotationCall::count = 0U;
    Model expected;
    expected_model(state, reference, expected);
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
#else
    auto const actual = formal_eskf::accelerometer_jacobian(state, reference);
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
    __ESBMC_assert(same_state(state, state_before) && same_matrix(reference, reference_before),
                   "E-OBS: unchecked Jacobian preserves every input");
}

void verify_observation_rotation(observation_proof::quaternion_type q)
{
    using namespace observation_proof;
    constexpr bool valid =
        configured && model == 4U && !FORMAL_ESKF_PROOF_OBSERVATION_ROTATION && !FORMAL_ESKF_PROOF_OBSERVATION_CONTRACT;
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

int main() { return 0; }
