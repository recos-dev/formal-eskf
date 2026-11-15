/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "covariance_oracle.hpp"
#include <formal_eskf/runtime/navigation.hpp>

#ifndef FORMAL_ESKF_PROOF_NAVIGATION_CALLER
#define FORMAL_ESKF_PROOF_NAVIGATION_CALLER 0
#endif

#ifndef FORMAL_ESKF_PROOF_NAVIGATION_RESET
#define FORMAL_ESKF_PROOF_NAVIGATION_RESET 0
#endif
using contract_proof::Linalg;
using contract_proof::same;
using contract_proof::Scalar;
using covariance_oracle::same_matrix;
using formal_eskf::Status;
using State = formal_eskf::configuration::Ins::NominalState<Linalg>;
using Covariance = formal_eskf::linalg::Matrix<Linalg, 15U, 15U>;
constexpr unsigned model = FORMAL_ESKF_PROOF_NAVIGATION_RESET;
constexpr std::size_t size = model == 0U ? 2U : model == 3U ? 3U : 1U;
using Observation = formal_eskf::linalg::Vector<Linalg, size>;
static_assert(model < 4U && FORMAL_ESKF_PROOF_ALIAS < 4U);

bool same_state(State const & a, State const & b)
{
    return same_matrix(a.p_n, b.p_n) && same_matrix(a.v_n, b.v_n) && same_matrix(a.b_a, b.b_a) &&
           same_matrix(a.b_g, b.b_g) && same_matrix(a.q_nb.coefficients(), b.q_nb.coefficients());
}
bool finite_state(State const & s)
{
    return covariance_oracle::finite_matrix(s.p_n) && covariance_oracle::finite_matrix(s.v_n) &&
           covariance_oracle::finite_matrix(s.b_a) && covariance_oracle::finite_matrix(s.b_g) &&
           covariance_oracle::finite_matrix(s.q_nb.coefficients());
}
Status validation(State const & s, Covariance const & P, Observation const & observation, Observation const & variance)
{
    if (!finite_state(s) || !covariance_oracle::finite_matrix(P) || !covariance_oracle::finite_matrix(observation) ||
        !covariance_oracle::finite_matrix(variance))
    {
        return Status::non_finite_input;
    }
    Scalar prior[225];
    covariance_oracle::flatten(P, prior);
    Scalar maximum{0};
    for (std::size_t i = 0U; i < 15U; ++i)
    {
        for (std::size_t j = 0U; j < 15U; ++j)
        {
            Scalar const residual = std::fabs(prior[15U * i + j] - prior[15U * j + i]);
            if (!std::isfinite(residual))
            {
                return Status::domain_error;
            }
            if (residual > maximum)
            {
                maximum = residual;
            }
        }
    }
    if (maximum > Scalar{0})
    {
        return Status::domain_error;
    }
    for (std::size_t i = 0U; i < 15U; ++i)
    {
        if (prior[16U * i] < Scalar{0})
        {
            return Status::domain_error;
        }
    }
    for (std::size_t i = 0U; i < size; ++i)
    {
        if (variance(i) < Scalar{0})
        {
            return Status::domain_error;
        }
    }
    return Status::success;
}
bool selected(std::size_t axis)
{
    if constexpr (model == 0U)
    {
        return axis < 2U;
    }
    if constexpr (model == 1U)
    {
        return axis == 2U;
    }
    if constexpr (model == 2U)
    {
        return axis == 2U || axis == 5U;
    }
    return axis < 6U;
}
Scalar floored(Scalar v)
{
    constexpr Scalar deviation = static_cast<Scalar>(0.01);
    constexpr Scalar floor = deviation * deviation;
    return v < floor ? floor : v;
}
Scalar expected_diagonal(Scalar prior, Observation const & variance, Scalar horizontal, std::size_t axis)
{
    if constexpr (model == 0U)
    {
        return floored(horizontal);
    }
    if constexpr (model == 1U)
    {
        return floored(variance(0U));
    }
    if constexpr (model == 2U)
    {
        return axis == 2U ? prior : floored(variance(0U));
    }
    if (axis < 3U)
    {
        return prior;
    }
    return floored(variance(axis - 3U));
}

// Scalar link between the zero-residual symmetry test and exact coefficient
// equality. Includes underflow, signed zero and subtraction overflow.
void verify_navigation_symmetry_pair(Scalar a, Scalar b)
{
    bool const finite = std::isfinite(a) && std::isfinite(b);
    Scalar const residual = std::fabs(a - b);
    __ESBMC_assert(!finite || ((std::isfinite(residual) && residual <= Scalar{0}) == (a == b)),
                   "R-GNSS-RESET: finite transposed coefficients pass zero residual exactly when equal");
}

// This exact-type producer is checked separately before its failure-atomic,
// input-pure status boundary is composed with each numerical reset caller.
#if !FORMAL_ESKF_PROOF_NAVIGATION_CALLER
void verify_navigation_validation(State state, Covariance P, Observation observation, Observation variance)
{
    auto const old_state = state;
    auto const old_P = P;
    auto const old_observation = observation;
    auto const old_variance = variance;
    auto const expected = validation(state, P, observation, variance);
    auto const actual = formal_eskf::runtime::detail::validate_navigation_reset_inputs(state, P, observation, variance);
    __ESBMC_assert(actual == expected && same_state(state, old_state) && same_matrix(P, old_P) &&
                       same_matrix(observation, old_observation) && same_matrix(variance, old_variance),
                   "R-GNSS-RESET: actual validation producer has exact status and preserves every borrowed input");
}

#endif

#if FORMAL_ESKF_PROOF_NAVIGATION_CALLER
struct ValidationCall
{
    inline static State const * state{};
    inline static Covariance const * P{};
    inline static Observation const * observation{};
    inline static Observation const * variance{};
    inline static Status status{};
    inline static unsigned count{};
};
namespace formal_eskf::runtime::detail
{
template <>
inline Status validate_navigation_reset_inputs<Linalg, size>(State const & state, Covariance const & P,
                                                             Observation const & observation,
                                                             Observation const & variance)
{
    __ESBMC_assert(ValidationCall::count++ == 0U && same_state(state, *ValidationCall::state) &&
                       same_matrix(P, *ValidationCall::P) && same_matrix(observation, *ValidationCall::observation) &&
                       same_matrix(variance, *ValidationCall::variance),
                   "R-GNSS-RESET: caller validates the complete original request exactly once");
    return ValidationCall::status;
}
} // namespace formal_eskf::runtime::detail
#endif

void verify_navigation_reset(State state, Covariance P, Observation observation, Observation variance,
                             State state_output, Covariance covariance_output, Observation delta_output,
                             Status validation_status, std::size_t row, std::size_t column)
{
    __ESBMC_assume(row < 15U && column < 15U);
    constexpr bool state_alias = (FORMAL_ESKF_PROOF_ALIAS & 1U) != 0;
    constexpr bool covariance_alias = (FORMAL_ESKF_PROOF_ALIAS & 2U) != 0;
    auto const old_state = state;
    auto const old_P = P;
    auto const old_observation = observation;
    auto const old_variance = variance;
    State * state_pointer = &state_output;
    Covariance * covariance_pointer = &covariance_output;
    if constexpr (state_alias)
    {
        state_pointer = &state;
    }
    if constexpr (covariance_alias)
    {
        covariance_pointer = &P;
    }
    State & target_state = *state_pointer;
    Covariance & target_P = *covariance_pointer;
    auto const old_target_state = target_state;
    auto const old_target_P = target_P;
    auto const old_delta = delta_output;
    auto expected = state;
    Observation delta;
    for (std::size_t i = 0U; i < size; ++i)
    {
        if constexpr (model == 0U)
        {
            delta(i) = observation(i) - state.p_n(i);
            expected.p_n(i) = observation(i);
        }
        else if constexpr (model == 1U)
        {
            delta(i) = observation(i) - state.p_n(2U);
            expected.p_n(2U) = observation(i);
        }
        else if constexpr (model == 2U)
        {
            delta(i) = observation(i) - state.v_n(2U);
            expected.v_n(2U) = observation(i);
        }
        else
        {
            delta(i) = observation(i) - state.v_n(i);
            expected.v_n(i) = observation(i);
        }
    }
    Scalar horizontal{0};
    if constexpr (model == 0U)
    {
        horizontal = variance(0U) + variance(1U);
    }
#if FORMAL_ESKF_PROOF_NAVIGATION_CALLER
    ValidationCall::state = &old_state;
    ValidationCall::P = &old_P;
    ValidationCall::observation = &old_observation;
    ValidationCall::variance = &old_variance;
    ValidationCall::status = validation_status;
    ValidationCall::count = 0U;
    Status status = validation_status;
#else
    Status status = validation(state, P, observation, variance);
#endif
    if (status == Status::success && (!covariance_oracle::finite_matrix(delta) || !std::isfinite(horizontal)))
    {
        status = Status::non_finite_result;
    }
    Status actual;
#if FORMAL_ESKF_PROOF_NAVIGATION_RESET == 0
    actual = formal_eskf::runtime::reset_gnss_horizontal_position(state, P, observation, variance, target_state,
                                                                  target_P, delta_output);
#elif FORMAL_ESKF_PROOF_NAVIGATION_RESET == 1
    actual = formal_eskf::runtime::reset_vertical_position(state, P, observation(0U), variance(0U), target_state,
                                                           target_P, delta_output(0U));
#elif FORMAL_ESKF_PROOF_NAVIGATION_RESET == 2
    actual = formal_eskf::runtime::reset_vertical_velocity(state, P, observation(0U), variance(0U), target_state,
                                                           target_P, delta_output(0U));
#else
    actual =
        formal_eskf::runtime::reset_velocity(state, P, observation, variance, target_state, target_P, delta_output);
#endif
#if FORMAL_ESKF_PROOF_NAVIGATION_CALLER
    __ESBMC_assert(ValidationCall::count == 1U, "R-GNSS-RESET: validation cannot be skipped");
#endif
    __ESBMC_assert(actual == status, "R-GNSS-RESET: exact validation and finite-input overflow status");
    if (actual == Status::success)
    {
        __ESBMC_assert(same_state(target_state, expected) && same_matrix(delta_output, delta),
                       "R-GNSS-RESET: exact selected nominal coordinates and new-minus-old delta");
        Scalar before[225], after[225];
        covariance_oracle::flatten(old_P, before);
        covariance_oracle::flatten(target_P, after);
        // Arbitrary row/column universally quantifies the full matrix. This
        // is not a representative diagonal or a finite sample of coefficients.
        Scalar value = before[15U * row + column];
        if (row != column && (selected(row) || selected(column)))
        {
            value = Scalar{0};
        }
        if (row == column && selected(row))
        {
            value = expected_diagonal(value, old_variance, horizontal, row);
        }
        __ESBMC_assert(same(after[15U * row + column], value),
                       "R-GNSS-RESET: every covariance coefficient, complete decoupling and encoded variance floor");
    }
    else
    {
        __ESBMC_assert(same_state(target_state, old_target_state) && same_matrix(target_P, old_target_P) &&
                           same_matrix(delta_output, old_delta),
                       "R-GNSS-RESET: every failure preserves all three outputs for each legal alias mode");
    }
    __ESBMC_assert(same_matrix(observation, old_observation) && same_matrix(variance, old_variance),
                   "R-GNSS-RESET: borrowed observations and variances are unchanged");
    if constexpr (!state_alias)
    {
        __ESBMC_assert(same_state(state, old_state), "R-GNSS-RESET: disjoint prior state is unchanged");
    }
    if constexpr (!covariance_alias)
    {
        __ESBMC_assert(same_matrix(P, old_P), "R-GNSS-RESET: disjoint prior covariance is unchanged");
    }
}
int main() { return 0; }
