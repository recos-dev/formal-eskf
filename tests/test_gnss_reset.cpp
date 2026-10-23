/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <array>
#include <bit>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string_view>

// Exercise the standalone public header on each configured backend.
#include <formal_eskf/runtime/gnss_reset.hpp>
#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{

using formal_eskf::Status;
using TestContext = formal_eskf::test::Context;

enum class ResetGroup
{
    horizontal_position,
    vertical_position,
    vertical_velocity,
    velocity,
};

template <typename Scalar> bool same_scalar(Scalar a, Scalar b)
{
    using bytes_type = std::array<std::byte, sizeof(Scalar)>;
    return std::bit_cast<bytes_type>(a) == std::bit_cast<bytes_type>(b);
}

template <typename Matrix> bool same_bits(Matrix const & a, Matrix const & b)
{
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            if (!same_scalar(a(row, column), b(row, column)))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename State> bool same_state(State const & a, State const & b)
{
    return same_bits(a.p_n, b.p_n) && same_bits(a.v_n, b.v_n) && same_bits(a.b_a, b.b_a) && same_bits(a.b_g, b.b_g) &&
           same_bits(a.q_nb.coefficients(), b.q_nb.coefficients());
}

template <typename Scalar, ResetGroup Group> struct Fixture
{
    using linalg_type = formal_eskf::test::Backend<Scalar>;
    using state_type = formal_eskf::configuration::Ins::NominalState<linalg_type>;
    using covariance_type = formal_eskf::linalg::Matrix<linalg_type, 15U, 15U>;
    static constexpr bool vertical = Group == ResetGroup::vertical_position || Group == ResetGroup::vertical_velocity;
    static constexpr bool velocity = Group == ResetGroup::velocity || Group == ResetGroup::vertical_velocity;
    static constexpr std::size_t size = Group == ResetGroup::horizontal_position ? 2U : vertical ? 1U : 3U;
    static constexpr std::size_t axis_offset = vertical ? 2U : 0U;
    using vector_type = formal_eskf::linalg::Matrix<linalg_type, size, 1U>;

    state_type state{};
    covariance_type P{};
    vector_type observation{};
    vector_type variances{};

    Fixture()
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            Scalar const n = static_cast<Scalar>(axis + 1U);
            state.p_n.set(axis, n * Scalar{10});
            state.v_n.set(axis, -n * Scalar{2});
            state.b_a.set(axis, n / Scalar{8});
            state.b_g.set(axis, -n / Scalar{16});
        }
        // Dense, strictly diagonally dominant symmetric P: every reset row has
        // cross-covariances, so identity-only fixtures cannot hide omissions.
        for (std::size_t row = 0U; row < 15U; ++row)
        {
            for (std::size_t column = 0U; column < 15U; ++column)
            {
                Scalar const value = row == column               ? Scalar{16} + static_cast<Scalar>(row)
                                     : (row + column) % 2U == 0U ? Scalar{0.125}
                                                                 : Scalar{-0.125};
                P.set(row, column, value);
            }
        }
        // A retained signed zero must not be changed by blanket symmetrization.
        P.set(10U, 11U, -Scalar{0});
        P.set(11U, 10U, -Scalar{0});
        for (std::size_t axis = 0U; axis < size; ++axis)
        {
            observation.set(axis, Scalar{4} - static_cast<Scalar>(axis) * Scalar{7});
            variances.set(axis, Scalar{1} + static_cast<Scalar>(axis) * Scalar{3});
        }
    }

    [[nodiscard]] Status reset(state_type & state_output, covariance_type & covariance_output,
                               vector_type & delta_output) const
    {
        if constexpr (Group == ResetGroup::horizontal_position)
        {
            return formal_eskf::runtime::try_reset_gnss_horizontal_position(
                state, P, observation, variances, state_output, covariance_output, delta_output);
        }
        else if constexpr (vertical)
        {
            Scalar delta = delta_output(0U);
            Status status{};
            if constexpr (velocity)
            {
                status = formal_eskf::runtime::try_reset_gnss_vertical_velocity(
                    state, P, observation(0U), variances(0U), state_output, covariance_output, delta);
            }
            else
            {
                status = formal_eskf::runtime::try_reset_gnss_vertical_position(
                    state, P, observation(0U), variances(0U), state_output, covariance_output, delta);
            }
            // Forward even on failure: rollback checks must see the actual scalar output.
            delta_output.set(0U, delta);
            return status;
        }
        else
        {
            return formal_eskf::runtime::try_reset_gnss_velocity(state, P, observation, variances, state_output,
                                                                 covariance_output, delta_output);
        }
    }
};

template <typename Scalar, ResetGroup Group>
void check_success(TestContext & test, Fixture<Scalar, Group> const & fixture, std::string_view profile)
{
    using F = Fixture<Scalar, Group>;
    auto expected_state = fixture.state;
    auto expected_P = fixture.P;
    typename F::vector_type expected_delta;
    constexpr Scalar minimum_std = static_cast<Scalar>(0.01);
    constexpr Scalar floor = minimum_std * minimum_std;
    // Independent entry-wise oracle, not calls to the production reset helper.
    for (std::size_t axis = 0U; axis < F::size; ++axis)
    {
        auto & target = F::velocity ? expected_state.v_n : expected_state.p_n;
        auto const state_axis = F::axis_offset + axis;
        expected_delta.set(axis, fixture.observation(axis) - target(state_axis));
        target.set(state_axis, fixture.observation(axis));
    }
    for (std::size_t row = 0U; row < 15U; ++row)
    {
        for (std::size_t column = 0U; column < 15U; ++column)
        {
            bool const decoupled = Group == ResetGroup::horizontal_position ? row < 2U || column < 2U
                                   : Group == ResetGroup::vertical_position ? row == 2U || column == 2U
                                   : Group == ResetGroup::vertical_velocity
                                       ? row == 2U || column == 2U || row == 5U || column == 5U
                                       : row < 6U || column < 6U;
            if (row != column && decoupled)
            {
                expected_P.set(row, column, Scalar{0});
            }
            bool const replaced = Group == ResetGroup::horizontal_position ? row < 2U
                                  : Group == ResetGroup::vertical_position ? row == 2U
                                  : Group == ResetGroup::vertical_velocity ? row == 5U
                                                                           : row >= 3U && row < 6U;
            if (row == column && replaced)
            {
                Scalar variance{};
                if constexpr (Group == ResetGroup::horizontal_position)
                {
                    variance = fixture.variances(0U) + fixture.variances(1U);
                }
                else if constexpr (F::vertical)
                {
                    variance = fixture.variances(0U);
                }
                else
                {
                    variance = fixture.variances(row - 3U);
                }
                expected_P.set(row, column, variance < floor ? floor : variance);
            }
        }
    }
    for (unsigned aliases = 0U; aliases < 4U; ++aliases)
    {
        auto input = fixture;
        typename F::state_type separate_state;
        auto separate_P = F::covariance_type::identity() * Scalar{99};
        auto & state = (aliases & 1U) != 0U ? input.state : separate_state;
        auto & P = (aliases & 2U) != 0U ? input.P : separate_P;
        typename F::vector_type delta;
        test.expect(input.reset(state, P, delta) == Status::success, profile, "reset succeeds for valid inputs");
        test.expect(same_state(state, expected_state) && same_bits(P, expected_P) && same_bits(delta, expected_delta),
                    profile, "all state/P/delta entries match independent oracle, including retained bits");
        test.expect(((aliases & 1U) != 0U || same_state(input.state, fixture.state)) &&
                        ((aliases & 2U) != 0U || same_bits(input.P, fixture.P)) &&
                        same_bits(input.observation, fixture.observation) &&
                        same_bits(input.variances, fixture.variances),
                    profile, "unaliased inputs are unchanged");
    }
}

template <typename Scalar, ResetGroup Group>
void check_failure(TestContext & test, Fixture<Scalar, Group> const & fixture, Status expected,
                   std::string_view profile)
{
    using F = Fixture<Scalar, Group>;
    for (unsigned aliases = 0U; aliases < 4U; ++aliases)
    {
        auto input = fixture;
        typename F::state_type separate_state;
        separate_state.p_n.set(0U, Scalar{99});
        auto separate_P = F::covariance_type::identity() * Scalar{99};
        auto & state = (aliases & 1U) != 0U ? input.state : separate_state;
        auto & P = (aliases & 2U) != 0U ? input.P : separate_P;
        auto const before_state = state;
        auto const before_P = P;
        typename F::vector_type delta;
        for (std::size_t axis = 0U; axis < F::size; ++axis)
        {
            delta.set(axis, Scalar{-64});
        }
        auto const before_delta = delta;
        test.expect(input.reset(state, P, delta) == expected, profile, "failure reports the specified status");
        test.expect(same_state(state, before_state) && same_bits(P, before_P) && same_bits(delta, before_delta) &&
                        same_state(input.state, fixture.state) && same_bits(input.P, fixture.P) &&
                        same_bits(input.observation, fixture.observation) &&
                        same_bits(input.variances, fixture.variances),
                    profile, "failure preserves separate sentinels, aliased inputs and delta");
    }
}

template <typename Scalar, ResetGroup Group> void run_conformance(TestContext & test, std::string_view profile)
{
    using F = Fixture<Scalar, Group>;
    F fixture;
    test.expect(F::state_type::quaternion_type::try_from_coefficients(Scalar{0.5}, Scalar{-0.5}, Scalar{0.5},
                                                                      Scalar{-0.5}, Scalar{0.125},
                                                                      fixture.state.q_nb) == Status::success,
                profile, "non-identity attitude fixture is valid");
    check_success(test, fixture, profile);
    if constexpr (F::vertical)
    {
        Scalar const prior = F::velocity ? fixture.state.v_n(2U) : fixture.state.p_n(2U);
        for (Scalar down : {Scalar{-40}, -Scalar{0}, Scalar{0}, prior, Scalar{40}})
        {
            auto input = fixture;
            input.observation.set(0U, down);
            check_success(test, input, profile);
        }
    }
    constexpr Scalar floor = static_cast<Scalar>(0.01) * static_cast<Scalar>(0.01);
    for (Scalar variance : {Scalar{0}, -Scalar{0}, std::numeric_limits<Scalar>::denorm_min(), floor / Scalar{4},
                            floor / Scalar{2}, floor, Scalar{4}})
    {
        auto input = fixture;
        for (std::size_t axis = 0U; axis < F::size; ++axis)
        {
            input.variances.set(axis, variance);
        }
        check_success(test, input, profile);
    }
    auto mixed = fixture;
    mixed.variances.set(0U, Scalar{0});
    check_success(test, mixed, profile);
    auto singular = fixture;
    singular.P = typename F::covariance_type{};
    check_success(test, singular, profile);

    Scalar const nan = std::numeric_limits<Scalar>::quiet_NaN();
    Scalar const inf = std::numeric_limits<Scalar>::infinity();
    Scalar const max = std::numeric_limits<Scalar>::max();
    for (Scalar bad_value : {nan, inf, -inf})
    {
        for (std::size_t axis = 0U; axis < F::size; ++axis)
        {
            auto input = fixture;
            input.observation.set(axis, bad_value);
            check_failure(test, input, Status::non_finite_input, profile);
            input = fixture;
            input.variances.set(axis, bad_value);
            check_failure(test, input, Status::non_finite_input, profile);
        }
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            for (auto member : {&F::state_type::p_n, &F::state_type::v_n, &F::state_type::b_a, &F::state_type::b_g})
            {
                auto input = fixture;
                (input.state.*member).set(axis, bad_value);
                check_failure(test, input, Status::non_finite_input, profile);
            }
        }
        for (std::size_t row = 0U; row < 15U; ++row)
        {
            for (std::size_t column = 0U; column < 15U; ++column)
            {
                auto input = fixture;
                input.P.set(row, column, bad_value);
                check_failure(test, input, Status::non_finite_input, profile);
            }
        }
    }
    for (std::size_t axis = 0U; axis < F::size; ++axis)
    {
        auto input = fixture;
        input.variances.set(axis, Scalar{-1});
        check_failure(test, input, Status::domain_error, profile);
        input.observation.set(axis, nan);
        check_failure(test, input, Status::non_finite_input, profile);
        for (Scalar observation : {max, -max})
        {
            input = fixture;
            if constexpr (F::velocity)
            {
                input.state.v_n.set(F::axis_offset + axis, -observation);
            }
            else
            {
                input.state.p_n.set(F::axis_offset + axis, -observation);
            }
            input.observation.set(axis, observation);
            check_failure(test, input, Status::non_finite_result, profile);
        }
    }
    for (std::size_t row = 0U; row < 15U; ++row)
    {
        auto input = fixture;
        input.P.set(row, row, Scalar{-1});
        check_failure(test, input, Status::domain_error, profile);
        for (std::size_t column = row + 1U; column < 15U; ++column)
        {
            input = fixture;
            input.P.set(row, column, Scalar{1});
            check_failure(test, input, Status::domain_error, profile);
        }
    }
    auto extreme = fixture;
    extreme.variances.set(0U, max);
    if constexpr (!F::vertical)
    {
        extreme.variances.set(1U, max);
    }
    if constexpr (Group == ResetGroup::horizontal_position)
    {
        check_failure(test, extreme, Status::non_finite_result, profile);
    }
    else
    {
        check_success(test, extreme, profile);
    }
}

} /* end namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    TestContext test;
    run_conformance<float, ResetGroup::horizontal_position>(test, "horizontal position / float");
    run_conformance<double, ResetGroup::horizontal_position>(test, "horizontal position / double");
    run_conformance<float, ResetGroup::vertical_position>(test, "vertical position / float");
    run_conformance<double, ResetGroup::vertical_position>(test, "vertical position / double");
    run_conformance<float, ResetGroup::vertical_velocity>(test, "vertical velocity / float");
    run_conformance<double, ResetGroup::vertical_velocity>(test, "vertical velocity / double");
    run_conformance<float, ResetGroup::velocity>(test, "velocity / float");
    run_conformance<double, ResetGroup::velocity>(test, "velocity / double");
    if (test.failures() != 0)
    {
        return 1;
    }
    std::cout << "GNSS reset tests passed\n";
    return 0;
}
