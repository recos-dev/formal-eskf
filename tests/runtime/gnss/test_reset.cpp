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
#include <cmath>
#include <iostream>
#include <limits>
#include <string_view>

// Exercise the standalone public header on each configured backend.
#include <formal_eskf/runtime/navigation.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"
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
            state.p_n(axis) = n * Scalar{10};
            state.v_n(axis) = -n * Scalar{2};
            state.b_a(axis) = n / Scalar{8};
            state.b_g(axis) = -n / Scalar{16};
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
                P(row, column) = value;
            }
        }
        // A retained signed zero must not be changed by blanket symmetrization.
        P(10U, 11U) = -Scalar{0};
        P(11U, 10U) = -Scalar{0};
        for (std::size_t axis = 0U; axis < size; ++axis)
        {
            observation(axis) = Scalar{4} - static_cast<Scalar>(axis) * Scalar{7};
            variances(axis) = Scalar{1} + static_cast<Scalar>(axis) * Scalar{3};
        }
    }

    [[nodiscard]] Status reset(state_type & state_output, covariance_type & covariance_output,
                               vector_type & delta_output) const
    {
        if constexpr (Group == ResetGroup::horizontal_position)
        {
            return formal_eskf::runtime::reset_gnss_horizontal_position(state, P, observation, variances, state_output,
                                                                        covariance_output, delta_output);
        }
        else if constexpr (vertical)
        {
            Scalar delta = delta_output(0U);
            Status status{};
            if constexpr (velocity)
            {
                status = formal_eskf::runtime::reset_vertical_velocity(state, P, observation(0U), variances(0U),
                                                                       state_output, covariance_output, delta);
            }
            else
            {
                status = formal_eskf::runtime::reset_vertical_position(state, P, observation(0U), variances(0U),
                                                                       state_output, covariance_output, delta);
            }
            // Forward even on failure: rollback checks must see the actual scalar output.
            delta_output(0U) = delta;
            return status;
        }
        else
        {
            return formal_eskf::runtime::reset_velocity(state, P, observation, variances, state_output,
                                                        covariance_output, delta_output);
        }
    }
};

template <typename Scalar, ResetGroup Group>
void check_success(TestContext & test, Fixture<Scalar, Group> const & fixture, std::string_view profile,
                   unsigned alias_mask = 15U)
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
        // Coefficient writes require the non-const matrix operator().
        // cppcheck-suppress constVariableReference
        auto & target = F::velocity ? expected_state.v_n : expected_state.p_n;
        auto const state_axis = F::axis_offset + axis;
        expected_delta(axis) = fixture.observation(axis) - target(state_axis);
        target(state_axis) = fixture.observation(axis);
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
                expected_P(row, column) = Scalar{0};
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
                expected_P(row, column) = variance < floor ? floor : variance;
            }
        }
    }
    for (unsigned aliases = 0U; aliases < 4U; ++aliases)
    {
        if ((alias_mask & (1U << aliases)) == 0U)
        {
            continue;
        }
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
        separate_state.p_n(0U) = Scalar{99};
        auto separate_P = F::covariance_type::identity() * Scalar{99};
        auto & state = (aliases & 1U) != 0U ? input.state : separate_state;
        auto & P = (aliases & 2U) != 0U ? input.P : separate_P;
        auto const before_state = state;
        auto const before_P = P;
        typename F::vector_type delta;
        for (std::size_t axis = 0U; axis < F::size; ++axis)
        {
            delta(axis) = Scalar{-64};
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

template <typename Scalar, ResetGroup Group> void equations(TestContext & test, std::string_view profile)
{
    using F = Fixture<Scalar, Group>;
    F fixture;
    test.expect(F::state_type::quaternion_type::from_coefficients(Scalar{0.5}, Scalar{-0.5}, Scalar{0.5}, Scalar{-0.5},
                                                                  Scalar{0.125}, fixture.state.q_nb) == Status::success,
                profile, "non-identity attitude fixture is valid");
    check_success(test, fixture, profile, 1U);
    constexpr Scalar floor = static_cast<Scalar>(0.01) * static_cast<Scalar>(0.01);
    for (Scalar variance : {Scalar{0}, -Scalar{0}, std::numeric_limits<Scalar>::denorm_min(), floor / Scalar{4},
                            floor / Scalar{2}, std::nextafter(floor, Scalar{0}), floor,
                            std::nextafter(floor, std::numeric_limits<Scalar>::infinity()), Scalar{4}})
    {
        auto input = fixture;
        for (std::size_t axis = 0U; axis < F::size; ++axis)
        {
            input.variances(axis) = variance;
        }
        check_success(test, input, profile, 1U);
    }
    auto mixed = fixture;
    mixed.variances(0U) = Scalar{0};
    check_success(test, mixed, profile, 1U);
    auto singular = fixture;
    singular.P = typename F::covariance_type{};
    check_success(test, singular, profile, 1U);
}
template <typename Scalar, ResetGroup Group> void aliases(TestContext & test, std::string_view profile)
{
    using F = Fixture<Scalar, Group>;
    F fixture;
    test.expect(F::state_type::quaternion_type::from_coefficients(Scalar{0.5}, Scalar{-0.5}, Scalar{0.5}, Scalar{-0.5},
                                                                  Scalar{0.125}, fixture.state.q_nb) == Status::success,
                profile, "exact nonidentity fixture");
    check_success(test, fixture, profile);
    if constexpr (F::vertical)
    {
        Scalar const prior = F::velocity ? fixture.state.v_n(2U) : fixture.state.p_n(2U);
        for (Scalar down : {Scalar{-40}, -Scalar{0}, Scalar{0}, prior, Scalar{40}})
        {
            auto input = fixture;
            input.observation(0U) = down;
            check_success(test, input, profile);
        }
    }
}
template <typename Scalar, ResetGroup Group> void failures(TestContext & test, std::string_view profile)
{
    using F = Fixture<Scalar, Group>;
    F fixture;
    Scalar const nan = std::numeric_limits<Scalar>::quiet_NaN();
    Scalar const inf = std::numeric_limits<Scalar>::infinity();
    Scalar const max = std::numeric_limits<Scalar>::max();
    for (Scalar bad_value : {nan, inf, -inf})
    {
        for (std::size_t axis = 0U; axis < F::size; ++axis)
        {
            auto input = fixture;
            input.observation(axis) = bad_value;
            check_failure(test, input, Status::non_finite_input, profile);
            input = fixture;
            input.variances(axis) = bad_value;
            check_failure(test, input, Status::non_finite_input, profile);
        }
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            for (auto member : {&F::state_type::p_n, &F::state_type::v_n, &F::state_type::b_a, &F::state_type::b_g})
            {
                auto input = fixture;
                (input.state.*member)(axis) = bad_value;
                check_failure(test, input, Status::non_finite_input, profile);
            }
        }
        for (std::size_t row = 0U; row < 15U; ++row)
        {
            for (std::size_t column = 0U; column < 15U; ++column)
            {
                auto input = fixture;
                input.P(row, column) = bad_value;
                check_failure(test, input, Status::non_finite_input, profile);
            }
        }
    }
    for (std::size_t axis = 0U; axis < F::size; ++axis)
    {
        auto input = fixture;
        input.variances(axis) = Scalar{-1};
        check_failure(test, input, Status::domain_error, profile);
        input.observation(axis) = nan;
        check_failure(test, input, Status::non_finite_input, profile);
        for (Scalar observation : {max, -max})
        {
            input = fixture;
            if constexpr (F::velocity)
            {
                input.state.v_n(F::axis_offset + axis) = -observation;
            }
            else
            {
                input.state.p_n(F::axis_offset + axis) = -observation;
            }
            input.observation(axis) = observation;
            check_failure(test, input, Status::non_finite_result, profile);
        }
    }
    for (std::size_t row = 0U; row < 15U; ++row)
    {
        auto input = fixture;
        input.P(row, row) = Scalar{-1};
        check_failure(test, input, Status::domain_error, profile);
        for (std::size_t column = row + 1U; column < 15U; ++column)
        {
            input = fixture;
            input.P(row, column) = Scalar{1};
            check_failure(test, input, Status::domain_error, profile);
        }
    }
    auto extreme = fixture;
    extreme.variances(0U) = max;
    if constexpr (!F::vertical)
    {
        extreme.variances(1U) = max;
    }
    if constexpr (Group == ResetGroup::horizontal_position)
    {
        check_failure(test, extreme, Status::non_finite_result, profile);
    }
    else
    {
        check_success(test, extreme, profile);
    }
    // Reach nonfinite quaternion coefficients through the public raw product, without mutating private representation.
    auto malformed = fixture;
    Scalar const tiny = std::ldexp(Scalar{1}, std::is_same_v<Scalar, float> ? -74 : -537);
    test.expect(F::state_type::quaternion_type::from_coefficients(tiny, tiny / Scalar{2}, Scalar{0}, Scalar{0},
                                                                  std::numeric_limits<Scalar>::min(),
                                                                  malformed.state.q_nb) == Status::success,
                profile, "public rounded normalization witness");
    for (unsigned count = 0; count < 64U && formal_eskf::linalg::all_finite(malformed.state.q_nb.coefficients());
         ++count)
    {
        malformed.state.q_nb = malformed.state.q_nb * malformed.state.q_nb;
    }
    test.expect(!formal_eskf::linalg::all_finite(malformed.state.q_nb.coefficients()), profile,
                "raw multiplication reaches nonfinite prior");
    check_failure(test, malformed, Status::non_finite_input, profile);
}
template <typename Scalar, ResetGroup Group> void repeated_reset(TestContext & test, std::string_view profile)
{
    using F = Fixture<Scalar, Group>;
    using Linalg = typename F::linalg_type;
    F fixture;
    typename F::vector_type delta;
    test.expect(fixture.reset(fixture.state, fixture.P, delta) == Status::success, profile,
                "first actual reset publishes prior");
    auto const once = fixture;
    test.expect(fixture.reset(fixture.state, fixture.P, delta) == Status::success &&
                    same_state(fixture.state, once.state) && same_bits(fixture.P, once.P) &&
                    formal_eskf::linalg::max_abs(delta) == Scalar{0},
                profile, "identical reset is idempotent and reports zero actual change");
    using Position = formal_eskf::linalg::Vector<Linalg, 2U>;
    using Noise = formal_eskf::linalg::Matrix<Linalg, 2U, 2U>;
    Position position =
        Position::from_row_major({fixture.state.p_n(0) + Scalar{0.125}, fixture.state.p_n(1) - Scalar{0.25}});
    auto const corrected =
        formal_eskf::runtime::fuse_horizontal_position(fixture.state, fixture.P, position, Noise::identity(),
                                                       static_cast<Scalar>(1e-6), Scalar{10}, fixture.state, fixture.P);
    test.expect(
        corrected.status == Status::success && corrected.decision == formal_eskf::runtime::FusionDecision::fused,
        profile, "interleave actual correction before a later reset; correction accuracy belongs to its own cases");
    for (std::size_t axis = 0; axis < F::size; ++axis)
    {
        fixture.observation(axis) += static_cast<Scalar>(axis + 1U);
    }
    check_success(test, fixture, profile);
    auto const before = fixture;
    test.expect(fixture.reset(fixture.state, fixture.P, delta) == Status::success, profile,
                "later supplied observation resets current estimate");
    for (std::size_t axis = 0; axis < F::size; ++axis)
    {
        auto const prior =
            F::velocity ? before.state.v_n(F::axis_offset + axis) : before.state.p_n(F::axis_offset + axis);
        test.expect(delta(axis) == fixture.observation(axis) - prior, profile,
                    "delta references corrected current state, not original reset history");
    }
}

} /* end namespace */

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Case;
    Case const cases[]{
        {"TC-R-GNSS-RESET-01", "horizontal_position.binary32", equations<float, ResetGroup::horizontal_position>},
        {"TC-R-GNSS-RESET-01", "horizontal_position.binary64", equations<double, ResetGroup::horizontal_position>},
        {"TC-R-GNSS-RESET-01", "vertical_position.binary32", equations<float, ResetGroup::vertical_position>},
        {"TC-R-GNSS-RESET-01", "vertical_position.binary64", equations<double, ResetGroup::vertical_position>},
        {"TC-R-GNSS-RESET-01", "vertical_velocity.binary32", equations<float, ResetGroup::vertical_velocity>},
        {"TC-R-GNSS-RESET-01", "vertical_velocity.binary64", equations<double, ResetGroup::vertical_velocity>},
        {"TC-R-GNSS-RESET-01", "velocity.binary32", equations<float, ResetGroup::velocity>},
        {"TC-R-GNSS-RESET-01", "velocity.binary64", equations<double, ResetGroup::velocity>},
        {"TC-R-GNSS-RESET-02", "horizontal_position.binary32", aliases<float, ResetGroup::horizontal_position>},
        {"TC-R-GNSS-RESET-02", "horizontal_position.binary64", aliases<double, ResetGroup::horizontal_position>},
        {"TC-R-GNSS-RESET-02", "vertical_position.binary32", aliases<float, ResetGroup::vertical_position>},
        {"TC-R-GNSS-RESET-02", "vertical_position.binary64", aliases<double, ResetGroup::vertical_position>},
        {"TC-R-GNSS-RESET-02", "vertical_velocity.binary32", aliases<float, ResetGroup::vertical_velocity>},
        {"TC-R-GNSS-RESET-02", "vertical_velocity.binary64", aliases<double, ResetGroup::vertical_velocity>},
        {"TC-R-GNSS-RESET-02", "velocity.binary32", aliases<float, ResetGroup::velocity>},
        {"TC-R-GNSS-RESET-02", "velocity.binary64", aliases<double, ResetGroup::velocity>},
        {"TC-R-GNSS-RESET-03", "horizontal_position.binary32", failures<float, ResetGroup::horizontal_position>},
        {"TC-R-GNSS-RESET-03", "horizontal_position.binary64", failures<double, ResetGroup::horizontal_position>},
        {"TC-R-GNSS-RESET-03", "vertical_position.binary32", failures<float, ResetGroup::vertical_position>},
        {"TC-R-GNSS-RESET-03", "vertical_position.binary64", failures<double, ResetGroup::vertical_position>},
        {"TC-R-GNSS-RESET-03", "vertical_velocity.binary32", failures<float, ResetGroup::vertical_velocity>},
        {"TC-R-GNSS-RESET-03", "vertical_velocity.binary64", failures<double, ResetGroup::vertical_velocity>},
        {"TC-R-GNSS-RESET-03", "velocity.binary32", failures<float, ResetGroup::velocity>},
        {"TC-R-GNSS-RESET-03", "velocity.binary64", failures<double, ResetGroup::velocity>},
        {"TC-R-GNSS-RESET-04", "horizontal_position.binary32", repeated_reset<float, ResetGroup::horizontal_position>},
        {"TC-R-GNSS-RESET-04", "horizontal_position.binary64", repeated_reset<double, ResetGroup::horizontal_position>},
        {"TC-R-GNSS-RESET-04", "vertical_position.binary32", repeated_reset<float, ResetGroup::vertical_position>},
        {"TC-R-GNSS-RESET-04", "vertical_position.binary64", repeated_reset<double, ResetGroup::vertical_position>},
        {"TC-R-GNSS-RESET-04", "vertical_velocity.binary32", repeated_reset<float, ResetGroup::vertical_velocity>},
        {"TC-R-GNSS-RESET-04", "vertical_velocity.binary64", repeated_reset<double, ResetGroup::vertical_velocity>},
        {"TC-R-GNSS-RESET-04", "velocity.binary32", repeated_reset<float, ResetGroup::velocity>},
        {"TC-R-GNSS-RESET-04", "velocity.binary64", repeated_reset<double, ResetGroup::velocity>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
