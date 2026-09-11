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
#include <initializer_list>
#include <iostream>
#include <limits>
#include <string_view>

// Check that the measurement header includes its own dependencies.
#include <formal_eskf/eskf/velocity.hpp>
#include <formal_eskf/eskf/covariance_prediction.hpp>
#include <formal_eskf/linalg/backend/eigen.hpp>

#include "test_support.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::Status;
using formal_eskf::test::near;

template <typename Matrix> [[nodiscard]] bool same_bits(Matrix const & a, Matrix const & b)
{
    using bytes_type = std::array<std::byte, sizeof(typename Matrix::value_type)>;
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            if (std::bit_cast<bytes_type>(a(row, column)) != std::bit_cast<bytes_type>(b(row, column)))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename State> [[nodiscard]] bool same_state(State const & a, State const & b)
{
    return same_bits(a.v_n, b.v_n) && same_bits(a.p_n, b.p_n) && same_bits(a.b_a, b.b_a) && same_bits(a.b_g, b.b_g) &&
           same_bits(a.q_nb.coefficients(), b.q_nb.coefficients());
}

template <typename Linalg> struct HorizontalVelocityFixture
{
    using linalg_type = Linalg;
    using value_type = typename Linalg::value_type;
    using state_type = formal_eskf::configuration::Ins::NominalState<Linalg>;
    using covariance_type = formal_eskf::linalg::Matrix<Linalg, 15U, 15U>;
    using vector_type = formal_eskf::linalg::Matrix<Linalg, 3U, 1U>;
    using measurement_type = formal_eskf::linalg::Matrix<Linalg, 2U, 1U>;
    using measurement_covariance_type = formal_eskf::linalg::Matrix<Linalg, 2U, 2U>;
    using jacobian_type = formal_eskf::linalg::Matrix<Linalg, 2U, 15U>;
    static constexpr std::size_t axis_offset = 0U;
    static constexpr value_type minimum_norm = static_cast<value_type>(1.0e-6);

    state_type state{};
    covariance_type P = covariance_type::identity();
    measurement_type z{};
    measurement_covariance_type V = measurement_covariance_type::identity();

    HorizontalVelocityFixture()
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            value_type const n = static_cast<value_type>(axis + 1U);
            state.v_n.set(axis, n);
            state.p_n.set(axis, -n);
            state.b_a.set(axis, n / value_type{8});
            state.b_g.set(axis, -n / value_type{16});
        }
        for (std::size_t axis = 0U; axis < measurement_type::row_count; ++axis)
        {
            value_type const n = state.v_n(axis_offset + axis);
            z.set(axis, n + n / value_type{32});
        }
    }

    [[nodiscard]] static jacobian_type jacobian() noexcept
    {
        return formal_eskf::horizontal_velocity_jacobian<Linalg>();
    }

    [[nodiscard]] static constexpr bool observes_axis(std::size_t axis) noexcept { return axis < 2U; }

    [[nodiscard]] static Status correct(state_type const & prior, covariance_type const & covariance,
                                        measurement_type const & observation, measurement_covariance_type const & noise,
                                        value_type norm_bound, state_type & posterior,
                                        covariance_type & posterior_covariance) noexcept
    {
        return formal_eskf::try_correct_horizontal_velocity(prior, covariance, observation, noise, norm_bound,
                                                            posterior, posterior_covariance);
    }
};

template <typename Linalg> struct VelocityFixture
{
    using linalg_type = Linalg;
    using value_type = typename Linalg::value_type;
    using state_type = formal_eskf::configuration::Ins::NominalState<Linalg>;
    using covariance_type = formal_eskf::linalg::Matrix<Linalg, 15U, 15U>;
    using vector_type = formal_eskf::linalg::Matrix<Linalg, 3U, 1U>;
    using measurement_type = formal_eskf::linalg::Matrix<Linalg, 3U, 1U>;
    using measurement_covariance_type = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    using jacobian_type = formal_eskf::linalg::Matrix<Linalg, 3U, 15U>;
    static constexpr std::size_t axis_offset = 0U;
    static constexpr value_type minimum_norm = static_cast<value_type>(1.0e-6);

    state_type state{};
    covariance_type P = covariance_type::identity();
    measurement_type z{};
    measurement_covariance_type V = measurement_covariance_type::identity();

    VelocityFixture()
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            value_type const n = static_cast<value_type>(axis + 1U);
            state.v_n.set(axis, n);
            state.p_n.set(axis, -n);
            state.b_a.set(axis, n / value_type{8});
            state.b_g.set(axis, -n / value_type{16});
        }
        for (std::size_t axis = 0U; axis < measurement_type::row_count; ++axis)
        {
            value_type const n = state.v_n(axis_offset + axis);
            z.set(axis, n + n / value_type{32});
        }
    }

    [[nodiscard]] static jacobian_type jacobian() noexcept { return formal_eskf::velocity_jacobian<Linalg>(); }

    [[nodiscard]] static constexpr bool observes_axis(std::size_t axis) noexcept { return axis < 3U; }

    [[nodiscard]] static Status correct(state_type const & prior, covariance_type const & covariance,
                                        measurement_type const & observation, measurement_covariance_type const & noise,
                                        value_type norm_bound, state_type & posterior,
                                        covariance_type & posterior_covariance) noexcept
    {
        return formal_eskf::try_correct_velocity(prior, covariance, observation, noise, norm_bound, posterior,
                                                 posterior_covariance);
    }
};

template <typename Linalg, typename State, typename Covariance, std::size_t Size>
concept SupportsHorizontalVelocity = requires(State state, Covariance P,
                                              formal_eskf::linalg::Matrix<Linalg, Size, 1U> z,
                                              formal_eskf::linalg::Matrix<Linalg, Size, Size> V)
{
    formal_eskf::try_correct_horizontal_velocity(state, P, z, V, typename Linalg::value_type{1}, state, P);
};

template <typename Linalg, typename State, typename Covariance, std::size_t Size>
concept SupportsVelocity = requires(State state, Covariance P, formal_eskf::linalg::Matrix<Linalg, Size, 1U> z,
                                    formal_eskf::linalg::Matrix<Linalg, Size, Size> V)
{
    formal_eskf::try_correct_velocity(state, P, z, V, typename Linalg::value_type{1}, state, P);
};

template <typename Fixture>
void test_velocity_jacobian(TestContext & test, std::string_view profile, typename Fixture::value_type tolerance)
{
    using Linalg = typename Fixture::linalg_type;
    constexpr std::size_t measurement_size = Fixture::measurement_type::row_count;
    using value_type = typename Fixture::value_type;
    using error_type = formal_eskf::configuration::Ins::ErrorState<Linalg>;
    static_assert(
        SupportsHorizontalVelocity<Linalg, typename Fixture::state_type, typename Fixture::covariance_type, 2U>);
    static_assert(
        !SupportsHorizontalVelocity<Linalg, typename Fixture::state_type, typename Fixture::covariance_type, 3U>);
    static_assert(!SupportsHorizontalVelocity<Linalg, formal_eskf::configuration::Ahrs::NominalState<Linalg>,
                                              formal_eskf::linalg::Matrix<Linalg, 3U, 3U>, 2U>);
    static_assert(SupportsVelocity<Linalg, typename Fixture::state_type, typename Fixture::covariance_type, 3U>);
    static_assert(!SupportsVelocity<Linalg, typename Fixture::state_type, typename Fixture::covariance_type, 2U>);
    static_assert(!SupportsVelocity<Linalg, formal_eskf::configuration::Ahrs::NominalState<Linalg>,
                                    formal_eskf::linalg::Matrix<Linalg, 3U, 3U>, 3U>);
    static_assert(noexcept(formal_eskf::horizontal_velocity_jacobian<Linalg>()));
    static_assert(noexcept(formal_eskf::velocity_jacobian<Linalg>()));

    Fixture const fixture;
    auto const H = Fixture::jacobian();
    constexpr value_type step = value_type{0.0625};
    // Differentiate h(inject(state, delta_x)), not z - h. This checks the sign
    // and all 15 column meanings against the actual injection convention.
    for (std::size_t column = 0U; column < 15U; ++column)
    {
        error_type positive;
        error_type negative;
        std::array<typename Fixture::vector_type *, 5U> positive_blocks{&positive.delta_p_n, &positive.delta_v_n,
                                                                        &positive.delta_theta_b, &positive.delta_b_a,
                                                                        &positive.delta_b_g};
        std::array<typename Fixture::vector_type *, 5U> negative_blocks{&negative.delta_p_n, &negative.delta_v_n,
                                                                        &negative.delta_theta_b, &negative.delta_b_a,
                                                                        &negative.delta_b_g};
        positive_blocks[column / 3U]->set(column % 3U, step);
        negative_blocks[column / 3U]->set(column % 3U, -step);
        typename Fixture::state_type plus;
        typename Fixture::state_type minus;
        test.expect(formal_eskf::try_inject_nominal(fixture.state, positive, Fixture::minimum_norm, plus) ==
                            Status::success &&
                        formal_eskf::try_inject_nominal(fixture.state, negative, Fixture::minimum_norm, minus) ==
                            Status::success,
                    profile, "finite-difference perturbations succeed");
        for (std::size_t row = 0U; row < measurement_size; ++row)
        {
            value_type const expected = row + 3U == column ? value_type{1} : value_type{0};
            value_type const derivative =
                (plus.v_n(Fixture::axis_offset + row) - minus.v_n(Fixture::axis_offset + row)) / (value_type{2} * step);
            test.expect(H(row, column) == expected && near(H(row, column), derivative, tolerance), profile,
                        "velocity Jacobian matches layout and injected observation derivative");
        }
    }
}

template <typename Fixture>
void test_velocity_analytic(TestContext & test, std::string_view profile, typename Fixture::value_type tolerance)
{
    constexpr std::size_t measurement_size = Fixture::measurement_type::row_count;
    using value_type = typename Fixture::value_type;
    Fixture fixture;
    // P is SPD: velocity block 4I, all other diagonal blocks I, with
    // velocity cross-covariances to position and both biases (but not attitude).
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        fixture.P.set(3U + axis, 3U + axis, value_type{4});
        for (std::size_t block : {0U, 3U, 4U})
        {
            value_type const cross = block == 0U ? value_type{1} : (block == 3U ? value_type{0.5} : value_type{-0.25});
            fixture.P.set(3U + axis, 3U * block + axis, cross);
            fixture.P.set(3U * block + axis, 3U + axis, cross);
        }
    }
    for (std::size_t axis = 0U; axis < measurement_size; ++axis)
    {
        fixture.V.set(axis, axis, static_cast<value_type>(axis + 1U));
    }
    fixture.V.set(0U, 1U, value_type{0.5});
    fixture.V.set(1U, 0U, value_type{0.5});
    // Independent 2x2/3x3 inverse: S has diagonal 5, 6, 7 as
    // applicable, and off-diagonal .5 for the first two observed axes.
    // This oracle does not use production Jacobian, solve, Joseph or reset.
    constexpr long double determinant = 5.0L * 6.0L - 0.5L * 0.5L;
    // The horizontal model uses the upper-left 2x2 block of this oracle.
    constexpr std::array<std::array<long double, 3U>, 3U> inverse{{{6.0L / determinant, -0.5L / determinant, 0.0L},
                                                                   {-0.5L / determinant, 5.0L / determinant, 0.0L},
                                                                   {0.0L, 0.0L, 1.0L / 7.0L}}};
    std::array<long double, 15U> correction{};
    typename Fixture::state_type state_output;
    typename Fixture::covariance_type P_output;
    test.expect(Fixture::correct(fixture.state, fixture.P, fixture.z, fixture.V, Fixture::minimum_norm, state_output,
                                 P_output) == Status::success,
                profile, "velocity correction with correlated V succeeds");
    for (std::size_t row = 0U; row < 15U; ++row)
    {
        std::array<long double, measurement_size> gain{};
        for (std::size_t axis = 0U; axis < measurement_size; ++axis)
        {
            for (std::size_t k = 0U; k < measurement_size; ++k)
            {
                gain[axis] += static_cast<long double>(fixture.P(row, 3U + k)) * inverse[k][axis];
            }
            correction[row] += gain[axis] * (static_cast<long double>(fixture.z(axis)) - fixture.state.v_n(axis));
        }
        for (std::size_t column = 0U; column < 15U; ++column)
        {
            long double expected = fixture.P(row, column);
            for (std::size_t axis = 0U; axis < measurement_size; ++axis)
            {
                expected -= gain[axis] * fixture.P(3U + axis, column);
            }
            test.expect(near(P_output(row, column), static_cast<value_type>(expected), tolerance), profile,
                        "velocity covariance matches independent conditional covariance");
        }
    }
    std::array<typename Fixture::vector_type const *, 4U> const before{&fixture.state.v_n, &fixture.state.p_n,
                                                                       &fixture.state.b_a, &fixture.state.b_g};
    std::array<typename Fixture::vector_type const *, 4U> const after{&state_output.v_n, &state_output.p_n,
                                                                      &state_output.b_a, &state_output.b_g};
    constexpr std::array<std::size_t, 4U> offsets{3U, 0U, 9U, 12U};
    for (std::size_t block = 0U; block < offsets.size(); ++block)
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            auto const expected = static_cast<value_type>((*before[block])(axis) + correction[offsets[block] + axis]);
            test.expect(near((*after[block])(axis), expected, tolerance), profile,
                        "residual sign and cross-covariance corrections match analytic result");
        }
    }
    test.expect(same_bits(state_output.q_nb.coefficients(), fixture.state.q_nb.coefficients()), profile,
                "uncorrelated attitude stays unchanged");
}

template <typename Fixture> void test_velocity_delegation(TestContext & test, std::string_view profile)
{
    using Linalg = typename Fixture::linalg_type;
    constexpr std::size_t measurement_size = Fixture::measurement_type::row_count;
    using value_type = typename Fixture::value_type;
    Fixture fixture;
    // Dense SPD P and V exercise attitude injection/reset and cross-axis noise.
    for (std::size_t row = 0U; row < 15U; ++row)
    {
        for (std::size_t column = 0U; column < 15U; ++column)
        {
            fixture.P(row, column) += static_cast<value_type>((row + 1U) * (column + 1U)) / value_type{256};
        }
    }
    fixture.V.set(0U, 1U, value_type{0.25});
    fixture.V.set(1U, 0U, value_type{0.25});
    test.expect(formal_eskf::so3::UnitQuaternion<Linalg>::try_from_coefficients(
                    value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5}, Fixture::minimum_norm,
                    fixture.state.q_nb) == Status::success,
                profile, "non-identity prior attitude is valid");
    formal_eskf::linalg::Matrix<Linalg, measurement_size, 15U> H;
    for (std::size_t axis = 0U; axis < measurement_size; ++axis)
    {
        H.set(axis, 3U + axis, value_type{1});
    }
    auto const r = fixture.z - fixture.state.v_n.template segment<Fixture::axis_offset, measurement_size>();
    typename Fixture::state_type expected_state;
    typename Fixture::covariance_type expected_P;
    test.expect(formal_eskf::try_correct(fixture.state, fixture.P, r, H, fixture.V, Fixture::minimum_norm,
                                         expected_state, expected_P) == Status::success,
                profile, "generic correction reference succeeds");
    test.expect(!same_bits(expected_state.q_nb.coefficients(), fixture.state.q_nb.coefficients()), profile,
                "velocity-attitude cross covariance produces an attitude correction");
    for (unsigned aliases = 0U; aliases < 4U; ++aliases)
    {
        auto input = fixture;
        typename Fixture::state_type separate_state;
        typename Fixture::covariance_type separate_P;
        auto & state_output = (aliases & 1U) != 0U ? input.state : separate_state;
        auto & P_output = (aliases & 2U) != 0U ? input.P : separate_P;
        test.expect(Fixture::correct(input.state, input.P, input.z, input.V, Fixture::minimum_norm, state_output,
                                     P_output) == Status::success,
                    profile, "all output-alias combinations succeed");
        test.expect(same_state(state_output, expected_state) && same_bits(P_output, expected_P), profile,
                    "wrapper preserves full generic correction result, including attitude reset");
        test.expect(((aliases & 1U) != 0U || same_state(input.state, fixture.state)) &&
                        ((aliases & 2U) != 0U || same_bits(input.P, fixture.P)) && same_bits(input.z, fixture.z) &&
                        same_bits(input.V, fixture.V),
                    profile, "non-output inputs remain unchanged");
    }
    auto zero_residual = fixture;
    test.expect(Fixture::correct(zero_residual.state, zero_residual.P,
                                 zero_residual.state.v_n.template segment<Fixture::axis_offset, measurement_size>(),
                                 zero_residual.V, Fixture::minimum_norm, zero_residual.state,
                                 zero_residual.P) == Status::success,
                profile, "measurement from in-place state succeeds");
    test.expect(same_state(zero_residual.state, fixture.state) && zero_residual.P(3U, 3U) < fixture.P(3U, 3U), profile,
                "zero residual leaves state unchanged but still reduces velocity uncertainty");
}

template <typename Fixture> void test_velocity_failures(TestContext & test, std::string_view profile)
{
    constexpr std::size_t measurement_size = Fixture::measurement_type::row_count;
    using value_type = typename Fixture::value_type;
    Fixture const fixture;
    auto const fail = [&](Fixture const & bad, value_type minimum_norm, Status expected, std::string_view description)
    {
        for (unsigned aliases = 0U; aliases < 4U; ++aliases)
        {
            auto input = bad;
            auto separate_state = fixture.state;
            typename Fixture::covariance_type separate_P = fixture.P * value_type{7};
            auto & state_output = (aliases & 1U) != 0U ? input.state : separate_state;
            auto & P_output = (aliases & 2U) != 0U ? input.P : separate_P;
            auto const state_before = state_output;
            auto const P_before = P_output;
            Status const status =
                Fixture::correct(input.state, input.P, input.z, input.V, minimum_norm, state_output, P_output);
            test.expect(status == expected && same_state(state_output, state_before) && same_bits(P_output, P_before),
                        profile, description);
            test.expect(same_state(input.state, bad.state) && same_bits(input.P, bad.P) && same_bits(input.z, bad.z) &&
                            same_bits(input.V, bad.V),
                        profile, "failed velocity correction preserves all inputs bit-for-bit");
        }
    };
    for (value_type invalid :
         {std::numeric_limits<value_type>::quiet_NaN(), std::numeric_limits<value_type>::infinity(),
          -std::numeric_limits<value_type>::infinity()})
    {
        for (std::size_t axis = 0U; axis < measurement_size; ++axis)
        {
            auto bad = fixture;
            bad.z.set(axis, invalid);
            fail(bad, Fixture::minimum_norm, Status::non_finite_input, "non-finite measured velocity rolls back");
            bad = fixture;
            bad.V.set(axis, axis, invalid);
            fail(bad, Fixture::minimum_norm, Status::non_finite_input, "non-finite measurement variance rolls back");
        }
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            auto bad = fixture;
            bad.state.v_n.set(axis, invalid);
            fail(bad, Fixture::minimum_norm, Status::non_finite_input, "non-finite prior velocity rolls back");
        }
    }
    auto bad = fixture;
    bad.state.v_n.set(Fixture::axis_offset, -std::numeric_limits<value_type>::max());
    bad.z.set(0U, std::numeric_limits<value_type>::max());
    fail(bad, Fixture::minimum_norm, Status::non_finite_result, "finite velocity subtraction overflow rolls back");
    bad = fixture;
    bad.state.p_n.set(0U, std::numeric_limits<value_type>::quiet_NaN());
    fail(bad, Fixture::minimum_norm, Status::non_finite_input, "downstream injection validation rolls back");
    bad = fixture;
    bad.V.set(0U, 0U, std::numeric_limits<value_type>::infinity());
    fail(bad, Fixture::minimum_norm, Status::non_finite_input, "non-finite measurement covariance rolls back");
    bad = fixture;
    bad.V.set(0U, 1U, value_type{0.25});
    fail(bad, Fixture::minimum_norm, Status::domain_error, "asymmetric V is not silently repaired");
    for (value_type variance : {value_type{0}, value_type{-1}})
    {
        bad = fixture;
        bad.V.set(measurement_size - 1U, measurement_size - 1U, variance);
        fail(bad, Fixture::minimum_norm, Status::domain_error, "nonpositive measurement variance rolls back");
    }
    bad = fixture;
    bad.P.set(1U, 0U, value_type{0.25});
    fail(bad, Fixture::minimum_norm, Status::domain_error, "asymmetric prior covariance rolls back");
    bad = fixture;
    bad.P.set(3U, 4U, value_type{4});
    bad.P.set(4U, 3U, value_type{4});
    // Invalid prior violates PSD precondition and gives an indefinite S here.
    fail(bad, Fixture::minimum_norm, Status::not_positive_definite, "innovation Cholesky failure propagates");
    fail(fixture, value_type{0}, Status::domain_error, "invalid quaternion norm bound propagates");
    fail(fixture, std::numeric_limits<value_type>::quiet_NaN(), Status::non_finite_input,
         "non-finite quaternion norm bound propagates");
}

template <typename Fixture>
void test_velocity_translation(TestContext & test, std::string_view profile, typename Fixture::value_type tolerance)
{
    using value_type = typename Fixture::value_type;
    Fixture original;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        original.P.set(axis, 3U + axis, value_type{0.25});
        original.P.set(3U + axis, axis, value_type{0.25});
    }
    auto translated = original;
    auto const offset = Fixture::vector_type::from_row_major({value_type{32}, value_type{-16}, value_type{8}});
    translated.state.p_n = original.state.p_n + offset;
    test.expect(Fixture::correct(original.state, original.P, original.z, original.V, Fixture::minimum_norm,
                                 original.state, original.P) == Status::success &&
                    Fixture::correct(translated.state, translated.P, translated.z, translated.V, Fixture::minimum_norm,
                                     translated.state, translated.P) == Status::success,
                profile, "velocity corrections at translated prior positions succeed");
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        test.expect(near(translated.state.p_n(axis) - original.state.p_n(axis), offset(axis), tolerance), profile,
                    "velocity correction preserves an arbitrary absolute position offset");
    }
    test.expect(same_bits(translated.state.v_n, original.state.v_n) && same_bits(translated.P, original.P) &&
                    same_bits(translated.state.q_nb.coefficients(), original.state.q_nb.coefficients()) &&
                    same_bits(translated.state.b_a, original.state.b_a) &&
                    same_bits(translated.state.b_g, original.state.b_g),
                profile, "position translation does not affect velocity, attitude, biases or covariance");
}

// Independent scalar position/velocity filter with a VELOCITY observation.
// It uses no production linalg, prediction, correction or quaternion operations.
struct PositionVelocityReference
{
    long double p = 0.0L;
    long double v = 0.0L;
    long double pp = 4.0L;
    long double pv = 0.0L;
    long double vv = 1.0L;

    void predict(long double dt)
    {
        p += v * dt;
        pp += 2.0L * dt * pv + dt * dt * vv;
        pv += dt * vv;
    }

    void correct(long double z, long double variance)
    {
        long double const S = vv + variance;
        long double const kp = pv / S;
        long double const kv = vv / S;
        long double const residual = z - v;
        p += kp * residual;
        v += kv * residual;
        pp -= kp * pv;
        pv *= 1.0L - kv;
        vv *= 1.0L - kv;
    }
};

template <typename Fixture>
void test_velocity_sequence(TestContext & test, std::string_view profile, typename Fixture::value_type tolerance,
                            bool moving)
{
    using Linalg = typename Fixture::linalg_type;
    constexpr std::size_t measurement_size = Fixture::measurement_type::row_count;
    using value_type = typename Fixture::value_type;
    typename Fixture::state_type state;
    typename Fixture::covariance_type P;
    typename Fixture::measurement_covariance_type V;
    formal_eskf::configuration::Ins::Parameters<Linalg> parameters;
    formal_eskf::ImuSample<Linalg> imu;
    constexpr value_type dt = value_type{0.125};
    parameters.dt_min = dt;
    parameters.dt_max = dt;
    parameters.minimum_quaternion_norm = Fixture::minimum_norm;
    parameters.gravity_n.set(2U, value_type{8});
    imu.specific_force_b.set(2U, value_type{-8});
    // Known attitude/bias and zero process noise reduce this scenario to three
    // independent position-velocity filters. Velocity does not observe p(0).
    std::array<long double, 3U> const truth_velocity =
        moving ? std::array<long double, 3U>{1.0L, -0.5L, 0.25L} : std::array<long double, 3U>{};
    std::array<long double, 3U> const initial_position{2.0L, -1.0L, 3.0L};
    std::array<long double, 3U> const velocity_error{0.5L, -0.25L, 0.125L};
    std::array<PositionVelocityReference, 3U> reference{};
    typename Fixture::measurement_type z;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        reference[axis].p = initial_position[axis];
        reference[axis].v = truth_velocity[axis] + velocity_error[axis];
        state.p_n.set(axis, static_cast<value_type>(reference[axis].p));
        state.v_n.set(axis, static_cast<value_type>(reference[axis].v));
        P.set(axis, axis, value_type{4});
        P.set(3U + axis, 3U + axis, value_type{1});
    }
    for (std::size_t axis = 0U; axis < measurement_size; ++axis)
    {
        V.set(axis, axis, static_cast<value_type>(axis + 1U) / value_type{4});
        z.set(axis, static_cast<value_type>(truth_velocity[axis]));
    }
    constexpr std::size_t steps = 80U;
    for (std::size_t step = 1U; step <= steps; ++step)
    {
        for (auto & axis_reference : reference)
        {
            axis_reference.predict(dt);
        }
        for (std::size_t axis = 0U; axis < measurement_size; ++axis)
        {
            reference[axis].correct(z(axis), V(axis, axis));
        }
        Status const predict_status = formal_eskf::try_predict(state, P, imu, dt, parameters, state, P);
        Status const correct_status = Fixture::correct(state, P, z, V, Fixture::minimum_norm, state, P);
        test.expect(predict_status == Status::success && correct_status == Status::success, profile,
                    "repeated in-place predict and velocity correct succeed");
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            auto const & expected = reference[axis];
            test.expect(near(state.p_n(axis), static_cast<value_type>(expected.p), tolerance) &&
                            near(state.v_n(axis), static_cast<value_type>(expected.v), tolerance) &&
                            near(P(axis, axis), static_cast<value_type>(expected.pp), tolerance) &&
                            near(P(axis, 3U + axis), static_cast<value_type>(expected.pv), tolerance) &&
                            near(P(3U + axis, 3U + axis), static_cast<value_type>(expected.vv), tolerance),
                        profile, "each predict/correct step matches independent velocity-observation filter");
        }
        test.expect(formal_eskf::linalg::all_finite(P) && formal_eskf::linalg::is_symmetric(P, value_type{0}), profile,
                    "sequence covariance remains finite and symmetric");
    }
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        long double const time = static_cast<long double>(steps) * dt;
        if (!Fixture::observes_axis(axis))
        {
            auto const expected_v = static_cast<value_type>(truth_velocity[axis] + velocity_error[axis]);
            auto const expected_p = static_cast<value_type>(initial_position[axis] + time * expected_v);
            test.expect(near(state.v_n(axis), expected_v, tolerance) && near(state.p_n(axis), expected_p, tolerance) &&
                            near(P(3U + axis, 3U + axis), value_type{1}, tolerance) &&
                            near(P(axis, axis), static_cast<value_type>(4.0L + time * time), tolerance),
                        profile, "unobserved vertical axis follows prediction only, without a fabricated measurement");
            continue;
        }
        long double const variance = V(axis, axis);
        // Initial velocity variance is 1; each of N observations adds 1/V
        // information. p(t) = p(0) + t*v; p(0) remains unobserved.
        long double const vv = variance / (variance + static_cast<long double>(steps));
        auto const expected_v = static_cast<value_type>(truth_velocity[axis] + velocity_error[axis] * vv);
        auto const expected_p =
            static_cast<value_type>(initial_position[axis] + time * (truth_velocity[axis] + velocity_error[axis] * vv));
        test.expect(near(state.v_n(axis), expected_v, tolerance) && near(state.p_n(axis), expected_p, tolerance) &&
                        near(P(axis, axis), static_cast<value_type>(4.0L + time * time * vv), tolerance) &&
                        near(P(3U + axis, 3U + axis), static_cast<value_type>(vv), tolerance),
                    profile, "sequence matches closed-form posterior and retains initial position uncertainty");
        test.expect(near(state.v_n(axis), static_cast<value_type>(truth_velocity[axis]), static_cast<value_type>(0.01)),
                    profile, "velocity observations reduce initial velocity error");
    }
}

template <typename Fixture>
void run_velocity_tests(TestContext & test, std::string_view profile, typename Fixture::value_type tolerance)
{
    test_velocity_jacobian<Fixture>(test, profile, tolerance);
    test_velocity_analytic<Fixture>(test, profile, tolerance);
    test_velocity_delegation<Fixture>(test, profile);
    test_velocity_failures<Fixture>(test, profile);
    test_velocity_translation<Fixture>(test, profile, tolerance);
    test_velocity_sequence<Fixture>(test, profile, tolerance, false);
    test_velocity_sequence<Fixture>(test, profile, tolerance, true);
}

} /* end namespace */

int main()
{
    TestContext test;
    Eigen::internal::set_is_malloc_allowed(false);
    run_velocity_tests<HorizontalVelocityFixture<formal_eskf::linalg::EigenBackend<float>>>(
        test, "INS horizontal velocity binary32", 8.0e-6F);
    run_velocity_tests<HorizontalVelocityFixture<formal_eskf::linalg::EigenBackend<double>>>(
        test, "INS horizontal velocity binary64", 2.0e-12);
    run_velocity_tests<VelocityFixture<formal_eskf::linalg::EigenBackend<float>>>(test, "INS 3D velocity binary32",
                                                                                  8.0e-6F);
    run_velocity_tests<VelocityFixture<formal_eskf::linalg::EigenBackend<double>>>(test, "INS 3D velocity binary64",
                                                                                   2.0e-12);
    if (test.failures() != 0)
    {
        std::cerr << test.failures() << " ESKF velocity test(s) failed\n";
        return 1;
    }
    std::cout << "All ESKF velocity tests passed\n";
    return 0;
}
