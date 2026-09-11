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

// Check that the measurement header includes its own dependencies.
#include <formal_eskf/eskf/position.hpp>
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
    return same_bits(a.p_n, b.p_n) && same_bits(a.v_n, b.v_n) && same_bits(a.b_a, b.b_a) && same_bits(a.b_g, b.b_g) &&
           same_bits(a.q_nb.coefficients(), b.q_nb.coefficients());
}

template <typename Linalg> struct HorizontalPositionFixture
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

    HorizontalPositionFixture()
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            value_type const n = static_cast<value_type>(axis + 1U);
            state.p_n.set(axis, n);
            state.v_n.set(axis, -n);
            state.b_a.set(axis, n / value_type{8});
            state.b_g.set(axis, -n / value_type{16});
        }
        for (std::size_t axis = 0U; axis < measurement_type::row_count; ++axis)
        {
            value_type const n = state.p_n(axis_offset + axis);
            z.set(axis, n + n / value_type{32});
        }
    }

    [[nodiscard]] static jacobian_type jacobian() noexcept
    {
        return formal_eskf::horizontal_position_jacobian<Linalg>();
    }

    [[nodiscard]] static constexpr bool observes_axis(std::size_t axis) noexcept { return axis < 2U; }

    [[nodiscard]] static Status correct(state_type const & prior, covariance_type const & covariance,
                                        measurement_type const & observation, measurement_covariance_type const & noise,
                                        value_type norm_bound, state_type & posterior,
                                        covariance_type & posterior_covariance) noexcept
    {
        return formal_eskf::try_correct_horizontal_position(prior, covariance, observation, noise, norm_bound,
                                                            posterior, posterior_covariance);
    }
};

template <typename Linalg> struct VerticalPositionFixture
{
    using linalg_type = Linalg;
    using value_type = typename Linalg::value_type;
    using state_type = formal_eskf::configuration::Ins::NominalState<Linalg>;
    using covariance_type = formal_eskf::linalg::Matrix<Linalg, 15U, 15U>;
    using vector_type = formal_eskf::linalg::Matrix<Linalg, 3U, 1U>;
    using measurement_type = formal_eskf::linalg::Matrix<Linalg, 1U, 1U>;
    using measurement_covariance_type = formal_eskf::linalg::Matrix<Linalg, 1U, 1U>;
    using jacobian_type = formal_eskf::linalg::Matrix<Linalg, 1U, 15U>;
    static constexpr std::size_t axis_offset = 2U;
    static constexpr value_type minimum_norm = static_cast<value_type>(1.0e-6);

    state_type state{};
    covariance_type P = covariance_type::identity();
    measurement_type z{};
    measurement_covariance_type V = measurement_covariance_type::identity();

    VerticalPositionFixture()
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            value_type const n = static_cast<value_type>(axis + 1U);
            state.p_n.set(axis, n);
            state.v_n.set(axis, -n);
            state.b_a.set(axis, n / value_type{8});
            state.b_g.set(axis, -n / value_type{16});
        }
        for (std::size_t axis = 0U; axis < measurement_type::row_count; ++axis)
        {
            value_type const n = state.p_n(axis_offset + axis);
            z.set(axis, n + n / value_type{32});
        }
    }

    [[nodiscard]] static jacobian_type jacobian() noexcept { return formal_eskf::vertical_position_jacobian<Linalg>(); }

    [[nodiscard]] static constexpr bool observes_axis(std::size_t axis) noexcept { return axis == 2U; }

    [[nodiscard]] static Status correct(state_type const & prior, covariance_type const & covariance,
                                        measurement_type const & observation, measurement_covariance_type const & noise,
                                        value_type norm_bound, state_type & posterior,
                                        covariance_type & posterior_covariance) noexcept
    {
        return formal_eskf::try_correct_vertical_position(prior, covariance, observation(0U), noise(0U, 0U), norm_bound,
                                                          posterior, posterior_covariance);
    }
};

template <typename Linalg, typename State, typename Covariance, std::size_t Size>
concept SupportsHorizontalPosition = requires(State state, Covariance P,
                                              formal_eskf::linalg::Matrix<Linalg, Size, 1U> z,
                                              formal_eskf::linalg::Matrix<Linalg, Size, Size> V)
{
    formal_eskf::try_correct_horizontal_position(state, P, z, V, typename Linalg::value_type{1}, state, P);
};

template <typename Linalg, typename State, typename Covariance>
concept SupportsVerticalPosition = requires(State state, Covariance P, typename Linalg::value_type z)
{
    formal_eskf::try_correct_vertical_position(state, P, z, z, z, state, P);
};

template <typename Fixture>
void test_position_jacobian(TestContext & test, std::string_view profile, typename Fixture::value_type tolerance)
{
    using Linalg = typename Fixture::linalg_type;
    constexpr std::size_t measurement_size = Fixture::measurement_type::row_count;
    using value_type = typename Fixture::value_type;
    using error_type = formal_eskf::configuration::Ins::ErrorState<Linalg>;
    static_assert(
        SupportsHorizontalPosition<Linalg, typename Fixture::state_type, typename Fixture::covariance_type, 2U>);
    static_assert(
        !SupportsHorizontalPosition<Linalg, typename Fixture::state_type, typename Fixture::covariance_type, 3U>);
    static_assert(!SupportsHorizontalPosition<Linalg, formal_eskf::configuration::Ahrs::NominalState<Linalg>,
                                              formal_eskf::linalg::Matrix<Linalg, 3U, 3U>, 2U>);
    static_assert(SupportsVerticalPosition<Linalg, typename Fixture::state_type, typename Fixture::covariance_type>);
    static_assert(!SupportsVerticalPosition<Linalg, formal_eskf::configuration::Ahrs::NominalState<Linalg>,
                                            formal_eskf::linalg::Matrix<Linalg, 3U, 3U>>);
    static_assert(noexcept(formal_eskf::horizontal_position_jacobian<Linalg>()));
    static_assert(noexcept(formal_eskf::vertical_position_jacobian<Linalg>()));

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
            value_type const expected = row + Fixture::axis_offset == column ? value_type{1} : value_type{0};
            value_type const derivative =
                (plus.p_n(Fixture::axis_offset + row) - minus.p_n(Fixture::axis_offset + row)) / (value_type{2} * step);
            test.expect(H(row, column) == expected && near(H(row, column), derivative, tolerance), profile,
                        "position Jacobian matches layout and injected observation derivative");
        }
    }
}

template <typename Fixture>
void test_position_analytic(TestContext & test, std::string_view profile, typename Fixture::value_type tolerance)
{
    constexpr std::size_t measurement_size = Fixture::measurement_type::row_count;
    using value_type = typename Fixture::value_type;
    Fixture fixture;
    // P is SPD: position block 4I, all other diagonal blocks I, with
    // position cross-covariances to velocity and both biases (but not attitude).
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        fixture.P.set(axis, axis, value_type{4});
        for (std::size_t block : {1U, 3U, 4U})
        {
            value_type const cross = block == 1U ? value_type{1} : (block == 3U ? value_type{0.5} : value_type{-0.25});
            fixture.P.set(axis, 3U * block + axis, cross);
            fixture.P.set(3U * block + axis, axis, cross);
        }
    }
    for (std::size_t axis = 0U; axis < measurement_size; ++axis)
    {
        fixture.V.set(axis, axis, static_cast<value_type>(axis + 1U));
    }
    if constexpr (measurement_size > 1U)
    {
        fixture.V.set(0U, 1U, value_type{0.5});
        fixture.V.set(1U, 0U, value_type{0.5});
    }
    // Independent scalar/2x2 inverse: S = [5] or [[5,.5],[.5,6]].
    // This oracle does not use production Jacobian, solve, Joseph or reset.
    constexpr auto inverse = []
    {
        if constexpr (measurement_size == 1U)
        {
            return std::array<std::array<long double, 1U>, 1U>{{{1.0L / 5.0L}}};
        }
        else
        {
            constexpr long double determinant = 5.0L * 6.0L - 0.5L * 0.5L;
            return std::array<std::array<long double, 2U>, 2U>{
                {{6.0L / determinant, -0.5L / determinant}, {-0.5L / determinant, 5.0L / determinant}}};
        }
    }();
    std::array<long double, 15U> correction{};
    typename Fixture::state_type state_output;
    typename Fixture::covariance_type P_output;
    test.expect(Fixture::correct(fixture.state, fixture.P, fixture.z, fixture.V, Fixture::minimum_norm, state_output,
                                 P_output) == Status::success,
                profile, "position correction with supplied measurement covariance succeeds");
    for (std::size_t row = 0U; row < 15U; ++row)
    {
        std::array<long double, measurement_size> gain{};
        for (std::size_t axis = 0U; axis < measurement_size; ++axis)
        {
            for (std::size_t k = 0U; k < measurement_size; ++k)
            {
                gain[axis] += static_cast<long double>(fixture.P(row, Fixture::axis_offset + k)) * inverse[k][axis];
            }
            correction[row] += gain[axis] * (static_cast<long double>(fixture.z(axis)) -
                                             fixture.state.p_n(Fixture::axis_offset + axis));
        }
        for (std::size_t column = 0U; column < 15U; ++column)
        {
            long double expected = fixture.P(row, column);
            for (std::size_t axis = 0U; axis < measurement_size; ++axis)
            {
                expected -= gain[axis] * fixture.P(Fixture::axis_offset + axis, column);
            }
            test.expect(near(P_output(row, column), static_cast<value_type>(expected), tolerance), profile,
                        "position covariance matches independent conditional covariance");
        }
    }
    std::array<typename Fixture::vector_type const *, 4U> const before{&fixture.state.p_n, &fixture.state.v_n,
                                                                       &fixture.state.b_a, &fixture.state.b_g};
    std::array<typename Fixture::vector_type const *, 4U> const after{&state_output.p_n, &state_output.v_n,
                                                                      &state_output.b_a, &state_output.b_g};
    constexpr std::array<std::size_t, 4U> offsets{0U, 3U, 9U, 12U};
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

template <typename Fixture> void test_position_delegation(TestContext & test, std::string_view profile)
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
    if constexpr (measurement_size > 1U)
    {
        fixture.V.set(0U, 1U, value_type{0.25});
        fixture.V.set(1U, 0U, value_type{0.25});
    }
    test.expect(formal_eskf::so3::UnitQuaternion<Linalg>::try_from_coefficients(
                    value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5}, Fixture::minimum_norm,
                    fixture.state.q_nb) == Status::success,
                profile, "non-identity prior attitude is valid");
    formal_eskf::linalg::Matrix<Linalg, measurement_size, 15U> H;
    for (std::size_t axis = 0U; axis < measurement_size; ++axis)
    {
        H.set(axis, Fixture::axis_offset + axis, value_type{1});
    }
    auto const r = fixture.z - fixture.state.p_n.template segment<Fixture::axis_offset, measurement_size>();
    typename Fixture::state_type expected_state;
    typename Fixture::covariance_type expected_P;
    test.expect(formal_eskf::try_correct(fixture.state, fixture.P, r, H, fixture.V, Fixture::minimum_norm,
                                         expected_state, expected_P) == Status::success,
                profile, "generic correction reference succeeds");
    test.expect(!same_bits(expected_state.q_nb.coefficients(), fixture.state.q_nb.coefficients()), profile,
                "position-attitude cross covariance produces an attitude correction");
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
                                 zero_residual.state.p_n.template segment<Fixture::axis_offset, measurement_size>(),
                                 zero_residual.V, Fixture::minimum_norm, zero_residual.state,
                                 zero_residual.P) == Status::success,
                profile, "measurement from in-place state succeeds");
    test.expect(same_state(zero_residual.state, fixture.state) &&
                    zero_residual.P(Fixture::axis_offset, Fixture::axis_offset) <
                        fixture.P(Fixture::axis_offset, Fixture::axis_offset),
                profile, "zero residual leaves state unchanged but still reduces position uncertainty");
}

template <typename Fixture> void test_position_failures(TestContext & test, std::string_view profile)
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
                        profile, "failed position correction preserves all inputs bit-for-bit");
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
            fail(bad, Fixture::minimum_norm, Status::non_finite_input, "non-finite measured position rolls back");
            bad = fixture;
            bad.V.set(axis, axis, invalid);
            fail(bad, Fixture::minimum_norm, Status::non_finite_input, "non-finite measurement variance rolls back");
        }
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            auto bad = fixture;
            bad.state.p_n.set(axis, invalid);
            fail(bad, Fixture::minimum_norm, Status::non_finite_input, "non-finite prior position rolls back");
        }
    }
    auto bad = fixture;
    bad.state.p_n.set(Fixture::axis_offset, -std::numeric_limits<value_type>::max());
    bad.z.set(0U, std::numeric_limits<value_type>::max());
    fail(bad, Fixture::minimum_norm, Status::non_finite_result, "finite position subtraction overflow rolls back");
    bad = fixture;
    bad.state.v_n.set(0U, std::numeric_limits<value_type>::quiet_NaN());
    fail(bad, Fixture::minimum_norm, Status::non_finite_input, "downstream injection validation rolls back");
    bad = fixture;
    bad.V.set(0U, 0U, std::numeric_limits<value_type>::infinity());
    fail(bad, Fixture::minimum_norm, Status::non_finite_input, "non-finite measurement covariance rolls back");
    if constexpr (measurement_size > 1U)
    {
        bad = fixture;
        bad.V.set(0U, 1U, value_type{0.25});
        fail(bad, Fixture::minimum_norm, Status::domain_error, "asymmetric V is not silently repaired");
    }
    for (value_type variance : {value_type{0}, value_type{-1}})
    {
        bad = fixture;
        bad.V.set(measurement_size - 1U, measurement_size - 1U, variance);
        fail(bad, Fixture::minimum_norm, Status::domain_error, "nonpositive measurement variance rolls back");
    }
    bad = fixture;
    bad.P.set(1U, 0U, value_type{0.25});
    fail(bad, Fixture::minimum_norm, Status::domain_error, "asymmetric prior covariance rolls back");
    if constexpr (measurement_size > 1U)
    {
        bad = fixture;
        bad.P.set(0U, 1U, value_type{4});
        bad.P.set(1U, 0U, value_type{4});
        // Invalid prior violates PSD precondition and gives an indefinite S here.
        fail(bad, Fixture::minimum_norm, Status::not_positive_definite, "innovation Cholesky failure propagates");
    }
    fail(fixture, value_type{0}, Status::domain_error, "invalid quaternion norm bound propagates");
    fail(fixture, std::numeric_limits<value_type>::quiet_NaN(), Status::non_finite_input,
         "non-finite quaternion norm bound propagates");
}

// Independent scalar position/velocity Kalman filter for each navigation axis.
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
        long double const S = pp + variance;
        long double const kp = pp / S;
        long double const kv = pv / S;
        long double const residual = z - p;
        p += kp * residual;
        v += kv * residual;
        vv -= kv * pv;
        pv *= 1.0L - kp;
        pp *= 1.0L - kp;
    }
};

template <typename Fixture>
void test_position_sequence(TestContext & test, std::string_view profile, typename Fixture::value_type tolerance,
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
    // Zero process noise; attitude and bias are known exactly in this scenario.
    // The remaining 6D system reduces to three independent 2D Kalman filters.
    std::array<long double, 3U> const truth_velocity =
        moving ? std::array<long double, 3U>{1.0L, -0.5L, 0.25L} : std::array<long double, 3U>{};
    std::array<long double, 3U> const initial_position{2.0L, -1.0L, 3.0L};
    std::array<long double, 3U> const velocity_error{0.5L, -0.25L, 0.125L};
    std::array<PositionVelocityReference, 3U> reference{};
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
    }
    constexpr std::size_t steps = 80U;
    for (std::size_t step = 1U; step <= steps; ++step)
    {
        for (auto & axis_reference : reference)
        {
            axis_reference.predict(dt);
        }
        typename Fixture::measurement_type z;
        for (std::size_t axis = 0U; axis < measurement_size; ++axis)
        {
            std::size_t const nav_axis = Fixture::axis_offset + axis;
            z.set(axis, static_cast<value_type>(truth_velocity[nav_axis] * static_cast<long double>(step) * dt));
            reference[nav_axis].correct(z(axis), V(axis, axis));
        }
        Status const predict_status = formal_eskf::try_predict(state, P, imu, dt, parameters, state, P);
        Status const correct_status = Fixture::correct(state, P, z, V, Fixture::minimum_norm, state, P);
        test.expect(predict_status == Status::success && correct_status == Status::success, profile,
                    "repeated in-place predict and position correct succeed");
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            auto const & expected = reference[axis];
            test.expect(near(state.p_n(axis), static_cast<value_type>(expected.p), tolerance) &&
                            near(state.v_n(axis), static_cast<value_type>(expected.v), tolerance) &&
                            near(P(axis, axis), static_cast<value_type>(expected.pp), tolerance) &&
                            near(P(axis, 3U + axis), static_cast<value_type>(expected.pv), tolerance) &&
                            near(P(3U + axis, 3U + axis), static_cast<value_type>(expected.vv), tolerance),
                        profile, "each predict/correct step matches independent position-velocity filter");
        }
        test.expect(formal_eskf::linalg::all_finite(P) && formal_eskf::linalg::is_symmetric(P, value_type{0}), profile,
                    "sequence covariance remains finite and symmetric");
    }
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        if (!Fixture::observes_axis(axis))
        {
            test.expect(near(P(3U + axis, 3U + axis), value_type{1}, tolerance), profile,
                        "unobserved independent velocity uncertainty is not reduced");
            continue;
        }
        auto const truth_position =
            static_cast<value_type>(truth_velocity[axis] * static_cast<long double>(steps) * dt);
        auto const truth_speed = static_cast<value_type>(truth_velocity[axis]);
        test.expect(near(state.p_n(axis), truth_position, static_cast<value_type>(0.02)) &&
                        near(state.v_n(axis), truth_speed, static_cast<value_type>(0.02)),
                    profile, "position observations reduce both initial position and velocity errors");
    }
}

template <typename Fixture>
void run_position_tests(TestContext & test, std::string_view profile, typename Fixture::value_type tolerance)
{
    test_position_jacobian<Fixture>(test, profile, tolerance);
    test_position_analytic<Fixture>(test, profile, tolerance);
    test_position_delegation<Fixture>(test, profile);
    test_position_failures<Fixture>(test, profile);
    test_position_sequence<Fixture>(test, profile, tolerance, false);
    test_position_sequence<Fixture>(test, profile, tolerance, true);
}

} /* end namespace */

int main()
{
    TestContext test;
    Eigen::internal::set_is_malloc_allowed(false);
    run_position_tests<HorizontalPositionFixture<formal_eskf::linalg::EigenBackend<float>>>(
        test, "INS horizontal position binary32", 8.0e-6F);
    run_position_tests<HorizontalPositionFixture<formal_eskf::linalg::EigenBackend<double>>>(
        test, "INS horizontal position binary64", 2.0e-12);
    run_position_tests<VerticalPositionFixture<formal_eskf::linalg::EigenBackend<float>>>(
        test, "INS vertical position binary32", 8.0e-6F);
    run_position_tests<VerticalPositionFixture<formal_eskf::linalg::EigenBackend<double>>>(
        test, "INS vertical position binary64", 2.0e-12);
    if (test.failures() != 0)
    {
        std::cerr << test.failures() << " ESKF position test(s) failed\n";
        return 1;
    }
    std::cout << "All ESKF position tests passed\n";
    return 0;
}
