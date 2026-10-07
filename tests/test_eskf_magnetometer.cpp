/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <string_view>

// Check that the measurement header includes its own dependencies.
#include <formal_eskf/eskf/magnetometer.hpp>
#include "test_backend.hpp"

#include "test_support.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::Status;
using formal_eskf::try_correct_magnetometer;
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
    bool result = same_bits(a.q_nb.coefficients(), b.q_nb.coefficients());
    if constexpr (requires { a.p_n; })
    {
        result = result && same_bits(a.p_n, b.p_n) && same_bits(a.v_n, b.v_n) && same_bits(a.b_a, b.b_a) &&
                 same_bits(a.b_g, b.b_g);
    }
    return result;
}

template <typename Linalg, typename Configuration> struct MagnetometerFixture
{
    using value_type = typename Linalg::value_type;
    using state_type = typename Configuration::template NominalState<Linalg>;
    using error_type = typename Configuration::template ErrorState<Linalg>;
    static constexpr std::size_t size = Configuration::error_state_dimension;
    static constexpr std::size_t attitude_offset = size == 3U ? 0U : 6U;
    static constexpr value_type minimum_norm = static_cast<value_type>(1.0e-6);
    using vector_type = formal_eskf::linalg::Matrix<Linalg, 3U, 1U>;
    using matrix3_type = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    using covariance_type = formal_eskf::linalg::Matrix<Linalg, size, size>;

    state_type state{};
    covariance_type P = covariance_type::identity();
    vector_type m_n = vector_type::from_row_major({value_type{2}, value_type{1}, value_type{-3}});
    vector_type m_b = vector_type::from_row_major({value_type{2.125}, value_type{1}, value_type{-3}});
    matrix3_type V = matrix3_type::identity();

    Status correct(state_type & output, covariance_type & P_output,
                   formal_eskf::configuration::Ins::BiasUpdate const & bias_update = {}) const
    {
        if constexpr (std::is_same_v<Configuration, formal_eskf::configuration::Ins>)
        {
            return try_correct_magnetometer(state, P, m_b, m_n, V, minimum_norm, output, P_output, bias_update);
        }
        else
        {
            return try_correct_magnetometer(state, P, m_b, m_n, V, minimum_norm, output, P_output);
        }
    }

    static void perturb(error_type & error, std::size_t column, value_type amount)
    {
        if constexpr (size == 3U)
        {
            error.delta_theta_b.set(column, amount);
        }
        else
        {
            std::array<vector_type *, 5U> blocks{&error.delta_p_n, &error.delta_v_n, &error.delta_theta_b,
                                                 &error.delta_b_a, &error.delta_b_g};
            blocks[column / 3U]->set(column % 3U, amount);
        }
    }
};

template <typename Linalg, typename Configuration>
void test_magnetometer_jacobian(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using Fixture = MagnetometerFixture<Linalg, Configuration>;
    Fixture fixture;
    static_assert(noexcept(formal_eskf::magnetometer_jacobian(fixture.state, fixture.m_n)));
    static_assert(noexcept(try_correct_magnetometer(fixture.state, fixture.P, fixture.m_b, fixture.m_n, fixture.V,
                                                    Fixture::minimum_norm, fixture.state, fixture.P)));
    test.expect(formal_eskf::so3::UnitQuaternion<Linalg>::try_from_coefficients(
                    value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5}, Fixture::minimum_norm,
                    fixture.state.q_nb) == Status::success,
                profile, "cyclic rotation is valid");
    // R rotates (x,y,z) to (z,x,y), so R^T*m_n = (1,-3,2).
    // This oracle deliberately does not call production inverse_rotate or hat.
    auto const H = formal_eskf::magnetometer_jacobian(fixture.state, fixture.m_n);
    constexpr int expected[3][3]{{0, -2, -3}, {2, 0, -1}, {3, 1, 0}};
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < Fixture::size; ++column)
        {
            bool const attitude = column >= Fixture::attitude_offset && column < Fixture::attitude_offset + 3U;
            value_type const coefficient =
                attitude ? static_cast<value_type>(expected[row][column - Fixture::attitude_offset]) : value_type{0};
            test.expect(H(row, column) == coefficient, profile,
                        "Jacobian has correct rotation direction, sign, scale and state columns");
        }
    }
    test.expect(formal_eskf::so3::UnitQuaternion<Linalg>::try_from_coefficients(
                    value_type{2}, value_type{-1}, value_type{3}, value_type{4}, Fixture::minimum_norm,
                    fixture.state.q_nb) == Status::success,
                profile, "tilted attitude is valid");
    auto const tilted_H = formal_eskf::magnetometer_jacobian(fixture.state, fixture.m_n);
    constexpr bool binary32 = std::numeric_limits<value_type>::digits == 24;
    constexpr value_type step = static_cast<value_type>(binary32 ? 0.01 : 1.0e-5);
    constexpr value_type tolerance = static_cast<value_type>(binary32 ? 2.0e-4 : 2.0e-9);
    for (std::size_t column = 0U; column < Fixture::size; ++column)
    {
        typename Fixture::error_type positive;
        typename Fixture::error_type negative;
        Fixture::perturb(positive, column, step);
        Fixture::perturb(negative, column, -step);
        typename Fixture::state_type plus;
        typename Fixture::state_type minus;
        test.expect(formal_eskf::try_inject_nominal(fixture.state, positive, Fixture::minimum_norm, plus) ==
                            Status::success &&
                        formal_eskf::try_inject_nominal(fixture.state, negative, Fixture::minimum_norm, minus) ==
                            Status::success,
                    profile, "finite-difference injections succeed");
        auto const h_plus = formal_eskf::so3::inverse_rotate(plus.q_nb, fixture.m_n);
        auto const h_minus = formal_eskf::so3::inverse_rotate(minus.q_nb, fixture.m_n);
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            auto const derivative = (h_plus(row) - h_minus(row)) / (value_type{2} * step);
            test.expect(near(tilted_H(row, column), derivative, tolerance), profile,
                        "Jacobian differentiates the right-injected observation, not the residual");
        }
    }
}

template <typename Linalg, typename Configuration>
void test_magnetometer_analytic(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    using Fixture = MagnetometerFixture<Linalg, Configuration>;
    Fixture fixture;
    fixture.m_n = Fixture::vector_type::from_row_major({value_type{2}, value_type{0}, value_type{0}});
    fixture.m_b = fixture.m_n;
    typename Fixture::state_type output;
    typename Fixture::covariance_type P_output;
    test.expect(try_correct_magnetometer(fixture.state, fixture.P, fixture.m_b, fixture.m_n, fixture.V,
                                         Fixture::minimum_norm, output, P_output) == Status::success,
                profile, "zero residual succeeds");
    test.expect(same_state(output, fixture.state), profile, "zero residual leaves nominal state unchanged");
    // H^T H = diag(0,4,4). Protection retains only the heading gain row.
    for (std::size_t row = 0U; row < Fixture::size; ++row)
    {
        value_type const variance =
            (!ESKF_MAG_TILT && row == Fixture::attitude_offset + 1U) || row == Fixture::attitude_offset + 2U
                ? value_type{1} / value_type{5}
                : value_type{1};
        for (std::size_t column = 0U; column < Fixture::size; ++column)
        {
            test.expect(near(P_output(row, column), row == column ? variance : value_type{0}, tolerance), profile,
                        "zero-residual covariance matches the selected magnetic gain policy");
        }
    }
    fixture.m_b.set(2U, value_type{0.125});
    test.expect(try_correct_magnetometer(fixture.state, fixture.P, fixture.m_b, fixture.m_n, fixture.V,
                                         Fixture::minimum_norm, output, P_output) == Status::success,
                profile, "nonzero magnetic residual succeeds");
    constexpr long double angle = 0.05L;
#if !ESKF_MAG_TILT
    // K_theta(1,2)=2/5 gives delta_theta=(0,0.05,0). Independent Exp and
    // reset oracle: no production solve, Joseph or SO(3) Jacobian is used here.
    test.expect(near(output.q_nb.q0(), static_cast<value_type>(std::cos(angle / 2)), tolerance) &&
                    near(output.q_nb.q2(), static_cast<value_type>(std::sin(angle / 2)), tolerance) &&
                    output.q_nb.q1() == value_type{0} && output.q_nb.q3() == value_type{0},
                profile, "full model corrects pitch with the expected sign and magnitude");
#if ESKF_RESET_APPROX
    constexpr long double diagonal = 1;
    constexpr long double off_diagonal = angle / 2;
#else
    long double const diagonal = std::sin(angle) / angle;
    long double const off_diagonal = (1 - std::cos(angle)) / angle;
#endif
    long double const G[3][3]{{diagonal, 0, -off_diagonal}, {0, 1, 0}, {off_diagonal, 0, diagonal}};
    constexpr long double variances[3]{1, 0.2L, 0.2L};
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            long double expected = 0;
            for (std::size_t k = 0U; k < 3U; ++k)
            {
                expected += G[row][k] * variances[k] * G[column][k];
            }
            test.expect(near(P_output(Fixture::attitude_offset + row, Fixture::attitude_offset + column),
                             static_cast<value_type>(expected), tolerance),
                        profile, "nonzero correction covariance matches independent reset oracle");
        }
    }

#else
    test.expect(same_state(output, fixture.state), profile, "protected mode rejects the same pitch-only correction");
    // K' retains only K_z,y=-2/5. With a Z residual the attitude stays put,
    // but heading variance still decreases. Pitch variance must NOT decrease.
    for (std::size_t row = 0U; row < Fixture::size; ++row)
    {
        value_type const variance =
            row == Fixture::attitude_offset + 2U ? value_type{1} / value_type{5} : value_type{1};
        for (std::size_t column = 0U; column < Fixture::size; ++column)
        {
            test.expect(near(P_output(row, column), row == column ? variance : value_type{0}, tolerance), profile,
                        "protected Joseph update preserves tilt variance instead of shrinking it post hoc");
        }
    }
    fixture.m_b.set(1U, value_type{0.125});
    test.expect(fixture.correct(output, P_output) == Status::success &&
                    near(output.q_nb.q0(), static_cast<value_type>(std::cos(angle / 2)), tolerance) &&
                    near(output.q_nb.q3(), static_cast<value_type>(-std::sin(angle / 2)), tolerance) &&
                    output.q_nb.q1() == value_type{0} && output.q_nb.q2() == value_type{0},
                profile, "protected mode retains the independently computed heading correction");
#endif
}

#if !ESKF_MAG_TILT
template <typename Linalg, typename Configuration>
void test_magnetometer_delegation(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    using Fixture = MagnetometerFixture<Linalg, Configuration>;
    Fixture fixture;
    test.expect(formal_eskf::so3::UnitQuaternion<Linalg>::try_from_coefficients(
                    value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5}, Fixture::minimum_norm,
                    fixture.state.q_nb) == Status::success,
                profile, "delegation prior is valid");
    auto const h = Fixture::vector_type::from_row_major({value_type{1}, value_type{-3}, value_type{2}});
    auto const residual =
        Fixture::vector_type::from_row_major({value_type{0.125}, value_type{-0.0625}, value_type{0.25}});
    fixture.m_b = h + residual;
    // Dense SPD P=I+u*u^T; the non-attitude gain entries must not be discarded.
    for (std::size_t row = 0U; row < Fixture::size; ++row)
    {
        for (std::size_t column = 0U; column < Fixture::size; ++column)
        {
            fixture.P(row, column) += static_cast<value_type>((row + 1U) * (column + 1U)) / value_type{256};
        }
    }
    fixture.V.set(0U, 1U, value_type{0.25});
    fixture.V.set(1U, 0U, value_type{0.25});
    formal_eskf::linalg::Matrix<Linalg, 3U, Fixture::size> H;
    H.template set_block<0U, Fixture::attitude_offset>(Fixture::matrix3_type::from_row_major(
        {value_type{0}, value_type{-2}, value_type{-3}, value_type{2}, value_type{0}, value_type{-1}, value_type{3},
         value_type{1}, value_type{0}}));
    typename Fixture::state_type expected_state;
    typename Fixture::covariance_type expected_P;
    test.expect(formal_eskf::try_correct(fixture.state, fixture.P, residual, H, fixture.V, Fixture::minimum_norm,
                                         expected_state, expected_P) == Status::success,
                profile, "generic full-covariance correction succeeds");
    if constexpr (Fixture::size == 15U)
    {
        test.expect(
            !same_bits(expected_state.p_n, fixture.state.p_n) && !same_bits(expected_state.v_n, fixture.state.v_n) &&
                !same_bits(expected_state.b_a, fixture.state.b_a) && !same_bits(expected_state.b_g, fixture.state.b_g),
            profile, "magnetic cross covariances correct position, velocity and both biases");
    }
    for (unsigned aliases = 0U; aliases < 4U; ++aliases)
    {
        auto input = fixture;
        typename Fixture::state_type separate_state;
        typename Fixture::covariance_type separate_P;
        auto & state_output = (aliases & 1U) != 0U ? input.state : separate_state;
        auto & P_output = (aliases & 2U) != 0U ? input.P : separate_P;
        test.expect(try_correct_magnetometer(input.state, input.P, input.m_b, input.m_n, input.V, Fixture::minimum_norm,
                                             state_output, P_output) == Status::success,
                    profile, "all output-alias combinations succeed");
        test.expect(same_state(state_output, expected_state) && same_bits(P_output, expected_P), profile,
                    "wrapper retains the complete generic correction, including reset");
        test.expect(((aliases & 1U) != 0U || same_state(input.state, fixture.state)) &&
                        ((aliases & 2U) != 0U || same_bits(input.P, fixture.P)) && same_bits(input.m_b, fixture.m_b) &&
                        same_bits(input.m_n, fixture.m_n) && same_bits(input.V, fixture.V),
                    profile, "non-output inputs remain unchanged");
    }
    auto scaled = fixture;
    scaled.m_b = fixture.m_b * value_type{2};
    scaled.m_n = fixture.m_n * value_type{2};
    scaled.V = fixture.V * value_type{4};
    test.expect(try_correct_magnetometer(scaled.state, scaled.P, scaled.m_b, scaled.m_n, scaled.V,
                                         Fixture::minimum_norm, scaled.state, scaled.P) == Status::success,
                profile, "consistent field and noise rescaling succeeds");
    test.expect(formal_eskf::so3::same_rotation(scaled.state.q_nb, expected_state.q_nb, tolerance) &&
                    formal_eskf::linalg::max_abs(scaled.P - expected_P) <= tolerance,
                profile, "physical field scaling with squared covariance scaling preserves the update");
    auto opposite = fixture;
    opposite.state.q_nb = -fixture.state.q_nb;
    test.expect(try_correct_magnetometer(opposite.state, opposite.P, opposite.m_b, opposite.m_n, opposite.V,
                                         Fixture::minimum_norm, opposite.state, opposite.P) == Status::success,
                profile, "opposite quaternion sign succeeds");
    test.expect(formal_eskf::so3::same_rotation(opposite.state.q_nb, expected_state.q_nb, tolerance) &&
                    formal_eskf::linalg::max_abs(opposite.P - expected_P) <= tolerance,
                profile, "q and -q yield the same physical update and covariance");

    if constexpr (Fixture::size == 15U)
    {
        for (unsigned permissions = 0U; permissions < 64U; ++permissions)
        {
            formal_eskf::configuration::Ins::BiasUpdate bias_update;
            for (unsigned axis = 0U; axis < 3U; ++axis)
            {
                bias_update.accelerometer[axis] = (permissions & (1U << axis)) != 0U;
                bias_update.gyroscope[axis] = (permissions & (1U << (3U + axis))) != 0U;
            }
            test.expect(formal_eskf::try_correct(fixture.state, fixture.P, residual, H, fixture.V,
                                                 Fixture::minimum_norm, expected_state, expected_P,
                                                 bias_update) == Status::success,
                        profile, "controlled generic correction succeeds");
            for (unsigned aliases = 0U; aliases < 4U; ++aliases)
            {
                auto input = fixture;
                typename Fixture::state_type separate_state;
                typename Fixture::covariance_type separate_P;
                auto & state_output = (aliases & 1U) != 0U ? input.state : separate_state;
                auto & P_output = (aliases & 2U) != 0U ? input.P : separate_P;
                test.expect(try_correct_magnetometer(input.state, input.P, input.m_b, input.m_n, input.V,
                                                     Fixture::minimum_norm, state_output, P_output,
                                                     bias_update) == Status::success &&
                                same_state(state_output, expected_state) && same_bits(P_output, expected_P),
                            profile, "all bias permissions and aliases match controlled generic correction");
                test.expect(((aliases & 1U) != 0U || same_state(input.state, fixture.state)) &&
                                ((aliases & 2U) != 0U || same_bits(input.P, fixture.P)) &&
                                same_bits(input.m_b, fixture.m_b) && same_bits(input.m_n, fixture.m_n) &&
                                same_bits(input.V, fixture.V),
                            profile, "controlled correction preserves non-output inputs");
                auto const state_before = state_output;
                auto const P_before = P_output;
                test.expect(try_correct_magnetometer(input.state, input.P, input.m_b, input.m_n, input.V, value_type{0},
                                                     state_output, P_output, bias_update) == Status::domain_error &&
                                same_state(state_output, state_before) && same_bits(P_output, P_before),
                            profile, "controlled correction preserves both outputs on downstream failure");
            }
        }
    }
}
#endif

template <typename Linalg, typename Configuration>
void test_magnetometer_failures(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using Fixture = MagnetometerFixture<Linalg, Configuration>;
    Fixture const fixture;
    auto const fail = [&](Fixture const & bad, value_type minimum_norm, Status expected, std::string_view description)
    {
        for (unsigned aliases = 0U; aliases < 4U; ++aliases)
        {
            auto input = bad;
            auto separate_state = fixture.state;
            auto separate_P = fixture.P * value_type{7};
            auto & state_output = (aliases & 1U) != 0U ? input.state : separate_state;
            auto & P_output = (aliases & 2U) != 0U ? input.P : separate_P;
            auto const state_before = state_output;
            auto const P_before = P_output;
            auto const status = try_correct_magnetometer(input.state, input.P, input.m_b, input.m_n, input.V,
                                                         minimum_norm, state_output, P_output);
            test.expect(status == expected && same_state(state_output, state_before) && same_bits(P_output, P_before),
                        profile, description);
            test.expect(same_state(input.state, bad.state) && same_bits(input.P, bad.P) &&
                            same_bits(input.m_b, bad.m_b) && same_bits(input.m_n, bad.m_n) && same_bits(input.V, bad.V),
                        profile, "failure preserves all inputs bit-for-bit");
        }
    };
    for (value_type invalid :
         {std::numeric_limits<value_type>::quiet_NaN(), std::numeric_limits<value_type>::infinity(),
          -std::numeric_limits<value_type>::infinity()})
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            auto bad = fixture;
            bad.m_b.set(axis, invalid);
            fail(bad, Fixture::minimum_norm, Status::non_finite_input, "invalid measured field rolls back");
            bad = fixture;
            bad.m_n.set(axis, invalid);
            fail(bad, Fixture::minimum_norm, Status::non_finite_input, "invalid reference field rolls back");
        }
    }
    auto bad = fixture;
    bad.m_b = Fixture::vector_type::zero();
    fail(bad, Fixture::minimum_norm, Status::domain_error, "zero measured field rolls back");
    bad = fixture;
    bad.m_n = Fixture::vector_type::zero();
    fail(bad, Fixture::minimum_norm, Status::domain_error, "zero reference field rolls back");
    bad = fixture;
    bad.m_n.set(0U, -std::numeric_limits<value_type>::max());
    bad.m_b.set(0U, std::numeric_limits<value_type>::max());
    fail(bad, Fixture::minimum_norm, Status::non_finite_result, "finite residual overflow rolls back");
    test.expect(formal_eskf::so3::UnitQuaternion<Linalg>::try_from_coefficients(
                    value_type{2}, value_type{0}, value_type{1}, value_type{0}, Fixture::minimum_norm,
                    bad.state.q_nb) == Status::success,
                profile, "overflow-test quaternion is valid");
    bad.m_n.set(2U, std::numeric_limits<value_type>::max());
    fail(bad, Fixture::minimum_norm, Status::non_finite_result, "finite rotation overflow rolls back");
    bad = fixture;
    bad.V.set(0U, 1U, value_type{0.25});
    fail(bad, Fixture::minimum_norm, Status::domain_error, "asymmetric V rolls back");
    bad = fixture;
    bad.V.set(0U, 0U, value_type{0});
    fail(bad, Fixture::minimum_norm, Status::domain_error, "nonpositive measurement variance rolls back");
    bad = fixture;
    bad.P.set(0U, 0U, std::numeric_limits<value_type>::quiet_NaN());
    fail(bad, Fixture::minimum_norm, Status::non_finite_input, "non-finite prior covariance rolls back");
    bad = fixture;
    bad.P.set(0U, 1U, value_type{0.25});
    fail(bad, Fixture::minimum_norm, Status::domain_error, "asymmetric prior covariance rolls back");
    bad = fixture;
    bad.m_n = Fixture::vector_type::from_row_major({value_type{2}, value_type{0}, value_type{0}});
    bad.P.set(Fixture::attitude_offset + 1U, Fixture::attitude_offset + 2U, value_type{4});
    bad.P.set(Fixture::attitude_offset + 2U, Fixture::attitude_offset + 1U, value_type{4});
    // Invalid prior violates the PSD precondition and makes S indefinite.
    fail(bad, Fixture::minimum_norm, Status::not_positive_definite, "failed innovation Cholesky propagates");
    fail(fixture, value_type{0}, Status::domain_error, "invalid quaternion norm bound rolls back");
    fail(fixture, value_type{2}, Status::domain_error, "excessive quaternion norm bound rolls back");
    fail(fixture, std::numeric_limits<value_type>::quiet_NaN(), Status::non_finite_input,
         "non-finite quaternion norm bound rolls back");
    if constexpr (Fixture::size == 15U)
    {
        bad = fixture;
        bad.state.b_g.set(1U, std::numeric_limits<value_type>::infinity());
        fail(bad, Fixture::minimum_norm, Status::non_finite_input, "downstream INS validation rolls back");
    }

#if ESKF_MAG_TILT
    // Projection can overflow individual coefficients even with a finite K and
    // a valid rank-one projector. Its internal scratch gain must not be changed.
    auto const down = Fixture::vector_type::from_row_major(
        {value_type{4} / value_type{5}, value_type{3} / value_type{5}, value_type{0}});
    formal_eskf::detail::MagneticTiltProjection<Linalg, Fixture::attitude_offset> const projection{
        down * formal_eskf::linalg::transpose(down)};
    formal_eskf::linalg::Matrix<Linalg, Fixture::size, 3U> gain;
    gain.set(Fixture::attitude_offset, 0U, std::numeric_limits<value_type>::max());
    gain.set(Fixture::attitude_offset + 1U, 0U, std::numeric_limits<value_type>::max());
    auto const gain_before = gain;
    test.expect(projection.apply(gain) == Status::non_finite_result && same_bits(gain, gain_before), profile,
                "finite projection overflow is reported without publishing a partial gain block");
#endif
}

#if ESKF_MAG_TILT
/** Independent long-double R(q), not production rotation/projection. */
template <typename Quaternion> std::array<std::array<long double, 3U>, 3U> reference_rotation(Quaternion const & q)
{
    long double const w = q.q0(), x = q.q1(), y = q.q2(), z = q.q3();
    return {{{w * w + x * x - y * y - z * z, 2 * (x * y - w * z), 2 * (x * z + w * y)},
             {2 * (x * y + w * z), w * w - x * x + y * y - z * z, 2 * (y * z - w * x)},
             {2 * (x * z - w * y), 2 * (y * z + w * x), w * w - x * x - y * y + z * z}}};
}

/** Independent small Cholesky, gain projection and dense Joseph arithmetic. */
template <typename Fixture>
void projected_correction_oracle(Fixture const & fixture,
                                 formal_eskf::configuration::Ins::BiasUpdate const & bias_update,
                                 typename Fixture::error_type & error, typename Fixture::covariance_type & covariance)
{
    using value_type = typename Fixture::value_type;
    constexpr auto size = Fixture::size;
    constexpr auto offset = Fixture::attitude_offset;
    auto const R = reference_rotation(fixture.state.q_nb);
    std::array<long double, 3U> h{}, residual{};
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        for (std::size_t k = 0U; k < 3U; ++k)
        {
            h[axis] += R[k][axis] * static_cast<long double>(fixture.m_n(k));
        }
        residual[axis] = static_cast<long double>(fixture.m_b(axis)) - h[axis];
    }
    std::array<std::array<long double, size>, 3U> H{};
    H[0][offset + 1U] = -h[2];
    H[0][offset + 2U] = h[1];
    H[1][offset] = h[2];
    H[1][offset + 2U] = -h[0];
    H[2][offset] = -h[1];
    H[2][offset + 1U] = h[0];
    std::array<std::array<long double, 3U>, size> PHt{}, K{};
    for (std::size_t row = 0U; row < size; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            for (std::size_t k = 0U; k < size; ++k)
            {
                PHt[row][column] += static_cast<long double>(fixture.P(row, k)) * H[column][k];
            }
        }
    }
    std::array<std::array<long double, 3U>, 3U> S{}, lower{};
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            S[row][column] = static_cast<long double>(fixture.V(row, column));
            for (std::size_t k = 0U; k < size; ++k)
            {
                S[row][column] += H[row][k] * PHt[k][column];
            }
        }
        for (std::size_t column = 0U; column <= row; ++column)
        {
            long double value = S[row][column];
            for (std::size_t k = 0U; k < column; ++k)
            {
                value -= lower[row][k] * lower[column][k];
            }
            lower[row][column] = row == column ? std::sqrt(value) : value / lower[column][column];
        }
    }
    for (std::size_t row = 0U; row < size; ++row)
    {
        std::array<long double, 3U> intermediate{};
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            long double value = PHt[row][axis];
            for (std::size_t k = 0U; k < axis; ++k)
            {
                value -= lower[axis][k] * intermediate[k];
            }
            intermediate[axis] = value / lower[axis][axis];
        }
        for (std::size_t reverse = 0U; reverse < 3U; ++reverse)
        {
            auto const axis = 2U - reverse;
            long double value = intermediate[axis];
            for (std::size_t k = axis + 1U; k < 3U; ++k)
            {
                value -= lower[k][axis] * K[row][k];
            }
            K[row][axis] = value / lower[axis][axis];
        }
    }
    for (std::size_t column = 0U; column < 3U; ++column)
    {
        long double vertical_gain = 0;
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            vertical_gain += R[2][axis] * K[offset + axis][column];
        }
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            K[offset + axis][column] = R[2][axis] * vertical_gain;
            if constexpr (size == 15U)
            {
                if (!bias_update.accelerometer[axis])
                {
                    K[9U + axis][column] = 0;
                }
                if (!bias_update.gyroscope[axis])
                {
                    K[12U + axis][column] = 0;
                }
            }
        }
    }
    std::array<std::array<long double, size>, size> A{}, AP{};
    for (std::size_t row = 0U; row < size; ++row)
    {
        long double correction = 0;
        for (std::size_t k = 0U; k < 3U; ++k)
        {
            correction += K[row][k] * residual[k];
        }
        Fixture::perturb(error, row, static_cast<value_type>(correction));
        for (std::size_t column = 0U; column < size; ++column)
        {
            A[row][column] = row == column ? 1 : 0;
            for (std::size_t k = 0U; k < 3U; ++k)
            {
                A[row][column] -= K[row][k] * H[k][column];
            }
        }
    }
    for (std::size_t row = 0U; row < size; ++row)
    {
        for (std::size_t column = 0U; column < size; ++column)
        {
            for (std::size_t k = 0U; k < size; ++k)
            {
                AP[row][column] += A[row][k] * static_cast<long double>(fixture.P(k, column));
            }
        }
    }
    for (std::size_t row = 0U; row < size; ++row)
    {
        for (std::size_t column = 0U; column <= row; ++column)
        {
            long double value = 0;
            for (std::size_t k = 0U; k < size; ++k)
            {
                value += AP[row][k] * A[column][k];
            }
            for (std::size_t j = 0U; j < 3U; ++j)
            {
                for (std::size_t k = 0U; k < 3U; ++k)
                {
                    value += K[row][j] * static_cast<long double>(fixture.V(j, k)) * K[column][k];
                }
            }
            covariance.set(row, column, static_cast<value_type>(value));
            covariance.set(column, row, static_cast<value_type>(value));
        }
    }
}

template <typename Linalg, typename Configuration>
void test_magnetometer_tilt(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using Fixture = MagnetometerFixture<Linalg, Configuration>;
    using value_type = typename Fixture::value_type;
    constexpr std::array<std::array<int, 4U>, 4U> rotations{{{1, 0, 0, 0}, {1, 1, 1, 1}, {1, 0, 1, 0}, {2, -1, 3, 4}}};
    for (auto const & coefficients : rotations)
    {
        Fixture fixture;
        test.expect(formal_eskf::so3::UnitQuaternion<Linalg>::try_from_coefficients(
                        static_cast<value_type>(coefficients[0]), static_cast<value_type>(coefficients[1]),
                        static_cast<value_type>(coefficients[2]), static_cast<value_type>(coefficients[3]),
                        Fixture::minimum_norm, fixture.state.q_nb) == Status::success,
                    profile, "tilt-protection prior includes tilted and Euler-singular attitudes");
        for (std::size_t row = 0U; row < Fixture::size; ++row)
        {
            for (std::size_t column = 0U; column < Fixture::size; ++column)
            {
                fixture.P(row, column) += static_cast<value_type>((row + 1U) * (column + 1U)) / value_type{256};
            }
        }
        fixture.V = Fixture::matrix3_type::from_row_major({value_type{2}, value_type{0.25}, value_type{-0.125},
                                                           value_type{0.25}, value_type{1}, value_type{0.125},
                                                           value_type{-0.125}, value_type{0.125}, value_type{1.5}});
        fixture.m_b = formal_eskf::so3::inverse_rotate(fixture.state.q_nb, fixture.m_n) +
                      Fixture::vector_type::from_row_major({value_type{0.25}, value_type{-0.125}, value_type{0.375}});
        auto const before_R = reference_rotation(fixture.state.q_nb);
        constexpr unsigned masks = Fixture::size == 15U ? 64U : 1U;
        for (unsigned permissions = 0U; permissions < masks; ++permissions)
        {
            formal_eskf::configuration::Ins::BiasUpdate bias_update;
            for (unsigned axis = 0U; axis < 3U; ++axis)
            {
                bias_update.accelerometer[axis] = (permissions & (1U << axis)) != 0U;
                bias_update.gyroscope[axis] = (permissions & (1U << (3U + axis))) != 0U;
            }
            typename Fixture::error_type error, reset_error;
            typename Fixture::covariance_type P_corrected, expected_P;
            typename Fixture::state_type expected_state;
            projected_correction_oracle(fixture, bias_update, error, P_corrected);
            test.expect(formal_eskf::try_inject_and_reset(fixture.state, P_corrected, error, Fixture::minimum_norm,
                                                          expected_state, expected_P, reset_error) == Status::success,
                        profile, "independent projected-gain/Joseph oracle feeds existing injection and reset");
            for (unsigned aliases = 0U; aliases < 4U; ++aliases)
            {
                auto input = fixture;
                typename Fixture::state_type separate_state;
                typename Fixture::covariance_type separate_P;
                auto & output = (aliases & 1U) != 0U ? input.state : separate_state;
                auto & P_output = (aliases & 2U) != 0U ? input.P : separate_P;
                test.expect(input.correct(output, P_output, bias_update) == Status::success, profile,
                            "tilt-protected correction supports all independent aliases and bias permissions");
                auto const after_R = reference_rotation(output.q_nb);
                for (std::size_t axis = 0U; axis < 3U; ++axis)
                {
                    test.expect(std::abs(after_R[2][axis] - before_R[2][axis]) <= static_cast<long double>(tolerance),
                                profile, "navigation Down in body is unchanged, including at 90 degree pitch");
                }
                test.expect(formal_eskf::linalg::max_abs(output.q_nb.coefficients() -
                                                         expected_state.q_nb.coefficients()) <= tolerance &&
                                formal_eskf::linalg::max_abs(P_output - expected_P) <= tolerance,
                            profile, "state and complete reset covariance match independent constrained-gain oracle");
                if constexpr (Fixture::size == 15U)
                {
                    test.expect(formal_eskf::linalg::max_abs(output.p_n - expected_state.p_n) <= tolerance &&
                                    formal_eskf::linalg::max_abs(output.v_n - expected_state.v_n) <= tolerance &&
                                    formal_eskf::linalg::max_abs(output.b_a - expected_state.b_a) <= tolerance &&
                                    formal_eskf::linalg::max_abs(output.b_g - expected_state.b_g) <= tolerance,
                                profile, "non-attitude updates remain intact except for explicit bias restrictions");
                }
                test.expect(((aliases & 1U) != 0U || same_state(input.state, fixture.state)) &&
                                ((aliases & 2U) != 0U || same_bits(input.P, fixture.P)) &&
                                same_bits(input.m_b, fixture.m_b) && same_bits(input.m_n, fixture.m_n) &&
                                same_bits(input.V, fixture.V),
                            profile, "protected correction leaves every non-output input unchanged");
            }
        }
        auto opposite = fixture;
        opposite.state.q_nb = -fixture.state.q_nb;
        typename Fixture::state_type positive_output, negative_output;
        typename Fixture::covariance_type positive_P, negative_P;
        test.expect(fixture.correct(positive_output, positive_P) == Status::success &&
                        opposite.correct(negative_output, negative_P) == Status::success &&
                        formal_eskf::so3::same_rotation(positive_output.q_nb, negative_output.q_nb, tolerance) &&
                        formal_eskf::linalg::max_abs(positive_P - negative_P) <= tolerance,
                    profile, "protected mode respects quaternion sign equivalence");
        auto scaled = fixture;
        scaled.m_b = fixture.m_b * value_type{2};
        scaled.m_n = fixture.m_n * value_type{2};
        scaled.V = fixture.V * value_type{4};
        test.expect(scaled.correct(negative_output, negative_P) == Status::success &&
                        formal_eskf::so3::same_rotation(positive_output.q_nb, negative_output.q_nb, tolerance) &&
                        formal_eskf::linalg::max_abs(positive_P - negative_P) <= tolerance,
                    profile, "protected update is invariant under consistent field and noise rescaling");
    }
}
#endif

template <typename Linalg, typename Configuration>
void run_magnetometer_tests(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    test_magnetometer_jacobian<Linalg, Configuration>(test, profile);
    test_magnetometer_analytic<Linalg, Configuration>(test, profile, tolerance);
    test_magnetometer_failures<Linalg, Configuration>(test, profile);
#if ESKF_MAG_TILT
    test_magnetometer_tilt<Linalg, Configuration>(test, profile, tolerance);
#else
    test_magnetometer_delegation<Linalg, Configuration>(test, profile, tolerance);
#endif
}

} /* end namespace */

int main()
{
    TestContext test;
    formal_eskf::test::configure_backend_test();
    using formal_eskf::configuration::Ahrs;
    using formal_eskf::configuration::Ins;
    using formal_eskf::test::Backend;
    run_magnetometer_tests<Backend<float>, Ahrs>(test, "AHRS magnetometer binary32", 8.0e-6F);
    run_magnetometer_tests<Backend<double>, Ahrs>(test, "AHRS magnetometer binary64", 2.0e-12);
    run_magnetometer_tests<Backend<float>, Ins>(test, "INS magnetometer binary32", 8.0e-6F);
    run_magnetometer_tests<Backend<double>, Ins>(test, "INS magnetometer binary64", 2.0e-12);
    if (test.failures() != 0)
    {
        std::cerr << test.failures() << " ESKF magnetometer test(s) failed\n";
        return 1;
    }
    std::cout << "All ESKF magnetometer tests passed\n";
    return 0;
}
