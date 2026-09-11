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
#include <formal_eskf/eskf/accelerometer.hpp>
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
    bool result = same_bits(a.q_nb.coefficients(), b.q_nb.coefficients());
    if constexpr (requires { a.p_n; })
    {
        result = result && same_bits(a.p_n, b.p_n) && same_bits(a.v_n, b.v_n) && same_bits(a.b_a, b.b_a) &&
                 same_bits(a.b_g, b.b_g);
    }
    return result;
}

template <typename Linalg, typename Configuration> struct AccelerometerFixture
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
    using jacobian_type = formal_eskf::linalg::Matrix<Linalg, 3U, size>;

    state_type state{};
    covariance_type P = covariance_type::identity();
    vector_type f_b = vector_type::from_row_major({value_type{0}, value_type{0}, value_type{-2}});
    vector_type omega_m{};
    vector_type g_n = vector_type::from_row_major({value_type{0}, value_type{0}, value_type{2}});
    matrix3_type V = matrix3_type::identity();

    [[nodiscard]] jacobian_type jacobian() const noexcept
    {
        if constexpr (size == 3U)
        {
            return formal_eskf::accelerometer_jacobian(state, g_n);
        }
        else
        {
            return formal_eskf::accelerometer_jacobian(state, omega_m, g_n);
        }
    }

    [[nodiscard]] Status correct(state_type & output, covariance_type & P_output,
                                 value_type norm = minimum_norm) const noexcept
    {
        if constexpr (size == 3U)
        {
            return formal_eskf::try_correct_accelerometer(state, P, f_b, g_n, V, norm, output, P_output);
        }
        else
        {
            return formal_eskf::try_correct_accelerometer(state, P, f_b, omega_m, g_n, V, norm, output, P_output);
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

// Independent scalar observation oracle: no production rotation, hat, cross,
// or measurement helper is used. Long double limits finite-difference noise.
template <typename State, typename Vector>
[[nodiscard]] std::array<long double, 3U> observation(State const & state, Vector const & g_n, Vector const & omega_m)
{
    long double const w = state.q_nb.q0();
    long double const x = state.q_nb.q1();
    long double const y = state.q_nb.q2();
    long double const z = state.q_nb.q3();
    long double const Rt[3][3]{{1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)},
                               {2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)},
                               {2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)}};
    std::array<long double, 3U> h{};
    std::array<long double, 3U> v{};
    std::array<long double, 3U> omega{};
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            h[row] -= Rt[row][column] * g_n(column);
            if constexpr (requires { state.v_n; })
            {
                v[row] += Rt[row][column] * state.v_n(column);
            }
        }
        if constexpr (requires { state.b_g; })
        {
            omega[row] = static_cast<long double>(omega_m(row)) - state.b_g(row);
            h[row] += state.b_a(row);
        }
    }
    h[0] += omega[1] * v[2] - omega[2] * v[1];
    h[1] += omega[2] * v[0] - omega[0] * v[2];
    h[2] += omega[0] * v[1] - omega[1] * v[0];
    return h;
}

template <typename Fixture> void moving_state(TestContext & test, Fixture & fixture, std::string_view profile)
{
    using value_type = typename Fixture::value_type;
    using quaternion_type = decltype(fixture.state.q_nb);
    test.expect(quaternion_type::try_from_coefficients(value_type{0.5}, value_type{0.5}, value_type{0.5},
                                                       value_type{0.5}, Fixture::minimum_norm,
                                                       fixture.state.q_nb) == Status::success,
                profile, "cyclic rotation is valid");
    if constexpr (Fixture::size == 15U)
    {
        fixture.state.v_n = Fixture::vector_type::from_row_major({value_type{3}, value_type{1}, value_type{-2}});
        fixture.state.b_a =
            Fixture::vector_type::from_row_major({value_type{0.25}, value_type{-0.5}, value_type{0.125}});
        fixture.state.b_g = Fixture::vector_type::from_row_major({value_type{1}, value_type{-1}, value_type{2}});
        fixture.omega_m = Fixture::vector_type::from_row_major({value_type{3}, value_type{2}, value_type{1}});
    }
}

template <typename Linalg, typename Configuration>
void test_accelerometer_jacobian(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using Fixture = AccelerometerFixture<Linalg, Configuration>;
    Fixture fixture;
    static_assert(noexcept(fixture.jacobian()));
    static_assert(noexcept(fixture.correct(fixture.state, fixture.P)));
    moving_state(test, fixture, profile);
    // R^T maps (x,y,z) to (y,z,x): g_b=(0,2,0), v_b=(1,-2,3), omega_b=(2,3,-1).
    typename Fixture::jacobian_type expected;
    if constexpr (Fixture::size == 3U)
    {
        expected = Fixture::matrix3_type::from_row_major({value_type{0}, value_type{0}, value_type{-2}, value_type{0},
                                                          value_type{0}, value_type{0}, value_type{2}, value_type{0},
                                                          value_type{0}});
    }
    else
    {
        constexpr int entries[3][15]{{0, 0, 0, 3, 0, 1, 9, 3, -3, 1, 0, 0, 0, -3, -2},
                                     {0, 0, 0, -2, -1, 0, -4, 1, 2, 0, 1, 0, 3, 0, -1},
                                     {0, 0, 0, 0, -3, 2, 8, 9, 4, 0, 0, 1, 2, 1, 0}};
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            for (std::size_t column = 0U; column < Fixture::size; ++column)
            {
                expected(row, column) = static_cast<value_type>(entries[row][column]);
            }
        }
    }
    test.expect(formal_eskf::linalg::max_abs(fixture.jacobian() - expected) == value_type{0}, profile,
                "independent Jacobian checks frame, cross order, bias signs and every state column");

    test.expect(formal_eskf::so3::UnitQuaternion<Linalg>::try_from_coefficients(
                    value_type{2}, value_type{-1}, value_type{3}, value_type{4}, Fixture::minimum_norm,
                    fixture.state.q_nb) == Status::success,
                profile, "general attitude is valid");
    fixture.g_n = Fixture::vector_type::from_row_major({value_type{0.25}, value_type{-0.5}, value_type{2}});
    auto const H = fixture.jacobian();
    constexpr bool binary32 = std::numeric_limits<value_type>::digits == 24;
    constexpr value_type step = static_cast<value_type>(binary32 ? 0.005 : 1.0e-5);
    constexpr value_type tolerance = static_cast<value_type>(binary32 ? 0.002 : 3.0e-9);
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
                    profile, "finite-difference state injections succeed");
        auto const h_plus = observation(plus, fixture.g_n, fixture.omega_m);
        auto const h_minus = observation(minus, fixture.g_n, fixture.omega_m);
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            auto const derivative = static_cast<value_type>((h_plus[row] - h_minus[row]) / (2 * step));
            test.expect(near(H(row, column), derivative, tolerance), profile,
                        "Jacobian differentiates the observation under right injection, not the residual");
        }
    }
}

template <typename Linalg, typename Configuration>
void test_accelerometer_analytic(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    using Fixture = AccelerometerFixture<Linalg, Configuration>;
    constexpr auto offset = Fixture::attitude_offset;
    Fixture fixture;
    typename Fixture::state_type output;
    typename Fixture::covariance_type P_output;
    test.expect(fixture.correct(output, P_output) == Status::success && same_state(output, fixture.state), profile,
                "level stationary negative-g measurement has zero residual");
    // P=I, V=I, h=(0,0,-2), H_theta=[[0,2,0],[-2,0,0],[0,0,0]].
    // INS additionally has H_ba=I. Build the conditional covariance without
    // production H, solve or Joseph helpers, including attitude/bias coupling.
    long double H[3][Fixture::size]{};
    H[0][offset + 1U] = 2;
    H[1][offset] = -2;
    constexpr bool ins = Fixture::size == 15U;
    if constexpr (ins)
    {
        H[0][9] = 1;
        H[1][10] = 1;
        H[2][11] = 1;
    }
    long double const S[3]{ins ? 6.0L : 5.0L, ins ? 6.0L : 5.0L, ins ? 2.0L : 1.0L};
    long double posterior[Fixture::size][Fixture::size]{};
    for (std::size_t row = 0U; row < Fixture::size; ++row)
    {
        for (std::size_t column = 0U; column < Fixture::size; ++column)
        {
            posterior[row][column] = row == column ? 1 : 0;
            for (std::size_t k = 0U; k < 3U; ++k)
            {
                posterior[row][column] -= H[k][row] * H[k][column] / S[k];
            }
            test.expect(near(P_output(row, column), static_cast<value_type>(posterior[row][column]), tolerance),
                        profile, "stationary covariance matches analytic oracle; gravity does not observe heading");
        }
    }
    fixture.f_b.set(0U, value_type{0.125});
    test.expect(fixture.correct(output, P_output) == Status::success, profile, "horizontal residual succeeds");
    long double const angle = 0.25L / S[0];
    test.expect(near(output.q_nb.q0(), static_cast<value_type>(std::cos(angle / 2)), tolerance) &&
                    near(output.q_nb.q2(), static_cast<value_type>(std::sin(angle / 2)), tolerance) &&
                    output.q_nb.q1() == value_type{0} && output.q_nb.q3() == value_type{0},
                profile, "attitude correction has the independently calculated sign and magnitude");
    if constexpr (ins)
    {
        test.expect(near(output.b_a(0U), value_type{1} / value_type{48}, tolerance) &&
                        output.b_a(1U) == value_type{0} && output.b_a(2U) == value_type{0} &&
                        same_bits(output.v_n, fixture.state.v_n) && same_bits(output.b_g, fixture.state.b_g) &&
                        same_bits(output.p_n, fixture.state.p_n),
                    profile, "stationary raw measurement corrects accelerometer bias without double subtraction");
    }
#if ESKF_RESET_APPROX
    constexpr long double diagonal = 1;
    long double const off_diagonal = angle / 2;
#else
    long double const diagonal = std::sin(angle) / angle;
    long double const off_diagonal = (1 - std::cos(angle)) / angle;
#endif
    long double G[Fixture::size][Fixture::size]{};
    for (std::size_t index = 0U; index < Fixture::size; ++index)
    {
        G[index][index] = 1;
    }
    G[offset][offset] = diagonal;
    G[offset + 2U][offset + 2U] = diagonal;
    G[offset][offset + 2U] = -off_diagonal;
    G[offset + 2U][offset] = off_diagonal;
    for (std::size_t row = 0U; row < Fixture::size; ++row)
    {
        for (std::size_t column = 0U; column < Fixture::size; ++column)
        {
            long double expected = 0;
            for (std::size_t i = 0U; i < Fixture::size; ++i)
            {
                for (std::size_t j = 0U; j < Fixture::size; ++j)
                {
                    expected += G[row][i] * posterior[i][j] * G[column][j];
                }
            }
            test.expect(near(P_output(row, column), static_cast<value_type>(expected), tolerance), profile,
                        "independent reset oracle includes all attitude/bias cross covariances");
        }
    }
}

template <typename Linalg, typename Configuration>
void test_accelerometer_delegation(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    using Fixture = AccelerometerFixture<Linalg, Configuration>;
    Fixture fixture;
    moving_state(test, fixture, profile);
    auto const h = observation(fixture.state, fixture.g_n, fixture.omega_m);
    auto const r = Fixture::vector_type::from_row_major({value_type{0.125}, value_type{-0.0625}, value_type{0.25}});
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        fixture.f_b.set(axis, static_cast<value_type>(h[axis]) + r(axis));
    }
    // Dense SPD prior ensures measurement updates are not restricted to tilt.
    for (std::size_t row = 0U; row < Fixture::size; ++row)
    {
        for (std::size_t column = 0U; column < Fixture::size; ++column)
        {
            fixture.P(row, column) += static_cast<value_type>((row + 1U) * (column + 1U)) / value_type{256};
        }
    }
    fixture.V.set(0U, 1U, value_type{0.25});
    fixture.V.set(1U, 0U, value_type{0.25});
    if constexpr (Fixture::size == 15U)
    {
        // Caller-owned V_a + hat(v_b)*V_g*hat(v_b)^T, with v_b=(1,-2,3)
        // and V_g=diag(1,2,4)/16. This is NOT process-noise intensity or P.
        auto const gyro_contribution =
            Fixture::matrix3_type::from_row_major({value_type{34}, value_type{8}, value_type{-6}, value_type{8},
                                                   value_type{13}, value_type{6}, value_type{-6}, value_type{6},
                                                   value_type{6}}) /
            value_type{16};
        fixture.V = fixture.V + gyro_contribution;
    }
    typename Fixture::state_type expected_state;
    typename Fixture::covariance_type expected_P;
    test.expect(formal_eskf::try_correct(fixture.state, fixture.P, r, fixture.jacobian(), fixture.V,
                                         Fixture::minimum_norm, expected_state, expected_P) == Status::success,
                profile, "generic correction with effective correlated V succeeds");
    if constexpr (Fixture::size == 15U)
    {
        test.expect(
            !same_bits(expected_state.p_n, fixture.state.p_n) && !same_bits(expected_state.v_n, fixture.state.v_n) &&
                !same_bits(expected_state.b_a, fixture.state.b_a) && !same_bits(expected_state.b_g, fixture.state.b_g),
            profile, "full INS correction retains velocity, position and both bias gains");
    }
    for (unsigned aliases = 0U; aliases < 4U; ++aliases)
    {
        auto input = fixture;
        typename Fixture::state_type separate_state;
        typename Fixture::covariance_type separate_P;
        auto & state_output = (aliases & 1U) != 0U ? input.state : separate_state;
        auto & P_output = (aliases & 2U) != 0U ? input.P : separate_P;
        test.expect(input.correct(state_output, P_output) == Status::success &&
                        same_state(state_output, expected_state) && same_bits(P_output, expected_P),
                    profile, "all output aliases match generic Joseph/injection/reset exactly");
        test.expect(((aliases & 1U) != 0U || same_state(input.state, fixture.state)) &&
                        ((aliases & 2U) != 0U || same_bits(input.P, fixture.P)) && same_bits(input.f_b, fixture.f_b) &&
                        same_bits(input.g_n, fixture.g_n) && same_bits(input.omega_m, fixture.omega_m) &&
                        same_bits(input.V, fixture.V),
                    profile, "non-output inputs remain unchanged");
    }
    auto opposite = fixture;
    opposite.state.q_nb = -fixture.state.q_nb;
    test.expect(opposite.correct(opposite.state, opposite.P) == Status::success &&
                    formal_eskf::so3::same_rotation(opposite.state.q_nb, expected_state.q_nb, tolerance) &&
                    formal_eskf::linalg::max_abs(opposite.P - expected_P) <= tolerance,
                profile, "q and -q give the same physical correction and covariance");
}

template <typename Linalg> void test_ins_motion_model(TestContext & test, typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    using Fixture = AccelerometerFixture<Linalg, formal_eskf::configuration::Ins>;
    constexpr std::string_view profile = "INS motion model";
    Fixture fixture;
    fixture.state.v_n.set(0U, value_type{3});
    fixture.omega_m.set(2U, value_type{1});
    fixture.f_b.set(1U, value_type{3});
    auto const before = fixture.state;
    test.expect(fixture.correct(fixture.state, fixture.P) == Status::success && same_state(fixture.state, before),
                profile, "steady body velocity with nonzero turn acceleration gives zero residual");
    fixture = Fixture{};
    fixture.state.b_a.set(2U, value_type{0.25});
    fixture.state.b_g.set(0U, value_type{0.5});
    fixture.omega_m = fixture.state.b_g;
    fixture.f_b.set(2U, value_type{-1.75});
    auto const biased = fixture.state;
    test.expect(fixture.correct(fixture.state, fixture.P) == Status::success && same_state(fixture.state, biased),
                profile, "raw stationary IMU includes both estimated biases exactly once");

    // With fixed biases and no velocity uncertainty/cross-covariance, INS at
    // rest reduces to AHRS. Bias H=I must not create information from zero P.
    AccelerometerFixture<Linalg, formal_eskf::configuration::Ahrs> ahrs;
    ahrs.f_b.set(0U, value_type{0.125});
    fixture = Fixture{};
    fixture.P = Fixture::covariance_type::zero();
    fixture.P.template set_block<6U, 6U>(ahrs.P);
    fixture.f_b = ahrs.f_b;
    test.expect(ahrs.correct(ahrs.state, ahrs.P) == Status::success &&
                    fixture.correct(fixture.state, fixture.P) == Status::success &&
                    formal_eskf::so3::same_rotation(ahrs.state.q_nb, fixture.state.q_nb, tolerance) &&
                    formal_eskf::linalg::max_abs(ahrs.P - fixture.P.template block<6U, 6U, 3U, 3U>()) <= tolerance,
                profile, "INS reduces to AHRS when cross compensation and uncertain extra states are absent");
}

template <typename Linalg, typename Configuration>
void test_accelerometer_failures(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using Fixture = AccelerometerFixture<Linalg, Configuration>;
    Fixture const fixture;
    auto const fail = [&](Fixture const & bad, value_type minimum_norm, Status expected, std::string_view description)
    {
        for (unsigned aliases = 0U; aliases < 4U; ++aliases)
        {
            auto input = bad;
            auto separate_state = fixture.state;
            typename Fixture::covariance_type separate_P = fixture.P * value_type{7};
            auto & output = (aliases & 1U) != 0U ? input.state : separate_state;
            auto & P_output = (aliases & 2U) != 0U ? input.P : separate_P;
            auto const state_before = output;
            auto const P_before = P_output;
            test.expect(input.correct(output, P_output, minimum_norm) == expected && same_state(output, state_before) &&
                            same_bits(P_output, P_before),
                        profile, description);
            test.expect(same_state(input.state, bad.state) && same_bits(input.P, bad.P) &&
                            same_bits(input.f_b, bad.f_b) && same_bits(input.omega_m, bad.omega_m) &&
                            same_bits(input.g_n, bad.g_n) && same_bits(input.V, bad.V),
                        profile, "failure preserves every input bit-for-bit");
        }
    };
    for (value_type invalid :
         {std::numeric_limits<value_type>::quiet_NaN(), std::numeric_limits<value_type>::infinity(),
          -std::numeric_limits<value_type>::infinity()})
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            auto bad = fixture;
            bad.f_b.set(axis, invalid);
            fail(bad, Fixture::minimum_norm, Status::non_finite_input, "non-finite specific force rolls back");
            bad = fixture;
            bad.g_n.set(axis, invalid);
            fail(bad, Fixture::minimum_norm, Status::non_finite_input, "non-finite gravity rolls back");
            if constexpr (Fixture::size == 15U)
            {
                for (unsigned field = 0U; field < 5U; ++field)
                {
                    bad = fixture;
                    std::array<typename Fixture::vector_type *, 5U> fields{&bad.omega_m, &bad.state.v_n, &bad.state.b_a,
                                                                           &bad.state.b_g, &bad.state.p_n};
                    fields[field]->set(axis, invalid);
                    fail(bad, Fixture::minimum_norm, Status::non_finite_input, "non-finite INS input rolls back");
                }
            }
        }
    }
    auto bad = fixture;
    bad.g_n = Fixture::vector_type::zero();
    fail(bad, Fixture::minimum_norm, Status::domain_error, "zero gravity reference rolls back");
    bad = fixture;
    bad.g_n.set(2U, std::numeric_limits<value_type>::max());
    bad.f_b.set(2U, std::numeric_limits<value_type>::max());
    fail(bad, Fixture::minimum_norm, Status::non_finite_result, "finite residual overflow rolls back");
    bad = fixture;
    bad.V.set(0U, 1U, value_type{0.25});
    fail(bad, Fixture::minimum_norm, Status::domain_error, "asymmetric effective V rolls back");
    bad = fixture;
    bad.V.set(0U, 0U, value_type{0});
    fail(bad, Fixture::minimum_norm, Status::domain_error, "zero measurement variance rolls back");
    bad = fixture;
    bad.V.set(1U, 1U, std::numeric_limits<value_type>::infinity());
    fail(bad, Fixture::minimum_norm, Status::non_finite_input, "non-finite effective V rolls back");
    bad = fixture;
    bad.P.set(0U, 1U, value_type{0.25});
    fail(bad, Fixture::minimum_norm, Status::domain_error, "asymmetric P rolls back");
    bad = fixture;
    bad.P.set(0U, 0U, value_type{-1});
    fail(bad, Fixture::minimum_norm, Status::domain_error, "negative prior variance rolls back");
    bad = fixture;
    bad.P.set(Fixture::attitude_offset, Fixture::attitude_offset + 1U, value_type{4});
    bad.P.set(Fixture::attitude_offset + 1U, Fixture::attitude_offset, value_type{4});
    fail(bad, Fixture::minimum_norm, Status::not_positive_definite, "indefinite innovation fails Cholesky atomically");
    fail(fixture, value_type{0}, Status::domain_error, "zero quaternion norm bound rolls back");
    fail(fixture, value_type{2}, Status::domain_error, "excessive quaternion norm bound rolls back");
    fail(fixture, std::numeric_limits<value_type>::quiet_NaN(), Status::non_finite_input,
         "non-finite quaternion norm bound rolls back");
    if constexpr (Fixture::size == 15U)
    {
        bad = fixture;
        bad.omega_m.set(0U, std::numeric_limits<value_type>::max());
        bad.state.b_g.set(0U, -std::numeric_limits<value_type>::max());
        fail(bad, Fixture::minimum_norm, Status::non_finite_result, "gyro-bias subtraction overflow rolls back");
        bad = fixture;
        bad.state.v_n.set(0U, std::numeric_limits<value_type>::max() / value_type{2});
        bad.omega_m.set(1U, value_type{4});
        fail(bad, Fixture::minimum_norm, Status::non_finite_result, "cross-product overflow rolls back");
        bad = fixture;
        bad.state.v_n.set(0U, std::numeric_limits<value_type>::max() / value_type{4});
        bad.omega_m.set(0U, value_type{8});
        fail(bad, Fixture::minimum_norm, Status::non_finite_result,
             "Jacobian overflow with finite observation rolls back");
    }
    auto zero_measurement = fixture;
    zero_measurement.f_b = Fixture::vector_type::zero();
    zero_measurement.P = Fixture::covariance_type::zero();
    test.expect(zero_measurement.correct(zero_measurement.state, zero_measurement.P) == Status::success &&
                    same_state(zero_measurement.state, fixture.state),
                profile, "unnormalized zero measurement is algebraically accepted; motion validity is caller-owned");
}

template <typename Linalg, typename Configuration>
void run_accelerometer_tests(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    test_accelerometer_jacobian<Linalg, Configuration>(test, profile);
    test_accelerometer_analytic<Linalg, Configuration>(test, profile, tolerance);
    test_accelerometer_delegation<Linalg, Configuration>(test, profile, tolerance);
    test_accelerometer_failures<Linalg, Configuration>(test, profile);
}

} /* end namespace */

int main()
{
    TestContext test;
    Eigen::internal::set_is_malloc_allowed(false);
    using formal_eskf::configuration::Ahrs;
    using formal_eskf::configuration::Ins;
    using formal_eskf::linalg::EigenBackend;
    run_accelerometer_tests<EigenBackend<float>, Ahrs>(test, "AHRS accelerometer binary32", 8.0e-6F);
    run_accelerometer_tests<EigenBackend<double>, Ahrs>(test, "AHRS accelerometer binary64", 2.0e-12);
    run_accelerometer_tests<EigenBackend<float>, Ins>(test, "INS accelerometer binary32", 8.0e-6F);
    run_accelerometer_tests<EigenBackend<double>, Ins>(test, "INS accelerometer binary64", 2.0e-12);
    test_ins_motion_model<EigenBackend<float>>(test, 8.0e-6F);
    test_ins_motion_model<EigenBackend<double>>(test, 2.0e-12);
    if (test.failures() != 0)
    {
        std::cerr << test.failures() << " ESKF accelerometer test(s) failed\n";
        return 1;
    }
    std::cout << "All ESKF accelerometer tests passed\n";
    return 0;
}
