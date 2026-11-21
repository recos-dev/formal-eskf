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
#include <cmath>
#include <type_traits>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <string_view>

// Check that the measurement header includes its own dependencies.
#include <formal_eskf/eskf/accelerometer.hpp>
#include "test_backend.hpp"

#include "test_support.hpp"
#include "test_cases.hpp"
#include "correction_reference.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::Status;
using formal_eskf::test::near;

template <typename Matrix> bool same_bits(Matrix const & a, Matrix const & b)
{
    using bytes_type = std::array<std::byte, sizeof(typename Matrix::value_type)>;
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            if (!(std::isnan(a(row, column)) && std::isnan(b(row, column))) &&
                std::bit_cast<bytes_type>(a(row, column)) != std::bit_cast<bytes_type>(b(row, column)))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename State> bool same_state(State const & a, State const & b)
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
    // Used through dependent fixture types in the test/oracle templates.
    // cppcheck-suppress unusedStructMember
    static constexpr std::size_t attitude_offset = size == 3U ? 0U : 6U;
    static constexpr value_type minimum_norm = static_cast<value_type>(1.0e-6);
    using vector_type = formal_eskf::linalg::Vector<Linalg, 3U>;
    using matrix3_type = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    using covariance_type = formal_eskf::linalg::Matrix<Linalg, size, size>;
    using jacobian_type = formal_eskf::linalg::Matrix<Linalg, 3U, size>;

    state_type state{};
    covariance_type P = covariance_type::identity();
    vector_type f_b = vector_type::from_row_major({value_type{0}, value_type{0}, value_type{-2}});
    vector_type g_n = vector_type::from_row_major({value_type{0}, value_type{0}, value_type{2}});
    matrix3_type V = matrix3_type::identity();

    jacobian_type jacobian() const { return formal_eskf::accelerometer_jacobian(state, g_n); }

    [[nodiscard]] Status correct(state_type & output, covariance_type & P_output, value_type norm = minimum_norm,
                                 formal_eskf::configuration::Ins::BiasUpdate const & bias_update = {}) const
    {
        if constexpr (size == 3U)
        {
            return formal_eskf::correct_accelerometer(state, P, f_b, g_n, V, norm, output, P_output);
        }
        else
        {
            return formal_eskf::correct_accelerometer(state, P, f_b, g_n, V, norm, output, P_output, bias_update);
        }
    }

    static void perturb(error_type & error, std::size_t column, value_type amount)
    {
        if constexpr (size == 3U)
        {
            error.delta_theta_b(column) = amount;
        }
        else
        {
            std::array<vector_type *, 5U> blocks{&error.delta_p_n, &error.delta_v_n, &error.delta_theta_b,
                                                 &error.delta_b_a, &error.delta_b_g};
            (*blocks[column / 3U])(column % 3U) = amount;
        }
    }
};

// Independent scalar observation oracle: no production rotation, hat, cross,
// or measurement helper is used. Long double limits finite-difference noise.
template <typename State, typename Vector>
std::array<long double, 3U> observation(State const & state, Vector const & g_n)
{
    long double const w = state.q_nb.q0();
    long double const x = state.q_nb.q1();
    long double const y = state.q_nb.q2();
    long double const z = state.q_nb.q3();
    long double const Rt[3][3]{{1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)},
                               {2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)},
                               {2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)}};
    std::array<long double, 3U> h{};
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            h[row] -= Rt[row][column] * g_n(column);
        }
    }
    return h;
}

template <typename Fixture> void moving_state(TestContext & test, Fixture & fixture, std::string_view profile)
{
    using value_type = typename Fixture::value_type;
    using quaternion_type = decltype(fixture.state.q_nb);
    test.expect(quaternion_type::from_coefficients(value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5},
                                                   Fixture::minimum_norm, fixture.state.q_nb) == Status::success,
                profile, "cyclic rotation is valid");
    if constexpr (Fixture::size == 15U)
    {
        fixture.state.v_n = Fixture::vector_type::from_row_major({value_type{3}, value_type{1}, value_type{-2}});
        fixture.state.b_a =
            Fixture::vector_type::from_row_major({value_type{0.25}, value_type{-0.5}, value_type{0.125}});
        fixture.state.b_g = Fixture::vector_type::from_row_major({value_type{1}, value_type{-1}, value_type{2}});
    }
}

template <typename Linalg, typename Configuration>
void test_accelerometer_jacobian(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using Fixture = AccelerometerFixture<Linalg, Configuration>;
    Fixture fixture;
    moving_state(test, fixture, profile);
    // R^T maps (x,y,z) to (y,z,x): g_b=(0,2,0). Only the attitude block is nonzero.
    typename Fixture::jacobian_type expected;
    expected.template set_block<0U, Fixture::attitude_offset>(Fixture::matrix3_type::from_row_major(
        {value_type{0}, value_type{0}, value_type{-2}, value_type{0}, value_type{0}, value_type{0}, value_type{2},
         value_type{0}, value_type{0}}));
    test.expect(formal_eskf::linalg::max_abs(fixture.jacobian() - expected) == value_type{0}, profile,
                "independent Jacobian checks frame, attitude sign and every zero state column");

    test.expect(formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(
                    value_type{2}, value_type{-1}, value_type{3}, value_type{4}, Fixture::minimum_norm,
                    fixture.state.q_nb) == Status::success,
                profile, "general attitude is valid");
    fixture.g_n = Fixture::vector_type::from_row_major({value_type{0.25}, value_type{-0.5}, value_type{2}});
    auto const H = fixture.jacobian();
    // Supplemental finite-difference regression; this step/tolerance pair is
    // not a certified general derivative/truncation bound.
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
        test.expect(
            formal_eskf::inject_nominal(fixture.state, positive, Fixture::minimum_norm, plus) == Status::success &&
                formal_eskf::inject_nominal(fixture.state, negative, Fixture::minimum_norm, minus) == Status::success,
            profile, "finite-difference state injections succeed");
        auto const h_plus = observation(plus, fixture.g_n);
        auto const h_minus = observation(minus, fixture.g_n);
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
    // Build the conditional covariance without production H, solve or Joseph
    // helpers. INS bias columns are zero: preprocessing is held fixed.
    long double H[3][Fixture::size]{};
    H[0][offset + 1U] = 2;
    H[1][offset] = -2;
    constexpr bool ins = Fixture::size == 15U;
    long double const S[3]{5, 5, 1};
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
    fixture.f_b(0U) = value_type{0.125};
    test.expect(fixture.correct(output, P_output) == Status::success, profile, "horizontal residual succeeds");
    long double const angle = 0.25L / S[0];
    test.expect(near(output.q_nb.q0(), static_cast<value_type>(std::cos(angle / 2)), tolerance) &&
                    near(output.q_nb.q2(), static_cast<value_type>(std::sin(angle / 2)), tolerance) &&
                    output.q_nb.q1() == value_type{0} && output.q_nb.q3() == value_type{0},
                profile, "attitude correction has the independently calculated sign and magnitude");
    if constexpr (ins)
    {
        test.expect(output.b_a(0U) == value_type{0} && output.b_a(1U) == value_type{0} &&
                        output.b_a(2U) == value_type{0} && same_bits(output.v_n, fixture.state.v_n) &&
                        same_bits(output.b_g, fixture.state.b_g) && same_bits(output.p_n, fixture.state.p_n),
                    profile, "diagonal prior and zero bias Jacobian leave bias unchanged");
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

template <typename Linalg, typename Configuration> auto coupled_fixture(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using Fixture = AccelerometerFixture<Linalg, Configuration>;
    Fixture fixture;
    moving_state(test, fixture, profile);
    auto const h = observation(fixture.state, fixture.g_n);
    auto const r = Fixture::vector_type::from_row_major({value_type{0.125}, value_type{-0.0625}, value_type{0.25}});
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        fixture.f_b(axis) = static_cast<value_type>(h[axis]) + r(axis);
        if constexpr (Fixture::size == 15U)
        {
            // Fixed preprocessing subtracts this prior bias exactly once.
            fixture.f_b(axis) = fixture.f_b(axis) + fixture.state.b_a(axis);
        }
    }
    // Dense SPD prior ensures measurement updates are not restricted to tilt.
    for (std::size_t row = 0U; row < Fixture::size; ++row)
    {
        for (std::size_t column = 0U; column < Fixture::size; ++column)
        {
            fixture.P(row, column) += static_cast<value_type>((row + 1U) * (column + 1U)) / value_type{256};
        }
    }
    fixture.V(0U, 1U) = value_type{0.25};
    fixture.V(1U, 0U) = value_type{0.25};
    fixture.V(0U, 2U) = value_type{0.125};
    fixture.V(2U, 0U) = value_type{0.125};
    fixture.V(1U, 2U) = value_type{-0.125};
    fixture.V(2U, 1U) = value_type{-0.125};
    return fixture;
}

template <typename Fixture> struct GravityReference
{
    using value_type = typename Fixture::value_type;
    using state_type = typename Fixture::state_type;
    using covariance_type = typename Fixture::covariance_type;
    static constexpr bool is_ins = Fixture::size == 15U;
    static constexpr std::size_t size = Fixture::size;
    static constexpr std::size_t attitude_offset = Fixture::attitude_offset;
    // Used through dependent fixture types in the test/oracle templates.
    // cppcheck-suppress unusedStructMember
    static constexpr std::size_t measurement_size = 3U;
    state_type state;
    covariance_type P;
    typename Fixture::jacobian_type H;
    typename Fixture::vector_type r;
    typename Fixture::matrix3_type V;

    explicit GravityReference(Fixture const & fixture) : state(fixture.state), P(fixture.P), V(fixture.V)
    {
        auto const h = observation(fixture.state, fixture.g_n);
        long double const skew[3][3]{{0, -h[2], h[1]}, {h[2], 0, -h[0]}, {-h[1], h[0], 0}};
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            long double measurement = fixture.f_b(row);
            if constexpr (is_ins)
            {
                measurement -= fixture.state.b_a(row);
            }
            r(row) = static_cast<value_type>(measurement - h[row]);
            for (std::size_t column = 0U; column < 3U; ++column)
            {
                H(row, attitude_offset + column) = static_cast<value_type>(skew[row][column]);
            }
        }
    }
};

template <typename Linalg, typename Configuration>
void test_accelerometer_delegation(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using Fixture = AccelerometerFixture<Linalg, Configuration>;
    auto fixture = coupled_fixture<Linalg, Configuration>(test, profile);
    GravityReference<Fixture> const independent_input(fixture);
    auto const independent =
        formal_eskf::test::reference::reference_correction(independent_input, ESKF_RESET_APPROX != 0);
    auto const r = independent_input.r;
    typename Fixture::state_type expected_state;
    typename Fixture::covariance_type expected_P;
    // Bitwise delegation/alias checks use the same selected arithmetic. Dense
    // numerical agreement is tested independently in test_codegen.cpp.
    auto const reference_status = [&]()
    {
        if constexpr (Fixture::size == 15U)
        {
            return formal_eskf::correct<Linalg, 3U, formal_eskf::detail::CorrectionModel::body_vector>(
                fixture.state, fixture.P, r, fixture.jacobian(), fixture.V, Fixture::minimum_norm, expected_state,
                expected_P);
        }
        else
        {
            return formal_eskf::correct(fixture.state, fixture.P, r, fixture.jacobian(), fixture.V,
                                        Fixture::minimum_norm, expected_state, expected_P);
        }
    }();
    test.expect(reference_status == Status::success, profile, "selected correction with correlated V succeeds");
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
        test.expect(formal_eskf::test::reference::matches_reference(independent_input, independent, state_output,
                                                                    P_output, tolerance),
                    profile, "whole gravity wrapper matches independently constructed h/H/r and full correction");
        test.expect(((aliases & 1U) != 0U || same_state(input.state, fixture.state)) &&
                        ((aliases & 2U) != 0U || same_bits(input.P, fixture.P)) && same_bits(input.f_b, fixture.f_b) &&
                        same_bits(input.g_n, fixture.g_n) && same_bits(input.V, fixture.V),
                    profile, "non-output inputs remain unchanged");
    }
    auto opposite = fixture;
    opposite.state.q_nb = -fixture.state.q_nb;
    test.expect(opposite.correct(opposite.state, opposite.P) == Status::success &&
                    formal_eskf::so3::same_rotation(opposite.state.q_nb, expected_state.q_nb, tolerance) &&
                    formal_eskf::linalg::max_abs(opposite.P - expected_P) <= tolerance,
                profile, "q and -q give the same physical correction and covariance");
}

template <typename Linalg>
void test_ins_gravity_model(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    using Fixture = AccelerometerFixture<Linalg, formal_eskf::configuration::Ins>;
    Fixture fixture;
    fixture.state.v_n(0U) = value_type{3};
    auto const before = fixture.state;
    test.expect(fixture.correct(fixture.state, fixture.P) == Status::success && same_state(fixture.state, before),
                profile, "gravity prediction does not depend on the prior velocity");
    fixture = Fixture{};
    fixture.state.b_a(0U) = value_type{0.25};
    fixture.state.b_a(2U) = value_type{0.25};
    fixture.state.b_g(0U) = value_type{0.5};
    fixture.f_b(0U) = value_type{0.25};
    fixture.f_b(2U) = value_type{-1.75};
    auto const biased = fixture.state;
    test.expect(fixture.correct(fixture.state, fixture.P) == Status::success && same_state(fixture.state, biased),
                profile, "raw stationary specific force subtracts the accelerometer bias exactly once");

    // With no attitude cross-covariance, INS attitude correction reduces to AHRS.
    AccelerometerFixture<Linalg, formal_eskf::configuration::Ahrs> ahrs;
    ahrs.f_b(0U) = value_type{0.125};
    fixture = Fixture{};
    fixture.P = Fixture::covariance_type::zero();
    fixture.P.template set_block<6U, 6U>(ahrs.P);
    fixture.f_b = ahrs.f_b;
    test.expect(ahrs.correct(ahrs.state, ahrs.P) == Status::success &&
                    fixture.correct(fixture.state, fixture.P) == Status::success &&
                    formal_eskf::so3::same_rotation(ahrs.state.q_nb, fixture.state.q_nb, tolerance) &&
                    formal_eskf::linalg::max_abs(ahrs.P - fixture.P.template block<6U, 6U, 3U, 3U>()) <= tolerance,
                profile, "INS attitude correction reduces to AHRS with a matching attitude prior");
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
            separate_state.q_nb = -separate_state.q_nb;
            if constexpr (Fixture::size == 15U)
            {
                separate_state.p_n(0U) = std::numeric_limits<value_type>::quiet_NaN();
                separate_state.v_n(1U) = -value_type{0};
            }
            typename Fixture::covariance_type separate_P = fixture.P * value_type{7};
            separate_P(0U, 1U) = -value_type{0};
            separate_P(1U, 0U) = std::numeric_limits<value_type>::quiet_NaN();
            auto & output = (aliases & 1U) != 0U ? input.state : separate_state;
            auto & P_output = (aliases & 2U) != 0U ? input.P : separate_P;
            auto const state_before = output;
            auto const P_before = P_output;
            test.expect(input.correct(output, P_output, minimum_norm) == expected && same_state(output, state_before) &&
                            same_bits(P_output, P_before),
                        profile, description);
            test.expect(same_state(input.state, bad.state) && same_bits(input.P, bad.P) &&
                            same_bits(input.f_b, bad.f_b) && same_bits(input.g_n, bad.g_n) && same_bits(input.V, bad.V),
                        profile, "failure preserves every logical input field");
        }
    };
    for (value_type invalid :
         {std::numeric_limits<value_type>::quiet_NaN(), std::numeric_limits<value_type>::infinity(),
          -std::numeric_limits<value_type>::infinity()})
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            auto bad = fixture;
            bad.f_b(axis) = invalid;
            fail(bad, Fixture::minimum_norm, Status::non_finite_input, "non-finite specific force rolls back");
            bad = fixture;
            bad.g_n(axis) = invalid;
            fail(bad, Fixture::minimum_norm, Status::non_finite_input, "non-finite gravity rolls back");
            if constexpr (Fixture::size == 15U)
            {
                for (unsigned field = 0U; field < 4U; ++field)
                {
                    bad = fixture;
                    std::array<typename Fixture::vector_type *, 4U> fields{&bad.state.v_n, &bad.state.b_a,
                                                                           &bad.state.b_g, &bad.state.p_n};
                    (*fields[field])(axis) = invalid;
                    fail(bad, Fixture::minimum_norm, Status::non_finite_input, "non-finite INS input rolls back");
                }
            }
        }
    }
    auto bad = fixture;
    bad.g_n = Fixture::vector_type::zero();
    fail(bad, Fixture::minimum_norm, Status::domain_error, "zero gravity reference rolls back");
    bad = fixture;
    bad.g_n(2U) = std::numeric_limits<value_type>::max();
    bad.f_b(2U) = std::numeric_limits<value_type>::max();
    fail(bad, Fixture::minimum_norm, Status::non_finite_result, "finite residual overflow rolls back");
    bad = fixture;
    test.expect(formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(value_type{2}, value_type{0}, value_type{0},
                                                                            value_type{1}, Fixture::minimum_norm,
                                                                            bad.state.q_nb) == Status::success,
                profile, "nonaxis rotation-overflow fixture is unit");
    bad.g_n(0U) = std::numeric_limits<value_type>::max();
    bad.g_n(1U) = std::numeric_limits<value_type>::max();
    fail(bad, Fixture::minimum_norm, Status::non_finite_result, "finite gravity rotation overflow rolls back");
    bad = fixture;
    bad.V(0U, 1U) = value_type{0.25};
    fail(bad, Fixture::minimum_norm, Status::domain_error, "asymmetric effective V rolls back");
    bad = fixture;
    bad.V(0U, 0U) = value_type{0};
    fail(bad, Fixture::minimum_norm, Status::domain_error, "zero measurement variance rolls back");
    bad = fixture;
    bad.V(1U, 1U) = std::numeric_limits<value_type>::infinity();
    fail(bad, Fixture::minimum_norm, Status::non_finite_input, "non-finite effective V rolls back");
    bad = fixture;
    bad.P(0U, 1U) = value_type{0.25};
    fail(bad, Fixture::minimum_norm, Status::domain_error, "asymmetric P rolls back");
    bad = fixture;
    bad.P(0U, 0U) = value_type{-1};
    fail(bad, Fixture::minimum_norm, Status::domain_error, "negative prior variance rolls back");
    bad = fixture;
    bad.P(Fixture::attitude_offset, Fixture::attitude_offset + 1U) = value_type{4};
    bad.P(Fixture::attitude_offset + 1U, Fixture::attitude_offset) = value_type{4};
    fail(bad, Fixture::minimum_norm, Status::not_positive_definite, "indefinite innovation fails Cholesky atomically");
    fail(fixture, value_type{0}, Status::domain_error, "zero quaternion norm bound rolls back");
    fail(fixture, value_type{2}, Status::domain_error, "excessive quaternion norm bound rolls back");
    fail(fixture, std::numeric_limits<value_type>::quiet_NaN(), Status::non_finite_input,
         "non-finite quaternion norm bound rolls back");
    if constexpr (Fixture::size == 15U)
    {
        bad = fixture;
        bad.f_b(0U) = std::numeric_limits<value_type>::max();
        bad.state.b_a(0U) = -std::numeric_limits<value_type>::max();
        fail(bad, Fixture::minimum_norm, Status::non_finite_result, "bias preprocessing overflow rolls back");
    }
    auto zero_measurement = fixture;
    zero_measurement.f_b = Fixture::vector_type::zero();
    zero_measurement.P = Fixture::covariance_type::zero();
    test.expect(zero_measurement.correct(zero_measurement.state, zero_measurement.P) == Status::success &&
                    same_state(zero_measurement.state, fixture.state),
                profile, "unnormalized zero measurement is algebraically accepted; motion validity is caller-owned");
}

template <typename Linalg, typename Configuration>
void test_fixed_preprocessing(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using Fixture = AccelerometerFixture<Linalg, Configuration>;
    auto fixture = coupled_fixture<Linalg, Configuration>(test, profile);
    auto original = fixture;
    test.expect(original.correct(original.state, original.P) == Status::success, profile,
                "fixed-preprocessing control correction succeeds");
    if constexpr (Fixture::size == 15U)
    {
        auto shifted = fixture;
        auto const offset = Fixture::vector_type::from_row_major({value_type{0.5}, value_type{-1}, value_type{2}});
        shifted.state.b_a = shifted.state.b_a + offset;
        shifted.f_b = shifted.f_b + offset;
        auto const H = shifted.jacobian();
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            for (std::size_t column = 9U; column < 15U; ++column)
            {
                test.expect(H(row, column) == value_type{0}, profile,
                            "fixed prior-bias preprocessing does not add bias Jacobian columns");
            }
        }
        test.expect(shifted.correct(shifted.state, shifted.P) == Status::success, profile,
                    "equal raw-force/prior-bias shifts preserve the processed measurement");
        auto expected = original.state;
        expected.b_a = expected.b_a + offset;
        // Encoded dyadic shifts preserve r exactly in this fixture. Nominal
        // bias addition can round differently after the changed offset.
        constexpr value_type rounding = value_type{8} * std::numeric_limits<value_type>::epsilon();
        test.expect(same_bits(shifted.P, original.P) &&
                        same_bits(shifted.state.q_nb.coefficients(), original.state.q_nb.coefficients()) &&
                        same_bits(shifted.state.p_n, original.state.p_n) &&
                        same_bits(shifted.state.v_n, original.state.v_n) &&
                        same_bits(shifted.state.b_g, original.state.b_g),
                    profile, "fixed measurement leaves gain, nonbias correction and covariance identical");
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            test.expect(near(shifted.state.b_a(axis), expected.b_a(axis), rounding), profile,
                        "changed prior bias receives the same estimated increment within two-addition rounding");
        }
        test_ins_gravity_model<Linalg>(test, profile, rounding);
    }
    else
    {
        auto scaled = fixture;
        scaled.f_b = scaled.f_b * value_type{2};
        scaled.g_n = scaled.g_n * value_type{2};
        scaled.V = scaled.V * value_type{4};
        test.expect(scaled.correct(scaled.state, scaled.P) == Status::success &&
                        same_state(scaled.state, original.state) && same_bits(scaled.P, original.P),
                    profile, "power-of-two unit scaling of force/gravity and covariance leaves AHRS update unchanged");
    }
}

template <typename Linalg> void test_bias_forwarding(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using Fixture = AccelerometerFixture<Linalg, formal_eskf::configuration::Ins>;
    auto fixture = coupled_fixture<Linalg, formal_eskf::configuration::Ins>(test, profile);
    GravityReference<Fixture> const independent_input(fixture);
    // Independent finite-angle regression; general acceptance envelope is open.
    constexpr value_type tolerance = static_cast<value_type>(std::is_same_v<value_type, float> ? 8e-6 : 2e-12);
    std::array<unsigned, 14U> permissions{0U, 63U};
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        permissions[2U + axis] = 1U << axis;
        permissions[8U + axis] = 63U ^ (1U << axis);
    }
    auto unrestricted = fixture;
    test.expect(unrestricted.correct(unrestricted.state, unrestricted.P) == Status::success, profile,
                "default gravity permissions permit correction");
    for (unsigned mask : permissions)
    {
        formal_eskf::configuration::Ins::BiasUpdate permission;
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            permission.accelerometer[axis] = (mask & (1U << axis)) != 0U;
            permission.gyroscope[axis] = (mask & (1U << (3U + axis))) != 0U;
        }
        auto const reference =
            formal_eskf::test::reference::reference_correction(independent_input, ESKF_RESET_APPROX != 0, permission);
        for (unsigned aliases = 0U; aliases < 4U; ++aliases)
        {
            auto input = fixture;
            typename Fixture::state_type separate_state;
            typename Fixture::covariance_type separate_P;
            auto & state = (aliases & 1U) != 0U ? input.state : separate_state;
            auto & P = (aliases & 2U) != 0U ? input.P : separate_P;
            test.expect(
                input.correct(state, P, Fixture::minimum_norm, permission) == Status::success &&
                    formal_eskf::test::reference::matches_reference(independent_input, reference, state, P, tolerance),
                profile, "gravity wrapper forwards each bias permission to both error and Joseph update");
            if (mask == 63U)
            {
                test.expect(same_state(state, unrestricted.state) && same_bits(P, unrestricted.P), profile,
                            "default/all-enabled and omitted gravity permissions agree exactly");
            }
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                test.expect((permission.accelerometer[axis] || state.b_a(axis) == fixture.state.b_a(axis)) &&
                                (permission.gyroscope[axis] || state.b_g(axis) == fixture.state.b_g(axis)),
                            profile, "disabled nominal bias coordinates remain unchanged");
            }
            test.expect(((aliases & 1U) != 0U || same_state(input.state, fixture.state)) &&
                            ((aliases & 2U) != 0U || same_bits(input.P, fixture.P)) &&
                            same_bits(input.f_b, fixture.f_b) && same_bits(input.g_n, fixture.g_n) &&
                            same_bits(input.V, fixture.V),
                        profile, "bias permissions preserve borrowed observation inputs");
            auto const saved = state;
            auto const saved_P = P;
            test.expect(input.correct(state, P, value_type{0}, permission) == Status::domain_error &&
                            same_state(state, saved) && same_bits(P, saved_P),
                        profile, "permissions do not bypass downstream checks or atomic failure");
        }
    }
}

template <typename Linalg, typename Configuration>
void test_domain_transaction(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    constexpr value_type tolerance = static_cast<value_type>(std::is_same_v<value_type, float> ? 8e-6 : 2e-12);
    test_accelerometer_analytic<Linalg, Configuration>(test, profile, tolerance);
    test_accelerometer_delegation<Linalg, Configuration>(test, profile, tolerance);
    test_accelerometer_failures<Linalg, Configuration>(test, profile);
}

template <typename Linalg, typename Configuration>
void test_boundary_ownership(TestContext & test, std::string_view profile)
{
    using Fixture = AccelerometerFixture<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    for (bool const cyclic : {false, true})
    {
        Fixture fixture;
        if (cyclic)
        {
            moving_state(test, fixture, profile);
        }
        auto const h = observation(fixture.state, fixture.g_n);
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            fixture.f_b(axis) = static_cast<value_type>(h[axis]);
            if constexpr (Fixture::size == 15U)
            {
                fixture.f_b(axis) += fixture.state.b_a(axis);
            }
        }
        fixture.V = Fixture::matrix3_type::identity() * value_type{12};
        auto const H = fixture.jacobian();
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            value_type null_component = value_type{0};
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                null_component += H(row, Fixture::attitude_offset + axis) * static_cast<value_type>(h[axis]);
            }
            test.expect(null_component == value_type{0}, profile,
                        "instantaneous observation derivative annihilates rotation along the gravity vector");
        }
        auto state = fixture.state;
        auto P = fixture.P;
        test.expect(fixture.correct(state, P) == Status::success && same_state(state, fixture.state), profile,
                    "zero residual at each current prior preserves nominal state without skipping covariance");
        std::size_t const gravity_axis = cyclic ? 1U : 2U;
        for (std::size_t row = 0U; row < Fixture::size; ++row)
        {
            bool const observed = row >= Fixture::attitude_offset && row < Fixture::attitude_offset + 3U &&
                                  row != Fixture::attitude_offset + gravity_axis;
            for (std::size_t column = 0U; column < Fixture::size; ++column)
            {
                value_type const expected =
                    row == column ? (observed ? value_type{0.75} : value_type{1}) : value_type{0};
                test.expect(
                    P(row, column) == expected, profile,
                    "exact posterior is relinearized at supplied attitude, retaining one field-direction variance");
            }
        }
    }
}

} /* end namespace */

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Case const cases[] = {
        {"TC-E-OBS-boundary-ownership", "INS.accelerometer.binary32",
         test_boundary_ownership<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ins>},
        {"TC-E-OBS-boundary-ownership", "INS.accelerometer.binary64",
         test_boundary_ownership<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ins>},
        {"TC-E-OBS-boundary-ownership", "AHRS.accelerometer.binary32",
         test_boundary_ownership<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ahrs>},
        {"TC-E-OBS-boundary-ownership", "AHRS.accelerometer.binary64",
         test_boundary_ownership<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ahrs>},
        {"TC-E-OBS-ACCEL-gravity-model", "INS.binary32",
         test_accelerometer_jacobian<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ins>},
        {"TC-E-OBS-ACCEL-gravity-model", "INS.binary64",
         test_accelerometer_jacobian<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ins>},
        {"TC-E-OBS-ACCEL-gravity-model", "AHRS.binary32",
         test_accelerometer_jacobian<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ahrs>},
        {"TC-E-OBS-ACCEL-gravity-model", "AHRS.binary64",
         test_accelerometer_jacobian<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ahrs>},
        {"TC-E-OBS-ACCEL-fixed-preprocessing", "INS.binary32",
         test_fixed_preprocessing<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ins>},
        {"TC-E-OBS-ACCEL-fixed-preprocessing", "INS.binary64",
         test_fixed_preprocessing<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ins>},
        {"TC-E-OBS-ACCEL-fixed-preprocessing", "AHRS.binary32",
         test_fixed_preprocessing<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ahrs>},
        {"TC-E-OBS-ACCEL-fixed-preprocessing", "AHRS.binary64",
         test_fixed_preprocessing<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ahrs>},
        {"TC-E-OBS-ACCEL-domain-transaction", "INS.binary32",
         test_domain_transaction<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ins>},
        {"TC-E-OBS-ACCEL-domain-transaction", "INS.binary64",
         test_domain_transaction<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ins>},
        {"TC-E-OBS-ACCEL-domain-transaction", "AHRS.binary32",
         test_domain_transaction<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ahrs>},
        {"TC-E-OBS-ACCEL-domain-transaction", "AHRS.binary64",
         test_domain_transaction<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ahrs>},
        {"TC-E-OBS-bias-forwarding", "INS.accelerometer.binary32",
         test_bias_forwarding<formal_eskf::test::Backend<float>>},
        {"TC-E-OBS-bias-forwarding", "INS.accelerometer.binary64",
         test_bias_forwarding<formal_eskf::test::Backend<double>>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
