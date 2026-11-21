/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <array>
#include <algorithm>
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
#include "test_cases.hpp"
#include "correction_reference.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::correct_magnetometer;
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

template <typename Linalg, typename Configuration> struct MagnetometerFixture
{
    using linalg_type = Linalg;
    using value_type = typename Linalg::value_type;
    using state_type = typename Configuration::template NominalState<Linalg>;
    using error_type = typename Configuration::template ErrorState<Linalg>;
    static constexpr std::size_t size = Configuration::error_state_dimension;
    // Used through dependent fixture types in the test/oracle templates.
    // cppcheck-suppress unusedStructMember
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
            return correct_magnetometer(state, P, m_b, m_n, V, minimum_norm, output, P_output, bias_update);
        }
        else
        {
            return correct_magnetometer(state, P, m_b, m_n, V, minimum_norm, output, P_output);
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

/** Independent long-double R(q), not production rotation/projection. */
template <typename Quaternion> std::array<std::array<long double, 3U>, 3U> reference_rotation(Quaternion const & q)
{
    long double const w = q.q0(), x = q.q1(), y = q.q2(), z = q.q3();
    return {{{w * w + x * x - y * y - z * z, 2 * (x * y - w * z), 2 * (x * z + w * y)},
             {2 * (x * y + w * z), w * w - x * x + y * y - z * z, 2 * (y * z - w * x)},
             {2 * (x * z - w * y), 2 * (y * z + w * x), w * w - x * x - y * y + z * z}}};
}

template <typename Fixture> struct MagneticReference
{
    using value_type = typename Fixture::value_type;
    using state_type = typename Fixture::state_type;
    using covariance_type = typename Fixture::covariance_type;
    // Used through dependent fixture types in the test/oracle templates.
    // cppcheck-suppress unusedStructMember
    static constexpr bool is_ins = Fixture::size == 15U;
    static constexpr std::size_t size = Fixture::size;
    static constexpr std::size_t attitude_offset = Fixture::attitude_offset;
    // Used through dependent fixture types in the test/oracle templates.
    // cppcheck-suppress unusedStructMember
    static constexpr std::size_t measurement_size = 3U;
    using linalg_type = typename Fixture::linalg_type;
    state_type state;
    covariance_type P;
    formal_eskf::linalg::Matrix<linalg_type, 3U, size> H;
    typename Fixture::vector_type r;
    typename Fixture::matrix3_type V;

    explicit MagneticReference(Fixture const & fixture) : state(fixture.state), P(fixture.P), V(fixture.V)
    {
        auto const R = reference_rotation(fixture.state.q_nb);
        std::array<long double, 3U> h{};
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            for (std::size_t column = 0U; column < 3U; ++column)
            {
                h[row] += R[column][row] * fixture.m_n(column);
            }
            r(row) = static_cast<value_type>(static_cast<long double>(fixture.m_b(row)) - h[row]);
        }
        long double const skew[3][3]{{0, -h[2], h[1]}, {h[2], 0, -h[0]}, {-h[1], h[0], 0}};
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            for (std::size_t column = 0U; column < 3U; ++column)
            {
                H(row, attitude_offset + column) = static_cast<value_type>(skew[row][column]);
            }
        }
    }
};

template <typename Linalg, typename Configuration>
void test_magnetometer_jacobian(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using Fixture = MagnetometerFixture<Linalg, Configuration>;
    Fixture fixture;
    test.expect(formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(
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
    test.expect(formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(
                    value_type{2}, value_type{-1}, value_type{3}, value_type{4}, Fixture::minimum_norm,
                    fixture.state.q_nb) == Status::success,
                profile, "tilted attitude is valid");
    auto const tilted_H = formal_eskf::magnetometer_jacobian(fixture.state, fixture.m_n);
    // Supplemental finite-difference regression; a general truncation/rounding
    // acceptance envelope remains unresolved.
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
        test.expect(
            formal_eskf::inject_nominal(fixture.state, positive, Fixture::minimum_norm, plus) == Status::success &&
                formal_eskf::inject_nominal(fixture.state, negative, Fixture::minimum_norm, minus) == Status::success,
            profile, "finite-difference injections succeed");
        auto const R_plus = reference_rotation(plus.q_nb);
        auto const R_minus = reference_rotation(minus.q_nb);
        std::array<long double, 3U> h_plus{}, h_minus{};
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                h_plus[row] += R_plus[axis][row] * fixture.m_n(axis);
                h_minus[row] += R_minus[axis][row] * fixture.m_n(axis);
            }
        }
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            auto const derivative = static_cast<value_type>((h_plus[row] - h_minus[row]) / (2 * step));
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
    test.expect(correct_magnetometer(fixture.state, fixture.P, fixture.m_b, fixture.m_n, fixture.V,
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
    fixture.m_b(2U) = value_type{0.125};
    test.expect(correct_magnetometer(fixture.state, fixture.P, fixture.m_b, fixture.m_n, fixture.V,
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
    fixture.m_b(1U) = value_type{0.125};
    test.expect(fixture.correct(output, P_output) == Status::success &&
                    near(output.q_nb.q0(), static_cast<value_type>(std::cos(angle / 2)), tolerance) &&
                    near(output.q_nb.q3(), static_cast<value_type>(-std::sin(angle / 2)), tolerance) &&
                    output.q_nb.q1() == value_type{0} && output.q_nb.q2() == value_type{0},
                profile, "protected mode retains the independently computed heading correction");
#endif
}

template <typename Fixture>
void expect_magnetic_failure(TestContext & test, std::string_view profile, Fixture const & bad,
                             typename Fixture::value_type minimum_norm, Status expected, std::string_view description)
{
    using value_type = typename Fixture::value_type;
    Fixture const fixture;
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
        auto separate_P = fixture.P * value_type{7};
        separate_P(0U, 1U) = -value_type{0};
        separate_P(1U, 0U) = std::numeric_limits<value_type>::quiet_NaN();
        auto & state_output = (aliases & 1U) != 0U ? input.state : separate_state;
        auto & P_output = (aliases & 2U) != 0U ? input.P : separate_P;
        auto const state_before = state_output;
        auto const P_before = P_output;
        auto const status = correct_magnetometer(input.state, input.P, input.m_b, input.m_n, input.V, minimum_norm,
                                                 state_output, P_output);
        test.expect(status == expected && same_state(state_output, state_before) && same_bits(P_output, P_before),
                    profile, description);
        test.expect(same_state(input.state, bad.state) && same_bits(input.P, bad.P) && same_bits(input.m_b, bad.m_b) &&
                        same_bits(input.m_n, bad.m_n) && same_bits(input.V, bad.V),
                    profile, "failure preserves every logical input field");
    }
}

template <typename Linalg, typename Configuration>
void test_failure_selection(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using Fixture = MagnetometerFixture<Linalg, Configuration>;
    Fixture const fixture;
    auto const fail = [&](Fixture const & bad, value_type minimum_norm, Status expected, std::string_view description)
    { expect_magnetic_failure(test, profile, bad, minimum_norm, expected, description); };
    auto bad = fixture;
    bad.V(0U, 1U) = value_type{0.25};
    fail(bad, Fixture::minimum_norm, Status::domain_error, "asymmetric V rolls back");
    bad = fixture;
    bad.V(0U, 0U) = value_type{0};
    fail(bad, Fixture::minimum_norm, Status::domain_error, "nonpositive measurement variance rolls back");
    bad = fixture;
    bad.P(0U, 0U) = std::numeric_limits<value_type>::quiet_NaN();
    fail(bad, Fixture::minimum_norm, Status::non_finite_input, "non-finite prior covariance rolls back");
    bad = fixture;
    bad.P(0U, 1U) = value_type{0.25};
    fail(bad, Fixture::minimum_norm, Status::domain_error, "asymmetric prior covariance rolls back");
    bad = fixture;
    bad.m_n = Fixture::vector_type::from_row_major({value_type{2}, value_type{0}, value_type{0}});
    bad.P(Fixture::attitude_offset + 1U, Fixture::attitude_offset + 2U) = value_type{4};
    bad.P(Fixture::attitude_offset + 2U, Fixture::attitude_offset + 1U) = value_type{4};
    // Invalid prior violates the PSD precondition and makes S indefinite.
    fail(bad, Fixture::minimum_norm, Status::not_positive_definite, "failed innovation Cholesky propagates");
    fail(fixture, value_type{0}, Status::domain_error, "invalid quaternion norm bound rolls back");
    fail(fixture, value_type{2}, Status::domain_error, "excessive quaternion norm bound rolls back");
    fail(fixture, std::numeric_limits<value_type>::quiet_NaN(), Status::non_finite_input,
         "non-finite quaternion norm bound rolls back");
    if constexpr (Fixture::size == 15U)
    {
        bad = fixture;
        bad.state.b_g(1U) = std::numeric_limits<value_type>::infinity();
        fail(bad, Fixture::minimum_norm, Status::non_finite_input, "downstream INS validation rolls back");
    }

#if ESKF_MAG_TILT
    typename Fixture::matrix3_type invalid_R;
    invalid_R(2U, 0U) = std::numeric_limits<value_type>::max();
    auto projector_output = Fixture::matrix3_type::identity();
    auto const projector_before = projector_output;
    // This nonrotation input is an internal guard probe, not a claim that a
    // finite unit public prior can produce it.
    test.expect(formal_eskf::detail::magnetic_tilt_projector(invalid_R, projector_output) ==
                        Status::non_finite_result &&
                    same_bits(projector_output, projector_before),
                profile, "internal projector overflow preserves its disjoint output");
    // Projection can overflow individual coefficients even with a finite K and
    // a valid rank-one projector. Its internal scratch gain must not be changed.
    auto const down = Fixture::vector_type::from_row_major(
        {value_type{4} / value_type{5}, value_type{3} / value_type{5}, value_type{0}});
    auto const projector = down * formal_eskf::linalg::transpose(down);
    formal_eskf::linalg::Matrix<Linalg, Fixture::size, 3U> gain;
    gain(Fixture::attitude_offset, 0U) = std::numeric_limits<value_type>::max();
    gain(Fixture::attitude_offset + 1U, 0U) = std::numeric_limits<value_type>::max();
    auto const gain_before = gain;
    test.expect(formal_eskf::detail::project_magnetic_gain<Fixture::attitude_offset>(projector, gain) ==
                        Status::non_finite_result &&
                    same_bits(gain, gain_before),
                profile, "finite projection overflow is reported without publishing a partial gain block");
#endif
}

#if ESKF_MAG_TILT
template <typename Linalg, typename Configuration>
void test_magnetometer_tilt(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using Fixture = MagnetometerFixture<Linalg, Configuration>;
    using value_type = typename Fixture::value_type;
    constexpr std::array<std::array<int, 4U>, 4U> rotations{{{1, 0, 0, 0}, {1, 1, 1, 1}, {1, 0, 1, 0}, {2, -1, 3, 4}}};
    for (auto const & coefficients : rotations)
    {
        Fixture fixture;
        test.expect(formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(
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
            MagneticReference<Fixture> const independent_input(fixture);
            auto const independent = formal_eskf::test::reference::reference_correction(
                independent_input, ESKF_RESET_APPROX != 0, bias_update, true);
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
                test.expect(formal_eskf::test::reference::matches_reference(independent_input, independent, output,
                                                                            P_output, tolerance),
                            profile,
                            "projected gain, Joseph, injection and reset match independent complete reference");
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

template <typename Linalg, typename Configuration> auto coupled_fixture(TestContext & test, std::string_view profile)
{
    using Fixture = MagnetometerFixture<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    Fixture fixture;
    test.expect(formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(
                    value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5}, Fixture::minimum_norm,
                    fixture.state.q_nb) == Status::success,
                profile, "cyclic tilted magnetic prior is valid");
    fixture.m_b = Fixture::vector_type::from_row_major({value_type{1.125}, value_type{-3.0625}, value_type{2.25}});
    fixture.V = Fixture::matrix3_type::from_row_major({value_type{2}, value_type{0.25}, value_type{-0.125},
                                                       value_type{0.25}, value_type{1}, value_type{0.125},
                                                       value_type{-0.125}, value_type{0.125}, value_type{1.5}});
    for (std::size_t row = 0U; row < Fixture::size; ++row)
    {
        for (std::size_t column = 0U; column < Fixture::size; ++column)
        {
            fixture.P(row, column) += static_cast<value_type>((row + 1U) * (column + 1U)) / value_type{256};
        }
    }
    return fixture;
}

template <typename Linalg, typename Configuration>
void test_strength_domain(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using Fixture = MagnetometerFixture<Linalg, Configuration>;
    Fixture const fixture;
    auto const fail = [&](Fixture const & bad, value_type minimum_norm, Status expected, std::string_view description)
    { expect_magnetic_failure(test, profile, bad, minimum_norm, expected, description); };
    for (value_type invalid :
         {std::numeric_limits<value_type>::quiet_NaN(), std::numeric_limits<value_type>::infinity(),
          -std::numeric_limits<value_type>::infinity()})
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            auto bad = fixture;
            bad.m_b(axis) = invalid;
            fail(bad, Fixture::minimum_norm, Status::non_finite_input, "invalid measured field rolls back");
            bad = fixture;
            bad.m_n(axis) = invalid;
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
    bad.m_n(0U) = -std::numeric_limits<value_type>::max();
    bad.m_b(0U) = std::numeric_limits<value_type>::max();
    fail(bad, Fixture::minimum_norm, Status::non_finite_result, "finite residual overflow rolls back");
    test.expect(formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(value_type{2}, value_type{0}, value_type{1},
                                                                            value_type{0}, Fixture::minimum_norm,
                                                                            bad.state.q_nb) == Status::success,
                profile, "overflow-test quaternion is valid");
    bad.m_n(2U) = std::numeric_limits<value_type>::max();
    fail(bad, Fixture::minimum_norm, Status::non_finite_result, "finite rotation overflow rolls back");

    for (bool const measured : {false, true})
    {
        bad = fixture;
        auto & vector = measured ? bad.m_b : bad.m_n;
        vector = Fixture::vector_type::from_row_major({-value_type{0}, value_type{0}, -value_type{0}});
        fail(bad, Fixture::minimum_norm, Status::domain_error, "signed zero vector remains an absent magnetic field");
    }
    for (value_type const magnitude :
         {std::numeric_limits<value_type>::denorm_min(), std::numeric_limits<value_type>::min(), value_type{2},
          std::numeric_limits<value_type>::max()})
    {
        for (std::size_t axis : {0U, 2U})
        {
            auto extreme = fixture;
            extreme.P = {};
            extreme.m_n = {};
            extreme.m_n(axis) = magnitude;
            extreme.m_b = extreme.m_n;
            auto output = extreme.state;
            auto P_output = fixture.P;
            test.expect(extreme.correct(output, P_output) == Status::success && same_state(output, extreme.state) &&
                            same_bits(P_output, extreme.P),
                        profile,
                        "finite nonzero tiny/vertical/large field is accepted without a squared-strength health gate");
        }
    }
    auto original = coupled_fixture<Linalg, Configuration>(test, profile);
    auto scaled = original;
    scaled.m_b = scaled.m_b * value_type{2};
    scaled.m_n = scaled.m_n * value_type{2};
    scaled.V = scaled.V * value_type{4};
    test.expect(original.correct(original.state, original.P) == Status::success &&
                    scaled.correct(scaled.state, scaled.P) == Status::success &&
                    same_state(scaled.state, original.state) && same_bits(scaled.P, original.P),
                profile, "power-of-two field units and squared covariance scaling leave the correction unchanged");
}

template <typename Linalg, typename Configuration> void test_wrapper(TestContext & test, std::string_view profile)
{
    using Fixture = MagnetometerFixture<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    constexpr value_type tolerance = static_cast<value_type>(std::is_same_v<value_type, float> ? 8e-6 : 2e-12);
    auto fixture = coupled_fixture<Linalg, Configuration>(test, profile);
    MagneticReference<Fixture> const input_reference(fixture);
    auto const reference = formal_eskf::test::reference::reference_correction(input_reference, ESKF_RESET_APPROX != 0,
                                                                              {}, ESKF_MAG_TILT != 0);
    for (unsigned aliases = 0U; aliases < 4U; ++aliases)
    {
        auto input = fixture;
        typename Fixture::state_type separate_state;
        typename Fixture::covariance_type separate_P;
        auto & state = (aliases & 1U) != 0U ? input.state : separate_state;
        auto & P = (aliases & 2U) != 0U ? input.P : separate_P;
        test.expect(input.correct(state, P) == Status::success && formal_eskf::test::reference::matches_reference(
                                                                      input_reference, reference, state, P, tolerance),
                    profile, "complete wrapper uses independent vector geometry, selected gain, Joseph and reset");
        test.expect(((aliases & 1U) != 0U || same_state(input.state, fixture.state)) &&
                        ((aliases & 2U) != 0U || same_bits(input.P, fixture.P)) && same_bits(input.m_b, fixture.m_b) &&
                        same_bits(input.m_n, fixture.m_n) && same_bits(input.V, fixture.V),
                    profile, "wrapper preserves all borrowed measurement and distinct prior objects");
    }
    auto opposite = fixture;
    opposite.state.q_nb = -opposite.state.q_nb;
    MagneticReference<Fixture> const opposite_input(opposite);
    auto const opposite_reference = formal_eskf::test::reference::reference_correction(
        opposite_input, ESKF_RESET_APPROX != 0, {}, ESKF_MAG_TILT != 0);
    test.expect(opposite.correct(opposite.state, opposite.P) == Status::success &&
                    formal_eskf::test::reference::matches_reference(opposite_input, opposite_reference, opposite.state,
                                                                    opposite.P, tolerance),
                profile, "opposite quaternion sign follows the same full physical correction model");
    test_magnetometer_analytic<Linalg, Configuration>(test, profile, tolerance);
}

template <typename Linalg, typename Configuration> void test_exact_policy(TestContext & test, std::string_view profile)
{
    using Fixture = MagnetometerFixture<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    for (bool const cyclic : {false, true})
    {
        Fixture fixture;
        if (cyclic)
        {
            test.expect(formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(
                            value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5}, Fixture::minimum_norm,
                            fixture.state.q_nb) == Status::success,
                        profile, "exact tilted projection control");
        }
        fixture.m_n = Fixture::vector_type::from_row_major({value_type{2}, value_type{0}, value_type{0}});
        fixture.m_b =
            cyclic ? Fixture::vector_type::from_row_major({value_type{0}, value_type{0}, value_type{2}}) : fixture.m_n;
        fixture.V = Fixture::matrix3_type::identity() * value_type{12};
        auto state = fixture.state;
        auto P = fixture.P;
        auto expected_P = fixture.P;
        std::size_t const down_axis = cyclic ? 1U : 2U;
        std::size_t const field_axis = cyclic ? 2U : 0U;
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            bool const observed = ESKF_MAG_TILT ? axis == down_axis : axis != field_axis;
            expected_P(Fixture::attitude_offset + axis, Fixture::attitude_offset + axis) =
                observed ? value_type{0.75} : value_type{1};
        }
        MagneticReference<Fixture> const reference_input(fixture);
        auto const reference = formal_eskf::test::reference::reference_correction(
            reference_input, ESKF_RESET_APPROX != 0, {}, ESKF_MAG_TILT != 0);
        test.expect(formal_eskf::test::reference::matches_reference(reference_input, reference, fixture.state,
                                                                    expected_P, value_type{0}),
                    profile,
                    "independent projection reference agrees with analytic identity/cyclic rational posterior");
        test.expect(fixture.correct(state, P) == Status::success && same_state(state, fixture.state) &&
                        same_bits(P, expected_P),
                    profile, "selected gain also controls Joseph covariance: only allowed axes lose uncertainty");
    }
}

template <typename Linalg, typename Configuration>
void test_projected_gain(TestContext & test, std::string_view profile)
{
    test_exact_policy<Linalg, Configuration>(test, profile);
#if ESKF_MAG_TILT
    using value_type = typename Linalg::value_type;
    // All64 bias masks interact with tilted attitude projection at this owner.
    constexpr value_type tolerance = static_cast<value_type>(std::is_same_v<value_type, float> ? 8e-6 : 2e-12);
    test_magnetometer_tilt<Linalg, Configuration>(test, profile, tolerance);
#endif
}

template <typename Linalg, typename Configuration>
void test_geometric_invariant(TestContext & test, std::string_view profile)
{
    using Fixture = MagnetometerFixture<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
#if ESKF_MAG_TILT
    constexpr value_type tolerance = static_cast<value_type>(std::is_same_v<value_type, float> ? 8e-6 : 2e-12);
#endif
    auto const fixture = coupled_fixture<Linalg, Configuration>(test, profile);
    auto const before = reference_rotation(fixture.state.q_nb);
    for (value_type const residual_scale :
         {value_type{0}, value_type{1}, std::ldexp(value_type{1}, -std::numeric_limits<value_type>::digits / 2)})
    {
        auto input = fixture;
        auto const predicted = Fixture::vector_type::from_row_major({value_type{1}, value_type{-3}, value_type{2}});
        input.m_b = predicted + (fixture.m_b - predicted) * residual_scale;
        auto const old_state = input.state;
        test.expect(input.correct(input.state, input.P) == Status::success, profile,
                    "geometric control exercises finite and Taylor-small actual corrections");
        auto const after = reference_rotation(input.state.q_nb);
        long double difference = 0;
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            difference = std::max(difference, std::abs(after[2U][axis] - before[2U][axis]));
        }
        if (residual_scale == value_type{0})
        {
            test.expect(same_state(input.state, old_state), profile, "zero residual is an explicit geometric control");
        }
        else
        {
            test.expect(!same_bits(input.state.q_nb.coefficients(), old_state.q_nb.coefficients()), profile,
                        "tilt invariant is tested with a nonzero attitude correction");
#if ESKF_MAG_TILT
            test.expect(difference <= static_cast<long double>(tolerance), profile,
                        "projected correction preserves navigation Down within supplemental numeric envelope");
#else
            if (residual_scale == value_type{1})
            {
                test.expect(difference > 1e-5L, profile,
                            "unrestricted negative control changes tilt for this fixed coupled field fixture");
            }
#endif
        }
    }
}

template <typename Linalg> void test_bias_forwarding(TestContext & test, std::string_view profile)
{
    using Fixture = MagnetometerFixture<Linalg, formal_eskf::configuration::Ins>;
    using value_type = typename Linalg::value_type;
    constexpr value_type tolerance = static_cast<value_type>(std::is_same_v<value_type, float> ? 8e-6 : 2e-12);
    auto fixture = coupled_fixture<Linalg, formal_eskf::configuration::Ins>(test, profile);
    MagneticReference<Fixture> const reference_input(fixture);
    std::array<unsigned, 14U> permissions{0U, 63U};
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        permissions[2U + axis] = 1U << axis;
        permissions[8U + axis] = 63U ^ (1U << axis);
    }
    auto unrestricted = fixture;
    test.expect(unrestricted.correct(unrestricted.state, unrestricted.P) == Status::success, profile,
                "default magnetic permissions permit correction");
    for (unsigned mask : permissions)
    {
        formal_eskf::configuration::Ins::BiasUpdate permission;
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            permission.accelerometer[axis] = (mask & (1U << axis)) != 0U;
            permission.gyroscope[axis] = (mask & (1U << (3U + axis))) != 0U;
        }
        auto const reference = formal_eskf::test::reference::reference_correction(
            reference_input, ESKF_RESET_APPROX != 0, permission, ESKF_MAG_TILT != 0);
        for (unsigned aliases = 0U; aliases < 4U; ++aliases)
        {
            auto input = fixture;
            typename Fixture::state_type separate_state;
            typename Fixture::covariance_type separate_P;
            auto & state = (aliases & 1U) != 0U ? input.state : separate_state;
            auto & P = (aliases & 2U) != 0U ? input.P : separate_P;
            test.expect(
                input.correct(state, P, permission) == Status::success &&
                    formal_eskf::test::reference::matches_reference(reference_input, reference, state, P, tolerance),
                profile, "magnetic permission partitions reach the complete independent gain/Joseph model");
            if (mask == 63U)
            {
                test.expect(same_state(state, unrestricted.state) && same_bits(P, unrestricted.P), profile,
                            "explicit default/all-enabled equals omitted magnetic permissions");
            }
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                test.expect((permission.accelerometer[axis] || state.b_a(axis) == fixture.state.b_a(axis)) &&
                                (permission.gyroscope[axis] || state.b_g(axis) == fixture.state.b_g(axis)),
                            profile, "disabled magnetic bias axes retain their own estimates");
            }
            test.expect(((aliases & 1U) != 0U || same_state(input.state, fixture.state)) &&
                            ((aliases & 2U) != 0U || same_bits(input.P, fixture.P)) &&
                            same_bits(input.m_b, fixture.m_b) && same_bits(input.m_n, fixture.m_n) &&
                            same_bits(input.V, fixture.V),
                        profile, "controlled magnetic correction preserves all distinct inputs");
            auto const before_state = state;
            auto const before_P = P;
            test.expect(correct_magnetometer(input.state, input.P, input.m_b, input.m_n, input.V, value_type{0}, state,
                                             P, permission) == Status::domain_error &&
                            same_state(state, before_state) && same_bits(P, before_P),
                        profile, "magnetic permissions preserve complete rollback on a downstream failure");
        }
    }
}

} /* end namespace */

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Case const cases[] = {
        {"TC-E-OBS-MAGNETIC-vector-model", "INS.binary32",
         test_magnetometer_jacobian<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ins>},
        {"TC-E-OBS-MAGNETIC-vector-model", "INS.binary64",
         test_magnetometer_jacobian<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ins>},
        {"TC-E-OBS-MAGNETIC-vector-model", "AHRS.binary32",
         test_magnetometer_jacobian<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ahrs>},
        {"TC-E-OBS-MAGNETIC-vector-model", "AHRS.binary64",
         test_magnetometer_jacobian<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ahrs>},
        {"TC-E-OBS-MAGNETIC-strength-domain", "INS.binary32",
         test_strength_domain<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ins>},
        {"TC-E-OBS-MAGNETIC-strength-domain", "INS.binary64",
         test_strength_domain<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ins>},
        {"TC-E-OBS-MAGNETIC-strength-domain", "AHRS.binary32",
         test_strength_domain<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ahrs>},
        {"TC-E-OBS-MAGNETIC-strength-domain", "AHRS.binary64",
         test_strength_domain<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ahrs>},
        {"TC-E-OBS-MAGNETIC-wrapper", "INS.binary32",
         test_wrapper<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ins>},
        {"TC-E-OBS-MAGNETIC-wrapper", "INS.binary64",
         test_wrapper<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ins>},
        {"TC-E-OBS-MAGNETIC-wrapper", "AHRS.binary32",
         test_wrapper<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ahrs>},
        {"TC-E-OBS-MAGNETIC-wrapper", "AHRS.binary64",
         test_wrapper<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ahrs>},
        {"TC-E-MAG-TILT-projected-gain", "INS.binary32",
         test_projected_gain<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ins>},
        {"TC-E-MAG-TILT-projected-gain", "INS.binary64",
         test_projected_gain<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ins>},
        {"TC-E-MAG-TILT-projected-gain", "AHRS.binary32",
         test_projected_gain<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ahrs>},
        {"TC-E-MAG-TILT-projected-gain", "AHRS.binary64",
         test_projected_gain<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ahrs>},
        {"TC-E-MAG-TILT-geometric-invariant", "INS.binary32",
         test_geometric_invariant<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ins>},
        {"TC-E-MAG-TILT-geometric-invariant", "INS.binary64",
         test_geometric_invariant<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ins>},
        {"TC-E-MAG-TILT-geometric-invariant", "AHRS.binary32",
         test_geometric_invariant<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ahrs>},
        {"TC-E-MAG-TILT-geometric-invariant", "AHRS.binary64",
         test_geometric_invariant<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ahrs>},
        {"TC-E-MAG-TILT-failure-selection", "INS.binary32",
         test_failure_selection<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ins>},
        {"TC-E-MAG-TILT-failure-selection", "INS.binary64",
         test_failure_selection<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ins>},
        {"TC-E-MAG-TILT-failure-selection", "AHRS.binary32",
         test_failure_selection<formal_eskf::test::Backend<float>, formal_eskf::configuration::Ahrs>},
        {"TC-E-MAG-TILT-failure-selection", "AHRS.binary64",
         test_failure_selection<formal_eskf::test::Backend<double>, formal_eskf::configuration::Ahrs>},
        {"TC-E-OBS-bias-forwarding", "INS.magnetometer.binary32",
         test_bias_forwarding<formal_eskf::test::Backend<float>>},
        {"TC-E-OBS-bias-forwarding", "INS.magnetometer.binary64",
         test_bias_forwarding<formal_eskf::test::Backend<double>>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
