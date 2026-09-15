/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <numeric>
#include <string_view>
#include <type_traits>

// Exercise the standalone header, without relying on the ESKF master header.
#include <formal_eskf/eskf/correction.hpp>
#include <formal_eskf/eskf/types.hpp>
#include "test_backend.hpp"

#include "test_support.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::Status;
using formal_eskf::try_correct;
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
    bool same = same_bits(a.q_nb.coefficients(), b.q_nb.coefficients());
    if constexpr (requires { a.p_n; })
    {
        same = same && same_bits(a.p_n, b.p_n) && same_bits(a.v_n, b.v_n) && same_bits(a.b_a, b.b_a) &&
               same_bits(a.b_g, b.b_g);
    }
    return same;
}

template <typename Linalg, typename Configuration, std::size_t MeasurementSize> struct CorrectionFixture
{
    using types = formal_eskf::EskfTypes<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    using correction_type = formal_eskf::linalg::Matrix<Linalg, types::error_state_dimension, 1U>;
    using state_type = typename types::nominal_state_type;
    using covariance_type = typename types::error_covariance_type;
    static constexpr bool is_ins = std::is_same_v<Configuration, formal_eskf::configuration::Ins>;
    static constexpr std::size_t size = types::error_state_dimension;
    static constexpr std::size_t measurement_size = MeasurementSize;
    static constexpr std::size_t attitude_offset = is_ins ? 6U : 0U;
    static constexpr value_type minimum_norm = static_cast<value_type>(1.0e-6);

    state_type state{};
    covariance_type P{};
    formal_eskf::linalg::Matrix<Linalg, MeasurementSize, 1U> r{};
    formal_eskf::linalg::Matrix<Linalg, MeasurementSize, size> H{};
    formal_eskf::linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> V{};
    Status attitude_status{};

    CorrectionFixture()
    {
        attitude_status = formal_eskf::so3::UnitQuaternion<Linalg>::try_from_coefficients(
            value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5}, minimum_norm, state.q_nb);
        if constexpr (is_ins)
        {
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                value_type const n = static_cast<value_type>(axis + 1U);
                state.p_n.set(axis, n);
                state.v_n.set(axis, -n);
                state.b_a.set(axis, n / value_type{8});
                state.b_g.set(axis, -n / value_type{16});
            }
        }
        // P = I + u*u^T; V = I/4 + w*w^T. Both are dense SPD matrices.
        for (std::size_t row = 0U; row < size; ++row)
        {
            for (std::size_t column = 0U; column < size; ++column)
            {
                value_type const u = static_cast<value_type>(row + 1U) / value_type{32};
                value_type const v = static_cast<value_type>(column + 1U) / value_type{32};
                P.set(row, column, u * v + (row == column ? value_type{1} : value_type{0}));
            }
        }
        for (std::size_t row = 0U; row < MeasurementSize; ++row)
        {
            r.set(row, static_cast<value_type>(row + 1U) / value_type{128});
            for (std::size_t column = 0U; column < size; ++column)
            {
                auto const pattern = static_cast<int>((row * 3U + column * 5U) % 7U) - 3;
                H.set(row, column,
                      static_cast<value_type>(pattern) / value_type{16} +
                          (column == attitude_offset + row % 3U ? value_type{1} : value_type{0}));
            }
            for (std::size_t column = 0U; column < MeasurementSize; ++column)
            {
                value_type const u = static_cast<value_type>(row + 1U) / value_type{16};
                value_type const v = static_cast<value_type>(column + 1U) / value_type{16};
                V.set(row, column, u * v + (row == column ? value_type{0.25} : value_type{0}));
            }
        }
    }
};

// Independent long-double oracle: no production linalg, LLT, Exp or reset calls.
template <std::size_t Rows, std::size_t Columns> using Dense = std::array<std::array<long double, Columns>, Rows>;

template <std::size_t Rows, std::size_t Inner, std::size_t Columns>
[[nodiscard]] Dense<Rows, Columns> product(Dense<Rows, Inner> const & a, Dense<Inner, Columns> const & b)
{
    Dense<Rows, Columns> output{};
    for (std::size_t row = 0U; row < Rows; ++row)
    {
        for (std::size_t column = 0U; column < Columns; ++column)
        {
            for (std::size_t k = 0U; k < Inner; ++k)
            {
                output[row][column] += a[row][k] * b[k][column];
            }
        }
    }
    return output;
}

template <std::size_t Rows, std::size_t Columns>
[[nodiscard]] Dense<Columns, Rows> transposed(Dense<Rows, Columns> const & a)
{
    Dense<Columns, Rows> output{};
    for (std::size_t row = 0U; row < Rows; ++row)
    {
        for (std::size_t column = 0U; column < Columns; ++column)
        {
            output[column][row] = a[row][column];
        }
    }
    return output;
}

template <typename Matrix> [[nodiscard]] Dense<Matrix::row_count, Matrix::column_count> copied(Matrix const & a)
{
    Dense<Matrix::row_count, Matrix::column_count> output{};
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            output[row][column] = static_cast<long double>(a(row, column));
        }
    }
    return output;
}

template <std::size_t Size> struct ReferenceCorrection
{
    Dense<Size, 1U> delta_x{};
    Dense<Size, Size> joseph_covariance{};
    Dense<Size, Size> covariance{};
    bool valid = false;
};

template <typename Fixture>
[[nodiscard]] ReferenceCorrection<Fixture::size> reference_correction(Fixture const & fixture)
{
    constexpr std::size_t n = Fixture::size;
    constexpr std::size_t m = Fixture::measurement_size;
    constexpr std::size_t offset = Fixture::attitude_offset;
    ReferenceCorrection<n> result;
    auto const P = copied(fixture.P);
    auto const H = copied(fixture.H);
    auto const V = copied(fixture.V);
    auto const HP = product(H, P);
    auto S = product(HP, transposed(H));
    Dense<m, m + n> augmented{};
    for (std::size_t row = 0U; row < m; ++row)
    {
        for (std::size_t column = 0U; column < m; ++column)
        {
            S[row][column] += V[row][column];
            augmented[row][column] = S[row][column];
        }
        for (std::size_t column = 0U; column < n; ++column)
        {
            augmented[row][m + column] = HP[row][column];
        }
    }
    // Pivoted Gauss-Jordan elimination is a test oracle, not a production backend.
    for (std::size_t k = 0U; k < m; ++k)
    {
        std::size_t pivot = k;
        for (std::size_t row = k + 1U; row < m; ++row)
        {
            if (std::abs(augmented[row][k]) > std::abs(augmented[pivot][k]))
            {
                pivot = row;
            }
        }
        if (!(std::abs(augmented[pivot][k]) > 0.0L))
        {
            return result;
        }
        std::swap(augmented[k], augmented[pivot]);
        long double const divisor = augmented[k][k];
        std::transform(augmented[k].begin(), augmented[k].end(), augmented[k].begin(),
                       [divisor](long double entry) { return entry / divisor; });
        for (std::size_t row = 0U; row < m; ++row)
        {
            if (row != k)
            {
                long double const factor = augmented[row][k];
                for (std::size_t column = 0U; column < m + n; ++column)
                {
                    augmented[row][column] -= factor * augmented[k][column];
                }
            }
        }
    }
    Dense<n, m> K{};
    for (std::size_t row = 0U; row < n; ++row)
    {
        for (std::size_t column = 0U; column < m; ++column)
        {
            K[row][column] = augmented[column][m + row];
        }
    }
    result.delta_x = product(K, copied(fixture.r));
    auto A = product(K, H);
    for (std::size_t row = 0U; row < n; ++row)
    {
        for (std::size_t column = 0U; column < n; ++column)
        {
            A[row][column] = (row == column ? 1.0L : 0.0L) - A[row][column];
        }
    }
    auto joseph = product(product(A, P), transposed(A));
    auto const noise = product(product(K, V), transposed(K));
    for (std::size_t row = 0U; row < n; ++row)
    {
        for (std::size_t column = 0U; column < n; ++column)
        {
            joseph[row][column] += noise[row][column];
        }
    }
    result.joseph_covariance = joseph;
    long double const x = result.delta_x[offset][0U];
    long double const y = result.delta_x[offset + 1U][0U];
    long double const z = result.delta_x[offset + 2U][0U];
    Dense<3U, 3U> const negative_hat{{{0.0L, z, -y}, {-z, 0.0L, x}, {y, -x, 0.0L}}};
    Dense<3U, 3U> term{{{1.0L, 0.0L, 0.0L}, {0.0L, 1.0L, 0.0L}, {0.0L, 0.0L, 1.0L}}};
    auto Jr = term;
    // Jr(a) = sum_k (-hat(a))^k / (k+1)!, independent of the production formula.
    constexpr std::size_t terms = ESKF_RESET_APPROX ? 1U : 32U;
    for (std::size_t k = 1U; k <= terms; ++k)
    {
        term = product(term, negative_hat);
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            for (std::size_t column = 0U; column < 3U; ++column)
            {
                term[row][column] /= static_cast<long double>(k + 1U);
                Jr[row][column] += term[row][column];
            }
        }
    }
    Dense<n, n> G{};
    for (std::size_t row = 0U; row < n; ++row)
    {
        G[row][row] = 1.0L;
    }
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            G[offset + row][offset + column] = Jr[row][column];
        }
    }
    result.covariance = product(product(G, joseph), transposed(G));
    result.valid = true;
    return result;
}

template <typename Fixture>
[[nodiscard]] bool matches_reference(Fixture const & fixture, ReferenceCorrection<Fixture::size> const & reference,
                                     typename Fixture::state_type const & state,
                                     typename Fixture::covariance_type const & covariance,
                                     typename Fixture::value_type tolerance)
{
    using value_type = typename Fixture::value_type;
    constexpr std::size_t offset = Fixture::attitude_offset;
    long double const x = reference.delta_x[offset][0U];
    long double const y = reference.delta_x[offset + 1U][0U];
    long double const z = reference.delta_x[offset + 2U][0U];
    long double const theta = std::sqrt(x * x + y * y + z * z);
    long double const scale = theta == 0.0L ? 0.5L : std::sin(theta / 2.0L) / theta;
    long double const c = std::cos(theta / 2.0L);
    long double const a = x * scale;
    long double const b = y * scale;
    long double const d = z * scale;
    auto const q = copied(fixture.state.q_nb.coefficients());
    std::array<long double, 4U> expected{q[0U][0U] * c - q[1U][0U] * a - q[2U][0U] * b - q[3U][0U] * d,
                                         q[0U][0U] * a + q[1U][0U] * c + q[2U][0U] * d - q[3U][0U] * b,
                                         q[0U][0U] * b - q[1U][0U] * d + q[2U][0U] * c + q[3U][0U] * a,
                                         q[0U][0U] * d + q[1U][0U] * b - q[2U][0U] * a + q[3U][0U] * c};
    long double const squared_norm = std::inner_product(expected.begin(), expected.end(), expected.begin(), 0.0L);
    bool matches = reference.valid && formal_eskf::linalg::all_finite(covariance) &&
                   formal_eskf::linalg::is_symmetric(covariance, value_type{0});
    auto const coefficients = state.q_nb.coefficients();
    for (std::size_t index = 0U; index < 4U; ++index)
    {
        matches = matches && near(coefficients(index),
                                  static_cast<value_type>(expected[index] / std::sqrt(squared_norm)), tolerance);
    }
    if constexpr (Fixture::is_ins)
    {
        using state_type = typename Fixture::state_type;
        auto const members = std::array{&state_type::p_n, &state_type::v_n, &state_type::b_a, &state_type::b_g};
        constexpr std::array<std::size_t, 4U> offsets{0U, 3U, 9U, 12U};
        for (std::size_t group = 0U; group < members.size(); ++group)
        {
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                long double const expected_value = static_cast<long double>((fixture.state.*members[group])(axis)) +
                                                   reference.delta_x[offsets[group] + axis][0U];
                matches =
                    matches && near((state.*members[group])(axis), static_cast<value_type>(expected_value), tolerance);
            }
        }
    }
    for (std::size_t row = 0U; row < Fixture::size; ++row)
    {
        matches = matches && covariance(row, row) >= value_type{0};
        for (std::size_t column = 0U; column < Fixture::size; ++column)
        {
            matches = matches && near(covariance(row, column),
                                      static_cast<value_type>(reference.covariance[row][column]), tolerance);
        }
    }
    return matches;
}

template <typename Fixture>
void check_success(TestContext & test, std::string_view profile, Fixture const & fixture,
                   typename Fixture::value_type tolerance)
{
    auto const reference = reference_correction(fixture);
    test.expect(reference.valid && fixture.attitude_status == Status::success, profile,
                "valid correction fixture and oracle");
    typename Fixture::correction_type delta;
    typename Fixture::covariance_type joseph;
    Status const linear_status = formal_eskf::detail::try_compute_correction(fixture.P, fixture.r, fixture.H, fixture.V,
                                                                             Fixture::minimum_norm, delta, joseph);
    bool linear_matches = linear_status == Status::success;
    for (std::size_t row = 0U; row < Fixture::size; ++row)
    {
        linear_matches =
            near(delta(row), static_cast<typename Fixture::value_type>(reference.delta_x[row][0U]), tolerance) &&
            linear_matches;
        for (std::size_t column = 0U; column < Fixture::size; ++column)
        {
            linear_matches =
                near(joseph(row, column),
                     static_cast<typename Fixture::value_type>(reference.joseph_covariance[row][column]), tolerance) &&
                linear_matches;
        }
    }
    test.expect(linear_matches, profile, "pre-injection error and full Joseph covariance match independent oracle");
    auto const saved_delta = delta;
    auto const saved_joseph = joseph;
    Status const rejected = formal_eskf::detail::try_compute_correction(fixture.P, fixture.r, fixture.H, fixture.V,
                                                                        typename Fixture::value_type{0}, delta, joseph);
    test.expect(rejected == Status::domain_error && same_bits(delta, saved_delta) && same_bits(joseph, saved_joseph),
                profile, "pre-injection failure preserves both scratch outputs");
    for (unsigned aliases = 0U; aliases < 4U; ++aliases)
    {
        auto input = fixture;
        typename Fixture::state_type separate_state;
        auto separate_covariance = Fixture::covariance_type::identity();
        auto & state_output = (aliases & 1U) != 0U ? input.state : separate_state;
        auto & covariance_output = (aliases & 2U) != 0U ? input.P : separate_covariance;
        Status const status = try_correct(input.state, input.P, input.r, input.H, input.V, Fixture::minimum_norm,
                                          state_output, covariance_output);
        test.expect(status == Status::success &&
                        matches_reference(fixture, reference, state_output, covariance_output, tolerance),
                    profile, "correction agrees with independent gain/Joseph/Exp/reset oracle for every output alias");
        bool const immutable = same_bits(input.r, fixture.r) && same_bits(input.H, fixture.H) &&
                               same_bits(input.V, fixture.V) &&
                               ((aliases & 1U) != 0U || same_state(input.state, fixture.state)) &&
                               ((aliases & 2U) != 0U || same_bits(input.P, fixture.P));
        test.expect(immutable, profile, "correction does not modify distinct inputs");
    }
}

template <typename Linalg, typename Configuration, std::size_t MeasurementSize>
void test_correction_success(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using fixture_type = CorrectionFixture<Linalg, Configuration, MeasurementSize>;
    using value_type = typename Linalg::value_type;
    fixture_type fixture;
    check_success(test, profile, fixture, tolerance);
    auto larger_residual = fixture;
    larger_residual.r = larger_residual.r * value_type{8};
    check_success(test, profile, larger_residual, tolerance);
    auto zero_residual = fixture;
    zero_residual.r = {};
    check_success(test, profile, zero_residual, tolerance); // K still updates P when r is zero.
    auto zero_jacobian = fixture;
    zero_jacobian.H = {};
    check_success(test, profile, zero_jacobian, tolerance);
    auto large_unobserved_prior = zero_jacobian;
    large_unobserved_prior.P.set(0U, 0U, std::numeric_limits<value_type>::max());
    check_success(test, profile, large_unobserved_prior, tolerance); // Symmetry cleanup must not overflow P + P^T.
    auto zero_prior = fixture;
    zero_prior.P = {};
    check_success(test, profile, zero_prior, tolerance); // A PSD prior need not be positive definite.
    auto semidefinite_prior = fixture;
    for (std::size_t index = 0U; index < fixture_type::size; ++index)
    {
        semidefinite_prior.P.set(0U, index, value_type{0});
        semidefinite_prior.P.set(index, 0U, value_type{0});
    }
    check_success(test, profile, semidefinite_prior, tolerance);
    auto negative_quaternion = fixture;
    negative_quaternion.state.q_nb = -negative_quaternion.state.q_nb;
    check_success(test, profile, negative_quaternion, tolerance);
}

template <typename Linalg, typename Configuration>
void test_repeated_scalar_correction(TestContext & test, std::string_view profile,
                                     typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    using fixture_type = CorrectionFixture<Linalg, Configuration, 1U>;
    fixture_type fixture;
    fixture.P = fixture_type::covariance_type::identity();
    fixture.H = {};
    fixture.H.set(0U, fixture_type::attitude_offset, value_type{1});
    fixture.V.set(0U, 0U, value_type{0.25});
    fixture.r = {};
    auto const prior_state = fixture.state;
    // With zero residual the reset is identity. Repeated scalar updates have
    // the independent closed form p_k = 1 / (1 + k / v).
    for (std::size_t count = 1U; count <= 100U; ++count)
    {
        Status const status = try_correct(fixture.state, fixture.P, fixture.r, fixture.H, fixture.V,
                                          fixture_type::minimum_norm, fixture.state, fixture.P);
        value_type const expected = value_type{1} / (value_type{1} + value_type{4} * static_cast<value_type>(count));
        bool correct = status == Status::success && same_state(fixture.state, prior_state);
        for (std::size_t row = 0U; row < fixture_type::size; ++row)
        {
            for (std::size_t column = 0U; column < fixture_type::size; ++column)
            {
                value_type const diagonal = row == fixture_type::attitude_offset ? expected : value_type{1};
                correct = correct && near(fixture.P(row, column), row == column ? diagonal : value_type{0}, tolerance);
            }
        }
        test.expect(correct, profile,
                    "100 scalar corrections match analytic variance and do not drift at zero residual");
    }
}

template <typename Fixture>
void check_failure(TestContext & test, std::string_view profile, Fixture const & bad,
                   typename Fixture::value_type minimum_norm, Status expected, std::string_view message)
{
    for (unsigned aliases = 0U; aliases < 4U; ++aliases)
    {
        auto input = bad;
        typename Fixture::state_type separate_state;
        auto separate_covariance = Fixture::covariance_type::identity();
        auto const saved_state = separate_state;
        auto const saved_covariance = separate_covariance;
        auto & state_output = (aliases & 1U) != 0U ? input.state : separate_state;
        auto & covariance_output = (aliases & 2U) != 0U ? input.P : separate_covariance;
        Status const status =
            try_correct(input.state, input.P, input.r, input.H, input.V, minimum_norm, state_output, covariance_output);
        bool const unchanged = same_state(input.state, bad.state) && same_bits(input.P, bad.P) &&
                               same_bits(input.r, bad.r) && same_bits(input.H, bad.H) && same_bits(input.V, bad.V) &&
                               same_state(separate_state, saved_state) &&
                               same_bits(separate_covariance, saved_covariance);
        test.expect(status == expected && unchanged, profile, message);
    }
}

template <typename Linalg, typename Configuration>
void test_joseph_cancellation(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using fixture_type = CorrectionFixture<Linalg, Configuration, 1U>;
    fixture_type fixture;
    fixture.P = fixture_type::covariance_type::identity();
    fixture.H = {};
    fixture.H.set(0U, fixture_type::attitude_offset, value_type{1});
    fixture.r = {};
    value_type const epsilon = std::numeric_limits<value_type>::epsilon();
    value_type const variance = epsilon / value_type{4};
    fixture.V.set(0U, 0U, variance);
    // S rounds to 1 and K to 1. The simplified (I-KH)P would lose the
    // remaining variance entirely; Joseph retains it through K V K^T.
    Status const status = try_correct(fixture.state, fixture.P, fixture.r, fixture.H, fixture.V,
                                      fixture_type::minimum_norm, fixture.state, fixture.P);
    value_type const posterior = fixture.P(fixture_type::attitude_offset, fixture_type::attitude_offset);
    long double const v = static_cast<long double>(variance);
    value_type const expected = static_cast<value_type>(v / (1.0L + v));
    test.expect(status == Status::success && posterior > value_type{0} &&
                    near(posterior, expected, variance * epsilon * value_type{4}),
                profile, "Joseph retains positive measurement variance when I-KH cancels to zero");
}

template <typename Linalg, typename Configuration>
void test_correction_failures(TestContext & test, std::string_view profile)
{
    using fixture_type = CorrectionFixture<Linalg, Configuration, 2U>;
    using value_type = typename Linalg::value_type;
    fixture_type fixture;
    value_type const nan = std::numeric_limits<value_type>::quiet_NaN();
    value_type const infinity = std::numeric_limits<value_type>::infinity();
    value_type const maximum = std::numeric_limits<value_type>::max();
    auto fail = [&](fixture_type const & bad, Status expected, std::string_view message)
    { check_failure(test, profile, bad, fixture_type::minimum_norm, expected, message); };
    for (auto const invalid : {value_type{0}, value_type{-1}, value_type{2}, nan, infinity})
    {
        check_failure(test, profile, fixture, invalid,
                      std::isfinite(invalid) ? Status::domain_error : Status::non_finite_input,
                      "invalid norm threshold preserves both outputs");
    }
    auto non_finite_entries = [&](auto member)
    {
        using matrix_type = std::remove_cvref_t<decltype(fixture.*member)>;
        for (std::size_t row = 0U; row < matrix_type::row_count; ++row)
        {
            for (std::size_t column = 0U; column < matrix_type::column_count; ++column)
            {
                for (auto const invalid : {nan, infinity, -infinity})
                {
                    auto bad = fixture;
                    (bad.*member).set(row, column, invalid);
                    fail(bad, Status::non_finite_input, "every non-finite P/r/H/V coefficient is rejected atomically");
                }
            }
        }
    };
    non_finite_entries(&fixture_type::P);
    non_finite_entries(&fixture_type::r);
    non_finite_entries(&fixture_type::H);
    non_finite_entries(&fixture_type::V);
    auto bad = fixture;
    bad.P.set(0U, 1U, std::nextafter(bad.P(0U, 1U), infinity));
    fail(bad, Status::domain_error, "one-ULP asymmetric prior is rejected, not silently symmetrized");
    bad = fixture;
    bad.V.set(0U, 1U, std::nextafter(bad.V(0U, 1U), infinity));
    fail(bad, Status::domain_error, "one-ULP asymmetric measurement covariance is rejected");
    bad = fixture;
    bad.P.set(0U, 0U, value_type{-1});
    fail(bad, Status::domain_error, "negative prior variance is rejected");
    for (auto const invalid : {value_type{0}, value_type{-1}})
    {
        bad = fixture;
        bad.V.set(0U, 0U, invalid);
        fail(bad, Status::domain_error, "nonpositive measurement variance is rejected");
    }
    // Deliberately violate V's SPD precondition to exercise innovation LLT failure.
    // Passing the diagonal checks alone does not certify a valid covariance.
    bad = fixture;
    bad.H = {};
    bad.V.set(0U, 1U, value_type{2});
    bad.V.set(1U, 0U, value_type{2});
    fail(bad, Status::not_positive_definite, "indefinite innovation propagates LLT failure atomically");

    // All remaining matrix inputs satisfy P PSD and V SPD; failures are numerical.
    fixture.P = fixture_type::covariance_type::identity();
    fixture.H = {};
    fixture.V = decltype(fixture.V)::identity();
    fixture.r = {};
    bad = fixture;
    bad.P.set(0U, 0U, maximum);
    bad.H.set(0U, 0U, value_type{2});
    fail(bad, Status::non_finite_result, "P H^T overflow preserves outputs");
    bad = fixture;
    bad.P.set(0U, 0U, maximum / value_type{2});
    bad.H.set(0U, 0U, value_type{1.5});
    fail(bad, Status::non_finite_result, "H P H^T overflow preserves outputs");
    bad = fixture;
    bad.P.set(0U, 0U, maximum);
    bad.H.set(0U, 0U, value_type{1});
    bad.V.set(0U, 0U, maximum);
    fail(bad, Status::non_finite_result, "innovation addition overflow preserves outputs");
    bad = fixture;
    bad.H.set(0U, fixture_type::attitude_offset, value_type{0.25});
    bad.V.set(0U, 0U, value_type{0.0625});
    bad.r.set(0U, maximum); // K = 2, so delta_x itself overflows.
    fail(bad, Status::non_finite_result, "gain-residual overflow preserves outputs");
    bad = fixture;
    bad.H.set(0U, fixture_type::attitude_offset, value_type{1});
    bad.r.set(0U, maximum); // Finite delta_x, but Exp's squared norm overflows.
    fail(bad, Status::non_finite_result, "late injection failure does not commit the Joseph covariance");

    bad = fixture;
    constexpr std::size_t offset = fixture_type::attitude_offset;
    value_type const large_variance = maximum * value_type{0.75};
    // The xy block is rank-one PSD. Correction observes only z, leaving this
    // block intact until reset. Jr(pi/2 * e_z) concentrates its variance into x
    // beyond the scalar range (the first-order reset overflows too).
    bad.P.set(offset, offset, large_variance);
    bad.P.set(offset, offset + 1U, large_variance);
    bad.P.set(offset + 1U, offset, large_variance);
    bad.P.set(offset + 1U, offset + 1U, large_variance);
    bad.H.set(0U, offset + 2U, value_type{1});
    bad.r.set(0U, static_cast<value_type>(std::acos(-1.0L)));
    fail(bad, Status::non_finite_result, "reset overflow rolls back the already-computed nominal injection");

    if constexpr (fixture_type::is_ins)
    {
        using state_type = typename fixture_type::state_type;
        auto const members = std::array{&state_type::p_n, &state_type::v_n, &state_type::b_a, &state_type::b_g};
        constexpr std::array<std::size_t, 4U> offsets{0U, 3U, 9U, 12U};
        for (std::size_t group = 0U; group < members.size(); ++group)
        {
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                bad = fixture;
                (bad.state.*members[group]).set(axis, nan);
                fail(bad, Status::non_finite_input, "non-finite INS nominal fields preserve outputs");
                bad = fixture;
                (bad.state.*members[group]).set(axis, maximum);
                bad.H.set(0U, offsets[group] + axis, value_type{0.25});
                bad.V.set(0U, 0U, value_type{0.0625});
                bad.r.set(0U, maximum / value_type{4});
                fail(bad, Status::non_finite_result, "late additive INS injection overflow preserves both outputs");
            }
        }
    }
}

template <typename Linalg, typename Configuration>
void run_correction_tests(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    test_correction_success<Linalg, Configuration, 1U>(test, profile, tolerance);
    test_correction_success<Linalg, Configuration, 2U>(test, profile, tolerance);
    test_correction_success<Linalg, Configuration, 3U>(test, profile, tolerance);
    test_correction_success<Linalg, Configuration, 6U>(test, profile, tolerance);
    test_repeated_scalar_correction<Linalg, Configuration>(test, profile, tolerance);
    test_joseph_cancellation<Linalg, Configuration>(test, profile);
    test_correction_failures<Linalg, Configuration>(test, profile);
}

} /* end namespace */

int main()
{
    TestContext test;
    formal_eskf::test::configure_backend_test();
    using FloatLinalg = formal_eskf::test::Backend<float>;
    using DoubleLinalg = formal_eskf::test::Backend<double>;
    using formal_eskf::configuration::Ahrs;
    using formal_eskf::configuration::Ins;
    run_correction_tests<FloatLinalg, Ahrs>(test, "AHRS binary32", 8.0e-6F);
    run_correction_tests<FloatLinalg, Ins>(test, "INS binary32", 8.0e-6F);
    run_correction_tests<DoubleLinalg, Ahrs>(test, "AHRS binary64", 2.0e-12);
    run_correction_tests<DoubleLinalg, Ins>(test, "INS binary64", 2.0e-12);
    if (test.failures() != 0)
    {
        std::cerr << test.failures() << " ESKF correction test(s) failed\n";
        return 1;
    }
    std::cout << "All ESKF correction tests passed (" << (ESKF_RESET_APPROX ? "first-order reset" : "closed-form reset")
              << ")\n";
    return 0;
}
