/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numeric>

#include <formal_eskf/eskf/configuration/ins.hpp>
#include "test_support.hpp"

namespace formal_eskf::test::reference
{

// Fixture supplies sizes/offset/is_ins and state/P/H/V/r plus their scalar/state/
// covariance aliases. Callers own geometry and acceptance bounds. The explicit
// reset selector is part of this reference model, not a call to production reset.
// Independent long-double regression oracle: no production linalg, LLT, Exp
// or reset calls. This is not a certified enclosure or a general conditioning/
// attitude error budget. Callers supply separate exact dyadic equality oracles.
template <std::size_t Rows, std::size_t Columns> using Dense = std::array<std::array<long double, Columns>, Rows>;

template <std::size_t Rows, std::size_t Inner, std::size_t Columns>
Dense<Rows, Columns> product(Dense<Rows, Inner> const & a, Dense<Inner, Columns> const & b)
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

template <std::size_t Rows, std::size_t Columns> Dense<Columns, Rows> transposed(Dense<Rows, Columns> const & a)
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

template <typename Matrix> Dense<Matrix::row_count, Matrix::column_count> copied(Matrix const & a)
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
ReferenceCorrection<Fixture::size>
reference_correction(Fixture const & fixture, bool approximate_reset,
                     formal_eskf::configuration::Ins::BiasUpdate const & bias_update = {})
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
    if constexpr (Fixture::is_ins)
    {
        // Independent dense projection oracle, not the production row-zeroing loop.
        Dense<n, n> selection{};
        for (std::size_t row = 0U; row < n; ++row)
        {
            selection[row][row] = 1.0L;
        }
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            selection[9U + axis][9U + axis] = bias_update.accelerometer[axis] ? 1.0L : 0.0L;
            selection[12U + axis][12U + axis] = bias_update.gyroscope[axis] ? 1.0L : 0.0L;
        }
        K = product(selection, K);
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
    std::size_t const terms = approximate_reset ? 1U : 32U;
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
bool matches_reference(Fixture const & fixture, ReferenceCorrection<Fixture::size> const & reference,
                       typename Fixture::state_type const & state, typename Fixture::covariance_type const & covariance,
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
    bool matches = reference.valid;
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
            matches =
                matches && std::isfinite(covariance(row, column)) &&
                covariance(row, column) == covariance(column, row) &&
                near(covariance(row, column), static_cast<value_type>(reference.covariance[row][column]), tolerance);
        }
    }
    return matches;
}

} /* end namespace formal_eskf::test::reference */
