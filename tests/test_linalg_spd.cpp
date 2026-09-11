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
#include <string_view>

// Check that the solver header does not depend on the backend include order.
#include <formal_eskf/linalg/solve.hpp>
#include "test_backend.hpp"

#include "test_support.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::Status;
using formal_eskf::linalg::right_solve_spd;
using formal_eskf::linalg::solve_spd;

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

template <typename Matrix> [[nodiscard]] bool matrix_near(Matrix const & actual, Matrix const & expected)
{
    using value_type = typename Matrix::value_type;
    long double const tolerance = 64.0L * std::numeric_limits<value_type>::epsilon();
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            long double const reference = expected(row, column);
            long double const error = std::abs(static_cast<long double>(actual(row, column)) - reference);
            if (!(error <= tolerance * std::max(1.0L, std::abs(reference))))
            {
                return false;
            }
        }
    }
    return true;
}

// Manufacture right-hand sides independently of the backend and the production
// matrix product. The bounded dyadic fixtures below have exactly representable
// products/sums, so their known solutions are also independent solve oracles.
template <typename Left, typename Right> [[nodiscard]] auto reference_product(Left const & a, Right const & b)
{
    static_assert(Left::column_count == Right::row_count);
    using linalg_type = typename Left::linalg_type;
    using value_type = typename Left::value_type;
    typename linalg_type::template matrix_type<Left::row_count, Right::column_count> result;
    for (std::size_t row = 0U; row < Left::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Right::column_count; ++column)
        {
            long double sum = 0;
            for (std::size_t inner = 0U; inner < Left::column_count; ++inner)
            {
                sum += static_cast<long double>(a(row, inner)) * static_cast<long double>(b(inner, column));
            }
            result(row, column) = static_cast<value_type>(sum);
        }
    }
    return result;
}

template <typename Linalg, std::size_t Size, std::size_t RightColumns>
void test_dimensions(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using system_type = typename Linalg::template matrix_type<Size, Size>;
    using rhs_type = typename Linalg::template matrix_type<Size, RightColumns>;
    system_type system;
    rhs_type expected;
    for (std::size_t row = 0U; row < Size; ++row)
    {
        for (std::size_t column = 0U; column < Size; ++column)
        {
            // Symmetric, strictly diagonally dominant with positive diagonal: SPD.
            system(row, column) = row == column ? static_cast<value_type>(Size + row + 1U) : value_type{0.25};
        }
        for (std::size_t column = 0U; column < RightColumns; ++column)
        {
            expected(row, column) = static_cast<value_type>(row + 1U) * value_type{0.5} -
                                    static_cast<value_type>(column) * value_type{0.25};
        }
    }
    auto const rhs = reference_product(system, expected);
    auto const saved_system = system;
    auto const saved_rhs = rhs;
    rhs_type output;
    output(0U, 0U) = std::numeric_limits<value_type>::quiet_NaN();
    Status status = solve_spd(system, rhs, output);
    test.expect(status == Status::success && matrix_near(output, expected), profile,
                "left solve recovers independent known solution for fixed dimensions and multiple RHS");
    test.expect(matrix_near(reference_product(system, output), rhs), profile, "left solve satisfies A*X=B");
    test.expect(same_bits(system, saved_system) && same_bits(rhs, saved_rhs), profile,
                "left solve preserves distinct inputs");

    auto rhs_alias = rhs;
    status = solve_spd(system, rhs_alias, rhs_alias);
    test.expect(status == Status::success && matrix_near(rhs_alias, expected), profile,
                "left solve permits output to alias rectangular RHS");

    auto const right_expected = transpose(expected);
    auto const right_rhs = reference_product(right_expected, system);
    auto right_output = right_rhs;
    status = right_solve_spd(right_rhs, system, right_output);
    test.expect(status == Status::success && matrix_near(right_output, right_expected), profile,
                "right SPD solve recovers the independent known solution");
    test.expect(matrix_near(reference_product(right_output, system), right_rhs), profile,
                "right solve satisfies X*A=B");
    auto right_alias = right_rhs;
    status = right_solve_spd(right_alias, system, right_alias);
    test.expect(status == Status::success && matrix_near(right_alias, right_expected), profile,
                "right solve permits output to alias rectangular RHS");
}

template <typename Matrix>
void expect_failure(TestContext & test, std::string_view profile, Matrix const & system, Matrix const & rhs,
                    Status expected_status)
{
    using value_type = typename Matrix::value_type;
    Matrix sentinel = Matrix::identity();
    sentinel(0U, 0U) = std::numeric_limits<value_type>::quiet_NaN();
    sentinel(0U, 1U) = -value_type{0};
    auto const saved_system = system;
    auto const saved_rhs = rhs;

    auto output = sentinel;
    Status status = solve_spd(system, rhs, output);
    test.expect(status == expected_status && same_bits(output, sentinel), profile,
                "failed left solve reports its cause and preserves every output bit");
    status = right_solve_spd(rhs, system, output);
    test.expect(status == expected_status && same_bits(output, sentinel), profile,
                "failed right solve reports its cause and preserves every output bit");

    auto system_alias = system;
    status = solve_spd(system_alias, rhs, system_alias);
    test.expect(status == expected_status && same_bits(system_alias, saved_system), profile,
                "failed left solve preserves output aliased to system");
    system_alias = system;
    status = right_solve_spd(rhs, system_alias, system_alias);
    test.expect(status == expected_status && same_bits(system_alias, saved_system), profile,
                "failed right solve preserves output aliased to system");

    auto rhs_alias = rhs;
    status = solve_spd(system, rhs_alias, rhs_alias);
    test.expect(status == expected_status && same_bits(rhs_alias, saved_rhs), profile,
                "failed left solve preserves output aliased to RHS");
    rhs_alias = rhs;
    status = right_solve_spd(rhs_alias, system, rhs_alias);
    test.expect(status == expected_status && same_bits(rhs_alias, saved_rhs), profile,
                "failed right solve preserves output aliased to RHS");
    test.expect(same_bits(system, saved_system) && same_bits(rhs, saved_rhs), profile,
                "failed solves preserve distinct inputs");
}

template <typename Linalg> void test_aliases(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using matrix_type = typename Linalg::template matrix_type<2U, 2U>;
    constexpr value_type system_values[4] = {4, 1, 1, 3};
    constexpr value_type expected_values[4] = {1, -2, 3, 4};
    matrix_type const system = matrix_type::from_row_major(system_values);
    matrix_type const expected = matrix_type::from_row_major(expected_values);
    auto const rhs = reference_product(system, expected);
    auto const right_rhs = reference_product(expected, system);
    auto alias = system;
    Status status = solve_spd(alias, rhs, alias);
    test.expect(status == Status::success && matrix_near(alias, expected), profile,
                "left SPD solve allows output to overwrite the system");
    alias = system;
    status = right_solve_spd(right_rhs, alias, alias);
    test.expect(status == Status::success && matrix_near(alias, expected), profile,
                "right SPD solve allows output to overwrite the system");

    alias = system;
    status = solve_spd(alias, alias, alias);
    test.expect(status == Status::success && matrix_near(alias, matrix_type::identity()), profile,
                "left solve permits all three arguments to alias");
    alias = system;
    status = right_solve_spd(alias, alias, alias);
    test.expect(status == Status::success && matrix_near(alias, matrix_type::identity()), profile,
                "right solve permits all three arguments to alias");

    matrix_type indefinite = matrix_type::identity();
    indefinite(1U, 1U) = value_type{-2};
    matrix_type output;
    status = solve_spd(indefinite, indefinite, output);
    test.expect(status == Status::not_positive_definite, profile, "SPD solve rejects an invertible indefinite system");
    matrix_type zero_rhs;
    status = solve_spd(system, zero_rhs, output);
    test.expect(status == Status::success && matrix_near(output, zero_rhs), profile,
                "invertible system with zero RHS has the unique zero solution");
}

template <typename Linalg> void test_failures(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using matrix_type = typename Linalg::template matrix_type<2U, 2U>;
    matrix_type const identity = matrix_type::identity();
    matrix_type const zero;
    expect_failure(test, profile, zero, identity, Status::not_positive_definite);
    expect_failure(test, profile, zero, zero, Status::not_positive_definite);
    constexpr value_type singular_values[4] = {1, 2, 2, 4};
    auto const singular = matrix_type::from_row_major(singular_values);
    expect_failure(test, profile, singular, identity, Status::not_positive_definite);
    expect_failure(test, profile, singular, singular, Status::not_positive_definite);
    auto all_alias = singular;
    Status status = solve_spd(all_alias, all_alias, all_alias);
    test.expect(status == Status::not_positive_definite && same_bits(all_alias, singular), profile,
                "singular left solve preserves fully aliased arguments");
    status = right_solve_spd(all_alias, all_alias, all_alias);
    test.expect(status == Status::not_positive_definite && same_bits(all_alias, singular), profile,
                "singular right solve preserves fully aliased arguments");

    std::array<value_type, 3U> const invalid_values{std::numeric_limits<value_type>::quiet_NaN(),
                                                    std::numeric_limits<value_type>::infinity(),
                                                    -std::numeric_limits<value_type>::infinity()};
    for (value_type const invalid : invalid_values)
    {
        for (std::size_t row = 0U; row < 2U; ++row)
        {
            for (std::size_t column = 0U; column < 2U; ++column)
            {
                auto invalid_matrix = identity;
                invalid_matrix(row, column) = invalid;
                expect_failure(test, profile, invalid_matrix, identity, Status::non_finite_input);
                expect_failure(test, profile, identity, invalid_matrix, Status::non_finite_input);
            }
        }
    }

    value_type const largest = std::numeric_limits<value_type>::max();
    // Both singular and indefinite covariance inputs are rejected.
    expect_failure(test, profile, -identity, identity, Status::not_positive_definite);
    constexpr value_type asymmetric_values[4] = {4, 2, 1, 3};
    auto const asymmetric = matrix_type::from_row_major(asymmetric_values);
    expect_failure(test, profile, asymmetric, identity, Status::not_positive_definite);
    auto roundoff_asymmetric = matrix_type::from_row_major(singular_values);
    roundoff_asymmetric(1U, 1U) = value_type{5};
    roundoff_asymmetric(0U, 1U) = std::nextafter(value_type{2}, value_type{3});
    expect_failure(test, profile, roundoff_asymmetric, identity, Status::not_positive_definite);
    // Explicitly document the current strict-symmetry boundary.
    value_type const mean =
        value_type{0.5} * roundoff_asymmetric(0U, 1U) + value_type{0.5} * roundoff_asymmetric(1U, 0U);
    roundoff_asymmetric(0U, 1U) = mean;
    roundoff_asymmetric(1U, 0U) = mean;
    matrix_type output;
    status = solve_spd(roundoff_asymmetric, identity, output);
    test.expect(status == Status::success, profile, "SPD system succeeds after caller-side symmetry cleanup");
    expect_failure(test, profile, identity * value_type{0.5}, identity * largest, Status::non_finite_result);
}

template <typename Linalg> void test_scaling(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using matrix_type = typename Linalg::template matrix_type<2U, 2U>;
    matrix_type const identity = matrix_type::identity();
    constexpr value_type system_values[4] = {4, 1, 1, 3};
    constexpr value_type expected_values[4] = {1, -2, 3, 4};
    auto const base_system = matrix_type::from_row_major(system_values);
    auto const expected = matrix_type::from_row_major(expected_values);
    for (int const exponent : {-40, 0, 40})
    {
        auto const system = base_system * std::ldexp(value_type{1}, exponent);
        auto const rhs = reference_product(system, expected);
        auto const right_rhs = reference_product(expected, system);
        matrix_type output;
        Status status = solve_spd(system, rhs, output);
        test.expect(status == Status::success && matrix_near(output, expected), profile,
                    "left solve supports binary-rescaled coupled SPD systems");
        status = right_solve_spd(right_rhs, system, output);
        test.expect(status == Status::success && matrix_near(output, expected), profile,
                    "right solve supports binary-rescaled coupled SPD systems");
    }

    // No absolute pivot cutoff: a small positive variance is not itself an
    // error. This fixture does not claim general accuracy for ill-conditioned S.
    auto unequal_scales = identity;
    unequal_scales(1U, 1U) = std::numeric_limits<value_type>::epsilon() * value_type{0.25};
    matrix_type output;
    Status status = solve_spd(unequal_scales, unequal_scales, output);
    test.expect(status == Status::success && matrix_near(output, identity), profile,
                "small positive diagonal is accepted without an arbitrary conditioning cutoff");

    std::array<value_type, 2U> const extreme_scales{std::numeric_limits<value_type>::min(),
                                                    std::numeric_limits<value_type>::max()};
    for (value_type const scale : extreme_scales)
    {
        auto const system = identity * scale;
        auto const rhs = identity * (scale * value_type{0.5});
        status = solve_spd(system, rhs, output);
        test.expect(status == Status::success && matrix_near(output, identity * value_type{0.5}), profile,
                    "left solve accepts very small/large well-conditioned systems with representable solutions");
        status = right_solve_spd(rhs, system, output);
        test.expect(status == Status::success && matrix_near(output, identity * value_type{0.5}), profile,
                    "right solve accepts very small/large well-conditioned systems with representable solutions");
    }
}

template <typename Linalg> void test_scalar_precision(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using matrix_type = typename Linalg::template matrix_type<3U, 3U>;
    using rhs_type = typename Linalg::template matrix_type<3U, 2U>;
    // Coupled, strictly diagonally dominant SPD system. The non-dyadic
    // off-diagonal entries expose binary64 factorization narrowed to float.
    value_type const a = value_type{1} / value_type{3};
    value_type const b = value_type{1} / value_type{7};
    auto const system = matrix_type::from_row_major({2, a, b, a, 3, -a, b, -a, 4});
    auto const expected = rhs_type::from_row_major({1, -2, 3, 4, -5, 6});
    auto const rhs = reference_product(system, expected);
    rhs_type output;
    Status const status = solve_spd(system, rhs, output);
    test.expect(status == Status::success && matrix_near(output, expected), profile,
                "Cholesky accumulation preserves the selected scalar precision");
    test.expect(matrix_near(reference_product(system, output), rhs), profile,
                "non-dyadic system satisfies A*X=B at the selected scalar precision");
}

template <typename Linalg> void test_factor_overflow(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using matrix_type = typename Linalg::template matrix_type<3U, 3U>;
    matrix_type system = matrix_type::identity();
    system(0U, 0U) = std::numeric_limits<value_type>::min();
    system(0U, 2U) = std::numeric_limits<value_type>::max();
    system(2U, 0U) = system(0U, 2U);
    // Finite symmetric but indefinite input: elimination overflows, then
    // inf*0 produces NaN. A successful LLT info() alone must not accept it.
    expect_failure(test, profile, system, matrix_type::identity(), Status::non_finite_result);
}

template <typename Linalg> void run_conformance_tests(TestContext & test, std::string_view profile)
{
    test_dimensions<Linalg, 1U, 1U>(test, profile);
    test_dimensions<Linalg, 1U, 3U>(test, profile);
    test_dimensions<Linalg, 2U, 3U>(test, profile);
    test_dimensions<Linalg, 3U, 3U>(test, profile);
    test_dimensions<Linalg, 3U, 15U>(test, profile);
    test_dimensions<Linalg, 6U, 15U>(test, profile);
    test_dimensions<Linalg, 15U, 1U>(test, profile);
    test_aliases<Linalg>(test, profile);
    test_failures<Linalg>(test, profile);
    test_scaling<Linalg>(test, profile);
    test_scalar_precision<Linalg>(test, profile);
    test_factor_overflow<Linalg>(test, profile);
}

} /* end namespace */

int main()
{
    TestContext test;
    formal_eskf::test::configure_backend_test();
    run_conformance_tests<formal_eskf::test::Backend<float>>(test, "binary32");
    run_conformance_tests<formal_eskf::test::Backend<double>>(test, "binary64");
    if (test.failures() != 0)
    {
        std::cerr << test.failures() << " linear solve test(s) failed\n";
        return 1;
    }
    std::cout << "All Cholesky solve conformance tests passed\n";
    return 0;
}
