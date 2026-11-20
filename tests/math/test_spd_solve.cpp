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

#include "test_cases.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using Context = TestContext;
using formal_eskf::Status;
using formal_eskf::linalg::right_solve_spd;
using formal_eskf::linalg::solve_spd;

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

// Supplemental legacy regression envelope, not the acceptance bound for general
// conditioning or rounding. Exact known-solution cases below do not use it.
template <typename Matrix> bool matrix_near(Matrix const & actual, Matrix const & expected)
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
template <typename Left, typename Right> auto reference_product(Left const & a, Right const & b)
{
    static_assert(Left::column_count == Right::row_count);
    using linalg_type = typename Left::linalg_type;
    using value_type = typename Left::value_type;
    formal_eskf::linalg::Matrix<linalg_type, Left::row_count, Right::column_count> result;
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

template <typename Matrix> bool exact(Matrix const & a, Matrix const & b)
{
    for (std::size_t row = 0; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0; column < Matrix::column_count; ++column)
        {
            if (a(row, column) != b(row, column))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename B, std::size_t Size, std::size_t Columns> struct KnownFixture
{
    using Scalar = typename B::value_type;
    using System = formal_eskf::linalg::Matrix<B, Size, Size>;
    using Solution = formal_eskf::linalg::Matrix<B, Size, Columns>;
    System system;
    Solution solution;
    KnownFixture()
    {
        // L has diagonal 2 and lower triangle 1/2. All operations in S=L*L^T,
        // B=S*X and the Cholesky solve of these small dyadic values are exact.
        for (std::size_t row = 0; row < Size; ++row)
        {
            for (std::size_t column = 0; column < Size; ++column)
            {
                Scalar coefficient{};
                for (std::size_t inner = 0; inner < Size; ++inner)
                {
                    Scalar const left = inner > row ? Scalar{0} : (inner == row ? Scalar{2} : Scalar{0.5});
                    Scalar const right = inner > column ? Scalar{0} : (inner == column ? Scalar{2} : Scalar{0.5});
                    coefficient += left * right;
                }
                system(row, column) = coefficient;
            }
            for (std::size_t column = 0; column < Columns; ++column)
            {
                solution(row, column) =
                    static_cast<Scalar>(row + 1) / Scalar{2} - static_cast<Scalar>(column) / Scalar{4};
            }
        }
    }
};

template <typename B, std::size_t Size, std::size_t Columns> void known_left(Context & test, std::string_view profile)
{
    KnownFixture<B, Size, Columns> fixture;
    auto const rhs = reference_product(fixture.system, fixture.solution);
    auto const saved = fixture.system;
    auto const before_rhs = rhs;
    typename KnownFixture<B, Size, Columns>::Solution output;
    test.expect(solve_spd(fixture.system, rhs, output) == Status::success && exact(output, fixture.solution) &&
                    exact(reference_product(fixture.system, output), rhs),
                profile, "independent dyadic L*L^T system recovers exact multiple-RHS solution/residual");
    test.expect(same_bits(fixture.system, saved) && same_bits(rhs, before_rhs), profile,
                "solver inputs remain unchanged");
    typename KnownFixture<B, Size, Columns>::Solution zero;
    test.expect(solve_spd(fixture.system, zero, output) == Status::success && exact(output, zero), profile,
                "validated SPD system with zero RHS publishes zero");
}

template <typename B> void known(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    using Matrix = formal_eskf::linalg::Matrix<B, 2, 2>;
    auto const system = Matrix::from_row_major({4, 2, 2, 2});
    auto const rhs = Matrix::from_row_major({8, 2, 6, 4});
    auto const expected = Matrix::from_row_major({1, -1, 2, 3});
    Matrix output;
    test.expect(solve_spd(system, rhs, output) == Status::success && exact(output, expected), profile,
                "hand-solved correlated 2x2 system with two RHS columns");
    formal_eskf::linalg::Vector<B, 2> single;
    auto const b = formal_eskf::linalg::Vector<B, 2>::from_row_major({8, 6});
    test.expect(solve_spd(system, b, single) == Status::success && single(0) == Scalar{1} && single(1) == Scalar{2},
                profile, "single RHS uses the same equation");
    known_left<B, 1, 1>(test, profile);
    known_left<B, 1, 3>(test, profile);
    known_left<B, 1, 15>(test, profile);
    known_left<B, 2, 2>(test, profile);
    known_left<B, 2, 3>(test, profile);
    known_left<B, 2, 15>(test, profile);
    known_left<B, 3, 3>(test, profile);
    known_left<B, 3, 15>(test, profile);
    known_left<B, 6, 15>(test, profile);
    known_left<B, 15, 1>(test, profile);
}

template <typename B, std::size_t Rows, std::size_t Size> void known_right(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    using Matrix = formal_eskf::linalg::Matrix<B, Rows, Size>;
    KnownFixture<B, Size, 1> fixture;
    Matrix expected;
    for (std::size_t row = 0; row < Rows; ++row)
    {
        for (std::size_t column = 0; column < Size; ++column)
        {
            expected(row, column) = static_cast<Scalar>(row + 1) - static_cast<Scalar>(column + 1) / Scalar{4};
        }
    }
    auto const rhs = reference_product(expected, fixture.system);
    auto const saved_rhs = rhs;
    auto const saved_system = fixture.system;
    Matrix output;
    test.expect(right_solve_spd(rhs, fixture.system, output) == Status::success && exact(output, expected) &&
                    exact(reference_product(output, fixture.system), rhs),
                profile, "independent nonsquare right-system equation X*S=B");
    test.expect(same_bits(rhs, saved_rhs) && same_bits(fixture.system, saved_system), profile,
                "right solve does not modify borrowed system or RHS");
}

template <typename B> void right(Context & test, std::string_view profile)
{
    known_right<B, 3, 1>(test, profile);
    known_right<B, 15, 2>(test, profile);
    known_right<B, 2, 3>(test, profile);
    known_right<B, 15, 3>(test, profile);
    known_right<B, 15, 6>(test, profile);
    known_right<B, 1, 15>(test, profile);
}

template <typename Matrix>
void expect_failure(TestContext & test, std::string_view profile, Matrix const & system, Matrix const & rhs,
                    Status expected_status, bool aliases = false)
{
    using Scalar = typename Matrix::value_type;
    Matrix sentinel = Matrix::identity();
    sentinel(0, 0) = std::numeric_limits<Scalar>::quiet_NaN();
    sentinel(0, 1) = -Scalar{0};
    for (bool right_side : {false, true})
    {
        for (unsigned arrangement = 0; arrangement < (aliases ? 3U : 1U); ++arrangement)
        {
            auto a = system, b = rhs, separate = sentinel;
            auto & output = arrangement == 1 ? a : (arrangement == 2 ? b : separate);
            auto const before = output;
            auto const status = right_side ? right_solve_spd(b, a, output) : solve_spd(a, b, output);
            test.expect(status == expected_status && same_bits(output, before) && same_bits(a, system) &&
                            same_bits(b, rhs),
                        profile, "failed left/right solve preserves complete outputs and borrowed inputs");
        }
    }
}

template <typename B> void domain(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    using Matrix = formal_eskf::linalg::Matrix<B, 2, 2>;
    auto const identity = Matrix::identity();
    Matrix const zero;
    expect_failure(test, profile, zero, identity, Status::not_positive_definite);
    expect_failure(test, profile, zero, zero, Status::not_positive_definite);
    expect_failure(test, profile, -identity, zero, Status::not_positive_definite);
    expect_failure(test, profile, Matrix::from_row_major({1, 2, 2, 4}), identity, Status::not_positive_definite);
    expect_failure(test, profile, Matrix::from_row_major({1, 2, 2, 1}), identity, Status::not_positive_definite);
    for (std::size_t slot = 0; slot < 4; ++slot)
    {
        for (Scalar invalid : {std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity(),
                               -std::numeric_limits<Scalar>::infinity()})
        {
            auto bad = identity;
            bad(slot / 2, slot % 2) = invalid;
            expect_failure(test, profile, bad, identity, Status::non_finite_input);
            expect_failure(test, profile, identity, bad, Status::non_finite_input);
        }
    }
    auto asymmetric = Matrix::from_row_major({4, 2, 2, 2});
    asymmetric(0, 1) = std::nextafter(Scalar{2}, Scalar{3});
    expect_failure(test, profile, asymmetric, identity, Status::not_positive_definite);
    auto bad_rhs = identity;
    bad_rhs(1, 1) = std::numeric_limits<Scalar>::quiet_NaN();
    expect_failure(test, profile, asymmetric, bad_rhs, Status::non_finite_input);
    auto signed_zero = identity;
    signed_zero(1, 0) = -Scalar{0};
    Matrix output;
    test.expect(solve_spd(signed_zero, identity, output) == Status::success && exact(output, identity), profile,
                "opposite signed-zero off-diagonals compare symmetric");
}

template <typename B> void aliases(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    using Matrix = formal_eskf::linalg::Matrix<B, 2, 2>;
    auto const system = Matrix::from_row_major({4, 2, 2, 2});
    auto const expected = Matrix::from_row_major({1, -1, 2, 3});
    for (bool right_side : {false, true})
    {
        auto const rhs = right_side ? reference_product(expected, system) : reference_product(system, expected);
        for (unsigned arrangement = 0; arrangement < 3; ++arrangement)
        {
            auto a = system, b = rhs, separate = -Matrix::identity();
            auto & output = arrangement == 1 ? a : (arrangement == 2 ? b : separate);
            auto const status = right_side ? right_solve_spd(b, a, output) : solve_spd(a, b, output);
            test.expect(status == Status::success && exact(output, expected) &&
                            (arrangement == 1 || same_bits(a, system)) && (arrangement == 2 || same_bits(b, rhs)),
                        profile, "successful disjoint/system/RHS alias publishes the independent exact solution");
        }
        auto shared = system;
        auto status = right_side ? right_solve_spd(shared, shared, shared) : solve_spd(shared, shared, shared);
        test.expect(status == Status::success && exact(shared, Matrix::identity()), profile,
                    "all three operands may alias");
        shared = Matrix::from_row_major({1, 2, 2, 4});
        auto const before = shared;
        status = right_side ? right_solve_spd(shared, shared, shared) : solve_spd(shared, shared, shared);
        test.expect(status == Status::not_positive_definite && same_bits(shared, before), profile,
                    "failed all-shared solve preserves input");
    }
    Matrix invalid = system;
    invalid(1, 1) = std::numeric_limits<Scalar>::quiet_NaN();
    expect_failure(test, profile, invalid, system, Status::non_finite_input, true);
    expect_failure(test, profile, Matrix::from_row_major({1, 2, 2, 1}), system, Status::not_positive_definite, true);
    expect_failure(test, profile, Matrix::identity() * Scalar{0.5},
                   Matrix::identity() * std::numeric_limits<Scalar>::max(), Status::non_finite_result, true);
    KnownFixture<B, 3, 2> rectangle;
    auto const rhs = reference_product(rectangle.system, rectangle.solution);
    auto output = rhs;
    test.expect(solve_spd(rectangle.system, output, output) == Status::success && exact(output, rectangle.solution),
                profile, "rectangular left RHS alias");
    auto const right_expected = transpose(rectangle.solution);
    auto right_output = reference_product(right_expected, rectangle.system);
    test.expect(right_solve_spd(right_output, rectangle.system, right_output) == Status::success &&
                    exact(right_output, right_expected),
                profile, "rectangular right RHS alias");
}

template <typename Base> struct ControlledSolve : Base
{
    using Scalar = typename Base::value_type;
    template <std::size_t R, std::size_t C> using Storage = typename Base::template storage_type<R, C>;
    static inline Status status{};
    static inline Scalar candidate_value{};
    static inline unsigned calls{};
    template <std::size_t Size, std::size_t Columns>
    static Status solve_spd(Storage<Size, Size> const &, Storage<Size, Columns> const &,
                            Storage<Size, Columns> & output)
    {
        ++calls;
        Base::template set_zero<Size, Columns>(output);
        Base::template coefficient<Size, Columns>(output, Size - 1, Columns - 1) = candidate_value;
        return status;
    }
};

template <typename B> void result_failure(Context & test, std::string_view profile)
{
    using Backend = ControlledSolve<B>;
    using Scalar = typename Backend::value_type;
    using Matrix = formal_eskf::linalg::Matrix<Backend, 2, 2>;
    auto const system = Matrix::identity();
    Backend::candidate_value = Scalar{17};
    for (auto status : {Status::non_finite_input, Status::domain_error, Status::out_of_range,
                        Status::zero_or_unsafe_divisor, Status::invalid_quaternion_norm, Status::not_positive_definite,
                        Status::ill_conditioned, Status::non_finite_result})
    {
        Backend::status = status;
        Backend::calls = 0;
        expect_failure(test, profile, system, system, status, true);
        test.expect(Backend::calls == 6, profile,
                    "one backend invocation per left/right call; internal factor count is source evidence");
    }
    Backend::status = Status::success;
    for (Scalar invalid : {std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity(),
                           -std::numeric_limits<Scalar>::infinity()})
    {
        Backend::candidate_value = invalid;
        expect_failure(test, profile, system, system, Status::non_finite_result, true);
    }
    Backend::calls = 0;
    Backend::candidate_value = Scalar{17};
    auto invalid_system = system;
    invalid_system(0, 1) = Scalar{1};
    expect_failure(test, profile, invalid_system, system, Status::not_positive_definite);
    test.expect(Backend::calls == 0, profile, "frontend rejects asymmetry without invoking a solve backend");
    // Actual backend witness, distinct from the controlled producer checks.
    using Actual = formal_eskf::linalg::Matrix<B, 2, 2>;
    expect_failure(test, profile, Actual::identity() * Scalar{0.5},
                   Actual::identity() * std::numeric_limits<Scalar>::max(), Status::non_finite_result);
}
template <typename Linalg> void test_scaling(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using matrix_type = formal_eskf::linalg::Matrix<Linalg, 2U, 2U>;
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
    using matrix_type = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    using rhs_type = formal_eskf::linalg::Matrix<Linalg, 3U, 2U>;
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
    using matrix_type = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    matrix_type system = matrix_type::identity();
    system(0U, 0U) = std::numeric_limits<value_type>::min();
    system(0U, 2U) = std::numeric_limits<value_type>::max();
    system(2U, 0U) = system(0U, 2U);
    // Finite symmetric but indefinite input: elimination overflows, then
    // inf*0 produces NaN. A successful LLT info() alone must not accept it.
    expect_failure(test, profile, system, matrix_type::identity(), Status::non_finite_result);
}

template <typename B> void conditioning(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    using Matrix = formal_eskf::linalg::Matrix<B, 2, 2>;
    auto const expected = Matrix::from_row_major({1, -2, 3, 4});
    for (int exponent : {0, 4, 12, 24, 48})
    {
        auto system = Matrix::identity();
        system(1, 1) = std::ldexp(Scalar{1}, -exponent);
        auto const rhs = reference_product(system, expected);
        Matrix output;
        auto const status = solve_spd(system, rhs, output);
        // Exact eigenvalues are 1 and 2^-exponent, hence kappa_2=2^exponent.
        // The dyadic diagonal factor/solutions are exact, even at high condition.
        test.expect(
            status == Status::success && exact(output, expected) && exact(reference_product(system, output), rhs),
            profile, "recorded diagonal spectrum has zero forward/backward error and no artificial condition cutoff");
        long double forward = 0, residual = 0, system_norm = 0, rhs_norm = 0, output_norm = 0;
        for (std::size_t row = 0; row < 2; ++row)
        {
            long double system_row = 0, rhs_row = 0, output_row = 0, residual_row = 0;
            for (std::size_t column = 0; column < 2; ++column)
            {
                forward =
                    std::max(forward, std::abs(static_cast<long double>(output(row, column)) - expected(row, column)));
                long double value = -static_cast<long double>(rhs(row, column));
                for (std::size_t inner = 0; inner < 2; ++inner)
                {
                    value += static_cast<long double>(system(row, inner)) * output(inner, column);
                }
                residual_row += std::abs(value);
                system_row += std::abs(static_cast<long double>(system(row, column)));
                rhs_row += std::abs(static_cast<long double>(rhs(row, column)));
                output_row += std::abs(static_cast<long double>(output(row, column)));
            }
            residual = std::max(residual, residual_row);
            system_norm = std::max(system_norm, system_row);
            rhs_norm = std::max(rhs_norm, rhs_row);
            output_norm = std::max(output_norm, output_row);
        }
        std::cout << "DIAGONAL kappa2=" << std::ldexp(1.0L, exponent) << " represented_lambda_min=" << std::hexfloat
                  << system(1, 1) << std::defaultfloat << " max_forward_error=" << forward
                  << " normalized_residual=" << residual / (system_norm * output_norm + rhs_norm) << '\n';
    }
    // Retain distinct scale/precision regressions without upgrading their
    // historical envelope into the still-unresolved general conditioning bound.
    test_scaling<B>(test, profile);
    test_scalar_precision<B>(test, profile);
    test_factor_overflow<B>(test, profile);
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using F = formal_eskf::test::Backend<float>;
    using D = formal_eskf::test::Backend<double>;
    using formal_eskf::test::Case;
    Case const cases[]{
        {"TC-F-SOLVE-KNOWN", "binary32", known<F>},
        {"TC-F-SOLVE-KNOWN", "binary64", known<D>},
        {"TC-F-SOLVE-RIGHT", "binary32", right<F>},
        {"TC-F-SOLVE-RIGHT", "binary64", right<D>},
        {"TC-F-SOLVE-DOMAIN", "binary32", domain<F>},
        {"TC-F-SOLVE-DOMAIN", "binary64", domain<D>},
        {"TC-F-SOLVE-ALIASES", "binary32", aliases<F>},
        {"TC-F-SOLVE-ALIASES", "binary64", aliases<D>},
        {"TC-F-SOLVE-RESULT", "controlled+native.binary32", result_failure<F>},
        {"TC-F-SOLVE-RESULT", "controlled+native.binary64", result_failure<D>},
        {"TC-F-SOLVE-CONDITIONING", "partial.binary32", conditioning<F>},
        {"TC-F-SOLVE-CONDITIONING", "partial.binary64", conditioning<D>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
