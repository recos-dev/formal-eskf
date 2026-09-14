/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string_view>
#include <type_traits>
#include <utility>

#include <formal_eskf/linalg/backend/cmsis_dsp.hpp>

#include "test_support.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::linalg::detail::MatrixAccess;

template <typename Matrix> void mark_padding(Matrix & matrix)
{
    auto & storage = MatrixAccess::storage(matrix);
    for (std::size_t index = Matrix::row_count * Matrix::column_count; index < storage.values.size(); ++index)
    {
        storage.values[index] = typename Matrix::value_type{37};
    }
}

template <typename Matrix> [[nodiscard]] bool padding_is(Matrix const & matrix, typename Matrix::value_type expected)
{
    auto const & storage = MatrixAccess::storage(matrix);
    for (std::size_t index = Matrix::row_count * Matrix::column_count; index < storage.values.size(); ++index)
    {
        if (storage.values[index] != expected)
        {
            return false;
        }
    }
    return true;
}

template <typename Linalg, std::size_t Rows, std::size_t Inner, std::size_t Columns>
void test_shape(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using left_type = typename Linalg::template matrix_type<Rows, Inner>;
    using right_type = typename Linalg::template matrix_type<Inner, Columns>;
    using storage_type = typename Linalg::template storage_type<Rows, Inner>;
    static_assert(alignof(storage_type) >= 16U);
    static_assert(storage_type::padding_size * sizeof(value_type) >= 12U);
    static_assert(std::is_trivially_copyable_v<storage_type>);

    left_type left;
    right_type right;
    test.expect(padding_is(left, value_type{0}) && padding_is(right, value_type{0}), profile,
                "owning storage initializes SIMD tail padding");
    for (std::size_t row = 0U; row < Rows; ++row)
    {
        for (std::size_t column = 0U; column < Inner; ++column)
        {
            left(row, column) = static_cast<value_type>(row + 1U) * value_type{0.5} - static_cast<value_type>(column);
        }
    }
    for (std::size_t row = 0U; row < Inner; ++row)
    {
        for (std::size_t column = 0U; column < Columns; ++column)
        {
            right(row, column) = static_cast<value_type>(column + 1U) * value_type{0.25} - static_cast<value_type>(row);
        }
    }
    // Nonzero padding must not participate in the logical matrix operation.
    mark_padding(left);
    mark_padding(right);
    auto const product = left * right;
    for (std::size_t row = 0U; row < Rows; ++row)
    {
        for (std::size_t column = 0U; column < Columns; ++column)
        {
            long double expected = 0;
            for (std::size_t inner = 0U; inner < Inner; ++inner)
            {
                expected += static_cast<long double>(left(row, inner)) * static_cast<long double>(right(inner, column));
            }
            test.expect(product(row, column) == static_cast<value_type>(expected), profile,
                        "rectangular/tail product agrees with independent exact dyadic oracle");
        }
    }

    auto const transposed = formal_eskf::linalg::transpose(left);
    auto const sum = left + left;
    auto const difference = sum - left;
    auto const negative = -left;
    auto const scaled = left * value_type{2};
    auto const divided = scaled / value_type{2};
    for (std::size_t row = 0U; row < Rows; ++row)
    {
        for (std::size_t column = 0U; column < Inner; ++column)
        {
            value_type const entry = left(row, column);
            test.expect(transposed(column, row) == entry && sum(row, column) == entry * value_type{2} &&
                            difference(row, column) == entry && negative(row, column) == -entry &&
                            scaled(row, column) == entry * value_type{2} && divided(row, column) == entry,
                        profile, "elementwise and transpose kernels preserve row-major layout and odd tails");
        }
    }
    test.expect(padding_is(left, value_type{37}) && padding_is(right, value_type{37}), profile,
                "read-only descriptors do not modify source padding");
    test.expect(padding_is(product, value_type{0}) && padding_is(transposed, value_type{0}) &&
                    padding_is(sum, value_type{0}) && padding_is(difference, value_type{0}) &&
                    padding_is(negative, value_type{0}) && padding_is(scaled, value_type{0}) &&
                    padding_is(divided, value_type{0}),
                profile, "kernels write logical output coefficients only");
}

template <typename Linalg, std::size_t Size> void test_vector_tail(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    typename Linalg::template vector_type<Size> vector;
    value_type expected{0};
    for (std::size_t index = 0U; index < Size; ++index)
    {
        vector(index) = static_cast<value_type>(index + 1U);
        expected += vector(index) * vector(index);
    }
    mark_padding(vector);
    test.expect(formal_eskf::linalg::dot(vector, vector) == expected &&
                    formal_eskf::linalg::squared_norm(vector) == expected,
                profile, "dot product excludes tail padding at the selected scalar precision");
    value_type const expected_norm = std::sqrt(expected);
    test.expect(formal_eskf::test::near(formal_eskf::linalg::norm(vector), expected_norm,
                                        value_type{8} * std::numeric_limits<value_type>::epsilon() * expected_norm),
                profile, "norm uses the selected scalar square root");
}

template <typename Linalg> void test_ownership(TestContext & test, std::string_view profile)
{
    using matrix_type = typename Linalg::template matrix_type<3U, 3U>;
    auto source = matrix_type::from_row_major({1, 2, 3, 4, 5, 6, 7, 8, 9});
    auto const copied = source;
    matrix_type assigned;
    assigned = source;
    auto moved = std::move(assigned);
    matrix_type move_assigned;
    move_assigned = std::move(moved);
    source(0U, 0U) = 99;
    auto const identity = matrix_type::identity();
    test.expect(source(0U, 0U) == 99 && (copied * identity)(0U, 0U) == 1 && (move_assigned * identity)(0U, 0U) == 1,
                profile, "copy/move construction and assignment never retain a descriptor to source storage");
    auto const after_source_lifetime = []()
    {
        auto local = matrix_type::from_row_major({1, 2, 3, 4, 5, 6, 7, 8, 9});
        return formal_eskf::linalg::transpose(local);
    }();
    test.expect((after_source_lifetime * identity)(0U, 1U) == 4, profile,
                "returned storage is independent of expired input descriptors");
}

template <typename Linalg> void test_precision_and_division(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using vector_type = typename Linalg::template vector_type<1U>;
    value_type const delta = value_type{8} * std::numeric_limits<value_type>::epsilon();
    auto const one = vector_type::from_row_major({1});
    auto const increment = vector_type::from_row_major({delta});
    auto const precise = one + increment;
    test.expect(precise(0U) == value_type{1} + delta && (precise - one)(0U) == delta &&
                    (one * (value_type{1} + delta))(0U) == value_type{1} + delta &&
                    (precise * one)(0U) == value_type{1} + delta &&
                    formal_eskf::linalg::dot(precise, one) == value_type{1} + delta,
                profile, "binary64 kernels do not silently narrow to binary32");

    // Gradual-underflow profile: a reciprocal would overflow, but a direct
    // quotient is exactly one. This is not a claim for FTZ-enabled targets.
    value_type const tiny = std::numeric_limits<value_type>::min() / value_type{8};
    auto const numerator = vector_type::from_row_major({tiny});
    test.expect((numerator / tiny)(0U) == value_type{1}, profile,
                "coefficient division does not replace division with reciprocal multiplication");
    auto const signed_zero = vector_type::from_row_major({-value_type{0}});
    test.expect(std::signbit((signed_zero / value_type{2})(0U)), profile, "division preserves negative zero");
}

template <typename Scalar> void run_conformance_tests(TestContext & test, std::string_view profile)
{
    using Linalg = formal_eskf::linalg::CmsisDspBackend<Scalar>;
    test_shape<Linalg, 1U, 1U, 1U>(test, profile);
    test_shape<Linalg, 2U, 2U, 2U>(test, profile);
    test_shape<Linalg, 2U, 3U, 4U>(test, profile);
    test_shape<Linalg, 3U, 3U, 3U>(test, profile);
    test_shape<Linalg, 4U, 4U, 4U>(test, profile);
    test_shape<Linalg, 5U, 7U, 3U>(test, profile);
    test_shape<Linalg, 15U, 15U, 15U>(test, profile);
    test_shape<Linalg, 15U, 3U, 15U>(test, profile);
    test_vector_tail<Linalg, 1U>(test, profile);
    test_vector_tail<Linalg, 3U>(test, profile);
    test_vector_tail<Linalg, 4U>(test, profile);
    test_vector_tail<Linalg, 15U>(test, profile);
    test_ownership<Linalg>(test, profile);
    test_precision_and_division<Linalg>(test, profile);
}

} /* end namespace */

int main()
{
    TestContext test;
    run_conformance_tests<float>(test, "binary32");
    run_conformance_tests<double>(test, "binary64");
    if (test.failures() != 0)
    {
        std::cerr << test.failures() << " CMSIS-DSP backend test(s) failed\n";
        return 1;
    }
    std::cout << "All CMSIS-DSP storage and precision tests passed\n";
    return 0;
}
