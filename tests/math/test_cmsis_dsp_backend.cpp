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
#include <formal_eskf/linalg/linalg.hpp>

#include "test_cases.hpp"

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

template <typename Matrix> bool padding_is(Matrix const & matrix, typename Matrix::value_type expected)
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
void test_product_shape(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using left_type = formal_eskf::linalg::Matrix<Linalg, Rows, Inner>;
    using right_type = formal_eskf::linalg::Matrix<Linalg, Inner, Columns>;
    using storage_type = typename Linalg::template storage_type<Rows, Inner>;
    static_assert(alignof(storage_type) >= 16U);
    static_assert(storage_type::padding_size * sizeof(value_type) >= 12U);

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

    // Padding is private adapter storage, not part of the public coefficient
    // write contract. This probe establishes exclusion from logical products;
    // target SIMD overread/write safety requires its instrumented target build.
}

template <typename Linalg, std::size_t Size> void test_vector_tail(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    formal_eskf::linalg::Vector<Linalg, Size> vector;
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
    // The integer dot sum above is exact. This checks adapter wiring to the
    // selected scalar root; independent elementary accuracy belongs to F-SCALAR.
    test.expect(formal_eskf::linalg::norm(vector) == expected_norm, profile,
                "adapter routes the exact logical dot to the selected scalar square root");
}

template <typename Linalg> void test_ownership(TestContext & test, std::string_view profile)
{
    using matrix_type = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
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
    using vector_type = formal_eskf::linalg::Vector<Linalg, 1U>;
    value_type const delta = value_type{8} * std::numeric_limits<value_type>::epsilon();
    auto const one = vector_type::from_row_major({1});
    auto const increment = vector_type::from_row_major({delta});
    auto const precise = one + increment;
    test.expect(precise(0U) == value_type{1} + delta && (precise - one)(0U) == delta &&
                    (one * (value_type{1} + delta))(0U) == value_type{1} + delta,
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

template <typename Linalg, std::size_t Rows, std::size_t Columns>
void test_elementwise_shape(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    using Matrix = formal_eskf::linalg::Matrix<Linalg, Rows, Columns>;
    Matrix input;
    for (std::size_t row = 0; row < Rows; ++row)
    {
        for (std::size_t column = 0; column < Columns; ++column)
        {
            input(row, column) = static_cast<Scalar>(row + 1) * Scalar{0.5} - static_cast<Scalar>(column);
        }
    }
    mark_padding(input);
    auto const transposed = formal_eskf::linalg::transpose(input), sum = input + input, difference = sum - input;
    auto const negative = -input, scaled = input * Scalar{2}, divided = scaled / Scalar{2};
    for (std::size_t row = 0; row < Rows; ++row)
    {
        for (std::size_t column = 0; column < Columns; ++column)
        {
            Scalar const expected = static_cast<Scalar>(row + 1) * Scalar{0.5} - static_cast<Scalar>(column);
            test.expect(input(row, column) == expected && transposed(column, row) == expected &&
                            sum(row, column) == 2 * expected && difference(row, column) == expected &&
                            negative(row, column) == -expected && scaled(row, column) == 2 * expected &&
                            divided(row, column) == expected,
                        profile, "odd-tail arithmetic/transpose excludes private padding and preserves inputs");
        }
    }
}

template <typename Scalar> void construction(TestContext & test, std::string_view profile)
{
    using B = formal_eskf::linalg::CmsisDspBackend<Scalar>;
    using Storage = typename B::template storage_type<3, 3>;
    static_assert(alignof(Storage) >= 16);
    static_assert(Storage::padding_size * sizeof(Scalar) >= 3 * sizeof(std::uint32_t));
    // Dependency-specific guard allocation, not a general Matrix ABI promise.
    formal_eskf::linalg::Matrix<B, 3, 3> matrix;
    test.expect(padding_is(matrix, Scalar{0}), profile, "CMSIS read guard lies in initialized owned storage");
    test_ownership<B>(test, profile);
}

template <typename Scalar> void products(TestContext & test, std::string_view profile)
{
    using B = formal_eskf::linalg::CmsisDspBackend<Scalar>;
    test_product_shape<B, 1, 1, 1>(test, profile);
    test_product_shape<B, 2, 2, 2>(test, profile);
    test_product_shape<B, 2, 3, 4>(test, profile);
    test_product_shape<B, 3, 3, 3>(test, profile);
    test_product_shape<B, 4, 4, 4>(test, profile);
    test_product_shape<B, 5, 7, 3>(test, profile);
    test_product_shape<B, 15, 15, 15>(test, profile);
    test_product_shape<B, 15, 3, 15>(test, profile);
    using Matrix = formal_eskf::linalg::Matrix<B, 1, 1>;
    Scalar const precise = Scalar{1} + Scalar{8} * std::numeric_limits<Scalar>::epsilon();
    auto const a = Matrix::from_row_major({precise}), one = Matrix::from_row_major({1});
    test.expect((a * one)(0, 0) == precise, profile, "product dispatch retains selected binary precision");
}

template <typename Scalar> void arithmetic(TestContext & test, std::string_view profile)
{
    using B = formal_eskf::linalg::CmsisDspBackend<Scalar>;
    test_elementwise_shape<B, 1, 1>(test, profile);
    test_elementwise_shape<B, 2, 3>(test, profile);
    test_elementwise_shape<B, 3, 3>(test, profile);
    test_elementwise_shape<B, 4, 4>(test, profile);
    test_elementwise_shape<B, 5, 7>(test, profile);
    test_elementwise_shape<B, 15, 15>(test, profile);
    test_precision_and_division<B>(test, profile);
}

template <typename Scalar> void reductions(TestContext & test, std::string_view profile)
{
    using B = formal_eskf::linalg::CmsisDspBackend<Scalar>;
    test_vector_tail<B, 1>(test, profile);
    test_vector_tail<B, 3>(test, profile);
    test_vector_tail<B, 4>(test, profile);
    test_vector_tail<B, 15>(test, profile);
    using Vector = formal_eskf::linalg::Vector<B, 1>;
    Scalar const precise = Scalar{1} + Scalar{8} * std::numeric_limits<Scalar>::epsilon();
    auto const a = Vector::from_row_major({precise}), one = Vector::from_row_major({1});
    test.expect(formal_eskf::linalg::dot(a, one) == precise, profile, "dot kernel retains selected binary precision");
}
} // namespace

int main(int argc, char ** argv)
{
    using formal_eskf::test::Case;
    std::cout << "Backend: CMSIS-DSP adapter host supplement; target SIMD qualification separate\n";
    Case const cases[]{
        {"TC-F-LINALG-CONSTRUCTION", "cmsis.guard.binary32", construction<float>},
        {"TC-F-LINALG-CONSTRUCTION", "cmsis.guard.binary64", construction<double>},
        {"TC-F-LINALG-PRODUCT", "cmsis.tail.binary32", products<float>},
        {"TC-F-LINALG-PRODUCT", "cmsis.tail.binary64", products<double>},
        {"TC-F-LINALG-ELEMENTWISE", "cmsis.tail.binary32", arithmetic<float>},
        {"TC-F-LINALG-ELEMENTWISE", "cmsis.tail.binary64", arithmetic<double>},
        {"TC-F-LINALG-REDUCTIONS", "cmsis.tail.binary32", reductions<float>},
        {"TC-F-LINALG-REDUCTIONS", "cmsis.tail.binary64", reductions<double>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
