/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this file.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <array>
#include <bit>
#include <cmath>
#include <limits>
#include <utility>

#include <formal_eskf/linalg/linalg.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{
using formal_eskf::Status;
using formal_eskf::test::Context;
namespace la = formal_eskf::linalg;

template <typename Scalar> bool same_scalar(Scalar a, Scalar b)
{
    using bytes = std::array<std::byte, sizeof(Scalar)>;
    return (std::isnan(a) && std::isnan(b)) || std::bit_cast<bytes>(a) == std::bit_cast<bytes>(b);
}
template <typename Matrix> bool same(Matrix const & a, Matrix const & b)
{
    for (std::size_t row = 0; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0; column < Matrix::column_count; ++column)
        {
            if (!same_scalar(a(row, column), b(row, column)))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename Backend, std::size_t Rows, std::size_t Columns>
void construction_shape(Context & test, std::string_view profile)
{
    using Matrix = la::Matrix<Backend, Rows, Columns>;
    using Scalar = typename Backend::value_type;
    Matrix initial, zero = Matrix::zero();
    Scalar values[Rows * Columns];
    for (std::size_t index = 0; index < Rows * Columns; ++index)
    {
        values[index] = static_cast<Scalar>(index + 1);
    }
    Matrix source = Matrix::from_row_major(values);
    Matrix copy(source), assigned;
    assigned = source;
    Matrix moved(std::move(copy)), move_assigned;
    move_assigned = std::move(assigned);
    source(0, 0) = Scalar{-17};
    moved(Rows - 1, Columns - 1) = Scalar{-23};
    for (std::size_t row = 0; row < Rows; ++row)
    {
        for (std::size_t column = 0; column < Columns; ++column)
        {
            test.expect(initial(row, column) == Scalar{0} && zero(row, column) == Scalar{0} &&
                            move_assigned(row, column) == values[row * Columns + column] &&
                            source(row, column) ==
                                (row == 0 && column == 0 ? Scalar{-17} : values[row * Columns + column]),
                        profile, "all required shapes initialize/index/copy/move owning coefficients");
            test.expect(moved(row, column) ==
                            (row == Rows - 1 && column == Columns - 1 ? Scalar{-23} : values[row * Columns + column]),
                        profile, "copy/move result does not borrow subsequently modified source storage");
        }
    }
    if constexpr (Rows == Columns)
    {
        auto const identity = Matrix::identity();
        for (std::size_t row = 0; row < Rows; ++row)
        {
            for (std::size_t column = 0; column < Columns; ++column)
            {
                test.expect(identity(row, column) == (row == column ? Scalar{1} : Scalar{0}), profile,
                            "identity diagonal and every off-diagonal coefficient");
            }
        }
    }
}

template <typename B> void construction(Context & test, std::string_view profile)
{
    construction_shape<B, 1, 1>(test, profile);
    construction_shape<B, 2, 1>(test, profile);
    construction_shape<B, 3, 1>(test, profile);
    construction_shape<B, 4, 1>(test, profile);
    construction_shape<B, 6, 1>(test, profile);
    construction_shape<B, 15, 1>(test, profile);
    construction_shape<B, 16, 1>(test, profile);
    construction_shape<B, 1, 3>(test, profile);
    construction_shape<B, 1, 15>(test, profile);
    construction_shape<B, 2, 3>(test, profile);
    construction_shape<B, 3, 2>(test, profile);
    construction_shape<B, 6, 3>(test, profile);
    construction_shape<B, 3, 6>(test, profile);
    construction_shape<B, 2, 2>(test, profile);
    construction_shape<B, 2, 15>(test, profile);
    construction_shape<B, 2, 16>(test, profile);
    construction_shape<B, 15, 2>(test, profile);
    construction_shape<B, 3, 3>(test, profile);
    construction_shape<B, 3, 15>(test, profile);
    construction_shape<B, 3, 16>(test, profile);
    construction_shape<B, 15, 3>(test, profile);
    construction_shape<B, 4, 3>(test, profile);
    construction_shape<B, 4, 4>(test, profile);
    construction_shape<B, 4, 15>(test, profile);
    construction_shape<B, 4, 16>(test, profile);
    construction_shape<B, 15, 4>(test, profile);
    construction_shape<B, 6, 6>(test, profile);
    construction_shape<B, 6, 15>(test, profile);
    construction_shape<B, 6, 16>(test, profile);
    construction_shape<B, 15, 6>(test, profile);
    construction_shape<B, 12, 12>(test, profile);
    construction_shape<B, 12, 15>(test, profile);
    construction_shape<B, 15, 12>(test, profile);
    construction_shape<B, 15, 15>(test, profile);
    construction_shape<B, 16, 15>(test, profile);
}

template <typename B, std::size_t Row, std::size_t Column> void block_access(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    using Matrix = la::Matrix<B, 4, 5>;
    Matrix matrix;
    for (std::size_t row = 0; row < 4; ++row)
    {
        for (std::size_t column = 0; column < 5; ++column)
        {
            matrix(row, column) = static_cast<Scalar>(row * 5 + column + 1);
        }
    }
    Matrix const & view = matrix;
    auto block = view.template block<Row, Column, 2, 2>();
    for (std::size_t row = 0; row < 2; ++row)
    {
        for (std::size_t column = 0; column < 2; ++column)
        {
            test.expect(block(row, column) == static_cast<Scalar>((Row + row) * 5 + Column + column + 1), profile,
                        "block extracts independent row-major offset");
            block(row, column) = -static_cast<Scalar>(row * 2 + column + 1);
        }
    }
    test.expect(view(Row, Column) == static_cast<Scalar>(Row * 5 + Column + 1), profile, "extraction is owning");
    matrix.template set_block<Row, Column>(block);
    auto const saved = matrix;
    matrix.template set_block<0, 0>(matrix);
    test.expect(same(matrix, saved), profile, "full-object block self assignment");
    for (std::size_t row = 0; row < 4; ++row)
    {
        for (std::size_t column = 0; column < 5; ++column)
        {
            bool const inside = row >= Row && row < Row + 2 && column >= Column && column < Column + 2;
            Scalar const expected = inside ? -static_cast<Scalar>((row - Row) * 2 + column - Column + 1)
                                           : static_cast<Scalar>(row * 5 + column + 1);
            test.expect(matrix(row, column) == expected, profile, "block write changes only its intended rectangle");
        }
    }
}

template <typename B, std::size_t Offset> void segment_access(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    auto vector = la::Vector<B, 6>::from_row_major({1, 2, 3, 4, 5, 6});
    auto const & view = vector;
    auto part = view.template segment<Offset, 2>();
    test.expect(part(0) == static_cast<Scalar>(Offset + 1) && part(1) == static_cast<Scalar>(Offset + 2), profile,
                "segment extracts independent offset");
    part(0) = Scalar{-7};
    part(1) = Scalar{-8};
    test.expect(vector(Offset) == static_cast<Scalar>(Offset + 1), profile, "segment owns its data");
    vector.template set_segment<Offset>(part);
    auto const saved = vector;
    vector.template set_segment<0>(vector);
    test.expect(same(vector, saved), profile, "full vector segment self assignment");
    for (std::size_t axis = 0; axis < 6; ++axis)
    {
        Scalar const expected =
            axis == Offset ? Scalar{-7} : (axis == Offset + 1 ? Scalar{-8} : static_cast<Scalar>(axis + 1));
        test.expect(view(axis) == expected && view(axis, 0) == expected, profile,
                    "segment write frame and indexing agreement");
    }
}

template <typename B> void access(Context & test, std::string_view profile)
{
    block_access<B, 0, 0>(test, profile);
    block_access<B, 1, 2>(test, profile);
    block_access<B, 2, 3>(test, profile);
    segment_access<B, 0>(test, profile);
    segment_access<B, 2>(test, profile);
    segment_access<B, 4>(test, profile);
}

template <typename B> void elementwise(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    using Matrix = la::Matrix<B, 2, 3>;
    Matrix a = Matrix::from_row_major({1, -2, 3, -4, 5, -6});
    Matrix b = Matrix::from_row_major({7, 8, -9, 10, -11, 12});
    auto const a_before = a, b_before = b;
    for (Scalar factor : {Scalar{0}, Scalar{-1}, Scalar{2}, Scalar{0.5}})
    {
        auto const plus = a + b, minus = a - b, negative = -a, left = factor * a, right = a * factor;
        auto const nested = (a + b) - b;
        for (std::size_t row = 0; row < 2; ++row)
        {
            for (std::size_t column = 0; column < 3; ++column)
            {
                test.expect(plus(row, column) == a(row, column) + b(row, column) &&
                                minus(row, column) == a(row, column) - b(row, column) &&
                                negative(row, column) == -a(row, column) &&
                                left(row, column) == factor * a(row, column) &&
                                right(row, column) == factor * a(row, column) && nested(row, column) == a(row, column),
                            profile, "independent exact coefficient arithmetic");
            }
        }
    }
    auto eager = a + b;
    a(0, 0) = Scalar{-100};
    b(1, 2) = Scalar{-200};
    test.expect(eager(0, 0) == Scalar{8} && eager(1, 2) == Scalar{6}, profile, "expression result owns values eagerly");
    a = a_before;
    b = b_before;
    a = a + b;
    for (std::size_t row = 0; row < 2; ++row)
    {
        for (std::size_t column = 0; column < 3; ++column)
        {
            test.expect(a(row, column) == a_before(row, column) + b_before(row, column), profile,
                        "assign over operand");
        }
    }
    test.expect(same(b, b_before), profile, "unassigned operand stays unchanged");
    for (Scalar divisor : {Scalar{0.5}, Scalar{2}, std::numeric_limits<Scalar>::min() / Scalar{8}})
    {
        auto const input = Matrix::from_row_major({divisor, -divisor, 0, 2 * divisor, -2 * divisor, 0});
        auto const output = input / divisor;
        auto const expected = Matrix::from_row_major({1, -1, 0, 2, -2, 0});
        test.expect(same(output, expected), profile, "division avoids overflowing an unnecessary reciprocal");
    }
}

template <typename B> void product(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    auto const a = la::Matrix<B, 2, 3>::from_row_major({1, 2, 3, 4, 5, 6});
    auto const b = la::Matrix<B, 3, 2>::from_row_major({1, -1, 2, 0, 0, 3});
    auto const result = a * b;
    Scalar const expected[2][2]{{5, 8}, {14, 14}};
    for (std::size_t row = 0; row < 2; ++row)
    {
        for (std::size_t column = 0; column < 2; ++column)
        {
            test.expect(result(row, column) == expected[row][column], profile,
                        "independent nonsymmetric rectangular product");
        }
    }
    auto const vector = la::Vector<B, 3>::from_row_major({1, -2, 3});
    auto const action = a * vector;
    test.expect(action(0) == Scalar{6} && action(1) == Scalar{12}, profile,
                "independent rectangular matrix-vector product");
    auto transposed = la::transpose(a);
    for (std::size_t row = 0; row < 2; ++row)
    {
        for (std::size_t column = 0; column < 3; ++column)
        {
            test.expect(transposed(column, row) == a(row, column), profile,
                        "transpose exchanges mathematical coordinates");
        }
    }
    transposed(0, 0) = Scalar{-20};
    test.expect(a(0, 0) == Scalar{1}, profile, "transpose is owning");
    for (std::size_t inner = 0; inner < 12; ++inner)
    {
        la::Matrix<B, 15, 12> left;
        la::Matrix<B, 12, 15> right;
        for (std::size_t index = 0; index < 15; ++index)
        {
            left(index, inner) = static_cast<Scalar>(index + 1);
            right(inner, index) = -static_cast<Scalar>(index + 2);
        }
        auto const reverse = right * left;
        for (std::size_t row = 0; row < 12; ++row)
        {
            for (std::size_t column = 0; column < 12; ++column)
            {
                test.expect(reverse(row, column) == (row == inner && column == inner ? Scalar{-1360} : Scalar{0}),
                            profile, "reverse 12x15 by 15x12 product includes every inner contribution");
            }
        }
        auto const outer = left * right;
        for (std::size_t row = 0; row < 15; ++row)
        {
            for (std::size_t column = 0; column < 15; ++column)
            {
                test.expect(outer(row, column) == -static_cast<Scalar>((row + 1) * (column + 2)), profile,
                            "each inner coordinate contributes once in 15x12 by 12x15 product");
            }
        }
    }
}

template <typename B> void reductions(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    auto const v = la::Vector<B, 3>::from_row_major({3, 4, 0}), w = la::Vector<B, 3>::from_row_major({2, -1, 5});
    test.expect(la::dot(v, w) == Scalar{2} && la::squared_norm(v) == Scalar{25} && la::norm(v) == Scalar{5}, profile,
                "independent exact dot and Pythagorean norm");
    auto const zero = la::Vector<B, 3>::zero();
    test.expect(la::dot(v, zero) == Scalar{0} && la::squared_norm(zero) == Scalar{0} && la::norm(zero) == Scalar{0},
                profile, "zero reductions");
    auto large = zero;
    large(0) = std::ldexp(Scalar{1}, std::numeric_limits<Scalar>::max_exponent / 4);
    test.expect(la::norm(large) == large(0) && la::squared_norm(large) == large(0) * large(0), profile,
                "large exactly representable square/root remains finite");
    large(0) = std::numeric_limits<Scalar>::max();
    test.expect(!std::isfinite(la::squared_norm(large)), profile,
                "raw unrepresentable square is nonfinite; no checked Status is promised");
    la::Matrix<B, 15, 15> matrix;
    for (std::size_t row = 0; row < 15; ++row)
    {
        for (std::size_t column = 0; column < 15; ++column)
        {
            matrix(row, column) = row == column ? static_cast<Scalar>(row + 1) : Scalar{-1024};
        }
    }
    auto const diagonal = la::diagonal(matrix);
    test.expect(la::trace(matrix) == Scalar{120}, profile, "trace ignores every off-diagonal decoy");
    for (std::size_t row = 0; row < 15; ++row)
    {
        test.expect(diagonal(row) == static_cast<Scalar>(row + 1), profile, "diagonal extraction exact coordinates");
    }
    la::Vector<B, 15> a, b;
    for (std::size_t axis = 0; axis < 15; ++axis)
    {
        a(axis) = static_cast<Scalar>(axis + 1) / Scalar{4};
        b(axis) = axis % 2 == 0 ? Scalar{2} : Scalar{-2};
    }
    test.expect(la::dot(a, b) == Scalar{4} && la::squared_norm(a) == Scalar{77.5}, profile,
                "full-length signed reduction with independent integer sums");
}

template <typename B> void cross_product(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    using Vector = la::Vector<B, 3>;
    std::array const a{Vector::from_row_major({1, 0, 0}), Vector::from_row_major({0, 1, 0}),
                       Vector::from_row_major({2, -1, 3}), Vector{}, Vector::from_row_major({1, 2, 3})};
    std::array const b{Vector::from_row_major({0, 1, 0}), Vector::from_row_major({1, 0, 0}),
                       Vector::from_row_major({-2, 4, 1}), Vector::from_row_major({1, 2, 3}),
                       Vector::from_row_major({2, 4, 6})};
    Scalar const expected[5][3]{{0, 0, 1}, {0, 0, -1}, {-13, -8, 6}, {0, 0, 0}, {0, 0, 0}};
    for (std::size_t index = 0; index < a.size(); ++index)
    {
        auto const result = la::cross(a[index], b[index]);
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            test.expect(result(axis) == expected[index][axis], profile,
                        "independent determinant/right-handed cross coefficients");
        }
    }
}

template <typename B> void finite(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    using Matrix = la::Matrix<B, 2, 3>;
    auto const good = Matrix::from_row_major({1, -2, 3, -9, 5, -6});
    test.expect(la::all_finite(good) && la::max_abs(good) == Scalar{9} && la::max_abs(Matrix{}) == Scalar{0}, profile,
                "independent finite magnitude maximum");
    Matrix zeros{};
    zeros(0, 1) = -Scalar{0};
    auto const zero_max = la::max_abs(zeros);
    test.expect(zero_max == Scalar{0} && !std::signbit(zero_max), profile,
                "maximum magnitude of signed zeros is positive zero");
    for (std::size_t index = 0; index < 6; ++index)
    {
        for (Scalar invalid : {std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity(),
                               -std::numeric_limits<Scalar>::infinity()})
        {
            auto input = good;
            input(index / 3, index % 3) = invalid;
            auto const saved = input;
            auto const magnitude = la::max_abs(input);
            bool const classified =
                std::isnan(invalid) ? std::isnan(magnitude) : magnitude == std::numeric_limits<Scalar>::infinity();
            test.expect(!la::all_finite(input) && classified && same(input, saved), profile,
                        "NaN remains NaN and either infinity has positive infinite magnitude; input preserved");
        }
    }
    for (bool nan_first : {false, true})
    {
        auto mixed = good;
        mixed(0, 0) = nan_first ? std::numeric_limits<Scalar>::quiet_NaN() : -std::numeric_limits<Scalar>::infinity();
        mixed(1, 2) = nan_first ? -std::numeric_limits<Scalar>::infinity() : std::numeric_limits<Scalar>::quiet_NaN();
        auto const magnitude = la::max_abs(mixed);
        test.expect(!la::all_finite(mixed) &&
                        (std::isnan(magnitude) || magnitude == std::numeric_limits<Scalar>::infinity()),
                    profile, "mixed faults allow NaN or positive infinity without an ordering guarantee");
    }
}

template <typename B> void symmetry(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    using Matrix = la::Matrix<B, 2, 2>;
    auto const matrix = Matrix::from_row_major({1, 4, 2, 3});
    auto const symmetric = la::symmetrize(matrix);
    auto const expected = Matrix::from_row_major({1, 3, 3, 3});
    test.expect(same(symmetric, expected), profile, "independent exact average of each mirrored pair");
    auto near = Matrix::from_row_major({1, 1, 1.5, 2});
    for (Scalar tolerance :
         {std::nextafter(Scalar{0.5}, Scalar{0}), Scalar{0.5}, std::nextafter(Scalar{0.5}, Scalar{1})})
    {
        test.expect(la::is_symmetric(near, tolerance) == (tolerance >= Scalar{0.5}), profile,
                    "inclusive encoded symmetry residual threshold");
    }
    for (Scalar invalid :
         {Scalar{-1}, std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity()})
    {
        test.expect(!la::is_symmetric(expected, invalid), profile, "invalid tolerance is not symmetry evidence");
    }
    near(0, 1) = Scalar{0};
    near(1, 0) = -Scalar{0};
    test.expect(la::is_symmetric(near, Scalar{0}) && la::is_symmetric(near, -Scalar{0}), profile,
                "signed zero pair is symmetric with either zero tolerance");
    for (std::size_t slot = 0; slot < 4; ++slot)
    {
        for (Scalar invalid : {std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity(),
                               -std::numeric_limits<Scalar>::infinity()})
        {
            auto bad = expected;
            bad(slot / 2, slot % 2) = invalid;
            test.expect(!la::is_symmetric(bad, Scalar{1}), profile, "nonfinite matrix cannot establish symmetry");
        }
    }
    near(0, 1) = std::numeric_limits<Scalar>::max();
    near(1, 0) = -std::numeric_limits<Scalar>::max();
    test.expect(!la::is_symmetric(near, std::numeric_limits<Scalar>::max()), profile,
                "overflowed residual is not symmetric");
}

template <typename B> void sandwich(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    auto const transform = la::Matrix<B, 2, 3>::from_row_major({1, -2, 3, 0, 1, -1});
    for (bool symmetric : {false, true})
    {
        auto const matrix = symmetric ? la::Matrix<B, 3, 3>::from_row_major({4, 1, 2, 1, 5, -1, 2, -1, 6})
                                      : la::Matrix<B, 3, 3>::from_row_major({4, 1, 2, -3, 5, -1, 7, 8, 6});
        auto const result = la::sandwich(transform, matrix);
        for (std::size_t row = 0; row < 2; ++row)
        {
            for (std::size_t column = 0; column < 2; ++column)
            {
                Scalar expected{};
                for (std::size_t left = 0; left < 3; ++left)
                {
                    for (std::size_t right = 0; right < 3; ++right)
                    {
                        expected += transform(row, left) * matrix(left, right) * transform(column, right);
                    }
                }
                test.expect(result(row, column) == expected, profile,
                            "independent four-index sum preserves correlated/general matrix contributions");
            }
        }
    }
}

template <typename B> void normalization(Context & test, std::string_view profile)
{
    using Scalar = typename B::value_type;
    using Vector = la::Vector<B, 3>;
    auto const input = Vector::from_row_major({3, 4, 0});
    for (Scalar minimum : {std::nextafter(Scalar{5}, Scalar{0}), Scalar{5}, std::nextafter(Scalar{5}, Scalar{6})})
    {
        for (bool alias : {false, true})
        {
            auto value = input;
            Vector separate = Vector::from_row_major({7, -8, 9});
            auto & output = alias ? value : separate;
            auto const before = output;
            auto const status = la::normalize(value, minimum, output);
            if (minimum <= Scalar{5})
            {
                test.expect(status == Status::success && output(0) == Scalar{3} / Scalar{5} &&
                                output(1) == Scalar{4} / Scalar{5} && output(2) == Scalar{0},
                            profile,
                            "exact norm and independently rounded rational coefficients at inclusive threshold");
            }
            else
            {
                test.expect(status == Status::zero_or_unsafe_divisor && same(output, before), profile,
                            "next threshold rejects atomically");
            }
            test.expect(alias || same(value, input), profile, "disjoint normalization input remains unchanged");
        }
    }
    auto fail = [&](Vector const & source, Scalar minimum, Status expected)
    {
        for (bool alias : {false, true})
        {
            auto value = source;
            Vector separate = Vector::from_row_major({-Scalar{0}, std::numeric_limits<Scalar>::quiet_NaN(), 9});
            auto & output = alias ? value : separate;
            auto const before = output;
            test.expect(la::normalize(value, minimum, output) == expected && same(output, before) &&
                            (alias || same(value, source)),
                        profile, "candidate normalization binding preserves failure/alias frame");
        }
    };
    fail(Vector{}, Scalar{1}, Status::zero_or_unsafe_divisor);
    for (Scalar threshold : {Scalar{0}, Scalar{-1}, -Scalar{0}})
    {
        fail(input, threshold, Status::zero_or_unsafe_divisor);
    }
    for (Scalar invalid : {std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity(),
                           -std::numeric_limits<Scalar>::infinity()})
    {
        fail(input, invalid, Status::non_finite_input);
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            auto bad = input;
            bad(axis) = invalid;
            fail(bad, Scalar{0}, Status::non_finite_input);
        }
    }
    auto const large =
        Vector::from_row_major({std::numeric_limits<Scalar>::max(), std::numeric_limits<Scalar>::max(), 0});
    fail(large, Scalar{1}, Status::non_finite_result);
    fail(large, Scalar{0}, Status::non_finite_result);
}

template <typename Base> struct ControlledNorm : Base
{
    using Scalar = typename Base::value_type;
    template <std::size_t Rows, std::size_t Columns>
    using Storage = typename Base::template storage_type<Rows, Columns>;
    static inline Scalar norm_result{}, division_result{}, forwarded_divisor{};
    static inline unsigned norm_calls{}, division_calls{};
    template <std::size_t Size> static Scalar norm(Storage<Size, 1> const &)
    {
        ++norm_calls;
        return norm_result;
    }
    template <std::size_t Rows, std::size_t Columns>
    static void divide(Storage<Rows, Columns> const & input, Scalar divisor, Storage<Rows, Columns> & output)
    {
        ++division_calls;
        forwarded_divisor = divisor;
        Base::template divide<Rows, Columns>(input, divisor, output);
        Base::template coefficient<Rows, Columns>(output, Rows - 1, Columns - 1) = division_result;
    }
};

template <typename Base> void normalization_result(Context & test, std::string_view profile)
{
    using B = ControlledNorm<Base>;
    using Scalar = typename B::value_type;
    using Vector = la::Vector<B, 3>;
    auto const input = Vector::from_row_major({3, 4, 0});
    for (Scalar norm : {Scalar{0}, Scalar{0.5}, std::numeric_limits<Scalar>::quiet_NaN(),
                        std::numeric_limits<Scalar>::infinity(), -std::numeric_limits<Scalar>::infinity()})
    {
        Vector output = Vector::from_row_major({-Scalar{0}, 8, 9});
        auto const before = output;
        B::norm_result = norm;
        B::norm_calls = B::division_calls = 0;
        test.expect(la::normalize(input, Scalar{1}, output) ==
                            (std::isfinite(norm) ? Status::zero_or_unsafe_divisor : Status::non_finite_result) &&
                        same(output, before) && B::norm_calls == 1 && B::division_calls == 0,
                    profile, "controlled norm result fails before divide without publication");
    }
    for (Scalar invalid : {std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity(),
                           -std::numeric_limits<Scalar>::infinity()})
    {
        for (bool alias : {false, true})
        {
            B::norm_result = Scalar{5};
            B::division_result = invalid;
            B::norm_calls = B::division_calls = 0;
            auto value = input;
            Vector separate = Vector::from_row_major({-Scalar{0}, 8, 9});
            auto & output = alias ? value : separate;
            auto const before = output;
            test.expect(la::normalize(value, Scalar{1}, output) == Status::non_finite_result && same(output, before) &&
                            B::division_calls == 1 && B::forwarded_divisor == Scalar{5},
                        profile, "controlled last coefficient division failure is atomic including in-place output");
        }
    }
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using F = formal_eskf::test::Backend<float>;
    using D = formal_eskf::test::Backend<double>;
    using formal_eskf::test::Case;
    Case const cases[]{
        {"TC-F-LINALG-CONSTRUCTION", "binary32", construction<F>},
        {"TC-F-LINALG-CONSTRUCTION", "binary64", construction<D>},
        {"TC-F-LINALG-ACCESS", "binary32", access<F>},
        {"TC-F-LINALG-ACCESS", "binary64", access<D>},
        {"TC-F-LINALG-ELEMENTWISE", "binary32", elementwise<F>},
        {"TC-F-LINALG-ELEMENTWISE", "binary64", elementwise<D>},
        {"TC-F-LINALG-PRODUCT", "binary32", product<F>},
        {"TC-F-LINALG-PRODUCT", "binary64", product<D>},
        {"TC-F-LINALG-REDUCTIONS", "binary32", reductions<F>},
        {"TC-F-LINALG-REDUCTIONS", "binary64", reductions<D>},
        {"TC-F-LINALG-CROSS", "binary32", cross_product<F>},
        {"TC-F-LINALG-CROSS", "binary64", cross_product<D>},
        {"TC-F-LINALG-FINITE", "candidate.binary32", finite<F>},
        {"TC-F-LINALG-FINITE", "candidate.binary64", finite<D>},
        {"TC-F-LINALG-SYMMETRY", "candidate.binary32", symmetry<F>},
        {"TC-F-LINALG-SYMMETRY", "candidate.binary64", symmetry<D>},
        {"TC-F-LINALG-SANDWICH", "binary32", sandwich<F>},
        {"TC-F-LINALG-SANDWICH", "binary64", sandwich<D>},
        {"TC-F-LINALG-NORMALIZE", "candidate.binary32", normalization<F>},
        {"TC-F-LINALG-NORMALIZE", "candidate.binary64", normalization<D>},
        {"TC-F-LINALG-NORMALIZE-RESULT", "controlled.binary32", normalization_result<F>},
        {"TC-F-LINALG-NORMALIZE-RESULT", "controlled.binary64", normalization_result<D>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
