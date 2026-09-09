#pragma once

/**
 * @file
 * Fixed-size matrix value type shared by all linear algebra backends.
 */

#include <cstddef>

namespace formal_eskf::linalg
{

namespace detail
{
class MatrixAccess;
} /* end namespace detail */

/**
 * Fixed-size matrix whose storage and primitive operations are supplied by a
 * compile-time linear algebra backend.
 *
 * All arithmetic operators evaluate eagerly and return an owning value.
 * Coefficient access uses zero-based indices and requires valid bounds.
 */
template <typename Linalg, std::size_t Rows, std::size_t Columns> class Matrix
{

    static_assert(Rows > 0U);
    static_assert(Columns > 0U);

public:
    using value_type = typename Linalg::value_type;
    using linalg_type = Linalg;

    static constexpr std::size_t row_count = Rows;
    static constexpr std::size_t column_count = Columns;

    Matrix() noexcept { Linalg::template set_zero<Rows, Columns>(m_storage); }
    Matrix(Matrix const &) = default;
    Matrix(Matrix &&) noexcept = default;
    Matrix & operator=(Matrix const &) = default;
    Matrix & operator=(Matrix &&) noexcept = default;
    ~Matrix() = default;

    /** Return a zero matrix. */
    [[nodiscard]] static Matrix zero() noexcept { return Matrix(); }

    /** Return an identity matrix.  This function is available only for square matrices. */
    [[nodiscard]] static Matrix identity() noexcept;

    /** Construct a matrix from coefficients in row-major mathematical order. */
    [[nodiscard]] static Matrix from_row_major(value_type const (&coefficients)[Rows * Columns]) noexcept;

    /** Access a coefficient.  The caller must prove row < Rows and column < Columns. */
    [[nodiscard]] value_type const & operator()(std::size_t row, std::size_t column) const noexcept;
    [[nodiscard]] value_type & operator()(std::size_t row, std::size_t column) noexcept;

    /** Access a vector coefficient.  This overload is available only when Columns is one. */
    [[nodiscard]] value_type const & operator()(std::size_t index) const noexcept;
    [[nodiscard]] value_type & operator()(std::size_t index) noexcept;

    [[nodiscard]] Matrix operator+(Matrix const & other) const noexcept;
    [[nodiscard]] Matrix operator-(Matrix const & other) const noexcept;
    [[nodiscard]] Matrix operator-() const noexcept;
    [[nodiscard]] Matrix operator*(value_type scalar) const noexcept;
    [[nodiscard]] Matrix operator/(value_type scalar) const noexcept;

    template <std::size_t OtherColumns>
    [[nodiscard]] Matrix<Linalg, Rows, OtherColumns>
    operator*(Matrix<Linalg, Columns, OtherColumns> const & other) const noexcept;

    /** Return a compile-time-positioned block. */
    template <std::size_t StartRow, std::size_t StartColumn, std::size_t BlockRows, std::size_t BlockColumns>
    [[nodiscard]] Matrix<Linalg, BlockRows, BlockColumns> block() const noexcept;

    /** Replace a compile-time-positioned block.  The block dimensions are deduced. */
    template <std::size_t StartRow, std::size_t StartColumn, std::size_t BlockRows, std::size_t BlockColumns>
    void set_block(Matrix<Linalg, BlockRows, BlockColumns> const & block) noexcept;

    /** Return a compile-time-positioned vector segment. */
    template <std::size_t Offset, std::size_t SegmentSize>
    [[nodiscard]] Matrix<Linalg, SegmentSize, 1U> segment() const noexcept;

    /** Replace a compile-time-positioned vector segment.  Its size is deduced. */
    template <std::size_t Offset, std::size_t SegmentSize>
    void set_segment(Matrix<Linalg, SegmentSize, 1U> const & segment) noexcept;

private:
    using storage_type = typename Linalg::template storage_type<Rows, Columns>;

    storage_type m_storage;

    template <typename, std::size_t, std::size_t> friend class Matrix;
    friend class detail::MatrixAccess;

}; /* end class Matrix */

namespace detail
{

class MatrixAccess
{
public:
    template <typename Linalg, std::size_t Rows, std::size_t Columns>
    [[nodiscard]] static auto const & storage(Matrix<Linalg, Rows, Columns> const & matrix) noexcept
    {
        return matrix.m_storage;
    }

    template <typename Linalg, std::size_t Rows, std::size_t Columns>
    [[nodiscard]] static auto & storage(Matrix<Linalg, Rows, Columns> & matrix) noexcept
    {
        return matrix.m_storage;
    }
}; /* end class MatrixAccess */

} /* end namespace detail */

template <typename Linalg, std::size_t Rows, std::size_t Columns>
Matrix<Linalg, Rows, Columns> Matrix<Linalg, Rows, Columns>::identity() noexcept
{
    static_assert(Rows == Columns);
    Matrix result;
    for (std::size_t index = 0U; index < Rows; ++index)
    {
        result(index, index) = value_type{1};
    }
    return result;
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
Matrix<Linalg, Rows, Columns>
Matrix<Linalg, Rows, Columns>::from_row_major(value_type const (&coefficients)[Rows * Columns]) noexcept
{
    Matrix result;
    for (std::size_t row = 0U; row < Rows; ++row)
    {
        for (std::size_t column = 0U; column < Columns; ++column)
        {
            result(row, column) = coefficients[row * Columns + column];
        }
    }
    return result;
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
typename Matrix<Linalg, Rows, Columns>::value_type const &
Matrix<Linalg, Rows, Columns>::operator()(std::size_t row, std::size_t column) const noexcept
{
    return Linalg::template coefficient<Rows, Columns>(m_storage, row, column);
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
typename Matrix<Linalg, Rows, Columns>::value_type &
Matrix<Linalg, Rows, Columns>::operator()(std::size_t row, std::size_t column) noexcept
{
    return Linalg::template coefficient<Rows, Columns>(m_storage, row, column);
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
typename Matrix<Linalg, Rows, Columns>::value_type const &
Matrix<Linalg, Rows, Columns>::operator()(std::size_t index) const noexcept
{
    static_assert(Columns == 1U);
    return (*this)(index, 0U);
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
typename Matrix<Linalg, Rows, Columns>::value_type &
Matrix<Linalg, Rows, Columns>::operator()(std::size_t index) noexcept
{
    static_assert(Columns == 1U);
    return (*this)(index, 0U);
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
Matrix<Linalg, Rows, Columns> Matrix<Linalg, Rows, Columns>::operator+(Matrix const & other) const noexcept
{
    Matrix result;
    Linalg::template add<Rows, Columns>(m_storage, other.m_storage, result.m_storage);
    return result;
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
Matrix<Linalg, Rows, Columns> Matrix<Linalg, Rows, Columns>::operator-(Matrix const & other) const noexcept
{
    Matrix result;
    Linalg::template subtract<Rows, Columns>(m_storage, other.m_storage, result.m_storage);
    return result;
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
Matrix<Linalg, Rows, Columns> Matrix<Linalg, Rows, Columns>::operator-() const noexcept
{
    Matrix result;
    Linalg::template negate<Rows, Columns>(m_storage, result.m_storage);
    return result;
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
Matrix<Linalg, Rows, Columns> Matrix<Linalg, Rows, Columns>::operator*(value_type scalar) const noexcept
{
    Matrix result;
    Linalg::template scale<Rows, Columns>(m_storage, scalar, result.m_storage);
    return result;
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
Matrix<Linalg, Rows, Columns> Matrix<Linalg, Rows, Columns>::operator/(value_type scalar) const noexcept
{
    Matrix result;
    Linalg::template divide<Rows, Columns>(m_storage, scalar, result.m_storage);
    return result;
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
template <std::size_t OtherColumns>
Matrix<Linalg, Rows, OtherColumns>
Matrix<Linalg, Rows, Columns>::operator*(Matrix<Linalg, Columns, OtherColumns> const & other) const noexcept
{
    Matrix<Linalg, Rows, OtherColumns> result;
    Linalg::template multiply<Rows, Columns, OtherColumns>(m_storage, other.m_storage, result.m_storage);
    return result;
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
template <std::size_t StartRow, std::size_t StartColumn, std::size_t BlockRows, std::size_t BlockColumns>
Matrix<Linalg, BlockRows, BlockColumns> Matrix<Linalg, Rows, Columns>::block() const noexcept
{
    static_assert(StartRow + BlockRows <= Rows);
    static_assert(StartColumn + BlockColumns <= Columns);
    Matrix<Linalg, BlockRows, BlockColumns> result;
    for (std::size_t row = 0U; row < BlockRows; ++row)
    {
        for (std::size_t column = 0U; column < BlockColumns; ++column)
        {
            result(row, column) = (*this)(StartRow + row, StartColumn + column);
        }
    }
    return result;
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
template <std::size_t StartRow, std::size_t StartColumn, std::size_t BlockRows, std::size_t BlockColumns>
void Matrix<Linalg, Rows, Columns>::set_block(Matrix<Linalg, BlockRows, BlockColumns> const & block) noexcept
{
    static_assert(StartRow + BlockRows <= Rows);
    static_assert(StartColumn + BlockColumns <= Columns);
    for (std::size_t row = 0U; row < BlockRows; ++row)
    {
        for (std::size_t column = 0U; column < BlockColumns; ++column)
        {
            (*this)(StartRow + row, StartColumn + column) = block(row, column);
        }
    }
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
template <std::size_t Offset, std::size_t SegmentSize>
Matrix<Linalg, SegmentSize, 1U> Matrix<Linalg, Rows, Columns>::segment() const noexcept
{
    static_assert(Columns == 1U);
    static_assert(Offset + SegmentSize <= Rows);
    Matrix<Linalg, SegmentSize, 1U> result;
    for (std::size_t index = 0U; index < SegmentSize; ++index)
    {
        result(index) = (*this)(Offset + index);
    }
    return result;
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
template <std::size_t Offset, std::size_t SegmentSize>
void Matrix<Linalg, Rows, Columns>::set_segment(Matrix<Linalg, SegmentSize, 1U> const & segment) noexcept
{
    static_assert(Columns == 1U);
    static_assert(Offset + SegmentSize <= Rows);
    for (std::size_t index = 0U; index < SegmentSize; ++index)
    {
        (*this)(Offset + index) = segment(index);
    }
}

template <typename Linalg, std::size_t Rows, std::size_t Columns>
[[nodiscard]] Matrix<Linalg, Rows, Columns> operator*(typename Matrix<Linalg, Rows, Columns>::value_type scalar,
                                                      Matrix<Linalg, Rows, Columns> const & matrix) noexcept
{
    return matrix * scalar;
}

} /* end namespace formal_eskf::linalg */
