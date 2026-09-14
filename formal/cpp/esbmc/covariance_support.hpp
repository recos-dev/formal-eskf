/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include "contracts.hpp"

#ifndef FORMAL_ESKF_PROOF_PRODUCT_ENTRY_CONTRACT
#define FORMAL_ESKF_PROOF_PRODUCT_ENTRY_CONTRACT 0
#endif
#ifndef FORMAL_ESKF_PROOF_DOT_CONTRACT
#define FORMAL_ESKF_PROOF_DOT_CONTRACT 0
#endif
#ifndef FORMAL_ESKF_PROOF_ACCESS_CONTRACT
#define FORMAL_ESKF_PROOF_ACCESS_CONTRACT 0
#endif

namespace formal_eskf::verification
{

// Actual storage for 15x15 covariance proofs. Named scalar fields let ESBMC
// slice individual output entries without encoding every other floating-point
// product in a native SMT array. Two levels avoid a 225-deep aggregate type.
// Explicit scalar copies avoid bytewise array-copy modeling. Arithmetic and
// all existing smaller shapes are unchanged; no storage summary is assumed.
template <typename Cell> struct CovarianceRow
{
    Cell c0{};
    Cell c1{};
    Cell c2{};
    Cell c3{};
    Cell c4{};
    Cell c5{};
    Cell c6{};
    Cell c7{};
    Cell c8{};
    Cell c9{};
    Cell c10{};
    Cell c11{};
    Cell c12{};
    Cell c13{};
    Cell c14{};

    CovarianceRow() = default;
    CovarianceRow(CovarianceRow const & other)
        : c0(other.c0), c1(other.c1), c2(other.c2), c3(other.c3), c4(other.c4), c5(other.c5), c6(other.c6),
          c7(other.c7), c8(other.c8), c9(other.c9), c10(other.c10), c11(other.c11), c12(other.c12), c13(other.c13),
          c14(other.c14)
    {
    }
    CovarianceRow & operator=(CovarianceRow const & other)
    {
        c0 = other.c0;
        c1 = other.c1;
        c2 = other.c2;
        c3 = other.c3;
        c4 = other.c4;
        c5 = other.c5;
        c6 = other.c6;
        c7 = other.c7;
        c8 = other.c8;
        c9 = other.c9;
        c10 = other.c10;
        c11 = other.c11;
        c12 = other.c12;
        c13 = other.c13;
        c14 = other.c14;
        return *this;
    }

    template <typename Row> static auto & at(Row & row, std::size_t index)
    {
        __ESBMC_assert(index < 15U, "covariance row index is in bounds");
        switch (index)
        {
        case 0U:
            return row.c0;
        case 1U:
            return row.c1;
        case 2U:
            return row.c2;
        case 3U:
            return row.c3;
        case 4U:
            return row.c4;
        case 5U:
            return row.c5;
        case 6U:
            return row.c6;
        case 7U:
            return row.c7;
        case 8U:
            return row.c8;
        case 9U:
            return row.c9;
        case 10U:
            return row.c10;
        case 11U:
            return row.c11;
        case 12U:
            return row.c12;
        case 13U:
            return row.c13;
        default:
            return row.c14;
        }
    }
    Cell & operator[](std::size_t index) { return at(*this, index); }
    Cell const & operator[](std::size_t index) const { return at(*this, index); }
};

template <typename Number> struct CovarianceCells;

#if FORMAL_ESKF_PROOF_ACCESS_CONTRACT
// The actual row-major accessor is discharged by verify_reset_storage. Here
// arbitrary row/column values stand for its 30 reads, without encoding 450
// unrelated matrix cells into every symbolic-coordinate selection.
struct CovarianceAccessContract
{
    inline static CovarianceCells<contract_proof::Scalar> const * left = nullptr;
    inline static CovarianceCells<contract_proof::Scalar> const * right = nullptr;
    inline static contract_proof::Linalg::vector_type<15U> row_values{}, column_values{};
    inline static std::size_t row = 0U, column = 0U, calls = 0U;
};
#endif

template <typename Number> struct CovarianceCells
{
    CovarianceRow<CovarianceRow<Number>> rows;

    Number & operator[](std::size_t index)
    {
        __ESBMC_assert(index < 225U, "covariance storage index is in bounds");
        return rows[index / 15U][index % 15U];
    }
    Number const & operator[](std::size_t index) const
    {
        __ESBMC_assert(index < 225U, "covariance storage index is in bounds");
#if FORMAL_ESKF_PROOF_ACCESS_CONTRACT
        using Contract = CovarianceAccessContract;
        __ESBMC_assert(Contract::calls < 30U, "E-RESET access contract: exactly 30 row/column reads");
        auto const k = Contract::calls / 2U;
        if (Contract::calls++ % 2U == 0U)
        {
            __ESBMC_assert(this == Contract::left && index == Contract::row * 15U + k,
                           "E-RESET access contract: requested left row in ascending inner-index order");
            return linalg::detail::MatrixAccess::storage(Contract::row_values).values[k];
        }
        __ESBMC_assert(this == Contract::right && index == k * 15U + Contract::column,
                       "E-RESET access contract: requested right column in ascending inner-index order");
        return linalg::detail::MatrixAccess::storage(Contract::column_values).values[k];
#else
        return rows[index / 15U][index % 15U];
#endif
    }
};

template <>
template <>
struct FixedArrayLinalg<contract_proof::Scalar, NoNormObserver<contract_proof::Scalar>,
                        contract_proof::OpaqueMath>::storage_type<15U, 15U>
{
    CovarianceCells<contract_proof::Scalar> values;
};

#if FORMAL_ESKF_PROOF_PRODUCT_ENTRY_CONTRACT
struct CovarianceEntryContract
{
    inline static CovarianceCells<contract_proof::Scalar> const * left = nullptr;
    inline static CovarianceCells<contract_proof::Scalar> const * right = nullptr;
    inline static CovarianceCells<contract_proof::Scalar> const * result = nullptr;
    inline static std::size_t calls = 0U;
};
#endif

#if FORMAL_ESKF_PROOF_DOT_CONTRACT
struct CovarianceDotContract
{
    inline static contract_proof::Linalg::vector_type<15U> left{}, right{};
    inline static contract_proof::Scalar result{};
    inline static unsigned calls = 0U;
};
#endif

} // namespace formal_eskf::verification

#if FORMAL_ESKF_PROOF_DOT_CONTRACT
namespace formal_eskf::linalg
{
template <>
inline contract_proof::Scalar dot(contract_proof::Linalg::vector_type<15U> const & left,
                                  contract_proof::Linalg::vector_type<15U> const & right) noexcept
{
    using verification::CovarianceDotContract;
    __ESBMC_assert(CovarianceDotContract::calls == 0U, "E-RESET dot contract: exactly one ordered dot product");
    ++CovarianceDotContract::calls;
    CovarianceDotContract::left = left;
    CovarianceDotContract::right = right;
    return CovarianceDotContract::result;
}
} // namespace formal_eskf::linalg
#endif

namespace formal_eskf::verification
{

// The same ordered reduction as FixedArrayLinalg::multiply. Isolating a single
// entry permits a compositional proof instead of encoding 3,375 floating-point
// products every time a complete covariance matrix is used.
inline contract_proof::Scalar covariance_product_entry(CovarianceCells<contract_proof::Scalar> const & left,
                                                       CovarianceCells<contract_proof::Scalar> const & right,
                                                       std::size_t row, std::size_t column)
{
    __ESBMC_assert(row < 15U && column < 15U, "covariance entry indices are in bounds");
#if FORMAL_ESKF_PROOF_PRODUCT_ENTRY_CONTRACT
    __ESBMC_assert(&left == CovarianceEntryContract::left && &right == CovarianceEntryContract::right &&
                       row * 15U + column == CovarianceEntryContract::calls,
                   "E-RESET entry contract: original operands and every row-major coordinate exactly once");
    ++CovarianceEntryContract::calls;
    return (*CovarianceEntryContract::result)[row * 15U + column];
#else
    contract_proof::Linalg::vector_type<15U> row_vector, column_vector;
    for (std::size_t k = 0U; k < 15U; ++k)
    {
        row_vector.set(k, left[row * 15U + k]);
        column_vector.set(k, right[k * 15U + column]);
    }
    return linalg::dot(row_vector, column_vector);
#endif
}

template <>
template <>
inline void
FixedArrayLinalg<contract_proof::Scalar, NoNormObserver<contract_proof::Scalar>,
                 contract_proof::OpaqueMath>::multiply<15U, 15U, 15U>(storage_type<15U, 15U> const & left,
                                                                      storage_type<15U, 15U> const & right,
                                                                      storage_type<15U, 15U> & result) noexcept
{
    for (std::size_t row = 0U; row < 15U; ++row)
    {
        for (std::size_t column = 0U; column < 15U; ++column)
        {
            result.values[row * 15U + column] = covariance_product_entry(left.values, right.values, row, column);
        }
    }
    NoNormObserver<contract_proof::Scalar>::product<15U, 15U, 15U>(left.values, right.values, result.values);
}

} // namespace formal_eskf::verification
