/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <cstddef>
#include <formal_eskf/linalg/matrix.hpp>

extern void __ESBMC_assert(bool, char const *);
extern float nondet_float();
extern double nondet_double();

#if FORMAL_ESKF_PROOF_BINARY64
using Scalar = double;
#else
using Scalar = float;
#endif

struct FixedArray
{
    using value_type = Scalar;
    template <std::size_t Rows, std::size_t Columns> struct storage_type
    {
        Scalar value[Rows * Columns];
    };

    template <std::size_t Rows, std::size_t Columns> static void set_zero(storage_type<Rows, Columns> & storage)
    {
        for (std::size_t i = 0U; i < Rows; ++i)
        {
            for (std::size_t j = 0U; j < Columns; ++j)
            {
                storage.value[i * Columns + j] = Scalar{0};
            }
        }
    }

    template <std::size_t Rows, std::size_t Columns>
    static Scalar const & coefficient(storage_type<Rows, Columns> const & storage, std::size_t row, std::size_t column)
    {
        __ESBMC_assert(row < Rows && column < Columns, "generated read is in bounds");
        return storage.value[row * Columns + column];
    }

    template <std::size_t Rows, std::size_t Columns>
    static Scalar & coefficient(storage_type<Rows, Columns> & storage, std::size_t row, std::size_t column)
    {
        __ESBMC_assert(row < Rows && column < Columns, "generated write is in bounds");
        return storage.value[row * Columns + column];
    }
};

template <std::size_t Rows, std::size_t Columns>
void symbolic(formal_eskf::linalg::Matrix<FixedArray, Rows, Columns> & matrix)
{
    for (std::size_t row = 0U; row < Rows; ++row)
    {
        for (std::size_t column = 0U; column < Columns; ++column)
        {
#if FORMAL_ESKF_PROOF_BINARY64
            matrix(row, column) = nondet_double();
#else
            matrix(row, column) = nondet_float();
#endif
        }
    }
}
