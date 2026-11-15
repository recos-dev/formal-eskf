/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include "contracts.hpp"

#include <formal_eskf/linalg/backend/cholesky.hpp>

#ifndef FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT
#define FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT 0
#endif

#ifndef FORMAL_ESKF_PROOF_SIZE
#define FORMAL_ESKF_PROOF_SIZE 3
#endif
#ifndef FORMAL_ESKF_PROOF_COLUMNS
#define FORMAL_ESKF_PROOF_COLUMNS 3
#endif
#ifndef FORMAL_ESKF_PROOF_COLUMN
#define FORMAL_ESKF_PROOF_COLUMN 0
#endif

namespace solve_proof
{
using contract_proof::same;
using contract_proof::Scalar;
using formal_eskf::Status;
using formal_eskf::verification::ScalarArray;
constexpr std::size_t size = FORMAL_ESKF_PROOF_SIZE;
constexpr std::size_t columns = FORMAL_ESKF_PROOF_COLUMNS;
static_assert(size >= 1U && size <= 3U);
static_assert(columns == size || columns == 3U || columns == 15U);

// Root accuracy is outside this source-level claim. All IEEE root returns are
// admitted, including non-finite and non-positive values. Check call eligibility
// and diagonal publication. The coefficient observer separately checks the
// ordered recurrences; neither claim treats an opaque root as a real square root.
struct Math : formal_eskf::scalar::StandardMath<Scalar>
{
    // Pivot eligibility is a passive F-SOLVE observer, not a general libm
    // precondition. Field geometry also takes roots of zero. Keep the same
    // unconstrained return sequence while checking those producers separately.
    inline static bool observe_pivots = true;
#if FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT
    static bool is_finite(Scalar value);
    static void record_root(Scalar input, Scalar result, unsigned index);
#endif

    struct Roots
    {
        ScalarArray<Scalar, 3U> results;
        unsigned calls;
        Scalar argument;
        Scalar operator()(Scalar value)
        {
            __ESBMC_assert(calls < (observe_pivots ? size : 3U), "scalar root: bounded return sequence");
            if (observe_pivots)
            {
                __ESBMC_assert(std::isfinite(value) && value > Scalar{0},
                               "F-SOLVE factor: scalar root receives a finite positive pivot");
            }
            Scalar const result = results[calls];
            argument = value;
#if FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT
            Math::record_root(value, result, calls);
#endif
            ++calls;
            return result;
        }
    };
    inline static Roots sqrt{};

    struct Angles
    {
        ScalarArray<Scalar, 3U> results;
        unsigned calls;
        Scalar y, x;
        Scalar operator()(Scalar input_y, Scalar input_x)
        {
            if (observe_pivots)
            {
                return formal_eskf::scalar::StandardMath<Scalar>::atan2(input_y, input_x);
            }
            __ESBMC_assert(calls < 3U, "scalar angle: bounded return sequence");
            y = input_y;
            x = input_x;
            return results[calls++];
        }
    };
    inline static Angles atan2{};
};
class Backend
    : public formal_eskf::verification::FixedArrayLinalg<Scalar, formal_eskf::verification::NoNormObserver<Scalar>,
                                                         Math>
{
public:
    template <std::size_t Size, std::size_t Columns>
    static Status solve_spd(storage_type<Size, Size> const & system, storage_type<Size, Columns> const & rhs,
                            storage_type<Size, Columns> & output)
    {
        return formal_eskf::linalg::Cholesky<Backend>::template solve<Size, Columns>(system, rhs, output);
    }
};
using Kernel = formal_eskf::linalg::Cholesky<Backend>;
using Factor = Backend::storage_type<size, size>;
using Right = Backend::storage_type<size, columns>;

template <std::size_t Count>
bool equal_cells(ScalarArray<Scalar, Count> const & a, ScalarArray<Scalar, Count> const & b)
{
    bool const first = same(a.head, b.head);
    if constexpr (Count == 1U)
    {
        return first;
    }
    else
    {
        return first && equal_cells(a.tail, b.tail);
    }
}

} // namespace solve_proof
