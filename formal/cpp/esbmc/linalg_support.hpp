/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include "correction_support.hpp"
#include <utility>

#ifndef FORMAL_ESKF_PROOF_LINALG_BACKEND
#define FORMAL_ESKF_PROOF_LINALG_BACKEND 0
#endif
#ifndef FORMAL_ESKF_PROOF_ROWS
#define FORMAL_ESKF_PROOF_ROWS 3
#endif
#ifndef FORMAL_ESKF_PROOF_STORAGE_PART
#define FORMAL_ESKF_PROOF_STORAGE_PART 0
#endif
#ifndef FORMAL_ESKF_PROOF_PRODUCT_COLUMNS
#define FORMAL_ESKF_PROOF_PRODUCT_COLUMNS 3
#endif
#ifndef FORMAL_ESKF_PROOF_START_ROW
#define FORMAL_ESKF_PROOF_START_ROW 0
#endif
#ifndef FORMAL_ESKF_PROOF_OTHER_COLUMNS
#define FORMAL_ESKF_PROOF_OTHER_COLUMNS 1
#endif
#ifndef FORMAL_ESKF_PROOF_START_COLUMN
#define FORMAL_ESKF_PROOF_START_COLUMN 0
#endif
#ifndef FORMAL_ESKF_PROOF_BLOCK_ROWS
#define FORMAL_ESKF_PROOF_BLOCK_ROWS 1
#endif
#ifndef FORMAL_ESKF_PROOF_BLOCK_COLUMNS
#define FORMAL_ESKF_PROOF_BLOCK_COLUMNS 1
#endif

namespace linalg_proof
{
using correction_proof::same;
using correction_proof::same_matrix;
using correction_proof::Scalar;
using covariance_oracle::flatten;
using Backend = std::conditional_t<FORMAL_ESKF_PROOF_LINALG_BACKEND == 0, contract_proof::Linalg, solve_proof::Backend>;
constexpr std::size_t rows = FORMAL_ESKF_PROOF_ROWS;
constexpr std::size_t columns = FORMAL_ESKF_PROOF_PRODUCT_COLUMNS;
constexpr std::size_t count = rows * columns;
constexpr unsigned storage_part = FORMAL_ESKF_PROOF_STORAGE_PART;
using Matrix = Backend::matrix_type<rows, columns>;

// No producer summaries, observer cutpoints or numerical-domain restriction.
// These are the two actual existing fixed-array backend instantiations; their
// different math/product policies are not assumed equivalent.
constexpr bool actual = FORMAL_ESKF_PROOF_LINALG_BACKEND >= 0 && FORMAL_ESKF_PROOF_LINALG_BACKEND <= 1 &&
                        !FORMAL_ESKF_PROOF_CORRECTION_CONTRACT && !FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT &&
                        !FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY && !FORMAL_ESKF_PROOF_FINITE_CONTRACT &&
                        !FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT && !FORMAL_ESKF_PROOF_ACCESS_CONTRACT &&
                        !FORMAL_ESKF_PROOF_PRODUCT_ENTRY_CONTRACT && !FORMAL_ESKF_PROOF_DOT_CONTRACT;

// Independent IEEE classification, including positive zero after abs(-0).
inline Scalar magnitude(Scalar value) { return std::fabs(value); }
} // namespace linalg_proof
