/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "covariance_oracle.hpp"

#include <cstdint>
#include <type_traits>

#include <formal_eskf/eskf/codegen.hpp>
#include <formal_eskf/eskf/covariance.hpp>
#include <formal_eskf/eskf/generated/ins_covariance.hpp>
#include <formal_eskf/eskf/generated/ins_reset.hpp>

namespace generated_pcov_proof
{
using namespace contract_proof;
using namespace covariance_oracle;
using Matrix = formal_eskf::linalg::Matrix<Linalg, 15U, 15U>;
using Matrix3 = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
using NoiseMatrix = formal_eskf::linalg::Matrix<Linalg, 12U, 12U>;
static_assert(ESKF_CODEGEN == 1);
static_assert(FORMAL_ESKF_PROOF_ALIAS == 0 || FORMAL_ESKF_PROOF_ALIAS == 1);
static_assert(!FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT && !FORMAL_ESKF_PROOF_ACCESS_CONTRACT &&
              !FORMAL_ESKF_PROOF_PRODUCT_ENTRY_CONTRACT && !FORMAL_ESKF_PROOF_DOT_CONTRACT);
} // namespace generated_pcov_proof

using namespace generated_pcov_proof;

void check_mirrored_covariance(Matrix const & result)
{
    using Bits = std::conditional_t<sizeof(Scalar) == 4U, std::uint32_t, std::uint64_t>;
    // Read the named backing cells directly. A second native SMT array would
    // force every arithmetic expression into each independent mirror query.
    auto const & entries = cells(result);
    for (std::size_t row = 0U; row < 15U; ++row)
    {
        for (std::size_t column = row + 1U; column < 15U; ++column)
        {
            // ESBMC 8.4 needs scalar values, not reference-return expressions,
            // as operands of its bit-cast intrinsic.
            Scalar const upper = entries[row][column];
            Scalar const lower = entries[column][row];
            __ESBMC_assert(
                __builtin_bit_cast(Bits, upper) == __builtin_bit_cast(Bits, lower) ||
                    (std::isnan(upper) && std::isnan(lower)),
                "Generated covariance: every off-diagonal pair is mirrored, including signed zero/NaN class");
        }
    }
}

// Actual emitted arithmetic on the exact caller backend. No finite, symmetry,
// PSD or structured-input premise is needed for frames and mirrored output.
// Exact-real algebra and numerical accuracy are separate codegen obligations.
void verify_generated_covariance(Matrix F, Matrix P, Matrix3 R, NoiseMatrix Q_i)
{
    auto const old_F = F;
    auto const old_P = P;
    auto const old_R = R;
    auto const old_Q = Q_i;
    auto const result = formal_eskf::detail::generated::ins_covariance(F, P, R, Q_i);
    check_mirrored_covariance(result);
    __ESBMC_assert(same_matrix(F, old_F) && same_matrix(P, old_P) && same_matrix(R, old_R) && same_matrix(Q_i, old_Q),
                   "E-PCOV generated: complete transition, covariance, rotation and noise input frames");
}

void verify_generated_reset(Matrix P, Matrix3 J)
{
    auto const old_P = P;
    auto const old_J = J;
    auto const result = formal_eskf::detail::generated::ins_reset(P, J);
    check_mirrored_covariance(result);
    __ESBMC_assert(same_matrix(P, old_P) && same_matrix(J, old_J),
                   "E-RESET generated: full covariance and Jacobian input frames");
}

// Actual finite scan and assignment. Arbitrary IEEE matrices admit every NaN,
// infinity, subnormal and signed zero, with arbitrary old output and both aliases.
void verify_publish_covariance(Matrix candidate, Matrix output)
{
    auto const old_candidate = candidate;
#if FORMAL_ESKF_PROOF_ALIAS
    auto & target = candidate;
    (void)output;
#else
    auto & target = output;
#endif
    auto const before = target;
    Status const status = formal_eskf::detail::publish_covariance(candidate, target);
    bool const finite = finite_matrix(old_candidate);
    __ESBMC_assert(status == (finite ? Status::success : Status::non_finite_result),
                   "E-PCOV publisher: reject exactly non-finite candidates");
    if (finite)
    {
        __ESBMC_assert(same_matrix(target, old_candidate), "E-PCOV publisher: exact copy without averaging on success");
    }
    else
    {
        __ESBMC_assert(same_matrix(target, before), "E-PCOV publisher: unchanged output on failure");
    }
    __ESBMC_assert(same_matrix(candidate, old_candidate), "E-PCOV publisher: unchanged candidate, including alias");
}

int main() { return 0; }
