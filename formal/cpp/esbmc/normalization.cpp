/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "contracts.hpp"

#ifndef FORMAL_ESKF_PROOF_NORMALIZATION_BOUNDARY
#define FORMAL_ESKF_PROOF_NORMALIZATION_BOUNDARY 0
#endif

namespace normalization_proof
{
using namespace contract_proof;

// Unconstrained pure returns. No feasible success or failure path is removed.
struct NormContract
{
    inline static Scalar result{};
    inline static Vector4 received{};
    inline static unsigned calls = 0U;
};
struct CandidateContract
{
    inline static Vector4 result{}, received{};
    inline static Scalar divisor{};
    inline static unsigned calls = 0U;
};
struct NormalizeContract
{
    inline static Vector4 result{}, received{};
    inline static Scalar minimum{};
    inline static Status status{};
    inline static unsigned calls = 0U;
};
} // namespace normalization_proof

#if FORMAL_ESKF_PROOF_NORMALIZATION_BOUNDARY == 1
namespace formal_eskf::linalg
{
template <> inline contract_proof::Scalar norm(contract_proof::Vector4 const & input) noexcept
{
    using normalization_proof::NormContract;
    __ESBMC_assert(NormContract::calls == 0U, "normalization contract: one norm request");
    ++NormContract::calls;
    NormContract::received = input;
    return NormContract::result;
}
namespace detail
{
template <>
inline contract_proof::Vector4 normalization_candidate(contract_proof::Vector4 const & input,
                                                       contract_proof::Scalar divisor) noexcept
{
    using normalization_proof::CandidateContract;
    __ESBMC_assert(CandidateContract::calls == 0U, "normalization contract: one division request");
    ++CandidateContract::calls;
    CandidateContract::received = input;
    CandidateContract::divisor = divisor;
    return CandidateContract::result;
}
} // namespace detail
} // namespace formal_eskf::linalg
#elif FORMAL_ESKF_PROOF_NORMALIZATION_BOUNDARY == 2
namespace formal_eskf::linalg
{
template <>
inline Status try_normalize(contract_proof::Vector4 const & input, contract_proof::Scalar minimum,
                            contract_proof::Vector4 & output) noexcept
{
    using normalization_proof::NormalizeContract;
    __ESBMC_assert(NormalizeContract::calls == 0U, "normalization contract: one checked-vector request");
    ++NormalizeContract::calls;
    NormalizeContract::received = input;
    NormalizeContract::minimum = minimum;
    if (NormalizeContract::status == Status::success)
    {
        output = NormalizeContract::result;
    }
    return NormalizeContract::status;
}
} // namespace formal_eskf::linalg
#endif

using namespace normalization_proof;

// Actual fixed-array norm, including its reduction order and root dispatch.
// The primitive return is arbitrary, so no numerical sqrt claim is hidden here.
void verify_norm_producer(Vector4 input, Scalar root)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_NORMALIZATION_BOUNDARY == 0, "Runner error: norm producer must be actual");
    auto const before = input;
    OpaqueMath::prepare(root, Scalar{0}, Scalar{0}, Scalar{0});
    Scalar const actual = formal_eskf::linalg::norm(input);
    __ESBMC_assert(OpaqueMath::sqrt.calls == 1U && same(OpaqueMath::sqrt.argument, ordered_squared_norm(before)),
                   "Q-NORMALIZE: computed norm roots the ordered sum of all four squared coefficients");
    __ESBMC_assert(same(actual, root) && same_vector(input, before),
                   "Q-NORMALIZE: norm returns the primitive result and preserves its input");
}

void verify_division_producer(Vector4 input, Scalar norm)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_NORMALIZATION_BOUNDARY == 0, "Runner error: division producer must be actual");
    auto const before = input;
    auto const actual = formal_eskf::linalg::detail::normalization_candidate(input, norm);
    for (std::size_t i = 0U; i < 4U; ++i)
    {
        __ESBMC_assert(same(actual(i), before(i) / norm),
                       "Q-NORMALIZE: all four candidate divisions use the same norm");
    }
    __ESBMC_assert(same_vector(input, before), "Q-NORMALIZE: division preserves input");
}

// All IEEE coefficients, thresholds, primitive returns and initial outputs.
// Both legal vector alias layouts are independent profiles. The norm and
// quotient summaries are closed by the two actual producer proofs above.
void verify_checked_normalization(Vector4 input, Scalar minimum, Vector4 output, Scalar norm, Vector4 candidate)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_NORMALIZATION_BOUNDARY == 1, "Runner error: checked normalization boundaries");
    __ESBMC_assert(FORMAL_ESKF_PROOF_ALIAS <= 1, "Runner error: vector normalization alias must be 0 or 1");
    (void)output;
    auto const input_before = input;
#if FORMAL_ESKF_PROOF_ALIAS
    auto const before = input;
    auto & target = input;
#else
    auto const before = output;
    auto & target = output;
#endif
    NormContract::result = norm;
    NormContract::calls = 0U;
    CandidateContract::result = candidate;
    CandidateContract::calls = 0U;
    bool const inputs_ok = finite_vector(input_before) && std::isfinite(minimum);
    bool const divide = inputs_ok && std::isfinite(norm) && minimum > Scalar{0} && !(norm < minimum);
    Status const expected = !inputs_ok                  ? Status::non_finite_input
                            : !std::isfinite(norm)      ? Status::non_finite_result
                            : !divide                   ? Status::zero_or_unsafe_divisor
                            : !finite_vector(candidate) ? Status::non_finite_result
                                                        : Status::success;
    auto const actual = formal_eskf::linalg::try_normalize(input, minimum, target);
    __ESBMC_assert(actual == expected, "Q-NORMALIZE: vector status precedence and inclusive minimum boundary");
    __ESBMC_assert(NormContract::calls == (inputs_ok ? 1U : 0U) && CandidateContract::calls == (divide ? 1U : 0U),
                   "Q-NORMALIZE: dependencies called exactly after their checks");
    if (inputs_ok)
    {
        __ESBMC_assert(same_vector(NormContract::received, input_before), "Q-NORMALIZE: norm receives original input");
    }
    if (divide)
    {
        __ESBMC_assert(same_vector(CandidateContract::received, input_before) && same(CandidateContract::divisor, norm),
                       "Q-NORMALIZE: division uses original coefficients and the computed norm");
    }
    __ESBMC_assert(actual != Status::success || same_vector(target, candidate),
                   "Q-NORMALIZE: publishes every successful coefficient");
    __ESBMC_assert(actual == Status::success || same_vector(target, before),
                   "Q-NORMALIZE: preserves every prior output on failure");
#if !FORMAL_ESKF_PROOF_ALIAS
    __ESBMC_assert(same_vector(input, input_before), "Q-NORMALIZE: distinct vector input is unchanged");
#endif
}

// Both public constructor overloads, all thresholds (including equality, >1,
// negative, NaN and infinities), arbitrary output, every callee status.
void verify_constructor(Vector4 input, Scalar minimum, Quaternion output, Vector4 normalized, Status callee_status)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_NORMALIZATION_BOUNDARY == 2 && !FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT,
                   "Runner error: actual quaternion constructor with checked-vector summary");
    auto const before = output;
    auto const input_before = input;
    bool const delegate = std::isfinite(minimum) && minimum > Scalar{0};
    Status const expected = !std::isfinite(minimum)                           ? Status::non_finite_input
                            : !(minimum > Scalar{0})                          ? Status::domain_error
                            : callee_status == Status::zero_or_unsafe_divisor ? Status::invalid_quaternion_norm
                                                                              : callee_status;
    NormalizeContract::result = normalized;
    NormalizeContract::status = callee_status;
    for (unsigned overload = 0U; overload < 2U; ++overload)
    {
        output = before;
        NormalizeContract::calls = 0U;
        auto const actual = overload == 0U ? Quaternion::try_from_coefficients(input, minimum, output)
                                           : Quaternion::try_from_coefficients(input(0U), input(1U), input(2U),
                                                                               input(3U), minimum, output);
        __ESBMC_assert(actual == expected && NormalizeContract::calls == (delegate ? 1U : 0U),
                       "Q-NORMALIZE: both constructors validate threshold first and translate divisor failure");
        if (delegate)
        {
            __ESBMC_assert(same_vector(NormalizeContract::received, input_before) &&
                               same(NormalizeContract::minimum, minimum),
                           "Q-NORMALIZE: constructors forward every coefficient and threshold unchanged");
        }
        __ESBMC_assert(actual != Status::success || same_vector(output.coefficients(), normalized),
                       "Q-NORMALIZE: both constructors publish all normalized coefficients");
        __ESBMC_assert(actual == Status::success || same_vector(output.coefficients(), before.coefficients()),
                       "Q-NORMALIZE: both constructors preserve arbitrary output on failure");
        __ESBMC_assert(same_vector(input, input_before), "Q-NORMALIZE: constructor input is preserved");
    }
}

void verify_normalize_wrapper(Quaternion q, Scalar minimum, Quaternion output, Quaternion result, Status status)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT,
                   "Runner error: quaternion normalization uses constructor summary");
    __ESBMC_assert(FORMAL_ESKF_PROOF_ALIAS <= 1, "Runner error: quaternion normalization alias must be 0 or 1");
    (void)output;
    auto const q_before = q;
#if FORMAL_ESKF_PROOF_ALIAS
    auto const before = q;
    auto & target = q;
#else
    auto const before = output;
    auto & target = output;
#endif
    ConstructorContract::prepare(result, status);
    auto const actual = formal_eskf::so3::try_normalize(q, minimum, target);
    __ESBMC_assert(ConstructorContract::calls == 1U &&
                       same_vector(ConstructorContract::received, q_before.coefficients()) &&
                       same(ConstructorContract::minimum, minimum) && actual == status,
                   "Q-NORMALIZE: wrapper forwards the complete request and status");
    __ESBMC_assert(status != Status::success || same_vector(target.coefficients(), result.coefficients()),
                   "Q-NORMALIZE: wrapper publishes every successful coefficient, including self alias");
    __ESBMC_assert(status == Status::success || same_vector(target.coefficients(), before.coefficients()),
                   "Q-NORMALIZE: wrapper preserves arbitrary output on failure, including self alias");
#if !FORMAL_ESKF_PROOF_ALIAS
    __ESBMC_assert(same_vector(q.coefficients(), q_before.coefficients()),
                   "Q-NORMALIZE: distinct quaternion input unchanged");
#endif
}

// Five legal alias partitions: all distinct; output=a; output=b; a=b; all same.
// Actual Hamilton multiplication, coefficient packing and normalization wrapper;
// only the separately proved vector constructor is summarized.
void verify_composition_general(Quaternion a, Quaternion b, Scalar minimum, Quaternion output, Quaternion result,
                                Status status)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT, "Runner error: composition uses constructor summary");
    static_assert(FORMAL_ESKF_PROOF_ALIAS < 5);
    (void)output;
#if FORMAL_ESKF_PROOF_ALIAS >= 3
    auto const & operand_b = a;
#else
    auto const & operand_b = b;
#endif
#if FORMAL_ESKF_PROOF_ALIAS == 1 || FORMAL_ESKF_PROOF_ALIAS == 4
    auto & target = a;
#elif FORMAL_ESKF_PROOF_ALIAS == 2
    auto & target = b;
#else
    auto & target = output;
#endif
    [[maybe_unused]] auto const a_before = a;
    [[maybe_unused]] auto const b_before = b;
    auto const before = target;
    auto const product = a * operand_b; // actual producer is independently proved by verify_product
    ConstructorContract::prepare(result, status);
    auto const actual = formal_eskf::so3::try_compose_normalized(a, operand_b, minimum, target);
    __ESBMC_assert(ConstructorContract::calls == 1U &&
                       same_vector(ConstructorContract::received, product.coefficients()) &&
                       same(ConstructorContract::minimum, minimum) && actual == status,
                   "Q-ROT-COMPOSE: old left-times-right product and threshold reach the checked constructor");
    __ESBMC_assert(status != Status::success || same_vector(target.coefficients(), result.coefficients()),
                   "Q-ROT-COMPOSE: all successful outputs and legal aliases follow the constructor result");
    __ESBMC_assert(status == Status::success || same_vector(target.coefficients(), before.coefficients()),
                   "Q-ROT-COMPOSE: every failed output and legal alias is preserved");
#if FORMAL_ESKF_PROOF_ALIAS != 1 && FORMAL_ESKF_PROOF_ALIAS != 4
    __ESBMC_assert(same_vector(a.coefficients(), a_before.coefficients()), "Q-ROT-COMPOSE: non-output left preserved");
#endif
#if FORMAL_ESKF_PROOF_ALIAS != 2
    __ESBMC_assert(same_vector(b.coefficients(), b_before.coefficients()), "Q-ROT-COMPOSE: non-output right preserved");
#endif
}

int main() { return 0; }
