/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "contracts.hpp"

#ifndef FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT
#define FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT 0
#endif

using namespace contract_proof;

namespace maps_proof
{
struct SquaredNormContract
{
    inline static Scalar result{};
    inline static Vector3 received{};
    inline static unsigned calls = 0U;
};
} // namespace maps_proof

#if FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT
namespace formal_eskf::linalg
{
template <> inline contract_proof::Scalar squared_norm(contract_proof::Vector3 const & input) noexcept
{
    using maps_proof::SquaredNormContract;
    __ESBMC_assert(SquaredNormContract::calls == 0U, "Exp norm contract: at most one request");
    ++SquaredNormContract::calls;
    SquaredNormContract::received = input;
    return SquaredNormContract::result;
}
} // namespace formal_eskf::linalg
#endif

// The actual three-vector reduction closes Exp's pure squared-norm summary.
// There is no finite/positive assumption on either the input or its result.
void verify_squared_norm_producer(Vector3 value)
{
    __ESBMC_assert(!FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT, "Runner error: squared-norm producer must be actual");
    auto const before = value;
    auto const actual = formal_eskf::linalg::squared_norm(value);
    __ESBMC_assert(same(actual, ordered_squared_norm(before)) && same_vector(value, before),
                   "SO3-EXP: actual squared norm is the ordered sum and preserves its input");
    // Concrete feasibility evidence for zero, exact cutoff and regular cases.
    // These dyadic inputs are exactly representable in their respective formats.
    Vector3 boundary;
    __ESBMC_assert(formal_eskf::linalg::squared_norm(boundary) == Scalar{0}, "SO3-SMALL-ANGLE: zero is reachable");
#if FORMAL_ESKF_PROOF_BINARY64
    boundary(0U) = Scalar{0x1p-26};
#else
    boundary(0U) = Scalar{0x1p-12};
    boundary(1U) = Scalar{0x1p-12};
#endif
    __ESBMC_assert(formal_eskf::linalg::squared_norm(boundary) == std::numeric_limits<Scalar>::epsilon(),
                   "SO3-SMALL-ANGLE: exact epsilon boundary is reachable");
    boundary(0U) *= Scalar{2};
    __ESBMC_assert(formal_eskf::linalg::squared_norm(boundary) > std::numeric_limits<Scalar>::epsilon(),
                   "SO3-SMALL-ANGLE: regular side is reachable");
}

// Pure producer equations also cover calls outside try_log's reachable domain.
// These retain and extend the old restricted scale/coefficient checks.
void verify_log_arithmetic(Scalar q0, Scalar q0_squared, Scalar squared, Scalar norm, Scalar angle, Vector3 qv,
                           Scalar scale)
{
    __ESBMC_assert(same(formal_eskf::so3::detail::log_taylor_scale(q0, q0_squared, squared),
                        Scalar{2} / q0 * (Scalar{1} - squared / (Scalar{3} * q0_squared))),
                   "SO3-SMALL-ANGLE: exact ordered Taylor scale expression on all IEEE arguments");
    __ESBMC_assert(same(formal_eskf::so3::detail::log_closed_form_scale(norm, angle), Scalar{2} * angle / norm),
                   "SO3-LOG: exact ordered regular scale expression on all IEEE arguments");
    auto const before = qv;
    auto const candidate = formal_eskf::so3::detail::log_candidate<Linalg>(qv, scale);
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        __ESBMC_assert(same(candidate(i), qv(i) * scale),
                       "SO3-LOG: each candidate coefficient uses the supplied scale");
    }
    __ESBMC_assert(same_vector(qv, before), "SO3-LOG: pure scale candidate preserves input");
}

// Actual checked wrappers, arbitrary IEEE inputs/outputs and primitive returns.
// This establishes dispatch, status precedence and transaction semantics, not
// libm accuracy. No premise on root/trig/atan2 values is assumed.
void verify_scalar_wrappers(Scalar x, Scalar y, Scalar output, Scalar old_sine, Scalar old_cosine, Scalar root,
                            Scalar sine, Scalar cosine, Scalar angle)
{
    Scalar const before = output;
    OpaqueMath::prepare(root, sine, cosine, angle);
    auto actual = formal_eskf::scalar::try_sqrt<OpaqueMath>(x, output);
    Status expected = !std::isfinite(x)      ? Status::non_finite_input
                      : x < Scalar{0}        ? Status::domain_error
                      : !std::isfinite(root) ? Status::non_finite_result
                                             : Status::success;
    bool const root_call = std::isfinite(x) && !(x < Scalar{0});
    __ESBMC_assert(actual == expected && OpaqueMath::sqrt.calls == (root_call ? 1U : 0U) &&
                       same(output, actual == Status::success ? root : before),
                   "F-SCALAR: checked sqrt validates before dispatch and commits only a finite result");
    __ESBMC_assert(!root_call || same(OpaqueMath::sqrt.argument, x), "F-SCALAR: sqrt forwards its input");

    formal_eskf::scalar::SinCos<Scalar> pair{old_sine, old_cosine};
    auto const pair_before = pair;
    actual = formal_eskf::scalar::try_sin_cos<OpaqueMath>(x, pair);
    expected = !std::isfinite(x)                                 ? Status::non_finite_input
               : !(std::isfinite(sine) && std::isfinite(cosine)) ? Status::non_finite_result
                                                                 : Status::success;
    __ESBMC_assert(actual == expected && OpaqueMath::sin.calls == (std::isfinite(x) ? 1U : 0U) &&
                       OpaqueMath::cos.calls == (std::isfinite(x) ? 1U : 0U) &&
                       same(pair.sine, actual == Status::success ? sine : pair_before.sine) &&
                       same(pair.cosine, actual == Status::success ? cosine : pair_before.cosine),
                   "F-SCALAR: checked trig validates once and publishes both results atomically");
    __ESBMC_assert(!std::isfinite(x) || (same(OpaqueMath::sin.argument, x) && same(OpaqueMath::cos.argument, x)),
                   "F-SCALAR: both trig calls receive the same angle");

    output = before;
    actual = formal_eskf::scalar::try_atan2<OpaqueMath>(y, x, output);
    bool const finite = std::isfinite(x) && std::isfinite(y);
    bool const origin = x == Scalar{0} && y == Scalar{0};
    expected = !finite                 ? Status::non_finite_input
               : origin                ? Status::domain_error
               : !std::isfinite(angle) ? Status::non_finite_result
                                       : Status::success;
    __ESBMC_assert(actual == expected && OpaqueMath::atan2.calls == (finite && !origin ? 1U : 0U) &&
                       same(output, actual == Status::success ? angle : before),
                   "F-SCALAR: checked atan2 rejects nonfinite/origin inputs and preserves every failed output");
    __ESBMC_assert(!(finite && !origin) || (same(OpaqueMath::atan2.y, y) && same(OpaqueMath::atan2.x, x)),
                   "F-SCALAR: atan2 preserves y,x argument order");
}

// SO3-EXP/SO3-SMALL-ANGLE: all IEEE vectors and every public threshold, not
// prediction's narrower (0,1] domain. The actual branch formulas, reductions,
// scalar checks and packing execute. The pure squared norm and checked
// constructor are opaque; verify_squared_norm_producer and normalization.cpp
// discharge those boundaries for exactly the same scalar/backend types.
void verify_exp_behavior(Vector3 theta, Scalar minimum, Quaternion output, Quaternion normalized,
                         Status constructor_status, Scalar squared, Scalar root, Scalar sine, Scalar cosine)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT && FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT,
                   "Runner error: Exp requires constructor and squared-norm boundaries");
    auto const before = output;
    auto const theta_before = theta;
    maps_proof::SquaredNormContract::result = squared;
    maps_proof::SquaredNormContract::calls = 0U;
    ConstructorContract::prepare(normalized, constructor_status);
    OpaqueMath::prepare(root, sine, cosine, Scalar{0});
    auto const actual = formal_eskf::so3::try_exp(theta, minimum, output);

    Status expected = Status::success;
    bool root_call = false;
    bool trig_call = false;
    bool constructor_call = false;
    Vector4 coefficients;
    if (!finite_vector(theta_before) || !std::isfinite(minimum))
    {
        expected = Status::non_finite_input;
    }
    else if (!(minimum > Scalar{0}))
    {
        expected = Status::domain_error;
    }
    else if (!std::isfinite(squared))
    {
        expected = Status::non_finite_result;
    }
    else if (squared < Scalar{0})
    {
        expected = Status::domain_error;
    }
    else
    {
        Scalar scale{};
        if (squared <= std::numeric_limits<Scalar>::epsilon())
        {
            Scalar const fourth = squared * squared;
            coefficients(0U) = Scalar{1} - squared / Scalar{8} + fourth / Scalar{384};
            scale = Scalar{0.5} - squared / Scalar{48} + fourth / Scalar{3840};
        }
        else
        {
            root_call = true;
            if (!std::isfinite(root))
            {
                expected = Status::non_finite_result;
            }
            else
            {
                trig_call = true;
                if (!std::isfinite(sine) || !std::isfinite(cosine))
                {
                    expected = Status::non_finite_result;
                }
                else
                {
                    coefficients(0U) = cosine;
                    scale = sine / root;
                }
            }
        }
        if (expected == Status::success)
        {
            if (!std::isfinite(coefficients(0U)) || !std::isfinite(scale))
            {
                expected = Status::non_finite_result;
            }
            else
            {
                constructor_call = true;
                coefficients(1U) = scale * theta_before(0U);
                coefficients(2U) = scale * theta_before(1U);
                coefficients(3U) = scale * theta_before(2U);
                expected = constructor_status;
            }
        }
    }
    __ESBMC_assert(actual == expected, "SO3-EXP: exact status precedence, scalar failures and constructor outcome");
    bool const norm_call = finite_vector(theta_before) && std::isfinite(minimum) && minimum > Scalar{0};
    __ESBMC_assert(maps_proof::SquaredNormContract::calls == (norm_call ? 1U : 0U),
                   "SO3-EXP: squared norm is requested exactly after input validation");
    __ESBMC_assert(!norm_call || same_vector(maps_proof::SquaredNormContract::received, theta_before),
                   "SO3-EXP: squared norm consumes the original rotation vector");
    __ESBMC_assert(OpaqueMath::sqrt.calls == (root_call ? 1U : 0U) && OpaqueMath::sin.calls == (trig_call ? 1U : 0U) &&
                       OpaqueMath::cos.calls == (trig_call ? 1U : 0U) &&
                       ConstructorContract::calls == (constructor_call ? 1U : 0U),
                   "SO3-SMALL-ANGLE: epsilon equality selects Taylor; its complement selects checked half-angle math");
    __ESBMC_assert(!root_call || same(OpaqueMath::sqrt.argument, squared),
                   "SO3-EXP: root receives ordered squared norm");
    __ESBMC_assert(!trig_call || (same(OpaqueMath::sin.argument, Scalar{0.5} * root) &&
                                  same(OpaqueMath::cos.argument, Scalar{0.5} * root)),
                   "SO3-EXP: both trigonometric functions consume the half angle");
    if (constructor_call)
    {
        __ESBMC_assert(same_vector(ConstructorContract::received, coefficients) &&
                           same(ConstructorContract::minimum, minimum),
                       "SO3-EXP: all four Taylor/regular coefficients and the unchanged threshold reach normalization");
    }
    __ESBMC_assert(actual != Status::success || same_vector(output.coefficients(), normalized.coefficients()),
                   "SO3-EXP: exact successful publication");
    __ESBMC_assert(actual == Status::success || same_vector(output.coefficients(), before.coefficients()),
                   "SO3-EXP: general failure rollback");
    __ESBMC_assert(same_vector(theta, theta_before), "SO3-EXP: input preservation");
}

// SO3-LOG: all IEEE representations, not an assumed-unit fixture. The zero
// vector branch intentionally describes the implementation even for q=0;
// the mathematical principal-range theorem separately requires a unit input.
void verify_log_behavior(Quaternion q, Vector3 output, Scalar root, Scalar half_angle)
{
    __ESBMC_assert(!FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT, "Runner error: Log uses the actual squared norm");
    auto const q_before = q.coefficients();
    auto const before = output;
    OpaqueMath::prepare(root, Scalar{0}, Scalar{0}, half_angle);
    auto const actual = formal_eskf::so3::try_log(q, output);
    Scalar const sign = q_before(0U) < Scalar{0} ? Scalar{-1} : Scalar{1};
    Scalar const q0 = sign * q_before(0U);
    Vector3 qv;
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        qv(i) = sign * q_before(i + 1U);
    }
    Scalar const squared = ordered_squared_norm(qv);
    bool root_call = false;
    bool atan_call = false;
    Status expected = Status::success;
    Vector3 candidate;
    if (!finite_vector(q_before))
    {
        expected = Status::non_finite_input;
    }
    else if (!std::isfinite(squared))
    {
        expected = Status::non_finite_result;
    }
    else if (squared < Scalar{0})
    {
        expected = Status::domain_error;
    }
    else if (squared != Scalar{0})
    {
        Scalar scale{};
        if (squared <= std::numeric_limits<Scalar>::epsilon())
        {
            Scalar const scalar_squared = q0 * q0;
            if (!(scalar_squared > Scalar{0}))
            {
                expected = Status::zero_or_unsafe_divisor;
            }
            else
            {
                scale = Scalar{2} / q0 * (Scalar{1} - squared / (Scalar{3} * scalar_squared));
            }
        }
        else
        {
            root_call = true;
            if (!std::isfinite(root))
            {
                expected = Status::non_finite_result;
            }
            else if (root == Scalar{0} && q0 == Scalar{0})
            {
                expected = Status::domain_error;
            }
            else
            {
                atan_call = true;
                if (!std::isfinite(half_angle))
                {
                    expected = Status::non_finite_result;
                }
                else
                {
                    scale = Scalar{2} * half_angle / root;
                }
            }
        }
        if (expected == Status::success)
        {
            if (!std::isfinite(scale))
            {
                expected = Status::non_finite_result;
            }
            else
            {
                for (std::size_t i = 0U; i < 3U; ++i)
                {
                    candidate(i) = qv(i) * scale;
                }
                if (!finite_vector(candidate))
                {
                    expected = Status::non_finite_result;
                }
            }
        }
    }
    __ESBMC_assert(actual == expected, "SO3-LOG: complete zero/Taylor/regular status and overflow behavior");
    __ESBMC_assert(
        OpaqueMath::sqrt.calls == (root_call ? 1U : 0U) && OpaqueMath::atan2.calls == (atan_call ? 1U : 0U),
        "SO3-SMALL-ANGLE: zero precedes Taylor, epsilon equality remains Taylor, regular calls checked math");
    __ESBMC_assert(!root_call || same(OpaqueMath::sqrt.argument, squared),
                   "SO3-LOG: root receives principal vector squared norm");
    __ESBMC_assert(!atan_call || (same(OpaqueMath::atan2.y, root) && same(OpaqueMath::atan2.x, q0)),
                   "SO3-LOG: atan2 receives vector norm then principal scalar coefficient");
    __ESBMC_assert(actual != Status::success || same_vector(output, candidate),
                   "SO3-LOG: all three computed coefficients on success");
    __ESBMC_assert(actual == Status::success || same_vector(output, before),
                   "SO3-LOG: arbitrary old output on every failure");
    __ESBMC_assert(same_vector(q.coefficients(), q_before), "SO3-LOG: input quaternion is unchanged");
}

int main() { return 0; }
