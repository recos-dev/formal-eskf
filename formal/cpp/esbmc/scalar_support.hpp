/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <cstdint>
#include <cmath>
#include <limits>
#include <numbers>
#include <type_traits>

#ifndef FORMAL_ESKF_PROOF_BINARY64
#define FORMAL_ESKF_PROOF_BINARY64 0
#endif
#ifndef FORMAL_ESKF_PROOF_ALIAS
#define FORMAL_ESKF_PROOF_ALIAS 0
#endif
#ifndef FORMAL_ESKF_PROOF_SCALAR_LIBM
#define FORMAL_ESKF_PROOF_SCALAR_LIBM 0
#endif

extern void __ESBMC_assert(bool, char const *);

// ESBMC recognizes even a user-defined member named sqrt as an intrinsic and
// skips its body. Alpha-rename these four tokens in the TWO production headers
// and harness calls only; their bodies, predicates and types are unchanged.
// The std endpoints below are explicit libm summaries, not reimplemented math.
// Include standard headers first so their contents are not macro-rewritten.
#if FORMAL_ESKF_PROOF_SCALAR_LIBM
namespace std
{
float formal_eskf_proof_sqrt(float) noexcept;
double formal_eskf_proof_sqrt(double) noexcept;
float formal_eskf_proof_sin(float) noexcept;
double formal_eskf_proof_sin(double) noexcept;
float formal_eskf_proof_cos(float) noexcept;
double formal_eskf_proof_cos(double) noexcept;
float formal_eskf_proof_atan2(float, float) noexcept;
double formal_eskf_proof_atan2(double, double) noexcept;
} // namespace std
#define sqrt formal_eskf_proof_sqrt
#define sin formal_eskf_proof_sin
#define cos formal_eskf_proof_cos
#define atan2 formal_eskf_proof_atan2
#endif

#include <formal_eskf/scalar/backend/standard.hpp>
#include <formal_eskf/scalar/math.hpp>

namespace scalar_proof
{
#if FORMAL_ESKF_PROOF_BINARY64
using Scalar = double;
using Bits = std::uint64_t;
constexpr Bits sign_mask = 0x8000000000000000ULL;
constexpr Bits exponent_mask = 0x7ff0000000000000ULL;
constexpr Bits fraction_mask = 0x000fffffffffffffULL;
constexpr Bits pi_bits = 0x400921fb54442d18ULL;
#else
using Scalar = float;
using Bits = std::uint32_t;
constexpr Bits sign_mask = 0x80000000U;
constexpr Bits exponent_mask = 0x7f800000U;
constexpr Bits fraction_mask = 0x007fffffU;
constexpr Bits pi_bits = 0x40490fdbU;
#endif
static_assert(sizeof(Scalar) == sizeof(Bits));
using Math = formal_eskf::scalar::StandardMath<Scalar>;
using Pair = formal_eskf::scalar::SinCos<Scalar>;
using formal_eskf::Status;
constexpr unsigned alias = FORMAL_ESKF_PROOF_ALIAS;
constexpr bool modeled_libm = FORMAL_ESKF_PROOF_SCALAR_LIBM == 1;

// Independent representation oracle. Signed zero is observable; NaN payloads,
// signalling/quiet distinctions and floating-point exception flags are not.
inline Bits bits(Scalar value) { return __builtin_bit_cast(Bits, value); }
inline bool finite(Scalar value) { return (bits(value) & exponent_mask) != exponent_mask; }
inline bool nan(Scalar value)
{
    return (bits(value) & exponent_mask) == exponent_mask && (bits(value) & fraction_mask) != 0U;
}
inline bool zero(Scalar value) { return (bits(value) & ~sign_mask) == 0U; }
inline bool negative(Scalar value) { return !nan(value) && !zero(value) && (bits(value) & sign_mask) != 0U; }
inline bool same(Scalar a, Scalar b) { return bits(a) == bits(b) || (nan(a) && nan(b)); }
inline bool same_pair(Pair a, Pair b) { return same(a.sine, b.sine) && same(a.cosine, b.cosine); }

struct UnaryCall
{
    Scalar result{}, argument{};
    unsigned calls{}, order{};
    Scalar record(Scalar value);
};
struct BinaryCall
{
    Scalar result{}, y{}, x{};
    unsigned calls{}, order{};
    Scalar record(Scalar first, Scalar second);
};
struct Libm
{
    inline static UnaryCall root{}, sine{}, cosine{};
    inline static BinaryCall angle{};
    inline static unsigned count{};
    static void prepare(Scalar root_result, Scalar sine_result, Scalar cosine_result, Scalar angle_result)
    {
        root = {root_result};
        sine = {sine_result};
        cosine = {cosine_result};
        angle = {angle_result};
        count = 0U;
    }
};
inline Scalar UnaryCall::record(Scalar value)
{
    __ESBMC_assert(calls++ == 0U, "F-SCALAR libm boundary: each requested primitive is called once");
    argument = value;
    order = Libm::count++;
    return result;
}
inline Scalar BinaryCall::record(Scalar first, Scalar second)
{
    __ESBMC_assert(calls++ == 0U, "F-SCALAR libm boundary: atan2 is called once");
    y = first;
    x = second;
    order = Libm::count++;
    return result;
}

inline Status sqrt_status(Scalar input, Scalar result)
{
    if (!finite(input))
        return Status::non_finite_input;
    if (negative(input))
        return Status::domain_error;
    return finite(result) ? Status::success : Status::non_finite_result;
}
inline Status sin_cos_status(Scalar input, Pair result)
{
    if (!finite(input))
        return Status::non_finite_input;
    return finite(result.sine) && finite(result.cosine) ? Status::success : Status::non_finite_result;
}
inline Status atan2_status(Scalar y, Scalar x, Scalar result)
{
    if (!finite(y) || !finite(x))
        return Status::non_finite_input;
    if (zero(y) && zero(x))
        return Status::domain_error;
    return finite(result) ? Status::success : Status::non_finite_result;
}
} // namespace scalar_proof

// Isolated library boundary: no support.hpp/first-quadrant atan2f model is
// imported. Arbitrary IEEE returns check dispatch/publication, not library
// implementation or accuracy. Wrong scalar overloads fail explicitly.
#if FORMAL_ESKF_PROOF_SCALAR_LIBM
namespace std
{
#if FORMAL_ESKF_PROOF_BINARY64
using OtherScalar = float;
#else
using OtherScalar = double;
#endif
inline scalar_proof::Scalar formal_eskf_proof_sqrt(scalar_proof::Scalar x) noexcept
{
    return scalar_proof::Libm::root.record(x);
}
inline scalar_proof::Scalar formal_eskf_proof_sin(scalar_proof::Scalar x) noexcept
{
    return scalar_proof::Libm::sine.record(x);
}
inline scalar_proof::Scalar formal_eskf_proof_cos(scalar_proof::Scalar x) noexcept
{
    return scalar_proof::Libm::cosine.record(x);
}
inline scalar_proof::Scalar formal_eskf_proof_atan2(scalar_proof::Scalar y, scalar_proof::Scalar x) noexcept
{
    return scalar_proof::Libm::angle.record(y, x);
}
inline OtherScalar formal_eskf_proof_sqrt(OtherScalar) noexcept
{
    __ESBMC_assert(false, "F-SCALAR: wrong libm scalar overload");
    return OtherScalar{0};
}
inline OtherScalar formal_eskf_proof_sin(OtherScalar) noexcept
{
    __ESBMC_assert(false, "F-SCALAR: wrong libm scalar overload");
    return OtherScalar{0};
}
inline OtherScalar formal_eskf_proof_cos(OtherScalar) noexcept
{
    __ESBMC_assert(false, "F-SCALAR: wrong libm scalar overload");
    return OtherScalar{0};
}
inline OtherScalar formal_eskf_proof_atan2(OtherScalar, OtherScalar) noexcept
{
    __ESBMC_assert(false, "F-SCALAR: wrong libm scalar overload");
    return OtherScalar{0};
}
} // namespace std
#endif
