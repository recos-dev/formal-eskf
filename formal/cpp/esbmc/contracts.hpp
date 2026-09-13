/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include "support.hpp"

#ifndef FORMAL_ESKF_PROOF_BINARY64
#define FORMAL_ESKF_PROOF_BINARY64 0
#endif
#ifndef FORMAL_ESKF_PROOF_ALIAS
#define FORMAL_ESKF_PROOF_ALIAS 0
#endif
#ifndef FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT
#define FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT 0
#endif

namespace contract_proof
{

#if FORMAL_ESKF_PROOF_BINARY64
using Scalar = double;
#else
using Scalar = float;
#endif
using formal_eskf::Status;

// Value, signed zero and NaN classification; not a NaN payload guarantee.
inline bool same(Scalar a, Scalar b)
{
    return (a == b && (a != Scalar{0} || std::signbit(a) == std::signbit(b))) || (std::isnan(a) && std::isnan(b));
}

template <typename Vector> bool same_vector(Vector const & a, Vector const & b)
{
    for (std::size_t i = 0U; i < Vector::row_count; ++i)
    {
        if (!same(a(i), b(i)))
        {
            return false;
        }
    }
    return true;
}

template <typename Vector> bool finite_vector(Vector const & value)
{
    bool finite = true;
    for (std::size_t i = 0U; i < Vector::row_count; ++i)
    {
        finite = finite && std::isfinite(value(i));
    }
    return finite;
}

template <typename Vector> Scalar ordered_squared_norm(Vector const & value)
{
    Scalar result{0};
    for (std::size_t i = 0U; i < Vector::row_count; ++i)
    {
        result += value(i) * value(i);
    }
    return result;
}

// An unconstrained scalar-return boundary, NOT a model of accurate libm.
// Every IEEE return is allowed, including NaN/Inf and negative roots. Therefore
// caller control/data-flow results hold for any pure scalar implementation;
// numerical properties require separate, stronger contracts. Callable members
// avoid ESBMC replacing a function named sqrt with its intrinsic.
struct OpaqueMath : formal_eskf::scalar::StandardMath<Scalar>
{
    struct Unary
    {
        Scalar result;
        mutable Scalar argument;
        mutable unsigned calls;
        Scalar operator()(Scalar value) const
        {
            __ESBMC_assert(calls == 0U, "scalar boundary: at most one call per primitive");
            ++calls;
            argument = value;
            return result;
        }
    };
    struct Binary
    {
        Scalar result;
        mutable Scalar y, x;
        mutable unsigned calls;
        Scalar operator()(Scalar a, Scalar b) const
        {
            __ESBMC_assert(calls == 0U, "scalar boundary: at most one atan2 call");
            ++calls;
            y = a;
            x = b;
            return result;
        }
    };
    inline static Unary sqrt{}, sin{}, cos{};
    inline static Binary atan2{};

    static void prepare(Scalar root, Scalar sine, Scalar cosine, Scalar angle)
    {
        sqrt = {root, Scalar{0}, 0U};
        sin = {sine, Scalar{0}, 0U};
        cos = {cosine, Scalar{0}, 0U};
        atan2 = {angle, Scalar{0}, Scalar{0}, 0U};
    }
};

using Linalg =
    formal_eskf::verification::FixedArrayLinalg<Scalar, formal_eskf::verification::NoNormObserver<Scalar>, OpaqueMath>;
using Quaternion = formal_eskf::so3::UnitQuaternion<Linalg>;
using Vector3 = Linalg::vector_type<3U>;
using Vector4 = Linalg::vector_type<4U>;

// The actual vector constructor is proved in normalization.cpp. This summary
// assumes only failure atomicity and input purity; it does NOT assume success,
// finiteness, a norm bound or particular coefficients. The caller must forward
// the complete request and publish exactly the opaque successful return.
struct ConstructorContract
{
    inline static Quaternion result{};
    inline static Status status{};
    inline static Vector4 received{};
    inline static Scalar minimum{};
    inline static unsigned calls = 0U;

    static void prepare(Quaternion const & candidate, Status candidate_status)
    {
        result = candidate;
        status = candidate_status;
        calls = 0U;
    }
};

} // namespace contract_proof

#if FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT
namespace formal_eskf::so3
{
template <>
inline Status UnitQuaternion<contract_proof::Linalg>::try_from_coefficients(vector4_type const & coefficients,
                                                                            value_type minimum_norm,
                                                                            UnitQuaternion & output) noexcept
{
    using contract_proof::ConstructorContract;
    __ESBMC_assert(ConstructorContract::calls == 0U, "constructor contract: at most one request");
    ++ConstructorContract::calls;
    ConstructorContract::received = coefficients;
    ConstructorContract::minimum = minimum_norm;
    if (ConstructorContract::status == Status::success)
    {
        output = ConstructorContract::result;
    }
    return ConstructorContract::status;
}
} // namespace formal_eskf::so3
#endif
