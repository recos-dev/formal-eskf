// SPDX-License-Identifier: BSD-3-Clause

// Tool qualification only; this is not an estimator proof or a NaN-payload guarantee.
#include <cmath>
#include <cstdint>
#include <limits>
extern "C" void __ESBMC_assert(bool, const char *);
extern "C" void __ESBMC_assume(bool);
#if FORMAL_ESKF_PROOF_BINARY64
using Scalar = double;
using Bits = std::uint64_t;
constexpr Bits sign = Bits{1} << 63;
constexpr Bits exponent = Bits{0x7ff} << 52;
constexpr Bits fraction = (Bits{1} << 52) - 1;
#else
using Scalar = float;
using Bits = std::uint32_t;
constexpr Bits sign = Bits{1} << 31;
constexpr Bits exponent = Bits{0xff} << 23;
constexpr Bits fraction = (Bits{1} << 23) - 1;
#endif
bool same(Scalar a, Scalar b)
{
    return (__builtin_bit_cast(Bits, a) == __builtin_bit_cast(Bits, b)) | (std::isnan(a) & std::isnan(b));
}
void verify_relation(Scalar a, Scalar b)
{
    __ESBMC_assert(same(a, b) == ((a == b && (a != Scalar{0} || std::signbit(a) == std::signbit(b))) ||
                                  (std::isnan(a) && std::isnan(b))),
                   "arbitrary IEEE frame relation");
}
void verify_literals()
{
    __ESBMC_assert(__builtin_bit_cast(Bits, Scalar{0}) == 0, "positive zero bits");
    __ESBMC_assert(__builtin_bit_cast(Bits, -Scalar{0}) == sign, "negative zero bits");
    __ESBMC_assert(!same(Scalar{0}, -Scalar{0}), "distinct signed zeros");
    __ESBMC_assert(Scalar{0} == -Scalar{0}, "numeric zero equality retained");
}
void verify_round_trip(Bits bits)
{
    __ESBMC_assume((bits & exponent) != exponent || (bits & fraction) == 0);
    auto const value = __builtin_bit_cast(Scalar, bits);
    __ESBMC_assert(__builtin_bit_cast(Bits, value) == bits, "non-NaN representation round trip");
}
unsigned calls;
Scalar const & reference(Scalar const & x)
{
    ++calls;
    return x;
}
Scalar value(Scalar x)
{
    ++calls;
    return x;
}
void verify_reference(Scalar x)
{
    calls = 0;
    auto const actual = __builtin_bit_cast(Bits, reference(x));
    __ESBMC_assert(actual == __builtin_bit_cast(Bits, x) && calls == 1, "reference read with one evaluation");
}
void verify_value(Scalar x)
{
    calls = 0;
    auto const actual = __builtin_bit_cast(Bits, value(x));
    auto const restored = __builtin_bit_cast(Scalar, actual);
    __ESBMC_assert(same(restored, x) && calls == 1, "temporary value with one evaluation and IEEE field preservation");
}
void verify_negation(Scalar x)
{
    __ESBMC_assume(!std::isnan(x));
    auto const expected = __builtin_bit_cast(Bits, x) ^ sign;
    __ESBMC_assert(__builtin_bit_cast(Bits, -x) == expected, "non-NaN unary sign inversion");
}
void verify_nan()
{
    Scalar const a = std::numeric_limits<Scalar>::quiet_NaN();
    Scalar const b = -a;
    __ESBMC_assert(a != a && b != b && same(a, b), "NaN numeric inequality and shared frame classification");
}
void reject_zero_collapse() { __ESBMC_assert(same(Scalar{0}, -Scalar{0}), "negative control: zero collapse"); }
void reject_overwrite() { __ESBMC_assert(same(Scalar{1}, Scalar{2}), "negative control: changed value"); }
void reject_nan_rejection()
{
    Scalar const a = std::numeric_limits<Scalar>::quiet_NaN();
    __ESBMC_assert(!same(a, a), "negative control: rejected NaN classification");
}
