/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
// Actual INS reset body; only the three 3x3 and one 3x15 products use
// arbitrary-return summaries. Same-type F-LINALG product profiles establish
// their ordered IEEE coefficients and input frames. The caller proves every
// operand, call order, remaining numerical operation and all 225 output cells.
#include <cstdint>
#include <type_traits>
#include "covariance_oracle.hpp"
#include <formal_eskf/runtime/magnetometer.hpp>
namespace reset_composition
{
using contract_proof::Linalg;
using contract_proof::Scalar;
using covariance_oracle::same_matrix;
using formal_eskf::Status;
inline bool same(Scalar a, Scalar b)
{
    using Bits = std::conditional_t<sizeof(Scalar) == 4, std::uint32_t, std::uint64_t>;
    return __builtin_bit_cast(Bits, a) == __builtin_bit_cast(Bits, b) || (std::isnan(a) && std::isnan(b));
}
using Matrix = formal_eskf::linalg::Matrix<Linalg, 15, 15>;
using Matrix3 = formal_eskf::linalg::Matrix<Linalg, 3, 3>;
using Cross = formal_eskf::linalg::Matrix<Linalg, 3, 15>;
struct Replies
{
    Matrix3 projected, A, retained;
    Cross cross;
};
struct Calls
{
    inline static Matrix3 before, after;
    inline static Scalar prior[225];
    inline static Replies replies;
    inline static unsigned small{}, wide{};
};
} // namespace reset_composition
namespace formal_eskf::linalg
{
template <>
template <>
inline Matrix<contract_proof::Linalg, 3, 3> Matrix<contract_proof::Linalg, 3, 3>::operator*
    <3>(Matrix<contract_proof::Linalg, 3, 3> const & rhs) const
{
    using namespace reset_composition;
    __ESBMC_assert(Calls::small < 3, "reset composition: exactly three small products");
    bool valid = true;
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j)
        {
            Scalar const left = Calls::small == 0   ? Calls::after(j, i)
                                : Calls::small == 1 ? Calls::replies.projected(i, j)
                                                    : Calls::replies.cross(i, 6 + j);
            Scalar const right = Calls::small == 0   ? (i == j && i < 2 ? Scalar{1} : Scalar{0})
                                 : Calls::small == 1 ? Calls::before(i, j)
                                                     : Calls::replies.A(j, i);
            valid = same((*this)(i, j), left) && same(rhs(i, j), right) && valid;
        }
    }
    __ESBMC_assert(
        valid,
        "reset composition: old/new frames, tilt projector and returned cross-attitude block reach correct products");
    __ESBMC_assert(Calls::wide == (Calls::small == 2 ? 1U : 0U),
                   "reset composition: cross product follows A and precedes retained attitude product");
    unsigned const stage = Calls::small++;
    if (stage == 0)
    {
        return Calls::replies.projected;
    }
    if (stage == 1)
    {
        return Calls::replies.A;
    }
    return Calls::replies.retained;
}
template <>
template <>
inline Matrix<contract_proof::Linalg, 3, 15> Matrix<contract_proof::Linalg, 3, 3>::operator*
    <15>(Matrix<contract_proof::Linalg, 3, 15> const & rhs) const
{
    using namespace reset_composition;
    __ESBMC_assert(Calls::small == 2 && Calls::wide++ == 0 && same_matrix(*this, Calls::replies.A),
                   "reset composition: returned A is used for cross covariance once");
    Scalar cells[45];
    covariance_oracle::flatten(rhs, cells);
    bool valid = true;
    for (std::size_t i = 0; i < 45; ++i)
    {
        valid = same(cells[i], Calls::prior[90 + i]) && valid;
    }
    __ESBMC_assert(valid, "reset composition: INS attitude rows at offset six reach the cross product");
    return Calls::replies.cross;
}
} // namespace formal_eskf::linalg
void verify_magnetic_reset_ins(reset_composition::Matrix covariance, reset_composition::Matrix3 before,
                               reset_composition::Matrix3 after, reset_composition::Scalar variance,
                               reset_composition::Matrix output, reset_composition::Replies replies)
{
    using namespace reset_composition;
    Scalar prior[225], old_output[225], cross[45], retained[9], attitude[9], axis[3]{};
    covariance_oracle::flatten(covariance, prior);
    covariance_oracle::flatten(output, old_output);
    covariance_oracle::flatten(replies.cross, cross);
    covariance_oracle::flatten(replies.retained, retained);
    Calls::before = before;
    Calls::after = after;
    Calls::replies = replies;
    Calls::small = Calls::wide = 0;
    for (std::size_t i = 0; i < 225; ++i)
    {
        Calls::prior[i] = prior[i];
    }
    auto const old_before = before, old_after = after;
    Scalar const down[3]{0, 0, 1};
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j)
        {
            axis[i] += after(j, i) * down[j];
        }
    }
    constexpr Scalar minimum = static_cast<Scalar>(0.01), floor = minimum * minimum;
    Scalar const effective = variance < floor ? floor : variance;
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j)
        {
            attitude[3 * i + j] = retained[3 * i + j] + (Scalar{0} + axis[i] * axis[j]) * effective;
        }
    }
    bool finite = true;
    for (auto value : cross)
    {
        finite = std::isfinite(value) && finite;
    }
    for (auto value : attitude)
    {
        finite = std::isfinite(value) && finite;
    }
    Status wanted = finite ? Status::success : Status::non_finite_result;
    for (std::size_t i = 0; i < 3; ++i)
    {
        if (wanted == Status::success && attitude[4 * i] < Scalar{0})
        {
            wanted = Status::not_positive_definite;
        }
    }
    Matrix * target = &output;
    if constexpr (FORMAL_ESKF_PROOF_ALIAS)
    {
        target = &covariance;
    }
    auto const status =
        formal_eskf::runtime::detail::reset_heading_covariance<6>(covariance, before, after, variance, *target);
    __ESBMC_assert(status == wanted && Calls::small == 3 && Calls::wide == 1,
                   "reset composition: all products precede ordered finite/diagonal checks");
    Scalar actual[225], input[225];
    covariance_oracle::flatten(*target, actual);
    covariance_oracle::flatten(covariance, input);
    for (std::size_t i = 0; i < 15; ++i)
    {
        for (std::size_t j = 0; j < 15; ++j)
        {
            Scalar expected = prior[15 * i + j];
            bool const a = i >= 6 && i < 9, b = j >= 6 && j < 9;
            if (a && b)
            {
                std::size_t const r = i - 6, c = j - 6;
                expected = r <= c ? attitude[3 * r + c] / Scalar{2} + attitude[3 * c + r] / Scalar{2}
                                  : attitude[3 * c + r] / Scalar{2} + attitude[3 * r + c] / Scalar{2};
            }
            else if (a)
            {
                expected = cross[15 * (i - 6) + j];
            }
            else if (b)
            {
                expected = cross[15 * (j - 6) + i];
            }
            if (status != Status::success)
            {
                expected = FORMAL_ESKF_PROOF_ALIAS ? prior[15 * i + j] : old_output[15 * i + j];
            }
            __ESBMC_assert(same(actual[15 * i + j], expected),
                           "reset composition: all 225 yaw/tilt/cross/untouched coefficients and failure frame");
            if constexpr (!FORMAL_ESKF_PROOF_ALIAS)
            {
                __ESBMC_assert(same(input[15 * i + j], prior[15 * i + j]),
                               "reset composition: distinct prior remains unchanged");
            }
        }
    }
    __ESBMC_assert(same_matrix(before, old_before) && same_matrix(after, old_after),
                   "reset composition: both frame inputs remain unchanged");
}
int main() { return 0; }
