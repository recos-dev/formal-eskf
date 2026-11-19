/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "correction_support.hpp"
#include <formal_eskf/eskf/generated/ins_reset.hpp>

#ifndef FORMAL_ESKF_PROOF_GENERATED_BOUNDARY
#define FORMAL_ESKF_PROOF_GENERATED_BOUNDARY 0
#endif

namespace generated_correction_proof
{
using namespace correction_proof;
using Vector3 = formal_eskf::linalg::Vector<Backend, 3U>;
static_assert(ESKF_CODEGEN && state_size == 15U);
static_assert(FORMAL_ESKF_PROOF_CORRECTION_MODEL >= 1 && FORMAL_ESKF_PROOF_CORRECTION_MODEL <= 5);
static_assert(!FORMAL_ESKF_PROOF_CORRECTION_CONTRACT && !FORMAL_ESKF_PROOF_FINITE_CONTRACT &&
              !FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY && !FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT);
struct Boundary
{
    inline static Covariance const * P = nullptr;
    inline static Covariance const * A = nullptr;
    inline static Gain const * K = nullptr;
    inline static Jacobian const * H = nullptr;
    inline static Noise const * V = nullptr;
    inline static Gain cross{};
    inline static Noise innovation{};
    inline static Covariance joseph{};
    inline static unsigned calls = 0U;
};
void check_body_vector(Vector3 const & h)
{
    __ESBMC_assert(same(h(0U), (*Boundary::H)(2U, 7U)) && same(h(1U), (*Boundary::H)(0U, 8U)) &&
                       same(h(2U), (*Boundary::H)(1U, 6U)),
                   "Generated correction: body vector extracted from the correct skew-Jacobian entries");
}
} // namespace generated_correction_proof

#if FORMAL_ESKF_PROOF_CORRECTION_MODEL == 1
#define GENERATED_CROSS ins_horizontal_position_cross
#define GENERATED_INNOVATION ins_horizontal_position_innovation
#define GENERATED_JOSEPH ins_horizontal_position_joseph
#elif FORMAL_ESKF_PROOF_CORRECTION_MODEL == 2
#define GENERATED_CROSS ins_vertical_position_cross
#define GENERATED_INNOVATION ins_vertical_position_innovation
#define GENERATED_JOSEPH ins_vertical_position_joseph
#elif FORMAL_ESKF_PROOF_CORRECTION_MODEL == 3
#define GENERATED_CROSS ins_horizontal_velocity_cross
#define GENERATED_INNOVATION ins_horizontal_velocity_innovation
#define GENERATED_JOSEPH ins_horizontal_velocity_joseph
#elif FORMAL_ESKF_PROOF_CORRECTION_MODEL == 4
#define GENERATED_CROSS ins_velocity_cross
#define GENERATED_INNOVATION ins_velocity_innovation
#define GENERATED_JOSEPH ins_velocity_joseph
#else
#define GENERATED_CROSS ins_body_vector_cross
#define GENERATED_INNOVATION ins_body_vector_innovation
#define GENERATED_JOSEPH ins_body_vector_joseph
#endif

#if FORMAL_ESKF_PROOF_GENERATED_BOUNDARY
namespace formal_eskf::detail::generated
{
using namespace generated_correction_proof;
template <>
inline Gain GENERATED_CROSS<Backend>(Covariance const & P
#if FORMAL_ESKF_PROOF_CORRECTION_MODEL == 5
                                     ,
                                     Vector3 const & h
#endif
)
{
    __ESBMC_assert(Boundary::calls++ == 0U && same_matrix(P, *Boundary::P),
                   "Generated correction: selected cross kernel receives full original P exactly once");
#if FORMAL_ESKF_PROOF_CORRECTION_MODEL == 5
    check_body_vector(h);
#endif
    return Boundary::cross;
}
template <>
inline Noise GENERATED_INNOVATION<Backend>(Gain const & C,
#if FORMAL_ESKF_PROOF_CORRECTION_MODEL == 5
                                           Vector3 const & h,
#endif
                                           Noise const & V)
{
    __ESBMC_assert(Boundary::calls++ == 0U && same_matrix(C, *Boundary::K) && same_matrix(V, *Boundary::V),
                   "Generated correction: selected innovation kernel receives complete cross/V exactly once");
#if FORMAL_ESKF_PROOF_CORRECTION_MODEL == 5
    check_body_vector(h);
#endif
    return Boundary::innovation;
}
template <>
inline Covariance GENERATED_JOSEPH<Backend>(Covariance const & P, Covariance const & A, Gain const & K, Noise const & V)
{
    __ESBMC_assert(Boundary::calls++ == 0U && same_matrix(P, *Boundary::P) && same_matrix(A, *Boundary::A) &&
                       same_matrix(K, *Boundary::K) && same_matrix(V, *Boundary::V),
                   "Generated correction: selected Joseph kernel receives original P, full A/K/V exactly once");
    return Boundary::joseph;
}
} // namespace formal_eskf::detail::generated
#endif

using namespace generated_correction_proof;

void verify_generated_cross(Covariance P, Jacobian H, Gain result)
{
    auto const old_P = P;
    auto const old_H = H;
    Boundary::P = &old_P;
    Boundary::H = &old_H;
    Boundary::cross = result;
    Boundary::calls = 0U;
    auto const actual = formal_eskf::detail::correction_cross<model>(P, H);
#if FORMAL_ESKF_PROOF_GENERATED_BOUNDARY
    __ESBMC_assert(Boundary::calls == 1U && same_matrix(actual, result),
                   "Generated correction: cross dispatch publishes the selected kernel result unchanged");
#else
    (void)actual;
#endif
    __ESBMC_assert(same_matrix(P, old_P) && same_matrix(H, old_H), "Generated correction: cross complete input frames");
}

void verify_generated_innovation(Gain C, Jacobian H, Noise V, Noise result)
{
    auto const old_C = C;
    auto const old_H = H;
    auto const old_V = V;
    Boundary::K = &old_C;
    Boundary::H = &old_H;
    Boundary::V = &old_V;
    Boundary::innovation = result;
    Boundary::calls = 0U;
    auto const actual = formal_eskf::detail::correction_innovation<model>(C, H, V);
#if FORMAL_ESKF_PROOF_GENERATED_BOUNDARY
    __ESBMC_assert(Boundary::calls == 1U && same_matrix(actual, result),
                   "Generated correction: innovation dispatch publishes the selected kernel result unchanged");
#else
    (void)actual;
#endif
    __ESBMC_assert(same_matrix(C, old_C) && same_matrix(H, old_H) && same_matrix(V, old_V),
                   "Generated correction: innovation complete input frames");
}

void verify_generated_joseph(Covariance P, Covariance A, Gain K, Noise V, Covariance result)
{
    auto const old_P = P;
    auto const old_A = A;
    auto const old_K = K;
    auto const old_V = V;
    Boundary::P = &old_P;
    Boundary::A = &old_A;
    Boundary::K = &old_K;
    Boundary::V = &old_V;
    Boundary::joseph = result;
    Boundary::calls = 0U;
    auto const actual = formal_eskf::detail::correction_joseph<model>(P, A, K, V);
#if FORMAL_ESKF_PROOF_GENERATED_BOUNDARY
    __ESBMC_assert(Boundary::calls == 1U && same_matrix(actual, result),
                   "Generated correction: Joseph dispatch publishes the selected kernel result unchanged");
#else
    auto const & entries = cells(actual);
    for (std::size_t row = 0U; row < state_size; ++row)
    {
        for (std::size_t column = row + 1U; column < state_size; ++column)
        {
            __ESBMC_assert(same(entries[row][column], entries[column][row]),
                           "Generated correction: all Joseph off-diagonal pairs are mirrored");
        }
    }
#endif
    __ESBMC_assert(same_matrix(P, old_P) && same_matrix(A, old_A) && same_matrix(K, old_K) && same_matrix(V, old_V),
                   "Generated correction: Joseph complete input frames");
}

void verify_generated_publisher(Covariance candidate, Covariance output)
{
#if FORMAL_ESKF_PROOF_ALIAS
    auto & target = candidate;
    (void)output;
#else
    auto & target = output;
#endif
    auto const before = target;
    auto const input = candidate;
    bool const finite = finite_matrix(input);
    auto const status = formal_eskf::detail::publish_covariance(candidate, target);
    __ESBMC_assert(status == (finite ? Status::success : Status::non_finite_result),
                   "Generated correction: exact-type publisher rejects every nonfinite candidate");
    if (finite)
    {
        __ESBMC_assert(same_matrix(target, input), "Generated correction: publisher copies without averaging");
    }
    else
    {
        __ESBMC_assert(same_matrix(target, before), "Generated correction: publisher failure is atomic");
    }
    __ESBMC_assert(same_matrix(candidate, input), "Generated correction: publisher preserves candidate");
}

void verify_generated_reset_frame(Covariance P, formal_eskf::linalg::Matrix<Backend, 3U, 3U> J)
{
    auto const old_P = P;
    auto const old_J = J;
    (void)formal_eskf::detail::generated::ins_reset(P, J);
    __ESBMC_assert(same_matrix(P, old_P) && same_matrix(J, old_J),
                   "Generated correction: actual same-backend reset kernel input frames");
}

int main() { return 0; }
