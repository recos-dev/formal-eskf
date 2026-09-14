/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "contracts.hpp"

#include <formal_eskf/so3/right_jacobian.hpp>

#ifndef FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT
#define FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT 0
#endif
#ifndef FORMAL_ESKF_PROOF_FINITE_CONTRACT
#define FORMAL_ESKF_PROOF_FINITE_CONTRACT 0
#endif
#ifndef FORMAL_ESKF_PROOF_JACOBIAN_BRANCH
#define FORMAL_ESKF_PROOF_JACOBIAN_BRANCH -1
#endif

namespace jacobian_proof
{
using namespace contract_proof;
using Matrix3 = Linalg::matrix_type<3U, 3U>;

// The same actual three-vector reduction is proved in maps.cpp. Its return
// is arbitrary here: no positivity, finiteness or successful branch is assumed.
struct SquaredNormContract
{
    inline static Scalar result{};
    inline static Vector3 received{};
    inline static unsigned calls = 0U;
};

// Separate candidate construction from its final predicate. The actual
// predicate is proved below for every matrix, not assumed to return true.
struct FiniteContract
{
    inline static bool result = false;
    inline static Matrix3 received{};
    inline static unsigned calls = 0U;
};

} // namespace jacobian_proof

#if FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT
namespace formal_eskf::linalg
{
template <> inline contract_proof::Scalar squared_norm(contract_proof::Vector3 const & input) noexcept
{
    using jacobian_proof::SquaredNormContract;
    __ESBMC_assert(SquaredNormContract::calls == 0U, "F-JR contract: one squared-norm request");
    ++SquaredNormContract::calls;
    SquaredNormContract::received = input;
    return SquaredNormContract::result;
}
} // namespace formal_eskf::linalg
#endif

#if FORMAL_ESKF_PROOF_FINITE_CONTRACT
namespace formal_eskf::linalg
{
template <> inline bool all_finite(jacobian_proof::Matrix3 const & input) noexcept
{
    using jacobian_proof::FiniteContract;
    __ESBMC_assert(FiniteContract::calls == 0U, "F-JR contract: one complete candidate check");
    ++FiniteContract::calls;
    FiniteContract::received = input;
    return FiniteContract::result;
}
} // namespace formal_eskf::linalg
#endif

using namespace jacobian_proof;

// Actual public operation, checked scalar wrappers and matrix arithmetic.
// Squared norm, final finite predicate and primitive sqrt/sin/cos returns are
// opaque. The norm/predicate producers are proved independently. The flow
// profile has no assumptions; two entry profiles use exactly its proved
// candidate eligibility, never an assumed finite candidate or accurate libm.
// Their independent oracle follows RightJacobianTaylor/rightJacobian_halfAngle
// with IEEE evaluation order, not exact-real reassociation.
void verify_right_jacobian(Vector3 phi, Matrix3 output, Scalar squared, Scalar root, Scalar sine, Scalar cosine,
                           bool finite_candidate)
{
    constexpr bool configured = FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT && FORMAL_ESKF_PROOF_FINITE_CONTRACT &&
                                !FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT;
    __ESBMC_assert(configured, "Runner error: right Jacobian requires norm and final-predicate summaries");
    if constexpr (!configured)
    {
        return; // The failed guard is decisive; do not solve an invalid setup.
    }
    bool const regular = std::isfinite(squared) && squared > Scalar{1} / Scalar{256};
    static_assert(FORMAL_ESKF_PROOF_JACOBIAN_BRANCH >= -1 && FORMAL_ESKF_PROOF_JACOBIAN_BRANCH <= 1);
#if FORMAL_ESKF_PROOF_JACOBIAN_BRANCH >= 0
    // Coefficient claims use precisely the conditions for reaching the final
    // predicate. The unpartitioned flow profile proves that eligibility and
    // every rejection; these two profiles cover both eligible branches.
    __ESBMC_assume(finite_vector(phi) && std::isfinite(squared) && squared >= Scalar{0});
#if FORMAL_ESKF_PROOF_JACOBIAN_BRANCH == 0
    __ESBMC_assume(!regular);
#else
    __ESBMC_assume(regular && std::isfinite(root) && std::isfinite(sine) && std::isfinite(cosine));
#endif
#endif
    auto const phi_before = phi;
    auto const before = output;
    SquaredNormContract::result = squared;
    SquaredNormContract::calls = 0U;
    FiniteContract::result = finite_candidate;
    FiniteContract::calls = 0U;
    OpaqueMath::prepare(root, sine, cosine, Scalar{0});

    bool const finite_input = finite_vector(phi_before);
    Status expected = Status::success;
    bool root_call = false;
    bool trig_call = false;
    if (!finite_input)
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
    else if (regular)
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
        }
    }

#if FORMAL_ESKF_PROOF_JACOBIAN_BRANCH >= 0
    Scalar axis[3U]{phi_before(0U), phi_before(1U), phi_before(2U)};
    Scalar identity{}, outer{}, skew{};
    if constexpr (FORMAL_ESKF_PROOF_JACOBIAN_BRANCH == 0)
    {
        skew = Scalar{0.5} -
               squared * (Scalar{1} / Scalar{24} - squared * (Scalar{1} / Scalar{720} - squared / Scalar{40320}));
        outer = Scalar{1} / Scalar{6} -
                squared * (Scalar{1} / Scalar{120} - squared * (Scalar{1} / Scalar{5040} - squared / Scalar{362880}));
        identity = Scalar{1} - squared * outer;
    }
    else
    {
        for (std::size_t axis_index = 0U; axis_index < 3U; ++axis_index)
        {
            axis[axis_index] = phi_before(axis_index) / root;
        }
        identity = (Scalar{2} * sine * cosine) / root;
        outer = Scalar{1} - identity;
        skew = (Scalar{2} * sine * sine) / root;
    }
    Matrix3 entries;
    Scalar const hat_entries[3U][3U]{
        {Scalar{0}, -axis[2U], axis[1U]}, {axis[2U], Scalar{0}, -axis[0U]}, {-axis[1U], axis[0U], Scalar{0}}};
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            // The actual 3x1 * 1x3 reduction starts at +0, including for
            // signed-zero products. Do not silently replace it with a*b.
            Scalar value = ((Scalar{0} + axis[row] * axis[column]) * outer) - hat_entries[row][column] * skew;
            if (row == column)
            {
                value = value + identity;
            }
            entries(row, column) = value;
        }
    }
#endif
    bool const candidate_call = expected == Status::success;
    if (candidate_call && !finite_candidate)
    {
        expected = Status::non_finite_result;
    }
    auto const actual = formal_eskf::so3::try_right_jacobian(phi, output);
    __ESBMC_assert(FiniteContract::calls == (candidate_call ? 1U : 0U),
                   "F-JR: final predicate is requested exactly after valid input and scalar results");
    __ESBMC_assert(actual == expected, "F-JR: input, squared-norm, scalar and complete candidate status");
    __ESBMC_assert(SquaredNormContract::calls == (finite_input ? 1U : 0U),
                   "F-JR: squared norm is requested exactly after input validation");
    if (finite_input)
    {
        __ESBMC_assert(same_vector(SquaredNormContract::received, phi_before),
                       "F-JR: squared norm receives all inputs");
    }
    __ESBMC_assert(OpaqueMath::sqrt.calls == (root_call ? 1U : 0U) && OpaqueMath::sin.calls == (trig_call ? 1U : 0U) &&
                       OpaqueMath::cos.calls == (trig_call ? 1U : 0U),
                   "F-JR: equality selects Taylor; regular branch dispatches checked root and both trig calls");
    if (root_call)
    {
        __ESBMC_assert(same(OpaqueMath::sqrt.argument, squared), "F-JR: root receives the squared norm");
    }
    if (trig_call)
    {
        __ESBMC_assert(same(OpaqueMath::sin.argument, root * Scalar{0.5}) &&
                           same(OpaqueMath::cos.argument, root * Scalar{0.5}),
                       "F-JR: both trigonometric functions receive the same half angle");
    }
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
#if FORMAL_ESKF_PROOF_JACOBIAN_BRANCH >= 0
            __ESBMC_assert(FiniteContract::calls == 1U &&
                               same(FiniteContract::received(row, column), entries(row, column)),
                           "F-JR: each candidate entry matches the ordered branch equations");
#endif
            __ESBMC_assert(actual != Status::success ||
                               same(output(row, column), FiniteContract::received(row, column)),
                           "F-JR: each successful output entry is the checked candidate");
            __ESBMC_assert(actual == Status::success || same(output(row, column), before(row, column)),
                           "F-JR: each old output value is preserved on failure");
        }
    }
    __ESBMC_assert(same_vector(phi, phi_before), "F-JR: all input coefficients are unchanged");
}

// Actual predicate for the exact same backend/scalar and shape used above.
void verify_right_jacobian_finite(Matrix3 matrix)
{
    __ESBMC_assert(!FORMAL_ESKF_PROOF_FINITE_CONTRACT && !FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT &&
                       !FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT,
                   "Runner error: matrix-finite producer must be actual");
    auto const before = matrix;
    bool expected = true;
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            expected = expected && std::isfinite(before(row, column));
        }
    }
    __ESBMC_assert(formal_eskf::linalg::all_finite(matrix) == expected,
                   "F-JR dependency: actual predicate checks all nine IEEE entries");
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            __ESBMC_assert(same(matrix(row, column), before(row, column)),
                           "F-JR dependency: matrix-finite producer preserves every entry");
        }
    }
}

// Actual norm and matrix operations: zero identity and reachable branch sides.
// These exact dyadic witnesses complement, not replace, the all-IEEE proof.
void verify_right_jacobian_boundaries()
{
    __ESBMC_assert(!FORMAL_ESKF_PROOF_SQUARED_NORM_CONTRACT && !FORMAL_ESKF_PROOF_FINITE_CONTRACT &&
                       !FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT,
                   "Runner error: right-Jacobian boundary witnesses require the actual squared norm");
    Vector3 phi;
    Matrix3 output;
    OpaqueMath::prepare(Scalar{0}, Scalar{0}, Scalar{0}, Scalar{0});
    auto actual = formal_eskf::so3::try_right_jacobian(phi, output);
    __ESBMC_assert(actual == Status::success && OpaqueMath::sqrt.calls == 0U && OpaqueMath::sin.calls == 0U &&
                       OpaqueMath::cos.calls == 0U,
                   "F-JR: zero succeeds without nonlinear scalar calls");
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            __ESBMC_assert(same(output(row, column), row == column ? Scalar{1} : Scalar{0}),
                           "F-JR: J_r(0) is identity");
        }
    }
    for (unsigned side = 0U; side < 3U; ++side)
    {
#if FORMAL_ESKF_PROOF_BINARY64
        Scalar const values[]{Scalar{0x1.fffffffffffffp-5}, Scalar{0x1p-4}, Scalar{0x1.0000000000001p-4}};
#else
        Scalar const values[]{Scalar{0x1.fffffep-5}, Scalar{0x1p-4}, Scalar{0x1.000002p-4}};
#endif
        phi(0U) = values[side];
        Scalar const squared = formal_eskf::linalg::squared_norm(phi);
        __ESBMC_assert(side == 0U   ? squared < Scalar{1} / Scalar{256}
                       : side == 1U ? squared == Scalar{1} / Scalar{256}
                                    : squared > Scalar{1} / Scalar{256},
                       "F-JR: adjacent axis inputs reach below, exact and above the Taylor cutoff");
        // Finite dispatch probes, not accuracy witnesses for sin/cos.
        OpaqueMath::prepare(phi(0U), Scalar{0.5}, Scalar{0.5}, Scalar{0});
        actual = formal_eskf::so3::try_right_jacobian(phi, output);
        unsigned const calls = side == 2U ? 1U : 0U;
        __ESBMC_assert(actual == Status::success && OpaqueMath::sqrt.calls == calls && OpaqueMath::sin.calls == calls &&
                           OpaqueMath::cos.calls == calls,
                       "F-JR: executable cutoff witnesses select Taylor inclusively and regular exclusively");
    }
}

int main() { return 0; }
