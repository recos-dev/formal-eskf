/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "solve_support.hpp"
#include <formal_eskf/linalg/solve.hpp>

#ifndef FORMAL_ESKF_PROOF_SOLVE_BOUNDARY
#define FORMAL_ESKF_PROOF_SOLVE_BOUNDARY 0
#endif

namespace solve_proof
{
using SystemMatrix = Backend::matrix_type<size, size>;
using LeftMatrix = Backend::matrix_type<size, columns>;
using RightMatrix = Backend::matrix_type<columns, size>;
using formal_eskf::linalg::detail::MatrixAccess;
using Statuses = ScalarArray<Status, 2U * columns>;
extern double nondet_double();

Scalar fresh_scalar()
{
#if FORMAL_ESKF_PROOF_BINARY64
    return nondet_double();
#else
    return nondet_float();
#endif
}
template <std::size_t Count> bool finite_cells(ScalarArray<Scalar, Count> const & values)
{
    if constexpr (Count == 1U)
        return std::isfinite(values.head);
    else
        return std::isfinite(values.head) && finite_cells(values.tail);
}
template <typename Matrix> bool equal_matrix(Matrix const & a, Matrix const & b)
{
    return equal_cells(MatrixAccess::storage(a).values, MatrixAccess::storage(b).values);
}
struct Flow
{
    inline static Factor system{}, candidate_factor{};
    inline static Right rhs{}, work{};
    inline static Status factor_status{}, last_status{};
    inline static Statuses statuses{};
    inline static unsigned factor_calls{}, calls{};
    inline static Right const * rhs_input = nullptr;

    static Status substitute(Factor const & factor, Right & output, unsigned column, bool forward)
    {
        __ESBMC_assert(factor_calls == 1U && last_status == Status::success && calls < 2U * columns &&
                           column == calls / 2U && forward == (calls % 2U == 0U),
                       "F-SOLVE: per-column forward/backward sequence and stop on failure");
        __ESBMC_assume(calls < 2U * columns && column < columns);
        __ESBMC_assert(equal_cells(factor.values, candidate_factor.values),
                       "F-SOLVE: substitutions consume the computed factor");
        __ESBMC_assert(static_cast<void const *>(&factor) != static_cast<void const *>(&output),
                       "F-SOLVE: substitution factor and workspace are disjoint");
        __ESBMC_assert(equal_cells(output.values, work.values), "F-SOLVE: substitution receives previous workspace");
        // The producer proves that other columns and read-only inputs survive.
        // This overapproximates even failure: every cell of this column may
        // change, with no finite/coefficient/success premise.
        for (std::size_t row = 0U; row < size; ++row)
        {
            Scalar const value = fresh_scalar();
            output.values[row * columns + column] = value;
            work.values[row * columns + column] = value;
        }
        last_status = statuses[calls++];
        return last_status;
    }
};
struct SolveContract
{
    inline static Factor system{};
    inline static Right rhs{}, candidate{};
    inline static Status status{};
    inline static unsigned calls{};
};
struct SymmetryContract
{
    inline static SystemMatrix input{};
    inline static bool result{};
    inline static unsigned calls{};
};
} // namespace solve_proof

#if FORMAL_ESKF_PROOF_SOLVE_BOUNDARY == 1
namespace formal_eskf::linalg
{
template <>
template <>
inline Status Cholesky<solve_proof::Backend>::factorize<solve_proof::size>(solve_proof::Factor const & system,
                                                                           solve_proof::Factor & factor) noexcept
{
    using namespace solve_proof;
    __ESBMC_assert(Flow::factor_calls == 0U && Flow::calls == 0U,
                   "F-SOLVE: exactly one factorization, before substitutions");
    __ESBMC_assert(equal_cells(system.values, Flow::system.values), "F-SOLVE: factorization receives original system");
    __ESBMC_assert(&system != &factor &&
                       static_cast<void const *>(&factor) != static_cast<void const *>(Flow::rhs_input),
                   "F-SOLVE: factor workspace is disjoint from both original inputs");
    ++Flow::factor_calls;
    factor = Flow::candidate_factor;
    Flow::last_status = Flow::factor_status;
    return Flow::factor_status;
}
template <>
template <>
inline Status Cholesky<solve_proof::Backend>::forward_substitute<solve_proof::size, solve_proof::columns>(
    solve_proof::Factor const & factor, solve_proof::Right const & rhs, solve_proof::Right & output,
    std::size_t column) noexcept
{
    using namespace solve_proof;
    __ESBMC_assert(equal_cells(rhs.values, Flow::rhs.values), "F-SOLVE: forward receives original RHS");
    __ESBMC_assert(&rhs != &output && static_cast<void const *>(&rhs) != static_cast<void const *>(&factor),
                   "F-SOLVE: forward inputs and workspace meet producer disjointness");
    return Flow::substitute(factor, output, column, true);
}
template <>
template <>
inline Status Cholesky<solve_proof::Backend>::backward_substitute<solve_proof::size, solve_proof::columns>(
    solve_proof::Factor const & factor, solve_proof::Right & output, std::size_t column) noexcept
{
    return solve_proof::Flow::substitute(factor, output, column, false);
}
} // namespace formal_eskf::linalg
#elif FORMAL_ESKF_PROOF_SOLVE_BOUNDARY == 2
namespace formal_eskf::linalg
{
template <>
template <>
inline Status Cholesky<solve_proof::Backend>::solve<solve_proof::size, solve_proof::columns>(
    solve_proof::Factor const & system, solve_proof::Right const & rhs, solve_proof::Right & output) noexcept
{
    using namespace solve_proof;
    __ESBMC_assert(SolveContract::calls == 0U && SymmetryContract::calls == 1U && SymmetryContract::result,
                   "F-SOLVE: kernel called once, after symmetry validation");
    __ESBMC_assert(equal_cells(system.values, SolveContract::system.values) &&
                       equal_cells(rhs.values, SolveContract::rhs.values),
                   "F-SOLVE: kernel receives original system and RHS");
    __ESBMC_assert(&rhs != &output && static_cast<void const *>(&system) != static_cast<void const *>(&output),
                   "F-SOLVE: public wrapper gives kernel separate workspace");
    ++SolveContract::calls;
    output = SolveContract::candidate; // Even on failure, scratch may change.
    return SolveContract::status;
}
template <>
inline bool is_symmetric<solve_proof::Backend, solve_proof::size>(solve_proof::SystemMatrix const & input,
                                                                  solve_proof::Scalar tolerance) noexcept
{
    using namespace solve_proof;
    __ESBMC_assert(SymmetryContract::calls == 0U && equal_matrix(input, SymmetryContract::input) &&
                       same(tolerance, Scalar{0}),
                   "F-SOLVE: complete original system and zero symmetry tolerance");
    ++SymmetryContract::calls;
    return SymmetryContract::result;
}
} // namespace formal_eskf::linalg
#elif FORMAL_ESKF_PROOF_SOLVE_BOUNDARY == 3
namespace formal_eskf::linalg
{
template <>
inline Status
solve_spd<solve_proof::Backend, solve_proof::size, solve_proof::columns>(solve_proof::SystemMatrix const & system,
                                                                         solve_proof::LeftMatrix const & rhs,
                                                                         solve_proof::LeftMatrix & output) noexcept
{
    using namespace solve_proof;
    __ESBMC_assert(SolveContract::calls == 0U, "F-SOLVE: right solve delegates once");
    __ESBMC_assert(equal_cells(MatrixAccess::storage(system).values, SolveContract::system.values) &&
                       equal_cells(MatrixAccess::storage(rhs).values, SolveContract::rhs.values),
                   "F-SOLVE: right solve uses S and the complete transposed RHS");
    __ESBMC_assert(&rhs != &output && static_cast<void const *>(&system) != static_cast<void const *>(&output),
                   "F-SOLVE: transposed solve uses separate scratch output");
    ++SolveContract::calls;
    MatrixAccess::storage(output) = SolveContract::candidate;
    return SolveContract::status;
}
} // namespace formal_eskf::linalg
#endif

using namespace solve_proof;

void verify_solve_orchestration(Factor system, Right rhs, Right output, Factor factor_candidate, Status factor_status,
                                Statuses statuses)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_SOLVE_BOUNDARY == 1 && !FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT,
                   "Runner error: orchestration requires only helper summaries");
    __ESBMC_assert(FORMAL_ESKF_PROOF_ALIAS <= 1, "Runner error: kernel supports separate/identical read-only inputs");
    Factor const old_system = system;
    Right const old_rhs = rhs;
#if FORMAL_ESKF_PROOF_SOLVE_BOUNDARY == 1 && FORMAL_ESKF_PROOF_ALIAS
    static_assert(size == columns);
    auto const & requested_rhs = system;
#else
    auto const & requested_rhs = rhs;
#endif
    Flow::system = system;
    Flow::rhs = requested_rhs;
    Flow::work = output;
    Flow::rhs_input = &requested_rhs;
    Flow::candidate_factor = factor_candidate;
    Flow::factor_status = factor_status;
    Flow::statuses = statuses;
    Flow::factor_calls = 0U;
    Flow::calls = 0U;
    Status expected = factor_status;
    unsigned count = 0U;
    if (expected == Status::success)
    {
        for (unsigned step = 0U; step < 2U * columns; ++step)
        {
            ++count;
            expected = statuses[step];
            if (expected != Status::success)
                break;
        }
    }
    Status const actual = Backend::solve_spd<size, columns>(system, requested_rhs, output);
    __ESBMC_assert(actual == expected && Flow::factor_calls == 1U && Flow::calls == count,
                   "F-SOLVE: exact composition status and call count");
    __ESBMC_assert(equal_cells(output.values, Flow::work.values),
                   "F-SOLVE: publishes the final workspace, or preserves it on factor failure");
    __ESBMC_assert(equal_cells(system.values, old_system.values) && equal_cells(rhs.values, old_rhs.values),
                   "F-SOLVE: orchestration preserves both original input objects");
}

void verify_left_solve(SystemMatrix system, LeftMatrix rhs, LeftMatrix output, Right candidate, Status backend_status,
                       bool symmetric)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_SOLVE_BOUNDARY == 2 && !FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT,
                   "Runner error: left solve requires kernel and symmetry summaries");
    static_assert(FORMAL_ESKF_PROOF_ALIAS <= 4);
    auto const old_system = system;
    auto const old_rhs = rhs;
    auto const old_output = output;
#if FORMAL_ESKF_PROOF_ALIAS >= 3
    static_assert(size == columns);
    auto const & requested_rhs = system;
#else
    auto const & requested_rhs = rhs;
#endif
#if FORMAL_ESKF_PROOF_ALIAS == 1
    auto & target = rhs;
#elif FORMAL_ESKF_PROOF_ALIAS == 2 || FORMAL_ESKF_PROOF_ALIAS == 4
    static_assert(size == columns);
    auto & target = system;
#else
    auto & target = output;
#endif
    auto const before = target;
    SolveContract::system = MatrixAccess::storage(system);
    SolveContract::rhs = MatrixAccess::storage(requested_rhs);
    SolveContract::candidate = candidate;
    SolveContract::status = backend_status;
    SolveContract::calls = 0U;
    SymmetryContract::input = system;
    SymmetryContract::result = symmetric;
    SymmetryContract::calls = 0U;
    bool const finite_input = finite_cells(SolveContract::system.values) && finite_cells(SolveContract::rhs.values);
    Status const expected = !finite_input                       ? Status::non_finite_input
                            : !symmetric                        ? Status::not_positive_definite
                            : backend_status != Status::success ? backend_status
                            : !finite_cells(candidate.values)   ? Status::non_finite_result
                                                                : Status::success;
    Status const actual = formal_eskf::linalg::solve_spd(system, requested_rhs, target);
    __ESBMC_assert(actual == expected && SymmetryContract::calls == (finite_input ? 1U : 0U) &&
                       SolveContract::calls == (finite_input && symmetric ? 1U : 0U),
                   "F-SOLVE: left validation, status precedence and call eligibility");
    __ESBMC_assert(actual == Status::success ? equal_cells(MatrixAccess::storage(target).values, candidate.values)
                                             : equal_matrix(target, before),
                   "F-SOLVE: complete left publication or rollback under every alias arrangement");
#if FORMAL_ESKF_PROOF_ALIAS != 2 && FORMAL_ESKF_PROOF_ALIAS != 4
    __ESBMC_assert(equal_matrix(system, old_system), "F-SOLVE: non-output system unchanged");
#endif
#if FORMAL_ESKF_PROOF_ALIAS != 1
    __ESBMC_assert(equal_matrix(rhs, old_rhs), "F-SOLVE: non-output RHS unchanged");
#endif
#if FORMAL_ESKF_PROOF_ALIAS != 0 && FORMAL_ESKF_PROOF_ALIAS != 3
    __ESBMC_assert(equal_matrix(output, old_output), "F-SOLVE: unused output unchanged");
#endif
}

void verify_right_solve(SystemMatrix system, RightMatrix rhs, RightMatrix output, Right candidate, Status left_status)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_SOLVE_BOUNDARY == 3 && !FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT,
                   "Runner error: right solve requires only left solve summary");
    static_assert(FORMAL_ESKF_PROOF_ALIAS <= 4);
    auto const old_system = system;
    auto const old_rhs = rhs;
    auto const old_output = output;
#if FORMAL_ESKF_PROOF_ALIAS >= 3
    static_assert(size == columns);
    auto const & requested_rhs = system;
#else
    auto const & requested_rhs = rhs;
#endif
#if FORMAL_ESKF_PROOF_ALIAS == 1
    auto & target = rhs;
#elif FORMAL_ESKF_PROOF_ALIAS == 2 || FORMAL_ESKF_PROOF_ALIAS == 4
    static_assert(size == columns);
    auto & target = system;
#else
    auto & target = output;
#endif
    auto const before = target;
    SolveContract::system = MatrixAccess::storage(system);
    SolveContract::candidate = candidate;
    SolveContract::status = left_status;
    SolveContract::calls = 0U;
    for (std::size_t i = 0U; i < size; ++i)
        for (std::size_t j = 0U; j < columns; ++j)
            SolveContract::rhs.values[i * columns + j] = requested_rhs(j, i);
    Status const actual = formal_eskf::linalg::right_solve_spd(requested_rhs, system, target);
    __ESBMC_assert(actual == left_status && SolveContract::calls == 1U, "F-SOLVE: right status propagation");
    for (std::size_t i = 0U; i < columns; ++i)
        for (std::size_t j = 0U; j < size; ++j)
            __ESBMC_assert(
                same(target(i, j), actual == Status::success ? candidate.values[j * columns + i] : before(i, j)),
                "F-SOLVE: complete transposed publication or rollback under every alias arrangement");
#if FORMAL_ESKF_PROOF_ALIAS != 2 && FORMAL_ESKF_PROOF_ALIAS != 4
    __ESBMC_assert(equal_matrix(system, old_system), "F-SOLVE: right non-output system unchanged");
#endif
#if FORMAL_ESKF_PROOF_ALIAS != 1
    __ESBMC_assert(equal_matrix(rhs, old_rhs), "F-SOLVE: right non-output RHS unchanged");
#endif
#if FORMAL_ESKF_PROOF_ALIAS != 0 && FORMAL_ESKF_PROOF_ALIAS != 3
    __ESBMC_assert(equal_matrix(output, old_output), "F-SOLVE: right unused output unchanged");
#endif
}

void verify_solve_symmetry(SystemMatrix input)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_SOLVE_BOUNDARY == 0 && !FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT,
                   "Runner error: symmetry producer must be actual");
    auto const before = input;
    bool expected = finite_cells(MatrixAccess::storage(input).values);
    for (std::size_t i = 0U; i < size; ++i)
        for (std::size_t j = i + 1U; j < size; ++j)
            expected = expected && input(i, j) == input(j, i);
    // Each cutpoint is asserted before being assumed; all assertions are
    // checked. This separates pairwise IEEE facts from the max-abs reduction.
    auto const difference = input - formal_eskf::linalg::transpose(input);
    for (std::size_t i = 0U; i < size; ++i)
    {
        for (std::size_t j = 0U; j < size; ++j)
        {
            bool const zero_law = !std::isfinite(input(i, j)) || !std::isfinite(input(j, i)) ||
                                  ((difference(i, j) == Scalar{0}) == (input(i, j) == input(j, i)));
            __ESBMC_assert(zero_law, "F-SOLVE: finite IEEE subtraction is zero exactly for equal operands");
            __ESBMC_assume(zero_law);
        }
    }
    __ESBMC_assert(formal_eskf::linalg::is_symmetric(input, Scalar{0}) == expected,
                   "F-SOLVE: exact-zero symmetry predicate on arbitrary IEEE entries");
    __ESBMC_assert(equal_matrix(input, before), "F-SOLVE: symmetry input unchanged");
}

int main() { return 0; }
