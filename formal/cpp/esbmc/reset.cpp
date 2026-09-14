/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "covariance_oracle.hpp"

#include <formal_eskf/eskf/injection.hpp>

#ifndef FORMAL_ESKF_PROOF_INS
#define FORMAL_ESKF_PROOF_INS 0
#endif
#ifndef FORMAL_ESKF_PROOF_RESET_CONTRACT
#define FORMAL_ESKF_PROOF_RESET_CONTRACT 0
#endif
#ifndef FORMAL_ESKF_PROOF_FINITE_CONTRACT
#define FORMAL_ESKF_PROOF_FINITE_CONTRACT 0
#endif
#ifndef FORMAL_ESKF_PROOF_PRODUCT_CONTRACT
#define FORMAL_ESKF_PROOF_PRODUCT_CONTRACT 0
#endif
namespace reset_proof
{
using namespace contract_proof;
static_assert(FORMAL_ESKF_PROOF_INS == 0 || FORMAL_ESKF_PROOF_INS == 1);
static_assert(FORMAL_ESKF_PROOF_ALIAS == 0 || FORMAL_ESKF_PROOF_ALIAS == 1);
constexpr std::size_t size = FORMAL_ESKF_PROOF_INS ? 15U : 3U;
constexpr std::size_t attitude_offset = FORMAL_ESKF_PROOF_INS ? 6U : 0U;
using Matrix = Linalg::matrix_type<size, size>;
using Matrix3 = Linalg::matrix_type<3U, 3U>;
#if FORMAL_ESKF_PROOF_INS
using Error = formal_eskf::configuration::Ins::ErrorState<Linalg>;
#else
using Error = formal_eskf::configuration::Ahrs::ErrorState<Linalg>;
#endif

using namespace covariance_oracle;

bool same_error(Error const & a, Error const & b)
{
    bool valid = same_vector(a.delta_theta_b, b.delta_theta_b);
#if FORMAL_ESKF_PROOF_INS
    valid = same_vector(a.delta_p_n, b.delta_p_n) && same_vector(a.delta_v_n, b.delta_v_n) &&
            same_vector(a.delta_b_a, b.delta_b_a) && same_vector(a.delta_b_g, b.delta_b_g) && valid;
#endif
    return valid;
}

#if FORMAL_ESKF_PROOF_RESET_CONTRACT
// Pure same-type callee summaries, paired with actual producers. No successful,
// finite, symmetric or PSD result is assumed. The reset caller must validate
// its inputs, construct every G entry and preserve the entire covariance/error
// frame. F-JR discharges the first boundary; the two producers below discharge
// the other boundaries without substituting any matrix arithmetic.
struct JacobianContract
{
    inline static Matrix3 result{};
    inline static Vector3 received{};
    inline static Status status{};
    inline static unsigned calls = 0U;
};

struct SandwichContract
{
    inline static Matrix result{}, transform{}, covariance{};
    inline static unsigned calls = 0U;
};

struct FinishContract
{
    inline static Matrix result{}, received{};
    inline static Status status{};
    inline static unsigned calls = 0U;
};
#endif
#if FORMAL_ESKF_PROOF_FINITE_CONTRACT
struct FiniteContract
{
    inline static Matrix received{};
    inline static bool result = false;
    inline static unsigned calls = 0U;
};
#endif
#if FORMAL_ESKF_PROOF_PRODUCT_CONTRACT
struct ProductContract
{
    inline static Matrix first_left{}, first_right{}, second_left{}, second_right{};
    inline static Matrix const * first_result = nullptr;
    inline static Matrix const * second_result = nullptr;
    inline static unsigned calls = 0U;
};
#endif
} // namespace reset_proof

#if FORMAL_ESKF_PROOF_PRODUCT_CONTRACT
namespace formal_eskf::linalg
{
template <>
template <>
inline reset_proof::Matrix Matrix<contract_proof::Linalg, reset_proof::size, reset_proof::size>::operator*
    <reset_proof::size>(reset_proof::Matrix const & other) const noexcept
{
    using reset_proof::ProductContract;
    __ESBMC_assert(ProductContract::calls < 2U, "E-RESET product contract: two matrix products in the sandwich");
    if (ProductContract::calls++ == 0U)
    {
        ProductContract::first_left = *this;
        ProductContract::first_right = other;
        return *ProductContract::first_result;
    }
    ProductContract::second_left = *this;
    ProductContract::second_right = other;
    return *ProductContract::second_result;
}
} // namespace formal_eskf::linalg
#endif

#if FORMAL_ESKF_PROOF_FINITE_CONTRACT
namespace formal_eskf::linalg
{
template <> inline bool all_finite(reset_proof::Matrix const & matrix) noexcept
{
    using reset_proof::FiniteContract;
    __ESBMC_assert(FiniteContract::calls == 0U, "E-RESET finalizer contract: exactly one complete candidate check");
    ++FiniteContract::calls;
    FiniteContract::received = matrix;
    return FiniteContract::result;
}
} // namespace formal_eskf::linalg
#endif

#if FORMAL_ESKF_PROOF_RESET_CONTRACT
namespace formal_eskf::so3
{
template <>
inline Status try_right_jacobian<contract_proof::Linalg>(contract_proof::Vector3 const & input,
                                                         reset_proof::Matrix3 & output) noexcept
{
    using reset_proof::JacobianContract;
    __ESBMC_assert(JacobianContract::calls == 0U, "E-RESET contract: at most one right Jacobian");
    ++JacobianContract::calls;
    JacobianContract::received = input;
    if (JacobianContract::status == Status::success)
    {
        output = JacobianContract::result;
    }
    return JacobianContract::status;
}
} // namespace formal_eskf::so3

namespace formal_eskf::linalg
{
template <>
inline reset_proof::Matrix sandwich(reset_proof::Matrix const & transform,
                                    reset_proof::Matrix const & covariance) noexcept
{
    using reset_proof::SandwichContract;
    __ESBMC_assert(SandwichContract::calls == 0U, "E-RESET contract: at most one full covariance sandwich");
    ++SandwichContract::calls;
    SandwichContract::transform = transform;
    SandwichContract::covariance = covariance;
    return SandwichContract::result;
}
} // namespace formal_eskf::linalg

namespace formal_eskf::detail
{
template <> inline Status try_finish_covariance(reset_proof::Matrix & candidate, reset_proof::Matrix & output) noexcept
{
    using reset_proof::FinishContract;
    __ESBMC_assert(FinishContract::calls == 0U && reset_proof::SandwichContract::calls == 1U,
                   "E-RESET contract: one finalizer after the sandwich");
    ++FinishContract::calls;
    FinishContract::received = candidate;
    if (FinishContract::status == Status::success)
    {
        candidate = FinishContract::result;
        output = FinishContract::result;
    }
    return FinishContract::status;
}
} // namespace formal_eskf::detail
#endif

using namespace reset_proof;

// Actual row-major storage/access, zero initialization, copy and setter frame.
// Bounds are the documented coefficient-access precondition, not a numerical
// operating-range assumption.
void verify_reset_storage(Matrix source, Matrix output, std::size_t row, std::size_t column, Scalar replacement)
{
    constexpr bool configured = FORMAL_ESKF_PROOF_INS && !FORMAL_ESKF_PROOF_RESET_CONTRACT &&
                                !FORMAL_ESKF_PROOF_FINITE_CONTRACT && !FORMAL_ESKF_PROOF_ACCESS_CONTRACT;
    __ESBMC_assert(configured, "Runner error: reset storage producer requires actual INS storage");
    if constexpr (!configured)
    {
        return;
    }
#if FORMAL_ESKF_PROOF_INS
    __ESBMC_assume(row < size && column < size);
    auto const original = source;
    auto const copy = source;
    output = source;
    __ESBMC_assert(same_matrix(copy, original) && same_matrix(output, original),
                   "E-RESET storage: actual complete copy construction and assignment");
    auto const zero = Matrix::zero();
    Scalar zeros[size * size]{};
    flatten(zero, zeros);
    for (std::size_t i = 0U; i < size * size; ++i)
    {
        __ESBMC_assert(same(zeros[i], Scalar{0}), "E-RESET storage: actual initialization of every positive-zero cell");
    }
    auto const index = row * size + column;
    Scalar before[size * size]{}, after[size * size]{};
    flatten(original, before);
    auto const & const_source = source;
    __ESBMC_assert(same(source(row, column), before[index]) && same(const_source(row, column), before[index]),
                   "E-RESET storage: actual mutable and const access uses row-major coordinates");
    source.set(row, column, replacement);
    flatten(source, after);
    bool frame = true;
    for (std::size_t i = 0U; i < size * size; ++i)
    {
        frame = same(after[i], i == index ? replacement : before[i]) && frame;
    }
    __ESBMC_assert(frame, "E-RESET storage: actual setter changes exactly the requested cell");
    __ESBMC_assert(same_matrix(output, original), "E-RESET storage: separate copied output has no shared cells");
#else
    (void)source;
    (void)output;
    (void)row;
    (void)column;
    (void)replacement;
#endif
}

void verify_reset(Error error, Matrix covariance, Matrix output, Matrix3 jacobian, Status jacobian_status,
                  Matrix product, Matrix finished, Status finish_status, bool covariance_finite)
{
    constexpr bool configured = FORMAL_ESKF_PROOF_RESET_CONTRACT && !FORMAL_ESKF_PROOF_CONSTRUCTOR_CONTRACT &&
                                FORMAL_ESKF_PROOF_FINITE_CONTRACT;
    __ESBMC_assert(configured, "Runner error: reset caller requires its explicit callee summaries");
    if constexpr (!configured)
    {
        return;
    }
#if FORMAL_ESKF_PROOF_RESET_CONTRACT && FORMAL_ESKF_PROOF_FINITE_CONTRACT
    (void)output;
    auto const error_before = error;
    auto const covariance_before = covariance;
#if FORMAL_ESKF_PROOF_ALIAS
    auto & target = covariance;
#else
    auto & target = output;
#endif
    auto const before = target;
    bool const call_finite = finite_vector(error_before.delta_theta_b);
    bool const valid = call_finite && covariance_finite;
    bool const call_jacobian = valid && !ESKF_RESET_APPROX;
    bool const call_sandwich = valid && (ESKF_RESET_APPROX || jacobian_status == Status::success);
    Status const expected = !valid ? Status::non_finite_input : call_sandwich ? finish_status : jacobian_status;
    JacobianContract::result = jacobian;
    JacobianContract::status = jacobian_status;
    JacobianContract::calls = 0U;
    SandwichContract::result = product;
    SandwichContract::calls = 0U;
    FinishContract::result = finished;
    FinishContract::status = finish_status;
    FinishContract::calls = 0U;
    FiniteContract::result = covariance_finite;
    FiniteContract::calls = 0U;

    auto const actual = formal_eskf::try_reset_covariance(error, covariance, target);
    __ESBMC_assert(actual == expected, "E-RESET: consumed-input validation and complete callee status propagation");
    __ESBMC_assert(FiniteContract::calls == (call_finite ? 1U : 0U),
                   "E-RESET: covariance is classified exactly after the consumed attitude error");
    if (call_finite)
    {
        __ESBMC_assert(same_matrix(FiniteContract::received, covariance_before),
                       "E-RESET: the complete prior covariance is checked, without other error components");
    }
    __ESBMC_assert(JacobianContract::calls == (call_jacobian ? 1U : 0U) &&
                       SandwichContract::calls == (call_sandwich ? 1U : 0U) &&
                       FinishContract::calls == (call_sandwich ? 1U : 0U),
                   "E-RESET: exact mode selection, call eligibility and finalization");
    if (call_jacobian)
    {
        __ESBMC_assert(same_vector(JacobianContract::received, error_before.delta_theta_b),
                       "E-RESET: right Jacobian receives the pre-reset local attitude correction");
    }
    if (call_sandwich)
    {
        // Literal signed entries, not a call to production hat/set_block.
        Scalar const x = error_before.delta_theta_b(0U) * Scalar{0.5};
        Scalar const y = error_before.delta_theta_b(1U) * Scalar{0.5};
        Scalar const z = error_before.delta_theta_b(2U) * Scalar{0.5};
        Scalar const skew[3U][3U]{{Scalar{0}, -z, y}, {z, Scalar{0}, -x}, {-y, x, Scalar{0}}};
        for (std::size_t row = 0U; row < size; ++row)
        {
            for (std::size_t column = 0U; column < size; ++column)
            {
                Scalar entry = row == column ? Scalar{1} : Scalar{0};
                if (row >= attitude_offset && row < attitude_offset + 3U && column >= attitude_offset &&
                    column < attitude_offset + 3U)
                {
                    entry = ESKF_RESET_APPROX ? entry - skew[row - attitude_offset][column - attitude_offset]
                                              : jacobian(row - attitude_offset, column - attitude_offset);
                }
                __ESBMC_assert(same(SandwichContract::transform(row, column), entry),
                               "E-RESET: every entry of G, including identity and all attitude cross-block positions");
            }
        }
        __ESBMC_assert(same_matrix(SandwichContract::covariance, covariance_before),
                       "E-RESET: the entire prior covariance is forwarded, with no dropped cross-covariances");
        __ESBMC_assert(same_matrix(FinishContract::received, product),
                       "E-RESET: the complete sandwich result is finalized");
    }
    if (actual == Status::success)
    {
        __ESBMC_assert(same_matrix(target, finished), "E-RESET: every successful output coefficient is published");
    }
    else
    {
        __ESBMC_assert(same_matrix(target, before), "E-RESET: every output coefficient is preserved on failure");
    }
    __ESBMC_assert(same_error(error, error_before), "E-RESET: every error component is read-only, never zeroed here");
#if !FORMAL_ESKF_PROOF_ALIAS
    __ESBMC_assert(same_matrix(covariance, covariance_before), "E-RESET: distinct prior covariance is unchanged");
#endif
#else
    (void)error;
    (void)covariance;
    (void)output;
    (void)jacobian;
    (void)jacobian_status;
    (void)product;
    (void)finished;
    (void)finish_status;
    (void)covariance_finite;
#endif
}

// Pure product summaries allow the two ordered matrix products to be proved
// once, independently of sandwich wiring. Both results are arbitrary IEEE
// matrices. Actual transpose/copy/storage execute here.
void verify_reset_sandwich(Matrix transform, Matrix covariance, Matrix first, Matrix second)
{
    constexpr bool configured = !FORMAL_ESKF_PROOF_RESET_CONTRACT && FORMAL_ESKF_PROOF_PRODUCT_CONTRACT;
    __ESBMC_assert(configured, "Runner error: actual sandwich requires only its product summary");
    if constexpr (!configured)
    {
        return;
    }
#if FORMAL_ESKF_PROOF_PRODUCT_CONTRACT
    auto const transform_before = transform;
    auto const covariance_before = covariance;
    ProductContract::first_result = &first;
    ProductContract::second_result = &second;
    ProductContract::calls = 0U;
    auto const actual = formal_eskf::linalg::sandwich(transform, covariance);
    __ESBMC_assert(ProductContract::calls == 2U && same_matrix(ProductContract::first_left, transform_before) &&
                       same_matrix(ProductContract::first_right, covariance_before) &&
                       same_matrix(ProductContract::second_left, first) && same_matrix(actual, second),
                   "E-RESET sandwich: ordered G times the entire P, then returned product times G transpose");
    Scalar g[size * size]{}, transposed[size * size]{};
    flatten(transform_before, g);
    flatten(ProductContract::second_right, transposed);
    bool entries = true;
    for (std::size_t row = 0U; row < size; ++row)
    {
        for (std::size_t column = 0U; column < size; ++column)
        {
            entries = same(transposed[row * size + column], g[column * size + row]) && entries;
        }
    }
    __ESBMC_assert(entries, "E-RESET sandwich: every entry of the actual right-hand transpose");
    __ESBMC_assert(same_matrix(transform, transform_before) && same_matrix(covariance, covariance_before),
                   "E-RESET sandwich: both input matrices are unchanged");
#else
    (void)transform;
    (void)covariance;
    (void)first;
    (void)second;
#endif
}

// Actual complete product wiring. AHRS executes all arithmetic directly. INS
// uses the separately proved scalar-entry helper at every coordinate, with
// independent arbitrary IEEE returns and no symmetry/finite-result assumption.
void verify_covariance_product(Matrix left, Matrix right, Matrix entries)
{
    constexpr bool configured =
        !FORMAL_ESKF_PROOF_PRODUCT_CONTRACT && (!FORMAL_ESKF_PROOF_INS || FORMAL_ESKF_PROOF_PRODUCT_ENTRY_CONTRACT);
    __ESBMC_assert(configured, "Runner error: actual covariance product requires the matching entry boundary");
    if constexpr (!configured)
    {
        return;
    }
    auto const left_before = left;
    auto const right_before = right;
#if FORMAL_ESKF_PROOF_INS && FORMAL_ESKF_PROOF_PRODUCT_ENTRY_CONTRACT
    using formal_eskf::verification::CovarianceEntryContract;
    CovarianceEntryContract::left = &formal_eskf::linalg::detail::MatrixAccess::storage(left).values;
    CovarianceEntryContract::right = &formal_eskf::linalg::detail::MatrixAccess::storage(right).values;
    CovarianceEntryContract::result = &formal_eskf::linalg::detail::MatrixAccess::storage(entries).values;
    CovarianceEntryContract::calls = 0U;
    auto const actual = left * right;
    __ESBMC_assert(CovarianceEntryContract::calls == 225U && same_matrix(actual, entries),
                   "E-RESET product: all 225 returned entries are stored at their requested coordinates");
#else
    (void)entries;
    Scalar a[size * size]{}, b[size * size]{};
    flatten(left_before, a);
    flatten(right_before, b);
    Matrix expected;
    for (std::size_t row = 0U; row < size; ++row)
    {
        for (std::size_t column = 0U; column < size; ++column)
        {
            Scalar sum{0};
            for (std::size_t k = 0U; k < size; ++k)
            {
                sum += a[row * size + k] * b[k * size + column];
            }
            expected.set(row, column, sum);
        }
    }
    auto const actual = left * right;
    for (std::size_t row = 0U; row < size; ++row)
    {
        for (std::size_t column = 0U; column < size; ++column)
        {
            __ESBMC_assert(same(actual(row, column), expected(row, column)),
                           "E-RESET product: every ordered matrix entry");
        }
    }
#endif
    __ESBMC_assert(same_matrix(left, left_before) && same_matrix(right, right_before),
                   "E-RESET product: both input matrices are unchanged");
}

// All valid symbolic coordinates, with no restriction on the IEEE operands.
// This proves the actual inner loop once rather than one proof per output cell.
void verify_covariance_entry(Matrix left, Matrix right, std::size_t row, std::size_t column, Scalar dot,
                             Linalg::vector_type<15U> row_values, Linalg::vector_type<15U> column_values)
{
    constexpr bool configured = FORMAL_ESKF_PROOF_INS && !FORMAL_ESKF_PROOF_PRODUCT_ENTRY_CONTRACT &&
                                FORMAL_ESKF_PROOF_DOT_CONTRACT && FORMAL_ESKF_PROOF_ACCESS_CONTRACT;
    __ESBMC_assert(configured, "Runner error: actual INS entry requires dot and access summaries");
    if constexpr (!configured)
    {
        return;
    }
#if FORMAL_ESKF_PROOF_INS && FORMAL_ESKF_PROOF_DOT_CONTRACT && FORMAL_ESKF_PROOF_ACCESS_CONTRACT
    __ESBMC_assume(row < 15U && column < 15U);
    auto const left_before = left;
    auto const right_before = right;
    using formal_eskf::verification::CovarianceAccessContract;
    using formal_eskf::verification::CovarianceDotContract;
    CovarianceAccessContract::left = &formal_eskf::linalg::detail::MatrixAccess::storage(left).values;
    CovarianceAccessContract::right = &formal_eskf::linalg::detail::MatrixAccess::storage(right).values;
    CovarianceAccessContract::row = row;
    CovarianceAccessContract::column = column;
    CovarianceAccessContract::row_values = row_values;
    CovarianceAccessContract::column_values = column_values;
    CovarianceAccessContract::calls = 0U;
    CovarianceDotContract::result = dot;
    CovarianceDotContract::calls = 0U;
    auto const actual = formal_eskf::verification::covariance_product_entry(
        formal_eskf::linalg::detail::MatrixAccess::storage(left).values,
        formal_eskf::linalg::detail::MatrixAccess::storage(right).values, row, column);
    __ESBMC_assert(CovarianceDotContract::calls == 1U && same(actual, dot),
                   "E-RESET entry: exactly one complete dot reduction and unchanged scalar return");
    __ESBMC_assert(CovarianceAccessContract::calls == 30U && same_vector(CovarianceDotContract::left, row_values) &&
                       same_vector(CovarianceDotContract::right, column_values),
                   "E-RESET entry: the complete requested row and column, in increasing inner-index order");
    __ESBMC_assert(same_matrix(left, left_before) && same_matrix(right, right_before),
                   "E-RESET entry: both complete input matrices are unchanged");
#else
    (void)left;
    (void)right;
    (void)row;
    (void)column;
    (void)dot;
    (void)row_values;
    (void)column_values;
#endif
}

void verify_covariance_dot(Linalg::vector_type<15U> left, Linalg::vector_type<15U> right)
{
    __ESBMC_assert(!FORMAL_ESKF_PROOF_DOT_CONTRACT, "Runner error: covariance dot producer must be actual");
    if constexpr (FORMAL_ESKF_PROOF_DOT_CONTRACT)
    {
        return;
    }
    auto const left_before = left;
    auto const right_before = right;
    Scalar expected{0};
    for (std::size_t k = 0U; k < 15U; ++k)
    {
        expected += left_before(k) * right_before(k);
    }
    auto const actual = formal_eskf::linalg::dot(left, right);
    __ESBMC_assert(same(actual, expected), "E-RESET dot: actual positive-zero initialized ordered IEEE reduction");
    __ESBMC_assert(same_vector(left, left_before) && same_vector(right, right_before),
                   "E-RESET dot: both complete input vectors are unchanged");
}

// Shared finalizer proof, once per size/scalar/alias, independent of reset mode.
// Covers all IEEE candidates and old outputs; no PSD/symmetry assumption.
void verify_finish_case(Matrix candidate, Matrix output, bool finite)
{
    constexpr bool configured = !FORMAL_ESKF_PROOF_RESET_CONTRACT && FORMAL_ESKF_PROOF_FINITE_CONTRACT;
    __ESBMC_assert(configured, "Runner error: actual covariance finalizer requires only the finite-predicate summary");
    if constexpr (!configured)
    {
        return;
    }
#if FORMAL_ESKF_PROOF_FINITE_CONTRACT
    (void)output;
    auto const candidate_before = candidate;
#if FORMAL_ESKF_PROOF_ALIAS
    auto & target = candidate;
#else
    auto & target = output;
#endif
    auto const before = target;
    FiniteContract::result = finite;
    FiniteContract::calls = 0U;
    auto const actual = formal_eskf::detail::try_finish_covariance(candidate, target);
    __ESBMC_assert(FiniteContract::calls == 1U && same_matrix(FiniteContract::received, candidate_before),
                   "E-RESET finalizer: the complete original candidate is checked before any mutation");
    __ESBMC_assert(actual == (finite ? Status::success : Status::non_finite_result),
                   "E-RESET finalizer: success exactly for a completely finite candidate");
    Scalar prior[size * size]{}, old_output[size * size]{}, current[size * size]{}, result[size * size]{};
    flatten(candidate_before, prior);
    flatten(before, old_output);
    flatten(candidate, current);
    flatten(target, result);
    bool coefficients = true;
    bool publication = true;
    bool symmetric = true;
    for (std::size_t row = 0U; row < size; ++row)
    {
        for (std::size_t column = 0U; column < size; ++column)
        {
            Scalar expected = prior[row * size + column];
            if (finite && row != column)
            {
                // The upper triangle is evaluated first, including zero signs.
                auto const low = row < column ? row : column;
                auto const high = row < column ? column : row;
                expected = Scalar{0.5} * prior[low * size + high] + Scalar{0.5} * prior[high * size + low];
            }
            coefficients = same(current[row * size + column], expected) && coefficients;
            publication =
                same(result[row * size + column], finite ? expected : old_output[row * size + column]) && publication;
            if (finite)
            {
                symmetric = same(result[row * size + column], result[column * size + row]) && symmetric;
            }
        }
    }
    __ESBMC_assert(coefficients, "E-RESET finalizer: unchanged diagonal, paired means, candidate rollback on failure");
    __ESBMC_assert(publication, "E-RESET finalizer: complete successful publication or output rollback");
    __ESBMC_assert(symmetric, "E-RESET finalizer: successful output is symmetric, not a PSD guarantee");
#else
    (void)candidate;
    (void)output;
    (void)finite;
#endif
}

void verify_reset_finish(Matrix candidate, Matrix output)
{
    // Both possible predicate returns are checked for the same arbitrary
    // inputs. Constant cases avoid coupling 225 copies to one symbolic flag;
    // neither result is assumed to be the actual predicate's return.
    verify_finish_case(candidate, output, false);
    verify_finish_case(candidate, output, true);
}

void verify_reset_finite(Matrix candidate)
{
    constexpr bool configured = !FORMAL_ESKF_PROOF_FINITE_CONTRACT && !FORMAL_ESKF_PROOF_RESET_CONTRACT;
    __ESBMC_assert(configured, "Runner error: covariance finite predicate must be actual");
    if constexpr (!configured)
    {
        return;
    }
    auto const before = candidate;
    __ESBMC_assert(formal_eskf::linalg::all_finite(candidate) == finite_matrix(before),
                   "E-RESET finite: true exactly when every covariance entry is finite");
    __ESBMC_assert(same_matrix(candidate, before), "E-RESET finite: every input coefficient is unchanged");
}

// Shape-independent IEEE lemma for the expression checked by the finalizer.
// No accuracy or PSD claim. Every finite pair, including subnormals and
// opposite signs, is covered; do not duplicate this per entry or reset mode.
void verify_covariance_mean(Scalar a, Scalar b)
{
    if (std::isfinite(a) && std::isfinite(b))
    {
        Scalar const mean = Scalar{0.5} * a + Scalar{0.5} * b;
        __ESBMC_assert(std::isfinite(mean), "E-RESET mean: halving finite operands before addition cannot overflow");
    }
}

int main() { return 0; }
