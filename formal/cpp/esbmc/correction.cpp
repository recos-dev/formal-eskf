/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "correction_support.hpp"

namespace correction_proof
{
// Pure matrix/solver/finalizer boundaries. Returns remain arbitrary IEEE data,
// even on failure; no success, PSD, finite-result or numerical premise is used.
// All corresponding actual producers are mandatory evidence for composition.
struct Results
{
    Gain cross_covariance, gain;
    Noise observation_covariance, innovation_sum, innovation;
    Correction delta;
    Covariance gain_times_H, transform, prior_term, noise_term, joseph_sum, covariance;
    Status input_status, innovation_status, solve_status, covariance_status;
    bool prior_symmetric, noise_symmetric;
    bool cross_finite, delta_finite, transform_finite;
};
struct Calls
{
    inline static Results const * results = nullptr;
    inline static Covariance const * prior = nullptr;
    inline static Jacobian const * H = nullptr;
    inline static Noise const * V = nullptr;
    inline static Residual const * r = nullptr;
    inline static Correction const * correction_output = nullptr;
    inline static Correction const * correction_before = nullptr;
    inline static Covariance const * covariance_output = nullptr;
    inline static Covariance const * covariance_before = nullptr;
    inline static Scalar minimum{};
    inline static bool observe = false;
    inline static std::size_t observed_row{}, observed_column{};
    inline static unsigned adds{}, subtracts{}, finites{}, validations{}, products{}, symmetries{}, finishes{},
        solves{}, sandwiches{};
};
// The observation coordinates occur only in assertions, never in production or
// eligibility. Each matrix's modulo projection is surjective onto all its
// coordinates. Universal indices therefore prove every coefficient together
// with status/frames, without duplicating 225 floating-point comparisons per call.
template <typename M> bool matches(M const & a, M const & b)
{
    static_assert(M::row_count <= state_size && M::column_count <= state_size);
    if (!Calls::observe)
        return same_matrix(a, b);
    auto const row = Calls::observed_row % M::row_count;
    auto const column = Calls::observed_column % M::column_count;
    Scalar left[M::row_count * M::column_count]{}, right[M::row_count * M::column_count]{};
    covariance_oracle::flatten(a, left);
    covariance_oracle::flatten(b, right);
    return same(left[row * M::column_count + column], right[row * M::column_count + column]);
}
void unpublished()
{
    if (Calls::correction_output == nullptr)
        return; // Validation-only entry has no output objects.
    __ESBMC_assert(matches(*Calls::correction_output, *Calls::correction_before) &&
                       matches(*Calls::covariance_output, *Calls::covariance_before),
                   "E-CORRECT: no output publication before all required operations succeed");
}
template <typename A, typename B> bool transpose_matches(A const & actual, B const & input)
{
    bool valid = true;
    for (std::size_t row = 0U; row < B::row_count; ++row)
        for (std::size_t column = 0U; column < B::column_count; ++column)
            valid = same(actual(column, row), input(row, column)) && valid;
    return valid;
}
template <std::size_t Size>
Backend::matrix_type<Size, Size> addition(Backend::matrix_type<Size, Size> const & left,
                                          Backend::matrix_type<Size, Size> const & right)
{
    auto const & data = *Calls::results;
    unsigned const call = Calls::adds++;
    if constexpr (Size == measurement_size)
    {
        if (call == 0U || state_size != measurement_size)
        {
            __ESBMC_assert(
                call == 0U && Calls::products == 2U && matches(left, data.observation_covariance) &&
                    matches(right, *Calls::V),
                "E-CORRECT: innovation adds the full original measurement covariance, including correlations");
            return data.innovation_sum;
        }
    }
    if constexpr (Size == state_size)
    {
        if (call == 1U || state_size != measurement_size)
        {
            __ESBMC_assert(call == 1U && Calls::sandwiches == 2U && matches(left, data.prior_term) &&
                               matches(right, data.noise_term),
                           "E-CORRECT: Joseph adds both complete prior and measurement terms");
            return data.joseph_sum;
        }
    }
    __ESBMC_assert(false, "E-CORRECT: only innovation and Joseph additions execute");
    return {};
}
// Do not name this finite: ESBMC recognizes that libc name as an intrinsic,
// even in a namespace, and would bypass the observer body.
template <std::size_t Rows, std::size_t Columns> bool check_finite(Backend::matrix_type<Rows, Columns> const & input)
{
    auto const & data = *Calls::results;
    unsigned const call = Calls::finites++;
    if constexpr (Rows == state_size && Columns == measurement_size)
    {
        if (call == 0U)
        {
            __ESBMC_assert(Calls::products == 1U && matches(input, data.cross_covariance),
                           "E-CORRECT: complete PHt finite check after first product");
            return data.cross_finite;
        }
    }
    if constexpr (Rows == state_size && Columns == 1U)
    {
        if (call == 1U)
        {
            __ESBMC_assert(Calls::products == 4U && matches(input, data.delta),
                           "E-CORRECT: complete error vector finite check");
            return data.delta_finite;
        }
    }
    if constexpr (Rows == state_size && Columns == state_size)
    {
        if (call == 2U)
        {
            __ESBMC_assert(Calls::products == 4U && data.delta_finite,
                           "E-CORRECT: transform check after finite error vector");
            __ESBMC_assert(Calls::subtracts == 1U && matches(input, data.transform),
                           "E-CORRECT: complete identity-minus-KH result is checked");
            return data.transform_finite;
        }
    }
    __ESBMC_assert(false, "E-CORRECT: only the three required finite checks execute");
    return false;
}
template <std::size_t Rows, std::size_t Inner, std::size_t Columns>
Backend::matrix_type<Rows, Columns> product(Backend::matrix_type<Rows, Inner> const & left,
                                            Backend::matrix_type<Inner, Columns> const & right)
{
    auto const & data = *Calls::results;
    unsigned const call = Calls::products++;
    if constexpr (Rows == state_size && Inner == state_size && Columns == measurement_size)
    {
        if (call == 0U || state_size != measurement_size)
        {
            __ESBMC_assert(call == 0U, "E-CORRECT: PHt is product zero");
            __ESBMC_assert(matches(left, *Calls::prior) && transpose_matches(right, *Calls::H),
                           "E-CORRECT: PHt uses complete original P and transposed original H");
            return data.cross_covariance;
        }
    }
    if constexpr (Rows == measurement_size && Inner == state_size && Columns == measurement_size)
    {
        if (call == 1U || state_size != measurement_size)
        {
            __ESBMC_assert(call == 1U, "E-CORRECT: innovation is product one");
            __ESBMC_assert(matches(left, *Calls::H) && matches(right, data.cross_covariance),
                           "E-CORRECT: innovation product uses H and the computed PHt");
            return data.observation_covariance;
        }
    }
    if constexpr (Rows == state_size && Inner == measurement_size && Columns == 1U)
    {
        if (call == 2U || state_size != measurement_size)
        {
            __ESBMC_assert(call == 2U, "E-CORRECT: error vector is product two");
            __ESBMC_assert(Calls::solves == 1U && data.solve_status == Status::success && matches(left, data.gain) &&
                               matches(right, *Calls::r),
                           "E-CORRECT: every correction coordinate comes from K times original r");
            return data.delta;
        }
    }
    if constexpr (Rows == state_size && Inner == measurement_size && Columns == state_size)
    {
        if (call == 3U || state_size != measurement_size)
        {
            __ESBMC_assert(call == 3U, "E-CORRECT: KH is product three");
            __ESBMC_assert(matches(left, data.gain) && matches(right, *Calls::H),
                           "E-CORRECT: correction transform uses computed K and original H");
            return data.gain_times_H;
        }
    }
    __ESBMC_assert(false, "E-CORRECT: only the four required ordered matrix products execute");
    return {};
}
template <std::size_t Size> bool symmetry(Backend::matrix_type<Size, Size> const & input, Scalar tolerance)
{
    __ESBMC_assert(same(tolerance, Scalar{0}) && Calls::products == 0U,
                   "E-CORRECT: exact input symmetry is validated before arithmetic");
    unsigned const call = Calls::symmetries++;
    if constexpr (Size == state_size)
    {
        if (call == 0U || state_size != measurement_size)
        {
            __ESBMC_assert(call == 0U, "E-CORRECT: prior symmetry is first");
            __ESBMC_assert(matches(input, *Calls::prior), "E-CORRECT: validate original P symmetry");
            return Calls::results->prior_symmetric;
        }
    }
    if constexpr (Size == measurement_size)
    {
        if (call == 1U || state_size != measurement_size)
        {
            __ESBMC_assert(call == 1U, "E-CORRECT: noise symmetry is second");
            __ESBMC_assert(Calls::results->prior_symmetric && matches(input, *Calls::V),
                           "E-CORRECT: validate original V symmetry after P");
            return Calls::results->noise_symmetric;
        }
    }
    __ESBMC_assert(false, "E-CORRECT: no additional symmetry checks or repaired caller inputs");
    return false;
}
template <std::size_t Size>
Status finish(Backend::matrix_type<Size, Size> & candidate, Backend::matrix_type<Size, Size> & output)
{
    unpublished();
    __ESBMC_assert(&candidate == &output && static_cast<void const *>(&output) != Calls::covariance_output,
                   "E-CORRECT: covariance finalizers receive internal in-place scratch");
    unsigned const call = Calls::finishes++;
    if constexpr (Size == measurement_size)
    {
        if (call == 0U || state_size != measurement_size)
        {
            __ESBMC_assert(call == 0U, "E-CORRECT: innovation finalization is first");
            __ESBMC_assert(Calls::products == 2U && Calls::adds == 1U &&
                               matches(candidate, Calls::results->innovation_sum),
                           "E-CORRECT: S retains all entries and correlations of V");
            output = Calls::results->innovation;
            return Calls::results->innovation_status;
        }
    }
    if constexpr (Size == state_size)
    {
        if (call == 1U || state_size != measurement_size)
        {
            __ESBMC_assert(call == 1U, "E-CORRECT: Joseph finalization is second");
            __ESBMC_assert(Calls::sandwiches == 2U && Calls::adds == 2U &&
                               matches(candidate, Calls::results->joseph_sum),
                           "E-CORRECT: Joseph covariance adds both complete terms in order");
            output = Calls::results->covariance;
            return Calls::results->covariance_status;
        }
    }
    __ESBMC_assert(false, "E-CORRECT: only innovation and Joseph scratch are finalized");
    return Status::domain_error;
}
template <std::size_t Inner>
Covariance sandwich(Backend::matrix_type<state_size, Inner> const & transform,
                    Backend::matrix_type<Inner, Inner> const & covariance)
{
    __ESBMC_assert(Calls::products == 4U, "E-CORRECT: Joseph follows delta and I-KH construction");
    unsigned const call = Calls::sandwiches++;
    if constexpr (Inner == state_size)
    {
        if (call == 0U || state_size != measurement_size)
        {
            __ESBMC_assert(call == 0U, "E-CORRECT: prior Joseph sandwich is first");
            __ESBMC_assert(matches(transform, Calls::results->transform) && matches(covariance, *Calls::prior),
                           "E-CORRECT: prior Joseph term uses I-KH and the SAME original P");
            return Calls::results->prior_term;
        }
    }
    if constexpr (Inner == measurement_size)
    {
        if (call == 1U || state_size != measurement_size)
        {
            __ESBMC_assert(call == 1U, "E-CORRECT: noise Joseph sandwich is second");
            __ESBMC_assert(matches(transform, Calls::results->gain) && matches(covariance, *Calls::V),
                           "E-CORRECT: noise Joseph term uses K and the complete correlated V");
            return Calls::results->noise_term;
        }
    }
    __ESBMC_assert(false, "E-CORRECT: exactly the two Joseph terms are computed");
    return {};
}
} // namespace correction_proof

#if FORMAL_ESKF_PROOF_CORRECTION_CONTRACT
namespace formal_eskf::linalg
{
template <>
inline correction_proof::Covariance
Matrix<correction_proof::Backend, correction_proof::state_size, correction_proof::state_size>::operator-(
    correction_proof::Covariance const & other) const noexcept
{
    using namespace correction_proof;
    __ESBMC_assert(Calls::subtracts++ == 0U && Calls::products == 4U && identity_matrix(*this) &&
                       matches(other, Calls::results->gain_times_H),
                   "E-CORRECT: I-KH uses the complete identity and computed K times original H");
    return Calls::results->transform;
}
#define CORRECTION_ADDITION(N)                                                                                         \
    template <>                                                                                                        \
    inline Matrix<correction_proof::Backend, N, N> Matrix<correction_proof::Backend, N, N>::operator+(                 \
        Matrix<correction_proof::Backend, N, N> const & other) const noexcept                                          \
    {                                                                                                                  \
        return correction_proof::addition<N>(*this, other);                                                            \
    }
CORRECTION_ADDITION(FORMAL_ESKF_PROOF_STATE_SIZE)
#if FORMAL_ESKF_PROOF_STATE_SIZE != FORMAL_ESKF_PROOF_MEASUREMENT_SIZE
CORRECTION_ADDITION(FORMAL_ESKF_PROOF_MEASUREMENT_SIZE)
#endif
#undef CORRECTION_ADDITION
// Actual finite producers cover each shape. Validation uses the exact predicate;
// arithmetic flow admits both results and checks every argument at the boundary.
#if FORMAL_ESKF_PROOF_CORRECTION_CONTRACT == 1
#define CORRECTION_FINITE(R, C)                                                                                        \
    template <>                                                                                                        \
    inline bool all_finite<correction_proof::Backend, R, C>(                                                           \
        Matrix<correction_proof::Backend, R, C> const & input) noexcept                                                \
    {                                                                                                                  \
        return correction_proof::check_finite<R, C>(input);                                                            \
    }
#else
#define CORRECTION_FINITE(R, C)                                                                                        \
    template <>                                                                                                        \
    inline bool all_finite<correction_proof::Backend, R, C>(                                                           \
        Matrix<correction_proof::Backend, R, C> const & input) noexcept                                                \
    {                                                                                                                  \
        return correction_proof::finite_matrix(input);                                                                 \
    }
#endif
CORRECTION_FINITE(1U, 1U)
CORRECTION_FINITE(2U, 1U)
CORRECTION_FINITE(3U, 1U)
CORRECTION_FINITE(15U, 1U)
CORRECTION_FINITE(1U, 3U)
CORRECTION_FINITE(2U, 3U)
CORRECTION_FINITE(3U, 2U)
CORRECTION_FINITE(3U, 3U)
CORRECTION_FINITE(1U, 15U)
CORRECTION_FINITE(2U, 15U)
CORRECTION_FINITE(3U, 15U)
CORRECTION_FINITE(15U, 2U)
CORRECTION_FINITE(15U, 3U)
CORRECTION_FINITE(2U, 2U)
CORRECTION_FINITE(15U, 15U)
#undef CORRECTION_FINITE
#define CORRECTION_PRODUCT(R, K, C)                                                                                    \
    template <>                                                                                                        \
    template <>                                                                                                        \
    inline Matrix<correction_proof::Backend, R, C> Matrix<correction_proof::Backend, R, K>::operator*                  \
        <C>(Matrix<correction_proof::Backend, K, C> const & right) const noexcept                                      \
    {                                                                                                                  \
        return correction_proof::product<R, K, C>(*this, right);                                                       \
    }
CORRECTION_PRODUCT(FORMAL_ESKF_PROOF_STATE_SIZE, FORMAL_ESKF_PROOF_STATE_SIZE, FORMAL_ESKF_PROOF_MEASUREMENT_SIZE)
#if FORMAL_ESKF_PROOF_STATE_SIZE != FORMAL_ESKF_PROOF_MEASUREMENT_SIZE
CORRECTION_PRODUCT(FORMAL_ESKF_PROOF_MEASUREMENT_SIZE, FORMAL_ESKF_PROOF_STATE_SIZE, FORMAL_ESKF_PROOF_MEASUREMENT_SIZE)
CORRECTION_PRODUCT(FORMAL_ESKF_PROOF_STATE_SIZE, FORMAL_ESKF_PROOF_MEASUREMENT_SIZE, FORMAL_ESKF_PROOF_STATE_SIZE)
#endif
CORRECTION_PRODUCT(FORMAL_ESKF_PROOF_STATE_SIZE, FORMAL_ESKF_PROOF_MEASUREMENT_SIZE, 1U)
#undef CORRECTION_PRODUCT

#define CORRECTION_SYMMETRY(N)                                                                                         \
    template <>                                                                                                        \
    inline bool is_symmetric<correction_proof::Backend, N>(Matrix<correction_proof::Backend, N, N> const & input,      \
                                                           correction_proof::Scalar tolerance) noexcept                \
    {                                                                                                                  \
        return correction_proof::symmetry<N>(input, tolerance);                                                        \
    }
CORRECTION_SYMMETRY(FORMAL_ESKF_PROOF_STATE_SIZE)
#if FORMAL_ESKF_PROOF_STATE_SIZE != FORMAL_ESKF_PROOF_MEASUREMENT_SIZE
CORRECTION_SYMMETRY(FORMAL_ESKF_PROOF_MEASUREMENT_SIZE)
#endif
#undef CORRECTION_SYMMETRY

template <>
inline Status
right_solve_spd<correction_proof::Backend, correction_proof::state_size, correction_proof::measurement_size>(
    correction_proof::Gain const & right, correction_proof::Noise const & system,
    correction_proof::Gain & output) noexcept
{
    using namespace correction_proof;
    __ESBMC_assert(Calls::solves++ == 0U && Calls::finishes == 1U &&
                       Calls::results->innovation_status == Status::success &&
                       matches(right, Calls::results->cross_covariance) && matches(system, Calls::results->innovation),
                   "E-CORRECT: K*S=PHt uses finalized innovation, once after successful finalization");
    __ESBMC_assert(&output != &right && static_cast<void const *>(&output) != &system,
                   "E-CORRECT: gain uses a separate solve workspace");
    output = Calls::results->gain;
    return Calls::results->solve_status;
}
#define CORRECTION_SANDWICH(K)                                                                                         \
    template <>                                                                                                        \
    inline correction_proof::Covariance sandwich<correction_proof::Backend, correction_proof::state_size, K>(          \
        Matrix<correction_proof::Backend, correction_proof::state_size, K> const & transform,                          \
        Matrix<correction_proof::Backend, K, K> const & covariance) noexcept                                           \
    {                                                                                                                  \
        return correction_proof::sandwich<K>(transform, covariance);                                                   \
    }
CORRECTION_SANDWICH(FORMAL_ESKF_PROOF_STATE_SIZE)
#if FORMAL_ESKF_PROOF_STATE_SIZE != FORMAL_ESKF_PROOF_MEASUREMENT_SIZE
CORRECTION_SANDWICH(FORMAL_ESKF_PROOF_MEASUREMENT_SIZE)
#endif
#undef CORRECTION_SANDWICH
} // namespace formal_eskf::linalg
namespace formal_eskf::detail
{
#if FORMAL_ESKF_PROOF_CORRECTION_CONTRACT == 1
template <>
inline Status
validate_correction_inputs<correction_proof::Backend, correction_proof::state_size, correction_proof::measurement_size>(
    correction_proof::Covariance const & P, correction_proof::Residual const & r, correction_proof::Jacobian const & H,
    correction_proof::Noise const & V, correction_proof::Scalar minimum) noexcept
{
    using namespace correction_proof;
    __ESBMC_assert(Calls::validations++ == 0U && Calls::products == 0U && matches(P, *Calls::prior) &&
                       matches(r, *Calls::r) && matches(H, *Calls::H) && matches(V, *Calls::V) &&
                       same(minimum, Calls::minimum),
                   "E-CORRECT: validate the complete original inputs before all matrix arithmetic");
    return Calls::results->input_status;
}
#endif
#define CORRECTION_FINISH(N)                                                                                           \
    template <>                                                                                                        \
    inline Status try_finish_covariance<correction_proof::Backend, N>(                                                 \
        linalg::Matrix<correction_proof::Backend, N, N> & candidate,                                                   \
        linalg::Matrix<correction_proof::Backend, N, N> & output) noexcept                                             \
    {                                                                                                                  \
        return correction_proof::finish<N>(candidate, output);                                                         \
    }
CORRECTION_FINISH(FORMAL_ESKF_PROOF_STATE_SIZE)
#if FORMAL_ESKF_PROOF_STATE_SIZE != FORMAL_ESKF_PROOF_MEASUREMENT_SIZE
CORRECTION_FINISH(FORMAL_ESKF_PROOF_MEASUREMENT_SIZE)
#endif
#undef CORRECTION_FINISH
} // namespace formal_eskf::detail
#endif

using namespace correction_proof;
using Configuration =
    std::conditional_t<state_size == 15U, formal_eskf::configuration::Ins, formal_eskf::configuration::Ahrs>;
using Error = Configuration::ErrorState<Backend>;

void verify_correction_validation(Covariance prior, Residual residual, Jacobian H, Noise V, Scalar minimum,
                                  Results results)
{
    constexpr bool configured = FORMAL_ESKF_PROOF_CORRECTION_CONTRACT == 2 && !FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT &&
                                !FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY && !FORMAL_ESKF_PROOF_FINITE_CONTRACT;
    __ESBMC_assert(configured, "Runner error: correction validation requires only finite and symmetry summaries");
    if constexpr (!configured)
        return;
    auto const old_prior = prior;
    auto const old_residual = residual;
    auto const old_H = H;
    auto const old_V = V;
    Calls::prior = &old_prior;
    Calls::H = &old_H;
    Calls::V = &old_V;
    Calls::r = &old_residual;
    Calls::results = &results;
    Calls::correction_output = nullptr;
    Calls::products = 0U;
    Calls::symmetries = 0U;
    bool const finite = finite_matrix(prior) && finite_matrix(residual) && finite_matrix(H) && finite_matrix(V) &&
                        std::isfinite(minimum);
    bool const norm = minimum > Scalar{0} && minimum <= Scalar{1};
    bool const valid = norm && results.prior_symmetric && results.noise_symmetric && nonnegative_diagonal(prior) &&
                       positive_diagonal(V);
    Status const expected = !finite ? Status::non_finite_input : !valid ? Status::domain_error : Status::success;
    auto const actual = formal_eskf::detail::validate_correction_inputs(prior, residual, H, V, minimum);
    __ESBMC_assert(actual == expected &&
                       Calls::symmetries == (finite && norm ? (results.prior_symmetric ? 2U : 1U) : 0U),
                   "E-CORRECT validation: finite inputs, norm range, exact symmetry, P nonnegative and V positive "
                   "diagonals, ordered status");
    __ESBMC_assert(matches(prior, old_prior) && matches(residual, old_residual) && matches(H, old_H) &&
                       matches(V, old_V),
                   "E-CORRECT validation: all original inputs are unchanged");
}

void verify_correction_unpack(Correction delta, Error error)
{
    constexpr bool configured = !FORMAL_ESKF_PROOF_CORRECTION_CONTRACT && !FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT &&
                                !FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY && !FORMAL_ESKF_PROOF_FINITE_CONTRACT;
    __ESBMC_assert(configured, "Runner error: correction unpack must be actual");
    if constexpr (!configured)
        return;
    auto const before = delta;
    formal_eskf::detail::unpack_correction(delta, error);
    Scalar expected[state_size]{};
    covariance_oracle::flatten(before, expected);
    for (std::size_t i = 0U; i < 3U; ++i)
    {
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
        __ESBMC_assert(same(error.delta_p_n(i), expected[i]) && same(error.delta_v_n(i), expected[3U + i]) &&
                           same(error.delta_theta_b(i), expected[6U + i]) &&
                           same(error.delta_b_a(i), expected[9U + i]) && same(error.delta_b_g(i), expected[12U + i]),
                       "E-CORRECT unpack: complete INS p,v,theta,ba,bg order");
#else
        __ESBMC_assert(same(error.delta_theta_b(i), expected[i]), "E-CORRECT unpack: all three AHRS error coordinates");
#endif
    }
    __ESBMC_assert(matches(delta, before), "E-CORRECT unpack: source correction vector is unchanged");
}

void verify_correction(Covariance prior, Residual residual, Jacobian H, Noise V, Scalar minimum,
                       Correction correction_output, Covariance covariance_output, Results results,
                       std::size_t observed_row, std::size_t observed_column)
{
    constexpr bool configured = FORMAL_ESKF_PROOF_CORRECTION_CONTRACT == 1 && !FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT &&
                                measurement_size <= 3U && !FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY &&
                                !FORMAL_ESKF_PROOF_FINITE_CONTRACT;
    __ESBMC_assert(configured,
                   "Runner error: correction requires only matrix, symmetry, solve and finalizer summaries");
    if constexpr (!configured)
        return;
    __ESBMC_assume(observed_row < state_size && observed_column < state_size);
    Calls::observe = true;
    Calls::observed_row = observed_row;
    Calls::observed_column = observed_column;
    auto const old_prior = prior;
    auto const old_residual = residual;
    auto const old_H = H;
    auto const old_V = V;
    auto const correction_before = correction_output;
    auto const covariance_before = covariance_output;
    Calls::prior = &old_prior;
    Calls::H = &old_H;
    Calls::V = &old_V;
    Calls::r = &old_residual;
    Calls::results = &results;
    Calls::correction_output = &correction_output;
    Calls::correction_before = &correction_before;
    Calls::covariance_output = &covariance_output;
    Calls::covariance_before = &covariance_before;
    Calls::products = 0U;
    Calls::symmetries = 0U;
    Calls::finishes = 0U;
    Calls::solves = 0U;
    Calls::sandwiches = 0U;
    Calls::minimum = minimum;
    Calls::validations = 0U;
    Calls::finites = 0U;
    Calls::adds = 0U;
    Calls::subtracts = 0U;
    bool const domain_valid = results.input_status == Status::success;
    bool const innovation_reached = domain_valid && results.cross_finite;
    bool const solve_reached = innovation_reached && results.innovation_status == Status::success;
    bool const correction_reached = solve_reached && results.solve_status == Status::success;
    bool const joseph_reached = correction_reached && results.delta_finite && results.transform_finite;
    Status const expected = !domain_valid         ? results.input_status
                            : !innovation_reached ? Status::non_finite_result
                            : !solve_reached      ? results.innovation_status
                            : !correction_reached ? results.solve_status
                            : !joseph_reached     ? Status::non_finite_result
                                                  : results.covariance_status;
    Status const actual = formal_eskf::detail::try_compute_correction(prior, residual, H, V, minimum, correction_output,
                                                                      covariance_output);
    __ESBMC_assert(actual == expected, "E-CORRECT: exact validation, arithmetic-failure and callee-status precedence");
    __ESBMC_assert(Calls::validations == 1U && Calls::symmetries == 0U &&
                       Calls::adds == (innovation_reached ? 1U : 0U) + (joseph_reached ? 1U : 0U) &&
                       Calls::subtracts == (correction_reached ? 1U : 0U) &&
                       Calls::finites == (domain_valid ? 1U : 0U) + (correction_reached ? 1U : 0U) +
                                             (correction_reached && results.delta_finite ? 1U : 0U) &&
                       Calls::products ==
                           (domain_valid ? 1U : 0U) + (innovation_reached ? 1U : 0U) + (correction_reached ? 2U : 0U) &&
                       Calls::finishes == (innovation_reached ? 1U : 0U) + (joseph_reached ? 1U : 0U) &&
                       Calls::solves == (solve_reached ? 1U : 0U) && Calls::sandwiches == (joseph_reached ? 2U : 0U),
                   "E-CORRECT: every required operation occurs exactly when eligible, with no work after failure");
    __ESBMC_assert(actual == Status::success
                       ? matches(correction_output, results.delta) && matches(covariance_output, results.covariance)
                       : matches(correction_output, correction_before) && matches(covariance_output, covariance_before),
                   "E-CORRECT: complete correction/Joseph publication or complete two-output rollback");
    __ESBMC_assert(matches(prior, old_prior) && matches(residual, old_residual) && matches(H, old_H) &&
                       matches(V, old_V),
                   "E-CORRECT: all original inputs remain unchanged");
}

int main() { return 0; }
