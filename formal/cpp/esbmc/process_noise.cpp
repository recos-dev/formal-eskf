/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "support.hpp"

#include <formal_eskf/eskf/process_noise.hpp>

#ifndef FORMAL_ESKF_PROOF_BINARY64
#define FORMAL_ESKF_PROOF_BINARY64 0
#endif
#ifndef FORMAL_ESKF_PROOF_STORAGE_CONTRACT
#define FORMAL_ESKF_PROOF_STORAGE_CONTRACT 0
#endif

namespace noise_proof
{
#if FORMAL_ESKF_PROOF_BINARY64
using Scalar = double;
#else
using Scalar = float;
#endif
using Linalg = formal_eskf::verification::FixedArrayLinalg<Scalar>;
using Vector3 = Linalg::vector_type<3U>;
using AhrsNoise = formal_eskf::configuration::Ahrs::ProcessNoise<Linalg>;
using InsNoise = formal_eskf::configuration::Ins::ProcessNoise<Linalg>;
using AhrsCovariance = Linalg::matrix_type<3U, 3U>;
using InsCovariance = Linalg::matrix_type<12U, 12U>;
using formal_eskf::Status;

struct CommitContract
{
    using Cells = formal_eskf::verification::ScalarArray<Scalar, 144U>;
    inline static Cells * target = nullptr;
    inline static unsigned calls = 0U;
    inline static Cells received{};
};
} // namespace noise_proof

#if FORMAL_ESKF_PROOF_STORAGE_CONTRACT
namespace formal_eskf::verification
{
// Observe assignment to the designated output's backing storage; assignments
// to other 144-cell objects retain the original head/tail implementation.
// Capture with the actual smaller-array assignment, without recursing through
// this specialization. The REAL 12x12 assignment is proved in verify_ins_storage.
template <>
inline ScalarArray<noise_proof::Scalar, 144U> &
ScalarArray<noise_proof::Scalar, 144U>::operator=(ScalarArray const & source)
{
    using noise_proof::CommitContract;
    if (this == CommitContract::target)
    {
        __ESBMC_assert(CommitContract::calls == 0U, "E-NOISE storage contract: output commits at most once");
        ++CommitContract::calls;
        CommitContract::received.head = source.head;
        CommitContract::received.tail = source.tail;
    }
    else
    {
        head = source.head;
        tail = source.tail;
    }
    return *this;
}

// INS initialization realizes the proved positive-zero postcondition.
template <>
template <>
inline void FixedArrayLinalg<noise_proof::Scalar>::set_zero<12U, 12U>(storage_type<12U, 12U> & matrix) noexcept
{
    matrix = {};
}
} // namespace formal_eskf::verification
#endif

namespace noise_proof
{

// Value and signed-zero preservation; NaN classification, not payload bits.
bool same(Scalar a, Scalar b)
{
    return (a == b && (a != Scalar{0} || std::signbit(a) == std::signbit(b))) || (std::isnan(a) && std::isnan(b));
}

template <std::size_t Size>
bool same_cells(formal_eskf::verification::ScalarArray<Scalar, Size> const & a,
                formal_eskf::verification::ScalarArray<Scalar, Size> const & b)
{
    bool valid = same(a.head, b.head);
    if constexpr (Size > 1U)
    {
        bool const rest = same_cells(a.tail, b.tail);
        valid = valid && rest;
    }
    return valid;
}

Vector3 const & group(AhrsNoise const & noise, std::size_t index)
{
    __ESBMC_assert(index == 0U, "E-NOISE oracle: AHRS has only the gyro measurement group");
    return noise.angular_rate_variance;
}

Vector3 const & group(InsNoise const & noise, std::size_t index)
{
    switch (index)
    {
    case 0U:
        return noise.specific_force_variance;
    case 1U:
        return noise.angular_rate_variance;
    case 2U:
        return noise.accelerometer_bias_random_walk_variance_density;
    default:
        __ESBMC_assert(index == 3U, "E-NOISE oracle: INS has four source-coordinate groups");
        return noise.gyroscope_bias_random_walk_variance_density;
    }
}

// Independent status specification: dt first, then source groups in order;
// all three finite checks precede negative-value rejection within each group.
// Do not assume validity or call the production validator in the oracle.
template <std::size_t Groups, typename Noise> Status input_status(Noise const & noise, Scalar dt)
{
    if (!std::isfinite(dt))
    {
        return Status::non_finite_input;
    }
    if (!(dt > Scalar{0}))
    {
        return Status::out_of_range;
    }
    for (std::size_t block = 0U; block < Groups; ++block)
    {
        auto const & coefficients = group(noise, block);
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            if (!std::isfinite(coefficients(axis)))
            {
                return Status::non_finite_input;
            }
        }
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            if (coefficients(axis) < Scalar{0})
            {
                return Status::domain_error;
            }
        }
    }
    return Status::success;
}

// Observe the proof backend's row-major scalar fields once each. This avoids
// quadratic recursive indexing in the ORACLE, without replacing any access,
// finite check or diagonal write performed by the production operation. Every
// field is checked, including arbitrary old NaNs and signed zeros on failure.
template <std::size_t Dimension, std::size_t Remaining>
bool check_cells(formal_eskf::verification::ScalarArray<Scalar, Remaining> const & actual,
                 formal_eskf::verification::ScalarArray<Scalar, Remaining> const & before,
                 Scalar const (&diagonal)[Dimension], Status status)
{
    static_assert(Remaining <= Dimension * Dimension);
    constexpr std::size_t index = Dimension * Dimension - Remaining;
    constexpr std::size_t row = index / Dimension;
    constexpr std::size_t column = index % Dimension;
    bool valid;
    if (status == Status::success)
    {
        Scalar const expected = row == column ? diagonal[row] : Scalar{0};
        valid = same(actual.head, expected) && std::isfinite(actual.head) && actual.head >= Scalar{0};
    }
    else
    {
        valid = same(actual.head, before.head);
    }
    if constexpr (Remaining > 1U)
    {
        // One conjunction has the same obligations as separate per-cell
        // assertions, without asking the solver to re-encode the entire
        // noise calculation for each of the 144 cells.
        bool const rest = check_cells(actual.tail, before.tail, diagonal, status);
        valid = valid && rest;
    }
    return valid;
}

// Actual public function and finite checks. The optional INS initialization
// and commit summaries are closed by verify_ins_storage. No finite/success
// premise or scalar-math summary is used; AHRS remains entirely direct.
template <std::size_t Groups, typename Noise, typename Covariance>
void verify_process_noise(Noise const & noise, Scalar dt, Covariance & output)
{
    static_assert(Covariance::row_count == 3U * Groups && Covariance::column_count == 3U * Groups);
    auto const noise_before = noise;
    auto const before = output;
    Status expected_status = input_status<Groups>(noise_before, dt);
    Scalar diagonal[3U * Groups]{};
    if (expected_status == Status::success)
    {
        for (std::size_t index = 0U; index < 3U * Groups; ++index)
        {
            Scalar const variance = group(noise_before, index / 3U)(index % 3U);
            // Keep the IEEE evaluation order, especially for zero variance
            // and a dt whose square would overflow. Groups 0/1 are sampled
            // measurement variances; INS groups 2/3 are bias intensities.
            diagonal[index] = index < 6U ? (variance * dt) * dt : variance * dt;
            if (!std::isfinite(diagonal[index]))
            {
                expected_status = Status::non_finite_result;
            }
        }
    }
#if FORMAL_ESKF_PROOF_STORAGE_CONTRACT
    if constexpr (Groups == 4U)
    {
        CommitContract::target = &formal_eskf::linalg::detail::MatrixAccess::storage(output).values;
        CommitContract::calls = 0U;
    }
#endif
    auto const actual = formal_eskf::try_discretize_process_noise(noise, dt, output);
    __ESBMC_assert(actual == expected_status, "E-NOISE: exact input, ordered-group and arithmetic status");
#if FORMAL_ESKF_PROOF_STORAGE_CONTRACT
    if constexpr (Groups == 4U)
    {
        __ESBMC_assert(CommitContract::calls == (actual == Status::success ? 1U : 0U),
                       "E-NOISE: commit occurs exactly on success, never on rejection or overflow");
        __ESBMC_assert(same_cells(formal_eskf::linalg::detail::MatrixAccess::storage(output).values,
                                  formal_eskf::linalg::detail::MatrixAccess::storage(before).values),
                       "E-NOISE: there are no output writes outside the modeled commit");
        if (actual == Status::success)
        {
            __ESBMC_assert(check_cells(CommitContract::received, CommitContract::received, diagonal, Status::success),
                           "E-NOISE: every committed diagonal/zero entry is correct, finite and nonnegative");
        }
    }
    else
#endif
    {
        bool const valid_covariance =
            check_cells(formal_eskf::linalg::detail::MatrixAccess::storage(output).values,
                        formal_eskf::linalg::detail::MatrixAccess::storage(before).values, diagonal, actual);
        __ESBMC_assert(valid_covariance,
                       "E-NOISE: all diagonal/zero entries, finite nonnegative success, and complete failure rollback");
    }
    for (std::size_t block = 0U; block < Groups; ++block)
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            __ESBMC_assert(same(group(noise, block)(axis), group(noise_before, block)(axis)),
                           "E-NOISE: all input noise coefficients are unchanged on every return");
        }
    }
}
} // namespace noise_proof

// --assign-param-nondet supplies every IEEE input and every previous matrix
// coefficient; no constructor-validity or positive-noise assumption is made.
void verify_ahrs_process_noise(noise_proof::AhrsNoise noise, noise_proof::Scalar dt, noise_proof::AhrsCovariance output)
{
    noise_proof::verify_process_noise<1U>(noise, dt, output);
}

void verify_ins_process_noise(noise_proof::InsNoise noise, noise_proof::Scalar dt, noise_proof::InsCovariance output)
{
    noise_proof::verify_process_noise<4U>(noise, dt, output);
}

void verify_ins_storage(noise_proof::InsCovariance matrix, noise_proof::InsCovariance source, std::size_t row,
                        std::size_t column)
{
    __ESBMC_assert(!FORMAL_ESKF_PROOF_STORAGE_CONTRACT,
                   "Runner error: storage producer must execute actual assignment and set_zero");
    __ESBMC_assume(row < 12U && column < 12U);
    auto const before = source;
    auto & returned = (matrix = source);
    __ESBMC_assert(&returned == &matrix &&
                       noise_proof::same_cells(formal_eskf::linalg::detail::MatrixAccess::storage(matrix).values,
                                               formal_eskf::linalg::detail::MatrixAccess::storage(before).values),
                   "E-NOISE dependency: actual distinct-matrix assignment copies every cell and returns its target");
    auto & storage = formal_eskf::linalg::detail::MatrixAccess::storage(matrix);
    noise_proof::Linalg::set_zero(storage);
    __ESBMC_assert(noise_proof::same(matrix(row, column), noise_proof::Scalar{0}),
                   "E-NOISE dependency: actual 12x12 set_zero overwrites every old cell with positive zero");
    __ESBMC_assert(noise_proof::same_cells(formal_eskf::linalg::detail::MatrixAccess::storage(source).values,
                                           formal_eskf::linalg::detail::MatrixAccess::storage(before).values),
                   "E-NOISE dependency: copying and clearing a distinct target preserve every source cell");
}

int main() { return 0; }
