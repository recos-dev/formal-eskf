/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "covariance_oracle.hpp"
#include <formal_eskf/runtime/magnetometer.hpp>
#ifndef FORMAL_ESKF_PROOF_MAG_RESET_SIZE
#define FORMAL_ESKF_PROOF_MAG_RESET_SIZE 3
#endif
namespace magnetic_reset_proof
{
using contract_proof::Linalg;
using contract_proof::same;
using contract_proof::Scalar;
using covariance_oracle::finite_matrix;
using covariance_oracle::same_matrix;
using formal_eskf::Status;
constexpr std::size_t size = FORMAL_ESKF_PROOF_MAG_RESET_SIZE;
constexpr std::size_t offset = size == 3U ? 0U : 6U;
static_assert(size == 3U || size == 15U);
using Matrix = formal_eskf::linalg::Matrix<Linalg, size, size>;
using Matrix3 = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
using Cross = formal_eskf::linalg::Matrix<Linalg, 3U, size>;
using Vector = contract_proof::Vector3;
struct ResetTerms
{
    Scalar attitude[9]{};
    Scalar cross[3 * size]{};
};
// Independent row-major scalar oracle. Flat oracle arrays avoid repeatedly
// expanding named production covariance access; the actual body is unchanged.
inline ResetTerms covariance_terms(Scalar const * P, Matrix3 const & before, Matrix3 const & after, Scalar variance)
{
    Scalar const down[3]{Scalar{0}, Scalar{0}, Scalar{1}};
    Scalar axis[3]{}, tilt[9]{}, projected[9]{}, A[9]{};
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        for (std::size_t j = 0U; j < 3U; ++j)
        {
            tilt[3U * i + j] = (i == j ? Scalar{1} : Scalar{0}) - (Scalar{0} + down[i] * down[j]);
        }
        for (std::size_t k = 0U; k < 3U; ++k)
        {
            axis[i] += after(k, i) * down[k];
        }
    }
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        for (std::size_t j = 0U; j < 3U; ++j)
        {
            for (std::size_t k = 0U; k < 3U; ++k)
            {
                projected[3U * i + j] += after(k, i) * tilt[3U * k + j];
            }
        }
    }
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        for (std::size_t j = 0U; j < 3U; ++j)
        {
            for (std::size_t k = 0U; k < 3U; ++k)
            {
                A[3U * i + j] += projected[3U * i + k] * before(k, j);
            }
        }
    }
    ResetTerms result;
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        for (std::size_t j = 0U; j < size; ++j)
        {
            for (std::size_t k = 0U; k < 3U; ++k)
            {
                result.cross[size * i + j] += A[3U * i + k] * P[size * (offset + k) + j];
            }
        }
    }
    constexpr Scalar standard = static_cast<Scalar>(0.01), floor = standard * standard;
    Scalar const effective = variance < floor ? floor : variance;
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        for (std::size_t j = 0U; j < 3U; ++j)
        {
            Scalar retained{0};
            for (std::size_t k = 0U; k < 3U; ++k)
            {
                retained += result.cross[size * i + offset + k] * A[3U * j + k];
            }
            result.attitude[3U * i + j] = retained + (Scalar{0} + axis[i] * axis[j]) * effective;
        }
    }
    return result;
}
} // namespace magnetic_reset_proof
void verify_magnetic_reset_covariance(magnetic_reset_proof::Matrix covariance, magnetic_reset_proof::Matrix3 before,
                                      magnetic_reset_proof::Matrix3 after, magnetic_reset_proof::Scalar variance,
                                      magnetic_reset_proof::Matrix output)
{
    using namespace magnetic_reset_proof;
    static_assert(FORMAL_ESKF_PROOF_ALIAS < 2);
    Scalar prior[size * size], old_output[size * size];
    covariance_oracle::flatten(covariance, prior);
    covariance_oracle::flatten(output, old_output);
    auto const old_before = before, old_after = after;
    auto const terms = covariance_terms(prior, before, after, variance);
    bool finite = true;
    for (std::size_t i = 0U; i < 3U * size; ++i)
    {
        finite = std::isfinite(terms.cross[i]) && finite;
    }
    for (std::size_t i = 0U; i < 9U; ++i)
    {
        finite = std::isfinite(terms.attitude[i]) && finite;
    }
    Status wanted = !finite ? Status::non_finite_result : Status::success;
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        if (wanted == Status::success && terms.attitude[4U * i] < Scalar{0})
        {
            wanted = Status::not_positive_definite;
        }
    }
    Matrix & target = FORMAL_ESKF_PROOF_ALIAS ? covariance : output;
    Status const status =
        formal_eskf::runtime::detail::reset_heading_covariance<offset>(covariance, before, after, variance, target);
    __ESBMC_assert(status == wanted, "R-MAGNETOMETER-RESET: covariance finite/negative-variance guard priority");
    Scalar actual[size * size], retained_prior[size * size];
    covariance_oracle::flatten(target, actual);
    covariance_oracle::flatten(covariance, retained_prior);
    for (std::size_t i = 0U; i < size; ++i)
    {
        for (std::size_t j = 0U; j < size; ++j)
        {
            Scalar expected = prior[size * i + j];
            bool const a = i >= offset && i < offset + 3U, b = j >= offset && j < offset + 3U;
            if (a && b)
            {
                std::size_t const r = i - offset, c = j - offset;
                expected = r <= c ? terms.attitude[3U * r + c] / Scalar{2} + terms.attitude[3U * c + r] / Scalar{2}
                                  : terms.attitude[3U * c + r] / Scalar{2} + terms.attitude[3U * r + c] / Scalar{2};
            }
            else if (a)
            {
                expected = terms.cross[size * (i - offset) + j];
            }
            else if (b)
            {
                expected = terms.cross[size * (j - offset) + i];
            }
            if (status != Status::success)
            {
                expected = FORMAL_ESKF_PROOF_ALIAS ? prior[size * i + j] : old_output[size * i + j];
            }
            __ESBMC_assert(
                same(actual[size * i + j], expected),
                "R-MAGNETOMETER-RESET: every yaw/tilt/cross/untouched coefficient and complete failure frame");
            if constexpr (!FORMAL_ESKF_PROOF_ALIAS)
            {
                __ESBMC_assert(same(retained_prior[size * i + j], prior[size * i + j]),
                               "R-MAGNETOMETER-RESET: separate output preserves complete prior");
            }
        }
    }
    __ESBMC_assert(same_matrix(before, old_before) && same_matrix(after, old_after),
                   "R-MAGNETOMETER-RESET: both rotation inputs remain unchanged");
}
int main() { return 0; }
