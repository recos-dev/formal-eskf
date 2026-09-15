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
using Transform = Backend::matrix_type<state_size, measurement_size>;
struct SandwichCalls
{
    inline static Transform const * transform = nullptr;
    inline static Noise const * covariance = nullptr;
    inline static Transform const * first = nullptr;
    inline static Covariance const * second = nullptr;
    inline static unsigned count{};
};
template <std::size_t Columns>
Backend::matrix_type<state_size, Columns>
sandwich_product(Transform const & left, Backend::matrix_type<measurement_size, Columns> const & right)
{
    unsigned const call = SandwichCalls::count++;
    if constexpr (Columns == measurement_size)
    {
        if (call == 0U)
        {
            __ESBMC_assert(same_matrix(left, *SandwichCalls::transform) &&
                               same_matrix(right, *SandwichCalls::covariance),
                           "E-CORRECT sandwich: first product uses the complete original transform and covariance");
            return *SandwichCalls::first;
        }
    }
    if constexpr (Columns == state_size)
    {
        if (call == 1U)
        {
            __ESBMC_assert(same_matrix(left, *SandwichCalls::first),
                           "E-CORRECT sandwich: second product uses the full first product");
            for (std::size_t i = 0U; i < state_size; ++i)
                for (std::size_t j = 0U; j < measurement_size; ++j)
                    __ESBMC_assert(same(right(j, i), (*SandwichCalls::transform)(i, j)),
                                   "E-CORRECT sandwich: transpose of the SAME complete original transform");
            return *SandwichCalls::second;
        }
    }
    __ESBMC_assert(false, "E-CORRECT sandwich: only the two expected products execute");
    return {};
}
} // namespace correction_proof

#if FORMAL_ESKF_PROOF_CORRECTION_CONTRACT
namespace formal_eskf::linalg
{
#define SANDWICH_PRODUCT(C)                                                                                            \
    template <>                                                                                                        \
    template <>                                                                                                        \
    inline Matrix<correction_proof::Backend, correction_proof::state_size, C>                                          \
    Matrix<correction_proof::Backend, correction_proof::state_size, correction_proof::measurement_size>::operator*     \
        <C>(Matrix<correction_proof::Backend, correction_proof::measurement_size, C> const & right) const noexcept     \
    {                                                                                                                  \
        return correction_proof::sandwich_product<C>(*this, right);                                                    \
    }
SANDWICH_PRODUCT(FORMAL_ESKF_PROOF_MEASUREMENT_SIZE)
#if FORMAL_ESKF_PROOF_STATE_SIZE != FORMAL_ESKF_PROOF_MEASUREMENT_SIZE
SANDWICH_PRODUCT(FORMAL_ESKF_PROOF_STATE_SIZE)
#endif
#undef SANDWICH_PRODUCT
} // namespace formal_eskf::linalg
#endif

using namespace correction_proof;
void verify_correction_sandwich(Transform transform, Noise covariance, Transform first, Covariance second)
{
    constexpr bool configured = FORMAL_ESKF_PROOF_CORRECTION_CONTRACT == 1 && !FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT &&
                                !FORMAL_ESKF_PROOF_PRODUCT_BOUNDARY && !FORMAL_ESKF_PROOF_FINITE_CONTRACT;
    __ESBMC_assert(configured, "Runner error: sandwich requires only product summaries");
    if constexpr (!configured)
        return;
    auto const old_transform = transform;
    auto const old_covariance = covariance;
    SandwichCalls::transform = &old_transform;
    SandwichCalls::covariance = &old_covariance;
    SandwichCalls::first = &first;
    SandwichCalls::second = &second;
    SandwichCalls::count = 0U;
    auto const result = formal_eskf::linalg::sandwich(transform, covariance);
    __ESBMC_assert(SandwichCalls::count == 2U && same_matrix(result, second),
                   "E-CORRECT sandwich: publish every coefficient of (T*P)*transpose(T)");
    __ESBMC_assert(same_matrix(transform, old_transform) && same_matrix(covariance, old_covariance),
                   "E-CORRECT sandwich: both complete original inputs survive");
}

int main() { return 0; }
