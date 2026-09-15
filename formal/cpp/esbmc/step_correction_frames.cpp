/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "step_correction_support.hpp"

// Pure scalar-library boundary, not a libm implementation or accuracy claim.
// Keep StandardMath and the checked scalar wrappers actual. Linking these
// models avoids expanding musl range reduction for an input-frame property.
// Every invocation returns a fresh arbitrary IEEE value, including NaN/Inf;
// there is no finite, successful or argument-range assumption.
extern double nondet_double();
extern "C" float sinf(float) noexcept { return nondet_float(); }
extern "C" float cosf(float) noexcept { return nondet_float(); }
extern "C" float sqrtf(float) noexcept { return nondet_float(); }
extern "C" double sin(double) noexcept { return nondet_double(); }
extern "C" double cos(double) noexcept { return nondet_double(); }
extern "C" double sqrt(double) noexcept { return nondet_double(); }

namespace step_correction_proof
{
struct ProductResult
{
    inline static Covariance value{};
    inline static unsigned calls{};
};
} // namespace step_correction_proof

#if FORMAL_ESKF_PROOF_STEP_FRAME == 2
namespace formal_eskf::linalg
{
// The same-type actual sandwich and every ordered product/input-frame producer
// are E-CORRECT dependencies. Only the numerical result is abstract here;
// reset validation, G construction, Jacobian and finalizer execute normally.
template <>
inline step_correction_proof::Covariance
sandwich<step_correction_proof::Backend, step_correction_proof::state_size, step_correction_proof::state_size>(
    step_correction_proof::Covariance const &, step_correction_proof::Covariance const &) noexcept
{
    __ESBMC_assert(step_correction_proof::ProductResult::calls++ == 0U,
                   "E-STEP-CORRECT frame: at most one covariance sandwich");
    return step_correction_proof::ProductResult::value;
}
} // namespace formal_eskf::linalg
#endif

using namespace step_correction_proof;

void verify_correction_injection_frame(State state, Error error, Scalar minimum, State output, Scalar root)
{
    constexpr bool configured =
        FORMAL_ESKF_PROOF_STEP_FRAME == 1 && !FORMAL_ESKF_PROOF_STEP_CONTRACT && boundaries_clear;
    __ESBMC_assert(configured,
                   "Runner error: correction injection frame requires actual component and quaternion code");
    if constexpr (!configured)
        return;
    auto const old_state = state;
    auto const old_error = error;
    solve_proof::Math::sqrt = {};
    solve_proof::Math::sqrt.results.head = root;
    // Only Exp's non-Taylor branch uses the positive-pivot root observer.
    // Quaternion norms use the actual standard-sqrt norm<4> implementation.
    (void)formal_eskf::try_inject_nominal(state, error, minimum, output);
    __ESBMC_assert(solve_proof::Math::sqrt.calls <= 1U && same_state(state, old_state) && same_error(error, old_error),
                   "E-STEP-CORRECT frame: actual injection preserves every nominal and error input coordinate");
}

void verify_correction_reset_frame(Error error, Covariance covariance, Covariance output, Covariance sandwich_result,
                                   Scalar root)
{
    constexpr bool configured =
        FORMAL_ESKF_PROOF_STEP_FRAME == 2 && !FORMAL_ESKF_PROOF_STEP_CONTRACT && boundaries_clear;
    __ESBMC_assert(configured, "Runner error: correction reset frame requires only the same-type sandwich summary");
    if constexpr (!configured)
        return;
    auto const old_error = error;
    auto const old_covariance = covariance;
    ProductResult::value = sandwich_result;
    ProductResult::calls = 0U;
    solve_proof::Math::sqrt = {};
    solve_proof::Math::sqrt.results.head = root;
    (void)formal_eskf::try_reset_covariance(error, covariance, output);
    __ESBMC_assert(solve_proof::Math::sqrt.calls <= 1U && same_error(error, old_error) &&
                       same_matrix(covariance, old_covariance),
                   "E-STEP-CORRECT frame: actual reset preserves every error and prior covariance coordinate");
}

int main() { return 0; }
