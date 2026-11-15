/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bias_support.hpp"

static_assert(correction_proof::state_size == 15U && correction_proof::measurement_size <= 3U);
static_assert(!FORMAL_ESKF_PROOF_CORRECTION_CONTRACT && !FORMAL_ESKF_PROOF_ARITHMETIC_CONTRACT &&
              !FORMAL_ESKF_PROOF_FINITE_CONTRACT);

void verify_bias_gain(correction_proof::Gain gain, bias_proof::permissions_type permissions,
                      correction_proof::Residual residual)
{
    using correction_proof::Scalar;
    auto const original = gain;
    auto const old_permissions = permissions;
    formal_eskf::detail::constrain_bias_gain(permissions, gain);
    __ESBMC_assert(bias_proof::constrained_gain(gain, original, permissions),
                   "E-BIAS-UPDATE: zero exactly the disabled bias rows, preserving every other coefficient");
    __ESBMC_assert(bias_proof::same_permissions(permissions, old_permissions),
                   "E-BIAS-UPDATE: gain constraints never modify caller permissions");

    auto unrestricted = original;
    bias_proof::permissions_type const defaults;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        __ESBMC_assert(defaults.accelerometer[axis] && defaults.gyroscope[axis],
                       "E-BIAS-UPDATE: every default permission is enabled");
    }
    formal_eskf::detail::constrain_bias_gain(defaults, unrestricted);
    __ESBMC_assert(correction_proof::same_matrix(unrestricted, original),
                   "E-BIAS-UPDATE: all-enabled permissions preserve the unrestricted gain");

    // Public correction validates r before gain use. Even arbitrary other gain
    // rows cannot change a disabled row's zero correction on this finite domain.
    if (correction_proof::finite_matrix(residual))
    {
        auto const delta = gain * residual;
        for (std::size_t row = 9U; row < 15U; ++row)
        {
            __ESBMC_assert(bias_proof::permitted(permissions, row) || delta(row) == Scalar{0},
                           "E-BIAS-UPDATE: finite residuals produce zero correction on disabled bias axes");
        }
    }
}

int main() { return 0; }
