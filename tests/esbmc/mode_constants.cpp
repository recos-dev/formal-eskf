/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Intentional negative fixture, not an ESKF proof. ESBMC 8.4.0 prints both
// constants as 1.000000e+0f in GOTO/symbol text, but the solver distinguishes them.
#if ESKF_QUAT_APPROX
#define BOUNDARY 1.00000011920928955078125f
#else
#define BOUNDARY 1.0000002384185791015625f
#endif

void verify_mode_constants(float x)
{
    __ESBMC_assume(x == 1.00000011920928955078125f);
    __ESBMC_assert(x != BOUNDARY, "different floating-point constant");
}
