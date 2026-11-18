/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

// Keep this choice identical in every translation unit of an application.
// INS generated arithmetic; AHRS and arbitrary-H correction retain dense products.
#ifndef ESKF_CODEGEN
#define ESKF_CODEGEN 1
#endif

static_assert(ESKF_CODEGEN == 0 || ESKF_CODEGEN == 1, "ESKF_CODEGEN must be 0 or 1");
