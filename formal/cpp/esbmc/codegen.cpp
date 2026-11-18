/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Instantiates the actual emitted header and actual production Matrix wrapper.
// The backend models fixed storage/access only; no production Eigen/PX4 internals
// or floating-point accuracy is claimed by this memory-safety profile.
#include "codegen_storage.hpp"
#include <formal_eskf/eskf/generated/ins_covariance.hpp>

void verify_codegen_memory()
{
    formal_eskf::linalg::Matrix<FixedArray, 15U, 15U> F, P;
    formal_eskf::linalg::Matrix<FixedArray, 3U, 3U> R;
    formal_eskf::linalg::Matrix<FixedArray, 12U, 12U> Q_i;
    symbolic(F);
    symbolic(P);
    symbolic(R);
    symbolic(Q_i);
    auto const result = formal_eskf::detail::generated::ins_covariance(F, P, R, Q_i);
    (void)result;
}
