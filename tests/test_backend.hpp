/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <iostream>
#include <type_traits>

#if defined(FORMAL_ESKF_TEST_PX4_MATRIX)
#include <formal_eskf/linalg/backend/px4_matrix.hpp>
#else
#include <formal_eskf/linalg/backend/eigen.hpp>
#endif

namespace formal_eskf::test
{

#if defined(FORMAL_ESKF_TEST_PX4_MATRIX)
template <typename Scalar> using Backend = linalg::Px4MatrixBackend<Scalar>;
static_assert(std::is_same_v<Backend<float>::storage_type<2U, 3U>, ::matrix::Matrix<float, 2U, 3U>>);
static_assert(std::is_same_v<Backend<double>::storage_type<2U, 3U>, ::matrix::Matrix<double, 2U, 3U>>);
#else
template <typename Scalar> using Backend = linalg::EigenBackend<Scalar>;
#endif

inline void configure_backend_test()
{
#if defined(FORMAL_ESKF_TEST_PX4_MATRIX)
    std::cout << "Backend: PX4 matrix\n";
#else
    Eigen::internal::set_is_malloc_allowed(false);
    std::cout << "Backend: Eigen (runtime allocation guard enabled)\n";
#endif
}

} /* end namespace formal_eskf::test */
