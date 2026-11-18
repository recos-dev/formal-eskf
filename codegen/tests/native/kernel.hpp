/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <formal_eskf/eskf/generated/ins_covariance.hpp>
#include <formal_eskf/eskf/eskf.hpp>

#ifdef FORMAL_ESKF_TEST_PX4_MATRIX
#include <formal_eskf/linalg/backend/px4_matrix.hpp>
#else
#include <formal_eskf/linalg/backend/eigen.hpp>
#endif

namespace codegen_test
{
#ifdef FORMAL_ESKF_TEST_PX4_MATRIX
template <typename S> using Backend = formal_eskf::linalg::Px4MatrixBackend<S>;
#else
template <typename S> using Backend = formal_eskf::linalg::EigenBackend<S>;
#endif
template <typename S> using Types = formal_eskf::EskfTypes<Backend<S>, formal_eskf::configuration::Ins>;
template <typename S, std::size_t R, std::size_t C> using Matrix = formal_eskf::linalg::Matrix<Backend<S>, R, C>;

template <typename S>
__attribute__((noinline)) auto covariance_kernel(Matrix<S, 15U, 15U> const & F, Matrix<S, 15U, 15U> const & P,
                                                 Matrix<S, 3U, 3U> const & R, Matrix<S, 12U, 12U> const & Q_i)
{
#if GENERATED
    return formal_eskf::detail::generated::ins_covariance(F, P, R, Q_i);
#else
    auto candidate = formal_eskf::linalg::sandwich(F, P);
    candidate.template set_block<3U, 3U>(candidate.template block<3U, 3U, 3U, 3U>() +
                                         formal_eskf::linalg::sandwich(R, Q_i.template block<0U, 0U, 3U, 3U>()));
    candidate.template set_block<6U, 6U>(candidate.template block<6U, 6U, 3U, 3U>() +
                                         Q_i.template block<3U, 3U, 3U, 3U>());
    candidate.template set_block<9U, 9U>(candidate.template block<9U, 9U, 3U, 3U>() +
                                         Q_i.template block<6U, 6U, 3U, 3U>());
    candidate.template set_block<12U, 12U>(candidate.template block<12U, 12U, 3U, 3U>() +
                                           Q_i.template block<9U, 9U, 3U, 3U>());
    for (unsigned i = 0U; i < 15U; ++i)
    {
        for (unsigned j = i + 1U; j < 15U; ++j)
        {
            S const mean = S{0.5} * candidate(i, j) + S{0.5} * candidate(j, i);
            candidate(i, j) = candidate(j, i) = mean;
        }
    }
    return candidate;
#endif
}

} // namespace codegen_test
