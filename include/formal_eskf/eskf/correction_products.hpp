/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <formal_eskf/linalg/linalg.hpp>
#include <formal_eskf/eskf/codegen.hpp>
#if ESKF_CODEGEN
#include <formal_eskf/eskf/generated/ins_innovation.hpp>
#include <formal_eskf/eskf/generated/ins_joseph.hpp>
#endif

namespace formal_eskf::detail
{

/** Internal arithmetic selection. Callers must supply the named model's exact H.
 * Generic public calls use arbitrary_h; known observation wrappers select explicitly.
 * Dimensions alone never identify a sensor model. No runtime dispatch or fallback.
 */
enum class CorrectionModel
{
    arbitrary_h,
    horizontal_position,
    vertical_position,
    horizontal_velocity,
    velocity,
    body_vector,
};

template <CorrectionModel Model, typename Linalg, std::size_t Size, std::size_t MeasurementSize>
linalg::Matrix<Linalg, Size, MeasurementSize> correction_cross(linalg::Matrix<Linalg, Size, Size> const & P,
                                                               linalg::Matrix<Linalg, MeasurementSize, Size> const & H)
{
#if ESKF_CODEGEN
    if constexpr (Model == CorrectionModel::horizontal_position)
    {
        static_assert(Size == 15U && MeasurementSize == 2U);
        return generated::ins_horizontal_position_cross(P);
    }
    else if constexpr (Model == CorrectionModel::vertical_position)
    {
        static_assert(Size == 15U && MeasurementSize == 1U);
        return generated::ins_vertical_position_cross(P);
    }
    else if constexpr (Model == CorrectionModel::horizontal_velocity)
    {
        static_assert(Size == 15U && MeasurementSize == 2U);
        return generated::ins_horizontal_velocity_cross(P);
    }
    else if constexpr (Model == CorrectionModel::velocity)
    {
        static_assert(Size == 15U && MeasurementSize == 3U);
        return generated::ins_velocity_cross(P);
    }
    else if constexpr (Model == CorrectionModel::body_vector)
    {
        static_assert(Size == 15U && MeasurementSize == 3U);
        auto const h = linalg::Vector<Linalg, 3U>::from_row_major({H(2U, 7U), H(0U, 8U), H(1U, 6U)});
        return generated::ins_body_vector_cross(P, h);
    }
    else
#endif
    {
        return P * linalg::transpose(H);
    }
}

template <CorrectionModel Model, typename Linalg, std::size_t Size, std::size_t MeasurementSize>
linalg::Matrix<Linalg, MeasurementSize, MeasurementSize>
correction_innovation(linalg::Matrix<Linalg, Size, MeasurementSize> const & C,
                      linalg::Matrix<Linalg, MeasurementSize, Size> const & H,
                      linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> const & V)
{
#if ESKF_CODEGEN
    if constexpr (Model == CorrectionModel::horizontal_position)
    {
        static_assert(Size == 15U && MeasurementSize == 2U);
        return generated::ins_horizontal_position_innovation(C, V);
    }
    else if constexpr (Model == CorrectionModel::vertical_position)
    {
        static_assert(Size == 15U && MeasurementSize == 1U);
        return generated::ins_vertical_position_innovation(C, V);
    }
    else if constexpr (Model == CorrectionModel::horizontal_velocity)
    {
        static_assert(Size == 15U && MeasurementSize == 2U);
        return generated::ins_horizontal_velocity_innovation(C, V);
    }
    else if constexpr (Model == CorrectionModel::velocity)
    {
        static_assert(Size == 15U && MeasurementSize == 3U);
        return generated::ins_velocity_innovation(C, V);
    }
    else if constexpr (Model == CorrectionModel::body_vector)
    {
        static_assert(Size == 15U && MeasurementSize == 3U);
        auto const h = linalg::Vector<Linalg, 3U>::from_row_major({H(2U, 7U), H(0U, 8U), H(1U, 6U)});
        return generated::ins_body_vector_innovation(C, h, V);
    }
    else
#endif
    {
        return H * C + V;
    }
}

template <CorrectionModel Model, typename Linalg, std::size_t Size, std::size_t MeasurementSize>
linalg::Matrix<Linalg, Size, Size> correction_joseph(linalg::Matrix<Linalg, Size, Size> const & P,
                                                     linalg::Matrix<Linalg, Size, Size> const & A,
                                                     linalg::Matrix<Linalg, Size, MeasurementSize> const & K,
                                                     linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> const & V)
{
#if ESKF_CODEGEN
    if constexpr (Model == CorrectionModel::horizontal_position)
    {
        static_assert(Size == 15U && MeasurementSize == 2U);
        return generated::ins_horizontal_position_joseph(P, A, K, V);
    }
    else if constexpr (Model == CorrectionModel::vertical_position)
    {
        static_assert(Size == 15U && MeasurementSize == 1U);
        return generated::ins_vertical_position_joseph(P, A, K, V);
    }
    else if constexpr (Model == CorrectionModel::horizontal_velocity)
    {
        static_assert(Size == 15U && MeasurementSize == 2U);
        return generated::ins_horizontal_velocity_joseph(P, A, K, V);
    }
    else if constexpr (Model == CorrectionModel::velocity)
    {
        static_assert(Size == 15U && MeasurementSize == 3U);
        return generated::ins_velocity_joseph(P, A, K, V);
    }
    else if constexpr (Model == CorrectionModel::body_vector)
    {
        static_assert(Size == 15U && MeasurementSize == 3U);
        return generated::ins_body_vector_joseph(P, A, K, V);
    }
    else
#endif
    {
        auto const prior_term = linalg::sandwich(A, P);
        auto const noise_term = linalg::sandwich(K, V);
        return prior_term + noise_term;
    }
}

} /* end namespace formal_eskf::detail */
