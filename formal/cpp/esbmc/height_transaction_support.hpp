// SPDX-License-Identifier: BSD-3-Clause

#pragma once
#include "height_state_support.hpp"
#include "bias_support.hpp"

namespace height_transaction_proof
{
using namespace height_proof;
using step_correction_proof::Backend;
using step_correction_proof::Covariance;
using step_correction_proof::same_state;
using step_correction_proof::State;
using Acceleration = Manager::Acceleration;
using Permissions = formal_eskf::configuration::Ins::BiasUpdate;

template <typename Cell> void check_cells(Cell const & a, Cell const & b)
{
    if constexpr (std::is_same_v<Cell, Scalar>)
    {
        // Equivalent IEEE value equality with signed zero retained and NaNs
        // compared by classification. Avoid an arithmetic equality query for
        // each coefficient of a copied candidate.
        using Bits = std::conditional_t<sizeof(Scalar) == 4U, std::uint32_t, std::uint64_t>;
        __ESBMC_assert((__builtin_bit_cast(Bits, a) == __builtin_bit_cast(Bits, b)) | (std::isnan(a) & std::isnan(b)),
                       "Height frame: every matrix coefficient");
    }
    else if constexpr (requires { a.rows; })
    {
        check_cells(a.rows, b.rows);
    }
    else if constexpr (requires { a.head; })
    {
        check_cells(a.head, b.head);
        if constexpr (requires { a.tail; })
        {
            check_cells(a.tail, b.tail);
        }
    }
    else
    {
#define HEIGHT_CELL(I) check_cells(a.c##I, b.c##I)
        HEIGHT_CELL(0);
        HEIGHT_CELL(1);
        HEIGHT_CELL(2);
        HEIGHT_CELL(3);
        HEIGHT_CELL(4);
        HEIGHT_CELL(5);
        HEIGHT_CELL(6);
        HEIGHT_CELL(7);
        HEIGHT_CELL(8);
        HEIGHT_CELL(9);
        HEIGHT_CELL(10);
        HEIGHT_CELL(11);
        HEIGHT_CELL(12);
        HEIGHT_CELL(13);
        HEIGHT_CELL(14);
#undef HEIGHT_CELL
    }
}
// Compare every coefficient in the actual fixed-size proof storage. No input
// or output-property partition is selected by the registered owner profiles.
template <typename Matrix> void check_matrix(Matrix const & a, Matrix const & b)
{
    check_cells(covariance_oracle::cells(a), covariance_oracle::cells(b));
}
inline void check_state(State const & a, State const & b)
{
    check_matrix(a.p_n, b.p_n);
    check_matrix(a.v_n, b.v_n);
    check_matrix(a.b_a, b.b_a);
    check_matrix(a.b_g, b.b_g);
    check_matrix(a.q_nb.coefficients(), b.q_nb.coefficients());
}

// Fieldwise postconditions preserve the full frame while allowing independent
// solver queries. These are assertions, never assumed successful summaries.
inline void check_bias(Bias const & a, Bias const & b)
{
#define HEIGHT_FIELD(F) __ESBMC_assert(same(a.F, b.F), "Height frame: bias " #F)
    HEIGHT_FIELD(value);
    HEIGHT_FIELD(variance);
    HEIGHT_FIELD(maximum_variance);
    HEIGHT_FIELD(signed_ratio);
    HEIGHT_FIELD(positive_time);
    HEIGHT_FIELD(negative_time);
    HEIGHT_FIELD(dt);
#undef HEIGHT_FIELD
}
inline void check_session(Session const & a, Session const & b)
{
    check_bias(a.bias, b.bias);
    __ESBMC_assert(same_ratio(a.ratio, b.ratio), "Height frame: reporting epoch and filtered ratio");
    __ESBMC_assert(a.fusion.status == b.fusion.status && a.fusion.decision == b.fusion.decision &&
                       a.fusion.diagnostics_valid == b.fusion.diagnostics_valid,
                   "Height frame: fusion status and diagnostic validity");
    __ESBMC_assert(same_matrix(a.fusion.innovation, b.fusion.innovation), "Height frame: innovation");
    __ESBMC_assert(same_matrix(a.fusion.innovation_variance, b.fusion.innovation_variance),
                   "Height frame: innovation variance");
    __ESBMC_assert(same_matrix(a.fusion.test_ratio, b.fusion.test_ratio), "Height frame: test ratio");
    __ESBMC_assert(same(a.fusion_innovation, b.fusion_innovation), "Height frame: fusion innovation");
    __ESBMC_assert(
        a.innovation_limited == b.innovation_limited && a.sample_time == b.sample_time && a.data_time == b.data_time &&
            a.progress_time == b.progress_time && a.fused_time == b.fused_time && a.device_id == b.device_id &&
            a.calibration_count == b.calibration_count && a.generation == b.generation && a.active == b.active &&
            a.fusion_reset == b.fusion_reset && a.fault == b.fault && a.stop_reason == b.stop_reason,
        "Height frame: complete session timestamps, identity, lifecycle and diagnostics");
}
inline void check_manager(Manager const & a, Manager const & b)
{
    check_session(a.m_sessions[0U], b.m_sessions[0U]);
    check_session(a.m_sessions[1U], b.m_sessions[1U]);
    __ESBMC_assert(same(a.m_origin_altitude, b.m_origin_altitude), "Height frame: configured origin");
    __ESBMC_assert(same(a.m_barometer_lpf, b.m_barometer_lpf), "Height frame: barometer filter");
    __ESBMC_assert(a.m_reference == b.m_reference && a.m_barometer_samples == b.m_barometer_samples &&
                       a.m_started == b.m_started && a.m_tick == b.m_tick &&
                       a.m_last_height_progress == b.m_last_height_progress,
                   "Height frame: reference, warmup counter and owner clocks");
}
inline void check_result(Result const & a, Result const & b)
{
    __ESBMC_assert(a.status == b.status && a.fused == b.fused && a.rejected == b.rejected &&
                       a.height_resets == b.height_resets && a.velocity_resets == b.velocity_resets,
                   "Height frame: status and complete event/reset counts");
    __ESBMC_assert(same(a.height_delta, b.height_delta), "Height frame: height reset delta");
    __ESBMC_assert(same(a.velocity_delta, b.velocity_delta), "Height frame: velocity reset delta");
}

} // namespace height_transaction_proof
