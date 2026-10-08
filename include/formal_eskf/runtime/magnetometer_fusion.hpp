/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <cstdint>

#include <formal_eskf/runtime/measurement_fusion.hpp>

namespace formal_eskf::runtime
{

/**
 * Single-writer magnetic aiding lifecycle, independent of AHRS/INS and backend.
 * Call update on every estimator tick, even without a new sample. For an allowed
 * sample call fuse_magnetometer once, then record its actual fusion.decision
 * before the next tick. This controller owns no measurements, estimator state,
 * covariance, clock, yaw alignment or reset implementation.
 */
class MagnetometerFusion
{
public:
    enum class StopReason
    {
        none,
        disabled,
        not_ready,
        waiting_for_health,
        field_rejected,
        data_timeout,
        recovery_required,
        invalid_call,
    };

    enum class RecoveryReason
    {
        none,
        fusion_timeout,
        fusion_failed,
        source_changed,
    };

    struct Parameters
    {
        std::uint64_t health_time_us{};
        // Bounds both sample age and gaps between samples in a healthy interval.
        std::uint64_t maximum_sample_age_us{};
        std::uint64_t fusion_timeout_us{};
    };

    struct Conditions
    {
        bool enabled = false;
        // Caller establishes alignment, valid reference, state/P and aiding policy.
        bool ready = false;
    };

    struct Source
    {
        std::uint32_t device_id{}; // Nonzero, stable identity assigned by the adapter.
        std::uint32_t calibration_count{};
    };

    struct Sample
    {
        std::uint64_t time_us{}; // Zero means no new sample; other fields are ignored.
        Source source{};
        // Actual successful, accepted check_magnetic_field result at this horizon.
        bool field_accepted = false;
    };

    struct State
    {
        bool active = false; // Qualified monitoring session, not proof of estimate validity.
        bool fusion_allowed = false;
        StopReason stop_reason = StopReason::disabled;
        RecoveryReason recovery_reason = RecoveryReason::none;
        std::uint64_t health_start_time_us{}; // First consecutive good sample, not last failure.
        std::uint64_t start_time_us{};        // First authorization in this session.
        std::uint64_t last_fuse_time_us{};    // Only actual fused feedback updates this.
    };

    [[nodiscard]] State const & state() const noexcept { return m_state; }
    void reset() noexcept { *this = MagnetometerFusion{}; }

    /**
     * Positive durations remain fixed until reset. now_us and new sample times
     * strictly increase on one fusion timeline; samples are no later than now.
     * Health qualifies at elapsed >= health_time, deadlines expire only at >.
     * Invalid calls stop qualification/permission without consuming time/source
     * or erasing a session's progress deadline and recovery reason.
     */
    [[nodiscard]] Status update(std::uint64_t now_us, Conditions conditions, Parameters const & parameters,
                                Sample const & sample) noexcept;

    /** One feedback per granted sample/tick; invalid feedback preserves the controller. */
    [[nodiscard]] Status record_fusion(std::uint64_t sample_time_us, FusionDecision decision) noexcept;

    /**
     * Explicit authorization after caller establishes safe re-entry, including
     * alignment/state/P repair if needed. Clears session/health history, NOT the
     * estimator or time/source watermarks; requires new samples and requalification.
     * Also call after reference/frame/field-policy changes or an external yaw reset.
     * Never restart repeatedly merely to bypass a fusion timeout.
     */
    void restart() noexcept { m_state = State{}; }

private:
    void stop(StopReason reason) noexcept;
    void require_recovery(RecoveryReason reason) noexcept;

    State m_state{};
    Parameters m_parameters{};
    Source m_source{};
    std::uint64_t m_update_time_us{};
    std::uint64_t m_sample_time_us{};
}; /* end class MagnetometerFusion */

inline void MagnetometerFusion::stop(StopReason reason) noexcept
{
    m_state.active = false;
    m_state.fusion_allowed = false;
    m_state.health_start_time_us = 0U;
    m_state.stop_reason = reason;
}

inline void MagnetometerFusion::require_recovery(RecoveryReason reason) noexcept
{
    if (m_state.recovery_reason == RecoveryReason::none)
    {
        m_state.recovery_reason = reason;
    }
    stop(StopReason::recovery_required);
}

inline Status MagnetometerFusion::update(std::uint64_t now_us, Conditions conditions, Parameters const & parameters,
                                         Sample const & sample) noexcept
{
    m_state.fusion_allowed = false;
    if (parameters.health_time_us == 0U || parameters.maximum_sample_age_us == 0U ||
        parameters.fusion_timeout_us == 0U ||
        (m_update_time_us != 0U && (parameters.health_time_us != m_parameters.health_time_us ||
                                    parameters.maximum_sample_age_us != m_parameters.maximum_sample_age_us ||
                                    parameters.fusion_timeout_us != m_parameters.fusion_timeout_us)))
    {
        stop(StopReason::invalid_call);
        return Status::domain_error;
    }
    if (now_us <= m_update_time_us ||
        (sample.time_us != 0U && (sample.time_us <= m_sample_time_us || sample.time_us > now_us)))
    {
        stop(StopReason::invalid_call);
        return Status::out_of_range;
    }
    if (sample.time_us != 0U && sample.source.device_id == 0U)
    {
        stop(StopReason::invalid_call);
        return Status::domain_error;
    }

    m_update_time_us = now_us;
    m_parameters = parameters;
    bool const new_sample = sample.time_us != 0U;
    if (new_sample)
    {
        bool const source_changed =
            m_source.device_id != 0U && (sample.source.device_id != m_source.device_id ||
                                         sample.source.calibration_count != m_source.calibration_count);
        if (source_changed)
        {
            m_state = State{};
            require_recovery(RecoveryReason::source_changed);
        }
        else if (m_sample_time_us != 0U && sample.time_us - m_sample_time_us > parameters.maximum_sample_age_us)
        {
            stop(StopReason::waiting_for_health);
        }
        m_source = sample.source;
        m_sample_time_us = sample.time_us;
    }

    // Keep this deadline across short pauses/requalification. Toggling enable
    // or repeatedly providing good fields must not conceal persistent gate rejection.
    auto const last_progress = m_state.last_fuse_time_us != 0U ? m_state.last_fuse_time_us : m_state.start_time_us;
    if (last_progress != 0U && now_us - last_progress > parameters.fusion_timeout_us)
    {
        require_recovery(RecoveryReason::fusion_timeout);
    }
    if (!conditions.enabled)
    {
        stop(StopReason::disabled);
    }
    else if (!conditions.ready)
    {
        stop(StopReason::not_ready);
    }
    else if (m_state.recovery_reason != RecoveryReason::none)
    {
        stop(StopReason::recovery_required);
    }
    else if (m_sample_time_us == 0U || now_us - m_sample_time_us > parameters.maximum_sample_age_us)
    {
        stop(StopReason::data_timeout);
    }
    else if (new_sample && !sample.field_accepted)
    {
        stop(StopReason::field_rejected);
    }
    else
    {
        if (new_sample && m_state.health_start_time_us == 0U)
        {
            m_state.health_start_time_us = sample.time_us;
        }
        m_state.active = m_state.health_start_time_us != 0U &&
                         m_sample_time_us - m_state.health_start_time_us >= parameters.health_time_us;
        m_state.stop_reason = m_state.active ? StopReason::none : StopReason::waiting_for_health;
        m_state.fusion_allowed = m_state.active && new_sample;
        if (m_state.fusion_allowed && m_state.start_time_us == 0U)
        {
            m_state.start_time_us = now_us;
        }
    }
    return Status::success;
}

inline Status MagnetometerFusion::record_fusion(std::uint64_t sample_time_us, FusionDecision decision) noexcept
{
    if (decision != FusionDecision::fused && decision != FusionDecision::rejected && decision != FusionDecision::failed)
    {
        return Status::domain_error;
    }
    if (!m_state.fusion_allowed || sample_time_us != m_sample_time_us)
    {
        return Status::out_of_range;
    }
    m_state.fusion_allowed = false;
    if (decision == FusionDecision::fused)
    {
        m_state.last_fuse_time_us = m_update_time_us;
    }
    else if (decision == FusionDecision::failed)
    {
        require_recovery(RecoveryReason::fusion_failed);
    }
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
