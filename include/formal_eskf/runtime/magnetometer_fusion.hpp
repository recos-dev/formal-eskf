/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <cstdint>
#include <limits>

#include <formal_eskf/runtime/measurement_fusion.hpp>

namespace formal_eskf::runtime
{

/**
 * Single-writer magnetic aiding lifecycle, independent of AHRS/INS and backend.
 * Call update on every estimator tick, even without a new sample. For an allowed
 * sample call fuse_magnetometer once, then record its actual fusion.decision
 * before the next tick. This controller owns no measurements, estimator state,
 * covariance, clock, yaw alignment or reset implementation. Startup qualification
 * and active-session fusion use different conditions; only actual fusion starts
 * a session. Reset requests must be handled by the caller, never fabricated here.
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
        innovation_rejected,
        data_timeout,
        fusion_timeout,
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
        // Startup requires age < interval; no-data stop uses age >= 2 * interval.
        std::uint64_t maximum_interval_us{};
        std::uint64_t fusion_timeout_us{};
    };

    struct Conditions
    {
        bool enabled = false;
        // Caller establishes alignment, valid reference, state/P and aiding policy.
        bool ready = false;
        // No yaw-alignment change or external quaternion reset in this tick.
        bool starting_allowed = true;
        bool horizontal_aiding = false;
        bool at_rest = false;
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
        bool active = false; // Started by successful fusion, not proof of estimate validity.
        bool fusion_allowed = false;
        StopReason stop_reason = StopReason::disabled;
        RecoveryReason recovery_reason = RecoveryReason::none;
        // Fusion time of the last failed check, seeded by the first check.
        std::uint64_t last_field_failure_time_us{};
        std::uint64_t start_time_us{};     // Successful start of the latest session.
        std::uint64_t last_fuse_time_us{}; // Only actual fused feedback updates this.
    };

    [[nodiscard]] State const & state() const noexcept { return m_state; }
    void reset() noexcept { *this = MagnetometerFusion{}; }

    /**
     * Positive durations remain fixed until reset. now_us and new sample times
     * strictly increase on one fusion timeline; samples are no later than now.
     * Health qualifies at elapsed > health_time and after at least four samples.
     * No-data stops preserve field history; a new qualified sample can restart.
     * After an active fusion attempt, unsuccessful-fusion elapsed > timeout stops
     * the session or requests a reset according to aiding/rest conditions.
     * Invalid calls stop activity/permission without consuming time or source.
     */
    [[nodiscard]] Status update(std::uint64_t now_us, Conditions conditions, Parameters const & parameters,
                                Sample const & sample) noexcept;

    /** One feedback per granted sample/tick; invalid feedback preserves the controller. */
    [[nodiscard]] Status record_fusion(std::uint64_t sample_time_us, FusionDecision decision) noexcept;

    /**
     * Explicit authorization after caller establishes safe re-entry, including
     * alignment/state/P repair if needed. Clears session/health/sample-count
     * history, NOT the estimator or time/source watermarks. This is not needed
     * for ordinary data or fusion-timeout stops, which can restart automatically.
     * Also call after reference/frame/field-policy changes or an external yaw reset.
     * Never restart repeatedly merely to bypass a fusion timeout.
     */
    void restart() noexcept
    {
        m_state = State{};
        m_sample_count = 0U;
    }

private:
    void stop(StopReason reason) noexcept;
    void require_recovery(RecoveryReason reason) noexcept;
    void check_fusion_timeout() noexcept;

    State m_state{};
    Parameters m_parameters{};
    Source m_source{};
    std::uint64_t m_update_time_us{};
    std::uint64_t m_sample_time_us{};
    unsigned m_sample_count{}; // Saturates at the four-sample startup requirement.
    bool m_reset_on_timeout = false;
}; /* end class MagnetometerFusion */

inline void MagnetometerFusion::stop(StopReason reason) noexcept
{
    m_state.active = false;
    m_state.fusion_allowed = false;
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

inline void MagnetometerFusion::check_fusion_timeout() noexcept
{
    if (m_state.active && m_update_time_us - m_state.last_fuse_time_us > m_parameters.fusion_timeout_us)
    {
        if (m_reset_on_timeout)
        {
            require_recovery(RecoveryReason::fusion_timeout);
        }
        else
        {
            stop(StopReason::fusion_timeout);
        }
    }
}

inline Status MagnetometerFusion::update(std::uint64_t now_us, Conditions conditions, Parameters const & parameters,
                                         Sample const & sample) noexcept
{
    m_state.fusion_allowed = false;
    if (parameters.health_time_us == 0U || parameters.maximum_interval_us == 0U ||
        parameters.maximum_interval_us > std::numeric_limits<std::uint64_t>::max() / 2U ||
        parameters.fusion_timeout_us == 0U ||
        (m_update_time_us != 0U && (parameters.health_time_us != m_parameters.health_time_us ||
                                    parameters.maximum_interval_us != m_parameters.maximum_interval_us ||
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
    m_reset_on_timeout = !conditions.horizontal_aiding || conditions.at_rest;
    bool const new_sample = sample.time_us != 0U;
    if (new_sample)
    {
        bool const source_changed =
            m_source.device_id != 0U && (sample.source.device_id != m_source.device_id ||
                                         sample.source.calibration_count != m_source.calibration_count);
        if (source_changed)
        {
            m_state = State{};
            m_sample_count = 0U;
            require_recovery(RecoveryReason::source_changed);
        }
        m_source = sample.source;
        m_sample_time_us = sample.time_us;
    }

    if (!conditions.enabled)
    {
        stop(StopReason::disabled);
        return Status::success;
    }
    if (new_sample && m_sample_count < 4U)
    {
        ++m_sample_count;
    }
    if (!new_sample)
    {
        // No-data ticks stop on recency, not on the active fusion-attempt timer.
        if (m_state.recovery_reason != RecoveryReason::none)
        {
            stop(StopReason::recovery_required);
        }
        else if (m_sample_time_us == 0U || now_us - m_sample_time_us >= 2U * parameters.maximum_interval_us)
        {
            stop(StopReason::data_timeout);
        }
        return Status::success;
    }
    if (!conditions.ready)
    {
        stop(StopReason::not_ready);
    }
    else if (m_state.recovery_reason != RecoveryReason::none)
    {
        stop(StopReason::recovery_required);
    }
    else
    {
        if (!sample.field_accepted || m_state.last_field_failure_time_us == 0U)
        {
            m_state.last_field_failure_time_us = now_us;
        }
        if (m_state.active)
        {
            // A field disturbance inhibits navigation-state correction without
            // ending the session. The next good field need not requalify startup.
            m_state.fusion_allowed = sample.field_accepted;
            m_state.stop_reason = sample.field_accepted ? StopReason::none : StopReason::field_rejected;
            if (!sample.field_accepted)
            {
                check_fusion_timeout();
            }
        }
        else if (!conditions.starting_allowed)
        {
            stop(StopReason::not_ready);
        }
        else if (now_us - m_sample_time_us >= parameters.maximum_interval_us)
        {
            stop(StopReason::data_timeout);
        }
        else if (!sample.field_accepted)
        {
            stop(StopReason::field_rejected);
        }
        else if (m_sample_count < 4U || now_us - m_state.last_field_failure_time_us <= parameters.health_time_us)
        {
            stop(StopReason::waiting_for_health);
        }
        else
        {
            // A rejected start is still inactive and does not invent progress.
            m_state.fusion_allowed = true;
            m_state.stop_reason = StopReason::none;
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
        if (!m_state.active)
        {
            m_state.start_time_us = m_update_time_us;
        }
        m_state.active = true;
        m_state.last_fuse_time_us = m_update_time_us;
        m_state.stop_reason = StopReason::none;
    }
    else if (decision == FusionDecision::failed)
    {
        require_recovery(RecoveryReason::fusion_failed);
    }
    else
    {
        m_state.stop_reason = StopReason::innovation_rejected;
        // Try the current observation first: success at an overdue horizon
        // refreshes progress instead of being discarded before correction.
        check_fusion_timeout();
    }
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
