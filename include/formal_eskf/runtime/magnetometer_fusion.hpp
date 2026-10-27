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
 * covariance, clock or reset implementation. Recovery permits a separate heading
 * reset, acknowledged only after the caller runs it.
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
        heading_inconsistent,
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
        reset_failed,
        source_changed,
        reference_changed,
    };

    enum class FlightState
    {
        unknown,
        on_ground,
        in_air
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
        FlightState flight_state = FlightState::unknown;
        // Current MagneticHeading verdict; required only with horizontal aiding.
        bool heading_consistent = false;
    };

    struct Source
    {
        std::uint32_t device_id{}; // Nonzero, stable identity assigned by the adapter.
        std::uint32_t calibration_count{};
        std::uint32_t reference_id{}; // Revision of the caller's NED field; not a frame change.
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
        bool active = false; // Fusion started, or established by an acknowledged heading reset.
        bool fusion_allowed = false;
        bool reset_allowed = false;
        bool filter_reset = false; // Seed the measurement LPF on this enabled sample.
        StopReason stop_reason = StopReason::disabled;
        RecoveryReason recovery_reason = RecoveryReason::none;
        // Fusion time of the last failed check, seeded by the first check.
        std::uint64_t last_field_failure_time_us{};
        std::uint64_t start_time_us{};      // Successful start of the latest session.
        std::uint64_t last_fuse_time_us{};  // Only actual fused feedback updates this.
        std::uint64_t last_reset_time_us{}; // Only actual successful reset feedback updates this.
    };

    [[nodiscard]] State const & state() const noexcept { return m_state; }
    /** Reinitialize the timeline; optionally bind the source used by caller's initial heading. */
    void reset(Source const & source = {0U, 0U, 0U}) noexcept
    {
        *this = MagnetometerFusion{};
        m_source = source;
    }

    /**
     * Positive durations remain fixed until reset. now_us and new sample times
     * strictly increase on one fusion timeline; samples are no later than now.
     * Health qualifies at elapsed > health_time and after at least four samples.
     * No-data stops preserve field history; a new qualified sample can restart.
     * After an active fusion attempt, unsuccessful-fusion elapsed > timeout stops
     * the session or requests a reset according to aiding/rest conditions.
     * Invalid calls stop activity/permission without consuming time or source.
     */
    [[nodiscard]] Status update(std::uint64_t now_us, Conditions const & conditions, Parameters const & parameters,
                                Sample const & sample) noexcept;

    /** One feedback per granted sample/tick; invalid feedback preserves the controller. */
    [[nodiscard]] Status record_fusion(std::uint64_t sample_time_us, FusionDecision decision) noexcept;

    /** Consume a reset permission with the actual numerical result; no fabricated fusion. */
    [[nodiscard]] Status record_reset(std::uint64_t sample_time_us, bool reset_succeeded) noexcept;

    /**
     * Explicit authorization after caller establishes safe re-entry, including
     * alignment/state/P repair if needed. Clears session/health/sample-count
     * history, NOT the estimator or time/source watermarks. This is not needed
     * for ordinary data or fusion-timeout stops, which can restart automatically.
     * Also call after frame/field-policy changes or an external yaw reset.
     * Field-reference revisions use Source::reference_id during normal update.
     * Never restart repeatedly merely to bypass a fusion timeout.
     */
    void restart() noexcept
    {
        m_state = State{};
        m_sample_count = 0U;
        m_heading_reset_required = false;
    }

private:
    void stop(StopReason reason) noexcept;
    void require_recovery(RecoveryReason reason) noexcept;
    void check_fusion_timeout() noexcept;
    void update_source(Source const & source, Conditions const & conditions) noexcept;

    State m_state{};
    Parameters m_parameters{};
    Source m_source{};
    std::uint64_t m_update_time_us{};
    std::uint64_t m_sample_time_us{};
    unsigned m_sample_count{}; // Saturates at the four-sample startup requirement.
    bool m_reset_on_timeout = false;
    bool m_reset_sample_qualified = false;
    bool m_heading_reset_required = false;
}; /* end class MagnetometerFusion */

inline void MagnetometerFusion::stop(StopReason reason) noexcept
{
    m_state.active = false;
    m_state.fusion_allowed = false;
    m_state.reset_allowed = false;
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
    auto const progress =
        m_state.last_fuse_time_us > m_state.last_reset_time_us ? m_state.last_fuse_time_us : m_state.last_reset_time_us;
    if (m_state.active && m_update_time_us - progress > m_parameters.fusion_timeout_us)
    {
        if (m_reset_on_timeout)
        {
            require_recovery(RecoveryReason::fusion_timeout);
            m_state.reset_allowed = m_reset_sample_qualified;
        }
        else
        {
            stop(StopReason::fusion_timeout);
        }
    }
}

inline void MagnetometerFusion::update_source(Source const & source, Conditions const & conditions) noexcept
{
    bool const source_changed = m_source.device_id != 0U && (source.device_id != m_source.device_id ||
                                                             source.calibration_count != m_source.calibration_count);
    if (source_changed)
    {
        bool const numerical_repair = m_state.recovery_reason == RecoveryReason::fusion_failed ||
                                      m_state.recovery_reason == RecoveryReason::reset_failed;
        auto const reason = m_state.recovery_reason;
        auto const last_field_failure = m_state.last_field_failure_time_us;
        // Stopping magnetic heading aiding loses alignment without horizontal
        // aiding. A ground calibration change also requires fresh alignment.
        m_heading_reset_required = m_heading_reset_required || (m_state.active && !conditions.horizontal_aiding);
        m_state = State{};
        m_state.last_field_failure_time_us = last_field_failure;
        m_sample_count = 0U;
        require_recovery(numerical_repair ? reason : RecoveryReason::source_changed);
    }
    else if (m_source.device_id != 0U && source.reference_id != m_source.reference_id &&
             m_state.recovery_reason == RecoveryReason::none && m_state.active && m_reset_on_timeout)
    {
        // A reference update does not invalidate this sensor's LPF history.
        require_recovery(RecoveryReason::reference_changed);
    }
    m_source = source;
    if (m_state.recovery_reason == RecoveryReason::source_changed && conditions.flight_state == FlightState::on_ground)
    {
        m_heading_reset_required = true;
    }
}

inline Status MagnetometerFusion::update(std::uint64_t now_us, Conditions const & conditions,
                                         Parameters const & parameters, Sample const & sample) noexcept
{
    m_state.fusion_allowed = false;
    m_state.reset_allowed = false;
    m_state.filter_reset = false;
    m_reset_sample_qualified = false;
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
    if (conditions.flight_state != FlightState::unknown && conditions.flight_state != FlightState::on_ground &&
        conditions.flight_state != FlightState::in_air)
    {
        stop(StopReason::invalid_call);
        return Status::domain_error;
    }

    m_update_time_us = now_us;
    m_parameters = parameters;
    m_reset_on_timeout = !conditions.horizontal_aiding || (conditions.at_rest && conditions.heading_consistent);
    bool const new_sample = sample.time_us != 0U;
    if (new_sample)
    {
        update_source(sample.source, conditions);
        m_sample_time_us = sample.time_us;
    }

    if (!conditions.enabled)
    {
        stop(StopReason::disabled);
        return Status::success;
    }
    if (new_sample && m_sample_count < 4U)
    {
        m_state.filter_reset = m_sample_count == 0U;
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
    else if (m_state.recovery_reason == RecoveryReason::fusion_failed ||
             m_state.recovery_reason == RecoveryReason::reset_failed)
    {
        stop(StopReason::recovery_required);
    }
    else
    {
        if (!sample.field_accepted || m_state.last_field_failure_time_us == 0U)
        {
            m_state.last_field_failure_time_us = now_us;
        }
        m_reset_sample_qualified = conditions.starting_allowed && sample.field_accepted && m_sample_count >= 4U &&
                                   now_us - m_sample_time_us < parameters.maximum_interval_us &&
                                   now_us - m_state.last_field_failure_time_us > parameters.health_time_us;
        if (m_state.recovery_reason == RecoveryReason::source_changed)
        {
            stop(StopReason::recovery_required);
            if (m_reset_sample_qualified && conditions.flight_state != FlightState::unknown)
            {
                if (m_heading_reset_required)
                {
                    m_state.reset_allowed = true;
                }
                else
                {
                    m_state.recovery_reason = RecoveryReason::none;
                    m_state.stop_reason = StopReason::none;
                    m_state.fusion_allowed = true;
                }
            }
        }
        else if (m_state.recovery_reason == RecoveryReason::fusion_timeout ||
                 m_state.recovery_reason == RecoveryReason::reference_changed)
        {
            if (m_reset_on_timeout)
            {
                stop(StopReason::recovery_required);
                m_state.reset_allowed = m_reset_sample_qualified;
            }
            else
            {
                // A recovered horizontal aid removes the timeout-reset policy.
                // Stop normally; a following qualified sample may retry fusion.
                auto const reason = m_state.recovery_reason;
                m_state.recovery_reason = RecoveryReason::none;
                stop(reason == RecoveryReason::fusion_timeout ? StopReason::fusion_timeout : StopReason::none);
            }
        }
        else if (m_state.active)
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
    if (m_state.fusion_allowed && conditions.horizontal_aiding && !conditions.heading_consistent)
    {
        // Intentional navigation inhibition is neither a failed fusion attempt
        // nor magnetic-state-only fusion. Preserve the session and real clocks.
        m_state.fusion_allowed = false;
        m_state.stop_reason = StopReason::heading_inconsistent;
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

inline Status MagnetometerFusion::record_reset(std::uint64_t sample_time_us, bool reset_succeeded) noexcept
{
    if (!m_state.reset_allowed || sample_time_us != m_sample_time_us)
    {
        return Status::out_of_range;
    }
    m_state.reset_allowed = false;
    if (reset_succeeded)
    {
        if (m_state.start_time_us == 0U)
        {
            m_state.start_time_us = m_update_time_us;
        }
        m_state.active = true;
        m_state.recovery_reason = RecoveryReason::none;
        m_state.stop_reason = StopReason::none;
        m_state.last_reset_time_us = m_update_time_us;
        m_heading_reset_required = false;
    }
    else
    {
        m_state.recovery_reason = RecoveryReason::reset_failed;
        stop(StopReason::recovery_required);
    }
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
