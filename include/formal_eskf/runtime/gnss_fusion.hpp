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

#include <formal_eskf/runtime/gnss_quality.hpp>
#include <formal_eskf/runtime/measurement_fusion.hpp>

namespace formal_eskf::runtime
{

/**
 * Single-writer lifecycle for GNSS as the sole horizontal/height aiding source.
 * Owns no estimator, quality timer, clock, buffer or reset implementation.
 * Update every estimator tick; record each permitted group's actual decision
 * before the next update. Use horizontal OR 3D velocity, never both per sample.
 * With has_vertical_velocity, process velocity and any permitted Down-only
 * reset before this sample's height correction/reset.
 * Reset requests require caller-established state/P repair, not a blind retry.
 */
class GnssFusion
{
public:
    enum class Group
    {
        horizontal_position,
        vertical_position,
        velocity,
    };
    enum class StopReason
    {
        none,
        disabled,
        not_ready,
        waiting_for_quality,
        data_timeout,
        poor_quality,
        initialization_required,
        fusion_timeout,
        bad_vertical_acceleration,
        fusion_failed,
        reset_failed,
    };

    struct Parameters
    {
        std::uint64_t horizontal_timeout_us{};
        std::uint64_t height_data_timeout_us{};
        std::uint64_t height_fusion_timeout_us{};
    };

    struct GroupConditions
    {
        bool enabled = false;
        // Includes initialized frame/state and required attitude alignment.
        bool ready = false;
        // Caller has established safe startup without another estimator reset.
        bool start_without_reset = false;
    };

    /** Zero update time means unavailable; zero last-good with a known update means never healthy. */
    struct VerticalAccelerationTimes
    {
        std::uint64_t update_time_us{};
        std::uint64_t last_good_time_us{};
    };

    static constexpr std::uint64_t acceleration_reset_delay_us = 500'000U;

    struct Conditions
    {
        GroupConditions horizontal_position{};
        GroupConditions vertical_position{};
        GroupConditions velocity{};
        // Velocity group includes a qualified Down observation, not just N/E.
        bool has_vertical_velocity = false;
        // Verdict available before velocity control, not the held 3 s fault flag.
        VerticalAccelerationTimes vertical_acceleration{};
    };

    struct Sample
    {
        std::uint64_t time_us{}; // Zero: no new receiver sample; other fields ignored.
        GnssQualityDecision quality = GnssQualityDecision::invalid_call;
        // Missing adapter metadata is neither a passing nor a failing quality check.
        bool quality_available = true;
    };

    struct GroupState
    {
        bool active = false; // Monitoring session, not proof of estimate validity.
        bool fusion_allowed = false;
        bool reset_allowed = false; // Current accepted sample; mutually exclusive with fusion.
        bool recovery_required = false;
        StopReason stop_reason = StopReason::disabled;
        std::uint64_t start_time_us{};
        std::uint64_t last_fuse_time_us{}; // Actual fused feedback only, never startup/reset.
        std::uint64_t last_reset_time_us{};
    };

    struct State
    {
        GroupState horizontal_position{};
        GroupState vertical_position{};
        GroupState velocity{};
        bool vertical_velocity_reset_allowed = false;
        StopReason vertical_velocity_reset_reason = StopReason::none;
        std::uint64_t last_vertical_velocity_reset_time_us{};
        // One-update notification: restart quality health timers, retaining qualification.
        bool restart_quality = false;
    };

    [[nodiscard]] State const & state() const noexcept { return m_state; }
    void reset() noexcept { *this = GnssFusion{}; }

    /**
     * Positive durations are fixed until reset; horizontal timeout <= UINT64_MAX/2.
     * now_us increases, except distinct new samples may share one fusion tick.
     * Sample times strictly increase and cannot be in the future. Invalid calls
     * revoke permissions but consume no time/configuration/history.
     */
    [[nodiscard]] Status update(std::uint64_t now_us, Conditions const & conditions, Parameters const & parameters,
                                Sample const & sample) noexcept;

    /**
     * Consume one permission; only fused refreshes that group's progress.
     * Height consumes the CURRENT horizon's health times after its attempt.
     * Other groups ignore this argument. Unavailable health leaves timeout recovery intact.
     */
    [[nodiscard]] Status record_fusion(Group group, std::uint64_t sample_time_us, FusionDecision decision,
                                       VerticalAccelerationTimes const & vertical_acceleration = {0U, 0U}) noexcept;

    /** Consume a reset permission after the checked state/P operation. */
    [[nodiscard]] Status record_reset(Group group, std::uint64_t sample_time_us, bool reset_succeeded) noexcept;

    /** Acknowledge the permitted Down-only reset before this sample's height operation. */
    [[nodiscard]] Status record_vertical_velocity_reset(std::uint64_t sample_time_us, bool reset_succeeded) noexcept;

    /** Acknowledge caller-established safe re-entry; does not reset estimator or quality. */
    [[nodiscard]] Status restart(Group group) noexcept;

private:
    [[nodiscard]] GroupState * select_group(Group group) noexcept;
    static void stop(GroupState & group, StopReason reason) noexcept;
    static void require_recovery(GroupState & group, StopReason reason) noexcept;
    static void request_reset(GroupState & group, StopReason reason) noexcept;
    [[nodiscard]] bool expired(GroupState const & group, std::uint64_t duration) const noexcept;
    [[nodiscard]] StopReason height_reset_reason(VerticalAccelerationTimes const & health) const noexcept;
    void update_group(GroupState & group, GroupConditions conditions, bool new_sample, bool accepted,
                      bool fresh) noexcept;

    State m_state{};
    Parameters m_parameters{};
    std::uint64_t m_update_time_us{};
    std::uint64_t m_sample_time_us{};
    std::uint64_t m_last_quality_pass_us{};
    StopReason m_vertical_velocity_reset_pending = StopReason::none;
}; /* end class GnssFusion */

inline GnssFusion::GroupState * GnssFusion::select_group(Group group) noexcept
{
    switch (group)
    {
    case Group::horizontal_position:
        return &m_state.horizontal_position;
    case Group::vertical_position:
        return &m_state.vertical_position;
    case Group::velocity:
        return &m_state.velocity;
    }
    return nullptr;
}

inline void GnssFusion::stop(GroupState & group, StopReason reason) noexcept
{
    group.active = false;
    group.fusion_allowed = false;
    group.reset_allowed = false;
    if (!group.recovery_required)
    {
        group.stop_reason = reason;
    }
}

inline void GnssFusion::require_recovery(GroupState & group, StopReason reason) noexcept
{
    stop(group, reason);
    group.recovery_required = true;
}

inline void GnssFusion::request_reset(GroupState & group, StopReason reason) noexcept
{
    // A timeout reset keeps the session alive; it is not a quality stop.
    if (!group.recovery_required)
    {
        group.stop_reason = reason;
    }
    group.recovery_required = true;
    group.fusion_allowed = false;
}

inline bool GnssFusion::expired(GroupState const & group, std::uint64_t duration) const noexcept
{
    auto progress = group.last_fuse_time_us > group.start_time_us ? group.last_fuse_time_us : group.start_time_us;
    progress = group.last_reset_time_us > progress ? group.last_reset_time_us : progress;
    return progress == 0U || m_update_time_us - progress > duration;
}

inline GnssFusion::StopReason GnssFusion::height_reset_reason(VerticalAccelerationTimes const & health) const noexcept
{
    if (health.update_time_us != 0U &&
        (health.last_good_time_us == 0U || m_update_time_us - health.last_good_time_us > acceleration_reset_delay_us))
    {
        return StopReason::bad_vertical_acceleration;
    }
    return expired(m_state.vertical_position, m_parameters.height_fusion_timeout_us) ? StopReason::fusion_timeout
                                                                                     : StopReason::none;
}

inline void GnssFusion::update_group(GroupState & group, GroupConditions conditions, bool new_sample, bool accepted,
                                     bool fresh) noexcept
{
    if (!conditions.enabled)
    {
        stop(group, StopReason::disabled);
    }
    else if (!conditions.ready)
    {
        stop(group, StopReason::not_ready);
    }
    else if (!fresh)
    {
        stop(group, StopReason::data_timeout);
    }
    else if (new_sample || !group.active)
    {
        if (!group.recovery_required && accepted && !group.active && !conditions.start_without_reset)
        {
            request_reset(group, StopReason::initialization_required);
        }
        if (group.recovery_required)
        {
            group.reset_allowed = accepted && (group.stop_reason == StopReason::initialization_required ||
                                               group.stop_reason == StopReason::fusion_timeout ||
                                               group.stop_reason == StopReason::bad_vertical_acceleration);
            return;
        }
        group.stop_reason = accepted ? StopReason::none : StopReason::waiting_for_quality;
        if (accepted)
        {
            if (!group.active)
            {
                // Caller-authorized initialized start; not an invented successful fusion.
                group.start_time_us = m_update_time_us;
            }
            group.active = true;
            group.fusion_allowed = true;
        }
    }
}

inline Status GnssFusion::update(std::uint64_t now_us, Conditions const & conditions, Parameters const & parameters,
                                 Sample const & sample) noexcept
{
    m_state.horizontal_position.fusion_allowed = false;
    m_state.vertical_position.fusion_allowed = false;
    m_state.velocity.fusion_allowed = false;
    m_state.horizontal_position.reset_allowed = false;
    m_state.vertical_position.reset_allowed = false;
    m_state.velocity.reset_allowed = false;
    m_state.vertical_velocity_reset_allowed = false;
    m_state.vertical_velocity_reset_reason = StopReason::none;
    m_vertical_velocity_reset_pending = StopReason::none;
    if (parameters.horizontal_timeout_us == 0U ||
        parameters.horizontal_timeout_us > std::numeric_limits<std::uint64_t>::max() / 2U ||
        parameters.height_data_timeout_us == 0U || parameters.height_fusion_timeout_us == 0U ||
        (m_update_time_us != 0U && (parameters.horizontal_timeout_us != m_parameters.horizontal_timeout_us ||
                                    parameters.height_data_timeout_us != m_parameters.height_data_timeout_us ||
                                    parameters.height_fusion_timeout_us != m_parameters.height_fusion_timeout_us)))
    {
        return Status::domain_error;
    }
    if (now_us == 0U || now_us < m_update_time_us || (now_us == m_update_time_us && sample.time_us == 0U) ||
        (sample.time_us != 0U && (sample.time_us <= m_sample_time_us || sample.time_us > now_us)))
    {
        return Status::out_of_range;
    }
    auto const & health = conditions.vertical_acceleration;
    if (health.update_time_us != 0U &&
        (health.update_time_us > now_us || health.last_good_time_us > health.update_time_us))
    {
        return Status::out_of_range;
    }
    if (sample.time_us != 0U && sample.quality_available && sample.quality != GnssQualityDecision::accepted &&
        sample.quality != GnssQualityDecision::waiting && sample.quality != GnssQualityDecision::rejected)
    {
        return Status::domain_error;
    }

    bool const horizontal_was_active = m_state.horizontal_position.active || m_state.velocity.active;
    bool const velocity_was_active = m_state.velocity.active;
    m_update_time_us = now_us;
    m_parameters = parameters;
    m_state.restart_quality = false;
    bool const new_sample = sample.time_us != 0U;
    bool const accepted = new_sample && sample.quality_available && sample.quality == GnssQualityDecision::accepted;
    if (new_sample)
    {
        m_sample_time_us = sample.time_us;
        if (sample.quality_available && sample.quality != GnssQualityDecision::rejected)
        {
            // Waiting includes currently good checks during the health/recovery hold.
            m_last_quality_pass_us = now_us;
        }
    }
    auto const age = m_sample_time_us == 0U ? std::numeric_limits<std::uint64_t>::max() : now_us - m_sample_time_us;
    bool const horizontal_fresh = m_sample_time_us != 0U && age < parameters.horizontal_timeout_us;
    bool const height_fresh = m_sample_time_us != 0U && age < parameters.height_data_timeout_us;
    bool const poor_quality =
        new_sample && !accepted && horizontal_was_active &&
        (m_last_quality_pass_us == 0U || now_us - m_last_quality_pass_us > parameters.horizontal_timeout_us);

    if (poor_quality || (!new_sample && horizontal_was_active && !horizontal_fresh))
    {
        auto const reason = poor_quality ? StopReason::poor_quality : StopReason::data_timeout;
        stop(m_state.horizontal_position, reason);
        stop(m_state.velocity, reason);
        stop(m_state.vertical_position, reason);
    }
    else
    {
        // Evaluate the coupled horizontal reset decision before either group fuses.
        bool const horizontal_expired = (expired(m_state.horizontal_position, parameters.horizontal_timeout_us) &&
                                         expired(m_state.velocity, parameters.horizontal_timeout_us)) ||
                                        (conditions.horizontal_position.enabled &&
                                         expired(m_state.horizontal_position, 2U * parameters.horizontal_timeout_us));
        if (accepted && horizontal_fresh && horizontal_was_active && horizontal_expired)
        {
            if (m_state.horizontal_position.active && conditions.horizontal_position.enabled &&
                conditions.horizontal_position.ready)
            {
                request_reset(m_state.horizontal_position, StopReason::fusion_timeout);
            }
            if (m_state.velocity.active && conditions.velocity.enabled && conditions.velocity.ready)
            {
                request_reset(m_state.velocity, StopReason::fusion_timeout);
            }
        }
        update_group(m_state.velocity, conditions.velocity, new_sample, accepted, horizontal_fresh);
        update_group(m_state.horizontal_position, conditions.horizontal_position, new_sample, accepted,
                     horizontal_fresh);
        update_group(m_state.vertical_position, conditions.vertical_position, new_sample, accepted, height_fresh);
        // Snapshot height timeout and preceding acceleration health before height control.
        // A full velocity reset already includes Down; only normal fusion can
        // request this additional reset, after its actual result is recorded.
        if (conditions.has_vertical_velocity && velocity_was_active && m_state.velocity.fusion_allowed &&
            (m_state.vertical_position.fusion_allowed || m_state.vertical_position.reset_allowed) &&
            m_state.vertical_position.start_time_us != 0U)
        {
            m_vertical_velocity_reset_pending = height_reset_reason(health);
        }
    }
    if (horizontal_was_active && !m_state.horizontal_position.active && !m_state.velocity.active)
    {
        m_state.restart_quality = true;
        m_last_quality_pass_us = 0U;
    }
    return Status::success;
}

inline Status GnssFusion::record_fusion(Group group, std::uint64_t sample_time_us, FusionDecision decision,
                                        VerticalAccelerationTimes const & vertical_acceleration) noexcept
{
    auto * group_state = select_group(group);
    if (group_state == nullptr || (decision != FusionDecision::fused && decision != FusionDecision::rejected &&
                                   decision != FusionDecision::failed))
    {
        return Status::domain_error;
    }
    if (!group_state->fusion_allowed || sample_time_us != m_sample_time_us ||
        (group == Group::vertical_position &&
         (m_vertical_velocity_reset_pending != StopReason::none || m_state.vertical_velocity_reset_allowed)))
    {
        return Status::out_of_range;
    }
    if (group == Group::vertical_position && vertical_acceleration.update_time_us != 0U &&
        (vertical_acceleration.update_time_us != m_update_time_us ||
         vertical_acceleration.last_good_time_us > vertical_acceleration.update_time_us))
    {
        return Status::out_of_range;
    }
    group_state->fusion_allowed = false;
    if (decision == FusionDecision::fused)
    {
        group_state->last_fuse_time_us = m_update_time_us;
    }
    else if (decision == FusionDecision::failed)
    {
        require_recovery(*group_state, StopReason::fusion_failed);
    }
    if (group == Group::velocity)
    {
        m_state.vertical_velocity_reset_allowed =
            m_vertical_velocity_reset_pending != StopReason::none && decision != FusionDecision::failed;
        m_state.vertical_velocity_reset_reason =
            m_state.vertical_velocity_reset_allowed ? m_vertical_velocity_reset_pending : StopReason::none;
        m_vertical_velocity_reset_pending = StopReason::none;
    }
    if (group == Group::vertical_position && group_state->active)
    {
        auto const reason = height_reset_reason(vertical_acceleration);
        if (reason != StopReason::none)
        {
            // Successful height fusion clears its timeout, but not continuous bad acceleration.
            request_reset(*group_state, reason);
            group_state->reset_allowed = true;
        }
    }
    return Status::success;
}

inline Status GnssFusion::restart(Group group) noexcept
{
    auto * group_state = select_group(group);
    if (group_state == nullptr)
    {
        return Status::domain_error;
    }
    *group_state = GroupState{};
    if (group == Group::velocity || group == Group::vertical_position)
    {
        m_vertical_velocity_reset_pending = StopReason::none;
        m_state.vertical_velocity_reset_allowed = false;
        m_state.vertical_velocity_reset_reason = StopReason::none;
    }
    if (group == Group::velocity)
    {
        m_state.last_vertical_velocity_reset_time_us = 0U;
    }
    return Status::success;
}

inline Status GnssFusion::record_reset(Group group, std::uint64_t sample_time_us, bool reset_succeeded) noexcept
{
    auto * group_state = select_group(group);
    if (group_state == nullptr)
    {
        return Status::domain_error;
    }
    if (!group_state->reset_allowed || sample_time_us != m_sample_time_us ||
        (group == Group::vertical_position &&
         (m_vertical_velocity_reset_pending != StopReason::none || m_state.vertical_velocity_reset_allowed)))
    {
        return Status::out_of_range;
    }
    group_state->reset_allowed = false;
    if (!reset_succeeded)
    {
        group_state->stop_reason = StopReason::reset_failed;
        require_recovery(*group_state, StopReason::reset_failed);
        return Status::success;
    }
    if (!group_state->active)
    {
        group_state->start_time_us = m_update_time_us;
    }
    group_state->active = true;
    group_state->recovery_required = false;
    group_state->stop_reason = StopReason::none;
    group_state->last_reset_time_us = m_update_time_us;
    // No correction after reset; height may already have fused or rejected this sample.
    return Status::success;
}

inline Status GnssFusion::record_vertical_velocity_reset(std::uint64_t sample_time_us, bool reset_succeeded) noexcept
{
    if (!m_state.vertical_velocity_reset_allowed || sample_time_us != m_sample_time_us)
    {
        return Status::out_of_range;
    }
    m_state.vertical_velocity_reset_allowed = false;
    m_state.vertical_velocity_reset_reason = StopReason::none;
    if (reset_succeeded)
    {
        // A partial reset is neither successful 3D fusion nor horizontal progress.
        m_state.last_vertical_velocity_reset_time_us = m_update_time_us;
    }
    else
    {
        require_recovery(m_state.velocity, StopReason::reset_failed);
    }
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
