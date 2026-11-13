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
    /** One permitted operation for the current sample, consumed by feedback. */
    enum class Action : std::uint8_t
    {
        none,
        fuse,
        reset,
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

    static constexpr std::uint64_t acceleration_reset_delay_us = 500'000U;

    /** Zero update time means unavailable; zero last-good with a known update means never healthy. */
    struct VerticalAccelerationTimes
    {
        std::uint64_t update_time_us{};
        std::uint64_t last_good_time_us{};

        /** Caller validates timestamp ordering; unavailable health cannot request a reset. */
        [[nodiscard]] constexpr bool reset_required(std::uint64_t now_us) const
        {
            return update_time_us != 0U && update_time_us <= now_us && last_good_time_us <= update_time_us &&
                   (last_good_time_us == 0U || now_us - last_good_time_us > acceleration_reset_delay_us);
        }
    };

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
        bool active = false;          // Monitoring session, not proof of estimate validity.
        Action action = Action::none; // Independent of session activity and recovery history.
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
        // Non-none grants one Down-only reset; no second permission flag.
        StopReason vertical_velocity_reset_reason = StopReason::none;
        std::uint64_t last_vertical_velocity_reset_time_us{};
        // One-update notification: restart quality health timers, retaining qualification.
        bool restart_quality = false;
    };

    [[nodiscard]] State const & state() const { return m_state; }
    void reset() { *this = GnssFusion{}; }

    /**
     * Positive durations are fixed until reset; horizontal timeout <= UINT64_MAX/2.
     * now_us increases, except distinct new samples may share one fusion tick.
     * Sample times strictly increase and cannot be in the future. Invalid calls
     * revoke permissions but consume no time/configuration/history.
     */
    [[nodiscard]] Status update(std::uint64_t now_us, Conditions const & conditions, Parameters const & parameters,
                                Sample const & sample);

    /**
     * Consume one permission; only fused refreshes that group's progress.
     * Height consumes the CURRENT horizon's health times after its attempt.
     * Other groups ignore this argument. Unavailable health leaves timeout recovery intact.
     */
    [[nodiscard]] Status record_fusion(Group group, std::uint64_t sample_time_us, FusionDecision decision,
                                       VerticalAccelerationTimes const & vertical_acceleration = {0U, 0U});

    /** Consume a reset permission after the checked state/P operation. */
    [[nodiscard]] Status record_reset(Group group, std::uint64_t sample_time_us, bool reset_succeeded);

    /** Acknowledge the permitted Down-only reset before this sample's height operation. */
    [[nodiscard]] Status record_vertical_velocity_reset(std::uint64_t sample_time_us, bool reset_succeeded);

    /** Acknowledge caller-established safe re-entry; does not reset estimator or quality. */
    [[nodiscard]] Status restart(Group group);

private:
    void revoke_permissions();
    void clear_vertical_velocity_reset();
    [[nodiscard]] bool height_waiting_for_velocity() const;
    [[nodiscard]] Status validate_update(std::uint64_t now_us, Conditions const & conditions,
                                         Parameters const & parameters, Sample const & sample) const;
    void request_horizontal_resets(Conditions const & conditions);
    [[nodiscard]] GroupState * select_group(Group group);
    static void stop(GroupState & group, StopReason reason);
    static void require_recovery(GroupState & group, StopReason reason);
    static void request_reset(GroupState & group, StopReason reason);
    [[nodiscard]] bool expired(GroupState const & group, std::uint64_t duration) const;
    [[nodiscard]] StopReason height_reset_reason(VerticalAccelerationTimes const & health) const;
    void update_group(GroupState & group, GroupConditions conditions, bool new_sample, bool accepted, bool fresh);

    State m_state{};
    Parameters m_parameters{};
    std::uint64_t m_update_time_us{};
    std::uint64_t m_sample_time_us{};
    std::uint64_t m_last_quality_pass_us{};
    StopReason m_vertical_velocity_reset_pending = StopReason::none;
}; /* end class GnssFusion */

inline void GnssFusion::clear_vertical_velocity_reset()
{
    m_vertical_velocity_reset_pending = StopReason::none;
    m_state.vertical_velocity_reset_reason = StopReason::none;
}

inline void GnssFusion::revoke_permissions()
{
    m_state.horizontal_position.action = Action::none;
    m_state.vertical_position.action = Action::none;
    m_state.velocity.action = Action::none;
    clear_vertical_velocity_reset();
}

inline bool GnssFusion::height_waiting_for_velocity() const
{
    return m_vertical_velocity_reset_pending != StopReason::none ||
           m_state.vertical_velocity_reset_reason != StopReason::none;
}

inline GnssFusion::GroupState * GnssFusion::select_group(Group group)
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

inline void GnssFusion::stop(GroupState & group, StopReason reason)
{
    group.active = false;
    group.action = Action::none;
    if (!group.recovery_required)
    {
        group.stop_reason = reason;
    }
}

inline void GnssFusion::require_recovery(GroupState & group, StopReason reason)
{
    stop(group, reason);
    group.recovery_required = true;
}

inline void GnssFusion::request_reset(GroupState & group, StopReason reason)
{
    // A timeout reset keeps the session alive; it is not a quality stop.
    if (!group.recovery_required)
    {
        group.stop_reason = reason;
    }
    group.recovery_required = true;
    group.action = Action::none;
}

inline bool GnssFusion::expired(GroupState const & group, std::uint64_t duration) const
{
    auto progress = group.last_fuse_time_us > group.start_time_us ? group.last_fuse_time_us : group.start_time_us;
    progress = group.last_reset_time_us > progress ? group.last_reset_time_us : progress;
    return progress == 0U || m_update_time_us - progress > duration;
}

inline GnssFusion::StopReason GnssFusion::height_reset_reason(VerticalAccelerationTimes const & health) const
{
    if (health.reset_required(m_update_time_us))
    {
        return StopReason::bad_vertical_acceleration;
    }
    return expired(m_state.vertical_position, m_parameters.height_fusion_timeout_us) ? StopReason::fusion_timeout
                                                                                     : StopReason::none;
}

inline void GnssFusion::update_group(GroupState & group, GroupConditions conditions, bool new_sample, bool accepted,
                                     bool fresh)
{
    if (!conditions.enabled)
    {
        stop(group, StopReason::disabled);
        return;
    }
    if (!conditions.ready)
    {
        stop(group, StopReason::not_ready);
        return;
    }
    if (!fresh)
    {
        stop(group, StopReason::data_timeout);
        return;
    }
    if (!new_sample && group.active)
    {
        return;
    }
    if (!group.recovery_required && accepted && !group.active && !conditions.start_without_reset)
    {
        request_reset(group, StopReason::initialization_required);
    }
    if (group.recovery_required)
    {
        group.action = accepted && (group.stop_reason == StopReason::initialization_required ||
                                    group.stop_reason == StopReason::fusion_timeout ||
                                    group.stop_reason == StopReason::bad_vertical_acceleration)
                           ? Action::reset
                           : Action::none;
        return;
    }
    group.stop_reason = accepted ? StopReason::none : StopReason::waiting_for_quality;
    if (!accepted)
    {
        return;
    }
    if (!group.active)
    {
        // Caller-authorized initialized start; not an invented successful fusion.
        group.start_time_us = m_update_time_us;
    }
    group.active = true;
    group.action = Action::fuse;
}

inline Status GnssFusion::validate_update(std::uint64_t now_us, Conditions const & conditions,
                                          Parameters const & parameters, Sample const & sample) const
{
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

    return Status::success;
}

inline void GnssFusion::request_horizontal_resets(Conditions const & conditions)
{
    // Called with accepted fresh data, before either horizontal group attempts fusion.
    bool const both_expired = expired(m_state.horizontal_position, m_parameters.horizontal_timeout_us) &&
                              expired(m_state.velocity, m_parameters.horizontal_timeout_us);
    bool const position_expired = conditions.horizontal_position.enabled &&
                                  expired(m_state.horizontal_position, 2U * m_parameters.horizontal_timeout_us);
    if (!both_expired && !position_expired)
    {
        return;
    }
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

inline Status GnssFusion::update(std::uint64_t now_us, Conditions const & conditions, Parameters const & parameters,
                                 Sample const & sample)
{
    revoke_permissions();
    auto const status = validate_update(now_us, conditions, parameters, sample);
    if (!succeeded(status))
    {
        return status;
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
        if (accepted && horizontal_fresh && horizontal_was_active)
        {
            request_horizontal_resets(conditions);
        }
        update_group(m_state.velocity, conditions.velocity, new_sample, accepted, horizontal_fresh);
        update_group(m_state.horizontal_position, conditions.horizontal_position, new_sample, accepted,
                     horizontal_fresh);
        update_group(m_state.vertical_position, conditions.vertical_position, new_sample, accepted, height_fresh);
        // Snapshot height timeout and preceding acceleration health before height control.
        // A full velocity reset already includes Down; only normal fusion can
        // request this additional reset, after its actual result is recorded.
        if (conditions.has_vertical_velocity && velocity_was_active && m_state.velocity.action == Action::fuse &&
            m_state.vertical_position.action != Action::none && m_state.vertical_position.start_time_us != 0U)
        {
            m_vertical_velocity_reset_pending = height_reset_reason(conditions.vertical_acceleration);
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
                                        VerticalAccelerationTimes const & vertical_acceleration)
{
    auto * group_state = select_group(group);
    if (group_state == nullptr || (decision != FusionDecision::fused && decision != FusionDecision::rejected &&
                                   decision != FusionDecision::failed))
    {
        return Status::domain_error;
    }
    if (group_state->action != Action::fuse || sample_time_us != m_sample_time_us ||
        (group == Group::vertical_position && height_waiting_for_velocity()))
    {
        return Status::out_of_range;
    }
    if (group == Group::vertical_position && vertical_acceleration.update_time_us != 0U &&
        (vertical_acceleration.update_time_us != m_update_time_us ||
         vertical_acceleration.last_good_time_us > vertical_acceleration.update_time_us))
    {
        return Status::out_of_range;
    }
    group_state->action = Action::none;
    switch (decision)
    {
    case FusionDecision::fused:
        group_state->last_fuse_time_us = m_update_time_us;
        break;
    case FusionDecision::rejected:
        break;
    case FusionDecision::failed:
        require_recovery(*group_state, StopReason::fusion_failed);
        break;
    }
    if (group == Group::velocity)
    {
        m_state.vertical_velocity_reset_reason =
            decision != FusionDecision::failed ? m_vertical_velocity_reset_pending : StopReason::none;
        m_vertical_velocity_reset_pending = StopReason::none;
    }
    if (group == Group::vertical_position && group_state->active)
    {
        auto const reason = height_reset_reason(vertical_acceleration);
        if (reason != StopReason::none)
        {
            // Successful height fusion clears its timeout, but not continuous bad acceleration.
            request_reset(*group_state, reason);
            group_state->action = Action::reset;
        }
    }
    return Status::success;
}

inline Status GnssFusion::restart(Group group)
{
    auto * group_state = select_group(group);
    if (group_state == nullptr)
    {
        return Status::domain_error;
    }
    *group_state = GroupState{};
    if (group == Group::velocity || group == Group::vertical_position)
    {
        clear_vertical_velocity_reset();
    }
    if (group == Group::velocity)
    {
        m_state.last_vertical_velocity_reset_time_us = 0U;
    }
    return Status::success;
}

inline Status GnssFusion::record_reset(Group group, std::uint64_t sample_time_us, bool reset_succeeded)
{
    auto * group_state = select_group(group);
    if (group_state == nullptr)
    {
        return Status::domain_error;
    }
    if (group_state->action != Action::reset || sample_time_us != m_sample_time_us ||
        (group == Group::vertical_position && height_waiting_for_velocity()))
    {
        return Status::out_of_range;
    }
    group_state->action = Action::none;
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

inline Status GnssFusion::record_vertical_velocity_reset(std::uint64_t sample_time_us, bool reset_succeeded)
{
    if (m_state.vertical_velocity_reset_reason == StopReason::none || sample_time_us != m_sample_time_us)
    {
        return Status::out_of_range;
    }
    clear_vertical_velocity_reset();
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
