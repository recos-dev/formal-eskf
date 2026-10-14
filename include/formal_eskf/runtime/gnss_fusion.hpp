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
        fusion_failed,
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

    struct Conditions
    {
        GroupConditions horizontal_position{};
        GroupConditions vertical_position{};
        GroupConditions velocity{};
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
        bool recovery_required = false;
        StopReason stop_reason = StopReason::disabled;
        std::uint64_t start_time_us{};
        std::uint64_t last_fuse_time_us{}; // Actual fused feedback only, never startup/reset.
    };

    struct State
    {
        GroupState horizontal_position{};
        GroupState vertical_position{};
        GroupState velocity{};
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

    /** Consume one permission; only fused refreshes that group's progress. */
    [[nodiscard]] Status record_fusion(Group group, std::uint64_t sample_time_us, FusionDecision decision) noexcept;

    /** Acknowledge caller-established safe re-entry; does not reset estimator or quality. */
    [[nodiscard]] Status restart(Group group) noexcept;

private:
    [[nodiscard]] GroupState * select_group(Group group) noexcept;
    static void stop(GroupState & group, StopReason reason) noexcept;
    static void require_recovery(GroupState & group, StopReason reason) noexcept;
    [[nodiscard]] bool expired(GroupState const & group, std::uint64_t duration) const noexcept;
    void update_group(GroupState & group, GroupConditions conditions, bool new_sample, bool accepted,
                      bool fresh) noexcept;

    State m_state{};
    Parameters m_parameters{};
    std::uint64_t m_update_time_us{};
    std::uint64_t m_sample_time_us{};
    std::uint64_t m_last_quality_pass_us{};
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

inline bool GnssFusion::expired(GroupState const & group, std::uint64_t duration) const noexcept
{
    auto const progress = group.last_fuse_time_us > group.start_time_us ? group.last_fuse_time_us : group.start_time_us;
    return progress == 0U || m_update_time_us - progress > duration;
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
    else if (!group.recovery_required && (new_sample || !group.active))
    {
        group.stop_reason = accepted ? StopReason::none : StopReason::waiting_for_quality;
        if (accepted)
        {
            if (!group.active && !conditions.start_without_reset)
            {
                require_recovery(group, StopReason::initialization_required);
                return;
            }
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
    if (sample.time_us != 0U && sample.quality_available && sample.quality != GnssQualityDecision::accepted &&
        sample.quality != GnssQualityDecision::waiting && sample.quality != GnssQualityDecision::rejected)
    {
        return Status::domain_error;
    }

    bool const horizontal_was_active = m_state.horizontal_position.active || m_state.velocity.active;
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
                require_recovery(m_state.horizontal_position, StopReason::fusion_timeout);
            }
            if (m_state.velocity.active && conditions.velocity.enabled && conditions.velocity.ready)
            {
                require_recovery(m_state.velocity, StopReason::fusion_timeout);
            }
        }
        update_group(m_state.velocity, conditions.velocity, new_sample, accepted, horizontal_fresh);
        update_group(m_state.horizontal_position, conditions.horizontal_position, new_sample, accepted,
                     horizontal_fresh);
        update_group(m_state.vertical_position, conditions.vertical_position, new_sample, accepted, height_fresh);
    }
    if (horizontal_was_active && !m_state.horizontal_position.active && !m_state.velocity.active)
    {
        m_state.restart_quality = true;
        m_last_quality_pass_us = 0U;
    }
    return Status::success;
}

inline Status GnssFusion::record_fusion(Group group, std::uint64_t sample_time_us, FusionDecision decision) noexcept
{
    auto * group_state = select_group(group);
    if (group_state == nullptr || (decision != FusionDecision::fused && decision != FusionDecision::rejected &&
                                   decision != FusionDecision::failed))
    {
        return Status::domain_error;
    }
    if (!group_state->fusion_allowed || sample_time_us != m_sample_time_us)
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
    if (group == Group::vertical_position && group_state->active &&
        expired(*group_state, m_parameters.height_fusion_timeout_us))
    {
        // With GNSS the sole height reference, a failed height session needs reset.
        // Check after this attempt: successful correction at an overdue tick wins.
        require_recovery(*group_state, StopReason::fusion_timeout);
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
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
