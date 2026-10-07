/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <cstdint>

#include <formal_eskf/runtime/gnss_quality.hpp>
#include <formal_eskf/runtime/measurement_fusion.hpp>

namespace formal_eskf::runtime
{

/**
 * Single-writer GNSS lifecycle; owns no estimator state, clocks or quality timer.
 * Call update on every estimator tick, even without a new sample. On success,
 * fuse each permitted group once and record its actual FusionDecision before
 * the next tick. Use horizontal OR 3D velocity, never both for one sample.
 * All times are nonzero microseconds on one caller-supplied fusion timeline.
 * Reset this controller and GnssQuality on source/frame/configuration changes.
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
        fusion_timeout,
    };

    struct Parameters
    {
        std::uint64_t maximum_sample_age_us{};
        std::uint64_t fusion_timeout_us{};
    };

    struct GroupConditions
    {
        bool enabled = false;
        // Caller establishes the group's state/frame initialization and alignment.
        bool ready = false;
    };

    struct Conditions
    {
        GroupConditions horizontal_position{};
        GroupConditions vertical_position{};
        GroupConditions velocity{};
    };

    struct Sample
    {
        std::uint64_t time_us{}; // Zero means no new sample; quality is then ignored.
        GnssQualityDecision quality = GnssQualityDecision::invalid_call;
    };

    struct GroupState
    {
        bool active = false;         // Monitoring session, not proof of a valid estimate.
        bool fusion_allowed = false; // One new sample, current tick only.
        bool recovery_required = false;
        StopReason stop_reason = StopReason::disabled;
        std::uint64_t start_time_us{};     // First activation since restart; retained across pauses.
        std::uint64_t last_fuse_time_us{}; // Only an actual fused result updates this.
    };

    struct State
    {
        GroupState horizontal_position{};
        GroupState vertical_position{};
        GroupState velocity{};
    };

    [[nodiscard]] State const & state() const noexcept { return m_state; }
    void reset() noexcept { *this = GnssFusion{}; }

    /**
     * now_us strictly increases; each new sample also strictly increases and is
     * no later than now_us. Parameters are positive and fixed until reset.
     * Invalid calls revoke all fusion permissions but preserve other history.
     * Freshness and fusion deadlines are inclusive; only elapsed > limit expires.
     */
    [[nodiscard]] Status update(std::uint64_t now_us, Conditions const & conditions, Parameters const & parameters,
                                Sample const & sample) noexcept;

    /** Consume this group's permission. Rejected/failed never refresh its deadline. */
    [[nodiscard]] Status record_fusion(Group group, std::uint64_t sample_time_us, FusionDecision decision) noexcept;

    /**
     * Explicit caller authorization after establishing safe re-entry (external
     * re-alignment/reset if needed). Clears only this group's lifecycle history;
     * does NOT reset estimator state/P, qualify GNSS or record a successful fuse.
     * Requires a new accepted sample on a later update to activate again.
     */
    [[nodiscard]] Status restart(Group group) noexcept;

private:
    [[nodiscard]] GroupState * select_group(Group group) noexcept;
    void update_group(GroupState & group_state, GroupConditions conditions, bool fresh, bool accepted) noexcept;

    State m_state{};
    Parameters m_parameters{};
    std::uint64_t m_update_time_us{};
    std::uint64_t m_sample_time_us{};
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

inline void GnssFusion::update_group(GroupState & group_state, GroupConditions conditions, bool fresh,
                                     bool accepted) noexcept
{
    // Pausing or toggling enable/ready must not continually renew a failing session.
    auto const last_progress =
        group_state.last_fuse_time_us != 0U ? group_state.last_fuse_time_us : group_state.start_time_us;
    if (last_progress != 0U && m_update_time_us - last_progress > m_parameters.fusion_timeout_us)
    {
        group_state.recovery_required = true;
    }

    if (!conditions.enabled)
    {
        group_state.stop_reason = StopReason::disabled;
    }
    else if (!conditions.ready)
    {
        group_state.stop_reason = StopReason::not_ready;
    }
    else if (group_state.recovery_required)
    {
        group_state.stop_reason = StopReason::fusion_timeout;
    }
    else if (!fresh)
    {
        group_state.stop_reason = StopReason::data_timeout;
    }
    else if (!group_state.active && !accepted)
    {
        group_state.stop_reason = StopReason::waiting_for_quality;
    }
    else
    {
        group_state.stop_reason = StopReason::none;
        if (group_state.start_time_us == 0U)
        {
            group_state.start_time_us = m_update_time_us;
        }
        group_state.active = true;
        group_state.fusion_allowed = accepted;
        return;
    }
    group_state.active = false;
}

inline Status GnssFusion::update(std::uint64_t now_us, Conditions const & conditions, Parameters const & parameters,
                                 Sample const & sample) noexcept
{
    // Never leave an old authorization usable after a failed or sample-less tick.
    m_state.horizontal_position.fusion_allowed = false;
    m_state.vertical_position.fusion_allowed = false;
    m_state.velocity.fusion_allowed = false;

    if (parameters.maximum_sample_age_us == 0U || parameters.fusion_timeout_us == 0U ||
        (m_update_time_us != 0U && (parameters.maximum_sample_age_us != m_parameters.maximum_sample_age_us ||
                                    parameters.fusion_timeout_us != m_parameters.fusion_timeout_us)))
    {
        return Status::domain_error;
    }
    if (now_us <= m_update_time_us ||
        (sample.time_us != 0U && (sample.time_us <= m_sample_time_us || sample.time_us > now_us)))
    {
        return Status::out_of_range;
    }
    if (sample.time_us != 0U && sample.quality != GnssQualityDecision::accepted &&
        sample.quality != GnssQualityDecision::waiting && sample.quality != GnssQualityDecision::rejected)
    {
        return Status::domain_error;
    }

    m_update_time_us = now_us;
    m_parameters = parameters;
    if (sample.time_us != 0U)
    {
        m_sample_time_us = sample.time_us;
    }
    bool const fresh = m_sample_time_us != 0U && now_us - m_sample_time_us <= parameters.maximum_sample_age_us;
    bool const accepted = sample.time_us != 0U && sample.quality == GnssQualityDecision::accepted;
    update_group(m_state.horizontal_position, conditions.horizontal_position, fresh, accepted);
    update_group(m_state.vertical_position, conditions.vertical_position, fresh, accepted);
    update_group(m_state.velocity, conditions.velocity, fresh, accepted);
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
