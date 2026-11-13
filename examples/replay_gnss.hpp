/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <string_view>

#include <formal_eskf/runtime/gnss_fusion.hpp>

namespace formal_eskf::example
{

/** Convert the adapter's validated ten-bit selection, not a core configuration mask. */
inline runtime::GnssQualityChecks gnss_quality_checks(unsigned mask) noexcept
{
    return {.satellites = (mask & 1U) != 0U,
            .pdop = (mask & 2U) != 0U,
            .horizontal_accuracy = (mask & 4U) != 0U,
            .vertical_accuracy = (mask & 8U) != 0U,
            .speed_accuracy = (mask & 16U) != 0U,
            .horizontal_drift = (mask & 32U) != 0U,
            .vertical_drift = (mask & 64U) != 0U,
            .horizontal_speed = (mask & 128U) != 0U,
            .vertical_speed = (mask & 256U) != 0U,
            .spoofed = (mask & 512U) != 0U};
}

/** Replay-only handling of unavailable metadata around the unchanged quality monitor. */
template <typename Linalg> class ReplayGnssQuality
{
public:
    using monitor_type = runtime::GnssQuality<Linalg>;

    enum class Outcome
    {
        unavailable,
        invalid_velocity,
        invalid_call,
        rejected,
        waiting,
        accepted,
    };

    struct Metadata
    {
        bool velocity_valid = false;
        int in_air = -1;
        int at_rest = -1;
        int spoofed = -1; // -1 is absent metadata, not a receiver's reported UNKNOWN state.
    };

    struct Result
    {
        Outcome outcome = Outcome::unavailable;
        bool motion_available = false;
        bool spoofing_available = false;
        typename monitor_type::result_type quality{};

        [[nodiscard]] bool evaluated() const noexcept
        {
            return quality.decision != runtime::GnssQualityDecision::invalid_call;
        }
        [[nodiscard]] bool accepted() const noexcept
        {
            return quality.decision == runtime::GnssQualityDecision::accepted;
        }
        [[nodiscard]] std::string_view outcome_name() const noexcept;

        [[nodiscard]] runtime::GnssFusion::Sample lifecycle_sample(std::uint64_t time_us) const noexcept
        {
            // Invalid velocity never entered the receiver buffer. Unavailable
            // quality metadata is a received sample, but cannot refresh health.
            return {outcome == Outcome::invalid_velocity ? 0U : time_us, quality.decision, evaluated()};
        }
    };

    void restart_health() noexcept { m_monitor.restart_health(); }

    [[nodiscard]] Result update(typename monitor_type::sample_type sample, Metadata const & metadata,
                                std::uint64_t now_us,
                                typename monitor_type::parameters_type const & parameters) noexcept;

private:
    monitor_type m_monitor;
}; /* end class ReplayGnssQuality */

template <typename Linalg>
typename ReplayGnssQuality<Linalg>::Result
ReplayGnssQuality<Linalg>::update(typename monitor_type::sample_type sample, Metadata const & metadata,
                                  std::uint64_t now_us,
                                  typename monitor_type::parameters_type const & parameters) noexcept
{
    Result result;
    result.motion_available = metadata.in_air == 1 || (metadata.in_air == 0 && metadata.at_rest >= 0);
    result.spoofing_available = metadata.spoofed >= 0;
    if (metadata.in_air < -1 || metadata.in_air > 1 || metadata.at_rest < -1 || metadata.at_rest > 1 ||
        metadata.spoofed < -1 || metadata.spoofed > 1)
    {
        result.outcome = Outcome::invalid_call;
        result.quality.status = Status::domain_error;
        return result;
    }
    if (!metadata.velocity_valid)
    {
        // The source adapter drops these before quality evaluation; never
        // fabricate a bad fix or successful health update for this message.
        result.outcome = Outcome::invalid_velocity;
        return result;
    }
    auto const & checks = parameters.enabled_checks;
    bool const motion_required =
        checks.horizontal_drift || checks.vertical_drift || checks.horizontal_speed || checks.vertical_speed;
    if ((motion_required && !result.motion_available) || (checks.spoofed && !result.spoofing_available))
    {
        // An incomplete legacy log cannot establish uninterrupted health.
        // This is an explicit replay limitation, not an EKF2 transition.
        m_monitor.reset();
        return result;
    }
    sample.spoofed = metadata.spoofed == 1;
    result.quality = m_monitor.update(sample, {metadata.in_air == 1, metadata.in_air == 0 && metadata.at_rest == 1},
                                      now_us, parameters);
    switch (result.quality.decision)
    {
    case runtime::GnssQualityDecision::accepted:
        result.outcome = Outcome::accepted;
        break;
    case runtime::GnssQualityDecision::waiting:
        result.outcome = Outcome::waiting;
        break;
    case runtime::GnssQualityDecision::rejected:
        result.outcome = Outcome::rejected;
        break;
    case runtime::GnssQualityDecision::invalid_call:
        result.outcome = Outcome::invalid_call;
        break;
    }
    return result;
}

template <typename Linalg> std::string_view ReplayGnssQuality<Linalg>::Result::outcome_name() const noexcept
{
    switch (outcome)
    {
    case Outcome::unavailable:
        return "unavailable";
    case Outcome::invalid_velocity:
        return "invalid_velocity";
    case Outcome::invalid_call:
        return "invalid_call";
    case Outcome::rejected:
        return "rejected";
    case Outcome::waiting:
        return "waiting";
    case Outcome::accepted:
        return "accepted";
    }
    return "invalid_call";
}

} /* end namespace formal_eskf::example */
