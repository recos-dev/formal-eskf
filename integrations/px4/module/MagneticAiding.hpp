// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <algorithm>
#include <numbers>

#include <formal_eskf/runtime/magnetic_heading.hpp>
#include <formal_eskf/runtime/magnetometer.hpp>
#include <formal_eskf/runtime/magnetometer_filter.hpp>
#include <formal_eskf/runtime/magnetometer_fusion.hpp>
#include <formal_eskf/runtime/magnetometer_reset.hpp>

#include "BiasLimits.hpp"

namespace formal_eskf::px4
{

/** Fixed-field INS magnetic aiding for the deployment adapter. No clock or I/O. */
template <typename Linalg> class MagneticAiding
{
public:
    using value_type = typename Linalg::value_type;
    using vector_type = linalg::Matrix<Linalg, 3U, 1U>;
    using state_type = configuration::Ins::NominalState<Linalg>;
    using covariance_type = linalg::Matrix<Linalg, 15U, 15U>;
    using quaternion_type = so3::UnitQuaternion<Linalg>;
    using lifecycle_type = runtime::MagnetometerFusion;
    using source_type = lifecycle_type::Source;

    struct Sample
    {
        std::uint64_t time_us{};
        source_type source{};
        vector_type field_b{}; // Calibrated body FRD, microtesla.
    };

    struct Context
    {
        bool horizontal_aiding = false;
        bool at_rest = false;
        lifecycle_type::FlightState flight_state = lifecycle_type::FlightState::unknown;
    };

    struct Result
    {
        Status status = Status::success;
        bool fused = false;
        bool rejected = false;
        bool reset = false;
    };

    struct Reference
    {
        vector_type observation_n{};
        vector_type health_n{};
        bool model_available = false;
    };

    // Fixed deployment profile; runtime EKF2 parameter changes do not configure it.
    static constexpr runtime::MagneticFieldParameters<value_type> field_parameters{
        value_type{20}, std::numbers::pi_v<value_type> / value_type{9}, value_type{0}, value_type{5}};
    static constexpr runtime::MagneticFieldChecks field_checks{true, false, false};
    static constexpr typename runtime::MagneticHeading<Linalg>::Parameters heading_parameters{
        static_cast<value_type>(0.3), static_cast<value_type>(0.5), value_type{5}};
    static constexpr lifecycle_type::Parameters timing{1000000U, 500000U, 7000000U};
    static constexpr value_type noise_variance = value_type{25}; // (0.05 gauss)^2 in microtesla^2.
    static constexpr value_type gate_sigma = value_type{3};
    static constexpr value_type minimum_norm = static_cast<value_type>(1e-6);

    /** Bind the source of an actual successful initial alignment; invent no fusion time. */
    void initialize(source_type const & source, std::uint64_t sample_time_us) noexcept
    {
        *this = MagneticAiding{};
        m_source = source;
        m_sample_time = sample_time_us;
        m_lifecycle.reset(source);
        m_heading.reset_heading();
        m_aligned = true;
    }

    [[nodiscard]] Status update_acceleration(vector_type const & acceleration_n, value_type dt) noexcept
    {
        return m_heading.update_acceleration(acceleration_n, dt);
    }
    void reset_acceleration() noexcept { m_heading.reset_acceleration(); }
    [[nodiscard]] auto const & lifecycle() const noexcept { return m_lifecycle.state(); }
    [[nodiscard]] auto const & heading() const noexcept { return m_heading.state(); }
    [[nodiscard]] auto const & observation() const noexcept { return m_observation; }
    [[nodiscard]] bool aligned() const noexcept { return m_aligned; }
    [[nodiscard]] quaternion_type const & reset_delta() const noexcept { return m_reset_delta; }
    [[nodiscard]] std::uint8_t reset_counter() const noexcept { return m_reset_counter; }
    [[nodiscard]] std::uint64_t progress_time() const noexcept
    {
        return std::max(lifecycle().last_fuse_time_us, lifecycle().last_reset_time_us);
    }
    [[nodiscard]] bool heading_aiding() const noexcept
    {
        return lifecycle().active && m_aligned && m_observation.field.accepted &&
               (!m_horizontal_aiding || heading().consistent);
    }

    /** Every real estimator tick; a zero timestamp denotes no new observation. */
    [[nodiscard]] Result update(std::uint64_t now_us, Sample const & sample, Context const & context,
                                Reference const & reference, state_type & state, covariance_type & covariance,
                                configuration::Ins::BiasUpdate const & bias_update = {}) noexcept;

private:
    runtime::MagneticHeading<Linalg> m_heading{};
    runtime::MagnetometerFilter<Linalg> m_filter{};
    lifecycle_type m_lifecycle{};
    runtime::MagnetometerFusionResult<Linalg> m_observation{};
    source_type m_source{};
    quaternion_type m_reset_delta{};
    std::uint64_t m_sample_time{};
    std::uint8_t m_reset_counter{};
    bool m_aligned = false;
    bool m_horizontal_aiding = false;
}; /* end class MagneticAiding */

template <typename Linalg>
typename MagneticAiding<Linalg>::Result
MagneticAiding<Linalg>::update(std::uint64_t now_us, Sample const & sample, Context const & context,
                               Reference const & reference, state_type & state, covariance_type & covariance,
                               configuration::Ins::BiasUpdate const & bias_update) noexcept
{
    Result result;
    auto const previous = lifecycle();
    auto const * health_reference = reference.model_available ? &reference.health_n : nullptr;
    m_horizontal_aiding = context.horizontal_aiding;
    lifecycle_type::Sample event;
    bool usable = false;
    bool const received = sample.time_us > m_sample_time && sample.time_us <= now_us;
    bool const new_sample = received && now_us - sample.time_us < timing.maximum_interval_us;
    if (received)
    {
        // This adapter has no delayed fusion horizon. Old arrivals must not
        // refresh the lifecycle's data or successful-fusion timestamps.
        m_sample_time = sample.time_us;
        result.rejected = !new_sample;
    }
    if (new_sample)
    {
        m_observation = {};
        bool const source_changed = sample.source.device_id != m_source.device_id ||
                                    sample.source.calibration_count != m_source.calibration_count;
        if (source_changed && sample.source.device_id != 0U)
        {
            if (context.flight_state == lifecycle_type::FlightState::on_ground ||
                (previous.active && !context.horizontal_aiding))
            {
                m_aligned = false;
            }
        }
        if (previous.recovery_reason == lifecycle_type::RecoveryReason::source_changed &&
            context.flight_state == lifecycle_type::FlightState::on_ground)
        {
            m_aligned = false;
        }
        usable = sample.source.device_id != 0U && linalg::all_finite(sample.field_b) &&
                 linalg::dot(sample.field_b, sample.field_b) > value_type{0};
        if (sample.source.device_id != 0U)
        {
            m_source = sample.source;
            event = {sample.time_us, sample.source, false};
        }
        if (usable)
        {
            result.status = m_heading.update(state.q_nb, sample.field_b, reference.observation_n, heading_parameters,
                                             m_aligned, context.horizontal_aiding);
            if (!succeeded(result.status))
            {
                return result;
            }
            m_observation.field =
                runtime::check_magnetic_field(state.q_nb, sample.field_b, reference.observation_n, health_reference,
                                              field_parameters, false, field_checks);
            result.status = m_observation.field.status;
            if (!succeeded(result.status))
            {
                return result;
            }
            event.field_accepted = m_observation.field.accepted;
        }
        result.rejected = !usable || !event.field_accepted;
    }

    lifecycle_type::Conditions conditions;
    conditions.enabled = true;
    conditions.ready = !new_sample || usable;
    conditions.horizontal_aiding = context.horizontal_aiding;
    conditions.at_rest = context.at_rest;
    conditions.flight_state = context.flight_state;
    conditions.heading_consistent = heading().consistent;
    conditions.heading_aligned = m_aligned;
    result.status = m_lifecycle.update(now_us, conditions, timing, event);
    if (!succeeded(result.status))
    {
        return result;
    }
    if (lifecycle().filter_reset)
    {
        m_filter.reset();
    }
    if (previous.active && !lifecycle().active && !context.horizontal_aiding)
    {
        m_aligned = false;
    }
    if (new_sample && usable)
    {
        result.status = m_filter.update(sample.field_b);
        if (!succeeded(result.status))
        {
            return result;
        }
    }
    if (lifecycle().action == runtime::MagnetometerFusion::Action::fuse)
    {
        auto const V = linalg::Matrix<Linalg, 3U, 3U>::identity() * noise_variance;
        m_observation = runtime::fuse_magnetometer(state, covariance, sample.field_b, reference.observation_n,
                                                   health_reference, V, field_parameters, false, minimum_norm,
                                                   gate_sigma, state, covariance, bias_update, field_checks);
        result.status = m_lifecycle.record_fusion(event.time_us, m_observation.fusion.decision);
        if (!succeeded(result.status))
        {
            return result;
        }
        result.status = m_observation.fusion.status;
        result.fused = m_observation.fusion.decision == runtime::FusionDecision::fused;
        result.rejected = m_observation.fusion.decision == runtime::FusionDecision::rejected;
        if (!succeeded(result.status))
        {
            return result;
        }
        if (result.fused)
        {
            result.status = try_limit_imu_bias(state);
            if (!succeeded(result.status))
            {
                return result;
            }
        }
    }
    if (lifecycle().action == runtime::MagnetometerFusion::Action::reset)
    {
        quaternion_type delta;
        auto const reset_status = runtime::try_reset_magnetic_heading(
            state, covariance, m_filter.field_b(), reference.observation_n,
            heading_parameters.heading_noise * heading_parameters.heading_noise,
            field_parameters.minimum_horizontal_field, minimum_norm, state, covariance, delta);
        result.reset = succeeded(reset_status);
        result.status = m_lifecycle.record_reset(event.time_us, result.reset);
        if (!succeeded(result.status))
        {
            return result;
        }
        result.status = reset_status;
        if (result.reset)
        {
            m_reset_delta = delta;
            ++m_reset_counter;
            m_aligned = true;
            m_heading.reset_heading();
        }
    }
    if (previous.active && !lifecycle().active && !context.horizontal_aiding)
    {
        m_aligned = false;
    }
    return result;
}

} /* end namespace formal_eskf::px4 */
