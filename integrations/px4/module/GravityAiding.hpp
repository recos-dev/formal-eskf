// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <formal_eskf/runtime/accelerometer.hpp>
#include <formal_eskf/runtime/accelerometer_sample.hpp>
#include <formal_eskf/runtime/tilt_alignment.hpp>

#include "BiasLimits.hpp"

namespace formal_eskf::px4
{

/** Current-horizon gravity adapter; prediction holds are never observations. */
template <typename Linalg> class GravityAiding
{
public:
    using value_type = typename Linalg::value_type;
    using state_type = configuration::Ins::NominalState<Linalg>;
    using covariance_type = linalg::Matrix<Linalg, 15U, 15U>;
    using vector_type = linalg::Vector<Linalg, 3U>;
    using sampler_type = runtime::AccelerometerSample<Linalg, 10000U>;

    struct Context
    {
        bool enabled = false;
        bool at_rest = false; // Unknown/stale rest disables only the override.
        bool horizontal_aiding = false;
    };

    struct Result
    {
        Status status = Status::success;
        bool ready = false;
        std::uint64_t interval_us{};
        runtime::AccelerometerFusionResult fusion{};
    };

    /** A discontinuity discards partial increments and LPF history, not established tilt alignment. */
    void reset_samples() noexcept
    {
        m_sampler.reset();
        m_active = false;
    }
    [[nodiscard]] bool active() const noexcept { return m_active; }
    [[nodiscard]] bool tilt_aligned() const noexcept { return m_tilt.aligned(); }

    /** Tilt/sampler history is consumed before correction; a later failure does not roll it back. */
    [[nodiscard]] Result update(ImuSample<Linalg> const & imu, std::uint64_t angular_period_us,
                                std::uint64_t acceleration_period_us, int clipping_mask, Context const & context,
                                vector_type const & gravity_n, state_type & state, covariance_type & covariance,
                                configuration::Ins::BiasUpdate const & bias_update = {}) noexcept;

private:
    [[nodiscard]] runtime::AccelerometerFusionResult
    fuse_observation(typename sampler_type::Result const & sample, Context const & context,
                     vector_type const & gravity_n, state_type & state, covariance_type & covariance,
                     configuration::Ins::BiasUpdate const & bias_update) noexcept;

    static constexpr auto minimum_norm = static_cast<value_type>(1e-6);
    sampler_type m_sampler{};
    runtime::TiltAlignment<Linalg> m_tilt{};
    bool m_active = false;
}; /* end class GravityAiding */

template <typename Linalg>
typename GravityAiding<Linalg>::Result
GravityAiding<Linalg>::update(ImuSample<Linalg> const & imu, std::uint64_t angular_period_us,
                              std::uint64_t acceleration_period_us, int clipping_mask, Context const & context,
                              vector_type const & gravity_n, state_type & state, covariance_type & covariance,
                              configuration::Ins::BiasUpdate const & bias_update) noexcept
{
    Result result;
    if (angular_period_us == 0U || angular_period_us > 50000U || acceleration_period_us == 0U ||
        acceleration_period_us > 50000U)
    {
        result.status = Status::domain_error;
        return result;
    }
    result.status = m_tilt.update(state.q_nb, covariance.template block<6U, 6U, 3U, 3U>());
    if (!succeeded(result.status))
    {
        return result;
    }
    // The shared sampler uses one period. Preserve the measured angle integral
    // when gyro and accelerometer integration durations differ.
    auto interval_imu = imu;
    interval_imu.angular_rate_b = imu.angular_rate_b * (static_cast<value_type>(angular_period_us) /
                                                        static_cast<value_type>(acceleration_period_us));
    auto const sample = m_sampler.update(interval_imu, acceleration_period_us, clipping_mask, minimum_norm);
    result.status = sample.status;
    if (!succeeded(result.status))
    {
        return result;
    }
    result.ready = sample.ready;
    result.interval_us = sample.interval_us;
    if (!context.enabled || context.horizontal_aiding || !m_tilt.aligned())
    {
        m_active = false;
    }
    if (!sample.ready)
    {
        return result;
    }
    result.fusion = fuse_observation(sample, context, gravity_n, state, covariance, bias_update);
    result.status = result.fusion.status;
    if (result.fusion.decision == runtime::FusionDecision::fused)
    {
        result.status = try_limit_imu_bias(state);
    }
    return result;
}

template <typename Linalg>
runtime::AccelerometerFusionResult
GravityAiding<Linalg>::fuse_observation(typename sampler_type::Result const & sample, Context const & context,
                                        vector_type const & gravity_n, state_type & state, covariance_type & covariance,
                                        configuration::Ins::BiasUpdate const & bias_update) noexcept
{
    runtime::AccelerometerFusionResult result;
    if (sample.clipping_mask < 0)
    {
        m_active = false;
        result.decision = runtime::FusionDecision::rejected;
        return result;
    }
    runtime::AccelerometerFusionConditions const conditions{
        context.enabled,
        m_tilt.aligned(),
        context.horizontal_aiding,
        context.at_rest,
        {(sample.clipping_mask & 1) != 0, (sample.clipping_mask & 2) != 0, (sample.clipping_mask & 4) != 0}};
    // Existing unnormalized replay weight, not sensor noise or an innovation gate.
    auto const V = linalg::Matrix<Linalg, 3U, 3U>::identity() * linalg::dot(gravity_n, gravity_n);
    result = runtime::fuse_accelerometer(state, covariance, sample.specific_force_b, gravity_n, V, conditions,
                                         sample.magnitude, minimum_norm, state, covariance, bias_update);
    m_active = result.conditions.active;
    return result;
}

} /* end namespace formal_eskf::px4 */
