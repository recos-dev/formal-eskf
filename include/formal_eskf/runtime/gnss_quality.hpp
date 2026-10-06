#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

#include <formal_eskf/linalg/operations.hpp>
#include <formal_eskf/scalar/math.hpp>
#include <formal_eskf/status.hpp>

namespace formal_eskf::runtime
{

/** Named check selections and failure diagnostics. */
struct GnssQualityChecks
{
    bool satellites = false;
    bool pdop = false;
    bool horizontal_accuracy = false;
    bool vertical_accuracy = false;
    bool speed_accuracy = false;
    bool horizontal_drift = false;
    bool vertical_drift = false;
    bool horizontal_speed = false;
    bool vertical_speed = false;
    bool spoofed = false;

    [[nodiscard]] bool any_enabled(GnssQualityChecks const & enabled) const noexcept
    {
        return (satellites && enabled.satellites) || (pdop && enabled.pdop) ||
               (horizontal_accuracy && enabled.horizontal_accuracy) ||
               (vertical_accuracy && enabled.vertical_accuracy) || (speed_accuracy && enabled.speed_accuracy) ||
               (horizontal_drift && enabled.horizontal_drift) || (vertical_drift && enabled.vertical_drift) ||
               (horizontal_speed && enabled.horizontal_speed) || (vertical_speed && enabled.vertical_speed) ||
               (spoofed && enabled.spoofed);
    }
};

enum class GnssQualityDecision
{
    invalid_call, // Invalid configuration/time/reference; monitor is unchanged.
    rejected,     // Consumed sample fails quality or numerical checks.
    waiting,      // Current checks pass, but health/recovery time has not elapsed.
    accepted,     // Current checks and both health/recovery conditions pass.
};

/** GNSS quality thresholds and health qualification interval. */
template <typename Scalar> struct GnssQualityParameters
{
    GnssQualityChecks enabled_checks{.satellites = true,
                                     .pdop = true,
                                     .horizontal_accuracy = true,
                                     .vertical_accuracy = true,
                                     .speed_accuracy = true,
                                     .horizontal_drift = true,
                                     .vertical_drift = true,
                                     .horizontal_speed = true,
                                     .vertical_speed = true,
                                     .spoofed = true};
    std::uint8_t minimum_satellites = 6U;
    Scalar maximum_pdop = Scalar{2.5};
    Scalar maximum_horizontal_accuracy = Scalar{3};             // m, one standard deviation
    Scalar maximum_vertical_accuracy = Scalar{5};               // m, one standard deviation
    Scalar maximum_speed_accuracy = Scalar{0.5};                // m/s, one standard deviation
    Scalar maximum_horizontal_drift = static_cast<Scalar>(0.1); // m/s; also stationary horizontal speed
    Scalar maximum_vertical_drift = static_cast<Scalar>(0.2);   // m/s; also stationary vertical speed
    Scalar velocity_limit = Scalar{100};                        // m/s, including in flight
    std::uint64_t minimum_health_time_us = 10000000U;
};

template <typename Linalg> struct GnssQualitySample
{
    using value_type = typename Linalg::value_type;
    std::uint64_t time_us{};
    std::uint8_t fix_type{};
    std::uint8_t satellites{};
    value_type pdop{};
    value_type horizontal_accuracy{};
    value_type vertical_accuracy{};
    value_type speed_accuracy{};
    bool spoofed = false;
    linalg::Matrix<Linalg, 3U, 1U> velocity_n{};
    // Current fix relative to the preceding consumed fix, metres N/E/down.
    // Project N/E about that fix; down = old MSL altitude - new MSL altitude.
    // Ignored on the first fix, after invalid data, and outside stationary checks.
    linalg::Matrix<Linalg, 3U, 1U> position_delta_n{};
    std::uint64_t position_reference_time_us{}; // Timestamp of that preceding fix.
};

struct GnssQualityMotion
{
    bool in_air = false;
    bool vehicle_at_rest = false; // Caller determines this from independent motion information.
};

template <typename Scalar> struct GnssQualityResult
{
    GnssQualityDecision decision = GnssQualityDecision::invalid_call;
    Status status = Status::success; // Numerical/caller error detail, not a fusion decision.
    GnssQualityChecks failures{};
    bool invalid_fix = false; // Mandatory; cannot be disabled.
    bool qualified = false;   // Historical initial qualification, retained after rejection.
    bool drift_valid = false; // False on ground while moving and in flight.
    Scalar horizontal_drift{};
    Scalar vertical_drift{};
    Scalar horizontal_speed{};
    Scalar vertical_speed{};
    std::uint64_t last_fail_us{};
    std::uint64_t last_pass_us{};
};

/**
 * Stateful, single-writer GNSS quality monitor with no clocks or platform dependencies.
 * Sample time drives drift filters; now_us is the caller's fusion time horizon.
 * Reset on source, projection continuity or parameter changes. Freshness, motion
 * detection, projection, source selection and actual fusion remain caller-owned.
 * See R-GNSS-QUALITY for input domains and behavior.
 */
template <typename Linalg> class GnssQuality
{
public:
    using value_type = typename Linalg::value_type;
    using parameters_type = GnssQualityParameters<value_type>;
    using sample_type = GnssQualitySample<Linalg>;
    using result_type = GnssQualityResult<value_type>;

    void reset() noexcept { *this = GnssQuality{}; }

    /**
     * invalid_call preserves the monitor; every other decision consumes sample.
     * Rejected numerical data reset filters and restart the health wait.
     * accepted is a quality decision only, not an alignment/innovation/fusion check.
     */
    [[nodiscard]] result_type update(sample_type const & sample, GnssQualityMotion motion, std::uint64_t now_us,
                                     parameters_type const & parameters) noexcept;

private:
    using math_type = typename Linalg::scalar_math_type;
    using vector_type = linalg::Matrix<Linalg, 3U, 1U>;

    [[nodiscard]] static Status validate_parameters(parameters_type const & parameters) noexcept;
    [[nodiscard]] Status validate_sample(sample_type const & sample, GnssQualityMotion motion) const noexcept;
    [[nodiscard]] result_type check_sample(sample_type const & sample, GnssQualityMotion motion,
                                           parameters_type const & parameters) noexcept;
    [[nodiscard]] Status update_motion_checks(sample_type const & sample, GnssQualityMotion motion,
                                              parameters_type const & parameters, result_type & result) noexcept;
    void update_health(std::uint64_t now_us, parameters_type const & parameters, result_type & result) noexcept;
    [[nodiscard]] static value_type constrain(value_type value, value_type lower, value_type upper) noexcept;
    void reset_filters() noexcept;

    vector_type m_position_rate{};
    vector_type m_velocity{};
    std::uint64_t m_sample_time_us{};
    std::uint64_t m_update_time_us{};
    std::uint64_t m_last_fail_us{};
    std::uint64_t m_last_pass_us{};
    GnssQualityChecks m_failures{};
    bool m_previous_position_valid = false;
    bool m_qualified = false;
}; /* end class GnssQuality */

template <typename Linalg>
typename GnssQuality<Linalg>::value_type GnssQuality<Linalg>::constrain(value_type value, value_type lower,
                                                                        value_type upper) noexcept
{
    return value < lower ? lower : (value > upper ? upper : value);
}

template <typename Linalg> void GnssQuality<Linalg>::reset_filters() noexcept
{
    m_position_rate = vector_type::zero();
    m_velocity = vector_type::zero();
}

template <typename Linalg> Status GnssQuality<Linalg>::validate_parameters(parameters_type const & parameters) noexcept
{
    value_type const bounds[]{
        parameters.maximum_pdop,           parameters.maximum_horizontal_accuracy, parameters.maximum_vertical_accuracy,
        parameters.maximum_speed_accuracy, parameters.maximum_horizontal_drift,    parameters.maximum_vertical_drift,
        parameters.velocity_limit};
    for (auto const bound : bounds)
    {
        if (!scalar::is_finite<math_type>(bound))
        {
            return Status::non_finite_input;
        }
        if (bound < std::numeric_limits<value_type>::min())
        {
            return Status::domain_error;
        }
    }
    // Only motion thresholds enter arithmetic; accuracy/PDOP limits are compared.
    auto const horizontal_clip = value_type{10} * parameters.maximum_horizontal_drift;
    auto const vertical_clip = value_type{10} * parameters.maximum_vertical_drift;
    auto const horizontal_squared = parameters.maximum_horizontal_drift * parameters.maximum_horizontal_drift;
    auto const velocity_squared = parameters.velocity_limit * parameters.velocity_limit;
    if (!scalar::is_finite<math_type>(value_type{2} * horizontal_clip * horizontal_clip) ||
        !scalar::is_finite<math_type>(vertical_clip) || !scalar::is_finite<math_type>(velocity_squared) ||
        horizontal_squared < std::numeric_limits<value_type>::min() ||
        velocity_squared < std::numeric_limits<value_type>::min() || parameters.minimum_health_time_us == 0U)
    {
        return Status::domain_error;
    }
    return Status::success;
}

template <typename Linalg>
Status GnssQuality<Linalg>::validate_sample(sample_type const & sample, GnssQualityMotion motion) const noexcept
{
    value_type const accuracies[]{sample.pdop, sample.horizontal_accuracy, sample.vertical_accuracy,
                                  sample.speed_accuracy};
    for (auto const accuracy : accuracies)
    {
        if (!scalar::is_finite<math_type>(accuracy))
        {
            return Status::non_finite_input;
        }
        if (accuracy < value_type{0})
        {
            return Status::domain_error;
        }
    }
    if (!linalg::all_finite(sample.velocity_n) ||
        (!motion.in_air && motion.vehicle_at_rest && m_previous_position_valid &&
         !linalg::all_finite(sample.position_delta_n)))
    {
        return Status::non_finite_input;
    }
    return Status::success;
}

template <typename Linalg>
Status GnssQuality<Linalg>::update_motion_checks(sample_type const & sample, GnssQualityMotion motion,
                                                 parameters_type const & parameters, result_type & result) noexcept
{
    if (!motion.in_air && motion.vehicle_at_rest)
    {
        auto const dt =
            constrain(static_cast<value_type>(sample.time_us - m_sample_time_us) * static_cast<value_type>(1e-6),
                      static_cast<value_type>(0.001), value_type{10});
        auto const alpha = dt / value_type{10};
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            auto const limit = axis == 2U ? parameters.maximum_vertical_drift : parameters.maximum_horizontal_drift;
            auto const delta = m_previous_position_valid ? sample.position_delta_n(axis) : value_type{0};
            auto const rate = delta / dt * alpha + m_position_rate(axis) * (value_type{1} - alpha);
            if (!scalar::is_finite<math_type>(rate))
            {
                return Status::non_finite_result;
            }
            // Constrain position-filter state, but velocity-filter input.
            m_position_rate.set(axis, constrain(rate, -value_type{10} * limit, value_type{10} * limit));
            auto const velocity = constrain(sample.velocity_n(axis), -value_type{10} * limit, value_type{10} * limit);
            m_velocity.set(axis, velocity * alpha + m_velocity(axis) * (value_type{1} - alpha));
        }
        result.horizontal_drift = linalg::norm(m_position_rate.template segment<0U, 2U>());
        result.vertical_drift = scalar::absolute<math_type>(m_position_rate(2U));
        result.horizontal_speed = linalg::norm(m_velocity.template segment<0U, 2U>());
        result.vertical_speed = scalar::absolute<math_type>(m_velocity(2U));
        if (!scalar::is_finite<math_type>(result.horizontal_drift) ||
            !scalar::is_finite<math_type>(result.vertical_drift) ||
            !scalar::is_finite<math_type>(result.horizontal_speed) ||
            !scalar::is_finite<math_type>(result.vertical_speed))
        {
            return Status::non_finite_result;
        }
        result.drift_valid = true;
        result.failures.horizontal_drift = result.horizontal_drift > parameters.maximum_horizontal_drift;
        result.failures.vertical_drift = result.vertical_drift > parameters.maximum_vertical_drift;
        result.failures.horizontal_speed = result.horizontal_speed > parameters.maximum_horizontal_drift;
        result.failures.vertical_speed = result.vertical_speed > parameters.maximum_vertical_drift;
    }
    else
    {
        reset_filters();
        if (motion.in_air)
        {
            result.failures.horizontal_drift = false;
            result.failures.vertical_drift = false;
            result.failures.horizontal_speed = false;
            result.failures.vertical_speed = false;
        }
        // Ground movement retains the previous motion-check failures.
    }

    // Raw speed limits also apply in flight, independently of drift checks.
    auto const horizontal_squared = linalg::squared_norm(sample.velocity_n.template segment<0U, 2U>());
    auto const limit_squared = parameters.velocity_limit * parameters.velocity_limit;
    if (!scalar::is_finite<math_type>(horizontal_squared) || !scalar::is_finite<math_type>(limit_squared))
    {
        return Status::non_finite_result;
    }
    if (horizontal_squared > limit_squared)
    {
        result.failures.horizontal_speed = true;
    }
    if (scalar::absolute<math_type>(sample.velocity_n(2U)) > parameters.velocity_limit)
    {
        result.failures.vertical_speed = true;
    }
    return Status::success;
}

template <typename Linalg>
typename GnssQuality<Linalg>::result_type GnssQuality<Linalg>::check_sample(sample_type const & sample,
                                                                            GnssQualityMotion motion,
                                                                            parameters_type const & parameters) noexcept
{
    result_type result;
    result.failures = m_failures;
    result.status = validate_sample(sample, motion);
    if (succeeded(result.status))
    {
        result.status = update_motion_checks(sample, motion, parameters, result);
    }
    m_previous_position_valid = succeeded(result.status);
    if (!m_previous_position_valid)
    {
        reset_filters();
        result = result_type{.status = result.status};
    }
    result.invalid_fix = sample.fix_type < 3U;
    result.failures.satellites = sample.satellites < parameters.minimum_satellites;
    result.failures.pdop = sample.pdop > parameters.maximum_pdop;
    result.failures.horizontal_accuracy = sample.horizontal_accuracy > parameters.maximum_horizontal_accuracy;
    result.failures.vertical_accuracy = sample.vertical_accuracy > parameters.maximum_vertical_accuracy;
    result.failures.speed_accuracy = sample.speed_accuracy > parameters.maximum_speed_accuracy;
    result.failures.spoofed = sample.spoofed;
    m_failures = result.failures;

    auto const rejected =
        !succeeded(result.status) || result.invalid_fix || result.failures.any_enabled(parameters.enabled_checks);
    result.decision = rejected ? GnssQualityDecision::rejected : GnssQualityDecision::waiting;
    return result;
}

template <typename Linalg>
void GnssQuality<Linalg>::update_health(std::uint64_t now_us, parameters_type const & parameters,
                                        result_type & result) noexcept
{
    if (m_sample_time_us == 0U || result.decision == GnssQualityDecision::rejected)
    {
        m_last_fail_us = now_us;
    }
    if (result.decision == GnssQualityDecision::waiting)
    {
        m_last_pass_us = now_us;
        auto const recovery_time = parameters.minimum_health_time_us / 10U;
        auto const elapsed = now_us - m_last_fail_us;
        auto const recovered = elapsed > (recovery_time > 1000000U ? recovery_time : 1000000U);
        if (recovered && elapsed > parameters.minimum_health_time_us)
        {
            m_qualified = true;
        }
        if (recovered && m_qualified)
        {
            result.decision = GnssQualityDecision::accepted;
        }
    }
    result.qualified = m_qualified;
    result.last_fail_us = m_last_fail_us;
    result.last_pass_us = m_last_pass_us;
}

template <typename Linalg>
typename GnssQuality<Linalg>::result_type GnssQuality<Linalg>::update(sample_type const & sample,
                                                                      GnssQualityMotion motion, std::uint64_t now_us,
                                                                      parameters_type const & parameters) noexcept
{
    auto const parameter_status = validate_parameters(parameters);
    if (!succeeded(parameter_status))
    {
        return result_type{.status = parameter_status};
    }
    if (sample.time_us == 0U || sample.time_us <= m_sample_time_us || now_us < sample.time_us ||
        now_us < m_update_time_us)
    {
        return result_type{.status = Status::out_of_range};
    }
    if (!motion.in_air && motion.vehicle_at_rest && m_previous_position_valid &&
        sample.position_reference_time_us != m_sample_time_us)
    {
        return result_type{.status = Status::out_of_range};
    }

    auto result = check_sample(sample, motion, parameters);
    update_health(now_us, parameters, result);
    m_sample_time_us = sample.time_us;
    m_update_time_us = now_us;
    return result;
}

} /* end namespace formal_eskf::runtime */
