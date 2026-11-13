/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <formal_eskf/scalar/math.hpp>

namespace formal_eskf::runtime
{

/** Detect apparent inertial falling from vertical aiding and IMU clipping. */
template <typename Linalg> class VerticalAccelerationHealth
{
public:
    using value_type = typename Linalg::value_type;

    enum class Likelihood
    {
        low,
        medium,
        high,
    };

    struct Innovation
    {
        bool active = false;     // Aiding lifecycle is active; rejection alone does not deactivate it.
        value_type innovation{}; // Prediction minus observation, with positive meaning apparent falling.
        value_type variance{};   // Innovation variance S, not measurement noise V.
    };

    struct Observations
    {
        Innovation barometer_height{}; // Pressure reference.
        Innovation gnss_height{};      // Both GNSS observations share one reference type.
        Innovation gnss_velocity_down{};
        Innovation range_height{}; // Ground reference; negate the usual distance-to-ground innovation.
        Innovation vision_height{};
        Innovation vision_velocity_down{};
    };

    struct Sample
    {
        std::uint64_t time_us{};         // Increasing estimator-horizon time, not observation arrival time.
        value_type average_interval_s{}; // Caller-supplied average EKF interval, not a GNSS interval.
        bool clipping_available = false;
        std::array<bool, 3U> clipped{}; // Body axes; accumulated over this IMU interval.
    };

    struct State
    {
        Likelihood likelihood = Likelihood::low;
        std::array<std::uint16_t, 3U> clip_count{};
        bool bad_clipping = false;
        bool bad_sample = false;                // Instantaneous condition, before the hold below.
        bool bad_vertical_acceleration = false; // Held until 3 s since the last bad sample.
        std::uint64_t last_bad_time_us{};
        std::uint64_t last_good_time_us{};
        std::uint64_t last_update_time_us{}; // Zero means no successful evaluation yet.
    };

    /** Also required when restarting after missing IMU data or changing source/calibration/frame. */
    void reset() { m_state = {}; }
    State const & state() const { return m_state; }

    /**
     * Evaluate once per actual estimator IMU update. The caller owns source
     * activity, cached innovations and their update order. Our core residuals
     * use observation minus prediction: negate them before passing them here.
     * Unknown clipping returns domain_error; it must not be replaced by clear.
     * Failure preserves history, which is then NOT a verdict for this sample.
     * This monitor does not change ESKF state, noise, fusion gates or bias learning.
     */
    [[nodiscard]] Status update(Sample const & sample, Observations const & observations);

private:
    [[nodiscard]] static Status falling_likelihood(Observations const & observations, Likelihood & output);
    /** Advance clipping history and return the evidence used by medium likelihood. */
    [[nodiscard]] static bool update_clipping(std::array<bool, 3U> const & clipped, std::uint16_t limit,
                                              State & candidate);

    State m_state{};
}; /* end class VerticalAccelerationHealth */

template <typename Linalg>
Status VerticalAccelerationHealth<Linalg>::falling_likelihood(Observations const & observations, Likelihood & output)
{
    using math_type = typename Linalg::scalar_math_type;
    struct Evidence
    {
        bool above_minimum = false;
        bool above_limit = false;
    };
    Evidence pressure, gnss, ground;
    struct Check
    {
        Innovation const & observation;
        Evidence & reference;
    };
    // Pair each source with its reference directly; this order also determines
    // the first reported failure. Validate every active source before deciding.
    std::array<Check, 6U> const checks{{{observations.barometer_height, pressure},
                                        {observations.gnss_height, gnss},
                                        {observations.gnss_velocity_down, gnss},
                                        {observations.range_height, ground},
                                        {observations.vision_height, ground},
                                        {observations.vision_velocity_down, ground}}};
    for (auto const & check : checks)
    {
        auto const & input = check.observation;
        if (!input.active)
        {
            continue;
        }
        if (!scalar::is_finite<math_type>(input.innovation) || !scalar::is_finite<math_type>(input.variance))
        {
            return Status::non_finite_input;
        }
        if (input.variance < value_type{0})
        {
            return Status::domain_error;
        }
        // Reference compatibility floor is FLT_EPSILON, including for double.
        if (input.variance < static_cast<value_type>(std::numeric_limits<float>::epsilon()))
        {
            continue;
        }
        value_type deviation{};
        auto const status = scalar::sqrt<math_type>(input.variance, deviation);
        if (!succeeded(status))
        {
            return status;
        }
        auto const ratio = input.innovation / deviation;
        if (!scalar::is_finite<math_type>(ratio))
        {
            return Status::non_finite_result;
        }
        check.reference.above_minimum = check.reference.above_minimum || ratio > value_type{1};
        check.reference.above_limit = check.reference.above_limit || ratio > value_type{3};
    }

    if ((pressure.above_limit && (gnss.above_minimum || ground.above_minimum)) ||
        (gnss.above_limit && (pressure.above_minimum || ground.above_minimum)) ||
        (ground.above_limit && (pressure.above_minimum || gnss.above_minimum)))
    {
        output = Likelihood::high;
    }
    else if (pressure.above_limit || gnss.above_limit || ground.above_limit)
    {
        output = Likelihood::medium;
    }
    else
    {
        output = Likelihood::low;
    }
    return Status::success;
}

template <typename Linalg>
bool VerticalAccelerationHealth<Linalg>::update_clipping(std::array<bool, 3U> const & clipped, std::uint16_t limit,
                                                         State & candidate)
{
    std::array<bool, 3U> warning{};
    bool critical = false;
    for (std::size_t axis = 0U; axis < candidate.clip_count.size(); ++axis)
    {
        auto & count = candidate.clip_count[axis];
        if (clipped[axis] && count < limit)
        {
            ++count;
        }
        else if (count > 0U)
        {
            // Also decrements at the limit with clipping still present.
            --count;
        }
        warning[axis] = count >= limit / 2U;
        critical = critical || count >= limit;
    }
    candidate.bad_clipping = critical || (warning[0U] && warning[1U] && warning[2U]);
    return warning[2U] || candidate.bad_clipping;
}

template <typename Linalg>
Status VerticalAccelerationHealth<Linalg>::update(Sample const & sample, Observations const & observations)
{
    using math_type = typename Linalg::scalar_math_type;
    if (sample.time_us == 0U || sample.time_us <= m_state.last_update_time_us)
    {
        return Status::out_of_range;
    }
    if (!scalar::is_finite<math_type>(sample.average_interval_s))
    {
        return Status::non_finite_input;
    }
    if (!(sample.average_interval_s > value_type{0}) || !sample.clipping_available)
    {
        return Status::domain_error;
    }
    auto const count_limit = value_type{1} / sample.average_interval_s;
    if (!scalar::is_finite<math_type>(count_limit))
    {
        return Status::non_finite_result;
    }
    // Keep the uint16 conversion defined, and the half-limit at least one.
    if (count_limit < value_type{2} || count_limit >= value_type{65536})
    {
        return Status::out_of_range;
    }
    auto const limit = static_cast<std::uint16_t>(count_limit);
    auto candidate = m_state;
    auto const status = falling_likelihood(observations, candidate.likelihood);
    if (!succeeded(status))
    {
        return status;
    }
    bool const frequent_clipping = update_clipping(sample.clipped, limit, candidate);
    candidate.bad_sample =
        candidate.likelihood == Likelihood::high || (candidate.likelihood == Likelihood::medium && frequent_clipping);
    if (candidate.bad_sample)
    {
        candidate.last_bad_time_us = sample.time_us;
    }
    else
    {
        candidate.last_good_time_us = sample.time_us;
    }
    candidate.bad_vertical_acceleration =
        candidate.last_bad_time_us != 0U && sample.time_us - candidate.last_bad_time_us < 3'000'000U;
    candidate.last_update_time_us = sample.time_us;
    m_state = candidate;
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
