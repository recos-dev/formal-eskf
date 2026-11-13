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

#include <formal_eskf/linalg/operations.hpp>
#include <formal_eskf/scalar/math.hpp>

namespace formal_eskf::runtime
{

/** Adopt externally supplied geographic field estimates in the estimator's NED frame. */
template <typename Linalg> class MagneticReference
{
public:
    using value_type = typename Linalg::value_type;
    using vector_type = linalg::Vector<Linalg, 3U>;

    struct Sample
    {
        value_type declination{}; // East-positive radians, [-pi, pi].
        value_type inclination{}; // Down-positive radians, [-pi/2, pi/2].
        value_type strength{};    // Positive normal microtesla.
    };

    struct State
    {
        Sample model{};
        vector_type field_n{};
        std::uint32_t reference_id{}; // Zero means no model has been adopted.
        std::uint64_t last_check_time_us{};
    };

    struct Result
    {
        Status status = Status::success;
        bool checked = false;
        bool changed = false;
    };

    [[nodiscard]] State const & state() const noexcept { return m_state; }
    void reset() noexcept { *this = MagneticReference{}; }

    /**
     * Call on increasing fusion timestamps. An origin update requests an
     * immediate check; otherwise valid position permits checks strictly more
     * than 10 s apart. The first valid position also permits initialization.
     * Adopt changes strictly greater than 1 degree or 1 microtesla, relative
     * to the last adopted model. A checked unchanged sample advances only the
     * check clock. Invalid calls preserve all history. No lookup, geodesy,
     * estimator reset or learned magnetic state is owned here.
     *
     * The caller supplies a trustworthy model at a valid geographic position,
     * already expressed in this estimator's NED frame. origin_updated is a
     * lookup trigger, not authorization to change the estimator coordinate frame.
     */
    [[nodiscard]] Result update(std::uint64_t now_us, Sample const & sample, bool position_valid,
                                bool origin_updated = false) noexcept;

private:
    [[nodiscard]] static Status validate_sample(Sample const & sample) noexcept;
    [[nodiscard]] Status adopt(Sample const & sample) noexcept;

    State m_state{};
    std::uint64_t m_update_time_us{};
}; /* end class MagneticReference */

template <typename Linalg>
typename MagneticReference<Linalg>::Result MagneticReference<Linalg>::update(std::uint64_t now_us,
                                                                             Sample const & sample, bool position_valid,
                                                                             bool origin_updated) noexcept
{
    using math_type = typename Linalg::scalar_math_type;
    Result result;
    if (now_us <= m_update_time_us)
    {
        result.status = Status::out_of_range;
        return result;
    }
    bool const initialized = m_state.reference_id != 0U;
    if (!origin_updated && (!position_valid || (initialized && now_us - m_state.last_check_time_us <= 10000000U)))
    {
        m_update_time_us = now_us;
        return result;
    }
    result.checked = true;
    result.status = validate_sample(sample);
    if (!succeeded(result.status))
    {
        return result;
    }
    auto const angle_threshold = scalar::pi<math_type>() / value_type{180};
    bool const changed =
        !initialized || scalar::absolute<math_type>(sample.declination - m_state.model.declination) > angle_threshold ||
        scalar::absolute<math_type>(sample.inclination - m_state.model.inclination) > angle_threshold ||
        scalar::absolute<math_type>(sample.strength - m_state.model.strength) > value_type{1};
    if (changed)
    {
        result.status = adopt(sample);
        if (!succeeded(result.status))
        {
            return result;
        }
        result.changed = true;
    }
    m_state.last_check_time_us = now_us;
    m_update_time_us = now_us;
    return result;
}

template <typename Linalg> Status MagneticReference<Linalg>::validate_sample(Sample const & sample) noexcept
{
    using math_type = typename Linalg::scalar_math_type;
    if (!scalar::is_finite<math_type>(sample.declination) || !scalar::is_finite<math_type>(sample.inclination) ||
        !scalar::is_finite<math_type>(sample.strength))
    {
        return Status::non_finite_input;
    }
    auto const pi = scalar::pi<math_type>();
    if (scalar::absolute<math_type>(sample.declination) > pi ||
        scalar::absolute<math_type>(sample.inclination) > pi / value_type{2} ||
        sample.strength < std::numeric_limits<value_type>::min())
    {
        return Status::domain_error;
    }
    return Status::success;
}

template <typename Linalg> Status MagneticReference<Linalg>::adopt(Sample const & sample) noexcept
{
    using math_type = typename Linalg::scalar_math_type;
    if (m_state.reference_id == std::numeric_limits<std::uint32_t>::max())
    {
        return Status::out_of_range;
    }
    scalar::SinCos<value_type> declination, inclination;
    auto status = scalar::try_sin_cos<math_type>(sample.declination, declination);
    if (!succeeded(status))
    {
        return status;
    }
    status = scalar::try_sin_cos<math_type>(sample.inclination, inclination);
    if (!succeeded(status))
    {
        return status;
    }
    vector_type field;
    field(0U) = sample.strength * inclination.cosine * declination.cosine;
    field(1U) = sample.strength * inclination.cosine * declination.sine;
    field(2U) = sample.strength * inclination.sine;
    if (!linalg::all_finite(field))
    {
        return Status::non_finite_result;
    }
    m_state.model = sample;
    m_state.field_n = field;
    ++m_state.reference_id;
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
