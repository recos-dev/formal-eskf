// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cmath>
#include <optional>

#include <formal_eskf/runtime/magnetic_reference.hpp>

#include "DeploymentProfile.hpp"

namespace formal_eskf::px4
{

/** Geographic health reference and independently selected observation reference. */
template <typename Linalg> class MagneticModel
{
public:
    using value_type = typename Linalg::value_type;
    using vector_type = linalg::Matrix<Linalg, 3U, 1U>;
    using model_type = runtime::MagneticReference<Linalg>;
    using sample_type = typename model_type::Sample;
    using result_type = typename model_type::Result;

    enum class HeadingSource
    {
        magnetic_north,
        saved,
        geographic
    };

    struct Settings
    {
        std::uint32_t declination_source = 3U; // Bit 0: geographic; bit 1: saved fallback.
        value_type saved_declination{};        // East-positive radians, [-pi, pi].
    };

    /** Seed after tilt alignment, before yaw alignment; level_field is in microtesla. */
    [[nodiscard]] Status initialize(std::uint64_t now_us, Settings const & settings, vector_type const & level_field,
                                    std::optional<sample_type> const & model);

    /** The adapter calls the host lookup only when this predicate is true. */
    bool lookup_due(std::uint64_t now_us, bool position_valid, bool origin_updated = false) const
    {
        return now_us > m_last_lookup_time &&
               (origin_updated ||
                (position_valid && now_us - m_last_lookup_time > profile::magnetic::model_interval_us));
    }

    /** Missing/invalid candidates retain the adopted model; attempts are rate-limited too. */
    [[nodiscard]] result_type update(std::uint64_t now_us, std::optional<sample_type> const & model,
                                     bool position_valid, bool origin_updated = false);

    vector_type const & observation_n() const { return m_observation; }
    vector_type const & health_n() const { return m_model.state().field_n; }
    bool available() const { return m_model.state().reference_id != 0U; }
    std::uint32_t reference_id() const { return m_reference_id; }
    std::uint32_t model_revision() const { return m_model.state().reference_id; }
    std::uint64_t lookup_count() const { return m_lookup_count; }
    std::uint64_t last_lookup_time() const { return m_last_lookup_time; }
    HeadingSource heading_source() const { return m_heading_source; }
    value_type declination() const { return m_declination; }

    /** Express a local geographic NED model in the module's fixed origin NED. */
    [[nodiscard]] static Status to_origin_frame(sample_type const & local, double latitude_deg, double longitude_deg,
                                                double origin_latitude_deg, double origin_longitude_deg,
                                                sample_type & output);

private:
    model_type m_model{};
    Settings m_settings{};
    vector_type m_observation{};
    value_type m_declination{};
    HeadingSource m_heading_source = HeadingSource::magnetic_north;
    std::uint32_t m_reference_id{};
    std::uint64_t m_last_lookup_time{}, m_lookup_count{};
}; /* end class MagneticModel */

template <typename Linalg>
Status MagneticModel<Linalg>::initialize(std::uint64_t now_us, Settings const & settings,
                                         vector_type const & level_field, std::optional<sample_type> const & model)
{
    using math_type = typename Linalg::scalar_math_type;
    auto const pi = scalar::pi<math_type>();
    if (now_us == 0U || settings.declination_source > 3U || !std::isfinite(settings.saved_declination) ||
        std::abs(settings.saved_declination) > pi || !linalg::all_finite(level_field))
    {
        return Status::domain_error;
    }
    auto const horizontal = std::hypot(level_field(0U), level_field(1U));
    if (!(horizontal > value_type{0}) || !std::isfinite(horizontal))
    {
        return Status::domain_error;
    }
    MagneticModel candidate;
    candidate.m_settings = settings;
    if (model)
    {
        auto const result = candidate.m_model.update(now_us, *model, true, true);
        if (!succeeded(result.status))
        {
            return result.status;
        }
    }
    if ((settings.declination_source & 1U) != 0U && candidate.available())
    {
        candidate.m_declination = candidate.m_model.state().model.declination;
        candidate.m_heading_source = HeadingSource::geographic;
        candidate.m_observation = candidate.health_n();
    }
    else
    {
        if ((settings.declination_source & 2U) != 0U && settings.saved_declination != value_type{0})
        {
            candidate.m_declination = settings.saved_declination;
            candidate.m_heading_source = HeadingSource::saved;
        }
        // Equivalent to rotating the seed by its selected yaw, without making
        // its measured strength/inclination an independent health reference.
        candidate.m_observation =
            vector_type::from_row_major({horizontal * std::cos(candidate.m_declination),
                                         horizontal * std::sin(candidate.m_declination), level_field(2U)});
    }
    candidate.m_reference_id = 1U;
    candidate.m_last_lookup_time = now_us;
    candidate.m_lookup_count = 1U;
    *this = candidate;
    return Status::success;
}

template <typename Linalg>
typename MagneticModel<Linalg>::result_type MagneticModel<Linalg>::update(std::uint64_t now_us,
                                                                          std::optional<sample_type> const & model,
                                                                          bool position_valid, bool origin_updated)
{
    result_type result;
    if (m_reference_id == 0U || now_us <= m_last_lookup_time)
    {
        result.status = Status::out_of_range;
        return result;
    }
    if (!lookup_due(now_us, position_valid, origin_updated))
    {
        return result;
    }
    m_last_lookup_time = now_us;
    ++m_lookup_count;
    result.checked = true;
    if (!model)
    {
        return result;
    }
    auto candidate = m_model;
    result = candidate.update(now_us, *model, position_valid, origin_updated);
    if (!succeeded(result.status))
    {
        return result;
    }
    if (result.changed && (m_settings.declination_source & 1U) != 0U)
    {
        if (m_reference_id == std::numeric_limits<std::uint32_t>::max())
        {
            result.status = Status::out_of_range;
            result.changed = false;
            return result;
        }
        m_observation = candidate.state().field_n;
        m_declination = candidate.state().model.declination;
        m_heading_source = HeadingSource::geographic;
        ++m_reference_id;
    }
    m_model = candidate;
    return result;
}

template <typename Linalg>
Status MagneticModel<Linalg>::to_origin_frame(sample_type const & local, double latitude_deg, double longitude_deg,
                                              double origin_latitude_deg, double origin_longitude_deg,
                                              sample_type & output)
{
    auto valid = [](double latitude, double longitude)
    {
        return std::isfinite(latitude) && std::isfinite(longitude) && std::abs(latitude) <= 90. &&
               std::abs(longitude) <= 180.;
    };
    if (!valid(latitude_deg, longitude_deg) || !valid(origin_latitude_deg, origin_longitude_deg))
    {
        return Status::domain_error;
    }
    model_type candidate;
    auto const result = candidate.update(1U, local, true);
    if (!succeeded(result.status))
    {
        return result.status;
    }
    auto basis = [](double latitude, double longitude)
    {
        auto const radians = scalar::pi<typename Linalg::scalar_math_type>() / value_type{180};
        auto const lat = static_cast<value_type>(latitude) * radians;
        auto const lon = static_cast<value_type>(longitude) * radians;
        auto const slat = std::sin(lat), clat = std::cos(lat), slon = std::sin(lon), clon = std::cos(lon);
        return linalg::Matrix<Linalg, 3U, 3U>::from_row_major(
            {-slat * clon, -slat * slon, clat, -slon, clon, value_type{0}, -clat * clon, -clat * slon, -slat});
    };
    auto const field = basis(origin_latitude_deg, origin_longitude_deg) *
                       linalg::transpose(basis(latitude_deg, longitude_deg)) * candidate.state().field_n;
    auto const horizontal = std::hypot(field(0U), field(1U));
    sample_type const converted{std::atan2(field(1U), field(0U)), std::atan2(field(2U), horizontal),
                                std::hypot(horizontal, field(2U))};
    if (!linalg::all_finite(field) || !std::isfinite(converted.strength))
    {
        return Status::non_finite_result;
    }
    output = converted;
    return Status::success;
}

} /* end namespace formal_eskf::px4 */
