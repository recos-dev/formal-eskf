/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

#include <formal_eskf/eskf/eskf.hpp>
#include <formal_eskf/runtime/accelerometer.hpp>
#include <formal_eskf/runtime/gnss_reset.hpp>
#include <formal_eskf/runtime/magnetometer.hpp>
#include <formal_eskf/runtime/magnetometer_fusion.hpp>
#include <formal_eskf/runtime/magnetometer_reset.hpp>
#include <formal_eskf/runtime/position.hpp>
#include <formal_eskf/runtime/velocity.hpp>
#include "replay_accelerometer.hpp"
#include "replay_conditions.hpp"
#include "replay_gnss.hpp"
#if defined(FORMAL_ESKF_REPLAY_PX4_MATRIX)
#include <formal_eskf/linalg/backend/px4_matrix.hpp>
#else
#include <formal_eskf/linalg/backend/eigen.hpp>
#endif

// Replay settings, not calibrated deployment parameters or certified bounds.
#define GRAVITY 9.80665 // m/s^2, positive Down in NED.
#define QUAT_MIN_NORM 1e-6
#define QUAT_SQUARED_NORM_TOL 1e-12
#define DT_MIN 1e-6 // s.
#define DT_MAX 0.05 // s.

// Isotropic per-sample variances; the core scales these by dt^2.
#define ACCEL_VAR 1.0 // m^2/s^4.
#define GYRO_VAR 4e-4 // rad^2/s^2.
// Bias random-walk variance densities; the core scales these by dt.
#define ACCEL_BIAS_RW_VAR_DENSITY 1e-6 // m^2/s^5.
#define GYRO_BIAS_RW_VAR_DENSITY 1e-8  // rad^2/s^3.

// Gravity observation settings; independent of prediction's ACCEL_VAR.
#define ACCEL_OBS_STD GRAVITY      // m/s^2; effective pseudo-observation weight.
#define ACCEL_OBS_PERIOD_US 10000U // Nominal; accumulate whole IMU samples.

// Replay field checks; not calibrated disturbance-detection guarantees.
#define MAG_STRENGTH_TOL 20.0    // microtesla.
#define MAG_INCLINATION_TOL 20.0 // degrees.
#define MAG_MIN_HORIZONTAL 5.0   // microtesla; weak-heading guard, not an observability proof.
#define MAG_HEALTH_TIME_US 1000000U
#define MAG_MAX_INTERVAL_US 500000U // Startup freshness; no-data stop uses twice this.
#define MAG_FUSION_TIMEOUT_US 7000000U
#define MAG_HEADING_STD 0.3 // rad; fixed replay reset noise, not inferred from field noise.

// GNSS-only lifecycle profile: separate horizontal and height deadlines.
#define GNSS_HORIZONTAL_TIMEOUT_US 7000000U
#define GNSS_HEIGHT_DATA_TIMEOUT_US 1000000U
#define GNSS_HEIGHT_FUSION_TIMEOUT_US 5000000U

// Legacy/KITTI floors; PX4 replay supplies gnss_parameters.csv instead.
#define POS_XY_STD_MIN 1.0 // m.
#define POS_Z_STD_MIN 1.5  // m.
#define VEL_STD_MIN 0.3    // m/s.

// Initial local attitude-error and bias variances (already squared).
#define INIT_ATTITUDE_XY_VAR 0.04 // rad^2, body X/Y.
#define INIT_ATTITUDE_Z_VAR 0.25  // rad^2, body Z.
#define INIT_ACCEL_BIAS_VAR 0.25  // m^2/s^4.
#define INIT_GYRO_BIAS_VAR 0.0025 // rad^2/s^2.

namespace
{

#if defined(FORMAL_ESKF_REPLAY_PX4_MATRIX)
using Linalg = formal_eskf::linalg::Px4MatrixBackend<double>;
constexpr std::string_view backend_name = "px4_matrix";
#else
using Linalg = formal_eskf::linalg::EigenBackend<double>;
constexpr std::string_view backend_name = "eigen";
#endif
using Types = formal_eskf::EskfTypes<Linalg, formal_eskf::configuration::Ins>;
using State = Types::nominal_state_type;
using Covariance = Types::error_covariance_type;
using Vector3 = Linalg::vector_type<3U>;
using Imu = Types::imu_sample_type;
using formal_eskf::Status;
using FusionConditions = formal_eskf::example::ReplayFusionConditions<Linalg>;
using Accelerometer = formal_eskf::example::ReplayAccelerometer<Linalg, ACCEL_OBS_PERIOD_US>;
using MagneticLifecycle = formal_eskf::runtime::MagnetometerFusion;
using GnssQuality = formal_eskf::example::ReplayGnssQuality<Linalg>;
using GnssLifecycle = formal_eskf::runtime::GnssFusion;

std::ifstream open_csv(std::filesystem::path const & path, std::string_view expected_header)
{
    std::ifstream stream(path);
    std::string header;
    std::getline(stream, header);
    if (!header.empty() && header.back() == '\r')
    {
        header.pop_back();
    }
    if (!stream || header != expected_header)
    {
        throw std::runtime_error("cannot read expected CSV header: " + path.string());
    }
    return stream;
}

bool read_row(std::istream & stream, std::span<double> output, bool allow_non_finite = false)
{
    std::string line;
    if (!std::getline(stream, line))
    {
        if (!stream.eof())
        {
            throw std::runtime_error("CSV read failed");
        }
        return false;
    }
    if (!line.empty() && line.back() == '\r')
    {
        line.pop_back();
    }
    std::string_view remaining(line);
    for (std::size_t index = 0U; index < output.size(); ++index)
    {
        auto const comma = remaining.find(',');
        if ((comma == std::string_view::npos) != (index + 1U == output.size()))
        {
            throw std::runtime_error("wrong CSV column count");
        }
        auto const field = remaining.substr(0U, comma);
        auto const result = std::from_chars(field.data(), field.data() + field.size(), output[index]);
        if (result.ec != std::errc{} || result.ptr != field.data() + field.size() ||
            (!allow_non_finite && !std::isfinite(output[index])))
        {
            throw std::runtime_error("invalid CSV number");
        }
        if (comma != std::string_view::npos)
        {
            remaining.remove_prefix(comma + 1U);
        }
    }
    return true;
}

std::uint64_t timestamp(double value)
{
    // The CSV reader uses doubles; integers below 2^53 are represented exactly.
    if (!std::isfinite(value) || value < 1.0 || value >= 9007199254740992.0 || std::trunc(value) != value)
    {
        throw std::runtime_error("timestamp/interval must be a positive exact integer in microseconds");
    }
    return static_cast<std::uint64_t>(value);
}

void check_status(Status status, std::string_view operation, std::uint64_t time)
{
    if (!formal_eskf::succeeded(status))
    {
        throw std::runtime_error(std::string(operation) + " failed at " + std::to_string(time) +
                                 " us (Status=" + std::to_string(static_cast<int>(status)) + ")");
    }
}

Vector3 vector3(double x, double y, double z) { return Vector3::from_row_major({x, y, z}); }

int metadata_integer(double value, int minimum, int maximum)
{
    if (!std::isfinite(value) || std::trunc(value) != value || value < minimum || value > maximum)
    {
        throw std::runtime_error("invalid integer metadata");
    }
    return static_cast<int>(value);
}

Imu imu_sample(std::array<double, 8U> const & row)
{
    return {vector3(row[2U], row[3U], row[4U]), vector3(row[5U], row[6U], row[7U])};
}

std::array<int, 2U> read_imu_flags(std::ifstream & stream, std::uint64_t time)
{
    if (!stream.is_open())
    {
        return {-1, -1};
    }
    std::array<double, 3U> row{};
    if (!read_row(stream, row) || timestamp(row[0U]) != time || row[1U] < -1.0 || row[1U] > 1.0 ||
        std::trunc(row[1U]) != row[1U] || row[2U] < -1.0 || row[2U] > 7.0 || std::trunc(row[2U]) != row[2U])
    {
        throw std::runtime_error("IMU flags must match the sample timestamp and flag domains");
    }
    return {static_cast<int>(row[1U]), static_cast<int>(row[2U])};
}

double variance(double accuracy, double floor)
{
    if (!std::isfinite(accuracy) || !(accuracy > 0.0))
    {
        throw std::runtime_error("Navigation accuracy must be positive");
    }
    double const sigma = std::max(accuracy, floor);
    double const result = sigma * sigma;
    if (!std::isfinite(result) || result < std::numeric_limits<double>::min())
    {
        throw std::runtime_error("unsafe navigation variance");
    }
    return result;
}

struct GnssParameters
{
    double position_gate{};
    double velocity_gate{};
    double position_noise{};       // Standard deviation, m.
    double velocity_noise{};       // Standard deviation, m/s.
    double position_noise_limit{}; // Sole horizontal aiding source, m.
};

std::optional<GnssParameters> read_gnss_parameters(std::filesystem::path const & input)
{
    if (!std::filesystem::exists(input / "gnss_parameters.csv"))
    {
        return std::nullopt;
    }
    auto stream = open_csv(input / "gnss_parameters.csv",
                           "position_gate,velocity_gate,position_noise,velocity_noise,position_noise_limit");
    std::array<double, 5U> row{}, extra{};
    if (!read_row(stream, row) || read_row(stream, extra))
    {
        throw std::runtime_error("expected exactly one GNSS parameter row");
    }
    auto const & [position_gate, velocity_gate, position_noise, velocity_noise, position_noise_limit] = row;
    // A source-specific clamp belongs here, not in the generic innovation gate.
    GnssParameters result{std::max(position_gate, 1.0), std::max(velocity_gate, 1.0), position_noise, velocity_noise,
                          position_noise_limit};
    for (double value :
         {result.position_gate, result.velocity_gate, position_noise, velocity_noise, position_noise_limit})
    {
        (void)variance(value, 0.0);
    }
    return result;
}

std::optional<GnssQuality::monitor_type::parameters_type>
read_gnss_quality_parameters(std::filesystem::path const & input)
{
    bool const supplied = std::filesystem::exists(input / "gnss_quality_parameters.csv");
    if (supplied != std::filesystem::exists(input / "gnss_quality.csv") ||
        (supplied && !std::filesystem::exists(input / "gnss_parameters.csv")))
    {
        throw std::runtime_error("GNSS quality data/parameters must be paired with GNSS fusion parameters");
    }
    if (!supplied)
    {
        return std::nullopt;
    }
    auto stream =
        open_csv(input / "gnss_quality_parameters.csv",
                 "check_mask,minimum_satellites,maximum_pdop,maximum_horizontal_accuracy,maximum_vertical_accuracy,"
                 "maximum_speed_accuracy,maximum_horizontal_drift,maximum_vertical_drift,minimum_health_time_us");
    std::array<double, 9U> row{}, extra{};
    if (!read_row(stream, row) || read_row(stream, extra))
    {
        throw std::runtime_error("expected exactly one GNSS quality parameter row");
    }
    GnssQuality::monitor_type::parameters_type result;
    result.enabled_checks =
        formal_eskf::example::gnss_quality_checks(static_cast<unsigned>(metadata_integer(row[0U], 0, 1023)));
    result.minimum_satellites = static_cast<std::uint8_t>(metadata_integer(row[1U], 1, 255));
    result.maximum_pdop = row[2U];
    result.maximum_horizontal_accuracy = row[3U];
    result.maximum_vertical_accuracy = row[4U];
    result.maximum_speed_accuracy = row[5U];
    result.maximum_horizontal_drift = row[6U];
    result.maximum_vertical_drift = row[7U];
    result.minimum_health_time_us = timestamp(row[8U]);
    // Validate through the production monitor even if every replay row later has
    // unavailable metadata. This temporary monitor is not replay health history.
    GnssQuality::monitor_type validator;
    GnssQuality::monitor_type::sample_type sample;
    sample.time_us = 1U;
    check_status(validator.update(sample, {true, false}, 1U, result).status, "GNSS quality parameters", 1U);
    return result;
}

GnssQuality::Result check_gnss_quality(std::ifstream & stream, GnssQuality & monitor,
                                       GnssQuality::monitor_type::parameters_type const & parameters,
                                       std::array<double, 10U> const & navigation, std::uint64_t time)
{
    std::array<double, 15U> row{};
    // Numerical sensor errors reach the monitor; structural metadata remains strict.
    if (!read_row(stream, row, true) || timestamp(row[0U]) != timestamp(navigation[0U]))
    {
        throw std::runtime_error("GNSS quality rows must match navigation timestamps");
    }
    GnssQuality::monitor_type::sample_type sample;
    sample.time_us = timestamp(row[0U]);
    sample.fix_type = static_cast<std::uint8_t>(metadata_integer(row[1U], 0, 255));
    sample.satellites = static_cast<std::uint8_t>(metadata_integer(row[2U], 0, 255));
    sample.pdop = row[3U];
    sample.horizontal_accuracy = navigation[7U];
    sample.vertical_accuracy = navigation[8U];
    sample.speed_accuracy = navigation[9U];
    sample.velocity_n = vector3(row[8U], row[9U], row[10U]);
    sample.position_delta_n = vector3(row[11U], row[12U], row[13U]);
    sample.position_reference_time_us = row[14U] == 0.0 ? 0U : timestamp(row[14U]);
    GnssQuality::Metadata const metadata{metadata_integer(row[5U], 0, 1) == 1, metadata_integer(row[6U], -1, 1),
                                         metadata_integer(row[7U], -1, 1), metadata_integer(row[4U], -1, 1)};
    return monitor.update(sample, metadata, time, parameters);
}

void write_gnss_quality(std::ostream & stream, GnssQuality::Result const & result, std::uint64_t source_time,
                        std::uint64_t time)
{
    auto const & quality = result.quality;
    auto const & failures = quality.failures;
    stream << source_time << ',' << time << ',' << result.outcome << ',' << static_cast<int>(quality.status) << ','
           << result.evaluated << ',' << result.motion_available << ',' << result.spoofing_available << ','
           << quality.qualified << ',' << quality.invalid_fix;
    for (bool failed : {failures.satellites, failures.pdop, failures.horizontal_accuracy, failures.vertical_accuracy,
                        failures.speed_accuracy, failures.horizontal_drift, failures.vertical_drift,
                        failures.horizontal_speed, failures.vertical_speed, failures.spoofed})
    {
        stream << ',' << failed;
    }
    stream << ',' << quality.drift_valid << ',' << quality.horizontal_drift << ',' << quality.vertical_drift << ','
           << quality.horizontal_speed << ',' << quality.vertical_speed << ',' << quality.last_fail_us << ','
           << quality.last_pass_us << '\n';
}

std::string_view gnss_stop_reason(GnssLifecycle::StopReason reason)
{
    switch (reason)
    {
    case GnssLifecycle::StopReason::none:
        return "none";
    case GnssLifecycle::StopReason::disabled:
        return "disabled";
    case GnssLifecycle::StopReason::not_ready:
        return "not_ready";
    case GnssLifecycle::StopReason::waiting_for_quality:
        return "waiting_for_quality";
    case GnssLifecycle::StopReason::data_timeout:
        return "data_timeout";
    case GnssLifecycle::StopReason::poor_quality:
        return "poor_quality";
    case GnssLifecycle::StopReason::initialization_required:
        return "initialization_required";
    case GnssLifecycle::StopReason::fusion_timeout:
        return "fusion_timeout";
    case GnssLifecycle::StopReason::fusion_failed:
        return "fusion_failed";
    case GnssLifecycle::StopReason::reset_failed:
        return "reset_failed";
    }
    return "invalid_call";
}

bool gnss_recovery_required(GnssLifecycle::State const & state)
{
    return state.horizontal_position.recovery_required || state.vertical_position.recovery_required ||
           state.velocity.recovery_required;
}

void update_gnss_lifecycle(GnssLifecycle & lifecycle, GnssQuality & quality, GnssLifecycle::Sample const & sample,
                           std::uint64_t time)
{
    // The demo starts from an already seeded navigation frame and accel/mag
    // attitude. This is an initialization premise, not a detected alignment.
    // Every group's startup/re-entry uses an actual GNSS reset.
    GnssLifecycle::Conditions const conditions{{true, true, false}, {true, true, false}, {true, true, false}, true};
    GnssLifecycle::Parameters const timing{GNSS_HORIZONTAL_TIMEOUT_US, GNSS_HEIGHT_DATA_TIMEOUT_US,
                                           GNSS_HEIGHT_FUSION_TIMEOUT_US};
    check_status(lifecycle.update(time, conditions, timing, sample), "GNSS lifecycle", time);
    if (lifecycle.state().restart_quality)
    {
        quality.restart_health();
    }
}

void write_gnss_lifecycle(std::ostream & stream, GnssLifecycle::State const & previous,
                          GnssLifecycle::State const & state, std::uint64_t source_time, std::uint64_t time)
{
    struct Entry
    {
        std::string_view name;
        GnssLifecycle::GroupState GnssLifecycle::State::*group{};
    };
    for (auto const & entry : {Entry{"velocity", &GnssLifecycle::State::velocity},
                               Entry{"horizontal_position", &GnssLifecycle::State::horizontal_position},
                               Entry{"vertical_position", &GnssLifecycle::State::vertical_position}})
    {
        auto const & current = state.*entry.group;
        auto const & before = previous.*entry.group;
        if (source_time != 0U || current.active != before.active || current.stop_reason != before.stop_reason ||
            current.recovery_required != before.recovery_required)
        {
            stream << source_time << ',' << time << ',' << entry.name << ',' << current.active << ','
                   << gnss_stop_reason(current.stop_reason) << ',' << current.recovery_required << ','
                   << current.start_time_us << ',' << current.last_fuse_time_us << ',' << state.restart_quality << ','
                   << current.last_reset_time_us << '\n';
        }
    }
    if (gnss_recovery_required(state))
    {
        throw std::runtime_error("GNSS recovery is unsupported or failed; see gnss_lifecycle.csv and gnss_resets.csv");
    }
}

template <typename Vector>
void write_gnss_reset(std::ostream & stream, std::string_view group, GnssLifecycle::StopReason reason, Status status,
                      Vector const & delta, std::uint64_t source_time, std::uint64_t time)
{
    for (std::size_t axis = 0U; axis < Vector::row_count; ++axis)
    {
        stream << source_time << ',' << time << ',' << group << ',' << axis << ',' << gnss_stop_reason(reason) << ','
               << (status == Status::success ? "reset" : "failed") << ',' << static_cast<int>(status) << ','
               << (status == Status::success);
        // A failed checked reset has no valid delta; do not invent a zero reset.
        stream << ',';
        if (status == Status::success)
        {
            stream << delta(axis);
        }
        stream << '\n';
    }
}

struct NavigationVariances
{
    double position_xy{};
    double position_z{};
    double velocity_xy{};
    double velocity_z{};
};

NavigationVariances navigation_variances(double horizontal_accuracy, double vertical_accuracy, double speed_accuracy,
                                         std::optional<GnssParameters> const & gnss)
{
    if (!gnss)
    {
        double const velocity = variance(speed_accuracy, VEL_STD_MIN);
        return {variance(horizontal_accuracy, POS_XY_STD_MIN), variance(vertical_accuracy, POS_Z_STD_MIN), velocity,
                velocity};
    }
    if (!std::isfinite(horizontal_accuracy) || horizontal_accuracy <= 0.0)
    {
        throw std::runtime_error("Navigation accuracy must be positive and finite");
    }
    // This replay has no other horizontal aid and uses GNSS as its sole height
    // reference: cap horizontal noise only; no height bias or bias variance.
    double const position_std =
        std::min(std::max(horizontal_accuracy, gnss->position_noise), gnss->position_noise_limit);
    double const velocity_std = std::max(speed_accuracy, std::max(gnss->velocity_noise, 0.01));
    double const velocity_xy = variance(speed_accuracy, std::max(gnss->velocity_noise, 0.01));
    return {variance(position_std, 0.01), variance(vertical_accuracy, 1.5 * gnss->position_noise), velocity_xy,
            variance(1.5 * velocity_std, 0.0)};
}

struct MagneticReference
{
    Vector3 m_n{};
    Linalg::matrix_type<3U, 3U> V{};
    double gate_sigma{};
};

MagneticReference read_magnetic_reference(std::filesystem::path const & input)
{
    auto stream = open_csv(input / "magnetic_reference.csv", "mn,me,md,noise_std,gate_sigma");
    std::array<double, 5U> row{};
    std::array<double, 5U> extra{};
    if (!read_row(stream, row) || read_row(stream, extra))
    {
        throw std::runtime_error("expected exactly one magnetic reference");
    }
    auto const & [mn, me, md, noise_std, gate_sigma] = row;
    double const noise_variance = noise_std * noise_std;
    if (!(noise_std > 0.0) || !(gate_sigma > 0.0) || !std::isfinite(noise_variance) ||
        noise_variance < std::numeric_limits<double>::min() || std::hypot(mn, me) < MAG_MIN_HORIZONTAL)
    {
        throw std::runtime_error("invalid magnetic reference/noise/gate or insufficient horizontal field");
    }
    return {vector3(mn, me, md), Linalg::matrix_type<3U, 3U>::identity() * noise_variance, gate_sigma};
}

std::string_view fusion_outcome(formal_eskf::runtime::FusionDecision decision)
{
    switch (decision)
    {
    case formal_eskf::runtime::FusionDecision::fused:
        return "fused";
    case formal_eskf::runtime::FusionDecision::rejected:
        return "rejected";
    default:
        return "failed";
    }
}

std::string_view magnetic_stop_reason(MagneticLifecycle::StopReason reason)
{
    switch (reason)
    {
    case MagneticLifecycle::StopReason::none:
        return "none";
    case MagneticLifecycle::StopReason::disabled:
        return "disabled";
    case MagneticLifecycle::StopReason::not_ready:
        return "not_ready";
    case MagneticLifecycle::StopReason::waiting_for_health:
        return "waiting_for_health";
    case MagneticLifecycle::StopReason::field_rejected:
        return "field_rejected";
    case MagneticLifecycle::StopReason::innovation_rejected:
        return "innovation_rejected";
    case MagneticLifecycle::StopReason::data_timeout:
        return "data_timeout";
    case MagneticLifecycle::StopReason::fusion_timeout:
        return "fusion_timeout";
    case MagneticLifecycle::StopReason::recovery_required:
        return "recovery_required";
    default:
        return "invalid_call";
    }
}

std::string_view magnetic_recovery_reason(MagneticLifecycle::RecoveryReason reason)
{
    switch (reason)
    {
    case MagneticLifecycle::RecoveryReason::none:
        return "none";
    case MagneticLifecycle::RecoveryReason::fusion_timeout:
        return "fusion_timeout";
    case MagneticLifecycle::RecoveryReason::fusion_failed:
        return "fusion_failed";
    case MagneticLifecycle::RecoveryReason::reset_failed:
        return "reset_failed";
    default:
        return "source_changed";
    }
}

void write_magnetic_update(std::ostream & stream, std::uint64_t source_time, std::uint64_t time,
                           std::string_view outcome,
                           formal_eskf::runtime::MagnetometerFusionResult<Linalg> const & result,
                           MagneticLifecycle::State const & lifecycle)
{
    auto const & field = result.field;
    auto const & fusion = result.fusion;
    auto const status = formal_eskf::succeeded(field.status) ? fusion.status : field.status;
    stream << source_time << ',' << time << ',' << outcome << ',' << static_cast<int>(status) << ','
           << field.diagnostics_valid << ',' << field.failures.strength << ',' << field.failures.inclination << ','
           << field.failures.horizontal_field << ',' << fusion.diagnostics_valid << ',' << fusion.test_ratio(0U) << ','
           << fusion.test_ratio(1U) << ',' << fusion.test_ratio(2U) << ',' << lifecycle.active << ','
           << magnetic_stop_reason(lifecycle.stop_reason) << ',' << magnetic_recovery_reason(lifecycle.recovery_reason)
           << ',' << lifecycle.last_field_failure_time_us << ',' << lifecycle.start_time_us << ','
           << lifecycle.last_fuse_time_us << ',' << lifecycle.last_reset_time_us << '\n';
}

/** One row per observed axis; the outcome applies to the whole group. */
template <std::size_t Size>
bool record_navigation_fusion(std::ostream & stream, formal_eskf::runtime::FusionResult<Linalg, Size> const & result,
                              Linalg::matrix_type<Size, Size> const & V, std::string_view group,
                              std::uint64_t source_time, std::uint64_t time)
{
    for (std::size_t axis = 0U; axis < Size; ++axis)
    {
        stream << source_time << ',' << time << ',' << group << ',' << axis << ',' << fusion_outcome(result.decision)
               << ',' << static_cast<int>(result.status) << ',' << result.diagnostics_valid << ','
               << result.innovation(axis) << ',' << result.innovation_variance(axis) << ',' << result.test_ratio(axis)
               << ',' << V(axis, axis) << '\n';
    }
    return result.decision == formal_eskf::runtime::FusionDecision::fused;
}

Types::parameter_type parameters()
{
    Types::parameter_type result;
    result.gravity_n = vector3(0.0, 0.0, GRAVITY);
    result.minimum_quaternion_norm = QUAT_MIN_NORM;
    result.quaternion_squared_norm_tolerance = QUAT_SQUARED_NORM_TOL;
    result.dt_min = DT_MIN;
    result.dt_max = DT_MAX;
    result.process_noise.specific_force_variance = vector3(ACCEL_VAR, ACCEL_VAR, ACCEL_VAR);
    result.process_noise.angular_rate_variance = vector3(GYRO_VAR, GYRO_VAR, GYRO_VAR);
    result.process_noise.accelerometer_bias_random_walk_variance_density =
        vector3(ACCEL_BIAS_RW_VAR_DENSITY, ACCEL_BIAS_RW_VAR_DENSITY, ACCEL_BIAS_RW_VAR_DENSITY);
    result.process_noise.gyroscope_bias_random_walk_variance_density =
        vector3(GYRO_BIAS_RW_VAR_DENSITY, GYRO_BIAS_RW_VAR_DENSITY, GYRO_BIAS_RW_VAR_DENSITY);
    return result;
}

Covariance initial_covariance(NavigationVariances const & noise)
{
    // INS error-state order: [delta_p_n, delta_v_n, delta_theta_b, delta_b_a, delta_b_g].
    std::array<double, Types::error_state_dimension> const diagonal{
        noise.position_xy,    noise.position_xy,    noise.position_z,    // delta_p_n.
        noise.velocity_xy,    noise.velocity_xy,    noise.velocity_z,    // delta_v_n.
        INIT_ATTITUDE_XY_VAR, INIT_ATTITUDE_XY_VAR, INIT_ATTITUDE_Z_VAR, // delta_theta_b.
        INIT_ACCEL_BIAS_VAR,  INIT_ACCEL_BIAS_VAR,  INIT_ACCEL_BIAS_VAR, // delta_b_a.
        INIT_GYRO_BIAS_VAR,   INIT_GYRO_BIAS_VAR,   INIT_GYRO_BIAS_VAR,  // delta_b_g.
    };
    Covariance result;
    for (std::size_t index = 0U; index < diagonal.size(); ++index)
    {
        result.set(index, index, diagonal[index]);
    }
    return result;
}

void correct_navigation(State & state, Covariance & covariance, std::array<double, 10U> const & row, std::uint64_t time)
{
    // Adapter-provided standard deviations use independent axes and fixed
    // floors (m, m/s), not a full joint 6x6 observation covariance.
    auto const position = Linalg::vector_type<2U>::from_row_major({row[1U], row[2U]});
    double const horizontal_variance = variance(row[7U], POS_XY_STD_MIN);
    auto const position_noise = Linalg::matrix_type<2U, 2U>::identity() * horizontal_variance;
    check_status(formal_eskf::try_correct_horizontal_position(state, covariance, position, position_noise,
                                                              QUAT_MIN_NORM, state, covariance),
                 "horizontal position", time);
    check_status(formal_eskf::try_correct_vertical_position(
                     state, covariance, row[3U], variance(row[8U], POS_Z_STD_MIN), QUAT_MIN_NORM, state, covariance),
                 "vertical position", time);
    auto const velocity_noise = Linalg::matrix_type<3U, 3U>::identity() * variance(row[9U], VEL_STD_MIN);
    check_status(formal_eskf::try_correct_velocity(state, covariance, vector3(row[4U], row[5U], row[6U]),
                                                   velocity_noise, QUAT_MIN_NORM, state, covariance),
                 "velocity", time);
}

void check_state(State const & state, Covariance const & covariance)
{
    if (!formal_eskf::linalg::all_finite(state.p_n) || !formal_eskf::linalg::all_finite(state.v_n) ||
        !formal_eskf::linalg::all_finite(state.b_a) || !formal_eskf::linalg::all_finite(state.b_g) ||
        !formal_eskf::linalg::all_finite(state.q_nb.coefficients()) || !formal_eskf::linalg::all_finite(covariance) ||
        std::abs(formal_eskf::linalg::norm(state.q_nb.coefficients()) - 1.0) > 1e-10)
    {
        throw std::runtime_error("invalid state, covariance or quaternion norm");
    }
    for (std::size_t index = 0U; index < 15U; ++index)
    {
        if (covariance(index, index) < 0.0)
        {
            throw std::runtime_error("negative covariance diagonal");
        }
    }
    // Finite entries and nonnegative diagonals are NOT a complete PSD proof.
}

void write_state(std::ostream & stream, std::uint64_t time, State const & state, Covariance const & covariance)
{
    stream << time;
    for (auto const & vector : {state.p_n, state.v_n})
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            stream << ',' << vector(axis);
        }
    }
    stream << ',' << state.q_nb.q0() << ',' << state.q_nb.q1() << ',' << state.q_nb.q2() << ',' << state.q_nb.q3();
    for (auto const & vector : {state.b_a, state.b_g})
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            stream << ',' << vector(axis);
        }
    }
    for (std::size_t axis = 0U; axis < 15U; ++axis)
    {
        stream << ',' << covariance(axis, axis);
    }
    stream << '\n';
}

void replay(std::filesystem::path const & input, std::filesystem::path const & output, bool mag_enabled)
{
    auto const gnss = read_gnss_parameters(input);
    auto const quality_parameters = read_gnss_quality_parameters(input);
    GnssQuality quality_monitor;
    GnssLifecycle gnss_lifecycle;
    std::ifstream quality_samples;
    if (quality_parameters)
    {
        quality_samples =
            open_csv(input / "gnss_quality.csv", "timestamp_us,fix_type,satellites,pdop,spoofed,vel_ned_valid,in_air,"
                                                 "at_rest,vn,ve,vd,dn,de,dd,reference_timestamp_us");
    }
    auto initial = open_csv(input / "initial.csv", "timestamp_us,q0,q1,q2,q3,vn,ve,vd,eph,epv,speed_accuracy");
    auto imu = open_csv(input / "imu.csv", "timestamp_us,integral_dt_us,fx,fy,fz,wx,wy,wz");
    auto navigation = open_csv(input / "navigation.csv", "timestamp_us,pn,pe,pd,vn,ve,vd,eph,epv,speed_accuracy");
    std::ifstream magnetometer;
    MagneticReference magnetic;
    bool const has_magnetic_input = std::filesystem::exists(input / "magnetometer.csv");
    if (has_magnetic_input != std::filesystem::exists(input / "magnetic_reference.csv"))
    {
        throw std::runtime_error("magnetometer.csv and magnetic_reference.csv must be supplied together");
    }
    if (has_magnetic_input)
    {
        magnetic = read_magnetic_reference(input);
        magnetometer = open_csv(input / "magnetometer.csv", "timestamp_us,mx,my,mz");
    }
    std::ifstream flags;
    if (std::filesystem::exists(input / "imu_flags.csv"))
    {
        flags = open_csv(input / "imu_flags.csv", "timestamp_us,at_rest,accelerometer_clipping");
    }
    std::array<double, 11U> seed{};
    std::array<double, 8U> imu_row{};
    std::array<double, 10U> navigation_row{};
    std::array<double, 4U> magnetic_row{};
    if (!read_row(initial, seed) || !read_row(imu, imu_row))
    {
        throw std::runtime_error("empty initial state or IMU input");
    }
    std::array<double, 11U> extra_seed{};
    if (read_row(initial, extra_seed))
    {
        throw std::runtime_error("expected exactly one initial state");
    }
    auto const & [seed_time, q0, q1, q2, q3, vn, ve, vd, horizontal_accuracy, vertical_accuracy, speed_accuracy] = seed;
    std::uint64_t time = timestamp(seed_time);
    std::uint64_t const start_time = time;
    if (timestamp(imu_row[0U]) != time)
    {
        throw std::runtime_error("initial state must correspond to the first IMU row");
    }
    State state;
    state.v_n = vector3(vn, ve, vd);
    check_status(State::quaternion_type::try_from_coefficients(q0, q1, q2, q3, QUAT_MIN_NORM, state.q_nb),
                 "initial attitude", time);
    auto covariance =
        initial_covariance(navigation_variances(horizontal_accuracy, vertical_accuracy, speed_accuracy, gnss));
    auto const settings = parameters();
    FusionConditions fusion_conditions;
    Accelerometer accelerometer;
    MagneticLifecycle magnetic_lifecycle;
    MagneticLifecycle::Parameters const magnetic_timing{MAG_HEALTH_TIME_US, MAG_MAX_INTERVAL_US, MAG_FUSION_TIMEOUT_US};
    auto const accel_V = Linalg::matrix_type<3U, 3U>::identity() * (ACCEL_OBS_STD * ACCEL_OBS_STD);
    // Consume seed metadata, but initialization is not a successful aiding event.
    (void)read_imu_flags(flags, time);
    auto previous_imu = imu_sample(imu_row);
    std::uint64_t last_navigation_time = time;
    bool has_navigation = read_row(navigation, navigation_row, quality_parameters.has_value());
    std::uint64_t last_magnetic_time = time;
    bool has_magnetometer = has_magnetic_input && read_row(magnetometer, magnetic_row);
    if (has_magnetic_input && !has_magnetometer)
    {
        throw std::runtime_error("empty magnetometer input");
    }
    auto const field_parameters = formal_eskf::runtime::MagneticFieldParameters<double>{
        MAG_STRENGTH_TOL, MAG_INCLINATION_TOL * std::numbers::pi / 180.0, 0.0, MAG_MIN_HORIZONTAL};
    if (!std::filesystem::create_directory(output))
    {
        throw std::runtime_error("output directory already exists: " + output.string());
    }
    std::ofstream states(output / "state.csv");
    std::ofstream updates(output / "navigation_updates.csv");
    std::ofstream gnss_updates;
    std::ofstream quality_updates;
    std::ofstream lifecycle_updates;
    std::ofstream gnss_resets;
    if (quality_parameters)
    {
        lifecycle_updates.open(output / "gnss_lifecycle.csv");
        lifecycle_updates.exceptions(std::ios::failbit | std::ios::badbit);
        lifecycle_updates << "source_timestamp_us,fusion_timestamp_us,group,active,stop_reason,recovery_required,"
                             "start_timestamp_us,last_fuse_timestamp_us,restart_quality,last_reset_timestamp_us\n";
        gnss_resets.open(output / "gnss_resets.csv");
        gnss_resets.exceptions(std::ios::failbit | std::ios::badbit);
        gnss_resets << std::setprecision(17);
        gnss_resets << "source_timestamp_us,fusion_timestamp_us,group,axis,reason,outcome,status,delta_valid,delta\n";
        quality_updates.open(output / "gnss_quality.csv");
        quality_updates.exceptions(std::ios::failbit | std::ios::badbit);
        quality_updates << std::setprecision(17);
        quality_updates
            << "source_timestamp_us,fusion_timestamp_us,outcome,status,evaluated,motion_available,spoofing_available,"
               "qualified,invalid_fix,satellites,pdop,horizontal_accuracy,vertical_accuracy,speed_accuracy,"
               "horizontal_drift,vertical_drift,horizontal_speed,vertical_speed,spoofed,drift_valid,"
               "horizontal_drift_mps,vertical_drift_mps,horizontal_speed_mps,vertical_speed_mps,last_fail_us,last_pass_"
               "us\n";
    }
    if (gnss)
    {
        gnss_updates.open(output / "gnss_updates.csv");
        gnss_updates.exceptions(std::ios::failbit | std::ios::badbit);
        gnss_updates << std::setprecision(17);
        gnss_updates << "source_timestamp_us,fusion_timestamp_us,group,axis,outcome,status,diagnostics_valid,"
                        "innovation,innovation_variance,test_ratio,observation_variance\n";
    }
    std::ofstream condition_log(output / "fusion_conditions.csv");
    std::ofstream accel_updates(output / "accelerometer_updates.csv");
    std::ofstream mag_updates(output / "magnetometer_updates.csv");
    std::ofstream mag_resets(output / "magnetometer_resets.csv");
    states.exceptions(std::ios::failbit | std::ios::badbit);
    updates.exceptions(std::ios::failbit | std::ios::badbit);
    condition_log.exceptions(std::ios::failbit | std::ios::badbit);
    accel_updates.exceptions(std::ios::failbit | std::ios::badbit);
    mag_updates.exceptions(std::ios::failbit | std::ios::badbit);
    mag_resets.exceptions(std::ios::failbit | std::ios::badbit);
    mag_resets << std::setprecision(17)
               << "source_timestamp_us,fusion_timestamp_us,status,delta_q0,delta_q1,delta_q2,delta_q3,yaw_variance\n";
    mag_updates << std::setprecision(17);
    mag_updates << "source_timestamp_us,fusion_timestamp_us,outcome,status,field_valid,strength_rejected,"
                   "inclination_rejected,horizontal_rejected,innovation_valid,ratio_x,ratio_y,ratio_z,"
                   "active,stop_reason,recovery_reason,last_field_failure_timestamp_us,start_timestamp_us,last_fuse_"
                   "timestamp_us,last_reset_timestamp_us\n";
    accel_updates << std::setprecision(17);
    accel_updates << "timestamp_us,interval_us,fx,fy,fz,clipping_mask,lpf_squared_norm,within_gravity_band,"
                     "tilt_aligned,horizontal_aiding,at_rest,outcome\n";
    condition_log << std::setprecision(17);
    condition_log << "timestamp_us,tilt_variance,tilt_aligned,horizontal_aiding,at_rest,accelerometer_clipping,"
                     "conditions_available\n";
    states << std::setprecision(17);
    states << "timestamp_us,pn,pe,pd,vn,ve,vd,q0,q1,q2,q3,bax,bay,baz,bgx,bgy,bgz";
    for (std::size_t index = 0U; index < 15U; ++index)
    {
        states << ",P" << index;
    }
    states << '\n';
    updates << "source_timestamp_us,fusion_timestamp_us\n";
    check_state(state, covariance);
    write_state(states, time, state, covariance);
    std::size_t predictions = 0U, navigation_events = 0U, gaps = 0U;
    std::size_t position_xy_fused = 0U, position_z_fused = 0U, velocity_fused = 0U;
    std::size_t position_xy_resets = 0U, position_z_resets = 0U, velocity_resets = 0U;
    std::size_t vertical_velocity_resets = 0U;
    std::size_t quality_accepted = 0U, quality_rejected = 0U, quality_waiting = 0U, quality_unavailable = 0U;
    std::size_t invalid_velocity = 0U, navigation_attempts = 0U;
    std::size_t position_xy_attempts = 0U, position_z_attempts = 0U, velocity_attempts = 0U;
    std::size_t position_xy_skipped = 0U, position_z_skipped = 0U, velocity_skipped = 0U;
    std::uint64_t gap_us = 0U, max_alignment_us = 0U;
    std::size_t unknown_clipping_samples = 0U, available_condition_samples = 0U;
    std::size_t accel_observations = 0U, accel_corrections = 0U, accel_rejected = 0U, accel_unavailable = 0U;
    std::uint64_t accel_interval_min_us = 0U, accel_interval_max_us = 0U, accel_interval_total_us = 0U;
    std::size_t mag_events = 0U, mag_corrections = 0U, mag_rejected = 0U;
    std::size_t mag_skipped = 0U, mag_superseded = 0U;
    std::size_t mag_heading_resets = 0U;
    std::uint64_t mag_max_alignment_us = 0U;
    while (read_row(imu, imu_row))
    {
        auto const next_time = timestamp(imu_row[0U]);
        auto const period = timestamp(imu_row[1U]);
        if (next_time <= time || period > next_time - time || next_time - time > 50000U)
        {
            throw std::runtime_error("IMU timestamp is nonmonotonic, intervals overlap, or gap exceeds 50 ms");
        }
        auto const missing = next_time - time - period;
        if (missing != 0U)
        {
            // A logged average only covers its integral_dt. Explicitly hold
            // the PREVIOUS rate over the missing interval, then use this one.
            check_status(formal_eskf::try_predict(state, covariance, previous_imu, static_cast<double>(missing) * 1e-6,
                                                  settings, state, covariance),
                         "gap hold prediction", time + missing);
            ++gaps;
            gap_us += missing;
            // Held prediction rates are not new measurements. Discard the
            // incomplete observation and LPF history across a missing interval.
            accelerometer.reset();
        }
        auto const sample = imu_sample(imu_row);
        check_status(formal_eskf::try_predict(state, covariance, sample, static_cast<double>(period) * 1e-6, settings,
                                              state, covariance),
                     "prediction", next_time);
        time = next_time;
        previous_imu = sample;
        ++predictions;
        check_state(state, covariance);
        auto const sample_flags = read_imu_flags(flags, time);
        check_status(fusion_conditions.update_tilt(state.q_nb, covariance.block<6U, 6U, 3U, 3U>()),
                     "tilt alignment conditions", time);
        auto const observation = accelerometer.update(sample, period, sample_flags[1U], QUAT_MIN_NORM);
        check_status(observation.status, "accelerometer observation", time);
        // First-version timing approximation: each NEW navigation event is fused
        // at the following IMU tick. Do not split sampled-noise IMU steps or
        // pretend this compensates receiver latency. Report alignment error.
        bool lifecycle_updated = false;
        while (has_navigation && timestamp(navigation_row[0U]) <= time)
        {
            auto const source_time = timestamp(navigation_row[0U]);
            if (source_time <= last_navigation_time)
            {
                throw std::runtime_error("Navigation timestamps must increase after initialization");
            }
            bool quality_allows_fusion = true;
            auto const previous_lifecycle = gnss_lifecycle.state();
            bool lifecycle_sample_consumed = false;
            if (quality_parameters)
            {
                auto const result =
                    check_gnss_quality(quality_samples, quality_monitor, *quality_parameters, navigation_row, time);
                write_gnss_quality(quality_updates, result, source_time, time);
                if (result.outcome == "invalid_call")
                {
                    check_status(result.quality.status, "GNSS quality input", time);
                }
                quality_allows_fusion = result.accepted();
                quality_accepted += result.accepted() ? 1U : 0U;
                quality_rejected += result.outcome == "rejected" ? 1U : 0U;
                quality_waiting += result.outcome == "waiting" ? 1U : 0U;
                quality_unavailable += result.outcome == "unavailable" ? 1U : 0U;
                invalid_velocity += result.outcome == "invalid_velocity" ? 1U : 0U;
                auto const lifecycle_sample = result.lifecycle_sample(source_time);
                if (lifecycle_sample.time_us != 0U)
                {
                    update_gnss_lifecycle(gnss_lifecycle, quality_monitor, lifecycle_sample, time);
                    lifecycle_updated = true;
                    lifecycle_sample_consumed = true;
                    auto const & lifecycle = gnss_lifecycle.state();
                    if ((lifecycle.vertical_position.recovery_required && !lifecycle.vertical_position.reset_allowed) ||
                        (lifecycle.velocity.recovery_required && !lifecycle.velocity.reset_allowed) ||
                        (lifecycle.horizontal_position.recovery_required &&
                         !lifecycle.horizontal_position.reset_allowed))
                    {
                        // Record the request before any group mutates the estimate.
                        write_gnss_lifecycle(lifecycle_updates, previous_lifecycle, gnss_lifecycle.state(), source_time,
                                             time);
                    }
                }
            }
            // Every message advances the input cursor, including quality failures.
            // Skips do not invoke a correction or refresh horizontal-aiding time.
            updates << source_time << ',' << time << '\n';
            max_alignment_us = std::max(max_alignment_us, time - source_time);
            last_navigation_time = source_time;
            ++navigation_events;
            if (!quality_allows_fusion)
            {
                if (lifecycle_sample_consumed)
                {
                    write_gnss_lifecycle(lifecycle_updates, previous_lifecycle, gnss_lifecycle.state(), source_time,
                                         time);
                }
                has_navigation = read_row(navigation, navigation_row, true);
                continue;
            }
            ++navigation_attempts;
            if (gnss)
            {
                auto record_fusion = [&](auto const & result, auto const & V, std::string_view group)
                {
                    bool const fused = record_navigation_fusion(gnss_updates, result, V, group, source_time, time);
                    if (quality_parameters && result.status != Status::success)
                    {
                        // Preserve the recovery request before stopping on numerical failure.
                        write_gnss_lifecycle(lifecycle_updates, previous_lifecycle, gnss_lifecycle.state(), source_time,
                                             time);
                    }
                    check_status(result.status, group, time);
                    return fused;
                };
                auto record_reset = [&](GnssLifecycle::Group group, std::string_view name,
                                        GnssLifecycle::StopReason reason, Status status, auto const & delta)
                {
                    check_status(gnss_lifecycle.record_reset(group, source_time, status == Status::success),
                                 "GNSS reset feedback", time);
                    write_gnss_reset(gnss_resets, name, reason, status, delta, source_time, time);
                    if (status != Status::success)
                    {
                        write_gnss_lifecycle(lifecycle_updates, previous_lifecycle, gnss_lifecycle.state(), source_time,
                                             time);
                    }
                    check_status(status, "GNSS reset", time);
                    if (group != GnssLifecycle::Group::vertical_position)
                    {
                        fusion_conditions.record_horizontal_fusion(time);
                    }
                };
                auto const noise =
                    navigation_variances(navigation_row[7U], navigation_row[8U], navigation_row[9U], gnss);
                // Each group gates and corrects its own latest prior. A rejected
                // group does not block the others or refresh horizontal aiding.
                auto velocity_V = Linalg::matrix_type<3U, 3U>::identity() * noise.velocity_xy;
                velocity_V.set(2U, 2U, noise.velocity_z);
                auto const velocity = vector3(navigation_row[4U], navigation_row[5U], navigation_row[6U]);
                if (quality_parameters && gnss_lifecycle.state().velocity.reset_allowed)
                {
                    Vector3 delta;
                    auto const reason = gnss_lifecycle.state().velocity.stop_reason;
                    auto const status = formal_eskf::runtime::try_reset_gnss_velocity(
                        state, covariance, velocity, vector3(noise.velocity_xy, noise.velocity_xy, noise.velocity_z),
                        state, covariance, delta);
                    record_reset(GnssLifecycle::Group::velocity, "velocity", reason, status, delta);
                    ++velocity_resets;
                }
                else if (!quality_parameters || gnss_lifecycle.state().velocity.fusion_allowed)
                {
                    ++velocity_attempts;
                    auto const result = formal_eskf::runtime::fuse_velocity(
                        state, covariance, velocity, velocity_V, QUAT_MIN_NORM, gnss->velocity_gate, state, covariance);
                    if (quality_parameters)
                    {
                        check_status(
                            gnss_lifecycle.record_fusion(GnssLifecycle::Group::velocity, source_time, result.decision),
                            "GNSS velocity feedback", time);
                    }
                    if (record_fusion(result, velocity_V, "velocity"))
                    {
                        ++velocity_fused;
                        fusion_conditions.record_horizontal_fusion(time);
                    }
                }
                else
                {
                    ++velocity_skipped;
                }

                // Height timeout is sampled before this event's height correction.
                // Reset only Down after normal velocity fusion (even if gated out).
                if (quality_parameters && gnss_lifecycle.state().vertical_velocity_reset_allowed)
                {
                    double delta{};
                    auto const status = formal_eskf::runtime::try_reset_gnss_vertical_velocity(
                        state, covariance, velocity(2U), noise.velocity_z, state, covariance, delta);
                    check_status(gnss_lifecycle.record_vertical_velocity_reset(source_time, status == Status::success),
                                 "GNSS vertical velocity reset feedback", time);
                    write_gnss_reset(gnss_resets, "vertical_velocity", GnssLifecycle::StopReason::fusion_timeout,
                                     status, Linalg::vector_type<1U>::from_row_major({delta}), source_time, time);
                    if (status != Status::success)
                    {
                        write_gnss_lifecycle(lifecycle_updates, previous_lifecycle, gnss_lifecycle.state(), source_time,
                                             time);
                    }
                    check_status(status, "GNSS vertical velocity reset", time);
                    ++vertical_velocity_resets;
                }

                auto const position_V = Linalg::matrix_type<2U, 2U>::identity() * noise.position_xy;
                auto const position = Linalg::vector_type<2U>::from_row_major({navigation_row[1U], navigation_row[2U]});
                if (quality_parameters && gnss_lifecycle.state().horizontal_position.reset_allowed)
                {
                    Linalg::vector_type<2U> delta;
                    auto const reason = gnss_lifecycle.state().horizontal_position.stop_reason;
                    auto const variances =
                        Linalg::vector_type<2U>::from_row_major({noise.position_xy, noise.position_xy});
                    auto const status = formal_eskf::runtime::try_reset_gnss_horizontal_position(
                        state, covariance, position, variances, state, covariance, delta);
                    record_reset(GnssLifecycle::Group::horizontal_position, "horizontal_position", reason, status,
                                 delta);
                    ++position_xy_resets;
                }
                else if (!quality_parameters || gnss_lifecycle.state().horizontal_position.fusion_allowed)
                {
                    ++position_xy_attempts;
                    auto const result = formal_eskf::runtime::fuse_horizontal_position(
                        state, covariance, position, position_V, QUAT_MIN_NORM, gnss->position_gate, state, covariance);
                    if (quality_parameters)
                    {
                        check_status(gnss_lifecycle.record_fusion(GnssLifecycle::Group::horizontal_position,
                                                                  source_time, result.decision),
                                     "GNSS horizontal position feedback", time);
                    }
                    if (record_fusion(result, position_V, "horizontal_position"))
                    {
                        ++position_xy_fused;
                        fusion_conditions.record_horizontal_fusion(time);
                    }
                }
                else
                {
                    ++position_xy_skipped;
                }

                if (!quality_parameters || gnss_lifecycle.state().vertical_position.fusion_allowed)
                {
                    ++position_z_attempts;
                    auto const result = formal_eskf::runtime::fuse_vertical_position(
                        state, covariance, navigation_row[3U], noise.position_z, QUAT_MIN_NORM, gnss->position_gate,
                        state, covariance);
                    auto const height_V = Linalg::matrix_type<1U, 1U>::from_row_major({noise.position_z});
                    if (quality_parameters)
                    {
                        check_status(gnss_lifecycle.record_fusion(GnssLifecycle::Group::vertical_position, source_time,
                                                                  result.decision),
                                     "GNSS height feedback", time);
                    }
                    if (record_fusion(result, height_V, "vertical_position"))
                    {
                        ++position_z_fused;
                    }
                }
                else if (!gnss_lifecycle.state().vertical_position.reset_allowed)
                {
                    ++position_z_skipped;
                }
                // Height recovery is decided after the current correction attempt.
                // A successful overdue correction therefore needs no reset.
                if (quality_parameters && gnss_lifecycle.state().vertical_position.reset_allowed)
                {
                    double delta{};
                    auto const reason = gnss_lifecycle.state().vertical_position.stop_reason;
                    auto const status = formal_eskf::runtime::try_reset_gnss_vertical_position(
                        state, covariance, navigation_row[3U], noise.position_z, state, covariance, delta);
                    record_reset(GnssLifecycle::Group::vertical_position, "vertical_position", reason, status,
                                 Linalg::vector_type<1U>::from_row_major({delta}));
                    ++position_z_resets;
                }
            }
            else
            {
                correct_navigation(state, covariance, navigation_row, time);
                ++position_xy_fused;
                ++position_z_fused;
                ++velocity_fused;
                ++position_xy_attempts;
                ++position_z_attempts;
                ++velocity_attempts;
                fusion_conditions.record_horizontal_fusion(time);
            }
            if (quality_parameters)
            {
                write_gnss_lifecycle(lifecycle_updates, previous_lifecycle, gnss_lifecycle.state(), source_time, time);
            }
            check_state(state, covariance);
            has_navigation = read_row(navigation, navigation_row, quality_parameters.has_value());
        }
        if (quality_parameters && !lifecycle_updated)
        {
            auto const previous = gnss_lifecycle.state();
            update_gnss_lifecycle(gnss_lifecycle, quality_monitor, {}, time);
            write_gnss_lifecycle(lifecycle_updates, previous, gnss_lifecycle.state(), 0U, time);
        }
        // Retain raw-tick diagnostics; fusion uses the whole observation's clipping below.
        auto const conditions = fusion_conditions.conditions(time, true, sample_flags[0U], sample_flags[1U]);
        unknown_clipping_samples += sample_flags[1U] == -1 ? 1U : 0U;
        available_condition_samples += conditions.has_value() ? 1U : 0U;
        condition_log << time << ',' << fusion_conditions.tilt_variance() << ',' << fusion_conditions.tilt_aligned()
                      << ',' << fusion_conditions.horizontal_aiding(time) << ',' << sample_flags[0U] << ','
                      << sample_flags[1U] << ',' << conditions.has_value() << '\n';
        if (observation.ready)
        {
            ++accel_observations;
            accel_interval_total_us += observation.interval_us;
            accel_interval_min_us = accel_observations == 1U ? observation.interval_us
                                                             : std::min(accel_interval_min_us, observation.interval_us);
            accel_interval_max_us = std::max(accel_interval_max_us, observation.interval_us);
            auto const observation_conditions =
                fusion_conditions.conditions(time, true, sample_flags[0U], observation.clipping_mask);
            std::string_view outcome = "metadata_unavailable";
            if (observation_conditions)
            {
                auto const result = formal_eskf::runtime::fuse_accelerometer(
                    state, covariance, observation.specific_force_b, settings.gravity_n, accel_V,
                    *observation_conditions, observation.magnitude, QUAT_MIN_NORM, state, covariance);
                check_status(result.status, "accelerometer correction", time);
                bool const fused = result.decision == formal_eskf::runtime::FusionDecision::fused;
                accel_corrections += fused ? 1U : 0U;
                accel_rejected += fused ? 0U : 1U;
                outcome = fused ? "fused" : "rejected";
                check_state(state, covariance);
            }
            else
            {
                ++accel_unavailable;
            }
            accel_updates << time << ',' << observation.interval_us << ',' << observation.specific_force_b(0U) << ','
                          << observation.specific_force_b(1U) << ',' << observation.specific_force_b(2U) << ','
                          << observation.clipping_mask << ',' << observation.magnitude.squared_norm << ','
                          << observation.magnitude.within_gravity_band << ',' << fusion_conditions.tilt_aligned() << ','
                          << fusion_conditions.horizontal_aiding(time) << ',' << sample_flags[0U] << ',' << outcome
                          << '\n';
        }
        // Select the newest available sample once per tick. Older samples from
        // the same tick are superseded, not repeatedly fused at one horizon.
        std::uint64_t magnetic_time = 0U;
        Vector3 magnetic_sample;
        while (has_magnetometer && timestamp(magnetic_row[0U]) <= time)
        {
            auto const source_time = timestamp(magnetic_row[0U]);
            if (source_time <= last_magnetic_time)
            {
                throw std::runtime_error("Magnetometer timestamps must increase after initialization");
            }
            ++mag_events;
            mag_superseded += magnetic_time != 0U ? 1U : 0U;
            magnetic_time = source_time;
            magnetic_sample = vector3(magnetic_row[1U], magnetic_row[2U], magnetic_row[3U]);
            last_magnetic_time = source_time;
            has_magnetometer = read_row(magnetometer, magnetic_row);
        }
        if (has_magnetic_input)
        {
            auto const previous = magnetic_lifecycle.state();
            formal_eskf::runtime::MagnetometerFusionResult<Linalg> result;
            if (mag_enabled && magnetic_time != 0U)
            {
                // Same prior for field qualification and, if authorized, fusion.
                // GNSS availability alone does not prove heading observability.
                result.field = formal_eskf::runtime::check_magnetic_field(state.q_nb, magnetic_sample, magnetic.m_n,
                                                                          field_parameters, false);
            }
            // This fixed-reference CSV session has one adapter-assigned identity,
            // not a claimed hardware ID/calibration counter. Preparation rejects
            // recorded source/calibration changes; unrecorded changes are unknown.
            MagneticLifecycle::Sample const lifecycle_sample{
                magnetic_time, {1U, 0U}, formal_eskf::succeeded(result.field.status) && result.field.accepted};
            // Initial yaw/reference come from the causal seed. Tilt readiness is
            // the existing covariance-based latch, never imported EKF2 status.
            MagneticLifecycle::Conditions const magnetic_conditions{mag_enabled, fusion_conditions.tilt_aligned(), true,
                                                                    fusion_conditions.horizontal_aiding(time),
                                                                    sample_flags[0U] == 1};
            check_status(magnetic_lifecycle.update(time, magnetic_conditions, magnetic_timing, lifecycle_sample),
                         "magnetometer lifecycle", time);
            std::string_view outcome = magnetic_time == 0U ? "no_sample" : "skipped";
            if (!formal_eskf::succeeded(result.field.status))
            {
                outcome = "failed";
            }
            else if (magnetic_lifecycle.state().fusion_allowed)
            {
                result = formal_eskf::runtime::fuse_magnetometer(state, covariance, magnetic_sample, magnetic.m_n,
                                                                 magnetic.V, field_parameters, false, QUAT_MIN_NORM,
                                                                 magnetic.gate_sigma, state, covariance);
                check_status(magnetic_lifecycle.record_fusion(magnetic_time, result.fusion.decision),
                             "magnetometer fusion feedback", time);
                outcome = fusion_outcome(result.fusion.decision);
                mag_corrections += result.fusion.decision == formal_eskf::runtime::FusionDecision::fused ? 1U : 0U;
                mag_rejected += result.fusion.decision == formal_eskf::runtime::FusionDecision::rejected ? 1U : 0U;
            }
            else if (magnetic_time != 0U)
            {
                ++mag_skipped;
            }
            Status reset_status = Status::success;
            if (magnetic_lifecycle.state().reset_allowed)
            {
                formal_eskf::so3::UnitQuaternion<Linalg> delta;
                reset_status = formal_eskf::runtime::try_reset_magnetic_heading(
                    state, covariance, magnetic_sample, magnetic.m_n, MAG_HEADING_STD * MAG_HEADING_STD,
                    MAG_MIN_HORIZONTAL, QUAT_MIN_NORM, state, covariance, delta);
                bool const reset_succeeded = formal_eskf::succeeded(reset_status);
                check_status(magnetic_lifecycle.record_reset(magnetic_time, reset_succeeded),
                             "magnetometer reset feedback", time);
                mag_resets << magnetic_time << ',' << time << ',' << static_cast<int>(reset_status);
                for (std::size_t axis = 0U; axis < 4U; ++axis)
                {
                    // A failed transaction has no applied rotation.
                    mag_resets << ',' << (reset_succeeded ? delta.coefficients()(axis) : 0.0);
                }
                mag_resets << ',' << MAG_HEADING_STD * MAG_HEADING_STD << '\n';
                mag_heading_resets += reset_succeeded ? 1U : 0U;
                outcome = reset_succeeded ? "reset" : "reset_failed";
            }
            auto const & lifecycle = magnetic_lifecycle.state();
            if (magnetic_time != 0U || lifecycle.stop_reason != previous.stop_reason ||
                lifecycle.recovery_reason != previous.recovery_reason)
            {
                // Include no-data transitions so dropouts/timeouts remain visible.
                write_magnetic_update(mag_updates, magnetic_time, time, outcome, result, lifecycle);
            }
            if (magnetic_time != 0U)
            {
                mag_max_alignment_us = std::max(mag_max_alignment_us, time - magnetic_time);
            }
            check_status(result.field.status, "magnetic field check", time);
            check_status(result.fusion.status, "magnetometer correction", time);
            check_status(reset_status, "magnetometer heading reset", time);
            if (lifecycle.recovery_reason != MagneticLifecycle::RecoveryReason::none &&
                lifecycle.recovery_reason != MagneticLifecycle::RecoveryReason::fusion_timeout)
            {
                throw std::runtime_error("magnetometer requires caller repair (" +
                                         std::string(magnetic_recovery_reason(lifecycle.recovery_reason)) +
                                         "); see magnetometer_updates.csv and magnetometer_resets.csv");
            }
            if (result.fusion.decision == formal_eskf::runtime::FusionDecision::fused || outcome == "reset")
            {
                check_state(state, covariance);
            }
        }
        write_state(states, time, state, covariance);
    }
    if (has_navigation || has_magnetometer || predictions == 0U || navigation_events == 0U)
    {
        throw std::runtime_error("replay incomplete: unconsumed sensor data or no prediction/navigation events");
    }
    std::array<double, 3U> extra_flags{};
    if (flags.is_open() && read_row(flags, extra_flags))
    {
        throw std::runtime_error("unconsumed IMU flags");
    }
    std::array<double, 15U> extra_quality{};
    if (quality_samples.is_open() && read_row(quality_samples, extra_quality, true))
    {
        throw std::runtime_error("unconsumed GNSS quality data");
    }
    if (quality_updates.is_open())
    {
        quality_updates.close();
        lifecycle_updates.close();
        gnss_resets.close();
    }
    states.close();
    updates.close();
    if (gnss_updates.is_open())
    {
        gnss_updates.close();
    }
    condition_log.close();
    accel_updates.close();
    mag_updates.close();
    mag_resets.close();
    std::ofstream summary(output / "summary.json");
    summary.exceptions(std::ios::failbit | std::ios::badbit);
    summary << std::setprecision(17) << "{\n  \"result\": \"smoke_pass\",\n  \"accuracy_validated\": false,"
            << "\n  \"linalg_backend\": \"" << backend_name << "\","
            << "\n  \"duration_s\": " << static_cast<double>(time - start_time) * 1e-6
            << ",\n  \"imu_predictions\": " << predictions << ",\n  \"navigation_events\": " << navigation_events
            << ",\n  \"horizontal_position_corrections\": " << position_xy_fused
            << ",\n  \"vertical_position_corrections\": " << position_z_fused
            << ",\n  \"velocity_corrections\": " << velocity_fused
            << ",\n  \"horizontal_position_resets\": " << position_xy_resets
            << ",\n  \"vertical_position_resets\": " << position_z_resets
            << ",\n  \"velocity_resets\": " << velocity_resets
            << ",\n  \"vertical_velocity_resets\": " << vertical_velocity_resets
            << ",\n  \"horizontal_position_rejected\": " << position_xy_attempts - position_xy_fused
            << ",\n  \"vertical_position_rejected\": " << position_z_attempts - position_z_fused
            << ",\n  \"velocity_rejected\": " << velocity_attempts - velocity_fused
            << ",\n  \"gnss_lifecycle_enabled\": " << (quality_parameters ? "true" : "false")
            << ",\n  \"horizontal_position_lifecycle_skipped\": " << position_xy_skipped
            << ",\n  \"vertical_position_lifecycle_skipped\": " << position_z_skipped
            << ",\n  \"velocity_lifecycle_skipped\": " << velocity_skipped
            << ",\n  \"gnss_quality_enabled\": " << (quality_parameters ? "true" : "false")
            << ",\n  \"gnss_quality_accepted\": " << quality_accepted
            << ",\n  \"gnss_quality_rejected\": " << quality_rejected
            << ",\n  \"gnss_quality_waiting\": " << quality_waiting
            << ",\n  \"gnss_quality_unavailable\": " << quality_unavailable
            << ",\n  \"gnss_invalid_velocity\": " << invalid_velocity
            << ",\n  \"navigation_fusion_attempts\": " << navigation_attempts
            << ",\n  \"gnss_gating_enabled\": " << (gnss ? "true" : "false")
            << ",\n  \"gnss_position_gate\": " << (gnss ? gnss->position_gate : 0.0)
            << ",\n  \"gnss_velocity_gate\": " << (gnss ? gnss->velocity_gate : 0.0)
            << ",\n  \"imu_gap_holds\": " << gaps << ",\n  \"imu_gap_us\": " << gap_us
            << ",\n  \"navigation_max_alignment_us\": " << max_alignment_us
            << ",\n  \"navigation_delay_compensation_ms\": 0"
            << ",\n  \"available_gravity_condition_samples\": " << available_condition_samples
            << ",\n  \"unknown_accelerometer_clipping_samples\": " << unknown_clipping_samples
            << ",\n  \"accelerometer_observations\": " << accel_observations
            << ",\n  \"accelerometer_corrections\": " << accel_corrections
            << ",\n  \"accelerometer_rejected\": " << accel_rejected
            << ",\n  \"accelerometer_metadata_unavailable\": " << accel_unavailable
            << ",\n  \"accelerometer_observation_variance\": " << ACCEL_OBS_STD * ACCEL_OBS_STD
            << ",\n  \"accelerometer_target_interval_us\": " << ACCEL_OBS_PERIOD_US
            << ",\n  \"accelerometer_interval_min_us\": " << accel_interval_min_us
            << ",\n  \"accelerometer_interval_max_us\": " << accel_interval_max_us
            << ",\n  \"accelerometer_interval_mean_us\": "
            << (accel_observations == 0U
                    ? 0.0
                    : static_cast<double>(accel_interval_total_us) / static_cast<double>(accel_observations))
            << ",\n  \"magnetometer_enabled\": " << (mag_enabled && has_magnetic_input ? "true" : "false")
            << ",\n  \"magnetometer_events\": " << mag_events
            << ",\n  \"magnetometer_corrections\": " << mag_corrections
            << ",\n  \"magnetometer_heading_resets\": " << mag_heading_resets
            << ",\n  \"magnetometer_reset_yaw_variance\": " << MAG_HEADING_STD * MAG_HEADING_STD
            << ",\n  \"magnetometer_rejected\": " << mag_rejected
            << ",\n  \"magnetometer_lifecycle_skipped\": " << mag_skipped
            << ",\n  \"magnetometer_superseded\": " << mag_superseded
            << ",\n  \"magnetometer_health_time_us\": " << MAG_HEALTH_TIME_US
            << ",\n  \"magnetometer_maximum_interval_us\": " << MAG_MAX_INTERVAL_US
            << ",\n  \"magnetometer_data_timeout_us\": " << 2U * MAG_MAX_INTERVAL_US
            << ",\n  \"magnetometer_fusion_timeout_us\": " << MAG_FUSION_TIMEOUT_US
            << ",\n  \"magnetometer_stop_reason\": \"" << magnetic_stop_reason(magnetic_lifecycle.state().stop_reason)
            << '"' << ",\n  \"magnetometer_recovery_reason\": \""
            << magnetic_recovery_reason(magnetic_lifecycle.state().recovery_reason) << '"'
            << ",\n  \"magnetometer_max_alignment_us\": " << mag_max_alignment_us
            << ",\n  \"magnetometer_observation_variance\": " << magnetic.V(0U, 0U)
            << ",\n  \"magnetometer_gate_sigma\": " << magnetic.gate_sigma
            << ",\n  \"magnetometer_tilt_protection\": " << (ESKF_MAG_TILT ? "true" : "false")
            << ",\n  \"magnetometer_strength_tolerance_ut\": " << MAG_STRENGTH_TOL
            << ",\n  \"magnetometer_inclination_tolerance_deg\": " << MAG_INCLINATION_TOL
            << ",\n  \"magnetometer_minimum_horizontal_ut\": " << MAG_MIN_HORIZONTAL << "\n}\n";
    summary.close();
    std::cout << "IMU predictions: " << predictions << "\nNavigation events: " << navigation_events
              << "\nGNSS quality: " << (quality_parameters ? "ON" : "OFF (legacy input)") << " (" << quality_accepted
              << " accepted, " << quality_rejected << " rejected, " << quality_waiting << " waiting, "
              << quality_unavailable << " unavailable, " << invalid_velocity << " invalid velocity)"
              << "\nGNSS lifecycle: "
              << (quality_parameters ? "ON (GNSS-only, position and velocity resets)" : "OFF (legacy input)")
              << "\nHorizontal position: " << position_xy_fused << " fused, "
              << position_xy_attempts - position_xy_fused << " gate rejected, " << position_xy_resets << " reset"
              << "\nVertical position: " << position_z_fused << " fused, " << position_z_attempts - position_z_fused
              << " gate rejected, " << position_z_resets << " reset"
              << "\n3D velocity: " << velocity_fused << " fused, " << velocity_attempts - velocity_fused
              << " gate rejected, " << velocity_resets << " reset"
              << "\nDown velocity resets for height recovery: " << vertical_velocity_resets
              << "\nGNSS innovation gate: " << (gnss ? "ON" : "OFF (legacy input)")
              << "\nCSV interval gap holds: " << gaps << " (" << gap_us
              << " us)\nNavigation maximum tick alignment: " << max_alignment_us
              << " us; receiver latency compensation OFF\n"
              << "Accelerometer observations: " << accel_observations << " (" << accel_corrections << " fused, "
              << accel_rejected << " rejected, " << accel_unavailable << " metadata unavailable)\n"
              << "Accelerometer interval: target " << ACCEL_OBS_PERIOD_US << " us, actual " << accel_interval_min_us
              << ".." << accel_interval_max_us << " us\n"
              << "Magnetometer events: " << mag_events << " (" << mag_corrections << " fused, " << mag_rejected
              << " gate rejected, " << mag_skipped << " lifecycle skipped, " << mag_superseded << " superseded)\n"
              << "Magnetometer heading resets: " << mag_heading_resets << '\n'
              << "Magnetometer lifecycle: " << magnetic_stop_reason(magnetic_lifecycle.state().stop_reason)
              << "; recovery: " << magnetic_recovery_reason(magnetic_lifecycle.state().recovery_reason) << '\n';
}

} /* namespace */

int main(int argc, char ** argv)
{
    if (argc != 3 && !(argc == 4 && std::string_view(argv[3]) == "--no-mag"))
    {
        std::cerr << "Usage: replay_ins INPUT_DIR NEW_OUTPUT_DIR [--no-mag]\n";
        return 2;
    }
    try
    {
        replay(argv[1], argv[2], argc == 3);
    }
    catch (std::exception const & error)
    {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
