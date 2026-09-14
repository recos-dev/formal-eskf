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
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

#include <formal_eskf/eskf/eskf.hpp>
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

// GNSS standard-deviation floors, shared by initialization and correction.
#define GNSS_POS_XY_STD_MIN 1.0 // m.
#define GNSS_POS_Z_STD_MIN 1.5  // m.
#define GNSS_VEL_STD_MIN 0.3    // m/s.

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

bool read_row(std::istream & stream, std::span<double> output)
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
        if (result.ec != std::errc{} || result.ptr != field.data() + field.size() || !std::isfinite(output[index]))
        {
            throw std::runtime_error("CSV field must be a finite number");
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

Imu imu_sample(std::array<double, 8U> const & row)
{
    return {vector3(row[2U], row[3U], row[4U]), vector3(row[5U], row[6U], row[7U])};
}

double variance(double accuracy, double floor)
{
    if (!(accuracy > 0.0))
    {
        throw std::runtime_error("GNSS accuracy must be positive");
    }
    double const sigma = std::max(accuracy, floor);
    return sigma * sigma;
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

Covariance initial_covariance(double horizontal_accuracy, double vertical_accuracy, double speed_accuracy)
{
    double const position_xy = variance(horizontal_accuracy, GNSS_POS_XY_STD_MIN);
    double const position_z = variance(vertical_accuracy, GNSS_POS_Z_STD_MIN);
    double const velocity = variance(speed_accuracy, GNSS_VEL_STD_MIN);
    // INS error-state order: [delta_p_n, delta_v_n, delta_theta_b, delta_b_a, delta_b_g].
    std::array<double, Types::error_state_dimension> const diagonal{
        position_xy,
        position_xy,
        position_z, // delta_p_n.
        velocity,
        velocity,
        velocity, // delta_v_n.
        INIT_ATTITUDE_XY_VAR,
        INIT_ATTITUDE_XY_VAR,
        INIT_ATTITUDE_Z_VAR, // delta_theta_b.
        INIT_ACCEL_BIAS_VAR,
        INIT_ACCEL_BIAS_VAR,
        INIT_ACCEL_BIAS_VAR, // delta_b_a.
        INIT_GYRO_BIAS_VAR,
        INIT_GYRO_BIAS_VAR,
        INIT_GYRO_BIAS_VAR, // delta_b_g.
    };
    Covariance result;
    for (std::size_t index = 0U; index < diagonal.size(); ++index)
    {
        result.set(index, index, diagonal[index]);
    }
    return result;
}

void correct_gnss(State & state, Covariance & covariance, std::array<double, 10U> const & row, std::uint64_t time)
{
    // Receiver accuracy estimates are approximated as independent axis sigmas
    // with fixed floors (m, m/s). The log does not provide a full joint 6x6 V.
    auto const position = Linalg::vector_type<2U>::from_row_major({row[1U], row[2U]});
    double const horizontal_variance = variance(row[7U], GNSS_POS_XY_STD_MIN);
    auto const position_noise = Linalg::matrix_type<2U, 2U>::identity() * horizontal_variance;
    check_status(formal_eskf::try_correct_horizontal_position(state, covariance, position, position_noise,
                                                              QUAT_MIN_NORM, state, covariance),
                 "horizontal position", time);
    check_status(formal_eskf::try_correct_vertical_position(state, covariance, row[3U],
                                                            variance(row[8U], GNSS_POS_Z_STD_MIN), QUAT_MIN_NORM, state,
                                                            covariance),
                 "vertical position", time);
    auto const velocity_noise = Linalg::matrix_type<3U, 3U>::identity() * variance(row[9U], GNSS_VEL_STD_MIN);
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

void replay(std::filesystem::path const & input, std::filesystem::path const & output)
{
    auto initial = open_csv(input / "initial.csv", "timestamp_us,q0,q1,q2,q3,vn,ve,vd,eph,epv,speed_accuracy");
    auto imu = open_csv(input / "imu.csv", "timestamp_us,integral_dt_us,fx,fy,fz,wx,wy,wz");
    auto gnss = open_csv(input / "gnss.csv", "timestamp_us,pn,pe,pd,vn,ve,vd,eph,epv,speed_accuracy");
    std::array<double, 11U> seed{};
    std::array<double, 8U> imu_row{};
    std::array<double, 10U> gnss_row{};
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
    auto covariance = initial_covariance(horizontal_accuracy, vertical_accuracy, speed_accuracy);
    auto const settings = parameters();
    auto previous_imu = imu_sample(imu_row);
    std::uint64_t last_gnss_time = time;
    bool has_gnss = read_row(gnss, gnss_row);
    if (!std::filesystem::create_directory(output))
    {
        throw std::runtime_error("output directory already exists: " + output.string());
    }
    std::ofstream states(output / "state.csv");
    std::ofstream updates(output / "gnss_updates.csv");
    states.exceptions(std::ios::failbit | std::ios::badbit);
    updates.exceptions(std::ios::failbit | std::ios::badbit);
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
    std::size_t predictions = 0U, corrections = 0U, gaps = 0U;
    std::uint64_t gap_us = 0U, max_alignment_us = 0U;
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
        }
        auto const sample = imu_sample(imu_row);
        check_status(formal_eskf::try_predict(state, covariance, sample, static_cast<double>(period) * 1e-6, settings,
                                              state, covariance),
                     "prediction", next_time);
        time = next_time;
        previous_imu = sample;
        ++predictions;
        check_state(state, covariance);
        // First-version timing approximation: each NEW GNSS event is fused
        // at the following IMU tick. Do not split sampled-noise IMU steps or
        // pretend this compensates receiver latency. Report alignment error.
        while (has_gnss && timestamp(gnss_row[0U]) <= time)
        {
            auto const source_time = timestamp(gnss_row[0U]);
            if (source_time <= last_gnss_time)
            {
                throw std::runtime_error("GNSS timestamps must increase after initialization");
            }
            correct_gnss(state, covariance, gnss_row, time);
            check_state(state, covariance);
            updates << source_time << ',' << time << '\n';
            max_alignment_us = std::max(max_alignment_us, time - source_time);
            last_gnss_time = source_time;
            ++corrections;
            has_gnss = read_row(gnss, gnss_row);
        }
        write_state(states, time, state, covariance);
    }
    if (has_gnss || predictions == 0U || corrections == 0U)
    {
        throw std::runtime_error("replay incomplete: unconsumed GNSS or no prediction/correction steps");
    }
    states.close();
    updates.close();
    std::ofstream summary(output / "summary.json");
    summary.exceptions(std::ios::failbit | std::ios::badbit);
    summary << std::setprecision(17) << "{\n  \"result\": \"smoke_pass\",\n  \"accuracy_validated\": false,"
            << "\n  \"linalg_backend\": \"" << backend_name << "\","
            << "\n  \"duration_s\": " << static_cast<double>(time - start_time) * 1e-6
            << ",\n  \"imu_predictions\": " << predictions << ",\n  \"gnss_events\": " << corrections
            << ",\n  \"horizontal_position_corrections\": " << corrections
            << ",\n  \"vertical_position_corrections\": " << corrections
            << ",\n  \"velocity_corrections\": " << corrections << ",\n  \"imu_gap_holds\": " << gaps
            << ",\n  \"imu_gap_us\": " << gap_us << ",\n  \"gnss_max_alignment_us\": " << max_alignment_us
            << ",\n  \"gnss_delay_compensation_ms\": 0\n}\n";
    summary.close();
    std::cout << "IMU predictions: " << predictions << "\nGNSS events: " << corrections
              << " (horizontal position + vertical position + 3D velocity)\nIMU gap holds: " << gaps << " (" << gap_us
              << " us)\nGNSS maximum tick alignment: " << max_alignment_us
              << " us; receiver latency compensation OFF\n";
}

} /* namespace */

int main(int argc, char ** argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: replay_px4 INPUT_DIR NEW_OUTPUT_DIR\n";
        return 2;
    }
    try
    {
        replay(argv[1], argv[2]);
    }
    catch (std::exception const & error)
    {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
