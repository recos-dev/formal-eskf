// SPDX-License-Identifier: BSD-3-Clause

#include "contracts.hpp"
#include "delayed_library.hpp"
#include "../../../integrations/px4/module/ImuPrediction.hpp"
#include "../../../integrations/px4/module/DeploymentProfile.hpp"
// Expose the production input predicate only; no owner/buffer body is replaced.
#define private public
#define max formal_eskf_proof_max
#include "../../../integrations/px4/module/ImuHistory.hpp"
#undef max
#undef private

using contract_proof::Scalar;
using Imu = formal_eskf::px4::ImuHistory<contract_proof::Linalg>;
using Input = Imu::input_type;
using TimingParameters = formal_eskf::px4::TimingParameters;
using TimingConfiguration = formal_eskf::px4::TimingConfiguration;
namespace profile = formal_eskf::px4::profile;
static_assert(profile::history::filter_interval_us == 4000 && profile::history::minimum_period_us == 3800U &&
                  profile::history::maximum_period_us == 4200U && profile::history::maximum_lag_us == 300000U &&
                  profile::history::gnss_arrival_age_us == 150000U &&
                  profile::history::magnetic_arrival_age_us == 100000U &&
                  profile::history::barometer_arrival_age_us == 100000U && profile::output::maximum_age_us == 100000U &&
                  profile::history::imu_capacity == 64U,
              "Input/timing oracle pins the implemented 250 Hz deployment profile");

inline bool same_configuration(formal_eskf::px4::TimingConfiguration const & a,
                               formal_eskf::px4::TimingConfiguration const & b)
{
    return a.minimum_lag_us == b.minimum_lag_us && a.gnss_delay_us == b.gnss_delay_us &&
           a.magnetic_delay_us == b.magnetic_delay_us && a.barometer_delay_us == b.barometer_delay_us &&
           a.imu_length == b.imu_length;
}
inline bool same_source(formal_eskf::px4::ImuSource const & a, formal_eskf::px4::ImuSource const & b)
{
    return a.accelerometer_id == b.accelerometer_id && a.gyroscope_id == b.gyroscope_id &&
           a.accelerometer_calibration == b.accelerometer_calibration &&
           a.gyroscope_calibration == b.gyroscope_calibration;
}
inline bool same_input(Input const & a, Input const & b)
{
    return a.time_us == b.time_us && a.angular_period_us == b.angular_period_us &&
           a.acceleration_period_us == b.acceleration_period_us &&
           a.accelerometer_clipping == b.accelerometer_clipping && a.gyroscope_clipping == b.gyroscope_clipping &&
           same_source(a.source, b.source) && contract_proof::same_vector(a.delta_angle_b, b.delta_angle_b) &&
           contract_proof::same_vector(a.delta_velocity_b, b.delta_velocity_b);
}

void verify_timing_configuration(TimingParameters parameters, TimingConfiguration output)
{
    auto const before = output;
    bool const domain = parameters.filter_interval_us == 4000 && parameters.fusion_delay_ms > 0 &&
                        parameters.gnss_delay_ms >= 0 && parameters.magnetic_delay_ms >= 0 &&
                        parameters.barometer_delay_ms >= 0;
    std::uint64_t lag{}, gnss{}, magnetic{}, barometer{}, intervals{};
    bool eligible = false;
    if (domain)
    {
        lag = static_cast<std::uint64_t>(parameters.fusion_delay_ms) * 1000U;
        gnss = static_cast<std::uint64_t>(parameters.gnss_delay_ms) * 1000U;
        magnetic = static_cast<std::uint64_t>(parameters.magnetic_delay_ms) * 1000U;
        barometer = static_cast<std::uint64_t>(parameters.barometer_delay_ms) * 1000U;
        intervals = lag / 3800U + static_cast<std::uint64_t>(lag % 3800U != 0U);
        eligible = lag >= 150000U + gnss + 2100U && lag >= 100000U + magnetic + 2100U &&
                   lag >= 100000U + barometer + 2100U && lag <= 300000U && intervals < 64U;
    }
    bool const accepted = formal_eskf::px4::configure_timing(parameters, output);
    __ESBMC_assert(accepted == eligible,
                   "Delayed timing: exact signed-domain, arrival budget, lag and capacity admission");
    if (accepted)
    {
        __ESBMC_assert(output.minimum_lag_us == lag && output.gnss_delay_us == gnss &&
                           output.magnetic_delay_us == magnetic && output.barometer_delay_us == barometer &&
                           output.imu_length == intervals + 1U,
                       "Delayed timing: complete widened configuration, ceiling interval count plus one");
    }
    else
    {
        __ESBMC_assert(same_configuration(output, before), "Delayed timing: invalid configuration preserves output");
    }
}

bool admitted_input(Input const & sample, std::uint64_t now)
{
    if (sample.time_us == 0U || sample.time_us > now || now - sample.time_us > profile::output::maximum_age_us ||
        sample.angular_period_us < 3800U || sample.angular_period_us > 4200U || sample.acceleration_period_us < 3800U ||
        sample.acceleration_period_us > 4200U || sample.accelerometer_clipping > 7U || sample.gyroscope_clipping != 0U)
    {
        return false;
    }
    Scalar const angle_dt = static_cast<Scalar>(sample.angular_period_us) * static_cast<Scalar>(1e-6);
    Scalar const velocity_dt = static_cast<Scalar>(sample.acceleration_period_us) * static_cast<Scalar>(1e-6);
    bool finite = true;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        finite = finite && std::isfinite(sample.delta_angle_b(axis) / angle_dt) &&
                 std::isfinite(sample.delta_velocity_b(axis) / velocity_dt);
    }
    return finite;
}

void verify_imu_input(Input sample, std::uint64_t now)
{
    auto const before = sample;
    __ESBMC_assert(Imu::valid_input(sample, now) == admitted_input(sample, now) && same_input(sample, before),
                   "IMU history: exact timestamp, period, clipping and finite-rate admission without input mutation");
}

int main() { return 0; }
