// SPDX-License-Identifier: BSD-3-Clause

#include "contracts.hpp"
#include "delayed_library.hpp"
#include "../../../integrations/px4/module/ImuPrediction.hpp"
#include "../../../integrations/px4/module/DeploymentProfile.hpp"

#define private public
#define max formal_eskf_proof_max
#include "../../../integrations/px4/module/ImuHistory.hpp"
#undef max
#undef private

using Backend = contract_proof::Linalg;
using Scalar = contract_proof::Scalar;
using Imu = formal_eskf::px4::ImuHistory<Backend>;
constexpr std::size_t capacity = formal_eskf::px4::profile::history::imu_capacity;
static_assert(capacity == 64);
extern unsigned char nondet_index();

// All valid ring metadata, arbitrary timestamps and payloads. The compact seed
// covers each valid tuple without changing the production field types.
void metadata(Imu & owner)
{
    auto const length = nondet_index();
    auto const count = nondet_index();
    auto const oldest = nondet_index();
    __ESBMC_assume(length <= capacity && count <= length && (length == 0 ? oldest == 0 : oldest < length));
    owner.m_samples.m_length = length;
    owner.m_samples.m_count = count;
    owner.m_samples.m_oldest = oldest;
    auto const sum = static_cast<std::size_t>(oldest) + count;
    owner.m_samples.m_write = sum < length ? sum : sum - length;
}

// Homogeneous scalar snapshots keep the oracle independent of conditional
// aggregate-reference binding. Every physical slot and every field is checked.
// Integer widening is lossless; floating comparisons retain signed zero and
// NaN classification rather than claiming NaN-payload identity.
enum IntegerField : std::size_t
{
    interval_time,
    interval_start,
    interval_hold,
    input_time,
    angular_period,
    acceleration_period,
    accelerometer_clipping,
    gyroscope_clipping,
    accelerometer_id,
    gyroscope_id,
    accelerometer_calibration,
    gyroscope_calibration,
    integer_fields
};
enum VectorOffset : std::size_t
{
    angle = 0,
    velocity = 3,
    force = 6,
    rate = 9,
    coefficients = 12
};

bool same_input(Imu::input_type const & a, Imu::input_type const & b);

namespace
{
bool admission_result;
unsigned admission_calls;
Imu::input_type const * expected_input;
Imu::input_type const * original_input;
std::uint64_t expected_now;
} // namespace

// Admission may succeed or fail. Its actual coefficient/status/input-frame
// contract is separately checked by delayed-input for the same scalar format.
template <> bool formal_eskf::px4::ImuHistory<Backend>::valid_input(input_type const & sample, std::uint64_t now)
{
    __ESBMC_assert(&sample == expected_input && same_input(sample, *original_input) && now == expected_now,
                   "seed forwards complete input and time to admission producer");
    ++admission_calls;
    return admission_result;
}

bool same_input(Imu::input_type const & a, Imu::input_type const & b)
{
    return a.time_us == b.time_us && a.angular_period_us == b.angular_period_us &&
           a.acceleration_period_us == b.acceleration_period_us &&
           a.accelerometer_clipping == b.accelerometer_clipping && a.gyroscope_clipping == b.gyroscope_clipping &&
           a.source.accelerometer_id == b.source.accelerometer_id && a.source.gyroscope_id == b.source.gyroscope_id &&
           a.source.accelerometer_calibration == b.source.accelerometer_calibration &&
           a.source.gyroscope_calibration == b.source.gyroscope_calibration &&
           contract_proof::same_vector(a.delta_angle_b, b.delta_angle_b) &&
           contract_proof::same_vector(a.delta_velocity_b, b.delta_velocity_b);
}

void verify_imu_seed(Imu owner, Imu::input_type sample, std::uint64_t now, bool admitted)
{
    metadata(owner);
    std::uint64_t integers[capacity][integer_fields];
    Scalar values[capacity][coefficients];
    for (std::size_t index = 0; index < capacity; ++index)
    {
        integers[index][interval_time] = owner.m_samples.m_samples[index].time_us;
        integers[index][interval_start] = owner.m_samples.m_samples[index].start_time_us;
        integers[index][interval_hold] = owner.m_samples.m_samples[index].hold_period_us;
        integers[index][input_time] = owner.m_samples.m_samples[index].sample.time_us;
        integers[index][angular_period] = owner.m_samples.m_samples[index].sample.angular_period_us;
        integers[index][acceleration_period] = owner.m_samples.m_samples[index].sample.acceleration_period_us;
        integers[index][accelerometer_clipping] = owner.m_samples.m_samples[index].sample.accelerometer_clipping;
        integers[index][gyroscope_clipping] = owner.m_samples.m_samples[index].sample.gyroscope_clipping;
        integers[index][accelerometer_id] = owner.m_samples.m_samples[index].sample.source.accelerometer_id;
        integers[index][gyroscope_id] = owner.m_samples.m_samples[index].sample.source.gyroscope_id;
        integers[index][accelerometer_calibration] =
            owner.m_samples.m_samples[index].sample.source.accelerometer_calibration;
        integers[index][gyroscope_calibration] = owner.m_samples.m_samples[index].sample.source.gyroscope_calibration;
        values[index][angle + 0] = owner.m_samples.m_samples[index].sample.delta_angle_b(0);
        values[index][angle + 1] = owner.m_samples.m_samples[index].sample.delta_angle_b(1);
        values[index][angle + 2] = owner.m_samples.m_samples[index].sample.delta_angle_b(2);
        values[index][velocity + 0] = owner.m_samples.m_samples[index].sample.delta_velocity_b(0);
        values[index][velocity + 1] = owner.m_samples.m_samples[index].sample.delta_velocity_b(1);
        values[index][velocity + 2] = owner.m_samples.m_samples[index].sample.delta_velocity_b(2);
        values[index][force + 0] = owner.m_samples.m_samples[index].held_rate.specific_force_b(0);
        values[index][force + 1] = owner.m_samples.m_samples[index].held_rate.specific_force_b(1);
        values[index][force + 2] = owner.m_samples.m_samples[index].held_rate.specific_force_b(2);
        values[index][rate + 0] = owner.m_samples.m_samples[index].held_rate.angular_rate_b(0);
        values[index][rate + 1] = owner.m_samples.m_samples[index].held_rate.angular_rate_b(1);
        values[index][rate + 2] = owner.m_samples.m_samples[index].held_rate.angular_rate_b(2);
    }
    auto const previous = owner.m_previous;
    auto const input = sample;
    auto const config = owner.m_configuration;
    auto const count = owner.m_samples.m_count;
    auto const length = owner.m_samples.m_length;
    auto const oldest = owner.m_samples.m_oldest;
    auto const write = owner.m_samples.m_write;
    auto const fault = owner.m_fault;
    auto const fusion = owner.m_fusion_time_us;
    admission_calls = 0;
    admission_result = admitted;
    expected_input = &sample;
    original_input = &input;
    expected_now = now;
    bool const calls = length != 0 && previous.time_us == 0 && fault == Imu::Fault::none;
    bool const result = owner.seed(sample, now);
    __ESBMC_assert(result == (calls && admitted) && admission_calls == (calls ? 1U : 0U),
                   "seed decision and admission producer call order");
    for (std::size_t index = 0; index < capacity; ++index)
    {
        __ESBMC_assert(owner.m_samples.m_samples[index].time_us == integers[index][interval_time],
                       "seed preserves time_us");
        __ESBMC_assert(owner.m_samples.m_samples[index].start_time_us == integers[index][interval_start],
                       "seed preserves start_time_us");
        __ESBMC_assert(owner.m_samples.m_samples[index].hold_period_us == integers[index][interval_hold],
                       "seed preserves hold_period_us");
        __ESBMC_assert(owner.m_samples.m_samples[index].sample.time_us == integers[index][input_time],
                       "seed preserves sample.time_us");
        __ESBMC_assert(owner.m_samples.m_samples[index].sample.angular_period_us == integers[index][angular_period],
                       "seed preserves sample.angular_period_us");
        __ESBMC_assert(owner.m_samples.m_samples[index].sample.acceleration_period_us ==
                           integers[index][acceleration_period],
                       "seed preserves sample.acceleration_period_us");
        __ESBMC_assert(owner.m_samples.m_samples[index].sample.accelerometer_clipping ==
                           integers[index][accelerometer_clipping],
                       "seed preserves sample.accelerometer_clipping");
        __ESBMC_assert(owner.m_samples.m_samples[index].sample.gyroscope_clipping ==
                           integers[index][gyroscope_clipping],
                       "seed preserves sample.gyroscope_clipping");
        __ESBMC_assert(owner.m_samples.m_samples[index].sample.source.accelerometer_id ==
                           integers[index][accelerometer_id],
                       "seed preserves sample.source.accelerometer_id");
        __ESBMC_assert(owner.m_samples.m_samples[index].sample.source.gyroscope_id == integers[index][gyroscope_id],
                       "seed preserves sample.source.gyroscope_id");
        __ESBMC_assert(owner.m_samples.m_samples[index].sample.source.accelerometer_calibration ==
                           integers[index][accelerometer_calibration],
                       "seed preserves sample.source.accelerometer_calibration");
        __ESBMC_assert(owner.m_samples.m_samples[index].sample.source.gyroscope_calibration ==
                           integers[index][gyroscope_calibration],
                       "seed preserves sample.source.gyroscope_calibration");
        __ESBMC_assert(
            contract_proof::same(owner.m_samples.m_samples[index].sample.delta_angle_b(0), values[index][angle + 0]),
            "seed preserves sample.delta_angle_b coefficient");
        __ESBMC_assert(
            contract_proof::same(owner.m_samples.m_samples[index].sample.delta_angle_b(1), values[index][angle + 1]),
            "seed preserves sample.delta_angle_b coefficient");
        __ESBMC_assert(
            contract_proof::same(owner.m_samples.m_samples[index].sample.delta_angle_b(2), values[index][angle + 2]),
            "seed preserves sample.delta_angle_b coefficient");
        __ESBMC_assert(contract_proof::same(owner.m_samples.m_samples[index].sample.delta_velocity_b(0),
                                            values[index][velocity + 0]),
                       "seed preserves sample.delta_velocity_b coefficient");
        __ESBMC_assert(contract_proof::same(owner.m_samples.m_samples[index].sample.delta_velocity_b(1),
                                            values[index][velocity + 1]),
                       "seed preserves sample.delta_velocity_b coefficient");
        __ESBMC_assert(contract_proof::same(owner.m_samples.m_samples[index].sample.delta_velocity_b(2),
                                            values[index][velocity + 2]),
                       "seed preserves sample.delta_velocity_b coefficient");
        __ESBMC_assert(contract_proof::same(owner.m_samples.m_samples[index].held_rate.specific_force_b(0),
                                            values[index][force + 0]),
                       "seed preserves held_rate.specific_force_b coefficient");
        __ESBMC_assert(contract_proof::same(owner.m_samples.m_samples[index].held_rate.specific_force_b(1),
                                            values[index][force + 1]),
                       "seed preserves held_rate.specific_force_b coefficient");
        __ESBMC_assert(contract_proof::same(owner.m_samples.m_samples[index].held_rate.specific_force_b(2),
                                            values[index][force + 2]),
                       "seed preserves held_rate.specific_force_b coefficient");
        __ESBMC_assert(
            contract_proof::same(owner.m_samples.m_samples[index].held_rate.angular_rate_b(0), values[index][rate + 0]),
            "seed preserves held_rate.angular_rate_b coefficient");
        __ESBMC_assert(
            contract_proof::same(owner.m_samples.m_samples[index].held_rate.angular_rate_b(1), values[index][rate + 1]),
            "seed preserves held_rate.angular_rate_b coefficient");
        __ESBMC_assert(
            contract_proof::same(owner.m_samples.m_samples[index].held_rate.angular_rate_b(2), values[index][rate + 2]),
            "seed preserves held_rate.angular_rate_b coefficient");
    }
    // Explicit branches avoid the verifier's conditional aggregate-reference limitation.
    if (result)
    {
        __ESBMC_assert(same_input(owner.m_previous, input), "seed publishes the complete admitted input");
    }
    else
    {
        __ESBMC_assert(same_input(owner.m_previous, previous), "rejected seed preserves the complete previous input");
    }
    __ESBMC_assert(same_input(sample, input), "seed preserves its caller input");
    __ESBMC_assert(owner.m_configuration.minimum_lag_us == config.minimum_lag_us &&
                       owner.m_configuration.gnss_delay_us == config.gnss_delay_us &&
                       owner.m_configuration.magnetic_delay_us == config.magnetic_delay_us &&
                       owner.m_configuration.barometer_delay_us == config.barometer_delay_us &&
                       owner.m_configuration.imu_length == config.imu_length,
                   "seed configuration frame");
    __ESBMC_assert(owner.m_samples.m_length == length && owner.m_samples.m_count == count &&
                       owner.m_samples.m_oldest == oldest && owner.m_samples.m_write == write &&
                       owner.m_fault == fault && owner.m_fusion_time_us == (result ? sample.time_us : fusion),
                   "seed metadata and completion watermark");
}

bool input_is_zero(Imu::input_type const & input)
{
    if (input.time_us != 0 || input.angular_period_us != 0 || input.acceleration_period_us != 0 ||
        input.accelerometer_clipping != 0 || input.gyroscope_clipping != 0 || input.source.accelerometer_id != 0 ||
        input.source.gyroscope_id != 0 || input.source.accelerometer_calibration != 0 ||
        input.source.gyroscope_calibration != 0)
    {
        return false;
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        if (!contract_proof::same(input.delta_angle_b(axis), Scalar{0}) ||
            !contract_proof::same(input.delta_velocity_b(axis), Scalar{0}))
        {
            return false;
        }
    }
    return true;
}

void verify_imu_initialization()
{
    Imu owner;
    __ESBMC_assert(owner.m_samples.m_length == 0 && owner.m_samples.m_count == 0 && owner.m_samples.m_oldest == 0 &&
                       owner.m_samples.m_write == 0 && owner.m_fault == Imu::Fault::none &&
                       owner.m_fusion_time_us == 0 && input_is_zero(owner.m_previous),
                   "IMU construction establishes the unconfigured empty session");
    __ESBMC_assert(owner.m_configuration.minimum_lag_us == 0 && owner.m_configuration.gnss_delay_us == 0 &&
                       owner.m_configuration.magnetic_delay_us == 0 && owner.m_configuration.barometer_delay_us == 0 &&
                       owner.m_configuration.imu_length == 0,
                   "IMU construction initializes every configuration field");
    for (std::size_t index = 0; index < capacity; ++index)
    {
        auto const & interval = owner.m_samples.m_samples[index];
        __ESBMC_assert(interval.time_us == 0 && interval.start_time_us == 0 && interval.hold_period_us == 0 &&
                           input_is_zero(interval.sample),
                       "IMU construction initializes every retained interval and sample");
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            __ESBMC_assert(contract_proof::same(interval.held_rate.specific_force_b(axis), Scalar{0}) &&
                               contract_proof::same(interval.held_rate.angular_rate_b(axis), Scalar{0}),
                           "IMU construction initializes every held-rate coefficient");
        }
    }
}
