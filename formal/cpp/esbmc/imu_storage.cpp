// SPDX-License-Identifier: BSD-3-Clause

// Observe fixed-array scalar fields independently of the production accessors.
#define private public
#include <formal_eskf/linalg/matrix.hpp>
#undef private
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
static_assert(FORMAL_ESKF_PROOF_WRITE < capacity);
extern unsigned char nondet_index();

// Every valid tuple with this write position is represented. The runner must
// execute every position, in both formats, before accepting the complete frame.
template <typename Sample, std::size_t Capacity>
void metadata(formal_eskf::runtime::RingBuffer<Sample, Capacity> & ring)
{
    auto const length = nondet_index();
    auto const count = nondet_index();
    auto const oldest = nondet_index();
    __ESBMC_assume(length <= Capacity && count <= length && (length == 0 ? oldest == 0 : oldest < length));
    ring.m_length = length;
    ring.m_count = count;
    ring.m_oldest = oldest;
    auto const sum = static_cast<std::size_t>(oldest) + count;
    ring.m_write = sum < length ? sum : sum - length;
}

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

bool same_stored_scalar(Scalar a, Scalar b)
{
#if FORMAL_ESKF_PROOF_BINARY64
    using Bits = std::uint64_t;
#else
    using Bits = std::uint32_t;
#endif
    // Pure predicates: eager Boolean operators avoid path merges in the oracle.
    return (__builtin_bit_cast(Bits, a) == __builtin_bit_cast(Bits, b)) | (std::isnan(a) & std::isnan(b));
}
using TypedRing = formal_eskf::runtime::RingBuffer<Imu::Interval, capacity>;

// The actual ring and sample type are retained. Arbitrary payloads include all
// inactive slots and IEEE values; no chronological or finite-value premise is
// needed. The external source does not alias the ring, as in ImuHistory::push.
// Fixed destination partitions avoid dynamic aggregate-pointer expansion in
// ESBMC. Their exhaustive union covers all valid writes, without reducing storage.
// Homogeneous snapshots avoid copying nested aggregate arrays in the oracle.

void verify_imu_storage(TypedRing ring, Imu::Interval sample)
{
    metadata(ring);
    auto const expected_write = ring.m_write;
    ring.m_write = FORMAL_ESKF_PROOF_WRITE;
    __ESBMC_assume(expected_write == ring.m_write);
    __ESBMC_assume(ring.m_length > 0);
    std::uint64_t integers[capacity][integer_fields];
    std::uint64_t source_integers[integer_fields];
    Scalar values[capacity][coefficients];
    Scalar source_values[coefficients];
    source_integers[interval_time] = sample.time_us;
    source_integers[interval_start] = sample.start_time_us;
    source_integers[interval_hold] = sample.hold_period_us;
    source_integers[input_time] = sample.sample.time_us;
    source_integers[angular_period] = sample.sample.angular_period_us;
    source_integers[acceleration_period] = sample.sample.acceleration_period_us;
    source_integers[accelerometer_clipping] = sample.sample.accelerometer_clipping;
    source_integers[gyroscope_clipping] = sample.sample.gyroscope_clipping;
    source_integers[accelerometer_id] = sample.sample.source.accelerometer_id;
    source_integers[gyroscope_id] = sample.sample.source.gyroscope_id;
    source_integers[accelerometer_calibration] = sample.sample.source.accelerometer_calibration;
    source_integers[gyroscope_calibration] = sample.sample.source.gyroscope_calibration;
    source_values[angle + 0] = sample.sample.delta_angle_b.m_storage.values.head;
    source_values[angle + 1] = sample.sample.delta_angle_b.m_storage.values.tail.head;
    source_values[angle + 2] = sample.sample.delta_angle_b.m_storage.values.tail.tail.head;
    source_values[velocity + 0] = sample.sample.delta_velocity_b.m_storage.values.head;
    source_values[velocity + 1] = sample.sample.delta_velocity_b.m_storage.values.tail.head;
    source_values[velocity + 2] = sample.sample.delta_velocity_b.m_storage.values.tail.tail.head;
    source_values[force + 0] = sample.held_rate.specific_force_b.m_storage.values.head;
    source_values[force + 1] = sample.held_rate.specific_force_b.m_storage.values.tail.head;
    source_values[force + 2] = sample.held_rate.specific_force_b.m_storage.values.tail.tail.head;
    source_values[rate + 0] = sample.held_rate.angular_rate_b.m_storage.values.head;
    source_values[rate + 1] = sample.held_rate.angular_rate_b.m_storage.values.tail.head;
    source_values[rate + 2] = sample.held_rate.angular_rate_b.m_storage.values.tail.tail.head;
    for (std::size_t index = 0; index < capacity; ++index)
    {
        integers[index][interval_time] = ring.m_samples[index].time_us;
        integers[index][interval_start] = ring.m_samples[index].start_time_us;
        integers[index][interval_hold] = ring.m_samples[index].hold_period_us;
        integers[index][input_time] = ring.m_samples[index].sample.time_us;
        integers[index][angular_period] = ring.m_samples[index].sample.angular_period_us;
        integers[index][acceleration_period] = ring.m_samples[index].sample.acceleration_period_us;
        integers[index][accelerometer_clipping] = ring.m_samples[index].sample.accelerometer_clipping;
        integers[index][gyroscope_clipping] = ring.m_samples[index].sample.gyroscope_clipping;
        integers[index][accelerometer_id] = ring.m_samples[index].sample.source.accelerometer_id;
        integers[index][gyroscope_id] = ring.m_samples[index].sample.source.gyroscope_id;
        integers[index][accelerometer_calibration] = ring.m_samples[index].sample.source.accelerometer_calibration;
        integers[index][gyroscope_calibration] = ring.m_samples[index].sample.source.gyroscope_calibration;
        values[index][angle + 0] = ring.m_samples[index].sample.delta_angle_b.m_storage.values.head;
        values[index][angle + 1] = ring.m_samples[index].sample.delta_angle_b.m_storage.values.tail.head;
        values[index][angle + 2] = ring.m_samples[index].sample.delta_angle_b.m_storage.values.tail.tail.head;
        values[index][velocity + 0] = ring.m_samples[index].sample.delta_velocity_b.m_storage.values.head;
        values[index][velocity + 1] = ring.m_samples[index].sample.delta_velocity_b.m_storage.values.tail.head;
        values[index][velocity + 2] = ring.m_samples[index].sample.delta_velocity_b.m_storage.values.tail.tail.head;
        values[index][force + 0] = ring.m_samples[index].held_rate.specific_force_b.m_storage.values.head;
        values[index][force + 1] = ring.m_samples[index].held_rate.specific_force_b.m_storage.values.tail.head;
        values[index][force + 2] = ring.m_samples[index].held_rate.specific_force_b.m_storage.values.tail.tail.head;
        values[index][rate + 0] = ring.m_samples[index].held_rate.angular_rate_b.m_storage.values.head;
        values[index][rate + 1] = ring.m_samples[index].held_rate.angular_rate_b.m_storage.values.tail.head;
        values[index][rate + 2] = ring.m_samples[index].held_rate.angular_rate_b.m_storage.values.tail.tail.head;
    }
    auto const write = ring.m_write;
    auto const count = ring.m_count;
    auto const length = ring.m_length;
    auto const oldest = ring.m_oldest;
    ring.push(sample);
    for (std::size_t index = 0; index < capacity; ++index)
    {
        __ESBMC_assert(ring.m_samples[index].time_us ==
                           (index == write ? source_integers[interval_time] : integers[index][interval_time]),
                       "typed push frame time_us");
        __ESBMC_assert(ring.m_samples[index].start_time_us ==
                           (index == write ? source_integers[interval_start] : integers[index][interval_start]),
                       "typed push frame start_time_us");
        __ESBMC_assert(ring.m_samples[index].hold_period_us ==
                           (index == write ? source_integers[interval_hold] : integers[index][interval_hold]),
                       "typed push frame hold_period_us");
        __ESBMC_assert(ring.m_samples[index].sample.time_us ==
                           (index == write ? source_integers[input_time] : integers[index][input_time]),
                       "typed push frame sample.time_us");
        __ESBMC_assert(ring.m_samples[index].sample.angular_period_us ==
                           (index == write ? source_integers[angular_period] : integers[index][angular_period]),
                       "typed push frame sample.angular_period_us");
        __ESBMC_assert(
            ring.m_samples[index].sample.acceleration_period_us ==
                (index == write ? source_integers[acceleration_period] : integers[index][acceleration_period]),
            "typed push frame sample.acceleration_period_us");
        __ESBMC_assert(
            ring.m_samples[index].sample.accelerometer_clipping ==
                (index == write ? source_integers[accelerometer_clipping] : integers[index][accelerometer_clipping]),
            "typed push frame sample.accelerometer_clipping");
        __ESBMC_assert(ring.m_samples[index].sample.gyroscope_clipping ==
                           (index == write ? source_integers[gyroscope_clipping] : integers[index][gyroscope_clipping]),
                       "typed push frame sample.gyroscope_clipping");
        __ESBMC_assert(ring.m_samples[index].sample.source.accelerometer_id ==
                           (index == write ? source_integers[accelerometer_id] : integers[index][accelerometer_id]),
                       "typed push frame sample.source.accelerometer_id");
        __ESBMC_assert(ring.m_samples[index].sample.source.gyroscope_id ==
                           (index == write ? source_integers[gyroscope_id] : integers[index][gyroscope_id]),
                       "typed push frame sample.source.gyroscope_id");
        __ESBMC_assert(ring.m_samples[index].sample.source.accelerometer_calibration ==
                           (index == write ? source_integers[accelerometer_calibration]
                                           : integers[index][accelerometer_calibration]),
                       "typed push frame sample.source.accelerometer_calibration");
        __ESBMC_assert(
            ring.m_samples[index].sample.source.gyroscope_calibration ==
                (index == write ? source_integers[gyroscope_calibration] : integers[index][gyroscope_calibration]),
            "typed push frame sample.source.gyroscope_calibration");
        __ESBMC_assert(same_stored_scalar(ring.m_samples[index].sample.delta_angle_b.m_storage.values.head,
                                          index == write ? source_values[angle + 0] : values[index][angle + 0]),
                       "typed push frame sample.delta_angle_b coefficient");
        __ESBMC_assert(same_stored_scalar(ring.m_samples[index].sample.delta_angle_b.m_storage.values.tail.head,
                                          index == write ? source_values[angle + 1] : values[index][angle + 1]),
                       "typed push frame sample.delta_angle_b coefficient");
        __ESBMC_assert(same_stored_scalar(ring.m_samples[index].sample.delta_angle_b.m_storage.values.tail.tail.head,
                                          index == write ? source_values[angle + 2] : values[index][angle + 2]),
                       "typed push frame sample.delta_angle_b coefficient");
        __ESBMC_assert(same_stored_scalar(ring.m_samples[index].sample.delta_velocity_b.m_storage.values.head,
                                          index == write ? source_values[velocity + 0] : values[index][velocity + 0]),
                       "typed push frame sample.delta_velocity_b coefficient");
        __ESBMC_assert(same_stored_scalar(ring.m_samples[index].sample.delta_velocity_b.m_storage.values.tail.head,
                                          index == write ? source_values[velocity + 1] : values[index][velocity + 1]),
                       "typed push frame sample.delta_velocity_b coefficient");
        __ESBMC_assert(same_stored_scalar(ring.m_samples[index].sample.delta_velocity_b.m_storage.values.tail.tail.head,
                                          index == write ? source_values[velocity + 2] : values[index][velocity + 2]),
                       "typed push frame sample.delta_velocity_b coefficient");
        __ESBMC_assert(same_stored_scalar(ring.m_samples[index].held_rate.specific_force_b.m_storage.values.head,
                                          index == write ? source_values[force + 0] : values[index][force + 0]),
                       "typed push frame held_rate.specific_force_b coefficient");
        __ESBMC_assert(same_stored_scalar(ring.m_samples[index].held_rate.specific_force_b.m_storage.values.tail.head,
                                          index == write ? source_values[force + 1] : values[index][force + 1]),
                       "typed push frame held_rate.specific_force_b coefficient");
        __ESBMC_assert(
            same_stored_scalar(ring.m_samples[index].held_rate.specific_force_b.m_storage.values.tail.tail.head,
                               index == write ? source_values[force + 2] : values[index][force + 2]),
            "typed push frame held_rate.specific_force_b coefficient");
        __ESBMC_assert(same_stored_scalar(ring.m_samples[index].held_rate.angular_rate_b.m_storage.values.head,
                                          index == write ? source_values[rate + 0] : values[index][rate + 0]),
                       "typed push frame held_rate.angular_rate_b coefficient");
        __ESBMC_assert(same_stored_scalar(ring.m_samples[index].held_rate.angular_rate_b.m_storage.values.tail.head,
                                          index == write ? source_values[rate + 1] : values[index][rate + 1]),
                       "typed push frame held_rate.angular_rate_b coefficient");
        __ESBMC_assert(
            same_stored_scalar(ring.m_samples[index].held_rate.angular_rate_b.m_storage.values.tail.tail.head,
                               index == write ? source_values[rate + 2] : values[index][rate + 2]),
            "typed push frame held_rate.angular_rate_b coefficient");
    }
    __ESBMC_assert(sample.time_us == source_integers[interval_time], "typed push preserves source time_us");
    __ESBMC_assert(sample.start_time_us == source_integers[interval_start],
                   "typed push preserves source start_time_us");
    __ESBMC_assert(sample.hold_period_us == source_integers[interval_hold],
                   "typed push preserves source hold_period_us");
    __ESBMC_assert(sample.sample.time_us == source_integers[input_time], "typed push preserves source sample.time_us");
    __ESBMC_assert(sample.sample.angular_period_us == source_integers[angular_period],
                   "typed push preserves source sample.angular_period_us");
    __ESBMC_assert(sample.sample.acceleration_period_us == source_integers[acceleration_period],
                   "typed push preserves source sample.acceleration_period_us");
    __ESBMC_assert(sample.sample.accelerometer_clipping == source_integers[accelerometer_clipping],
                   "typed push preserves source sample.accelerometer_clipping");
    __ESBMC_assert(sample.sample.gyroscope_clipping == source_integers[gyroscope_clipping],
                   "typed push preserves source sample.gyroscope_clipping");
    __ESBMC_assert(sample.sample.source.accelerometer_id == source_integers[accelerometer_id],
                   "typed push preserves source sample.source.accelerometer_id");
    __ESBMC_assert(sample.sample.source.gyroscope_id == source_integers[gyroscope_id],
                   "typed push preserves source sample.source.gyroscope_id");
    __ESBMC_assert(sample.sample.source.accelerometer_calibration == source_integers[accelerometer_calibration],
                   "typed push preserves source sample.source.accelerometer_calibration");
    __ESBMC_assert(sample.sample.source.gyroscope_calibration == source_integers[gyroscope_calibration],
                   "typed push preserves source sample.source.gyroscope_calibration");
    __ESBMC_assert(same_stored_scalar(sample.sample.delta_angle_b.m_storage.values.head, source_values[angle + 0]),
                   "typed push preserves source coefficient");
    __ESBMC_assert(same_stored_scalar(sample.sample.delta_angle_b.m_storage.values.tail.head, source_values[angle + 1]),
                   "typed push preserves source coefficient");
    __ESBMC_assert(
        same_stored_scalar(sample.sample.delta_angle_b.m_storage.values.tail.tail.head, source_values[angle + 2]),
        "typed push preserves source coefficient");
    __ESBMC_assert(
        same_stored_scalar(sample.sample.delta_velocity_b.m_storage.values.head, source_values[velocity + 0]),
        "typed push preserves source coefficient");
    __ESBMC_assert(
        same_stored_scalar(sample.sample.delta_velocity_b.m_storage.values.tail.head, source_values[velocity + 1]),
        "typed push preserves source coefficient");
    __ESBMC_assert(
        same_stored_scalar(sample.sample.delta_velocity_b.m_storage.values.tail.tail.head, source_values[velocity + 2]),
        "typed push preserves source coefficient");
    __ESBMC_assert(
        same_stored_scalar(sample.held_rate.specific_force_b.m_storage.values.head, source_values[force + 0]),
        "typed push preserves source coefficient");
    __ESBMC_assert(
        same_stored_scalar(sample.held_rate.specific_force_b.m_storage.values.tail.head, source_values[force + 1]),
        "typed push preserves source coefficient");
    __ESBMC_assert(
        same_stored_scalar(sample.held_rate.specific_force_b.m_storage.values.tail.tail.head, source_values[force + 2]),
        "typed push preserves source coefficient");
    __ESBMC_assert(same_stored_scalar(sample.held_rate.angular_rate_b.m_storage.values.head, source_values[rate + 0]),
                   "typed push preserves source coefficient");
    __ESBMC_assert(
        same_stored_scalar(sample.held_rate.angular_rate_b.m_storage.values.tail.head, source_values[rate + 1]),
        "typed push preserves source coefficient");
    __ESBMC_assert(
        same_stored_scalar(sample.held_rate.angular_rate_b.m_storage.values.tail.tail.head, source_values[rate + 2]),
        "typed push preserves source coefficient");
    __ESBMC_assert(ring.m_count == (count < length ? count + 1 : count) && ring.m_length == length &&
                       ring.m_write == (write + 1 == length ? 0 : write + 1) &&
                       ring.m_oldest == (count == length ? (oldest + 1 == length ? 0 : oldest + 1) : oldest),
                   "typed push metadata");
}

void verify_imu_frame_equality(Scalar a, Scalar b)
{
    __ESBMC_assert(same_stored_scalar(a, b) == contract_proof::same(a, b),
                   "stored frame equality retains values, signed zero and NaN classification");
}
