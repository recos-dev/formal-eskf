// SPDX-License-Identifier: BSD-3-Clause

#include "observation_support.hpp"

using observation_proof::Observations;
using observation_proof::same_double;

// Check every field of the actual retained types. Each push destination is a
// disjoint partition; accepting the typed producer requires all destinations
// in both scalar formats. Payloads and inactive slots remain arbitrary IEEE.
// The owners always allocate their fixed profile capacity and supply an external
// record. General RingBuffer length and borrowed-input contracts are separate.
static_assert(FORMAL_ESKF_PROOF_OBSERVATION_SOURCE < 4);
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
using Record = Observations::GnssObservation;
constexpr std::size_t capacity = 16;
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
using Record = Observations::MagneticObservation;
constexpr std::size_t capacity = 64;
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
using Record = Observations::BarometerObservation;
constexpr std::size_t capacity = 128;
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 3
using Record = Observations::Context;
constexpr std::size_t capacity = 128;
#endif
using Ring = formal_eskf::runtime::RingBuffer<Record, capacity>;

void verify_observation_typed_push(Ring ring, Record source)
{
    static_assert(FORMAL_ESKF_PROOF_WRITE < capacity);
    auto const count = imu_proof::nondet_index();
    auto const oldest = imu_proof::nondet_index();
    __ESBMC_assume(count <= capacity && oldest < capacity);
    ring.m_length = capacity;
    ring.m_count = count;
    ring.m_oldest = oldest;
    ring.m_write = FORMAL_ESKF_PROOF_WRITE;
    auto const sum = static_cast<std::size_t>(oldest) + count;
    __ESBMC_assume(ring.m_write == (sum < capacity ? sum : sum - capacity));
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
    std::uint64_t integers[capacity][5];
    std::uint64_t input_integers[5];
    input_integers[0] = source.time_us;
    input_integers[1] = source.arrival_time_us;
    input_integers[2] = source.residual_delay_us;
    input_integers[3] = source.generation;
    input_integers[4] = source.sample.stamp.time_us;
    imu_proof::Scalar values[capacity][8];
    imu_proof::Scalar input_values[8];
    input_values[0] = source.sample.velocity_n.m_storage.values.head;
    input_values[1] = source.sample.velocity_n.m_storage.values.tail.head;
    input_values[2] = source.sample.velocity_n.m_storage.values.tail.tail.head;
    input_values[3] = source.sample.horizontal_accuracy_m;
    input_values[4] = source.sample.vertical_accuracy_m;
    input_values[5] = source.sample.speed_accuracy_m_s;
    input_values[6] = source.sample.hdop;
    input_values[7] = source.sample.vdop;
    double doubles[capacity][4];
    double input_doubles[4];
    input_doubles[0] = source.sample.latitude_deg;
    input_doubles[1] = source.sample.longitude_deg;
    input_doubles[2] = source.sample.altitude_msl_m;
    input_doubles[3] = source.sample.altitude_ellipsoid_m;
    std::uint32_t small[capacity][2];
    std::uint32_t input_small[2];
    input_small[0] = source.half_period_us;
    input_small[1] = source.sample.stamp.source.device_id;
    std::uint8_t tags[capacity][3];
    std::uint8_t input_tags[3];
    input_tags[0] = source.sample.stamp.source.calibration_count;
    input_tags[1] = source.sample.fix_type;
    input_tags[2] = source.sample.satellites;
    bool flags[capacity][2];
    bool input_flags[2];
    input_flags[0] = source.sample.velocity_valid;
    input_flags[1] = source.sample.spoofed;
    for (std::size_t index = 0; index < capacity; ++index)
    {
        integers[index][0] = ring.m_samples[index].time_us;
        integers[index][1] = ring.m_samples[index].arrival_time_us;
        integers[index][2] = ring.m_samples[index].residual_delay_us;
        integers[index][3] = ring.m_samples[index].generation;
        integers[index][4] = ring.m_samples[index].sample.stamp.time_us;
        values[index][0] = ring.m_samples[index].sample.velocity_n.m_storage.values.head;
        values[index][1] = ring.m_samples[index].sample.velocity_n.m_storage.values.tail.head;
        values[index][2] = ring.m_samples[index].sample.velocity_n.m_storage.values.tail.tail.head;
        values[index][3] = ring.m_samples[index].sample.horizontal_accuracy_m;
        values[index][4] = ring.m_samples[index].sample.vertical_accuracy_m;
        values[index][5] = ring.m_samples[index].sample.speed_accuracy_m_s;
        values[index][6] = ring.m_samples[index].sample.hdop;
        values[index][7] = ring.m_samples[index].sample.vdop;
        doubles[index][0] = ring.m_samples[index].sample.latitude_deg;
        doubles[index][1] = ring.m_samples[index].sample.longitude_deg;
        doubles[index][2] = ring.m_samples[index].sample.altitude_msl_m;
        doubles[index][3] = ring.m_samples[index].sample.altitude_ellipsoid_m;
        small[index][0] = ring.m_samples[index].half_period_us;
        small[index][1] = ring.m_samples[index].sample.stamp.source.device_id;
        tags[index][0] = ring.m_samples[index].sample.stamp.source.calibration_count;
        tags[index][1] = ring.m_samples[index].sample.fix_type;
        tags[index][2] = ring.m_samples[index].sample.satellites;
        flags[index][0] = ring.m_samples[index].sample.velocity_valid;
        flags[index][1] = ring.m_samples[index].sample.spoofed;
    }
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
    std::uint64_t integers[capacity][5];
    std::uint64_t input_integers[5];
    input_integers[0] = source.time_us;
    input_integers[1] = source.arrival_time_us;
    input_integers[2] = source.residual_delay_us;
    input_integers[3] = source.generation;
    input_integers[4] = source.sample.stamp.time_us;
    imu_proof::Scalar values[capacity][3];
    imu_proof::Scalar input_values[3];
    input_values[0] = source.sample.field_b.m_storage.values.head;
    input_values[1] = source.sample.field_b.m_storage.values.tail.head;
    input_values[2] = source.sample.field_b.m_storage.values.tail.tail.head;
    std::uint32_t small[capacity][2];
    std::uint32_t input_small[2];
    input_small[0] = source.half_period_us;
    input_small[1] = source.sample.stamp.source.device_id;
    std::uint8_t tags[capacity][1];
    std::uint8_t input_tags[1];
    input_tags[0] = source.sample.stamp.source.calibration_count;
    for (std::size_t index = 0; index < capacity; ++index)
    {
        integers[index][0] = ring.m_samples[index].time_us;
        integers[index][1] = ring.m_samples[index].arrival_time_us;
        integers[index][2] = ring.m_samples[index].residual_delay_us;
        integers[index][3] = ring.m_samples[index].generation;
        integers[index][4] = ring.m_samples[index].sample.stamp.time_us;
        values[index][0] = ring.m_samples[index].sample.field_b.m_storage.values.head;
        values[index][1] = ring.m_samples[index].sample.field_b.m_storage.values.tail.head;
        values[index][2] = ring.m_samples[index].sample.field_b.m_storage.values.tail.tail.head;
        small[index][0] = ring.m_samples[index].half_period_us;
        small[index][1] = ring.m_samples[index].sample.stamp.source.device_id;
        tags[index][0] = ring.m_samples[index].sample.stamp.source.calibration_count;
    }
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
    std::uint64_t integers[capacity][5];
    std::uint64_t input_integers[5];
    input_integers[0] = source.time_us;
    input_integers[1] = source.arrival_time_us;
    input_integers[2] = source.residual_delay_us;
    input_integers[3] = source.generation;
    input_integers[4] = source.sample.stamp.time_us;
    imu_proof::Scalar values[capacity][1];
    imu_proof::Scalar input_values[1];
    input_values[0] = source.sample.altitude_m;
    std::uint32_t small[capacity][2];
    std::uint32_t input_small[2];
    input_small[0] = source.half_period_us;
    input_small[1] = source.sample.stamp.source.device_id;
    std::uint8_t tags[capacity][1];
    std::uint8_t input_tags[1];
    input_tags[0] = source.sample.stamp.source.calibration_count;
    for (std::size_t index = 0; index < capacity; ++index)
    {
        integers[index][0] = ring.m_samples[index].time_us;
        integers[index][1] = ring.m_samples[index].arrival_time_us;
        integers[index][2] = ring.m_samples[index].residual_delay_us;
        integers[index][3] = ring.m_samples[index].generation;
        integers[index][4] = ring.m_samples[index].sample.stamp.time_us;
        values[index][0] = ring.m_samples[index].sample.altitude_m;
        small[index][0] = ring.m_samples[index].half_period_us;
        small[index][1] = ring.m_samples[index].sample.stamp.source.device_id;
        tags[index][0] = ring.m_samples[index].sample.stamp.source.calibration_count;
    }
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 3
    std::uint64_t integers[capacity][3];
    std::uint64_t input_integers[3];
    input_integers[0] = source.time_us;
    input_integers[1] = source.land.time_us;
    input_integers[2] = source.vehicle.time_us;
    bool flags[capacity][7];
    bool input_flags[7];
    input_flags[0] = source.land.known;
    input_flags[1] = source.land.landed;
    input_flags[2] = source.land.at_rest;
    input_flags[3] = source.land.in_ground_effect;
    input_flags[4] = source.vehicle.known;
    input_flags[5] = source.vehicle.armed;
    input_flags[6] = source.vehicle.fixed_wing;
    for (std::size_t index = 0; index < capacity; ++index)
    {
        integers[index][0] = ring.m_samples[index].time_us;
        integers[index][1] = ring.m_samples[index].land.time_us;
        integers[index][2] = ring.m_samples[index].vehicle.time_us;
        flags[index][0] = ring.m_samples[index].land.known;
        flags[index][1] = ring.m_samples[index].land.landed;
        flags[index][2] = ring.m_samples[index].land.at_rest;
        flags[index][3] = ring.m_samples[index].land.in_ground_effect;
        flags[index][4] = ring.m_samples[index].vehicle.known;
        flags[index][5] = ring.m_samples[index].vehicle.armed;
        flags[index][6] = ring.m_samples[index].vehicle.fixed_wing;
    }
#endif
    ring.push(source);
    constexpr auto write = FORMAL_ESKF_PROOF_WRITE;
    __ESBMC_assert(ring.m_length == capacity && ring.m_write == (write + 1 == capacity ? 0 : write + 1) &&
                       ring.m_count == (count == capacity ? count : count + 1) &&
                       ring.m_oldest == (count == capacity ? (oldest + 1 == capacity ? 0 : oldest + 1) : oldest),
                   "Typed push exact full/nonfull index transition");
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
    for (std::size_t index = 0; index < capacity; ++index)
    {
        __ESBMC_assert(ring.m_samples[index].time_us == (index == write ? input_integers[0] : integers[index][0]),
                       "Typed push frame time_us");
        __ESBMC_assert(ring.m_samples[index].arrival_time_us ==
                           (index == write ? input_integers[1] : integers[index][1]),
                       "Typed push frame arrival_time_us");
        __ESBMC_assert(ring.m_samples[index].residual_delay_us ==
                           (index == write ? input_integers[2] : integers[index][2]),
                       "Typed push frame residual_delay_us");
        __ESBMC_assert(ring.m_samples[index].generation == (index == write ? input_integers[3] : integers[index][3]),
                       "Typed push frame generation");
        __ESBMC_assert(ring.m_samples[index].sample.stamp.time_us ==
                           (index == write ? input_integers[4] : integers[index][4]),
                       "Typed push frame sample.stamp.time_us");
        __ESBMC_assert(imu_proof::same(ring.m_samples[index].sample.velocity_n.m_storage.values.head,
                                       (index == write ? input_values[0] : values[index][0])),
                       "Typed push frame sample.velocity_n.m_storage.values.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[index].sample.velocity_n.m_storage.values.tail.head,
                                       (index == write ? input_values[1] : values[index][1])),
                       "Typed push frame sample.velocity_n.m_storage.values.tail.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[index].sample.velocity_n.m_storage.values.tail.tail.head,
                                       (index == write ? input_values[2] : values[index][2])),
                       "Typed push frame sample.velocity_n.m_storage.values.tail.tail.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[index].sample.horizontal_accuracy_m,
                                       (index == write ? input_values[3] : values[index][3])),
                       "Typed push frame sample.horizontal_accuracy_m");
        __ESBMC_assert(imu_proof::same(ring.m_samples[index].sample.vertical_accuracy_m,
                                       (index == write ? input_values[4] : values[index][4])),
                       "Typed push frame sample.vertical_accuracy_m");
        __ESBMC_assert(imu_proof::same(ring.m_samples[index].sample.speed_accuracy_m_s,
                                       (index == write ? input_values[5] : values[index][5])),
                       "Typed push frame sample.speed_accuracy_m_s");
        __ESBMC_assert(
            imu_proof::same(ring.m_samples[index].sample.hdop, (index == write ? input_values[6] : values[index][6])),
            "Typed push frame sample.hdop");
        __ESBMC_assert(
            imu_proof::same(ring.m_samples[index].sample.vdop, (index == write ? input_values[7] : values[index][7])),
            "Typed push frame sample.vdop");
        __ESBMC_assert(same_double(ring.m_samples[index].sample.latitude_deg,
                                   (index == write ? input_doubles[0] : doubles[index][0])),
                       "Typed push frame sample.latitude_deg");
        __ESBMC_assert(same_double(ring.m_samples[index].sample.longitude_deg,
                                   (index == write ? input_doubles[1] : doubles[index][1])),
                       "Typed push frame sample.longitude_deg");
        __ESBMC_assert(same_double(ring.m_samples[index].sample.altitude_msl_m,
                                   (index == write ? input_doubles[2] : doubles[index][2])),
                       "Typed push frame sample.altitude_msl_m");
        __ESBMC_assert(same_double(ring.m_samples[index].sample.altitude_ellipsoid_m,
                                   (index == write ? input_doubles[3] : doubles[index][3])),
                       "Typed push frame sample.altitude_ellipsoid_m");
        __ESBMC_assert(ring.m_samples[index].half_period_us == (index == write ? input_small[0] : small[index][0]),
                       "Typed push frame half_period_us");
        __ESBMC_assert(ring.m_samples[index].sample.stamp.source.device_id ==
                           (index == write ? input_small[1] : small[index][1]),
                       "Typed push frame sample.stamp.source.device_id");
        __ESBMC_assert(ring.m_samples[index].sample.stamp.source.calibration_count ==
                           (index == write ? input_tags[0] : tags[index][0]),
                       "Typed push frame sample.stamp.source.calibration_count");
        __ESBMC_assert(ring.m_samples[index].sample.fix_type == (index == write ? input_tags[1] : tags[index][1]),
                       "Typed push frame sample.fix_type");
        __ESBMC_assert(ring.m_samples[index].sample.satellites == (index == write ? input_tags[2] : tags[index][2]),
                       "Typed push frame sample.satellites");
        __ESBMC_assert(ring.m_samples[index].sample.velocity_valid ==
                           (index == write ? input_flags[0] : flags[index][0]),
                       "Typed push frame sample.velocity_valid");
        __ESBMC_assert(ring.m_samples[index].sample.spoofed == (index == write ? input_flags[1] : flags[index][1]),
                       "Typed push frame sample.spoofed");
    }
    __ESBMC_assert(source.time_us == input_integers[0], "Typed push input time_us");
    __ESBMC_assert(source.arrival_time_us == input_integers[1], "Typed push input arrival_time_us");
    __ESBMC_assert(source.residual_delay_us == input_integers[2], "Typed push input residual_delay_us");
    __ESBMC_assert(source.generation == input_integers[3], "Typed push input generation");
    __ESBMC_assert(source.sample.stamp.time_us == input_integers[4], "Typed push input sample.stamp.time_us");
    __ESBMC_assert(imu_proof::same(source.sample.velocity_n.m_storage.values.head, input_values[0]),
                   "Typed push input sample.velocity_n.m_storage.values.head");
    __ESBMC_assert(imu_proof::same(source.sample.velocity_n.m_storage.values.tail.head, input_values[1]),
                   "Typed push input sample.velocity_n.m_storage.values.tail.head");
    __ESBMC_assert(imu_proof::same(source.sample.velocity_n.m_storage.values.tail.tail.head, input_values[2]),
                   "Typed push input sample.velocity_n.m_storage.values.tail.tail.head");
    __ESBMC_assert(imu_proof::same(source.sample.horizontal_accuracy_m, input_values[3]),
                   "Typed push input sample.horizontal_accuracy_m");
    __ESBMC_assert(imu_proof::same(source.sample.vertical_accuracy_m, input_values[4]),
                   "Typed push input sample.vertical_accuracy_m");
    __ESBMC_assert(imu_proof::same(source.sample.speed_accuracy_m_s, input_values[5]),
                   "Typed push input sample.speed_accuracy_m_s");
    __ESBMC_assert(imu_proof::same(source.sample.hdop, input_values[6]), "Typed push input sample.hdop");
    __ESBMC_assert(imu_proof::same(source.sample.vdop, input_values[7]), "Typed push input sample.vdop");
    __ESBMC_assert(same_double(source.sample.latitude_deg, input_doubles[0]), "Typed push input sample.latitude_deg");
    __ESBMC_assert(same_double(source.sample.longitude_deg, input_doubles[1]), "Typed push input sample.longitude_deg");
    __ESBMC_assert(same_double(source.sample.altitude_msl_m, input_doubles[2]),
                   "Typed push input sample.altitude_msl_m");
    __ESBMC_assert(same_double(source.sample.altitude_ellipsoid_m, input_doubles[3]),
                   "Typed push input sample.altitude_ellipsoid_m");
    __ESBMC_assert(source.half_period_us == input_small[0], "Typed push input half_period_us");
    __ESBMC_assert(source.sample.stamp.source.device_id == input_small[1],
                   "Typed push input sample.stamp.source.device_id");
    __ESBMC_assert(source.sample.stamp.source.calibration_count == input_tags[0],
                   "Typed push input sample.stamp.source.calibration_count");
    __ESBMC_assert(source.sample.fix_type == input_tags[1], "Typed push input sample.fix_type");
    __ESBMC_assert(source.sample.satellites == input_tags[2], "Typed push input sample.satellites");
    __ESBMC_assert(source.sample.velocity_valid == input_flags[0], "Typed push input sample.velocity_valid");
    __ESBMC_assert(source.sample.spoofed == input_flags[1], "Typed push input sample.spoofed");
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
    for (std::size_t index = 0; index < capacity; ++index)
    {
        __ESBMC_assert(ring.m_samples[index].time_us == (index == write ? input_integers[0] : integers[index][0]),
                       "Typed push frame time_us");
        __ESBMC_assert(ring.m_samples[index].arrival_time_us ==
                           (index == write ? input_integers[1] : integers[index][1]),
                       "Typed push frame arrival_time_us");
        __ESBMC_assert(ring.m_samples[index].residual_delay_us ==
                           (index == write ? input_integers[2] : integers[index][2]),
                       "Typed push frame residual_delay_us");
        __ESBMC_assert(ring.m_samples[index].generation == (index == write ? input_integers[3] : integers[index][3]),
                       "Typed push frame generation");
        __ESBMC_assert(ring.m_samples[index].sample.stamp.time_us ==
                           (index == write ? input_integers[4] : integers[index][4]),
                       "Typed push frame sample.stamp.time_us");
        __ESBMC_assert(imu_proof::same(ring.m_samples[index].sample.field_b.m_storage.values.head,
                                       (index == write ? input_values[0] : values[index][0])),
                       "Typed push frame sample.field_b.m_storage.values.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[index].sample.field_b.m_storage.values.tail.head,
                                       (index == write ? input_values[1] : values[index][1])),
                       "Typed push frame sample.field_b.m_storage.values.tail.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[index].sample.field_b.m_storage.values.tail.tail.head,
                                       (index == write ? input_values[2] : values[index][2])),
                       "Typed push frame sample.field_b.m_storage.values.tail.tail.head");
        __ESBMC_assert(ring.m_samples[index].half_period_us == (index == write ? input_small[0] : small[index][0]),
                       "Typed push frame half_period_us");
        __ESBMC_assert(ring.m_samples[index].sample.stamp.source.device_id ==
                           (index == write ? input_small[1] : small[index][1]),
                       "Typed push frame sample.stamp.source.device_id");
        __ESBMC_assert(ring.m_samples[index].sample.stamp.source.calibration_count ==
                           (index == write ? input_tags[0] : tags[index][0]),
                       "Typed push frame sample.stamp.source.calibration_count");
    }
    __ESBMC_assert(source.time_us == input_integers[0], "Typed push input time_us");
    __ESBMC_assert(source.arrival_time_us == input_integers[1], "Typed push input arrival_time_us");
    __ESBMC_assert(source.residual_delay_us == input_integers[2], "Typed push input residual_delay_us");
    __ESBMC_assert(source.generation == input_integers[3], "Typed push input generation");
    __ESBMC_assert(source.sample.stamp.time_us == input_integers[4], "Typed push input sample.stamp.time_us");
    __ESBMC_assert(imu_proof::same(source.sample.field_b.m_storage.values.head, input_values[0]),
                   "Typed push input sample.field_b.m_storage.values.head");
    __ESBMC_assert(imu_proof::same(source.sample.field_b.m_storage.values.tail.head, input_values[1]),
                   "Typed push input sample.field_b.m_storage.values.tail.head");
    __ESBMC_assert(imu_proof::same(source.sample.field_b.m_storage.values.tail.tail.head, input_values[2]),
                   "Typed push input sample.field_b.m_storage.values.tail.tail.head");
    __ESBMC_assert(source.half_period_us == input_small[0], "Typed push input half_period_us");
    __ESBMC_assert(source.sample.stamp.source.device_id == input_small[1],
                   "Typed push input sample.stamp.source.device_id");
    __ESBMC_assert(source.sample.stamp.source.calibration_count == input_tags[0],
                   "Typed push input sample.stamp.source.calibration_count");
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
    for (std::size_t index = 0; index < capacity; ++index)
    {
        __ESBMC_assert(ring.m_samples[index].time_us == (index == write ? input_integers[0] : integers[index][0]),
                       "Typed push frame time_us");
        __ESBMC_assert(ring.m_samples[index].arrival_time_us ==
                           (index == write ? input_integers[1] : integers[index][1]),
                       "Typed push frame arrival_time_us");
        __ESBMC_assert(ring.m_samples[index].residual_delay_us ==
                           (index == write ? input_integers[2] : integers[index][2]),
                       "Typed push frame residual_delay_us");
        __ESBMC_assert(ring.m_samples[index].generation == (index == write ? input_integers[3] : integers[index][3]),
                       "Typed push frame generation");
        __ESBMC_assert(ring.m_samples[index].sample.stamp.time_us ==
                           (index == write ? input_integers[4] : integers[index][4]),
                       "Typed push frame sample.stamp.time_us");
        __ESBMC_assert(imu_proof::same(ring.m_samples[index].sample.altitude_m,
                                       (index == write ? input_values[0] : values[index][0])),
                       "Typed push frame sample.altitude_m");
        __ESBMC_assert(ring.m_samples[index].half_period_us == (index == write ? input_small[0] : small[index][0]),
                       "Typed push frame half_period_us");
        __ESBMC_assert(ring.m_samples[index].sample.stamp.source.device_id ==
                           (index == write ? input_small[1] : small[index][1]),
                       "Typed push frame sample.stamp.source.device_id");
        __ESBMC_assert(ring.m_samples[index].sample.stamp.source.calibration_count ==
                           (index == write ? input_tags[0] : tags[index][0]),
                       "Typed push frame sample.stamp.source.calibration_count");
    }
    __ESBMC_assert(source.time_us == input_integers[0], "Typed push input time_us");
    __ESBMC_assert(source.arrival_time_us == input_integers[1], "Typed push input arrival_time_us");
    __ESBMC_assert(source.residual_delay_us == input_integers[2], "Typed push input residual_delay_us");
    __ESBMC_assert(source.generation == input_integers[3], "Typed push input generation");
    __ESBMC_assert(source.sample.stamp.time_us == input_integers[4], "Typed push input sample.stamp.time_us");
    __ESBMC_assert(imu_proof::same(source.sample.altitude_m, input_values[0]), "Typed push input sample.altitude_m");
    __ESBMC_assert(source.half_period_us == input_small[0], "Typed push input half_period_us");
    __ESBMC_assert(source.sample.stamp.source.device_id == input_small[1],
                   "Typed push input sample.stamp.source.device_id");
    __ESBMC_assert(source.sample.stamp.source.calibration_count == input_tags[0],
                   "Typed push input sample.stamp.source.calibration_count");
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 3
    for (std::size_t index = 0; index < capacity; ++index)
    {
        __ESBMC_assert(ring.m_samples[index].time_us == (index == write ? input_integers[0] : integers[index][0]),
                       "Typed push frame time_us");
        __ESBMC_assert(ring.m_samples[index].land.time_us == (index == write ? input_integers[1] : integers[index][1]),
                       "Typed push frame land.time_us");
        __ESBMC_assert(ring.m_samples[index].vehicle.time_us ==
                           (index == write ? input_integers[2] : integers[index][2]),
                       "Typed push frame vehicle.time_us");
        __ESBMC_assert(ring.m_samples[index].land.known == (index == write ? input_flags[0] : flags[index][0]),
                       "Typed push frame land.known");
        __ESBMC_assert(ring.m_samples[index].land.landed == (index == write ? input_flags[1] : flags[index][1]),
                       "Typed push frame land.landed");
        __ESBMC_assert(ring.m_samples[index].land.at_rest == (index == write ? input_flags[2] : flags[index][2]),
                       "Typed push frame land.at_rest");
        __ESBMC_assert(ring.m_samples[index].land.in_ground_effect ==
                           (index == write ? input_flags[3] : flags[index][3]),
                       "Typed push frame land.in_ground_effect");
        __ESBMC_assert(ring.m_samples[index].vehicle.known == (index == write ? input_flags[4] : flags[index][4]),
                       "Typed push frame vehicle.known");
        __ESBMC_assert(ring.m_samples[index].vehicle.armed == (index == write ? input_flags[5] : flags[index][5]),
                       "Typed push frame vehicle.armed");
        __ESBMC_assert(ring.m_samples[index].vehicle.fixed_wing == (index == write ? input_flags[6] : flags[index][6]),
                       "Typed push frame vehicle.fixed_wing");
    }
    __ESBMC_assert(source.time_us == input_integers[0], "Typed push input time_us");
    __ESBMC_assert(source.land.time_us == input_integers[1], "Typed push input land.time_us");
    __ESBMC_assert(source.vehicle.time_us == input_integers[2], "Typed push input vehicle.time_us");
    __ESBMC_assert(source.land.known == input_flags[0], "Typed push input land.known");
    __ESBMC_assert(source.land.landed == input_flags[1], "Typed push input land.landed");
    __ESBMC_assert(source.land.at_rest == input_flags[2], "Typed push input land.at_rest");
    __ESBMC_assert(source.land.in_ground_effect == input_flags[3], "Typed push input land.in_ground_effect");
    __ESBMC_assert(source.vehicle.known == input_flags[4], "Typed push input vehicle.known");
    __ESBMC_assert(source.vehicle.armed == input_flags[5], "Typed push input vehicle.armed");
    __ESBMC_assert(source.vehicle.fixed_wing == input_flags[6], "Typed push input vehicle.fixed_wing");
#endif
}

void verify_observation_typed_reset(Ring ring)
{
    auto const length = ring.m_length;
    ring.reset();
    __ESBMC_assert(ring.m_length == length && ring.m_count == 0 && ring.m_oldest == 0 && ring.m_write == 0,
                   "Typed reset retains length and clears indices without preconditions");
    for (std::size_t index = 0; index < capacity; ++index)
    {
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
        __ESBMC_assert(ring.m_samples[index].time_us == 0, "Typed reset zero time_us");
        __ESBMC_assert(ring.m_samples[index].arrival_time_us == 0, "Typed reset zero arrival_time_us");
        __ESBMC_assert(ring.m_samples[index].residual_delay_us == 0, "Typed reset zero residual_delay_us");
        __ESBMC_assert(ring.m_samples[index].generation == 0, "Typed reset zero generation");
        __ESBMC_assert(ring.m_samples[index].sample.stamp.time_us == 0, "Typed reset zero sample.stamp.time_us");
        __ESBMC_assert(
            imu_proof::same(ring.m_samples[index].sample.velocity_n.m_storage.values.head, imu_proof::Scalar{0}),
            "Typed reset zero sample.velocity_n.m_storage.values.head");
        __ESBMC_assert(
            imu_proof::same(ring.m_samples[index].sample.velocity_n.m_storage.values.tail.head, imu_proof::Scalar{0}),
            "Typed reset zero sample.velocity_n.m_storage.values.tail.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[index].sample.velocity_n.m_storage.values.tail.tail.head,
                                       imu_proof::Scalar{0}),
                       "Typed reset zero sample.velocity_n.m_storage.values.tail.tail.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[index].sample.horizontal_accuracy_m, imu_proof::Scalar{0}),
                       "Typed reset zero sample.horizontal_accuracy_m");
        __ESBMC_assert(imu_proof::same(ring.m_samples[index].sample.vertical_accuracy_m, imu_proof::Scalar{0}),
                       "Typed reset zero sample.vertical_accuracy_m");
        __ESBMC_assert(imu_proof::same(ring.m_samples[index].sample.speed_accuracy_m_s, imu_proof::Scalar{0}),
                       "Typed reset zero sample.speed_accuracy_m_s");
        __ESBMC_assert(imu_proof::same(ring.m_samples[index].sample.hdop, imu_proof::Scalar{0}),
                       "Typed reset zero sample.hdop");
        __ESBMC_assert(imu_proof::same(ring.m_samples[index].sample.vdop, imu_proof::Scalar{0}),
                       "Typed reset zero sample.vdop");
        __ESBMC_assert(same_double(ring.m_samples[index].sample.latitude_deg, double{0}),
                       "Typed reset zero sample.latitude_deg");
        __ESBMC_assert(same_double(ring.m_samples[index].sample.longitude_deg, double{0}),
                       "Typed reset zero sample.longitude_deg");
        __ESBMC_assert(same_double(ring.m_samples[index].sample.altitude_msl_m, double{0}),
                       "Typed reset zero sample.altitude_msl_m");
        __ESBMC_assert(same_double(ring.m_samples[index].sample.altitude_ellipsoid_m, double{0}),
                       "Typed reset zero sample.altitude_ellipsoid_m");
        __ESBMC_assert(ring.m_samples[index].half_period_us == std::uint32_t{0}, "Typed reset zero half_period_us");
        __ESBMC_assert(ring.m_samples[index].sample.stamp.source.device_id == std::uint32_t{0},
                       "Typed reset zero sample.stamp.source.device_id");
        __ESBMC_assert(ring.m_samples[index].sample.stamp.source.calibration_count == std::uint8_t{0},
                       "Typed reset zero sample.stamp.source.calibration_count");
        __ESBMC_assert(ring.m_samples[index].sample.fix_type == std::uint8_t{0}, "Typed reset zero sample.fix_type");
        __ESBMC_assert(ring.m_samples[index].sample.satellites == std::uint8_t{0},
                       "Typed reset zero sample.satellites");
        __ESBMC_assert(ring.m_samples[index].sample.velocity_valid == bool{0},
                       "Typed reset zero sample.velocity_valid");
        __ESBMC_assert(ring.m_samples[index].sample.spoofed == bool{0}, "Typed reset zero sample.spoofed");
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
        __ESBMC_assert(ring.m_samples[index].time_us == 0, "Typed reset zero time_us");
        __ESBMC_assert(ring.m_samples[index].arrival_time_us == 0, "Typed reset zero arrival_time_us");
        __ESBMC_assert(ring.m_samples[index].residual_delay_us == 0, "Typed reset zero residual_delay_us");
        __ESBMC_assert(ring.m_samples[index].generation == 0, "Typed reset zero generation");
        __ESBMC_assert(ring.m_samples[index].sample.stamp.time_us == 0, "Typed reset zero sample.stamp.time_us");
        __ESBMC_assert(
            imu_proof::same(ring.m_samples[index].sample.field_b.m_storage.values.head, imu_proof::Scalar{0}),
            "Typed reset zero sample.field_b.m_storage.values.head");
        __ESBMC_assert(
            imu_proof::same(ring.m_samples[index].sample.field_b.m_storage.values.tail.head, imu_proof::Scalar{0}),
            "Typed reset zero sample.field_b.m_storage.values.tail.head");
        __ESBMC_assert(
            imu_proof::same(ring.m_samples[index].sample.field_b.m_storage.values.tail.tail.head, imu_proof::Scalar{0}),
            "Typed reset zero sample.field_b.m_storage.values.tail.tail.head");
        __ESBMC_assert(ring.m_samples[index].half_period_us == std::uint32_t{0}, "Typed reset zero half_period_us");
        __ESBMC_assert(ring.m_samples[index].sample.stamp.source.device_id == std::uint32_t{0},
                       "Typed reset zero sample.stamp.source.device_id");
        __ESBMC_assert(ring.m_samples[index].sample.stamp.source.calibration_count == std::uint8_t{0},
                       "Typed reset zero sample.stamp.source.calibration_count");
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
        __ESBMC_assert(ring.m_samples[index].time_us == 0, "Typed reset zero time_us");
        __ESBMC_assert(ring.m_samples[index].arrival_time_us == 0, "Typed reset zero arrival_time_us");
        __ESBMC_assert(ring.m_samples[index].residual_delay_us == 0, "Typed reset zero residual_delay_us");
        __ESBMC_assert(ring.m_samples[index].generation == 0, "Typed reset zero generation");
        __ESBMC_assert(ring.m_samples[index].sample.stamp.time_us == 0, "Typed reset zero sample.stamp.time_us");
        __ESBMC_assert(imu_proof::same(ring.m_samples[index].sample.altitude_m, imu_proof::Scalar{0}),
                       "Typed reset zero sample.altitude_m");
        __ESBMC_assert(ring.m_samples[index].half_period_us == std::uint32_t{0}, "Typed reset zero half_period_us");
        __ESBMC_assert(ring.m_samples[index].sample.stamp.source.device_id == std::uint32_t{0},
                       "Typed reset zero sample.stamp.source.device_id");
        __ESBMC_assert(ring.m_samples[index].sample.stamp.source.calibration_count == std::uint8_t{0},
                       "Typed reset zero sample.stamp.source.calibration_count");
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 3
        __ESBMC_assert(ring.m_samples[index].time_us == 0, "Typed reset zero time_us");
        __ESBMC_assert(ring.m_samples[index].land.time_us == 0, "Typed reset zero land.time_us");
        __ESBMC_assert(ring.m_samples[index].vehicle.time_us == 0, "Typed reset zero vehicle.time_us");
        __ESBMC_assert(ring.m_samples[index].land.known == bool{0}, "Typed reset zero land.known");
        __ESBMC_assert(ring.m_samples[index].land.landed == bool{0}, "Typed reset zero land.landed");
        __ESBMC_assert(ring.m_samples[index].land.at_rest == bool{0}, "Typed reset zero land.at_rest");
        __ESBMC_assert(ring.m_samples[index].land.in_ground_effect == bool{0},
                       "Typed reset zero land.in_ground_effect");
        __ESBMC_assert(ring.m_samples[index].vehicle.known == bool{0}, "Typed reset zero vehicle.known");
        __ESBMC_assert(ring.m_samples[index].vehicle.armed == bool{0}, "Typed reset zero vehicle.armed");
        __ESBMC_assert(ring.m_samples[index].vehicle.fixed_wing == bool{0}, "Typed reset zero vehicle.fixed_wing");
#endif
    }
}
