// SPDX-License-Identifier: BSD-3-Clause
#include "observation_support.hpp"
using observation_proof::Observations;
using observation_proof::same_double;
static_assert(FORMAL_ESKF_PROOF_OBSERVATION_SOURCE < 3);
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
using Record = Observations::GnssObservation;
constexpr std::size_t capacity = 16;
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
using Record = Observations::MagneticObservation;
constexpr std::size_t capacity = 64;
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
using Record = Observations::BarometerObservation;
constexpr std::size_t capacity = 128;
#endif
using Ring = formal_eskf::runtime::RingBuffer<Record, capacity>;

// Source-extracted member steps use the actual records, storage and scalar
// formats. Accept every physical-slot partition, the original null guard and
// final suffix, plus the Lean loop induction and reviewed source correspondence.
// These are not successful whole-loop ESBMC executions. No chronology/finite
// payload premise is imposed. External output and control locals are disjoint.
// Flat native-width snapshots avoid nested aggregate-reference expansion.

void verify_observation_ring_search(Ring ring, Record output, std::uint64_t horizon, std::uint64_t age,
                                    std::size_t inspected)
{
    constexpr std::size_t chosen = FORMAL_ESKF_PROOF_SLOT;
    static_assert(chosen < capacity);
    ring.m_length = capacity;
    __ESBMC_assume(ring.m_count <= capacity && inspected < ring.m_count && ring.m_oldest < capacity &&
                   ring.m_write < capacity);
    auto const count = ring.m_count;
    auto const oldest = ring.m_oldest;
    auto const write = ring.m_write;
    auto const old_horizon = horizon;
    auto const old_age = age;
    auto const old_inspected = inspected;
    auto const time = ring.m_samples[chosen].time_us;
    std::size_t index = chosen + 1 == capacity ? 0 : chosen + 1;
    bool selected = false;
    std::size_t removed = 0;
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
    std::uint64_t integers[capacity][5];
    std::uint64_t old_integers[5];
    old_integers[0] = output.time_us;
    old_integers[1] = output.arrival_time_us;
    old_integers[2] = output.residual_delay_us;
    old_integers[3] = output.generation;
    old_integers[4] = output.sample.stamp.time_us;
    imu_proof::Scalar values[capacity][8];
    imu_proof::Scalar old_values[8];
    old_values[0] = output.sample.velocity_n.m_storage.values.head;
    old_values[1] = output.sample.velocity_n.m_storage.values.tail.head;
    old_values[2] = output.sample.velocity_n.m_storage.values.tail.tail.head;
    old_values[3] = output.sample.horizontal_accuracy_m;
    old_values[4] = output.sample.vertical_accuracy_m;
    old_values[5] = output.sample.speed_accuracy_m_s;
    old_values[6] = output.sample.hdop;
    old_values[7] = output.sample.vdop;
    double doubles[capacity][4];
    double old_doubles[4];
    old_doubles[0] = output.sample.latitude_deg;
    old_doubles[1] = output.sample.longitude_deg;
    old_doubles[2] = output.sample.altitude_msl_m;
    old_doubles[3] = output.sample.altitude_ellipsoid_m;
    std::uint32_t small[capacity][2];
    std::uint32_t old_small[2];
    old_small[0] = output.half_period_us;
    old_small[1] = output.sample.stamp.source.device_id;
    std::uint8_t tags[capacity][3];
    std::uint8_t old_tags[3];
    old_tags[0] = output.sample.stamp.source.calibration_count;
    old_tags[1] = output.sample.fix_type;
    old_tags[2] = output.sample.satellites;
    bool flags[capacity][2];
    bool old_flags[2];
    old_flags[0] = output.sample.velocity_valid;
    old_flags[1] = output.sample.spoofed;
    for (std::size_t i = 0; i < capacity; ++i)
    {
        integers[i][0] = ring.m_samples[i].time_us;
        integers[i][1] = ring.m_samples[i].arrival_time_us;
        integers[i][2] = ring.m_samples[i].residual_delay_us;
        integers[i][3] = ring.m_samples[i].generation;
        integers[i][4] = ring.m_samples[i].sample.stamp.time_us;
        values[i][0] = ring.m_samples[i].sample.velocity_n.m_storage.values.head;
        values[i][1] = ring.m_samples[i].sample.velocity_n.m_storage.values.tail.head;
        values[i][2] = ring.m_samples[i].sample.velocity_n.m_storage.values.tail.tail.head;
        values[i][3] = ring.m_samples[i].sample.horizontal_accuracy_m;
        values[i][4] = ring.m_samples[i].sample.vertical_accuracy_m;
        values[i][5] = ring.m_samples[i].sample.speed_accuracy_m_s;
        values[i][6] = ring.m_samples[i].sample.hdop;
        values[i][7] = ring.m_samples[i].sample.vdop;
        doubles[i][0] = ring.m_samples[i].sample.latitude_deg;
        doubles[i][1] = ring.m_samples[i].sample.longitude_deg;
        doubles[i][2] = ring.m_samples[i].sample.altitude_msl_m;
        doubles[i][3] = ring.m_samples[i].sample.altitude_ellipsoid_m;
        small[i][0] = ring.m_samples[i].half_period_us;
        small[i][1] = ring.m_samples[i].sample.stamp.source.device_id;
        tags[i][0] = ring.m_samples[i].sample.stamp.source.calibration_count;
        tags[i][1] = ring.m_samples[i].sample.fix_type;
        tags[i][2] = ring.m_samples[i].sample.satellites;
        flags[i][0] = ring.m_samples[i].sample.velocity_valid;
        flags[i][1] = ring.m_samples[i].sample.spoofed;
    }
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
    std::uint64_t integers[capacity][5];
    std::uint64_t old_integers[5];
    old_integers[0] = output.time_us;
    old_integers[1] = output.arrival_time_us;
    old_integers[2] = output.residual_delay_us;
    old_integers[3] = output.generation;
    old_integers[4] = output.sample.stamp.time_us;
    imu_proof::Scalar values[capacity][3];
    imu_proof::Scalar old_values[3];
    old_values[0] = output.sample.field_b.m_storage.values.head;
    old_values[1] = output.sample.field_b.m_storage.values.tail.head;
    old_values[2] = output.sample.field_b.m_storage.values.tail.tail.head;
    std::uint32_t small[capacity][2];
    std::uint32_t old_small[2];
    old_small[0] = output.half_period_us;
    old_small[1] = output.sample.stamp.source.device_id;
    std::uint8_t tags[capacity][1];
    std::uint8_t old_tags[1];
    old_tags[0] = output.sample.stamp.source.calibration_count;
    for (std::size_t i = 0; i < capacity; ++i)
    {
        integers[i][0] = ring.m_samples[i].time_us;
        integers[i][1] = ring.m_samples[i].arrival_time_us;
        integers[i][2] = ring.m_samples[i].residual_delay_us;
        integers[i][3] = ring.m_samples[i].generation;
        integers[i][4] = ring.m_samples[i].sample.stamp.time_us;
        values[i][0] = ring.m_samples[i].sample.field_b.m_storage.values.head;
        values[i][1] = ring.m_samples[i].sample.field_b.m_storage.values.tail.head;
        values[i][2] = ring.m_samples[i].sample.field_b.m_storage.values.tail.tail.head;
        small[i][0] = ring.m_samples[i].half_period_us;
        small[i][1] = ring.m_samples[i].sample.stamp.source.device_id;
        tags[i][0] = ring.m_samples[i].sample.stamp.source.calibration_count;
    }
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
    std::uint64_t integers[capacity][5];
    std::uint64_t old_integers[5];
    old_integers[0] = output.time_us;
    old_integers[1] = output.arrival_time_us;
    old_integers[2] = output.residual_delay_us;
    old_integers[3] = output.generation;
    old_integers[4] = output.sample.stamp.time_us;
    imu_proof::Scalar values[capacity][1];
    imu_proof::Scalar old_values[1];
    old_values[0] = output.sample.altitude_m;
    std::uint32_t small[capacity][2];
    std::uint32_t old_small[2];
    old_small[0] = output.half_period_us;
    old_small[1] = output.sample.stamp.source.device_id;
    std::uint8_t tags[capacity][1];
    std::uint8_t old_tags[1];
    old_tags[0] = output.sample.stamp.source.calibration_count;
    for (std::size_t i = 0; i < capacity; ++i)
    {
        integers[i][0] = ring.m_samples[i].time_us;
        integers[i][1] = ring.m_samples[i].arrival_time_us;
        integers[i][2] = ring.m_samples[i].residual_delay_us;
        integers[i][3] = ring.m_samples[i].generation;
        integers[i][4] = ring.m_samples[i].sample.stamp.time_us;
        values[i][0] = ring.m_samples[i].sample.altitude_m;
        small[i][0] = ring.m_samples[i].half_period_us;
        small[i][1] = ring.m_samples[i].sample.stamp.source.device_id;
        tags[i][0] = ring.m_samples[i].sample.stamp.source.calibration_count;
    }
#endif
    bool const stopped = ring.proof_search_step(index, inspected, horizon, &output, age, selected, removed);
    bool const matched = time <= old_horizon;
    bool const expected = matched && old_horizon - time < old_age;
    __ESBMC_assert(stopped == matched && selected == expected && index == chosen &&
                       removed == (matched ? count - old_inspected : 0),
                   "Search step exact control transfer");
    __ESBMC_assert(ring.m_count == count && ring.m_oldest == oldest && ring.m_write == write &&
                       ring.m_length == capacity,
                   "Search step ring metadata frame");
    __ESBMC_assert(horizon == old_horizon && age == old_age && inspected == old_inspected, "Search step input frame");
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
    for (std::size_t i = 0; i < capacity; ++i)
    {
        __ESBMC_assert(ring.m_samples[i].time_us == integers[i][0], "Step storage frame time_us");
        __ESBMC_assert(ring.m_samples[i].arrival_time_us == integers[i][1], "Step storage frame arrival_time_us");
        __ESBMC_assert(ring.m_samples[i].residual_delay_us == integers[i][2], "Step storage frame residual_delay_us");
        __ESBMC_assert(ring.m_samples[i].generation == integers[i][3], "Step storage frame generation");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.time_us == integers[i][4],
                       "Step storage frame sample.stamp.time_us");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.velocity_n.m_storage.values.head, values[i][0]),
                       "Step storage frame sample.velocity_n.m_storage.values.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.velocity_n.m_storage.values.tail.head, values[i][1]),
                       "Step storage frame sample.velocity_n.m_storage.values.tail.head");
        __ESBMC_assert(
            imu_proof::same(ring.m_samples[i].sample.velocity_n.m_storage.values.tail.tail.head, values[i][2]),
            "Step storage frame sample.velocity_n.m_storage.values.tail.tail.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.horizontal_accuracy_m, values[i][3]),
                       "Step storage frame sample.horizontal_accuracy_m");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.vertical_accuracy_m, values[i][4]),
                       "Step storage frame sample.vertical_accuracy_m");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.speed_accuracy_m_s, values[i][5]),
                       "Step storage frame sample.speed_accuracy_m_s");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.hdop, values[i][6]), "Step storage frame sample.hdop");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.vdop, values[i][7]), "Step storage frame sample.vdop");
        __ESBMC_assert(same_double(ring.m_samples[i].sample.latitude_deg, doubles[i][0]),
                       "Step storage frame sample.latitude_deg");
        __ESBMC_assert(same_double(ring.m_samples[i].sample.longitude_deg, doubles[i][1]),
                       "Step storage frame sample.longitude_deg");
        __ESBMC_assert(same_double(ring.m_samples[i].sample.altitude_msl_m, doubles[i][2]),
                       "Step storage frame sample.altitude_msl_m");
        __ESBMC_assert(same_double(ring.m_samples[i].sample.altitude_ellipsoid_m, doubles[i][3]),
                       "Step storage frame sample.altitude_ellipsoid_m");
        __ESBMC_assert(ring.m_samples[i].half_period_us == small[i][0], "Step storage frame half_period_us");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.device_id == small[i][1],
                       "Step storage frame sample.stamp.source.device_id");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.calibration_count == tags[i][0],
                       "Step storage frame sample.stamp.source.calibration_count");
        __ESBMC_assert(ring.m_samples[i].sample.fix_type == tags[i][1], "Step storage frame sample.fix_type");
        __ESBMC_assert(ring.m_samples[i].sample.satellites == tags[i][2], "Step storage frame sample.satellites");
        __ESBMC_assert(ring.m_samples[i].sample.velocity_valid == flags[i][0],
                       "Step storage frame sample.velocity_valid");
        __ESBMC_assert(ring.m_samples[i].sample.spoofed == flags[i][1], "Step storage frame sample.spoofed");
    }
    __ESBMC_assert(output.time_us == (expected ? integers[chosen][0] : old_integers[0]),
                   "Step output copy or preservation time_us");
    __ESBMC_assert(output.arrival_time_us == (expected ? integers[chosen][1] : old_integers[1]),
                   "Step output copy or preservation arrival_time_us");
    __ESBMC_assert(output.residual_delay_us == (expected ? integers[chosen][2] : old_integers[2]),
                   "Step output copy or preservation residual_delay_us");
    __ESBMC_assert(output.generation == (expected ? integers[chosen][3] : old_integers[3]),
                   "Step output copy or preservation generation");
    __ESBMC_assert(output.sample.stamp.time_us == (expected ? integers[chosen][4] : old_integers[4]),
                   "Step output copy or preservation sample.stamp.time_us");
    __ESBMC_assert(
        imu_proof::same(output.sample.velocity_n.m_storage.values.head, (expected ? values[chosen][0] : old_values[0])),
        "Step output copy or preservation sample.velocity_n.m_storage.values.head");
    __ESBMC_assert(imu_proof::same(output.sample.velocity_n.m_storage.values.tail.head,
                                   (expected ? values[chosen][1] : old_values[1])),
                   "Step output copy or preservation sample.velocity_n.m_storage.values.tail.head");
    __ESBMC_assert(imu_proof::same(output.sample.velocity_n.m_storage.values.tail.tail.head,
                                   (expected ? values[chosen][2] : old_values[2])),
                   "Step output copy or preservation sample.velocity_n.m_storage.values.tail.tail.head");
    __ESBMC_assert(imu_proof::same(output.sample.horizontal_accuracy_m, (expected ? values[chosen][3] : old_values[3])),
                   "Step output copy or preservation sample.horizontal_accuracy_m");
    __ESBMC_assert(imu_proof::same(output.sample.vertical_accuracy_m, (expected ? values[chosen][4] : old_values[4])),
                   "Step output copy or preservation sample.vertical_accuracy_m");
    __ESBMC_assert(imu_proof::same(output.sample.speed_accuracy_m_s, (expected ? values[chosen][5] : old_values[5])),
                   "Step output copy or preservation sample.speed_accuracy_m_s");
    __ESBMC_assert(imu_proof::same(output.sample.hdop, (expected ? values[chosen][6] : old_values[6])),
                   "Step output copy or preservation sample.hdop");
    __ESBMC_assert(imu_proof::same(output.sample.vdop, (expected ? values[chosen][7] : old_values[7])),
                   "Step output copy or preservation sample.vdop");
    __ESBMC_assert(same_double(output.sample.latitude_deg, (expected ? doubles[chosen][0] : old_doubles[0])),
                   "Step output copy or preservation sample.latitude_deg");
    __ESBMC_assert(same_double(output.sample.longitude_deg, (expected ? doubles[chosen][1] : old_doubles[1])),
                   "Step output copy or preservation sample.longitude_deg");
    __ESBMC_assert(same_double(output.sample.altitude_msl_m, (expected ? doubles[chosen][2] : old_doubles[2])),
                   "Step output copy or preservation sample.altitude_msl_m");
    __ESBMC_assert(same_double(output.sample.altitude_ellipsoid_m, (expected ? doubles[chosen][3] : old_doubles[3])),
                   "Step output copy or preservation sample.altitude_ellipsoid_m");
    __ESBMC_assert(output.half_period_us == (expected ? small[chosen][0] : old_small[0]),
                   "Step output copy or preservation half_period_us");
    __ESBMC_assert(output.sample.stamp.source.device_id == (expected ? small[chosen][1] : old_small[1]),
                   "Step output copy or preservation sample.stamp.source.device_id");
    __ESBMC_assert(output.sample.stamp.source.calibration_count == (expected ? tags[chosen][0] : old_tags[0]),
                   "Step output copy or preservation sample.stamp.source.calibration_count");
    __ESBMC_assert(output.sample.fix_type == (expected ? tags[chosen][1] : old_tags[1]),
                   "Step output copy or preservation sample.fix_type");
    __ESBMC_assert(output.sample.satellites == (expected ? tags[chosen][2] : old_tags[2]),
                   "Step output copy or preservation sample.satellites");
    __ESBMC_assert(output.sample.velocity_valid == (expected ? flags[chosen][0] : old_flags[0]),
                   "Step output copy or preservation sample.velocity_valid");
    __ESBMC_assert(output.sample.spoofed == (expected ? flags[chosen][1] : old_flags[1]),
                   "Step output copy or preservation sample.spoofed");
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
    for (std::size_t i = 0; i < capacity; ++i)
    {
        __ESBMC_assert(ring.m_samples[i].time_us == integers[i][0], "Step storage frame time_us");
        __ESBMC_assert(ring.m_samples[i].arrival_time_us == integers[i][1], "Step storage frame arrival_time_us");
        __ESBMC_assert(ring.m_samples[i].residual_delay_us == integers[i][2], "Step storage frame residual_delay_us");
        __ESBMC_assert(ring.m_samples[i].generation == integers[i][3], "Step storage frame generation");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.time_us == integers[i][4],
                       "Step storage frame sample.stamp.time_us");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.field_b.m_storage.values.head, values[i][0]),
                       "Step storage frame sample.field_b.m_storage.values.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.field_b.m_storage.values.tail.head, values[i][1]),
                       "Step storage frame sample.field_b.m_storage.values.tail.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.field_b.m_storage.values.tail.tail.head, values[i][2]),
                       "Step storage frame sample.field_b.m_storage.values.tail.tail.head");
        __ESBMC_assert(ring.m_samples[i].half_period_us == small[i][0], "Step storage frame half_period_us");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.device_id == small[i][1],
                       "Step storage frame sample.stamp.source.device_id");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.calibration_count == tags[i][0],
                       "Step storage frame sample.stamp.source.calibration_count");
    }
    __ESBMC_assert(output.time_us == (expected ? integers[chosen][0] : old_integers[0]),
                   "Step output copy or preservation time_us");
    __ESBMC_assert(output.arrival_time_us == (expected ? integers[chosen][1] : old_integers[1]),
                   "Step output copy or preservation arrival_time_us");
    __ESBMC_assert(output.residual_delay_us == (expected ? integers[chosen][2] : old_integers[2]),
                   "Step output copy or preservation residual_delay_us");
    __ESBMC_assert(output.generation == (expected ? integers[chosen][3] : old_integers[3]),
                   "Step output copy or preservation generation");
    __ESBMC_assert(output.sample.stamp.time_us == (expected ? integers[chosen][4] : old_integers[4]),
                   "Step output copy or preservation sample.stamp.time_us");
    __ESBMC_assert(
        imu_proof::same(output.sample.field_b.m_storage.values.head, (expected ? values[chosen][0] : old_values[0])),
        "Step output copy or preservation sample.field_b.m_storage.values.head");
    __ESBMC_assert(imu_proof::same(output.sample.field_b.m_storage.values.tail.head,
                                   (expected ? values[chosen][1] : old_values[1])),
                   "Step output copy or preservation sample.field_b.m_storage.values.tail.head");
    __ESBMC_assert(imu_proof::same(output.sample.field_b.m_storage.values.tail.tail.head,
                                   (expected ? values[chosen][2] : old_values[2])),
                   "Step output copy or preservation sample.field_b.m_storage.values.tail.tail.head");
    __ESBMC_assert(output.half_period_us == (expected ? small[chosen][0] : old_small[0]),
                   "Step output copy or preservation half_period_us");
    __ESBMC_assert(output.sample.stamp.source.device_id == (expected ? small[chosen][1] : old_small[1]),
                   "Step output copy or preservation sample.stamp.source.device_id");
    __ESBMC_assert(output.sample.stamp.source.calibration_count == (expected ? tags[chosen][0] : old_tags[0]),
                   "Step output copy or preservation sample.stamp.source.calibration_count");
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
    for (std::size_t i = 0; i < capacity; ++i)
    {
        __ESBMC_assert(ring.m_samples[i].time_us == integers[i][0], "Step storage frame time_us");
        __ESBMC_assert(ring.m_samples[i].arrival_time_us == integers[i][1], "Step storage frame arrival_time_us");
        __ESBMC_assert(ring.m_samples[i].residual_delay_us == integers[i][2], "Step storage frame residual_delay_us");
        __ESBMC_assert(ring.m_samples[i].generation == integers[i][3], "Step storage frame generation");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.time_us == integers[i][4],
                       "Step storage frame sample.stamp.time_us");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.altitude_m, values[i][0]),
                       "Step storage frame sample.altitude_m");
        __ESBMC_assert(ring.m_samples[i].half_period_us == small[i][0], "Step storage frame half_period_us");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.device_id == small[i][1],
                       "Step storage frame sample.stamp.source.device_id");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.calibration_count == tags[i][0],
                       "Step storage frame sample.stamp.source.calibration_count");
    }
    __ESBMC_assert(output.time_us == (expected ? integers[chosen][0] : old_integers[0]),
                   "Step output copy or preservation time_us");
    __ESBMC_assert(output.arrival_time_us == (expected ? integers[chosen][1] : old_integers[1]),
                   "Step output copy or preservation arrival_time_us");
    __ESBMC_assert(output.residual_delay_us == (expected ? integers[chosen][2] : old_integers[2]),
                   "Step output copy or preservation residual_delay_us");
    __ESBMC_assert(output.generation == (expected ? integers[chosen][3] : old_integers[3]),
                   "Step output copy or preservation generation");
    __ESBMC_assert(output.sample.stamp.time_us == (expected ? integers[chosen][4] : old_integers[4]),
                   "Step output copy or preservation sample.stamp.time_us");
    __ESBMC_assert(imu_proof::same(output.sample.altitude_m, (expected ? values[chosen][0] : old_values[0])),
                   "Step output copy or preservation sample.altitude_m");
    __ESBMC_assert(output.half_period_us == (expected ? small[chosen][0] : old_small[0]),
                   "Step output copy or preservation half_period_us");
    __ESBMC_assert(output.sample.stamp.source.device_id == (expected ? small[chosen][1] : old_small[1]),
                   "Step output copy or preservation sample.stamp.source.device_id");
    __ESBMC_assert(output.sample.stamp.source.calibration_count == (expected ? tags[chosen][0] : old_tags[0]),
                   "Step output copy or preservation sample.stamp.source.calibration_count");
#endif
}

void verify_observation_ring_retire(Ring ring)
{
    constexpr std::size_t chosen = FORMAL_ESKF_PROOF_SLOT;
    static_assert(chosen < capacity);
    ring.m_length = capacity;
    ring.m_oldest = chosen;
    // During retirement count still denotes the initial count. There is no
    // ordinary oldest/count/write invariant at this intermediate point.
    auto const count = ring.m_count;
    auto const write = ring.m_write;
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
    std::uint64_t integers[capacity][5];
    imu_proof::Scalar values[capacity][8];
    double doubles[capacity][4];
    std::uint32_t small[capacity][2];
    std::uint8_t tags[capacity][3];
    bool flags[capacity][2];
    for (std::size_t i = 0; i < capacity; ++i)
    {
        integers[i][0] = ring.m_samples[i].time_us;
        integers[i][1] = ring.m_samples[i].arrival_time_us;
        integers[i][2] = ring.m_samples[i].residual_delay_us;
        integers[i][3] = ring.m_samples[i].generation;
        integers[i][4] = ring.m_samples[i].sample.stamp.time_us;
        values[i][0] = ring.m_samples[i].sample.velocity_n.m_storage.values.head;
        values[i][1] = ring.m_samples[i].sample.velocity_n.m_storage.values.tail.head;
        values[i][2] = ring.m_samples[i].sample.velocity_n.m_storage.values.tail.tail.head;
        values[i][3] = ring.m_samples[i].sample.horizontal_accuracy_m;
        values[i][4] = ring.m_samples[i].sample.vertical_accuracy_m;
        values[i][5] = ring.m_samples[i].sample.speed_accuracy_m_s;
        values[i][6] = ring.m_samples[i].sample.hdop;
        values[i][7] = ring.m_samples[i].sample.vdop;
        doubles[i][0] = ring.m_samples[i].sample.latitude_deg;
        doubles[i][1] = ring.m_samples[i].sample.longitude_deg;
        doubles[i][2] = ring.m_samples[i].sample.altitude_msl_m;
        doubles[i][3] = ring.m_samples[i].sample.altitude_ellipsoid_m;
        small[i][0] = ring.m_samples[i].half_period_us;
        small[i][1] = ring.m_samples[i].sample.stamp.source.device_id;
        tags[i][0] = ring.m_samples[i].sample.stamp.source.calibration_count;
        tags[i][1] = ring.m_samples[i].sample.fix_type;
        tags[i][2] = ring.m_samples[i].sample.satellites;
        flags[i][0] = ring.m_samples[i].sample.velocity_valid;
        flags[i][1] = ring.m_samples[i].sample.spoofed;
    }
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
    std::uint64_t integers[capacity][5];
    imu_proof::Scalar values[capacity][3];
    std::uint32_t small[capacity][2];
    std::uint8_t tags[capacity][1];
    for (std::size_t i = 0; i < capacity; ++i)
    {
        integers[i][0] = ring.m_samples[i].time_us;
        integers[i][1] = ring.m_samples[i].arrival_time_us;
        integers[i][2] = ring.m_samples[i].residual_delay_us;
        integers[i][3] = ring.m_samples[i].generation;
        integers[i][4] = ring.m_samples[i].sample.stamp.time_us;
        values[i][0] = ring.m_samples[i].sample.field_b.m_storage.values.head;
        values[i][1] = ring.m_samples[i].sample.field_b.m_storage.values.tail.head;
        values[i][2] = ring.m_samples[i].sample.field_b.m_storage.values.tail.tail.head;
        small[i][0] = ring.m_samples[i].half_period_us;
        small[i][1] = ring.m_samples[i].sample.stamp.source.device_id;
        tags[i][0] = ring.m_samples[i].sample.stamp.source.calibration_count;
    }
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
    std::uint64_t integers[capacity][5];
    imu_proof::Scalar values[capacity][1];
    std::uint32_t small[capacity][2];
    std::uint8_t tags[capacity][1];
    for (std::size_t i = 0; i < capacity; ++i)
    {
        integers[i][0] = ring.m_samples[i].time_us;
        integers[i][1] = ring.m_samples[i].arrival_time_us;
        integers[i][2] = ring.m_samples[i].residual_delay_us;
        integers[i][3] = ring.m_samples[i].generation;
        integers[i][4] = ring.m_samples[i].sample.stamp.time_us;
        values[i][0] = ring.m_samples[i].sample.altitude_m;
        small[i][0] = ring.m_samples[i].half_period_us;
        small[i][1] = ring.m_samples[i].sample.stamp.source.device_id;
        tags[i][0] = ring.m_samples[i].sample.stamp.source.calibration_count;
    }
#endif
    ring.proof_retire_step();
    __ESBMC_assert(ring.m_count == count && ring.m_write == write && ring.m_length == capacity &&
                       ring.m_oldest == (chosen + 1 == capacity ? 0 : chosen + 1),
                   "Retirement step exact index and metadata frame");
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
    for (std::size_t i = 0; i < capacity; ++i)
    {
        __ESBMC_assert(ring.m_samples[i].time_us == (i == chosen ? 0 : integers[i][0]), "Step storage frame time_us");
        __ESBMC_assert(ring.m_samples[i].arrival_time_us == integers[i][1], "Step storage frame arrival_time_us");
        __ESBMC_assert(ring.m_samples[i].residual_delay_us == integers[i][2], "Step storage frame residual_delay_us");
        __ESBMC_assert(ring.m_samples[i].generation == integers[i][3], "Step storage frame generation");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.time_us == integers[i][4],
                       "Step storage frame sample.stamp.time_us");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.velocity_n.m_storage.values.head, values[i][0]),
                       "Step storage frame sample.velocity_n.m_storage.values.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.velocity_n.m_storage.values.tail.head, values[i][1]),
                       "Step storage frame sample.velocity_n.m_storage.values.tail.head");
        __ESBMC_assert(
            imu_proof::same(ring.m_samples[i].sample.velocity_n.m_storage.values.tail.tail.head, values[i][2]),
            "Step storage frame sample.velocity_n.m_storage.values.tail.tail.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.horizontal_accuracy_m, values[i][3]),
                       "Step storage frame sample.horizontal_accuracy_m");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.vertical_accuracy_m, values[i][4]),
                       "Step storage frame sample.vertical_accuracy_m");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.speed_accuracy_m_s, values[i][5]),
                       "Step storage frame sample.speed_accuracy_m_s");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.hdop, values[i][6]), "Step storage frame sample.hdop");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.vdop, values[i][7]), "Step storage frame sample.vdop");
        __ESBMC_assert(same_double(ring.m_samples[i].sample.latitude_deg, doubles[i][0]),
                       "Step storage frame sample.latitude_deg");
        __ESBMC_assert(same_double(ring.m_samples[i].sample.longitude_deg, doubles[i][1]),
                       "Step storage frame sample.longitude_deg");
        __ESBMC_assert(same_double(ring.m_samples[i].sample.altitude_msl_m, doubles[i][2]),
                       "Step storage frame sample.altitude_msl_m");
        __ESBMC_assert(same_double(ring.m_samples[i].sample.altitude_ellipsoid_m, doubles[i][3]),
                       "Step storage frame sample.altitude_ellipsoid_m");
        __ESBMC_assert(ring.m_samples[i].half_period_us == small[i][0], "Step storage frame half_period_us");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.device_id == small[i][1],
                       "Step storage frame sample.stamp.source.device_id");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.calibration_count == tags[i][0],
                       "Step storage frame sample.stamp.source.calibration_count");
        __ESBMC_assert(ring.m_samples[i].sample.fix_type == tags[i][1], "Step storage frame sample.fix_type");
        __ESBMC_assert(ring.m_samples[i].sample.satellites == tags[i][2], "Step storage frame sample.satellites");
        __ESBMC_assert(ring.m_samples[i].sample.velocity_valid == flags[i][0],
                       "Step storage frame sample.velocity_valid");
        __ESBMC_assert(ring.m_samples[i].sample.spoofed == flags[i][1], "Step storage frame sample.spoofed");
    }
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
    for (std::size_t i = 0; i < capacity; ++i)
    {
        __ESBMC_assert(ring.m_samples[i].time_us == (i == chosen ? 0 : integers[i][0]), "Step storage frame time_us");
        __ESBMC_assert(ring.m_samples[i].arrival_time_us == integers[i][1], "Step storage frame arrival_time_us");
        __ESBMC_assert(ring.m_samples[i].residual_delay_us == integers[i][2], "Step storage frame residual_delay_us");
        __ESBMC_assert(ring.m_samples[i].generation == integers[i][3], "Step storage frame generation");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.time_us == integers[i][4],
                       "Step storage frame sample.stamp.time_us");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.field_b.m_storage.values.head, values[i][0]),
                       "Step storage frame sample.field_b.m_storage.values.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.field_b.m_storage.values.tail.head, values[i][1]),
                       "Step storage frame sample.field_b.m_storage.values.tail.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.field_b.m_storage.values.tail.tail.head, values[i][2]),
                       "Step storage frame sample.field_b.m_storage.values.tail.tail.head");
        __ESBMC_assert(ring.m_samples[i].half_period_us == small[i][0], "Step storage frame half_period_us");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.device_id == small[i][1],
                       "Step storage frame sample.stamp.source.device_id");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.calibration_count == tags[i][0],
                       "Step storage frame sample.stamp.source.calibration_count");
    }
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
    for (std::size_t i = 0; i < capacity; ++i)
    {
        __ESBMC_assert(ring.m_samples[i].time_us == (i == chosen ? 0 : integers[i][0]), "Step storage frame time_us");
        __ESBMC_assert(ring.m_samples[i].arrival_time_us == integers[i][1], "Step storage frame arrival_time_us");
        __ESBMC_assert(ring.m_samples[i].residual_delay_us == integers[i][2], "Step storage frame residual_delay_us");
        __ESBMC_assert(ring.m_samples[i].generation == integers[i][3], "Step storage frame generation");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.time_us == integers[i][4],
                       "Step storage frame sample.stamp.time_us");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.altitude_m, values[i][0]),
                       "Step storage frame sample.altitude_m");
        __ESBMC_assert(ring.m_samples[i].half_period_us == small[i][0], "Step storage frame half_period_us");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.device_id == small[i][1],
                       "Step storage frame sample.stamp.source.device_id");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.calibration_count == tags[i][0],
                       "Step storage frame sample.stamp.source.calibration_count");
    }
#endif
}

void verify_observation_ring_finish(Ring ring, std::size_t removed, bool selected)
{
    __ESBMC_assume(removed <= ring.m_count && ring.m_count <= capacity);
    auto const count = ring.m_count;
    auto const length = ring.m_length;
    auto const oldest = ring.m_oldest;
    auto const write = ring.m_write;
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
    std::uint64_t integers[capacity][5];
    imu_proof::Scalar values[capacity][8];
    double doubles[capacity][4];
    std::uint32_t small[capacity][2];
    std::uint8_t tags[capacity][3];
    bool flags[capacity][2];
    for (std::size_t i = 0; i < capacity; ++i)
    {
        integers[i][0] = ring.m_samples[i].time_us;
        integers[i][1] = ring.m_samples[i].arrival_time_us;
        integers[i][2] = ring.m_samples[i].residual_delay_us;
        integers[i][3] = ring.m_samples[i].generation;
        integers[i][4] = ring.m_samples[i].sample.stamp.time_us;
        values[i][0] = ring.m_samples[i].sample.velocity_n.m_storage.values.head;
        values[i][1] = ring.m_samples[i].sample.velocity_n.m_storage.values.tail.head;
        values[i][2] = ring.m_samples[i].sample.velocity_n.m_storage.values.tail.tail.head;
        values[i][3] = ring.m_samples[i].sample.horizontal_accuracy_m;
        values[i][4] = ring.m_samples[i].sample.vertical_accuracy_m;
        values[i][5] = ring.m_samples[i].sample.speed_accuracy_m_s;
        values[i][6] = ring.m_samples[i].sample.hdop;
        values[i][7] = ring.m_samples[i].sample.vdop;
        doubles[i][0] = ring.m_samples[i].sample.latitude_deg;
        doubles[i][1] = ring.m_samples[i].sample.longitude_deg;
        doubles[i][2] = ring.m_samples[i].sample.altitude_msl_m;
        doubles[i][3] = ring.m_samples[i].sample.altitude_ellipsoid_m;
        small[i][0] = ring.m_samples[i].half_period_us;
        small[i][1] = ring.m_samples[i].sample.stamp.source.device_id;
        tags[i][0] = ring.m_samples[i].sample.stamp.source.calibration_count;
        tags[i][1] = ring.m_samples[i].sample.fix_type;
        tags[i][2] = ring.m_samples[i].sample.satellites;
        flags[i][0] = ring.m_samples[i].sample.velocity_valid;
        flags[i][1] = ring.m_samples[i].sample.spoofed;
    }
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
    std::uint64_t integers[capacity][5];
    imu_proof::Scalar values[capacity][3];
    std::uint32_t small[capacity][2];
    std::uint8_t tags[capacity][1];
    for (std::size_t i = 0; i < capacity; ++i)
    {
        integers[i][0] = ring.m_samples[i].time_us;
        integers[i][1] = ring.m_samples[i].arrival_time_us;
        integers[i][2] = ring.m_samples[i].residual_delay_us;
        integers[i][3] = ring.m_samples[i].generation;
        integers[i][4] = ring.m_samples[i].sample.stamp.time_us;
        values[i][0] = ring.m_samples[i].sample.field_b.m_storage.values.head;
        values[i][1] = ring.m_samples[i].sample.field_b.m_storage.values.tail.head;
        values[i][2] = ring.m_samples[i].sample.field_b.m_storage.values.tail.tail.head;
        small[i][0] = ring.m_samples[i].half_period_us;
        small[i][1] = ring.m_samples[i].sample.stamp.source.device_id;
        tags[i][0] = ring.m_samples[i].sample.stamp.source.calibration_count;
    }
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
    std::uint64_t integers[capacity][5];
    imu_proof::Scalar values[capacity][1];
    std::uint32_t small[capacity][2];
    std::uint8_t tags[capacity][1];
    for (std::size_t i = 0; i < capacity; ++i)
    {
        integers[i][0] = ring.m_samples[i].time_us;
        integers[i][1] = ring.m_samples[i].arrival_time_us;
        integers[i][2] = ring.m_samples[i].residual_delay_us;
        integers[i][3] = ring.m_samples[i].generation;
        integers[i][4] = ring.m_samples[i].sample.stamp.time_us;
        values[i][0] = ring.m_samples[i].sample.altitude_m;
        small[i][0] = ring.m_samples[i].half_period_us;
        small[i][1] = ring.m_samples[i].sample.stamp.source.device_id;
        tags[i][0] = ring.m_samples[i].sample.stamp.source.calibration_count;
    }
#endif
    bool const result = ring.proof_finish(removed, selected);
    __ESBMC_assert(result == selected && ring.m_count == count - removed && ring.m_length == length &&
                       ring.m_oldest == oldest && ring.m_write == write,
                   "Finish exact result, count and metadata frame");
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
    for (std::size_t i = 0; i < capacity; ++i)
    {
        __ESBMC_assert(ring.m_samples[i].time_us == integers[i][0], "Step storage frame time_us");
        __ESBMC_assert(ring.m_samples[i].arrival_time_us == integers[i][1], "Step storage frame arrival_time_us");
        __ESBMC_assert(ring.m_samples[i].residual_delay_us == integers[i][2], "Step storage frame residual_delay_us");
        __ESBMC_assert(ring.m_samples[i].generation == integers[i][3], "Step storage frame generation");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.time_us == integers[i][4],
                       "Step storage frame sample.stamp.time_us");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.velocity_n.m_storage.values.head, values[i][0]),
                       "Step storage frame sample.velocity_n.m_storage.values.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.velocity_n.m_storage.values.tail.head, values[i][1]),
                       "Step storage frame sample.velocity_n.m_storage.values.tail.head");
        __ESBMC_assert(
            imu_proof::same(ring.m_samples[i].sample.velocity_n.m_storage.values.tail.tail.head, values[i][2]),
            "Step storage frame sample.velocity_n.m_storage.values.tail.tail.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.horizontal_accuracy_m, values[i][3]),
                       "Step storage frame sample.horizontal_accuracy_m");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.vertical_accuracy_m, values[i][4]),
                       "Step storage frame sample.vertical_accuracy_m");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.speed_accuracy_m_s, values[i][5]),
                       "Step storage frame sample.speed_accuracy_m_s");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.hdop, values[i][6]), "Step storage frame sample.hdop");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.vdop, values[i][7]), "Step storage frame sample.vdop");
        __ESBMC_assert(same_double(ring.m_samples[i].sample.latitude_deg, doubles[i][0]),
                       "Step storage frame sample.latitude_deg");
        __ESBMC_assert(same_double(ring.m_samples[i].sample.longitude_deg, doubles[i][1]),
                       "Step storage frame sample.longitude_deg");
        __ESBMC_assert(same_double(ring.m_samples[i].sample.altitude_msl_m, doubles[i][2]),
                       "Step storage frame sample.altitude_msl_m");
        __ESBMC_assert(same_double(ring.m_samples[i].sample.altitude_ellipsoid_m, doubles[i][3]),
                       "Step storage frame sample.altitude_ellipsoid_m");
        __ESBMC_assert(ring.m_samples[i].half_period_us == small[i][0], "Step storage frame half_period_us");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.device_id == small[i][1],
                       "Step storage frame sample.stamp.source.device_id");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.calibration_count == tags[i][0],
                       "Step storage frame sample.stamp.source.calibration_count");
        __ESBMC_assert(ring.m_samples[i].sample.fix_type == tags[i][1], "Step storage frame sample.fix_type");
        __ESBMC_assert(ring.m_samples[i].sample.satellites == tags[i][2], "Step storage frame sample.satellites");
        __ESBMC_assert(ring.m_samples[i].sample.velocity_valid == flags[i][0],
                       "Step storage frame sample.velocity_valid");
        __ESBMC_assert(ring.m_samples[i].sample.spoofed == flags[i][1], "Step storage frame sample.spoofed");
    }
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
    for (std::size_t i = 0; i < capacity; ++i)
    {
        __ESBMC_assert(ring.m_samples[i].time_us == integers[i][0], "Step storage frame time_us");
        __ESBMC_assert(ring.m_samples[i].arrival_time_us == integers[i][1], "Step storage frame arrival_time_us");
        __ESBMC_assert(ring.m_samples[i].residual_delay_us == integers[i][2], "Step storage frame residual_delay_us");
        __ESBMC_assert(ring.m_samples[i].generation == integers[i][3], "Step storage frame generation");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.time_us == integers[i][4],
                       "Step storage frame sample.stamp.time_us");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.field_b.m_storage.values.head, values[i][0]),
                       "Step storage frame sample.field_b.m_storage.values.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.field_b.m_storage.values.tail.head, values[i][1]),
                       "Step storage frame sample.field_b.m_storage.values.tail.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.field_b.m_storage.values.tail.tail.head, values[i][2]),
                       "Step storage frame sample.field_b.m_storage.values.tail.tail.head");
        __ESBMC_assert(ring.m_samples[i].half_period_us == small[i][0], "Step storage frame half_period_us");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.device_id == small[i][1],
                       "Step storage frame sample.stamp.source.device_id");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.calibration_count == tags[i][0],
                       "Step storage frame sample.stamp.source.calibration_count");
    }
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
    for (std::size_t i = 0; i < capacity; ++i)
    {
        __ESBMC_assert(ring.m_samples[i].time_us == integers[i][0], "Step storage frame time_us");
        __ESBMC_assert(ring.m_samples[i].arrival_time_us == integers[i][1], "Step storage frame arrival_time_us");
        __ESBMC_assert(ring.m_samples[i].residual_delay_us == integers[i][2], "Step storage frame residual_delay_us");
        __ESBMC_assert(ring.m_samples[i].generation == integers[i][3], "Step storage frame generation");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.time_us == integers[i][4],
                       "Step storage frame sample.stamp.time_us");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.altitude_m, values[i][0]),
                       "Step storage frame sample.altitude_m");
        __ESBMC_assert(ring.m_samples[i].half_period_us == small[i][0], "Step storage frame half_period_us");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.device_id == small[i][1],
                       "Step storage frame sample.stamp.source.device_id");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.calibration_count == tags[i][0],
                       "Step storage frame sample.stamp.source.calibration_count");
    }
#endif
}

void verify_observation_ring_null_output(Ring ring, std::uint64_t horizon, std::uint64_t age)
{
    auto const count = ring.m_count;
    auto const length = ring.m_length;
    auto const oldest = ring.m_oldest;
    auto const write = ring.m_write;
    auto const old_horizon = horizon;
    auto const old_age = age;
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
    std::uint64_t integers[capacity][5];
    imu_proof::Scalar values[capacity][8];
    double doubles[capacity][4];
    std::uint32_t small[capacity][2];
    std::uint8_t tags[capacity][3];
    bool flags[capacity][2];
    for (std::size_t i = 0; i < capacity; ++i)
    {
        integers[i][0] = ring.m_samples[i].time_us;
        integers[i][1] = ring.m_samples[i].arrival_time_us;
        integers[i][2] = ring.m_samples[i].residual_delay_us;
        integers[i][3] = ring.m_samples[i].generation;
        integers[i][4] = ring.m_samples[i].sample.stamp.time_us;
        values[i][0] = ring.m_samples[i].sample.velocity_n.m_storage.values.head;
        values[i][1] = ring.m_samples[i].sample.velocity_n.m_storage.values.tail.head;
        values[i][2] = ring.m_samples[i].sample.velocity_n.m_storage.values.tail.tail.head;
        values[i][3] = ring.m_samples[i].sample.horizontal_accuracy_m;
        values[i][4] = ring.m_samples[i].sample.vertical_accuracy_m;
        values[i][5] = ring.m_samples[i].sample.speed_accuracy_m_s;
        values[i][6] = ring.m_samples[i].sample.hdop;
        values[i][7] = ring.m_samples[i].sample.vdop;
        doubles[i][0] = ring.m_samples[i].sample.latitude_deg;
        doubles[i][1] = ring.m_samples[i].sample.longitude_deg;
        doubles[i][2] = ring.m_samples[i].sample.altitude_msl_m;
        doubles[i][3] = ring.m_samples[i].sample.altitude_ellipsoid_m;
        small[i][0] = ring.m_samples[i].half_period_us;
        small[i][1] = ring.m_samples[i].sample.stamp.source.device_id;
        tags[i][0] = ring.m_samples[i].sample.stamp.source.calibration_count;
        tags[i][1] = ring.m_samples[i].sample.fix_type;
        tags[i][2] = ring.m_samples[i].sample.satellites;
        flags[i][0] = ring.m_samples[i].sample.velocity_valid;
        flags[i][1] = ring.m_samples[i].sample.spoofed;
    }
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
    std::uint64_t integers[capacity][5];
    imu_proof::Scalar values[capacity][3];
    std::uint32_t small[capacity][2];
    std::uint8_t tags[capacity][1];
    for (std::size_t i = 0; i < capacity; ++i)
    {
        integers[i][0] = ring.m_samples[i].time_us;
        integers[i][1] = ring.m_samples[i].arrival_time_us;
        integers[i][2] = ring.m_samples[i].residual_delay_us;
        integers[i][3] = ring.m_samples[i].generation;
        integers[i][4] = ring.m_samples[i].sample.stamp.time_us;
        values[i][0] = ring.m_samples[i].sample.field_b.m_storage.values.head;
        values[i][1] = ring.m_samples[i].sample.field_b.m_storage.values.tail.head;
        values[i][2] = ring.m_samples[i].sample.field_b.m_storage.values.tail.tail.head;
        small[i][0] = ring.m_samples[i].half_period_us;
        small[i][1] = ring.m_samples[i].sample.stamp.source.device_id;
        tags[i][0] = ring.m_samples[i].sample.stamp.source.calibration_count;
    }
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
    std::uint64_t integers[capacity][5];
    imu_proof::Scalar values[capacity][1];
    std::uint32_t small[capacity][2];
    std::uint8_t tags[capacity][1];
    for (std::size_t i = 0; i < capacity; ++i)
    {
        integers[i][0] = ring.m_samples[i].time_us;
        integers[i][1] = ring.m_samples[i].arrival_time_us;
        integers[i][2] = ring.m_samples[i].residual_delay_us;
        integers[i][3] = ring.m_samples[i].generation;
        integers[i][4] = ring.m_samples[i].sample.stamp.time_us;
        values[i][0] = ring.m_samples[i].sample.altitude_m;
        small[i][0] = ring.m_samples[i].half_period_us;
        small[i][1] = ring.m_samples[i].sample.stamp.source.device_id;
        tags[i][0] = ring.m_samples[i].sample.stamp.source.calibration_count;
    }
#endif
    bool const result = ring.pop_first_older_than(horizon, nullptr, age);
    __ESBMC_assert(!result && ring.m_count == count && ring.m_length == length && ring.m_oldest == oldest &&
                       ring.m_write == write && horizon == old_horizon && age == old_age,
                   "Null output preserves complete ring and inputs even for invalid metadata");
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
    for (std::size_t i = 0; i < capacity; ++i)
    {
        __ESBMC_assert(ring.m_samples[i].time_us == integers[i][0], "Step storage frame time_us");
        __ESBMC_assert(ring.m_samples[i].arrival_time_us == integers[i][1], "Step storage frame arrival_time_us");
        __ESBMC_assert(ring.m_samples[i].residual_delay_us == integers[i][2], "Step storage frame residual_delay_us");
        __ESBMC_assert(ring.m_samples[i].generation == integers[i][3], "Step storage frame generation");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.time_us == integers[i][4],
                       "Step storage frame sample.stamp.time_us");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.velocity_n.m_storage.values.head, values[i][0]),
                       "Step storage frame sample.velocity_n.m_storage.values.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.velocity_n.m_storage.values.tail.head, values[i][1]),
                       "Step storage frame sample.velocity_n.m_storage.values.tail.head");
        __ESBMC_assert(
            imu_proof::same(ring.m_samples[i].sample.velocity_n.m_storage.values.tail.tail.head, values[i][2]),
            "Step storage frame sample.velocity_n.m_storage.values.tail.tail.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.horizontal_accuracy_m, values[i][3]),
                       "Step storage frame sample.horizontal_accuracy_m");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.vertical_accuracy_m, values[i][4]),
                       "Step storage frame sample.vertical_accuracy_m");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.speed_accuracy_m_s, values[i][5]),
                       "Step storage frame sample.speed_accuracy_m_s");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.hdop, values[i][6]), "Step storage frame sample.hdop");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.vdop, values[i][7]), "Step storage frame sample.vdop");
        __ESBMC_assert(same_double(ring.m_samples[i].sample.latitude_deg, doubles[i][0]),
                       "Step storage frame sample.latitude_deg");
        __ESBMC_assert(same_double(ring.m_samples[i].sample.longitude_deg, doubles[i][1]),
                       "Step storage frame sample.longitude_deg");
        __ESBMC_assert(same_double(ring.m_samples[i].sample.altitude_msl_m, doubles[i][2]),
                       "Step storage frame sample.altitude_msl_m");
        __ESBMC_assert(same_double(ring.m_samples[i].sample.altitude_ellipsoid_m, doubles[i][3]),
                       "Step storage frame sample.altitude_ellipsoid_m");
        __ESBMC_assert(ring.m_samples[i].half_period_us == small[i][0], "Step storage frame half_period_us");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.device_id == small[i][1],
                       "Step storage frame sample.stamp.source.device_id");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.calibration_count == tags[i][0],
                       "Step storage frame sample.stamp.source.calibration_count");
        __ESBMC_assert(ring.m_samples[i].sample.fix_type == tags[i][1], "Step storage frame sample.fix_type");
        __ESBMC_assert(ring.m_samples[i].sample.satellites == tags[i][2], "Step storage frame sample.satellites");
        __ESBMC_assert(ring.m_samples[i].sample.velocity_valid == flags[i][0],
                       "Step storage frame sample.velocity_valid");
        __ESBMC_assert(ring.m_samples[i].sample.spoofed == flags[i][1], "Step storage frame sample.spoofed");
    }
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
    for (std::size_t i = 0; i < capacity; ++i)
    {
        __ESBMC_assert(ring.m_samples[i].time_us == integers[i][0], "Step storage frame time_us");
        __ESBMC_assert(ring.m_samples[i].arrival_time_us == integers[i][1], "Step storage frame arrival_time_us");
        __ESBMC_assert(ring.m_samples[i].residual_delay_us == integers[i][2], "Step storage frame residual_delay_us");
        __ESBMC_assert(ring.m_samples[i].generation == integers[i][3], "Step storage frame generation");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.time_us == integers[i][4],
                       "Step storage frame sample.stamp.time_us");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.field_b.m_storage.values.head, values[i][0]),
                       "Step storage frame sample.field_b.m_storage.values.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.field_b.m_storage.values.tail.head, values[i][1]),
                       "Step storage frame sample.field_b.m_storage.values.tail.head");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.field_b.m_storage.values.tail.tail.head, values[i][2]),
                       "Step storage frame sample.field_b.m_storage.values.tail.tail.head");
        __ESBMC_assert(ring.m_samples[i].half_period_us == small[i][0], "Step storage frame half_period_us");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.device_id == small[i][1],
                       "Step storage frame sample.stamp.source.device_id");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.calibration_count == tags[i][0],
                       "Step storage frame sample.stamp.source.calibration_count");
    }
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
    for (std::size_t i = 0; i < capacity; ++i)
    {
        __ESBMC_assert(ring.m_samples[i].time_us == integers[i][0], "Step storage frame time_us");
        __ESBMC_assert(ring.m_samples[i].arrival_time_us == integers[i][1], "Step storage frame arrival_time_us");
        __ESBMC_assert(ring.m_samples[i].residual_delay_us == integers[i][2], "Step storage frame residual_delay_us");
        __ESBMC_assert(ring.m_samples[i].generation == integers[i][3], "Step storage frame generation");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.time_us == integers[i][4],
                       "Step storage frame sample.stamp.time_us");
        __ESBMC_assert(imu_proof::same(ring.m_samples[i].sample.altitude_m, values[i][0]),
                       "Step storage frame sample.altitude_m");
        __ESBMC_assert(ring.m_samples[i].half_period_us == small[i][0], "Step storage frame half_period_us");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.device_id == small[i][1],
                       "Step storage frame sample.stamp.source.device_id");
        __ESBMC_assert(ring.m_samples[i].sample.stamp.source.calibration_count == tags[i][0],
                       "Step storage frame sample.stamp.source.calibration_count");
    }
#endif
}
