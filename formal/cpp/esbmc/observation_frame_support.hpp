// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include "observation_support.hpp"

// Verification observers: full owner metadata and either the complete owner or one mandatory
// 16-slot frame partition. Every buffer/chunk and scalar format must pass. Physical storage,
// payload values and runtime cursors are not reduced by this assertion split.
namespace
{
using observation_proof::Observations;
using observation_proof::same_double;
using observation_proof::same_timeline;
Observations * expected_owner;
constexpr std::size_t frame_begin = FORMAL_ESKF_PROOF_FRAME == 4 ? 0 : FORMAL_ESKF_PROOF_CHUNK * 16;
constexpr std::size_t frame_end = frame_begin + 16;
constexpr std::size_t frame_capacity = FORMAL_ESKF_PROOF_FRAME == 0 ? 16 : FORMAL_ESKF_PROOF_FRAME == 1 ? 64 : 128;
static_assert(FORMAL_ESKF_PROOF_FRAME >= 0 && FORMAL_ESKF_PROOF_FRAME <= 4);
static_assert(FORMAL_ESKF_PROOF_CHUNK >= 0);
static_assert(FORMAL_ESKF_PROOF_FRAME == 4 ? FORMAL_ESKF_PROOF_CHUNK == 0
                                           : FORMAL_ESKF_PROOF_CHUNK < frame_capacity / 16);
static_assert(FORMAL_ESKF_PROOF_FRAME == 4 || frame_end <= frame_capacity);
Observations::Timeline expected_gnss_time, expected_magnetic_time, expected_barometer_time;
formal_eskf::px4::TimingConfiguration expected_configuration;
std::uint64_t expected_seed, expected_latest, expected_fusion, expected_advanced;
double expected_average;
bool expected_configured;
std::size_t expected_ring_metadata[4][4];
void capture_metadata()
{
    expected_gnss_time = expected_owner->m_gnss_time;
    expected_magnetic_time = expected_owner->m_magnetic_time;
    expected_barometer_time = expected_owner->m_barometer_time;
    expected_configuration = expected_owner->m_configuration;
    expected_seed = expected_owner->m_seed_time_us;
    expected_latest = expected_owner->m_latest_time_us;
    expected_fusion = expected_owner->m_fusion_time_us;
    expected_advanced = expected_owner->m_advanced_at_us;
    expected_average = expected_owner->m_average_period_us;
    expected_configured = expected_owner->m_configured;
    expected_ring_metadata[0][0] = expected_owner->m_gnss.m_length;
    expected_ring_metadata[0][1] = expected_owner->m_gnss.m_count;
    expected_ring_metadata[0][2] = expected_owner->m_gnss.m_oldest;
    expected_ring_metadata[0][3] = expected_owner->m_gnss.m_write;
    expected_ring_metadata[1][0] = expected_owner->m_magnetic.m_length;
    expected_ring_metadata[1][1] = expected_owner->m_magnetic.m_count;
    expected_ring_metadata[1][2] = expected_owner->m_magnetic.m_oldest;
    expected_ring_metadata[1][3] = expected_owner->m_magnetic.m_write;
    expected_ring_metadata[2][0] = expected_owner->m_barometer.m_length;
    expected_ring_metadata[2][1] = expected_owner->m_barometer.m_count;
    expected_ring_metadata[2][2] = expected_owner->m_barometer.m_oldest;
    expected_ring_metadata[2][3] = expected_owner->m_barometer.m_write;
    expected_ring_metadata[3][0] = expected_owner->m_context.m_length;
    expected_ring_metadata[3][1] = expected_owner->m_context.m_count;
    expected_ring_metadata[3][2] = expected_owner->m_context.m_oldest;
    expected_ring_metadata[3][3] = expected_owner->m_context.m_write;
}
void check_metadata()
{
    __ESBMC_assert(same_timeline(expected_owner->m_gnss_time, expected_gnss_time) &&
                       same_timeline(expected_owner->m_magnetic_time, expected_magnetic_time) &&
                       same_timeline(expected_owner->m_barometer_time, expected_barometer_time) &&
                       imu_proof::same_configuration(expected_owner->m_configuration, expected_configuration) &&
                       expected_owner->m_seed_time_us == expected_seed &&
                       expected_owner->m_latest_time_us == expected_latest &&
                       expected_owner->m_fusion_time_us == expected_fusion &&
                       expected_owner->m_advanced_at_us == expected_advanced &&
                       expected_owner->m_configured == expected_configured &&
                       same_double(expected_owner->m_average_period_us, expected_average),
                   "Caller preserves complete producer-input owner metadata");
    __ESBMC_assert(expected_owner->m_gnss.m_length == expected_ring_metadata[0][0],
                   "Caller preserves producer-input gnss length");
    __ESBMC_assert(expected_owner->m_gnss.m_count == expected_ring_metadata[0][1],
                   "Caller preserves producer-input gnss count");
    __ESBMC_assert(expected_owner->m_gnss.m_oldest == expected_ring_metadata[0][2],
                   "Caller preserves producer-input gnss oldest");
    __ESBMC_assert(expected_owner->m_gnss.m_write == expected_ring_metadata[0][3],
                   "Caller preserves producer-input gnss write");
    __ESBMC_assert(expected_owner->m_magnetic.m_length == expected_ring_metadata[1][0],
                   "Caller preserves producer-input magnetic length");
    __ESBMC_assert(expected_owner->m_magnetic.m_count == expected_ring_metadata[1][1],
                   "Caller preserves producer-input magnetic count");
    __ESBMC_assert(expected_owner->m_magnetic.m_oldest == expected_ring_metadata[1][2],
                   "Caller preserves producer-input magnetic oldest");
    __ESBMC_assert(expected_owner->m_magnetic.m_write == expected_ring_metadata[1][3],
                   "Caller preserves producer-input magnetic write");
    __ESBMC_assert(expected_owner->m_barometer.m_length == expected_ring_metadata[2][0],
                   "Caller preserves producer-input barometer length");
    __ESBMC_assert(expected_owner->m_barometer.m_count == expected_ring_metadata[2][1],
                   "Caller preserves producer-input barometer count");
    __ESBMC_assert(expected_owner->m_barometer.m_oldest == expected_ring_metadata[2][2],
                   "Caller preserves producer-input barometer oldest");
    __ESBMC_assert(expected_owner->m_barometer.m_write == expected_ring_metadata[2][3],
                   "Caller preserves producer-input barometer write");
    __ESBMC_assert(expected_owner->m_context.m_length == expected_ring_metadata[3][0],
                   "Caller preserves producer-input context length");
    __ESBMC_assert(expected_owner->m_context.m_count == expected_ring_metadata[3][1],
                   "Caller preserves producer-input context count");
    __ESBMC_assert(expected_owner->m_context.m_oldest == expected_ring_metadata[3][2],
                   "Caller preserves producer-input context oldest");
    __ESBMC_assert(expected_owner->m_context.m_write == expected_ring_metadata[3][3],
                   "Caller preserves producer-input context write");
}
void assume_metadata()
{
    __ESBMC_assume(
        same_timeline(expected_owner->m_gnss_time, expected_gnss_time) &&
        same_timeline(expected_owner->m_magnetic_time, expected_magnetic_time) &&
        same_timeline(expected_owner->m_barometer_time, expected_barometer_time) &&
        imu_proof::same_configuration(expected_owner->m_configuration, expected_configuration) &&
        expected_owner->m_seed_time_us == expected_seed && expected_owner->m_latest_time_us == expected_latest &&
        expected_owner->m_fusion_time_us == expected_fusion && expected_owner->m_advanced_at_us == expected_advanced &&
        expected_owner->m_configured == expected_configured &&
        same_double(expected_owner->m_average_period_us, expected_average));
    __ESBMC_assume(expected_owner->m_gnss.m_length == expected_ring_metadata[0][0]);
    __ESBMC_assume(expected_owner->m_gnss.m_count == expected_ring_metadata[0][1]);
    __ESBMC_assume(expected_owner->m_gnss.m_oldest == expected_ring_metadata[0][2]);
    __ESBMC_assume(expected_owner->m_gnss.m_write == expected_ring_metadata[0][3]);
    __ESBMC_assume(expected_owner->m_magnetic.m_length == expected_ring_metadata[1][0]);
    __ESBMC_assume(expected_owner->m_magnetic.m_count == expected_ring_metadata[1][1]);
    __ESBMC_assume(expected_owner->m_magnetic.m_oldest == expected_ring_metadata[1][2]);
    __ESBMC_assume(expected_owner->m_magnetic.m_write == expected_ring_metadata[1][3]);
    __ESBMC_assume(expected_owner->m_barometer.m_length == expected_ring_metadata[2][0]);
    __ESBMC_assume(expected_owner->m_barometer.m_count == expected_ring_metadata[2][1]);
    __ESBMC_assume(expected_owner->m_barometer.m_oldest == expected_ring_metadata[2][2]);
    __ESBMC_assume(expected_owner->m_barometer.m_write == expected_ring_metadata[2][3]);
    __ESBMC_assume(expected_owner->m_context.m_length == expected_ring_metadata[3][0]);
    __ESBMC_assume(expected_owner->m_context.m_count == expected_ring_metadata[3][1]);
    __ESBMC_assume(expected_owner->m_context.m_oldest == expected_ring_metadata[3][2]);
    __ESBMC_assume(expected_owner->m_context.m_write == expected_ring_metadata[3][3]);
}

#if FORMAL_ESKF_PROOF_FRAME == 0 || FORMAL_ESKF_PROOF_FRAME == 4
std::uint64_t gnss_integers[16][5];
imu_proof::Scalar gnss_values[16][8];
double gnss_doubles[16][4];
std::uint32_t gnss_small[16][2];
std::uint8_t gnss_tags[16][3];
bool gnss_flags[16][2];
void capture_gnss_storage()
{
    auto const & owner = *expected_owner;
    for (std::size_t i = frame_begin; i < (FORMAL_ESKF_PROOF_FRAME == 4 ? 16 : frame_end); ++i)
    {
        gnss_integers[i][0] = owner.m_gnss.m_samples[i].time_us;
        gnss_integers[i][1] = owner.m_gnss.m_samples[i].arrival_time_us;
        gnss_integers[i][2] = owner.m_gnss.m_samples[i].residual_delay_us;
        gnss_integers[i][3] = owner.m_gnss.m_samples[i].generation;
        gnss_integers[i][4] = owner.m_gnss.m_samples[i].sample.stamp.time_us;
        gnss_values[i][0] = owner.m_gnss.m_samples[i].sample.velocity_n.m_storage.values.head;
        gnss_values[i][1] = owner.m_gnss.m_samples[i].sample.velocity_n.m_storage.values.tail.head;
        gnss_values[i][2] = owner.m_gnss.m_samples[i].sample.velocity_n.m_storage.values.tail.tail.head;
        gnss_values[i][3] = owner.m_gnss.m_samples[i].sample.horizontal_accuracy_m;
        gnss_values[i][4] = owner.m_gnss.m_samples[i].sample.vertical_accuracy_m;
        gnss_values[i][5] = owner.m_gnss.m_samples[i].sample.speed_accuracy_m_s;
        gnss_values[i][6] = owner.m_gnss.m_samples[i].sample.hdop;
        gnss_values[i][7] = owner.m_gnss.m_samples[i].sample.vdop;
        gnss_doubles[i][0] = owner.m_gnss.m_samples[i].sample.latitude_deg;
        gnss_doubles[i][1] = owner.m_gnss.m_samples[i].sample.longitude_deg;
        gnss_doubles[i][2] = owner.m_gnss.m_samples[i].sample.altitude_msl_m;
        gnss_doubles[i][3] = owner.m_gnss.m_samples[i].sample.altitude_ellipsoid_m;
        gnss_small[i][0] = owner.m_gnss.m_samples[i].half_period_us;
        gnss_small[i][1] = owner.m_gnss.m_samples[i].sample.stamp.source.device_id;
        gnss_tags[i][0] = owner.m_gnss.m_samples[i].sample.stamp.source.calibration_count;
        gnss_tags[i][1] = owner.m_gnss.m_samples[i].sample.fix_type;
        gnss_tags[i][2] = owner.m_gnss.m_samples[i].sample.satellites;
        gnss_flags[i][0] = owner.m_gnss.m_samples[i].sample.velocity_valid;
        gnss_flags[i][1] = owner.m_gnss.m_samples[i].sample.spoofed;
    }
}
void check_gnss_storage()
{
    auto const & owner = *expected_owner;
    for (std::size_t i = frame_begin; i < (FORMAL_ESKF_PROOF_FRAME == 4 ? 16 : frame_end); ++i)
    {
        __ESBMC_assert(owner.m_gnss.m_samples[i].time_us == gnss_integers[i][0], "Complete storage frame gnss.time_us");
        __ESBMC_assert(owner.m_gnss.m_samples[i].arrival_time_us == gnss_integers[i][1],
                       "Complete storage frame gnss.arrival_time_us");
        __ESBMC_assert(owner.m_gnss.m_samples[i].residual_delay_us == gnss_integers[i][2],
                       "Complete storage frame gnss.residual_delay_us");
        __ESBMC_assert(owner.m_gnss.m_samples[i].generation == gnss_integers[i][3],
                       "Complete storage frame gnss.generation");
        __ESBMC_assert(owner.m_gnss.m_samples[i].sample.stamp.time_us == gnss_integers[i][4],
                       "Complete storage frame gnss.sample.stamp.time_us");
        __ESBMC_assert(
            imu_proof::same(owner.m_gnss.m_samples[i].sample.velocity_n.m_storage.values.head, gnss_values[i][0]),
            "Complete storage frame gnss.sample.velocity_n.m_storage.values.head");
        __ESBMC_assert(
            imu_proof::same(owner.m_gnss.m_samples[i].sample.velocity_n.m_storage.values.tail.head, gnss_values[i][1]),
            "Complete storage frame gnss.sample.velocity_n.m_storage.values.tail.head");
        __ESBMC_assert(imu_proof::same(owner.m_gnss.m_samples[i].sample.velocity_n.m_storage.values.tail.tail.head,
                                       gnss_values[i][2]),
                       "Complete storage frame gnss.sample.velocity_n.m_storage.values.tail.tail.head");
        __ESBMC_assert(imu_proof::same(owner.m_gnss.m_samples[i].sample.horizontal_accuracy_m, gnss_values[i][3]),
                       "Complete storage frame gnss.sample.horizontal_accuracy_m");
        __ESBMC_assert(imu_proof::same(owner.m_gnss.m_samples[i].sample.vertical_accuracy_m, gnss_values[i][4]),
                       "Complete storage frame gnss.sample.vertical_accuracy_m");
        __ESBMC_assert(imu_proof::same(owner.m_gnss.m_samples[i].sample.speed_accuracy_m_s, gnss_values[i][5]),
                       "Complete storage frame gnss.sample.speed_accuracy_m_s");
        __ESBMC_assert(imu_proof::same(owner.m_gnss.m_samples[i].sample.hdop, gnss_values[i][6]),
                       "Complete storage frame gnss.sample.hdop");
        __ESBMC_assert(imu_proof::same(owner.m_gnss.m_samples[i].sample.vdop, gnss_values[i][7]),
                       "Complete storage frame gnss.sample.vdop");
        __ESBMC_assert(same_double(owner.m_gnss.m_samples[i].sample.latitude_deg, gnss_doubles[i][0]),
                       "Complete storage frame gnss.sample.latitude_deg");
        __ESBMC_assert(same_double(owner.m_gnss.m_samples[i].sample.longitude_deg, gnss_doubles[i][1]),
                       "Complete storage frame gnss.sample.longitude_deg");
        __ESBMC_assert(same_double(owner.m_gnss.m_samples[i].sample.altitude_msl_m, gnss_doubles[i][2]),
                       "Complete storage frame gnss.sample.altitude_msl_m");
        __ESBMC_assert(same_double(owner.m_gnss.m_samples[i].sample.altitude_ellipsoid_m, gnss_doubles[i][3]),
                       "Complete storage frame gnss.sample.altitude_ellipsoid_m");
        __ESBMC_assert(owner.m_gnss.m_samples[i].half_period_us == gnss_small[i][0],
                       "Complete storage frame gnss.half_period_us");
        __ESBMC_assert(owner.m_gnss.m_samples[i].sample.stamp.source.device_id == gnss_small[i][1],
                       "Complete storage frame gnss.sample.stamp.source.device_id");
        __ESBMC_assert(owner.m_gnss.m_samples[i].sample.stamp.source.calibration_count == gnss_tags[i][0],
                       "Complete storage frame gnss.sample.stamp.source.calibration_count");
        __ESBMC_assert(owner.m_gnss.m_samples[i].sample.fix_type == gnss_tags[i][1],
                       "Complete storage frame gnss.sample.fix_type");
        __ESBMC_assert(owner.m_gnss.m_samples[i].sample.satellites == gnss_tags[i][2],
                       "Complete storage frame gnss.sample.satellites");
        __ESBMC_assert(owner.m_gnss.m_samples[i].sample.velocity_valid == gnss_flags[i][0],
                       "Complete storage frame gnss.sample.velocity_valid");
        __ESBMC_assert(owner.m_gnss.m_samples[i].sample.spoofed == gnss_flags[i][1],
                       "Complete storage frame gnss.sample.spoofed");
    }
}
void assume_gnss_storage()
{
    auto const & owner = *expected_owner;
    for (std::size_t i = frame_begin; i < (FORMAL_ESKF_PROOF_FRAME == 4 ? 16 : frame_end); ++i)
    {
        __ESBMC_assume(owner.m_gnss.m_samples[i].time_us == gnss_integers[i][0]);
        __ESBMC_assume(owner.m_gnss.m_samples[i].arrival_time_us == gnss_integers[i][1]);
        __ESBMC_assume(owner.m_gnss.m_samples[i].residual_delay_us == gnss_integers[i][2]);
        __ESBMC_assume(owner.m_gnss.m_samples[i].generation == gnss_integers[i][3]);
        __ESBMC_assume(owner.m_gnss.m_samples[i].sample.stamp.time_us == gnss_integers[i][4]);
        __ESBMC_assume(
            imu_proof::same(owner.m_gnss.m_samples[i].sample.velocity_n.m_storage.values.head, gnss_values[i][0]));
        __ESBMC_assume(
            imu_proof::same(owner.m_gnss.m_samples[i].sample.velocity_n.m_storage.values.tail.head, gnss_values[i][1]));
        __ESBMC_assume(imu_proof::same(owner.m_gnss.m_samples[i].sample.velocity_n.m_storage.values.tail.tail.head,
                                       gnss_values[i][2]));
        __ESBMC_assume(imu_proof::same(owner.m_gnss.m_samples[i].sample.horizontal_accuracy_m, gnss_values[i][3]));
        __ESBMC_assume(imu_proof::same(owner.m_gnss.m_samples[i].sample.vertical_accuracy_m, gnss_values[i][4]));
        __ESBMC_assume(imu_proof::same(owner.m_gnss.m_samples[i].sample.speed_accuracy_m_s, gnss_values[i][5]));
        __ESBMC_assume(imu_proof::same(owner.m_gnss.m_samples[i].sample.hdop, gnss_values[i][6]));
        __ESBMC_assume(imu_proof::same(owner.m_gnss.m_samples[i].sample.vdop, gnss_values[i][7]));
        __ESBMC_assume(same_double(owner.m_gnss.m_samples[i].sample.latitude_deg, gnss_doubles[i][0]));
        __ESBMC_assume(same_double(owner.m_gnss.m_samples[i].sample.longitude_deg, gnss_doubles[i][1]));
        __ESBMC_assume(same_double(owner.m_gnss.m_samples[i].sample.altitude_msl_m, gnss_doubles[i][2]));
        __ESBMC_assume(same_double(owner.m_gnss.m_samples[i].sample.altitude_ellipsoid_m, gnss_doubles[i][3]));
        __ESBMC_assume(owner.m_gnss.m_samples[i].half_period_us == gnss_small[i][0]);
        __ESBMC_assume(owner.m_gnss.m_samples[i].sample.stamp.source.device_id == gnss_small[i][1]);
        __ESBMC_assume(owner.m_gnss.m_samples[i].sample.stamp.source.calibration_count == gnss_tags[i][0]);
        __ESBMC_assume(owner.m_gnss.m_samples[i].sample.fix_type == gnss_tags[i][1]);
        __ESBMC_assume(owner.m_gnss.m_samples[i].sample.satellites == gnss_tags[i][2]);
        __ESBMC_assume(owner.m_gnss.m_samples[i].sample.velocity_valid == gnss_flags[i][0]);
        __ESBMC_assume(owner.m_gnss.m_samples[i].sample.spoofed == gnss_flags[i][1]);
    }
}
#endif

#if FORMAL_ESKF_PROOF_FRAME == 1 || FORMAL_ESKF_PROOF_FRAME == 4
std::uint64_t magnetic_integers[64][5];
imu_proof::Scalar magnetic_values[64][3];
std::uint32_t magnetic_small[64][2];
std::uint8_t magnetic_tags[64][1];
void capture_magnetic_storage()
{
    auto const & owner = *expected_owner;
    for (std::size_t i = frame_begin; i < (FORMAL_ESKF_PROOF_FRAME == 4 ? 64 : frame_end); ++i)
    {
        magnetic_integers[i][0] = owner.m_magnetic.m_samples[i].time_us;
        magnetic_integers[i][1] = owner.m_magnetic.m_samples[i].arrival_time_us;
        magnetic_integers[i][2] = owner.m_magnetic.m_samples[i].residual_delay_us;
        magnetic_integers[i][3] = owner.m_magnetic.m_samples[i].generation;
        magnetic_integers[i][4] = owner.m_magnetic.m_samples[i].sample.stamp.time_us;
        magnetic_values[i][0] = owner.m_magnetic.m_samples[i].sample.field_b.m_storage.values.head;
        magnetic_values[i][1] = owner.m_magnetic.m_samples[i].sample.field_b.m_storage.values.tail.head;
        magnetic_values[i][2] = owner.m_magnetic.m_samples[i].sample.field_b.m_storage.values.tail.tail.head;
        magnetic_small[i][0] = owner.m_magnetic.m_samples[i].half_period_us;
        magnetic_small[i][1] = owner.m_magnetic.m_samples[i].sample.stamp.source.device_id;
        magnetic_tags[i][0] = owner.m_magnetic.m_samples[i].sample.stamp.source.calibration_count;
    }
}
void check_magnetic_storage()
{
    auto const & owner = *expected_owner;
    for (std::size_t i = frame_begin; i < (FORMAL_ESKF_PROOF_FRAME == 4 ? 64 : frame_end); ++i)
    {
        __ESBMC_assert(owner.m_magnetic.m_samples[i].time_us == magnetic_integers[i][0],
                       "Complete storage frame magnetic.time_us");
        __ESBMC_assert(owner.m_magnetic.m_samples[i].arrival_time_us == magnetic_integers[i][1],
                       "Complete storage frame magnetic.arrival_time_us");
        __ESBMC_assert(owner.m_magnetic.m_samples[i].residual_delay_us == magnetic_integers[i][2],
                       "Complete storage frame magnetic.residual_delay_us");
        __ESBMC_assert(owner.m_magnetic.m_samples[i].generation == magnetic_integers[i][3],
                       "Complete storage frame magnetic.generation");
        __ESBMC_assert(owner.m_magnetic.m_samples[i].sample.stamp.time_us == magnetic_integers[i][4],
                       "Complete storage frame magnetic.sample.stamp.time_us");
        __ESBMC_assert(
            imu_proof::same(owner.m_magnetic.m_samples[i].sample.field_b.m_storage.values.head, magnetic_values[i][0]),
            "Complete storage frame magnetic.sample.field_b.m_storage.values.head");
        __ESBMC_assert(imu_proof::same(owner.m_magnetic.m_samples[i].sample.field_b.m_storage.values.tail.head,
                                       magnetic_values[i][1]),
                       "Complete storage frame magnetic.sample.field_b.m_storage.values.tail.head");
        __ESBMC_assert(imu_proof::same(owner.m_magnetic.m_samples[i].sample.field_b.m_storage.values.tail.tail.head,
                                       magnetic_values[i][2]),
                       "Complete storage frame magnetic.sample.field_b.m_storage.values.tail.tail.head");
        __ESBMC_assert(owner.m_magnetic.m_samples[i].half_period_us == magnetic_small[i][0],
                       "Complete storage frame magnetic.half_period_us");
        __ESBMC_assert(owner.m_magnetic.m_samples[i].sample.stamp.source.device_id == magnetic_small[i][1],
                       "Complete storage frame magnetic.sample.stamp.source.device_id");
        __ESBMC_assert(owner.m_magnetic.m_samples[i].sample.stamp.source.calibration_count == magnetic_tags[i][0],
                       "Complete storage frame magnetic.sample.stamp.source.calibration_count");
    }
}
void assume_magnetic_storage()
{
    auto const & owner = *expected_owner;
    for (std::size_t i = frame_begin; i < (FORMAL_ESKF_PROOF_FRAME == 4 ? 64 : frame_end); ++i)
    {
        __ESBMC_assume(owner.m_magnetic.m_samples[i].time_us == magnetic_integers[i][0]);
        __ESBMC_assume(owner.m_magnetic.m_samples[i].arrival_time_us == magnetic_integers[i][1]);
        __ESBMC_assume(owner.m_magnetic.m_samples[i].residual_delay_us == magnetic_integers[i][2]);
        __ESBMC_assume(owner.m_magnetic.m_samples[i].generation == magnetic_integers[i][3]);
        __ESBMC_assume(owner.m_magnetic.m_samples[i].sample.stamp.time_us == magnetic_integers[i][4]);
        __ESBMC_assume(
            imu_proof::same(owner.m_magnetic.m_samples[i].sample.field_b.m_storage.values.head, magnetic_values[i][0]));
        __ESBMC_assume(imu_proof::same(owner.m_magnetic.m_samples[i].sample.field_b.m_storage.values.tail.head,
                                       magnetic_values[i][1]));
        __ESBMC_assume(imu_proof::same(owner.m_magnetic.m_samples[i].sample.field_b.m_storage.values.tail.tail.head,
                                       magnetic_values[i][2]));
        __ESBMC_assume(owner.m_magnetic.m_samples[i].half_period_us == magnetic_small[i][0]);
        __ESBMC_assume(owner.m_magnetic.m_samples[i].sample.stamp.source.device_id == magnetic_small[i][1]);
        __ESBMC_assume(owner.m_magnetic.m_samples[i].sample.stamp.source.calibration_count == magnetic_tags[i][0]);
    }
}
#endif

#if FORMAL_ESKF_PROOF_FRAME == 2 || FORMAL_ESKF_PROOF_FRAME == 4
std::uint64_t barometer_integers[128][5];
imu_proof::Scalar barometer_values[128][1];
std::uint32_t barometer_small[128][2];
std::uint8_t barometer_tags[128][1];
void capture_barometer_storage()
{
    auto const & owner = *expected_owner;
    for (std::size_t i = frame_begin; i < (FORMAL_ESKF_PROOF_FRAME == 4 ? 128 : frame_end); ++i)
    {
        barometer_integers[i][0] = owner.m_barometer.m_samples[i].time_us;
        barometer_integers[i][1] = owner.m_barometer.m_samples[i].arrival_time_us;
        barometer_integers[i][2] = owner.m_barometer.m_samples[i].residual_delay_us;
        barometer_integers[i][3] = owner.m_barometer.m_samples[i].generation;
        barometer_integers[i][4] = owner.m_barometer.m_samples[i].sample.stamp.time_us;
        barometer_values[i][0] = owner.m_barometer.m_samples[i].sample.altitude_m;
        barometer_small[i][0] = owner.m_barometer.m_samples[i].half_period_us;
        barometer_small[i][1] = owner.m_barometer.m_samples[i].sample.stamp.source.device_id;
        barometer_tags[i][0] = owner.m_barometer.m_samples[i].sample.stamp.source.calibration_count;
    }
}
void check_barometer_storage()
{
    auto const & owner = *expected_owner;
    for (std::size_t i = frame_begin; i < (FORMAL_ESKF_PROOF_FRAME == 4 ? 128 : frame_end); ++i)
    {
        __ESBMC_assert(owner.m_barometer.m_samples[i].time_us == barometer_integers[i][0],
                       "Complete storage frame barometer.time_us");
        __ESBMC_assert(owner.m_barometer.m_samples[i].arrival_time_us == barometer_integers[i][1],
                       "Complete storage frame barometer.arrival_time_us");
        __ESBMC_assert(owner.m_barometer.m_samples[i].residual_delay_us == barometer_integers[i][2],
                       "Complete storage frame barometer.residual_delay_us");
        __ESBMC_assert(owner.m_barometer.m_samples[i].generation == barometer_integers[i][3],
                       "Complete storage frame barometer.generation");
        __ESBMC_assert(owner.m_barometer.m_samples[i].sample.stamp.time_us == barometer_integers[i][4],
                       "Complete storage frame barometer.sample.stamp.time_us");
        __ESBMC_assert(imu_proof::same(owner.m_barometer.m_samples[i].sample.altitude_m, barometer_values[i][0]),
                       "Complete storage frame barometer.sample.altitude_m");
        __ESBMC_assert(owner.m_barometer.m_samples[i].half_period_us == barometer_small[i][0],
                       "Complete storage frame barometer.half_period_us");
        __ESBMC_assert(owner.m_barometer.m_samples[i].sample.stamp.source.device_id == barometer_small[i][1],
                       "Complete storage frame barometer.sample.stamp.source.device_id");
        __ESBMC_assert(owner.m_barometer.m_samples[i].sample.stamp.source.calibration_count == barometer_tags[i][0],
                       "Complete storage frame barometer.sample.stamp.source.calibration_count");
    }
}
void assume_barometer_storage()
{
    auto const & owner = *expected_owner;
    for (std::size_t i = frame_begin; i < (FORMAL_ESKF_PROOF_FRAME == 4 ? 128 : frame_end); ++i)
    {
        __ESBMC_assume(owner.m_barometer.m_samples[i].time_us == barometer_integers[i][0]);
        __ESBMC_assume(owner.m_barometer.m_samples[i].arrival_time_us == barometer_integers[i][1]);
        __ESBMC_assume(owner.m_barometer.m_samples[i].residual_delay_us == barometer_integers[i][2]);
        __ESBMC_assume(owner.m_barometer.m_samples[i].generation == barometer_integers[i][3]);
        __ESBMC_assume(owner.m_barometer.m_samples[i].sample.stamp.time_us == barometer_integers[i][4]);
        __ESBMC_assume(imu_proof::same(owner.m_barometer.m_samples[i].sample.altitude_m, barometer_values[i][0]));
        __ESBMC_assume(owner.m_barometer.m_samples[i].half_period_us == barometer_small[i][0]);
        __ESBMC_assume(owner.m_barometer.m_samples[i].sample.stamp.source.device_id == barometer_small[i][1]);
        __ESBMC_assume(owner.m_barometer.m_samples[i].sample.stamp.source.calibration_count == barometer_tags[i][0]);
    }
}
#endif

#if FORMAL_ESKF_PROOF_FRAME == 3 || FORMAL_ESKF_PROOF_FRAME == 4
std::uint64_t context_integers[128][3];
bool context_flags[128][7];
void capture_context_storage()
{
    auto const & owner = *expected_owner;
    for (std::size_t i = frame_begin; i < (FORMAL_ESKF_PROOF_FRAME == 4 ? 128 : frame_end); ++i)
    {
        context_integers[i][0] = owner.m_context.m_samples[i].time_us;
        context_integers[i][1] = owner.m_context.m_samples[i].land.time_us;
        context_integers[i][2] = owner.m_context.m_samples[i].vehicle.time_us;
        context_flags[i][0] = owner.m_context.m_samples[i].land.known;
        context_flags[i][1] = owner.m_context.m_samples[i].land.landed;
        context_flags[i][2] = owner.m_context.m_samples[i].land.at_rest;
        context_flags[i][3] = owner.m_context.m_samples[i].land.in_ground_effect;
        context_flags[i][4] = owner.m_context.m_samples[i].vehicle.known;
        context_flags[i][5] = owner.m_context.m_samples[i].vehicle.armed;
        context_flags[i][6] = owner.m_context.m_samples[i].vehicle.fixed_wing;
    }
}
void check_context_storage()
{
    auto const & owner = *expected_owner;
    for (std::size_t i = frame_begin; i < (FORMAL_ESKF_PROOF_FRAME == 4 ? 128 : frame_end); ++i)
    {
        __ESBMC_assert(owner.m_context.m_samples[i].time_us == context_integers[i][0],
                       "Complete storage frame context.time_us");
        __ESBMC_assert(owner.m_context.m_samples[i].land.time_us == context_integers[i][1],
                       "Complete storage frame context.land.time_us");
        __ESBMC_assert(owner.m_context.m_samples[i].vehicle.time_us == context_integers[i][2],
                       "Complete storage frame context.vehicle.time_us");
        __ESBMC_assert(owner.m_context.m_samples[i].land.known == context_flags[i][0],
                       "Complete storage frame context.land.known");
        __ESBMC_assert(owner.m_context.m_samples[i].land.landed == context_flags[i][1],
                       "Complete storage frame context.land.landed");
        __ESBMC_assert(owner.m_context.m_samples[i].land.at_rest == context_flags[i][2],
                       "Complete storage frame context.land.at_rest");
        __ESBMC_assert(owner.m_context.m_samples[i].land.in_ground_effect == context_flags[i][3],
                       "Complete storage frame context.land.in_ground_effect");
        __ESBMC_assert(owner.m_context.m_samples[i].vehicle.known == context_flags[i][4],
                       "Complete storage frame context.vehicle.known");
        __ESBMC_assert(owner.m_context.m_samples[i].vehicle.armed == context_flags[i][5],
                       "Complete storage frame context.vehicle.armed");
        __ESBMC_assert(owner.m_context.m_samples[i].vehicle.fixed_wing == context_flags[i][6],
                       "Complete storage frame context.vehicle.fixed_wing");
    }
}
void assume_context_storage()
{
    auto const & owner = *expected_owner;
    for (std::size_t i = frame_begin; i < (FORMAL_ESKF_PROOF_FRAME == 4 ? 128 : frame_end); ++i)
    {
        __ESBMC_assume(owner.m_context.m_samples[i].time_us == context_integers[i][0]);
        __ESBMC_assume(owner.m_context.m_samples[i].land.time_us == context_integers[i][1]);
        __ESBMC_assume(owner.m_context.m_samples[i].vehicle.time_us == context_integers[i][2]);
        __ESBMC_assume(owner.m_context.m_samples[i].land.known == context_flags[i][0]);
        __ESBMC_assume(owner.m_context.m_samples[i].land.landed == context_flags[i][1]);
        __ESBMC_assume(owner.m_context.m_samples[i].land.at_rest == context_flags[i][2]);
        __ESBMC_assume(owner.m_context.m_samples[i].land.in_ground_effect == context_flags[i][3]);
        __ESBMC_assume(owner.m_context.m_samples[i].vehicle.known == context_flags[i][4]);
        __ESBMC_assume(owner.m_context.m_samples[i].vehicle.armed == context_flags[i][5]);
        __ESBMC_assume(owner.m_context.m_samples[i].vehicle.fixed_wing == context_flags[i][6]);
    }
}
#endif
void capture_storage()
{
#if FORMAL_ESKF_PROOF_FRAME == 0 || FORMAL_ESKF_PROOF_FRAME == 4
    capture_gnss_storage();
#endif
#if FORMAL_ESKF_PROOF_FRAME == 1 || FORMAL_ESKF_PROOF_FRAME == 4
    capture_magnetic_storage();
#endif
#if FORMAL_ESKF_PROOF_FRAME == 2 || FORMAL_ESKF_PROOF_FRAME == 4
    capture_barometer_storage();
#endif
#if FORMAL_ESKF_PROOF_FRAME == 3 || FORMAL_ESKF_PROOF_FRAME == 4
    capture_context_storage();
#endif
}
void check_storage()
{
#if FORMAL_ESKF_PROOF_FRAME == 0 || FORMAL_ESKF_PROOF_FRAME == 4
    check_gnss_storage();
#endif
#if FORMAL_ESKF_PROOF_FRAME == 1 || FORMAL_ESKF_PROOF_FRAME == 4
    check_magnetic_storage();
#endif
#if FORMAL_ESKF_PROOF_FRAME == 2 || FORMAL_ESKF_PROOF_FRAME == 4
    check_barometer_storage();
#endif
#if FORMAL_ESKF_PROOF_FRAME == 3 || FORMAL_ESKF_PROOF_FRAME == 4
    check_context_storage();
#endif
}
void assume_storage()
{
#if FORMAL_ESKF_PROOF_FRAME == 0 || FORMAL_ESKF_PROOF_FRAME == 4
    assume_gnss_storage();
#endif
#if FORMAL_ESKF_PROOF_FRAME == 1 || FORMAL_ESKF_PROOF_FRAME == 4
    assume_magnetic_storage();
#endif
#if FORMAL_ESKF_PROOF_FRAME == 2 || FORMAL_ESKF_PROOF_FRAME == 4
    assume_barometer_storage();
#endif
#if FORMAL_ESKF_PROOF_FRAME == 3 || FORMAL_ESKF_PROOF_FRAME == 4
    assume_context_storage();
#endif
}
} // namespace
