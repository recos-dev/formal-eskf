// SPDX-License-Identifier: BSD-3-Clause

#include "observation_support.hpp"

static_assert(FORMAL_ESKF_PROOF_FRAME >= 0 && FORMAL_ESKF_PROOF_FRAME < 4);

using observation_proof::Limits;
using observation_proof::Observations;
using observation_proof::same_double;
using observation_proof::Stamp;

// Four complete-buffer partitions must run in both scalar formats. Their union
// covers every physical slot; no timestamp, payload, or owner-state sampling is used.
// Flat snapshots avoid ESBMC's aggregate-reference expansion without weakening
// the frame. Control/publication obligations execute in observation_control.cpp.
enum class FrameAction
{
    admission_gnss,
    admission_magnetic,
    admission_barometer,
    selection_gnss,
    selection_magnetic,
    selection_barometer,
    begin_fusion,
    configure,
    next_fusion,
    views
};
void verify_observation_storage_frame(Observations owner, Stamp stamp, std::uint64_t now, Limits limits,
                                      std::uint64_t effective, unsigned char action, imu_proof::Interval interval,
                                      formal_eskf::px4::TimingParameters parameters)
{
    __ESBMC_assume(owner.m_average_period_us >= 3800. && owner.m_average_period_us <= 4200.);
#if FORMAL_ESKF_PROOF_FRAME == 0
    auto const gnss_length = owner.m_gnss.m_length;
    auto const gnss_count = owner.m_gnss.m_count;
    auto const gnss_oldest = owner.m_gnss.m_oldest;
    auto const gnss_write = owner.m_gnss.m_write;
    std::uint64_t gnss_integers[16][5];
    imu_proof::Scalar gnss_values[16][8];
    double gnss_doubles[16][4];
    std::uint32_t gnss_small[16][2];
    std::uint8_t gnss_tags[16][3];
    bool gnss_flags[16][2];
    for (std::size_t i = 0; i < 16; ++i)
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
#endif
#if FORMAL_ESKF_PROOF_FRAME == 1
    auto const magnetic_length = owner.m_magnetic.m_length;
    auto const magnetic_count = owner.m_magnetic.m_count;
    auto const magnetic_oldest = owner.m_magnetic.m_oldest;
    auto const magnetic_write = owner.m_magnetic.m_write;
    std::uint64_t magnetic_integers[64][5];
    imu_proof::Scalar magnetic_values[64][3];
    std::uint32_t magnetic_small[64][2];
    std::uint8_t magnetic_tags[64][1];
    for (std::size_t i = 0; i < 64; ++i)
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
#endif
#if FORMAL_ESKF_PROOF_FRAME == 2
    auto const barometer_length = owner.m_barometer.m_length;
    auto const barometer_count = owner.m_barometer.m_count;
    auto const barometer_oldest = owner.m_barometer.m_oldest;
    auto const barometer_write = owner.m_barometer.m_write;
    std::uint64_t barometer_integers[128][5];
    imu_proof::Scalar barometer_values[128][1];
    std::uint32_t barometer_small[128][2];
    std::uint8_t barometer_tags[128][1];
    for (std::size_t i = 0; i < 128; ++i)
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
#endif
#if FORMAL_ESKF_PROOF_FRAME == 3
    auto const context_length = owner.m_context.m_length;
    auto const context_count = owner.m_context.m_count;
    auto const context_oldest = owner.m_context.m_oldest;
    auto const context_write = owner.m_context.m_write;
    std::uint64_t context_integers[128][3];
    bool context_flags[128][7];
    for (std::size_t i = 0; i < 128; ++i)
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
#endif
    __ESBMC_assume(action <= static_cast<unsigned char>(FrameAction::views));
    switch (static_cast<FrameAction>(action))
    {
    case FrameAction::admission_gnss:
        (void)owner.admit(owner.m_gnss_time, stamp, now, limits, effective);
        break;
    case FrameAction::admission_magnetic:
        (void)owner.admit(owner.m_magnetic_time, stamp, now, limits, effective);
        break;
    case FrameAction::admission_barometer:
        (void)owner.admit(owner.m_barometer_time, stamp, now, limits, effective);
        break;
    case FrameAction::selection_gnss:
        (void)owner.select(owner.m_gnss_time);
        break;
    case FrameAction::selection_magnetic:
        (void)owner.select(owner.m_magnetic_time);
        break;
    case FrameAction::selection_barometer:
        (void)owner.select(owner.m_barometer_time);
        break;
    case FrameAction::begin_fusion:
        (void)owner.begin_fusion(interval);
        break;
    case FrameAction::configure:
        (void)owner.configure(parameters);
        break;
    case FrameAction::next_fusion:
        (void)owner.next_fusion(interval);
        break;
    case FrameAction::views:
        (void)owner.gnss_source();
        (void)owner.magnetic_source();
        (void)owner.barometer_source();
        (void)owner.gnss_generation();
        (void)owner.magnetic_generation();
        (void)owner.barometer_generation();
        (void)owner.fusion_time_us();
        (void)owner.half_period_us();
        break;
    }
#if FORMAL_ESKF_PROOF_FRAME == 0
    __ESBMC_assert(owner.m_gnss.m_length == gnss_length, "Complete gnss metadata frame length");
    __ESBMC_assert(owner.m_gnss.m_count == gnss_count, "Complete gnss metadata frame count");
    __ESBMC_assert(owner.m_gnss.m_oldest == gnss_oldest, "Complete gnss metadata frame oldest");
    __ESBMC_assert(owner.m_gnss.m_write == gnss_write, "Complete gnss metadata frame write");
    for (std::size_t i = 0; i < 16; ++i)
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
#endif
#if FORMAL_ESKF_PROOF_FRAME == 1
    __ESBMC_assert(owner.m_magnetic.m_length == magnetic_length, "Complete magnetic metadata frame length");
    __ESBMC_assert(owner.m_magnetic.m_count == magnetic_count, "Complete magnetic metadata frame count");
    __ESBMC_assert(owner.m_magnetic.m_oldest == magnetic_oldest, "Complete magnetic metadata frame oldest");
    __ESBMC_assert(owner.m_magnetic.m_write == magnetic_write, "Complete magnetic metadata frame write");
    for (std::size_t i = 0; i < 64; ++i)
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
#endif
#if FORMAL_ESKF_PROOF_FRAME == 2
    __ESBMC_assert(owner.m_barometer.m_length == barometer_length, "Complete barometer metadata frame length");
    __ESBMC_assert(owner.m_barometer.m_count == barometer_count, "Complete barometer metadata frame count");
    __ESBMC_assert(owner.m_barometer.m_oldest == barometer_oldest, "Complete barometer metadata frame oldest");
    __ESBMC_assert(owner.m_barometer.m_write == barometer_write, "Complete barometer metadata frame write");
    for (std::size_t i = 0; i < 128; ++i)
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
#endif
#if FORMAL_ESKF_PROOF_FRAME == 3
    __ESBMC_assert(owner.m_context.m_length == context_length, "Complete context metadata frame length");
    __ESBMC_assert(owner.m_context.m_count == context_count, "Complete context metadata frame count");
    __ESBMC_assert(owner.m_context.m_oldest == context_oldest, "Complete context metadata frame oldest");
    __ESBMC_assert(owner.m_context.m_write == context_write, "Complete context metadata frame write");
    for (std::size_t i = 0; i < 128; ++i)
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
#endif
}
