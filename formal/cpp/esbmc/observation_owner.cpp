// SPDX-License-Identifier: BSD-3-Clause
#include "observation_support.hpp"
using observation_proof::Observations;
using observation_proof::same_double;
using observation_proof::same_timeline;
using observation_proof::Timeline;
namespace
{
void check_empty(Observations const & owner)
{
    __ESBMC_assert(same_timeline(owner.m_gnss_time, Timeline{}) && same_timeline(owner.m_magnetic_time, Timeline{}) &&
                       same_timeline(owner.m_barometer_time, Timeline{}) && owner.m_seed_time_us == 0 &&
                       owner.m_latest_time_us == 0 && owner.m_fusion_time_us == 0 && owner.m_advanced_at_us == 0 &&
                       owner.m_average_period_us == 4000.,
                   "Empty owner has cleared timelines and clocks");
    __ESBMC_assert(owner.m_gnss.m_count == 0 && owner.m_gnss.m_oldest == 0 && owner.m_gnss.m_write == 0,
                   "Empty gnss ring metadata");
    for (std::size_t i = 0; i < 16; ++i)
    {
        __ESBMC_assert(owner.m_gnss.m_samples[i].time_us == std::uint64_t{0}, "Cleared gnss field time_us");
        __ESBMC_assert(owner.m_gnss.m_samples[i].arrival_time_us == std::uint64_t{0},
                       "Cleared gnss field arrival_time_us");
        __ESBMC_assert(owner.m_gnss.m_samples[i].residual_delay_us == std::uint64_t{0},
                       "Cleared gnss field residual_delay_us");
        __ESBMC_assert(owner.m_gnss.m_samples[i].generation == std::uint64_t{0}, "Cleared gnss field generation");
        __ESBMC_assert(owner.m_gnss.m_samples[i].sample.stamp.time_us == std::uint64_t{0},
                       "Cleared gnss field sample.stamp.time_us");
        __ESBMC_assert(
            imu_proof::same(owner.m_gnss.m_samples[i].sample.velocity_n.m_storage.values.head, imu_proof::Scalar{0}),
            "Cleared gnss field sample.velocity_n.m_storage.values.head");
        __ESBMC_assert(imu_proof::same(owner.m_gnss.m_samples[i].sample.velocity_n.m_storage.values.tail.head,
                                       imu_proof::Scalar{0}),
                       "Cleared gnss field sample.velocity_n.m_storage.values.tail.head");
        __ESBMC_assert(imu_proof::same(owner.m_gnss.m_samples[i].sample.velocity_n.m_storage.values.tail.tail.head,
                                       imu_proof::Scalar{0}),
                       "Cleared gnss field sample.velocity_n.m_storage.values.tail.tail.head");
        __ESBMC_assert(imu_proof::same(owner.m_gnss.m_samples[i].sample.horizontal_accuracy_m, imu_proof::Scalar{0}),
                       "Cleared gnss field sample.horizontal_accuracy_m");
        __ESBMC_assert(imu_proof::same(owner.m_gnss.m_samples[i].sample.vertical_accuracy_m, imu_proof::Scalar{0}),
                       "Cleared gnss field sample.vertical_accuracy_m");
        __ESBMC_assert(imu_proof::same(owner.m_gnss.m_samples[i].sample.speed_accuracy_m_s, imu_proof::Scalar{0}),
                       "Cleared gnss field sample.speed_accuracy_m_s");
        __ESBMC_assert(imu_proof::same(owner.m_gnss.m_samples[i].sample.hdop, imu_proof::Scalar{0}),
                       "Cleared gnss field sample.hdop");
        __ESBMC_assert(imu_proof::same(owner.m_gnss.m_samples[i].sample.vdop, imu_proof::Scalar{0}),
                       "Cleared gnss field sample.vdop");
        __ESBMC_assert(same_double(owner.m_gnss.m_samples[i].sample.latitude_deg, double{0}),
                       "Cleared gnss field sample.latitude_deg");
        __ESBMC_assert(same_double(owner.m_gnss.m_samples[i].sample.longitude_deg, double{0}),
                       "Cleared gnss field sample.longitude_deg");
        __ESBMC_assert(same_double(owner.m_gnss.m_samples[i].sample.altitude_msl_m, double{0}),
                       "Cleared gnss field sample.altitude_msl_m");
        __ESBMC_assert(same_double(owner.m_gnss.m_samples[i].sample.altitude_ellipsoid_m, double{0}),
                       "Cleared gnss field sample.altitude_ellipsoid_m");
        __ESBMC_assert(owner.m_gnss.m_samples[i].half_period_us == std::uint32_t{0},
                       "Cleared gnss field half_period_us");
        __ESBMC_assert(owner.m_gnss.m_samples[i].sample.stamp.source.device_id == std::uint32_t{0},
                       "Cleared gnss field sample.stamp.source.device_id");
        __ESBMC_assert(owner.m_gnss.m_samples[i].sample.stamp.source.calibration_count == std::uint8_t{0},
                       "Cleared gnss field sample.stamp.source.calibration_count");
        __ESBMC_assert(owner.m_gnss.m_samples[i].sample.fix_type == std::uint8_t{0},
                       "Cleared gnss field sample.fix_type");
        __ESBMC_assert(owner.m_gnss.m_samples[i].sample.satellites == std::uint8_t{0},
                       "Cleared gnss field sample.satellites");
        __ESBMC_assert(owner.m_gnss.m_samples[i].sample.velocity_valid == bool{0},
                       "Cleared gnss field sample.velocity_valid");
        __ESBMC_assert(owner.m_gnss.m_samples[i].sample.spoofed == bool{0}, "Cleared gnss field sample.spoofed");
    }
    __ESBMC_assert(owner.m_magnetic.m_count == 0 && owner.m_magnetic.m_oldest == 0 && owner.m_magnetic.m_write == 0,
                   "Empty magnetic ring metadata");
    for (std::size_t i = 0; i < 64; ++i)
    {
        __ESBMC_assert(owner.m_magnetic.m_samples[i].time_us == std::uint64_t{0}, "Cleared magnetic field time_us");
        __ESBMC_assert(owner.m_magnetic.m_samples[i].arrival_time_us == std::uint64_t{0},
                       "Cleared magnetic field arrival_time_us");
        __ESBMC_assert(owner.m_magnetic.m_samples[i].residual_delay_us == std::uint64_t{0},
                       "Cleared magnetic field residual_delay_us");
        __ESBMC_assert(owner.m_magnetic.m_samples[i].generation == std::uint64_t{0},
                       "Cleared magnetic field generation");
        __ESBMC_assert(owner.m_magnetic.m_samples[i].sample.stamp.time_us == std::uint64_t{0},
                       "Cleared magnetic field sample.stamp.time_us");
        __ESBMC_assert(
            imu_proof::same(owner.m_magnetic.m_samples[i].sample.field_b.m_storage.values.head, imu_proof::Scalar{0}),
            "Cleared magnetic field sample.field_b.m_storage.values.head");
        __ESBMC_assert(imu_proof::same(owner.m_magnetic.m_samples[i].sample.field_b.m_storage.values.tail.head,
                                       imu_proof::Scalar{0}),
                       "Cleared magnetic field sample.field_b.m_storage.values.tail.head");
        __ESBMC_assert(imu_proof::same(owner.m_magnetic.m_samples[i].sample.field_b.m_storage.values.tail.tail.head,
                                       imu_proof::Scalar{0}),
                       "Cleared magnetic field sample.field_b.m_storage.values.tail.tail.head");
        __ESBMC_assert(owner.m_magnetic.m_samples[i].half_period_us == std::uint32_t{0},
                       "Cleared magnetic field half_period_us");
        __ESBMC_assert(owner.m_magnetic.m_samples[i].sample.stamp.source.device_id == std::uint32_t{0},
                       "Cleared magnetic field sample.stamp.source.device_id");
        __ESBMC_assert(owner.m_magnetic.m_samples[i].sample.stamp.source.calibration_count == std::uint8_t{0},
                       "Cleared magnetic field sample.stamp.source.calibration_count");
    }
    __ESBMC_assert(owner.m_barometer.m_count == 0 && owner.m_barometer.m_oldest == 0 && owner.m_barometer.m_write == 0,
                   "Empty barometer ring metadata");
    for (std::size_t i = 0; i < 128; ++i)
    {
        __ESBMC_assert(owner.m_barometer.m_samples[i].time_us == std::uint64_t{0}, "Cleared barometer field time_us");
        __ESBMC_assert(owner.m_barometer.m_samples[i].arrival_time_us == std::uint64_t{0},
                       "Cleared barometer field arrival_time_us");
        __ESBMC_assert(owner.m_barometer.m_samples[i].residual_delay_us == std::uint64_t{0},
                       "Cleared barometer field residual_delay_us");
        __ESBMC_assert(owner.m_barometer.m_samples[i].generation == std::uint64_t{0},
                       "Cleared barometer field generation");
        __ESBMC_assert(owner.m_barometer.m_samples[i].sample.stamp.time_us == std::uint64_t{0},
                       "Cleared barometer field sample.stamp.time_us");
        __ESBMC_assert(imu_proof::same(owner.m_barometer.m_samples[i].sample.altitude_m, imu_proof::Scalar{0}),
                       "Cleared barometer field sample.altitude_m");
        __ESBMC_assert(owner.m_barometer.m_samples[i].half_period_us == std::uint32_t{0},
                       "Cleared barometer field half_period_us");
        __ESBMC_assert(owner.m_barometer.m_samples[i].sample.stamp.source.device_id == std::uint32_t{0},
                       "Cleared barometer field sample.stamp.source.device_id");
        __ESBMC_assert(owner.m_barometer.m_samples[i].sample.stamp.source.calibration_count == std::uint8_t{0},
                       "Cleared barometer field sample.stamp.source.calibration_count");
    }
    __ESBMC_assert(owner.m_context.m_count == 0 && owner.m_context.m_oldest == 0 && owner.m_context.m_write == 0,
                   "Empty context ring metadata");
    for (std::size_t i = 0; i < 128; ++i)
    {
        __ESBMC_assert(owner.m_context.m_samples[i].time_us == std::uint64_t{0}, "Cleared context field time_us");
        __ESBMC_assert(owner.m_context.m_samples[i].land.time_us == std::uint64_t{0},
                       "Cleared context field land.time_us");
        __ESBMC_assert(owner.m_context.m_samples[i].vehicle.time_us == std::uint64_t{0},
                       "Cleared context field vehicle.time_us");
        __ESBMC_assert(owner.m_context.m_samples[i].land.known == bool{0}, "Cleared context field land.known");
        __ESBMC_assert(owner.m_context.m_samples[i].land.landed == bool{0}, "Cleared context field land.landed");
        __ESBMC_assert(owner.m_context.m_samples[i].land.at_rest == bool{0}, "Cleared context field land.at_rest");
        __ESBMC_assert(owner.m_context.m_samples[i].land.in_ground_effect == bool{0},
                       "Cleared context field land.in_ground_effect");
        __ESBMC_assert(owner.m_context.m_samples[i].vehicle.known == bool{0}, "Cleared context field vehicle.known");
        __ESBMC_assert(owner.m_context.m_samples[i].vehicle.armed == bool{0}, "Cleared context field vehicle.armed");
        __ESBMC_assert(owner.m_context.m_samples[i].vehicle.fixed_wing == bool{0},
                       "Cleared context field vehicle.fixed_wing");
    }
}
} // namespace

void verify_observation_constructor()
{
    Observations owner;
    check_empty(owner);
    __ESBMC_assert(owner.m_gnss.m_length == 16 && owner.m_magnetic.m_length == 64 &&
                       owner.m_barometer.m_length == 128 && owner.m_context.m_length == 128,
                   "Constructor allocates exact profile capacities");
    __ESBMC_assert(!owner.m_configured &&
                       imu_proof::same_configuration(owner.m_configuration, formal_eskf::px4::TimingConfiguration{}),
                   "Constructor has default configuration awaiting validation");
}

void verify_observation_reset(Observations owner)
{
    auto const configuration = owner.m_configuration;
    auto const configured = owner.m_configured;
    auto const gnss_length = owner.m_gnss.m_length;
    auto const magnetic_length = owner.m_magnetic.m_length;
    auto const barometer_length = owner.m_barometer.m_length;
    auto const context_length = owner.m_context.m_length;
    owner.reset();
    check_empty(owner);
    __ESBMC_assert(owner.m_gnss.m_length == gnss_length && owner.m_magnetic.m_length == magnetic_length &&
                       owner.m_barometer.m_length == barometer_length && owner.m_context.m_length == context_length,
                   "Owner reset retains allocated lengths without metadata preconditions");
    __ESBMC_assert(owner.m_configured == configured &&
                       imu_proof::same_configuration(owner.m_configuration, configuration),
                   "Owner reset retains complete configuration");
}
