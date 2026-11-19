// SPDX-License-Identifier: BSD-3-Clause

#include "imu_owner_support.hpp"

using imu_proof::Configuration;
using imu_proof::Imu;
using imu_proof::Input;
using imu_proof::same_configuration;
using imu_proof::same_input;

void verify_imu_completion(Imu owner, std::uint64_t time)
{
    imu_proof::metadata(owner.m_samples);
    std::uint64_t integers[imu_proof::capacity][12];
    imu_proof::Scalar values[imu_proof::capacity][12];
    for (std::size_t i = 0; i < imu_proof::capacity; ++i)
    {
        integers[i][0] = owner.m_samples.m_samples[i].time_us;
        integers[i][1] = owner.m_samples.m_samples[i].start_time_us;
        integers[i][2] = owner.m_samples.m_samples[i].hold_period_us;
        integers[i][3] = owner.m_samples.m_samples[i].sample.time_us;
        integers[i][4] = owner.m_samples.m_samples[i].sample.angular_period_us;
        integers[i][5] = owner.m_samples.m_samples[i].sample.acceleration_period_us;
        integers[i][6] = owner.m_samples.m_samples[i].sample.accelerometer_clipping;
        integers[i][7] = owner.m_samples.m_samples[i].sample.gyroscope_clipping;
        integers[i][8] = owner.m_samples.m_samples[i].sample.source.accelerometer_id;
        integers[i][9] = owner.m_samples.m_samples[i].sample.source.gyroscope_id;
        integers[i][10] = owner.m_samples.m_samples[i].sample.source.accelerometer_calibration;
        integers[i][11] = owner.m_samples.m_samples[i].sample.source.gyroscope_calibration;
        values[i][0] = owner.m_samples.m_samples[i].sample.delta_angle_b.m_storage.values.head;
        values[i][1] = owner.m_samples.m_samples[i].sample.delta_angle_b.m_storage.values.tail.head;
        values[i][2] = owner.m_samples.m_samples[i].sample.delta_angle_b.m_storage.values.tail.tail.head;
        values[i][3] = owner.m_samples.m_samples[i].sample.delta_velocity_b.m_storage.values.head;
        values[i][4] = owner.m_samples.m_samples[i].sample.delta_velocity_b.m_storage.values.tail.head;
        values[i][5] = owner.m_samples.m_samples[i].sample.delta_velocity_b.m_storage.values.tail.tail.head;
        values[i][6] = owner.m_samples.m_samples[i].held_rate.specific_force_b.m_storage.values.head;
        values[i][7] = owner.m_samples.m_samples[i].held_rate.specific_force_b.m_storage.values.tail.head;
        values[i][8] = owner.m_samples.m_samples[i].held_rate.specific_force_b.m_storage.values.tail.tail.head;
        values[i][9] = owner.m_samples.m_samples[i].held_rate.angular_rate_b.m_storage.values.head;
        values[i][10] = owner.m_samples.m_samples[i].held_rate.angular_rate_b.m_storage.values.tail.head;
        values[i][11] = owner.m_samples.m_samples[i].held_rate.angular_rate_b.m_storage.values.tail.tail.head;
    }
    auto const previous = owner.m_previous;
    auto const config = owner.m_configuration;
    auto const fusion = owner.m_fusion_time_us;
    auto const fault = owner.m_fault;
    auto const length = owner.m_samples.m_length;
    auto const count = owner.m_samples.m_count;
    auto const oldest = owner.m_samples.m_oldest;
    auto const write = owner.m_samples.m_write;
    bool const eligible = fault == Imu::Fault::none && length > 0 && count == length &&
                          owner.m_samples.m_samples[oldest].time_us > fusion &&
                          time == owner.m_samples.m_samples[oldest].time_us &&
                          owner.m_samples.m_samples[oldest].start_time_us == fusion;
    bool const result = owner.complete(time);
    __ESBMC_assert(result == eligible && owner.m_fusion_time_us == (eligible ? time : fusion) &&
                       owner.m_fault ==
                           (fault != Imu::Fault::none || eligible ? fault : Imu::Fault::invalid_completion),
                   "IMU completion exact contiguous oldest acknowledgment and fault precedence");
    __ESBMC_assert(same_input(owner.m_previous, previous) && same_configuration(owner.m_configuration, config) &&
                       owner.m_samples.m_length == length && owner.m_samples.m_count == count &&
                       owner.m_samples.m_oldest == oldest && owner.m_samples.m_write == write,
                   "IMU completion preserves previous input, configuration and ring indices");

    for (std::size_t i = 0; i < imu_proof::capacity; ++i)
    {
        __ESBMC_assert(owner.m_samples.m_samples[i].time_us == integers[i][0], "IMU owner preserves storage time_us");
        __ESBMC_assert(owner.m_samples.m_samples[i].start_time_us == integers[i][1],
                       "IMU owner preserves storage start_time_us");
        __ESBMC_assert(owner.m_samples.m_samples[i].hold_period_us == integers[i][2],
                       "IMU owner preserves storage hold_period_us");
        __ESBMC_assert(owner.m_samples.m_samples[i].sample.time_us == integers[i][3],
                       "IMU owner preserves storage sample.time_us");
        __ESBMC_assert(owner.m_samples.m_samples[i].sample.angular_period_us == integers[i][4],
                       "IMU owner preserves storage sample.angular_period_us");
        __ESBMC_assert(owner.m_samples.m_samples[i].sample.acceleration_period_us == integers[i][5],
                       "IMU owner preserves storage sample.acceleration_period_us");
        __ESBMC_assert(owner.m_samples.m_samples[i].sample.accelerometer_clipping == integers[i][6],
                       "IMU owner preserves storage sample.accelerometer_clipping");
        __ESBMC_assert(owner.m_samples.m_samples[i].sample.gyroscope_clipping == integers[i][7],
                       "IMU owner preserves storage sample.gyroscope_clipping");
        __ESBMC_assert(owner.m_samples.m_samples[i].sample.source.accelerometer_id == integers[i][8],
                       "IMU owner preserves storage sample.source.accelerometer_id");
        __ESBMC_assert(owner.m_samples.m_samples[i].sample.source.gyroscope_id == integers[i][9],
                       "IMU owner preserves storage sample.source.gyroscope_id");
        __ESBMC_assert(owner.m_samples.m_samples[i].sample.source.accelerometer_calibration == integers[i][10],
                       "IMU owner preserves storage sample.source.accelerometer_calibration");
        __ESBMC_assert(owner.m_samples.m_samples[i].sample.source.gyroscope_calibration == integers[i][11],
                       "IMU owner preserves storage sample.source.gyroscope_calibration");
        __ESBMC_assert(
            imu_proof::same(owner.m_samples.m_samples[i].sample.delta_angle_b.m_storage.values.head, values[i][0]),
            "IMU owner preserves storage coefficient 0");
        __ESBMC_assert(
            imu_proof::same(owner.m_samples.m_samples[i].sample.delta_angle_b.m_storage.values.tail.head, values[i][1]),
            "IMU owner preserves storage coefficient 1");
        __ESBMC_assert(
            imu_proof::same(owner.m_samples.m_samples[i].sample.delta_angle_b.m_storage.values.tail.tail.head,
                            values[i][2]),
            "IMU owner preserves storage coefficient 2");
        __ESBMC_assert(
            imu_proof::same(owner.m_samples.m_samples[i].sample.delta_velocity_b.m_storage.values.head, values[i][3]),
            "IMU owner preserves storage coefficient 3");
        __ESBMC_assert(imu_proof::same(owner.m_samples.m_samples[i].sample.delta_velocity_b.m_storage.values.tail.head,
                                       values[i][4]),
                       "IMU owner preserves storage coefficient 4");
        __ESBMC_assert(
            imu_proof::same(owner.m_samples.m_samples[i].sample.delta_velocity_b.m_storage.values.tail.tail.head,
                            values[i][5]),
            "IMU owner preserves storage coefficient 5");
        __ESBMC_assert(imu_proof::same(owner.m_samples.m_samples[i].held_rate.specific_force_b.m_storage.values.head,
                                       values[i][6]),
                       "IMU owner preserves storage coefficient 6");
        __ESBMC_assert(
            imu_proof::same(owner.m_samples.m_samples[i].held_rate.specific_force_b.m_storage.values.tail.head,
                            values[i][7]),
            "IMU owner preserves storage coefficient 7");
        __ESBMC_assert(
            imu_proof::same(owner.m_samples.m_samples[i].held_rate.specific_force_b.m_storage.values.tail.tail.head,
                            values[i][8]),
            "IMU owner preserves storage coefficient 8");
        __ESBMC_assert(
            imu_proof::same(owner.m_samples.m_samples[i].held_rate.angular_rate_b.m_storage.values.head, values[i][9]),
            "IMU owner preserves storage coefficient 9");
        __ESBMC_assert(imu_proof::same(owner.m_samples.m_samples[i].held_rate.angular_rate_b.m_storage.values.tail.head,
                                       values[i][10]),
                       "IMU owner preserves storage coefficient 10");
        __ESBMC_assert(
            imu_proof::same(owner.m_samples.m_samples[i].held_rate.angular_rate_b.m_storage.values.tail.tail.head,
                            values[i][11]),
            "IMU owner preserves storage coefficient 11");
    }
}

void verify_imu_reset(Imu owner)
{
    imu_proof::metadata(owner.m_samples);
    auto const config = owner.m_configuration;
    auto const length = owner.m_samples.m_length;
    owner.reset();
    __ESBMC_assert(same_configuration(owner.m_configuration, config) && owner.m_samples.m_length == length &&
                       owner.m_samples.m_count == 0 && owner.m_samples.m_oldest == 0 && owner.m_samples.m_write == 0 &&
                       owner.m_fusion_time_us == 0 && owner.m_fault == Imu::Fault::none,
                   "IMU reset preserves configuration and clears temporal state");
    imu_proof::check_zero_input(owner.m_previous);
    imu_proof::check_zero_storage(owner.m_samples);
}
