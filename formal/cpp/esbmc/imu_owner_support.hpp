// SPDX-License-Identifier: BSD-3-Clause
#pragma once

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

namespace imu_proof
{
using Backend = contract_proof::Linalg;
using Scalar = contract_proof::Scalar;
using Imu = formal_eskf::px4::ImuHistory<Backend>;
using Input = Imu::input_type;
using Interval = Imu::Interval;
using Ring = formal_eskf::runtime::RingBuffer<Interval, 64>;
using Configuration = formal_eskf::px4::TimingConfiguration;
constexpr std::size_t capacity = 64;
static_assert(formal_eskf::px4::profile::history::imu_capacity == capacity);
extern unsigned char nondet_index();

inline void metadata(Ring & ring)
{
    auto const length = nondet_index();
    auto const count = nondet_index();
    auto const oldest = nondet_index();
    __ESBMC_assume(length <= capacity && count <= length && (length == 0 ? oldest == 0 : oldest < length));
    ring.m_length = length;
    ring.m_count = count;
    ring.m_oldest = oldest;
    auto const sum = static_cast<std::size_t>(oldest) + count;
    ring.m_write = sum < length ? sum : sum - length;
}
inline bool same(Scalar a, Scalar b)
{
#if FORMAL_ESKF_PROOF_BINARY64
    using Bits = std::uint64_t;
#else
    using Bits = std::uint32_t;
#endif
    return (__builtin_bit_cast(Bits, a) == __builtin_bit_cast(Bits, b)) | (std::isnan(a) & std::isnan(b));
}
inline bool same_source(formal_eskf::px4::ImuSource const & a, formal_eskf::px4::ImuSource const & b)
{
    return a.accelerometer_id == b.accelerometer_id && a.gyroscope_id == b.gyroscope_id &&
           a.accelerometer_calibration == b.accelerometer_calibration &&
           a.gyroscope_calibration == b.gyroscope_calibration;
}
inline bool same_input(Input const & a, Input const & b)
{
    bool result = a.time_us == b.time_us && a.angular_period_us == b.angular_period_us &&
                  a.acceleration_period_us == b.acceleration_period_us &&
                  a.accelerometer_clipping == b.accelerometer_clipping &&
                  a.gyroscope_clipping == b.gyroscope_clipping && same_source(a.source, b.source);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        result = same(a.delta_angle_b(axis), b.delta_angle_b(axis)) &
                 same(a.delta_velocity_b(axis), b.delta_velocity_b(axis)) & result;
    }
    return result;
}
inline bool same_configuration(Configuration const & a, Configuration const & b)
{
    return a.minimum_lag_us == b.minimum_lag_us && a.gnss_delay_us == b.gnss_delay_us &&
           a.magnetic_delay_us == b.magnetic_delay_us && a.barometer_delay_us == b.barometer_delay_us &&
           a.imu_length == b.imu_length;
}

// Keep large homogeneous snapshot arrays directly in the entry-point scope.
// ESBMC's nested snapshot objects/references expand aliases or exhaust the
// solver even for a no-op. Explicit field frames preserve the complete domain.
inline void check_zero_input(Input const & input)
{
    __ESBMC_assert(input.time_us == 0 && input.angular_period_us == 0 && input.acceleration_period_us == 0 &&
                       input.accelerometer_clipping == 0 && input.gyroscope_clipping == 0 &&
                       input.source.accelerometer_id == 0 && input.source.gyroscope_id == 0 &&
                       input.source.accelerometer_calibration == 0 && input.source.gyroscope_calibration == 0,
                   "IMU empty input has zero integer fields");
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        __ESBMC_assert(same(input.delta_angle_b(axis), Scalar{0}) && same(input.delta_velocity_b(axis), Scalar{0}),
                       "IMU empty input has positive-zero coefficients");
    }
}
inline void check_zero_storage(Ring const & ring)
{
    for (std::size_t i = 0; i < capacity; ++i)
    {
        auto const & interval = ring.m_samples[i];
        __ESBMC_assert(interval.time_us == 0 && interval.start_time_us == 0 && interval.hold_period_us == 0,
                       "IMU empty interval integers");
        check_zero_input(interval.sample);
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            __ESBMC_assert(same(interval.held_rate.specific_force_b(axis), Scalar{0}) &&
                               same(interval.held_rate.angular_rate_b(axis), Scalar{0}),
                           "IMU empty interval held-rate coefficients");
        }
    }
}
} // namespace imu_proof
