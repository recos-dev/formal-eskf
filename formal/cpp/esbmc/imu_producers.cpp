// SPDX-License-Identifier: BSD-3-Clause

#include "imu_owner_support.hpp"

using imu_proof::Input;
using imu_proof::Scalar;

void verify_imu_mean_rates(Input input)
{
    __ESBMC_assume(input.angular_period_us >= 3800 && input.angular_period_us <= 4200 &&
                   input.acceleration_period_us >= 3800 && input.acceleration_period_us <= 4200);
    auto const before = input;
    Scalar const angle_dt = static_cast<Scalar>(input.angular_period_us) * static_cast<Scalar>(1e-6);
    Scalar const velocity_dt = static_cast<Scalar>(input.acceleration_period_us) * static_cast<Scalar>(1e-6);
    auto const result = input.mean_rates();
    __ESBMC_assert(angle_dt > 0 && velocity_dt > 0, "Admitted IMU periods produce positive divisors");
    // The input frame is a separate obligation; observe unchanged source fields
    // directly so the coefficient oracle does not reuse Matrix::operator().
    __ESBMC_assert(imu_proof::same_input(input, before), "IMU mean rates preserve complete input");
    __ESBMC_assert(imu_proof::same(result.angular_rate_b.m_storage.values.head,
                                   input.delta_angle_b.m_storage.values.head / angle_dt),
                   "IMU mean rate angular_rate_b coefficient 0");
    __ESBMC_assert(imu_proof::same(result.angular_rate_b.m_storage.values.tail.head,
                                   input.delta_angle_b.m_storage.values.tail.head / angle_dt),
                   "IMU mean rate angular_rate_b coefficient 1");
    __ESBMC_assert(imu_proof::same(result.angular_rate_b.m_storage.values.tail.tail.head,
                                   input.delta_angle_b.m_storage.values.tail.tail.head / angle_dt),
                   "IMU mean rate angular_rate_b coefficient 2");
    __ESBMC_assert(imu_proof::same(result.specific_force_b.m_storage.values.head,
                                   input.delta_velocity_b.m_storage.values.head / velocity_dt),
                   "IMU mean rate specific_force_b coefficient 0");
    __ESBMC_assert(imu_proof::same(result.specific_force_b.m_storage.values.tail.head,
                                   input.delta_velocity_b.m_storage.values.tail.head / velocity_dt),
                   "IMU mean rate specific_force_b coefficient 1");
    __ESBMC_assert(imu_proof::same(result.specific_force_b.m_storage.values.tail.tail.head,
                                   input.delta_velocity_b.m_storage.values.tail.tail.head / velocity_dt),
                   "IMU mean rate specific_force_b coefficient 2");
}

void verify_imu_owner_equality(Scalar a, Scalar b)
{
    __ESBMC_assert(imu_proof::same(a, b) == contract_proof::same(a, b),
                   "IMU owner frame equality retains values, signed zero and NaN classification");
}

void verify_imu_timing_input_frame(formal_eskf::px4::TimingParameters parameters, imu_proof::Configuration output)
{
    auto const before = parameters;
    (void)formal_eskf::px4::configure_timing(parameters, output);
    __ESBMC_assert(parameters.filter_interval_us == before.filter_interval_us &&
                       parameters.fusion_delay_ms == before.fusion_delay_ms &&
                       parameters.gnss_delay_ms == before.gnss_delay_ms &&
                       parameters.magnetic_delay_ms == before.magnetic_delay_ms &&
                       parameters.barometer_delay_ms == before.barometer_delay_ms,
                   "Timing producer preserves every caller parameter");
}
