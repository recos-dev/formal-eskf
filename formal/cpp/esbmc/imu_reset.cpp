// SPDX-License-Identifier: BSD-3-Clause
#include "imu_owner_support.hpp"
void verify_imu_storage_reset_owner_frame(imu_proof::Imu owner)
{
    // reset does not consume indices, so changed-length allocation may invoke it
    // with old indices/count outside the new length. No metadata restriction.
    auto const previous = owner.m_previous;
    auto const config = owner.m_configuration;
    auto const fusion = owner.m_fusion_time_us;
    auto const fault = owner.m_fault;
    auto const length = owner.m_samples.m_length;
    owner.m_samples.reset();
    __ESBMC_assert(imu_proof::same_input(owner.m_previous, previous) &&
                       imu_proof::same_configuration(owner.m_configuration, config) &&
                       owner.m_fusion_time_us == fusion && owner.m_fault == fault && owner.m_samples.m_length == length,
                   "Typed IMU reset preserves all enclosing owner fields and length");
    __ESBMC_assert(owner.m_samples.m_count == 0 && owner.m_samples.m_oldest == 0 && owner.m_samples.m_write == 0,
                   "Typed IMU reset clears all ring indices");
}

void verify_imu_storage_reset(imu_proof::Imu owner)
{
    // Same unrestricted input domain as the enclosing-owner frame profile.
    auto const length = owner.m_samples.m_length;
    owner.m_samples.reset();
    __ESBMC_assert(owner.m_samples.m_length == length && owner.m_samples.m_count == 0 &&
                       owner.m_samples.m_oldest == 0 && owner.m_samples.m_write == 0,
                   "Typed IMU reset preserves length and clears indices");
    imu_proof::check_zero_storage(owner.m_samples);
}
