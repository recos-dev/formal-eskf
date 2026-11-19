// SPDX-License-Identifier: BSD-3-Clause

#include "contracts.hpp"
#include "delayed_library.hpp"
#include "../../../integrations/px4/module/ImuPrediction.hpp"
#include "../../../integrations/px4/module/DeploymentProfile.hpp"
// Expose metadata to state the view precondition; retain every production body.
#define private public
#define max formal_eskf_proof_max
#include "../../../integrations/px4/module/ImuHistory.hpp"
#undef max
#include "../../../integrations/px4/module/OutputPredictor.hpp"
#undef private

using Backend = contract_proof::Linalg;
using Imu = formal_eskf::px4::ImuHistory<Backend>;
using Output = formal_eskf::px4::OutputPredictor<Backend>;
namespace profile = formal_eskf::px4::profile;
static_assert(profile::history::imu_capacity == 64);
extern unsigned char nondet_index();

// Every valid ring-metadata tuple is represented. Payloads and timestamps are
// arbitrary; no chronological, finite-value or successful-history premise is
// required by these views. Production storage retains its actual capacity.
template <typename Sample, std::size_t Capacity>
void seed_metadata(formal_eskf::runtime::RingBuffer<Sample, Capacity> & ring)
{
    static_assert(Capacity <= 255);
    auto const length = nondet_index();
    auto const oldest = nondet_index();
    auto const count = nondet_index();
    __ESBMC_assume(length <= Capacity && count <= length && (length == 0 ? oldest == 0 : oldest < length));
    ring.m_length = length;
    ring.m_count = count;
    ring.m_oldest = oldest;
    auto const sum = static_cast<std::size_t>(oldest) + count;
    ring.m_write = sum < length ? sum : sum - length;
}

void verify_imu_views(Imu owner)
{
    seed_metadata(owner.m_samples);
    auto const count = owner.m_samples.m_count;
    auto const length = owner.m_samples.m_length;
    auto const oldest = owner.m_samples.m_oldest;
    auto const write = owner.m_samples.m_write;
    auto const fault = owner.m_fault;
    auto const fusion = owner.m_fusion_time_us;
    auto const previous = owner.m_previous.time_us;
    bool const visible = fault == Imu::Fault::none && count != 0;
    auto const * latest = owner.latest();
    // Derive the logical last slot independently of the production write cursor.
    auto newest = oldest;
    if (count != 0)
    {
        newest += count - 1;
        if (newest >= length)
        {
            newest -= length;
        }
    }
    __ESBMC_assert(latest == (visible ? &owner.m_samples.m_samples[newest] : nullptr),
                   "IMU latest actual slot identity");
    bool const pending = fault == Imu::Fault::none && length != 0 && count == length &&
                         owner.m_samples.m_samples[oldest].time_us > fusion;
    __ESBMC_assert(owner.delayed() == (pending ? &owner.m_samples.m_samples[oldest] : nullptr),
                   "IMU delayed actual slot identity");
    __ESBMC_assert(&owner.configuration() == &owner.m_configuration && &owner.source() == &owner.m_previous.source,
                   "IMU configuration and source identity");
    __ESBMC_assert(owner.entries() == count && owner.fault() == fault && owner.fusion_time_us() == fusion &&
                       owner.latest_time_us() == previous,
                   "IMU scalar accessors");
    __ESBMC_assert(owner.m_samples.m_length == length && owner.m_samples.m_oldest == oldest &&
                       owner.m_samples.m_write == write,
                   "IMU metadata preserved");
}

void verify_output_views(Output owner)
{
    seed_metadata(owner.m_history);
    auto const count = owner.m_history.m_count;
    auto const length = owner.m_history.m_length;
    auto const oldest = owner.m_history.m_oldest;
    auto const write = owner.m_history.m_write;
    auto const fault = owner.m_fault;
    auto const fusion = owner.m_fusion_time_us;
    bool const tracking = owner.m_tracking;
    bool const pending = fault == Output::Fault::none && length != 0 && count == length &&
                         owner.m_history.m_samples[oldest].time_us > fusion;
    __ESBMC_assert(owner.delayed() == (pending ? &owner.m_history.m_samples[oldest] : nullptr),
                   "Output delayed actual slot identity");
    __ESBMC_assert(&owner.current() == &owner.m_current && &owner.tracking_error() == &owner.m_error &&
                       &owner.angular_rate_b() == &owner.m_output_angular_rate_b,
                   "Output current feedback and rate identity");
    __ESBMC_assert(owner.ready() == (tracking && fault == Output::Fault::none) && owner.entries() == count &&
                       owner.fault() == fault && owner.fusion_time_us() == fusion,
                   "Output scalar accessors");
    __ESBMC_assert(owner.m_history.m_length == length && owner.m_history.m_oldest == oldest &&
                       owner.m_history.m_write == write,
                   "Output metadata preserved");
}
