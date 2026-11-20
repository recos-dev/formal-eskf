// SPDX-License-Identifier: BSD-3-Clause
#include "observation_frame_support.hpp"
static_assert(FORMAL_ESKF_PROOF_FRAME >= 0 && FORMAL_ESKF_PROOF_FRAME < 4);
using observation_proof::Observations;
using observation_proof::same_context;
using observation_proof::same_double;
using observation_proof::same_interval;
using observation_proof::same_land;
using observation_proof::same_timeline;
using observation_proof::same_vehicle;
using Context = Observations::Context;
namespace
{
Context expected_record;
Observations::LandState expected_land;
Observations::VehicleState expected_vehicle;
std::uint64_t expected_time;
unsigned snapshots, pushes;
std::size_t expected_previous_index;
} // namespace

// Execute the actual newest-view function. Its returned reference must retain
// its real alias into owner storage; a fresh global reference is not a substitute.
// The complete pre-call frame is checked for every observed slot in the
// mandatory suite. Together with exact alias identity, it supplies the original
// previous-context value; do not replace it with a fresh reference.
// Snapshot itself returns an arbitrary value under its separately proved pure
// contract; no timestamp/content formula is assumed by this caller proof.
template <>
Context
formal_eskf::px4::ObservationHistory<imu_proof::Backend>::snapshot(Context const & previous, std::uint64_t time_us,
                                                                   LandState const & land, VehicleState const & vehicle)
{
    check_metadata();
    check_storage();
    __ESBMC_assert(&previous == &expected_owner->m_context.m_samples[expected_previous_index] &&
                       expected_owner->m_context.m_count > 0 && time_us == expected_time &&
                       same_land(land, expected_land) && same_vehicle(vehicle, expected_vehicle) && pushes == 0 &&
                       snapshots == 0,
                   "Record forwards complete newest context and original inputs to snapshot before storage");
    ++snapshots;
    return expected_record;
}
template <> void formal_eskf::runtime::RingBuffer<Context, 128>::push(Context const & record)
{
    check_metadata();
    check_storage();
    __ESBMC_assert(this == &expected_owner->m_context && pushes == 0 && snapshots == 1 &&
                       same_context(record, expected_record),
                   "Record forwards the unmodified producer snapshot once");
    __ESBMC_assert(m_length == 128 && m_count <= 128 && m_oldest < 128 && m_write < 128 &&
                       m_write == (m_oldest + m_count) % 128,
                   "Record meets actual storage metadata domain");
    for (std::size_t i = 0; i < 128; ++i)
    {
        __ESBMC_assert(&record != &m_samples[i], "Record supplies an external completed snapshot");
    }
    // Havoc the enclosing root deliberately. Only actual producer frame
    // conditions are restored; payload and changed metadata remain arbitrary.
    __ESBMC_init_object(expected_owner);
    expected_ring_metadata[3][1] = expected_owner->m_context.m_count;
    expected_ring_metadata[3][2] = expected_owner->m_context.m_oldest;
    expected_ring_metadata[3][3] = expected_owner->m_context.m_write;
    assume_metadata();
    ++pushes;
#if FORMAL_ESKF_PROOF_FRAME == 3
    capture_storage();
#else
    assume_storage();
#endif
}

void verify_context_record(Observations owner, imu_proof::Interval interval, Observations::LandState land,
                           Observations::VehicleState vehicle)
{
    auto const count = imu_proof::nondet_index();
    auto const oldest = imu_proof::nondet_index();
    __ESBMC_assume(count <= 128 && oldest < 128);
    owner.m_context.m_length = 128;
    owner.m_context.m_count = count;
    owner.m_context.m_oldest = oldest;
    owner.m_context.m_write = (static_cast<std::size_t>(oldest) + count) % 128;
    __ESBMC_assume(owner.m_seed_time_us == 0 ? (count == 0 && oldest == 0) : count > 0);
    auto const old_interval = interval;
    auto const old_land = land;
    auto const old_vehicle = vehicle;
    auto const latest = owner.m_latest_time_us;
    bool const ordered = owner.m_seed_time_us != 0 && interval.start_time_us == latest &&
                         interval.time_us == interval.sample.time_us && interval.time_us > latest;
    bool const expected = ordered && interval.time_us - latest >= 3800 && interval.time_us - latest <= 50000;
    expected_owner = &owner;
    pushes = 0;
    snapshots = 0;
    expected_previous_index = owner.m_context.m_write == 0 ? 127 : owner.m_context.m_write - 1;
    __ESBMC_init_object(&expected_record);
    expected_land = land;
    expected_vehicle = vehicle;
    expected_time = interval.time_us;
    capture_storage();
    capture_metadata();
    bool const result = owner.record_imu(interval, land, vehicle);
    __ESBMC_assert(result == expected && pushes == static_cast<unsigned>(expected) && snapshots == pushes,
                   "Record exact eligibility and producer call order");
    __ESBMC_assert(same_interval(interval, old_interval) && same_land(land, old_land) &&
                       same_vehicle(vehicle, old_vehicle),
                   "Record complete external input frame");
    expected_latest = expected ? interval.time_us : latest;
    check_metadata();
    check_storage();
}
