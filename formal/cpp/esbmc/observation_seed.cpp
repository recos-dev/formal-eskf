// SPDX-License-Identifier: BSD-3-Clause
#include "observation_frame_support.hpp"
static_assert(FORMAL_ESKF_PROOF_FRAME >= 0 && FORMAL_ESKF_PROOF_FRAME < 4);
using observation_proof::Observations;
using observation_proof::same_context;
using observation_proof::Timeline;
namespace
{
Observations::Context expected_context;
Observations::Context published_context;
unsigned push_calls;
} // namespace

// The actual seed/snapshot caller is executed. Only the typed push is summarized:
// it writes an arbitrary context to the one permitted cell, retains all other
// storage, and supplies the producer's exact empty-to-one metadata relation.
// The typed producer separately proves the inserted value equations.
template <>
void formal_eskf::runtime::RingBuffer<Observations::Context, 128>::push(Observations::Context const & sample)
{
    check_metadata();
    check_storage();
    __ESBMC_assert(this == &expected_owner->m_context && m_length == 128 && m_count == 0 && m_oldest == 0 &&
                       m_write == 0 && push_calls == 0,
                   "Seed calls typed context producer from empty metadata");
    __ESBMC_assert(same_context(sample, expected_context), "Seed forwards the complete expected snapshot");
    for (std::size_t i = 0; i < 128; ++i)
    {
        __ESBMC_assert(&sample != &m_samples[i], "Seed supplies an external context snapshot to storage");
    }
    __ESBMC_init_object(&published_context);
    m_samples[0].time_us = published_context.time_us;
    m_samples[0].land.time_us = published_context.land.time_us;
    m_samples[0].land.known = published_context.land.known;
    m_samples[0].land.landed = published_context.land.landed;
    m_samples[0].land.at_rest = published_context.land.at_rest;
    m_samples[0].land.in_ground_effect = published_context.land.in_ground_effect;
    m_samples[0].vehicle.time_us = published_context.vehicle.time_us;
    m_samples[0].vehicle.known = published_context.vehicle.known;
    m_samples[0].vehicle.armed = published_context.vehicle.armed;
    m_samples[0].vehicle.fixed_wing = published_context.vehicle.fixed_wing;
    m_count = 1;
    m_write = 1;
    expected_ring_metadata[3][1] = 1;
    expected_ring_metadata[3][3] = 1;
#if FORMAL_ESKF_PROOF_FRAME == 3
    capture_context_storage();
#endif
    ++push_calls;
}

void verify_observation_seed(Observations owner, Observations::Seed initial)
{
    auto const count = imu_proof::nondet_index();
    auto const oldest = imu_proof::nondet_index();
    __ESBMC_assume(count <= 128 && oldest < 128);
    owner.m_context.m_length = 128;
    owner.m_context.m_count = count;
    owner.m_context.m_oldest = oldest;
    owner.m_context.m_write = (static_cast<std::size_t>(oldest) + count) % 128;
    __ESBMC_assume(owner.m_seed_time_us == 0 ? (count == 0 && oldest == 0) : count > 0);
    auto const input = initial;
    auto const epoch = initial.context.time_us;
    bool const sources = initial.gnss.time_us <= epoch && initial.magnetic.time_us <= epoch &&
                         initial.barometer.time_us <= epoch &&
                         (initial.gnss.source.device_id == 0 || initial.gnss.time_us != 0) &&
                         (initial.magnetic.source.device_id == 0 || initial.magnetic.time_us != 0) &&
                         (initial.barometer.source.device_id == 0 || initial.barometer.time_us != 0);
    bool const expected = owner.m_configured && owner.m_seed_time_us == 0 && epoch != 0 && sources;
    expected_owner = &owner;
    push_calls = 0;
    expected_context = {};
    expected_context.time_us = epoch;
    if (initial.context.land.time_us > 0 && initial.context.land.time_us <= epoch)
    {
        expected_context.land = initial.context.land;
    }
    if (initial.context.vehicle.time_us > 0 && initial.context.vehicle.time_us <= epoch)
    {
        expected_context.vehicle = initial.context.vehicle;
    }
    capture_metadata();
    capture_storage();
    bool const result = owner.seed(initial);
    __ESBMC_assert(result == expected && push_calls == static_cast<unsigned>(expected),
                   "Seed exact guard and storage call count");
    if (expected)
    {
        expected_gnss_time = {input.gnss.source, input.gnss.time_us};
        expected_magnetic_time = {input.magnetic.source, input.magnetic.time_us};
        expected_barometer_time = {input.barometer.source, input.barometer.time_us};
        expected_seed = epoch;
        expected_latest = epoch;
        expected_fusion = epoch;
    }
    check_metadata();
    check_storage();
    __ESBMC_assert(same_context(initial.context, input.context) && initial.gnss.time_us == input.gnss.time_us &&
                       initial.gnss.source.device_id == input.gnss.source.device_id &&
                       initial.gnss.source.calibration_count == input.gnss.source.calibration_count &&
                       initial.magnetic.time_us == input.magnetic.time_us &&
                       initial.magnetic.source.device_id == input.magnetic.source.device_id &&
                       initial.magnetic.source.calibration_count == input.magnetic.source.calibration_count &&
                       initial.barometer.time_us == input.barometer.time_us &&
                       initial.barometer.source.device_id == input.barometer.source.device_id &&
                       initial.barometer.source.calibration_count == input.barometer.source.calibration_count,
                   "Seed preserves complete external input");
}
