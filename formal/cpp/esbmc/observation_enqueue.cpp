// SPDX-License-Identifier: BSD-3-Clause
#include "observation_frame_support.hpp"

static_assert(FORMAL_ESKF_PROOF_FRAME >= 0 && FORMAL_ESKF_PROOF_FRAME < 4);
static_assert(FORMAL_ESKF_PROOF_OBSERVATION_SOURCE >= 0 && FORMAL_ESKF_PROOF_OBSERVATION_SOURCE < 3);

using observation_proof::Admission;
using observation_proof::Observations;
using observation_proof::same_double;
using observation_proof::same_timeline;
using observation_proof::Timeline;
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
using Record = Observations::GnssObservation;
using Sample = Observations::GnssSample;
constexpr std::size_t capacity = 16;
#define SELECTED_RING m_gnss
#define SELECTED_TIME m_gnss_time
#define PUSH_SOURCE push_gnss
#define DELAY_FIELD gnss_delay_us
constexpr std::uint64_t arrival_age = 150000;
constexpr std::uint64_t spacing = 50000;
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
using Record = Observations::MagneticObservation;
using Sample = Observations::MagneticSample;
constexpr std::size_t capacity = 64;
#define SELECTED_RING m_magnetic
#define SELECTED_TIME m_magnetic_time
#define PUSH_SOURCE push_magnetic
#define DELAY_FIELD magnetic_delay_us
constexpr std::uint64_t arrival_age = 100000;
constexpr std::uint64_t spacing = 10000;
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
using Record = Observations::BarometerObservation;
using Sample = Observations::BarometerSample;
constexpr std::size_t capacity = 128;
#define SELECTED_RING m_barometer
#define SELECTED_TIME m_barometer_time
#define PUSH_SOURCE push_barometer
#define DELAY_FIELD barometer_delay_us
constexpr std::uint64_t arrival_age = 100000;
constexpr std::uint64_t spacing = 5000;
#endif
using Ring = formal_eskf::runtime::RingBuffer<Record, capacity>;
extern bool nondet_flag();
extern std::uint32_t nondet_half_period();
extern std::uint64_t nondet_time();
extern std::uint64_t nondet_u64();
extern std::uint32_t nondet_u32();
extern std::uint8_t nondet_u8();
extern bool nondet_bool();
extern imu_proof::Scalar nondet_scalar();
extern double nondet_double();
namespace
{
Sample expected_sample;
Timeline published_timeline;
std::uint64_t expected_now, effective;
std::uint32_t expected_half;
double expected_half_input;
unsigned half_calls;
Admission decision;
bool source_changed;
unsigned stage, resets, pushes;
void capture_storage_result()
{
#if FORMAL_ESKF_PROOF_FRAME == FORMAL_ESKF_PROOF_OBSERVATION_SOURCE
    capture_storage();
#endif
}

} // namespace
// The separately checked getter is a pure function of the unchanged period.
// Quantify over every uint32 result here; its exact binary64 equation and range
// are supplied by verify_period_range and the complete view/frame profiles.
template <> std::uint32_t Observations::half_period_us() const
{
    __ESBMC_assert(this == expected_owner && same_double(m_average_period_us, expected_half_input),
                   "Enqueue forwards the unchanged receiver/period to the offset producer");
    ++half_calls;
    return expected_half;
}

// Independently verified admission may reject or queue, with or without a
// source transition. The caller must handle every combination correctly.
template <>
Observations::PushResult Observations::admit(Timeline & timeline, SourceStamp const & stamp, std::uint64_t now,
                                             Limits const & limits, std::uint64_t & time)
{
    check_metadata();
    check_storage();
    __ESBMC_assert(this == expected_owner && &timeline == &SELECTED_TIME && stage == 0,
                   "Push calls admission once on its source timeline");
    __ESBMC_assert(stamp.time_us == expected_sample.stamp.time_us &&
                       stamp.source.device_id == expected_sample.stamp.source.device_id &&
                       stamp.source.calibration_count == expected_sample.stamp.source.calibration_count &&
                       now == expected_now && limits.delay_us == m_configuration.DELAY_FIELD &&
                       limits.arrival_age_us == arrival_age && limits.spacing_us == spacing && time == 0,
                   "Push forwards complete stamp, arrival, delay, age and spacing");
    auto const code = imu_proof::nondet_index();
    __ESBMC_assume(code <= static_cast<unsigned char>(Admission::too_fast));
    decision = static_cast<Admission>(code);
    source_changed = nondet_flag();
    effective = nondet_time();
    time = effective;
    __ESBMC_init_object(&published_timeline);
    timeline = published_timeline;
    stage = 1;
    capture_metadata();
    return {decision, source_changed};
}
template <> void formal_eskf::runtime::RingBuffer<Record, capacity>::reset()
{
    check_metadata();
    check_storage();
    __ESBMC_assert(this == &expected_owner->SELECTED_RING && source_changed && stage == 1 && resets == 0,
                   "Source transition clears the correct ring exactly once before enqueue");
    __ESBMC_init_object(expected_owner);
    expected_ring_metadata[FORMAL_ESKF_PROOF_OBSERVATION_SOURCE][1] = 0;
    expected_ring_metadata[FORMAL_ESKF_PROOF_OBSERVATION_SOURCE][2] = 0;
    expected_ring_metadata[FORMAL_ESKF_PROOF_OBSERVATION_SOURCE][3] = 0;
    assume_metadata();
#if FORMAL_ESKF_PROOF_FRAME != FORMAL_ESKF_PROOF_OBSERVATION_SOURCE
    assume_storage();
#endif
    ++resets;
    stage = 2;
    capture_metadata();
    capture_storage_result();
}
template <> void formal_eskf::runtime::RingBuffer<Record, capacity>::push(Record const & record)
{
    check_metadata();
    check_storage();
    __ESBMC_assert(this == &expected_owner->SELECTED_RING && decision == Admission::queued &&
                       stage == (source_changed ? 2U : 1U) && pushes == 0,
                   "Push calls typed storage once after any source reset");
    __ESBMC_assert(m_length == capacity && m_count <= capacity && m_oldest < capacity && m_write < capacity &&
                       m_write == (m_oldest + m_count) % capacity,
                   "Push satisfies typed storage metadata");
    for (std::size_t i = 0; i < capacity; ++i)
    {
        __ESBMC_assert(&record != &m_samples[i], "Push supplies an external record");
    }
    __ESBMC_assert(record.time_us == effective && record.arrival_time_us == expected_now &&
                       record.residual_delay_us == expected_owner->m_configuration.DELAY_FIELD &&
                       record.half_period_us == expected_half && record.generation == published_timeline.generation,
                   "Push constructs all observation metadata from admitted values");
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
    __ESBMC_assert(record.sample.stamp.time_us == expected_sample.stamp.time_us,
                   "Push complete input forwarding sample.stamp.time_us");
    __ESBMC_assert(imu_proof::same(record.sample.velocity_n.m_storage.values.head,
                                   expected_sample.velocity_n.m_storage.values.head),
                   "Push complete input forwarding sample.velocity_n.m_storage.values.head");
    __ESBMC_assert(imu_proof::same(record.sample.velocity_n.m_storage.values.tail.head,
                                   expected_sample.velocity_n.m_storage.values.tail.head),
                   "Push complete input forwarding sample.velocity_n.m_storage.values.tail.head");
    __ESBMC_assert(imu_proof::same(record.sample.velocity_n.m_storage.values.tail.tail.head,
                                   expected_sample.velocity_n.m_storage.values.tail.tail.head),
                   "Push complete input forwarding sample.velocity_n.m_storage.values.tail.tail.head");
    __ESBMC_assert(imu_proof::same(record.sample.horizontal_accuracy_m, expected_sample.horizontal_accuracy_m),
                   "Push complete input forwarding sample.horizontal_accuracy_m");
    __ESBMC_assert(imu_proof::same(record.sample.vertical_accuracy_m, expected_sample.vertical_accuracy_m),
                   "Push complete input forwarding sample.vertical_accuracy_m");
    __ESBMC_assert(imu_proof::same(record.sample.speed_accuracy_m_s, expected_sample.speed_accuracy_m_s),
                   "Push complete input forwarding sample.speed_accuracy_m_s");
    __ESBMC_assert(imu_proof::same(record.sample.hdop, expected_sample.hdop),
                   "Push complete input forwarding sample.hdop");
    __ESBMC_assert(imu_proof::same(record.sample.vdop, expected_sample.vdop),
                   "Push complete input forwarding sample.vdop");
    __ESBMC_assert(same_double(record.sample.latitude_deg, expected_sample.latitude_deg),
                   "Push complete input forwarding sample.latitude_deg");
    __ESBMC_assert(same_double(record.sample.longitude_deg, expected_sample.longitude_deg),
                   "Push complete input forwarding sample.longitude_deg");
    __ESBMC_assert(same_double(record.sample.altitude_msl_m, expected_sample.altitude_msl_m),
                   "Push complete input forwarding sample.altitude_msl_m");
    __ESBMC_assert(same_double(record.sample.altitude_ellipsoid_m, expected_sample.altitude_ellipsoid_m),
                   "Push complete input forwarding sample.altitude_ellipsoid_m");
    __ESBMC_assert(record.sample.stamp.source.device_id == expected_sample.stamp.source.device_id,
                   "Push complete input forwarding sample.stamp.source.device_id");
    __ESBMC_assert(record.sample.stamp.source.calibration_count == expected_sample.stamp.source.calibration_count,
                   "Push complete input forwarding sample.stamp.source.calibration_count");
    __ESBMC_assert(record.sample.fix_type == expected_sample.fix_type,
                   "Push complete input forwarding sample.fix_type");
    __ESBMC_assert(record.sample.satellites == expected_sample.satellites,
                   "Push complete input forwarding sample.satellites");
    __ESBMC_assert(record.sample.velocity_valid == expected_sample.velocity_valid,
                   "Push complete input forwarding sample.velocity_valid");
    __ESBMC_assert(record.sample.spoofed == expected_sample.spoofed, "Push complete input forwarding sample.spoofed");
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
    __ESBMC_assert(record.sample.stamp.time_us == expected_sample.stamp.time_us,
                   "Push complete input forwarding sample.stamp.time_us");
    __ESBMC_assert(
        imu_proof::same(record.sample.field_b.m_storage.values.head, expected_sample.field_b.m_storage.values.head),
        "Push complete input forwarding sample.field_b.m_storage.values.head");
    __ESBMC_assert(imu_proof::same(record.sample.field_b.m_storage.values.tail.head,
                                   expected_sample.field_b.m_storage.values.tail.head),
                   "Push complete input forwarding sample.field_b.m_storage.values.tail.head");
    __ESBMC_assert(imu_proof::same(record.sample.field_b.m_storage.values.tail.tail.head,
                                   expected_sample.field_b.m_storage.values.tail.tail.head),
                   "Push complete input forwarding sample.field_b.m_storage.values.tail.tail.head");
    __ESBMC_assert(record.sample.stamp.source.device_id == expected_sample.stamp.source.device_id,
                   "Push complete input forwarding sample.stamp.source.device_id");
    __ESBMC_assert(record.sample.stamp.source.calibration_count == expected_sample.stamp.source.calibration_count,
                   "Push complete input forwarding sample.stamp.source.calibration_count");
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
    __ESBMC_assert(record.sample.stamp.time_us == expected_sample.stamp.time_us,
                   "Push complete input forwarding sample.stamp.time_us");
    __ESBMC_assert(imu_proof::same(record.sample.altitude_m, expected_sample.altitude_m),
                   "Push complete input forwarding sample.altitude_m");
    __ESBMC_assert(record.sample.stamp.source.device_id == expected_sample.stamp.source.device_id,
                   "Push complete input forwarding sample.stamp.source.device_id");
    __ESBMC_assert(record.sample.stamp.source.calibration_count == expected_sample.stamp.source.calibration_count,
                   "Push complete input forwarding sample.stamp.source.calibration_count");
#endif
    __ESBMC_init_object(expected_owner);
    expected_ring_metadata[FORMAL_ESKF_PROOF_OBSERVATION_SOURCE][1] = expected_owner->SELECTED_RING.m_count;
    expected_ring_metadata[FORMAL_ESKF_PROOF_OBSERVATION_SOURCE][2] = expected_owner->SELECTED_RING.m_oldest;
    expected_ring_metadata[FORMAL_ESKF_PROOF_OBSERVATION_SOURCE][3] = expected_owner->SELECTED_RING.m_write;
    assume_metadata();
#if FORMAL_ESKF_PROOF_FRAME != FORMAL_ESKF_PROOF_OBSERVATION_SOURCE
    assume_storage();
#endif
    ++pushes;
    ++stage;
    capture_metadata();
    capture_storage_result();
}

void verify_observation_enqueue(Observations owner, Sample sample, std::uint64_t now)
{
    auto const count = imu_proof::nondet_index();
    auto const oldest = imu_proof::nondet_index();
    __ESBMC_assume(count <= capacity && oldest < capacity);
    owner.SELECTED_RING.m_length = capacity;
    owner.SELECTED_RING.m_count = count;
    owner.SELECTED_RING.m_oldest = oldest;
    owner.SELECTED_RING.m_write = (static_cast<std::size_t>(oldest) + count) % capacity;
    __ESBMC_assume(owner.m_average_period_us >= 3800. && owner.m_average_period_us <= 4200.);
    auto const write = owner.SELECTED_RING.m_write;
    auto const gnss_time = owner.m_gnss_time;
    auto const magnetic_time = owner.m_magnetic_time;
    auto const barometer_time = owner.m_barometer_time;
    auto const configuration = owner.m_configuration;
    auto const configured = owner.m_configured;
    auto const seed = owner.m_seed_time_us;
    auto const latest = owner.m_latest_time_us;
    auto const fusion = owner.m_fusion_time_us;
    auto const advanced = owner.m_advanced_at_us;
    auto const average = owner.m_average_period_us;
    expected_owner = &owner;
    expected_sample = sample;
    expected_now = now;
    // The actual getter equation/range and readonly frame are separate producer obligations.
    expected_half = nondet_half_period();
    expected_half_input = owner.m_average_period_us;
    half_calls = 0;
    stage = 0;
    resets = 0;
    pushes = 0;
    capture_storage();
    capture_metadata();
    auto const result = owner.PUSH_SOURCE(sample, now);
    bool const queued = decision == Admission::queued;
    __ESBMC_assert(half_calls == static_cast<unsigned>(queued),
                   "Enqueue calls the period-offset producer exactly when constructing an event");
    __ESBMC_assert(stage == 1U + static_cast<unsigned>(source_changed) + static_cast<unsigned>(queued) &&
                       resets == static_cast<unsigned>(source_changed) && pushes == static_cast<unsigned>(queued),
                   "Push complete admission/reset/enqueue call sequence");
    __ESBMC_assert(result.decision == decision && result.source_changed == source_changed &&
                       result.source_discarded == (source_changed ? count : 0) &&
                       result.overwritten == (queued && !source_changed && count == capacity),
                   "Push exact reported decision, source discard and overwrite");
    __ESBMC_assert(owner.SELECTED_RING.m_length == expected_ring_metadata[FORMAL_ESKF_PROOF_OBSERVATION_SOURCE][0] &&
                       owner.SELECTED_RING.m_count == expected_ring_metadata[FORMAL_ESKF_PROOF_OBSERVATION_SOURCE][1] &&
                       owner.SELECTED_RING.m_oldest ==
                           expected_ring_metadata[FORMAL_ESKF_PROOF_OBSERVATION_SOURCE][2] &&
                       owner.SELECTED_RING.m_write == expected_ring_metadata[FORMAL_ESKF_PROOF_OBSERVATION_SOURCE][3],
                   "Push preserves complete typed producer metadata");
    __ESBMC_assert(owner.m_configured == configured &&
                       imu_proof::same_configuration(owner.m_configuration, configuration) &&
                       owner.m_seed_time_us == seed && owner.m_latest_time_us == latest &&
                       owner.m_fusion_time_us == fusion && owner.m_advanced_at_us == advanced &&
                       same_double(owner.m_average_period_us, average) && now == expected_now,
                   "Push complete configuration, clock and arrival frame");
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
    __ESBMC_assert(same_timeline(owner.m_gnss_time, published_timeline), "Push preserves admission timeline poststate");
#else
    __ESBMC_assert(same_timeline(owner.m_gnss_time, gnss_time), "Push retains other source timelines");
#endif
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
    __ESBMC_assert(same_timeline(owner.m_magnetic_time, published_timeline),
                   "Push preserves admission timeline poststate");
#else
    __ESBMC_assert(same_timeline(owner.m_magnetic_time, magnetic_time), "Push retains other source timelines");
#endif
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
    __ESBMC_assert(same_timeline(owner.m_barometer_time, published_timeline),
                   "Push preserves admission timeline poststate");
#else
    __ESBMC_assert(same_timeline(owner.m_barometer_time, barometer_time), "Push retains other source timelines");
#endif
    check_metadata();
    check_storage();
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
    __ESBMC_assert(sample.stamp.time_us == expected_sample.stamp.time_us,
                   "Push external input frame sample.stamp.time_us");
    __ESBMC_assert(
        imu_proof::same(sample.velocity_n.m_storage.values.head, expected_sample.velocity_n.m_storage.values.head),
        "Push external input frame sample.velocity_n.m_storage.values.head");
    __ESBMC_assert(imu_proof::same(sample.velocity_n.m_storage.values.tail.head,
                                   expected_sample.velocity_n.m_storage.values.tail.head),
                   "Push external input frame sample.velocity_n.m_storage.values.tail.head");
    __ESBMC_assert(imu_proof::same(sample.velocity_n.m_storage.values.tail.tail.head,
                                   expected_sample.velocity_n.m_storage.values.tail.tail.head),
                   "Push external input frame sample.velocity_n.m_storage.values.tail.tail.head");
    __ESBMC_assert(imu_proof::same(sample.horizontal_accuracy_m, expected_sample.horizontal_accuracy_m),
                   "Push external input frame sample.horizontal_accuracy_m");
    __ESBMC_assert(imu_proof::same(sample.vertical_accuracy_m, expected_sample.vertical_accuracy_m),
                   "Push external input frame sample.vertical_accuracy_m");
    __ESBMC_assert(imu_proof::same(sample.speed_accuracy_m_s, expected_sample.speed_accuracy_m_s),
                   "Push external input frame sample.speed_accuracy_m_s");
    __ESBMC_assert(imu_proof::same(sample.hdop, expected_sample.hdop), "Push external input frame sample.hdop");
    __ESBMC_assert(imu_proof::same(sample.vdop, expected_sample.vdop), "Push external input frame sample.vdop");
    __ESBMC_assert(same_double(sample.latitude_deg, expected_sample.latitude_deg),
                   "Push external input frame sample.latitude_deg");
    __ESBMC_assert(same_double(sample.longitude_deg, expected_sample.longitude_deg),
                   "Push external input frame sample.longitude_deg");
    __ESBMC_assert(same_double(sample.altitude_msl_m, expected_sample.altitude_msl_m),
                   "Push external input frame sample.altitude_msl_m");
    __ESBMC_assert(same_double(sample.altitude_ellipsoid_m, expected_sample.altitude_ellipsoid_m),
                   "Push external input frame sample.altitude_ellipsoid_m");
    __ESBMC_assert(sample.stamp.source.device_id == expected_sample.stamp.source.device_id,
                   "Push external input frame sample.stamp.source.device_id");
    __ESBMC_assert(sample.stamp.source.calibration_count == expected_sample.stamp.source.calibration_count,
                   "Push external input frame sample.stamp.source.calibration_count");
    __ESBMC_assert(sample.fix_type == expected_sample.fix_type, "Push external input frame sample.fix_type");
    __ESBMC_assert(sample.satellites == expected_sample.satellites, "Push external input frame sample.satellites");
    __ESBMC_assert(sample.velocity_valid == expected_sample.velocity_valid,
                   "Push external input frame sample.velocity_valid");
    __ESBMC_assert(sample.spoofed == expected_sample.spoofed, "Push external input frame sample.spoofed");
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
    __ESBMC_assert(sample.stamp.time_us == expected_sample.stamp.time_us,
                   "Push external input frame sample.stamp.time_us");
    __ESBMC_assert(imu_proof::same(sample.field_b.m_storage.values.head, expected_sample.field_b.m_storage.values.head),
                   "Push external input frame sample.field_b.m_storage.values.head");
    __ESBMC_assert(
        imu_proof::same(sample.field_b.m_storage.values.tail.head, expected_sample.field_b.m_storage.values.tail.head),
        "Push external input frame sample.field_b.m_storage.values.tail.head");
    __ESBMC_assert(imu_proof::same(sample.field_b.m_storage.values.tail.tail.head,
                                   expected_sample.field_b.m_storage.values.tail.tail.head),
                   "Push external input frame sample.field_b.m_storage.values.tail.tail.head");
    __ESBMC_assert(sample.stamp.source.device_id == expected_sample.stamp.source.device_id,
                   "Push external input frame sample.stamp.source.device_id");
    __ESBMC_assert(sample.stamp.source.calibration_count == expected_sample.stamp.source.calibration_count,
                   "Push external input frame sample.stamp.source.calibration_count");
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
    __ESBMC_assert(sample.stamp.time_us == expected_sample.stamp.time_us,
                   "Push external input frame sample.stamp.time_us");
    __ESBMC_assert(imu_proof::same(sample.altitude_m, expected_sample.altitude_m),
                   "Push external input frame sample.altitude_m");
    __ESBMC_assert(sample.stamp.source.device_id == expected_sample.stamp.source.device_id,
                   "Push external input frame sample.stamp.source.device_id");
    __ESBMC_assert(sample.stamp.source.calibration_count == expected_sample.stamp.source.calibration_count,
                   "Push external input frame sample.stamp.source.calibration_count");
#endif
}
