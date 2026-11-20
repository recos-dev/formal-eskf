// SPDX-License-Identifier: BSD-3-Clause
#include "observation_frame_support.hpp"

static_assert(FORMAL_ESKF_PROOF_FRAME >= 0 && FORMAL_ESKF_PROOF_FRAME < 4);
static_assert(FORMAL_ESKF_PROOF_OBSERVATION_SOURCE >= 0 && FORMAL_ESKF_PROOF_OBSERVATION_SOURCE < 3);

using observation_proof::Observations;
using observation_proof::same_double;
using observation_proof::same_timeline;
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
using Record = Observations::GnssObservation;
constexpr std::size_t capacity = 16;
#define SELECTED_RING m_gnss
#define SELECTED_TIME m_gnss_time
#define POP_SOURCE pop_gnss
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
using Record = Observations::MagneticObservation;
constexpr std::size_t capacity = 64;
#define SELECTED_RING m_magnetic
#define SELECTED_TIME m_magnetic_time
#define POP_SOURCE pop_magnetic
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
using Record = Observations::BarometerObservation;
constexpr std::size_t capacity = 128;
#define SELECTED_RING m_barometer
#define SELECTED_TIME m_barometer_time
#define POP_SOURCE pop_barometer
#endif
using Ring = formal_eskf::runtime::RingBuffer<Record, capacity>;
extern bool nondet_selected();
namespace
{
Record * expected_output;
std::array<Record, capacity> published_storage;
Record published_output;
std::size_t removed;
bool published;
unsigned calls;
void capture_storage_result()
{
#if FORMAL_ESKF_PROOF_FRAME == FORMAL_ESKF_PROOF_OBSERVATION_SOURCE
    capture_storage();
#endif
}

} // namespace
// Broad caller boundary: typed ring search/retirement plus Lean composition
// discharge the exact payload semantics. This summary permits every payload
// poststate, while retaining the proved count/index and failure-output frames.
template <>
bool formal_eskf::runtime::RingBuffer<Record, capacity>::pop_first_older_than(std::uint64_t const & horizon,
                                                                              Record * output, std::uint64_t age)
{
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
    expected_gnss_time.selection_time_us = expected_fusion;
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
    expected_magnetic_time.selection_time_us = expected_fusion;
#else
    expected_barometer_time.selection_time_us = expected_fusion;
#endif
    check_metadata();
    check_storage();
    __ESBMC_assert(this == &expected_owner->SELECTED_RING && output == expected_output &&
                       horizon == expected_owner->m_fusion_time_us && age == 100000 && calls == 0,
                   "Pop caller forwards exact receiver, horizon, age and output");
    __ESBMC_assert(m_length == capacity && m_count <= capacity && m_oldest < capacity && m_write < capacity &&
                       m_write == (m_oldest + m_count) % capacity,
                   "Pop caller meets allocated ring metadata contract");
    for (std::size_t i = 0; i < capacity; ++i)
    {
        __ESBMC_assert(output != &m_samples[i], "Pop supplies external output");
        __ESBMC_assert(&horizon != &m_samples[i].time_us, "Pop supplies external horizon");
    }
    removed = imu_proof::nondet_index();
    published = nondet_selected();
    __ESBMC_assume(removed <= m_count && (!published || removed > 0));
    __ESBMC_init_object(&published_storage);
    // Havoc complete storage, then copy each value field without a bulk memcpy.
    // The contract covers values, signed zero and NaN class, not padding bytes.
    for (std::size_t i = 0; i < capacity; ++i)
    {
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
        m_samples[i].time_us = published_storage[i].time_us;
        m_samples[i].arrival_time_us = published_storage[i].arrival_time_us;
        m_samples[i].residual_delay_us = published_storage[i].residual_delay_us;
        m_samples[i].generation = published_storage[i].generation;
        m_samples[i].sample.stamp.time_us = published_storage[i].sample.stamp.time_us;
        m_samples[i].sample.velocity_n.m_storage.values.head =
            published_storage[i].sample.velocity_n.m_storage.values.head;
        m_samples[i].sample.velocity_n.m_storage.values.tail.head =
            published_storage[i].sample.velocity_n.m_storage.values.tail.head;
        m_samples[i].sample.velocity_n.m_storage.values.tail.tail.head =
            published_storage[i].sample.velocity_n.m_storage.values.tail.tail.head;
        m_samples[i].sample.horizontal_accuracy_m = published_storage[i].sample.horizontal_accuracy_m;
        m_samples[i].sample.vertical_accuracy_m = published_storage[i].sample.vertical_accuracy_m;
        m_samples[i].sample.speed_accuracy_m_s = published_storage[i].sample.speed_accuracy_m_s;
        m_samples[i].sample.hdop = published_storage[i].sample.hdop;
        m_samples[i].sample.vdop = published_storage[i].sample.vdop;
        m_samples[i].sample.latitude_deg = published_storage[i].sample.latitude_deg;
        m_samples[i].sample.longitude_deg = published_storage[i].sample.longitude_deg;
        m_samples[i].sample.altitude_msl_m = published_storage[i].sample.altitude_msl_m;
        m_samples[i].sample.altitude_ellipsoid_m = published_storage[i].sample.altitude_ellipsoid_m;
        m_samples[i].half_period_us = published_storage[i].half_period_us;
        m_samples[i].sample.stamp.source.device_id = published_storage[i].sample.stamp.source.device_id;
        m_samples[i].sample.stamp.source.calibration_count = published_storage[i].sample.stamp.source.calibration_count;
        m_samples[i].sample.fix_type = published_storage[i].sample.fix_type;
        m_samples[i].sample.satellites = published_storage[i].sample.satellites;
        m_samples[i].sample.velocity_valid = published_storage[i].sample.velocity_valid;
        m_samples[i].sample.spoofed = published_storage[i].sample.spoofed;
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
        m_samples[i].time_us = published_storage[i].time_us;
        m_samples[i].arrival_time_us = published_storage[i].arrival_time_us;
        m_samples[i].residual_delay_us = published_storage[i].residual_delay_us;
        m_samples[i].generation = published_storage[i].generation;
        m_samples[i].sample.stamp.time_us = published_storage[i].sample.stamp.time_us;
        m_samples[i].sample.field_b.m_storage.values.head = published_storage[i].sample.field_b.m_storage.values.head;
        m_samples[i].sample.field_b.m_storage.values.tail.head =
            published_storage[i].sample.field_b.m_storage.values.tail.head;
        m_samples[i].sample.field_b.m_storage.values.tail.tail.head =
            published_storage[i].sample.field_b.m_storage.values.tail.tail.head;
        m_samples[i].half_period_us = published_storage[i].half_period_us;
        m_samples[i].sample.stamp.source.device_id = published_storage[i].sample.stamp.source.device_id;
        m_samples[i].sample.stamp.source.calibration_count = published_storage[i].sample.stamp.source.calibration_count;
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
        m_samples[i].time_us = published_storage[i].time_us;
        m_samples[i].arrival_time_us = published_storage[i].arrival_time_us;
        m_samples[i].residual_delay_us = published_storage[i].residual_delay_us;
        m_samples[i].generation = published_storage[i].generation;
        m_samples[i].sample.stamp.time_us = published_storage[i].sample.stamp.time_us;
        m_samples[i].sample.altitude_m = published_storage[i].sample.altitude_m;
        m_samples[i].half_period_us = published_storage[i].half_period_us;
        m_samples[i].sample.stamp.source.device_id = published_storage[i].sample.stamp.source.device_id;
        m_samples[i].sample.stamp.source.calibration_count = published_storage[i].sample.stamp.source.calibration_count;
#endif
    }
    m_count -= removed;
    m_oldest = (m_oldest + removed) % capacity;
    if (published)
    {
        __ESBMC_init_object(&published_output);
        *output = published_output;
    }
    ++calls;
    capture_metadata();
    capture_storage_result();
    return published;
}

void verify_observation_pop(Observations owner, Record output)
{
    auto const count = imu_proof::nondet_index();
    auto const oldest = imu_proof::nondet_index();
    __ESBMC_assume(count <= capacity && oldest < capacity);
    owner.SELECTED_RING.m_length = capacity;
    owner.SELECTED_RING.m_count = count;
    owner.SELECTED_RING.m_oldest = oldest;
    owner.SELECTED_RING.m_write = (static_cast<std::size_t>(oldest) + count) % capacity;
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
    auto const selection = owner.SELECTED_TIME.selection_time_us;
    bool const eligible = advanced != 0 && selection != fusion;
    expected_owner = &owner;
    expected_output = &output;
    calls = 0;
    capture_storage();
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
    std::uint64_t output_integers[5];
    output_integers[0] = output.time_us;
    output_integers[1] = output.arrival_time_us;
    output_integers[2] = output.residual_delay_us;
    output_integers[3] = output.generation;
    output_integers[4] = output.sample.stamp.time_us;
    imu_proof::Scalar output_values[8];
    output_values[0] = output.sample.velocity_n.m_storage.values.head;
    output_values[1] = output.sample.velocity_n.m_storage.values.tail.head;
    output_values[2] = output.sample.velocity_n.m_storage.values.tail.tail.head;
    output_values[3] = output.sample.horizontal_accuracy_m;
    output_values[4] = output.sample.vertical_accuracy_m;
    output_values[5] = output.sample.speed_accuracy_m_s;
    output_values[6] = output.sample.hdop;
    output_values[7] = output.sample.vdop;
    double output_doubles[4];
    output_doubles[0] = output.sample.latitude_deg;
    output_doubles[1] = output.sample.longitude_deg;
    output_doubles[2] = output.sample.altitude_msl_m;
    output_doubles[3] = output.sample.altitude_ellipsoid_m;
    std::uint32_t output_small[2];
    output_small[0] = output.half_period_us;
    output_small[1] = output.sample.stamp.source.device_id;
    std::uint8_t output_tags[3];
    output_tags[0] = output.sample.stamp.source.calibration_count;
    output_tags[1] = output.sample.fix_type;
    output_tags[2] = output.sample.satellites;
    bool output_flags[2];
    output_flags[0] = output.sample.velocity_valid;
    output_flags[1] = output.sample.spoofed;
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
    std::uint64_t output_integers[5];
    output_integers[0] = output.time_us;
    output_integers[1] = output.arrival_time_us;
    output_integers[2] = output.residual_delay_us;
    output_integers[3] = output.generation;
    output_integers[4] = output.sample.stamp.time_us;
    imu_proof::Scalar output_values[3];
    output_values[0] = output.sample.field_b.m_storage.values.head;
    output_values[1] = output.sample.field_b.m_storage.values.tail.head;
    output_values[2] = output.sample.field_b.m_storage.values.tail.tail.head;
    std::uint32_t output_small[2];
    output_small[0] = output.half_period_us;
    output_small[1] = output.sample.stamp.source.device_id;
    std::uint8_t output_tags[1];
    output_tags[0] = output.sample.stamp.source.calibration_count;
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
    std::uint64_t output_integers[5];
    output_integers[0] = output.time_us;
    output_integers[1] = output.arrival_time_us;
    output_integers[2] = output.residual_delay_us;
    output_integers[3] = output.generation;
    output_integers[4] = output.sample.stamp.time_us;
    imu_proof::Scalar output_values[1];
    output_values[0] = output.sample.altitude_m;
    std::uint32_t output_small[2];
    output_small[0] = output.half_period_us;
    output_small[1] = output.sample.stamp.source.device_id;
    std::uint8_t output_tags[1];
    output_tags[0] = output.sample.stamp.source.calibration_count;
#endif
    capture_metadata();
    auto const result = owner.POP_SOURCE(output);
    __ESBMC_assert(calls == (eligible ? 1U : 0U), "Pop exact producer eligibility and multiplicity");
    auto const decision = advanced == 0         ? Observations::Selection::not_ready
                          : selection == fusion ? Observations::Selection::repeated
                          : published           ? Observations::Selection::selected
                                                : Observations::Selection::empty;
    __ESBMC_assert(result.decision == decision &&
                       result.retired == (eligible ? removed - static_cast<std::size_t>(published) : 0),
                   "Pop exact decision and retired count without underflow");
    __ESBMC_assert(owner.SELECTED_RING.m_length == capacity && owner.SELECTED_RING.m_write == write &&
                       owner.SELECTED_RING.m_count == (eligible ? count - removed : count) &&
                       owner.SELECTED_RING.m_oldest == (eligible ? (oldest + removed) % capacity : oldest),
                   "Pop preserves producer ring metadata or entire rejection metadata");
    __ESBMC_assert(
        owner.m_configured == configured && imu_proof::same_configuration(owner.m_configuration, configuration) &&
            owner.m_seed_time_us == seed && owner.m_latest_time_us == latest && owner.m_fusion_time_us == fusion &&
            owner.m_advanced_at_us == advanced && same_double(owner.m_average_period_us, average),
        "Pop configuration and clock frame");
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
    auto expected_time = gnss_time;
    if (eligible)
    {
        expected_time.selection_time_us = fusion;
    }
    __ESBMC_assert(same_timeline(owner.m_gnss_time, expected_time), "Pop only advances its selection watermark");
#else
    __ESBMC_assert(same_timeline(owner.m_gnss_time, gnss_time), "Pop retains other source timelines");
#endif
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
    auto expected_time = magnetic_time;
    if (eligible)
    {
        expected_time.selection_time_us = fusion;
    }
    __ESBMC_assert(same_timeline(owner.m_magnetic_time, expected_time), "Pop only advances its selection watermark");
#else
    __ESBMC_assert(same_timeline(owner.m_magnetic_time, magnetic_time), "Pop retains other source timelines");
#endif
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
    auto expected_time = barometer_time;
    if (eligible)
    {
        expected_time.selection_time_us = fusion;
    }
    __ESBMC_assert(same_timeline(owner.m_barometer_time, expected_time), "Pop only advances its selection watermark");
#else
    __ESBMC_assert(same_timeline(owner.m_barometer_time, barometer_time), "Pop retains other source timelines");
#endif
    check_metadata();
    check_storage();
#if FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 0
    __ESBMC_assert(output.time_us == (eligible && published ? published_output.time_us : output_integers[0]),
                   "Pop output publication or failure frame time_us");
    __ESBMC_assert(output.arrival_time_us ==
                       (eligible && published ? published_output.arrival_time_us : output_integers[1]),
                   "Pop output publication or failure frame arrival_time_us");
    __ESBMC_assert(output.residual_delay_us ==
                       (eligible && published ? published_output.residual_delay_us : output_integers[2]),
                   "Pop output publication or failure frame residual_delay_us");
    __ESBMC_assert(output.generation == (eligible && published ? published_output.generation : output_integers[3]),
                   "Pop output publication or failure frame generation");
    __ESBMC_assert(output.sample.stamp.time_us ==
                       (eligible && published ? published_output.sample.stamp.time_us : output_integers[4]),
                   "Pop output publication or failure frame sample.stamp.time_us");
    __ESBMC_assert(imu_proof::same(output.sample.velocity_n.m_storage.values.head,
                                   (eligible && published ? published_output.sample.velocity_n.m_storage.values.head
                                                          : output_values[0])),
                   "Pop output publication or failure frame sample.velocity_n.m_storage.values.head");
    __ESBMC_assert(
        imu_proof::same(
            output.sample.velocity_n.m_storage.values.tail.head,
            (eligible && published ? published_output.sample.velocity_n.m_storage.values.tail.head : output_values[1])),
        "Pop output publication or failure frame sample.velocity_n.m_storage.values.tail.head");
    __ESBMC_assert(
        imu_proof::same(output.sample.velocity_n.m_storage.values.tail.tail.head,
                        (eligible && published ? published_output.sample.velocity_n.m_storage.values.tail.tail.head
                                               : output_values[2])),
        "Pop output publication or failure frame sample.velocity_n.m_storage.values.tail.tail.head");
    __ESBMC_assert(
        imu_proof::same(output.sample.horizontal_accuracy_m,
                        (eligible && published ? published_output.sample.horizontal_accuracy_m : output_values[3])),
        "Pop output publication or failure frame sample.horizontal_accuracy_m");
    __ESBMC_assert(
        imu_proof::same(output.sample.vertical_accuracy_m,
                        (eligible && published ? published_output.sample.vertical_accuracy_m : output_values[4])),
        "Pop output publication or failure frame sample.vertical_accuracy_m");
    __ESBMC_assert(
        imu_proof::same(output.sample.speed_accuracy_m_s,
                        (eligible && published ? published_output.sample.speed_accuracy_m_s : output_values[5])),
        "Pop output publication or failure frame sample.speed_accuracy_m_s");
    __ESBMC_assert(
        imu_proof::same(output.sample.hdop, (eligible && published ? published_output.sample.hdop : output_values[6])),
        "Pop output publication or failure frame sample.hdop");
    __ESBMC_assert(
        imu_proof::same(output.sample.vdop, (eligible && published ? published_output.sample.vdop : output_values[7])),
        "Pop output publication or failure frame sample.vdop");
    __ESBMC_assert(same_double(output.sample.latitude_deg,
                               (eligible && published ? published_output.sample.latitude_deg : output_doubles[0])),
                   "Pop output publication or failure frame sample.latitude_deg");
    __ESBMC_assert(same_double(output.sample.longitude_deg,
                               (eligible && published ? published_output.sample.longitude_deg : output_doubles[1])),
                   "Pop output publication or failure frame sample.longitude_deg");
    __ESBMC_assert(same_double(output.sample.altitude_msl_m,
                               (eligible && published ? published_output.sample.altitude_msl_m : output_doubles[2])),
                   "Pop output publication or failure frame sample.altitude_msl_m");
    __ESBMC_assert(
        same_double(output.sample.altitude_ellipsoid_m,
                    (eligible && published ? published_output.sample.altitude_ellipsoid_m : output_doubles[3])),
        "Pop output publication or failure frame sample.altitude_ellipsoid_m");
    __ESBMC_assert(output.half_period_us == (eligible && published ? published_output.half_period_us : output_small[0]),
                   "Pop output publication or failure frame half_period_us");
    __ESBMC_assert(output.sample.stamp.source.device_id ==
                       (eligible && published ? published_output.sample.stamp.source.device_id : output_small[1]),
                   "Pop output publication or failure frame sample.stamp.source.device_id");
    __ESBMC_assert(
        output.sample.stamp.source.calibration_count ==
            (eligible && published ? published_output.sample.stamp.source.calibration_count : output_tags[0]),
        "Pop output publication or failure frame sample.stamp.source.calibration_count");
    __ESBMC_assert(output.sample.fix_type ==
                       (eligible && published ? published_output.sample.fix_type : output_tags[1]),
                   "Pop output publication or failure frame sample.fix_type");
    __ESBMC_assert(output.sample.satellites ==
                       (eligible && published ? published_output.sample.satellites : output_tags[2]),
                   "Pop output publication or failure frame sample.satellites");
    __ESBMC_assert(output.sample.velocity_valid ==
                       (eligible && published ? published_output.sample.velocity_valid : output_flags[0]),
                   "Pop output publication or failure frame sample.velocity_valid");
    __ESBMC_assert(output.sample.spoofed == (eligible && published ? published_output.sample.spoofed : output_flags[1]),
                   "Pop output publication or failure frame sample.spoofed");
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 1
    __ESBMC_assert(output.time_us == (eligible && published ? published_output.time_us : output_integers[0]),
                   "Pop output publication or failure frame time_us");
    __ESBMC_assert(output.arrival_time_us ==
                       (eligible && published ? published_output.arrival_time_us : output_integers[1]),
                   "Pop output publication or failure frame arrival_time_us");
    __ESBMC_assert(output.residual_delay_us ==
                       (eligible && published ? published_output.residual_delay_us : output_integers[2]),
                   "Pop output publication or failure frame residual_delay_us");
    __ESBMC_assert(output.generation == (eligible && published ? published_output.generation : output_integers[3]),
                   "Pop output publication or failure frame generation");
    __ESBMC_assert(output.sample.stamp.time_us ==
                       (eligible && published ? published_output.sample.stamp.time_us : output_integers[4]),
                   "Pop output publication or failure frame sample.stamp.time_us");
    __ESBMC_assert(imu_proof::same(output.sample.field_b.m_storage.values.head,
                                   (eligible && published ? published_output.sample.field_b.m_storage.values.head
                                                          : output_values[0])),
                   "Pop output publication or failure frame sample.field_b.m_storage.values.head");
    __ESBMC_assert(imu_proof::same(output.sample.field_b.m_storage.values.tail.head,
                                   (eligible && published ? published_output.sample.field_b.m_storage.values.tail.head
                                                          : output_values[1])),
                   "Pop output publication or failure frame sample.field_b.m_storage.values.tail.head");
    __ESBMC_assert(
        imu_proof::same(output.sample.field_b.m_storage.values.tail.tail.head,
                        (eligible && published ? published_output.sample.field_b.m_storage.values.tail.tail.head
                                               : output_values[2])),
        "Pop output publication or failure frame sample.field_b.m_storage.values.tail.tail.head");
    __ESBMC_assert(output.half_period_us == (eligible && published ? published_output.half_period_us : output_small[0]),
                   "Pop output publication or failure frame half_period_us");
    __ESBMC_assert(output.sample.stamp.source.device_id ==
                       (eligible && published ? published_output.sample.stamp.source.device_id : output_small[1]),
                   "Pop output publication or failure frame sample.stamp.source.device_id");
    __ESBMC_assert(
        output.sample.stamp.source.calibration_count ==
            (eligible && published ? published_output.sample.stamp.source.calibration_count : output_tags[0]),
        "Pop output publication or failure frame sample.stamp.source.calibration_count");
#elif FORMAL_ESKF_PROOF_OBSERVATION_SOURCE == 2
    __ESBMC_assert(output.time_us == (eligible && published ? published_output.time_us : output_integers[0]),
                   "Pop output publication or failure frame time_us");
    __ESBMC_assert(output.arrival_time_us ==
                       (eligible && published ? published_output.arrival_time_us : output_integers[1]),
                   "Pop output publication or failure frame arrival_time_us");
    __ESBMC_assert(output.residual_delay_us ==
                       (eligible && published ? published_output.residual_delay_us : output_integers[2]),
                   "Pop output publication or failure frame residual_delay_us");
    __ESBMC_assert(output.generation == (eligible && published ? published_output.generation : output_integers[3]),
                   "Pop output publication or failure frame generation");
    __ESBMC_assert(output.sample.stamp.time_us ==
                       (eligible && published ? published_output.sample.stamp.time_us : output_integers[4]),
                   "Pop output publication or failure frame sample.stamp.time_us");
    __ESBMC_assert(imu_proof::same(output.sample.altitude_m,
                                   (eligible && published ? published_output.sample.altitude_m : output_values[0])),
                   "Pop output publication or failure frame sample.altitude_m");
    __ESBMC_assert(output.half_period_us == (eligible && published ? published_output.half_period_us : output_small[0]),
                   "Pop output publication or failure frame half_period_us");
    __ESBMC_assert(output.sample.stamp.source.device_id ==
                       (eligible && published ? published_output.sample.stamp.source.device_id : output_small[1]),
                   "Pop output publication or failure frame sample.stamp.source.device_id");
    __ESBMC_assert(
        output.sample.stamp.source.calibration_count ==
            (eligible && published ? published_output.sample.stamp.source.calibration_count : output_tags[0]),
        "Pop output publication or failure frame sample.stamp.source.calibration_count");
#endif
}
