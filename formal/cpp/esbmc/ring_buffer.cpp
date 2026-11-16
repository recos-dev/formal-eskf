// SPDX-License-Identifier: BSD-3-Clause

#include <array>
#include <cstddef>
#include <cstdint>

// Expose state for induction only. Production buffer bodies execute with
// ESBMC's standard-library array model; no buffer operation is summarized.
#define private public
#include <formal_eskf/runtime/ring_buffer.hpp>
#undef private

extern void __ESBMC_assume(bool condition);
extern void __ESBMC_assert(bool condition, char const * description);

#ifndef FORMAL_ESKF_PROOF_CAPACITY
#define FORMAL_ESKF_PROOF_CAPACITY 3
#endif
constexpr std::size_t capacity = FORMAL_ESKF_PROOF_CAPACITY;

struct Sample
{
    std::uint64_t time_us = 0;
    std::uint64_t payload = 0;

    Sample() = default;
    Sample(std::uint64_t time, std::uint64_t value) : time_us(time), payload(value) {}
    Sample(Sample const & other) : time_us(other.time_us), payload(other.payload) {}
    Sample & operator=(Sample const & other)
    {
        time_us = other.time_us;
        payload = other.payload;
        return *this;
    }
};
using Buffer = formal_eskf::runtime::RingBuffer<Sample, capacity>;

bool same_sample(Sample const & a, Sample const & b) { return a.time_us == b.time_us && a.payload == b.payload; }

struct Snapshot
{
    std::uint64_t times[capacity];
    std::uint64_t payloads[capacity];
    std::size_t length;
    std::size_t oldest;
    std::size_t write;
    std::size_t count;
};

void snapshot(Buffer const & buffer, Snapshot & result)
{
    // Fieldwise oracle snapshot avoids ESBMC 8.4 aggregate-copy conversion.
    // Production storage, initialization, indexing and fill remain unchanged.
    result.length = buffer.m_length;
    result.oldest = buffer.m_oldest;
    result.write = buffer.m_write;
    result.count = buffer.m_count;
    for (std::size_t index = 0; index < capacity; ++index)
    {
        result.times[index] = buffer.m_samples[index].time_us;
        result.payloads[index] = buffer.m_samples[index].payload;
    }
}

// The independent observed index is universally quantified by each entry point.
bool same_buffer_at(Buffer const & buffer, Snapshot const & before, std::size_t index)
{
    auto const time = buffer.m_samples[index].time_us;
    auto const payload = buffer.m_samples[index].payload;
    return buffer.m_length == before.length && buffer.m_oldest == before.oldest && buffer.m_write == before.write &&
           buffer.m_count == before.count && time == before.times[index] && payload == before.payloads[index];
}

// A logical offset is at most one length. Conditional reduction avoids a wide
// symbolic divider in the oracle while retaining every physical wrap position.
std::size_t slot(std::size_t oldest, std::size_t offset, std::size_t length)
{
    auto const sum = oldest + offset;
    return sum < length ? sum : sum - length;
}

std::size_t distance(std::size_t index, std::size_t oldest, std::size_t length)
{
    return index < oldest ? index + length - oldest : index - oldest;
}

// Inactive payloads remain arbitrary; only tags are zero.
bool invariant_at(Buffer const & buffer, std::size_t index)
{
    auto const length = buffer.m_length;
    if (length > capacity || buffer.m_count > length)
    {
        return false;
    }
    if (length == 0)
    {
        if (buffer.m_oldest != 0 || buffer.m_write != 0)
        {
            return false;
        }
    }
    else if (buffer.m_oldest >= length || buffer.m_write >= length ||
             buffer.m_write != slot(buffer.m_oldest, buffer.m_count, length))
    {
        return false;
    }
    bool const active = index < length && distance(index, buffer.m_oldest, length) < buffer.m_count;
    if (active != (buffer.m_samples[index].time_us != 0))
    {
        return false;
    }
    if (index > 0 && index < buffer.m_count)
    {
        auto const previous = slot(buffer.m_oldest, index - 1, length);
        auto const current = slot(buffer.m_oldest, index, length);
        auto const previous_time = buffer.m_samples[previous].time_us;
        auto const current_time = buffer.m_samples[current].time_us;
        return previous_time < current_time;
    }
    return true;
}

// Construct every invariant state for the registered capacities. Small seed
// integers cover ALL valid metadata values (including invalid length zero),
// avoiding unconstrained high index bits in the solver. Production size_t and
// uint64_t fields are unchanged. Inactive payloads and active times stay symbolic.
extern std::uint8_t nondet_u8();
extern std::uint64_t nondet_u64();
static_assert(capacity <= 255);
void seed_history(Buffer & buffer)
{
    std::uint8_t const length = nondet_u8();
    std::uint8_t const count = nondet_u8();
    std::uint8_t const oldest = nondet_u8();
    __ESBMC_assume(length <= capacity && count <= length);
    __ESBMC_assume(length == 0 ? oldest == 0 : oldest < length);
    buffer.m_length = length;
    buffer.m_count = count;
    buffer.m_oldest = oldest;
    buffer.m_write = length == 0 ? 0 : slot(oldest, count, length);
    for (std::size_t index = 0; index < capacity; ++index)
    {
        buffer.m_samples[index].payload = nondet_u64();
        bool const active = index < length && distance(index, oldest, length) < count;
        buffer.m_samples[index].time_us = active ? nondet_u64() : 0;
        __ESBMC_assume(!active || buffer.m_samples[index].time_us != 0);
    }
    for (std::size_t offset = 1; offset < count; ++offset)
    {
        auto const previous_time = buffer.m_samples[slot(oldest, offset - 1, length)].time_us;
        auto const current_time = buffer.m_samples[slot(oldest, offset, length)].time_us;
        __ESBMC_assume(previous_time < current_time);
    }
}

void assert_empty(Buffer const & buffer, std::size_t length)
{
    __ESBMC_assert(buffer.m_length == length && buffer.m_count == 0 && buffer.m_oldest == 0 && buffer.m_write == 0,
                   "buffer: empty indices and configured length");
    for (std::size_t index = 0; index < capacity; ++index)
    {
        __ESBMC_assert(same_sample(buffer.m_samples[index], Sample{}), "buffer: every backing slot is cleared");
    }
}

void verify_buffer_initialization(std::size_t length, std::size_t observed)
{
    __ESBMC_assume(observed < capacity);
    Buffer buffer(length);
    bool const valid = length > 0 && length <= capacity;
    __ESBMC_assert(buffer.valid() == valid, "constructor: accepts exactly the supported lengths");
    assert_empty(buffer, valid ? length : 0);
    __ESBMC_assert(invariant_at(buffer, observed), "constructor: establishes induction invariant");
}

void verify_buffer_allocate(std::size_t length, std::size_t observed)
{
    __ESBMC_assume(observed < capacity);
    Buffer buffer(0);
    seed_history(buffer);
    Snapshot before;
    snapshot(buffer, before);
    bool const valid = length > 0 && length <= capacity;
    bool const accepted = buffer.allocate(length);
    __ESBMC_assert(accepted == valid, "allocate: reports exactly the supported lengths");
    if (!valid || length == before.length)
    {
        __ESBMC_assert(same_buffer_at(buffer, before, observed),
                       "allocate: invalid or unchanged length preserves all history");
    }
    else
    {
        assert_empty(buffer, length);
    }
    __ESBMC_assert(invariant_at(buffer, observed), "allocate: preserves induction invariant");
}

void verify_buffer_reset(std::size_t observed)
{
    __ESBMC_assume(observed < capacity);
    Buffer buffer(0);
    seed_history(buffer);
    auto const length = buffer.m_length;
    buffer.reset();
    assert_empty(buffer, length);
    __ESBMC_assert(invariant_at(buffer, observed),
                   "reset: preserves induction invariant including invalid startup state");
}

void verify_buffer_push(std::uint64_t time, std::uint64_t payload, std::size_t observed)
{
    Sample sample{time, payload};
    __ESBMC_assume(observed < capacity);
    Buffer buffer(0);
    seed_history(buffer);
    __ESBMC_assume(buffer.m_length != 0 && sample.time_us != 0);
    Snapshot before;
    snapshot(buffer, before);
    if (before.count != 0)
    {
        __ESBMC_assume(sample.time_us > before.times[slot(before.oldest, before.count - 1, before.length)]);
    }
    auto const input = sample;
    buffer.push(sample);
    auto const count = before.count < before.length ? before.count + 1 : before.length;
    auto const dropped = before.count == before.length ? 1U : 0U;
    __ESBMC_assert(buffer.m_count == count && buffer.m_length == before.length &&
                       buffer.m_oldest == slot(before.oldest, dropped, before.length) &&
                       buffer.m_write == slot(before.write, 1, before.length),
                   "push: exact occupancy, rollover and overwrite indices");
    if (observed < count)
    {
        auto const actual = slot(buffer.m_oldest, observed, buffer.m_length);
        auto const time_after = buffer.m_samples[actual].time_us;
        auto const payload_after = buffer.m_samples[actual].payload;
        auto const prior = slot(before.oldest, dropped + observed, before.length);
        __ESBMC_assert(time_after == (observed + 1 == count ? time : before.times[prior]) &&
                           payload_after == (observed + 1 == count ? payload : before.payloads[prior]),
                       "push: retained logical sequence plus new sample");
    }
    if (observed != before.write)
    {
        __ESBMC_assert(buffer.m_samples[observed].time_us == before.times[observed] &&
                           buffer.m_samples[observed].payload == before.payloads[observed],
                       "push: every other physical slot unchanged");
    }
    __ESBMC_assert(same_sample(sample, input) && invariant_at(buffer, observed),
                   "push: input unchanged and induction invariant retained");
}

void verify_buffer_selection(std::uint64_t horizon, std::uint64_t age, std::uint64_t output_time,
                             std::uint64_t output_payload, std::size_t observed)
{
    Sample output{output_time, output_payload};
    __ESBMC_assume(observed < capacity);
    Buffer buffer(0);
    seed_history(buffer);
    Snapshot before;
    snapshot(buffer, before);
    auto const old_output = output;
    bool const actual = buffer.pop_first_older_than(horizon, &output, age);
    if (before.length == 0)
    {
        __ESBMC_assert(!actual && same_sample(output, old_output) && same_buffer_at(buffer, before, observed),
                       "selection: invalid empty buffer changes nothing");
        return;
    }
    __ESBMC_assert(buffer.m_count <= before.count, "selection: cannot grow the history");
    if (buffer.m_count > before.count)
    {
        return; // The preceding assertion owns this invalid case; avoid oracle underflow.
    }
    auto const removed = before.count - buffer.m_count;
    __ESBMC_assert(buffer.m_length == before.length && buffer.m_write == before.write &&
                       buffer.m_oldest == slot(before.oldest, removed, before.length),
                   "selection: retained suffix and exact indices");
    // Independent relational oracle: every old logical element is retired iff
    // at/before the horizon. No reverse scan or duplicate selection algorithm.
    if (observed < before.count)
    {
        auto const time = before.times[slot(before.oldest, observed, before.length)];
        __ESBMC_assert((observed < removed) == (time <= horizon),
                       "selection: consumes exactly the at-or-before prefix and retains the future suffix");
    }
    bool const retired = observed < before.length && distance(observed, before.oldest, before.length) < removed;
    __ESBMC_assert(buffer.m_samples[observed].time_us == (retired ? 0 : before.times[observed]) &&
                       buffer.m_samples[observed].payload == before.payloads[observed],
                   "selection: clears only retired tags, preserving future samples and all payloads");
    if (removed == 0)
    {
        __ESBMC_assert(!actual, "selection: no past sample means no selected output");
    }
    if (!actual)
    {
        __ESBMC_assert(output.time_us == output_time && output.payload == output_payload,
                       "selection: every failure preserves output");
    }
    // For every old logical position: the last retired element is the only
    // candidate. Quantifying the position avoids an oracle lookup indexed by
    // the result being checked; all time and payload values remain arbitrary.
    if (observed < before.count && observed + 1 == removed)
    {
        auto const newest = slot(before.oldest, observed, before.length);
        auto const time = before.times[newest];
        bool const eligible = time <= horizon && horizon - time < age;
        __ESBMC_assert(actual == eligible, "selection: newest eligible sample and strict age window");
        if (actual)
        {
            __ESBMC_assert(output.time_us == time && output.payload == before.payloads[newest],
                           "selection: exact last-retired timestamp and payload");
        }
    }
    __ESBMC_assert(invariant_at(buffer, observed), "selection: preserves induction invariant");
}

void verify_buffer_access(std::size_t index, std::uint64_t payload, std::size_t observed, std::uint64_t horizon,
                          std::uint64_t age)
{
    __ESBMC_assume(observed < capacity);
    Buffer buffer(0);
    seed_history(buffer);
    Snapshot before;
    snapshot(buffer, before);
    Buffer const & view = buffer;
    __ESBMC_assert(view.valid() == (before.length != 0) && view.get_length() == before.length &&
                       view.entries() == before.count && view.get_oldest_index() == before.oldest,
                   "access: configuration, count and physical oldest index");
    __ESBMC_assert(view.get_total_size() == sizeof(Buffer) && view.get_used_size() == sizeof(Buffer) -
                                                                                          sizeof(buffer.m_samples) +
                                                                                          before.count * sizeof(Sample),
                   "access: reserved object bytes and logical occupied bytes");
    if (before.count != 0)
    {
        Sample const & oldest = view.get_oldest();
        Sample const & newest = view.get_newest();
        Sample const & expected_oldest = buffer.m_samples[before.oldest];
        Sample const & expected_newest = buffer.m_samples[slot(before.oldest, before.count - 1, before.length)];
        __ESBMC_assert(&oldest == &expected_oldest && &newest == &expected_newest,
                       "access: non-consuming oldest/newest reference identity");
    }
    if (index < before.length)
    {
        Sample const & constant = view[index];
        Sample & writable = buffer[index];
        Sample & expected = buffer.m_samples[index];
        __ESBMC_assert(&constant == &expected && &writable == &expected,
                       "access: const and mutable indexing preserve physical identity");
    }
    __ESBMC_assert(same_buffer_at(buffer, before, observed), "access: all queries preserve the entire history");
    bool const selected = buffer.pop_first_older_than(horizon, nullptr, age);
    __ESBMC_assert(!selected && same_buffer_at(buffer, before, observed),
                   "access: null selection preserves all history");
    if (index < before.length)
    {
        buffer[index].payload = payload;
        __ESBMC_assert(buffer.m_samples[observed].time_us == before.times[observed] &&
                           buffer.m_samples[observed].payload ==
                               (observed == index ? payload : before.payloads[observed]),
                       "access: one indexed payload write preserves timestamps and other slots");
        __ESBMC_assert(buffer.m_length == before.length && buffer.m_count == before.count &&
                           buffer.m_oldest == before.oldest && buffer.m_write == before.write &&
                           invariant_at(buffer, observed),
                       "access: payload correction preserves metadata and induction invariant");
    }
}

// The general selection profile quantifies every explicit age. This separate
// profile checks that the two-argument API supplies exactly the promised window.
void verify_buffer_default_window(std::uint64_t time, std::uint64_t payload, std::uint64_t horizon,
                                  std::uint64_t output_time, std::uint64_t output_payload)
{
    __ESBMC_assume(time != 0);
    Buffer buffer(1);
    Sample const input{time, payload};
    buffer.push(input);
    Sample output{output_time, output_payload};
    bool const selected = buffer.pop_first_older_than(horizon, &output);
    bool const expected = time <= horizon && horizon - time < 100000;
    __ESBMC_assert(selected == expected && output.time_us == (expected ? time : output_time) &&
                       output.payload == (expected ? payload : output_payload),
                   "default window: strict 100 ms selection and failure frame");
    __ESBMC_assert(buffer.entries() == (time <= horizon ? 0 : 1),
                   "default window: retire past samples, retain future samples");
}

int main() { return 0; }
