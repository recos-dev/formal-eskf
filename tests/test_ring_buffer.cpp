// SPDX-License-Identifier: BSD-3-Clause
#include <array>
#include <limits>
#include <type_traits>
#include <vector>

#include <formal_eskf/runtime/ring_buffer.hpp>
#include "test_support.hpp"

namespace
{
using formal_eskf::runtime::RingBuffer;
using formal_eskf::test::Context;

struct Sample
{
    std::uint64_t time_us = 0;
    std::uint32_t value = 0;
    bool operator==(Sample const &) const = default;
};

// Caller fixtures exercise the production container, not a production estimator.
struct ImuInterval
{
    std::uint64_t time_us = 0;
    std::uint64_t interval_us = 0;
    int delta_velocity = 0;
};

struct Observation
{
    std::uint64_t time_us = 0;
    unsigned calibration = 0;
    bool in_air = false;
    bool qualified = false;
};

struct OutputSample
{
    std::uint64_t time_us = 0;
    int velocity = 0;
    int position = 0;
    bool operator==(OutputSample const &) const = default;
};

static_assert(!std::is_default_constructible_v<RingBuffer<Sample, 3>>);
static_assert(!std::is_copy_constructible_v<RingBuffer<Sample, 3>>);
static_assert(!std::is_copy_assignable_v<RingBuffer<Sample, 3>>);
static_assert(!std::is_move_constructible_v<RingBuffer<Sample, 3>>);
static_assert(!std::is_move_assignable_v<RingBuffer<Sample, 3>>);

void configuration_and_reset(Context & test)
{
    RingBuffer<Sample, 4> buffer(0);
    RingBuffer<Sample, 4> oversized(5);
    test.expect(!buffer.valid() && !oversized.valid(), "configuration", "invalid constructor lengths");
    test.expect(buffer.entries() == 0 && buffer.get_length() == 0, "configuration", "invalid buffer is empty");
    test.expect(buffer.allocate(2) && buffer.valid() && buffer.get_length() == 2, "configuration",
                "choose an active length within fixed storage");
    auto const reserved = buffer.get_total_size();
    auto const metadata = reserved - 4 * sizeof(Sample);
    test.expect(reserved == sizeof(buffer) && buffer.get_used_size() == metadata, "memory",
                "reserved storage includes all capacity slots");
    buffer.push({10, 1});
    test.expect(buffer.allocate(2) && buffer.entries() == 1 && buffer.get_oldest() == Sample{10, 1}, "configuration",
                "same length preserves history");
    test.expect(!buffer.allocate(0) && !buffer.allocate(5) && buffer.get_length() == 2 && buffer.entries() == 1 &&
                    buffer.get_newest() == Sample{10, 1},
                "configuration", "invalid reconfiguration preserves existing history");
    test.expect(buffer.get_used_size() == metadata + sizeof(Sample), "memory", "occupied payload counted once");
    test.expect(buffer.allocate(4) && buffer.entries() == 0 && buffer.get_length() == 4, "configuration",
                "changed length clears history");
    buffer.push({20, 2});
    buffer.push({30, 3});
    buffer.reset();
    test.expect(buffer.valid() && buffer.get_length() == 4 && buffer.entries() == 0 && buffer.get_oldest_index() == 0,
                "reset", "clear occupancy and indices without changing configured or reserved capacity");
    for (std::size_t index = 0; index < buffer.get_length(); ++index)
    {
        test.expect(buffer[index] == Sample{}, "reset", "all accessible slots cleared");
    }
    Sample output{77, 99};
    test.expect(!buffer.pop_first_older_than(100, &output) && output == Sample{77, 99}, "empty",
                "selection cannot publish stale storage");
    buffer.push({1, 4});
    test.expect(buffer.get_oldest() == Sample{1, 4} && buffer.get_newest() == Sample{1, 4}, "reset",
                "restart at a new source epoch");
}

template <std::size_t Capacity> void history_access(Context & test)
{
    RingBuffer<Sample, Capacity> buffer(Capacity);
    for (std::size_t index = 1; index <= 3 * Capacity; ++index)
    {
        buffer.push({index, static_cast<std::uint32_t>(index)});
        auto const count = index < Capacity ? index : Capacity;
        auto const oldest = index - count + 1;
        test.expect(buffer.entries() == count && buffer.get_length() == Capacity &&
                        buffer.get_oldest() == Sample{oldest, static_cast<std::uint32_t>(oldest)} &&
                        buffer.get_newest() == Sample{index, static_cast<std::uint32_t>(index)},
                    "history", "non-consuming oldest/newest views track rollover");
    }
    test.expect(buffer.get_used_size() == buffer.get_total_size(), "memory", "full backing storage counted once");
    auto index = buffer.get_oldest_index();
    for (std::size_t offset = 0; offset < buffer.entries(); ++offset)
    {
        auto const time = 2 * Capacity + offset + 1;
        test.expect(buffer[index] == Sample{time, static_cast<std::uint32_t>(time)}, "history",
                    "physical slots follow oldest index and configured length");
        // Output-history correction changes payloads while retaining their epochs.
        buffer[index].value += 100;
        index = (index + 1) % buffer.get_length();
    }
    RingBuffer<Sample, Capacity> const & view = buffer;
    test.expect(view[view.get_oldest_index()].value == 2 * Capacity + 101 &&
                    view.get_newest().value == 3 * Capacity + 100 && view.entries() == Capacity,
                "history", "payload correction is visible without consuming history");
}

void observation_selection(Context & test)
{
    RingBuffer<Sample, 4> buffer(4);
    for (std::uint32_t index = 1; index <= 4; ++index)
    {
        buffer.push({10 * index, index});
    }
    Sample output{77, 99};
    test.expect(!buffer.pop_first_older_than(9, &output, 15) && buffer.entries() == 4 && output == Sample{77, 99},
                "observation", "future samples stay pending");
    test.expect(!buffer.pop_first_older_than(30, nullptr, 15) && buffer.entries() == 4, "observation",
                "null output cannot consume history");
    test.expect(buffer.pop_first_older_than(30, &output, 15) && output == Sample{30, 3} && buffer.entries() == 1 &&
                    buffer.get_oldest() == Sample{40, 4},
                "observation", "newest eligible chosen; future retained; older entries retired");
    for (std::size_t index = 0; index < 3; ++index)
    {
        test.expect(buffer[index].time_us == 0, "observation", "retired slots have invalid timestamps");
    }
    test.expect(!buffer.pop_first_older_than(30, &output, 15) && output == Sample{30, 3}, "observation",
                "same horizon cannot reuse a sample");
    buffer.push({50, 5});
    test.expect(buffer.pop_first_older_than(40, &output, 1) && output == Sample{40, 4} && buffer.entries() == 1,
                "observation", "sample exactly on horizon eligible after physical wrap");
    test.expect(!buffer.pop_first_older_than(60, &output, 10) && output == Sample{40, 4} && buffer.entries() == 0,
                "observation", "age equal to limit expires without publishing output");
    buffer.push({55, 6});
    test.expect(buffer.pop_first_older_than(65, &output, 100) && output == Sample{55, 6}, "late",
                "source-ordered late arrival within age window remains usable");
}

void timestamp_boundaries(Context & test)
{
    RingBuffer<Sample, 3> buffer(3);
    buffer.push({1000000, 1});
    Sample output{77, 99};
    test.expect(buffer.pop_first_older_than(1099999, &output) && output == Sample{1000000, 1}, "PX4 window",
                "default accepts age 99999 us");
    buffer.push({2000000, 2});
    test.expect(!buffer.pop_first_older_than(2100000, &output) && output == Sample{1000000, 1}, "PX4 window",
                "default excludes age 100000 us");
    buffer.reset();
    constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
    buffer.push({1, 1});
    buffer.push({maximum - 1, 2});
    buffer.push({maximum, 3});
    test.expect(!buffer.pop_first_older_than(0, &output, maximum) && buffer.entries() == 3, "timestamp",
                "early horizon cannot underflow age");
    test.expect(buffer.pop_first_older_than(maximum - 1, &output, 1) && output == Sample{maximum - 1, 2} &&
                    buffer.entries() == 1,
                "timestamp", "age cannot overflow near UINT64_MAX");
    test.expect(!buffer.pop_first_older_than(maximum, &output, 0) && output == Sample{maximum - 1, 2} &&
                    buffer.entries() == 0,
                "timestamp", "zero window expires even an entry exactly on the horizon");
}

void imu_history_scenario(Context & test)
{
    RingBuffer<ImuInterval, 8> history(4);
    constexpr std::uint64_t delay_us = 30000;
    std::uint64_t predicted_time = 0; // Fixture seed is at the first interval's start.
    int velocity = 0;
    std::vector<std::uint64_t> predicted_epochs;
    for (int tick = 1; tick <= 7; ++tick)
    {
        // A full ring is normal; only loss of unprocessed history is a fault.
        bool const loses_required_imu =
            history.entries() == history.get_length() && history.get_oldest().time_us > predicted_time;
        test.expect(!loses_required_imu, "IMU scenario", "only already processed history rolls out");
        if (loses_required_imu)
        {
            return;
        }
        history.push({static_cast<std::uint64_t>(tick) * 10000, 10000, tick});
        // Repeated callbacks without a new completed interval must not predict twice.
        for (unsigned callback = 0; callback < 2; ++callback)
        {
            if (history.entries() != history.get_length())
            {
                continue;
            }
            auto const & delayed = history.get_oldest();
            if (history.get_newest().time_us - delayed.time_us < delay_us || delayed.time_us <= predicted_time)
            {
                continue;
            }
            test.expect(delayed.time_us - predicted_time == delayed.interval_us, "IMU scenario",
                        "each predicted interval begins at the committed state epoch");
            velocity += delayed.delta_velocity;
            predicted_time = delayed.time_us;
            predicted_epochs.push_back(predicted_time);
        }
    }
    test.expect(predicted_epochs == std::vector<std::uint64_t>{10000, 20000, 30000, 40000} && velocity == 10,
                "IMU scenario", "warmup, steady rollover and duplicate callbacks preserve every interval once");

    history.reset();
    predicted_time = 9000; // Start of the first 1 ms interval in this unsupported-rate case.
    for (std::uint64_t time = 10000; time <= 13000; time += 1000)
    {
        history.push({time, 1000, 1});
    }
    test.expect(history.entries() == history.get_length() &&
                    history.get_newest().time_us - history.get_oldest().time_us < delay_us,
                "IMU scenario", "fullness alone does not establish the required time span");
    bool const loses_required_imu =
        history.entries() == history.get_length() && history.get_oldest().time_us > predicted_time;
    // A caller must fault here, before a new push would hide the missing interval.
    if (!loses_required_imu)
    {
        history.push({14000, 1000, 1});
    }
    test.expect(loses_required_imu && history.get_oldest().time_us == 10000 && history.get_newest().time_us == 13000,
                "IMU scenario", "insufficient retention is detected before unprocessed history is overwritten");
}

void observation_scenario(Context & test)
{
    RingBuffer<Observation, 4> history(2);
    std::uint64_t admitted_time = 0;
    unsigned overwritten = 0;
    auto admit = [&](Observation const & sample)
    {
        if (sample.time_us == 0 || sample.time_us <= admitted_time)
        {
            return false;
        }
        if (history.entries() == history.get_length())
        {
            ++overwritten;
        }
        history.push(sample);
        admitted_time = sample.time_us;
        return true;
    };
    test.expect(admit({100, 1, false, false}) && admit({110, 1, false, true}), "observation scenario",
                "admit timestamped context and qualification together");
    test.expect(!admit({0, 1, true, true}) && !admit({110, 1, true, true}) && !admit({105, 1, true, true}),
                "observation scenario", "caller rejects invalid, duplicate and out-of-order source times");
    test.expect(admit({130, 1, true, false}) && overwritten == 1, "observation scenario",
                "caller records pending-observation loss before full push");
    Observation selected;
    test.expect(history.pop_first_older_than(115, &selected, 30) && selected.time_us == 110 && !selected.in_air &&
                    selected.qualified && selected.calibration == 1,
                "observation scenario", "selected context is historical, not the latest arrival's context");
    test.expect(history.entries() == 1 && history.get_oldest().time_us == 130 &&
                    !history.pop_first_older_than(115, &selected, 30) && selected.time_us == 110,
                "observation scenario", "future observation retained and consumed observation not reused");
    test.expect(history.pop_first_older_than(140, &selected, 30) && selected.time_us == 130 && !selected.qualified,
                "observation scenario", "selection does not turn rejected qualification into fusion permission");
    test.expect(admit({135, 1, true, true}) && history.pop_first_older_than(150, &selected, 30) &&
                    selected.time_us == 135,
                "observation scenario", "source-ordered late measurement remains eligible");
    test.expect(history.entries() == 0 && !admit({135, 1, true, true}) && !admit({134, 1, true, true}),
                "observation scenario", "caller watermark survives complete consumption");
    test.expect(admit({180, 1, true, true}), "observation scenario", "pending sample from old calibration");
    // Explicit source restart after its lifecycle/state reset policy has been resolved.
    history.reset();
    admitted_time = 0;
    test.expect(admit({50, 2, false, true}) && history.pop_first_older_than(180, &selected, 200) &&
                    selected.time_us == 50 && selected.calibration == 2 && history.entries() == 0,
                "observation scenario", "source restart invalidates pending old-calibration samples");
}

// Minimal output-owner fixture: no tracking-filter or ESKF mathematics is modeled.
bool correct_output_history(RingBuffer<OutputSample, 4> & history, std::uint64_t fusion_time, int velocity_delta,
                            int position_delta)
{
    if (history.entries() == 0 || history.get_oldest().time_us != fusion_time)
    {
        return false;
    }
    auto index = history.get_oldest_index();
    for (std::size_t offset = 0; offset < history.entries(); ++offset)
    {
        history[index].velocity += velocity_delta;
        history[index].position += position_delta;
        index = (index + 1) % history.get_length();
    }
    return true;
}

void output_history_scenario(Context & test)
{
    RingBuffer<OutputSample, 4> history(3);
    RingBuffer<ImuInterval, 4> imu(3);
    test.expect(!correct_output_history(history, 0, 5, 50), "output scenario", "empty history cannot supply feedback");
    for (int tick = 1; tick <= 4; ++tick)
    {
        history.push({static_cast<std::uint64_t>(tick) * 10000, tick, tick * 10});
        imu.push({static_cast<std::uint64_t>(tick) * 10000, 10000, tick});
    }
    test.expect(correct_output_history(history, imu.get_oldest().time_us, 5, 50), "output scenario",
                "matching delayed epochs permit history correction");
    std::array const expected{OutputSample{20000, 7, 70}, OutputSample{30000, 8, 80}, OutputSample{40000, 9, 90}};
    auto index = history.get_oldest_index();
    for (auto const & sample : expected)
    {
        test.expect(history[index] == sample, "output scenario",
                    "correction changes retained payloads, not timestamps");
        index = (index + 1) % history.get_length();
    }
    test.expect(history.entries() == 3 && history.get_newest() == expected.back(), "output scenario",
                "latest corrected state remains available without consuming history");
    imu.push({50000, 10000, 5});
    test.expect(history.get_length() == imu.get_length() &&
                    !correct_output_history(history, imu.get_oldest().time_us, 1, 1),
                "output scenario", "equal buffer lengths do not authorize feedback at different epochs");
    index = history.get_oldest_index();
    for (auto const & sample : expected)
    {
        test.expect(history[index] == sample, "output scenario", "epoch mismatch preserves every output sample");
        index = (index + 1) % history.get_length();
    }
    history.reset();
    imu.reset();
    test.expect(history.entries() == 0 && imu.entries() == 0 && !correct_output_history(history, 20000, 1, 1),
                "output scenario", "session reset invalidates history and old feedback together");
    history.push({100000, 0, 0});
    test.expect(history.entries() == 1 && history.get_newest() == OutputSample{100000, 0, 0}, "output scenario",
                "restart seeds a real epoch without pretending the history is full");
}

// Linear-list oracle has no ring indices. Samples satisfy caller admission preconditions.
template <std::size_t Capacity> void sequence_model(Context & test)
{
    RingBuffer<Sample, Capacity> buffer(Capacity);
    std::vector<Sample> pending;
    std::size_t length = Capacity;
    std::uint64_t last_time = 0;
    std::uint32_t random = 43;
    for (std::uint32_t step = 0; step < 4096; ++step)
    {
        random = random * 1664525U + 1013904223U;
        auto const choice = (random >> 16) % 10;
        if (choice < 6)
        {
            Sample const sample{last_time + choice + 1, step};
            if (pending.size() == length)
            {
                pending.erase(pending.begin());
            }
            pending.push_back(sample);
            last_time = sample.time_us;
            buffer.push(sample);
        }
        else if (choice < 9)
        {
            std::uint64_t const horizon = last_time / 2 + (random % (last_time + 20));
            std::uint64_t const age = (random >> 8) % 30;
            Sample expected{77, 99};
            bool selected = false;
            std::size_t removed = 0;
            for (auto const & sample : pending)
            {
                if (sample.time_us <= horizon)
                {
                    ++removed;
                    if (horizon - sample.time_us < age)
                    {
                        expected = sample;
                        selected = true;
                    }
                }
            }
            pending.erase(pending.begin(), pending.begin() + static_cast<std::ptrdiff_t>(removed));
            Sample output{77, 99};
            test.expect(buffer.pop_first_older_than(horizon, &output, age) == selected && output == expected,
                        "sequence", "observation selection agrees with linear model");
        }
        else
        {
            buffer.reset();
            pending.clear();
            last_time = 0;
            length = 1 + (random % Capacity);
            test.expect(buffer.allocate(length), "sequence", "bounded startup reconfiguration");
        }
        test.expect(buffer.entries() == pending.size() && buffer.get_length() == length, "sequence",
                    "active count and length agree with linear model");
        RingBuffer<Sample, Capacity> const & view = buffer;
        auto index = view.get_oldest_index();
        for (auto const & expected : pending)
        {
            test.expect(view[index] == expected, "sequence", "all retained timestamps and payloads agree");
            index = (index + 1) % length;
        }
        if (!pending.empty())
        {
            test.expect(view.get_oldest() == pending.front() && view.get_newest() == pending.back(), "sequence",
                        "non-consuming endpoint views agree");
        }
    }
}
} // namespace

int main()
{
    Context test;
    configuration_and_reset(test);
    history_access<1>(test);
    history_access<3>(test);
    history_access<257>(test);
    observation_selection(test);
    timestamp_boundaries(test);
    imu_history_scenario(test);
    observation_scenario(test);
    output_history_scenario(test);
    sequence_model<1>(test);
    sequence_model<2>(test);
    sequence_model<3>(test);
    sequence_model<8>(test);
    return test.failures() == 0 ? 0 : 1;
}
