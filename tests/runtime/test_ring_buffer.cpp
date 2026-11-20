// SPDX-License-Identifier: BSD-3-Clause
#include <array>
#include <limits>
#include <type_traits>
#include <vector>

#include <formal_eskf/runtime/ring_buffer.hpp>
#include "test_cases.hpp"

namespace
{
using formal_eskf::runtime::RingBuffer;
using formal_eskf::test::Case;
using formal_eskf::test::Context;

struct Sample
{
    std::uint64_t time_us = 0;
    std::uint32_t value = 0;
    bool operator==(Sample const &) const = default;
};

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

// Bounded host observer: observes assignments through Sample's normal value API.
// It does not access RingBuffer's private representation or allocate storage.
struct ObservedSample
{
    std::uint64_t time_us{};
    std::uint32_t value{};
    static inline bool observing = false;
    static inline std::size_t assignments = 0;
    static inline std::size_t zero_assignments = 0;
    static inline std::array<ObservedSample const *, 257> destinations{};

    ObservedSample() = default;
    ObservedSample(std::uint64_t time, std::uint32_t payload) : time_us(time), value(payload) {}
    ObservedSample(ObservedSample const &) = default;
    ObservedSample & operator=(ObservedSample const & sample)
    {
        time_us = sample.time_us;
        value = sample.value;
        if (observing)
        {
            if (assignments < destinations.size())
            {
                destinations[assignments] = this;
            }
            ++assignments;
            zero_assignments += time_us == 0 && value == 0 ? 1 : 0;
        }
        return *this;
    }
    static void begin()
    {
        assignments = 0;
        zero_assignments = 0;
        destinations.fill(nullptr);
        observing = true;
    }
    static void end() { observing = false; }
};

void construction(Context & test, std::string_view profile)
{
    using Buffer = RingBuffer<Sample, 3>;
    static_assert(!std::is_default_constructible_v<Buffer>);
    static_assert(!std::is_copy_constructible_v<Buffer> && !std::is_copy_assignable_v<Buffer>);
    static_assert(!std::is_move_constructible_v<Buffer> && !std::is_move_assignable_v<Buffer>);
    for (std::size_t length : {0U, 1U, 3U, 4U})
    {
        Buffer buffer(length);
        bool const valid = length >= 1 && length <= 3;
        test.expect(buffer.valid() == valid && buffer.entries() == 0, profile, "constructor length domain");
        test.expect(buffer.get_length() == (valid ? length : 0) && buffer.get_oldest_index() == 0, profile,
                    "construction establishes empty metadata");
    }
}

void allocation_and_reset(Context & test, std::string_view profile)
{
    RingBuffer<ObservedSample, 8> buffer(3);
    for (std::uint64_t time = 1; time <= 8; ++time)
    {
        buffer.push({time, static_cast<std::uint32_t>(time * 17)});
    }
    auto const oldest = buffer.get_oldest_index();
    auto const oldest_time = buffer.get_oldest().time_us;
    auto const newest_time = buffer.get_newest().time_us;
    ObservedSample::begin();
    bool const unchanged = buffer.allocate(3) && !buffer.allocate(0) && !buffer.allocate(9);
    ObservedSample::end();
    test.expect(unchanged && ObservedSample::assignments == 0 && buffer.entries() == 3 &&
                    buffer.get_oldest_index() == oldest && buffer.get_oldest().time_us == oldest_time &&
                    buffer.get_newest().time_us == newest_time,
                profile, "same/invalid length preserves metadata and every storage assignment");

    buffer.push({9, 153});
    test.expect(buffer.entries() == 3 && buffer.get_oldest().time_us == 7 && buffer.get_newest().time_us == 9, profile,
                "same/invalid allocation preserves next-write position for continuation");
    auto slot = buffer.get_oldest_index();
    for (std::uint64_t time = 7; time <= 9; ++time)
    {
        test.expect(buffer[slot].time_us == time && buffer[slot].value == time * 17, profile,
                    "continued history retains exactly the expected suffix");
        slot = (slot + 1) % 3;
    }

    for (bool change_length : {false, true})
    {
        ObservedSample::begin();
        if (change_length)
        {
            test.expect(buffer.allocate(2), profile, "changed valid length accepted");
        }
        else
        {
            buffer.reset();
        }
        ObservedSample::end();
        test.expect(ObservedSample::assignments == 8 && ObservedSample::zero_assignments == 8, profile,
                    "reset clears full backing, including inaccessible unused slots");
        for (std::size_t index = 0; index < 8; ++index)
        {
            test.expect(ObservedSample::destinations[index] != nullptr, profile, "cleared destination exists");
            for (std::size_t prior = 0; prior < index; ++prior)
            {
                test.expect(ObservedSample::destinations[index] != ObservedSample::destinations[prior], profile,
                            "each backing slot cleared exactly once");
            }
        }
        test.expect(buffer.entries() == 0 && buffer.get_oldest_index() == 0 &&
                        buffer.get_length() == (change_length ? 2U : 3U),
                    profile, "reset preserves selected length and clears indices");
        buffer.push({1, 17});
        test.expect(buffer.get_oldest().time_us == 1 && buffer.get_newest().value == 17, profile,
                    "new timeline after explicit reset");
    }
}

template <std::size_t Capacity> void push_history(Context & test, std::string_view profile)
{
    for (std::size_t length : {std::size_t{1}, (Capacity + 1) / 2, Capacity})
    {
        RingBuffer<Sample, Capacity> buffer(length);
        for (std::size_t number = 1; number <= 3 * length; ++number)
        {
            Sample const input{number, static_cast<std::uint32_t>(number * 17)};
            buffer.push(input);
            auto const count = std::min(number, length);
            test.expect(buffer.entries() == count && buffer.get_oldest().time_us == number - count + 1 &&
                            buffer.get_newest() == input && input.value == number * 17,
                        profile, "push retains exactly the newest bounded chronological suffix");
            auto physical = buffer.get_oldest_index();
            for (std::size_t offset = 0; offset < count; ++offset)
            {
                auto const time = number - count + 1 + offset;
                test.expect(buffer[physical] == Sample{time, static_cast<std::uint32_t>(time * 17)}, profile,
                            "all live timestamps and payloads retain chronological order");
                physical = (physical + 1) % length;
            }
        }
    }
}

void push_cases(Context & test, std::string_view profile)
{
    push_history<1>(test, profile);
    push_history<2>(test, profile);
    push_history<3>(test, profile);
    push_history<8>(test, profile);
    push_history<257>(test, profile);
}

void selection(Context & test, std::string_view profile)
{
    for (std::size_t retained : {0U, 1U, 3U})
    {
        for (bool wrap : {false, true})
        {
            for (std::uint64_t horizon : {0U, 19U, 20U, 25U, 30U, 39U, 40U, 50U})
            {
                RingBuffer<Sample, 3> buffer(3);
                if (wrap)
                {
                    buffer.push({1, 1});
                    buffer.push({2, 2});
                    Sample discarded;
                    test.expect(buffer.pop_first_older_than(2, &discarded), profile, "prepare wrapped empty ring");
                }
                std::vector<Sample> pending;
                for (std::size_t index = 0; index < retained; ++index)
                {
                    Sample const input{20 + 10 * index, static_cast<std::uint32_t>(index + 1)};
                    pending.push_back(input);
                    buffer.push(input);
                }
                Sample expected{777, 999};
                std::size_t retired = 0;
                for (auto const & input : pending)
                {
                    if (input.time_us <= horizon)
                    {
                        expected = input;
                        ++retired;
                    }
                }
                Sample output{777, 999};
                bool const selected = buffer.pop_first_older_than(horizon, &output, 100);
                test.expect(selected == (retired > 0) && output == expected && buffer.entries() == retained - retired,
                            profile, "newest eligible sample selected, prefix retired and future preserved");
                if (retired < retained)
                {
                    test.expect(buffer.get_oldest() == pending[retired] && buffer.get_newest() == pending.back(),
                                profile, "future endpoint payloads remain intact");
                }
            }
        }
    }
}

void age_boundary(Context & test, std::string_view profile)
{
    for (std::uint64_t age : {0U, 9U, 10U, 11U, 99999U, 100000U, 100001U})
    {
        for (std::uint64_t window : {0U, 10U, 100000U})
        {
            RingBuffer<Sample, 3> buffer(3);
            buffer.push({90, 9});
            buffer.push({100, 10});
            buffer.push({100 + age + 1, 11});
            Sample output{777, 999};
            bool const selected = window == 100000 ? buffer.pop_first_older_than(100 + age, &output)
                                                   : buffer.pop_first_older_than(100 + age, &output, window);
            test.expect(selected == (age < window) && output == (selected ? Sample{100, 10} : Sample{777, 999}),
                        profile, "strict age boundary and omitted default window");
            test.expect(buffer.entries() == 1 && buffer.get_oldest() == Sample{100 + age + 1, 11} &&
                            buffer[0].time_us == 0 && buffer[1].time_us == 0,
                        profile, "selected or expired prefix retired; future untouched");
        }
    }
}

void null_output(Context & test, std::string_view profile)
{
    for (std::size_t pushes : {0U, 1U, 3U, 8U})
    {
        for (std::uint64_t horizon : {0U, 3U, 1000000U})
        {
            RingBuffer<Sample, 3> buffer(3);
            RingBuffer<Sample, 3> control(3);
            for (std::size_t time = 1; time <= pushes; ++time)
            {
                Sample const input{time, static_cast<std::uint32_t>(time)};
                buffer.push(input);
                control.push(input);
            }
            std::array<Sample, 3> before{buffer[0], buffer[1], buffer[2]};
            auto const count = buffer.entries();
            auto const oldest = buffer.get_oldest_index();
            test.expect(!buffer.pop_first_older_than(horizon, nullptr, 1), profile, "null output fails");
            test.expect(buffer.entries() == count && buffer.get_oldest_index() == oldest && buffer.get_length() == 3,
                        profile, "null output preserves metadata");
            for (std::size_t index = 0; index < 3; ++index)
            {
                test.expect(buffer[index] == before[index], profile, "null output preserves every payload/tag");
            }
            Sample const next{pushes + 1, 123};
            buffer.push(next);
            control.push(next);
            test.expect(buffer.get_newest() == control.get_newest() && buffer.get_oldest() == control.get_oldest() &&
                            buffer.entries() == control.entries(),
                        profile, "common continuation detects hidden next-write mutation");
            Sample output{}, expected{};
            bool const selected = buffer.pop_first_older_than(pushes + 1, &output);
            bool const control_selected = control.pop_first_older_than(pushes + 1, &expected);
            test.expect(selected == control_selected && output == expected && buffer.entries() == control.entries(),
                        profile, "continued selection matches unchanged control");
        }
    }
}

void time_and_horizon_aliases(Context & test, std::string_view profile)
{
    constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
    RingBuffer<Sample, 3> buffer(3);
    buffer.push({1, 1});
    buffer.push({maximum - 1, 2});
    buffer.push({maximum, 3});
    Sample output{777, 999};
    test.expect(!buffer.pop_first_older_than(0, &output, maximum) && buffer.entries() == 3 &&
                    output == Sample{777, 999},
                profile, "future data does not underflow age");
    test.expect(buffer.pop_first_older_than(maximum - 1, &output, 1) && output == Sample{maximum - 1, 2} &&
                    buffer.entries() == 1,
                profile, "ordered subtraction near UINT64_MAX");
    test.expect(!buffer.pop_first_older_than(maximum, &output, 0) && output == Sample{maximum - 1, 2}, profile,
                "zero window excludes exact horizon");
    for (bool alias_storage : {false, true})
    {
        buffer.reset();
        buffer.push({10, 1});
        buffer.push({20, 2});
        buffer.push({30, 3});
        output = {20, 999};
        std::uint64_t const & horizon = alias_storage ? buffer[1].time_us : output.time_us;
        test.expect(buffer.pop_first_older_than(horizon, &output, 100) && output == Sample{20, 2}, profile,
                    "borrowed horizon may alias receiving timestamp or retired slot");
        test.expect(buffer.entries() == 1 && buffer.get_oldest() == Sample{30, 3} && buffer[1].time_us == 0, profile,
                    "entry horizon used before permitted alias mutation");
    }
}

void slot_access(Context & test, std::string_view profile)
{
    RingBuffer<Sample, 3> buffer(3);
    for (std::uint64_t time = 1; time <= 5; ++time)
    {
        buffer.push({time, static_cast<std::uint32_t>(time)});
    }
    auto const oldest = buffer.get_oldest_index();
    RingBuffer<Sample, 3> const & view = buffer;
    for (int repetition = 0; repetition < 3; ++repetition)
    {
        test.expect(view.get_oldest() == Sample{3, 3} && view.get_newest() == Sample{5, 5} && view.entries() == 3 &&
                        view.get_oldest_index() == oldest,
                    profile, "endpoint views are nonconsuming and const slot access is physical");
    }
    buffer[oldest].value = 123;
    Sample output;
    test.expect(buffer.pop_first_older_than(3, &output, 1) && output == Sample{3, 123}, profile,
                "payload edit retained without timestamp mutation");
    test.expect(buffer.get_oldest() == Sample{4, 4} && buffer.get_newest() == Sample{5, 5}, profile,
                "future views follow pop");
}

void size_reporting(Context & test, std::string_view profile)
{
    RingBuffer<Sample, 257> buffer(257);
    auto const metadata = sizeof(buffer) - sizeof(std::array<Sample, 257>);
    for (std::size_t count = 0; count <= 257; ++count)
    {
        test.expect(buffer.entries() == count && buffer.get_length() == 257 &&
                        buffer.get_total_size() == sizeof(buffer) &&
                        buffer.get_used_size() == metadata + count * sizeof(Sample),
                    profile, "size_t occupancy and logical byte reporting beyond 255 entries");
        if (count < 257)
        {
            buffer.push({count + 1, static_cast<std::uint32_t>(count)});
        }
    }
    test.expect(buffer.allocate(1) && buffer.get_total_size() == sizeof(buffer) && buffer.get_used_size() == metadata,
                profile, "logical length does not resize reserved backing");
}

void bounded_copies(Context & test, std::string_view profile)
{
    RingBuffer<ObservedSample, 257> buffer(257);
    for (std::uint64_t time = 1; time <= 257; ++time)
    {
        ObservedSample::begin();
        buffer.push({time, static_cast<std::uint32_t>(time)});
        ObservedSample::end();
        test.expect(ObservedSample::assignments == 1, profile, "push copies exactly one bounded payload");
    }
    ObservedSample output;
    ObservedSample::begin();
    bool const selected = buffer.pop_first_older_than(257, &output);
    ObservedSample::end();
    test.expect(selected && output.time_us == 257 && ObservedSample::assignments == 1, profile,
                "pop copies one selected payload, retires tags without payload copies");
    ObservedSample::begin();
    buffer.reset();
    ObservedSample::end();
    test.expect(ObservedSample::assignments == 257 && ObservedSample::zero_assignments == 257, profile,
                "reset work scales with compile-time capacity");
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

template <std::size_t Capacity> void exhaustive_short_sequences(Context & test)
{
    constexpr std::size_t steps = 5;
    constexpr std::size_t combinations = 1U << (2U * steps);
    for (std::size_t sequence = 0; sequence < combinations; ++sequence)
    {
        RingBuffer<Sample, Capacity> buffer(Capacity);
        std::vector<Sample> pending;
        std::size_t length = Capacity;
        std::uint64_t next_time = 5;
        auto choices = sequence;
        for (std::size_t step = 0; step < steps; ++step)
        {
            auto const operation = choices % 4;
            choices /= 4;
            if (operation == 0)
            {
                Sample const input{next_time, static_cast<std::uint32_t>(step + 1)};
                next_time += 5;
                if (pending.size() == length)
                {
                    pending.erase(pending.begin());
                }
                pending.push_back(input);
                buffer.push(input);
            }
            else if (operation == 1)
            {
                std::uint64_t const horizon = next_time > 5 ? next_time - 6 : 0;
                Sample expected{777, 999};
                std::size_t removed = 0;
                for (auto const & input : pending)
                {
                    if (input.time_us <= horizon)
                    {
                        ++removed;
                        expected = input;
                    }
                }
                pending.erase(pending.begin(), pending.begin() + static_cast<std::ptrdiff_t>(removed));
                Sample output{777, 999};
                test.expect(buffer.pop_first_older_than(horizon, &output, 100) == (removed != 0) && output == expected,
                            "exhaustive", "list-model pop in every five-operation sequence");
            }
            else if (operation == 2)
            {
                buffer.reset();
                pending.clear();
                next_time = 5;
            }
            else
            {
                std::size_t const requested = length == Capacity ? 1 : Capacity;
                if (requested != length)
                {
                    pending.clear();
                    next_time = 5;
                }
                length = requested;
                test.expect(buffer.allocate(length), "exhaustive", "valid reconfiguration");
            }
            test.expect(buffer.entries() == pending.size() && buffer.get_length() == length &&
                            buffer.get_oldest_index() < length,
                        "exhaustive", "metadata matches independent list after each action");
            std::array<bool, Capacity> active{};
            auto index = buffer.get_oldest_index();
            for (auto const & expected : pending)
            {
                active[index] = true;
                test.expect(buffer[index] == expected, "exhaustive", "all retained payloads match");
                index = (index + 1) % length;
            }
            for (std::size_t slot = 0; slot < length; ++slot)
            {
                if (!active[slot])
                {
                    test.expect(buffer[slot].time_us == 0, "exhaustive", "inactive timestamp remains invalid");
                }
            }
        }
    }
}

void sequences(Context & test, std::string_view)
{
    exhaustive_short_sequences<1>(test);
    exhaustive_short_sequences<2>(test);
    exhaustive_short_sequences<3>(test);
    sequence_model<1>(test);
    sequence_model<2>(test);
    sequence_model<3>(test);
    sequence_model<8>(test);
    sequence_model<257>(test);
    // Supplementary payload-use regressions; these are not adapter integration evidence.
    imu_history_scenario(test);
    observation_scenario(test);
    output_history_scenario(test);
}
} // namespace

int main(int argc, char ** argv)
{
    constexpr std::array cases{
        Case{"TC-R-RING-BUFFER-01", "portable", construction},
        Case{"TC-R-RING-BUFFER-02", "portable", allocation_and_reset},
        Case{"TC-R-RING-BUFFER-03", "portable", push_cases},
        Case{"TC-R-RING-BUFFER-04", "portable", selection},
        Case{"TC-R-RING-BUFFER-05", "portable", age_boundary},
        Case{"TC-R-RING-BUFFER-06", "portable", null_output},
        Case{"TC-R-RING-BUFFER-07", "portable", time_and_horizon_aliases},
        Case{"TC-R-RING-BUFFER-08", "portable", slot_access},
        Case{"TC-R-RING-BUFFER-09", "portable", size_reporting},
        Case{"TC-R-RING-BUFFER-10", "portable", sequences},
        Case{"TC-R-RING-BUFFER-11", "host-copy-bound", bounded_copies},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
