// SPDX-License-Identifier: BSD-3-Clause
#include <array>
#include <bit>
#include <cstdint>
#include <limits>
#include <vector>

#include "../integrations/px4/module/ImuHistory.hpp"
#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{
using formal_eskf::test::Context;
using Backend = formal_eskf::test::Backend<double>;
using History = formal_eskf::px4::ImuHistory<Backend>;
using Input = History::input_type;
using Fault = History::Fault;
using Parameters = formal_eskf::px4::TimingParameters;
using Configuration = formal_eskf::px4::TimingConfiguration;
using formal_eskf::px4::configure_timing;

Input sample(std::uint64_t time_us, double rate = 1.)
{
    Input result;
    result.time_us = time_us;
    result.angular_period_us = 4000;
    result.acceleration_period_us = 4000;
    result.delta_angle_b = Input::vector3_type::from_row_major({0.004 * rate, 0.008, -0.012});
    result.delta_velocity_b = Input::vector3_type::from_row_major({0.016, -0.020 * rate, -0.024});
    result.source = {17, 29, 2, 3};
    return result;
}

bool equal(Configuration const & a, Configuration const & b)
{
    return a.imu_length == b.imu_length && a.minimum_lag_us == b.minimum_lag_us && a.gnss_delay_us == b.gnss_delay_us &&
           a.magnetic_delay_us == b.magnetic_delay_us && a.barometer_delay_us == b.barometer_delay_us;
}

bool equal(Input const & a, Input const & b)
{
    bool result = a.time_us == b.time_us && a.angular_period_us == b.angular_period_us &&
                  a.acceleration_period_us == b.acceleration_period_us && a.source.matches(b.source) &&
                  a.accelerometer_clipping == b.accelerometer_clipping && a.gyroscope_clipping == b.gyroscope_clipping;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        result &= std::bit_cast<std::uint64_t>(a.delta_angle_b(axis)) ==
                      std::bit_cast<std::uint64_t>(b.delta_angle_b(axis)) &&
                  std::bit_cast<std::uint64_t>(a.delta_velocity_b(axis)) ==
                      std::bit_cast<std::uint64_t>(b.delta_velocity_b(axis));
    }
    return result;
}

void configuration(Context & test)
{
    Configuration output;
    test.expect(configure_timing({}, output) && output.imu_length == 54 && output.minimum_lag_us == 200000,
                "configuration", "default 250 Hz mode derives 54 slots");
    struct Case
    {
        Parameters parameters{};
        std::size_t length{}; // Zero denotes rejection.
    };
    std::array<Case, 15> const cases{{
        {{4000, 230, 0, 0, 0}, 62},
        {{4000, 239, 0, 0, 0}, 64},
        {{4000, 240, 0, 0, 0}, 0},
        {{4000, 300, 0, 0, 0}, 0},
        {{10000, 200, 0, 0, 0}, 0},
        {{2500, 200, 0, 0, 0}, 0},
        {{2000, 200, 0, 0, 0}, 0},
        {{4000, 0, 0, 0, 0}, 0},
        {{4000, -1, 0, 0, 0}, 0},
        {{4000, 152, 0, 0, 0}, 0},
        {{4000, 153, 0, 0, 0}, 42},
        {{4000, 200, 47, 97, 97}, 54},
        {{4000, 200, 48, 0, 0}, 0},
        {{4000, 200, 0, 98, 0}, 0},
        {{4000, 200, 0, 0, 98}, 0},
    }};
    for (auto const & entry : cases)
    {
        auto const before = output;
        bool const accepted = configure_timing(entry.parameters, output);
        test.expect(accepted == (entry.length != 0), "configuration", "validate mode, capacity and arrival coverage");
        test.expect(accepted ? output.imu_length == entry.length : equal(output, before), "configuration",
                    "derive requested length or preserve previous configuration");
    }
    for (std::int32_t value : {-1, std::numeric_limits<std::int32_t>::min(), std::numeric_limits<std::int32_t>::max()})
    {
        std::array<Parameters, 5> const invalid{{{value, 200, 0, 0, 0},
                                                 {4000, value, 0, 0, 0},
                                                 {4000, 200, value, 0, 0},
                                                 {4000, 200, 0, value, 0},
                                                 {4000, 200, 0, 0, value}}};
        for (auto const & parameters : invalid)
        {
            auto const before = output;
            test.expect(!configure_timing(parameters, output) && equal(output, before), "configuration",
                        "signed boundaries and large millisecond conversion fail without output writes");
        }
    }
    test.expect(configure_timing({4000, 200, 12, 23, 34}, output) && output.gnss_delay_us == 12000 &&
                    output.magnetic_delay_us == 23000 && output.barometer_delay_us == 34000,
                "configuration", "convert each source delay to microseconds exactly once");
}

void startup(Context & test)
{
    History history;
    auto const initial = sample(1000000);
    test.expect(!history.seed(initial, initial.time_us) && !history.push(initial, initial.time_us) &&
                    history.delayed() == nullptr && history.entries() == 0,
                "startup", "unconfigured history cannot accept a seed or sample");
    test.expect(history.configure({}), "startup", "configure storage without a seed");
    auto const configuration_before = history.configuration();
    test.expect(!history.configure({10000, 200, 0, 0, 0}) && equal(history.configuration(), configuration_before),
                "startup", "bad replacement retains configured length");
    test.expect(!history.seed(sample(0), initial.time_us) && !history.seed(initial, initial.time_us - 1) &&
                    !history.seed(initial, initial.time_us + 100001) && history.latest_time_us() == 0,
                "startup", "zero, future and stale seed do not consume the seed epoch");
    test.expect(history.seed(initial, initial.time_us) && history.entries() == 0 &&
                    history.fusion_time_us() == initial.time_us && history.delayed() == nullptr,
                "startup", "seed is an epoch and preceding rate, not fabricated retained IMU");
    test.expect(!history.seed(initial, initial.time_us) && !history.configure({4000, 230, 0, 0, 0}) &&
                    history.fault() == Fault::none && equal(history.configuration(), configuration_before),
                "startup", "live reconfiguration and reseeding are rejected without disturbing the session");
    history.reset();
    test.expect(history.entries() == 0 && history.latest_time_us() == 0 && history.fusion_time_us() == 0 &&
                    history.fault() == Fault::none && equal(history.configuration(), configuration_before),
                "restart", "clear temporal state and preserve configuration");
    test.expect(history.configure({4000, 239, 0, 0, 0}) && history.configuration().imu_length == 64 &&
                    history.seed(initial, initial.time_us),
                "restart", "select full backing capacity only before a new seed");
}

void ordered_prediction(Context & test, Parameters const & parameters, std::uint64_t epoch)
{
    History history;
    test.expect(history.configure(parameters) && history.seed(sample(epoch), epoch), "ordered", "configure and seed");
    std::vector<Input> pending; // Independent host reference; no container enters production history.
    std::uint64_t predicted_time = epoch;
    std::size_t predictions = 0;
    auto const length = history.configuration().imu_length;
    for (std::size_t tick = 1; tick <= 3 * length; ++tick)
    {
        auto input = sample(epoch + tick * 4000, static_cast<double>(tick));
        input.delta_angle_b(2) = tick % 2 == 0 ? -0. : 0.;
        input.acceleration_period_us = tick % 2 == 0 ? 3800 : 4200;
        input.accelerometer_clipping = static_cast<std::uint8_t>(tick % 8);
        pending.push_back(input);
        test.expect(history.push(input, input.time_us), "ordered", "valid real interval admitted");
        auto const * delayed = history.delayed();
        if (tick < length)
        {
            test.expect(delayed == nullptr && history.fusion_time_us() == epoch, "warmup",
                        "no delayed prediction before full real history");
            continue;
        }
        test.expect(delayed != nullptr, "ordered", "one delayed interval becomes eligible");
        if (delayed == nullptr)
        {
            return;
        }
        auto const & expected = pending.front();
        test.expect(equal(delayed->sample, expected) && delayed->time_us == expected.time_us &&
                        delayed->start_time_us == predicted_time && delayed->hold_period_us == 0,
                    "ordered", "all raw values, periods, clipping and source fields reach the original endpoint");
        test.expect(history.delayed() == delayed && history.fusion_time_us() == predicted_time, "ordered",
                    "viewing a record does not acknowledge prediction");
        predicted_time = expected.time_us;
        pending.erase(pending.begin());
        ++predictions;
        test.expect(history.complete(predicted_time) && history.delayed() == nullptr &&
                        history.fusion_time_us() == predicted_time && history.entries() == length,
                    "ordered", "successful prediction advances once while retaining processed history");
    }
    test.expect(predictions == 2 * length + 1 && history.fault() == Fault::none, "ordered",
                "multiple wraps never repeat or skip an interval");
    history.reset();
    auto next_seed = sample(2000000);
    next_seed.source = {37, 49, 0, 0};
    test.expect(history.seed(next_seed, next_seed.time_us) && history.entries() == 0 && history.delayed() == nullptr,
                "restart", "populated history, old clock and old source are discarded only by restart");
    auto next = next_seed;
    next.time_us += 4000;
    test.expect(history.push(next, next.time_us) && history.entries() == 1 && history.delayed() == nullptr, "restart",
                "new source begins a fresh warmup without a previous delayed record");
}

void gap_segments(Context & test)
{
    for (std::uint32_t elapsed : {3800U, 4200U, 4201U, 50000U})
    {
        History history;
        auto const seed = sample(1000000, 9.);
        test.expect(history.configure({}) && history.seed(seed, seed.time_us), "gap", "setup");
        auto input = sample(seed.time_us + elapsed, 2.);
        test.expect(history.push(input, input.time_us), "gap", "accept tolerance endpoints and permitted gaps");
        for (std::size_t i = 1; i < history.configuration().imu_length; ++i)
        {
            input = sample(input.time_us + 4000, 3.);
            test.expect(history.push(input, input.time_us), "gap", "warm history after first interval");
        }
        auto const * delayed = history.delayed();
        test.expect(delayed != nullptr, "gap", "original interval eventually selected");
        if (delayed != nullptr)
        {
            std::uint32_t const hold = elapsed > 4200 ? elapsed - 4000 : 0;
            test.expect(delayed->start_time_us == seed.time_us && delayed->time_us == seed.time_us + elapsed &&
                            delayed->hold_period_us == hold && delayed->sample.angular_period_us == 4000,
                        "gap", "synthetic hold and measured period are separate in one record");
            if (hold != 0)
            {
                auto const expected = seed.mean_rates();
                for (std::size_t axis = 0; axis < 3; ++axis)
                {
                    test.expect(delayed->held_rate.specific_force_b(axis) == expected.specific_force_b(axis) &&
                                    delayed->held_rate.angular_rate_b(axis) == expected.angular_rate_b(axis),
                                "gap", "hold uses the preceding measured rate, not the newest rate");
                }
            }
            test.expect(history.complete(delayed->time_us), "gap", "complete the whole interval only once");
        }
    }
}

void admission_failures(Context & test)
{
    auto const seed = sample(1000000);
    auto const good = sample(1004000);
    struct Case
    {
        Input input{};
        std::uint64_t now_us{};
        Fault fault = Fault::none;
    };
    std::vector<Case> cases;
    auto add = [&](Input const & input, Fault fault) { cases.push_back({input, input.time_us, fault}); };
    add(seed, Fault::invalid_sample);
    add(sample(seed.time_us - 1), Fault::invalid_sample);
    add(sample(0), Fault::invalid_sample);
    add(sample(seed.time_us + 3799), Fault::discontinuity);
    add(sample(seed.time_us + 50001), Fault::discontinuity);
    cases.push_back({good, good.time_us - 1, Fault::invalid_sample});
    cases.push_back({good, good.time_us + 100001, Fault::invalid_sample});
    auto invalid = good;
    invalid.angular_period_us = 3799;
    add(invalid, Fault::invalid_sample);
    invalid = good;
    invalid.acceleration_period_us = 4201;
    add(invalid, Fault::invalid_sample);
    invalid = sample(seed.time_us + 3800);
    invalid.angular_period_us = 4001;
    add(invalid, Fault::discontinuity); // Negative gap just beyond the 200 us tolerance.
    invalid = good;
    invalid.gyroscope_clipping = 1;
    add(invalid, Fault::invalid_sample);
    invalid = good;
    invalid.accelerometer_clipping = 8;
    add(invalid, Fault::invalid_sample);
    for (double value : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::max()})
    {
        invalid = good;
        invalid.delta_angle_b(0) = value;
        add(invalid, Fault::invalid_sample);
        invalid = good;
        invalid.delta_velocity_b(2) = value;
        add(invalid, Fault::invalid_sample);
    }
    std::array<formal_eskf::px4::ImuSource, 4> const changed_sources{
        {{18, 29, 2, 3}, {17, 30, 2, 3}, {17, 29, 3, 3}, {17, 29, 2, 4}}};
    for (auto const & source : changed_sources)
    {
        invalid = good;
        invalid.source = source;
        add(invalid, Fault::discontinuity);
    }
    for (auto const & entry : cases)
    {
        History history;
        test.expect(history.configure({}) && history.seed(seed, seed.time_us), "admission", "setup");
        test.expect(!history.push(entry.input, entry.now_us) && history.fault() == entry.fault &&
                        history.latest_time_us() == seed.time_us && history.fusion_time_us() == seed.time_us &&
                        history.entries() == 0 && history.delayed() == nullptr,
                    "admission", "invalid arrival latches its cause without committing payload or epochs");
        test.expect(!history.push(good, good.time_us) && !history.complete(seed.time_us) && !history.configure({}) &&
                        history.fault() == entry.fault,
                    "admission", "fault cannot clear through a later good sample, completion or configuration");
        history.reset();
        test.expect(history.seed(seed, seed.time_us) && history.push(good, good.time_us), "admission",
                    "explicit restart admits a new session");
    }
}

void lost_history_and_completion(Context & test)
{
    for (bool complete_first : {false, true})
    {
        History history;
        auto input = sample(1000000);
        test.expect(history.configure({}) && history.seed(input, input.time_us), "loss", "setup");
        for (std::size_t i = 0; i < history.configuration().imu_length; ++i)
        {
            input = sample(input.time_us + 4000);
            test.expect(history.push(input, input.time_us), "loss", "fill full real history");
        }
        auto const * delayed = history.delayed();
        test.expect(delayed != nullptr, "loss", "ready interval");
        if (delayed == nullptr)
        {
            return;
        }
        auto const epoch = delayed->time_us;
        if (complete_first)
        {
            test.expect(history.complete(epoch), "completion", "first success advances fusion");
            test.expect(!history.complete(epoch) && history.fault() == Fault::invalid_completion &&
                            history.fusion_time_us() == epoch,
                        "completion", "repeated completion cannot repeat prediction");
        }
        else
        {
            auto const latest = history.latest_time_us();
            auto next = sample(latest + 4000);
            test.expect(!history.push(next, next.time_us) && history.fault() == Fault::history_loss &&
                            history.latest_time_us() == latest && history.entries() == 54,
                        "loss", "unprocessed oldest record cannot be overwritten");
        }
    }
    History history;
    test.expect(history.configure({}) && history.seed(sample(1000000), 1000000), "completion", "setup");
    test.expect(!history.complete(1004000) && history.fault() == Fault::invalid_completion &&
                    history.fusion_time_us() == 1000000,
                "completion", "future completion during warmup cannot move the filter cursor");
}

void lag_bounds(Context & test)
{
    for (std::uint32_t second_gap : {46000U, 46001U})
    {
        History history;
        auto input = sample(1000000);
        test.expect(history.configure({}) && history.seed(input, input.time_us), "lag", "setup");
        for (std::size_t tick = 0; tick < 54; ++tick)
        {
            // After the first record, two gaps enlarge the retained span to exactly 300 ms or 300 ms + 1 us.
            auto const elapsed = tick == 1 ? 50000U : tick == 2 ? second_gap : 4000U;
            input = sample(input.time_us + elapsed);
            auto const previous = history.latest_time_us();
            bool const admitted = history.push(input, input.time_us);
            bool const too_old = tick == 53 && second_gap == 46001;
            test.expect(admitted != too_old, "lag", "check prospective span before the last warmup push");
            if (too_old)
            {
                test.expect(history.fault() == Fault::lag_out_of_range && history.entries() == 53 &&
                                history.latest_time_us() == previous && history.delayed() == nullptr,
                            "lag", "excess lag faults without skipping or overwriting pending intervals");
            }
        }
        if (second_gap == 46000)
        {
            auto const * delayed = history.delayed();
            test.expect(delayed != nullptr && input.time_us - delayed->time_us == 300000, "lag",
                        "inclusive maximum retained lag is usable");
        }
    }
}
void rollover_lag(Context & test)
{
    History history;
    auto input = sample(1000000);
    test.expect(history.configure({}) && history.seed(input, input.time_us), "rollover", "setup");
    for (std::size_t tick = 0; tick < 3 * 54; ++tick)
    {
        input = sample(input.time_us + 4000);
        test.expect(history.push(input, input.time_us), "rollover", "advance through several wraps");
        if (auto const * interval = history.delayed())
        {
            test.expect(history.complete(interval->time_us), "rollover", "complete each ready interval");
        }
    }
    input = sample(input.time_us + 50000);
    test.expect(history.push(input, input.time_us), "rollover", "one gap leaves 258 ms retained span");
    auto const * interval = history.delayed();
    test.expect(interval != nullptr && input.time_us - interval->time_us == 258000, "rollover",
                "measure span against the next oldest slot after rollover");
    if (interval != nullptr)
    {
        test.expect(history.complete(interval->time_us), "rollover", "complete before the second gap");
    }
    auto const last_completed = history.fusion_time_us();
    auto const last_admitted = history.latest_time_us();
    input = sample(input.time_us + 50000);
    test.expect(!history.push(input, input.time_us) && history.fault() == Fault::lag_out_of_range &&
                    history.fusion_time_us() == last_completed && history.latest_time_us() == last_admitted,
                "rollover", "second individually permitted gap exceeds total lag without advancing either cursor");
    history.reset();
    test.expect(history.seed(input, input.time_us), "completion", "restart after populated-history fault");
    auto const seed_time = input.time_us;
    for (std::size_t tick = 0; tick < 54; ++tick)
    {
        input = sample(input.time_us + 4000);
        test.expect(history.push(input, input.time_us), "completion", "new warmup");
    }
    interval = history.delayed();
    test.expect(interval != nullptr, "completion", "new ready interval");
    if (interval != nullptr)
    {
        test.expect(!history.complete(interval->time_us + 1) && history.fault() == Fault::invalid_completion &&
                        history.fusion_time_us() == seed_time,
                    "completion", "ready history cannot acknowledge a different endpoint");
    }
}
} // namespace

int main()
{
    formal_eskf::test::configure_backend_test();
    Context test;
    configuration(test);
    startup(test);
    ordered_prediction(test, {}, 1000000);
    ordered_prediction(test, {4000, 239, 0, 0, 0}, 1000000);
    ordered_prediction(test, {}, std::numeric_limits<std::uint64_t>::max() - 3 * 54 * 4000);
    gap_segments(test);
    admission_failures(test);
    lost_history_and_completion(test);
    lag_bounds(test);
    rollover_lag(test);
    std::cout << "IMU history bytes: " << sizeof(History) << '\n';
    return test.failures() == 0 ? 0 : 1;
}
