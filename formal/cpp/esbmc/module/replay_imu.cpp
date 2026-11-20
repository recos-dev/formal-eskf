// SPDX-License-Identifier: BSD-3-Clause

// Exact host tick prefix with decoded-input and numerical callee contracts.
// Diagnostic string allocation is projected to the same exception transfer.
// Prior successful prediction is deliberately not rolled back on later failure.
#include <cstdint>
#include "replay_support.hpp"
#define optional formal_eskf_replay_optional
#include <formal_eskf/status.hpp>
#include "../algorithm_support.hpp"

extern bool nondet_bool();
extern unsigned nondet_unsigned();
extern double nondet_double();
extern int nondet_int();
extern std::uint64_t nondet_time();
extern void __ESBMC_assert(bool, char const *);
extern void __ESBMC_assume(bool);

namespace formal_eskf::runtime
{
struct ImuNoiseConditions
{
    bool bad_vertical_acceleration{};
    bool accelerometer_clipped[3]{};
};
} // namespace formal_eskf::runtime
namespace std
{
// Pure STL value contract, not a target-library proof.
double formal_eskf_proof_clamp(double value, double low, double high)
{
    __ESBMC_assert(!(high < low), "Replay clamp receives ordered bounds");
    if (value < low)
    {
        return low;
    }
    if (high < value)
    {
        return high;
    }
    return value;
}
} // namespace std

namespace
{
using Status = formal_eskf::Status;
struct ReplayFailure
{
};
enum Stage : unsigned
{
    hold,
    reset_observation,
    reset_heading,
    reset_health,
    bias,
    real,
    heading,
    stage_count
};
struct Trace
{
    bool success[stage_count]{};
    unsigned seen{}, last{};
    void record(Stage stage)
    {
        __ESBMC_assert(seen == 0 || unsigned(stage) > last, "Replay prediction stages execute once in source order");
        seen |= 1U << stage;
        last = stage;
    }
    bool has(Stage stage) const { return (seen & (1U << stage)) != 0; }
} trace;

struct Vector
{
};
Vector operator-(Vector, Vector) { return {}; }
struct State
{
    unsigned q_nb{};
    Vector b_a;
};
struct Imu
{
    unsigned identity{};
    Vector specific_force_b;
};
struct Flags
{
    int at_rest{}, accelerometer_clipping{};
} supplied_flags;
std::uint64_t supplied_time;
Flags read_imu_flags(unsigned, std::uint64_t epoch)
{
    __ESBMC_assert(epoch == supplied_time, "Replay flags belong to the current real IMU tick");
    return supplied_flags;
}
Imu imu_sample(std::uint64_t const *) { return {2}; }
// Decoded timestamp contract: the CSV parser rejects nonintegral/invalid input.
std::uint64_t timestamp(std::uint64_t value) { return value; }
#include "replay_status.inc"
struct BiasInhibition
{
    struct Conditions
    {
        bool bad_vertical_acceleration{}, at_rest{};
        int accelerometer_clipping{};
    };
};
struct GnssLifecycle
{
    struct VerticalAccelerationTimes
    {
        std::uint64_t last_update_time_us{}, last_good_time_us{};
    };
};
struct Health
{
    struct Value
    {
        std::uint64_t last_update_time_us{}, last_good_time_us{};
        bool bad_vertical_acceleration{};
    } value;
    Value const & state() const { return value; }
    void reset()
    {
        trace.record(reset_health);
        value = {};
    }
};
struct Learning
{
    struct Result
    {
        Status status{};
        unsigned bias_update{};
    };
    BiasInhibition::Conditions received;
    Result update(double dt, double accel_dt, State const &, Imu const & sample,
                  BiasInhibition::Conditions const & conditions)
    {
        trace.record(bias);
        __ESBMC_assert(dt == accel_dt && sample.identity == 2, "Bias learning uses current real rates and period");
        received = conditions;
        return {trace.success[bias] ? Status::success : Status::domain_error, 0};
    }
};
struct Accelerometer
{
    void reset() { trace.record(reset_observation); }
};
struct Heading
{
    void reset_acceleration() { trace.record(reset_heading); }
    Status update_acceleration(Vector, double)
    {
        trace.record(heading);
        return trace.success[heading] ? Status::success : Status::domain_error;
    }
};
} // namespace
namespace formal_eskf::so3
{
Vector rotate(unsigned, Vector) { return {}; }
} // namespace formal_eskf::so3
namespace
{
std::uint64_t old_time, real_period;
unsigned predictions_completed;
int predict_interval(State &, unsigned &, Imu const & sample, std::uint64_t period, std::uint64_t epoch,
                     unsigned const &, std::optional<formal_eskf::runtime::ImuNoiseConditions> const & conditions,
                     unsigned &)
{
    Stage stage = real;
    if (sample.identity == 1)
    {
        stage = hold;
    }
    trace.record(stage);
    if (stage == hold)
    {
        __ESBMC_assert(
            period == supplied_time - old_time - real_period && epoch == supplied_time - real_period &&
                !conditions.has_value(),
            "Gap prediction holds previous rates only over the uncovered interval without fresh noise evidence");
    }
    else
    {
        __ESBMC_assert(sample.identity == 2 && period == real_period && epoch == supplied_time &&
                           conditions.has_value() == (supplied_flags.accelerometer_clipping >= 0),
                       "Real prediction uses the actual endpoint, integration period and known clipping availability");
        if (conditions)
        {
            for (unsigned axis = 0; axis < 3; ++axis)
            {
                __ESBMC_assert(conditions->accelerometer_clipped[axis] ==
                                   ((supplied_flags.accelerometer_clipping & (1 << axis)) != 0),
                               "Noise selection receives every real clipping bit");
            }
        }
    }
    if (!trace.success[stage])
    {
        throw ReplayFailure{};
    }
    ++predictions_completed;
    return 0;
}
struct Flow
{
    std::uint64_t imu_row[2]{};
    std::uint64_t time{}, gap_us{};
    unsigned gaps{}, predictions{}, vertical_bias_inhibited{};
    State state;
    unsigned covariance{}, settings{}, flags{}, prediction_statistics{}, vertical_cache{};
    Imu previous_imu{1};
    Accelerometer accelerometer;
    Heading magnetic_heading;
    Health vertical_health;
    Learning bias_learning;
    std::optional<unsigned> quality_parameters;
    double average_interval{}, nominal_interval{0.004};
    bool has_magnetic_input{}, mag_enabled{};
    void step()
    {
#define max formal_eskf_proof_max
#define clamp formal_eskf_proof_clamp
#include "replay_imu.inc"
#undef clamp
#undef max
    }
};
} // namespace

extern "C" void verify_replay_imu()
{
    Flow flow;
    flow.time = old_time = nondet_time();
    flow.nominal_interval = static_cast<double>(nondet_time()) * 1e-6;
    flow.average_interval = nondet_double();
    flow.imu_row[0] = supplied_time = nondet_time();
    flow.imu_row[1] = real_period = nondet_time();
    __ESBMC_assume(supplied_time != 0 && real_period != 0);
    supplied_flags = {nondet_int(), nondet_int()};
    flow.gaps = nondet_unsigned();
    flow.predictions = nondet_unsigned();
    flow.gap_us = nondet_time();
    flow.vertical_health.value = {nondet_time(), nondet_time(), nondet_bool()};
    bool const prior_bad = flow.vertical_health.value.bad_vertical_acceleration;
    auto const prior_update = flow.vertical_health.value.last_update_time_us;
    if (nondet_bool())
    {
        flow.quality_parameters = 1;
    }
    flow.has_magnetic_input = nondet_bool();
    flow.mag_enabled = nondet_bool();
    for (unsigned i = 0; i < stage_count; ++i)
    {
        trace.success[i] = nondet_bool();
    }
    auto const gaps = flow.gaps;
    auto const predictions = flow.predictions;
    auto const gap_us = flow.gap_us;
    bool failed = false;
    try
    {
        flow.step();
    }
    catch (...)
    {
        failed = true;
    }
    bool const admitted =
        supplied_time > old_time && supplied_time - old_time <= 50000 && real_period <= supplied_time - old_time;
    bool const held = admitted && real_period != supplied_time - old_time;
    bool const prepared = admitted && (!held || trace.success[hold]);
    bool const predicted = prepared && trace.success[bias];
    bool const real_done = predicted && trace.success[real];
    bool const headed = real_done && flow.has_magnetic_input && flow.mag_enabled;
    bool const complete = real_done && (!headed || trace.success[heading]);
    __ESBMC_assert(failed == !complete && trace.has(hold) == held && trace.has(bias) == prepared &&
                       trace.has(real) == predicted && trace.has(heading) == headed,
                   "Every invalid interval or failed callee stops the replay tick at its first failure");
    __ESBMC_assert(flow.time == (complete ? supplied_time : old_time) &&
                       flow.previous_imu.identity == (complete ? 2U : 1U) &&
                       flow.predictions == predictions + unsigned(complete),
                   "Replay commits the endpoint and previous rates only after prediction and heading status succeed");
    __ESBMC_assert(trace.has(reset_observation) == (held && prepared) &&
                       trace.has(reset_heading) == (held && prepared) &&
                       trace.has(reset_health) == (prepared && (held || supplied_flags.accelerometer_clipping < 0)),
                   "Held intervals reset observation histories; unknown clipping also breaks health continuity");
    __ESBMC_assert(flow.gaps == gaps + unsigned(held && prepared) &&
                       flow.gap_us == gap_us + (held && prepared ? supplied_time - old_time - real_period : 0) &&
                       predictions_completed == unsigned(held && prepared) + unsigned(real_done),
                   "Earlier successful held or real predictions remain committed when a later stage throws");
    if (prepared)
    {
        __ESBMC_assert(flow.bias_learning.received.bad_vertical_acceleration ==
                               (flow.quality_parameters.has_value() && prior_update != 0 && prior_bad && !held &&
                                supplied_flags.accelerometer_clipping >= 0) &&
                           flow.bias_learning.received.at_rest == (supplied_flags.at_rest == 1),
                       "Bias learning uses the preceding health verdict only while continuity remains valid");
    }
}
