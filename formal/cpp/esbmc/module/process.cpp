// SPDX-License-Identifier: BSD-3-Clause

// Actual startup admission and immediate-processing bodies. Non-temporal data
// is projected to producer identities; every finite/non-finite and callee
// success/failure branch remains possible. This does not prove sensor decoding.
#include <cstdint>

extern bool nondet_bool();
extern unsigned nondet_value();
extern std::uint64_t nondet_time();
extern std::uint32_t nondet_period();
extern std::uint8_t nondet_mask();
extern void __ESBMC_assume(bool);
extern void __ESBMC_assert(bool, char const *);
#define PX4_ERR(...) ((void)0)

#include "module_clipping.inc"
namespace
{
#include "module_constants.inc"
struct Vector
{
    bool finite{};
};
struct Rates
{
    Vector specific_force_b{}, angular_rate_b{};
};
struct Input
{
    std::uint64_t time_us{};
    std::uint32_t angular_period_us{}, acceleration_period_us{};
    std::uint8_t accelerometer_clipping{}, gyroscope_clipping{};
    Rates rates;
    Rates mean_rates() const { return rates; }
};
struct Observation
{
    std::uint64_t time_us{}, arrival_time_us{}, residual_delay_us{};
    std::uint32_t half_period_us{};
    unsigned sample{};
};
std::uint64_t clock_time;
std::uint64_t admission_clock;
unsigned clock_reads;
std::uint64_t hrt_absolute_time()
{
    auto const next = nondet_time();
    __ESBMC_assume(next >= clock_time); // External monotonic-clock premise.
    clock_time = next;
    if (clock_reads == 0)
    {
        admission_clock = next;
    }
    ++clock_reads;
    return next;
}
struct History
{
    struct Interval
    {
        Input sample;
    } newest;
    bool pushed{}, success{};
    Input const * source{};
    bool push(Input const & sample, std::uint64_t now)
    {
        __ESBMC_assert(!pushed && &sample == source && now == clock_time, "History admission uses this current sample");
        pushed = true;
        if (success)
        {
            newest.sample = sample;
        }
        return success;
    }
    Interval const * latest() const
    {
        __ESBMC_assert(pushed && success, "Newest interval is borrowed only after accepted insertion");
        return &newest;
    }
};
struct Observations
{
    History * history{};
    bool recorded{}, success{};
    bool record_imu(History::Interval const & interval, unsigned land, unsigned vehicle)
    {
        __ESBMC_assert(history->success && &interval == &history->newest && land == 1 && vehicle == 2,
                       "Current context attaches to the just-admitted interval");
        recorded = true;
        return success;
    }
};
struct Output
{
    Observations * observations{};
    bool predicted{}, success{};
    bool predict(History::Interval const & interval)
    {
        __ESBMC_assert(observations->recorded && observations->success && &interval == &observations->history->newest,
                       "Current output propagation follows successful history/context admission");
        predicted = true;
        return success;
    }
};
} // namespace
namespace formal_eskf::linalg
{
bool all_finite(Vector vector) { return vector.finite; }
} // namespace formal_eskf::linalg

namespace
{
struct Flow
{
    using ImuInput = Input;
    bool m_fault{}, m_initialized{};
    bool reset{}, quality_called{}, quality_ok{}, initialized{}, healthy{}, validity_called{}, attitude_published{};
    unsigned m_errors{}, m_rejections{}, m_bias_learning_result{1};
    std::uint64_t m_time_us{}, m_imu_context{};
    struct Wire
    {
        std::uint64_t timestamp_sample{};
        unsigned identity{};
    } m_gps{0, 3}, m_baro{0, 4};
    struct Gnss
    {
        Observation input;
        std::uint64_t context{};
    } m_gnss;
    Observation m_barometer;
    History m_imu_history;
    Observations m_observations;
    Output m_output;
    std::uint64_t startup_context(std::uint64_t time, std::uint64_t) const { return time; }
    unsigned land_state() const { return 1; }
    unsigned vehicle_state() const { return 2; }
    static unsigned decode_gnss(Wire wire) { return wire.identity; }
    static unsigned decode_barometer(Wire wire) { return wire.identity; }
    void reset_imu_observations() { reset = true; }
    void update_gps_quality()
    {
        __ESBMC_assert(m_imu_context == m_time_us && m_gnss.context == m_gps.timestamp_sample &&
                           m_gnss.input.time_us == m_gps.timestamp_sample && m_gnss.input.sample == 3 &&
                           m_barometer.time_us == m_baro.timestamp_sample && m_barometer.sample == 4,
                       "Startup quality receives raw selected inputs and contexts at their own epochs");
        quality_called = true;
        m_fault = !quality_ok;
    }
    void initialize(Input const & sample, Rates const & rates)
    {
        __ESBMC_assert(quality_called && !m_fault && m_time_us == sample.time_us && rates.specific_force_b.finite &&
                           rates.angular_rate_b.finite,
                       "Initialization follows accepted finite startup IMU and quality evaluation");
        initialized = true;
        // Full initialization is a separate caller proof. It may set the
        // initialized flag and latch failure after a partial startup sequence.
        m_initialized = nondet_bool();
        m_fault = nondet_bool();
    }
    struct Validity
    {
        bool healthy{};
    };
    Validity output_validity(std::uint64_t now)
    {
        __ESBMC_assert(m_output.predicted && m_output.success && now == clock_time && !m_fault,
                       "Fast attitude eligibility is checked after successful current propagation");
        validity_called = true;
        return {healthy};
    }
    void publish_attitude(std::uint64_t now)
    {
        __ESBMC_assert(validity_called && healthy && now == clock_time, "Fast attitude requires a healthy output gate");
        attitude_published = true;
    }
    bool check_timing(bool succeeded);
    bool accept_imu(Input const & sample);
    void process_imu(Input const & sample);
};

#include "module_process.inc"
} // namespace

extern "C" void verify_module_process()
{
    Flow flow;
    flow.m_errors = nondet_value();
    flow.m_rejections = nondet_value();
    flow.m_bias_learning_result = nondet_value();
    auto const errors = flow.m_errors;
    auto const rejections = flow.m_rejections;
    auto const learning = flow.m_bias_learning_result;
    Input sample;
    sample.time_us = nondet_time();
    sample.angular_period_us = nondet_period();
    sample.acceleration_period_us = nondet_period();
    sample.accelerometer_clipping = nondet_mask();
    sample.gyroscope_clipping = nondet_mask();
    sample.rates.specific_force_b.finite = nondet_bool();
    sample.rates.angular_rate_b.finite = nondet_bool();
    clock_time = nondet_time();
    flow.m_time_us = nondet_time();
    flow.m_gps.timestamp_sample = nondet_time();
    flow.m_baro.timestamp_sample = nondet_time();
    flow.m_initialized = nondet_bool();
    flow.m_fault = nondet_bool();
    flow.quality_ok = nondet_bool();
    flow.healthy = nondet_bool();
    flow.m_imu_history.source = &sample;
    flow.m_imu_history.success = nondet_bool();
    flow.m_observations.history = &flow.m_imu_history;
    flow.m_observations.success = nondet_bool();
    flow.m_output.observations = &flow.m_observations;
    flow.m_output.success = nondet_bool();
    bool const fault = flow.m_fault;
    bool const initialized = flow.m_initialized;
    auto const epoch = flow.m_time_us;
    flow.process_imu(sample);
    bool const running = !fault && initialized;
    bool const recorded = running && flow.m_imu_history.success;
    bool const predicted = recorded && flow.m_observations.success;
    bool const eligible = predicted && flow.m_output.success;
    bool const periods = sample.angular_period_us >= profile::history::minimum_period_us &&
                         sample.angular_period_us <= profile::history::maximum_period_us &&
                         sample.acceleration_period_us >= profile::history::minimum_period_us &&
                         sample.acceleration_period_us <= profile::history::maximum_period_us;
    bool const admitted = sample.time_us > epoch && sample.time_us <= admission_clock &&
                          admission_clock - sample.time_us <= profile::output::maximum_age_us && periods &&
                          sample.accelerometer_clipping == 0 && sample.gyroscope_clipping == 0;
    bool const startup = !fault && !initialized;
    bool const finite = sample.rates.specific_force_b.finite && sample.rates.angular_rate_b.finite;
    bool const accepted = startup && admitted && finite;
    __ESBMC_assert(
        flow.m_imu_history.pushed == running && flow.m_observations.recorded == recorded &&
            flow.m_output.predicted == predicted && flow.validity_called == eligible &&
            flow.attitude_published == (eligible && flow.healthy),
        "Immediate path has one ordered admission, context, prediction and conditional attitude publication");
    __ESBMC_assert(flow.quality_called == accepted && flow.initialized == (accepted && flow.quality_ok) &&
                       flow.m_time_us == (accepted ? sample.time_us : epoch),
                   "Only accepted finite startup IMU advances the initial horizon; running IMU preserves delayed time");
    __ESBMC_assert(flow.reset == (startup && !admitted) &&
                       flow.m_bias_learning_result == (startup && !admitted ? 0U : learning) &&
                       flow.m_rejections == rejections + unsigned(startup && !(admitted && finite)),
                   "Startup rejects invalid timing/clipping or rates without consuming the horizon");
    if (running)
    {
        __ESBMC_assert(flow.m_fault == !eligible && flow.m_errors == errors + unsigned(!eligible),
                       "Any immediate history/context/output failure latches exactly one timing fault");
    }
    if (fault)
    {
        __ESBMC_assert(flow.m_fault && flow.m_initialized == initialized && flow.m_errors == errors,
                       "An existing fault leaves processing idle");
    }
}
