// SPDX-License-Identifier: BSD-3-Clause

// Complete initialization caller, preserving startup waiting and partial-failure
// paths. Opaque payload labels check provenance; numeric correctness is supplied
// by the component contracts and is not asserted by this control-flow binding.
#include <cstdint>
#include <formal_eskf/status.hpp>

extern bool nondet_bool();
extern std::uint64_t nondet_time();
extern unsigned nondet_value();
extern std::uint8_t nondet_mask();
extern void __ESBMC_assume(bool);
extern void __ESBMC_assert(bool, char const *);
#define PX4_ERR(...) ((void)0)
#define PX4_INFO(...) ((void)0)

namespace
{
#include "module_constants.inc"
enum Event : unsigned
{
    collect,
    ready,
    height_origin,
    attitude,
    navigation,
    magnetic_alignment,
    bias,
    gps,
    health,
    height,
    imu_seed,
    observation_seed,
    output_seed,
    event_count
};
struct Trace
{
    bool success[event_count]{};
    unsigned seen{}, last{};
    void record(Event event)
    {
        __ESBMC_assert(seen == 0 || unsigned(event) > last, "Initialization preserves correction and seed order");
        seen |= 1U << event;
        last = event;
    }
    bool has(Event event) const { return (seen & (1U << event)) != 0; }
} trace;
struct Input
{
    std::uint64_t time_us{};
    std::uint8_t accelerometer_clipping{};
};
struct Raw
{
    unsigned stamp{};
};
struct Observation
{
    Raw sample;
};
std::uint64_t clock_time;
std::uint64_t hrt_absolute_time() { return clock_time; }
std::uint64_t epoch;
unsigned state_identity;
unsigned altitude_identity;
Input const * input;
unsigned gnss_stamp, magnetic_stamp, barometer_stamp;
struct Height
{
    formal_eskf::Status initialize(std::uint64_t time, unsigned altitude)
    {
        trace.record(height_origin);
        __ESBMC_assert(time == epoch && altitude == altitude_identity,
                       "Height origin uses the alignment epoch and fix");
        return trace.success[height_origin] ? formal_eskf::Status::success : formal_eskf::Status::domain_error;
    }
};
struct History
{
    bool seed(Input const & sample, std::uint64_t now)
    {
        trace.record(imu_seed);
        __ESBMC_assert(&sample == input && now == clock_time, "IMU seed retains the actual alignment sample");
        return trace.success[imu_seed];
    }
};
struct Observations
{
    struct Seed
    {
        std::uint64_t context{};
        unsigned gnss{}, magnetic{}, barometer{};
    };
    bool seed(Seed const & sample)
    {
        trace.record(observation_seed);
        __ESBMC_assert(sample.context == epoch && sample.gnss == gnss_stamp && sample.magnetic == magnetic_stamp &&
                           sample.barometer == barometer_stamp && trace.has(imu_seed) && trace.success[imu_seed],
                       "Observation seed transfers alignment epoch and all consumed source watermarks");
        return trace.success[observation_seed];
    }
};
struct Output
{
    unsigned const * state{};
    bool seed(unsigned const & candidate, std::uint64_t time)
    {
        trace.record(output_seed);
        __ESBMC_assert(&candidate == state && candidate == state_identity && time == epoch && trace.has(height) &&
                           trace.has(observation_seed) && trace.success[observation_seed],
                       "Output seeds the final corrected main state after both histories succeed");
        return trace.success[output_seed];
    }
};
struct Flow
{
    using ImuInput = Input;
    using imu_type = unsigned;
    using observation_history_type = Observations;
    bool m_initialized{}, m_fault{};
    bool attitude_fault{};
    unsigned m_errors{}, callee_errors{}, m_state{};
    std::uint64_t m_time_us{};
    Height m_height_aiding;
    History m_imu_history;
    Observations m_observations;
    Output m_output;
    struct Gps
    {
        unsigned altitude_msl_m{};
    } m_gps;
    struct Mag
    {
        std::uint64_t timestamp_sample{};
        unsigned stamp{};
    } m_mag;
    struct Gnss
    {
        Observation input;
    } m_gnss;
    Observation m_barometer;
    struct Source
    {
        unsigned reference_id{};
    };
    struct Initialization
    {
        Source magnetic_source;
    } m_initialization;
    struct MagneticModel
    {
        unsigned id{};
        unsigned reference_id() const { return id; }
    } m_magnetic_model;

    void failure(bool success)
    {
        if (!success)
        {
            m_fault = true;
            m_errors = nondet_value(); // Preserve opaque callee diagnostic effects.
        }
        callee_errors = m_errors;
    }

    bool collect_initialization_samples(unsigned rates)
    {
        trace.record(collect);
        __ESBMC_assert(rates == 1, "Initialization consumes the admitted mean rates");
        return trace.success[collect];
    }
    bool initialization_ready()
    {
        trace.record(ready);
        return trace.success[ready];
    }
    bool initialize_attitude(double & yaw)
    {
        trace.record(attitude);
        yaw = 0.; // Pure opaque diagnostic payload, unused in control flow.
        failure(trace.success[attitude] || !attitude_fault);
        return trace.success[attitude];
    }
    void initialize_navigation(unsigned rates)
    {
        trace.record(navigation);
        __ESBMC_assert(rates == 1 && !m_initialized, "Navigation initialization precedes initialized state");
    }
    void initialize_magnetic_aiding(Source const & source, std::uint64_t time)
    {
        trace.record(magnetic_alignment);
        __ESBMC_assert(&source == &m_initialization.magnetic_source && source.reference_id == m_magnetic_model.id &&
                           time == m_mag.timestamp_sample,
                       "Magnetic startup transfers model identity and the actual sample watermark");
    }
    bool update_imu_bias(Input const & sample, unsigned rates)
    {
        trace.record(bias);
        __ESBMC_assert(&sample == input && rates == 1 && m_initialized, "Startup bias update follows alignment");
        failure(trace.success[bias]);
        return trace.success[bias];
    }
    void fuse_gps(unsigned rates)
    {
        trace.record(gps);
        __ESBMC_assert(rates == 1, "Startup GNSS correction uses the alignment IMU rates");
        failure(trace.success[gps]);
        m_state = nondet_value();
        state_identity = m_state;
    }
    bool update_imu_health(std::uint8_t clipping)
    {
        trace.record(health);
        __ESBMC_assert(clipping == input->accelerometer_clipping, "Startup health uses alignment clipping metadata");
        failure(trace.success[health]);
        return trace.success[health];
    }
    void fuse_height()
    {
        trace.record(height);
        failure(trace.success[height]);
        m_state = nondet_value();
        state_identity = m_state;
    }
    std::uint64_t startup_context(std::uint64_t time, std::uint64_t age)
    {
        __ESBMC_assert(time == epoch && age == profile::gnss::context_maximum_age_us,
                       "Seed context is evaluated at the alignment epoch");
        return time;
    }
    static Raw decode_magnetic(Mag sample) { return {sample.stamp}; }
    bool check_timing(bool succeeded);
    void initialize(Input const & sample, unsigned const & imu);
};

#include "module_initialization.inc"
} // namespace

extern "C" void verify_module_initialization()
{
    Flow flow;
    flow.m_errors = nondet_value();
    flow.callee_errors = flow.m_errors;
    Input sample;
    unsigned const rates = 1;
    sample.time_us = nondet_time();
    sample.accelerometer_clipping = nondet_mask();
    clock_time = nondet_time();
    __ESBMC_assume(sample.time_us != 0 && sample.time_us <= clock_time);
    epoch = sample.time_us;
    input = &sample;
    flow.m_time_us = epoch;
    flow.m_state = nondet_value();
    flow.m_gps.altitude_msl_m = nondet_value();
    altitude_identity = flow.m_gps.altitude_msl_m;
    flow.m_gnss.input.sample.stamp = gnss_stamp = nondet_value();
    flow.m_mag.stamp = magnetic_stamp = nondet_value();
    flow.m_barometer.sample.stamp = barometer_stamp = nondet_value();
    flow.m_mag.timestamp_sample = nondet_time();
    flow.m_magnetic_model.id = nondet_value();
    flow.m_output.state = &flow.m_state;
    flow.attitude_fault = nondet_bool();
    for (unsigned stage = 0; stage < event_count; ++stage)
    {
        trace.success[stage] = nondet_bool();
    }
    flow.initialize(sample, rates);
    bool const origin = trace.success[collect] && trace.success[ready];
    bool const oriented = origin && trace.success[height_origin];
    bool const aligned = oriented && trace.success[attitude];
    bool const gps_attempt = aligned && trace.success[bias];
    bool const health_attempt = gps_attempt && trace.success[gps];
    bool const height_attempt = health_attempt && trace.success[health];
    bool const seed = height_attempt && trace.success[height];
    bool const observations = seed && trace.success[imu_seed];
    bool const output = observations && trace.success[observation_seed];
    __ESBMC_assert(
        trace.has(collect) && trace.has(ready) == trace.success[collect] && trace.has(height_origin) == origin &&
            trace.has(attitude) == oriented && trace.has(navigation) == aligned &&
            trace.has(magnetic_alignment) == aligned && trace.has(bias) == aligned && trace.has(gps) == gps_attempt &&
            trace.has(health) == health_attempt && trace.has(height) == height_attempt && trace.has(imu_seed) == seed &&
            trace.has(observation_seed) == observations && trace.has(output_seed) == output,
        "Startup waiting, correction failures and each partial seed failure preserve the exact call prefix");
    __ESBMC_assert(flow.m_initialized == aligned && flow.m_time_us == epoch,
                   "Initialized flag follows alignment; the common alignment epoch is never retimestamped");
    bool const timing_failed = seed && !(output && trace.success[output_seed]);
    __ESBMC_assert(flow.m_errors == flow.callee_errors + unsigned(timing_failed),
                   "Startup seed failure adds one timing error to the callee's diagnostic effects");
    bool const expected_fault =
        (oriented && !trace.success[attitude] && flow.attitude_fault) || (aligned && !seed) || timing_failed;
    __ESBMC_assert(flow.m_fault == expected_fault,
                   "Waiting for valid altitude/attitude is distinct from latched correction or history failure");
}
