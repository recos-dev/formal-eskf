// SPDX-License-Identifier: BSD-3-Clause

// Temporal/control projection of the exact predict_and_fuse body. Ghost labels
// identify numeric producers; they are not replacement arithmetic. Numeric
// return/status contracts overapproximate every successful and failing result.
#include <cstdint>
#include <formal_eskf/status.hpp>

extern bool nondet_bool();
extern std::uint64_t nondet_time();
extern std::uint32_t nondet_period();
extern unsigned nondet_value();
extern std::uint8_t nondet_mask();
extern void __ESBMC_assume(bool);
extern void __ESBMC_assert(bool, char const *);
#define PX4_ERR(...) ((void)0)

namespace
{
#include "module_constants.inc"
using status_type = formal_eskf::Status;

enum Event : unsigned
{
    context,
    clear_imu,
    clear_acceleration,
    held_prediction,
    bias,
    real_prediction,
    begin,
    select,
    heading_acceleration,
    gps,
    health,
    ground,
    height,
    gravity,
    magnetic,
    event_count
};
struct Trace
{
    unsigned seen{};
    unsigned last{};
    bool success[event_count]{};
    void record(Event event)
    {
        __ESBMC_assert(seen == 0 || unsigned(event) > last, "Delayed stages execute once in prescribed order");
        seen |= 1U << event;
        last = event;
    }
    bool has(Event event) const { return (seen & (1U << event)) != 0; }
} trace;

struct Vector
{
    unsigned origin{};
};
Vector operator-(Vector, Vector) { return {}; }
struct State
{
    unsigned q_nb{};
    Vector b_a{};
};
struct Rates
{
    bool held{};
    Vector specific_force_b{};
};
struct Input
{
    std::uint64_t time_us{};
    std::uint32_t angular_period_us{}, acceleration_period_us{};
    std::uint8_t accelerometer_clipping{};
    Rates mean_rates() const { return {}; }
    double angular_interval_s() const { return double(angular_period_us) * profile::seconds_per_microsecond; }
    double acceleration_interval_s() const { return double(acceleration_period_us) * profile::seconds_per_microsecond; }
};
struct History
{
    struct Interval
    {
        Input sample;
        Rates held_rate{true};
        std::uint32_t hold_period_us{};
    };
};
History::Interval const * borrowed;
std::uint64_t * main_epoch;
State * main_state;
unsigned * covariance;
unsigned * parameters;
bool old_bad_acceleration;

status_type result(Event event)
{
    trace.record(event);
    return trace.success[event] ? status_type::success : status_type::out_of_range;
}
struct Observations
{
    unsigned prediction_context(History::Interval const & interval, std::uint64_t maximum_age)
    {
        trace.record(context);
        __ESBMC_assert(&interval == borrowed && maximum_age == profile::imu::rest_maximum_age_us,
                       "Prediction context belongs to the borrowed delayed interval");
        return 1;
    }
    bool begin_fusion(History::Interval const & interval)
    {
        trace.record(begin);
        __ESBMC_assert(&interval == borrowed && *main_epoch == interval.sample.time_us && trace.has(real_prediction),
                       "Observation horizon advances only after real prediction commits its endpoint");
        return trace.success[begin];
    }
};
struct Heading
{
    void reset_acceleration() { trace.record(clear_acceleration); }
    status_type update_acceleration(Vector, double interval)
    {
        __ESBMC_assert(interval == borrowed->sample.acceleration_interval_s(),
                       "Heading acceleration uses the delayed acceleration integration period");
        return result(heading_acceleration);
    }
};
struct VerticalHealth
{
    struct State
    {
        bool bad_vertical_acceleration{};
    } value;
    State const & state() const { return value; }
};
} // namespace

namespace formal_eskf::so3
{
Vector rotate(unsigned, Vector) { return {}; }
} // namespace formal_eskf::so3
namespace formal_eskf::px4
{
status_type predict_imu(State & state, unsigned & P, Rates const & rates, double dt, unsigned const & settings,
                        std::uint8_t clipping, bool bad_acceleration)
{
    __ESBMC_assert(&state == main_state && &P == covariance && &settings == parameters,
                   "Both prediction segments use the live main state, covariance and parameter owner");
    if (rates.held)
    {
        __ESBMC_assert(borrowed->hold_period_us != 0 &&
                           dt == borrowed->hold_period_us * profile::seconds_per_microsecond && clipping == 0 &&
                           !bad_acceleration,
                       "Synthetic segment uses held rates, hold duration and no invented clipping/health flags");
        return result(held_prediction);
    }
    __ESBMC_assert(dt == borrowed->sample.angular_interval_s() && clipping == borrowed->sample.accelerometer_clipping &&
                       bad_acceleration == old_bad_acceleration && trace.has(bias),
                   "Real segment uses current rates/period/clipping and preceding acceleration health");
    return result(real_prediction);
}
} // namespace formal_eskf::px4

namespace
{
struct Flow
{
    using imu_history_type = History;
    using status_type = formal_eskf::Status;
    bool m_fault{};
    std::uint64_t m_time_us{};
    unsigned m_errors{}, m_gaps{}, m_predictions{}, m_imu_context{};
    State m_state;
    unsigned m_covariance{}, m_parameters{};
    Observations m_observations;
    struct Magnetic
    {
        Heading heading;
    } m_magnetic;
    VerticalHealth m_vertical_health;

    bool check(status_type status);
    bool check_timing(bool succeeded);
    void predict_and_fuse(imu_history_type::Interval const & interval);
    void reset_imu_observations()
    {
        trace.record(clear_imu);
        // Production clears health before evaluating the real interval.
        m_vertical_health.value.bad_vertical_acceleration = false;
        old_bad_acceleration = false;
    }
    bool update_imu_bias(Input const & sample, Rates const & rates)
    {
        __ESBMC_assert(&sample == &borrowed->sample && !rates.held && m_imu_context == 1,
                       "Bias learning uses the delayed sample and its historical context");
        return check(result(bias));
    }
    void select_observations()
    {
        __ESBMC_assert(m_time_us == borrowed->sample.time_us, "Sensor selection uses the committed delayed epoch");
        check(result(select));
    }
    void fuse_gps(Rates const & rates)
    {
        __ESBMC_assert(!rates.held, "GNSS rotational compensation uses delayed real-interval rates");
        check(result(gps));
    }
    bool update_imu_health(std::uint8_t clipping)
    {
        __ESBMC_assert(clipping == borrowed->sample.accelerometer_clipping && trace.has(gps) && !trace.has(height),
                       "Health sees current GNSS evidence and preceding height evidence");
        return check(result(health));
    }
    void update_ground_effect() { trace.record(ground); }
    void fuse_height() { check(result(height)); }
    void fuse_gravity(Input const & sample, Rates const & rates)
    {
        __ESBMC_assert(&sample == &borrowed->sample && !rates.held, "Gravity sampling uses the delayed IMU interval");
        check(result(gravity));
    }
    void fuse_mag() { check(result(magnetic)); }
};

#include "module_prediction.inc"
} // namespace

extern "C" void verify_module_prediction()
{
    Flow flow;
    flow.m_predictions = nondet_value();
    flow.m_gaps = nondet_value();
    flow.m_errors = nondet_value();
    auto const predictions = flow.m_predictions;
    auto const gaps = flow.m_gaps;
    History::Interval interval;
    interval.sample.time_us = nondet_time();
    interval.sample.angular_period_us = nondet_period();
    interval.sample.acceleration_period_us = nondet_period();
    interval.sample.accelerometer_clipping = nondet_mask();
    interval.hold_period_us = nondet_period();
    flow.m_time_us = nondet_time();
    flow.m_vertical_health.value.bad_vertical_acceleration = nondet_bool();
    old_bad_acceleration = flow.m_vertical_health.value.bad_vertical_acceleration;
    borrowed = &interval;
    main_epoch = &flow.m_time_us;
    main_state = &flow.m_state;
    covariance = &flow.m_covariance;
    parameters = &flow.m_parameters;
    for (unsigned stage = 0; stage < event_count; ++stage)
    {
        trace.success[stage] = nondet_bool();
    }
    // ImuHistory supplies an admitted interval after the previous fusion epoch.
    __ESBMC_assume(flow.m_time_us < interval.sample.time_us);
    auto const previous_epoch = flow.m_time_us;
    flow.predict_and_fuse(interval);

    bool const held = interval.hold_period_us != 0;
    bool const biased = !held || trace.success[held_prediction];
    bool const predicted = biased && trace.success[bias];
    bool const committed = predicted && trace.success[real_prediction];
    bool const selected = committed && trace.success[begin];
    bool const headed = selected && trace.success[select];
    bool const gps_attempt = headed && trace.success[heading_acceleration];
    bool const health_attempt = gps_attempt && trace.success[gps];
    bool const height_attempt = health_attempt && trace.success[health];
    bool const gravity_attempt = height_attempt && trace.success[height];
    bool const magnetic_attempt = gravity_attempt && trace.success[gravity];
    __ESBMC_assert(trace.has(context) && trace.has(clear_imu) == held && trace.has(clear_acceleration) == held &&
                       trace.has(held_prediction) == held && trace.has(bias) == biased &&
                       trace.has(real_prediction) == predicted && trace.has(begin) == committed &&
                       trace.has(select) == selected && trace.has(heading_acceleration) == headed &&
                       trace.has(gps) == gps_attempt && trace.has(health) == health_attempt &&
                       trace.has(ground) == height_attempt && trace.has(height) == height_attempt &&
                       trace.has(gravity) == gravity_attempt && trace.has(magnetic) == magnetic_attempt,
                   "All delayed stages follow the exact successful prefix, including every failure exit");
    __ESBMC_assert(flow.m_time_us == (committed ? interval.sample.time_us : previous_epoch) &&
                       flow.m_predictions == predictions + unsigned(committed) && flow.m_gaps == gaps + unsigned(held),
                   "Only accepted real prediction advances the main epoch and prediction count");
    __ESBMC_assert(flow.m_fault == !(magnetic_attempt && trace.success[magnetic]),
                   "Any failed delayed stage is latched before caller feedback or publication");
}
