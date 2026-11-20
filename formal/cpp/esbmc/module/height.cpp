// SPDX-License-Identifier: BSD-3-Clause

// Actual HeightAiding call binding and notification/reporting transaction.
// The owner result remains arbitrary: its numerical/atomicity properties are
// supplied by the accepted HeightAiding producer proofs, not assumed as success.
#include "../contracts.hpp"
#include "../height_support.hpp"
#define any_of formal_eskf_proof_any_of
#include "../../../../integrations/px4/module/HeightAiding.hpp"
#undef any_of

extern bool nondet_bool();
extern unsigned nondet_value();
extern std::uint64_t nondet_time();
extern std::uint8_t nondet_byte();
extern double nondet_double();
#define PX4_ERR(...) ((void)0)

namespace
{
namespace profile = formal_eskf::px4::profile;
using Backend = contract_proof::Linalg;
using Actual = formal_eskf::px4::HeightAiding<Backend>;
using Vector = formal_eskf::linalg::Vector<Backend, 3>;
using Quaternion = formal_eskf::so3::UnitQuaternion<Backend>;
using Permissions = formal_eskf::configuration::Ins::BiasUpdate;
using Status = formal_eskf::Status;
static_assert(sizeof(Backend::value_type) == sizeof(double));
Vector vector3(double x, double y, double z) { return Vector::from_row_major({x, y, z}); }
double square(double x) { return x * x; }
struct Source
{
    std::uint32_t device_id{};
    std::uint8_t calibration_count{};
};
struct Raw
{
    struct Stamp
    {
        Source source;
    } stamp;
    double altitude_m{}, altitude_msl_m{}, vertical_accuracy_m{};
};
struct Observation
{
    std::uint64_t time_us{}, generation{};
    Raw sample;
};
struct Health
{
    struct State
    {
        bool bad_vertical_acceleration{};
        std::uint64_t last_update_time_us{}, last_good_time_us{};
    } value;
    State const & state() const { return value; }
};
struct Boundary
{
    Observation barometer, gps;
    std::uint64_t gnss_time{}, epoch{};
    double offset{};
    bool qualified{}, velocity_active{}, ground_effect{}, notification{}, called{}, notified{};
    Health::State health;
    unsigned * state{};
    unsigned * covariance{};
    Permissions const * permissions{};
    Actual::Result result;
} boundary;
struct Height
{
    using Source = Actual::Source;
    using Sample = Actual::Sample;
    using Acceleration = Actual::Acceleration;
    using Result = Actual::Result;
    struct Session
    {
        std::uint64_t fused_time{};
    } barometer, gnss;
    unsigned session_reads{};
    Session const & session(Source source)
    {
        ++session_reads;
        if (source == Source::barometer)
        {
            return barometer;
        }
        return gnss;
    }
    Result update(std::uint64_t now, Sample const & baro, Sample const & gps, bool velocity_active, unsigned & state,
                  unsigned & covariance, Permissions const & permissions, Acceleration const & acceleration,
                  bool ground_effect)
    {
        __ESBMC_assert(!boundary.called && now == boundary.epoch && &state == boundary.state &&
                           &covariance == boundary.covariance && &permissions == boundary.permissions &&
                           velocity_active == boundary.velocity_active && ground_effect == boundary.ground_effect,
                       "Height owner receives the delayed main owners, velocity authority and correction permissions");
        __ESBMC_assert(baro.time_us == boundary.barometer.time_us &&
                           baro.device_id == boundary.barometer.sample.stamp.source.device_id &&
                           baro.calibration_count == boundary.barometer.sample.stamp.source.calibration_count &&
                           baro.generation == boundary.barometer.generation && baro.qualified &&
                           contract_proof::same(baro.altitude, boundary.barometer.sample.altitude_m) &&
                           baro.variance == profile::height::barometer_variance,
                       "Barometer binding retains effective epoch, source/calibration/generation and altitude");
        if (boundary.gnss_time != 0)
        {
            __ESBMC_assert(gps.time_us == boundary.gnss_time && gps.calibration_count == 0 &&
                               gps.device_id == boundary.gps.sample.stamp.source.device_id &&
                               gps.generation == boundary.gps.generation && gps.qualified == boundary.qualified,
                           "GNSS height uses the consumed quality-event epoch and matching selected generation");
            __ESBMC_assert(contract_proof::same(gps.altitude, boundary.gps.sample.altitude_msl_m + boundary.offset),
                           "GNSS antenna altitude is translated to the IMU reference point with the original sign");
            auto const noise = std::formal_eskf_proof_max(boundary.gps.sample.vertical_accuracy_m,
                                                          profile::height::minimum_gnss_noise_m);
            __ESBMC_assert(contract_proof::same(gps.variance, noise * noise),
                           "GNSS vertical standard deviation is floored before squaring");
        }
        else
        {
            __ESBMC_assert(gps.time_us == 0 && gps.device_id == 0 && gps.generation == 0,
                           "Retained GNSS diagnostics do not invent a new height observation");
        }
        __ESBMC_assert(acceleration.bad == boundary.health.bad_vertical_acceleration &&
                           acceleration.times.update_time_us == boundary.health.last_update_time_us &&
                           acceleration.times.last_good_time_us == boundary.health.last_good_time_us,
                       "Height receives the complete current vertical-health evidence");
        boundary.called = true;
        return boundary.result;
    }
};
struct Reporting
{
    unsigned height_calls{}, velocity_calls{};
    std::uint8_t odometry_count{};
    void record_height(std::uint64_t now, double delta, std::uint8_t count)
    {
        __ESBMC_assert(now == boundary.epoch && boundary.notified && boundary.notification &&
                           contract_proof::same(delta, boundary.result.height_delta) &&
                           count == static_cast<std::uint8_t>(boundary.result.height_resets),
                       "Height reset reporting follows accepted output notification, with the original delta/count");
        ++height_calls;
    }
    void record_vertical_velocity(std::uint64_t now, double delta)
    {
        __ESBMC_assert(now == boundary.epoch && boundary.notified && boundary.notification &&
                           contract_proof::same(delta, boundary.result.velocity_delta),
                       "Height-driven velocity reset is reported after accepted output notification");
        ++velocity_calls;
    }
};
struct Flow
{
    using status_type = Status;
    using height_type = Height;
    std::uint64_t m_time_us{};
    unsigned m_state{}, m_covariance{}, m_errors{}, m_rejections{}, m_baro_updates{};
    bool m_fault{};
    Observation m_barometer;
    struct Gnss
    {
        Observation input;
        struct Sample
        {
            std::uint64_t time_us{};
        } sample;
        struct Lifecycle
        {
            struct State
            {
                struct Velocity
                {
                    bool active{};
                } velocity;
            } value;
            State const & state() const { return value; }
        } lifecycle;
        unsigned corrections{};
    } m_gnss;
    Health m_vertical_health;
    Height m_height_aiding;
    Reporting m_resets;
    struct Bias
    {
        Permissions bias_update;
    } m_bias_learning_result;
    struct Ground
    {
        bool active{};
    } m_ground_effect;
    Vector offset;
    bool qualified{};
    Vector gnss_position_offset() const { return offset; }
    bool gps_good() const { return qualified; }
    bool check(Status status);
    bool reset_output(Quaternion const & q, Vector const & velocity, Vector const & position)
    {
        __ESBMC_assert(boundary.called && formal_eskf::succeeded(boundary.result.status) && q.q0() == 1. &&
                           q.q1() == 0. && q.q2() == 0. && q.q3() == 0. &&
                           contract_proof::same_vector(velocity, vector3(0., 0., boundary.result.velocity_delta)) &&
                           contract_proof::same_vector(position, vector3(0., 0., boundary.result.height_delta)),
                       "Height reset notification preserves both Down deltas and leaves other axes/attitude unchanged");
        boundary.notified = true;
        m_fault |= !boundary.notification;
        return boundary.notification;
    }
    void fuse_height();
};
#include "module_status_check.inc"
#define max formal_eskf_proof_max
#include "module_height.inc"
#undef max
Observation symbolic_observation()
{
    Observation value;
    value.time_us = nondet_time();
    value.generation = nondet_time();
    value.sample.stamp.source = {nondet_value(), nondet_byte()};
    value.sample.altitude_m = nondet_double();
    value.sample.altitude_msl_m = nondet_double();
    value.sample.vertical_accuracy_m = nondet_double();
    return value;
}
} // namespace

extern "C" void verify_module_height()
{
    Flow flow;
    flow.m_time_us = nondet_time();
    flow.m_barometer = symbolic_observation();
    flow.m_gnss.input = symbolic_observation();
    flow.m_gnss.sample.time_us = nondet_time();
    flow.m_gnss.lifecycle.value.velocity.active = nondet_bool();
    flow.m_ground_effect.active = nondet_bool();
    flow.m_vertical_health.value = {nondet_bool(), nondet_time(), nondet_time()};
    flow.offset = vector3(nondet_double(), nondet_double(), nondet_double());
    flow.qualified = nondet_bool();
    flow.m_rejections = nondet_value();
    flow.m_baro_updates = nondet_value();
    flow.m_gnss.corrections = nondet_value();
    flow.m_resets.odometry_count = nondet_byte();
    flow.m_height_aiding.barometer.fused_time = nondet_time();
    flow.m_height_aiding.gnss.fused_time = nondet_time();
    boundary.barometer = flow.m_barometer;
    boundary.gps = flow.m_gnss.input;
    boundary.gnss_time = flow.m_gnss.sample.time_us;
    boundary.epoch = flow.m_time_us;
    boundary.offset = flow.offset(2);
    boundary.qualified = flow.qualified;
    boundary.velocity_active = flow.m_gnss.lifecycle.value.velocity.active;
    boundary.ground_effect = flow.m_ground_effect.active;
    boundary.health = flow.m_vertical_health.value;
    boundary.state = &flow.m_state;
    boundary.covariance = &flow.m_covariance;
    boundary.permissions = &flow.m_bias_learning_result.bias_update;
    boundary.notification = nondet_bool();
    boundary.result.status = nondet_bool() ? Status::success : Status::domain_error;
    boundary.result.height_resets = nondet_value();
    boundary.result.velocity_resets = nondet_value();
    boundary.result.rejected = nondet_value();
    boundary.result.height_delta = nondet_double();
    boundary.result.velocity_delta = nondet_double();
    auto const rejected = flow.m_rejections;
    auto const barometer_updates = flow.m_baro_updates;
    auto const gnss_updates = flow.m_gnss.corrections;
    auto const odometry = flow.m_resets.odometry_count;
    flow.fuse_height();
    bool const numerical = formal_eskf::succeeded(boundary.result.status);
    bool const reset = boundary.result.height_resets != 0 || boundary.result.velocity_resets != 0;
    bool const complete = numerical && (!reset || boundary.notification);
    __ESBMC_assert(boundary.called && boundary.notified == (numerical && reset) && flow.m_fault == !complete,
                   "Height owner failure or output reset failure suppresses subsequent reporting");
    __ESBMC_assert(flow.m_resets.height_calls == unsigned(complete && boundary.result.height_resets != 0) &&
                       flow.m_resets.velocity_calls == unsigned(complete && boundary.result.velocity_resets != 0) &&
                       flow.m_height_aiding.session_reads == (complete ? 2U : 0U),
                   "Height reports exactly the successful owner results after necessary output notification");
    __ESBMC_assert(
        flow.m_rejections == rejected + (complete ? boundary.result.rejected : 0U) &&
            flow.m_resets.odometry_count ==
                static_cast<std::uint8_t>(
                    odometry + (complete ? boundary.result.height_resets + boundary.result.velocity_resets : 0U)) &&
            flow.m_baro_updates ==
                barometer_updates + unsigned(complete && flow.m_height_aiding.barometer.fused_time == flow.m_time_us) &&
            flow.m_gnss.corrections ==
                gnss_updates + unsigned(complete && flow.m_height_aiding.gnss.fused_time == flow.m_time_us),
        "Height accounting uses current delayed fusion epochs and preserves unsigned counter semantics");
}
