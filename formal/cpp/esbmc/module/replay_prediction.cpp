// SPDX-License-Identifier: BSD-3-Clause

// Whole production helper with actual binary64 parameter/noise fields and
// opaque state/P identities. Arithmetic producers remain separate dependencies.
#include <cstdint>
#include "../contracts.hpp"
#include "replay_support.hpp"
#include <formal_eskf/eskf/configuration/ins.hpp>
#define optional formal_eskf_replay_optional

extern bool nondet_bool();
extern double nondet_double();
extern std::uint64_t nondet_time();
extern unsigned nondet_value();
extern void __ESBMC_init_object(void *);

namespace formal_eskf::runtime
{
struct ImuNoiseConditions
{
    bool bad_vertical_acceleration{};
    bool accelerometer_clipped[3]{};
};
} // namespace formal_eskf::runtime
namespace
{
using contract_proof::Linalg;
using contract_proof::same;
using contract_proof::same_vector;
using Noise = formal_eskf::configuration::Ins::ProcessNoise<Linalg>;
using Parameters = formal_eskf::configuration::Ins::Parameters<Linalg>;
using Status = formal_eskf::Status;
using State = unsigned;
using Covariance = unsigned;
struct Imu
{
    unsigned identity{};
};
struct Types
{
    using parameter_type = Parameters;
};
struct ReplayFailure
{
};
#include "replay_prediction_types.inc"
bool same_noise(Noise const & a, Noise const & b)
{
    return same_vector(a.specific_force_variance, b.specific_force_variance) &&
           same_vector(a.angular_rate_variance, b.angular_rate_variance) &&
           same_vector(a.accelerometer_bias_random_walk_variance_density,
                       b.accelerometer_bias_random_walk_variance_density) &&
           same_vector(a.gyroscope_bias_random_walk_variance_density, b.gyroscope_bias_random_walk_variance_density);
}
struct Boundary
{
    Parameters const * settings{};
    State * state{};
    Covariance * covariance{};
    Imu const * sample{};
    formal_eskf::runtime::ImuNoiseConditions const * conditions{};
    Noise selected, inhibited;
    Status selection{}, inhibition{}, prediction{};
    unsigned selections{}, inhibitions{}, predictions{};
    std::uint64_t period{};
} boundary;
#include "replay_status.inc"
} // namespace
namespace formal_eskf::runtime
{
Status select_imu_process_noise(Noise const & input, ImuNoiseConditions const & conditions, double variance,
                                Noise & output)
{
    __ESBMC_assert(&input == &boundary.settings->process_noise && &conditions == boundary.conditions &&
                       &input != &output && same(variance, BAD_ACCEL_STD * BAD_ACCEL_STD),
                   "Replay noise selection reads the configured noise and actual conditions into a separate step copy");
    ++boundary.selections;
    if (succeeded(boundary.selection))
    {
        output = boundary.selected;
    }
    return boundary.selection;
}
Status inhibit_bias_process_noise(Covariance const & covariance, Noise const & input, Noise & output)
{
    __ESBMC_assert(&covariance == boundary.covariance && &input == &output && boundary.inhibitions == 0 &&
                       boundary.selections == unsigned(boundary.conditions != nullptr),
                   "Replay inhibition follows optional noise selection with the intended in-place noise alias");
    if (boundary.conditions)
    {
        __ESBMC_assert(same_noise(input, boundary.selected),
                       "Bias inhibition sees the selected noise, not stale defaults");
    }
    else
    {
        __ESBMC_assert(same_noise(input, boundary.settings->process_noise),
                       "Unavailable clipping retains configured noise");
    }
    ++boundary.inhibitions;
    if (succeeded(boundary.inhibition))
    {
        output = boundary.inhibited;
    }
    return boundary.inhibition;
}
} // namespace formal_eskf::runtime
namespace formal_eskf
{
Status predict(State const & state, Covariance const & covariance, Imu const & sample, double dt,
               Parameters const & settings, State & state_out, Covariance & covariance_out)
{
    __ESBMC_assert(&state == boundary.state && &state_out == boundary.state && &covariance == boundary.covariance &&
                       &covariance_out == boundary.covariance && &sample == boundary.sample &&
                       same(dt, static_cast<double>(boundary.period) * 1e-6),
                   "Replay prediction forwards the same state/P in-place, actual IMU and microsecond duration");
    __ESBMC_assert(
        boundary.inhibitions == 1 && same_noise(settings.process_noise, boundary.inhibited) &&
            same_vector(settings.gravity_n, boundary.settings->gravity_n) &&
            same(settings.minimum_quaternion_norm, boundary.settings->minimum_quaternion_norm) &&
            same(settings.quaternion_squared_norm_tolerance, boundary.settings->quaternion_squared_norm_tolerance) &&
            same(settings.dt_min, boundary.settings->dt_min) && same(settings.dt_max, boundary.settings->dt_max),
        "Prediction receives selected/inhibited noise while all other configured parameters are preserved");
    ++boundary.predictions;
    if (succeeded(boundary.prediction))
    {
        state_out = 1;
        covariance_out = 2;
    }
    return boundary.prediction;
}
} // namespace formal_eskf
namespace
{
#include "replay_prediction.inc"
Status symbolic_status()
{
    unsigned value = nondet_value();
    __ESBMC_assume(value <= unsigned(Status::non_finite_result));
    return static_cast<Status>(value);
}
} // namespace
extern "C" void verify_replay_prediction()
{
    Parameters settings;
    Noise selected, inhibited;
    __ESBMC_init_object(&settings);
    __ESBMC_init_object(&selected);
    __ESBMC_init_object(&inhibited);
    State state = nondet_value();
    Covariance covariance = nondet_value();
    Imu sample{nondet_value()};
    PredictionStatistics statistics;
    __ESBMC_init_object(&statistics);
    auto const before = statistics;
    std::optional<formal_eskf::runtime::ImuNoiseConditions> conditions;
    if (nondet_bool())
    {
        auto & value = conditions.emplace();
        value.bad_vertical_acceleration = nondet_bool();
        for (unsigned axis = 0; axis < 3; ++axis)
        {
            value.accelerometer_clipped[axis] = nondet_bool();
        }
    }
    boundary.settings = &settings;
    boundary.state = &state;
    boundary.covariance = &covariance;
    boundary.sample = &sample;
    if (conditions)
    {
        boundary.conditions = &*conditions;
    }
    boundary.selected = selected;
    boundary.inhibited = inhibited;
    boundary.selection = symbolic_status();
    boundary.inhibition = symbolic_status();
    boundary.prediction = symbolic_status();
    boundary.period = nondet_time();
    bool failed = false;
    Noise result;
    try
    {
        result = predict_interval(state, covariance, sample, boundary.period, nondet_time(), settings, conditions,
                                  statistics);
    }
    catch (...)
    {
        failed = true;
    }
    bool const inhibited_called = !conditions || formal_eskf::succeeded(boundary.selection);
    bool const predicted = inhibited_called && formal_eskf::succeeded(boundary.inhibition);
    bool const success = predicted && formal_eskf::succeeded(boundary.prediction);
    __ESBMC_assert(failed == !success && boundary.selections == unsigned(conditions.has_value()) &&
                       boundary.inhibitions == unsigned(inhibited_called) &&
                       boundary.predictions == unsigned(predicted),
                   "Each failed noise or prediction status aborts before subsequent replay numerical work");
    __ESBMC_assert(statistics.accel_noise_unavailable ==
                       before.accel_noise_unavailable + std::size_t(success && !conditions),
                   "Unavailable-noise accounting commits only after successful prediction");
    for (unsigned axis = 0; axis < 3; ++axis)
    {
        bool const overridden =
            success && conditions && (conditions->bad_vertical_acceleration || conditions->accelerometer_clipped[axis]);
        bool const accel = success &&
                           settings.process_noise.accelerometer_bias_random_walk_variance_density(axis) > 0.0 &&
                           inhibited.accelerometer_bias_random_walk_variance_density(axis) == 0.0;
        bool const gyro = success && settings.process_noise.gyroscope_bias_random_walk_variance_density(axis) > 0.0 &&
                          inhibited.gyroscope_bias_random_walk_variance_density(axis) == 0.0;
        __ESBMC_assert(statistics.accel_noise_overridden[axis] ==
                               before.accel_noise_overridden[axis] + std::size_t(overridden) &&
                           statistics.accel_bias_noise_inhibited[axis] ==
                               before.accel_bias_noise_inhibited[axis] + std::size_t(accel) &&
                           statistics.gyro_bias_noise_inhibited[axis] ==
                               before.gyro_bias_noise_inhibited[axis] + std::size_t(gyro),
                       "Each diagnostic counts the noise actually passed to a successful prediction");
    }
    if (success)
    {
        __ESBMC_assert(same_noise(result, inhibited), "Returned noise is the selected prediction noise");
    }
}
