// SPDX-License-Identifier: BSD-3-Clause

// Actual startup admission/configuration order. OS/uORB/parameter services have
// explicit return/side-effect contracts; their implementation and concurrent
// execution are outside this single-owner source proof.
#include <cstdint>
#include <cstring>
#include "../contracts.hpp"
#include "../../../../integrations/px4/module/DeploymentProfile.hpp"
#include "module_timing_type.inc"

extern bool nondet_bool();
extern std::int32_t nondet_parameter();
extern std::uint64_t nondet_time();
#define PX4_ERR(...) ((void)0)

namespace
{
namespace profile = formal_eskf::px4::profile;
using Timing = formal_eskf::px4::TimingParameters;
struct Services
{
    bool parameter_ok[6]{};
    std::int32_t values[6]{};
    bool configure_ok[3]{};
    unsigned parameter_reads{}, configuration_calls{};
    bool mutex_ok{}, mutex_called{}, magnetic_ok{}, sensor_positions_ok{}, publications_ok{}, callback_ok{};
    bool magnetic_called{}, positions_called{}, prepared{}, snapshot{}, callback_called{}, scheduled{};
    std::uint64_t clock{};
} services;

int param_find(char const * name)
{
    char const * const names[] = {"FESKF_DT_US",   "FESKF_DELAY_MS", "FESKF_GPS_DLY",
                                  "FESKF_MAG_DLY", "FESKF_BARO_DLY", "FESKF_GRAV_EN"};
    __ESBMC_assert(services.parameter_reads < 6 && ::strcmp(name, names[services.parameter_reads]) == 0,
                   "Startup reads each timing/gravity setting from its declared PX4 parameter");
    return int(services.parameter_reads);
}
int param_get(int id, std::int32_t * output)
{
    __ESBMC_assert(id == int(services.parameter_reads) && id >= 0 && id < 6,
                   "Parameter lookup and destination write belong to the same read");
    *output = services.values[id];
    ++services.parameter_reads;
    return services.parameter_ok[id] ? 0 : -1;
}
int module_mutex_init(unsigned *, void const *)
{
    services.mutex_called = true;
    return services.mutex_ok ? 0 : -1;
}
std::uint64_t hrt_absolute_time() { return services.clock; }

struct Owner
{
    unsigned identity{};
    bool configured{};
    bool configure(Timing const & timing)
    {
        __ESBMC_assert(services.parameter_reads == 5 && services.configuration_calls == identity,
                       "All timing reads finish before IMU, observation and output configuration");
        __ESBMC_assert(timing.filter_interval_us == services.values[0] &&
                           timing.fusion_delay_ms == services.values[1] && timing.gnss_delay_ms == services.values[2] &&
                           timing.magnetic_delay_ms == services.values[3] &&
                           timing.barometer_delay_ms == services.values[4],
                       "Every owner receives the same complete timing configuration with correctly mapped fields");
        ++services.configuration_calls;
        configured = services.configure_ok[identity];
        return configured;
    }
};
struct Flow
{
    bool m_diagnostics_ready{}, m_gravity_enabled{};
    unsigned m_diagnostics_mutex{};
    unsigned perf{};
    unsigned *m_cycle_perf{}, *m_output_perf{}, *m_fusion_perf{};
    Owner m_imu_history{0}, m_observations{1}, m_output{2};
    struct Subscription
    {
        bool registerCallback()
        {
            __ESBMC_assert(
                services.snapshot && services.prepared && services.configuration_calls == 3 &&
                    services.configure_ok[0] && services.configure_ok[1] && services.configure_ok[2],
                "Callbacks cannot start until all timing owners, publications and initial snapshot are ready");
            services.callback_called = true;
            return services.callback_ok;
        }
    } m_imu_sub;
    bool load_magnetic_parameters()
    {
        services.magnetic_called = true;
        return services.magnetic_ok;
    }
    bool load_sensor_positions()
    {
        __ESBMC_assert(services.magnetic_called && services.magnetic_ok,
                       "Sensor position settings follow accepted magnetic settings");
        services.positions_called = true;
        return services.sensor_positions_ok;
    }
    bool prepare_publications()
    {
        __ESBMC_assert(services.positions_called && services.sensor_positions_ok,
                       "Publication setup follows complete accepted startup configuration");
        services.prepared = true;
        return services.publications_ok;
    }
    void update_diagnostics(std::uint64_t now)
    {
        __ESBMC_assert(m_diagnostics_ready && services.prepared && services.publications_ok &&
                           !services.callback_called && now == services.clock,
                       "Initial diagnostic snapshot is complete before callback registration");
        services.snapshot = true;
    }
    void ScheduleNow()
    {
        __ESBMC_assert(services.callback_called && services.callback_ok,
                       "Immediate scheduling requires callback registration");
        services.scheduled = true;
    }
    bool load_timing_parameters();
    bool load_startup_parameters();
    bool init();
};
#define pthread_mutex_init module_mutex_init
#include "module_startup.inc"
#undef pthread_mutex_init
} // namespace

extern "C" void verify_module_startup()
{
    Flow flow;
    flow.m_diagnostics_ready = nondet_bool();
    flow.m_gravity_enabled = nondet_bool();
    flow.m_cycle_perf = nondet_bool() ? &flow.perf : nullptr;
    flow.m_output_perf = nondet_bool() ? &flow.perf : nullptr;
    flow.m_fusion_perf = nondet_bool() ? &flow.perf : nullptr;
    for (unsigned i = 0; i < 6; ++i)
    {
        services.parameter_ok[i] = nondet_bool();
        services.values[i] = nondet_parameter();
    }
    for (unsigned i = 0; i < 3; ++i)
    {
        services.configure_ok[i] = nondet_bool();
    }
    services.mutex_ok = nondet_bool();
    services.magnetic_ok = nondet_bool();
    services.sensor_positions_ok = nondet_bool();
    services.publications_ok = nondet_bool();
    services.callback_ok = nondet_bool();
    services.clock = nondet_time();
    bool const ready = flow.m_diagnostics_ready;
    bool const gravity = flow.m_gravity_enabled;
    bool const prerequisites =
        (ready || services.mutex_ok) && flow.m_cycle_perf && flow.m_output_perf && flow.m_fusion_perf;
    unsigned reads{};
    if (prerequisites)
    {
        do
        {
            ++reads;
        } while (reads < 5 && services.parameter_ok[reads - 1]);
    }
    bool const timing_reads = prerequisites && reads == 5 && services.parameter_ok[4];
    unsigned configurations{};
    if (timing_reads)
    {
        do
        {
            ++configurations;
        } while (configurations < 3 && services.configure_ok[configurations - 1]);
    }
    bool const timing = timing_reads && configurations == 3 && services.configure_ok[2];
    bool const gravity_valid = timing && services.parameter_ok[5] && services.values[5] >= 0 && services.values[5] <= 1;
    bool const configured = gravity_valid && services.magnetic_ok && services.sensor_positions_ok;
    bool const snapshot = configured && services.publications_ok;
    bool const result = flow.init();
    __ESBMC_assert(result == (snapshot && services.callback_ok) && services.scheduled == result &&
                       services.callback_called == snapshot && services.snapshot == snapshot &&
                       services.prepared == configured,
                   "Every startup failure prevents callback scheduling; success requires all startup resources");
    __ESBMC_assert(services.mutex_called == !ready && flow.m_diagnostics_ready == (ready || services.mutex_ok) &&
                       services.parameter_reads == reads + unsigned(timing) &&
                       services.configuration_calls == configurations,
                   "Startup evaluates only the successful resource/configuration prefix");
    __ESBMC_assert(services.magnetic_called == gravity_valid &&
                       services.positions_called == (gravity_valid && services.magnetic_ok) &&
                       flow.m_gravity_enabled ==
                           (gravity_valid ? profile::gravity::enabled && services.values[5] != 0 : gravity),
                   "Runtime gravity setting cannot enable compile-time excluded support or bypass remaining settings");
}
