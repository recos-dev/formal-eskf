// SPDX-License-Identifier: BSD-3-Clause

// Actual source selection and startup context freshness. Queue pop contracts
// supply a new event or preserve the destination; GNSS quality's consumed
// watermark is a separate producer obligation, not assumed by this selection.
#include <cstdint>

extern bool nondet_bool();
extern std::uint64_t nondet_time();
extern unsigned nondet_value();
extern void __ESBMC_assert(bool, char const *);

namespace
{
#include "module_constants.inc"
struct Observation
{
    std::uint64_t time_us{}, generation{};
    unsigned payload{};
};
struct Observations
{
    struct LandState
    {
        std::uint64_t time_us{};
        bool known{}, landed{}, at_rest{}, in_ground_effect{};
    };
    struct VehicleState
    {
        std::uint64_t time_us{};
        bool known{}, armed{}, fixed_wing{};
    };
    struct Context
    {
        std::uint64_t time_us{};
        LandState land;
        VehicleState vehicle;
    };
    struct Result
    {
        std::uint64_t retired{};
    };
    Observation selected[3];
    bool available[3]{};
    std::uint64_t retired[3]{};
    unsigned pops{}, contexts{};
    std::uint64_t context_times[2]{}, context_ages[2]{};
    Result pop(unsigned source, Observation & output)
    {
        __ESBMC_assert(pops == source, "One pop per source in GNSS, magnetic, barometer order");
        ++pops;
        if (available[source])
        {
            output = selected[source];
        }
        return {retired[source]};
    }
    Result pop_gnss(Observation & output) { return pop(0, output); }
    Result pop_magnetic(Observation & output) { return pop(1, output); }
    Result pop_barometer(Observation & output) { return pop(2, output); }
    Context context(std::uint64_t time, std::uint64_t age)
    {
        __ESBMC_assert(pops == 3 && contexts < 2, "Context lookup follows all three source selections");
        context_times[contexts] = time;
        context_ages[contexts] = age;
        ++contexts;
        return {time, {}, {}};
    }
};
struct Flow
{
    using observation_history_type = Observations;
    Observations m_observations;
    std::uint64_t m_time_us{}, m_retired_observations{};
    struct Selected
    {
        Observation input;
        Observations::Context context;
    } m_gnss, m_magnetic;
    Observation m_barometer;
    bool quality_called{};
    Observations::LandState land;
    Observations::VehicleState vehicle;
    Observations::LandState land_state() const { return land; }
    Observations::VehicleState vehicle_state() const { return vehicle; }
    void update_gps_quality()
    {
        __ESBMC_assert(m_observations.pops == 3 && m_observations.contexts == 2,
                       "GNSS quality is evaluated only after selected inputs and historical context are installed");
        quality_called = true;
    }
    void select_observations();
    Observations::Context startup_context(std::uint64_t epoch, std::uint64_t maximum_age) const;
};

#include "module_selection.inc"
#include "module_context.inc"

Observation symbolic_observation() { return {nondet_time(), nondet_time(), nondet_value()}; }
bool same(Observation a, Observation b)
{
    return a.time_us == b.time_us && a.generation == b.generation && a.payload == b.payload;
}
} // namespace

extern "C" void verify_module_selection()
{
    Flow flow;
    flow.m_time_us = nondet_time();
    flow.m_retired_observations = nondet_time();
    flow.m_gnss.input = symbolic_observation();
    flow.m_magnetic.input = symbolic_observation();
    flow.m_barometer = symbolic_observation();
    auto const old_gnss = flow.m_gnss.input;
    auto const retired = flow.m_retired_observations;
    for (unsigned source = 0; source < 3; ++source)
    {
        flow.m_observations.available[source] = nondet_bool();
        flow.m_observations.selected[source] = symbolic_observation();
        flow.m_observations.retired[source] = nondet_time();
    }
    flow.select_observations();
    auto const & queue = flow.m_observations;
    __ESBMC_assert(
        (queue.available[0] ? same(flow.m_gnss.input, queue.selected[0]) : same(flow.m_gnss.input, old_gnss)) &&
            (queue.available[1] ? same(flow.m_magnetic.input, queue.selected[1])
                                : same(flow.m_magnetic.input, Observation{})) &&
            (queue.available[2] ? same(flow.m_barometer, queue.selected[2]) : same(flow.m_barometer, Observation{})),
        "A missing magnetic/barometer event is cleared; only GNSS diagnostic data can be retained");
    __ESBMC_assert(queue.context_times[0] == flow.m_gnss.input.time_us &&
                       queue.context_times[1] ==
                           (flow.m_magnetic.input.time_us == 0 ? flow.m_time_us : flow.m_magnetic.input.time_us) &&
                       queue.context_ages[0] == profile::gnss::context_maximum_age_us &&
                       queue.context_ages[1] == profile::magnetic::context_maximum_age_us &&
                       flow.m_gnss.context.time_us == queue.context_times[0] &&
                       flow.m_magnetic.context.time_us == queue.context_times[1],
                   "Event context uses the selected effective epoch; magnetic no-event context uses delayed time");
    __ESBMC_assert(flow.quality_called && queue.pops == 3 && queue.contexts == 2 &&
                       flow.m_retired_observations == retired + queue.retired[0] + queue.retired[1] + queue.retired[2],
                   "Selection reports all retired events and evaluates quality once");
}

extern "C" void verify_module_startup_context()
{
    Flow flow;
    flow.land = {nondet_time(), nondet_bool(), nondet_bool(), nondet_bool(), nondet_bool()};
    flow.vehicle = {nondet_time(), nondet_bool(), nondet_bool(), nondet_bool()};
    auto const epoch = nondet_time();
    auto const age = nondet_time();
    auto const context = flow.startup_context(epoch, age);
    __ESBMC_assert(context.time_us == epoch && context.land.time_us == flow.land.time_us &&
                       context.vehicle.time_us == flow.vehicle.time_us,
                   "Startup context retains original message epochs and its own evaluation epoch");
    __ESBMC_assert(context.land.known ==
                           (flow.land.known && flow.land.time_us <= epoch && epoch - flow.land.time_us < age) &&
                       context.vehicle.known ==
                           (flow.vehicle.known && flow.vehicle.time_us <= epoch && epoch - flow.vehicle.time_us < age),
                   "Future, expired or unknown flight context cannot acquire startup authority");
    __ESBMC_assert(context.land.landed == flow.land.landed && context.land.at_rest == flow.land.at_rest &&
                       context.land.in_ground_effect == flow.land.in_ground_effect &&
                       context.vehicle.armed == flow.vehicle.armed &&
                       context.vehicle.fixed_wing == flow.vehicle.fixed_wing,
                   "Freshness processing preserves the reported flight-state values");
}
