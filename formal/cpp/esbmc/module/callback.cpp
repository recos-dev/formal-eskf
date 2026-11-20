// SPDX-License-Identifier: BSD-3-Clause

// Source-level Run() ordering. Opaque sensor/IMU arguments retain identity;
// component implementations, uORB delivery and work-queue exclusion are the
// explicit call boundary. No numerical payload or target execution claim.
#include <cstdint>

extern bool nondet_bool();
extern std::uint64_t nondet_time();
extern void __ESBMC_assume(bool);
extern void __ESBMC_assert(bool, char const *);

namespace
{
#include "module_constants.inc"

enum class Event : unsigned
{
    idle,
    process,
    gnss,
    magnetic,
    barometer,
    advance,
    status,
    diagnostics
};

struct Trace
{
    unsigned seen{};
    Event last{};
    void record(Event event)
    {
        __ESBMC_assert(static_cast<unsigned>(event) > static_cast<unsigned>(last),
                       "Run calls each stage at most once in production order");
        last = event;
        seen |= 1U << static_cast<unsigned>(event);
    }
    bool has(Event event) const { return (seen & (1U << static_cast<unsigned>(event))) != 0; }
} trace;

std::uint64_t clock_time;
std::uint64_t hrt_absolute_time() { return clock_time; }
void perf_begin(unsigned) {}
void perf_end(unsigned) {}

struct Sample
{
    unsigned identity;
};

struct Observations
{
    unsigned push_gnss(Sample sample, std::uint64_t now)
    {
        trace.record(Event::gnss);
        __ESBMC_assert(sample.identity == 2 && now == clock_time, "GNSS arrival uses current callback time");
        return 1;
    }
    unsigned push_magnetic(Sample sample, std::uint64_t now)
    {
        trace.record(Event::magnetic);
        __ESBMC_assert(sample.identity == 3 && now == clock_time, "Magnetic arrival uses current callback time");
        return 1;
    }
    unsigned push_barometer(Sample sample, std::uint64_t now)
    {
        trace.record(Event::barometer);
        __ESBMC_assert(sample.identity == 4 && now == clock_time, "Barometer arrival uses current callback time");
        return 1;
    }
};

struct Flow
{
    struct InputUpdates
    {
        Sample sample{1};
        bool imu{}, gps{}, magnetic{}, barometer{};
    } updates;
    struct Subscription
    {
        bool unregistered{};
        void unregisterCallback() { unregistered = true; }
    } m_imu_sub;
    bool m_initialized{}, m_fault{}, exit{}, cleared{}, cleaned{}, scheduled{}, read{};
    bool fault_after_process{}, initialized_after_process{};
    unsigned admissions{};
    std::uint64_t m_last_status{};
    unsigned m_cycle_perf{}, m_output_perf{};
    Sample m_gps{2}, m_mag{3}, m_baro{4};
    Observations m_observations;

    bool should_exit() const { return exit; }
    void ScheduleClear() { cleared = true; }
    void exit_and_cleanup()
    {
        __ESBMC_assert(m_imu_sub.unregistered && cleared, "Exit unregisters and clears before cleanup");
        cleaned = true;
    }
    void ScheduleDelayed(std::uint64_t interval)
    {
        __ESBMC_assert(interval == profile::backup_interval_us, "Backup schedule uses the configured interval");
        scheduled = true;
    }
    InputUpdates read_inputs()
    {
        __ESBMC_assert(scheduled && !read, "Read one input snapshot after scheduling backup");
        read = true;
        return updates;
    }
    static Sample decode_imu(Sample sample) { return sample; }
    static Sample decode_gnss(Sample sample) { return sample; }
    static Sample decode_magnetic(Sample sample) { return sample; }
    static Sample decode_barometer(Sample sample) { return sample; }
    void process_imu(Sample sample)
    {
        trace.record(Event::process);
        __ESBMC_assert(read && sample.identity == 1, "Current processing uses this callback's IMU sample");
        // Overapproximate the separately checked process_imu contract. Failure
        // can occur before or after initialization; a latched fault stays set.
        if (!m_fault)
        {
            m_fault = nondet_bool();
            m_initialized = m_initialized || nondet_bool();
        }
        fault_after_process = m_fault;
        initialized_after_process = m_initialized;
    }
    void record_admission(unsigned)
    {
        // Actual method only updates admission counters; it cannot latch fault.
        ++admissions;
    }
    void advance_fusion()
    {
        trace.record(Event::advance);
        __ESBMC_assert(trace.has(Event::process) && m_initialized && !m_fault,
                       "Delayed work requires this tick's current propagation and no fault");
        m_fault = nondet_bool();
    }
    void publish_status(std::uint64_t now)
    {
        trace.record(Event::status);
        __ESBMC_assert(now == clock_time, "Status uses the callback wall time");
    }
    void update_diagnostics(std::uint64_t now)
    {
        trace.record(Event::diagnostics);
        __ESBMC_assert(now == clock_time, "Diagnostic snapshot uses the callback wall time");
    }
    void Run();
};

#include "module_callback.inc"
} // namespace

extern "C" void verify_module_callback()
{
    Flow flow;
    flow.exit = nondet_bool();
    flow.m_initialized = nondet_bool();
    flow.m_fault = nondet_bool();
    flow.updates.imu = nondet_bool();
    flow.updates.gps = nondet_bool();
    flow.updates.magnetic = nondet_bool();
    flow.updates.barometer = nondet_bool();
    flow.m_last_status = nondet_time();
    clock_time = nondet_time();
    // PX4's monotonic clock is an external premise, not proved by ESBMC.
    __ESBMC_assume(clock_time >= flow.m_last_status);
    bool const initialized = flow.m_initialized;
    bool const fault = flow.m_fault;
    auto const last_status = flow.m_last_status;
    flow.Run();
    if (flow.exit)
    {
        __ESBMC_assert(flow.cleaned && !flow.read && !flow.scheduled && trace.seen == 0,
                       "Exit performs no estimator or publication work");
        __ESBMC_assert(flow.m_fault == fault && flow.m_initialized == initialized && flow.m_last_status == last_status,
                       "Exit preserves estimator flags and status watermark");
        return;
    }
    bool const admitted = initialized && !(flow.updates.imu ? flow.fault_after_process : fault);
    __ESBMC_assert(trace.has(Event::process) == flow.updates.imu, "Backup callbacks never invent an IMU step");
    __ESBMC_assert(trace.has(Event::gnss) == (admitted && flow.updates.gps) &&
                       trace.has(Event::magnetic) == (admitted && flow.updates.magnetic) &&
                       trace.has(Event::barometer) == (admitted && flow.updates.barometer),
                   "Only eligible new arrivals enter histories; initialization tick does not enqueue twice");
    __ESBMC_assert(trace.has(Event::advance) == (admitted && flow.updates.imu),
                   "One delayed attempt only on an already initialized IMU callback");
    __ESBMC_assert(flow.admissions == unsigned(trace.has(Event::gnss)) + unsigned(trace.has(Event::magnetic)) +
                                          unsigned(trace.has(Event::barometer)),
                   "Every arrival decision is reported exactly once");
    bool const status_due = clock_time - last_status >= profile::status_interval_us;
    __ESBMC_assert(trace.has(Event::status) == status_due &&
                       flow.m_last_status == (status_due ? clock_time : last_status),
                   "Status scheduling and watermark are independent of estimator fault");
    __ESBMC_assert(trace.has(Event::diagnostics) && !flow.cleaned,
                   "Every continuing callback finishes with one diagnostic snapshot");
    __ESBMC_assert(!fault || flow.m_fault, "A preexisting fault cannot be cleared by the callback");
}
