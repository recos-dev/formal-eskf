// SPDX-License-Identifier: BSD-3-Clause

// Actual advance_fusion body under the accepted history, prediction and output
// contracts. Numeric state is an opaque identity here, never a numeric model.
#include <cstdint>

extern bool nondet_bool();
extern std::uint64_t nondet_time();
extern unsigned nondet_value();
extern void __ESBMC_assume(bool);
extern void __ESBMC_assert(bool, char const *);
#define PX4_ERR(...) ((void)0)

namespace
{
void perf_begin(unsigned) {}
void perf_end(unsigned) {}

struct Sample
{
    std::uint64_t time_us{};
    std::uint64_t mean_rates() const { return time_us; }
};
struct Interval
{
    Sample sample{};
};
struct History
{
    Interval oldest{}, newest{};
    bool available{}, completed{}, completion_ok{};
    std::uint64_t completed_epoch{};
    unsigned latest_reads{};
    Interval const * delayed() const { return available ? &oldest : nullptr; }
    Interval const * latest()
    {
        ++latest_reads;
        __ESBMC_assert(available, "A retained delayed interval implies a retained newest interval");
        return &newest;
    }
    bool complete(std::uint64_t time)
    {
        __ESBMC_assert(!completed && time == oldest.sample.time_us, "Complete exactly the consumed delayed epoch");
        completed = true;
        completed_epoch = time;
        // ImuHistory.complete changes its watermark, not sample storage.
        return completion_ok;
    }
};
struct Output
{
    History * history{};
    unsigned * main_state{};
    bool called{}, success{};
    std::uint64_t epoch{};
    bool correct(std::uint64_t time, unsigned const & state)
    {
        __ESBMC_assert(!called && !history->completed && time == history->oldest.sample.time_us && &state == main_state,
                       "Output feedback precedes completion and uses the committed main state and delayed epoch");
        called = true;
        epoch = time;
        return success;
    }
};
struct Flow
{
    History m_imu_history;
    Output m_output;
    bool m_fault{}, predicted{}, prediction_ok{}, published{};
    unsigned m_state{}, m_errors{}, errors_after_prediction{}, m_fusion_perf{};
    std::uint64_t m_time_us{}, publication_input{};
    void predict_and_fuse(Interval const & interval)
    {
        __ESBMC_assert(!predicted && !m_fault && &interval == &m_imu_history.oldest,
                       "Main prediction consumes the borrowed oldest eligible interval once");
        predicted = true;
        // Numerical/fusion work may fail after committing a prefix. Neither
        // old coefficients nor the old epoch are assumed restored on failure.
        m_state = nondet_value();
        if (prediction_ok || nondet_bool())
        {
            m_time_us = interval.sample.time_us;
        }
        m_fault = !prediction_ok;
        if (!prediction_ok)
        {
            m_errors = nondet_value();
        }
        errors_after_prediction = m_errors;
    }
    void publish(std::uint64_t rates)
    {
        __ESBMC_assert(!published && !m_fault && m_output.called && m_output.success && m_imu_history.completed &&
                           m_imu_history.completion_ok,
                       "Navigation publication requires successful feedback and completion with no fault");
        __ESBMC_assert(rates == m_imu_history.newest.sample.time_us,
                       "Publication's IMU rates come from the newest interval, not the delayed one");
        published = true;
        publication_input = rates;
    }
    bool check_timing(bool succeeded);
    void advance_fusion();
};

#include "module_feedback.inc"
} // namespace

extern "C" void verify_module_feedback()
{
    Flow flow;
    flow.m_output.history = &flow.m_imu_history;
    flow.m_output.main_state = &flow.m_state;
    flow.m_fault = nondet_bool();
    flow.prediction_ok = nondet_bool();
    flow.m_output.success = nondet_bool();
    flow.m_imu_history.available = nondet_bool();
    flow.m_imu_history.completion_ok = nondet_bool();
    flow.m_time_us = nondet_time();
    flow.m_imu_history.oldest.sample.time_us = nondet_time();
    flow.m_imu_history.newest.sample.time_us = nondet_time();
    flow.m_state = nondet_value();
    flow.m_errors = nondet_value();
    __ESBMC_assume(!flow.m_imu_history.available ||
                   (flow.m_time_us < flow.m_imu_history.oldest.sample.time_us &&
                    flow.m_imu_history.oldest.sample.time_us <= flow.m_imu_history.newest.sample.time_us));
    bool const fault = flow.m_fault;
    auto const epoch = flow.m_time_us;
    auto const state = flow.m_state;
    auto const errors = flow.m_errors;
    auto const oldest = flow.m_imu_history.oldest.sample.time_us;
    auto const newest = flow.m_imu_history.newest.sample.time_us;
    flow.advance_fusion();
    bool const entered = flow.m_imu_history.available && !fault;
    bool const feedback = entered && flow.prediction_ok;
    bool const complete = feedback && flow.m_output.success;
    bool const publish = complete && flow.m_imu_history.completion_ok;
    __ESBMC_assert(flow.predicted == entered && flow.m_output.called == feedback &&
                       flow.m_imu_history.completed == complete && flow.published == publish,
                   "Every failed prefix prevents its dependent stages");
    __ESBMC_assert(flow.m_fault == (fault || (entered && !publish)), "Delayed failure remains latched");
    __ESBMC_assert(flow.m_errors == (entered ? flow.errors_after_prediction : errors) + unsigned(feedback && !publish),
                   "Timing failure adds one error to the numerical callee's reported errors");
    __ESBMC_assert(flow.m_imu_history.oldest.sample.time_us == oldest &&
                       flow.m_imu_history.newest.sample.time_us == newest,
                   "Borrowed history samples survive the complete delayed call chain");
    __ESBMC_assert(flow.m_imu_history.latest_reads == unsigned(publish),
                   "Newest history is dereferenced only after successful completion");
    if (!entered)
    {
        __ESBMC_assert(flow.m_time_us == epoch && flow.m_state == state,
                       "Warmup or preexisting fault preserves the main state and epoch");
    }
    if (feedback)
    {
        __ESBMC_assert(flow.m_time_us == oldest && flow.m_output.epoch == oldest,
                       "Successful delayed prediction and output feedback share an epoch");
    }
}
