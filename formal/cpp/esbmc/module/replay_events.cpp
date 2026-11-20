// SPDX-License-Identifier: BSD-3-Clause

// Arbitrary event-drain steps, including the original loop guard and exit
// frame. Iterating these invariant-preserving steps consumes a finite stream;
// this is not a fixed-length replay or a model of stream/CSV implementation.
#include <cstdint>
#include "../algorithm_support.hpp"
#include "replay_support.hpp"
#define optional formal_eskf_replay_optional

extern bool nondet_bool();
extern unsigned nondet_unsigned();
extern std::uint64_t nondet_time();
extern void __ESBMC_assert(bool, char const *);
extern void __ESBMC_assume(bool);

namespace
{
struct ReplayFailure
{
};
std::uint64_t timestamp(std::uint64_t decoded) { return decoded; }
struct Stream
{
    template <typename Value> Stream & operator<<(Value const &) { return *this; }
};
struct Input
{
    unsigned advances{};
    bool next_available{}, allow_non_finite{};
    std::uint64_t next_time{};
};
bool read_row(Input & stream, std::uint64_t * row, bool allow_non_finite = false)
{
    ++stream.advances;
    stream.allow_non_finite = allow_non_finite;
    row[0] = stream.next_time;
    return stream.next_available;
}
struct Lifecycle
{
    struct Value
    {
        struct Position
        {
            bool active{};
        } vertical_position;
    } value;
    Value const & state() const { return value; }
};
void write_gnss_lifecycle(Stream &, Lifecycle::Value const &, Lifecycle::Value const &, std::uint64_t, std::uint64_t) {}
struct Navigation
{
    std::uint64_t navigation_row[1]{}, time{}, last_navigation_time{}, max_alignment_us{};
    unsigned navigation_events{}, corrections{}, captures{};
    bool has_navigation{}, quality_allows_fusion{}, lifecycle_sample_consumed{}, correction_ok{};
    Input navigation;
    Stream updates, lifecycle_updates;
    Lifecycle gnss_lifecycle;
    Lifecycle::Value previous_lifecycle;
    std::optional<unsigned> quality_parameters;
    void capture_vertical_evidence(bool) { ++captures; }
    void step()
    {
        // Original loop guard. The do/while(false) represents one iteration;
        // its continue leaves the step, matching the original loop backedge.
        if (has_navigation && timestamp(navigation_row[0U]) <= time)
        {
            do
            {
#include "replay_navigation_guard.inc"
                // Quality/lifecycle contracts provide quality_allows_fusion
                // without writing the stream cursor or current IMU epoch.
#define max formal_eskf_proof_max
#include "replay_navigation_cursor.inc"
#undef max
                ++corrections;
                if (!correction_ok)
                {
                    throw ReplayFailure{};
                }
#include "replay_navigation_advance.inc"
            } while (false);
        }
    }
};
} // namespace

extern "C" void verify_replay_navigation()
{
    Navigation flow;
    flow.navigation_row[0] = nondet_time();
    flow.time = nondet_time();
    flow.last_navigation_time = nondet_time();
    flow.max_alignment_us = nondet_time();
    flow.navigation_events = nondet_unsigned();
    flow.has_navigation = nondet_bool();
    flow.quality_allows_fusion = nondet_bool();
    flow.lifecycle_sample_consumed = nondet_bool();
    flow.correction_ok = nondet_bool();
    if (nondet_bool())
    {
        flow.quality_parameters = 1;
    }
    flow.navigation.next_available = nondet_bool();
    flow.navigation.next_time = nondet_time();
    __ESBMC_assume(flow.navigation_row[0] != 0 && flow.last_navigation_time <= flow.time);
    auto const before = flow;
    bool failed = false;
    try
    {
        flow.step();
    }
    catch (...)
    {
        failed = true;
    }
    bool const due = before.has_navigation && before.navigation_row[0] <= before.time;
    bool const admitted = due && before.navigation_row[0] > before.last_navigation_time;
    bool const attempted = admitted && before.quality_allows_fusion;
    bool const advanced = admitted && (!attempted || before.correction_ok);
    __ESBMC_assert(failed == (due && (!admitted || (attempted && !before.correction_ok))) &&
                       flow.corrections == unsigned(attempted) && flow.captures == unsigned(admitted && !attempted),
                   "Replay rejects repeated/out-of-order navigation before fusion and never fuses quality rejection");
    __ESBMC_assert(
        flow.navigation.advances == unsigned(advanced) &&
            flow.has_navigation == (advanced ? before.navigation.next_available : before.has_navigation) &&
            flow.navigation_row[0] == (advanced ? before.navigation.next_time : before.navigation_row[0]),
        "Both accepted and quality-rejected events advance exactly once; future/failed work does not read ahead");
    __ESBMC_assert(
        flow.time == before.time && flow.last_navigation_time <= flow.time &&
            flow.last_navigation_time == (admitted ? before.navigation_row[0] : before.last_navigation_time) &&
            flow.navigation_events == before.navigation_events + unsigned(admitted),
        "Navigation source watermark increases at the current IMU epoch without retimestamping the estimator");
    if (admitted)
    {
        auto const alignment = before.time - before.navigation_row[0];
        __ESBMC_assert(flow.max_alignment_us ==
                           (alignment > before.max_alignment_us ? alignment : before.max_alignment_us),
                       "Replay records the true next-tick alignment age even on quality rejection");
    }
    if (advanced)
    {
        __ESBMC_assert(flow.navigation.allow_non_finite == (!attempted || flow.quality_parameters.has_value()),
                       "Input advance preserves the existing quality-input admission mode");
    }
}

namespace
{
// Opaque vector payload identity, not a replacement numerical representation.
struct Vector3
{
    std::uint64_t identity{};
};
Vector3 vector3(std::uint64_t x, std::uint64_t, std::uint64_t) { return {x}; }
Vector3 operator-(Vector3, Vector3) { return {}; }
bool reference_different;
} // namespace
namespace formal_eskf::linalg
{
double max_abs(Vector3) { return reference_different ? 1.0 : 0.0; }
} // namespace formal_eskf::linalg
namespace
{
struct Context
{
    struct Source
    {
        unsigned reference_id{};
    } source;
    Vector3 m_n;
} supplied_context;
unsigned context_calls;
std::uint64_t expected_source;
bool context_ok;
Context read_magnetic_context(unsigned, std::uint64_t epoch, Context const &)
{
    ++context_calls;
    __ESBMC_assert(epoch == expected_source, "Magnetic context is selected at the measurement source time");
    if (!context_ok)
    {
        throw ReplayFailure{};
    }
    return supplied_context;
}
struct Magnetic
{
    std::uint64_t magnetic_row[4]{}, time{}, last_magnetic_time{}, magnetic_time{};
    unsigned mag_events{}, mag_superseded{}, magnetic_context_stream{};
    bool has_magnetometer{}, model_heading{};
    Vector3 magnetic_sample, seed_field;
    Context magnetic_context;
    struct Parameters
    {
        Vector3 m_n;
    } magnetic;
    Input magnetometer;
    void step()
    {
#include "replay_magnetic.inc"
    }
};
} // namespace

extern "C" void verify_replay_magnetic()
{
    Magnetic flow;
    flow.magnetic_row[0] = expected_source = nondet_time();
    flow.magnetic_row[1] = nondet_time();
    flow.time = nondet_time();
    flow.last_magnetic_time = nondet_time();
    flow.magnetic_time = nondet_bool() ? flow.last_magnetic_time : 0;
    flow.mag_events = nondet_unsigned();
    flow.mag_superseded = nondet_unsigned();
    flow.magnetic_sample.identity = nondet_time();
    flow.magnetic_context.m_n.identity = nondet_time();
    flow.magnetic_context.source.reference_id = nondet_unsigned();
    flow.magnetic.m_n.identity = nondet_time();
    flow.has_magnetometer = nondet_bool();
    flow.model_heading = nondet_bool();
    flow.magnetometer.next_available = nondet_bool();
    flow.magnetometer.next_time = nondet_time();
    supplied_context.source.reference_id = nondet_unsigned();
    supplied_context.m_n.identity = nondet_time();
    context_ok = nondet_bool();
    reference_different = nondet_bool();
    __ESBMC_assume(flow.last_magnetic_time != 0 && flow.last_magnetic_time <= flow.time && expected_source != 0);
    auto const before = flow;
    bool failed = false;
    try
    {
        flow.step();
    }
    catch (...)
    {
        failed = true;
    }
    bool const due = before.has_magnetometer && expected_source <= before.time;
    bool const admitted = due && expected_source > before.last_magnetic_time;
    bool const valid_reference =
        !before.model_heading || (!supplied_context.source.reference_id && !reference_different);
    bool const advanced = admitted && context_ok && valid_reference;
    __ESBMC_assert(failed == (due && !advanced) && context_calls == unsigned(admitted) &&
                       flow.magnetometer.advances == unsigned(advanced),
                   "Magnetic drain rejects disorder or context/reference failure before reading another event");
    __ESBMC_assert(
        flow.magnetic_time == (admitted ? expected_source : before.magnetic_time) &&
            flow.magnetic_sample.identity == (admitted ? before.magnetic_row[1] : before.magnetic_sample.identity) &&
            flow.mag_events == before.mag_events + unsigned(admitted) &&
            flow.mag_superseded == before.mag_superseded + unsigned(admitted && before.magnetic_time != 0),
        "Each newer magnetic event replaces the pending sample and accounts for supersession without fusion");
    __ESBMC_assert(flow.time == before.time && flow.last_magnetic_time <= flow.time &&
                       flow.last_magnetic_time == (advanced ? expected_source : before.last_magnetic_time),
                   "Magnetic source time remains separate from the current IMU fusion epoch");
    if (!failed)
    {
        __ESBMC_assert(flow.magnetic_time == 0 || flow.magnetic_time == flow.last_magnetic_time,
                       "Successful drain steps preserve the newest-pending-event invariant");
    }
    __ESBMC_assert(flow.magnetic.m_n.identity == (admitted && context_ok && !before.model_heading
                                                      ? supplied_context.m_n.identity
                                                      : before.magnetic.m_n.identity),
                   "Context supplies the field only when the geographic model does not own it");
    __ESBMC_assert(flow.has_magnetometer == (advanced ? before.magnetometer.next_available : before.has_magnetometer) &&
                       flow.magnetic_row[0] == (advanced ? before.magnetometer.next_time : before.magnetic_row[0]),
                   "Future magnetic events are retained; every successfully consumed event advances once");
}
