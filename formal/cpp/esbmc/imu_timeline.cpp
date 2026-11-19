// SPDX-License-Identifier: BSD-3-Clause

#include <cstddef>
#include <cstdint>

#include "../../../integrations/px4/module/DeploymentProfile.hpp"

// Contract-derived temporal induction. These are not replacement production
// bodies. Rejected operations preserve this projection by their source frame
// contracts, so identity closes their induction step. Success relations below
// retain all necessary guards. Every relation conjunct must be discharged by the matching actual
// owner behavior, complete frame, admission and typed-storage/reset proofs.
namespace
{
constexpr std::size_t capacity = 64;
static_assert(sizeof(std::size_t) == sizeof(std::uint64_t));
static_assert(formal_eskf::px4::profile::history::imu_capacity == capacity);
static_assert(formal_eskf::px4::profile::history::minimum_period_us == 3800);
static_assert(formal_eskf::px4::profile::history::maximum_period_us == 4200);
static_assert(formal_eskf::px4::profile::history::maximum_lag_us == 300000);
static_assert(formal_eskf::px4::profile::imu::maximum_interval_us == 50000);
struct Metadata
{
    std::size_t length;
    std::size_t count;
    std::size_t oldest;
    std::size_t write;
    std::size_t configured_length;
    std::uint64_t minimum_lag;
    std::uint64_t previous;
    std::uint64_t fusion;
    std::uint32_t angular_period;
    std::uint32_t acceleration_period;
};

struct AdmittedInput
{
    std::uint64_t time;
    std::uint32_t angular_period;
    std::uint32_t acceleration_period;
};

bool admitted_time(AdmittedInput input)
{
    // Necessary consequences only. Float, clipping, clock and source decisions
    // remain nondeterministic; this relation does not classify exact admission.
    return input.time > 0 && input.angular_period >= 3800 && input.angular_period <= 4200 &&
           input.acceleration_period >= 3800 && input.acceleration_period <= 4200;
}

std::size_t next(std::size_t index, std::size_t length) { return index + 1 == length ? 0 : index + 1; }
std::size_t newest(Metadata state)
{
    return state.count == 0 ? 0 : (state.write == 0 ? state.length - 1 : state.write - 1);
}

bool common(Metadata state)
{
    if (state.length > capacity || state.length == 1 || state.count > state.length ||
        state.configured_length != state.length)
    {
        return false;
    }
    if (state.length == 0)
    {
        return state.count == 0 && state.oldest == 0 && state.write == 0 && state.previous == 0 && state.fusion == 0;
    }
    if (state.oldest >= state.length || state.write >= state.length ||
        state.write != (state.oldest + state.count < state.length ? state.oldest + state.count
                                                                  : state.oldest + state.count - state.length))
    {
        return false;
    }
    if (state.previous == 0)
    {
        return state.count == 0 && state.fusion == 0;
    }
    return state.fusion > 0 && state.fusion <= state.previous && state.angular_period >= 3800 &&
           state.angular_period <= 4200 && state.acceleration_period >= 3800 && state.acceleration_period <= 4200 &&
           (state.count != 0 || state.fusion == state.previous);
}

bool slot(Metadata state, std::size_t index, std::uint64_t endpoint, std::uint64_t start, std::uint64_t input_endpoint,
          std::uint64_t predecessor)
{
    auto const rank = index >= state.oldest ? index - state.oldest : state.length - state.oldest + index;
    if (index >= state.length || rank >= state.count)
    {
        return true;
    }
    if (start == 0 || endpoint <= start || endpoint - start < 3800 || endpoint - start > 50000 ||
        endpoint > state.previous || input_endpoint != endpoint)
    {
        return false;
    }
    if (rank == 0)
    {
        if (state.fusion != start && !(state.count == state.length && state.fusion == endpoint))
        {
            return false;
        }
        if (state.count == state.length &&
            (state.previous - endpoint < state.minimum_lag || state.previous - endpoint > 300000))
        {
            return false;
        }
    }
    else if (start != predecessor)
    {
        return false;
    }
    return rank + 1 != state.count || endpoint == state.previous;
}

std::size_t predecessor_index(Metadata state, std::size_t index)
{
    return index > 0 ? index - 1 : (state.length > 0 ? state.length - 1 : 0);
}
} // namespace

void verify_imu_temporal_initialization(Metadata state, unsigned char observer, std::uint64_t endpoint,
                                        std::uint64_t start, std::uint64_t tag, std::uint64_t predecessor)
{
    __ESBMC_assume(observer < capacity);
    __ESBMC_assume(state.length == 0 && state.configured_length == 0 && state.count == 0 && state.oldest == 0 &&
                   state.write == 0 && state.previous == 0 && state.fusion == 0);
    __ESBMC_assert(common(state) && slot(state, observer, endpoint, start, tag, predecessor),
                   "Constructor contract establishes the temporal invariant");
}

// Read consistency is a consequence of a single backing array. Keeping only
// queried cells is an overapproximation of all 64 slots, not a smaller ring.
struct Cell
{
    std::uint64_t end;
    std::uint64_t start;
    std::uint64_t tag;
    std::uint64_t predecessor;
};

bool consistent(Metadata state, std::size_t i, Cell a, std::size_t j, Cell b)
{
    auto const pi = predecessor_index(state, i);
    auto const pj = predecessor_index(state, j);
    return (i != j || (a.end == b.end && a.start == b.start && a.tag == b.tag)) &&
           (pi != j || a.predecessor == b.end) && (pj != i || b.predecessor == a.end) &&
           (pi != pj || a.predecessor == b.predecessor);
}

bool consistent_endpoint(Metadata state, std::size_t i, Cell a, std::size_t j, std::uint64_t endpoint)
{
    return (i != j || a.end == endpoint) && (predecessor_index(state, i) != j || a.predecessor == endpoint);
}

void verify_imu_temporal_step(Metadata state, AdmittedInput input, unsigned char observer, Cell observed, Cell oldest,
                              Cell latest, std::uint64_t retained_end)
{
    __ESBMC_assume(observer < capacity && common(state));
    auto const newest_index = newest(state);
    __ESBMC_assume(slot(state, observer, observed.end, observed.start, observed.tag, observed.predecessor));
    __ESBMC_assume(slot(state, state.oldest, oldest.end, oldest.start, oldest.tag, oldest.predecessor));
    __ESBMC_assume(slot(state, newest_index, latest.end, latest.start, latest.tag, latest.predecessor));
    __ESBMC_assume(consistent(state, observer, observed, state.oldest, oldest) &&
                   consistent(state, observer, observed, newest_index, latest) &&
                   consistent(state, state.oldest, oldest, newest_index, latest));
#if FORMAL_ESKF_PROOF_COMPLETION == 0
    __ESBMC_assume(state.previous > 0 && admitted_time(input) && input.time > state.previous &&
                   input.time - state.previous >= 3800 && input.time - state.previous <= 50000);
    bool const full = state.count == state.length;
    if (full)
    {
        __ESBMC_assume(oldest.end <= state.fusion);
    }
    auto const retained = full ? next(state.oldest, state.length) : state.oldest;
    __ESBMC_assume(consistent_endpoint(state, observer, observed, retained, retained_end) &&
                   consistent_endpoint(state, state.oldest, oldest, retained, retained_end) &&
                   consistent_endpoint(state, newest_index, latest, retained, retained_end));
    if (state.count + 1 >= state.length)
    {
        auto const lag = input.time - retained_end;
        __ESBMC_assume(lag >= state.minimum_lag && lag <= 300000);
    }
    // Exact typed-storage/frame equations, observed at an arbitrary index.
    if (observer == state.write)
    {
        observed.end = input.time;
        observed.start = state.previous;
        observed.tag = input.time;
    }
    if (predecessor_index(state, observer) == state.write)
    {
        observed.predecessor = input.time;
    }
    state.oldest = retained;
    state.write = next(state.write, state.length);
    if (!full)
    {
        ++state.count;
    }
    state.previous = input.time;
    state.angular_period = input.angular_period;
    state.acceleration_period = input.acceleration_period;
#elif FORMAL_ESKF_PROOF_COMPLETION == 1
    __ESBMC_assume(state.length > 0 && state.count == state.length && oldest.end > state.fusion &&
                   oldest.start == state.fusion);
    state.fusion = oldest.end;
#else
#error Expected push or completion
#endif
    __ESBMC_assert(common(state), "Owner contracts preserve temporal metadata");
    __ESBMC_assume(common(state));
    __ESBMC_assert(slot(state, observer, observed.end, observed.start, observed.tag, observed.predecessor),
                   "Owner contracts preserve each chronological slot");
}

void verify_imu_temporal_views(Metadata state, std::uint64_t oldest_end, std::uint64_t oldest_start,
                               std::uint64_t oldest_tag, std::uint64_t oldest_predecessor, std::uint64_t newest_end,
                               std::uint64_t newest_start, std::uint64_t newest_tag, std::uint64_t newest_predecessor)
{
    __ESBMC_assume(common(state));
    __ESBMC_assume(slot(state, state.oldest, oldest_end, oldest_start, oldest_tag, oldest_predecessor) &&
                   slot(state, newest(state), newest_end, newest_start, newest_tag, newest_predecessor));
    if (state.count > 0)
    {
        __ESBMC_assert(newest_tag == state.previous && oldest_tag <= state.previous,
                       "Retained latest and oldest inputs cannot advance the previous endpoint");
    }
}

void verify_imu_temporal_configure(Metadata state, std::size_t candidate_length, std::uint64_t candidate_lag)
{
    __ESBMC_assume(common(state));
    __ESBMC_assume(state.previous == 0 && candidate_length >= 2 && candidate_length <= capacity);
    if (candidate_length != state.length)
    {
        state.count = 0;
        state.oldest = 0;
        state.write = 0;
    }
    state.length = candidate_length;
    state.configured_length = candidate_length;
    state.minimum_lag = candidate_lag;
    __ESBMC_assert(common(state) && state.count == 0, "Configure preserves the empty temporal metadata");
}
void verify_imu_temporal_empty(Metadata state, unsigned char observer, std::uint64_t endpoint, std::uint64_t start,
                               std::uint64_t tag, std::uint64_t predecessor)
{
    __ESBMC_assume(observer < capacity && common(state) && state.count == 0);
    __ESBMC_assert(slot(state, observer, endpoint, start, tag, predecessor),
                   "Empty temporal metadata entails every slot clause independently of payloads");
}
void verify_imu_temporal_seed(Metadata state, AdmittedInput input)
{
    __ESBMC_assume(common(state) && state.previous == 0 && state.length > 0 && admitted_time(input));
    state.previous = input.time;
    state.fusion = input.time;
    state.angular_period = input.angular_period;
    state.acceleration_period = input.acceleration_period;
    __ESBMC_assert(common(state) && state.count == 0, "Seed preserves the empty temporal metadata");
}
void verify_imu_temporal_reset(Metadata state)
{
    __ESBMC_assume(common(state));
    state.count = 0;
    state.oldest = 0;
    state.write = 0;
    state.previous = 0;
    state.fusion = 0;
    state.angular_period = 0;
    state.acceleration_period = 0;
    __ESBMC_assert(common(state) && state.count == 0, "Reset establishes the empty temporal metadata");
}
