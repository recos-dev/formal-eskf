/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "contracts.hpp"
// Widen access only for this already dependency-loaded header. This exposes
// the actual private producer to a separate proof; no body/layout is rewritten.
#define private public
#include <formal_eskf/runtime/vertical_acceleration_health.hpp>
#undef private
#ifndef FORMAL_ESKF_PROOF_HEALTH_CALLER
#define FORMAL_ESKF_PROOF_HEALTH_CALLER 0
#endif

using contract_proof::same;
using contract_proof::Scalar;
using formal_eskf::Status;

// All scalar-library returns are admitted, independently at every call.
// The frame proof claims only enum range, atomic output and input purity;
// source threshold/reference equations remain a separate, unaccepted obligation.
extern Scalar nondet_health_root();
struct HealthMath : formal_eskf::scalar::StandardMath<Scalar>
{
    struct Root
    {
        Scalar operator()(Scalar) { return nondet_health_root(); }
    };
    inline static Root sqrt{};
};
struct Profile
{
    using value_type = Scalar;
    using scalar_math_type = HealthMath;
};
using Monitor = formal_eskf::runtime::VerticalAccelerationHealth<Profile>;
using History = Monitor::State;
using Likelihood = Monitor::Likelihood;

bool same_history(History const & a, History const & b)
{
    return a.likelihood == b.likelihood && a.clip_count == b.clip_count && a.bad_clipping == b.bad_clipping &&
           a.bad_sample == b.bad_sample && a.bad_vertical_acceleration == b.bad_vertical_acceleration &&
           a.last_bad_time_us == b.last_bad_time_us && a.last_good_time_us == b.last_good_time_us &&
           a.last_update_time_us == b.last_update_time_us;
}
bool clock_invariant(History const & state)
{
    return state.last_bad_time_us <= state.last_update_time_us &&
           state.last_good_time_us <= state.last_update_time_us &&
           (state.last_bad_time_us == state.last_update_time_us ||
            state.last_good_time_us == state.last_update_time_us) &&
           (state.last_bad_time_us == 0U || state.last_bad_time_us != state.last_good_time_us) &&
           state.bad_sample ==
               (state.last_update_time_us != 0U && state.last_bad_time_us == state.last_update_time_us) &&
           state.bad_vertical_acceleration ==
               (state.last_bad_time_us != 0U && state.last_update_time_us - state.last_bad_time_us < 3'000'000U);
}

bool same_observations(Monitor::Observations const & a, Monitor::Observations const & b)
{
    std::array<Monitor::Innovation, 6U> const left{a.barometer_height, a.gnss_height,   a.gnss_velocity_down,
                                                   a.range_height,     a.vision_height, a.vision_velocity_down};
    std::array<Monitor::Innovation, 6U> const right{b.barometer_height, b.gnss_height,   b.gnss_velocity_down,
                                                    b.range_height,     b.vision_height, b.vision_velocity_down};
    for (std::size_t i = 0U; i < 6U; ++i)
    {
        if (left[i].active != right[i].active || !same(left[i].innovation, right[i].innovation) ||
            !same(left[i].variance, right[i].variance))
        {
            return false;
        }
    }
    return true;
}
struct FallingResult
{
    Status status;
    Likelihood likelihood;
};
struct FallingCall
{
    inline static FallingResult result{};
    inline static Monitor::Observations input{};
    inline static unsigned calls = 0U;
};
#if FORMAL_ESKF_PROOF_HEALTH_CALLER
namespace formal_eskf::runtime
{
template <>
inline Status VerticalAccelerationHealth<Profile>::falling_likelihood(Observations const & observations,
                                                                      Likelihood & output)
{
    __ESBMC_assert(FallingCall::calls++ == 0U && same_observations(observations, FallingCall::input),
                   "vertical health caller: exactly one original observation request after validation");
    // Failure may dirty the candidate: the caller must not publish it.
    output = FallingCall::result.likelihood;
    return FallingCall::result.status;
}
} // namespace formal_eskf::runtime
#endif
#if !FORMAL_ESKF_PROOF_HEALTH_CALLER
void verify_vertical_likelihood_frame(Monitor::Observations observations, Likelihood output)
{
    auto const before = observations;
    auto const old_output = output;
    auto const status = Monitor::falling_likelihood(observations, output);
    __ESBMC_assert(
        status == Status::success
            ? (output == Likelihood::low || output == Likelihood::medium || output == Likelihood::high)
            : output == old_output,
        "R-VERTICAL-ACCELERATION: actual likelihood producer returns a declared verdict or preserves output");
    __ESBMC_assert(same_observations(observations, before),
                   "R-VERTICAL-ACCELERATION: actual likelihood producer preserves every observation input");
}
#endif
void verify_vertical_initialization(History arbitrary)
{
    Monitor monitor;
    __ESBMC_assert(same_history(monitor.state(), History{}) && clock_invariant(monitor.state()),
                   "R-VERTICAL-ACCELERATION: constructor establishes empty history and clock invariant");
    // The underlying object is mutable. Seed the published history for induction
    // without rewriting update() or assuming that arbitrary prior history is reachable.
    const_cast<History &>(monitor.state()) = arbitrary;
    monitor.reset();
    __ESBMC_assert(same_history(monitor.state(), History{}) && clock_invariant(monitor.state()),
                   "R-VERTICAL-ACCELERATION: reset establishes the base case from any history");
}

#if FORMAL_ESKF_PROOF_HEALTH_CALLER
void verify_vertical_transition(History prior, Monitor::Sample sample, Monitor::Observations observations,
                                FallingResult falling)
{
    __ESBMC_assume(clock_invariant(prior));
    __ESBMC_assume(prior.likelihood == Likelihood::low || prior.likelihood == Likelihood::medium ||
                   prior.likelihood == Likelihood::high);
    auto const before_sample = sample;
    auto const before_observations = observations;
    Monitor monitor;
    const_cast<History &>(monitor.state()) = prior;
    __ESBMC_assume(falling.likelihood == Likelihood::low || falling.likelihood == Likelihood::medium ||
                   falling.likelihood == Likelihood::high);
    FallingCall::result = falling;
    FallingCall::input = observations;
    FallingCall::calls = 0U;
    History expected = prior;
    Status expected_status = Status::success;
    Scalar reciprocal{};
    if (sample.time_us == 0U || sample.time_us <= prior.last_update_time_us)
    {
        expected_status = Status::out_of_range;
    }
    else if (!std::isfinite(sample.average_interval_s))
    {
        expected_status = Status::non_finite_input;
    }
    else if (sample.average_interval_s <= Scalar{0} || !sample.clipping_available)
    {
        expected_status = Status::domain_error;
    }
    else
    {
        reciprocal = Scalar{1} / sample.average_interval_s;
        expected_status = !std::isfinite(reciprocal)                              ? Status::non_finite_result
                          : reciprocal < Scalar{2} || reciprocal >= Scalar{65536} ? Status::out_of_range
                                                                                  : Status::success;
    }

    bool const call_expected = expected_status == Status::success;
    if (call_expected)
    {
        expected_status = falling.status;
    }
    if (expected_status == Status::success)
    {
        expected.likelihood = falling.likelihood;
        bool const high = expected.likelihood == Likelihood::high;
        bool const medium = expected.likelihood == Likelihood::medium;
        auto const count_limit = static_cast<std::uint16_t>(reciprocal);
        unsigned warnings = 0U, critical = 0U;
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            auto const old = prior.clip_count[axis];
            int const change = sample.clipped[axis] && old < count_limit ? 1 : old > 0U ? -1 : 0;
            expected.clip_count[axis] = static_cast<std::uint16_t>(static_cast<int>(old) + change);
            warnings += expected.clip_count[axis] >= count_limit / 2U;
            critical += expected.clip_count[axis] >= count_limit;
        }
        expected.bad_clipping = critical != 0U || warnings == 3U;
        bool const frequent = expected.bad_clipping || expected.clip_count[2U] >= count_limit / 2U;
        expected.bad_sample = high || (medium && frequent);
        expected.last_bad_time_us = expected.bad_sample ? sample.time_us : prior.last_bad_time_us;
        expected.last_good_time_us = expected.bad_sample ? prior.last_good_time_us : sample.time_us;
        expected.last_update_time_us = sample.time_us;
        expected.bad_vertical_acceleration =
            expected.last_bad_time_us != 0U && sample.time_us - expected.last_bad_time_us < 3'000'000U;
    }
    auto const status = monitor.update(sample, observations);
    __ESBMC_assert(
        status == expected_status && same_history(monitor.state(), expected),
        "R-VERTICAL-ACCELERATION: ordered validation, verdict consumption, counter recurrence and atomic history");
    __ESBMC_assert(clock_invariant(monitor.state()),
                   "R-VERTICAL-ACCELERATION: all update outcomes preserve the inductive clock/hold invariant");

    __ESBMC_assert(FallingCall::calls == static_cast<unsigned>(call_expected),
                   "R-VERTICAL-ACCELERATION: caller checks domain before observation evaluation");
    __ESBMC_assert(sample.time_us == before_sample.time_us &&
                       same(sample.average_interval_s, before_sample.average_interval_s) &&
                       sample.clipping_available == before_sample.clipping_available &&
                       sample.clipped == before_sample.clipped && same_observations(observations, before_observations),
                   "R-VERTICAL-ACCELERATION: update preserves every input");
}
#endif
int main() { return 0; }
