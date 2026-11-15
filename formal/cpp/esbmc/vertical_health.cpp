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

// Every IEEE root return is admitted independently. The optional observer only
// records scalar calls; it never changes a primitive's return or production state.
extern Scalar nondet_health_root();
struct HealthMath : formal_eskf::scalar::StandardMath<Scalar>
{
    struct Root
    {
        Scalar operator()(Scalar value);
    };
    inline static Root sqrt{};
    static bool is_finite(Scalar value);
};
struct Profile
{
    using value_type = Scalar;
    using scalar_math_type = HealthMath;
};
using Monitor = formal_eskf::runtime::VerticalAccelerationHealth<Profile>;
using History = Monitor::State;
using Likelihood = Monitor::Likelihood;

// 0: protocol/failure; 1: classification; 2: all source IEEE ratios.
// 3 retains all assertions for native mutation witnesses.
#ifndef FORMAL_ESKF_PROOF_HEALTH_PROPERTY
#define FORMAL_ESKF_PROOF_HEALTH_PROPERTY 3
#endif
static_assert(FORMAL_ESKF_PROOF_HEALTH_PROPERTY >= 0 && FORMAL_ESKF_PROOF_HEALTH_PROPERTY <= 3);

void check_likelihood_protocol(bool condition, char const * message)
{
#if FORMAL_ESKF_PROOF_HEALTH_PROPERTY == 0 || FORMAL_ESKF_PROOF_HEALTH_PROPERTY == 3
    __ESBMC_assert(condition, message);
#else
    // Property 0 proves these exact prefix invariants with no assumptions.
    // Consumers reuse that same-type producer. No verdict or quotient equality
    // is assumed here; those are the separate assertions below.
    (void)message;
    __ESBMC_assume(condition);
#endif
}

bool same_likelihood_scalar(Scalar a, Scalar b)
{
    using Bits = std::conditional_t<sizeof(Scalar) == 4, std::uint32_t, std::uint64_t>;
    return __builtin_bit_cast(Bits, a) == __builtin_bit_cast(Bits, b) || (std::isnan(a) && std::isnan(b));
}
struct LikelihoodTrace
{
    inline static bool enabled = false;
    inline static bool active[6]{};
    inline static Scalar innovations[6]{}, variances[6]{};
    inline static bool minimum[6]{}, limit[6]{};
    enum class Stage
    {
        innovation,
        variance,
        checked_variance,
        root_call,
        root_result,
        ratio,
    };
    inline static unsigned cursor{};
    inline static unsigned next_source[6]{};
    inline static Stage stage{};
    inline static Scalar root{}, consumed_innovation{};
    inline static Status status{};
};
Scalar HealthMath::Root::operator()(Scalar value)
{
    if (!LikelihoodTrace::enabled)
    {
        return nondet_health_root();
    }
    check_likelihood_protocol(LikelihoodTrace::cursor < 6 &&
                                  LikelihoodTrace::stage == LikelihoodTrace::Stage::root_call &&
                                  LikelihoodTrace::status == Status::success,
                              "root order");
    check_likelihood_protocol(same_likelihood_scalar(value, LikelihoodTrace::variances[LikelihoodTrace::cursor]),
                              "root argument");
    LikelihoodTrace::stage = LikelihoodTrace::Stage::root_result;
    LikelihoodTrace::root = nondet_health_root();
    return LikelihoodTrace::root;
}
bool HealthMath::is_finite(Scalar value)
{
    if (!LikelihoodTrace::enabled)
    {
        return std::isfinite(value);
    }
    bool const valid = std::isfinite(value);
    check_likelihood_protocol(LikelihoodTrace::status == Status::success, "no consumption after first failure");
    check_likelihood_protocol(LikelihoodTrace::cursor < 6, "all checks belong to active source");
    Scalar const innovation = LikelihoodTrace::innovations[LikelihoodTrace::cursor];
    Scalar const variance = LikelihoodTrace::variances[LikelihoodTrace::cursor];
    if (LikelihoodTrace::stage == LikelihoodTrace::Stage::innovation)
    {
        check_likelihood_protocol(same_likelihood_scalar(value, innovation), "source innovation order");
        LikelihoodTrace::consumed_innovation = value;
        if (!valid)
        {
            LikelihoodTrace::status = Status::non_finite_input;
        }
        LikelihoodTrace::stage = LikelihoodTrace::Stage::variance;
    }
    else if (LikelihoodTrace::stage == LikelihoodTrace::Stage::variance)
    {
        check_likelihood_protocol(same_likelihood_scalar(value, variance), "source variance order");
        if (!valid)
        {
            LikelihoodTrace::status = Status::non_finite_input;
        }
        else if (value < Scalar{0})
        {
            LikelihoodTrace::status = Status::domain_error;
        }
        else if (value < static_cast<Scalar>(std::numeric_limits<float>::epsilon()))
        {
            LikelihoodTrace::cursor = LikelihoodTrace::next_source[LikelihoodTrace::cursor];
            LikelihoodTrace::stage = LikelihoodTrace::Stage::innovation;
        }
        else
        {
            LikelihoodTrace::stage = LikelihoodTrace::Stage::checked_variance;
        }
    }
    else if (LikelihoodTrace::stage == LikelihoodTrace::Stage::checked_variance)
    {
        check_likelihood_protocol(same_likelihood_scalar(value, variance), "checked root revalidates variance");
        LikelihoodTrace::stage = LikelihoodTrace::Stage::root_call;
    }
    else if (LikelihoodTrace::stage == LikelihoodTrace::Stage::root_result)
    {
        check_likelihood_protocol(same_likelihood_scalar(value, LikelihoodTrace::root),
                                  "checked root validates raw result");
        if (!valid)
        {
            LikelihoodTrace::status = Status::non_finite_result;
        }
        LikelihoodTrace::stage = LikelihoodTrace::Stage::ratio;
    }
    else
    {
        check_likelihood_protocol(LikelihoodTrace::stage == LikelihoodTrace::Stage::ratio,
                                  "ratio follows checked root");
#if FORMAL_ESKF_PROOF_HEALTH_PROPERTY == 2 || FORMAL_ESKF_PROOF_HEALTH_PROPERTY == 3
        __ESBMC_assert(same_likelihood_scalar(value, LikelihoodTrace::consumed_innovation / LikelihoodTrace::root),
                       "exact signed source innovation / root ratio");
#endif
        if (!valid)
        {
            LikelihoodTrace::status = Status::non_finite_result;
        }
        LikelihoodTrace::minimum[LikelihoodTrace::cursor] = value > Scalar{1};
        LikelihoodTrace::limit[LikelihoodTrace::cursor] = value > Scalar{3};
        LikelihoodTrace::cursor = LikelihoodTrace::next_source[LikelihoodTrace::cursor];
        LikelihoodTrace::stage = LikelihoodTrace::Stage::innovation;
    }
    return valid;
}
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
void verify_vertical_likelihood(Monitor::Observations observations, Monitor::Likelihood output)
{
    LikelihoodTrace::enabled = true;
    LikelihoodTrace::active[0] = observations.barometer_height.active;
    LikelihoodTrace::innovations[0] = observations.barometer_height.innovation;
    LikelihoodTrace::variances[0] = observations.barometer_height.variance;
    LikelihoodTrace::active[1] = observations.gnss_height.active;
    LikelihoodTrace::innovations[1] = observations.gnss_height.innovation;
    LikelihoodTrace::variances[1] = observations.gnss_height.variance;
    LikelihoodTrace::active[2] = observations.gnss_velocity_down.active;
    LikelihoodTrace::innovations[2] = observations.gnss_velocity_down.innovation;
    LikelihoodTrace::variances[2] = observations.gnss_velocity_down.variance;
    LikelihoodTrace::active[3] = observations.range_height.active;
    LikelihoodTrace::innovations[3] = observations.range_height.innovation;
    LikelihoodTrace::variances[3] = observations.range_height.variance;
    LikelihoodTrace::active[4] = observations.vision_height.active;
    LikelihoodTrace::innovations[4] = observations.vision_height.innovation;
    LikelihoodTrace::variances[4] = observations.vision_height.variance;
    LikelihoodTrace::active[5] = observations.vision_velocity_down.active;
    LikelihoodTrace::innovations[5] = observations.vision_velocity_down.innovation;
    LikelihoodTrace::variances[5] = observations.vision_velocity_down.variance;
    for (unsigned i = 0; i < 6; ++i)
    {
        LikelihoodTrace::minimum[i] = false;
        LikelihoodTrace::limit[i] = false;
    }
    // Next active source depends only on the original activity flags. Build it
    // once; floating-point validation must never change which source comes next.
    unsigned next = 6;
    for (unsigned i = 6; i > 0; --i)
    {
        LikelihoodTrace::next_source[i - 1] = next;
        if (LikelihoodTrace::active[i - 1])
        {
            next = i - 1;
        }
    }
    LikelihoodTrace::cursor = next;
    LikelihoodTrace::stage = LikelihoodTrace::Stage::innovation;
    LikelihoodTrace::status = Status::success;
    auto const old = output;
    auto const status = Monitor::falling_likelihood(observations, output);
    check_likelihood_protocol(status == LikelihoodTrace::status, "first failure matches source validation");
    if (status != Status::success)
    {
        check_likelihood_protocol(output == old, "failure atomicity");
        return;
    }
    check_likelihood_protocol(LikelihoodTrace::cursor == 6 &&
                                  LikelihoodTrace::stage == LikelihoodTrace::Stage::innovation,
                              "all active inputs validated before verdict");
#if FORMAL_ESKF_PROOF_HEALTH_PROPERTY == 1 || FORMAL_ESKF_PROOF_HEALTH_PROPERTY == 3
    // Build reference membership from the six independent source records.
    // A high verdict requires a >3 source and a >1 source in another group.
    bool const minimum[3]{LikelihoodTrace::minimum[0], LikelihoodTrace::minimum[1] || LikelihoodTrace::minimum[2],
                          LikelihoodTrace::minimum[3] || LikelihoodTrace::minimum[4] || LikelihoodTrace::minimum[5]};
    bool const limit[3]{LikelihoodTrace::limit[0], LikelihoodTrace::limit[1] || LikelihoodTrace::limit[2],
                        LikelihoodTrace::limit[3] || LikelihoodTrace::limit[4] || LikelihoodTrace::limit[5]};
    bool const high = (limit[0] && (minimum[1] || minimum[2])) || (limit[1] && (minimum[0] || minimum[2])) ||
                      (limit[2] && (minimum[0] || minimum[1]));
    bool const medium = limit[0] || limit[1] || limit[2];
    auto const expected = high     ? Monitor::Likelihood::high
                          : medium ? Monitor::Likelihood::medium
                                   : Monitor::Likelihood::low;
    __ESBMC_assert(output == expected, "strict signed thresholds and distinct reference pairing");
#endif
}

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
