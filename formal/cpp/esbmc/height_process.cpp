// SPDX-License-Identifier: BSD-3-Clause

#include "height_transaction_support.hpp"

#ifndef FORMAL_ESKF_PROOF_HEIGHT_SOURCE
#define FORMAL_ESKF_PROOF_HEIGHT_SOURCE 0
#endif

namespace height_process_proof
{
using namespace height_transaction_proof;
using Measurement = formal_eskf::runtime::NavigationFusionResult<Backend, 1U>;
constexpr Source source = static_cast<Source>(FORMAL_ESKF_PROOF_HEIGHT_SOURCE);
constexpr bool barometer = source == Source::barometer;
static_assert(FORMAL_ESKF_PROOF_HEIGHT_SOURCE == 0 || FORMAL_ESKF_PROOF_HEIGHT_SOURCE == 1);

// These fields are outside every summarized producer's write footprint.
void retain_constants(Manager & candidate, Manager const & original)
{
    candidate.m_origin_altitude = original.m_origin_altitude;
    candidate.m_started = original.m_started;
    candidate.m_tick = original.m_tick;
}

struct Management
{
    Manager manager;
    Result result;
};
struct Phase
{
    Manager manager;
    State state;
    Covariance covariance;
    Result result;
};
struct Calls
{
    inline static Management const * admitted = nullptr;
    inline static Management const * prepared = nullptr;
    inline static Phase const * reset = nullptr;
    inline static Phase const * recovered = nullptr;
    inline static Manager before_recovery{};
    inline static Result recovery_result{};
    inline static State const * recovery_state = nullptr;
    inline static Covariance const * recovery_covariance = nullptr;
    inline static Manager initial{}, ready{};
    inline static Result initial_result{}, ready_result{};
    inline static Sample sample{};
    inline static State prior{}, fused{}, limited{};
    inline static Covariance covariance{}, fused_covariance{};
    inline static Measurement measurement{};
    inline static Bias bias{};
    inline static Permissions permissions{};
    inline static Acceleration acceleration{};
    inline static std::uint64_t now{};
    inline static Scalar variance{}, observation{}, observation_variance{};
    inline static Status limit_status{};
    inline static bool admit{}, prepare{}, velocity{}, ground{};
    inline static unsigned admissions{}, preparations{}, biases{}, resets{}, fusions{}, limits{}, recoveries{},
        reports{};
    inline static double reported{};
};
} // namespace height_process_proof

// Each summary is a separate producer boundary: admission/filter, bias equation,
// reset/recovery, numerical fusion and bias limiting. Arbitrary outputs retain
// failure paths. This file proves their actual process() caller, not the producer.
namespace formal_eskf::px4
{
inline void InnovationRatio::update(std::uint64_t time, double innovation, double ratio)
{
    using namespace height_process_proof;
    auto const & prior = Calls::ready.m_sessions[FORMAL_ESKF_PROOF_HEIGHT_SOURCE].ratio;
    auto const & fusion = Calls::measurement.fusion;
    double const expected_innovation = -fusion.innovation(0U);
    double const expected_ratio = fusion.diagnostics_valid ? double(fusion.test_ratio(0U)) : double(NAN);
    auto same_double = [](double a, double b)
    {
        return (__builtin_bit_cast(std::uint64_t, a) == __builtin_bit_cast(std::uint64_t, b)) |
               (std::isnan(a) & std::isnan(b));
    };
    __ESBMC_assert(Calls::reports++ == 0U && time == Calls::sample.time_us && same_ratio(*this, prior) &&
                       same_double(innovation, expected_innovation) && same_double(ratio, expected_ratio),
                   "Height process: reporting receives its original history and exact raw gate diagnostics");
    // The actual reporting producer establishes its supplied epoch and modifies
    // only these fields. Every IEEE result remains admitted at this boundary.
    time_us = time;
    filtered = Calls::reported;
}
template <>
inline bool HeightAiding<height_process_proof::Backend>::admit(Source selected, std::uint64_t now,
                                                               Sample const & sample, Result & result)
{
    using namespace height_process_proof;
    __ESBMC_assert(Calls::admissions++ == 0U && selected == source && now == Calls::now &&
                       same_sample(sample, Calls::sample),
                   "Height process: source admission receives the original source, epoch and complete sample");
    check_manager(*this, Calls::initial);
    check_result(result, Calls::initial_result);
    *this = Calls::admitted->manager;
    result = Calls::admitted->result;
    return Calls::admit;
}
template <>
inline bool HeightAiding<height_process_proof::Backend>::prepare_barometer(Sample const & sample, value_type altitude,
                                                                           Result & result)
{
    using namespace height_process_proof;
    __ESBMC_assert(barometer && Calls::admit && Calls::preparations++ == 0U && same_sample(sample, Calls::sample) &&
                       same(altitude, Calls::admitted->manager.m_origin_altitude - Calls::prior.p_n(2U)),
                   "Height process: barometer warmup uses the original state altitude after admission");
    check_manager(*this, Calls::admitted->manager);
    check_result(result, Calls::admitted->result);
    *this = Calls::prepared->manager;
    result = Calls::prepared->result;
    return Calls::prepare;
}
template <>
inline void HeightBias<height_process_proof::Scalar>::fuse(height_process_proof::Scalar measurement,
                                                           height_process_proof::Scalar noise,
                                                           height_process_proof::Scalar sensor)
{
    using namespace height_process_proof;
    auto const & ready = Calls::ready;
    auto const & aid = ready.m_sessions[FORMAL_ESKF_PROOF_HEIGHT_SOURCE];
    __ESBMC_assert(Calls::biases++ == 0U && aid.active && ready.m_reference != source && same_bias(*this, aid.bias) &&
                       same(measurement, Calls::sample.altitude -
                                             (Calls::admitted->manager.m_origin_altitude - Calls::prior.p_n(2U))) &&
                       same(noise, Calls::variance + Calls::covariance(2U, 2U)) && same(sensor, Calls::variance) &&
                       std::isfinite(sensor) && sensor > Scalar{0},
                   "Height process: only an active nonreference learns bias, with admitted positive sensor variance");
    Scalar const prior_interval = dt;
    *this = Calls::bias;
    dt = prior_interval;
}
template <>
inline void HeightAiding<height_process_proof::Backend>::reset_height(Source selected, std::uint64_t now,
                                                                      value_type altitude, value_type variance,
                                                                      state_type & state, covariance_type & P,
                                                                      Result & result)
{
    using namespace height_process_proof;
    check_state(state, Calls::prior);
    check_matrix(P, Calls::covariance);
    __ESBMC_assert(
        !barometer && Calls::resets++ == 0U && selected == source && now == Calls::now &&
            same(altitude, Calls::sample.altitude) && same(variance, Calls::variance),
        "Height process: GNSS startup resets the current candidate using raw altitude and selected variance");
    check_manager(*this, Calls::ready);
    check_result(result, Calls::ready_result);
    *this = Calls::reset->manager;
    state = Calls::reset->state;
    P = Calls::reset->covariance;
    result = Calls::reset->result;
}
template <>
inline void HeightAiding<height_process_proof::Backend>::recover(Source selected, std::uint64_t now,
                                                                 Sample const & sample, value_type variance,
                                                                 bool velocity, state_type & state, covariance_type & P,
                                                                 Result & result, Acceleration const & acceleration)
{
    using namespace height_process_proof;
    auto const & expected = Calls::before_recovery;
    __ESBMC_assert(Calls::recoveries++ == 0U && selected == source && now == Calls::now &&
                       same_sample(sample, Calls::sample) && same(variance, Calls::variance) &&
                       velocity == Calls::velocity && acceleration.bad == Calls::acceleration.bad &&
                       acceleration.times.update_time_us == Calls::acceleration.times.update_time_us &&
                       acceleration.times.last_good_time_us == Calls::acceleration.times.last_good_time_us,
                   "Height process: recovery follows successful fusion status and receives complete accounting/health");
    check_state(state, *Calls::recovery_state);
    check_matrix(P, *Calls::recovery_covariance);
    check_manager(*this, expected);
    check_result(result, Calls::recovery_result);
    *this = Calls::recovered->manager;
    state = Calls::recovered->state;
    P = Calls::recovered->covariance;
    result = Calls::recovered->result;
}
} // namespace formal_eskf::px4

namespace formal_eskf::runtime
{
template <>
inline NavigationFusionResult<height_process_proof::Backend, 1U>
fuse_height(height_process_proof::State const & state, height_process_proof::Covariance const & P,
            height_process_proof::Scalar observation, height_process_proof::Scalar variance,
            height_process_proof::Scalar minimum, height_process_proof::Scalar gate, bool bad,
            height_process_proof::State & output, height_process_proof::Covariance & covariance,
            configuration::Ins::BiasUpdate const & permissions, height_process_proof::Scalar deadzone)
{
    using namespace height_process_proof;
    check_state(state, Calls::prior);
    check_matrix(P, Calls::covariance);
    __ESBMC_assert(
        Calls::fusions++ == 0U && &state == &output && &P == &covariance && same(observation, Calls::observation) &&
            same(variance, Calls::observation_variance) &&
            same(minimum, static_cast<Scalar>(px4::profile::minimum_quaternion_norm)) &&
            same(gate, static_cast<Scalar>(px4::profile::height::gate_sigma)) && bad == Calls::acceleration.bad &&
            bias_proof::same_permissions(permissions, Calls::permissions) &&
            same(deadzone, barometer && Calls::ground
                               ? static_cast<Scalar>(px4::profile::height::ground_effect_deadzone_m)
                               : Scalar{0}),
        "Height process: fusion freezes PRIOR bias, forwards complete permissions and limits deadzone to barometer");
    output = Calls::fused;
    covariance = Calls::fused_covariance;
    return Calls::measurement;
}
template <>
inline Status limit_bias(height_process_proof::State & state, BiasLimits<height_process_proof::Scalar> limits)
{
    using namespace height_process_proof;
    auto const expected = px4::profile::bias_limits<Scalar>();
    check_state(state, Calls::fused);
    __ESBMC_assert(Calls::limits++ == 0U && same(limits.accelerometer, expected.accelerometer) &&
                       same(limits.gyroscope, expected.gyroscope),
                   "Height process: successful fusion invokes deployment bias limits before recording progress");
    state = Calls::limited;
    return Calls::limit_status;
}
} // namespace formal_eskf::runtime

using namespace height_process_proof;

void verify_height_process(Manager owner, State state, Covariance P, Sample sample, std::uint64_t now, Result result,
                           Permissions permissions, Acceleration acceleration, bool velocity, bool ground,
                           Management admitted, Management prepared, Phase reset, Phase recovered,
                           Measurement measurement, State fused, Covariance fused_P, State limited, Status limit_status,
                           Bias bias, bool admit, bool prepare, double reported)
{
    // The independently checked admission producer establishes this on true.
    __ESBMC_assume(!admit || (sample.time_us != 0U && sample.time_us <= now && altitude_valid(sample.altitude) &&
                              std::isfinite(sample.variance) && sample.variance > Scalar{0}));
    // Producer frame contracts retain these fields. Construct the summaries
    // with the original values rather than independent symbols constrained by
    // floating-point equality; all mutable producer outputs remain arbitrary.
    retain_constants(admitted.manager, owner);
    retain_constants(prepared.manager, owner);
    retain_constants(reset.manager, owner);
    retain_constants(recovered.manager, owner);
    __ESBMC_assume(history_domain(owner, now) && history_domain(admitted.manager, now) &&
                   history_domain(prepared.manager, now) && history_domain(reset.manager, now) &&
                   history_domain(recovered.manager, now) && bias_domain(bias));
    bool const control = control_domain(owner);
    __ESBMC_assume(!control || (control_domain(admitted.manager) && control_domain(prepared.manager) &&
                                control_domain(reset.manager) && control_domain(recovered.manager)));
    auto const before = owner;
    Calls::initial = owner;
    Calls::initial_result = result;
    Calls::admitted = &admitted;
    Calls::prepared = &prepared;
    Calls::reset = &reset;
    Calls::recovered = &recovered;
    Calls::sample = sample;
    Calls::prior = state;
    Calls::covariance = P;
    Calls::permissions = permissions;
    Calls::acceleration = acceleration;
    Calls::now = now;
    Calls::velocity = velocity;
    Calls::ground = ground;
    Calls::admit = admit;
    Calls::prepare = prepare;
    Calls::measurement = measurement;
    Calls::fused = fused;
    Calls::fused_covariance = fused_P;
    Calls::limited = limited;
    Calls::limit_status = limit_status;
    Calls::bias = bias;
    Calls::admissions = Calls::preparations = Calls::biases = Calls::resets = Calls::fusions = Calls::limits =
        Calls::recoveries = Calls::reports = 0U;
    Calls::reported = reported;
    Manager expected = admitted.manager;
    Result expected_result = admitted.result;
    State const * expected_state = &Calls::prior;
    Covariance const * expected_covariance = &Calls::covariance;
    if (admit && barometer)
    {
        expected = prepared.manager;
        expected_result = prepared.result;
    }
    bool const ready = admit && (!barometer || prepare);
    Calls::ready = expected;
    Calls::ready_result = expected_result;
    auto & aid = expected.m_sessions[FORMAL_ESKF_PROOF_HEIGHT_SOURCE];
    bool const learn = ready && aid.active && expected.m_reference != source;
    bool const start = ready && !aid.active;
    bool const fuse = ready && aid.active;
    bool const success = fuse && measurement.fusion.status == Status::success;
    bool const limit = success && measurement.fusion.decision == formal_eskf::runtime::FusionDecision::fused;
    bool const recover = success && (!limit || limit_status == Status::success);
    Calls::variance =
        !barometer && expected.m_sessions[0U].active && sample.variance > Scalar{100} ? Scalar{100} : sample.variance;
    Calls::observation = expected.m_origin_altitude - (sample.altitude - aid.bias.value);
    Calls::observation_variance = Calls::variance + aid.bias.variance;
    if (learn)
    {
        Scalar const prior_interval = aid.bias.dt;
        aid.bias = bias;
        aid.bias.dt = prior_interval;
    }
    if (start)
    {
        if (!barometer)
        {
            expected = reset.manager;
            expected_result = reset.result;
            expected_state = &reset.state;
            expected_covariance = &reset.covariance;
            if (reset.result.status == Status::success)
            {
                aid.bias = {};
                expected.m_reference = source;
            }
        }
        else
        {
            aid.bias.value = expected.m_barometer_lpf - (admitted.manager.m_origin_altitude - state.p_n(2U));
        }
        if (barometer || reset.result.status == Status::success)
        {
            aid.active = true;
            aid.stop_reason = Reason::none;
            aid.progress_time = now;
        }
    }
    if (fuse)
    {
        aid.fusion_reset = false;
        aid.fusion = measurement.fusion;
        aid.ratio.time_us = sample.time_us;
        aid.ratio.filtered = reported;
        aid.fusion_innovation = measurement.fusion_innovation(0U);
        aid.innovation_limited = measurement.innovation_limited;
        expected_result.status = aid.fusion.status;
        expected_state = &fused;
        expected_covariance = &fused_P;
        if (limit)
        {
            expected_state = &limited;
            expected_result.status = limit_status;
            if (limit_status == Status::success)
            {
                aid.fused_time = aid.progress_time = expected.m_last_height_progress = now;
                ++expected_result.fused;
            }
        }
        else if (success)
        {
            ++expected_result.rejected;
        }
        Calls::before_recovery = expected;
        Calls::recovery_result = expected_result;
        Calls::recovery_state = expected_state;
        Calls::recovery_covariance = expected_covariance;
    }
    owner.process(source, now, sample, velocity, state, P, result, permissions, acceleration, ground);
    __ESBMC_assert(Calls::admissions == 1U && Calls::preparations == static_cast<unsigned>(admit && barometer) &&
                       Calls::biases == static_cast<unsigned>(learn) &&
                       Calls::resets == static_cast<unsigned>(start && !barometer) &&
                       Calls::fusions == static_cast<unsigned>(fuse) && Calls::reports == static_cast<unsigned>(fuse) &&
                       Calls::limits == static_cast<unsigned>(limit) &&
                       Calls::recoveries == static_cast<unsigned>(recover),
                   "Height process: exact startup, nonreference bias, fusion, limiting and recovery eligibility");
    if (recover)
    {
        check_manager(owner, recovered.manager);
        check_result(result, recovered.result);
        check_state(state, recovered.state);
        check_matrix(P, recovered.covariance);
    }
    else
    {
        check_manager(owner, expected);
        check_result(result, expected_result);
        check_state(state, *expected_state);
        check_matrix(P, *expected_covariance);
    }
    __ESBMC_assert(!control || control_domain(owner),
                   "Height process: control-domain induction through every source phase and failure");
    __ESBMC_assert(history_domain(owner, now) && same_owner_constants(owner, before),
                   "Height process: admitted producer domains preserve chronological and bias-ceiling induction");
    __ESBMC_assert(same_sample(sample, Calls::sample) && bias_proof::same_permissions(permissions, Calls::permissions),
                   "Height process: borrowed sample and bias permissions remain unchanged");
}

int main() { return 0; }
