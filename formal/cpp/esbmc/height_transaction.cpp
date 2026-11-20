// SPDX-License-Identifier: BSD-3-Clause

#include "height_transaction_support.hpp"
#include <initializer_list>

namespace height_transaction_proof
{
struct ProcessResult
{
    Manager manager;
    State state;
    Covariance covariance;
    Result result;
};
struct Calls
{
    inline static ProcessResult const * first = nullptr;
    inline static ProcessResult const * second = nullptr;
    inline static Manager const * original_manager = nullptr;
    inline static State const * original_state = nullptr;
    inline static Covariance const * original_covariance = nullptr;
    inline static Manager const * owner = nullptr;
    inline static State const * state_output = nullptr;
    inline static Covariance const * covariance_output = nullptr;
    inline static Sample const * barometer = nullptr;
    inline static Sample const * gnss = nullptr;
    inline static Permissions const * permissions = nullptr;
    inline static Acceleration acceleration{};
    inline static std::uint64_t now{};
    inline static Scalar first_variance{}, second_variance{};
    inline static bool velocity{}, ground{};
    inline static unsigned count{}, predictions{};
};
} // namespace height_transaction_proof

// Pure source-processing boundary. The candidate may be partially changed on
// any failure. Actual process/recovery/reset producers must discharge the
// boundary's input frames; this harness proves the enclosing transaction only.
namespace formal_eskf::px4
{
template <> inline void HeightBias<height_transaction_proof::Scalar>::predict(height_transaction_proof::Scalar interval)
{
    using namespace height_transaction_proof;
    unsigned const index = Calls::count;
    __ESBMC_assert(index < 2U, "Height transaction: bias prediction belongs to one of the two source stages");
    Manager const * prior = Calls::original_manager;
    Scalar predicted_variance = Calls::first_variance;
    if (index != 0U)
    {
        prior = &Calls::first->manager;
        predicted_variance = Calls::second_variance;
    }
    auto const & aid = prior->m_sessions[index];
    Source const source = index == 0U ? Source::barometer : Source::gnss;
    Scalar const expected_interval = Calls::original_manager->m_tick == 0U
                                         ? Scalar{0}
                                         : static_cast<Scalar>(Calls::now - Calls::original_manager->m_tick) *
                                               static_cast<Scalar>(profile::seconds_per_microsecond);
    __ESBMC_assert(
        aid.active && prior->m_reference != source && same_bias(*this, aid.bias) && same(interval, expected_interval) &&
            std::isfinite(interval) && interval >= Scalar{0},
        "Height transaction: predict only the active nonreference bias using its exact prior and tick interval");
    // Actual height_bias.cpp discharges these variance and field-frame contracts.
    // Nonfinite scratch remains admitted; no successful owner update is assumed.
    bool const bounded = std::isfinite(predicted_variance) &&
                         predicted_variance >= static_cast<Scalar>(profile::height_bias::minimum_variance) &&
                         predicted_variance <= maximum_variance;
    __ESBMC_assume(!std::isfinite(variance) || bounded);
    __ESBMC_assume(!std::isfinite(predicted_variance) || bounded);
    variance = predicted_variance;
    dt = interval;
    ++Calls::predictions;
}

template <>
inline void HeightAiding<height_transaction_proof::Backend>::process(Source source, std::uint64_t now,
                                                                     Sample const & sample, bool velocity,
                                                                     state_type & state, covariance_type & covariance,
                                                                     Result & result,
                                                                     configuration::Ins::BiasUpdate const & permissions,
                                                                     Acceleration const & acceleration, bool ground)
{
    using namespace height_transaction_proof;
    unsigned const index = Calls::count++;
    __ESBMC_assert(index < 2U && source == (index == 0U ? Source::barometer : Source::gnss),
                   "Height transaction: barometer then GNSS, once each");
    __ESBMC_assert(now == Calls::now && velocity == Calls::velocity && ground == Calls::ground &&
                       bias_proof::same_permissions(permissions, *Calls::permissions) &&
                       acceleration.bad == Calls::acceleration.bad &&
                       acceleration.times.update_time_us == Calls::acceleration.times.update_time_us &&
                       acceleration.times.last_good_time_us == Calls::acceleration.times.last_good_time_us,
                   "Height transaction: exact timestamp, permissions, health and ground-effect forwarding");
    Manager predicted = *Calls::original_manager;
    if (index != 0U)
    {
        predicted = Calls::first->manager;
    }
    Scalar const dt = Calls::original_manager->m_tick == 0U
                          ? Scalar{0}
                          : static_cast<Scalar>(now - Calls::original_manager->m_tick) *
                                static_cast<Scalar>(profile::seconds_per_microsecond);
    auto & aid = predicted.m_sessions[index];
    if (aid.active && predicted.m_reference != source)
    {
        aid.bias.dt = dt;
        aid.bias.variance = Calls::first_variance;
        if (index != 0U)
        {
            aid.bias.variance = Calls::second_variance;
        }
    }
    check_manager(*this, predicted);
    __ESBMC_assert(std::isfinite(dt) && dt >= Scalar{0} && history_domain(*this, now),
                   "Height transaction: exact bias prediction policy/interval establishes the process domain");
    if (index == 0U)
    {
        check_state(state, *Calls::original_state);
        check_matrix(covariance, *Calls::original_covariance);
        __ESBMC_assert(same_sample(sample, *Calls::barometer) && same_result(result, Result{}),
                       "Height transaction: first source receives barometer and complete original numerical inputs");
    }
    else
    {
        check_state(state, Calls::first->state);
        check_matrix(covariance, Calls::first->covariance);
        __ESBMC_assert(Calls::first->result.status == Status::success && same_sample(sample, *Calls::gnss) &&
                           same_result(result, Calls::first->result),
                       "Height transaction: GNSS receives the successful barometer candidate");
    }
    __ESBMC_assert(this != Calls::owner && &state != Calls::state_output && &covariance != Calls::covariance_output,
                   "Height transaction: all three caller-owned objects remain unpublished during processing");
    check_manager(*Calls::owner, *Calls::original_manager);
    check_state(*Calls::state_output, *Calls::original_state);
    check_matrix(*Calls::covariance_output, *Calls::original_covariance);
    if (index == 0U)
    {
        *this = Calls::first->manager;
        state = Calls::first->state;
        covariance = Calls::first->covariance;
        result = Calls::first->result;
    }
    else
    {
        *this = Calls::second->manager;
        state = Calls::second->state;
        covariance = Calls::second->covariance;
        result = Calls::second->result;
    }
}
} // namespace formal_eskf::px4

using namespace height_transaction_proof;

void verify_height_transaction(Manager owner, State state, Covariance covariance, std::uint64_t now, Sample barometer,
                               Sample gnss, bool velocity, Permissions permissions, Acceleration acceleration,
                               bool ground, ProcessResult first, ProcessResult second, Scalar first_variance,
                               Scalar second_variance, bool shared_sample)
{
    auto const epoch = owner.m_tick > owner.m_started ? owner.m_tick : owner.m_started;
    __ESBMC_assume(history_domain(owner, epoch));
    bool const control = control_domain(owner);
    auto const before = owner;
    auto const old_state = state;
    auto const old_covariance = covariance;
    auto const old_barometer = barometer;
    auto const old_unused_gnss = gnss;
    Sample const * gnss_input = &gnss;
    if (shared_sample)
    {
        gnss_input = &barometer;
    }
    auto const old_gnss = *gnss_input;
    auto const old_permissions = permissions;
    Calls::first = &first;
    Calls::second = &second;
    Calls::original_manager = &before;
    Calls::original_state = &old_state;
    Calls::original_covariance = &old_covariance;
    Calls::owner = &owner;
    Calls::state_output = &state;
    Calls::covariance_output = &covariance;
    Calls::barometer = &old_barometer;
    Calls::gnss = &old_gnss;
    Calls::permissions = &old_permissions;
    Calls::acceleration = acceleration;
    Calls::now = now;
    Calls::velocity = velocity;
    Calls::ground = ground;
    Calls::first_variance = first_variance;
    Calls::second_variance = second_variance;
    Calls::count = Calls::predictions = 0U;
    bool const time_ok = before.m_started != 0U && now >= before.m_started && now > before.m_tick;
    bool const health_ok = acceleration.times.update_time_us <= now &&
                           acceleration.times.last_good_time_us <= acceleration.times.update_time_us &&
                           (!acceleration.bad || acceleration.times.update_time_us != 0U);
    bool const finite = std::isfinite(old_state.p_n(2U)) && std::isfinite(old_covariance(2U, 2U));
    bool const admitted = time_ok && health_ok && finite && old_covariance(2U, 2U) >= Scalar{0};
    bool const first_ok = admitted && first.result.status == Status::success;
    bool const second_ok = first_ok && second.result.status == Status::success;
    // Only a reached process() call supplies a postcondition. In particular,
    // output chronology must not rule out an invalid outer timestamp.
    __ESBMC_assume(!admitted || (history_domain(first.manager, now) && same_owner_constants(first.manager, before)));
    __ESBMC_assume(!first_ok || (history_domain(second.manager, now) && same_owner_constants(second.manager, before)));
    __ESBMC_assume(!admitted || !control || control_domain(first.manager));
    __ESBMC_assume(!first_ok || !control || control_domain(second.manager));
    bool const biases_finite = second.manager.m_sessions[0U].bias.formal_eskf_proof_finite() &&
                               second.manager.m_sessions[1U].bias.formal_eskf_proof_finite();
    auto const expected = !time_ok || !health_ok ? Status::out_of_range
                          : !finite              ? Status::non_finite_input
                          : !admitted            ? Status::domain_error
                          : !first_ok            ? first.result.status
                          : !second_ok           ? second.result.status
                          : !biases_finite       ? Status::non_finite_result
                                                 : Status::success;
    auto const result =
        owner.update(now, barometer, *gnss_input, velocity, state, covariance, permissions, acceleration, ground);
    __ESBMC_assert(result.status == expected &&
                       Calls::count == static_cast<unsigned>(admitted) + static_cast<unsigned>(first_ok),
                   "Height transaction: validation precedence, early return and no later source after failure");
    unsigned const predictions =
        static_cast<unsigned>(admitted && before.m_sessions[0U].active && before.m_reference != Source::barometer) +
        static_cast<unsigned>(first_ok && first.manager.m_sessions[1U].active &&
                              first.manager.m_reference != Source::gnss);
    __ESBMC_assert(
        Calls::predictions == predictions,
        "Height transaction: exactly the eligible source biases are predicted, with no prediction after failure");
    if (result.status != Status::success)
    {
        check_manager(owner, before);
        check_state(state, old_state);
        check_matrix(covariance, old_covariance);
        __ESBMC_assert(result.fused == 0U && result.rejected == 0U && result.height_resets == 0U &&
                           result.velocity_resets == 0U && same(result.height_delta, Scalar{0}) &&
                           same(result.velocity_delta, Scalar{0}),
                       "Height transaction: full owner/state/covariance rollback, with no success/reset report");
    }
    else
    {
        __ESBMC_assert(history_domain(owner, now) && owner.m_sessions[0U].bias.formal_eskf_proof_finite() &&
                           owner.m_sessions[1U].bias.formal_eskf_proof_finite() &&
                           same(owner.m_origin_altitude, before.m_origin_altitude) &&
                           owner.m_started == before.m_started,
                       "Height transaction: successful commit preserves chronological and finite-bias induction");
        auto expected_manager = second.manager;
        if (expected_manager.m_reference == Source::none)
        {
            expected_manager.m_reference = expected_manager.m_sessions[1U].active   ? Source::gnss
                                           : expected_manager.m_sessions[0U].active ? Source::barometer
                                                                                    : Source::none;
        }
        expected_manager.m_tick = now;
        check_manager(owner, expected_manager);
        check_state(state, second.state);
        check_matrix(covariance, second.covariance);
        __ESBMC_assert(same_result(result, second.result),
                       "Height transaction: publish complete candidate, final reference and current tick");
    }
    __ESBMC_assert(!control || control_domain(owner),
                   "Height transaction: initialization control invariants survive rollback or successful commit");
    __ESBMC_assert(same_sample(barometer, old_barometer) && same_sample(gnss, old_unused_gnss) &&
                       bias_proof::same_permissions(permissions, old_permissions),
                   "Height transaction: samples and permissions are read-only");
}

int main() { return 0; }
