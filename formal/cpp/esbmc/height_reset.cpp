// SPDX-License-Identifier: BSD-3-Clause

#include "height_transaction_support.hpp"

#ifndef FORMAL_ESKF_PROOF_HEIGHT_SOURCE
#define FORMAL_ESKF_PROOF_HEIGHT_SOURCE 0
#endif
static_assert(FORMAL_ESKF_PROOF_HEIGHT_SOURCE == 0 || FORMAL_ESKF_PROOF_HEIGHT_SOURCE == 1);

namespace height_transaction_proof
{
struct ResetResult
{
    State state;
    Covariance covariance;
    Scalar delta;
    Status status;
};
struct ResetCall
{
    inline static ResetResult const * result = nullptr;
    inline static ResetResult const * velocity = nullptr;
    inline static unsigned velocity_calls{};
    inline static State const * state = nullptr;
    inline static Covariance const * covariance = nullptr;
    inline static Scalar position{}, variance{};
    inline static unsigned count{};
};
} // namespace height_transaction_proof

namespace formal_eskf::runtime
{
template <>
inline Status reset_vertical_position<height_transaction_proof::Backend>(
    height_transaction_proof::State const & state, height_transaction_proof::Covariance const & P,
    height_transaction_proof::Scalar position, height_transaction_proof::Scalar variance,
    height_transaction_proof::State & state_output, height_transaction_proof::Covariance & P_output,
    height_transaction_proof::Scalar & delta)
{
    using namespace height_transaction_proof;
    check_state(state, *ResetCall::state);
    check_matrix(P, *ResetCall::covariance);
    __ESBMC_assert(ResetCall::count++ == 0U && same(position, ResetCall::position) &&
                       same(variance, ResetCall::variance) && &state == &state_output && &P == &P_output,
                   "Height reset: exact prior, down coordinate, variance and in-place numerical boundary");
    if (ResetCall::result->status == Status::success)
    {
        state_output = ResetCall::result->state;
        P_output = ResetCall::result->covariance;
        delta = ResetCall::result->delta;
    }
    return ResetCall::result->status;
}

template <>
inline Status reset_vertical_velocity<height_transaction_proof::Backend>(
    height_transaction_proof::State const & state, height_transaction_proof::Covariance const & P,
    height_transaction_proof::Scalar velocity, height_transaction_proof::Scalar variance,
    height_transaction_proof::State & state_output, height_transaction_proof::Covariance & P_output,
    height_transaction_proof::Scalar & delta)
{
    using namespace height_transaction_proof;
    check_state(state, ResetCall::result->state);
    check_matrix(P, ResetCall::result->covariance);
    __ESBMC_assert(ResetCall::count == 1U && ResetCall::result->status == Status::success &&
                       ResetCall::velocity_calls++ == 0U && same(velocity, Scalar{0}) &&
                       same(variance, static_cast<Scalar>(setting::reset_velocity_variance)) &&
                       &state == &state_output && &P == &P_output,
                   "Height recovery: velocity reset follows successful height reset and receives its candidate");
    if (ResetCall::velocity->status == Status::success)
    {
        state_output = ResetCall::velocity->state;
        P_output = ResetCall::velocity->covariance;
        delta = ResetCall::velocity->delta;
    }
    return ResetCall::velocity->status;
}
} // namespace formal_eskf::runtime

using namespace height_transaction_proof;

void verify_height_reset(Manager owner, State state, Covariance P, std::uint64_t now, Scalar altitude, Scalar variance,
                         Result result, ResetResult numerical)
{
    constexpr Source source = static_cast<Source>(FORMAL_ESKF_PROOF_HEIGHT_SOURCE);
    __ESBMC_assume(history_domain(owner, now));
    bool const control = control_domain(owner);
    auto expected = owner;
    auto const old_state = state;
    auto const old_P = P;
    auto expected_result = result;
    ResetCall::result = &numerical;
    ResetCall::state = &old_state;
    ResetCall::covariance = &old_P;
    ResetCall::position = owner.m_origin_altitude - altitude;
    ResetCall::variance = variance;
    ResetCall::count = 0U;
    expected_result.status = numerical.status;
    if (numerical.status == Status::success)
    {
        expected.m_sessions[0U].bias.value += numerical.delta;
        expected.m_sessions[1U].bias.value += numerical.delta;
        expected_result.height_delta =
            result.height_resets == 0U ? numerical.delta : result.height_delta + numerical.delta;
        expected_result.height_resets = result.height_resets + 1U;
        expected.m_last_height_progress = now;
        auto & selected = expected.m_sessions[FORMAL_ESKF_PROOF_HEIGHT_SOURCE];
        selected.progress_time = now;
        selected.fusion_reset = true;
        selected.ratio.time_us = selected.sample_time;
        selected.ratio.filtered = 0.;
    }
    owner.reset_height(source, now, altitude, variance, state, P, result);
    __ESBMC_assert(ResetCall::count == 1U, "Height reset: exactly one numerical position reset");
    __ESBMC_assert(!control || control_domain(owner),
                   "Height reset/recovery: owner filter, intervals and active reference are preserved");
    check_manager(owner, expected);
    check_result(result, expected_result);
    if (numerical.status == Status::success)
    {
        // Height reset: publish numerical result unchanged
        check_state(state, numerical.state);
        check_matrix(P, numerical.covariance);
    }
    else
    {
        // Height reset: failed numerical reset preserves state and covariance
        check_state(state, old_state);
        check_matrix(P, old_P);
    }
    __ESBMC_assert(history_domain(owner, now), "Height reset: owner history and bias variance domains preserved");
}

void verify_height_recovery(Manager owner, State state, Covariance P, std::uint64_t now, Sample sample, Scalar variance,
                            Result result, ResetResult position, ResetResult velocity, bool velocity_aiding,
                            Acceleration acceleration)
{
    constexpr Source source = static_cast<Source>(FORMAL_ESKF_PROOF_HEIGHT_SOURCE);
    constexpr bool barometer = source == Source::barometer;
    __ESBMC_assume(history_domain(owner, now));
    bool const control = control_domain(owner);
    auto expected = owner;
    auto const old_state = state;
    auto const old_P = P;
    auto expected_result = result;
    auto & aid = expected.m_sessions[FORMAL_ESKF_PROOF_HEIGHT_SOURCE];
    __ESBMC_assume(aid.progress_time <= now);
    auto const progress = owner.m_last_height_progress == 0U ? owner.m_started : owner.m_last_height_progress;
    bool const acceleration_reset =
        acceleration.times.update_time_us != 0U && acceleration.times.update_time_us <= now &&
        acceleration.times.last_good_time_us <= acceleration.times.update_time_us &&
        (acceleration.times.last_good_time_us == 0U || now - acceleration.times.last_good_time_us > 500000U);
    bool const needs_reset =
        acceleration_reset || (progress != 0U && now > progress && now - progress > setting::fusion_timeout_us);
    bool const reset = owner.m_reference == source && needs_reset;
    bool const vertical = reset && position.status == Status::success && barometer && !velocity_aiding;
    ResetCall::result = &position;
    ResetCall::velocity = &velocity;
    ResetCall::count = ResetCall::velocity_calls = 0U;
    ResetCall::state = &old_state;
    ResetCall::covariance = &old_P;
    ResetCall::position =
        owner.m_origin_altitude - (barometer ? owner.m_barometer_lpf - aid.bias.value : sample.altitude);
    ResetCall::variance = variance;
    if (reset)
    {
        expected_result.status = position.status;
        if (position.status == Status::success)
        {
            expected.m_sessions[0U].bias.value += position.delta;
            expected.m_sessions[1U].bias.value += position.delta;
            expected_result.height_delta =
                result.height_resets == 0U ? position.delta : result.height_delta + position.delta;
            expected_result.height_resets = result.height_resets + 1U;
            expected.m_last_height_progress = now;
            aid.progress_time = now;
            aid.fusion_reset = true;
            aid.ratio.time_us = aid.sample_time;
            aid.ratio.filtered = 0.;
            aid.bias.value = (barometer ? expected.m_barometer_lpf : sample.altitude) -
                             (expected.m_origin_altitude - position.state.p_n(2U));
            if (vertical)
            {
                expected_result.status = velocity.status;
                if (velocity.status == Status::success)
                {
                    expected_result.velocity_delta = velocity.delta;
                    ++expected_result.velocity_resets;
                }
            }
        }
    }
    else if (now - aid.progress_time > setting::fusion_timeout_us)
    {
        aid.active = false;
        aid.stop_reason = Reason::fusion_timeout;
        aid.fusion = {};
        if (expected.m_reference == source)
        {
            expected.m_reference = Source::none;
        }
        aid.fault = barometer && !needs_reset;
    }
    owner.recover(source, now, sample, variance, velocity_aiding, state, P, result, acceleration);
    __ESBMC_assert(ResetCall::count == static_cast<unsigned>(reset) &&
                       ResetCall::velocity_calls == static_cast<unsigned>(vertical),
                   "Height recovery: reference/reset authority, timeout latch and exact partial-result protocol");
    __ESBMC_assert(!control || control_domain(owner),
                   "Height reset/recovery: owner filter, intervals and active reference are preserved");
    check_manager(owner, expected);
    check_result(result, expected_result);
    if (vertical && velocity.status == Status::success)
    {
        // Height recovery: accepted velocity candidate
        check_state(state, velocity.state);
        check_matrix(P, velocity.covariance);
    }
    else if (reset && position.status == Status::success)
    {
        // Height recovery: height candidate survives velocity failure for outer rollback
        check_state(state, position.state);
        check_matrix(P, position.covariance);
    }
    else
    {
        // Height recovery: no accepted numerical reset preserves inputs
        check_state(state, old_state);
        check_matrix(P, old_P);
    }
    __ESBMC_assert(history_domain(owner, now), "Height recovery: owner history and bias variance domains preserved");
}

int main() { return 0; }
