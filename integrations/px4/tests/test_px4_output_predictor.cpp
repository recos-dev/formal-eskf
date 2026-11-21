// SPDX-License-Identifier: BSD-3-Clause
#include <array>
#include <bit>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <limits>

#include "../module/OutputPredictor.hpp"
#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{
using formal_eskf::test::Context;
using formal_eskf::test::near;
using Backend = formal_eskf::test::Backend<double>;
using Predictor = formal_eskf::px4::OutputPredictor<Backend>;
using History = formal_eskf::px4::ImuHistory<Backend>;
using Vector = Predictor::vector3_type;
using Quaternion = Predictor::quaternion_type;
using State = Predictor::state_type;
using Fault = Predictor::Fault;
namespace profile = formal_eskf::px4::profile;

Vector vector(double x, double y, double z) { return Vector::from_row_major({x, y, z}); }

Quaternion rotation(Vector const & angle)
{
    Quaternion result;
    if (!formal_eskf::succeeded(formal_eskf::so3::exp(angle, profile::minimum_quaternion_norm, result)))
    {
        std::abort();
    }
    return result;
}

History::input_type sample(std::uint64_t time, Vector const & force = vector(0, 0, -profile::gravity_m_s2),
                           Vector const & rate = {})
{
    History::input_type result;
    result.time_us = time;
    result.angular_period_us = 4000;
    result.acceleration_period_us = 4000;
    result.delta_angle_b = rate * 0.004;
    result.delta_velocity_b = force * 0.004;
    result.source = {17, 29, 2, 3};
    return result;
}

bool near_vector(Vector const & a, Vector const & b, double tolerance = 1e-11)
{
    return near(a(0), b(0), tolerance) && near(a(1), b(1), tolerance) && near(a(2), b(2), tolerance);
}

bool same_sample(Predictor::Sample const & a, Predictor::Sample const & b)
{
    if (a.time_us != b.time_us || a.start_time_us != b.start_time_us || a.angular_period_us != b.angular_period_us)
    {
        return false;
    }
    auto const qa = a.q_nb.coefficients();
    auto const qb = b.q_nb.coefficients();
    for (std::size_t i = 0; i < 4; ++i)
    {
        if (std::bit_cast<std::uint64_t>(qa(i)) != std::bit_cast<std::uint64_t>(qb(i)))
        {
            return false;
        }
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
        if (std::bit_cast<std::uint64_t>(a.v_n(i)) != std::bit_cast<std::uint64_t>(b.v_n(i)) ||
            std::bit_cast<std::uint64_t>(a.p_n(i)) != std::bit_cast<std::uint64_t>(b.p_n(i)))
        {
            return false;
        }
    }
    return true;
}

struct Session
{
    History imu;
    Predictor output;
    std::uint64_t time = 1000000;

    void seed(Context & test, State const & state = {}, formal_eskf::px4::TimingParameters const & parameters = {})
    {
        test.expect(imu.configure(parameters) && output.configure(parameters) && imu.seed(sample(time), time) &&
                        output.seed(state, time),
                    "setup", "matched configuration and seed");
    }

    void push(Context & test, Vector const & force = vector(0, 0, -profile::gravity_m_s2), Vector const & rate = {})
    {
        time += 4000;
        test.expect(imu.push(sample(time, force, rate), time), "input", "admit real interval");
        auto const * input = imu.latest();
        test.expect(input != nullptr && output.predict(*input), "output", "predict exactly one admitted interval");
    }

    void fill(Context & test)
    {
        for (std::size_t tick = 0; tick < imu.configuration().imu_length; ++tick)
        {
            push(test);
        }
    }
};

void startup_and_configuration(Context & test)
{
    Predictor output;
    test.expect(!output.seed({}, 1000000) && !output.predict({}) && !output.correct(1000000, {}) && !output.ready(),
                "startup", "no use before configuration and seed");
    test.expect(!output.configure({10000, 200, 0, 0, 0}) && !output.configure({4000, 240, 0, 0, 0}), "startup",
                "reject unsupported interval and excessive length");
    test.expect(output.configure({}) && !output.seed({}, 0), "startup", "zero seed rejected");
    State bad;
    bad.b_a(1) = std::numeric_limits<double>::quiet_NaN();
    test.expect(!output.seed(bad, 1000000) && output.seed({}, 1000000), "startup", "bad seed preserves empty state");
    test.expect(!output.seed({}, 1000000) && !output.configure({}) && output.entries() == 0 && !output.ready(),
                "startup", "seed does not fabricate history or permit live reconfiguration");
    auto const before = output.current();
    test.expect(!output.correct(1004000, {}) && output.fault() == Fault::epoch_mismatch &&
                    same_sample(before, output.current()),
                "startup", "feedback during warmup faults atomically");
    output.reset();
    test.expect(output.fault() == Fault::none && output.entries() == 0 && output.current().time_us == 0 &&
                    output.fusion_time_us() == 0 && !output.ready() && output.seed({}, 2000000),
                "restart", "reset retains configuration and clears all temporal state");
}

void analytic_translation(Context & test, formal_eskf::px4::TimingParameters const & parameters, std::uint64_t epoch)
{
    Session session;
    session.time = epoch;
    State initial;
    initial.p_n = vector(1, -2, 3);
    initial.v_n = vector(2, -1, 0.5);
    initial.b_a = vector(0.1, -0.2, 0.3);
    initial.b_g = vector(0.01, 0.02, -0.03);
    session.seed(test, initial, parameters);
    auto const force = vector(1.1, 1.8, 0.3 - profile::gravity_m_s2);
    auto const acceleration = vector(1, 2, 0);
    auto const length = session.imu.configuration().imu_length;
    for (std::size_t tick = 1; tick <= 4 * length; ++tick)
    {
        session.push(test, force, initial.b_g);
        double const t = static_cast<double>(tick) * 0.004;
        test.expect(
            near_vector(session.output.current().p_n, initial.p_n + initial.v_n * t + acceleration * (0.5 * t * t)) &&
                near_vector(session.output.current().v_n, initial.v_n + acceleration * t),
            "translation", "independent constant-acceleration solution with nonzero biases");
        auto const * delayed = session.imu.delayed();
        if (delayed == nullptr)
        {
            test.expect(!session.output.ready() && session.output.delayed() == nullptr, "warmup",
                        "matched real warmup");
            continue;
        }
        auto const time = delayed->time_us;
        auto const * output = session.output.delayed();
        test.expect(output != nullptr && output->time_us == time, "history", "exact epoch through repeated wraps");
        double const td = static_cast<double>(time - epoch) * 1e-6;
        auto main = initial;
        main.p_n = initial.p_n + initial.v_n * td + acceleration * (0.5 * td * td);
        main.v_n = initial.v_n + acceleration * td;
        test.expect(session.output.correct(time, main) && session.imu.complete(time) && session.output.ready(),
                    "feedback", "same motion has negligible tracking error");
        test.expect(near_vector(session.output.tracking_error().velocity_n, {}) &&
                        near_vector(session.output.tracking_error().position_n, {}) &&
                        session.output.delayed() == nullptr,
                    "feedback", "completed interval cannot be selected again");
    }
}

double angular_step(double rate, double dt)
{
#if ESKF_QUAT_APPROX
    return 2 * std::atan(rate * dt * 0.5);
#else
    return rate * dt;
#endif
}

void rotating_force(Context & test)
{
    Session session;
    session.seed(test);
    double yaw = 0;
    Vector p;
    Vector v;
    for (std::size_t tick = 0; tick < 20; ++tick)
    {
        session.push(test, vector(1, 0, -profile::gravity_m_s2), vector(0, 0, 1));
        auto const a = vector(std::cos(yaw), std::sin(yaw), 0);
        p = p + v * 0.004 + a * (0.5 * 0.004 * 0.004);
        v = v + a * 0.004;
        yaw += angular_step(1., 0.004);
        auto const & output = session.output.current();
        test.expect(near_vector(output.p_n, p) && near_vector(output.v_n, v) &&
                        near(output.q_nb.q0(), std::cos(yaw / 2), 1e-12) &&
                        near(output.q_nb.q3(), std::sin(yaw / 2), 1e-12),
                    "rotation", "prior attitude rotates force; Hamilton right update has correct sign");
    }
}

void gap_and_periods(Context & test)
{
    Session session;
    test.expect(session.imu.configure({}) && session.output.configure({}) &&
                    session.imu.seed(sample(session.time, vector(2, 0, -profile::gravity_m_s2)), session.time) &&
                    session.output.seed({}, session.time),
                "gap", "seed held rate");
    auto input = sample(session.time + 14000, vector(3, 0, -profile::gravity_m_s2));
    input.acceleration_period_us = 4200;
    input.delta_velocity_b = vector(3, 0, -profile::gravity_m_s2) * 0.0042;
    test.expect(session.imu.push(input, input.time_us) && session.imu.latest() != nullptr, "gap", "admit gap");
    auto const interval = *session.imu.latest();
    test.expect(interval.hold_period_us == 10000 && session.output.predict(interval), "gap", "hold then real step");
    test.expect(near(session.output.current().v_n(0), 2 * 0.010 + 3 * 0.004, 1e-12) &&
                    near(session.output.current().p_n(0),
                         0.5 * 2 * 0.010 * 0.010 + 0.020 * 0.004 + 0.5 * 3 * 0.004 * 0.004, 1e-12) &&
                    near(session.output.current().v_n(2), 0., 1e-12) && session.output.entries() == 1,
                "gap", "separate measured periods and one endpoint; hold is not extra history");
}

void variable_periods(Context & test)
{
    Session session;
    session.seed(test);
    for (std::size_t tick = 0; tick < 160; ++tick)
    {
        std::uint32_t const period_us = tick % 2 == 0 ? 3800 : 4200;
        session.time += period_us;
        auto input = sample(session.time);
        input.angular_period_us = period_us;
        input.acceleration_period_us = period_us;
        input.delta_velocity_b = vector(1, 0, -profile::gravity_m_s2) * (static_cast<double>(period_us) * 1e-6);
        test.expect(session.imu.push(input, session.time) && session.imu.latest() != nullptr &&
                        session.output.predict(*session.imu.latest()),
                    "jitter", "admit and predict alternating integration-period bounds");
        double const t = static_cast<double>(session.time - 1000000) * 1e-6;
        test.expect(near(session.output.current().v_n(0), t, 1e-11) &&
                        near(session.output.current().p_n(0), 0.5 * t * t, 1e-11),
                    "jitter", "current state follows actual integration time");
        if (auto const * delayed = session.imu.delayed())
        {
            auto const time = delayed->time_us;
            double const td = static_cast<double>(time - 1000000) * 1e-6;
            State main;
            main.v_n(0) = td;
            main.p_n(0) = 0.5 * td * td;
            test.expect(session.output.correct(time, main) && session.imu.complete(time), "jitter",
                        "variable periods retain the same delayed epoch");
        }
    }
}

void feedback_and_bias(Context & test)
{
    Session session;
    session.seed(test);
    session.fill(test);
    State main;
    main.v_n = vector(1, -2, 3);
    main.p_n = vector(10, 20, -30);
    main.q_nb = rotation(vector(0, 0, 0.2));
    main.b_a = vector(0.2, 0, 0);
    main.b_g = vector(0, 0, 0.03);
    auto const delayed_time = session.imu.delayed()->time_us;
    auto const before = session.output.current();
    test.expect(session.output.correct(delayed_time, main) && session.imu.complete(delayed_time), "tracking",
                "first correction");
    double const gain = 0.004 / 0.25;
    double const scale = gain + 0.1 * gain * gain;
    test.expect(near_vector(session.output.current().v_n, main.v_n * scale) &&
                    near_vector(session.output.current().p_n, main.p_n * scale) &&
                    session.output.current().q_nb.q3() == before.q_nb.q3() &&
                    session.output.current().time_us == before.time_us,
                "tracking", "PI translation adjusts history immediately; attitude waits for fast prediction");
    auto const corrected = session.output.current();
    double const correction_rate = std::sin(0.1) / 0.212;
    test.expect(near_vector(session.output.angular_rate_b(), {}), "tracking",
                "feedback does not retroactively change the latest propagated angular rate");
    session.push(test);
    double const angle = angular_step(correction_rate - 0.03, 0.004);
    test.expect(near(session.output.current().q_nb.q3(), std::sin(angle / 2), 1e-12) &&
                    near(session.output.current().v_n(0), corrected.v_n(0) - 0.2 * 0.004, 1e-12),
                "tracking", "new bias and correct-sign attitude feedback affect the next real interval");
    test.expect(near_vector(session.output.angular_rate_b(), vector(0, 0, correction_rate - 0.03)), "tracking",
                "output reference-point velocity uses the same bias/tracking-corrected rate as propagation");
    auto const * old = session.output.delayed();
    test.expect(old != nullptr && near_vector(old->v_n, main.v_n * scale) && near_vector(old->p_n, main.p_n * scale),
                "tracking", "the next delayed record received the same translation as current output");
}

void convergence_and_sign(Context & test)
{
    for (bool negative : {false, true})
    {
        Session session;
        session.seed(test);
        State main;
        main.q_nb = rotation(vector(0, 0, 0.2));
        if (negative)
        {
            main.q_nb = -main.q_nb;
        }
        main.p_n = vector(2, -1, 3);
        main.v_n = vector(0.5, -0.2, 0.1);
        auto const origin = main.p_n;
        for (std::size_t tick = 0; tick < 5000; ++tick)
        {
            session.push(test);
            if (auto const * delayed = session.imu.delayed())
            {
                auto const time = delayed->time_us;
                main.p_n = origin + main.v_n * (static_cast<double>(time - 1000000) * 1e-6);
                test.expect(session.output.correct(time, main) && session.imu.complete(time), "convergence",
                            "feedback");
            }
        }
        auto const & current = session.output.current();
        auto const expected_position = origin + main.v_n * 20.;
        std::cout << "20 s tracking residual: yaw=" << 2 * std::atan2(current.q_nb.q3(), current.q_nb.q0()) - 0.2
                  << ", position=" << current.p_n(0) - expected_position(0) << ','
                  << current.p_n(1) - expected_position(1) << ',' << current.p_n(2) - expected_position(2) << '\n';
        test.expect(near(current.q_nb.q3(), std::sin(0.1), 1e-5) &&
                        near_vector(current.p_n, expected_position, 0.001) &&
                        near_vector(current.v_n, main.v_n, 0.0001),
                    "convergence", "constant-velocity reference tracks for either quaternion sign");
    }
}

void feedback_during_gap(Context & test)
{
    Session session;
    session.seed(test);
    session.fill(test);
    State main;
    main.q_nb = rotation(vector(0, 0, 0.2));
    auto const time = session.imu.delayed()->time_us;
    test.expect(session.output.correct(time, main) && session.imu.complete(time), "gap feedback", "establish feedback");
    auto input = sample(session.time + 14000);
    test.expect(session.imu.push(input, input.time_us) && session.imu.latest() != nullptr &&
                    session.output.predict(*session.imu.latest()),
                "gap feedback", "predict a held and real segment with existing angular feedback");
    double const rate = std::sin(0.1) / 0.212;
    double const angle = angular_step(rate, 0.010) + angular_step(rate, 0.004);
    test.expect(near(session.output.current().q_nb.q3(), std::sin(angle * 0.5), 1e-12), "gap feedback",
                "feedback rate integrates over each segment, not twice with a fixed angle");
}

void resets(Context & test)
{
    for (std::size_t fill : {0U, 3U, 54U})
    {
        Session session;
        State initial;
        initial.q_nb = rotation(vector(0.3, 0, 0));
        session.seed(test, initial);
        for (std::size_t tick = 0; tick < fill; ++tick)
        {
            session.push(test);
        }
        auto const before = session.output.current();
        double const angle = 0.4;
        auto const q_delta = rotation(vector(0, 0, angle));
        auto const dv = vector(1, 2, 3);
        auto const dp = vector(4, 5, 6);
        test.expect(session.output.apply_reset(q_delta, dv, dp), "reset", "seed, partial or full history reset");
        auto const & after = session.output.current();
        test.expect(near_vector(after.v_n, before.v_n + dv) && near_vector(after.p_n, before.p_n + dp) &&
                        near(after.q_nb.q0(), std::cos(0.2) * std::cos(0.15), 1e-12) &&
                        near(after.q_nb.q1(), std::cos(0.2) * std::sin(0.15), 1e-12) &&
                        near(after.q_nb.q2(), std::sin(0.2) * std::sin(0.15), 1e-12) &&
                        near(after.q_nb.q3(), std::sin(0.2) * std::cos(0.15), 1e-12) &&
                        after.time_us == before.time_us && session.output.entries() == fill,
                    "reset", "left navigation rotation, additive deltas and preserved epochs");
        if (auto const * delayed = session.output.delayed())
        {
            test.expect(near(delayed->q_nb.q2(), after.q_nb.q2(), 1e-12) && near(delayed->p_n(0), dp(0), 1e-12),
                        "reset", "oldest record receives the same reset");
        }
    }
}

void numerical_rollback(Context & test)
{
    Session session;
    State initial;
    initial.b_a(0) = -std::numeric_limits<double>::max();
    session.seed(test, initial);
    auto const before = session.output.current();
    auto input =
        sample(session.time + 14000, vector(std::numeric_limits<double>::max() * 0.5, 0, -profile::gravity_m_s2));
    test.expect(session.imu.push(input, input.time_us) && session.imu.latest() != nullptr, "atomicity",
                "finite rates accepted before bias subtraction");
    test.expect(!session.output.predict(*session.imu.latest()) && session.output.fault() == Fault::numerical_failure &&
                    same_sample(before, session.output.current()) && session.output.entries() == 0,
                "atomicity", "real-segment overflow also rolls back the successful held segment");

    Session reset_session;
    State large;
    large.p_n(0) = std::numeric_limits<double>::max() * 0.5;
    large.v_n(0) = std::numeric_limits<double>::max();
    reset_session.seed(test, large);
    reset_session.fill(test);
    auto const reset_before = reset_session.output.current();
    auto const reset_cursor = reset_session.output.fusion_time_us();
    test.expect(!reset_session.output.apply_reset({}, {}, vector(std::numeric_limits<double>::max() * 0.3, 0, 0)) &&
                    reset_session.output.fault() == Fault::numerical_failure &&
                    same_sample(reset_before, reset_session.output.current()) &&
                    reset_session.output.fusion_time_us() == reset_cursor,
                "atomicity", "late history reset overflow cannot partially publish the transform");

    Session feedback_session;
    large.p_n(0) = std::numeric_limits<double>::max() * 0.7839;
    feedback_session.seed(test, large);
    feedback_session.fill(test);
    auto const feedback_before = feedback_session.output.current();
    auto const feedback_cursor = feedback_session.output.fusion_time_us();
    auto target = large;
    target.p_n(0) = std::numeric_limits<double>::max();
    test.expect(!feedback_session.output.correct(feedback_session.imu.delayed()->time_us, target) &&
                    feedback_session.output.fault() == Fault::numerical_failure &&
                    same_sample(feedback_before, feedback_session.output.current()) &&
                    feedback_session.output.fusion_time_us() == feedback_cursor,
                "atomicity", "finite PI translation that overflows a later record cannot advance feedback");
}

void reset_feedback_invariance(Context & test)
{
    Session original;
    Session reset_session;
    original.seed(test);
    reset_session.seed(test);
    original.fill(test);
    reset_session.fill(test);
    State main;
    main.q_nb = rotation(vector(0, 0, 0.1));
    main.p_n = vector(2, 3, 4);
    auto const time = original.imu.delayed()->time_us;
    test.expect(original.output.correct(time, main) && original.imu.complete(time) &&
                    reset_session.output.correct(time, main) && reset_session.imu.complete(time),
                "reset feedback", "establish nonzero tracking feedback and integrals");
    auto const q_delta = rotation(vector(0, 0, 0.3));
    auto const translation = vector(5, 6, 7);
    test.expect(reset_session.output.apply_reset(q_delta, {}, translation), "reset feedback",
                "global yaw and position reset");
    original.push(test);
    reset_session.push(test);
    auto const expected_q = q_delta * original.output.current().q_nb;
    test.expect(near(reset_session.output.current().q_nb.q3(), expected_q.q3(), 1e-12) &&
                    near_vector(reset_session.output.current().p_n, original.output.current().p_n + translation),
                "reset feedback", "existing angular feedback is invariant under common left rotation");
    auto reset_main = main;
    reset_main.q_nb = q_delta * main.q_nb;
    reset_main.p_n = main.p_n + translation;
    auto const next_time = original.imu.delayed()->time_us;
    test.expect(
        original.output.correct(next_time, main) && reset_session.output.correct(next_time, reset_main) &&
            near_vector(reset_session.output.current().p_n, original.output.current().p_n + translation) &&
            near_vector(reset_session.output.tracking_error().position_n, original.output.tracking_error().position_n),
        "reset feedback", "common reset preserves tracking residuals and accumulated PI correction");
}

void failures(Context & test)
{
    enum class Case
    {
        wrong_epoch,
        repeated_feedback,
        invalid_bias,
        overflow,
        unacknowledged,
        repeated_prediction,
        bad_reset
    };
    for (auto scenario : {Case::wrong_epoch, Case::repeated_feedback, Case::invalid_bias, Case::overflow,
                          Case::unacknowledged, Case::repeated_prediction, Case::bad_reset})
    {
        Session session;
        State initial;
        if (scenario == Case::overflow)
        {
            initial.p_n(0) = -std::numeric_limits<double>::max();
        }
        session.seed(test, initial);
        session.fill(test);
        auto time = session.imu.delayed()->time_us;
        State main = initial;
        if (scenario == Case::repeated_feedback)
        {
            test.expect(session.output.correct(time, main), "failure", "complete first feedback");
        }
        auto const before = session.output.current();
        auto const fusion = session.output.fusion_time_us();
        auto const count = session.output.entries();
        bool accepted = true;
        Fault expected = Fault::epoch_mismatch;
        switch (scenario)
        {
        case Case::wrong_epoch:
            accepted = session.output.correct(time + 1, main);
            break;
        case Case::repeated_feedback:
            accepted = session.output.correct(time, main);
            break;
        case Case::invalid_bias:
            main.b_g(2) = std::numeric_limits<double>::quiet_NaN();
            accepted = session.output.correct(time, main);
            expected = Fault::invalid_state;
            break;
        case Case::overflow:
            main.p_n(0) = std::numeric_limits<double>::max();
            accepted = session.output.correct(time, main);
            expected = Fault::numerical_failure;
            break;
        case Case::unacknowledged:
            test.expect(session.imu.complete(time), "failure", "advance IMU owner only");
            session.time += 4000;
            test.expect(session.imu.push(sample(session.time), session.time), "failure", "new input");
            accepted = session.output.predict(*session.imu.latest());
            expected = Fault::history_loss;
            break;
        case Case::repeated_prediction:
            accepted = session.output.predict(*session.imu.latest());
            break;
        case Case::bad_reset:
            accepted = session.output.apply_reset({}, {}, vector(0, std::numeric_limits<double>::infinity(), 0));
            expected = Fault::invalid_state;
            break;
        }
        test.expect(!accepted && session.output.fault() == expected && !session.output.ready() &&
                        same_sample(before, session.output.current()) && session.output.fusion_time_us() == fusion &&
                        session.output.entries() == count && session.output.delayed() == nullptr,
                    "failure", "fault preserves output, retained count and cursor");
        test.expect(!session.output.correct(time, {}) && !session.output.configure({}) &&
                        !session.output.apply_reset({}, {}, {}) && session.output.fault() == expected,
                    "failure", "later calls cannot clear the first fault");
        session.output.reset();
        test.expect(near_vector(session.output.angular_rate_b(), {}), "failure", "restart clears output body rate");
        test.expect(session.output.seed({}, 2000000) && session.output.entries() == 0 && !session.output.ready() &&
                        near_vector(session.output.tracking_error().position_n, {}),
                    "failure", "explicit restart clears feedback and starts new warmup");
    }
}
} // namespace

int main()
{
    formal_eskf::test::configure_backend_test();
    static_assert(sizeof(History) + sizeof(Predictor) <= profile::history::storage_budget_bytes);
    Context test;
    startup_and_configuration(test);
    analytic_translation(test, {}, 1000000);
    analytic_translation(test, {4000, 153, 0, 0, 0}, 1000000);
    analytic_translation(test, {4000, 239, 0, 0, 0}, 1000000);
    analytic_translation(test, {}, std::numeric_limits<std::uint64_t>::max() - 4 * 54 * 4000);
    rotating_force(test);
    gap_and_periods(test);
    variable_periods(test);
    feedback_and_bias(test);
    convergence_and_sign(test);
    feedback_during_gap(test);
    resets(test);
    numerical_rollback(test);
    reset_feedback_invariance(test);
    failures(test);
    std::cout << "IMU history + output predictor bytes: " << sizeof(History) + sizeof(Predictor) << '\n';
    return test.failures() == 0 ? 0 : 1;
}
