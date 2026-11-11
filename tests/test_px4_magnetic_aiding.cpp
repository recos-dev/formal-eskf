// SPDX-License-Identifier: BSD-3-Clause
#include "../integrations/px4/module/MagneticAiding.hpp"
#include "test_backend.hpp"
#include "test_support.hpp"

#include <limits>

namespace
{
using formal_eskf::Status;
using formal_eskf::test::Context;
using Lifecycle = formal_eskf::runtime::MagnetometerFusion;

template <typename Linalg> struct MagneticFixture
{
    using aiding_type = formal_eskf::px4::MagneticAiding<Linalg>;
    using value_type = typename Linalg::value_type;
    using vector_type = typename aiding_type::vector_type;
    aiding_type aiding;
    typename aiding_type::state_type state{};
    typename aiding_type::covariance_type P = aiding_type::covariance_type::identity() * static_cast<value_type>(0.01);
    vector_type reference = vector_type::from_row_major({30, 0, 40});
    vector_type sample = reference;
    typename aiding_type::source_type source{1, 0, 1};
    typename aiding_type::Context context{true, true, Lifecycle::FlightState::on_ground};
    std::uint64_t now = 1000000U;

    MagneticFixture() { aiding.initialize(source, now); }

    typename aiding_type::Reference reference_data() const { return {this->reference, this->reference, true}; }

    auto tick(bool received = true)
    {
        now += 10000U;
        typename aiding_type::Sample observation;
        if (received)
        {
            observation = {now, source, sample};
        }
        return aiding.update(now, observation, context, reference_data(), state, P);
    }

    void start(Context & test)
    {
        for (unsigned i = 0; i < 103; ++i)
        {
            test.expect(tick().status == Status::success, "adapter", "healthy startup succeeds");
        }
        test.expect(aiding.lifecycle().active && aiding.heading_aiding(), "adapter", "qualified startup fuses");
    }
};

template <typename Matrix> bool equal(Matrix const & a, Matrix const & b)
{
    for (std::size_t row = 0; row < Matrix::row_count; ++row)
    {
        for (std::size_t col = 0; col < Matrix::column_count; ++col)
        {
            if (a(row, col) != b(row, col))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename Linalg> void run(Context & test)
{
    using F = MagneticFixture<Linalg>;
    using Scalar = typename F::value_type;
    using Vector = typename F::vector_type;
    constexpr auto tolerance = static_cast<Scalar>(1e-4);
    {
        F f;
        f.start(test);
        f.P = F::aiding_type::covariance_type::identity();
        f.P.set(8, 9, static_cast<Scalar>(0.2));
        f.P.set(9, 8, static_cast<Scalar>(0.2));
        f.P.set(8, 12, static_cast<Scalar>(0.2));
        f.P.set(12, 8, static_cast<Scalar>(0.2));
        auto open = f;
        auto field = f.reference;
        field.set(1, Scalar{1});
        typename F::aiding_type::Sample sample{f.now + 10000U, f.source, field};
        formal_eskf::configuration::Ins::BiasUpdate inhibited{{false, false, false}, {true, true, true}};
        auto result = f.aiding.update(sample.time_us, sample, f.context, f.reference_data(), f.state, f.P, inhibited);
        auto comparison =
            open.aiding.update(sample.time_us, sample, open.context, open.reference_data(), open.state, open.P);
        test.expect(result.fused && comparison.fused && f.state.b_a(0) == Scalar{0} && open.state.b_a(0) != Scalar{0} &&
                        f.state.b_g(0) != Scalar{0} && f.P(9, 9) == Scalar{1},
                    "bias permissions", "magnetic correction forwards accel inhibition while gyro can learn");
    }
    {
        F f;
        for (unsigned i = 0; i < 101; ++i)
        {
            auto const result = f.tick();
            test.expect(!result.fused && !result.reset && f.aiding.progress_time() == 0, "startup",
                        "strict health duration does not invent fusion progress");
        }
        test.expect(f.tick().fused, "startup", "fusion starts strictly after one healthy second");
        auto const P = f.P;
        auto const q = f.state.q_nb;
        auto const progress = f.aiding.progress_time();
        f.sample = f.reference * Scalar{2};
        auto result = f.tick();
        test.expect(result.rejected && !result.fused && equal(P, f.P) &&
                        equal(q.coefficients(), f.state.q_nb.coefficients()),
                    "field", "disturbance preserves state/P");
        test.expect(f.aiding.lifecycle().active && !f.aiding.heading_aiding() && f.aiding.progress_time() == progress &&
                        f.aiding.observation().field.failures.strength,
                    "field", "field inhibition retains session without refreshing progress");
        f.sample = f.reference;
        test.expect(f.tick().fused, "field", "active session resumes on a good field without startup delay");
        f.sample.set(0, std::numeric_limits<Scalar>::quiet_NaN());
        result = f.tick();
        test.expect(result.status == Status::success && result.rejected && !f.aiding.lifecycle().active, "invalid",
                    "non-finite external observation stops aiding without estimator numerical fault");
    }
    {
        F f;
        f.start(test);
        auto const P = f.P;
        auto const progress = f.aiding.progress_time();
        typename F::aiding_type::Sample stale{f.now + 10000U, f.source, f.sample};
        f.now = stale.time_us + F::aiding_type::timing.maximum_interval_us;
        auto result = f.aiding.update(f.now, stale, f.context, f.reference_data(), f.state, f.P);
        test.expect(result.rejected && !result.fused && equal(P, f.P) && f.aiding.progress_time() == progress, "time",
                    "late arrival cannot update state or refresh fusion progress");
        f.now += F::aiding_type::timing.maximum_interval_us;
        result = f.aiding.update(f.now, {}, f.context, f.reference_data(), f.state, f.P);
        test.expect(result.status == Status::success && !f.aiding.lifecycle().active, "time",
                    "late arrival cannot postpone data timeout");
    }
    for (bool aided : {false, true})
    {
        F f;
        f.context.horizontal_aiding = aided;
        f.start(test);
        for (unsigned i = 0; i < 100; ++i)
        {
            test.expect(f.tick(false).status == Status::success, "outage", "empty ticks advance timeout");
        }
        test.expect(!f.aiding.lifecycle().active && f.aiding.aligned() == aided, "outage",
                    "no data stops session; alignment survives only with horizontal aiding");
        auto result = f.tick();
        test.expect(result.status == Status::success && result.reset == !aided && result.fused == aided, "outage",
                    "returning data resets lost alignment or resumes aided fusion");
        test.expect(f.aiding.reset_counter() == (aided ? 0 : 1), "outage", "only a successful reset increments count");
    }
    {
        F f;
        f.start(test);
        f.source.calibration_count++;
        for (unsigned i = 0; i < 3; ++i)
        {
            auto result = f.tick();
            test.expect(!result.reset && !result.fused && !f.aiding.aligned(), "source",
                        "ground calibration change waits for four new samples");
        }
        auto result = f.tick();
        test.expect(result.reset && !result.fused && f.aiding.aligned() && f.aiding.reset_counter() == 1, "source",
                    "qualified source change performs reset once, not correction");
        test.expect(f.tick().fused && f.aiding.reset_counter() == 1, "source", "next sample resumes ordinary fusion");
    }
    {
        F f;
        f.start(test);
        f.context.flight_state = Lifecycle::FlightState::unknown;
        f.context.at_rest = false;
        ++f.source.device_id;
        for (unsigned i = 0; i < 10; ++i)
        {
            auto result = f.tick();
            test.expect(!result.reset && !result.fused, "source",
                        "unknown flight context cannot authorize source recovery");
        }
        f.context.flight_state = Lifecycle::FlightState::in_air;
        test.expect(f.tick().fused && f.aiding.reset_counter() == 0, "source",
                    "aided in-flight switch preserves heading");
    }
    {
        F f;
        f.context.horizontal_aiding = false;
        f.start(test);
        ++f.source.device_id;
        f.context.flight_state = Lifecycle::FlightState::unknown;
        for (unsigned i = 0; i < 5; ++i)
        {
            auto const result = f.tick();
            test.expect(!result.fused && !result.reset && !f.aiding.aligned(), "source",
                        "lost heading cannot bypass unknown flight context during source recovery");
        }
        f.context.flight_state = Lifecycle::FlightState::on_ground;
        test.expect(f.tick().reset && f.aiding.reset_counter() == 1, "source",
                    "known context permits qualified source alignment");
    }
    {
        F f;
        f.start(test);
        f.context.at_rest = false;
        f.sample = Vector::from_row_major({0, 30, 40});
        for (unsigned i = 0; i < 30; ++i)
        {
            test.expect(f.tick().status == Status::success, "heading",
                        "heading disagreement is not a numerical failure");
        }
        test.expect(!f.aiding.heading().consistent && !f.aiding.heading_aiding(), "heading",
                    "filtered inconsistency inhibits aided navigation correction");
        auto const progress = f.aiding.progress_time();
        f.sample = f.reference;
        for (unsigned i = 0; i < 100; ++i)
        {
            test.expect(!f.tick().fused, "heading", "small innovation alone does not restore consistency");
        }
        test.expect(f.aiding.progress_time() == progress && f.aiding.reset_counter() == 0, "heading",
                    "inhibition does not fabricate progress or a yaw reset");
        for (unsigned i = 0; i < 200; ++i)
        {
            test.expect(f.aiding.update_acceleration(Vector::from_row_major({1, 0, 0}), static_cast<Scalar>(0.01)) ==
                            Status::success,
                        "heading", "actual prediction acceleration updates maneuver filter");
        }
        test.expect(f.tick().fused && f.aiding.heading().consistent, "heading",
                    "aided maneuver restores consistent fusion");
        f.aiding.reset_acceleration();
        test.expect(f.aiding.heading().horizontal_acceleration == 0 && f.aiding.heading().consistent, "heading",
                    "IMU gap clears maneuver evidence, not heading history");
    }
    {
        F f;
        f.context.horizontal_aiding = false;
        f.start(test);
        f.sample = Vector::from_row_major({0, 30, 40});
        unsigned resets = 0;
        for (unsigned i = 0; i < 710; ++i)
        {
            auto result = f.tick();
            test.expect(result.status == Status::success, "timeout", "persistent innovation rejection remains handled");
            resets += result.reset;
        }
        auto const mapped = formal_eskf::so3::rotate(f.state.q_nb, f.sample);
        auto const down = formal_eskf::so3::inverse_rotate(f.state.q_nb, Vector::from_row_major({0, 0, 1}));
        test.expect(resets == 1 && f.aiding.reset_counter() == 1 && f.aiding.aligned(), "timeout",
                    "unaided overdue rejection performs one real heading reset");
        test.expect(formal_eskf::test::near(mapped(0), Scalar{30}, tolerance) &&
                        formal_eskf::test::near(mapped(1), Scalar{0}, tolerance) &&
                        formal_eskf::test::near(down(2), Scalar{1}, tolerance),
                    "timeout", "reset aligns field while retaining tilt");
    }
    {
        F f;
        f.start(test);
        ++f.source.calibration_count;
        f.P.set(0, 0, Scalar{-1});
        auto const P = f.P;
        auto const q = f.state.q_nb;
        for (unsigned i = 0; i < 3; ++i)
        {
            f.tick();
        }
        auto const result = f.tick();
        test.expect(result.status != Status::success && !result.reset && f.aiding.reset_counter() == 0 &&
                        equal(P, f.P) && equal(q.coefficients(), f.state.q_nb.coefficients()),
                    "reset failure", "failed reset preserves estimate and notification count");
        test.expect(f.aiding.lifecycle().recovery_reason == Lifecycle::RecoveryReason::reset_failed, "reset failure",
                    "numerical failure latches repair requirement");
        f.P.set(0, 0, static_cast<Scalar>(0.01));
        ++f.source.device_id;
        for (unsigned i = 0; i < 5; ++i)
        {
            auto const retry = f.tick();
            test.expect(!retry.reset && !retry.fused && f.aiding.reset_counter() == 0, "reset failure",
                        "missing heading and a new sensor cannot bypass the numerical repair latch");
        }
    }
    {
        F f;
        f.start(test);
        auto const P = f.P;
        auto const time = f.now;
        typename F::aiding_type::Sample future{time + 20000U, f.source, f.sample};
        f.now += 10000U;
        test.expect(!f.aiding.update(f.now, future, f.context, f.reference_data(), f.state, f.P).fused && equal(P, f.P),
                    "time", "future sample waits without mutation");
        f.now += 10000U;
        test.expect(f.aiding.update(f.now, future, f.context, f.reference_data(), f.state, f.P).fused, "time",
                    "sample becomes eligible at its timestamp");
        f.now += 10000U;
        test.expect(!f.aiding.update(f.now, future, f.context, f.reference_data(), f.state, f.P).fused, "time",
                    "duplicate sample cannot fuse twice");
    }
}
} /* end namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    Context test;
    run<formal_eskf::test::Backend<float>>(test);
    run<formal_eskf::test::Backend<double>>(test);
    return test.failures() == 0 ? 0 : 1;
}
