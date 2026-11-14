// SPDX-License-Identifier: BSD-3-Clause
#include <limits>

#include "../integrations/px4/module/HeightAiding.hpp"
#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{
using formal_eskf::Status;
using formal_eskf::test::Context;

template <typename Linalg> struct HeightFixture
{
    using aiding_type = formal_eskf::px4::HeightAiding<Linalg>;
    using value_type = typename Linalg::value_type;
    using Source = typename aiding_type::Source;
    aiding_type aiding;
    typename aiding_type::state_type state;
    typename aiding_type::covariance_type P = aiding_type::covariance_type::identity() * static_cast<value_type>(0.01);
    std::uint64_t now = 1000000U;
    value_type baro_altitude = 550, gnss_altitude = 500;
    bool quality = true, velocity_aiding = true;
    std::uint8_t calibration{};
    std::uint8_t reset_counter{};
    value_type reset_delta{};

    typename aiding_type::Result update(std::uint64_t time, typename aiding_type::Sample const & baro,
                                        typename aiding_type::Sample const & gps, bool velocity,
                                        typename aiding_type::state_type & estimate,
                                        typename aiding_type::covariance_type & covariance,
                                        formal_eskf::configuration::Ins::BiasUpdate const & permissions = {},
                                        typename aiding_type::Acceleration const & acceleration = {false, {0U, 0U}})
    {
        auto const result = aiding.update(time, baro, gps, velocity, estimate, covariance, permissions, acceleration);
        if (result.status == Status::success && result.height_resets != 0U)
        {
            reset_counter = static_cast<std::uint8_t>(reset_counter + result.height_resets);
            reset_delta = result.height_delta;
        }
        return result;
    }

    typename aiding_type::Result tick(bool barometer = true, bool gnss = true, std::uint64_t dt = 20000U)
    {
        now += dt;
        typename aiding_type::Sample baro{}, gps{};
        if (barometer)
        {
            baro = {now, 1, calibration, baro_altitude,
                    static_cast<value_type>(formal_eskf::px4::profile::height::barometer_variance)};
        }
        if (gnss)
        {
            gps = {now, 2, 0, gnss_altitude, 1, quality};
        }
        return update(now, baro, gps, velocity_aiding, state, P);
    }

    void start(Context & test)
    {
        test.expect(aiding.initialize(now, value_type{500}) == Status::success, "setup", "fixed origin");
        for (unsigned i = 0; i < 22; ++i)
        {
            test.expect(tick().status == Status::success, "setup", "qualified two-source startup");
        }
        test.expect(aiding.session(Source::barometer).active && aiding.session(Source::gnss).active &&
                        aiding.reference() == Source::gnss,
                    "setup", "both fuse, GNSS is the reference");
    }
};

template <typename Linalg> void run(Context & test)
{
    using F = HeightFixture<Linalg>;
    using Aiding = typename F::aiding_type;
    using Scalar = typename F::value_type;
    using Source = typename F::Source;
    auto near = [](Scalar const & a, Scalar const & b) { return std::abs(a - b) < static_cast<Scalar>(1e-3); };
    {
        F f;
        f.start(test);
        f.P = Aiding::covariance_type::identity();
        f.P(2, 9) = static_cast<Scalar>(0.2);
        f.P(9, 2) = static_cast<Scalar>(0.2);
        f.P(2, 12) = static_cast<Scalar>(0.2);
        f.P(12, 2) = static_cast<Scalar>(0.2);
        auto open = f;
        typename Aiding::Sample gps{f.now + 20000U, 2, 0, 501, 1};
        formal_eskf::configuration::Ins::BiasUpdate inhibited{{false, false, false}, {true, true, true}};
        auto result = f.update(gps.time_us, {}, gps, true, f.state, f.P, inhibited);
        auto comparison = open.update(gps.time_us, {}, gps, true, open.state, open.P);
        test.expect(result.fused == 1 && comparison.fused == 1 && f.state.b_a(0) == Scalar{0} &&
                        open.state.b_a(0) != Scalar{0} && f.state.b_g(0) != Scalar{0} && f.P(9, 9) == Scalar{1},
                    "bias permissions",
                    "height forwards inhibited accel gain without disabling gyro learning or fusion");
        auto innovation = f.aiding.vertical_innovation(Source::gnss);
        test.expect(innovation.active && innovation.innovation > Scalar{0} && innovation.variance > Scalar{0}, "health",
                    "height converts residual to prediction-minus-observation");
        gps.time_us += 20000U;
        gps.altitude = 1500;
        result = f.update(gps.time_us, {}, gps, true, f.state, f.P, inhibited);
        test.expect(result.rejected != 0 && f.aiding.vertical_innovation(Source::gnss).active, "health",
                    "rejected height retains evidence while its source is active");
    }
    for (bool barometer : {false, true})
    {
        F f;
        f.start(test);
        f.state.b_a(0) = Scalar{-1};
        f.state.b_g(1) = Scalar{1};
        formal_eskf::configuration::Ins::BiasUpdate inhibited{{false, false, false}, {false, false, false}};
        typename Aiding::Sample sample{f.now + 20000U, barometer ? 1U : 2U, 0,
                                       barometer ? f.baro_altitude : f.gnss_altitude, 1};
        auto result = f.update(sample.time_us, barometer ? sample : typename Aiding::Sample{},
                               barometer ? typename Aiding::Sample{} : sample, true, f.state, f.P, inhibited);
        test.expect(result.fused == 1 && f.state.b_a(0) == static_cast<Scalar>(-0.4) &&
                        f.state.b_g(1) == static_cast<Scalar>(0.15),
                    "bias limits", "each height source limits successful corrections even with learning inhibited");
        f.state.b_a(0) = Scalar{-1};
        sample.time_us += 20000U;
        sample.altitude += 1000;
        result = f.update(sample.time_us, barometer ? sample : typename Aiding::Sample{},
                          barometer ? typename Aiding::Sample{} : sample, true, f.state, f.P, inhibited);
        test.expect(result.rejected == 1 && f.state.b_a(0) == Scalar{-1}, "bias limits",
                    "rejected height does not invoke limiting");
    }
    for (bool barometer : {false, true})
    {
        F f;
        f.start(test);
        if (barometer)
        {
            for (unsigned tick = 0; tick < 51; ++tick)
            {
                test.expect(f.tick(true, false).status == Status::success, "setup", "barometer fallback");
            }
        }
        auto const source = barometer ? Source::barometer : Source::gnss;
        test.expect(f.aiding.reference() == source, "setup", "selected height reference");
        auto const now = f.now + 20000U;
        typename Aiding::Sample sample{now, barometer ? 1U : 2U, 0,
                                       (barometer ? f.baro_altitude : f.gnss_altitude) + Scalar{100}, 1};
        typename Aiding::Acceleration held{true, {now, now}};
        auto ordinary = f;
        auto const rejected =
            ordinary.update(now, barometer ? sample : typename Aiding::Sample{},
                            barometer ? typename Aiding::Sample{} : sample, true, ordinary.state, ordinary.P);
        auto const limited = f.update(now, barometer ? sample : typename Aiding::Sample{},
                                      barometer ? typename Aiding::Sample{} : sample, true, f.state, f.P, {}, held);
        auto const & aid = f.aiding.session(source);
        auto const innovation = f.aiding.vertical_innovation(source);
        test.expect(rejected.rejected == 1 && limited.fused == 1 && limited.height_resets == 0 &&
                        aid.innovation_limited && aid.fusion.test_ratio(0) > Scalar{1} &&
                        std::abs(aid.fusion_innovation) < std::abs(aid.fusion.innovation(0)) && innovation.active &&
                        innovation.innovation == -aid.fusion_innovation,
                    "acceleration fault", "both height sources bound rejected residuals and retain raw diagnostics");
        test.expect(!f.aiding.reset_required(now, {now, now - 500000U}) &&
                        f.aiding.reset_required(now, {now, now - 500001U}) && f.aiding.reset_required(now, {now, 0U}) &&
                        !f.aiding.reset_required(now),
                    "reset time", "strict 500 ms last-good boundary is independent of recent fusion and held fault");
        sample.time_us += 20000U;
        held.times = {sample.time_us, sample.time_us - 500001U};
        auto unqualified = f;
        auto invalid_sample = sample;
        invalid_sample.qualified = false;
        auto const blocked = unqualified.update(sample.time_us, barometer ? invalid_sample : typename Aiding::Sample{},
                                                barometer ? typename Aiding::Sample{} : invalid_sample, false,
                                                unqualified.state, unqualified.P, {}, held);
        test.expect(blocked.fused == 0 && blocked.height_resets == 0 && blocked.velocity_resets == 0 &&
                        unqualified.state.p_n(2) == f.state.p_n(2),
                    "permission", "bad acceleration never grants quality permission or reset without a source");
        auto invalid = f;
        auto future = held;
        future.times.update_time_us = sample.time_us + 1U;
        auto const failed = invalid.update(sample.time_us, {}, {}, true, invalid.state, invalid.P, {}, future);
        test.expect(failed.status == Status::out_of_range && invalid.state.p_n(2) == f.state.p_n(2) &&
                        invalid.reset_counter == f.reset_counter,
                    "time", "future health does not mutate the height transaction");
        f.state.v_n(2) = Scalar{2};
        auto const reset_counter = f.reset_counter;
        auto const reset = f.update(sample.time_us, barometer ? sample : typename Aiding::Sample{},
                                    barometer ? typename Aiding::Sample{} : sample, false, f.state, f.P, {}, held);
        test.expect(reset.fused == 1 && reset.height_resets == 1 &&
                        f.reset_counter == static_cast<std::uint8_t>(reset_counter + 1U) &&
                        !f.aiding.vertical_innovation(source).active && reset.velocity_resets == unsigned(barometer),
                    "recovery", "successful bounded fusion does not postpone independent reference-height reset");
        if (barometer)
        {
            test.expect(f.state.v_n(2) == Scalar{0} && f.P(5, 5) == Scalar{10}, "recovery",
                        "barometer reference resets Down velocity only when velocity aiding is absent");
        }
    }
    {
        F f;
        test.expect(f.aiding.initialize(f.now, 100000) == Status::domain_error && !f.aiding.valid(f.now), "startup",
                    "invalid external origin does not create height progress");
        test.expect(f.aiding.initialize(f.now, 500) == Status::success, "startup", "initialize metadata only");
        test.expect(!f.aiding.valid(f.now) && f.aiding.progress_time() == 0, "startup", "no invented fusion");
        auto result = f.tick();
        test.expect(result.height_resets == 1 && result.fused == 0 && f.reset_counter == 1 && near(f.P(2, 2), 1) &&
                        f.aiding.valid(f.now),
                    "startup", "first GNSS starts with a real height/covariance reset");
        for (unsigned i = 1; i < formal_eskf::px4::profile::height::barometer_warmup_samples; ++i)
        {
            test.expect(f.tick().status == Status::success, "startup", "barometer warmup");
        }
        test.expect(!f.aiding.session(Source::barometer).active, "startup", "strict sample-count boundary");
        result = f.tick();
        test.expect(f.aiding.session(Source::barometer).active && f.aiding.session(Source::barometer).fused_time == 0 &&
                        result.height_resets == 0 && near(f.aiding.session(Source::barometer).bias.value, 50),
                    "startup", "secondary aligns offset without a height reset or fake correction");
        test.expect(f.tick().fused == 2 && near(f.state.p_n(2), 0), "startup",
                    "different raw altitude datums agree after offset alignment");
    }
    {
        F f;
        f.start(test);
        auto const resets = f.reset_counter;
        test.expect(f.tick(true, false, 999999).status == Status::success && f.aiding.reference() == Source::gnss,
                    "timeout", "GNSS data grace is shorter than its fusion timeout");
        test.expect(f.tick(true, false, 1).status == Status::success && f.aiding.reference() == Source::barometer &&
                        !f.aiding.session(Source::gnss).active && f.aiding.valid(f.now) && f.reset_counter == resets,
                    "fallback", "at 1 s missing GNSS selects working baro without resetting height");
        auto const bias = f.aiding.session(Source::barometer).bias.value;
        auto const bias_variance = f.aiding.session(Source::barometer).bias.variance;
        for (unsigned i = 0; i < 10; ++i)
        {
            test.expect(f.tick(true, false).status == Status::success, "fallback", "baro reference remains active");
        }
        test.expect(f.aiding.session(Source::barometer).bias.value == bias &&
                        f.aiding.session(Source::barometer).bias.variance == bias_variance,
                    "bias", "reference bias and variance are frozen");
        f.quality = false;
        test.expect(f.tick().height_resets == 0 && !f.aiding.session(Source::gnss).active, "quality",
                    "unqualified returning GNSS cannot reset or start");
        f.quality = true;
        f.gnss_altitude += 5;
        auto const before = f.state.p_n(2);
        auto result = f.tick();
        test.expect(result.height_resets == 1 && f.aiding.reference() == Source::gnss && near(f.state.p_n(2), -5) &&
                        near(f.reset_delta, -5 - before),
                    "recovery", "qualified preferred source reclaims reference with a real reset");
        test.expect(near(f.aiding.session(Source::barometer).bias.value, bias - 5) &&
                        f.aiding.session(Source::gnss).bias.value == Scalar{0},
                    "reset", "height delta transports other-source bias; GNSS reference offset resets");
        test.expect(f.tick().height_resets == 0, "recovery", "normal fusion resumes without repeating reset");
    }
    {
        F f;
        f.start(test);
        auto const reset_count = f.reset_counter;
        f.tick(false, true, 399999);
        test.expect(f.aiding.session(Source::barometer).active, "baro timeout", "active before 400 ms");
        f.tick(false, true, 1);
        test.expect(!f.aiding.session(Source::barometer).active && f.aiding.reference() == Source::gnss &&
                        f.aiding.valid(f.now),
                    "baro timeout", "missing secondary does not invalidate GNSS height");
        f.baro_altitude += 25;
        test.expect(f.tick().height_resets == 0 && f.reset_counter == reset_count, "baro recovery",
                    "secondary re-entry aligns offset, not the navigation state");
        f.baro_altitude = std::numeric_limits<Scalar>::quiet_NaN();
        test.expect(f.tick().status == Status::success && !f.aiding.session(Source::barometer).active, "invalid",
                    "bad external altitude stops that source without estimator fault");
    }
    {
        F f;
        f.start(test);
        auto const resets = f.reset_counter;
        f.baro_altitude += 1000;
        for (unsigned i = 0; i < 251; ++i)
        {
            auto result = f.tick();
            test.expect(result.status == Status::success && result.height_resets == 0, "rejection",
                        "working GNSS prevents a height reset from bad barometer");
        }
        test.expect(f.aiding.session(Source::barometer).fault && !f.aiding.session(Source::barometer).active &&
                        f.reset_counter == resets && near(f.state.p_n(2), 0),
                    "rejection", "sustained baro rejection latches fault while GNSS continues");
        f.baro_altitude = 550;
        test.expect(f.tick().height_resets == 0 && !f.aiding.session(Source::barometer).active, "latch",
                    "a good reading alone cannot clear the fault latch");
        ++f.calibration;
        for (unsigned i = 0; i < 22; ++i)
        {
            test.expect(f.tick().status == Status::success, "calibration", "requalify changed barometer");
        }
        test.expect(f.aiding.session(Source::barometer).active && !f.aiding.session(Source::barometer).fault &&
                        f.reset_counter == resets,
                    "calibration", "calibration clears fault, reseeds LPF and restarts without height reset");
    }
    {
        F f;
        f.start(test);
        f.gnss_altitude += 1000;
        auto const resets = f.reset_counter;
        for (unsigned i = 0; i < 251; ++i)
        {
            test.expect(f.tick().height_resets == 0, "GNSS rejection", "baro progress prevents all-source timeout");
        }
        test.expect(!f.aiding.session(Source::gnss).active && f.aiding.reference() == Source::barometer &&
                        f.reset_counter == resets,
                    "GNSS rejection", "failed primary stops and falls back to active secondary");
    }
    {
        F f;
        f.start(test);
        f.baro_altitude += 1000;
        f.gnss_altitude += 1000;
        for (unsigned i = 0; i < 250; ++i)
        {
            test.expect(f.tick().height_resets == 0, "global timeout", "no reset at or before 5 s");
        }
        auto result = f.tick();
        test.expect(result.height_resets == 1 && near(f.state.p_n(2), -1000) && near(f.P(2, 2), 1) &&
                        f.aiding.valid(f.now),
                    "global timeout", "all-source rejection resets to fresh reference after 5 s");
        test.expect(f.aiding.session(Source::gnss).fusion.diagnostics_valid &&
                        !f.aiding.vertical_innovation(Source::gnss).active,
                    "health", "reset retains raw diagnostics but excludes their pre-reset innovation from health");
        f.gnss_altitude += 100;
        test.expect(f.tick().status == Status::success && f.aiding.vertical_innovation(Source::gnss).active, "health",
                    "the next height attempt supplies fresh evidence even when rejected");
    }
    {
        F f;
        f.start(test);
        f.quality = false;
        f.velocity_aiding = false;
        f.tick();
        f.state.v_n(0) = 2;
        f.state.v_n(1) = 3;
        f.state.v_n(2) = 4;
        f.baro_altitude += 1000;
        typename Aiding::Result result;
        for (unsigned i = 0; i < 251; ++i)
        {
            result = f.tick(true, false);
        }
        test.expect(result.height_resets == 1 && result.velocity_resets == 1 && near(result.velocity_delta, -4) &&
                        f.state.v_n(2) == Scalar{0} && near(f.P(5, 5), 10) && f.state.v_n(0) == Scalar{2} &&
                        f.state.v_n(1) == Scalar{3},
                    "baro reset", "baro fallback resets Down velocity only when no velocity aiding is active");
    }
    {
        F f;
        f.start(test);
        for (unsigned i = 0; i < 500; ++i)
        {
            f.baro_altitude += static_cast<Scalar>(0.005);
            test.expect(f.tick().status == Status::success, "drift", "slow barometer drift remains numerical");
        }
        test.expect(f.aiding.session(Source::barometer).bias.value > static_cast<Scalar>(50.5) &&
                        f.aiding.session(Source::gnss).bias.value == Scalar{0} &&
                        std::abs(f.state.p_n(2)) < static_cast<Scalar>(0.5),
                    "drift", "secondary offset learns drift while GNSS retains the altitude reference");
    }
    {
        F f;
        f.start(test);
        f.quality = false;
        f.tick();
        f.baro_altitude += 1000;
        f.state.v_n(2) = 4;
        for (unsigned i = 0; i < 250; ++i)
        {
            test.expect(f.tick(true, false).height_resets == 0, "combined reset", "reference timeout pending");
        }
        f.quality = true;
        f.gnss_altitude += 3;
        auto result = f.tick();
        test.expect(result.height_resets == 2 && result.velocity_resets == 0 && f.reset_counter == 3 &&
                        near(f.reset_delta, -3) && near(f.state.p_n(2), -3) && f.state.v_n(2) == Scalar{4},
                    "combined reset", "two real height resets accumulate delta; active velocity is not zeroed");
    }
    {
        F f;
        f.start(test);
        auto const progress = f.aiding.progress_time();
        auto const bias = f.aiding.session(Source::barometer).bias.value;
        auto const prior = f.state;
        f.P(0, 0) = std::numeric_limits<Scalar>::quiet_NaN();
        f.baro_altitude += 1;
        test.expect(f.tick().status == Status::non_finite_input && f.aiding.progress_time() == progress &&
                        f.aiding.session(Source::barometer).bias.value == bias &&
                        formal_eskf::linalg::max_abs(f.state.p_n - prior.p_n) == Scalar{0} && std::isnan(f.P(0, 0)),
                    "numerical failure", "production correction failure rolls back tentative offset/history/state");
    }
    {
        F f;
        f.start(test);
        auto const resets = f.reset_counter;
        auto const progress = f.aiding.progress_time();
        auto const sample_time = f.aiding.session(Source::gnss).sample_time;
        f.tick(false, false, 5000001);
        test.expect(!f.aiding.valid(f.now) && f.aiding.reference() == Source::none && f.reset_counter == resets &&
                        f.aiding.progress_time() == progress,
                    "outage", "no measurements means no reset, success or validity refresh");
        typename Aiding::Sample future{f.now + 50000, 2, 0, 500, 1};
        auto result = f.update(f.now + 1, {}, future, false, f.state, f.P);
        test.expect(result.status == Status::success && f.aiding.session(Source::gnss).sample_time == sample_time,
                    "time", "future sample is not consumed");
        result = f.update(future.time_us, {}, future, true, f.state, f.P);
        test.expect(result.height_resets == 1, "time", "future sample becomes usable at its timestamp");
        test.expect(f.update(future.time_us + 1, {}, future, true, f.state, f.P).height_resets == 0, "time",
                    "duplicate sample cannot reset twice");
        test.expect(f.update(future.time_us, {}, {}, true, f.state, f.P).status == Status::out_of_range, "time",
                    "reversed tick rejected");
        auto const old_tick_progress = f.aiding.progress_time();
        f.P(2, 2) = -1;
        test.expect(f.update(future.time_us + 2, {}, {}, true, f.state, f.P).status == Status::domain_error &&
                        f.aiding.progress_time() == old_tick_progress && f.P(2, 2) == Scalar{-1},
                    "failure", "bad prior preserves helper and estimator outputs");
    }
    {
        formal_eskf::px4::HeightBias<Scalar> bias;
        bias.variance = 1;
        bias.fuse(2, 3, 3);
        test.expect(near(bias.value, static_cast<Scalar>(0.5)) && near(bias.variance, static_cast<Scalar>(0.75)),
                    "bias", "scalar Kalman offset estimate has the expected analytical posterior");
        auto const old = bias.value;
        bias.fuse(1000, 1, 1);
        test.expect(bias.value == old, "bias", "bias innovation gate rejects a large offset jump");
        bias.predict(1000);
        test.expect(bias.variance <= Scalar{1} && bias.finite(), "bias", "bounded bias uncertainty");
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
