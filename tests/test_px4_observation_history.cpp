// SPDX-License-Identifier: BSD-3-Clause
#include <bit>
#include <cstdint>
#include <cstdlib>
#include <limits>

#include "../integrations/px4/module/ObservationHistory.hpp"
#include "../integrations/px4/module/OutputPredictor.hpp"
#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{
using formal_eskf::test::Context;
using Backend = formal_eskf::test::Backend<double>;
using History = formal_eskf::px4::ObservationHistory<Backend>;
using Imu = formal_eskf::px4::ImuHistory<Backend>;
using Admission = History::Admission;
using Selection = History::Selection;
namespace profile = formal_eskf::px4::profile;

History::GnssSample gnss(std::uint64_t time)
{
    History::GnssSample sample;
    sample.stamp = {time, {17, 0}};
    sample.latitude_deg = 25.1;
    sample.longitude_deg = 121.5;
    sample.altitude_msl_m = 50;
    sample.altitude_ellipsoid_m = 60;
    sample.velocity_n = History::vector3_type::from_row_major({1, 2, 3});
    sample.horizontal_accuracy_m = 1;
    sample.vertical_accuracy_m = 2;
    sample.speed_accuracy_m_s = 0.1;
    sample.hdop = 0.7;
    sample.vdop = 0.9;
    sample.fix_type = 3;
    sample.satellites = 12;
    sample.velocity_valid = true;
    return sample;
}

History::MagneticSample magnetic(std::uint64_t time)
{
    return {{time, {29, 3}}, History::vector3_type::from_row_major({20, 1, 30})};
}

History::BarometerSample barometer(std::uint64_t time) { return {{time, {31, 4}}, 123.5}; }

Imu::input_type imu_input(std::uint64_t time, std::uint32_t angular = 4000, std::uint32_t acceleration = 4000)
{
    Imu::input_type sample;
    sample.time_us = time;
    sample.angular_period_us = angular;
    sample.acceleration_period_us = acceleration;
    sample.delta_velocity_b(2) = -profile::gravity_m_s2 * (static_cast<double>(acceleration) * 1e-6);
    sample.source = {41, 43, 0, 0};
    return sample;
}

// Fail fixture setup immediately; subsequent pointer-based assertions require an admitted interval.
void require(bool condition)
{
    if (!condition)
    {
        std::cerr << "observation-history fixture setup failed\n";
        std::abort();
    }
}

struct Session
{
    Imu imu;
    History history;
    std::uint64_t epoch = 1000000;
    std::uint64_t time = epoch;
    History::LandState land{epoch, true, true, true};
    History::VehicleState vehicle{epoch, true, false};

    void seed(formal_eskf::px4::TimingParameters const & parameters = {})
    {
        time = epoch;
        land = {epoch, true, true, true};
        vehicle = {epoch, true, false};
        require(imu.configure(parameters) && history.configure(parameters) && imu.seed(imu_input(time), time));
        require(history.seed({{epoch, land, vehicle}, {epoch, {17, 0}}, {epoch, {29, 3}}, {epoch, {31, 4}}}));
    }

    void tick(std::uint32_t elapsed = 4000, std::uint32_t angular = 4000, std::uint32_t acceleration = 4000)
    {
        time += elapsed;
        require(imu.push(imu_input(time, angular, acceleration), time));
        require(imu.latest() != nullptr && history.record_imu(*imu.latest(), land, vehicle));
        if (auto const * delayed = imu.delayed())
        {
            require(history.begin_fusion(*delayed) && imu.complete(delayed->time_us));
        }
    }

    void advance(std::uint64_t target)
    {
        while (time < target)
        {
            tick();
        }
        require(time == target);
    }

    void fuse_until(std::uint64_t target)
    {
        while (history.fusion_time_us() < target)
        {
            tick();
        }
        require(history.fusion_time_us() == target);
    }
};

void configuration_and_seed(Context & test)
{
    History history;
    History::Seed seed{{1000000, {999000, true, true, true}, {999500, true, false}}};
    test.expect(!history.seed(seed) && history.push_gnss(gnss(1006000), 1006000).decision == Admission::not_ready,
                "startup", "configuration and alignment epoch are required");
    test.expect(!history.configure({10000, 200, 0, 0, 0}) && history.configure({4000, 200, 12, 23, 34}), "startup",
                "only supported mode with valid delay coverage");
    auto bad = seed;
    bad.gnss = {1000001, {17, 0}};
    test.expect(!history.seed(bad) && history.fusion_time_us() == 0, "seed",
                "future source watermark rejected atomically");
    test.expect(history.seed(seed) && !history.seed(seed) && !history.configure({}), "seed",
                "no reseed or live reconfiguration");
    auto const context = history.context(1000000, 3000000);
    test.expect(context.land.known && context.land.at_rest && context.vehicle.known && !context.vehicle.armed, "seed",
                "retain context with original timestamps");
    history.reset();
    test.expect(history.fusion_time_us() == 0 && history.half_period_us() == 2000 &&
                    !history.context(1000000, 3000000).land.known && history.seed(seed),
                "restart", "clear temporal state and retain configuration");
    auto sample = gnss(1030000);
    test.expect(history.push_gnss(sample, sample.stamp.time_us).decision == Admission::queued, "restart",
                "new source after restart can queue");
}

void selection_and_payload(Context & test, std::uint64_t epoch,
                           formal_eskf::px4::TimingParameters const & parameters = {})
{
    Session session;
    session.epoch = epoch;
    session.seed(parameters);
    auto g = gnss(epoch + 6000);
    g.velocity_valid = false;
    g.spoofed = true;
    g.horizontal_accuracy_m = std::numeric_limits<double>::quiet_NaN();
    test.expect(session.history.push_gnss(g, epoch + 6100).decision == Admission::queued &&
                    session.history.push_magnetic(magnetic(epoch + 6001), epoch + 6100).decision == Admission::queued &&
                    session.history.push_barometer(barometer(epoch + 6000), epoch + 6100).decision == Admission::queued,
                "arrival", "buffer timestamps and original metadata without inventing a quality pass");
    History::GnssObservation output;
    output.time_us = 123;
    test.expect(session.history.pop_gnss(output).decision == Selection::not_ready && output.time_us == 123, "warmup",
                "no consumption before delayed fusion begins");
    session.fuse_until(epoch + 4000);
    auto selected = session.history.pop_gnss(output);
    test.expect(selected.decision == Selection::selected && selected.retired == 0 && output.time_us == epoch + 4000 &&
                    output.sample.stamp.time_us == epoch + 6000 && output.arrival_time_us == epoch + 6100 &&
                    output.half_period_us == 2000 && output.residual_delay_us == 0 &&
                    output.sample.stamp.source.device_id == 17 && output.sample.latitude_deg == g.latitude_deg &&
                    output.sample.longitude_deg == g.longitude_deg && output.sample.altitude_msl_m == 50 &&
                    output.sample.altitude_ellipsoid_m == 60 && output.sample.velocity_n(2) == 3 &&
                    !output.sample.velocity_valid && output.sample.spoofed &&
                    std::bit_cast<std::uint64_t>(output.sample.horizontal_accuracy_m) ==
                        std::bit_cast<std::uint64_t>(g.horizontal_accuracy_m),
                "selection", "one GNSS event preserves provenance, flags and numerical payload for all consumers");
    test.expect(session.history.pop_gnss(output).decision == Selection::repeated && output.time_us == epoch + 4000,
                "selection", "height cannot pop the same GNSS source a second time in a step");
    History::MagneticObservation mag;
    mag.time_us = 456;
    test.expect(session.history.pop_magnetic(mag).decision == Selection::empty && mag.time_us == 456, "selection",
                "future effective timestamp remains pending");
    History::BarometerObservation baro;
    test.expect(session.history.pop_barometer(baro).decision == Selection::selected &&
                    baro.sample.altitude_m == 123.5 && baro.sample.stamp.source.calibration_count == 4,
                "selection", "barometer selection is independent from GNSS");
    session.tick();
    test.expect(session.history.pop_magnetic(mag).decision == Selection::selected && mag.time_us == epoch + 4001 &&
                    mag.sample.field_b(0) == 20 && mag.sample.stamp.source.calibration_count == 3,
                "selection", "pending future event becomes eligible on the next step");
    test.expect(session.history.push_gnss(g, session.time).decision == Admission::out_of_order, "watermark",
                "selection does not erase the arrival watermark");
}

void arrival_limits(Context & test)
{
    enum class Case
    {
        zero,
        future,
        duplicate,
        older,
        stale,
        too_fast,
        paused_imu,
        backward_clock,
        invalid_source
    };
    for (auto scenario : {Case::zero, Case::future, Case::duplicate, Case::older, Case::stale, Case::too_fast,
                          Case::paused_imu, Case::backward_clock, Case::invalid_source})
    {
        Session session;
        session.seed();
        session.advance(1400000);
        auto input = gnss(session.time);
        auto now = session.time;
        Admission expected = Admission::invalid_time;
        switch (scenario)
        {
        case Case::zero:
            input.stamp.time_us = 0;
            break;
        case Case::future:
            input.stamp.time_us = now + 1;
            break;
        case Case::duplicate:
            input.stamp.time_us = session.epoch;
            expected = Admission::out_of_order;
            break;
        case Case::older:
            input.stamp.time_us = session.epoch - 1;
            expected = Admission::out_of_order;
            break;
        case Case::stale:
            input.stamp.time_us = now - 150000;
            expected = Admission::stale;
            break;
        case Case::too_fast:
            require(session.history.push_gnss(gnss(now - 10000), now).decision == Admission::queued);
            expected = Admission::too_fast;
            break;
        case Case::paused_imu:
            now += 100000;
            input.stamp.time_us = now;
            expected = Admission::imu_stale;
            break;
        case Case::backward_clock:
            --now;
            break;
        case Case::invalid_source:
            input.stamp.source.device_id = 0;
            expected = Admission::invalid_source;
            break;
        }
        auto const result = session.history.push_gnss(input, now);
        test.expect(result.decision == expected && !result.overwritten, "arrival", "explicit rejection reason");
    }
    Session session;
    session.seed();
    auto result = session.history.push_gnss(gnss(session.epoch + 1000), session.epoch + 1000);
    test.expect(result.decision == Admission::before_seed, "alignment", "pre-seed effective sample is not replayed");
    session.advance(1400000);
    test.expect(
        session.history.push_gnss(gnss(session.time - 149999), session.time).decision == Admission::queued &&
            session.history.push_magnetic(magnetic(session.time - 99999), session.time).decision == Admission::queued &&
            session.history.push_barometer(barometer(session.time - 100000), session.time).decision == Admission::stale,
        "age", "strict source-specific arrival age bounds");
    Session early;
    early.epoch = 1000;
    early.seed({4000, 200, 20, 0, 0});
    test.expect(early.history.push_gnss(gnss(3000), 3000).decision == Admission::offset_underflow, "offset",
                "delay subtraction cannot wrap unsigned time");
}

void source_switches(Context & test)
{
    Session session;
    session.seed();
    require(session.history.push_gnss(gnss(1006000), 1006000).decision == Admission::queued);
    require(session.history.push_magnetic(magnetic(1006000), 1006000).decision == Admission::queued);
    require(session.history.push_barometer(barometer(1006000), 1006000).decision == Admission::queued);
    auto g = gnss(1007000);
    g.stamp.source.device_id = 18;
    auto m = magnetic(1007000);
    ++m.stamp.source.calibration_count;
    auto b = barometer(1007000);
    b.stamp.source.device_id = 0;
    auto const gr = session.history.push_gnss(g, 1007000);
    auto const mr = session.history.push_magnetic(m, 1007000);
    auto const br = session.history.push_barometer(b, 1007000);
    test.expect(gr.decision == Admission::too_fast && gr.source_changed && gr.source_discarded == 1 &&
                    mr.decision == Admission::too_fast && mr.source_changed && mr.source_discarded == 1 &&
                    br.decision == Admission::invalid_source && br.source_changed && br.source_discarded == 1,
                "source", "switch/unavailable indications clear old-generation data even without a queued replacement");
    test.expect(session.history.gnss_generation() == 1 && session.history.magnetic_generation() == 1 &&
                    session.history.barometer_generation() == 1 && session.history.barometer_source().device_id == 0,
                "source", "generation identity advances even when replacement payload is rejected");
    auto old = magnetic(1006000);
    test.expect(session.history.push_magnetic(old, 1007000).decision == Admission::out_of_order, "source",
                "old arrival cannot switch back to the preceding calibration");
    session.fuse_until(1008000);
    History::GnssObservation go;
    History::MagneticObservation mo;
    History::BarometerObservation bo;
    test.expect(session.history.pop_gnss(go).decision == Selection::empty &&
                    session.history.pop_magnetic(mo).decision == Selection::empty &&
                    session.history.pop_barometer(bo).decision == Selection::empty,
                "source", "no pending event survives the source invalidation");
    m.stamp.time_us = session.time;
    auto const restored = session.history.push_magnetic(m, session.time);
    test.expect(restored.decision == Admission::queued && !restored.source_changed, "source",
                "next admissible event keeps the new source identity");
    auto const time = session.time - session.history.half_period_us();
    session.fuse_until(time + 2000);
    test.expect(session.history.pop_magnetic(mo).decision == Selection::selected && mo.generation == 1, "source",
                "selection preserves the generation attached at admission");
}

void source_monitoring_during_rejection(Context & test)
{
    Session paused;
    paused.seed();
    auto sample = magnetic(paused.epoch + 99999);
    require(paused.history.push_magnetic(sample, sample.stamp.time_us).decision == Admission::queued);
    ++sample.stamp.time_us;
    ++sample.stamp.source.device_id;
    auto const result = paused.history.push_magnetic(sample, sample.stamp.time_us);
    test.expect(result.decision == Admission::imu_stale && result.source_changed && result.source_discarded == 1,
                "pause", "at 100 ms stop queueing but still process a source change");

    Session stale;
    stale.seed();
    stale.advance(1400000);
    require(stale.history.push_magnetic(magnetic(1399000), 1400000).decision == Admission::queued);
    sample = magnetic(1399001);
    ++sample.stamp.source.calibration_count;
    auto const rejected = stale.history.push_magnetic(sample, 1499001);
    test.expect(rejected.decision == Admission::stale && rejected.source_changed && rejected.source_discarded == 1,
                "source", "stale calibration change still retires the old generation");
}

void clock_rejection_preserves_pending(Context & test)
{
    Session session;
    session.seed();
    auto sample = magnetic(1006000);
    require(session.history.push_magnetic(sample, 1008000).decision == Admission::queued);
    sample.stamp = {1007000, {99, 9}};
    auto const backward = session.history.push_magnetic(sample, 1007000);
    sample.stamp.time_us = 1009000;
    auto const future = session.history.push_magnetic(sample, 1008000);
    test.expect(backward.decision == Admission::invalid_time && !backward.source_changed &&
                    future.decision == Admission::invalid_time && !future.source_changed,
                "clock", "invalid clocks cannot change source identity or consume a future watermark");
    session.fuse_until(1004000);
    History::MagneticObservation output;
    test.expect(session.history.pop_magnetic(output).decision == Selection::selected &&
                    output.sample.stamp.source.device_id == 29 && output.time_us == 1004000,
                "clock", "pending valid generation survives rejected clock input");
}

void selection_retirement(Context & test)
{
    for (bool exact_expiry : {false, true})
    {
        Session session;
        session.seed();
        std::uint64_t const effective = 1004000U + (exact_expiry ? 0U : 1U);
        require(session.history.push_gnss(gnss(effective + 2000), effective + 2000).decision == Admission::queued);
        session.fuse_until(1104000);
        History::GnssObservation output;
        output.time_us = 99;
        auto const result = session.history.pop_gnss(output);
        test.expect(result.decision == (exact_expiry ? Selection::empty : Selection::selected) &&
                        result.retired == (exact_expiry ? 1U : 0U) &&
                        output.time_us == (exact_expiry ? 99U : static_cast<std::uint64_t>(effective)),
                    "expiry", "age is strictly below 100 ms; miss preserves caller output");
    }
    Session session;
    session.seed();
    for (std::uint64_t time : {1050000U, 1100000U, 1150000U})
    {
        session.advance(session.epoch + (time - session.epoch) / 4000 * 4000);
        require(session.history.push_gnss(gnss(time), time).decision == Admission::queued);
    }
    session.fuse_until(1104000);
    History::GnssObservation output;
    auto const result = session.history.pop_gnss(output);
    test.expect(result.decision == Selection::selected && result.retired == 1 && output.time_us == 1098000, "selection",
                "newest eligible event supersedes older input while keeping future events");
    session.fuse_until(1148000);
    test.expect(session.history.pop_gnss(output).decision == Selection::selected && output.time_us == 1148000,
                "selection", "future event retained through the earlier pop");
}

void period_average_and_frozen_offsets(Context & test)
{
    Session session;
    session.seed({4000, 200, 12, 23, 0});
    for (std::size_t tick = 0; tick < 53; ++tick)
    {
        session.tick(4000, 4200, 4200);
    }
    test.expect(session.history.half_period_us() == 2000, "period",
                "latest IMU does not advance delayed timing average");
    require(session.history.push_barometer(barometer(1214000), 1214000).decision == Admission::queued);
    session.tick(4000, 4200, 4200);
    test.expect(session.history.half_period_us() == 2001, "period",
                "delayed real periods update the 0.99/0.01 average");
    test.expect(session.history.push_barometer(barometer(1219000), 1219000).decision == Admission::too_fast &&
                    session.history.push_barometer(barometer(1219001), 1219001).decision == Admission::queued,
                "rate", "spacing uses frozen effective timestamps, not raw sample spacing");
    auto invalid = *session.imu.latest();
    invalid.start_time_us = session.history.fusion_time_us();
    invalid.time_us = invalid.start_time_us + 4000;
    invalid.sample.time_us = invalid.time_us;
    auto const before = session.history.half_period_us();
    test.expect(!session.history.begin_fusion(invalid) && session.history.half_period_us() == before, "period",
                "second delayed step at the same latest IMU endpoint is rejected");
    session.fuse_until(1212000);
    History::BarometerObservation output;
    test.expect(session.history.pop_barometer(output).decision == Selection::selected && output.time_us == 1212000 &&
                    output.half_period_us == 2000 && output.sample.stamp.time_us == 1214000,
                "offset", "later average changes never retime an already queued event");
    auto g = gnss(session.time);
    auto m = magnetic(session.time);
    require(session.history.push_gnss(g, session.time).decision == Admission::queued);
    require(session.history.push_magnetic(m, session.time).decision == Admission::queued);
    auto const expected_half = session.history.half_period_us();
    session.fuse_until(session.time);
    History::GnssObservation go;
    History::MagneticObservation mo;
    test.expect(session.history.pop_gnss(go).decision == Selection::selected && go.residual_delay_us == 12000 &&
                    go.time_us == g.stamp.time_us - 12000 - expected_half &&
                    session.history.pop_magnetic(mo).decision == Selection::selected && mo.residual_delay_us == 23000 &&
                    mo.time_us == m.stamp.time_us - 23000 - expected_half,
                "offset", "per-source delay and half period are subtracted exactly once");
}

void prediction_context(Context & test)
{
    Session session;
    session.seed();
    for (unsigned i = 0; i < 53; ++i)
    {
        session.tick();
    }
    session.time += 4000;
    require(session.imu.push(imu_input(session.time), session.time));
    require(session.history.record_imu(*session.imu.latest(), {session.time, true, false, false},
                                       {session.time, true, true}));
    auto const * interval = session.imu.delayed();
    require(interval != nullptr);
    auto const historical = session.history.prediction_context(*interval, 3000000);
    test.expect(historical.land.known && historical.land.at_rest && !historical.vehicle.armed &&
                    session.history.fusion_time_us() == session.epoch &&
                    !session.history.context(interval->time_us, 3000000).land.known,
                "prediction-context",
                "next admitted delayed interval can read historical context without advancing selection");
    auto invalid = *interval;
    ++invalid.start_time_us;
    test.expect(!session.history.prediction_context(invalid, 3000000).land.known &&
                    !session.history.prediction_context(*session.imu.latest(), 3000000).land.known,
                "prediction-context", "wrong start or latest interval cannot claim delayed prediction context");
    require(session.history.begin_fusion(*interval));
    test.expect(!session.history.prediction_context(*interval, 3000000).land.known &&
                    session.history.context(interval->time_us, 3000000).land.at_rest,
                "prediction-context",
                "completed step switches to ordinary context lookup without granting a second prediction");
}

void historical_context(Context & test)
{
    Session session;
    session.seed();
    session.land = {1005000, true, false, false, true}; // Newer than the first endpoint.
    session.vehicle = {1004000, true, true, true};
    session.tick();
    session.tick();
    session.land = {1005000, true, true, true}; // Same timestamp is the same immutable message.
    session.fuse_until(1004000);
    auto first = session.history.context(1004000, 3000000);
    test.expect(first.time_us == 1004000 && first.land.time_us == 1000000 && first.land.landed &&
                    first.vehicle.time_us == 1004000 && first.vehicle.armed && first.vehicle.fixed_wing &&
                    !first.land.in_ground_effect,
                "context", "future message cannot replace information known at an earlier endpoint");
    session.tick();
    auto second = session.history.context(1008000, 3000000);
    test.expect(second.land.time_us == 1005000 && !second.land.landed && !second.land.at_rest &&
                    second.land.in_ground_effect,
                "context", "same timestamp cannot rewrite an already observed message");
    auto stale = session.history.context(1008000, 3000);
    test.expect(!stale.land.known && stale.land.time_us == 1005000 && !stale.vehicle.known, "context",
                "freshness uses original message time, with strict age boundary");
    test.expect(!session.history.context(1008001, 3000000).land.known &&
                    !session.history.context(999999, 3000000).land.known,
                "context", "future fusion context and pre-seed context are unknown");
    session.land = {1010000, true, true, true}; // Arrives after the corresponding endpoint was recorded.
    session.tick();
    auto historical = session.history.context(1012000, 3000000);
    test.expect(!historical.land.landed && historical.land.time_us == 1005000, "context",
                "late-arriving old timestamp does not backfill previously recorded snapshots");
    auto const known_at = session.time;
    session.fuse_until(known_at);
    auto current = session.history.context(known_at, 3000000);
    test.expect(current.land.landed && current.land.time_us == 1010000, "context",
                "late message becomes available only from the endpoint where it was known");
    session.fuse_until(1600000);
    test.expect(!session.history.context(1004000, 3000000).land.known &&
                    session.history.context(1600000, 3000000).land.time_us == 1010000,
                "context", "ring wrap drops old snapshots without refreshing message age");
}

void gap_context_and_unknown(Context & test)
{
    Session session;
    session.seed();
    session.land = {1030000, true, false, false};
    session.tick(50000);
    session.fuse_until(1050000);
    test.expect(session.history.half_period_us() == 2000 && session.history.context(1035000, 3000000).land.landed &&
                    !session.history.context(1050000, 3000000).land.landed,
                "gap", "held segment adds neither a timing observation nor fabricated context snapshots");
    auto const before = session.history.fusion_time_us();
    test.expect(!session.history.record_imu(*session.imu.latest(), {}, {}) &&
                    session.history.fusion_time_us() == before,
                "context", "repeated IMU endpoint cannot overwrite context");
    auto const unknown_at = session.time + 4000;
    session.land = {unknown_at, false, true, true};
    session.vehicle = {unknown_at, false, false};
    session.tick();
    session.fuse_until(unknown_at);
    auto const context = session.history.context(unknown_at, 3000000);
    test.expect(!context.land.known && !context.vehicle.known && context.land.time_us == unknown_at, "context",
                "explicit unknown overrides earlier known state despite a fresh timestamp");
}

void bounded_overflow(Context & test)
{
    Session session;
    session.seed();
    std::size_t gnss_count = 0, mag_count = 0, baro_count = 0;
    std::size_t gnss_loss = 0, mag_loss = 0, baro_loss = 0;
    for (std::size_t tick = 1; tick <= 600; ++tick)
    {
        session.tick();
        if (tick % 13 == 0)
        {
            auto const result = session.history.push_gnss(gnss(session.time), session.time);
            require(result.decision == Admission::queued);
            ++gnss_count;
            gnss_loss += result.overwritten;
        }
        if (tick % 3 == 0)
        {
            auto const result = session.history.push_magnetic(magnetic(session.time), session.time);
            require(result.decision == Admission::queued);
            ++mag_count;
            mag_loss += result.overwritten;
        }
        if (tick % 2 == 0)
        {
            auto const result = session.history.push_barometer(barometer(session.time), session.time);
            require(result.decision == Admission::queued);
            ++baro_count;
            baro_loss += result.overwritten;
        }
    }
    test.expect(gnss_loss == gnss_count - profile::history::gnss_capacity &&
                    mag_loss == mag_count - profile::history::magnetic_capacity &&
                    baro_loss == baro_count - profile::history::barometer_capacity,
                "capacity", "each full-ring overwrite is reported independently across wraps");
    History::GnssObservation g;
    History::MagneticObservation m;
    History::BarometerObservation b;
    auto const epoch = session.history.fusion_time_us();
    test.expect(session.history.pop_gnss(g).decision == Selection::selected &&
                    g.time_us == session.epoch + ((epoch - session.epoch + 2000) / 52000) * 52000 - 2000 &&
                    session.history.pop_magnetic(m).decision == Selection::selected &&
                    m.time_us == session.epoch + ((epoch - session.epoch + 2000) / 12000) * 12000 - 2000 &&
                    session.history.pop_barometer(b).decision == Selection::selected &&
                    b.time_us == session.epoch + ((epoch - session.epoch + 2000) / 8000) * 8000 - 2000,
                "wrap", "select correct newest eligible event after multiple wraps");
}
} // namespace

int main()
{
    formal_eskf::test::configure_backend_test();
    constexpr auto storage = sizeof(History) + sizeof(Imu) + sizeof(formal_eskf::px4::OutputPredictor<Backend>);
    static_assert(storage <= profile::history::storage_budget_bytes);
    static_assert((profile::history::context_capacity - 1) * profile::history::minimum_period_us >=
                  profile::history::maximum_lag_us + profile::history::observation_maximum_age_us +
                      profile::imu::maximum_interval_us);
    Context test;
    configuration_and_seed(test);
    selection_and_payload(test, 1000000);
    selection_and_payload(test, 1000000, {4000, 153, 0, 0, 0});
    selection_and_payload(test, 1000000, {4000, 239, 0, 0, 0});
    selection_and_payload(test, std::numeric_limits<std::uint64_t>::max() - 1000000);
    arrival_limits(test);
    source_switches(test);
    source_monitoring_during_rejection(test);
    clock_rejection_preserves_pending(test);
    selection_retirement(test);
    period_average_and_frozen_offsets(test);
    prediction_context(test);
    historical_context(test);
    gap_context_and_unknown(test);
    bounded_overflow(test);
    std::cout << "IMU + output + observation/context history bytes: " << storage << '\n';
    return test.failures() == 0 ? 0 : 1;
}
