// SPDX-License-Identifier: BSD-3-Clause
#include "../integrations/px4/module/MagneticAiding.hpp"
#include "../integrations/px4/module/MagneticModel.hpp"
#include "test_backend.hpp"
#include "test_support.hpp"

#include <limits>

namespace
{
using formal_eskf::Status;
using formal_eskf::test::Context;

template <typename Linalg> void run(Context & test)
{
    using Model = formal_eskf::px4::MagneticModel<Linalg>;
    using Aiding = formal_eskf::px4::MagneticAiding<Linalg>;
    using Scalar = typename Model::value_type;
    using Vector = typename Model::vector_type;
    auto const pi = formal_eskf::scalar::pi<typename Linalg::scalar_math_type>();
    auto const tolerance = static_cast<Scalar>(2e-5);
    auto const seed = Vector::from_row_major({30, 0, 40});
    typename Model::sample_type const initial{0, std::atan2(Scalar{40}, Scalar{30}), 50};
    auto near = [=](Scalar const & a, Scalar const & b) { return std::abs(a - b) <= tolerance; };

    for (std::uint32_t mask = 0; mask <= 3; ++mask)
    {
        for (bool available : {false, true})
        {
            Model model;
            auto candidate = initial;
            candidate.declination = static_cast<Scalar>(0.3);
            std::optional<typename Model::sample_type> geographic;
            if (available)
            {
                geographic = candidate;
            }
            test.expect(model.initialize(1, {mask, static_cast<Scalar>(-0.2)}, seed, geographic) == Status::success,
                        "selection", "each source-mask/model-availability combination initializes");
            bool const use_geo = available && (mask == 1 || mask == 3);
            bool const use_saved = !use_geo && (mask == 2 || mask == 3);
            auto const expected = use_geo     ? static_cast<Scalar>(0.3)
                                  : use_saved ? static_cast<Scalar>(-0.2)
                                              : Scalar{0};
            auto const source = use_geo     ? Model::HeadingSource::geographic
                                : use_saved ? Model::HeadingSource::saved
                                            : Model::HeadingSource::magnetic_north;
            test.expect(near(model.declination(), expected) && model.heading_source() == source &&
                            near(std::atan2(model.observation_n()(1), model.observation_n()(0)), expected),
                        "selection", "geographic, stored, then magnetic-north precedence");
            test.expect(model.available() == available && model.reference_id() == 1, "selection",
                        "health availability does not depend on heading source");
            auto const observation = model.observation_n();
            candidate.strength = 55;
            auto const updated = model.update(10000002, candidate, true);
            test.expect(updated.changed && model.available(), "selection",
                        "WMM still updates with geographic heading disabled");
            test.expect(model.reference_id() == ((mask & 1U) != 0U ? 2U : 1U), "selection",
                        "only geographic-heading adoption changes the observation revision");
            if ((mask & 1U) == 0U)
            {
                test.expect(formal_eskf::linalg::max_abs(model.observation_n() - observation) == Scalar{0}, "selection",
                            "health update cannot rewrite stored/zero-declination observation");
            }
        }
    }
    {
        Model model;
        test.expect(model.initialize(10, {}, seed, initial) == Status::success, "schedule", "initial model adoption");
        test.expect(!model.lookup_due(10000010, true) && model.lookup_due(10000011, true) &&
                        !model.lookup_due(10000011, false) && model.lookup_due(11, false, true),
                    "schedule", "strict 10 s, valid-position and origin-trigger boundaries");
        auto candidate = initial;
        candidate.strength = 51;
        auto result = model.update(10000011, candidate, true);
        test.expect(result.checked && !result.changed && model.model_revision() == 1, "schedule",
                    "threshold equality advances only the check clock");
        result = model.update(20000012, std::nullopt, true);
        test.expect(result.checked && !result.changed && model.available() && !model.lookup_due(20000013, true),
                    "unavailable", "failed lookup retains last valid model and does not retry each IMU tick");
        auto const old_field = model.health_n();
        candidate.strength = std::numeric_limits<Scalar>::quiet_NaN();
        result = model.update(30000013, candidate, true);
        test.expect(result.status == Status::non_finite_input && !result.changed && model.reference_id() == 1 &&
                        formal_eskf::linalg::max_abs(model.health_n() - old_field) == Scalar{0},
                    "invalid", "invalid model cannot change either reference or authorize a reset");
        test.expect(!model.lookup_due(30000014, true), "invalid", "invalid candidate attempts are rate-limited");
        candidate = initial;
        candidate.strength = 52;
        result = model.update(30000014, candidate, false, true);
        test.expect(result.changed && model.reference_id() == 2, "origin", "origin trigger bypasses periodic wait");
        auto const checks = model.lookup_count();
        test.expect(model.update(30000014, candidate, true).status == Status::out_of_range &&
                        model.lookup_count() == checks,
                    "time", "duplicate calls cannot change metadata");
        test.expect(model.initialize(20, {4, 0}, seed, initial) == Status::domain_error && model.reference_id() == 2,
                    "settings", "invalid source settings preserve an existing model");
    }
    {
        typename Model::sample_type converted{}, round_trip{};
        auto const status = Model::to_origin_frame(initial, 35, 179.8, -10, -179.9, converted);
        test.expect(status == Status::success && near(converted.strength, initial.strength), "frame",
                    "rotation preserves strength across longitude wrap");
        test.expect(Model::to_origin_frame(converted, -10, -179.9, 35, 179.8, round_trip) == Status::success &&
                        near(round_trip.declination, initial.declination) &&
                        near(round_trip.inclination, initial.inclination),
                    "frame", "local NED to origin NED round trip");
        typename Model::sample_type down{0, pi / Scalar{2}, 40};
        test.expect(Model::to_origin_frame(down, 45, 0, 0, 0, converted) == Status::success &&
                        near(converted.inclination, pi / Scalar{4}) && near(std::abs(converted.declination), pi),
                    "frame", "Down at latitude 45 has negative North and positive Down at equator");
        auto const old = converted;
        test.expect(Model::to_origin_frame(initial, 91, 0, 0, 0, converted) == Status::domain_error &&
                        converted.declination == old.declination && converted.inclination == old.inclination,
                    "frame", "invalid geographic coordinates preserve output");
    }
    for (bool geographic : {false, true})
    {
        Model model;
        Aiding aiding;
        typename Aiding::state_type state;
        auto P = Aiding::covariance_type::identity() * static_cast<Scalar>(0.01);
        typename Aiding::source_type source{1, 0, 1};
        typename Aiding::Context context{true, true, Aiding::lifecycle_type::FlightState::on_ground};
        test.expect(model.initialize(1000000, {geographic ? 3U : 2U, 0}, seed, initial) == Status::success,
                    "integration", "reference setup");
        aiding.initialize(source, 1000000);
        auto tick = [&](std::uint64_t now)
        {
            typename Aiding::Reference reference{model.observation_n(), model.health_n(), model.available()};
            source.reference_id = model.reference_id();
            return aiding.update(now, {now, source, seed}, context, reference, state, P);
        };
        for (unsigned i = 1; i <= 103; ++i)
        {
            test.expect(tick(1000000 + 10000 * i).status == Status::success, "integration", "healthy startup");
        }
        auto candidate = initial;
        candidate.declination = static_cast<Scalar>(0.05);
        test.expect(model.update(2040000, candidate, true, true).changed, "integration",
                    "adopt changed geographic model");
        auto result = tick(2040000);
        test.expect(result.status == Status::success && result.reset == geographic && result.fused != geographic &&
                        aiding.reset_counter() == (geographic ? 1 : 0),
                    "integration", "geographic revision resets resting heading; health-only revision does not");
        if (!geographic)
        {
            candidate.strength = 80;
            test.expect(model.update(2050000, candidate, true, true).changed, "integration",
                        "health-only strength adoption");
            auto const old_q = state.q_nb;
            result = tick(2050000);
            test.expect(result.rejected && !result.reset && !result.fused &&
                            aiding.observation().field.failures.strength &&
                            formal_eskf::linalg::max_abs(old_q.coefficients() - state.q_nb.coefficients()) == Scalar{0},
                        "integration", "independent WMM can reject a seed-matching field without resetting heading");
        }
    }
    {
        Model model;
        Aiding aiding;
        typename Aiding::state_type state;
        auto P = Aiding::covariance_type::identity() * static_cast<Scalar>(0.01);
        typename Aiding::source_type source{1, 0, 1};
        typename Aiding::Context context{false, true, Aiding::lifecycle_type::FlightState::on_ground};
        test.expect(model.initialize(1, {}, seed, std::nullopt) == Status::success, "fallback",
                    "missing WMM permits seed initialization");
        typename Aiding::Reference reference{model.observation_n(), model.health_n(), model.available()};
        aiding.initialize(source, 1);
        typename Aiding::Result result;
        for (unsigned i = 1; i <= 103; ++i)
        {
            auto const now = 1U + i * 10000U;
            result = aiding.update(now, {now, source, seed}, context, reference, state, P);
            test.expect(result.status == Status::success, "fallback",
                        "missing health reference is not invalid geometry");
        }
        test.expect(result.fused && !model.available() && aiding.reset_counter() == 0, "fallback",
                    "seed correction uses broad field-strength fallback without invented WMM");
        test.expect(model.initialize(1, {}, Vector::from_row_major({0, 0, 50}), initial) == Status::domain_error,
                    "seed", "WMM cannot make a vertical-only seed heading observable");
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
