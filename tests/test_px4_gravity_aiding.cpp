// SPDX-License-Identifier: BSD-3-Clause
#include "../integrations/px4/module/GravityAiding.hpp"
#include "test_backend.hpp"
#include "test_support.hpp"

#include <array>
#include <limits>

namespace
{
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

template <typename Linalg> void run(formal_eskf::test::Context & test)
{
    using Aiding = formal_eskf::px4::GravityAiding<Linalg>;
    using Scalar = typename Aiding::value_type;
    using Vector = typename Aiding::vector_type;
    using Decision = formal_eskf::runtime::FusionDecision;
    using formal_eskf::Status;
    auto const gravity = Vector::from_row_major({0, 0, static_cast<Scalar>(9.80665)});
    formal_eskf::ImuSample<Linalg> imu{-gravity, {}};
    typename Aiding::Context context{true, false, false};
    enum class Scenario
    {
        eligible,
        disabled,
        aided,
        high_force,
        rest_override,
        clipped,
        unknown_clipping,
        unaligned
    };
    constexpr std::array scenarios{Scenario::eligible,         Scenario::disabled,      Scenario::aided,
                                   Scenario::high_force,       Scenario::rest_override, Scenario::clipped,
                                   Scenario::unknown_clipping, Scenario::unaligned};
    for (auto const scenario : scenarios)
    {
        Aiding aiding;
        typename Aiding::state_type state;
        auto P = Aiding::covariance_type::identity() * static_cast<Scalar>(0.0001);
        auto conditions = context;
        auto sample = imu;
        int clipping = 0;
        if (scenario == Scenario::disabled)
        {
            conditions.enabled = false;
        }
        if (scenario == Scenario::aided)
        {
            conditions.horizontal_aiding = true;
        }
        if (scenario == Scenario::high_force || scenario == Scenario::rest_override)
        {
            sample.specific_force_b = -gravity * Scalar{2};
        }
        if (scenario == Scenario::rest_override)
        {
            conditions.at_rest = true;
        }
        if (scenario == Scenario::clipped)
        {
            clipping = 1;
        }
        if (scenario == Scenario::unknown_clipping)
        {
            clipping = -1;
        }
        if (scenario == Scenario::unaligned)
        {
            P = Aiding::covariance_type::identity();
        }
        auto const before = P;
        typename Aiding::Result result;
        for (unsigned tick = 0; tick < 3 && !result.ready; ++tick)
        {
            result = aiding.update(sample, 5000, 5000, clipping, conditions, gravity, state, P);
        }
        bool const expected = scenario == Scenario::eligible || scenario == Scenario::rest_override;
        test.expect(result.status == Status::success && result.ready && result.interval_us == 10000 &&
                        (result.fusion.decision == Decision::fused) == expected,
                    "eligibility", "enabled, aided, motion, rest override, clipping, unknown metadata and tilt");
        if (!expected)
        {
            test.expect(equal(P, before) &&
                            equal(state.q_nb.coefficients(), typename Aiding::state_type{}.q_nb.coefficients()),
                        "rejection", "unavailable gravity correction preserves state and covariance");
        }
    }
    {
        Aiding aiding;
        typename Aiding::state_type state;
        auto P = Aiding::covariance_type::identity() * static_cast<Scalar>(0.0001);
        test.expect(formal_eskf::so3::exp<Linalg>(Vector::from_row_major({static_cast<Scalar>(0.02), 0, 0}),
                                                  static_cast<Scalar>(1e-6), state.q_nb) == Status::success,
                    "setup", "tilted prior");
        auto const prior = state.q_nb.coefficients()(1);
        auto first = aiding.update(imu, 5000, 5000, 0, context, gravity, state, P);
        auto second = aiding.update(imu, 5000, 5000, 0, context, gravity, state, P);
        test.expect(!first.ready && second.fusion.decision == Decision::fused && state.q_nb.coefficients()(1) < prior,
                    "correction", "production path actually corrects tilt toward measured gravity");
        state.b_a(0) = Scalar{1};
        state.b_g(1) = Scalar{-1};
        formal_eskf::configuration::Ins::BiasUpdate inhibited{{false, false, false}, {false, false, false}};
        first = aiding.update(imu, 5000, 5000, 0, context, gravity, state, P, inhibited);
        second = aiding.update(imu, 5000, 5000, 0, context, gravity, state, P, inhibited);
        test.expect(first.status == Status::success && !first.ready && second.fusion.decision == Decision::fused &&
                        state.b_a(0) == static_cast<Scalar>(0.4) && state.b_g(1) == static_cast<Scalar>(-0.15),
                    "bias limits", "successful gravity group forwards inhibition and applies fixed limits");
    }
    {
        Aiding a, b;
        typename Aiding::state_type sa, sb;
        auto Pa = Aiding::covariance_type::identity() * static_cast<Scalar>(0.0001);
        auto Pb = Pa;
        auto independent = imu;
        independent.angular_rate_b(0) = Scalar{0.5};
        auto equivalent = independent;
        equivalent.angular_rate_b(0) = static_cast<Scalar>(0.4);
        for (unsigned tick = 0; tick < 2; ++tick)
        {
            auto const ra = a.update(independent, 4000, 5000, 0, context, gravity, sa, Pa);
            auto const rb = b.update(equivalent, 5000, 5000, 0, context, gravity, sb, Pb);
            test.expect(ra.status == Status::success && rb.status == Status::success && ra.ready == rb.ready, "periods",
                        "independent gyro and accel periods produce identical measured increments");
        }
        test.expect(equal(sa.q_nb.coefficients(), sb.q_nb.coefficients()) && equal(Pa, Pb), "periods",
                    "adapter preserves angle integral when gyro and accel durations differ");
        a.reset_samples();
        auto const before = Pa;
        auto result = a.update(imu, 0, 5000, 0, context, gravity, sa, Pa);
        test.expect(result.status == Status::domain_error && equal(Pa, before), "failure", "invalid period rolls back");
        result = a.update(imu, 5000, 5000, 0, context, gravity, sa, Pa);
        test.expect(!result.ready && !a.active() && a.tilt_aligned(), "gap", "history clears, alignment survives");
        a.reset_samples();
        result = a.update(imu, 5000, 5000, 0, context, gravity, sa, Pa);
        test.expect(!result.ready, "gap", "partial interval cannot cross a discontinuity");
    }
}
template <typename Linalg> void test_history_boundaries(formal_eskf::test::Context & test)
{
    using Aiding = formal_eskf::px4::GravityAiding<Linalg>;
    using Scalar = typename Aiding::value_type;
    using Vector = typename Aiding::vector_type;
    using Decision = formal_eskf::runtime::FusionDecision;
    using formal_eskf::Status;
    auto const gravity = Vector::from_row_major({0, 0, static_cast<Scalar>(9.80665)});
    formal_eskf::ImuSample<Linalg> const imu{-gravity, {}};
    typename Aiding::Context context{true, false, false};
    typename Aiding::state_type state;
    auto P = Aiding::covariance_type::identity() * static_cast<Scalar>(0.0001);
    Aiding aiding;

    auto invalid = imu;
    invalid.specific_force_b(0U) = std::numeric_limits<Scalar>::quiet_NaN();
    auto result = aiding.update(invalid, 5000U, 5000U, 0, context, gravity, state, P);
    test.expect(result.status == Status::non_finite_input && !result.ready && aiding.tilt_aligned(), "history",
                "tilt consumes the prior before a rejected IMU, without consuming an increment");
    result = aiding.update(imu, 5000U, 5000U, 0, context, gravity, state, P);
    test.expect(result.status == Status::success && !result.ready, "history", "failed IMU did not fill the interval");
    auto const before = P;
    auto const before_q = state.q_nb.coefficients();
    result = aiding.update(imu, 5000U, 5000U, 0, context, Vector{}, state, P);
    test.expect(result.ready && result.status == Status::domain_error && result.fusion.conditions.sample_eligible &&
                    result.fusion.decision == Decision::failed && aiding.active() && equal(P, before) &&
                    equal(state.q_nb.coefficients(), before_q),
                "history", "failed correction preserves the estimate, not the already consumed observation");
    result = aiding.update(imu, 5000U, 5000U, 0, context, gravity, state, P);
    test.expect(!result.ready && aiding.active(), "history", "failed correction leaves a fresh partial interval");
    result = aiding.update(imu, 5000U, 5000U, 0, context, gravity, state, P);
    test.expect(result.fusion.decision == Decision::fused && aiding.active(), "history", "next observation can fuse");

    context.enabled = false;
    result = aiding.update(imu, 5000U, 5000U, 0, context, gravity, state, P);
    test.expect(!result.ready && !aiding.active(), "mode", "disable clears activity even on a partial interval");
    context.enabled = true;
    result = aiding.update(imu, 5000U, 5000U, 1, context, gravity, state, P);
    test.expect(result.ready && result.fusion.decision == Decision::rejected && aiding.active(), "mode",
                "known clipping rejects the sample while preserving an otherwise active mode");
    (void)aiding.update(imu, 5000U, 5000U, -1, context, gravity, state, P);
    result = aiding.update(imu, 5000U, 5000U, 0, context, gravity, state, P);
    test.expect(result.ready && result.fusion.decision == Decision::rejected && !aiding.active(), "mode",
                "unknown clipping in any constituent sample clears activity at emission");

    aiding.reset_samples();
    context.enabled = false;
    auto high_force = imu;
    high_force.specific_force_b = -gravity * Scalar{2};
    (void)aiding.update(high_force, 5000U, 5000U, 0, context, gravity, state, P);
    result = aiding.update(high_force, 5000U, 5000U, 0, context, gravity, state, P);
    test.expect(result.ready && result.status == Status::success && !aiding.active(), "filter",
                "disabled fusion still emits observations and seeds the magnitude filter");
    context.enabled = true;
    (void)aiding.update(imu, 5000U, 5000U, 0, context, gravity, state, P);
    result = aiding.update(imu, 5000U, 5000U, 0, context, gravity, state, P);
    test.expect(result.ready && result.fusion.decision == Decision::rejected && !aiding.active(), "filter",
                "re-enabling fusion retains high-force LPF history instead of reseeding from the new sample");
    aiding.reset_samples();
    (void)aiding.update(imu, 5000U, 5000U, 0, context, gravity, state, P);
    result = aiding.update(imu, 5000U, 5000U, 0, context, gravity, state, P);
    test.expect(result.fusion.decision == Decision::fused && aiding.tilt_aligned(), "filter",
                "sample reset clears LPF history but preserves established tilt alignment");
}
} // namespace

int main()
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Context test;
    run<formal_eskf::test::Backend<float>>(test);
    run<formal_eskf::test::Backend<double>>(test);
    test_history_boundaries<formal_eskf::test::Backend<float>>(test);
    test_history_boundaries<formal_eskf::test::Backend<double>>(test);
    return test.failures() == 0 ? 0 : 1;
}
