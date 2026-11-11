// SPDX-License-Identifier: BSD-3-Clause
#include "../integrations/px4/module/GravityAiding.hpp"
#include "test_backend.hpp"
#include "test_support.hpp"

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
    for (unsigned scenario = 0; scenario < 8; ++scenario)
    {
        Aiding aiding;
        typename Aiding::state_type state;
        auto P = Aiding::covariance_type::identity() * static_cast<Scalar>(0.0001);
        auto conditions = context;
        auto sample = imu;
        int clipping = 0;
        if (scenario == 1)
        {
            conditions.enabled = false;
        }
        if (scenario == 2)
        {
            conditions.horizontal_aiding = true;
        }
        if (scenario == 3 || scenario == 4)
        {
            sample.specific_force_b = -gravity * Scalar{2};
        }
        if (scenario == 4)
        {
            conditions.at_rest = true;
        }
        if (scenario == 5)
        {
            clipping = 1;
        }
        if (scenario == 6)
        {
            clipping = -1;
        }
        if (scenario == 7)
        {
            P = Aiding::covariance_type::identity();
        }
        auto const before = P;
        typename Aiding::Result result;
        for (unsigned tick = 0; tick < 3 && !result.ready; ++tick)
        {
            result = aiding.update(sample, 5000, 5000, clipping, conditions, gravity, state, P);
        }
        bool const expected = scenario == 0 || scenario == 4;
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
        test.expect(formal_eskf::so3::try_exp<Linalg>(Vector::from_row_major({static_cast<Scalar>(0.02), 0, 0}),
                                                      static_cast<Scalar>(1e-6), state.q_nb) == Status::success,
                    "setup", "tilted prior");
        auto const prior = state.q_nb.coefficients()(1);
        auto first = aiding.update(imu, 5000, 5000, 0, context, gravity, state, P);
        auto second = aiding.update(imu, 5000, 5000, 0, context, gravity, state, P);
        test.expect(!first.ready && second.fusion.decision == Decision::fused && state.q_nb.coefficients()(1) < prior,
                    "correction", "production path actually corrects tilt toward measured gravity");
        state.b_a.set(0, Scalar{1});
        state.b_g.set(1, Scalar{-1});
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
        independent.angular_rate_b.set(0, Scalar{0.5});
        auto equivalent = independent;
        equivalent.angular_rate_b.set(0, static_cast<Scalar>(0.4));
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
} // namespace

int main()
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Context test;
    run<formal_eskf::test::Backend<float>>(test);
    run<formal_eskf::test::Backend<double>>(test);
    return test.failures() == 0 ? 0 : 1;
}
