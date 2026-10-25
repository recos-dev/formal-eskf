/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <array>
#include <bit>
#include <cmath>
#include <limits>

#include <formal_eskf/runtime/gnss_measurement.hpp>
#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{
using formal_eskf::Status;
using formal_eskf::runtime::FusionDecision;
using Ins = formal_eskf::configuration::Ins;
using Context = formal_eskf::test::Context;
constexpr std::string_view profile = "GNSS bounded vertical fusion";

template <typename Matrix> bool same_bits(Matrix const & a, Matrix const & b)
{
    using bytes_type = std::array<std::byte, sizeof(typename Matrix::value_type)>;
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            if (std::bit_cast<bytes_type>(a(row, column)) != std::bit_cast<bytes_type>(b(row, column)))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename State> bool same_state(State const & a, State const & b)
{
    return same_bits(a.p_n, b.p_n) && same_bits(a.v_n, b.v_n) && same_bits(a.b_a, b.b_a) && same_bits(a.b_g, b.b_g) &&
           same_bits(a.q_nb.coefficients(), b.q_nb.coefficients());
}

template <typename Linalg> struct GnssMeasurementFixture
{
    using value_type = typename Linalg::value_type;
    using state_type = Ins::NominalState<Linalg>;
    using covariance_type = typename Linalg::template matrix_type<15U, 15U>;
    using noise_type = typename Linalg::template matrix_type<3U, 3U>;
    using vector_type = typename Linalg::template vector_type<3U>;
    state_type state{};
    covariance_type P = covariance_type::identity() * value_type{3};
    noise_type V = noise_type::identity();
    vector_type velocity = vector_type::from_row_major({value_type{0.5}, value_type{-0.5}, value_type{-10}});
    value_type gate = value_type{1}; // S=4, limit=2, exactly representable boundaries.
    value_type minimum_norm = static_cast<value_type>(1e-6);
    formal_eskf::runtime::GnssVelocityConditions<value_type> conditions{true, value_type{0.25}, value_type{0.5}};

    auto fuse(state_type & output, covariance_type & output_P, Ins::BiasUpdate const & permissions = {}) const
    {
        return formal_eskf::runtime::fuse_gnss_velocity(state, P, velocity, V, minimum_norm, gate, conditions, output,
                                                        output_P, permissions);
    }
};

template <typename Linalg> void test_policy(Context & test)
{
    using F = GnssMeasurementFixture<Linalg>;
    using T = typename F::value_type;
    // Strict horizontal boundaries include equality, while the ordinary gate accepts equality.
    for (bool fault : {false, true})
    {
        for (T accuracy : {T{0}, T{0.25}, T{0.5}, T{1}})
        {
            for (T horizontal : {T{0}, T{2}, T{3}})
            {
                for (std::size_t axis : {0U, 1U})
                {
                    for (T down : {T{-10}, T{-2}, T{0}, T{2}, T{10}})
                    {
                        F fixture;
                        fixture.conditions = {fault, accuracy, T{0.5}};
                        fixture.velocity.set(axis, horizontal);
                        fixture.velocity.set(2U, down);
                        auto output = fixture.state;
                        auto output_P = fixture.P;
                        auto const result = fixture.fuse(output, output_P);
                        bool const rejected = horizontal > T{2} || std::abs(down) > T{2};
                        bool const limited = fault && accuracy < T{0.5} && horizontal < T{2} && std::abs(down) > T{2};
                        test.expect(result.fusion.status == Status::success && result.fusion.diagnostics_valid &&
                                        result.innovation_limited == limited &&
                                        result.fusion.decision ==
                                            (rejected && !limited ? FusionDecision::rejected : FusionDecision::fused),
                                    profile,
                                    "fault, accuracy, sign and both horizontal boundaries select the exact policy");
                        T const expected = limited ? (down < T{0} ? T{-2} : T{2}) : down;
                        test.expect(result.fusion_innovation(2U) == expected && result.fusion.innovation(2U) == down &&
                                        result.fusion.test_ratio(2U) == down * down / T{4} &&
                                        result.fusion_innovation(0U) == fixture.velocity(0U) &&
                                        result.fusion_innovation(1U) == fixture.velocity(1U),
                                    profile, "retain raw evidence and leave horizontal residuals unchanged");
                        if (rejected && !limited)
                        {
                            test.expect(same_state(output, fixture.state) && same_bits(output_P, fixture.P), profile,
                                        "rejected group preserves the entire estimate");
                        }
                        else
                        {
                            test.expect(output.v_n(2U) == T{0.75} * expected, profile,
                                        "actual correction uses bounded residual with original gain");
                        }
                    }
                }
            }
        }
    }
    for (bool fault : {false, true})
    {
        for (T down : {T{-10}, T{-2}, T{0}, T{2}, T{10}})
        {
            F fixture;
            auto output = fixture.state;
            auto output_P = fixture.P;
            auto const result = formal_eskf::runtime::fuse_gnss_height(
                fixture.state, fixture.P, down, T{1}, fixture.minimum_norm, T{1}, fault, output, output_P);
            bool const rejected = std::abs(down) > T{2};
            bool const limited = fault && rejected;
            T const expected = limited ? (down < T{0} ? T{-2} : T{2}) : down;
            test.expect(result.fusion.status == Status::success && result.innovation_limited == limited &&
                            result.fusion.decision ==
                                (rejected && !fault ? FusionDecision::rejected : FusionDecision::fused) &&
                            result.fusion_innovation(0U) == expected && result.fusion.innovation(0U) == down,
                        profile, "height exception requires fault and rejection, with symmetric sign handling");
            test.expect(output.p_n(2U) == (rejected && !fault ? T{0} : T{0.75} * expected), profile,
                        "height correction actually applies bounded innovation");
        }
    }
}

template <typename Linalg> void test_correction_contract(Context & test)
{
    using F = GnssMeasurementFixture<Linalg>;
    using T = typename F::value_type;
    for (bool height : {false, true})
    {
        for (bool fault : {false, true})
        {
            for (unsigned aliases = 0U; aliases < 4U; ++aliases)
            {
                F fixture;
                // PSD prior and correlated SPD measurement noise; couple biases to the observation.
                fixture.P.set(9U, height ? 2U : 5U, T{0.25});
                fixture.P.set(height ? 2U : 5U, 9U, T{0.25});
                fixture.P.set(12U, height ? 2U : 5U, T{0.125});
                fixture.P.set(height ? 2U : 5U, 12U, T{0.125});
                fixture.V.set(0U, 1U, T{0.125});
                fixture.V.set(1U, 0U, T{0.125});
                fixture.conditions.bad_vertical_acceleration = fault;
                if (!fault)
                {
                    fixture.velocity.set(2U, T{1});
                }
                Ins::BiasUpdate permissions;
                permissions.accelerometer = {false, false, false};
                auto expected = fixture.state;
                auto expected_P = fixture.P;
                auto expected_r = fixture.velocity;
                if (fault)
                {
                    expected_r.set(2U, T{-2});
                }
                Status status{};
                if (height)
                {
                    status = formal_eskf::try_correct(
                        fixture.state, fixture.P, Linalg::template vector_type<1U>::from_row_major({expected_r(2U)}),
                        formal_eskf::vertical_position_jacobian<Linalg>(),
                        Linalg::template matrix_type<1U, 1U>::identity(), fixture.minimum_norm, expected, expected_P,
                        permissions);
                }
                else
                {
                    status = formal_eskf::try_correct(fixture.state, fixture.P, expected_r,
                                                      formal_eskf::velocity_jacobian<Linalg>(), fixture.V,
                                                      fixture.minimum_norm, expected, expected_P, permissions);
                }
                auto output = fixture.state;
                auto output_P = fixture.P;
                auto const prior = fixture;
                auto & state_output = (aliases & 1U) ? fixture.state : output;
                auto & covariance_output = (aliases & 2U) ? fixture.P : output_P;
                auto const fused =
                    height ? formal_eskf::runtime::fuse_gnss_height(fixture.state, fixture.P, fixture.velocity(2U),
                                                                    T{1}, fixture.minimum_norm, fixture.gate, fault,
                                                                    state_output, covariance_output, permissions)
                                 .fusion.decision
                           : fixture.fuse(state_output, covariance_output, permissions).fusion.decision;
                test.expect(status == Status::success && fused == FusionDecision::fused &&
                                same_state(state_output, expected) && same_bits(covariance_output, expected_P) &&
                                same_bits(state_output.b_a, prior.state.b_a),
                            profile, "all aliases match checked correction including correlations and bias inhibition");
                test.expect(((aliases & 1U) || same_state(fixture.state, prior.state)) &&
                                ((aliases & 2U) || same_bits(fixture.P, prior.P)),
                            profile, "disjoint priors are preserved");
            }
        }
    }
}

template <typename Linalg> void test_rounded_boundary(Context & test)
{
    using F = GnssMeasurementFixture<Linalg>;
    using T = typename F::value_type;
    bool exercised = false;
    for (unsigned variance = 4U; variance < 64U && !exercised; ++variance)
    {
        F fixture;
        T const S = static_cast<T>(variance);
        T const limit = std::sqrt(S);
        // Seek an IEEE-754 sqrt whose square rounds just outside the gate.
        if (limit * limit / S <= T{1})
        {
            continue;
        }
        fixture.V = F::noise_type::identity() * (S - T{3});
        fixture.velocity.set(2U, T{-100});
        auto output = fixture.state;
        auto output_P = fixture.P;
        auto const result = fixture.fuse(output, output_P);
        test.expect(result.fusion.decision == FusionDecision::fused && result.innovation_limited &&
                        result.fusion_innovation(2U) == -limit,
                    profile, "rounded boundary is not incorrectly rejected by a second gate");
        exercised = true;
    }
    test.expect(exercised, profile, "floating-point boundary fixture is exercised");
}

template <typename Linalg> void test_failures(Context & test)
{
    using F = GnssMeasurementFixture<Linalg>;
    using T = typename F::value_type;
    auto const inf = std::numeric_limits<T>::infinity();
    auto const nan = std::numeric_limits<T>::quiet_NaN();
    for (bool height : {false, true})
    {
        for (unsigned failure = 0U; failure < 12U; ++failure)
        {
            if (height && failure < 4U)
            {
                continue;
            }
            for (unsigned aliases = 0U; aliases < 4U; ++aliases)
            {
                F fixture;
                switch (failure)
                {
                case 0U:
                    fixture.conditions.speed_accuracy = T{-1};
                    break;
                case 1U:
                    fixture.conditions.speed_accuracy = nan;
                    break;
                case 2U:
                    fixture.conditions.maximum_speed_accuracy = T{0};
                    break;
                case 3U:
                    fixture.conditions.maximum_speed_accuracy = inf;
                    break;
                case 4U:
                    fixture.velocity.set(2U, nan);
                    break;
                case 5U:
                    fixture.gate = T{0};
                    break;
                case 6U:
                    fixture.gate = inf;
                    break;
                case 7U:
                    fixture.P.set(0U, 0U, nan);
                    break;
                case 8U:
                    fixture.V.set(0U, 0U, T{-1});
                    break;
                case 9U:
                    fixture.minimum_norm = T{2};
                    break; // Gate valid; checked correction rejects quaternion norm.
                case 10U:
                    // Positive diagonal but indefinite S: the exception must not bypass LLT.
                    fixture.P.set(height ? 2U : 3U, height ? 2U : 3U, T{0});
                    if (!height)
                    {
                        fixture.V.set(0U, 1U, T{8});
                        fixture.V.set(1U, 0U, T{8});
                    }
                    else
                    {
                        fixture.minimum_norm = T{2};
                    }
                    break;
                case 11U:
                    fixture.velocity.set(2U, std::numeric_limits<T>::max());
                    break; // Finite innovation, overflowing squared gate numerator.
                }
                auto output = fixture.state;
                auto output_P = fixture.P * T{2};
                output.p_n.set(0U, T{7});
                auto & state_output = (aliases & 1U) ? fixture.state : output;
                auto & covariance_output = (aliases & 2U) ? fixture.P : output_P;
                auto const before_state = state_output;
                auto const before_P = covariance_output;
                auto check_failure = [&](auto const & result)
                {
                    test.expect(
                        result.decision == FusionDecision::failed && result.status != Status::success &&
                            same_state(state_output, before_state) && same_bits(covariance_output, before_P),
                        profile,
                        "invalid metadata, gate arithmetic and downstream correction failures preserve outputs");
                };
                if (height)
                {
                    check_failure(formal_eskf::runtime::fuse_gnss_height(
                                      fixture.state, fixture.P, fixture.velocity(2U), fixture.V(0U, 0U),
                                      fixture.minimum_norm, fixture.gate, true, state_output, covariance_output)
                                      .fusion);
                }
                else
                {
                    check_failure(fixture.fuse(state_output, covariance_output).fusion);
                }
            }
        }
    }
    F fixture;
    auto const prior = fixture;
    auto const evidence = formal_eskf::runtime::check_gnss_velocity(
        fixture.state, fixture.P, fixture.velocity, fixture.V, fixture.minimum_norm, fixture.gate, fixture.conditions);
    test.expect(evidence.diagnostics_valid && evidence.innovation_limited && evidence.status == Status::success &&
                    !evidence.rejected && same_state(fixture.state, prior.state) && same_bits(fixture.P, prior.P),
                profile, "quality-rejected sample can update diagnostics without attempting correction");
}

} /* end namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    Context test;
    using Float = formal_eskf::test::Backend<float>;
    using Double = formal_eskf::test::Backend<double>;
    test_policy<Float>(test);
    test_policy<Double>(test);
    test_correction_contract<Float>(test);
    test_correction_contract<Double>(test);
    test_rounded_boundary<Float>(test);
    test_rounded_boundary<Double>(test);
    test_failures<Float>(test);
    test_failures<Double>(test);
    return test.failures() == 0 ? 0 : 1;
}
