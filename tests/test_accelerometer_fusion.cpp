/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/accelerometer.hpp>

#include <array>
#include <bit>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string_view>
#include <type_traits>

#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{

using formal_eskf::Status;
using formal_eskf::configuration::Ahrs;
using formal_eskf::configuration::Ins;
using formal_eskf::runtime::FusionDecision;
using TestContext = formal_eskf::test::Context;

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
    bool equal = same_bits(a.q_nb.coefficients(), b.q_nb.coefficients());
    if constexpr (requires { a.p_n; })
    {
        equal = equal && same_bits(a.p_n, b.p_n) && same_bits(a.v_n, b.v_n) && same_bits(a.b_a, b.b_a) &&
                same_bits(a.b_g, b.b_g);
    }
    return equal;
}

template <typename Linalg, typename Configuration> struct FusionFixture
{
    using value_type = typename Linalg::value_type;
    using state_type = typename Configuration::template NominalState<Linalg>;
    using covariance_type =
        formal_eskf::linalg::Matrix<Linalg, Configuration::error_state_dimension, Configuration::error_state_dimension>;
    using vector_type = formal_eskf::linalg::Matrix<Linalg, 3U, 1U>;
    using noise_type = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    using magnitude_type = formal_eskf::runtime::AccelerometerMagnitude<Linalg>;

    state_type state{};
    covariance_type P = covariance_type::identity();
    vector_type gravity = vector_type::from_row_major({value_type{0}, value_type{0}, static_cast<value_type>(9.80665)});
    vector_type sample = -gravity;
    noise_type V = noise_type::identity();
    formal_eskf::runtime::AccelerometerFusionConditions conditions{true, true, false, false, {}};
    magnitude_type monitor{};
    typename magnitude_type::Result magnitude{};
    value_type minimum_norm = static_cast<value_type>(1e-6);
    Ins::BiasUpdate bias_update{};

    FusionFixture()
    {
        if constexpr (std::is_same_v<Configuration, Ins>)
        {
            state.p_n = vector_type::from_row_major({value_type{1}, value_type{2}, value_type{3}});
            state.v_n = vector_type::from_row_major({value_type{4}, value_type{5}, value_type{6}});
            state.b_a = vector_type::from_row_major({value_type{0.125}, value_type{-0.25}, value_type{0.125}});
            state.b_g = vector_type::from_row_major({value_type{0.125}, value_type{0.25}, value_type{-0.125}});
            sample = sample + state.b_a;
        }
        magnitude = monitor.update(sample);
        sample = sample + vector_type::from_row_major({value_type{0.25}, value_type{-0.125}, value_type{0}});
        magnitude = monitor.update(sample);
        // Positive-definite matrices with nonzero state cross-covariance and correlated V.
        for (std::size_t row = 0U; row < P.row_count; ++row)
        {
            for (std::size_t column = 0U; column < P.column_count; ++column)
            {
                P(row, column) += static_cast<value_type>((row + 1U) * (column + 1U)) / value_type{256};
            }
        }
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            for (std::size_t column = 0U; column < 3U; ++column)
            {
                V(row, column) += static_cast<value_type>((row + 1U) * (column + 1U)) / value_type{16};
            }
        }
    }

    auto fuse(state_type & output, covariance_type & output_P) const
    {
        if constexpr (std::is_same_v<Configuration, Ins>)
        {
            return formal_eskf::runtime::fuse_accelerometer(state, P, sample, gravity, V, conditions, magnitude,
                                                            minimum_norm, output, output_P, bias_update);
        }
        else
        {
            return formal_eskf::runtime::fuse_accelerometer(state, P, sample, gravity, V, conditions, magnitude,
                                                            minimum_norm, output, output_P);
        }
    }

    Status correct(state_type & output, covariance_type & output_P) const
    {
        if constexpr (std::is_same_v<Configuration, Ins>)
        {
            return formal_eskf::try_correct_accelerometer(state, P, sample, gravity, V, minimum_norm, output, output_P,
                                                          bias_update);
        }
        else
        {
            return formal_eskf::try_correct_accelerometer(state, P, sample, gravity, V, minimum_norm, output, output_P);
        }
    }
};

template <typename Linalg, typename Configuration> void test_correction(TestContext & test, std::string_view profile)
{
    using fixture_type = FusionFixture<Linalg, Configuration>;
    fixture_type fixture;
    test.expect(fixture.magnitude.within_gravity_band &&
                    !same_bits(fixture.sample, fixture.monitor.filtered_specific_force_b()),
                profile, "real LPF result permits a distinct current-sample correction");
    unsigned const masks = std::is_same_v<Configuration, Ins> ? 64U : 1U;
    for (unsigned mask = 0U; mask < masks; ++mask)
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            fixture.bias_update.accelerometer[axis] = (mask & (1U << axis)) != 0U;
            fixture.bias_update.gyroscope[axis] = (mask & (1U << (axis + 3U))) != 0U;
        }
        typename fixture_type::state_type expected;
        typename fixture_type::covariance_type expected_P;
        test.expect(fixture.correct(expected, expected_P) == Status::success, profile, "direct core oracle succeeds");
        test.expect(!same_bits(expected.q_nb.coefficients(), fixture.state.q_nb.coefficients()) &&
                        !same_bits(expected_P, fixture.P),
                    profile, "nonzero residual exercises actual attitude and covariance correction");
        for (unsigned alias = 0U; alias < 4U; ++alias)
        {
            auto current = fixture;
            typename fixture_type::state_type separate;
            auto separate_P = fixture_type::covariance_type::identity();
            auto const old_separate = separate;
            auto const old_separate_P = separate_P;
            bool const state_alias = (alias & 1U) != 0U;
            bool const covariance_alias = (alias & 2U) != 0U;
            auto & output = state_alias ? current.state : separate;
            auto & output_P = covariance_alias ? current.P : separate_P;
            auto const result = current.fuse(output, output_P);
            test.expect(result.decision == FusionDecision::fused && result.status == Status::success &&
                            result.conditions.active && result.conditions.sample_eligible &&
                            same_state(output, expected) && same_bits(output_P, expected_P),
                        profile, "all aliases and bias masks match direct correction using current raw measurement");
            test.expect(
                same_state(state_alias ? separate : current.state, state_alias ? old_separate : fixture.state) &&
                    same_bits(covariance_alias ? separate_P : current.P, covariance_alias ? old_separate_P : fixture.P),
                profile, "separate priors and unused output objects are unchanged");
        }
        if constexpr (std::is_same_v<Configuration, Ins>)
        {
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                test.expect((expected.b_a(axis) != fixture.state.b_a(axis)) ==
                                    fixture.bias_update.accelerometer[axis] &&
                                (expected.b_g(axis) != fixture.state.b_g(axis)) == fixture.bias_update.gyroscope[axis],
                            profile, "cross-covariance exercises each enabled bias row; disabled rows stay fixed");
            }
        }
    }
    typename fixture_type::state_type output;
    typename fixture_type::covariance_type output_P;
    auto const result = formal_eskf::runtime::fuse_accelerometer(
        fixture.state, fixture.P, fixture.sample, fixture.gravity, fixture.V, fixture.conditions, fixture.magnitude,
        fixture.minimum_norm, output, output_P);
    typename fixture_type::state_type expected;
    typename fixture_type::covariance_type expected_P;
    fixture.bias_update = {};
    test.expect(fixture.correct(expected, expected_P) == Status::success && result.decision == FusionDecision::fused &&
                    same_state(output, expected) && same_bits(output_P, expected_P),
                profile, "omitted permissions retain the default core behavior");
}

template <typename Fixture>
void expect_preserved(TestContext & test, std::string_view profile, Fixture const & fixture, FusionDecision decision,
                      Status status)
{
    using value_type = typename Fixture::value_type;
    for (unsigned alias = 0U; alias < 4U; ++alias)
    {
        auto current = fixture;
        typename Fixture::state_type separate;
        auto separate_P = Fixture::covariance_type::identity();
        separate_P(0U, 0U) = std::numeric_limits<value_type>::quiet_NaN();
        separate_P(0U, 1U) = -value_type{0};
        auto const old_separate = separate;
        auto const old_separate_P = separate_P;
        auto const result =
            current.fuse((alias & 1U) != 0U ? current.state : separate, (alias & 2U) != 0U ? current.P : separate_P);
        test.expect(result.decision == decision && result.status == status, profile,
                    "rejection and numerical failure remain distinct and propagate status");
        test.expect(same_state(current.state, fixture.state) && same_bits(current.P, fixture.P) &&
                        same_state(separate, old_separate) && same_bits(separate_P, old_separate_P),
                    profile, "all alias arrangements preserve outputs, including NaN and signed zero sentinels");
    }
}

template <typename Linalg, typename Configuration> void test_conditions(TestContext & test, std::string_view profile)
{
    using fixture_type = FusionFixture<Linalg, Configuration>;
    using value_type = typename fixture_type::value_type;
    fixture_type const baseline;
    for (unsigned reason = 0U; reason < 7U; ++reason)
    {
        auto fixture = baseline;
        switch (reason)
        {
        case 0U:
            fixture.conditions.enabled = false;
            break;
        case 1U:
            fixture.conditions.tilt_aligned = false;
            break;
        case 2U:
            fixture.conditions.horizontal_aiding = true;
            break;
        case 3U:
            fixture.sample = -fixture.gravity * value_type{2};
            fixture.monitor.reset();
            fixture.magnitude = fixture.monitor.update(fixture.sample);
            break;
        default:
            fixture.conditions.clipped[reason - 4U] = true;
            break;
        }
        expect_preserved(test, profile, fixture, FusionDecision::rejected, Status::success);
    }
    // No innovation gate is introduced: a large residual can pass the magnitude conditions.
    auto fixture = baseline;
    fixture.sample = fixture_type::vector_type::from_row_major({fixture.gravity(2U), value_type{0}, value_type{0}});
    fixture.monitor.reset();
    fixture.magnitude = fixture.monitor.update(fixture.sample);
    typename fixture_type::state_type output;
    typename fixture_type::covariance_type output_P;
    auto result = fixture.fuse(output, output_P);
    test.expect(result.decision == FusionDecision::fused, profile, "no implicit residual gate or vector normalization");

    fixture = baseline;
    fixture.sample = -fixture.gravity * value_type{2};
    fixture.monitor.reset();
    fixture.magnitude = fixture.monitor.update(fixture.sample);
    fixture.conditions.at_rest = true;
    result = fixture.fuse(output, output_P);
    test.expect(!fixture.magnitude.within_gravity_band && result.decision == FusionDecision::fused, profile,
                "caller rest bypasses only the magnitude band");
    fixture.conditions.clipped[0U] = true;
    result = fixture.fuse(output, output_P);
    test.expect(result.conditions.active && !result.conditions.sample_eligible &&
                    result.decision == FusionDecision::rejected,
                profile, "clipping rejects the sample without clearing active mode");
    fixture.conditions.clipped[0U] = false;
    test.expect(fixture.fuse(output, output_P).decision == FusionDecision::fused, profile,
                "a rejected call creates no hidden latch or recovery timer");
}

template <typename Linalg, typename Configuration> void test_failures(TestContext & test, std::string_view profile)
{
    using fixture_type = FusionFixture<Linalg, Configuration>;
    using value_type = typename fixture_type::value_type;
    using limits_type = std::numeric_limits<value_type>;
    fixture_type const baseline;
    auto fixture = baseline;
    fixture.conditions.at_rest = true;
    fixture.conditions.enabled = false;
    fixture.sample(0U) = limits_type::quiet_NaN();
    fixture.magnitude = fixture.monitor.update(fixture.sample);
    expect_preserved(test, profile, fixture, FusionDecision::failed, Status::non_finite_input);
    fixture = baseline;
    fixture.sample(0U) = limits_type::max();
    fixture.magnitude = fixture.monitor.update(fixture.sample);
    expect_preserved(test, profile, fixture, FusionDecision::failed, Status::non_finite_result);

    fixture = baseline;
    fixture.V(0U, 0U) = limits_type::quiet_NaN();
    expect_preserved(test, profile, fixture, FusionDecision::failed, Status::non_finite_input);
    fixture.conditions.enabled = false;
    expect_preserved(test, profile, fixture, FusionDecision::rejected, Status::success);
    fixture = baseline;
    fixture.gravity = {};
    expect_preserved(test, profile, fixture, FusionDecision::failed, Status::domain_error);
    fixture = baseline;
    fixture.minimum_norm = value_type{0};
    expect_preserved(test, profile, fixture, FusionDecision::failed, Status::domain_error);
    fixture = baseline;
    fixture.V(0U, 1U) = value_type{1024};
    fixture.V(1U, 0U) = value_type{1024};
    expect_preserved(test, profile, fixture, FusionDecision::failed, Status::not_positive_definite);
    fixture = baseline;
    fixture.P(0U, 0U) = limits_type::infinity();
    expect_preserved(test, profile, fixture, FusionDecision::failed, Status::non_finite_input);
    fixture = baseline;
    // Test-only invalid quaternion representation reaches the core's late injection failure.
    using quaternion_type = typename fixture_type::state_type::quaternion_type;
    fixture.state.q_nb = std::bit_cast<quaternion_type>(std::array<value_type, 4U>{});
    typename fixture_type::state_type output;
    typename fixture_type::covariance_type output_P;
    auto const result = fixture.fuse(output, output_P);
    test.expect(result.conditions.sample_eligible && result.decision == FusionDecision::failed &&
                    result.status == Status::invalid_quaternion_norm,
                profile, "eligibility remains diagnostic, not success, after late correction failure");
    expect_preserved(test, profile, fixture, FusionDecision::failed, Status::invalid_quaternion_norm);
}

template <typename Scalar> void run_conformance(TestContext & test, std::string_view profile)
{
    using Linalg = formal_eskf::test::Backend<Scalar>;
    test_correction<Linalg, Ahrs>(test, profile);
    test_correction<Linalg, Ins>(test, profile);
    test_conditions<Linalg, Ahrs>(test, profile);
    test_conditions<Linalg, Ins>(test, profile);
    test_failures<Linalg, Ahrs>(test, profile);
    test_failures<Linalg, Ins>(test, profile);
}

} /* end namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    TestContext test;
    run_conformance<float>(test, "float");
    run_conformance<double>(test, "double");
    if (test.failures() != 0)
    {
        return 1;
    }
    std::cout << "Accelerometer fusion tests passed\n";
    return 0;
}
