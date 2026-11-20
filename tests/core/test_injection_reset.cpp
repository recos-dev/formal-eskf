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
#include <cstddef>
#include <iostream>
#include <limits>
#include <string_view>
#include <type_traits>

// Exercise the standalone header, without relying on the ESKF master header.
#include <formal_eskf/eskf/injection.hpp>
#include <formal_eskf/eskf/types.hpp>
#include "test_backend.hpp"

#include "test_support.hpp"
#include "test_cases.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::inject_and_reset;
using formal_eskf::inject_nominal;
using formal_eskf::reset_covariance;
using formal_eskf::Status;
using formal_eskf::test::near;

template <typename Matrix> bool same_fields(Matrix const & a, Matrix const & b)
{
    using bytes_type = std::array<std::byte, sizeof(typename Matrix::value_type)>;
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            // Preserve NaN classification, not payloads; finite values include signed zero.
            if (!(std::isnan(a(row, column)) && std::isnan(b(row, column))) &&
                std::bit_cast<bytes_type>(a(row, column)) != std::bit_cast<bytes_type>(b(row, column)))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename State> bool same_state(State const & a, State const & b)
{
    bool same = same_fields(a.q_nb.coefficients(), b.q_nb.coefficients());
    if constexpr (requires { a.p_n; })
    {
        same = same && same_fields(a.p_n, b.p_n) && same_fields(a.v_n, b.v_n) && same_fields(a.b_a, b.b_a) &&
               same_fields(a.b_g, b.b_g);
    }
    return same;
}

template <typename Error> bool same_error(Error const & a, Error const & b)
{
    bool same = same_fields(a.delta_theta_b, b.delta_theta_b);
    if constexpr (requires { a.delta_p_n; })
    {
        same = same && same_fields(a.delta_p_n, b.delta_p_n) && same_fields(a.delta_v_n, b.delta_v_n) &&
               same_fields(a.delta_b_a, b.delta_b_a) && same_fields(a.delta_b_g, b.delta_b_g);
    }
    return same;
}

template <typename Linalg, typename Configuration> struct InjectionFixture
{
    using types = formal_eskf::EskfTypes<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    using state_type = typename types::nominal_state_type;
    using error_type = typename types::error_state_type;
    using covariance_type = typename types::error_covariance_type;
    static constexpr bool is_ins = std::is_same_v<Configuration, formal_eskf::configuration::Ins>;
    static constexpr std::size_t size = types::error_state_dimension;
    static constexpr std::size_t attitude_offset = is_ins ? 6U : 0U;
    static constexpr value_type minimum_norm = static_cast<value_type>(1.0e-6);

    state_type state{};
    error_type error{};
    covariance_type covariance{};
    Status attitude_status{};

    InjectionFixture()
    {
        attitude_status = formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(
            value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5}, minimum_norm, state.q_nb);
        error.delta_theta_b(0U) = value_type{0.125};
        error.delta_theta_b(1U) = value_type{-0.0625};
        error.delta_theta_b(2U) = value_type{0.03125};
        if constexpr (is_ins)
        {
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                value_type const n = static_cast<value_type>(axis + 1U);
                state.p_n(axis) = n;
                state.v_n(axis) = -n;
                state.b_a(axis) = n / value_type{8};
                state.b_g(axis) = -n / value_type{16};
                error.delta_p_n(axis) = n / value_type{2};
                error.delta_v_n(axis) = -n / value_type{4};
                error.delta_b_a(axis) = -n / value_type{32};
                error.delta_b_g(axis) = n / value_type{64};
            }
        }
        // D + u*u^T is SPD and has nonzero attitude cross-covariances.
        for (std::size_t row = 0U; row < size; ++row)
        {
            for (std::size_t column = 0U; column < size; ++column)
            {
                value_type const u = static_cast<value_type>(row + 1U) / value_type{32};
                value_type const v = static_cast<value_type>(column + 1U) / value_type{32};
                covariance(row, column) = u * v + (row == column ? value_type{1} : value_type{0});
            }
        }
    }
};

template <typename Linalg, typename Configuration>
void test_injection_mode(TestContext & test, std::string_view profile)
{
    constexpr auto tolerance = static_cast<typename Linalg::value_type>(
        std::is_same_v<typename Linalg::value_type, float> ? 3.0e-6F : 1.0e-12);
    using value_type = typename Linalg::value_type;
    using fixture_type = InjectionFixture<Linalg, Configuration>;
    fixture_type fixture;
    test.expect(fixture.attitude_status == Status::success, profile, "valid non-identity attitude fixture");

    // Supplemental finite-angle regression; these historical tolerances are not
    // an approved general quaternion error envelope. Exact local fixtures below
    // supply native equality cases independently of this regression.
    // Independent axis-angle and scalar Hamilton-product oracle, also valid
    // when prediction was compiled in normalized-Euler mode.
    auto check_attitude = [&](typename fixture_type::error_type const & error)
    {
        long double const x = static_cast<long double>(error.delta_theta_b(0U));
        long double const y = static_cast<long double>(error.delta_theta_b(1U));
        long double const z = static_cast<long double>(error.delta_theta_b(2U));
        long double const theta = std::sqrt(x * x + y * y + z * z);
        long double const scale = theta == 0.0 ? 0.5 : std::sin(theta / 2.0) / theta;
        long double const c = std::cos(theta / 2.0);
        long double const a = x * scale;
        long double const b = y * scale;
        long double const d = z * scale;
        std::array<long double, 4U> const expected{(c - a - b - d) / 2.0, (c + a + d - b) / 2.0, (c + b + a - d) / 2.0,
                                                   (c + d + b - a) / 2.0};
        auto output = fixture.state;
        Status const status = inject_nominal(fixture.state, error, fixture.minimum_norm, output);
        bool correct = status == Status::success;
        auto const coefficients = output.q_nb.coefficients();
        for (std::size_t index = 0U; index < 4U; ++index)
        {
            correct = correct && near(coefficients(index), static_cast<value_type>(expected[index]), tolerance);
        }
        test.expect(correct && near(formal_eskf::linalg::norm(coefficients), value_type{1}, tolerance), profile,
                    "nominal attitude matches right-multiplied Exp and remains normalized");
    };
    check_attitude(fixture.error);
    auto error = fixture.error;
    error.delta_theta_b = {};
    check_attitude(error);
    error.delta_theta_b(0U) = std::sqrt(std::numeric_limits<value_type>::epsilon()) / value_type{4};
    check_attitude(error);
    error.delta_theta_b(1U) = value_type{0.75};
    check_attitude(error); // Nominal Exp is not restricted to the reset approximation's small-angle regime.

    auto output = fixture.state;
    Status const status = inject_nominal(fixture.state, fixture.error, fixture.minimum_norm, output);
    test.expect(status == Status::success, profile, "nominal injection succeeds");
    if constexpr (fixture_type::is_ins)
    {
        test.expect(same_fields(output.p_n, fixture.state.p_n + fixture.error.delta_p_n) &&
                        same_fields(output.v_n, fixture.state.v_n + fixture.error.delta_v_n) &&
                        same_fields(output.b_a, fixture.state.b_a + fixture.error.delta_b_a) &&
                        same_fields(output.b_g, fixture.state.b_g + fixture.error.delta_b_g),
                    profile, "all INS additive errors reach the correct state components");
    }
    auto negative = fixture.state;
    negative.q_nb = -negative.q_nb;
    Status const negative_status = inject_nominal(negative, fixture.error, fixture.minimum_norm, negative);
    test.expect(negative_status == Status::success &&
                    same_fields(negative.q_nb.coefficients(), -output.q_nb.coefficients()),
                profile, "in-place injection respects quaternion sign equivalence");
    auto inverse_error = fixture.error;
    inverse_error.delta_theta_b = -inverse_error.delta_theta_b;
    Status const inverse_status = inject_nominal(output, inverse_error, fixture.minimum_norm, output);
    test.expect(inverse_status == Status::success &&
                    formal_eskf::so3::same_rotation(output.q_nb, fixture.state.q_nb, tolerance),
                profile, "opposite attitude correction recovers the original rotation");
}

template <typename Linalg, typename Configuration>
void test_covariance_reset(TestContext & test, std::string_view profile)
{
    constexpr auto tolerance = static_cast<typename Linalg::value_type>(
        ESKF_RESET_APPROX ? 0.0 : (std::is_same_v<typename Linalg::value_type, float> ? 3.0e-6 : 1.0e-12));
    using value_type = typename Linalg::value_type;
    using fixture_type = InjectionFixture<Linalg, Configuration>;
    constexpr std::size_t size = fixture_type::size;
    constexpr std::size_t offset = fixture_type::attitude_offset;
    fixture_type fixture;

    // Supplemental finite-angle regression in exact-Jacobian mode. Its legacy
    // tolerance is not a general numerical acceptance bound. In first-order
    // mode this dyadic fixture is exactly representable.
    // Independent dense scalar oracle: no production hat(), sandwich(), or
    // backend matrix product is used to construct the expected result.
    std::array<long double, size * size> G{};
    for (std::size_t index = 0U; index < size; ++index)
    {
        G[index * size + index] = 1.0;
    }
    long double const x = static_cast<long double>(fixture.error.delta_theta_b(0U));
    long double const y = static_cast<long double>(fixture.error.delta_theta_b(1U));
    long double const z = static_cast<long double>(fixture.error.delta_theta_b(2U));
    std::array<long double, 9U> const W{0, -z, y, z, 0, -x, -y, x, 0};
#if ESKF_RESET_APPROX
    long double const skew_scale = 0.5;
    long double const square_scale = 0.0;
#else
    long double const theta_squared = x * x + y * y + z * z;
    long double const theta = std::sqrt(theta_squared);
    long double const skew_scale = (1.0 - std::cos(theta)) / theta_squared;
    long double const square_scale = (theta - std::sin(theta)) / (theta_squared * theta);
#endif
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            long double square = 0.0;
            for (std::size_t inner = 0U; inner < 3U; ++inner)
            {
                square += W[row * 3U + inner] * W[inner * 3U + column];
            }
            G[(offset + row) * size + offset + column] += -skew_scale * W[row * 3U + column] + square_scale * square;
        }
    }
    auto output = fixture.covariance;
    Status const status = reset_covariance(fixture.error, fixture.covariance, output);
    bool correct = status == Status::success;
    for (std::size_t row = 0U; row < size; ++row)
    {
        for (std::size_t column = 0U; column < size; ++column)
        {
            long double expected = 0.0;
            for (std::size_t a = 0U; a < size; ++a)
            {
                for (std::size_t b = 0U; b < size; ++b)
                {
                    expected +=
                        G[row * size + a] * static_cast<long double>(fixture.covariance(a, b)) * G[column * size + b];
                }
            }
            correct = correct && near(output(row, column), static_cast<value_type>(expected), tolerance);
        }
    }
    test.expect(correct, profile, "full G*P*G^T agrees with independent oracle, including cross-covariances");
    test.expect(formal_eskf::linalg::is_symmetric(output, value_type{0}), profile, "reset restores exact symmetry");
    test.expect(!same_fields(output, fixture.covariance), profile, "nonzero correction does not silently use G=I");

    auto alias = fixture.covariance;
    Status const alias_status = reset_covariance(fixture.error, alias, alias);
    test.expect(alias_status == Status::success && same_fields(alias, output), profile,
                "reset supports aliased output");
    typename fixture_type::error_type const zero_error{};
    Status const zero_status = reset_covariance(zero_error, fixture.covariance, output);
    test.expect(zero_status == Status::success && same_fields(output, fixture.covariance), profile,
                "zero correction preserves covariance rather than clearing it");

    // Finite rank-one PSD covariance at the scalar limit must not overflow
    // during symmetry cleanup when G is identity.
    for (std::size_t row = 0U; row < size; ++row)
    {
        for (std::size_t column = 0U; column < size; ++column)
        {
            output(row, column) = std::numeric_limits<value_type>::max();
        }
    }
    auto const large = output;
    Status const large_status = reset_covariance(zero_error, output, output);
    test.expect(large_status == Status::success && same_fields(output, large), profile,
                "symmetry cleanup does not overflow for a finite covariance at zero correction");

    // A rank-one PSD prior with its attitude component parallel to the reset
    // axis is unchanged by either J_r or I-hat(theta)/2. Cross-covariances to
    // every Euclidean coordinate remain nonzero in INS.
    auto rank_one = fixture.covariance;
    for (std::size_t row = 0U; row < size; ++row)
    {
        value_type const u = (row == offset + 1U || row == offset + 2U) ? value_type{0} : value_type{1};
        for (std::size_t column = 0U; column < size; ++column)
        {
            value_type const v = (column == offset + 1U || column == offset + 2U) ? value_type{0} : value_type{1};
            rank_one(row, column) = u * v;
        }
    }
    auto axis_error = fixture.error;
    axis_error.delta_theta_b = {};
    axis_error.delta_theta_b(0U) = value_type{0.25};
    auto const axis_before = axis_error;
    auto const rank_before = rank_one;
    test.expect(reset_covariance(axis_error, rank_one, output) == Status::success && same_fields(output, rank_one) &&
                    same_fields(rank_one, rank_before) && same_error(axis_error, axis_before),
                profile, "rotation-axis rank-one PSD and cross-covariances are exactly invariant; inputs preserved");
    typename fixture_type::covariance_type const zero_covariance{};
    test.expect(reset_covariance(fixture.error, zero_covariance, output) == Status::success &&
                    same_fields(output, zero_covariance),
                profile, "zero PSD covariance remains zero under nonzero correction");

    auto invalid = fixture.error;
    invalid.delta_theta_b(1U) = std::numeric_limits<value_type>::quiet_NaN();
    auto const saved = output;
    test.expect(reset_covariance(invalid, fixture.covariance, output) == Status::non_finite_input &&
                    same_fields(output, saved),
                profile, "standalone reset rejects invalid attitude error atomically");
}

template <typename Linalg, typename Configuration>
void test_atomic_injection(TestContext & test, std::string_view profile)
{
    using fixture_type = InjectionFixture<Linalg, Configuration>;
    fixture_type fixture;
    // This is a wiring/alias oracle only. Independent local and covariance
    // equations are exercised by their separate case callbacks.
    auto expected_state = fixture.state;
    auto expected_covariance = fixture.covariance;
    typename fixture_type::error_type const zero_error{};
    test.expect(inject_nominal(fixture.state, fixture.error, fixture.minimum_norm, expected_state) == Status::success &&
                    reset_covariance(fixture.error, fixture.covariance, expected_covariance) == Status::success,
                profile, "separately called injection and reset steps succeed");
    for (unsigned mask = 0U; mask < 8U; ++mask)
    {
        auto input = fixture;
        typename fixture_type::state_type state_output{};
        typename fixture_type::covariance_type covariance_output{};
        auto error_output = fixture.error;
        auto const state_before = state_output;
        auto const covariance_before = covariance_output;
        auto const error_before = error_output;
        auto & state = (mask & 1U) != 0U ? input.state : state_output;
        auto & covariance = (mask & 2U) != 0U ? input.covariance : covariance_output;
        auto & error = (mask & 4U) != 0U ? input.error : error_output;
        Status const status = inject_and_reset(input.state, input.covariance, input.error, fixture.minimum_norm, state,
                                               covariance, error);
        test.expect(status == Status::success && same_state(state, expected_state) &&
                        same_fields(covariance, expected_covariance) && same_error(error, zero_error),
                    profile, "all eight alias combinations commit state, covariance and zero error mean");
        test.expect(((mask & 1U) != 0U || same_state(input.state, fixture.state)) &&
                        ((mask & 2U) != 0U || same_fields(input.covariance, fixture.covariance)) &&
                        ((mask & 4U) != 0U || same_error(input.error, fixture.error)),
                    profile, "non-aliased inputs remain unchanged");
        test.expect(((mask & 1U) == 0U || same_state(state_output, state_before)) &&
                        ((mask & 2U) == 0U || same_fields(covariance_output, covariance_before)) &&
                        ((mask & 4U) == 0U || same_error(error_output, error_before)),
                    profile, "unused separate output objects remain unchanged");
    }
    auto exact = fixture;
    exact.error.delta_theta_b = {};
    for (unsigned mask = 0U; mask < 8U; ++mask)
    {
        auto prior = exact;
        typename fixture_type::state_type state_output;
        typename fixture_type::covariance_type covariance_output;
        typename fixture_type::error_type error_output;
        auto & state = (mask & 1U) != 0U ? prior.state : state_output;
        auto & covariance = (mask & 2U) != 0U ? prior.covariance : covariance_output;
        auto & error = (mask & 4U) != 0U ? prior.error : error_output;
        test.expect(inject_and_reset(prior.state, prior.covariance, prior.error, fixture.minimum_norm, state,
                                     covariance, error) == Status::success &&
                        same_fields(state.q_nb.coefficients(), exact.state.q_nb.coefficients()) &&
                        same_fields(covariance, exact.covariance) && same_error(error, zero_error),
                    profile, "independent zero-angle transaction preserves covariance and publishes zero error");
        if constexpr (fixture_type::is_ins)
        {
            using value_type = typename fixture_type::value_type;
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                value_type const n = static_cast<value_type>(axis + 1U);
                test.expect(state.p_n(axis) == value_type{1.5} * n && state.v_n(axis) == value_type{-1.25} * n &&
                                state.b_a(axis) == value_type{3} * n / value_type{32} &&
                                state.b_g(axis) == -value_type{3} * n / value_type{64},
                            profile, "complete transaction has independent exact additive-state oracle");
            }
        }
    }
    auto const initial_state = fixture.state;
    auto const initial_covariance = fixture.covariance;
    auto error = zero_error;
    Status const status = inject_and_reset(fixture.state, fixture.covariance, error, fixture.minimum_norm,
                                           fixture.state, fixture.covariance, error);
    test.expect(status == Status::success && same_state(fixture.state, initial_state) &&
                    same_fields(fixture.covariance, initial_covariance) && same_error(error, zero_error),
                profile, "zero-error full step leaves the basis fixture unchanged");
}

template <typename Fixture>
void expect_failed_injection(TestContext & test, std::string_view profile, Fixture const & input,
                             typename Fixture::value_type minimum_norm, Status expected)
{
    using value_type = typename Fixture::value_type;
    value_type const nan = std::numeric_limits<value_type>::quiet_NaN();
    value_type const infinity = std::numeric_limits<value_type>::infinity();
    Fixture sentinel;
    sentinel.state.q_nb = -sentinel.state.q_nb;
    sentinel.covariance = sentinel.covariance * value_type{2};
    sentinel.covariance(0U, 0U) = nan;
    sentinel.covariance(1U, 2U) = -value_type{0};
    sentinel.covariance(2U, 2U) = infinity;
    sentinel.error.delta_theta_b(0U) = nan;
    sentinel.error.delta_theta_b(1U) = infinity;
    sentinel.error.delta_theta_b(2U) = -value_type{0};
    if constexpr (Fixture::is_ins)
    {
        sentinel.state.p_n(0U) = nan;
        sentinel.state.v_n(1U) = infinity;
        sentinel.state.b_a(2U) = -value_type{0};
        sentinel.error.delta_p_n(0U) = nan;
        sentinel.error.delta_v_n(1U) = infinity;
        sentinel.error.delta_b_a(2U) = -value_type{0};
    }
    for (unsigned mask = 0U; mask < 8U; ++mask)
    {
        auto source = input;
        auto output = sentinel;
        auto & state = (mask & 1U) != 0U ? source.state : output.state;
        auto & covariance = (mask & 2U) != 0U ? source.covariance : output.covariance;
        auto & error = (mask & 4U) != 0U ? source.error : output.error;
        Status const status =
            inject_and_reset(source.state, source.covariance, source.error, minimum_norm, state, covariance, error);
        test.expect(status == expected && same_state(source.state, input.state) &&
                        same_fields(source.covariance, input.covariance) && same_error(source.error, input.error) &&
                        same_state(output.state, sentinel.state) &&
                        same_fields(output.covariance, sentinel.covariance) && same_error(output.error, sentinel.error),
                    profile, "failure preserves all inputs and sentinel outputs for all eight alias combinations");
    }
}

template <typename Linalg, typename Configuration>
void test_injection_failures(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using fixture_type = InjectionFixture<Linalg, Configuration>;
    fixture_type fixture;
    value_type const nan = std::numeric_limits<value_type>::quiet_NaN();
    value_type const infinity = std::numeric_limits<value_type>::infinity();
    value_type const maximum = std::numeric_limits<value_type>::max();
    for (value_type const minimum_norm : {value_type{0}, value_type{-1}, value_type{1.25}})
    {
        expect_failed_injection(test, profile, fixture, minimum_norm, Status::domain_error);
    }
    expect_failed_injection(test, profile, fixture, nan, Status::non_finite_input);
    expect_failed_injection(test, profile, fixture, infinity, Status::non_finite_input);
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        auto bad = fixture;
        bad.error.delta_theta_b(axis) = nan;
        expect_failed_injection(test, profile, bad, fixture.minimum_norm, Status::non_finite_input);
        bad.error.delta_theta_b(axis) = infinity;
        expect_failed_injection(test, profile, bad, fixture.minimum_norm, Status::non_finite_input);
        bad.error.delta_theta_b(axis) = maximum;
        expect_failed_injection(test, profile, bad, fixture.minimum_norm, Status::non_finite_result);
    }
    // Test standalone nominal rollback independently of the atomic wrapper.
    auto bad_error = fixture.error;
    bad_error.delta_theta_b(0U) = maximum;
    auto output = fixture.state;
    Status const status = inject_nominal(output, bad_error, fixture.minimum_norm, output);
    test.expect(status == Status::non_finite_result && same_state(output, fixture.state), profile,
                "standalone nominal injection rolls back on Exp overflow");

    if constexpr (fixture_type::is_ins)
    {
        using state_type = typename fixture_type::state_type;
        using error_type = typename fixture_type::error_type;
        auto const state_members = std::array{&state_type::p_n, &state_type::v_n, &state_type::b_a, &state_type::b_g};
        auto const error_members =
            std::array{&error_type::delta_p_n, &error_type::delta_v_n, &error_type::delta_b_a, &error_type::delta_b_g};
        for (std::size_t group = 0U; group < state_members.size(); ++group)
        {
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                auto bad = fixture;
                (bad.state.*state_members[group])(axis) = nan;
                expect_failed_injection(test, profile, bad, fixture.minimum_norm, Status::non_finite_input);
                bad = fixture;
                (bad.error.*error_members[group])(axis) = infinity;
                expect_failed_injection(test, profile, bad, fixture.minimum_norm, Status::non_finite_input);
                bad = fixture;
                (bad.state.*state_members[group])(axis) = maximum;
                (bad.error.*error_members[group])(axis) = maximum;
                expect_failed_injection(test, profile, bad, fixture.minimum_norm, Status::non_finite_result);
                auto const saved = bad.state;
                Status const nominal_status = inject_nominal(bad.state, bad.error, fixture.minimum_norm, bad.state);
                test.expect(nominal_status == Status::non_finite_result && same_state(bad.state, saved), profile,
                            "late additive overflow does not partially inject attitude or other INS components");
            }
        }
    }
}

template <typename Linalg, typename Configuration> void test_first_failure(TestContext & test, std::string_view profile)
{
    using fixture_type = InjectionFixture<Linalg, Configuration>;
    using value_type = typename fixture_type::value_type;
    fixture_type fixture;
    fixture.covariance(0U, 0U) = std::numeric_limits<value_type>::quiet_NaN();
    auto state_output = fixture.state;
    auto covariance_output = fixture.covariance;
    test.expect(inject_nominal(fixture.state, fixture.error, value_type{0}, state_output) == Status::domain_error,
                profile, "nominal injection rejects the invalid normalization threshold");
    test.expect(reset_covariance(fixture.error, fixture.covariance, covariance_output) == Status::non_finite_input,
                profile, "covariance reset would return a distinct failure");
    expect_failed_injection(test, profile, fixture, value_type{0}, Status::domain_error);
}

template <typename Linalg, typename Configuration> void test_late_failure(TestContext & test, std::string_view profile)
{
    using fixture_type = InjectionFixture<Linalg, Configuration>;
    using value_type = typename fixture_type::value_type;
    fixture_type fixture;
    value_type const nan = std::numeric_limits<value_type>::quiet_NaN();
    value_type const infinity = std::numeric_limits<value_type>::infinity();
    value_type const maximum = std::numeric_limits<value_type>::max();
    auto state_output = fixture.state;
    test.expect(inject_nominal(fixture.state, fixture.error, fixture.minimum_norm, state_output) == Status::success &&
                    !same_state(state_output, fixture.state),
                profile, "nominal injection makes progress before the covariance-only failures");
    for (std::size_t row = 0U; row < fixture_type::size; ++row)
    {
        for (std::size_t column = 0U; column < fixture_type::size; ++column)
        {
            auto bad = fixture;
            bad.covariance(row, column) = (row == column) ? infinity : nan;
            expect_failed_injection(test, profile, bad, fixture.minimum_norm, Status::non_finite_input);
        }
    }
    auto overflow = fixture;
    // A finite PSD rank-one input whose transformed entries exceed the scalar
    // range in BOTH reset modes (J_r need not amplify an isotropic covariance).
    for (std::size_t row = 0U; row < fixture_type::size; ++row)
    {
        for (std::size_t column = 0U; column < fixture_type::size; ++column)
        {
            overflow.covariance(row, column) = maximum;
        }
    }
    expect_failed_injection(test, profile, overflow, fixture.minimum_norm, Status::non_finite_result);
}

template <typename Linalg, typename Configuration> void test_local_map(TestContext & test, std::string_view profile)
{
    using fixture_type = InjectionFixture<Linalg, Configuration>;
    using value_type = typename fixture_type::value_type;
    fixture_type fixture;
    test.expect(formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(value_type{0}, value_type{1}, value_type{0},
                                                                            value_type{0}, fixture.minimum_norm,
                                                                            fixture.state.q_nb) == Status::success,
                profile, "construct exact half-turn about body x");
    value_type const small = std::ldexp(value_type{1}, -std::numeric_limits<value_type>::digits);
    for (bool const zero : {true, false})
    {
        fixture.error.delta_theta_b(0U) = zero ? value_type{0} : small;
        fixture.error.delta_theta_b(1U) = zero ? value_type{0} : -small;
        fixture.error.delta_theta_b(2U) = zero ? value_type{0} : small * value_type{2};
        // Higher Taylor terms and squared-norm residual round away for this
        // fixture. Independent Hamilton multiplication by [0,1,0,0] gives
        // [-a,1,-c,b], where (a,b,c)=delta_theta/2.
        std::array<value_type, 4U> const expected{-fixture.error.delta_theta_b(0U) / value_type{2}, value_type{1},
                                                  -fixture.error.delta_theta_b(2U) / value_type{2},
                                                  fixture.error.delta_theta_b(1U) / value_type{2}};
        for (bool const negative : {false, true})
        {
            for (bool const alias : {false, true})
            {
                auto prior = fixture.state;
                if (negative)
                {
                    prior.q_nb = -prior.q_nb;
                }
                auto const prior_before = prior;
                auto const error_before = fixture.error;
                auto const covariance_before = fixture.covariance;
                typename fixture_type::state_type separate;
                auto & output = alias ? prior : separate;
                test.expect(inject_nominal(prior, fixture.error, fixture.minimum_norm, output) == Status::success,
                            profile, "exact local injection succeeds for either alias/sign");
                for (std::size_t axis = 0U; axis < 4U; ++axis)
                {
                    test.expect(output.q_nb.coefficients()(axis) == (negative ? -expected[axis] : expected[axis]),
                                profile, "right-Hamilton tangent coefficients match exact dyadic oracle");
                }
                if constexpr (fixture_type::is_ins)
                {
                    for (std::size_t axis = 0U; axis < 3U; ++axis)
                    {
                        value_type const n = static_cast<value_type>(axis + 1U);
                        test.expect(output.p_n(axis) == value_type{1.5} * n &&
                                        output.v_n(axis) == value_type{-1.25} * n &&
                                        output.b_a(axis) == value_type{3} * n / value_type{32} &&
                                        output.b_g(axis) == -value_type{3} * n / value_type{64},
                                    profile, "independent additive coordinates reach only their named blocks");
                    }
                }
                test.expect((alias || same_state(prior, prior_before)) && same_error(fixture.error, error_before) &&
                                same_fields(fixture.covariance, covariance_before),
                            profile, "standalone injection preserves distinct prior, error and covariance");
            }
        }
    }
}

template <typename Fixture>
void expect_nominal_failure(TestContext & test, std::string_view profile, Fixture const & fixture,
                            typename Fixture::value_type minimum_norm, Status expected)
{
    using value_type = typename Fixture::value_type;
    for (bool const alias : {false, true})
    {
        auto prior = fixture.state;
        auto output = fixture.state;
        output.q_nb = -output.q_nb;
        if constexpr (Fixture::is_ins)
        {
            output.p_n(0U) = std::numeric_limits<value_type>::quiet_NaN();
            output.v_n(1U) = -value_type{0};
        }
        auto const output_before = output;
        auto const error_before = fixture.error;
        auto & result = alias ? prior : output;
        test.expect(inject_nominal(prior, fixture.error, minimum_norm, result) == expected &&
                        same_state(prior, fixture.state) && same_state(output, output_before) &&
                        same_error(fixture.error, error_before),
                    profile, "standalone failure preserves full state and distinct error for both aliases");
    }
}

template <typename Linalg, typename Configuration>
void test_nominal_failure_frame(TestContext & test, std::string_view profile)
{
    using fixture_type = InjectionFixture<Linalg, Configuration>;
    using value_type = typename fixture_type::value_type;
    fixture_type fixture;
    value_type const nan = std::numeric_limits<value_type>::quiet_NaN();
    value_type const infinity = std::numeric_limits<value_type>::infinity();
    value_type const maximum = std::numeric_limits<value_type>::max();
    for (value_type const threshold : {nan, infinity, -infinity})
    {
        expect_nominal_failure(test, profile, fixture, threshold, Status::non_finite_input);
    }
    for (value_type const threshold : {value_type{0}, value_type{-1}, std::nextafter(value_type{1}, value_type{2})})
    {
        expect_nominal_failure(test, profile, fixture, threshold, Status::domain_error);
    }
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        for (value_type const invalid : {nan, infinity, -infinity, maximum})
        {
            auto bad = fixture;
            bad.error.delta_theta_b(axis) = invalid;
            expect_nominal_failure(test, profile, bad, fixture.minimum_norm,
                                   std::isfinite(invalid) ? Status::non_finite_result : Status::non_finite_input);
        }
    }
    if constexpr (fixture_type::is_ins)
    {
        using state_type = typename fixture_type::state_type;
        using error_type = typename fixture_type::error_type;
        auto const states = std::array{&state_type::p_n, &state_type::v_n, &state_type::b_a, &state_type::b_g};
        auto const errors =
            std::array{&error_type::delta_p_n, &error_type::delta_v_n, &error_type::delta_b_a, &error_type::delta_b_g};
        for (std::size_t group = 0U; group < states.size(); ++group)
        {
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                for (value_type const invalid : {nan, infinity, -infinity})
                {
                    auto bad = fixture;
                    (bad.state.*states[group])(axis) = invalid;
                    expect_nominal_failure(test, profile, bad, fixture.minimum_norm, Status::non_finite_input);
                    bad = fixture;
                    (bad.error.*errors[group])(axis) = invalid;
                    expect_nominal_failure(test, profile, bad, fixture.minimum_norm, Status::non_finite_input);
                }
                auto bad = fixture;
                (bad.state.*states[group])(axis) = maximum;
                (bad.error.*errors[group])(axis) = maximum;
                expect_nominal_failure(test, profile, bad, fixture.minimum_norm, Status::non_finite_result);
            }
        }
    }
    // A public normalization underflow fixture reaches a non-unit prior without
    // private-storage access. Injection repairs it; prediction's tolerance is
    // deliberately not an injection policy.
    int const exponent = (std::numeric_limits<value_type>::min_exponent - std::numeric_limits<value_type>::digits) / 2;
    value_type const tiny = std::ldexp(value_type{1}, exponent);
    test.expect(formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(
                    tiny, tiny / value_type{2}, value_type{0}, value_type{0}, std::numeric_limits<value_type>::min(),
                    fixture.state.q_nb) == Status::success &&
                    fixture.state.q_nb.q0() == value_type{1} && fixture.state.q_nb.q1() == value_type{0.5},
                profile, "public construction reaches specified nonunit guard fixture");
    fixture.error = {};
    auto repaired = fixture.state;
    test.expect(inject_nominal(fixture.state, fixture.error, fixture.minimum_norm, repaired) == Status::success,
                profile, "finite nonunit prior is subject to normalization feasibility, not prediction tolerance");
    auto large = fixture.state.q_nb;
    value_type const norm_overflow_limit = std::sqrt(maximum);
    for (std::size_t step = 0U;
         step < 20U && std::abs(large.q0()) <= norm_overflow_limit && std::abs(large.q1()) <= norm_overflow_limit;
         ++step)
    {
        large = large * large;
    }
    fixture.state.q_nb = large;
    test.expect(std::isfinite(large.q0()) && std::isfinite(large.q1()) &&
                    (std::abs(large.q0()) > norm_overflow_limit || std::abs(large.q1()) > norm_overflow_limit),
                profile, "public multiplication reaches finite coefficients whose squared norm overflows");
    expect_nominal_failure(test, profile, fixture, fixture.minimum_norm, Status::non_finite_result);
    for (std::size_t step = 0U; step < 20U && std::isfinite(large.q0()) && std::isfinite(large.q1()); ++step)
    {
        large = large * large;
    }
    fixture.state.q_nb = large;
    test.expect(!std::isfinite(large.q0()) || !std::isfinite(large.q1()), profile,
                "public multiplication reaches nonfinite prior without layout assumptions");
    expect_nominal_failure(test, profile, fixture, fixture.minimum_norm, Status::non_finite_input);
}

template <typename Linalg, typename Configuration>
void test_reset_consumption_frame(TestContext & test, std::string_view profile)
{
    using fixture_type = InjectionFixture<Linalg, Configuration>;
    using value_type = typename fixture_type::value_type;
    fixture_type fixture;
    auto reference = fixture.covariance;
    test.expect(reset_covariance(fixture.error, fixture.covariance, reference) == Status::success, profile,
                "valid reset for consumed-field metamorphic comparison");
    if constexpr (fixture_type::is_ins)
    {
        using error_type = typename fixture_type::error_type;
        auto const members =
            std::array{&error_type::delta_p_n, &error_type::delta_v_n, &error_type::delta_b_a, &error_type::delta_b_g};
        for (auto member : members)
        {
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                for (value_type const invalid :
                     {std::numeric_limits<value_type>::quiet_NaN(), std::numeric_limits<value_type>::infinity(),
                      -std::numeric_limits<value_type>::infinity()})
                {
                    auto error = fixture.error;
                    (error.*member)(axis) = invalid;
                    auto const error_before = error;
                    auto output = fixture.covariance;
                    test.expect(reset_covariance(error, fixture.covariance, output) == Status::success &&
                                    same_fields(output, reference) && same_error(error, error_before),
                                profile, "every unused Euclidean error coordinate is ignored and preserved");
                }
            }
        }
    }
    auto check = [&](fixture_type const & input, Status expected)
    {
        for (bool const alias : {false, true})
        {
            auto covariance = input.covariance;
            auto output = fixture.covariance;
            output(0U, 0U) = std::numeric_limits<value_type>::quiet_NaN();
            output(1U, 2U) = -value_type{0};
            auto const saved = output;
            auto const error_before = input.error;
            auto & result = alias ? covariance : output;
            test.expect(reset_covariance(input.error, covariance, result) == expected &&
                            same_fields(covariance, input.covariance) && same_fields(output, saved) &&
                            same_error(input.error, error_before),
                        profile, "reset failure preserves all covariance entries and error for both aliases");
        }
    };
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        for (value_type const invalid :
             {std::numeric_limits<value_type>::quiet_NaN(), std::numeric_limits<value_type>::infinity(),
              -std::numeric_limits<value_type>::infinity()})
        {
            auto bad = fixture;
            bad.error.delta_theta_b(axis) = invalid;
            check(bad, Status::non_finite_input);
        }
    }
    for (std::size_t row = 0U; row < fixture_type::size; ++row)
    {
        for (std::size_t column = 0U; column < fixture_type::size; ++column)
        {
            auto bad = fixture;
            bad.covariance(row, column) = std::numeric_limits<value_type>::quiet_NaN();
            check(bad, Status::non_finite_input);
        }
    }
    auto overflow = fixture;
    for (std::size_t row = 0U; row < fixture_type::size; ++row)
    {
        for (std::size_t column = 0U; column < fixture_type::size; ++column)
        {
            overflow.covariance(row, column) = std::numeric_limits<value_type>::max();
        }
    }
    check(overflow, Status::non_finite_result);
    overflow = fixture;
    overflow.error.delta_theta_b(0U) = std::numeric_limits<value_type>::max();
    check(overflow, Status::non_finite_result);
    auto aliased = fixture.covariance;
    auto const error_before = fixture.error;
    test.expect(reset_covariance(fixture.error, aliased, aliased) == Status::success &&
                    same_fields(aliased, reference) && same_error(fixture.error, error_before),
                profile, "successful whole-P alias uses the original covariance and preserves error");
}

template <typename Linalg, typename Configuration> void test_transaction(TestContext & test, std::string_view profile)
{
    test_atomic_injection<Linalg, Configuration>(test, profile);
    test_injection_failures<Linalg, Configuration>(test, profile);
    test_first_failure<Linalg, Configuration>(test, profile);
    test_late_failure<Linalg, Configuration>(test, profile);
}

} /* end namespace */

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using FloatLinalg = formal_eskf::test::Backend<float>;
    using DoubleLinalg = formal_eskf::test::Backend<double>;
    using formal_eskf::configuration::Ahrs;
    using formal_eskf::configuration::Ins;
    formal_eskf::test::Case const cases[] = {
        {"TC-E-INJECT-local-map", "INS.binary32", test_local_map<FloatLinalg, Ins>},
        {"TC-E-INJECT-local-map", "INS.binary64", test_local_map<DoubleLinalg, Ins>},
        {"TC-E-INJECT-local-map", "AHRS.binary32", test_local_map<FloatLinalg, Ahrs>},
        {"TC-E-INJECT-local-map", "AHRS.binary64", test_local_map<DoubleLinalg, Ahrs>},
        {"TC-E-INJECT-mode-independence", "INS.binary32", test_injection_mode<FloatLinalg, Ins>},
        {"TC-E-INJECT-mode-independence", "INS.binary64", test_injection_mode<DoubleLinalg, Ins>},
        {"TC-E-INJECT-mode-independence", "AHRS.binary32", test_injection_mode<FloatLinalg, Ahrs>},
        {"TC-E-INJECT-mode-independence", "AHRS.binary64", test_injection_mode<DoubleLinalg, Ahrs>},
        {"TC-E-INJECT-failure-frame", "INS.binary32", test_nominal_failure_frame<FloatLinalg, Ins>},
        {"TC-E-INJECT-failure-frame", "INS.binary64", test_nominal_failure_frame<DoubleLinalg, Ins>},
        {"TC-E-INJECT-failure-frame", "AHRS.binary32", test_nominal_failure_frame<FloatLinalg, Ahrs>},
        {"TC-E-INJECT-failure-frame", "AHRS.binary64", test_nominal_failure_frame<DoubleLinalg, Ahrs>},
        {"TC-E-RESET-full-transport", "INS.binary32", test_covariance_reset<FloatLinalg, Ins>},
        {"TC-E-RESET-full-transport", "INS.binary64", test_covariance_reset<DoubleLinalg, Ins>},
        {"TC-E-RESET-full-transport", "AHRS.binary32", test_covariance_reset<FloatLinalg, Ahrs>},
        {"TC-E-RESET-full-transport", "AHRS.binary64", test_covariance_reset<DoubleLinalg, Ahrs>},
        {"TC-E-RESET-consumption-frame", "INS.binary32", test_reset_consumption_frame<FloatLinalg, Ins>},
        {"TC-E-RESET-consumption-frame", "INS.binary64", test_reset_consumption_frame<DoubleLinalg, Ins>},
        {"TC-E-RESET-consumption-frame", "AHRS.binary32", test_reset_consumption_frame<FloatLinalg, Ahrs>},
        {"TC-E-RESET-consumption-frame", "AHRS.binary64", test_reset_consumption_frame<DoubleLinalg, Ahrs>},
        {"TC-E-STEP-INJECT-RESET-transaction", "INS.binary32", test_transaction<FloatLinalg, Ins>},
        {"TC-E-STEP-INJECT-RESET-transaction", "INS.binary64", test_transaction<DoubleLinalg, Ins>},
        {"TC-E-STEP-INJECT-RESET-transaction", "AHRS.binary32", test_transaction<FloatLinalg, Ahrs>},
        {"TC-E-STEP-INJECT-RESET-transaction", "AHRS.binary64", test_transaction<DoubleLinalg, Ahrs>},
    };
    std::cout << "Configuration: reset_approx=" << ESKF_RESET_APPROX << " codegen=" << ESKF_CODEGEN << '\n';
    return formal_eskf::test::run_cases(cases, argc, argv);
}
