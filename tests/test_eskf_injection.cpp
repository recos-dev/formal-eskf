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

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::Status;
using formal_eskf::try_inject_and_reset;
using formal_eskf::try_inject_nominal;
using formal_eskf::try_reset_covariance;
using formal_eskf::test::near;

template <typename Matrix> [[nodiscard]] bool same_bits(Matrix const & a, Matrix const & b)
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

template <typename State> [[nodiscard]] bool same_state(State const & a, State const & b)
{
    bool same = same_bits(a.q_nb.coefficients(), b.q_nb.coefficients());
    if constexpr (requires { a.p_n; })
    {
        same = same && same_bits(a.p_n, b.p_n) && same_bits(a.v_n, b.v_n) && same_bits(a.b_a, b.b_a) &&
               same_bits(a.b_g, b.b_g);
    }
    return same;
}

template <typename Error> [[nodiscard]] bool same_error(Error const & a, Error const & b)
{
    bool same = same_bits(a.delta_theta_b, b.delta_theta_b);
    if constexpr (requires { a.delta_p_n; })
    {
        same = same && same_bits(a.delta_p_n, b.delta_p_n) && same_bits(a.delta_v_n, b.delta_v_n) &&
               same_bits(a.delta_b_a, b.delta_b_a) && same_bits(a.delta_b_g, b.delta_b_g);
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
        attitude_status = formal_eskf::so3::UnitQuaternion<Linalg>::try_from_coefficients(
            value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5}, minimum_norm, state.q_nb);
        error.delta_theta_b.set(0U, value_type{0.125});
        error.delta_theta_b.set(1U, value_type{-0.0625});
        error.delta_theta_b.set(2U, value_type{0.03125});
        if constexpr (is_ins)
        {
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                value_type const n = static_cast<value_type>(axis + 1U);
                state.p_n.set(axis, n);
                state.v_n.set(axis, -n);
                state.b_a.set(axis, n / value_type{8});
                state.b_g.set(axis, -n / value_type{16});
                error.delta_p_n.set(axis, n / value_type{2});
                error.delta_v_n.set(axis, -n / value_type{4});
                error.delta_b_a.set(axis, -n / value_type{32});
                error.delta_b_g.set(axis, n / value_type{64});
            }
        }
        // D + u*u^T is SPD and has nonzero attitude cross-covariances.
        for (std::size_t row = 0U; row < size; ++row)
        {
            for (std::size_t column = 0U; column < size; ++column)
            {
                value_type const u = static_cast<value_type>(row + 1U) / value_type{32};
                value_type const v = static_cast<value_type>(column + 1U) / value_type{32};
                covariance.set(row, column, u * v + (row == column ? value_type{1} : value_type{0}));
            }
        }
    }
};

template <typename Linalg, typename Configuration>
void test_nominal_injection(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    using fixture_type = InjectionFixture<Linalg, Configuration>;
    fixture_type fixture;
    test.expect(fixture.attitude_status == Status::success, profile, "valid non-identity attitude fixture");

    // Independent axis-angle and scalar Hamilton-product oracle, also valid
    // when prediction was compiled in normalized-Euler mode.
    auto check_attitude = [&](typename fixture_type::error_type const & error)
    {
        double const x = static_cast<double>(error.delta_theta_b(0U));
        double const y = static_cast<double>(error.delta_theta_b(1U));
        double const z = static_cast<double>(error.delta_theta_b(2U));
        double const theta = std::sqrt(x * x + y * y + z * z);
        double const scale = theta == 0.0 ? 0.5 : std::sin(theta / 2.0) / theta;
        double const c = std::cos(theta / 2.0);
        double const a = x * scale;
        double const b = y * scale;
        double const d = z * scale;
        std::array<double, 4U> const expected{(c - a - b - d) / 2.0, (c + a + d - b) / 2.0, (c + b + a - d) / 2.0,
                                              (c + d + b - a) / 2.0};
        auto output = fixture.state;
        Status const status = try_inject_nominal(fixture.state, error, fixture.minimum_norm, output);
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
    error.delta_theta_b.set(0U, std::sqrt(std::numeric_limits<value_type>::epsilon()) / value_type{4});
    check_attitude(error);
    error.delta_theta_b.set(1U, value_type{0.75});
    check_attitude(error); // Nominal Exp is not restricted to the reset approximation's small-angle regime.

    auto output = fixture.state;
    Status const status = try_inject_nominal(fixture.state, fixture.error, fixture.minimum_norm, output);
    test.expect(status == Status::success, profile, "nominal injection succeeds");
    if constexpr (fixture_type::is_ins)
    {
        test.expect(same_bits(output.p_n, fixture.state.p_n + fixture.error.delta_p_n) &&
                        same_bits(output.v_n, fixture.state.v_n + fixture.error.delta_v_n) &&
                        same_bits(output.b_a, fixture.state.b_a + fixture.error.delta_b_a) &&
                        same_bits(output.b_g, fixture.state.b_g + fixture.error.delta_b_g),
                    profile, "all INS additive errors reach the correct state components");
    }
    auto negative = fixture.state;
    negative.q_nb = -negative.q_nb;
    Status const negative_status = try_inject_nominal(negative, fixture.error, fixture.minimum_norm, negative);
    test.expect(negative_status == Status::success &&
                    same_bits(negative.q_nb.coefficients(), -output.q_nb.coefficients()),
                profile, "in-place injection respects quaternion sign equivalence");
    auto inverse_error = fixture.error;
    inverse_error.delta_theta_b = -inverse_error.delta_theta_b;
    Status const inverse_status = try_inject_nominal(output, inverse_error, fixture.minimum_norm, output);
    test.expect(inverse_status == Status::success &&
                    formal_eskf::so3::same_rotation(output.q_nb, fixture.state.q_nb, tolerance),
                profile, "opposite attitude correction recovers the original rotation");
}

template <typename Linalg, typename Configuration>
void test_covariance_reset(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    using fixture_type = InjectionFixture<Linalg, Configuration>;
    constexpr std::size_t size = fixture_type::size;
    constexpr std::size_t offset = fixture_type::attitude_offset;
    fixture_type fixture;

    // Independent dense scalar oracle: no production hat(), sandwich(), or
    // backend matrix product is used to construct the expected result.
    std::array<double, size * size> G{};
    for (std::size_t index = 0U; index < size; ++index)
    {
        G[index * size + index] = 1.0;
    }
    double const x = static_cast<double>(fixture.error.delta_theta_b(0U));
    double const y = static_cast<double>(fixture.error.delta_theta_b(1U));
    double const z = static_cast<double>(fixture.error.delta_theta_b(2U));
    std::array<double, 9U> const W{0, -z, y, z, 0, -x, -y, x, 0};
#if ESKF_RESET_APPROX
    double const skew_scale = 0.5;
    double const square_scale = 0.0;
#else
    double const theta_squared = x * x + y * y + z * z;
    double const theta = std::sqrt(theta_squared);
    double const skew_scale = (1.0 - std::cos(theta)) / theta_squared;
    double const square_scale = (theta - std::sin(theta)) / (theta_squared * theta);
#endif
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            double square = 0.0;
            for (std::size_t inner = 0U; inner < 3U; ++inner)
            {
                square += W[row * 3U + inner] * W[inner * 3U + column];
            }
            G[(offset + row) * size + offset + column] += -skew_scale * W[row * 3U + column] + square_scale * square;
        }
    }
    auto output = fixture.covariance;
    Status const status = try_reset_covariance(fixture.error, fixture.covariance, output);
    bool correct = status == Status::success;
    for (std::size_t row = 0U; row < size; ++row)
    {
        for (std::size_t column = 0U; column < size; ++column)
        {
            double expected = 0.0;
            for (std::size_t a = 0U; a < size; ++a)
            {
                for (std::size_t b = 0U; b < size; ++b)
                {
                    expected +=
                        G[row * size + a] * static_cast<double>(fixture.covariance(a, b)) * G[column * size + b];
                }
            }
            correct = correct && near(output(row, column), static_cast<value_type>(expected), tolerance);
        }
    }
    test.expect(correct, profile, "full G*P*G^T agrees with independent oracle, including cross-covariances");
    test.expect(formal_eskf::linalg::is_symmetric(output, value_type{0}), profile, "reset restores exact symmetry");
    test.expect(!same_bits(output, fixture.covariance), profile, "nonzero correction does not silently use G=I");

    auto alias = fixture.covariance;
    Status const alias_status = try_reset_covariance(fixture.error, alias, alias);
    test.expect(alias_status == Status::success && same_bits(alias, output), profile, "reset supports aliased output");
    typename fixture_type::error_type const zero_error{};
    Status const zero_status = try_reset_covariance(zero_error, fixture.covariance, output);
    test.expect(zero_status == Status::success && same_bits(output, fixture.covariance), profile,
                "zero correction preserves covariance rather than clearing it");

    // Finite rank-one PSD covariance at the scalar limit must not overflow
    // during symmetry cleanup when G is identity.
    for (std::size_t row = 0U; row < size; ++row)
    {
        for (std::size_t column = 0U; column < size; ++column)
        {
            output.set(row, column, std::numeric_limits<value_type>::max());
        }
    }
    auto const large = output;
    Status const large_status = try_reset_covariance(zero_error, output, output);
    test.expect(large_status == Status::success && same_bits(output, large), profile,
                "symmetry cleanup does not overflow for a finite covariance at zero correction");

    auto invalid = fixture.error;
    invalid.delta_theta_b.set(1U, std::numeric_limits<value_type>::quiet_NaN());
    auto const saved = output;
    test.expect(try_reset_covariance(invalid, fixture.covariance, output) == Status::non_finite_input &&
                    same_bits(output, saved),
                profile, "standalone reset rejects invalid attitude error atomically");
}

template <typename Linalg, typename Configuration>
void test_atomic_injection(TestContext & test, std::string_view profile)
{
    using fixture_type = InjectionFixture<Linalg, Configuration>;
    fixture_type fixture;
    auto expected_state = fixture.state;
    auto expected_covariance = fixture.covariance;
    typename fixture_type::error_type const zero_error{};
    test.expect(try_inject_nominal(fixture.state, fixture.error, fixture.minimum_norm, expected_state) ==
                        Status::success &&
                    try_reset_covariance(fixture.error, fixture.covariance, expected_covariance) == Status::success,
                profile, "independent injection and reset steps succeed");
    for (unsigned mask = 0U; mask < 8U; ++mask)
    {
        auto input = fixture;
        typename fixture_type::state_type state_output{};
        typename fixture_type::covariance_type covariance_output{};
        auto error_output = fixture.error;
        auto & state = (mask & 1U) != 0U ? input.state : state_output;
        auto & covariance = (mask & 2U) != 0U ? input.covariance : covariance_output;
        auto & error = (mask & 4U) != 0U ? input.error : error_output;
        Status const status = try_inject_and_reset(input.state, input.covariance, input.error, fixture.minimum_norm,
                                                   state, covariance, error);
        test.expect(status == Status::success && same_state(state, expected_state) &&
                        same_bits(covariance, expected_covariance) && same_error(error, zero_error),
                    profile, "all eight alias combinations commit state, covariance and zero error mean");
        test.expect(((mask & 1U) != 0U || same_state(input.state, fixture.state)) &&
                        ((mask & 2U) != 0U || same_bits(input.covariance, fixture.covariance)) &&
                        ((mask & 4U) != 0U || same_error(input.error, fixture.error)),
                    profile, "non-aliased inputs remain unchanged");
    }
    auto const initial_state = fixture.state;
    auto const initial_covariance = fixture.covariance;
    auto error = zero_error;
    Status const status = try_inject_and_reset(fixture.state, fixture.covariance, error, fixture.minimum_norm,
                                               fixture.state, fixture.covariance, error);
    test.expect(status == Status::success && same_state(fixture.state, initial_state) &&
                    same_bits(fixture.covariance, initial_covariance) && same_error(error, zero_error),
                profile, "zero-error full step leaves the basis fixture unchanged");
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
    auto expect_failure = [&](fixture_type input, value_type minimum_norm, Status expected)
    {
        auto output = fixture;
        auto const saved = input;
        Status const separate_status = try_inject_and_reset(input.state, input.covariance, input.error, minimum_norm,
                                                            output.state, output.covariance, output.error);
        test.expect(separate_status == expected && same_state(output.state, fixture.state) &&
                        same_bits(output.covariance, fixture.covariance) && same_error(output.error, fixture.error),
                    profile, "failure preserves all separate output coefficients");
        Status const alias_status = try_inject_and_reset(input.state, input.covariance, input.error, minimum_norm,
                                                         input.state, input.covariance, input.error);
        test.expect(alias_status == expected && same_state(input.state, saved.state) &&
                        same_bits(input.covariance, saved.covariance) && same_error(input.error, saved.error),
                    profile, "failure preserves aliased inputs, including NaN bits and unconsumed error mean");
    };
    for (value_type const minimum_norm : {value_type{0}, value_type{-1}, value_type{1.25}})
    {
        expect_failure(fixture, minimum_norm, Status::domain_error);
    }
    expect_failure(fixture, nan, Status::non_finite_input);
    expect_failure(fixture, infinity, Status::non_finite_input);
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        auto bad = fixture;
        bad.error.delta_theta_b.set(axis, nan);
        expect_failure(bad, fixture.minimum_norm, Status::non_finite_input);
        bad.error.delta_theta_b.set(axis, infinity);
        expect_failure(bad, fixture.minimum_norm, Status::non_finite_input);
        bad.error.delta_theta_b.set(axis, maximum);
        expect_failure(bad, fixture.minimum_norm, Status::non_finite_result);
    }
    for (std::size_t row = 0U; row < fixture_type::size; ++row)
    {
        for (std::size_t column = 0U; column < fixture_type::size; ++column)
        {
            auto bad = fixture;
            bad.covariance.set(row, column, (row == column) ? infinity : nan);
            expect_failure(bad, fixture.minimum_norm, Status::non_finite_input);
        }
    }
    auto overflow = fixture;
    // A finite PSD rank-one input whose transformed entries exceed the scalar
    // range in BOTH reset modes (J_r need not amplify an isotropic covariance).
    for (std::size_t row = 0U; row < fixture_type::size; ++row)
    {
        for (std::size_t column = 0U; column < fixture_type::size; ++column)
        {
            overflow.covariance.set(row, column, maximum);
        }
    }
    expect_failure(overflow, fixture.minimum_norm, Status::non_finite_result);

    // Test standalone nominal rollback independently of the atomic wrapper.
    auto bad_error = fixture.error;
    bad_error.delta_theta_b.set(0U, maximum);
    auto output = fixture.state;
    Status const status = try_inject_nominal(output, bad_error, fixture.minimum_norm, output);
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
                (bad.state.*state_members[group]).set(axis, nan);
                expect_failure(bad, fixture.minimum_norm, Status::non_finite_input);
                bad = fixture;
                (bad.error.*error_members[group]).set(axis, infinity);
                expect_failure(bad, fixture.minimum_norm, Status::non_finite_input);
                bad = fixture;
                (bad.state.*state_members[group]).set(axis, maximum);
                (bad.error.*error_members[group]).set(axis, maximum);
                expect_failure(bad, fixture.minimum_norm, Status::non_finite_result);
                auto const saved = bad.state;
                Status const nominal_status = try_inject_nominal(bad.state, bad.error, fixture.minimum_norm, bad.state);
                test.expect(nominal_status == Status::non_finite_result && same_state(bad.state, saved), profile,
                            "late additive overflow does not partially inject attitude or other INS components");
            }
        }
    }
}

template <typename Linalg, typename Configuration>
void run_injection_tests(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    test_nominal_injection<Linalg, Configuration>(test, profile, tolerance);
    test_covariance_reset<Linalg, Configuration>(test, profile, tolerance);
    test_atomic_injection<Linalg, Configuration>(test, profile);
    test_injection_failures<Linalg, Configuration>(test, profile);
}

} /* end namespace */

int main()
{
    TestContext test;
    formal_eskf::test::configure_backend_test();
    using FloatLinalg = formal_eskf::test::Backend<float>;
    using DoubleLinalg = formal_eskf::test::Backend<double>;
    using formal_eskf::configuration::Ahrs;
    using formal_eskf::configuration::Ins;
    run_injection_tests<FloatLinalg, Ahrs>(test, "AHRS binary32", 3.0e-6F);
    run_injection_tests<FloatLinalg, Ins>(test, "INS binary32", 3.0e-6F);
    run_injection_tests<DoubleLinalg, Ahrs>(test, "AHRS binary64", 1.0e-12);
    run_injection_tests<DoubleLinalg, Ins>(test, "INS binary64", 1.0e-12);
    if (test.failures() != 0)
    {
        std::cerr << test.failures() << " ESKF injection/reset test(s) failed\n";
        return 1;
    }
    std::cout << "All ESKF injection/reset tests passed ("
              << (ESKF_RESET_APPROX ? "first-order reset" : "closed-form reset") << ")\n";
    return 0;
}
