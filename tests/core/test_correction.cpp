/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <numeric>
#include <string_view>
#include <type_traits>

// Exercise the standalone header, without relying on the ESKF master header.
#include <formal_eskf/eskf/correction.hpp>
#include <formal_eskf/eskf/types.hpp>
#include <formal_eskf/eskf/covariance_prediction.hpp>
#include "test_backend.hpp"

#include "test_support.hpp"
#include "test_cases.hpp"
#include "correction_reference.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::correct;
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

template <typename Linalg, typename Configuration, std::size_t MeasurementSize> struct CorrectionFixture
{
    using types = formal_eskf::EskfTypes<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    using correction_type = formal_eskf::linalg::Vector<Linalg, types::error_state_dimension>;
    using gain_type = formal_eskf::linalg::Matrix<Linalg, types::error_state_dimension, MeasurementSize>;
    using state_type = typename types::nominal_state_type;
    using covariance_type = typename types::error_covariance_type;
    static constexpr bool is_ins = std::is_same_v<Configuration, formal_eskf::configuration::Ins>;
    static constexpr std::size_t size = types::error_state_dimension;
    static constexpr std::size_t measurement_size = MeasurementSize;
    static constexpr std::size_t attitude_offset = is_ins ? 6U : 0U;
    static constexpr value_type minimum_norm = static_cast<value_type>(1.0e-6);

    state_type state{};
    covariance_type P{};
    formal_eskf::linalg::Matrix<Linalg, MeasurementSize, 1U> r{};
    formal_eskf::linalg::Matrix<Linalg, MeasurementSize, size> H{};
    formal_eskf::linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> V{};
    Status attitude_status{};

    CorrectionFixture()
    {
        attitude_status = formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(
            value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5}, minimum_norm, state.q_nb);
        if constexpr (is_ins)
        {
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                value_type const n = static_cast<value_type>(axis + 1U);
                state.p_n(axis) = n;
                state.v_n(axis) = -n;
                state.b_a(axis) = n / value_type{8};
                state.b_g(axis) = -n / value_type{16};
            }
        }
        // P = I + u*u^T; V = I/4 + w*w^T. Both are dense SPD matrices.
        for (std::size_t row = 0U; row < size; ++row)
        {
            for (std::size_t column = 0U; column < size; ++column)
            {
                value_type const u = static_cast<value_type>(row + 1U) / value_type{32};
                value_type const v = static_cast<value_type>(column + 1U) / value_type{32};
                P(row, column) = u * v + (row == column ? value_type{1} : value_type{0});
            }
        }
        for (std::size_t row = 0U; row < MeasurementSize; ++row)
        {
            r(row) = static_cast<value_type>(row + 1U) / value_type{128};
            for (std::size_t column = 0U; column < size; ++column)
            {
                auto const pattern = static_cast<int>((row * 3U + column * 5U) % 7U) - 3;
                H(row, column) = static_cast<value_type>(pattern) / value_type{16} +
                                 (column == attitude_offset + row % 3U ? value_type{1} : value_type{0});
            }
            for (std::size_t column = 0U; column < MeasurementSize; ++column)
            {
                value_type const u = static_cast<value_type>(row + 1U) / value_type{16};
                value_type const v = static_cast<value_type>(column + 1U) / value_type{16};
                V(row, column) = u * v + (row == column ? value_type{0.25} : value_type{0});
            }
        }
    }
};

using formal_eskf::test::reference::copied;
using formal_eskf::test::reference::matches_reference;
using formal_eskf::test::reference::product;
using formal_eskf::test::reference::reference_correction;
using formal_eskf::test::reference::transposed;

template <typename Fixture>
void check_success(TestContext & test, std::string_view profile, Fixture const & fixture,
                   typename Fixture::value_type tolerance)
{
    auto const reference = reference_correction(fixture, ESKF_RESET_APPROX != 0);
    test.expect(reference.valid && fixture.attitude_status == Status::success, profile,
                "valid correction fixture and oracle");
    typename Fixture::correction_type delta;
    typename Fixture::covariance_type joseph;
    typename Fixture::gain_type K;
    Status linear_status = formal_eskf::detail::compute_correction_gain(fixture.P, fixture.r, fixture.H, fixture.V,
                                                                        Fixture::minimum_norm, K);
    if (linear_status == Status::success)
    {
        linear_status =
            formal_eskf::detail::apply_correction_gain(fixture.P, fixture.r, fixture.H, fixture.V, K, delta, joseph);
    }
    bool linear_matches = linear_status == Status::success;
    for (std::size_t row = 0U; row < Fixture::size; ++row)
    {
        linear_matches =
            near(delta(row), static_cast<typename Fixture::value_type>(reference.delta_x[row][0U]), tolerance) &&
            linear_matches;
        for (std::size_t column = 0U; column < Fixture::size; ++column)
        {
            linear_matches =
                near(joseph(row, column),
                     static_cast<typename Fixture::value_type>(reference.joseph_covariance[row][column]), tolerance) &&
                linear_matches;
        }
    }
    test.expect(linear_matches, profile, "pre-injection error and full Joseph covariance match independent oracle");
    auto const saved_delta = delta;
    auto const saved_joseph = joseph;
    auto const saved_gain = K;
    Status rejected = formal_eskf::detail::compute_correction_gain(fixture.P, fixture.r, fixture.H, fixture.V,
                                                                   typename Fixture::value_type{0}, K);
    if (rejected == Status::success)
    {
        rejected =
            formal_eskf::detail::apply_correction_gain(fixture.P, fixture.r, fixture.H, fixture.V, K, delta, joseph);
    }
    test.expect(rejected == Status::domain_error && same_fields(K, saved_gain) && same_fields(delta, saved_delta) &&
                    same_fields(joseph, saved_joseph),
                profile, "invalid inputs preserve gain, correction and covariance scratch outputs");
    for (unsigned aliases = 0U; aliases < 4U; ++aliases)
    {
        auto input = fixture;
        typename Fixture::state_type separate_state;
        auto separate_covariance = Fixture::covariance_type::identity();
        auto & state_output = (aliases & 1U) != 0U ? input.state : separate_state;
        auto & covariance_output = (aliases & 2U) != 0U ? input.P : separate_covariance;
        Status const status = correct(input.state, input.P, input.r, input.H, input.V, Fixture::minimum_norm,
                                      state_output, covariance_output);
        test.expect(status == Status::success &&
                        matches_reference(fixture, reference, state_output, covariance_output, tolerance),
                    profile, "correction agrees with independent gain/Joseph/Exp/reset oracle for every output alias");
        bool const immutable = same_fields(input.r, fixture.r) && same_fields(input.H, fixture.H) &&
                               same_fields(input.V, fixture.V) &&
                               ((aliases & 1U) != 0U || same_state(input.state, fixture.state)) &&
                               ((aliases & 2U) != 0U || same_fields(input.P, fixture.P));
        test.expect(immutable, profile, "correction does not modify distinct inputs");
    }
}

template <typename Linalg, typename Configuration, std::size_t MeasurementSize>
void test_correction_success(TestContext & test, std::string_view profile)
{
    constexpr auto tolerance = static_cast<typename Linalg::value_type>(
        std::is_same_v<typename Linalg::value_type, float> ? 8.0e-6F : 2.0e-12);
    using fixture_type = CorrectionFixture<Linalg, Configuration, MeasurementSize>;
    using value_type = typename Linalg::value_type;
    fixture_type fixture;
    check_success(test, profile, fixture, tolerance);
    auto larger_residual = fixture;
    larger_residual.r = larger_residual.r * value_type{8};
    check_success(test, profile, larger_residual, tolerance);
    auto semidefinite_prior = fixture;
    for (std::size_t index = 0U; index < fixture_type::size; ++index)
    {
        semidefinite_prior.P(0U, index) = value_type{0};
        semidefinite_prior.P(index, 0U) = value_type{0};
    }
    check_success(test, profile, semidefinite_prior, tolerance);
    auto negative_quaternion = fixture;
    negative_quaternion.state.q_nb = -negative_quaternion.state.q_nb;
    check_success(test, profile, negative_quaternion, tolerance);
}

template <typename Linalg, typename Configuration>
void test_repeated_scalar_correction(TestContext & test, std::string_view profile)
{
    constexpr auto tolerance = static_cast<typename Linalg::value_type>(
        std::is_same_v<typename Linalg::value_type, float> ? 8.0e-6F : 2.0e-12);
    using value_type = typename Linalg::value_type;
    using fixture_type = CorrectionFixture<Linalg, Configuration, 1U>;
    fixture_type fixture;
    fixture.P = fixture_type::covariance_type::identity();
    fixture.H = {};
    fixture.H(0U, fixture_type::attitude_offset) = value_type{1};
    fixture.V(0U, 0U) = value_type{0.25};
    fixture.r = {};
    auto const prior_state = fixture.state;
    // With zero residual the reset is identity. Repeated scalar updates have
    // the independent closed form p_k = 1 / (1 + k / v).
    for (std::size_t count = 1U; count <= 100U; ++count)
    {
        Status const status = correct(fixture.state, fixture.P, fixture.r, fixture.H, fixture.V,
                                      fixture_type::minimum_norm, fixture.state, fixture.P);
        value_type const expected = value_type{1} / (value_type{1} + value_type{4} * static_cast<value_type>(count));
        bool correct = status == Status::success && same_state(fixture.state, prior_state);
        for (std::size_t row = 0U; row < fixture_type::size; ++row)
        {
            for (std::size_t column = 0U; column < fixture_type::size; ++column)
            {
                value_type const diagonal = row == fixture_type::attitude_offset ? expected : value_type{1};
                correct = correct && near(fixture.P(row, column), row == column ? diagonal : value_type{0}, tolerance);
            }
        }
        test.expect(correct, profile,
                    "100 scalar corrections match analytic variance and do not drift at zero residual");
    }
}

template <typename State, typename Matrix> void set_failure_sentinels(State & state, Matrix & covariance)
{
    using value_type = typename Matrix::value_type;
    value_type const nan = std::numeric_limits<value_type>::quiet_NaN();
    value_type const infinity = std::numeric_limits<value_type>::infinity();
    state.q_nb = -state.q_nb;
    covariance(0U, 0U) = nan;
    covariance(1U, 2U) = -value_type{0};
    covariance(2U, 2U) = infinity;
    if constexpr (requires { state.p_n; })
    {
        state.p_n(0U) = nan;
        state.v_n(1U) = infinity;
        state.b_a(2U) = -value_type{0};
    }
}

template <typename Fixture>
void check_failure(TestContext & test, std::string_view profile, Fixture const & bad,
                   typename Fixture::value_type minimum_norm, Status expected, std::string_view message)
{
    for (unsigned aliases = 0U; aliases < 4U; ++aliases)
    {
        auto input = bad;
        typename Fixture::state_type separate_state;
        auto separate_covariance = Fixture::covariance_type::identity();
        set_failure_sentinels(separate_state, separate_covariance);
        auto const saved_state = separate_state;
        auto const saved_covariance = separate_covariance;
        auto & state_output = (aliases & 1U) != 0U ? input.state : separate_state;
        auto & covariance_output = (aliases & 2U) != 0U ? input.P : separate_covariance;
        Status const status =
            correct(input.state, input.P, input.r, input.H, input.V, minimum_norm, state_output, covariance_output);
        bool const unchanged = same_state(input.state, bad.state) && same_fields(input.P, bad.P) &&
                               same_fields(input.r, bad.r) && same_fields(input.H, bad.H) &&
                               same_fields(input.V, bad.V) && same_state(separate_state, saved_state) &&
                               same_fields(separate_covariance, saved_covariance);
        test.expect(status == expected && unchanged, profile, message);
    }
}

template <typename Fixture>
void check_late_stages(TestContext & test, std::string_view profile, Fixture const & fixture, Status expected_injection)
{
    typename Fixture::gain_type gain;
    typename Fixture::correction_type delta;
    typename Fixture::covariance_type joseph;
    Status status = formal_eskf::detail::compute_correction_gain(fixture.P, fixture.r, fixture.H, fixture.V,
                                                                 Fixture::minimum_norm, gain);
    if (status == Status::success)
    {
        status =
            formal_eskf::detail::apply_correction_gain(fixture.P, fixture.r, fixture.H, fixture.V, gain, delta, joseph);
    }
    test.expect(status == Status::success, profile, "gain and Joseph stages succeed before the late failure");
    if (status != Status::success)
    {
        return;
    }
    typename Fixture::types::error_state_type error;
    formal_eskf::detail::unpack_correction(delta, error);
    auto state = fixture.state;
    status = formal_eskf::inject_nominal(fixture.state, error, Fixture::minimum_norm, state);
    test.expect(status == expected_injection, profile, "late failure occurs at the specified injection/reset stage");
    if (status == Status::success)
    {
        auto covariance = joseph;
        test.expect(!same_state(state, fixture.state) &&
                        formal_eskf::reset_covariance(error, joseph, covariance) == Status::non_finite_result,
                    profile, "reset fails only after a successful changing nominal injection");
    }
}

template <typename Linalg, typename Configuration>
void test_joseph_cancellation(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using fixture_type = CorrectionFixture<Linalg, Configuration, 1U>;
    fixture_type fixture;
    fixture.P = fixture_type::covariance_type::identity();
    fixture.H = {};
    fixture.H(0U, fixture_type::attitude_offset) = value_type{1};
    fixture.r = {};
    value_type const epsilon = std::numeric_limits<value_type>::epsilon();
    value_type const variance = epsilon / value_type{4};
    fixture.V(0U, 0U) = variance;
    // S rounds to 1 and K to 1. The simplified (I-KH)P would lose the
    // remaining variance entirely; Joseph retains it through K V K^T.
    Status const status = correct(fixture.state, fixture.P, fixture.r, fixture.H, fixture.V, fixture_type::minimum_norm,
                                  fixture.state, fixture.P);
    value_type const posterior = fixture.P(fixture_type::attitude_offset, fixture_type::attitude_offset);
    long double const v = static_cast<long double>(variance);
    value_type const expected = static_cast<value_type>(v / (1.0L + v));
    test.expect(status == Status::success && posterior > value_type{0} &&
                    near(posterior, expected, variance * epsilon * value_type{4}),
                profile, "Joseph retains positive measurement variance when I-KH cancels to zero");
}

template <typename Fixture>
void check_preparation_failure(TestContext & test, std::string_view profile, Fixture const & bad,
                               typename Fixture::value_type minimum_norm, Status expected);

template <typename Linalg, typename Configuration> void test_matrix_domain(TestContext & test, std::string_view profile)
{
    using fixture_type = CorrectionFixture<Linalg, Configuration, 2U>;
    using value_type = typename Linalg::value_type;
    fixture_type fixture;
    value_type const nan = std::numeric_limits<value_type>::quiet_NaN();
    value_type const infinity = std::numeric_limits<value_type>::infinity();
    auto fail = [&](fixture_type const & bad, Status expected, std::string_view message)
    {
        check_failure(test, profile, bad, fixture_type::minimum_norm, expected, message);
        check_preparation_failure(test, profile, bad, fixture_type::minimum_norm, expected);
    };
    for (auto const invalid : {value_type{0}, value_type{-1}, value_type{2}, nan, infinity})
    {
        check_failure(test, profile, fixture, invalid,
                      std::isfinite(invalid) ? Status::domain_error : Status::non_finite_input,
                      "invalid norm threshold preserves both outputs");
    }
    auto non_finite_entries = [&](auto member)
    {
        using matrix_type = std::remove_cvref_t<decltype(fixture.*member)>;
        for (std::size_t row = 0U; row < matrix_type::row_count; ++row)
        {
            for (std::size_t column = 0U; column < matrix_type::column_count; ++column)
            {
                for (auto const invalid : {nan, infinity, -infinity})
                {
                    auto bad = fixture;
                    (bad.*member)(row, column) = invalid;
                    fail(bad, Status::non_finite_input, "every non-finite P/r/H/V coefficient is rejected atomically");
                }
            }
        }
    };
    non_finite_entries(&fixture_type::P);
    non_finite_entries(&fixture_type::r);
    non_finite_entries(&fixture_type::H);
    non_finite_entries(&fixture_type::V);
    auto bad = fixture;
    bad.P(0U, 1U) = std::nextafter(bad.P(0U, 1U), infinity);
    fail(bad, Status::domain_error, "one-ULP asymmetric prior is rejected, not silently symmetrized");
    bad = fixture;
    bad.V(0U, 1U) = std::nextafter(bad.V(0U, 1U), infinity);
    fail(bad, Status::domain_error, "one-ULP asymmetric measurement covariance is rejected");
    bad = fixture;
    bad.P(0U, 0U) = value_type{-1};
    fail(bad, Status::domain_error, "negative prior variance is rejected");
    for (auto const invalid : {value_type{0}, value_type{-1}})
    {
        bad = fixture;
        bad.V(0U, 0U) = invalid;
        fail(bad, Status::domain_error, "nonpositive measurement variance is rejected");
    }
    for (std::size_t axis = 0U; axis < fixture_type::size; ++axis)
    {
        bad = fixture;
        bad.P(axis, axis) = -std::numeric_limits<value_type>::denorm_min();
        fail(bad, Status::domain_error, "every negative prior diagonal, including smallest negative, is rejected");
    }
    for (std::size_t axis = 0U; axis < fixture_type::measurement_size; ++axis)
    {
        for (value_type const invalid : {value_type{0}, -value_type{0}, value_type{-1}})
        {
            bad = fixture;
            bad.V(axis, axis) = invalid;
            fail(bad, Status::domain_error, "every nonpositive measurement diagonal is rejected");
        }
    }
    bad = fixture;
    bad.P(fixture_type::size - 1U, 0U) = std::nextafter(bad.P(fixture_type::size - 1U, 0U), infinity);
    fail(bad, Status::domain_error, "symmetry validation includes the full prior beyond the leading block");
    bad = fixture;
    bad.P(0U, 0U) = value_type{-1};
    bad.H(0U, 0U) = nan;
    fail(bad, Status::non_finite_input, "all finite-input checks precede domain checks");
}

template <typename Linalg, typename Configuration>
void test_arithmetic_boundaries(TestContext & test, std::string_view profile)
{
    using fixture_type = CorrectionFixture<Linalg, Configuration, 2U>;
    using value_type = typename Linalg::value_type;
    fixture_type fixture;
    value_type const nan = std::numeric_limits<value_type>::quiet_NaN();
    value_type const maximum = std::numeric_limits<value_type>::max();
    auto fail = [&](fixture_type const & bad, Status expected, std::string_view message)
    { check_failure(test, profile, bad, fixture_type::minimum_norm, expected, message); };
    auto bad = fixture;
    // All remaining matrix inputs satisfy P PSD and V SPD; failures are numerical.
    fixture.P = fixture_type::covariance_type::identity();
    fixture.H = {};
    fixture.V = decltype(fixture.V)::identity();
    fixture.r = {};
    bad = fixture;
    bad.P(0U, 0U) = maximum;
    bad.H(0U, 0U) = value_type{2};
    fail(bad, Status::non_finite_result, "P H^T overflow preserves outputs");
    check_preparation_failure(test, profile, bad, fixture.minimum_norm, Status::non_finite_result);
    bad = fixture;
    bad.P(0U, 0U) = maximum / value_type{2};
    bad.H(0U, 0U) = value_type{1.5};
    fail(bad, Status::non_finite_result, "H P H^T overflow preserves outputs");
    check_preparation_failure(test, profile, bad, fixture.minimum_norm, Status::non_finite_result);
    bad = fixture;
    bad.P(0U, 0U) = maximum;
    bad.H(0U, 0U) = value_type{1};
    bad.V(0U, 0U) = maximum;
    fail(bad, Status::non_finite_result, "innovation addition overflow preserves outputs");
    check_preparation_failure(test, profile, bad, fixture.minimum_norm, Status::non_finite_result);
    bad = fixture;
    bad.H(0U, fixture_type::attitude_offset) = value_type{0.25};
    bad.V(0U, 0U) = value_type{0.0625};
    bad.r(0U) = maximum; // K = 2, so delta_x itself overflows.
    fail(bad, Status::non_finite_result, "gain-residual overflow preserves outputs");
    bad = fixture;
    bad.H(0U, fixture_type::attitude_offset) = value_type{1};
    bad.r(0U) = maximum; // Finite delta_x, but Exp's squared norm overflows.
    check_late_stages(test, profile, bad, Status::non_finite_result);
    fail(bad, Status::non_finite_result, "late injection failure does not commit the Joseph covariance");

    bad = fixture;
    constexpr std::size_t offset = fixture_type::attitude_offset;
    value_type const large_variance = maximum * value_type{0.75};
    // The xy block is rank-one PSD. Correction observes only z, leaving this
    // block intact until reset. Jr(pi/2 * e_z) concentrates its variance into x
    // beyond the scalar range (the first-order reset overflows too).
    bad.P(offset, offset) = large_variance;
    bad.P(offset, offset + 1U) = large_variance;
    bad.P(offset + 1U, offset) = large_variance;
    bad.P(offset + 1U, offset + 1U) = large_variance;
    bad.H(0U, offset + 2U) = value_type{1};
    bad.r(0U) = static_cast<value_type>(std::acos(-1.0L));
    check_late_stages(test, profile, bad, Status::success);
    fail(bad, Status::non_finite_result, "reset overflow rolls back the already-computed nominal injection");

    if constexpr (fixture_type::is_ins)
    {
        using state_type = typename fixture_type::state_type;
        auto const members = std::array{&state_type::p_n, &state_type::v_n, &state_type::b_a, &state_type::b_g};
        constexpr std::array<std::size_t, 4U> offsets{0U, 3U, 9U, 12U};
        for (std::size_t group = 0U; group < members.size(); ++group)
        {
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                bad = fixture;
                (bad.state.*members[group])(axis) = nan;
                fail(bad, Status::non_finite_input, "non-finite INS nominal fields preserve outputs");
                bad = fixture;
                (bad.state.*members[group])(axis) = maximum;
                bad.H(0U, offsets[group] + axis) = value_type{0.25};
                bad.V(0U, 0U) = value_type{0.0625};
                bad.r(0U) = maximum / value_type{4};
                check_late_stages(test, profile, bad, Status::non_finite_result);
                fail(bad, Status::non_finite_result, "late additive INS injection overflow preserves both outputs");
            }
        }
    }
    enum class InternalOverflow
    {
        gain_product,
        prior_contribution,
        noise_contribution,
        contribution_sum
    };
    for (auto const stage : {InternalOverflow::gain_product, InternalOverflow::prior_contribution,
                             InternalOverflow::noise_contribution, InternalOverflow::contribution_sum})
    {
        auto input = fixture;
        typename fixture_type::gain_type gain;
        gain(0U, 0U) = value_type{1};
        switch (stage)
        {
        case InternalOverflow::gain_product:
            gain(0U, 0U) = maximum;
            input.H(0U, 0U) = value_type{2};
            break;
        case InternalOverflow::prior_contribution:
            input.P(0U, 0U) = maximum;
            input.H(0U, 0U) = value_type{-1};
            break;
        case InternalOverflow::noise_contribution:
            gain(0U, 0U) = value_type{2};
            input.V(0U, 0U) = maximum;
            break;
        case InternalOverflow::contribution_sum:
            input.P(0U, 0U) = maximum * value_type{0.75};
            input.V(0U, 0U) = maximum * value_type{0.75};
            break;
        }
        // These finite gains are internal boundary probes, not assertions that
        // a valid unconstrained public solve can produce every such K.
        typename fixture_type::correction_type delta;
        delta(0U) = nan;
        auto covariance = fixture.P;
        covariance(0U, 0U) = -value_type{0};
        auto const saved_delta = delta;
        auto const saved_covariance = covariance;
        test.expect(formal_eskf::detail::apply_correction_gain(input.P, input.r, input.H, input.V, gain, delta,
                                                               covariance) == Status::non_finite_result &&
                        same_fields(delta, saved_delta) && same_fields(covariance, saved_covariance),
                    profile, "internal finite-gain arithmetic boundary preserves both scratch outputs");
    }
}

template <typename Linalg, std::size_t MeasurementSize>
void test_bias_update(TestContext & test, std::string_view profile)
{
    constexpr auto tolerance = static_cast<typename Linalg::value_type>(
        std::is_same_v<typename Linalg::value_type, float> ? 8.0e-6F : 2.0e-12);
    using Ins = formal_eskf::configuration::Ins;
    using fixture_type = CorrectionFixture<Linalg, Ins, MeasurementSize>;
    fixture_type fixture;
    typename fixture_type::state_type unrestricted_state;
    typename fixture_type::covariance_type unrestricted_covariance;
    test.expect(correct(fixture.state, fixture.P, fixture.r, fixture.H, fixture.V, fixture_type::minimum_norm,
                        unrestricted_state, unrestricted_covariance) == Status::success,
                profile, "unrestricted bias-update baseline");
    typename fixture_type::state_type default_state;
    typename fixture_type::covariance_type default_covariance;
    test.expect(correct(fixture.state, fixture.P, fixture.r, fixture.H, fixture.V, fixture_type::minimum_norm,
                        default_state, default_covariance, Ins::BiasUpdate{}) == Status::success &&
                    same_state(default_state, unrestricted_state) &&
                    same_fields(default_covariance, unrestricted_covariance),
                profile, "explicit default bias permissions equal the omitted argument");

    // Every combination of six body-axis permissions, with dense P and correlated V.
    for (unsigned permissions = 0U; permissions < 64U; ++permissions)
    {
        Ins::BiasUpdate bias_update;
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            bias_update.accelerometer[axis] = (permissions & (1U << axis)) != 0U;
            bias_update.gyroscope[axis] = (permissions & (1U << (3U + axis))) != 0U;
        }
        auto const reference = reference_correction(fixture, ESKF_RESET_APPROX != 0, bias_update);
        for (unsigned aliases = 0U; aliases < 4U; ++aliases)
        {
            auto input = fixture;
            typename fixture_type::state_type separate_state;
            typename fixture_type::covariance_type separate_covariance;
            auto & state_output = (aliases & 1U) != 0U ? input.state : separate_state;
            auto & covariance_output = (aliases & 2U) != 0U ? input.P : separate_covariance;
            Status const status = correct(input.state, input.P, input.r, input.H, input.V, fixture_type::minimum_norm,
                                          state_output, covariance_output, bias_update);
            test.expect(status == Status::success &&
                            matches_reference(fixture, reference, state_output, covariance_output, tolerance),
                        profile, "all bias permissions and aliases match independent constrained-gain/Joseph oracle");
            bool preserved = true;
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                if (!bias_update.accelerometer[axis])
                {
                    preserved = preserved && state_output.b_a(axis) == fixture.state.b_a(axis) &&
                                covariance_output(9U + axis, 9U + axis) == fixture.P(9U + axis, 9U + axis);
                }
                if (!bias_update.gyroscope[axis])
                {
                    preserved = preserved && state_output.b_g(axis) == fixture.state.b_g(axis) &&
                                covariance_output(12U + axis, 12U + axis) == fixture.P(12U + axis, 12U + axis);
                }
            }
            test.expect(preserved, profile, "inhibited biases retain their estimates and variances, not zero them");
            bool const immutable = same_fields(input.r, fixture.r) && same_fields(input.H, fixture.H) &&
                                   same_fields(input.V, fixture.V) &&
                                   ((aliases & 1U) != 0U || same_state(input.state, fixture.state)) &&
                                   ((aliases & 2U) != 0U || same_fields(input.P, fixture.P));
            test.expect(immutable, profile, "controlled correction preserves distinct inputs");
            if (permissions == 63U)
            {
                test.expect(same_state(state_output, unrestricted_state) &&
                                same_fields(covariance_output, unrestricted_covariance),
                            profile, "all-enabled bias control is bitwise equivalent to unrestricted correction");
            }
        }
    }
    if constexpr (MeasurementSize == 1U)
    {
        using value_type = typename fixture_type::value_type;
        fixture_type exact;
        exact.P = fixture_type::covariance_type::identity();
        for (std::size_t row = 0U; row < fixture_type::size; ++row)
        {
            value_type const u = row == 0U || row >= 9U ? value_type{1} : value_type{0};
            for (std::size_t column = 0U; column < fixture_type::size; ++column)
            {
                value_type const v = column == 0U || column >= 9U ? value_type{1} : value_type{0};
                exact.P(row, column) += u * v;
            }
        }
        exact.H = {};
        exact.H(0U, 0U) = value_type{1};
        exact.V(0U, 0U) = value_type{2};
        exact.r(0U) = value_type{4};
        for (unsigned mask = 0U; mask < 64U; ++mask)
        {
            Ins::BiasUpdate permission;
            std::array<value_type, fixture_type::size> gain{};
            gain[0U] = value_type{0.5};
            auto expected_state = exact.state;
            expected_state.p_n(0U) += value_type{2};
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                permission.accelerometer[axis] = (mask & (1U << axis)) != 0U;
                permission.gyroscope[axis] = (mask & (1U << (3U + axis))) != 0U;
                gain[9U + axis] = permission.accelerometer[axis] ? value_type{0.25} : value_type{0};
                gain[12U + axis] = permission.gyroscope[axis] ? value_type{0.25} : value_type{0};
                expected_state.b_a(axis) += value_type{4} * gain[9U + axis];
                expected_state.b_g(axis) += value_type{4} * gain[12U + axis];
            }
            auto expected_covariance = exact.P;
            // Independent expanded Joseph identity for ANY effective gain:
            // P - K C^T - C K^T + K S K^T, C=P[:,0], S=4.
            for (std::size_t row = 0U; row < fixture_type::size; ++row)
            {
                for (std::size_t column = 0U; column < fixture_type::size; ++column)
                {
                    expected_covariance(row, column) -= gain[row] * exact.P(column, 0U) +
                                                        exact.P(row, 0U) * gain[column] -
                                                        value_type{4} * gain[row] * gain[column];
                }
            }
            auto state = exact.state;
            auto covariance = exact.P;
            test.expect(correct(exact.state, exact.P, exact.r, exact.H, exact.V, exact.minimum_norm, state, covariance,
                                permission) == Status::success &&
                            same_state(state, expected_state) && same_fields(covariance, expected_covariance),
                        profile, "all 64 masks satisfy exact effective-gain state and expanded Joseph equations");
        }
    }
}

template <typename Linalg> void test_bias_update_failures(TestContext & test, std::string_view profile)
{
    using Ins = formal_eskf::configuration::Ins;
    using fixture_type = CorrectionFixture<Linalg, Ins, 2U>;
    using value_type = typename Linalg::value_type;
    fixture_type fixture;
    Ins::BiasUpdate const bias_update{.accelerometer = {false, false, false}, .gyroscope = {false, false, false}};
    auto fail = [&](fixture_type const & bad, Status expected)
    {
        for (unsigned aliases = 0U; aliases < 4U; ++aliases)
        {
            auto input = bad;
            auto separate_state = fixture.state;
            auto separate_covariance = fixture.P;
            set_failure_sentinels(separate_state, separate_covariance);
            auto const saved_state = separate_state;
            auto const saved_covariance = separate_covariance;
            auto & state_output = (aliases & 1U) != 0U ? input.state : separate_state;
            auto & covariance_output = (aliases & 2U) != 0U ? input.P : separate_covariance;
            Status const status = correct(input.state, input.P, input.r, input.H, input.V, fixture_type::minimum_norm,
                                          state_output, covariance_output, bias_update);
            test.expect(status == expected && same_state(input.state, bad.state) && same_fields(input.P, bad.P) &&
                            same_fields(input.r, bad.r) && same_fields(input.H, bad.H) && same_fields(input.V, bad.V) &&
                            same_state(separate_state, saved_state) &&
                            same_fields(separate_covariance, saved_covariance),
                        profile, "controlled correction rejects atomically for every output alias");
        }
    };
    auto bad = fixture;
    bad.H(0U, 9U) = std::numeric_limits<value_type>::quiet_NaN();
    fail(bad, Status::non_finite_input); // Disabled row must not conceal invalid inputs.
    bad = fixture;
    bad.H = {};
    bad.V(0U, 1U) = value_type{2};
    bad.V(1U, 0U) = value_type{2};
    fail(bad, Status::not_positive_definite);
    bad = fixture;
    bad.state.b_a(0U) = std::numeric_limits<value_type>::infinity();
    fail(bad, Status::non_finite_input); // Even an inhibited nominal bias must be valid.
    bad = fixture;
    bad.P = fixture_type::covariance_type::identity();
    bad.H = {};
    bad.H(0U, 6U) = value_type{1};
    bad.V = decltype(bad.V)::identity();
    bad.r(0U) = std::numeric_limits<value_type>::max();
    fail(bad, Status::non_finite_result); // Late injection failure cannot publish Joseph covariance.
    fixture.P = fixture_type::covariance_type::identity();
    fixture.P(0U, 9U) = value_type{0.5};
    fixture.P(9U, 0U) = value_type{0.5};
    fixture.H = {};
    fixture.H(0U, 0U) = value_type{1};
    fixture.V = decltype(fixture.V)::identity() * value_type{3};
    fixture.r = {};
    fixture.r(0U) = value_type{4};
    auto disabled_state = fixture.state;
    auto disabled_covariance = fixture.P;
    test.expect(correct(fixture.state, fixture.P, fixture.r, fixture.H, fixture.V, fixture.minimum_norm, disabled_state,
                        disabled_covariance, bias_update) == Status::success &&
                    same_fields(disabled_state.b_a, fixture.state.b_a) &&
                    same_fields(disabled_state.b_g, fixture.state.b_g) &&
                    disabled_covariance(9U, 9U) == value_type{1} && disabled_covariance(0U, 9U) == value_type{0.375},
                profile, "all-disabled correction still computes full covariance and changes bias cross-covariance");
    auto enabled_state = fixture.state;
    auto enabled_covariance = fixture.P;
    test.expect(correct(fixture.state, fixture.P, fixture.r, fixture.H, fixture.V, fixture.minimum_norm, enabled_state,
                        enabled_covariance) == Status::success &&
                    enabled_state.b_a(0U) == fixture.state.b_a(0U) + value_type{0.5},
                profile, "next unrestricted call learns bias; permissions do not persist");
    typename fixture_type::types::parameter_type parameters;
    parameters.dt_min = value_type{0.125};
    parameters.dt_max = value_type{1};
    parameters.minimum_quaternion_norm = fixture.minimum_norm;
    parameters.quaternion_squared_norm_tolerance = value_type{0.125};
    typename fixture_type::types::imu_sample_type imu;
    imu.angular_rate_b = disabled_state.b_g;
    imu.specific_force_b = disabled_state.b_a;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        parameters.process_noise.accelerometer_bias_random_walk_variance_density(axis) = value_type{0.5};
        parameters.process_noise.gyroscope_bias_random_walk_variance_density(axis) = value_type{1};
    }
    auto predicted = disabled_covariance;
    test.expect(formal_eskf::predict_covariance(disabled_state, disabled_covariance, imu, value_type{0.5}, parameters,
                                                predicted) == Status::success,
                profile, "prediction remains available after all-disabled bias correction");
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        test.expect(predicted(9U + axis, 9U + axis) == disabled_covariance(9U + axis, 9U + axis) + value_type{0.25} &&
                        predicted(12U + axis, 12U + axis) ==
                            disabled_covariance(12U + axis, 12U + axis) + value_type{0.5},
                    profile, "bias random-walk covariance grows independently of previous correction permissions");
    }
}

template <typename Linalg, typename Configuration, std::size_t MeasurementSize>
void test_preparation(TestContext & test, std::string_view profile)
{
    using fixture_type = CorrectionFixture<Linalg, Configuration, MeasurementSize>;
    using value_type = typename fixture_type::value_type;
    fixture_type fixture;
    for (unsigned partition = 0U; partition < 3U; ++partition)
    {
        auto input = fixture;
        if (partition == 1U)
        {
            input.H = {};
        }
        if (partition == 2U)
        {
            input.P = {};
        }
        auto const expected_cross = product(copied(input.P), transposed(copied(input.H)));
        auto expected_innovation = product(copied(input.H), expected_cross);
        typename fixture_type::gain_type cross;
        auto innovation = input.V;
        for (std::size_t row = 0U; row < MeasurementSize; ++row)
        {
            for (std::size_t column = 0U; column < MeasurementSize; ++column)
            {
                expected_innovation[row][column] += static_cast<long double>(input.V(row, column));
            }
        }
        auto const before = input;
        test.expect(formal_eskf::correction_covariances(input.P, input.r, input.H, input.V, fixture.minimum_norm, cross,
                                                        innovation) == Status::success,
                    profile, "preparation accepts PSD prior without nominal state or solve");
        // The fixture's polynomial inputs are dyadic and these small products
        // have representable sums; independent long-double scalar loops are
        // exact here. No generic covariance-error tolerance is assumed.
        for (std::size_t row = 0U; row < fixture_type::size; ++row)
        {
            for (std::size_t column = 0U; column < MeasurementSize; ++column)
            {
                test.expect(static_cast<long double>(cross(row, column)) == expected_cross[row][column], profile,
                            "every cross-covariance coefficient equals the independent dyadic product");
            }
        }
        for (std::size_t row = 0U; row < MeasurementSize; ++row)
        {
            for (std::size_t column = 0U; column < MeasurementSize; ++column)
            {
                test.expect(static_cast<long double>(innovation(row, column)) == expected_innovation[row][column],
                            profile, "innovation retains all measurement correlations exactly");
            }
        }
        test.expect(same_fields(input.P, before.P) && same_fields(input.H, before.H) &&
                        same_fields(input.V, before.V) && same_fields(input.r, before.r),
                    profile, "preparation preserves all borrowed matrix inputs");
    }
    // It computes finite products even when an otherwise checkable input would
    // not factorize; successful preparation is not permission to correct.
    if constexpr (MeasurementSize > 1U)
    {
        fixture.H = {};
        fixture.V(0U, 1U) = value_type{2};
        fixture.V(1U, 0U) = value_type{2};
        typename fixture_type::gain_type cross;
        auto innovation = fixture.V;
        test.expect(formal_eskf::correction_covariances(fixture.P, fixture.r, fixture.H, fixture.V,
                                                        fixture.minimum_norm, cross, innovation) == Status::success &&
                        same_fields(innovation, fixture.V),
                    profile, "preparation does not add an undocumented factorization requirement");
    }
}

template <typename Linalg, typename Configuration> void test_right_solve(TestContext & test, std::string_view profile)
{
    using fixture_type = CorrectionFixture<Linalg, Configuration, 2U>;
    using value_type = typename fixture_type::value_type;
    fixture_type fixture;
    fixture.P = fixture_type::covariance_type::identity();
    fixture.P(0U, 0U) = value_type{2};
    fixture.P(1U, 1U) = value_type{2};
    fixture.P(0U, 1U) = value_type{1};
    fixture.P(1U, 0U) = value_type{1};
    fixture.H = {};
    fixture.H(0U, 0U) = value_type{1};
    fixture.H(1U, 1U) = value_type{1};
    fixture.V(0U, 0U) = value_type{2};
    fixture.V(0U, 1U) = value_type{1};
    fixture.V(1U, 0U) = value_type{1};
    fixture.V(1U, 1U) = value_type{3};
    fixture.r = {};
    typename fixture_type::gain_type gain;
    test.expect(formal_eskf::detail::compute_correction_gain(fixture.P, fixture.r, fixture.H, fixture.V,
                                                             fixture.minimum_norm, gain) == Status::success,
                profile, "correlated exact Cholesky system is solvable");
    // S=[[4,2],[2,5]], L=[[2,0],[1,2]], K=[[8,0],[1,6]]/16.
    // Every multiply/divide of this fixture is representable, including LLT.
    for (std::size_t row = 0U; row < fixture_type::size; ++row)
    {
        value_type const expected0 = row == 0U ? value_type{0.5} : (row == 1U ? value_type{0.0625} : value_type{0});
        value_type const expected1 = row == 1U ? value_type{0.375} : value_type{0};
        test.expect(gain(row, 0U) == expected0 && gain(row, 1U) == expected1, profile,
                    "nonsymmetric rectangular gain equals independent rational right solve");
        test.expect(value_type{4} * gain(row, 0U) + value_type{2} * gain(row, 1U) == fixture.P(row, 0U) &&
                        value_type{2} * gain(row, 0U) + value_type{5} * gain(row, 1U) == fixture.P(row, 1U),
                    profile, "independent exact K S = C residual checks solve orientation");
    }
    auto expected_covariance = fixture.P;
    expected_covariance(0U, 0U) = value_type{1};
    expected_covariance(0U, 1U) = value_type{0.5};
    expected_covariance(1U, 0U) = value_type{0.5};
    expected_covariance(1U, 1U) = value_type{1.1875};
    auto state = fixture.state;
    auto covariance = fixture.P;
    test.expect(correct(fixture.state, fixture.P, fixture.r, fixture.H, fixture.V, fixture.minimum_norm, state,
                        covariance) == Status::success &&
                    same_state(state, fixture.state) && same_fields(covariance, expected_covariance),
                profile, "public correction uses the exact right solve and retains correlated Joseph posterior");
    fixture.H = {};
    fixture.V(0U, 1U) = value_type{4};
    fixture.V(1U, 0U) = value_type{4};
    check_failure(test, profile, fixture, fixture.minimum_norm, Status::not_positive_definite,
                  "outside-premise indefinite innovation propagates checked LLT failure without inverse fallback");
}

template <typename Linalg, typename Configuration>
void test_zero_information(TestContext & test, std::string_view profile)
{
    using fixture_type = CorrectionFixture<Linalg, Configuration, 1U>;
    using value_type = typename fixture_type::value_type;
    fixture_type fixture;
    fixture.P = fixture_type::covariance_type::identity();
    fixture.V(0U, 0U) = value_type{3};
    fixture.H = {};
    fixture.r(0U) = value_type{4};
    for (unsigned partition = 0U; partition < 4U; ++partition)
    {
        auto input = fixture;
        auto expected = input.P;
        if (partition == 1U)
        {
            input.P = {};
            expected = {};
            input.H(0U, 0U) = value_type{1};
        }
        if (partition == 2U)
        {
            input.H(0U, 0U) = value_type{1};
            input.r = {};
            expected(0U, 0U) = value_type{0.75};
        }
        if (partition == 3U)
        {
            input.P(0U, 0U) = std::numeric_limits<value_type>::max();
            expected = input.P;
        }
        auto state = input.state;
        auto covariance = input.P;
        test.expect(correct(input.state, input.P, input.r, input.H, input.V, input.minimum_norm, state, covariance) ==
                            Status::success &&
                        same_state(state, input.state) && same_fields(covariance, expected),
                    profile, "zero H/P preserves prior; zero residual still reduces informative variance exactly");
    }
    test_repeated_scalar_correction<Linalg, Configuration>(test, profile);
}

template <typename Linalg, typename Configuration, std::size_t MeasurementSize>
void test_joseph_prior(TestContext & test, std::string_view profile)
{
    test_correction_success<Linalg, Configuration, MeasurementSize>(test, profile);
    if constexpr (MeasurementSize == 1U)
    {
        test_joseph_cancellation<Linalg, Configuration>(test, profile);
    }
}

template <typename Fixture>
void check_preparation_failure(TestContext & test, std::string_view profile, Fixture const & bad,
                               typename Fixture::value_type minimum_norm, Status expected)
{
    using value_type = typename Fixture::value_type;
    typename Fixture::gain_type cross;
    auto innovation = bad.V;
    cross(0U, 0U) = std::numeric_limits<value_type>::quiet_NaN();
    cross(1U, 0U) = -value_type{0};
    innovation(0U, 0U) = -value_type{0};
    innovation(0U, 1U) = std::numeric_limits<value_type>::quiet_NaN();
    auto const saved_cross = cross;
    auto const saved_innovation = innovation;
    auto const before = bad;
    test.expect(formal_eskf::correction_covariances(bad.P, bad.r, bad.H, bad.V, minimum_norm, cross, innovation) ==
                        expected &&
                    same_fields(cross, saved_cross) && same_fields(innovation, saved_innovation) &&
                    same_fields(bad.P, before.P) && same_fields(bad.H, before.H) && same_fields(bad.V, before.V) &&
                    same_fields(bad.r, before.r),
                profile, "preparation failure preserves both disjoint outputs and every borrowed input");
}

template <typename Linalg, typename Configuration>
void test_preparation_domains(TestContext & test, std::string_view profile)
{
    test_matrix_domain<Linalg, Configuration>(test, profile);
    using fixture_type = CorrectionFixture<Linalg, Configuration, 2U>;
    using value_type = typename fixture_type::value_type;
    fixture_type fixture;
    for (value_type const bound : {value_type{0}, value_type{-1}, std::nextafter(value_type{1}, value_type{2})})
    {
        check_preparation_failure(test, profile, fixture, bound, Status::domain_error);
    }
    typename fixture_type::gain_type cross;
    auto innovation = fixture.V;
    test.expect(formal_eskf::correction_covariances(fixture.P, fixture.r, fixture.H, fixture.V, value_type{1}, cross,
                                                    innovation) == Status::success,
                profile, "upper normalization threshold endpoint is admitted by preparation");
}

template <typename Linalg, typename Configuration> void test_transaction(TestContext & test, std::string_view profile)
{
    using fixture_type = CorrectionFixture<Linalg, Configuration, 2U>;
    using value_type = typename fixture_type::value_type;
    if constexpr (!fixture_type::is_ins)
    {
        static_assert(!requires(fixture_type f, typename fixture_type::state_type out,
                                typename fixture_type::covariance_type covariance,
                                formal_eskf::configuration::Ins::BiasUpdate permission) {
            correct(f.state, f.P, f.r, f.H, f.V, f.minimum_norm, out, covariance, permission);
        });
    }
    fixture_type fixture;
    auto bad = fixture;
    bad.H(0U, 0U) = std::numeric_limits<value_type>::quiet_NaN();
    check_failure(test, profile, bad, value_type{0}, Status::non_finite_input,
                  "transaction returns matrix nonfinite failure before competing threshold domain failure");
    // Matrix preparation is required before nominal injection; invalid nominal
    // state must not overwrite an earlier solve failure.
    bad = fixture;
    bad.H = {};
    bad.V(0U, 1U) = value_type{2};
    bad.V(1U, 0U) = value_type{2};
    if constexpr (fixture_type::is_ins)
    {
        bad.state.p_n(0U) = std::numeric_limits<value_type>::quiet_NaN();
    }
    check_failure(test, profile, bad, fixture.minimum_norm, Status::not_positive_definite,
                  "transaction propagates first solve failure without publishing nominal or covariance");
    // Retry uses an unchanged prior after failure. The successful oracle has
    // exact dyadic C/S/Cholesky and zero attitude increment.
    fixture.P = fixture_type::covariance_type::identity();
    fixture.H = {};
    fixture.H(0U, 0U) = value_type{1};
    fixture.V = decltype(fixture.V)::identity() * value_type{3};
    fixture.r = {};
    auto expected_covariance = fixture.P;
    expected_covariance(0U, 0U) = value_type{0.75};
    for (unsigned aliases = 0U; aliases < 4U; ++aliases)
    {
        auto input = fixture;
        auto state_output = fixture.state;
        state_output.q_nb = -state_output.q_nb;
        auto covariance_output = fixture.P * value_type{2};
        auto & state = (aliases & 1U) != 0U ? input.state : state_output;
        auto & covariance = (aliases & 2U) != 0U ? input.P : covariance_output;
        auto const before_state = state;
        auto const before_covariance = covariance;
        test.expect(correct(input.state, input.P, input.r, input.H, input.V, value_type{0}, state, covariance) ==
                            Status::domain_error &&
                        same_state(state, before_state) && same_fields(covariance, before_covariance),
                    profile, "failed attempt leaves the same caller objects ready for a valid retry");
        test.expect(correct(input.state, input.P, input.r, input.H, input.V, fixture.minimum_norm, state, covariance) ==
                            Status::success &&
                        same_state(state, fixture.state) && same_fields(covariance, expected_covariance),
                    profile, "retry after independent failed calls gives exact joint publication for all aliases");
        test.expect(same_fields(input.H, fixture.H) && same_fields(input.V, fixture.V) &&
                        same_fields(input.r, fixture.r) &&
                        ((aliases & 1U) != 0U || same_state(input.state, fixture.state)) &&
                        ((aliases & 2U) != 0U || same_fields(input.P, fixture.P)),
                    profile, "successful transaction preserves every distinct borrowed input");
    }
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
        {"TC-E-CORRECT-preparation", "INS.binary32.M1", test_preparation<FloatLinalg, Ins, 1U>},
        {"TC-E-CORRECT-preparation", "INS.binary32.M2", test_preparation<FloatLinalg, Ins, 2U>},
        {"TC-E-CORRECT-preparation", "INS.binary32.M3", test_preparation<FloatLinalg, Ins, 3U>},
        {"TC-E-CORRECT-preparation", "INS.binary32.M4", test_preparation<FloatLinalg, Ins, 4U>},
        {"TC-E-CORRECT-preparation", "INS.binary32.M6", test_preparation<FloatLinalg, Ins, 6U>},
        {"TC-E-CORRECT-preparation", "INS.binary64.M1", test_preparation<DoubleLinalg, Ins, 1U>},
        {"TC-E-CORRECT-preparation", "INS.binary64.M2", test_preparation<DoubleLinalg, Ins, 2U>},
        {"TC-E-CORRECT-preparation", "INS.binary64.M3", test_preparation<DoubleLinalg, Ins, 3U>},
        {"TC-E-CORRECT-preparation", "INS.binary64.M4", test_preparation<DoubleLinalg, Ins, 4U>},
        {"TC-E-CORRECT-preparation", "INS.binary64.M6", test_preparation<DoubleLinalg, Ins, 6U>},
        {"TC-E-CORRECT-preparation", "AHRS.binary32.M1", test_preparation<FloatLinalg, Ahrs, 1U>},
        {"TC-E-CORRECT-preparation", "AHRS.binary32.M2", test_preparation<FloatLinalg, Ahrs, 2U>},
        {"TC-E-CORRECT-preparation", "AHRS.binary32.M3", test_preparation<FloatLinalg, Ahrs, 3U>},
        {"TC-E-CORRECT-preparation", "AHRS.binary32.M4", test_preparation<FloatLinalg, Ahrs, 4U>},
        {"TC-E-CORRECT-preparation", "AHRS.binary32.M6", test_preparation<FloatLinalg, Ahrs, 6U>},
        {"TC-E-CORRECT-preparation", "AHRS.binary64.M1", test_preparation<DoubleLinalg, Ahrs, 1U>},
        {"TC-E-CORRECT-preparation", "AHRS.binary64.M2", test_preparation<DoubleLinalg, Ahrs, 2U>},
        {"TC-E-CORRECT-preparation", "AHRS.binary64.M3", test_preparation<DoubleLinalg, Ahrs, 3U>},
        {"TC-E-CORRECT-preparation", "AHRS.binary64.M4", test_preparation<DoubleLinalg, Ahrs, 4U>},
        {"TC-E-CORRECT-preparation", "AHRS.binary64.M6", test_preparation<DoubleLinalg, Ahrs, 6U>},
        {"TC-E-CORRECT-matrix-domain", "INS.binary32", test_preparation_domains<FloatLinalg, Ins>},
        {"TC-E-CORRECT-matrix-domain", "INS.binary64", test_preparation_domains<DoubleLinalg, Ins>},
        {"TC-E-CORRECT-matrix-domain", "AHRS.binary32", test_preparation_domains<FloatLinalg, Ahrs>},
        {"TC-E-CORRECT-matrix-domain", "AHRS.binary64", test_preparation_domains<DoubleLinalg, Ahrs>},
        {"TC-E-CORRECT-right-solve", "INS.binary32", test_right_solve<FloatLinalg, Ins>},
        {"TC-E-CORRECT-right-solve", "INS.binary64", test_right_solve<DoubleLinalg, Ins>},
        {"TC-E-CORRECT-right-solve", "AHRS.binary32", test_right_solve<FloatLinalg, Ahrs>},
        {"TC-E-CORRECT-right-solve", "AHRS.binary64", test_right_solve<DoubleLinalg, Ahrs>},
        {"TC-E-CORRECT-joseph-prior", "INS.binary32.M1", test_joseph_prior<FloatLinalg, Ins, 1U>},
        {"TC-E-CORRECT-joseph-prior", "INS.binary32.M2", test_joseph_prior<FloatLinalg, Ins, 2U>},
        {"TC-E-CORRECT-joseph-prior", "INS.binary32.M3", test_joseph_prior<FloatLinalg, Ins, 3U>},
        {"TC-E-CORRECT-joseph-prior", "INS.binary32.M4", test_joseph_prior<FloatLinalg, Ins, 4U>},
        {"TC-E-CORRECT-joseph-prior", "INS.binary32.M6", test_joseph_prior<FloatLinalg, Ins, 6U>},
        {"TC-E-CORRECT-joseph-prior", "INS.binary64.M1", test_joseph_prior<DoubleLinalg, Ins, 1U>},
        {"TC-E-CORRECT-joseph-prior", "INS.binary64.M2", test_joseph_prior<DoubleLinalg, Ins, 2U>},
        {"TC-E-CORRECT-joseph-prior", "INS.binary64.M3", test_joseph_prior<DoubleLinalg, Ins, 3U>},
        {"TC-E-CORRECT-joseph-prior", "INS.binary64.M4", test_joseph_prior<DoubleLinalg, Ins, 4U>},
        {"TC-E-CORRECT-joseph-prior", "INS.binary64.M6", test_joseph_prior<DoubleLinalg, Ins, 6U>},
        {"TC-E-CORRECT-joseph-prior", "AHRS.binary32.M1", test_joseph_prior<FloatLinalg, Ahrs, 1U>},
        {"TC-E-CORRECT-joseph-prior", "AHRS.binary32.M2", test_joseph_prior<FloatLinalg, Ahrs, 2U>},
        {"TC-E-CORRECT-joseph-prior", "AHRS.binary32.M3", test_joseph_prior<FloatLinalg, Ahrs, 3U>},
        {"TC-E-CORRECT-joseph-prior", "AHRS.binary32.M4", test_joseph_prior<FloatLinalg, Ahrs, 4U>},
        {"TC-E-CORRECT-joseph-prior", "AHRS.binary32.M6", test_joseph_prior<FloatLinalg, Ahrs, 6U>},
        {"TC-E-CORRECT-joseph-prior", "AHRS.binary64.M1", test_joseph_prior<DoubleLinalg, Ahrs, 1U>},
        {"TC-E-CORRECT-joseph-prior", "AHRS.binary64.M2", test_joseph_prior<DoubleLinalg, Ahrs, 2U>},
        {"TC-E-CORRECT-joseph-prior", "AHRS.binary64.M3", test_joseph_prior<DoubleLinalg, Ahrs, 3U>},
        {"TC-E-CORRECT-joseph-prior", "AHRS.binary64.M4", test_joseph_prior<DoubleLinalg, Ahrs, 4U>},
        {"TC-E-CORRECT-joseph-prior", "AHRS.binary64.M6", test_joseph_prior<DoubleLinalg, Ahrs, 6U>},
        {"TC-E-CORRECT-zero-information", "INS.binary32", test_zero_information<FloatLinalg, Ins>},
        {"TC-E-CORRECT-zero-information", "INS.binary64", test_zero_information<DoubleLinalg, Ins>},
        {"TC-E-CORRECT-zero-information", "AHRS.binary32", test_zero_information<FloatLinalg, Ahrs>},
        {"TC-E-CORRECT-zero-information", "AHRS.binary64", test_zero_information<DoubleLinalg, Ahrs>},
        {"TC-E-CORRECT-arithmetic-boundaries", "INS.binary32", test_arithmetic_boundaries<FloatLinalg, Ins>},
        {"TC-E-CORRECT-arithmetic-boundaries", "INS.binary64", test_arithmetic_boundaries<DoubleLinalg, Ins>},
        {"TC-E-CORRECT-arithmetic-boundaries", "AHRS.binary32", test_arithmetic_boundaries<FloatLinalg, Ahrs>},
        {"TC-E-CORRECT-arithmetic-boundaries", "AHRS.binary64", test_arithmetic_boundaries<DoubleLinalg, Ahrs>},
        {"TC-E-STEP-CORRECT-transaction", "INS.binary32", test_transaction<FloatLinalg, Ins>},
        {"TC-E-STEP-CORRECT-transaction", "INS.binary64", test_transaction<DoubleLinalg, Ins>},
        {"TC-E-STEP-CORRECT-transaction", "AHRS.binary32", test_transaction<FloatLinalg, Ahrs>},
        {"TC-E-STEP-CORRECT-transaction", "AHRS.binary64", test_transaction<DoubleLinalg, Ahrs>},
        {"TC-E-BIAS-UPDATE-all-permissions", "INS.binary32.M1", test_bias_update<FloatLinalg, 1U>},
        {"TC-E-BIAS-UPDATE-all-permissions", "INS.binary32.M2", test_bias_update<FloatLinalg, 2U>},
        {"TC-E-BIAS-UPDATE-all-permissions", "INS.binary32.M3", test_bias_update<FloatLinalg, 3U>},
        {"TC-E-BIAS-UPDATE-all-permissions", "INS.binary32.M4", test_bias_update<FloatLinalg, 4U>},
        {"TC-E-BIAS-UPDATE-all-permissions", "INS.binary32.M6", test_bias_update<FloatLinalg, 6U>},
        {"TC-E-BIAS-UPDATE-call-local", "INS.binary32", test_bias_update_failures<FloatLinalg>},
        {"TC-E-BIAS-UPDATE-all-permissions", "INS.binary64.M1", test_bias_update<DoubleLinalg, 1U>},
        {"TC-E-BIAS-UPDATE-all-permissions", "INS.binary64.M2", test_bias_update<DoubleLinalg, 2U>},
        {"TC-E-BIAS-UPDATE-all-permissions", "INS.binary64.M3", test_bias_update<DoubleLinalg, 3U>},
        {"TC-E-BIAS-UPDATE-all-permissions", "INS.binary64.M4", test_bias_update<DoubleLinalg, 4U>},
        {"TC-E-BIAS-UPDATE-all-permissions", "INS.binary64.M6", test_bias_update<DoubleLinalg, 6U>},
        {"TC-E-BIAS-UPDATE-call-local", "INS.binary64", test_bias_update_failures<DoubleLinalg>},
    };
    std::cout << "Configuration: reset_approx=" << ESKF_RESET_APPROX << " codegen=" << ESKF_CODEGEN << '\n';
    return formal_eskf::test::run_cases(cases, argc, argv);
}
