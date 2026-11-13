/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Include standalone quality API first to check its own dependencies.
#include <formal_eskf/runtime/magnetic_field.hpp>
#include <formal_eskf/runtime/magnetometer.hpp>

#include <array>
#include <bit>
#include <cmath>
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
using formal_eskf::runtime::check_magnetic_field;
using formal_eskf::runtime::FusionDecision;
using TestContext = formal_eskf::test::Context;

template <typename Linalg> using Vector = formal_eskf::linalg::Matrix<Linalg, 3U, 1U>;

template <typename Linalg> auto field_parameters()
{
    using value_type = typename Linalg::value_type;
    return formal_eskf::runtime::MagneticFieldParameters<value_type>{value_type{5}, static_cast<value_type>(0.2),
                                                                     static_cast<value_type>(0.2), value_type{1}};
}

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

template <typename Linalg> void test_field_checks(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using vector_type = Vector<Linalg>;
    formal_eskf::so3::UnitQuaternion<Linalg> const identity;
    auto parameters = field_parameters<Linalg>();
    auto const reference = vector_type::from_row_major({value_type{30}, value_type{0}, value_type{40}});
    auto result = check_magnetic_field(identity, reference, reference, parameters, true);
    test.expect(result.status == Status::success && result.accepted && result.diagnostics_valid &&
                    result.heading_checked && result.strength == value_type{50} &&
                    result.reference_strength == value_type{50} && result.heading_error == value_type{0} &&
                    result.inclination_error == value_type{0},
                profile, "matching calibrated field passes all checks");

    auto sample = reference * value_type{2};
    result = check_magnetic_field(identity, sample, reference, parameters, true);
    test.expect(result.status == Status::success && !result.accepted && result.failures.strength &&
                    !result.failures.inclination && !result.failures.heading,
                profile, "same direction with excessive strength is rejected");
    sample = vector_type::from_row_major({value_type{40}, value_type{0}, value_type{30}});
    result = check_magnetic_field(identity, sample, reference, parameters, true);
    test.expect(!result.accepted && !result.failures.strength && result.failures.inclination &&
                    !result.failures.heading,
                profile, "same strength but different inclination is rejected");
    sample = vector_type::from_row_major({value_type{0}, value_type{30}, value_type{40}});
    result = check_magnetic_field(identity, sample, reference, parameters, true);
    test.expect(!result.accepted && !result.failures.strength && !result.failures.inclination &&
                    result.failures.heading && result.heading_checked,
                profile, "horizontal-direction disturbance needs an independent heading premise");
    result = check_magnetic_field(identity, sample, reference, parameters, false);
    test.expect(result.accepted && !result.heading_checked && result.heading_error == value_type{0}, profile,
                "unobservable heading skips only the heading consistency check");

    auto const vertical = vector_type::from_row_major({value_type{0}, value_type{0}, value_type{50}});
    result = check_magnetic_field(identity, vertical, vertical, parameters, true);
    test.expect(result.status == Status::success && result.diagnostics_valid && !result.accepted &&
                    result.failures.horizontal_field && !result.heading_checked,
                profile, "vertical fields have valid inclination but no usable heading");
    for (bool weak_reference : {false, true})
    {
        result = check_magnetic_field(identity, weak_reference ? reference : vertical,
                                      weak_reference ? vertical : reference, parameters, false);
        test.expect(result.failures.horizontal_field && !result.accepted, profile,
                    "either weak horizontal field rejects, even when heading consistency is skipped");
    }

    auto const west_n = vector_type::from_row_major({value_type{-40}, value_type{0.5}, value_type{10}});
    auto const west_b = vector_type::from_row_major({value_type{-40}, value_type{-0.5}, value_type{10}});
    for (bool reverse : {false, true})
    {
        result = check_magnetic_field(identity, reverse ? west_n : west_b, reverse ? west_b : west_n, parameters, true);
        long double const expected = (reverse ? -2.0L : 2.0L) * std::atan2(0.5L, 40.0L);
        test.expect(result.accepted && std::abs(static_cast<long double>(result.heading_error) - expected) < 1e-6L,
                    profile, "heading wraps across both sides of the +/-pi boundary");
    }

    auto const north = vector_type::from_row_major({value_type{5}, value_type{0}, value_type{0}});
    auto const stronger = north * value_type{2};
    result = check_magnetic_field(identity, stronger, north, parameters, true);
    test.expect(result.accepted, profile, "strength tolerance equality passes");
    parameters.strength_tolerance = std::nextafter(value_type{5}, value_type{0});
    test.expect(check_magnetic_field(identity, stronger, north, parameters, true).failures.strength, profile,
                "just beyond strength tolerance rejects");
    parameters = field_parameters<Linalg>();
    parameters.minimum_horizontal_field = value_type{5};
    test.expect(check_magnetic_field(identity, north, north, parameters, true).accepted, profile,
                "minimum horizontal equality passes");
    parameters.minimum_horizontal_field = std::nextafter(value_type{5}, value_type{6});
    test.expect(check_magnetic_field(identity, north, north, parameters, true).failures.horizontal_field, profile,
                "just below minimum horizontal rejects");

    parameters = field_parameters<Linalg>();
    sample = vector_type::from_row_major({value_type{0}, value_type{5}, value_type{0}});
    result = check_magnetic_field(identity, sample, north, parameters, true);
    parameters.heading_tolerance = std::abs(result.heading_error);
    test.expect(check_magnetic_field(identity, sample, north, parameters, true).accepted, profile,
                "computed heading tolerance equality passes");
    parameters.heading_tolerance = std::nextafter(parameters.heading_tolerance, value_type{0});
    test.expect(check_magnetic_field(identity, sample, north, parameters, true).failures.heading, profile,
                "computed heading just beyond tolerance rejects");
    parameters = field_parameters<Linalg>();
    sample = vector_type::from_row_major({value_type{3}, value_type{0}, value_type{4}});
    result = check_magnetic_field(identity, sample, north, parameters, true);
    parameters.inclination_tolerance = std::abs(result.inclination_error);
    test.expect(check_magnetic_field(identity, sample, north, parameters, true).accepted, profile,
                "computed inclination tolerance equality passes");
    parameters.inclination_tolerance = std::nextafter(parameters.inclination_tolerance, value_type{0});
    test.expect(check_magnetic_field(identity, sample, north, parameters, true).failures.inclination, profile,
                "computed inclination just beyond tolerance rejects");
}

template <typename Linalg> void test_field_selection(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using vector_type = Vector<Linalg>;
    formal_eskf::so3::UnitQuaternion<Linalg> const identity;
    auto const reference = vector_type::from_row_major({value_type{30}, value_type{0}, value_type{40}});
    auto const parameters = field_parameters<Linalg>();
    std::array<vector_type, 3U> const samples{
        reference, reference * value_type{2},
        vector_type::from_row_major({value_type{40}, value_type{0}, value_type{30}})};
    for (unsigned mask = 0U; mask < 8U; ++mask)
    {
        for (bool available : {false, true})
        {
            formal_eskf::runtime::MagneticFieldChecks const checks{(mask & 1U) != 0U, (mask & 2U) != 0U,
                                                                   (mask & 4U) != 0U, available};
            for (std::size_t sample = 0U; sample < samples.size(); ++sample)
            {
                auto const result =
                    check_magnetic_field(identity, samples[sample], reference, parameters, false, checks);
                bool const waiting = !available && checks.require_reference && (checks.strength || checks.inclination);
                bool const strength_checked = checks.strength && (available || !checks.require_reference);
                bool const inclination_checked = checks.inclination && available;
                bool const bad_strength = strength_checked && sample == 1U;
                bool const bad_inclination = inclination_checked && sample == 2U;
                test.expect(
                    result.status == Status::success && result.diagnostics_valid &&
                        result.failures.reference == waiting && result.strength_checked == strength_checked &&
                        result.inclination_checked == inclination_checked && result.failures.strength == bad_strength &&
                        result.failures.inclination == bad_inclination &&
                        result.accepted == !(waiting || bad_strength || bad_inclination),
                    profile, "all field selections distinguish absent reference, disabled checks and disturbances");
                if (!inclination_checked)
                {
                    test.expect(result.inclination_error == value_type{0}, profile,
                                "unchecked inclination has no reported comparison residual");
                }
            }
        }
    }
    formal_eskf::runtime::MagneticFieldChecks const fallback{true, true, false, false};
    for (value_type magnitude : {value_type{5}, value_type{85}})
    {
        auto const sample = vector_type::from_row_major({magnitude, value_type{0}, value_type{0}});
        auto const result = check_magnetic_field(identity, sample, reference, parameters, false, fallback);
        test.expect(result.accepted && result.strength_checked && !result.inclination_checked &&
                        result.reference_strength == value_type{45},
                    profile, "without a reference the inclusive 5 to 85 microtesla range applies");
    }
    for (value_type magnitude :
         {std::nextafter(value_type{5}, value_type{0}), std::nextafter(value_type{85}, value_type{100})})
    {
        auto const sample = vector_type::from_row_major({magnitude, value_type{0}, value_type{0}});
        test.expect(check_magnetic_field(identity, sample, reference, parameters, false, fallback).failures.strength,
                    profile, "just outside the fallback range rejects independently of configured tolerance");
    }
    auto const vertical = vector_type::from_row_major({value_type{0}, value_type{0}, value_type{50}});
    formal_eskf::runtime::MagneticFieldChecks const disabled{false, false, false, false};
    test.expect(
        check_magnetic_field(identity, vertical, reference, parameters, false, disabled).failures.horizontal_field,
        profile, "disabling physical checks retains the separate weak-heading guard");
    auto const east = vector_type::from_row_major({value_type{0}, value_type{30}, value_type{40}});
    test.expect(check_magnetic_field(identity, east, reference, parameters, true, disabled).failures.heading, profile,
                "field selection does not disable the separately requested heading check");
}

template <typename Linalg> void test_field_frames(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using quaternion_type = formal_eskf::so3::UnitQuaternion<Linalg>;
    using coefficients_type = formal_eskf::linalg::Matrix<Linalg, 4U, 1U>;
    auto const reference = Vector<Linalg>::from_row_major({value_type{25}, value_type{-12}, value_type{-40}});
    std::array<std::array<value_type, 4U>, 5U> const orientations{{
        {value_type{1}, value_type{0}, value_type{0}, value_type{0}},
        {value_type{1}, value_type{0}, value_type{1}, value_type{0}}, // 90-degree pitch, no Euler singularity.
        {value_type{0}, value_type{1}, value_type{0}, value_type{0}},
        {value_type{4}, value_type{1}, value_type{-2}, value_type{2}},
        {value_type{-4}, value_type{-1}, value_type{2}, value_type{-2}},
    }};
    for (auto const & coefficients : orientations)
    {
        quaternion_type q;
        test.expect(
            quaternion_type::try_from_coefficients(
                coefficients_type::from_row_major({coefficients[0], coefficients[1], coefficients[2], coefficients[3]}),
                static_cast<value_type>(1e-6), q) == Status::success,
            profile, "test orientation constructed");
        // Independent long-double rotation: v_b = v_n + 2 w (u x v_n) + 2 u x (u x v_n), u=-q_vec.
        std::array<long double, 3U> const u{-q.q1(), -q.q2(), -q.q3()};
        std::array<long double, 3U> first{};
        Vector<Linalg> measured;
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            auto const j = (axis + 1U) % 3U;
            auto const k = (axis + 2U) % 3U;
            first[axis] = u[j] * reference(k) - u[k] * reference(j);
        }
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            auto const j = (axis + 1U) % 3U;
            auto const k = (axis + 2U) % 3U;
            measured(axis) = static_cast<value_type>(reference(axis) + 2.0L * q.q0() * first[axis] +
                                                     2.0L * (u[j] * first[k] - u[k] * first[j]));
        }
        auto const result = check_magnetic_field(q, measured, reference, field_parameters<Linalg>(), true);
        test.expect(result.accepted && std::abs(result.inclination_error) < static_cast<value_type>(1e-5) &&
                        std::abs(result.heading_error) < static_cast<value_type>(1e-5) &&
                        std::abs(static_cast<long double>(result.strength) - std::sqrt(2369.0L)) < 1e-4L,
                    profile, "body-to-NED sign, declination, negative dip and quaternion sign agree with oracle");
    }
}

template <typename Linalg> void test_invalid_field(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using limits_type = std::numeric_limits<value_type>;
    using quaternion_type = formal_eskf::so3::UnitQuaternion<Linalg>;
    quaternion_type const identity;
    auto const reference = Vector<Linalg>::from_row_major({value_type{30}, value_type{0}, value_type{40}});
    auto const parameters = field_parameters<Linalg>();
    auto expect_failure = [&](auto const & result, Status expected)
    {
        test.expect(result.status == expected && !result.accepted && !result.diagnostics_valid &&
                        !result.heading_checked && result.strength == value_type{0} &&
                        result.reference_strength == value_type{0} && result.inclination_error == value_type{0} &&
                        result.heading_error == value_type{0},
                    profile, "invalid input/arithmetic fails closed without partially published diagnostics");
    };
    for (value_type invalid : {limits_type::quiet_NaN(), limits_type::infinity(), -limits_type::infinity()})
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            auto bad = reference;
            bad(axis) = invalid;
            expect_failure(check_magnetic_field(identity, bad, reference, parameters, true), Status::non_finite_input);
            expect_failure(check_magnetic_field(identity, reference, bad, parameters, true), Status::non_finite_input);
        }
        for (std::size_t index = 0U; index < 4U; ++index)
        {
            auto bad = parameters;
            std::array<value_type *, 4U> fields{&bad.strength_tolerance, &bad.inclination_tolerance,
                                                &bad.heading_tolerance, &bad.minimum_horizontal_field};
            *fields[index] = invalid;
            expect_failure(check_magnetic_field(identity, reference, reference, bad, false), Status::non_finite_input);
            // Test-only invalid representation; never construct this through the checked public API.
            std::array<value_type, 4U> coefficients{value_type{1}, value_type{0}, value_type{0}, value_type{0}};
            coefficients[index] = invalid;
            static_assert(sizeof(quaternion_type) == sizeof(coefficients));
            auto const q = std::bit_cast<quaternion_type>(coefficients);
            expect_failure(check_magnetic_field(q, reference, reference, parameters, true), Status::non_finite_input);
        }
    }
    for (std::size_t index = 0U; index < 4U; ++index)
    {
        auto bad = parameters;
        std::array<value_type *, 4U> fields{&bad.strength_tolerance, &bad.inclination_tolerance, &bad.heading_tolerance,
                                            &bad.minimum_horizontal_field};
        *fields[index] = value_type{-1};
        expect_failure(check_magnetic_field(identity, reference, reference, bad, true), Status::domain_error);
    }
    auto bad_parameters = parameters;
    bad_parameters.inclination_tolerance = value_type{4};
    expect_failure(check_magnetic_field(identity, reference, reference, bad_parameters, true), Status::domain_error);
    bad_parameters = parameters;
    bad_parameters.heading_tolerance = value_type{4};
    expect_failure(check_magnetic_field(identity, reference, reference, bad_parameters, false), Status::domain_error);
    expect_failure(check_magnetic_field(identity, reference, reference, {}, true), Status::domain_error);
    expect_failure(check_magnetic_field(identity, Vector<Linalg>{}, reference, parameters, true), Status::domain_error);
    expect_failure(check_magnetic_field(identity, reference, Vector<Linalg>{}, parameters, true), Status::domain_error);
    auto huge = reference;
    huge(2U) = limits_type::max();
    expect_failure(check_magnetic_field(identity, huge, reference, parameters, true), Status::non_finite_result);
    expect_failure(check_magnetic_field(identity, reference, huge, parameters, true), Status::non_finite_result);
    auto const tiny = Vector<Linalg>::from_row_major({limits_type::min(), value_type{0}, value_type{0}});
    expect_failure(check_magnetic_field(identity, tiny, reference, parameters, true), Status::zero_or_unsafe_divisor);
    expect_failure(check_magnetic_field(identity, reference, tiny, parameters, true), Status::zero_or_unsafe_divisor);
}

template <typename Linalg, typename Configuration> struct FusionFixture
{
    using value_type = typename Linalg::value_type;
    using state_type = typename Configuration::template NominalState<Linalg>;
    static constexpr std::size_t size = Configuration::error_state_dimension;
    using covariance_type = formal_eskf::linalg::Matrix<Linalg, size, size>;
    using noise_type = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    state_type state{};
    covariance_type P = covariance_type::identity();
    Vector<Linalg> reference = Vector<Linalg>::from_row_major({value_type{30}, value_type{0}, value_type{40}});
    Vector<Linalg> sample = Vector<Linalg>::from_row_major({value_type{30}, value_type{0.5}, value_type{40}});
    noise_type V = noise_type::identity();
    formal_eskf::runtime::MagneticFieldParameters<value_type> parameters = field_parameters<Linalg>();
    value_type minimum_norm = static_cast<value_type>(1e-6);
    value_type sigma = value_type{3};
    bool heading_observable = true;
    Ins::BiasUpdate bias_update{};
    formal_eskf::runtime::MagneticFieldChecks checks{};

    FusionFixture()
    {
        for (std::size_t row = 0U; row < size; ++row)
        {
            for (std::size_t column = 0U; column < size; ++column)
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
            return formal_eskf::runtime::fuse_magnetometer(state, P, sample, reference, V, parameters,
                                                           heading_observable, minimum_norm, sigma, output, output_P,
                                                           bias_update, checks);
        }
        else
        {
            return formal_eskf::runtime::fuse_magnetometer(state, P, sample, reference, V, parameters,
                                                           heading_observable, minimum_norm, sigma, output, output_P,
                                                           checks);
        }
    }

    Status correct(state_type & output, covariance_type & output_P) const
    {
        if constexpr (std::is_same_v<Configuration, Ins>)
        {
            return formal_eskf::try_correct_magnetometer(state, P, sample, reference, V, minimum_norm, output, output_P,
                                                         bias_update);
        }
        else
        {
            return formal_eskf::try_correct_magnetometer(state, P, sample, reference, V, minimum_norm, output,
                                                         output_P);
        }
    }
};

template <typename Linalg, typename Configuration>
void test_missing_reference(TestContext & test, std::string_view profile)
{
    FusionFixture<Linalg, Configuration> fixture;
    fixture.checks.require_reference = true;
    fixture.checks.reference_available = false;
    // A missing required reference must reject before consuming covariance/noise.
    fixture.V(0U, 0U) = std::numeric_limits<typename Linalg::value_type>::quiet_NaN();
    auto output = fixture.state;
    auto output_P = fixture.P;
    auto const result = fixture.fuse(output, output_P);
    test.expect(result.fusion.status == Status::success && result.fusion.decision == FusionDecision::rejected &&
                    result.field.failures.reference && result.field.diagnostics_valid &&
                    !result.fusion.diagnostics_valid && same_state(output, fixture.state) &&
                    same_bits(output_P, fixture.P),
                profile, "missing required reference rejects atomically before innovation/correction");
}

template <typename Linalg, typename Configuration> void test_fusion(TestContext & test, std::string_view profile)
{
    using fixture_type = FusionFixture<Linalg, Configuration>;
    using value_type = typename fixture_type::value_type;
    fixture_type fixture;
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
        test.expect(fixture.correct(expected, expected_P) == Status::success, profile,
                    "existing magnetic correction succeeds for comparison");
        for (unsigned aliases = 0U; aliases < 4U; ++aliases)
        {
            auto current = fixture;
            typename fixture_type::state_type output;
            auto output_P = fixture.P * value_type{-1};
            auto & state_output = (aliases & 1U) != 0U ? current.state : output;
            auto & covariance_output = (aliases & 2U) != 0U ? current.P : output_P;
            auto const result = current.fuse(state_output, covariance_output);
            test.expect(result.fusion.status == Status::success && result.fusion.decision == FusionDecision::fused &&
                            result.field.accepted && result.fusion.diagnostics_valid &&
                            same_state(state_output, expected) && same_bits(covariance_output, expected_P),
                        profile, "all aliases and bias permissions retain the configured magnetic correction");
            test.expect(((aliases & 1U) != 0U || same_state(current.state, fixture.state)) &&
                            ((aliases & 2U) != 0U || same_bits(current.P, fixture.P)) &&
                            same_bits(current.sample, fixture.sample) &&
                            same_bits(current.reference, fixture.reference) && same_bits(current.V, fixture.V),
                        profile, "disjoint inputs remain unchanged");
        }
    }
    typename fixture_type::state_type output;
    typename fixture_type::covariance_type output_P;
    auto const result = fixture.fuse(output, output_P);
    // Independent identity-attitude Jacobian and scalar summation, not production H/S.
    std::array<std::array<long double, 3U>, 3U> const hat{{{0, -40, 0}, {40, 0, -30}, {0, 30, 0}}};
    constexpr std::size_t offset = std::is_same_v<Configuration, Ins> ? 6U : 0U;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        long double variance = fixture.V(axis, axis);
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            for (std::size_t column = 0U; column < 3U; ++column)
            {
                variance += hat[axis][row] * fixture.P(offset + row, offset + column) * hat[axis][column];
            }
        }
        long double const residual = axis == 1U ? 0.5L : 0.0L;
        long double const tolerance = std::is_same_v<value_type, float> ? 1e-6L : 1e-12L;
        test.expect(std::abs(static_cast<long double>(result.fusion.innovation_variance(axis)) - variance) <=
                            tolerance * variance &&
                        std::abs(static_cast<long double>(result.fusion.test_ratio(axis)) -
                                 residual * residual / (9.0L * variance)) < tolerance &&
                        result.fusion.innovation(axis) == static_cast<value_type>(residual),
                    profile, "innovation gate uses original full H/P/V, including prior attitude uncertainty");
    }
    test.expect(result.fusion.innovation_variance(1U) > fixture.V(1U, 1U), profile,
                "gate variance is not the measurement noise alone");
}

template <typename Linalg, typename Configuration> void test_rejection(TestContext & test, std::string_view profile)
{
    enum class Scenario
    {
        excessive_field_strength,
        heading_mismatch,
        innovation_outlier,
        non_finite_noise_covariance,
        zero_gate_sigma,
        indefinite_innovation_covariance,
        negative_measurement_variance,
        invalid_norm_bound,
        field_rejection_before_noise_validation,
        non_finite_measurement,
    };

    using fixture_type = FusionFixture<Linalg, Configuration>;
    using value_type = typename fixture_type::value_type;
    using limits_type = std::numeric_limits<value_type>;
    for (auto const scenario :
         {Scenario::excessive_field_strength, Scenario::heading_mismatch, Scenario::innovation_outlier,
          Scenario::non_finite_noise_covariance, Scenario::zero_gate_sigma, Scenario::indefinite_innovation_covariance,
          Scenario::negative_measurement_variance, Scenario::invalid_norm_bound,
          Scenario::field_rejection_before_noise_validation, Scenario::non_finite_measurement})
    {
        fixture_type fixture;
        FusionDecision expected = FusionDecision::failed;
        Status status = Status::success;
        bool gate_valid = false;
        switch (scenario)
        {
        case Scenario::excessive_field_strength:
            fixture.sample = fixture.reference * value_type{2};
            expected = FusionDecision::rejected;
            break;
        case Scenario::heading_mismatch:
            fixture.sample = Vector<Linalg>::from_row_major({value_type{0}, value_type{30}, value_type{40}});
            expected = FusionDecision::rejected;
            break;
        case Scenario::innovation_outlier:
            fixture.heading_observable = false;
            fixture.P = typename fixture_type::covariance_type{};
            fixture.sample = Vector<Linalg>::from_row_major({value_type{0}, value_type{30}, value_type{40}});
            expected = FusionDecision::rejected;
            gate_valid = true;
            break;
        case Scenario::non_finite_noise_covariance:
            fixture.V(2U, 2U) = limits_type::quiet_NaN();
            status = Status::non_finite_input;
            break;
        case Scenario::zero_gate_sigma:
            fixture.sigma = value_type{0};
            status = Status::domain_error;
            break;
        case Scenario::indefinite_innovation_covariance:
            // Positive diagonal S can pass the component gate yet fail LLT.
            fixture.P = typename fixture_type::covariance_type{};
            fixture.V = fixture_type::noise_type::identity();
            fixture.V(0U, 1U) = value_type{2};
            fixture.V(1U, 0U) = value_type{2};
            status = Status::not_positive_definite;
            gate_valid = true;
            break;
        case Scenario::negative_measurement_variance:
            fixture.V(0U, 0U) = value_type{-1};
            status = Status::domain_error;
            break;
        case Scenario::invalid_norm_bound:
            fixture.minimum_norm = value_type{2};
            status = Status::domain_error;
            break;
        case Scenario::field_rejection_before_noise_validation:
            // Field rejection does not certify inputs of later, unexecuted stages.
            fixture.sample = fixture.reference * value_type{2};
            fixture.V(2U, 2U) = limits_type::quiet_NaN();
            expected = FusionDecision::rejected;
            break;
        case Scenario::non_finite_measurement:
            fixture.sample(2U) = limits_type::infinity();
            status = Status::non_finite_input;
            break;
        }
        for (unsigned aliases = 0U; aliases < 4U; ++aliases)
        {
            auto current = fixture;
            typename fixture_type::state_type output;
            using quaternion_type = formal_eskf::so3::UnitQuaternion<Linalg>;
            std::array<value_type, 4U> const sentinel{limits_type::quiet_NaN(), value_type{1}, -value_type{0},
                                                      value_type{0}};
            output.q_nb = std::bit_cast<quaternion_type>(sentinel);
            auto output_P = fixture.P * value_type{-1};
            output_P(0U, 0U) = limits_type::quiet_NaN();
            output_P(0U, 1U) = -value_type{0};
            auto & state_output = (aliases & 1U) != 0U ? current.state : output;
            auto & covariance_output = (aliases & 2U) != 0U ? current.P : output_P;
            auto const old_state = state_output;
            auto const old_P = covariance_output;
            auto const result = current.fuse(state_output, covariance_output);
            test.expect(result.fusion.decision == expected && result.fusion.status == status &&
                            result.fusion.diagnostics_valid == gate_valid && same_state(state_output, old_state) &&
                            same_bits(covariance_output, old_P),
                        profile, "field/gate rejection and numerical failures preserve all output aliases");
            if (scenario == Scenario::innovation_outlier)
            {
                test.expect(result.field.accepted && !result.field.heading_checked &&
                                result.fusion.test_ratio(0U) > value_type{1} &&
                                result.fusion.test_ratio(1U) > value_type{1},
                            profile, "skipping heading consistency never skips innovation rejection");
            }
        }
    }
}

template <typename Linalg, typename Configuration>
void test_fusion_boundaries(TestContext & test, std::string_view profile)
{
    using fixture_type = FusionFixture<Linalg, Configuration>;
    using value_type = typename fixture_type::value_type;
    fixture_type fixture;
    fixture.P = typename fixture_type::covariance_type{};
    fixture.reference = Vector<Linalg>::from_row_major({value_type{3}, value_type{4}, value_type{0}});
    fixture.sample = fixture.reference * value_type{2};
    fixture.V = fixture_type::noise_type::identity();
    fixture.V(0U, 0U) = value_type{9};
    fixture.V(1U, 1U) = value_type{16};
    fixture.sigma = value_type{1};
    typename fixture_type::state_type output;
    typename fixture_type::covariance_type output_P;
    auto result = fixture.fuse(output, output_P);
    test.expect(result.fusion.decision == FusionDecision::fused && result.fusion.test_ratio(0U) == value_type{1} &&
                    result.fusion.test_ratio(1U) == value_type{1},
                profile, "field tolerance and two innovation boundaries can pass at equality");
    fixture.V(0U, 0U) = std::nextafter(value_type{9}, value_type{0});
    result = fixture.fuse(output, output_P);
    test.expect(result.field.accepted && result.fusion.decision == FusionDecision::rejected &&
                    result.fusion.test_ratio(0U) > value_type{1} && result.fusion.test_ratio(1U) == value_type{1},
                profile, "one outlying axis rejects the whole magnetic group");

    // A tilted prior exercises the gate/correction's common body-frame model,
    // while the independent field-direction check operates in NED.
    fixture = fixture_type{};
    using quaternion_type = formal_eskf::so3::UnitQuaternion<Linalg>;
    test.expect(quaternion_type::try_from_coefficients(value_type{4}, value_type{1}, value_type{-2}, value_type{2},
                                                       fixture.minimum_norm, fixture.state.q_nb) == Status::success,
                profile, "tilted fusion prior constructed");
    fixture.sample = formal_eskf::so3::inverse_rotate(fixture.state.q_nb, fixture.reference);
    fixture.sample(1U) += value_type{0.5};
    typename fixture_type::state_type expected;
    typename fixture_type::covariance_type expected_P;
    test.expect(fixture.correct(expected, expected_P) == Status::success, profile, "tilted core correction succeeds");
    result = fixture.fuse(output, output_P);
    test.expect(result.fusion.decision == FusionDecision::fused && same_state(output, expected) &&
                    same_bits(output_P, expected_P),
                profile, "tilted gated correction retains core projection and full covariance update");
    fixture.state.q_nb = -fixture.state.q_nb;
    result = fixture.fuse(output, output_P);
    test.expect(result.fusion.decision == FusionDecision::fused && same_bits(output_P, expected_P), profile,
                "quaternion sign does not change admission or covariance");

    if constexpr (std::is_same_v<Configuration, Ins>)
    {
        for (std::size_t field = 0U; field < 4U; ++field)
        {
            auto invalid = fixture;
            std::array<Vector<Linalg> *, 4U> fields{&invalid.state.p_n, &invalid.state.v_n, &invalid.state.b_a,
                                                    &invalid.state.b_g};
            (*fields[field])(0U) = std::numeric_limits<value_type>::quiet_NaN();
            auto const before = output;
            auto const before_P = output_P;
            auto const failure = invalid.fuse(output, output_P);
            test.expect(failure.fusion.decision == FusionDecision::failed &&
                            failure.fusion.status == Status::non_finite_input && !failure.field.diagnostics_valid &&
                            !failure.fusion.diagnostics_valid && same_state(output, before) &&
                            same_bits(output_P, before_P),
                        profile, "invalid INS nominal fields fail before all measurement stages");
        }
    }
}

template <typename Linalg, typename Configuration>
void test_separate_references(TestContext & test, std::string_view profile)
{
    using fixture_type = FusionFixture<Linalg, Configuration>;
    using value_type = typename fixture_type::value_type;
    fixture_type fixture;
    // Same strength/inclination, but a 90-degree different model declination.
    auto health = Vector<Linalg>::from_row_major({value_type{0}, value_type{30}, value_type{40}});
    typename fixture_type::state_type output, expected;
    typename fixture_type::covariance_type output_P, expected_P;
    auto fuse = [&]()
    {
        if constexpr (std::is_same_v<Configuration, Ins>)
        {
            return formal_eskf::runtime::fuse_magnetometer(
                fixture.state, fixture.P, fixture.sample, fixture.reference, health, fixture.V, fixture.parameters,
                fixture.heading_observable, fixture.minimum_norm, fixture.sigma, output, output_P, fixture.bias_update,
                fixture.checks);
        }
        else
        {
            return formal_eskf::runtime::fuse_magnetometer(
                fixture.state, fixture.P, fixture.sample, fixture.reference, health, fixture.V, fixture.parameters,
                fixture.heading_observable, fixture.minimum_norm, fixture.sigma, output, output_P, fixture.checks);
        }
    };
    test.expect(fixture.correct(expected, expected_P) == Status::success, profile, "core comparison succeeds");
    auto const result = fuse();
    test.expect(result.fusion.decision == FusionDecision::fused && result.field.heading_checked &&
                    !result.field.failures.heading && same_state(output, expected) && same_bits(output_P, expected_P) &&
                    result.fusion.innovation(0U) == value_type{0} && result.fusion.innovation(1U) == value_type{0.5} &&
                    result.fusion.innovation(2U) == value_type{0},
                profile, "health declination does not drive heading checks, innovation or state/covariance correction");
    auto const before = output;
    auto const before_P = output_P;
    health = health * value_type{2};
    auto const rejected = fuse();
    test.expect(rejected.fusion.decision == FusionDecision::rejected && rejected.field.failures.strength &&
                    !rejected.fusion.diagnostics_valid && same_state(output, before) && same_bits(output_P, before_P),
                profile, "a matching observation reference cannot bypass a bad geographic strength check");
    health = Vector<Linalg>::from_row_major({value_type{0}, value_type{40}, value_type{30}});
    auto const inclined = fuse();
    test.expect(inclined.fusion.decision == FusionDecision::rejected && inclined.field.failures.inclination &&
                    !inclined.field.failures.strength && same_state(output, before) && same_bits(output_P, before_P),
                profile, "geographic inclination is independent of the observation reference");
    health(0U) = std::numeric_limits<value_type>::quiet_NaN();
    auto const invalid = fuse();
    test.expect(invalid.fusion.status == Status::non_finite_input && !invalid.field.diagnostics_valid &&
                    same_state(output, before) && same_bits(output_P, before_P),
                profile, "invalid consumed health reference fails without publishing state or covariance");
    fixture.checks.reference_available = false;
    auto const fallback = fuse();
    test.expect(fallback.fusion.decision == FusionDecision::fused && !fallback.field.inclination_checked &&
                    fallback.field.reference_strength == value_type{45} && same_state(output, expected),
                profile, "unavailable health reference is unused and retains the ordinary fallback");
    fixture.checks = {false, false, false, true};
    test.expect(fuse().fusion.decision == FusionDecision::fused, profile,
                "disabled physical checks do not consume an independent health vector");
    fixture.checks = {true, false, false, true};
    health = Vector<Linalg>::from_row_major({value_type{0}, value_type{0}, value_type{50}});
    test.expect(fuse().fusion.decision == FusionDecision::fused, profile,
                "a strength-only reference needs no heading information");
    health = Vector<Linalg>{};
    test.expect(fuse().fusion.status == Status::domain_error, profile, "zero consumed health reference fails");
    health(0U) = std::numeric_limits<value_type>::max();
    test.expect(fuse().fusion.status == Status::non_finite_result, profile, "health geometry overflow fails");
}

template <typename Linalg> void run_tests(TestContext & test, std::string_view profile)
{
    test_field_checks<Linalg>(test, profile);
    test_field_frames<Linalg>(test, profile);
    test_field_selection<Linalg>(test, profile);
    test_separate_references<Linalg, Ahrs>(test, profile);
    test_separate_references<Linalg, Ins>(test, profile);
    test_missing_reference<Linalg, Ahrs>(test, profile);
    test_missing_reference<Linalg, Ins>(test, profile);
    test_invalid_field<Linalg>(test, profile);
    test_fusion<Linalg, Ahrs>(test, profile);
    test_fusion<Linalg, Ins>(test, profile);
    test_rejection<Linalg, Ahrs>(test, profile);
    test_rejection<Linalg, Ins>(test, profile);
    test_fusion_boundaries<Linalg, Ahrs>(test, profile);
    test_fusion_boundaries<Linalg, Ins>(test, profile);
}

} /* end namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    TestContext test;
    run_tests<formal_eskf::test::Backend<float>>(test, "float");
    run_tests<formal_eskf::test::Backend<double>>(test, "double");
    if (test.failures() != 0)
    {
        return 1;
    }
    std::cout << "Magnetometer field checks and gated fusion: pass\n";
    return 0;
}
