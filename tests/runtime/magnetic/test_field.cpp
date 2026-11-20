/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Include standalone quality API first to check its own dependencies.
#include <formal_eskf/runtime/magnetic_field.hpp>

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <limits>
#include <string_view>
#include <type_traits>

#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{

using formal_eskf::Status;
using formal_eskf::runtime::check_magnetic_field;
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

template <typename Scalar> bool invalid_diagnostics(formal_eskf::runtime::MagneticFieldCheck<Scalar> const & result)
{
    return !result.accepted && !result.diagnostics_valid && !result.strength_checked && !result.inclination_checked &&
           !result.heading_checked && !result.failures.reference && !result.failures.strength &&
           !result.failures.inclination && !result.failures.horizontal_field && !result.failures.heading &&
           result.strength == Scalar{0} && result.reference_strength == Scalar{0} &&
           result.inclination_error == Scalar{0} && result.heading_error == Scalar{0};
}

template <typename Linalg> void test_field_checks(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using vector_type = Vector<Linalg>;
    formal_eskf::so3::UnitQuaternion<Linalg> const identity;
    auto parameters = field_parameters<Linalg>();
    auto const reference = vector_type::from_row_major({value_type{30}, value_type{0}, value_type{40}});
    auto result = check_magnetic_field<Linalg>(identity, reference, reference, &reference, parameters, true);
    test.expect(result.status == Status::success && result.accepted && result.diagnostics_valid &&
                    result.heading_checked && result.strength == value_type{50} &&
                    result.reference_strength == value_type{50} && result.heading_error == value_type{0} &&
                    result.inclination_error == value_type{0},
                profile, "matching calibrated field passes all checks");

    auto sample = reference * value_type{2};
    result = check_magnetic_field<Linalg>(identity, sample, reference, &reference, parameters, true);
    test.expect(result.status == Status::success && !result.accepted && result.failures.strength &&
                    !result.failures.inclination && !result.failures.heading,
                profile, "same direction with excessive strength is rejected");
    sample = vector_type::from_row_major({value_type{40}, value_type{0}, value_type{30}});
    result = check_magnetic_field<Linalg>(identity, sample, reference, &reference, parameters, true);
    test.expect(!result.accepted && !result.failures.strength && result.failures.inclination &&
                    !result.failures.heading,
                profile, "same strength but different inclination is rejected");
    sample = vector_type::from_row_major({value_type{0}, value_type{30}, value_type{40}});
    result = check_magnetic_field<Linalg>(identity, sample, reference, &reference, parameters, true);
    test.expect(!result.accepted && !result.failures.strength && !result.failures.inclination &&
                    result.failures.heading && result.heading_checked,
                profile, "horizontal-direction disturbance needs an independent heading premise");
    result = check_magnetic_field<Linalg>(identity, sample, reference, &reference, parameters, false);
    test.expect(result.accepted && !result.heading_checked && result.heading_error == value_type{0}, profile,
                "unobservable heading skips only the heading consistency check");

    auto const vertical = vector_type::from_row_major({value_type{0}, value_type{0}, value_type{50}});
    result = check_magnetic_field<Linalg>(identity, vertical, vertical, &vertical, parameters, true);
    test.expect(result.status == Status::success && result.diagnostics_valid && !result.accepted &&
                    result.failures.horizontal_field && !result.heading_checked,
                profile, "vertical fields have valid inclination but no usable heading");
    for (bool weak_reference : {false, true})
    {
        result = check_magnetic_field<Linalg>(identity, weak_reference ? reference : vertical,
                                              weak_reference ? vertical : reference,
                                              &(weak_reference ? vertical : reference), parameters, false);
        test.expect(result.failures.horizontal_field && !result.accepted, profile,
                    "either weak horizontal field rejects, even when heading consistency is skipped");
    }

    auto const west_n = vector_type::from_row_major({value_type{-40}, value_type{0.5}, value_type{10}});
    auto const west_b = vector_type::from_row_major({value_type{-40}, value_type{-0.5}, value_type{10}});
    for (bool reverse : {false, true})
    {
        result = check_magnetic_field<Linalg>(identity, reverse ? west_n : west_b, reverse ? west_b : west_n,
                                              &(reverse ? west_b : west_n), parameters, true);
        long double const expected = (reverse ? -2.0L : 2.0L) * std::atan2(0.5L, 40.0L);
        test.expect(result.accepted && std::abs(static_cast<long double>(result.heading_error) - expected) <
                                           32 * std::numeric_limits<value_type>::epsilon(),
                    profile, "heading wraps across both sides of the +/-pi boundary");
    }

    auto const north = vector_type::from_row_major({value_type{5}, value_type{0}, value_type{0}});
    auto const stronger = north * value_type{2};
    result = check_magnetic_field<Linalg>(identity, stronger, north, &north, parameters, true);
    test.expect(result.accepted, profile, "strength tolerance equality passes");
    parameters.strength_tolerance = std::nextafter(value_type{5}, value_type{0});
    test.expect(check_magnetic_field<Linalg>(identity, stronger, north, &north, parameters, true).failures.strength,
                profile, "just beyond strength tolerance rejects");
    parameters = field_parameters<Linalg>();
    parameters.minimum_horizontal_field = value_type{5};
    test.expect(check_magnetic_field<Linalg>(identity, north, north, &north, parameters, true).accepted, profile,
                "minimum horizontal equality passes");
    parameters.minimum_horizontal_field = std::nextafter(value_type{5}, value_type{6});
    test.expect(
        check_magnetic_field<Linalg>(identity, north, north, &north, parameters, true).failures.horizontal_field,
        profile, "just below minimum horizontal rejects");

    parameters = field_parameters<Linalg>();
    sample = vector_type::from_row_major({value_type{0}, value_type{5}, value_type{0}});
    result = check_magnetic_field<Linalg>(identity, sample, north, &north, parameters, true);
    parameters.heading_tolerance = std::abs(result.heading_error);
    test.expect(check_magnetic_field<Linalg>(identity, sample, north, &north, parameters, true).accepted, profile,
                "computed heading tolerance equality passes");
    parameters.heading_tolerance = std::nextafter(parameters.heading_tolerance, value_type{0});
    test.expect(check_magnetic_field<Linalg>(identity, sample, north, &north, parameters, true).failures.heading,
                profile, "computed heading just beyond tolerance rejects");
    parameters = field_parameters<Linalg>();
    sample = vector_type::from_row_major({value_type{3}, value_type{0}, value_type{4}});
    result = check_magnetic_field<Linalg>(identity, sample, north, &north, parameters, true);
    parameters.inclination_tolerance = std::abs(result.inclination_error);
    test.expect(check_magnetic_field<Linalg>(identity, sample, north, &north, parameters, true).accepted, profile,
                "computed inclination tolerance equality passes");
    parameters.inclination_tolerance = std::nextafter(parameters.inclination_tolerance, value_type{0});
    test.expect(check_magnetic_field<Linalg>(identity, sample, north, &north, parameters, true).failures.inclination,
                profile, "computed inclination just beyond tolerance rejects");
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
                                                                   (mask & 4U) != 0U};
            for (std::size_t sample = 0U; sample < samples.size(); ++sample)
            {
                auto const result = check_magnetic_field<Linalg>(
                    identity, samples[sample], reference, available ? &reference : nullptr, parameters, false, checks);
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
    formal_eskf::runtime::MagneticFieldChecks const fallback{true, true, false};
    for (value_type magnitude : {value_type{5}, value_type{85}})
    {
        auto const sample = vector_type::from_row_major({magnitude, value_type{0}, value_type{0}});
        auto const result =
            check_magnetic_field<Linalg>(identity, sample, reference, nullptr, parameters, false, fallback);
        test.expect(result.accepted && result.strength_checked && !result.inclination_checked &&
                        result.reference_strength == value_type{45},
                    profile, "without a reference the inclusive 5 to 85 microtesla range applies");
    }
    for (value_type magnitude :
         {std::nextafter(value_type{5}, value_type{0}), std::nextafter(value_type{85}, value_type{100})})
    {
        auto const sample = vector_type::from_row_major({magnitude, value_type{0}, value_type{0}});
        test.expect(check_magnetic_field<Linalg>(identity, sample, reference, nullptr, parameters, false, fallback)
                        .failures.strength,
                    profile, "just outside the fallback range rejects independently of configured tolerance");
    }
    auto const vertical = vector_type::from_row_major({value_type{0}, value_type{0}, value_type{50}});
    formal_eskf::runtime::MagneticFieldChecks const disabled{false, false, false};
    test.expect(check_magnetic_field<Linalg>(identity, vertical, reference, nullptr, parameters, false, disabled)
                    .failures.horizontal_field,
                profile, "disabling physical checks retains the separate weak-heading guard");
    auto const east = vector_type::from_row_major({value_type{0}, value_type{30}, value_type{40}});
    test.expect(
        check_magnetic_field<Linalg>(identity, east, reference, nullptr, parameters, true, disabled).failures.heading,
        profile, "field selection does not disable the separately requested heading check");
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
            quaternion_type::from_coefficients(
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
        auto const result =
            check_magnetic_field<Linalg>(q, measured, reference, &reference, field_parameters<Linalg>(), true);
        test.expect(result.accepted &&
                        std::abs(result.inclination_error) <
                            value_type{128} * std::numeric_limits<value_type>::epsilon() &&
                        std::abs(result.heading_error) < value_type{128} * std::numeric_limits<value_type>::epsilon() &&
                        std::abs(static_cast<long double>(result.strength) - std::sqrt(2369.0L)) <
                            128 * std::numeric_limits<value_type>::epsilon() * 50,
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
        test.expect(result.status == expected && invalid_diagnostics(result), profile,
                    "invalid input/arithmetic fails closed without partially published diagnostics");
    };
    for (value_type invalid : {limits_type::quiet_NaN(), limits_type::infinity(), -limits_type::infinity()})
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            auto bad = reference;
            bad(axis) = invalid;
            expect_failure(check_magnetic_field<Linalg>(identity, bad, reference, &reference, parameters, true),
                           Status::non_finite_input);
            expect_failure(check_magnetic_field<Linalg>(identity, reference, bad, &bad, parameters, true),
                           Status::non_finite_input);
        }
        for (std::size_t index = 0U; index < 4U; ++index)
        {
            auto bad = parameters;
            std::array<value_type *, 4U> fields{&bad.strength_tolerance, &bad.inclination_tolerance,
                                                &bad.heading_tolerance, &bad.minimum_horizontal_field};
            *fields[index] = invalid;
            expect_failure(check_magnetic_field<Linalg>(identity, reference, reference, &reference, bad, false),
                           Status::non_finite_input);
        }
    }
    for (std::size_t index = 0U; index < 4U; ++index)
    {
        auto bad = parameters;
        std::array<value_type *, 4U> fields{&bad.strength_tolerance, &bad.inclination_tolerance, &bad.heading_tolerance,
                                            &bad.minimum_horizontal_field};
        *fields[index] = value_type{-1};
        expect_failure(check_magnetic_field<Linalg>(identity, reference, reference, &reference, bad, true),
                       Status::domain_error);
    }
    auto bad_parameters = parameters;
    bad_parameters.inclination_tolerance = value_type{4};
    expect_failure(check_magnetic_field<Linalg>(identity, reference, reference, &reference, bad_parameters, true),
                   Status::domain_error);
    bad_parameters = parameters;
    bad_parameters.heading_tolerance = value_type{4};
    expect_failure(check_magnetic_field<Linalg>(identity, reference, reference, &reference, bad_parameters, false),
                   Status::domain_error);
    expect_failure(check_magnetic_field<Linalg>(identity, reference, reference, &reference, {}, true),
                   Status::domain_error);
    expect_failure(check_magnetic_field<Linalg>(identity, Vector<Linalg>{}, reference, &reference, parameters, true),
                   Status::domain_error);
    expect_failure(check_magnetic_field<Linalg>(identity, reference, Vector<Linalg>{}, nullptr, parameters, true),
                   Status::domain_error);
    auto huge = reference;
    huge(2U) = limits_type::max();
    expect_failure(check_magnetic_field<Linalg>(identity, huge, reference, &reference, parameters, true),
                   Status::non_finite_result);
    expect_failure(check_magnetic_field<Linalg>(identity, reference, huge, &huge, parameters, true),
                   Status::non_finite_result);
    auto const tiny = Vector<Linalg>::from_row_major({limits_type::min(), value_type{0}, value_type{0}});
    expect_failure(check_magnetic_field<Linalg>(identity, tiny, reference, &reference, parameters, true),
                   Status::zero_or_unsafe_divisor);
    expect_failure(check_magnetic_field<Linalg>(identity, reference, tiny, &tiny, parameters, true),
                   Status::zero_or_unsafe_divisor);
}

template <typename Backend> void geometry(TestContext & test, std::string_view profile)
{
    test_field_frames<Backend>(test, profile);
    using Scalar = typename Backend::value_type;
    using Q = formal_eskf::so3::UnitQuaternion<Backend>;
    auto const observation = Vector<Backend>::from_row_major({Scalar{30}, Scalar{0}, Scalar{40}});
    auto const body = Vector<Backend>::from_row_major({Scalar{0}, Scalar{40}, Scalar{30}});
    Q q;
    test.expect(Q::from_coefficients(Scalar{0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.125}, q) ==
                    Status::success,
                profile, "exact cyclic attitude fixture");
    auto result = check_magnetic_field(q, body, observation, &observation, field_parameters<Backend>(), true);
    test.expect(result.accepted && result.strength == Scalar{50} && result.heading_error == Scalar{0} &&
                    result.inclination_error == Scalar{0},
                profile, "exact cyclic body-to-navigation mapping has independently known coefficients");
    auto parameters = field_parameters<Backend>();
    parameters.heading_tolerance = std::numbers::pi_v<Scalar>;
    parameters.inclination_tolerance = std::numbers::pi_v<Scalar>;
    auto const north = Vector<Backend>::from_row_major({Scalar{30}, Scalar{0}, Scalar{0}});
    for (int sign : {-1, 1})
    {
        auto const east = Vector<Backend>::from_row_major({Scalar{0}, static_cast<Scalar>(30 * sign), Scalar{0}});
        result = check_magnetic_field(Q{}, east, north, &north, parameters, true);
        long double const expected = sign * std::numbers::pi_v<long double> / 2;
        test.expect(result.accepted && std::abs(static_cast<long double>(result.heading_error) - expected) <=
                                           8 * std::numeric_limits<Scalar>::epsilon(),
                    profile, "East heading has measured-minus-reference sign independently of quaternion helpers");
        auto const dipped = Vector<Backend>::from_row_major({Scalar{18}, Scalar{0}, static_cast<Scalar>(24 * sign)});
        result = check_magnetic_field(Q{}, dipped, north, &north, parameters, true);
        test.expect(result.accepted && std::abs(static_cast<long double>(result.inclination_error) -
                                                std::atan2(static_cast<long double>(24 * sign), 18.0L)) <=
                                           8 * std::numeric_limits<Scalar>::epsilon(),
                    profile, "positive Down raises inclination, negative Down lowers it");
    }
    // The non-axis fixtures above are fixed-domain geometric regressions; their
    // wider-reference allowances are not universal libm/rotation error bounds.
}

template <typename Backend> void selection(TestContext & test, std::string_view profile)
{
    test_field_selection<Backend>(test, profile);
    using Scalar = typename Backend::value_type;
    using Q = formal_eskf::so3::UnitQuaternion<Backend>;
    auto const observation = Vector<Backend>::from_row_major({Scalar{30}, Scalar{0}, Scalar{40}});
    auto const body = observation;
    auto const health = observation;
    for (unsigned mask = 0; mask < 8; ++mask)
    {
        formal_eskf::runtime::MagneticFieldChecks const checks{(mask & 1U) != 0, (mask & 2U) != 0, (mask & 4U) != 0};
        for (auto pointer : {&observation, &body, &health})
        {
            auto const result =
                check_magnetic_field(Q{}, body, observation, pointer, field_parameters<Backend>(), true, checks);
            test.expect(result.accepted && result.strength_checked == checks.strength &&
                            result.inclination_checked == checks.inclination &&
                            result.reference_strength ==
                                ((checks.strength || checks.inclination) ? Scalar{50} : Scalar{0}),
                        profile, "all legal health-pointer arrangements preserve selected roles and placeholders");
        }
    }
    auto invalid = health;
    invalid(2U) = std::numeric_limits<Scalar>::quiet_NaN();
    auto const result =
        check_magnetic_field(Q{}, body, observation, &invalid, field_parameters<Backend>(), true, {false, false, true});
    test.expect(result.accepted && result.reference_strength == Scalar{0} && !result.failures.reference, profile,
                "unselected health coefficients are not consumed, even when reference requested");
    auto const vertical_health = Vector<Backend>::from_row_major({Scalar{0}, Scalar{0}, Scalar{50}});
    auto const vertical =
        check_magnetic_field(Q{}, body, observation, &vertical_health, field_parameters<Backend>(), true);
    test.expect(!vertical.accepted && vertical.failures.inclination && !vertical.failures.horizontal_field &&
                    vertical.heading_checked && vertical.heading_error == Scalar{0},
                profile, "vertical health reference affects dip qualification but not observation heading geometry");
}

template <typename Backend> void validation(TestContext & test, std::string_view profile)
{
    test_invalid_field<Backend>(test, profile);
    using Scalar = typename Backend::value_type;
    using Q = formal_eskf::so3::UnitQuaternion<Backend>;
    auto const observation = Vector<Backend>::from_row_major({Scalar{30}, -Scalar{0}, Scalar{40}});
    auto const before = observation;
    for (auto value : {Scalar{0}, std::numeric_limits<Scalar>::min(), std::numeric_limits<Scalar>::max(),
                       std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity()})
    {
        auto health = Vector<Backend>::from_row_major({value, Scalar{0}, Scalar{0}});
        auto const health_before = health;
        auto const result =
            check_magnetic_field(Q{}, observation, observation, &health, field_parameters<Backend>(), true);
        Status const expected = !std::isfinite(value)                         ? Status::non_finite_input
                                : value == Scalar{0}                          ? Status::domain_error
                                : value == std::numeric_limits<Scalar>::min() ? Status::zero_or_unsafe_divisor
                                                                              : Status::non_finite_result;
        test.expect(result.status == expected && !result.accepted && !result.diagnostics_valid &&
                        result.strength == Scalar{0} && result.reference_strength == Scalar{0} &&
                        same_bits(health, health_before) && same_bits(observation, before),
                    profile, "independently supplied consumed health failures preserve every borrowed coefficient");
    }
    Scalar const tiny =
        std::ldexp(Scalar{1}, (std::numeric_limits<Scalar>::min_exponent - std::numeric_limits<Scalar>::digits) / 2);
    Q growing;
    test.expect(Q::from_coefficients(tiny, tiny / Scalar{2}, Scalar{0}, Scalar{0}, std::numeric_limits<Scalar>::min(),
                                     growing) == Status::success,
                profile, "public gradual-underflow constructor produces guard witness");
    for (unsigned step = 0;
         step < 32 && std::isfinite(formal_eskf::linalg::dot(growing.coefficients(), growing.coefficients())); ++step)
    {
        growing = growing * growing;
    }
    test.expect(formal_eskf::linalg::all_finite(growing.coefficients()), profile,
                "finite malformed coefficients reach rotation arithmetic overflow without layout mutation");
    auto result =
        check_magnetic_field(growing, observation, observation, &observation, field_parameters<Backend>(), true);
    test.expect(result.status == Status::non_finite_result && invalid_diagnostics(result), profile,
                "rotation overflow does not publish partial field geometry");
    for (unsigned step = 0; step < 4 && formal_eskf::linalg::all_finite(growing.coefficients()); ++step)
    {
        growing = growing * growing;
    }
    result = check_magnetic_field(growing, observation, observation, &observation, field_parameters<Backend>(), false);
    test.expect(result.status == Status::non_finite_input && invalid_diagnostics(result) &&
                    same_bits(observation, before),
                profile, "nonfinite public quaternion witness fails before rotation");
    auto parameters = field_parameters<Backend>();
    parameters.minimum_horizontal_field = std::nextafter(std::numeric_limits<Scalar>::min(), Scalar{0});
    result = check_magnetic_field(Q{}, observation, observation, &observation, parameters, true);
    test.expect(result.status == Status::domain_error && invalid_diagnostics(result), profile,
                "positive subnormal horizontal threshold is outside the positive-normal domain");
    parameters.minimum_horizontal_field = std::numeric_limits<Scalar>::min();
    parameters.strength_tolerance = Scalar{0};
    parameters.heading_tolerance = Scalar{0};
    parameters.inclination_tolerance = Scalar{0};
    test.expect(check_magnetic_field(Q{}, observation, observation, &observation, parameters, true).accepted, profile,
                "minimum-normal horizontal bound and zero physical tolerances are legal on exactly matching data");
    for (bool heading : {false, true})
    {
        parameters = field_parameters<Backend>();
        auto & angle = heading ? parameters.heading_tolerance : parameters.inclination_tolerance;
        angle = std::numbers::pi_v<Scalar>;
        test.expect(check_magnetic_field(Q{}, observation, observation, &observation, parameters, true).accepted,
                    profile, "encoded pi is a legal inclusive angle tolerance");
        angle = std::nextafter(angle, std::numeric_limits<Scalar>::infinity());
        result = check_magnetic_field(Q{}, observation, observation, &observation, parameters, true);
        test.expect(result.status == Status::domain_error && invalid_diagnostics(result), profile,
                    "one represented angle step above pi is invalid");
    }
    enum class Scenario
    {
        accepted,
        strength_rejected,
        invalid_health
    };
    for (auto scenario : {Scenario::accepted, Scenario::strength_rejected, Scenario::invalid_health})
    {
        auto body = observation;
        auto health = observation;
        Q attitude;
        auto config = field_parameters<Backend>();
        Status expected_status = Status::success;
        switch (scenario)
        {
        case Scenario::accepted:
            break;
        case Scenario::strength_rejected:
            body = body * Scalar{2};
            break;
        case Scenario::invalid_health:
            health(1) = std::numeric_limits<Scalar>::quiet_NaN();
            expected_status = Status::non_finite_input;
            break;
        }
        auto const body_before = body, health_before = health;
        auto const attitude_before = attitude.coefficients();
        std::array<Scalar, 4> const config_before{config.strength_tolerance, config.inclination_tolerance,
                                                  config.heading_tolerance, config.minimum_horizontal_field};
        auto const verdict = check_magnetic_field(attitude, body, observation, &health, config, true);
        std::array<Scalar, 4> const config_after{config.strength_tolerance, config.inclination_tolerance,
                                                 config.heading_tolerance, config.minimum_horizontal_field};
        test.expect(verdict.status == expected_status && verdict.accepted == (scenario == Scenario::accepted) &&
                        same_bits(body, body_before) && same_bits(health, health_before) &&
                        same_bits(attitude.coefficients(), attitude_before) && same_bits(observation, before) &&
                        std::bit_cast<std::array<std::byte, sizeof(config_before)>>(config_before) ==
                            std::bit_cast<std::array<std::byte, sizeof(config_after)>>(config_after),
                    profile,
                    "success/rejection/failure all preserve complete quaternion, parameters and borrowed field data");
    }
    // Valid StandardMath sqrt/atan2 arguments do not provide a native provider
    // failure witness. Provider propagation is a scalar/formal contract boundary.
}

template <typename Backend> void rejection(TestContext & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    using Q = formal_eskf::so3::UnitQuaternion<Backend>;
    auto const observation = Vector<Backend>::from_row_major({Scalar{30}, Scalar{0}, Scalar{40}});
    std::array<Vector<Backend>, 5> const samples{observation * Scalar{2},
                                                 Vector<Backend>::from_row_major({Scalar{40}, Scalar{0}, Scalar{30}}),
                                                 Vector<Backend>::from_row_major({Scalar{0}, Scalar{30}, Scalar{40}}),
                                                 Vector<Backend>::from_row_major({Scalar{0}, Scalar{0}, Scalar{50}}),
                                                 Vector<Backend>::from_row_major({Scalar{0}, Scalar{80}, Scalar{60}})};
    for (std::size_t index = 0; index < samples.size(); ++index)
    {
        auto const result =
            check_magnetic_field(Q{}, samples[index], observation, &observation, field_parameters<Backend>(), true);
        test.expect(result.status == Status::success && result.diagnostics_valid && !result.accepted &&
                        !result.failures.reference && result.failures.strength == (index == 0 || index == 4) &&
                        result.failures.inclination == (index == 1 || index == 3 || index == 4) &&
                        result.failures.heading == (index == 2 || index == 4) &&
                        result.failures.horizontal_field == (index == 3),
                    profile, "independent disturbance table distinguishes physical flags from numerical errors");
        auto const recovered =
            check_magnetic_field(Q{}, observation, observation, &observation, field_parameters<Backend>(), true);
        test.expect(recovered.status == Status::success && recovered.accepted && recovered.diagnostics_valid &&
                        !recovered.failures.reference && !recovered.failures.strength &&
                        !recovered.failures.inclination && !recovered.failures.heading &&
                        !recovered.failures.horizontal_field,
                    profile, "stateless successful call inherits no rejected-call flags");
    }
    auto const missing =
        check_magnetic_field(Q{}, observation, observation, static_cast<Vector<Backend> const *>(nullptr),
                             field_parameters<Backend>(), true, {true, true, true});
    test.expect(missing.status == Status::success && missing.diagnostics_valid && !missing.accepted &&
                    missing.failures.reference && !missing.failures.strength && !missing.failures.inclination,
                profile, "missing selected required reference is a finite rejection, not a numerical error");
}

template <typename Backend> void representation(TestContext & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    using Q = formal_eskf::so3::UnitQuaternion<Backend>;
    auto const body = Vector<Backend>::from_row_major({Scalar{0}, Scalar{40}, Scalar{30}});
    auto const observation = Vector<Backend>::from_row_major({Scalar{30}, Scalar{0}, Scalar{40}});
    auto const rotated_observation = Vector<Backend>::from_row_major({Scalar{-30}, Scalar{0}, Scalar{40}});
    Q q, negative, yawed;
    test.expect(
        Q::from_coefficients(Scalar{0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.125}, q) == Status::success &&
            Q::from_coefficients(Scalar{-0.5}, Scalar{-0.5}, Scalar{-0.5}, Scalar{-0.5}, Scalar{0.125}, negative) ==
                Status::success &&
            Q::from_coefficients(Scalar{-0.5}, Scalar{-0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.125}, yawed) ==
                Status::success,
        profile, "independently derived cyclic and navigation-yaw-composed coefficients");
    for (auto const & attitude : {q, negative, yawed})
    {
        auto const & reference =
            attitude.q1() < Scalar{0} && attitude.q2() > Scalar{0} ? rotated_observation : observation;
        auto const result =
            check_magnetic_field(attitude, body, reference, &reference, field_parameters<Backend>(), true);
        test.expect(result.accepted && result.strength == Scalar{50} && result.heading_error == Scalar{0} &&
                        result.inclination_error == Scalar{0},
                    profile, "q sign and consistent navigation yaw preserve exact geometric plausibility");
    }
    auto const alternate = Vector<Backend>::from_row_major({Scalar{40}, Scalar{0}, Scalar{30}});
    auto const changed = check_magnetic_field(q, body, observation, &alternate, field_parameters<Backend>(), true);
    test.expect(changed.status == Status::success && !changed.accepted && changed.failures.inclination &&
                    changed.heading_checked && changed.heading_error == Scalar{0} && changed.strength == Scalar{50},
                profile, "health-only geometry can change qualification without replacing observation heading");
}
} /* end namespace */

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Case;
    using Float = formal_eskf::test::Backend<float>;
    using Double = formal_eskf::test::Backend<double>;
    Case const cases[] = {
        {"TC-R-MAGNETIC-FIELD-01", "float", geometry<Float>},
        {"TC-R-MAGNETIC-FIELD-01", "double", geometry<Double>},
        {"TC-R-MAGNETIC-FIELD-02", "float", selection<Float>},
        {"TC-R-MAGNETIC-FIELD-02", "double", selection<Double>},
        {"TC-R-MAGNETIC-FIELD-03", "float", test_field_checks<Float>},
        {"TC-R-MAGNETIC-FIELD-03", "double", test_field_checks<Double>},
        {"TC-R-MAGNETIC-FIELD-04", "float", validation<Float>},
        {"TC-R-MAGNETIC-FIELD-04", "double", validation<Double>},
        {"TC-R-MAGNETIC-FIELD-05", "float", rejection<Float>},
        {"TC-R-MAGNETIC-FIELD-05", "double", rejection<Double>},
        {"TC-R-MAGNETIC-FIELD-06", "float", representation<Float>},
        {"TC-R-MAGNETIC-FIELD-06", "double", representation<Double>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
