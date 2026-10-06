/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// The public header must include its own dependencies.
#include <formal_eskf/eskf/innovation.hpp>

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <string_view>

#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{

using formal_eskf::Status;
using TestContext = formal_eskf::test::Context;

template <typename Vector> [[nodiscard]] bool same_bits(Vector const & a, Vector const & b)
{
    using bytes_type = std::array<std::byte, sizeof(typename Vector::value_type)>;
    for (std::size_t axis = 0U; axis < Vector::row_count; ++axis)
    {
        if (std::bit_cast<bytes_type>(a(axis)) != std::bit_cast<bytes_type>(b(axis)))
        {
            return false;
        }
    }
    return true;
}

template <typename Linalg, std::size_t Size> void run_conformance(TestContext & context, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using vector_type = formal_eskf::linalg::Matrix<Linalg, Size, 1U>;
    using check_type = formal_eskf::InnovationCheck<Linalg, Size>;
    using limits_type = std::numeric_limits<value_type>;

    vector_type innovation;
    vector_type variance;
    for (std::size_t axis = 0U; axis < Size; ++axis)
    {
        variance.set(axis, value_type{4});
    }
    check_type result;
    context.expect(formal_eskf::try_check_innovation(innovation, variance, value_type{2}, result) == Status::success &&
                       !result.rejected,
                   profile, "zero innovation passes");
    for (std::size_t axis = 0U; axis < Size; ++axis)
    {
        context.expect(result.test_ratio(axis) == value_type{0}, profile, "zero ratio on every axis");
    }

    // Check each axis independently, including the strict comparison at one.
    for (std::size_t axis = 0U; axis < Size; ++axis)
    {
        for (auto const sign : {value_type{-1}, value_type{1}})
        {
            for (auto const boundary : {std::nextafter(value_type{4}, value_type{0}), value_type{4},
                                        std::nextafter(value_type{4}, limits_type::infinity())})
            {
                innovation = vector_type::zero();
                innovation.set(axis, sign * boundary);
                auto const status = formal_eskf::try_check_innovation(innovation, variance, value_type{2}, result);
                context.expect(status == Status::success && result.rejected == (boundary > value_type{4}), profile,
                               "below/equal/above boundary, either residual sign");
                context.expect(result.test_ratio(axis) == (boundary * boundary) / value_type{16}, profile,
                               "boundary diagnostic uses the stated operation order");
            }
        }
    }

    // An early rejected axis must not prevent publishing later diagnostics.
    for (std::size_t axis = 0U; axis < Size; ++axis)
    {
        innovation.set(axis, axis == 0U ? value_type{8} : value_type{-2});
    }
    context.expect(formal_eskf::try_check_innovation(innovation, variance, value_type{2}, result) == Status::success &&
                       result.rejected,
                   profile, "one outlier rejects the group, not the calculation");
    for (std::size_t axis = 0U; axis < Size; ++axis)
    {
        context.expect(result.test_ratio(axis) == (axis == 0U ? value_type{4} : value_type{0.25}), profile,
                       "all ratios published after rejection");
    }
    innovation = vector_type::zero();
    innovation.set(0U, value_type{0.5});
    context.expect(formal_eskf::try_check_innovation(innovation, variance, value_type{0.25}, result) ==
                           Status::success &&
                       !result.rejected && result.test_ratio(0U) == value_type{1},
                   profile, "sub-unit sigma is not clamped; a fresh accepted call clears rejection");

    // Independent higher-precision normalized-residual oracle on moderate inputs.
    for (int sample = -16; sample <= 16; ++sample)
    {
        bool rejected = false;
        for (std::size_t axis = 0U; axis < Size; ++axis)
        {
            innovation.set(axis, static_cast<value_type>(sample) / value_type{3});
            variance.set(axis, static_cast<value_type>(axis + 1U) / value_type{2});
        }
        auto const status = formal_eskf::try_check_innovation(innovation, variance, value_type{2}, result);
        context.expect(status == Status::success, profile, "moderate-domain evaluation succeeds");
        for (std::size_t axis = 0U; axis < Size; ++axis)
        {
            auto const normalized = static_cast<long double>(innovation(axis)) /
                                    (2.0L * std::sqrt(static_cast<long double>(variance(axis))));
            auto const expected = normalized * normalized;
            auto const tolerance = 8.0L * static_cast<long double>(limits_type::epsilon()) * (1.0L + expected);
            context.expect(std::abs(static_cast<long double>(result.test_ratio(axis)) - expected) <= tolerance, profile,
                           "ratio agrees with normalized-residual oracle");
            rejected = rejected || std::abs(normalized) > 1.0L;
        }
        context.expect(result.rejected == rejected, profile, "group decision agrees with oracle away from boundary");
    }

    auto expect_failure = [&](vector_type const & residual, vector_type const & variances, value_type sigma,
                              Status expected, std::string_view description)
    {
        auto const old_residual = residual;
        auto const old_variances = variances;
        // Both Boolean states and unusual old values must survive failure.
        for (bool const old_rejected : {false, true})
        {
            check_type output;
            output.rejected = old_rejected;
            for (std::size_t axis = 0U; axis < Size; ++axis)
            {
                output.test_ratio.set(axis, axis == 0U ? -value_type{0} : limits_type::quiet_NaN());
            }
            auto const before = output;
            auto const status = formal_eskf::try_check_innovation(residual, variances, sigma, output);
            context.expect(status == expected, profile, description);
            context.expect(output.rejected == before.rejected && same_bits(output.test_ratio, before.test_ratio),
                           profile, "failure preserves all old diagnostics");
        }
        context.expect(same_bits(residual, old_residual) && same_bits(variances, old_variances), profile,
                       "inputs unchanged");
    };

    innovation = vector_type::zero();
    for (std::size_t axis = 0U; axis < Size; ++axis)
    {
        variance.set(axis, value_type{1});
    }
    for (auto const invalid : {limits_type::quiet_NaN(), limits_type::infinity(), -limits_type::infinity()})
    {
        expect_failure(innovation, variance, invalid, Status::non_finite_input, "non-finite sigma");
        for (std::size_t axis = 0U; axis < Size; ++axis)
        {
            auto residual = innovation;
            residual.set(0U, value_type{2});
            residual.set(axis, invalid);
            expect_failure(residual, variance, value_type{1}, Status::non_finite_input, "non-finite residual axis");
            auto variances = variance;
            variances.set(axis, invalid);
            expect_failure(innovation, variances, value_type{1}, Status::non_finite_input, "non-finite variance axis");
        }
    }
    for (auto const invalid : {value_type{0}, -value_type{0}, value_type{-1}})
    {
        expect_failure(innovation, variance, invalid, Status::domain_error, "nonpositive sigma");
        for (std::size_t axis = 0U; axis < Size; ++axis)
        {
            auto variances = variance;
            variances.set(axis, invalid);
            expect_failure(innovation, variances, value_type{1}, Status::domain_error, "nonpositive variance axis");
        }
    }
    expect_failure(innovation, variance, limits_type::max(), Status::non_finite_result, "squared sigma overflow");
    expect_failure(innovation, variance, limits_type::min(), Status::zero_or_unsafe_divisor, "squared sigma underflow");
    expect_failure(innovation, variance, std::sqrt(limits_type::min()) / value_type{2}, Status::zero_or_unsafe_divisor,
                   "subnormal squared sigma");
    for (std::size_t axis = 0U; axis < Size; ++axis)
    {
        auto residual = innovation;
        residual.set(0U, value_type{2});
        residual.set(axis, limits_type::max());
        expect_failure(residual, variance, value_type{1}, Status::non_finite_result, "squared residual overflow");
        auto variances = variance;
        variances.set(axis, limits_type::max());
        expect_failure(innovation, variances, value_type{2}, Status::non_finite_result, "denominator overflow");
        variances.set(axis, limits_type::min());
        expect_failure(innovation, variances, value_type{0.5}, Status::zero_or_unsafe_divisor, "subnormal denominator");
        expect_failure(innovation, variances, std::sqrt(limits_type::min()), Status::zero_or_unsafe_divisor,
                       "zero denominator");
        residual.set(axis, value_type{4});
        expect_failure(residual, variances, value_type{1}, Status::non_finite_result, "finite operands overflow ratio");
        variances.set(axis, limits_type::denorm_min());
        expect_failure(innovation, variances, value_type{1}, Status::zero_or_unsafe_divisor, "subnormal variance");
    }

    innovation.set(0U, std::sqrt(limits_type::min()));
    context.expect(formal_eskf::try_check_innovation(innovation, variance, std::sqrt(limits_type::min()), result) ==
                           Status::success &&
                       !result.rejected && result.test_ratio(0U) == value_type{1},
                   profile, "minimum normal squared sigma is inclusive");
    variance.set(0U, limits_type::min());
    context.expect(formal_eskf::try_check_innovation(innovation, variance, value_type{1}, result) == Status::success &&
                       !result.rejected && result.test_ratio(0U) == value_type{1},
                   profile, "minimum normal denominator is inclusive");
    innovation.set(0U, limits_type::min());
    context.expect(formal_eskf::try_check_innovation(innovation, variance, value_type{1}, result) == Status::success &&
                       !result.rejected && result.test_ratio(0U) == value_type{0},
                   profile, "tiny innovation may square to zero");

    for (bool const alias_variance : {false, true})
    {
        for (bool const fail : {false, true})
        {
            result.test_ratio = alias_variance ? variance : innovation;
            auto const before = result;
            auto const & residual = alias_variance ? innovation : result.test_ratio;
            auto const & variances = alias_variance ? result.test_ratio : variance;
            check_type expected;
            auto const sigma = fail ? value_type{0} : value_type{1};
            auto const expected_status = formal_eskf::try_check_innovation(residual, variances, sigma, expected);
            auto const status = formal_eskf::try_check_innovation(residual, variances, sigma, result);
            auto const & expected_output = fail ? before : expected;
            context.expect(status == expected_status && result.rejected == expected_output.rejected &&
                               same_bits(result.test_ratio, expected_output.test_ratio),
                           profile, "complete input/output alias preserves success and failure semantics");
        }
    }
}

} /* end anonymous namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    TestContext context;
    run_conformance<formal_eskf::test::Backend<float>, 1U>(context, "float/1");
    run_conformance<formal_eskf::test::Backend<float>, 2U>(context, "float/2");
    run_conformance<formal_eskf::test::Backend<float>, 3U>(context, "float/3");
    run_conformance<formal_eskf::test::Backend<double>, 1U>(context, "double/1");
    run_conformance<formal_eskf::test::Backend<double>, 2U>(context, "double/2");
    run_conformance<formal_eskf::test::Backend<double>, 3U>(context, "double/3");
    if (context.failures() != 0)
    {
        std::cerr << context.failures() << " innovation check(s) failed\n";
        return 1;
    }
    std::cout << "Innovation checks passed\n";
    return 0;
}
