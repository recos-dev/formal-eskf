/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// The public header must include its own dependencies.
#include <formal_eskf/eskf/innovation.hpp>
#include <formal_eskf/eskf/correction.hpp>

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
#include "test_cases.hpp"

namespace
{

using formal_eskf::Status;
using TestContext = formal_eskf::test::Context;

template <typename Vector> bool same_bits(Vector const & a, Vector const & b)
{
    using bytes_type = std::array<std::byte, sizeof(typename Vector::value_type)>;
    for (std::size_t axis = 0U; axis < Vector::row_count; ++axis)
    {
        if (std::isnan(a(axis)) && std::isnan(b(axis)))
        {
            continue; // NaN payloads are not a preservation contract.
        }
        if (std::bit_cast<bytes_type>(a(axis)) != std::bit_cast<bytes_type>(b(axis)))
        {
            return false;
        }
    }
    return true;
}

template <typename Linalg, std::size_t Size> void test_ratio_decision(TestContext & context, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using vector_type = formal_eskf::linalg::Vector<Linalg, Size>;
    using check_type = formal_eskf::InnovationCheck<Linalg, Size>;
    using limits_type = std::numeric_limits<value_type>;

    vector_type innovation;
    vector_type variance;
    for (std::size_t axis = 0U; axis < Size; ++axis)
    {
        variance(axis) = value_type{4};
    }
    check_type result;
    context.expect(formal_eskf::check_innovation(innovation, variance, value_type{2}, result) == Status::success &&
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
                innovation(axis) = sign * boundary;
                auto const status = formal_eskf::check_innovation(innovation, variance, value_type{2}, result);
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
        innovation(axis) = axis == 0U ? value_type{8} : value_type{-2};
    }
    context.expect(formal_eskf::check_innovation(innovation, variance, value_type{2}, result) == Status::success &&
                       result.rejected,
                   profile, "one outlier rejects the group, not the calculation");
    for (std::size_t axis = 0U; axis < Size; ++axis)
    {
        context.expect(result.test_ratio(axis) == (axis == 0U ? value_type{4} : value_type{0.25}), profile,
                       "all ratios published after rejection");
    }
    innovation = vector_type::zero();
    innovation(0U) = value_type{0.5};
    context.expect(formal_eskf::check_innovation(innovation, variance, value_type{0.25}, result) == Status::success &&
                       !result.rejected && result.test_ratio(0U) == value_type{1},
                   profile, "sub-unit sigma is not clamped; a fresh accepted call clears rejection");

    // Independent higher-precision normalized-residual oracle on moderate inputs.
    for (int sample = -16; sample <= 16; ++sample)
    {
        bool rejected = false;
        for (std::size_t axis = 0U; axis < Size; ++axis)
        {
            innovation(axis) = static_cast<value_type>(sample) / value_type{3};
            variance(axis) = static_cast<value_type>(axis + 1U) / value_type{2};
        }
        auto const status = formal_eskf::check_innovation(innovation, variance, value_type{2}, result);
        context.expect(status == Status::success, profile, "moderate-domain evaluation succeeds");
        for (std::size_t axis = 0U; axis < Size; ++axis)
        {
            // Three normal arithmetic operations in the production expression;
            // the wider reference has three too. 8*epsilon covers both gamma_3
            // rounding bounds on this finite moderate-domain fixture only.
            static_assert(std::numeric_limits<long double>::digits > limits_type::digits);
            auto const r = static_cast<long double>(innovation(axis));
            auto const expected = (r * r) / (4.0L * static_cast<long double>(variance(axis)));
            auto const tolerance = 8.0L * static_cast<long double>(limits_type::epsilon()) * (1.0L + expected);
            context.expect(std::abs(static_cast<long double>(result.test_ratio(axis)) - expected) <= tolerance, profile,
                           "ratio agrees with normalized-residual oracle");
            rejected = rejected || expected > 1.0L;
        }
        context.expect(result.rejected == rejected, profile, "group decision agrees with oracle away from boundary");
    }

    if constexpr (Size > 1U)
    {
        for (std::size_t axis = 0U; axis < Size; ++axis)
        {
            innovation(axis) = value_type{1};
            variance(axis) = value_type{2};
        }
        context.expect(formal_eskf::check_innovation(innovation, variance, value_type{1}, result) == Status::success &&
                           !result.rejected,
                       profile, "passing component ratios do not become a rejected joint sum");
        for (std::size_t axis = 0U; axis < Size; ++axis)
        {
            context.expect(result.test_ratio(axis) == value_type{0.5}, profile,
                           "independent exact component ratio remains one half");
        }
    }
}

template <typename Linalg, std::size_t Size>
void expect_failure(TestContext & context, std::string_view profile,
                    formal_eskf::linalg::Vector<Linalg, Size> const & residual,
                    formal_eskf::linalg::Vector<Linalg, Size> const & variances, typename Linalg::value_type sigma,
                    Status expected, std::string_view description)
{
    using value_type = typename Linalg::value_type;
    using check_type = formal_eskf::InnovationCheck<Linalg, Size>;
    using limits_type = std::numeric_limits<value_type>;
    auto const old_residual = residual;
    auto const old_variances = variances;
    // Both Boolean states and unusual old values must survive failure.
    for (bool const old_rejected : {false, true})
    {
        // Separate output, residual alias, and variance alias. This also
        // exercises failures after earlier candidate ratios were computed.
        for (unsigned alias = 0U; alias < 3U; ++alias)
        {
            check_type output;
            output.rejected = old_rejected;
            for (std::size_t axis = 0U; axis < Size; ++axis)
            {
                output.test_ratio(axis) = axis == 0U ? -value_type{0} : limits_type::quiet_NaN();
            }
            if (alias == 1U)
            {
                output.test_ratio = residual;
            }
            else if (alias == 2U)
            {
                output.test_ratio = variances;
            }
            auto const before = output;
            auto const & r = alias == 1U ? output.test_ratio : residual;
            auto const & S_diagonal = alias == 2U ? output.test_ratio : variances;
            auto const status = formal_eskf::check_innovation(r, S_diagonal, sigma, output);
            context.expect(status == expected, profile, description);
            context.expect(output.rejected == before.rejected && same_bits(output.test_ratio, before.test_ratio),
                           profile, "failure preserves all old diagnostics with each legal alias");
        }
    }
    context.expect(same_bits(residual, old_residual) && same_bits(variances, old_variances), profile,
                   "inputs unchanged");
}

template <typename Linalg, std::size_t Size> void test_safe_arithmetic(TestContext & context, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using vector_type = formal_eskf::linalg::Vector<Linalg, Size>;
    using check_type = formal_eskf::InnovationCheck<Linalg, Size>;
    using limits_type = std::numeric_limits<value_type>;
    vector_type innovation, variance;
    check_type result;

    innovation = vector_type::zero();
    for (std::size_t axis = 0U; axis < Size; ++axis)
    {
        variance(axis) = value_type{1};
    }
    for (auto const invalid : {limits_type::quiet_NaN(), limits_type::infinity(), -limits_type::infinity()})
    {
        expect_failure(context, profile, innovation, variance, invalid, Status::non_finite_input, "non-finite sigma");
        for (std::size_t axis = 0U; axis < Size; ++axis)
        {
            auto residual = innovation;
            residual(0U) = value_type{2};
            residual(axis) = invalid;
            expect_failure(context, profile, residual, variance, value_type{1}, Status::non_finite_input,
                           "non-finite residual axis");
            auto variances = variance;
            variances(axis) = invalid;
            expect_failure(context, profile, innovation, variances, value_type{1}, Status::non_finite_input,
                           "non-finite variance axis");
        }
    }
    for (auto const invalid : {value_type{0}, -value_type{0}, value_type{-1}})
    {
        expect_failure(context, profile, innovation, variance, invalid, Status::domain_error, "nonpositive sigma");
        for (std::size_t axis = 0U; axis < Size; ++axis)
        {
            auto variances = variance;
            variances(axis) = invalid;
            expect_failure(context, profile, innovation, variances, value_type{1}, Status::domain_error,
                           "nonpositive variance axis");
        }
    }
    if constexpr (Size > 1U)
    {
        auto residual = innovation;
        auto variances = variance;
        variances(0U) = value_type{0};
        residual(Size - 1U) = limits_type::quiet_NaN();
        expect_failure(context, profile, residual, variances, value_type{1}, Status::non_finite_input,
                       "all finite checks precede domain checks, including later axes");

        residual = innovation;
        residual(0U) = limits_type::max();
        variances = variance;
        variances(Size - 1U) = value_type{0};
        expect_failure(context, profile, residual, variances, limits_type::max(), Status::domain_error,
                       "variance validation precedes both sigma and residual arithmetic");

        // Independent domain/safe-range faults have no universal precedence.
        // The isolated boundary cases below check each required status.
        residual = innovation;
        residual(0U) = value_type{1};
        variances = variance;
        variances(Size - 1U) = limits_type::min();
        expect_failure(context, profile, residual, variances, value_type{0.5}, Status::zero_or_unsafe_divisor,
                       "an early rejected axis does not hide a later unsafe denominator");
    }
    expect_failure(context, profile, innovation, variance, limits_type::max(), Status::non_finite_result,
                   "squared sigma overflow");
    expect_failure(context, profile, innovation, variance, limits_type::min(), Status::zero_or_unsafe_divisor,
                   "squared sigma underflow");
    expect_failure(context, profile, innovation, variance, std::sqrt(limits_type::min()) / value_type{2},
                   Status::zero_or_unsafe_divisor, "subnormal squared sigma");
    for (std::size_t axis = 0U; axis < Size; ++axis)
    {
        auto residual = innovation;
        residual(0U) = value_type{2};
        residual(axis) = limits_type::max();
        expect_failure(context, profile, residual, variance, value_type{1}, Status::non_finite_result,
                       "squared residual overflow");
        auto variances = variance;
        variances(axis) = limits_type::max();
        expect_failure(context, profile, innovation, variances, value_type{2}, Status::non_finite_result,
                       "denominator overflow");
        variances(axis) = limits_type::min();
        expect_failure(context, profile, innovation, variances, value_type{0.5}, Status::zero_or_unsafe_divisor,
                       "subnormal denominator");
        expect_failure(context, profile, innovation, variances, std::sqrt(limits_type::min()),
                       Status::zero_or_unsafe_divisor, "zero denominator");
        residual(axis) = value_type{4};
        expect_failure(context, profile, residual, variances, value_type{1}, Status::non_finite_result,
                       "finite operands overflow ratio");
        variances(axis) = limits_type::denorm_min();
        expect_failure(context, profile, innovation, variances, value_type{1}, Status::zero_or_unsafe_divisor,
                       "subnormal variance");
    }

    innovation(0U) = std::sqrt(limits_type::min());
    context.expect(formal_eskf::check_innovation(innovation, variance, std::sqrt(limits_type::min()), result) ==
                           Status::success &&
                       !result.rejected && result.test_ratio(0U) == value_type{1},
                   profile, "minimum normal squared sigma is inclusive");
    variance(0U) = limits_type::min();
    context.expect(formal_eskf::check_innovation(innovation, variance, value_type{1}, result) == Status::success &&
                       !result.rejected && result.test_ratio(0U) == value_type{1},
                   profile, "minimum normal denominator is inclusive");
    innovation(0U) = limits_type::min();
    context.expect(formal_eskf::check_innovation(innovation, variance, value_type{1}, result) == Status::success &&
                       !result.rejected && result.test_ratio(0U) == value_type{0},
                   profile, "tiny innovation may square to zero");

    innovation(0U) = std::sqrt(limits_type::min());
    variance(0U) = limits_type::max();
    context.expect(innovation(0U) * innovation(0U) == limits_type::min(), profile,
                   "ratio-underflow fixture retains a nonzero normal numerator");
    context.expect(formal_eskf::check_innovation(innovation, variance, value_type{1}, result) == Status::success &&
                       !result.rejected && result.test_ratio(0U) == value_type{0},
                   profile, "a finite safe denominator may produce a zero underflowed ratio");

    variance(0U) = std::nextafter(limits_type::min(), value_type{0});
    expect_failure(context, profile, innovation, variance, value_type{1}, Status::zero_or_unsafe_divisor,
                   "variance immediately below the minimum normal value is unsafe");
    variance(0U) = value_type{1};
    value_type const sigma_below = std::nextafter(std::sqrt(limits_type::min()), value_type{0});
    context.expect(sigma_below * sigma_below < limits_type::min(), profile,
                   "squared-sigma boundary fixture lies on the intended represented side");
    expect_failure(context, profile, innovation, variance, sigma_below, Status::zero_or_unsafe_divisor,
                   "squared sigma immediately below the minimum normal value is unsafe");
    variance(0U) = std::nextafter(std::nextafter(value_type{4} * limits_type::min(), value_type{0}), value_type{0});
    context.expect(value_type{0.25} * variance(0U) == std::nextafter(limits_type::min(), value_type{0}), profile,
                   "denominator boundary fixture is exactly one subnormal step below minimum");
    expect_failure(context, profile, innovation, variance, value_type{0.5}, Status::zero_or_unsafe_divisor,
                   "denominator immediately below minimum is unsafe despite valid variance and sigma");
}

template <typename Linalg, std::size_t Size> void test_atomic_alias(TestContext & context, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using vector_type = formal_eskf::linalg::Vector<Linalg, Size>;
    using check_type = formal_eskf::InnovationCheck<Linalg, Size>;
    enum class Alias
    {
        disjoint,
        inputs_shared,
        output_residual,
        output_variance,
        all_shared
    };
    for (Alias const alias :
         {Alias::disjoint, Alias::inputs_shared, Alias::output_residual, Alias::output_variance, Alias::all_shared})
    {
        for (value_type const value : {value_type{0.25}, value_type{16}})
        {
            vector_type residual;
            vector_type variance;
            check_type output;
            for (std::size_t axis = 0U; axis < Size; ++axis)
            {
                value_type const coordinate = std::ldexp(value, static_cast<int>(axis));
                residual(axis) = coordinate;
                variance(axis) = coordinate;
                output.test_ratio(axis) = coordinate;
            }
            auto const & r =
                alias == Alias::output_residual || alias == Alias::all_shared ? output.test_ratio : residual;
            auto const & S = alias == Alias::output_variance || alias == Alias::all_shared ? output.test_ratio
                             : alias == Alias::inputs_shared                               ? residual
                                                                                           : variance;
            auto const old_residual = residual;
            auto const old_variance = variance;
            // r=S=coordinate and sigma=2 imply ratio=coordinate/4 exactly;
            // the output differs from its aliased input on every axis.
            auto status = formal_eskf::check_innovation(r, S, value_type{2}, output);
            context.expect(status == Status::success && output.rejected == (value > value_type{4}), profile,
                           "each of five alias partitions publishes correct acceptance");
            for (std::size_t axis = 0U; axis < Size; ++axis)
            {
                context.expect(output.test_ratio(axis) == std::ldexp(value, static_cast<int>(axis) - 2), profile,
                               "aliased ratio equals exact independent value");
            }
            context.expect(same_bits(residual, old_residual) && same_bits(variance, old_variance), profile,
                           "disjoint borrowed objects are not modified");

            for (bool const previous_flag : {false, true})
            {
                output.rejected = previous_flag;
                auto const before = output;
                status = formal_eskf::check_innovation(r, S, value_type{0}, output);
                context.expect(status == Status::domain_error && output.rejected == previous_flag &&
                                   same_bits(output.test_ratio, before.test_ratio),
                               profile, "all aliases preserve diagnostics on validation failure");
            }
        }
    }
    if constexpr (Size > 1U)
    {
        vector_type residual;
        vector_type variance;
        for (std::size_t axis = 0U; axis < Size; ++axis)
        {
            residual(axis) = value_type{2};
            variance(axis) = value_type{1};
        }
        residual(Size - 1U) = std::numeric_limits<value_type>::max();
        expect_failure(context, profile, residual, variance, value_type{1}, Status::non_finite_result,
                       "early outlier cannot hide late squared-innovation overflow");
        // In the shared-input classes every component supplies both quantities;
        // the last component still overflows only after earlier ratios rejected.
        for (bool const output_shared : {false, true})
        {
            check_type output;
            output.rejected = false;
            output.test_ratio = residual;
            auto const before = output;
            auto const & shared = output_shared ? output.test_ratio : residual;
            auto const status = formal_eskf::check_innovation(shared, shared, value_type{1}, output);
            context.expect(status == Status::non_finite_result && output.rejected == before.rejected &&
                               same_bits(output.test_ratio, before.test_ratio),
                           profile, "shared input aliases preserve complete output after late failure");
        }
    }
}

template <typename Linalg> void test_preparation_composition(TestContext & context, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using matrix_type = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    using vector_type = formal_eskf::linalg::Vector<Linalg, 3U>;
    auto const P = matrix_type::identity();
    auto const H = matrix_type::identity();
    auto V = matrix_type::identity();
    V(0U, 1U) = value_type{0.5};
    V(1U, 0U) = value_type{0.5};
    vector_type r;
    r(0U) = value_type{1.25};
    matrix_type C, S;
    auto status = formal_eskf::correction_covariances(P, r, H, V, value_type{0.5}, C, S);
    context.expect(status == Status::success, profile, "checked preparation produces an innovation covariance");
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t col = 0U; col < 3U; ++col)
        {
            value_type const cross = row == col ? value_type{1} : value_type{0};
            value_type const innovation = row == col        ? value_type{2}
                                          : row + col == 1U ? value_type{0.5}
                                                            : value_type{0};
            context.expect(C(row, col) == cross && S(row, col) == innovation, profile,
                           "independent exact preparation oracle retains prior and correlated noise");
        }
    }
    formal_eskf::InnovationCheck<Linalg, 3U> correct_gate, noise_only_gate;
    context.expect(formal_eskf::check_innovation(r, formal_eskf::linalg::diagonal(S), value_type{1}, correct_gate) ==
                           Status::success &&
                       !correct_gate.rejected && correct_gate.test_ratio(0U) == value_type{0.78125},
                   profile, "innovation covariance gate matches independent exact ratio");
    context.expect(formal_eskf::check_innovation(r, formal_eskf::linalg::diagonal(V), value_type{1}, noise_only_gate) ==
                           Status::success &&
                       noise_only_gate.rejected,
                   profile, "negative control using only measurement variance makes the wrong decision");

    formal_eskf::configuration::Ahrs::NominalState<Linalg> state, output;
    matrix_type corrected_covariance;
    context.expect(formal_eskf::correct(state, P, r, H, V, value_type{0.5}, output, corrected_covariance) ==
                       Status::success,
                   profile, "unchanged preparation inputs remain valid for an independent correction attempt");
    output.q_nb = -output.q_nb;
    auto const output_before = output;
    matrix_type covariance_output;
    covariance_output(2U, 1U) = value_type{-3};
    auto const saved = covariance_output;
    V(2U, 2U) = value_type{-1};
    status = formal_eskf::correct(state, P, r, H, V, value_type{0.5}, output, covariance_output);
    context.expect(status == Status::domain_error &&
                       formal_eskf::so3::same_coefficients(output.q_nb, output_before.q_nb),
                   profile, "old gate success does not bypass correction input validation");
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t col = 0U; col < 3U; ++col)
        {
            context.expect(covariance_output(row, col) == saved(row, col), profile,
                           "revalidation failure preserves covariance output");
        }
    }
}

} /* end anonymous namespace */

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Case const cases[] = {
        {"TC-E-INNOV-ratio-decision", "binary32.dim1", test_ratio_decision<formal_eskf::test::Backend<float>, 1U>},
        {"TC-E-INNOV-ratio-decision", "binary32.dim2", test_ratio_decision<formal_eskf::test::Backend<float>, 2U>},
        {"TC-E-INNOV-ratio-decision", "binary32.dim3", test_ratio_decision<formal_eskf::test::Backend<float>, 3U>},
        {"TC-E-INNOV-ratio-decision", "binary64.dim1", test_ratio_decision<formal_eskf::test::Backend<double>, 1U>},
        {"TC-E-INNOV-ratio-decision", "binary64.dim2", test_ratio_decision<formal_eskf::test::Backend<double>, 2U>},
        {"TC-E-INNOV-ratio-decision", "binary64.dim3", test_ratio_decision<formal_eskf::test::Backend<double>, 3U>},
        {"TC-E-INNOV-safe-arithmetic", "binary32.dim1", test_safe_arithmetic<formal_eskf::test::Backend<float>, 1U>},
        {"TC-E-INNOV-safe-arithmetic", "binary32.dim2", test_safe_arithmetic<formal_eskf::test::Backend<float>, 2U>},
        {"TC-E-INNOV-safe-arithmetic", "binary32.dim3", test_safe_arithmetic<formal_eskf::test::Backend<float>, 3U>},
        {"TC-E-INNOV-safe-arithmetic", "binary64.dim1", test_safe_arithmetic<formal_eskf::test::Backend<double>, 1U>},
        {"TC-E-INNOV-safe-arithmetic", "binary64.dim2", test_safe_arithmetic<formal_eskf::test::Backend<double>, 2U>},
        {"TC-E-INNOV-safe-arithmetic", "binary64.dim3", test_safe_arithmetic<formal_eskf::test::Backend<double>, 3U>},
        {"TC-E-INNOV-atomic-alias", "binary32.dim1", test_atomic_alias<formal_eskf::test::Backend<float>, 1U>},
        {"TC-E-INNOV-atomic-alias", "binary32.dim2", test_atomic_alias<formal_eskf::test::Backend<float>, 2U>},
        {"TC-E-INNOV-atomic-alias", "binary32.dim3", test_atomic_alias<formal_eskf::test::Backend<float>, 3U>},
        {"TC-E-INNOV-atomic-alias", "binary64.dim1", test_atomic_alias<formal_eskf::test::Backend<double>, 1U>},
        {"TC-E-INNOV-atomic-alias", "binary64.dim2", test_atomic_alias<formal_eskf::test::Backend<double>, 2U>},
        {"TC-E-INNOV-atomic-alias", "binary64.dim3", test_atomic_alias<formal_eskf::test::Backend<double>, 3U>},
        {"TC-E-INNOV-preparation-composition", "AHRS.binary32",
         test_preparation_composition<formal_eskf::test::Backend<float>>},
        {"TC-E-INNOV-preparation-composition", "AHRS.binary64",
         test_preparation_composition<formal_eskf::test::Backend<double>>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
