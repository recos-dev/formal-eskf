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
#include <formal_eskf/runtime/magnetometer.hpp>
#include <formal_eskf/runtime/magnetic_reference.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{
using formal_eskf::Status;
using Context = formal_eskf::test::Context;

template <typename Vector> bool same(Vector const & a, Vector const & b)
{
    using Bits = std::array<std::byte, sizeof(typename Vector::value_type)>;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        if (std::bit_cast<Bits>(a(axis)) != std::bit_cast<Bits>(b(axis)))
        {
            return false;
        }
    }
    return true;
}

template <typename Vector> bool bounded(Vector const & actual, Vector const & expected)
{
    using Scalar = typename Vector::value_type;
    // Ordinary recurrence fixtures have stored coefficients <=40. The opposing
    // control uses input -180, difference -200 and a scaled increment of magnitude20.
    // Eight scalar epsilons at scale40 cover setup and the short affine sequence;
    // this local affine check is not a general filtering-error/time-response claim.
    Scalar const bound = Scalar{8} * std::numeric_limits<Scalar>::epsilon() * Scalar{40};
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        if (!std::isfinite(actual(axis)) || !std::isfinite(expected(axis)) ||
            std::abs(actual(axis) - expected(axis)) > bound)
        {
            return false;
        }
    }
    return true;
}

template <typename Backend> void recurrence(Context & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    using Filter = formal_eskf::runtime::MagnetometerFilter<Backend>;
    using Vector = typename Filter::vector_type;
    Filter filter;
    test.expect(!filter.initialized() && same(filter.field_b(), Vector{}), profile,
                "default filter has no measurement history");
    auto const first = Vector::from_row_major({Scalar{20}, Scalar{0}, Scalar{40}});
    auto const second = Vector::from_row_major({Scalar{0}, Scalar{20}, Scalar{40}});
    test.expect(filter.update(first) == Status::success && filter.initialized() && same(filter.field_b(), first),
                profile, "first actual sample seeds every coefficient exactly");
    test.expect(filter.update(second) == Status::success &&
                    bounded(filter.field_b(), Vector::from_row_major({Scalar{18}, Scalar{2}, Scalar{40}})),
                profile, "independent bounded one-tenth step uses body vector components, not unit direction");
    std::array<long double, 3> expected{18, 2, 40};
    std::array<Vector, 3> const steps{second, second, Vector::from_row_major({Scalar{40}, Scalar{-20}, Scalar{0}})};
    for (auto const & sample : steps)
    {
        Vector reference;
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            expected[axis] += (static_cast<long double>(sample(axis)) - expected[axis]) / 10;
            reference(axis) = static_cast<Scalar>(expected[axis]);
        }
        test.expect(filter.update(sample) == Status::success && bounded(filter.field_b(), reference), profile,
                    "independent multi-step recurrence covers repeated constant input and a subsequent changed vector");
    }
    auto const snapshot = filter.field_b();
    test.expect(filter.initialized() && same(filter.field_b(), snapshot) && same(filter.field_b(), snapshot), profile,
                "both accessors are pure");
    Filter opposing;
    auto const x = Vector::from_row_major({Scalar{20}, Scalar{0}, Scalar{0}});
    auto const reversed = Vector::from_row_major({Scalar{-180}, Scalar{0}, Scalar{0}});
    test.expect(opposing.update(x) == Status::success && opposing.update(reversed) == Status::success &&
                    bounded(opposing.field_b(), Vector{}),
                profile, "opposing signed vectors cancel, unlike filtering field strengths");
    filter.reset();
    test.expect(!filter.initialized() && same(filter.field_b(), Vector{}) && filter.update(second) == Status::success &&
                    same(filter.field_b(), second),
                profile, "reset clears initialization and all stored components before fresh seed");
}

template <typename Backend> void failures(Context & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    using Filter = formal_eskf::runtime::MagnetometerFilter<Backend>;
    using Vector = typename Filter::vector_type;
    auto const seed = Vector::from_row_major({Scalar{20}, -Scalar{0}, Scalar{40}});
    for (bool initialized : {false, true})
    {
        for (bool reseed : {false, true})
        {
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                for (auto invalid : {std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity(),
                                     -std::numeric_limits<Scalar>::infinity()})
                {
                    Filter filter;
                    if (initialized)
                    {
                        test.expect(filter.update(seed) == Status::success, profile, "seed failure history");
                    }
                    auto control = filter;
                    auto const previous = filter.field_b();
                    auto sample = seed;
                    sample(axis) = invalid;
                    auto const original = sample;
                    test.expect(filter.update(sample, reseed) == Status::non_finite_input &&
                                    filter.initialized() == initialized && same(filter.field_b(), previous) &&
                                    same(sample, original),
                                profile, "each nonfinite axis preserves history and borrowed input, including reseed");
                    test.expect(filter.update(seed) == control.update(seed) &&
                                    same(filter.field_b(), control.field_b()),
                                profile, "common continuation detects initialization or history changes after failure");
                }
                Filter filter;
                Vector maximum;
                maximum(axis) = std::numeric_limits<Scalar>::max();
                test.expect(filter.update(maximum, true) == Status::success, profile,
                            "finite maximum is legal raw field data");
                Vector opposite = maximum * Scalar{-1};
                auto const original_opposite = opposite;
                auto control = filter;
                test.expect(filter.update(opposite) == Status::non_finite_result && filter.initialized() &&
                                same(filter.field_b(), maximum) && same(opposite, original_opposite),
                            profile,
                            "finite subtraction overflow before interpolation preserves complete filter state");
                test.expect(filter.update(maximum) == control.update(maximum) && filter.initialized() &&
                                same(filter.field_b(), control.field_b()),
                            profile,
                            "ordinary continuation preserves initialization and old history after arithmetic failure");
                test.expect(filter.update(seed, true) == Status::success && same(filter.field_b(), seed), profile,
                            "explicit valid reseed recovers immediately after failed arithmetic");
            }
        }
    }
}

template <typename Backend> void source_reseed(Context & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    using Filter = formal_eskf::runtime::MagnetometerFilter<Backend>;
    using Vector = typename Filter::vector_type;
    auto const old_source = Vector::from_row_major({Scalar{20}, Scalar{0}, Scalar{40}});
    auto const new_source = Vector::from_row_major({Scalar{-30}, Scalar{50}, -Scalar{0}});
    Filter switched, fresh;
    formal_eskf::runtime::MagneticReference<Backend> reference;
    typename decltype(reference)::Sample model{Scalar{0}, Scalar{0}, Scalar{20}};
    test.expect(reference.update(1U, model, true).changed && switched.update(old_source) == Status::success, profile,
                "reference and physical filter begin independent histories");
    model.strength = Scalar{22};
    auto const unchanged_source = Vector::from_row_major({Scalar{0}, Scalar{20}, Scalar{40}});
    test.expect(reference.update(2U, model, true, true).changed && reference.state().reference_id == 2U &&
                    switched.update(unchanged_source) == Status::success &&
                    bounded(switched.field_b(), Vector::from_row_major({Scalar{18}, Scalar{2}, Scalar{40}})),
                profile, "reference-only adoption leaves physical LPF history intact under an ordinary update");
    switched.reset();
    test.expect(
        switched.update(old_source) == Status::success && switched.update(old_source) == Status::success &&
            switched.update(new_source, true) == Status::success && fresh.update(new_source) == Status::success &&
            same(switched.field_b(), new_source) && same(switched.field_b(), fresh.field_b()),
        profile, "source/calibration reseed replaces old history exactly and preserves raw signed coefficients");
    auto const next = Vector::from_row_major({Scalar{10}, Scalar{-20}, Scalar{30}});
    test.expect(switched.update(next) == Status::success && fresh.update(next) == Status::success &&
                    same(switched.field_b(), fresh.field_b()),
                profile, "post-switch continuation contains no weighted contribution from the retired source");
    auto const borrowed = switched.field_b();
    test.expect(switched.update(switched.field_b(), true) == Status::success && same(switched.field_b(), borrowed),
                profile, "borrowing the current stored vector as reseed input is safe");
    // Ordinary correction input remains caller-owned raw data; the filter has no
    // estimator pointer, timestamp, calibration, normalization or feedback API.
    test.expect(same(new_source, Vector::from_row_major({Scalar{-30}, Scalar{50}, -Scalar{0}})), profile,
                "source observations remain unchanged by reset filtering");
}
} /* end namespace */

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Case;
    using Float = formal_eskf::test::Backend<float>;
    using Double = formal_eskf::test::Backend<double>;
    Case const cases[] = {
        {"TC-R-MAGNETOMETER-FILTER-01", "float", recurrence<Float>},
        {"TC-R-MAGNETOMETER-FILTER-01", "double", recurrence<Double>},
        {"TC-R-MAGNETOMETER-FILTER-02", "float", failures<Float>},
        {"TC-R-MAGNETOMETER-FILTER-02", "double", failures<Double>},
        {"TC-R-MAGNETOMETER-FILTER-03", "float", source_reseed<Float>},
        {"TC-R-MAGNETOMETER-FILTER-03", "double", source_reseed<Double>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
