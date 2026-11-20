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
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string_view>

#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{

using formal_eskf::Status;
using TestContext = formal_eskf::test::Context;

template <typename Vector> bool same_bits(Vector const & left, Vector const & right)
{
    using Bytes = std::array<std::byte, sizeof(typename Vector::value_type)>;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        if (std::bit_cast<Bytes>(left(axis)) != std::bit_cast<Bytes>(right(axis)))
        {
            return false;
        }
    }
    return true;
}

template <typename Linalg> void test_filter(TestContext & test, std::string_view profile)
{
    using Monitor = formal_eskf::runtime::AccelerometerMagnitude<Linalg>;
    using value_type = typename Monitor::value_type;
    using vector_type = typename Monitor::vector_type;
    Monitor monitor;
    constexpr auto dt = static_cast<value_type>(0.01);
    constexpr auto tau = static_cast<value_type>(0.09);
    constexpr auto alpha = dt / (tau + dt);
    constexpr auto gravity = static_cast<value_type>(9.80665);
    auto const first = vector_type::from_row_major({value_type{1}, value_type{-2}, value_type{-9}});
    test.expect(!monitor.initialized(), profile, "new monitor has no sample history");
    auto result = monitor.update(first);
    test.expect(result.status == Status::success && monitor.initialized() && result.within_gravity_band &&
                    result.squared_norm == value_type{86} &&
                    formal_eskf::linalg::max_abs(monitor.filtered_specific_force_b() - first) == value_type{0},
                profile, "first sample seeds the vector directly, without a zero-start transient");

    // Ordered IEEE recurrence checks coefficient/order, not a real-number error bound.
    // The explicit scalar reference does not call production vector/filter operations.
    std::array<value_type, 3U> expected{value_type{1}, value_type{-2}, value_type{-9}};
    for (std::size_t index = 0U; index < 200U; ++index)
    {
        auto const sample = vector_type::from_row_major(
            {static_cast<value_type>(index % 7U), -static_cast<value_type>(index % 11U), value_type{-10}});
        result = monitor.update(sample);
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            expected[axis] += alpha * (sample(axis) - expected[axis]);
            test.expect(monitor.filtered_specific_force_b()(axis) == expected[axis], profile,
                        "fixed-alpha vector recurrence matches the scalar reference");
        }
        // Three products and two sums: gamma_5 bounds a normal, nonnegative dot.
        // Long-double reference error is separately bounded, including binary64.
        long double reference_squared = 0;
        for (auto const coefficient : expected)
        {
            auto const wide = static_cast<long double>(coefficient);
            reference_squared += wide * wide;
        }
        long double const unit_roundoff = std::numeric_limits<value_type>::epsilon() / 2.0L;
        long double const gamma5 = 5 * unit_roundoff / (1 - 5 * unit_roundoff);
        long double const reference_bound = 8 * std::numeric_limits<long double>::epsilon();
        long double const bound = (gamma5 + reference_bound) * reference_squared;
        test.expect(result.status == Status::success &&
                        std::abs(static_cast<long double>(result.squared_norm) - reference_squared) <= bound,
                    profile, "nonnegative norm agrees with an independently bounded wider reference");
        auto const lower = static_cast<value_type>(0.9) * gravity;
        auto const upper = static_cast<value_type>(1.1) * gravity;
        bool const expected_band = result.squared_norm > lower * lower && result.squared_norm < upper * upper;
        test.expect(result.within_gravity_band == expected_band, profile,
                    "every update re-evaluates the band without latching an earlier decision");
    }

    Monitor opposing;
    test.expect(opposing.update(first).status == Status::success, profile, "seed opposite-vector fixture");
    auto const opposite_result = opposing.update(first * value_type{-1});
    test.expect(opposite_result.status == Status::success && !opposite_result.within_gravity_band &&
                    opposite_result.squared_norm < value_type{60},
                profile, "opposite vectors cancel before the norm; filtering magnitudes would stay in band");

    monitor.reset();
    test.expect(!monitor.initialized() &&
                    formal_eskf::linalg::max_abs(monitor.filtered_specific_force_b()) == value_type{0},
                profile, "reset clears initialization and vector history");
    result = monitor.update(first);
    test.expect(result.status == Status::success && result.squared_norm == value_type{86}, profile,
                "first sample after reset is not mixed with the previous source");
    result = monitor.update(monitor.filtered_specific_force_b());
    test.expect(result.status == Status::success && result.squared_norm == value_type{86}, profile,
                "constant sample and self-alias retain the vector");
}

template <typename Linalg> void test_band(TestContext & test, std::string_view profile)
{
    using Monitor = formal_eskf::runtime::AccelerometerMagnitude<Linalg>;
    using value_type = typename Monitor::value_type;
    using vector_type = typename Monitor::vector_type;
    constexpr auto gravity = static_cast<value_type>(9.80665);
    auto const lower = gravity * static_cast<value_type>(0.9);
    auto const upper = gravity * static_cast<value_type>(1.1);
    auto const infinity = std::numeric_limits<value_type>::infinity();
    std::array<value_type, 9U> const magnitudes{value_type{0},
                                                std::nextafter(lower, value_type{0}),
                                                lower,
                                                std::nextafter(lower, infinity),
                                                gravity,
                                                std::nextafter(upper, value_type{0}),
                                                upper,
                                                std::nextafter(upper, infinity),
                                                value_type{2} * gravity};
    constexpr std::array<bool, 9U> expected{false, false, false, true, true, true, false, false, false};
    for (std::size_t index = 0; index < magnitudes.size(); ++index)
    {
        value_type const squared = magnitudes[index] * magnitudes[index];
        test.expect((squared > lower * lower && squared < upper * upper) == expected[index], profile,
                    "fixture squared value reaches its intended represented partition");
    }
    for (std::size_t index = 0U; index < magnitudes.size(); ++index)
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            for (int const sign : std::array<int, 2U>{-1, 1})
            {
                Monitor monitor;
                vector_type sample;
                sample(axis) = static_cast<value_type>(sign) * magnitudes[index];
                auto const result = monitor.update(sample);
                test.expect(result.status == Status::success && result.within_gravity_band == expected[index], profile,
                            "strict gravity band, adjacent floats, all axes and both signs");
            }
        }
    }

    Monitor monitor;
    auto const first = vector_type::from_row_major({value_type{0}, value_type{0}, -gravity});
    auto const seed = monitor.update(first);
    auto const second = vector_type::from_row_major({gravity, value_type{0}, value_type{0}});
    auto const result = monitor.update(second);
    test.expect(seed.status == Status::success && result.status == Status::success && result.within_gravity_band &&
                    result.squared_norm < static_cast<value_type>(0.83) * gravity * gravity,
                profile, "filter the vector before magnitude, not the magnitude or a peak envelope");

    monitor.reset();
    auto const accelerating =
        vector_type::from_row_major({static_cast<value_type>(0.2) * gravity, value_type{0}, -gravity});
    test.expect(monitor.update(accelerating).within_gravity_band, profile,
                "0.2g horizontal acceleration can pass: a band pass is not fusion permission");
}

template <typename Linalg> void test_failures(TestContext & test, std::string_view profile)
{
    using Monitor = formal_eskf::runtime::AccelerometerMagnitude<Linalg>;
    using value_type = typename Monitor::value_type;
    using vector_type = typename Monitor::vector_type;
    auto const good = vector_type::from_row_major({value_type{1}, value_type{-2}, value_type{-9}});
    // max() exercises squared-norm overflow, not an independently reachable
    // component-overflow branch. Successful stored history has finite norm;
    // the latter guard is retained in formal/cpp/esbmc/accelerometer_runtime.cpp.
    std::array<value_type, 4U> const invalid{
        std::numeric_limits<value_type>::quiet_NaN(), std::numeric_limits<value_type>::infinity(),
        -std::numeric_limits<value_type>::infinity(), std::numeric_limits<value_type>::max()};
    for (bool const initialized : std::array<bool, 2U>{false, true})
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            for (auto const value : invalid)
            {
                Monitor monitor;
                if (initialized)
                {
                    test.expect(monitor.update(good).status == Status::success, profile, "seed failure fixture");
                }
                auto const before = monitor.filtered_specific_force_b();
                auto bad = good;
                bad(axis) = value;
                auto const bad_before = bad;
                auto const result = monitor.update(bad);
                for (std::size_t component = 0; component < 3; ++component)
                {
                    test.expect(std::isnan(bad_before(component)) ? std::isnan(bad(component))
                                                                  : bad(component) == bad_before(component),
                                profile, "failed sample is not modified");
                }
                auto const expected = std::isfinite(value) ? Status::non_finite_result : Status::non_finite_input;
                test.expect(result.status == expected && !result.within_gravity_band &&
                                result.squared_norm == value_type{0},
                            profile, "non-finite input or norm overflow returns no valid diagnostics");
                test.expect(monitor.initialized() == initialized &&
                                same_bits(monitor.filtered_specific_force_b(), before),
                            profile, "failed update preserves all vector history and initialization");
                auto const recovered = monitor.update(good);
                test.expect(recovered.status == Status::success && recovered.squared_norm == value_type{86}, profile,
                            "invalid sample cannot poison subsequent valid updates");
            }
        }
    }
    Monitor signed_zero;
    vector_type const zero = vector_type::from_row_major({value_type{0}, -value_type{0}, value_type{0}});
    test.expect(signed_zero.update(zero).status == Status::success, profile, "seed signed-zero history");
    auto const before_zero = signed_zero.filtered_specific_force_b();
    auto invalid_zero = zero;
    invalid_zero(0) = std::numeric_limits<value_type>::infinity();
    test.expect(signed_zero.update(invalid_zero).status == Status::non_finite_input && signed_zero.initialized() &&
                    same_bits(signed_zero.filtered_specific_force_b(), before_zero),
                profile, "failure preserves signed-zero representations in finite history");
}

template <typename Linalg> void test_units_and_cadence(TestContext & test, std::string_view profile)
{
    using Monitor = formal_eskf::runtime::AccelerometerMagnitude<Linalg>;
    using Scalar = typename Monitor::value_type;
    using Vector = typename Monitor::vector_type;
    Vector const raw = Vector::from_row_major({Scalar{1}, Scalar{-2}, Scalar{-9}});
    Monitor raw_monitor, scaled_monitor, preprocessed_monitor;
    auto const raw_result = raw_monitor.update(raw);
    auto const scaled_result = scaled_monitor.update(raw * Scalar{2});
    Vector const wrongly_bias_subtracted = Vector::from_row_major({Scalar{0}, Scalar{0}, Scalar{-9}});
    auto const wrong_result = preprocessed_monitor.update(wrongly_bias_subtracted);
    test.expect(wrong_result.status == Status::success && wrong_result.squared_norm == Scalar{81} &&
                    wrong_result.squared_norm != raw_result.squared_norm,
                profile, "bias-subtracted negative control differs from the required raw measurement");
    test.expect(raw_result.status == Status::success && raw_result.squared_norm == Scalar{86} &&
                    scaled_result.status == Status::success && scaled_result.squared_norm == Scalar{344},
                profile, "raw physical units retained; no normalization or bias preprocessing");
    Monitor fast, sparse;
    for (unsigned index = 0; index < 16; ++index)
    {
        Vector const sample = Vector::from_row_major({static_cast<Scalar>(index), Scalar{0}, Scalar{-10}});
        auto const before = sample;
        auto const fast_result = fast.update(sample);
        // External idle activity has no update call and therefore no filter time.
        for (unsigned idle = 0; idle < index; ++idle)
        {
            test.expect(sparse.initialized() == (index != 0), profile, "query does not consume a sample");
        }
        auto const sparse_result = sparse.update(sample);
        test.expect(fast_result.status == sparse_result.status &&
                        fast_result.squared_norm == sparse_result.squared_norm &&
                        fast_result.within_gravity_band == sparse_result.within_gravity_band,
                    profile, "same consumed sequence has same result irrespective of caller cadence");
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            test.expect(sample(axis) == before(axis) &&
                            fast.filtered_specific_force_b()(axis) == sparse.filtered_specific_force_b()(axis),
                        profile, "pure input and no hidden elapsed-time or source dependence");
        }
    }
    Monitor out_of_band;
    test.expect(out_of_band.update(Vector{}).status == Status::success && out_of_band.initialized(), profile,
                "zero sample is successful history, not an input error");
    Vector const large = Vector::from_row_major({Scalar{16}, Scalar{0}, Scalar{0}});
    auto const result = out_of_band.update(large);
    Scalar const alpha = static_cast<Scalar>(0.010) / (static_cast<Scalar>(0.090) + static_cast<Scalar>(0.010));
    test.expect(result.status == Status::success && !result.within_gravity_band &&
                    out_of_band.filtered_specific_force_b()(0) == alpha * Scalar{16},
                profile, "finite out-of-band observation still advances fixed-alpha history");
}

} /* end namespace */

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Case;
    using Float = formal_eskf::test::Backend<float>;
    using Double = formal_eskf::test::Backend<double>;
    constexpr std::array cases{
        Case{"TC-R-ACCELEROMETER-MAGNITUDE-01", "float", test_filter<Float>},
        Case{"TC-R-ACCELEROMETER-MAGNITUDE-01", "double", test_filter<Double>},
        Case{"TC-R-ACCELEROMETER-MAGNITUDE-02", "float", test_band<Float>},
        Case{"TC-R-ACCELEROMETER-MAGNITUDE-02", "double", test_band<Double>},
        Case{"TC-R-ACCELEROMETER-MAGNITUDE-03", "float", test_failures<Float>},
        Case{"TC-R-ACCELEROMETER-MAGNITUDE-03", "double", test_failures<Double>},
        Case{"TC-R-ACCELEROMETER-MAGNITUDE-04", "float", test_units_and_cadence<Float>},
        Case{"TC-R-ACCELEROMETER-MAGNITUDE-04", "double", test_units_and_cadence<Double>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
