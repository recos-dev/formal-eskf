/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/accelerometer_magnitude.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string_view>

#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{

using formal_eskf::Status;
using TestContext = formal_eskf::test::Context;

template <typename Linalg> void test_filter(TestContext & test, std::string_view profile)
{
    using Monitor = formal_eskf::runtime::AccelerometerMagnitude<Linalg>;
    using value_type = typename Monitor::value_type;
    using vector_type = typename Monitor::vector_type;
    Monitor monitor;
    constexpr auto tolerance = value_type{32} * std::numeric_limits<value_type>::epsilon();
    constexpr auto dt = static_cast<value_type>(0.01);
    constexpr auto tau = static_cast<value_type>(0.09);
    constexpr auto alpha = dt / (tau + dt);
    constexpr auto gravity = static_cast<value_type>(9.80665);
    auto const first = vector_type::from_row_major({value_type{1}, value_type{-2}, value_type{-9}});
    static_assert(noexcept(monitor.update(first)));
    static_assert(noexcept(monitor.reset()));
    test.expect(!monitor.initialized(), profile, "new monitor has no sample history");
    auto result = monitor.update(first);
    test.expect(result.status == Status::success && monitor.initialized() && result.within_gravity_band &&
                    result.squared_norm == value_type{86} &&
                    formal_eskf::linalg::max_abs(monitor.filtered_specific_force_b() - first) == value_type{0},
                profile, "first sample seeds the vector directly, without a zero-start transient");

    // Independent scalar recurrence: each call uses the same coefficient.
    std::array<value_type, 3U> expected{value_type{1}, value_type{-2}, value_type{-9}};
    for (std::size_t index = 0U; index < 200U; ++index)
    {
        auto const sample = vector_type::from_row_major(
            {static_cast<value_type>(index % 7U), -static_cast<value_type>(index % 11U), value_type{-10}});
        result = monitor.update(sample);
        value_type expected_squared{};
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            expected[axis] += alpha * (sample(axis) - expected[axis]);
            expected_squared += expected[axis] * expected[axis];
            test.expect(formal_eskf::test::near(monitor.filtered_specific_force_b()(axis), expected[axis], tolerance),
                        profile, "fixed-alpha vector recurrence matches the scalar reference");
        }
        test.expect(result.status == Status::success &&
                        formal_eskf::test::near(result.squared_norm, expected_squared, tolerance * expected_squared),
                    profile, "diagnostic is the squared magnitude of the filtered vector");
        auto const magnitude = std::sqrt(expected_squared);
        bool const expected_band =
            magnitude > static_cast<value_type>(0.9) * gravity && magnitude < static_cast<value_type>(1.1) * gravity;
        test.expect(result.within_gravity_band == expected_band, profile,
                    "every update re-evaluates the band without latching an earlier decision");
    }

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
    for (std::size_t index = 0U; index < magnitudes.size(); ++index)
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            for (int const sign : std::array<int, 2U>{-1, 1})
            {
                Monitor monitor;
                vector_type sample;
                sample.set(axis, static_cast<value_type>(sign) * magnitudes[index]);
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
                bad.set(axis, value);
                auto const result = monitor.update(bad);
                auto const expected = std::isfinite(value) ? Status::non_finite_result : Status::non_finite_input;
                test.expect(result.status == expected && !result.within_gravity_band &&
                                result.squared_norm == value_type{0},
                            profile, "non-finite input or norm overflow returns no valid diagnostics");
                test.expect(monitor.initialized() == initialized &&
                                formal_eskf::linalg::max_abs(monitor.filtered_specific_force_b() - before) ==
                                    value_type{0},
                            profile, "failed update preserves all vector history and initialization");
                auto const recovered = monitor.update(good);
                test.expect(recovered.status == Status::success && recovered.squared_norm == value_type{86}, profile,
                            "invalid sample cannot poison subsequent valid updates");
            }
        }
    }
}

template <typename Scalar> void run_conformance(TestContext & test, std::string_view profile)
{
    using Linalg = formal_eskf::test::Backend<Scalar>;
    test_filter<Linalg>(test, profile);
    test_band<Linalg>(test, profile);
    test_failures<Linalg>(test, profile);
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
    std::cout << "Accelerometer magnitude tests passed\n";
    return 0;
}
