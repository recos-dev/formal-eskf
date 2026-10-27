/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/magnetic_reference.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <string_view>

#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{

using formal_eskf::Status;
using TestContext = formal_eskf::test::Context;
constexpr std::string_view profile = "geographic magnetic reference";

template <typename Scalar> void run_tests(TestContext & test)
{
    using Linalg = formal_eskf::test::Backend<Scalar>;
    using Model = formal_eskf::runtime::MagneticReference<Linalg>;
    using Sample = typename Model::Sample;
    auto const pi = formal_eskf::scalar::pi<typename Linalg::scalar_math_type>();
    auto const degree = pi / Scalar{180};
    auto near = [](Scalar a, Scalar b) { return std::abs(a - b) < static_cast<Scalar>(1e-4); };
    Sample sample{pi / Scalar{2}, pi / Scalar{6}, Scalar{60}};
    Model model;
    auto result = model.update(100U, {}, false);
    test.expect(result.status == Status::success && !result.checked && model.state().reference_id == 0U, profile,
                "no geographic position or origin means no model initialization");
    result = model.update(101U, sample, false, true);
    test.expect(
        result.status == Status::success && result.checked && result.changed && model.state().reference_id == 1U &&
            model.state().last_check_time_us == 101U && near(model.state().field_n(0U), Scalar{0}) &&
            near(model.state().field_n(1U), Scalar{30} * std::sqrt(Scalar{3})) &&
            near(model.state().field_n(2U), Scalar{30}),
        profile, "origin initializes east-positive declination, Down-positive inclination and microtesla field");
    auto next = sample;
    next.strength = Scalar{62};
    result = model.update(10000101U, next, true);
    test.expect(result.status == Status::success && !result.checked && model.state().reference_id == 1U, profile,
                "ten-second equality does not trigger a periodic check");
    result = model.update(10000102U, next, true);
    test.expect(result.status == Status::success && result.checked && result.changed &&
                    model.state().reference_id == 2U,
                profile, "one microsecond later checks and adopts a significant change");
    result = model.update(10000103U, sample, false, true);
    test.expect(result.status == Status::success && result.checked && result.changed &&
                    model.state().reference_id == 3U,
                profile, "origin update bypasses the periodic interval and position-valid condition");
    result = model.update(30000103U, next, false);
    test.expect(result.status == Status::success && !result.checked && model.state().last_check_time_us == 10000103U,
                profile, "position loss preserves the last adopted reference without advancing the check clock");
    result = model.update(30000104U, sample, true);
    test.expect(result.status == Status::success && result.checked && !result.changed &&
                    model.state().last_check_time_us == 30000104U && model.state().reference_id == 3U,
                profile, "unchanged successful lookup refreshes only the check clock");

    for (unsigned axis = 0U; axis < 3U; ++axis)
    {
        model.reset();
        sample = {Scalar{0}, Scalar{0}, Scalar{50}};
        test.expect(model.update(1U, sample, true).changed, profile,
                    "valid position can initialize without origin event");
        next = sample;
        if (axis == 0U)
        {
            next.declination = degree;
        }
        else if (axis == 1U)
        {
            next.inclination = degree;
        }
        else
        {
            next.strength = Scalar{51};
        }
        result = model.update(2U, next, false, true);
        test.expect(result.status == Status::success && result.checked && !result.changed &&
                        model.state().reference_id == 1U && model.state().model.strength == Scalar{50},
                    profile, "one-degree and one-microtesla equalities retain the adopted model");
        if (axis == 0U)
        {
            next.declination = std::nextafter(next.declination, Scalar{1});
        }
        else if (axis == 1U)
        {
            next.inclination = std::nextafter(next.inclination, Scalar{1});
        }
        else
        {
            next.strength = std::nextafter(next.strength, Scalar{52});
        }
        result = model.update(3U, next, false, true);
        test.expect(result.status == Status::success && result.changed && model.state().reference_id == 2U, profile,
                    "next representable value above each threshold changes the reference exactly once");
    }
    model.reset();
    sample = {Scalar{0}, Scalar{0}, Scalar{50}};
    test.expect(model.update(1U, sample, true).changed, profile, "cumulative change setup");
    next = sample;
    next.strength = static_cast<Scalar>(50.6);
    test.expect(!model.update(2U, next, false, true).changed, profile, "small movement retains the adopted value");
    next.strength = static_cast<Scalar>(51.2);
    test.expect(model.update(3U, next, false, true).changed, profile,
                "small successive changes accumulate against the adopted model");
    next = {pi, -pi / Scalar{2}, Scalar{40}};
    test.expect(model.update(4U, next, false, true).changed && near(model.state().field_n(2U), Scalar{-40}), profile,
                "negative inclination points Up and endpoint angles are valid");
    next.declination = -pi;
    test.expect(model.update(5U, next, false, true).changed, profile,
                "direct angle differences follow the reference update rule across the declination cut");

    auto const saved = model.state();
    auto same = [&]()
    {
        return model.state().reference_id == saved.reference_id &&
               model.state().last_check_time_us == saved.last_check_time_us &&
               formal_eskf::linalg::max_abs(model.state().field_n - saved.field_n) == Scalar{0} &&
               model.state().model.declination == saved.model.declination &&
               model.state().model.inclination == saved.model.inclination &&
               model.state().model.strength == saved.model.strength;
    };
    for (unsigned kind = 0U; kind < 6U; ++kind)
    {
        auto invalid = sample;
        if (kind == 0U)
        {
            invalid.declination = std::numeric_limits<Scalar>::quiet_NaN();
        }
        else if (kind == 1U)
        {
            invalid.inclination = std::numeric_limits<Scalar>::infinity();
        }
        else if (kind == 2U)
        {
            invalid.strength = std::numeric_limits<Scalar>::infinity();
        }
        else if (kind == 3U)
        {
            invalid.declination = Scalar{4};
        }
        else if (kind == 4U)
        {
            invalid.inclination = Scalar{2};
        }
        else
        {
            invalid.strength = Scalar{0};
        }
        result = model.update(6U, invalid, true, true);
        test.expect(result.status == (kind < 3U ? Status::non_finite_input : Status::domain_error) && !result.changed &&
                        same(),
                    profile, "failed model ingestion preserves reference and timing history");
    }
    test.expect(model.update(6U, sample, true, true).status == Status::success, profile,
                "failure does not consume the timestamp; corrected data can retry");
    test.expect(model.update(6U, sample, true).status == Status::out_of_range &&
                    model.update(0U, sample, true).status == Status::out_of_range,
                profile, "reject repeated or backwards timestamps");
    model.reset();
    result = model.update(std::numeric_limits<std::uint64_t>::max(), sample, true);
    test.expect(result.status == Status::success && result.changed, profile,
                "timestamp arithmetic does not add a deadline");
}

} // namespace

int main()
{
    TestContext test;
    run_tests<float>(test);
    run_tests<double>(test);
    if (test.failures() != 0)
    {
        return 1;
    }
    std::cout << "Magnetic reference: pass\n";
    return 0;
}
