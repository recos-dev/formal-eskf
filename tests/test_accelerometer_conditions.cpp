/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/accelerometer_conditions.hpp>

#include <array>
#include <iostream>
#include <limits>
#include <string_view>

#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{

using formal_eskf::Status;
using formal_eskf::runtime::AccelerometerFusionConditions;
using formal_eskf::runtime::check_accelerometer_fusion_conditions;
using TestContext = formal_eskf::test::Context;

template <typename Linalg> void test_truth_table(TestContext & test, std::string_view profile)
{
    using Magnitude = formal_eskf::runtime::AccelerometerMagnitude<Linalg>;
    constexpr std::array<Status, 9U> statuses{Status::success,
                                              Status::non_finite_input,
                                              Status::domain_error,
                                              Status::out_of_range,
                                              Status::zero_or_unsafe_divisor,
                                              Status::invalid_quaternion_norm,
                                              Status::not_positive_definite,
                                              Status::ill_conditioned,
                                              Status::non_finite_result};
    // Five logical bits: enable, tilt, horizontal aiding, rest, magnitude band.
    for (unsigned flags = 0U; flags < 32U; ++flags)
    {
        for (unsigned clipping = 0U; clipping < 8U; ++clipping)
        {
            AccelerometerFusionConditions const conditions{
                (flags & 1U) != 0U,
                (flags & 2U) != 0U,
                (flags & 4U) != 0U,
                (flags & 8U) != 0U,
                {(clipping & 1U) != 0U, (clipping & 2U) != 0U, (clipping & 4U) != 0U}};
            for (auto const status : statuses)
            {
                typename Magnitude::Result const magnitude{status, (flags & 16U) != 0U, {}};
                auto const result = check_accelerometer_fusion_conditions<Linalg>(conditions, magnitude);
                // Enumerated active rows, independent of the production boolean expression.
                bool const expected_active =
                    status == Status::success && (flags == 11U || flags == 19U || flags == 27U);
                test.expect(result.status == status && result.active == expected_active &&
                                result.sample_eligible == (expected_active && clipping == 0U),
                            profile, "all 256 boolean combinations and every Status preserve the two-stage decision");
            }
        }
    }
    constexpr typename Magnitude::Result magnitude{Status::success, true, {}};
    constexpr auto disabled = check_accelerometer_fusion_conditions<Linalg>({}, magnitude);
    static_assert(noexcept(check_accelerometer_fusion_conditions<Linalg>({}, magnitude)));
    static_assert(disabled.status == Status::success && !disabled.active && !disabled.sample_eligible);
}

template <typename Linalg> void test_magnitude_integration(TestContext & test, std::string_view profile)
{
    using Magnitude = formal_eskf::runtime::AccelerometerMagnitude<Linalg>;
    using value_type = typename Magnitude::value_type;
    using vector_type = typename Magnitude::vector_type;
    Magnitude monitor;
    AccelerometerFusionConditions conditions;
    conditions.enabled = true;
    conditions.tilt_aligned = true;
    auto const gravity = vector_type::from_row_major({value_type{0}, value_type{0}, static_cast<value_type>(-9.80665)});
    auto magnitude = monitor.update(gravity);
    auto result = check_accelerometer_fusion_conditions<Linalg>(conditions, magnitude);
    test.expect(result.status == Status::success && result.active && result.sample_eligible, profile,
                "actual successful magnitude result enables the pre-innovation path without horizontal aiding");

    conditions.horizontal_aiding = true;
    result = check_accelerometer_fusion_conditions<Linalg>(conditions, magnitude);
    test.expect(!result.active && !result.sample_eligible, profile, "active horizontal aiding disables gravity mode");
    conditions.horizontal_aiding = false;
    conditions.clipped[1U] = true;
    result = check_accelerometer_fusion_conditions<Linalg>(conditions, magnitude);
    test.expect(result.active && !result.sample_eligible, profile, "clipping rejects this sample, not the active flag");
    conditions.clipped[1U] = false;
    test.expect(check_accelerometer_fusion_conditions<Linalg>(conditions, magnitude).sample_eligible, profile,
                "clearing clipping has no hidden recovery timer or latched rejection");

    monitor.reset();
    magnitude = monitor.update(gravity * value_type{2});
    result = check_accelerometer_fusion_conditions<Linalg>(conditions, magnitude);
    test.expect(result.status == Status::success && !result.active && !result.sample_eligible, profile,
                "out-of-band magnitude alone does not enable gravity mode");
    conditions.at_rest = true;
    result = check_accelerometer_fusion_conditions<Linalg>(conditions, magnitude);
    test.expect(result.status == Status::success && result.active && result.sample_eligible, profile,
                "caller rest flag bypasses only the successful magnitude-band rejection");

    auto invalid = gravity;
    invalid.set(0U, std::numeric_limits<value_type>::quiet_NaN());
    magnitude = monitor.update(invalid);
    result = check_accelerometer_fusion_conditions<Linalg>(conditions, magnitude);
    test.expect(result.status == Status::non_finite_input && !result.active && !result.sample_eligible, profile,
                "rest cannot override a failed current-sample magnitude evaluation");
}

template <typename Scalar> void run_conformance(TestContext & test, std::string_view profile)
{
    using Linalg = formal_eskf::test::Backend<Scalar>;
    test_truth_table<Linalg>(test, profile);
    test_magnitude_integration<Linalg>(test, profile);
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
    std::cout << "Accelerometer fusion condition tests passed\n";
    return 0;
}
