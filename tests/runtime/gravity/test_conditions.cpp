/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/accelerometer.hpp>

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <string_view>

#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{

using formal_eskf::Status;
using formal_eskf::runtime::AccelerometerFusionConditions;
using formal_eskf::runtime::check_accelerometer_fusion_conditions;
using TestContext = formal_eskf::test::Context;

template <typename Linalg> void test_truth_table(TestContext & test, std::string_view profile)
{
    using Magnitude = formal_eskf::runtime::AccelerometerMagnitude<Linalg>;
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
            typename Magnitude::Result const magnitude{Status::success, (flags & 16U) != 0U, {}};
            auto const result = check_accelerometer_fusion_conditions<Linalg>(conditions, magnitude);
            // Explicit accepted rows avoid reproducing the production Boolean expression.
            bool const expected_active = flags == 11U || flags == 19U || flags == 27U;
            test.expect(result.status == Status::success && result.active == expected_active &&
                            result.sample_eligible == (expected_active && clipping == 0U),
                        profile, "all 256 Boolean/clipping partitions preserve activation versus eligibility");
        }
    }
    constexpr typename Magnitude::Result magnitude{Status::success, true, {}};
    constexpr auto disabled = check_accelerometer_fusion_conditions<Linalg>({}, magnitude);
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
    invalid(0U) = std::numeric_limits<value_type>::quiet_NaN();
    magnitude = monitor.update(invalid);
    result = check_accelerometer_fusion_conditions<Linalg>(conditions, magnitude);
    test.expect(result.status == Status::non_finite_input && !result.active && !result.sample_eligible, profile,
                "rest cannot override a failed current-sample magnitude evaluation");
}

template <typename Linalg> void test_failed_evidence(TestContext & test, std::string_view profile)
{
    using Magnitude = formal_eskf::runtime::AccelerometerMagnitude<Linalg>;
    using Scalar = typename Linalg::value_type;
    constexpr std::array statuses{Status::non_finite_input,
                                  Status::domain_error,
                                  Status::out_of_range,
                                  Status::zero_or_unsafe_divisor,
                                  Status::invalid_quaternion_norm,
                                  Status::not_positive_definite,
                                  Status::ill_conditioned,
                                  Status::non_finite_result};
    for (auto status : statuses)
    {
        for (bool at_rest : {false, true})
        {
            AccelerometerFusionConditions const conditions{true, true, false, at_rest, {false, false, false}};
            typename Magnitude::Result const magnitude{status, true, std::numeric_limits<Scalar>::quiet_NaN()};
            auto const result = check_accelerometer_fusion_conditions<Linalg>(conditions, magnitude);
            test.expect(result.status == status && !result.active && !result.sample_eligible, profile,
                        "every failed producer status inhibits both flags even at rest");
            test.expect(magnitude.status == status && magnitude.within_gravity_band &&
                            std::isnan(magnitude.squared_norm) && conditions.enabled && conditions.tilt_aligned &&
                            !conditions.horizontal_aiding && conditions.at_rest == at_rest,
                        profile, "condition and evidence inputs remain unchanged");
        }
    }
    for (Scalar unused :
         {Scalar{-1}, std::numeric_limits<Scalar>::infinity(), std::numeric_limits<Scalar>::quiet_NaN()})
    {
        typename Magnitude::Result const magnitude{Status::success, true, unused};
        AccelerometerFusionConditions const conditions{true, true, false, false, {false, false, false}};
        auto const result = check_accelerometer_fusion_conditions<Linalg>(conditions, magnitude);
        test.expect(result.status == Status::success && result.active && result.sample_eligible, profile,
                    "only producer status and band verdict consumed; unused numeric diagnostics add no new gate");
    }
    test_magnitude_integration<Linalg>(test, profile);
}

} /* end namespace */

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Case;
    using Float = formal_eskf::test::Backend<float>;
    using Double = formal_eskf::test::Backend<double>;
    constexpr std::array cases{
        Case{"TC-R-ACCELEROMETER-CONDITIONS-01", "float", test_truth_table<Float>},
        Case{"TC-R-ACCELEROMETER-CONDITIONS-01", "double", test_truth_table<Double>},
        Case{"TC-R-ACCELEROMETER-CONDITIONS-02", "float", test_failed_evidence<Float>},
        Case{"TC-R-ACCELEROMETER-CONDITIONS-02", "double", test_failed_evidence<Double>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
