/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "../examples/replay_conditions.hpp"

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

template <typename Linalg> void test_tilt_alignment(TestContext & test, std::string_view profile)
{
    using tracker_type = formal_eskf::example::ReplayFusionConditions<Linalg>;
    using value_type = typename Linalg::value_type;
    using matrix_type = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    using quaternion_type = formal_eskf::so3::UnitQuaternion<Linalg>;
    tracker_type tracker;
    quaternion_type const identity;
    auto P = matrix_type::identity();
    test.expect(tracker.update_tilt(identity, P) == Status::success && !tracker.tilt_aligned(), profile,
                "a valid initial quaternion does not certify tilt alignment");
    P = {};
    P(0U, 0U) = tracker_type::tilt_limit * tracker_type::tilt_limit;
    P(2U, 2U) = value_type{100};
    test.expect(tracker.update_tilt(identity, P) == Status::success && !tracker.tilt_aligned(), profile,
                "strict three-degree tilt-variance threshold rejects equality");
    P(0U, 0U) = std::nextafter(P(0U, 0U), value_type{0});
    test.expect(tracker.update_tilt(identity, P) == Status::success && tracker.tilt_aligned(), profile,
                "just below the threshold aligns despite high yaw uncertainty");
    P = matrix_type::identity();
    test.expect(tracker.update_tilt(identity, P) == Status::success && tracker.tilt_aligned(), profile,
                "alignment is latched, not toggled by later covariance growth");
    auto const old_variance = tracker.tilt_variance();
    P(0U, 0U) = std::numeric_limits<value_type>::quiet_NaN();
    test.expect(tracker.update_tilt(identity, P) == Status::non_finite_input && tracker.tilt_aligned() &&
                    tracker.tilt_variance() == old_variance,
                profile, "invalid covariance does not change the alignment history");
    tracker = tracker_type{};
    P = {};
    P(1U, 1U) = value_type{-1};
    test.expect(tracker.update_tilt(identity, P) == Status::domain_error && !tracker.tilt_aligned(), profile,
                "negative tilt variance cannot align");
    P = {};
    P(0U, 0U) = std::numeric_limits<value_type>::max();
    P(1U, 1U) = std::numeric_limits<value_type>::max();
    test.expect(tracker.update_tilt(identity, P) == Status::non_finite_result && !tracker.tilt_aligned(), profile,
                "variance-sum overflow cannot align");

    quaternion_type pitch;
    test.expect(quaternion_type::try_from_coefficients(value_type{1}, value_type{0}, value_type{1}, value_type{0},
                                                       static_cast<value_type>(1e-6), pitch) == Status::success,
                profile, "90-degree pitch fixture is normalized");
    P = {};
    P(0U, 0U) = value_type{9};
    P(1U, 1U) = value_type{4};
    P(2U, 2U) = value_type{1};
    test.expect(tracker.update_tilt(pitch, P) == Status::success &&
                    std::abs(tracker.tilt_variance() - value_type{5}) < static_cast<value_type>(1e-4),
                profile, "tilt uses NED X/Y variance, not body covariance entries at arbitrary attitude");
}

template <typename Linalg> void test_aiding_and_flags(TestContext & test, std::string_view profile)
{
    using tracker_type = formal_eskf::example::ReplayFusionConditions<Linalg>;
    using matrix_type = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    tracker_type tracker;
    test.expect(!tracker.horizontal_aiding(100U), profile, "initialization is not a successful aiding event");
    test.expect(tracker.update_tilt({}, matrix_type{}) == Status::success, profile, "aligned test prior");
    tracker.record_horizontal_fusion(100U);
    test.expect(tracker.horizontal_aiding(101U) &&
                    tracker.horizontal_aiding(100U + tracker_type::horizontal_timeout_us - 1U),
                profile, "active aiding persists through ticks without a new navigation sample");
    test.expect(!tracker.horizontal_aiding(100U + tracker_type::horizontal_timeout_us), profile,
                "missing-data timeout releases aiding at the strict recency boundary");
    test.expect(!tracker.conditions(99U, true, 0, 0).has_value(), profile,
                "a timestamp before recorded fusion does not produce usable conditions");
    tracker.record_horizontal_fusion(200U + tracker_type::horizontal_timeout_us);
    test.expect(tracker.horizontal_aiding(201U + tracker_type::horizontal_timeout_us), profile,
                "actual resumed fusion reactivates this fail-fast replay source");

    tracker = tracker_type{};
    test.expect(tracker.update_tilt({}, matrix_type{}) == Status::success, profile, "fresh aligned test prior");
    typename formal_eskf::runtime::AccelerometerMagnitude<Linalg>::Result magnitude;
    magnitude.within_gravity_band = true;
    for (int mask = 0; mask < 8; ++mask)
    {
        auto const conditions = tracker.conditions(100U, true, -1, mask);
        test.expect(conditions.has_value(), profile, "valid clipping mask yields complete conditions");
        if (!conditions)
        {
            continue;
        }
        auto const check = formal_eskf::runtime::check_accelerometer_fusion_conditions<Linalg>(*conditions, magnitude);
        test.expect(conditions->tilt_aligned && !conditions->horizontal_aiding && !conditions->at_rest &&
                        conditions->clipped[0U] == ((mask & 1) != 0) && conditions->clipped[1U] == ((mask & 2) != 0) &&
                        conditions->clipped[2U] == ((mask & 4) != 0) && check.active &&
                        check.sample_eligible == (mask == 0),
                    profile, "all clipping bits are mapped and unknown rest does not enable its override");
    }
    for (int mask : {-1, -2, 8})
    {
        test.expect(!tracker.conditions(100U, true, 1, mask).has_value(), profile,
                    "unknown/invalid clipping cannot authorize a sample, even when at rest");
    }
    for (int rest : {-2, 2})
    {
        test.expect(!tracker.conditions(100U, true, rest, 0).has_value(), profile, "invalid rest flag is rejected");
    }
    magnitude.within_gravity_band = false;
    for (int rest : {-1, 0, 1})
    {
        auto const conditions = tracker.conditions(100U, true, rest, 0);
        test.expect(conditions &&
                        formal_eskf::runtime::check_accelerometer_fusion_conditions<Linalg>(*conditions, magnitude)
                                .sample_eligible == (rest == 1),
                    profile, "only an explicit true rest flag overrides the magnitude band");
    }
    auto const disabled = tracker.conditions(100U, false, 1, 0);
    test.expect(disabled && !disabled->enabled &&
                    !formal_eskf::runtime::check_accelerometer_fusion_conditions<Linalg>(*disabled, magnitude).active,
                profile, "enablement remains explicit even with complete metadata");
    test.expect(!tracker.conditions(0U, true, 0, 0), profile, "zero timestamp cannot produce conditions");
}

template <typename Scalar> void run_conformance(TestContext & test, std::string_view profile)
{
    using Linalg = formal_eskf::test::Backend<Scalar>;
    test_tilt_alignment<Linalg>(test, profile);
    test_aiding_and_flags<Linalg>(test, profile);
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
    std::cout << "Replay fusion condition tests passed\n";
    return 0;
}
