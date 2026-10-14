/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <iostream>
#include <limits>
#include <string_view>

#include "../examples/replay_gnss.hpp"
#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;

template <typename Scalar> void run_conformance(TestContext & test, std::string_view profile)
{
    using Linalg = formal_eskf::test::Backend<Scalar>;
    using Replay = formal_eskf::example::ReplayGnssQuality<Linalg>;
    using Decision = formal_eskf::runtime::GnssQualityDecision;
    typename Replay::monitor_type::parameters_type parameters;
    parameters.enabled_checks = formal_eskf::example::gnss_quality_checks(245U);
    parameters.minimum_health_time_us = 2'000'000U;
    typename Replay::monitor_type::sample_type sample;
    sample.fix_type = 3U;
    sample.satellites = 10U;
    sample.pdop = Scalar{1};
    sample.horizontal_accuracy = Scalar{1};
    sample.vertical_accuracy = Scalar{1};
    sample.speed_accuracy = static_cast<Scalar>(0.2);
    typename Replay::Metadata metadata{true, 0, -1, -1};
    Replay replay;
    auto update = [&](std::uint64_t time)
    {
        sample.time_us = time;
        return replay.update(sample, metadata, time, parameters);
    };

    auto result = update(100U);
    test.expect(result.outcome == "unavailable" && !result.evaluated && !result.accepted(), profile,
                "landed without rest metadata cannot pass selected stationary checks");
    metadata.in_air = 1;
    result = update(200U);
    test.expect(result.evaluated && result.motion_available && !result.spoofing_available &&
                    result.outcome == "waiting" && !result.quality.drift_valid,
                profile, "in-air needs no rest flag; absent unselected spoofing check is explicit");
    test.expect(!update(2'000'200U).accepted() && update(2'000'201U).accepted(), profile,
                "initial qualification is strictly greater than health time");
    sample.fix_type = 2U;
    result = update(2'000'202U);
    test.expect(result.outcome == "rejected" && result.quality.invalid_fix && result.quality.qualified, profile,
                "bad fix reaches the monitor and retains only historical qualification");
    sample.fix_type = 3U;
    test.expect(!update(3'000'202U).accepted() && update(3'000'203U).accepted(), profile,
                "recovery takes strictly more than one second, not another full startup wait");

    metadata.in_air = 0;
    test.expect(update(3'000'204U).outcome == "unavailable", profile, "lost required metadata blocks fusion");
    metadata.in_air = 1;
    result = update(3'000'205U);
    test.expect(result.outcome == "waiting" && !result.quality.qualified && !update(5'000'205U).accepted() &&
                    update(5'000'206U).accepted(),
                profile, "unavailable interval cannot count toward uninterrupted qualification");
    metadata.velocity_valid = false;
    result = update(5'000'207U);
    test.expect(result.outcome == "invalid_velocity" && !result.evaluated && !result.accepted(), profile,
                "invalid velocity is skipped before quality evaluation");
    metadata.velocity_valid = true;
    test.expect(update(5'000'208U).accepted(), profile, "skipped velocity message did not fabricate a quality failure");

    parameters.enabled_checks.spoofed = true;
    test.expect(update(5'000'209U).outcome == "unavailable", profile, "missing enabled spoofing data blocks quality");
    metadata.spoofed = 1;
    result = update(5'000'210U);
    test.expect(result.outcome == "rejected" && result.quality.failures.spoofed, profile,
                "actual spoofing metadata reaches the selected check");
    metadata.spoofed = 0;
    test.expect(update(7'000'211U).accepted(), profile, "fresh complete metadata can qualify again");
    metadata.in_air = 2;
    result = update(7'000'212U);
    test.expect(result.outcome == "invalid_call" && result.quality.status == formal_eskf::Status::domain_error, profile,
                "malformed flags are not converted silently to false");
    metadata.in_air = 1;
    test.expect(update(7'000'213U).accepted(), profile, "invalid flag did not mutate health history");
    sample.pdop = std::numeric_limits<Scalar>::quiet_NaN();
    result = update(7'000'214U);
    test.expect(result.evaluated && result.outcome == "rejected" && !result.accepted() &&
                    result.quality.status == formal_eskf::Status::non_finite_input,
                profile, "numerical quality rejection is not an invalid adapter call or successful sample");

    parameters.enabled_checks = {};
    metadata = {true, -1, -1, -1};
    sample.pdop = Scalar{1};
    replay = Replay{};
    result = update(1U);
    test.expect(result.evaluated && !result.motion_available && !result.quality.drift_valid &&
                    update(2'000'002U).accepted(),
                profile, "unknown unselected diagnostics do not authorize stationary checks or block selected checks");
    sample.fix_type = 2U;
    result = update(2'000'003U);
    test.expect(result.quality.decision == Decision::rejected && result.quality.invalid_fix, profile,
                "fix validity remains mandatory even with mask zero");

    for (unsigned bit = 0U; bit < 10U; ++bit)
    {
        auto const selection = formal_eskf::example::gnss_quality_checks(1U << bit);
        for (unsigned other = 0U; other < 10U; ++other)
        {
            test.expect(selection.any_enabled(formal_eskf::example::gnss_quality_checks(1U << other)) == (bit == other),
                        profile, "adapter mask maps each check independently");
        }
    }
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
    std::cout << "Replay GNSS quality tests passed\n";
    return 0;
}
