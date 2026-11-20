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
#include <numbers>

#include <formal_eskf/runtime/accelerometer.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{
using formal_eskf::Status;
using formal_eskf::test::Case;
using Context = formal_eskf::test::Context;
template <typename Backend> using Sampler = formal_eskf::runtime::AccelerometerSample<Backend, 10000U>;

template <typename Backend> void integration(Context & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    constexpr Scalar minimum_norm = Scalar{0.125};
    // Fixed fixtures only: normal increments, <= three rotations of at most pi/2,
    // force <=16. This regression envelope is not a general libm/error guarantee.
    constexpr Scalar rotation_bound = Scalar{64} * std::numeric_limits<Scalar>::epsilon();
    formal_eskf::ImuSample<Backend> imu{};
    imu.specific_force_b(0) = Scalar{16};
    Sampler<Backend> sampler;
    auto const constant = sampler.update(imu, 15625, 0, minimum_norm);
    test.expect(constant.status == Status::success && constant.ready && constant.interval_us == 15625 &&
                    constant.specific_force_b(0) == Scalar{16} && constant.specific_force_b(1) == Scalar{0} &&
                    constant.specific_force_b(2) == Scalar{0} && constant.magnitude.squared_norm == Scalar{256},
                profile, "zero-angle power-of-two force integrates and divides by actual duration exactly");

    sampler.reset();
    imu = {};
    imu.specific_force_b(0) = Scalar{1};
    test.expect(!sampler.update(imu, 5000, 0, minimum_norm).ready, profile, "first interval retained");
    imu.specific_force_b = {};
    imu.angular_rate_b(2) = std::numbers::pi_v<Scalar> * Scalar{100};
    auto const transported = sampler.update(imu, 5000, 0, minimum_norm);
    test.expect(transported.status == Status::success && transported.ready &&
                    std::abs(transported.specific_force_b(0)) <= rotation_bound &&
                    std::abs(transported.specific_force_b(1) + Scalar{0.5}) <= rotation_bound &&
                    std::abs(transported.specific_force_b(2)) <= rotation_bound,
                profile, "positive Z quarter-turn transports earlier body X into current minus Y");

    sampler.reset();
    imu = {};
    test.expect(!sampler.update(imu, 5000, 0, minimum_norm).ready, profile, "zero initial increment");
    imu.specific_force_b(0) = Scalar{1};
    imu.angular_rate_b(2) = std::numbers::pi_v<Scalar> * Scalar{100};
    auto const trapezoid = sampler.update(imu, 5000, 0, minimum_norm);
    test.expect(trapezoid.ready && std::abs(trapezoid.specific_force_b(0) - Scalar{0.25}) <= rotation_bound &&
                    std::abs(trapezoid.specific_force_b(1) + Scalar{0.25}) <= rotation_bound,
                profile, "current increment averages both endpoint frames");

    sampler.reset();
    imu = {};
    imu.specific_force_b(0) = Scalar{1};
    test.expect(!sampler.update(imu, 2500, 0, minimum_norm).ready, profile, "noncommuting sequence starts partial");
    imu = {};
    imu.specific_force_b(1) = Scalar{1};
    imu.angular_rate_b(2) = std::numbers::pi_v<Scalar> * Scalar{200};
    test.expect(!sampler.update(imu, 2500, 0, minimum_norm).ready, profile, "minimum-period equality remains partial");
    imu = {};
    imu.specific_force_b(2) = Scalar{1};
    imu.angular_rate_b(0) = std::numbers::pi_v<Scalar> * Scalar{200};
    auto const noncommuting = sampler.update(imu, 2500, 0, minimum_norm);
    // Signed quarter-turn matrices give total increment (1/800,1/800,1/400).
    std::array<Scalar, 3> const expected{Scalar{1} / Scalar{6}, Scalar{1} / Scalar{6}, Scalar{1} / Scalar{3}};
    test.expect(noncommuting.ready && noncommuting.interval_us == 7500, profile, "actual three-sample duration");
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        test.expect(std::abs(noncommuting.specific_force_b(axis) - expected[axis]) <= rotation_bound, profile,
                    "independent signed-rotation oracle detects reversed or omitted transport");
    }

    sampler.reset();
    imu = {};
    imu.specific_force_b(2) = Scalar{-8};
    test.expect(!sampler.update(imu, 2000, 0, minimum_norm).ready, profile, "unequal-period average starts partial");
    imu.specific_force_b(2) = Scalar{-12};
    auto const weighted = sampler.update(imu, 9000, 0, minimum_norm);
    Scalar const exact_average = Scalar{-124} / Scalar{11};
    test.expect(weighted.ready && weighted.interval_us == 11000 &&
                    std::abs(weighted.specific_force_b(2) - exact_average) <=
                        Scalar{16} * std::numeric_limits<Scalar>::epsilon() * std::abs(exact_average),
                profile, "independent duration-weighted average excludes count-average/latest-sample substitutes");
}

template <typename Backend> void timing(Context & test, std::string_view profile)
{
    formal_eskf::ImuSample<Backend> imu{};
    using Scalar = typename Backend::value_type;
    // A separate schedule ledger has no production sampler state or math calls.
    std::array<std::array<std::uint64_t, 4>, 5> const schedules{{{4000, 4000, 4000, 4000},
                                                                 {2500, 2500, 2500, 2500},
                                                                 {10000, 10000, 10000, 10000},
                                                                 {1000, 9000, 4000, 6000},
                                                                 {5000, 5000, 5000, 5000}}};
    for (auto const & schedule : schedules)
    {
        Sampler<Backend> sampler;
        double average_us = 5000;
        std::uint64_t required = 2, duration = 0, count = 0, emitted_us = 0, input_us = 0;
        double target_us = 10000, minimum_us = 5000;
        for (std::size_t index = 0; index < 512; ++index)
        {
            auto const period = schedule[index % schedule.size()];
            duration += period;
            input_us += period;
            ++count;
            bool const expected_ready = (count >= required && static_cast<double>(duration) > minimum_us) ||
                                        static_cast<double>(duration) > target_us;
            auto const result = sampler.update(imu, period, 0, Scalar{0.125});
            test.expect(result.status == Status::success && result.ready == expected_ready, profile,
                        "250/400/100 Hz and variable-period schedules follow the independent completion ledger");
            if (expected_ready)
            {
                test.expect(result.interval_us == duration && result.clipping_mask == 0 &&
                                result.magnitude.squared_norm == Scalar{0},
                            profile, "one emitted interval contains every actual input exactly once");
                emitted_us += duration;
            }
            else
            {
                test.expect(result.interval_us == 0, profile, "partial result publishes no interval");
            }
            average_us = average_us + 0.1 * (static_cast<double>(period) - average_us);
            if (expected_ready)
            {
                required = static_cast<std::uint64_t>(std::floor(10000.0 / average_us + 0.5));
                required = required == 0 ? 1 : required;
                target_us = static_cast<double>(required) * average_us;
                minimum_us = std::max(static_cast<double>(required - 1) * average_us, average_us / 2);
                count = 0;
                duration = 0;
            }
            test.expect(emitted_us + duration == input_us, profile, "no split, overlap, duplicate or implicit flush");
        }
    }
    for (auto const periods : {std::array<std::uint64_t, 2>{10000, 1}, std::array<std::uint64_t, 2>{5000, 1},
                               std::array<std::uint64_t, 2>{5000, 5000}})
    {
        Sampler<Backend> sampler;
        auto const first = sampler.update(imu, periods[0], 0, Scalar{0.125});
        auto const second = sampler.update(imu, periods[1], 0, Scalar{0.125});
        test.expect(!first.ready && second.ready && second.interval_us == periods[0] + periods[1], profile,
                    "strict initial minimum/target equality and first real increment above it");
    }
    Sampler<Backend> slow;
    auto const result = slow.update(imu, 50000, 0, Scalar{0.125});
    test.expect(result.status == Status::success && result.ready && result.interval_us == 50000, profile,
                "maximum legal interval emits once with actual duration");
}

template <typename Backend> void clipping(Context & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    formal_eskf::ImuSample<Backend> imu{};
    for (int first = -1; first <= 7; ++first)
    {
        for (int second = -1; second <= 7; ++second)
        {
            Sampler<Backend> sampler;
            test.expect(!sampler.update(imu, 5000, first, Scalar{0.125}).ready, profile,
                        "mask interval starts partial");
            auto const result = sampler.update(imu, 5000, second, Scalar{0.125});
            int const expected = first == -1 || second == -1 ? -1 : first | second;
            test.expect(result.status == Status::success && result.ready && result.clipping_mask == expected, profile,
                        "all 81 mask pairs preserve unknown or OR known body-axis flags");
            test.expect(!sampler.update(imu, 5000, 0, Scalar{0.125}).ready, profile, "next interval is independent");
            auto const clear = sampler.update(imu, 5000, 0, Scalar{0.125});
            test.expect(clear.ready && clear.clipping_mask == 0, profile, "unknown/clip flags clear at emission");
        }
    }
}

template <typename Backend> void failures(Context & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    formal_eskf::ImuSample<Backend> imu{};
    imu.specific_force_b(2) = Scalar{-8};
    enum class Fault
    {
        zero_period,
        excessive_period,
        unknown_mask,
        excessive_mask,
        norm_limit,
        unsafe_norm,
        nan_limit,
        force_nan,
        angular_nan,
        angular_overflow,
        norm_overflow,
        priority
    };
    constexpr std::array faults{Fault::zero_period,      Fault::excessive_period, Fault::unknown_mask,
                                Fault::excessive_mask,   Fault::norm_limit,       Fault::unsafe_norm,
                                Fault::nan_limit,        Fault::force_nan,        Fault::angular_nan,
                                Fault::angular_overflow, Fault::norm_overflow,    Fault::priority};
    for (unsigned prefix : {0U, 1U, 2U, 3U})
    {
        Sampler<Backend> baseline;
        for (unsigned index = 0; index < prefix; ++index)
        {
            test.expect(baseline.update(imu, 5000, 1, Scalar{0.125}).status == Status::success, profile,
                        "reachable sampler prefix");
        }
        for (auto fault : faults)
        {
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                auto sampler = baseline;
                auto control = baseline;
                auto bad = imu;
                std::uint64_t period = 5000;
                int mask = 0;
                Scalar minimum = Scalar{0.125};
                Status expected = Status::domain_error;
                switch (fault)
                {
                case Fault::zero_period:
                    period = 0;
                    break;
                case Fault::excessive_period:
                    period = 50001;
                    break;
                case Fault::unknown_mask:
                    mask = -2;
                    break;
                case Fault::excessive_mask:
                    mask = 8;
                    break;
                case Fault::norm_limit:
                    minimum = 0;
                    break;
                case Fault::unsafe_norm:
                    minimum = Scalar{2};
                    expected = Status::invalid_quaternion_norm;
                    break;
                case Fault::nan_limit:
                    minimum = std::numeric_limits<Scalar>::quiet_NaN();
                    expected = Status::non_finite_input;
                    break;
                case Fault::force_nan:
                    bad.specific_force_b(axis) = std::numeric_limits<Scalar>::quiet_NaN();
                    expected = Status::non_finite_input;
                    break;
                case Fault::angular_nan:
                    bad.angular_rate_b(axis) = std::numeric_limits<Scalar>::infinity();
                    expected = Status::non_finite_input;
                    break;
                case Fault::angular_overflow:
                    bad.angular_rate_b(axis) = std::numeric_limits<Scalar>::max();
                    expected = Status::non_finite_result;
                    break;
                case Fault::norm_overflow:
                    bad.specific_force_b(axis) = std::numeric_limits<Scalar>::max();
                    period = 50000;
                    expected = Status::non_finite_result;
                    break;
                case Fault::priority:
                    bad.specific_force_b(axis) = std::numeric_limits<Scalar>::quiet_NaN();
                    period = 0;
                    mask = 8;
                    expected = Status::non_finite_input;
                    break;
                }
                auto const old_force = bad.specific_force_b;
                auto const old_rate = bad.angular_rate_b;
                auto const rejected = sampler.update(bad, period, mask, minimum);
                using Bits = std::array<std::byte, sizeof(Scalar)>;
                for (std::size_t component = 0; component < 3; ++component)
                {
                    auto same = [](Scalar left, Scalar right) {
                        return (std::isnan(left) && std::isnan(right)) ||
                               std::bit_cast<Bits>(left) == std::bit_cast<Bits>(right);
                    };
                    test.expect(same(bad.specific_force_b(component), old_force(component)) &&
                                    same(bad.angular_rate_b(component), old_rate(component)),
                                profile,
                                "failing update preserves borrowed coefficients, excluding unspecified NaN payloads");
                }
                test.expect(rejected.status == expected && !rejected.ready && rejected.interval_us == 0, profile,
                            "invalid input/domain/arithmetic does not publish or consume a sample");
                for (auto next_period : {4000U, 9000U, 1U, 5000U})
                {
                    auto const actual = sampler.update(imu, next_period, 2, Scalar{0.125});
                    auto const wanted = control.update(imu, next_period, 2, Scalar{0.125});
                    test.expect(actual.status == wanted.status && actual.ready == wanted.ready &&
                                    actual.interval_us == wanted.interval_us &&
                                    actual.clipping_mask == wanted.clipping_mask &&
                                    actual.magnitude.status == wanted.magnitude.status &&
                                    actual.magnitude.squared_norm == wanted.magnitude.squared_norm &&
                                    actual.magnitude.within_gravity_band == wanted.magnitude.within_gravity_band,
                                profile,
                                "common continuation checks timing, interval, clipping and magnitude history rollback");
                    for (std::size_t component = 0; component < 3; ++component)
                    {
                        test.expect(actual.specific_force_b(component) == wanted.specific_force_b(component), profile,
                                    "all integrated force components preserved after failure");
                    }
                }
            }
        }
    }
}

template <typename Backend> void reset_and_downstream(Context & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    formal_eskf::ImuSample<Backend> imu{};
    imu.specific_force_b(2) = Scalar{-8};
    Sampler<Backend> sampler;
    auto const first = sampler.update(imu, 50000, 7, Scalar{0.125});
    test.expect(first.ready && first.magnitude.squared_norm == Scalar{64}, profile,
                "first emission seeds LPF from actual force");
    formal_eskf::runtime::AccelerometerFusionConditions disabled{};
    auto const condition =
        formal_eskf::runtime::check_accelerometer_fusion_conditions<Backend>(disabled, first.magnitude);
    test.expect(!condition.active && !condition.sample_eligible, profile, "disabled fusion does not undo sampling");
    imu.specific_force_b(2) = Scalar{-16};
    auto const second = sampler.update(imu, 50000, -1, Scalar{0.125});
    Scalar const alpha = static_cast<Scalar>(0.010) / (static_cast<Scalar>(0.090) + static_cast<Scalar>(0.010));
    Scalar const filtered = Scalar{-8} + alpha * Scalar{-8};
    test.expect(second.ready && second.specific_force_b(2) == Scalar{-16} &&
                    second.magnitude.squared_norm == filtered * filtered,
                profile, "raw average remains distinct; magnitude updates once even after disabled fusion");
    test.expect(!sampler.update(imu, 1, 7, Scalar{0.125}).ready, profile, "leave old-source partial interval");
    sampler.reset();
    Sampler<Backend> fresh;
    imu.specific_force_b(2) = Scalar{-4};
    for (auto period : {5000U, 5000U, 4000U, 9000U})
    {
        auto const result = sampler.update(imu, period, 0, Scalar{0.125});
        auto const reference = fresh.update(imu, period, 0, Scalar{0.125});
        test.expect(result.status == reference.status && result.ready == reference.ready &&
                        result.interval_us == reference.interval_us &&
                        result.clipping_mask == reference.clipping_mask &&
                        result.magnitude.squared_norm == reference.magnitude.squared_norm,
                    profile, "explicit gap/source/calibration reset clears cadence, partial data, mask and LPF");
        if (result.ready)
        {
            test.expect(result.specific_force_b(2) == Scalar{-4} && result.clipping_mask == 0 &&
                            result.magnitude.squared_norm == Scalar{16},
                        profile, "new source seeds without an invented held or flushed observation");
        }
    }
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using Float = formal_eskf::test::Backend<float>;
    using Double = formal_eskf::test::Backend<double>;
    constexpr std::array cases{
        Case{"TC-R-ACCELEROMETER-SAMPLING-01", "float", integration<Float>},
        Case{"TC-R-ACCELEROMETER-SAMPLING-01", "double", integration<Double>},
        Case{"TC-R-ACCELEROMETER-SAMPLING-02", "float", timing<Float>},
        Case{"TC-R-ACCELEROMETER-SAMPLING-02", "double", timing<Double>},
        Case{"TC-R-ACCELEROMETER-SAMPLING-03", "float", clipping<Float>},
        Case{"TC-R-ACCELEROMETER-SAMPLING-03", "double", clipping<Double>},
        Case{"TC-R-ACCELEROMETER-SAMPLING-04", "float", failures<Float>},
        Case{"TC-R-ACCELEROMETER-SAMPLING-04", "double", failures<Double>},
        Case{"TC-R-ACCELEROMETER-SAMPLING-05", "float", reset_and_downstream<Float>},
        Case{"TC-R-ACCELEROMETER-SAMPLING-05", "double", reset_and_downstream<Double>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
