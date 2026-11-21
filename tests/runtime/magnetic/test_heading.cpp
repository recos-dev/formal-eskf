/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/magnetometer.hpp>
#include <array>
#include <bit>
#include <cmath>
#include <limits>
#include <numbers>
#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{
using formal_eskf::Status;
using Context = formal_eskf::test::Context;
template <typename Backend> using Heading = formal_eskf::runtime::MagneticHeading<Backend>;
template <typename Backend> using Vector = typename Heading<Backend>::vector_type;
template <typename Backend> using Quaternion = formal_eskf::so3::UnitQuaternion<Backend>;
template <typename Backend> auto parameters()
{
    using Scalar = typename Backend::value_type;
    return typename Heading<Backend>::Parameters{Scalar{0.25}, Scalar{0.5}, Scalar{5}};
}
template <typename Backend> auto north()
{
    using Scalar = typename Backend::value_type;
    return Vector<Backend>::from_row_major({Scalar{20}, Scalar{0}, Scalar{40}});
}
template <typename State> bool same(State const & a, State const & b)
{
    using Bits = std::array<std::byte, sizeof(a.innovation)>;
    return std::bit_cast<Bits>(a.innovation) == std::bit_cast<Bits>(b.innovation) &&
           std::bit_cast<Bits>(a.filtered_innovation) == std::bit_cast<Bits>(b.filtered_innovation) &&
           std::bit_cast<Bits>(a.horizontal_acceleration) == std::bit_cast<Bits>(b.horizontal_acceleration) &&
           a.innovation_valid == b.innovation_valid && a.observable == b.observable && a.consistent == b.consistent;
}
template <typename Backend> auto seeded(Context & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    Heading<Backend> monitor;
    auto const acceleration = Vector<Backend>::from_row_major({Scalar{2}, Scalar{-4}, Scalar{100}});
    auto const field = Vector<Backend>::from_row_major({Scalar{20}, Scalar{2}, Scalar{40}});
    test.expect(monitor.update_acceleration(acceleration, Scalar{1}) == Status::success &&
                    monitor.update(Quaternion<Backend>{}, field, north<Backend>(), parameters<Backend>(), true, true) ==
                        Status::success &&
                    monitor.state().consistent && monitor.state().observable &&
                    monitor.state().filtered_innovation != Scalar{0},
                profile, "nonzero private acceleration and heading histories establish a meaningful baseline");
    return monitor;
}

template <typename Backend> void acceleration(Context & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    Heading<Backend> monitor;
    auto const empty = monitor.state();
    test.expect(same(empty, typename Heading<Backend>::State{}), profile, "default state is entirely empty");
    std::array<std::array<Scalar, 4>, 4> const samples{{{Scalar{2}, Scalar{0}, Scalar{100}, Scalar{1}},
                                                        {Scalar{-3}, Scalar{4}, Scalar{-200}, Scalar{3}},
                                                        {Scalar{2}, Scalar{-3}, Scalar{999}, Scalar{1}},
                                                        {Scalar{0}, Scalar{0}, Scalar{-999}, Scalar{0.25}}}};
    long double n = 0, e = 0;
    for (auto const & sample : samples)
    {
        long double const alpha = static_cast<long double>(sample[3]) / (1 + sample[3]);
        n += (sample[0] - n) * alpha;
        e += (sample[1] - e) * alpha;
        auto const vector = Vector<Backend>::from_row_major({sample[0], sample[1], sample[2]});
        test.expect(monitor.update_acceleration(vector, sample[3]) == Status::success && !monitor.state().observable &&
                        std::abs(static_cast<long double>(monitor.state().horizontal_acceleration) -
                                 std::hypot(n, e)) <= 32 * std::numeric_limits<Scalar>::epsilon(),
                    profile, "independent variable-dt vector recurrence precedes horizontal norm");
    }
    // Bounded fixture allowance above covers a four-step scalar recurrence and sqrt,
    // not a general target libm certificate or arbitrary-duration accumulated bound.
    Heading<Backend> opposite;
    for (auto down : {Scalar{-100}, Scalar{0}, Scalar{100}})
    {
        opposite.reset();
        test.expect(opposite.update_acceleration(Vector<Backend>::from_row_major({Scalar{2}, Scalar{0}, down}),
                                                 Scalar{1}) == Status::success &&
                        opposite.state().horizontal_acceleration == Scalar{1} &&
                        opposite.update_acceleration(Vector<Backend>::from_row_major({Scalar{-1}, Scalar{0}, -down}),
                                                     Scalar{1}) == Status::success &&
                        opposite.state().horizontal_acceleration == Scalar{0},
                    profile, "signed vectors cancel exactly; any finite Down contributes no horizontal excitation");
    }
    for (int side : {-1, 0, 1})
    {
        Heading<Backend> boundary;
        auto params = parameters<Backend>();
        params.acceleration_limit = side < 0   ? std::nextafter(Scalar{1}, Scalar{0})
                                    : side > 0 ? std::nextafter(Scalar{1}, Scalar{2})
                                               : Scalar{1};
        test.expect(boundary.update_acceleration(Vector<Backend>::from_row_major({Scalar{2}, Scalar{0}, Scalar{0}}),
                                                 Scalar{1}) == Status::success &&
                        boundary.update(Quaternion<Backend>{}, north<Backend>(), north<Backend>(), params, true,
                                        true) == Status::success &&
                        boundary.state().observable == (side < 0) && boundary.state().consistent == (side < 0),
                    profile, "strict acceleration excitation boundary uses exact known norm one");
    }
}

template <typename Backend> void innovation(Context & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    auto const params = parameters<Backend>();
    for (int sign : {-1, 1})
    {
        Heading<Backend> monitor;
        auto const east = Vector<Backend>::from_row_major({Scalar{0}, static_cast<Scalar>(20 * sign), Scalar{40}});
        long double expected = 0;
        for (unsigned sample = 0; sample < 20; ++sample)
        {
            expected += (sign * std::numbers::pi_v<long double> / 2 - expected) / 10;
            test.expect(monitor.update(Quaternion<Backend>{}, east, north<Backend>(), params, true, false) ==
                                Status::success &&
                            monitor.state().innovation_valid &&
                            std::abs(static_cast<long double>(monitor.state().innovation) -
                                     sign * std::numbers::pi_v<long double> / 2) <=
                                8 * std::numeric_limits<Scalar>::epsilon() &&
                            std::abs(static_cast<long double>(monitor.state().filtered_innovation) - expected) <=
                                128 * std::numeric_limits<Scalar>::epsilon() &&
                            !monitor.state().consistent,
                        profile, "signed heading and one update per magnetic sample follow independent recurrence");
        }
        for (unsigned sample = 0; sample < 100; ++sample)
        {
            test.expect(monitor.update(Quaternion<Backend>{}, north<Backend>(), north<Backend>(), params, true, true) ==
                            Status::success,
                        profile, "consume good recovery samples without excitation");
        }
        test.expect(!monitor.state().consistent, profile,
                    "small innovation without excitation cannot acquire consistency");
        test.expect(monitor.update_acceleration(Vector<Backend>::from_row_major({Scalar{2}, Scalar{0}, Scalar{0}}),
                                                Scalar{1}) == Status::success &&
                        monitor.update(Quaternion<Backend>{}, north<Backend>(), north<Backend>(), params, true,
                                       false) == Status::success &&
                        !monitor.state().consistent && !monitor.state().observable,
                    profile, "excitation without aiding cannot acquire");
        test.expect(monitor.update(Quaternion<Backend>{}, north<Backend>(), north<Backend>(), params, true, true) ==
                            Status::success &&
                        monitor.state().consistent,
                    profile, "renewed independent aiding and excitation recover consistency");
        monitor.reset_acceleration();
        test.expect(monitor.update(Quaternion<Backend>{}, north<Backend>(), north<Backend>(), params, true, true) ==
                            Status::success &&
                        monitor.state().consistent && !monitor.state().observable,
                    profile, "low-excitation history retains acquired latch");
        auto const field = Vector<Backend>::from_row_major({Scalar{-20}, static_cast<Scalar>(-sign), Scalar{40}});
        auto const reference = Vector<Backend>::from_row_major({Scalar{-20}, static_cast<Scalar>(sign), Scalar{40}});
        test.expect(monitor.update(Quaternion<Backend>{}, field, reference, params, true, true) == Status::success &&
                        std::abs(static_cast<long double>(monitor.state().innovation) -
                                 sign * 2 * std::atan2(1.0L, 20.0L)) <= 8 * std::numeric_limits<Scalar>::epsilon(),
                    profile, "heading residual wraps across both pi cuts");
    }
    Quaternion<Backend> cyclic, negative;
    test.expect(Quaternion<Backend>::from_coefficients(Scalar{0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.5},
                                                       Scalar{0.125}, cyclic) == Status::success &&
                    Quaternion<Backend>::from_coefficients(Scalar{-0.5}, Scalar{-0.5}, Scalar{-0.5}, Scalar{-0.5},
                                                           Scalar{0.125}, negative) == Status::success,
                profile, "exact tilted quaternion pair");
    auto const body = Vector<Backend>::from_row_major({Scalar{20}, Scalar{40}, Scalar{0}});
    for (auto const & q : {cyclic, negative})
    {
        Heading<Backend> monitor;
        test.expect(monitor.update(q, body, north<Backend>(), params, true, true) == Status::success &&
                        std::abs(static_cast<long double>(monitor.state().innovation) -
                                 std::numbers::pi_v<long double> / 2) <= 8 * std::numeric_limits<Scalar>::epsilon(),
                    profile, "independent cyclic map produces East heading for either quaternion sign");
    }
    Heading<Backend> probe;
    auto const east = Vector<Backend>::from_row_major({Scalar{0}, Scalar{20}, Scalar{40}});
    test.expect(probe.update(Quaternion<Backend>{}, east, north<Backend>(), params, true, false) == Status::success,
                profile, "record represented first residual for comparator-only boundary");
    Scalar const threshold = probe.state().filtered_innovation;
    for (int side : {-1, 0, 1})
    {
        Heading<Backend> monitor;
        monitor.reset_heading();
        auto boundary = params;
        boundary.heading_noise = side < 0   ? std::nextafter(threshold, Scalar{0})
                                 : side > 0 ? std::nextafter(threshold, Scalar{1})
                                            : threshold;
        test.expect(monitor.update(Quaternion<Backend>{}, east, north<Backend>(), boundary, true, false) ==
                            Status::success &&
                        monitor.state().consistent == (side > 0),
                    profile, "absolute filtered innovation equal to or above noise clears latch; below preserves it");
    }
}

template <typename Backend> void weak_field(Context & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    auto const baseline = seeded<Backend>(test, profile);
    auto const params = parameters<Backend>();
    for (bool weak_reference : {false, true})
    {
        for (int side : {-1, 0, 1})
        {
            auto monitor = baseline;
            Scalar const h = side < 0   ? std::nextafter(Scalar{5}, Scalar{0})
                             : side > 0 ? std::nextafter(Scalar{5}, Scalar{6})
                                        : Scalar{5};
            auto const weak = Vector<Backend>::from_row_major({h, Scalar{0}, Scalar{40}});
            test.expect(
                monitor.update(Quaternion<Backend>{}, weak_reference ? north<Backend>() : weak,
                               weak_reference ? weak : north<Backend>(), params, true, true) == Status::success &&
                    monitor.state().innovation_valid == (side >= 0) && monitor.state().consistent == (side >= 0),
                profile, "both horizontal norms require inclusive minimum equality");
            if (side < 0)
            {
                test.expect(monitor.state().innovation == Scalar{0} &&
                                monitor.state().filtered_innovation == baseline.state().filtered_innovation,
                            profile, "weak field clears current innovation but preserves filtered residual");
                test.expect(monitor.update(Quaternion<Backend>{}, north<Backend>(), north<Backend>(), params, true,
                                           true) == Status::success &&
                                monitor.state().consistent,
                            profile, "valid field with aiding and excitation recovers after weak geometry");
            }
        }
    }
    auto monitor = baseline;
    auto const huge = Vector<Backend>::from_row_major({std::numeric_limits<Scalar>::max(), Scalar{0}, Scalar{0}});
    test.expect(monitor.update(Quaternion<Backend>{}, huge, huge, params, false, true) == Status::success &&
                    !monitor.state().innovation_valid && !monitor.state().consistent &&
                    monitor.state().innovation == Scalar{0} && monitor.state().filtered_innovation == Scalar{0} &&
                    monitor.state().horizontal_acceleration == baseline.state().horizontal_acceleration,
                profile, "unaligned update validates finite inputs then skips geometry and clears heading history");
}

template <typename Backend> void resets(Context & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    auto baseline = seeded<Backend>(test, profile);
    auto monitor = baseline;
    auto expected = baseline.state();
    expected.horizontal_acceleration = Scalar{0};
    expected.observable = false;
    monitor.reset_acceleration();
    test.expect(same(monitor.state(), expected), profile,
                "acceleration reset changes only acceleration and observability");
    monitor = baseline;
    expected = baseline.state();
    expected.innovation = Scalar{0};
    expected.filtered_innovation = Scalar{0};
    expected.innovation_valid = false;
    expected.consistent = true;
    monitor.reset_heading();
    test.expect(same(monitor.state(), expected) && same(monitor.state(), expected), profile,
                "heading reset clears only heading history and explicitly establishes consistency; queries pure");
    monitor.reset();
    test.expect(same(monitor.state(), typename Heading<Backend>::State{}), profile,
                "full reset returns false latch and zero histories");
    Heading<Backend> fresh;
    auto const sample = Vector<Backend>::from_row_major({Scalar{1}, Scalar{2}, Scalar{3}});
    test.expect(monitor.update_acceleration(sample, Scalar{1}) == fresh.update_acceleration(sample, Scalar{1}) &&
                    same(monitor.state(), fresh.state()),
                profile, "full reset also clears private vector as common continuation confirms");
}

template <typename Backend> void failures(Context & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    using Monitor = Heading<Backend>;
    using Q = Quaternion<Backend>;
    auto const baseline = seeded<Backend>(test, profile);
    auto const params = parameters<Backend>();
    auto unchanged = [&](Monitor monitor, auto action, Status expected)
    {
        auto control = baseline;
        test.expect(action(monitor) == expected && same(monitor.state(), baseline.state()), profile,
                    "failed operation preserves every public history field");
        auto const next = Vector<Backend>::from_row_major({Scalar{-2}, Scalar{1}, Scalar{-100}});
        test.expect(monitor.update_acceleration(next, Scalar{0.5}) == control.update_acceleration(next, Scalar{0.5}) &&
                        monitor.update(Q{}, north<Backend>(), north<Backend>(), params, true, true) ==
                            control.update(Q{}, north<Backend>(), north<Backend>(), params, true, true) &&
                        same(monitor.state(), control.state()),
                    profile, "common acceleration/heading continuation detects private history mutation");
    };
    for (auto invalid : {std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity(),
                         -std::numeric_limits<Scalar>::infinity()})
    {
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            auto bad = north<Backend>();
            bad(axis) = invalid;
            unchanged(
                baseline, [&](auto & m) { return m.update_acceleration(bad, Scalar{1}); }, Status::non_finite_input);
            for (bool aligned : {false, true})
            {
                unchanged(
                    baseline, [&](auto & m) { return m.update(Q{}, bad, north<Backend>(), params, aligned, true); },
                    Status::non_finite_input);
                unchanged(
                    baseline, [&](auto & m) { return m.update(Q{}, north<Backend>(), bad, params, aligned, true); },
                    Status::non_finite_input);
                for (std::size_t field = 0; field < 3; ++field)
                {
                    auto p = params;
                    std::array<Scalar *, 3> fields{&p.heading_noise, &p.acceleration_limit,
                                                   &p.minimum_horizontal_field};
                    *fields[field] = invalid;
                    unchanged(
                        baseline,
                        [&](auto & m) { return m.update(Q{}, north<Backend>(), north<Backend>(), p, aligned, true); },
                        Status::non_finite_input);
                }
            }
        }
        unchanged(
            baseline, [&](auto & m) { return m.update_acceleration(north<Backend>(), invalid); },
            Status::non_finite_input);
    }
    for (auto dt : {Scalar{0}, Scalar{-1}, std::nextafter(std::numeric_limits<Scalar>::min(), Scalar{0})})
    {
        unchanged(
            baseline, [&](auto & m) { return m.update_acceleration(north<Backend>(), dt); }, Status::domain_error);
    }
    for (auto dt : {std::numeric_limits<Scalar>::min(), std::numeric_limits<Scalar>::max()})
    {
        Monitor monitor;
        auto const zero = Vector<Backend>{};
        test.expect(monitor.update_acceleration(zero, dt) == Status::success &&
                        monitor.state().horizontal_acceleration == Scalar{0},
                    profile, "positive normal dt endpoints accepted for finite zero recurrence");
    }
    for (bool aligned : {false, true})
    {
        for (auto p : {typename Monitor::Parameters{}, typename Monitor::Parameters{Scalar{0}, Scalar{0}, Scalar{1}},
                       typename Monitor::Parameters{Scalar{4}, Scalar{0}, Scalar{1}},
                       typename Monitor::Parameters{Scalar{1}, Scalar{-1}, Scalar{1}},
                       typename Monitor::Parameters{Scalar{1}, Scalar{0},
                                                    std::nextafter(std::numeric_limits<Scalar>::min(), Scalar{0})}})
        {
            unchanged(
                baseline, [&](auto & m) { return m.update(Q{}, north<Backend>(), north<Backend>(), p, aligned, true); },
                Status::domain_error);
        }
    }
    auto const huge = Vector<Backend>::from_row_major({std::numeric_limits<Scalar>::max(), Scalar{0}, Scalar{0}});
    unchanged(
        baseline, [&](auto & m) { return m.update_acceleration(huge, Scalar{1}); }, Status::non_finite_result);
    unchanged(
        baseline, [&](auto & m) { return m.update(Q{}, huge, north<Backend>(), params, true, true); },
        Status::non_finite_result);
    unchanged(
        baseline, [&](auto & m) { return m.update(Q{}, north<Backend>(), huge, params, true, true); },
        Status::non_finite_result);
    Scalar const tiny =
        std::ldexp(Scalar{1}, (std::numeric_limits<Scalar>::min_exponent - std::numeric_limits<Scalar>::digits) / 2);
    Q malformed;
    test.expect(Q::from_coefficients(tiny, tiny / Scalar{2}, Scalar{0}, Scalar{0}, std::numeric_limits<Scalar>::min(),
                                     malformed) == Status::success,
                profile, "public quaternion guard witness");
    for (unsigned step = 0;
         step < 32 && std::isfinite(formal_eskf::linalg::dot(malformed.coefficients(), malformed.coefficients()));
         ++step)
    {
        malformed = malformed * malformed;
    }
    unchanged(
        baseline, [&](auto & m) { return m.update(malformed, north<Backend>(), north<Backend>(), params, true, true); },
        Status::non_finite_result);
    for (unsigned step = 0; step < 4 && formal_eskf::linalg::all_finite(malformed.coefficients()); ++step)
    {
        malformed = malformed * malformed;
    }
    for (bool aligned : {false, true})
    {
        unchanged(
            baseline,
            [&](auto & m) { return m.update(malformed, north<Backend>(), north<Backend>(), params, aligned, true); },
            Status::non_finite_input);
    }
    // Valid bounded StandardMath inputs expose no native sqrt/atan2 provider
    // failure witness; propagation remains a scoped scalar/formal obligation.
}

template <typename Backend> void ownership(Context & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    auto const params = parameters<Backend>();
    Heading<Backend> immediate, idle;
    auto const specific_force = Vector<Backend>::from_row_major({Scalar{2}, Scalar{0}, Scalar{100}});
    auto const east = Vector<Backend>::from_row_major({Scalar{0}, Scalar{20}, Scalar{40}});
    for (unsigned sample = 0; sample < 8; ++sample)
    {
        test.expect(immediate.update_acceleration(specific_force, Scalar{0.5}) == Status::success &&
                        idle.update_acceleration(specific_force, Scalar{0.5}) == Status::success,
                    profile, "equal consumed IMU intervals");
        auto const before = idle.state();
        for (unsigned queries = 0; queries < 50; ++queries)
        {
            test.expect(same(idle.state(), before), profile,
                        "external idle/read operations consume no hidden filter ticks");
        }
        test.expect(
            immediate.update(Quaternion<Backend>{}, east, north<Backend>(), params, true, true) == Status::success &&
                idle.update(Quaternion<Backend>{}, east, north<Backend>(), params, true, true) == Status::success &&
                same(immediate.state(), idle.state()),
            profile, "identical consumed sequence has identical history regardless of external idle");
    }
    auto const heading = idle.state().filtered_innovation;
    idle.reset_acceleration();
    test.expect(idle.state().filtered_innovation == heading && idle.state().horizontal_acceleration == Scalar{0},
                profile, "caller declares an IMU gap with explicit acceleration reset, preserving magnetic history");
    idle.reset();
    Heading<Backend> fresh;
    test.expect(idle.update(Quaternion<Backend>{}, north<Backend>(), north<Backend>(), params, true, true) ==
                        fresh.update(Quaternion<Backend>{}, north<Backend>(), north<Backend>(), params, true, true) &&
                    same(idle.state(), fresh.state()) && !idle.state().consistent,
                profile,
                "caller source/frame reset creates a fresh session without inventing successful heading alignment");
}
} /* end namespace */
int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Case;
    using Float = formal_eskf::test::Backend<float>;
    using Double = formal_eskf::test::Backend<double>;
    Case const cases[] = {
        {"TC-R-MAGNETIC-HEADING-01", "float", acceleration<Float>},
        {"TC-R-MAGNETIC-HEADING-01", "double", acceleration<Double>},
        {"TC-R-MAGNETIC-HEADING-02", "float", innovation<Float>},
        {"TC-R-MAGNETIC-HEADING-02", "double", innovation<Double>},
        {"TC-R-MAGNETIC-HEADING-03", "float", weak_field<Float>},
        {"TC-R-MAGNETIC-HEADING-03", "double", weak_field<Double>},
        {"TC-R-MAGNETIC-HEADING-04", "float", resets<Float>},
        {"TC-R-MAGNETIC-HEADING-04", "double", resets<Double>},
        {"TC-R-MAGNETIC-HEADING-05", "float", failures<Float>},
        {"TC-R-MAGNETIC-HEADING-05", "double", failures<Double>},
        {"TC-R-MAGNETIC-HEADING-06", "float", ownership<Float>},
        {"TC-R-MAGNETIC-HEADING-06", "double", ownership<Double>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
