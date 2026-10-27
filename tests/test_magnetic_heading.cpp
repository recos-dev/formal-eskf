/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/magnetic_heading.hpp>
#include <formal_eskf/so3/rotation_vector.hpp>

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
constexpr std::string_view profile = "magnetic heading";

template <typename Scalar> void run_tests(TestContext & test)
{
    using Linalg = formal_eskf::test::Backend<Scalar>;
    using Monitor = formal_eskf::runtime::MagneticHeading<Linalg>;
    using Vector = typename Monitor::vector_type;
    using Quaternion = formal_eskf::so3::UnitQuaternion<Linalg>;
    typename Monitor::Parameters parameters{static_cast<Scalar>(0.3), static_cast<Scalar>(0.5), Scalar{5}};
    auto const north = Vector::from_row_major({Scalar{20}, Scalar{0}, Scalar{40}});
    auto const east = Vector::from_row_major({Scalar{0}, Scalar{20}, Scalar{40}});
    auto const acceleration = Vector::from_row_major({Scalar{2}, Scalar{0}, Scalar{100}});
    Scalar const pi = formal_eskf::scalar::pi<typename Linalg::scalar_math_type>();
    auto near = [](Scalar a, Scalar b) { return std::abs(a - b) < static_cast<Scalar>(2e-5); };
    Monitor monitor;
    auto observe = [&](Vector const & field, bool aided = true, bool aligned = true)
    { return monitor.update(Quaternion{}, field, north, parameters, aligned, aided); };
    test.expect(observe(north) == Status::success && !monitor.state().observable && !monitor.state().consistent,
                profile, "matching magnetic field alone cannot establish consistency");
    test.expect(monitor.update_acceleration(acceleration, Scalar{1}) == Status::success &&
                    near(monitor.state().horizontal_acceleration, Scalar{1}),
                profile, "one-second acceleration LPF starts at zero and ignores vertical force");
    test.expect(observe(north, false) == Status::success && !monitor.state().consistent && !monitor.state().observable,
                profile, "maneuver without independent horizontal aiding cannot confirm heading");
    test.expect(observe(north) == Status::success && monitor.state().observable && monitor.state().consistent, profile,
                "aided horizontal excitation and low innovation establish consistency");
    monitor.reset_acceleration();
    test.expect(observe(north) == Status::success && !monitor.state().observable && monitor.state().consistent, profile,
                "consistency survives low excitation");
    test.expect(observe(east) == Status::success && near(monitor.state().innovation, pi / Scalar{2}) &&
                    near(monitor.state().filtered_innovation, pi / Scalar{20}) && monitor.state().consistent,
                profile, "positive heading innovation is low-pass filtered before the threshold");
    for (unsigned i = 0U; i < 20U; ++i)
    {
        test.expect(observe(east) == Status::success, profile, "sustained heading disturbance is consumable");
    }
    test.expect(!monitor.state().consistent, profile, "large filtered innovation clears consistency without motion");
    for (unsigned i = 0U; i < 100U; ++i)
    {
        test.expect(observe(north) == Status::success, profile, "recover magnetic measurements");
    }
    test.expect(!monitor.state().consistent, profile, "small residual alone does not requalify after disturbance");
    test.expect(monitor.update_acceleration(acceleration, Scalar{1}) == Status::success &&
                    observe(north) == Status::success && monitor.state().consistent,
                profile, "independent aiding plus renewed excitation requalifies");
    test.expect(observe(north, true, false) == Status::success && !monitor.state().innovation_valid &&
                    !monitor.state().consistent && monitor.state().filtered_innovation == Scalar{0},
                profile, "unaligned heading clears the history");
    monitor.reset_heading();
    test.expect(monitor.state().consistent && !monitor.state().innovation_valid &&
                    near(monitor.state().horizontal_acceleration, Scalar{1}),
                profile, "actual heading reset establishes consistency without fabricating an observation");

    // Strict threshold boundaries in computed arithmetic.
    monitor.reset();
    test.expect(monitor.update_acceleration(acceleration, Scalar{1}) == Status::success, profile, "boundary setup");
    parameters.acceleration_limit = monitor.state().horizontal_acceleration;
    test.expect(observe(north) == Status::success && !monitor.state().observable && !monitor.state().consistent,
                profile, "acceleration equality is not observable");
    parameters.acceleration_limit = std::nextafter(parameters.acceleration_limit, Scalar{0});
    test.expect(observe(north) == Status::success && monitor.state().consistent, profile,
                "the next representable acceleration threshold permits observability");
    monitor.reset();
    test.expect(observe(east) == Status::success, profile, "heading threshold setup");
    parameters.heading_noise = monitor.state().filtered_innovation;
    monitor.reset();
    monitor.reset_heading();
    test.expect(observe(east) == Status::success && !monitor.state().consistent, profile,
                "heading threshold equality is inconsistent");
    parameters.heading_noise = static_cast<Scalar>(0.3);
    parameters.acceleration_limit = static_cast<Scalar>(0.5);

    // Filter a vector, not magnitudes: reversing acceleration cancels.
    monitor.reset();
    test.expect(monitor.update_acceleration(acceleration, Scalar{1}) == Status::success &&
                    monitor.update_acceleration(acceleration * static_cast<Scalar>(-0.5), Scalar{1}) ==
                        Status::success &&
                    monitor.state().horizontal_acceleration == Scalar{0},
                profile, "opposing accelerations cancel before taking the horizontal norm");

    // Tilt-compensated heading is equivalent to yaw minus zero-yaw magnetic heading.
    Quaternion q;
    test.expect(formal_eskf::so3::try_exp<Linalg>(
                    Vector::from_row_major({static_cast<Scalar>(0.4), static_cast<Scalar>(-0.3), Scalar{1}}),
                    static_cast<Scalar>(1e-6), q) == Status::success,
                profile, "tilted quaternion fixture");
    auto const rotation = formal_eskf::so3::to_rotation_matrix(q);
    Scalar const yaw = std::atan2(rotation(1U, 0U), rotation(0U, 0U));
    auto const measured_n = formal_eskf::so3::rotate(q, east);
    Scalar const zero_yaw_x = std::cos(yaw) * measured_n(0U) + std::sin(yaw) * measured_n(1U);
    Scalar const zero_yaw_y = -std::sin(yaw) * measured_n(0U) + std::cos(yaw) * measured_n(1U);
    Scalar const expected = std::remainder(yaw + std::atan2(zero_yaw_y, zero_yaw_x), Scalar{2} * pi);
    test.expect(monitor.update(q, east, north, parameters, true, true) == Status::success &&
                    near(monitor.state().innovation, expected),
                profile, "tilted attitude uses the same heading innovation as the zero-yaw construction");
    Quaternion opposite;
    test.expect(Quaternion::try_from_coefficients(q.coefficients() * Scalar{-1}, static_cast<Scalar>(1e-6), opposite) ==
                        Status::success &&
                    monitor.update(opposite, east, north, parameters, true, true) == Status::success &&
                    near(monitor.state().innovation, expected),
                profile, "quaternion sign does not change heading innovation");
    for (Scalar sign : {Scalar{-1}, Scalar{1}})
    {
        auto const reference = Vector::from_row_major({Scalar{-20}, sign * static_cast<Scalar>(0.1), Scalar{40}});
        auto const field = Vector::from_row_major({Scalar{-20}, -sign * static_cast<Scalar>(0.1), Scalar{40}});
        test.expect(monitor.update(Quaternion{}, field, reference, parameters, true, true) == Status::success &&
                        near(monitor.state().innovation, sign * static_cast<Scalar>(0.01)),
                    profile, "heading difference wraps across both sides of the pi cut");
    }
    monitor.reset_heading();
    auto weak = Vector::from_row_major({Scalar{0}, Scalar{0}, Scalar{40}});
    test.expect(observe(weak) == Status::success && !monitor.state().innovation_valid && !monitor.state().consistent,
                profile, "vertical magnetic field has no valid heading");
    test.expect(monitor.update(Quaternion{}, north, weak, parameters, true, true) == Status::success &&
                    !monitor.state().innovation_valid && !monitor.state().consistent,
                profile, "a weak horizontal reference also invalidates heading");

    monitor.reset_heading();
    auto const saved = monitor.state();
    auto invalid = north;
    invalid.set(0U, std::numeric_limits<Scalar>::quiet_NaN());
    test.expect(observe(invalid) == Status::non_finite_input &&
                    monitor.update_acceleration(invalid, Scalar{1}) == Status::non_finite_input &&
                    monitor.update_acceleration(acceleration, Scalar{0}) == Status::domain_error &&
                    monitor.update_acceleration(acceleration, std::numeric_limits<Scalar>::infinity()) ==
                        Status::non_finite_input &&
                    monitor.update(Quaternion{}, north, north, {}, true, true) == Status::domain_error,
                profile, "reject invalid sensor values, intervals and unconfigured thresholds");
    auto extreme = north;
    extreme.set(0U, std::numeric_limits<Scalar>::max());
    test.expect(observe(extreme) == Status::non_finite_result &&
                    monitor.update_acceleration(extreme, Scalar{1}) == Status::non_finite_result &&
                    monitor.state().consistent == saved.consistent &&
                    monitor.state().filtered_innovation == saved.filtered_innovation &&
                    monitor.state().horizontal_acceleration == saved.horizontal_acceleration &&
                    monitor.state().innovation_valid == saved.innovation_valid,
                profile, "arithmetic failures preserve the preceding history");
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
    std::cout << "Magnetic heading: pass\n";
    return 0;
}
