/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this file.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <array>
#include <cfenv>
#include <cmath>
#include <limits>
#include <type_traits>

#include <formal_eskf/eskf/prediction.hpp>
#include <formal_eskf/runtime/accelerometer.hpp>
#include <formal_eskf/status.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{
using formal_eskf::Status;
using formal_eskf::test::Context;

void status_meanings(Context & test, std::string_view profile)
{
    test.expect(formal_eskf::succeeded(Status::success), profile, "success is accepted");
    for (Status status : {Status::non_finite_input, Status::domain_error, Status::out_of_range,
                          Status::zero_or_unsafe_divisor, Status::invalid_quaternion_norm,
                          Status::not_positive_definite, Status::ill_conditioned, Status::non_finite_result})
    {
        test.expect(!formal_eskf::succeeded(status), profile, "each declared failure remains a failure");
    }
}

template <typename Scalar> void numerical_profile(Context & test, std::string_view profile)
{
    using limits = std::numeric_limits<Scalar>;
    constexpr bool binary32 = std::is_same_v<Scalar, float>;
    test.expect(limits::is_iec559 && limits::radix == 2 && limits::digits == (binary32 ? 24 : 53) &&
                    limits::max_exponent == (binary32 ? 128 : 1024),
                profile, "selected IEEE binary format");
    test.expect(std::fegetround() == FE_TONEAREST, profile, "round-to-nearest execution mode");
    volatile Scalar one = Scalar{1};
    volatile Scalar half_ulp = limits::epsilon() / Scalar{2};
    volatile Scalar odd = Scalar{1} + limits::epsilon();
    test.expect(one + half_ulp == Scalar{1} && odd + half_ulp == Scalar{1} + Scalar{2} * limits::epsilon(), profile,
                "opposite halfway ties both round to an even significand");
    volatile Scalar minimum = limits::min();
    volatile Scalar subnormal = limits::denorm_min();
    test.expect(minimum * Scalar{0.5} > Scalar{0} &&
                    subnormal * std::ldexp(Scalar{1}, limits::digits) == limits::min() * Scalar{2},
                profile, "this host evidence profile preserves subnormal outputs and inputs");
    volatile Scalar nan = limits::quiet_NaN();
    volatile Scalar infinity = limits::infinity();
    test.expect(std::isnan(nan) && !std::isfinite(infinity) && !std::isfinite(-infinity), profile,
                "exceptional classification survives compiler optimization");
#if defined(__FAST_MATH__)
    test.expect(false, profile, "fast-math is outside this evidence profile");
#else
    test.expect(true, profile, "fast-math is disabled; contraction flags require the build record");
#endif
}

template <typename Linalg> void independent_instances(Context & test, std::string_view profile)
{
    using scalar_type = typename Linalg::value_type;
    using types = formal_eskf::EskfTypes<Linalg, formal_eskf::configuration::Ins>;
    using state_type = typename types::nominal_state_type;
    using magnitude_type = formal_eskf::runtime::AccelerometerMagnitude<Linalg>;
    typename types::parameter_type parameters{};
    parameters.minimum_quaternion_norm = scalar_type{0.5};
    parameters.quaternion_squared_norm_tolerance = scalar_type{0.25};
    parameters.dt_min = scalar_type{0.125};
    parameters.dt_max = scalar_type{0.5};
    auto state_matches = [](state_type const & state, scalar_type position, scalar_type velocity)
    {
        bool matches = state.q_nb.q0() == scalar_type{1} && state.q_nb.q1() == scalar_type{0} &&
                       state.q_nb.q2() == scalar_type{0} && state.q_nb.q3() == scalar_type{0};
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            matches = matches && state.p_n(axis) == (axis == 0 ? position : scalar_type{0}) &&
                      state.v_n(axis) == (axis == 0 ? velocity : scalar_type{0}) && state.b_a(axis) == scalar_type{0} &&
                      state.b_g(axis) == scalar_type{0};
        }
        return matches;
    };
    typename types::imu_sample_type imu{};
    imu.specific_force_b(0) = scalar_type{2};
    for (bool interleave : {false, true})
    {
        // Reconstruct twice; no hidden global filter state may survive restart.
        for (unsigned repeat = 0; repeat < 2; ++repeat)
        {
            state_type state{}, other{};
            state.p_n(0) = scalar_type{3};
            state.v_n(0) = scalar_type{1};
            magnitude_type magnitude, control, other_magnitude;
            for (unsigned step = 1; step <= 8; ++step)
            {
                auto output = state;
                auto const rejected = formal_eskf::predict_nominal(state, imu, scalar_type{0}, parameters, output);
                test.expect(rejected == Status::out_of_range && state_matches(output, state.p_n(0), state.v_n(0)),
                            profile, "rejected step does not advance the sequence");
                if (interleave)
                {
                    auto other_imu = imu;
                    for (std::size_t axis = 0; axis < 3; ++axis)
                    {
                        scalar_type const coordinate = static_cast<scalar_type>(axis + 1);
                        other_imu.specific_force_b(axis) = -scalar_type{4} * coordinate;
                        other_imu.angular_rate_b(axis) = scalar_type{0.125} * coordinate;
                        other.b_a(axis) = scalar_type{0.25} * coordinate;
                        other.b_g(axis) = -scalar_type{0.0625} * coordinate;
                    }
                    test.expect(formal_eskf::predict_nominal(other, other_imu, scalar_type{0.25}, parameters, other) ==
                                    Status::success,
                                profile, "independent estimator advances with different data");
                    test.expect(other_magnitude.update(other_imu.specific_force_b).status == Status::success, profile,
                                "independent runtime filter advances with different data");
                    if (step % 2 == 0)
                    {
                        other_magnitude.reset();
                    }
                }
                test.expect(formal_eskf::predict_nominal(state, imu, scalar_type{0.25}, parameters, state) ==
                                Status::success,
                            profile, "valid step recovers and advances");
                scalar_type const time = static_cast<scalar_type>(step) * scalar_type{0.25};
                test.expect(
                    state_matches(state, scalar_type{3} + time + time * time, scalar_type{1} + scalar_type{2} * time),
                    profile, "exact constant-acceleration trajectory is independent of other instances");
                auto sample = imu.specific_force_b;
                sample(1) = static_cast<scalar_type>(step);
                auto const result = magnitude.update(sample);
                auto const expected = control.update(sample);
                test.expect(result.status == expected.status &&
                                result.within_gravity_band == expected.within_gravity_band &&
                                result.squared_norm == expected.squared_norm,
                            profile, "interleaving does not change runtime diagnostics");
                for (std::size_t axis = 0; axis < 3; ++axis)
                {
                    test.expect(magnitude.filtered_specific_force_b()(axis) ==
                                    control.filtered_specific_force_b()(axis),
                                profile, "interleaving preserves each private filter history observation");
                }
                sample(0) = std::numeric_limits<scalar_type>::quiet_NaN();
                test.expect(magnitude.update(sample).status == Status::non_finite_input, profile,
                            "failed sample is not committed to the filter");
            }
            magnitude.reset();
            control.reset();
            auto const result = magnitude.update(imu.specific_force_b);
            test.expect(!control.initialized() && result.status == Status::success &&
                            result.squared_norm == scalar_type{4},
                        profile, "explicit reset reseeds independently");
        }
    }
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Backend;
    formal_eskf::test::Case const cases[] = {
        {"TC-C-FAILURE-STATUS", "portable", status_meanings},
        {"TC-C-NUMERICAL-PROFILE-MODES", "binary32", numerical_profile<float>},
        {"TC-C-NUMERICAL-PROFILE-MODES", "binary64", numerical_profile<double>},
        {"TC-C-CPP-SUBSET-INDEPENDENCE", "INS.binary32", independent_instances<Backend<float>>},
        {"TC-C-CPP-SUBSET-INDEPENDENCE", "INS.binary64", independent_instances<Backend<double>>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
