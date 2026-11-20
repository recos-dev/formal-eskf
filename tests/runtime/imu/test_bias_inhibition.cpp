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
#include <string_view>

#include <formal_eskf/runtime/imu.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{
using formal_eskf::Status;
using TestContext = formal_eskf::test::Context;
constexpr std::array<bool, 3U> all{true, true, true};
constexpr std::array<bool, 3U> none{false, false, false};

template <typename Linalg> void peaks(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    using Policy = formal_eskf::runtime::ImuBiasInhibition<Linalg>;
    typename Policy::Parameters const parameters{Scalar{25}, Scalar{3}, Scalar{0.5}, true, true};
    typename Policy::state_type state;
    state.b_a(2U) = Scalar{-2};
    state.b_g(0U) = Scalar{1};
    formal_eskf::ImuSample<Linalg> spike, quiet;
    spike.specific_force_b(2U) = Scalar{-52};
    spike.angular_rate_b(0U) = Scalar{7};
    quiet.specific_force_b(2U) = Scalar{-12};
    quiet.angular_rate_b(0U) = Scalar{1};
    Policy policy(parameters);
    auto result = policy.update(Scalar{0.125}, Scalar{0.25}, state, spike, {});
    test.expect(result.status == Status::success && result.specific_force_peak == Scalar{50} &&
                    result.angular_rate_peak == Scalar{6} && result.high_dynamics &&
                    result.bias_update.accelerometer == none && result.bias_update.gyroscope == all,
                profile, "first real impulse seeds both bias-subtracted peaks without a startup delay");
    std::array<Scalar, 3> const forces{Scalar{25}, Scalar{12.5}, Scalar{10}};
    std::array<Scalar, 3> const rates{Scalar{4.5}, Scalar{3.375}, Scalar{2.53125}};
    for (std::size_t step = 0; step < forces.size(); ++step)
    {
        result = policy.update(Scalar{0.125}, Scalar{0.25}, state, quiet, {});
        test.expect(result.specific_force_peak == forces[step] && result.angular_rate_peak == rates[step] &&
                        result.high_dynamics == (step < 2),
                    profile, "independent exact dyadic recurrence uses force and angular periods separately");
    }
    for (auto period : {Scalar{0.25}, Scalar{0.5}, Scalar{1}})
    {
        Policy timed(parameters);
        test.expect(timed.update(Scalar{0.125}, Scalar{0.125}, state, spike, {}).status == Status::success, profile,
                    "seed before tau partition");
        result = timed.update(period, period, state, quiet, {});
        test.expect(result.status == Status::success &&
                        result.specific_force_peak == (period < Scalar{0.5} ? Scalar{25} : Scalar{10}) &&
                        result.angular_rate_peak == (period < Scalar{0.5} ? Scalar{3} : Scalar{0}) &&
                        !result.high_dynamics,
                    profile, "below/equal/above tau decays only old evidence");
    }
    for (bool angular : {false, true})
    {
        Scalar const limit = angular ? Scalar{3} : Scalar{25};
        for (auto magnitude :
             {std::nextafter(limit, Scalar{0}), limit, std::nextafter(limit, std::numeric_limits<Scalar>::infinity())})
        {
            Policy threshold(parameters);
            formal_eskf::ImuSample<Linalg> sample;
            (angular ? sample.angular_rate_b : sample.specific_force_b)(0U) = magnitude;
            result = threshold.update(Scalar{0.125}, Scalar{0.125}, {}, sample, {true, false, false, false, 0});
            test.expect(result.status == Status::success && result.high_dynamics == (magnitude > limit) &&
                            (angular ? result.angular_rate_peak : result.specific_force_peak) == magnitude &&
                            result.bias_update.accelerometer == (magnitude > limit ? none : all),
                        profile, "one-axis exact norm distinguishes strict limit equality and adjacent scalar inputs");
        }
    }
}

template <typename Linalg> void test_permissions(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    using Policy = formal_eskf::runtime::ImuBiasInhibition<Linalg>;
    using Quaternion = typename Policy::state_type::quaternion_type;
    // Exact unit quaternions mapping body X, Y or Z onto navigation Down.
    std::array<std::array<Scalar, 4U>, 3U> const attitudes{{{Scalar{0.5}, Scalar{-0.5}, Scalar{-0.5}, Scalar{-0.5}},
                                                            {Scalar{0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.5}},
                                                            {Scalar{1}, Scalar{0}, Scalar{0}, Scalar{0}}}};
    for (std::size_t down_axis = 0U; down_axis < attitudes.size(); ++down_axis)
    {
        typename Policy::state_type state;
        auto const & q = attitudes[down_axis];
        test.expect(Quaternion::from_coefficients(q[0], q[1], q[2], q[3], Scalar{0.125}, state.q_nb) == Status::success,
                    profile, "exact axis-aligned attitude fixture");
        for (unsigned flags = 0U; flags < 128U; ++flags)
        {
            bool const accel_enabled = (flags & 1U) != 0U;
            bool const gyro_enabled = (flags & 2U) != 0U;
            bool const at_rest = (flags & 4U) != 0U;
            bool const fake_height = (flags & 8U) != 0U;
            bool const fake_position = (flags & 16U) != 0U;
            bool const vertical_fault = (flags & 32U) != 0U;
            bool const high_dynamics = (flags & 64U) != 0U;
            Policy policy({Scalar{25}, Scalar{3}, Scalar{0.5}, accel_enabled, gyro_enabled});
            formal_eskf::ImuSample<Linalg> imu;
            imu.specific_force_b(2U) = high_dynamics ? Scalar{-50} : Scalar{-10};
            for (int clipping = 0; clipping < 8; ++clipping)
            {
                auto const result = policy.update(Scalar{0.5}, Scalar{0.5}, state, imu,
                                                  {at_rest, fake_height, fake_position, vertical_fault, clipping});
                test.expect(result.status == Status::success && result.high_dynamics == high_dynamics, profile,
                            "switches and observability modes do not bypass motion detection");
                for (std::size_t axis = 0U; axis < 3U; ++axis)
                {
                    bool const expected = accel_enabled && !high_dynamics && !vertical_fault &&
                                          (clipping & (1 << axis)) == 0 &&
                                          (at_rest || (!fake_height && (!fake_position || axis == down_axis)));
                    test.expect(result.bias_update.accelerometer[axis] == expected &&
                                    result.bias_update.gyroscope[axis] == gyro_enabled,
                                profile,
                                "complete permission combinations preserve rest precedence and body-axis masks");
                }
            }
        }
    }
}

template <typename Linalg> void observability(TestContext & test, std::string_view profile)
{
    test_permissions<Linalg>(test, profile);
    using Scalar = typename Linalg::value_type;
    using Policy = formal_eskf::runtime::ImuBiasInhibition<Linalg>;
    using Quaternion = typename Policy::state_type::quaternion_type;
    Scalar const threshold = static_cast<Scalar>(0.966);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        for (int sign : {-1, 1})
        {
            for (int side : {-1, 1})
            {
                long double const target =
                    static_cast<long double>(sign) * (0.966L + static_cast<long double>(side) * 0.0001L);
                // R_zx=-sin(pitch), R_zy=sin(roll), R_zz=cos(roll).
                long double const angle = axis == 2 ? std::acos(target) : std::asin(axis == 0 ? -target : target);
                Scalar const c = static_cast<Scalar>(std::cos(angle / 2));
                Scalar const s = static_cast<Scalar>(std::sin(angle / 2));
                typename Policy::state_type state;
                test.expect(Quaternion::from_coefficients(c, axis == 0 ? Scalar{0} : s, axis == 0 ? s : Scalar{0},
                                                          Scalar{0}, Scalar{0.125}, state.q_nb) == Status::success,
                            profile, "prepare proper oblique unit attitude through public normalization");
                auto const q = state.q_nb.coefficients();
                long double const a = q(0U), x = q(1U), y = q(2U);
                long double const projection = axis == 0 ? -2 * a * y : axis == 1 ? 2 * a * x : a * a - x * x;
                // The chosen margin exceeds 8 epsilon for these bounded products, so roundoff
                // cannot move this fixture onto the other side of the encoded threshold.
                test.expect(std::abs(std::abs(projection) - static_cast<long double>(threshold)) >
                                    8 * std::numeric_limits<Scalar>::epsilon() &&
                                (std::abs(projection) > threshold) == (side > 0),
                            profile,
                            "independent coefficient oracle certifies the chosen side of the observability threshold");
                Policy policy({Scalar{25}, Scalar{3}, Scalar{0.5}, true, true});
                auto const result = policy.update(Scalar{0.5}, Scalar{0.5}, state, {}, {false, false, true, false, 0});
                test.expect(result.status == Status::success && result.bias_update.accelerometer[axis] == (side > 0),
                            profile, "signed tilted axes use absolute Down projection with strict observability");
            }
        }
    }
    // Exact abs(R_zi)==encoded 0.966 through a public unit quaternion has not been
    // established by these fixtures. That endpoint remains the imu-policies ESBMC
    // comparison obligation; this native family covers separated one-sided inputs.
}

template <typename Matrix> bool same_bits(Matrix const & a, Matrix const & b)
{
    using Bits = std::array<std::byte, sizeof(typename Matrix::value_type)>;
    for (std::size_t i = 0; i < Matrix::row_count; ++i)
    {
        if (std::bit_cast<Bits>(a(i)) != std::bit_cast<Bits>(b(i)))
        {
            return false;
        }
    }
    return true;
}

template <typename Linalg> void failures(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    using Policy = formal_eskf::runtime::ImuBiasInhibition<Linalg>;
    typename Policy::Parameters const parameters{Scalar{25}, Scalar{3}, Scalar{0.5}, true, true};
    typename Policy::state_type original_state;
    original_state.p_n = decltype(original_state.p_n)::from_row_major({Scalar{1}, -Scalar{0}, Scalar{-2}});
    original_state.v_n = decltype(original_state.v_n)::from_row_major({Scalar{-3}, Scalar{4}, -Scalar{0}});
    formal_eskf::ImuSample<Linalg> spike;
    spike.specific_force_b(2U) = Scalar{-50};
    spike.angular_rate_b(0U) = Scalar{6};
    Policy baseline(parameters);
    test.expect(baseline.update(Scalar{0.125}, Scalar{0.25}, original_state, spike, {}).status == Status::success,
                profile, "reachable two-envelope history");
    auto check = [&](Policy policy, typename Policy::state_type const & state,
                     formal_eskf::ImuSample<Linalg> const & imu, Scalar angular, Scalar force,
                     typename Policy::Conditions const & conditions, Status expected)
    {
        auto const old_state = state;
        auto const old_imu = imu;
        auto const result = policy.update(angular, force, state, imu, conditions);
        test.expect(result.status == expected && result.high_dynamics && result.specific_force_peak == Scalar{0} &&
                        result.angular_rate_peak == Scalar{0} && result.bias_update.accelerometer == none &&
                        result.bias_update.gyroscope == none,
                    profile, "failure publishes disabled placeholder permissions");
        test.expect(same_bits(state.p_n, old_state.p_n) && same_bits(state.v_n, old_state.v_n) &&
                        same_bits(state.b_a, old_state.b_a) && same_bits(state.b_g, old_state.b_g) &&
                        same_bits(state.q_nb.coefficients(), old_state.q_nb.coefficients()) &&
                        same_bits(imu.specific_force_b, old_imu.specific_force_b) &&
                        same_bits(imu.angular_rate_b, old_imu.angular_rate_b),
                    profile, "borrowed inputs remain bitwise intact");
        auto const recovery = policy.update(Scalar{0.125}, Scalar{0.25}, original_state, {}, {});
        test.expect(recovery.status == Status::success && recovery.specific_force_peak == Scalar{25} &&
                        recovery.angular_rate_peak == Scalar{4.5},
                    profile, "common valid continuation shows neither peak advanced on failure");
    };
    for (auto invalid : {Scalar{0}, Scalar{-1}, std::numeric_limits<Scalar>::infinity(),
                         -std::numeric_limits<Scalar>::infinity(), std::numeric_limits<Scalar>::quiet_NaN()})
    {
        auto expected = std::isfinite(invalid) ? Status::domain_error : Status::non_finite_input;
        check(baseline, original_state, spike, invalid, Scalar{0.25}, {}, expected);
        check(baseline, original_state, spike, Scalar{0.125}, invalid, {}, expected);
        for (unsigned field = 0; field < 3; ++field)
        {
            auto configured = parameters;
            std::array<Scalar *, 3> const fields{&configured.maximum_specific_force, &configured.maximum_angular_rate,
                                                 &configured.decay_time_constant};
            *fields[field] = invalid;
            Policy bad(configured);
            auto const result = bad.update(Scalar{0.125}, Scalar{0.25}, original_state, spike, {});
            test.expect(result.status == expected && result.bias_update.accelerometer == none &&
                            result.bias_update.gyroscope == none,
                        profile, "each invalid constructor parameter fails closed");
        }
    }
    for (int mask : {-1, 8})
    {
        check(baseline, original_state, spike, Scalar{0.125}, Scalar{0.25}, {true, false, false, false, mask},
              Status::domain_error);
    }
    for (auto invalid : {std::numeric_limits<Scalar>::infinity(), -std::numeric_limits<Scalar>::infinity(),
                         std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::max()})
    {
        for (std::size_t field = 0; field < 4; ++field)
        {
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                auto state = original_state;
                auto imu = spike;
                auto fields = std::array{&state.b_a, &state.b_g, &imu.specific_force_b, &imu.angular_rate_b};
                (*fields[field])(axis) = invalid;
                check(baseline, state, imu, Scalar{0.125}, Scalar{0.25}, {},
                      std::isfinite(invalid) ? Status::non_finite_result : Status::non_finite_input);
            }
        }
    }
    // Public underflow constructor followed by raw products yields a finite
    // quaternion whose quadratic rotation overflows, without private-layout writes.
    Scalar const tiny =
        std::ldexp(Scalar{1}, (std::numeric_limits<Scalar>::min_exponent - std::numeric_limits<Scalar>::digits) / 2);
    auto malformed = original_state;
    test.expect(decltype(malformed.q_nb)::from_coefficients(tiny, tiny * Scalar{0.5}, Scalar{0}, Scalar{0},
                                                            std::numeric_limits<Scalar>::min(),
                                                            malformed.q_nb) == Status::success,
                profile, "public gradual-underflow quaternion fixture");
    for (unsigned count = 0;
         count < 32 && std::isfinite(formal_eskf::linalg::squared_norm(malformed.q_nb.coefficients())); ++count)
    {
        malformed.q_nb = malformed.q_nb * malformed.q_nb;
    }
    check(baseline, malformed, spike, Scalar{0.125}, Scalar{0.25}, {}, Status::non_finite_result);
}

template <typename Linalg> void sessions(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    using Policy = formal_eskf::runtime::ImuBiasInhibition<Linalg>;
    typename Policy::Parameters const parameters{Scalar{25}, Scalar{3}, Scalar{0.5}, true, true};
    Policy active(parameters), idle(parameters), fresh(parameters);
    formal_eskf::ImuSample<Linalg> sample;
    sample.specific_force_b(2U) = Scalar{-50};
    auto first = active.update(Scalar{0.125}, Scalar{0.125}, {}, sample, {});
    auto same = idle.update(Scalar{0.125}, Scalar{0.125}, {}, sample, {});
    test.expect(first.status == Status::success && same.specific_force_peak == Scalar{50}, profile,
                "both sessions consume the same first real sample");
    // No update represents arbitrary external idle time; this API has no wall clock.
    sample.specific_force_b(2U) = Scalar{-10};
    first = active.update(Scalar{0.125}, Scalar{0.125}, {}, sample, {});
    same = idle.update(Scalar{0.125}, Scalar{0.125}, {}, sample, {});
    auto const restart = fresh.update(Scalar{0.125}, Scalar{0.125}, {}, sample, {});
    test.expect(
        first.specific_force_peak == Scalar{37.5} && same.specific_force_peak == Scalar{37.5} && first.high_dynamics &&
            same.high_dynamics && restart.specific_force_peak == Scalar{10} && !restart.high_dynamics,
        profile, "only actual updates decay peaks; a new controller starts at zero and specific force retains gravity");
}
} /* end namespace */

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Case;
    using Float = formal_eskf::test::Backend<float>;
    using Double = formal_eskf::test::Backend<double>;
    Case const cases[] = {
        {"TC-R-IMU-BIAS-INHIBITION-01", "float", peaks<Float>},
        {"TC-R-IMU-BIAS-INHIBITION-01", "double", peaks<Double>},
        {"TC-R-IMU-BIAS-INHIBITION-02", "float", observability<Float>},
        {"TC-R-IMU-BIAS-INHIBITION-02", "double", observability<Double>},
        {"TC-R-IMU-BIAS-INHIBITION-03", "float", failures<Float>},
        {"TC-R-IMU-BIAS-INHIBITION-03", "double", failures<Double>},
        {"TC-R-IMU-BIAS-INHIBITION-04", "float", sessions<Float>},
        {"TC-R-IMU-BIAS-INHIBITION-04", "double", sessions<Double>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
