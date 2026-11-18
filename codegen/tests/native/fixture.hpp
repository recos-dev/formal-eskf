/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include "kernel.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <vector>

namespace codegen_test
{
enum class EdgeCase : unsigned
{
    zero_motion = 64U,
    subnormal_covariance = 65U,
    signed_zero = 66U,
    overflow_cancellation = 67U,
    infinite_covariance = 68U,
    nan_covariance = 69U,
    infinite_rate = 70U,
    nan_dt = 71U,
    zero_dt = 72U,
    negative_noise = 73U,
    invalid_minimum_norm = 74U,
    zero_measurement_variance = 75U,
    indefinite_measurement_noise = 76U,
    asymmetric_prior = 77U,
    negative_prior_diagonal = 78U,
    maximum_prior = 79U,
    maximum_force = 80U,
    maximum_rate = 81U,
    minimum_dt = 82U,
    maximum_dt = 83U,
    zero_observation_gravity = 84U,
    nan_magnetic_observation = 85U,
    infinite_measurement_variance = 86U,
    invalid_norm_tolerance = 87U
};

struct Random
{
    std::uint64_t state;
    template <typename S> S value()
    {
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<S>(static_cast<double>(state >> 11U) / 9007199254740992.0 * 2.0 - 1.0);
    }
};

template <typename S> struct Fixture
{
    using T = Types<S>;
    using Mat = Matrix<S, 15U, 15U>;
    using Vec = Matrix<S, 3U, 1U>;
    typename T::nominal_state_type state;
    Mat P;
    typename T::imu_sample_type imu;
    typename T::parameter_type parameters;
    Matrix<S, 3U, 3U> V;
    Vec gravity, m_n, m_b;
    formal_eskf::configuration::Ins::BiasUpdate bias;
    S dt = S{0.01};
};

template <typename S> auto fixture(unsigned seed)
{
    Fixture<S> f;
    Random random{0x8fae9212U + seed};
    f.parameters.dt_min = S{0.001};
    f.parameters.dt_max = S{2};
    f.parameters.minimum_quaternion_norm = S{1e-6};
    f.parameters.quaternion_squared_norm_tolerance = S{1e-4};
    typename Fixture<S>::Vec angle;
    for (std::size_t i = 0; i < 3U; ++i)
    {
        angle(i) = random.template value<S>() * S{2};
        f.imu.specific_force_b(i) = random.template value<S>() * S{10};
        f.imu.angular_rate_b(i) = random.template value<S>();
        f.parameters.process_noise.angular_rate_variance(i) = S{0.001} * static_cast<S>(i + 1U);
        f.m_n(i) = static_cast<S>(i + 1U) * S{0.1};
        f.m_b(i) = random.template value<S>() * S{0.1};
        f.V(i, i) = S{0.25};
        f.bias.accelerometer[i] = (seed & (1U << i)) != 0U;
        f.bias.gyroscope[i] = (seed & (1U << (i + 3U))) != 0U;
        f.state.p_n(i) = random.template value<S>() * S{10};
        f.state.v_n(i) = random.template value<S>();
        f.state.b_a(i) = random.template value<S>() * S{0.01};
        f.state.b_g(i) = random.template value<S>() * S{0.001};
        f.parameters.process_noise.specific_force_variance(i) = S{0.01} * static_cast<S>(i + 1U);
        f.parameters.process_noise.accelerometer_bias_random_walk_variance_density(i) = S{0.001};
        f.parameters.process_noise.gyroscope_bias_random_walk_variance_density(i) = S{0.0001};
    }
    f.V(0, 1) = f.V(1, 0) = S{0.04};
    f.V(1, 2) = f.V(2, 1) = S{-0.02};
    f.gravity(2) = S{9.80665};
    f.parameters.gravity_n = f.gravity;
    if (static_cast<int>(formal_eskf::so3::exp(angle, S{1e-6}, f.state.q_nb)) != 0)
    {
        std::abort();
    }
    typename Fixture<S>::Mat factor;
    for (std::size_t i = 0; i < 15U; ++i)
    {
        for (std::size_t j = 0; j < 15U; ++j)
        {
            factor(i, j) = random.template value<S>() * S{0.1};
        }
    }
    // Exact mirrored construction: all correction inputs meet exact-symmetry checks.
    for (std::size_t i = 0; i < 15U; ++i)
    {
        for (std::size_t j = 0; j <= i; ++j)
        {
            S sum = S{0};
            for (std::size_t k = 0; k < 15U; ++k)
            {
                sum += factor(i, k) * factor(j, k);
            }
            if (i == j)
            {
                sum += S{0.5};
            }
            f.P(i, j) = f.P(j, i) = sum;
        }
    }
    S const maximum = std::numeric_limits<S>::max();
    switch (static_cast<EdgeCase>(seed))
    {
    case EdgeCase::zero_motion:
        f.imu.angular_rate_b = {};
        f.state = {};
        break;
    case EdgeCase::subnormal_covariance:
        f.P = decltype(f.P)::identity() * std::numeric_limits<S>::denorm_min();
        break;
    case EdgeCase::signed_zero:
        f.state = {};
        f.imu = {};
        f.P = decltype(f.P)::identity();
        f.P(0, 1) = -S{0};
        f.P(1, 0) = S{0};
        f.imu.angular_rate_b(0) = -S{0};
        break;
    case EdgeCase::overflow_cancellation:
        f.state = {};
        f.imu = {};
        f.parameters.process_noise = {};
        f.dt = S{1};
        f.imu.specific_force_b(1) = f.imu.specific_force_b(2) = S{1};

        {
            int const v[15] = {-1, 0, 0, 1, 0, 0, 1, 1, 1, 1, -1, 1, 1, 1, 1};
            for (std::size_t i = 0; i < 15U; ++i)
            {
                for (std::size_t j = 0; j < 15U; ++j)
                {
                    f.P(i, j) = static_cast<S>(v[i] * v[j]) * (S{0.75} * maximum);
                }
            }
        }
        break;
    case EdgeCase::infinite_covariance:
        f.P(0, 0) = std::numeric_limits<S>::infinity();
        break;
    case EdgeCase::nan_covariance:
        f.P(0, 0) = std::numeric_limits<S>::quiet_NaN();
        break;
    case EdgeCase::infinite_rate:
        f.imu.angular_rate_b(0) = std::numeric_limits<S>::infinity();
        break;
    case EdgeCase::nan_dt:
        f.dt = std::numeric_limits<S>::quiet_NaN();
        break;
    case EdgeCase::zero_dt:
        f.dt = S{0};
        break;
    case EdgeCase::negative_noise:
        f.parameters.process_noise.angular_rate_variance(0) = S{-1};
        break;
    case EdgeCase::invalid_minimum_norm:
        f.parameters.minimum_quaternion_norm = S{0};
        break;
    case EdgeCase::zero_measurement_variance:
        f.V(0, 0) = S{0};
        break;
    case EdgeCase::indefinite_measurement_noise:
        f.V(0, 1) = f.V(1, 0) = S{100};
        break;
    case EdgeCase::asymmetric_prior:
        f.P(0, 1) = S{0.125};
        break; // Runtime diagnostic outside PSD prior premise.
    case EdgeCase::negative_prior_diagonal:
        f.P(0, 0) = S{-1};
        break;
    case EdgeCase::maximum_prior:
        f.P = decltype(f.P)::identity() * maximum;
        break;
    case EdgeCase::maximum_force:
        f.imu.specific_force_b(0) = maximum;
        break;
    case EdgeCase::maximum_rate:
        f.imu.angular_rate_b(0) = maximum;
        break;
    case EdgeCase::minimum_dt:
        f.dt = f.parameters.dt_min;
        break;
    case EdgeCase::maximum_dt:
        f.dt = f.parameters.dt_max;
        break;
    case EdgeCase::zero_observation_gravity:
        f.gravity = {};
        break;
    case EdgeCase::nan_magnetic_observation:
        f.m_b(0) = std::numeric_limits<S>::quiet_NaN();
        break;
    case EdgeCase::infinite_measurement_variance:
        f.V(0, 0) = std::numeric_limits<S>::infinity();
        break;
    case EdgeCase::invalid_norm_tolerance:
        f.parameters.quaternion_squared_norm_tolerance = S{0};
        break;
    default:
        break;
    }
    return f;
}

template <typename S> std::uint64_t bits(S value)
{
    if constexpr (sizeof(S) == 4U)
    {
        return std::bit_cast<std::uint32_t>(value);
    }
    else
    {
        return std::bit_cast<std::uint64_t>(value);
    }
}

template <typename State, typename Mat> auto capture(State const & state, Mat const & P)
{
    using S = typename Mat::value_type;
    std::vector<std::uint64_t> result;
    for (std::size_t i = 0; i < 4U; ++i)
    {
        result.push_back(bits(state.q_nb.coefficients()(i)));
    }

    for (std::size_t i = 0; i < 3U; ++i)
    {
        for (S value : {state.p_n(i), state.v_n(i), state.b_a(i), state.b_g(i)})
        {
            result.push_back(bits(value));
        }
    }
    for (std::size_t i = 0; i < 15U; ++i)
    {
        for (std::size_t j = 0; j < 15U; ++j)
        {
            result.push_back(bits(P(i, j)));
        }
    }
    return result;
}

template <typename S> auto trial_fixture(unsigned seed)
{
    auto f = fixture<S>(seed);
    if (seed >= 600U)
    {
        unsigned const k = seed - 600U;
        int const exponents32[8] = {-149, -126, -100, -30, 0, 30, 100, 120};
        int const exponents64[8] = {-1074, -1022, -900, -30, 0, 30, 900, 1015};
        S const scale = std::ldexp(S{1}, sizeof(S) == 4U ? exponents32[k % 8U] : exponents64[k % 8U]);
        f.parameters.process_noise = {};
        f.dt = k % 3U == 0U ? f.parameters.dt_min : (k % 3U == 1U ? S{0.01} : S{1});
        for (unsigned i = 0; i < 15U; ++i)
        {
            for (unsigned j = 0; j < 15U; ++j)
            {
                int const vi = static_cast<int>((i + k) % 5U) - 2;
                int const vj = static_cast<int>((j + k) % 5U) - 2;
                f.P(i, j) = k / 8U % 3U == 0U ? (i == j ? scale : S{0}) : static_cast<S>(vi * vj) * scale;
                if (k / 8U % 3U == 2U)
                {
                    f.P(i, j) = -S{0};
                }
            }
        }
    }
    return f;
}

template <typename S> auto transition(Fixture<S> const & f)
{
    Matrix<S, 3U, 3U> T;
    auto const omega = f.imu.angular_rate_b - f.state.b_g;
    if (!formal_eskf::succeeded(
            formal_eskf::detail::attitude_error_transition(omega, f.dt, f.parameters.minimum_quaternion_norm, T)))
    {
        std::abort();
    }
    auto F = decltype(f.P)::identity();
    auto const minus_R = -formal_eskf::so3::to_rotation_matrix(f.state.q_nb);
    auto const I_dt = decltype(T)::identity() * f.dt;
    auto const force = f.imu.specific_force_b - f.state.b_a;
    F.template set_block<0U, 3U>(I_dt);
    F.template set_block<3U, 6U>(minus_R * formal_eskf::so3::hat<Backend<S>>(force) * f.dt);
    F.template set_block<3U, 9U>(minus_R * f.dt);
    F.template set_block<6U, 6U>(T);
    F.template set_block<6U, 12U>(-I_dt);
    return F;
}
} // namespace codegen_test
