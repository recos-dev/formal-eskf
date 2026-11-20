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
#include <cstddef>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>
#include <type_traits>

#include <Eigen/Eigenvalues>

#include <formal_eskf/eskf/covariance_prediction.hpp>
#include <formal_eskf/eskf/types.hpp>
#include "test_backend.hpp"

#include "test_support.hpp"
#include "test_cases.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::discretize_process_noise;
using formal_eskf::predict;
using formal_eskf::predict_covariance;
using formal_eskf::Status;
using formal_eskf::test::near;

template <typename Linalg, typename Configuration> struct CovariancePredictionFixture
{
    using types = formal_eskf::EskfTypes<Linalg, Configuration>;
    using value_type = typename types::value_type;
    using covariance_type = typename types::error_covariance_type;
    using noise_covariance_type = typename types::process_noise_covariance_type;
    static constexpr bool is_ins = std::is_same_v<Configuration, formal_eskf::configuration::Ins>;
    static constexpr std::size_t size = types::error_state_dimension;

    typename types::nominal_state_type state{};
    typename types::imu_sample_type imu{};
    typename types::parameter_type parameters{};
    covariance_type covariance{};

    CovariancePredictionFixture()
    {
        parameters.dt_min = value_type{0.125};
        parameters.dt_max = value_type{2};
        parameters.minimum_quaternion_norm = static_cast<value_type>(1.0e-6);
        parameters.quaternion_squared_norm_tolerance = value_type{32} * std::numeric_limits<value_type>::epsilon();
    }

    auto noise_groups()
    {
        using vector_type = formal_eskf::linalg::Vector<Linalg, 3U>;
        auto & noise = parameters.process_noise;
        if constexpr (is_ins)
        {
            return std::array<vector_type *, 4>{&noise.specific_force_variance, &noise.angular_rate_variance,
                                                &noise.accelerometer_bias_random_walk_variance_density,
                                                &noise.gyroscope_bias_random_walk_variance_density};
        }
        else
        {
            return std::array<vector_type *, 1>{&noise.angular_rate_variance};
        }
    }

    void set_distinct_noise()
    {
        std::size_t coefficient = 1U;
        for (auto * group : noise_groups())
        {
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                (*group)(axis) = static_cast<value_type>(coefficient++);
            }
        }
    }
};

template <typename Matrix>
bool matrix_near(Matrix const & actual, Matrix const & expected, typename Matrix::value_type tolerance)
{
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            if (!near(actual(row, column), expected(row, column), tolerance))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename State> bool same_state(State const & actual, State const & expected)
{
    using value_type = typename State::quaternion_type::value_type;
    bool result = formal_eskf::so3::same_coefficients(actual.q_nb, expected.q_nb);
    if constexpr (requires { actual.p_n; })
    {
        result = result && matrix_near(actual.p_n, expected.p_n, value_type{0}) &&
                 matrix_near(actual.v_n, expected.v_n, value_type{0}) &&
                 matrix_near(actual.b_a, expected.b_a, value_type{0}) &&
                 matrix_near(actual.b_g, expected.b_g, value_type{0});
    }
    return result;
}

// Observe values, signed zero and NaN classification, not payloads or object padding.
template <typename Scalar> bool same_scalar_field(Scalar actual, Scalar expected)
{
    using bytes_type = std::array<std::byte, sizeof(Scalar)>;
    return (std::isnan(actual) && std::isnan(expected)) ||
           std::bit_cast<bytes_type>(actual) == std::bit_cast<bytes_type>(expected);
}

template <typename Matrix> bool same_matrix_fields(Matrix const & actual, Matrix const & expected)
{
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            if (!same_scalar_field(actual(row, column), expected(row, column)))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename State> bool same_state_fields(State const & actual, State const & expected)
{
    bool same = same_matrix_fields(actual.q_nb.coefficients(), expected.q_nb.coefficients());
    if constexpr (requires { actual.p_n; })
    {
        same = same && same_matrix_fields(actual.p_n, expected.p_n) && same_matrix_fields(actual.v_n, expected.v_n) &&
               same_matrix_fields(actual.b_a, expected.b_a) && same_matrix_fields(actual.b_g, expected.b_g);
    }
    return same;
}

template <typename Fixture>
void expect_unchanged_imu_and_parameters(TestContext & test, std::string_view profile, Fixture const & actual,
                                         Fixture const & expected)
{
    test.expect(same_matrix_fields(actual.imu.specific_force_b, expected.imu.specific_force_b) &&
                    same_matrix_fields(actual.imu.angular_rate_b, expected.imu.angular_rate_b),
                profile, "prediction leaves the IMU input unchanged");
    auto const & parameters = actual.parameters;
    auto const & original = expected.parameters;
    test.expect(
        same_scalar_field(parameters.dt_min, original.dt_min) &&
            same_scalar_field(parameters.dt_max, original.dt_max) &&
            same_scalar_field(parameters.minimum_quaternion_norm, original.minimum_quaternion_norm) &&
            same_scalar_field(parameters.quaternion_squared_norm_tolerance, original.quaternion_squared_norm_tolerance),
        profile, "prediction leaves the time and quaternion parameters unchanged");
    auto const & noise = parameters.process_noise;
    auto const & original_noise = original.process_noise;
    test.expect(same_matrix_fields(noise.angular_rate_variance, original_noise.angular_rate_variance), profile,
                "prediction leaves the angular-rate noise parameter unchanged");
    if constexpr (Fixture::is_ins)
    {
        test.expect(same_matrix_fields(noise.specific_force_variance, original_noise.specific_force_variance) &&
                        same_matrix_fields(noise.accelerometer_bias_random_walk_variance_density,
                                           original_noise.accelerometer_bias_random_walk_variance_density) &&
                        same_matrix_fields(noise.gyroscope_bias_random_walk_variance_density,
                                           original_noise.gyroscope_bias_random_walk_variance_density),
                    profile, "prediction leaves the INS process noise parameters unchanged");
        test.expect(same_matrix_fields(parameters.gravity_n, original.gravity_n), profile,
                    "prediction leaves the gravity parameter unchanged");
    }
}

template <typename Linalg, typename Configuration>
void test_noise_discretization(TestContext & test, std::string_view profile)
{
    // At most one division and two normal products relate the fixed-frequency
    // fixture back to its target; 8*epsilon bounds their combined rounding.
    constexpr auto tolerance =
        typename Linalg::value_type{8} * std::numeric_limits<typename Linalg::value_type>::epsilon();
    using fixture_type = CovariancePredictionFixture<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    using noise_matrix = typename fixture_type::noise_covariance_type;
    fixture_type fixture;
    fixture.set_distinct_noise();
    noise_matrix Q_i;
    noise_matrix Q_twice;
    value_type const dt = value_type{0.25};
    Status status = discretize_process_noise(fixture.parameters.process_noise, dt, Q_i);
    test.expect(status == Status::success, profile, "equation (15) accepts per-axis variance and intensity");
    status = discretize_process_noise(fixture.parameters.process_noise, value_type{2} * dt, Q_twice);
    test.expect(status == Status::success, profile, "noise is recomputed for the current step length");
    for (std::size_t row = 0U; row < noise_matrix::row_count; ++row)
    {
        bool const bias = fixture_type::is_ins && row >= 6U;
        value_type const expected = static_cast<value_type>(row + 1U) * (bias ? dt : dt * dt);
        test.expect(Q_i(row, row) == expected, profile,
                    "each noise block has the prescribed scale and is not squared twice");
        test.expect(Q_twice(row, row) == Q_i(row, row) * (bias ? value_type{2} : value_type{4}), profile,
                    "doubling dt quadruples measurement noise and doubles bias noise");
        for (std::size_t column = 0U; column < noise_matrix::column_count; ++column)
        {
            if (row != column)
            {
                test.expect(Q_i(row, column) == value_type{0}, profile, "independent noise has zero off-diagonals");
            }
        }
    }

    // A fixed dt can reproduce pre-discretized constants from the sketch.
    auto groups = fixture.noise_groups();
    value_type const fixed_dt = static_cast<value_type>(0.01);
    for (std::size_t group = 0U; group < groups.size(); ++group)
    {
        bool const bias = fixture_type::is_ins && group >= 2U;
        value_type const target = bias ? static_cast<value_type>(1.0e-10)
                                       : (fixture_type::is_ins && group == 0U ? static_cast<value_type>(1.0e-8)
                                                                              : static_cast<value_type>(1.0e-6));
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            (*groups[group])(axis) = target / (bias ? fixed_dt : fixed_dt * fixed_dt);
        }
    }
    status = discretize_process_noise(fixture.parameters.process_noise, fixed_dt, Q_i);
    test.expect(status == Status::success, profile,
                "standalone discretization is independent of application time bounds");
    for (std::size_t row = 0U; row < noise_matrix::row_count; ++row)
    {
        value_type const target = fixture_type::is_ins && row >= 6U
                                      ? static_cast<value_type>(1.0e-10)
                                      : (fixture_type::is_ins && row < 3U ? static_cast<value_type>(1.0e-8)
                                                                          : static_cast<value_type>(1.0e-6));
        test.expect(near(Q_i(row, row), target, tolerance * target), profile,
                    "fixed-frequency parameters reproduce the reference's fixed Q_i");
    }

    fixture.parameters.process_noise = {};
    status = discretize_process_noise(fixture.parameters.process_noise, value_type{1}, Q_i);
    test.expect(status == Status::success && formal_eskf::linalg::max_abs(Q_i) == value_type{0}, profile,
                "zero process noise is valid");
    status = discretize_process_noise(fixture.parameters.process_noise, std::numeric_limits<value_type>::max(), Q_i);
    test.expect(status == Status::success && formal_eskf::linalg::max_abs(Q_i) == value_type{0}, profile,
                "zero noise does not spuriously overflow by forming dt squared first");
}

template <typename Linalg, typename Configuration>
void test_noise_failures(TestContext & test, std::string_view profile)
{
    using fixture_type = CovariancePredictionFixture<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    using noise_matrix = typename fixture_type::noise_covariance_type;
    fixture_type fixture;
    noise_matrix const sentinel = noise_matrix::identity() * value_type{7};
    auto output = sentinel;
    value_type const nan = std::numeric_limits<value_type>::quiet_NaN();
    value_type const inf = std::numeric_limits<value_type>::infinity();
    for (value_type const dt : {value_type{0}, value_type{-1}, nan, inf})
    {
        Status const expected = std::isfinite(dt) ? Status::out_of_range : Status::non_finite_input;
        Status const status = discretize_process_noise(fixture.parameters.process_noise, dt, output);
        test.expect(status == expected && matrix_near(output, sentinel, value_type{0}), profile,
                    "invalid noise time step leaves output unchanged");
    }
    for (auto * group : fixture.noise_groups())
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            for (value_type const invalid : {value_type{-1}, nan, inf})
            {
                (*group)(axis) = invalid;
                Status const expected = std::isfinite(invalid) ? Status::domain_error : Status::non_finite_input;
                Status const status = discretize_process_noise(fixture.parameters.process_noise, value_type{1}, output);
                test.expect(status == expected && matrix_near(output, sentinel, value_type{0}), profile,
                            "every invalid noise coefficient is rejected without partial output");
            }
            (*group)(axis) = value_type{0};
        }
    }
}

template <typename Linalg, typename Configuration>
void test_noise_ieee_scaling(TestContext & test, std::string_view profile)
{
    using fixture_type = CovariancePredictionFixture<Linalg, Configuration>;
    using value_type = typename fixture_type::value_type;
    using matrix_type = typename fixture_type::noise_covariance_type;
    fixture_type fixture;
    matrix_type output;
    auto status =
        discretize_process_noise(fixture.parameters.process_noise, std::numeric_limits<value_type>::max(), output);
    test.expect(status == Status::success && formal_eskf::linalg::max_abs(output) == value_type{0}, profile,
                "zero noise with huge finite dt must not form an overflowing dt squared");
    for (std::size_t group_index = 0U; group_index < fixture.noise_groups().size(); ++group_index)
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            fixture.parameters.process_noise = {};
            (*fixture.noise_groups()[group_index])(axis) = std::numeric_limits<value_type>::max();
            output = matrix_type::identity();
            output(0U, 1U) = -value_type{0};
            output(1U, 0U) = std::numeric_limits<value_type>::quiet_NaN();
            auto const saved = output;
            auto const before = fixture;
            status = discretize_process_noise(fixture.parameters.process_noise, value_type{2}, output);
            test.expect(status == Status::non_finite_result && same_matrix_fields(output, saved), profile,
                        "each sample/density product overflow preserves all old output coefficients");
            expect_unchanged_imu_and_parameters(test, profile, fixture, before);

            fixture.parameters.process_noise = {};
            (*fixture.noise_groups()[group_index])(axis) = std::numeric_limits<value_type>::min();
            status = discretize_process_noise(fixture.parameters.process_noise, value_type{0.5}, output);
            value_type const expected =
                std::numeric_limits<value_type>::min() *
                (fixture_type::is_ins && group_index >= 2U ? value_type{0.5} : value_type{0.25});
            test.expect(status == Status::success &&
                            output(3U * group_index + axis, 3U * group_index + axis) == expected,
                        profile, "normal variance may produce the exact nonnegative subnormal discretization");
            (*fixture.noise_groups()[group_index])(axis) = std::numeric_limits<value_type>::denorm_min();
            status = discretize_process_noise(fixture.parameters.process_noise, value_type{0.5}, output);
            test.expect(status == Status::success &&
                            output(3U * group_index + axis, 3U * group_index + axis) == value_type{0},
                        profile, "small noise products may underflow to zero without rejecting the step");
        }
    }
}

template <typename Linalg, typename Configuration>
void test_noise_validation_order(TestContext & test, std::string_view profile)
{
    test_noise_failures<Linalg, Configuration>(test, profile);
    using fixture_type = CovariancePredictionFixture<Linalg, Configuration>;
    using value_type = typename fixture_type::value_type;
    using matrix_type = typename fixture_type::noise_covariance_type;
    fixture_type fixture;
    matrix_type output = matrix_type::identity();
    output(0U, 1U) = -value_type{0};
    output(1U, 0U) = std::numeric_limits<value_type>::quiet_NaN();
    auto const before = output;
    for (std::size_t group = 0U; group < fixture.noise_groups().size(); ++group)
    {
        fixture.parameters.process_noise = {};
        (*fixture.noise_groups()[group])(0U) = value_type{-1};
        (*fixture.noise_groups()[group])(2U) = std::numeric_limits<value_type>::quiet_NaN();
        auto status = discretize_process_noise(fixture.parameters.process_noise, value_type{0.5}, output);
        test.expect(status == Status::non_finite_input && same_matrix_fields(output, before), profile,
                    "nonfinite entry precedes a negative entry within one source group");
        status = discretize_process_noise(fixture.parameters.process_noise, value_type{0}, output);
        test.expect(status == Status::out_of_range && same_matrix_fields(output, before), profile,
                    "dt domain is checked before source values");
        if (group > 0U)
        {
            (*fixture.noise_groups()[group - 1U])(0U) = value_type{-1};
            status = discretize_process_noise(fixture.parameters.process_noise, value_type{0.5}, output);
            test.expect(status == Status::domain_error && same_matrix_fields(output, before), profile,
                        "earlier source group failure precedes a later group's nonfinite input");
        }
    }
}

// Independent Rodrigues oracle for R(Exp(theta))^T, using ordinary arrays and
// scalar loops rather than production quaternion, hat, block, or sandwich code.
std::array<long double, 9> reference_attitude_transition(std::array<long double, 3> const & omega, long double dt)
{
    std::array<long double, 9> const W{
        0, -omega[2] * dt, omega[1] * dt, omega[2] * dt, 0, -omega[0] * dt, -omega[1] * dt, omega[0] * dt, 0};
    long double first = 1;
    long double second = 0;
#if !ESKF_QUAT_APPROX
    long double const theta = dt * std::sqrt(omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2]);
    first = theta == 0 ? 1 : std::sin(theta) / theta;
    second = theta == 0 ? 0.5 : (1 - std::cos(theta)) / (theta * theta);
#endif
    std::array<long double, 9> result{};
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            long double square = 0;
            for (std::size_t inner = 0U; inner < 3U; ++inner)
            {
                square += W[row * 3U + inner] * W[inner * 3U + column];
            }
            result[row * 3U + column] = (row == column ? 1.0L : 0.0L) - first * W[row * 3U + column] + second * square;
        }
    }
    return result;
}

template <typename Linalg, typename Configuration>
auto reference_covariance(CovariancePredictionFixture<Linalg, Configuration> const & fixture, long double dt)
{
    using fixture_type = CovariancePredictionFixture<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    constexpr std::size_t size = fixture_type::size;
    std::array<long double, size * size> F{};
    for (std::size_t row = 0U; row < size; ++row)
    {
        F[row * size + row] = 1;
    }
    // The dense fixture has q=(0.5,0.5,0.5,0.5), hence x->y, y->z, z->x.
    std::array<long double, 9> const R{0, 0, 1, 1, 0, 0, 0, 1, 0};
    std::array<long double, 3> omega{};
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        omega[axis] = static_cast<long double>(fixture.imu.angular_rate_b(axis));
        if constexpr (fixture_type::is_ins)
        {
            omega[axis] -= static_cast<long double>(fixture.state.b_g(axis));
        }
    }
    auto const attitude = reference_attitude_transition(omega, dt);
    constexpr std::size_t attitude_offset = fixture_type::is_ins ? 6U : 0U;
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            F[(attitude_offset + row) * size + attitude_offset + column] = attitude[row * 3U + column];
        }
    }
    if constexpr (fixture_type::is_ins)
    {
        std::array<long double, 3> force{};
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            force[axis] = static_cast<long double>(fixture.imu.specific_force_b(axis)) -
                          static_cast<long double>(fixture.state.b_a(axis));
            F[axis * size + 3U + axis] = dt;
            F[(6U + axis) * size + 12U + axis] = -dt;
        }
        std::array<long double, 9> const A{0, -force[2], force[1], force[2], 0, -force[0], -force[1], force[0], 0};
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            for (std::size_t column = 0U; column < 3U; ++column)
            {
                F[(3U + row) * size + 9U + column] = -R[row * 3U + column] * dt;
                for (std::size_t inner = 0U; inner < 3U; ++inner)
                {
                    F[(3U + row) * size + 6U + column] -= R[row * 3U + inner] * A[inner * 3U + column] * dt;
                }
            }
        }
    }
    typename fixture_type::covariance_type result;
    for (std::size_t row = 0U; row < size; ++row)
    {
        for (std::size_t column = 0U; column < size; ++column)
        {
            long double coefficient = 0;
            for (std::size_t left = 0U; left < size; ++left)
            {
                for (std::size_t right = 0U; right < size; ++right)
                {
                    coefficient += F[row * size + left] * static_cast<long double>(fixture.covariance(left, right)) *
                                   F[column * size + right];
                }
            }
            // set_distinct_noise() fixes the independent variances to 1..N.
            if constexpr (fixture_type::is_ins)
            {
                if (row >= 3U && row < 6U && column >= 3U && column < 6U)
                {
                    for (std::size_t axis = 0U; axis < 3U; ++axis)
                    {
                        coefficient += R[(row - 3U) * 3U + axis] * R[(column - 3U) * 3U + axis] *
                                       static_cast<long double>(axis + 1U) * dt * dt;
                    }
                }
                if (row == column && row >= 6U)
                {
                    coefficient += static_cast<long double>(row - 2U) * (row < 9U ? dt * dt : dt);
                }
            }
            else if (row == column)
            {
                coefficient += static_cast<long double>(row + 1U) * dt * dt;
            }
            result(row, column) = static_cast<value_type>(coefficient);
        }
    }
    return result;
}

template <typename Matrix> bool representative_psd(Matrix const & covariance, typename Matrix::value_type tolerance)
{
    constexpr int size = static_cast<int>(Matrix::row_count);
    Eigen::Matrix<typename Matrix::value_type, size, size> matrix;
    for (int row = 0; row < size; ++row)
    {
        for (int column = 0; column < size; ++column)
        {
            matrix(row, column) = covariance(static_cast<std::size_t>(row), static_cast<std::size_t>(column));
        }
    }
    Eigen::SelfAdjointEigenSolver<decltype(matrix)> solver(matrix, Eigen::EigenvaluesOnly);
    return solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() >= -tolerance;
}

template <typename Linalg, typename Configuration> auto dense_fixture(TestContext & test, std::string_view profile)
{
    using fixture_type = CovariancePredictionFixture<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    fixture_type fixture;
    fixture.set_distinct_noise();
    Status status = formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(
        value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5}, fixture.parameters.minimum_quaternion_norm,
        fixture.state.q_nb);
    test.expect(status == Status::success, profile, "cyclic body-to-navigation rotation fixture is valid");
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        fixture.imu.angular_rate_b(axis) = static_cast<value_type>(axis + 1U);
        if constexpr (fixture_type::is_ins)
        {
            fixture.state.b_g(axis) = value_type{0.5};
            fixture.state.b_a(axis) = static_cast<value_type>(axis + 2U);
            fixture.imu.specific_force_b(axis) = static_cast<value_type>(2U * axis + 1U);
        }
    }
    // Dense SPD P = I + u*u^T, with mixed-sign cross-state covariances.
    for (std::size_t row = 0U; row < fixture_type::size; ++row)
    {
        auto const u =
            (row % 2U == 0U ? value_type{1} : value_type{-1}) * static_cast<value_type>(row + 1U) / value_type{16};
        for (std::size_t column = 0U; column < fixture_type::size; ++column)
        {
            auto const v = (column % 2U == 0U ? value_type{1} : value_type{-1}) * static_cast<value_type>(column + 1U) /
                           value_type{16};
            fixture.covariance(row, column) = u * v + (row == column ? value_type{1} : value_type{0});
        }
    }
    return fixture;
}

template <typename Linalg, typename Configuration>
void test_dense_prediction(TestContext & test, std::string_view profile)
{
    constexpr auto tolerance = static_cast<typename Linalg::value_type>(
        std::is_same_v<typename Linalg::value_type, float> ? 2.0e-5F : 1.0e-11);
    using fixture_type = CovariancePredictionFixture<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    auto fixture = dense_fixture<Linalg, Configuration>(test, profile);
    Status status;
    auto const initial = fixture;
    value_type const dt = value_type{0.25};
    auto const expected = reference_covariance(initial, static_cast<double>(dt));
    auto output = fixture.covariance;
    if constexpr (fixture_type::is_ins)
    {
        status = predict_covariance(fixture.state, fixture.covariance, fixture.imu, dt, fixture.parameters, output);
    }
    else
    {
        status = predict_covariance(fixture.covariance, fixture.imu, dt, fixture.parameters, output);
    }
    test.expect(status == Status::success && matrix_near(output, expected, tolerance), profile,
                "all covariance coefficients match an independent dense equation (18) oracle");
    test.expect(formal_eskf::linalg::is_symmetric(output, value_type{0}) && representative_psd(output, tolerance),
                profile, "representative propagated covariance is symmetric and numerically PSD");

    auto negative = initial.state;
    negative.q_nb = -negative.q_nb;
    if constexpr (fixture_type::is_ins)
    {
        status = predict_covariance(negative, initial.covariance, initial.imu, dt, initial.parameters, output);
    }
    else
    {
        status = predict_covariance(initial.covariance, initial.imu, dt, initial.parameters, output);
    }
    test.expect(status == Status::success && matrix_near(output, expected, tolerance), profile,
                "covariance prediction is invariant to quaternion sign");
}

template <typename Linalg, typename Configuration> void test_joint_success(TestContext & test, std::string_view profile)
{
    using fixture_type = CovariancePredictionFixture<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    // Supplemental general-domain regression; exact equation cases are below.
    constexpr value_type tolerance = static_cast<value_type>(std::is_same_v<value_type, float> ? 2.0e-5F : 1.0e-11);
    auto fixture = dense_fixture<Linalg, Configuration>(test, profile);
    auto const initial = fixture;
    value_type const dt = value_type{0.25};
    auto const expected = reference_covariance(initial, static_cast<long double>(dt));
    auto output = fixture.covariance;
    Status status;
    if constexpr (fixture_type::is_ins)
    {
        status = predict_covariance(fixture.state, fixture.covariance, fixture.imu, dt, fixture.parameters, output);
    }
    else
    {
        status = predict_covariance(fixture.covariance, fixture.imu, dt, fixture.parameters, output);
    }
    test.expect(status == Status::success, profile, "standalone covariance prerequisite succeeds");
    auto nominal = fixture.state;
    status = formal_eskf::predict_nominal(fixture.state, fixture.imu, dt, fixture.parameters, nominal);
    test.expect(status == Status::success, profile, "separately called nominal component succeeds");
    for (bool const state_alias : {false, true})
    {
        for (bool const covariance_alias : {false, true})
        {
            fixture = initial;
            auto separate_state = initial.state;
            separate_state.q_nb = -separate_state.q_nb;
            auto separate_covariance = initial.covariance * value_type{2};
            auto & state_output = state_alias ? fixture.state : separate_state;
            auto & covariance_output = covariance_alias ? fixture.covariance : separate_covariance;
            status = predict(fixture.state, fixture.covariance, fixture.imu, dt, fixture.parameters, state_output,
                             covariance_output);
            test.expect(status == Status::success && same_state(state_output, nominal) &&
                            matrix_near(covariance_output, expected, tolerance) &&
                            matrix_near(covariance_output, output, value_type{0}),
                        profile, "every alias arrangement publishes nominal and covariance from the same old prior");
            test.expect(state_alias || same_state(fixture.state, initial.state), profile,
                        "separate state output leaves the nominal input unchanged");
            test.expect(covariance_alias || matrix_near(fixture.covariance, initial.covariance, value_type{0}), profile,
                        "separate covariance output leaves the covariance input unchanged");
            expect_unchanged_imu_and_parameters(test, profile, fixture, initial);
        }
    }

    auto exact = dense_fixture<Linalg, Configuration>(test, profile);
    exact.imu.angular_rate_b = {};
    auto expected_state = exact.state;
    if constexpr (fixture_type::is_ins)
    {
        exact.imu.angular_rate_b = exact.state.b_g;
        exact.parameters.gravity_n = {};
        exact.parameters.gravity_n(2U) = value_type{8};
        // Unbiased body force is (-1,0,1), cyclic R maps it to (1,-1,0).
        // Old-state acceleration is therefore exactly (1,-1,8).
        constexpr std::array<value_type, 3U> acceleration{value_type{1}, value_type{-1}, value_type{8}};
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            value_type const n = static_cast<value_type>(axis + 1U);
            exact.state.p_n(axis) = n;
            exact.state.v_n(axis) = -value_type{2} * n;
            expected_state.p_n(axis) = n / value_type{2} + acceleration[axis] / value_type{32};
            expected_state.v_n(axis) = -value_type{2} * n + acceleration[axis] / value_type{4};
        }
    }
    auto const expected_covariance = reference_covariance(exact, 0.25L);
    for (bool const state_alias : {false, true})
    {
        for (bool const covariance_alias : {false, true})
        {
            auto local = exact;
            auto separate_state = exact.state;
            separate_state.q_nb = -separate_state.q_nb;
            auto separate_covariance = exact.covariance * value_type{2};
            auto & state_output = state_alias ? local.state : separate_state;
            auto & covariance_output = covariance_alias ? local.covariance : separate_covariance;
            test.expect(predict(local.state, local.covariance, local.imu, dt, local.parameters, state_output,
                                covariance_output) == Status::success &&
                            same_state(state_output, expected_state) &&
                            matrix_near(covariance_output, expected_covariance, value_type{0}),
                        profile, "independent exact joint oracle checks every nominal field and covariance entry");
            test.expect((state_alias || same_state_fields(local.state, exact.state)) &&
                            (covariance_alias || same_matrix_fields(local.covariance, exact.covariance)),
                        profile, "exact joint fixture preserves each distinct prior for all four aliases");
            expect_unchanged_imu_and_parameters(test, profile, local, exact);
        }
    }

    bool all_succeeded = true;
    for (std::size_t step = 0U; step < 20U; ++step)
    {
        status = predict(fixture.state, fixture.covariance, fixture.imu, value_type{0.125}, fixture.parameters,
                         fixture.state, fixture.covariance);
        all_succeeded = all_succeeded && status == Status::success;
    }
    test.expect(all_succeeded && formal_eskf::linalg::is_symmetric(fixture.covariance, value_type{0}) &&
                    representative_psd(fixture.covariance, tolerance * value_type{10}),
                profile, "repeated prediction preserves representative covariance symmetry and PSD within tolerance");
}

template <typename Linalg, typename Configuration>
void test_exact_transition(TestContext & test, std::string_view profile)
{
    using fixture_type = CovariancePredictionFixture<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    for (bool const positive_diagonal : {false, true})
    {
        auto fixture = dense_fixture<Linalg, Configuration>(test, profile);
        if constexpr (fixture_type::is_ins)
        {
            fixture.imu.angular_rate_b = fixture.state.b_g;
        }
        else
        {
            fixture.imu.angular_rate_b = {};
        }
        if (!positive_diagonal)
        {
            for (std::size_t axis = 0U; axis < fixture_type::size; ++axis)
            {
                fixture.covariance(axis, axis) -= value_type{1};
            }
        }
        // All coefficients are small dyadic rationals and the attitude block
        // is identity: each product and sum here is exactly representable.
        auto const expected = reference_covariance(fixture, 0.25L);
        auto output = fixture.covariance;
        Status status;
        if constexpr (fixture_type::is_ins)
        {
            status = predict_covariance(fixture.state, fixture.covariance, fixture.imu, value_type{0.25},
                                        fixture.parameters, output);
        }
        else
        {
            status = predict_covariance(fixture.covariance, fixture.imu, value_type{0.25}, fixture.parameters, output);
        }
        test.expect(status == Status::success && matrix_near(output, expected, value_type{0}), profile,
                    "all full-transition entries exactly match independent PSD/SPD dyadic fixtures");
    }
}

template <typename Linalg, typename Configuration>
void test_full_transition(TestContext & test, std::string_view profile)
{
    test_exact_transition<Linalg, Configuration>(test, profile);
    test_dense_prediction<Linalg, Configuration>(test, profile);
}

template <typename Linalg> void test_finalization(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using matrix_type = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    matrix_type candidate;
    candidate(0U, 0U) = value_type{7};
    candidate(0U, 1U) = candidate(1U, 0U) = std::numeric_limits<value_type>::max();
    candidate(1U, 2U) = value_type{4};
    candidate(2U, 1U) = value_type{8};
    matrix_type output;
    auto status = formal_eskf::detail::finish_covariance(candidate, output);
    test.expect(status == Status::success && output(0U, 1U) == std::numeric_limits<value_type>::max() &&
                    output(1U, 0U) == output(0U, 1U) && output(1U, 2U) == value_type{6} &&
                    output(2U, 1U) == value_type{6} && output(0U, 0U) == value_type{7},
                profile, "dense cleanup halves before adding, mirrors and leaves diagonals unchanged");
    candidate = {};
    candidate(0U, 1U) = candidate(1U, 0U) = std::numeric_limits<value_type>::denorm_min();
    status = formal_eskf::detail::publish_covariance(candidate, output);
    test.expect(status == Status::success && same_matrix_fields(output, candidate), profile,
                "already symmetric publication preserves subnormal pairs without a second average");
    auto const saved = output;
    candidate(2U, 2U) = std::numeric_limits<value_type>::quiet_NaN();
    status = formal_eskf::detail::publish_covariance(candidate, output);
    test.expect(status == Status::non_finite_result && same_matrix_fields(output, saved), profile,
                "finite publication rejects a late nonfinite coefficient without any output write");
    status = formal_eskf::detail::finish_covariance(candidate, output);
    test.expect(status == Status::non_finite_result && same_matrix_fields(output, saved), profile,
                "dense cleanup rejects a late nonfinite coefficient before publication");
}

template <typename Linalg> void test_rotated_noise(TestContext & test, std::string_view profile)
{
    constexpr auto tolerance = static_cast<typename Linalg::value_type>(
        std::is_same_v<typename Linalg::value_type, float> ? 2.0e-5F : 1.0e-11);
    using fixture_type = CovariancePredictionFixture<Linalg, formal_eskf::configuration::Ins>;
    using value_type = typename Linalg::value_type;
    fixture_type fixture;
    value_type const angle = static_cast<value_type>(std::acos(-1.0) / 8.0);
    Status status = formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(
        std::cos(angle), value_type{0}, value_type{0}, std::sin(angle), fixture.parameters.minimum_quaternion_norm,
        fixture.state.q_nb);
    test.expect(status == Status::success, profile, "45 degree yaw fixture is valid");
    fixture.parameters.process_noise.specific_force_variance(0U) = value_type{1};
    fixture.parameters.process_noise.specific_force_variance(1U) = value_type{3};
    fixture.parameters.process_noise.specific_force_variance(2U) = value_type{5};
    status = predict_covariance(fixture.state, fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters,
                                fixture.covariance);
    test.expect(status == Status::success && near(fixture.covariance(3U, 3U), value_type{0.5}, tolerance) &&
                    near(fixture.covariance(4U, 4U), value_type{0.5}, tolerance) &&
                    near(fixture.covariance(3U, 4U), value_type{-0.25}, tolerance) &&
                    near(fixture.covariance(5U, 5U), value_type{1.25}, tolerance),
                profile, "anisotropic body noise rotates into navigation velocity, including off-diagonal sign");
    test.expect(formal_eskf::linalg::max_abs(fixture.covariance.template block<0U, 0U, 3U, 15U>()) == value_type{0},
                profile, "paper truncation adds no direct position noise in the first step");

    fixture.covariance = {};
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        fixture.parameters.process_noise.specific_force_variance(axis) = value_type{4};
    }
    status = predict_covariance(fixture.state, fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters,
                                fixture.covariance);
    auto const velocity_noise = fixture.covariance.template block<3U, 3U, 3U, 3U>();
    test.expect(status == Status::success &&
                    matrix_near(velocity_noise, decltype(velocity_noise)::identity(), tolerance),
                profile, "isotropic noise recovers the paper's identity-frame injection");
}

template <typename Linalg, typename Configuration>
void test_prediction_failures(TestContext & test, std::string_view profile)
{
    using fixture_type = CovariancePredictionFixture<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    using covariance_type = typename fixture_type::covariance_type;
    fixture_type fixture;
    fixture.imu.angular_rate_b(2U) = value_type{1};
    covariance_type const sentinel = covariance_type::identity() * value_type{7};
    auto covariance_output = sentinel;
    auto state_output = fixture.state;
    auto const state_before = state_output;
    value_type const nan = std::numeric_limits<value_type>::quiet_NaN();
    value_type const inf = std::numeric_limits<value_type>::infinity();
    for (auto * group : fixture.noise_groups())
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            for (value_type const invalid : {value_type{-1}, nan, inf})
            {
                (*group)(axis) = invalid;
                Status const expected = std::isfinite(invalid) ? Status::domain_error : Status::non_finite_input;
                Status const status = predict(fixture.state, fixture.covariance, fixture.imu, value_type{0.5},
                                              fixture.parameters, state_output, covariance_output);
                test.expect(status == expected && same_state(state_output, state_before) &&
                                matrix_near(covariance_output, sentinel, value_type{0}),
                            profile,
                            "invalid process noise rolls back BOTH outputs after successful nominal calculation");
            }
            (*group)(axis) = value_type{0};
        }
    }
    for (std::size_t row = 0U; row < fixture_type::size; ++row)
    {
        for (std::size_t column = 0U; column < fixture_type::size; ++column)
        {
            fixture.covariance(row, column) = nan;
            Status const status = predict(fixture.state, fixture.covariance, fixture.imu, value_type{0.5},
                                          fixture.parameters, state_output, covariance_output);
            test.expect(status == Status::non_finite_input && same_state(state_output, state_before) &&
                            matrix_near(covariance_output, sentinel, value_type{0}),
                        profile, "non-finite covariance at every position rolls back both outputs");
            fixture.covariance(row, column) = value_type{0};
        }
    }
    for (value_type const dt : {value_type{0}, value_type{-1}, value_type{0.0625}, value_type{4}, nan, inf})
    {
        Status const expected = std::isfinite(dt) ? Status::out_of_range : Status::non_finite_input;
        Status status;
        if constexpr (fixture_type::is_ins)
        {
            status = predict_covariance(fixture.state, fixture.covariance, fixture.imu, dt, fixture.parameters,
                                        covariance_output);
        }
        else
        {
            status = predict_covariance(fixture.covariance, fixture.imu, dt, fixture.parameters, covariance_output);
        }
        test.expect(status == expected && matrix_near(covariance_output, sentinel, value_type{0}), profile,
                    "covariance prediction enforces finite dt and inclusive application time bounds");
    }
    for (value_type const dt : {fixture.parameters.dt_min, fixture.parameters.dt_max})
    {
        auto output = sentinel;
        Status status;
        if constexpr (fixture_type::is_ins)
        {
            status = predict_covariance(fixture.state, fixture.covariance, fixture.imu, dt, fixture.parameters, output);
        }
        else
        {
            status = predict_covariance(fixture.covariance, fixture.imu, dt, fixture.parameters, output);
        }
        test.expect(status == Status::success && formal_eskf::linalg::max_abs(output) == value_type{0}, profile,
                    "both inclusive time boundaries accept zero covariance and noise");
    }
    auto parameters = fixture.parameters;
    parameters.dt_min = value_type{0};
    Status status;
    if constexpr (fixture_type::is_ins)
    {
        status = predict_covariance(fixture.state, fixture.covariance, fixture.imu, value_type{0.5}, parameters,
                                    covariance_output);
    }
    else
    {
        status = predict_covariance(fixture.covariance, fixture.imu, value_type{0.5}, parameters, covariance_output);
    }
    test.expect(status == Status::domain_error && matrix_near(covariance_output, sentinel, value_type{0}), profile,
                "invalid time configuration is rejected");
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        auto imu = fixture.imu;
        imu.angular_rate_b(axis) = inf;
        if constexpr (fixture_type::is_ins)
        {
            status = predict_covariance(fixture.state, fixture.covariance, imu, value_type{0.5}, fixture.parameters,
                                        covariance_output);
        }
        else
        {
            status =
                predict_covariance(fixture.covariance, imu, value_type{0.5}, fixture.parameters, covariance_output);
        }
        test.expect(status == Status::non_finite_input && matrix_near(covariance_output, sentinel, value_type{0}),
                    profile, "non-finite angular rate is rejected by standalone covariance prediction");
    }
    auto imu = fixture.imu;
    imu.angular_rate_b(0U) = std::numeric_limits<value_type>::max();
    if constexpr (fixture_type::is_ins)
    {
        status = predict_covariance(fixture.state, fixture.covariance, imu, value_type{2}, fixture.parameters,
                                    covariance_output);
    }
    else
    {
        status = predict_covariance(fixture.covariance, imu, value_type{2}, fixture.parameters, covariance_output);
    }
    test.expect(status == Status::non_finite_result && matrix_near(covariance_output, sentinel, value_type{0}), profile,
                "overflow in angular increment leaves covariance unchanged");

    // Each operand is finite; only FPF^T + Q overflows.
    fixture.imu.angular_rate_b = {};
    fixture.covariance = covariance_type::identity() * std::numeric_limits<value_type>::max();
    fixture.parameters.process_noise.angular_rate_variance(0U) = std::numeric_limits<value_type>::max();
    status = predict(fixture.state, fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters, state_output,
                     covariance_output);
    test.expect(status == Status::non_finite_result && same_state(state_output, state_before) &&
                    matrix_near(covariance_output, sentinel, value_type{0}),
                profile, "late covariance overflow leaves both outputs unchanged");

    // In-place rollback too, including a nominal attitude that would change.
    fixture.covariance = sentinel;
    fixture.parameters.process_noise.angular_rate_variance(0U) = value_type{-1};
    fixture.imu.angular_rate_b(2U) = value_type{1};
    status = predict(fixture.state, fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters, fixture.state,
                     fixture.covariance);
    test.expect(status == Status::domain_error && same_state(fixture.state, state_before) &&
                    matrix_near(fixture.covariance, sentinel, value_type{0}),
                profile, "in-place prediction failure does not commit nominal state before covariance validation");

    fixture.parameters.process_noise = {};
    fixture.imu.angular_rate_b(0U) = nan;
    status = predict(fixture.state, fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters, fixture.state,
                     fixture.covariance);
    test.expect(status == Status::non_finite_input && same_state(fixture.state, state_before) &&
                    matrix_near(fixture.covariance, sentinel, value_type{0}),
                profile, "nominal prediction failure also preserves both outputs");
}

template <typename Fixture>
void expect_failed_prediction(TestContext & test, std::string_view profile, Fixture const & input,
                              typename Fixture::value_type dt, Status expected)
{
    using value_type = typename Fixture::value_type;
    using covariance_type = typename Fixture::covariance_type;
    for (bool const state_alias : {false, true})
    {
        for (bool const covariance_alias : {false, true})
        {
            auto fixture = input;
            auto state_output = input.state;
            state_output.q_nb = -state_output.q_nb;
            if constexpr (Fixture::is_ins)
            {
                state_output.p_n(0U) = std::numeric_limits<value_type>::quiet_NaN();
                state_output.v_n(1U) = std::numeric_limits<value_type>::infinity();
                state_output.b_a(2U) = -value_type{0};
                state_output.b_g(0U) = value_type{7};
            }
            auto covariance_output = covariance_type::identity() * value_type{7};
            covariance_output(0U, 0U) = std::numeric_limits<value_type>::quiet_NaN();
            covariance_output(1U, 2U) = -value_type{0};
            covariance_output(2U, 2U) = std::numeric_limits<value_type>::infinity();
            auto & output_state = state_alias ? fixture.state : state_output;
            auto & output_covariance = covariance_alias ? fixture.covariance : covariance_output;
            auto const state_before = output_state;
            auto const covariance_before = output_covariance;
            auto const before = fixture;
            std::string const arrangement = std::string(profile) + (state_alias ? " state=in" : " state=out") +
                                            (covariance_alias ? " P=in" : " P=out");
            Status const status = predict(fixture.state, fixture.covariance, fixture.imu, dt, fixture.parameters,
                                          output_state, output_covariance);
            test.expect(status == expected, arrangement, "first failing stage determines the returned status");
            test.expect(same_state_fields(output_state, state_before), arrangement,
                        "failure preserves every nominal output field, including signed-zero/non-finite sentinels");
            test.expect(same_matrix_fields(output_covariance, covariance_before), arrangement,
                        "failure preserves every covariance coefficient, including signed zero and NaN classification");
            test.expect(same_state_fields(fixture.state, before.state) &&
                            same_matrix_fields(fixture.covariance, before.covariance),
                        arrangement, "failure preserves the input state and covariance");
            expect_unchanged_imu_and_parameters(test, arrangement, fixture, before);
        }
    }
}

template <typename Linalg, typename Configuration> void test_first_failure(TestContext & test, std::string_view profile)
{
    using fixture_type = CovariancePredictionFixture<Linalg, Configuration>;
    using value_type = typename fixture_type::value_type;
    fixture_type fixture;
    fixture.parameters.dt_min = static_cast<value_type>(1.0e-6);
    fixture.parameters.quaternion_squared_norm_tolerance = value_type{64} * std::numeric_limits<value_type>::epsilon();
    if constexpr (fixture_type::is_ins)
    {
        fixture.parameters.gravity_n(2U) = static_cast<value_type>(9.80665);
    }
    fixture.imu.angular_rate_b(0U) = std::numeric_limits<value_type>::max();
    fixture.covariance = fixture_type::covariance_type::identity();
    fixture.covariance(0U, 0U) = std::numeric_limits<value_type>::quiet_NaN();
    auto nominal_output = fixture.state;
    Status status =
        formal_eskf::predict_nominal(fixture.state, fixture.imu, value_type{2}, fixture.parameters, nominal_output);
    test.expect(status == Status::non_finite_result, profile, "nominal stage overflows before either attitude branch");
    auto covariance_output = fixture.covariance;
    if constexpr (fixture_type::is_ins)
    {
        status = predict_covariance(fixture.state, fixture.covariance, fixture.imu, value_type{2}, fixture.parameters,
                                    covariance_output);
    }
    else
    {
        status =
            predict_covariance(fixture.covariance, fixture.imu, value_type{2}, fixture.parameters, covariance_output);
    }
    test.expect(status == Status::non_finite_input, profile, "covariance would fail with a distinct input status");
    expect_failed_prediction(test, profile, fixture, value_type{2}, Status::non_finite_result);
}

template <typename Linalg, typename Configuration> void test_late_failure(TestContext & test, std::string_view profile)
{
    using fixture_type = CovariancePredictionFixture<Linalg, Configuration>;
    using value_type = typename fixture_type::value_type;
    fixture_type fixture;
    fixture.covariance = fixture_type::covariance_type::identity();
    fixture.imu.angular_rate_b(2U) = value_type{0.25};
    fixture.parameters.process_noise.angular_rate_variance(0U) = value_type{-1};
    auto nominal_output = fixture.state;
    Status const status =
        formal_eskf::predict_nominal(fixture.state, fixture.imu, value_type{0.5}, fixture.parameters, nominal_output);
    test.expect(status == Status::success && !same_state(nominal_output, fixture.state), profile,
                "nominal prediction succeeds and changes attitude before covariance validation fails");
    expect_failed_prediction(test, profile, fixture, value_type{0.5}, Status::domain_error);

    // Finite operands and valid parameters; the covariance result exceeds the scalar range.
    fixture.covariance = fixture.covariance * std::numeric_limits<value_type>::max();
    fixture.parameters.process_noise.angular_rate_variance(0U) = std::numeric_limits<value_type>::max();
    expect_failed_prediction(test, profile, fixture, value_type{0.5}, Status::non_finite_result);
}

template <typename Linalg> void test_ins_input_boundary(TestContext & test, std::string_view profile)
{
    using fixture_type = CovariancePredictionFixture<Linalg, formal_eskf::configuration::Ins>;
    using value_type = typename Linalg::value_type;
    using covariance_type = typename fixture_type::covariance_type;
    fixture_type fixture;
    covariance_type const sentinel = covariance_type::identity() * value_type{7};
    auto output = sentinel;
    value_type const nan = std::numeric_limits<value_type>::quiet_NaN();
    value_type const inf = std::numeric_limits<value_type>::infinity();
    value_type const largest = std::numeric_limits<value_type>::max();
    std::array<formal_eskf::linalg::Vector<Linalg, 3U> *, 3> const consumed{&fixture.imu.specific_force_b,
                                                                            &fixture.state.b_a, &fixture.state.b_g};
    for (auto * field : consumed)
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            for (value_type const invalid : {nan, inf})
            {
                (*field)(axis) = invalid;
                Status const status = predict_covariance(fixture.state, fixture.covariance, fixture.imu,
                                                         value_type{0.5}, fixture.parameters, output);
                test.expect(status == Status::non_finite_input && matrix_near(output, sentinel, value_type{0}), profile,
                            "every consumed INS force and bias coefficient must be finite");
            }
            (*field)(axis) = value_type{0};
        }
    }
    fixture.imu.specific_force_b(0U) = largest;
    fixture.state.b_a(0U) = -largest;
    Status status =
        predict_covariance(fixture.state, fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters, output);
    test.expect(status == Status::non_finite_result && matrix_near(output, sentinel, value_type{0}), profile,
                "overflow in bias-corrected force is distinguished from invalid input");
    fixture.imu.specific_force_b = {};
    fixture.state.b_a = {};
    fixture.imu.angular_rate_b(0U) = largest;
    fixture.state.b_g(0U) = -largest;
    status =
        predict_covariance(fixture.state, fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters, output);
    test.expect(status == Status::non_finite_result && matrix_near(output, sentinel, value_type{0}), profile,
                "overflow in bias-corrected angular rate leaves output unchanged");
    fixture.imu.angular_rate_b = {};
    fixture.state.b_g = {};
    fixture.imu.specific_force_b(0U) = largest;
    status =
        predict_covariance(fixture.state, fixture.covariance, fixture.imu, value_type{2}, fixture.parameters, output);
    test.expect(status == Status::non_finite_result && matrix_near(output, sentinel, value_type{0}), profile,
                "overflow in the velocity-attitude transition is detected before covariance multiplication");

    fixture.imu.specific_force_b = {};
    fixture.state.p_n(0U) = nan;
    fixture.state.v_n(1U) = nan;
    fixture.parameters.gravity_n(2U) = nan;
    status =
        predict_covariance(fixture.state, fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters, output);
    test.expect(status == Status::success && formal_eskf::linalg::max_abs(output) == value_type{0}, profile,
                "standalone covariance does not consume position, velocity, or gravity");
}

template <typename Linalg, typename Configuration>
void test_prior_rejection(TestContext & test, std::string_view profile)
{
    using fixture_type = CovariancePredictionFixture<Linalg, Configuration>;
    using value_type = typename fixture_type::value_type;
    using quaternion_type = formal_eskf::so3::UnitQuaternion<Linalg>;
    fixture_type fixture;
    fixture.covariance = fixture_type::covariance_type::identity();
    int const exponent = (std::numeric_limits<value_type>::min_exponent - std::numeric_limits<value_type>::digits) / 2;
    value_type const tiny = std::ldexp(value_type{1}, exponent);
    // Public-constructor underflow fixture, with squared norm exactly 1.25.
    auto const construction =
        quaternion_type::from_coefficients(tiny, tiny * value_type{0.5}, value_type{0}, value_type{0},
                                           std::numeric_limits<value_type>::min(), fixture.state.q_nb);
    test.expect(construction == Status::success && fixture.state.q_nb.q0() == value_type{1} &&
                    fixture.state.q_nb.q1() == value_type{0.5},
                profile, "non-unit prior fixture is constructed");
    auto const initial = fixture;
    Status status;
    if constexpr (fixture_type::is_ins)
    {
        status = predict_covariance(fixture.state, fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters,
                                    fixture.covariance);
    }
    else
    {
        status = predict_covariance(fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters,
                                    fixture.covariance);
    }
    test.expect(status == (fixture_type::is_ins ? Status::invalid_quaternion_norm : Status::success) &&
                    matrix_near(fixture.covariance, initial.covariance, value_type{0}) &&
                    same_state(fixture.state, initial.state),
                profile, "standalone covariance checks prior attitude only when its configuration consumes it");

    for (bool state_alias : {false, true})
    {
        for (bool covariance_alias : {false, true})
        {
            auto state = initial.state;
            auto covariance = initial.covariance;
            auto separate_state = initial.state;
            separate_state.q_nb = -separate_state.q_nb;
            auto separate_covariance = initial.covariance * value_type{2};
            auto & state_output = state_alias ? state : separate_state;
            auto & covariance_output = covariance_alias ? covariance : separate_covariance;
            auto const state_before = state_output;
            auto const covariance_before = covariance_output;
            status = predict(state, covariance, fixture.imu, value_type{0.5}, fixture.parameters, state_output,
                             covariance_output);
            test.expect(
                status == Status::invalid_quaternion_norm && same_state(state_output, state_before) &&
                    matrix_near(covariance_output, covariance_before, value_type{0}) &&
                    same_state(state, initial.state) && matrix_near(covariance, initial.covariance, value_type{0}),
                profile, "combined prior rejection preserves both outputs for every complete-object alias case");
        }
    }
    fixture.parameters.quaternion_squared_norm_tolerance = std::numeric_limits<value_type>::quiet_NaN();
    if constexpr (fixture_type::is_ins)
    {
        status = predict_covariance(fixture.state, fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters,
                                    fixture.covariance);
    }
    else
    {
        status = predict_covariance(fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters,
                                    fixture.covariance);
    }
    test.expect(status == (fixture_type::is_ins ? Status::non_finite_input : Status::success) &&
                    matrix_near(fixture.covariance, initial.covariance, value_type{0}),
                profile, "AHRS covariance ignores the unused prior tolerance while INS validates it");
}

template <typename Linalg> void test_noise_accumulation(TestContext & test, std::string_view profile)
{
    constexpr auto tolerance = typename Linalg::value_type{0}; // Exact dyadic zero-rate fixtures.
    using value_type = typename Linalg::value_type;
    CovariancePredictionFixture<Linalg, formal_eskf::configuration::Ins> fixture;
    fixture.parameters.process_noise.specific_force_variance(0U) = value_type{4};
    for (std::size_t step = 0U; step < 2U; ++step)
    {
        Status const status = predict(fixture.state, fixture.covariance, fixture.imu, value_type{0.5},
                                      fixture.parameters, fixture.state, fixture.covariance);
        test.expect(status == Status::success, profile, "two-step noise accumulation succeeds");
    }
    test.expect(near(fixture.covariance(0U, 0U), value_type{0.25}, tolerance) &&
                    near(fixture.covariance(0U, 3U), value_type{0.5}, tolerance) &&
                    near(fixture.covariance(3U, 3U), value_type{2}, tolerance),
                profile, "previous velocity noise reaches position and cross-covariance on the next step");

    fixture = {};
    fixture.parameters.process_noise.accelerometer_bias_random_walk_variance_density(0U) = value_type{4};
    fixture.parameters.process_noise.gyroscope_bias_random_walk_variance_density(1U) = value_type{8};
    for (value_type const dt : {value_type{0.125}, value_type{0.375}, value_type{0.5}})
    {
        Status const status = predict(fixture.state, fixture.covariance, fixture.imu, dt, fixture.parameters,
                                      fixture.state, fixture.covariance);
        test.expect(status == Status::success, profile, "variable-dt bias random walk succeeds");
    }
    test.expect(near(fixture.covariance(9U, 9U), value_type{4}, tolerance) &&
                    near(fixture.covariance(13U, 13U), value_type{8}, tolerance) &&
                    formal_eskf::linalg::max_abs(fixture.state.b_a) == value_type{0} &&
                    formal_eskf::linalg::max_abs(fixture.state.b_g) == value_type{0},
                profile, "bias variance accumulates with elapsed time while nominal biases stay constant");

    CovariancePredictionFixture<Linalg, formal_eskf::configuration::Ahrs> ahrs;
    ahrs.imu.specific_force_b(0U) = std::numeric_limits<value_type>::quiet_NaN();
    Status const status =
        predict(ahrs.state, ahrs.covariance, ahrs.imu, value_type{0.5}, ahrs.parameters, ahrs.state, ahrs.covariance);
    test.expect(status == Status::success && formal_eskf::linalg::max_abs(ahrs.covariance) == value_type{0}, profile,
                "rotation-only prediction does not consume accelerometer measurements");
}

template <typename Linalg, typename Configuration>
void test_covariance_alias(TestContext & test, std::string_view profile)
{
    using fixture_type = CovariancePredictionFixture<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    enum class Outcome
    {
        success,
        invalid_parameter,
        invalid_noise,
        nonfinite_covariance,
        transition_overflow,
        covariance_overflow,
        invalid_prior
    };
    for (Outcome const outcome :
         {Outcome::success, Outcome::invalid_parameter, Outcome::invalid_noise, Outcome::nonfinite_covariance,
          Outcome::transition_overflow, Outcome::covariance_overflow, Outcome::invalid_prior})
    {
        auto fixture = dense_fixture<Linalg, Configuration>(test, profile);
        if constexpr (fixture_type::is_ins)
        {
            fixture.imu.angular_rate_b = fixture.state.b_g;
        }
        else
        {
            fixture.imu.angular_rate_b = {};
        }
        auto const expected = reference_covariance(fixture, 0.25L);
        value_type dt = value_type{0.25};
        Status expected_status = Status::success;
        switch (outcome)
        {
        case Outcome::success:
            break;
        case Outcome::invalid_parameter:
            fixture.parameters.dt_min = value_type{0};
            expected_status = Status::domain_error;
            break;
        case Outcome::invalid_noise:
            fixture.parameters.process_noise.angular_rate_variance(2U) = value_type{-1};
            expected_status = Status::domain_error;
            break;
        case Outcome::nonfinite_covariance:
            fixture.covariance(0U, 0U) = std::numeric_limits<value_type>::quiet_NaN();
            expected_status = Status::non_finite_input;
            break;
        case Outcome::transition_overflow:
            fixture.imu.angular_rate_b(0U) = std::numeric_limits<value_type>::max();
            dt = value_type{2};
            expected_status = Status::non_finite_result;
            break;
        case Outcome::covariance_overflow:
            fixture.covariance = fixture_type::covariance_type::identity() * std::numeric_limits<value_type>::max();
            fixture.parameters.process_noise.angular_rate_variance(0U) = std::numeric_limits<value_type>::max();
            dt = value_type{1};
            expected_status = Status::non_finite_result;
            break;
        case Outcome::invalid_prior:
            if constexpr (fixture_type::is_ins)
            {
                int const exponent =
                    (std::numeric_limits<value_type>::min_exponent - std::numeric_limits<value_type>::digits) / 2;
                value_type const tiny = std::ldexp(value_type{1}, exponent);
                auto const status = formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(
                    tiny, tiny * value_type{0.5}, value_type{0}, value_type{0}, std::numeric_limits<value_type>::min(),
                    fixture.state.q_nb);
                test.expect(status == Status::success && fixture.state.q_nb.q1() == value_type{0.5}, profile,
                            "malformed prior is constructed through the public normalization boundary");
                expected_status = Status::invalid_quaternion_norm;
            }
            else
            {
                continue; // AHRS covariance has no nominal-state argument.
            }
            break;
        }
        for (bool const alias : {false, true})
        {
            auto local = fixture;
            auto separate = fixture.covariance;
            separate(0U, 1U) = -value_type{0};
            separate(1U, 0U) = std::numeric_limits<value_type>::quiet_NaN();
            auto & output = alias ? local.covariance : separate;
            auto const before = output;
            Status status;
            if constexpr (fixture_type::is_ins)
            {
                status = predict_covariance(local.state, local.covariance, local.imu, dt, local.parameters, output);
            }
            else
            {
                status = predict_covariance(local.covariance, local.imu, dt, local.parameters, output);
            }
            test.expect(status == expected_status &&
                            (expected_status != Status::success ? same_matrix_fields(output, before)
                                                                : matrix_near(output, expected, value_type{0})),
                        profile, "separate and whole-P alias satisfy independent exact success and full rollback");
            test.expect(same_state_fields(local.state, fixture.state) &&
                            (alias || same_matrix_fields(local.covariance, fixture.covariance)),
                        profile, "standalone covariance does not mutate borrowed nominal or separate prior covariance");
            expect_unchanged_imu_and_parameters(test, profile, local, fixture);
        }
    }
}

template <typename Linalg, typename Configuration>
void test_consumption_failure(TestContext & test, std::string_view profile)
{
    using fixture_type = CovariancePredictionFixture<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    if constexpr (fixture_type::is_ins)
    {
        test_ins_input_boundary<Linalg>(test, profile);
    }
    else
    {
        fixture_type fixture;
        fixture.covariance = fixture_type::covariance_type::identity();
        fixture.imu.specific_force_b(0U) = std::numeric_limits<value_type>::quiet_NaN();
        fixture.parameters.quaternion_squared_norm_tolerance = std::numeric_limits<value_type>::quiet_NaN();
        auto output = fixture.covariance;
        auto const status =
            predict_covariance(fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters, output);
        test.expect(status == Status::success && same_matrix_fields(output, fixture.covariance), profile,
                    "AHRS covariance ignores force and the nominal-only quaternion tolerance");
    }
    fixture_type fixture;
    for (std::size_t row = 0U; row < fixture_type::size; ++row)
    {
        for (std::size_t col = 0U; col < fixture_type::size; ++col)
        {
            fixture.covariance(row, col) = std::numeric_limits<value_type>::quiet_NaN();
            for (bool const alias : {false, true})
            {
                auto local = fixture;
                auto separate = fixture_type::covariance_type::identity();
                separate(0U, 1U) = -value_type{0};
                auto & output = alias ? local.covariance : separate;
                auto const before = output;
                Status status;
                if constexpr (fixture_type::is_ins)
                {
                    status = predict_covariance(local.state, local.covariance, local.imu, value_type{0.5},
                                                local.parameters, output);
                }
                else
                {
                    status = predict_covariance(local.covariance, local.imu, value_type{0.5}, local.parameters, output);
                }
                test.expect(status == Status::non_finite_input && same_matrix_fields(output, before), profile,
                            "each consumed covariance coefficient fails without partial publication for both aliases");
            }
            fixture.covariance(row, col) = value_type{0};
        }
    }
}

template <typename Linalg, typename Configuration>
void test_joint_publication(TestContext & test, std::string_view profile)
{
    test_joint_success<Linalg, Configuration>(test, profile);
    test_prediction_failures<Linalg, Configuration>(test, profile);
    test_first_failure<Linalg, Configuration>(test, profile);
    test_late_failure<Linalg, Configuration>(test, profile);
    test_prior_rejection<Linalg, Configuration>(test, profile);
}

template <typename Linalg> void test_noise_frame(TestContext & test, std::string_view profile)
{
    using fixture_type = CovariancePredictionFixture<Linalg, formal_eskf::configuration::Ins>;
    using value_type = typename Linalg::value_type;
    fixture_type fixture;
    fixture.set_distinct_noise();
    test.expect(formal_eskf::so3::UnitQuaternion<Linalg>::from_coefficients(
                    value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5},
                    fixture.state.q_nb) == Status::success,
                profile, "exact cyclic noise-frame fixture is valid");
    for (value_type const dt : {value_type{0.25}, value_type{0.5}})
    {
        typename fixture_type::covariance_type output;
        auto const status =
            predict_covariance(fixture.state, fixture.covariance, fixture.imu, dt, fixture.parameters, output);
        test.expect(status == Status::success, profile, "zero prior with independent source noise is supported");
        for (std::size_t row = 0U; row < 15U; ++row)
        {
            for (std::size_t col = 0U; col < 15U; ++col)
            {
                value_type expected = value_type{0};
                if (row == col && row >= 3U && row < 6U)
                {
                    constexpr unsigned rotated_axes[3] = {3U, 1U, 2U};
                    expected = static_cast<value_type>(rotated_axes[row - 3U]) * dt * dt;
                }
                else if (row == col && row >= 6U)
                {
                    expected = static_cast<value_type>(row - 2U) * (row < 9U ? dt * dt : dt);
                }
                test.expect(output(row, col) == expected, profile,
                            "every source appears once in the exact rotated block with no position/cross-source noise");
            }
        }
    }
    test_rotated_noise<Linalg>(test, profile); // Supplemental nonaxis numeric regression.
    test_noise_accumulation<Linalg>(test, profile);
}

} /* end namespace */

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using float_backend = formal_eskf::test::Backend<float>;
    using double_backend = formal_eskf::test::Backend<double>;
    using formal_eskf::configuration::Ahrs;
    using formal_eskf::configuration::Ins;
    formal_eskf::test::Case const cases[] = {
        {"TC-E-NOISE-source-scaling", "INS.binary32", test_noise_discretization<float_backend, Ins>},
        {"TC-E-NOISE-source-scaling", "INS.binary64", test_noise_discretization<double_backend, Ins>},
        {"TC-E-NOISE-source-scaling", "AHRS.binary32", test_noise_discretization<float_backend, Ahrs>},
        {"TC-E-NOISE-source-scaling", "AHRS.binary64", test_noise_discretization<double_backend, Ahrs>},
        {"TC-E-NOISE-validation-order", "INS.binary32", test_noise_validation_order<float_backend, Ins>},
        {"TC-E-NOISE-validation-order", "INS.binary64", test_noise_validation_order<double_backend, Ins>},
        {"TC-E-NOISE-validation-order", "AHRS.binary32", test_noise_validation_order<float_backend, Ahrs>},
        {"TC-E-NOISE-validation-order", "AHRS.binary64", test_noise_validation_order<double_backend, Ahrs>},
        {"TC-E-NOISE-ieee-scaling", "INS.binary32", test_noise_ieee_scaling<float_backend, Ins>},
        {"TC-E-NOISE-ieee-scaling", "INS.binary64", test_noise_ieee_scaling<double_backend, Ins>},
        {"TC-E-NOISE-ieee-scaling", "AHRS.binary32", test_noise_ieee_scaling<float_backend, Ahrs>},
        {"TC-E-NOISE-ieee-scaling", "AHRS.binary64", test_noise_ieee_scaling<double_backend, Ahrs>},
        {"TC-E-PCOV-full-transition", "INS.binary32", test_full_transition<float_backend, Ins>},
        {"TC-E-PCOV-full-transition", "INS.binary64", test_full_transition<double_backend, Ins>},
        {"TC-E-PCOV-full-transition", "AHRS.binary32", test_full_transition<float_backend, Ahrs>},
        {"TC-E-PCOV-full-transition", "AHRS.binary64", test_full_transition<double_backend, Ahrs>},
        {"TC-E-PCOV-noise-frame", "INS+AHRS.binary32", test_noise_frame<float_backend>},
        {"TC-E-PCOV-noise-frame", "INS+AHRS.binary64", test_noise_frame<double_backend>},
        {"TC-E-PCOV-consumption-failure", "INS.binary32", test_consumption_failure<float_backend, Ins>},
        {"TC-E-PCOV-consumption-failure", "INS.binary64", test_consumption_failure<double_backend, Ins>},
        {"TC-E-PCOV-consumption-failure", "AHRS.binary32", test_consumption_failure<float_backend, Ahrs>},
        {"TC-E-PCOV-consumption-failure", "AHRS.binary64", test_consumption_failure<double_backend, Ahrs>},
        {"TC-E-PCOV-finalization", "INS+AHRS.binary32", test_finalization<float_backend>},
        {"TC-E-PCOV-finalization", "INS+AHRS.binary64", test_finalization<double_backend>},
        {"TC-E-PCOV-alias-frame", "INS.binary32", test_covariance_alias<float_backend, Ins>},
        {"TC-E-PCOV-alias-frame", "INS.binary64", test_covariance_alias<double_backend, Ins>},
        {"TC-E-PCOV-alias-frame", "AHRS.binary32", test_covariance_alias<float_backend, Ahrs>},
        {"TC-E-PCOV-alias-frame", "AHRS.binary64", test_covariance_alias<double_backend, Ahrs>},
        {"TC-E-STEP-PRED-joint-publication", "INS.binary32", test_joint_publication<float_backend, Ins>},
        {"TC-E-STEP-PRED-joint-publication", "INS.binary64", test_joint_publication<double_backend, Ins>},
        {"TC-E-STEP-PRED-joint-publication", "AHRS.binary32", test_joint_publication<float_backend, Ahrs>},
        {"TC-E-STEP-PRED-joint-publication", "AHRS.binary64", test_joint_publication<double_backend, Ahrs>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
