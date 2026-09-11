/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string_view>
#include <type_traits>

#include <Eigen/Eigenvalues>

#include <formal_eskf/eskf/eskf.hpp>
#include <formal_eskf/linalg/backend/eigen.hpp>

#include "test_support.hpp"

namespace
{

using TestContext = formal_eskf::test::Context;
using formal_eskf::Status;
using formal_eskf::try_discretize_process_noise;
using formal_eskf::try_predict;
using formal_eskf::try_predict_covariance;
using formal_eskf::test::near;

template <typename Linalg, typename Configuration> struct Fixture
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

    Fixture()
    {
        parameters.dt_min = value_type{0.125};
        parameters.dt_max = value_type{2};
        parameters.minimum_quaternion_norm = static_cast<value_type>(1.0e-6);
    }

    [[nodiscard]] auto noise_groups()
    {
        using vector_type = typename Linalg::template vector_type<3U>;
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
                group->set(axis, static_cast<value_type>(coefficient++));
            }
        }
    }
};

template <typename Matrix>
[[nodiscard]] bool matrix_near(Matrix const & actual, Matrix const & expected, typename Matrix::value_type tolerance)
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

template <typename State> [[nodiscard]] bool same_state(State const & actual, State const & expected)
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

template <typename Linalg, typename Configuration>
void test_noise_discretization(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using fixture_type = Fixture<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    using noise_matrix = typename fixture_type::noise_covariance_type;
    fixture_type fixture;
    fixture.set_distinct_noise();
    noise_matrix Q_i;
    noise_matrix Q_twice;
    value_type const dt = value_type{0.25};
    Status status = try_discretize_process_noise(fixture.parameters.process_noise, dt, Q_i);
    test.expect(status == Status::success, profile, "equation (15) accepts per-axis variance and intensity");
    status = try_discretize_process_noise(fixture.parameters.process_noise, value_type{2} * dt, Q_twice);
    test.expect(status == Status::success, profile, "noise is recomputed for the current step length");
    for (std::size_t row = 0U; row < noise_matrix::row_count; ++row)
    {
        bool const bias = fixture_type::is_ins && row >= 6U;
        value_type const expected = static_cast<value_type>(row + 1U) * (bias ? dt : dt * dt);
        test.expect(near(Q_i(row, row), expected, tolerance), profile,
                    "each noise block has the prescribed scale and is not squared twice");
        test.expect(near(Q_twice(row, row), Q_i(row, row) * (bias ? value_type{2} : value_type{4}), tolerance), profile,
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
            groups[group]->set(axis, target / (bias ? fixed_dt : fixed_dt * fixed_dt));
        }
    }
    status = try_discretize_process_noise(fixture.parameters.process_noise, fixed_dt, Q_i);
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
    status = try_discretize_process_noise(fixture.parameters.process_noise, value_type{1}, Q_i);
    test.expect(status == Status::success && formal_eskf::linalg::max_abs(Q_i) == value_type{0}, profile,
                "zero process noise is valid");
    status =
        try_discretize_process_noise(fixture.parameters.process_noise, std::numeric_limits<value_type>::max(), Q_i);
    test.expect(status == Status::success && formal_eskf::linalg::max_abs(Q_i) == value_type{0}, profile,
                "zero noise does not spuriously overflow by forming dt squared first");
}

template <typename Linalg, typename Configuration>
void test_noise_failures(TestContext & test, std::string_view profile)
{
    using fixture_type = Fixture<Linalg, Configuration>;
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
        Status const status = try_discretize_process_noise(fixture.parameters.process_noise, dt, output);
        test.expect(status == expected && matrix_near(output, sentinel, value_type{0}), profile,
                    "invalid noise time step leaves output unchanged");
    }
    for (auto * group : fixture.noise_groups())
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            for (value_type const invalid : {value_type{-1}, nan, inf})
            {
                group->set(axis, invalid);
                Status const expected = std::isfinite(invalid) ? Status::domain_error : Status::non_finite_input;
                Status const status =
                    try_discretize_process_noise(fixture.parameters.process_noise, value_type{1}, output);
                test.expect(status == expected && matrix_near(output, sentinel, value_type{0}), profile,
                            "every invalid noise coefficient is rejected without partial output");
            }
            group->set(axis, std::numeric_limits<value_type>::max());
            Status const status = try_discretize_process_noise(fixture.parameters.process_noise, value_type{2}, output);
            test.expect(status == Status::non_finite_result && matrix_near(output, sentinel, value_type{0}), profile,
                        "overflow in each discretized noise coefficient leaves output unchanged");
            group->set(axis, value_type{0});
        }
    }
}

// Independent Rodrigues oracle for R(Exp(theta))^T, using ordinary arrays and
// scalar loops rather than production quaternion, hat, block, or sandwich code.
[[nodiscard]] std::array<double, 9> reference_attitude_transition(std::array<double, 3> const & omega, double dt)
{
    std::array<double, 9> const W{
        0, -omega[2] * dt, omega[1] * dt, omega[2] * dt, 0, -omega[0] * dt, -omega[1] * dt, omega[0] * dt, 0};
    double first = 1;
    double second = 0;
#if !ESKF_QUAT_APPROX
    double const theta = dt * std::sqrt(omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2]);
    first = theta == 0 ? 1 : std::sin(theta) / theta;
    second = theta == 0 ? 0.5 : (1 - std::cos(theta)) / (theta * theta);
#endif
    std::array<double, 9> result{};
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            double square = 0;
            for (std::size_t inner = 0U; inner < 3U; ++inner)
            {
                square += W[row * 3U + inner] * W[inner * 3U + column];
            }
            result[row * 3U + column] = (row == column ? 1.0 : 0.0) - first * W[row * 3U + column] + second * square;
        }
    }
    return result;
}

template <typename Linalg, typename Configuration>
[[nodiscard]] auto reference_covariance(Fixture<Linalg, Configuration> const & fixture, double dt)
{
    using fixture_type = Fixture<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    constexpr std::size_t size = fixture_type::size;
    std::array<double, size * size> F{};
    for (std::size_t row = 0U; row < size; ++row)
    {
        F[row * size + row] = 1;
    }
    // The dense fixture has q=(0.5,0.5,0.5,0.5), hence x->y, y->z, z->x.
    std::array<double, 9> const R{0, 0, 1, 1, 0, 0, 0, 1, 0};
    std::array<double, 3> omega{};
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        omega[axis] = static_cast<double>(fixture.imu.angular_rate_b(axis));
        if constexpr (fixture_type::is_ins)
        {
            omega[axis] -= static_cast<double>(fixture.state.b_g(axis));
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
        std::array<double, 3> force{};
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            force[axis] =
                static_cast<double>(fixture.imu.specific_force_b(axis)) - static_cast<double>(fixture.state.b_a(axis));
            F[axis * size + 3U + axis] = dt;
            F[(6U + axis) * size + 12U + axis] = -dt;
        }
        std::array<double, 9> const A{0, -force[2], force[1], force[2], 0, -force[0], -force[1], force[0], 0};
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
            double coefficient = 0;
            for (std::size_t left = 0U; left < size; ++left)
            {
                for (std::size_t right = 0U; right < size; ++right)
                {
                    coefficient += F[row * size + left] * static_cast<double>(fixture.covariance(left, right)) *
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
                                       static_cast<double>(axis + 1U) * dt * dt;
                    }
                }
                if (row == column && row >= 6U)
                {
                    coefficient += static_cast<double>(row - 2U) * (row < 9U ? dt * dt : dt);
                }
            }
            else if (row == column)
            {
                coefficient += static_cast<double>(row + 1U) * dt * dt;
            }
            result.set(row, column, static_cast<value_type>(coefficient));
        }
    }
    return result;
}

template <typename Matrix>
[[nodiscard]] bool representative_psd(Matrix const & covariance, typename Matrix::value_type tolerance)
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

template <typename Linalg, typename Configuration>
void test_dense_prediction(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using fixture_type = Fixture<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    fixture_type fixture;
    fixture.set_distinct_noise();
    Status status = formal_eskf::so3::UnitQuaternion<Linalg>::try_from_coefficients(
        value_type{0.5}, value_type{0.5}, value_type{0.5}, value_type{0.5}, fixture.parameters.minimum_quaternion_norm,
        fixture.state.q_nb);
    test.expect(status == Status::success, profile, "cyclic body-to-navigation rotation fixture is valid");
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        fixture.imu.angular_rate_b.set(axis, static_cast<value_type>(axis + 1U));
        if constexpr (fixture_type::is_ins)
        {
            fixture.state.b_g.set(axis, value_type{0.5});
            fixture.state.b_a.set(axis, static_cast<value_type>(axis + 2U));
            fixture.imu.specific_force_b.set(axis, static_cast<value_type>(2U * axis + 1U));
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
            fixture.covariance.set(row, column, u * v + (row == column ? value_type{1} : value_type{0}));
        }
    }
    auto const initial = fixture;
    value_type const dt = value_type{0.25};
    auto const expected = reference_covariance(initial, static_cast<double>(dt));
    auto output = fixture.covariance;
    status = try_predict_covariance(fixture.state, fixture.covariance, fixture.imu, dt, fixture.parameters, output);
    test.expect(status == Status::success && matrix_near(output, expected, tolerance), profile,
                "all covariance coefficients match an independent dense equation (18) oracle");
    test.expect(formal_eskf::linalg::is_symmetric(output, value_type{0}) && representative_psd(output, tolerance),
                profile, "representative propagated covariance is symmetric and numerically PSD");

    auto nominal = fixture.state;
    status = formal_eskf::try_predict_nominal(fixture.state, fixture.imu, dt, fixture.parameters, nominal);
    test.expect(status == Status::success, profile, "independent nominal prediction succeeds");
    status = try_predict(fixture.state, fixture.covariance, fixture.imu, dt, fixture.parameters, fixture.state,
                         fixture.covariance);
    test.expect(status == Status::success && same_state(fixture.state, nominal) &&
                    matrix_near(fixture.covariance, expected, tolerance),
                profile, "atomic in-place prediction linearizes at the old state, not the updated attitude");

    auto separate_state = initial.state;
    auto separate_covariance = initial.covariance;
    status = try_predict(initial.state, initial.covariance, initial.imu, dt, initial.parameters, separate_state,
                         separate_covariance);
    test.expect(status == Status::success && same_state(separate_state, fixture.state) &&
                    matrix_near(separate_covariance, fixture.covariance, value_type{0}),
                profile, "separate and aliased outputs agree");

    auto negative = initial.state;
    negative.q_nb = -negative.q_nb;
    status = try_predict_covariance(negative, initial.covariance, initial.imu, dt, initial.parameters, output);
    test.expect(status == Status::success && matrix_near(output, expected, tolerance), profile,
                "covariance prediction is invariant to quaternion sign");

    bool all_succeeded = true;
    for (std::size_t step = 0U; step < 20U; ++step)
    {
        status = try_predict(fixture.state, fixture.covariance, fixture.imu, value_type{0.125}, fixture.parameters,
                             fixture.state, fixture.covariance);
        all_succeeded = all_succeeded && status == Status::success;
    }
    test.expect(all_succeeded && formal_eskf::linalg::is_symmetric(fixture.covariance, value_type{0}) &&
                    representative_psd(fixture.covariance, tolerance * value_type{10}),
                profile, "repeated prediction preserves representative covariance symmetry and PSD within tolerance");
}

template <typename Linalg>
void test_rotated_noise(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using fixture_type = Fixture<Linalg, formal_eskf::configuration::Ins>;
    using value_type = typename Linalg::value_type;
    fixture_type fixture;
    value_type const angle = static_cast<value_type>(std::acos(-1.0) / 8.0);
    Status status = formal_eskf::so3::UnitQuaternion<Linalg>::try_from_coefficients(
        std::cos(angle), value_type{0}, value_type{0}, std::sin(angle), fixture.parameters.minimum_quaternion_norm,
        fixture.state.q_nb);
    test.expect(status == Status::success, profile, "45 degree yaw fixture is valid");
    auto & variance = fixture.parameters.process_noise.specific_force_variance;
    variance.set(0U, value_type{1});
    variance.set(1U, value_type{3});
    variance.set(2U, value_type{5});
    status = try_predict_covariance(fixture.state, fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters,
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
        variance.set(axis, value_type{4});
    }
    status = try_predict_covariance(fixture.state, fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters,
                                    fixture.covariance);
    auto const velocity_noise = fixture.covariance.template block<3U, 3U, 3U, 3U>();
    test.expect(status == Status::success &&
                    matrix_near(velocity_noise, decltype(velocity_noise)::identity(), tolerance),
                profile, "isotropic noise recovers the paper's identity-frame injection");
}

template <typename Linalg, typename Configuration>
void test_prediction_failures(TestContext & test, std::string_view profile)
{
    using fixture_type = Fixture<Linalg, Configuration>;
    using value_type = typename Linalg::value_type;
    using covariance_type = typename fixture_type::covariance_type;
    fixture_type fixture;
    fixture.imu.angular_rate_b.set(2U, value_type{1});
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
                group->set(axis, invalid);
                Status const expected = std::isfinite(invalid) ? Status::domain_error : Status::non_finite_input;
                Status const status = try_predict(fixture.state, fixture.covariance, fixture.imu, value_type{0.5},
                                                  fixture.parameters, state_output, covariance_output);
                test.expect(status == expected && same_state(state_output, state_before) &&
                                matrix_near(covariance_output, sentinel, value_type{0}),
                            profile,
                            "invalid process noise rolls back BOTH outputs after successful nominal calculation");
            }
            group->set(axis, value_type{0});
        }
    }
    for (std::size_t row = 0U; row < fixture_type::size; ++row)
    {
        for (std::size_t column = 0U; column < fixture_type::size; ++column)
        {
            fixture.covariance.set(row, column, nan);
            Status const status = try_predict(fixture.state, fixture.covariance, fixture.imu, value_type{0.5},
                                              fixture.parameters, state_output, covariance_output);
            test.expect(status == Status::non_finite_input && same_state(state_output, state_before) &&
                            matrix_near(covariance_output, sentinel, value_type{0}),
                        profile, "non-finite covariance at every position rolls back both outputs");
            fixture.covariance.set(row, column, value_type{0});
        }
    }
    for (value_type const dt : {value_type{0}, value_type{-1}, value_type{0.0625}, value_type{4}, nan, inf})
    {
        Status const expected = std::isfinite(dt) ? Status::out_of_range : Status::non_finite_input;
        Status const status = try_predict_covariance(fixture.state, fixture.covariance, fixture.imu, dt,
                                                     fixture.parameters, covariance_output);
        test.expect(status == expected && matrix_near(covariance_output, sentinel, value_type{0}), profile,
                    "covariance prediction enforces finite dt and inclusive application time bounds");
    }
    for (value_type const dt : {fixture.parameters.dt_min, fixture.parameters.dt_max})
    {
        auto output = sentinel;
        Status const status =
            try_predict_covariance(fixture.state, fixture.covariance, fixture.imu, dt, fixture.parameters, output);
        test.expect(status == Status::success && formal_eskf::linalg::max_abs(output) == value_type{0}, profile,
                    "both inclusive time boundaries accept zero covariance and noise");
    }
    auto parameters = fixture.parameters;
    parameters.dt_min = value_type{0};
    Status status = try_predict_covariance(fixture.state, fixture.covariance, fixture.imu, value_type{0.5}, parameters,
                                           covariance_output);
    test.expect(status == Status::domain_error && matrix_near(covariance_output, sentinel, value_type{0}), profile,
                "invalid time configuration is rejected");
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        auto imu = fixture.imu;
        imu.angular_rate_b.set(axis, inf);
        status = try_predict_covariance(fixture.state, fixture.covariance, imu, value_type{0.5}, fixture.parameters,
                                        covariance_output);
        test.expect(status == Status::non_finite_input && matrix_near(covariance_output, sentinel, value_type{0}),
                    profile, "non-finite angular rate is rejected by standalone covariance prediction");
    }
    auto imu = fixture.imu;
    imu.angular_rate_b.set(0U, std::numeric_limits<value_type>::max());
    status = try_predict_covariance(fixture.state, fixture.covariance, imu, value_type{2}, fixture.parameters,
                                    covariance_output);
    test.expect(status == Status::non_finite_result && matrix_near(covariance_output, sentinel, value_type{0}), profile,
                "overflow in angular increment leaves covariance unchanged");

    // Each operand is finite; only FPF^T + Q overflows.
    fixture.imu.angular_rate_b = {};
    fixture.covariance = covariance_type::identity() * std::numeric_limits<value_type>::max();
    fixture.parameters.process_noise.angular_rate_variance.set(0U, std::numeric_limits<value_type>::max());
    status = try_predict(fixture.state, fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters,
                         state_output, covariance_output);
    test.expect(status == Status::non_finite_result && same_state(state_output, state_before) &&
                    matrix_near(covariance_output, sentinel, value_type{0}),
                profile, "late covariance overflow leaves both outputs unchanged");

    // In-place rollback too, including a nominal attitude that would change.
    fixture.covariance = sentinel;
    fixture.parameters.process_noise.angular_rate_variance.set(0U, value_type{-1});
    fixture.imu.angular_rate_b.set(2U, value_type{1});
    status = try_predict(fixture.state, fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters,
                         fixture.state, fixture.covariance);
    test.expect(status == Status::domain_error && same_state(fixture.state, state_before) &&
                    matrix_near(fixture.covariance, sentinel, value_type{0}),
                profile, "in-place prediction failure does not commit nominal state before covariance validation");

    fixture.parameters.process_noise = {};
    fixture.imu.angular_rate_b.set(0U, nan);
    status = try_predict(fixture.state, fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters,
                         fixture.state, fixture.covariance);
    test.expect(status == Status::non_finite_input && same_state(fixture.state, state_before) &&
                    matrix_near(fixture.covariance, sentinel, value_type{0}),
                profile, "nominal prediction failure also preserves both outputs");
}

template <typename Linalg> void test_ins_input_boundary(TestContext & test, std::string_view profile)
{
    using fixture_type = Fixture<Linalg, formal_eskf::configuration::Ins>;
    using value_type = typename Linalg::value_type;
    using covariance_type = typename fixture_type::covariance_type;
    fixture_type fixture;
    covariance_type const sentinel = covariance_type::identity() * value_type{7};
    auto output = sentinel;
    value_type const nan = std::numeric_limits<value_type>::quiet_NaN();
    value_type const inf = std::numeric_limits<value_type>::infinity();
    value_type const largest = std::numeric_limits<value_type>::max();
    std::array<typename Linalg::template vector_type<3U> *, 3> const consumed{&fixture.imu.specific_force_b,
                                                                              &fixture.state.b_a, &fixture.state.b_g};
    for (auto * field : consumed)
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            for (value_type const invalid : {nan, inf})
            {
                field->set(axis, invalid);
                Status const status = try_predict_covariance(fixture.state, fixture.covariance, fixture.imu,
                                                             value_type{0.5}, fixture.parameters, output);
                test.expect(status == Status::non_finite_input && matrix_near(output, sentinel, value_type{0}), profile,
                            "every consumed INS force and bias coefficient must be finite");
            }
            field->set(axis, value_type{0});
        }
    }
    fixture.imu.specific_force_b.set(0U, largest);
    fixture.state.b_a.set(0U, -largest);
    Status status = try_predict_covariance(fixture.state, fixture.covariance, fixture.imu, value_type{0.5},
                                           fixture.parameters, output);
    test.expect(status == Status::non_finite_result && matrix_near(output, sentinel, value_type{0}), profile,
                "overflow in bias-corrected force is distinguished from invalid input");
    fixture.imu.specific_force_b = {};
    fixture.state.b_a = {};
    fixture.imu.angular_rate_b.set(0U, largest);
    fixture.state.b_g.set(0U, -largest);
    status = try_predict_covariance(fixture.state, fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters,
                                    output);
    test.expect(status == Status::non_finite_result && matrix_near(output, sentinel, value_type{0}), profile,
                "overflow in bias-corrected angular rate leaves output unchanged");
    fixture.imu.angular_rate_b = {};
    fixture.state.b_g = {};
    fixture.imu.specific_force_b.set(0U, largest);
    status = try_predict_covariance(fixture.state, fixture.covariance, fixture.imu, value_type{2}, fixture.parameters,
                                    output);
    test.expect(status == Status::non_finite_result && matrix_near(output, sentinel, value_type{0}), profile,
                "overflow in the velocity-attitude transition is detected before covariance multiplication");

    fixture.imu.specific_force_b = {};
    fixture.state.p_n.set(0U, nan);
    fixture.state.v_n.set(1U, nan);
    fixture.parameters.gravity_n.set(2U, nan);
    status = try_predict_covariance(fixture.state, fixture.covariance, fixture.imu, value_type{0.5}, fixture.parameters,
                                    output);
    test.expect(status == Status::success && formal_eskf::linalg::max_abs(output) == value_type{0}, profile,
                "standalone covariance does not consume position, velocity, or gravity");
}

template <typename Linalg>
void test_noise_accumulation(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    Fixture<Linalg, formal_eskf::configuration::Ins> fixture;
    fixture.parameters.process_noise.specific_force_variance.set(0U, value_type{4});
    for (std::size_t step = 0U; step < 2U; ++step)
    {
        Status const status = try_predict(fixture.state, fixture.covariance, fixture.imu, value_type{0.5},
                                          fixture.parameters, fixture.state, fixture.covariance);
        test.expect(status == Status::success, profile, "two-step noise accumulation succeeds");
    }
    test.expect(near(fixture.covariance(0U, 0U), value_type{0.25}, tolerance) &&
                    near(fixture.covariance(0U, 3U), value_type{0.5}, tolerance) &&
                    near(fixture.covariance(3U, 3U), value_type{2}, tolerance),
                profile, "previous velocity noise reaches position and cross-covariance on the next step");

    fixture = {};
    fixture.parameters.process_noise.accelerometer_bias_random_walk_variance_density.set(0U, value_type{4});
    fixture.parameters.process_noise.gyroscope_bias_random_walk_variance_density.set(1U, value_type{8});
    for (value_type const dt : {value_type{0.125}, value_type{0.375}, value_type{0.5}})
    {
        Status const status = try_predict(fixture.state, fixture.covariance, fixture.imu, dt, fixture.parameters,
                                          fixture.state, fixture.covariance);
        test.expect(status == Status::success, profile, "variable-dt bias random walk succeeds");
    }
    test.expect(near(fixture.covariance(9U, 9U), value_type{4}, tolerance) &&
                    near(fixture.covariance(13U, 13U), value_type{8}, tolerance) &&
                    formal_eskf::linalg::max_abs(fixture.state.b_a) == value_type{0} &&
                    formal_eskf::linalg::max_abs(fixture.state.b_g) == value_type{0},
                profile, "bias variance accumulates with elapsed time while nominal biases stay constant");

    Fixture<Linalg, formal_eskf::configuration::Ahrs> ahrs;
    ahrs.imu.specific_force_b.set(0U, std::numeric_limits<value_type>::quiet_NaN());
    Status const status = try_predict(ahrs.state, ahrs.covariance, ahrs.imu, value_type{0.5}, ahrs.parameters,
                                      ahrs.state, ahrs.covariance);
    test.expect(status == Status::success && formal_eskf::linalg::max_abs(ahrs.covariance) == value_type{0}, profile,
                "rotation-only prediction does not consume accelerometer measurements");
}

template <typename Linalg, typename Configuration>
void run_covariance_tests(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    test_noise_discretization<Linalg, Configuration>(test, profile, tolerance);
    test_noise_failures<Linalg, Configuration>(test, profile);
    test_dense_prediction<Linalg, Configuration>(test, profile, tolerance);
    test_prediction_failures<Linalg, Configuration>(test, profile);
}

} /* end namespace */

int main()
{
    TestContext test;
    Eigen::internal::set_is_malloc_allowed(false);
    using double_backend = formal_eskf::linalg::EigenBackend<double>;
    using float_backend = formal_eskf::linalg::EigenBackend<float>;
    using formal_eskf::configuration::Ahrs;
    using formal_eskf::configuration::Ins;
    run_covariance_tests<double_backend, Ins>(test, "INS binary64", 1.0e-11);
    run_covariance_tests<float_backend, Ins>(test, "INS binary32", 2.0e-5F);
    run_covariance_tests<double_backend, Ahrs>(test, "AHRS binary64", 1.0e-11);
    run_covariance_tests<float_backend, Ahrs>(test, "AHRS binary32", 2.0e-5F);
    test_rotated_noise<double_backend>(test, "INS binary64", 1.0e-11);
    test_rotated_noise<float_backend>(test, "INS binary32", 2.0e-5F);
    test_ins_input_boundary<double_backend>(test, "INS binary64");
    test_ins_input_boundary<float_backend>(test, "INS binary32");
    test_noise_accumulation<double_backend>(test, "binary64", 1.0e-11);
    test_noise_accumulation<float_backend>(test, "binary32", 2.0e-5F);
    if (test.failures() != 0)
    {
        std::cerr << test.failures() << " ESKF covariance test(s) failed\n";
        return 1;
    }
    std::cout << "All ESKF covariance tests passed\n";
    return 0;
}
