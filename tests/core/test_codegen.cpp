/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "test_backend.hpp"
#include "test_cases.hpp"
#include "correction_reference.hpp"
#include <formal_eskf/eskf/eskf.hpp>
#include <formal_eskf/runtime/navigation.hpp>
#include <formal_eskf/runtime/magnetometer.hpp>
#include <cmath>
#include <string_view>
#include <limits>
#include <type_traits>

namespace
{
namespace linalg = formal_eskf::linalg;
namespace eskf = formal_eskf;
using Model = eskf::detail::CorrectionModel;

using Context = eskf::test::Context;

bool require(Context & test, std::string_view profile, bool condition, char const * message)
{
    test.expect(condition, profile, message);
    return condition;
}

template <typename L> linalg::Matrix<L, 15U, 15U> covariance_prior(Context & test, std::string_view profile)
{
    using S = typename L::value_type;
    using Covariance = linalg::Matrix<L, 15U, 15U>;
    auto B = Covariance::identity();
    for (std::size_t i = 0U; i < 15U; ++i)
    {
        for (std::size_t j = 0U; j < i; ++j)
        {
            B(i, j) = static_cast<S>(0.01) * static_cast<S>(1U + ((i + j) % 7U));
        }
    }
    auto P = linalg::sandwich(B, Covariance::identity());
    require(test, profile, eskf::succeeded(eskf::detail::finish_covariance(P, P)), "prior construction");
    return P;
}

// Observe the actual backend boundary, not a replacement numerical implementation.
template <typename Scalar> struct CountingBackend : eskf::test::Backend<Scalar>
{
    using Base = eskf::test::Backend<Scalar>;
    template <std::size_t R, std::size_t C> using Storage = typename Base::template storage_type<R, C>;
    static inline unsigned full_products = 0;
    static inline unsigned products = 0;
    template <std::size_t R, std::size_t I, std::size_t C>
    static void multiply(Storage<R, I> const & a, Storage<I, C> const & b, Storage<R, C> & output)
    {
        ++products;
        if constexpr (R == 15U && I == 15U && C == 15U)
        {
            ++full_products;
        }
        Base::template multiply<R, I, C>(a, b, output);
    }
};

template <typename Matrix> void near(Context & test, std::string_view profile, Matrix const & a, Matrix const & b)
{
    using S = typename Matrix::value_type;
    for (std::size_t i = 0; i < Matrix::row_count; ++i)
    {
        for (std::size_t j = 0; j < Matrix::column_count; ++j)
        {
            S const tolerance = S{512} * std::numeric_limits<S>::epsilon() * (S{1} + std::abs(b(i, j)));
            require(test, profile,
                    std::isfinite(a(i, j)) && std::isfinite(b(i, j)) && std::abs(a(i, j) - b(i, j)) <= tolerance,
                    "production codegen/dense numerical difference");
        }
    }
}

template <typename L>
void near_state(Context & test, std::string_view profile, eskf::configuration::Ins::NominalState<L> const & a,
                eskf::configuration::Ins::NominalState<L> const & b)
{
    near(test, profile, a.p_n, b.p_n);
    near(test, profile, a.v_n, b.v_n);
    near(test, profile, a.b_a, b.b_a);
    near(test, profile, a.b_g, b.b_g);
    near(test, profile, a.q_nb.coefficients(), b.q_nb.coefficients());
}

// The general finite-angle comparisons in this file are scoped regressions.
// Their 512-epsilon allowance is not a certified conditioning/roundoff envelope.
// Dyadic controls below compare every entry exactly and qualify caller selection
// separately from those general numerical examples.
namespace reference = eskf::test::reference;

template <typename Linalg, typename Configuration, std::size_t MeasurementSize> struct CorrectionReference
{
    using value_type = typename Linalg::value_type;
    using types = eskf::EskfTypes<Linalg, Configuration>;
    using state_type = typename types::nominal_state_type;
    using covariance_type = typename types::error_covariance_type;
    static constexpr bool is_ins = std::is_same_v<Configuration, eskf::configuration::Ins>;
    static constexpr std::size_t size = types::error_state_dimension;
    static constexpr std::size_t attitude_offset = is_ins ? 6U : 0U;
    static constexpr std::size_t measurement_size = MeasurementSize;
    state_type state{};
    covariance_type P{};
    linalg::Matrix<Linalg, MeasurementSize, size> H{};
    linalg::Vector<Linalg, MeasurementSize> r{};
    linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> V{};
};

template <typename Matrix>
void compare_reference(Context & test, std::string_view profile, Matrix const & actual,
                       reference::Dense<Matrix::row_count, Matrix::column_count> const & expected, bool exact,
                       char const * message)
{
    using value_type = typename Matrix::value_type;
    long double const allowance = exact ? 0 : 512 * std::numeric_limits<value_type>::epsilon();
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            long double const value = actual(row, column);
            test.expect(std::isfinite(value) && std::abs(value - expected[row][column]) <=
                                                    allowance * (1 + std::abs(expected[row][column])),
                        profile, message);
        }
    }
}

template <typename Linalg, Model Observation, std::size_t MeasurementSize>
void check_exact_model_products(Context & test, std::string_view profile,
                                linalg::Matrix<Linalg, MeasurementSize, 15U> const & H)
{
    using value_type = typename Linalg::value_type;
    using covariance_type = linalg::Matrix<Linalg, 15U, 15U>;
    covariance_type P;
    linalg::Matrix<Linalg, MeasurementSize, MeasurementSize> V;
    for (std::size_t row = 0U; row < 15U; ++row)
    {
        for (std::size_t column = 0U; column < 15U; ++column)
        {
            P(row, column) = value_type{0.0625} + (row == column ? value_type{1} : value_type{0});
        }
    }
    for (std::size_t row = 0U; row < MeasurementSize; ++row)
    {
        for (std::size_t column = 0U; column < MeasurementSize; ++column)
        {
            V(row, column) = value_type{0.25} + (row == column ? value_type{1} : value_type{0});
        }
    }
    auto const wide_P = reference::copied(P);
    auto const wide_H = reference::copied(H);
    auto const wide_V = reference::copied(V);
    auto const expected_cross = reference::product(wide_P, reference::transposed(wide_H));
    auto expected_innovation = reference::product(wide_H, expected_cross);
    for (std::size_t row = 0U; row < MeasurementSize; ++row)
    {
        for (std::size_t column = 0U; column < MeasurementSize; ++column)
        {
            expected_innovation[row][column] += wide_V[row][column];
        }
    }
    auto const cross = eskf::detail::correction_cross<Observation>(P, H);
    auto const innovation = eskf::detail::correction_innovation<Observation>(cross, H, V);
    compare_reference(test, profile, cross, expected_cross, true, "selected model computes exact full P H transpose");
    compare_reference(test, profile, innovation, expected_innovation, true,
                      "selected model computes exact correlated H P H transpose plus V");
    for (bool constrained : {false, true})
    {
        linalg::Matrix<Linalg, 15U, MeasurementSize> K;
        for (std::size_t row = 0U; row < 15U; ++row)
        {
            for (std::size_t column = 0U; column < MeasurementSize; ++column)
            {
                // A fixed dyadic K tests the kernel algebra independently of the
                // solve. The public wrapper below separately tests its solved K.
                bool const suppressed = constrained && (row == 6U || row == 7U || row == 9U || row == 14U);
                K(row, column) =
                    suppressed ? value_type{0} : static_cast<value_type>(1U + (row + column) % 3U) / value_type{16};
            }
        }
        auto const wide_K = reference::copied(K);
        auto const KH = reference::product(wide_K, wide_H);
        reference::Dense<15U, 15U> wide_A{};
        covariance_type A;
        for (std::size_t row = 0U; row < 15U; ++row)
        {
            for (std::size_t column = 0U; column < 15U; ++column)
            {
                wide_A[row][column] = (row == column ? 1 : 0) - KH[row][column];
                A(row, column) = static_cast<value_type>(wide_A[row][column]);
            }
        }
        auto expected = reference::product(reference::product(wide_A, wide_P), reference::transposed(wide_A));
        auto const noise = reference::product(reference::product(wide_K, wide_V), reference::transposed(wide_K));
        for (std::size_t row = 0U; row < 15U; ++row)
        {
            for (std::size_t column = 0U; column < 15U; ++column)
            {
                expected[row][column] += noise[row][column];
            }
        }
        auto const actual = eskf::detail::correction_joseph<Observation>(P, A, K, V);
        compare_reference(
            test, profile, actual, expected, true,
            "selected Joseph kernel uses every full prior/noise entry with the supplied constrained gain");
        test.expect(linalg::is_symmetric(actual, value_type{0}), profile,
                    "selected Joseph output populates both triangles consistently");
    }
}

template <typename Linalg> void test_exact_observation_products(Context & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    linalg::Matrix<Linalg, 2U, 15U> horizontal_position;
    horizontal_position(0U, 0U) = horizontal_position(1U, 1U) = value_type{1};
    check_exact_model_products<Linalg, Model::horizontal_position>(test, profile, horizontal_position);
    linalg::Matrix<Linalg, 1U, 15U> vertical_position;
    vertical_position(0U, 2U) = value_type{1};
    check_exact_model_products<Linalg, Model::vertical_position>(test, profile, vertical_position);
    linalg::Matrix<Linalg, 2U, 15U> horizontal_velocity;
    horizontal_velocity(0U, 3U) = horizontal_velocity(1U, 4U) = value_type{1};
    check_exact_model_products<Linalg, Model::horizontal_velocity>(test, profile, horizontal_velocity);
    linalg::Matrix<Linalg, 3U, 15U> velocity;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        velocity(axis, 3U + axis) = value_type{1};
    }
    check_exact_model_products<Linalg, Model::velocity>(test, profile, velocity);
    linalg::Matrix<Linalg, 3U, 15U> body_vector;
    constexpr value_type skew[3][3]{{0, -4, 2}, {4, 0, -1}, {-2, 1, 0}};
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            body_vector(row, 6U + column) = skew[row][column];
        }
    }
    check_exact_model_products<Linalg, Model::body_vector>(test, profile, body_vector);
}

template <typename L, Model Observation, std::size_t M>
void check_observation(Context & test, std::string_view profile,
                       eskf::configuration::Ins::NominalState<L> const & state, linalg::Matrix<L, 15U, 15U> const & P,
                       linalg::Matrix<L, M, 15U> const & H, linalg::Vector<L, M> const & measurement,
                       linalg::Vector<L, M> const & residual, linalg::Vector<L, 3U> const & reference, bool accel)
{
    using S = typename L::value_type;
    using State = eskf::configuration::Ins::NominalState<L>;
    auto V = linalg::Matrix<L, M, M>::identity();
    if constexpr (M > 1U)
    {
        V(0U, 1U) = V(1U, 0U) = static_cast<S>(0.125);
    }
    eskf::configuration::Ins::BiasUpdate permissions{{false, true, false}, {true, false, true}};
    State actual, expected;
    linalg::Matrix<L, 15U, 15U> covariance, dense;
    constexpr S minimum = static_cast<S>(1e-6);
    constexpr S gate = S{5};
    auto const expected_check = eskf::runtime::check_measurement(state, P, residual, H, V, minimum, gate);
    L::products = 0;
    auto const check = eskf::runtime::check_measurement<L, M, Observation>(state, P, residual, H, V, minimum, gate);
    require(test, profile, (L::products == 0U) == bool(ESKF_CODEGEN), "gate did not select configured products");
    require(test, profile, check.status == expected_check.status && check.rejected == expected_check.rejected,
            "innovation decision differs");
    near(test, profile, check.innovation_variance, expected_check.innovation_variance);
    near(test, profile, check.test_ratio, expected_check.test_ratio);
    if (!require(test, profile,
                 eskf::succeeded(eskf::correct(state, P, residual, H, V, minimum, expected, dense, permissions)),
                 "dense reference correction failed"))
    {
        return;
    }
    L::full_products = 0;
    L::products = 0;
    eskf::Status status{};
    if constexpr (Observation == Model::horizontal_position)
    {
        status = eskf::correct_horizontal_position(state, P, measurement, V, minimum, actual, covariance, permissions);
    }
    else if constexpr (Observation == Model::vertical_position)
    {
        status = eskf::correct_vertical_position(state, P, measurement(0U), V(0U, 0U), minimum, actual, covariance,
                                                 permissions);
    }
    else if constexpr (Observation == Model::horizontal_velocity)
    {
        status = eskf::correct_horizontal_velocity(state, P, measurement, V, minimum, actual, covariance, permissions);
    }
    else if constexpr (Observation == Model::velocity)
    {
        status = eskf::correct_velocity(state, P, measurement, V, minimum, actual, covariance, permissions);
    }
    else
    {
        if (accel)
        {
            status = eskf::correct_accelerometer(state, P, measurement, reference, V, minimum, actual, covariance,
                                                 permissions);
        }
        else
        {
            status = eskf::correct_magnetometer(state, P, measurement, reference, V, minimum, actual, covariance,
                                                permissions);
        }
    }
    auto const correction_products = L::products;
    if (!require(test, profile, eskf::succeeded(status), "built-in correction failed"))
    {
        return;
    }
    require(test, profile, (L::full_products == 0U) == bool(ESKF_CODEGEN),
            "built-in did not select configured products");
    CorrectionReference<L, eskf::configuration::Ins, M> const input{state, P, H, residual, V};
    bool const projected = Observation == Model::body_vector && !accel && ESKF_MAG_TILT;
    auto const independent = reference::reference_correction(input, ESKF_RESET_APPROX != 0, permissions, projected);
    require(test, profile,
            reference::matches_reference(input, independent, actual, covariance,
                                         S{512} * std::numeric_limits<S>::epsilon()),
            "built-in complete correction agrees with independent solved-gain/Joseph/injection/reset model");
    // Compare numerics only when the generic reference has the same gain model.
    if (Observation != Model::body_vector || accel || !ESKF_MAG_TILT)
    {
        near_state(test, profile, actual, expected);
        near(test, profile, covariance, dense);
    }
    // Runtime gate/correction must select the same model, including recovery wrappers.
    if constexpr (Observation != Model::body_vector)
    {
        L::full_products = 0;
        L::products = 0;
        auto const fusion = [&]()
        {
            if constexpr (Observation == Model::horizontal_position)
            {
                return eskf::runtime::fuse_horizontal_position(state, P, measurement, V, minimum, gate, actual,
                                                               covariance, permissions);
            }
            else if constexpr (Observation == Model::vertical_position)
            {
                return eskf::runtime::fuse_vertical_position(state, P, measurement(0U), V(0U, 0U), minimum, gate,
                                                             actual, covariance, permissions);
            }
            else if constexpr (Observation == Model::horizontal_velocity)
            {
                return eskf::runtime::fuse_horizontal_velocity(state, P, measurement, V, minimum, gate, actual,
                                                               covariance, permissions);
            }
            else
            {
                return eskf::runtime::fuse_velocity(state, P, measurement, V, minimum, gate, actual, covariance,
                                                    permissions);
            }
        }();
        require(test, profile, L::products == correction_products + (ESKF_CODEGEN ? 0U : 2U),
                "runtime gate and correction use inconsistent arithmetic");
        if (!require(test, profile, fusion.decision == eskf::runtime::FusionDecision::fused, "runtime fusion failed"))
        {
            return;
        }
        require(test, profile, (L::full_products == 0U) == bool(ESKF_CODEGEN),
                "runtime did not select configured products");
        near_state(test, profile, actual, expected);
        near(test, profile, covariance, dense);
    }
}

template <typename S> void test_observation_dispatch(Context & test, std::string_view profile)
{
    using L = CountingBackend<S>;
    using Vector = linalg::Vector<L, 3U>;
    eskf::configuration::Ins::NominalState<L> state;
    auto const P = covariance_prior<L>(test, profile);
    test_exact_observation_products<L>(test, profile);
    constexpr S minimum = static_cast<S>(1e-6);
    for (unsigned seed = 0U; seed < 16U; ++seed)
    {
        S const t = static_cast<S>(0.01) * static_cast<S>(seed + 1U);
        Vector const rotation = Vector::from_row_major({t, -t, t});
        if (!require(test, profile, eskf::succeeded(eskf::so3::exp(rotation, minimum, state.q_nb)), "attitude setup"))
        {
            return;
        }
        Vector const r = Vector::from_row_major({t, -t, t});
        auto const r2 = r.template segment<0U, 2U>();
        auto const r1 = linalg::Vector<L, 1U>::from_row_major({t});
        check_observation<L, Model::horizontal_position>(
            test, profile, state, P, eskf::horizontal_position_jacobian<L>(), r2, r2, Vector{}, false);
        check_observation<L, Model::vertical_position>(test, profile, state, P, eskf::vertical_position_jacobian<L>(),
                                                       r1, r1, Vector{}, false);
        check_observation<L, Model::horizontal_velocity>(
            test, profile, state, P, eskf::horizontal_velocity_jacobian<L>(), r2, r2, Vector{}, false);
        check_observation<L, Model::velocity>(test, profile, state, P, eskf::velocity_jacobian<L>(), r, r, Vector{},
                                              false);
        for (bool accel : {false, true})
        {
            Vector const reference = Vector::from_row_major({static_cast<S>(0.3), static_cast<S>(0.2), S{1}});
            Vector h = eskf::so3::inverse_rotate(state.q_nb, reference);
            if (accel)
            {
                h = -h;
            }
            linalg::Matrix<L, 3U, 15U> H;
            H.template set_block<0U, 6U>(eskf::so3::hat<L>(h));
            check_observation<L, Model::body_vector>(test, profile, state, P, H, h + r, r, reference, accel);
        }
    }
}

template <typename Matrix, typename Vector>
auto independent_reset(Matrix const & P, Vector const & rotation, std::size_t offset)
{
    constexpr std::size_t size = Matrix::row_count;
    long double const x = rotation(0U);
    long double const y = rotation(1U);
    long double const z = rotation(2U);
    reference::Dense<3U, 3U> negative_skew{{{0, z, -y}, {-z, 0, x}, {y, -x, 0}}};
    reference::Dense<3U, 3U> power{};
    reference::Dense<3U, 3U> J{};
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        power[axis][axis] = J[axis][axis] = 1;
    }
    long double divisor = 1;
    // Independent integral-of-rotation series: J_r(a) = sum (-hat(a))^k/(k+1)!.
    // The bounded angles here are below 0.5 rad; 32 terms make truncation much
    // smaller than binary64 rounding. This is not a universal numerical bound.
    for (std::size_t order = 1U; order <= (ESKF_RESET_APPROX ? 1U : 32U); ++order)
    {
        power = reference::product(power, negative_skew);
        divisor *= static_cast<long double>(order + 1U);
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            for (std::size_t column = 0U; column < 3U; ++column)
            {
                J[row][column] += power[row][column] / divisor;
            }
        }
    }
    reference::Dense<size, size> G{};
    for (std::size_t axis = 0U; axis < size; ++axis)
    {
        G[axis][axis] = 1;
    }
    for (std::size_t row = 0U; row < 3U; ++row)
    {
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            G[offset + row][offset + column] = J[row][column];
        }
    }
    return reference::product(reference::product(G, reference::copied(P)), reference::transposed(G));
}

template <typename S, typename Configuration> void test_reset_dispatch(Context & test, std::string_view profile)
{
    using L = CountingBackend<S>;
    using types = eskf::EskfTypes<L, Configuration>;
    using covariance_type = typename types::error_covariance_type;
    constexpr bool ins = std::is_same_v<Configuration, eskf::configuration::Ins>;
    constexpr std::size_t offset = ins ? 6U : 0U;
    constexpr std::size_t size = types::error_state_dimension;
    covariance_type P;
    for (std::size_t row = 0U; row < types::error_state_dimension; ++row)
    {
        for (std::size_t column = 0U; column < types::error_state_dimension; ++column)
        {
            P(row, column) = S{0.0625} + (row == column ? S{1} : S{0});
        }
    }
    auto coupled_P = P;
    if constexpr (ins)
    {
        // Retain the original nonuniform full-covariance reset regression.
        coupled_P = covariance_prior<L>(test, profile);
    }
    else
    {
        for (std::size_t row = 0U; row < size; ++row)
        {
            for (std::size_t column = 0U; column < size; ++column)
            {
                coupled_P(row, column) =
                    static_cast<S>((row + 1U) * (column + 1U)) / S{64} + (row == column ? S{1} : S{0});
            }
        }
    }
    for (unsigned seed = 0U; seed < 17U; ++seed)
    {
        auto const & prior = seed == 0U ? P : coupled_P;
        S const t = seed == 0U ? S{0} : static_cast<S>(0.01) * static_cast<S>(seed);
        typename types::error_state_type error;
        error.delta_theta_b = linalg::Vector<L, 3U>::from_row_major({t, -t, t});
        covariance_type actual;
        L::full_products = 0U;
        L::products = 0U;
        if (!require(test, profile, eskf::succeeded(eskf::reset_covariance(error, prior, actual)), "reset failed"))
        {
            return;
        }
        if constexpr (ins)
        {
            require(test, profile, (L::full_products == 0U) == bool(ESKF_CODEGEN), "INS reset selection failed");
        }
        else
        {
            require(test, profile, L::products > 0U, "AHRS reset must retain dense backend arithmetic");
        }
        auto const expected = independent_reset(prior, error.delta_theta_b, offset);
        compare_reference(test, profile, actual, expected, seed == 0U,
                          "reset transport agrees with independent integral-series G P G transpose");
        require(test, profile, linalg::is_symmetric(actual, S{0}), "reset publishes both symmetric triangles");
    }
    // Dyadic nonzero first-order reset supplies an exact full-transport control.
    typename types::error_state_type error;
    error.delta_theta_b = linalg::Vector<L, 3U>::from_row_major({S{0.25}, S{-0.125}, S{0.0625}});
    covariance_type actual;
    require(test, profile, eskf::reset_covariance(error, P, actual) == eskf::Status::success,
            "dyadic nonzero reset succeeds");
    compare_reference(test, profile, actual, independent_reset(P, error.delta_theta_b, offset), ESKF_RESET_APPROX != 0,
                      "nonzero reset uses the independently stated G in every attitude cross block");
}

template <typename S, typename Configuration> void test_prediction_dispatch(Context & test, std::string_view profile)
{
    using L = CountingBackend<S>;
    using types = eskf::EskfTypes<L, Configuration>;
    constexpr bool ins = std::is_same_v<Configuration, eskf::configuration::Ins>;
    constexpr std::size_t size = types::error_state_dimension;
    constexpr std::size_t offset = ins ? 6U : 0U;
    typename types::nominal_state_type state;
    typename types::error_covariance_type P;
    typename types::parameter_type parameters;
    parameters.minimum_quaternion_norm = static_cast<S>(1e-6);
    parameters.dt_min = parameters.dt_max = S{0.25};
    parameters.quaternion_squared_norm_tolerance = S{0.125};
    eskf::ImuSample<L> imu;
    if constexpr (ins)
    {
        require(test, profile,
                eskf::so3::UnitQuaternion<L>::from_coefficients(S{0.5}, S{0.5}, S{0.5}, S{0.5},
                                                                parameters.minimum_quaternion_norm,
                                                                state.q_nb) == eskf::Status::success,
                "cyclic attitude setup");
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            state.b_a(axis) = static_cast<S>(axis + 1U) / S{8};
            state.b_g(axis) = -static_cast<S>(axis + 1U) / S{16};
            parameters.process_noise.specific_force_variance(axis) = static_cast<S>(axis + 1U) / S{4};
            parameters.process_noise.accelerometer_bias_random_walk_variance_density(axis) =
                static_cast<S>(axis + 1U) / S{64};
            parameters.process_noise.gyroscope_bias_random_walk_variance_density(axis) =
                static_cast<S>(axis + 1U) / S{256};
        }
        imu.specific_force_b = state.b_a + linalg::Vector<L, 3U>::from_row_major({S{1}, S{2}, S{-4}});
    }
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        parameters.process_noise.angular_rate_variance(axis) = static_cast<S>(axis + 1U) / S{16};
    }
    for (std::size_t row = 0U; row < size; ++row)
    {
        for (std::size_t column = 0U; column < size; ++column)
        {
            P(row, column) = S{0.0625} + (row == column ? S{1} : S{0});
        }
    }
    for (bool zero_rate : {true, false})
    {
        auto const rate =
            zero_rate ? linalg::Vector<L, 3U>{} : linalg::Vector<L, 3U>::from_row_major({S{0.125}, S{-0.25}, S{0.375}});
        imu.angular_rate_b = rate;
        if constexpr (ins)
        {
            imu.angular_rate_b = imu.angular_rate_b + state.b_g;
        }
        reference::Dense<size, size> F{};
        for (std::size_t axis = 0U; axis < size; ++axis)
        {
            F[axis][axis] = 1;
        }
        constexpr long double dt = 0.25L;
        if constexpr (ins)
        {
            constexpr long double R[3][3]{{0, 0, 1}, {1, 0, 0}, {0, 1, 0}};
            constexpr long double force_hat[3][3]{{0, 4, 2}, {-4, 0, -1}, {-2, 1, 0}};
            for (std::size_t row = 0U; row < 3U; ++row)
            {
                F[row][3U + row] = dt;
                F[6U + row][12U + row] = -dt;
                for (std::size_t column = 0U; column < 3U; ++column)
                {
                    F[3U + row][9U + column] = -R[row][column] * dt;
                    for (std::size_t k = 0U; k < 3U; ++k)
                    {
                        F[3U + row][6U + column] -= R[row][k] * force_hat[k][column] * dt;
                    }
                }
            }
        }
        long double const x = rate(0U) * dt;
        long double const y = rate(1U) * dt;
        long double const z = rate(2U) * dt;
        reference::Dense<3U, 3U> negative_skew{{{0, z, -y}, {-z, 0, x}, {y, -x, 0}}};
        reference::Dense<3U, 3U> power{};
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            power[axis][axis] = 1;
        }
        long double divisor = 1;
        for (std::size_t order = 1U; order <= (ESKF_QUAT_APPROX ? 1U : 32U); ++order)
        {
            power = reference::product(power, negative_skew);
            divisor *= static_cast<long double>(order);
            for (std::size_t row = 0U; row < 3U; ++row)
            {
                for (std::size_t column = 0U; column < 3U; ++column)
                {
                    F[offset + row][offset + column] += power[row][column] / divisor;
                }
            }
        }
        auto expected = reference::product(reference::product(F, reference::copied(P)), reference::transposed(F));
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            expected[offset + axis][offset + axis] += static_cast<long double>(axis + 1U) / 16 * dt * dt;
            if constexpr (ins)
            {
                constexpr long double rotated_force_variance[3]{0.75L, 0.25L, 0.5L};
                expected[3U + axis][3U + axis] += rotated_force_variance[axis] * dt * dt;
                expected[9U + axis][9U + axis] += static_cast<long double>(axis + 1U) / 64 * dt;
                expected[12U + axis][12U + axis] += static_cast<long double>(axis + 1U) / 256 * dt;
            }
        }
        typename types::error_covariance_type actual;
        L::full_products = 0U;
        L::products = 0U;
        eskf::Status status;
        if constexpr (ins)
        {
            status = eskf::predict_covariance(state, P, imu, S{0.25}, parameters, actual);
            require(test, profile, (L::full_products == 0U) == bool(ESKF_CODEGEN),
                    "INS prediction selects the configured covariance arithmetic");
        }
        else
        {
            status = eskf::predict_covariance(P, imu, S{0.25}, parameters, actual);
            require(test, profile, L::products > 0U, "AHRS prediction retains dense backend arithmetic");
        }
        require(test, profile, status == eskf::Status::success, "prediction succeeds on common bounded domain");
        // Zero-rate controls are exact in both modes. The nonzero first-order
        // fixture is dyadic as well; finite-angle Exp retains scoped acceptance.
        compare_reference(test, profile, actual, expected, zero_rate || ESKF_QUAT_APPROX,
                          "selected prediction computes independent full F P F transpose and anisotropic noise");
        require(test, profile, linalg::is_symmetric(actual, S{0}), "prediction publishes both triangles");
    }
}

template <typename S, typename Configuration, std::size_t MeasurementSize>
void test_generic_shape(Context & test, std::string_view profile)
{
    using L = CountingBackend<S>;
    using fixture_type = CorrectionReference<L, Configuration, MeasurementSize>;
    fixture_type input;
    for (std::size_t row = 0U; row < fixture_type::size; ++row)
    {
        for (std::size_t column = 0U; column < fixture_type::size; ++column)
        {
            input.P(row, column) = static_cast<S>((row + 1U) * (column + 1U)) / S{1024} + (row == column ? S{1} : S{0});
        }
    }
    for (std::size_t row = 0U; row < MeasurementSize; ++row)
    {
        // Deliberately violates every specialized selector/body-vector model's
        // premise, including for the otherwise matching 1/2/3 row dimensions.
        input.H(row, row % fixture_type::size) = S{1};
        input.H(row, fixture_type::size - 1U) += S{0.25};
        input.r(row) = static_cast<S>(row + 1U) / S{16};
        for (std::size_t column = 0U; column < MeasurementSize; ++column)
        {
            input.V(row, column) = S{0.25} + (row == column ? S{1} : S{0});
        }
    }
    linalg::Matrix<L, fixture_type::size, MeasurementSize> C;
    linalg::Matrix<L, MeasurementSize, MeasurementSize> innovation;
    require(test, profile,
            eskf::correction_covariances(input.P, input.r, input.H, input.V, static_cast<S>(1e-6), C, innovation) ==
                eskf::Status::success,
            "generic arbitrary-H preparation succeeds for matching and unmatched dimensions");
    auto const wide_H = reference::copied(input.H);
    auto const expected_C = reference::product(reference::copied(input.P), reference::transposed(wide_H));
    auto expected_S = reference::product(wide_H, expected_C);
    for (std::size_t row = 0U; row < MeasurementSize; ++row)
    {
        for (std::size_t column = 0U; column < MeasurementSize; ++column)
        {
            expected_S[row][column] += input.V(row, column);
        }
    }
    compare_reference(test, profile, C, expected_C, true, "generic cross covariance honors every arbitrary H entry");
    compare_reference(test, profile, innovation, expected_S, true,
                      "generic innovation covariance honors arbitrary H rather than a dimension-selected model");
    typename fixture_type::state_type corrected;
    typename fixture_type::covariance_type output;
    L::full_products = 0U;
    L::products = 0U;
    require(test, profile,
            eskf::correct(input.state, input.P, input.r, input.H, input.V, static_cast<S>(1e-6), corrected, output) ==
                eskf::Status::success,
            "arbitrary H correction succeeds");
    if constexpr (fixture_type::is_ins)
    {
        require(test, profile, L::full_products >= 2U, "arbitrary H incorrectly selected a generated model");
    }
    else
    {
        require(test, profile, L::products > 0U, "AHRS generic correction no longer uses the dense backend");
    }
    auto const expected = reference::reference_correction(input, ESKF_RESET_APPROX != 0);
    require(
        test, profile,
        reference::matches_reference(input, expected, corrected, output, S{512} * std::numeric_limits<S>::epsilon()),
        "arbitrary-H transaction agrees with independent dense scalar correction");
}

template <typename S, typename Configuration> void test_generic_dispatch(Context & test, std::string_view profile)
{
    test_generic_shape<S, Configuration, 1U>(test, profile);
    test_generic_shape<S, Configuration, 2U>(test, profile);
    test_generic_shape<S, Configuration, 3U>(test, profile);
    test_generic_shape<S, Configuration, 4U>(test, profile);
}

template <typename S> void test_subnormal_publication(Context & test, std::string_view profile)
{
    using L = CountingBackend<S>;
    // A mirrored generated output needs no second average: halving denorm_min
    // would round both copies to zero. Dense mode retains its original cleanup.
    auto tiny = linalg::Matrix<L, 15U, 15U>::identity();
    tiny(0U, 1U) = tiny(1U, 0U) = std::numeric_limits<S>::denorm_min();
    eskf::configuration::Ins::ErrorState<L> zero_error;
    linalg::Matrix<L, 15U, 15U> output;
    if (!require(test, profile, eskf::succeeded(eskf::reset_covariance(zero_error, tiny, output)),
                 "subnormal reset failed"))
    {
        return;
    }
    S const expected_tiny = ESKF_CODEGEN ? std::numeric_limits<S>::denorm_min() : S{0};
    require(test, profile, output(0U, 1U) == expected_tiny && output(1U, 0U) == expected_tiny,
            "covariance publication added an unexpected rounding step");
}

template <typename S> void test_ins_reset(Context & test, std::string_view profile)
{
    test_reset_dispatch<S, eskf::configuration::Ins>(test, profile);
    test_subnormal_publication<S>(test, profile);
}

template <typename S> void test_ins_models(Context & test, std::string_view profile)
{
    test_observation_dispatch<S>(test, profile);
    test_generic_dispatch<S, eskf::configuration::Ins>(test, profile);
}

} // namespace

int main(int argc, char ** argv)
{
    eskf::test::configure_backend_test();
    eskf::test::Case const cases[] = {
        {"TC-E-PCOV-generated-scope", "INS.binary32", test_prediction_dispatch<float, eskf::configuration::Ins>},
        {"TC-E-PCOV-generated-scope", "INS.binary64", test_prediction_dispatch<double, eskf::configuration::Ins>},
        {"TC-E-PCOV-generated-scope", "AHRS.binary32", test_prediction_dispatch<float, eskf::configuration::Ahrs>},
        {"TC-E-PCOV-generated-scope", "AHRS.binary64", test_prediction_dispatch<double, eskf::configuration::Ahrs>},
        {"TC-E-RESET-arithmetic-selection", "INS.binary32", test_ins_reset<float>},
        {"TC-E-RESET-arithmetic-selection", "INS.binary64", test_ins_reset<double>},
        {"TC-E-RESET-arithmetic-selection", "AHRS.binary32", test_reset_dispatch<float, eskf::configuration::Ahrs>},
        {"TC-E-RESET-arithmetic-selection", "AHRS.binary64", test_reset_dispatch<double, eskf::configuration::Ahrs>},
        {"TC-E-CORRECT-model-dispatch", "INS.binary32", test_ins_models<float>},
        {"TC-E-CORRECT-model-dispatch", "INS.binary64", test_ins_models<double>},
        {"TC-E-CORRECT-model-dispatch", "AHRS.binary32", test_generic_dispatch<float, eskf::configuration::Ahrs>},
        {"TC-E-CORRECT-model-dispatch", "AHRS.binary64", test_generic_dispatch<double, eskf::configuration::Ahrs>},
    };
    return eskf::test::run_cases(cases, argc, argv);
}
