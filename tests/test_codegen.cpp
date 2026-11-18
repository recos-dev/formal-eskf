/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "test_backend.hpp"
#include <formal_eskf/eskf/eskf.hpp>
#include <formal_eskf/runtime/navigation.hpp>
#include <formal_eskf/runtime/magnetometer.hpp>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace
{
namespace linalg = formal_eskf::linalg;
namespace eskf = formal_eskf;
using Model = eskf::detail::CorrectionModel;

void require(bool condition, char const * message)
{
    if (!condition)
    {
        std::cerr << message << '\n';
        std::abort();
    }
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

template <typename Matrix> void near(Matrix const & a, Matrix const & b)
{
    using S = typename Matrix::value_type;
    for (std::size_t i = 0; i < Matrix::row_count; ++i)
    {
        for (std::size_t j = 0; j < Matrix::column_count; ++j)
        {
            S const tolerance = S{512} * std::numeric_limits<S>::epsilon() * (S{1} + std::abs(b(i, j)));
            require(std::isfinite(a(i, j)) && std::abs(a(i, j) - b(i, j)) <= tolerance,
                    "production codegen/dense numerical difference");
        }
    }
}

template <typename L>
void near_state(eskf::configuration::Ins::NominalState<L> const & a,
                eskf::configuration::Ins::NominalState<L> const & b)
{
    near(a.p_n, b.p_n);
    near(a.v_n, b.v_n);
    near(a.b_a, b.b_a);
    near(a.b_g, b.b_g);
    near(a.q_nb.coefficients(), b.q_nb.coefficients());
}

template <typename L, Model Observation, std::size_t M>
void check_observation(eskf::configuration::Ins::NominalState<L> const & state, linalg::Matrix<L, 15U, 15U> const & P,
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
    require((L::products == 0U) == bool(ESKF_CODEGEN), "gate did not select configured products");
    require(check.status == expected_check.status && check.rejected == expected_check.rejected,
            "innovation decision differs");
    near(check.innovation_variance, expected_check.innovation_variance);
    near(check.test_ratio, expected_check.test_ratio);
    require(eskf::succeeded(eskf::correct(state, P, residual, H, V, minimum, expected, dense, permissions)),
            "dense reference correction failed");
    // Magnetic tilt is deliberately checked against its actual constrained-gain path below.
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
    require(eskf::succeeded(status), "built-in correction failed");
    require((L::full_products == 0U) == bool(ESKF_CODEGEN), "built-in did not select configured products");
    if (Observation != Model::body_vector || accel || !ESKF_MAG_TILT)
    {
        near_state(actual, expected);
        near(covariance, dense);
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
        require(L::products == correction_products + (ESKF_CODEGEN ? 0U : 2U),
                "runtime gate and correction use inconsistent arithmetic");
        require(fusion.decision == eskf::runtime::FusionDecision::fused, "runtime fusion failed");
        require((L::full_products == 0U) == bool(ESKF_CODEGEN), "runtime did not select configured products");
        near_state(actual, expected);
        near(covariance, dense);
    }
}

template <typename S> void run()
{
    using L = CountingBackend<S>;
    using Covariance = linalg::Matrix<L, 15U, 15U>;
    using Vector = linalg::Vector<L, 3U>;
    using State = eskf::configuration::Ins::NominalState<L>;
    constexpr S minimum = static_cast<S>(1e-6);
    State state;
    auto B = Covariance::identity();
    for (std::size_t i = 0U; i < 15U; ++i)
    {
        for (std::size_t j = 0U; j < i; ++j)
        {
            B(i, j) = static_cast<S>(0.01) * static_cast<S>(1U + ((i + j) % 7U));
        }
    }
    auto P = linalg::sandwich(B, Covariance::identity());
    require(eskf::succeeded(eskf::detail::finish_covariance(P, P)), "prior construction");
    for (unsigned seed = 0; seed < 16; ++seed)
    {
        S const t = static_cast<S>(0.01) * static_cast<S>(seed + 1U);
        Vector rotation = Vector::from_row_major({t, -t, t});
        require(eskf::succeeded(eskf::so3::exp(rotation, minimum, state.q_nb)), "attitude setup");
        Vector r = Vector::from_row_major({t, -t, t});
        auto const r2 = r.template segment<0U, 2U>();
        auto const r1 = linalg::Vector<L, 1U>::from_row_major({t});
        check_observation<L, Model::horizontal_position>(state, P, eskf::horizontal_position_jacobian<L>(), r2, r2,
                                                         Vector{}, false);
        check_observation<L, Model::vertical_position>(state, P, eskf::vertical_position_jacobian<L>(), r1, r1,
                                                       Vector{}, false);
        check_observation<L, Model::horizontal_velocity>(state, P, eskf::horizontal_velocity_jacobian<L>(), r2, r2,
                                                         Vector{}, false);
        check_observation<L, Model::velocity>(state, P, eskf::velocity_jacobian<L>(), r, r, Vector{}, false);
        for (bool accel : {false, true})
        {
            Vector reference = Vector::from_row_major({static_cast<S>(0.3), static_cast<S>(0.2), S{1}});
            Vector h = eskf::so3::inverse_rotate(state.q_nb, reference);
            if (accel)
            {
                h = -h;
            }
            linalg::Matrix<L, 3U, 15U> H;
            H.template set_block<0U, 6U>(eskf::so3::hat<L>(h));
            check_observation<L, Model::body_vector>(state, P, H, h + r, r, reference, accel);
        }
        eskf::configuration::Ins::ErrorState<L> error;
        error.delta_theta_b = rotation;
        Covariance actual, dense;
        L::full_products = 0;
        require(eskf::succeeded(eskf::reset_covariance(error, P, actual)), "reset failed");
        require((L::full_products == 0U) == bool(ESKF_CODEGEN), "reset selection failed");
        linalg::Matrix<L, 3U, 3U> J;
#if ESKF_RESET_APPROX
        J = linalg::Matrix<L, 3U, 3U>::identity() - eskf::so3::hat<L>(rotation * static_cast<S>(0.5));
#else
        require(eskf::succeeded(eskf::so3::right_jacobian(rotation, J)), "reset Jacobian failed");
#endif
        auto G = Covariance::identity();
        G.template set_block<6U, 6U>(J);
        dense = linalg::sandwich(G, P);
        require(eskf::succeeded(eskf::detail::finish_covariance(dense, dense)), "dense reset failed");
        near(actual, dense);
    }
    eskf::configuration::Ins::Parameters<L> parameters;
    parameters.minimum_quaternion_norm = minimum;
    parameters.dt_min = static_cast<S>(1e-6);
    parameters.dt_max = static_cast<S>(0.05);
    parameters.quaternion_squared_norm_tolerance = S{64} * std::numeric_limits<S>::epsilon();
    eskf::ImuSample<L> imu;
    imu.specific_force_b = Vector::from_row_major({S{1}, S{2}, S{-9}});
    Covariance output;
    L::full_products = 0;
    require(eskf::succeeded(eskf::predict_covariance(state, P, imu, static_cast<S>(0.004), parameters, output)),
            "prediction failed");
    require((L::full_products == 0U) == bool(ESKF_CODEGEN), "prediction selection failed");
    // Arbitrary H of the same 3x15 shape is still generic, not a body vector.
    auto H = eskf::velocity_jacobian<L>();
    H(0U, 14U) = static_cast<S>(0.3);
    State corrected;
    L::full_products = 0;
    require(eskf::succeeded(eskf::correct(state, P, Vector{}, H, linalg::Matrix<L, 3U, 3U>::identity(), minimum,
                                          corrected, output)),
            "arbitrary H failed");
    require(L::full_products >= 2U, "arbitrary H incorrectly selected a generated model");
    // A mirrored generated output needs no second average: halving denorm_min
    // would round both copies to zero. Dense mode retains its original cleanup.
    auto tiny = Covariance::identity();
    tiny(0U, 1U) = tiny(1U, 0U) = std::numeric_limits<S>::denorm_min();
    eskf::configuration::Ins::ErrorState<L> zero_error;
    require(eskf::succeeded(eskf::reset_covariance(zero_error, tiny, output)), "subnormal reset failed");
    S const expected_tiny = ESKF_CODEGEN ? std::numeric_limits<S>::denorm_min() : S{0};
    require(output(0U, 1U) == expected_tiny && output(1U, 0U) == expected_tiny,
            "covariance publication added an unexpected rounding step");
    eskf::configuration::Ahrs::ErrorState<L> error;
    auto small = linalg::Matrix<L, 3U, 3U>::identity();
    L::products = 0;
    require(eskf::succeeded(eskf::reset_covariance(error, small, small)), "AHRS reset failed");
    require(L::products > 0U, "AHRS dense reset disappeared");
}
} // namespace

int main()
{
    eskf::test::configure_backend_test();
    run<float>();
    run<double>();
    std::cout << "Codegen integration: pass (ESKF_CODEGEN=" << ESKF_CODEGEN << ")\n";
}
