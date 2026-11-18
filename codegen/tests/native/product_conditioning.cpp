/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Additional full-caller conditioning diagnostics; generated arithmetic is unchanged.
#define PRODUCT_KERNELS_ONLY
#include "products.cpp"
#include <codegen_fixture.hpp>
#include <algorithm>

using Model = codegen_fixture::Model;
template <typename S> using State = formal_eskf::configuration::Ins::NominalState<Backend<S>>;

template <typename M> double difference(M const & a, M const & b)
{
    if (!formal_eskf::linalg::all_finite(a) || !formal_eskf::linalg::all_finite(b))
    {
        std::abort();
    }
    double error = 0.0, scale = 1.0;
    for (std::size_t i = 0U; i < M::row_count; ++i)
    {
        for (std::size_t j = 0U; j < M::column_count; ++j)
        {
            error = std::max(error, std::abs(static_cast<double>(a(i, j)) - static_cast<double>(b(i, j))));
            scale = std::max(scale,
                             std::max(std::abs(static_cast<double>(a(i, j))), std::abs(static_cast<double>(b(i, j)))));
        }
    }
    return error / scale;
}

template <typename S, std::size_t M> void cases(Model model, unsigned sensor)
{
    codegen_fixture::selected = model;
    for (unsigned seed = 0U; seed < 8U; ++seed)
    {
        State<S> state;
        Matrix<S, 15U, 1U> u;
        for (std::size_t i = 0U; i < 15U; ++i)
        {
            u(i) = static_cast<S>(1U + (i + seed) % 7U) / S{16};
        }
        // Exactly representable dyadic rank-one PSD prior, with nonzero cross terms.
        auto const P = u * formal_eskf::linalg::transpose(u);
        Matrix<S, M, 15U> H;
        Matrix<S, M, 1U> r;
        for (std::size_t i = 0U; i < M; ++i)
        {
            r(i) = S{0.001} * static_cast<S>(i + 1U);
        }
        Matrix<S, 3U, 1U> reference, measurement;
        if constexpr (M == 1U)
        {
            H = formal_eskf::vertical_position_jacobian<Backend<S>>();
        }
        if constexpr (M == 2U)
        {
            H = model == Model::horizontal_position ? formal_eskf::horizontal_position_jacobian<Backend<S>>()
                                                    : formal_eskf::horizontal_velocity_jacobian<Backend<S>>();
        }
        if constexpr (M == 3U)
        {
            if (model == Model::body_vector)
            {
                reference(0U) = sensor == 1U ? S{0.4} : S{0};
                reference(2U) = sensor == 1U ? S{0.2} : S{9.80665};
                auto h = formal_eskf::so3::inverse_rotate(state.q_nb, reference);
                if (sensor == 2U)
                {
                    h = -h;
                }
                H.template set_block<0U, 6U>(formal_eskf::so3::hat<Backend<S>>(h));
                measurement = h + r;
            }
            else
            {
                H = formal_eskf::velocity_jacobian<Backend<S>>();
            }
        }
        auto const Hu = H * u;
        double norm_squared = 0.0;
        for (std::size_t i = 0U; i < M; ++i)
        {
            norm_squared += static_cast<double>(Hu(i)) * static_cast<double>(Hu(i));
        }
        for (unsigned level : {2U, 4U, 6U, 8U, 10U, 12U})
        {
            if (sizeof(S) == 4U && level > 6U)
            {
                continue;
            }
            S const noise = static_cast<S>(std::max(1.0, norm_squared) * std::pow(10.0, -static_cast<double>(level)));
            auto const V = Matrix<S, M, M>::identity() * noise;
            double const condition = M == 1U ? 1.0 : 1.0 + norm_squared / static_cast<double>(noise);
            State<S> output[2];
            Matrix<S, 15U, 15U> corrected[2];
            for (unsigned variant = 0U; variant < 2U; ++variant)
            {
                codegen_fixture::generated = variant == 1U;
                formal_eskf::Status status;
                if constexpr (M == 3U)
                {
                    if (sensor == 1U)
                    {
                        status = formal_eskf::correct_magnetometer(state, P, measurement, reference, V, S{1e-6},
                                                                   output[variant], corrected[variant]);
                    }
                    else if (sensor == 2U)
                    {
                        status = formal_eskf::correct_accelerometer(state, P, measurement, reference, V, S{1e-6},
                                                                    output[variant], corrected[variant]);
                    }
                    else
                    {
                        status = formal_eskf::correct(state, P, r, H, V, S{1e-6}, output[variant], corrected[variant]);
                    }
                }
                else
                {
                    status = formal_eskf::correct(state, P, r, H, V, S{1e-6}, output[variant], corrected[variant]);
                }
                if (!formal_eskf::succeeded(status))
                {
                    std::fprintf(
                        stderr,
                        "conditioning failed: model=%u sensor=%u bits=%zu seed=%u level=%u variant=%u status=%u\n",
                        static_cast<unsigned>(model), sensor, sizeof(S) * 8U, seed, level, variant,
                        static_cast<unsigned>(status));
                    std::abort();
                }
            }
            double error = difference(corrected[0], corrected[1]);
            error = std::max(error, difference(output[0].q_nb.coefficients(), output[1].q_nb.coefficients()));
            error = std::max(error, difference(output[0].p_n, output[1].p_n));
            error = std::max(error, difference(output[0].v_n, output[1].v_n));
            error = std::max(error, difference(output[0].b_a, output[1].b_a));
            error = std::max(error, difference(output[0].b_g, output[1].b_g));
            if (!std::isfinite(error))
            {
                std::abort();
            }
            std::printf("{\"model\":%u,\"sensor\":%u,\"bits\":%zu,\"seed\":%u,\"level\":%u,\"condition_estimate\":%."
                        "17g,\"scaled_difference\":%.17g}\n",
                        static_cast<unsigned>(model), sensor, sizeof(S) * 8U, seed, level, condition, error);
        }
    }
}

template <typename S> void all()
{
    // Raw innovation boundary regression: cancellation in H*C must finish
    // before adding V. This C is not claimed to come from a PSD Kalman prior.
    Matrix<S, 15U, 3U> C;
    Matrix<S, 3U, 1U> h;
    h(1U) = h(2U) = S{1};
    C(7U, 0U) = C(8U, 0U) = std::ldexp(S{1}, sizeof(S) == 4U ? 25 : 54);
    auto const V = Matrix<S, 3U, 3U>::identity();
    auto const innovation = formal_eskf::detail::generated::ins_body_vector_innovation(C, h, V);
    if (!same_bits(innovation, V))
    {
        std::fprintf(stderr, "innovation noise was absorbed before product cancellation\n");
        std::abort();
    }
    cases<S, 2U>(Model::horizontal_position, 0U);
    cases<S, 1U>(Model::vertical_position, 0U);
    cases<S, 2U>(Model::horizontal_velocity, 0U);
    cases<S, 3U>(Model::velocity, 0U);
    cases<S, 3U>(Model::body_vector, 1U);
    cases<S, 3U>(Model::body_vector, 2U);
}
int main()
{
#ifndef USE_PX4
    Eigen::internal::set_is_malloc_allowed(false);
#endif
    if (std::fegetround() != FE_TONEAREST)
    {
        return 1;
    }
#if defined(__SSE__)
    if (_mm_getcsr() & ((1U << 15U) | (1U << 6U)))
    {
        return 1;
    }
#endif
    all<float>();
    all<double>();
}
