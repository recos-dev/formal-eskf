/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Isolated integration experiment; production headers are never patched in place.
#define PRODUCT_KERNELS_ONLY
#include "products.cpp"
#include <codegen_fixture.hpp>
#include <algorithm>

using Model = codegen_fixture::Model;
using formal_eskf::Status;

template <typename S> using State = formal_eskf::configuration::Ins::NominalState<Backend<S>>;

template <typename S> bool same_state(State<S> const & a, State<S> const & b)
{
    return same_bits(a.q_nb.coefficients(), b.q_nb.coefficients()) && same_bits(a.p_n, b.p_n) &&
           same_bits(a.v_n, b.v_n) && same_bits(a.b_a, b.b_a) && same_bits(a.b_g, b.b_g);
}

template <typename M> double scaled_difference(M const & a, M const & b)
{
    double error = 0.0;
    double scale = 1.0;
    for (std::size_t i = 0; i < M::row_count; ++i)
    {
        for (std::size_t j = 0; j < M::column_count; ++j)
        {
            error = std::max(error, std::abs(static_cast<double>(a(i, j)) - static_cast<double>(b(i, j))));
            scale = std::max(scale,
                             std::max(std::abs(static_cast<double>(a(i, j))), std::abs(static_cast<double>(b(i, j)))));
        }
    }
    return error / scale;
}

enum class Case : unsigned
{
    ordinary,
    nonfinite,
    asymmetric,
    invalid_variance,
    invalid_nominal,
    ill_conditioned,
    overflow
};

template <typename S, std::size_t M> void scenario(Model model, unsigned sensor, bool bench)
{
    codegen_fixture::selected = model;
    double max_difference = 0.0;
    unsigned accepted = 0U, rejected = 0U, status_differences = 0U, gate_differences = 0U;
    for (unsigned seed = 0U; seed < (bench ? 8U : 64U); ++seed)
    {
        for (unsigned index = 0U; index < (bench ? 1U : 7U); ++index)
        {
            auto kind = static_cast<Case>(index);
            Random random{seed + 294729U};
            State<S> state;
            Matrix<S, 3U, 1U> angle;
            fill(angle, random);
            if (!formal_eskf::succeeded(formal_eskf::so3::exp(angle, S{1e-6}, state.q_nb)))
            {
                std::abort();
            }
            fill(state.p_n, random);
            fill(state.v_n, random);
            fill(state.b_a, random);
            fill(state.b_g, random);
            Matrix<S, 15U, 15U> P;
            Matrix<S, M, M> V;
            positive(P, random);
            positive(V, random);
            Matrix<S, M, 1U> r;
            fill(r, random);
            r = r * S{0.01};
            Matrix<S, M, 15U> H;
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
                    if (sensor == 2U)
                    {
                        measurement = measurement + state.b_a;
                    }
                }
                else
                {
                    H = formal_eskf::velocity_jacobian<Backend<S>>();
                }
            }
            if (kind == Case::nonfinite)
            {
                P(0U, 0U) = std::numeric_limits<S>::quiet_NaN();
            }
            if (kind == Case::asymmetric)
            {
                P(0U, 1U) += S{1};
            }
            if (kind == Case::invalid_variance)
            {
                V(0U, 0U) = S{-1};
            }
            if (kind == Case::invalid_nominal)
            {
                state.p_n(0U) = std::numeric_limits<S>::quiet_NaN();
            }
            if (kind == Case::ill_conditioned)
            {
                P = Matrix<S, 15U, 15U>{};
                V = Matrix<S, M, M>::identity();
                V(0U, 0U) = std::numeric_limits<S>::epsilon() * S{0.01};
            }
            if (kind == Case::overflow)
            {
                P = P * (std::numeric_limits<S>::max() / S{8});
            }
            formal_eskf::configuration::Ins::BiasUpdate permission;
            for (unsigned i = 0; i < 3; ++i)
            {
                permission.accelerometer[i] = (seed & (1U << i)) != 0U;
                permission.gyroscope[i] = (seed & (1U << (i + 3U))) != 0U;
            }
            auto invoke = [&](State<S> const & prior, Matrix<S, 15U, 15U> const & covariance, State<S> & output,
                              Matrix<S, 15U, 15U> & corrected)
            {
                if constexpr (M == 3U)
                {
                    if (sensor == 1U)
                    {
                        return formal_eskf::correct_magnetometer(prior, covariance, measurement, reference, V, S{1e-6},
                                                                 output, corrected, permission);
                    }
                    if (sensor == 2U)
                    {
                        return formal_eskf::correct_accelerometer(prior, covariance, measurement, reference, V, S{1e-6},
                                                                  output, corrected, permission);
                    }
                }
                return formal_eskf::correct(prior, covariance, r, H, V, S{1e-6}, output, corrected, permission);
            };
            if (bench)
            {
                State<S> output;
                Matrix<S, 15U, 15U> corrected;
                for (unsigned round = 0U; round < 6U; ++round)
                {
                    for (unsigned order = 0U; order < 2U; ++order)
                    {
                        unsigned variant = (round + order) % 2U;
                        codegen_fixture::generated = variant == 1U;
                        auto start = std::chrono::steady_clock::now();
                        for (unsigned count = 0U; count < 1024U; ++count)
                        {
                            asm volatile("" ::: "memory");
                            if (!formal_eskf::succeeded(invoke(state, P, output, corrected)))
                            {
                                std::abort();
                            }
                            checksum = static_cast<double>(corrected(0U, 0U));
                        }
                        double ns =
                            std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - start).count() /
                            1024U;
                        std::printf("{\"model\":%u,\"sensor\":%u,\"bits\":%zu,\"seed\":%u,\"round\":%u,\"variant\":%u,"
                                    "\"ns\":%.9g}\n",
                                    static_cast<unsigned>(model), sensor, sizeof(S) * 8U, seed, round, variant, ns);
                    }
                }
                continue;
            }
            State<S> output[2];
            Matrix<S, 15U, 15U> corrected[2];
            Status status[2];
            bool gates[2][3]{};
            for (unsigned variant = 0U; variant < 2U; ++variant)
            {
                codegen_fixture::generated = variant == 1U;
                output[variant] = state;
                corrected[variant] = P;
                auto initial_state = output[variant];
                auto initial_P = corrected[variant];
                status[variant] = invoke(state, P, output[variant], corrected[variant]);
                if (!formal_eskf::succeeded(status[variant]))
                {
                    if (!same_state(output[variant], initial_state) || !same_bits(corrected[variant], initial_P))
                    {
                        std::abort();
                    }
                }
                // Exercise every independent state/covariance alias combination.
                for (unsigned aliases = 1U; aliases < 4U; ++aliases)
                {
                    auto alias_state = state;
                    auto alias_P = P;
                    auto other_state = state;
                    auto other_P = P;
                    auto & state_target = (aliases & 1U) ? alias_state : other_state;
                    auto & P_target = (aliases & 2U) ? alias_P : other_P;
                    auto alias_status = invoke(alias_state, alias_P, state_target, P_target);
                    if (alias_status != status[variant] || !same_state(state_target, output[variant]) ||
                        !same_bits(P_target, corrected[variant]))
                    {
                        std::abort();
                    }
                }
                if (kind == Case::ordinary)
                {
                    Matrix<S, 15U, M> C;
                    Matrix<S, M, M> innovation;
                    if (!formal_eskf::succeeded(
                            formal_eskf::correction_covariances(P, r, H, V, S{1e-6}, C, innovation)))
                    {
                        std::abort();
                    }
                    for (unsigned edge = 0; edge < 3; ++edge)
                    {
                        Matrix<S, M, 1U> residual, diagonal;
                        for (std::size_t i = 0; i < M; ++i)
                        {
                            diagonal(i) = innovation(i, i);
                        }
                        // Same physical residual is chosen from the dense S below, outside this loop.
                        codegen_fixture::generated = false;
                        Matrix<S, 15U, M> dense_C;
                        Matrix<S, M, M> dense_S;
                        if (!formal_eskf::succeeded(
                                formal_eskf::correction_covariances(P, r, H, V, S{1e-6}, dense_C, dense_S)))
                        {
                            std::abort();
                        }
                        codegen_fixture::generated = variant == 1U;
                        residual(0U) = S{3} * std::sqrt(dense_S(0U, 0U));
                        if (edge != 1U)
                        {
                            residual(0U) =
                                std::nextafter(residual(0U), edge == 0U ? S{0} : std::numeric_limits<S>::infinity());
                        }
                        formal_eskf::InnovationCheck<Backend<S>, M> diagnostic;
                        if (!formal_eskf::succeeded(
                                formal_eskf::check_innovation(residual, diagonal, S{3}, diagnostic)))
                        {
                            std::abort();
                        }
                        gates[variant][edge] = diagnostic.rejected;
                    }
                }
            }
            if (status[0] != status[1])
            {
                ++status_differences;
                if (kind != Case::overflow)
                {
                    std::abort();
                }
            }
            if (formal_eskf::succeeded(status[0]) && formal_eskf::succeeded(status[1]))
            {
                ++accepted;
                double difference = scaled_difference(corrected[0], corrected[1]);
                difference = std::max(difference,
                                      scaled_difference(output[0].q_nb.coefficients(), output[1].q_nb.coefficients()));
                difference = std::max(difference, scaled_difference(output[0].p_n, output[1].p_n));
                difference = std::max(difference, scaled_difference(output[0].v_n, output[1].v_n));
                difference = std::max(difference, scaled_difference(output[0].b_a, output[1].b_a));
                difference = std::max(difference, scaled_difference(output[0].b_g, output[1].b_g));
                max_difference = std::max(max_difference, difference);
                // Native regression threshold, not a mathematical uniform error theorem.
                if (kind != Case::overflow && difference > 256.0 * std::numeric_limits<S>::epsilon())
                {
                    std::abort();
                }
            }
            else
            {
                ++rejected;
            }
            for (unsigned edge = 0; edge < 3; ++edge)
            {
                gate_differences += gates[0][edge] != gates[1][edge];
            }
        }
    }
    if (!bench)
    {
        std::printf("{\"model\":%u,\"sensor\":%u,\"bits\":%zu,\"accepted\":%u,\"rejected\":%u,"
                    "\"status_differences\":%u,\"gate_boundary_differences\":%u,\"max_scaled_difference\":%.17g}\n",
                    static_cast<unsigned>(model), sensor, sizeof(S) * 8U, accepted, rejected, status_differences,
                    gate_differences, max_difference);
    }
}

template <typename S> void all_scenarios(bool bench)
{
    scenario<S, 2U>(Model::horizontal_position, 0U, bench);
    scenario<S, 1U>(Model::vertical_position, 0U, bench);
    scenario<S, 2U>(Model::horizontal_velocity, 0U, bench);
    scenario<S, 3U>(Model::velocity, 0U, bench);
    scenario<S, 3U>(Model::body_vector, 1U, bench);
    scenario<S, 3U>(Model::body_vector, 2U, bench);
}

int main(int argc, char ** argv)
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
    bool bench = argc == 2 && std::strcmp(argv[1], "benchmark") == 0;
    all_scenarios<float>(bench);
    all_scenarios<double>(bench);
    for (auto count : codegen_fixture::calls)
    {
        if (count == 0U)
        {
            return 2;
        }
    }
}
