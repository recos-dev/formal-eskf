/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fixture.hpp"
#include <cfenv>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#if defined(__SSE__)
#include <xmmintrin.h>
#else
#error "The native IEEE witness harness requires x86 SSE to check FTZ/DAZ. Use stack.cpp for target compilation."
#endif
namespace codegen_test
{
enum class Operation
{
    covariance,
    prediction,
    correction_cycle
};

static_assert(std::numeric_limits<float>::is_iec559 && std::numeric_limits<float>::digits == 24);
static_assert(std::numeric_limits<double>::is_iec559 && std::numeric_limits<double>::digits == 53);

template <typename Matrix> void write_matrix(char const * name, Matrix const & value)
{
    std::cout << ",\"" << name << "\":[";
    bool first = true;
    for (std::size_t i = 0; i < Matrix::row_count; ++i)
    {
        for (std::size_t j = 0; j < Matrix::column_count; ++j)
        {
            std::cout << (first ? "" : ",") << bits(value(i, j));
            first = false;
        }
    }
    std::cout << ']';
}

void require(bool condition)
{
    if (!condition)
    {
        std::cerr << "native contract assertion failed\n";
        std::abort();
    }
}

template <typename S> void native_checks()
{
    using L = Backend<S>;
    unsigned calls = 0U;
    for (unsigned seed = 0U; seed < 728U; ++seed)
    {
        auto const original = trial_fixture<S>(seed);
        auto covariance = decltype(original.P)::identity() * S{123};
        int const covariance_status = static_cast<int>(formal_eskf::predict_covariance(
            original.state, original.P, original.imu, original.dt, original.parameters, covariance));
        auto state = original.state;
        auto combined = decltype(original.P)::identity() * S{123};
        int const prediction_status = static_cast<int>(formal_eskf::predict(
            original.state, original.P, original.imu, original.dt, original.parameters, state, combined));
        calls += 2U;
        if (prediction_status == 0)
        {
            require(covariance_status == 0 && capture(state, combined) == capture(state, covariance));
            auto nominal = original.state;
            require(static_cast<int>(formal_eskf::predict_nominal(original.state, original.imu, original.dt,
                                                                  original.parameters, nominal)) == 0);
            require(capture(state, combined) == capture(nominal, combined));
        }
        auto const expected_cov = capture(original.state, covariance);
        auto const expected_predict = capture(state, combined);
        for (unsigned alias = 0U; alias < 4U; ++alias)
        {
            auto f = original;
            auto storage_state = original.state;
            auto storage_p = decltype(original.P)::identity() * S{123};
            auto & out_state = alias & 1U ? f.state : storage_state;
            auto & out_p = alias & 2U ? f.P : storage_p;
            auto const before = capture(out_state, out_p);
            auto const before_input = capture(f.state, f.P);
            int const status =
                static_cast<int>(formal_eskf::predict(f.state, f.P, f.imu, f.dt, f.parameters, out_state, out_p));
            auto const after = capture(out_state, out_p);
            auto const after_input = capture(f.state, f.P);
            require(status == prediction_status);
            require(status == 0 ? after == expected_predict : before == after);
            for (unsigned i = 0U; i < 241U; ++i)
            {
                if (i < 16U ? !(alias & 1U) : !(alias & 2U))
                {
                    require(before_input[i] == after_input[i]);
                }
            }
            ++calls;
        }
        for (bool alias : {false, true})
        {
            auto f = original;
            auto storage = decltype(original.P)::identity() * S{123};
            auto & out = alias ? f.P : storage;
            auto const before = capture(f.state, out);
            auto const before_input = capture(f.state, f.P);
            int const status =
                static_cast<int>(formal_eskf::predict_covariance(f.state, f.P, f.imu, f.dt, f.parameters, out));
            require(status == covariance_status);
            require(status == 0 ? capture(f.state, out) == expected_cov : capture(f.state, out) == before);
            if (!alias)
            {
                require(before_input == capture(f.state, f.P));
            }
            ++calls;
        }
        std::cout << "{\"kind\":\"status\",\"bits\":" << sizeof(S) * 8U << ",\"seed\":" << seed
                  << ",\"covariance\":" << covariance_status << ",\"predict\":" << prediction_status << "}\n";
        if (covariance_status != 0)
        {
            continue;
        }
        require(formal_eskf::linalg::all_finite(covariance));
        bool symmetric = true;
        for (unsigned i = 0U; i < 15U; ++i)
        {
            for (unsigned j = 0U; j < 15U; ++j)
            {
                require(bits(covariance(i, j)) == bits(covariance(j, i)));
                symmetric &= original.P(i, j) == original.P(j, i);
            }
        }
        // Certify all finite symmetric results, including extreme and non-PSD diagnostics.
        if (!symmetric)
        {
            continue;
        }
        auto const F = transition(original);
        auto const R = formal_eskf::so3::to_rotation_matrix(original.state.q_nb);
        formal_eskf::linalg::Matrix<L, 12U, 12U> Q_i;
        require(static_cast<int>(
                    formal_eskf::discretize_process_noise(original.parameters.process_noise, original.dt, Q_i)) == 0);
#if GENERATED
        auto const direct = formal_eskf::detail::generated::ins_covariance(F, original.P, R, Q_i);
        require(capture(original.state, direct) == expected_cov);
#endif
        std::cout << "{\"kind\":\"numerical\",\"bits\":" << sizeof(S) * 8U << ",\"seed\":" << seed;
        write_matrix("F", F);
        write_matrix("P", original.P);
        write_matrix("R", R);
        write_matrix("Q_i", Q_i);
        write_matrix("output", covariance);
        std::cout << "}\n";
    }
    std::cout << "{\"kind\":\"contracts\",\"bits\":" << sizeof(S) * 8U << ",\"calls\":" << calls
              << ",\"failures\":0}\n";
}

template <typename S> __attribute__((noinline)) void full_predict(Fixture<S> const & f)
{
    auto state = f.state;
    auto P = f.P;
    require(static_cast<int>(formal_eskf::predict(f.state, f.P, f.imu, f.dt, f.parameters, state, P)) == 0);
    asm volatile("" : "+m"(state), "+m"(P) : : "memory");
}

template <typename S> __attribute__((noinline)) void full_cycle(Fixture<S> const & input)
{
    auto f = input;
    require(static_cast<int>(formal_eskf::predict(f.state, f.P, f.imu, f.dt, f.parameters, f.state, f.P)) == 0);
    require(static_cast<int>(formal_eskf::correct_accelerometer(f.state, f.P, f.imu.specific_force_b, f.gravity, f.V,
                                                                f.parameters.minimum_quaternion_norm, f.state, f.P,
                                                                f.bias)) == 0);
    require(static_cast<int>(formal_eskf::correct_magnetometer(
                f.state, f.P, f.m_b, f.m_n, f.V, f.parameters.minimum_quaternion_norm, f.state, f.P, f.bias)) == 0);
    asm volatile("" : "+m"(f) : : "memory");
}

template <typename S> double timing(Operation op, unsigned count)
{
    std::array<Fixture<S>, 16U> inputs;
    std::array<Matrix<S, 15U, 15U>, 16U> transitions;
    std::array<Matrix<S, 3U, 3U>, 16U> rotations;
    std::array<Matrix<S, 12U, 12U>, 16U> impulses;
    for (unsigned i = 0U; i < 16U; ++i)
    {
        inputs[i] = fixture<S>(i);
        transitions[i] = transition(inputs[i]);
        rotations[i] = formal_eskf::so3::to_rotation_matrix(inputs[i].state.q_nb);
        require(static_cast<int>(formal_eskf::discretize_process_noise(inputs[i].parameters.process_noise, inputs[i].dt,
                                                                       impulses[i])) == 0);
    }
    auto const begin = std::chrono::steady_clock::now();
    for (unsigned i = 0U; i < count; ++i)
    {
        unsigned const j = i % 16U;
        if (op == Operation::covariance)
        {
            auto output = covariance_kernel<S>(transitions[j], inputs[j].P, rotations[j], impulses[j]);
            asm volatile("" : "+m"(output) : : "memory");
        }
        else if (op == Operation::prediction)
        {
            full_predict(inputs[j]);
        }
        else
        {
            full_cycle(inputs[j]);
        }
    }
    return std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - begin).count() / count;
}
} // namespace codegen_test

using codegen_test::native_checks;
using codegen_test::Operation;
using codegen_test::require;
using codegen_test::timing;

int main(int argc, char ** argv)
{
    require(std::fegetround() == FE_TONEAREST);
#ifndef FORMAL_ESKF_TEST_PX4_MATRIX
    Eigen::internal::set_is_malloc_allowed(false);
#endif
#if defined(__SSE__)
    require((_mm_getcsr() & 0x8040U) == 0U);
#endif
    std::cout << std::setprecision(17);
    if (argc == 1)
    {
        native_checks<float>();
        native_checks<double>();
    }
    else
    {
        require(argc == 2 && std::string(argv[1]) == "benchmark");
        for (unsigned scalar : {4U, 8U})
        {
            for (Operation op : {Operation::covariance, Operation::prediction, Operation::correction_cycle})
            {
                for (unsigned sample = 0U; sample < 12U; ++sample)
                {
                    double const ns = scalar == 4U ? timing<float>(op, 4096U) : timing<double>(op, 4096U);
                    if (sample < 2U)
                    {
                        continue;
                    }
                    std::cout << "{\"bytes\":" << scalar << ",\"op\":" << static_cast<unsigned>(op)
                              << ",\"sample\":" << sample - 2U << ",\"ns\":" << ns << "}\n";
                }
            }
        }
    }
}
