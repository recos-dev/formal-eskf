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
#include <iostream>
#include <limits>

#include <formal_eskf/eskf/velocity.hpp>
#include <formal_eskf/runtime/bias_limits.hpp>
#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{

using formal_eskf::Status;
using TestContext = formal_eskf::test::Context;
constexpr std::string_view profile = "bias limits";

template <typename Matrix> bool same_bits(Matrix const & a, Matrix const & b)
{
    using bytes_type = std::array<std::byte, sizeof(typename Matrix::value_type)>;
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            if (std::bit_cast<bytes_type>(a(row, column)) != std::bit_cast<bytes_type>(b(row, column)))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename State> bool same_navigation(State const & a, State const & b)
{
    return same_bits(a.p_n, b.p_n) && same_bits(a.v_n, b.v_n) &&
           same_bits(a.q_nb.coefficients(), b.q_nb.coefficients());
}

template <typename Linalg> void test_bounds(TestContext & test)
{
    using Scalar = typename Linalg::value_type;
    formal_eskf::configuration::Ins::NominalState<Linalg> state;
    state.p_n.set(0U, Scalar{3});
    state.v_n.set(2U, Scalar{-2});
    auto const original = state;
    formal_eskf::runtime::BiasLimits<Scalar> const limits{Scalar{0.5}, Scalar{0.125}};
    for (auto sign : {Scalar{-1}, Scalar{1}})
    {
        // Exact bound, one ULP inside and one ULP outside on different axes.
        for (auto const group : {0U, 1U})
        {
            auto & bias = group == 0U ? state.b_a : state.b_g;
            auto const bound = sign * (group == 0U ? limits.accelerometer : limits.gyroscope);
            bias.set(0U, bound);
            bias.set(1U, std::nextafter(bound, Scalar{0}));
            bias.set(2U, std::nextafter(bound, sign * std::numeric_limits<Scalar>::infinity()));
        }
        auto const inside_a = state.b_a(1U);
        auto const inside_g = state.b_g(1U);
        test.expect(formal_eskf::runtime::try_limit_bias<Linalg>(state, limits) == Status::success &&
                        state.b_a(0U) == sign * limits.accelerometer && state.b_a(1U) == inside_a &&
                        state.b_a(2U) == sign * limits.accelerometer && state.b_g(0U) == sign * limits.gyroscope &&
                        state.b_g(1U) == inside_g && state.b_g(2U) == sign * limits.gyroscope &&
                        same_navigation(state, original),
                    profile, "inclusive per-axis bounds preserve inside values and other nominal state");
        auto const bounded = state;
        test.expect(formal_eskf::runtime::try_limit_bias<Linalg>(state, limits) == Status::success &&
                        same_bits(state.b_a, bounded.b_a) && same_bits(state.b_g, bounded.b_g),
                    profile, "limiting is idempotent; it does not impose a vector norm bound");
    }
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        state.b_a.set(axis, std::numeric_limits<Scalar>::max());
        state.b_g.set(axis, -std::numeric_limits<Scalar>::max());
    }
    test.expect(formal_eskf::runtime::try_limit_bias<Linalg>(state, {}) == Status::success, profile,
                "zero bounds accept and limit even largest finite inputs without overflow");
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        test.expect(state.b_a(axis) == Scalar{0} && state.b_g(axis) == Scalar{0}, profile, "zero bounds force zero");
    }
}

template <typename Linalg> void test_invalid(TestContext & test)
{
    using Scalar = typename Linalg::value_type;
    for (auto invalid : {Scalar{-1}, std::numeric_limits<Scalar>::infinity(), -std::numeric_limits<Scalar>::infinity(),
                         std::numeric_limits<Scalar>::quiet_NaN()})
    {
        for (std::size_t field = 0U; field < 8U; ++field)
        {
            // Out-of-range valid coefficients must also survive a later invalid input.
            formal_eskf::configuration::Ins::NominalState<Linalg> state;
            state.b_a.set(0U, Scalar{2});
            state.b_g.set(0U, Scalar{-2});
            formal_eskf::runtime::BiasLimits<Scalar> limits{Scalar{0.5}, Scalar{0.125}};
            if (field < 2U)
            {
                (field == 0U ? limits.accelerometer : limits.gyroscope) = invalid;
            }
            else
            {
                if (std::isfinite(invalid))
                {
                    continue; // Negative biases are valid.
                }
                auto & bias = field < 5U ? state.b_a : state.b_g;
                bias.set((field - 2U) % 3U, invalid);
            }
            auto const before = state;
            auto const expected = std::isfinite(invalid) ? Status::domain_error : Status::non_finite_input;
            test.expect(formal_eskf::runtime::try_limit_bias<Linalg>(state, limits) == expected &&
                            same_navigation(state, before) && same_bits(state.b_a, before.b_a) &&
                            same_bits(state.b_g, before.b_g),
                        profile, "invalid limits or any non-finite bias leave the complete state unchanged");
        }
    }
}

template <typename Linalg> void test_correction(TestContext & test)
{
    using Scalar = typename Linalg::value_type;
    using Vector = typename Linalg::template vector_type<3U>;
    using Covariance = typename Linalg::template matrix_type<15U, 15U>;
    using Matrix3 = typename Linalg::template matrix_type<3U, 3U>;
    using State = formal_eskf::configuration::Ins::NominalState<Linalg>;
    formal_eskf::runtime::BiasLimits<Scalar> const limits{Scalar{0.125}, Scalar{0.0625}};
    Covariance P = Covariance::identity();
    // SPD covariance linking measured velocity to both biases, with opposing signs.
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        P.set(3U + axis, 9U + axis, Scalar{0.25});
        P.set(9U + axis, 3U + axis, Scalar{0.25});
        P.set(3U + axis, 12U + axis, Scalar{-0.25});
        P.set(12U + axis, 3U + axis, Scalar{-0.25});
    }
    State state;
    auto const V = Matrix3::identity();
    auto const velocity = Vector::from_row_major({Scalar{4}, Scalar{-4}, Scalar{4}});
    test.expect(formal_eskf::try_correct_velocity(state, P, velocity, V, (Scalar{1} / Scalar{1024}), state, P) ==
                    Status::success,
                profile, "real correction produces nonzero bias through cross-covariances");
    auto const corrected = state;
    auto const corrected_P = P;
    test.expect(corrected.b_a(0U) > limits.accelerometer && corrected.b_g(0U) < -limits.gyroscope &&
                    formal_eskf::runtime::try_limit_bias<Linalg>(state, limits) == Status::success &&
                    state.b_a(0U) == limits.accelerometer && state.b_a(1U) == -limits.accelerometer &&
                    state.b_g(0U) == -limits.gyroscope && state.b_g(1U) == limits.gyroscope &&
                    same_navigation(state, corrected) && same_bits(P, corrected_P),
                profile, "post-correction limiting preserves navigation and the computed covariance");

    auto const bounded = state;
    formal_eskf::configuration::Ins::BiasUpdate const disabled{{false, false, false}, {false, false, false}};
    test.expect(formal_eskf::try_correct_velocity(state, P, velocity, V, (Scalar{1} / Scalar{1024}), state, P,
                                                  disabled) == Status::success &&
                    formal_eskf::runtime::try_limit_bias<Linalg>(state, limits) == Status::success &&
                    same_bits(state.b_a, bounded.b_a) && same_bits(state.b_g, bounded.b_g),
                profile, "inhibited biases remain held when already inside fixed limits");
    state = corrected;
    test.expect(formal_eskf::try_correct_velocity(state, P, velocity, V, (Scalar{1} / Scalar{1024}), state, P,
                                                  disabled) == Status::success &&
                    same_bits(state.b_a, corrected.b_a) &&
                    formal_eskf::runtime::try_limit_bias<Linalg>(state, limits) == Status::success &&
                    state.b_a(0U) == limits.accelerometer && state.b_g(0U) == -limits.gyroscope,
                profile, "limiting still repairs out-of-range bias despite disabled learning");
    // A correction in the opposite direction must be able to leave saturation.
    P = corrected_P;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        P.set(3U + axis, 12U + axis, Scalar{0});
        P.set(12U + axis, 3U + axis, Scalar{0});
    }
    auto const reverse_velocity = state.v_n - Vector::from_row_major({Scalar{0.25}, Scalar{-0.25}, Scalar{0.25}});
    test.expect(formal_eskf::try_correct_velocity(state, P, reverse_velocity, V, (Scalar{1} / Scalar{1024}), state,
                                                  P) == Status::success &&
                    formal_eskf::runtime::try_limit_bias<Linalg>(state, limits) == Status::success &&
                    state.b_a(0U) < limits.accelerometer && state.b_a(0U) > Scalar{0},
                profile, "saturation does not latch or prevent a later inward correction");
}

} /* end namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    TestContext test;
    test_bounds<formal_eskf::test::Backend<float>>(test);
    test_bounds<formal_eskf::test::Backend<double>>(test);
    test_invalid<formal_eskf::test::Backend<float>>(test);
    test_invalid<formal_eskf::test::Backend<double>>(test);
    test_correction<formal_eskf::test::Backend<float>>(test);
    test_correction<formal_eskf::test::Backend<double>>(test);
    if (test.failures() != 0)
    {
        return 1;
    }
    std::cout << "Bias limits: pass\n";
    return 0;
}
