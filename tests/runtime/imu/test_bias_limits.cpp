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
#include <formal_eskf/runtime/imu.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{

using formal_eskf::Status;
using TestContext = formal_eskf::test::Context;

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

template <typename Linalg> void test_bounds(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    formal_eskf::configuration::Ins::NominalState<Linalg> state;
    state.p_n(0U) = Scalar{3};
    state.v_n(2U) = Scalar{-2};
    auto const original = state;
    formal_eskf::runtime::BiasLimits<Scalar> const limits{Scalar{0.5}, Scalar{0.125}};
    for (auto sign : {Scalar{-1}, Scalar{1}})
    {
        // Exact bound, one ULP inside and one ULP outside on different axes.
        for (auto const group : {0U, 1U})
        {
            // Coefficient writes require the non-const matrix operator().
            // cppcheck-suppress constVariableReference
            auto & bias = group == 0U ? state.b_a : state.b_g;
            auto const bound = sign * (group == 0U ? limits.accelerometer : limits.gyroscope);
            bias(0U) = bound;
            bias(1U) = std::nextafter(bound, Scalar{0});
            bias(2U) = std::nextafter(bound, sign * std::numeric_limits<Scalar>::infinity());
        }
        auto const inside_a = state.b_a(1U);
        auto const inside_g = state.b_g(1U);
        test.expect(formal_eskf::runtime::limit_bias<Linalg>(state, limits) == Status::success &&
                        state.b_a(0U) == sign * limits.accelerometer && state.b_a(1U) == inside_a &&
                        state.b_a(2U) == sign * limits.accelerometer && state.b_g(0U) == sign * limits.gyroscope &&
                        state.b_g(1U) == inside_g && state.b_g(2U) == sign * limits.gyroscope &&
                        same_navigation(state, original),
                    profile, "inclusive per-axis bounds preserve inside values and other nominal state");
        auto const bounded = state;
        test.expect(formal_eskf::runtime::limit_bias<Linalg>(state, limits) == Status::success &&
                        same_bits(state.b_a, bounded.b_a) && same_bits(state.b_g, bounded.b_g),
                    profile, "limiting is idempotent; it does not impose a vector norm bound");
    }
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        state.b_a(axis) = std::numeric_limits<Scalar>::max();
        state.b_g(axis) = -std::numeric_limits<Scalar>::max();
    }
    test.expect(formal_eskf::runtime::limit_bias<Linalg>(state, {}) == Status::success, profile,
                "zero bounds accept and limit even largest finite inputs without overflow");
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        test.expect(state.b_a(axis) == Scalar{0} && state.b_g(axis) == Scalar{0}, profile, "zero bounds force zero");
    }
    state = original;
    state.b_a(0U) = -Scalar{0};
    state.b_g(2U) = -Scalar{0};
    auto const signed_zero = state;
    test.expect(formal_eskf::runtime::limit_bias<Linalg>(state, limits) == Status::success &&
                    same_bits(state.b_a, signed_zero.b_a) && same_bits(state.b_g, signed_zero.b_g),
                profile, "in-range positive and negative zero representations remain untouched");
}

template <typename Linalg> void test_invalid(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    for (auto invalid : {Scalar{-1}, std::numeric_limits<Scalar>::infinity(), -std::numeric_limits<Scalar>::infinity(),
                         std::numeric_limits<Scalar>::quiet_NaN()})
    {
        for (std::size_t field = 0U; field < 8U; ++field)
        {
            // Out-of-range valid coefficients must also survive a later invalid input.
            formal_eskf::configuration::Ins::NominalState<Linalg> state;
            state.b_a(0U) = Scalar{2};
            state.b_g(0U) = Scalar{-2};
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
                // Coefficient writes require the non-const matrix operator().
                // cppcheck-suppress constVariableReference
                auto & bias = field < 5U ? state.b_a : state.b_g;
                bias((field - 2U) % 3U) = invalid;
            }
            auto const before = state;
            auto const expected = std::isfinite(invalid) ? Status::domain_error : Status::non_finite_input;
            test.expect(formal_eskf::runtime::limit_bias<Linalg>(state, limits) == expected &&
                            same_navigation(state, before) && same_bits(state.b_a, before.b_a) &&
                            same_bits(state.b_g, before.b_g),
                        profile, "invalid limits or any non-finite bias leave the complete state unchanged");
        }
    }
    // The limiter consumes only limits and six biases, not unrelated nominal fields.
    for (std::size_t field = 0U; field < 6U; ++field)
    {
        formal_eskf::configuration::Ins::NominalState<Linalg> state;
        if (field < 3U)
        {
            state.p_n(field) = std::numeric_limits<Scalar>::quiet_NaN();
        }
        else
        {
            state.v_n(field - 3U) = std::numeric_limits<Scalar>::infinity();
        }
        state.b_a(0U) = Scalar{2};
        auto const before = state;
        test.expect(formal_eskf::runtime::limit_bias<Linalg>(state, {Scalar{0.5}, Scalar{0.125}}) == Status::success &&
                        same_navigation(state, before) && state.b_a(0U) == Scalar{0.5},
                    profile, "nonfinite unused navigation fields neither reject nor get repaired");
        state = before;
        state.b_g(2U) = std::numeric_limits<Scalar>::quiet_NaN();
        auto const invalid_state = state;
        test.expect(formal_eskf::runtime::limit_bias<Linalg>(state, {Scalar{-1}, Scalar{0.125}}) ==
                            Status::non_finite_input &&
                        same_navigation(state, invalid_state) && same_bits(state.b_a, invalid_state.b_a) &&
                        same_bits(state.b_g, invalid_state.b_g),
                    profile, "nonfinite last bias wins before negative-limit domain validation and earlier clamp");
    }
}

template <typename Linalg> void test_correction(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    using Vector = formal_eskf::linalg::Vector<Linalg, 3U>;
    using Covariance = formal_eskf::linalg::Matrix<Linalg, 15U, 15U>;
    using Matrix3 = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    using State = formal_eskf::configuration::Ins::NominalState<Linalg>;
    formal_eskf::runtime::BiasLimits<Scalar> const limits{Scalar{0.125}, Scalar{0.0625}};
    Covariance P = Covariance::identity();
    // SPD covariance linking measured velocity to both biases, with opposing signs.
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        P(3U + axis, 9U + axis) = Scalar{0.25};
        P(9U + axis, 3U + axis) = Scalar{0.25};
        P(3U + axis, 12U + axis) = Scalar{-0.25};
        P(12U + axis, 3U + axis) = Scalar{-0.25};
    }
    State state;
    auto const V = Matrix3::identity();
    auto const velocity = Vector::from_row_major({Scalar{4}, Scalar{-4}, Scalar{4}});
    test.expect(formal_eskf::correct_velocity(state, P, velocity, V, (Scalar{1} / Scalar{1024}), state, P) ==
                    Status::success,
                profile, "real correction produces nonzero bias through cross-covariances");
    auto const corrected = state;
    auto const corrected_P = P;
    test.expect(corrected.b_a(0U) > limits.accelerometer && corrected.b_g(0U) < -limits.gyroscope &&
                    formal_eskf::runtime::limit_bias<Linalg>(state, limits) == Status::success &&
                    state.b_a(0U) == limits.accelerometer && state.b_a(1U) == -limits.accelerometer &&
                    state.b_g(0U) == -limits.gyroscope && state.b_g(1U) == limits.gyroscope &&
                    same_navigation(state, corrected) && same_bits(P, corrected_P),
                profile, "post-correction limiting preserves navigation and the computed covariance");

    auto const bounded = state;
    formal_eskf::configuration::Ins::BiasUpdate const disabled{{false, false, false}, {false, false, false}};
    test.expect(formal_eskf::correct_velocity(state, P, velocity, V, (Scalar{1} / Scalar{1024}), state, P, disabled) ==
                        Status::success &&
                    formal_eskf::runtime::limit_bias<Linalg>(state, limits) == Status::success &&
                    same_bits(state.b_a, bounded.b_a) && same_bits(state.b_g, bounded.b_g),
                profile, "inhibited biases remain held when already inside fixed limits");
    state = corrected;
    test.expect(formal_eskf::correct_velocity(state, P, velocity, V, (Scalar{1} / Scalar{1024}), state, P, disabled) ==
                        Status::success &&
                    same_bits(state.b_a, corrected.b_a) &&
                    formal_eskf::runtime::limit_bias<Linalg>(state, limits) == Status::success &&
                    state.b_a(0U) == limits.accelerometer && state.b_g(0U) == -limits.gyroscope,
                profile, "limiting still repairs out-of-range bias despite disabled learning");
    // A correction in the opposite direction must be able to leave saturation.
    P = corrected_P;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        P(3U + axis, 12U + axis) = Scalar{0};
        P(12U + axis, 3U + axis) = Scalar{0};
    }
    auto const reverse_velocity = state.v_n - Vector::from_row_major({Scalar{0.25}, Scalar{-0.25}, Scalar{0.25}});
    test.expect(formal_eskf::correct_velocity(state, P, reverse_velocity, V, (Scalar{1} / Scalar{1024}), state, P) ==
                        Status::success &&
                    formal_eskf::runtime::limit_bias<Linalg>(state, limits) == Status::success &&
                    state.b_a(0U) < limits.accelerometer && state.b_a(0U) > Scalar{0},
                profile, "saturation does not latch or prevent a later inward correction");
}

template <typename Linalg> void no_latch(TestContext & test, std::string_view profile)
{
    test_correction<Linalg>(test, profile);
    using Scalar = typename Linalg::value_type;
    using Vector = formal_eskf::linalg::Vector<Linalg, 3U>;
    using Covariance = formal_eskf::linalg::Matrix<Linalg, 15U, 15U>;
    using Matrix3 = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    formal_eskf::configuration::Ins::NominalState<Linalg> state;
    state.b_a(0U) = Scalar{1};
    formal_eskf::runtime::BiasLimits<Scalar> const limits{Scalar{0.125}, Scalar{0.125}};
    test.expect(formal_eskf::runtime::limit_bias<Linalg>(state, limits) == Status::success &&
                    state.b_a(0U) == Scalar{0.125},
                profile, "begin on positive saturation boundary");
    auto P = Covariance::identity() * Scalar{2};
    P(3U, 9U) = Scalar{0.25};
    P(9U, 3U) = Scalar{0.25};
    auto const V = Matrix3::identity() * Scalar{2};
    Vector measurement;
    measurement(0U) = Scalar{-0.5};
    // S=4I and L=2I: K_ba,x=1/16; delta_ba,x=(-1/2)/16=-1/32.
    test.expect(formal_eskf::correct_velocity(state, P, measurement, V, Scalar{0.125}, state, P) == Status::success &&
                    state.b_a(0U) == Scalar{0.09375},
                profile, "independent dyadic correction leaves saturation by exactly one thirty-second");
    auto const corrected = state;
    auto const corrected_P = P;
    test.expect(formal_eskf::runtime::limit_bias<Linalg>(state, limits) == Status::success &&
                    same_navigation(state, corrected) && same_bits(state.b_a, corrected.b_a) &&
                    same_bits(state.b_g, corrected.b_g) && same_bits(P, corrected_P),
                profile, "inward correction releases immediately without covariance or nominal changes");
}

} /* end namespace */

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Case;
    using Float = formal_eskf::test::Backend<float>;
    using Double = formal_eskf::test::Backend<double>;
    Case const cases[] = {
        {"TC-R-BIAS-LIMITS-01", "float", test_bounds<Float>},  {"TC-R-BIAS-LIMITS-01", "double", test_bounds<Double>},
        {"TC-R-BIAS-LIMITS-02", "float", test_invalid<Float>}, {"TC-R-BIAS-LIMITS-02", "double", test_invalid<Double>},
        {"TC-R-BIAS-LIMITS-03", "float", no_latch<Float>},     {"TC-R-BIAS-LIMITS-03", "double", no_latch<Double>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
