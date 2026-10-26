/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/magnetometer_reset.hpp>
#include <formal_eskf/runtime/magnetometer_filter.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <string_view>
#include <type_traits>

#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{

using formal_eskf::Status;
using TestContext = formal_eskf::test::Context;
constexpr std::string_view profile = "magnetic heading reset";

template <typename Matrix> bool same_matrix(Matrix const & a, Matrix const & b)
{
    return formal_eskf::linalg::max_abs(a - b) == typename Matrix::value_type{0};
}

template <typename Scalar> void test_reset_filter(TestContext & test)
{
    using Linalg = formal_eskf::test::Backend<Scalar>;
    using Filter = formal_eskf::runtime::MagnetometerFilter<Linalg>;
    using Vector = typename Filter::vector_type;
    auto const first = Vector::from_row_major({Scalar{20}, Scalar{0}, Scalar{40}});
    auto const second = Vector::from_row_major({Scalar{0}, Scalar{20}, Scalar{40}});
    Filter filter;
    test.expect(filter.update(first) == Status::success && filter.initialized() && same_matrix(filter.field_b(), first),
                profile, "first valid magnetic sample seeds reset LPF exactly");
    test.expect(filter.update(second) == Status::success &&
                    formal_eskf::linalg::max_abs(filter.field_b() -
                                                 Vector::from_row_major({Scalar{18}, Scalar{2}, Scalar{40}})) <
                        static_cast<Scalar>(1e-5),
                profile, "reset LPF uses nominal 10 ms / 90 ms alpha");
    test.expect(filter.update(second, true) == Status::success && same_matrix(filter.field_b(), second), profile,
                "source reseed retains no old-source weighted field");
    auto invalid = first;
    invalid.set(0U, std::numeric_limits<Scalar>::quiet_NaN());
    test.expect(filter.update(invalid, true) == Status::non_finite_input && same_matrix(filter.field_b(), second),
                profile, "invalid reseed preserves the previous valid filter");
    auto extreme = first;
    extreme.set(0U, std::numeric_limits<Scalar>::max());
    test.expect(filter.update(extreme, true) == Status::success, profile, "seed finite extreme sample");
    test.expect(filter.update(extreme * Scalar{-1}) == Status::non_finite_result &&
                    same_matrix(filter.field_b(), extreme),
                profile, "overflow during filtering preserves history");
    filter.reset();
    test.expect(!filter.initialized() && filter.update(second) == Status::success &&
                    same_matrix(filter.field_b(), second),
                profile, "explicit filter reset requires a fresh seed");
}

template <typename Scalar, typename Configuration> void run_reset_tests(TestContext & test)
{
    using Linalg = formal_eskf::test::Backend<Scalar>;
    using State = typename Configuration::template NominalState<Linalg>;
    constexpr bool ins = std::is_same_v<Configuration, formal_eskf::configuration::Ins>;
    constexpr std::size_t size = ins ? 15U : 3U;
    constexpr std::size_t offset = ins ? 6U : 0U;
    using Covariance = formal_eskf::linalg::Matrix<Linalg, size, size>;
    using Vector = formal_eskf::linalg::Matrix<Linalg, 3U, 1U>;
    using Quaternion = formal_eskf::so3::UnitQuaternion<Linalg>;
    constexpr Scalar minimum_norm = static_cast<Scalar>(1e-6);
    constexpr Scalar tolerance = std::is_same_v<Scalar, float> ? static_cast<Scalar>(3e-4) : static_cast<Scalar>(1e-11);
    Scalar const pi = formal_eskf::scalar::pi<typename Linalg::scalar_math_type>();
    auto const reference = Vector::from_row_major({Scalar{20}, Scalar{-7}, Scalar{42}});
    auto const down = Vector::from_row_major({Scalar{0}, Scalar{0}, Scalar{1}});
    auto exp = [&](Scalar x, Scalar y, Scalar z)
    {
        Quaternion q;
        test.expect(formal_eskf::so3::try_exp<Linalg>(Vector::from_row_major({x, y, z}), minimum_norm, q) ==
                        Status::success,
                    profile, "construct finite rotation fixture");
        return q;
    };
    Covariance P;
    for (std::size_t row = 0U; row < size; ++row)
    {
        for (std::size_t column = 0U; column < size; ++column)
        {
            // Dense symmetric, strictly diagonally dominant positive definite prior.
            P.set(row, column,
                  row == column ? Scalar{4} + static_cast<Scalar>(row) / Scalar{4} : static_cast<Scalar>(0.125));
        }
    }
    for (Scalar pitch : {Scalar{0}, pi / Scalar{2}, -pi / Scalar{2}, pi / Scalar{5}})
    {
        Quaternion const truth = exp(Scalar{0}, Scalar{0}, pi * static_cast<Scalar>(0.9)) *
                                 exp(Scalar{0}, pitch, Scalar{0}) * exp(pi / Scalar{7}, Scalar{0}, Scalar{0});
        auto const measurement = formal_eskf::so3::inverse_rotate(truth, reference);
        for (Scalar error : {Scalar{0}, pi, -pi * static_cast<Scalar>(0.8), pi / Scalar{3}})
        {
            State state;
            state.q_nb = exp(Scalar{0}, Scalar{0}, error) * truth;
            if constexpr (ins)
            {
                state.p_n = Vector::from_row_major({Scalar{1}, Scalar{2}, Scalar{-3}});
                state.v_n = state.p_n * Scalar{2};
                state.b_a = state.p_n / Scalar{8};
                state.b_g = state.p_n / Scalar{16};
            }
            State output;
            Covariance result;
            Quaternion delta;
            test.expect(formal_eskf::runtime::try_reset_magnetic_heading(state, P, measurement, reference, Scalar{0},
                                                                         Scalar{5}, minimum_norm, output, result,
                                                                         delta) == Status::success,
                        profile, "reset succeeds across heading wrap and vertical Euler singularities");
            test.expect(formal_eskf::so3::same_rotation(output.q_nb, truth, tolerance) &&
                            formal_eskf::so3::same_rotation(delta * state.q_nb, output.q_nb, tolerance) &&
                            formal_eskf::linalg::max_abs(formal_eskf::so3::inverse_rotate(output.q_nb, down) -
                                                         formal_eskf::so3::inverse_rotate(state.q_nb, down)) <
                                tolerance,
                        profile, "field recovers true heading; applied left delta preserves tilt");
            if constexpr (ins)
            {
                test.expect(same_matrix(state.p_n, output.p_n) && same_matrix(state.v_n, output.v_n) &&
                                same_matrix(state.b_a, output.b_a) && same_matrix(state.b_g, output.b_g),
                            profile, "reset preserves all non-attitude nominal values");
            }

            // Independent oracle: convert full P to navigation attitude errors,
            // replace/decouple global yaw, then compare there after the reset.
            auto T = Covariance::identity();
            T.template set_block<offset, offset>(formal_eskf::so3::to_rotation_matrix(state.q_nb));
            auto expected = T * P * formal_eskf::linalg::transpose(T);
            for (std::size_t index = 0U; index < size; ++index)
            {
                expected.set(offset + 2U, index, Scalar{0});
                expected.set(index, offset + 2U, Scalar{0});
            }
            Scalar const minimum_std = static_cast<Scalar>(0.01);
            expected.set(offset + 2U, offset + 2U, minimum_std * minimum_std);
            T.template set_block<offset, offset>(formal_eskf::so3::to_rotation_matrix(output.q_nb));
            auto const actual = T * result * formal_eskf::linalg::transpose(T);
            test.expect(formal_eskf::linalg::max_abs(actual - expected) < tolerance &&
                            formal_eskf::linalg::is_symmetric(result, Scalar{0}),
                        profile, "global tilt/cross terms retained, yaw decorrelated and floored; exact symmetry");
            for (std::size_t row = 0U; row < size; ++row)
            {
                for (std::size_t column = 0U; column < size; ++column)
                {
                    if ((row < offset || row >= offset + 3U) && (column < offset || column >= offset + 3U))
                    {
                        test.expect(result(row, column) == P(row, column), profile, "non-attitude P block unchanged");
                    }
                }
            }
            auto negated = state;
            negated.q_nb = -state.q_nb;
            Covariance alias_P = P;
            Quaternion alias_delta;
            test.expect(formal_eskf::runtime::try_reset_magnetic_heading(negated, alias_P, measurement, reference,
                                                                         Scalar{0}, Scalar{5}, minimum_norm, negated,
                                                                         alias_P, alias_delta) == Status::success &&
                            formal_eskf::so3::same_rotation(negated.q_nb, output.q_nb, tolerance) &&
                            formal_eskf::linalg::max_abs(alias_P - result) < tolerance,
                        profile, "q/-q equivalence and in-place state/P reset");
            for (bool alias_state : {false, true})
            {
                auto prior = state;
                auto prior_P = P;
                State separate;
                Covariance separate_P;
                auto & state_result = alias_state ? prior : separate;
                auto & P_result = alias_state ? separate_P : prior_P;
                test.expect(formal_eskf::runtime::try_reset_magnetic_heading(
                                prior, prior_P, measurement, reference, Scalar{0}, Scalar{5}, minimum_norm,
                                state_result, P_result, alias_delta) == Status::success &&
                                formal_eskf::so3::same_rotation(state_result.q_nb, output.q_nb, tolerance) &&
                                same_matrix(P_result, result),
                            profile, "state and P outputs may independently alias their complete priors");
            }
        }
    }

    State state;
    State output;
    output.q_nb = exp(Scalar{1}, Scalar{0}, Scalar{0});
    Quaternion delta = output.q_nb;
    auto const sentinel = output.q_nb;
    Covariance result = Covariance::identity() * Scalar{17};
    for (unsigned failure = 0U; failure < 12U; ++failure)
    {
        auto input_P = P;
        auto measurement = reference;
        auto field = reference;
        Scalar variance = static_cast<Scalar>(0.09);
        Scalar horizontal = Scalar{5};
        Scalar norm = minimum_norm;
        switch (failure)
        {
        case 0U:
            measurement.set(0U, std::numeric_limits<Scalar>::quiet_NaN());
            break;
        case 1U:
            field.set(1U, std::numeric_limits<Scalar>::infinity());
            break;
        case 2U:
            input_P.set(0U, 0U, Scalar{-1});
            break;
        case 3U:
            input_P.set(0U, 1U, Scalar{1});
            break;
        case 4U:
            variance = Scalar{-1};
            break;
        case 5U:
            variance = std::numeric_limits<Scalar>::infinity();
            break;
        case 6U:
            horizontal = Scalar{0};
            break;
        case 7U:
            horizontal = Scalar{100};
            break;
        case 8U:
            norm = Scalar{0};
            break;
        case 9U:
            measurement = down * Scalar{40};
            break;
        case 10U:
            measurement.set(0U, std::numeric_limits<Scalar>::max());
            break;
        default:
            field = down * Scalar{40};
            break;
        }
        auto const before_P = result;
        auto const status = formal_eskf::runtime::try_reset_magnetic_heading(
            state, input_P, measurement, field, variance, horizontal, norm, output, result, delta);
        test.expect(!formal_eskf::succeeded(status) && formal_eskf::so3::same_coefficients(output.q_nb, sentinel) &&
                        formal_eskf::so3::same_coefficients(delta, sentinel) && same_matrix(result, before_P),
                    profile, "invalid inputs/weak field/overflow preserve all outputs");
    }
    auto overflowing_P = P;
    for (std::size_t row = 0U; row < size; ++row)
    {
        for (std::size_t column = 0U; column < size; ++column)
        {
            overflowing_P.set(row, column, std::numeric_limits<Scalar>::max());
        }
    }
    auto const before_P = result;
    test.expect(formal_eskf::runtime::try_reset_magnetic_heading(
                    state, overflowing_P, Vector::from_row_major({Scalar{20}, Scalar{-20}, Scalar{40}}), reference,
                    Scalar{1}, Scalar{5}, minimum_norm, output, result, delta) == Status::non_finite_result &&
                    formal_eskf::so3::same_coefficients(output.q_nb, sentinel) &&
                    formal_eskf::so3::same_coefficients(delta, sentinel) && same_matrix(result, before_P),
                profile, "late covariance overflow preserves the already-computed candidate attitude and all outputs");
    if constexpr (ins)
    {
        auto invalid_state = state;
        invalid_state.b_a.set(0U, std::numeric_limits<Scalar>::quiet_NaN());
        test.expect(formal_eskf::runtime::try_reset_magnetic_heading(invalid_state, P, reference, reference, Scalar{1},
                                                                     Scalar{5}, minimum_norm, output, result,
                                                                     delta) == Status::non_finite_input &&
                        formal_eskf::so3::same_coefficients(output.q_nb, sentinel) && same_matrix(result, before_P),
                    profile, "retained non-finite INS fields cannot pass through reset");
    }
    Scalar const variance = static_cast<Scalar>(0.09);
    test.expect(formal_eskf::runtime::try_reset_magnetic_heading(state, P, reference, reference, variance, Scalar{5},
                                                                 minimum_norm, output, result,
                                                                 delta) == Status::success &&
                    std::abs(result(offset + 2U, offset + 2U) - variance) < tolerance,
                profile, "configured yaw variance above floor is used in radians squared");
}

} /* end namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    TestContext test;
    test_reset_filter<float>(test);
    test_reset_filter<double>(test);
    run_reset_tests<float, formal_eskf::configuration::Ahrs>(test);
    run_reset_tests<float, formal_eskf::configuration::Ins>(test);
    run_reset_tests<double, formal_eskf::configuration::Ahrs>(test);
    run_reset_tests<double, formal_eskf::configuration::Ins>(test);
    if (test.failures() != 0)
    {
        return 1;
    }
    std::cout << "Magnetic heading reset: pass\n";
    return 0;
}
