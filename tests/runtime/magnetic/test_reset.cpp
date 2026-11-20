/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/magnetometer.hpp>
#include <array>
#include <bit>
#include <cmath>
#include <limits>
#include <numbers>
#include <type_traits>
#include "test_backend.hpp"
#include "test_cases.hpp"
#include "correction_reference.hpp"

namespace
{
using formal_eskf::Status;
using Context = formal_eskf::test::Context;
using formal_eskf::configuration::Ahrs;
using formal_eskf::configuration::Ins;
namespace reference = formal_eskf::test::reference;
using QuaternionValue = std::array<long double, 4>;
using Rotation = reference::Dense<3, 3>;

template <typename Matrix> bool same(Matrix const & a, Matrix const & b)
{
    using Bits = std::array<std::byte, sizeof(typename Matrix::value_type)>;
    for (std::size_t row = 0; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0; column < Matrix::column_count; ++column)
        {
            if (std::bit_cast<Bits>(a(row, column)) != std::bit_cast<Bits>(b(row, column)))
            {
                return false;
            }
        }
    }
    return true;
}
template <typename State> bool same_state(State const & a, State const & b)
{
    bool equal = same(a.q_nb.coefficients(), b.q_nb.coefficients());
    if constexpr (requires { a.p_n; })
    {
        equal = equal && same(a.p_n, b.p_n) && same(a.v_n, b.v_n) && same(a.b_a, b.b_a) && same(a.b_g, b.b_g);
    }
    return equal;
}
QuaternionValue product(QuaternionValue const & a, QuaternionValue const & b)
{
    return {
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1], a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}
Rotation rotation(QuaternionValue const & q)
{
    auto const w = q[0], x = q[1], y = q[2], z = q[3];
    return {{{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
             {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
             {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}}};
}
template <typename Q> QuaternionValue copied(Q const & q) { return {q.q0(), q.q1(), q.q2(), q.q3()}; }

template <typename Linalg, typename Configuration> struct Fixture
{
    using Scalar = typename Linalg::value_type;
    using State = typename Configuration::template NominalState<Linalg>;
    using Q = formal_eskf::so3::UnitQuaternion<Linalg>;
    static constexpr bool ins = std::is_same_v<Configuration, Ins>;
    static constexpr std::size_t size = Configuration::error_state_dimension, offset = ins ? 6 : 0;
    using Matrix = formal_eskf::linalg::Matrix<Linalg, size, size>;
    using Vector = formal_eskf::linalg::Vector<Linalg, 3>;
    State state{};
    Matrix P{};
    Vector field = Vector::from_row_major({Scalar{20}, Scalar{-7}, Scalar{42}});
    Vector sample = field;
    Scalar yaw_variance = Scalar{0}, minimum_horizontal = Scalar{5}, minimum_norm = Scalar{0.125};
    Fixture()
    {
        for (std::size_t row = 0; row < size; ++row)
        {
            for (std::size_t column = 0; column < size; ++column)
            {
                P(row, column) = row == column ? Scalar{4} + static_cast<Scalar>(row) / Scalar{4} : Scalar{0.125};
            }
        }
        if constexpr (ins)
        {
            state.p_n = Vector::from_row_major({Scalar{1}, Scalar{2}, Scalar{-3}});
            state.v_n = state.p_n * Scalar{2};
            state.b_a = state.p_n / Scalar{8};
            state.b_g = state.p_n / Scalar{16};
        }
    }
    Status apply(State & output, Matrix & output_P, Q & delta) const
    {
        return formal_eskf::runtime::reset_magnetic_heading(state, P, sample, field, yaw_variance, minimum_horizontal,
                                                            minimum_norm, output, output_P, delta);
    }
};

template <typename F> struct Expected
{
    QuaternionValue delta{}, attitude{};
    Rotation before{}, after{};
    reference::Dense<F::size, F::size> P{};
};
template <typename F> Expected<F> oracle(F const & fixture)
{
    Expected<F> result;
    result.before = rotation(copied(fixture.state.q_nb));
    std::array<long double, 3> measured{};
    for (std::size_t row = 0; row < 3; ++row)
    {
        for (std::size_t column = 0; column < 3; ++column)
        {
            measured[row] += result.before[row][column] * fixture.sample(column);
        }
    }
    long double angle =
        std::atan2(static_cast<long double>(fixture.field(1)), static_cast<long double>(fixture.field(0))) -
        std::atan2(measured[1], measured[0]);
    if (angle > std::numbers::pi_v<long double>)
    {
        angle -= 2 * std::numbers::pi_v<long double>;
    }
    if (angle < -std::numbers::pi_v<long double>)
    {
        angle += 2 * std::numbers::pi_v<long double>;
    }
    result.delta = {std::cos(angle / 2), 0, 0, std::sin(angle / 2)};
    result.attitude = product(result.delta, copied(fixture.state.q_nb));
    long double squared = 0;
    for (auto coefficient : result.attitude)
    {
        squared += coefficient * coefficient;
    }
    for (auto & coefficient : result.attitude)
    {
        coefficient /= std::sqrt(squared);
    }
    result.after = rotation(result.attitude);
    Rotation A{};
    // Removing navigation Down keeps exactly the first two navigation rows.
    for (std::size_t row = 0; row < 3; ++row)
    {
        for (std::size_t column = 0; column < 3; ++column)
        {
            for (std::size_t k = 0; k < 2; ++k)
            {
                A[row][column] += result.after[k][row] * result.before[k][column];
            }
        }
    }
    result.P = reference::copied(fixture.P);
    reference::Dense<3, F::size> cross{};
    for (std::size_t row = 0; row < 3; ++row)
    {
        for (std::size_t column = 0; column < F::size; ++column)
        {
            for (std::size_t k = 0; k < 3; ++k)
            {
                cross[row][column] += A[row][k] * fixture.P(F::offset + k, column);
            }
        }
    }
    using Scalar = typename F::Scalar;
    Scalar const minimum = static_cast<Scalar>(0.01);
    Scalar const variance = std::max(fixture.yaw_variance, minimum * minimum);
    for (std::size_t row = 0; row < 3; ++row)
    {
        for (std::size_t column = 0; column < F::size; ++column)
        {
            result.P[F::offset + row][column] = cross[row][column];
            result.P[column][F::offset + row] = cross[row][column];
        }
    }
    for (std::size_t row = 0; row < 3; ++row)
    {
        for (std::size_t column = 0; column < 3; ++column)
        {
            long double value = result.after[2][row] * result.after[2][column] * variance;
            for (std::size_t k = 0; k < 3; ++k)
            {
                value += cross[row][F::offset + k] * A[column][k];
            }
            result.P[F::offset + row][F::offset + column] = value;
        }
    }
    return result;
}
template <typename F>
void compare(Context & test, std::string_view profile, F const & fixture, Expected<F> const & expected,
             typename F::State const & actual, typename F::Matrix const & P, typename F::Q const & delta)
{
    using Scalar = typename F::Scalar;
    long double const bound = 256 * std::numeric_limits<Scalar>::epsilon();
    auto const actual_rotation = rotation(copied(actual.q_nb));
    auto const delta_rotation = rotation(copied(delta));
    auto const expected_delta = rotation(expected.delta);
    bool attitude = true, covariance = true;
    for (std::size_t row = 0; row < 3; ++row)
    {
        for (std::size_t column = 0; column < 3; ++column)
        {
            attitude = attitude && std::abs(actual_rotation[row][column] - expected.after[row][column]) <= bound &&
                       std::abs(delta_rotation[row][column] - expected_delta[row][column]) <= bound;
        }
    }
    for (std::size_t row = 0; row < F::size; ++row)
    {
        for (std::size_t column = 0; column < F::size; ++column)
        {
            covariance = covariance && std::abs(static_cast<long double>(P(row, column)) - expected.P[row][column]) <=
                                           bound * (1 + std::abs(expected.P[row][column]));
            if ((row < F::offset || row >= F::offset + 3) && (column < F::offset || column >= F::offset + 3))
            {
                covariance = covariance && P(row, column) == fixture.P(row, column);
            }
            covariance = covariance && P(row, column) == P(column, row);
        }
    }
    test.expect(attitude && covariance, profile,
                "independent navigation-Down left reset and covariance transport agree; non-attitude block exact and "
                "symmetric");
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        test.expect(std::abs(actual_rotation[2][axis] - expected.before[2][axis]) <= bound, profile,
                    "body-expressed navigation Down is unchanged: reset cannot alter tilt");
    }
    if constexpr (F::ins)
    {
        test.expect(same(actual.p_n, fixture.state.p_n) && same(actual.v_n, fixture.state.v_n) &&
                        same(actual.b_a, fixture.state.b_a) && same(actual.b_g, fixture.state.b_g),
                    profile, "all non-attitude nominal fields retain exact coefficients");
    }
    // This fixed-domain nonlinear allowance is regression evidence, not a global
    // trig/rounding enclosure; exact axis controls are tested separately.
}

template <typename F>
void orient(Context & test, std::string_view profile, F & fixture, long double pitch, long double yaw_error)
{
    using Scalar = typename F::Scalar;
    long double const roll = std::numbers::pi_v<long double> / 7, yaw = 0.9L * std::numbers::pi_v<long double>;
    QuaternionValue const z{std::cos(yaw / 2), 0, 0, std::sin(yaw / 2)};
    QuaternionValue const y{std::cos(pitch / 2), 0, std::sin(pitch / 2), 0};
    QuaternionValue const x{std::cos(roll / 2), std::sin(roll / 2), 0, 0};
    auto const truth = product(product(z, y), x);
    auto const actual = product(QuaternionValue{std::cos(yaw_error / 2), 0, 0, std::sin(yaw_error / 2)}, truth);
    test.expect(F::Q::from_coefficients(static_cast<Scalar>(actual[0]), static_cast<Scalar>(actual[1]),
                                        static_cast<Scalar>(actual[2]), static_cast<Scalar>(actual[3]),
                                        fixture.minimum_norm, fixture.state.q_nb) == Status::success,
                profile, "independent Euler-composed fixture, including vertical pitch");
    auto const R = rotation(truth);
    for (std::size_t row = 0; row < 3; ++row)
    {
        long double value = 0;
        for (std::size_t column = 0; column < 3; ++column)
        {
            value += R[column][row] * fixture.field(column);
        }
        fixture.sample(row) = static_cast<Scalar>(value);
    }
}

template <typename Linalg, typename Configuration> void geometry(Context & test, std::string_view profile)
{
    using F = Fixture<Linalg, Configuration>;
    long double const pi = std::numbers::pi_v<long double>;
    for (auto pitch : {0.0L, pi / 2, -pi / 2, pi / 5})
    {
        for (auto error : {0.0L, pi, -0.8L * pi, pi / 3, -pi / 3})
        {
            F fixture;
            orient(test, profile, fixture, pitch, error);
            for (bool negative : {false, true})
            {
                auto signed_fixture = fixture;
                if (negative)
                {
                    signed_fixture.state.q_nb = -signed_fixture.state.q_nb;
                }
                auto const expected = oracle(signed_fixture);
                for (unsigned aliases = 0; aliases < 4; ++aliases)
                {
                    auto current = signed_fixture;
                    typename F::State output;
                    typename F::Matrix P;
                    typename F::Q delta;
                    auto & state_output = (aliases & 1U) ? current.state : output;
                    auto & covariance_output = (aliases & 2U) ? current.P : P;
                    test.expect(current.apply(state_output, covariance_output, delta) == Status::success, profile,
                                "reset succeeds at all heading/tilt/alias partitions");
                    compare(test, profile, signed_fixture, expected, state_output, covariance_output, delta);
                    test.expect(((aliases & 1U) != 0 || same_state(current.state, signed_fixture.state)) &&
                                    ((aliases & 2U) != 0 || same(current.P, signed_fixture.P)) &&
                                    same(current.sample, signed_fixture.sample) &&
                                    same(current.field, signed_fixture.field),
                                profile, "disjoint prior/measurement/reference inputs are pure");
                }
            }
        }
    }
}

template <typename Linalg, typename Configuration> void covariance(Context & test, std::string_view profile)
{
    using F = Fixture<Linalg, Configuration>;
    using Scalar = typename F::Scalar;
    Scalar const minimum = static_cast<Scalar>(0.01), floor = minimum * minimum;
    for (auto variance :
         {Scalar{0}, std::nextafter(floor, Scalar{0}), floor, std::nextafter(floor, Scalar{1}), Scalar{0.25}})
    {
        F fixture;
        fixture.yaw_variance = variance;
        auto output = fixture.state;
        auto P = fixture.P;
        typename F::Q delta;
        test.expect(fixture.apply(output, P, delta) == Status::success, profile, "axis-aligned floor fixture succeeds");
        auto expected = fixture.P;
        for (std::size_t axis = 0; axis < F::size; ++axis)
        {
            expected(F::offset + 2, axis) = Scalar{0};
            expected(axis, F::offset + 2) = Scalar{0};
        }
        expected(F::offset + 2, F::offset + 2) = variance < floor ? floor : variance;
        test.expect(same(P, expected) && same_state(output, fixture.state), profile,
                    "exact identity reset clears all yaw cross terms and applies encoded floor at below/equal/above "
                    "boundaries");
        orient(test, profile, fixture, 0.63L, -1.1L);
        auto const transported = oracle(fixture);
        test.expect(fixture.apply(output, P, delta) == Status::success, profile,
                    "strongly tilted dense cross-covariance fixture succeeds");
        compare(test, profile, fixture, transported, output, P, delta);
    }
}

template <typename F> void fail(Context & test, std::string_view profile, F const & fixture, Status expected)
{
    using Scalar = typename F::Scalar;
    for (unsigned aliases = 0; aliases < 4; ++aliases)
    {
        auto current = fixture;
        typename F::State separate;
        auto output_P = F::Matrix::identity() * Scalar{17};
        output_P(0, 1) = std::numeric_limits<Scalar>::quiet_NaN();
        output_P(1, 0) = -Scalar{0};
        typename F::Q delta;
        test.expect(F::Q::from_coefficients(Scalar{0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.125},
                                            separate.q_nb) == Status::success,
                    profile, "distinct valid output sentinel");
        delta = separate.q_nb;
        auto & output = (aliases & 1U) ? current.state : separate;
        auto & P = (aliases & 2U) ? current.P : output_P;
        auto const old = output;
        auto const old_P = P;
        auto const old_delta = delta;
        test.expect(current.apply(output, P, delta) == expected && same_state(output, old) && same(P, old_P) &&
                        same(delta.coefficients(), old_delta.coefficients()) &&
                        same_state(current.state, fixture.state) && same(current.P, fixture.P) &&
                        same(current.sample, fixture.sample) && same(current.field, fixture.field),
                    profile,
                    "specific failure status preserves all three outputs and inputs across four allowed prior aliases");
    }
}

template <typename Linalg, typename Configuration> void failures(Context & test, std::string_view profile)
{
    using F = Fixture<Linalg, Configuration>;
    using Scalar = typename F::Scalar;
    F baseline;
    for (auto invalid : {std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity(),
                         -std::numeric_limits<Scalar>::infinity()})
    {
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            auto bad = baseline;
            bad.sample(axis) = invalid;
            fail(test, profile, bad, Status::non_finite_input);
            bad = baseline;
            bad.field(axis) = invalid;
            fail(test, profile, bad, Status::non_finite_input);
        }
        for (std::size_t row = 0; row < F::size; ++row)
        {
            for (std::size_t column = 0; column < F::size; ++column)
            {
                auto bad = baseline;
                bad.P(row, column) = invalid;
                fail(test, profile, bad, Status::non_finite_input);
            }
        }
        for (auto member : {&F::yaw_variance, &F::minimum_horizontal, &F::minimum_norm})
        {
            auto bad = baseline;
            bad.*member = invalid;
            fail(test, profile, bad, Status::non_finite_input);
        }
        if constexpr (F::ins)
        {
            for (auto member : {&F::State::p_n, &F::State::v_n, &F::State::b_a, &F::State::b_g})
            {
                for (std::size_t axis = 0; axis < 3; ++axis)
                {
                    auto bad = baseline;
                    (bad.state.*member)(axis) = invalid;
                    bad.yaw_variance = Scalar{-1};
                    fail(test, profile, bad, Status::non_finite_input);
                }
            }
        }
    }
    for (std::size_t axis = 0; axis < F::size; ++axis)
    {
        auto bad = baseline;
        bad.P(axis, axis) = Scalar{-1};
        fail(test, profile, bad, Status::domain_error);
    }
    auto bad = baseline;
    bad.P(0, 1) = std::nextafter(bad.P(0, 1), Scalar{1});
    fail(test, profile, bad, Status::domain_error);
    bad = baseline;
    bad.yaw_variance = Scalar{-1};
    fail(test, profile, bad, Status::domain_error);
    for (auto bound : {Scalar{0}, Scalar{-1}, std::nextafter(std::numeric_limits<Scalar>::min(), Scalar{0})})
    {
        bad = baseline;
        bad.minimum_horizontal = bound;
        fail(test, profile, bad, Status::domain_error);
    }
    for (auto bound : {Scalar{0}, Scalar{-1}, std::nextafter(Scalar{1}, Scalar{2})})
    {
        bad = baseline;
        bad.minimum_norm = bound;
        fail(test, profile, bad, Status::domain_error);
    }
    for (bool reference_field : {false, true})
    {
        bad = baseline;
        auto & vector = reference_field ? bad.field : bad.sample;
        vector = F::Vector::from_row_major({Scalar{0}, Scalar{0}, Scalar{40}});
        fail(test, profile, bad, Status::domain_error);
        vector = {};
        fail(test, profile, bad, Status::zero_or_unsafe_divisor);
        vector(0) = std::numeric_limits<Scalar>::min();
        fail(test, profile, bad, Status::zero_or_unsafe_divisor);
        vector(0) = std::numeric_limits<Scalar>::max();
        fail(test, profile, bad, Status::non_finite_result);
    }
    bad = baseline;
    for (std::size_t row = 0; row < F::size; ++row)
    {
        for (std::size_t column = 0; column < F::size; ++column)
        {
            bad.P(row, column) = std::numeric_limits<Scalar>::max();
        }
    }
    bad.sample = F::Vector::from_row_major({Scalar{20}, Scalar{-20}, Scalar{40}});
    fail(test, profile, bad, Status::non_finite_result);
    bad = baseline;
    bad.P = F::Matrix::identity();
    bad.P(F::offset, F::offset + 1) = Scalar{2};
    bad.P(F::offset + 1, F::offset) = Scalar{2};
    bad.sample = F::Vector::from_row_major({Scalar{20}, Scalar{20}, Scalar{0}});
    bad.field = F::Vector::from_row_major({Scalar{20}, Scalar{0}, Scalar{0}});
    fail(test, profile, bad, Status::not_positive_definite);
    for (bool reference_field : {false, true})
    {
        for (int side : {-1, 0, 1})
        {
            auto boundary = baseline;
            Scalar const horizontal = side < 0   ? std::nextafter(Scalar{5}, Scalar{0})
                                      : side > 0 ? std::nextafter(Scalar{5}, Scalar{6})
                                                 : Scalar{5};
            auto & vector = reference_field ? boundary.field : boundary.sample;
            vector = F::Vector::from_row_major({horizontal, Scalar{0}, Scalar{40}});
            if (side < 0)
            {
                fail(test, profile, boundary, Status::domain_error);
            }
            else
            {
                auto output = boundary.state;
                auto P = boundary.P;
                typename F::Q delta;
                test.expect(boundary.apply(output, P, delta) == Status::success, profile,
                            "both measured/reference horizontal norms accept equality and the adjacent larger value");
            }
        }
    }
    auto endpoint = baseline;
    endpoint.minimum_horizontal = std::numeric_limits<Scalar>::min();
    endpoint.minimum_norm = Scalar{1};
    auto endpoint_state = endpoint.state;
    auto endpoint_P = endpoint.P;
    typename F::Q endpoint_delta;
    test.expect(endpoint.apply(endpoint_state, endpoint_P, endpoint_delta) == Status::success, profile,
                "positive-normal horizontal endpoint and normalization bound one are valid for identity prior");
    // The indefinite prior above deliberately violates the PSD success premise;
    // it exercises an explicit negative computed attitude variance guard, not PSD repair.
    Scalar const tiny =
        std::ldexp(Scalar{1}, (std::numeric_limits<Scalar>::min_exponent - std::numeric_limits<Scalar>::digits) / 2);
    bad = baseline;
    test.expect(F::Q::from_coefficients(tiny * Scalar{0.875}, Scalar{0}, Scalar{0}, Scalar{0},
                                        std::numeric_limits<Scalar>::min(), bad.state.q_nb) == Status::success,
                profile, "public shrinking quaternion witness");
    for (unsigned step = 0; step < 32 && bad.state.q_nb.q0() != Scalar{0}; ++step)
    {
        bad.state.q_nb = bad.state.q_nb * bad.state.q_nb;
    }
    fail(test, profile, bad, Status::zero_or_unsafe_divisor);
    test.expect(F::Q::from_coefficients(tiny * Scalar{0.875}, Scalar{0}, Scalar{0}, Scalar{0},
                                        std::numeric_limits<Scalar>::min(), bad.state.q_nb) == Status::success,
                profile, "finite subunit prior survives field geometry before normalization failure");
    bad.minimum_norm = Scalar{1};
    fail(test, profile, bad, Status::invalid_quaternion_norm);
    test.expect(F::Q::from_coefficients(tiny, tiny / Scalar{2}, Scalar{0}, Scalar{0},
                                        std::numeric_limits<Scalar>::min(), bad.state.q_nb) == Status::success,
                profile, "public growing quaternion witness");
    for (unsigned step = 0; step < 32 && formal_eskf::linalg::all_finite(bad.state.q_nb.coefficients()); ++step)
    {
        bad.state.q_nb = bad.state.q_nb * bad.state.q_nb;
    }
    fail(test, profile, bad, Status::non_finite_input);
    // StandardMath internal trig/sqrt failures remain scalar-boundary evidence;
    // no private-state fixture or modified production provider is introduced.
}

template <typename Linalg, typename Configuration> void repeated(Context & test, std::string_view profile)
{
    using F = Fixture<Linalg, Configuration>;
    using Scalar = typename F::Scalar;
    F fixture;
    orient(test, profile, fixture, 0.6L, 1.2L);
    for (unsigned count = 0; count < 2; ++count)
    {
        auto const expected = oracle(fixture);
        auto output = fixture.state;
        auto P = fixture.P;
        typename F::Q delta;
        auto const sample = fixture.sample, field = fixture.field;
        test.expect(fixture.apply(output, P, delta) == Status::success, profile, "repeated public reset succeeds");
        compare(test, profile, fixture, expected, output, P, delta);
        test.expect(same(fixture.sample, sample) && same(fixture.field, field), profile,
                    "reset borrows field data without consuming or changing it");
        if (count == 1)
        {
            test.expect(std::abs(delta.q1()) <= Scalar{64} * std::numeric_limits<Scalar>::epsilon() &&
                            std::abs(delta.q2()) <= Scalar{64} * std::numeric_limits<Scalar>::epsilon() &&
                            std::abs(delta.q3()) <= Scalar{64} * std::numeric_limits<Scalar>::epsilon(),
                        profile,
                        "second heading delta is numerically zero, without asserting covariance bitwise idempotence");
        }
        fixture.state = output;
        fixture.P = P;
    }
}
} /* end namespace */
int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Case;
    using Float = formal_eskf::test::Backend<float>;
    using Double = formal_eskf::test::Backend<double>;
    Case const cases[] = {
        {"TC-R-MAGNETOMETER-RESET-01", "AHRS-float", geometry<Float, Ahrs>},
        {"TC-R-MAGNETOMETER-RESET-01", "AHRS-double", geometry<Double, Ahrs>},
        {"TC-R-MAGNETOMETER-RESET-01", "INS-float", geometry<Float, Ins>},
        {"TC-R-MAGNETOMETER-RESET-01", "INS-double", geometry<Double, Ins>},
        {"TC-R-MAGNETOMETER-RESET-02", "AHRS-float", covariance<Float, Ahrs>},
        {"TC-R-MAGNETOMETER-RESET-02", "AHRS-double", covariance<Double, Ahrs>},
        {"TC-R-MAGNETOMETER-RESET-02", "INS-float", covariance<Float, Ins>},
        {"TC-R-MAGNETOMETER-RESET-02", "INS-double", covariance<Double, Ins>},
        {"TC-R-MAGNETOMETER-RESET-03", "AHRS-float", failures<Float, Ahrs>},
        {"TC-R-MAGNETOMETER-RESET-03", "AHRS-double", failures<Double, Ahrs>},
        {"TC-R-MAGNETOMETER-RESET-03", "INS-float", failures<Float, Ins>},
        {"TC-R-MAGNETOMETER-RESET-03", "INS-double", failures<Double, Ins>},
        {"TC-R-MAGNETOMETER-RESET-04", "AHRS-float", repeated<Float, Ahrs>},
        {"TC-R-MAGNETOMETER-RESET-04", "AHRS-double", repeated<Double, Ahrs>},
        {"TC-R-MAGNETOMETER-RESET-04", "INS-float", repeated<Float, Ins>},
        {"TC-R-MAGNETOMETER-RESET-04", "INS-double", repeated<Double, Ins>},

    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
