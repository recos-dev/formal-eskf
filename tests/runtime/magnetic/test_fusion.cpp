/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Include standalone quality API first to check its own dependencies.
#include <formal_eskf/runtime/magnetic_field.hpp>
#include <formal_eskf/runtime/magnetometer.hpp>

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string_view>
#include <type_traits>

#include "test_backend.hpp"
#include "test_cases.hpp"
#include "correction_reference.hpp"

namespace
{

using formal_eskf::Status;
using formal_eskf::configuration::Ahrs;
using formal_eskf::configuration::Ins;
using formal_eskf::runtime::check_magnetic_field;
using formal_eskf::runtime::FusionDecision;
using TestContext = formal_eskf::test::Context;

template <typename Linalg> using Vector = formal_eskf::linalg::Matrix<Linalg, 3U, 1U>;

template <typename Linalg> auto field_parameters()
{
    using value_type = typename Linalg::value_type;
    return formal_eskf::runtime::MagneticFieldParameters<value_type>{value_type{5}, static_cast<value_type>(0.2),
                                                                     static_cast<value_type>(0.2), value_type{1}};
}

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

template <typename State> bool same_state(State const & a, State const & b)
{
    bool equal = same_bits(a.q_nb.coefficients(), b.q_nb.coefficients());
    if constexpr (requires { a.p_n; })
    {
        equal = equal && same_bits(a.p_n, b.p_n) && same_bits(a.v_n, b.v_n) && same_bits(a.b_a, b.b_a) &&
                same_bits(a.b_g, b.b_g);
    }
    return equal;
}

enum class Health
{
    absent,
    observation,
    separate,
    sample
};

template <typename Linalg, typename Configuration> struct Fixture
{
    using value_type = typename Linalg::value_type;
    using state_type = typename Configuration::template NominalState<Linalg>;
    static constexpr std::size_t size = Configuration::error_state_dimension;
    static constexpr std::size_t measurement_size = 3;
    static constexpr bool is_ins = std::is_same_v<Configuration, Ins>;
    static constexpr std::size_t attitude_offset = is_ins ? 6 : 0;
    using covariance_type = formal_eskf::linalg::Matrix<Linalg, size, size>;
    using noise_type = formal_eskf::linalg::Matrix<Linalg, 3, 3>;
    state_type state{};
    covariance_type P = covariance_type::identity();
    Vector<Linalg> reference = Vector<Linalg>::from_row_major({value_type{30}, value_type{0}, value_type{40}});
    Vector<Linalg> sample = Vector<Linalg>::from_row_major({value_type{30}, value_type{0.5}, value_type{40}});
    Vector<Linalg> health = reference;
    Vector<Linalg> r{};
    formal_eskf::linalg::Matrix<Linalg, 3, size> H{};
    noise_type V = noise_type::identity();
    formal_eskf::runtime::MagneticFieldParameters<value_type> parameters = field_parameters<Linalg>();
    value_type minimum_norm = value_type{0.125};
    value_type sigma = value_type{3};
    bool heading_observable = true;
    Health health_role = Health::observation;
    Ins::BiasUpdate bias_update{};
    bool default_permissions = false;
    formal_eskf::runtime::MagneticFieldChecks checks{};
    Fixture()
    {
        for (std::size_t row = 0; row < size; ++row)
        {
            for (std::size_t column = 0; column < size; ++column)
            {
                P(row, column) += static_cast<value_type>((row + 1) * (column + 1)) / value_type{256};
            }
        }
        for (std::size_t row = 0; row < 3; ++row)
        {
            for (std::size_t column = 0; column < 3; ++column)
            {
                V(row, column) += static_cast<value_type>((row + 1) * (column + 1)) / value_type{16};
            }
        }
    }
    auto fuse(state_type & output, covariance_type & output_P) const
    {
        Vector<Linalg> const * reference_pointer = nullptr;
        switch (health_role)
        {
        case Health::absent:
            break;
        case Health::observation:
            reference_pointer = &reference;
            break;
        case Health::separate:
            reference_pointer = &health;
            break;
        case Health::sample:
            reference_pointer = &sample;
            break;
        }
        if constexpr (is_ins)
        {
            if (default_permissions)
            {
                return formal_eskf::runtime::fuse_magnetometer(state, P, sample, reference, reference_pointer, V,
                                                               parameters, heading_observable, minimum_norm, sigma,
                                                               output, output_P);
            }
            return formal_eskf::runtime::fuse_magnetometer(state, P, sample, reference, reference_pointer, V,
                                                           parameters, heading_observable, minimum_norm, sigma, output,
                                                           output_P, bias_update, checks);
        }
        else
        {
            return formal_eskf::runtime::fuse_magnetometer(state, P, sample, reference, reference_pointer, V,
                                                           parameters, heading_observable, minimum_norm, sigma, output,
                                                           output_P, checks);
        }
    }
    void prepare_reference()
    {
        // Independent unit-quaternion rotation, transposed, followed by explicit
        // skew entries. No production rotation, observation or covariance helper.
        long double const w = state.q_nb.q0(), x = state.q_nb.q1(), y = state.q_nb.q2(), z = state.q_nb.q3();
        std::array<std::array<long double, 3>, 3> const rotation{
            {{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
             {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
             {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}}};
        std::array<long double, 3> h{};
        for (std::size_t row = 0; row < 3; ++row)
        {
            for (std::size_t column = 0; column < 3; ++column)
            {
                h[row] += rotation[column][row] * reference(column);
            }
            r(row) = static_cast<value_type>(sample(row) - h[row]);
        }
        H = {};
        H(0, attitude_offset + 1) = static_cast<value_type>(-h[2]);
        H(0, attitude_offset + 2) = static_cast<value_type>(h[1]);
        H(1, attitude_offset) = static_cast<value_type>(h[2]);
        H(1, attitude_offset + 2) = static_cast<value_type>(-h[0]);
        H(2, attitude_offset) = static_cast<value_type>(-h[1]);
        H(2, attitude_offset + 1) = static_cast<value_type>(h[0]);
    }
};

template <typename Linalg> auto non_finite_quaternion(TestContext & test, std::string_view profile)
{
    using Scalar = typename Linalg::value_type;
    using Q = formal_eskf::so3::UnitQuaternion<Linalg>;
    Scalar const tiny =
        std::ldexp(Scalar{1}, (std::numeric_limits<Scalar>::min_exponent - std::numeric_limits<Scalar>::digits) / 2);
    Q q;
    test.expect(Q::from_coefficients(tiny, tiny / Scalar{2}, Scalar{0}, Scalar{0}, std::numeric_limits<Scalar>::min(),
                                     q) == Status::success,
                profile, "public gradual-underflow quaternion witness avoids private layout assumptions");
    for (unsigned step = 0; step < 32 && formal_eskf::linalg::all_finite(q.coefficients()); ++step)
    {
        q = q * q;
    }
    test.expect(!formal_eskf::linalg::all_finite(q.coefficients()), profile,
                "public raw products reach a nonfinite quaternion input guard");
    return q;
}

template <typename F>
void failure(TestContext & test, std::string_view profile, F const & fixture, FusionDecision decision, Status status,
             bool field_valid, bool field_accepted, bool gate_valid)
{
    using Scalar = typename F::value_type;
    for (unsigned aliases = 0; aliases < 4; ++aliases)
    {
        auto current = fixture;
        typename F::state_type output;
        auto output_P = F::covariance_type::identity() * Scalar{-2};
        output_P(0, 1) = std::numeric_limits<Scalar>::quiet_NaN();
        output_P(1, 0) = -Scalar{0};
        auto & state_output = (aliases & 1U) ? current.state : output;
        auto & covariance_output = (aliases & 2U) ? current.P : output_P;
        auto const previous_state = state_output;
        auto const previous_P = covariance_output;
        auto const result = current.fuse(state_output, covariance_output);
        test.expect(result.fusion.status == status && result.fusion.decision == decision &&
                        result.field.diagnostics_valid == field_valid && result.field.accepted == field_accepted &&
                        result.fusion.diagnostics_valid == gate_valid && same_state(state_output, previous_state) &&
                        same_bits(covariance_output, previous_P) && same_state(current.state, fixture.state) &&
                        same_bits(current.P, fixture.P) && same_bits(current.V, fixture.V) &&
                        same_bits(current.sample, fixture.sample) && same_bits(current.reference, fixture.reference) &&
                        same_bits(current.health, fixture.health),
                    profile, "exact stage outcome preserves all output aliases and borrowed numerical inputs");
        if (!gate_valid)
        {
            test.expect(result.fusion.innovation(0) == Scalar{0} && result.fusion.innovation(1) == Scalar{0} &&
                            result.fusion.innovation(2) == Scalar{0} &&
                            result.fusion.innovation_variance(0) == Scalar{0} &&
                            result.fusion.innovation_variance(1) == Scalar{0} &&
                            result.fusion.innovation_variance(2) == Scalar{0},
                        profile, "unreached gate publishes fresh invalid zero diagnostics");
        }
    }
}

template <typename Linalg, typename Configuration> void stages(TestContext & test, std::string_view profile)
{
    using F = Fixture<Linalg, Configuration>;
    using Scalar = typename Linalg::value_type;
    F fixture;
    auto bad = fixture;
    bad.sample(1) = std::numeric_limits<Scalar>::infinity();
    failure(test, profile, bad, FusionDecision::failed, Status::non_finite_input, false, false, false);
    for (bool missing : {false, true})
    {
        bad = fixture;
        bad.sample = fixture.reference * Scalar{2};
        if (missing)
        {
            bad.sample = fixture.reference;
            bad.health_role = Health::absent;
            bad.checks.require_reference = true;
        }
        bad.V(2, 2) = std::numeric_limits<Scalar>::quiet_NaN();
        bad.P(0, 0) = Scalar{-1};
        bad.sigma = Scalar{0};
        failure(test, profile, bad, FusionDecision::rejected, Status::success, true, false, false);
    }
    bad = fixture;
    bad.heading_observable = false;
    bad.P = {};
    bad.sample = Vector<Linalg>::from_row_major({Scalar{0}, Scalar{30}, Scalar{40}});
    failure(test, profile, bad, FusionDecision::rejected, Status::success, true, true, true);
    if constexpr (F::is_ins)
    {
        for (auto member : {&F::state_type::p_n, &F::state_type::v_n, &F::state_type::b_a, &F::state_type::b_g})
        {
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                bad = fixture;
                (bad.state.*member)(axis) = std::numeric_limits<Scalar>::quiet_NaN();
                bad.sample = {};
                failure(test, profile, bad, FusionDecision::failed, Status::non_finite_input, false, false, false);
            }
        }
    }
    bad = fixture;
    bad.state.q_nb = non_finite_quaternion<Linalg>(test, profile);
    bad.sample = {};
    bad.sigma = Scalar{0};
    failure(test, profile, bad, FusionDecision::failed, Status::non_finite_input, false, false, false);
    // Exact two-axis innovation equality, then one represented axis outside.
    fixture.P = {};
    fixture.reference = Vector<Linalg>::from_row_major({Scalar{3}, Scalar{4}, Scalar{0}});
    fixture.sample = fixture.reference * Scalar{2};
    fixture.V = F::noise_type::identity();
    fixture.V(0, 0) = Scalar{9};
    fixture.V(1, 1) = Scalar{16};
    fixture.sigma = Scalar{1};
    auto output = fixture.state;
    auto P = fixture.P;
    auto result = fixture.fuse(output, P);
    test.expect(result.fusion.decision == FusionDecision::fused && result.fusion.test_ratio(0) == Scalar{1} &&
                    result.fusion.test_ratio(1) == Scalar{1},
                profile, "physical-field and two innovation equalities pass");
    fixture.V(0, 0) = std::nextafter(Scalar{9}, Scalar{0});
    failure(test, profile, fixture, FusionDecision::rejected, Status::success, true, true, true);
}

template <typename Linalg, typename Configuration> void correction(TestContext & test, std::string_view profile)
{
    using F = Fixture<Linalg, Configuration>;
    using Scalar = typename Linalg::value_type;
    F fixture;
    test.expect(decltype(fixture.state.q_nb)::from_coefficients(Scalar{0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.5},
                                                                fixture.minimum_norm,
                                                                fixture.state.q_nb) == Status::success,
                profile, "exact nonzero-tilt cyclic prior");
    fixture.sample = Vector<Linalg>::from_row_major({Scalar{0.5}, Scalar{40.25}, Scalar{30}});
    std::array<unsigned, 14> const masks{63, 0, 1, 2, 4, 8, 16, 32, 62, 61, 59, 55, 47, 31};
    for (bool opposite : {false, true})
    {
        auto signed_fixture = fixture;
        if (opposite)
        {
            signed_fixture.state.q_nb = -signed_fixture.state.q_nb;
        }
        for (std::size_t mask_index = 0; mask_index < (F::is_ins ? masks.size() + 1U : 1U); ++mask_index)
        {
            auto expected_fixture = signed_fixture;
            unsigned const mask = mask_index == masks.size() ? 63U : masks[mask_index];
            expected_fixture.default_permissions = mask_index == masks.size();
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                expected_fixture.bias_update.accelerometer[axis] = (mask & (1U << axis)) != 0;
                expected_fixture.bias_update.gyroscope[axis] = (mask & (1U << (axis + 3))) != 0;
            }
            expected_fixture.prepare_reference();
            auto const expected = formal_eskf::test::reference::reference_correction(
                expected_fixture, ESKF_RESET_APPROX != 0, expected_fixture.bias_update, ESKF_MAG_TILT != 0);
            auto const H = formal_eskf::test::reference::copied(expected_fixture.H);
            auto const HP =
                formal_eskf::test::reference::product(H, formal_eskf::test::reference::copied(expected_fixture.P));
            auto const HPHt = formal_eskf::test::reference::product(HP, formal_eskf::test::reference::transposed(H));
            for (unsigned aliases = 0; aliases < 4; ++aliases)
            {
                auto current = expected_fixture;
                typename F::state_type output;
                auto output_P = F::covariance_type::identity() * Scalar{-2};
                auto & state_output = (aliases & 1U) ? current.state : output;
                auto & covariance_output = (aliases & 2U) ? current.P : output_P;
                auto const result = current.fuse(state_output, covariance_output);
                test.expect(result.fusion.decision == FusionDecision::fused &&
                                result.fusion.status == Status::success && result.field.accepted &&
                                result.fusion.diagnostics_valid &&
                                formal_eskf::test::reference::matches_reference(
                                    expected_fixture, expected, state_output, covariance_output,
                                    Scalar{512} * std::numeric_limits<Scalar>::epsilon()),
                            profile,
                            "independent projected/full gain, Joseph and reset oracle covers both signs, aliases and "
                            "forwarding masks");
                for (std::size_t axis = 0; axis < 3; ++axis)
                {
                    long double const variance = HPHt[axis][axis] + expected_fixture.V(axis, axis);
                    long double const r = expected_fixture.r(axis);
                    test.expect(std::abs(static_cast<long double>(result.fusion.innovation_variance(axis)) -
                                         variance) <= 32 * std::numeric_limits<Scalar>::epsilon() * variance &&
                                    result.fusion.innovation(axis) == expected_fixture.r(axis) &&
                                    std::abs(static_cast<long double>(result.fusion.test_ratio(axis)) -
                                             r * r / (9 * variance)) <= 32 * std::numeric_limits<Scalar>::epsilon(),
                                profile,
                                "gate always uses unprojected full H/P/V independent of correction permissions");
                }
                test.expect(((aliases & 1U) != 0 || same_state(current.state, expected_fixture.state)) &&
                                ((aliases & 2U) != 0 || same_bits(current.P, expected_fixture.P)) &&
                                same_bits(current.sample, expected_fixture.sample) &&
                                same_bits(current.reference, expected_fixture.reference) &&
                                same_bits(current.V, expected_fixture.V),
                            profile, "disjoint prior and observations remain unchanged");
            }
        }
    }
    // Exact diagonal control supplements the bounded nonlinear regression oracle.
    F exact;
    exact.reference = Vector<Linalg>::from_row_major({Scalar{8}, Scalar{0}, Scalar{0}});
    exact.sample = exact.reference;
    exact.P = F::covariance_type::identity();
    exact.V = F::noise_type::identity();
    exact.V(0, 0) = Scalar{256};
    exact.V(1, 1) = Scalar{192};
    exact.V(2, 2) = Scalar{192};
    auto output = exact.state;
    auto P = exact.P;
    auto const result = exact.fuse(output, P);
    auto expected = exact.P;
    expected(F::attitude_offset + 2, F::attitude_offset + 2) = Scalar{0.75};
    if constexpr (ESKF_MAG_TILT == 0)
    {
        expected(F::attitude_offset + 1, F::attitude_offset + 1) = Scalar{0.75};
    }
    test.expect(result.fusion.decision == FusionDecision::fused && same_state(output, exact.state) &&
                    same_bits(P, expected),
                profile, "exact S=256 dyadic zero-residual control verifies yaw-only versus full posterior covariance");
}

template <typename Linalg, typename Configuration> void failures(TestContext & test, std::string_view profile)
{
    using F = Fixture<Linalg, Configuration>;
    using Scalar = typename Linalg::value_type;
    F fixture;
    for (auto role : {Health::absent, Health::observation, Health::separate, Health::sample})
    {
        fixture.health_role = role;
        auto bad = fixture;
        bad.V(0, 0) = std::numeric_limits<Scalar>::quiet_NaN();
        failure(test, profile, bad, FusionDecision::failed, Status::non_finite_input, true, true, false);
        bad = fixture;
        bad.sigma = Scalar{0};
        failure(test, profile, bad, FusionDecision::failed, Status::domain_error, true, true, false);
        bad = fixture;
        bad.minimum_norm = Scalar{2};
        failure(test, profile, bad, FusionDecision::failed, Status::domain_error, true, true, false);
        bad = fixture;
        bad.V(0, 0) = Scalar{-1};
        failure(test, profile, bad, FusionDecision::failed, Status::domain_error, true, true, false);
        bad = fixture;
        bad.P = {};
        bad.V = F::noise_type::identity();
        bad.V(0, 1) = Scalar{2};
        bad.V(1, 0) = Scalar{2};
        failure(test, profile, bad, FusionDecision::failed, Status::not_positive_definite, true, true, true);
        bad = fixture;
        Scalar const tiny = std::ldexp(
            Scalar{1}, (std::numeric_limits<Scalar>::min_exponent - std::numeric_limits<Scalar>::digits) / 2);
        test.expect(decltype(bad.state.q_nb)::from_coefficients(tiny * Scalar{0.875}, Scalar{0}, Scalar{0}, Scalar{0},
                                                                std::numeric_limits<Scalar>::min(),
                                                                bad.state.q_nb) == Status::success &&
                        bad.state.q_nb.q0() == Scalar{0.875},
                    profile, "public subunit scalar quaternion fixture");
        bad.minimum_norm = Scalar{1};
        bad.reference = Vector<Linalg>::from_row_major({Scalar{8}, Scalar{0}, Scalar{0}});
        bad.sample = bad.reference / Scalar{0.765625};
        bad.checks = {false, false, false};
        bad.V = F::noise_type::identity() * Scalar{1024};
        failure(test, profile, bad, FusionDecision::failed, Status::invalid_quaternion_norm, true, true, true);
        auto output = bad.state;
        auto P = bad.P;
        auto const actual = bad.fuse(output, P);
        test.expect(actual.field.strength > Scalar{0} && actual.fusion.innovation(0) > Scalar{0} &&
                        actual.fusion.innovation_variance(0) == Scalar{1024},
                    profile, "late normalization failure retains meaningful field and original gate evidence");
    }
    auto bad = fixture;
    bad.health_role = Health::absent;
    bad.checks = {false, false, false};
    Scalar const tiny =
        std::ldexp(Scalar{1}, (std::numeric_limits<Scalar>::min_exponent - std::numeric_limits<Scalar>::digits) / 2);
    test.expect(decltype(bad.state.q_nb)::from_coefficients(tiny * Scalar{1.0625}, Scalar{0}, Scalar{0}, Scalar{0},
                                                            std::numeric_limits<Scalar>::min(),
                                                            bad.state.q_nb) == Status::success,
                profile, "public growing scalar quaternion witness");
    for (unsigned step = 0; step < 32; ++step)
    {
        auto const next = bad.state.q_nb * bad.state.q_nb;
        if (!std::isfinite(next.q0() * next.q0()))
        {
            break;
        }
        bad.state.q_nb = next;
    }
    Scalar const scale = bad.state.q_nb.q0() * bad.state.q_nb.q0();
    Scalar const reference_scale = std::sqrt(std::numeric_limits<Scalar>::max()) / Scalar{4};
    test.expect(std::isfinite(scale) && scale > std::sqrt(std::numeric_limits<Scalar>::max()) * Scalar{8}, profile,
                "finite rotation coefficient can overflow inverse action while tiny measured field remains finite");
    bad.sample = Vector<Linalg>::from_row_major({Scalar{32} / scale, Scalar{0}, Scalar{0}});
    bad.reference = Vector<Linalg>::from_row_major({reference_scale, Scalar{0}, Scalar{0}});
    bad.heading_observable = false;
    failure(test, profile, bad, FusionDecision::failed, Status::non_finite_result, true, true, false);
    // The finite malformed witness is outside the unit-prior success premise;
    // it verifies that inverse-observation overflow cannot publish a partial update.
}

template <typename Linalg, typename Configuration> void health_roles(TestContext & test, std::string_view profile)
{
    using F = Fixture<Linalg, Configuration>;
    using Scalar = typename Linalg::value_type;
    F fixture;
    fixture.health = Vector<Linalg>::from_row_major({Scalar{0}, Scalar{30}, Scalar{40}});
    fixture.prepare_reference();
    auto const expected = formal_eskf::test::reference::reference_correction(fixture, ESKF_RESET_APPROX != 0,
                                                                             fixture.bias_update, ESKF_MAG_TILT != 0);
    for (auto role : {Health::absent, Health::observation, Health::separate, Health::sample})
    {
        fixture.health_role = role;
        auto output = fixture.state;
        auto P = fixture.P;
        auto const result = fixture.fuse(output, P);
        test.expect(result.fusion.decision == FusionDecision::fused && result.field.heading_checked &&
                        result.fusion.innovation(1) == Scalar{0.5} &&
                        formal_eskf::test::reference::matches_reference(
                            fixture, expected, output, P, Scalar{512} * std::numeric_limits<Scalar>::epsilon()),
                    profile,
                    "accepted independent/absent/shared/aliased health data never changes the observation correction");
    }
    fixture.health_role = Health::separate;
    fixture.health = fixture.health * Scalar{2};
    failure(test, profile, fixture, FusionDecision::rejected, Status::success, true, false, false);
    fixture.health = Vector<Linalg>::from_row_major({Scalar{0}, Scalar{40}, Scalar{30}});
    failure(test, profile, fixture, FusionDecision::rejected, Status::success, true, false, false);
    fixture.health(0) = std::numeric_limits<Scalar>::quiet_NaN();
    failure(test, profile, fixture, FusionDecision::failed, Status::non_finite_input, false, false, false);
    fixture.checks = {false, false, false};
    auto output = fixture.state;
    auto P = fixture.P;
    test.expect(fixture.fuse(output, P).fusion.decision == FusionDecision::fused, profile,
                "disabled physical checks ignore invalid unused health vector");
    fixture.checks = {true, false, false};
    fixture.health = Vector<Linalg>::from_row_major({Scalar{0}, Scalar{0}, Scalar{50}});
    test.expect(fixture.fuse(output, P).fusion.decision == FusionDecision::fused, profile,
                "strength-only health reference need not have horizontal information");
}
} /* end namespace */
int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Case;
    using Float = formal_eskf::test::Backend<float>;
    using Double = formal_eskf::test::Backend<double>;
    Case const cases[] = {
        {"TC-R-MAGNETOMETER-FUSION-01", "AHRS-float", stages<Float, Ahrs>},
        {"TC-R-MAGNETOMETER-FUSION-01", "AHRS-double", stages<Double, Ahrs>},
        {"TC-R-MAGNETOMETER-FUSION-01", "INS-float", stages<Float, Ins>},
        {"TC-R-MAGNETOMETER-FUSION-01", "INS-double", stages<Double, Ins>},
        {"TC-R-MAGNETOMETER-FUSION-02", "AHRS-float", correction<Float, Ahrs>},
        {"TC-R-MAGNETOMETER-FUSION-02", "AHRS-double", correction<Double, Ahrs>},
        {"TC-R-MAGNETOMETER-FUSION-02", "INS-float", correction<Float, Ins>},
        {"TC-R-MAGNETOMETER-FUSION-02", "INS-double", correction<Double, Ins>},
        {"TC-R-MAGNETOMETER-FUSION-03", "AHRS-float", failures<Float, Ahrs>},
        {"TC-R-MAGNETOMETER-FUSION-03", "AHRS-double", failures<Double, Ahrs>},
        {"TC-R-MAGNETOMETER-FUSION-03", "INS-float", failures<Float, Ins>},
        {"TC-R-MAGNETOMETER-FUSION-03", "INS-double", failures<Double, Ins>},
        {"TC-R-MAGNETOMETER-FUSION-04", "AHRS-float", health_roles<Float, Ahrs>},
        {"TC-R-MAGNETOMETER-FUSION-04", "AHRS-double", health_roles<Double, Ahrs>},
        {"TC-R-MAGNETOMETER-FUSION-04", "INS-float", health_roles<Float, Ins>},
        {"TC-R-MAGNETOMETER-FUSION-04", "INS-double", health_roles<Double, Ins>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
