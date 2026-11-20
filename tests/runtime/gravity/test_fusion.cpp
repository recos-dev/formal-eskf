/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/accelerometer.hpp>

#include <array>
#include <bit>
#include <cstddef>
#include <cmath>
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
using formal_eskf::runtime::FusionDecision;
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

template <typename Linalg, typename Configuration> struct FusionFixture
{
    using value_type = typename Linalg::value_type;
    using state_type = typename Configuration::template NominalState<Linalg>;
    using covariance_type =
        formal_eskf::linalg::Matrix<Linalg, Configuration::error_state_dimension, Configuration::error_state_dimension>;
    using vector_type = formal_eskf::linalg::Matrix<Linalg, 3U, 1U>;
    using noise_type = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    using magnitude_type = formal_eskf::runtime::AccelerometerMagnitude<Linalg>;
    static constexpr bool is_ins = std::is_same_v<Configuration, Ins>;
    static constexpr std::size_t size = Configuration::error_state_dimension;
    static constexpr std::size_t measurement_size = 3U;
    static constexpr std::size_t attitude_offset = is_ins ? 6U : 0U;
    formal_eskf::linalg::Matrix<Linalg, 3U, size> H{};
    vector_type r{};

    state_type state{};
    covariance_type P = covariance_type::identity();
    vector_type gravity = vector_type::from_row_major({value_type{0}, value_type{0}, value_type{10}});
    vector_type sample = -gravity;
    noise_type V = noise_type::identity();
    formal_eskf::runtime::AccelerometerFusionConditions conditions{true, true, false, false, {}};
    magnitude_type monitor{};
    typename magnitude_type::Result magnitude{};
    value_type minimum_norm = static_cast<value_type>(1e-6);
    Ins::BiasUpdate bias_update{};

    FusionFixture()
    {
        // Exact cyclic unit rotation: body Y maps to navigation Down.
        (void)state_type::quaternion_type::from_coefficients(value_type{0.5}, value_type{0.5}, value_type{0.5},
                                                             value_type{0.5}, value_type{0.125}, state.q_nb);
        sample = vector_type::from_row_major({value_type{0}, value_type{-10}, value_type{0}});
        if constexpr (std::is_same_v<Configuration, Ins>)
        {
            state.p_n = vector_type::from_row_major({value_type{1}, value_type{2}, value_type{3}});
            state.v_n = vector_type::from_row_major({value_type{4}, value_type{5}, value_type{6}});
            state.b_a = vector_type::from_row_major({value_type{0.125}, value_type{-0.25}, value_type{0.125}});
            state.b_g = vector_type::from_row_major({value_type{0.125}, value_type{0.25}, value_type{-0.125}});
            sample = sample + state.b_a;
        }
        magnitude = monitor.update(sample);
        sample = sample + vector_type::from_row_major({value_type{0.25}, value_type{-0.125}, value_type{0}});
        magnitude = monitor.update(sample);
        // Positive-definite matrices with nonzero state cross-covariance and correlated V.
        for (std::size_t row = 0U; row < P.row_count; ++row)
        {
            for (std::size_t column = 0U; column < P.column_count; ++column)
            {
                P(row, column) += static_cast<value_type>((row + 1U) * (column + 1U)) / value_type{256};
            }
        }
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            for (std::size_t column = 0U; column < 3U; ++column)
            {
                V(row, column) += static_cast<value_type>((row + 1U) * (column + 1U)) / value_type{16};
            }
        }
        prepare_reference();
    }

    auto fuse(state_type & output, covariance_type & output_P) const
    {
        if constexpr (std::is_same_v<Configuration, Ins>)
        {
            return formal_eskf::runtime::fuse_accelerometer(state, P, sample, gravity, V, conditions, magnitude,
                                                            minimum_norm, output, output_P, bias_update);
        }
        else
        {
            return formal_eskf::runtime::fuse_accelerometer(state, P, sample, gravity, V, conditions, magnitude,
                                                            minimum_norm, output, output_P);
        }
    }

    void prepare_reference()
    {
        auto const q = state.q_nb.coefficients();
        long double const a = q(0U), x = q(1U), y = q(2U), z = q(3U);
        long double const rotation[3][3] = {{a * a + x * x - y * y - z * z, 2 * (x * y - a * z), 2 * (x * z + a * y)},
                                            {2 * (x * y + a * z), a * a - x * x + y * y - z * z, 2 * (y * z - a * x)},
                                            {2 * (x * z - a * y), 2 * (y * z + a * x), a * a - x * x - y * y + z * z}};
        vector_type h;
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            long double prediction = 0;
            for (std::size_t component = 0; component < 3; ++component)
            {
                prediction -= rotation[component][axis] * gravity(component);
            }
            h(axis) = static_cast<value_type>(prediction);
            r(axis) = sample(axis) - h(axis);
            if constexpr (is_ins)
            {
                r(axis) -= state.b_a(axis);
            }
        }
        H = {};
        H(0U, attitude_offset + 1U) = -h(2U);
        H(0U, attitude_offset + 2U) = h(1U);
        H(1U, attitude_offset) = h(2U);
        H(1U, attitude_offset + 2U) = -h(0U);
        H(2U, attitude_offset) = -h(1U);
        H(2U, attitude_offset + 1U) = h(0U);
    }
};

template <typename Linalg, typename Configuration> void test_correction(TestContext & test, std::string_view profile)
{
    using fixture_type = FusionFixture<Linalg, Configuration>;
    fixture_type fixture;
    using value_type = typename fixture_type::value_type;
    auto const q = fixture.state.q_nb.coefficients();
    test.expect(q(0U) == value_type{0.5} && q(1U) == value_type{0.5} && q(2U) == value_type{0.5} &&
                    q(3U) == value_type{0.5},
                profile, "nonidentity fixture is the intended exact cyclic rotation");
    test.expect(fixture.magnitude.within_gravity_band &&
                    !same_bits(fixture.sample, fixture.monitor.filtered_specific_force_b()),
                profile, "real LPF result permits a distinct current-sample correction");
    constexpr std::array<unsigned, 14U> masks{0U, 63U, 1U, 2U, 4U, 8U, 16U, 32U, 62U, 61U, 59U, 55U, 47U, 31U};
    for (unsigned mask : masks)
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            fixture.bias_update.accelerometer[axis] = (mask & (1U << axis)) != 0U;
            fixture.bias_update.gyroscope[axis] = (mask & (1U << (axis + 3U))) != 0U;
        }
        if constexpr (!fixture_type::is_ins)
        {
            if (mask != 0U)
            {
                continue;
            }
        }
        auto const reference =
            formal_eskf::test::reference::reference_correction(fixture, ESKF_RESET_APPROX != 0, fixture.bias_update);
        // Supplemental finite-fixture regression against long-double real algebra.
        // This is not a generally approved nonlinear/libm forward-error budget.
        value_type const tolerance = value_type{128} * std::numeric_limits<value_type>::epsilon();
        test.expect(reference.valid, profile, "independent elimination and Joseph/reset reference succeeds");
        for (unsigned alias = 0U; alias < 4U; ++alias)
        {
            auto current = fixture;
            typename fixture_type::state_type separate;
            auto separate_P = fixture_type::covariance_type::identity();
            auto const old_separate = separate;
            auto const old_separate_P = separate_P;
            bool const state_alias = (alias & 1U) != 0U;
            bool const covariance_alias = (alias & 2U) != 0U;
            auto & output = state_alias ? current.state : separate;
            auto & output_P = covariance_alias ? current.P : separate_P;
            auto const result = current.fuse(output, output_P);
            test.expect(
                result.decision == FusionDecision::fused && result.status == Status::success &&
                    result.conditions.active && result.conditions.sample_eligible &&
                    formal_eskf::test::reference::matches_reference(fixture, reference, output, output_P, tolerance),
                profile, "all aliases and bias masks match independent raw-measurement correction reference");
            test.expect(!same_bits(output.q_nb.coefficients(), fixture.state.q_nb.coefficients()) &&
                            !same_bits(output_P, fixture.P),
                        profile, "nonzero residual changes actual attitude and covariance");
            if constexpr (fixture_type::is_ins)
            {
                for (std::size_t axis = 0; axis < 3; ++axis)
                {
                    test.expect(
                        (output.b_a(axis) != fixture.state.b_a(axis)) == fixture.bias_update.accelerometer[axis] &&
                            (output.b_g(axis) != fixture.state.b_g(axis)) == fixture.bias_update.gyroscope[axis],
                        profile, "dense cross-covariance exercises enabled bias rows; disabled rows held");
                }
            }
            test.expect(
                same_state(state_alias ? separate : current.state, state_alias ? old_separate : fixture.state) &&
                    same_bits(covariance_alias ? separate_P : current.P, covariance_alias ? old_separate_P : fixture.P),
                profile, "separate priors and unused output objects are unchanged");
        }
    }
    typename fixture_type::state_type output;
    typename fixture_type::covariance_type output_P;
    auto const result = formal_eskf::runtime::fuse_accelerometer(
        fixture.state, fixture.P, fixture.sample, fixture.gravity, fixture.V, fixture.conditions, fixture.magnitude,
        fixture.minimum_norm, output, output_P);
    fixture.bias_update = {};
    auto const reference = formal_eskf::test::reference::reference_correction(fixture, ESKF_RESET_APPROX != 0);
    test.expect(result.decision == FusionDecision::fused &&
                    formal_eskf::test::reference::matches_reference(fixture, reference, output, output_P,
                                                                    value_type{128} *
                                                                        std::numeric_limits<value_type>::epsilon()),
                profile, "omitted permissions retain all-enabled behavior");
}

template <typename Fixture>
void expect_preserved(TestContext & test, std::string_view profile, Fixture const & fixture, FusionDecision decision,
                      Status status)
{
    using value_type = typename Fixture::value_type;
    for (unsigned alias = 0U; alias < 4U; ++alias)
    {
        auto current = fixture;
        typename Fixture::state_type separate;
        auto separate_P = Fixture::covariance_type::identity();
        separate_P(0U, 0U) = std::numeric_limits<value_type>::quiet_NaN();
        separate_P(0U, 1U) = -value_type{0};
        auto const old_separate = separate;
        auto const old_separate_P = separate_P;
        auto const result =
            current.fuse((alias & 1U) != 0U ? current.state : separate, (alias & 2U) != 0U ? current.P : separate_P);
        test.expect(result.decision == decision && result.status == status, profile,
                    "rejection and numerical failure remain distinct and propagate status");
        test.expect(same_state(current.state, fixture.state) && same_bits(current.P, fixture.P) &&
                        same_state(separate, old_separate) && same_bits(separate_P, old_separate_P),
                    profile, "all alias arrangements preserve outputs, including NaN and signed zero sentinels");
    }
}

template <typename Linalg, typename Configuration> void test_conditions(TestContext & test, std::string_view profile)
{
    using fixture_type = FusionFixture<Linalg, Configuration>;
    using value_type = typename fixture_type::value_type;
    fixture_type const baseline;
    for (unsigned reason = 0U; reason < 7U; ++reason)
    {
        auto fixture = baseline;
        switch (reason)
        {
        case 0U:
            fixture.conditions.enabled = false;
            break;
        case 1U:
            fixture.conditions.tilt_aligned = false;
            break;
        case 2U:
            fixture.conditions.horizontal_aiding = true;
            break;
        case 3U:
            fixture.sample = -fixture.gravity * value_type{2};
            fixture.monitor.reset();
            fixture.magnitude = fixture.monitor.update(fixture.sample);
            break;
        default:
            fixture.conditions.clipped[reason - 4U] = true;
            break;
        }
        expect_preserved(test, profile, fixture, FusionDecision::rejected, Status::success);
    }
    // No innovation gate is introduced: a large residual can pass the magnitude conditions.
    auto fixture = baseline;
    fixture.sample = fixture_type::vector_type::from_row_major({fixture.gravity(2U), value_type{0}, value_type{0}});
    fixture.monitor.reset();
    fixture.magnitude = fixture.monitor.update(fixture.sample);
    typename fixture_type::state_type output;
    typename fixture_type::covariance_type output_P;
    auto result = fixture.fuse(output, output_P);
    test.expect(result.decision == FusionDecision::fused, profile, "no implicit residual gate or vector normalization");

    fixture = baseline;
    fixture.sample = -fixture.gravity * value_type{2};
    fixture.monitor.reset();
    fixture.magnitude = fixture.monitor.update(fixture.sample);
    fixture.conditions.at_rest = true;
    result = fixture.fuse(output, output_P);
    test.expect(!fixture.magnitude.within_gravity_band && result.decision == FusionDecision::fused, profile,
                "caller rest bypasses only the magnitude band");
    fixture.conditions.clipped[0U] = true;
    result = fixture.fuse(output, output_P);
    test.expect(result.conditions.active && !result.conditions.sample_eligible &&
                    result.decision == FusionDecision::rejected,
                profile, "clipping rejects the sample without clearing active mode");
    fixture.conditions.clipped[0U] = false;
    test.expect(fixture.fuse(output, output_P).decision == FusionDecision::fused, profile,
                "a rejected call creates no hidden latch or recovery timer");
}

template <typename Linalg, typename Configuration> void test_failures(TestContext & test, std::string_view profile)
{
    using fixture_type = FusionFixture<Linalg, Configuration>;
    using value_type = typename fixture_type::value_type;
    using limits_type = std::numeric_limits<value_type>;
    fixture_type const baseline;
    auto fixture = baseline;
    fixture.conditions.at_rest = true;
    fixture.conditions.enabled = false;
    fixture.sample(0U) = limits_type::quiet_NaN();
    fixture.magnitude = fixture.monitor.update(fixture.sample);
    expect_preserved(test, profile, fixture, FusionDecision::failed, Status::non_finite_input);
    fixture = baseline;
    fixture.sample(0U) = limits_type::max();
    fixture.magnitude = fixture.monitor.update(fixture.sample);
    expect_preserved(test, profile, fixture, FusionDecision::failed, Status::non_finite_result);

    fixture = baseline;
    fixture.V(0U, 0U) = limits_type::quiet_NaN();
    expect_preserved(test, profile, fixture, FusionDecision::failed, Status::non_finite_input);
    fixture.conditions.enabled = false;
    expect_preserved(test, profile, fixture, FusionDecision::rejected, Status::success);
    fixture = baseline;
    fixture.gravity = {};
    expect_preserved(test, profile, fixture, FusionDecision::failed, Status::domain_error);
    fixture = baseline;
    fixture.minimum_norm = value_type{0};
    expect_preserved(test, profile, fixture, FusionDecision::failed, Status::domain_error);
    fixture = baseline;
    fixture.V(0U, 1U) = value_type{1024};
    fixture.V(1U, 0U) = value_type{1024};
    expect_preserved(test, profile, fixture, FusionDecision::failed, Status::not_positive_definite);
    fixture = baseline;
    fixture.gravity(2U) = limits_type::max();
    expect_preserved(test, profile, fixture, FusionDecision::failed, Status::non_finite_result);
    fixture = baseline;
    fixture.P(0U, 0U) = limits_type::infinity();
    expect_preserved(test, profile, fixture, FusionDecision::failed, Status::non_finite_input);
    fixture = baseline;
    // A public gradual-underflow normalization yields scalar 7/8; repeated raw
    // products approach zero. No private representation or object-layout writes.
    using quaternion_type = typename fixture_type::state_type::quaternion_type;
    value_type const tiny = std::ldexp(value_type{1}, (limits_type::min_exponent - limits_type::digits) / 2);
    test.expect(quaternion_type::from_coefficients(tiny * value_type{0.875}, value_type{0}, value_type{0},
                                                   value_type{0}, limits_type::min(),
                                                   fixture.state.q_nb) == Status::success,
                profile, "public normalization constructs underflow witness");
    for (unsigned count = 0; count < 32 && fixture.state.q_nb.coefficients()(0U) != value_type{0}; ++count)
    {
        fixture.state.q_nb = fixture.state.q_nb * fixture.state.q_nb;
    }
    test.expect(fixture.state.q_nb.coefficients()(0U) == value_type{0}, profile,
                "public products reach zero quaternion");
    typename fixture_type::state_type output;
    typename fixture_type::covariance_type output_P;
    auto const result = fixture.fuse(output, output_P);
    test.expect(result.conditions.sample_eligible && result.decision == FusionDecision::failed &&
                    result.status == Status::invalid_quaternion_norm,
                profile, "eligibility remains diagnostic, not success, after late correction failure");
    expect_preserved(test, profile, fixture, FusionDecision::failed, Status::invalid_quaternion_norm);
}

template <typename Linalg, typename Configuration> void condition_first(TestContext & test, std::string_view profile)
{
    test_conditions<Linalg, Configuration>(test, profile);
    using Fixture = FusionFixture<Linalg, Configuration>;
    using Scalar = typename Linalg::value_type;
    Fixture fixture;
    fixture.V(0U, 0U) = std::numeric_limits<Scalar>::quiet_NaN();
    fixture.P(0U, 0U) = std::numeric_limits<Scalar>::infinity();
    fixture.minimum_norm = Scalar{-1};
    fixture.sample(0U) = std::numeric_limits<Scalar>::quiet_NaN();
    fixture.gravity = {};
    if constexpr (Fixture::is_ins)
    {
        fixture.state.b_a(0U) = std::numeric_limits<Scalar>::infinity();
    }
    for (bool enabled : {false, true})
    {
        fixture.conditions.enabled = enabled;
        for (auto status : {Status::non_finite_input, Status::domain_error, Status::out_of_range,
                            Status::zero_or_unsafe_divisor, Status::invalid_quaternion_norm,
                            Status::not_positive_definite, Status::ill_conditioned, Status::non_finite_result})
        {
            fixture.magnitude.status = status;
            expect_preserved(test, profile, fixture, FusionDecision::failed, status);
        }
    }
    fixture.conditions.enabled = false;
    fixture.magnitude.status = Status::success;
    expect_preserved(test, profile, fixture, FusionDecision::rejected, Status::success);
}

template <typename Linalg, typename Configuration> void exact_observation(TestContext & test, std::string_view profile)
{
    test_correction<Linalg, Configuration>(test, profile);
    using Fixture = FusionFixture<Linalg, Configuration>;
    using Scalar = typename Linalg::value_type;
    Fixture fixture;
    fixture.state.q_nb = {};
    fixture.gravity = {};
    fixture.gravity(2U) = Scalar{8};
    fixture.sample = -fixture.gravity;
    if constexpr (Fixture::is_ins)
    {
        fixture.sample = fixture.sample + fixture.state.b_a;
    }
    fixture.P = Fixture::covariance_type::identity();
    fixture.V = {};
    fixture.V(0U, 0U) = Scalar{192};
    fixture.V(1U, 1U) = Scalar{192};
    fixture.V(2U, 2U) = Scalar{256};
    fixture.conditions.at_rest = true;
    fixture.monitor.reset();
    fixture.magnitude = fixture.monitor.update(fixture.sample);
    auto expected_P = fixture.P;
    expected_P(Fixture::attitude_offset, Fixture::attitude_offset) = Scalar{0.75};
    expected_P(Fixture::attitude_offset + 1U, Fixture::attitude_offset + 1U) = Scalar{0.75};
    // h=(0,0,-8), S=256I, L=16I, K_theta=[0,-1/32,0;1/32,0,0;0,0,0].
    // Zero residual keeps nominal state; Joseph diagonal is 9/16+3/16=3/4.
    for (unsigned alias = 0; alias < 4; ++alias)
    {
        auto current = fixture;
        typename Fixture::state_type separate;
        typename Fixture::covariance_type separate_P;
        auto & output = (alias & 1U) ? current.state : separate;
        auto & output_P = (alias & 2U) ? current.P : separate_P;
        auto const result = current.fuse(output, output_P);
        test.expect(result.decision == FusionDecision::fused && result.status == Status::success &&
                        same_state(output, fixture.state) && same_bits(output_P, expected_P),
                    profile,
                    "independent exact dyadic Joseph oracle preserves gravity magnitude and subtracts INS bias once");
    }
}

template <typename Linalg, typename Configuration> void failure_ownership(TestContext & test, std::string_view profile)
{
    test_failures<Linalg, Configuration>(test, profile);
    using Fixture = FusionFixture<Linalg, Configuration>;
    using Scalar = typename Linalg::value_type;
    Fixture fixture;
    auto control_monitor = fixture.monitor;
    auto const stored = fixture.monitor.filtered_specific_force_b();
    fixture.V(0U, 0U) = std::numeric_limits<Scalar>::infinity();
    typename Fixture::state_type state;
    typename Fixture::covariance_type P;
    auto const failed = fixture.fuse(state, P);
    test.expect(failed.decision == FusionDecision::failed && failed.conditions.sample_eligible &&
                    same_bits(fixture.monitor.filtered_specific_force_b(), stored),
                profile, "estimator failure neither updates nor rolls back previously consumed magnitude history");
    fixture.V = Fixture::noise_type::identity();
    auto const recovered = fixture.fuse(state, P);
    test.expect(recovered.decision == FusionDecision::fused, profile,
                "corrected request retries without hidden failure latch");
    fixture.sample(0U) += Scalar{0.5};
    auto const a = fixture.monitor.update(fixture.sample);
    auto const b = control_monitor.update(fixture.sample);
    test.expect(a.status == b.status && a.squared_norm == b.squared_norm &&
                    same_bits(fixture.monitor.filtered_specific_force_b(), control_monitor.filtered_specific_force_b()),
                profile, "common producer continuation confirms fusion did not consume another filter sample");
}

template <typename Linalg, typename Configuration>
void capability_sequence(TestContext & test, std::string_view profile)
{
    using Fixture = FusionFixture<Linalg, Configuration>;
    using Scalar = typename Linalg::value_type;
    formal_eskf::runtime::AccelerometerSample<Linalg, 10000U> sampler;
    formal_eskf::ImuSample<Linalg> imu;
    imu.specific_force_b(2U) = Scalar{-8};
    auto const partial = sampler.update(imu, 5000U, 0, Scalar{0.125});
    test.expect(partial.status == Status::success && !partial.ready, profile,
                "portable driver does not dispatch a correction for partial real intervals");
    auto emitted = sampler.update(imu, 5000U, 0, Scalar{0.125});
    test.expect(emitted.ready && emitted.interval_us == 10000U && emitted.specific_force_b(2U) == Scalar{-8} &&
                    emitted.magnitude.squared_norm == Scalar{64},
                profile, "first complete actual interval seeds diagnostic");
    Fixture fixture;
    fixture.state = {};
    fixture.P = Fixture::covariance_type::identity();
    fixture.V = Fixture::noise_type::identity() * Scalar{64};
    fixture.gravity = {};
    fixture.gravity(2U) = Scalar{8};
    fixture.conditions.at_rest = true;
    Scalar filtered_z = Scalar{-8};
    Scalar filtered_x = 0;
    Scalar const alpha = static_cast<Scalar>(0.010) / (static_cast<Scalar>(0.090) + static_cast<Scalar>(0.010));
    enum class Event
    {
        accepted,
        disabled,
        horizontal_aiding,
        clipped,
        numeric_failure,
        recovered
    };
    for (auto event : {Event::accepted, Event::disabled, Event::horizontal_aiding, Event::clipped,
                       Event::numeric_failure, Event::recovered})
    {
        if (event != Event::accepted)
        {
            imu.specific_force_b(0U) = Scalar{0.5};
            imu.specific_force_b(2U) = Scalar{-16};
            emitted = sampler.update(imu, 50000U, event == Event::clipped ? 2 : 0, Scalar{0.125});
            filtered_x += alpha * (Scalar{0.5} - filtered_x);
            filtered_z += alpha * (Scalar{-16} - filtered_z);
            test.expect(emitted.ready && emitted.specific_force_b(0U) == Scalar{0.5} &&
                            emitted.specific_force_b(2U) == Scalar{-16} &&
                            emitted.magnitude.squared_norm == filtered_x * filtered_x + filtered_z * filtered_z,
                        profile, "LPF advances once per actual emission independently of downstream decision");
        }
        fixture.sample = emitted.specific_force_b;
        fixture.magnitude = emitted.magnitude;
        fixture.conditions.enabled = event != Event::disabled;
        fixture.conditions.horizontal_aiding = event == Event::horizontal_aiding;
        fixture.conditions.clipped = {false, event == Event::clipped, false};
        fixture.V(0U, 0U) = event == Event::numeric_failure ? std::numeric_limits<Scalar>::quiet_NaN() : Scalar{64};
        auto const old_state = fixture.state;
        auto const old_P = fixture.P;
        fixture.prepare_reference();
        auto const reference = formal_eskf::test::reference::reference_correction(fixture, ESKF_RESET_APPROX != 0);
        auto const result = fixture.fuse(fixture.state, fixture.P);
        if (event == Event::accepted || event == Event::recovered)
        {
            auto prior = fixture;
            prior.state = old_state;
            prior.P = old_P;
            test.expect(result.decision == FusionDecision::fused &&
                            formal_eskf::test::reference::matches_reference(prior, reference, fixture.state, fixture.P,
                                                                            Scalar{128} *
                                                                                std::numeric_limits<Scalar>::epsilon()),
                        profile, "emitted raw observation reaches permitted numerical correction once");
        }
        else
        {
            test.expect(result.decision ==
                                (event == Event::numeric_failure ? FusionDecision::failed : FusionDecision::rejected) &&
                            same_state(fixture.state, old_state) && same_bits(fixture.P, old_P),
                        profile,
                        "disabled/aided/clipped/failed event leaves estimator unchanged, not producer history");
        }
    }
}

template <typename Linalg, typename Configuration>
void capability_discontinuity(TestContext & test, std::string_view profile)
{
    using Fixture = FusionFixture<Linalg, Configuration>;
    using Scalar = typename Linalg::value_type;
    formal_eskf::runtime::AccelerometerSample<Linalg, 10000U> sampler;
    formal_eskf::ImuSample<Linalg> imu;
    imu.specific_force_b(0U) = Scalar{100};
    test.expect(!sampler.update(imu, 5000U, -1, Scalar{0.125}).ready, profile,
                "old source leaves partial unknown interval");
    sampler.reset();
    imu.specific_force_b = {};
    imu.specific_force_b(2U) = Scalar{-8};
    auto const emitted = sampler.update(imu, 50000U, 0, Scalar{0.125});
    test.expect(emitted.ready && emitted.specific_force_b(0U) == Scalar{0} &&
                    emitted.specific_force_b(2U) == Scalar{-8} && emitted.clipping_mask == 0 &&
                    emitted.magnitude.squared_norm == Scalar{64},
                profile, "explicit caller source/gap reset excludes all old data, unknown clipping and filter history");
    Fixture fixture;
    fixture.sample = emitted.specific_force_b;
    fixture.magnitude = emitted.magnitude;
    fixture.conditions.at_rest = false;
    expect_preserved(test, profile, fixture, FusionDecision::rejected, Status::success);
    fixture.conditions.at_rest = true;
    typename Fixture::state_type state;
    typename Fixture::covariance_type P;
    test.expect(fixture.fuse(state, P).decision == FusionDecision::fused, profile,
                "known caller rest can bypass band; missing rest remains false rather than inferred");
    auto const unknown = sampler.update(imu, 50000U, -1, Scalar{0.125});
    test.expect(unknown.ready && unknown.clipping_mask == -1, profile,
                "unknown clipping remains unavailable to the driver, never silently converted to clear flags");
    // The portable driver must withhold the fusion call for an unknown mask: the
    // Boolean fusion API deliberately cannot encode unavailable clipping metadata.
    auto const before = state;
    auto const before_P = P;
    unsigned calls = 0;
    if (unknown.ready && unknown.clipping_mask >= 0)
    {
        ++calls;
        fixture.magnitude = unknown.magnitude;
        fixture.sample = unknown.specific_force_b;
        (void)fixture.fuse(state, P);
    }
    test.expect(calls == 0 && same_state(state, before) && same_bits(P, before_P), profile,
                "driver withholds unavailable metadata without inventing held/zero observations");
}

} /* end namespace */

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Case;
    using Float = formal_eskf::test::Backend<float>;
    using Double = formal_eskf::test::Backend<double>;
    Case const cases[] = {
        {"TC-R-ACCELEROMETER-FUSION-01", "float-ahrs", condition_first<Float, Ahrs>},
        {"TC-R-ACCELEROMETER-FUSION-01", "double-ahrs", condition_first<Double, Ahrs>},
        {"TC-R-ACCELEROMETER-FUSION-01", "float-ins", condition_first<Float, Ins>},
        {"TC-R-ACCELEROMETER-FUSION-01", "double-ins", condition_first<Double, Ins>},
        {"TC-R-ACCELEROMETER-FUSION-02", "float-ahrs", exact_observation<Float, Ahrs>},
        {"TC-R-ACCELEROMETER-FUSION-02", "double-ahrs", exact_observation<Double, Ahrs>},
        {"TC-R-ACCELEROMETER-FUSION-02", "float-ins", exact_observation<Float, Ins>},
        {"TC-R-ACCELEROMETER-FUSION-02", "double-ins", exact_observation<Double, Ins>},
        {"TC-R-ACCELEROMETER-FUSION-03", "float-ahrs", failure_ownership<Float, Ahrs>},
        {"TC-R-ACCELEROMETER-FUSION-03", "double-ahrs", failure_ownership<Double, Ahrs>},
        {"TC-R-ACCELEROMETER-FUSION-03", "float-ins", failure_ownership<Float, Ins>},
        {"TC-R-ACCELEROMETER-FUSION-03", "double-ins", failure_ownership<Double, Ins>},
        {"TC-SW-GRAVITY-01", "float-ahrs", capability_sequence<Float, Ahrs>},
        {"TC-SW-GRAVITY-01", "double-ahrs", capability_sequence<Double, Ahrs>},
        {"TC-SW-GRAVITY-01", "float-ins", capability_sequence<Float, Ins>},
        {"TC-SW-GRAVITY-01", "double-ins", capability_sequence<Double, Ins>},
        {"TC-SW-GRAVITY-02", "float-ahrs", capability_discontinuity<Float, Ahrs>},
        {"TC-SW-GRAVITY-02", "double-ahrs", capability_discontinuity<Double, Ahrs>},
        {"TC-SW-GRAVITY-02", "float-ins", capability_discontinuity<Float, Ins>},
        {"TC-SW-GRAVITY-02", "double-ins", capability_discontinuity<Double, Ins>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
