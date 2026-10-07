/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <formal_eskf/runtime/measurement_fusion.hpp>
#include <formal_eskf/runtime/position.hpp>
#include <formal_eskf/runtime/velocity.hpp>

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string_view>
#include <type_traits>

#include "test_backend.hpp"
#include "test_support.hpp"

namespace
{

using formal_eskf::Status;
using formal_eskf::runtime::FusionDecision;
using TestContext = formal_eskf::test::Context;
using BiasUpdate = formal_eskf::configuration::Ins::BiasUpdate;

// Test-only fault injection; checked quaternion construction rejects these values.
// This native-layout fixture is not a library serialization contract.
template <typename Linalg> auto invalid_quaternion(std::array<typename Linalg::value_type, 4U> const & coefficients)
{
    using quaternion_type = formal_eskf::so3::UnitQuaternion<Linalg>;
    static_assert(std::is_trivially_copyable_v<quaternion_type>);
    static_assert(std::is_standard_layout_v<quaternion_type>);
    static_assert(sizeof(quaternion_type) == sizeof(coefficients));
    return std::bit_cast<quaternion_type>(coefficients);
}

template <typename Matrix> bool same_matrix_bits(Matrix const & a, Matrix const & b)
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

template <typename State> bool same_fusion_state(State const & a, State const & b)
{
    bool equal = same_matrix_bits(a.q_nb.coefficients(), b.q_nb.coefficients());
    if constexpr (requires { a.p_n; })
    {
        equal = equal && same_matrix_bits(a.p_n, b.p_n) && same_matrix_bits(a.v_n, b.v_n) &&
                same_matrix_bits(a.b_a, b.b_a) && same_matrix_bits(a.b_g, b.b_g);
    }
    return equal;
}

BiasUpdate bias_permissions(unsigned mask)
{
    BiasUpdate result;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        result.accelerometer[axis] = (mask & (1U << axis)) != 0U;
        result.gyroscope[axis] = (mask & (1U << (3U + axis))) != 0U;
    }
    return result;
}

template <typename Linalg, typename Configuration, std::size_t MeasurementSize> struct FusionFixture
{
    using value_type = typename Linalg::value_type;
    using state_type = typename Configuration::template NominalState<Linalg>;
    static constexpr std::size_t size = Configuration::error_state_dimension;
    using covariance_type = formal_eskf::linalg::Matrix<Linalg, size, size>;
    using vector_type = formal_eskf::linalg::Matrix<Linalg, MeasurementSize, 1U>;
    using noise_type = formal_eskf::linalg::Matrix<Linalg, MeasurementSize, MeasurementSize>;
    using jacobian_type = formal_eskf::linalg::Matrix<Linalg, MeasurementSize, size>;
    value_type minimum_norm = static_cast<value_type>(1e-6);
    state_type state{};
    covariance_type P = covariance_type::identity();
    vector_type r{};
    jacobian_type H{};
    noise_type V = noise_type::identity();
    value_type sigma = value_type{3};

    FusionFixture()
    {
        for (std::size_t row = 0U; row < size; ++row)
        {
            for (std::size_t column = 0U; column < size; ++column)
            {
                P(row, column) += static_cast<value_type>((row + 1U) * (column + 1U)) / value_type{256};
            }
        }
        for (std::size_t row = 0U; row < MeasurementSize; ++row)
        {
            H.set(row, row % size, value_type{1});
            r.set(row, static_cast<value_type>(row + 1U) / value_type{64});
            for (std::size_t column = 0U; column < MeasurementSize; ++column)
            {
                V(row, column) += static_cast<value_type>((row + 1U) * (column + 1U)) / value_type{16};
            }
        }
    }

    auto fuse(state_type & output, covariance_type & covariance_output) const noexcept
    {
        return formal_eskf::runtime::fuse_measurement(state, P, r, H, V, minimum_norm, sigma, output,
                                                      covariance_output);
    }
};

template <typename Linalg, typename Configuration, std::size_t MeasurementSize>
void test_generic_fusion(TestContext & test, std::string_view profile)
{
    using fixture_type = FusionFixture<Linalg, Configuration, MeasurementSize>;
    using value_type = typename fixture_type::value_type;
    fixture_type const fixture;
    typename fixture_type::state_type expected_state;
    typename fixture_type::covariance_type expected_P;
    test.expect(formal_eskf::try_correct(fixture.state, fixture.P, fixture.r, fixture.H, fixture.V,
                                         fixture.minimum_norm, expected_state, expected_P) == Status::success,
                profile, "ungated correction reference succeeds");
    // All four whole-object alias arrangements; input matrices are correlated.
    for (unsigned aliases = 0U; aliases < 4U; ++aliases)
    {
        auto current = fixture;
        typename fixture_type::state_type output;
        auto output_P = fixture.P * value_type{-1};
        auto & state_output = (aliases & 1U) != 0U ? current.state : output;
        auto & covariance_output = (aliases & 2U) != 0U ? current.P : output_P;
        auto const result = current.fuse(state_output, covariance_output);
        test.expect(result.status == Status::success && result.decision == FusionDecision::fused &&
                        result.diagnostics_valid && same_fusion_state(state_output, expected_state) &&
                        same_matrix_bits(covariance_output, expected_P),
                    profile, "accepted gate delegates the unchanged full correlated correction with legal aliases");
        test.expect(((aliases & 1U) != 0U || same_fusion_state(current.state, fixture.state)) &&
                        ((aliases & 2U) != 0U || same_matrix_bits(current.P, fixture.P)) &&
                        same_matrix_bits(current.H, fixture.H) && same_matrix_bits(current.V, fixture.V) &&
                        same_matrix_bits(current.r, fixture.r),
                    profile, "disjoint inputs and linearization are unchanged");
        for (std::size_t axis = 0U; axis < MeasurementSize; ++axis)
        {
            // Independent scalar summation for diag(H P H^T + V), not V alone.
            long double variance = fixture.V(axis, axis);
            for (std::size_t row = 0U; row < fixture_type::size; ++row)
            {
                for (std::size_t column = 0U; column < fixture_type::size; ++column)
                {
                    variance += static_cast<long double>(fixture.H(axis, row)) * fixture.P(row, column) *
                                fixture.H(axis, column);
                }
            }
            long double const residual = fixture.r(axis);
            long double const sigma = fixture.sigma;
            long double const ratio = residual * residual / (sigma * sigma * variance);
            long double const tolerance = std::is_same_v<value_type, float> ? 1e-5L : 1e-12L;
            test.expect(result.innovation(axis) == fixture.r(axis) &&
                            std::abs(static_cast<long double>(result.innovation_variance(axis)) - variance) <
                                tolerance &&
                            std::abs(static_cast<long double>(result.test_ratio(axis)) - ratio) < tolerance,
                        profile, "diagnostics describe the supplied prior, not the posterior or noise alone");
        }
    }
}

template <typename Linalg, typename Configuration>
void test_gate_and_failures(TestContext & test, std::string_view profile)
{
    using fixture_type = FusionFixture<Linalg, Configuration, 3U>;
    using value_type = typename fixture_type::value_type;
    using limits_type = std::numeric_limits<value_type>;
    using covariance_type = typename fixture_type::covariance_type;
    fixture_type fixture;
    fixture.P = covariance_type::identity() * value_type{3};
    fixture.V = fixture_type::noise_type::identity();
    fixture.sigma = value_type{1};
    fixture.r = fixture_type::vector_type::zero();
    auto const preserved =
        [&](fixture_type const & source, FusionDecision expected, Status expected_status, bool diagnostics_valid)
    {
        for (unsigned aliases = 0U; aliases < 4U; ++aliases)
        {
            auto current = source;
            typename fixture_type::state_type output;
            auto output_P = covariance_type::identity() * value_type{-2};
            output_P.set(0U, 1U, limits_type::quiet_NaN());
            output_P.set(1U, 0U, -value_type{0});
            auto & state_output = (aliases & 1U) != 0U ? current.state : output;
            auto & covariance_output = (aliases & 2U) != 0U ? current.P : output_P;
            auto const before_state = state_output;
            auto const before_P = covariance_output;
            auto const result = current.fuse(state_output, covariance_output);
            test.expect(result.decision == expected && result.status == expected_status &&
                            result.diagnostics_valid == diagnostics_valid,
                        profile, "rejection and numerical failure have distinct outcomes and diagnostic validity");
            test.expect(same_fusion_state(state_output, before_state) && same_matrix_bits(covariance_output, before_P),
                        profile,
                        "rejection/failure preserves both arbitrary outputs, including all alias arrangements");
            if (!diagnostics_valid)
            {
                test.expect(same_matrix_bits(result.innovation, fixture_type::vector_type::zero()) &&
                                same_matrix_bits(result.innovation_variance, fixture_type::vector_type::zero()) &&
                                same_matrix_bits(result.test_ratio, fixture_type::vector_type::zero()),
                            profile, "failed gate has fresh invalid placeholders, never old diagnostics");
            }
        }
    };
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        for (value_type sign : {value_type{-1}, value_type{1}})
        {
            auto boundary = fixture;
            boundary.r.set(axis, sign * value_type{2}); // S_ii=4, sigma=1.
            typename fixture_type::state_type output;
            covariance_type output_P;
            auto result = boundary.fuse(output, output_P);
            test.expect(result.decision == FusionDecision::fused && result.test_ratio(axis) == value_type{1}, profile,
                        "gate equality fuses on every axis and either sign");
            boundary.r.set(axis, sign * std::nextafter(value_type{2}, limits_type::infinity()));
            preserved(boundary, FusionDecision::rejected, Status::success, true);
            boundary.r.set(axis, sign * std::nextafter(value_type{2}, value_type{0}));
            result = boundary.fuse(output, output_P);
            test.expect(result.decision == FusionDecision::fused, profile, "next value below gate boundary fuses");
        }
    }
    for (value_type invalid : {limits_type::quiet_NaN(), limits_type::infinity(), -limits_type::infinity()})
    {
        auto bad = fixture;
        bad.r.set(0U, value_type{3}); // An early outlier must not hide another invalid input.
        bad.r.set(2U, invalid);
        preserved(bad, FusionDecision::failed, Status::non_finite_input, false);
        bad = fixture;
        bad.P.set(0U, fixture_type::size - 1U, invalid);
        preserved(bad, FusionDecision::failed, Status::non_finite_input, false);
        bad = fixture;
        bad.H.set(2U, fixture_type::size - 1U, invalid);
        preserved(bad, FusionDecision::failed, Status::non_finite_input, false);
        bad = fixture;
        bad.V.set(0U, 2U, invalid);
        preserved(bad, FusionDecision::failed, Status::non_finite_input, false);
        bad = fixture;
        bad.sigma = invalid;
        preserved(bad, FusionDecision::failed, Status::non_finite_input, false);
        bad = fixture;
        bad.state.q_nb = invalid_quaternion<Linalg>({value_type{1}, value_type{0}, value_type{0}, invalid});
        bad.r.set(0U, value_type{3});
        preserved(bad, FusionDecision::failed, Status::non_finite_input, false);
        if constexpr (std::is_same_v<Configuration, formal_eskf::configuration::Ins>)
        {
            for (unsigned field = 0U; field < 12U; ++field)
            {
                bad = fixture;
                std::array<formal_eskf::linalg::Matrix<Linalg, 3U, 1U> *, 4U> fields{&bad.state.p_n, &bad.state.v_n,
                                                                                     &bad.state.b_a, &bad.state.b_g};
                fields[field / 3U]->set(field % 3U, invalid);
                preserved(bad, FusionDecision::failed, Status::non_finite_input, false);
            }
        }
    }
    for (value_type sigma : {value_type{0}, value_type{-1}})
    {
        auto bad = fixture;
        bad.sigma = sigma;
        preserved(bad, FusionDecision::failed, Status::domain_error, false);
    }
    for (value_type bound : {value_type{0}, value_type{-1}, value_type{2}})
    {
        auto bad = fixture;
        bad.minimum_norm = bound;
        preserved(bad, FusionDecision::failed, Status::domain_error, false);
    }
    auto bad = fixture;
    bad.minimum_norm = limits_type::quiet_NaN();
    preserved(bad, FusionDecision::failed, Status::non_finite_input, false);
    bad = fixture;
    bad.sigma = limits_type::max();
    preserved(bad, FusionDecision::failed, Status::non_finite_result, false);
    bad.sigma = limits_type::min();
    preserved(bad, FusionDecision::failed, Status::zero_or_unsafe_divisor, false);
    bad = fixture;
    bad.P.set(0U, 1U, value_type{1}); // Asymmetry must not be silently repaired.
    preserved(bad, FusionDecision::failed, Status::domain_error, false);
    bad = fixture;
    bad.V.set(0U, 1U, value_type{1});
    preserved(bad, FusionDecision::failed, Status::domain_error, false);
    bad = fixture;
    bad.P.set(0U, 0U, value_type{-1});
    preserved(bad, FusionDecision::failed, Status::domain_error, false);
    bad = fixture;
    bad.V.set(0U, 0U, value_type{0});
    preserved(bad, FusionDecision::failed, Status::domain_error, false);
    bad = fixture;
    bad.P.set(0U, 0U, limits_type::max());
    bad.H.set(0U, 0U, value_type{2}); // PHt overflow.
    preserved(bad, FusionDecision::failed, Status::non_finite_result, false);
    bad = fixture;
    bad.H.set(0U, 0U, std::sqrt(limits_type::max())); // Finite PHt, overflowing S.
    preserved(bad, FusionDecision::failed, Status::non_finite_result, false);
    bad = fixture;
    bad.r.set(0U, limits_type::max());
    preserved(bad, FusionDecision::failed, Status::non_finite_result, false);
    bad = fixture;
    bad.P = covariance_type::identity() * limits_type::min();
    bad.V = fixture_type::noise_type::identity() * limits_type::min();
    bad.sigma = value_type{0.5}; // Positive but unsafe denominator, not an outlier.
    preserved(bad, FusionDecision::failed, Status::zero_or_unsafe_divisor, false);

    // Gate passes, LLT fails: positive innovation diagonal is not an SPD proof.
    bad = fixture;
    bad.V.set(0U, 1U, value_type{8});
    bad.V.set(1U, 0U, value_type{8});
    preserved(bad, FusionDecision::failed, Status::not_positive_definite, true);
    // Gate passes, injection fails; valid diagnostics survive but no estimate is published.
    bad = fixture;
    bad.state.q_nb = invalid_quaternion<Linalg>({value_type{0}, value_type{0}, value_type{0}, value_type{0}});
    preserved(bad, FusionDecision::failed, Status::invalid_quaternion_norm, true);
    bad.r.set(0U, value_type{3});
    preserved(bad, FusionDecision::rejected, Status::success, true); // Rejection does not run injection.
}

enum class Observation
{
    horizontal_position,
    vertical_position,
    horizontal_velocity,
    velocity,
};

template <typename Linalg, Observation Kind, std::size_t MeasurementSize>
struct ObservationFusionFixture : FusionFixture<Linalg, formal_eskf::configuration::Ins, MeasurementSize>
{
    static constexpr std::size_t measurement_size = MeasurementSize;
    using base_type = FusionFixture<Linalg, formal_eskf::configuration::Ins, measurement_size>;
    using value_type = typename base_type::value_type;
    using state_type = typename base_type::state_type;
    using covariance_type = typename base_type::covariance_type;
    using vector_type = typename base_type::vector_type;
    static constexpr std::size_t offset = Kind == Observation::vertical_position ? 2U : 0U;
    vector_type z{};

    ObservationFusionFixture()
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            this->state.p_n.set(axis, static_cast<value_type>(axis + 1U));
            this->state.v_n.set(axis, -static_cast<value_type>(axis + 1U));
        }
        for (std::size_t axis = 0U; axis < measurement_size; ++axis)
        {
            z.set(axis, observed_state()(offset + axis) + this->r(axis));
        }
    }

    auto & observed_state() noexcept
    {
        if constexpr (Kind == Observation::horizontal_position || Kind == Observation::vertical_position)
        {
            return this->state.p_n;
        }
        else
        {
            return this->state.v_n;
        }
    }

    auto fuse(state_type & output, covariance_type & output_P, BiasUpdate const & permissions = {}) const noexcept
    {
        if constexpr (Kind == Observation::horizontal_position)
        {
            return formal_eskf::runtime::fuse_horizontal_position(this->state, this->P, z, this->V, this->minimum_norm,
                                                                  this->sigma, output, output_P, permissions);
        }
        else if constexpr (Kind == Observation::vertical_position)
        {
            return formal_eskf::runtime::fuse_vertical_position(this->state, this->P, z(0U), this->V(0U, 0U),
                                                                this->minimum_norm, this->sigma, output, output_P,
                                                                permissions);
        }
        else if constexpr (Kind == Observation::horizontal_velocity)
        {
            return formal_eskf::runtime::fuse_horizontal_velocity(this->state, this->P, z, this->V, this->minimum_norm,
                                                                  this->sigma, output, output_P, permissions);
        }
        else
        {
            return formal_eskf::runtime::fuse_velocity(this->state, this->P, z, this->V, this->minimum_norm,
                                                       this->sigma, output, output_P, permissions);
        }
    }

    Status correct(state_type & output, covariance_type & output_P, BiasUpdate const & permissions) const noexcept
    {
        if constexpr (Kind == Observation::horizontal_position)
        {
            return formal_eskf::try_correct_horizontal_position(this->state, this->P, z, this->V, this->minimum_norm,
                                                                output, output_P, permissions);
        }
        else if constexpr (Kind == Observation::vertical_position)
        {
            return formal_eskf::try_correct_vertical_position(this->state, this->P, z(0U), this->V(0U, 0U),
                                                              this->minimum_norm, output, output_P, permissions);
        }
        else if constexpr (Kind == Observation::horizontal_velocity)
        {
            return formal_eskf::try_correct_horizontal_velocity(this->state, this->P, z, this->V, this->minimum_norm,
                                                                output, output_P, permissions);
        }
        else
        {
            return formal_eskf::try_correct_velocity(this->state, this->P, z, this->V, this->minimum_norm, output,
                                                     output_P, permissions);
        }
    }
};

template <typename Linalg, Observation Kind, std::size_t MeasurementSize>
void test_observation_fusion(TestContext & test, std::string_view profile, std::size_t error_offset)
{
    using fixture_type = ObservationFusionFixture<Linalg, Kind, MeasurementSize>;
    using value_type = typename fixture_type::value_type;
    using state_type = typename fixture_type::state_type;
    using covariance_type = typename fixture_type::covariance_type;
    fixture_type fixture;
    static_assert(noexcept(fixture.fuse(fixture.state, fixture.P)));
    for (unsigned mask = 0U; mask < 64U; ++mask)
    {
        auto const permissions = bias_permissions(mask);
        state_type expected_state;
        covariance_type expected_P;
        test.expect(fixture.correct(expected_state, expected_P, permissions) == Status::success, profile,
                    "ungated sensor model succeeds with each bias permission combination");
        for (unsigned aliases = 0U; aliases < 4U; ++aliases)
        {
            auto current = fixture;
            state_type output;
            covariance_type output_P;
            auto & state_output = (aliases & 1U) != 0U ? current.state : output;
            auto & covariance_output = (aliases & 2U) != 0U ? current.P : output_P;
            auto const result = current.fuse(state_output, covariance_output, permissions);
            test.expect(result.decision == FusionDecision::fused && result.status == Status::success &&
                            result.diagnostics_valid && same_fusion_state(state_output, expected_state) &&
                            same_matrix_bits(covariance_output, expected_P),
                        profile, "gated sensor API preserves the core model, full V, bias mask and aliases");
            for (std::size_t axis = 0U; axis < fixture_type::measurement_size; ++axis)
            {
                auto const variance = fixture.P(error_offset + axis, error_offset + axis) + fixture.V(axis, axis);
                test.expect(result.innovation(axis) == fixture.r(axis) && result.innovation_variance(axis) == variance,
                            profile, "sensor gate selects the right prior block and observation-minus-prediction sign");
            }
        }
    }
    auto const no_publication = [&](fixture_type const & input, FusionDecision decision, Status status)
    {
        auto current = input;
        auto const before_state = current.state;
        auto const before_P = current.P;
        auto const result = current.fuse(current.state, current.P);
        test.expect(result.decision == decision && result.status == status &&
                        same_fusion_state(current.state, before_state) && same_matrix_bits(current.P, before_P),
                    profile, "sensor rejection or residual failure cannot publish a partial correction");
    };
    for (std::size_t axis = 0U; axis < fixture_type::measurement_size; ++axis)
    {
        auto bad = fixture;
        bad.z.set(axis, value_type{100});
        no_publication(bad, FusionDecision::rejected, Status::success);
        bad.z.set(axis, std::numeric_limits<value_type>::quiet_NaN());
        no_publication(bad, FusionDecision::failed, Status::non_finite_input);
        bad.z.set(axis, std::numeric_limits<value_type>::max());
        bad.observed_state().set(fixture_type::offset + axis, -std::numeric_limits<value_type>::max());
        no_publication(bad, FusionDecision::failed, Status::non_finite_result);
    }
}

template <typename Linalg> void test_sequential_groups(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using state_type = formal_eskf::configuration::Ins::NominalState<Linalg>;
    using covariance_type = formal_eskf::linalg::Matrix<Linalg, 15U, 15U>;
    using vector2_type = formal_eskf::linalg::Matrix<Linalg, 2U, 1U>;
    using vector3_type = formal_eskf::linalg::Matrix<Linalg, 3U, 1U>;
    auto const V2 = formal_eskf::linalg::Matrix<Linalg, 2U, 2U>::identity();
    auto const V3 = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>::identity();
    auto const minimum_norm = static_cast<value_type>(1e-6);
    state_type state;
    auto P = covariance_type::identity();
    P.set(0U, 3U, value_type{0.5});
    P.set(3U, 0U, value_type{0.5});
    auto const position = vector2_type::from_row_major({value_type{2}, value_type{0}});
    auto const position_result =
        formal_eskf::runtime::fuse_horizontal_position(state, P, position, V2, minimum_norm, value_type{2}, state, P);
    // This analytic oracle crosses an LLT sqrt/divide: allow rounding at unit scale.
    auto const tolerance = value_type{16} * std::numeric_limits<value_type>::epsilon();
    test.expect(position_result.decision == FusionDecision::fused &&
                    formal_eskf::test::near(state.p_n(0U), value_type{1}, tolerance) &&
                    formal_eskf::test::near(state.v_n(0U), value_type{0.5}, tolerance) &&
                    formal_eskf::test::near(P(3U, 3U), value_type{0.875}, tolerance),
                profile, "first group updates velocity through prior cross-covariance");
    auto const after_position = state;
    auto const after_position_P = P;
    auto const height_result = formal_eskf::runtime::fuse_vertical_position(state, P, value_type{100}, value_type{1},
                                                                            minimum_norm, value_type{1}, state, P);
    test.expect(height_result.decision == FusionDecision::rejected && same_fusion_state(state, after_position) &&
                    same_matrix_bits(P, after_position_P),
                profile, "height rejection does not undo accepted horizontal position");
    auto const velocity = vector3_type::from_row_major({value_type{1.5}, value_type{0}, value_type{0}});
    state_type expected_state;
    covariance_type expected_P;
    test.expect(formal_eskf::try_correct_velocity(state, P, velocity, V3, minimum_norm, expected_state, expected_P) ==
                    Status::success,
                profile, "sequential correction reference succeeds");
    auto const velocity_result =
        formal_eskf::runtime::fuse_velocity(state, P, velocity, V3, minimum_norm, value_type{1}, state, P);
    test.expect(velocity_result.decision == FusionDecision::fused && velocity_result.innovation(0U) == value_type{1} &&
                    velocity_result.innovation_variance(0U) == value_type{1.875} &&
                    same_fusion_state(state, expected_state) && same_matrix_bits(P, expected_P),
                profile, "later group recomputes residual and S from the latest committed prior");
    // Using the pre-position prior would reject: 1.5^2 / (1 + 1) > 1.
    state_type old_state;
    auto old_P = covariance_type::identity();
    old_P.set(0U, 3U, value_type{0.5});
    old_P.set(3U, 0U, value_type{0.5});
    auto const stale = formal_eskf::runtime::fuse_velocity(old_state, old_P, velocity, V3, minimum_norm, value_type{1},
                                                           old_state, old_P);
    test.expect(stale.decision == FusionDecision::rejected, profile,
                "the sequence detects stale-prior gating, rather than merely testing independent covariances");
    auto const failed = formal_eskf::runtime::fuse_vertical_position(state, P, value_type{0}, value_type{-1},
                                                                     minimum_norm, value_type{1}, state, P);
    test.expect(failed.decision == FusionDecision::failed && same_fusion_state(state, expected_state) &&
                    same_matrix_bits(P, expected_P),
                profile, "a later numerical failure also preserves earlier successful groups");
}

template <typename Linalg> void run_conformance(TestContext & test, std::string_view profile)
{
    using Ahrs = formal_eskf::configuration::Ahrs;
    using Ins = formal_eskf::configuration::Ins;
    test_generic_fusion<Linalg, Ahrs, 1U>(test, profile);
    test_generic_fusion<Linalg, Ahrs, 2U>(test, profile);
    test_generic_fusion<Linalg, Ahrs, 3U>(test, profile);
    test_generic_fusion<Linalg, Ahrs, 6U>(test, profile);
    test_generic_fusion<Linalg, Ins, 1U>(test, profile);
    test_generic_fusion<Linalg, Ins, 2U>(test, profile);
    test_generic_fusion<Linalg, Ins, 3U>(test, profile);
    test_generic_fusion<Linalg, Ins, 6U>(test, profile);
    test_gate_and_failures<Linalg, Ahrs>(test, profile);
    test_gate_and_failures<Linalg, Ins>(test, profile);
    test_observation_fusion<Linalg, Observation::horizontal_position, 2U>(test, profile, 0U);
    test_observation_fusion<Linalg, Observation::vertical_position, 1U>(test, profile, 2U);
    test_observation_fusion<Linalg, Observation::horizontal_velocity, 2U>(test, profile, 3U);
    test_observation_fusion<Linalg, Observation::velocity, 3U>(test, profile, 3U);
    test_sequential_groups<Linalg>(test, profile);
}

} /* end anonymous namespace */

int main()
{
    formal_eskf::test::configure_backend_test();
    TestContext test;
    run_conformance<formal_eskf::test::Backend<float>>(test, "float");
    run_conformance<formal_eskf::test::Backend<double>>(test, "double");
    if (test.failures() != 0)
    {
        std::cerr << test.failures() << " measurement fusion check(s) failed\n";
        return 1;
    }
    std::cout << "Measurement fusion checks passed\n";
    return 0;
}
