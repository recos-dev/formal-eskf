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
#include <limits>

#include <formal_eskf/runtime/navigation.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"
#include "correction_reference.hpp"
#include "test_support.hpp"

namespace
{
using formal_eskf::Status;
using formal_eskf::runtime::FusionDecision;
using Ins = formal_eskf::configuration::Ins;
using Context = formal_eskf::test::Context;

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
    return same_bits(a.p_n, b.p_n) && same_bits(a.v_n, b.v_n) && same_bits(a.b_a, b.b_a) && same_bits(a.b_g, b.b_g) &&
           same_bits(a.q_nb.coefficients(), b.q_nb.coefficients());
}

template <typename Linalg> struct GnssMeasurementFixture
{
    using value_type = typename Linalg::value_type;
    using state_type = Ins::NominalState<Linalg>;
    using covariance_type = formal_eskf::linalg::Matrix<Linalg, 15U, 15U>;
    using noise_type = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>;
    using vector_type = formal_eskf::linalg::Vector<Linalg, 3U>;
    state_type state{};
    covariance_type P = covariance_type::identity() * value_type{3};
    noise_type V = noise_type::identity();
    vector_type velocity = vector_type::from_row_major({value_type{0.5}, value_type{-0.5}, value_type{-10}});
    value_type gate = value_type{1}; // S=4, limit=2, exactly representable boundaries.
    value_type minimum_norm = static_cast<value_type>(1e-6);
    formal_eskf::runtime::GnssVelocityConditions<value_type> conditions{true, value_type{0.25}, value_type{0.5}};

    auto fuse(state_type & output, covariance_type & output_P, Ins::BiasUpdate const & permissions = {}) const
    {
        return formal_eskf::runtime::fuse_gnss_velocity(state, P, velocity, V, minimum_norm, gate, conditions, output,
                                                        output_P, permissions);
    }
};

template <typename Base> struct LimitMath : Base
{
    using value_type = typename Base::value_type;
    static inline bool oversized_root = false;
    static value_type sqrt(value_type value)
    {
        // Tests change the fault selector before invoking this backend through templates.
        // cppcheck-suppress knownConditionTrueFalse
        return oversized_root ? std::numeric_limits<value_type>::max() : Base::sqrt(value);
    }
};
template <typename Linalg> struct LimitBackend : Linalg
{
    using scalar_math_type = LimitMath<typename Linalg::scalar_math_type>;
};
template <typename Linalg, std::size_t M> struct ReferenceFixture
{
    using value_type = typename Linalg::value_type;
    using state_type = typename GnssMeasurementFixture<Linalg>::state_type;
    using covariance_type = typename GnssMeasurementFixture<Linalg>::covariance_type;
    static constexpr std::size_t size = 15U;
    // Used through dependent fixture types in the test/oracle templates.
    // cppcheck-suppress unusedStructMember
    static constexpr std::size_t measurement_size = M;
    // Used through dependent fixture types in the test/oracle templates.
    // cppcheck-suppress unusedStructMember
    static constexpr std::size_t attitude_offset = 6U;
    // Used through dependent fixture types in the test/oracle templates.
    // cppcheck-suppress unusedStructMember
    static constexpr bool is_ins = true;
    state_type state;
    covariance_type P;
    formal_eskf::linalg::Matrix<Linalg, M, 15U> H;
    formal_eskf::linalg::Matrix<Linalg, M, M> V;
    formal_eskf::linalg::Vector<Linalg, M> r;
};

template <typename Linalg, bool Height>
auto reference_fixture(GnssMeasurementFixture<Linalg> const & f, typename Linalg::value_type selected_down)
{
    constexpr std::size_t m = Height ? 1U : 3U;
    ReferenceFixture<Linalg, m> result;
    result.state = f.state;
    result.P = f.P;
    for (std::size_t row = 0; row < m; ++row)
    {
        result.H(row, Height ? 2U : 3U + row) = typename Linalg::value_type{1};
        result.r(row) = Height || row == 2U ? selected_down : f.velocity(row) - f.state.v_n(row);
        for (std::size_t column = 0; column < m; ++column)
        {
            result.V(row, column) = Height ? typename Linalg::value_type{1} : f.V(row, column);
        }
    }
    return result;
}
template <typename Linalg> void velocity_exception(Context & test, std::string_view profile)
{
    using F = GnssMeasurementFixture<Linalg>;
    using T = typename F::value_type;
    auto const below = std::nextafter(T{2}, T{0});
    auto const above = std::nextafter(T{2}, std::numeric_limits<T>::infinity());
    for (bool fault : {false, true})
    {
        for (T accuracy : {T{0}, std::nextafter(T{0.5}, T{0}), T{0.5}, std::nextafter(T{0.5}, T{1})})
        {
            for (T north : {T{0}, below, T{2}, above})
            {
                for (T east : {T{0}, below, T{2}, above})
                {
                    for (T down : {T{-10}, -above, T{-2}, -below, T{0}, below, T{2}, above, T{10}})
                    {
                        F f;
                        f.conditions = {fault, accuracy, T{0.5}};
                        f.velocity = F::vector_type::from_row_major({north, east, down});
                        auto state = f.state;
                        auto P = f.P;
                        auto const result = f.fuse(state, P);
                        bool const rejected = north > T{2} || east > T{2} || std::abs(down) > T{2};
                        bool const limited =
                            fault && accuracy < T{0.5} && north < T{2} && east < T{2} && std::abs(down) > T{2};
                        T const selected = limited ? std::copysign(T{2}, down) : down;
                        test.expect(result.fusion.status == Status::success && result.fusion.diagnostics_valid &&
                                        result.innovation_limited == limited &&
                                        result.fusion.decision ==
                                            (rejected && !limited ? FusionDecision::rejected : FusionDecision::fused) &&
                                        result.fusion_innovation(2) == selected &&
                                        result.fusion.innovation(2) == down &&
                                        result.fusion.test_ratio(2) == down * down / T{4} &&
                                        result.fusion_innovation(0) == north && result.fusion_innovation(1) == east,
                                    profile,
                                    "independent strict conjunction covers both horizontal neighbours, Down signs and "
                                    "receiver accuracy");
                        if (rejected && !limited)
                        {
                            test.expect(same_state(state, f.state) && same_bits(P, f.P), profile,
                                        "raw group rejection preserves all outputs");
                        }
                        else
                        {
                            test.expect(state.v_n(2) == T{0.75} * selected, profile,
                                        "exact dyadic gain applies only selected residual");
                        }
                    }
                }
            }
        }
    }
    for (bool fault : {false, true})
    {
        for (bool maximum : {false, true})
        {
            for (T value : {T{-1}, T{0}, std::numeric_limits<T>::quiet_NaN(), std::numeric_limits<T>::infinity(),
                            -std::numeric_limits<T>::infinity()})
            {
                if (!maximum && value == T{0})
                {
                    continue;
                }
                F f;
                f.conditions.bad_vertical_acceleration = fault;
                (maximum ? f.conditions.maximum_speed_accuracy : f.conditions.speed_accuracy) = value;
                auto state = f.state;
                auto P = f.P;
                auto const result = f.fuse(state, P);
                test.expect(result.fusion.status ==
                                    (std::isfinite(value) ? Status::domain_error : Status::non_finite_input) &&
                                result.fusion.decision == FusionDecision::failed && !result.fusion.diagnostics_valid &&
                                same_state(state, f.state) && same_bits(P, f.P),
                            profile, "accuracy domain checked independently of held fault");
            }
        }
    }
}

template <typename Linalg, bool Height>
void publication_fixture(Context & test, std::string_view profile, bool limited);

template <typename Linalg> void signed_diagnostics(Context & test, std::string_view profile)
{
    publication_fixture<Linalg, false>(test, profile, true);
    publication_fixture<Linalg, true>(test, profile, true);
    using F = GnssMeasurementFixture<Linalg>;
    using T = typename F::value_type;
    for (T down : {T{-10}, T{10}})
    {
        F f;
        f.velocity(2) = down;
        f.P(9, 5) = f.P(5, 9) = T{0.25};
        // S upper 2x2 = [[4,1],[1,4.25]] with exact dyadic Cholesky [2,0; .5,2].
        f.V(0, 1) = f.V(1, 0) = T{1};
        f.V(1, 1) = T{1.25};
        test.expect(F::state_type::quaternion_type::from_coefficients(T{0.5}, T{0.5}, T{0.5}, T{0.5}, T{0.125},
                                                                      f.state.q_nb) == Status::success,
                    profile, "exact nonidentity unit attitude fixture");
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            f.state.b_a(axis) = static_cast<T>(axis + 1U) / T{8};
            f.state.b_g(axis) = -static_cast<T>(axis + 1U) / T{16};
        }
        f.state.p_n(0) = T{2};
        f.state.p_n(1) = T{-3};
        auto const prior = f;
        auto const check = formal_eskf::runtime::check_gnss_velocity(f.state, f.P, f.velocity, f.V, f.minimum_norm,
                                                                     f.gate, f.conditions);
        auto output = f.state;
        auto output_P = f.P;
        auto const result = f.fuse(output, output_P);
        T const selected = down < T{0} ? T{-2} : T{2};
        auto const independent = reference_fixture<Linalg, false>(f, selected);
        auto const oracle = formal_eskf::test::reference::reference_correction(independent, ESKF_RESET_APPROX != 0);
        test.expect(check.gate.status == Status::success && check.gate.diagnostics_valid && check.innovation_limited &&
                        check.gate.passed() && same_state(f.state, prior.state) && same_bits(f.P, prior.P) &&
                        same_bits(check.gate.innovation, result.fusion.innovation) &&
                        same_bits(check.fusion_innovation, result.fusion_innovation) &&
                        result.fusion.decision == FusionDecision::fused && result.fusion.innovation(2) == down &&
                        result.fusion_innovation(2) == selected &&
                        formal_eskf::test::reference::matches_reference(independent, oracle, output, output_P, T{0}),
                    profile, "check-only is pure; independent dyadic correlated solve/Joseph verifies bounded result");
    }
    for (bool fault : {false, true})
    {
        for (T down : {T{-10}, T{-2}, T{0}, T{2}, T{10}})
        {
            F fixture;
            auto output = fixture.state;
            auto output_P = fixture.P;
            auto const result = formal_eskf::runtime::fuse_height(fixture.state, fixture.P, down, T{1},
                                                                  fixture.minimum_norm, T{1}, fault, output, output_P);
            bool const rejected = std::abs(down) > T{2};
            bool const limited = fault && rejected;
            T const expected = limited ? (down < T{0} ? T{-2} : T{2}) : down;
            test.expect(result.fusion.status == Status::success && result.innovation_limited == limited &&
                            result.fusion.decision ==
                                (rejected && !fault ? FusionDecision::rejected : FusionDecision::fused) &&
                            result.fusion_innovation(0U) == expected && result.fusion.innovation(0U) == down,
                        profile, "height exception requires fault and rejection, with symmetric sign handling");
            test.expect(output.p_n(2U) == (rejected && !fault ? T{0} : T{0.75} * expected), profile,
                        "height correction actually applies bounded innovation");
        }
    }

    bool exercised = false;
    for (unsigned variance = 4U; variance < 64U && !exercised; ++variance)
    {
        F fixture;
        T const S = static_cast<T>(variance);
        T const limit = std::sqrt(S);
        // Seek an IEEE-754 sqrt whose square rounds just outside the gate.
        if (limit * limit / S <= T{1})
        {
            continue;
        }
        fixture.V = F::noise_type::identity() * (S - T{3});
        fixture.velocity(2U) = T{-100};
        auto output = fixture.state;
        auto output_P = fixture.P;
        auto const result = fixture.fuse(output, output_P);
        test.expect(result.fusion.decision == FusionDecision::fused && result.innovation_limited &&
                        result.fusion_innovation(2U) == -limit,
                    profile, "rounded boundary is not incorrectly rejected by a second gate");
        exercised = true;
    }
    test.expect(exercised, profile, "floating-point boundary fixture is exercised");
}

template <typename Linalg> void height_deadzone(Context & test, std::string_view profile)
{
    using F = GnssMeasurementFixture<Linalg>;
    using T = typename F::value_type;
    // P_DD=3, R=1: S=4, K_D=3/4; a 5-sigma gate ends at +/-10 m.
    for (bool fault : {false, true})
    {
        for (T residual : {T{-12}, T{-4}, T{0}, T{2}, T{4}, T{6}, T{10}, T{12}})
        {
            F fixture;
            auto output = fixture.state;
            auto output_P = fixture.P;
            auto const result =
                formal_eskf::runtime::fuse_height(fixture.state, fixture.P, residual, T{1}, fixture.minimum_norm, T{5},
                                                  fault, output, output_P, {}, T{4});
            bool const rejected = std::abs(residual) > T{10};
            bool const limited = rejected && fault;
            T const bounded = limited ? (residual < 0 ? T{-10} : T{10}) : residual;
            T const selected = bounded > 0 ? std::max(T{0}, bounded - T{4}) : bounded;
            bool const fused = !rejected || fault;
            test.expect(result.fusion.status == Status::success && result.fusion.diagnostics_valid &&
                            result.fusion.innovation(0U) == residual &&
                            result.fusion.test_ratio(0U) == residual * residual / T{100} &&
                            result.fusion_innovation(0U) == selected && result.innovation_limited == limited &&
                            result.fusion.decision == (fused ? FusionDecision::fused : FusionDecision::rejected),
                        profile, "positive-only deadzone follows raw gating and the acceleration-fault limit");
            test.expect(output.p_n(2U) == (fused ? T{0.75} * selected : T{0}) &&
                            output_P(2U, 2U) == (fused ? T{0.75} : T{3}),
                        profile, "zero residual still performs covariance correction; raw rejection preserves state/P");
            for (std::size_t row = 0; row < 15; ++row)
            {
                for (std::size_t column = 0; column < 15; ++column)
                {
                    T const expected = row != column ? T{0} : row == 2U && fused ? T{0.75} : T{3};
                    test.expect(output_P(row, column) == expected, profile,
                                "independent exact scalar Joseph leaves every unobserved block unchanged");
                }
            }
        }
    }
    for (T deadzone : {T{0}, std::nextafter(T{2}, T{0}), T{2}, std::nextafter(T{2}, T{3}), T{4}})
    {
        F f;
        auto state = f.state;
        auto P = f.P;
        auto const result = formal_eskf::runtime::fuse_height(f.state, f.P, T{2}, T{1}, f.minimum_norm, T{1}, false,
                                                              state, P, {}, deadzone);
        T const selected = deadzone < T{2} ? T{2} - deadzone : T{0};
        test.expect(result.fusion.decision == FusionDecision::fused && !result.innovation_limited &&
                        result.fusion_innovation(0) == selected && state.p_n(2) == T{0.75} * selected &&
                        P(2, 2) == T{0.75},
                    profile, "positive deadzone threshold neighbours change residual but still update covariance");
    }
}

template <typename Linalg> void limit_failure(Context & test, std::string_view profile)
{
    using B = LimitBackend<Linalg>;
    using F = GnssMeasurementFixture<B>;
    using T = typename F::value_type;
    for (bool height : {false, true})
    {
        for (unsigned aliases = 0; aliases < 4U; ++aliases)
        {
            F f;
            f.gate = T{2};
            auto separate = f.state;
            separate.p_n(0) = T{99};
            auto separate_P = f.P * T{2};
            auto & target = (aliases & 1U) ? f.state : separate;
            auto & target_P = (aliases & 2U) ? f.P : separate_P;
            auto const before = target;
            auto const before_P = target_P;
            // Deliberately nonconforming math result exercises error propagation; it is not a claim
            // that finite valid standard sqrt can overflow the already-validated gate denominator.
            B::scalar_math_type::oversized_root = true;
            if (height)
            {
                auto const result = formal_eskf::runtime::fuse_height(f.state, f.P, T{-10}, T{1}, f.minimum_norm,
                                                                      f.gate, true, target, target_P);
                test.expect(result.fusion.status == Status::non_finite_result && result.fusion.diagnostics_valid &&
                                result.fusion.decision == FusionDecision::failed && !result.innovation_limited &&
                                result.fusion.innovation(0) == T{-10} && same_state(target, before) &&
                                same_bits(target_P, before_P),
                            profile, "controlled height limit arithmetic failure retains raw diagnostics and outputs");
            }
            else
            {
                auto const result = f.fuse(target, target_P);
                test.expect(result.fusion.status == Status::non_finite_result && result.fusion.diagnostics_valid &&
                                result.fusion.decision == FusionDecision::failed && !result.innovation_limited &&
                                result.fusion.innovation(2) == T{-10} && same_state(target, before) &&
                                same_bits(target_P, before_P),
                            profile,
                            "controlled velocity limit arithmetic failure retains raw diagnostics and outputs");
            }
            B::scalar_math_type::oversized_root = false;
        }
    }
}
template <typename Linalg> void failures(Context & test, std::string_view profile)
{
    limit_failure<Linalg>(test, profile);
    enum class Failure
    {
        negative_speed_accuracy,
        non_finite_speed_accuracy,
        zero_speed_accuracy_limit,
        non_finite_speed_accuracy_limit,
        non_finite_measurement,
        zero_gate_sigma,
        non_finite_gate_sigma,
        non_finite_prior_covariance,
        negative_measurement_variance,
        invalid_norm_bound,
        indefinite_innovation_covariance,
        invalid_norm_with_zero_height_variance,
        innovation_square_overflow,
    };

    using F = GnssMeasurementFixture<Linalg>;
    using T = typename F::value_type;
    auto const inf = std::numeric_limits<T>::infinity();
    auto const nan = std::numeric_limits<T>::quiet_NaN();
    for (bool height : {false, true})
    {
        for (auto const failure :
             {Failure::negative_speed_accuracy, Failure::non_finite_speed_accuracy, Failure::zero_speed_accuracy_limit,
              Failure::non_finite_speed_accuracy_limit, Failure::non_finite_measurement, Failure::zero_gate_sigma,
              Failure::non_finite_gate_sigma, Failure::non_finite_prior_covariance,
              Failure::negative_measurement_variance, Failure::invalid_norm_bound,
              Failure::indefinite_innovation_covariance, Failure::invalid_norm_with_zero_height_variance,
              Failure::innovation_square_overflow})
        {
            if (height &&
                (failure == Failure::negative_speed_accuracy || failure == Failure::non_finite_speed_accuracy ||
                 failure == Failure::zero_speed_accuracy_limit || failure == Failure::non_finite_speed_accuracy_limit ||
                 failure == Failure::indefinite_innovation_covariance))
            {
                continue;
            }
            if (!height && failure == Failure::invalid_norm_with_zero_height_variance)
            {
                continue;
            }
            for (unsigned aliases = 0U; aliases < 4U; ++aliases)
            {
                F fixture;
                switch (failure)
                {
                case Failure::negative_speed_accuracy:
                    fixture.conditions.speed_accuracy = T{-1};
                    break;
                case Failure::non_finite_speed_accuracy:
                    fixture.conditions.speed_accuracy = nan;
                    break;
                case Failure::zero_speed_accuracy_limit:
                    fixture.conditions.maximum_speed_accuracy = T{0};
                    break;
                case Failure::non_finite_speed_accuracy_limit:
                    fixture.conditions.maximum_speed_accuracy = inf;
                    break;
                case Failure::non_finite_measurement:
                    fixture.velocity(2U) = nan;
                    break;
                case Failure::zero_gate_sigma:
                    fixture.gate = T{0};
                    break;
                case Failure::non_finite_gate_sigma:
                    fixture.gate = inf;
                    break;
                case Failure::non_finite_prior_covariance:
                    fixture.P(0U, 0U) = nan;
                    break;
                case Failure::negative_measurement_variance:
                    fixture.V(0U, 0U) = T{-1};
                    break;
                case Failure::invalid_norm_bound:
                    fixture.minimum_norm = T{2};
                    break; // Invalid normalization bound is rejected before gate diagnostics.
                case Failure::indefinite_innovation_covariance:
                    // Positive diagonal but indefinite S: the exception must not bypass LLT.
                    fixture.P(3U, 3U) = T{0};
                    fixture.V(0U, 1U) = T{8};
                    fixture.V(1U, 0U) = T{8};
                    break;
                case Failure::invalid_norm_with_zero_height_variance:
                    fixture.P(2U, 2U) = T{0};
                    fixture.minimum_norm = T{2};
                    break;
                case Failure::innovation_square_overflow:
                    fixture.velocity(2U) = std::numeric_limits<T>::max();
                    break; // Finite innovation, overflowing squared gate numerator.
                }
                auto output = fixture.state;
                auto output_P = fixture.P * T{2};
                output.p_n(0U) = T{7};
                auto & state_output = (aliases & 1U) ? fixture.state : output;
                auto & covariance_output = (aliases & 2U) ? fixture.P : output_P;
                auto const before_state = state_output;
                auto const before_P = covariance_output;
                auto const prior = fixture;
                auto check_failure = [&](auto const & result)
                {
                    Status const expected = failure == Failure::indefinite_innovation_covariance
                                                ? Status::not_positive_definite
                                            : failure == Failure::innovation_square_overflow ? Status::non_finite_result
                                            : failure == Failure::non_finite_speed_accuracy ||
                                                    failure == Failure::non_finite_speed_accuracy_limit ||
                                                    failure == Failure::non_finite_measurement ||
                                                    failure == Failure::non_finite_gate_sigma ||
                                                    failure == Failure::non_finite_prior_covariance
                                                ? Status::non_finite_input
                                                : Status::domain_error;
                    test.expect(
                        result.decision == FusionDecision::failed && result.status == expected &&
                            result.diagnostics_valid == (failure == Failure::indefinite_innovation_covariance) &&
                            same_state(fixture.state, prior.state) && same_bits(fixture.P, prior.P) &&
                            same_bits(fixture.velocity, prior.velocity) && same_bits(fixture.V, prior.V) &&
                            same_state(state_output, before_state) && same_bits(covariance_output, before_P),
                        profile,
                        "invalid metadata, gate arithmetic and downstream correction failures preserve outputs");
                };
                if (height)
                {
                    check_failure(formal_eskf::runtime::fuse_height(fixture.state, fixture.P, fixture.velocity(2U),
                                                                    fixture.V(0U, 0U), fixture.minimum_norm,
                                                                    fixture.gate, true, state_output, covariance_output)
                                      .fusion);
                }
                else
                {
                    check_failure(fixture.fuse(state_output, covariance_output).fusion);
                }
            }
        }
    }
    for (T deadzone : {T{-1}, std::numeric_limits<T>::infinity(), std::numeric_limits<T>::quiet_NaN()})
    {
        F fixture;
        auto const prior = fixture;
        auto const result =
            formal_eskf::runtime::fuse_height(fixture.state, fixture.P, T{1}, T{1}, fixture.minimum_norm, T{5}, false,
                                              fixture.state, fixture.P, {}, deadzone);
        test.expect(result.fusion.status == (deadzone < 0 ? Status::domain_error : Status::non_finite_input) &&
                        !result.fusion.diagnostics_valid && same_state(prior.state, fixture.state) &&
                        same_bits(prior.P, fixture.P),
                    profile, "invalid deadzone preserves whole-prior aliases and cannot expose gate evidence");
    }
}

template <typename Linalg, bool Height> void publication_fixture(Context & test, std::string_view profile, bool limited)
{
    using F = GnssMeasurementFixture<Linalg>;
    using T = typename F::value_type;
    for (bool fault : {false, true})
    {
        if (limited && !fault)
        {
            continue;
        }
        for (unsigned mask = 0; mask < 64U; ++mask)
        {
            for (unsigned aliases = 0; aliases < 4U; ++aliases)
            {
                F f;
                f.velocity(2) = limited ? T{-10} : T{1};
                f.conditions.bad_vertical_acceleration = fault;
                for (std::size_t axis = 0; axis < 6U; ++axis)
                {
                    auto const coefficient = static_cast<T>(axis + 1U) / T{16};
                    f.P(9U + axis, Height ? 2U : 5U) = coefficient;
                    f.P(Height ? 2U : 5U, 9U + axis) = coefficient;
                }
                f.V(0, 1) = f.V(1, 0) = T{1};
                f.V(1, 1) = T{1.25};
                Ins::BiasUpdate permission;
                for (std::size_t axis = 0; axis < 3; ++axis)
                {
                    permission.accelerometer[axis] = (mask & (1U << axis)) != 0U;
                    permission.gyroscope[axis] = (mask & (1U << (axis + 3U))) != 0U;
                }
                test.expect(F::state_type::quaternion_type::from_coefficients(T{0.5}, T{0.5}, T{0.5}, T{0.5}, T{0.125},
                                                                              f.state.q_nb) == Status::success,
                            profile, "exact nonidentity unit attitude fixture");
                for (std::size_t axis = 0; axis < 3; ++axis)
                {
                    f.state.b_a(axis) = static_cast<T>(axis + 1U) / T{8};
                    f.state.b_g(axis) = -static_cast<T>(axis + 1U) / T{16};
                }
                f.state.p_n(0) = T{2};
                f.state.p_n(1) = T{-3};
                auto const independent = reference_fixture<Linalg, Height>(f, limited ? T{-2} : T{1});
                auto const expected =
                    formal_eskf::test::reference::reference_correction(independent, ESKF_RESET_APPROX != 0, permission);
                auto generic_state = f.state;
                auto generic_P = f.P;
                auto const generic =
                    formal_eskf::runtime::fuse_measurement(f.state, f.P, independent.r, independent.H, independent.V,
                                                           f.minimum_norm, T{1}, generic_state, generic_P, permission);
                auto state = f.state;
                auto P = f.P;
                auto const before = f;
                auto & target = (aliases & 1U) ? f.state : state;
                auto & target_P = (aliases & 2U) ? f.P : P;
                auto const result = [&]()
                {
                    if constexpr (Height)
                    {
                        return formal_eskf::runtime::fuse_height(f.state, f.P, f.velocity(2), T{1}, f.minimum_norm,
                                                                 T{1}, fault, target, target_P, permission);
                    }
                    else
                    {
                        return formal_eskf::runtime::fuse_gnss_velocity(f.state, f.P, f.velocity, f.V, f.minimum_norm,
                                                                        T{1}, f.conditions, target, target_P,
                                                                        permission);
                    }
                }();
                test.expect(
                    generic.decision == FusionDecision::fused && result.fusion.decision == FusionDecision::fused &&
                        result.innovation_limited == limited &&
                        formal_eskf::test::reference::matches_reference(independent, expected, target, target_P,
                                                                        T{0}) &&
                        formal_eskf::test::reference::matches_reference(independent, expected, generic_state, generic_P,
                                                                        T{0}) &&
                        same_state(target, generic_state) && same_bits(target_P, generic_P),
                    profile,
                    "independent dyadic correlated oracle verifies selected-residual generic/navigation agreement and "
                    "all six bias permissions");
                test.expect(((aliases & 1U) || same_state(f.state, before.state)) &&
                                ((aliases & 2U) || same_bits(f.P, before.P)) &&
                                same_bits(f.velocity, before.velocity) && same_bits(f.V, before.V),
                            profile, "all alias arrangements preserve borrowed storage");
            }
        }
    }
}
template <typename Linalg> void ordinary(Context & test, std::string_view profile)
{
    publication_fixture<Linalg, false>(test, profile, false);
    publication_fixture<Linalg, true>(test, profile, false);
}

} // namespace
int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using F = formal_eskf::test::Backend<float>;
    using D = formal_eskf::test::Backend<double>;
    using formal_eskf::test::Case;
    Case const cases[]{
        {"TC-R-GNSS-VERTICAL-RECOVERY-01", "binary32", velocity_exception<F>},
        {"TC-R-GNSS-VERTICAL-RECOVERY-01", "binary64", velocity_exception<D>},
        {"TC-R-GNSS-VERTICAL-RECOVERY-02", "binary32", signed_diagnostics<F>},
        {"TC-R-GNSS-VERTICAL-RECOVERY-02", "binary64", signed_diagnostics<D>},
        {"TC-R-GNSS-VERTICAL-RECOVERY-03", "binary32", height_deadzone<F>},
        {"TC-R-GNSS-VERTICAL-RECOVERY-03", "binary64", height_deadzone<D>},
        {"TC-R-GNSS-VERTICAL-RECOVERY-04", "binary32", failures<F>},
        {"TC-R-GNSS-VERTICAL-RECOVERY-04", "binary64", failures<D>},
        {"TC-R-GNSS-VERTICAL-RECOVERY-05", "binary32", ordinary<F>},
        {"TC-R-GNSS-VERTICAL-RECOVERY-05", "binary64", ordinary<D>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
