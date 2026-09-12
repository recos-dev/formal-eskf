/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * E-STATE / E-PRED production checks. Compile separately for binary32/64 and
 * both ESKF_QUAT_APPROX modes. The source is not a replacement estimator.
 * Domains and conditional properties are stated at each entry point.
 */

#include <limits>

#include "support.hpp"
#include <formal_eskf/eskf/prediction.hpp>
#include <formal_eskf/eskf/correction.hpp>

#ifndef FORMAL_ESKF_PROOF_COEFFICIENT
#define FORMAL_ESKF_PROOF_COEFFICIENT 0
#endif
#ifndef FORMAL_ESKF_PROOF_ALIAS
#define FORMAL_ESKF_PROOF_ALIAS 0
#endif
#ifndef FORMAL_ESKF_PROOF_AXIS
#define FORMAL_ESKF_PROOF_AXIS 0
#endif
#ifndef FORMAL_ESKF_PROOF_SCENARIO
#define FORMAL_ESKF_PROOF_SCENARIO 0
#endif
#ifndef FORMAL_ESKF_PROOF_ATTITUDE_CONTRACT
#define FORMAL_ESKF_PROOF_ATTITUDE_CONTRACT 0
#endif

namespace prediction_proof
{

#if FORMAL_ESKF_PROOF_BINARY64
using Scalar = double;
#else
using Scalar = float;
#endif
template <std::size_t Size> using Scalars = formal_eskf::verification::ScalarArray<Scalar, Size>;
bool preserved_scalar(Scalar a, Scalar b);
// Passive observation only: norm arithmetic still runs unchanged. This avoids
// asking a solver to rediscover equivalence between two large sqrt circuits.
struct NormObserver
{
    inline static formal_eskf::verification::ScalarArray<Scalars<4U>, 2U> arguments{};
    inline static Scalars<2U> norms{};
    inline static unsigned count = 0U;
    inline static bool active = false;
    inline static Scalars<4U> initial_q{};
    inline static Scalars<9U> rotation{};
    inline static Scalars<3U> force{};
    inline static Scalars<3U> rotated_force{};
    inline static unsigned product_count = 0U;
    inline static formal_eskf::verification::ScalarArray<Scalars<3U>, 4U> add_left{}, add_right{}, add_result{};
    inline static unsigned add_count = 0U;
    inline static formal_eskf::verification::ScalarArray<Scalars<4U>, 2U> divided{};
    inline static unsigned division_count = 0U;
    inline static formal_eskf::verification::ScalarArray<Scalars<3U>, 5U> scale_input{}, scale_result{};
    inline static Scalars<5U> scale_factor{};
    inline static unsigned scale_count = 0U;

    // Stages have fixed storage slots. Each cut is asserted before it becomes
    // an assumption; a failed stage bound or correspondence fails the proof.
    template <std::size_t Stage, typename Values, typename Result>
    static void record_quotient(Values const & values, Scalar divisor, Result const & result)
    {
        static_assert(Stage < 2U);
        // The four coefficient profiles share one input domain. Together they
        // prove every division; no fact about another coefficient is assumed.
        constexpr std::size_t i = FORMAL_ESKF_PROOF_COEFFICIENT;
        static_assert(i < 4U);
        bool const division_matches = preserved_scalar(result[i], values[i] / divisor);
        __ESBMC_assert(division_matches, "E-PRED: normalization uses IEEE division for the profiled coefficient");
        __ESBMC_assume(division_matches);
        bool const operands_match =
            preserved_scalar(values[i], arguments[Stage][i]) && preserved_scalar(divisor, norms[Stage]);
        __ESBMC_assert(operands_match, "E-PRED: division uses the same candidate coefficient and computed norm");
        __ESBMC_assume(operands_match);
        for (std::size_t j = 0U; j < 4U; ++j)
        {
            divided[Stage][j] = result[j];
        }
    }

    template <std::size_t Size, typename Values, typename Result>
    static void quotient(Values const & values, Scalar divisor, Result const & result)
    {
        if constexpr (Size == 4U)
        {
            if (active)
            {
                __ESBMC_assert(division_count < 2U, "E-PRED: quotient trace is in bounds");
                __ESBMC_assume(division_count < 2U);
                if (division_count == 0U)
                {
                    record_quotient<0U>(values, divisor, result);
                }
                else
                {
                    record_quotient<1U>(values, divisor, result);
                }
                ++division_count;
            }
        }
    }

    template <std::size_t Size, typename Values, typename Result>
    static void scaling(Values const & values, Scalar factor, Result const & result)
    {
        if constexpr (Size == 3U)
        {
            __ESBMC_assert(scale_count < 5U, "E-PRED: scaling trace is in bounds");
            scale_factor[scale_count] = factor;
            for (std::size_t i = 0U; i < 3U; ++i)
            {
                __ESBMC_assert(preserved_scalar(result[i], values[i] * factor),
                               "E-PRED: scaling uses IEEE multiplication for each coordinate");
                scale_input[scale_count][i] = values[i];
                scale_result[scale_count][i] = result[i];
            }
            ++scale_count;
        }
    }

    template <std::size_t Size, typename Left, typename Right, typename Result>
    static void addition(Left const & left, Right const & right, Result const & result)
    {
        if constexpr (Size == 3U)
        {
            __ESBMC_assert(add_count < 4U, "E-PRED: addition trace is in bounds");
            for (std::size_t i = 0U; i < 3U; ++i)
            {
                __ESBMC_assert(preserved_scalar(result[i], left[i] + right[i]),
                               "E-PRED: each recorded addition obeys IEEE scalar addition");
                add_left[add_count][i] = left[i];
                add_right[add_count][i] = right[i];
                add_result[add_count][i] = result[i];
            }
            ++add_count;
        }
    }

    template <std::size_t Rows, std::size_t Inner, std::size_t Columns, typename Left, typename Right, typename Result>
    static void product(Left const & left, Right const & right, Result const & result)
    {
        if constexpr (Rows == 3U && Inner == 3U && Columns == 1U)
        {
            for (std::size_t i = 0U; i < 9U; ++i)
            {
                rotation[i] = left[i];
            }
            for (std::size_t i = 0U; i < 3U; ++i)
            {
                Scalar dot{0};
                for (std::size_t j = 0U; j < 3U; ++j)
                {
                    dot += left[3U * i + j] * right[j];
                }
                __ESBMC_assert(preserved_scalar(result[i], dot),
                               "E-PRED: actual rotation product is the specified ordered IEEE dot product");
                force[i] = right[i];
                rotated_force[i] = result[i];
            }
            ++product_count;
        }
    }

    template <std::size_t Size, typename Values> static void input(Values const & values)
    {
        if constexpr (Size == 4U)
        {
            if (active)
            {
                __ESBMC_assert(count < 2U, "E-PRED: at most two normalization calls per attitude prediction");
                __ESBMC_assume(count < 2U);
#if !ESKF_QUAT_APPROX
                if (count == 1U)
                {
                    __ESBMC_assert(division_count == 1U,
                                   "E-PRED: composition follows one completed normalization division");
                    __ESBMC_assume(division_count == 1U);
                    auto const & q = initial_q;
                    auto const & d = divided[0U];
                    Scalars<4U> h;
                    h[0] = q[0] * d[0] - q[1] * d[1] - q[2] * d[2] - q[3] * d[3];
                    h[1] = q[0] * d[1] + q[1] * d[0] + q[2] * d[3] - q[3] * d[2];
                    h[2] = q[0] * d[2] - q[1] * d[3] + q[2] * d[0] + q[3] * d[1];
                    h[3] = q[0] * d[3] + q[1] * d[2] - q[2] * d[1] + q[3] * d[0];
                    constexpr std::size_t i = FORMAL_ESKF_PROOF_COEFFICIENT;
                    __ESBMC_assert(preserved_scalar(values[i], h[i]),
                                   "E-PRED: composition is OLD quaternion times normalized Exp increment");
                }
#endif
                for (std::size_t i = 0U; i < 4U; ++i)
                {
                    if (count == 0U)
                    {
                        arguments[0U][i] = values[i];
                    }
                    else
                    {
                        arguments[1U][i] = values[i];
                    }
                }
            }
        }
    }

    static void result(Scalar value)
    {
        if (active)
        {
            __ESBMC_assert(count < 2U, "E-PRED: normalization observation is in bounds");
            __ESBMC_assume(count < 2U);
            if (count == 0U)
            {
                norms[0U] = value;
            }
            else
            {
                norms[1U] = value;
            }
            ++count;
        }
    }

    static void start(Scalar q0, Scalar q1, Scalar q2, Scalar q3)
    {
        count = 0U;
        initial_q[0] = q0;
        initial_q[1] = q1;
        initial_q[2] = q2;
        initial_q[3] = q3;
        active = true;
    }
};

// A separate scalar-boundary profile uses only sqrt sign/axis and trig range
// contracts. Interior results are symbolic; no accuracy, unit-circle identity
// or successful normalization is assumed. Actual StandardMath profiles remain
// separate. See the scalar-boundary contract in the prediction specification.
// All estimator code, arithmetic operators, comparisons and divisions remain
// production C++ with IEEE arithmetic. Real-function meaning is proved in Lean.
extern double nondet_double();
struct ScalarBoundary : formal_eskf::scalar::StandardMath<Scalar>
{
    inline static Scalars<4U> sqrt_arguments{};
    inline static Scalars<4U> sqrt_results{};
    inline static unsigned sqrt_count = 0U;
    inline static Scalar sin_argument{};
    inline static Scalar sin_result{};
    inline static Scalar cos_argument{};
    inline static Scalar cos_result{};

    static Scalar fresh()
    {
#if FORMAL_ESKF_PROOF_BINARY64
        return nondet_double();
#else
        return nondet_float();
#endif
    }
    // ESBMC recognizes a function named sqrt as an intrinsic, even on a user
    // math adapter. A callable data member preserves the API but ensures that
    // this explicitly declared contract body is actually executed.
    struct SquareRoot
    {
        Scalar operator()(Scalar input) const { return root_result(input); }
    };
    inline static constexpr SquareRoot sqrt{};

    template <std::size_t Slot> static void record_root(Scalar input, Scalar result)
    {
        static_assert(Slot < 4U);
        sqrt_arguments[Slot] = input;
        sqrt_results[Slot] = result;
    }

    static Scalar root_result(Scalar input)
    {
        __ESBMC_assert(sqrt_count < 4U, "E-PRED: scalar-boundary trace is in bounds");
        __ESBMC_assume(sqrt_count < 4U);
        Scalar value;
        if (input == Scalar{0} || input == Scalar{1} || input == std::numeric_limits<Scalar>::infinity())
        {
            value = input;
        }
        else if (!(input > Scalar{0}))
        {
            value = std::numeric_limits<Scalar>::quiet_NaN();
        }
        else
        {
            value = fresh();
            __ESBMC_assume(is_finite(value) && value > Scalar{0});
            __ESBMC_assume(input < Scalar{1} ? (value >= input && value <= Scalar{1})
                                             : (value >= Scalar{1} && value <= input));
            Scalar const square = value * value;
            __ESBMC_assume(square >= input * Scalar{0.5} && square <= input * Scalar{2});
        }
        switch (sqrt_count)
        {
        case 0U:
            record_root<0U>(input, value);
            break;
        case 1U:
            record_root<1U>(input, value);
            break;
        case 2U:
            record_root<2U>(input, value);
            break;
        case 3U:
            record_root<3U>(input, value);
            break;
        default:
            __ESBMC_assert(false, "E-PRED: impossible scalar trace slot");
        }
        ++sqrt_count;
        return value;
    }
    static Scalar sin(Scalar input)
    {
        sin_argument = input;
        sin_result = input == Scalar{0} ? input : fresh();
        __ESBMC_assume(sin_result >= Scalar{-1} && sin_result <= Scalar{1});
        return sin_result;
    }
    static Scalar cos(Scalar input)
    {
        cos_argument = input;
        cos_result = input == Scalar{0} ? Scalar{1} : fresh();
        __ESBMC_assume(cos_result >= Scalar{-1} && cos_result <= Scalar{1});
        return cos_result;
    }
};
#if FORMAL_ESKF_PROOF_SCALAR_BOUNDARY
using Linalg = formal_eskf::verification::FixedArrayLinalg<Scalar, NormObserver, ScalarBoundary>;
#else
using Linalg = formal_eskf::verification::FixedArrayLinalg<Scalar, NormObserver>;
#endif
using Math = Linalg::scalar_math_type;
using Vector = Linalg::vector_type<3U>;
using Quaternion = formal_eskf::so3::UnitQuaternion<Linalg>;
using Ins = formal_eskf::EskfTypes<Linalg, formal_eskf::configuration::Ins>;
using Ahrs = formal_eskf::EskfTypes<Linalg, formal_eskf::configuration::Ahrs>;
using Imu = formal_eskf::ImuSample<Linalg>;
using formal_eskf::Status;

// Verification-only callee summary. Its postconditions are discharged by the
// REAL helper in verify_attitude_euler / verify_attitude_exp_general, for both
// formats, every coefficient, and both Exp branches. Caller proofs deliberately
// know no coefficient equation: they must publish the opaque result unchanged.
// No production estimator or library source is replaced in deployed builds.
struct AttitudeContract
{
    inline static bool enabled = false;
    inline static bool called = false;
    inline static Quaternion result{};
    inline static Status status{};
    inline static Quaternion received_q{};
    inline static Vector received_rate{};
    inline static Scalar received_dt{};
    inline static Scalar received_minimum{};

    static void prepare(Quaternion const & candidate, Status candidate_status)
    {
        // This is the sole callee-postcondition assumption. It is NOT a
        // constructor-validity/unit-norm assumption and gives no error bound.
        __ESBMC_assume(candidate_status != Status::success ||
                       formal_eskf::linalg::all_finite(candidate.coefficients()));
        result = candidate;
        status = candidate_status;
        enabled = true;
        called = false;
    }
};

} // namespace prediction_proof

#if FORMAL_ESKF_PROOF_ATTITUDE_CONTRACT
namespace formal_eskf::detail
{
template <>
Status try_predict_attitude<prediction_proof::Linalg>(prediction_proof::Quaternion const & q,
                                                      prediction_proof::Vector const & rate,
                                                      prediction_proof::Scalar dt, prediction_proof::Scalar minimum,
                                                      prediction_proof::Quaternion & output) noexcept
{
    using namespace prediction_proof;
    __ESBMC_assert(AttitudeContract::enabled, "Runner error: attitude summary requires an explicit caller harness");
    __ESBMC_assert(!AttitudeContract::called, "E-PRED contract: caller delegates at most once");
    __ESBMC_assert(&q != &output, "E-PRED contract: caller supplies a distinct attitude candidate");
    auto const coefficients = q.coefficients();
    for (std::size_t i = 0U; i < 4U; ++i)
    {
        __ESBMC_assert(coefficients(i) >= Scalar{-1} && coefficients(i) <= Scalar{1},
                       "E-PRED contract: caller meets the proved quaternion domain");
    }
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        __ESBMC_assert(rate(i) >= Scalar{-2} && rate(i) <= Scalar{2},
                       "E-PRED contract: caller meets the proved corrected-rate domain");
    }
    __ESBMC_assert(dt >= Scalar{0.0009765625} && dt <= Scalar{1} && minimum == Scalar{0.125},
                   "E-PRED contract: caller meets the proved time and normalization domain");
    AttitudeContract::called = true;
    AttitudeContract::received_q = q;
    AttitudeContract::received_rate = rate;
    AttitudeContract::received_dt = dt;
    AttitudeContract::received_minimum = minimum;
    if (AttitudeContract::status == Status::success)
    {
        output = AttitudeContract::result;
    }
    // No write to output on failure, or to either input on any return. These
    // frame conditions are also proved for the actual helper and arbitrary
    // initial output; nothing else in the caller is summarized.
    return AttitudeContract::status;
}
} // namespace formal_eskf::detail
#endif

namespace prediction_proof
{

// Discharge the coarse root envelope used by ScalarBoundary against the
// verifier's actual IEEE sqrt, independently of the prediction profiles.
void verify_sqrt_envelope(Scalar input)
{
    __ESBMC_assume(std::isfinite(input) && input >= Scalar{0});
    Scalar const value = formal_eskf::scalar::StandardMath<Scalar>::sqrt(input);
    __ESBMC_assert(std::isfinite(value), "E-PRED scalar contract: finite nonnegative sqrt input has finite output");
    __ESBMC_assert(input == Scalar{0} ? value == Scalar{0} : value > Scalar{0},
                   "E-PRED scalar contract: sqrt preserves zero and is positive off zero");
    __ESBMC_assert(input < Scalar{1} ? (value >= input && value <= Scalar{1}) : (value >= Scalar{1} && value <= input),
                   "E-PRED scalar contract: sqrt lies between its input and one");
    __ESBMC_assert(input != Scalar{0} || std::signbit(value) == std::signbit(input),
                   "E-PRED scalar contract: sqrt preserves the sign of zero");
}

void verify_sqrt_squared_envelope(Scalar input)
{
    __ESBMC_assume(std::isfinite(input) && input >= Scalar{0});
    Scalar const value = formal_eskf::scalar::StandardMath<Scalar>::sqrt(input);
    Scalar const square = value * value;
    __ESBMC_assert(square >= input * Scalar{0.5} && square <= input * Scalar{2},
                   "E-PRED scalar contract: rounded sqrt squared has a conservative factor-two enclosure");
}

void verify_scalar_boundary_dispatch(Scalar input)
{
#if FORMAL_ESKF_PROOF_SCALAR_BOUNDARY
    __ESBMC_assume(std::isfinite(input) && input >= Scalar{0});
    Scalar const result = Math::sqrt(input);
    __ESBMC_assert(ScalarBoundary::sqrt_count == 1U && preserved_scalar(ScalarBoundary::sqrt_arguments[0], input) &&
                       preserved_scalar(ScalarBoundary::sqrt_results[0], result),
                   "E-PRED: the root contract body executes and records its actual argument and result");
#else
    (void)input;
    __ESBMC_assert(false, "Runner error: dispatch check requires the scalar-boundary profile");
#endif
}

void verify_storage_copy(Linalg::vector_type<15U> source)
{
    auto const original = source;
    auto copy = source;
    Linalg::vector_type<15U> assigned;
    assigned = source;
    for (std::size_t i = 0U; i < 15U; ++i)
    {
        __ESBMC_assert(preserved_scalar(copy(i), original(i)) && preserved_scalar(assigned(i), original(i)),
                       "E-STATE: proof storage copy construction and assignment preserve every IEEE value");
    }
    copy.set(0U, Scalar{42});
    assigned.set(14U, Scalar{-42});
    for (std::size_t i = 0U; i < 15U; ++i)
    {
        __ESBMC_assert(preserved_scalar(source(i), original(i)),
                       "E-STATE: copied matrix storage does not alias its source");
    }
}

// Value preservation includes signed zero and NaN classification. NaN payload
// bits are not modeled by ESBMC's scalar floating-point representation.
bool preserved_scalar(Scalar a, Scalar b)
{
    return (a == b && (a != Scalar{0} || std::signbit(a) == std::signbit(b))) || (std::isnan(a) && std::isnan(b));
}

template <std::size_t Size>
bool preserved_vector(Linalg::vector_type<Size> const & a, Linalg::vector_type<Size> const & b)
{
    bool same = true;
    for (std::size_t i = 0U; i < Size; ++i)
    {
        same = same && preserved_scalar(a(i), b(i));
    }
    return same;
}

template <typename State> bool preserved_state(State const & a, State const & b)
{
    bool same = preserved_vector(a.q_nb.coefficients(), b.q_nb.coefficients());
    if constexpr (requires { a.p_n; })
    {
        same = same && preserved_vector(a.p_n, b.p_n) && preserved_vector(a.v_n, b.v_n) &&
               preserved_vector(a.b_a, b.b_a) && preserved_vector(a.b_g, b.b_g);
    }
    return same;
}

template <typename Parameters> Parameters valid_parameters()
{
    Parameters parameters;
    parameters.dt_min = Scalar{0.0009765625};
    parameters.dt_max = Scalar{1};
    parameters.minimum_quaternion_norm = Scalar{0.125};
    return parameters;
}

void assume_bounded(Vector const & v)
{
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        __ESBMC_assume(v(i) >= Scalar{-1} && v(i) <= Scalar{1});
    }
}

} // namespace prediction_proof

using namespace prediction_proof;

// A compositional proof of the ACTUAL public AHRS wrapper, not its helper.
// Both compile-time alias profiles are required. The opaque callee result may
// be any finite quaternion on success and any IEEE values on failure.
void verify_ahrs_attitude_contract(Quaternion q, Vector rate, Scalar dt, Quaternion next, Status next_status,
                                   Ahrs::nominal_state_type separate)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_ATTITUDE_CONTRACT, "Runner error: AHRS caller requires the callee summary");
    auto const coefficients = q.coefficients();
    for (std::size_t i = 0U; i < 4U; ++i)
    {
        __ESBMC_assume(coefficients(i) >= Scalar{-1} && coefficients(i) <= Scalar{1});
    }
    assume_bounded(rate);
    __ESBMC_assume(dt >= Scalar{0.0009765625} && dt <= Scalar{1});
    AttitudeContract::prepare(next, next_status);
    Ahrs::nominal_state_type source;
    source.q_nb = q;
#if FORMAL_ESKF_PROOF_ALIAS
    auto & destination = source;
#else
    auto & destination = separate;
#endif
    auto const before = destination;
    Imu imu;
    imu.angular_rate_b = rate;
    auto parameters = valid_parameters<Ahrs::parameter_type>();
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        imu.specific_force_b.set(i, ScalarBoundary::fresh());
        parameters.process_noise.angular_rate_variance.set(i, ScalarBoundary::fresh());
    }
    auto const status = formal_eskf::try_predict_nominal(source, imu, dt, parameters, destination);
    __ESBMC_assert(AttitudeContract::called && status == next_status,
                   "E-PRED: public AHRS delegates and propagates the callee status");
    __ESBMC_assert(preserved_vector(AttitudeContract::received_q.coefficients(), q.coefficients()) &&
                       preserved_vector(AttitudeContract::received_rate, rate) && AttitudeContract::received_dt == dt &&
                       AttitudeContract::received_minimum == Scalar{0.125},
                   "E-PRED: public AHRS forwards the old attitude, sampled body rate and time/norm parameters");
    __ESBMC_assert(status != Status::success || preserved_vector(destination.q_nb.coefficients(), next.coefficients()),
                   "E-PRED: public AHRS publishes the callee result without changing any coefficient");
    __ESBMC_assert(status == Status::success || preserved_state(destination, before),
                   "E-PRED: public AHRS preserves its complete actual output on failure");
#if !FORMAL_ESKF_PROOF_ALIAS
    __ESBMC_assert(preserved_vector(source.q_nb.coefficients(), q.coefficients()),
                   "E-PRED: public AHRS never changes its distinct input");
#endif
}

void verify_state_defaults()
{
    static_assert(Ins::nominal_state_dimension == 16U && Ins::error_state_dimension == 15U &&
                  Ins::process_noise_dimension == 12U);
    static_assert(Ahrs::nominal_state_dimension == 4U && Ahrs::error_state_dimension == 3U &&
                  Ahrs::process_noise_dimension == 3U);
    static_assert(Ins::error_covariance_type::row_count == 15U && Ins::error_covariance_type::column_count == 15U);
    static_assert(Ins::process_noise_covariance_type::row_count == 12U);
    static_assert(Ahrs::error_covariance_type::row_count == 3U && Ahrs::error_covariance_type::column_count == 3U);
    static_assert(Ahrs::process_noise_covariance_type::row_count == 3U);
    Ins::nominal_state_type ins;
    Ahrs::nominal_state_type ahrs;
    Ins::error_state_type ins_error;
    Ahrs::error_state_type ahrs_error;
    Imu imu;
    Vector zero;
    __ESBMC_assert(preserved_vector(ins.p_n, zero) && preserved_vector(ins.v_n, zero) &&
                       preserved_vector(ins.b_a, zero) && preserved_vector(ins.b_g, zero),
                   "E-STATE: INS Euclidean defaults are zero");
    __ESBMC_assert(preserved_vector(ins.q_nb.coefficients(), Quaternion::identity().coefficients()) &&
                       preserved_vector(ahrs.q_nb.coefficients(), Quaternion::identity().coefficients()),
                   "E-STATE: both configurations default to scalar-first identity");
    __ESBMC_assert(preserved_vector(ins_error.delta_p_n, zero) && preserved_vector(ins_error.delta_v_n, zero) &&
                       preserved_vector(ins_error.delta_theta_b, zero) && preserved_vector(ins_error.delta_b_a, zero) &&
                       preserved_vector(ins_error.delta_b_g, zero) && preserved_vector(ahrs_error.delta_theta_b, zero),
                   "E-STATE: every error coordinate defaults to zero");
    __ESBMC_assert(preserved_vector(imu.specific_force_b, zero) && preserved_vector(imu.angular_rate_b, zero),
                   "E-STATE: IMU rate fields default to zero");
}

// No finite restriction: also check signed zero and NaN classification.
void verify_state_unpack(Linalg::vector_type<15U> delta_x)
{
    Ins::error_state_type error;
    formal_eskf::detail::unpack_correction(delta_x, error);
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        __ESBMC_assert(preserved_scalar(error.delta_p_n(i), delta_x(i)) &&
                           preserved_scalar(error.delta_v_n(i), delta_x(3U + i)) &&
                           preserved_scalar(error.delta_theta_b(i), delta_x(6U + i)) &&
                           preserved_scalar(error.delta_b_a(i), delta_x(9U + i)) &&
                           preserved_scalar(error.delta_b_g(i), delta_x(12U + i)),
                       "E-STATE: all five INS error blocks have the specified coordinates");
    }
    Ahrs::error_state_type ahrs_error;
    auto const attitude = delta_x.segment<6U, 3U>();
    formal_eskf::detail::unpack_correction(attitude, ahrs_error);
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        Scalar const actual = ahrs_error.delta_theta_b(i);
        Scalar const expected = attitude(i);
        __ESBMC_assert(preserved_scalar(actual, expected),
                       "E-STATE: AHRS error is exactly three local attitude coordinates");
    }
}

void verify_prediction_parameters(Scalar dt, Scalar dt_min, Scalar dt_max, Scalar minimum_norm)
{
    Status const actual = formal_eskf::detail::validate_prediction_parameters<Math>(dt, dt_min, dt_max, minimum_norm);
    bool const finite =
        Math::is_finite(dt) && Math::is_finite(dt_min) && Math::is_finite(dt_max) && Math::is_finite(minimum_norm);
    bool const domain = dt_min > Scalar{0} && dt_max >= dt_min && minimum_norm > Scalar{0} && minimum_norm <= Scalar{1};
    bool const interval = dt >= dt_min && dt <= dt_max;
    __ESBMC_assert((actual == Status::success) == (finite && domain && interval),
                   "E-PRED: acceptance iff finite valid parameters and inclusive step bounds");
    __ESBMC_assert(finite || actual == Status::non_finite_input,
                   "E-PRED: non-finite parameters have first error priority");
    __ESBMC_assert(!finite || domain || actual == Status::domain_error,
                   "E-PRED: invalid parameter domains are reported");
    __ESBMC_assert(!finite || !domain || interval || actual == Status::out_of_range,
                   "E-PRED: invalid step interval is reported");
}

// Parameter rejection precedes state/IMU processing. Prior and output fields
// are arbitrary IEEE values, not identity sentinels or assumed-valid states.
void verify_prediction_parameter_rejection(Scalar dt, Scalar dt_min, Scalar dt_max, Scalar minimum_norm, bool alias,
                                           Ins::nominal_state_type ins, Ahrs::nominal_state_type ahrs, Imu imu,
                                           Ins::nominal_state_type oi, Ahrs::nominal_state_type oa)
{
    Status const expected = formal_eskf::detail::validate_prediction_parameters<Math>(dt, dt_min, dt_max, minimum_norm);
    __ESBMC_assume(expected != Status::success);
    Ins::parameter_type pi;
    Ahrs::parameter_type pa;
    pi.dt_min = pa.dt_min = dt_min;
    pi.dt_max = pa.dt_max = dt_max;
    pi.minimum_quaternion_norm = pa.minimum_quaternion_norm = minimum_norm;
    auto const before_i = alias ? ins : oi;
    auto const before_a = alias ? ahrs : oa;
    Status const si = formal_eskf::try_predict_nominal(ins, imu, dt, pi, alias ? ins : oi);
    Status const sa = formal_eskf::try_predict_nominal(ahrs, imu, dt, pa, alias ? ahrs : oa);
    __ESBMC_assert(si == expected && sa == expected && preserved_state(alias ? ins : oi, before_i) &&
                       preserved_state(alias ? ahrs : oa, before_a),
                   "E-PRED: both public APIs preserve output on parameter rejection, including aliases");
}

// A symbolic non-finite value is placed in each consumed INS scalar field.
void verify_ins_non_finite_input(Scalar invalid, unsigned field, bool alias, Ins::nominal_state_type output)
{
    __ESBMC_assume(!Math::is_finite(invalid));
    __ESBMC_assume(field < 21U);
    Ins::nominal_state_type state;
    Imu imu;
    auto parameters = valid_parameters<Ins::parameter_type>();
    Vector * fields[] = {&state.p_n,          &state.v_n,           &state.b_a, &state.b_g, &imu.specific_force_b,
                         &imu.angular_rate_b, &parameters.gravity_n};
    fields[field / 3U]->set(field % 3U, invalid);
    auto const before = alias ? state : output;
    Status const status = formal_eskf::try_predict_nominal(state, imu, Scalar{0.5}, parameters, alias ? state : output);
    __ESBMC_assert(status == Status::non_finite_input && preserved_state(alias ? state : output, before),
                   "E-PRED: every non-finite consumed INS vector field is rejected atomically");
}

// General bounded translation through the ACTUAL INS wrapper. Only its
// attitude helper uses the separately discharged summary; rotation, bias
// subtraction, integration, validation and publishing all execute normally.
void verify_ins_translation(Vector const position_n, Vector const velocity_n, Vector const bias_a, Vector const force_b,
                            Vector const gravity, Quaternion q, Vector const rate_b, Vector const bias_g, Scalar dt,
                            Quaternion next, Status next_status, Ins::nominal_state_type output)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_ATTITUDE_CONTRACT, "Runner error: INS caller requires the callee summary");
    assume_bounded(position_n);
    assume_bounded(velocity_n);
    assume_bounded(bias_a);
    assume_bounded(force_b);
    assume_bounded(gravity);
    assume_bounded(rate_b);
    assume_bounded(bias_g);
    __ESBMC_assume(q.q0() >= Scalar{-1} && q.q0() <= Scalar{1});
    __ESBMC_assume(q.q1() >= Scalar{-1} && q.q1() <= Scalar{1});
    __ESBMC_assume(q.q2() >= Scalar{-1} && q.q2() <= Scalar{1});
    __ESBMC_assume(q.q3() >= Scalar{-1} && q.q3() <= Scalar{1});
    __ESBMC_assume(dt >= Scalar{0.0009765625} && dt <= Scalar{1});
    AttitudeContract::prepare(next, next_status);
    Ins::nominal_state_type state;
    state.p_n = position_n;
    state.v_n = velocity_n;
    state.b_a = bias_a;
    Imu imu;
    imu.specific_force_b = force_b;
    auto parameters = valid_parameters<Ins::parameter_type>();
    parameters.gravity_n = gravity;
    state.q_nb = q;
    state.b_g = bias_g;
    imu.angular_rate_b = rate_b;
    auto const before = state;
    constexpr std::size_t i = FORMAL_ESKF_PROOF_AXIS;
    static_assert(i < 3U);
    // Evaluate the independent oracle before the call, including in-place use.
    // Write the old quaternion's matrix row independently; preserve the
    // backend's dot-product order rather than applying real-only rewrites.
    Scalar const q0 = state.q_nb.q0();
    Scalar const q1 = state.q_nb.q1();
    Scalar const q2 = state.q_nb.q2();
    Scalar const q3 = state.q_nb.q3();
    Scalars<3U> row;
#if FORMAL_ESKF_PROOF_AXIS == 0
    row[0] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    row[1] = Scalar{2} * (q1 * q2 - q0 * q3);
    row[2] = Scalar{2} * (q1 * q3 + q0 * q2);
#elif FORMAL_ESKF_PROOF_AXIS == 1
    row[0] = Scalar{2} * (q1 * q2 + q0 * q3);
    row[1] = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
    row[2] = Scalar{2} * (q2 * q3 - q0 * q1);
#else
    row[0] = Scalar{2} * (q1 * q3 - q0 * q2);
    row[1] = Scalar{2} * (q2 * q3 + q0 * q1);
    row[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
#endif
    Scalar const old_position = before.p_n(i);
    Scalar const old_velocity = before.v_n(i);
#if FORMAL_ESKF_PROOF_ALIAS
    auto & result = state;
#else
    auto & result = output;
#endif
    auto const before_output = result;
    Status const status = formal_eskf::try_predict_nominal(state, imu, dt, parameters, result);
    __ESBMC_assert(AttitudeContract::called, "E-PRED: bounded INS inputs reach the attitude helper");
    // This proved cut avoids rediscovering input validation in every later
    // output/status obligation. Its assertion remains part of the proof.
    __ESBMC_assume(AttitudeContract::called);
    __ESBMC_assert(preserved_vector(AttitudeContract::received_q.coefficients(), q.coefficients()) &&
                       AttitudeContract::received_dt == dt && AttitudeContract::received_minimum == Scalar{0.125},
                   "E-PRED: INS delegates the old attitude and unchanged time/norm parameters");
    for (std::size_t j = 0U; j < 3U; ++j)
    {
        __ESBMC_assert(preserved_scalar(AttitudeContract::received_rate(j), rate_b(j) - bias_g(j)),
                       "E-PRED: INS delegates sampled body rate minus the OLD gyro bias on every axis");
    }
    bool finite_translation = true;
    for (std::size_t j = 0U; j < 3U; ++j)
    {
        finite_translation = finite_translation && std::isfinite(NormObserver::add_result[0U][j]) &&
                             std::isfinite(NormObserver::add_result[2U][j]) &&
                             std::isfinite(NormObserver::add_result[3U][j]);
    }
    Status const expected_status =
        next_status == Status::success && !finite_translation ? Status::non_finite_result : next_status;
    __ESBMC_assert(status == expected_status,
                   "E-PRED: INS status exactly reflects callee failure or finite translation results");
    Ins::nominal_state_type const & observed = result;
    if (status == Status::success)
    {
        __ESBMC_assert(formal_eskf::linalg::all_finite(observed.p_n) && formal_eskf::linalg::all_finite(observed.v_n) &&
                           formal_eskf::linalg::all_finite(observed.q_nb.coefficients()),
                       "E-PRED: a successful INS prediction publishes only finite computed state fields");
        __ESBMC_assert(preserved_vector(observed.q_nb.coefficients(), next.coefficients()),
                       "E-PRED: INS publishes the callee attitude without modifying any coefficient");
        __ESBMC_assert(NormObserver::product_count == 1U,
                       "E-PRED: translation applies one body-to-navigation rotation");
        for (std::size_t j = 0U; j < 3U; ++j)
        {
            __ESBMC_assert(preserved_scalar(NormObserver::rotation[3U * i + j], row[j]),
                           "E-PRED: specific force rotation uses the OLD quaternion matrix");
            __ESBMC_assert(preserved_scalar(NormObserver::force[j], force_b(j) - bias_a(j)),
                           "E-PRED: rotation consumes measured specific force minus OLD accelerometer bias");
        }
        __ESBMC_assert(NormObserver::add_count == 4U, "E-PRED: translation completes four vector additions");
        __ESBMC_assert(NormObserver::scale_count == 3U, "E-PRED: translation completes three vector scalings");
        __ESBMC_assert(preserved_scalar(NormObserver::add_left[0][i], NormObserver::rotated_force[i]) &&
                           preserved_scalar(NormObserver::add_right[0][i], gravity(i)),
                       "E-PRED: navigation acceleration adds gravity after rotating specific force");
        __ESBMC_assert(preserved_scalar(NormObserver::add_left[1][i], old_position) &&
                           preserved_scalar(NormObserver::scale_input[0U][i], old_velocity) &&
                           preserved_scalar(NormObserver::scale_factor[0U], dt) &&
                           preserved_scalar(NormObserver::add_right[1][i], NormObserver::scale_result[0U][i]),
                       "E-PRED: position first adds OLD velocity times dt");
        __ESBMC_assert(preserved_scalar(NormObserver::add_left[2][i], NormObserver::add_result[1][i]) &&
                           preserved_scalar(NormObserver::scale_input[1U][i], NormObserver::add_result[0][i]) &&
                           preserved_scalar(NormObserver::scale_factor[1U], (Scalar{0.5} * dt) * dt) &&
                           preserved_scalar(NormObserver::add_right[2][i], NormObserver::scale_result[1U][i]),
                       "E-PRED: position adds navigation acceleration times one-half dt squared");
        __ESBMC_assert(preserved_scalar(NormObserver::add_left[3][i], old_velocity) &&
                           preserved_scalar(NormObserver::scale_input[2U][i], NormObserver::add_result[0][i]) &&
                           preserved_scalar(NormObserver::scale_factor[2U], dt) &&
                           preserved_scalar(NormObserver::add_right[3][i], NormObserver::scale_result[2U][i]),
                       "E-PRED: velocity adds navigation acceleration times dt");
        __ESBMC_assert(preserved_scalar(observed.p_n(i), NormObserver::add_result[2][i]) &&
                           preserved_scalar(observed.v_n(i), NormObserver::add_result[3][i]),
                       "E-PRED: successful prediction publishes the integrated position and velocity");
        __ESBMC_assert(preserved_vector(result.b_a, before.b_a) && preserved_vector(result.b_g, before.b_g),
                       "E-PRED: nominal biases are unchanged on success");
    }
    __ESBMC_assert(status == Status::success || preserved_state(result, before_output),
                   "E-PRED: every failed symbolic INS prediction preserves its complete output");
#if !FORMAL_ESKF_PROOF_ALIAS
    __ESBMC_assert(preserved_state(state, before), "E-PRED: distinct input state is unchanged on every return path");
#endif
}

// A success witness in the symbolic translation domain, with changing attitude
// and nonzero acceleration on every axis. Also exercises successful in-place use.
void verify_ins_translation_execution()
{
    Ins::nominal_state_type state;
    auto const construction = Quaternion::try_from_coefficients(Scalar{0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.5},
                                                                Scalar{0.125}, state.q_nb);
    __ESBMC_assert(construction == Status::success, "E-PRED: translation execution fixture is admissible");
    state.p_n.set(0U, Scalar{0.25});
    state.p_n.set(1U, Scalar{0.5});
    state.p_n.set(2U, Scalar{1});
    state.v_n.set(0U, Scalar{-0.5});
    state.v_n.set(1U, Scalar{0.25});
    state.v_n.set(2U, Scalar{0.5});
    state.b_a.set(0U, Scalar{-0.25});
    state.b_a.set(1U, Scalar{0.25});
    state.b_a.set(2U, Scalar{0.5});
    state.b_g.set(0U, Scalar{0.25});
    state.b_g.set(1U, Scalar{-0.5});
    state.b_g.set(2U, Scalar{0.5});
    Imu imu;
    imu.specific_force_b.set(0U, Scalar{0.5});
    imu.specific_force_b.set(1U, Scalar{-0.5});
    imu.specific_force_b.set(2U, Scalar{1});
    imu.angular_rate_b.set(0U, Scalar{0.75});
    imu.angular_rate_b.set(1U, Scalar{-0.5});
    imu.angular_rate_b.set(2U, Scalar{0.5});
    auto parameters = valid_parameters<Ins::parameter_type>();
    parameters.gravity_n.set(0U, Scalar{0.125});
    parameters.gravity_n.set(1U, Scalar{0.25});
    parameters.gravity_n.set(2U, Scalar{1});
    auto const status = formal_eskf::try_predict_nominal(state, imu, Scalar{0.5}, parameters, state);
    __ESBMC_assert(status == Status::success && state.p_n(0U) == Scalar{0.078125} && state.p_n(1U) == Scalar{0.75} &&
                       state.p_n(2U) == Scalar{1.28125} && state.v_n(0U) == Scalar{-0.1875} &&
                       state.v_n(1U) == Scalar{0.75} && state.v_n(2U) == Scalar{0.625},
                   "E-PRED: nonzero bounded translation succeeds in place with independent exact coordinates");
}

// All IEEE quaternion/rate values that fail the consumed-input finite check.
void verify_attitude_non_finite(Quaternion q, Vector rate, Quaternion output)
{
    __ESBMC_assume(!formal_eskf::linalg::all_finite(q.coefficients()) || !formal_eskf::linalg::all_finite(rate));
    auto const before = output;
    Status const status = formal_eskf::detail::try_predict_attitude(q, rate, Scalar{0.5}, Scalar{0.125}, output);
    __ESBMC_assert(status == Status::non_finite_input && preserved_vector(output.coefficients(), before.coefficients()),
                   "E-PRED: non-finite attitude/rate is rejected before arithmetic");
}

// Check the public callers as well as the helper: non-finite attitude/rate
// rejection must reach both APIs without publishing any nominal-state field.
void verify_prediction_attitude_non_finite(Quaternion q, Vector rate, bool alias, Ins::nominal_state_type oi,
                                           Ahrs::nominal_state_type oa)
{
    __ESBMC_assume(!formal_eskf::linalg::all_finite(q.coefficients()) || !formal_eskf::linalg::all_finite(rate));
    Ins::nominal_state_type ins;
    Ahrs::nominal_state_type ahrs;
    ins.q_nb = ahrs.q_nb = q;
    Imu imu;
    imu.angular_rate_b = rate;
    auto const pi = valid_parameters<Ins::parameter_type>();
    auto const pa = valid_parameters<Ahrs::parameter_type>();
    auto const before_i = alias ? ins : oi;
    auto const before_a = alias ? ahrs : oa;
    auto const si = formal_eskf::try_predict_nominal(ins, imu, Scalar{0.5}, pi, alias ? ins : oi);
    auto const sa = formal_eskf::try_predict_nominal(ahrs, imu, Scalar{0.5}, pa, alias ? ahrs : oa);
    __ESBMC_assert(si == Status::non_finite_input && sa == Status::non_finite_input &&
                       preserved_state(alias ? ins : oi, before_i) && preserved_state(alias ? ahrs : oa, before_a),
                   "E-PRED: public attitude/rate rejection preserves every output field, including aliases");
}

// A successful boundary witness: equal time bounds and a normalization norm
// exactly equal to the maximum valid threshold are accepted by both APIs.
// A concrete success witness for the inclusive API boundaries. Symbolic output
// preservation and aliasing are checked separately by the failure harnesses.
void verify_prediction_boundary_success()
{
    Ins::nominal_state_type ins;
    Ahrs::nominal_state_type ahrs;
    Ins::nominal_state_type oi;
    Ahrs::nominal_state_type oa;
    oi.q_nb = -Quaternion::identity();
    oa.q_nb = -Quaternion::identity();
    Imu imu;
    auto pi = valid_parameters<Ins::parameter_type>();
    auto pa = valid_parameters<Ahrs::parameter_type>();
    pi.dt_min = pi.dt_max = pa.dt_min = pa.dt_max = Scalar{0.125};
    pi.minimum_quaternion_norm = pa.minimum_quaternion_norm = Scalar{1};
    pi.gravity_n.set(2U, Scalar{1});
    imu.specific_force_b.set(2U, Scalar{-1});
    auto const before_i = ins;
    auto const before_a = ahrs;
    auto const si = formal_eskf::try_predict_nominal(ins, imu, Scalar{0.125}, pi, oi);
    // The passive scaling trace is sized for one call, not a batch of calls.
    NormObserver::scale_count = 0U;
    auto const sa = formal_eskf::try_predict_nominal(ahrs, imu, Scalar{0.125}, pa, oa);
    __ESBMC_assert(si == Status::success && sa == Status::success && preserved_state(oi, before_i) &&
                       preserved_state(oa, before_a),
                   "E-PRED: inclusive equal time bounds and exact normalization threshold accept a valid step");
}

// AHRS consumes no accelerometer sample or process-noise parameter. Their
// contents stay completely symbolic, including NaN/Inf, in a successful call.
void verify_ahrs_zero_rate(Imu imu, Ahrs::parameter_type parameters, Scalar dt, bool alias)
{
    __ESBMC_assume(dt >= Scalar{0.0009765625} && dt <= Scalar{1});
    parameters.dt_min = Scalar{0.0009765625};
    parameters.dt_max = Scalar{1};
    parameters.minimum_quaternion_norm = Scalar{0.125};
    imu.angular_rate_b = Vector::zero();
    Ahrs::nominal_state_type state;
    Ahrs::nominal_state_type output;
    output.q_nb = -Quaternion::identity();
    auto const before = state;
    Status const status = formal_eskf::try_predict_nominal(state, imu, dt, parameters, alias ? state : output);
    __ESBMC_assert(status == Status::success && preserved_state(alias ? state : output, before),
                   "E-PRED: zero body rate preserves identity for every profiled dt; unused AHRS inputs are ignored");
}

void verify_ins_gyro_bias(Vector bias, Scalar dt)
{
    assume_bounded(bias);
    __ESBMC_assume(dt >= Scalar{0.0009765625} && dt <= Scalar{1});
    Ins::nominal_state_type state;
    Imu imu;
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        state.b_g.set(i, bias(i));
        imu.angular_rate_b.set(i, bias(i));
    }
    auto const parameters = valid_parameters<Ins::parameter_type>();
    auto const before = state;
    Status const status = formal_eskf::try_predict_nominal(state, imu, dt, parameters, state);
    __ESBMC_assert(status == Status::success && preserved_state(state, before),
                   "E-PRED: all gyro-bias components are subtracted, not added or omitted");
}

// Arbitrary finite q coefficients in [-1,1], corrected rates in [-2,2], and
// arbitrary initial output (including NaN/Inf). This is a coefficient and
// status correspondence claim, not a premise that every q here is unit.
// Independent numerator equations correspond to eulerCandidate_coefficients.
// A passive observer records the actual norm input/result, not a model return.
void verify_attitude_euler(Quaternion q, Vector sample, Scalar dt, Quaternion output)
{
#if ESKF_QUAT_APPROX
    __ESBMC_assert(!FORMAL_ESKF_PROOF_ATTITUDE_CONTRACT, "Runner error: Euler proof must execute the REAL helper");
    __ESBMC_assume(q.q0() >= Scalar{-1} && q.q0() <= Scalar{1});
    __ESBMC_assume(q.q1() >= Scalar{-1} && q.q1() <= Scalar{1});
    __ESBMC_assume(q.q2() >= Scalar{-1} && q.q2() <= Scalar{1});
    __ESBMC_assume(q.q3() >= Scalar{-1} && q.q3() <= Scalar{1});
    for (std::size_t j = 0U; j < 3U; ++j)
    {
        __ESBMC_assume(sample(j) >= Scalar{-2} && sample(j) <= Scalar{2});
    }
    auto const rate = sample;
    __ESBMC_assume(dt >= Scalar{0.0009765625} && dt <= Scalar{1});
    Scalar const x = (rate(0U) * dt) * Scalar{0.5};
    Scalar const y = (rate(1U) * dt) * Scalar{0.5};
    Scalar const z = (rate(2U) * dt) * Scalar{0.5};
    Scalars<4U> expected;
    expected[0] = q.q0() - q.q1() * x - q.q2() * y - q.q3() * z;
    expected[1] = q.q1() + q.q0() * x + q.q2() * z - q.q3() * y;
    expected[2] = q.q2() + q.q0() * y + q.q3() * x - q.q1() * z;
    expected[3] = q.q3() + q.q0() * z + q.q1() * y - q.q2() * x;
    auto const before = output;
    auto const q_before = q;
    auto const rate_before = sample;
    NormObserver::start(q.q0(), q.q1(), q.q2(), q.q3());
    Status const status = formal_eskf::detail::try_predict_attitude(q, sample, dt, Scalar{0.125}, output);
    __ESBMC_assert(preserved_vector(q.coefficients(), q_before.coefficients()) && preserved_vector(sample, rate_before),
                   "E-PRED: the attitude helper preserves both distinct input objects");
    __ESBMC_assert(status != Status::success || formal_eskf::linalg::all_finite(output.coefficients()),
                   "E-PRED: successful Euler prediction has finite output coefficients");
    __ESBMC_assert(status != Status::success || NormObserver::count == 1U,
                   "E-PRED: successful Euler prediction has exactly one normalization");
    if (NormObserver::count == 1U)
    {
        auto const coefficients = output.coefficients();
        constexpr std::size_t i = FORMAL_ESKF_PROOF_COEFFICIENT;
        static_assert(i < 4U);
        // All four profiles share the same symbolic input domain.
        {
            __ESBMC_assert(preserved_scalar(NormObserver::arguments[0][i], expected[i]),
                           "E-PRED: production normalization receives the specified Euler numerator");
            if (status == Status::success)
            {
                __ESBMC_assert(preserved_scalar(coefficients(i), expected[i] / NormObserver::norms[0]),
                               "E-PRED: successful Euler output divides every numerator by the computed norm");
            }
        }
    }
    __ESBMC_assert(status == Status::success || preserved_vector(output.coefficients(), before.coefficients()),
                   "E-PRED: unsuccessful Euler prediction leaves output unchanged");
#else
    (void)q;
    (void)sample;
    (void)dt;
    (void)output;
    __ESBMC_assert(false, "Runner error: Euler harness must only run with ESKF_QUAT_APPROX=1");
#endif
}

// General Exp/Taylor branch and Hamilton-composition correspondence. This is
// NOT a second Exp implementation used to generate a reference answer: it
// checks independently stated scalar equations against the SAME call's trace.
void verify_attitude_exp_general(Quaternion const q, Vector const sample, Scalar dt, Quaternion output)
{
#if !ESKF_QUAT_APPROX && FORMAL_ESKF_PROOF_SCALAR_BOUNDARY
    __ESBMC_assert(!FORMAL_ESKF_PROOF_ATTITUDE_CONTRACT, "Runner error: Exp proof must execute the REAL helper");
    __ESBMC_assume(q.q0() >= Scalar{-1} && q.q0() <= Scalar{1});
    __ESBMC_assume(q.q1() >= Scalar{-1} && q.q1() <= Scalar{1});
    __ESBMC_assume(q.q2() >= Scalar{-1} && q.q2() <= Scalar{1});
    __ESBMC_assume(q.q3() >= Scalar{-1} && q.q3() <= Scalar{1});
    for (std::size_t j = 0U; j < 3U; ++j)
    {
        __ESBMC_assume(sample(j) >= Scalar{-2} && sample(j) <= Scalar{2});
    }
    auto const rate = sample;
    __ESBMC_assume(dt >= Scalar{0.0009765625} && dt <= Scalar{1});
    Scalars<3U> theta;
    Scalar theta_squared{0};
    for (std::size_t j = 0U; j < 3U; ++j)
    {
        theta[j] = rate(j) * dt;
        theta_squared += theta[j] * theta[j];
    }
#if FORMAL_ESKF_PROOF_TAYLOR
    __ESBMC_assume(theta_squared <= std::numeric_limits<Scalar>::epsilon());
#else
    __ESBMC_assume(theta_squared > std::numeric_limits<Scalar>::epsilon());
#endif
    auto const before = output;
    auto const q_before = q;
    auto const rate_before = sample;
    NormObserver::start(q.q0(), q.q1(), q.q2(), q.q3());
    auto const status = formal_eskf::detail::try_predict_attitude(q, sample, dt, Scalar{0.125}, output);
    __ESBMC_assert(preserved_vector(q.coefficients(), q_before.coefficients()) && preserved_vector(sample, rate_before),
                   "E-PRED: the attitude helper preserves both distinct input objects");
    __ESBMC_assert(status != Status::success || formal_eskf::linalg::all_finite(output.coefficients()),
                   "E-PRED: successful Exp prediction has finite output coefficients");
    constexpr std::size_t i = FORMAL_ESKF_PROOF_COEFFICIENT;
    static_assert(i < 4U);
    if (NormObserver::count != 0U)
    {
        Scalar scalar_part;
        Scalar scale;
        if (theta_squared <= std::numeric_limits<Scalar>::epsilon())
        {
            Scalar const fourth = theta_squared * theta_squared;
            scalar_part = Scalar{1} - theta_squared / Scalar{8} + fourth / Scalar{384};
            scale = Scalar{0.5} - theta_squared / Scalar{48} + fourth / Scalar{3840};
        }
        else
        {
            __ESBMC_assert(preserved_scalar(ScalarBoundary::sqrt_arguments[0], theta_squared),
                           "E-PRED: Exp root receives the ordered squared rotation-vector norm");
            Scalar const angle = Scalar{0.5} * ScalarBoundary::sqrt_results[0];
            __ESBMC_assert(preserved_scalar(ScalarBoundary::sin_argument, angle) &&
                               preserved_scalar(ScalarBoundary::cos_argument, angle),
                           "E-PRED: both trig primitives receive the half angle");
            scalar_part = ScalarBoundary::cos_result;
            scale = ScalarBoundary::sin_result / ScalarBoundary::sqrt_results[0];
        }
        Scalar expected_increment;
        if constexpr (i == 0U)
        {
            expected_increment = scalar_part;
        }
        else
        {
            expected_increment = scale * theta[i - 1U];
        }
        __ESBMC_assert(preserved_scalar(NormObserver::arguments[0][i], expected_increment),
                       "E-PRED: Exp normalization receives the specified scalar/vector branch coefficient");
    }
    if (NormObserver::count == 2U)
    {
        if (status == Status::success)
        {
            auto const coefficients = output.coefficients();
            __ESBMC_assert(preserved_scalar(coefficients(i), NormObserver::divided[1][i]),
                           "E-PRED: Exp prediction publishes the checked normalized Hamilton coefficient");
        }
    }
    __ESBMC_assert(status != Status::success || NormObserver::count == 2U,
                   "E-PRED: successful Exp prediction completes both normalization stages");
    __ESBMC_assert(status == Status::success || preserved_vector(output.coefficients(), before.coefficients()),
                   "E-PRED: every failed Exp branch preserves the output");
#else
    (void)q;
    (void)sample;
    (void)dt;
    (void)output;
    __ESBMC_assert(false, "Runner error: general Exp requires scalar-boundary Exp profile");
#endif
}

// Noncommuting initial attitude and a nonzero body-X increment. Both the
// Taylor and regular Exp branches are selected by the runner, independently
// of the normalized-Euler compile-time mode.
void verify_attitude_exp()
{
#if !ESKF_QUAT_APPROX
    Scalar const dt = Scalar{0.5};
#if FORMAL_ESKF_PROOF_TAYLOR
    Scalar const rate_x = Scalar{0.0000000037252902984619140625}; // 2^-28; small for binary32 AND binary64.
#else
    Scalar const rate_x = Scalar{0.5};
#endif
    Vector rate;
    rate.set(0U, rate_x);
    Quaternion q;
    auto const construction =
        Quaternion::try_from_coefficients(Scalar{0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.125}, q);
    __ESBMC_assert(construction == Status::success, "E-PRED: noncommuting attitude fixture is valid");
    Quaternion increment;
    auto const exp_status = formal_eskf::so3::try_exp(rate * dt, Scalar{0.125}, increment);
    __ESBMC_assert(exp_status == Status::success, "E-PRED: profiled Exp increment succeeds");
    Scalar const c = increment.q0();
    Scalar const s = increment.q1();
    Quaternion expected;
    auto const expected_status = Quaternion::try_from_coefficients(
        Scalar{0.5} * c - Scalar{0.5} * s, Scalar{0.5} * s + Scalar{0.5} * c, Scalar{0.5} * c + Scalar{0.5} * s,
        -Scalar{0.5} * s + Scalar{0.5} * c, Scalar{0.125}, expected);
    Quaternion output;
    auto const status = formal_eskf::detail::try_predict_attitude(q, rate, dt, Scalar{0.125}, output);
    __ESBMC_assert(expected_status == Status::success && status == Status::success &&
                       preserved_vector(output.coefficients(), expected.coefficients()),
                   "E-PRED: Exp uses q_old times increment, not increment times q_old");
#else
    __ESBMC_assert(false, "Runner error: Exp harness must only run with ESKF_QUAT_APPROX=0");
#endif
}

// Actual IEEE sqrt (not the modular norm profile), with a mixed-axis Euler
// step and a non-identity unit quaternion. Exact dyadic numerators permit an
// independent closed-form normalization oracle for both scalar formats.
void verify_attitude_euler_execution()
{
#if ESKF_QUAT_APPROX
    Ahrs::nominal_state_type state;
    auto const construction = Quaternion::try_from_coefficients(Scalar{0.5}, Scalar{0.5}, Scalar{0.5}, Scalar{0.5},
                                                                Scalar{0.125}, state.q_nb);
    __ESBMC_assert(construction == Status::success, "E-PRED: Euler execution fixture is admissible");
    Imu imu;
    imu.angular_rate_b.set(0U, Scalar{0.5});
    imu.angular_rate_b.set(1U, Scalar{1});
    imu.angular_rate_b.set(2U, Scalar{-0.5});
    auto const parameters = valid_parameters<Ahrs::parameter_type>();
    auto const status = formal_eskf::try_predict_nominal(state, imu, Scalar{0.5}, parameters, state);
    Scalar const norm = std::sqrt(Scalar{1.09375});
    __ESBMC_assert(status == Status::success && state.q_nb.q0() == Scalar{0.375} / norm &&
                       state.q_nb.q1() == Scalar{0.375} / norm && state.q_nb.q2() == Scalar{0.75} / norm &&
                       state.q_nb.q3() == Scalar{0.5} / norm,
                   "E-PRED: mixed-axis Euler prediction matches the closed-form IEEE oracle in place");
#else
    __ESBMC_assert(false, "Runner error: Euler execution requires approximation and actual IEEE sqrt");
#endif
}

// A finite input can still overflow. Cover early bias subtraction, rotation
// increment, acceleration, position, and velocity, before/after attitude work.
void verify_prediction_overflow(bool alias, Ins::nominal_state_type output)
{
    constexpr unsigned scenario = FORMAL_ESKF_PROOF_SCENARIO;
    static_assert(scenario < 6U);
    Ins::nominal_state_type state;
    Imu imu;
    auto parameters = valid_parameters<Ins::parameter_type>();
    Scalar const maximum = std::numeric_limits<Scalar>::max();
    Scalar dt = Scalar{1};
    switch (scenario)
    {
    case 0U:
        imu.specific_force_b.set(0U, maximum);
        state.b_a.set(0U, -maximum);
        break;
    case 1U:
        imu.angular_rate_b.set(0U, maximum);
        state.b_g.set(0U, -maximum);
        break;
    case 2U:
        imu.angular_rate_b.set(0U, maximum);
        dt = Scalar{2};
        parameters.dt_max = dt;
        break;
    case 3U:
        imu.specific_force_b.set(0U, maximum);
        parameters.gravity_n.set(0U, maximum);
        break;
    case 4U:
        state.p_n.set(0U, maximum);
        state.v_n.set(0U, maximum);
        break;
    case 5U:
        state.v_n.set(0U, maximum);
        imu.specific_force_b.set(0U, maximum);
        break;
    }
    auto const before = alias ? state : output;
    Status const status = formal_eskf::try_predict_nominal(state, imu, dt, parameters, alias ? state : output);
    __ESBMC_assert(status == Status::non_finite_result && preserved_state(alias ? state : output, before),
                   "E-PRED: finite-input overflow is reported without publishing a partial state");
}

void verify_attitude_norm_failure(Quaternion q, bool overflow, Quaternion output)
{
    Scalar const value = overflow ? std::numeric_limits<Scalar>::max() : Scalar{0};
    __ESBMC_assume(q.q0() == value && q.q1() == Scalar{0} && q.q2() == Scalar{0} && q.q3() == Scalar{0});
    Vector rate;
    auto const before = output;
    auto const status = formal_eskf::detail::try_predict_attitude(q, rate, Scalar{0.5}, Scalar{0.125}, output);
    __ESBMC_assert(status == (overflow ? Status::non_finite_result : Status::invalid_quaternion_norm) &&
                       preserved_vector(output.coefficients(), before.coefficients()),
                   "E-PRED: zero/corrupt quaternion normalization failure is propagated atomically");
}
