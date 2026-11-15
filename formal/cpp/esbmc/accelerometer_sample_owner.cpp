/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// The actual owner body is checked against same-type integration, magnitude,
// interval-clear and vector-division contracts. Replies admit every failure and
// every IEEE quotient; no successful result or finite quotient is assumed.
#include "contracts.hpp"
#include "algorithm_support.hpp"
#include <formal_eskf/eskf/accelerometer.hpp>
#include <formal_eskf/eskf/types.hpp>
#include <formal_eskf/runtime/measurement_fusion.hpp>
#define private public
#define max formal_eskf_proof_max
#include <formal_eskf/runtime/accelerometer.hpp>
#undef max
#undef private
namespace owner_proof
{
using contract_proof::Linalg;
using contract_proof::Scalar;
using Bits = std::conditional_t<sizeof(Scalar) == 4, std::uint32_t, std::uint64_t>;
inline bool same(Scalar a, Scalar b)
{
    return __builtin_bit_cast(Bits, a) == __builtin_bit_cast(Bits, b) || (std::isnan(a) && std::isnan(b));
}
template <class V> bool same_elements(V const & a, V const & b)
{
    bool ok = true;
    for (std::size_t i = 0; i < V::row_count; ++i)
    {
        ok = same(a(i), b(i)) && ok;
    }
    return ok;
}
using contract_proof::finite_vector;
using formal_eskf::Status;
using Sampler = formal_eskf::runtime::AccelerometerSample<Linalg, 10000U>;
using Vector = Sampler::vector_type;
using Imu = formal_eskf::ImuSample<Linalg>;
using Magnitude = Sampler::magnitude_type;
inline bool same_double(double a, double b)
{
    return (__builtin_bit_cast(std::uint64_t, a) == __builtin_bit_cast(std::uint64_t, b)) ||
           (std::isnan(a) && std::isnan(b));
}
inline bool same_magnitude(Magnitude const & a, Magnitude const & b)
{
    return a.initialized() == b.initialized() &&
           same_elements(a.filtered_specific_force_b(), b.filtered_specific_force_b());
}
inline bool same_history(Sampler const & a, Sampler const & b)
{
    return same_magnitude(a.m_magnitude, b.m_magnitude) && same_elements(a.m_delta_velocity, b.m_delta_velocity) &&
           a.m_interval_us == b.m_interval_us && a.m_samples == b.m_samples && a.m_clipping_mask == b.m_clipping_mask &&
           same_double(a.m_average_period_us, b.m_average_period_us) && a.m_required_samples == b.m_required_samples &&
           same_double(a.m_target_period_us, b.m_target_period_us) &&
           same_double(a.m_minimum_period_us, b.m_minimum_period_us);
}
inline bool thresholds(std::uint64_t count, double target, double minimum)
{
    return count >= 1U && count <= 65536U && std::isfinite(target) && target >= 1.0 && target <= 4294967296.0 &&
           std::isfinite(minimum) && minimum >= 0.0 && minimum <= 4294967296.0;
}
// This intentionally broad reachable-state superset makes integer additions
// safe without assuming exact relationships among the adaptive thresholds.
// The actual initialization establishes it; clear_interval and update preserve it.
inline bool invariant(Sampler const & h)
{
    return std::isfinite(h.m_average_period_us) && h.m_average_period_us >= 1.0 && h.m_average_period_us <= 50000.0 &&
           thresholds(h.m_required_samples, h.m_target_period_us, h.m_minimum_period_us) &&
           h.m_samples <= h.m_interval_us && h.m_interval_us <= 4294967296ULL && h.m_clipping_mask >= -1 &&
           h.m_clipping_mask <= 7 && finite_vector(h.m_delta_velocity) &&
           finite_vector(h.m_magnitude.filtered_specific_force_b());
}
struct Replies
{
    Status integration;
    Vector accumulated, filtered, quotient;
    Magnitude::Result magnitude;
    std::uint64_t count;
    double target, minimum;
};
struct Calls
{
    inline static Sampler const * before{};
    inline static Sampler * owner{};
    inline static Imu const * imu{};
    inline static Vector force;
    inline static Replies replies;
    inline static Scalar dt{}, minimum{};
    inline static unsigned integrations{}, magnitudes{}, clears{}, quotients{};
    inline static Scalar denominator{};
    inline static double average{};
};
} // namespace owner_proof
namespace formal_eskf::verification
{
template <>
template <>
inline void owner_proof::Linalg::divide<3U, 1U>(storage_type<3U, 1U> const & value, owner_proof::Scalar divisor,
                                                storage_type<3U, 1U> & out)
{
    using namespace owner_proof;
    __ESBMC_assert(Calls::quotients++ == 0U && Calls::integrations == 1U && Calls::magnitudes == 0U &&
                       same(divisor, Calls::denominator),
                   "sample owner: quotient scalar and order");
    for (unsigned i = 0; i < 3; ++i)
    {
        __ESBMC_assert(same(value.values[i], Calls::replies.accumulated(i)), "sample owner: quotient input");
        out.values[i] = Calls::replies.quotient(i);
    }
}
} // namespace formal_eskf::verification
namespace formal_eskf::runtime
{
template <>
inline Status AccelerometerSample<owner_proof::Linalg, 10000U>::integrate(ImuSample<owner_proof::Linalg> const & imu,
                                                                          owner_proof::Scalar dt,
                                                                          owner_proof::Scalar minimum,
                                                                          owner_proof::Vector & out) const
{
    using namespace owner_proof;
    __ESBMC_assert(
        this == Calls::owner && Calls::integrations++ == 0 && same_history(*this, *Calls::before) &&
            same_elements(imu.specific_force_b, Calls::imu->specific_force_b) &&
            same_elements(imu.angular_rate_b, Calls::imu->angular_rate_b) && same(dt, Calls::dt) &&
            same(minimum, Calls::minimum),
        "sample owner: integration receives original history, both IMU vectors, dt and normalization threshold once");
    if (Calls::replies.integration == Status::success)
    {
        out = Calls::replies.accumulated;
    }
    return Calls::replies.integration;
}
template <>
inline owner_proof::Magnitude::Result
AccelerometerMagnitude<owner_proof::Linalg>::update(owner_proof::Vector const & force)
{
    using namespace owner_proof;
    __ESBMC_assert(
        Calls::integrations == 1 && Calls::magnitudes++ == 0 && Calls::clears == 0 &&
            this == &Calls::owner->m_magnitude && same_elements(force, Calls::force) && finite_vector(force) &&
            same_history(*Calls::owner, *Calls::before),
        "sample owner: one magnitude update uses finite interval force before any interval/history publication");
    if (Calls::replies.magnitude.status == Status::success)
    {
        m_filtered_specific_force_b = Calls::replies.filtered;
        m_initialized = true;
    }
    return Calls::replies.magnitude;
}
template <> inline void AccelerometerSample<owner_proof::Linalg, 10000U>::clear_interval()
{
    using namespace owner_proof;
    __ESBMC_assert(this == Calls::owner && Calls::clears++ == 0 && Calls::magnitudes == 1 &&
                       Calls::replies.magnitude.status == Status::success &&
                       same_double(m_average_period_us, Calls::average),
                   "sample owner: successful ready observation updates average before clearing interval");
    m_delta_velocity = {};
    m_interval_us = 0;
    m_samples = 0;
    m_clipping_mask = 0;
    m_required_samples = Calls::replies.count;
    m_target_period_us = Calls::replies.target;
    m_minimum_period_us = Calls::replies.minimum;
}
} // namespace formal_eskf::runtime
void verify_sample_transition(owner_proof::Sampler history, owner_proof::Imu imu, std::uint64_t period, int mask,
                              owner_proof::Scalar minimum, owner_proof::Replies replies)
{
    using namespace owner_proof;
    __ESBMC_assume(invariant(history));
    // Independently established same-type integration, magnitude and interval-clear postconditions.
    __ESBMC_assume(replies.integration != Status::success || finite_vector(replies.accumulated));
    __ESBMC_assume(replies.magnitude.status != Status::success ||
                   (finite_vector(replies.filtered) && std::isfinite(replies.magnitude.squared_norm) &&
                    replies.magnitude.squared_norm >= Scalar{0}));
    __ESBMC_assume(replies.magnitude.status == Status::success ||
                   (!replies.magnitude.within_gravity_band && same(replies.magnitude.squared_norm, Scalar{0})));
    __ESBMC_assume(thresholds(replies.count, replies.target, replies.minimum));
    auto const before = history;
    auto const old_imu = imu;
    Calls::before = &before;
    Calls::owner = &history;
    Calls::imu = &old_imu;
    Calls::replies = replies;
    Calls::dt = static_cast<Scalar>(period) * static_cast<Scalar>(1e-6);
    Calls::minimum = minimum;
    Calls::integrations = Calls::magnitudes = Calls::clears = Calls::quotients = 0;
    bool const input = finite_vector(imu.specific_force_b) && finite_vector(imu.angular_rate_b);
    bool const domain = period >= 1U && period <= 50000U && mask >= -1 && mask <= 7;
    Status wanted = !input ? Status::non_finite_input : !domain ? Status::domain_error : replies.integration;
    Sampler expected = before;
    Sampler::Result result;
    bool ready = false, measured = false, committed = false;
    if (wanted == Status::success)
    {
        std::uint64_t const interval = before.m_interval_us + period, samples = before.m_samples + 1U;
        int const combined = before.m_clipping_mask < 0 || mask < 0 ? -1 : before.m_clipping_mask | mask;
        ready = (samples >= before.m_required_samples && static_cast<double>(interval) > before.m_minimum_period_us) ||
                static_cast<double>(interval) > before.m_target_period_us;
        if (ready)
        {
            Calls::denominator = static_cast<Scalar>(interval) * static_cast<Scalar>(1e-6);
            Calls::force = replies.quotient;
            if (!finite_vector(Calls::force))
            {
                wanted = Status::non_finite_result;
            }
            else
            {
                measured = true;
                result.magnitude = replies.magnitude;
                wanted = replies.magnitude.status;
            }
        }
        if (wanted == Status::success)
        {
            committed = true;
            Calls::average =
                before.m_average_period_us + 0.1 * (static_cast<double>(period) - before.m_average_period_us);
            expected.m_average_period_us = Calls::average;
            if (ready)
            {
                result.ready = true;
                result.specific_force_b = Calls::force;
                result.interval_us = interval;
                result.clipping_mask = combined;
                expected.m_magnitude.m_initialized = true;
                expected.m_magnitude.m_filtered_specific_force_b = replies.filtered;
                expected.m_delta_velocity = {};
                expected.m_interval_us = 0;
                expected.m_samples = 0;
                expected.m_clipping_mask = 0;
                expected.m_required_samples = replies.count;
                expected.m_target_period_us = replies.target;
                expected.m_minimum_period_us = replies.minimum;
            }
            else
            {
                expected.m_delta_velocity = replies.accumulated;
                expected.m_interval_us = interval;
                expected.m_samples = samples;
                expected.m_clipping_mask = combined;
            }
        }
    }
    result.status = wanted;
    auto const actual = history.update(imu, period, mask, minimum);
    __ESBMC_assert(actual.status == result.status && actual.ready == result.ready &&
                       same_elements(actual.specific_force_b, result.specific_force_b) &&
                       actual.interval_us == result.interval_us && actual.clipping_mask == result.clipping_mask &&
                       actual.magnitude.status == result.magnitude.status &&
                       actual.magnitude.within_gravity_band == result.magnitude.within_gravity_band &&
                       same(actual.magnitude.squared_norm, result.magnitude.squared_norm),
                   "sample owner: exact complete result including empty failure/not-ready diagnostics");
    __ESBMC_assert(Calls::integrations == unsigned(input && domain) && Calls::magnitudes == unsigned(measured) &&
                       Calls::clears == unsigned(ready && committed) && Calls::quotients == unsigned(ready),
                   "sample owner: validation, strict readiness, magnitude and clear order");
    __ESBMC_assert(history.m_interval_us == expected.m_interval_us, "sample owner: exact m_interval_us");
    __ESBMC_assert(history.m_samples == expected.m_samples, "sample owner: exact m_samples");
    __ESBMC_assert(history.m_clipping_mask == expected.m_clipping_mask, "sample owner: exact m_clipping_mask");
    __ESBMC_assert(history.m_required_samples == expected.m_required_samples, "sample owner: exact m_required_samples");
    __ESBMC_assert(same_double(history.m_average_period_us, expected.m_average_period_us),
                   "sample owner: exact m_average_period_us");
    __ESBMC_assert(same_double(history.m_target_period_us, expected.m_target_period_us),
                   "sample owner: exact m_target_period_us");
    __ESBMC_assert(same_double(history.m_minimum_period_us, expected.m_minimum_period_us),
                   "sample owner: exact m_minimum_period_us");
    __ESBMC_assert(history.m_magnitude.m_initialized == expected.m_magnitude.m_initialized,
                   "sample owner: exact LPF initialization");
    __ESBMC_assert(same(history.m_delta_velocity(0), expected.m_delta_velocity(0)), "sample owner: exact increment 0");
    __ESBMC_assert(
        same(history.m_magnitude.m_filtered_specific_force_b(0), expected.m_magnitude.m_filtered_specific_force_b(0)),
        "sample owner: exact filter 0");
    __ESBMC_assert(same(history.m_delta_velocity(1), expected.m_delta_velocity(1)), "sample owner: exact increment 1");
    __ESBMC_assert(
        same(history.m_magnitude.m_filtered_specific_force_b(1), expected.m_magnitude.m_filtered_specific_force_b(1)),
        "sample owner: exact filter 1");
    __ESBMC_assert(same(history.m_delta_velocity(2), expected.m_delta_velocity(2)), "sample owner: exact increment 2");
    __ESBMC_assert(
        same(history.m_magnitude.m_filtered_specific_force_b(2), expected.m_magnitude.m_filtered_specific_force_b(2)),
        "sample owner: exact filter 2");

    __ESBMC_assert(std::isfinite(history.m_average_period_us) && history.m_average_period_us >= 1.0 &&
                       history.m_average_period_us <= 50000.0,
                   "sample owner: inductive average");
    __ESBMC_assert(thresholds(history.m_required_samples, history.m_target_period_us, history.m_minimum_period_us),
                   "sample owner: inductive timing thresholds");
    __ESBMC_assert(history.m_samples <= history.m_interval_us && history.m_interval_us <= 4294967296ULL &&
                       history.m_clipping_mask >= -1 && history.m_clipping_mask <= 7,
                   "sample owner: inductive counters and mask");
    __ESBMC_assert(finite_vector(history.m_delta_velocity) &&
                       finite_vector(history.m_magnitude.filtered_specific_force_b()),
                   "sample owner: inductive vector history");
    __ESBMC_assert(same_elements(imu.specific_force_b, old_imu.specific_force_b) &&
                       same_elements(imu.angular_rate_b, old_imu.angular_rate_b),
                   "sample owner: input frame");
}
int main() { return 0; }
