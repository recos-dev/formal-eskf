/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "contracts.hpp"
#include "algorithm_support.hpp"
#include <formal_eskf/eskf/accelerometer.hpp>
#include <formal_eskf/eskf/types.hpp>
#include <formal_eskf/runtime/measurement_fusion.hpp>
// Only std::round is summarized. The producer below checks the actual library
// model's nearest-integer rule; the clear body also proves its input domain.
namespace timing_proof
{
inline double rounded{}, argument{};
inline unsigned rounds{};
} // namespace timing_proof
namespace std
{
inline double formal_eskf_proof_round(double value)
{
    timing_proof::argument = value;
    ++timing_proof::rounds;
    return timing_proof::rounded;
}
} // namespace std
#define private public
#define max formal_eskf_proof_max
#define round formal_eskf_proof_round
#include <formal_eskf/runtime/accelerometer.hpp>
#undef round
#undef max
#undef private
using Sampler = formal_eskf::runtime::AccelerometerSample<contract_proof::Linalg, 10000U>;
void verify_sample_clear(Sampler history, double rounded)
{
    using namespace timing_proof;
    __ESBMC_assume(std::isfinite(history.m_average_period_us) && history.m_average_period_us >= 1.0 &&
                   history.m_average_period_us <= 50000.0);
    __ESBMC_assume(std::isfinite(rounded) && rounded >= 0.0 && rounded <= 50000.0);
    auto const before = history;
    timing_proof::rounded = rounded;
    rounds = 0;
    history.clear_interval();
    std::uint64_t const raw = static_cast<std::uint64_t>(rounded);
    std::uint64_t const count = 1U < raw ? raw : 1U;
    double const target = static_cast<double>(count) * before.m_average_period_us;
    double const previous = static_cast<double>(count - 1U) * before.m_average_period_us;
    double const half = before.m_average_period_us * 0.5;
    __ESBMC_assert(history.m_required_samples == count &&
                       __builtin_bit_cast(std::uint64_t, history.m_target_period_us) ==
                           __builtin_bit_cast(std::uint64_t, target) &&
                       __builtin_bit_cast(std::uint64_t, history.m_minimum_period_us) ==
                           __builtin_bit_cast(std::uint64_t, (previous < half ? half : previous)),
                   "clear: exact round/cast/count and target/minimum equations");
    __ESBMC_assert(rounds == 1 && argument == 10000.0 / before.m_average_period_us && std::isfinite(argument) &&
                       argument >= 0.02 && argument <= 50000.0,
                   "clear: actual ratio establishes round contract domain");
    __ESBMC_assert(history.m_required_samples >= 1U && history.m_required_samples <= 65536U, "clear: bounded count");
    __ESBMC_assert(std::isfinite(history.m_target_period_us) && history.m_target_period_us >= 1.0 &&
                       history.m_target_period_us <= 4294967296.0,
                   "clear: bounded target");
    __ESBMC_assert(std::isfinite(history.m_minimum_period_us) && history.m_minimum_period_us >= 0.0 &&
                       history.m_minimum_period_us <= 4294967296.0,
                   "clear: bounded minimum");
    __ESBMC_assert(history.m_interval_us == 0 && history.m_samples == 0 && history.m_clipping_mask == 0 &&
                       contract_proof::same_vector(history.m_delta_velocity, Sampler::vector_type{}),
                   "clear: empty accumulation");
    __ESBMC_assert(history.m_average_period_us == before.m_average_period_us &&
                       history.m_magnitude.initialized() == before.m_magnitude.initialized() &&
                       contract_proof::same_vector(history.m_magnitude.filtered_specific_force_b(),
                                                   before.m_magnitude.filtered_specific_force_b()),
                   "clear: preserves average and magnitude history");
}
void verify_sample_round(double ratio)
{
    __ESBMC_assume(std::isfinite(ratio) && ratio >= 0.02 && ratio <= 50000.0);
    double const rounded = std::round(ratio);
    auto const floor = static_cast<std::uint64_t>(ratio);
    auto const nearest = ratio - static_cast<double>(floor) < 0.5 ? floor : floor + 1U;
    __ESBMC_assert(rounded == static_cast<double>(nearest),
                   "sample: nearest integer with positive ties away from zero");
    __ESBMC_assert(std::isfinite(rounded) && rounded >= 0.0 && rounded <= 50000.0,
                   "sample: round is finite and in cast range");
    __ESBMC_assert(rounded - ratio <= 0.5 && ratio - rounded <= 0.5, "sample: round differs by at most half a sample");
}
int main() { return 0; }
