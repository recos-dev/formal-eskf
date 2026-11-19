/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "height_state_support.hpp"

// Admission bounds the measurement. This producer lemma gives a finite LPF
// invariant, including binary rounding; the owner must establish its premise.
void verify_height_filter_range(height_proof::Scalar prior, height_proof::Scalar sample)
{
    using namespace height_proof;
    constexpr Scalar bound{131072};
    __ESBMC_assume(std::isfinite(prior) && prior >= -bound && prior <= bound && altitude_valid(sample));
    Scalar const next = prior + static_cast<Scalar>(setting::barometer_lpf_gain) * (sample - prior);
    __ESBMC_assert(std::isfinite(next) && next >= -bound && next <= bound,
                   "R-HEIGHT-FUSION: admitted LPF preserves a finite bounded altitude under IEEE rounding");
}

void verify_height_filter(height_proof::Manager manager, height_proof::Sample sample, height_proof::Scalar altitude,
                          height_proof::Result result)
{
    using namespace height_proof;
    __ESBMC_assume(manager.m_barometer_samples <= setting::barometer_warmup_samples + 1U);
    auto const epoch = manager.m_tick > manager.m_started ? manager.m_tick : manager.m_started;
    bool const domain = history_domain(manager, epoch);
    auto const old_sample = sample;
    auto expected = manager;
    auto expected_result = result;
    bool const first = manager.m_barometer_samples == 0U;
    Scalar const filtered = first ? sample.altitude
                                  : manager.m_barometer_lpf + static_cast<Scalar>(setting::barometer_lpf_gain) *
                                                                  (sample.altitude - manager.m_barometer_lpf);
    expected.m_barometer_lpf = filtered;
    unsigned const count = manager.m_barometer_samples + 1U;
    expected.m_barometer_samples = count > 21U ? 21U : count;
    bool ready = false;
    if (count <= 20U)
    {
        expected.m_sessions[0U].bias.value = filtered - altitude;
    }
    else if (manager.m_sessions[0U].fault)
    {
        ++expected_result.rejected;
    }
    else
    {
        ready = true;
    }
    bool const actual = manager.prepare_barometer(sample, altitude, result);
    __ESBMC_assert(!domain || history_domain(manager, epoch),
                   "R-HEIGHT-FUSION: warmup/filter preserves the complete owner induction domain");
    __ESBMC_assert(actual == ready && same_manager(manager, expected) && same_result(result, expected_result),
                   "R-HEIGHT-FUSION: actual LPF, twenty-sample warmup, saturation, fault rejection and full frame");
    __ESBMC_assert(manager.m_barometer_samples >= 1U && manager.m_barometer_samples <= 21U,
                   "R-HEIGHT-FUSION: the bounded warmup counter cannot wrap or grow with flight duration");
    __ESBMC_assert(same_sample(sample, old_sample), "R-HEIGHT-FUSION: filter preserves the borrowed sample");
}
int main() { return 0; }
