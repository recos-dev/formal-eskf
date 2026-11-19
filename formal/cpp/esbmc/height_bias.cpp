/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "contracts.hpp"
#include "height_support.hpp"
#include <formal_eskf/runtime/gnss_lifecycle.hpp>
#include <formal_eskf/runtime/navigation.hpp>
#include <formal_eskf/runtime/vertical_acceleration_health.hpp>
#include "../../../integrations/px4/module/DeploymentProfile.hpp"
// Scoped library models and member-name workaround; source bodies are unchanged.
#define max formal_eskf_proof_max
#define min formal_eskf_proof_min
#define any_of formal_eskf_proof_any_of
#define finite formal_eskf_proof_finite
#include "../../../integrations/px4/module/HeightAiding.hpp"
#undef finite
#undef any_of
#undef min
#undef max
using contract_proof::same;
using contract_proof::Scalar;
using Bias = formal_eskf::px4::HeightBias<Scalar>;
namespace settings = formal_eskf::px4::profile::height_bias;
static_assert(settings::initial_maximum_variance == 2.0 && settings::initial_interval_s == 0.01 &&
                  settings::process_variance_density == 0.13 * 0.13 && settings::minimum_variance == 1e-8 &&
                  settings::measurement_variance_floor == 1e-4 && settings::gate_squared == 9.0 &&
                  settings::offset_ratio == 0.2 && settings::offset_duration_s == 10.0 &&
                  settings::offset_variance_multiplier == 1000.0 && settings::ratio_filter_time_s == 10.0,
              "height bias deployment defaults match the specified constants");
bool same_bias(Bias const & a, Bias const & b)
{
    return same(a.value, b.value) && same(a.variance, b.variance) && same(a.maximum_variance, b.maximum_variance) &&
           same(a.signed_ratio, b.signed_ratio) && same(a.positive_time, b.positive_time) &&
           same(a.negative_time, b.negative_time) && same(a.dt, b.dt);
}
Scalar clipped(Scalar v, Scalar lo, Scalar hi) { return v < lo ? lo : (hi < v ? hi : v); }
void verify_height_bias_initialization(Bias arbitrary)
{
    Bias bias;
    __ESBMC_assert(bias.formal_eskf_proof_finite() && bias.value == Scalar{0} && bias.variance == Scalar{0} &&
                       bias.maximum_variance == static_cast<Scalar>(settings::initial_maximum_variance) &&
                       bias.signed_ratio == Scalar{0} && bias.positive_time == Scalar{0} &&
                       bias.negative_time == Scalar{0} && bias.dt == static_cast<Scalar>(settings::initial_interval_s),
                   "R-HEIGHT-FUSION: actual bias initialization and exact scalar-format defaults");
    auto const before = arbitrary;
    bool const expected = std::isfinite(arbitrary.value) && std::isfinite(arbitrary.variance) &&
                          std::isfinite(arbitrary.signed_ratio) && std::isfinite(arbitrary.positive_time) &&
                          std::isfinite(arbitrary.negative_time);
    __ESBMC_assert(arbitrary.formal_eskf_proof_finite() == expected && same_bias(arbitrary, before),
                   "R-HEIGHT-FUSION: finite predicate covers precisely its five fields and is pure");
}
void verify_height_bias_prediction(Bias bias, Scalar dt)
{
    // Internal producer domain: committed variance/ceiling and tick interval.
    // HeightAiding's initialization/transition must establish these separately.
    bool const finite_prior = std::isfinite(bias.variance);
    __ESBMC_assume((!finite_prior || bias.variance >= Scalar{0}) && std::isfinite(bias.maximum_variance) &&
                   bias.maximum_variance >= static_cast<Scalar>(settings::minimum_variance) && std::isfinite(dt) &&
                   dt >= Scalar{0});
    auto expected = bias;
    bool const offset = std::fabs(bias.signed_ratio) > static_cast<Scalar>(settings::offset_ratio) ||
                        bias.positive_time > static_cast<Scalar>(settings::offset_duration_s) ||
                        bias.negative_time > static_cast<Scalar>(settings::offset_duration_s);
    expected.dt = dt;
    expected.variance =
        clipped(bias.variance + static_cast<Scalar>(settings::process_variance_density) * dt *
                                    (offset ? static_cast<Scalar>(settings::offset_variance_multiplier) : Scalar{1}),
                static_cast<Scalar>(settings::minimum_variance), bias.maximum_variance);
    bias.predict(dt);
    __ESBMC_assert(same_bias(bias, expected),
                   "R-HEIGHT-FUSION: exact offset-triggered variance prediction and untouched bias fields");
    __ESBMC_assert(!finite_prior || (std::isfinite(bias.variance) &&
                                     bias.variance >= static_cast<Scalar>(settings::minimum_variance) &&
                                     bias.variance <= bias.maximum_variance),
                   "R-HEIGHT-FUSION: prediction contains finite variance within its configured interval");
    __ESBMC_assert(!std::isfinite(bias.variance) || (bias.variance >= static_cast<Scalar>(settings::minimum_variance) &&
                                                     bias.variance <= bias.maximum_variance),
                   "R-HEIGHT-FUSION: arbitrary nonfinite scratch preserves the finite variance domain");
}
void verify_height_bias_update(Bias bias, Scalar measurement, Scalar measurement_variance, Scalar sensor_variance)
{
    __ESBMC_assume(std::isfinite(sensor_variance) && sensor_variance > Scalar{0});
    bool const nonnegative_prior = !std::isfinite(bias.variance) || bias.variance >= Scalar{0};
    auto expected = bias;
    Scalar const floor = static_cast<Scalar>(settings::measurement_variance_floor);
    expected.maximum_variance = sensor_variance < floor ? floor : sensor_variance;
    Scalar const residual = measurement - bias.value;
    Scalar const S = bias.variance + (measurement_variance < floor ? floor : measurement_variance);
    Scalar const ratio = residual * residual / (static_cast<Scalar>(settings::gate_squared) * S);
    if (ratio < Scalar{1})
    {
        Scalar const K = bias.variance / S;
        expected.value = bias.value + K * residual;
        expected.variance = clipped((Scalar{1} - K) * bias.variance, static_cast<Scalar>(settings::minimum_variance),
                                    expected.maximum_variance);
    }
    Scalar const sign = residual > Scalar{0} ? Scalar{1} : (residual < Scalar{0} ? Scalar{-1} : Scalar{0});
    expected.signed_ratio = bias.signed_ratio + bias.dt /
                                                    (bias.dt + static_cast<Scalar>(settings::ratio_filter_time_s)) *
                                                    (clipped(sign * ratio, Scalar{-1}, Scalar{1}) - bias.signed_ratio);
    expected.positive_time = residual > Scalar{0} ? Scalar{0} : bias.positive_time + bias.dt;
    expected.negative_time = residual > Scalar{0} ? bias.negative_time + bias.dt : Scalar{0};
    bias.fuse(measurement, measurement_variance, sensor_variance);
    __ESBMC_assert(same_bias(bias, expected),
                   "R-HEIGHT-FUSION: exact strict gate, posterior, signed-ratio filter and one-sided timers");
    __ESBMC_assert(std::isfinite(bias.maximum_variance) && bias.maximum_variance >= floor,
                   "R-HEIGHT-FUSION: validated sensor variance establishes the next finite variance ceiling");
    __ESBMC_assert(!nonnegative_prior || !std::isfinite(bias.variance) || bias.variance >= Scalar{0},
                   "R-HEIGHT-FUSION: finite posterior variance preserves the nonnegative prior invariant");
    __ESBMC_assert(!(ratio < Scalar{1}) || !std::isfinite(bias.variance) ||
                       (bias.variance >= static_cast<Scalar>(settings::minimum_variance) &&
                        bias.variance <= bias.maximum_variance),
                   "R-HEIGHT-FUSION: finite accepted posterior obeys the variance floor and sensor ceiling");
    // The owner rejects nonfinite scratch. This producer has no failure-return
    // contract and intentionally admits NaN/overflow in all remaining arguments.
}
int main() { return 0; }
