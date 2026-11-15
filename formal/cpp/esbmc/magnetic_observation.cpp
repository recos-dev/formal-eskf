/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "contracts.hpp"
#ifndef FORMAL_ESKF_PROOF_MAG_SOLVE_BACKEND
#define FORMAL_ESKF_PROOF_MAG_SOLVE_BACKEND 0
#endif
#if FORMAL_ESKF_PROOF_MAG_SOLVE_BACKEND
#include "step_correction_support.hpp"
#endif
#include <formal_eskf/runtime/magnetometer.hpp>
#ifndef FORMAL_ESKF_PROOF_MAG_OBSERVATION_CONTRACT
#define FORMAL_ESKF_PROOF_MAG_OBSERVATION_CONTRACT 0
#endif
namespace magnetic_observation_proof
{
using contract_proof::finite_vector;
#if FORMAL_ESKF_PROOF_MAG_SOLVE_BACKEND
using Linalg = solve_proof::Backend;
using Quaternion = formal_eskf::so3::UnitQuaternion<Linalg>;
using Vector3 = formal_eskf::linalg::Vector<Linalg, 3U>;
#else
using contract_proof::Linalg;
using contract_proof::Quaternion;
using contract_proof::Vector3;
#endif
using contract_proof::same;
using contract_proof::same_vector;
using contract_proof::Scalar;
using formal_eskf::Status;
using Parameters = formal_eskf::runtime::MagneticFieldParameters<Scalar>;
using Checks = formal_eskf::runtime::MagneticFieldChecks;
using Field = formal_eskf::runtime::MagneticFieldCheck<Scalar>;
inline bool same_parameters(Parameters const & a, Parameters const & b)
{
    return same(a.strength_tolerance, b.strength_tolerance) && same(a.inclination_tolerance, b.inclination_tolerance) &&
           same(a.heading_tolerance, b.heading_tolerance) &&
           same(a.minimum_horizontal_field, b.minimum_horizontal_field);
}
inline bool same_checks(Checks const & a, Checks const & b)
{
    return a.strength == b.strength && a.inclination == b.inclination && a.require_reference == b.require_reference;
}
inline bool same_field(Field const & a, Field const & b)
{
    return a.status == b.status && a.accepted == b.accepted && a.diagnostics_valid == b.diagnostics_valid &&
           a.strength_checked == b.strength_checked && a.inclination_checked == b.inclination_checked &&
           a.heading_checked == b.heading_checked && a.failures.reference == b.failures.reference &&
           a.failures.strength == b.failures.strength && a.failures.inclination == b.failures.inclination &&
           a.failures.horizontal_field == b.failures.horizontal_field && a.failures.heading == b.failures.heading &&
           same(a.strength, b.strength) && same(a.reference_strength, b.reference_strength) &&
           same(a.inclination_error, b.inclination_error) && same(a.heading_error, b.heading_error);
}
struct Calls
{
    inline static Quaternion attitude;
    inline static Vector3 measured, reference, predicted;
    inline static Vector3 const * health;
    inline static Parameters parameters;
    inline static Checks checks;
    inline static Field field;
    inline static bool observable;
    inline static unsigned fields, rotations;
};
} // namespace magnetic_observation_proof
#if FORMAL_ESKF_PROOF_MAG_OBSERVATION_CONTRACT
namespace formal_eskf::runtime
{
template <>
inline magnetic_observation_proof::Field check_magnetic_field<magnetic_observation_proof::Linalg>(
    magnetic_observation_proof::Quaternion const & q, magnetic_observation_proof::Vector3 const & measured,
    magnetic_observation_proof::Vector3 const & reference, magnetic_observation_proof::Vector3 const * health,
    magnetic_observation_proof::Parameters const & parameters, bool observable,
    magnetic_observation_proof::Checks const & checks)
{
    using namespace magnetic_observation_proof;
    __ESBMC_assert(
        Calls::fields++ == 0U && Calls::rotations == 0U &&
            same_vector(q.coefficients(), Calls::attitude.coefficients()) && same_vector(measured, Calls::measured) &&
            same_vector(reference, Calls::reference) && health == Calls::health &&
            same_parameters(parameters, Calls::parameters) && same_checks(checks, Calls::checks) &&
            observable == Calls::observable,
        "R-MAGNETOMETER-FUSION: field qualification receives original attitude, distinct references and policy first");
    // Arbitrary return overapproximates every result of magnetic_check.cpp;
    // no accepted/success/finite diagnostic premise is assumed here.
    return Calls::field;
}
} // namespace formal_eskf::runtime
namespace formal_eskf::so3
{
template <>
inline magnetic_observation_proof::Vector3
inverse_rotate<magnetic_observation_proof::Linalg>(magnetic_observation_proof::Quaternion const & q,
                                                   magnetic_observation_proof::Vector3 const & v)
{
    using namespace magnetic_observation_proof;
    __ESBMC_assert(
        Calls::fields == 1U && Calls::field.status == Status::success && Calls::field.accepted &&
            Calls::rotations++ == 0U && same_vector(q.coefficients(), Calls::attitude.coefficients()) &&
            same_vector(v, Calls::reference),
        "R-MAGNETOMETER-FUSION: accepted field predicts body observation from original observation reference");
    return Calls::predicted;
}
} // namespace formal_eskf::so3
#endif
void verify_magnetic_inverse_frame(magnetic_observation_proof::Quaternion q, magnetic_observation_proof::Vector3 v)
{
    using namespace magnetic_observation_proof;
#if FORMAL_ESKF_PROOF_MAG_SOLVE_BACKEND
    solve_proof::Math::observe_pivots = false;
#endif
    __ESBMC_assert(!FORMAL_ESKF_PROOF_MAG_OBSERVATION_CONTRACT, "Runner error: inverse action must be actual");
    auto const old_q = q;
    auto const old_v = v;
    auto const result = formal_eskf::so3::inverse_rotate(q, v);
    (void)result;
    __ESBMC_assert(
        same_vector(q.coefficients(), old_q.coefficients()) && same_vector(v, old_v),
        "R-MAGNETOMETER-FUSION: actual same-backend inverse action preserves both inputs for every IEEE return");
}
void verify_magnetic_observation(magnetic_observation_proof::Quaternion q, magnetic_observation_proof::Vector3 measured,
                                 magnetic_observation_proof::Vector3 reference,
                                 magnetic_observation_proof::Vector3 health, unsigned health_source,
                                 magnetic_observation_proof::Parameters parameters,
                                 magnetic_observation_proof::Checks checks, bool observable,
                                 magnetic_observation_proof::Field field, magnetic_observation_proof::Vector3 predicted,
                                 magnetic_observation_proof::Vector3 prediction,
                                 magnetic_observation_proof::Vector3 residual)
{
    using namespace magnetic_observation_proof;
#if FORMAL_ESKF_PROOF_MAG_SOLVE_BACKEND
    solve_proof::Math::observe_pivots = false;
#endif
    __ESBMC_assert(FORMAL_ESKF_PROOF_MAG_OBSERVATION_CONTRACT,
                   "Runner error: observation needs paired producer summaries");
    __ESBMC_assume(health_source < 4U);
    Vector3 const * selected = health_source == 0U   ? nullptr
                               : health_source == 1U ? &health
                               : health_source == 2U ? &measured
                                                     : &reference;
    auto const old_q = q;
    auto const old_measured = measured, old_reference = reference, old_health = health;
    auto const old_prediction = prediction, old_residual = residual;
    auto const old_parameters = parameters;
    auto const old_checks = checks;
    Calls::attitude = q;
    Calls::measured = measured;
    Calls::reference = reference;
    Calls::health = selected;
    Calls::parameters = parameters;
    Calls::checks = checks;
    Calls::observable = observable;
    Calls::field = field;
    Calls::predicted = predicted;
    Calls::fields = Calls::rotations = 0U;
    // These are disjoint local scratch outputs, matching both public callers.
    auto const result = formal_eskf::runtime::detail::check_magnetic_observation(
        q, measured, reference, selected, parameters, observable, checks, prediction, residual);
    bool const qualified = field.status == Status::success && field.accepted;
    Vector3 wanted_residual;
    for (std::size_t i = 0U; i < 3U; ++i)
        wanted_residual(i) = old_measured(i) - predicted(i);
    bool const finite = finite_vector(predicted) && finite_vector(wanted_residual);
    Status const wanted = qualified && !finite ? Status::non_finite_result : field.status;
    auto const decision = field.status == Status::success && !field.accepted
                              ? formal_eskf::runtime::FusionDecision::rejected
                              : formal_eskf::runtime::FusionDecision::failed;
    __ESBMC_assert(
        same_field(result.field, field) && result.fusion.status == wanted && result.fusion.decision == decision &&
            !result.fusion.diagnostics_valid && Calls::fields == 1U && Calls::rotations == (qualified ? 1U : 0U),
        "R-MAGNETOMETER-FUSION: field diagnostics, stop priority and invalid innovation diagnostics are preserved");
    if (qualified && finite)
    {
        __ESBMC_assert(same_vector(prediction, predicted) && same_vector(residual, wanted_residual),
                       "R-MAGNETOMETER-FUSION: successful prediction and exact measured-minus-predicted residual "
                       "publish together");
    }
    else
    {
        __ESBMC_assert(same_vector(prediction, old_prediction) && same_vector(residual, old_residual),
                       "R-MAGNETOMETER-FUSION: failed or rejected observation preserves both scratch outputs");
    }
    __ESBMC_assert(same_vector(q.coefficients(), old_q.coefficients()) && same_vector(measured, old_measured) &&
                       same_vector(reference, old_reference) && same_vector(health, old_health) &&
                       same_parameters(parameters, old_parameters) && same_checks(checks, old_checks),
                   "R-MAGNETOMETER-FUSION: observation preserves all borrowed inputs and policies");
}
int main() { return 0; }
