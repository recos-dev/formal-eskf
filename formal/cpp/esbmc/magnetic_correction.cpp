/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "step_correction_support.hpp"
#include "bias_support.hpp"
#include <formal_eskf/eskf/magnetometer.hpp>

static_assert(ESKF_MAG_TILT == 1 && step_correction_proof::boundaries_clear &&
              correction_proof::measurement_size == 3U);

namespace magnetic_proof
{
using namespace step_correction_proof;
using correction_proof::Gain;
using Observation = formal_eskf::detail::MagneticObservation<Backend>;
constexpr std::size_t attitude_offset = state_size == 15U ? 6U : 0U;
struct Results
{
    Observation observation;
    Matrix3 projector, attitude_gain;
    Gain gain;
    Correction delta;
    Covariance joseph, reset;
    State injected;
    Status projector_status, gain_status, projection_status, correction_status, injection_status, reset_status;
};
struct Calls
{
    inline static Results const * results = nullptr;
    inline static State const * state = nullptr;
    inline static Covariance const * covariance = nullptr;
    inline static Vector3 const * measurement = nullptr;
    inline static Vector3 const * reference = nullptr;
    inline static Noise const * V = nullptr;
    inline static State const * state_target = nullptr;
    inline static Covariance const * covariance_target = nullptr;
    inline static State const * state_before = nullptr;
    inline static Covariance const * covariance_before = nullptr;
    inline static bias_proof::permissions_type const * permissions = nullptr;
    inline static Scalar minimum{};
    inline static unsigned observation{}, projector{}, gain{}, projection{}, correction{}, injection{}, reset{};
};
void unpublished()
{
    __ESBMC_assert(same_state(*Calls::state_target, *Calls::state_before) &&
                       same_matrix(*Calls::covariance_target, *Calls::covariance_before),
                   "E-MAG-TILT: public outputs remain unpublished until final reset succeeds");
}
bool jacobian_matches(Jacobian const & H)
{
    auto const & h = Calls::results->observation.h_m_b;
    formal_eskf::verification::ScalarArray<formal_eskf::verification::ScalarArray<Scalar, 3U>, 3U> block;
    block[0][0] = 0;
    block[0][1] = -h(2U);
    block[0][2] = h(1U);
    block[1][0] = h(2U);
    block[1][1] = 0;
    block[1][2] = -h(0U);
    block[2][0] = -h(1U);
    block[2][1] = h(0U);
    block[2][2] = 0;
    bool equal = true;
    for (std::size_t i = 0U; i < 3U; ++i)
        for (std::size_t j = 0U; j < state_size; ++j)
        {
            Scalar const expected =
                j >= attitude_offset && j < attitude_offset + 3U ? block[i][j - attitude_offset] : Scalar{0};
            equal = same(H(i, j), expected) && equal;
        }
    return equal;
}
} // namespace magnetic_proof

// Each summary is paired with an actual same-backend producer in the magnetic,
// bias, correction and step-correction suites. No finite/unit/PSD or success
// assumption is imposed on this caller proof. Scratch may be overwritten on
// failure except where the actual producer proves an atomic frame.
namespace formal_eskf::detail
{
template <>
inline magnetic_proof::Observation
prepare_magnetic_observation<magnetic_proof::Backend>(magnetic_proof::Quaternion const & q,
                                                      magnetic_proof::Vector3 const & measurement,
                                                      magnetic_proof::Vector3 const & reference)
{
    using namespace magnetic_proof;
    __ESBMC_assert(Calls::observation++ == 0U && Calls::projector == 0U && Calls::gain == 0U,
                   "E-MAG-TILT: observation preparation is first and once");
    __ESBMC_assert(same_matrix(q.coefficients(), Calls::state->q_nb.coefficients()) &&
                       same_matrix(measurement, *Calls::measurement) && same_matrix(reference, *Calls::reference),
                   "E-MAG-TILT: observation uses the original prior and both raw fields");
    unpublished();
    return Calls::results->observation;
}
template <>
inline Status magnetic_tilt_projector<magnetic_proof::Backend>(magnetic_proof::Matrix3 const & R,
                                                               magnetic_proof::Matrix3 & output)
{
    using namespace magnetic_proof;
    __ESBMC_assert(Calls::observation == 1U && Calls::results->observation.status == Status::success &&
                       Calls::projector++ == 0U && Calls::gain == 0U,
                   "E-MAG-TILT: projector is prepared once after successful observation");
    __ESBMC_assert(same_matrix(R, Calls::results->observation.R) && &output != &R &&
                       static_cast<void const *>(&output) != Calls::covariance_target,
                   "E-MAG-TILT: projector consumes the SAME prior rotation into separate scratch");
    unpublished();
    if (Calls::results->projector_status == Status::success)
        output = Calls::results->projector;
    return Calls::results->projector_status;
}
template <>
inline Status
compute_correction_gain<magnetic_proof::Backend, magnetic_proof::state_size, 3U,
                        (magnetic_proof::state_size == 15U ? formal_eskf::detail::CorrectionModel::body_vector
                                                           : formal_eskf::detail::CorrectionModel::arbitrary_h)>(
    magnetic_proof::Covariance const & P, magnetic_proof::Residual const & r, magnetic_proof::Jacobian const & H,
    magnetic_proof::Noise const & V, magnetic_proof::Scalar minimum, magnetic_proof::Gain & output)
{
    using namespace magnetic_proof;
    __ESBMC_assert(Calls::projector == 1U && Calls::results->projector_status == Status::success &&
                       Calls::gain++ == 0U && Calls::projection == 0U && Calls::correction == 0U,
                   "E-MAG-TILT: gain is solved once after successful projector");
    __ESBMC_assert(same_matrix(P, *Calls::covariance) && same_matrix(r, Calls::results->observation.r) &&
                       jacobian_matches(H) && same_matrix(V, *Calls::V) && same(minimum, Calls::minimum),
                   "E-MAG-TILT: unconstrained gain uses original prior/noise and full-field residual/Jacobian");
    __ESBMC_assert(static_cast<void const *>(&output) != &P &&
                       static_cast<void const *>(&output) != Calls::covariance_target,
                   "E-MAG-TILT: gain uses separate scratch");
    __ESBMC_assert(static_cast<void const *>(&output) != &H && static_cast<void const *>(&output) != &V &&
                       static_cast<void const *>(&output) != &r,
                   "E-MAG-TILT: gain output is disjoint from every measurement input");
    unpublished();
    output = Calls::results->gain;
    return Calls::results->gain_status;
}
template <>
inline Status
project_magnetic_gain<magnetic_proof::attitude_offset, magnetic_proof::Backend, magnetic_proof::state_size, 3U>(
    magnetic_proof::Matrix3 const & projector, magnetic_proof::Gain & gain)
{
    using namespace magnetic_proof;
    __ESBMC_assert(Calls::gain == 1U && Calls::results->gain_status == Status::success && Calls::projection++ == 0U &&
                       Calls::correction == 0U,
                   "E-MAG-TILT: attitude projection follows successful gain exactly once");
    __ESBMC_assert(same_matrix(projector, Calls::results->projector) &&
                       bias_proof::constrained_gain(gain, Calls::results->gain, *Calls::permissions),
                   "E-MAG-TILT: projection uses the original projector and all bias constraints");
    __ESBMC_assert(static_cast<void const *>(&projector) != &gain &&
                       static_cast<void const *>(&gain) != Calls::covariance_target &&
                       static_cast<void const *>(&gain) != Calls::covariance,
                   "E-MAG-TILT: projection producer receives disjoint projector and scratch gain");
    unpublished();
    if (Calls::results->projection_status == Status::success)
        gain.template set_block<attitude_offset, 0U>(Calls::results->attitude_gain);
    return Calls::results->projection_status;
}
template <>
inline Status
apply_correction_gain<magnetic_proof::Backend, magnetic_proof::state_size, 3U,
                      (magnetic_proof::state_size == 15U ? formal_eskf::detail::CorrectionModel::body_vector
                                                         : formal_eskf::detail::CorrectionModel::arbitrary_h)>(
    magnetic_proof::Covariance const & P, magnetic_proof::Residual const & r, magnetic_proof::Jacobian const & H,
    magnetic_proof::Noise const & V, magnetic_proof::Gain const & K, magnetic_proof::Correction & delta,
    magnetic_proof::Covariance & output)
{
    using namespace magnetic_proof;
    __ESBMC_assert(Calls::projection == 1U && Calls::results->projection_status == Status::success &&
                       Calls::correction++ == 0U && Calls::injection == 0U,
                   "E-MAG-TILT: correction/Joseph follows successful projection exactly once");
    __ESBMC_assert(same_matrix(P, *Calls::covariance) && same_matrix(r, Calls::results->observation.r) &&
                       jacobian_matches(H) && same_matrix(V, *Calls::V),
                   "E-MAG-TILT: full correction/Joseph retains prior P, H, residual and correlated V");
    for (std::size_t row = 0U; row < state_size; ++row)
        for (std::size_t column = 0U; column < 3U; ++column)
        {
            Scalar const expected =
                row >= attitude_offset && row < attitude_offset + 3U
                    ? Calls::results->attitude_gain(row - attitude_offset, column)
                    : (bias_proof::permitted(*Calls::permissions, row) ? Calls::results->gain(row, column) : Scalar{0});
            __ESBMC_assert(same(K(row, column), expected),
                           "E-MAG-TILT: the SAME projected and bias-constrained gain reaches delta AND Joseph");
        }
    __ESBMC_assert(&output != &P && &output != Calls::covariance_target && static_cast<void const *>(&delta) != &r,
                   "E-MAG-TILT: linear correction outputs use scratch");
    __ESBMC_assert(static_cast<void const *>(&output) != &H && static_cast<void const *>(&output) != &V &&
                       static_cast<void const *>(&output) != &K && static_cast<void const *>(&output) != &r &&
                       static_cast<void const *>(&delta) != &P && static_cast<void const *>(&delta) != &H &&
                       static_cast<void const *>(&delta) != &V && static_cast<void const *>(&delta) != &K &&
                       static_cast<void const *>(&delta) != &output,
                   "E-MAG-TILT: delta and covariance outputs are disjoint from each other and every input");
    unpublished();
    delta = Calls::results->delta;
    output = Calls::results->joseph;
    return Calls::results->correction_status;
}
} // namespace formal_eskf::detail
namespace formal_eskf
{
template <>
inline Status inject_nominal<magnetic_proof::Backend>(magnetic_proof::State const & state,
                                                      magnetic_proof::Error const & error,
                                                      magnetic_proof::Scalar minimum, magnetic_proof::State & output)
{
    using namespace magnetic_proof;
    __ESBMC_assert(Calls::correction == 1U && Calls::results->correction_status == Status::success &&
                       Calls::injection++ == 0U && Calls::reset == 0U,
                   "E-MAG-TILT: injection occurs once only after successful correction");
    __ESBMC_assert(same_state(state, *Calls::state) && unpacked(error, Calls::results->delta) &&
                       same(minimum, Calls::minimum),
                   "E-MAG-TILT: injection receives OLD nominal state and every unpacked error coordinate");
    __ESBMC_assert(&output != &state && &output != Calls::state_target,
                   "E-MAG-TILT: nominal injection writes separate scratch");
    unpublished();
    output = Calls::results->injected;
    return Calls::results->injection_status;
}

template <>
inline Status reset_covariance<magnetic_proof::Backend>(magnetic_proof::Error const & error,
                                                        magnetic_proof::Covariance const & covariance,
                                                        magnetic_proof::Covariance & output)
{
    using namespace magnetic_proof;
    __ESBMC_assert(Calls::injection == 1U && Calls::results->injection_status == Status::success &&
                       Calls::reset++ == 0U,
                   "E-MAG-TILT: reset occurs once only after successful injection");
    __ESBMC_assert(unpacked(error, Calls::results->delta) && same_matrix(covariance, Calls::results->joseph),
                   "E-MAG-TILT: reset uses the complete Joseph covariance and error BEFORE zeroing");
    __ESBMC_assert(&output != &covariance && &output == Calls::covariance_target,
                   "E-MAG-TILT: final reset receives caller output, disjoint from Joseph P");
    unpublished();
    if (Calls::results->reset_status == Status::success)
    {
        output = Calls::results->reset;
    }
    return Calls::results->reset_status;
}
} // namespace formal_eskf

using namespace magnetic_proof;
void verify_magnetic_correction(State state, Covariance covariance, Vector3 measurement, Vector3 reference, Noise V,
                                Scalar minimum, State state_output, Covariance covariance_output, Results results,
                                bias_proof::permissions_type permissions)
{
    static_assert(FORMAL_ESKF_PROOF_STEP_ALIAS >= 0 && FORMAL_ESKF_PROOF_STEP_ALIAS < 4);
    constexpr bool state_alias = (FORMAL_ESKF_PROOF_STEP_ALIAS & 1) != 0;
    constexpr bool covariance_alias = (FORMAL_ESKF_PROOF_STEP_ALIAS & 2) != 0;
    auto const old_state = state;
    auto const old_covariance = covariance;
    auto const old_measurement = measurement;
    auto const old_reference = reference;
    auto const old_V = V;
    auto const old_permissions = permissions;
    auto const old_state_output = state_output;
    auto const old_covariance_output = covariance_output;
    State * state_pointer = &state_output;
    Covariance * covariance_pointer = &covariance_output;
    if constexpr (state_alias)
        state_pointer = &state;
    if constexpr (covariance_alias)
        covariance_pointer = &covariance;
    auto const before_state = *state_pointer;
    auto const before_covariance = *covariance_pointer;
    Calls::results = &results;
    Calls::state = &old_state;
    Calls::covariance = &old_covariance;
    Calls::measurement = &old_measurement;
    Calls::reference = &old_reference;
    Calls::V = &old_V;
    Calls::permissions = &old_permissions;
    Calls::minimum = minimum;
    Calls::state_target = state_pointer;
    Calls::covariance_target = covariance_pointer;
    Calls::state_before = &before_state;
    Calls::covariance_before = &before_covariance;
    Calls::observation = Calls::projector = Calls::gain = Calls::projection = Calls::correction = Calls::injection =
        Calls::reset = 0U;
    auto const status = formal_eskf::correct_magnetometer(state, covariance, measurement, reference, V, minimum,
                                                          *state_pointer, *covariance_pointer
#if FORMAL_ESKF_PROOF_STATE_SIZE == 15
                                                          ,
                                                          permissions
#endif
    );
    bool const observation_ok = results.observation.status == Status::success;
    bool const projector_ok = observation_ok && results.projector_status == Status::success;
    bool const gain_ok = projector_ok && results.gain_status == Status::success;
    bool const projection_ok = gain_ok && results.projection_status == Status::success;
    bool const correction_ok = projection_ok && results.correction_status == Status::success;
    bool const injection_ok = correction_ok && results.injection_status == Status::success;
    Status const expected = !observation_ok  ? results.observation.status
                            : !projector_ok  ? results.projector_status
                            : !gain_ok       ? results.gain_status
                            : !projection_ok ? results.projection_status
                            : !correction_ok ? results.correction_status
                            : !injection_ok  ? results.injection_status
                                             : results.reset_status;
    __ESBMC_assert(status == expected && Calls::observation == 1U && Calls::projector == (observation_ok ? 1U : 0U) &&
                       Calls::gain == (projector_ok ? 1U : 0U) && Calls::projection == (gain_ok ? 1U : 0U) &&
                       Calls::correction == (projection_ok ? 1U : 0U) &&
                       Calls::injection == (correction_ok ? 1U : 0U) && Calls::reset == (injection_ok ? 1U : 0U),
                   "E-MAG-TILT: exact first failure and no repeated, skipped or post-failure stages");
    __ESBMC_assert(status == Status::success
                       ? same_state(*state_pointer, results.injected) && same_matrix(*covariance_pointer, results.reset)
                       : same_state(*state_pointer, before_state) &&
                             same_matrix(*covariance_pointer, before_covariance),
                   "E-MAG-TILT: complete publication on success and atomic failure for every alias");
    __ESBMC_assert(state_alias ? same_state(state_output, old_state_output) : same_state(state, old_state),
                   "E-MAG-TILT: preserve distinct state input or unused output");
    __ESBMC_assert(covariance_alias ? same_matrix(covariance_output, old_covariance_output)
                                    : same_matrix(covariance, old_covariance),
                   "E-MAG-TILT: preserve distinct covariance input or unused output");
    __ESBMC_assert(same_matrix(measurement, old_measurement) && same_matrix(reference, old_reference) &&
                       same_matrix(V, old_V) && bias_proof::same_permissions(permissions, old_permissions),
                   "E-MAG-TILT: preserve fields, noise and caller permissions");
}
int main() { return 0; }
