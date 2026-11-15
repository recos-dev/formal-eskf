/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "contracts.hpp"
#include <cstdint>
#include <limits>
#define private public
#include <formal_eskf/runtime/magnetic_reference.hpp>
#undef private
#ifndef FORMAL_ESKF_PROOF_MAG_REFERENCE_CONTRACT
#define FORMAL_ESKF_PROOF_MAG_REFERENCE_CONTRACT 0
#endif
namespace reference_proof
{
using contract_proof::finite_vector;
using contract_proof::Linalg;
using contract_proof::OpaqueMath;
using contract_proof::same;
using contract_proof::same_vector;
using contract_proof::Scalar;
using formal_eskf::Status;
using Reference = formal_eskf::runtime::MagneticReference<Linalg>;
using Sample = Reference::Sample;
using State = Reference::State;
using Vector = Reference::vector_type;
using Pair = formal_eskf::scalar::SinCos<Scalar>;
inline bool same_sample(Sample const & a, Sample const & b)
{
    return same(a.declination, b.declination) && same(a.inclination, b.inclination) && same(a.strength, b.strength);
}
inline bool same_state(State const & a, State const & b)
{
    return same_sample(a.model, b.model) && same_vector(a.field_n, b.field_n) && a.reference_id == b.reference_id &&
           a.last_check_time_us == b.last_check_time_us;
}
inline Status sample_status(Sample const & sample)
{
    constexpr Scalar pi = std::numbers::pi_v<Scalar>;
    if (!std::isfinite(sample.declination) || !std::isfinite(sample.inclination) || !std::isfinite(sample.strength))
    {
        return Status::non_finite_input;
    }
    if (sample.declination < -pi || sample.declination > pi || sample.inclination < -pi / Scalar{2} ||
        sample.inclination > pi / Scalar{2} || sample.strength < std::numeric_limits<Scalar>::min())
    {
        return Status::domain_error;
    }
    return Status::success;
}
inline Status pair_status(Scalar angle, Pair pair)
{
    return !std::isfinite(angle)                                      ? Status::non_finite_input
           : !std::isfinite(pair.sine) || !std::isfinite(pair.cosine) ? Status::non_finite_result
                                                                      : Status::success;
}
struct Calls
{
    inline static Sample sample;
    inline static Pair declination, inclination;
    inline static unsigned trig{}, adoptions{};
    inline static Reference const * before{};
    inline static Vector field;
    inline static Status status{};
};
} // namespace reference_proof
#if FORMAL_ESKF_PROOF_MAG_REFERENCE_CONTRACT == 1 || FORMAL_ESKF_PROOF_MAG_REFERENCE_CONTRACT == 3
// Closed by verify_reference_trig below: actual checked sin/cos on OpaqueMath.
// This exact status/publication relation admits every IEEE libm return.
namespace formal_eskf::scalar
{
template <>
inline Status sin_cos<reference_proof::OpaqueMath>(reference_proof::Scalar angle, reference_proof::Pair & output)
{
    using namespace reference_proof;
    __ESBMC_assert(Calls::trig < 2U, "R-MAGNETIC-REFERENCE: at most declination and inclination trigonometry");
    bool const first = Calls::trig++ == 0U;
    __ESBMC_assert(same(angle, first ? Calls::sample.declination : Calls::sample.inclination),
                   "R-MAGNETIC-REFERENCE: declination then inclination reach checked sin/cos unchanged");
    Pair const pair{first ? Calls::declination.sine : Calls::inclination.sine,
                    first ? Calls::declination.cosine : Calls::inclination.cosine};
#if FORMAL_ESKF_PROOF_MAG_REFERENCE_CONTRACT == 3
    __ESBMC_assert(pair_status(angle, pair) == Status::success,
                   "R-MAGNETIC-REFERENCE: numeric path discharges successful trig domain");
    output = pair;
    return Status::success;
#else
    auto const status = pair_status(angle, pair);
    if (status == Status::success)
    {
        output = pair;
    }
    return status;
#endif
}
} // namespace formal_eskf::scalar
#endif
#if FORMAL_ESKF_PROOF_MAG_REFERENCE_CONTRACT == 2
// Exact-type adopt producer proves this status/write frame. Finite successful
// fields are unconstrained; no trig accuracy or physical WMM premise is added.
namespace formal_eskf::runtime
{
template <> inline Status MagneticReference<reference_proof::Linalg>::adopt(reference_proof::Sample const & sample)
{
    using namespace reference_proof;
    __ESBMC_assert(Calls::adoptions++ == 0U && same_sample(sample, Calls::sample) &&
                       same_state(m_state, Calls::before->m_state) &&
                       m_update_time_us == Calls::before->m_update_time_us,
                   "R-MAGNETIC-REFERENCE: adoption receives original sample and unchanged owner history once");
    if (m_state.reference_id == std::numeric_limits<std::uint32_t>::max())
    {
        return Status::out_of_range;
    }
    if (Calls::status == Status::success)
    {
        m_state.model = sample;
        m_state.field_n = Calls::field;
        ++m_state.reference_id;
    }
    return Calls::status;
}
} // namespace formal_eskf::runtime
#endif
void verify_reference_trig(reference_proof::Scalar angle, reference_proof::Pair output, reference_proof::Pair pair)
{
    using namespace reference_proof;
    __ESBMC_assert(FORMAL_ESKF_PROOF_MAG_REFERENCE_CONTRACT == 0, "Runner error: check actual scalar producer");
    auto const before = output;
    OpaqueMath::prepare(Scalar{0}, pair.sine, pair.cosine, Scalar{0});
    auto const actual = formal_eskf::scalar::sin_cos<OpaqueMath>(angle, output);
    auto const expected = pair_status(angle, pair);
    __ESBMC_assert(actual == expected && same(output.sine, actual == Status::success ? pair.sine : before.sine) &&
                       same(output.cosine, actual == Status::success ? pair.cosine : before.cosine),
                   "R-MAGNETIC-REFERENCE: actual trig wrapper status and two-coefficient atomic publication");
    unsigned const count = std::isfinite(angle) ? 1U : 0U;
    __ESBMC_assert(OpaqueMath::sin.calls == count && OpaqueMath::cos.calls == count &&
                       (!count || (same(OpaqueMath::sin.argument, angle) && same(OpaqueMath::cos.argument, angle))),
                   "R-MAGNETIC-REFERENCE: validated angle reaches each primitive exactly once");
}
void verify_reference_initialization(reference_proof::Reference reference)
{
    using namespace reference_proof;
    Reference fresh;
    State zero;
    __ESBMC_assert(same_state(fresh.state(), zero) && fresh.m_update_time_us == 0U,
                   "R-MAGNETIC-REFERENCE: construction establishes empty reference, revision and both clocks");
    reference.reset();
    __ESBMC_assert(same_state(reference.state(), zero) && reference.m_update_time_us == 0U,
                   "R-MAGNETIC-REFERENCE: reset discards every state field and both clocks");
    auto const & state = reference.state();
    __ESBMC_assert(&state == &reference.m_state, "R-MAGNETIC-REFERENCE: accessor refers to owned state");
}
void verify_reference_validation(reference_proof::Sample sample)
{
    using namespace reference_proof;
    auto const before = sample;
    __ESBMC_assert(Reference::validate_sample(sample) == sample_status(sample) && same_sample(sample, before),
                   "R-MAGNETIC-REFERENCE: exact finite, angular and positive-normal strength domain with input purity");
}
// This producer establishes atomic finite-field and failure-status contracts.
// Successful scalar paths compose with verify_reference_coefficients below.
void verify_reference_adoption_frame(reference_proof::Reference reference, reference_proof::Sample sample,
                                     reference_proof::Pair declination, reference_proof::Pair inclination)
{
    using namespace reference_proof;
    __ESBMC_assert(FORMAL_ESKF_PROOF_MAG_REFERENCE_CONTRACT == 1,
                   "Runner error: adoption frame uses proved checked trig");
    auto const before = reference;
    auto const old_sample = sample;
    Calls::sample = sample;
    Calls::declination = declination;
    Calls::inclination = inclination;
    Calls::trig = 0U;
    auto const actual = reference.adopt(sample);
    if (before.m_state.reference_id == std::numeric_limits<std::uint32_t>::max())
    {
        __ESBMC_assert(actual == Status::out_of_range && Calls::trig == 0U,
                       "R-MAGNETIC-REFERENCE: saturated revision rejects before any numerical call");
    }
    auto const first = pair_status(sample.declination, declination),
               second = pair_status(sample.inclination, inclination);
    if (before.m_state.reference_id != std::numeric_limits<std::uint32_t>::max())
    {
        __ESBMC_assert(
            Calls::trig == (first == Status::success ? 2U : 1U) && (first == Status::success || actual == first) &&
                (first != Status::success || second == Status::success || actual == second),
            "R-MAGNETIC-REFERENCE: both checked-trig failures propagate without later evaluation or publication");
    }
    if (actual == Status::success)
    {
        __ESBMC_assert(same_sample(reference.m_state.model, sample) && finite_vector(reference.m_state.field_n) &&
                           reference.m_state.reference_id == before.m_state.reference_id + 1U &&
                           reference.m_state.reference_id > before.m_state.reference_id && Calls::trig == 2U,
                       "R-MAGNETIC-REFERENCE: successful adoption publishes the supplied model and finite field with "
                       "one revision advance");
    }
    else
    {
        __ESBMC_assert(same_state(reference.m_state, before.m_state),
                       "R-MAGNETIC-REFERENCE: every failed adoption preserves the entire prior reference");
    }
    __ESBMC_assert(reference.m_state.last_check_time_us == before.m_state.last_check_time_us &&
                       reference.m_update_time_us == before.m_update_time_us && same_sample(sample, old_sample),
                   "R-MAGNETIC-REFERENCE: adoption leaves both clocks and the borrowed model unchanged");
}
void verify_reference_coefficients(reference_proof::Reference reference, reference_proof::Sample sample,
                                   reference_proof::Pair declination, reference_proof::Pair inclination)
{
    using namespace reference_proof;
    __ESBMC_assert(FORMAL_ESKF_PROOF_MAG_REFERENCE_CONTRACT == 3,
                   "Runner error: numeric branch composes successful trig path");
    // Exact successful-wrapper preconditions, proved by verify_reference_trig.
    // Failure frames/statuses belong to verify_reference_adoption_frame.
    __ESBMC_assume(pair_status(sample.declination, declination) == Status::success &&
                   pair_status(sample.inclination, inclination) == Status::success);
    auto const before = reference;
    Calls::sample = sample;
    Calls::declination = declination;
    Calls::inclination = inclination;
    Calls::trig = 0U;
    Vector field;
    field(0U) = sample.strength * inclination.cosine * declination.cosine;
    field(1U) = sample.strength * inclination.cosine * declination.sine;
    field(2U) = sample.strength * inclination.sine;
    auto const expected = before.m_state.reference_id == std::numeric_limits<std::uint32_t>::max()
                              ? Status::out_of_range
                          : finite_vector(field) ? Status::success
                                                 : Status::non_finite_result;
    auto const result = reference.adopt(sample);
    __ESBMC_assert(result == expected,
                   "R-MAGNETIC-REFERENCE: successful trigonometry still rejects unrepresentable NED fields");
    __ESBMC_assert(result != Status::success || same_vector(reference.m_state.field_n, field),
                   "R-MAGNETIC-REFERENCE: all three NED coefficients retain exact ordered IEEE products");
}
void verify_reference_update(reference_proof::Reference reference, std::uint64_t now, reference_proof::Sample sample,
                             bool position_valid, bool origin_updated, reference_proof::Status adoption_status,
                             reference_proof::Vector field)
{
    using namespace reference_proof;
    __ESBMC_assert(FORMAL_ESKF_PROOF_MAG_REFERENCE_CONTRACT == 2,
                   "Runner error: update requires actual validation and proved adoption frame");
    // Base is construction/reset. Every successful update establishes these
    // field/clock facts; every failure preserves them. No bounded trace premise.
    __ESBMC_assume(reference.m_state.last_check_time_us <= reference.m_update_time_us &&
                   finite_vector(reference.m_state.field_n));
    __ESBMC_assume(reference.m_state.reference_id == 0U || sample_status(reference.m_state.model) == Status::success);
    __ESBMC_assume(adoption_status != Status::success || finite_vector(field));
    auto const before = reference;
    auto const old_sample = sample;
    Calls::sample = sample;
    Calls::before = &before;
    Calls::field = field;
    Calls::status = adoption_status;
    Calls::adoptions = 0U;
    bool const initialized = before.m_state.reference_id != 0U;
    bool const increasing = now > before.m_update_time_us;
    bool const eligible =
        origin_updated || (position_valid && (!initialized || now - before.m_state.last_check_time_us > 10000000U));
    bool const checked = increasing && eligible;
    constexpr Scalar threshold = std::numbers::pi_v<Scalar> / Scalar{180};
    bool const changed = !initialized || std::fabs(sample.declination - before.m_state.model.declination) > threshold ||
                         std::fabs(sample.inclination - before.m_state.model.inclination) > threshold ||
                         std::fabs(sample.strength - before.m_state.model.strength) > Scalar{1};
    bool const adopt = checked && sample_status(sample) == Status::success && changed;
    Status const result_status =
        adopt ? (before.m_state.reference_id == std::numeric_limits<std::uint32_t>::max() ? Status::out_of_range
                                                                                          : adoption_status)
              : Status::success;
    Status const expected = !increasing                                ? Status::out_of_range
                            : !checked                                 ? Status::success
                            : sample_status(sample) != Status::success ? sample_status(sample)
                                                                       : result_status;
    auto const result = reference.update(now, sample, position_valid, origin_updated);
    __ESBMC_assert(
        result.status == expected && result.checked == checked &&
            result.changed == (adopt && expected == Status::success) && Calls::adoptions == (adopt ? 1U : 0U),
        "R-MAGNETIC-REFERENCE: exact clock/position/origin admission, strict change thresholds and adoption outcome");
    State state = before.m_state;
    if (expected == Status::success && checked)
    {
        state.last_check_time_us = now;
        if (adopt)
        {
            state.model = sample;
            state.field_n = field;
            ++state.reference_id;
        }
    }
    __ESBMC_assert(
        same_state(reference.m_state, state) &&
            reference.m_update_time_us == (expected == Status::success ? now : before.m_update_time_us),
        "R-MAGNETIC-REFERENCE: skipped checks advance only update time; failure preserves all fields and clocks");
    __ESBMC_assert(
        reference.m_state.last_check_time_us <= reference.m_update_time_us &&
            finite_vector(reference.m_state.field_n) &&
            (reference.m_state.reference_id == 0U || sample_status(reference.m_state.model) == Status::success),
        "R-MAGNETIC-REFERENCE: clock order, finite field and adopted-model domain are inductive");
    __ESBMC_assert(reference.m_state.reference_id >= before.m_state.reference_id && same_sample(sample, old_sample),
                   "R-MAGNETIC-REFERENCE: revision never wraps and sample remains unchanged");
}
int main() { return 0; }
