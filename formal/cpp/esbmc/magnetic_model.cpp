/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "contracts.hpp"
#ifndef FORMAL_ESKF_PROOF_MODEL_LEVEL_ALIAS
#define FORMAL_ESKF_PROOF_MODEL_LEVEL_ALIAS 0
#endif
#include <array>
#include <optional>
#define private public
#include <formal_eskf/runtime/magnetic_reference.hpp>
#undef private
namespace model_proof
{
using contract_proof::finite_vector;
using contract_proof::Linalg;
using contract_proof::Quaternion;
using contract_proof::same;
using contract_proof::same_vector;
using contract_proof::Scalar;
using formal_eskf::Status;
using Reference = formal_eskf::runtime::MagneticReference<Linalg>;
using Sample = Reference::Sample;
using Vector = Reference::vector_type;
using Result = Reference::Result;
inline bool same_sample(Sample const & a, Sample const & b)
{
    return same(a.declination, b.declination) && same(a.inclination, b.inclination) && same(a.strength, b.strength);
}
inline bool same_reference(Reference const & a, Reference const & b)
{
    return same_sample(a.m_state.model, b.m_state.model) && same_vector(a.m_state.field_n, b.m_state.field_n) &&
           a.m_state.reference_id == b.m_state.reference_id &&
           a.m_state.last_check_time_us == b.m_state.last_check_time_us && a.m_update_time_us == b.m_update_time_us;
}
inline bool valid_sample(Sample const & v)
{
    constexpr Scalar pi = std::numbers::pi_v<Scalar>;
    return std::isfinite(v.declination) && std::isfinite(v.inclination) && std::isfinite(v.strength) &&
           v.declination >= -pi && v.declination <= pi && v.inclination >= -pi / Scalar{2} &&
           v.inclination <= pi / Scalar{2} && v.strength >= std::numeric_limits<Scalar>::min();
}
inline bool valid_reference(Reference const & r)
{
    return r.m_state.last_check_time_us <= r.m_update_time_us && finite_vector(r.m_state.field_n) &&
           (!r.m_state.reference_id || valid_sample(r.m_state.model));
}
struct Calls
{
    inline static Reference before;
    inline static Sample sample;
    inline static Vector field, level;
    inline static Result result;
    inline static std::uint64_t now;
    inline static bool position, origin;
    inline static Scalar horizontal, sine, cosine, declination;
    inline static unsigned references, roots, sines, cosines;
};
inline Reference updated(Reference const & before, std::uint64_t now, Sample const & sample, Result result,
                         Vector const & field)
{
    auto after = before;
    if (result.status == Status::success)
    {
        after.m_update_time_us = now;
        if (result.checked)
        {
            after.m_state.last_check_time_us = now;
        }
        if (result.changed)
        {
            after.m_state.model = sample;
            after.m_state.field_n = field;
            ++after.m_state.reference_id;
        }
    }
    return after;
}
inline void constrain_result(Reference const & before, Sample const & sample, Result result, Vector const & field)
{
    // Actual same-type update establishes these facts (magnetic_reference.cpp).
    // All failure statuses and unchanged/unchecked results remain admitted.
    __ESBMC_assume(!result.changed ||
                   (result.status == Status::success && result.checked && valid_sample(sample) &&
                    finite_vector(field) && before.m_state.reference_id < std::numeric_limits<std::uint32_t>::max()));
}
} // namespace model_proof
namespace formal_eskf::runtime
{
template <>
inline model_proof::Result MagneticReference<model_proof::Linalg>::update(std::uint64_t now,
                                                                          model_proof::Sample const & sample,
                                                                          bool position, bool origin)
{
    using namespace model_proof;
    __ESBMC_assert(Calls::references++ == 0U && same_reference(*this, Calls::before) &&
                       same_sample(sample, Calls::sample) && now == Calls::now && position == Calls::position &&
                       origin == Calls::origin && valid_reference(*this) && now > m_update_time_us,
                   "PX4 magnetic model: original model request and valid increasing producer history");
    *this = updated(*this, now, sample, Calls::result, Calls::field);
    return Calls::result;
}
} // namespace formal_eskf::runtime
// Pure arbitrary platform-libm returns. No accuracy/range assumptions. The
// token aliases apply only to the actual module header below, not core math.
namespace std
{
inline model_proof::Scalar formal_eskf_model_hypot(model_proof::Scalar x, model_proof::Scalar y)
{
    using namespace model_proof;
    __ESBMC_assert(Calls::roots++ == 0U && Calls::references == 0U && same(x, Calls::level(0U)) &&
                       same(y, Calls::level(1U)),
                   "PX4 magnetic model: seed horizontal norm receives original North/East before model lookup");
    return Calls::horizontal;
}
inline model_proof::Scalar formal_eskf_model_cos(model_proof::Scalar value)
{
    using namespace model_proof;
    __ESBMC_assert(Calls::cosines++ == 0U && Calls::roots == 1U && same(value, Calls::declination),
                   "PX4 magnetic model: selected declination reaches cosine");
    return Calls::cosine;
}
inline model_proof::Scalar formal_eskf_model_sin(model_proof::Scalar value)
{
    using namespace model_proof;
    __ESBMC_assert(Calls::sines++ == 0U && Calls::roots == 1U && same(value, Calls::declination),
                   "PX4 magnetic model: selected declination reaches sine");
    return Calls::sine;
}
} // namespace std
#define hypot formal_eskf_model_hypot
#define cos formal_eskf_model_cos
#define sin formal_eskf_model_sin
#define private public
#include "../../../integrations/px4/module/MagneticModel.hpp"
#undef private
#undef sin
#undef cos
#undef hypot
namespace model_proof
{
using Model = formal_eskf::px4::MagneticModel<Linalg>;
using Settings = Model::Settings;
using Heading = Model::HeadingSource;
inline bool same_settings(Settings const & a, Settings const & b)
{
    return a.declination_source == b.declination_source && same(a.saved_declination, b.saved_declination);
}
inline bool same_model(Model const & a, Model const & b)
{
    return same_reference(a.m_model, b.m_model) && same_settings(a.m_settings, b.m_settings) &&
           same_vector(a.m_observation, b.m_observation) && same(a.m_declination, b.m_declination) &&
           a.m_heading_source == b.m_heading_source && a.m_reference_id == b.m_reference_id &&
           a.m_last_lookup_time == b.m_last_lookup_time && a.m_lookup_count == b.m_lookup_count;
}
inline bool public_view(Model const & m)
{
    return same_vector(m.observation_n(), m.m_observation) && same_vector(m.health_n(), m.m_model.m_state.field_n) &&
           m.available() == (m.m_model.m_state.reference_id != 0U) && m.reference_id() == m.m_reference_id &&
           m.model_revision() == m.m_model.m_state.reference_id && m.lookup_count() == m.m_lookup_count &&
           m.last_lookup_time() == m.m_last_lookup_time && m.heading_source() == m.m_heading_source &&
           same(m.declination(), m.m_declination);
}
inline bool valid_model(Model const & m)
{
    constexpr Scalar pi = std::numbers::pi_v<Scalar>;
    bool const initialized = m.m_reference_id != 0U;
    return valid_reference(m.m_model) && m.m_model.m_update_time_us <= m.m_last_lookup_time &&
           m.m_lookup_count <= m.m_last_lookup_time && initialized == (m.m_lookup_count != 0U) &&
           initialized == (m.m_last_lookup_time != 0U) && m.m_settings.declination_source <= 3U &&
           std::isfinite(m.m_settings.saved_declination) && std::fabs(m.m_settings.saved_declination) <= pi &&
           std::isfinite(m.m_declination) && std::fabs(m.m_declination) <= pi &&
           (m.m_heading_source == Heading::magnetic_north || m.m_heading_source == Heading::saved ||
            (m.m_heading_source == Heading::geographic && (m.m_settings.declination_source & 1U) && m.available()));
}
inline void prepare(Reference const & before, Sample sample, Result result, Vector field, std::uint64_t now,
                    bool position, bool origin)
{
    Calls::before = before;
    Calls::sample = sample;
    Calls::result = result;
    Calls::field = field;
    Calls::now = now;
    Calls::position = position;
    Calls::origin = origin;
    Calls::references = Calls::roots = Calls::sines = Calls::cosines = 0U;
    constrain_result(before, sample, result, field);
}
inline bool same_result(Result a, Result b)
{
    return a.status == b.status && a.checked == b.checked && a.changed == b.changed;
}
} // namespace model_proof
void verify_model_initialization()
{
    using namespace model_proof;
    Model m;
    __ESBMC_assert(valid_model(m) && public_view(m) && !m.available() && m.reference_id() == 0U &&
                       m.lookup_count() == 0U && m.last_lookup_time() == 0U &&
                       m.heading_source() == Heading::magnetic_north && same(m.declination(), Scalar{0}),
                   "PX4 magnetic model: construction establishes uninitialized clocks, model and heading");
    Vector zero;
    __ESBMC_assert(same_vector(m.observation_n(), zero) && same_vector(m.health_n(), zero),
                   "PX4 magnetic model: no fabricated field at construction");
}
void verify_model_initialize(model_proof::Model model, std::uint64_t now, model_proof::Settings settings,
                             model_proof::Vector independent_level, bool has_model, model_proof::Sample sample,
                             model_proof::Result reply, model_proof::Vector field, model_proof::Scalar horizontal,
                             model_proof::Scalar sine, model_proof::Scalar cosine)
{
    using namespace model_proof;
    static_assert(FORMAL_ESKF_PROOF_MODEL_LEVEL_ALIAS < 3);
#if FORMAL_ESKF_PROOF_MODEL_LEVEL_ALIAS == 0
    auto const & level = independent_level;
#elif FORMAL_ESKF_PROOF_MODEL_LEVEL_ALIAS == 1
    auto const & level = model.observation_n();
#else
    auto const & level = model.health_n();
#endif

    __ESBMC_assume(valid_model(model));
    auto const before = model;
    auto const old_level = level;
    auto const old_settings = settings;
    auto const old_sample = sample;
    Reference empty;
    prepare(empty, sample, reply, field, now, true, true);
    Calls::level = level;
    Calls::horizontal = horizontal;
    Calls::sine = sine;
    Calls::cosine = cosine;
    auto expected = before;
    constexpr Scalar pi = std::numbers::pi_v<Scalar>;
    bool const valid = now != 0U && settings.declination_source <= 3U && std::isfinite(settings.saved_declination) &&
                       std::fabs(settings.saved_declination) <= pi && finite_vector(level);
    bool const norm_ok = horizontal > Scalar{0} && std::isfinite(horizontal);
    Status wanted = !valid || !norm_ok ? Status::domain_error : has_model ? reply.status : Status::success;
    bool const called = valid && norm_ok && has_model;
    bool geographic = false;
    if (wanted == Status::success)
    {
        expected = Model{};
        expected.m_settings = settings;
        if (has_model)
        {
            expected.m_model = updated(empty, now, sample, reply, field);
        }
        geographic = (settings.declination_source & 1U) && expected.available();
        if (geographic)
        {
            expected.m_declination = expected.m_model.state().model.declination;
            expected.m_heading_source = Heading::geographic;
            expected.m_observation = expected.health_n();
        }
        else
        {
            if ((settings.declination_source & 2U) && settings.saved_declination != Scalar{0})
            {
                expected.m_declination = settings.saved_declination;
                expected.m_heading_source = Heading::saved;
            }
            expected.m_observation(0U) = horizontal * cosine;
            expected.m_observation(1U) = horizontal * sine;
            expected.m_observation(2U) = level(2U);
        }
        expected.m_reference_id = 1U;
        expected.m_last_lookup_time = now;
        expected.m_lookup_count = 1U;
    }
    Calls::declination = expected.m_declination;
    std::optional<Sample> supplied;
    if (has_model)
    {
        supplied = sample;
    }
    auto const status = model.initialize(now, settings, level, supplied);
    __ESBMC_assert(supplied.has_value() == has_model && (!has_model || same_sample(*supplied, old_sample)),
                   "PX4 magnetic model: borrowed optional presence and sample remain unchanged");
    __ESBMC_assert(status == wanted && same_model(model, expected) && Calls::roots == (valid ? 1U : 0U) &&
                       Calls::references == (called ? 1U : 0U) &&
                       Calls::sines == (wanted == Status::success && !geographic ? 1U : 0U) &&
                       Calls::cosines == Calls::sines,
                   "PX4 magnetic model: validation, source priority, fallback coefficients and complete initialization "
                   "transaction");
    __ESBMC_assert(valid_model(model) && public_view(model) &&
                       ((FORMAL_ESKF_PROOF_MODEL_LEVEL_ALIAS != 0 && status == Status::success) ||
                        same_vector(level, old_level)) &&
                       same_settings(settings, old_settings) && same_sample(sample, old_sample),
                   "PX4 magnetic model: initialization preserves inputs and establishes clock/source invariants");
}
void verify_model_update(model_proof::Model model, std::uint64_t now, bool has_model, model_proof::Sample sample,
                         bool position, bool origin, model_proof::Result reply, model_proof::Vector field)
{
    using namespace model_proof;
    __ESBMC_assume(valid_model(model));
    auto const before = model;
    auto const old_sample = sample;
    prepare(model.m_model, sample, reply, field, now, position, origin);
    auto expected = before;
    Result wanted;
    bool const increasing = now > before.m_last_lookup_time;
    bool const due = increasing && (origin || (position && now - before.m_last_lookup_time > 10000000U));
    __ESBMC_assert(model.lookup_due(now, position, origin) == due,
                   "PX4 magnetic model: strict lookup threshold, origin trigger and increasing timestamp");
    bool const called = before.m_reference_id != 0U && due && has_model;
    if (!before.m_reference_id || !increasing)
    {
        wanted.status = Status::out_of_range;
    }
    else if (due)
    {
        expected.m_last_lookup_time = now;
        expected.m_lookup_count = before.m_lookup_count + 1U;
        wanted.checked = true;
        if (has_model)
        {
            wanted = reply;
            if (reply.status == Status::success)
            {
                auto const candidate = updated(before.m_model, now, sample, reply, field);
                if (reply.changed && (before.m_settings.declination_source & 1U))
                {
                    if (before.m_reference_id == std::numeric_limits<std::uint32_t>::max())
                    {
                        wanted.status = Status::out_of_range;
                        wanted.changed = false;
                    }
                    else
                    {
                        expected.m_observation = candidate.state().field_n;
                        expected.m_declination = candidate.state().model.declination;
                        expected.m_heading_source = Heading::geographic;
                        ++expected.m_reference_id;
                    }
                }
                if (wanted.status == Status::success)
                {
                    expected.m_model = candidate;
                }
            }
        }
    }
    std::optional<Sample> supplied;
    if (has_model)
    {
        supplied = sample;
    }
    auto const actual = model.update(now, supplied, position, origin);
    __ESBMC_assert(supplied.has_value() == has_model && (!has_model || same_sample(*supplied, old_sample)),
                   "PX4 magnetic model: borrowed optional presence and sample remain unchanged");
    __ESBMC_assert(same_result(actual, wanted) && same_model(model, expected) &&
                       Calls::references == (called ? 1U : 0U) && Calls::roots == 0U && Calls::sines == 0U &&
                       Calls::cosines == 0U,
                   "PX4 magnetic model: attempts advance scheduling; only successful authorized changes replace "
                   "observation/reference");
    __ESBMC_assert(valid_model(model) && public_view(model) && model.lookup_count() >= before.lookup_count() &&
                       model.reference_id() >= before.reference_id() && same_sample(sample, old_sample),
                   "PX4 magnetic model: initialized clocks/source domains are inductive; counters never wrap or "
                   "fabricate a model");
}
int main() { return 0; }
