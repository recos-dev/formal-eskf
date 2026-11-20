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
#include <numbers>
#include <formal_eskf/runtime/magnetic_reference.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{
using formal_eskf::Status;
using Context = formal_eskf::test::Context;

template <typename Backend>
bool equal_reference(typename formal_eskf::runtime::MagneticReference<Backend>::State const & a,
                     typename formal_eskf::runtime::MagneticReference<Backend>::State const & b)
{
    using Scalar = typename Backend::value_type;
    using Bits = std::array<std::byte, sizeof(Scalar)>;
    bool equal = a.reference_id == b.reference_id && a.last_check_time_us == b.last_check_time_us &&
                 std::bit_cast<Bits>(a.model.declination) == std::bit_cast<Bits>(b.model.declination) &&
                 std::bit_cast<Bits>(a.model.inclination) == std::bit_cast<Bits>(b.model.inclination) &&
                 std::bit_cast<Bits>(a.model.strength) == std::bit_cast<Bits>(b.model.strength);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        equal = equal && std::bit_cast<Bits>(a.field_n(axis)) == std::bit_cast<Bits>(b.field_n(axis));
    }
    return equal;
}

template <typename Backend> void geometry(Context & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    using Model = formal_eskf::runtime::MagneticReference<Backend>;
    Model model;
    typename Model::State const empty{};
    test.expect(equal_reference<Backend>(model.state(), empty), profile,
                "initial model, vector, revision and check clock are empty");
    Scalar const pi = std::numbers::pi_v<Scalar>;
    for (auto D : {Scalar{0}, -pi / Scalar{2}, pi / Scalar{2}, -pi, pi, Scalar{0.25}})
    {
        for (auto I : {Scalar{0}, -pi / Scalar{2}, pi / Scalar{2}, Scalar{-0.375}})
        {
            model.reset();
            typename Model::Sample const sample{D, I, Scalar{60}};
            auto const result = model.update(1, sample, true);
            test.expect(result.status == Status::success && result.checked && result.changed &&
                            model.state().reference_id == 1 && model.state().last_check_time_us == 1,
                        profile, "first adopted model initializes a revision independently of origin event");
            long double const d = D, i = I;
            std::array<long double, 3> const expected{60 * std::cos(i) * std::cos(d), 60 * std::cos(i) * std::sin(d),
                                                      60 * std::sin(i)};
            // Fixed-domain trig regression against a wider reference, not a universal libm accuracy certificate.
            long double const bound = 32 * static_cast<long double>(std::numeric_limits<Scalar>::epsilon()) * 60;
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                test.expect(std::abs(static_cast<long double>(model.state().field_n(axis)) - expected[axis]) <= bound,
                            profile,
                            "independent NED conversion keeps East declination, Down inclination and microtesla scale");
            }
            auto const snapshot = model.state();
            test.expect(equal_reference<Backend>(model.state(), snapshot), profile, "state access is pure");
        }
    }
    model.reset();
    test.expect(equal_reference<Backend>(model.state(), empty) &&
                    model.update(1, {Scalar{0}, Scalar{0}, Scalar{20}}, true).changed,
                profile, "reset clears both public state and private update watermark");
    test.expect(model.state().field_n(0U) == Scalar{20} && model.state().field_n(1U) == Scalar{0} &&
                    model.state().field_n(2U) == Scalar{0},
                profile, "North-axis geometry has an exact scalar oracle");
}

template <typename Backend> void schedule(Context & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    using Model = formal_eskf::runtime::MagneticReference<Backend>;
    Model model;
    typename Model::Sample invalid{Scalar{0}, Scalar{0}, std::numeric_limits<Scalar>::quiet_NaN()};
    typename Model::Sample const valid{Scalar{0}, Scalar{0}, Scalar{50}};
    auto result = model.update(1, invalid, false);
    test.expect(result.status == Status::success && !result.checked && !result.changed &&
                    model.state().reference_id == 0,
                profile, "no position/origin consumes time without validating unused model values");
    test.expect(model.update(1, valid, false, true).status == Status::out_of_range, profile,
                "successful skip consumes update watermark");
    result = model.update(2, valid, false, true);
    test.expect(result.checked && result.changed && model.state().reference_id == 1, profile,
                "origin bypasses position-valid prerequisite");
    auto const adopted = model.state();
    for (auto now : {10000001ULL, 10000002ULL})
    {
        result = model.update(now, invalid, true);
        test.expect(result.status == Status::success && !result.checked &&
                        equal_reference<Backend>(model.state(), adopted),
                    profile, "ten-second minus-one/equality do not check or adopt invalid skipped model");
    }
    result = model.update(10000003, valid, true);
    test.expect(result.status == Status::success && result.checked && !result.changed &&
                    model.state().last_check_time_us == 10000003 && model.state().reference_id == 1,
                profile, "strict plus-one periodic check updates only check clock for unchanged model");
    result = model.update(10000004, model.state().model, false, true);
    test.expect(result.status == Status::success && result.checked && !result.changed &&
                    model.state().last_check_time_us == 10000004,
                profile, "origin can immediately check aliased adopted sample");
    result = model.update(30000005, invalid, false);
    test.expect(result.status == Status::success && !result.checked && model.state().last_check_time_us == 10000004,
                profile, "position loss leaves adopted reference and check clock intact");
}

template <typename Backend> void change_thresholds(Context & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    using Model = formal_eskf::runtime::MagneticReference<Backend>;
    Scalar const pi = std::numbers::pi_v<Scalar>;
    Scalar const degree = pi / Scalar{180};
    for (std::size_t field = 0; field < 3; ++field)
    {
        for (int sign : {-1, 1})
        {
            for (int side : {-1, 0, 1})
            {
                Model model;
                typename Model::Sample sample{Scalar{0}, Scalar{0}, Scalar{50}};
                test.expect(model.update(1, sample, true).changed, profile, "threshold baseline adopted");
                Scalar const threshold = static_cast<Scalar>(sign) * (field == 2 ? Scalar{1} : degree);
                Scalar change = threshold;
                if (side != 0)
                {
                    change = std::nextafter(threshold, side < 0 ? Scalar{0}
                                                                : static_cast<Scalar>(sign) *
                                                                      std::numeric_limits<Scalar>::infinity());
                }
                if (field == 0)
                {
                    sample.declination = change;
                }
                else if (field == 1)
                {
                    sample.inclination = change;
                }
                else
                {
                    // Select neighbours of the represented final B, not neighbours
                    // of1 that would disappear when added to50.
                    sample.strength = Scalar{50} + threshold;
                    if (side != 0)
                    {
                        sample.strength = std::nextafter(
                            sample.strength,
                            side < 0 ? Scalar{50} : (sign > 0 ? std::numeric_limits<Scalar>::infinity() : Scalar{0}));
                    }
                }
                auto const result = model.update(2, sample, false, true);
                test.expect(result.status == Status::success && result.checked && result.changed == (side > 0) &&
                                model.state().reference_id == (side > 0 ? 2U : 1U) &&
                                model.state().last_check_time_us == 2,
                            profile, "strict positive/negative change threshold and represented neighbours");
            }
        }
    }
    Model model;
    typename Model::Sample sample{Scalar{0}, Scalar{0}, Scalar{50}};
    test.expect(model.update(1, sample, true).changed, profile, "cumulative change baseline");
    for (auto strength : {Scalar{50.5}, Scalar{51}, Scalar{51.5}})
    {
        sample.strength = strength;
        auto const result =
            model.update(static_cast<std::uint64_t>((strength - Scalar{50}) * Scalar{2}) + 1, sample, false, true);
        test.expect(result.checked && result.changed == (strength > Scalar{51}), profile,
                    "subthreshold changes accumulate relative to adopted model rather than previous lookup");
    }
    sample = {pi, Scalar{0}, Scalar{40}};
    test.expect(model.update(5, sample, false, true).changed, profile, "positive declination endpoint accepted");
    sample.declination = -pi;
    test.expect(model.update(6, sample, false, true).changed && model.state().reference_id == 4, profile,
                "direct angle difference across pi cut advances revision even for equivalent physical heading");
}

template <typename Backend> void failure_frames(Context & test, std::string_view profile)
{
    using Scalar = typename Backend::value_type;
    using Model = formal_eskf::runtime::MagneticReference<Backend>;
    Scalar const pi = std::numbers::pi_v<Scalar>;
    typename Model::Sample const valid{Scalar{0.25}, Scalar{-0.125}, Scalar{50}};
    Model baseline;
    test.expect(baseline.update(100, valid, true).changed, profile, "seed public state and private clock");
    for (std::size_t field = 0; field < 3; ++field)
    {
        for (auto invalid : {std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity(),
                             -std::numeric_limits<Scalar>::infinity()})
        {
            auto model = baseline;
            auto const before = model.state();
            auto sample = valid;
            std::array<Scalar *, 3> fields{&sample.declination, &sample.inclination, &sample.strength};
            *fields[field] = invalid;
            auto const result = model.update(101, sample, true, true);
            test.expect(result.status == Status::non_finite_input && result.checked && !result.changed &&
                            equal_reference<Backend>(model.state(), before),
                        profile, "each nonfinite model coefficient preserves entire adopted state");
            test.expect(model.update(101, valid, true, true).status == Status::success, profile,
                        "failed numerical update does not consume timestamp");
        }
    }
    for (auto sample :
         {typename Model::Sample{std::nextafter(pi, Scalar{4}), Scalar{0}, Scalar{50}},
          typename Model::Sample{-std::nextafter(pi, Scalar{4}), Scalar{0}, Scalar{50}},
          typename Model::Sample{Scalar{0}, std::nextafter(pi / Scalar{2}, Scalar{2}), Scalar{50}},
          typename Model::Sample{Scalar{0}, -std::nextafter(pi / Scalar{2}, Scalar{2}), Scalar{50}},
          typename Model::Sample{Scalar{0}, Scalar{0}, Scalar{0}},
          typename Model::Sample{Scalar{0}, Scalar{0}, Scalar{-1}},
          typename Model::Sample{Scalar{0}, Scalar{0}, std::nextafter(std::numeric_limits<Scalar>::min(), Scalar{0})}})
    {
        auto model = baseline;
        auto const before = model.state();
        auto const result = model.update(101, sample, true, true);
        test.expect(result.status == Status::domain_error && result.checked && !result.changed &&
                        equal_reference<Backend>(model.state(), before) &&
                        model.update(101, valid, true, true).status == Status::success,
                    profile, "angle domain and positive-normal strength reject before publication and allow retry");
    }
    for (auto time : {0ULL, 99ULL, 100ULL})
    {
        auto model = baseline;
        auto const before = model.state();
        auto const result = model.update(time, valid, true, true);
        test.expect(result.status == Status::out_of_range && !result.checked && !result.changed &&
                        equal_reference<Backend>(model.state(), before) &&
                        model.update(101, valid, true, true).status == Status::success,
                    profile, "zero/backward/duplicate clock preserves both watermarks");
    }
    for (auto strength : {std::numeric_limits<Scalar>::min(), std::numeric_limits<Scalar>::max()})
    {
        Model model;
        test.expect(model.update(1, {Scalar{0}, Scalar{0}, strength}, true).status == Status::success &&
                        model.state().field_n(0U) == strength,
                    profile, "finite positive-normal endpoint strengths accepted on exact North axis");
    }
    Model translated;
    auto const end = std::numeric_limits<std::uint64_t>::max();
    test.expect(translated.update(end - 10000002U, valid, true).changed, profile, "near-maximum timeline seed");
    test.expect(!translated.update(end - 2U, valid, true).checked && translated.update(end - 1U, valid, true).checked,
                profile, "strict elapsed interval is translation-invariant without overflow-prone deadline addition");
    auto bad = valid;
    bad.strength = Scalar{-1};
    test.expect(translated.update(end, bad, true, true).status == Status::domain_error &&
                    translated.update(end, valid, true, true).status == Status::success,
                profile, "last representable timestamp permits failed-check retry");
    // Checked trig-provider failures/finite-arithmetic overflow are formal scalar-boundary
    // obligations; bounded StandardMath input here supplies no native failure witness.
}

// TC-R-MAGNETIC-REFERENCE-05 is deliberately NOT a native PASS selector.
// Revision exhaustion: formal/cpp/esbmc/magnetic_reference.cpp
// verify_reference_adoption_frame, verify_reference_coefficients, verify_reference_update.
// The current source/profile must be reviewed/run by the formal evidence pipeline;
// this suite neither fabricates private revision state nor executes 2^32 adoptions.
} /* end namespace */
int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Case;
    using Float = formal_eskf::test::Backend<float>;
    using Double = formal_eskf::test::Backend<double>;
    Case const cases[] = {
        {"TC-R-MAGNETIC-REFERENCE-01", "float", geometry<Float>},
        {"TC-R-MAGNETIC-REFERENCE-01", "double", geometry<Double>},
        {"TC-R-MAGNETIC-REFERENCE-02", "float", schedule<Float>},
        {"TC-R-MAGNETIC-REFERENCE-02", "double", schedule<Double>},
        {"TC-R-MAGNETIC-REFERENCE-03", "float", change_thresholds<Float>},
        {"TC-R-MAGNETIC-REFERENCE-03", "double", change_thresholds<Double>},
        {"TC-R-MAGNETIC-REFERENCE-04", "float", failure_frames<Float>},
        {"TC-R-MAGNETIC-REFERENCE-04", "double", failure_frames<Double>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
