/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "step_correction_support.hpp"
#include <formal_eskf/runtime/magnetic_field.hpp>
using Math = solve_proof::Math;
using Scalar = solve_proof::Scalar;
using Backend = solve_proof::Backend;
using Vector = step_correction_proof::Vector3;
using Geometry = formal_eskf::runtime::detail::MagneticFieldGeometry<Scalar>;
using correction_proof::same;
using formal_eskf::Status;
void verify_general_root(Scalar value, Scalar root, Scalar output)
{
    Math::observe_pivots = false;
    Math::sqrt.calls = 0;
    Math::sqrt.results[0] = root;
    Scalar const before = output;
    Status const status = formal_eskf::scalar::sqrt<Math>(value, output);
    bool const eligible = std::isfinite(value) && value >= Scalar{0};
    Status const wanted = !std::isfinite(value)  ? Status::non_finite_input
                          : value < Scalar{0}    ? Status::domain_error
                          : !std::isfinite(root) ? Status::non_finite_result
                                                 : Status::success;
    __ESBMC_assert(status == wanted && Math::sqrt.calls == unsigned(eligible),
                   "general root: complete domain/status/call contract, including zero");
    __ESBMC_assert(!eligible || same(Math::sqrt.argument, value),
                   "general root: original argument reaches the raw boundary");
    __ESBMC_assert(same(output, status == Status::success ? root : before),
                   "general root: exact publication or atomic failure");
}
void verify_root_observer_equivalence(Scalar value, Scalar root, unsigned index)
{
    __ESBMC_assume(std::isfinite(value) && value > Scalar{0});
    __ESBMC_assume(index < solve_proof::size);
    Math::sqrt.results[index] = root;
    Math::sqrt.calls = index;
    Math::observe_pivots = true;
    auto const pivot = Math::sqrt(value);
    Math::sqrt.calls = index;
    Math::observe_pivots = false;
    auto const general = Math::sqrt(value);
    __ESBMC_assert(same(pivot, general) && same(general, root) && Math::sqrt.calls == index + 1U,
                   "passive pivot observer does not change any raw result");
}
void verify_general_angle(Scalar y, Scalar x, Scalar angle, Scalar output)
{
    Math::observe_pivots = false;
    Math::atan2.calls = 0;
    Math::atan2.results[0] = angle;
    Scalar const before = output;
    Status const status = formal_eskf::scalar::atan2<Math>(y, x, output);
    bool const eligible = std::isfinite(y) && std::isfinite(x) && (y != Scalar{0} || x != Scalar{0});
    Status const wanted = !std::isfinite(y) || !std::isfinite(x) ? Status::non_finite_input
                          : y == Scalar{0} && x == Scalar{0}     ? Status::domain_error
                          : !std::isfinite(angle)                ? Status::non_finite_result
                                                                 : Status::success;
    __ESBMC_assert(status == wanted && Math::atan2.calls == unsigned(eligible),
                   "general angle: complete domain/status/call contract");
    __ESBMC_assert(!eligible || (same(Math::atan2.y, y) && same(Math::atan2.x, x)),
                   "general angle: original y,x argument order reaches the raw boundary");
    __ESBMC_assert(same(output, status == Status::success ? angle : before),
                   "general angle: exact publication or atomic failure");
}
void verify_general_geometry(Vector field, Geometry output, Scalar strength, Scalar horizontal, Scalar angle)
{
    Math::observe_pivots = false;
    Math::sqrt.calls = Math::atan2.calls = 0;
    Math::sqrt.results[0] = strength;
    Math::sqrt.results[1] = horizontal;
    Math::atan2.results[0] = angle;
    auto const before = output;
    auto const old_field = field;
    auto const status = formal_eskf::runtime::detail::magnetic_field_geometry<Backend>(field, output);
    __ESBMC_assert(status != Status::success || (std::isfinite(output.strength) && std::isfinite(output.horizontal) &&
                                                 std::isfinite(output.inclination)),
                   "actual same-backend geometry: successful fields are finite");
    __ESBMC_assert(status == Status::success ||
                       (same(output.strength, before.strength) && same(output.horizontal, before.horizontal) &&
                        same(output.inclination, before.inclination)),
                   "actual same-backend geometry: atomic output frame");
    __ESBMC_assert(correction_proof::same_matrix(field, old_field),
                   "actual same-backend geometry: field input is unchanged");
}
void verify_general_heading(Vector a, Vector b, Scalar output, Scalar angle0, Scalar angle1)
{
    Math::observe_pivots = false;
    Math::atan2.calls = 0;
    Math::atan2.results[0] = angle0;
    Math::atan2.results[1] = angle1;
    auto const old_a = a, old_b = b;
    Scalar const before = output;
    auto const status = formal_eskf::runtime::detail::magnetic_heading_error<Backend>(a, b, output);
    __ESBMC_assert(status == Status::success || same(output, before),
                   "actual same-backend heading: atomic output frame");
    __ESBMC_assert(correction_proof::same_matrix(a, old_a) && correction_proof::same_matrix(b, old_b),
                   "actual same-backend heading: both inputs are unchanged");
}
int main() { return 0; }
