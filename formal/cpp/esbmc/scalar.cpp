/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "scalar_support.hpp"

using namespace scalar_proof;
namespace scalar = formal_eskf::scalar;

void verify_scalar_primitives(Scalar value)
{
    __ESBMC_assert(FORMAL_ESKF_PROOF_SCALAR_LIBM == 0,
                   "Runner error: scalar primitives require the actual library model");
    __ESBMC_assert(Math::is_finite(value) == scalar_proof::finite(value) &&
                       scalar::is_finite<Math>(value) == scalar_proof::finite(value),
                   "F-SCALAR: finite classification matches the IEEE exponent field");
    Bits const magnitude = bits(value) & ~sign_mask;
    auto const backend = Math::absolute(value);
    auto const facade = scalar::absolute<Math>(value);
    __ESBMC_assert(nan(value) ? nan(backend) && nan(facade) : bits(backend) == magnitude && bits(facade) == magnitude,
                   "F-SCALAR: absolute clears the sign, including negative zero and infinity");
    __ESBMC_assert(bits(Math::pi()) == pi_bits && bits(scalar::pi<Math>()) == pi_bits,
                   "F-SCALAR: pi uses the selected binary32 or binary64 rounded constant");
    Pair const initialized{};
    __ESBMC_assert(bits(initialized.sine) == 0U && bits(initialized.cosine) == 0U,
                   "F-SCALAR: default SinCos owns two initialized positive zeros");
}

void verify_scalar_backend(Scalar x, Scalar y, Scalar root, Scalar sine, Scalar cosine, Scalar angle)
{
    __ESBMC_assert(modeled_libm, "Runner error: scalar backend requires the observed library boundary");
    if constexpr (!modeled_libm)
        return;
    Libm::prepare(root, sine, cosine, angle);
    auto const actual_root = Math::sqrt(x);
    bool const calls_root = !nan(x) && !negative(x);
    __ESBMC_assert(Libm::root.calls == (calls_root ? 1U : 0U) &&
                       (calls_root ? same(Libm::root.argument, x) && same(actual_root, root) : nan(actual_root)),
                   "F-SCALAR: StandardMath sqrt forwards nonnegative inputs and rejects negative/NaN inputs");
    auto const actual_sine = Math::sin(x);
    auto const actual_cosine = Math::cos(x);
    auto const actual_angle = Math::atan2(y, x);
    __ESBMC_assert(same(actual_sine, sine) && same(actual_cosine, cosine) && same(actual_angle, angle),
                   "F-SCALAR: StandardMath forwards library results without clamping");
    __ESBMC_assert(Libm::sine.calls == 1U && same(Libm::sine.argument, x) && Libm::cosine.calls == 1U &&
                       same(Libm::cosine.argument, x) && Libm::angle.calls == 1U && same(Libm::angle.y, y) &&
                       same(Libm::angle.x, x) && Libm::count == (calls_root ? 4U : 3U),
                   "F-SCALAR: StandardMath calls the matching unary and y,x library interfaces");
}

void verify_scalar_sqrt(Scalar input, Scalar output, Scalar root)
{
    __ESBMC_assert(modeled_libm && alias <= 1U, "Runner error: scalar sqrt requires library models and a valid alias");
    if constexpr (!modeled_libm || alias > 1U)
        return;
    Scalar const argument = input;
    Scalar * destination = &output;
    if constexpr (alias == 1U)
        destination = &input;
    Scalar const before = *destination;
    Scalar const unused_output = output;
    Libm::prepare(root, Scalar{0}, Scalar{0}, Scalar{0});
    auto const actual = scalar::try_sqrt<Math>(input, *destination);
    auto const expected = sqrt_status(argument, root);
    bool const calls_root = scalar_proof::finite(argument) && !negative(argument);
    __ESBMC_assert(actual == expected && same(*destination, expected == Status::success ? root : before),
                   "F-SCALAR: checked StandardMath sqrt status and complete output publication/rollback");
    __ESBMC_assert(Libm::root.calls == (calls_root ? 1U : 0U) && Libm::count == Libm::root.calls &&
                       (!calls_root || same(Libm::root.argument, argument)),
                   "F-SCALAR: checked sqrt validates before forwarding its original input");
    __ESBMC_assert(alias == 0U ? same(input, argument) : same(output, unused_output),
                   "F-SCALAR: checked sqrt preserves objects not selected as output");
}

void verify_scalar_sin_cos(Scalar input, Pair output, Scalar sine, Scalar cosine)
{
    __ESBMC_assert(modeled_libm && alias <= 2U,
                   "Runner error: scalar sincos requires library models and a valid alias");
    if constexpr (!modeled_libm || alias > 2U)
        return;
    Scalar * source = &input;
    if constexpr (alias == 1U)
        source = &output.sine;
    if constexpr (alias == 2U)
        source = &output.cosine;
    Scalar const argument = *source;
    Scalar const unused_input = input;
    Pair const before = output;
    Pair const candidate{sine, cosine};
    Libm::prepare(Scalar{0}, sine, cosine, Scalar{0});
    auto const actual = scalar::try_sin_cos<Math>(*source, output);
    auto const expected = sin_cos_status(argument, candidate);
    bool const calls_pair = scalar_proof::finite(argument);
    __ESBMC_assert(actual == expected, "F-SCALAR: sincos validates the input and both library results in order");
    __ESBMC_assert(expected == Status::success ? same_pair(output, candidate) : same_pair(output, before),
                   "F-SCALAR: sin and cos publish together only when both results are finite");
    __ESBMC_assert(Libm::sine.calls == (calls_pair ? 1U : 0U) && Libm::cosine.calls == (calls_pair ? 1U : 0U) &&
                       Libm::count == (calls_pair ? 2U : 0U) &&
                       (!calls_pair || (same(Libm::sine.argument, argument) && same(Libm::cosine.argument, argument) &&
                                        Libm::sine.order == 0U && Libm::cosine.order == 1U)),
                   "F-SCALAR: sincos validates first and forwards the same original angle in order");
    __ESBMC_assert(same(input, unused_input), "F-SCALAR: sincos preserves its separate scalar object");
}

void verify_scalar_atan2(Scalar y, Scalar x, Scalar output, Scalar angle)
{
    __ESBMC_assert(modeled_libm && alias <= 3U, "Runner error: scalar atan2 requires library models and a valid alias");
    if constexpr (!modeled_libm || alias > 3U)
        return;
    Scalar * destination = &output;
    Scalar * x_source = &x;
    if constexpr (alias == 1U)
        destination = &y;
    if constexpr (alias == 2U)
        destination = &x;
    if constexpr (alias == 3U)
    {
        destination = &y;
        x_source = &y;
    }
    Scalar const old_y = y, old_x = x, old_output = output;
    Scalar const first = y, second = *x_source, before = *destination;
    Libm::prepare(Scalar{0}, Scalar{0}, Scalar{0}, angle);
    auto const actual = scalar::try_atan2<Math>(y, *x_source, *destination);
    auto const expected = atan2_status(first, second, angle);
    bool const calls_angle =
        scalar_proof::finite(first) && scalar_proof::finite(second) && !(zero(first) && zero(second));
    __ESBMC_assert(actual == expected && same(*destination, expected == Status::success ? angle : before),
                   "F-SCALAR: atan2 status and complete output publication/rollback");
    __ESBMC_assert(Libm::angle.calls == (calls_angle ? 1U : 0U) && Libm::count == Libm::angle.calls &&
                       (!calls_angle || (same(Libm::angle.y, first) && same(Libm::angle.x, second))),
                   "F-SCALAR: atan2 rejects every signed-zero origin and preserves original y,x order");
    __ESBMC_assert((destination == &y || same(y, old_y)) && (destination == &x || same(x, old_x)) &&
                       (destination == &output || same(output, old_output)),
                   "F-SCALAR: atan2 preserves every object not selected as output");
}

int main() { return 0; }
