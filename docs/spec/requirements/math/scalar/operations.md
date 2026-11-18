# F-SCALAR: Checked scalar operations

**ID:** `F-SCALAR`

**Status:** Draft

**Applies to:** Binary32/binary64 scalar frontends and their selected math backends.

**Uses:** [C-NUMERICAL-PROFILE](../../common/numerical-profile.md), [C-INPUT-DOMAIN](../../common/input-domain.md), [C-FAILURE](../../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../../common/numerical-acceptance.md).

## Basis

The project's [checked scalar API](../../../../../include/formal_eskf/scalar/math.hpp)
separates elementary mathematical operations from status/publication control.
Use the [numerical profile](../../common/numerical-profile.md),
[domain](../../common/input-domain.md), [failure](../../common/failure.md) and
[residual](../../common/numerical-acceptance.md) contracts.

## Required behavior

The scalar boundary supplies `pi`, `is_finite`, `absolute`, checked `sqrt`,
checked `sin_cos` and checked `atan2(y, x)`. Angles are radians. The math backend
owns elementary evaluation; it shall not enter ESKF interfaces directly.

| Operation | Required checked behavior |
|---|---|
| `sqrt(input, output)` | Reject non-finite input, then negative finite input. Evaluate the nonnegative square root and require a finite result before publication. |
| `sin_cos(angle, output)` | Reject a non-finite angle. Evaluate both sine and cosine; publish the pair only when both are finite. |
| `atan2(y, x, output)` | Reject either non-finite input, then the origin where both coordinates compare equal to zero. Evaluate the quadrant-aware angle using this argument order; require a finite result. |

Invalid consumed values return `non_finite_input`, a checked domain violation
returns `domain_error`, and a non-finite backend result returns `non_finite_result`.
Failure preserves the complete output. Success publishes the backend result
unchanged; do not silently clamp, substitute or normalize it. Scalar inputs are
passed by value, so an output alias to a caller's input scalar does not invalidate
these rules. Positive and negative zero compare equal for domain checks.

`is_finite` rejects NaN and both infinities. `absolute` supplies scalar magnitude;
these inspection operations and `pi` do not use the checked-output protocol.
No global epsilon, argument reduction policy or deployed-libm error bound is
introduced by this contract. Principal-angle branch endpoints, signed zero and
rounding must be interpreted under the selected math-library profile.

## Acceptance and verification

Verify domain/error order, argument order, both sin/cos outputs and failure
preservation, including non-finite inputs and backend results. Exact real-function
properties, source control flow and native backend conformance are distinct.
Status safety under a backend summary is not a bound on deployed math accuracy.
