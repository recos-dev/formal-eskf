## Summary

This specification defines the scalar, IEEE 754, domain-error and failure behavior applied to the operations in [ESKF Linear Algebra Abstraction Layer](eskf-linear-algebra-abstraction-layer.md) and [SO(3) and Unit Quaternion Abstraction Layer](so3-and-unit-quaternion-abstraction-layer.md).

Those specifications own the exact mathematical behavior. This specification defines how finite-precision execution is bounded and how domain violations are reported. It does not select numerical tolerance values.

## Scalar models

The project distinguishes three scalar meanings:

| Scalar | Purpose |
|---|---|
| Mathematical real | Exact specifications and algebraic reasoning |
| `float` | IEEE-754 binary32 deployment and verification profile |
| `double` | IEEE-754 binary64 deployment and verification profile |

A proof over mathematical reals is not a proof of the corresponding `float` or `double` implementation.

## IEEE-754 profile

Each verified or released configuration must record:

~~~text
scalar format
rounding mode
subnormal/flush-to-zero behavior
FMA contraction behavior
compiler and target
compile flags
math-library implementation
~~~

The default numerical profile is:

~~~text
rounding mode: round to nearest, ties to even
fast-math transformations: disabled
FMA contraction: disabled unless explicitly modeled
NaN and infinity: invalid for consumed ESKF state and sensor input
~~~

Correctness must not depend on preserving subnormal values. Required input bounds
and numerical thresholds must remain safely inside the selected normal range.
This portability requirement remains an acceptance obligation; the existing
IEEE source-proof profiles preserve subnormals and do not establish it.
Flush-to-zero or denormals-are-zero is a different execution profile and cannot reuse those
claims without review. Normal-range divisor/threshold checks do not make all
intermediate arithmetic independent of subnormal behavior.

## Valid inputs

Public ESKF transitions accept only inputs satisfying their documented domains:

~~~text
all consumed state, covariance, sensor, and time values are finite
dt_min <= dt <= dt_max
required quaternion normalization candidates have norm >= minimum_quaternion_norm
every divisor magnitude >= its minimum safe magnitude
SPD solve inputs satisfy the configured symmetry and conditioning bounds
sensor values remain inside the configured physical ranges
physical frames, units and model validity satisfy the caller's declared assumptions
~~~

Distinguish mathematical caller premises from runtime checks. The core does not
validate physical sensor ranges globally. `solve_spd` checks finite inputs, exact
symmetry, factorization and finite output; it imposes no condition-number cutoff.
A numerical-error or deployment claim needs its own reviewed domain, conditioning
and residual bounds. Concrete bounds belong to the selected configuration and
must be present before the corresponding floating-point proof is claimed.
Conditioning-policy enforcement or matching caller evidence remains to be
established; this gap does not remove the required domain. Source-control/status
proofs do not supply these bounds or prove physical sensor validity.

Nominal prediction (INS/AHRS) and INS covariance prediction additionally check the prior's squared-norm residual against `quaternion_squared_norm_tolerance`. It must be finite and in `(0,1)`, with `abs(squared_norm(q_nb) - 1) <= tolerance`. Zero defaults require explicit configuration. This prior validation never repairs the input; it is distinct from normalization's safe-divisor threshold. AHRS covariance does not consume prior attitude or this tolerance. See the ordered checks in [nominal prediction requirements](requirements/core/prediction.md#e-pred-input-consumed-inputs-and-caller-premises).

## Operation failures

Numerical operations must report failure explicitly. Required failure categories are:

~~~text
NonFiniteInput
DomainError
OutOfRange
ZeroOrUnsafeDivisor
InvalidQuaternionNorm
NotPositiveDefinite
IllConditioned
NonFiniteResult
~~~

The C++ names are defined by `formal_eskf::Status`. Not every operation emits
every category: `ill_conditioned` exists in the enum, but the current SPD solve
does not perform a condition-number test. Distinct detected failures must not
collapse into an undocumented successful result.

Operations must not silently:

~~~text
replace an invalid quaternion with identity
reuse a previous value
clamp a value without an explicit clamping operation
return a zero matrix after solve failure
continue with NaN or infinity
~~~

## Domain behavior

~~~text
divide(a, b)
  requires abs(b) >= minimum_safe_divisor

sqrt(x)
  requires x >= 0

normalize(v)
  requires norm(v) >= minimum_safe_norm

Log_q(q)
  requires a valid unit quaternion and follows its principal-domain definition

solve_spd(S, B)
  requires S to satisfy the configured SPD and conditioning policy
~~~

Checked domain violations return failure. The unit-quaternion premise of the
mathematical Log map is not a claim that the C++ Log API measures a unit-norm
residual. Its actual coefficient/domain checks and the independent mathematical
premise must remain distinct. Likewise, matrix/scalar arithmetic operators are
not interchangeable with checked normalization or solve operations.

## Failure atomicity

A failed public ESKF transition leaves externally visible estimator state unchanged:

~~~text
before = [nominal_state, covariance]

result = predict_or_correct(...)

if result is failure:
  after = before
~~~

Low-level operations should return a result/status before their output is committed to estimator state.

Logging, retry, sensor rejection counters, and fallback behavior belong to the caller or platform adapter.

## Comparison and residual behavior

No single global epsilon is used.

Every approximate property must define:

~~~text
input domain
absolute tolerance
relative tolerance
scale used by the relative tolerance
units
scalar format
operation being checked
~~~

A general scalar comparison has the form:

~~~text
abs(actual - expected)
  <= absolute_tolerance
   + relative_tolerance * scale
~~~

Matrix and geometric properties use named residuals, for example:

~~~text
unit quaternion: abs(norm(q) - 1)
rotation:        max_abs(R * transpose(R) - I)
symmetry:        max_abs(P - transpose(P))
linear solve:    max_abs(S * X - B)
~~~

Exact mathematical equality and bounded IEEE-754 residuals must be reported as different claims.

## State-output behavior

On success, the required postconditions are:

~~~text
all nominal-state coefficients are finite
all covariance coefficients are finite
the quaternion satisfies its configured unit-norm residual
the covariance satisfies its configured symmetry residual
the operation-specific residual bounds hold
~~~

These are acceptance obligations, not a claim that all have been established.
Finite-result and covariance-symmetry checks apply at the documented operation
boundaries. Quaternion and operation-specific residual bounds are not checked
by every API; the matching numerical evidence remains required.
In particular, prediction checks its prior quaternion residual; normalization of
a candidate does not prove that every future step meets a deployment tolerance.
No new global tolerance or hidden repair is introduced.

On failure, no success postcondition may be inferred other than the specified
failure atomicity. Runtime history and platform effects have their own contracts.

Compiler options that assume NaN, infinity, signed-zero, or associativity behavior incompatible with this specification are not permitted in a conforming build.
