## Summary

Define the scalar, IEEE 754, domain-error, and failure behavior applied to the operations in [ESKF Linear Algebra Abstraction Layer](https://github.com/shengwen-tw/formal-eskf/issues/2) and [SO(3) and Unit Quaternion Abstraction Layer](https://github.com/shengwen-tw/formal-eskf/issues/3).

Those issues own the exact mathematical behavior. This specification only defines how finite-precision execution is bounded and how domain violations are reported. It does not select numerical tolerance values.

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
NaN and infinity: invalid for ESKF state and sensor input
~~~

Correctness must not depend on preserving subnormal values. Required input bounds and numerical thresholds must remain safely inside the selected normal range.

## Valid inputs

Public ESKF transitions accept only inputs satisfying their documented domains:

~~~text
all state, covariance, sensor, and time values are finite
dt_min <= dt <= dt_max
quaternion norm >= minimum_quaternion_norm
every divisor magnitude >= its minimum safe magnitude
SPD solve inputs satisfy the configured symmetry and conditioning bounds
sensor values remain inside the configured physical ranges
~~~

Concrete bounds belong to the selected estimator configuration and must be present before the corresponding floating-point proof is claimed.

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

Equivalent names may be used, but distinct causes must not collapse into an undocumented successful result.

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

A domain violation returns failure rather than an arbitrary numeric value.

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

On success:

~~~text
all nominal-state coefficients are finite
all covariance coefficients are finite
the quaternion satisfies its configured unit-norm residual
the covariance satisfies its configured symmetry residual
the operation-specific residual bounds hold
~~~

On failure, no success postcondition may be inferred other than failure atomicity.

Compiler options that assume NaN, infinity, signed-zero, or associativity behavior incompatible with this specification are not permitted in a conforming build.
