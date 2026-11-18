# C-NUMERICAL-PROFILE: Numerical execution profile

**ID:** `C-NUMERICAL-PROFILE`

**Applies to:** Numerical operations and their selected deployment/verification configurations.

## Basis

Shared finite-precision and failure policy; operation equations remain in their owning requirements.

## Required behavior

### Scalar models

The project distinguishes three scalar meanings:

| Scalar | Purpose |
|---|---|
| Mathematical real | Exact specifications and algebraic reasoning |
| `float` | IEEE-754 binary32 deployment and verification profile |
| `double` | IEEE-754 binary64 deployment and verification profile |

A proof over mathematical reals is not a proof of the corresponding `float` or `double` implementation.

### IEEE-754 profile

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

Use [input-domain](input-domain.md), [failure](failure.md) and
[numerical-acceptance](numerical-acceptance.md) contracts for execution claims.

## Acceptance and verification

Review the applicable domain, checked versus assumed conditions, and execution
profile for each claim. A source/status proof does not discharge missing numerical
or target evidence. Operation-specific requirements retain their stricter conditions.
