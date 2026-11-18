# C-NUMERICAL-ACCEPTANCE: Numerical residual acceptance

**ID:** `C-NUMERICAL-ACCEPTANCE`

**Applies to:** Numerical operations and their selected deployment/verification configurations.

## Basis

Shared finite-precision and failure policy; operation equations remain in their owning requirements.

## Required behavior

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

## Acceptance and verification

Review the applicable domain, checked versus assumed conditions, and execution
profile for each claim. A source/status proof does not discharge missing numerical
or target evidence. Operation-specific requirements retain their stricter conditions.
