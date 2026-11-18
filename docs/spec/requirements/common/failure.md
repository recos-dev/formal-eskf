# C-FAILURE: Failure reporting and atomicity

**ID:** `C-FAILURE`

**Applies to:** Numerical operations and their selected deployment/verification configurations.

## Basis

Shared finite-precision and failure policy; operation equations remain in their owning requirements.

## Required behavior

### Operation failures

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

### Failure atomicity

A failed public ESKF transition leaves externally visible estimator state unchanged:

~~~text
before = [nominal_state, covariance]

result = predict_or_correct(...)

if result is failure:
  after = before
~~~

Low-level operations should return a result/status before their output is committed to estimator state.

Logging, retry, sensor rejection counters, and fallback behavior belong to the caller or platform adapter.

## Acceptance and verification

Review the applicable domain, checked versus assumed conditions, and execution
profile for each claim. A source/status proof does not discharge missing numerical
or target evidence. Operation-specific requirements retain their stricter conditions.
