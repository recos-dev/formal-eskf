# C-INPUT-DOMAIN: Input-domain and caller obligations

**ID:** `C-INPUT-DOMAIN`

**Applies to:** Numerical operations and their selected deployment/verification configurations.

## Basis

Shared finite-precision and failure policy; operation equations remain in their owning requirements.

## Required behavior

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

Nominal prediction (INS/AHRS) and INS covariance prediction additionally check the prior's squared-norm residual against `quaternion_squared_norm_tolerance`. It must be finite and in `(0,1)`, with `abs(squared_norm(q_nb) - 1) <= tolerance`. Zero defaults require explicit configuration. This prior validation never repairs the input; it is distinct from normalization's safe-divisor threshold. AHRS covariance does not consume prior attitude or this tolerance. See the ordered checks in [nominal prediction requirements](../core/prediction.md#e-pred-input-consumed-inputs-and-caller-premises).

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

## Acceptance and verification

Review the applicable domain, checked versus assumed conditions, and execution
profile for each claim. A source/status proof does not discharge missing numerical
or target evidence. Operation-specific requirements retain their stricter conditions.
