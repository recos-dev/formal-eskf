# R-TILT-ALIGNMENT: Covariance-based tilt alignment latch

**ID:** `R-TILT-ALIGNMENT`

**Status:** Draft

**Applies to:** Portable `runtime::TiltAlignment` for INS and AHRS, independent
of sensor fusion enablement, clocks and platform scheduling.

**Uses:** [E-STATE](../core/state.md),
[Q-ROT-MATRIX](../math/quaternion/rotation-matrix.md),
[F-LINALG](../math/linalg/operations.md), [C-FAILURE](../common/failure.md),
[C-NUMERICAL-ACCEPTANCE](../common/numerical-acceptance.md).

## Basis

This shared producer owns covariance-based alignment history. The stateless
[gravity condition predicate](gravity/conditions.md) consumes a caller-supplied
alignment flag; it does not calculate alignment. PX4 and replay callers retain
their own scheduling and estimator-session boundaries.

The project uses the body-to-navigation rotation and local attitude-error
covariance defined by E-STATE. A small estimated tilt variance is an alignment
milestone, not proof of actual attitude accuracy or a continuously valid health
flag. The latch deliberately records that milestone across later variance growth.

## Required behavior

### Initialization and observations

A default-constructed object shall have `aligned() == false` and `variance() == 0`.
The zero diagnostic alone shall not acquire alignment. Both accessors shall be
read-only. Default-constructing a fresh object starts a new alignment session; there is
no separate reset operation. The caller owns when a new session is required.

### Update and caller premises

`update(q_nb, P_theta_b)` consumes the attitude and its complete 3x3 body-local
attitude-error covariance at the same estimator horizon. The caller shall
establish unit attitude and symmetric positive-semidefinite covariance for the
physical interpretation. The operation checks finiteness but does not normalize
the quaternion or check covariance symmetry/PSD; its success does not discharge
these premises. INS callers extract the attitude-error block; AHRS callers use
their attitude covariance. Both inputs shall remain unchanged.

With `R_nb = R(q_nb)`, the exact-real reference is:

~~~text
P_theta_n = (R_nb * P_theta_b) * transpose(R_nb)
v_tilt   = P_theta_n(0,0) + P_theta_n(1,1)
~~~

The diagnostic is the navigation North/East rotation-error variance sum, in
rad². The full covariance, including cross terms, shall contribute to the
transformation. It is not the body-frame X/Y diagonal sum at a general attitude.
The implementation shall use the selected scalar/backend arithmetic for these
operations, with the left-associated matrix products shown above.

### Validation and state transition

The following rows are ordered; each later row applies only if earlier rows did
not reject. They also define detected-fault behavior for inputs outside the
physical premises above; successful guard evaluation alone does not validate
those premises.

| Condition | Required response |
|---|---|
| Any quaternion or input covariance coefficient is nonfinite | Return `non_finite_input`. |
| Any transformed covariance coefficient or `v_tilt` is nonfinite | Return `non_finite_result`. |
| Transformed North or East diagonal is negative | Return `domain_error`. |
| Otherwise | Return `success`, store `variance = v_tilt`, and update the latch as below. |

On success, `aligned = previous_aligned OR (v_tilt < tilt_limit²)`.
`tilt_limit` is the fixed three-degree limit in radians, encoded as the scalar
format's `std::numbers::pi_v<value_type> / 60`; its square shall be evaluated in
that scalar format. Equality shall not acquire alignment. Successful later
updates shall refresh the diagnostic even when the latch is already true;
variance growth shall not clear it. Signed zero is not a negative diagonal.

Every failure shall preserve both stored fields exactly. No failed update shall
publish a diagnostic, acquire or clear alignment. The operation shall not modify
the estimator state/covariance, perform a correction, infer rest/horizontal
aiding, or own a timer. There is no additional clamping or repair of negative
computed variances.

## Acceptance and verification

- Check fresh construction, pure accessors, identity and tilted covariance
  projection with nonzero cross terms, strict threshold equality/neighbors,
  latch persistence, reconstruction, every failure category and continuation
  after failure in both scalar formats and selected backends.
- Verify initialization and inductive latch/diagnostic preservation separately
  from the numerical producers. A source proof using rotation/product summaries
  needs matching producer evidence; arbitrary producer failures must remain
  represented. Native fixtures do not prove all backend executions.
- Exact-real PSD projection supports nonnegativity and the trace relationship;
  it does not prove floating-point PSD preservation, a physical confidence level
  or the truth of the caller's covariance model. General numerical acceptance
  requires a bounded profile under C-NUMERICAL-ACCEPTANCE.
- Callers shall provide coherent attitude/covariance horizons and observe the
  returned status. Module/replay timing, failure policy and session lifetime
  are separate acceptance obligations.

The [alignment cases](../../verification/tests/runtime/storage-alignment.md#tilt-alignment)
retain their existing test IDs. Source/Lean/ESBMC references remain grouped under
the existing `R-ACCELEROMETER-CONDITIONS` traceability row, which references this
requirement explicitly; extracting this contract does not add a proof result.
