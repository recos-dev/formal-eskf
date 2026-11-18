# R-MAGNETOMETER-RESET: fixed-field heading and covariance reset

**ID:** `R-MAGNETOMETER-RESET`

**Status:** Draft

**Applies to:** Portable magnetic runtime policy; caller bindings are specified separately.

**Uses:** [E-STATE](../../core/state.md), [R-MAGNETOMETER-FILTER](reset-filter.md).

## Required behavior

`runtime::reset_magnetic_heading` supports AHRS and INS independently of the
lifecycle. Given a calibrated body field `m_b`, trusted fixed NED `m_n` and an
aligned prior tilt, rotate only about navigation Down to align the horizontal
direction of `R_before*m_b` with `m_n`. Fields are in microtesla. Wrap the heading
difference to `[-pi, pi]`, form `q_delta=Exp([0,0,delta_heading])`, and normalize
`q_after=q_delta*q_before`. Return this applied left rotation, retaining all
other nominal fields. This does not require an Euler-angle parameterization.

The covariance operation is defined in navigation attitude-error coordinates,
then expressed in our local right-error convention. With `e_D=[0,0,1]^T`:

```text
C = I - e_D e_D^T
A = R_after^T C R_before
u = R_after^T e_D
P_theta_theta' = A P_theta_theta A^T + max(yaw_variance, 0.01^2) u u^T
P_theta_other' = A P_theta_other
```

Retain the non-attitude P block exactly and mirror the cross block. Symmetrize
the computed attitude block with `a_ij / 2 + a_ji / 2`, retaining the established
overflow/rounding behavior. `yaw_variance` is rad²; zero receives the explicit
floor. The transform removes navigation yaw correlations while retaining
navigation tilt/cross terms; simply clearing body-Z rows is wrong at general tilt.

Inputs must be finite, prior quaternion unit and P symmetric PSD. Check exact
symmetry, nonnegative diagonals/variance, normal positive horizontal-field
threshold and `0 < minimum_quaternion_norm <= 1`. Check both horizontal field
magnitudes against that threshold and reject non-finite intermediate arithmetic.
A negative computed attitude variance returns `not_positive_definite`.
PSD/prior norm, physical field validity, alignment, freshness and authorization
remain caller premises. Outputs publish together only on success. State/P may
independently alias their entire priors; other storage, including delta, is disjoint.

The reference is PX4 v1.16.2
[heading reset](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp#L564)
and [yaw covariance reset](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/yaw_fusion.cpp#L118).
PX4's global left-error definition is explicit in
[its derivation](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/python/ekf_derivation/derivation.py#L152).
The primitive consumes the caller-supplied body field. Replay supplies the
reset LPF after current field qualification, as PX4 does; PX4 additionally resets
estimated magnetic states. Native AHRS/INS float/double and replay
tests cover this narrower operation. Lean proves exact Down-axis Exp,
normalized unit-attitude composition, tilt preservation, horizontal alignment
under the stated polar-field premise, wrap invariance, yaw-axis removal, and
covariance block equations/symmetry/PSD through congruence plus rank-one noise.
The block model places attitude first; INS uses its explicit production offset.
These real statements do not establish C++ execution, IEEE PSD, libm accuracy or
physical field validity. ESBMC checks AHRS and INS covariance reset in both
scalar formats with separate/in-place outputs. The INS query uses independently
proved same-type 3x3 and 3x15 product contracts; actual operand routing, yaw floor,
ordered remaining IEEE operations, finite/diagonal checks, symmetrization and all
225 covariance cells remain checked. Composer checks cover input validation,
every child failure, wrapped reference-minus-measured angle, left composition and
old/new covariance frames. Public AHRS/INS APIs prove simultaneous publication
and complete rollback for all four state/P aliases, even with partially written
helper scratch. INS position/velocity/bias values are checked before composition
and retained exactly. Floating-point accuracy/PSD bounds, external backends and
adapter sequencing remain separate obligations. The
[verification guide](../../../../guides/verification.md) describes evidence scope and retention;
execution status must be checked against the claimed source and configuration.

## Caller bindings

- PX4 module magnetic aiding: [R-PX4-MAGNETIC-AIDING](../../integration/px4/magnetic-aiding.md).
- PX4 module geographic magnetic reference: [R-PX4-MAGNETIC-MODEL](../../integration/px4/magnetic-model.md).
