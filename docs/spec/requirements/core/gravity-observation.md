# E-OBS-ACCEL: gravity observation with fixed bias preprocessing

**ID:** `E-OBS-ACCEL`

**Applies to:** INS/AHRS gravity observation; binary32/binary64.

**Parent:** [E-OBS](observations.md).

## Required behavior

INS and AHRS use the gravity observation only; there is no model-selection macro. INS uses a 3×15 local-error Jacobian and subtracts its prior accelerometer bias from raw input. AHRS uses a 3×3 Jacobian with externally bias-calibrated input.

For INS, use raw body-frame specific force, fixed navigation-frame gravity and the prior bias estimate:

\[
z=f_m-\hat b_a,\qquad h=-R(q_{nb})^Tg_n,\qquad r=z-h,
\]
\[
H=\begin{bmatrix}0_{3\times3}&0_{3\times3}&[h]_\times&0_{3\times3}&0_{3\times3}\end{bmatrix}.
\]

The columns are `[delta_p_n, delta_v_n, delta_theta_b, delta_b_a, delta_b_g]`, using the existing right-multiplicative attitude error. Construct this 3×15 Jacobian directly; the nominal 3×16 Jacobian is for manual derivation review only.

- Assume negligible navigation-frame acceleration `dot(v_n)`, not necessarily zero velocity. Keep gravity magnitude and measurement units; do not normalize either vector.
- Subtract the prior accelerometer bias exactly once. Hold this preprocessing fixed in the linearization: both bias Jacobian blocks are zero. This is an explicit approximation, not the joint derivative of `-R^T*g_n+b_a`. Zero bias columns do not freeze learning through cross-covariance; forward the existing optional bias-update permissions unchanged.
- Neither accelerometer API takes angular rate. The complete INS state must still be finite for correction/injection. Reject non-finite bias subtraction, prediction or residual arithmetic as `non_finite_result`; retain existing checked-correction statuses, alias support and atomic output publication.
- Use caller-supplied effective measurement covariance in raw specific-force units. Motion gating, model error and prior/measurement correlation remain caller responsibilities; no gating, differencing or correlated-noise update is introduced here.


## Acceptance and verification

Verify the stated residual and local Jacobian, including zero bias columns,
fixed prior preprocessing and full measurement units. Gravity must be finite and
nonzero; non-finite consumed data is `non_finite_input` and zero gravity is
`domain_error`. A zero measured force is algebraically permitted and does not
certify the motion assumption. Standalone Jacobian evaluation is unchecked.
Check wrapper forwarding, finite arithmetic, failures and all whole-object aliases
under [E-CORRECT](correction.md). Gravity alone does not observe heading. Reusing
prediction IMU samples or subtracting an uncertain prior bias can correlate the
observation with prior error; an effective V alone does not remove this model
limitation or establish statistical consistency.
