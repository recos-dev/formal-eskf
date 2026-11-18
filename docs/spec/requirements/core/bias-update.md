# E-BIAS-UPDATE: INS bias-update control

**ID:** `E-BIAS-UPDATE`

**Status:** Draft

**Applies to:** Shared core correction policy; configurations and limits specified below.

**Parent:** [SW-CORE](../software/core-estimation.md).

## Required behavior

INS `correct` accepts an optional trailing `Ins::BiasUpdate` with three body-axis permissions each for accelerometer and gyroscope biases. Every permission defaults to `true`; omitting the argument retains unrestricted correction. AHRS has no bias states and does not accept this control.

INS horizontal/vertical position, horizontal/3D velocity, magnetometer and accelerometer correction APIs accept the same optional trailing permissions and forward them unchanged. Magnetic tilt protection is selected separately by the build-wide `ESKF_MAG_TILT` macro. Their observation models and Jacobians remain unchanged.

- Compute the same innovation covariance and unconstrained gain as `E-CORRECT`. After a successful checked solve, set every entry of each disabled bias gain row to zero. Error-state rows 9..11 are accelerometer bias and 12..14 are gyroscope bias; position, velocity and attitude rows are untouched.
- Use the resulting `K_eff` for both `delta_x = K_eff r` and the full Joseph update `(I-K_eff H) P (I-K_eff H)^T + K_eff V K_eff^T`, then perform the existing injection/reset. Keep the same prior P and full correlated V; do not alter H or erase covariance rows/columns.
- A disabled bias receives zero correction, retaining its numerical estimate. Its uncertainty is not set to zero; cross-covariances may change. This is per-call correction control, not a freeze of prediction's bias random walk or a persistent health state.
- Disabling bias updates does not bypass input checks or solve failures. Preserve the existing validation order, statuses, whole-state/covariance alias support and atomic publication on success; all failures preserve both outputs.
- All-enabled permissions must agree with unrestricted correction. Test all 64 permission combinations against an independent constrained-gain/Joseph/injection/reset oracle, including nonzero cross-covariance, correlated measurement noise and both reset modes.

The caller owns the learning policy. Runtime policy wiring, IMU fault detection, recovery timing and adaptive process noise are separate work. Lean proves the exact constrained-gain algebra, disabled-axis estimate/prior-variance retention and the existing correction invariants. The `bias` ESBMC suite checks the actual row constraints, controlled INS correction transactions and six sensor wrappers with all six permissions symbolic, both scalar formats and all state/covariance aliases. It reuses the same-backend `E-CORRECT` and injection/reset producer obligations; it does not prove IEEE positive semidefiniteness. Magnetic wrappers in this suite select `ESKF_MAG_TILT=0`; the `magnetic` suite separately covers the projected M1 path with the same six symbolic permissions. Native tests retain the all-64-mask numerical/Joseph regression. See the [verification scope and evidence rules](../../../guides/verification.md).
