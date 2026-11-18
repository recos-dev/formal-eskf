# R-BIAS-LIMITS: per-axis bias-value limits

**ID:** `R-BIAS-LIMITS`

**Status:** Draft

**Applies to:** Portable runtime policy; integrations supply timestamps, metadata and deployment parameters.

**Parent:** [SW-IMU](../../software/imu-protection.md).

**Uses:** [E-STATE](../../core/state.md).

## Required behavior

`runtime::limit_bias(state, limits)` explicitly limits each body-axis INS
accelerometer bias to `[-limits.accelerometer, +limits.accelerometer]` (m/s²)
and gyro bias to `[-limits.gyroscope, +limits.gyroscope]` (rad/s).

- Bounds are inclusive and per coefficient, not a vector-norm constraint.
  Zero is a valid limit that forces zero bias; in-range coefficients are preserved.
  Validate both limits and all six bias coefficients before writing. Non-finite
  inputs return `non_finite_input`; otherwise negative limits return `domain_error`.
  Failure preserves the entire state. Success changes only the six bias fields.
- This operation does not alter covariance, reject a measurement, reset a bias,
  or change learning permissions. Limiting applies even to an inhibited axis if
  its prior value is already outside the bound. With fixed limits and an in-range
  prior, inhibition still holds the estimated bias. An inward correction can
  leave saturation immediately; there is no latch or recovery timer.

Behavioral reference: PX4 v1.16.2
[bias correction](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L757).
PX4 limits after each scalar fusion; replay and module limit after each existing measurement
group. The axis-bound rule agrees, but sequential saturation trajectories need
not agree. Clamping does not prove posterior covariance consistency or physical
bias accuracy. Native boundary, failure, correction and replay tests cover this
feature. The `imu-policies` ESBMC suite additionally verifies the actual limiter
in binary32/binary64: inclusive bounds, preservation of in-range representations,
complete validation before writes, unchanged non-bias fields and whole-state
rollback on failure. Non-bias fields remain arbitrary, including nonfinite values.
This is a direct source-level proof; no new Lean algebra is needed for these
comparison/selection properties. Replay/module invocation, fixed-limit loading
and covariance consistency are not discharged by the standalone limiter proof.

## Caller bindings

- [R-REPLAY-BIAS](../../integration/replay/imu-bias.md).
- [R-PX4-IMU-BIAS](../../integration/px4/imu-bias.md).
