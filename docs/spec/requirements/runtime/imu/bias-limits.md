# R-BIAS-LIMITS: per-axis bias-value limits

**ID:** `R-BIAS-LIMITS`

**Applies to:** Portable runtime policy; integrations supply timestamps, metadata and deployment parameters.

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
- Replay fixes limits for the session and validates them against the zero bias
  initial state before publishing output. After each successful GNSS, gravity or
  magnetic correction group, apply limits before the next correction/prediction.
  Rejected/failed corrections do not invoke limiting. Successful core correction
  guarantees finite biases, so limiting with those fixed valid bounds cannot fail.
  Standalone numerical-core correction/injection APIs remain additive and unchanged.
- Preparation writes `bias_limits.csv` from logged `EKF2_ABL_LIM` and
  `EKF2_GYR_B_LIM`; absent values use EKF2 module defaults 0.4 m/s² and 0.15 rad/s,
  with each source recorded in metadata. Recorded changes through session end
  are rejected. CSV input without the file uses the same explicit defaults.
  Summary reports effective bounds/source and per-axis counts of corrections
  actually clipped (not time at the bound or evidence of sensor failure).

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

The module uses `px4::limit_imu_bias` with fixed bounds 0.4 m/s² and
0.15 rad/s; published sensor-bias limits use the same constants. It limits startup biases and each successful GNSS velocity/horizontal
position, GNSS/barometer height and magnetic correction group before the next
operation. Rejections and resets do not invoke limiting. Each height source is
handled separately, rather than waiting until the entire height update returns.
Native adapter tests exercise accepted and rejected sources with inhibited
learning; the existing limiter tests cover boundaries, numerical failures and
inward recovery. The gravity adapter applies the same limiter after successful gravity correction.
