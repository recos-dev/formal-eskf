# R-PX4-STATUS: confidence and failure reporting

**ID:** `R-PX4-STATUS`

**Status:** Draft

**Applies to:** Single-owner PX4 INS module; active deployment profile and stated dependencies apply.

**Parent:** [SW-PX4](../../software/px4-estimation.md).

## Required behavior

The single-instance module maps its supported capabilities to the pinned PX4
v1.16.2 topics. A sensor being active, an estimate being usable, and a bias having
converged are different facts. Reporting must not change fusion decisions.

- Produce packed control/fault masks and named status flags from the same owner
  snapshot. Use the actual pinned `EKF/common.h` wire layout: vertical-acceleration
  and clipping faults are bits 10 and 11. The older `EstimatorStatus.msg` comment
  is not the wire definition. A generic numerical/history failure is not an
  accelerometer fault; it remains restart-required, stops valid output, clears
  usable solution/bias flags, and appears in module fault/error diagnostics.
- Resolve attitude, horizontal velocity/position, vertical velocity and absolute
  height capabilities independently. Current source generation, freshness and
  estimator health constrain validity. For PX4 controller compatibility local
  `z_valid` and `v_z_valid` both allow vertical velocity **or** height aiding;
  global `alt_valid` still requires height. Delayed source activity flags are not
  substitutes for current usable-solution flags. Rejection never advances a
  successful-fusion timestamp. Full source resets clear the applicable rejection
  and innovation history, without being counted as ordinary corrections. The
  Down-only velocity rescue retains that attempt's velocity diagnostics, as in
  pinned PX4.
- Report horizontal position/velocity uncertainty as `sqrt(P_NN + P_EE)` and
  vertical uncertainty as `sqrt(P_DD)`. Attitude reporting follows
  `R-PX4-FRAME-OUTPUT`. These remain delayed IMU-state covariance proxies beside
  current body-point output, without predictor covariance propagation,
  installation uncertainty, lever-arm correlations or PX4's extra dead-reckoning
  residual inflation. They are not calibrated accuracy bounds. Odometry quality
  remains zero (no percentage score).
- Bias validity requires a healthy estimator, a nonzero sensor ID, finite
  published bias/variance, nonnegative variance components and a variance-vector
  norm in `(0, 0.1]`, using the published float representation as PX4 does.
  Temporary learning inhibition does not invalidate a stored estimate. Keep
  `*_bias_stable` false: this module has no calibration-convergence tracker.
- Summarize each active source's **signed filtered squared** gate ratios. Convert
  our measurement-minus-prediction residual to PX4's opposite sign. At each new
  diagnostic measurement use `dt = clamp(delta_time, 0.001, 1)` seconds and
  `alpha = dt / (dt + 0.5)`, per axis. First/repeated/regressed timestamps seed
  the unsigned raw ratio; a non-finite previous value is reseeded. For advancing
  epochs, raw ratios above 4 limit the filtered result to `[-4,4]`. Missing or
  invalid diagnostics are NaN. Source/calibration changes discard the history;
  a successful full source reset zeros its history at the observation epoch
  (the Down-only velocity rescue retains the attempt's diagnostics).
  Publication alone must never run this filter. The module uses double
  diagnostics; identical arithmetic bits to PX4's float are not required.
- Published summary ratios are `sqrt(abs(filtered_ratio))`, maximizing over the
  relevant axes. Horizontal and vertical velocity preflight checks are separate;
  height averages the available active barometer/GNSS summaries. The preflight
  threshold is strictly greater than 0.5. The heading preflight boolean also
  vetoes missing tilt alignment or unavailable magnetic heading according to
  `OutputValidity::magnetic`, preserving its source/generation, field, consistency,
  age, variance and health checks. Keep `hdg_test_ratio` numeric/NaN independently.
  This is a conservative module-to-Commander compatibility rule: the pinned
  consumer does not inspect our tilt/yaw capability flags, while our GNSS policy
  lacks EKF2's tilt/yaw admission prerequisite. Do not remove this veto merely
  because NaN is not greater than the threshold. Source/calibration replacement
  must remain blocked until heading readiness actually returns. This change does
  not inhibit existing in-air GNSS fusion or introduce a new fusion prerequisite.
  Fusion still uses its existing raw componentwise gate and vertical-limiting rules. Rejection flags describe actual
  rejection, while raw GNSS gate exceedance supplies the solution glitch flag;
  a bounded accepted vertical correction must not be reported as rejected.
- Unsupported airspeed/HAGL/sideslip ratios, magnetic angle/strength telemetry
  and cumulative IMU time-slip diagnostics are NaN, not invented zero values.
  Zeroed legacy LPE health/timeout flags and false unsupported capability bits
  remain unused. Missing numeric diagnostics do not prove good health: consumers
  must also check solution capabilities, topic freshness and source status.
- Status change counters count changes between published snapshots (currently
  10 Hz). They are not a lossless event log and can miss transitions between
  snapshots. Do not claim PX4's on-change event accounting. Filtering is bounded,
  allocation-free and module-owned; no shared-core equation or public API changes.

Native evidence must cover independent capability combinations, actual failure
withdrawal/restart behavior, variance-norm boundaries, packed/named consistency,
raw rejection versus filtered warning, measurement-time filter/reset/source
semantics and duplicate publication. SIH must arm normally with the existing
preflight checks and exercise flight/fault recovery; retain failed runs. Native,
sanitizer and SIH evidence do not establish target timing, calibration accuracy,
formal source correctness or hardware-flight qualification.

Reference: pinned PX4 [topic publication](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp),
[summary/filter/solution semantics](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp),
[vector diagnostic filter](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf.h)
and [wire masks](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/common.h).
