# R-PX4-IMU-BIAS: module health and bias-learning inhibition

**ID:** `R-PX4-IMU-BIAS`

**Status:** Draft

**Applies to:** Single-owner PX4 INS module; active deployment profile and stated dependencies apply.

**Parent:** [SW-PX4](../../software/px4-estimation.md).

**Uses:** [R-IMU-BIAS-INHIBITION](../../runtime/imu/bias-inhibition.md), [R-VERTICAL-ACCELERATION](../../runtime/imu/vertical-health.md), [R-IMU-PROCESS-NOISE](../../runtime/imu/process-noise.md), [R-BIAS-PROCESS-NOISE](../../runtime/imu/bias-process-noise.md), [R-BIAS-LIMITS](../../runtime/imu/bias-limits.md), [PX4-CONFIG](configuration.md).

## Required behavior

The PX4 module uses the shared `runtime::ImuBiasInhibition` decaying-peak policy
specified in `R-IMU-BIAS-INHIBITION`, and the detector in `R-VERTICAL-ACCELERATION`.
No separate continuous-good recovery timer is used.

- Before actual IMU prediction, evaluate once with the prior unit attitude/biases.
  Use separate angular/velocity integration durations for their respective peak
  decay. Build-time defaults are 25 m/s², 3 rad/s and tau 0.5 s, both bias estimators
  enabled. Freeze the selected settings for the session.
  Invalid durations/data disable permissions and preserve peak history.
- Fresh land-detector `at_rest` is the only rest override. This module has no
  fake-position or fake-height modes; GNSS age does not fabricate these modes.
  Consume the preceding available vertical-health fault, not a future verdict.
  High dynamics or that fault inhibits accel bias only; gyro learning follows its
  configured enable switch. These heuristics do not prove health or observability.
- Pass the same six permissions to all GNSS position/velocity, barometer/GNSS
  height, gravity and magnetic corrections at that horizon. Reuse the existing
  constrained gain and Joseph update; do not suppress an entire observation merely because
  bias learning is inhibited. Checked navigation/heading resets preserve biases.
- Evaluate vertical health after the current GNSS velocity attempt and before
  current height attempts. Use current active source sessions and latest valid
  selected innovations, including rejected observations, negating core residuals.
  A full velocity reset supplies zero innovation; a Down-only velocity reset
  retains that attempt's selected diagnostics. A height reset excludes its
  pre-reset diagnostics until the next attempt, without erasing raw gate evidence.
  GNSS height and velocity share one reference; barometer supplies another.
- The pinned 250 Hz module profile averages the two IMU periods using 0.99/0.01
  weights and a 2–8 ms input clamp, starting at 4 ms. This is the module cadence,
  not a claim of identical EKF2 clipping/fault timelines at its downsampled cadence.
  Missing/rejected IMU data resets health history and clears learning permission;
  held prediction rates never update peaks or health. Real measured peaks survive
  gaps. Source/calibration discontinuities retain the restart-required policy.
- After initialization, valid accelerometer-clipping flags no longer reject the
  IMU interval: `R-IMU-PROCESS-NOISE` selects its uncertainty before prediction.
  The same flags inhibit the corresponding accel-bias axes and reach the current
  health evaluation, so accepted clipped intervals retain detector continuity.
  Gyro clipping, undefined accel-clipping bits and clipping during initialization
  remain rejected; no prediction/correction occurs on those samples.
- Publish the vertical detector's held fault through `fs_bad_acc_vertical`, not
  the generic numerical-failure latch. Expose learning masks and health availability
  in module status. Disabled learning is neither invalid bias nor a sensor fault.

Process-noise selection is specified separately in `R-IMU-PROCESS-NOISE`.
Bias-value limiting, gravity correction and height-reset exceptions are specified
in their respective requirements and wired into this module. Shared inhibition
has the scoped ESBMC evidence described under `R-IMU-BIAS-INHIBITION`; this does not close
the module's admission, history invalidation, ordering, forwarding or publication
clauses. Vertical-health source equations and state safety are separately covered
under `R-VERTICAL-ACCELERATION`; that does not prove this module's use of the
verdict. Native backend/correction tests and SITL
are empirical evidence; actual module proof and physical disturbance validation
remain pending.

## Bias-value limiting

The module uses `px4::limit_imu_bias` with configured bounds, defaulting to
0.4 m/s² and 0.15 rad/s; published sensor-bias limits use the same selected values.
It limits startup biases and each successful GNSS velocity/horizontal
position, GNSS/barometer height and magnetic correction group before the next
operation. Rejections and resets do not invoke limiting. Each height source is
handled separately, rather than waiting until the entire height update returns.
Native adapter tests exercise accepted and rejected sources with inhibited
learning; the existing limiter tests cover boundaries, numerical failures and
inward recovery. The gravity adapter applies the same limiter after successful gravity correction.

## Bias process-noise binding

The PX4 module applies the same production selector through `px4::predict_imu`
before real and synthetic gap-hold predictions. Real intervals first select
fault/clipping-dependent IMU variance; gap holds use configured IMU variance.
Both use the prior covariance and start from configured intensities on every
call, so suppression does not latch. Default intensities are 1e-6 m²/s⁵ and
1e-8 rad²/s³, with the existing `intensity * dt` scaling. No correction permission
or bias-value limit is introduced here. Native module tests check equality,
above/below threshold, clipping-dependent release, subsequent recovery and
failure rollback.

## Sampled IMU variance binding

The PX4 module uses `px4::predict_imu` for actual accepted IMU intervals.
It copies configured parameters, selects noise with the configured fault variance
(default `4.9^2` m²/s⁴),
applies `R-BIAS-PROCESS-NOISE`, then calls atomic INS prediction in place.
Configured noise and gyro variance are preserved. Selection/prediction failure
preserves state/P and feeds the existing numerical-fault latch.

The module first checks timestamps, periods, finite increments and continuity.
It accepts only the three defined accel-clipping bits, only after initialization;
gyro clipping remains rejected. Before prediction, both noise selection and bias
permissions consume the preceding health verdict plus current clipping. After
the GNSS velocity attempt, current clipping reaches the health detector before
height fusion. A newly detected held fault therefore affects the next prediction.
Clearing clipping restores configured noise unless the held fault remains set.
Synthetic gap holds retain configured IMU variances and do not consume either adjacent
sample's clipping or update health/peaks. The existing health-history reset on
gaps is unchanged. This fallback and gyro rejection are explicit module limits,
not assertions of EKF2 equivalence. Clipping flags do not reconstruct lost motion.

## Verification boundary

The `imu-prediction` suite verifies actual `ImuPrediction.hpp` clipping admission and
composition: a fresh parameter copy, exact bit decoding/fault variance, selection
before bias inhibition using prior P, one in-place prediction and first-error
rollback. Same-type actual selector producers and E-STEP supply its explicit
callee frames; arbitrary callee statuses/results are admitted. This is helper
wiring under the proof backend, not the actual PX4 Matrix instantiation, input
scheduler, real/synthetic interval selection, health timing, fault latch or output
publisher. These callers must establish the helper's premises separately.
