# R-PX4-IMU-BIAS: module health and bias-learning inhibition

**ID:** `R-PX4-IMU-BIAS`

**Applies to:** Single-owner PX4 INS module; active deployment profile and stated dependencies apply.

## Required behavior

The PX4 module uses the shared `runtime::ImuBiasInhibition` decaying-peak policy
specified in `R-REPLAY-BIAS`, and the detector in `R-VERTICAL-ACCELERATION`.
No separate continuous-good recovery timer is used.

- Before actual IMU prediction, evaluate once with the prior unit attitude/biases.
  Use separate angular/velocity integration durations for their respective peak
  decay. Fixed settings are 25 m/s², 3 rad/s and tau 0.5 s, both bias estimators
  enabled. Replay supplies equal periods through the same interface.
  Invalid durations/data disable permissions and preserve peak history.
- Fresh land-detector `at_rest` is the only rest override. This module has no
  fake-position or fake-height modes; GNSS age does not fabricate these modes.
  Consume the preceding available vertical-health fault, not a future verdict.
  High dynamics or that fault inhibits accel bias only; gyro learning remains
  enabled. These heuristics do not prove physical health or observability.
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
has the scoped ESBMC evidence described under `R-REPLAY-BIAS`; this does not close
the module's admission, history invalidation, ordering, forwarding or publication
clauses. Vertical-health source equations and state safety are separately covered
under `R-VERTICAL-ACCELERATION`; that does not prove this module's use of the
verdict. Native backend/correction tests and SITL
are empirical evidence; actual module proof and physical disturbance validation
remain pending.
