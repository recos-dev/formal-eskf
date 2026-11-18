# R-REPLAY-GNSS-GATING: Replay GNSS gating

**ID:** `R-REPLAY-GNSS-GATING`

**Status:** Draft

**Applies to:** Host replay input preparation and same-horizon execution.

**Parent:** [SW-REPLAY](../../software/replay-aiding.md).

**Uses:** [R-MEASUREMENT-FUSION](../../runtime/measurement-fusion.md), [E-INNOV](../../core/innovation.md).

## Required behavior

PX4 preparation supplies one `gnss_parameters.csv` row from logged
`EKF2_GPS_P_GATE`, `EKF2_GPS_V_GATE`, `EKF2_GPS_P_NOISE`, `EKF2_GPS_V_NOISE`
and `EKF2_NOAID_NOISE`. Gates are clamped to at least one sigma at the adapter
boundary, not in the core. Require finite settings, positive noise values and
safe finite squared variances. Reject recorded setting/lever-arm changes during
the replay; require equal configured GNSS/IMU offsets because compensation is
not implemented. Missing required parameters are errors, not guessed defaults.

The noise profile follows the ordinary GNSS path of
[PX4 v1.16.2](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp)
under this demo's sole horizontal/height aiding assumptions:

- Horizontal position: square `max(min(max(eph, position_noise),
  position_noise_limit), 0.01)` for both axes.
- Height: square `max(epv, 1.5 * position_noise)`, without a noise cap or a
  height-bias estimate/variance. GNSS is the fixed sole height reference.
- Velocity: square `max(speed_accuracy, velocity_noise, 0.01)` for N/E;
  use 1.5 times that standard deviation for Down. All covariances are diagonal.
- Initialize position/velocity covariance with these same noise rules;
  retain existing attitude/bias initial uncertainties. Receiver accuracy axes
  are treated as the fixed navigation axes, a local-flight approximation.

Consume each navigation event once at the following IMU tick. Fuse 3D velocity,
horizontal position, then height through the existing runtime APIs. Each group
forms its gate and correction from the latest committed prior. This differs
from PX4's precomputed position/velocity diagnostics and scalar update schedule;
neither identical decisions nor numerical EKF2 equivalence is claimed.
Rejection preserves that group's prior and does not block other groups. Only
actual horizontal-position or 3D-velocity correction/reset refreshes horizontal-aiding
time; height operations, Down-only velocity reset, receipt, rejection and failure
never do so.

`gnss_updates.csv` records each group's outcome/status and per-axis innovation,
S diagonal, test ratio, V diagonal and diagnostic validity. Numerical fusion
failure is logged before stopping. Invalid prepared input stops replay as an
input error. Event counts are not successful-correction counts; report accepted
and rejected groups separately. A fully rejected event still counts as consumed.
[Quality qualification](gnss-quality.md) supplies acceptance; the
[GNSS lifecycle](gnss-lifecycle.md) controls permission
and GNSS reset execution, including height and Down-only velocity recovery.
`R-GNSS-VERTICAL-RECOVERY` supplies the bounded vertical-innovation exceptions.
Delay compensation and changes to magnetic/gravity scheduling are not included.
Without the parameter file, retain legacy/KITTI ungated noise and fusion order.
Replay checks are integration evidence, not new Lean/ESBMC proofs or accuracy claims.

## Vertical innovation recovery binding

Replay enables velocity recovery only with a quality/lifecycle profile, using
its existing maximum-speed-accuracy setting. It consumes the preceding valid
health verdict for velocity, then evaluates current velocity/preceding height
health before attempting height. Both health caches use selected innovations;
quality-rejected velocity updates evidence without correction. Missing clipping
or a broken IMU interval clears health, so no stale fault releases a gate.
Multiple due events retain the existing sequential estimator updates and the
[single-monitor-transition batching rule](imu-bias.md#vertical-health-binding).

`gnss_updates.csv` adds the selected innovation and group-level limit flag;
summary counts limited successful velocity/height fusions separately. Rejected
quality/lifecycle conditions still skip correction and do not refresh progress.
Horizontal position has no exception. Existing timeouts/reset behavior is
separate from the gate exception; the independent 500 ms last-good-acceleration
reset is specified in `R-GNSS-FUSION`.
