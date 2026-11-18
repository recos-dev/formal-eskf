# R-REPLAY-GRAVITY-CONDITIONS: Replay condition sources (host demo)

**ID:** `R-REPLAY-GRAVITY-CONDITIONS`

**Status:** Draft

**Applies to:** Host replay input preparation and same-horizon execution.

**Uses:** [R-ACCELEROMETER-CONDITIONS](../../runtime/gravity/conditions.md).

## Required behavior

`examples/replay_conditions.hpp` supplies conditions for the replay's existing
single navigation source. It does not change correction equations or implement
the full GNSS qualification/reset controller. `fusion_conditions.csv` records
raw-tick conditions; interval observations and correction outcomes are recorded
separately as specified in [R-REPLAY-GRAVITY-SAMPLING](gravity-sampling.md).

- Derive tilt uncertainty from this estimator, never a logged EKF status:
  `P_theta_n = R_nb * P_theta_b * R_nb^T`, using INS covariance rows/columns 6..8.
  Latch alignment when `P_theta_n(0,0)+P_theta_n(1,1) < (3*pi/180)^2`.
  Evaluate tilt on the predicted prior before that tick's navigation corrections.
  A new replay resets the latch; subsequent variance growth does not clear it.
  A constructed quaternion alone is not alignment. Unit attitude and PSD
  covariance remain caller premises; non-finite/negative computed tilt variance
  is a replay error, not permission to fuse.
- Horizontal aiding starts only after horizontal position/velocity correction
  or a permitted full horizontal-position/velocity reset succeeds,
  remains active between navigation events and expires when elapsed time since
  the last successful event reaches 7 seconds. A resumed successful event
  reactivates it. Height and Down-only velocity resets do not refresh this timer;
  the initial seed is not a fusion event. With the quality/lifecycle profile,
  each navigation group may reject independently. Legacy ungated input retains
  fail-fast correction behavior. This successful-progress timer is not a substitute for
  independent source lifecycle, quality qualification or recovery in deployment.
- PX4 preparation takes explicit `vehicle_land_detected.at_rest` at or before
  each IMU timestamp, with age strictly below 3 seconds. Absent/stale data is
  unknown (`-1`) and disables only the rest override; `landed`, constant speed
  and a gravity-sized acceleration are not substitutes. Unlike the delayed
  system-flag buffer, this undelayed demo checks freshness on each IMU tick.
  The external detector's flag is not independent physical ground truth.
- Decode `sensor_combined.accelerometer_clipping` bits X/Y/Z from that sample.
  Missing clipping stays unknown (`-1`), not zero. No complete fusion conditions
  are returned until clipping is known. Optional `imu_flags.csv` must match
  every IMU timestamp exactly, including the seed; malformed/missing/extra rows
  are errors. An absent sidecar leaves flags unknown, preserving legacy/KITTI
  input compatibility without inventing metadata.

Behavior references at PX4 `e1c28e2`:
[tilt latch](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/control.cpp#L72),
[NED tilt variance](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/ekf_helper.cpp#L932),
[active horizontal sources](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/estimator_interface.cpp#L675),
[GNSS missing-data stop](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp#L115),
[7-second limit](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/common.h#L512),
[rest metadata](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF2.cpp#L2849),
[clipping metadata](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF2.cpp#L737).
The local-error covariance transformation and missing-metadata refusal are
project integration behavior, not copied estimator status. This is native-tested
condition preparation, not proof of physical alignment or full PX4 equivalence.
