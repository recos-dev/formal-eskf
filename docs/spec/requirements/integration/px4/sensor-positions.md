# R-PX4-SENSOR-POSITIONS: sensor and output reference points

**ID:** `R-PX4-SENSOR-POSITIONS`

**Status:** Draft

**Applies to:** Single-owner PX4 INS module; active deployment profile and stated dependencies apply.

**Parent:** [SW-PX4](../../software/px4-estimation.md).

## Required behavior

F-05.1 adopts the pinned PX4 v1.16.2 nominal lever-arm compensation. The main
INS and output-predictor states describe the selected IMU. Published local/global
position and odometry describe a fixed vehicle body origin, normally its centre
of gravity. These are different points in the same FRD/NED frames.

- Read `FESKF_IMU_POS_X/Y/Z` and `FESKF_GPS_POS_X/Y/Z` at startup: metres from
  the body origin to IMU and GNSS antenna, respectively, in body FRD. Defaults
  are zero. Reject non-finite or out-of-range coordinates before registration;
  the initial small-vehicle profile permits each coordinate in `[-10, 10]` m.
  This is a deployment bound, not a PX4 requirement or an accuracy guarantee.
  Installation calibration belongs to the airframe integrator. Changes require
  restart; source selection does not silently change calibration or output point.
  A replacement GNSS identity is assumed to use the same configured mounting
  point; a receiver at a different point requires new calibration and restart.
- With `r = gps_b - imu_b`, delayed attitude `R` and the admitted delayed IMU
  rate, subtract `R r` from GNSS NED position and
  `R ((omega_measured - b_g) cross r)` from GNSS NED velocity. Add `(R r)_D`
  to GNSS MSL altitude because altitude has the opposite sign to Down.
  Compensation precedes gating, correction and all GNSS reset paths. Use each
  operation's latest committed prior, consistent with the module's existing
  ordered fusion contract; do not substitute current output attitude/rate.
- Keep the geographic origin at the first accepted antenna fix. Initialize
  IMU position to `-R r` and velocity to compensated receiver velocity. Keep
  MSL and ellipsoid reference altitudes unchanged. Raw receiver quality checks,
  drift checks, buffered data and source metadata remain at the antenna.
- At publication, subtract `R_current imu_b` from current IMU position and
  `R_current (omega_output cross imu_b)` from current IMU velocity. Use the
  latest propagated, bias/tracking-corrected body rate retained under
  [R-PX4-OUTPUT-PREDICTOR](output-predictor.md). All three navigation topics
  use this same point and output epoch; their publication does not mutate the
  retained IMU state. `z_deriv` retains the existing corrected-velocity convention.
- Main resets still transform the retained/current IMU output. An orientation
  reset with a nonzero IMU offset also changes the published point: report its
  actual position/velocity deltas and increment the associated local/global
  reset counters. Accumulate these with translation resets in the same fusion
  step. Do not double count their translation or the odometry reset event.
  A zero IMU offset adds no reference-point reset notifications.
- Preserve receiver noise floors/caps, observation Jacobians, state dimension
  and core covariance equations. As in the reference's nominal preprocessing,
  treat installation offsets as deterministic. Calibration uncertainty and
  attitude/gyro-error correlations introduced by preprocessing are not modeled.
  Published variance fields remain the existing delayed **IMU-state proxies**,
  not covariance transported to the body point or current epoch. Local
  acceleration fields also retain the IMU-point acceleration; angular acceleration
  and centripetal terms for body-origin acceleration are not supplied. These
  limitations require separate F-06.1 review, not an implicit precision claim.
- Keep all transforms fixed-size and allocation-free. Non-finite transformed
  navigation output must not be published as valid. GNSS numerical failures
  retain the existing checked-operation/fault behavior. No terrain, barometer
  lever arm, moving installation geometry or geographic recentering is added.

Native module tests must exercise zero/coincident and nonzero installations,
tilted rotation, gyro bias, startup and normal/reset fusion, altitude signs,
delayed versus current epochs, publication and reset deltas. Independently
differentiate rotating-point positions for the velocity oracle; do not simply
repeat the production cross-product expression. Check actual parameter binding,
invalid inputs and restart-only ownership. Tests and sanitizer runs are empirical
evidence; no new Lean/ESBMC proof or physical installation qualification is claimed.

Reference: pinned PX4 [GNSS velocity/position preprocessing](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp),
[GNSS height preprocessing](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gnss_height_control.cpp)
and [output reference point](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/output_predictor/output_predictor.h).
