# R-IMU-PROCESS-NOISE: fault-dependent sampled IMU variance

**ID:** `R-IMU-PROCESS-NOISE`

**Status:** Draft

**Applies to:** Portable runtime policy; integrations supply timestamps, metadata and deployment parameters.

**Uses:** [E-NOISE](../../core/process-noise.md).

## Required behavior

`runtime::select_imu_process_noise(configured_noise, conditions,
fault_variance, output)` selects body-axis accelerometer measurement variances
before INS prediction. Conditions contain the preceding vertical-health fault
and known clipping flags for the current IMU interval.

- Replace an axis's `specific_force_variance` with `fault_variance` when the
  vertical fault is set or that body axis is clipped. Otherwise retain its
  configured variance. A vertical fault therefore selects all three axes.
  This is replacement, not `max(configured, fault)`; it can lower an unusually
  large configured value. Preserve gyro variance and both bias-driving vectors.
- Inputs are variances in m²/s⁴, already squared. Validate the fault variance
  first, then all twelve configured noise coefficients, including replaced or
  unselected values. Non-finite values return `non_finite_input`; negative values
  return `domain_error`. Zero is valid. No arithmetic is required on the noise
  values. Failure preserves output; success permits a whole-noise input/output
  alias and otherwise preserves input. No state, P, timer or permission is changed.
- Start every step from original configured noise. Clearing clipping restores
  that axis immediately unless the held vertical fault still selects all axes.
  The caller owns metadata availability and fault timing. Correction's bias
  permissions are independent: disabled learning does not disable this selection.
- Apply `R-BIAS-PROCESS-NOISE` **after** this selection so its accel bias variance
  threshold uses the selected measurement variance. Its driving intensity retains
  `intensity * dt` scaling. Prediction retains `E-NOISE`/`E-PCOV`: sampled IMU
  variance contributes with `dt²`, and body-axis noise maps through `R(q)`.
  No rotation or time scaling is duplicated in this selector.

Behavioral reference: PX4 v1.16.2 [covariance prediction](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/covariance.cpp#L113)
and [bad-accelerometer noise constant](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/common.h#L83).
Despite its reference name, `BADACC_BIAS_PNOISE = 4.9 m/s²` is a sampled-acceleration
standard deviation here, not bias random-walk intensity. The deployment selects
the fault variance explicitly; the reference value is
4.9^2 m^2/s^4. Alignment covers the selection rule and ordering, not all covariance
equations, precision, sampling rates or sensor calibration.

Native tests cover all selector choices, aliases, actual rotated covariance,
module clipping admission/prediction, restoration, gyro variance and bias-noise thresholds,
failure rollback and detector/bias-policy sequencing. Replay retains its existing
metadata behavior. The `imu-policies` ESBMC suite verifies the actual portable
selector for binary32/binary64, every Boolean condition and both legal output
aliases, including all invalid inputs and first-error/publication behavior. Lean
proves preservation of nonnegative noise and exact prediction PSD under the
existing covariance assumptions. Detector decisions, physical clipping/calibration,
module/replay sequencing and target numerical behavior remain separate obligations;
normal-flight SITL does not establish clipped-flight accuracy. See the
[verification scope](../../../../guides/verification.md).

## Caller bindings

- [R-REPLAY-BIAS](../../integration/replay/imu-bias.md).
- [R-PX4-IMU-BIAS](../../integration/px4/imu-bias.md).
