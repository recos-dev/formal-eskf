# R-IMU-PROCESS-NOISE: fault-dependent sampled IMU variance

**ID:** `R-IMU-PROCESS-NOISE`

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
standard deviation here, not bias random-walk intensity. Replay sets
`BAD_ACCEL_STD = 4.9` and passes its square (approximately 24.01 m²/s⁴); nominal
accel/gyro variances remain 1 m²/s⁴ and 0.0004 rad²/s². Alignment covers the
selection rule, constant and ordering, not all covariance equations, precision,
IMU sampling rates or calibration of a particular sensor.

Replay uses the preceding valid vertical verdict and current clipping before
prediction. With unknown clipping it retains configured measurement noise,
records unavailability and does not feed assumed-clear flags to this selector.
Synthetic gap holds likewise use configured noise without reusing previous or
future clipping. These are explicit replay fallbacks, not EKF2 behavior or
proof of healthy data. Known clipping still selects noise without a GNSS health
profile; unavailable vertical-health evidence remains separately reported.
`bias_learning.csv` records the three selected accel variances for actual IMU
ticks. Summary records the fault standard deviation, per-axis override counts
and unavailable predictions (including gap holds). Override counts describe
selection, even if configured and replacement variances happen to be equal.

The PX4 module uses `px4::predict_imu` for actual accepted IMU intervals.
It copies configured parameters, selects noise with fixed `4.9^2` fault variance,
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

Native tests cover all selector choices, aliases, actual rotated covariance,
module clipping admission/prediction, restoration, gyro variance and bias-noise thresholds,
failure rollback and detector/bias-policy sequencing. Replay retains its existing
metadata behavior. The `imu-policies` ESBMC suite verifies the actual portable
selector for binary32/binary64, every Boolean condition and both legal output
aliases, including all invalid inputs and first-error/publication behavior. Lean
proves preservation of nonnegative noise and exact prediction PSD under the
existing covariance assumptions. Detector decisions, physical clipping/calibration,
module/replay sequencing and target numerical behavior remain separate obligations;
normal-flight SITL does not establish clipped-flight accuracy. The additional
`imu-prediction` suite verifies actual `ImuPrediction.hpp` clipping admission and
composition: a fresh parameter copy, exact bit decoding/fault variance, selection
before bias inhibition using prior P, one in-place prediction and first-error
rollback. Same-type actual selector producers and E-STEP supply its explicit
callee frames; arbitrary callee statuses/results are admitted. This is helper
wiring under the proof backend, not the actual PX4 Matrix instantiation, input
scheduler, health timing, fault latch or output publisher. See the
[verification scope](../../../../guides/verification.md).
