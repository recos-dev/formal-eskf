# R-REPLAY-BIAS: Replay bias-learning binding

**ID:** `R-REPLAY-BIAS`

**Status:** Draft

**Applies to:** Host PX4 replay.

**Uses:** [R-IMU-BIAS-INHIBITION](../../runtime/imu/bias-inhibition.md), [R-VERTICAL-ACCELERATION](../../runtime/imu/vertical-health.md), [R-IMU-PROCESS-NOISE](../../runtime/imu/process-noise.md), [R-BIAS-PROCESS-NOISE](../../runtime/imu/bias-process-noise.md), [R-BIAS-LIMITS](../../runtime/imu/bias-limits.md).

## Required behavior

Replay directly uses the shared `runtime::ImuBiasInhibition`. It evaluates
the prior biases and unit attitude once before
prediction, then supplies the same six `Ins::BiasUpdate` permissions to every
GNSS, gravity and magnetic correction at that horizon. It uses decaying peaks
without a separate continuous-good timer. Learning inhibition
does not control prediction noise; `R-BIAS-PROCESS-NOISE` supplies that independent
policy. Core equations and bias-preserving resets remain unchanged.

Use [R-IMU-BIAS-INHIBITION](../../runtime/imu/bias-inhibition.md) with the same
logged period for both sensor intervals. On policy failure, terminate replay;
a new replay session starts with fresh peaks. Missing intervals do not supply
held measurements or reset envelopes.

- Preparation exports logged `EKF2_ABL_ACCLIM`, `EKF2_ABL_GYRLIM`, `EKF2_ABL_TAU`
  and the two enable flags in `bias_parameters.csv`. New logs use
  `EKF2_IMU_CTRL` bits 0/1; legacy `EKF2_AID_MASK` bit 2 disables accel bias,
  with gyro bias enabled. Parameter changes within the session are rejected.
  CSV inputs without this file use explicit 25 m/s², 3 rad/s, 0.5 s defaults
  with both bias estimators enabled. The 0.5 s value is a decay time constant.

- Preserve the three-column `imu_flags.csv`. In the replay input adapter only,
  missing accel clipping assumes clear **for bias learning**, with an explicit
  diagnostic/count. Gravity fusion still sees unknown clipping and is blocked.
  Unknown rest does not supply a rest override. Gyro clipping is not an input to
  this inhibition rule. Reject recorded IMU selection/calibration/frame changes;
  absent metadata cannot prove continuity or physical sensor health.

- The replay implements no fake-position/height modes, so their flags are false.
  `R-VERTICAL-ACCELERATION` supplies the preceding valid IMU horizon's fault to
  bias inhibition. A fault disables all accel axes; gyro permissions remain
  independent. Unknown clipping or an IMU gap invalidates that verdict before
  learning is evaluated. Unavailable health adds no fault inhibition under this
  replay compatibility profile; this is not evidence of physical sensor health.
  `bias_learning.csv` records metadata, assumptions, peaks, permissions and the
  health verdict actually consumed; counts describe IMU ticks, not corrections.

## Bias-value limiting

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

## Bias process-noise binding

Replay applies bias-noise selection before every real or gap-hold prediction,
using the IMU variances selected by `R-IMU-PROCESS-NOISE` (or configured variances
when that selection is unavailable) and unchanged configured intensities
(1e-6 m²/s⁵ and 1e-8 rad²/s³). These are explicit replay settings, not newly
calibrated flight noise. Prediction failure still aborts.
`bias_learning.csv` records the six selected-noise flags for real IMU ticks;
summary inhibition counts include gap-hold predictions and count only configured
positive intensities suppressed by the rule. Selection is not proof that a tiny
floating-point increment changed P.

## Sampled IMU variance binding

Replay sets
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

## Vertical-health binding

- Evaluate once per real IMU prediction, including ticks without GNSS messages.
  Require the GNSS quality/lifecycle profile and known clipping. Legacy inputs
  without that profile report unavailable; do not infer source activity from a
  correction count. Barometer/range/vision slots remain inactive. GNSS height
  and velocity alone can only give medium likelihood, requiring clipping to
  trigger a fault.
- Use actual GNSS correction diagnostics, including gate-rejected results:
  negate the selected `fusion_innovation` and retain S. A permitted limit is
  reflected in the monitor cache, as in the reference's aid-source status;
  raw residuals/ratios remain in the fusion log. While velocity aiding remains
  active after a quality rejection, evaluate its velocity gate/limit for the
  cache without correcting the estimator.
  Invalid receiver velocity is not a new received sample; unavailable quality
  metadata invalidates the velocity cache. Numerical failures abort replay.
- Take the diagnostic snapshot after current velocity processing and before
  current height processing. It contains the current velocity cache and the
  preceding height cache. Height activity is from before height control; velocity
  activity reflects current horizontal control. Inactive sources are excluded;
  caches are cleared after lifecycle stop. Successful full velocity/height resets
  set that cache's innovation to zero and S to observation variance, as in the
  reference's aid-status reset. A Down-only reset retains velocity diagnostics.
- Evaluate the snapshot before height fusion, so height can consume the current
  fault. Prediction, bias permissions and every velocity event at this horizon
  consume the preceding horizon's fault. For multiple due GNSS messages,
  re-evaluate each pre-height snapshot from the same saved preceding monitor
  state and retain the final result. Clipping therefore advances once, while
  every height attempt sees its current pre-height evidence. This batching
  remains a replay approximation, not EKF2's receiver-buffer schedule.
- Initialize nominal/average interval from the seed IMU integration period.
  For each actual sample update `average = 0.99*average +
  0.01*clamp(dt, 0.5*nominal, 2*nominal)`. Accel/gyro periods are synchronized in
  replay. The averaging rule follows the reference; replay's logged prediction
  cadence replaces EKF2's configured downsampled cadence. Do not use GNSS dt.
  Missing intervals/unknown clipping clear monitor, diagnostic caches and average
  history before consuming a prior verdict. Held gap predictions never increment
  clipping counters. Restart from the next actual sample with known metadata.
- `vertical_acceleration_health.csv` reports availability, interval, counters,
  likelihood (0/1/2 = low/medium/high), current fault/timers and the signed input
  diagnostics with source timestamps. `bias_learning.csv` reports the preceding
  verdict used for permissions. Summary counts separate evaluated, unavailable,
  fault-held and fault-inhibited ticks. An available low verdict with no active
  diagnostics still does not prove health.

The replay fault feeds accel bias-learning permissions and the sampled
noise selection in `R-IMU-PROCESS-NOISE`, plus the GNSS fusion exceptions in
`R-GNSS-VERTICAL-RECOVERY`. Its last-good time separately feeds the 500 ms
reset criterion in `R-GNSS-FUSION`, shared by the module height coordinator.
## Acceptance and verification

Check metadata parsing, parameter/default reporting, real-sample sequencing,
same-horizon forwarding, bias preservation/recovery and metadata interruptions.
Portable policy proofs do not prove this adapter wiring. The replay has no
fake-aiding modes and does not establish physical observability or flight accuracy.
