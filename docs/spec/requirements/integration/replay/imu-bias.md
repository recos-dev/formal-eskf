# R-REPLAY-BIAS: Replay bias-learning binding

**ID:** `R-REPLAY-BIAS`

**Applies to:** Host PX4 replay.

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

## Acceptance and verification

Check metadata parsing, parameter/default reporting, real-sample sequencing and
same-horizon forwarding. Portable policy proofs do not prove this adapter wiring.
The replay has no fake-aiding modes and does not establish physical observability.
