# R-REPLAY-MAGNETIC-LIFECYCLE: Replay magnetic lifecycle

**ID:** `R-REPLAY-MAGNETIC-LIFECYCLE`

**Applies to:** PX4 replay host preparation/execution.

**Uses:** [R-MAGNETOMETER-LIFECYCLE](../../runtime/magnetic/lifecycle.md), [R-MAGNETOMETER-RESET](../../runtime/magnetic/heading-reset.md).

## Required behavior

Replay uses [the portable lifecycle](../../runtime/magnetic/lifecycle.md) with 1 s startup health (disabled for check mask zero), a 0.5 s maximum
interval and a 7 s active-fusion timeout. Own covariance-based tilt alignment
supplies readiness; yaw and the fixed field reference remain causal seed
assumptions. No external yaw/reset event occurs during replay; magnetic timeout
recovery may perform [the heading reset](../../runtime/magnetic/heading-reset.md).
Horizontal aiding means an actual recent horizontal GNSS fusion, not merely a
received GNSS packet. Only a recorded `at_rest=true` enables the rest override;
missing rest metadata is not evidence of motion. With horizontal aiding, the
rest override additionally requires consistent magnetic heading.

Supply the replay-owned `heading_aligned` flag on every lifecycle update. When
an active session stops without horizontal aiding, clear alignment whether the
stop comes from the update or actual fusion feedback. A successful heading reset
restores activity and alignment together; returning GNSS aiding alone cannot do
so. A stop with horizontal aiding retains alignment. This follows the pinned
`stopMagFusion()` heading-loss rule within our fixed-field profile.

The reference is [PX4 v1.16.2 magnetometer control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp),
with timing from [ekf.h](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf.h)
and [common.h](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/common.h).

| Decision | Implemented rule |
|---|---|
| Start | Continuing conditions pass, stable yaw/reset frame, at least four samples, more than 1 s since the last failed field check unless the health duration is disabled, newest sample age strictly below 0.5 s. If heading is unaligned, apply a real heading reset first; otherwise actual successful fusion activates. |
| Continue | A current good field may fuse without repeating startup health, subject to `!horizontal_aiding || heading_consistent`. Field rejection or heading inhibition does not itself end the session. |
| No data | On a no-sample tick, stop at age **>= 1 s**; preserve field history and allow fresh qualified data to restart. |
| Fusion timeout | Try the current observation first. Only an active session still without success for **> 7 s** takes the timeout branch. |
| Timeout with horizontal aiding and no rest override | Stop, without a permanent fault latch; later qualified observations may retry automatically. |
| Timeout without horizontal aiding or with rest override | Request a heading reset; execute only with current fresh, health-qualified field data and readiness. Record actual reset progress separately from fusion. |

The metadata controller is not a complete EKF2 replacement. PX4 owns magnetic
earth/bias states, integrated WMM lookup, declination fusion, full mode selection and separate
latest/delayed timelines. Our supported reset is
heading plus covariance against the existing fixed field, not magnetic-state reset.
Our fixed-field core has no magnetic-only update: field rejection or aided
heading inconsistency inhibits navigation correction without reporting
fictitious magnetic-state fusion. Heading inhibition preserves the session and
real progress timestamps, does not run a failed-attempt timeout, and permits
retry once consistent or no longer horizontally aided. Field/gate rejection
still follows the existing timeout rules. Thus a physical disturbance need not
produce the same outcomes or
progress timestamps as EKF2. Replay's single timestamp timeline does not model
PX4 buffer-push recency or sensor delay.

Ordinary stops restart automatically. A timeout reset waits for accepted field
health and uses `reset_magnetic_heading` with `MagnetometerFilter::field_b()`;
it does not replace the reference with the observed field. Replay uses an explicit
heading standard deviation from the [heading parameters](magnetic-observation.md), independent of
vector observation noise. Successful resets and their applied left quaternion deltas
are logged in `magnetometer_resets.csv`; `last_reset_timestamp_us` remains distinct
from actual fusion time. Numerical errors terminate after logging.
Known source/calibration changes execute [the lifecycle reinitialization policy](../../runtime/magnetic/lifecycle.md):
reseed the LPF, qualify the new source and either reset heading or attempt ordinary
fusion. Unknown flight state leaves source recovery pending. Reference revisions
keep the measurement LPF and apply the reference-update policy. Reset CSV rows
include the actual cause (`fusion_timeout`, `source_changed`, `reference_changed`).
Raw samples remain the input to field checks and normal correction. Filter updates
run once per consumed enabled sample, even while waiting for health/readiness;
filter failure terminates replay. No correction follows reset on that sample.
Summary counts partition
events into fused, gate-rejected, lifecycle-skipped and superseded; heading resets
are a separate action count and may follow a rejected attempt. No-data
state transitions use source timestamp zero. Native checks are not formal proof.
