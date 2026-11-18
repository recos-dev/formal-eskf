# R-REPLAY-GNSS-LIFECYCLE: Replay GNSS lifecycle

**ID:** `R-REPLAY-GNSS-LIFECYCLE`

**Status:** Draft

**Applies to:** Host replay input preparation and same-horizon execution.

**Uses:** [R-GNSS-FUSION](../../runtime/gnss/lifecycle.md).

## Required behavior

Behavior reference: pinned [horizontal GNSS controller](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp)
and [height controller](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gnss_height_control.cpp).

| Rule | Replay profile |
|---|---|
| Horizontal missing data / poor quality | 7 s; data expires at equality, quality strictly beyond |
| Height missing data | 1 s, expires at equality |
| Horizontal unsuccessful fusion | Both groups >7 s, or enabled position >14 s; before correction |
| Height unsuccessful fusion | >5 s after current height attempt |
| Horizontal stop | Restart quality health timing, retain initial qualification |
| Horizontal startup / timeout reset | Execute velocity then horizontal-position reset, instead of same-sample correction |
| Height startup / re-entry | Reset local Down position and its covariance on the accepted sample; no correction on that sample |
| Height timeout reset | After rejected correction beyond 5 s, reset Down using the same accepted observation |
| Height-failure vertical velocity reset | After normal 3D velocity correction (fused or rejected), reset only `v_D` when preceding height progress is >5 s old; execute before horizontal-position/height operations |
| Numerical correction / reset failure | Log and terminate; no automatic recovery |

Enable lifecycle with the quality input pair, not for legacy/KITTI inputs.
Invalid-velocity messages do not refresh receiver receipt; missing required
quality metadata advances receipt only. Run no-data updates between events.
Preserve group-independent gate/correction decisions, velocity/position/height
order. Only successful horizontal position/velocity correction or reset refreshes
horizontal-aiding timestamps; height correction/reset and Down-only velocity reset
never do. Log every received
lifecycle sample and no-data transitions in `result/gnss_lifecycle.csv`.
Quality skips, lifecycle skips, innovation rejections and resets are distinct outcomes.
Record per-axis reset reason, status and new-minus-old delta in `gnss_resets.csv`;
failed resets have invalid, empty deltas. Keep successful reset counts separate
from correction and rejection counts. A height-recovery event can contain both
a correction (fused or rejected) and a successful reset; these counters count operations,
not mutually exclusive events. Reset success refreshes progress without
claiming an innovation passed or restarting quality qualification.
Log Down-only resets as `vertical_velocity` in `gnss_resets.csv`, with their own
`vertical_velocity_resets` summary count. A normal velocity correction and this
reset may both occur on one sample. Preserve the actual correction outcome.
The Down observation variance is already prepared as `(1.5 * speed_std)^2`;
the reset API must not apply that scaling again.

Replay passes the preceding valid health times into GNSS lifecycle update and
current pre-height health times into height fusion feedback. Missing IMU/clipping
history supplies unavailable times. Reset logs distinguish
`bad_vertical_acceleration` from `fusion_timeout`; summary counts acceleration
height/Down-velocity resets separately and records the 500000 us delay.

The replay's existing seed establishes its initial frame and assumed attitude
alignment; it is not an alignment detector. All three groups require an actual
reset on their first accepted sample and on re-entry. Height-only data loss
leaves horizontal sessions running; return of qualified data resets height.
A complete GNSS outage stops all groups and restarts quality health timing;
reset/re-entry waits for renewed acceptance instead of treating receipt as recovery.

Velocity, Down-only velocity, horizontal-position and height operations are
independent checked transactions, in that order; a later failure does not roll
back earlier completed operations. Log
the completed/failed operations and terminate without a success summary. Numerical
correction failure is not converted into an automatic reset with the same input.

This is a lifecycle integration, not full EKF2 equivalence: one replay clock
uses source sample recency instead of separate arrival/delayed horizons; no
alternate sources, GNSS heading/GSF emergency yaw or height-bias estimator is
implemented. Continuous-acceleration recovery follows [the portable continuous-acceleration criterion](../../runtime/gnss/lifecycle.md#continuous-bad-vertical-acceleration);
bounded innovation exceptions are specified in `R-GNSS-VERTICAL-RECOVERY`. Existing seed,
sequential gates and numerical models are retained.
