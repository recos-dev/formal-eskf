# R-GNSS-FUSION: GNSS fusion lifecycle

**ID:** `R-GNSS-FUSION`

**Applies to:** Portable runtime policy; integrations supply timestamps, metadata and deployment parameters.

## Required behavior

`runtime::GnssFusion` owns lifecycle metadata for horizontal position, vertical
position and velocity, not estimator state/P, quality qualification or clocks.
This profile assumes GNSS is the sole horizontal and height source; use horizontal
OR 3D velocity per sample. Alternate aiding and reset execution remain separate.

Each group's `action` is `Action::none`, `Action::fuse` or `Action::reset`
for the current sample. This single-use permission is separate from `active` session
state and persistent `recovery_required` history. Sample/time/order conditions
still apply; an action does not execute an estimator operation.

- Caller supplies each group's `enabled`, `ready` and `start_without_reset`.
  Readiness includes initialized frame/state and the group's required attitude
  alignment. Starting without reset requires explicit caller-established safe
  initialization; quality acceptance alone cannot establish this. Otherwise,
  request `initialization_required`, with no fusion permission.
- Update on every estimator tick, including no-data ticks. Nonzero current time
  increases; distinct new samples may share one tick after previous feedback.
  Sample timestamps strictly increase and cannot be future times. `Sample{}`
  means no new receiver sample. Unavailable quality metadata may accompany a
  received sample but cannot authorize fusion or refresh quality health.
  Available quality must be an actual accepted/waiting/rejected result.
- Three positive durations remain fixed until full reset:
  `horizontal_timeout_us`, `height_data_timeout_us`,
  `height_fusion_timeout_us`. Horizontal duration is at most UINT64_MAX/2.
  Compare elapsed differences only after validating time order. No overflowing
  timestamp-plus-duration arithmetic.
- Receipt freshness expires at elapsed **>=** the corresponding data duration.
  Height stops independently at its data deadline. Losing horizontal receipt
  while a horizontal group is active stops all GNSS groups. These are ordinary
  stops, not permanent recovery latches.
- Accepted quality permits a new sample. Waiting/rejected/unavailable quality
  never permits correction. Waiting represents currently passing checks during
  qualification/recovery and refreshes last quality-pass time; unavailable
  metadata does not. On new non-accepted data, elapsed **> horizontal timeout**
  since the last quality pass stops all groups when horizontal aiding was active.
- Starting an authorized session records `start_time_us`, not a fictitious
  successful fuse. Each update revokes old permissions. Record each permitted
  group's actual matching `FusionDecision` once before the next update.
  Only `fused` changes `last_fuse_time_us`; rejection leaves it unchanged.
  Numerical failure requests repair for that group immediately.
- With accepted fresh data and an active horizontal session, decide horizontal
  reset **before either horizontal group attempts fusion**: both position and
  velocity progress older than one horizontal timeout, OR enabled position
  progress older than twice that timeout. Thus one healthy horizontal group
  can sustain the other temporarily. Request recovery for active, enabled,
  ready horizontal groups, without changing state/P. Progress uses the later
  of actual last fusion, successful reset and caller-authorized session start;
  a never-started group has no progress. A horizontal timeout request retains
  its active session while waiting for reset; it does not restart quality health.
- Each group's `initialization_required`/`fusion_timeout`/`bad_vertical_acceleration` requests may grant
  `Action::reset` only on a new, accepted, fresh, enabled and ready sample.
  Reset and correction permissions are mutually exclusive. Each update, even
  an invalid one, revokes old reset permission. `record_reset(group, sample, reset_succeeded)`
  consumes one matching permission after the checked numerical reset returns.
  Success clears recovery, activates the group and updates `last_reset_time_us`,
  but not `last_fuse_time_us`. Only inactive-to-active reset changes session start.
  No ordinary correction is permitted for the same sample after reset.
  Failure stops the group with persistent `reset_failed`, changes no progress
  timestamp and cannot auto-retry. `fusion_failed` also never grants automatic
  reset permission. Stale/missing/duplicate feedback returns `out_of_range`.
- Height uses its own progress deadline, evaluated **after** its current
  permitted attempt. Success at an overdue tick refreshes it; rejection beyond
  the deadline grants reset permission for that same accepted sample because
  this profile has no alternate height reference. The acceleration criterion
  below can also request reset after a successful height correction. Retain the
  active session;
  successful reset renews progress without changing the last correction time.
  A numerical failure never grants this permission. Equality does not expire
  fusion deadlines. Height feedback does not alter horizontal groups or quality.
- `Conditions::has_vertical_velocity` explicitly declares that the velocity group
  includes a qualified Down observation; horizontal-only callers leave it false.
  For an already active velocity session doing ordinary fusion, snapshot the
  height deadline at `update`, before this sample's height operation. Height
  must have an established session and permit either correction or reset on
  this fresh accepted sample. If its progress is strictly beyond the height
  timeout, or the preceding acceleration-health criterion below is expired,
  velocity feedback of `fused` OR `rejected` grants one Down-only reset.
  Numerical failure does not. Startup and full 3D velocity resets already cover
  Down and must not request another reset.
- Execute/acknowledge Down-only reset with `record_vertical_velocity_reset`
  before this sample's height correction/reset. Out-of-order height feedback
  returns `out_of_range` without consuming permission. Even a later successful
  height correction does not cancel the earlier velocity reset decision.
  Success updates only `last_vertical_velocity_reset_time_us`, not height or
  horizontal progress, the velocity group's full-reset/fusion timestamps, or
  quality health. Failure latches velocity `reset_failed`; no automatic retry.
  Wrong-sample/duplicate feedback returns `out_of_range`. Every update (including
  invalid updates), or explicit velocity/height repair, revokes pending permission.
  Restarting the velocity group also clears its Down-reset history. While
  permission is outstanding, `vertical_velocity_reset_reason` identifies
  fusion timeout or bad vertical acceleration. `none` means no permission has
  been granted; consumption/revocation clears the reason. There is no separate
  boolean permission to keep synchronized.
- Ordinary stops can start again after accepted quality and caller-established
  initialization. Losing the last active horizontal group emits one
  `restart_quality` notification; call `GnssQuality::restart_health()` before
  processing the next sample. This clears health timestamps, preserving initial
  qualification, filters and time/reference watermarks. The next check starts
  a new recovery wait; it does not reuse the outage as healthy time.
- Recovery requests persist across good quality and enable/readiness toggles
  until successful permitted reset feedback or explicit repair acknowledgment.
  `restart(group)` acknowledges actual caller-established repair and clears
  only that group's lifecycle metadata, not state/P, quality or time watermarks.
  A new sample is still required. Full `reset()` clears the controller; source,
  frame, configuration or timeline changes also require quality reset and
  re-established initialization.
- Invalid durations/changes return domain_error; invalid time order returns
  out_of_range; invalid available quality returns domain_error. Failed updates
  revoke permissions but preserve history/time/configuration. Invalid feedback
  or restart preserves metadata. The controller cannot verify whether an
  external correction/reset actually ran.

ESBMC checks the actual portable controller over arbitrary uint64 times, flags,
quality/result enums and an initialized, inductively preserved history invariant.
Construction/reset establish ordered progress and permissions; every public update,
feedback and restart preserves it. Checks cover exact validation/failure frames,
strict freshness/deadlines, coupled progress, one-use feedback, height/Down ordering,
recovery latches and unchanged borrowed inputs. Accepted feedback consumes its
permission; the universal guard rejects feedback without permission, establishing
one-use behavior without a fixed multi-call trace. No numerical/backend summary is
used by this integer/Boolean controller. Native tests remain complementary.

These source-level proofs do not establish that an external correction/reset ran,
that quality/health metadata is physically valid, or that module/replay callers
honor the required chronology. Numerical reset is `R-GNSS-RESET`; actual height
coordination is `R-HEIGHT-FUSION`. No separate Lean state-machine proof is claimed.

#### Continuous bad vertical acceleration

`VerticalAccelerationTimes` carries the monitor's update time and last-good time,
not its 3 s held fault flag. `Conditions::vertical_acceleration` supplies the
verdict available before velocity control. Height feedback additionally passes
this horizon's times to `record_fusion`; other groups ignore that argument.

- Nonzero update time means an evaluated snapshot. Known snapshots must satisfy
  `last_good_time_us <= update_time_us <= now_us`; height feedback requires
  `update_time_us == now_us`. Future/reversed times, or an older height snapshot,
  return `out_of_range`. Failed updates revoke permissions without consuming
  history; invalid feedback preserves permission and metadata for a corrected call.
- Zero update time means unavailable; ignore its last-good field and disable
  only the acceleration criterion. Fusion-timeout recovery remains available.
  For a known snapshot, zero last-good means never healthy and is immediately
  expired, matching the reference's zero-timestamp rule. Do not manufacture a
  healthy timestamp on startup, reset or missing data.
- With known health, request recovery if last-good is zero OR
  `now_us - last_good_time_us > 500000`. Equality does not expire. The fixed
  `acceleration_reset_delay_us` matches the reference's internal constant.
  Ordered subtraction avoids overflow. The caller must provide the appropriate
  monitor snapshot and clear availability across gaps/source changes; numerical
  timestamp validation cannot establish physical sensor continuity.
- For established, permitted 3D velocity, retain this criterion at `update`.
  Actual fused/rejected velocity feedback can then authorize Down-only reset
  before height processing. Source quality/readiness, established height
  availability and the existing single-use/reset-order restrictions still apply.
  Startup/full velocity reset already covers Down and grants no duplicate reset.
- After height correction, combine its current acceleration criterion with the
  remaining fusion timeout. Successful correction clears the timeout but does
  not erase continuous bad-acceleration evidence. Thus a sample can legitimately
  record both successful height fusion and successful height reset. A numerical
  correction failure still cannot grant automatic reset. If both criteria hold,
  report `bad_vertical_acceleration` as the reason.
- Reset success changes estimator/progress through existing checked operations;
  it does not change last-good acceleration time or start a cooldown. Continuing
  bad health can require another reset on the next permitted sample. Conversely,
  a new healthy verdict stops the acceleration criterion even while the separate
  3 s fault remains held. Pending reset requests still require qualified data;
  numerical reset failure remains terminal until caller repair.

Reference: PX4 v1.16.2
[height reset predicate](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L48),
[velocity control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp#L132)
and [height control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gnss_height_control.cpp#L103).
This profile remains GNSS-only with an established height reference. Native
transition/replay evidence covers the new criterion; Lean/ESBMC evidence is pending.

#### Replay GNSS lifecycle

See [gnss-lifecycle.md](../../integration/replay/gnss-lifecycle.md).
