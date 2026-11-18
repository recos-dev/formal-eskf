# R-MAGNETOMETER-LIFECYCLE: magnetic health and fusion lifecycle

**ID:** `R-MAGNETOMETER-LIFECYCLE`

**Applies to:** Portable magnetic runtime policy; caller bindings are specified separately.

## Required behavior

`runtime::MagnetometerFusion` manages one selected magnetic source without owning
measurements, estimator state/P, clocks or numerical reset operations.
`State::action` is one consumable instruction for the current sample: `none`,
`fuse` or `reset`. It is independent of the active session, persistent recovery
reason and progress timestamps. Each update revokes the previous instruction;
only matching feedback consumes a new instruction. `filter_reset` separately
requests LPF reseeding and does not authorize correction or acknowledge a reset.
`Conditions::ready` covers continuing prerequisites: tilt alignment, valid
reference/state/P and permitted aiding policy. Initial heading is caller-established;
the controller tracks the new heading-reset requirement after source changes.
`starting_allowed` additionally excludes a yaw-alignment change or quaternion-reset frame. Horizontal-aiding and
at-rest flags select the timeout action; they must reflect actual caller state.
`heading_consistent` defaults to false and carries the current heading monitor's
verdict. Ordinary navigation fusion requires it when horizontally aided.
Heading inhibition reports `heading_inconsistent`, preserves health/session and
actual progress clocks, and cannot accept fusion feedback. Aided at-rest timeout
or reference changes cannot authorize yaw reset while heading is inconsistent;
explicit source-change alignment recovery retains its separate reset policy.

- Call `update` on every estimator tick. Current time is nonzero and strictly
  increasing. New sample times strictly increase, including across source changes,
  and cannot be future times. `Sample{}` means no new sample and ignores the other
  fields. A source has a nonzero adapter-assigned ID, calibration count and
  `reference_id` revision for the supplied field in the same navigation frame.
  `field_accepted` is the actual successful same-sample/same-prior field check,
  not an innovation outcome. The adapter supplies aligned usable samples and
  truthful freshness on one timeline; delayed buffering is outside this API.
- `health_time_us=0` disables only health-duration qualification; a positive value
  requires strictly more elapsed time. Positive `maximum_interval_us` and
  `fusion_timeout_us`, and the selected health duration remain fixed until full
  reset. Require `maximum_interval_us <= UINT64_MAX/2`
  so the two-interval no-data bound is representable. Compare elapsed unsigned
  differences after validating order; never add durations to timestamps.
- Count consumed samples while enabled, saturating at four. Check health on new
  enabled/ready samples. A rejected field sets `last_field_failure_time_us=now_us`;
  the first check seeds that timestamp even when good. Startup requires strictly
  more than the enabled health duration since that timestamp, at least four samples,
  current field acceptance, stable starting conditions and age strictly less than
  one maximum interval. Permission is a start attempt, not an active session.
- Every update revokes old permission. Call the actual gated transaction at most
  once per permitted sample, then record its matching timestamp and decision.
  Only `fused` starts a new session and updates `last_fuse_time_us=now_us`;
  an inactive-to-active success also sets `start_time_us`. A rejected startup
  stays inactive, has no invented successful time and may retry later.
- An active session does not repeat startup health/sample-count tests. Current
  field rejection inhibits navigation correction but leaves it active; the next
  good field may fuse immediately. This controller does not emulate EKF2's
  magnetic-only state update. `stop_reason` also describes temporary inhibition,
  so `field_rejected` or `innovation_rejected` can coexist with `active=true`.
- Disable stops activity. On new data, lost readiness stops it; on no-data ticks,
  age **>= twice the maximum interval** stops it. Ordinary stops retain field
  health/count and successful-fusion history; gaps do not fabricate bad field
  checks. Fresh qualified observations can restart without calling `restart()`.
- Apply the unsuccessful-fusion timer only to an active session after the current
  attempt, or when a current field inhibits correction. Measure progress from
  `max(last_fuse_time_us, last_reset_time_us)`. A success at an overdue
  horizon refreshes progress before this test. Elapsed equality is allowed;
  elapsed **> fusion_timeout_us** selects an ordinary `fusion_timeout` stop when
  horizontal aiding is active and the rest flag is false, otherwise a
  `recovery_required` reset request. No-data ticks use their recency branch,
  not a permanent unsuccessful-fusion latch. Inactive startup rejections do not
  acquire an active-session deadline.
- A timeout request grants one `Action::reset` permission only while enabled,
  ready, without horizontal aiding or at rest, on a stable starting frame with
  a fresh accepted sample satisfying startup count/health. A bad field must
  requalify health; no-data ticks revoke permission. Pending recovery does not
  fabricate a correction. Restored horizontal aiding without rest cancels this
  timeout request into an ordinary stop; the next sample may retry fusion.
- After the actual reset, `record_reset(sample_time, succeeded)` consumes that
  permission. Success reactivates the existing session, clears recovery and
  sets `last_reset_time_us=now_us`, retaining actual fusion history. An existing
  session retains its start time; source reinitialization starts a new session
  at the reset time. There is no second correction on that sample. Failure latches `reset_failed` until
  caller-established repair. Wrong/missing permission preserves metadata.
- `failed` feedback requests numerical repair immediately. `fusion_failed` and
  `reset_failed` remain latched even if a new source arrives; replacing a sensor
  cannot repair invalid estimator state/P. Only explicit caller repair permits retry.
- `heading_aligned=false` inhibits ordinary correction and requests a real heading
  reset on a fresh, accepted, health-qualified sample with stable starting
  conditions. It does not bypass a numerical-repair latch or the known-flight
  requirement during source recovery. The default is true for callers that
  already establish yaw alignment. The caller clears lost alignment and reports
  it true again only after actual successful alignment/reset.
  This branch takes precedence over the timeout/reference retry rules:
  restored horizontal aiding does not by itself establish heading alignment.
- Device/calibration changes stop activity, clear source-specific progress/count
  and revoke old feedback. Preserve the field-check failure timestamp (as PX4
  does); four new samples are still required. `filter_reset` is a one-update
  instruction to seed the measurement LPF on the first enabled sample, including
  initial startup. It is false on no-data/invalid calls and reference-only changes.
- Source recovery requires fresh, accepted, health-qualified data, continuing and
  stable-start conditions, and known `FlightState`. A ground switch requires
  heading reset. Stopping an active magnetic session without horizontal aiding
  also latches that requirement. Subsequent takeoff or restored aiding does not
  silently restore alignment. Unknown flight state waits; it is not ground/rest.
  If heading reset is required, grant `Action::reset` and use actual reset feedback.
  Otherwise clear source recovery and grant a normal gated startup attempt; only
  actual fusion activates it. No caller `restart()` is required for this path.
- A reference-only change retains physical-sensor LPF/count/health/progress. An
  active session without horizontal aiding or at rest requests heading reset
  (`reference_changed`); current fresh field qualification remains required.
  Aided motion retains heading and uses the new reference in ordinary gated
  correction. Restored aiding without rest cancels a pending reference reset into
  an ordinary stop/retry. Source recovery and numerical repair take precedence.
  Reference updates are explicit caller-authorized field changes, not coordinate
  frame resets or proof that a WMM/calibration is correct.
- `restart()` acknowledges safe re-entry after any needed estimator repair. It
  clears session/health/count metadata, retains time/source/configuration
  watermarks and requires startup qualification again. Never use it to pretend
  a requested estimator reset ran. `reset()` clears everything for an explicitly
  reinitialized timeline/configuration; an optional seed `Source` binds the identity
  used for initial heading without inventing a measurement or fusion timestamp.
- Invalid/changed durations return `domain_error` before time checks; invalid
  current/sample time returns `out_of_range`; zero new-source ID returns
  `domain_error`; an invalid flight-state enum also returns `domain_error`. Invalid
  updates stop activity/permission but do not consume time/configuration/source or rewrite health/progress history. Invalid feedback
  enum returns `domain_error`; wrong/missing permission returns `out_of_range`.
  Invalid feedback preserves all metadata. Pending reset causes remain visible
  independently of disable/readiness display.

Native tests cover strict health/freshness boundaries, sample count, active versus
startup behavior, data outages, both timeout actions, retry, real AHRS/INS gated
correction, source changes, failure atomicity and unsigned-time extremes.
Scoped ESBMC profiles call the actual constructor/reset/restart, complete
update, fusion feedback and reset feedback. All transitive controller helpers
remain inline. Initialized and inductive invariants cover ordered clocks,
saturated sample count, active-session progress and consumable permissions.
Exact transition/frame checks include invalid inputs, strict boundary equalities,
source/reference priorities and unsuccessful-fusion timing. One-use feedback
follows from permission consumption plus rejection of every missing-permission
call. These integer/Boolean source obligations do not add a separate Lean model.
Physical health, caller repair/condition truth, estimator covariance consistency,
actual adapter sequencing and deployed reset notifications are not proved here.

Source detection and filter/count reset follow PX4 v1.16.2
[EKF2 input](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L2481)
and [magnetic control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp#L66).
This is the initialized, fixed-field AHRS/INS profile: initial heading is supplied
by the caller; external-yaw modes and magnetic earth/bias states are absent.
PX4 can reset/learn those magnetic states while preserving yaw. Here the new
reference changes the observation model and still faces the existing gate; it
cannot absorb sensor offsets into nonexistent magnetic states. Reference adoption
is covered by `R-MAGNETIC-REFERENCE`; deployment lookup/origin policy and
downstream reset notifications remain adapter work.
