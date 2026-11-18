# R-PX4-OBSERVATION-HISTORY: delayed observations and context

**ID:** `R-PX4-OBSERVATION-HISTORY`

**Status:** Draft

**Applies to:** Single-owner PX4 INS module; active deployment profile and stated dependencies apply.

**Parent:** [SW-PX4](../../software/px4-estimation.md).

## Required behavior

`px4::ObservationHistory` owns source time admission and retained observations /
context under [the deployment profile](../../../profiles/px4-sih-250hz.md).
It does not own numerical validation, source qualification, lifecycle or fusion.

- Validate startup timing settings. Seed at a nonzero alignment epoch with known
  context and existing source watermarks; reject invalid/repeated seeds and live
  reconfiguration without mutation. Never replay pre-seed events as new corrections.
- Record one context snapshot per admitted real IMU endpoint. Preserve original
  message times and explicit known flags; reject IMU sequence errors. Future or
  late-arriving messages cannot rewrite past snapshots. Held gaps are not evidence.
- Admit source observations in monotonic raw-time order using explicit arrival
  clocks, per-source age bounds, effective-time spacing and current IMU freshness.
  Check offset subtraction before use. Retain raw/effective/arrival epochs, source
  identity, calibration and frozen offsets; keep watermarks across empty buffers.
- Apply source/calibration invalidation before rate/age rejection: clear only
  that source's pending generation and report the change/loss. Retain a generation
  counter through switch-away/back so identity equality cannot preserve obsolete
  qualification. Keep raw numerical
  and quality metadata for delayed validation; queued does not mean usable.
- Begin fusion only at the next admitted delayed interval, within lag bounds and
  once per latest IMU endpoint. Update the half-period reference from delayed real
  integration periods. Main numerical success remains a caller premise.
- Select each source at most once per fusion step: newest unconsumed event at or
  before the horizon, strictly within the age window. Retire older superseded or
  expired records with counts, preserve future entries, and preserve output on
  a miss. Share the selected GNSS event across all GNSS consumers.
- Resolve context at/before the requested effective epoch using original message
  age. Missing/stale/explicit unknown context cannot grant fusion permission.
  Before prediction, permit a read-only context lookup for exactly the next
  admissible delayed interval, with the same sequence/lag checks as begin-fusion.
  This grants neither observation selection nor a second advance.
- Use fixed bounded storage/work; report aiding overwrites without claiming a
  successful fusion or moving the horizon. Reset clears temporal state while
  retaining configuration. Caller sequencing errors must not partially advance it.

Current evidence is native binary64 scenario testing on Eigen and PX4 matrix,
including timing limits, source switches, independent selection and context
histories. Formal source evidence and target storage/stack/WCET remain open.
Live policy/time binding is owned by [R-PX4-DELAYED-FUSION](delayed-fusion.md). Existing ring-buffer proof does
not prove this owner's provenance, larger concrete payloads or temporal protocol.

### Admission and context details

Use [the deployment profile](../../../profiles/px4-sih-250hz.md) for timestamp
bindings, offsets, ages, capacities and bounded-work limits.

Repeated context timestamps shall not overwrite values or refresh age. Late
messages cannot backfill older snapshots. Reject inconsistent/future seed
watermarks without mutation; transfer qualification separately at the seed epoch.

Rejected observed source timestamps advance the raw watermark; impossible times
or clock regression do not. A nonzero source/calibration change, or device ID zero
at a valid newer timestamp, clears that source's pending history even if the
payload fails age/rate/IMU-freshness checks. Preserve rate watermarks across the
switch and report the discarded count, source change, rejection reason and any
oldest-record overwrite. Source generations advance even for rejected payloads.

At begin_fusion, update the mean period in microseconds as
`0.99 * previous + 0.01 * (angular_period + acceleration_period) / 2` using only
real delayed intervals. Admission freezes `floor(mean_period / 2)` in each event;
queued timestamps never change. Context snapshots use actual IMU endpoints and
original message times, without this half-period offset. This differs from PX4's
system-flag event buffer. Ordinary `context()` lookup cannot exceed the begun
fusion epoch; `prediction_context()` separately permits only the next admissible
delayed interval before prediction. Neither lookup advances selection/history.
Sequence errors preserve state; callers stop rather than skip work.

Queueing preserves raw numerical/quality data, including invalid velocity,
spoofing indicators and non-finite payloads, for delayed validators. It never
advances qualification, alignment, fusion or reset counters. Current failures may
veto use without rewriting past qualification. uORB generation loss is a separate
adapter diagnostic.
