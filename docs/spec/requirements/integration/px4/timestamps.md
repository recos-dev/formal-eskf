# PX4-TIME: timestamps and bounded history

**ID:** `PX4-TIME`

**Applies to:** Single-instance PX4 INS module; linked deployment profile bounds apply.

## Basis

[PX4 module architecture](../../../architecture/px4-module.md) identifies the pinned reference, project adaptations and scope.

## Required behavior

Keep these meanings distinct, in one monotonic PX4 time domain:

| Time | Meaning and use |
|---|---|
| Sample time | Physical measurement epoch, with its integration interval where applicable |
| Effective observation time | Sample epoch adjusted once for residual sensor delay and the specified interval reference |
| Latest IMU time | Epoch of the current output state; advances only with admitted IMU intervals |
| Fusion time | Epoch of the main filter state/covariance, behind the latest IMU by the configured history delay |
| Wall time | Scheduling, arrival freshness, output age and detection of stopped inputs |

Each topic binding must name its timestamp field, clock domain, units, sample
point/interval convention and any residual delay correction. Receiver UTC and
publication time must not silently substitute for measurement time. Do not
subtract a latency already compensated by the producer. Validate ordering and
offset arithmetic before unsigned subtraction. Check timestamps against the
arrival clock and declared clock uncertainty; a future observation eligible for
buffering means future relative to the fusion horizon, not arbitrary clock error.

Fix the fusion-lag bounds, active lengths and maximum history capacities at
startup. The retained span must cover the declared sensor latency, arrival jitter
and filter-period quantization. Match IMU, observation and output history by
timestamps; equal ring-buffer indices or capacities alone do not establish alignment.

Bind module timing parameters once at startup and derive active lengths within
compile-time backing capacities. Validate the supported interval mode and the
complete timing/storage configuration before estimation starts. Reject a required
length exceeding capacity; never silently clamp it or resize during operation.
The [timing profile](../../../profiles/px4-sih-250hz.md#startup-parameters-and-fixed-storage)
defines defaults and cross-constraints. Parameter changes require restart.

The IMU-history, [output-predictor](output-predictor.md#r-px4-output-predictor-current-time-output-tracking)
and [observation/context components](observation-history.md#r-px4-observation-history-delayed-observations-and-context)
implement these ownership contracts and are connected to the live module under
[PX4-TIME-LIVE](delayed-fusion.md#r-px4-delayed-fusion-live-module-timeline).
Formal source evidence and target timing/resource acceptance remain open.

At each fusion step, consume at most one observation per source: the newest
unconsumed sample no later than the fusion time and within that source's allowed
age window. Retire older superseded samples. Keep future eligible samples pending;
discard duplicates, out-of-order arrivals and expired samples with a reason.
Never reuse a sample as fresh evidence or refresh successful-fusion time after
rejection. This bounded selection follows PX4's newest-eligible observation
policy; the allowed timing error remains explicit in the deployment profile.

Flight/rest context affecting delayed fusion must also be valid for that horizon.
Current arrival health and delayed fusion eligibility are separate facts.
Qualification/lifecycle calculations receive their declared time explicitly;
wall-time input/output expiry must continue even when no IMU arrives.
If qualification happens before buffering, retain the result and epoch with
the observation. Current safety conditions may inhibit fusion explicitly; they
must not silently rewrite the historical context of a buffered measurement.

Histories have fixed maximum capacity and no steady-state resizing. Capacity
must cover the retention span at the maximum admitted rate, including jitter.
If an observation history overflows, retire its oldest pending observation and
record the loss; it is not a successful fusion. If required IMU/output history
is lost, stop valid estimation and require restart. Do not silently move the
fusion horizon to conceal missing history.

Count uORB generation losses separately from timestamp gaps. A permitted short
gap may use the existing bounded held-rate prediction policy; held intervals
must also reach the output/history path and must never become fresh gravity,
motion-health or other sensor observations. Unsupported discontinuity follows
the restart-required policy.

### History-owner use of the ring buffer

The portable API, storage invariants and bounded-operation contract are owned by
[R-RING-BUFFER](../../runtime/ring-buffer.md).

The caller checks `valid()` during startup and source timestamps before `push`.
Its last-admitted source timestamp survives complete consumption of the buffer;
an empty history does not permit duplicate or backward-time samples. Clear this
watermark only as part of an explicit source/session restart after resolving its
lifecycle and state-reset policy. A buffer reset alone does not resolve that policy.
Before a full-buffer push, it examines `entries() == get_length()` and the oldest
entry's epoch: overwriting already processed retained history is ordinary
rollover; loss of a pending observation must be recorded, and loss of required
IMU/output history must fault the session. The buffer does not distinguish those
roles. Source admission/loss diagnostics are integration obligations, not checks
already implemented by this standalone component. If separate expiry/supersession
counts are needed, the source owner inspects the retained epochs before selection;
`bool` indicates selection only, never fusion success.

The owner supplies already converted effective observation times and a monotonic
fusion horizon, retains matching flight context/qualification, and handles
source/calibration changes. A source-ordered late arrival older than the preceding
fusion horizon may still be used within the current age window. The buffer does
not subtract latency, read clocks or validate payloads. Age uses ordered
subtraction to avoid unsigned overflow; equality with the age limit is expired.
A zero window expires every entry at/before the horizon. Deployment must choose
an explicit positive useful window; 100 ms is PX4's selection window, not a
sensor latency or fusion-delay setting.

#### Use by each history owner

| Owner | Stored history and permitted operations |
|---|---|
| Delayed IMU prediction | Complete admitted integration intervals, with end timestamps and measured durations. Push once per completed main-filter interval and read the delayed end without consuming history. Observation coalescing must never skip IMU intervals. |
| Sensor/context admission | Effective observation epoch, measurement and the qualification/context that belong to it. Select at most one per source per fusion step. A future sample remains pending; a selected but unqualified sample does not become permission to fuse. A measurement's stored context is not overwritten by current arrival context. |
| Current-time output prediction | State snapshots at the completed IMU-history epochs, alongside the separately propagated current output. Read matching historical state and traverse retained entries to apply tracking corrections or state-reset deltas. Modify payloads, retaining timestamps, order and occupancy. |

The session declares its initial filter-state epoch. For the retained-oldest
IMU pattern, warmup requires the configured history length **and** the declared
actual timestamp span before delayed prediction begins. The first interval must
start at the seed epoch; each later interval starts at the last committed state
epoch. Track that epoch independently of the buffer so repeated callbacks cannot
predict the same interval twice. If warmup cannot establish the span before an
unprocessed interval would be overwritten, the profile/input envelope has failed;
do not advance the horizon to conceal that loss. Supported held-rate intervals
follow PX4-TIME; an unsupported discontinuity requires restart.

Output feedback requires a historical output epoch equal to the delayed
IMU/filter epoch. Equal capacities, indices or update counts do not establish
that equality. At startup, do not read empty oldest/newest views or present
default/duplicated samples as measured history. Seed the current state explicitly
and accumulate real timestamped history. Missing matching output history cannot
produce feedback or valid estimation that depends on it.

Distinguish state correction from history invalidation: payload corrections keep
epochs and apply coherently to current and retained output states; session/history
invalidation uses `reset()` and resets the associated cursors/readiness together.
Do not clear slots by assigning `{}` through `operator[]`: that bypasses occupancy
bookkeeping. Iterate only `entries()` retained samples from `get_oldest_index()`
with wraparound by `get_length()`, including during partial warmup.

Per-source capacity and active length come from the declared admitted rate,
retention span, jitter and filter-period quantization. Budget the full backing
capacity using the actual sample types, even for a shorter active length. Keep
histories as estimator-owned members, not large callback-local temporaries.
Reject unsupported startup configurations; changes to timing/length require
restart and requalification. No final target capacities are selected by this
component contract.
