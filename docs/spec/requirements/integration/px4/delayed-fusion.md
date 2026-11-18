# R-PX4-DELAYED-FUSION: live module timeline

**ID:** `R-PX4-DELAYED-FUSION`

**Status:** Draft

**Applies to:** Single-owner PX4 INS module; active deployment profile and stated dependencies apply.

## Required behavior

The adapter connects the component contracts under
[the deployment profile](../../../profiles/px4-sih-250hz.md) for the initial
250 Hz INS/binary64 SIH profile. This is an integration requirement, not a proof
of the scheduler, backend, sensor producers or complete flight system.

- Validate startup timing before callback registration; freeze one configuration
  for IMU, output and observations. Running parameter changes cannot retime history.
- Seed all owners at a real alignment epoch after startup corrections. Admit real
  output intervals during warmup, withholding valid estimates until matching
  delayed feedback succeeds.
- Propagate/publish current attitude first, then predict at most one oldest
  delayed interval. Replay held/real segments in the same order in both predictors.
  Advance the main epoch only after successful prediction; never label failure
  with the newest output epoch.
- Select aiding once per source; share GNSS among velocity, horizontal position
  and height. Use historical event/IMU context for permissions. Source-generation
  changes prevent reuse of obsolete qualification, filter and height-bias history.
- Preserve fusion order and checked reset operations. Transform current/output
  history on each main reset, preserving epochs and reporting deltas/counters.
  Ordinary feedback is not a reset.
- Match exact main/output epochs before feedback/acknowledgment. Faults stop
  subsequent valid publication; an earlier fast attitude is not undone.
- Publish attitude/navigation at latest output time; status/bias/covariance
  diagnostics at delayed main time. Navigation covariance fields are delayed
  proxies. Strictly expire output at 100 ms without fabricating backup propagation.
- Bound storage and per-callback work, separate loss counters and enforce the
  64 KiB owner budget. Phase counters do not establish a wall-time deadline.

Evidence must exercise production `Run`, native uORB decoding/publication,
configuration, warmup, delays, source changes, context, resets and failures, plus
the pinned SIH acceptance scenario. Source formal proof, nonlinear tracking-error
bounds, hardware/resource qualification and flight-fault safety remain open.

### Caller and context binding

`FormalEskf::Run` separates raw arrival caches from selected internal observations.
GNSS quality consumes event-time historical context. Magnetic policy uses selected
event context, or delayed-horizon context when there is no sample. IMU/bias,
gravity and ground-effect policies use delayed IMU context with their own freshness
limits. Do not reconstruct observations as retimestamped uORB messages.
Source-generation changes may veto current validity and delayed use; requalification
runs on a selected new generation, not on arrival. During IMU absence, wall-time
arrival/status work continues without advancing delayed policy time.

Expose both horizons, lag, readiness, occupancy, arrival/rate/staleness losses,
generation discards, overflow, retired events and uORB loss. PX4 clock phase counters
in lockstep simulation do not establish wall-time execution bounds.
