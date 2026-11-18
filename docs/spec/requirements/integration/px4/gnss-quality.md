# R-PX4-GNSS-QUALITY: PX4 GNSS integration

**ID:** `R-PX4-GNSS-QUALITY`

**Applies to:** PX4 module GNSS input and quality binding.

**Uses:** [R-GNSS-QUALITY](../../runtime/gnss/quality.md).

## Required behavior

The SITL adapter in `integrations/px4/module/` consumes the current portable
quality, gating and horizontal lifecycle APIs. Its behavior reference is PX4
v1.16.2 (`54f0455ffcd755534539a7cf33a09a20bf71d29d`), not moving `main`.

- Use the fixed reference defaults: all ten quality checks, 10 s initial health,
  and strictly more than 1 s of good checks after a failure. Source identity
  changes reset qualification, drift continuity and horizontal sessions.
- Convert receiver accuracies as standard deviations, PDOP as `hypot(hdop, vdop)`,
  and spoofing `MULTIPLE` as the spoofed flag. Skip invalid velocity. Reject
  malformed geographic metadata; never relabel that failure as a bad fix.
  Drift uses an adjacent-fix `MapProjection` and MSL altitude differences.
  Fresh land-detector `at_rest`/`landed` supply independent motion context;
  fresh armed state supplies the in-air fallback, as in the reference wrapper.
- During alignment, process at most one new GNSS sample per usable IMU tick
  with the existing sample-time freshness check. After seeding, admit and select
  observations under `PX4-TIME-LIVE`; stricter arrival/selection limits apply
  before GNSS quality. One selected event serves velocity, position and height.
  Effective sample time drives drift; the delayed IMU horizon drives qualification
  and lifecycle using historical motion context. Duplicate/stale events cannot
  refresh health or aiding. GNSS sample-time binding differs from the pinned
  reference's publication-time binding, as recorded in the timing profile.
- Initialize only after quality acceptance. With initialized attitude/frame,
  execute the horizontal lifecycle every prediction, including missing-data
  ticks. Velocity precedes horizontal position. Each gate uses the latest
  committed prior; rejection of one group does not reject other groups.
  Use the reference's 0.5 m / 0.3 m/s noise floors, 10 m horizontal noise cap,
  2.25 Down velocity variance multiplier, and 5-sigma gates. These are fixed
  SITL settings, not reads of live `EKF2_*` parameters or calibrated tuning.
- Execute checked velocity/position resets at first start, re-entry and permitted
  fusion timeout. Reset acknowledgments and actual fusion feedback remain distinct.
  Preserve local-origin coordinates. Publish successful reset deltas/counters in
  local position, horizontal reset count in global position, aggregate count in
  odometry and corresponding estimator-status counters. Never invent a reset
  notification after rejection/failure. Publish individual quality failure bits.
- GNSS is the sole horizontal source in this profile. The 7 s stop and coupled
  7/14 s reset rules follow `R-GNSS-FUSION`; renewed quality is required after
  stopping. Separate aiding/validity/session status: after 1 s without horizontal
  position OR velocity progress report dead reckoning; after 5 s withdraw
  horizontal position/velocity validity. These do not prematurely stop the 7 s
  session. Alignment persists through loss of magnetometer freshness.
  Height is managed separately by `R-HEIGHT-FUSION`; the GNSS-only height
  lifecycle remains disabled. Its shared height-progress timeout now authorizes
  the Down-only velocity reset after normal velocity correction and before height
  processing. Height does not refresh horizontal progress.
  The vertical-acceleration exception is governed by `R-PX4-IMU-BIAS` and
  `R-HEIGHT-FUSION`. Lever-arm preprocessing follows `R-PX4-SENSOR-POSITIONS`;
  no emergency yaw or receiver blending is claimed.

PX4 compilation, existing native runtime tests and closed-loop SITL with quality
failure, outage and recovery checks are empirical evidence. This adapter has no
Lean/ESBMC proof and does not establish complete EKF2 equivalence.
