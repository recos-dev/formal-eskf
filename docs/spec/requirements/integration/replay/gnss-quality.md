# R-REPLAY-GNSS-QUALITY: Replay GNSS quality

**ID:** `R-REPLAY-GNSS-QUALITY`

**Applies to:** Host replay input preparation and same-horizon execution.

**Uses:** [R-GNSS-QUALITY](../../runtime/gnss/quality.md).

## Required behavior

PX4 replay evaluates the existing monitor before its independent navigation
gates. Follow [v1.16.2 GNSS checks](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_checks.cpp)
and [health qualification](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp).
The separate [GNSS lifecycle](gnss-lifecycle.md) consumes these quality results and orchestrates
checked GNSS resets. Delayed buffering remains absent. The seed is an
initialization assumption, not qualified fusion.

- Read `EKF2_GPS_CHECK` and `EKF2_REQ_*` thresholds from the log. Map the ten
  mask bits to named checks; do not silently enable/disable checks. Support the
  legacy `EKF2_REQ_GDOP` name for PDOP. Use recorded `EKF2_REQ_GPS_H`, or explicitly
  record the reference's 10 s default when that parameter is absent in old logs.
  Raw speed limit remains 100 m/s. Reject known parameter/receiver changes;
  absent identity metadata does not prove receiver continuity.
- Preserve every post-seed message, including bad fixes. Pair navigation and
  quality CSV rows by exact timestamp. Fix, satellite count, reported accuracies,
  `hypot(hdop, vdop)` and receiver-local NED velocity feed the monitor. Malformed
  quality numerics must be rejected, not sanitized into valid data. Invalid
  geographic coordinates stop preparation rather than fabricate a position.
- Match the source adapter's `vel_ned_valid=false` skip before monitor evaluation.
  Such a row cannot authorize fusion or refresh health. Position-reference history
  advances only for velocity-valid messages. Drift uses the preceding fix's
  spherical azimuthal-equidistant projection (radius 6371000 m); Down uses the
  MSL altitude difference. This is separate from the trajectory's fixed-origin
  ECEF/NED projection. The first displacement after monitor reset is ignored.
- Obtain `in_air = !landed` and `at_rest` causally from land-detector messages
  less than 3 s old, not estimator outputs or GNSS velocity. Never infer rest
  from landed. In air, rest is unnecessary; on ground, selected motion checks
  require known rest. Missing required motion metadata blocks quality and resets
  qualification; this explicit incomplete-log policy is not an EKF2 transition.
- Map reported spoofing state `MULTIPLE(3)` to the spoofed flag as in EKF2.
  Missing spoofing metadata blocks qualification only when that check is enabled.
  Preserve availability diagnostics; missing metadata is not a measured healthy
  receiver. No jamming detection is added.
- Only quality `accepted` enters the existing gate/correction calls. Waiting,
  rejection, unavailable metadata and invalid velocity do not change state/P
  through GNSS or refresh horizontal aiding. Prediction and other sensors still
  run. Numerical quality rejection continues; malformed configuration/time/CSV
  or numerical correction failure stops replay. Count quality skips separately
  from innovation rejections. Log outcomes, failure flags, diagnostic validity
  and health times in `result/gnss_quality.csv`.

Replay control flow uses the named `ReplayGnssQuality::Outcome` values. Convert
them to the existing CSV labels only when writing output. `evaluated()` derives
whether the monitor consumed the sample from its quality decision; this is
separate from receiver receipt, so unavailable metadata and invalid velocity
retain their different lifecycle meanings.

Without both quality files, legacy/KITTI input retains its previous behavior
and reports quality OFF. A partial pair is an error. Native and replay checks
support this integration; new Lean/ESBMC proofs and flight tuning remain pending.
