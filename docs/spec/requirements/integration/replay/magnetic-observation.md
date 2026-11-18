# R-REPLAY-MAGNETIC-OBSERVATION: Replay magnetic observations

**ID:** `R-REPLAY-MAGNETIC-OBSERVATION`

**Status:** Draft

**Applies to:** PX4 replay host preparation/execution.

**Parent:** [SW-REPLAY](../../software/replay-aiding.md).

**Uses:** [E-OBS-MAGNETIC](../../core/magnetic-observation.md), [R-MAGNETOMETER-FUSION](../../runtime/magnetic/fusion.md).

## Required behavior

The PX4 replay supplies calibrated body-FRD `vehicle_magnetometer` instance 0
observations in microtesla (Gauss multiplied by 100). Do not repeat sensor
calibration or import the external estimator's learned magnetic bias. By default,
freeze `m_n = R(q_seed) * mean(m_b_seed)` from the same causal 0.2 s alignment window
already used for initialization. This assumes that initial field is suitable;
it is not an independent WMM reference or evidence of magnetic health.
An optional geographic model supplies health checks independently of heading use.
When geographic declination is selected it also replaces the observation reference;
otherwise the seed-derived observation field retains the selected saved/zero
declination. This does not change the fixed-field observation model or estimate mag bias.

Initial declination follows the pinned `getMagDeclination()` source selection:
use an available geographic declination when `EKF2_DECL_TYPE` bit 0 is set;
otherwise use finite nonzero `EKF2_MAG_DECL` only when bit 1 is set; otherwise
use zero (magnetic north, not established true north). Missing source settings
use the explicit mask default 3 and saved-value default 0. Legacy bit 2 does not
select an initialization source; reject other unknown bits. Record the selected
source/default and reject source-setting changes during the replay. No learned
earth-field branch is available. The executable's bit-1 condition takes precedence
over its parameter description, which suggests unconditional saved-value fallback.

- Optional `magnetometer.csv` and `magnetic_reference.csv` must occur together;
  without both, existing navigation/accelerometer replay remains unchanged.
  The reference row supplies NED field, per-axis noise standard deviation,
  gate sigma and `use_model_heading` (0 or 1). The old five-column header remains
  accepted with geographic heading enabled when a model exists. A six-column
  row must match the new header; malformed/non-Boolean mode values fail before
  replay output creation. Without a model the flag has no effect. PX4 preparation
  converts logged `EKF2_MAG_NOISE` to microtesla
  before squaring for `V = sigma^2 I`, and uses logged `EKF2_MAG_GATE`.
- Replay field guards are explicitly 20 microtesla strength tolerance,
  20 degrees inclination tolerance and 5 microtesla minimum horizontal field.
  The first two use the [PX4 v1.16.2 parameter defaults](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/params_magnetometer.yaml).
  The horizontal guard is a replay choice (the default nominal noise sigma), not a PX4 rule or validated heading-accuracy bound. The separate
  `R-MAGNETIC-HEADING` monitor supplies filtered consistency; keep the stateless
  optional heading check disabled to avoid stacking two different policies.
  Retain field checks and the vector innovation gate. Do not retune to match output.
- Optional `magnetic_heading_parameters.csv` supplies heading noise (rad) and
  acceleration limit (m/s^2). Preparation uses logged `EKF2_HEAD_NOISE` and
  `EKF2_MAG_ACCLIM`, reporting fallbacks of 0.3 and 0.5 when absent; legacy CSV
  inputs use those same defaults. Reject in-log changes to these parameters.
  Heading noise also supplies reset variance. Log instantaneous/filtered heading
  innovation, filtered horizontal acceleration, observability and consistency.
  Initial magnetic alignment seeds consistency; this is not independent truth.
  Update acceleration after prediction, using that prediction's bias-corrected
  input and predicted attitude. Held gap IMU values provide no motion evidence.
  Ground/unaided source changes clear heading alignment until a successful reset;
  aided flight preserves the heading history. Reference updates are evaluated
  against the revised reference without reseeding heading consistency.
- Select the newest available topic-timestamped magnetic sample at each IMU
  tick, after navigation and any gravity correction, using the current prior.
  Consume older samples from that tick without fusion and count them as superseded.
  Record the alignment offset; no physical sensor-delay compensation is claimed.
  Only lifecycle permission authorizes `runtime::fuse_magnetometer`, preserving
  `ESKF_MAG_TILT` and the current replay bias-learning permissions. Record outcome/status, field
  failures, valid innovation ratios and lifecycle state/timestamps in
  `magnetometer_updates.csv`. A skipped sample is not a gate rejection.
  No-data lifecycle transitions also produce rows (`source_timestamp_us=0`).
  Rejection continues; numerical failure is logged and terminates replay.
- Optional `magnetic_context.csv` carries the seed context and exactly one row
  per magnetic observation: timestamp, nonzero device ID, calibration count,
  reference revision, causal in-air flag (`-1/0/1`) and NED field. Device/count/
  revision are uint32 values. A changed field requires a changed revision, in the
  same navigation frame; validate field geometry before consuming it. Missing,
  mismatched or extra rows are errors. Without this file retain legacy session
  identity 1/count 0/revision 0, unknown flight state and the seed field.
- PX4 preparation retains published device/count changes after initialization,
  with causal land flags (unknown after 3 s). Reject changes inside the seed
  alignment window, untracked calibration/selection changes and frame/noise/gate/
  declination-parameter changes. Missing identity metadata uses explicitly reported
  session fallbacks; it cannot prove continuity. Without a geographic provider,
  the prepared NED field stays fixed. A changed sensor never establishes a new
  reference field on its own.
- `--compare-mag` runs the same prepared inputs and initialization twice; the
  C++ `--no-mag` path disables magnetic aiding and subsequent reference adoption. Plots report yaw/path
  differences, not accuracy. Existing initialization still uses magnetometer
  data in both runs. Missing-input and disabled paths must preserve the prior
  replay's state/navigation results. KITTI supplies no magnetic input.
