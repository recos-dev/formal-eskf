## Summary

This specification records the required behavior of the numerical core, its supporting operations, portable runtime policies and explicitly scoped replay/PX4 integration paths. A requirement applies only to the layer and configurations named in its definition; a core proof does not automatically cover its runtime or adapter callers.

Each ID identifies a behavior to verify, not a C++ function or a single theorem. Mathematical definitions and assumptions come from papers and design specifications. Detailed clauses define inputs, decisions, success/failure behavior and exclusions. Entries with only a scope description still need those clauses before acceptance; registration does not supply a missing contract.

Evidence mappings belong to the [traceability map](../../formal/agent-review/traceability-map.json); execution results and gaps belong to validated review reports. This list records neither proof status nor completion.

## Reading this specification

| Concern | Requirements |
|---|---|
| State, prediction, correction and covariance | [ESKF core](#eskf-core), with detailed [prediction](requirements/core/prediction.md#e-pred-nominal-prediction) and [composition](requirements/core/composed-steps.md#e-step-atomic-composed-operations) clauses |
| Magnetic observations, health and recovery | [Magnetic aiding](#magnetic-aiding), including explicitly named replay and PX4 module profiles |
| GNSS, height, gravity and IMU/bias policies | [Portable runtime](#portable-runtime), including explicitly named deployment profiles |
| PX4 module target architecture | [Module design clauses](px4-module-architecture.md), covering ownership, execution, timing, lifecycle, output and real-time acceptance; distinct from the currently mapped integration profiles |
| Rotation mathematics | [Quaternion and SO(3)](#quaternion-and-so3) |
| Scalar and linear-algebra dependencies | [Supporting operations](#supporting-operations) |

Requirement IDs remain stable when documentation is reorganized. Child clauses refine their parent requirement; they are not extra passing requirements. Follow the [review criteria](formal-verification-traceability-and-review-criteria.md) for acceptance and change-impact review. Implementation or evidence limitations recorded below constrain claims; they do not waive the required behavior.

## ESKF core

- `E-STATE` — [State, frames and perturbation convention](requirements/core/state.md).
- `E-PRED` — Nominal-state propagation from the prior state, with applicable bias compensation and the selected paper-based integration and attitude models.
- `E-NOISE` — [Discrete process-noise covariance](requirements/core/process-noise.md).
- `E-PCOV` — [Error covariance prediction](requirements/core/covariance-prediction.md).
- `E-INJECT` — [Nominal error injection](requirements/core/injection.md).
- `E-RESET` — [Post-injection covariance coordinates](requirements/core/covariance-reset.md).
- `E-CORRECT` — [Measurement correction and Joseph covariance](requirements/core/correction.md).
- `E-BIAS-UPDATE` — Explicit per-axis INS bias-update permissions applied consistently to the correction vector and Joseph covariance.
- `E-MAG-TILT` — Optional magnetic attitude-gain projection onto navigation Down, with consistent correction and Joseph covariance.
- `E-INNOV` — Componentwise innovation gate and checked, failure-atomic diagnostics, separate from correction.
- `E-OBS` — [Observation conventions and model ownership](requirements/core/observations.md).
- `E-STEP` — Composition of nominal/covariance prediction and correction/injection/reset, preserving component preconditions and committing all outputs only when the whole operation succeeds.

### E-PRED: nominal prediction requirements

See [prediction.md](requirements/core/prediction.md).

### E-STEP: composed-operation requirements

See [composed-steps.md](requirements/core/composed-steps.md).

### E-OBS-ACCEL: gravity observation with fixed bias preprocessing

See [gravity-observation.md](requirements/core/gravity-observation.md).

### E-BIAS-UPDATE: INS bias-update control

See [bias-update.md](requirements/core/bias-update.md).

### E-MAG-TILT: magnetic tilt protection

See [magnetic-tilt.md](requirements/core/magnetic-tilt.md).

### E-INNOV: componentwise innovation gate

See [innovation.md](requirements/core/innovation.md).

## Magnetic aiding

This section groups magnetic runtime policies and their explicitly scoped replay/PX4 callers. The numerical observation and gain-projection requirements remain `E-OBS` and `E-MAG-TILT`; sharing this section does not combine their proof claims.

### R-MAGNETIC-FIELD: instantaneous magnetic-field plausibility

See [field.md](requirements/runtime/magnetic/field.md).

### R-MAGNETIC-HEADING: aided heading consistency

See [heading.md](requirements/runtime/magnetic/heading.md).

### R-MAGNETIC-REFERENCE: geographic magnetic reference adoption

See [reference.md](requirements/runtime/magnetic/reference.md).

### R-MAGNETOMETER-FUSION: field checks and gated magnetic correction

See [fusion.md](requirements/runtime/magnetic/fusion.md).

### R-MAGNETOMETER-LIFECYCLE: magnetic health and fusion lifecycle

See [lifecycle.md](requirements/runtime/magnetic/lifecycle.md).

### R-MAGNETOMETER-FILTER: magnetic data for heading reset

See [reset-filter.md](requirements/runtime/magnetic/reset-filter.md).

### R-MAGNETOMETER-RESET: fixed-field heading and covariance reset

See [heading-reset.md](requirements/runtime/magnetic/heading-reset.md).

## Portable runtime

### R-RING-BUFFER: fixed-storage timestamped history

See [ring-buffer.md](requirements/runtime/ring-buffer.md).

### R-PX4-IMU-HISTORY: delayed IMU timeline

`px4::ImuHistory` owns the retained intervals and the completed prediction cursor
under [PX4-TIME-IMU](px4-timing-profile.md#px4-time-imu-retained-interval-owner).
It does not own the numerical state, output predictor, observations or a clock.

- Validate signed startup settings before conversion. Accept only the implemented
  250 Hz mode and a lag covering source arrival/delay bounds within the fixed
  capacity. Derive the active length; invalid configuration preserves the old
  configuration. Reconfiguration after seeding or fault requires explicit reset.
- Seed from one validated input endpoint and its preceding measured rate; seed
  history remains empty. Reject invalid/repeated seeds without changing state.
- Admit ordered nonzero endpoints, separate valid integration periods, finite
  mean rates and permitted clipping. Check arrival age, source/calibration,
  interval spacing and overlap tolerance before writing history.
- Store the complete real input and a separate preceding-rate hold when the
  permitted gap exceeds tolerance. Preserve the measured periods; a synthetic
  hold is not new sensor evidence. Record the previous endpoint as interval start.
- Wait for the configured number of real intervals and valid actual lag. Never
  overwrite an unprocessed interval or skip it to repair excessive lag. Check
  the prospective retained span before mutation; an admission fault preserves
  history and cursors, latches its cause and blocks further use until reset.
- Selection is non-consuming. Advance the fusion cursor only on completion of
  the oldest eligible interval whose start equals the previous cursor. Wrong,
  premature or repeated completion faults without advancing. Numerical success
  is a caller premise; the timestamp acknowledgment cannot prove it occurred.
- Expose the latest admitted real interval without consuming it, including
  during warmup; return no interval before admission or after a latched fault.
- Reset clears stored inputs, cursors and fault, retaining configuration. All
  storage/work is bounded, with no dynamic allocation or platform effects.

The current evidence is native binary64 testing on Eigen and PX4 matrix, including
default/full active lengths, wrap, faults and timestamp limits. Formal source
proof remains open. `R-RING-BUFFER` does not extend its proof to this owner's
payload/capacity, time arithmetic, acknowledgment protocol, adapter or WCET.

### R-PX4-OUTPUT-PREDICTOR: current-time output tracking

`px4::OutputPredictor` owns current nominal INS output and matching history under
[PX4-TIME-OUTPUT](px4-timing-profile.md#px4-time-output-current-time-output-and-tracking).
It owns neither covariance nor sensor admission, clock, fusion or publication.

- Configure fixed storage and seed a finite state with a valid unit quaternion
  at a nonzero epoch; reject invalid/repeated seed or live reconfiguration without
  mutation. Use the IMU owner's configuration and seed epoch; no synthetic history.
- Consume each admitted IMU interval exactly once, in order. Propagate held and
  real segments using the declared core model and feedback biases; commit the
  endpoint only if both succeed. Never overwrite output awaiting feedback.
- Retain the body angular rate used for the latest real propagation, including
  its bias and tracking correction. Later feedback must not retroactively change
  this rate; failed propagation preserves it and session reset clears it.
- Enforce actual lag bounds and exact delayed/start-epoch pairing. After main
  success, apply the specified attitude-rate feedback, velocity/position PI
  history translations and bias feedback. No interpolation or ordinary-feedback
  reset counters. Main success and valid IMU admission are caller premises.
- Apply a main reset's common left quaternion delta and additive NED deltas to
  all occupied history and current output, preserving epochs and tracking sums.
  A reset before the first interval transforms the seed without creating history.
- Validate all history candidates before committing a correction/reset. On an
  operational fault, preserve current output, retained history, feedback and
  completion cursor, latch the cause, and block use until explicit session reset.
  A session reset clears temporal state and feedback, retaining configuration.
- Use bounded work and fixed persistent storage. Readiness indicates completed
  feedback and no fault; it does not certify freshness, accuracy or safe publication.

Current evidence is native binary64 testing with Eigen and PX4 matrix, including
analytic motion, variable periods, repeated wraps, feedback, reset and numerical
rollback. Lean/ESBMC source evidence, nonlinear stability/error bounds and target
resource bounds remain open. Live binding has its own requirement below.

### R-PX4-SENSOR-POSITIONS: sensor and output reference points

F-05.1 adopts the pinned PX4 v1.16.2 nominal lever-arm compensation. The main
INS and output-predictor states describe the selected IMU. Published local/global
position and odometry describe a fixed vehicle body origin, normally its centre
of gravity. These are different points in the same FRD/NED frames.

- Read `FESKF_IMU_POS_X/Y/Z` and `FESKF_GPS_POS_X/Y/Z` at startup: metres from
  the body origin to IMU and GNSS antenna, respectively, in body FRD. Defaults
  are zero. Reject non-finite or out-of-range coordinates before registration;
  the initial small-vehicle profile permits each coordinate in `[-10, 10]` m.
  This is a deployment bound, not a PX4 requirement or an accuracy guarantee.
  Installation calibration belongs to the airframe integrator. Changes require
  restart; source selection does not silently change calibration or output point.
  A replacement GNSS identity is assumed to use the same configured mounting
  point; a receiver at a different point requires new calibration and restart.
- With `r = gps_b - imu_b`, delayed attitude `R` and the admitted delayed IMU
  rate, subtract `R r` from GNSS NED position and
  `R ((omega_measured - b_g) cross r)` from GNSS NED velocity. Add `(R r)_D`
  to GNSS MSL altitude because altitude has the opposite sign to Down.
  Compensation precedes gating, correction and all GNSS reset paths. Use each
  operation's latest committed prior, consistent with the module's existing
  ordered fusion contract; do not substitute current output attitude/rate.
- Keep the geographic origin at the first accepted antenna fix. Initialize
  IMU position to `-R r` and velocity to compensated receiver velocity. Keep
  MSL and ellipsoid reference altitudes unchanged. Raw receiver quality checks,
  drift checks, buffered data and source metadata remain at the antenna.
- At publication, subtract `R_current imu_b` from current IMU position and
  `R_current (omega_output cross imu_b)` from current IMU velocity. Use the
  latest output propagation rate defined above. All three navigation topics
  use this same point and output epoch; their publication does not mutate the
  retained IMU state. `z_deriv` retains the existing corrected-velocity convention.
- Main resets still transform the retained/current IMU output. An orientation
  reset with a nonzero IMU offset also changes the published point: report its
  actual position/velocity deltas and increment the associated local/global
  reset counters. Accumulate these with translation resets in the same fusion
  step. Do not double count their translation or the odometry reset event.
  A zero IMU offset adds no reference-point reset notifications.
- Preserve receiver noise floors/caps, observation Jacobians, state dimension
  and core covariance equations. As in the reference's nominal preprocessing,
  treat installation offsets as deterministic. Calibration uncertainty and
  attitude/gyro-error correlations introduced by preprocessing are not modeled.
  Published variance fields remain the existing delayed **IMU-state proxies**,
  not covariance transported to the body point or current epoch. Local
  acceleration fields also retain the IMU-point acceleration; angular acceleration
  and centripetal terms for body-origin acceleration are not supplied. These
  limitations require separate F-06.1 review, not an implicit precision claim.
- Keep all transforms fixed-size and allocation-free. Non-finite transformed
  navigation output must not be published as valid. GNSS numerical failures
  retain the existing checked-operation/fault behavior. No terrain, barometer
  lever arm, moving installation geometry or geographic recentering is added.

Native module tests must exercise zero/coincident and nonzero installations,
tilted rotation, gyro bias, startup and normal/reset fusion, altitude signs,
delayed versus current epochs, publication and reset deltas. Independently
differentiate rotating-point positions for the velocity oracle; do not simply
repeat the production cross-product expression. Check actual parameter binding,
invalid inputs and restart-only ownership. Tests and sanitizer runs are empirical
evidence; no new Lean/ESBMC proof or physical installation qualification is claimed.

Reference: pinned PX4 [GNSS velocity/position preprocessing](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp),
[GNSS height preprocessing](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gnss_height_control.cpp)
and [output reference point](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/output_predictor/output_predictor.h).

### R-PX4-FRAME-OUTPUT: geographic and rotation-variance output frames

The module retains a fixed local origin and its current small-area Cartesian
model. Projection accuracy, model fidelity and estimator accuracy are distinct;
F-05.2 in the [functional backlog](../../FEATURE_TODO.md#f-05--sensor-location-and-frames)
tracks geographic-domain validation and the remaining operating-envelope decisions.

- The core's attitude error is body-local. For the delayed attitude/covariance
  pair compute `P_theta_n = R(q_nb) P_theta_b R(q_nb)^T`, retaining all attitude
  cross terms. Publish its Down diagonal as `heading_var` and the sum of its
  North/East diagonals as `tilt_var`. The heading-validity threshold uses the
  same Down variance. Do not combine current predictor attitude with delayed P.
- These fields follow PX4's navigation-frame rotation-variance proxies, not an
  exact Euler-yaw variance or a current-epoch covariance. At large pitch, the
  Euler-yaw Jacobian has a separate singularity; this change does not certify
  heading near that singularity. Odometry orientation variances retain the
  body-local covariance diagonals, as in PX4's `getRotVarBody()` mapping.
- Normalize computed global longitude and the estimated longitude used for WMM
  lookup to `[-180,180)`. This changes the representation only: retain the local
  origin, state, covariance, reference timestamp and reset history. Raw malformed
  receiver coordinates still follow the existing rejection policy.
- The module's horizontal coordinates use PX4 spherical azimuthal-equidistant
  projection, its Down coordinate uses MSL altitude difference, and receiver
  velocity remains local geographic NED. Replay uses WGS84 ECEF projected to
  fixed-origin NED, with ellipsoidal height and transported velocity. These are
  different approximations; do not claim identical coordinates or reuse one
  path's validation as proof of the other.
- No distance-based invalidation, automatic rebasing, ECEF core, gravity model
  or navigation-frame transport dynamics is introduced. Existing freshness,
  health and fault conditions continue to govern output validity outside the
  assessment grid. Those flags do not certify geographic model fidelity. A
  flight-qualified range/latitude/height envelope requires an accepted error
  budget and validation; it is not inferred from a successful map round trip.

Native evidence must cover tilted and correlated body covariance, a separate
current output attitude, heading permission, unchanged body odometry variances,
and both antimeridian crossing directions through publication and real WMM lookup.
Geographic analysis must call the host projection and compare it with independent
geodesy; record source/library identities, grid and numerical tolerances. Finite
grid checks, sanitizer checks and SIH regression are not global bounds, physical
flight qualification or Lean/ESBMC proof.

Reference: pinned PX4 [rotation variance](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L920),
[odometry mapping](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L1689)
and [longitude arithmetic](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/lib/lat_lon_alt/lat_lon_alt.cpp#L117).

### R-PX4-STATUS: confidence and failure reporting

The single-instance module maps its supported capabilities to the pinned PX4
v1.16.2 topics. A sensor being active, an estimate being usable, and a bias having
converged are different facts. Reporting must not change fusion decisions.

- Produce packed control/fault masks and named status flags from the same owner
  snapshot. Use the actual pinned `EKF/common.h` wire layout: vertical-acceleration
  and clipping faults are bits 10 and 11. The older `EstimatorStatus.msg` comment
  is not the wire definition. A generic numerical/history failure is not an
  accelerometer fault; it remains restart-required, stops valid output, clears
  usable solution/bias flags, and appears in module fault/error diagnostics.
- Resolve attitude, horizontal velocity/position, vertical velocity and absolute
  height capabilities independently. Current source generation, freshness and
  estimator health constrain validity. For PX4 controller compatibility local
  `z_valid` and `v_z_valid` both allow vertical velocity **or** height aiding;
  global `alt_valid` still requires height. Delayed source activity flags are not
  substitutes for current usable-solution flags. Rejection never advances a
  successful-fusion timestamp. Full source resets clear the applicable rejection
  and innovation history, without being counted as ordinary corrections. The
  Down-only velocity rescue retains that attempt's velocity diagnostics, as in
  pinned PX4.
- Report horizontal position/velocity uncertainty as `sqrt(P_NN + P_EE)` and
  vertical uncertainty as `sqrt(P_DD)`. Attitude reporting follows
  `R-PX4-FRAME-OUTPUT`. These remain delayed IMU-state covariance proxies beside
  current body-point output, without predictor covariance propagation,
  installation uncertainty, lever-arm correlations or PX4's extra dead-reckoning
  residual inflation. They are not calibrated accuracy bounds. Odometry quality
  remains zero (no percentage score).
- Bias validity requires a healthy estimator, a nonzero sensor ID, finite
  published bias/variance, nonnegative variance components and a variance-vector
  norm in `(0, 0.1]`, using the published float representation as PX4 does.
  Temporary learning inhibition does not invalidate a stored estimate. Keep
  `*_bias_stable` false: this module has no calibration-convergence tracker.
- Summarize each active source's **signed filtered squared** gate ratios. Convert
  our measurement-minus-prediction residual to PX4's opposite sign. At each new
  diagnostic measurement use `dt = clamp(delta_time, 0.001, 1)` seconds and
  `alpha = dt / (dt + 0.5)`, per axis. First/repeated/regressed timestamps seed
  the unsigned raw ratio; a non-finite previous value is reseeded. For advancing
  epochs, raw ratios above 4 limit the filtered result to `[-4,4]`. Missing or
  invalid diagnostics are NaN. Source/calibration changes discard the history;
  a successful full source reset zeros its history at the observation epoch
  (the Down-only velocity rescue retains the attempt's diagnostics).
  Publication alone must never run this filter. The module uses double
  diagnostics; identical arithmetic bits to PX4's float are not required.
- Published summary ratios are `sqrt(abs(filtered_ratio))`, maximizing over the
  relevant axes. Horizontal and vertical velocity preflight checks are separate;
  height averages the available active barometer/GNSS summaries. The preflight
  threshold is strictly greater than 0.5. The heading preflight boolean also
  vetoes missing tilt alignment or unavailable magnetic heading according to
  `OutputValidity::magnetic`, preserving its source/generation, field, consistency,
  age, variance and health checks. Keep `hdg_test_ratio` numeric/NaN independently.
  This is a conservative module-to-Commander compatibility rule: the pinned
  consumer does not inspect our tilt/yaw capability flags, while our GNSS policy
  lacks EKF2's tilt/yaw admission prerequisite. Do not remove this veto merely
  because NaN is not greater than the threshold. Source/calibration replacement
  must remain blocked until heading readiness actually returns. This change does
  not inhibit existing in-air GNSS fusion or introduce a new fusion prerequisite.
  Fusion still uses its existing raw componentwise gate and vertical-limiting rules. Rejection flags describe actual
  rejection, while raw GNSS gate exceedance supplies the solution glitch flag;
  a bounded accepted vertical correction must not be reported as rejected.
- Unsupported airspeed/HAGL/sideslip ratios, magnetic angle/strength telemetry
  and cumulative IMU time-slip diagnostics are NaN, not invented zero values.
  Zeroed legacy LPE health/timeout flags and false unsupported capability bits
  remain unused. Missing numeric diagnostics do not prove good health: consumers
  must also check solution capabilities, topic freshness and source status.
- Status change counters count changes between published snapshots (currently
  10 Hz). They are not a lossless event log and can miss transitions between
  snapshots. Do not claim PX4's on-change event accounting. Filtering is bounded,
  allocation-free and module-owned; no shared-core equation or public API changes.

Native evidence must cover independent capability combinations, actual failure
withdrawal/restart behavior, variance-norm boundaries, packed/named consistency,
raw rejection versus filtered warning, measurement-time filter/reset/source
semantics and duplicate publication. SIH must arm normally with the existing
preflight checks and exercise flight/fault recovery; retain failed runs. Native,
sanitizer and SIH evidence do not establish target timing, calibration accuracy,
formal source correctness or hardware-flight qualification.

Reference: pinned PX4 [topic publication](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp),
[summary/filter/solution semantics](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp),
[vector diagnostic filter](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf.h)
and [wire masks](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/common.h).

### R-PX4-OBSERVATION-HISTORY: delayed observations and context

`px4::ObservationHistory` owns source time admission and retained observations /
context under [PX4-TIME-OBSERVATIONS](px4-timing-profile.md#px4-time-observations-observation-and-context-ownership).
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
Live policy/time binding has its own requirement below. Existing ring-buffer proof does
not prove this owner's provenance, larger concrete payloads or temporal protocol.

### R-PX4-DELAYED-FUSION: live module timeline

The adapter connects the component contracts under
[PX4-TIME-LIVE](px4-timing-profile.md#px4-time-live-module-binding) for the initial
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

### R-GNSS-RESET: checked GNSS position and velocity reset

See [reset.md](requirements/runtime/gnss/reset.md).

### R-GNSS-FUSION: GNSS fusion lifecycle

See [lifecycle.md](requirements/runtime/gnss/lifecycle.md).

### R-MEASUREMENT-FUSION: gated correction of one observation group

See [measurement-fusion.md](requirements/runtime/measurement-fusion.md).

### R-GNSS-VERTICAL-RECOVERY: bounded vertical innovations during IMU faults

See [vertical-recovery.md](requirements/runtime/gnss/vertical-recovery.md).

### R-ACCELEROMETER-MAGNITUDE: filtered gravity-band diagnostic

See [magnitude.md](requirements/runtime/gravity/magnitude.md).

### R-ACCELEROMETER-CONDITIONS: gravity-fusion prerequisites

See [conditions.md](requirements/runtime/gravity/conditions.md).

### R-ACCELEROMETER-FUSION: conditional gravity correction

See [fusion.md](requirements/runtime/gravity/fusion.md).

### R-IMU-DYNAMICS: instantaneous IMU magnitude check

**Retired on 2026-11-14.** The standalone `check_imu_dynamics` API and its
exclusive tests were removed with the timed qualification path. In-tree module
and replay callers use `ImuBiasInhibition`; neither consumed this detector.
The original contract remains in Git history at `7029404`.

### R-IMU-BIAS-LEARNING: bias-learning qualification

**Retired on 2026-11-14.** The `ImuBiasLearning` API, six continuous-good timers
and their exclusive tests were removed. The deployed/replayed decaying-peak
policy remains specified by `R-REPLAY-BIAS` and `R-PX4-IMU-BIAS`; it is not a
behaviorally equivalent replacement for the retired public API. The original
contract remains in Git history at `7029404`.

Both IDs are reserved historical records and excluded from the active
traceability map. Their missing Lean/ESBMC evidence was not completed or converted
to PASS. Retiring these APIs changes supported library scope; it leaves current
module/replay behavior and every retained proof obligation unchanged.

### R-REPLAY-BIAS: replay bias-learning inhibition

See [imu-bias.md](requirements/integration/replay/imu-bias.md).

### R-PX4-IMU-BIAS: module health and bias-learning inhibition

The PX4 module uses the shared `runtime::ImuBiasInhibition` decaying-peak policy
specified in `R-REPLAY-BIAS`, and the detector in `R-VERTICAL-ACCELERATION`.
No separate continuous-good recovery timer is used.

- Before actual IMU prediction, evaluate once with the prior unit attitude/biases.
  Use separate angular/velocity integration durations for their respective peak
  decay. Fixed settings are 25 m/s², 3 rad/s and tau 0.5 s, both bias estimators
  enabled. Replay supplies equal periods through the same interface.
  Invalid durations/data disable permissions and preserve peak history.
- Fresh land-detector `at_rest` is the only rest override. This module has no
  fake-position or fake-height modes; GNSS age does not fabricate these modes.
  Consume the preceding available vertical-health fault, not a future verdict.
  High dynamics or that fault inhibits accel bias only; gyro learning remains
  enabled. These heuristics do not prove physical health or observability.
- Pass the same six permissions to all GNSS position/velocity, barometer/GNSS
  height, gravity and magnetic corrections at that horizon. Reuse the existing
  constrained gain and Joseph update; do not suppress an entire observation merely because
  bias learning is inhibited. Checked navigation/heading resets preserve biases.
- Evaluate vertical health after the current GNSS velocity attempt and before
  current height attempts. Use current active source sessions and latest valid
  selected innovations, including rejected observations, negating core residuals.
  A full velocity reset supplies zero innovation; a Down-only velocity reset
  retains that attempt's selected diagnostics. A height reset excludes its
  pre-reset diagnostics until the next attempt, without erasing raw gate evidence.
  GNSS height and velocity share one reference; barometer supplies another.
- The pinned 250 Hz module profile averages the two IMU periods using 0.99/0.01
  weights and a 2–8 ms input clamp, starting at 4 ms. This is the module cadence,
  not a claim of identical EKF2 clipping/fault timelines at its downsampled cadence.
  Missing/rejected IMU data resets health history and clears learning permission;
  held prediction rates never update peaks or health. Real measured peaks survive
  gaps. Source/calibration discontinuities retain the restart-required policy.
- After initialization, valid accelerometer-clipping flags no longer reject the
  IMU interval: `R-IMU-PROCESS-NOISE` selects its uncertainty before prediction.
  The same flags inhibit the corresponding accel-bias axes and reach the current
  health evaluation, so accepted clipped intervals retain detector continuity.
  Gyro clipping, undefined accel-clipping bits and clipping during initialization
  remain rejected; no prediction/correction occurs on those samples.
- Publish the vertical detector's held fault through `fs_bad_acc_vertical`, not
  the generic numerical-failure latch. Expose learning masks and health availability
  in module status. Disabled learning is neither invalid bias nor a sensor fault.

Process-noise selection is specified separately in `R-IMU-PROCESS-NOISE`.
Bias-value limiting, gravity correction and height-reset exceptions are specified
in their respective requirements and wired into this module. Shared inhibition
has the scoped ESBMC evidence described under `R-REPLAY-BIAS`; this does not close
the module's admission, history invalidation, ordering, forwarding or publication
clauses. Vertical-health source equations and state safety are separately covered
under `R-VERTICAL-ACCELERATION`; that does not prove this module's use of the
verdict. Native backend/correction tests and SITL
are empirical evidence; actual module proof and physical disturbance validation
remain pending.

### R-IMU-PROCESS-NOISE: fault-dependent sampled IMU variance

See [process-noise.md](requirements/runtime/imu/process-noise.md).

### R-BIAS-PROCESS-NOISE: variance-dependent bias driving noise

See [bias-process-noise.md](requirements/runtime/imu/bias-process-noise.md).

### R-BIAS-LIMITS: per-axis bias-value limits

See [bias-limits.md](requirements/runtime/imu/bias-limits.md).

### R-VERTICAL-ACCELERATION: apparent falling and clipping detection

See [vertical-health.md](requirements/runtime/imu/vertical-health.md).

### R-GNSS-QUALITY: GNSS quality checks and health qualification

See [quality.md](requirements/runtime/gnss/quality.md).


### R-HEIGHT-FUSION: PX4 GNSS/barometer height management

`integrations/px4/module/HeightAiding.hpp` owns the two-source deployment policy;
its `HeightBias` type holds separate scalar sensor offsets, outside the 15-state INS.
The module supplies calibrated MSL altitude (GNSS compensated to the IMU under
`R-PX4-SENSOR-POSITIONS`), source identity, qualified GNSS,
ordered IMU time and velocity-aiding activity. The helper returns applied reset
counts/deltas; the module owns and publishes accumulated reset history only
after the height transaction succeeds. uORB, clocks and estimator ownership
remain in the module. No numerical-core equations or state dimensions change.
The caller retains the core's finite-state, unit-quaternion and symmetric-PSD
covariance preconditions; this helper does not prove or recheck positive semidefiniteness.

- Use the pinned v1.16.2 public defaults: GNSS is preferred height reference,
  barometer is enabled, barometer standard deviation 3.5 m, GNSS standard deviation
  `max(epv, 0.75 m)` capped at 10 m when barometer is active, and 5-sigma height gates.
  Process barometer before GNSS, after horizontal/velocity operations. These are
  fixed deployment settings, not live `EKF2_*` parameter bindings.
- With altitude `h = origin_altitude - p_D`, define each offset as `b = z_h - h`.
  The core observes `p_D = origin_altitude - z_h + b` with variance `R + P_b`.
  Freeze the current observation at the prior offset before training the offset
  for subsequent samples. Only active non-reference sources predict/learn their
  offset; the current reference's offset and variance remain frozen.
- Offset estimation uses a scalar Kalman update with a strict 3-sigma gate,
  0.13 m/sqrt(s) process noise density; prediction and accepted updates constrain
  variance between `1e-8` and `max(R, 1e-4)` m². Persistent signed innovations increase process variance by
  1000: signed-ratio LPF magnitude >0.2 or a one-sided sequence >10 s, with a 10 s
  LPF time constant. Prediction and sequence monitoring use the estimator tick
  interval; their cadence is not a claim of bit-identical EKF2 bias evolution.
- Start GNSS only with caller-qualified, finite data; start barometer after more
  than 20 finite samples, with a reset-only altitude LPF (alpha 0.1). These fixed
  warmup/LPF values follow the reference's 200 ms / 10 ms / 90 ms defaults;
  warmup counts selected events, not arrivals or elapsed wall time. Queue admission
  and selection follow `PX4-TIME-LIVE`. Source, calibration or generation changes
  stop that session and clear its offset history; barometer also clears its fault
  latch and repeats warmup.
  Altitude must lie strictly between -12000 and 100000 m, with finite positive
  measurement variance. An invalid initial GNSS altitude delays module startup
  instead of latching a numerical-core fault.
- GNSS start/re-entry resets height/P to its raw qualified altitude, selects GNSS
  as reference and zeroes its offset. Secondary barometer start aligns its offset
  to the current estimate without resetting navigation. Track session startup
  grace separately from actual successful fusion/reset progress. An invalid or
  unqualified observation stops only that source; it cannot claim fusion success.
- Missing-data deadlines are 400 ms for barometer and 1 s for GNSS, equality
  expired. The helper retains its sample-age checks (200/500 ms); the live adapter
  applies the stricter `PX4-TIME-LIVE` admission and selection limits first.
  Future buffered samples wait; duplicate/stale samples cannot refresh progress. When the reference
  stops, select an active GNSS source, otherwise active barometer, otherwise none.
  Fallback selection alone changes no state/P or reset counter.
- After the current height attempt, a source whose progress is >5 s old stops
  if it cannot reset. A failed barometer is fault-latched when another height
  source still has recent progress; ordinary good samples do not clear that latch.
  A reset is required if all height progress is >5 s old OR available health has
  no last-good acceleration or that last-good time is strictly >500 ms old.
  This predicate is shared with `GnssFusion::VerticalAccelerationTimes`; it is
  independent of the 3 s held fault. Height consumes current pre-height health;
  velocity consumes preceding health. Successful bounded fusion may refresh
  progress but cannot cancel the independent acceleration reset. Unavailable
  health alone cannot request reset. Future/inconsistent health times are rejected.
  Only the current reference with a fresh, qualified sample may reset height:
  GNSS uses raw altitude; barometer uses its filtered altitude minus offset. An outage without data cannot authorize a reset.
- A real height reset preserves origin coordinates, decorrelates Down position
  using the existing checked reset primitive and transports every sensor offset
  by the applied Down delta. Barometer also resets only Down velocity to zero,
  variance 10 (m/s)², when no velocity-aiding session is active. With active GNSS
  velocity, the module uses preceding health and shared height progress for its
  checked Down-only reset after the normal velocity attempt. A Down-only reset
  retains that attempt's diagnostics; a full velocity reset clears its innovation.
  Horizontal values and reset counters are unaffected by either Down-only operation.
- Commit the height tick's state/P, history and counters only if all numerical
  operations succeed. Rejection can update diagnostics/lifecycle but cannot act
  as successful correction. Preserve raw gate evidence after a timeout reset;
  reset metadata is separate. Publish applied Down/altitude deltas with opposite
  signs and matching local/global/status counters, plus aggregate odometry reset
  count. Accumulate multiple reset deltas within a tick. Height validity lasts
  at most 5 s after actual height fusion/reset, separately from session activity.

This profile covers GNSS and barometer with a fixed GNSS preference, including
the bounded IMU-fault innovation exception and independent reset timing.
The ground-effect extension is specified below; delayed/current timeline binding
belongs to `R-PX4-DELAYED-FUSION`. GNSS lever-arm compensation belongs to the
module input boundary, not this height manager. Dynamic-pressure correction,
barometer lever arms, range/vision height and configurable reference selection
remain outside this profile. Native/backend tests and closed-loop
SITL are empirical evidence. `HeightBias.lean` proves the real scalar gain,
unclamped posterior covariance (including Joseph equivalence), residual
contraction and offset transport before source-specific re-alignment. It does
not assert that the C++ variance floor always decreases variance.

The `height` ESBMC suite checks actual scalar-offset equations and field frames,
explicit defaults, finite classification, both sources' admission/stop behavior
and read-only timeout/innovation queries. The actual barometer filter/warmup
producer preserves its full field frame and saturates the counter at 21; a
separate IEEE lemma proves the admitted LPF remains within ±131072 m from a
prior within that bound. This bound is a verification invariant, not a new
operational altitude limit. Source branches and both scalar formats
are covered separately. Prediction's internal finite/nonnegative variance,
ceiling and interval premises, and admission's ordered watermark premises must
still be established by the complete public transition. These private producer
checks alone are not an inductive proof of `HeightAiding::update`.

The scoped standard-library models preserve min/max reference selection,
bounded `any_of`, and the two sessions' value construction/copy/index/iteration
semantics. ESBMC 8.4.0's member-name and container initialization/copy defects
are recorded with native controls; no production body is rewritten to obtain
a pass. Model controls do not prove target STL layout, ABI or implementation.
The complete height transaction, history induction, numerical callee composition,
actual module/replay ordering/publication and floating-point error bounds remain
separate obligations. Bias estimates are not independent altitude truth or a
proof of physical sensor health.


#### PX4 module barometer ground effect

F-03.2 follows the pinned v1.16.2 no-terrain branch. The module has no terrain or
height-above-ground estimate; local Down position is not a substitute for HAGL.

- Retain `vehicle_land_detected.in_ground_effect` and vehicle fixed-wing status
  in known-at context history. Apply the policy at the delayed main IMU epoch,
  before height fusion, using original message timestamps with strict 3 s
  freshness. Fresh land state overrides armed-state flight fallback.
- A new land/status message carrying a known positive ground-effect indication
  activates protection only in flight and with fresh non-fixed-wing vehicle
  status. Hold until strictly more than 10 s after its last delayed detection;
  a negative request alone does not immediately clear it. Repeated snapshots of
  unchanged source messages do not renew detection. Landing, fixed-wing state or
  unavailable vehicle/flight context clears the latch; an old detection time alone
  cannot reactivate it. Missing vehicle status is an explicit stricter guard than
  the reference wrapper's default false fixed-wing flag.
- Supply the public `EKF2_GND_EFF_DZ` default **4 m** from the fixed deployment
  profile only to barometer fusion while protected. The internal PX4 parameter
  struct initializes 5 m, but the wrapper binds the public default 4 m. This
  module does not read live `EKF2_*` settings. Configurable tuning remains F-06.2.
- Our NED residual is observed-minus-predicted, opposite to PX4's innovation.
  A transient pressure increase lowers indicated altitude and produces a positive
  residual here. Apply `r_used = max(r_selected - 4 m, 0)` only when positive,
  after the existing bad-acceleration clamp. Negative residuals are unchanged.
  Preserve raw innovation, variance and gate ratio; the deadzone alone never
  authorizes fusion of a rejected observation. A zero selected residual still
  executes covariance correction, matching the reference behavior.
- GNSS, barometer scalar-bias learning, reference arbitration, timeout resets and
  reset counters keep their existing rules. Vertical-health evidence consumes
  the selected residual with the existing sign conversion. Publish protection
  via `CS_GND_EFFECT` / `cs_gnd_effect`, gated by healthy current output; this flag
  is not proof of actual ground proximity or successful barometer fusion.

Reference: [barometer control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/barometer/baro_height_control.cpp#L90),
[mode/timeout](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L945)
and [public parameters](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/params_barometer.yaml#L42).
Native tests cover the actual module path and finite-state timing rules;
`navigation-recovery` covers the scoped residual/correction composition. Physical
rotor-wash response, tuned deadzone, terrain-based activation and whole-module
formal proof remain separate; this does not resolve the retained SIH landing
anomaly from F-04.

## Quaternion and SO(3)

- `Q-IDENTITY` — [Identity](requirements/math/quaternion/identity.md).
- `Q-NORMALIZE` — [Normalization](requirements/math/quaternion/normalization.md).
- `Q-MUL` — [Hamilton multiplication](requirements/math/quaternion/multiplication.md).
- `Q-ROT-COMPOSE` — [Rotation composition](requirements/math/quaternion/composition.md).
- `Q-INVERSE` — [Inverse](requirements/math/quaternion/inverse.md).
- `Q-ROT-MATRIX` — [Quaternion rotation matrix](requirements/math/quaternion/rotation-matrix.md).
- `Q-ROTATE` — [Rotation action](requirements/math/quaternion/rotation-action.md).
- `Q-SIGN` — [Quaternion sign equivalence](requirements/math/quaternion/sign-equivalence.md).
- `SO3-HAT` — [Hat operator](requirements/math/so3/hat.md).
- `SO3-EXP` — [Exponential map](requirements/math/so3/exp.md).
- `SO3-LOG` — [Logarithmic map](requirements/math/so3/log.md).
- `SO3-SMALL-ANGLE` — [Small-angle behavior](requirements/math/so3/small-angle.md).

## Supporting operations

- `F-LINALG` — [Fixed-size linear algebra operations](requirements/math/linalg/operations.md).
- `F-SOLVE` — [Symmetric positive-definite linear solve](requirements/math/linalg/spd-solve.md).
- `F-JR` — [SO(3) right Jacobian](requirements/math/so3/right-jacobian.md).
- `F-SCALAR` — [Checked scalar operations](requirements/math/scalar/operations.md).

Before claiming a requirement, define its applicable configurations, input domain and success/failure behavior. Exact mathematical properties, bounded C++ checks and numerical guarantees remain separate claims under the [review criteria](formal-verification-traceability-and-review-criteria.md).
