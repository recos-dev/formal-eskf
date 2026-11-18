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

These caller-invoked INS operations replace navigation estimates; they are not
Kalman corrections or `E-RESET`'s post-injection change of error coordinates.
The navigation reset operations and GNSS horizontal-variance policy belong to
`runtime/navigation.hpp`, not to the generic numerical correction. Inputs are already aligned to the prior's
local NED frame, body/IMU origin and time; quality, alignment and reset authority
remain caller responsibilities. No clock, lifecycle or automatic retry is owned.

Behavior reference: PX4 v1.16.2, revision
`54f0455ffcd755534539a7cf33a09a20bf71d29d`:
[GNSS reset dispatch](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp),
[position reset](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp),
[altitude reset](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/position_fusion.cpp),
[GNSS height control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gnss_height_control.cpp),
and [velocity reset](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/velocity_fusion.cpp).

| Operation | Nominal state | Covariance |
|---|---|---|
| `reset_gnss_horizontal_position` | Set `p_N,p_E` to the supplied 2-vector; leave `p_D` and all other state fields unchanged | Clear rows/columns 0 and 1, then set both diagonals to `max(variance_N + variance_E, 0.01^2)` in m^2 |
| `reset_vertical_position` | Set only `p_D` to the supplied scalar; leave velocity, horizontal position, attitude and biases unchanged | Clear off-diagonal entries in row/column 2; set `P(2,2) = max(variance_D, 0.01^2)` in m^2; preserve every other entry |
| `reset_vertical_velocity` | Set only `v_D` to the supplied scalar in m/s; preserve all other nominal fields | Clear off-diagonal entries in rows/columns 2 and 5; retain `P(2,2)`; set `P(5,5) = max(variance_D, 0.01^2)` in m^2/s^2; preserve every other entry |
| `reset_velocity` | Set all three components of `v_n`; leave position, attitude and biases unchanged | Clear off-diagonal entries in rows/columns 0..5. Preserve position diagonals 0..2; set velocity diagonals 3..5 to `max(variance_i, 0.01^2)` in m^2/s^2 |

Vertical position and velocity resets are source-independent numerical operations.
The GNSS prefix is retained only for horizontal position, whose variance-sum
policy is receiver-specific. Source permissions and acknowledgment stay in callers.

The horizontal variance sum follows `resetHorizontalPositionToGnss` calling
`resetLatLonTo`; it is not the per-axis policy of PX4's other position-reset
overload. Velocity reset also decorrelates position, even though position values
and variances do not change. Untouched covariance entries are copied exactly.
The input variances describe diagonal observation noise, not standard
deviations or a full correlated measurement covariance. Noise preprocessing,
including any vertical-velocity scaling, happens before this call.
The Down-only velocity reset follows `resetVerticalVelocityTo`: decorrelate
height as well as vertical velocity, while retaining height value/variance and
both horizontal velocity values/variances. It reports `new v_D - old v_D`.

Vertical position is already converted to the existing local NED origin. For
an altitude-relative Down coordinate with a fixed, common datum,
`p_D = h_origin - h_GNSS`, so `p_D_new - p_D_old` agrees with PX4's
`-(altitude_new - altitude_old)`: climbing gives a negative Down delta.
An ECEF-to-NED adapter must instead supply its projected Down coordinate
consistently; it is not generally identical to altitude difference over distance.
This API does not accept absolute altitude or initialize an origin.
Its covariance rule follows `resetAltitudeTo`, which
does not change velocity. PX4's velocity controller can separately reset `v_D`
when height recovery is required; that policy is not part of this operation.
For GNSS-reference height reset, PX4 uses the prepared GNSS measurement variance,
not the height-bias variance added to its ordinary correction diagnostics. This
API neither estimates nor changes sensor-height biases, terrain or output history.

- All nominal coefficients, P, observation and variances must be finite; otherwise
  return `non_finite_input` before arithmetic. P must be exactly symmetric with
  nonnegative diagonal, and input variances nonnegative; violations return
  `domain_error`. Zero/tiny nonnegative variances receive the explicit floor.
  Symmetric PSD P and a unit prior quaternion remain caller premises; the checks
  are not a PSD test, attitude repair or arbitrary-corruption recovery.
- Compute reset delta as new minus old, in NED meters or m/s. A non-finite delta
  or horizontal variance sum from finite inputs returns `non_finite_result`.
  Check all results before publishing state, covariance or delta.
- On success publish all three outputs. On failure preserve all three, including
  distinct sentinel outputs. Complete state and covariance outputs may each
  independently alias their respective prior. All other input/output storage is
  disjoint. No quaternion normalization, injection, solve or allocation occurs.
- Unlike PX4's internal reset helpers, this checked boundary rejects negative
  or non-finite variances instead of clamping negatives or using NaN to retain
  covariance. The finite valid-input reset behavior above is the aligned scope.
- Return a delta per successful call, not a fusion success, clock update or
  counter increment. The caller must separately acknowledge completed recovery
  and aggregate/publish reset notifications. `R-GNSS-FUSION` supplies
  reset-on-start/timeout orchestration separately. These operations do not perform
  origin changes, output-predictor adjustment, height-reference/bias management,
  automatic height-failure decisions or yaw reset.

Lean expresses navigation covariance reset as `D P Dᵀ` plus nonnegative
replacement diagonal terms. It proves PSD preservation, untouched/decoupled
coefficients, each operation's variance policy and retained nominal attitude/biases.
The real model uses exact decimal `0.01²`; the C++ proof checks each scalar format's
encoded constant and operation order. These are separate evidence layers.

ESBMC verifies the actual input validator for observation sizes 1/2/3, a scalar
lemma connecting finite zero subtraction residuals to exact equality, and all four
reset callers in binary32/binary64 with every independent whole-prior state/P alias.
The height owner additionally has exact `solve_proof::Backend` profiles for its
size-one validator and in-place Down-position/Down-velocity resets; this avoids
assuming equivalence with the original opaque-math backend.
The same-type validation summary is input-pure and allows every returned status;
callers check its exact arguments and invocation. State/delta behavior, complete
covariance decorrelation/floors, overflow errors, failure atomicity and preserved
borrowed inputs are checked. An arbitrary bounded row/column quantifies every
covariance output, not selected sample entries. Comparisons retain values, signed
zero and NaN classification; NaN payload preservation is not claimed. Public invalid
inputs are not assumed away; source/frame correctness, PSD and unit attitude retain their specified
caller ownership. No target-backend proof, floating-point error bound or machine-
checked Lean-to-C++ refinement is claimed. Native tests and mutation controls
complement these proofs. Actual caller permission, acknowledgment and publication
remain separate; `E-RESET` covers a different operation.

### R-GNSS-FUSION: GNSS fusion lifecycle

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
implemented. Continuous-acceleration recovery follows the criterion above;
bounded innovation exceptions are specified in `R-GNSS-VERTICAL-RECOVERY`. Existing seed,
sequential gates and numerical models are retained.

### R-MEASUREMENT-FUSION: gated correction of one observation group

`runtime::fuse_measurement` supports INS/AHRS and accepts `r = z - h(state)`, `H`, full measurement covariance `V`, an explicit sigma threshold and the existing normalization threshold. The caller supplies one prior, zero error mean, valid frames/time, unit quaternion, symmetric PSD `P` and symmetric SPD `V`, as for `correct`. INS additionally accepts the same optional per-axis bias permissions.

`runtime::check_measurement` evaluates the same gate without attempting correction. Its `MeasurementCheck` reports status, diagnostic validity and rejection; `passed()` requires successful, valid, non-rejected evidence. It owns no history and grants neither source permission nor permission to reuse a prior linearization after the estimate changes.

- Check finite nominal fields and sigma, then call the numerical core's `correction_covariances`. This operation checks the correction inputs in their existing order and publishes `PHt` and `S` together only on success; its outputs are disjoint from each other and from inputs. It does not inspect nominal state, solve for gain or perform injection. Compute `PHt = P * transpose(H)`, then `S = H * PHt + V`, with the same finite checks and symmetry cleanup as correction. Gate with `r` and `diag(S)`, never `diag(V)`. Rejection is componentwise, not a joint NIS test; gate errors retain `E-INNOV` statuses.
- Return a fresh `FusionResult` with `decision = fused`, `rejected` or `failed`. Only `fused` publishes both state and covariance. `rejected` has `status = success`; any numerical/input failure has `decision = failed` and its failing status. Neither rejection nor failure modifies either output. Each complete output may independently alias its respective prior; all other input/output storage is disjoint.
- Only after the gate succeeds, publish `innovation`, `innovation_variance` and `test_ratio` with `diagnostics_valid = true`, including rejected observations. These diagnostics describe the call's prior and remain valid if correction later fails. Before gate success, diagnostics are invalid zero placeholders; never reuse a prior call's diagnostics.
- If any ratio exceeds one, reject this whole observation group without correction, LLT or injection. Otherwise call the existing correction with the identical state, `P`, `r`, `H`, `V`, normalization threshold and bias permissions. The gate and correction use the same covariance implementation. Accepted calls intentionally recompute the products: correction remains independently checked, without a retained workspace or caller-supplied cached gain. GNSS may use a limited residual, and magnetic correction retains its own projection/check order. Equations and validation order are unchanged. A gate pass does not certify full covariance validity, conditioning or prior quaternion norm. Rejection does not certify estimator health.

The named runtime APIs form their residual from the supplied state and reuse the existing core Jacobians:

| API | One rejection group |
|---|---|
| `fuse_horizontal_position` | NED north/east position |
| `fuse_vertical_position` | NED down position, not altitude or HAGL |
| `fuse_horizontal_velocity` | NED north/east velocity |
| `fuse_velocity` | All three NED velocity components |

These models preserve the corresponding core API's units, full within-group noise correlations and bias permissions. Non-finite observations return `non_finite_input`; residual overflow from finite observations/state returns `non_finite_result`. Use horizontal OR 3D velocity for a given sample, not both.

Each call is independent: a rejected or failed group neither undoes previous fusion nor automatically rejects later groups. After a successful group, form the next residual/Jacobian and gate using the newly committed state/covariance; after rejection/failure retain the prior. No fixed sensor order or whole-GNSS transaction is imposed. Separate groups require uncorrelated measurement noise across groups, not zero state cross-covariance; use a joint model if cross-group noise correlations must be retained. Alignment, GNSS quality, source lifecycle, persistent-rejection recovery, clocks and last-fusion timestamps remain caller/runtime work.

Native tests cover INS/AHRS, binary32/binary64, legal aliases, grouped rejection, input/arithmetic and downstream failures, bias permissions and a cross-coupled sequential-fusion case. They do not establish flight suitability. The `fusion` ESBMC suite checks the public covariance-preparation frame, actual portable check/fuse control flow and these four named wrappers: binary32/binary64, INS/AHRS where applicable, observation sizes 1/2/3, every legal state/covariance alias and symbolic INS bias permissions. It composes with the same-backend `E-INNOV`, correction and injection/reset producers; arbitrary inputs and failing intermediate results remain admitted. The source assertions cover skipped correction on rejection, diagnostics retained on downstream failure, unchanged inputs and atomic publication. Existing Lean gate/correction theorems are mathematical dependencies, not a new proof of the runtime state machine or a machine-checked Lean-to-C++ refinement. Replay GNSS settings/wiring below, caller history/timing, external backends and numerical error bounds remain open. See the [verification scope and evidence rules](../guides/verification.md).

#### Replay GNSS gating

PX4 preparation supplies one `gnss_parameters.csv` row from logged
`EKF2_GPS_P_GATE`, `EKF2_GPS_V_GATE`, `EKF2_GPS_P_NOISE`, `EKF2_GPS_V_NOISE`
and `EKF2_NOAID_NOISE`. Gates are clamped to at least one sigma at the adapter
boundary, not in the core. Require finite settings, positive noise values and
safe finite squared variances. Reject recorded setting/lever-arm changes during
the replay; require equal configured GNSS/IMU offsets because compensation is
not implemented. Missing required parameters are errors, not guessed defaults.

The noise profile follows the ordinary GNSS path of
[PX4 v1.16.2](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp)
under this demo's sole horizontal/height aiding assumptions:

- Horizontal position: square `max(min(max(eph, position_noise),
  position_noise_limit), 0.01)` for both axes.
- Height: square `max(epv, 1.5 * position_noise)`, without a noise cap or a
  height-bias estimate/variance. GNSS is the fixed sole height reference.
- Velocity: square `max(speed_accuracy, velocity_noise, 0.01)` for N/E;
  use 1.5 times that standard deviation for Down. All covariances are diagonal.
- Initialize position/velocity covariance with these same noise rules;
  retain existing attitude/bias initial uncertainties. Receiver accuracy axes
  are treated as the fixed navigation axes, a local-flight approximation.

Consume each navigation event once at the following IMU tick. Fuse 3D velocity,
horizontal position, then height through the existing runtime APIs. Each group
forms its gate and correction from the latest committed prior. This differs
from PX4's precomputed position/velocity diagnostics and scalar update schedule;
neither identical decisions nor numerical EKF2 equivalence is claimed.
Rejection preserves that group's prior and does not block other groups. Only
actual horizontal-position or 3D-velocity correction/reset refreshes horizontal-aiding
time; height operations, Down-only velocity reset, receipt, rejection and failure
never do so.

`gnss_updates.csv` records each group's outcome/status and per-axis innovation,
S diagonal, test ratio, V diagonal and diagnostic validity. Numerical fusion
failure is logged before stopping. Invalid prepared input stops replay as an
input error. Event counts are not successful-correction counts; report accepted
and rejected groups separately. A fully rejected event still counts as consumed.
Quality qualification is described below; the lifecycle above controls permission
and GNSS reset execution, including height and Down-only velocity recovery.
`R-GNSS-VERTICAL-RECOVERY` supplies the bounded vertical-innovation exceptions.
Delay compensation and changes to magnetic/gravity scheduling are not included.
Without the parameter file, retain legacy/KITTI ungated noise and fusion order.
Replay checks are integration evidence, not new Lean/ESBMC proofs or accuracy claims.

### R-GNSS-VERTICAL-RECOVERY: bounded vertical innovations during IMU faults

`runtime/navigation.hpp` provides GNSS velocity fusion and
source-independent NED Down position fusion (`fuse_height`). Generic gates and
ungated correction APIs are unchanged. The caller must first establish source
quality/lifecycle permission, frame/time, unit quaternion, symmetric PSD P and
SPD measurement noise, as for ordinary
fusion. A fault never grants source permission or requests a reset.

| Observation | Additional condition to release a rejected gate |
|---|---|
| 3D GNSS velocity | Bad vertical acceleration, Down ratio > 1, both N/E ratios strictly < 1, receiver speed accuracy strictly below its configured maximum |
| Down position (GNSS; also qualified barometer in the module) | Bad vertical acceleration and rejected height gate |

Only the vertical innovation is constrained to `[-gate_sigma * sqrt(S_D),
+gate_sigma * sqrt(S_D)]`. Preserve its sign, horizontal residuals, H, V and
bias-update permissions; call the existing checked correction with that selected
residual. Do not re-gate the rounded boundary value. Ordinary gate equality is
accepted, but horizontal equality prevents the velocity exception. Receiver
accuracy is its reported standard deviation in m/s, not the noise-floor-adjusted
variance; its strict exception bound applies even when the general quality
speed-accuracy check is disabled. Both accuracy values must be finite, accuracy
nonnegative and maximum positive, regardless of the fault flag.

Height fusion additionally accepts a finite nonnegative `deadzone` in metres,
zero by default. After the IMU-fault limit, transform a positive selected residual
`r` to `max(r - deadzone, 0)`; retain zero/negative residuals. Never recompute or
release the gate because of this transform. Nonfinite/negative settings fail
before gate/correction and preserve outputs. `R-HEIGHT-FUSION` defines when the
module may supply a nonzero value. `innovation_limited` continues to describe
only the acceleration-fault clamp; `fusion_innovation` includes both transforms.

`NavigationFusionResult` retains raw innovation, S and test ratio in `fusion`, and
separately exposes `fusion_innovation` and `innovation_limited`. Raw ratio > 1
can therefore accompany successful limited fusion. Selected innovation is valid
only with `diagnostics_valid`; only `decision == fused` means it was applied.
`check_gnss_velocity` returns `NavigationInnovationCheck` without correction, for an
active source's quality-rejected sample. Its `gate` member owns status, diagnostic
validity and effective gate rejection; it cannot report successful fusion.
Gate/numerical errors retain their existing statuses; invalid accuracy returns
non_finite_input/domain_error. Rejection or any failure, including
LLT/normalization after limiting, preserves both outputs.
Independent whole-prior state/P aliases are supported; other storage is disjoint.

Replay enables velocity recovery only with a quality/lifecycle profile, using
its existing maximum-speed-accuracy setting. It consumes the preceding valid
health verdict for velocity, then evaluates current velocity/preceding height
health before attempting height. Both health caches use selected innovations;
quality-rejected velocity updates evidence without correction. Missing clipping
or a broken IMU interval clears health, so no stale fault releases a gate.
Multiple due events retain the existing sequential estimator updates and the
single-monitor-transition batching rule in `R-VERTICAL-ACCELERATION`.

`gnss_updates.csv` adds the selected innovation and group-level limit flag;
summary counts limited successful velocity/height fusions separately. Rejected
quality/lifecycle conditions still skip correction and do not refresh progress.
Horizontal position has no exception. Existing timeouts/reset behavior is
separate from the gate exception; the independent 500 ms last-good-acceleration
reset is specified in `R-GNSS-FUSION`.

Behavior reference: PX4 v1.16.2
[velocity exception](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp#L209)
and [height exception](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/position_fusion.cpp#L36).
Alignment covers predicates, limiting and fault-consumption order. Our grouped
Joseph correction and latest-prior diagnostics differ from PX4's scalar update
schedule; replay cadence/batching and missing-data safeguards also differ.
Native backend and replay tests are evidence for these contracts, not flight
accuracy or full EKF2 equivalence. `NavigationRecovery.lean` proves exact-real
clipping bounds, sign, idempotence, selected-axis isolation and the resulting
ratio bound. This is not a rounded-boundary re-gating guarantee.

`navigation_recovery.cpp` checks actual gate diagnostic production, the actual
limiter and both public recovery callers in binary32/64 with all independent
state/P alias modes. The limiter retains raw diagnostics, uses the selected
variance for its root request and handles arbitrary scalar-library returns.
The public callers preserve strict policy boundaries and publish only after
successful correction. The height caller's symbolic deadzone covers disabled,
positive and invalid values, ordering after the limit, unchanged raw gate evidence
and all independent state/P alias modes. Same-type covariance/correction contracts and the
separately checked exact limiter relation bound the composed caller proof;
they are not proofs of external backend internals. Shared IEEE scalar lemmas
establish nonnegative finite limits and sign/bounds under an explicit finite,
nonnegative-root premise. Root accuracy, machine-checked real-to-C++ refinement,
external source permission and actual module/replay health ordering remain
separate obligations.

The PX4 module connects the same checked velocity/height entry points.
Velocity consumes the preceding detector verdict and receiver-reported speed
accuracy; quality/lifecycle failure never grants fusion or reset. A numerically
valid quality-rejected sample from a still-active velocity source updates only
the gate/health diagnostics, not state/P or progress. Invalid metadata clears
that cache. Both GNSS and barometer convert their qualified altitude/offset
observations to scalar NED Down before using the existing bounded-height path.
That historical GNSS-named entry point contains no receiver policy. No horizontal
position exception is added. Raw test ratios remain visible even after a limited
fusion; health consumes the selected innovation with the core-to-monitor sign
conversion. Last-good timing separately drives `R-HEIGHT-FUSION` recovery.

### R-ACCELEROMETER-MAGNITUDE: filtered gravity-band diagnostic

`runtime::AccelerometerMagnitude<Linalg>` filters body-frame specific force in
m/s^2 before estimator-bias subtraction. The caller supplies
`delta_vel / delta_vel_dt`, not an integrated increment. It owns only one filtered vector and
its initialization flag; no timestamps, estimator state or covariance.

- The first valid sample seeds the vector directly. Later samples use
  `filtered += alpha * (sample - filtered)`, then compute `squared_norm(filtered)`.
  Filter the vector, not its magnitude or a peak envelope.
- Keep alpha fixed at `0.010 / (0.090 + 0.010)`, evaluated in the selected scalar
  format. Do not recompute it from each sample's dt. The nominal 90 ms time
  constant assumes 10 ms updates; actual cadence belongs to the caller. This
  component does not resample, enforce that cadence or promise 90 ms at 250 Hz.
- With `g = 9.80665 m/s^2`, report `within_gravity_band` only when the computed
  squared norm is strictly between `(0.9*g)^2` and `(1.1*g)^2`. Equality fails.
  A finite out-of-band sample still updates filter history successfully.
- Non-finite input returns `non_finite_input`; a non-finite filtered vector or
  squared norm returns `non_finite_result`. Failure preserves initialization and
  filter history, with a fresh false band flag and invalid zero diagnostic.
  Successful zero input is not an error. `reset()` clears history and requires
  the next valid sample to seed again, including after source/frame/calibration
  changes. The filtered-vector accessor is meaningful only after initialization.
- This result is not fusion permission, sensor health or evidence of rest.
  Persistent horizontal acceleration can pass; body rotation can reduce the
  filtered magnitude. No bias subtraction, normalization, correction, motion
  policy, clipping logic, at-rest override or innovation gate is added here.

The valid-input recurrence, initialization, fixed coefficient and strict band
follow PX4 revision `e1c28e27550114e85ddb8760ab3355eaa7a89d8a`:
[coefficient settings](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/ekf.h#L650),
[initial nominal interval](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/estimator_interface.h#L358),
[sample updates](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/ekf.cpp#L162),
[magnitude check](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/aid_sources/gravity/gravity_fusion.cpp#L50).
Checked failure behavior follows this project's scalar contract. This is not
alignment with the older v1.16.2 gravity-fusion peak envelope, a claim of complete
PX4 fusion equivalence, or a backend-independent bitwise equivalence claim.

`tests/test_accelerometer_magnitude.cpp` checks both scalar formats, recurrence,
startup/reset, strict band boundaries, vector-before-magnitude behavior and
failure preservation. Separate Lean theorems prove the real affine recurrence,
constant-input preservation, convex bounds, one-step residual contraction and
exact residual after any number of constant-input updates. These do not prove
that the signal is gravity or bound floating-point error.

ESBMC checks the actual constructor/reset, exact scalar-format filter/norm/band
expressions, failure statuses and complete history/input frames in binary32 and
binary64 with the fixed-array proof backend. Finite history is established by
construction/reset and preserved inductively by every public update, including
failures; successful diagnostics are finite and nonnegative. Accessor identity
is checked. Target backend arithmetic, cadence and physical interpretation remain
outside this source claim.

### R-ACCELEROMETER-CONDITIONS: gravity-fusion prerequisites

`runtime::check_accelerometer_fusion_conditions<Linalg>` consumes caller-supplied
enable, tilt-alignment, horizontal-aiding, rest and per-axis clipping flags,
together with the actual current `AccelerometerMagnitude::update` result.
All inputs describe the same sample/fusion horizon. Availability of a GNSS fix
is not the same as active horizontal aiding. The caller owns those facts; this
function detects neither rest nor alignment and owns no history.

- On a successful magnitude evaluation, `active = enabled AND (within_gravity_band
  OR at_rest) AND NOT horizontal_aiding AND tilt_aligned`.
- `sample_eligible = active AND NOT any_axis_clipped`. Clipping blocks this
  sample without clearing the otherwise active mode. Rest bypasses only the
  band check, not disable, missing alignment, horizontal aiding or clipping.
- A failed magnitude evaluation propagates its status and returns both flags
  false, even at rest. This is the project's checked-input behavior, not a claim
  that PX4 handles invalid arithmetic identically. Do not substitute a previous
  successful result after the current update fails.
- The function is stateless and leaves every input unchanged. Neither flag
  means fusion succeeded, nor establishes a physically valid gravity observation.
  `sample_eligible` is a pre-innovation result; no innovation gate, correction,
  state/P update, timer, sensor selection or recovery logic is introduced.

The separation of mode activation and sample clipping follows
[PX4 e1c28e2 gravity fusion](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/aid_sources/gravity/gravity_fusion.cpp#L56).
Its additional innovation rejection remains a separate deferred task here.
The same predicate supports INS and AHRS; any configuration without horizontal
aiding supplies false for that flag. Native tests enumerate all 256 boolean
combinations with all declared statuses, and exercise real magnitude-result
consumption in both scalar formats. ESBMC additionally checks the actual
stateless predicate for all eight flags, every status and arbitrary unused IEEE
diagnostics, including input purity, in both formats. Rest cannot bypass failed
magnitude evaluation or clipping; horizontal aiding prevents activation. No
separate Lean state-machine theorem is needed for this predicate. Actual
rest/aiding producers and sample/horizon correspondence retain their own
obligations; the portable fusion composition is covered separately below.

`TiltAlignment` has its own initialized inductive proof: finite nonnegative
stored variance and a monotone alignment latch, with both fields preserved on
failure. ESBMC checks the actual source in both formats, consuming the exact-type
rotation and 3x3 product frame contracts. Observers bind `R_nb * P_theta_b * R_nbᵀ`,
all operands and call order; the real transpose and all finite/negative checks
remain inline. Success stores precisely the N/E diagonal sum and applies the
strict encoded `(pi/60)^2` threshold. Numerical outputs may be arbitrary IEEE
values, so overflow/nonfinite/negative-result paths are retained. Lean separately
proves the real tilt projection's nonnegativity, trace decomposition/bound and
identity-frame formula under PSD covariance. It does not establish the truth
of covariance-based attitude confidence, floating-point PSD, or target libraries.

#### Replay condition sources (host demo)

`examples/replay_conditions.hpp` supplies conditions for the replay's existing
single navigation source. It does not change correction equations or implement
the full GNSS qualification/reset controller. `fusion_conditions.csv` records
raw-tick conditions; interval observations and correction outcomes are recorded
separately as specified under `R-ACCELEROMETER-FUSION` below.

- Derive tilt uncertainty from this estimator, never a logged EKF status:
  `P_theta_n = R_nb * P_theta_b * R_nb^T`, using INS covariance rows/columns 6..8.
  Latch alignment when `P_theta_n(0,0)+P_theta_n(1,1) < (3*pi/180)^2`.
  Evaluate tilt on the predicted prior before that tick's navigation corrections.
  A new replay resets the latch; subsequent variance growth does not clear it.
  A constructed quaternion alone is not alignment. Unit attitude and PSD
  covariance remain caller premises; non-finite/negative computed tilt variance
  is a replay error, not permission to fuse.
- Horizontal aiding starts only after replay horizontal correction succeeds,
  remains active between navigation events and expires when elapsed time since
  the last successful event reaches 7 seconds. A resumed successful event
  reactivates it. Initialization is not a fusion event. This is limited to the
  existing fail-fast replay: every accepted navigation event fuses position and
  velocity or ends the run. Its successful-event timer is not a substitute for
  independent source lifecycle, quality qualification or recovery in deployment.
- PX4 preparation takes explicit `vehicle_land_detected.at_rest` at or before
  each IMU timestamp, with age strictly below 3 seconds. Absent/stale data is
  unknown (`-1`) and disables only the rest override; `landed`, constant speed
  and a gravity-sized acceleration are not substitutes. Unlike the delayed
  system-flag buffer, this undelayed demo checks freshness on each IMU tick.
  The external detector's flag is not independent physical ground truth.
- Decode `sensor_combined.accelerometer_clipping` bits X/Y/Z from that sample.
  Missing clipping stays unknown (`-1`), not zero. No complete fusion conditions
  are returned until clipping is known. Optional `imu_flags.csv` must match
  every IMU timestamp exactly, including the seed; malformed/missing/extra rows
  are errors. An absent sidecar leaves flags unknown, preserving legacy/KITTI
  input compatibility without inventing metadata.

Behavior references at PX4 `e1c28e2`:
[tilt latch](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/control.cpp#L72),
[NED tilt variance](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/ekf_helper.cpp#L932),
[active horizontal sources](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/estimator_interface.cpp#L675),
[GNSS missing-data stop](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp#L115),
[7-second limit](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/common.h#L512),
[rest metadata](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF2.cpp#L2849),
[clipping metadata](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF2.cpp#L737).
The local-error covariance transformation and missing-metadata refusal are
project integration behavior, not copied estimator status. This is native-tested
condition preparation, not proof of physical alignment or full PX4 equivalence.

### R-ACCELEROMETER-FUSION: conditional gravity correction

`runtime::fuse_accelerometer` composes `R-ACCELEROMETER-CONDITIONS` with the
unchanged `E-OBS-ACCEL` correction for AHRS and INS. No innovation gate is used.

- Supply the current body-frame specific force in m/s^2, not the LPF vector or
  delta-velocity. INS subtracts its prior bias exactly once inside the core;
  AHRS consumes externally bias-calibrated input. Preserve gravity magnitude,
  the existing Jacobian and caller-supplied effective V in m^2/s^4. Forward
  INS per-axis bias-update permissions unchanged; omission retains all enabled.
- The caller updates its magnitude filter once for each sample, independently
  of fusion enablement, and passes the actual result for that sample with
  conditions and prior state at the same fusion horizon. The caller owns
  timestamps, cadence and filter reset on source/calibration/frame changes.
  This stateless composition neither updates nor rolls back filter history.
- Evaluate conditions first. A failed magnitude result returns `failed` with
  its status. Unmet conditions return `rejected` with `Status::success` without
  calling correction or certifying its unused inputs. Otherwise call the
  checked correction: propagate failure status, and report `fused` only on
  success. Return the condition check separately; eligibility can remain true
  even if correction fails. No innovation diagnostics or test ratio is implied.
- Publish both state and covariance only on `fused`. Rejection or failure
  preserves both outputs. Each output may independently alias its respective
  complete prior; all other storage is disjoint. Separate inputs are unchanged.
  There is no retry, persistent mode, timer, source selection or reset policy.

Native tests cover real magnitude results, current-versus-filtered measurement
use, INS bias permissions, both configurations/scalar formats, all four alias
arrangements, rejection and numerical failures. ESBMC checks both actual public
fusion overloads and their inline condition predicate, binary32/binary64 and
all four independent whole-state/P alias modes. It composes the exact-type
`E-OBS` / `E-BIAS-UPDATE` correction frame: arbitrary success outputs and every
status, no writes on failure. Assertions bind every original request/output
argument, all six INS bias permissions, exact invocation count, result conditions,
status/decision, publication and all borrowed/disjoint input frames. No successful
or finite input premise removes rejected/failed paths. This control composition
needs no additional real-algebra theorem. Sample integration, magnitude-result
freshness and actual module/replay wiring remain separate obligations.
The composition does not establish a valid physical gravity observation,
noise independence, tuned V, full external-estimator equivalence or flight safety.

#### Replay sampling and tuning

| Setting | Selected replay baseline |
|---|---|
| Gravity magnitude | `g = 9.80665 m/s^2` |
| Effective per-axis observation standard deviation | `sigma = g` |
| Observation covariance | `V = g^2 * I_3 = 96.1703842225 * I_3 m^2/s^4` |
| Nominal observation/LPF interval | `10,000 us`; actual intervals must be reported |

PX4 `e1c28e2` uses normalized observations, defaults
[`EKF2_GRAV_NOISE`](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/params_gravity.yaml)
to `1.0`, and squares it for observation variance. Multiplying that variance by
`g^2` matches the local tangential information scale near gravity-sized specific
force, not the complete normalized nonlinear update. This is an effective
pseudo-observation weight, not measured hardware noise or validated flight tuning.
It is separate from prediction's `ACCEL_VAR`; do not multiply V by `dt` or `dt^2`.
The MATLAB reference's enabled `accel_correct1` uses normalized observations and
`V_accel = 0.7 * I_3`, corresponding to `67.31926895575 * I_3` on this same scale;
its cross-product preprocessing is not part of the current model.

The PX4 [nominal EKF period](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/module.yaml#L15)
is 10 ms. Its [IMU downsampler](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/imu_down_sampler/imu_down_sampler.cpp)
accumulates rotation-aware increments over whole input samples; it does not just
select the latest raw sample every 10 ms. The selected target is not a promise of
exactly 100 Hz or an instruction to fuse each roughly 250 Hz raw input. Fixed LPF
alpha remains `0.1`; its effective time constant depends on actual update intervals.

`runtime::AccelerometerSample` prepares non-overlapping observations; replay and
the PX4 adapter use it directly. It does not change prediction/navigation timing
or add a delayed fusion buffer:

- For each real sample, form `dtheta = omega_m * dt`, `dv = f_m * dt` and
  `D = R(Exp(dtheta))^T`. Accumulate in the latest body frame as
  `dv_sum_new = D * dv_sum_old + (dv + D * dv)/2`. This is an endpoint rotation
  approximation, not an exact continuous-time/sculling integral. No estimator
  bias is subtracted here; INS correction subtracts its prior bias once.
- Track mean input period `T` starting at 5 ms, using `T += 0.1*(dt-T)`.
  At each interval start set `N = max(1, round(10 ms/T))`, target `N*T` and
  minimum `max((N-1)*T, T/2)`. Emit once the sample count reaches N and duration
  exceeds the minimum, or duration exceeds the target. All decisions use the
  actual integration periods; no interpolated, repeated or overlapping samples.
  Timing rules follow the referenced downsampler, not bitwise PX4 equivalence.
- On emission, divide accumulated dv by the actual interval duration and update
  `AccelerometerMagnitude` exactly once, even when fusion is unavailable/rejected.
  Correction consumes that current interval average, not the LPF vector. Partial
  final intervals are not flushed as shorter observations. Non-finite input,
  invalid periods/masks or arithmetic failure preserves sampler/LPF history and
  ends this fail-fast replay, without publishing an observation.
- OR all constituent clipping masks; any unknown makes interval clipping unknown.
  Rest and alignment/aiding conditions use the emission tick. Gaps discard partial
  increments and LPF history; resume with real samples after the gap. Prediction
  holds are never accumulated as measurements. A new replay also resets history.
- After that tick's navigation corrections, call `runtime::fuse_accelerometer`
  only when interval clipping is known; its conditions decide whether to correct.
  This uses the agreed V and the current replay bias-learning permissions. A failed
  correction ends replay. Record each observation, actual duration, aggregated
  clipping, LPF diagnostic and outcome (`fused`, `rejected`, `metadata_unavailable`)
  in `accelerometer_updates.csv`; report counts and actual interval statistics.

Missing clipping still blocks correction, unknown rest disables only its override,
and innovation gating remains deferred. The existing legacy ULog and KITTI inputs
lack clipping and therefore do not exercise successful accel correction. These
starting settings and native tests are not calibrated flight parameters or new
Lean/ESBMC evidence. Prediction/measurement reuse correlation, interval-averaging
model error and full PX4 estimator equivalence remain outside the claim.

The PX4 module shares this sampler and `runtime::TiltAlignment` with replay.
`FormalEskf::fuse_gravity` runs after navigation corrections and before magnetic aiding.
It receives actual gyro/accel integration periods: sampler angular rate is scaled
by their ratio, preserving measured delta-angle while averaging force over the
accel period. Fresh `vehicle_land_detected.at_rest` (at most 2 s old, not future)
provides the rest override; missing/stale rest is false. Horizontal aiding is the
actual GNSS position/velocity lifecycle activity, not merely a valid fix.
Tilt alignment latches when the NED roll/pitch variance sum is below (3°)²;
the estimator owns this latch independently of the gravity sampler and enablement,
and module alignment flags expose it. Clipping is ORed across intervals.
Rejected IMU samples and small gaps clear partial samples and LPF history;
synthetic holds never enter the sampler. Existing restart-required identity,
calibration and large-gap faults terminate the session before further sampling.

`FESKF_GRAV_EN` defaults to 1 and is read only at startup. It requests the existing
eligibility/correction path, not unconditional fusion. Disabled sampling still
updates the LPF. The current-horizon module uses the same g² I weight and no
innovation gate. Each successful correction applies the shared bias limits;
rejection preserves state/P. Status distinguishes emitted samples from successful
corrections and publishes the latest emission interval and gravity mode. Mode is
not a successful-fusion count. The pinned PX4 gravity controller supplies the
condition reference; its normalized sequential/gated update remains a declared
difference. Native tests, flight A/B evidence and formal-proof gaps remain distinct.

The adapter consumes tilt history before sampling, and a completed interval/LPF
update before correction. A later failure does not roll back those earlier
stages. `reset_samples()` clears partial increments, LPF history and active mode,
but preserves the tilt-alignment latch. After successful sampling, disabled,
horizontally aided or unaligned contexts clear active mode even on partial
intervals; otherwise it is held until an emission. The module admits only known
0–7 clipping masks and valid integration periods before this private phase;
unsupported metadata rejects the IMU and resets observation history. The shared
sampler still supports unknown metadata for replay, where it blocks correction.
Known clipping preserves an otherwise active mode while rejecting the sample.
These module history boundaries are checked through production gravity and
`Run()` paths in `integrations/px4/tests/ModuleTest.cpp`. Sampler cadence,
unknown metadata and failure preservation remain checked across native backends
in `tests/test_accelerometer_sample.cpp`. Neither establishes a whole-tick
transaction or closes the pending runtime Lean/ESBMC obligations.

Sampling proof scope: Lean proves the discrete endpoint rule's identity-rotation,
zero-increment, zero-history and transported-sum identities. ESBMC checks actual
10 ms sampler construction/reset and its integration helper in both scalar
formats: exact dt scaling, Exp threshold/status, two inverse-action arguments,
trapezoidal expression, finite-result check, output atomicity and all history/input
frames. Same-type Exp and inverse-action pure frames allow arbitrary numerical
returns; this does not prove rotation accuracy or exact continuous integration.
The inverse input frame is independently checked on its actual implementation.
The 10 ms public sampling owner now has same-type compositional proofs of all
result/history fields, strict readiness, unknown-clipping propagation, child-call
ordering and failure atomicity. Initialization establishes a reachable-state
superset; every update preserves finite vector history, average period in
[1, 50000] us, bounded adaptive thresholds and counters with no integer wrap.
The actual clear body verifies round/cast/count and target/minimum equations;
a separate double-precision producer verifies nearest-integer rounding including
positive half ties. Vector division is composed from the actual same-backend
F-LINALG producer; arbitrary IEEE quotient replies include non-finite failures.
Integration and magnitude producers supply the remaining same-type contracts.
The proof is for `IntervalUs=10000`; it does not certify other template periods,
exact 100 Hz cadence, target libm accuracy or module/replay sequencing. See the
[verification scope and evidence rules](../guides/verification.md).

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

Replay directly uses the shared `runtime::ImuBiasInhibition`. It evaluates
the prior biases and unit attitude once before
prediction, then supplies the same six `Ins::BiasUpdate` permissions to every
GNSS, gravity and magnetic correction at that horizon. It uses decaying peaks
without a separate continuous-good timer. Learning inhibition
does not control prediction noise; `R-BIAS-PROCESS-NOISE` supplies that independent
policy. Core equations and bias-preserving resets remain unchanged.

- Supply angular and force integration periods explicitly; replay passes the same
  logged period for both. For synchronized periods `dt > 0`, compute
  `decay = 1 - clamp(dt / tau, 0, 1)` and independently update each envelope as
  `peak = max(norm(measurement - prior_bias), decay * previous_peak)`.
  Specific force retains gravity. Initial peaks are zero. High dynamics means
  either peak strictly exceeds its configured limit; equality permits learning.
  No startup wait or additional clipping-recovery timer is imposed.
- Gyro permissions follow the gyro-bias enable switch. Accel permissions require
  its enable switch, no high dynamics, no bad-vertical-acceleration fault, and
  no clipping on that axis. Rest overrides only the observability restriction;
  otherwise fake height inhibits all axes, and fake position permits only axes
  with `abs(R_nb(2,axis)) > 0.966`. These are actual mode/fault inputs, never
  substitutes inferred from GNSS age or sample availability.
- Validate finite inputs, positive finite limits/tau/dt and clipping bits 0..7.
  Invalid input or non-finite arithmetic returns disabled permissions and leaves
  envelope history unchanged; replay terminates on failure. A new session starts
  fresh. Missing IMU intervals do not erase peaks or supply held measurements;
  only real samples decay the envelope using their logged integration period.
- Preparation exports logged `EKF2_ABL_ACCLIM`, `EKF2_ABL_GYRLIM`, `EKF2_ABL_TAU`
  and the two enable flags in `bias_parameters.csv`. New logs use
  `EKF2_IMU_CTRL` bits 0/1; legacy `EKF2_AID_MASK` bit 2 disables accel bias,
  with gyro bias enabled. Parameter changes within the session are rejected.
  CSV inputs without this file use explicit 25 m/s², 3 rad/s, 0.5 s defaults
  with both bias estimators enabled. The 0.5 s value is a decay time constant.
- Preserve the three-column `imu_flags.csv`. In the replay input adapter only,
  missing accel clipping assumes clear **for bias learning**, with an explicit
  diagnostic/count. Gravity fusion still sees unknown clipping and is blocked.
  Unknown rest does not supply a rest override. Gyro clipping is not an input to
  this inhibition rule. Reject recorded IMU selection/calibration/frame changes;
  absent metadata cannot prove continuity or physical sensor health.
- The replay implements no fake-position/height modes, so their flags are false.
  `R-VERTICAL-ACCELERATION` supplies the preceding valid IMU horizon's fault to
  bias inhibition. A fault disables all accel axes; gyro permissions remain
  independent. Unknown clipping or an IMU gap invalidates that verdict before
  learning is evaluated. Unavailable health adds no fault inhibition under this
  replay compatibility profile; this is not evidence of physical sensor health.
  `bias_learning.csv` records metadata, assumptions, peaks, permissions and the
  health verdict actually consumed; counts describe IMU ticks, not corrections.

Behavioral reference: PX4 v1.16.2
[IMU bias inhibition](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L971),
[learning limits](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/params_accel_bias.yaml),
and [legacy parameter semantics](https://docs.px4.io/v1.13/en/advanced_config/parameter_reference#EKF2_AID_MASK).
This aligns the inhibition rule under supplied conditions, not all EKF2 behavior:
replay uses its own IMU horizons and precision. Its variance-dependent bias noise
selection, fault-dependent IMU noise and bias-value limits are specified below.
GNSS overrides/resets and fake-aiding modes remain pending. Native tests support
this integration. Scoped binary32/binary64 ESBMC checks now establish the shared
policy's constructor, exact peak decay, finite/nonnegative peak invariant,
permissions, failure atomicity and input purity. The actual norm/rotation producers
are included, with arbitrary scalar-root returns (not an accurate-libm claim).
The caller uses the separately proved peak invariant and checks every callee
premise, including distinct integration periods. ESBMC's incorrect `std::max`
tie model is replaced locally by an explicit standard-library contract model;
this does not prove a deployed STL. Replay metadata parsing, same-horizon
sequencing and forwarding remain open. No Lean-to-C++ refinement or physical
observability claim is made. See the dated verification-extension record.

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

`runtime::select_imu_process_noise(configured_noise, conditions,
fault_variance, output)` selects body-axis accelerometer measurement variances
before INS prediction. Conditions contain the preceding vertical-health fault
and known clipping flags for the current IMU interval.

- Replace an axis's `specific_force_variance` with `fault_variance` when the
  vertical fault is set or that body axis is clipped. Otherwise retain its
  configured variance. A vertical fault therefore selects all three axes.
  This is replacement, not `max(configured, fault)`; it can lower an unusually
  large configured value. Preserve gyro variance and both bias-driving vectors.
- Inputs are variances in m²/s⁴, already squared. Validate the fault variance
  first, then all twelve configured noise coefficients, including replaced or
  unselected values. Non-finite values return `non_finite_input`; negative values
  return `domain_error`. Zero is valid. No arithmetic is required on the noise
  values. Failure preserves output; success permits a whole-noise input/output
  alias and otherwise preserves input. No state, P, timer or permission is changed.
- Start every step from original configured noise. Clearing clipping restores
  that axis immediately unless the held vertical fault still selects all axes.
  The caller owns metadata availability and fault timing. Correction's bias
  permissions are independent: disabled learning does not disable this selection.
- Apply `R-BIAS-PROCESS-NOISE` **after** this selection so its accel bias variance
  threshold uses the selected measurement variance. Its driving intensity retains
  `intensity * dt` scaling. Prediction retains `E-NOISE`/`E-PCOV`: sampled IMU
  variance contributes with `dt²`, and body-axis noise maps through `R(q)`.
  No rotation or time scaling is duplicated in this selector.

Behavioral reference: PX4 v1.16.2 [covariance prediction](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/covariance.cpp#L113)
and [bad-accelerometer noise constant](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/common.h#L83).
Despite its reference name, `BADACC_BIAS_PNOISE = 4.9 m/s²` is a sampled-acceleration
standard deviation here, not bias random-walk intensity. Replay sets
`BAD_ACCEL_STD = 4.9` and passes its square (approximately 24.01 m²/s⁴); nominal
accel/gyro variances remain 1 m²/s⁴ and 0.0004 rad²/s². Alignment covers the
selection rule, constant and ordering, not all covariance equations, precision,
IMU sampling rates or calibration of a particular sensor.

Replay uses the preceding valid vertical verdict and current clipping before
prediction. With unknown clipping it retains configured measurement noise,
records unavailability and does not feed assumed-clear flags to this selector.
Synthetic gap holds likewise use configured noise without reusing previous or
future clipping. These are explicit replay fallbacks, not EKF2 behavior or
proof of healthy data. Known clipping still selects noise without a GNSS health
profile; unavailable vertical-health evidence remains separately reported.
`bias_learning.csv` records the three selected accel variances for actual IMU
ticks. Summary records the fault standard deviation, per-axis override counts
and unavailable predictions (including gap holds). Override counts describe
selection, even if configured and replacement variances happen to be equal.

The PX4 module uses `px4::predict_imu` for actual accepted IMU intervals.
It copies configured parameters, selects noise with fixed `4.9^2` fault variance,
applies `R-BIAS-PROCESS-NOISE`, then calls atomic INS prediction in place.
Configured noise and gyro variance are preserved. Selection/prediction failure
preserves state/P and feeds the existing numerical-fault latch.

The module first checks timestamps, periods, finite increments and continuity.
It accepts only the three defined accel-clipping bits, only after initialization;
gyro clipping remains rejected. Before prediction, both noise selection and bias
permissions consume the preceding health verdict plus current clipping. After
the GNSS velocity attempt, current clipping reaches the health detector before
height fusion. A newly detected held fault therefore affects the next prediction.
Clearing clipping restores configured noise unless the held fault remains set.
Synthetic gap holds retain configured IMU variances and do not consume either adjacent
sample's clipping or update health/peaks. The existing health-history reset on
gaps is unchanged. This fallback and gyro rejection are explicit module limits,
not assertions of EKF2 equivalence. Clipping flags do not reconstruct lost motion.

Native tests cover all selector choices, aliases, actual rotated covariance,
module clipping admission/prediction, restoration, gyro variance and bias-noise thresholds,
failure rollback and detector/bias-policy sequencing. Replay retains its existing
metadata behavior. The `imu-policies` ESBMC suite verifies the actual portable
selector for binary32/binary64, every Boolean condition and both legal output
aliases, including all invalid inputs and first-error/publication behavior. Lean
proves preservation of nonnegative noise and exact prediction PSD under the
existing covariance assumptions. Detector decisions, physical clipping/calibration,
module/replay sequencing and target numerical behavior remain separate obligations;
normal-flight SITL does not establish clipped-flight accuracy. The additional
`imu-prediction` suite verifies actual `ImuPrediction.hpp` clipping admission and
composition: a fresh parameter copy, exact bit decoding/fault variance, selection
before bias inhibition using prior P, one in-place prediction and first-error
rollback. Same-type actual selector producers and E-STEP supply its explicit
callee frames; arbitrary callee statuses/results are admitted. This is helper
wiring under the proof backend, not the actual PX4 Matrix instantiation, input
scheduler, health timing, fault latch or output publisher. See the
[verification scope](../guides/verification.md).

### R-BIAS-PROCESS-NOISE: variance-dependent bias driving noise

`runtime::inhibit_bias_process_noise(P, step_noise, output_noise)`
selects INS process noise for one prediction. Its rule follows PX4 v1.16.2
[covariance prediction](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/covariance.cpp#L145):

| Body axis i | Keep the configured intensity only when |
|---|---|
| Accelerometer bias | `P(9+i,9+i) < specific_force_variance(i)` |
| Gyroscope bias | `P(12+i,12+i) < angular_rate_variance(i)` |

- Otherwise set only that bias intensity to zero. Preserve both IMU measurement
  variance vectors and every other bias axis. No new threshold parameter is
  introduced. Zero variance/intensity is valid; equality inhibits the increment.
- The INS bias transition rows are identity and sampled measurement noise has
  no bias rows. Thus prior bias diagonals equal their propagated values before
  bias noise, including with nonzero cross-covariances. Reading the prior avoids
  a second prediction. This is scoped to this INS model, not arbitrary dynamics.
- Validate all twelve noise coefficients and six consumed bias diagonals as
  finite/nonnegative, even on inhibited axes. Non-finite input returns
  `non_finite_input`; negative input returns `domain_error`. No universal
  precedence is promised for multiple independent invalid inputs. Other entries
  of P are not read here; valid symmetric PSD covariance remains the prediction
  caller's premise. Failure preserves the whole output, including an alias of
  configured noise; success permits that alias. State/P are never changed here.
- Start every step from configured noise, apply `R-IMU-PROCESS-NOISE`
  when fault/clipping metadata is available, then apply this selector. A correction
  that reduces variance below the selected threshold releases inhibition immediately. This rule takes
  no learning permissions, motion/clipping or enable flags directly; fault and
  clipping influence its thresholds through the supplied IMU variances.
- This is a pre-increment check, not a clamp: one permitted increment may cross
  the threshold. The next step inhibits further bias noise while variance stays
  above it; an already-high covariance is not reduced. No hard covariance bound,
  physical sensor-health claim or bias-value limit follows from this rule.

The threshold quantities have matching units: m²/s⁴ for accelerometer bias and
sampled specific force, rad²/s² for gyro bias and sampled angular rate. Preserve
`E-NOISE`: the selected continuous driving-noise intensity is multiplied by `dt`.
PX4's bias-driving parameters instead have units m/s³ and rad/s² and contribute
`(sigma * dt)^2`. At a fixed dt, matching that contribution would require
`intensity = sigma^2 * dt`; there is no constant conversion across variable dt.
This feature aligns the threshold decision, not parameter values or discretization.
Do not copy PX4's bias-noise numbers into our intensity fields unchanged.

Replay applies bias-noise selection before every real or gap-hold prediction,
using the IMU variances selected by `R-IMU-PROCESS-NOISE` (or configured variances
when that selection is unavailable) and unchanged configured intensities
(1e-6 m²/s⁵ and 1e-8 rad²/s³). These are explicit replay settings, not newly
calibrated flight noise. Prediction failure still aborts.
`bias_learning.csv` records the six selected-noise flags for real IMU ticks;
summary inhibition counts include gap-hold predictions and count only configured
positive intensities suppressed by the rule. Selection is not proof that a tiny
floating-point increment changed P. Native backend, threshold, rollback,
correction/recovery and replay tests support the rule. The `imu-policies` ESBMC
suite verifies the actual portable selector, both scalar formats/aliases, every
consumed coefficient, equality behavior, unchanged P and failure publication.
Unused P entries remain arbitrary, not assumed finite or PSD by the harness.
Lean proves nonnegative selected noise, exact identity bias transition rows and
exact PSD preservation under the existing prediction premises. These results
do not prove a hard covariance bound, IEEE PSD preservation or caller sequencing.
Full PX4 covariance constraints remain separate work.

The PX4 module applies the same production selector through `px4::predict_imu`
before real and synthetic gap-hold predictions. Real intervals first select
fault/clipping-dependent IMU variance; gap holds use configured IMU variance.
Both use the prior covariance and start from configured intensities on every
call, so suppression does not latch. Fixed intensities remain 1e-6 m²/s⁵ and
1e-8 rad²/s³, with the existing `intensity * dt` scaling. No correction permission
or bias-value limit is introduced here. Native module tests check equality,
above/below threshold, clipping-dependent release, subsequent recovery and
failure rollback. The `imu-prediction` suite now covers this actual helper's
ordering, configured-noise restoration and atomic return, with matching
same-type selector and E-STEP frames. Module-level decisions to call it, the
real/synthetic distinction and target backend remain separate obligations.

### R-BIAS-LIMITS: per-axis bias-value limits

`runtime::limit_bias(state, limits)` explicitly limits each body-axis INS
accelerometer bias to `[-limits.accelerometer, +limits.accelerometer]` (m/s²)
and gyro bias to `[-limits.gyroscope, +limits.gyroscope]` (rad/s).

- Bounds are inclusive and per coefficient, not a vector-norm constraint.
  Zero is a valid limit that forces zero bias; in-range coefficients are preserved.
  Validate both limits and all six bias coefficients before writing. Non-finite
  inputs return `non_finite_input`; otherwise negative limits return `domain_error`.
  Failure preserves the entire state. Success changes only the six bias fields.
- This operation does not alter covariance, reject a measurement, reset a bias,
  or change learning permissions. Limiting applies even to an inhibited axis if
  its prior value is already outside the bound. With fixed limits and an in-range
  prior, inhibition still holds the estimated bias. An inward correction can
  leave saturation immediately; there is no latch or recovery timer.
- Replay fixes limits for the session and validates them against the zero bias
  initial state before publishing output. After each successful GNSS, gravity or
  magnetic correction group, apply limits before the next correction/prediction.
  Rejected/failed corrections do not invoke limiting. Successful core correction
  guarantees finite biases, so limiting with those fixed valid bounds cannot fail.
  Standalone numerical-core correction/injection APIs remain additive and unchanged.
- Preparation writes `bias_limits.csv` from logged `EKF2_ABL_LIM` and
  `EKF2_GYR_B_LIM`; absent values use EKF2 module defaults 0.4 m/s² and 0.15 rad/s,
  with each source recorded in metadata. Recorded changes through session end
  are rejected. CSV input without the file uses the same explicit defaults.
  Summary reports effective bounds/source and per-axis counts of corrections
  actually clipped (not time at the bound or evidence of sensor failure).

Behavioral reference: PX4 v1.16.2
[bias correction](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L757).
PX4 limits after each scalar fusion; replay and module limit after each existing measurement
group. The axis-bound rule agrees, but sequential saturation trajectories need
not agree. Clamping does not prove posterior covariance consistency or physical
bias accuracy. Native boundary, failure, correction and replay tests cover this
feature. The `imu-policies` ESBMC suite additionally verifies the actual limiter
in binary32/binary64: inclusive bounds, preservation of in-range representations,
complete validation before writes, unchanged non-bias fields and whole-state
rollback on failure. Non-bias fields remain arbitrary, including nonfinite values.
This is a direct source-level proof; no new Lean algebra is needed for these
comparison/selection properties. Replay/module invocation, fixed-limit loading
and covariance consistency are not discharged by the standalone limiter proof.

The module uses `px4::limit_imu_bias` with fixed bounds 0.4 m/s² and
0.15 rad/s; published sensor-bias limits use the same constants. It limits startup biases and each successful GNSS velocity/horizontal
position, GNSS/barometer height and magnetic correction group before the next
operation. Rejections and resets do not invoke limiting. Each height source is
handled separately, rather than waiting until the entire height update returns.
Native adapter tests exercise accepted and rejected sources with inhibited
learning; the existing limiter tests cover boundaries, numerical failures and
inward recovery. The gravity adapter applies the same limiter after successful gravity correction.

### R-VERTICAL-ACCELERATION: apparent falling and clipping detection

`runtime::VerticalAccelerationHealth<Linalg>` is a standalone monitor. It owns
clipping counters, falling likelihood and fault timing, not estimator state/P,
noise, bias permissions, fusion decisions or resets.

| Input | Meaning |
|---|---|
| IMU time | Nonzero, strictly increasing estimator-horizon timestamp in microseconds |
| Average interval | Caller-supplied average EKF interval in seconds, not observation cadence |
| Clipping | Known per-body-axis flags accumulated over this IMU interval |
| Six observation slots | Active flag, signed innovation and innovation variance S; pressure height, GNSS height/Down velocity, range height, vision height/Down velocity |

All innovations use prediction minus observation with positive meaning apparent
falling. Negate our core's observation-minus-prediction residual; range callers
must also convert the opposite distance-to-ground direction. The caller supplies
the latest cached diagnostics from active aiding sources, including gate-rejected
observations. Lifecycle owns source activity and data expiry; this monitor adds
no independent freshness timeout and does not treat rejection as deactivation.

- An active, finite, nonnegative variance below **FLT_EPSILON** is ignored. This
  fixed reference floor is used for float and double, not replaced by DBL_EPSILON.
  Otherwise form `innovation / sqrt(S)`, without a gate-sigma divisor or absolute
  value. Any source strictly above 3 gives medium likelihood. A >3 source plus
  a >1 source of a different reference type gives high likelihood; otherwise low.
  Types are pressure, GNSS, and ground: GNSS height/velocity share GNSS, and range
  plus both vision slots share ground. Equality at 1 or 3 does not exceed a test.
- Set `N = trunc(1 / average_interval_s)`. Per axis, increment if clipped and
  count < N; otherwise decrement if count > 0. Preserve the decrement even when
  clipping continues at N, and when a changed N is below the stored count. This
  is not a sliding window or a saturating counter. Warning means count >= N/2
  with integer division; critical means count >= N. `bad_clipping` means all
  three axes warn or any axis is critical. Frequent clipping means body Z warns
  or `bad_clipping` is set.
- `bad_sample` means high likelihood, or medium likelihood with frequent
  clipping. Update last-bad time on bad samples, otherwise last-good time.
  `bad_vertical_acceleration` holds until elapsed time since last bad is >=3 s.
  Every bad sample refreshes the hold. Use ordered timestamp subtraction to
  avoid overflow. Low likelihood/no active sources is absence of evidence, not
  proof of a healthy IMU. Last-good time describes the instantaneous predicate,
  even while the fault is held; it supplies the independent 500 ms reset policy
  in `R-GNSS-FUSION`.
- Validate time first (`out_of_range`), then a finite positive interval
  (`non_finite_input`/`domain_error`) and known clipping (`domain_error` if absent).
  A non-finite reciprocal returns `non_finite_result`; require reciprocal in
  [2,65536) before uint16 conversion (`out_of_range` otherwise). This keeps the
  conversion defined and the half-limit nonzero. For each active observation,
  require finite innovation/variance and nonnegative variance, even below the
  floor. Non-finite input returns `non_finite_input`, negative variance returns
  `domain_error`, and non-finite computed ratio returns `non_finite_result`.
  Inactive observation coefficients are not consumed. Scalar-root failures
  propagate. Source validation follows slot order: barometer height, GNSS height,
  GNSS Down velocity, range height, vision height, vision Down velocity. Return
  the first failure; already sufficient falling evidence must not skip later
  active sources. Validate all sources before publishing counters or timestamps.
- Failure preserves every stored field. The stored state then belongs to the
  previous successful timestamp and is not a verdict for the failed sample.
  Unknown clipping must not be converted to clear. Reset clears all history;
  callers restart on IMU gaps, unknown-metadata interruptions or source/calibration/
  frame changes. After reset, last-update zero means no evaluation yet.

Reference: PX4 v1.16.2 [height health monitor](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/height_control.cpp#L122)
and its [source-combination tests](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/test/test_EKF_accelerometer.cpp#L111).
Alignment covers valid inputs and finite arithmetic. Explicit invalid-input
rejection, missing-metadata handling and overflow-safe time subtraction are our
API safeguards; no matching invalid-data behavior or bitwise float/double parity
is claimed. The caller must match filter cadence, averaged interval and source
diagnostic ordering for matching timelines. PX4 evaluates after GNSS velocity
and before height fusion; bias/noise prediction consumes the preceding fault.
Native tests cover source combinations, signed residual conversion, counters,
recovery and failure atomicity. The `vertical-health` ESBMC suite additionally
checks initialization/reset, the actual likelihood producer's enum/failure frame,
and the actual public update under that frame, in binary32/binary64. It establishes
ordered sample validation, clipping-count recurrence, verdict consumption, atomic
history and inductive timestamp/three-second-hold invariants. Arbitrary samples,
including failures and timestamp boundaries, remain admitted; the prior-history
premise is established at initialization and preserved by update/reset.

The actual likelihood producer also has binary32/binary64 source-equation checks:
ordered source/scalar arguments, the fixed FLT_EPSILON floor, signed IEEE division,
strict thresholds, distinct-reference grouping, first-failure priority and atomic
publication. Every IEEE root return remains admitted independently, including
negative, zero and non-finite values. The observer preserves scalar returns and
production state. A separate protocol proof establishes its prefix invariants
without assumptions; the classification and ratio profiles reuse exactly those
proved invariants. They do not assume their numerical conclusions, successful
inputs or successful results. Default safety and complete-unwinding checks remain
enabled in all profiles; the ratio profile uses non-incremental multi-property
solving to bound memory.

Scalar-root accuracy, physical detection validity and module/replay sequencing
remain **open**. No new Lean algebra is needed for the scoped state-transition
properties. See the [verification scope and evidence rules](../guides/verification.md).

Replay integration:

- Evaluate once per real IMU prediction, including ticks without GNSS messages.
  Require the GNSS quality/lifecycle profile and known clipping. Legacy inputs
  without that profile report unavailable; do not infer source activity from a
  correction count. Barometer/range/vision slots remain inactive. GNSS height
  and velocity alone can only give medium likelihood, requiring clipping to
  trigger a fault.
- Use actual GNSS correction diagnostics, including gate-rejected results:
  negate the selected `fusion_innovation` and retain S. A permitted limit is
  reflected in the monitor cache, as in the reference's aid-source status;
  raw residuals/ratios remain in the fusion log. While velocity aiding remains
  active after a quality rejection, evaluate its velocity gate/limit for the
  cache without correcting the estimator.
  Invalid receiver velocity is not a new received sample; unavailable quality
  metadata invalidates the velocity cache. Numerical failures abort replay.
- Take the diagnostic snapshot after current velocity processing and before
  current height processing. It contains the current velocity cache and the
  preceding height cache. Height activity is from before height control; velocity
  activity reflects current horizontal control. Inactive sources are excluded;
  caches are cleared after lifecycle stop. Successful full velocity/height resets
  set that cache's innovation to zero and S to observation variance, as in the
  reference's aid-status reset. A Down-only reset retains velocity diagnostics.
- Evaluate the snapshot before height fusion, so height can consume the current
  fault. Prediction, bias permissions and every velocity event at this horizon
  consume the preceding horizon's fault. For multiple due GNSS messages,
  re-evaluate each pre-height snapshot from the same saved preceding monitor
  state and retain the final result. Clipping therefore advances once, while
  every height attempt sees its current pre-height evidence. This batching
  remains a replay approximation, not EKF2's receiver-buffer schedule.
- Initialize nominal/average interval from the seed IMU integration period.
  For each actual sample update `average = 0.99*average +
  0.01*clamp(dt, 0.5*nominal, 2*nominal)`. Accel/gyro periods are synchronized in
  replay. The averaging rule follows the reference; replay's logged prediction
  cadence replaces EKF2's configured downsampled cadence. Do not use GNSS dt.
  Missing intervals/unknown clipping clear monitor, diagnostic caches and average
  history before consuming a prior verdict. Held gap predictions never increment
  clipping counters. Restart from the next actual sample with known metadata.
- `vertical_acceleration_health.csv` reports availability, interval, counters,
  likelihood (0/1/2 = low/medium/high), current fault/timers and the signed input
  diagnostics with source timestamps. `bias_learning.csv` reports the preceding
  verdict used for permissions. Summary counts separate evaluated, unavailable,
  fault-held and fault-inhibited ticks. An available low verdict with no active
  diagnostics still does not prove health.

The replay fault now feeds accel bias-learning permissions and the sampled
noise selection in `R-IMU-PROCESS-NOISE`, plus the GNSS fusion exceptions in
`R-GNSS-VERTICAL-RECOVERY`. Its last-good time separately feeds the 500 ms
reset criterion in `R-GNSS-FUSION`, shared by the module height coordinator.
Native replay tests exercise rejection, input ordering, actual bias preservation/
recovery and
metadata interruptions;
this is not a formal proof or flight-accuracy result.

### R-GNSS-QUALITY: GNSS quality checks and health qualification

Define portable GNSS quality checks and health qualification without platform messages, clocks, geographic projection or EKF state.

- Fix below 3 always fails. The ten configurable checks are satellite count, PDOP, horizontal/vertical position accuracy, speed accuracy, horizontal/vertical drift, horizontal/vertical speed and spoofing. Too few satellites or values strictly above their upper thresholds fail; equality passes. Use named `enabled_checks` and `failures`; report diagnostics even for disabled checks. Platform-specific bit-mask conversion belongs in the adapter, not this API.
- Defaults: all ten checks enabled, 6 satellites, PDOP 2.5, position accuracies 3/5 m, speed accuracy 0.5 m/s, stationary drift/speed limits 0.1/0.2 m/s, health time 10 s. The raw speed limit is 100 m/s. Accuracy inputs are standard deviations, not variances. Scalar parameters must be finite positive normal values; health time must be nonzero. Motion bounds must also permit finite ten-times clamps and the horizontal clamp's squared two-axis norm; squared horizontal-drift and raw-speed thresholds must be normal. Compare-only metadata thresholds have no artificial arithmetic upper bound.
- At rest on the ground, use `dt = clamp(sample_dt, 0.001, 10)` seconds and `alpha = dt / 10`. Filter position-derived velocity, then constrain each filter state to +/- ten times its axis threshold. Constrain each GNSS velocity input to the same bounds before filtering it. Horizontal checks use the two-axis norm; vertical checks use absolute value. The first position delta after construction/reset/invalid data is ignored, but velocity filtering still runs.
- Ground movement resets filters while retaining previous motion-check failure flags. In flight, reset filters and clear those flags. In all modes, raw horizontal speed magnitude and absolute vertical speed above `velocity_limit` force the corresponding flags; their contribution to rejection still follows `enabled_checks`. `drift_valid` is false outside stationary checks; zero metric placeholders are not valid drift measurements.
- `update` returns one result with a primary `decision`: `invalid_call`, `rejected`, `waiting` or `accepted`. All but `invalid_call` consume the sample. A currently passing sample waits until both initial qualification and recovery are satisfied. `accepted` is GNSS quality acceptance only; alignment, innovation gating and fusion remain separate caller conditions. `status` describes caller/numerical errors, not permission to fuse.
- Maintain the last passing/failing **fusion-horizon** times. The first consumed sample establishes the initial fail time even if its checks pass. Recovery requires time since failure strictly greater than `max(1 s, health_time / 10)`. Once a recovered sample also exceeds `health_time`, latch `qualified` until reset. Later rejection blocks the current sample without clearing this historical diagnostic; recovery does not repeat initial qualification.
- Non-finite or negative accuracy data, consumed non-finite vectors and arithmetic overflow produce `rejected` with a failing `status`, reset filters/position continuity and restart the health wait. Such rejection cannot be disabled. Other quality failures have `status == success` but still return `rejected`.
- Invalid configuration, zero/duplicate/backward sample time, a sample beyond `now_us`, or backward `now_us` returns `invalid_call` without changing the monitor. When a position delta is consumed, its `position_reference_time_us` must equal the preceding consumed sample's timestamp; mismatch also returns `invalid_call`. Only `decision` and `status` are meaningful in that result. Time is unsigned microseconds; validate ordering before subtraction. No wall clock is read.
- The adapter provides NED velocity and the current-minus-previous fix displacement with its reference timestamp: north/east projected about the preceding consumed fix, down = old MSL altitude minus new MSL altitude. Advance that reference only when the monitor consumes a sample. The adapter owns geographic validity, PDOP calculation, velocity-validity and spoof-state conversion, receiver identity, freshness and projection continuity. Reset on receiver/configuration changes or lost continuity. Stationary detection must not be inferred from GNSS being checked. Sample gaps do not independently revoke the health latch; freshness and fusion stop/recovery remain separate work.

The `gnss-quality` ESBMC suite covers specified defaults, all named check
selections, parameter/sample domains, ordered IEEE filter/clamp equations, mode
priorities, strict health timers and the composed public update. Initialization,
reset, health restart and arbitrary-prior update checks establish ordered clocks
and historical qualification; invalid public inputs are not assumed away.
Private motion checks use the actual public validation premises (including the
standard clamp domain), with arbitrary prior filters and arithmetic failures.
The actual two-axis norm producer checks its reduction and pure scalar-return
boundary; root outputs remain unconstrained, so accurate libm and physical
norm accuracy are not claimed. A separately checked motion-producer frame
supports the public transition, including numerical rejection, history
consumption and input purity. Native tests remain supporting evidence.
Adapter metadata, coordinate conversion, caller chronology and physical sensor
validity remain open. This monitor does not implement innovation gating,
source-specific fusion lifecycle, stationary-motion detection or jamming detection.
See the [verification scope](../guides/verification.md).

#### Replay GNSS quality

PX4 replay evaluates the existing monitor before its independent navigation
gates. Follow [v1.16.2 GNSS checks](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_checks.cpp)
and [health qualification](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp).
The separate lifecycle above consumes these quality results and orchestrates
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

#### PX4 GNSS integration

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
