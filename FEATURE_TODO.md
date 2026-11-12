# ESKF Feature TODO

## Scope and baseline

This is the authoritative functional backlog for the portable estimator and its
PX4 integration. The target is a credible PX4 research demonstration, not full
EKF2 feature parity or a hardware-flight safety claim.

Audited on 2026-11-12 against formal-eskf `a5b460f` and the in-tree
`integrations/px4/` module. The behavior reference is PX4 v1.16.2,
`54f0455ffcd755534539a7cf33a09a20bf71d29d`. Historical notes about the separate
adapter checkout do not describe this baseline.
Latest completion: F-03.1 was committed as `3fafdb6` on 2026-11-12.

- Keep task IDs and scope stable. Close completed items in place; do not renumber
  the remaining work or reintroduce completed work under another name.
- Report lists using these IDs. Identify whether a gap is in the core, portable
  runtime, replay, or deployed module; a helper alone is not module integration.
- Record newly discovered work under its existing parent, with the reason and
  dependency. Changing the agreed scope or priority requires explicit review.
- `Partial` means some implementation exists but the stated completion criterion
  is unmet. `Planned` means implementation is pending. `Design` needs a reviewed
  design before implementation. `Blocked` names its missing prerequisite.
  `Implemented; review pending` means the scoped code/checks are complete but
  acceptance is still pending; keep that item visible until reviewed.
  `Completed` records accepted, committed functionality within its stated scope.
- Completion here concerns scoped functionality and tests. Lean, ESBMC and
  numerical/physical assurance are separate claims tracked through the
  [requirements](docs/spec/verification-requirements.md) and
  [traceability map](formal/agent-review/traceability-map.json).

## Completed baseline

These capabilities remain completed within the stated scope. Their limitations
are tracked below; they are not fresh integration tasks.

| Capability | Implemented scope | Evidence/reference |
|---|---|---|
| Numerical core | INS/AHRS prediction, covariance, correction, injection/reset; Cholesky/Joseph, quaternion/SO(3), position/velocity and magnetic/gravity observations | [Core](include/formal_eskf/eskf/eskf.hpp); requirements and traceability map |
| PX4 package and GNSS | Single-instance primary SITL estimator, quality qualification, independent gated groups, horizontal lifecycle, real resets and uORB notifications | `6dce0a9`; [module](integrations/px4/module/FormalEskf.cpp) |
| Magnetic aiding | Field/heading checks, lifecycle, source/calibration changes, fixed-field correction with tilt protection, real heading resets | `73b7683`; [MagneticAiding](integrations/px4/module/MagneticAiding.hpp) |
| Geographic magnetic reference | Host WMM lookup, periodic adoption using own estimated position, separate health/heading references, geographic/saved/zero declination | `e7f660d`; [MagneticModel](integrations/px4/module/MagneticModel.hpp) |
| Height sources | GNSS/barometer arbitration, scalar offsets, fallback/re-entry, timeout recovery and coordinated height/Down-velocity reset publication | `a115afe`; [HeightAiding](integrations/px4/module/HeightAiding.hpp) |
| IMU health and bias permissions | Shared decaying-peak inhibition, vertical-innovation health, reset-safe history, permissions forwarded to module corrections | `a5b460f`; [integration contract](docs/spec/verification-requirements.md#r-px4-imu-bias-module-health-and-bias-learning-inhibition) |

The module is a stationary-start, single-IMU/single-GNSS, binary64 SITL profile.
It uses current-horizon fusion and fixed local Cartesian state, and requires
restart after numerical faults or IMU identity/calibration discontinuity.
F-01.1 now allows accel-clipped prediction after initialization; startup/gyro
clipping remains rejected. Gravity correction is wired through the shared sampler
and eligibility path (F-02.1); bounded vertical recovery is wired as F-03.1.

The [integration guide](integrations/px4/README.md) owns setup and commands;
[dated validation records](integrations/px4/validation-history.md) retain results
and source/binary evidence references. Recorded normal-flight and ground-fault
checks do not establish in-flight fault tolerance, physical tuning or
runtime/adapter formal proof.

Retained replay evidence from the previous backlog: on
`e8744135-8493-4f5b-bab8-625c5b287d5a.ulg`, 12,397 magnetic events produced
11,576 corrections, no field/gate rejections and no heading resets with the
external model and 20 microtesla / 20 degree tolerances. Strength residual
p99/max was 1.94/4.92 microtesla; inclination was 5.41/7.64 degrees. A 10/10
sensitivity run made the same decisions. Requiring an absent model blocked all
events. These are historical single-log observations, not new executions or
justification for tighter defaults. The pinned host table identifies WMM2020
evaluated at 2024.41257; the host owns model/epoch updates. F-07.1 retains the
independent field/calibration validation gap.

## Fixed work groups

These preserve the preceding work list's categories 1–7. Its former item 8,
architecture/formal verification, is tracked separately in [ROADMAP.md](ROADMAP.md#current-work-tracks).
The table order is the functional implementation order; validation accompanies
every change rather than waiting until F-07.

| ID | Work group | Remaining scope |
|---|---|---|
| F-01 | IMU and bias protection | F-01.1/F-01.2/F-01.3 completed |
| F-02 | Gravity aiding | F-02.1 completed |
| F-03 | Height protection | F-03.1 completed; F-03.2 ground effect remains |
| F-04 | Timing | Delayed fusion, bounded history and current-time output prediction |
| F-05 | Sensor location and frames | Lever arms, output reference point and local-frame operating envelope |
| F-06 | Diagnostics and configuration | Truthful confidence/failure reporting and deployment parameters |
| F-07 | Deployment validation | In-flight disturbances, accuracy/tuning, resource budgets and hardware evidence |

### F-01 — IMU and bias protection

| Task | Status and existing work | Completion criterion |
|---|---|---|
| F-01.1 — IMU noise and accel clipping | Completed (`20dbb80`): [module prediction](integrations/px4/module/ImuPrediction.hpp), native and SITL checks complete | Module selects uncertainty before prediction from preceding vertical health and current accel clipping; accepted clipped intervals reach health/bias policy. Axis selection, motion integration, fault/recovery and rollback are native-tested. Startup/gyro clipping remains rejected; gap holds retain the explicit configured-noise fallback. Physical clipped-flight validation remains F-07.1. |
| F-01.2 — Bias driving noise | Completed (`7c99696`): module uses the existing variance-dependent policy for real and gap-hold predictions | Apply before module predictions, including gap holds, after F-01.1 noise selection. Preserve our variance-density/time units; check threshold boundaries without conflating driving noise with learning permissions. |
| F-01.3 — Bias limits | Completed (`dc4e2cb`): shared fixed bounds at startup and after every successful module correction group | Apply after every successful module correction group; test boundaries, failure behavior and all call sites. Preserve the declared covariance behavior and document grouped-update differences from PX4. |

### F-02 — Gravity aiding

| Task | Status and existing work | Completion criterion |
|---|---|---|
| F-02.1 — Module gravity path | Completed (`c2acd61`): shared sampling/tilt and module eligibility/correction, enabled/disabled SITL checked | Supply actual tilt, rest, horizontal-aiding and clipping context; accumulate rotation-aware intervals, reset history on discontinuity, and correct only on eligible samples. Test aided/unaided, moving/resting and unknown-context paths, with an enabled/disabled SITL comparison. |

Keep the chosen unnormalized gravity model: INS subtracts prior accel bias once;
its 3x15 observation Jacobian has zero bias columns. Do not revive the removed
cross-product/accel2 model. The existing replay baseline is `V = g^2 I` with a
nominal 10 ms observation interval, not calibrated flight tuning.
PX4 uses a normalized observation and an innovation gate. Align the applicable
fusion conditions without claiming identical observation units or decisions.
Gravity innovation gating remains **deferred as D-02**, not silently included
in F-02.1. Unknown required metadata is not a positive eligibility decision.

### F-03 — Height protection

| Task | Status and existing work | Completion criterion |
|---|---|---|
| F-03.1 — Vertical acceleration-fault recovery | Completed (`3fafdb6`): bounded GNSS velocity and GNSS/barometer height innovations, shared last-good reset timing, qualified-source checks and reset notifications wired; native/SITL regression passed | After F-01.1, wire the exception and independent reset criterion into module GNSS/height coordination. Preserve quality permission, raw rejection diagnostics, event ordering and actual reset notifications. Distinguish the 3 s held fault from the 500 ms last-good criterion. |
| F-03.2 — Barometer ground effect | Planned: ordinary barometer fusion/offset handling exists | Review pinned PX4 conditions and residual sign, then test near-ground rejection/compensation and exit behavior. Declare any unavailable height-above-ground context; do not treat it as known. |

Dynamic-pressure compensation is D-03: it needs a defensible airflow input/model,
not just another constant in the existing barometer helper.

### F-04 — Timing

| Task | Status and dependency | Completion criterion |
|---|---|---|
| F-04.1 — Timing design | Design; requires the A-01 ownership/refactor review in ROADMAP | Specify sample versus publication time, supported delays, fixed capacities, horizon, overflow/out-of-order policy, resets and latency/memory budgets against the pinned PX4 behavior. |
| F-04.2 — Delayed fusion and output prediction | Blocked by F-04.1 | Implement the reviewed history/horizon/output path in bounded batches. Test deterministic delay, reordering, dropout and reset sequences; verify output timestamps, continuity and measured resource bounds. |

### F-05 — Sensor location and frames

| Task | Status and existing work | Completion criterion |
|---|---|---|
| F-05.1 — Lever arms and output reference point | Planned: current module assumes coincident sensor/estimated points | Define IMU/GNSS/output points, frames and calibration ownership; implement position/velocity compensation with rotation and uncertainty handling. Independent rotating-rig cases must expose sign/point mistakes. |
| F-05.2 — Fixed-frame operating envelope | Design: module local projection and replay conversions already exist | Quantify projection/frame/curvature error against an independent geographic oracle; declare usable distance, height datum, exclusions and behavior beyond the envelope. Review heading-variance/frame assumptions. No silent public-origin reset. |

A geodetic nominal-state extension is D-07. F-05.2 first establishes what the
current Cartesian model supports; it does not pre-authorize new core equations.

### F-06 — Diagnostics and configuration

| Task | Status and existing work | Completion criterion |
|---|---|---|
| F-06.1 — Confidence and failure reporting | Partial: source status, individual GNSS quality flags and reset counters exist; some aggregate fault/bias-validity reporting remains coarse | Audit actual uORB meanings against internal decisions. Distinguish alignment, active fusion, validity, convergence and numerical faults; validate yaw/bias uncertainty reporting and restart-required behavior. Rejection must not refresh successful-fusion time. |
| F-06.2 — Deployment parameters | Partial: startup magnetic-declination settings exist; most module settings are fixed | Expose the approved noise/gate/source/limit settings with documented units, bounds and startup/live-update semantics. Validate configuration changes without silently importing live EKF2 settings. |

### F-07 — Deployment validation

| Task | Status and dependency | Completion criterion |
|---|---|---|
| F-07.1 — Disturbance and accuracy evidence | Partial: native, replay and normal-flight/ground-fault SITL evidence exists | Add independent-oracle cases for magnetic disturbance, high dynamics/free fall, clipping, outages and recovery in flight. Quantify detection/rejection, attitude/position errors and recovery; test WMM epoch/calibration assumptions. Retain failing runs and compare feature enabled/disabled where meaningful. |
| F-07.2 — Resource budgets | Planned; target profile must be explicit | Measure wall-clock CPU, peak RAM and stack under nominal/fault load. Separate simulator virtual time from execution cost. Check bounded history against F-04 budgets. |
| F-07.3 — Hardware validation | Blocked on target hardware and an agreed test envelope | Establish target builds, timing/interfaces and HIL evidence before any hardware-flight claim. Simulation can progress now; unavailable hardware is not a passed check. |

## Deferred and optional decisions

These are visible gaps/options, not additions to the default implementation queue.
Promoting one requires a reviewed scope and dependency; retain its ID.

| ID | Decision | Reason it is separate |
|---|---|---|
| D-01 | Estimated magnetic field/bias, AUTO/3D and terrain-based in-flight realignment | Current fixed-field tilt-protected model is intentional. Six magnetic error states change INS 15 to 21 and covariance storage 225 to 441 coefficients; require observability/reset/proof review. WMM alone does not require them. |
| D-02 | Gravity innovation gate | Explicitly deferred by user. First quantify effective V, cadence, prior uncertainty, valid rejection and induced tilt/recovery. Earlier 0.25/0.10 candidates are not validated tuning; prediction/observation reuse correlation remains a model limitation. |
| D-03 | Dynamic-pressure compensation / wind-airflow inputs | Required inputs and model are absent; decide the supported vehicle/sensor configuration first. |
| D-04 | Emergency yaw recovery | Additional estimator/recovery policy; ordinary magnetic heading resets already exist. |
| D-05 | Multiple receivers/IMUs and estimator selection | Current scope is single instance/sensor; switching and redundancy introduce new ownership/lifecycle contracts. |
| D-06 | Rangefinder and terrain | New observation/reference model; not part of existing GNSS/barometer closure. |
| D-07 | Geodetic nominal state / long-range navigation | Depends on F-05.2 results and a separate core-model decision. |
| D-08 | Automatic recovery after numerical or IMU-identity faults | Current restart-required behavior is explicit. New recovery must define reinitialization and downstream continuity; do not hide it inside ordinary sensor timeout handling. |

## Next implementation

**F-03.2: barometer ground effect.** Review the pinned conditions/residual sign
and available near-ground context before implementation. The four authorized
integration tasks are completed in separate commits; physical disturbance
validation and formal proofs remain separate work.

The fixed order remains **F-01.2 → F-01.3 → F-02.1 → F-03.1 → F-03.2**, one
reviewed change at a time. F-04 starts with design review. F-07 evidence is
accumulated alongside these changes. Keep completed IDs in this file.
