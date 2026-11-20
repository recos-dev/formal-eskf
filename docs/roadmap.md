# Roadmap

This is the single work list for functionality, architecture, verification and
tooling. The target is a credible PX4 research demonstration, not full EKF2
parity or hardware-flight qualification. Current behavior belongs to the
[requirements](spec/README.md) and [PX4 integration guide](../integrations/px4/README.md);
the [verification guide](guides/verification.md) defines evidence boundaries.

## Using this roadmap

- Preserve task IDs, scope and agreed order. Add discovered work under its existing
  parent with its reason and dependency; review changes to scope or priority.
- Identify the affected layer: core, portable runtime, replay or deployed module.
  A reusable helper alone does not complete its module integration.
- Use **Completed** for accepted, committed work within its stated scope;
  **Partial** when a completion condition remains unmet; **Review pending** when
  implementation/checks await acceptance; **Planned** for pending implementation;
  **Design** for work needing a reviewed design; and **Blocked** with its prerequisite.
- Task status is separate from requirement Draft/Approved status and from proof
  or target acceptance. Completed functionality does not close another task's
  numerical, formal, physical or resource obligations.
- Keep each task's status here once. Link contracts and acceptance evidence rather
  than copying equations, parameter inventories, run results or development logs.

## Functional work

F-01 through F-07 retain the agreed functional order. Validation accompanies each
change; it is not postponed until F-07. Completed scopes remain listed to avoid
reintroducing them as new tasks. Reference comparisons use the revision pinned
by the [PX4 architecture](spec/architecture/px4-module.md).

### F-01 — IMU and bias protection

| Task | Status | Scope and completion boundary |
|---|---|---|
| F-01.1 — IMU noise and accel clipping | Completed | Module noise selection and accepted post-startup accel-clipped prediction, including health/bias handoff and gap-hold behavior. Startup/gyro clipping remains rejected. Physical clipped-flight validation remains F-07.1. |
| F-01.2 — Bias driving noise | Completed | Variance-dependent policy applied after F-01.1 noise selection to real and gap-hold predictions. Driving noise remains distinct from bias-learning permission. |
| F-01.3 — Bias limits | Completed | Startup and successful correction-group bounds, including boundary/failure and call-site checks. Covariance behavior and grouped-update differences remain explicit. |

Contracts: [IMU health and bias binding](spec/requirements/integration/px4/imu-bias.md),
[configuration](spec/requirements/integration/px4/configuration.md).

### F-02 — Gravity aiding

| Task | Status | Scope and completion boundary |
|---|---|---|
| F-02.1 — Module gravity path | Completed | Shared rotation-aware sampling, tilt/context eligibility and module correction; aided/unaided, moving/resting, unknown-context and enabled/disabled cases. Gravity innovation gating remains D-02; effective covariance and physical tuning remain F-07.1. |

Contracts: [gravity observation](spec/requirements/core/gravity-observation.md),
[module binding](spec/requirements/integration/px4/gravity-aiding.md).
The chosen unnormalized model and removed accel2 model are not new design choices
in this backlog; applicable PX4 fusion conditions do not imply identical units
or acceptance decisions.

### F-03 — Height protection

| Task | Status | Scope and completion boundary |
|---|---|---|
| F-03.1 — Vertical acceleration-fault recovery | Completed | Bounded vertical-innovation exception and independent last-good reset criterion wired into GNSS/height coordination, with qualified-source checks, raw rejection diagnostics and reset notifications. |
| F-03.2 — Barometer ground effect | Partial | Delayed land-context activation, configured hold/deadzone and diagnostics are integrated. No terrain/HAGL estimate is supplied. Physical ground-effect and landing acceptance remain open under F-07.1. |

Contracts: [height fusion](spec/requirements/integration/px4/height-fusion.md),
[ground effect](spec/requirements/integration/px4/barometer-ground-effect.md).
Dynamic-pressure compensation remains D-03.

### F-04 — Timing

| Task | Status | Scope and completion boundary |
|---|---|---|
| F-04.1 — Timing design | Completed | Initial [250 Hz SIH profile](spec/profiles/px4-sih-250hz.md): epochs, delays, capacities, horizon, ordering/overflow and reset behavior. Target resource and accuracy acceptance remain F-07. |
| F-04.2 — Delayed fusion and output prediction | Partial | IMU, observation/context and output owners are connected to live delayed prediction/fusion and current-time publication, with source-composed caller checks and native scenarios. Full deployed refinement remains outside V-01's scoped claim; landing, accuracy and target resource acceptance remain F-07. |

Contracts: [delayed fusion](spec/requirements/integration/px4/delayed-fusion.md),
[output tracking](spec/requirements/integration/px4/output-predictor.md).
Completed integration does not establish numerical continuity bounds or target
latency/memory budgets.

### F-05 — Sensor location and frames

| Task | Status | Scope and completion boundary |
|---|---|---|
| F-05.1 — Lever arms and output reference point | Completed | Nominal fixed-geometry compensation, delayed GNSS inputs, current body-point output and reset deltas; independent rotating-point cases check signs and points. [Sensor positions](spec/requirements/integration/px4/sensor-positions.md) excludes installation uncertainty and covariance transport. Extending those limits needs a separate reviewed scope. |
| F-05.2 — Fixed-frame operating envelope | Partial | [Output-frame fixes](spec/requirements/integration/px4/output-frames.md) are implemented; geographic assessment awaits systematic review. Compare projection/frame/curvature error with independent geodesy and accept a distance/latitude/height envelope, datum, exclusions and out-of-envelope behavior. Review heading-variance assumptions. No accepted flight envelope or silent origin reset. |

A geodetic nominal-state extension remains D-07; F-05.2 does not authorize new
core equations.

### F-06 — Diagnostics and configuration

| Task | Status | Scope and completion boundary |
|---|---|---|
| F-06.1 — Confidence and failure reporting | Completed | Capability flags, bias validity, faults, innovation summaries and covariance proxies follow the [status contract](spec/requirements/integration/px4/status.md). No bias-convergence or transported/current-time covariance claim; numerical/history faults require restart. |
| F-06.2 — Deployment parameters | Partial | Startup parameters and the Kconfiglib build frontend bind packaged PX4 controls. Remaining work: review supported tuning profiles and additional startup bindings against the [configuration inventory](guides/configuration.md#px4-deployment-menu). Preserve units, defaults, restart semantics and configuration-specific evidence. Replay stays separate; do not import live EKF2 settings implicitly. |

### F-07 — Deployment validation

| Task | Status | Scope and completion boundary |
|---|---|---|
| F-07.1 — Disturbance and accuracy evidence | Partial | Native, replay and scoped SITL checks exist. Add independent-oracle cases for magnetic disturbance, high dynamics/free fall, clipping, outages and recovery; quantify errors, detection/rejection and recovery, including WMM epoch/calibration and physical tuning. Compare enabled/disabled features where meaningful and use post-event publications for cross-topic assertions. SIH landing/contact acceptance remains failed; simulator repair/full landing acceptance is deferred. Before an upstream report or fix, review contact boundaries, timestep, oblique motion and takeoff behavior. |
| F-07.2 — Resource budgets | Planned | For an explicit target profile, establish non-throwing dependency behavior and allocation phases; measure CPU, peak RAM, full call-chain stack, synchronization/response time and bounded-history costs under nominal/fault load. Include logging, diagnostic readers and forced stop under the [real-time contract](spec/requirements/integration/px4/real-time.md). Simulator virtual time is not execution cost. |
| F-07.3 — Hardware validation | Blocked | Requires target hardware and an agreed test envelope. Establish target builds, timing/interfaces and HIL evidence before hardware-flight claims. |

[Validation-data guidance](guides/validation-data.md) owns oracle and data
requirements. Retain failed/excluded cases in retrievable evidence; an isolated
simulator defect does not turn a failed acceptance scenario into a pass.

## Architecture, verification and tooling

These tracks do not reorder the functional work. Apply the
[verification policy](spec/verification/assurance.md) and preserve the
requirement-to-source-to-evidence chain.

### V-01 — Remaining proof boundaries

**Status: Partial.** Scoped core/runtime evidence exists. Close the source,
numerical and composition gaps listed in the
[open verification boundaries](guides/verification.md#open-boundaries).
IMU, observation/context, output-predictor and
`HeightAiding` owners have scoped contracts and inductive evidence within their
documented producer and source-adapter boundaries. Owner/caller coverage also
includes source-composed PX4/replay temporal checks; its
[reviewed frames and platform assumptions](guides/verification.md#px4-and-replay-temporal-callers)
do not establish full deployed-module refinement or numerical tracking bounds.
Keep producer/caller dependencies and history/feedback transactions explicit;
the entire callback is not assumed atomic. Record accepted profiles and remaining
gaps in the [traceability map](../formal/agent-review/traceability-map.json) and
execution evidence. Platform/resource acceptance is shared with F-07.2/F-07.3;
neither track substitutes for the other's evidence.

### V-02 — Design and execution-evidence pilot

**Status: Design.** Review one `E-STEP-PRED` pilot before expanding the tooling.

1. Link ownership, call order, scratch storage, publication and failure boundaries
   to the existing requirements and production symbols; cite the equations.
2. Give existing native cases stable IDs/profiles for same-prior prediction, first
   failure, arbitrary sentinel outputs and supported aliases. Reject unknown or
   duplicate selections; filenames do not establish coverage.
3. Extend the existing map/report schema with source/build/tool identity, command,
   outcome, duration and log hashes. Dirty-tree runs need snapshot identity.
4. Check missing cases/profiles, stale references, duplicate results and modified
   runner-owned outcomes offline; they must not produce a complete PASS.
5. Expand to correction/injection/reset only after pilot review. Keep native,
   mathematical, bounded-source and AI evidence distinct and retrievable.

### V-03 — AST mapping and review completeness

**Status: Design.** [AST extraction](../scripts/inventory_cpp.py) exists. Review a
versioned symbol-to-requirement baseline and configuration-completeness rules,
including unclassified/new/removed symbols, before requiring one review result
per function. Follow the [inventory design](spec/verification/source-inventory.md);
declaration discovery or hash equality alone does not establish coverage.

### V-04 — Analysis inventory and version validity

**Status: Design.** Review FHA/FMEA/FTA applicability, hazard seeds, analysis
boundaries and dependencies, including common causes and supporting timing/model
evidence. Pilot IMU timing, then define dependency snapshots, offline completeness
checks and per-release dispositions. Analysis validity, evidence applicability
and residual-risk acceptance are separate decisions. Drafts await systematic
review; no adopted analysis baseline, inventory checker or release gate exists.

### DC-09 — Reusable call-graph review script

**Status: Completed.** The [call-graph extractor](../scripts/call_graph.py)
reuses the Clang inventory and produces a manual review queue with change-impact
checks. Follow the [review guide](guides/call-graph.md) to inspect one call chain
at a time. This completes the reusable tooling, not the semantic review or V-03.

- Generate caller/callee edges, call-site locations, per-function incoming/outgoing
  lists, readable group diagrams and a numbered review inventory. Declare entry
  points, dependencies and build profiles; report unresolved/indirect calls.
- Record revision, source hashes, configuration and tool version in reproducible
  snapshots under `output/`. Compare snapshots, preserve review identities and
  mark affected conclusions stale rather than carrying them forward silently.
- Review call-chain premises, value/alias changes, status precedence and failure
  outputs. Repeated predicates or graph edges alone do not establish redundancy.
- Check known calls/branches and representative profiles. Completion is a
  rerunnable graph/review queue with explicit coverage limits, not automatic
  semantic review or code modification.

### V-05 — Generated/dense equivalence proof

**Status: Partial.** The [INS codegen package](../codegen/README.md)
contains prediction covariance, reset covariance, innovation products and Joseph
covariance, with exact-algebra certificates, scoped memory checks and isolated
native evaluation. The physical model is separated from CSE/emission and the
checker constructs its reference independently. `ESKF_CODEGEN` selects generated
INS arithmetic by default, with dense arithmetic selectable at build time. Native tests
cover actual core/runtime dispatch. AHRS generation is deferred.

The [generation scope](../codegen/README.md#generation-scope) keeps nominal/SO(3)
operations, transition/noise producers, short observation formulas, the solve and
control flow in ordinary C++. Generated `E-PCOV`, `E-RESET`, innovation/Joseph
correction and combined `E-STEP` have explicit caller, actual-kernel
frame/mirroring and publication
profiles, with [shared producer dependencies](guides/verification.md#generated-covariance-prediction).
Source acceptance requires complete matching execution evidence. Qualification
of the selected deployment's numerical budget, performance and stack remains
open. Dense core ESBMC profiles and standalone kernel certificates do not discharge
those obligations. Preserve symmetry premises and selected-path overflow/status
behavior; do not require universal bitwise agreement.

- Fix dense/generated revisions, derivation/artifacts, project state/frame
  conventions, scalar/backend and compiler modes; require reproducible generation.
- Separate exact-real equality, IEEE-754 results and observable API behavior. If
  reordering prevents bitwise equality, retain counterexamples and review any
  bounded-error alternative; do not silently relax tolerances or input domains.
- Prove actual kernels and then their composition, including status precedence,
  non-finite/overflow behavior, aliases and failure-output preservation. Link each
  to requirements and evidence; generated-artifact proof is not generator proof.
- Keep differential/high-precision tests, operation counts and performance
  benchmarks separate from formal equivalence evidence.

## Completed refactors

These IDs retain their accepted implementation scope, not whole-system proof or
target qualification. Git owns the revision history and local output owns run logs.

| ID | Status | Completed scope |
|---|---|---|
| RF-01–RF-05 | Completed | Numerical core injection/reset, correction, observations, innovation and combined acceptance. |
| RT-01–RT-11 | Completed | Portable runtime responsibilities, caller/interface migration and subsequent file consolidation. |
| A-01 | Completed | PX4 single-writer ownership, input/admission, aiding, publication and lifecycle refactor. Resource, dependency and numerical qualification remain F-07/V-01. |
| DC-01–DC-08 | Completed | Core call-graph review and adopted defensive-check/computation changes. Retained checks and sparse-algebra proposals were not authorized removals. Tool reuse is DC-09. |

## Deferred and optional decisions

These are visible gaps/options, not additions to the implementation queue.
Promotion requires reviewed scope and dependencies; retain the existing IDs.

| ID | Decision | Boundary |
|---|---|---|
| D-01 | Estimated magnetic field/bias, AUTO/3D and terrain-based realignment | Changes state dimension and requires observability/reset/proof review. WMM alone does not require estimated magnetic states. |
| D-02 | Gravity innovation gate | Explicitly deferred. Quantify effective covariance, cadence, prior uncertainty, valid rejection and tilt/recovery before choosing tuning. Prediction/observation reuse correlation remains a model limitation. |
| D-03 | Dynamic-pressure compensation / wind-airflow inputs | Supported vehicle/sensor configuration and defensible inputs/model are missing. |
| D-04 | Emergency yaw recovery | Additional estimator/recovery policy; ordinary magnetic heading resets already exist. |
| D-05 | Multiple receivers/IMUs and estimator selection | Current scope is single instance/sensor; switching and redundancy need new ownership/lifecycle contracts. |
| D-06 | Rangefinder and terrain | New observation/reference model outside GNSS/barometer closure. |
| D-07 | Geodetic nominal state / long-range navigation | Depends on F-05.2 results and a separate core-model decision. |
| D-08 | Automatic recovery after numerical or IMU-identity faults | Current behavior requires restart. New recovery must define reinitialization and downstream continuity, separately from sensor timeouts. |
