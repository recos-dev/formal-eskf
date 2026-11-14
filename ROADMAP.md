# Roadmap

## Current work tracks

Reconciled on 2026-11-12 against `a5b460f`. Functional priorities and completion
states live only in [FEATURE_TODO.md](FEATURE_TODO.md). Its fixed F-01–F-07 groups
preserve the previous functional list; architecture/formal work (previous item 8)
is separated below. These planning IDs are not verification requirement IDs.

| ID | Work | Status and completion criterion |
|---|---|---|
| A-01 | Architecture and refactor | Current-horizon PX4 refactor accepted and committed: batches 1–6 are `95c22cf`, `2c8815d`, `ec85c36`, `dac5451`, `b045ff3` and `3e29d33`, with replay prerequisite `5c781f7`. F-04 timing changes, F-06 semantics/configuration and F-07.2 target resource acceptance remain separate obligations; the dated validation history maps the PX4-* clauses to their evidence and remaining work. |
| V-01 | Runtime/module proof plan | Inventory and proposed proof sequence below are ready for review against `3e29d33`. First implementation: E-INNOV. No new proof completed by this planning step. Retained per-profile measurements inform a sequential pilot on the 16 GB host; new-profile costs and an enforced aggregate memory limit must be checked before expansion. |
| V-02 | Design and execution-evidence pilot | Pending, after the scope review. Use the E-STEP-PRED pilot below; add source-linked native evidence without treating it as new algebraic proof or rewriting all verification tools. Expand only after the pilot is reviewed. |
| V-03 | AST mapping and review completeness | Design pending. AST inventory generation already exists. Review a versioned symbol-to-requirement baseline and completeness rules before connecting it to mandatory AI review; inventory generation alone does not prove coverage. |

Small functional integrations keep their existing contracts and tests current.
They do not close V-01 or imply proof of the PX4 module. A-01 review is required
before major ownership/timing refactors, including F-04; do not silently replace
the functional queue with a proof/tooling queue.

## V-01: runtime and module proof plan

Baseline: `3e29d33`, inventoried on 2026-11-14. This plan follows the accepted
current-horizon refactor; it does not reorder the F-series or implement the
future delayed-fusion architecture. Contracts remain in the specifications,
references in the existing traceability map, and execution evidence in retained
reports. Do not introduce a second live proof registry.

### Established evidence and open scope

| Inventory unit | Count | Interpretation |
|---|---:|---|
| Registered requirement IDs | 50 | Current map, excluding two retired IMU-policy IDs |
| IDs with both Lean and ESBMC references | 25 | Numerical-core/supporting claims with their existing limitations; reference presence is not fresh execution acceptance |
| IDs without Lean or ESBMC references | 25 | Three core extensions and all 22 registered runtime/integration requirements |
| Scheduled ESBMC profiles | 1,538 | 127 source/entry-point pairs in 26 harness sources; configurations are not independent requirements |
| Portable runtime headers | 12 | All included in the component scopes below; file coverage is not property coverage |
| PX4 architecture clauses | 7 | PX4-OWN/EXEC/TIME/LIFE/OUT/RT/VERIFY; separate design obligations, not extra proved rows in the map |

All mapped files and literal symbol references resolve. No scheduled harness
currently checks a runtime/module implementation. The existing proofs of
unrestricted correction do not cover bias inhibition or default magnetic tilt
protection. Do not report this inventory as "50% of the estimator proved" or
reuse a historical PASS after a dependency changed without checking its identity.

The [combined module acceptance](integrations/px4/validation-history.md#combined-module-acceptance-2026-11-14)
supplies production-path regression, sanitizer, package and SITL evidence.
Its 87 scenarios and 70 source-reference entries are neither formal proofs nor
a complete requirement denominator. The generated planning snapshot is
`output/runtime-proof-plan-20261114/inventory.json`; it records map/source hashes,
the scheduled inventory and retained resource-log references. It is generated
evidence, not another tracked specification or an AST review baseline.

### Proposed dependency order

These are work packages within V-01, not new requirement IDs or promises of one
large commit per row. Implement one independently reviewable contract at a time.
Each registered ID appears once below. For requirements containing module/replay
clauses, proving the portable component alone leaves the caller clauses open
until package 9 checks their actual wiring.

| Order | Existing requirements | Production scope and acceptance obligation |
|---|---|---|
| 1 | `E-INNOV` | `eskf/innovation.hpp`: every computed ratio, equality acceptance, status/failure frame and legal aliases. First cost/contract pilot; no filter, solve or lifecycle expansion. |
| 2 | `E-BIAS-UPDATE`, `E-MAG-TILT` | `eskf/correction.hpp`, `eskf/magnetometer.hpp`: permitted gain rows, vertical projector and use of the **same final gain** for error and Joseph covariance; injection/reset failure propagation. Prove bias control first, then projection and their combination. |
| 3 | `R-MEASUREMENT-FUSION` | `runtime/measurement_fusion.hpp`, `runtime/navigation.hpp`: S and gate use the same prior as correction; rejected/failed observations cannot become fused or mutate the checked estimate. Preserve diagnostic availability after a later correction failure. |
| 4 | `R-IMU-PROCESS-NOISE`, `R-BIAS-PROCESS-NOISE`, `R-BIAS-LIMITS`, `R-VERTICAL-ACCELERATION`, `R-REPLAY-BIAS`, `R-PX4-IMU-BIAS` | `runtime/imu.hpp`, `runtime/vertical_acceleration_health.hpp`, `ImuPrediction.hpp`: selected noise starts from configured values; exact mask/limit behavior, decaying peaks and health transitions. Caller obligations include previous-health use, separate periods and no health/peak update from held-rate prediction. |
| 5 | `R-GNSS-QUALITY`, `R-GNSS-FUSION` | `runtime/gnss_quality.hpp`, `runtime/gnss_lifecycle.hpp`: qualification history, monotonic admission, one-use permissions, timeout/recovery and actual-result feedback. Invalid calls may revoke permissions while preserving history; do not impose whole-object rollback. |
| 6 | `R-GNSS-RESET`, `R-GNSS-VERTICAL-RECOVERY`, `R-HEIGHT-FUSION` | `runtime/navigation.hpp`, `HeightAiding.hpp`: selected state/covariance reset blocks and deltas, bounded recovery innovations, height datum/offset history and two-source transaction. Reset requests alone must not advance successful-reset history. |
| 7 | `R-ACCELEROMETER-MAGNITUDE`, `R-ACCELEROMETER-CONDITIONS`, `R-ACCELEROMETER-FUSION` | `runtime/accelerometer.hpp`, `runtime/tilt_alignment.hpp`: LPF/rotation-aware interval construction, tilt latch, eligibility and correction delegation. Missing/clipped metadata, source/gap resets and consumed sample history remain explicit. |
| 8 | `R-MAGNETIC-FIELD`, `R-MAGNETIC-HEADING`, `R-MAGNETIC-REFERENCE`, `R-MAGNETOMETER-FUSION`, `R-MAGNETOMETER-LIFECYCLE`, `R-MAGNETOMETER-FILTER`, `R-MAGNETOMETER-RESET` | `runtime/magnetic_field.hpp`, `runtime/magnetic_reference.hpp`, `runtime/magnetometer.hpp`, `runtime/magnetometer_lifecycle.hpp`, `MagneticModel.hpp`: qualify field/reference and history first, then lifecycle/filter, reset and fusion. Keep health reference distinct from observation reference; only actual reset success establishes alignment. |
| 9 | Caller portions of the above IDs; `PX4-*` design clauses | Four actual `FormalEskf` translation units and replay callers: admission, initialization, ordered calls, feedback, partial failure, fault latch, publication/reset metadata and diagnostics. Retain separate module/replay clocks and priors. Record platform assumptions and F-series gaps rather than treating component passes as module proof. |

### Proof boundaries and completion rules

- **Exact mathematics:** extend Lean for the gate's exact-real interpretation,
  constrained-gain/projector identities and reset/covariance transformations
  where needed. Do not require a new algebraic theorem for every boolean policy,
  or claim exact arithmetic proves IEEE thresholds, physical health or accuracy.
- **Production numerical paths:** use the fixed-array backend and existing
  scalar/solve/product contracts. A caller proof must check actual arguments,
  prior identity, invocation order, status and publication. Every summary needs
  matching producer evidence for its exact type/configuration. Permit every
  result allowed by that contract, including failures and scratch writes;
  never replace an unproved operation with an always-successful stub.
  Cover supported INS/AHRS paths, both scalar formats and applicable independent
  macro modes at the portable boundary. Do not multiply boolean-only lifecycle
  proofs by irrelevant scalar/backend modes. Module wiring starts with its actual
  INS/binary64/PX4 matrix profile, `ESKF_QUAT_APPROX=0`, `ESKF_RESET_APPROX=0`
  and `ESKF_MAG_TILT=1`; other deployments need separate acceptance.
- **Persistent policies:** first check actual initialization and bounded sequences
  of update, fusion/reset feedback, timeout, source change and restart. Before
  claiming safety for arbitrary flight duration, establish an invariant at
  initialization and its preservation across every permitted public transition,
  including invalid calls and timestamp/counter boundaries. A short successful
  sequence is only bounded evidence. Any separate Lean transition model still
  needs reviewed C++ correspondence; it is not a refinement proof.
- **Composition:** preserve each operation's mutation frame. GNSS can retain an
  earlier group's success after a later failure; height commits state/P/history
  together; gravity/magnetic sampling may consume history before a failure.
  The owner prevents later aiding/valid publication after a latched fault. Check
  real lifecycle feedback, bias permissions and reset reporting at these boundaries.
- **Module feasibility:** first instantiate the actual `ImuPrediction` and
  `HeightAiding` templates with the proof backend. `FormalEskf` instead binds concrete
  PX4 matrix/OS types; core fixed-array results do not verify that binding.
  Try isolated actual member paths with explicit platform-effect contracts before
  expanding the module checker. If frontend/backend support blocks them, retain
  the gap and review a specific solution; do not substitute a rewritten estimator,
  silently change the deployment backend or introduce a framework just for proofs.
- **Platform and physical claims:** pthread/uORB/work-queue lifetime, dependency
  allocation/locking, forced stop, complete stack/deadline bounds and actual
  sensor/model validity need their own evidence. Existing native concurrency tests
  are supporting tests, not a data-race or WCET proof. F-04/F-05/F-06/F-07 remain
  their owners; future delayed histories/output prediction are not current proof
  subjects. V-03's mandatory AST-to-requirement review remains unimplemented.

For each completed contract, retain requirement/source/tool/configuration identity,
input domain, invariants, alias cases, dependency assumptions, exact invocation,
result and resource usage. Run targeted mutations that violate the claimed
property and retain their counterexamples; a timeout is not mutation detection.
Resolve affected caller obligations before marking the requirement complete.
Do not delete production checks or change behavior as part of proof construction.

### First implementation: E-INNOV

Use the existing `check_innovation` body, its native tests and fixed-array helpers.
Cover binary32/binary64, observation sizes 1/2/3 and all five whole-vector alias
partitions: all distinct, input/input shared, output/first shared, output/second
shared and all shared. An input being output storage is still a consumed input;
do not assume the old diagnostic vector is finite or zero when it is independent.

Check the specified IEEE operation order, every published ratio, aggregate
rejection and output preservation on failure. Include NaN/Inf, signed zero,
nonpositive/subnormal variances, arithmetic overflow/underflow and an early
outlier followed by an invalid later axis. Isolated failure statuses and declared
validation order matter; do not invent a universal precedence for competing
per-axis domain/safe-divisor failures. Gate success is diagnostics, not fusion.

Begin with one scalar/shape profile to measure solver cost, then complete the
declared matrix. If splitting arithmetic from control flow becomes necessary,
prove the arithmetic producer and caller boundary together. Acceptance requires
positive symbolic runs, dependency closure and mutations of equality, aggregation,
denominator and premature output publication. This closes E-INNOV only, not
R-MEASUREMENT-FUSION or the choice/calibration of a gate sigma.

### Resource policy for the 16 GB host

Existing retained logs were parsed again; these are historical measurements,
not fresh proof runs or bounds for the new runtime work:

| Measured execution set | Profiles | Sum of elapsed times | Maximum single-process RSS |
|---|---:|---:|---:|
| `output/refactor-correction-mag-20261113/` | 316 | 4,165.70 s | 2,233,304 KiB (2.13 GiB) |
| `output/refactor-core-20261113/` | 130 | 425.55 s | 539,556 KiB (527 MiB) |

The earlier strategy review records a 55.11 GiB OOM; that is a failed historical
query, not necessary memory for every proof. Current script defaults are 120 s /
4 GiB, with selected profiles using other limits, including 900 s / 8 GiB for
prediction. Those internal limits do not enforce an aggregate host budget.

Start new profiles sequentially, without simultaneous builds or full-suite runs.
Before executing the pilot, verify an OS-enforced process-group/container limit;
propose a 6 GiB aggregate ceiling with swap disabled for that group and retain at
least 4 GiB available host headroom beyond the ceiling before starting. Start
with 120 s / 4 GiB internal limits and an outer wall deadline; these limits are
a proposed execution policy, not changes
already made to `verify_esbmc.sh`. Do not claim an RSS sampler is an enforced cap.
If enforcement is unavailable, resolve that execution prerequisite before heavy
queries rather than relying on the internal limit alone.

Record exit/timeout/OOM, wall and CPU time, per-process RSS and aggregate peak
separately. Stop and review an oversized query's proof boundary; never narrow its
input domain or disable unwinding assertions to fit the budget. New-profile costs
are unknown until measured. Parallelism requires aggregate measurements first;
past two-worker success does not certify safe concurrency on this host.

Planning self-review: all 25 requirements without formal references are assigned
once, all 12 runtime headers have an owner, and no source/spec/map/tool behavior
was changed. No new solver, Lean build, native test, semantic AI audit or whole-source
AST coverage is claimed by this inventory.

The pilot below defines V-02's proposed scope and acceptance criteria; it is not
completed verification. Use specifications and the traceability map for live
contracts and evidence. Historical implementation checklists are kept outside
this current plan.

## Design and execution-evidence pilot

Extend the existing specification-driven verification system with lightweight
Software Architecture Design (SWAD), Software Unit Design (SWUD), and native
test execution evidence. These are planned tasks, not completed verification.
Do not change ESKF core behavior or weaken existing proofs for this migration.

### 1. Reconcile the existing baseline

- Align the review criteria with the requirements actually registered in
  `formal/agent-review/traceability-map.json`; remove stale scope statements.
- Reconcile the already registered observation-model requirement `E-OBS` with
  its actual references and explicit gaps, without implying its proofs are complete.
- Preserve requirement IDs and existing Lean/ESBMC obligations, including the
  Lean evidence for `E-STEP`. Design and native tests do not replace proofs.

### 2. Complete one E-STEP-PRED pilot

- Add concise SWAD coverage of component responsibilities, dependency direction,
  ownership, call order, scratch storage, publication, and failure containment.
- Add SWUD coverage of the prediction units' interfaces, preconditions,
  postconditions, aliasing, implementation decisions, and verification boundaries.
  A unit is a reviewable module, not necessarily one function. Cite existing
  specifications for equations and behavior instead of duplicating them.
- Map existing nominal/covariance prediction success and rollback cases first.
  Verify use of the same prior, no publication before all stages succeed,
  first failing stage propagation, and supported aliases. Separate-output
  rollback fixtures must use distinct sentinel outputs, not just input copies.
- Give cases stable IDs and record their requirement/design links, source
  symbols, oracle, applicable profiles, and limitations. Do not reuse retired
  IDs. Classify unit versus integration cases by the contract under test;
  filenames and use of a real backend alone do not determine the level.
- Extend the existing test support and CTest registration to select and report
  cases with their profiles. Unknown/duplicate selections must fail; a selected
  case must not silently run unrelated cases. Reuse existing test bodies.

Acceptance: the pilot has a validated chain from `E-STEP-PRED` through design,
production code, and named native cases to actual execution results. Other
`E-STEP` clauses remain explicitly outside pilot completion.

### 3. Validate and collect the pilot evidence

- Extend the existing map and report schema in place. Declare required evidence
  per requirement or scoped clause, with reviewed applicability. Missing required
  evidence remains a gap; removing a requirement to obtain PASS is not allowed.
- Identify executions by case and applicable configuration, not case ID alone.
  Keep backend, INS/AHRS, scalar format, measurement dimension, and both
  approximation switches distinct where relevant. Expand only dimensions used
  by the case; prediction and reset switches are independent, not paired modes.
- Have the runner record the tested source revision, build profile, compiler
  and dependency versions, relevant flags, command, result, duration, and log
  hashes. Release evidence requires the exact reviewed clean source revision;
  local dirty-tree runs need a source snapshot identity and must not be labeled
  as evidence for an unchanged commit. Retain results and logs as CI/release
  artifacts, not a mutable checked-in latest-pass file.
- Validate ID uniqueness, safe file references, symbols, design/requirement
  links, case/profile registration, and expected versus executed profiles.
  Add offline regression tests before enforcing the new checks.
- Keep deterministic test counts and results runner-owned and separate from
  Lean/ESBMC evidence and agent findings. A test failure must fail the combined
  result; missing execution must make it incomplete. Older passes or agent
  output cannot override failures or supply missing evidence.
- During migration, distinguish the existing scoped proof verdict from the
  incomplete design/test coverage. Never present partial migration as complete
  assurance. Report requirements, cases, profiles, and proofs separately.

Acceptance: missing cases/profiles, stale source evidence, duplicate execution
entries, and altered deterministic results cannot produce a complete PASS.
CI checks remain offline; semantic AI review is a separate invocation.

### 4. Expand only after the pilot works

- Extend to injection/reset and complete correction transactions, including
  `E-CORRECT` and the remaining `E-STEP` clauses, then other numerical units and
  observation models. Reuse existing tests before adding missing cases.
- Add real-component integration and backend differential tests where the
  mapped obligations lack them. Controlled test doubles can verify call order
  and failure handling, but do not establish real-backend integration behavior.
- Configured Clang AST inventory generation already exists in
  `scripts/inventory_cpp.py`. V-03 must first review baseline mapping and
  completeness rules before integrating that inventory into mandatory review.
  Inventory generation and file/symbol text checks alone do not establish that
  all production functions have been mapped or verified.
- Keep PX4/KITTI replay classified separately as demonstrations until scenarios
  and acceptance oracles are defined. They do not provide independent navigation
  accuracy evidence merely by executing successfully.

Do not pre-create a large architecture/unit document tree, move the map into
another registry, reorganize all test directories, or replace the test framework
for this work. Add only the documents, catalog entries, and tooling needed by
the pilot, then expand incrementally. Platform stack/timing, runtime/adapter
verification, numerical error bounds, and certification remain separate work.

## Planning ownership

- Functional next steps and completed integration scope: [FEATURE_TODO.md](FEATURE_TODO.md).
- Architecture/formal proposals: the fixed A-01/V-01/V-02/V-03 tracks above.
- Behavior and proof obligations: [specifications](docs/spec/) and
  [traceability map](formal/agent-review/traceability-map.json).
- Tested module configurations and empirical results: [PX4 integration](integrations/px4/README.md).

Keep implementation status, formal evidence and physical validation separate.
Do not turn historical notes, test counts or a successful demo into a whole-project
completion percentage or an implicit full-EKF2-parity requirement.
