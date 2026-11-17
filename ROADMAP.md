# Roadmap

Functional scope and ordering belong to [FEATURE_TODO.md](FEATURE_TODO.md).
This file owns architecture, verification and tooling work. Planning IDs are
stable; they are not proof requirements or a whole-estimator completion measure.
The document index is [docs/README.md](docs/README.md).

## Current work tracks

| ID | Work | Status and completion criterion |
|---|---|---|
| A-01 | Architecture and refactor | Accepted module refactor completed at `3e29d33`; delayed-fusion and configuration integration followed. Maintain the architecture contracts. Target resources, platform dependencies and numerical qualification remain open under F-07/V-01. |
| V-01 | Runtime/module proof plan | Core extensions and the subsequent portable runtime proof batches have scoped evidence; see [verification scope](docs/guides/verification.md). The 2026-11-16 continuation covers magnetic fusion/reset and accelerometer sampling. Full PX4 owner/composition, numerical and target claims stay open; the functional queue is unchanged. |
| V-02 | Design and execution-evidence pilot | Pending, after the scope review. Use the E-STEP-PRED pilot below; add source-linked native evidence without treating it as new algebraic proof or rewriting all verification tools. Expand only after the pilot is reviewed. |
| V-03 | AST mapping and review completeness | Design pending. AST inventory generation already exists. Review a versioned symbol-to-requirement baseline and completeness rules before connecting it to mandatory AI review; inventory generation alone does not prove coverage. |
| V-04 | Analysis inventory and version validity | Systematic analysis pending: use FHA/FMEA/FTA as the primary methods, review scope/applicability, pilot IMU timing, then add dependency snapshots, offline completeness checks and per-release dispositions. Include common-cause dependencies and supporting timing/numerical evidence; analysis validity, evidence applicability and residual-risk acceptance remain distinct. Drafts await separate review; no inventory checker or release gate is implemented yet. |
| V-05 | Generated/dense equivalence proof | Queued for later consideration: establish a reviewed equivalence contract and prove the generated production kernels against the dense implementation; see the scope below. Experiments and benchmarks are supporting evidence, not equivalence proofs. |

## Completed refactors

| Work | Accepted scope / durable record |
|---|---|
| RF-01–RF-05 | Numerical core injection/reset, correction, observations, innovation and combined acceptance; completion `cc2e1d1`. |
| RT-01–RT-11 | Portable runtime responsibilities, callers and interface migration; final batch `333561a`, followed by the accepted runtime file consolidation. This does not establish target timing or module proof. |
| A-01 batches 1–6 | Single-writer PX4 ownership, input/admission, aiding, publication and lifecycle refactor; completion `3e29d33`. Whole-module formal and target acceptance remain separate. |
| DC-01–DC-08 | Nine-group core call-graph audit, followed by 22 implemented and 6 retained candidates. Three sparse-algebra proposals were retained. The completed audit and run logs are local tracking records; DC-09 is the reusable-tool follow-up. |

Completed implementation diaries are not another active queue. Preserve equations,
IEEE evaluation/status precedence, aliases, failure atomicity and defaults in any
later refactor; changed behavior requires an explicit reviewed fix. Update affected
requirements, source references and producer/caller evidence together.

## V-01: remaining proof boundaries

Use the [requirements](docs/spec/verification-requirements.md),
[traceability map](formal/agent-review/traceability-map.json) and actual runner
results. Historical extension runs are not fresh execution against later revisions.
See the [evidence boundaries](docs/guides/verification.md).

- Complete the delayed module owners and their real callers: `ImuHistory`,
  `ObservationHistory`, `OutputPredictor` and `FormalEskf` sequencing. Their native
  scenario tests do not supply missing source proofs.
- Preserve explicit abstraction dependencies. A caller summary requires matching
  actual-producer evidence; no rewritten estimator may stand in for the production
  path. A fixed-array backend result does not prove deployed PX4 matrix or libm.
- Treat owner history/feedback and numerical transactions separately. Sampling may
  consume input before a later failure; the entire callback is not atomic.
- Numerical error bounds, repeated-step behavior, dependency concurrency/allocation,
  forced shutdown and target timing remain separate obligations. Do not change
  production behavior merely to make a proof feasible.

For each proof retain source/configuration/tool identity, domain, invariants,
aliases, assumptions, invocation, result and resource use. Missing execution,
timeout or incomplete unwinding is a gap. See [verification scope and resource limits](docs/guides/verification.md).

### Resource policy for the 16 GB host

Run new heavy profiles sequentially, without simultaneous builds or full suites.
Check an OS-enforced aggregate memory/wall-time limit and available host headroom
before running. Internal verifier limits and RSS sampling are not a host-wide cap.
The earlier proposed envelope was 6 GiB aggregate, swap disabled, with at least
4 GiB available beyond it; enforcement is an execution prerequisite, not a
feature already supplied by the verifier script. Profile costs must be measured.
Never narrow a proof domain or disable unwinding assertions to fit a budget.

## V-02: design and execution-evidence pilot

Pending. Start with `E-STEP-PRED`; do not reorganize all verification tools.

1. Link one concise architecture/unit design to the existing requirements and
   production symbols: ownership, call order, scratch storage, publication and
   failure boundaries. Cite equations instead of duplicating specifications.
2. Give existing native cases stable IDs and profiles. Cover same-prior prediction,
   first failure, arbitrary sentinel outputs and supported aliases. Reject unknown
   or duplicate selections; do not infer coverage from test filenames.
3. Extend the existing map/report schema in place with source/build/tool identity,
   command, result, duration and log hashes. A release needs reproducible reviewed
   source; dirty-tree evidence needs its own snapshot identity.
4. Test completeness offline: missing cases/profiles, stale references, duplicate
   results or modified runner-owned outcomes must not produce a complete PASS.
5. Expand to correction/injection/reset only after pilot review. Native tests,
   mathematical proofs, bounded source checks and AI findings stay distinct.

Artifacts must remain retrievable for the claimed release; ignored local output
alone is not a published evidence archive. Replay without an independent oracle
and acceptance limits remains a demonstration.

## V-03: AST mapping and review completeness

`scripts/inventory_cpp.py` generates declarations and source/configuration identity
for the configured roots; it does not enforce one review result per function.
First review a versioned symbol-to-requirement baseline and profile completeness,
including unclassified/new/removed symbols. Only then connect it to mandatory
review. Hash equality is identity evidence, not proof of semantic correctness.

## V-04: analysis inventory and version validity

Review an FHA/FMEA/FTA inventory covering hazard seeds, analysis boundaries,
dependency records and version dispositions. Pilot IMU timing before implementing
an offline completeness checker or release gate. Drafts are not an adopted
analysis baseline; no automated safety acceptance exists.

## DC-09 — Reusable call-graph review script

Queued on 2026-11-18 for future refactors; not started and does not reorder the
current F-06.2 work. Turn the previous defensive-check/computation call-graph
extraction into a reusable script under `scripts/`, reusing the existing Clang
inventory infrastructure where appropriate.

- Generate explicit caller → callee edges with call-site locations, per-function
  incoming/outgoing lists, readable group diagrams and a numbered review inventory
  for defensive checks and repeated computations. Keep the analyzed entry points,
  dependencies and build profiles explicit; report unresolved/indirect calls.
- Record Git revision, source hashes, compile configuration and tool version;
  place generated snapshots under `output/`. Make regeneration reproducible.
- Compare with a previous snapshot: identify added, removed and changed entries,
  preserve review identities/conclusions, and mark affected conclusions stale
  rather than silently carrying them forward or overwriting them.
- Retain the prior review method: inspect each call chain's premises, value/alias
  changes, status precedence and failure outputs. A repeated predicate or graph
  edge alone does not prove redundancy or authorize removing a check.
- Validate extraction against known calls/branches and representative build
  profiles. Acceptance is a rerunnable graph and review queue with documented
  coverage limits, not automatic semantic review or automatic code modification.

## V-05 — Generated/dense equivalence proof

Queued on 2026-11-18 for later consideration, alongside the existing codegen
evaluation. This does not reorder current implementation work or authorize
replacing the dense path.

- Fix the compared dense/generated revisions, derivation, generated artifacts,
  state/frame conventions, scalar/backend and compiler modes. Use this project's
  formulation and make generation reproducible.
- Define the equivalence claim before proving it: exact-real algebraic equality,
  IEEE-754 result equivalence and observable API behavior are distinct obligations.
  Preserve discovered counterexamples. If reordering prevents bitwise equality,
  report it; a bounded-error alternative requires an explicitly reviewed contract,
  not a silently widened test tolerance or narrowed input domain.
- Prove the actual generated kernels, beginning with individual operations and
  then their integration. Include validation/status precedence, non-finite and
  overflow behavior, aliasing and failure-output preservation. Mathematical
  equality alone does not establish C++ implementation equivalence.
- Connect each kernel to requirements, source identities and proof evidence;
  report completed obligations, assumptions and remaining gaps. Distinguish proof
  of specific generated artifacts from proof of the generator itself. Differential
  tests, high-precision comparisons and benchmarks remain separate evidence.
