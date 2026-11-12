# Roadmap

## Current work tracks

Reconciled on 2026-11-12 against `a5b460f`. Functional priorities and completion
states live only in [FEATURE_TODO.md](FEATURE_TODO.md). Its fixed F-01–F-07 groups
preserve the previous functional list; architecture/formal work (previous item 8)
is separated below. These planning IDs are not verification requirement IDs.

| ID | Work | Status and completion criterion |
|---|---|---|
| A-01 | Architecture and refactor proposal | Pending review. Map current core/runtime/PX4 ownership, state/history, call order and external effects. Propose only justified moves, with migration/test/proof boundaries and resource budgets. Submit the proposal before structural code changes; this is not approval for a blanket refactor. |
| V-01 | Runtime/module proof plan | Pending, after A-01. Reconcile existing requirements/evidence; identify component invariants, caller assumptions and integration obligations. Separate mathematical, C++ control-flow, platform and physical claims. Define compositional ESBMC boundaries with measured per-profile time/peak memory for the 16 GB host before expanding expensive runs. |
| V-02 | Design and execution-evidence pilot | Pending, after the scope review. Use the E-STEP-PRED pilot below; add source-linked native evidence without treating it as new algebraic proof or rewriting all verification tools. Expand only after the pilot is reviewed. |
| V-03 | AST mapping and review completeness | Design pending. AST inventory generation already exists. Review a versioned symbol-to-requirement baseline and completeness rules before connecting it to mandatory AI review; inventory generation alone does not prove coverage. |

Small functional integrations keep their existing contracts and tests current.
They do not close V-01 or imply proof of the PX4 module. A-01 review is required
before major ownership/timing refactors, including F-04; do not silently replace
the functional queue with a proof/tooling queue.

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
