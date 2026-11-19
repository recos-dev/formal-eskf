# Verification scope and evidence

This guide explains what the existing evidence can establish and how to review
it. It is not a live pass report. Requirements belong to [spec/](../spec);
[traceability-map.json](../../formal/agent-review/traceability-map.json) maps sources,
Lean declarations and ESBMC harnesses. A reference or inventory row is not an
executed proof. Consult the [review criteria](../spec/verification/assurance.md)
for acceptance and the [roadmap](../roadmap.md) for remaining work.

The [verification strategy](../spec/verification/assurance.md) defines the
separate claims of Lean, ESBMC, native/static checks and agent review. Some source
proofs use actual producers plus caller summaries; all matching dependencies and
profiles must be accepted together. The AST inventory discovers declarations;
it is neither a proof nor a mandatory review of every function.

## Open boundaries

- Full `HeightAiding` transactions remain outside the completed producer checks.
- Delayed IMU/observation/output owners and complete PX4/replay sequencing need
  their own source-composition evidence. Native owner/module tests are not that proof.
- Deployed matrix/libm accuracy, numerical error budgets and repeated-step
  invariants are not implied by finite outputs or exact-real theorems.
- Work-queue/consumer behavior, dependency allocation/locking, forced shutdown,
  full stack/response-time bounds and physical sensor/model validity require
  target/system evidence. SITL and recorded-log replay do not qualify hardware.
- AST-to-requirement baselining and one-result-per-symbol completeness remain V-03;
  the existing extractor does not enforce them.

The current module's functional boundaries are in [the integration guide](../../integrations/px4/README.md),
[PX4 timing](../spec/profiles/px4-sih-250hz.md) and the
[functional roadmap](../roadmap.md#functional-work). The SIH contact-model failure
tracked in F-07.1 is not
converted into a passing landing result by excluding the failed scenario.

The [INS codegen package](../../codegen/README.md) is selected by default through
`ESKF_CODEGEN`. It has separate exact-algebra, conditional numerical and
memory-safety checks. Production selection is tested by CTest; differential
fixtures alone do not cover it. Existing ESBMC core producer/caller profiles are
explicitly dense (`ESKF_CODEGEN=0`) except the explicit generated suites below.
Run native suites in both modes.

### Generated covariance prediction

`./scripts/verify_esbmc.sh pcov-generated` selects `ESKF_CODEGEN=1`. Its INS
caller profiles cover both scalar formats, attitude modes and whole-P aliases.
They execute the production caller with unconstrained pure producer returns,
checking complete F/P/R/Q forwarding, validation order, status propagation and
publication/rollback. Separate actual-kernel profiles check input frames and
mirroring on the same backend. Actual `publish_covariance` profiles check the
finite scan, exact copying without averaging and failure preservation; they
include all IEEE candidates and both aliases.

Acceptance also requires the matching `pcov` transition, quaternion-validation,
rotation and INS noise/storage producers, their Exp/normalization dependencies,
and the reused same-type matrix/finite/storage producers. Successful noise
construction supplies diagonal Q_i; the caller checks every F entry. Symmetric
PSD P remains a caller premise. Neither kernel nor publisher summaries assume
success, finiteness or symmetry. Exact kernel equations are covered by the
separate codegen algebra evidence, not by these unconstrained-return summaries.

### Generated reset, correction and transactions

`reset-generated` covers the actual INS reset caller in both reset modes and
both whole-P alias arrangements, with the same right-Jacobian/finite producers
as `reset`. Its kernel profiles check complete input frames and mirrored output;
it reuses the actual publisher from `pcov-generated`.

`correction-generated` separates three obligations for each explicit observation
layout: actual cross/innovation/Joseph kernel frames, actual dispatch forwarding,
and checked preparation/gain/Joseph callers. Body-vector dispatch must extract
the three correct skew-Jacobian entries. Joseph receives the full effective gain
and full correlated V; it does not assume an unconstrained optimal gain. Known
layouts require their exact H, supplied by the observation model/wrapper proofs.
Symmetric P/V and the structured A = I - KH are premises of the exact kernel
identities, not assumptions used to hide caller failures.

The caller summaries return arbitrary IEEE matrices and statuses. They are
paired with the existing exact-type validation, solve, matrix-product, subtraction,
finite and innovation-finalizer producers, plus an actual same-backend generated
publisher. Generated Joseph publication copies the mirrored result without a
second average. Logical stage counters in the common caller oracle describe
algebraic stages; they are not counts of executed dense operations.

`step-generated` checks production INS prediction, injection/reset and full
correction transactions, including state/P/error aliases and symbolic bias
permissions. Same-backend reset kernel and actual reset-frame profiles discharge
the terminal component's input/rollback contract in both reset modes. Injection,
observation model/wrapper, bias-mask and solver dependencies remain mandatory;
a component summary alone is not a closed composition proof. AHRS and arbitrary-H
products retain their dense producer obligations.

These are source behavior claims on the fixed-array proof backends. Exact-real
equivalence comes from the independent codegen algebra/Lean certificates;
finite-record IEEE error certificates do not establish a uniform deployment
error budget, repeated-step PSD or identical dense/generated failure boundaries.
Target numerical budgets and deployed backend/library internals remain separate.

## Running checks

From the repository root, choose checks affected by the change:

```bash
./scripts/verify_lean.sh --cache --wfail
./scripts/verify_esbmc.sh --list
./scripts/verify_cppcheck.sh
./scripts/verify_asan.sh
./scripts/format_cpp.sh --check
./tests/test_esbmc.sh
./tests/test_agent_review.sh
```

`--list` inventories scheduled profiles; it does not run them. The two shell
tests use offline fixtures by default. For real source checking and agent review,
consult each runner's `--help` and the review criteria; do not launch the entire
ESBMC suite for an editorial change. No formatting command above fixes source.

## Record ownership and reuse

Keep development logs, trial configurations and per-run investigation notes in
ignored local output. They are useful for local tracking, not standing project
specifications. A durable design decision retains only its reason, affected
contract and unresolved limitation; do not copy the experiment diary into Git.

An accepted result needs source/dependency/tool identity, configuration, exact
command, domain/assumptions, outcome and retrievable artifacts. A release must
retain that evidence outside disposable local output, for example in its CI or
release artifact bundle. Documentation links alone do not supply missing logs.
A dirty-tree run needs a snapshot identity, not a claim about unchanged HEAD.

Reassess changed sources, assumptions, models and configurations before reusing
results. Keep failed acceptance cases and counterexamples visible in the relevant
issue/analysis. Missing execution, timeout, OOM or incomplete unwinding stays a
gap. Printed ESBMC programs lose information and are not an equivalence oracle;
do not revive the withdrawn text-based deduplication optimization.

## Proof execution resources

Run heavy profiles sequentially, without simultaneous builds or full suites.
Before running, check an OS-enforced aggregate memory/wall-time limit and host
headroom. Internal verifier limits and per-process RSS sampling are not a
host-wide cap; measure process and aggregate use separately.

The proposed profile for a 16 GB host is a 6 GiB aggregate cap, swap disabled,
and at least 4 GiB available beyond the cap. This is an execution prerequisite
to establish and check, not enforcement supplied by the verifier script or a
measured worst-case proof cost. Review producer/caller decomposition when a
query exceeds the budget; never narrow the domain or disable unwinding assertions
to obtain a pass. Record the enforced limits and measured costs with the run.

## Running the agent review

The runner requires Python 3 with `jsonschema` supporting Draft 2020-12, plus the
verifier, analysis, sanitizer-build and agent prerequisites checked by the scripts.
The offline regression checks report validation and runner behavior without
executing real verifiers or an agent. Reports missing required tool results must
be regenerated, not relabeled as having passed unrun checks.

```bash
./tests/test_agent_review.sh
./scripts/run_agent_review.sh report.json
```
