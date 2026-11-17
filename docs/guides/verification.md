# Verification scope and evidence

This guide explains what the existing evidence can establish and how to review
it. It is not a live pass report. Requirements belong to [spec/](../spec);
[traceability-map.json](../../formal/agent-review/traceability-map.json) maps sources,
Lean declarations and ESBMC harnesses. A reference or inventory row is not an
executed proof. Consult the [review criteria](../spec/formal-verification-traceability-and-review-criteria.md)
for acceptance and the [roadmap](../../ROADMAP.md) for remaining work.

The [verification strategy](../spec/formal-verification-strategy.md) defines the
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
[PX4 timing](../spec/px4-timing-profile.md) and the F-series backlog. In particular,
the SIH contact-model failure tracked in F-07.1 is not
converted into a passing landing result by excluding the failed scenario.

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

For new heavy proofs use sequential, resource-bounded pilots on the 16 GB host.
Measure process and aggregate memory separately; the historical 55.11 GiB OOM
was a failed monolithic query, not necessary RAM for every proof. Review the
producer/caller decomposition instead of weakening the domain to obtain a pass.
