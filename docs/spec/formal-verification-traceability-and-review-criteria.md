## Summary

This specification defines how requirements, implementation and verification evidence support an auditable, scoped claim. It applies the [Formal Verification Strategy](formal-verification-strategy.md) to numerical-core dependencies, ESKF operations and the registered runtime/replay/PX4 integration requirements.

The [traceability map](../../formal/agent-review/traceability-map.json) records the currently registered requirement IDs and evidence references. A mapping records where to review evidence; it is not evidence that the referenced proof ran or passed.

## Record ownership

The development approach supports future ISO certification work through explicit requirements, traceable design decisions and retained verification evidence. These records support review; they do not themselves establish conformity to a particular ISO standard.

| Record | Authoritative content |
|---|---|
| [Specifications and requirement IDs](verification-requirements.md) | Required behavior, sources, assumptions, configuration scope and acceptance obligations |
| [Architecture](eskf-software-architecture-and-verification-boundaries.md) and production source | Responsibility boundaries, current ownership and the implemented behavior to inspect |
| [Traceability map](../../formal/agent-review/traceability-map.json) | Requirement-to-source, model, harness and test references; no execution-success claim |
| Execution logs and validated review reports | Results for identified source/configuration/tool baselines, unresolved findings and evidence limitations |
| [Functional backlog](../../FEATURE_TODO.md) and [roadmap](../../ROADMAP.md) | Agreed functional work and architecture/verification plans, respectively; no substitute for acceptance evidence |

Review the chain from requirement to design/source, verification method, execution result and acceptance decision. Preserve the baseline and failed attempts when retaining a result. A dated summary may point to evidence, but cannot replace missing logs or establish that later source revisions pass. Current proof totals belong in reports derived from the registered scope, not duplicated in specifications or backlog tables.

## Proof boundary

| Evidence | Responsibility |
|---|---|
| Behavioral specifications | Define conventions, domains, equations and required success/failure behavior. |
| Lean 4 | Prove exact algebra and real-number properties of the mathematical models. |
| ESBMC | Check production C++ control flow, coefficient equations, statuses and output preservation under declared IEEE-754 profiles through the fixed-array proof backend. |
| Eigen and PX4 matrix tests | Check backend conformance and regressions, without proving backend internals. |
| Cppcheck | Run configured static analysis without modifying source. This is a quality gate, not mathematical proof evidence. |
| ASan/UBSan | Check memory safety and undefined behavior along executed test paths; this is not exhaustive symbolic coverage. |
| Agent review | Audit specification ↔ Lean ↔ C++ ↔ ESBMC correspondence and expose missing or inconsistent evidence. |

Lean and ESBMC provide separate evidence. There is no machine-checked real-to-float refinement theorem connecting them. Agent review does not create that bridge.

## Requirements

| Requirement family | Specification |
|---|---|
| `Q-*` and `SO3-*` | [Quaternion and SO(3) behavior](so3-and-unit-quaternion-abstraction-layer.md), including normalization, composition, rotation, Exp/Log and small-angle behavior. |
| `F-*` | [Supporting operations](verification-requirements.md#supporting-operations), [linear algebra](eskf-linear-algebra-abstraction-layer.md) and [scalar/failure semantics](scalar-ieee754-and-failure-semantics.md). |
| `E-*` | [State conventions](eskf-state-conventions.md) and [core requirements](verification-requirements.md#eskf-core): prediction, process noise, covariance, correction, injection/reset, observations and composed steps. |
| `R-*` | [Magnetic aiding](verification-requirements.md#magnetic-aiding) and [runtime requirements](verification-requirements.md#portable-runtime): sensor qualification, lifecycle, recovery and explicitly scoped replay/PX4 wiring. |

Process noise, covariance prediction, injection/reset, correction/solve, sensor models and combined-step requirements are registered alongside quaternion and prediction requirements. Registration does not mean every entry has complete evidence: runtime and module claims must retain their missing formal layers as gaps. Inspect each mapped entry and its execution evidence before reporting completion.

The map is not a complete estimator coverage denominator. Unmapped code must remain visible through the [function inventory and review queue](cpp-function-inventory-and-formal-proof-review-queue.md); the mandatory function-by-function review baseline is still a design task. Keep registered-requirement coverage separate from whole-source and configuration coverage.

## ESBMC assumptions

General quaternion/SO(3) and state/prediction profiles use both binary32 and binary64; prediction also covers both attitude modes. Each result is limited to its harness domain, fixed dimensions and configuration. Splitting coefficient checks must preserve the same quantified input domain and include every required coefficient.

General representation and coefficient checks allow arbitrary IEEE values. Arithmetic identity laws and matrix-sign equivalence have explicit finite-input premises; bounded/native-math binary32 witnesses remain separately scheduled. Unit norm is a premise only where required by the mathematical property, not a consequence of the C++ type name.

Normalization is checked compositionally: actual norm/root dispatch and raw divisions support checked vector normalization, which supports both quaternion constructors and then normalization/composition callers. Every boundary requires matching producer and caller evidence in both formats. Constructor/vector error precedence, arbitrary old outputs, inclusive thresholds and all legal aliases are checked without assuming success or a unit-norm residual.

Exp additionally separates its actual three-vector squared-norm producer from branch/control-flow verification. Summarized returns are unconstrained; the caller checks request arguments, eligibility, status propagation and exact publication. The actual producer and complete normalization chain must be verified before the composed source claim is accepted.

The [rotation harness](../../formal/cpp/esbmc/rotation.cpp) separately verifies actual matrix coefficients over arbitrary IEEE inputs, action wiring under an unconstrained matrix-producer summary, and the actual backend's finite-matrix basis property. Acceptance requires every producer row and caller profile in both formats. Producer finiteness for bounded q discharges the basis premise; basis examples alone do not establish arbitrary-vector floating-point behavior. These source-level claims do not establish numerical orthogonality, length preservation or inverse recovery.

General Exp/Log flow checks use arbitrary scalar returns, including non-finite values, to verify checked scalar-result use and failures without an accuracy assumption. The retained Log witnesses use a first-quadrant `atan2f` contract because ESBMC 8.4 lacks its body: interior results are nondeterministic in `[0,pi/2]` and axis values are fixed. Neither is a proof of deployed math-library accuracy. General Log checks describe behavior even outside the mathematical unit-quaternion domain; they do not establish a runtime unit-norm check or principal-range bound for those inputs.

### Prediction proof assumptions

The [prediction harness](../../formal/cpp/esbmc/prediction.cpp) owns per-entry domains, fixtures and scalar models; the [runner](../../scripts/verify_esbmc.sh) owns profile selection. These are verification evidence, not replacements for paper-based specifications.

General attitude-helper profiles quantify over arbitrary IEEE quaternion/rate coefficients and initial output, every finite positive `dt`, and every finite normalization threshold in `(0,1]`. Public callers cover all valid time bounds; INS translation covers all finite consumed vector coefficients, including subtraction and integration overflow. These are source-level coefficient/status/frame claims, not assumptions of unit norm or success. Root/trig contract models provide coarse enclosures, not numerical accuracy; root enclosures and model dispatch require independent checks. StandardMath fixtures remain separate execution witnesses, not deployed-libm proofs.

The public prior validator executes normally in caller proofs. Its independent status oracle must cover every IEEE quaternion and tolerance value, including non-finite inputs, invalid tolerance domains, squared-norm overflow and the inclusive residual boundary. Caller proofs must establish that rejection precedes prediction, preserves arbitrary outputs and does not repair the prior. Its runtime residual check does not establish an exact unit quaternion or a repeated-step numerical invariant.

The actual attitude helper must discharge every property used by its caller summary: finite coefficients on success, unchanged output on failure and unchanged inputs. Callers must establish the helper domain, propagate its result unchanged and preserve their entire output on failure, including in-place calls. Match all required coefficient, branch, scalar and attitude-mode profiles. A passing caller with a missing helper proof is a gap. Observers must be passive, and assert-then-assume proof cuts retain their assertion obligations.

Exp has an additional compositional boundary: the actual Exp body proves its branch equations, finite successful output and input/output preservation for every public threshold, including invalid values and positive thresholds above one; the actual attitude helper proves rate-to-increment scaling, unchanged Exp status/result use and checked Hamilton composition within prediction's parameter domain. The Exp summary assumes only finite coefficients on success and the proved frame conditions, not unit norm, accuracy or success. Both dependency levels require complete execution evidence before the public prediction claim is accepted.

These IEEE profiles use per-operator round-to-nearest, ties-to-even and subnormals, not FMA contraction, reassociation, flush-to-zero or other rounding modes. Full-domain source checks do not establish physical input validity, deployed-libm accuracy or numerical error bounds; copying checks do not claim NaN-payload preservation.

## Review and acceptance

Review actual specification, Lean, production C++ and harness bodies, not names or comments alone. Compare equations, conventions, input domains, failure behavior and configuration coverage. Proof assumptions must not contain the conclusions being checked.

Require successful execution evidence for every applicable profile and dependency. Use `./scripts/verify_esbmc.sh --list` for the planned inventory and runner `RESULT` records for completed invocations. Partial suites and shards must be combined before claiming full-suite completion; timeouts, missing results and incomplete unwinding are not passes.

Use `PASS` for aligned evidence within the declared claim, `GAP` for missing required evidence, `MISMATCH` for disagreement, and `NUMERICAL` for a missing in-scope floating-point error bound. Report target-specific exclusions separately; they do not excuse missing exact-algebra, coefficient, control-flow, status or failure-atomicity evidence.

Validate the complete report schema locally. An empty required reference layer cannot be classified as `PASS` or `NUMERICAL`; `PASS` requires source citations from every mapped layer and no error-severity findings. These mechanical checks establish evidence presence, not semantic correctness. The runner returns 0 for pass, 1 for failure and 2 for incomplete review. A Lean, ESBMC, Cppcheck or ASan/UBSan failure makes the overall result fail; the latter two checks do not change the requirement coverage denominator.

The agent returns semantic findings only. Its prompt includes Lean/ESBMC execution evidence, but no Cppcheck or sanitizer results or logs. The runner validates the audit, adds the actual results of all four checks and computes the overall result. These runner-owned fields are not accepted from the agent; tool failures cannot be overridden by its findings. The report's summary and limitations describe the semantic audit, not the independent quality checks.

Do not weaken specifications, narrow coverage silently or change production behavior merely to pass the review. Changes to required behavior, source, assumptions, configurations, tool models or dependencies require review of the affected evidence and callers. Distinguish editorial changes from changed obligations; identify which results remain applicable and which must be rerun, rather than treating an old pass as automatically current.

## Claims not made

Current mapped coverage does not establish deployed libm correctness, global floating-point error bounds, identical binary32/binary64 results, external matrix-backend internals, compiler correctness, generated machine code or ARM/STM32 operating environments.

These require separate numerical or target-specific evidence. An agent `PASS` is a scoped audit result, not certification of the whole estimator.

## Running the review

The runner executes Lean, ESBMC, Cppcheck, ASan/UBSan and the agent audit, in that order. ASan and UBSan share one instrumented build and one full CTest run through `verify_asan.sh`, reported as `asan_ubsan`. The sanitizer claim remains limited to that script's build configuration, runtime options and executed tests.

The runner reports total elapsed wall time as `HH:MM:SS` on completion or failure. Static analysis and sanitizer runs may generate build caches, binaries and diagnostics, but do not fix or format source code.

The runner requires Python 3 with `jsonschema` supporting Draft 2020-12, in addition to the verifier, static-analysis, sanitizer-build and agent prerequisites checked by the scripts. The offline regression test exercises report validation, execution-inventory checks and runner behavior without running the real analysis tools or agent. Reports missing any of the four tool results must be regenerated, not relabeled as having passed checks that were never run.

Run the tool regression tests, then the proofs and semantic audit with:

```sh
./tests/test_agent_review.sh
./scripts/run_agent_review.sh report.json
```
