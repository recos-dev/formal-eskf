## Summary

Define how specifications, mathematical proofs and C++ verification evidence support a scoped formal claim. This applies the [Formal Verification Strategy](formal-verification-strategy.md) to the mapped quaternion/SO(3), ESKF state and nominal-prediction requirements.

The [traceability map](../../formal/agent-review/traceability-map.json) records the currently registered requirement IDs and evidence references. A mapping records where to review evidence; it is not evidence that the referenced proof ran or passed.

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
| `E-STATE` | [State conventions](eskf-state-conventions.md). |
| `E-PRED` | [Nominal prediction requirements](verification-requirements.md#e-pred-nominal-prediction-requirements), including paper references and eight behavior clauses. Prior-validity policy and wider-domain evidence remain open; mapping the specification alone does not close them. |

The map is not a complete ESKF coverage denominator. Process noise, covariance prediction, injection/reset, correction/solve, sensor models and combined-step obligations remain outside the registered proof scope. Unmapped code must remain visible through the [function inventory and review queue](cpp-function-inventory-and-formal-proof-review-queue.md).

## ESBMC assumptions

Quaternion/SO(3) profiles use binary32; state/prediction profiles also use binary64 and both attitude modes. Each result is limited to its harness domain, fixed dimensions and configuration. Splitting coefficient checks must preserve the same quantified input domain and include every required coefficient.

Quaternion representation profiles use finite coefficients in `[-1,1]`. Unit norm is a premise only where required by the property, not a consequence of the C++ type name.

The Log harness uses a first-quadrant `atan2f` contract because ESBMC 8.4 lacks its body: interior results are nondeterministic in `[0,pi/2]` and axis values are fixed. This checks use of the scalar result, not the deployed math library.

### Prediction proof assumptions

The [prediction harness](../../formal/cpp/esbmc/prediction.cpp) owns per-entry domains, fixtures and scalar models; the [runner](../../scripts/verify_esbmc.sh) owns profile selection. These are verification evidence, not replacements for paper-based specifications.

General helper profiles use raw quaternion coefficients in `[-1,1]`, corrected rates in `[-2,2]`, `dt` in `[2^-10,1]` and minimum norm `1/8`. They assume neither unit norm nor success. Root/trig contract models provide coarse enclosures, not numerical accuracy; root enclosures and model dispatch require independent checks. StandardMath fixtures are separate execution witnesses, not general-input or deployed-libm proofs.

The actual attitude helper must discharge every property used by its caller summary: finite coefficients on success, unchanged output on failure and unchanged inputs. Callers must establish the helper domain, propagate its result unchanged and preserve their entire output on failure, including in-place calls. Match all required coefficient, branch, scalar and attitude-mode profiles. A passing caller with a missing helper proof is a gap. Observers must be passive, and assert-then-assume proof cuts retain their assertion obligations.

These IEEE profiles use per-operator round-to-nearest, ties-to-even and subnormals, not FMA contraction, reassociation, flush-to-zero or other rounding modes. Bounded inputs do not cover every accepted runtime input; copying checks do not claim NaN-payload preservation.

## Review and acceptance

Review actual specification, Lean, production C++ and harness bodies, not names or comments alone. Compare equations, conventions, input domains, failure behavior and configuration coverage. Proof assumptions must not contain the conclusions being checked.

Require successful execution evidence for every applicable profile and dependency. Use `./scripts/verify_esbmc.sh --list` for the planned inventory and runner `RESULT` records for completed invocations. Partial suites and shards must be combined before claiming full-suite completion; timeouts, missing results and incomplete unwinding are not passes.

Use `PASS` for aligned evidence within the declared claim, `GAP` for missing required evidence, `MISMATCH` for disagreement, and `NUMERICAL` for a missing in-scope floating-point error bound. Report target-specific exclusions separately; they do not excuse missing exact-algebra, coefficient, control-flow, status or failure-atomicity evidence.

Validate the complete report schema locally. An empty required reference layer cannot be classified as `PASS` or `NUMERICAL`; `PASS` requires source citations from every mapped layer and no error-severity findings. These mechanical checks establish evidence presence, not semantic correctness. The runner returns 0 for pass, 1 for failure and 2 for incomplete review. A Lean, ESBMC, Cppcheck or ASan/UBSan failure makes the overall result fail; the latter two checks do not change the requirement coverage denominator.

The agent returns semantic findings only. Its prompt includes Lean/ESBMC execution evidence, but no Cppcheck or sanitizer results or logs. The runner validates the audit, adds the actual results of all four checks and computes the overall result. These runner-owned fields are not accepted from the agent; tool failures cannot be overridden by its findings. The report's summary and limitations describe the semantic audit, not the independent quality checks.

Do not weaken specifications, narrow coverage silently or change production behavior merely to pass the review. Changes to source, assumptions or dependencies require affected evidence to be reviewed again.

## Claims not made

Current mapped coverage does not establish deployed libm correctness, global floating-point error bounds, identical binary32/binary64 results, external matrix-backend internals, compiler correctness, generated machine code or ARM/STM32 operating environments.

These require separate numerical or target-specific evidence. An agent `PASS` is a scoped audit result, not certification of the whole estimator.

The runner executes Lean, ESBMC, Cppcheck, ASan/UBSan and the agent audit, in that order. ASan and UBSan share one instrumented build and one full CTest run through `verify_asan.sh`, reported as `asan_ubsan`. The sanitizer claim remains limited to that script's build configuration, runtime options and executed tests.

The runner reports total elapsed wall time as `HH:MM:SS` on completion or failure. Static analysis and sanitizer runs may generate build caches, binaries and diagnostics, but do not fix or format source code.

The runner requires Python 3 with `jsonschema` supporting Draft 2020-12, in addition to the verifier, static-analysis, sanitizer-build and agent prerequisites checked by the scripts. The offline regression test exercises report validation, execution-inventory checks and runner behavior without running the real analysis tools or agent. Reports missing any of the four tool results must be regenerated, not relabeled as having passed checks that were never run.

Run the tool regression tests, then the proofs and semantic audit with:

```sh
./tests/test_agent_review.sh
./scripts/run_agent_review.sh report.json
```
