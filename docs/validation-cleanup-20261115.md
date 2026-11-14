# Defensive-check cleanup acceptance

Baseline: `9efbf7763f5477d9653f1e333011470b7a0c5480`. The fixed implementation
sequence is DC-01–DC-08 in [ROADMAP.md](../ROADMAP.md). Numbers below preserve
the review list; they are work items, not proof requirements or measured speedups.

| Item | Audit origin | Candidate | Disposition |
|---|---|---|---|
| 1 | G01 | Reuse finite input premises in symmetry checks | Implemented in DC-02 |
| 2 | G01-C02 | Pairwise symmetry residual | Implemented in DC-02 |
| 3 | G01-C01 | Forward gain solve output | Implemented in DC-03 |
| 4 | G01-C03, G06-C8, G08-C7 | Same-call PHt/S reuse after gating | Pending |
| 5 | G01-C04 | Hoist bias permissions out of column loops | Implemented in DC-01 |
| 6 | G02-C1 | Reuse validated prior quaternion | Implemented in DC-05 |
| 7 | G02-C2 | AHRS nominal output forwarding | Implemented in DC-03 |
| 8 | G02-021.1 | Acceleration finite check subsumption | Implemented in DC-04 |
| 9 | G03-C1 | Diagonal-only process-noise result scan | Implemented in DC-01 |
| 10 | G02-C3, G03-C2, G04-C1–C3 | Shared prediction validation and mathematics | Retained in DC-05 |
| 11 | G04-C4 | Scan only variable F_x blocks | Implemented in DC-01 |
| 12 | G04-C5 | Reuse -R | Implemented in DC-01 |
| 13 | G04-C6 | Forward covariance prediction output | Implemented in DC-03 |
| 14 | G05-C1–C2 | Reuse injection/reset input premises | Retained in DC-06 |
| 15 | G05-C3 | Share Exp/Jr angle intermediates | Retained in DC-06 |
| 16 | G05-C6 | Forward covariance reset output | Implemented in DC-03 |
| 17 | G05-C8 | Avoid copied-and-overwritten INS candidate | Implemented in DC-03 |
| 18 | G06-C1 | Reuse runtime innovation premises | Pending |
| 19 | G06-C2 | Simplify innovation range comparisons | Implemented in DC-04 |
| 20 | G07-C1, C5 | Construction-valid H and delayed construction | Implemented in DC-01 |
| 21 | G07-C2 | Accel h/z finite check subsumption | Implemented in DC-04 |
| 22 | G07-C3, G08-C4 | Finite vector nonzero checks | Implemented in DC-01 |
| 23 | G08-C1 | Build runtime magnetic H from existing h | Implemented in DC-01 |
| 24 | G08-C2–C3 | Share magnetic model preparation across gate/core | Pending |
| 25 | G08-C6 | Share magnetic prediction/projection rotation | Pending |
| 26 | G08-C5 | Magnetic h/u finite check subsumption | Implemented in DC-04 |
| 27 | G09-C1 | Direct constant selector Jacobians | Implemented in DC-01 |
| 28 | G09-C3 | Reuse height local H | Implemented in DC-01 |

Three separate numerical proposals are sparse covariance propagation (G04-C9),
block covariance reset (G05-C9) and selector-specific PHt/S (G09-C8). Their
disposition belongs to DC-08; no numerical rewrite is presumed equivalent.

## Evidence and acceptance

Local commands, logs and baseline snapshots live in the ignored
`output/validation-cleanup-20261115/` directory. Each accepted batch records its
tests and limitations below. Existing mathematical Lean statements are unchanged
unless explicitly recorded. A retained candidate must explain the contract or
numerical reason; a missing proof is not a pass.

## Batch records

### DC-01: construction and local reuse — accepted

- Changed: diagonal-only Q_i result checks (3/12 entries); three 3x3 F_x
  result checks (27 entries); shared negative R; INS acceleration H is built
  after h/r checks without another 45-entry scan; finite gravity/field vectors
  use direct zero comparisons; bias permissions are checked once per row;
  constant selectors write their ones directly; height reuses H; magnetic
  runtime builds H from its existing prediction.
- Correctness review: all skipped entries are construction-known finite values.
  The dt scaling, matrix-product order, early input failures, bias/projection
  ordering and final output publication remain unchanged. E-PCOV's harness now
  executes result-block scans instead of abstracting the complete predicate;
  its independent oracle and full transition-entry assertions are preserved.
- Behavior review: baseline/current comparison covers 29,952 operation calls
  on Eigen and PX4 Matrix, float/double, INS/AHRS and four output alias modes.
  It includes finite extremes, signed zero, subnormals, infinities and NaNs;
  all 1,248 cumulative status/state/P/diagnostic digests match. This is native
  evidence, not exhaustive proof; NaN payload and fenv are not compared.
- Completed: 24 default native test executables (Eigen/PX4/CMSIS-DSP), 8 alternate Q/R/M executions,
  8 ASan/UBSan tests (LeakSanitizer disabled in this sandbox), configured
  Cppcheck unix32/unix64, and all six E-NOISE profiles. Two magnetic-count probes
  preserve core/runtime output equality and show reference-field actions 3→2
  and residual-helper finite inspections 22→16. These counts are not timings.
- Completed: all eight modified INS E-PCOV caller profiles and 38 affected
  E-OBS profiles. A negative control deliberately removes the force-block
  finite check in an isolated copy: ESBMC detects the changed validation/status
  path. The production source and real oracle are not weakened.
- Final proof execution: 66/66 selected profiles pass (6 E-NOISE, 16 E-PCOV
  callers, 6 matching OpaqueMath noise producers, 38 affected E-OBS profiles).
  Formatting and all mapped implementation/proof symbol references pass. No requirement
  assertion was weakened; the deliberately broken copy is rejected.
- Self-review: changes are correct within the recorded backend/scalar/failure
  contracts; observed functionality is unchanged. Commit accepted. Runtime
  tests and replay-related checks remain native evidence, not runtime proofs.
- Initial native commands corrected a nonexistent noise test filename and a
  missing Eigen include used by the PX4 covariance test oracle. The initial
  CTest selector matched no tests; the corrected selectors executed all eight
  intended sanitizer tests. Failed setup logs remain local; no assertion was
  changed to make these runs pass.
- Harness symbol references are retained. The traceability summary records the
  changed E-PCOV boundary; no Lean theorem, target-libm claim or backend proof
  has been added. The original call-graph audit remains historical baseline
  evidence; its source hashes must not be presented as current acceptance.


### DC-02: symmetry validation — accepted

- Changed: stream each residual coefficient and the original row-major maximum
  instead of constructing transpose and difference matrices. Public
  `is_symmetric` retains its input/tolerance checks. Correction's two inputs and
  the solve's system use one private finite-input helper after their own checks.
- Correctness review: both subtraction directions, diagonal differences,
  absolute values and reduction order remain unchanged. Finite operands can
  still overflow during subtraction; that residual remains rejected. Caller
  harnesses now assert the helper's finite-input premise instead of assuming it.
  Tolerance is checked at the public entry or is the literal zero at callers.
- Behavior review: all 1,248 cumulative baseline/current comparison digests match;
  no API, status, alias, tolerance or output-publication change. The three
  repeated complete input scans disappear on the accepted correction path;
  this does not remove the first input validation or claim a measured speedup.
- Validation: 92/92 scoped ESBMC profiles (16 F-LINALG, 20 correction, 56 solve),
  13 native executables across Eigen/PX4/CMSIS-DSP, 3 ASan/UBSan tests, configured
  unix32/unix64 Cppcheck, formatting and mapped-symbol checks pass. The first
  CTest regex omitted the differently named SPD test; it was run separately.
- A prototype lemma for eliminating the second subtraction direction exceeded
  the 4 GiB solver limit. That prototype is not adopted and is not proof
  evidence. The implemented stream preserves both directions and needs no new
  floating-point antisymmetry premise; no switch to coefficient equality or
  relaxed tolerance is made.
- Self-review: correct under the existing contracts; observed functionality
  unchanged. Commit accepted. Lean mathematics and runner profile inventory are
  unchanged; source-level helper preconditions are covered by the caller checks.

### DC-03: transaction outputs — accepted

- Changed: forward the final solve/covariance result to the caller's output;
  forward AHRS attitude output directly; initialize the INS injection candidate
  without copying the old state that will be completely overwritten. Keep
  nominal scratch storage whenever covariance work can still fail.
- Correctness review: forwarding depends on the final callee's unchanged-on-
  failure contract. Matching actual producer checks cover right solve,
  prediction/reset and attitude aliases. The correction transaction uses a
  different proof backend: its actual reset frame now explicitly checks failure
  preservation too. Intermediate summaries may still overwrite scratch on
  failure. All caller rollback, status, order and input-preservation assertions
  remain. The specification now states this permitted final-callee composition
  instead of prescribing unnecessary scratch everywhere.
- Validation: 306/306 selected ESBMC profiles pass: 100 step, 40 prediction,
  12 correction, 50 right solve, 4 INS injection, 16 reset and 84 correction-step
  profiles. Native tests pass in 20 executions across Eigen/PX4/CMSIS-DSP,
  including alternate approximation/projection configuration; 4 ASan/UBSan
  tests, unix32/unix64 Cppcheck, formatting and ShellCheck pass. Both offline
  verification-tool suites pass; all 589 mapped implementation/proof symbol references
  resolve. Attitude producer alias coverage adds 16 profiles (1554 total,
  still 127 entry points); this is not 1554 mathematical requirements.
- Behavior review: all 1248 baseline/current digests match over 29,952 native
  calls. Public APIs, arithmetic, complete-output aliases and failure behavior
  are unchanged. Preservation covers values, signed zero and NaN classification,
  not NaN payload or floating-point exception flags. No whole-backend or target
  timing claim follows from these checks.
- Failed attempts are retained locally: reference-selected alias harnesses
  exhausted the existing prediction memory cap; the final concrete compile-time
  alias representation passes without removing numerical/failure assertions.
  An experimental NaN-payload comparison exceeded the preservation contract and
  was removed. A running shell observed an edited runner and failed parsing;
  its profile was rerun after the runner stabilized. Interrupted or failed runs
  are not counted as passes. Solver memory limits were not raised (prediction
  already uses 8 GiB; other suites retain their existing limits).
- Self-review: final-callee forwarding is correct under the independently checked
  frames; observed functionality is unchanged. Commit accepted. Lean equations
  are unchanged. Local manifest: `dc03-manifest.json` in the evidence directory.

### DC-04: numerical implications — accepted

- Changed: retain final p/v checks instead of separately rescanning acceleration;
  retain residual checks instead of h/z scans; retain the complete Down projector
  check instead of its input scan; group each innovation variance's lower-bound
  comparisons while retaining its original error status and axis order.
- Correctness review: F-SCALAR now proves, for arbitrary binary32/binary64 operands,
  that finite addition/subtraction/multiplication results require finite operands.
  Thus finite v implies finite acceleration, finite residual implies finite h/z,
  and finite diagonal products of the projector imply finite Down coordinates.
  Division is not covered by this implication. The converse is false: finite
  operands can overflow, so final product/gain checks remain.
- Validation: 62 affected ESBMC profiles pass (28 prediction, 32 magnetic/acceleration
  observations, 2 scalar primitives), retaining the original independent caller/status oracles.
  The scalar cases are rerun after formatting. Native tests pass in 24 executions
  across Eigen/PX4/CMSIS-DSP, including alternate modes and runtime consumers;
  4 ASan/UBSan tests, unix32/unix64 Cppcheck and formatting pass. Two initial
  native invocations used incorrect source paths/names; corrected runs execute
  the intended tests. Initial model selection ran 32 position/velocity profiles;
  self-review corrected selection to models 4–7. The earlier passes are extra
  regression evidence, not the affected-model acceptance. No failed setup is
  counted as acceptance evidence.
- Behavior review: all 1248 baseline/current digests match over 29,952 calls.
  Inputs, formulas, per-axis failure precedence, aliases and output rollback
  are unchanged. Rejected paths may evaluate an additional subtraction or
  outer product before returning the same non_finite_result; NaN payload,
  floating-point flags and trapping profiles are outside this comparison.
- Self-review: correct under the recorded IEEE profile; observed
  functionality is unchanged. Commit accepted. Local arithmetic implication
  evidence does not close the outstanding complete E-INNOV/E-MAG-TILT proof
  boundaries, and does not constitute a deployed-backend proof. Lean is unchanged.

### DC-05: prediction validation ownership — accepted

- Changed (#6): the existing private attitude helper requires finite q and
  validated time/norm parameters. The two public overloads retain prior
  validation; remove only the repeated four-coefficient private scan. Rate,
  increment, normalization and result checks remain.
- Correctness review: actual public caller proofs assert the finite-q premise;
  actual helper proofs use precisely that private premise, retaining their
  equations, alias and rollback assertions. Public non-finite input coverage
  is retained. The rejection caller is paired with the actual same-backend
  finite-q/non-finite-rate producer (the caller uses its checked dt/norm values).
  It asserts the producer's delegation domain instead of assuming rejection.
- Validation: 76 selected caller/helper profiles pass across both scalar formats,
  prediction modes, coefficients and aliases. Twelve native default/alternate
  Eigen/PX4 test executions, prediction ASan/UBSan, formatting, ShellCheck and
  both offline verification-tool suites pass. All 1248 baseline/current digests
  match over 29,952 calls. Runner inventory remains 1554 profiles/127 entry points.
  The new offline check protects the actual-producer/summary-caller pairing.
- Initial monolithic public rejection execution exceeded its existing 120 s
  cap with CVC5, Bitwuzla and Z3; the prior source passed. Full incremental SMT
  also timed out. The accepted composition preserves the same input domain and
  original public assertions, and passes without increasing limits. A deliberate
  isolated removal of public quaternion rejection fails the new caller-premise
  assertion. Failed attempts and this negative control remain local evidence;
  neither is counted as a successful production check.
- Retained (#10): keep independently checked nominal/covariance/noise operations.
  A combined-only prepared path could reuse dt, bias differences, old R and the
  default-mode Exp result. It would also need to carry stage-specific outcomes
  while preserving nominal-first failure order, old-state linearization and the
  distinct normalized-Euler / I-hat(theta) approximation rules. Choose the direct
  checked composition over a second prepared execution path in this cleanup.
  Repeated mathematics remains; it is not claimed impossible or eliminated.
  Core determinism is already specified, but current opaque scalar summaries
  would need relational producer evidence for a future shared implementation.
- Self-review: the local private-premise change is correct under the paired
  contracts; observed public functionality is unchanged. Commit accepted.
  No new public API, validation token or prediction workspace is introduced.

### DC-06: injection/reset boundaries — retained, review complete

- Retained (#14): use the same checked injection/reset/SO(3) paths for standalone
  and correction callers. Correction establishes finite error and Joseph P,
  but not all old nominal fields; standalone inject_and_reset has not validated
  P when it reaches reset. Reset consumes theta only, while injection also
  consumes the additive error fields. Bypassing their checks requires distinct
  prepared paths with different premises, not one common unchecked entry.
  Choose the existing independently callable operations over those extra paths.
  The repeated input scans on qualified correction paths are acknowledged.
- Retained (#15): Exp and J_r have different formulas, Taylor thresholds and
  failure stages. Exp switches at machine epsilon in theta squared; J_r uses
  1/256. At theta=1/64 only Exp uses trig; at 1/4 both use trig. Approximate reset
  has no J_r trig at all. Sharing raw intermediates is possible, but would
  couple these independent operations through a new prepared-angle interface.
  Keep them separate; normalized quaternion coefficients cannot replace raw
  trig, and the two normalizations act on different operands.
- Validation: four native probe executions on current unchanged injection/SO(3)
  sources (Eigen/PX4, reset=0/1, each float/double) preserve status precedence,
  consumed-field differences, mode-dependent domains and rollback. They confirm
  the recorded norm/trig counts; those are operation counts, not target timings.
- Self-review: no production change and no new proof claim. These two candidates
  are deliberately retained for the stated interface/semantic boundaries,
  not marked as implemented optimizations or passing missing proofs.
