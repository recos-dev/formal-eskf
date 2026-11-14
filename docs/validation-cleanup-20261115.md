# Defensive-check cleanup acceptance

Baseline: `9efbf7763f5477d9653f1e333011470b7a0c5480`. The fixed implementation
sequence is DC-01–DC-08 in [ROADMAP.md](../ROADMAP.md). Numbers below preserve
the review list; they are work items, not proof requirements or measured speedups.

| Item | Audit origin | Candidate | Disposition |
|---|---|---|---|
| 1 | G01 | Reuse finite input premises in symmetry checks | Implemented in DC-02 |
| 2 | G01-C02 | Pairwise symmetry residual | Implemented in DC-02 |
| 3 | G01-C01 | Forward gain solve output | Pending |
| 4 | G01-C03, G06-C8, G08-C7 | Same-call PHt/S reuse after gating | Pending |
| 5 | G01-C04 | Hoist bias permissions out of column loops | Implemented in DC-01 |
| 6 | G02-C1 | Reuse validated prior quaternion | Pending |
| 7 | G02-C2 | AHRS nominal output forwarding | Pending |
| 8 | G02-021.1 | Acceleration finite check subsumption | Pending |
| 9 | G03-C1 | Diagonal-only process-noise result scan | Implemented in DC-01 |
| 10 | G02-C3, G03-C2, G04-C1–C3 | Shared prediction validation and mathematics | Pending |
| 11 | G04-C4 | Scan only variable F_x blocks | Implemented in DC-01 |
| 12 | G04-C5 | Reuse -R | Implemented in DC-01 |
| 13 | G04-C6 | Forward covariance prediction output | Pending |
| 14 | G05-C1–C2 | Reuse injection/reset input premises | Pending |
| 15 | G05-C3 | Share Exp/Jr angle intermediates | Pending |
| 16 | G05-C6 | Forward covariance reset output | Pending |
| 17 | G05-C8 | Avoid copied-and-overwritten INS candidate | Pending |
| 18 | G06-C1 | Reuse runtime innovation premises | Pending |
| 19 | G06-C2 | Simplify innovation range comparisons | Pending |
| 20 | G07-C1, C5 | Construction-valid H and delayed construction | Implemented in DC-01 |
| 21 | G07-C2 | Accel h/z finite check subsumption | Pending |
| 22 | G07-C3, G08-C4 | Finite vector nonzero checks | Implemented in DC-01 |
| 23 | G08-C1 | Build runtime magnetic H from existing h | Implemented in DC-01 |
| 24 | G08-C2–C3 | Share magnetic model preparation across gate/core | Pending |
| 25 | G08-C6 | Share magnetic prediction/projection rotation | Pending |
| 26 | G08-C5 | Magnetic h/u finite check subsumption | Pending |
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
  Formatting and all mapped C++/ESBMC symbol references pass. No requirement
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
