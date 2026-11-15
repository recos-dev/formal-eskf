# Runtime proof completion — 2026-11-16

Baseline: `d3f01a6`. Scope is the portable runtime and its required numerical
producers. Production core/runtime, PX4 integration and replay are unchanged.
The previous [execution record](verification-extensions-20261115.md) retains its
historical results and open boundaries; the groups below supersede only their
explicitly named obligations. No target-libm accuracy, external-backend internals,
physical sensor validity, scheduling guarantee or Lean-to-C++ refinement is implied.

## Magnetic field/correction backend boundary

The field and observation helpers now have evidence on the exact fixed-array
backend used by gate/correction. The proof backend distinguishes the passive
positive-pivot observer from general scalar calls: field geometry legitimately
uses a zero horizontal squared magnitude. Both modes retain the same arbitrary
root sequence. General atan2 calls admit every IEEE return instead of assuming
accurate angles or a principal range. Existing Cholesky profiles keep pivot
eligibility checks enabled.

| Evidence | Accepted checks | Scope |
|---|---:|---|
| New ESBMC profiles | 32 | binary32/64 scalar argument/status/atomicity contracts, root-observer equivalence, actual geometry/heading frames, rotations, field qualification and observation preparation |
| Existing factor profiles rerun | 6 | both formats, sizes 1/2/3, actual ordered Cholesky coefficients and positive-pivot eligibility |
| Native mutation checks | 32 mutants + 22 controls | scalar domain/arguments/atomicity and observation routing, stop priority, residual sign, finite checks and publication |
| Native deployment-backend tests | 2 | rebuilt Eigen and PX4-matrix magnetic fusion tests |

The field caller checks all four health-reference cases, both failure/control and
successful-computation diagnostics. No public numerical input, selection flag or
failure status is assumed valid. The general geometry producer guarantees finite
successful fields and atomic failure; the heading producer supplies its input/
output frame. Existing exact geometry/wrap equations remain separately scoped.
General observer mode is not a weakened SPD requirement: F-SOLVE still proves
positive-pivot eligibility, and the equivalence profile quantifies every valid
root index and every IEEE result. There are no production changes.

The new profiles pass in 204.17 s; factor refresh takes 17.69 s. Their enclosing
scope peaks at 891,613,184 bytes (850 MiB), with a 6 GiB cap and swap disabled.
The current inventory is 2,111 profiles / 234 entry points; only the listed
profiles were rerun. No new Lean theorem or AI semantic audit is claimed.
The complete public AHRS/INS magnetic transaction remains a separate obligation.

Self-review checked same-type dependencies, observer/default behavior, the
zero-root case, unrestricted non-finite returns, scalar argument order, reference
alias partitions and preservation of all input/output fields. Native mutants
fail on their intended assertions; unmodified controls pass. The runner inventory,
100 offline agent-review regressions, ShellCheck and formatting pass. Source and
log hashes are retained locally; protected production/draft hashes match baseline.

Evidence directory: `output/runtime-proof-completion-20261116/`:
`bridge-final/results.json`, `bridge-factor-regression/results.json`,
`scalar-native-results.json`, `observation-native-mutations.json`,
`native-magnetic-fusion.log`, `offline-review-tools.log` and `bridge-manifest.json`.

## Complete portable magnetic fusion

The actual public AHRS and INS overloads now have 16 accepted profiles:
binary32/64, state sizes 3/15 and all four independent whole-prior output aliases.
Each profile includes null/separate/body/reference health-pointer arrangements.
The proof observes field/observation, gate and correction calls, checks the actual
attitude block and finite guard, and checks every diagnostic and published field.
All six INS bias permissions must reach correction. Child replies overapproximate
all outcomes; successful inputs/results are not assumed. Same-type producer
contracts from the preceding boundary closure and existing measurement/correction
families support composition. The public body is independent of `ESKF_MAG_TILT`;
the correction summaries are valid for both separately proved M0/M1 families.

Native mutation checks accept 16 original controls and reject 40 modified cases
at their intended assertions: wrong health/raw references, stage bypasses, wrong
Jacobian sign/offset, missing finite checks, false fused status and ignored bias
permissions. Each binary exercises 11 outcome scenarios and all four health
selections. Compilation failure is not counted as a caught mutation.

Formal execution totals 433.85 s, with scope peak 433,803,264 bytes and no swap.
The inventory is now 2,127 profiles / 235 entry points. Self-review verified
unconditional failure coverage, four alias partitions, complete input/output
frames and backend identity. Formatting, ShellCheck, inventory validation and
100 offline agent-review tests pass. Production and unrelated draft hashes match
the baseline. No new Lean theorem, target-libm accuracy or full AI audit is claimed.
PX4/replay sequencing remains outside this portable transaction.

Evidence: `fusion-final/results.json`, `fusion-native-results.json`,
`fusion-inventory.log`, `fusion-format.log`, `fusion-offline-review.log` and
`fusion-manifest.json` under the execution directory above. Earlier CVC5/Z3
resource failures are retained in the exploratory logs; only the final complete
Bitwuzla profiles are accepted. No public domain restriction was added to make
the solver finish.

## Complete portable INS magnetic heading reset

The INS covariance producer now composes the actual reset body with independently
proved same-type 3x3 and 3x15 products. It checks every operand and call order,
the yaw floor/Down axis, ordered remaining arithmetic, finite/diagonal failure
priority, divide-before-add symmetrization, mirrored cross blocks and all 225
output coefficients. Product replies may contain any IEEE values. No successful,
finite, unit, symmetric or PSD input/result premise is imposed on this producer.
The full helper and public transaction now cover INS as well as AHRS, including
finite non-attitude validation, unchanged position/velocity/bias, all child
failures, partial scratch writes and all four state/P aliases.

There are 16 new accepted profiles (four INS covariance, four INS helper and eight
INS public transaction), plus 12 affected AHRS profiles rerun. The inventory is
2,143 profiles / 236 entry points. The matrix producers and exact-real reset
algebra retain their independently recorded evidence; this adds no Lean theorem
or floating-point PSD/error-bound claim.

Native checks pass 32 controls and reject 52 modified cases at intended
assertions. These include wrong tilt/frame/cross blocks, yaw variance,
finite/negative guards, reordered floating-point halves, incorrect wrapped
heading/composition, bypassed failures and lost non-attitude INS values.
Compiler errors and equivalent mutations are not accepted failure witnesses.

Accepted execution totals 740.93 s; scope peak is 1,113,202,688 bytes, with no swap.
The original conditional-reference query timed out during symbolic expansion;
explicit pointer selection preserves the same four aliases and makes the full
query tractable. No production change or domain restriction was required.
Self-review checked each summary against same-type producers, complete state/P/
delta publication, finite-validation priority and the distinct scratch contract.
Formatting, ShellCheck, inventory validation and 100 offline review-tool tests
pass. Rebuilt Eigen/PX4-matrix reset, vertical-health and sampler tests pass (six).
Production and unrelated draft hashes remain identical to the baseline.

Evidence: `reset-final/results.json`, `reset-ins-native-results.json`,
`reset-flow-native-mutations.json`, `runtime-native-build.log`,
`runtime-native-tests.log`, `reset-inventory.log`, `reset-format.log`,
`reset-offline-review.log` and `reset-manifest.json`. The exploratory failed
monolithic/solver queries remain separate from these accepted profiles.
