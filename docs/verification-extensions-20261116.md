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

## Complete 10 ms adaptive accelerometer sampling owner

Five new profiles close the actual interval-clear equations and public owner:
binary32/64 clear and update, plus the double-precision rounding producer shared
by both formats. Twelve existing initialization/integration, magnitude and
same-backend vector-operation profiles were refreshed. All 17 pass in 33.68 s;
the capped scope peaks at 401,453,056 bytes (383 MiB), with no swap. The inventory
is now 2,148 profiles / 239 entry points.

The owner checks every result/history field, validation priority, dt and interval
denominators, strict readiness, unknown-clipping propagation, magnitude/clear
order and complete failure atomicity. Its inductive invariant includes finite
vector history, average period in [1, 50000] us, bounded thresholds and counters
that cannot wrap. The deliberately broad power-of-two bounds include all actual
reachable histories; they are proof bounds, not new public input restrictions.
Actual clear verifies the round argument/domain, cast, minimum sample count,
target/minimum equations and all cleared/preserved fields. The rounding producer
checks nearest-integer behavior including positive half ties in the verifier's
library model. Other template intervals and deployed libm remain outside scope.

The owner uses independently proved same-type integration, magnitude, clear and
vector-division contracts. Division replies admit every IEEE value, so overflow
and non-finite rejection remain covered. This removes duplicated division bit
circuits from the owner without assuming their outputs finite or successful.
Earlier monolithic timeouts/memory failures are retained as failed experiments;
only final complete profiles are accepted. No production change was needed.

Native witnesses pass four original controls and reject 48 intentional changes
at their intended assertions. These exercise input domains, strict timing edges,
unknown clipping, division units, output selection, failure stops, average update
and interval clearing/preservation. The witnesses use the final tracked harnesses.
Self-review checked every assumption against its same-type producer, the initial
base case, all inductive fields and complete result publication. Formatting,
ShellCheck, inventory tests, all traceability references and 100 offline review
tests pass. Protected production/draft hashes match baseline. No new Lean theorem
or AI semantic audit is claimed.

Evidence: `sample-final/results.json`, `sample-linalg-refresh/results.json`,
`sample-owner-native-results.json`, `sample-clear-native-results.json`,
`sample-inventory.log`, `sample-format.log`, `sample-offline-review.log` and
`sample-manifest.json` under the execution directory above.

## Vertical-health source equations

The actual `falling_likelihood` body now has separate protocol, classification
and IEEE-ratio profiles for binary32/binary64. The protocol proof admits arbitrary
observations, old output and independent IEEE root returns, with no input-domain
assumptions. It establishes source order, scalar arguments, the FLT_EPSILON floor,
first-failure priority, complete validation and atomic failure. Classification and
ratio consumers reuse only those proved prefix invariants. Neither quotient
identity nor the desired verdict is assumed. Both numeric consumers and the
protocol producer retain default safety checks and complete unwinding.

The oracle records six independent source slots, then combines them into pressure,
GNSS and ground references. It checks signed division, strict >1/>3 thresholds,
same-reference exclusion and high/medium/low publication. Scalar equality includes
signed zero and NaN classification, without a NaN-payload guarantee. The passive
observer returns the original scalar results and writes no production state.
Existing initialization/frame and inductive clipping/timestamp/hold proofs are
refreshed against this observer. Accurate sqrt, physical falling detection and
module/replay sequencing remain separate claims.

Monolithic ratio attempts exhausted solver resources, including an 8 GiB-capped
experiment; smaller helper/prefix attempts also failed. These are retained as
failed experiments, not accepted coverage. The accepted decomposition uses Z3's
non-incremental multi-property mode for ratios, with a longer time budget and the
normal 4 GiB solver allocation. No bounds, pointer, alignment or unwinding check
is disabled. The final source has no experimental source selector, prefix cut,
cached-quotient variant or production rewrite. Earlier observer traversal traces
are not used as evidence of a production bug.

Native checks use all assertions together, with no protocol assumptions: two
original controls pass and 32 intentionally changed executions fail at the intended
assertions. Four additional ESBMC negative witnesses reject multiplication in
place of division and an incorrect GNSS reference, in both formats. These negative
witnesses establish that the targeted assertions detect those errors; they are
not additional passing proof profiles.

Six new profiles and six affected existing profiles all pass in 1314.15 s.
The 6 GiB-capped, swap-disabled scope peaks at 1,523,425,280 bytes
(1.42 GiB). The inventory is 2,154 profiles / 240 entry
points. Self-review checked the non-circular protocol dependency, all six source
slots, complete failure paths and the observed IEEE arithmetic. Formatting,
ShellCheck, inventory checks, all 1,203 traceability references and 100 offline
review-tool tests pass. Production and pre-existing draft hashes match baseline.
No new Lean theorem or AI semantic audit is claimed.

Evidence: `vertical-final/results.json`, `vertical-native-results.json`,
`vertical-negative-results.json`, `vertical-inventory.log`, `vertical-format.log`,
`vertical-shellcheck.log`, `vertical-offline-review.log` and
`vertical-manifest.json` under the execution directory above. Exploratory logs,
including `round35-results.json` and `round39-results.json`, retain the rejected
resource-limited runs separately.

## Final cross-group review and totals

Cross-group self-review found 12 unbraced control-flow bodies in the newly added
magnetic-fusion harness. They now follow STYLE.md. All tokens other than braces
and whitespace are unchanged; review confirmed that the braces only surround
existing single bodies. Its 16 profiles were rerun successfully in 406.62 s,
with peak 434,642,944 bytes and no swap. Sixteen native controls pass
and 40 intentional changes are again rejected. This adds no obligation or input
restriction and changes no production code. These refreshed results supersede
only that harness's earlier execution identity. Other proof translation units
and all production dependencies remain unchanged.

| Completed group | New profiles | Previously existing profiles refreshed |
|---|---:|---:|
| Magnetic field/correction backend contracts | 32 | 6 |
| Full AHRS/INS magnetic fusion | 16 | 0 |
| INS magnetic heading reset | 16 | 12 |
| Adaptive 10 ms accelerometer sampling owner/timing | 5 | 12 |
| Vertical-health source equations | 6 | 6 |
| **Total distinct profiles executed** | **75** | **36** |

The accepted execution set contains **111 distinct profiles**. The 16 final
magnetic-fusion rechecks bring the number of accepted executions to **127**;
they are not new coverage. Final native witness sets contain 204 mutation checks
and 76 controls, plus four ESBMC negative witnesses. The style refresh repeats
40 of those mutations and 16 controls. Counts describe configured checks, not
requirements, theorems, functions or a coverage percentage. Across the accepted
executions, peak memory is 1,523,425,280 bytes (1.42 GiB).
Failed exploratory runs are excluded from these totals and retained separately.

The four previously listed portable-runtime obligations are now closed within
their recorded scalar, state-size, interval and producer-contract boundaries.
This does not close deployment `HeightAiding` transactions, full PX4/replay
sequencing, target libm/backend accuracy, numerical error budgets, physical
sensor validity, compiler/machine-code correctness or real-time qualification.
No new Lean theorem or machine-checked Lean-to-C++ refinement is claimed.

Final evidence: `fusion-style-final/results.json`,
`fusion-style-native/fusion-native-results.json`, `fusion-style-manifest.json`
and `completion-final-summary.json` in the execution directory. The original
manifests remain historical records rather than being silently rewritten.
