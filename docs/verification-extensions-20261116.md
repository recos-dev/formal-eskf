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
