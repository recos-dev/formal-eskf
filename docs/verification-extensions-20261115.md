# Verification extensions — 2026-11-15

Execution and self-review record for the existing V-01 sequence in
[ROADMAP](../ROADMAP.md). Requirements remain owned by
[verification-requirements](spec/verification-requirements.md); source and proof
symbols remain owned by the [traceability map](../formal/agent-review/traceability-map.json).
This record does not change the functional backlog or certify a deployment.

## E-INNOV

The exact-real model defines the componentwise ratio and proves nonnegativity,
sign invariance, the squared-residual threshold equivalence, equality at the
boundary, all-axis acceptance and acceptance when every ratio equals one.
It does not equate a rounded IEEE decision with the exact-real decision.

The C++ harness calls the actual `check_innovation` with the same fixed-array
backend type used by the correction proofs. It substitutes no arithmetic or
classification operations. All input coefficients, sigma and old output fields
are symbolic, without finite, bounded or successful-input assumptions. Six
scalar/size combinations each cover five vector alias partitions. The oracle
classifies all axes and counts outliers independently of the production early
returns and Boolean aggregation. Assertions check first failure, ordered ratios,
finite/nonnegative successful diagnostics, unchanged separate inputs and atomic
publication even after a prior axis was rejected.

Value preservation includes signed zero and NaN classification; NaN payloads,
floating-point exception flags, other dimensions/backends and target execution
remain outside the ESBMC claim. Mathematical model correspondence is reviewed,
not a machine-checked Lean-to-C++ refinement. Gate sigma calibration and runtime
fusion are separate requirements.

Accepted validation:

| Evidence | Result |
|---|---|
| Lean | Six theorems; complete project build with `--wfail` passes |
| ESBMC | 30/30 profiles; binary32/64 × sizes 1/2/3 × five alias partitions |
| Negative checks | Four production mutations detected in both formats: 8/8 counterexamples |
| Native regression | Rebuilt innovation test; all six scalar/size cases pass |
| Infrastructure | Offline ESBMC inventory/argument checks and 100 agent-review regression tests pass; formatting and ShellCheck pass |

Mutations change `>` to `>=`, aggregate with `&&`, omit squared sigma from the
denominator, or publish a partial candidate before a later axis fails. Concrete
witnesses exercise equality, an outlier, a non-unit sigma and a later overflow;
these witnesses validate the harness, not replace its unrestricted positive runs.
Commands, mutant headers, witnesses and counterexamples are retained locally.

The accepted ESBMC matrix took 678.88 seconds in sequential execution. Maximum
single-process RSS was 1,099,076 KiB; the cgroup-accounted aggregate peak was
1,073,295,360 bytes (different memory-accounting measures). Each run used the
existing 120 s / 4 GiB internal limits, an outer 150 s deadline, and an enforced
6 GiB group cap with group swap disabled. No cgroup OOM occurred. CVC5 1.1.2 via
ESBMC 8.4.0 is selected for this suite: the initial Bitwuzla two-axis query
exhausted its internal limit. Early helper-name ambiguities and formatting
failures were corrected before rerunning the entire accepted matrix. Those
attempts are retained and are not accepted evidence.

Self-review: the Lean model follows the specified componentwise formula; the
C++ oracle retains operation order, finite-input precedence, first-axis failures,
signed zero, arbitrary old diagnostics and every legal vector alias. No
assumption excludes a failure path. The mutations produce actual assertion
counterexamples, not timeouts. Production code, API and behavior are unchanged.
This closes the scoped E-INNOV source claim, not runtime fusion, gate calibration,
external backends or whole-estimator verification. No full AI audit was run.

Local evidence: `output/proof-extension-20261115/innovation-manifest.json`, the
`innovation-accepted/` logs/results, Lean/native/tool logs and mutation records.
The manifest records exact source/configuration identities and log hashes.
