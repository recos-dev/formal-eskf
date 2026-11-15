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

## E-BIAS-UPDATE

Nine exact-real theorems define the six INS permissions and constrained gain.
They establish enabled/disabled rows, all-enabled equivalence, zero disabled-axis
correction, retention of that row's prior variance in Joseph, unchanged disabled
bias estimates after injection, and the existing unit-quaternion/PSD/reset-error
invariants. PSD is exact algebra, not a floating-point invariant.

The actual row-constraint producer accepts arbitrary IEEE gain coefficients and
all six symbolic permissions together; it covers all 64 masks without creating
64 execution profiles. It also checks the actual all-enabled default and, for
finite residuals, zero numerical correction on disabled axes. Caller proofs
check the same constrained gain enters both delta and Joseph, every old prior
and correlated noise input is retained, all failures propagate, and both public
outputs remain atomic. Six INS sensor wrappers forward all six permissions.
The original omitted-argument profiles remain present.

| Evidence | Result |
|---|---|
| Lean | Nine theorems; complete project build with `--wfail` passes |
| New ESBMC profiles | 78/78: six actual row producers, 24 controlled correction transactions, 48 sensor-wrapper profiles |
| Existing-path regression | 58/58 default caller profiles, including all AHRS aliases and every INS size/model |
| Negative checks | Four mutations detected in both formats: 8/8 counterexamples |
| Native regression | Correction and four sensor test executables rebuilt and passed in both reset modes; ten CTest entries |
| Infrastructure | Offline ESBMC runner tests, 100 agent-review tests, formatting and ShellCheck pass |

Every new profile covers binary32 or binary64; correction covers measurement
sizes 1/2/3, and caller profiles cover all four independent state/covariance
aliases. All state, covariance, measurement and intermediate-result coefficients
remain symbolic. No successful-solve, finite-prior, unit-norm or PSD premise is
used to exclude failure paths. The default regressions execute all AHRS aliases
and the distinct-output INS defaults. The new symbolic-mask profiles additionally
cover the all-enabled INS case for every alias; the actual default constructor
is checked separately in each row producer.

The compositional dependencies remain the same-backend `E-CORRECT` gain and
arbitrary-gain delta/Joseph bodies, `F-SOLVE`, and the injection/reset frames in
`step-correction`. Their accepted baseline evidence is reused; it was not all
rerun in this group. Actual constraint arithmetic is not summarized. Magnetic
wrapper profiles explicitly select M0: M1 and its combination with bias masks
are the following E-MAG-TILT group, so this commit does not close that path.
Prediction noise, learning policy, sensor health, IEEE PSD/error bounds and
external backend execution remain separate requirements.

The 78-profile positive matrix took 2,100.23 seconds sequentially, with aggregate
cgroup peak 514,572,288 bytes and no OOM. The same 120 s / 4 GiB per-query limits,
150 s outer deadline and enforced 6 GiB group cap with no group swap applied.
Negative checks remove masking, select the wrong rows, disable a default
permission or omit a sensor's forwarded permissions. An initial mutation
command selected 3D velocity while mutating horizontal velocity; it correctly
passed that unaffected API. The corrected model/shape produced counterexamples
in both formats. The failed test setup remains recorded, not accepted evidence.

Self-review: changes extend the existing caller assertions rather than replace
them with success-only checks. Unrestricted calls still omit the optional
argument. The six independent flags are checked symbolically, and failures
cannot be bypassed by disabled learning. Native tests retain their independent
constrained-gain/Joseph oracle, including cross-covariance, correlated noise and
all 64 masks. No production source, API, default or numerical behavior changed.
No full AI audit was run. Local evidence and hashes are in
`output/proof-extension-20261115/bias-manifest.json`, `bias-accepted/`, the
`bias-review*-results.json` records, and the Lean/native/tool logs.

## E-MAG-TILT

Lean adds 14 exact-real theorems (12 public, two helpers): the body Down vector
is unit for a unit prior, its outer product is symmetric/idempotent, projected
corrections are parallel to Down, and right-multiplicative injection preserves
tilt. The normalization theorem covers any nonzero parallel-axis quaternion
candidate, including the Taylor candidate without equating its angle to Exp.
The existing Joseph PSD theorem applies to the same constrained gain.

ESBMC checks the actual M1 observation preparation, ordered projector product,
and projected attitude block. Separate caller profiles cover AHRS/INS, both
scalar formats, four independent state/covariance aliases and every symbolic
INS bias mask. They check first failure, original rotation/residual/Jacobian,
mask-before-projection, the same final gain for delta/Joseph, injection/reset
arguments and atomic publication. Arbitrary IEEE inputs and intermediate
results remain admitted, including failure cases; no finite/unit/PSD/success
assumption removes a path. Private producer scratch buffers are disjoint.

| Evidence | Result |
|---|---|
| Lean | 14 new theorems; full project `--wfail` build passes |
| ESBMC | 24/24 new profiles and 2/2 reused rotation producer profiles |
| Negative checks | Four mutations in both formats: 8/8 assertion counterexamples; unmutated caller witnesses pass in both formats |
| Native regression | Two rebuilt M1 magnetometer CTest entries pass, covering both reset modes; M0 tests were run in the preceding group |
| Infrastructure | Offline ESBMC inventory/argument checks and 100 agent-review regression tests pass; formatting and ShellCheck pass |

The same-backend rotation result is connected through its actual producer. Gain,
arbitrary-gain delta/Joseph, injection/reset and bias-row producer evidence is
reused from the existing E-CORRECT/F-SOLVE/E-STEP/E-BIAS-UPDATE boundaries; these
producers were not all rerun here. This does not prove external backends, target
execution, floating-point tilt/PSD error bounds, sensor validity or runtime
magnetic lifecycle behavior. No full AI audit or Lean-to-C++ refinement was run.

The accepted 26-profile matrix took 955.42 seconds sequentially, with a cgroup
peak of 623,935,488 bytes and no OOM. The 6 GiB group cap and zero group swap
remain enforced. Caller profiles explicitly allow 180 s internal / 210 s outer time;
other profiles retain 120 s internally. All use the existing 4 GiB internal memory
limit. Full INS caller expansion takes approximately 110 seconds, so the extra
time is headroom for that measured cost, not a domain restriction.

Self-review retained full covariance-coefficient assertions and strengthened
scratch/input disjointness. An attempted symbolic-coordinate optimization timed
out and was discarded. A locally reproduced ESBMC 8.4.0 conditional aggregate-
reference issue produced a false projector counterexample: replacing
`same_matrix(output, valid ? expected : before)` with Boolean selection between
the two comparisons passed the unchanged property/domain. An isolated checker
reproducer and 200,000 native IEEE selections are retained; this is not a
production defect or a vendor-confirmed diagnosis. Failed/interrupted attempts
are recorded separately and do not count as proof passes.

Production code, API, defaults and numerical behavior are unchanged. Local
commands, source/log hashes, producer dependencies and failed attempts are in
`output/proof-extension-20261115/magnetic-manifest.json`, `magnetic-reviewed/`,
mutation/witness records and native/Lean/tool logs.
