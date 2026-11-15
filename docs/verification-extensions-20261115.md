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

## R-MEASUREMENT-FUSION

This group verifies the portable check/fuse APIs and four navigation wrappers.
It does not close the requirement's replay GNSS parameter, variance-setting or
module-wiring clauses. Caller sequencing, observation independence and timing
remain caller obligations; the existing native sequential-fusion case supports
those uses but is not a formal caller-loop proof.

The new public covariance-preparation frame checks first failure and two-output
publication under the existing E-CORRECT validation/product/finalizer producer
contracts. Generic check/fuse proofs execute actual nominal-state/sigma checks,
diagonal extraction, diagnostic/result construction, gate/correction routing
and output forwarding. They require the identical prior, residual, H, full V,
normalization threshold and six INS permissions at correction. Gate rejection
must skip correction; later correction failure retains valid gate diagnostics.
All paths preserve separate inputs, and only fused may change both outputs.
Check-only has no FusionDecision; the harness also checks its `passed()` and
rejection fields before using the result constructor as a common oracle.

The four named wrappers execute their actual finite-input checks, residuals and
selector Jacobians, then forward every decision/status/diagnostic unchanged.
Their profiles select M0 to reuse the observation harness support; none of these
four APIs depends on the magnetic-projection macro. This does not claim coverage
of runtime magnetic fusion.
No finite, unit, PSD/SPD, successful-input or successful-intermediate assumption
restricts the positive input domain. Arbitrary failed scratch results are
admitted to check that callers do not consume them. All four legal whole-state/
covariance alias arrangements are explicit; every covariance coefficient is
checked. Preservation distinguishes signed zero and NaN classification; NaN
payloads and floating-point exception flags are not proved. The six permissions
remain symbolic together, without a 64-fold
execution-profile expansion.

| Evidence | Result |
|---|---|
| ESBMC preparation | 12/12: binary32/64 × state 3/15 × observation 1/2/3 |
| ESBMC check/fuse | 48/48: the same shapes × four output aliases; check-only is symbolic |
| ESBMC named wrappers | 32/32: two formats × four navigation models × four aliases |
| Negative checks | Four mutations × both formats: 8/8 assertion counterexamples |
| Native regression | Rebuilt measurement-fusion CTest entry passes |
| Infrastructure | Offline ESBMC inventory/argument checks and 100 agent-review regression tests pass; formatting and ShellCheck pass |

There is no new mathematical formula in this group. The existing Lean gate and
correction theorems are linked as dependencies, not counted again or presented
as a Lean proof of C++ control flow. Same-backend E-INNOV, E-CORRECT/F-SOLVE,
E-BIAS-UPDATE and injection/reset producer evidence is reused, not all rerun.
Existing correction helper bodies and profiles are unchanged: the addition to
`correction.cpp` is a separate entry for its public preparation frame.

Mutation checks substitute diag(V) for diag(S), bypass rejection, erase valid
diagnostics after correction fails, or substitute the wrong navigation
Jacobian. They validate sensitivity of the compositional harness assertions;
timeouts do not count as detection. The initial partial batch was stopped to
finish explicit control-flow braces in the new harnesses; the reviewed source matrix was then executed. Its first 30 preparation/generic
profiles passed. The first navigation profile exposed the same conditional
aggregate-reference checker issue recorded above: an equivalent compile-time
branch in the oracle passed, with a native control agreeing with the original
expression. Only navigation_fusion.cpp changed; the first 30 results remain
applicable because their roots, headers and flags did not change or include that
file. All 32 navigation profiles and the remaining generic/preparation profiles
were then run on the corrected source. The manifest keeps both execution
records and the exact retained-prefix dependency check. Failed/interrupted
attempts do not count as passes.

Self-review found no production behavior change. Failure precedence, aliases,
full correlated V, bias permissions and fresh diagnostics are covered rather
than assuming successful correction. External backend/target execution,
floating-point numerical bounds and runtime source health remain outside this
scope. No full AI audit or machine-checked Lean-to-C++ refinement was run.
Local commands, source/log identities, timings and retained attempts are in
`output/proof-extension-20261115/fusion-manifest.json`, `fusion-reviewed/`,
mutation records and native/tool logs.

The accepted 92 profiles took 1564.72 seconds in total. The two accepted
execution scopes recorded an aggregate peak of 480,735,232 bytes, with no OOM.
All queries retained 120 s / 4 GiB internal limits, a 150 s outer deadline,
and the enforced 6 GiB group cap with group swap disabled.

## Portable IMU and bias policies

This closes the standalone component scope of R-IMU-PROCESS-NOISE,
R-BIAS-PROCESS-NOISE and R-BIAS-LIMITS. Their replay/module metadata, scheduling,
startup and post-fusion invocation clauses remain open. The vertical detector,
learning-permission policy and physical calibration are not part of this group.
The C++ claim uses the fixed-array proof backend; external backends and deployed
toolchain/standard-library execution remain separate.

The actual C++ functions execute without numerical summaries or input-domain
assumptions. Fault variance and all twelve configured coefficients are validated
before replacement, including unselected values. The bias-noise selector checks
its eighteen consumed coefficients/diagonals, inhibits at equality, preserves
measurement variances and leaves P unchanged. All other P coefficients remain
arbitrary. Both noise selectors cover separate and whole-noise aliased outputs,
with complete rollback for any failure. The limiter checks both bounds and all
six biases before writing; success clamps only those six fields and preserves
in-range IEEE representations. Unused nominal fields remain arbitrary, including
nonfinite values. Signed zeros and NaN classification are distinguished by the
existing oracle; NaN payloads and floating-point exception flags are not proved.

Six exact-real Lean theorems establish nonnegative selected noise, equality
inhibition, identity bias transition rows even with cross-covariance, and PSD
preservation when these selected noises feed the existing prediction model.
The premises remain PSD prior, nonnegative configured/fault variances and dt.
These results do not establish IEEE PSD preservation, a hard covariance bound,
noise calibration or covariance consistency after clamping. A permitted noise
increment can still cross the threshold; inhibition is a pre-increment decision.
The clamp's comparison/selection properties are proved directly in ESBMC and
are not inflated into additional Lean theorem counts.

| Evidence | Result |
|---|---|
| Lean | Six new theorems; complete project build with `--wfail` passes |
| ESBMC IMU noise | 4/4 profiles: binary32/64 × both output aliases |
| ESBMC bias noise | 4/4 profiles: binary32/64 × both output aliases |
| ESBMC bias limits | 2/2 profiles: binary32/64, in-place state update |
| Negative checks | Three mutations × both formats: 6/6 assertion counterexamples |
| Native regression | Three rebuilt noise/limit CTest entries pass |
| Infrastructure | Offline ESBMC inventory/argument checks and 100 agent-review regression tests pass; formatting and ShellCheck pass |

Mutations replace the specified noise replacement with max, change the bias
threshold from inclusive to strict, or omit gyroscope clamping. Counterexamples
must violate the relevant assertions; timeouts and tool errors are not accepted.
Initial Lean elaboration/simplifier failures are retained separately. Explicit
finite-index cases and block dimensions remove those failures without adding
axioms, admitting goals or changing theorem statements. A native syntax command
initially included the ESBMC-only numbers shim and failed; rerunning with native
standard headers passed. Neither the shim nor production code was changed.

Self-review confirms correspondence between the two selectors and the Lean
noise definitions, including selection before bias-noise inhibition. The spec's
ordering sentence is clarified to match its existing required order and current
production call sites; the call sites themselves were not changed or proved
here. No production code, API, parameters or defaults changed. No full AI audit
or machine-checked Lean-to-C++ refinement was run. Local commands, source/log
identities, resource results and attempts are retained in
`output/proof-extension-20261115/imu-manifest.json`, `imu-reviewed/`, mutation
records and Lean/native/tool logs.

The ten-profile matrix took 21.79 seconds sequentially, with aggregate
cgroup peak 166,232,064 bytes and no OOM. It retained 120 s / 4 GiB internal
limits, a 150 s outer deadline and the enforced 6 GiB group cap without swap.

## Initial five-group totals

| Group | New Lean theorems | New ESBMC profiles | Detected mutations |
|---|---:|---:|---:|
| E-INNOV | 6 | 30 | 8 |
| E-BIAS-UPDATE | 9 | 78 | 8 |
| E-MAG-TILT | 14 | 24 | 8 |
| Portable R-MEASUREMENT-FUSION | 0 (existing math reused) | 92 | 8 |
| Three portable IMU/bias policies | 6 | 10 | 6 |
| Total | 35 | 234 | 38 |

The theorem count includes two private helpers. The profile inventory grows from
1,554 to 1,788; source entry points grow from 127 to 139. All 234 new profiles
passed. The sequence also reran 58 default correction/observation profiles and
two rotation-producer profiles; these 60 are not counted as new coverage.
Concrete mutation-control witnesses, interrupted/failed attempts and repeated
runs do not inflate the new-profile count. Offline agent-review tests are tool
regressions, not a full AI semantic audit.

These are different evidence units, not a percentage of whole-project proof
completion. Existing producer contracts and exact-real premises remain part of
the claims. Runtime lifecycle/caller sequencing, replay/PX4 module integration,
numerical error bounds and deployed backend/toolchain behavior remain separate.
The production implementation and the functional backlog were not changed.

## Continuation scope

The first five commits above complete only the initial stable components, not
all currently implemented proof obligations. Continue the existing V-01 dependency
order through the remaining IMU/health policies, GNSS quality/lifecycle and reset,
height management, gravity sampling/eligibility/fusion, magnetic policy/lifecycle
and reset, then the actual module/replay caller boundaries. Reuse existing
mathematical dependencies; Boolean policies do not require artificial new Lean
algebra. Do not change the functional F-series order or production behavior.

A portable component pass does not close its requirement's caller clauses.
For persistent components, distinguish bounded sequences from initialization and
inductive preservation across public transitions. Retain failed attempts and
mutations alongside accepted source/configuration identities. Continue independent
work if one obligation is blocked; a blocker needs a concrete counterexample,
frontend limitation, measured resource limit or missing design/platform premise.
Future features, physical sensor validity, unselected deployment numerical bounds,
external-backend internals and OS/WCET guarantees must not be silently labeled
complete or replaced with component proofs.

## Vertical health state safety

`R-VERTICAL-ACCELERATION` now has a scoped state-safety proof; the complete
requirement is **not closed**. Six binary32/binary64 profiles check the actual
constructor/reset, the actual private likelihood producer's enum/failure frame,
and the public update composed with that frame. They establish ordered time,
interval and metadata validation, safe count-limit conversion, every clipping
counter, verdict consumption, last-good/last-bad timestamps, strict three-second
hold, input purity and atomic history on all failures.

The clock/hold invariant is checked at initialization and preserved from an
arbitrary prior satisfying it, including every failed update. Reset establishes
the base case from arbitrary history. This is inductive state safety, not a
short trace presented as arbitrary-duration evidence. The header's private access
is widened only in the harness after loading dependencies, to call its actual
producer; production bodies and layout are unchanged. The caller summary admits
all statuses and all three verdicts, even candidate writes on failure. Its actual
producer frame is proved separately with arbitrary scalar-root returns. No claim
of source classification or accurate sqrt is hidden in that summary.

Accepted checks: 6/6 profiles in 75.89 s, cgroup peak 1,022,902,272 bytes, no OOM
or swap; six assertion counterexamples and six passing controls across both
formats. Mutations change the hold boundary, increment at the clipping limit,
or publish a timestamp on producer failure. The native vertical-health test,
runner inventory, 100 offline agent-review tests, formatting and ShellCheck pass.
The offline tests are not an AI semantic audit. No Lean files changed; no new
mathematical theorem is needed for these source transition properties.

The complete six-source numerical oracle remains unaccepted. Retained attempts
include full and separated forms, two solvers, independent source classification
and scalar storage variants. Queries timed out or reached the 4 GiB internal
limit; incremental CVC5 also crashed on a caller query. These are tool/analysis
limitations, not passes or production counterexamples. The accepted caller uses
non-incremental CVC5. Source-ratio equations, the FLT_EPSILON floor and distinct
reference classification remain open, alongside module/replay wiring and physical
validity. Retiring a failed query from the accepted suite does not retire those
requirements; its harness snapshots, commands and logs are retained locally.

Self-review: verified that no sample-domain assumption hides invalid input or a
failure; prior-state assumptions are initialized and preserved. Changed a mutation
from candidate publication (potentially unobservable with the real callee's stronger
failure frame) to an actual timestamp write before accepting its counterexamples.
Production sources and existing functional/roadmap drafts remain unchanged.
Evidence: `output/runtime-proof-continuation-20261115/vertical-manifest.json`,
`vertical-final/results.json`, `vertical-mutations.json` and the retained failed
attempts. Other independent V-01 obligations continue.

## Shared IMU bias inhibition

`R-REPLAY-BIAS` and `R-PX4-IMU-BIAS` now include the portable policy, not the
actual replay/module sequencing. Fourteen binary32/binary64 profiles check the
actual norm and rotation producers, exact decay/max, finite nonnegative peak
invariant, permission predicate and composed public update. Constructor peaks
establish the base case; arbitrary valid prior peaks and every public update
establish induction. Invalid samples are not assumed away. Caller summaries
admit arbitrary root/rotation results and use only the separately proved peak
postcondition, with assertions for each call's domain, order, sensor period and
prior value. All borrowed inputs remain unchanged; failures preserve history
and return disabled permissions. Equality permits learning; rest overrides only
observability, not clipping, dynamics or vertical faults.

Accepted checks: 14/14 profiles, 316.71 s sequentially, cgroup peak 511,578,112
bytes (488 MiB), no OOM or swap. Binary64's peak invariant took 224.87 s; the
formula and invariant are separate required profiles, not weaker replacements.
Peak profiles use a 300 s internal limit; others retain 120 s. All use 4 GiB
internal and 6 GiB group memory limits without swap. Six mutations across both
formats yielded 12 assertion counterexamples and 12 passing controls: swapped
integration periods, gyro dynamics coupling, wrong clipping axis, equality
rejection, history clearing on failure and growing rather than decaying peaks.
Native policy tests and 200 native `std::max` control pairs passed, alongside
runner inventory, 100 offline agent-review tests, formatting and ShellCheck.
No full AI semantic audit or new Lean theorem is claimed.

A minimized counterexample exposed ESBMC 8.4.0's embedded `<algorithm>` model:
its `max` selects the second equal argument and includes nonstandard by-value
overloads. Native `std::max(-0,+0)` retains the first reference and negative
zero; the embedded model does not. `algorithm_support.hpp` models the standard
two-argument reference contract and alpha-renames only the selected header's
calls, after dependency includes. Its symbolic checks include aliasing, both
zero signs, NaN classification and input purity. This is an explicit STL
contract boundary, not proof of the tool's original model or a deployed STL.
The production implementation was not changed. Original counterexamples and
unsuccessful solver attempts remain in local evidence; they are not core bugs
or accepted passes. Existing numerical core proofs do not consume this max.

Self-review: checked all assumptions against producer domains and initialized/
preserved invariants; retained root, nonfinite and invalid-input failure paths.
A slower binary64 query was decomposed by property without dropping either
property. Both scalar formats, mutation controls, unchanged production hashes
and exact staged scope were checked before commit. Physical observability,
accurate libm, backend internals, replay metadata and actual module forwarding
remain separate. Evidence: `output/runtime-proof-continuation-20261115/imu-manifest.json`,
`imu-split-properties/results.json`, `imu-mutations.json`, native controls and
the retained embedded-library counterexamples.

## Module IMU prediction helper

Eight binary32/binary64 profiles cover actual `ImuPrediction.hpp`: clipping
admission, both actual same-type noise producers and the composed in-place
prediction caller. They check fixed fault variance/bit decoding, a fresh copy
of every configured parameter, selection before bias inhibition using prior P,
exact delegation/first error and atomic state/P return. The two policy summaries
admit arbitrary statuses and successful noise outputs; matching actual producers
are required. The terminal prediction uses existing E-STEP's same-type, in-place
failure-atomic and input-pure frame. There is no assumption of successful
prediction, finite state or PSD covariance in the wrapper proof.

Accepted checks: 8/8 profiles in 45.71 s, cgroup peak 440,815,616 bytes, no OOM or
swap; 120 s / 4 GiB internal and 6 GiB group limits. Eight assertion
counterexamples and eight passing controls cover feeding configured instead of
selected noise into bias inhibition, shifted clipping bits, wrong fault variance
and covariance publication on admission failure. Native module-prediction tests,
runner inventory, 100 offline agent-review tests, formatting and ShellCheck pass.
The inventory is 1,816 profiles and 152 source entry points; these are not whole-
project coverage percentages. No full AI semantic audit or new Lean theorem.

The initial monolithic query timed out with CVC5 and Z3. The compositional Z3
bias producer also timed out. The accepted matrix uses the existing policy
suite's default Bitwuzla solver. Initial covariance initialization required
increasing unwind from 20 to 230; unwinding failures were retained and never
counted as passes. The ESBMC `<numbers>` model adds only the standard double
`pi = pi_v<double>` alias needed to parse the actual deployment-profile header.
No module code or deployment constants were substituted or changed.

Self-review: matched all summaries to concrete producers and legal alias modes,
verified complete parameter/input preservation, and checked that mutation
failures are assertions rather than frontend/resource failures. Tightened two
legacy inventory regexes to match the profile-column start, avoiding accidental
counting of the new `imu-prediction` profiles as core prediction profiles.
This closes the helper's ordering/rollback obligation under the proof backend;
actual PX4 Matrix, the estimator loop, real/gap admission, health chronology,
fault latch and publication remain separate. Evidence: `output/runtime-proof-continuation-20261115/imu-prediction-manifest.json`,
`imu-prediction-bitwuzla/results.json`, `imu-prediction-mutations.json`, native
and tooling logs, plus failed attempts.

## GNSS quality policy and health

Fourteen binary32/binary64 profiles cover `R-GNSS-QUALITY`'s portable monitor:
specified defaults and ten named checks, all parameter/sample domains, actual
IEEE motion/filter equations, raw-speed restrictions, initial/recovery health
timers and the composed public update. Constructor/reset and health restart
establish/preserve ordered history. An arbitrary prior satisfying that invariant
is used for every public update, including invalid calls and consumed numerical
rejections; qualification remains latched until reset. No bounded sequence is
presented as evidence for arbitrary-duration operation.

Private motion profiles use the premises established by public validation: valid
parameters, valid consumed sample fields and a positive sample-time difference.
These ensure standard clamp bounds are ordered; they do not assume finite prior
filters or exclude arithmetic failures. Public invalid inputs remain unrestricted
and checked. The actual two-axis norm producer verifies ordered reduction, scalar
call/input purity and return forwarding. Its arbitrary root returns are an
explicit library boundary, not an accurate sqrt or physical drift guarantee.
Norm arguments/order are checked against the independently computed filtered
vectors. The motion frame permits only filter and motion-diagnostic writes,
including failure; the public caller admits arbitrary such outputs/statuses.
No target/backend error bound, adapter projection or actual replay/module
chronology is claimed.

Accepted final checks: 14/14 profiles in 106.12 s sequentially, cgroup peak
782,020,608 bytes (746 MiB), no OOM/swap. Eighteen assertion counterexamples and
eighteen passing controls cover mandatory fix removal, wrong check enablement,
inclusive initial-health timing, lost qualification, invalid-time consumption,
first-delta use, unclipped velocity filtering, cleared ground-motion flags and
inverted raw-speed comparison. Native GNSS-quality tests, runner inventory,
100 offline agent-review tests, formatting and ShellCheck pass. The inventory
now has 1,830 profiles and 159 entry points. No new Lean theorem or full AI
semantic audit is claimed.

Retained failures: the known ESBMC conditional aggregate-reference issue appeared
again in a norm-argument comparison. Boolean selection between the two complete
comparisons proves the same property without changing production or the input
domain. The first Bitwuzla public-transition query reached its internal memory
limit; the accepted transition profiles use non-incremental CVC5. Other profiles
use default Bitwuzla. Every profile retains 120 s / 4 GiB internal limits and a
6 GiB no-swap cgroup. Failed attempts are not passes.

Self-review added explicit default-parameter and restart-invariant assertions,
and froze the named-check oracle inputs before invoking the producer. Rechecked
all fourteen profiles and all mutation controls after those improvements.
The public transition establishes every private motion premise, preserves all
borrowed inputs and distinguishes invalid-call rollback from consumed rejection.
Production headers, module sources and unrelated drafts are unchanged.
Evidence: `output/runtime-proof-continuation-20261115/gnss-quality-manifest.json`,
`gnss-quality-final/results.json`, `gnss-quality-mutations.json`, native/tooling
logs and the retained unsuccessful attempts.

## GNSS lifecycle and feedback

Seven profiles/entry points check the actual portable `GnssFusion` controller:
construction/reset, update, fusion feedback, full reset feedback, Down-only reset
feedback, explicit restart and acceleration-health time classification. This
controller uses integer times and Boolean/enumeration state, so these profiles
are not artificially duplicated by floating-point format. No callee summaries.

The initialized, inductively preserved invariant orders progress/sample/quality
clocks and relates permissions to active/recovery state. Each public transition
is checked from an arbitrary invariant state with unrestricted public inputs.
It proves exact validation and failure frames, data/quality deadlines, coupled
horizontal progress, height/Down ordering, persistent repair requests and input
purity. Only actual fused feedback advances fusion time; reset progress is
separate. Accepted feedback consumes authority and the universal feedback guard
rejects missing authority. This establishes one-use behavior by induction rather
than treating a short repeated-call trace as arbitrary-duration evidence.

Accepted checks: 7/7 profiles in 217.72 s, cgroup peak 937,943,040 bytes, no OOM or
swap, non-incremental CVC5, 120 s / 4 GiB internal and 6 GiB group limits. Ten
assertion counterexamples and ten passing controls cover freshness/deadline
equality, reset/rejection masquerading as fusion, stale authority, height/velocity
ordering, failed Down reset progress, premature quality restart, invalid-clock
consumption and incomplete reset. Native GNSS-fusion tests, runner inventory,
100 offline agent-review tests, formatting and ShellCheck pass. Inventory:
1,837 profiles / 166 entry points. No new Lean theorem or full AI audit.

Retained attempts: Bitwuzla update reached its internal memory limit; CVC5 with
two expanded fusion calls timed out. The accepted proof uses the same universal
transition plus the consumed-authority postcondition, eliminating redundant
second-call expansion without weakening the one-use requirement or increasing
the time limit. Failed queries are not passes.

Self-review matched every public branch and all persistent fields, checked the
initialization/induction chain and invalid-call domains, and confirmed mutation
failures were target assertions. The invariant is an overapproximation of
reachable history, not an assumption that every abstract state was reached.
Actual external result honesty, quality/health validity, module/replay ordering
and numerical reset remain separate obligations. Production and unrelated drafts
are unchanged. Evidence: `output/runtime-proof-continuation-20261115/gnss-lifecycle-manifest.json`,
`gnss-lifecycle-single-transition/results.json`, `gnss-lifecycle-mutations.json`,
native/tooling logs and failed attempts.

## Navigation reset mathematics and C++

Sixteen Lean theorems cover `R-GNSS-RESET`: covariance reset decomposes into
`D P Dᵀ` plus a nonnegative replacement diagonal; all four operations preserve
PSD, with the specified retained/decoupled entries and variance policies.
Nominal attitude and biases are retained. PSD is not strengthened to SPD.
The exact-real decimal floor and scalar-format encoded C++ floor are distinct.

Forty ESBMC profiles (three source entry points) check both scalar formats:
finite zero-residual/equality arithmetic, actual input validation for sizes
1/2/3, and four reset callers with all four independent state/P alias modes.
The validation summary is matched to its actual same-type producer, checks the
complete original request exactly once and admits any returned status. Actual
numerical reset bodies, coordinate writes and covariance storage remain inline.
An arbitrary bounded row/column quantifies every output coefficient; no public
input is assumed valid or restricted to representative coordinates. Checks cover
exact statuses, overflow, new-minus-old delta, floor/decorrelation, rollback and
input purity. Comparisons preserve values, signed zeros and NaN classification,
not NaN payloads.

Accepted results: 40/40 profiles in 739.46 s; cgroup peak 1,566,097,408 bytes
(1.46 GiB), no OOM/swap, 120 s / 4 GiB per profile and 6 GiB group limits.
Full Lean build passes with `--wfail` (3,421 jobs). Twenty-four native mutation
failures and twenty-four positive controls exercise twelve fault patterns in
both formats: floor, variance summation, delta direction, negative diagonal,
asymmetry, both overflow paths, premature publication, missed column clearing,
wrong height coordinate, lost Down-position variance and retained position
correlations. Seven of these concrete mutations and eight positive controls
also completed under ESBMC. These counts are distinct from symbolic proof
profiles; interrupted mutation runs are not passes. Native navigation-reset
tests, 100 offline agent-review tests, inventory, formatting and ShellCheck pass.
Inventory: 1,877 profiles / 169 entry points. No full AI audit.

Retained failures: full expansion timed out; asking one query to establish
225 zero-residual/equality relations exhausted its internal solver budget.
The accepted split proves the scalar relation once and uses an independent
row-major residual oracle for the actual validator. A conditional aggregate-
reference binding then reproduced the already recorded ESBMC model issue;
explicit pointer selection preserved all alias cases and passed. The initial
all-entry assertion loop was replaced by universal symbolic indices, preserving
the complete coefficient obligation. No production change or weaker contract.
Concrete mutation fixtures initially repeated expensive symbolic expansion;
all fixed witnesses were then checked natively, with completed ESBMC controls
retained separately. The original and reduced fixtures and interrupted logs
remain in the evidence directory.

Self-review checked real/source coordinate masks, the full validation-to-caller
chain, IEEE error precedence, independent aliases and every mutation failure
message. Confirmed the live protected production/draft hashes and limited the
commit to proof, runner, traceability and execution evidence descriptions.
External backends, real-data quality, caller permission/acknowledgment and
publication remain separate; this is not a machine-checked Lean-to-C++ refinement.
Evidence: `output/runtime-proof-continuation-20261115/navigation-reset-manifest.json`,
`navigation-reset-final/results.json`, `navigation-reset-native-mutations.json`,
`navigation-reset-mutations.json`, `navigation-reset-lean.log`, native/tooling
logs and retained unsuccessful attempts.

## Vertical innovation recovery

Seven Lean theorems cover real clipping bounds, unchanged in-bound values,
sign, idempotence, axis isolation and the resulting innovation ratio. They do
not justify re-gating a rounded boundary or establish sensor health.

Twenty-eight ESBMC profiles (five entry points) cover binary32/64: shared scalar
bound/sign lemmas, actual gate diagnostic production, actual vertical limiting,
and both height/GNSS velocity callers with all four independent state/P aliases.
The producer establishes positive finite variance and valid raw diagnostics.
The limiter checks the exact root argument and arbitrary root return, its error
paths, selected axis and unchanged raw diagnostics. Composed callers reuse the
separately checked exact limiter relation and same-type covariance/correction
contracts, checking complete arguments, strict fault/accuracy/ratio conditions,
call order, no re-gating, failure frames and publication only after actual success.
Public failures remain in scope. The proof backend is the same fixed-array
`solve_proof::Backend` used by the correction dependencies.

Accepted results: 28/28 profiles in 681.41 s, cgroup peak 1,049,473,024 bytes
(0.98 GiB), no OOM/swap; 120 s / 4 GiB internal and 6 GiB group limits. Full Lean
passes with `--wfail` (3,422 jobs). Fourteen ESBMC mutation counterexamples were
found, with eight matching positive profiles reused from the completed matrix.
All twelve mutation patterns also have passing native controls and failing
native witnesses in both formats (24 + 24). Patterns cover wrong root/axis,
ignored permission, overwritten raw diagnostics, ignored scalar failure,
unbounded innovation, bypassed rejection, inclusive strict thresholds, ignored
IMU fault, correcting with raw residual and misreporting correction failure.
Native navigation-measurement tests, 100 offline review-tool tests, runner
inventory, reference resolution, formatting and ShellCheck pass. Inventory:
1,905 profiles / 174 entry points. No full AI audit.

Retained failed attempts: Bitwuzla exhausted memory on the combined limiter;
CVC5 exhausted memory on an expanded caller. The accepted split proves the
actual limiter once, then composes its exact relation. Shared scalar bound
lemmas avoid repeating identical arithmetic for every shape/alias. CVC5 is used
for scalar/limiter obligations; Bitwuzla for diagnostics/callers. Initial native
mutation linkage lacked an unused nondeterministic scalar symbol; the fixture
now aborts if that unexpected path is called. Repeated symbolic mutation search
was stopped after the recorded 14 counterexamples; all fixed witnesses were
checked natively. Interrupted runs, parse/link failures and timeouts are not passes.

Self-review checked every production branch, producer/consumer type and premise,
all input/output fields, and target assertion failures. It caught and added the
root-argument observer before the accepted matrix. IEEE boundedness requires a
finite nonnegative scalar root; root accuracy, target backends, physical health,
module/replay scheduling and machine-checked Lean-to-C++ refinement remain
separate. Production and unrelated drafts are unchanged. Evidence:
`output/runtime-proof-continuation-20261115/navigation-recovery-manifest.json`,
`navigation-recovery-final/results.json`, `navigation-recovery-mutations.json`,
`navigation-recovery-native-mutations.json`, Lean/native/tooling logs and retained
unsuccessful attempts.

## Height offset and admission producers

Seven Lean theorems cover `R-HEIGHT-FUSION`'s real scalar gain, unclamped
posterior variance and Joseph equivalence, convex mean update/residual
contraction, and offset transport under a Down-position reset. The transport
identity precedes source-specific re-alignment. Neither variance reduction
after the C++ floor nor floating-point error bounds are claimed.

The `height` suite adds 28 ESBMC profiles / 11 entry points: 20 production
producer/query profiles and eight standard-library model controls. Both scalar
formats, both source branches and every relevant field are checked. Actual
`HeightBias` initialization/finite classification, prediction and update retain
the specified IEEE equations, thresholds, floors, ceiling and timers, including
nonfinite scratch. Its finite posterior retains a nonnegative prior variance.
Actual `HeightAiding` construction/initialization binds all defaults explicitly;
stop/admission/query checks cover timestamp equality/future data, metadata and
quality, source/calibration clearing, reference/diagnostic revocation, returned
reference identity, reset timing and selected-innovation sign/suppression.
Value frames preserve signed zeros and NaN classification, not payloads.

These are private producer lemmas, not yet an inductive proof of the complete
height owner. Prediction requires finite/nonnegative variance and interval plus
a valid ceiling; admission requires chronological watermarks. The public
transition must establish those premises. Warmup/filtering, numerical callee
composition, successful/failed whole-tick publication and history induction,
actual module/replay wiring and target-library implementation remain separate.

ESBMC 8.4.0 required narrowly scoped library/name models: its `min(double)`
returns int, `any_of` is absent, a member named `finite` is replaced by a C
intrinsic applied to `this`, and its nested `std::array<Session,2>` initialization
zeroes nonzero element defaults and crashes on copy construction. The accepted
model retains the actual element constructors/copy operations in named fixed
storage, with independently checked indexing, copy/assignment/self-assignment,
non-aliasing and bounded mutable/const iteration. It models the operations used
by this owner, not target STL layout/contiguity/ABI. Existing `std::max` controls
are reused. No production body is rewritten. Reference-return calls use explicit
bindings/return branches to avoid the previously recorded converter defects.

Accepted checks: 28/28 in 46.26 s; cgroup peak 163,246,080 bytes (156 MiB), no
OOM/swap, 120 s / 4 GiB internal and 6 GiB group limits. Full Lean passes with
`--wfail` (3,423 jobs) after reducing the new module's imports. All 21 production
mutation patterns fail with their passing controls in both native formats
(42 + 42); six fixed witnesses and six controls also pass the expected ESBMC
outcomes. Patterns cover defaults/finite guards, offset thresholds, strict gate,
gain sign, timer/ceiling selection, clearing, future/expired data, quality,
fault/warmup reset and reset/innovation diagnostics. Native std::array/model
comparison passes 128 histories per format; min/max reference controls cover
484 pairs, finite classification 154 cases, any_of 12 ranges and unsigned min
16 pairs. Existing Eigen/PX4-matrix height tests, 100 offline review-tool tests,
runner inventory, reference resolution, formatting and ShellCheck pass.
Inventory: 1,933 profiles / 185 entry points. No full AI audit.

Retained unsuccessful attempts include missing-library parsing, wrong `finite`
classification, constructor-default counterexamples, copy-converter backtraces,
conditional-reference counterexamples and cross-source solver timeouts. Native
controls reproduce correct source behavior. Literal source partitioning reduced
the formerly timed-out stop query to about one second without dropping either
source or any state assertion. The earlier experimental contiguous-array model
is not used; failed/interrupted attempts are not passes.

Self-review checked every field/default, all actual producer branches, exact
backend/scalar identity, library-model assumptions and negative assertion
messages. Production and unrelated drafts are unchanged. Evidence:
`output/runtime-proof-continuation-20261115/height-producers-manifest.json`,
`height-complete-producers/results.json`, `height-native-mutations.json`,
`height-esbmc-mutations.json`, `height-producers-lean-minimal.log`, native
library/storage/height logs, tooling logs and retained failed attempts.

## Height caller's exact reset backend

Six additional navigation-reset profiles close the backend-identity prerequisite
for height composition: binary32/binary64 size-one validation and both in-place
vertical resets with `solve_proof::Backend`. They reuse the existing exact-status,
all-field frame, overflow, covariance-coefficient and argument assertions; no
cross-backend equivalence is assumed. The previous 40 default configurations have
identical preprocessed C++ tokens, and their runner arguments remain checked.

Accepted checks: 6/6 in 80.28 s, cgroup peak 1,588,203,520 bytes (1.48 GiB), no
OOM/swap. Four native mutated executions (wrong Down axis and lost position
variance, both formats) fail their intended assertions; all four controls pass.
Runner inventory, 100 offline review-tool tests, formatting, ShellCheck and
reference checks pass. Inventory: 1,939 profiles / 185 entry points. No new Lean
theorem or AI audit. Height transaction/history induction remains separate.

Self-review confirmed exact backend/scalar identity, the actual in-place alias
mode, the unchanged original evidence boundary and all source hashes. Production
and unrelated drafts are unchanged. Evidence:
`output/runtime-proof-continuation-20261115/height-reset-manifest.json`,
`height-reset-final/results.json`, `height-reset-default-equivalence.json`,
`height-reset-native-mutations.json` and runner/tooling logs.

## Height warmup and bounded altitude filter

Four additional height profiles prove the actual barometer warmup/filter's exact
IEEE expression, all-field frame, fault rejection, first-sample handling and
counter saturation at 21 in both formats. From an admitted altitude and a prior
LPF within ±131072 m, a separate scalar lemma proves that the computed LPF stays
finite and within that bound, including rounding. The loose bound is an
induction aid, not an operational threshold. The complete owner must establish
the count/range premises; this is not yet its public transition proof.

Accepted checks: 4/4 in 19.53 s, cgroup peak 336,683,008 bytes (321 MiB), no
OOM/swap. Five faulty variants (warmup boundary, counter saturation, gain,
fault suppression and offset sign) fail with passing controls in both native
formats (10 + 10); five fixed witnesses and five controls also produce the
expected ESBMC outcomes. Native Eigen/PX4-matrix height tests, runner inventory,
100 offline review-tool tests, references, formatting and ShellCheck pass.
Inventory: 1,943 profiles / 187 entry points. No new Lean theorem or AI audit.
The initial native fixture lacked an unused nondeterministic-math link symbol;
it was corrected with an aborting definition and all controls rerun.

Self-review checked the private premises, their pending owner responsibility,
the actual producer-to-scalar formula correspondence and every preserved field.
No production change. Evidence:
`output/runtime-proof-continuation-20261115/height-filter-manifest.json`,
`height-filter-final/results.json`, `height-filter-native-mutations.json`,
`height-filter-esbmc-mutations.json` and native/tooling logs.

## Accelerometer filter and fusion conditions

Six Lean theorems establish the component-wise real LPF's affine form,
constant-input preservation, convex bounds, one-step residual contraction and
exact error after any number of constant-input steps. Six ESBMC profiles / three
entry points check the actual magnitude constructor/reset/update and stateless
fusion predicate in both scalar formats. The implementation checks exact encoded
coefficients, vector-before-norm order, strict band limits, status priority,
rollback, borrowed-input frames and accessor identity. Finite history has an
initialization base and an inductive public-update step. The predicate exhausts
all eight flags and statuses, including arbitrary unused IEEE diagnostics.
This does not identify physical rest/gravity, establish caller freshness, verify
target libraries or close the sample-integration/fusion/module chain.

Accepted checks: 6/6 in 6.98 s, cgroup peak 165,519,360 bytes (158 MiB), no OOM/swap.
Lean passes with `--wfail` (3,424 jobs). Twelve mutation patterns in both native
formats fail their intended assertions with all controls passing (24 + 24);
six fixed ESBMC mutants and six controls give the expected outcomes. These cover
wrong coefficient/seed/norm, early writes, missing finite checks, inclusive band,
incomplete reset, ignored enable/clipping/aiding/status. Native Eigen/PX4-matrix
magnitude/condition tests, runner inventory, 100 offline review-tool tests,
reference resolution, formatting and ShellCheck pass. Missing native executables
on the first CTest attempt were built, then all four tests rerun successfully;
the initial not-run log is retained. Inventory: 1,949 profiles / 190 entry points.
No full AI audit or machine-checked Lean-to-C++ refinement.

Self-review checked exact expression order and scalar/backend identity, all
history fields, all status paths, boolean truth-table rows and negative witnesses.
Production and unrelated drafts are unchanged. Evidence:
`output/runtime-proof-continuation-20261115/accelerometer-runtime-manifest.json`,
`accelerometer-runtime-final/results.json`,
`accelerometer-runtime-native-mutations.json`,
`accelerometer-runtime-esbmc-mutations.json` and Lean/native/tooling logs.

### Retained height-composition gap

The reset dependencies and filter producers above are complete within scope;
whole `HeightAiding::reset_height/recover/process/update` composition is not.
The scratch recovery query hit the 4 GiB solver limit with CVC5 and Bitwuzla.
Using the proved reset write frame and arbitrary-row/column quantification let
the isolated state/P claims pass, but the grouped result/history arithmetic
still exhausted Bitwuzla memory; CVC5/Z3 attempts timed out. Individual passed
claims from an incomplete run are not counted as a completed profile. The
initial independent health oracle also lacked future/inconsistent-time guards;
that was corrected before later attempts, without claiming the earlier run.

Retained evidence includes `height-recovery-probe.log`,
`height-recovery-bitwuzla.log`, `height-recovery-frame.log`,
`height-reset-producer-probe.log`, `height-reset-producer-properties.log`,
`height-reset-indexed.log` and `height-reset-indexed-z3.log` under the same output
directory. Experimental harnesses remain outside the accepted proof suite.
Further composition needs smaller independently justified scalar/frame lemmas;
no failed path or public input was removed to force acceptance. Independent
accelerometer obligations proceeded while this gap remained explicit.
