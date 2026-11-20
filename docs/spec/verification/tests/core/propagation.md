# Core state and propagation tests

**Status:** Draft

**Scope:** INS/AHRS state conventions, nominal prediction, discrete process noise,
covariance prediction and their atomic composition. Integration adapters, sensor
availability policy and navigation accuracy are excluded.

**Level / purpose:** Unit and component / requirements verification and regression.

**Profiles:** [Shared profiles](../profiles.md). Use binary32 and binary64, INS and
AHRS where applicable, both `ESKF_QUAT_APPROX` settings, and both `ESKF_CODEGEN`
settings for INS covariance. Reset and magnetic settings do not affect these
operations. Dependencies must be exercised with the same backend as their caller.

**Setup:** Construct states through public types and checked quaternion construction.
Use identity, signed axis rotations and a non-axis-aligned unit attitude; distinct
nonzero position, velocity and biases; dyadic time and noise values. Construct
symmetric PSD covariance independently as `L D L^T` using a small integer lower
triangular L and nonnegative dyadic D: include zero, rank-deficient and positive
definite cases, with nonzero off-diagonal blocks. Never use a production predictor,
rotation or matrix product to calculate its own expected result.

Use exact rational arithmetic for representable polynomial fixtures. For general
rotation/matrix cases, evaluate the stated equations in an independent high
precision implementation with a certified reference enclosure, then apply a
reviewed operation-specific rounding/truncation bound. An enclosure for the real
answer alone is not a floating-point implementation error allowance. Unspecified
bounds block numeric acceptance of those partitions; they are not arbitrary epsilons.
Exact statuses, selector values, copied fields and rollback need no numeric tolerance.

**Regression:** Run affected cases after changes to configuration, prediction,
noise, covariance, code generation or their math/backend dependencies. Run the
complete applicable profile allocation before accepting a propagation change.

## Cases

| Case ID | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|
| TC-E-STATE-shape | [E-STATE](../../../requirements/core/state.md#required-behavior), configuration dimensions/order | Instantiate INS and AHRS through `EskfTypes`; independently populate every named state/error field and exercise each error block through injection. Compile accepted API shapes and rejected cross-configuration shapes. | INS nominal/error/noise dimensions are 16/15/12; AHRS 4/3/3. Distinct error coordinates reach only their named logical fields before rotation coupling. No binary-layout assertion. Detects dimension, offset and configuration leakage. Injection arithmetic is allocated in [update](update.md). | Native: [test_types.cpp](../../../../../tests/core/test_types.cpp) |
| TC-E-STATE-frame | [E-STATE](../../../requirements/core/state.md#required-behavior), NED/right local error | Nonidentity prior with a local rotation about a different axis; repeat with q and -q. Use stationary NED-aligned force and separate navigation-axis translation. | Independent Hamilton/right-action model gives body-to-NED mapping and equivalent physical attitudes for either sign. Stationary specific force is opposite positive-Down gravity. Detects scalar order, left/right multiplication and gravity sign swaps; general rotations use bounded reference acceptance. | Native: [test_types.cpp](../../../../../tests/core/test_types.cpp) |
| TC-E-PRED-time-domain | [E-PRED-TIME](../../../requirements/core/prediction.md#required-behavior) | Test inclusive min/max, min=max, adjacent representable dt outside either end, zero/negative dt, reversed/nonpositive bounds, norm threshold zero/above one, and NaN/infinity in each consumed parameter. Combine bad time with bad prior/input. | Exact status: nonfinite parameters first; invalid parameter domain next; out-of-range dt next. Endpoints including min=max succeed with otherwise valid data. No output change on rejection. Detects exclusive endpoints, absent setup and wrong precedence. | Native: [test_prediction.cpp](../../../../../tests/core/test_prediction.cpp) |
| TC-E-PRED-prior-validation | [E-PRED-INPUT](../../../requirements/core/prediction.md#required-behavior), prior check order | Finite publicly constructible quaternions, including accumulated multiplication drift; tolerance NaN/infinity, zero, one, valid interior. For a nonzero computed norm residual, set tolerance at that residual and neighboring representable values, when constructible in (0,1). | Exact parameter statuses and inclusive computed-residual boundary; no implicit repair before validation. Time errors retain precedence. Independently reproduce specified scalar evaluation for the boundary classification, separately from the real-unit oracle. Publicly constructible malformed quaternion partitions follow the profile premises below. | Native: [test_prediction.cpp](../../../../../tests/core/test_prediction.cpp) |
| TC-E-PRED-consumed-fields | [E-PRED-INPUT](../../../requirements/core/prediction.md#required-behavior) | Poison each consumed INS Euclidean/IMU/gravity field individually with NaN/infinity. For AHRS poison specific force; for both poison process-noise fields while nominal inputs remain valid. | Consumed nonfinite input returns `non_finite_input`; unused fields do not affect nominal status/result. AHRS needs no nonexistent bias/position state. Detects accidental input reads and validation beyond the API's consumption contract. | Native: [test_prediction.cpp](../../../../../tests/core/test_prediction.cpp) |
| TC-E-PRED-ATTITUDE-selected-map | [E-PRED-ATTITUDE](../../../requirements/core/attitude-prediction.md#required-behavior) | Zero, single-axis and nonparallel multi-axis rates; nonidentity prior; nonzero INS gyro bias; moderate finite increments making exact/Euler results distinguishable; decreasing small increments. | Exact mode matches normalized right multiplication by Exp; approximate mode matches normalization of q multiplied by [1,theta/2]. Compare each to its own independent formula, not to each other at finite dt. Zero rate preserves physical attitude; small-increment difference follows the reference expansion and reviewed bounds. Detects wrong mode, sign, bias subtraction and multiplication side. | Partial: [test_prediction.cpp](../../../../../tests/core/test_prediction.cpp); exact zero/dyadic branches and supplemental finite-angle regression; general numerical envelope remains open |
| TC-E-PRED-INS-frozen-acceleration | [E-PRED-INS](../../../requirements/core/translation-prediction.md#required-behavior), [E-PRED-CONSISTENCY](../../../requirements/core/prediction.md#required-behavior) | Construct exact dyadic stationary, constant-velocity and nonzero-acceleration fixtures; then tilted prior with nonzero angular rate, biases and all velocity components. | Independent equations use old q/biases/v: p+=v*dt+a*dt^2/2 and v+=a*dt. Bias estimates copy unchanged. Exact fixtures require equality; general rotation uses bounded reference. Detects missing half factor, use of updated velocity/attitude and body/navigation confusion. | Native: [test_prediction.cpp](../../../../../tests/core/test_prediction.cpp) |
| TC-E-PRED-arithmetic-failures | [E-PRED-SUCCESS / FAILURE](../../../requirements/core/prediction.md#required-behavior) | Finite inputs isolating overflow in bias subtraction, rate*dt, force rotation/addition or position/velocity arithmetic; include attitude success followed by translation failure. | `non_finite_result` for generated nonfinite arithmetic; dependency statuses propagate. Every logical output is preserved, not replaced by identity or a partial candidate. Use independently computed IEEE exceptional classifications; do not assume cross-mode overflow equivalence. | Partial: [test_prediction.cpp](../../../../../tests/core/test_prediction.cpp); native finite-overflow partitions; every internal normalization-threshold failure is not independently isolated |
| TC-E-PRED-alias-frame | [E-PRED-ATOMICITY](../../../requirements/core/prediction.md#required-behavior) | Success and each reachable failure from preceding cases, with separate output or whole-state alias. Seed unrelated Euclidean output fields with distinct values, signed zero and NaN; quaternion sentinels must be constructible. | Alias and disjoint calls agree within the same profile; failures preserve logical fields, signed zero and NaN classification. Distinct inputs/IMU/parameters remain unchanged. Detects early publication and alias-dependent RHS reads. | Native: [test_prediction.cpp](../../../../../tests/core/test_prediction.cpp) |
| TC-E-NOISE-source-scaling | [E-NOISE](../../../requirements/core/process-noise.md#required-behavior), source order/scaling | Set one source/axis at a time, then all unequal; zero variance; dyadic dt and 2*dt. Exercise INS12 and AHRS3. | Independent diagonal oracle: sample variance scales by dt^2 and bias density by dt; expected ratio 4 or 2 respectively. Off-diagonals exactly zero; AHRS equals the angular source block. Detects squaring a variance, source offset errors, wrong time exponent and premature frame rotation. | Native: [test_covariance.cpp](../../../../../tests/core/test_covariance.cpp) |
| TC-E-NOISE-validation-order | [E-NOISE](../../../requirements/core/process-noise.md#required-behavior), validation/status order | Zero/negative/nonfinite dt; negative/nonfinite entries in each group; paired faults within a group and in successive INS groups. | Exact dt nonfinite/positive-domain and group-order statuses; nonfinite wins over negative within a group. Failure preserves all output entries. A positive dt outside prediction's configured bounds is valid here. Detects undocumented prediction limits and reordered validation. | Native: [test_covariance.cpp](../../../../../tests/core/test_covariance.cpp) |
| TC-E-NOISE-ieee-scaling | [E-NOISE](../../../requirements/core/process-noise.md#required-behavior), left-to-right arithmetic | Zero variance with finite dt whose square would overflow; tiny positive variance/time causing underflow; finite operands causing sample or density product overflow. | Independent IEEE evaluation of `(variance*dt)*dt` or `density*dt` gives exact classification and representable fixture values. Zero noise must not fail merely because dt*dt overflows. Generated nonfinite results return `non_finite_result` with unchanged output. Detects reassociation and missing result checks. | Native: [test_covariance.cpp](../../../../../tests/core/test_covariance.cpp) |
| TC-E-PCOV-full-transition | [E-PCOV](../../../requirements/core/covariance-prediction.md#required-behavior), F and full sandwich | Independent PSD fixtures with cross-covariances in every block; nonidentity q, force, rate and biases; zero/rank-deficient/full-rank P. Both attitude modes. | Build full F independently from required blocks and evaluate FPF^T+FiQiFi^T with reference arithmetic. Check every entry, not only diagonal/symmetry. AHRS uses its own A P A^T+Q. Detects missing cross-blocks, wrong signs/offsets and mistaken exact full-system discretization. | Partial: [test_covariance.cpp](../../../../../tests/core/test_covariance.cpp); exact dyadic full-matrix fixtures plus supplemental finite-angle regression; general error envelope remains open |
| TC-E-PCOV-noise-frame | [E-PCOV](../../../requirements/core/covariance-prediction.md#required-behavior), Fi injection | Zero prior P, unequal body acceleration variances, nonidentity rotation; isotropic control; each other source block separately; two dt values. | Independent rotated covariance R Da R^T dt^2 appears in velocity block; isotropic case is rotation-invariant. No direct position-noise or undocumented cross-source blocks; angular/bias terms appear once at specified offsets/scales. Detects omitted rotation, double discretization/injection and added high-order terms. | Partial: [test_covariance.cpp](../../../../../tests/core/test_covariance.cpp); exact cyclic anisotropic and source scaling fixtures; nonaxis rotation regression is supplemental |
| TC-E-PCOV-consumption-failure | [E-PCOV](../../../requirements/core/covariance-prediction.md#required-behavior), checked domain | Nonfinite consumed P/rate/noise and INS q/bias/force; isolate transition/noise/product overflow. Independently poison unused INS p/v/gravity and AHRS force/quaternion tolerance. | Specified input/parameter/dependency/result statuses; unused fields have no effect. INS applies prior quaternion validation, AHRS covariance has no absolute-attitude input. PSD is caller premise: these tests do not demand a missing full-PSD runtime check. Detects invalid success and accidental state coupling. | Native: [test_covariance.cpp](../../../../../tests/core/test_covariance.cpp) |
| TC-E-PCOV-finalization | [E-PCOV](../../../requirements/core/covariance-prediction.md#required-behavior), [E-RESET](../../../requirements/core/covariance-reset.md#required-behavior), final publication | Exercise dense and generated covariance callers; directly unit-check internal finalization with finite unequal symmetric-pair candidates, large same-sign entries, and a nonfinite late entry. Internal fixture is not an invalid public prior. | Dense cleanup uses 0.5*a+0.5*b; diagonal unchanged. Generated already-symmetric output is mirrored and published without another average. Nonfinite candidates do not publish. Exact scalar oracle catches overflow from `(a+b)/2`, missing lower-triangle write and partial publication. These internal checks support public clauses rather than create public helper contracts. | Native: [test_covariance.cpp](../../../../../tests/core/test_covariance.cpp) |
| TC-E-PCOV-generated-scope | [E-PCOV](../../../requirements/core/covariance-prediction.md#required-behavior), arithmetic mode | Run the full-transition/noise fixtures in INS codegen 0/1; AHRS codegen 0/1; inspect compile-selected production caller dispatch. | Each result satisfies the independent equation with its own rounding bound; agreement alone is insufficient. INS selects its generated kernel only for mode 1, AHRS remains dense. Do not require bitwise/status equality near overflow boundaries. Detects unused/wrong kernel, invalid symmetry assumptions and tolerance masking. | Pending |
| TC-E-PCOV-alias-frame | [E-PCOV](../../../requirements/core/covariance-prediction.md#required-behavior), unchanged failure/alias | Every success/failure partition with separate output and complete P alias; old outputs include finite sentinels, signed zero and NaN. | Same-profile success relation and exact logical rollback; distinct P/state/IMU/parameters unchanged. Detects in-place overwrite before all products finish. | Native: [test_covariance.cpp](../../../../../tests/core/test_covariance.cpp) |
| TC-E-STEP-PRED-joint-publication | [E-STEP-PRED](../../../requirements/core/prediction-step.md#required-behavior), [E-STEP-FRAME](../../../requirements/core/composed-steps.md#required-behavior) | Nominal+covariance success; nominal failure with covariance otherwise valid; nominal success with invalid noise or covariance overflow. All four whole-state/P alias arrangements and arbitrary constructible old outputs. | Independent reference gives both final outputs from the same old state/P. First failing required operation determines status; neither output publishes on either failure. Trace/order evidence confirms covariance is skipped after nominal failure and receives old, not predicted, state. Output comparisons alone do not prove absence of a skipped call. | Partial: [test_covariance.cpp](../../../../../tests/core/test_covariance.cpp); independent exact joint oracle and all four aliases; skipped-callee property remains source/formal evidence |

## Source scope

Paths below are relative to `include/formal_eskf/eskf/`.

| Header / API group | Case allocation |
|---|---|
| `eskf.hpp`, `types.hpp`, `configuration/ins.hpp`, `configuration/ahrs.hpp` | `TC-E-STATE-shape`, `TC-E-STATE-frame`; complete operation exposure across all three core families; compile/resource profile checks are shared. |
| `prediction.hpp`: both `predict_nominal` overloads and parameter/prior/attitude helpers | `TC-E-PRED-*`, `TC-E-PRED-ATTITUDE-selected-map`, `TC-E-PRED-INS-frozen-acceleration`. |
| `process_noise.hpp`: both `discretize_process_noise` overloads and validators | `TC-E-NOISE-*`. |
| `covariance_prediction.hpp`: both `predict_covariance` and `predict` overloads, attitude transition | `TC-E-PCOV-*`, `TC-E-STEP-PRED-joint-publication`. |
| `covariance.hpp`: dense finalization and finite publication helpers | `TC-E-PCOV-finalization` plus correction/reset callers in [update](update.md). |
| `codegen.hpp`, `generated/ins_covariance.hpp` | `TC-E-PCOV-generated-scope`; macro validity/default/translation-unit consistency in shared profiles. No independent contract is assigned to generated temporaries. |
| Other ESKF headers | Exhaustively allocated in [update](update.md#source-scope) and [observations](observations.md#source-scope). |

## Gaps and exclusions

- General floating-point quaternion, rotation and covariance error bounds are not
  fully specified by [C-NUMERICAL-ACCEPTANCE](../../../requirements/common/numerical-acceptance.md).
  Exact fixtures are executable design; the general numeric partitions require
  approved domain/bound records before they can pass. Real PSD identities do not
  establish IEEE PSD or estimation accuracy.
- Malformed quaternion fixtures must use public operations; no test may bypass
  private storage or assume an object layout. Under the declared gradual-underflow
  profile, tiny coefficient normalization can produce a finite non-unit value;
  repeated public multiplication reaches finite norm overflow and nonfinite
  coefficients. Prediction exercises these guard partitions. This fixture does
  not establish a normalization accuracy bound or FTZ-profile acceptance.
- An executable call-order trace/seam is not yet selected. Real-body successful
  and failing transactions remain required; substituted callees alone are not
  evidence for component arithmetic or matching backend behavior.
- No arbitrary mismatched overlapping storage, non-PSD public prior behavior,
  concurrency, target deadlines or sensor model accuracy is specified here.

## Review

Pending. These are planned cases, not reviewed or executed coverage.
