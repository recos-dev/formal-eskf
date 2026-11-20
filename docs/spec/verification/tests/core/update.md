# Core correction, injection and reset tests

**Status:** Draft

**Scope:** Generic correction, innovation-covariance preparation, INS bias
permissions, error injection, covariance reset and atomic composed operations.
Sensor geometry and magnetic gain projection belong to [observations](observations.md).

**Level / purpose:** Unit and component / requirements verification and regression.

**Profiles:** [Shared profiles](../profiles.md): INS/AHRS, binary32/binary64 and both
reset modes. INS generated and dense arithmetic must each be exercised. Generic
arbitrary-H correction and all AHRS products remain dense even when codegen is on;
INS reset after generic correction still follows codegen selection. Repeat selected
injection/correction fixtures under both attitude-prediction settings to establish
that this unrelated setting does not alter them.

**Setup:** Use the independently constructed PSD matrices and unit attitudes in
[propagation](propagation.md), plus SPD measurement covariance `V=B B^T+D` with
positive dyadic diagonal D and nonzero correlations. Include zero/rank-deficient P,
nonzero cross-covariances involving every state block, arbitrary dense H and
measurement dimensions 1, 2, 3 and one larger generic dimension (4). The finite set
is representative generic-template coverage, not proof for every possible size.

Independently evaluate C, S, the right linear solve, constrained gain, Joseph
covariance, quaternion injection and reset in high precision, with reviewed
reference enclosures and implementation error bounds. For exact fixtures choose
diagonal/power-of-two systems and zero attitude correction so every expected value
is representable. Use residual checks as supplementary evidence, not a substitute
for checking the complete answer. Never calculate expected values using production
`correct`, `inject_and_reset`, backend solve or generated kernels. Expected failure
statuses/rollback are exact. This family inherits the unresolved-bound and safe
quaternion-fixture restrictions in [propagation](propagation.md#gaps-and-exclusions).

**Regression:** Re-run affected callers as well as primitive cases after changes to
solve, correction products, policies, injection, reset or code generation. Exercise
full transactions in the same backend/scalar/mode profile as their dependencies.

## Cases

| Case ID | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|
| TC-E-INJECT-local-map | [E-INJECT](../../../requirements/core/injection.md#required-behavior), additive/right map | Zero and distinct nonzero errors in each block; nonidentity q and nonparallel delta-theta; q/-q pair; both configurations. | Independent additive equations and normalized Hamilton q*Exp(delta-theta) give every nominal field. No covariance write and input error is retained. Exact zero/dyadic additive fixtures plus bounded rotational oracle detect wrong offsets, left injection, doubled rotation and premature error clearing. | Native: [test_injection_reset.cpp](../../../../../tests/core/test_injection_reset.cpp) |
| TC-E-INJECT-mode-independence | [E-INJECT](../../../requirements/core/injection.md#required-behavior), Exp always | A finite rotation where normalized Euler differs materially from Exp; repeat under both prediction approximation settings and reset settings. | Same injection model in every build: independent Exp oracle, not approximate prediction. Distinguishes compile-time macro leakage from ordinary rounding. | Partial: [test_injection_reset.cpp](../../../../../tests/core/test_injection_reset.cpp); finite-angle mode regression; general quaternion numerical envelope remains open |
| TC-E-INJECT-failure-frame | [E-INJECT](../../../requirements/core/injection.md#required-behavior), validation/atomicity | Nonfinite consumed error/state/threshold; zero/negative/above-one threshold; finite additive overflow after successful attitude injection; reachable Exp/normalization failures. Separate and whole-state alias output. | Required input/domain/generated-result/dependency statuses; every nominal output unchanged on failure, distinct state/error unchanged. Norm-feasibility checks must not become prediction's residual-tolerance policy. Native malformed-q coverage is excluded only as explicitly recorded below. | Native: [test_injection_reset.cpp](../../../../../tests/core/test_injection_reset.cpp) |
| TC-E-RESET-full-transport | [E-RESET](../../../requirements/core/covariance-reset.md#required-behavior), G P G^T | Zero, small and finite multi-axis delta-theta, PSD P with all cross-blocks nonzero; exact and approximate reset; both configurations. | Independent G uses right Jacobian or I-hat(theta)/2 at correct offset. Every covariance entry matches independently evaluated GPG^T; zero increment is identity on exact fixture. Detects wrong Jacobian sign, offset, missing cross-covariance transport and accidental zeroing before reset. | Partial: [test_injection_reset.cpp](../../../../../tests/core/test_injection_reset.cpp); exact dyadic and axis-invariant fixtures; general exact-Jacobian numerical envelope remains open |
| TC-E-RESET-consumption-frame | [E-RESET](../../../requirements/core/covariance-reset.md#required-behavior), consumed fields/failure/alias | Poison unused INS Euclidean error fields; separately poison attitude error/P or cause finite Jacobian/sandwich overflow. Separate output and whole-P alias. | Unused errors do not matter; consumed nonfinite inputs fail; generated/dependency errors propagate; failures preserve complete P output and all distinct inputs. No requirement for standalone reset to zero the caller error or validate PSD. Detects excessive validation, partial writes and undocumented state mutation. | Native: [test_injection_reset.cpp](../../../../../tests/core/test_injection_reset.cpp) |
| TC-E-RESET-arithmetic-selection | [E-RESET](../../../requirements/core/covariance-reset.md#required-behavior), generated/dense behavior | Full-transport fixtures in INS codegen 0/1, AHRS 0/1, both reset modes; inspect selected caller/kernel interface. | Both INS variants satisfy the same independent GPG^T equation within separately justified bounds; generated upper triangle is mirrored, dense path cleans symmetry. AHRS stays dense. No bitwise or extreme-input failure equivalence is demanded. Detects wrong G extraction, generated dispatch and absent mirror. | Pending |
| TC-E-STEP-INJECT-RESET-transaction | [E-STEP-INJECT-RESET](../../../requirements/core/injection-reset-step.md#required-behavior), [E-STEP-FRAME](../../../requirements/core/composed-steps.md#required-behavior) | Both stages succeed; injection fails; injection succeeds but reset fails; all eight independent state/P/error whole-object alias arrangements and arbitrary constructible old outputs. | Only full success publishes injected state, reset P and zero in every error coordinate. Reset sees original P and pre-zero error. Failure preserves all three outputs and first failing status. Independent equation/rollback oracle plus trace evidence detects stale arguments, early error clearing or partial commit. | Partial: [test_injection_reset.cpp](../../../../../tests/core/test_injection_reset.cpp); native publication and all eight aliases; skipped-callee property remains source/formal evidence |
| TC-E-CORRECT-preparation | [E-CORRECT](../../../requirements/core/correction.md#required-behavior), `correction_covariances` | PSD P, arbitrary H, correlated V and finite r/threshold; dimensions 1/2/3/4; separate disjoint outputs. Include zero H/P and nontrivial full cross-covariance. | Independently computed C=PH^T and S=H C+V are returned together, with only computed S symmetry cleanup. No solve/nominal state needed; r and threshold are still validated. Detects transpose/dimension mistakes, discarded V correlations and an unnecessary factorization dependency. | Native: [test_correction.cpp](../../../../../tests/core/test_correction.cpp) |
| TC-E-CORRECT-matrix-domain | [E-CORRECT](../../../requirements/core/correction.md#required-behavior), validation | Nonfinite P/r/H/V/threshold, threshold at/outside (0,1], negative P diagonal, nonpositive V diagonal, one-ULP asymmetry in either matrix; combine nonfinite with domain failures. Also valid singular PSD P with SPD V. | Nonfinite input precedes domain checks; invalid checked domain returns `domain_error`, without symmetrizing caller inputs. Valid singular PSD P is supported. Necessary diagonal/symmetry checks are not falsely treated as a full PSD/PD validator. Exact classification and unchanged outputs detect missing or overstrong checks. | Native: [test_correction.cpp](../../../../../tests/core/test_correction.cpp) |
| TC-E-CORRECT-right-solve | [E-CORRECT](../../../requirements/core/correction.md#required-behavior), KS=C; [F-SOLVE](../../../requirements/math/linalg/spd-solve.md#required-behavior) | Small independently solvable systems with dense correlated S and non-square K; vary P rank and V scale within a reviewed well-conditioned domain. Include positive-diagonal indefinite S produced from deliberately invalid caller matrices solely to exercise checked factorization rejection. | Independent solution and KS-C residual validate orientation and every gain column via final correction. Factorization failure propagates; no explicit matrix inverse fallback. Outside-premise indefinite fixtures claim only the specified solve failure, not valid estimator mathematics. Conditioning acceptance beyond the selected domain remains a gap. | Partial: [test_correction.cpp](../../../../../tests/core/test_correction.cpp); exact correlated rational gain and posterior; general conditioning and scale budgets remain open |
| TC-E-CORRECT-joseph-prior | [E-CORRECT](../../../requirements/core/correction.md#required-behavior), same prior/effective gain | Dense arbitrary H, correlated V, full prior P; nonzero r; exact diagonal zero-attitude fixture and general cross-coupled fixture. | Independent K gives delta=Kr and full Joseph P_c from the same prior P, followed by independent injection/reset. Returned P must be post-reset. Check every field/entry; detects simplified `(I-KH)P`, dropped KVK^T, updated-prior reuse and omitted reset. | Partial: [test_correction.cpp](../../../../../tests/core/test_correction.cpp); exact cancellation and independent full-state regression; general nonlinear error envelopes remain open |
| TC-E-CORRECT-zero-information | [E-CORRECT](../../../requirements/core/correction.md#required-behavior), zero prior/error behavior | H=0 with nonzero r; P=0 with valid V; and r=0 with informative H and nonzero P. | First two give zero gain/error and preserve corresponding prior covariance mathematically. With r=0 the nominal state is physically unchanged but covariance still updates. Exact diagonal fixtures expose accidental dependence on residual magnitude or skipped covariance update. Quaternion normalization need not preserve coefficient bits. | Native: [test_correction.cpp](../../../../../tests/core/test_correction.cpp) |
| TC-E-CORRECT-arithmetic-boundaries | [E-CORRECT](../../../requirements/core/correction.md#required-behavior), finite intermediates | Isolate finite-input overflow in C, S, Kr, I-KH and each Joseph contribution; preparation outputs start as distinct sentinels. Use internal validated gain application only where a public fixture cannot isolate its branch, documenting that narrower caller premise. | `non_finite_result` at generated arithmetic boundaries; no preparation pair or correction output partially publishes. Independent IEEE classifications detect overflow masking, false success and lost finite-result checks. Internal helper results are not new unchecked-public guarantees. | Native: [test_correction.cpp](../../../../../tests/core/test_correction.cpp) |
| TC-E-CORRECT-model-dispatch | [E-CORRECT](../../../requirements/core/correction.md#required-behavior), generic versus known model | Generic H of dimensions 1/2/3 deliberately different from every selector/body-vector model; also 4-row H. For each built-in model use its exact H and a nonzero correlated P/V; both codegen modes. | Generic path honors arbitrary H regardless of dimensions. Each built-in C/S/Joseph kernel satisfies independently evaluated equations, including all constrained-gain entries. INS reset may be generated after generic correction. Compare against a separate model oracle, not merely dense versus generated output. Detects dimension-based dispatch and specialized-kernel premise violations. | Pending |
| TC-E-BIAS-UPDATE-all-permissions | [E-BIAS-UPDATE](../../../requirements/core/bias-update.md#required-behavior), all 64 controls | All 64 six-axis masks, nonzero bias cross-covariances and correlated V; both reset modes; omitted/default/all-enabled mask comparison. AHRS API must not accept INS-only control. | Independently compute K then zero exactly disabled rows 9..14; use that K for both error and full Joseph update. Disabled estimates retain values, uncertainty is not erased, other rows unaffected. All-enabled equals unrestricted. Detects axis mixup, H modification, covariance row deletion and inconsistent effective gains. | Partial: [test_correction.cpp](../../../../../tests/core/test_correction.cpp); all 64 masks with exact Joseph controls; general correlated finite-angle error envelope remains open |
| TC-E-BIAS-UPDATE-call-local | [E-BIAS-UPDATE](../../../requirements/core/bias-update.md#required-behavior), no bypass/persistence | Correction with all permissions disabled, first invalid then valid; next correction enabled; then prediction with nonzero bias random walk. | Disabled mask does not bypass validation/solve or freeze later calls/prediction noise. Independent equations show bias variances can evolve and cross-covariances can change despite zero bias correction. Detects persistent freeze and incorrectly disabled covariance arithmetic. Wrapper forwarding is allocated in observations. | Native: [test_correction.cpp](../../../../../tests/core/test_correction.cpp) |
| TC-E-STEP-CORRECT-transaction | [E-STEP-CORRECT](../../../requirements/core/correction-step.md#required-behavior), [E-STEP-FRAME](../../../requirements/core/composed-steps.md#required-behavior) | Success and failures at validation/preparation, solve, error/Joseph finalization, injection and reset; all four whole-state/P alias arrangements; arbitrary constructible old outputs. | Exact first-failure status, complete rollback on failure and joint final publication. Trace/model evidence confirms shared prior, final effective gain and pre-zero reset error. Re-running valid correction after failure matches a fresh reference fixture. No rollback reliance on successful scratch contents. | Partial: [test_correction.cpp](../../../../../tests/core/test_correction.cpp); all aliases and public stage failures; skipped calls and internal normalization reachability remain source/formal obligations |
| TC-E-STEP-independent-frames | [E-STEP](../../../requirements/core/composed-steps.md#required-behavior), frame across composed APIs | Execute the predict, inject/reset and correct transaction families while observing disjoint inputs, IMU, parameters, V, H/r and unrelated output sentinels. For a callee-summary harness allow failing scratch to be overwritten. | Only named outputs change, only on success; copies preserve signed zero and NaN classification where required. Caller commit logic tolerates arbitrary failed scratch; no padding/payload promise. Real-body cases remain required alongside summary evidence. Detects hidden side effects and invalid summary frame assumptions. | Pending |
| TC-SW-CORE-composition | [SW-CORE](../../../requirements/software/core-estimation.md#required-behavior), [E-STEP](../../../requirements/core/composed-steps.md#required-behavior) | Short deterministic prediction/correction sequences with variable dt, gravity and magnetic observations, INS navigation observations, full P and nonzero noise. Recompute each observation at its current prior; use independent measurement-noise groups. Include subsequent prediction and failure/retry branches. No scheduler or integration module. | Independent full-state/reference timeline confirms old-state linearization, one noise discretization/injection per prediction, effective gain reuse including magnetic projection, error reset and next-step covariance coordinates. A failed transaction preserves its entry state but does not roll back an earlier successful independent call. Detects composition errors and interacting macro selection not exposed by isolated identities; no long-run accuracy claim. | Pending |

## Source scope

Paths below are relative to `include/formal_eskf/eskf/`.

| Header / API group | Case allocation |
|---|---|
| `injection.hpp`: `inject_nominal`, `reset_covariance`, `inject_and_reset`, both configurations and helpers | `TC-E-INJECT-*`, `TC-E-RESET-*`, `TC-E-STEP-INJECT-RESET-transaction`. |
| `correction.hpp`: `correction_covariances`, both `correct` overloads; input validation, gain, unpack and gain application helpers | `TC-E-CORRECT-*`, `TC-E-BIAS-UPDATE-*`, `TC-E-STEP-CORRECT-transaction`. |
| `correction_products.hpp`: arbitrary-H and five built-in model products | `TC-E-CORRECT-model-dispatch`, `TC-E-CORRECT-preparation`, `TC-E-CORRECT-joseph-prior`; actual wrappers in [observations](observations.md). |
| `generated/ins_innovation.hpp`: five cross/innovation pairs | `TC-E-CORRECT-model-dispatch`; generated helpers require their named model's H, not arbitrary same-size H. |
| `generated/ins_joseph.hpp`: five Joseph kernels | `TC-E-CORRECT-model-dispatch`, `TC-E-BIAS-UPDATE-all-permissions` and magnetic policy cases in [observations](observations.md). |
| `generated/ins_reset.hpp` | `TC-E-RESET-arithmetic-selection`, generic/built-in correction transactions. |
| Shared `covariance.hpp`, `codegen.hpp`, configuration/types | [Propagation allocation](propagation.md#source-scope), exercised through these callers as well. |

## Gaps and exclusions

- [C-NUMERICAL-ACCEPTANCE](../../../requirements/common/numerical-acceptance.md)
  does not yet supply all solve conditioning/error bounds or floating-point
  covariance/attitude tolerances. A stable high-precision result is not itself a
  certified allowance for production error. Mark affected general numeric
  partitions blocked until reviewed bounds exist; do not claim IEEE PSD from
  symmetry or Joseph form.
- Successful diagonal/symmetry checks do not establish the caller's PSD P/SPD V
  premises. Outside-premise fixtures can exercise an explicitly checked rejection
  but cannot impose rejection of every indefinite input.
- The [propagation gaps](propagation.md#gaps-and-exclusions) apply to invalid
  quaternion construction and call tracing. Fault substitution may establish
  caller rollback/order only; each production producer still needs real-body
  evidence in the matching profile.
- Dimension 4 checks generic behavior beyond built-in sensors; unbounded template
  sizes, arbitrary overlapping storage, target timing and integration sequencing
  are not covered by these finite native cases. Shared resource requirements have
  separate allocations.

## Review

Pending. All executable bindings and numerical acceptance records remain to be
implemented/reviewed; this design does not credit previous tests as coverage.
