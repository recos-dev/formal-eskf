# Core observation and innovation tests

**Status:** Draft

**Scope:** Position/velocity selectors, gravity and magnetic vector models,
their checked correction wrappers, magnetic tilt protection and the pure
componentwise innovation gate. Sensor health, reference generation, lifecycle,
gating policy and integration adapters have other owners.

**Level / purpose:** Unit and component / requirements verification and regression.

**Profiles:** [Shared profiles](../profiles.md): binary32/binary64; INS for position
and velocity; INS/AHRS for gravity and magnetometer; both `ESKF_MAG_TILT` settings
for magnetic correction; both reset modes and INS dense/generated arithmetic for
wrapper transactions. Prediction approximation is unrelated to these models.

**Setup:** Independently constructed nonidentity unit attitudes, distinct state
coordinates, full PSD P with cross-block correlations, and full SPD V with unequal
diagonal entries and correlations. Evaluate observation predictions and local
Jacobians from the canonical equations using independent high precision or exact
axis-rotation fixtures. A local derivative check uses the independently evaluated
right perturbation q*Exp(epsilon), signed coordinate directions and a reviewed
truncation/rounding enclosure; production injection/rotation cannot supply both
the tested function and its oracle. For accelerometer differentiation, hold the
preprocessed measurement fixed: varying prior bias would test a different model.

For either vector model, also inspect the instantaneous null direction
`hat(h) h = 0` within its existing Jacobian case; this is an algebraic rank check,
not a separate claim about long-term system observability.

For wrapper results, use the independent complete correction/injection/reset
oracle defined in [update](update.md). For gate arithmetic boundaries use an
independent scalar IEEE evaluation of the specified order and adjacent
representable inputs. Gate decisions use the computed ratio, not an exact-real
replacement. No universal numeric tolerance is introduced; general rotational,
solve and covariance partitions need reviewed operation-specific bounds.

**Regression:** Model changes run both standalone model and complete wrapper
cases, plus correction dependencies. Policy changes run projected/unrestricted
magnetic cases and INS bias-control interactions. Gate changes run all dimensions
and allowed input/output alias partitions.

## Cases

| Case ID | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|
| TC-E-OBS-POSITION-selectors | [E-OBS-POSITION](../../../requirements/core/position-observation.md#required-behavior), exact H/residual | Horizontal and vertical API; distinct N/E/D position; positive and negative Down measurements, nonzero velocity; exact selector inspection. | H has ones only at horizontal columns 0/1 or vertical column 2; every other entry zero. Independent r=z-p has correct sign and units. No vertical velocity input or altitude/range conversion. Exact fixture detects wrong axis and positive-up interpretation. | Pending |
| TC-E-OBS-POSITION-transaction | [E-OBS-POSITION](../../../requirements/core/position-observation.md#required-behavior), wrapper forwarding | Horizontal correlated 2x2 V; vertical scalar variance deliberately different from its square; full cross-covariance P and distinct bias masks. Nonfinite position/measurement/vertical variance; finite residual overflow; downstream correction failure. All four state/P aliases. | Independent complete update receives unchanged V/variance and permissions; off-position state may change through P. Pre-residual nonfinite and generated-residual statuses precede downstream errors as specified. Every failure rolls back both outputs. Detects variance-versus-standard-deviation confusion, dropped correlations and partial commit. | Pending |
| TC-E-OBS-VELOCITY-selectors | [E-OBS-VELOCITY](../../../requirements/core/velocity-observation.md#required-behavior), exact H/residual | Horizontal and 3D velocity; distinct N/E/D values and nonzero position; vary only measured vertical component for the 3D path. | H selects columns 3/4 or 3/4/5 exactly; r=z-v in NED. Independent linear oracle detects position-column, body-frame/speed and horizontal/vertical mixing. Horizontal path has no vertical measurement dependence; no differencing is introduced. | Pending |
| TC-E-OBS-VELOCITY-transaction | [E-OBS-VELOCITY](../../../requirements/core/velocity-observation.md#required-behavior), full V/failure frame | Full correlated 2x2/3x3 V and state cross-covariance; masks; nonfinite complete prior velocity (including its unused selector component), measurement and residual overflow; downstream failure; all four aliases. | Complete independent correction oracle validates all state/P entries and full V forwarding. Specified pre-residual input/generated statuses and rollback apply. A horizontal selector still checks the prior velocity vector as specified. Detects over-narrow validation, lost cross-covariance and gain policy omission. | Pending |
| TC-E-OBS-ACCEL-gravity-model | [E-OBS-ACCEL](../../../requirements/core/gravity-observation.md#required-behavior), h/r/local H | Stationary identity attitude and nonidentity q; nonzero gravity magnitude, unequal force components, nonzero INS prior accelerometer bias; AHRS calibrated equivalent input; nonzero constant navigation velocity. | Independent h=-R^T g, z=f-b_a for INS and calibrated f for AHRS, r=z-h, H=hat(h) in attitude columns only. Exact axis fixtures and bounded local derivative distinguish sign, body/navigation direction, removed gravity magnitude, factor two and erroneous velocity/cross-product terms. | Pending |
| TC-E-OBS-ACCEL-fixed-preprocessing | [E-OBS-ACCEL](../../../requirements/core/gravity-observation.md#required-behavior), fixed bias/units | Vary INS raw force and prior bias together so z remains fixed; freeze preprocessing during local Jacobian perturbation. In a separate AHRS fixture, scale force/gravity by s and V by s^2. Nonzero INS bias cross-covariance; enabled/disabled bias permissions. | Zero bias Jacobian blocks are exact; bias is subtracted once, not included again in h. INS error increment remains the same when z is fixed, with final bias relative to its changed prior. AHRS reference equations predict the same correction for consistent measurement-unit scaling. Cross-covariance may still update INS bias unless disabled. Detects joint-bias model substitution and vector normalization. | Pending |
| TC-E-OBS-ACCEL-domain-transaction | [E-OBS-ACCEL](../../../requirements/core/gravity-observation.md#acceptance-and-verification), input domain/checked wrapper | Zero gravity versus zero measured force; nonfinite consumed state/force/gravity; finite bias-subtraction/prediction/residual overflow; all state/P aliases; correlated V and downstream solve/injection/reset failures. | Zero gravity returns `domain_error`; zero force is algebraically permitted. Nonfinite input/generated result and downstream statuses propagate with both outputs unchanged. Complete INS state must be finite by the full transaction, without inventing precedence among otherwise unspecified independent failures. Successful model arithmetic does not certify near-zero physical acceleration. | Pending |
| TC-E-OBS-MAGNETIC-vector-model | [E-OBS-MAGNETIC](../../../requirements/core/magnetic-observation.md#required-behavior), fixed reference/local H | Nonaxis-aligned m_n and q; q/-q; independently transformed m_b plus known residual; AHRS/INS and both tilt modes. | Independent h=R^T m_n and r=m_b-h; positive hat(h), INS attitude columns 6..8 only. H is independent of tilt selection; no magnetic-bias state or quaternion factor two. Exact axis fixtures and bounded local derivative catch transpose/sign/reference mistakes. | Pending |
| TC-E-OBS-MAGNETIC-strength-domain | [E-OBS-MAGNETIC](../../../requirements/core/magnetic-observation.md#required-behavior), units/nonzero checks | Scale both fields and V consistently; zero either vector (including signed zeros), finite tiny nonzero fields, vertical-only reference, NaN/infinity in each field and finite residual overflow. | Raw microtesla values are retained; zero field gives `domain_error`, nonfinite input/result gets its specified status. Nonzero but physically weak/unhelpful fields are not rejected by invented core health thresholds. Independent scaled equations detect hidden normalization, magnitude-squared overflow in a zero test and misplaced runtime policy. | Pending |
| TC-E-OBS-MAGNETIC-wrapper | [E-OBS-MAGNETIC](../../../requirements/core/magnetic-observation.md#required-behavior), [E-OBS](../../../requirements/core/observations.md#required-behavior) | Full correlated V, cross-coupled P, nonzero residual; both configurations, tilt modes and INS masks; valid and downstream failing cases; all four aliases. | Independent complete update uses the same prior/reference, full vector H/r/V and selected effective-gain policy. Both state/P publish together or remain unchanged. No scalar heading substitution or residual wrapping. Detects stale linearization, dropped V and wrapper policy loss. | Pending |
| TC-E-MAG-TILT-projected-gain | [E-MAG-TILT](../../../requirements/core/magnetic-tilt.md#required-behavior), projector/final gain | Strongly tilted prior where navigation Down is not body Z; nonzero gain in every attitude row, nonzero INS other-state gain; all bias permission masks with correlated V. | Independently compute u=R^T e_D, projector uu^T and K_eff. Only attitude rows project; disabled bias rows zero separately. Use identical final K_eff in error and Joseph. Check all output coordinates, not only yaw. Detects body-axis row deletion, H projection and mismatched Joseph gain. | Pending |
| TC-E-MAG-TILT-geometric-invariant | [E-MAG-TILT](../../../requirements/core/magnetic-tilt.md#required-behavior), tilt preservation | Unit tilted prior, nonzero parallel-to-u correction induced by a magnetic observation; exact and small-angle Exp branches; zero residual control; M0 control with a fixture that changes tilt. | Independent rotations show R(q_after)^T e_D equals prior body Down in exact arithmetic; native vector difference must meet a reviewed quaternion/tilt bound. Other states and covariance may change. M0 follows unrestricted full-vector reference. Detects wrong rotation axis or a vacuous test with no actual correction. | Pending |
| TC-E-MAG-TILT-failure-selection | [E-MAG-TILT](../../../requirements/core/magnetic-tilt.md#required-behavior), checked projection/build mode | M1 internal projector/projected-gain nonfinite result fixtures under documented helper inputs; reachable public solve/Joseph/injection/reset failures and four aliases. Compile default/on/off and invalid macro values. | Generated projection failures return `non_finite_result`; downstream status and complete rollback remain intact. M0 compiles without projection path and matches unrestricted model. Default is M1; invalid values rejected. Unrealizable malformed-prior public paths remain an explicit formal-fixture gap, not fabricated native success. | Pending |
| TC-E-OBS-bias-forwarding | [E-BIAS-UPDATE](../../../requirements/core/bias-update.md#required-behavior), six INS wrappers | Through all six INS wrappers use omitted/default, all-enabled, all-disabled, each single-disabled and each single-enabled mask; full cross-covariance and correlated within-group V; both reset modes and applicable magnetic modes. | Independently constrained-gain oracle predicts each wrapper's complete result; omitted argument matches all-enabled. Disabled biases retain values but covariance rows/columns remain. These partitions expose omitted/swapped permissions at forwarding sites. Exhaustive 64-mask row semantics belong to `TC-E-BIAS-UPDATE-all-permissions`; all-mask projection interaction belongs to `TC-E-MAG-TILT-projected-gain`. Do not duplicate those obligations six times without a new forwarding branch. | Pending |
| TC-E-OBS-boundary-ownership | [E-OBS](../../../requirements/core/observations.md#required-behavior), model versus wrapper | Valid standalone Jacobian evaluations and equivalent checked-wrapper inputs; local prior perturbation/relinearization; change P cross-covariance while keeping selected measurement rows fixed. | Unchecked Jacobians compute model only and have no promised invalid-input status; checked wrappers use r=z-h at the supplied prior. Full-state updates through cross-covariance are expected. No core gate, lifecycle, frame conversion or timestamp policy is inferred. Independent model/correction oracle detects residual-sign and stale-H errors. | Pending |
| TC-E-INNOV-ratio-decision | [E-INNOV](../../../requirements/core/innovation.md#required-behavior), componentwise formula | Dimensions 1/2/3; unequal positive innovation variances; sigma below and above one; r=0, either sign, exact dyadic boundary ratio=1 and adjacent representable inputs on both sides. Include several individually passing axes whose summed ratio exceeds one. | Independent ordered IEEE ratio calculation gives every diagnostic and aggregate `any(ratio>1)` decision. Equality accepted, sign invariant, no implicit sigma minimum or joint-sum/Mahalanobis gate. Detects V-versus-S confusion when paired with preparation and wrong aggregation. | Native: [test_innovation.cpp](../../../../../tests/core/test_innovation.cpp) |
| TC-E-INNOV-safe-arithmetic | [E-INNOV](../../../requirements/core/innovation.md#required-behavior), input/domain/safe-range | Each input NaN/infinity; sigma/variance zero/negative; subnormal/least-normal variances; squared sigma and denominator just below/at safe minimum; overflow at squared sigma, squared r, denominator and ratio; benign r/ratio underflow. | Nonfinite inputs first, then isolated domain/safe-divisor/result statuses exactly as specified. Independent IEEE expression order classifies each boundary. No general precedence invented for competing domain/safe failures. Large denominator cannot silently accept after overflow; safe underflow need not fail. | Native: [test_innovation.cpp](../../../../../tests/core/test_innovation.cpp) |
| TC-E-INNOV-atomic-alias | [E-INNOV](../../../requirements/core/innovation.md#required-behavior), all axes/publication | All five vector alias partitions: three disjoint; r=variance only; output ratios=r only; output ratios=variance only; all three same. Old flag both values, arbitrary ratio sentinels. First axis outlier with later invalid/overflowing axis. | Success publishes all finite nonnegative ratios and rejection flag, even for rejected observations. Later arithmetic failure rolls back entire diagnostics; outlier cannot short-circuit validation. Exact logical preservation includes signed zero/NaN classification. Detects early publish and alias-dependent rereads. | Native: [test_innovation.cpp](../../../../../tests/core/test_innovation.cpp) |
| TC-E-INNOV-preparation-composition | [E-INNOV](../../../requirements/core/innovation.md#required-behavior), S diagonal; [E-CORRECT](../../../requirements/core/correction.md#required-behavior), preparation | Form C/S for a prior with nonzero HP H^T; gate using diag(S); compare a deliberately different result using V alone as a negative control. Keep P/r/H/V fixed between preparation and independent correction attempt. | Independent S and ratios yield the required diagnostics. Gate success plus rejected=true is not fuse permission, and preparation success cannot bypass correction revalidation. This composition demonstrates correct arguments without creating a new core gate-and-correct API; runtime orchestration is allocated elsewhere. | Native: [test_innovation.cpp](../../../../../tests/core/test_innovation.cpp) |

## Source scope

Paths below are relative to `include/formal_eskf/eskf/`.

| Header / API group | Case allocation |
|---|---|
| `position.hpp`: horizontal/vertical Jacobians and correction wrappers | `TC-E-OBS-POSITION-*`, `TC-E-OBS-bias-forwarding`, shared observation boundaries. |
| `velocity.hpp`: horizontal/3D Jacobians and correction wrappers | `TC-E-OBS-VELOCITY-*`, `TC-E-OBS-bias-forwarding`, shared observation boundaries. |
| `accelerometer.hpp`: both Jacobians and correction overloads | `TC-E-OBS-ACCEL-*`, `TC-E-OBS-bias-forwarding`, shared observation boundaries. |
| `magnetometer.hpp`: both Jacobians/wrappers, preparation/projector/gain helpers | `TC-E-OBS-MAGNETIC-*`, `TC-E-MAG-TILT-*`, `TC-E-OBS-bias-forwarding`. |
| `innovation.hpp`: `InnovationCheck`, `check_innovation` | `TC-E-INNOV-*`. |
| Correction/generated/reset dependencies | [Update allocation](update.md#source-scope); actual sensor selection/forwarding is exercised by these wrapper cases. |
| All remaining ESKF headers | [Propagation allocation](propagation.md#source-scope). The three source tables jointly allocate the complete ESKF header tree. |

## Gaps and exclusions

- General Jacobian finite-difference, numerical tilt preservation, solve and
  full-wrapper error bounds need reviewed domains and tolerance derivations under
  [C-NUMERICAL-ACCEPTANCE](../../../requirements/common/numerical-acceptance.md).
  Exact selector/axis fixtures, exact statuses and rollback remain separable from
  those unresolved numeric partitions. Dense/generated agreement alone cannot
  validate a shared model or supply the missing error bound.
- Public malformed quaternion fixtures and some projection-overflow paths are
  not constructible through the normal quaternion API. Use the scoped evidence
  approach recorded in [propagation](propagation.md#gaps-and-exclusions); do not
  mutate private storage or pretend helper-only tests establish caller reachability.
- Same timestamp/origin, calibration, reference uncertainty, gravity applicability,
  prior/measurement independence and avoiding double-counting are caller premises.
  Core APIs cannot validate those physical facts from the supplied numbers. A
  sequence of independent sensor groups does not model their cross-group noise.
- Gravity alone leaves heading unobserved; a single magnetic vector leaves one
  rotational direction unobserved. The models and gain restriction do not prove
  flight accuracy, observability of every state or safe sensor acceptance.
- Gate coverage is diagnostic arithmetic only. Runtime owns permission to fuse,
  lifecycle/recovery and successful-update accounting. Integration layers are out
  of scope for this test design.

## Review

Pending. Case definitions and profile partitions are planned; no existing test or
prior run is credited as executed coverage.
