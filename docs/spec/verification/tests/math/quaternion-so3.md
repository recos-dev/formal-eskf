# Quaternion and SO(3) Tests

**Status:** Draft
**Scope:** The thirteen quaternion/SO(3) requirements, public UnitQuaternion and
SO(3) operations. Mathematical unit premises are distinguished from checked C++
behavior. Estimator injection/reset belongs to the core families.
**Level / purpose:** Unit and primitive-composition regression.
**Profiles:** [Shared profiles](../profiles.md); binary32/64 and each supported
backend. These primitives have no ESKF mode macro.
**Setup:** Construct quaternions through public factories, then use identity,
sign, inverse and products. Exact dyadic unit fixtures include (1,0,0,0), the
three axis half-turns and (1/2,1/2,1/2,1/2). Nonaxis references use independent
high-precision/interval trigonometry. Snapshot all input/output coefficients.
**Regression:** SO(3), scalar/linalg/backend or numerical-profile changes;
include core and runtime consumers of the changed operation.

## Cases

| Case ID | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|
| TC-Q-IDENTITY-VALUE | [Q-IDENTITY](../../../requirements/math/quaternion/identity.md#required-behavior) | Default and identity construction, scalar getters and coefficient copy; left/right multiply a nonidentity dyadic unit quaternion. | Exactly [1,0,0,0]; identity leaves coefficients unchanged. Mutating returned coefficients cannot change the quaternion. Detects scalar-order/storage error. | Pending |
| TC-Q-MUL-HAMILTON | [Q-MUL](../../../requirements/math/quaternion/multiplication.md#required-behavior) | All axis half-turn pairs, noncommuting mixed-axis pair and (1/2,1/2,1/2,1/2); raw product, sign and repeated products. | Independently evaluate four Hamilton coefficients; no implicit normalization/canonical sign in operator*. Detects handedness/order and unwanted repair. General products use error budgets. | Pending |
| TC-Q-INVERSE-CONJUGATE | [Q-INVERSE](../../../requirements/math/quaternion/inverse.md#required-behavior) | Nonidentity dyadic unit, its negative and inverse twice; q q⁻¹ and q⁻¹ q. | Scalar preserved, vector signs reversed, exact identity for exact fixtures; nonaxis numerical recovery separately bounded. | Pending |
| TC-Q-NORMALIZE-COEFFICIENTS | [Q-NORMALIZE](../../../requirements/math/quaternion/normalization.md#required-behavior) | Both from_coefficients overloads on scaled axis/dyadic unit fixtures and mixed signs; normalize a reachable quaternion, disjoint and in-place. | Divide each input by computed norm, preserve coefficient ordering/sign; overload agreement and independent unit residual. No q0 sign canonicalization. | Pending |
| TC-Q-NORMALIZE-DOMAIN | Q-NORMALIZE, threshold/error order | Threshold NaN/±Inf, zero/negative, valid greater than 1; nonfinite coefficients individually; zero vector; norm exactly threshold and neighbours; finite coefficient norm overflow. Combine nonpositive threshold with nonfinite coefficients. | Threshold nonfinite → non_finite_input; nonpositive → domain_error before coefficient use; insufficient norm → invalid_quaternion_norm; nonfinite computed norm → non_finite_result. Equality accepted; threshold>1 not universally rejected. All failures preserve old quaternion. | Pending |
| TC-Q-NORMALIZE-RESULT | Q-NORMALIZE, candidate publication | Controlled norm/division returns unsafe or nonfinite result while calling actual factory; valid scalar and vector overloads. | Linalg unsafe-divisor status mapped to invalid_quaternion_norm; other failure propagated; all four fields published only on success. Actual primitive evidence remains separate. | Pending |
| TC-Q-ROT-COMPOSE-ORDER | [Q-ROT-COMPOSE](../../../requirements/math/quaternion/composition.md#required-behavior) | Compose noncommuting q_a,q_b, rotate nonaxis v; output separate or aliases either operand. Invalid threshold and controlled normalization failure. | q_b acts first, then q_a, and checked composition normalizes the Hamilton product; aliases agree. Failure preserves output/inputs as applicable. Detects order reversal and raw-product substitution. | Pending |
| TC-Q-SIGN-COMPARE | [Q-SIGN](../../../requirements/math/quaternion/sign-equivalence.md#required-behavior) | q, -q, distinct nearby rotations; same_coefficients and same_rotation with zero, below/equal/above coefficient residual, negative/NaN/Inf tolerance. | Coefficient equality differs from rotation equivalence; min of plus/minus maximum coefficient residual decides inclusive tolerance. Invalid tolerance false, no implicit modification. | Pending |
| TC-Q-ROT-MATRIX-COEFFICIENTS | [Q-ROT-MATRIX](../../../requirements/math/quaternion/rotation-matrix.md#required-behavior), Q-SIGN | Identity, axis half-turns, dyadic nonaxis q, both signs and independent general unit rotations. | All nine coefficients match specified formula/reference, q/-q matrices equal within declared arithmetic criterion. Determinant/orthogonality residual are supplementary bounded checks, not consequences of success alone. | Pending |
| TC-Q-ROTATE-ACTION | [Q-ROTATE](../../../requirements/math/quaternion/rotation-action.md#required-behavior) | Forward/inverse action on basis and mixed vectors, q/-q, inverse recovery, length and composition. | Known basis mapping plus independent matrix-vector reference checks R vs Rᵀ and frame/order; norm/recovery relations bounded for general values. Do not calculate expected action with production to_rotation_matrix. | Pending |
| TC-SO3-HAT-COEFFICIENTS | [SO3-HAT](../../../requirements/math/so3/hat.md#required-behavior) | Zero, each basis and mixed signed integer vector; multiply by independent b. | Six signed entries/zero diagonal exact, skew transpose and determinant-based a×b agree. Detects cross convention without circular use of production cross. | Pending |
| TC-SO3-EXP-MAP | [SO3-EXP](../../../requirements/math/so3/exp.md#required-behavior) | Zero, signed axes, mixed-axis vectors below pi, at/above pi and beyond 2pi, on a declared finite angle domain. | Zero identity; half-angle formula followed by checked normalization. Compare both quaternion and independent Rodrigues action, accounting for sign where appropriate. No artificial principal-input restriction. | Pending |
| TC-SO3-EXP-FAILURE | SO3-EXP, checks/status/publication | Nonfinite vector/threshold, nonpositive threshold, squared-angle overflow; controlled negative squared norm/root/trig failures and final normalization failure. Pair input failure with threshold domain failure. | Ordered statuses specified by Exp; output preserved at every stage. Controls test defensive paths, not real squared norms becoming negative. Detects skipped candidate normalization or premature publication. | Pending |
| TC-SO3-SMALL-ANGLE-EXP | [SO3-SMALL-ANGLE](../../../requirements/math/so3/small-angle.md#c-branch-binding) | Construct vectors whose computed theta² is zero, immediately below/equal/above epsilon; mixed-axis small vectors. Passive seam confirms selected path. | Taylor candidate coefficients exactly follow stated polynomial, then normalization; above threshold uses half-angle scalar path. Independent enclosure bounds truncation/rounding/branch jump when budget is approved. | Pending, numerical boundary budget |
| TC-SO3-LOG-MAP | [SO3-LOG](../../../requirements/math/so3/log.md#required-behavior) | Identity/negative identity, interior unit quaternions both signs, near-pi from each side, q0=+0/-0 at pi. | Zero at identity, same principal vector away from cut, norm≤pi up to accepted numerical error. At pi either opposite axis sign is valid; compare represented rotations instead of requiring identical vectors. | Pending |
| TC-SO3-SMALL-ANGLE-LOG | SO3-SMALL-ANGLE, C++ branch binding | Computed qv² zero and below/equal/above epsilon, q0>0; controlled q0² unsafe or scalar/result failures where valid seam permits. | Zero first, Taylor scale 2/q0*(1-r²/(3q0²)) on small branch, root/atan2 above; unsafe divisor status/output preservation. Do not claim this checks a unit-norm residual. | Pending, inaccessible-domain/numerical criteria |
| TC-SO3-LOG-INVERSE | SO3-LOG, local inverse; SO3-EXP | Independently generated phi with norm<pi, mixed axes and near zero; also inputs outside principal domain. | Log(Exp(phi)) agrees on open principal domain with bounded combined error; outside it require equivalent rotation, not phi equality. Known-answer map checks prevent mutually wrong inverse implementations passing alone. | Pending |
| TC-SO3-LOG-FAILURE | SO3-LOG, checked coefficient evaluation | Reachable input plus controlled squared-norm/root/atan2/scale-result failures; malformed quaternion coefficients only through a reviewed source-verification seam. | Finite-input/result checks, propagated scalar statuses, no partial vector publication. Out-of-unit accepted finite paths must not be credited with principal-range proof. | Pending, inaccessible-domain allocation |
| TC-F-JR-MAP | [F-JR](../../../requirements/math/so3/right-jacobian.md#required-behavior) | Zero, signed axes and nonaxis phi, near pi and larger angles within numerical domain. | Zero I; compare to independent closed form and right-perturbation derivative Exp(phi+d)=Exp(phi)Exp(Jr(phi)d)+O(d²). Detects left Jacobian/inverse-Jacobian sign confusion; derivative step sweep required. | Pending |
| TC-F-JR-BRANCH | F-JR, Taylor/half-angle binding | Computed theta² immediately below/equal/above 1/256; equality fixture phi=(1/16,0,0); mixed axes. | Stated A/B polynomial at/below boundary and half-angle expression above; no ESKF macro changes this primitive. Reference enclosure separates truncation and rounding. | Pending, numerical boundary budget |
| TC-F-JR-FAILURE | F-JR, ordered failure/publication | NaN/Inf vector, finite squared-norm overflow; controlled negative norm, root/trig and final matrix nonfinite results; prefill all nine output entries. | Required input/domain/result status and unchanged matrix; success publishes all entries. Detects per-row partial updates. | Pending |

## Source allocation

Paths are relative to `include/formal_eskf/so3/`.

| Source | API group → cases above |
|---|---|
| `unit_quaternion.hpp` | constructors/getters/coefficients/sign/product/inverse → Q-IDENTITY, Q-MUL, Q-INVERSE, Q-NORMALIZE rows. |
| `operations.hpp` | normalize/compose_normalized/comparisons/R/rotate/inverse_rotate/hat → Q-NORMALIZE through SO3-HAT. |
| `rotation_vector.hpp` | exp/log including principal sign and Taylor helpers through public maps → SO3-EXP/LOG/SMALL-ANGLE rows. |
| `right_jacobian.hpp` | right_jacobian and its branch helpers → F-JR rows. |
| `so3.hpp` | Umbrella consumer build with representative public operations; no additional numerical behavior. |

## Gaps and exclusions

General rotation residuals, libm/argument reduction, branch-transition bounds and
repeated-operation drift require domain-specific numerical acceptance. A sampled
unit residual is not an invariant proof. Private nonfinite/nonunit quaternion
states must not be fabricated by representation casts; allocate inaccessible
source-domain paths to reviewed formal/seam evidence and retain the native gap.
Near zero, FTZ/DAZ needs a separate profile. Malformed backend controls do not
establish deployed backend behavior. No principal Log equality is claimed at the
pi sign cut or outside its mathematical domain.

## Review

Pending owner review; all executable allocations remain Pending.
