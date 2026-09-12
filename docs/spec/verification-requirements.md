## Summary

Identify verification requirements for the portable numerical core and its supporting operations, not the estimator runtime or platform integration.

Each ID identifies a behavior to verify, not a C++ function or a single theorem. Mathematical definitions and assumptions come from papers and design specifications. Only `E-PRED` is expanded into concrete requirements below; the other entries remain scope definitions.

Evidence mappings belong to the [traceability map](../../formal/agent-review/traceability-map.json); execution results and gaps belong to validated review reports. This list records neither proof status nor completion.

## ESKF core

- `E-STATE` — INS/AHRS state, error and noise dimensions; coordinate ordering, frames, initialization and logical field correspondence.
- `E-PRED` — Nominal-state propagation from the prior state, with applicable bias compensation and the selected paper-based integration and attitude models.
- `E-NOISE` — Discretization of sampled IMU measurement noise and bias driving noise in noise-source coordinates, with the specified units and time scaling.
- `E-PCOV` — Error-state covariance propagation using the selected transition model and injection of process noise into error-state coordinates.
- `E-INJECT` — Injection of estimated error into the nominal state using additive Euclidean and right-multiplicative attitude updates.
- `E-RESET` — Reset of the estimated error mean and transformation of the full covariance into post-injection error coordinates, including cross-covariances.
- `E-CORRECT` — Innovation, Kalman gain, estimated error and Joseph-form covariance update from the same prior estimate, before nominal injection and coordinate reset.
- `E-OBS` — Supported sensor observation models, residual conventions and Jacobians with respect to the local error state.
- `E-STEP` — Composition of nominal/covariance prediction and correction/injection/reset, preserving component preconditions and committing all outputs only when the whole operation succeeds.

### E-PRED: nominal prediction requirements

These child IDs refine the mapped `E-PRED` requirement; they are not additional passing requirements. The scope is one call to either INS or AHRS `try_predict_nominal`, not covariance propagation or repeated-step stability. The policy decisions listed at the end remain open for review.

#### Sources and interpretation

| Source | Required correspondence |
|---|---|
| Cheng and Huang, [SICE 2023](https://doi.org/10.23919/SICE59929.2023.10354209), equations (5), (9), (11) | Body-frame IMU measurements, bias subtraction, nominal kinematics and the selected discrete INS update. |
| Solà, [Quaternion kinematics for the error-state Kalman filter, arXiv:1711.02508v1](https://arxiv.org/pdf/1711.02508v1), equations (97)–(101), (211)–(215), (260a)–(260e) | Quaternion Exp, body-rate integration by right multiplication, and the nominal-state update. |
| [State conventions](eskf-state-conventions.md) and [scalar/failure semantics](scalar-ieee754-and-failure-semantics.md) | Project configuration, frames, units, finite-precision and checked-API behavior. These software requirements are not attributed to the papers. |

The SICE prose before (4) describes the opposite frame direction to the one implied by (5), (9) and (11). Use the project's explicit body-to-navigation `q_nb` and `R(q_nb)` convention. The position formula includes a constant-acceleration term; the paper's use of the word Euler must not reduce it to `p_next = p_old + v_old * dt`.

Solà's gravity state is not added to this project: INS takes fixed navigation-frame gravity as a parameter. AHRS is the rotation-only specialization with external gyro-bias calibration. Normalized Euler below is a project-selected truncation of (211)–(214) followed by normalization, not Solà's varying-rate first-order integrator in section 4.6.2.

#### E-PRED-INPUT: consumed inputs and caller premises

| Configuration | Consumed values, all finite |
|---|---|
| AHRS | Prior `q_nb`, `imu.angular_rate_b`, `dt`, `dt_min`, `dt_max`, `minimum_quaternion_norm`. |
| INS | All AHRS values, prior `p_n`, `v_n`, `b_a`, `b_g`, `imu.specific_force_b`, and `gravity_n`. |

IMU values are body-frame rates, not integrated increments. AHRS receives an already bias-calibrated angular rate; INS subtracts its estimated biases. The caller supplies consistent frames, units, gravity and a sample representative of the same step as `dt`; this API has no timestamps or frame identifiers to validate.

Changing unconsumed fields must not change the result or status: AHRS ignores specific force, and both configurations ignore process-noise parameters. A full prediction step must validate process noise separately.

The physical rotation model requires a unit prior quaternion over the reals. A floating-point `UnitQuaternion` value or successful normalization is not proof of exact unit norm. Finite-coefficient safety checks and the mathematical rotation premise are separate obligations.

#### E-PRED-TIME: time and threshold validation

The accepted parameter domain is:

~~~text
0 < dt_min <= dt_max
dt_min <= dt <= dt_max
0 < minimum_quaternion_norm <= 1
~~~

All four values must be finite. Time bounds are inclusive, including the case `dt_min == dt_max`. Non-finite parameters return `non_finite_input`; otherwise invalid bounds or threshold return `domain_error`; otherwise a step outside the interval returns `out_of_range`. These parameter checks precede state and IMU processing. Mathematical zero-step identities do not make `dt == 0` a valid public call.

#### E-PRED-INS: translation and bias propagation

For the prior INS state, define the exact-real step by SICE (11), equivalently Solà (260a)–(260e):

~~~text
f_b      = imu.specific_force_b - b_a_old
omega_b  = imu.angular_rate_b - b_g_old
a_n      = R(q_nb_old) * f_b + gravity_n
p_next   = p_old + v_old * dt + 0.5 * a_n * dt^2
v_next   = v_old + a_n * dt
b_a_next = b_a_old
b_g_next = b_g_old
~~~

The integration freezes `a_n` over this step. It is not an exact solution of the fully coupled dynamics under arbitrary changing attitude or acceleration. In particular, the position increment must retain the factor `0.5` and use `v_old`, not `v_next`.

#### E-PRED-ATTITUDE: selected attitude propagation

Let `q_old = q_nb_old` and let `omega_b` be the INS bias-corrected rate above, or the supplied AHRS rate, held constant over the step. Use scalar-first coefficients and Hamilton multiplication:

~~~text
theta = omega_b * dt

ESKF_QUAT_APPROX=0 (default):
    q_next = q_old ⊗ Exp_q(theta)

ESKF_QUAT_APPROX=1:
    c      = q_old ⊗ [1, theta_x/2, theta_y/2, theta_z/2]
    q_next = c / norm(c)
~~~

The default exact-real map solves `q_dot = 0.5 * q ⊗ [0, omega_b]` for a constant body rate. The approximation is normalized Euler; it must have the same first-order derivative at `dt = 0`, not be claimed equal to Exp for a finite nonzero step. Both preserve unit norm in the exact model for unit `q_old`.

The C++ Exp path includes its existing small-angle branch and checked normalization of the increment and product. The Euler path normalizes its candidate without first normalizing the increment. Their finite-precision correspondence must be checked separately; the model equations above are not assertions of bitwise equality to real arithmetic.

#### E-PRED-CONSISTENCY: prior-state and limiting behavior

Every right-hand side must use the same prior snapshot, including the attitude used to rotate specific force and both subtracted biases. This is a data-dependency requirement, not a prescribed order of assignments. Attitude calculation must not feed `q_next` back into this step's translation.

In the exact model, zero corrected angular rate preserves attitude; replacing `q_old` by `-q_old` negates the predicted quaternion without changing the physical prediction. INS with `v_old = 0`, `a_n = 0` and zero corrected angular rate preserves its nominal state. Floating-point re-normalization need not preserve quaternion coefficients bit for bit in the zero-rate case.

#### E-PRED-SUCCESS: successful output

When input validation and every required checked calculation succeed, return `success` and publish the complete nominal result. All output coefficients must be finite; INS biases must equal their prior values. The quaternion must be produced by the selected checked-normalization path, not silently replaced by identity or an old value.

A C++ proof must establish the selected coefficient computations as well as the status. It must not assume success to avoid rejection or overflow paths. Finite inputs alone do not imply success: intermediate values or norms may overflow, or a computed normalization norm may fail its threshold.

Exact unit norm is a mathematical postcondition. A numerical claim additionally needs the scalar policy's explicit unit-norm and operation-residual bounds; finite output or a status check does not establish those bounds.

#### E-PRED-FAILURE: rejection and status propagation

After the parameter checks, require these outcomes when the preceding relevant stages succeed:

| Condition | Required outcome |
|---|---|
| A consumed state, IMU or gravity coefficient is non-finite | `non_finite_input`. |
| Finite inputs produce a non-finite bias difference, increment, quaternion candidate, computed norm, acceleration, position or velocity | `non_finite_result`; an internally generated invalid value must not be mislabeled as invalid caller input. |
| A required quaternion normalization has a finite computed norm below `minimum_quaternion_norm` | `invalid_quaternion_norm`; equality with the threshold is not rejected for being too small. |
| A checked scalar/quaternion dependency otherwise fails | Propagate its failure; do not continue with its output. |

Never clamp `dt`, repair invalid inputs, or publish a fallback estimate as success. Apart from the parameter precedence above, simultaneous independent failures do not require one universal priority; isolated causes must retain their specified status.

#### E-PRED-ATOMICITY: output and alias behavior

On any failure, every logical field of the caller's output must remain as it was before the call, even if attitude succeeded before a later translation failure. The initial output is arbitrary, not assumed equal to the prior or to identity.

The output may be the same complete nominal-state object as the prior. With otherwise disjoint input/parameter storage, in-place and separate-output calls must have the same status and, on success, the same result under the same scalar/backend profile. Separate prior state, IMU and parameters must not be modified. This is not a concurrency or object-padding contract.

#### Evidence boundary and review decisions

Each claim must identify INS/AHRS, binary32/binary64, attitude mode, backend/scalar contracts and its input domain. Existing harness limits such as corrected rates in `[-2,2]`, `dt` in `[2^-10,1]` and threshold `1/8` are evidence restrictions, not new API limits. Fixed examples do not cover a symbolic domain. Callee summaries require matching actual-callee evidence and callers that establish their preconditions.

Before claiming complete `E-PRED` coverage, resolve these boundaries with the existing specifications:

- Consumed-field validation specializes the scalar policy's broader wording about all sensor inputs being finite. The proposed behavior retains AHRS's independence from specific force and both nominal APIs' independence from process noise.
- The scalar policy mentions a minimum norm for the input quaternion. Current prediction checks normalization candidates, not a separate prior-norm threshold or prior unit-norm residual. Normalized Euler can increase the candidate norm relative to the prior. Decide whether prior validity remains a caller premise or needs an additional runtime check; do not present candidate validation as an existing prior-validity check.
- Numerical input/output residual bounds and admissible deployment ranges are not yet selected. Keep those claims open; this draft does not choose tolerances or reinterpret bounded harnesses as all-input proofs.

## Quaternion and SO(3)

- `Q-IDENTITY` — Scalar-first identity representation and two-sided quaternion identity behavior.
- `Q-NORMALIZE` — Nonzero quaternion normalization, its exact unit-norm property and checked failure behavior.
- `Q-MUL` — Raw Hamilton multiplication, coefficient ordering and the exact norm-product relationship, without implicit normalization.
- `Q-ROT-COMPOSE` — Normalized rotation composition and agreement with matrix composition in the specified order.
- `Q-INVERSE` — Unit-quaternion inversion by conjugation and its inverse laws.
- `Q-ROT-MATRIX` — Quaternion-to-matrix conversion under the active, column-vector rotation convention.
- `Q-ROTATE` — Forward and inverse vector rotation, inverse recovery and exact length preservation for unit rotations.
- `Q-SIGN` — Equivalent rotations represented by opposite quaternion signs, distinct from coefficient equality.
- `SO3-HAT` — Hat/cross-product correspondence and antisymmetry.
- `SO3-EXP` — Rotation-vector exponential, its zero behavior and correspondence between quaternion and matrix rotations.
- `SO3-LOG` — Principal rotation-vector logarithm, sign selection, the pi boundary and local inverse behavior.
- `SO3-SMALL-ANGLE` — Exact-zero limits and correspondence of selected small-angle Exp/Log approximations to their exact maps.

## Supporting operations

- `F-LINALG` — Fixed-size storage, access and arithmetic under backend-independent linear-algebra contracts.
- `F-SOLVE` — Cholesky factorization and solution of SPD linear systems under declared scalar and failure contracts.
- `F-JR` — SO(3) right Jacobian, its zero behavior and selected approximation under the local right-error convention.
- `F-SCALAR` — Elementary scalar functions, finite/domain checks, status reporting and output preservation under declared scalar profiles.

Before claiming a requirement, define its applicable configurations, input domain and success/failure behavior. Exact mathematical properties, bounded C++ checks and numerical guarantees remain separate claims under the [review criteria](formal-verification-traceability-and-review-criteria.md).
