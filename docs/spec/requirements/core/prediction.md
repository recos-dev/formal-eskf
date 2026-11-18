# E-PRED: Nominal prediction

**ID:** `E-PRED`

**Status:** Draft

**Applies to:** One INS/AHRS predict_nominal call; binary32/binary64 and the selected attitude mode.

**Parent:** [SW-CORE](../software/core-estimation.md).

## Required behavior

These child IDs refine the mapped `E-PRED` requirement; they are not additional passing requirements. The scope is one call to either INS or AHRS `predict_nominal`, not covariance propagation or repeated-step stability. Deployment tolerances and numerical error bounds remain separate obligations.

### Sources and interpretation

| Source | Required correspondence |
|---|---|
| Cheng and Huang, [SICE 2023](https://doi.org/10.23919/SICE59929.2023.10354209), equations (5), (9), (11) | Body-frame IMU measurements, bias subtraction, nominal kinematics and the selected discrete INS update. |
| Solà, [Quaternion kinematics for the error-state Kalman filter, arXiv:1711.02508v1](https://arxiv.org/pdf/1711.02508v1), equations (97)–(101), (211)–(215), (260a)–(260e) | Quaternion Exp, body-rate integration by right multiplication, and the nominal-state update. |
| [State conventions](state.md) and [scalar/failure semantics](../common/numerical-profile.md) | Project configuration, frames, units, finite-precision and checked-API behavior. These software requirements are not attributed to the papers. |

The SICE prose before (4) describes the opposite frame direction to the one implied by (5), (9) and (11). Use the project's explicit body-to-navigation `q_nb` and `R(q_nb)` convention. The position formula includes a constant-acceleration term; the paper's use of the word Euler must not reduce it to `p_next = p_old + v_old * dt`.

Solà's gravity state is not added to this project: INS takes fixed navigation-frame gravity as a parameter. AHRS is the rotation-only specialization with external gyro-bias calibration. The normalized Euler option in [E-PRED-ATTITUDE](attitude-prediction.md) is a project-selected truncation of (211)–(214) followed by normalization, not Solà's varying-rate first-order integrator in section 4.6.2.

### E-PRED-INPUT: consumed inputs and caller premises

| Configuration | Consumed values, all finite |
|---|---|
| AHRS | Prior `q_nb`, `imu.angular_rate_b`, `dt`, `dt_min`, `dt_max`, `minimum_quaternion_norm`, `quaternion_squared_norm_tolerance`. |
| INS | All AHRS values, prior `p_n`, `v_n`, `b_a`, `b_g`, `imu.specific_force_b`, and `gravity_n`. |

IMU values are body-frame rates, not integrated increments. AHRS receives an already bias-calibrated angular rate; INS subtracts its estimated biases. The caller supplies consistent frames, units, gravity and a sample representative of the same step as `dt`; this API has no timestamps or frame identifiers to validate.

Changing unconsumed fields must not change the result or status: AHRS ignores specific force, and both configurations ignore process-noise parameters. A full prediction step must validate process noise separately.

The physical rotation model requires a unit prior quaternion over the reals. A floating-point `UnitQuaternion` value or successful normalization is not proof of exact unit norm. Finite-coefficient safety checks and the mathematical rotation premise are separate obligations.

Public prediction must check the prior before attitude or translation calculations; it must not rely only on the caller or the type name. After the time/normalization-parameter checks below, require:

~~~text
0 < quaternion_squared_norm_tolerance < 1
q_nb has finite coefficients
norm_squared = squared_norm(q_nb.coefficients())
norm_squared is finite
abs(norm_squared - 1) <= quaternion_squared_norm_tolerance
~~~

Validate in that order: non-finite tolerance returns `non_finite_input`; a tolerance outside `(0,1)` returns `domain_error`; non-finite prior coefficients return `non_finite_input`; overflow of the computed squared norm returns `non_finite_result`; excessive residual returns `invalid_quaternion_norm`. Equality at the residual boundary is accepted. These checks precede the remaining consumed-input checks and prediction calculations.

The tolerance is a dimensionless squared-norm residual, explicitly configured for the selected scalar/backend profile; its zero default is invalid. This predicate uses that profile's floating-point arithmetic, not exact real equality. Acceptance neither proves physical frame correctness nor a bound on all later numerical errors. Validation must not normalize or otherwise modify the old quaternion. `minimum_quaternion_norm` guards normalization candidates only; it is not this tolerance. Internal attitude helpers need not repeat this public residual check.

### E-PRED-TIME: time and threshold validation

The accepted parameter domain is:

~~~text
0 < dt_min <= dt_max
dt_min <= dt <= dt_max
0 < minimum_quaternion_norm <= 1
~~~

All four values must be finite. Time bounds are inclusive, including the case `dt_min == dt_max`. Non-finite parameters return `non_finite_input`; otherwise invalid bounds or threshold return `domain_error`; otherwise a step outside the interval returns `out_of_range`. These parameter checks precede state and IMU processing. Mathematical zero-step identities do not make `dt == 0` a valid public call.

### E-PRED-INS: translation and bias propagation

Apply [E-PRED-INS](translation-prediction.md).

### E-PRED-ATTITUDE: selected attitude propagation

Apply [E-PRED-ATTITUDE](attitude-prediction.md).

### E-PRED-CONSISTENCY: prior-state and limiting behavior

Every right-hand side must use the same prior snapshot, including the attitude used to rotate specific force and both subtracted biases. This is a data-dependency requirement, not a prescribed order of assignments. Attitude calculation must not feed `q_next` back into this step's translation.

In the exact model, zero corrected angular rate preserves attitude; replacing `q_old` by `-q_old` negates the predicted quaternion without changing the physical prediction. INS with `v_old = 0`, `a_n = 0` and zero corrected angular rate preserves its nominal state. Floating-point re-normalization need not preserve quaternion coefficients bit for bit in the zero-rate case.

### E-PRED-SUCCESS: successful output

When input validation and every required checked calculation succeed, return `success` and publish the complete nominal result. All output coefficients must be finite; INS biases must equal their prior values. The quaternion must be produced by the selected checked-normalization path, not silently replaced by identity or an old value.

A C++ proof must establish the selected coefficient computations as well as the status. It must not assume success to avoid rejection or overflow paths. Finite inputs alone do not imply success: intermediate values or norms may overflow, or a computed normalization norm may fail its threshold.

Exact unit norm is a mathematical postcondition. A numerical claim additionally needs the scalar policy's explicit unit-norm and operation-residual bounds; finite output or a status check does not establish those bounds.

### E-PRED-FAILURE: rejection and status propagation

After the parameter checks, require these outcomes when the preceding relevant stages succeed:

| Condition | Required outcome |
|---|---|
| A consumed state, IMU or gravity coefficient is non-finite | `non_finite_input`. |
| Prior squared-norm residual exceeds `quaternion_squared_norm_tolerance` | `invalid_quaternion_norm`, before prediction; no implicit repair. |
| Finite inputs produce a non-finite bias difference, increment, quaternion candidate, computed norm, acceleration, position or velocity | `non_finite_result`; an internally generated invalid value must not be mislabeled as invalid caller input. |
| A required quaternion normalization has a finite computed norm below `minimum_quaternion_norm` | `invalid_quaternion_norm`; equality with the threshold is not rejected for being too small. |
| A checked scalar/quaternion dependency otherwise fails | Propagate its failure; do not continue with its output. |

Never clamp `dt`, repair invalid inputs, or publish a fallback estimate as success. Apart from the ordered parameter and prior checks above, simultaneous independent failures do not require one universal priority; isolated causes must retain their specified status.

### E-PRED-ATOMICITY: output and alias behavior

On any failure, every logical field of the caller's output must remain as it was before the call, even if attitude succeeded before a later translation failure. The initial output is arbitrary, not assumed equal to the prior or to identity.

The output may be the same complete nominal-state object as the prior. With otherwise disjoint input/parameter storage, in-place and separate-output calls must have the same status and, on success, the same result under the same scalar/backend profile. Separate prior state, IMU and parameters must not be modified. This is not a concurrency or object-padding contract.

## Acceptance and verification

Each claim must identify INS/AHRS, binary32/binary64, attitude mode, backend/scalar contracts and its input domain. Historical harness limits such as corrected rates in `[-2,2]`, `dt` in `[2^-10,1]` and threshold `1/8` are not API limits. Full-domain profiles remove these restrictions and must check all prior-tolerance values, including rejection cases. Their complete execution inventory, not older bounded-profile or pre-validation results, is required as evidence. Fixed examples do not cover a symbolic domain. Callee summaries require matching actual-callee evidence and callers that establish their preconditions, including the Exp-to-attitude and attitude-to-public-API boundaries.

Keep these boundaries explicit:

- Consumed-field validation retains AHRS's independence from specific force and both nominal APIs' independence from process noise.
- The public prior check and candidate normalization are different obligations. An accepted floating-point prior is not the exact unit-norm premise used by the real-number model.
- Numerical output residual bounds and admissible deployment ranges are not yet proved. Tests may choose explicit input tolerances, but these are not certified deployment settings or evidence of a closed repeated-step invariant.
