## Summary

Identify verification requirements for the portable numerical core and its supporting operations, not the estimator runtime or platform integration.

Each ID identifies a behavior to verify, not a C++ function or a single theorem. Mathematical definitions and assumptions come from papers and design specifications. `E-PRED` and `E-STEP` are expanded into concrete requirements below; the other entries remain scope definitions.

Evidence mappings belong to the [traceability map](../../formal/agent-review/traceability-map.json); execution results and gaps belong to validated review reports. This list records neither proof status nor completion.

## ESKF core

- `E-STATE` — INS/AHRS state, error and noise dimensions; coordinate ordering, frames, initialization and logical field correspondence.
- `E-PRED` — Nominal-state propagation from the prior state, with applicable bias compensation and the selected paper-based integration and attitude models.
- `E-NOISE` — Discretization of sampled IMU measurement noise and bias driving noise in noise-source coordinates, with the specified units and time scaling.
- `E-PCOV` — Error-state covariance propagation using the selected transition model and injection of process noise into error-state coordinates.
- `E-INJECT` — Injection of estimated error into the nominal state using additive Euclidean and right-multiplicative attitude updates.
- `E-RESET` — Reset of the estimated error mean and transformation of the full covariance into post-injection error coordinates, including cross-covariances.
- `E-CORRECT` — Innovation, Kalman gain, estimated error and Joseph-form covariance update from the same prior estimate, before nominal injection and coordinate reset.
- `E-BIAS-UPDATE` — Explicit per-axis INS bias-update permissions applied consistently to the correction vector and Joseph covariance.
- `E-INNOV` — Componentwise innovation gate and checked, failure-atomic diagnostics, separate from correction.
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

### E-STEP: composed-operation requirements

These clauses refine `E-STEP`; they are not additional requirement families. They concern sequential calls on valid C++ objects, not concurrency, exception recovery or a new predict-plus-correct API. Numerical validity remains owned by the component and numerical requirements.

- **E-STEP-PRED** — `try_predict` calls nominal prediction first. Only if that succeeds, covariance prediction receives the same **old** nominal state, old covariance, IMU, `dt` and parameters. Publish both component results unchanged only after both succeed; otherwise return the first failure and preserve both outputs.
- **E-STEP-INJECT-RESET** — `try_inject_and_reset` injects the complete supplied error first. Only if that succeeds, covariance reset receives the original covariance and the error **before zeroing**. Publish the nominal result, reset covariance and zero in every error-mean coordinate only after both succeed. Either failure preserves all three outputs and returns that failure.
- **E-STEP-FRAME** — Each output may independently alias its respective complete input object; all other input/output storage is disjoint. Cover all four prediction and eight injection/reset arrangements. Distinct inputs, unused output objects, IMU and parameters are unchanged. Do not publish caller outputs before the last required callee succeeds. Initial outputs and scalar fields are arbitrary, including non-finite values. Preservation concerns values, signed zero and NaN classification, not padding or NaN payloads.
- **E-STEP-CORRECT** — `try_correct` must propagate solve, covariance-finalization and injection/reset failures without publishing either output. Its complete composition must use the component preconditions and result relations established by `E-CORRECT`, `F-SOLVE`, `E-INJECT` and `E-RESET`. Proving the two earlier combined APIs alone does not discharge this clause.

Compositional caller proofs may abstract a callee's result and status, but must check its arguments and call order, use separate scratch outputs, and identify matching input-purity evidence. A summary may even overwrite its scratch output on failure: transaction rollback must not depend on successful or unchanged scratch data. A callee declaration or a proof using an incompatible backend instantiation is not sufficient evidence. INS/AHRS, binary32/binary64 and each applicable prediction/reset mode must be accounted for.

### E-BIAS-UPDATE: INS bias-update control

INS `try_correct` accepts an optional trailing `Ins::BiasUpdate` with three body-axis permissions each for accelerometer and gyroscope biases. Every permission defaults to `true`; omitting the argument retains unrestricted correction. AHRS has no bias states and does not accept this control.

INS horizontal/vertical position, horizontal/3D velocity, magnetometer and accelerometer correction APIs accept the same optional trailing permissions and forward them unchanged. Their observation models, Jacobians and validation remain unchanged.

- Compute the same innovation covariance and unconstrained gain as `E-CORRECT`. After a successful checked solve, set every entry of each disabled bias gain row to zero. Error-state rows 9..11 are accelerometer bias and 12..14 are gyroscope bias; position, velocity and attitude rows are untouched.
- Use the resulting `K_eff` for both `delta_x = K_eff r` and the full Joseph update `(I-K_eff H) P (I-K_eff H)^T + K_eff V K_eff^T`, then perform the existing injection/reset. Keep the same prior P and full correlated V; do not alter H or erase covariance rows/columns.
- A disabled bias receives zero correction, retaining its numerical estimate. Its uncertainty is not set to zero; cross-covariances may change. This is per-call correction control, not a freeze of prediction's bias random walk or a persistent health state.
- Disabling bias updates does not bypass input checks or solve failures. Preserve the existing validation order, statuses, whole-state/covariance alias support and atomic publication on success; all failures preserve both outputs.
- All-enabled permissions must agree with unrestricted correction. Test all 64 permission combinations against an independent constrained-gain/Joseph/injection/reset oracle, including nonzero cross-covariance, correlated measurement noise and both reset modes.

The caller owns the learning policy. Runtime policy wiring, IMU fault detection, recovery timing and adaptive process noise are separate work. Native tests check all 64 permission combinations and whole-state/covariance aliases through each INS sensor wrapper; they are supporting evidence only. Existing `E-CORRECT`, `E-STEP` and `E-OBS` ESBMC profiles exercise unrestricted correction; they do not prove the controlled path. New Lean/ESBMC evidence remains pending.

### E-INNOV: componentwise innovation gate

This pure operation consumes a fixed-size innovation vector `r`, the diagonal of the innovation covariance `S = H P H^T + V`, and an explicit `gate_sigma > 0`. It does not compute S, validate its positive definiteness, apply correction or manage fusion lifecycle. Measurement-noise variances alone are not a substitute for `diag(S)`.

- Compute `test_ratio[i] = (r[i] * r[i]) / ((gate_sigma * gate_sigma) * S[i,i])`. Reject the whole observation if **any** computed ratio is greater than one; equality is accepted. This is not a joint Mahalanobis gate. No implicit minimum sigma, clamping or gate default is applied.
- All inputs must be finite; otherwise return `non_finite_input` before arithmetic. Nonpositive sigma or variance returns `domain_error`. Positive variances, the computed squared sigma and each denominator must be at least the scalar format's smallest positive normal value; smaller values return `zero_or_unsafe_divisor`. Isolated failure causes have these statuses; competing domain/safe-range violations have no universal precedence.
- Non-finite squared sigma, squared innovation, denominator or ratio returns `non_finite_result`. A large denominator must not overflow into a false acceptance. Small innovations or ratios may underflow; decisions use the computed IEEE-754 ratio, not an exact-real boundary guarantee.
- On success, publish every finite nonnegative ratio and the aggregate rejection flag, even when rejected. `success` reports valid diagnostics, not permission to fuse. The caller must require both success and no rejection before treating the gate as passed.
- On failure, preserve the entire diagnostic output. Its vector may alias either complete input vector; otherwise inputs are unchanged. Do not exit early on an outlier and miss invalid later axes. No estimator state or covariance is modified.

Native tests cover binary32/binary64 and dimensions 1, 2 and 3. Lean and ESBMC evidence for this new requirement is pending; existing correction proofs do not cover it. Sensor-specific gate settings and wiring into correction remain caller/runtime work.

## Portable runtime

### R-IMU-DYNAMICS: instantaneous IMU magnitude check

`runtime::check_imu_dynamics` consumes an `ImuSample`, accelerometer/gyroscope bias vectors and two explicit magnitude thresholds. All vectors are in the same body frame; specific force is in m/s^2 and angular rate in rad/s, not integrated increments. Compute `f = specific_force_b - b_a` and `omega = angular_rate_b - b_g`, then `high_dynamics = squared_norm(f) > maximum_specific_force^2 OR squared_norm(omega) > maximum_angular_rate^2`.

- Specific force retains the stationary gravity response: no rotation, gravity subtraction or normalization. The limits are vector magnitudes, not independent component limits. Threshold equality is accepted as not high dynamics. Decisions use computed floating-point squared quantities; no exact-real boundary or backend-independent rounding guarantee is claimed.
- Validate every consumed vector coefficient and both thresholds before calculation; non-finite input returns `non_finite_input`. Thresholds must be positive with finite squares at least the smallest positive normal scalar value; zero defaults, negative or unrepresentable/small squared limits return `domain_error`. Finite inputs causing bias-subtraction or squared-norm overflow return `non_finite_result`. A high reading on one sensor must not hide invalid data on the other.
- Return a fresh result without modifying any input. On success, both squared-norm diagnostics are finite/nonnegative and `high_dynamics` follows the comparison. On every failure it is true and diagnostics are invalid zero placeholders; failure must never be interpreted as a successful motion classification or reuse earlier results.
- The check is stateless. Supply successful `high_dynamics` to `ImuBiasLearning` once per IMU sample at the same fusion horizon. On failure, set both sensor `healthy` conditions false for that update; preserve independent clipping and observability restrictions on success. The existing monitor alone owns continuous-good recovery; this check introduces no second timer or envelope filter. Reset the monitor on dynamics-threshold, calibration or sensor/frame changes.

This upper-magnitude check does not detect free fall, saturation metadata, vibration, sensor faults or physical observability, nor authorize accelerometer measurement fusion. Passing it does not establish that a bias estimate or motion model is correct. Threshold tuning and adapter wiring remain caller/deployment work. Native arithmetic, boundary, failure and detector-to-policy-to-correction tests are supporting evidence; Lean and ESBMC evidence is pending.

### R-IMU-BIAS-LEARNING: bias-learning qualification

`runtime::ImuBiasLearning` converts caller-supplied conditions into `Ins::BiasUpdate`. It owns six independent recovery timers, not sensor detection or ESKF state. Body-axis `permitted` combines the caller's configuration and aiding/observability policy; `healthy` means suitable for learning, including relevant cross-sensor faults. Both default to false. The caller supplies clipping accumulated since the preceding update and a high-dynamics flag.

- An accelerometer axis is eligible only when its sensor is healthy, its permission is true, it is not clipped and high dynamics is false. A gyroscope axis requires its own health, permission and no clipping; high dynamics alone does not inhibit it. Loss of eligibility immediately disables that axis and clears its good interval. No attitude/gravity-axis observability heuristic is inferred here.
- Startup and reset inhibit all axes. An eligible axis starts its interval at the first good sample, not at the preceding bad sample. Enable only after uninterrupted eligible samples span at least `recovery_time_us`; equality is accepted. Other axes keep their own intervals. A new bad sample restarts recovery even after previous qualification.
- Both configured intervals must be positive. Sample timestamps are nonzero, strictly increasing unsigned microseconds on one IMU/fusion timeline. An interval strictly greater than `maximum_sample_interval_us`, or a change to either parameter, clears all recovery history before considering the current sample. A gap exactly at the limit is allowed. Compare elapsed differences only after validating time order; do not add durations to timestamps.
- Invalid configuration returns `domain_error` before time checks; zero/duplicate/backward time returns `out_of_range`. Every failure returns all-disabled permissions and clears the monitor's history. This deliberate fail-closed transition is not numerical-core failure rollback. The next valid sample begins fresh qualification, not an old authorization.
- `status == success` means a valid policy evaluation, not that any axis may learn or a sensor may fuse. Pass the returned permissions to all relevant INS corrections at that same horizon. Call once per new IMU sample even if no correction occurs; never retain an authorization across missing updates. Reset on IMU/source/calibration/body-frame changes. The caller owns validity, freshness, fault detection and consistent event ordering.

The policy does not alter nominal state, covariance, prediction, process noise or measurement admission. It is not a proof of physical observability or IMU health. `R-IMU-DYNAMICS` supplies an optional instantaneous magnitude check; sensor health, observability policy, adaptive process noise and platform wiring remain separate work. Native condition, timing and real-correction integration tests are supporting evidence; Lean and ESBMC evidence is pending.

### R-GNSS-QUALITY: GNSS quality checks and health qualification

Define portable GNSS quality checks and health qualification without platform messages, clocks, geographic projection or EKF state.

- Fix below 3 always fails. The ten configurable checks are satellite count, PDOP, horizontal/vertical position accuracy, speed accuracy, horizontal/vertical drift, horizontal/vertical speed and spoofing. Too few satellites or values strictly above their upper thresholds fail; equality passes. Use named `enabled_checks` and `failures`; report diagnostics even for disabled checks. Platform-specific bit-mask conversion belongs in the adapter, not this API.
- Defaults: all ten checks enabled, 6 satellites, PDOP 2.5, position accuracies 3/5 m, speed accuracy 0.5 m/s, stationary drift/speed limits 0.1/0.2 m/s, health time 10 s. The raw speed limit is 100 m/s. Accuracy inputs are standard deviations, not variances. Scalar parameters must be finite positive normal values; health time must be nonzero. Motion bounds must also permit finite ten-times clamps and the horizontal clamp's squared two-axis norm; squared horizontal-drift and raw-speed thresholds must be normal. Compare-only metadata thresholds have no artificial arithmetic upper bound.
- At rest on the ground, use `dt = clamp(sample_dt, 0.001, 10)` seconds and `alpha = dt / 10`. Filter position-derived velocity, then constrain each filter state to +/- ten times its axis threshold. Constrain each GNSS velocity input to the same bounds before filtering it. Horizontal checks use the two-axis norm; vertical checks use absolute value. The first position delta after construction/reset/invalid data is ignored, but velocity filtering still runs.
- Ground movement resets filters while retaining previous motion-check failure flags. In flight, reset filters and clear those flags. In all modes, raw horizontal speed magnitude and absolute vertical speed above `velocity_limit` force the corresponding flags; their contribution to rejection still follows `enabled_checks`. `drift_valid` is false outside stationary checks; zero metric placeholders are not valid drift measurements.
- `update` returns one result with a primary `decision`: `invalid_call`, `rejected`, `waiting` or `accepted`. All but `invalid_call` consume the sample. A currently passing sample waits until both initial qualification and recovery are satisfied. `accepted` is GNSS quality acceptance only; alignment, innovation gating and fusion remain separate caller conditions. `status` describes caller/numerical errors, not permission to fuse.
- Maintain the last passing/failing **fusion-horizon** times. The first consumed sample establishes the initial fail time even if its checks pass. Recovery requires time since failure strictly greater than `max(1 s, health_time / 10)`. Once a recovered sample also exceeds `health_time`, latch `qualified` until reset. Later rejection blocks the current sample without clearing this historical diagnostic; recovery does not repeat initial qualification.
- Non-finite or negative accuracy data, consumed non-finite vectors and arithmetic overflow produce `rejected` with a failing `status`, reset filters/position continuity and restart the health wait. Such rejection cannot be disabled. Other quality failures have `status == success` but still return `rejected`.
- Invalid configuration, zero/duplicate/backward sample time, a sample beyond `now_us`, or backward `now_us` returns `invalid_call` without changing the monitor. When a position delta is consumed, its `position_reference_time_us` must equal the preceding consumed sample's timestamp; mismatch also returns `invalid_call`. Only `decision` and `status` are meaningful in that result. Time is unsigned microseconds; validate ordering before subtraction. No wall clock is read.
- The adapter provides NED velocity and the current-minus-previous fix displacement with its reference timestamp: north/east projected about the preceding consumed fix, down = old MSL altitude minus new MSL altitude. Advance that reference only when the monitor consumes a sample. The adapter owns geographic validity, PDOP calculation, velocity-validity and spoof-state conversion, receiver identity, freshness and projection continuity. Reset on receiver/configuration changes or lost continuity. Stationary detection must not be inferred from GNSS being checked. Sample gaps do not independently revoke the health latch; freshness and fusion stop/recovery remain separate work.

Native temporal/boundary tests are supporting evidence only. Lean and ESBMC evidence is pending. This monitor does not implement innovation gating, source-specific fusion lifecycle, stationary-motion detection, jamming detection or platform adapter wiring.

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
