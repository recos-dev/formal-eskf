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
- `E-MAG-TILT` — Optional magnetic attitude-gain projection onto navigation Down, with consistent correction and Joseph covariance.
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

### E-OBS-ACCEL: gravity observation with fixed bias preprocessing

INS and AHRS use the gravity observation only; there is no model-selection macro. INS uses a 3×15 local-error Jacobian and subtracts its prior accelerometer bias from raw input. AHRS uses a 3×3 Jacobian with externally bias-calibrated input.

For INS, use raw body-frame specific force, fixed navigation-frame gravity and the prior bias estimate:

\[
z=f_m-\hat b_a,\qquad h=-R(q_{nb})^Tg_n,\qquad r=z-h,
\]
\[
H=\begin{bmatrix}0_{3\times3}&0_{3\times3}&[h]_\times&0_{3\times3}&0_{3\times3}\end{bmatrix}.
\]

The columns are `[delta_p_n, delta_v_n, delta_theta_b, delta_b_a, delta_b_g]`, using the existing right-multiplicative attitude error. Construct this 3×15 Jacobian directly; the nominal 3×16 Jacobian is for manual derivation review only.

- Assume negligible navigation-frame acceleration `dot(v_n)`, not necessarily zero velocity. Keep gravity magnitude and measurement units; do not normalize either vector.
- Subtract the prior accelerometer bias exactly once. Hold this preprocessing fixed in the linearization: both bias Jacobian blocks are zero. This is an explicit approximation, not the joint derivative of `-R^T*g_n+b_a`. Zero bias columns do not freeze learning through cross-covariance; forward the existing optional bias-update permissions unchanged.
- Neither accelerometer API takes angular rate. The complete INS state must still be finite for correction/injection. Reject non-finite bias subtraction, prediction or residual arithmetic as `non_finite_result`; retain existing checked-correction statuses, alias support and atomic output publication.
- Use caller-supplied effective measurement covariance in raw specific-force units. Motion gating, model error and prior/measurement correlation remain caller responsibilities; no gating, differencing or correlated-noise update is introduced here.

### E-BIAS-UPDATE: INS bias-update control

INS `try_correct` accepts an optional trailing `Ins::BiasUpdate` with three body-axis permissions each for accelerometer and gyroscope biases. Every permission defaults to `true`; omitting the argument retains unrestricted correction. AHRS has no bias states and does not accept this control.

INS horizontal/vertical position, horizontal/3D velocity, magnetometer and accelerometer correction APIs accept the same optional trailing permissions and forward them unchanged. Magnetic tilt protection is selected separately by the build-wide `ESKF_MAG_TILT` macro. Their observation models and Jacobians remain unchanged.

- Compute the same innovation covariance and unconstrained gain as `E-CORRECT`. After a successful checked solve, set every entry of each disabled bias gain row to zero. Error-state rows 9..11 are accelerometer bias and 12..14 are gyroscope bias; position, velocity and attitude rows are untouched.
- Use the resulting `K_eff` for both `delta_x = K_eff r` and the full Joseph update `(I-K_eff H) P (I-K_eff H)^T + K_eff V K_eff^T`, then perform the existing injection/reset. Keep the same prior P and full correlated V; do not alter H or erase covariance rows/columns.
- A disabled bias receives zero correction, retaining its numerical estimate. Its uncertainty is not set to zero; cross-covariances may change. This is per-call correction control, not a freeze of prediction's bias random walk or a persistent health state.
- Disabling bias updates does not bypass input checks or solve failures. Preserve the existing validation order, statuses, whole-state/covariance alias support and atomic publication on success; all failures preserve both outputs.
- All-enabled permissions must agree with unrestricted correction. Test all 64 permission combinations against an independent constrained-gain/Joseph/injection/reset oracle, including nonzero cross-covariance, correlated measurement noise and both reset modes.

The caller owns the learning policy. Runtime policy wiring, IMU fault detection, recovery timing and adaptive process noise are separate work. Native tests check all 64 permission combinations and whole-state/covariance aliases through each INS sensor wrapper; they are supporting evidence only. Existing `E-CORRECT`, `E-STEP` and `E-OBS` ESBMC profiles exercise unrestricted correction; they do not prove the controlled path. New Lean/ESBMC evidence remains pending.

### E-MAG-TILT: magnetic tilt protection

AHRS/INS magnetic correction uses compile-time `ESKF_MAG_TILT=1` by default; `0` selects the original unrestricted full-field update. Both use the same full-vector residual, Jacobian and observation covariance; protection is not a scalar heading-observation model. No runtime mode argument or selection branch is permitted. The setting must be consistent across translation units, with values other than 0/1 rejected at compile time. See [the geometric explanation and API](../magnetometer-tilt-protection.md).

- Under the existing unit-prior premise, form `u_b = R(q_nb)^T * [0,0,1]^T` from the prior attitude. After the gain solve, replace only its attitude block by `K_theta' = (u_b * u_b^T) * K_theta`. Use AHRS rows 0..2 and INS rows 6..8. Do not select body X/Y axes or change H.
- Preserve other INS gain rows, except for disabled bias rows under `E-BIAS-UPDATE`. Use this same final gain in the error correction and full Joseph update, followed by existing right-multiplicative injection and covariance reset.
- In exact arithmetic, the injected rotation preserves `R(q_nb)^T * [0,0,1]^T`; only the navigation-vertical rotation component changes. A floating-point tilt-error bound is a separate obligation. Other-state updates, later prediction and magnetic observability are not constrained by this identity.
- When protection is enabled, check the computed vertical axis, projector and projected gain for non-finite results, returning `non_finite_result`. Retain existing input/covariance checks and downstream status propagation. Unit prior and PSD/SPD remain caller premises; no implicit repair is introduced.
- Publish state and covariance together only on success. Preserve both on any failure and support all four independent whole-state/covariance alias arrangements. `ESKF_MAG_TILT=0` must agree with the original full-field behavior and must not compile the projection path.

Native tests in both macro configurations are supporting evidence only. Projection identities and the new constrained-gain C++ path require new Lean/ESBMC evidence; existing unrestricted `E-OBS`/`E-CORRECT` proofs do not discharge this requirement. E-OBS execution explicitly selects `ESKF_MAG_TILT=0`, not the new default.

### R-MAGNETIC-FIELD: instantaneous magnetic-field plausibility

`runtime::check_magnetic_field` consumes a unit `q_nb`, calibrated body-frame `m_b`, trustworthy NED reference `m_n` (both microtesla), explicit `MagneticFieldParameters`, and a caller-supplied `heading_observable` condition. It owns no state, clocks, calibration or WMM lookup. This check is for already aligned magnetic aiding, not initial yaw alignment or proof of sensor health.

- Form `measured_n = R(q_nb) m_b`. For measured/reference NED fields compute strength, horizontal magnitude and Down-positive inclination `atan2(z, sqrt(x*x+y*y))`. Compare absolute strength and inclination differences against their tolerances. Strength in the rotated frame equals body strength under the unit-quaternion premise; finite-precision decisions use the computed quantities.
- Require both horizontal magnitudes to be at least `minimum_horizontal_field`, regardless of heading-check selection or `ESKF_MAG_TILT`. This policy rejects fields with insufficient heading information; it is not a general vertical-field tilt-aiding API.
- Only when `heading_observable` is true and both horizontal fields are sufficient, compute `wrap_pi(atan2(measured_E, measured_N) - atan2(reference_E, reference_N))` and compare its absolute value with `heading_tolerance`. The caller must establish aligned heading with independent aiding and suitable motion/observability; neither magnetometer consistency nor a GNSS fix alone establishes that condition. False skips only this extra check, never the innovation gate. No heading filter, hysteresis or reset is implied.
- All consumed coefficients and all four parameters must be finite. Strength tolerance is nonnegative; both angle tolerances are in `[0, pi]`; minimum horizontal field is positive normal. Zero field vectors and invalid parameters return `domain_error`; non-finite inputs return `non_finite_input`. Default parameters are invalid until explicitly configured. No fallback field, tolerance clamp or implicit normalization is allowed.
- Non-finite rotation or squared-magnitude arithmetic returns `non_finite_result`; total squared magnitude below the smallest positive normal value returns `zero_or_unsafe_divisor`. Propagate checked square-root/atan2 failures. These checks intentionally reject extreme arithmetic domains rather than promise arbitrary-scale geometry. Inputs remain unchanged.
- Return fresh diagnostics only after all required arithmetic succeeds. On success, `diagnostics_valid` is true; named strength/inclination/horizontal-field/heading failure flags determine `accepted`. Upper-threshold equality and minimum-horizontal equality pass. `heading_error` is meaningful only when `heading_checked` is true; otherwise it is a zero placeholder. Any failure returns `accepted=false` and invalid zero diagnostics. A current rejection is `success` with `accepted=false`, not a numerical failure.

Physical disturbance detection, attitude/reference correctness and threshold tuning remain deployment assumptions. Native tests cover geometry, boundaries, frames and invalid arithmetic in binary32/binary64; Lean/ESBMC evidence remains pending.

### R-MAGNETOMETER-FUSION: field checks and gated magnetic correction

`runtime::fuse_magnetometer` supports AHRS/INS and returns `MagnetometerFusionResult` with separate `field` and `fusion` diagnostics. `fusion.decision/status` describe the overall call. It retains the existing core's calibration, time, unit-quaternion, PSD/SPD, noise-independence and whole-object alias premises.

- Check finite nominal fields, then apply `R-MAGNETIC-FIELD` to this prior. A field error returns `failed` with its status; a field rejection returns `rejected` with `success`. Neither runs the innovation gate or correction. Innovation diagnostics remain invalid; rejection does not certify unused covariance or later-stage parameters.
- After field acceptance, form the existing magnetic residual `r=m_b-R(q_nb)^T m_n` and `magnetometer_jacobian(state,m_n)`. Gate using `diag(H P H^T + V)` through the existing same-prior gate stage. Any component ratio above one rejects the entire three-axis group. The gate uses the full observation model, not a projected H or measurement-noise-only variance, in both tilt configurations.
- After gate acceptance, call `try_correct_magnetometer` with identical inputs, full V, normalization threshold and INS bias permissions. Do not call generic correction and bypass `ESKF_MAG_TILT`. The existing projection, constrained-gain Joseph update and injection/reset remain unchanged.
- Only `fusion.decision=fused` publishes state and covariance. Rejection/failure preserves both outputs, including all four independent whole-prior alias arrangements. Field diagnostics remain valid if a later stage fails; innovation diagnostics remain valid after gate success even if correction subsequently fails. Before the relevant stage succeeds, its diagnostics are invalid placeholders. Preserve the downstream failure status.

This is one stateless observation transaction, not full sensor health qualification, fusion lifecycle, persistent-rejection recovery, yaw initialization or adapter wiring. Native tests cover both configurations and scalar formats, full correlated covariance, all 64 INS bias masks, legal aliases and stage failures. Lean/ESBMC evidence remains pending; existing ungated observation/correction proofs do not discharge these two runtime requirements.

#### Replay magnetic observations

The PX4 replay supplies calibrated body-FRD `vehicle_magnetometer` instance 0
observations in microtesla (Gauss multiplied by 100). Do not repeat sensor
calibration or import the external estimator's learned magnetic bias. Freeze
`m_n = R(q_seed) * mean(m_b_seed)` from the same causal 0.2 s alignment window
already used for initialization. This assumes that initial field is suitable;
it is not an independent WMM reference or evidence of magnetic health.

- Optional `magnetometer.csv` and `magnetic_reference.csv` must occur together;
  without both, existing navigation/accelerometer replay remains unchanged.
  The reference row supplies NED field, per-axis noise standard deviation and
  gate sigma. PX4 preparation converts logged `EKF2_MAG_NOISE` to microtesla
  before squaring for `V = sigma^2 I`, and uses logged `EKF2_MAG_GATE`.
  The existing log gives approximately `sigma=5 microtesla`, `V=25 I`, gate 3.
- Replay field guards are explicitly 20 microtesla strength tolerance,
  20 degrees inclination tolerance and 5 microtesla minimum horizontal field.
  The first two use the [PX4 v1.16.2 parameter defaults](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/params_magnetometer.yaml).
  The horizontal guard is a replay choice (one nominal noise sigma for this
  log), not a PX4 rule or validated heading-accuracy bound. No independent
  heading-observability detector is supplied: pass `heading_observable=false`,
  retaining field checks and the innovation gate. Do not retune to match output.
- Select the newest available topic-timestamped magnetic sample at each IMU
  tick, after navigation and any gravity correction, using the current prior.
  Consume older samples from that tick without fusion and count them as superseded.
  Record the alignment offset; no physical sensor-delay compensation is claimed.
  Only lifecycle permission authorizes `runtime::fuse_magnetometer`, preserving
  `ESKF_MAG_TILT` and the current replay bias-learning permissions. Record outcome/status, field
  failures, valid innovation ratios and lifecycle state/timestamps in
  `magnetometer_updates.csv`. A skipped sample is not a gate rejection.
  No-data lifecycle transitions also produce rows (`source_timestamp_us=0`).
  Rejection continues; numerical failure is logged and terminates replay.
- Reject recorded sensor/calibration/frame or magnetic parameter changes that
  invalidate the frozen reference/settings. Missing change metadata remains a
  limitation, not evidence that the source never changed.
- `--compare-mag` runs the same prepared inputs and initialization twice; the
  C++ `--no-mag` path disables only magnetic correction. Plots report yaw/path
  differences, not accuracy. Existing initialization still uses magnetometer
  data in both runs. Missing-input and disabled paths must preserve the prior
  replay's state/navigation results. KITTI supplies no magnetic input.

#### Replay magnetic lifecycle

Replay uses the lifecycle below with 1 s startup health, a 0.5 s maximum
interval and a 7 s active-fusion timeout. Own covariance-based tilt alignment
supplies readiness; yaw and the fixed field reference remain causal seed
assumptions. No external yaw/reset event occurs during replay; magnetic timeout
recovery may perform the heading reset below.
Horizontal aiding means an actual recent horizontal GNSS fusion, not merely a
received GNSS packet. Only a recorded `at_rest=true` enables the rest override;
missing rest metadata is not evidence of motion.

The reference is [PX4 v1.16.2 magnetometer control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp),
with timing from [ekf.h](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf.h)
and [common.h](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/common.h).

| Decision | Implemented rule |
|---|---|
| Start | Continuing conditions pass, stable yaw/reset frame, at least four samples, more than 1 s since the last failed field check, newest sample age strictly below 0.5 s; actual successful fusion activates. |
| Continue | A current good field may fuse without repeating startup health. Field rejection inhibits navigation correction without ending the active session. |
| No data | On a no-sample tick, stop at age **>= 1 s**; preserve field history and allow fresh qualified data to restart. |
| Fusion timeout | Try the current observation first. Only an active session still without success for **> 7 s** takes the timeout branch. |
| Timeout with horizontal aiding and no rest override | Stop, without a permanent fault latch; later qualified observations may retry automatically. |
| Timeout without horizontal aiding or with rest override | Request a heading reset; execute only with current fresh, health-qualified field data and readiness. Record actual reset progress separately from fusion. |

The metadata controller is not a complete EKF2 replacement. PX4 owns magnetic
earth/bias states, magnetic LPF, WMM, declination fusion, heading-consistency and
mode selection and separate latest/delayed timelines. Our supported reset is
heading plus covariance against the existing fixed field, not magnetic-state reset.
Our fixed-field core has no magnetic-only update: when the field is rejected,
navigation correction is inhibited and no fictitious magnetic-state fusion is
reported. Thus a physical disturbance need not produce the same outcomes or
progress timestamps as EKF2. Replay's single timestamp timeline does not model
PX4 buffer-push recency or sensor delay.

Ordinary stops restart automatically. A timeout reset waits for accepted field
health and uses `try_reset_magnetic_heading`; it does not replace the reference
with the observed field. Replay uses an explicit 0.3 rad heading standard
deviation (PX4's default heading noise), independent of vector noise and not
imported from the log. Successful resets and their applied left quaternion deltas
are logged in `magnetometer_resets.csv`; `last_reset_timestamp_us` remains distinct
from actual fusion time. Numerical errors terminate after logging.
Known source/calibration changes still require a new replay/reference; one
fixed CSV session uses adapter identity `Source{1, 0}`, not a hardware ID/count.
Absent change metadata cannot prove source continuity. Summary counts partition
events into fused, gate-rejected, lifecycle-skipped and superseded; heading resets
are a separate action count and may follow a rejected attempt. No-data
state transitions use source timestamp zero. Native checks are not formal proof.

### R-MAGNETOMETER-LIFECYCLE: magnetic health and fusion lifecycle

`runtime::MagnetometerFusion` manages one selected magnetic source without owning
measurements, estimator state/P, clocks or numerical reset operations.
`Conditions::ready` covers continuing prerequisites: tilt/yaw alignment, valid
reference/state/P and permitted aiding policy. `starting_allowed` additionally
excludes a yaw-alignment change or quaternion-reset frame. Horizontal-aiding and
at-rest flags select the timeout action; they must reflect actual caller state.

- Call `update` on every estimator tick. Current time is nonzero and strictly
  increasing. New sample times strictly increase, including across source changes,
  and cannot be future times. `Sample{}` means no new sample and ignores the other
  fields. A source has a nonzero adapter-assigned ID and calibration count.
  `field_accepted` is the actual successful same-sample/same-prior field check,
  not an innovation outcome. The adapter supplies aligned usable samples and
  truthful freshness on one timeline; delayed buffering is outside this API.
- Positive `health_time_us`, `maximum_interval_us` and `fusion_timeout_us`
  remain fixed until full reset. Require `maximum_interval_us <= UINT64_MAX/2`
  so the two-interval no-data bound is representable. Compare elapsed unsigned
  differences after validating order; never add durations to timestamps.
- Count consumed samples while enabled, saturating at four. Check health on new
  enabled/ready samples. A rejected field sets `last_field_failure_time_us=now_us`;
  the first check seeds that timestamp even when good. Startup requires strictly
  more than the health duration since that timestamp, at least four samples,
  current field acceptance, stable starting conditions and age strictly less than
  one maximum interval. Permission is a start attempt, not an active session.
- Every update revokes old permission. Call the actual gated transaction at most
  once per permitted sample, then record its matching timestamp and decision.
  Only `fused` starts a new session and updates `last_fuse_time_us=now_us`;
  an inactive-to-active success also sets `start_time_us`. A rejected startup
  stays inactive, has no invented successful time and may retry later.
- An active session does not repeat startup health/sample-count tests. Current
  field rejection inhibits navigation correction but leaves it active; the next
  good field may fuse immediately. This controller does not emulate EKF2's
  magnetic-only state update. `stop_reason` also describes temporary inhibition,
  so `field_rejected` or `innovation_rejected` can coexist with `active=true`.
- Disable stops activity. On new data, lost readiness stops it; on no-data ticks,
  age **>= twice the maximum interval** stops it. Ordinary stops retain field
  health/count and successful-fusion history; gaps do not fabricate bad field
  checks. Fresh qualified observations can restart without calling `restart()`.
- Apply the unsuccessful-fusion timer only to an active session after the current
  attempt, or when a current field inhibits correction. Measure progress from
  `max(last_fuse_time_us, last_reset_time_us)`. A success at an overdue
  horizon refreshes progress before this test. Elapsed equality is allowed;
  elapsed **> fusion_timeout_us** selects an ordinary `fusion_timeout` stop when
  horizontal aiding is active and the rest flag is false, otherwise a
  `recovery_required` reset request. No-data ticks use their recency branch,
  not a permanent unsuccessful-fusion latch. Inactive startup rejections do not
  acquire an active-session deadline.
- A timeout request grants one `reset_allowed` permission only while enabled,
  ready, without horizontal aiding or at rest, on a stable starting frame with
  a fresh accepted sample satisfying startup count/health. A bad field must
  requalify health; no-data ticks revoke permission. Pending recovery does not
  fabricate a correction. Restored horizontal aiding without rest cancels this
  timeout request into an ordinary stop; the next sample may retry fusion.
- After the actual reset, `record_reset(sample_time, succeeded)` consumes that
  permission. Success reactivates the existing session, clears recovery and
  sets only `last_reset_time_us=now_us`, retaining start/fusion times. There is
  no second correction on that sample. Failure latches `reset_failed` until
  caller-established repair. Wrong/missing permission preserves metadata.
- `failed` feedback requests numerical repair immediately. Source-ID/calibration
  changes clear old-source history/count and request reinitialization, invalidating
  old feedback. These requests persist until explicit caller-established safe
  re-entry; a source change replaces an earlier cause. No state/P or yaw reset
  is implied. A known external reference/frame change requires the same caller
  coordination, not silent continuation.
- `restart()` acknowledges safe re-entry after any needed estimator repair. It
  clears session/health/count metadata, retains time/source/configuration
  watermarks and requires startup qualification again. Never use it to pretend
  a requested estimator reset ran. `reset()` clears everything for an explicitly
  reinitialized timeline/configuration.
- Invalid/changed durations return `domain_error` before time checks; invalid
  current/sample time returns `out_of_range`; zero new-source ID returns
  `domain_error`. Invalid updates stop activity/permission but do not consume
  time/configuration/source or rewrite health/progress history. Invalid feedback
  enum returns `domain_error`; wrong/missing permission returns `out_of_range`.
  Invalid feedback preserves all metadata. Pending reset causes remain visible
  independently of disable/readiness display.

Native tests cover strict health/freshness boundaries, sample count, active versus
startup behavior, data outages, both timeout actions, retry, real AHRS/INS gated
correction, source changes, failure atomicity and unsigned-time extremes.
Lean/ESBMC evidence remains pending. Physical health, covariance consistency,
deployed reset notifications are not proved here.

### R-MAGNETOMETER-RESET: fixed-field heading and covariance reset

`runtime::try_reset_magnetic_heading` supports AHRS and INS independently of the
lifecycle. Given a calibrated body field `m_b`, trusted fixed NED `m_n` and an
aligned prior tilt, rotate only about navigation Down to align the horizontal
direction of `R_before*m_b` with `m_n`. Fields are in microtesla. Wrap the heading
difference to `[-pi, pi]`, form `q_delta=Exp([0,0,delta_heading])`, and normalize
`q_after=q_delta*q_before`. Return this applied left rotation, retaining all
other nominal fields. This does not require an Euler-angle parameterization.

The covariance operation is defined in navigation attitude-error coordinates,
then expressed in our local right-error convention. With `e_D=[0,0,1]^T`:

```text
C = I - e_D e_D^T
A = R_after^T C R_before
u = R_after^T e_D
P_theta_theta' = A P_theta_theta A^T + max(yaw_variance, 0.01^2) u u^T
P_theta_other' = A P_theta_other
```

Retain the non-attitude P block exactly and mirror the cross block. Symmetrize
the computed attitude block. `yaw_variance` is rad²; zero receives the explicit
floor. The transform removes navigation yaw correlations while retaining
navigation tilt/cross terms; simply clearing body-Z rows is wrong at general tilt.

Inputs must be finite, prior quaternion unit and P symmetric PSD. Check exact
symmetry, nonnegative diagonals/variance, normal positive horizontal-field
threshold and `0 < minimum_quaternion_norm <= 1`. Check both horizontal field
magnitudes against that threshold and reject non-finite intermediate arithmetic.
A negative computed attitude variance returns `not_positive_definite`.
PSD/prior norm, physical field validity, alignment, freshness and authorization
remain caller premises. Outputs publish together only on success. State/P may
independently alias their entire priors; other storage, including delta, is disjoint.

The reference is PX4 v1.16.2
[heading reset](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp#L564)
and [yaw covariance reset](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/yaw_fusion.cpp#L118).
PX4's global left-error definition is explicit in
[its derivation](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/python/ekf_derivation/derivation.py#L152).
Our reset uses a currently qualified sample; PX4 uses filtered magnetic data and
also resets estimated magnetic states. Native AHRS/INS float/double and replay
tests cover this narrower operation; Lean/ESBMC and floating-point bounds are pending.

### E-INNOV: componentwise innovation gate

This pure operation consumes a fixed-size innovation vector `r`, the diagonal of the innovation covariance `S = H P H^T + V`, and an explicit `gate_sigma > 0`. It does not compute S, validate its positive definiteness, apply correction or manage fusion lifecycle. Measurement-noise variances alone are not a substitute for `diag(S)`.

- Compute `test_ratio[i] = (r[i] * r[i]) / ((gate_sigma * gate_sigma) * S[i,i])`. Reject the whole observation if **any** computed ratio is greater than one; equality is accepted. This is not a joint Mahalanobis gate. No implicit minimum sigma, clamping or gate default is applied.
- All inputs must be finite; otherwise return `non_finite_input` before arithmetic. Nonpositive sigma or variance returns `domain_error`. Positive variances, the computed squared sigma and each denominator must be at least the scalar format's smallest positive normal value; smaller values return `zero_or_unsafe_divisor`. Isolated failure causes have these statuses; competing domain/safe-range violations have no universal precedence.
- Non-finite squared sigma, squared innovation, denominator or ratio returns `non_finite_result`. A large denominator must not overflow into a false acceptance. Small innovations or ratios may underflow; decisions use the computed IEEE-754 ratio, not an exact-real boundary guarantee.
- On success, publish every finite nonnegative ratio and the aggregate rejection flag, even when rejected. `success` reports valid diagnostics, not permission to fuse. The caller must require both success and no rejection before treating the gate as passed.
- On failure, preserve the entire diagnostic output. Its vector may alias either complete input vector; otherwise inputs are unchanged. Do not exit early on an outlier and miss invalid later axes. No estimator state or covariance is modified.

Native tests cover binary32/binary64 and dimensions 1, 2 and 3. Lean and ESBMC evidence for this new requirement is pending; existing correction proofs do not cover it. `R-MEASUREMENT-FUSION` composes the gate with correction; sensor-specific settings and deployment wiring remain caller/runtime work.

## Portable runtime

### R-GNSS-RESET: checked GNSS position and velocity reset

These caller-invoked INS operations replace navigation estimates; they are not
Kalman corrections or `E-RESET`'s post-injection change of error coordinates.
The GNSS-specific variance policy belongs to `runtime/gnss_reset.hpp`, not to
the generic numerical correction. Inputs are already aligned to the prior's
local NED frame, body/IMU origin and time; quality, alignment and reset authority
remain caller responsibilities. No clock, lifecycle or automatic retry is owned.

Behavior reference: PX4 v1.16.2, revision
`54f0455ffcd755534539a7cf33a09a20bf71d29d`:
[GNSS reset dispatch](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp),
[position reset](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp),
[altitude reset](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/position_fusion.cpp),
[GNSS height control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gnss_height_control.cpp),
and [velocity reset](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/velocity_fusion.cpp).

| Operation | Nominal state | Covariance |
|---|---|---|
| `try_reset_gnss_horizontal_position` | Set `p_N,p_E` to the supplied 2-vector; leave `p_D` and all other state fields unchanged | Clear rows/columns 0 and 1, then set both diagonals to `max(variance_N + variance_E, 0.01^2)` in m^2 |
| `try_reset_gnss_vertical_position` | Set only `p_D` to the supplied scalar; leave velocity, horizontal position, attitude and biases unchanged | Clear off-diagonal entries in row/column 2; set `P(2,2) = max(variance_D, 0.01^2)` in m^2; preserve every other entry |
| `try_reset_gnss_vertical_velocity` | Set only `v_D` to the supplied scalar in m/s; preserve all other nominal fields | Clear off-diagonal entries in rows/columns 2 and 5; retain `P(2,2)`; set `P(5,5) = max(variance_D, 0.01^2)` in m^2/s^2; preserve every other entry |
| `try_reset_gnss_velocity` | Set all three components of `v_n`; leave position, attitude and biases unchanged | Clear off-diagonal entries in rows/columns 0..5. Preserve position diagonals 0..2; set velocity diagonals 3..5 to `max(variance_i, 0.01^2)` in m^2/s^2 |

The horizontal variance sum follows `resetHorizontalPositionToGnss` calling
`resetLatLonTo`; it is not the per-axis policy of PX4's other position-reset
overload. Velocity reset also decorrelates position, even though position values
and variances do not change. Untouched covariance entries are copied exactly.
The input variances describe diagonal observation noise, not standard
deviations or a full correlated measurement covariance. Noise preprocessing,
including any vertical-velocity scaling, happens before this call.
The Down-only velocity reset follows `resetVerticalVelocityTo`: decorrelate
height as well as vertical velocity, while retaining height value/variance and
both horizontal velocity values/variances. It reports `new v_D - old v_D`.

Vertical position is already converted to the existing local NED origin. For
an altitude-relative Down coordinate with a fixed, common datum,
`p_D = h_origin - h_GNSS`, so `p_D_new - p_D_old` agrees with PX4's
`-(altitude_new - altitude_old)`: climbing gives a negative Down delta.
An ECEF-to-NED adapter must instead supply its projected Down coordinate
consistently; it is not generally identical to altitude difference over distance.
This API does not accept absolute altitude or initialize an origin.
Its covariance rule follows `resetAltitudeTo`, which
does not change velocity. PX4's velocity controller can separately reset `v_D`
when height recovery is required; that policy is not part of this operation.
For GNSS-reference height reset, PX4 uses the prepared GNSS measurement variance,
not the height-bias variance added to its ordinary correction diagnostics. This
API neither estimates nor changes sensor-height biases, terrain or output history.

- All nominal coefficients, P, observation and variances must be finite; otherwise
  return `non_finite_input` before arithmetic. P must be exactly symmetric with
  nonnegative diagonal, and input variances nonnegative; violations return
  `domain_error`. Zero/tiny nonnegative variances receive the explicit floor.
  Symmetric PSD P and a unit prior quaternion remain caller premises; the checks
  are not a PSD test, attitude repair or arbitrary-corruption recovery.
- Compute reset delta as new minus old, in NED meters or m/s. A non-finite delta
  or horizontal variance sum from finite inputs returns `non_finite_result`.
  Check all results before publishing state, covariance or delta.
- On success publish all three outputs. On failure preserve all three, including
  distinct sentinel outputs. Complete state and covariance outputs may each
  independently alias their respective prior. All other input/output storage is
  disjoint. No quaternion normalization, injection, solve or allocation occurs.
- Unlike PX4's internal reset helpers, this checked boundary rejects negative
  or non-finite variances instead of clamping negatives or using NaN to retain
  covariance. The finite valid-input reset behavior above is the aligned scope.
- Return a delta per successful call, not a fusion success, clock update or
  counter increment. The caller must separately acknowledge completed recovery
  and aggregate/publish reset notifications. `R-GNSS-FUSION` supplies
  reset-on-start/timeout orchestration separately. These operations do not perform
  origin changes, output-predictor adjustment, height-reference/bias management,
  automatic height-failure decisions or yaw reset.

Native tests cover both scalar formats, per-entry state/P behavior, floors,
Down-delta sign, retained velocity/covariance, valid aliases, invalid inputs,
both delta-overflow signs and failure atomicity. Lean/ESBMC
evidence remains pending; existing `E-RESET` proofs do not cover these operations.

### R-GNSS-FUSION: GNSS fusion lifecycle

`runtime::GnssFusion` owns lifecycle metadata for horizontal position, vertical
position and velocity, not estimator state/P, quality qualification or clocks.
This profile assumes GNSS is the sole horizontal and height source; use horizontal
OR 3D velocity per sample. Alternate aiding and reset execution remain separate.

- Caller supplies each group's `enabled`, `ready` and `start_without_reset`.
  Readiness includes initialized frame/state and the group's required attitude
  alignment. Starting without reset requires explicit caller-established safe
  initialization; quality acceptance alone cannot establish this. Otherwise,
  request `initialization_required`, with no fusion permission.
- Update on every estimator tick, including no-data ticks. Nonzero current time
  increases; distinct new samples may share one tick after previous feedback.
  Sample timestamps strictly increase and cannot be future times. `Sample{}`
  means no new receiver sample. Unavailable quality metadata may accompany a
  received sample but cannot authorize fusion or refresh quality health.
  Available quality must be an actual accepted/waiting/rejected result.
- Three positive durations remain fixed until full reset:
  `horizontal_timeout_us`, `height_data_timeout_us`,
  `height_fusion_timeout_us`. Horizontal duration is at most UINT64_MAX/2.
  Compare elapsed differences only after validating time order. No overflowing
  timestamp-plus-duration arithmetic.
- Receipt freshness expires at elapsed **>=** the corresponding data duration.
  Height stops independently at its data deadline. Losing horizontal receipt
  while a horizontal group is active stops all GNSS groups. These are ordinary
  stops, not permanent recovery latches.
- Accepted quality permits a new sample. Waiting/rejected/unavailable quality
  never permits correction. Waiting represents currently passing checks during
  qualification/recovery and refreshes last quality-pass time; unavailable
  metadata does not. On new non-accepted data, elapsed **> horizontal timeout**
  since the last quality pass stops all groups when horizontal aiding was active.
- Starting an authorized session records `start_time_us`, not a fictitious
  successful fuse. Each update revokes old permissions. Record each permitted
  group's actual matching `FusionDecision` once before the next update.
  Only `fused` changes `last_fuse_time_us`; rejection leaves it unchanged.
  Numerical failure requests repair for that group immediately.
- With accepted fresh data and an active horizontal session, decide horizontal
  reset **before either horizontal group attempts fusion**: both position and
  velocity progress older than one horizontal timeout, OR enabled position
  progress older than twice that timeout. Thus one healthy horizontal group
  can sustain the other temporarily. Request recovery for active, enabled,
  ready horizontal groups, without changing state/P. Progress uses the later
  of actual last fusion, successful reset and caller-authorized session start;
  a never-started group has no progress. A horizontal timeout request retains
  its active session while waiting for reset; it does not restart quality health.
- Each group's `initialization_required`/`fusion_timeout`/`bad_vertical_acceleration` requests may grant
  `reset_allowed` only on a new, accepted, fresh, enabled and ready sample.
  Reset and correction permissions are mutually exclusive. Each update, even
  an invalid one, revokes old reset permission. `record_reset(group, sample, reset_succeeded)`
  consumes one matching permission after the checked numerical reset returns.
  Success clears recovery, activates the group and updates `last_reset_time_us`,
  but not `last_fuse_time_us`. Only inactive-to-active reset changes session start.
  No ordinary correction is permitted for the same sample after reset.
  Failure stops the group with persistent `reset_failed`, changes no progress
  timestamp and cannot auto-retry. `fusion_failed` also never grants automatic
  reset permission. Stale/missing/duplicate feedback returns `out_of_range`.
- Height uses its own progress deadline, evaluated **after** its current
  permitted attempt. Success at an overdue tick refreshes it; rejection beyond
  the deadline grants reset permission for that same accepted sample because
  this profile has no alternate height reference. The acceleration criterion
  below can also request reset after a successful height correction. Retain the
  active session;
  successful reset renews progress without changing the last correction time.
  A numerical failure never grants this permission. Equality does not expire
  fusion deadlines. Height feedback does not alter horizontal groups or quality.
- `Conditions::has_vertical_velocity` explicitly declares that the velocity group
  includes a qualified Down observation; horizontal-only callers leave it false.
  For an already active velocity session doing ordinary fusion, snapshot the
  height deadline at `update`, before this sample's height operation. Height
  must have an established session and permit either correction or reset on
  this fresh accepted sample. If its progress is strictly beyond the height
  timeout, or the preceding acceleration-health criterion below is expired,
  velocity feedback of `fused` OR `rejected` grants one Down-only reset.
  Numerical failure does not. Startup and full 3D velocity resets already cover
  Down and must not request another reset.
- Execute/acknowledge Down-only reset with `record_vertical_velocity_reset`
  before this sample's height correction/reset. Out-of-order height feedback
  returns `out_of_range` without consuming permission. Even a later successful
  height correction does not cancel the earlier velocity reset decision.
  Success updates only `last_vertical_velocity_reset_time_us`, not height or
  horizontal progress, the velocity group's full-reset/fusion timestamps, or
  quality health. Failure latches velocity `reset_failed`; no automatic retry.
  Wrong-sample/duplicate feedback returns `out_of_range`. Every update (including
  invalid updates), or explicit velocity/height repair, revokes pending permission.
  Restarting the velocity group also clears its Down-reset history. While
  permission is outstanding, `vertical_velocity_reset_reason` identifies
  fusion timeout or bad vertical acceleration; consumption/revocation clears it.
- Ordinary stops can start again after accepted quality and caller-established
  initialization. Losing the last active horizontal group emits one
  `restart_quality` notification; call `GnssQuality::restart_health()` before
  processing the next sample. This clears health timestamps, preserving initial
  qualification, filters and time/reference watermarks. The next check starts
  a new recovery wait; it does not reuse the outage as healthy time.
- Recovery requests persist across good quality and enable/readiness toggles
  until successful permitted reset feedback or explicit repair acknowledgment.
  `restart(group)` acknowledges actual caller-established repair and clears
  only that group's lifecycle metadata, not state/P, quality or time watermarks.
  A new sample is still required. Full `reset()` clears the controller; source,
  frame, configuration or timeline changes also require quality reset and
  re-established initialization.
- Invalid durations/changes return domain_error; invalid time order returns
  out_of_range; invalid available quality returns domain_error. Failed updates
  revoke permissions but preserve history/time/configuration. Invalid feedback
  or restart preserves metadata. The controller cannot verify whether an
  external correction/reset actually ran.

Native tests cover startup conditions, both data deadlines, strict fusion/quality
boundaries, coupled horizontal progress, height reset/attempt ordering, safe re-entry,
unavailable quality, invalid calls, same-tick samples and unsigned-time extremes.
Lean/ESBMC evidence remains pending.

#### Continuous bad vertical acceleration

`VerticalAccelerationTimes` carries the monitor's update time and last-good time,
not its 3 s held fault flag. `Conditions::vertical_acceleration` supplies the
verdict available before velocity control. Height feedback additionally passes
this horizon's times to `record_fusion`; other groups ignore that argument.

- Nonzero update time means an evaluated snapshot. Known snapshots must satisfy
  `last_good_time_us <= update_time_us <= now_us`; height feedback requires
  `update_time_us == now_us`. Future/reversed times, or an older height snapshot,
  return `out_of_range`. Failed updates revoke permissions without consuming
  history; invalid feedback preserves permission and metadata for a corrected call.
- Zero update time means unavailable; ignore its last-good field and disable
  only the acceleration criterion. Fusion-timeout recovery remains available.
  For a known snapshot, zero last-good means never healthy and is immediately
  expired, matching the reference's zero-timestamp rule. Do not manufacture a
  healthy timestamp on startup, reset or missing data.
- With known health, request recovery if last-good is zero OR
  `now_us - last_good_time_us > 500000`. Equality does not expire. The fixed
  `acceleration_reset_delay_us` matches the reference's internal constant.
  Ordered subtraction avoids overflow. The caller must provide the appropriate
  monitor snapshot and clear availability across gaps/source changes; numerical
  timestamp validation cannot establish physical sensor continuity.
- For established, permitted 3D velocity, retain this criterion at `update`.
  Actual fused/rejected velocity feedback can then authorize Down-only reset
  before height processing. Source quality/readiness, established height
  availability and the existing single-use/reset-order restrictions still apply.
  Startup/full velocity reset already covers Down and grants no duplicate reset.
- After height correction, combine its current acceleration criterion with the
  remaining fusion timeout. Successful correction clears the timeout but does
  not erase continuous bad-acceleration evidence. Thus a sample can legitimately
  record both successful height fusion and successful height reset. A numerical
  correction failure still cannot grant automatic reset. If both criteria hold,
  report `bad_vertical_acceleration` as the reason.
- Reset success changes estimator/progress through existing checked operations;
  it does not change last-good acceleration time or start a cooldown. Continuing
  bad health can require another reset on the next permitted sample. Conversely,
  a new healthy verdict stops the acceleration criterion even while the separate
  3 s fault remains held. Pending reset requests still require qualified data;
  numerical reset failure remains terminal until caller repair.

Reference: PX4 v1.16.2
[height reset predicate](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L48),
[velocity control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp#L132)
and [height control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gnss_height_control.cpp#L103).
This profile remains GNSS-only with an established height reference. Native
transition/replay evidence covers the new criterion; Lean/ESBMC evidence is pending.

#### Replay GNSS lifecycle

Behavior reference: pinned [horizontal GNSS controller](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp)
and [height controller](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gnss_height_control.cpp).

| Rule | Replay profile |
|---|---|
| Horizontal missing data / poor quality | 7 s; data expires at equality, quality strictly beyond |
| Height missing data | 1 s, expires at equality |
| Horizontal unsuccessful fusion | Both groups >7 s, or enabled position >14 s; before correction |
| Height unsuccessful fusion | >5 s after current height attempt |
| Horizontal stop | Restart quality health timing, retain initial qualification |
| Horizontal startup / timeout reset | Execute velocity then horizontal-position reset, instead of same-sample correction |
| Height startup / re-entry | Reset local Down position and its covariance on the accepted sample; no correction on that sample |
| Height timeout reset | After rejected correction beyond 5 s, reset Down using the same accepted observation |
| Height-failure vertical velocity reset | After normal 3D velocity correction (fused or rejected), reset only `v_D` when preceding height progress is >5 s old; execute before horizontal-position/height operations |
| Numerical correction / reset failure | Log and terminate; no automatic recovery |

Enable lifecycle with the quality input pair, not for legacy/KITTI inputs.
Invalid-velocity messages do not refresh receiver receipt; missing required
quality metadata advances receipt only. Run no-data updates between events.
Preserve group-independent gate/correction decisions, velocity/position/height
order. Only successful horizontal position/velocity correction or reset refreshes
horizontal-aiding timestamps; height correction/reset and Down-only velocity reset
never do. Log every received
lifecycle sample and no-data transitions in `result/gnss_lifecycle.csv`.
Quality skips, lifecycle skips, innovation rejections and resets are distinct outcomes.
Record per-axis reset reason, status and new-minus-old delta in `gnss_resets.csv`;
failed resets have invalid, empty deltas. Keep successful reset counts separate
from correction and rejection counts. A height-recovery event can contain both
a correction (fused or rejected) and a successful reset; these counters count operations,
not mutually exclusive events. Reset success refreshes progress without
claiming an innovation passed or restarting quality qualification.
Log Down-only resets as `vertical_velocity` in `gnss_resets.csv`, with their own
`vertical_velocity_resets` summary count. A normal velocity correction and this
reset may both occur on one sample. Preserve the actual correction outcome.
The Down observation variance is already prepared as `(1.5 * speed_std)^2`;
the reset API must not apply that scaling again.

Replay passes the preceding valid health times into GNSS lifecycle update and
current pre-height health times into height fusion feedback. Missing IMU/clipping
history supplies unavailable times. Reset logs distinguish
`bad_vertical_acceleration` from `fusion_timeout`; summary counts acceleration
height/Down-velocity resets separately and records the 500000 us delay.

The replay's existing seed establishes its initial frame and assumed attitude
alignment; it is not an alignment detector. All three groups require an actual
reset on their first accepted sample and on re-entry. Height-only data loss
leaves horizontal sessions running; return of qualified data resets height.
A complete GNSS outage stops all groups and restarts quality health timing;
reset/re-entry waits for renewed acceptance instead of treating receipt as recovery.

Velocity, Down-only velocity, horizontal-position and height operations are
independent checked transactions, in that order; a later failure does not roll
back earlier completed operations. Log
the completed/failed operations and terminate without a success summary. Numerical
correction failure is not converted into an automatic reset with the same input.

This is a lifecycle integration, not full EKF2 equivalence: one replay clock
uses source sample recency instead of separate arrival/delayed horizons; no
alternate sources, GNSS heading/GSF emergency yaw or height-bias estimator is
implemented. Continuous-acceleration recovery follows the criterion above;
bounded innovation exceptions are specified in `R-GNSS-VERTICAL-RECOVERY`. Existing seed,
sequential gates and numerical models are retained.

### R-MEASUREMENT-FUSION: gated correction of one observation group

`runtime::fuse_measurement` supports INS/AHRS and accepts `r = z - h(state)`, `H`, full measurement covariance `V`, an explicit sigma threshold and the existing normalization threshold. The caller supplies one prior, zero error mean, valid frames/time, unit quaternion, symmetric PSD `P` and symmetric SPD `V`, as for `try_correct`. INS additionally accepts the same optional per-axis bias permissions.

- Check finite nominal fields and sigma, then reuse correction's matrix/domain validation. Compute `PHt = P * transpose(H)`, then `S = H * PHt + V`, with the same finite checks and symmetry cleanup as correction. Gate with `r` and `diag(S)`, never `diag(V)`. Rejection is componentwise, not a joint NIS test; gate errors retain `E-INNOV` statuses.
- Return a fresh `FusionResult` with `decision = fused`, `rejected` or `failed`. Only `fused` publishes both state and covariance. `rejected` has `status = success`; any numerical/input failure has `decision = failed` and its failing status. Neither rejection nor failure modifies either output. Each complete output may independently alias its respective prior; all other input/output storage is disjoint.
- Only after the gate succeeds, publish `innovation`, `innovation_variance` and `test_ratio` with `diagnostics_valid = true`, including rejected observations. These diagnostics describe the call's prior and remain valid if correction later fails. Before gate success, diagnostics are invalid zero placeholders; never reuse a prior call's diagnostics.
- If any ratio exceeds one, reject this whole observation group without correction, LLT or injection. Otherwise call the existing correction with the identical state, `P`, `r`, `H`, `V`, normalization threshold and bias permissions. It currently recomputes the covariance products; its equations, validation and ungated API are unchanged. A gate pass does not certify full covariance validity, conditioning or prior quaternion norm. Rejection does not certify estimator health.

The named runtime APIs form their residual from the supplied state and reuse the existing core Jacobians:

| API | One rejection group |
|---|---|
| `fuse_horizontal_position` | NED north/east position |
| `fuse_vertical_position` | NED down position, not altitude or HAGL |
| `fuse_horizontal_velocity` | NED north/east velocity |
| `fuse_velocity` | All three NED velocity components |

These models preserve the corresponding core API's units, full within-group noise correlations and bias permissions. Non-finite observations return `non_finite_input`; residual overflow from finite observations/state returns `non_finite_result`. Use horizontal OR 3D velocity for a given sample, not both.

Each call is independent: a rejected or failed group neither undoes previous fusion nor automatically rejects later groups. After a successful group, form the next residual/Jacobian and gate using the newly committed state/covariance; after rejection/failure retain the prior. No fixed sensor order or whole-GNSS transaction is imposed. Separate groups require uncorrelated measurement noise across groups, not zero state cross-covariance; use a joint model if cross-group noise correlations must be retained. Alignment, GNSS quality, source lifecycle, persistent-rejection recovery, clocks and last-fusion timestamps remain caller/runtime work.

Native tests cover INS/AHRS, binary32/binary64, legal aliases, grouped rejection, input/arithmetic and downstream failures, bias permissions and a cross-coupled sequential-fusion case. They do not establish flight suitability. Lean and ESBMC evidence for this composition is pending; existing ungated correction proofs do not cover it.

#### Replay GNSS gating

PX4 preparation supplies one `gnss_parameters.csv` row from logged
`EKF2_GPS_P_GATE`, `EKF2_GPS_V_GATE`, `EKF2_GPS_P_NOISE`, `EKF2_GPS_V_NOISE`
and `EKF2_NOAID_NOISE`. Gates are clamped to at least one sigma at the adapter
boundary, not in the core. Require finite settings, positive noise values and
safe finite squared variances. Reject recorded setting/lever-arm changes during
the replay; require equal configured GNSS/IMU offsets because compensation is
not implemented. Missing required parameters are errors, not guessed defaults.

The noise profile follows the ordinary GNSS path of
[PX4 v1.16.2](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp)
under this demo's sole horizontal/height aiding assumptions:

- Horizontal position: square `max(min(max(eph, position_noise),
  position_noise_limit), 0.01)` for both axes.
- Height: square `max(epv, 1.5 * position_noise)`, without a noise cap or a
  height-bias estimate/variance. GNSS is the fixed sole height reference.
- Velocity: square `max(speed_accuracy, velocity_noise, 0.01)` for N/E;
  use 1.5 times that standard deviation for Down. All covariances are diagonal.
- Initialize position/velocity covariance with these same noise rules;
  retain existing attitude/bias initial uncertainties. Receiver accuracy axes
  are treated as the fixed navigation axes, a local-flight approximation.

Consume each navigation event once at the following IMU tick. Fuse 3D velocity,
horizontal position, then height through the existing runtime APIs. Each group
forms its gate and correction from the latest committed prior. This differs
from PX4's precomputed position/velocity diagnostics and scalar update schedule;
neither identical decisions nor numerical EKF2 equivalence is claimed.
Rejection preserves that group's prior and does not block other groups. Only
actual horizontal-position or 3D-velocity correction/reset refreshes horizontal-aiding
time; height operations, Down-only velocity reset, receipt, rejection and failure
never do so.

`gnss_updates.csv` records each group's outcome/status and per-axis innovation,
S diagonal, test ratio, V diagonal and diagnostic validity. Numerical fusion
failure is logged before stopping. Invalid prepared input stops replay as an
input error. Event counts are not successful-correction counts; report accepted
and rejected groups separately. A fully rejected event still counts as consumed.
Quality qualification is described below; the lifecycle above controls permission
and GNSS reset execution, including height and Down-only velocity recovery.
`R-GNSS-VERTICAL-RECOVERY` supplies the bounded vertical-innovation exceptions.
Delay compensation and changes to magnetic/gravity scheduling are not included.
Without the parameter file, retain legacy/KITTI ungated noise and fusion order.
Replay checks are integration evidence, not new Lean/ESBMC proofs or accuracy claims.

### R-GNSS-VERTICAL-RECOVERY: bounded vertical innovations during IMU faults

`runtime/gnss_measurement.hpp` adds source-specific INS velocity and NED Down
position fusion. Generic gates and ungated correction APIs are unchanged. The
caller must first establish source quality/lifecycle permission, frame/time,
unit quaternion, symmetric PSD P and SPD measurement noise, as for ordinary
fusion. A fault never grants source permission or requests a reset.

| Observation | Additional condition to release a rejected gate |
|---|---|
| 3D GNSS velocity | Bad vertical acceleration, Down ratio > 1, both N/E ratios strictly < 1, receiver speed accuracy strictly below its configured maximum |
| GNSS Down position | Bad vertical acceleration and rejected height gate |

Only the vertical innovation is constrained to `[-gate_sigma * sqrt(S_D),
+gate_sigma * sqrt(S_D)]`. Preserve its sign, horizontal residuals, H, V and
bias-update permissions; call the existing checked correction with that selected
residual. Do not re-gate the rounded boundary value. Ordinary gate equality is
accepted, but horizontal equality prevents the velocity exception. Receiver
accuracy is its reported standard deviation in m/s, not the noise-floor-adjusted
variance; its strict exception bound applies even when the general quality
speed-accuracy check is disabled. Both accuracy values must be finite, accuracy
nonnegative and maximum positive, regardless of the fault flag.

`GnssMeasurementResult` retains raw innovation, S and test ratio in `fusion`, and
separately exposes `fusion_innovation` and `innovation_limited`. Raw ratio > 1
can therefore accompany successful limited fusion. Selected innovation is valid
only with `diagnostics_valid`; only `decision == fused` means it was applied.
`check_gnss_velocity` returns `GnssInnovationCheck` without correction, for an
active source's quality-rejected sample. It reports status, diagnostic validity
and gate rejection; it cannot report successful fusion. Gate/numerical errors retain their existing
statuses; invalid accuracy returns non_finite_input/domain_error. Rejection or
any failure, including LLT/normalization after limiting, preserves both outputs.
Independent whole-prior state/P aliases are supported; other storage is disjoint.

Replay enables velocity recovery only with a quality/lifecycle profile, using
its existing maximum-speed-accuracy setting. It consumes the preceding valid
health verdict for velocity, then evaluates current velocity/preceding height
health before attempting height. Both health caches use selected innovations;
quality-rejected velocity updates evidence without correction. Missing clipping
or a broken IMU interval clears health, so no stale fault releases a gate.
Multiple due events retain the existing sequential estimator updates and the
single-monitor-transition batching rule in `R-VERTICAL-ACCELERATION`.

`gnss_updates.csv` adds the selected innovation and group-level limit flag;
summary counts limited successful velocity/height fusions separately. Rejected
quality/lifecycle conditions still skip correction and do not refresh progress.
Horizontal position has no exception. Existing timeouts/reset behavior is
separate from the gate exception; the independent 500 ms last-good-acceleration
reset is specified in `R-GNSS-FUSION`.

Behavior reference: PX4 v1.16.2
[velocity exception](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp#L209)
and [height exception](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/position_fusion.cpp#L36).
Alignment covers predicates, limiting and fault-consumption order. Our grouped
Joseph correction and latest-prior diagnostics differ from PX4's scalar update
schedule; replay cadence/batching and missing-data safeguards also differ.
Native backend and replay tests are evidence for these contracts, not flight
accuracy or full EKF2 equivalence. Lean/ESBMC evidence remains pending.

### R-ACCELEROMETER-MAGNITUDE: filtered gravity-band diagnostic

`runtime::AccelerometerMagnitude<Linalg>` filters body-frame specific force in
m/s^2 before estimator-bias subtraction. The caller supplies
`delta_vel / delta_vel_dt`, not an integrated increment. It owns only one filtered vector and
its initialization flag; no timestamps, estimator state or covariance.

- The first valid sample seeds the vector directly. Later samples use
  `filtered += alpha * (sample - filtered)`, then compute `squared_norm(filtered)`.
  Filter the vector, not its magnitude or a peak envelope.
- Keep alpha fixed at `0.010 / (0.090 + 0.010)`, evaluated in the selected scalar
  format. Do not recompute it from each sample's dt. The nominal 90 ms time
  constant assumes 10 ms updates; actual cadence belongs to the caller. This
  component does not resample, enforce that cadence or promise 90 ms at 250 Hz.
- With `g = 9.80665 m/s^2`, report `within_gravity_band` only when the computed
  squared norm is strictly between `(0.9*g)^2` and `(1.1*g)^2`. Equality fails.
  A finite out-of-band sample still updates filter history successfully.
- Non-finite input returns `non_finite_input`; a non-finite filtered vector or
  squared norm returns `non_finite_result`. Failure preserves initialization and
  filter history, with a fresh false band flag and invalid zero diagnostic.
  Successful zero input is not an error. `reset()` clears history and requires
  the next valid sample to seed again, including after source/frame/calibration
  changes. The filtered-vector accessor is meaningful only after initialization.
- This result is not fusion permission, sensor health or evidence of rest.
  Persistent horizontal acceleration can pass; body rotation can reduce the
  filtered magnitude. No bias subtraction, normalization, correction, motion
  policy, clipping logic, at-rest override or innovation gate is added here.

The valid-input recurrence, initialization, fixed coefficient and strict band
follow PX4 revision `e1c28e27550114e85ddb8760ab3355eaa7a89d8a`:
[coefficient settings](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/ekf.h#L650),
[initial nominal interval](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/estimator_interface.h#L358),
[sample updates](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/ekf.cpp#L162),
[magnitude check](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/aid_sources/gravity/gravity_fusion.cpp#L50).
Checked failure behavior follows this project's scalar contract. This is not
alignment with the older v1.16.2 gravity-fusion peak envelope, a claim of complete
PX4 fusion equivalence, or a backend-independent bitwise equivalence claim.

`tests/test_accelerometer_magnitude.cpp` checks both scalar formats, recurrence,
startup/reset, strict band boundaries, vector-before-magnitude behavior and
failure preservation. These are native tests; Lean and ESBMC evidence is pending.

### R-ACCELEROMETER-CONDITIONS: gravity-fusion prerequisites

`runtime::check_accelerometer_fusion_conditions<Linalg>` consumes caller-supplied
enable, tilt-alignment, horizontal-aiding, rest and per-axis clipping flags,
together with the actual current `AccelerometerMagnitude::update` result.
All inputs describe the same sample/fusion horizon. Availability of a GNSS fix
is not the same as active horizontal aiding. The caller owns those facts; this
function detects neither rest nor alignment and owns no history.

- On a successful magnitude evaluation, `active = enabled AND (within_gravity_band
  OR at_rest) AND NOT horizontal_aiding AND tilt_aligned`.
- `sample_eligible = active AND NOT any_axis_clipped`. Clipping blocks this
  sample without clearing the otherwise active mode. Rest bypasses only the
  band check, not disable, missing alignment, horizontal aiding or clipping.
- A failed magnitude evaluation propagates its status and returns both flags
  false, even at rest. This is the project's checked-input behavior, not a claim
  that PX4 handles invalid arithmetic identically. Do not substitute a previous
  successful result after the current update fails.
- The function is stateless and leaves every input unchanged. Neither flag
  means fusion succeeded, nor establishes a physically valid gravity observation.
  `sample_eligible` is a pre-innovation result; no innovation gate, correction,
  state/P update, timer, sensor selection or recovery logic is introduced.

The separation of mode activation and sample clipping follows
[PX4 e1c28e2 gravity fusion](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/aid_sources/gravity/gravity_fusion.cpp#L56).
Its additional innovation rejection remains a separate deferred task here.
The same predicate supports INS and AHRS; any configuration without horizontal
aiding supplies false for that flag. Native tests enumerate all 256 boolean
combinations with all declared statuses, and exercise real magnitude-result
consumption in both scalar formats. Lean and ESBMC evidence is pending.

#### Replay condition sources (host demo)

`examples/replay_conditions.hpp` supplies conditions for the replay's existing
single navigation source. It does not change correction equations or implement
the full GNSS qualification/reset controller. `fusion_conditions.csv` records
raw-tick conditions; interval observations and correction outcomes are recorded
separately as specified under `R-ACCELEROMETER-FUSION` below.

- Derive tilt uncertainty from this estimator, never a logged EKF status:
  `P_theta_n = R_nb * P_theta_b * R_nb^T`, using INS covariance rows/columns 6..8.
  Latch alignment when `P_theta_n(0,0)+P_theta_n(1,1) < (3*pi/180)^2`.
  Evaluate tilt on the predicted prior before that tick's navigation corrections.
  A new replay resets the latch; subsequent variance growth does not clear it.
  A constructed quaternion alone is not alignment. Unit attitude and PSD
  covariance remain caller premises; non-finite/negative computed tilt variance
  is a replay error, not permission to fuse.
- Horizontal aiding starts only after replay horizontal correction succeeds,
  remains active between navigation events and expires when elapsed time since
  the last successful event reaches 7 seconds. A resumed successful event
  reactivates it. Initialization is not a fusion event. This is limited to the
  existing fail-fast replay: every accepted navigation event fuses position and
  velocity or ends the run. Its successful-event timer is not a substitute for
  independent source lifecycle, quality qualification or recovery in deployment.
- PX4 preparation takes explicit `vehicle_land_detected.at_rest` at or before
  each IMU timestamp, with age strictly below 3 seconds. Absent/stale data is
  unknown (`-1`) and disables only the rest override; `landed`, constant speed
  and a gravity-sized acceleration are not substitutes. Unlike the delayed
  system-flag buffer, this undelayed demo checks freshness on each IMU tick.
  The external detector's flag is not independent physical ground truth.
- Decode `sensor_combined.accelerometer_clipping` bits X/Y/Z from that sample.
  Missing clipping stays unknown (`-1`), not zero. No complete fusion conditions
  are returned until clipping is known. Optional `imu_flags.csv` must match
  every IMU timestamp exactly, including the seed; malformed/missing/extra rows
  are errors. An absent sidecar leaves flags unknown, preserving legacy/KITTI
  input compatibility without inventing metadata.

Behavior references at PX4 `e1c28e2`:
[tilt latch](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/control.cpp#L72),
[NED tilt variance](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/ekf_helper.cpp#L932),
[active horizontal sources](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/estimator_interface.cpp#L675),
[GNSS missing-data stop](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp#L115),
[7-second limit](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/common.h#L512),
[rest metadata](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF2.cpp#L2849),
[clipping metadata](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF2.cpp#L737).
The local-error covariance transformation and missing-metadata refusal are
project integration behavior, not copied estimator status. This is native-tested
condition preparation, not proof of physical alignment or full PX4 equivalence.

### R-ACCELEROMETER-FUSION: conditional gravity correction

`runtime::fuse_accelerometer` composes `R-ACCELEROMETER-CONDITIONS` with the
unchanged `E-OBS-ACCEL` correction for AHRS and INS. No innovation gate is used.

- Supply the current body-frame specific force in m/s^2, not the LPF vector or
  delta-velocity. INS subtracts its prior bias exactly once inside the core;
  AHRS consumes externally bias-calibrated input. Preserve gravity magnitude,
  the existing Jacobian and caller-supplied effective V in m^2/s^4. Forward
  INS per-axis bias-update permissions unchanged; omission retains all enabled.
- The caller updates its magnitude filter once for each sample, independently
  of fusion enablement, and passes the actual result for that sample with
  conditions and prior state at the same fusion horizon. The caller owns
  timestamps, cadence and filter reset on source/calibration/frame changes.
  This stateless composition neither updates nor rolls back filter history.
- Evaluate conditions first. A failed magnitude result returns `failed` with
  its status. Unmet conditions return `rejected` with `Status::success` without
  calling correction or certifying its unused inputs. Otherwise call the
  checked correction: propagate failure status, and report `fused` only on
  success. Return the condition check separately; eligibility can remain true
  even if correction fails. No innovation diagnostics or test ratio is implied.
- Publish both state and covariance only on `fused`. Rejection or failure
  preserves both outputs. Each output may independently alias its respective
  complete prior; all other storage is disjoint. Separate inputs are unchanged.
  There is no retry, persistent mode, timer, source selection or reset policy.

Native tests cover real magnitude results, current-versus-filtered measurement
use, INS bias permissions, both configurations/scalar formats, all four alias
arrangements, rejection and numerical failures. Lean and ESBMC evidence is
pending. The composition does not establish a valid physical gravity observation,
noise independence, tuned V, full external-estimator equivalence or flight safety.

#### Replay sampling and tuning

| Setting | Selected replay baseline |
|---|---|
| Gravity magnitude | `g = 9.80665 m/s^2` |
| Effective per-axis observation standard deviation | `sigma = g` |
| Observation covariance | `V = g^2 * I_3 = 96.1703842225 * I_3 m^2/s^4` |
| Nominal observation/LPF interval | `10,000 us`; actual intervals must be reported |

PX4 `e1c28e2` uses normalized observations, defaults
[`EKF2_GRAV_NOISE`](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/params_gravity.yaml)
to `1.0`, and squares it for observation variance. Multiplying that variance by
`g^2` matches the local tangential information scale near gravity-sized specific
force, not the complete normalized nonlinear update. This is an effective
pseudo-observation weight, not measured hardware noise or validated flight tuning.
It is separate from prediction's `ACCEL_VAR`; do not multiply V by `dt` or `dt^2`.
The MATLAB reference's enabled `accel_correct1` uses normalized observations and
`V_accel = 0.7 * I_3`, corresponding to `67.31926895575 * I_3` on this same scale;
its cross-product preprocessing is not part of the current model.

The PX4 [nominal EKF period](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/module.yaml#L15)
is 10 ms. Its [IMU downsampler](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/imu_down_sampler/imu_down_sampler.cpp)
accumulates rotation-aware increments over whole input samples; it does not just
select the latest raw sample every 10 ms. The selected target is not a promise of
exactly 100 Hz or an instruction to fuse each roughly 250 Hz raw input. Fixed LPF
alpha remains `0.1`; its effective time constant depends on actual update intervals.

`examples/replay_accelerometer.hpp` prepares non-overlapping observations without
changing prediction/navigation timing or adding a delayed fusion buffer:

- For each real sample, form `dtheta = omega_m * dt`, `dv = f_m * dt` and
  `D = R(Exp(dtheta))^T`. Accumulate in the latest body frame as
  `dv_sum_new = D * dv_sum_old + (dv + D * dv)/2`. This is an endpoint rotation
  approximation, not an exact continuous-time/sculling integral. No estimator
  bias is subtracted here; INS correction subtracts its prior bias once.
- Track mean input period `T` starting at 5 ms, using `T += 0.1*(dt-T)`.
  At each interval start set `N = max(1, round(10 ms/T))`, target `N*T` and
  minimum `max((N-1)*T, T/2)`. Emit once the sample count reaches N and duration
  exceeds the minimum, or duration exceeds the target. All decisions use the
  actual integration periods; no interpolated, repeated or overlapping samples.
  Timing rules follow the referenced downsampler, not bitwise PX4 equivalence.
- On emission, divide accumulated dv by the actual interval duration and update
  `AccelerometerMagnitude` exactly once, even when fusion is unavailable/rejected.
  Correction consumes that current interval average, not the LPF vector. Partial
  final intervals are not flushed as shorter observations. Non-finite input,
  invalid periods/masks or arithmetic failure preserves sampler/LPF history and
  ends this fail-fast replay, without publishing an observation.
- OR all constituent clipping masks; any unknown makes interval clipping unknown.
  Rest and alignment/aiding conditions use the emission tick. Gaps discard partial
  increments and LPF history; resume with real samples after the gap. Prediction
  holds are never accumulated as measurements. A new replay also resets history.
- After that tick's navigation corrections, call `runtime::fuse_accelerometer`
  only when interval clipping is known; its conditions decide whether to correct.
  This uses the agreed V and the current replay bias-learning permissions. A failed
  correction ends replay. Record each observation, actual duration, aggregated
  clipping, LPF diagnostic and outcome (`fused`, `rejected`, `metadata_unavailable`)
  in `accelerometer_updates.csv`; report counts and actual interval statistics.

Missing clipping still blocks correction, unknown rest disables only its override,
and innovation gating remains deferred. The existing legacy ULog and KITTI inputs
lack clipping and therefore do not exercise successful accel correction. These
starting settings and native tests are not calibrated flight parameters or new
Lean/ESBMC evidence. Prediction/measurement reuse correlation, interval-averaging
model error and full PX4 estimator equivalence remain outside the claim.

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

### R-REPLAY-BIAS: replay bias-learning inhibition

`ReplayBiasLearning` evaluates the prior biases and unit attitude once before
prediction, then supplies the same six `Ins::BiasUpdate` permissions to every
GNSS, gravity and magnetic correction at that horizon. It uses decaying peaks,
not `ImuBiasLearning`'s separate continuous-good timers. Learning inhibition
does not control prediction noise; `R-BIAS-PROCESS-NOISE` supplies that independent
policy. Core equations and bias-preserving resets remain unchanged.

- For synchronized accel/gyro integration periods `dt > 0`, compute
  `decay = 1 - clamp(dt / tau, 0, 1)` and independently update each envelope as
  `peak = max(norm(measurement - prior_bias), decay * previous_peak)`.
  Specific force retains gravity. Initial peaks are zero. High dynamics means
  either peak strictly exceeds its configured limit; equality permits learning.
  No startup wait or additional clipping-recovery timer is imposed.
- Gyro permissions follow the gyro-bias enable switch. Accel permissions require
  its enable switch, no high dynamics, no bad-vertical-acceleration fault, and
  no clipping on that axis. Rest overrides only the observability restriction;
  otherwise fake height inhibits all axes, and fake position permits only axes
  with `abs(R_nb(2,axis)) > 0.966`. These are actual mode/fault inputs, never
  substitutes inferred from GNSS age or sample availability.
- Validate finite inputs, positive finite limits/tau/dt and clipping bits 0..7.
  Invalid input or non-finite arithmetic returns disabled permissions and leaves
  envelope history unchanged; replay terminates on failure. A new session starts
  fresh. Missing IMU intervals do not erase peaks or supply held measurements;
  only real samples decay the envelope using their logged integration period.
- Preparation exports logged `EKF2_ABL_ACCLIM`, `EKF2_ABL_GYRLIM`, `EKF2_ABL_TAU`
  and the two enable flags in `bias_parameters.csv`. New logs use
  `EKF2_IMU_CTRL` bits 0/1; legacy `EKF2_AID_MASK` bit 2 disables accel bias,
  with gyro bias enabled. Parameter changes within the session are rejected.
  CSV inputs without this file use explicit 25 m/s², 3 rad/s, 0.5 s defaults
  with both bias estimators enabled. The 0.5 s value is a decay time constant.
- Preserve the three-column `imu_flags.csv`. In the replay input adapter only,
  missing accel clipping assumes clear **for bias learning**, with an explicit
  diagnostic/count. Gravity fusion still sees unknown clipping and is blocked.
  Unknown rest does not supply a rest override. Gyro clipping is not an input to
  this inhibition rule. Reject recorded IMU selection/calibration/frame changes;
  absent metadata cannot prove continuity or physical sensor health.
- The replay implements no fake-position/height modes, so their flags are false.
  `R-VERTICAL-ACCELERATION` supplies the preceding valid IMU horizon's fault to
  bias inhibition. A fault disables all accel axes; gyro permissions remain
  independent. Unknown clipping or an IMU gap invalidates that verdict before
  learning is evaluated. Unavailable health adds no fault inhibition under this
  replay compatibility profile; this is not evidence of physical sensor health.
  `bias_learning.csv` records metadata, assumptions, peaks, permissions and the
  health verdict actually consumed; counts describe IMU ticks, not corrections.

Behavioral reference: PX4 v1.16.2
[IMU bias inhibition](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L971),
[learning limits](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/params_accel_bias.yaml),
and [legacy parameter semantics](https://docs.px4.io/v1.13/en/advanced_config/parameter_reference#EKF2_AID_MASK).
This aligns the inhibition rule under supplied conditions, not all EKF2 behavior:
replay uses its own IMU horizons and precision. Its variance-dependent bias noise
selection, fault-dependent IMU noise and bias-value limits are specified below.
GNSS overrides/resets and fake-aiding modes remain pending. Native tests support this integration;
Lean/ESBMC evidence remains pending.

### R-IMU-PROCESS-NOISE: fault-dependent sampled IMU variance

`runtime::try_select_imu_process_noise(configured_noise, conditions,
fault_variance, output)` selects body-axis accelerometer measurement variances
before INS prediction. Conditions contain the preceding vertical-health fault
and known clipping flags for the current IMU interval.

- Replace an axis's `specific_force_variance` with `fault_variance` when the
  vertical fault is set or that body axis is clipped. Otherwise retain its
  configured variance. A vertical fault therefore selects all three axes.
  This is replacement, not `max(configured, fault)`; it can lower an unusually
  large configured value. Preserve gyro variance and both bias-driving vectors.
- Inputs are variances in m²/s⁴, already squared. Validate the fault variance
  first, then all twelve configured noise coefficients, including replaced or
  unselected values. Non-finite values return `non_finite_input`; negative values
  return `domain_error`. Zero is valid. No arithmetic is required on the noise
  values. Failure preserves output; success permits a whole-noise input/output
  alias and otherwise preserves input. No state, P, timer or permission is changed.
- Start every step from original configured noise. Clearing clipping restores
  that axis immediately unless the held vertical fault still selects all axes.
  The caller owns metadata availability and fault timing. Correction's bias
  permissions are independent: disabled learning does not disable this selection.
- Apply `R-BIAS-PROCESS-NOISE` **after** this selection so its accel bias variance
  threshold uses the selected measurement variance. Its driving intensity retains
  `intensity * dt` scaling. Prediction retains `E-NOISE`/`E-PCOV`: sampled IMU
  variance contributes with `dt²`, and body-axis noise maps through `R(q)`.
  No rotation or time scaling is duplicated in this selector.

Behavioral reference: PX4 v1.16.2 [covariance prediction](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/covariance.cpp#L113)
and [bad-accelerometer noise constant](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/common.h#L83).
Despite its reference name, `BADACC_BIAS_PNOISE = 4.9 m/s²` is a sampled-acceleration
standard deviation here, not bias random-walk intensity. Replay sets
`BAD_ACCEL_STD = 4.9` and passes its square (approximately 24.01 m²/s⁴); nominal
accel/gyro variances remain 1 m²/s⁴ and 0.0004 rad²/s². Alignment covers the
selection rule, constant and ordering, not all covariance equations, precision,
IMU sampling rates or calibration of a particular sensor.

Replay uses the preceding valid vertical verdict and current clipping before
prediction. With unknown clipping it retains configured measurement noise,
records unavailability and does not feed assumed-clear flags to this selector.
Synthetic gap holds likewise use configured noise without reusing previous or
future clipping. These are explicit replay fallbacks, not EKF2 behavior or
proof of healthy data. Known clipping still selects noise without a GNSS health
profile; unavailable vertical-health evidence remains separately reported.
`bias_learning.csv` records the three selected accel variances for actual IMU
ticks. Summary records the fault standard deviation, per-axis override counts
and unavailable predictions (including gap holds). Override counts describe
selection, even if configured and replacement variances happen to be equal.

Native tests cover all fault/clipping choices, restoration, invalid inputs and
aliases, selected bias thresholds, actual rotated covariance prediction, and
replay ordering/metadata behavior. Lean/ESBMC evidence and deployment-adapter
wiring remain pending. No physical accuracy or noise-calibration claim is made.

### R-BIAS-PROCESS-NOISE: variance-dependent bias driving noise

`runtime::try_inhibit_bias_process_noise(P, step_noise, output_noise)`
selects INS process noise for one prediction. Its rule follows PX4 v1.16.2
[covariance prediction](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/covariance.cpp#L145):

| Body axis i | Keep the configured intensity only when |
|---|---|
| Accelerometer bias | `P(9+i,9+i) < specific_force_variance(i)` |
| Gyroscope bias | `P(12+i,12+i) < angular_rate_variance(i)` |

- Otherwise set only that bias intensity to zero. Preserve both IMU measurement
  variance vectors and every other bias axis. No new threshold parameter is
  introduced. Zero variance/intensity is valid; equality inhibits the increment.
- The INS bias transition rows are identity and sampled measurement noise has
  no bias rows. Thus prior bias diagonals equal their propagated values before
  bias noise, including with nonzero cross-covariances. Reading the prior avoids
  a second prediction. This is scoped to this INS model, not arbitrary dynamics.
- Validate all twelve noise coefficients and six consumed bias diagonals as
  finite/nonnegative, even on inhibited axes. Non-finite input returns
  `non_finite_input`; negative input returns `domain_error`. No universal
  precedence is promised for multiple independent invalid inputs. Other entries
  of P are not read here; valid symmetric PSD covariance remains the prediction
  caller's premise. Failure preserves the whole output, including an alias of
  configured noise; success permits that alias. State/P are never changed here.
- Derive every step from configured noise, then apply `R-IMU-PROCESS-NOISE`
  when fault/clipping metadata is available. A correction that reduces variance
  below the selected threshold releases inhibition immediately. This rule takes
  no learning permissions, motion/clipping or enable flags directly; fault and
  clipping influence its thresholds through the supplied IMU variances.
- This is a pre-increment check, not a clamp: one permitted increment may cross
  the threshold. The next step inhibits further bias noise while variance stays
  above it; an already-high covariance is not reduced. No hard covariance bound,
  physical sensor-health claim or bias-value limit follows from this rule.

The threshold quantities have matching units: m²/s⁴ for accelerometer bias and
sampled specific force, rad²/s² for gyro bias and sampled angular rate. Preserve
`E-NOISE`: the selected continuous driving-noise intensity is multiplied by `dt`.
PX4's bias-driving parameters instead have units m/s³ and rad/s² and contribute
`(sigma * dt)^2`. At a fixed dt, matching that contribution would require
`intensity = sigma^2 * dt`; there is no constant conversion across variable dt.
This feature aligns the threshold decision, not parameter values or discretization.
Do not copy PX4's bias-noise numbers into our intensity fields unchanged.

Replay applies bias-noise selection before every real or gap-hold prediction,
using the IMU variances selected by `R-IMU-PROCESS-NOISE` (or configured variances
when that selection is unavailable) and unchanged configured intensities
(1e-6 m²/s⁵ and 1e-8 rad²/s³). These are explicit replay settings, not newly
calibrated flight noise. Prediction failure still aborts.
`bias_learning.csv` records the six selected-noise flags for real IMU ticks;
summary inhibition counts include gap-hold predictions and count only configured
positive intensities suppressed by the rule. Selection is not proof that a tiny
floating-point increment changed P. Native backend, threshold, rollback,
correction/recovery and replay tests support the rule; new Lean/ESBMC evidence
remains pending. Full PX4 covariance constraints remain separate work.

### R-BIAS-LIMITS: per-axis bias-value limits

`runtime::try_limit_bias(state, limits)` explicitly limits each body-axis INS
accelerometer bias to `[-limits.accelerometer, +limits.accelerometer]` (m/s²)
and gyro bias to `[-limits.gyroscope, +limits.gyroscope]` (rad/s).

- Bounds are inclusive and per coefficient, not a vector-norm constraint.
  Zero is a valid limit that forces zero bias; in-range coefficients are preserved.
  Validate both limits and all six bias coefficients before writing. Non-finite
  inputs return `non_finite_input`; otherwise negative limits return `domain_error`.
  Failure preserves the entire state. Success changes only the six bias fields.
- This operation does not alter covariance, reject a measurement, reset a bias,
  or change learning permissions. Limiting applies even to an inhibited axis if
  its prior value is already outside the bound. With fixed limits and an in-range
  prior, inhibition still holds the estimated bias. An inward correction can
  leave saturation immediately; there is no latch or recovery timer.
- Replay fixes limits for the session and validates them against the zero bias
  initial state before publishing output. After each successful GNSS, gravity or
  magnetic correction group, apply limits before the next correction/prediction.
  Rejected/failed corrections do not invoke limiting. Successful core correction
  guarantees finite biases, so limiting with those fixed valid bounds cannot fail.
  Standalone numerical-core correction/injection APIs remain additive and unchanged.
- Preparation writes `bias_limits.csv` from logged `EKF2_ABL_LIM` and
  `EKF2_GYR_B_LIM`; absent values use EKF2 module defaults 0.4 m/s² and 0.15 rad/s,
  with each source recorded in metadata. Recorded changes through session end
  are rejected. CSV input without the file uses the same explicit defaults.
  Summary reports effective bounds/source and per-axis counts of corrections
  actually clipped (not time at the bound or evidence of sensor failure).

Behavioral reference: PX4 v1.16.2
[bias correction](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L757).
PX4 limits after each scalar fusion; replay limits after each existing measurement
group. The axis-bound rule agrees, but sequential saturation trajectories need
not agree. Clamping does not prove posterior covariance consistency or physical
bias accuracy. Native boundary, failure, correction and replay tests cover this
feature; Lean/ESBMC evidence and deployment-adapter wiring remain pending.

### R-VERTICAL-ACCELERATION: apparent falling and clipping detection

`runtime::VerticalAccelerationHealth<Linalg>` is a standalone monitor. It owns
clipping counters, falling likelihood and fault timing, not estimator state/P,
noise, bias permissions, fusion decisions or resets.

| Input | Meaning |
|---|---|
| IMU time | Nonzero, strictly increasing estimator-horizon timestamp in microseconds |
| Average interval | Caller-supplied average EKF interval in seconds, not observation cadence |
| Clipping | Known per-body-axis flags accumulated over this IMU interval |
| Six observation slots | Active flag, signed innovation and innovation variance S; pressure height, GNSS height/Down velocity, range height, vision height/Down velocity |

All innovations use prediction minus observation with positive meaning apparent
falling. Negate our core's observation-minus-prediction residual; range callers
must also convert the opposite distance-to-ground direction. The caller supplies
the latest cached diagnostics from active aiding sources, including gate-rejected
observations. Lifecycle owns source activity and data expiry; this monitor adds
no independent freshness timeout and does not treat rejection as deactivation.

- An active, finite, nonnegative variance below **FLT_EPSILON** is ignored. This
  fixed reference floor is used for float and double, not replaced by DBL_EPSILON.
  Otherwise form `innovation / sqrt(S)`, without a gate-sigma divisor or absolute
  value. Any source strictly above 3 gives medium likelihood. A >3 source plus
  a >1 source of a different reference type gives high likelihood; otherwise low.
  Types are pressure, GNSS, and ground: GNSS height/velocity share GNSS, and range
  plus both vision slots share ground. Equality at 1 or 3 does not exceed a test.
- Set `N = trunc(1 / average_interval_s)`. Per axis, increment if clipped and
  count < N; otherwise decrement if count > 0. Preserve the decrement even when
  clipping continues at N, and when a changed N is below the stored count. This
  is not a sliding window or a saturating counter. Warning means count >= N/2
  with integer division; critical means count >= N. `bad_clipping` means all
  three axes warn or any axis is critical. Frequent clipping means body Z warns
  or `bad_clipping` is set.
- `bad_sample` means high likelihood, or medium likelihood with frequent
  clipping. Update last-bad time on bad samples, otherwise last-good time.
  `bad_vertical_acceleration` holds until elapsed time since last bad is >=3 s.
  Every bad sample refreshes the hold. Use ordered timestamp subtraction to
  avoid overflow. Low likelihood/no active sources is absence of evidence, not
  proof of a healthy IMU. Last-good time describes the instantaneous predicate,
  even while the fault is held; it supplies the independent 500 ms reset policy
  in `R-GNSS-FUSION`.
- Validate time first (`out_of_range`), then a finite positive interval
  (`non_finite_input`/`domain_error`) and known clipping (`domain_error` if absent).
  A non-finite reciprocal returns `non_finite_result`; require reciprocal in
  [2,65536) before uint16 conversion (`out_of_range` otherwise). This keeps the
  conversion defined and the half-limit nonzero. For each active observation,
  require finite innovation/variance and nonnegative variance, even below the
  floor. Non-finite input returns `non_finite_input`, negative variance returns
  `domain_error`, and non-finite computed ratio returns `non_finite_result`.
  Inactive observation coefficients are not consumed. Scalar-root failures
  propagate. Validate all sources before publishing counters or timestamps.
- Failure preserves every stored field. The stored state then belongs to the
  previous successful timestamp and is not a verdict for the failed sample.
  Unknown clipping must not be converted to clear. Reset clears all history;
  callers restart on IMU gaps, unknown-metadata interruptions or source/calibration/
  frame changes. After reset, last-update zero means no evaluation yet.

Reference: PX4 v1.16.2 [height health monitor](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/height_control.cpp#L122)
and its [source-combination tests](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/test/test_EKF_accelerometer.cpp#L111).
Alignment covers valid inputs and finite arithmetic. Explicit invalid-input
rejection, missing-metadata handling and overflow-safe time subtraction are our
API safeguards; no matching invalid-data behavior or bitwise float/double parity
is claimed. The caller must match filter cadence, averaged interval and source
diagnostic ordering for matching timelines. PX4 evaluates after GNSS velocity
and before height fusion; bias/noise prediction consumes the preceding fault.
Native tests cover source combinations, signed residual conversion, counters,
recovery and failure atomicity. Lean/ESBMC evidence remains pending.

Replay integration:

- Evaluate once per real IMU prediction, including ticks without GNSS messages.
  Require the GNSS quality/lifecycle profile and known clipping. Legacy inputs
  without that profile report unavailable; do not infer source activity from a
  correction count. Barometer/range/vision slots remain inactive. GNSS height
  and velocity alone can only give medium likelihood, requiring clipping to
  trigger a fault.
- Use actual GNSS correction diagnostics, including gate-rejected results:
  negate the selected `fusion_innovation` and retain S. A permitted limit is
  reflected in the monitor cache, as in the reference's aid-source status;
  raw residuals/ratios remain in the fusion log. While velocity aiding remains
  active after a quality rejection, evaluate its velocity gate/limit for the
  cache without correcting the estimator.
  Invalid receiver velocity is not a new received sample; unavailable quality
  metadata invalidates the velocity cache. Numerical failures abort replay.
- Take the diagnostic snapshot after current velocity processing and before
  current height processing. It contains the current velocity cache and the
  preceding height cache. Height activity is from before height control; velocity
  activity reflects current horizontal control. Inactive sources are excluded;
  caches are cleared after lifecycle stop. Successful full velocity/height resets
  set that cache's innovation to zero and S to observation variance, as in the
  reference's aid-status reset. A Down-only reset retains velocity diagnostics.
- Evaluate the snapshot before height fusion, so height can consume the current
  fault. Prediction, bias permissions and every velocity event at this horizon
  consume the preceding horizon's fault. For multiple due GNSS messages,
  re-evaluate each pre-height snapshot from the same saved preceding monitor
  state and retain the final result. Clipping therefore advances once, while
  every height attempt sees its current pre-height evidence. This batching
  remains a replay approximation, not EKF2's receiver-buffer schedule.
- Initialize nominal/average interval from the seed IMU integration period.
  For each actual sample update `average = 0.99*average +
  0.01*clamp(dt, 0.5*nominal, 2*nominal)`. Accel/gyro periods are synchronized in
  replay. The averaging rule follows the reference; replay's logged prediction
  cadence replaces EKF2's configured downsampled cadence. Do not use GNSS dt.
  Missing intervals/unknown clipping clear monitor, diagnostic caches and average
  history before consuming a prior verdict. Held gap predictions never increment
  clipping counters. Restart from the next actual sample with known metadata.
- `vertical_acceleration_health.csv` reports availability, interval, counters,
  likelihood (0/1/2 = low/medium/high), current fault/timers and the signed input
  diagnostics with source timestamps. `bias_learning.csv` reports the preceding
  verdict used for permissions. Summary counts separate evaluated, unavailable,
  fault-held and fault-inhibited ticks. An available low verdict with no active
  diagnostics still does not prove health.

The replay fault now feeds accel bias-learning permissions and the sampled
noise selection in `R-IMU-PROCESS-NOISE`, plus the GNSS fusion exceptions in
`R-GNSS-VERTICAL-RECOVERY`. Its last-good time separately feeds the 500 ms
reset criterion in `R-GNSS-FUSION`. PX4 adapter wiring remains pending. Native replay
tests exercise rejection, input ordering, actual bias preservation/recovery and
metadata interruptions;
this is not a formal proof or flight-accuracy result.

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

#### Replay GNSS quality

PX4 replay evaluates the existing monitor before its independent navigation
gates. Follow [v1.16.2 GNSS checks](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_checks.cpp)
and [health qualification](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp).
The separate lifecycle above consumes these quality results and orchestrates
checked GNSS resets. Delayed buffering remains absent. The seed is an
initialization assumption, not qualified fusion.

- Read `EKF2_GPS_CHECK` and `EKF2_REQ_*` thresholds from the log. Map the ten
  mask bits to named checks; do not silently enable/disable checks. Support the
  legacy `EKF2_REQ_GDOP` name for PDOP. Use recorded `EKF2_REQ_GPS_H`, or explicitly
  record the reference's 10 s default when that parameter is absent in old logs.
  Raw speed limit remains 100 m/s. Reject known parameter/receiver changes;
  absent identity metadata does not prove receiver continuity.
- Preserve every post-seed message, including bad fixes. Pair navigation and
  quality CSV rows by exact timestamp. Fix, satellite count, reported accuracies,
  `hypot(hdop, vdop)` and receiver-local NED velocity feed the monitor. Malformed
  quality numerics must be rejected, not sanitized into valid data. Invalid
  geographic coordinates stop preparation rather than fabricate a position.
- Match the source adapter's `vel_ned_valid=false` skip before monitor evaluation.
  Such a row cannot authorize fusion or refresh health. Position-reference history
  advances only for velocity-valid messages. Drift uses the preceding fix's
  spherical azimuthal-equidistant projection (radius 6371000 m); Down uses the
  MSL altitude difference. This is separate from the trajectory's fixed-origin
  ECEF/NED projection. The first displacement after monitor reset is ignored.
- Obtain `in_air = !landed` and `at_rest` causally from land-detector messages
  less than 3 s old, not estimator outputs or GNSS velocity. Never infer rest
  from landed. In air, rest is unnecessary; on ground, selected motion checks
  require known rest. Missing required motion metadata blocks quality and resets
  qualification; this explicit incomplete-log policy is not an EKF2 transition.
- Map reported spoofing state `MULTIPLE(3)` to the spoofed flag as in EKF2.
  Missing spoofing metadata blocks qualification only when that check is enabled.
  Preserve availability diagnostics; missing metadata is not a measured healthy
  receiver. No jamming detection is added.
- Only quality `accepted` enters the existing gate/correction calls. Waiting,
  rejection, unavailable metadata and invalid velocity do not change state/P
  through GNSS or refresh horizontal aiding. Prediction and other sensors still
  run. Numerical quality rejection continues; malformed configuration/time/CSV
  or numerical correction failure stops replay. Count quality skips separately
  from innovation rejections. Log outcomes, failure flags, diagnostic validity
  and health times in `result/gnss_quality.csv`.

Without both quality files, legacy/KITTI input retains its previous behavior
and reports quality OFF. A partial pair is an error. Native and replay checks
support this integration; new Lean/ESBMC proofs and flight tuning remain pending.

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
