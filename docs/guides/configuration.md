# Configuration inventory and ownership

F-06.2 configuration reference, reconciled with `0eb3add` on 2026-11-18. This records existing choices from numerical foundations through
core, runtime and PX4. Kconfig now exposes build options and a packaged PX4
deployment profile while preserving defaults. Replay retains its separate
settings. Remaining configuration work is [F-06.2 in the roadmap](../roadmap.md#f-06--diagnostics-and-configuration).

## Scope and interpretation

Scope: scalar, linear algebra, SO(3), ESKF, portable runtime, root build
configuration and the PX4 module/package. Replay is a separate caller. Proof
harness bounds and tool settings are not estimator tuning.

Method: inspect declarations, macros, template choices, public call arguments,
fixed policy literals and their module call sites. This is a source inventory,
not an AST completeness proof or a claim that every possible configuration has
been tested. Arrays and aggregates below are named settings, not scalar counts.

| Kind | Meaning and owner | Change boundary |
| --- | --- | --- |
| Build choice | Scalar/backend, state formulation, algorithm switch, reserved capacity; selected by the embedding build | Rebuild and configuration-specific verification |
| Session setting | Noise, gate, installation, qualification/timing policy; supplied by caller/module | Core operations consume arguments; stateful runtime owners may require reset. PX4 settings are startup-only |
| Per-call input/control | Actual measurement/covariance, clipping, health, alignment, source identity, fusion permission | Supplied at the correct sample horizon; never a menu setting that fabricates sensor health |
| Fixed behavior | Frame conventions, numerical branches, supported reset/gating rules and protocol meanings | Design/specification change; not ordinary tuning |
| Derived value | Discrete noise, gains, history lengths, thresholds derived from another setting | Compute from the owning setting; do not expose a second independent knob |

## Scalar, linear algebra and SO(3)

| Choice or argument | Current behavior / domain | Recommendation |
| --- | --- | --- |
| Scalar format | [StandardMath](../../include/formal_eskf/scalar/backend/standard.hpp) admits IEEE binary32 `float` or binary64 `double`; no default template argument | Keep a build choice; evidence must identify the actual format |
| Math implementation | All three production linalg backends bind `scalar_math_type` to `StandardMath<value_type>` | No runtime libm selector; another implementation requires an explicit backend and evidence |
| Linalg backend | `EigenBackend<Scalar>`, `Px4MatrixBackend<Scalar>`, `CmsisDspBackend<Scalar>`; dependencies supplied by host | Keep compile-time selection; building adapters does not select one for every caller |
| Matrix/vector dimensions | `Matrix<Linalg, Rows, Columns>` / `Vector<Linalg, Size>` | Mathematical shape, not a tuning parameter; state dimensions follow the formulation |
| Vector/quaternion normalization `minimum_norm` | Explicit finite positive input; result must satisfy the normalization domain. Core prediction additionally limits its quaternion minimum to at most 1 | Caller-owned numerical contract, not a sensor-noise setting |
| Symmetry / quaternion equivalence `tolerance` | Explicit finite nonnegative absolute tolerance in `is_symmetric` / `equivalent`; no global tolerance setting | Keep per-operation meaning; do not share one epsilon across unrelated checks |
| SPD solve | Cholesky/LLT, exact symmetry checks, existing positive/finite divisor rules | No selectable LU, regularization or hidden diagonal jitter |
| Quaternion Exp/Log | Machine-epsilon small-angle branch and fixed Taylor coefficients | Numerical implementation, not an approximation-mode menu |
| SO(3) right Jacobian | Squared-angle branch threshold `0.00390625` (angle 1/16 rad), fixed series | Keep tied to the numerical specification/proofs |

Sources: [linalg operations](../../include/formal_eskf/linalg/operations.hpp),
[solve](../../include/formal_eskf/linalg/solve.hpp),
[quaternion operations](../../include/formal_eskf/so3/operations.hpp),
[Exp/Log](../../include/formal_eskf/so3/rotation_vector.hpp),
[right Jacobian](../../include/formal_eskf/so3/right_jacobian.hpp).
Machine epsilon/minimum normal, mathematical coefficients and unit conversions
are not user-adjustable safety thresholds.

## ESKF core

### Compile-time choices

| Choice | Library default / supported choices | PX4 module |
| --- | --- | --- |
| `Configuration` in `EskfTypes` | Explicit `Ins` or `Ahrs`; nominal/error/noise dimensions 16/15/12 or 4/3/3 | Fixed INS; no AHRS module switch |
| `ESKF_QUAT_APPROX` | 0 / CMake OFF: exponential attitude prediction; 1: normalized Euler and first-order attitude error transition | Packaged profile; default 0 |
| `ESKF_RESET_APPROX` | 0 / OFF: SO(3) right Jacobian; 1: first-order reset | Packaged profile; default 0 |
| `ESKF_MAG_TILT` | 1 / ON: restrict magnetic attitude gain to navigation Down; 0: full attitude gain | Packaged profile; default 1 |

These three algorithm switches apply consistently to every translation unit using
the same core types. Mixed definitions are not supported. No selectable accel2
model, ENU state frame, estimated gravity state, dynamic state dimension or
generated/sparse kernel exists in this baseline. Codegen remains a separate
experiment; it must not appear as an already supported configuration.

### Caller-supplied parameters

[INS](../../include/formal_eskf/eskf/configuration/ins.hpp) and
[AHRS](../../include/formal_eskf/eskf/configuration/ahrs.hpp) own these definitions.
**Their zero-initialized parameter aggregates are not ready-to-run defaults.**
Positive normalization/time/tolerance fields require explicit setup. The table
distinguishes legal zero noise from invalid zero domain limits.
Noise fields are members of the nested `process_noise` aggregate.

| Field / argument | Unit and current contract | Default module binding |
| --- | --- | --- |
| INS `gravity_n` | Fixed NED acceleration, m/s²; convention `[0,0,+g]`, finite consumed coefficients | `[0,0,9.80665]` |
| `specific_force_variance[3]` (INS) | Per-sample body-axis variance, m²/s⁴; finite ≥0; multiplied by `dt²` | Isotropic 1 |
| `angular_rate_variance[3]` | Per-sample body-axis variance, rad²/s²; finite ≥0; multiplied by `dt²` | Isotropic 0.0025 |
| INS `accelerometer_bias_random_walk_variance_density[3]` | m²/s⁵; finite ≥0; multiplied by `dt` | Isotropic 1e-6 |
| INS `gyroscope_bias_random_walk_variance_density[3]` | rad²/s³; finite ≥0; multiplied by `dt` | Isotropic 1e-8 |
| `minimum_quaternion_norm` | Dimensionless, finite; prediction requires `0 < value ≤ 1` | 1e-6 |
| `dt_min`, `dt_max` | Seconds, finite; `0 < dt_min ≤ dt_max`; actual `dt` in that interval | 1e-6, 0.05; module admission is narrower |
| `quaternion_squared_norm_tolerance` | Dimensionless, finite, strictly between 0 and 1; bound on prior squared-norm residual | 1e-10 |
| Initial nominal state and covariance `P` | Caller seed, with unit/frame and PSD premises; not a hidden global default | Module alignment and startup variance profile |
| Measurement covariance `V` / generic Jacobian `H` | Caller-supplied observation model and covariance; compatible dimensions and SPD innovation solve premises | Derived from sensor accuracy and profile noise; see below |
| `gate_sigma` in `check_innovation` | Finite >0; squared gate and denominators must remain usable; per-axis rejection above ratio 1 | Runtime selects whether to gate; no implicit gate inside core correction |
| INS `BiasUpdate.accelerometer[3]`, `.gyroscope[3]` | Per-correction Boolean permissions, default all true; separate from bias process-noise inhibition | Derived each interval by IMU/bias policy |
| Magnetic reference `m_n`, gravity observation `gravity_n` | Physical model inputs, not raw angle or normalized-vector replacements | Model/reference owner and fixed gravity |

Anisotropic noise is supported by the core; the module currently selects
isotropic values. Selecting observation functions is a caller decision, not a
global “sensor type” parameter. Covariance symmetrization, Joseph correction,
right-local attitude error, scalar-first quaternion order and NED/body-FRD
conventions remain fixed behavior.

## Runtime

### Existing configurable interfaces

“Default 0” below describes the aggregate initializer, not recommended tuning.
All floating inputs still require their operation's finite/arithmetic checks.

| Owner and fields | Current default / unit / domain | Module use and change boundary |
| --- | --- | --- |
| [ImuBiasInhibition::Parameters](../../include/formal_eskf/runtime/imu.hpp): `maximum_specific_force`, `maximum_angular_rate`, `decay_time_constant` | 0; m/s², rad/s, s; each >0 | 25, 3, 0.5; constructor-bound for the session |
| Same: `accelerometer_enabled`, `gyroscope_enabled` | true, true | Both true; these control correction permissions, not the independent process-noise inhibition rule |
| `select_imu_process_noise` argument `fault_variance` | Explicit variance in m²/s⁴, finite ≥0; replaces selected axes, even if lower than the nominal value | 4.9²; not an additive variance or a density |
| `BiasLimits.accelerometer`, `.gyroscope` | 0; m/s² and rad/s; finite ≥0, zero forces zero bias | Per-axis bounds 0.4 / 0.15 by default; menu selects one bound per bias type |
| [GnssQualityParameters](../../include/formal_eskf/runtime/gnss_quality.hpp): `enabled_checks` | Ten selectors listed below; all true in the parameter aggregate | Module uses the runtime defaults; change requires monitor reset/requalification |
| `minimum_satellites`, `maximum_pdop` | 6 satellites, 2.5; satellite field is uint8, PDOP positive normal finite | Same; configurable quality policy, not mandatory fix validity |
| `maximum_horizontal_accuracy`, `maximum_vertical_accuracy`, `maximum_speed_accuracy` | 3 m, 5 m, 0.5 m/s (standard deviations); positive normal finite | Same; speed limit is also used by the bounded vertical-innovation exception |
| `maximum_horizontal_drift`, `maximum_vertical_drift`, `velocity_limit` | 0.1, 0.2, 100 m/s; positive normal finite with overflow/underflow checks | Same; drift limits also bound stationary speed checks |
| `minimum_health_time_us` | 10,000,000 us, >0 | Same; recovery duration is derived, not another setting |
| [GnssFusion::Parameters](../../include/formal_eskf/runtime/gnss_lifecycle.hpp): `horizontal_timeout_us`, `height_data_timeout_us`, `height_fusion_timeout_us` | All 0; positive; horizontal timeout must permit safe doubling | 7 s / 1 s / 5 s; immutable after first update until reset. Module delegates height to its separate coordinator |
| [MagneticFieldParameters](../../include/formal_eskf/runtime/magnetic_field.hpp): `strength_tolerance`, `inclination_tolerance`, `heading_tolerance`, `minimum_horizontal_field` | All 0; ≥0 µT, [0,π] rad, [0,π] rad, positive normal µT | 20 µT / π/9 rad / 0 / 5 µT; heading check is not enabled by the module's per-call observability argument |
| `MagneticFieldChecks.strength`, `.inclination`, `.require_reference` | true / true / false | **true / false / false**; module intentionally differs from runtime defaults |
| [MagneticHeading::Parameters](../../include/formal_eskf/runtime/magnetometer.hpp): `heading_noise`, `acceleration_limit`, `minimum_horizontal_field` | All 0; `(0,π]` rad, ≥0 m/s², positive normal µT | 0.3 rad / 0.5 m/s² / 5 µT; a policy change must reconsider retained heading history |
| [MagnetometerFusion::Parameters](../../include/formal_eskf/runtime/magnetometer_lifecycle.hpp): `health_time_us`, `maximum_interval_us`, `fusion_timeout_us` | All 0; health 0 disables that duration only; others >0; interval must permit safe doubling | 1 s / 0.5 s / 7 s; immutable within session; health still needs four samples |
| Magnetic heading reset `yaw_variance` | Explicit finite nonnegative rad², with the fixed reset floor below | 0.3² rad²; heading noise currently couples consistency and reset uncertainty |
| [AccelerometerSample](../../include/formal_eskf/runtime/accelerometer.hpp) template `IntervalUs` | Explicit compile-time 1,000–50,000 us; whole input intervals, not exact periodic triggering | 10,000 us; changing it also requires reviewing magnitude-filter cadence |
| Gravity fusion `V`, `enabled` | Explicit effective covariance and caller enable; no innovation-gate parameter in this path | `V = (gravity_n·gravity_n) I`, ≈96.17 m²/s⁴; `FESKF_GRAV_EN` |
| [Navigation fusion](../../include/formal_eskf/runtime/navigation.hpp): `gate_sigma`, `GnssVelocityConditions.maximum_speed_accuracy`, height `deadzone` | Positive gate/accuracy limit; deadzone finite ≥0 m, default 0 | GNSS gate 5 / speed limit 0.5 m/s; height gate 5, conditional barometer deadzone 4 m |
| Navigation reset variances | Explicit finite nonnegative position/velocity variances; fixed floors below | Receiver-derived/reset profile; reset permission remains lifecycle-owned |
| [RingBuffer](../../include/formal_eskf/runtime/ring_buffer.hpp): `Capacity`, `length`, `max_age_us` | Capacity compile-time >0; active length 1..Capacity at startup/reset; age-window argument defaults 100,000 us, strict age comparison | Fixed backing storage; histories derive supported active lengths. Age zero admits no sample |

The ten GNSS selectors are `satellites`, `pdop`, `horizontal_accuracy`,
`vertical_accuracy`, `speed_accuracy`, `horizontal_drift`, `vertical_drift`,
`horizontal_speed`, `vertical_speed`, `spoofed`. A standalone
`GnssQualityChecks{}` has all false; it is also used to store failure bits.
It must not be confused with `GnssQualityParameters{}.enabled_checks` (all true).
Mandatory fix/time/numerical checks cannot be disabled by these selectors.

### Fixed policy values outside parameter structs

These are relevant to configuration design even though no setter exists.
Keep them fixed for the first parameter exposure; moving a literal to a parameter
must preserve its requirement or explicitly revise it and its evidence.

| Owner | Existing fixed behavior | Configuration implication |
| --- | --- | --- |
| `AccelerometerMagnitude` | g=9.80665 m/s²; strict 0.9g–1.1g band; nominal 10 ms / 90 ms LPF gives α=0.1 | Gravity duplicates the module model value; do not allow changing one independently |
| `AccelerometerSample` | Period admission ≤50 ms; average-period initial seed 5 ms; average update gain 0.1 | Internal sampling policy, not a new 200 Hz module setting |
| [TiltAlignment](../../include/formal_eskf/runtime/tilt_alignment.hpp) | 3° limit (`π/60`), latch when summed N/E tilt variance < limit² | Alignment criterion, not an arbitrary “aligned” configuration Boolean |
| `ImuBiasInhibition` | Fake-position observability threshold `abs(R(2,axis)) > 0.966` | Derived learning permission; keep separate from bias magnitude limits |
| `GnssQuality` | Mandatory fix type ≥3; drift filter dt clamped 0.001–10 s, α=dt/10; input clipping at 10× drift limit; recovery >max(health/10,1 s) | Filter and recovery semantics depend on the public thresholds |
| `inhibit_bias_process_noise` | Retain each bias intensity only while prior bias variance is below its corresponding sampled IMU variance | Derived from noise/P, independent of correction learning permission; no separate ceiling parameter |
| `GnssFusion` | Vertical acceleration reset delay 0.5 s; some horizontal recovery uses 2× timeout | Derived timing, not separate independent knobs |
| `MagneticHeading` | 1 s horizontal-acceleration LPF; nominal 10 ms/90 ms heading-innovation LPF | Heading noise threshold is not either LPF time constant |
| `MagnetometerFilter` | Fixed α=0.1, applied per consumed magnetic sample, used for resets | No runtime time-constant parameter |
| `check_magnetic_field` | Without a health reference and when allowed, strength fallback 45±40 µT, inclination skipped | Fallback does not replace the correction's NED field; not the configurable model-strength tolerance |
| `MagnetometerFusion` | Four samples to start; no-data stop at 2× maximum interval | Changes affect lifecycle evidence |
| [MagneticReference](../../include/formal_eskf/runtime/magnetic_reference.hpp) | Check after >10 s unless initial/origin trigger; adopt differences >1° or >1 µT | Model data are external inputs; cadence/change thresholds are fixed |
| [VerticalAccelerationHealth](../../include/formal_eskf/runtime/vertical_acceleration_health.hpp) | Positive innovation ratios >1 and >3; variance floor `FLT_EPSILON` even in double; clipping limit truncates 1 s / mean dt, warns at half; bad-acceleration hold 3 s | Reference-policy constants; no generic “fault sensitivity” setting |
| Navigation / heading reset | Minimum standard deviation 0.01 m, m/s or rad according to reset; horizontal position reset uses summed N/E variances | Keep units separate; identical literals do not imply one shared parameter |

`Conditions` / `Sample` / `Result` fields are not all options. At-rest/in-air,
clipping, healthy/ready/aligned, aiding activity, timestamps, calibration count,
source/reference generation, reported accuracy and innovation evidence describe
current inputs or state. Caller enable controls may originate in configuration;
eligibility and success must still be established by the runtime logic.

## PX4 binding

### Existing startup parameters: 15 definitions

Source: [formal_eskf_params.c](../../integrations/px4/module/formal_eskf_params.c).
The module reads 14 at startup in
[load_startup_parameters](../../integrations/px4/module/FormalEskf.cpp);
`FESKF_EN` belongs to the boot selection script. No running parameter-update
subscription exists. Metadata bounds are not a substitute for startup checks.

| Parameters | Default | Unit / admitted setting |
| --- | --- | --- |
| `FESKF_EN` | 1 | Boot chooses this module when value is 1; metadata Boolean. Boot also selects `SENS_IMU_MODE=1` and disables EKF2 |
| `FESKF_DECL_TYPE` | 3 | Integer bit mask 0..3: geographic heading / stored fallback |
| `FESKF_MAG_DECL` | 0 | Finite east-positive degrees, −180..180 |
| `FESKF_GRAV_EN` | 1 | Integer 0 or 1 |
| `FESKF_DT_US` | 4000 | Only 4000 us implemented; metadata's minimum 0 does not admit other periods |
| `FESKF_DELAY_MS` | 200 | Nonnegative metadata; actual positive value must satisfy joint arrival/delay/capacity constraints; zero source delays admit integer 153..239 ms |
| `FESKF_GPS_DLY`, `FESKF_MAG_DLY`, `FESKF_BARO_DLY` | 0 each | Nonnegative integer ms; jointly validated with delay and fixed storage |
| `FESKF_IMU_POS_X`, `FESKF_IMU_POS_Y`, `FESKF_IMU_POS_Z` | 0 each | Finite metres, body forward/right/down, each −10..10 |
| `FESKF_GPS_POS_X`, `FESKF_GPS_POS_Y`, `FESKF_GPS_POS_Z` | 0 each | Same installation convention and per-axis bounds |

Timing formulas and supported envelope are owned by the
[timing profile](../spec/profiles/px4-sih-250hz.md#startup-parameters-and-fixed-storage)
and [configure_timing](../../integrations/px4/module/ImuHistory.hpp). Requested lag,
active length, reserved capacity and actual measured interval are different
quantities. Active length is derived, not another independent parameter.

## Kconfiglib integration

The supported frontend is `make menuconfig`; it keeps CMake independently usable.
A pinned local Kconfiglib checkout supplies the menu. Exact/approximate prediction
and reset are exclusive choices; magnetic tilt protection defaults on. PX4 stays
INS/binary64/PX4 matrix. Packaged settings and startup parameters are separate;
neither enables live retuning. Instructions are in the [PX4 guide](../../integrations/px4/README.md#configure-a-px4-profile).

### PX4 deployment menu

| Group | Settings | Existing defaults / behavior |
| --- | --- | --- |
| Features (5) | WMM, ground-effect protection, gravity aiding support, accelerometer/gyro bias learning | All enabled; required motion, health and alignment conditions remain effective |
| IMU noise and bias (10) | Sample variances, bias variance densities, fault acceleration standard deviation, learning force/rate/decay and two bias limits | Units and sampled/density scaling remain as listed above |
| Magnetic aiding (14) | Three health selectors, strength/inclination tolerances, minimum horizontal field, heading noise/acceleration criterion, observation variance, innovation gate, health/sample/fusion/model intervals | Strength check on; inclination and require-WMM off; health 1 s, sample interval 0.5 s, fusion timeout 7 s, model lookup 10 s |
| GNSS quality (19) | Ten check selectors, satellites/PDOP/position/speed/drift limits, speed input bound and health duration | All ten checks on; six satellites, PDOP 2.5, 3/5 m, 0.5 m/s accuracy, health 10 s |
| GNSS fusion (5) | Horizontal timeout, velocity and position noise floors/cap, innovation gate | Timeout 7 s, velocity 0.3 m/s, position 0.5–10 m, gate 5 sigma |
| Height / ground effect (9) | Fusion and two data timeouts, barometer variance, GNSS noise floor/variance cap, gate, deadzone and hold time | Fusion 5 s; barometer 0.4 s / GNSS 1 s; deadzone 4 m; hold 10 s |
| Output predictor (2) | Velocity and position tracking time constants | Both 0.25 s |

WMM disabled suppresses host lookup and geographic adoption. The existing
measured-field/stored-declination/magnetic-north fallback remains; geographic
heading selection alone is distinct from the WMM build switch. Requiring a WMM
health reference depends on WMM being enabled. Ground-effect disabled leaves
ordinary barometric innovation unchanged; enabling it retains the delayed
land-detector and rotary-wing conditions. Gravity build exclusion takes precedence
over `FESKF_GRAV_EN`, while tilt alignment still runs. Bias permissions do not
zero the stored biases or disable independent bias process-noise inhibition.

Float strings must be finite decimal numbers in the admitted domain. Nonzero
values must have a positive normal finite binary64 square; zero is allowed only
for the declared variances, bias limits and tolerance/disable values. Heading
noise and inclination tolerance cannot exceed pi. Position noise floor/cap,
height noise/variance cap, and GNSS clipping arithmetic are checked together.
Positive millisecond durations are bounded by INT32_MAX; magnetic qualification
alone permits zero. This establishes representability and explicit relationships,
not calibrated flight-safe tuning. Existing per-operation checks remain intact.

Fixed admission ages/cadence and policy-derived expiration rules still apply.
For example, magnetic no-data expiry uses twice the selected maximum interval;
GNSS recovery derives from qualification duration; the geographic reference
owner retains its own adoption cadence even when lookup cadence changes.
Do not interpret one selected timeout as the sole source-health criterion.

### Precedence and scope

`make` uses `.config` through generated CMake settings. Direct `cmake` does not
read `.config`. The PX4 installer uses the menu only with explicit `--config`;
otherwise it uses checked-in defaults. A configured package records the config
and generated header in its manifest and is self-contained after export.
Changing source/configuration requires reinstall/rebuild; startup parameters
require restart. Replay retains its existing macros and tuning.

## Verification and review boundary

Configuration identity must include scalar/backend, algorithm switches, host
dependency versions, compile flags, fixed deployment profile and effective
startup values. Defaults alone do not identify a running session. Keep requested
and derived timing values distinguishable in evidence.

For each implementation batch, link the affected existing requirements in
[verification-requirements](../spec/README.md) and the
[traceability map](../../formal/agent-review/traceability-map.json). Core mathematical
theorems do not prove parameter parsing or real-time admissibility. Existing
ESBMC results retain their actual scalar/backend/mode/domain scope; exposing a
startup parameter does not automatically extend that evidence to a deployment.

| Setting change | Existing requirement impact to review |
| --- | --- |
| State model, prediction/noise and numerical limits | `E-STATE`, `E-PRED`, `E-NOISE`, `E-PCOV`, `E-STEP`; affected quaternion/scalar/linalg contracts |
| Reset mode, measurement weights and gates | `E-RESET`, `E-CORRECT`, `E-OBS`, `E-INNOV`, `R-MEASUREMENT-FUSION` |
| IMU fault noise, bias learning/limits | `E-BIAS-UPDATE`, `R-PX4-IMU-BIAS`, `R-IMU-PROCESS-NOISE`, `R-BIAS-PROCESS-NOISE`, `R-BIAS-LIMITS` |
| Gravity sampling/eligibility/weight | `R-ACCELEROMETER-MAGNITUDE`, `R-ACCELEROMETER-CONDITIONS`, `R-ACCELEROMETER-FUSION` |
| Magnetic mode, model/checks, heading and lifecycle | `E-MAG-TILT`, `R-MAGNETIC-FIELD`, `R-MAGNETIC-HEADING`, `R-MAGNETIC-REFERENCE`, `R-MAGNETOMETER-FUSION`, `R-MAGNETOMETER-LIFECYCLE`, `R-MAGNETOMETER-FILTER`, `R-MAGNETOMETER-RESET` |
| GNSS/height quality, timeouts and recovery | `R-GNSS-QUALITY`, `R-GNSS-FUSION`, `R-GNSS-RESET`, `R-GNSS-VERTICAL-RECOVERY`, `R-HEIGHT-FUSION`, `R-VERTICAL-ACCELERATION` |
| Timing, capacities and predictor | `R-RING-BUFFER`, `R-PX4-IMU-HISTORY`, `R-PX4-OBSERVATION-HISTORY`, `R-PX4-OUTPUT-PREDICTOR`, `R-PX4-DELAYED-FUSION`, architecture `PX4-TIME`/`PX4-RT` |
| Installation, reference frame and output thresholds | `R-PX4-SENSOR-POSITIONS`, `R-PX4-FRAME-OUTPUT`, `R-PX4-STATUS` |

Acceptance for parameter binding: unchanged defaults, actual conversion into
each consumer, invalid/NaN/infinite/boundary/cross-constraint rejection before
estimation, no live mutation before restart, and affected integration tests.
Then identify numerical/proof/resource gaps for new supported combinations.
No new proof pass or flight qualification is claimed by this documentation.

Review focus: ownership split, candidate versus fixed groups, noise units,
coupled settings, restart semantics and the standalone menu's build boundary.
Remaining F-06.2 decisions concern additional startup bindings and supported
profiles, not whether these 64 build-profile controls reach the module.

## Source of truth

The complete supported menu and defaults are in `Kconfig` and
`configs/px4.Kconfig`. `DeploymentConfig.hpp` is the checked-in default output;
`DeploymentProfile.hpp` binds it to typed module values. The 2026-11-18
configuration tests check that menu symbols reach production consumers.
Use the source declarations for exhaustive values rather than a second copied
profile table. Replay macros remain in `examples/replay_ins.cpp` and are not
controlled by the PX4 menu.
