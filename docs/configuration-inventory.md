# Configuration inventory and ownership

F-06.2 inventory and proposal, **review pending**. Inventory baseline: `478ccab`,
2026-11-18. This records existing choices from numerical foundations through
core, runtime and PX4. Kconfig now exposes build options and a packaged PX4
deployment profile while preserving defaults. Replay retains its separate
settings. The functional queue remains in [FEATURE_TODO](../FEATURE_TODO.md).

## Scope and interpretation

The scan covers all **44 production headers**: scalar 2, linalg 8, SO(3) 5,
ESKF 15, runtime 13 and the status header; root CMake; the PX4 module's nine
headers, four C++ translation units, parameter definitions, CMake/Kconfig and
boot binding. Replay settings are recorded separately because they are another
caller, not the deployed profile. Tests, proof harness bounds and tool options
are not estimator tuning.

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
| Scalar format | [StandardMath](../include/formal_eskf/scalar/backend/standard.hpp) admits IEEE binary32 `float` or binary64 `double`; no default template argument | Keep a build choice; evidence must identify the actual format |
| Math implementation | All three production linalg backends bind `scalar_math_type` to `StandardMath<value_type>` | No runtime libm selector; another implementation requires an explicit backend and evidence |
| Linalg backend | `EigenBackend<Scalar>`, `Px4MatrixBackend<Scalar>`, `CmsisDspBackend<Scalar>`; dependencies supplied by host | Keep compile-time selection; building adapters does not select one for every caller |
| Matrix/vector dimensions | `Matrix<Linalg, Rows, Columns>` / `Vector<Linalg, Size>` | Mathematical shape, not a tuning parameter; state dimensions follow the formulation |
| Vector/quaternion normalization `minimum_norm` | Explicit finite positive input; result must satisfy the normalization domain. Core prediction additionally limits its quaternion minimum to at most 1 | Caller-owned numerical contract, not a sensor-noise setting |
| Symmetry / quaternion equivalence `tolerance` | Explicit finite nonnegative absolute tolerance in `is_symmetric` / `equivalent`; no global tolerance setting | Keep per-operation meaning; do not share one epsilon across unrelated checks |
| SPD solve | Cholesky/LLT, exact symmetry checks, existing positive/finite divisor rules | No selectable LU, regularization or hidden diagonal jitter |
| Quaternion Exp/Log | Machine-epsilon small-angle branch and fixed Taylor coefficients | Numerical implementation, not an approximation-mode menu |
| SO(3) right Jacobian | Squared-angle branch threshold `0.00390625` (angle 1/16 rad), fixed series | Keep tied to the numerical specification/proofs |

Sources: [linalg operations](../include/formal_eskf/linalg/operations.hpp),
[solve](../include/formal_eskf/linalg/solve.hpp),
[quaternion operations](../include/formal_eskf/so3/operations.hpp),
[Exp/Log](../include/formal_eskf/so3/rotation_vector.hpp),
[right Jacobian](../include/formal_eskf/so3/right_jacobian.hpp).
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

[INS](../include/formal_eskf/eskf/configuration/ins.hpp) and
[AHRS](../include/formal_eskf/eskf/configuration/ahrs.hpp) own these definitions.
**Their zero-initialized parameter aggregates are not ready-to-run defaults.**
Positive normalization/time/tolerance fields require explicit setup. The table
distinguishes legal zero noise from invalid zero domain limits.
Noise fields are members of the nested `process_noise` aggregate.

| Field / argument | Unit and current contract | Current module binding |
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
| [ImuBiasInhibition::Parameters](../include/formal_eskf/runtime/imu.hpp): `maximum_specific_force`, `maximum_angular_rate`, `decay_time_constant` | 0; m/s², rad/s, s; each >0 | 25, 3, 0.5; constructor-bound for the session |
| Same: `accelerometer_enabled`, `gyroscope_enabled` | true, true | Both true; these control correction permissions, not the independent process-noise inhibition rule |
| `select_imu_process_noise` argument `fault_variance` | Explicit variance in m²/s⁴, finite ≥0; replaces selected axes, even if lower than the nominal value | 4.9²; not an additive variance or a density |
| `BiasLimits.accelerometer`, `.gyroscope` | 0; m/s² and rad/s; finite ≥0, zero forces zero bias | Per-axis bounds 0.4 / 0.15; currently fixed |
| [GnssQualityParameters](../include/formal_eskf/runtime/gnss_quality.hpp): `enabled_checks` | Ten selectors listed below; all true in the parameter aggregate | Module uses the runtime defaults; change requires monitor reset/requalification |
| `minimum_satellites`, `maximum_pdop` | 6 satellites, 2.5; satellite field is uint8, PDOP positive normal finite | Same; configurable quality policy, not mandatory fix validity |
| `maximum_horizontal_accuracy`, `maximum_vertical_accuracy`, `maximum_speed_accuracy` | 3 m, 5 m, 0.5 m/s (standard deviations); positive normal finite | Same; speed limit is also used by the bounded vertical-innovation exception |
| `maximum_horizontal_drift`, `maximum_vertical_drift`, `velocity_limit` | 0.1, 0.2, 100 m/s; positive normal finite with overflow/underflow checks | Same; drift limits also bound stationary speed checks |
| `minimum_health_time_us` | 10,000,000 us, >0 | Same; recovery duration is derived, not another setting |
| [GnssFusion::Parameters](../include/formal_eskf/runtime/gnss_lifecycle.hpp): `horizontal_timeout_us`, `height_data_timeout_us`, `height_fusion_timeout_us` | All 0; positive; horizontal timeout must permit safe doubling | 7 s / 1 s / 5 s; immutable after first update until reset. Module delegates height to its separate coordinator |
| [MagneticFieldParameters](../include/formal_eskf/runtime/magnetic_field.hpp): `strength_tolerance`, `inclination_tolerance`, `heading_tolerance`, `minimum_horizontal_field` | All 0; ≥0 µT, [0,π] rad, [0,π] rad, positive normal µT | 20 µT / π/9 rad / 0 / 5 µT; heading check is not enabled by the module's per-call observability argument |
| `MagneticFieldChecks.strength`, `.inclination`, `.require_reference` | true / true / false | **true / false / false**; module intentionally differs from runtime defaults |
| [MagneticHeading::Parameters](../include/formal_eskf/runtime/magnetometer.hpp): `heading_noise`, `acceleration_limit`, `minimum_horizontal_field` | All 0; `(0,π]` rad, ≥0 m/s², positive normal µT | 0.3 rad / 0.5 m/s² / 5 µT; a policy change must reconsider retained heading history |
| [MagnetometerFusion::Parameters](../include/formal_eskf/runtime/magnetometer_lifecycle.hpp): `health_time_us`, `maximum_interval_us`, `fusion_timeout_us` | All 0; health 0 disables that duration only; others >0; interval must permit safe doubling | 1 s / 0.5 s / 7 s; immutable within session; health still needs four samples |
| Magnetic heading reset `yaw_variance` | Explicit finite nonnegative rad², with the fixed reset floor below | 0.3² rad²; heading noise currently couples consistency and reset uncertainty |
| [AccelerometerSample](../include/formal_eskf/runtime/accelerometer.hpp) template `IntervalUs` | Explicit compile-time 1,000–50,000 us; whole input intervals, not exact periodic triggering | 10,000 us; changing it also requires reviewing magnitude-filter cadence |
| Gravity fusion `V`, `enabled` | Explicit effective covariance and caller enable; no innovation-gate parameter in this path | `V = (gravity_n·gravity_n) I`, ≈96.17 m²/s⁴; `FESKF_GRAV_EN` |
| [Navigation fusion](../include/formal_eskf/runtime/navigation.hpp): `gate_sigma`, `GnssVelocityConditions.maximum_speed_accuracy`, height `deadzone` | Positive gate/accuracy limit; deadzone finite ≥0 m, default 0 | GNSS gate 5 / speed limit 0.5 m/s; height gate 5, conditional barometer deadzone 4 m |
| Navigation reset variances | Explicit finite nonnegative position/velocity variances; fixed floors below | Receiver-derived/reset profile; reset permission remains lifecycle-owned |
| [RingBuffer](../include/formal_eskf/runtime/ring_buffer.hpp): `Capacity`, `length`, `max_age_us` | Capacity compile-time >0; active length 1..Capacity at startup/reset; age-window argument defaults 100,000 us, strict age comparison | Fixed backing storage; histories derive supported active lengths. Age zero admits no sample |

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
| [TiltAlignment](../include/formal_eskf/runtime/tilt_alignment.hpp) | 3° limit (`π/60`), latch when summed N/E tilt variance < limit² | Alignment criterion, not an arbitrary “aligned” configuration Boolean |
| `ImuBiasInhibition` | Fake-position observability threshold `abs(R(2,axis)) > 0.966` | Derived learning permission; keep separate from bias magnitude limits |
| `GnssQuality` | Mandatory fix type ≥3; drift filter dt clamped 0.001–10 s, α=dt/10; input clipping at 10× drift limit; recovery >max(health/10,1 s) | Filter and recovery semantics depend on the public thresholds |
| `inhibit_bias_process_noise` | Retain each bias intensity only while prior bias variance is below its corresponding sampled IMU variance | Derived from noise/P, independent of correction learning permission; no separate ceiling parameter |
| `GnssFusion` | Vertical acceleration reset delay 0.5 s; some horizontal recovery uses 2× timeout | Derived timing, not separate independent knobs |
| `MagneticHeading` | 1 s horizontal-acceleration LPF; nominal 10 ms/90 ms heading-innovation LPF | Heading noise threshold is not either LPF time constant |
| `MagnetometerFilter` | Fixed α=0.1, applied per consumed magnetic sample, used for resets | No runtime time-constant parameter |
| `check_magnetic_field` | Without a health reference and when allowed, strength fallback 45±40 µT, inclination skipped | Fallback does not replace the correction's NED field; not the configurable model-strength tolerance |
| `MagnetometerFusion` | Four samples to start; no-data stop at 2× maximum interval | Changes affect lifecycle evidence |
| [MagneticReference](../include/formal_eskf/runtime/magnetic_reference.hpp) | Check after >10 s unless initial/origin trigger; adopt differences >1° or >1 µT | Model data are external inputs; cadence/change thresholds are fixed |
| [VerticalAccelerationHealth](../include/formal_eskf/runtime/vertical_acceleration_health.hpp) | Positive innovation ratios >1 and >3; variance floor `FLT_EPSILON` even in double; clipping limit truncates 1 s / mean dt, warns at half; bad-acceleration hold 3 s | Reference-policy constants; no generic “fault sensitivity” setting |
| Navigation / heading reset | Minimum standard deviation 0.01 m, m/s or rad according to reset; horizontal position reset uses summed N/E variances | Keep units separate; identical literals do not imply one shared parameter |

`Conditions` / `Sample` / `Result` fields are not all options. At-rest/in-air,
clipping, healthy/ready/aligned, aiding activity, timestamps, calibration count,
source/reference generation, reported accuracy and innovation evidence describe
current inputs or state. Caller enable controls may originate in configuration;
eligibility and success must still be established by the runtime logic.

## PX4 binding

### Existing startup parameters: 15 definitions

Source: [formal_eskf_params.c](../integrations/px4/module/formal_eskf_params.c).
The module reads 14 at startup in
[load_startup_parameters](../integrations/px4/module/FormalEskf.cpp);
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
[timing profile](spec/px4-timing-profile.md#startup-parameters-and-fixed-storage)
and [configure_timing](../integrations/px4/module/ImuHistory.hpp). Requested lag,
active length, reserved capacity and actual measured interval are different
quantities. Active length is derived, not another independent parameter.

### Fixed module settings and implicit bindings

The appendix records the **123 baseline `inline constexpr` declarations** in
[DeploymentProfile.hpp](../integrations/px4/module/DeploymentProfile.hpp).
That count includes arrays/aggregates and conversion factors; it is **not** a
claim of 123 tunable scalar parameters. `bias_limits()` adds two explicit limits
(0.4 m/s² and 0.15 rad/s) outside those declarations.

Baseline bindings and their current configuration treatment:

- [FormalEskf.hpp](../integrations/px4/module/FormalEskf.hpp) fixes
  `Px4MatrixBackend<double>` + INS. GNSS quality now uses the packaged profile.
- [FormalEskf.cpp](../integrations/px4/module/FormalEskf.cpp) enables both bias
  learners by default, with independent profile permissions. Startup needs stationary IMU, magnetic heading, qualified GNSS and
  barometer; arbitrary source-disable switches are not yet supported.
- [FormalEskfAttitude.cpp](../integrations/px4/module/FormalEskfAttitude.cpp)
  derives gravity `V` from g²; normal magnetic correction defaults to diagonal 25 µT²,
  and the direct field-heading check is disabled while separate heading
  consistency is used. Reference availability is dynamic, not a check mask.
- [FormalEskfNavigation.cpp](../integrations/px4/module/FormalEskfNavigation.cpp)
  enables horizontal position and 3D velocity from valid GNSS metadata;
  [HeightAiding](../integrations/px4/module/HeightAiding.hpp) prefers active GNSS
  height, then barometer. No selectable reference-priority/source-disable policy.
- [MagneticModel](../integrations/px4/module/MagneticModel.hpp) uses the host WMM
  lookup when enabled by the build profile and retains separate declination selection; geographic coordinates/model output are
  inputs. [OutputPredictor](../integrations/px4/module/OutputPredictor.hpp) shares
  the model gravity/timing/numerical contract; its two tracking time constants are configurable.
- `SENS_IMU_MODE`, topic selection, source calibration and WMM version belong to
  the host integration. No live `EKF2_*` tuning values are read as our settings.

### Build and host-tool settings

| Owner | Options / current defaults | Treatment |
| --- | --- | --- |
| Root CMake | Three `ESKF_*` algorithm switches above | Core numerical configuration |
| Root CMake | `FORMAL_ESKF_BUILD_EIGEN_BACKEND=ON`, `FORMAL_ESKF_BUILD_PX4_MATRIX_BACKEND=OFF`, `FORMAL_ESKF_BUILD_CMSIS_DSP_BACKEND=OFF` | Build available adapters; no global runtime backend switch |
| Root CMake | `FORMAL_ESKF_PX4_MATRIX_TARGET=""`, `FORMAL_ESKF_PX4_INCLUDE_DIRS=""`, `FORMAL_ESKF_CMSIS_DSP_TARGET="CMSISDSP::CMSISDSP"` | Host dependency ownership; CPU/FPU/ABI/SIMD settings stay with the dependency/toolchain |
| Root CMake / CTest | `BUILD_TESTING=ON`; `FORMAL_ESKF_BUILD_EXAMPLES=OFF` | Build scope, not estimator behavior |
| Replay build | `FORMAL_ESKF_REPLAY_BACKEND=eigen` (or `px4_matrix`), `FORMAL_ESKF_PX4_WMM_TARGET=""` | Selected backend must be built; optional external magnetic-model helper |
| PX4 Kconfig | `MODULES_FORMAL_ESKF`, default n, depends on POSIX; selected in the installed SITL board profile | Host's build system owns module inclusion |
| PX4 CMake | Packaged algorithm definitions (default 0/0/1), C++20, no fast-math/FMA contraction, no exceptions/RTTI; `FORMAL_ESKF_MODULE_TESTS=OFF` | Supported deployment/evidence identity; language and FP constraints stay fixed |

Sources: [CMake](../CMakeLists.txt),
[PX4 CMake](../integrations/px4/module/CMakeLists.txt),
[module Kconfig](../integrations/px4/module/Kconfig).

[replay_ins.cpp](../examples/replay_ins.cpp) has **35 source macros** covering
gravity, quaternion/time limits, nominal/fault IMU noise, bias learning/limits,
gravity observation period/noise, magnetic checks/lifecycle, GNSS lifecycle,
measurement noise floors and initial variances. These are hardcoded source
definitions, not a documented external `-D` override interface. For example,
replay gyro variance is 4e-4 versus module 0.0025, quaternion tolerance 1e-12
versus 1e-10, position floors 1/1.5 m versus module's different GNSS floors,
and attitude/bias initial variances differ. Do not silently unify these profiles.
Input CSV flags/accuracy/model data and Python options (`--compare-mag`,
`--magnetic-model`, KITTI `--navigation-hz`) further select experimental inputs;
output paths, plotting and binary paths are host operation, not flight tuning.

### Couplings to resolve before adding parameters

| Finding | Required treatment |
| --- | --- |
| Gravity appears in the module model, runtime magnitude test and replay | Keep the same physical convention; a future configurable gravity must update the model and health band together. Replay remains a separate profile |
| 10 ms observation target and fixed 10 ms/90 ms LPFs coexist | A selectable cadence needs filter-response review; changing the sampler template alone does not retune the LPFs |
| Timing defaults occur in parameter metadata and `DeploymentProfile` | Add default-consistency tests when extending bindings; validate joint timing/storage constraints in the existing owner |
| Nominal IMU variance also determines bias-noise inhibition ceilings | Noise tuning changes both covariance growth and inhibition; test both effects |
| `heading_noise_rad` determines consistency threshold and yaw reset variance | Preserve the coupling initially; splitting it is a policy change |
| GNSS `maximum_speed_accuracy` controls quality and the vertical-innovation exception | Use one effective value for both consumers; a disabled quality selector must not silently disable the exception's limit |
| Initial covariance, learning permission and bias magnitude limits are distinct | Do not interpret a disabled learner as a bias reset, or a limit as uncertainty |

## Recommended F-06.2 design

Keep one ownership path: **caller/deployment configuration → existing core and
runtime parameter types → numerical operations**. The core must not read PX4
parameters, files, environment variables or Kconfig symbols for tuning.

| Group | Proposed disposition | Required before exposure |
| --- | --- | --- |
| IMU sample variances and bias variance densities | Exposed as build-profile values; preserve units, initially isotropic in module | Explicit domain, conversion/finite checks, default equivalence and changed-noise tests |
| GNSS, magnetic and barometer observation weights; gates where implemented | Exposed as build-profile values; gravity keeps g² effective weight | Distinguish standard deviation/variance/effective pseudo-observation weight; do not invent a gravity gate |
| GNSS quality thresholds/checks, magnetic health/consistency, bias enable/limits | Exposed as build-profile values | Preserve mandatory checks; reset/requalification and coupled consumers tested; do not independently disable safety prerequisites |
| Source selection / reference priority | Defer exposing new choices until their initialization, loss/recovery and publication behavior is specified | Existing dependence on GNSS/magnetometer/barometer cannot be removed with one Boolean |
| Lifecycle durations and predictor time constants | Exposed as build-profile values; startup covariance/alignment remains fixed | Domain/cross-constraints, history/cadence and convergence/recovery evidence |
| Capacities, supported rates, stack/queue/output deadline budgets | Compile-time deployment profile; only admitted lengths/delays at startup | PX4-TIME/PX4-RT bounds and target resource evidence |
| Frame/model constants, numerical branches, protocol/reporting thresholds | Keep specified behavior, not broad user tuning | A deliberate design change and affected proof/test review |

New startup values should be read into a small concrete parameter aggregate,
validated as a complete configuration, and installed before callbacks start.
Use direct assignments to existing types; no string-key registry, dynamic
allocation, generic parameter-reflection layer or duplicated estimator flow.
Preserve restart-only semantics initially. A future live update needs its own
transaction, timing/history and reset rules; `parameter_update` subscription alone
does not establish those rules.

Current constants are not automatically validated ranges. Before adding each
public parameter, define accepted lower/upper bounds and cross-constraints from
our numerical domain and the deployment profile. Do not copy bounds from EKF2
merely because names resemble ours, or imply every finite value is flight-safe.

In particular, pinned PX4 squares its gyro/accelerometer noise amplitudes, while
its bias random-walk addition uses `(dt * bias_noise)²` with its own variance
inhibition. Our bias fields are **variance densities multiplied by dt**. Identical
parameter values therefore do not express identical bias evolution. References:
[PX4 covariance](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/covariance.cpp#L114),
[PX4 parameter definitions](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/module.yaml#L116),
and our [process-noise definition](../include/formal_eskf/eskf/configuration/ins.hpp).

## Kconfiglib integration

`make menuconfig` configures the standalone build and the **PX4 deployment
profile**. [Kconfig](../Kconfig) owns the algorithm choices and standalone build;
[px4.Kconfig](../configs/px4.Kconfig) owns the 64 module settings below. The latter
are applied by package installation/export, not by the standalone CMake build.
INS/double/PX4 matrix remain fixed for the module. Its existing `FESKF_*` startup
parameters retain their host-owned interfaces and restart requirement.

Attitude prediction is an exclusive choice: **Exact (exponential map)**, the
default, maps to `ESKF_QUAT_APPROX=OFF`; **Approximate (first-order)** maps to
`ESKF_QUAT_APPROX=ON`. The existing C++ implementation and CMake interface are
unchanged.

Covariance reset likewise selects **Exact (SO(3) right Jacobian)**, the default
(`ESKF_RESET_APPROX=OFF`), or **Approximate (first-order)**
(`ESKF_RESET_APPROX=ON`). The two algorithm choices are independent.

The inspected [sysprog21/Kconfiglib revision](https://github.com/sysprog21/Kconfiglib/tree/6d9102abd55b614d3c6133529ffa40dbea7fd2cf)
provides dependency/choice evaluation, configuration generation and optional
interactive frontends. Its [symbol types](https://github.com/sysprog21/Kconfiglib/blob/6d9102abd55b614d3c6133529ffa40dbea7fd2cf/kconfiglib.py#L4358)
are bool/tristate/string/int/hex, with no native float parameter type. Floating
settings use decimal strings, parsed and validated before packaging; generated
C++ values are numeric literals. Timeouts use integer milliseconds and are
converted to uint64 microseconds at their consumers.

There is a concrete host compatibility issue: this revision
declares version 15.0.0 and [changes installed imports to package modules](https://github.com/sysprog21/Kconfiglib/blob/6d9102abd55b614d3c6133529ffa40dbea7fd2cf/README.md#installation-with-pip),
whereas pinned [PX4 kconfig.cmake](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/cmake/kconfig.cmake#L4)
imports/invokes top-level `menuconfig`, `defconfig` and `genconfig`. Installing
that fork into PX4's environment is not an established drop-in upgrade. Instead,
[configure.py](../scripts/configure.py) downloads the pinned source into ignored
`tools/kconfiglib/` on first use and imports it only for that process. Subsequent
runs use the checkout offline and check its revision and tracked-file integrity.
No global package installation or PX4 tool migration is performed.

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

### Commands and precedence

```bash
make menuconfig                       # Edit .config; save before exiting
make test                             # Configure, build, then run CTest
make configure CMAKE_ARGS=-DCMAKE_BUILD_TYPE=Release
make defconfig                        # Restore project defaults
python3 tests/test_kconfig.py          # Configuration integration checks
make px4-install PX4_DIR=/path/to/PX4-Autopilot
make -C /path/to/PX4-Autopilot px4_sitl_formal_eskf -j2
make px4-export PACKAGE_DIR=output/px4-package
```

- `CONFIG_FILE` selects the saved configuration; default `.config`. Kconfiglib
  keeps the previous saved contents in `.config.old`. Neither is tracked.
- `BUILD_DIR` defaults to `build/demo`; `JOBS` defaults to two compile workers.
  `make`/`make build` configures and builds; `make configure` only configures.
- `.config` owns menu-managed settings for the Make entry point. Every configure
  validates requests, then generates `BUILD_DIR/kconfig.cmake` and applies it to
  the CMake cache, including disabled options. Unknown/invalid assignments and
  unsatisfied dependencies fail rather than silently selecting another profile.
  The menu can be used to resolve obsolete or incompatible selections.
- `CMAKE_ARGS` supplies non-menu options such as toolchains, package search paths
  and build type. Attempts to set a menu-managed option with `-D` are rejected.
  External backend dependencies remain host-provided; selecting an adapter does
  not install its library or supply target CPU/FPU flags.
- Direct CMake builds and embedding via `add_subdirectory` remain independent of
  `.config` and Python. Use separate build directories when switching between
  direct CMake and Make; an existing CMake cache retains previously applied values.
  CMake owns the build interface, and the menu maps to its existing settings;
  `test_kconfig.py` checks that their defaults agree and mode changes reach the
  compiler flags without stale cache values. The standalone core needs no
  generated C++ header.
- `make px4-install` / `make px4-export` pass `CONFIG_FILE` explicitly to the
  installer. It validates the profile and generates `DeploymentConfig.hpp` in the
  package, with the three algorithm choices and 64 module settings. The selected
  configuration and generated header are retained and hashed in the package
  manifest. The source-tree default header is not overwritten. A relocated
  package builds without Kconfiglib or the original source repository.
- Direct `install.py` without `--config` uses the tracked default profile;
  exported packages already contain their selected profile. Reinstall and rebuild
  to change it. Restart the module after deploying; no configuration parsing or
  dynamic allocation is added to `Run()`. Replay settings are unaffected.

For reproducible evidence retain the saved `.config`, effective CMake cache,
compiler flags and dependency revisions alongside the verification results.
Changing the menu does not expand existing formal-proof configuration coverage.
Default equivalence and selected non-default module checks are recorded in the
[2026-11-18 validation](validation-px4-configuration-20261118.md).

## Verification and review boundary

Configuration identity must include scalar/backend, algorithm switches, host
dependency versions, compile flags, fixed deployment profile and effective
startup values. Defaults alone do not identify a running session. Keep requested
and derived timing values distinguishable in evidence.

For each implementation batch, link the affected existing requirements in
[verification-requirements](spec/verification-requirements.md) and the
[traceability map](../formal/agent-review/traceability-map.json). Core mathematical
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

## Appendix: complete module profile declaration list

This appendix retains baseline declaration expressions for comparison; the
deployment menu identifies values now configurable at build time. All remain
compile-time constants. “Candidate” records the earlier recommendation, not the
current implementation status; the menu section owns that status.
Numeric bounds are not supplied where no configurable domain exists yet; the
current value is not an invented accepted range. Namespace is relative to
`formal_eskf::px4::profile`; declaration expressions preserve source units.

| Declaration | Current expression | Unit / meaning | Proposed treatment |
| --- | --- | --- | --- |
| `gravity_m_s2` | `9.80665` | m/s² | Fixed model / unit convention |
| `minimum_quaternion_norm` | `1e-6` | dimensionless | Numerical contract |
| `quaternion_squared_norm_tolerance` | `1e-10` | dimensionless | Numerical contract |
| `seconds_per_microsecond` | `1e-6` | s/us conversion | Fixed model / unit convention |
| `microtesla_per_gauss` | `100.` | µT/gauss conversion | Fixed model / unit convention |
| `maximum_sensor_position_m` | `10.f` | m per axis | Fixed profile |
| `startup::gravity_tolerance_m_s2` | `0.5` | m/s² | Fixed profile |
| `startup::maximum_angular_rate_rad_s` | `0.1` | rad/s | Fixed profile |
| `startup::minimum_magnetic_field_gauss` | `0.1` | gauss | Fixed profile |
| `startup::minimum_imu_samples` | `250U` | samples | Fixed profile |
| `startup::maximum_imu_samples` | `1000U` | samples | Fixed profile |
| `startup::minimum_magnetic_samples` | `20U` | samples | Fixed profile |
| `startup::maximum_sample_age_us` | `500000U` | us | Fixed profile |
| `startup::maximum_horizontal_speed_m_s` | `0.8f` | m/s | Fixed profile |
| `startup::maximum_vertical_speed_m_s` | `0.5f` | m/s | Fixed profile |
| `startup::position_variance` | `{1., 1., 2.25}` | m², N/E/D | Fixed profile |
| `startup::velocity_variance` | `{0.09, 0.09, 0.09}` | m²/s², N/E/D | Fixed profile |
| `startup::attitude_variance` | `{0.01, 0.01, 0.04}` | rad², body-local axes | Fixed profile |
| `startup::accelerometer_bias_variance` | `0.04` | m²/s⁴ | Fixed profile |
| `startup::gyroscope_bias_variance` | `1e-4` | rad²/s² | Fixed profile |
| `imu::minimum_interval_s` | `1e-6` | s | Fixed profile |
| `imu::maximum_interval_s` | `0.05` | s | Fixed profile |
| `imu::nominal_interval_s` | `0.004` | s | Fixed profile |
| `imu::maximum_interval_us` | `50000U` | us | Fixed profile |
| `imu::gap_tolerance_us` | `200` | us | Fixed profile |
| `imu::rest_maximum_age_us` | `2000000U` | us | Fixed profile |
| `imu::interval_history_weight` | `0.99` | dimensionless | Fixed profile |
| `imu::interval_sample_weight` | `0.01` | dimensionless | Fixed profile |
| `imu::minimum_interval_ratio` | `0.5` | dimensionless | Fixed profile |
| `imu::maximum_interval_ratio` | `2.` | dimensionless | Fixed profile |
| `imu::specific_force_variance` | `1.` | m²/s⁴ per sample | Candidate startup setting |
| `imu::angular_rate_variance` | `0.0025` | rad²/s² per sample | Candidate startup setting |
| `imu::faulty_specific_force_variance` | `4.9 * 4.9` | m²/s⁴ per sample | Candidate startup setting |
| `imu::accelerometer_bias_variance_density` | `1e-6` | m²/s⁵ | Candidate startup setting |
| `imu::gyroscope_bias_variance_density` | `1e-8` | rad²/s³ | Candidate startup setting |
| `imu::maximum_specific_force_m_s2` | `25.` | m/s² including gravity | Candidate startup setting |
| `imu::maximum_angular_rate_rad_s` | `3.` | rad/s | Candidate startup setting |
| `imu::bias_learning_decay_s` | `0.5` | s | Candidate startup setting |
| `history::filter_interval_us` | `4000` | us | Existing startup default; fixed supported envelope |
| `history::fusion_delay_ms` | `200` | ms | Existing startup default; fixed supported envelope |
| `history::minimum_period_us` | `3800` | us | Fixed profile |
| `history::maximum_period_us` | `4200` | us | Fixed profile |
| `history::maximum_lag_us` | `300000` | us | Fixed profile |
| `history::gnss_arrival_age_us` | `150000` | us | Fixed profile |
| `history::magnetic_arrival_age_us` | `100000` | us | Fixed profile |
| `history::barometer_arrival_age_us` | `100000` | us | Fixed profile |
| `history::imu_capacity` | `64` | records per backing history | Fixed profile |
| `history::context_capacity` | `128` | records | Fixed profile |
| `history::gnss_capacity` | `16` | records | Fixed profile |
| `history::magnetic_capacity` | `64` | records | Fixed profile |
| `history::barometer_capacity` | `128` | records | Fixed profile |
| `history::observation_maximum_age_us` | `100000` | us | Fixed profile |
| `history::gnss_spacing_us` | `50000` | us | Fixed profile |
| `history::magnetic_spacing_us` | `10000` | us | Fixed profile |
| `history::barometer_spacing_us` | `5000` | us | Fixed profile |
| `history::storage_budget_bytes` | `64 * 1024` | bytes; selected history storage, not whole module RAM | Fixed profile |
| `gravity::observation_interval_us` | `10000U` | us, compile-time sampler target | Fixed profile |
| `magnetic::field` | `{20., std::numbers::pi / 9., 0., 5.}` | strength µT / inclination rad / heading rad / minimum horizontal µT | Candidate startup setting |
| `magnetic::checks` | `{true, false, false}` | strength / inclination / require reference | Candidate startup setting |
| `magnetic::timing` | `{1000000U, 500000U, 7000000U}` | health / maximum interval / fusion timeout, us | Fixed profile |
| `magnetic::heading_noise_rad` | `0.3` | rad | Candidate startup setting |
| `magnetic::acceleration_limit_m_s2` | `0.5` | m/s² | Candidate startup setting |
| `magnetic::noise_variance` | `25.` | µT² | Candidate startup setting |
| `magnetic::gate_sigma` | `3.` | standard deviations | Candidate startup setting |
| `magnetic::maximum_heading_variance` | `0.1` | rad² | Fixed profile |
| `magnetic::maximum_heading_age_us` | `1000000U` | us | Fixed profile |
| `magnetic::context_maximum_age_us` | `3000000U` | us | Fixed profile |
| `magnetic::model_interval_us` | `10000000U` | us | Fixed profile |
| `gnss::timing` | `{7000000U, 1000000U, 5000000U}` | horizontal / height-data / height-fusion timeout, us | Fixed profile |
| `gnss::maximum_sample_age_us` | `500000U` | us | Fixed profile |
| `gnss::context_maximum_age_us` | `3000000U` | us | Fixed profile |
| `gnss::maximum_aiding_age_us` | `5000000U` | us | Fixed profile |
| `gnss::minimum_velocity_noise_m_s` | `0.3` | m/s, standard deviation | Candidate startup setting |
| `gnss::vertical_variance_multiplier` | `2.25` | dimensionless | Fixed profile |
| `gnss::minimum_position_noise_m` | `0.5` | m, standard deviation | Candidate startup setting |
| `gnss::maximum_position_noise_m` | `10.` | m, standard deviation | Candidate startup setting |
| `gnss::gate_sigma` | `5.` | standard deviations | Candidate startup setting |
| `height::fusion_timeout_us` | `5000000U` | us | Fixed profile |
| `height::barometer_timeout_us` | `400000U` | us | Fixed profile |
| `height::gnss_timeout_us` | `1000000U` | us | Fixed profile |
| `height::barometer_maximum_age_us` | `200000U` | us | Fixed profile |
| `height::gnss_maximum_age_us` | `500000U` | us | Fixed profile |
| `height::minimum_altitude_m` | `-12000.` | m MSL | Fixed profile |
| `height::maximum_altitude_m` | `100000.` | m MSL | Fixed profile |
| `height::barometer_variance` | `3.5 * 3.5` | m² | Candidate startup setting |
| `height::minimum_gnss_noise_m` | `0.75` | m, standard deviation | Candidate startup setting |
| `height::maximum_aided_gnss_variance` | `100.` | m² | Candidate startup setting |
| `height::gate_sigma` | `5.` | standard deviations | Candidate startup setting |
| `height::barometer_lpf_gain` | `0.1` | dimensionless | Fixed profile |
| `height::barometer_warmup_samples` | `20U` | samples | Fixed profile |
| `height::reset_velocity_variance` | `10.` | m²/s² | Fixed profile |
| `height::ground_effect_deadzone_m` | `4.` | m | Candidate startup setting |
| `height::ground_effect_timeout_us` | `10000000U` | us | Fixed profile |
| `height::context_maximum_age_us` | `3000000U` | us | Fixed profile |
| `height_bias::initial_maximum_variance` | `2.` | m² | Fixed profile |
| `height_bias::initial_interval_s` | `0.01` | s | Fixed profile |
| `height_bias::process_variance_density` | `0.13 * 0.13` | m²/s | Fixed profile |
| `height_bias::minimum_variance` | `1e-8` | m² | Fixed profile |
| `height_bias::measurement_variance_floor` | `1e-4` | m² | Fixed profile |
| `height_bias::gate_squared` | `9.` | dimensionless, squared sigma | Fixed profile |
| `height_bias::offset_ratio` | `0.2` | dimensionless | Fixed profile |
| `height_bias::offset_duration_s` | `10.` | s | Fixed profile |
| `height_bias::offset_variance_multiplier` | `1000.` | dimensionless | Fixed profile |
| `height_bias::ratio_filter_time_s` | `10.` | s | Fixed profile |
| `output::maximum_bias_variance_norm` | `0.1f` | bias-variance vector norm; accel m²/s⁴ or gyro rad²/s² | Fixed profile |
| `output::preflight_innovation_ratio` | `0.5f` | dimensionless | Fixed profile |
| `output::innovation_time_constant_s` | `0.5` | s | Fixed profile |
| `output::innovation_minimum_interval_s` | `0.001` | s | Fixed profile |
| `output::innovation_maximum_interval_s` | `1.` | s | Fixed profile |
| `output::innovation_ratio_limit` | `4.` | dimensionless squared ratio | Fixed profile |
| `output::velocity_time_constant_s` | `0.25` | s | Fixed profile |
| `output::position_time_constant_s` | `0.25` | s | Fixed profile |
| `output::interval_history_weight` | `0.8` | dimensionless | Fixed profile |
| `output::interval_sample_weight` | `0.2` | dimensionless | Fixed profile |
| `output::attitude_tracking_gain` | `0.5` | dimensionless, divided by lag for rate correction | Fixed profile |
| `output::integral_tracking_gain` | `0.1` | dimensionless | Fixed profile |
| `output::navigation_interval_us` | `10000U` | us | Fixed profile |
| `output::maximum_age_us` | `100000U` | us | Fixed profile |
| `output::dead_reckoning_age_us` | `1000000U` | us | Fixed profile |
| `work_queue_stack_bytes` | `60U * 1024U` | bytes, SITL reservation only | Fixed profile |
| `work_queue_relative_priority` | `-15` | relative scheduler priority | Fixed profile |
| `backup_interval_us` | `20000U` | us | Fixed profile |
| `status_interval_us` | `100000U` | us | Fixed profile |

The two weight pairs (`imu` 0.99/0.01 and `output` 0.8/0.2) each sum to one.
The `_s` / `_us` interval limits are related representations; shared time/noise
contracts, not independently adjustable copies. Magnetic `field` and `timing`
aggregates map to the explicitly named runtime fields above.

## Appendix: replay source macros

Source expressions are preserved below; these do not override the PX4 profile.

| Macro | Current expression | Meaning / unit from source |
| --- | --- | --- |
| `GRAVITY` | `9.80665` | m/s^2, positive Down in NED. |
| `QUAT_MIN_NORM` | `1e-6` | Same parameter family above; explicit replay default |
| `QUAT_SQUARED_NORM_TOL` | `1e-12` | Same parameter family above; explicit replay default |
| `DT_MIN` | `1e-6` | s. |
| `DT_MAX` | `0.05` | s. |
| `ACCEL_VAR` | `1.0` | m^2/s^4. |
| `GYRO_VAR` | `4e-4` | rad^2/s^2. |
| `BAD_ACCEL_STD` | `4.9` | m/s^2; sampled accelerometer noise during faults/clipping. |
| `ACCEL_BIAS_RW_VAR_DENSITY` | `1e-6` | m^2/s^5. |
| `GYRO_BIAS_RW_VAR_DENSITY` | `1e-8` | rad^2/s^3. |
| `BIAS_MAX_SPECIFIC_FORCE` | `25.0` | m/s^2, including gravity response. |
| `BIAS_MAX_ANGULAR_RATE` | `3.0` | rad/s. |
| `BIAS_DECAY_TIME_CONSTANT` | `0.5` | s. |
| `ACCEL_BIAS_LIMIT` | `0.4` | m/s^2, per axis. |
| `GYRO_BIAS_LIMIT` | `0.15` | rad/s, per axis. |
| `ACCEL_OBS_STD` | `GRAVITY` | m/s^2; effective pseudo-observation weight. |
| `ACCEL_OBS_PERIOD_US` | `10000U` | Nominal; accumulate whole IMU samples. |
| `MAG_STRENGTH_TOL` | `20.0` | microtesla. |
| `MAG_INCLINATION_TOL` | `20.0` | degrees. |
| `MAG_MIN_HORIZONTAL` | `5.0` | microtesla; weak-heading guard, not an observability proof. |
| `MAG_HEALTH_TIME_US` | `1000000U` | Same parameter family above; explicit replay default |
| `MAG_MAX_INTERVAL_US` | `500000U` | Startup freshness; no-data stop uses twice this. |
| `MAG_FUSION_TIMEOUT_US` | `7000000U` | Same parameter family above; explicit replay default |
| `MAG_HEADING_STD` | `0.3` | rad; legacy input default, also used for heading consistency. |
| `MAG_ACCEL_LIMIT` | `0.5` | m/s^2; legacy input default. |
| `GNSS_HORIZONTAL_TIMEOUT_US` | `7000000U` | Same parameter family above; explicit replay default |
| `GNSS_HEIGHT_DATA_TIMEOUT_US` | `1000000U` | Same parameter family above; explicit replay default |
| `GNSS_HEIGHT_FUSION_TIMEOUT_US` | `5000000U` | Same parameter family above; explicit replay default |
| `POS_XY_STD_MIN` | `1.0` | m. |
| `POS_Z_STD_MIN` | `1.5` | m. |
| `VEL_STD_MIN` | `0.3` | m/s. |
| `INIT_ATTITUDE_XY_VAR` | `0.04` | rad^2, body X/Y. |
| `INIT_ATTITUDE_Z_VAR` | `0.25` | rad^2, body Z. |
| `INIT_ACCEL_BIAS_VAR` | `0.25` | m^2/s^4. |
| `INIT_GYRO_BIAS_VAR` | `0.0025` | rad^2/s^2. |
