# Specification Index

Read the relevant requirement and its declared dependencies. Directory placement
identifies ownership, not requirement derivation. Requirement definitions, design
explanations, deployment profiles and verification policy have separate owners.

Use [TEMPLATE.md](TEMPLATE.md) when authoring requirements or architecture.
Each requirement's `Status` records Draft or Approved under the
[approval rules](TEMPLATE.md#requirement-status), independently of verification results.
The [traceability map](../../formal/agent-review/traceability-map.json) registers
verification scope; an indexed requirement or linked proof is not a passing result.

Some existing review rows aggregate several contracts. Their `spec` references
must retain every extracted scope; splitting a file does not increase the passing
count. For example, `R-ACCELEROMETER-FUSION` still registers fusion, sampling and
caller bindings separately identified below. Execution results retain their own
source/configuration identity and must be assessed against the claimed scope.

## Requirement hierarchy

Start with the relevant software capability, then review its Decomposition table
and linked children. Each child records its Parent; Uses records dependency,
not derivation. The layer index below locates the same canonical contracts.

| Software requirement | Complete behavior to review |
|---|---|
| [SW-CORE](requirements/software/core-estimation.md) | Portable prediction, correction and coherent state/covariance transactions. |
| [SW-GNSS](requirements/software/gnss-aiding.md) | GNSS qualification, permission, actual update and recovery feedback. |
| [SW-GRAVITY](requirements/software/gravity-aiding.md) | Real-interval observation preparation and conditional gravity correction. |
| [SW-MAGNETIC](requirements/software/magnetic-aiding.md) | Fixed-field qualification, correction and heading recovery. |
| [SW-IMU](requirements/software/imu-protection.md) | Consistent IMU-health, process-noise and bias-learning controls. |
| [SW-PX4](requirements/software/px4-estimation.md) | Delayed estimation, current-time output and bounded platform execution. |
| [SW-REPLAY](requirements/software/replay-aiding.md) | Recorded sensor-policy decisions and actual replay outcomes. |

These are software-level obligations with explicit scope, not aircraft-level
requirements. Most children are direct contracts. E-PRED, E-OBS and E-STEP retain
their existing subrequirements under SW-CORE; those meaningful subgroups do not
require equivalent extra levels elsewhere.

Common and mathematical requirements remain shared contracts. E-STATE,
R-MEASUREMENT-FUSION, R-RING-BUFFER and R-TILT-ALIGNMENT also remain shared roots:
state conventions, a generic gated transaction, storage and alignment history
serve multiple callers.
Their absence of a Parent does not waive evidence obligations. Integration
requirements use portable contracts without becoming their children; in particular,
PX4 two-source height management does not inherit the GNSS-only height profile.

Each parent adds composition acceptance, not a new proof result. Preserve the
registered evidence scopes and assess parent completeness separately; parent and
child counts must not be combined into a verification percentage. All approval
decisions remain governed by each requirement's Status.

## Common requirements

| ID | Required behavior |
|---|---|
| `C-CPP-SUBSET` | [C++ language and portability subset](requirements/common/cpp-subset.md) |
| `C-FAILURE` | [Failure reporting and atomicity](requirements/common/failure.md) |
| `C-INPUT-DOMAIN` | [Input-domain and caller obligations](requirements/common/input-domain.md) |
| `C-LAYER-BOUNDARY` | [Layer ownership and dependency boundaries](requirements/common/layer-boundaries.md) |
| `C-NUMERICAL-ACCEPTANCE` | [Numerical residual acceptance](requirements/common/numerical-acceptance.md) |
| `C-NUMERICAL-PROFILE` | [Numerical execution profile](requirements/common/numerical-profile.md) |

## Math requirements

| ID | Required behavior |
|---|---|
| `F-LINALG` | [Fixed-size linear algebra operations](requirements/math/linalg/operations.md) |
| `F-SOLVE` | [Symmetric positive-definite linear solve](requirements/math/linalg/spd-solve.md) |
| `Q-ROT-COMPOSE` | [Rotation composition](requirements/math/quaternion/composition.md) |
| `Q-IDENTITY` | [Identity](requirements/math/quaternion/identity.md) |
| `Q-INVERSE` | [Inverse](requirements/math/quaternion/inverse.md) |
| `Q-MUL` | [Hamilton multiplication](requirements/math/quaternion/multiplication.md) |
| `Q-NORMALIZE` | [Normalization](requirements/math/quaternion/normalization.md) |
| `Q-ROTATE` | [Rotation action](requirements/math/quaternion/rotation-action.md) |
| `Q-ROT-MATRIX` | [Quaternion rotation matrix](requirements/math/quaternion/rotation-matrix.md) |
| `Q-SIGN` | [Quaternion sign equivalence](requirements/math/quaternion/sign-equivalence.md) |
| `F-SCALAR` | [Checked scalar operations](requirements/math/scalar/operations.md) |
| `SO3-EXP` | [Exponential map](requirements/math/so3/exp.md) |
| `SO3-HAT` | [Hat operator](requirements/math/so3/hat.md) |
| `SO3-LOG` | [Logarithmic map](requirements/math/so3/log.md) |
| `F-JR` | [SO(3) right Jacobian](requirements/math/so3/right-jacobian.md) |
| `SO3-SMALL-ANGLE` | [Small-angle behavior](requirements/math/so3/small-angle.md) |

### Proposed geodesy extension

These Draft contracts define a proposed utility outside the ESKF state model.
They have no production, native-test or formal-proof bindings yet and do not
extend the existing core/runtime test-plan or executable review-map denominator.

| ID | Required behavior |
|---|---|
| `F-GEO-ECEF` | [WGS84 geodetic and ECEF position conversion](requirements/math/geodesy/geodetic-ecef.md) |
| `F-GEO-NED` | [Fixed-origin ECEF and NED transformations](requirements/math/geodesy/local-ned.md) |

## Core requirements

| ID | Required behavior |
|---|---|
| `E-PRED-ATTITUDE` | [Attitude prediction](requirements/core/attitude-prediction.md) |
| `E-BIAS-UPDATE` | [INS bias-update control](requirements/core/bias-update.md) |
| `E-STEP` | [Atomic composed operations](requirements/core/composed-steps.md) |
| `E-STEP-CORRECT` | [Complete correction transaction](requirements/core/correction-step.md) |
| `E-CORRECT` | [Measurement correction and Joseph covariance](requirements/core/correction.md) |
| `E-PCOV` | [Error covariance prediction](requirements/core/covariance-prediction.md) |
| `E-RESET` | [Post-injection covariance coordinates](requirements/core/covariance-reset.md) |
| `E-OBS-ACCEL` | [gravity observation with fixed bias preprocessing](requirements/core/gravity-observation.md) |
| `E-STEP-INJECT-RESET` | [Combined injection and reset](requirements/core/injection-reset-step.md) |
| `E-INJECT` | [Nominal error injection](requirements/core/injection.md) |
| `E-INNOV` | [componentwise innovation gate](requirements/core/innovation.md) |
| `E-OBS-MAGNETIC` | [Fixed-reference magnetic observation](requirements/core/magnetic-observation.md) |
| `E-MAG-TILT` | [magnetic tilt protection](requirements/core/magnetic-tilt.md) |
| `E-OBS` | [Observation conventions and model ownership](requirements/core/observations.md) |
| `E-OBS-POSITION` | [Navigation-frame position observation](requirements/core/position-observation.md) |
| `E-STEP-PRED` | [Combined prediction](requirements/core/prediction-step.md) |
| `E-PRED` | [Nominal prediction](requirements/core/prediction.md) |
| `E-NOISE` | [Discrete process-noise covariance](requirements/core/process-noise.md) |
| `E-STATE` | [State, frames and perturbation convention](requirements/core/state.md) |
| `E-PRED-INS` | [INS translation prediction](requirements/core/translation-prediction.md) |
| `E-OBS-VELOCITY` | [Navigation-frame velocity observation](requirements/core/velocity-observation.md) |

## Runtime requirements

| ID | Required behavior |
|---|---|
| `R-GNSS-FUSION` | [GNSS fusion lifecycle](requirements/runtime/gnss/lifecycle.md) |
| `R-GNSS-QUALITY` | [GNSS quality checks and health qualification](requirements/runtime/gnss/quality.md) |
| `R-GNSS-RESET` | [checked GNSS position and velocity reset](requirements/runtime/gnss/reset.md) |
| `R-GNSS-VERTICAL-RECOVERY` | [bounded vertical innovations during IMU faults](requirements/runtime/gnss/vertical-recovery.md) |
| `R-ACCELEROMETER-CONDITIONS` | [gravity-fusion prerequisites](requirements/runtime/gravity/conditions.md) |
| `R-ACCELEROMETER-FUSION` | [conditional gravity correction](requirements/runtime/gravity/fusion.md) |
| `R-ACCELEROMETER-MAGNITUDE` | [filtered gravity-band diagnostic](requirements/runtime/gravity/magnitude.md) |
| `R-ACCELEROMETER-SAMPLING` | [Rotation-aware gravity observation sampling](requirements/runtime/gravity/sampling.md) |
| `R-IMU-BIAS-INHIBITION` | [Decaying-peak bias-learning permissions](requirements/runtime/imu/bias-inhibition.md) |
| `R-BIAS-LIMITS` | [per-axis bias-value limits](requirements/runtime/imu/bias-limits.md) |
| `R-BIAS-PROCESS-NOISE` | [variance-dependent bias driving noise](requirements/runtime/imu/bias-process-noise.md) |
| `R-IMU-PROCESS-NOISE` | [fault-dependent sampled IMU variance](requirements/runtime/imu/process-noise.md) |
| `R-VERTICAL-ACCELERATION` | [apparent falling and clipping detection](requirements/runtime/imu/vertical-health.md) |
| `R-MAGNETIC-FIELD` | [instantaneous magnetic-field plausibility](requirements/runtime/magnetic/field.md) |
| `R-MAGNETOMETER-FUSION` | [field checks and gated magnetic correction](requirements/runtime/magnetic/fusion.md) |
| `R-MAGNETOMETER-RESET` | [fixed-field heading and covariance reset](requirements/runtime/magnetic/heading-reset.md) |
| `R-MAGNETIC-HEADING` | [aided heading consistency](requirements/runtime/magnetic/heading.md) |
| `R-MAGNETOMETER-LIFECYCLE` | [magnetic health and fusion lifecycle](requirements/runtime/magnetic/lifecycle.md) |
| `R-MAGNETIC-REFERENCE` | [geographic magnetic reference adoption](requirements/runtime/magnetic/reference.md) |
| `R-MAGNETOMETER-FILTER` | [magnetic data for heading reset](requirements/runtime/magnetic/reset-filter.md) |
| `R-MEASUREMENT-FUSION` | [gated correction of one observation group](requirements/runtime/measurement-fusion.md) |
| `R-RING-BUFFER` | [Fixed-capacity timestamped history](requirements/runtime/ring-buffer.md) |
| `R-TILT-ALIGNMENT` | [Covariance-based tilt alignment latch](requirements/runtime/tilt-alignment.md) |

## Integration requirements

| ID | Required behavior |
|---|---|
| `PX4-VERIFY` | [acceptance and change control](requirements/integration/px4/acceptance.md) |
| `R-PX4-BARO-GROUND-EFFECT` | [PX4 module barometer ground effect](requirements/integration/px4/barometer-ground-effect.md) |
| `PX4-CONFIG` | [packaged build profile](requirements/integration/px4/configuration.md) |
| `R-PX4-DELAYED-FUSION` | [live module timeline](requirements/integration/px4/delayed-fusion.md) |
| `PX4-EXEC` | [execution and data flow](requirements/integration/px4/execution.md) |
| `R-PX4-GNSS-QUALITY` | [PX4 GNSS integration](requirements/integration/px4/gnss-quality.md) |
| `R-PX4-GRAVITY` | [Delayed gravity-aiding binding](requirements/integration/px4/gravity-aiding.md) |
| `R-HEIGHT-FUSION` | [PX4 GNSS/barometer height management](requirements/integration/px4/height-fusion.md) |
| `R-PX4-IMU-BIAS` | [module health and bias-learning inhibition](requirements/integration/px4/imu-bias.md) |
| `R-PX4-IMU-HISTORY` | [delayed IMU timeline](requirements/integration/px4/imu-history.md) |
| `PX4-LIFE` | [initialization, failure and shutdown](requirements/integration/px4/lifecycle.md) |
| `R-PX4-MAGNETIC-AIDING` | [PX4 module magnetic aiding](requirements/integration/px4/magnetic-aiding.md) |
| `R-PX4-MAGNETIC-MODEL` | [PX4 module geographic magnetic reference](requirements/integration/px4/magnetic-model.md) |
| `R-PX4-OBSERVATION-HISTORY` | [delayed observations and context](requirements/integration/px4/observation-history.md) |
| `R-PX4-FRAME-OUTPUT` | [geographic and rotation-variance output frames](requirements/integration/px4/output-frames.md) |
| `R-PX4-OUTPUT-PREDICTOR` | [current-time output tracking](requirements/integration/px4/output-predictor.md) |
| `PX4-OWN` | [ownership and interfaces](requirements/integration/px4/ownership.md) |
| `PX4-OUT` | [output prediction and publication](requirements/integration/px4/publication.md) |
| `PX4-RT` | [real-time constraints](requirements/integration/px4/real-time.md) |
| `R-PX4-SENSOR-POSITIONS` | [sensor and output reference points](requirements/integration/px4/sensor-positions.md) |
| `R-PX4-STATUS` | [confidence and failure reporting](requirements/integration/px4/status.md) |
| `PX4-TIME` | [timestamps and bounded history](requirements/integration/px4/timestamps.md) |
| `R-REPLAY-GNSS-GATING` | [Replay GNSS gating](requirements/integration/replay/gnss-gating.md) |
| `R-REPLAY-GNSS-LIFECYCLE` | [Replay GNSS lifecycle](requirements/integration/replay/gnss-lifecycle.md) |
| `R-REPLAY-GNSS-QUALITY` | [Replay GNSS quality](requirements/integration/replay/gnss-quality.md) |
| `R-REPLAY-GRAVITY-CONDITIONS` | [Replay condition sources (host demo)](requirements/integration/replay/gravity-conditions.md) |
| `R-REPLAY-GRAVITY-SAMPLING` | [Replay sampling and tuning](requirements/integration/replay/gravity-sampling.md) |
| `R-REPLAY-BIAS` | [Replay bias-learning binding](requirements/integration/replay/imu-bias.md) |
| `R-REPLAY-MAGNETIC-FIELD` | [Replay magnetic field checks](requirements/integration/replay/magnetic-field.md) |
| `R-REPLAY-MAGNETIC-LIFECYCLE` | [Replay magnetic lifecycle](requirements/integration/replay/magnetic-lifecycle.md) |
| `R-REPLAY-MAGNETIC-MODEL` | [Replay geographic magnetic model](requirements/integration/replay/magnetic-model.md) |
| `R-REPLAY-MAGNETIC-OBSERVATION` | [Replay magnetic observations](requirements/integration/replay/magnetic-observation.md) |

## Architecture

- [Numerical Architecture](architecture/numerics.md).
- [Software Architecture and Verification Boundaries](architecture/overview.md).
- [PX4 Module Architecture](architecture/px4-module.md).
- [Geodesy Architecture and Minimal API](architecture/geodesy.md) (Draft proposal).

## Deployment profiles

- [PX4 SIH 250 Hz Deployment Profile](profiles/px4-sih-250hz.md).

## Verification policy

- [Verification and Traceability Policy](verification/assurance.md).
- [Test Specification and Coverage Standard](verification/testing.md) (Draft), with the [core/runtime test design](verification/tests/README.md).
- [C++ Function Inventory and Review Boundary](verification/source-inventory.md).
- [C++ Static Analysis Policy](verification/static-analysis.md).


## Identity and scope

One canonical file owns each independently reviewable requirement. Named child
clauses may remain in that file when they are one operation's conditions or
transaction frame; file and clause counts are not proof counts. Parent acceptance
requires all applicable children. The current map preserves its registered IDs;
newly separated caller/shared contracts do not automatically add passing rows.
An unmapped requirement remains an obligation whose evidence must be registered
before claiming its verification.

### Legacy reference headings

These former document headings identify reference scopes, not additional active
requirements. Follow the owners below; the scopes are not interchangeable.

| Former heading | Canonical owners |
|---|---|
| `PX4-TIME-BUFFER` | [R-RING-BUFFER](requirements/runtime/ring-buffer.md) owns storage; [PX4-TIME](requirements/integration/px4/timestamps.md) owns caller/history use; [PX4-RT](requirements/integration/px4/real-time.md) owns resource acceptance. |
| `PX4-TIME-IMU` | [R-PX4-IMU-HISTORY](requirements/integration/px4/imu-history.md) |
| `PX4-TIME-OUTPUT` | [R-PX4-OUTPUT-PREDICTOR](requirements/integration/px4/output-predictor.md) |
| `PX4-TIME-OBSERVATIONS` | [R-PX4-OBSERVATION-HISTORY](requirements/integration/px4/observation-history.md) |
| `PX4-TIME-LIVE` | [R-PX4-DELAYED-FUSION](requirements/integration/px4/delayed-fusion.md), with [PX4-EXEC](requirements/integration/px4/execution.md) and [PX4-OUT](requirements/integration/px4/publication.md). |

### Reserved retired IDs

`R-IMU-DYNAMICS` (standalone instantaneous detector) and `R-IMU-BIAS-LEARNING`
(continuous-good qualification) are retired. Their contracts remain in Git at
`7029404`; do not reuse their IDs or convert their missing evidence into PASS.
The active [decaying-peak policy](requirements/runtime/imu/bias-inhibition.md) is
not a behaviorally equivalent replacement for either retired public API.
