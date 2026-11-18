## Summary

This specification records the required behavior of the numerical core, its supporting operations, portable runtime policies and explicitly scoped replay/PX4 integration paths. A requirement applies only to the layer and configurations named in its definition; a core proof does not automatically cover its runtime or adapter callers.

Each ID identifies a behavior to verify, not a C++ function or a single theorem. Mathematical definitions and assumptions come from papers and design specifications. Detailed clauses define inputs, decisions, success/failure behavior and exclusions. Entries with only a scope description still need those clauses before acceptance; registration does not supply a missing contract.

Evidence mappings belong to the [traceability map](../../formal/agent-review/traceability-map.json); execution results and gaps belong to validated review reports. This list records neither proof status nor completion.

## Reading this specification

| Concern | Requirements |
|---|---|
| State, prediction, correction and covariance | [ESKF core](#eskf-core), with detailed [prediction](requirements/core/prediction.md#e-pred-nominal-prediction) and [composition](requirements/core/composed-steps.md#e-step-atomic-composed-operations) clauses |
| Magnetic observations, health and recovery | [Magnetic aiding](#magnetic-aiding), including explicitly named replay and PX4 module profiles |
| GNSS, height, gravity and IMU/bias policies | [Portable runtime](#portable-runtime), including explicitly named deployment profiles |
| PX4 module target architecture | [Module design clauses](architecture/px4-module.md), covering ownership, execution, timing, lifecycle, output and real-time acceptance; distinct from the currently mapped integration profiles |
| Rotation mathematics | [Quaternion and SO(3)](#quaternion-and-so3) |
| Scalar and linear-algebra dependencies | [Supporting operations](#supporting-operations) |

Requirement IDs remain stable when documentation is reorganized. Child clauses refine their parent requirement; they are not extra passing requirements. Follow the [review criteria](formal-verification-traceability-and-review-criteria.md) for acceptance and change-impact review. Implementation or evidence limitations recorded below constrain claims; they do not waive the required behavior.

## ESKF core

- `E-STATE` — [State, frames and perturbation convention](requirements/core/state.md).
- `E-PRED` — Nominal-state propagation from the prior state, with applicable bias compensation and the selected paper-based integration and attitude models.
- `E-NOISE` — [Discrete process-noise covariance](requirements/core/process-noise.md).
- `E-PCOV` — [Error covariance prediction](requirements/core/covariance-prediction.md).
- `E-INJECT` — [Nominal error injection](requirements/core/injection.md).
- `E-RESET` — [Post-injection covariance coordinates](requirements/core/covariance-reset.md).
- `E-CORRECT` — [Measurement correction and Joseph covariance](requirements/core/correction.md).
- `E-BIAS-UPDATE` — Explicit per-axis INS bias-update permissions applied consistently to the correction vector and Joseph covariance.
- `E-MAG-TILT` — Optional magnetic attitude-gain projection onto navigation Down, with consistent correction and Joseph covariance.
- `E-INNOV` — Componentwise innovation gate and checked, failure-atomic diagnostics, separate from correction.
- `E-OBS` — [Observation conventions and model ownership](requirements/core/observations.md).
- `E-STEP` — Composition of nominal/covariance prediction and correction/injection/reset, preserving component preconditions and committing all outputs only when the whole operation succeeds.

### E-PRED: nominal prediction requirements

See [prediction.md](requirements/core/prediction.md).

### E-STEP: composed-operation requirements

See [composed-steps.md](requirements/core/composed-steps.md).

### E-OBS-ACCEL: gravity observation with fixed bias preprocessing

See [gravity-observation.md](requirements/core/gravity-observation.md).

### E-BIAS-UPDATE: INS bias-update control

See [bias-update.md](requirements/core/bias-update.md).

### E-MAG-TILT: magnetic tilt protection

See [magnetic-tilt.md](requirements/core/magnetic-tilt.md).

### E-INNOV: componentwise innovation gate

See [innovation.md](requirements/core/innovation.md).

## Magnetic aiding

This section groups magnetic runtime policies and their explicitly scoped replay/PX4 callers. The numerical observation and gain-projection requirements remain `E-OBS` and `E-MAG-TILT`; sharing this section does not combine their proof claims.

### R-MAGNETIC-FIELD: instantaneous magnetic-field plausibility

See [field.md](requirements/runtime/magnetic/field.md).

### R-MAGNETIC-HEADING: aided heading consistency

See [heading.md](requirements/runtime/magnetic/heading.md).

### R-MAGNETIC-REFERENCE: geographic magnetic reference adoption

See [reference.md](requirements/runtime/magnetic/reference.md).

### R-MAGNETOMETER-FUSION: field checks and gated magnetic correction

See [fusion.md](requirements/runtime/magnetic/fusion.md).

### R-MAGNETOMETER-LIFECYCLE: magnetic health and fusion lifecycle

See [lifecycle.md](requirements/runtime/magnetic/lifecycle.md).

### R-MAGNETOMETER-FILTER: magnetic data for heading reset

See [reset-filter.md](requirements/runtime/magnetic/reset-filter.md).

### R-MAGNETOMETER-RESET: fixed-field heading and covariance reset

See [heading-reset.md](requirements/runtime/magnetic/heading-reset.md).

## Portable runtime

### R-RING-BUFFER: fixed-storage timestamped history

See [ring-buffer.md](requirements/runtime/ring-buffer.md).

### R-PX4-IMU-HISTORY: delayed IMU timeline

See [imu-history.md](requirements/integration/px4/imu-history.md).

### R-PX4-OUTPUT-PREDICTOR: current-time output tracking

See [output-predictor.md](requirements/integration/px4/output-predictor.md).

### R-PX4-SENSOR-POSITIONS: sensor and output reference points

See [sensor-positions.md](requirements/integration/px4/sensor-positions.md).

### R-PX4-FRAME-OUTPUT: geographic and rotation-variance output frames

See [output-frames.md](requirements/integration/px4/output-frames.md).

### R-PX4-STATUS: confidence and failure reporting

See [status.md](requirements/integration/px4/status.md).

### R-PX4-OBSERVATION-HISTORY: delayed observations and context

See [observation-history.md](requirements/integration/px4/observation-history.md).

### R-PX4-DELAYED-FUSION: live module timeline

See [delayed-fusion.md](requirements/integration/px4/delayed-fusion.md).

### R-GNSS-RESET: checked GNSS position and velocity reset

See [reset.md](requirements/runtime/gnss/reset.md).

### R-GNSS-FUSION: GNSS fusion lifecycle

See [lifecycle.md](requirements/runtime/gnss/lifecycle.md).

### R-MEASUREMENT-FUSION: gated correction of one observation group

See [measurement-fusion.md](requirements/runtime/measurement-fusion.md).

### R-GNSS-VERTICAL-RECOVERY: bounded vertical innovations during IMU faults

See [vertical-recovery.md](requirements/runtime/gnss/vertical-recovery.md).

### R-ACCELEROMETER-MAGNITUDE: filtered gravity-band diagnostic

See [magnitude.md](requirements/runtime/gravity/magnitude.md).

### R-ACCELEROMETER-CONDITIONS: gravity-fusion prerequisites

See [conditions.md](requirements/runtime/gravity/conditions.md).

### R-ACCELEROMETER-FUSION: conditional gravity correction

See [fusion.md](requirements/runtime/gravity/fusion.md).

### R-IMU-DYNAMICS: instantaneous IMU magnitude check

**Retired on 2026-11-14.** The standalone `check_imu_dynamics` API and its
exclusive tests were removed with the timed qualification path. In-tree module
and replay callers use `ImuBiasInhibition`; neither consumed this detector.
The original contract remains in Git history at `7029404`.

### R-IMU-BIAS-LEARNING: bias-learning qualification

**Retired on 2026-11-14.** The `ImuBiasLearning` API, six continuous-good timers
and their exclusive tests were removed. The deployed/replayed decaying-peak
policy remains specified by `R-REPLAY-BIAS` and `R-PX4-IMU-BIAS`; it is not a
behaviorally equivalent replacement for the retired public API. The original
contract remains in Git history at `7029404`.

Both IDs are reserved historical records and excluded from the active
traceability map. Their missing Lean/ESBMC evidence was not completed or converted
to PASS. Retiring these APIs changes supported library scope; it leaves current
module/replay behavior and every retained proof obligation unchanged.

### R-REPLAY-BIAS: replay bias-learning inhibition

See [imu-bias.md](requirements/integration/replay/imu-bias.md).

### R-PX4-IMU-BIAS: module health and bias-learning inhibition

See [imu-bias.md](requirements/integration/px4/imu-bias.md).

### R-IMU-PROCESS-NOISE: fault-dependent sampled IMU variance

See [process-noise.md](requirements/runtime/imu/process-noise.md).

### R-BIAS-PROCESS-NOISE: variance-dependent bias driving noise

See [bias-process-noise.md](requirements/runtime/imu/bias-process-noise.md).

### R-BIAS-LIMITS: per-axis bias-value limits

See [bias-limits.md](requirements/runtime/imu/bias-limits.md).

### R-VERTICAL-ACCELERATION: apparent falling and clipping detection

See [vertical-health.md](requirements/runtime/imu/vertical-health.md).

### R-GNSS-QUALITY: GNSS quality checks and health qualification

See [quality.md](requirements/runtime/gnss/quality.md).


### R-HEIGHT-FUSION: PX4 GNSS/barometer height management

See [height-fusion.md](requirements/integration/px4/height-fusion.md).

## Quaternion and SO(3)

- `Q-IDENTITY` — [Identity](requirements/math/quaternion/identity.md).
- `Q-NORMALIZE` — [Normalization](requirements/math/quaternion/normalization.md).
- `Q-MUL` — [Hamilton multiplication](requirements/math/quaternion/multiplication.md).
- `Q-ROT-COMPOSE` — [Rotation composition](requirements/math/quaternion/composition.md).
- `Q-INVERSE` — [Inverse](requirements/math/quaternion/inverse.md).
- `Q-ROT-MATRIX` — [Quaternion rotation matrix](requirements/math/quaternion/rotation-matrix.md).
- `Q-ROTATE` — [Rotation action](requirements/math/quaternion/rotation-action.md).
- `Q-SIGN` — [Quaternion sign equivalence](requirements/math/quaternion/sign-equivalence.md).
- `SO3-HAT` — [Hat operator](requirements/math/so3/hat.md).
- `SO3-EXP` — [Exponential map](requirements/math/so3/exp.md).
- `SO3-LOG` — [Logarithmic map](requirements/math/so3/log.md).
- `SO3-SMALL-ANGLE` — [Small-angle behavior](requirements/math/so3/small-angle.md).

## Supporting operations

- `F-LINALG` — [Fixed-size linear algebra operations](requirements/math/linalg/operations.md).
- `F-SOLVE` — [Symmetric positive-definite linear solve](requirements/math/linalg/spd-solve.md).
- `F-JR` — [SO(3) right Jacobian](requirements/math/so3/right-jacobian.md).
- `F-SCALAR` — [Checked scalar operations](requirements/math/scalar/operations.md).

Before claiming a requirement, define its applicable configurations, input domain and success/failure behavior. Exact mathematical properties, bounded C++ checks and numerical guarantees remain separate claims under the [review criteria](formal-verification-traceability-and-review-criteria.md).
