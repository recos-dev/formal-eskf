# Specification Index

Read the relevant requirement and its declared dependencies. Directory placement
identifies ownership, not requirement derivation. Requirement definitions, design
explanations, deployment profiles and verification policy have separate owners.

Use [TEMPLATE.md](TEMPLATE.md) when authoring requirements or architecture.
The [traceability map](../../formal/agent-review/traceability-map.json) registers
verification scope; an indexed requirement or linked proof is not a passing result.

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
| `R-MAGNETIC-FIELD` | [instantaneous magnetic-field plausibility](requirements/runtime/magnetic/field.md) |
| `R-MAGNETOMETER-FUSION` | [field checks and gated magnetic correction](requirements/runtime/magnetic/fusion.md) |
| `R-MAGNETOMETER-RESET` | [fixed-field heading and covariance reset](requirements/runtime/magnetic/heading-reset.md) |
| `R-MAGNETIC-HEADING` | [aided heading consistency](requirements/runtime/magnetic/heading.md) |
| `R-MAGNETOMETER-LIFECYCLE` | [magnetic health and fusion lifecycle](requirements/runtime/magnetic/lifecycle.md) |
| `R-MAGNETIC-REFERENCE` | [geographic magnetic reference adoption](requirements/runtime/magnetic/reference.md) |
| `R-MAGNETOMETER-FILTER` | [magnetic data for heading reset](requirements/runtime/magnetic/reset-filter.md) |
| `R-RING-BUFFER` | [Fixed-capacity timestamped history](requirements/runtime/ring-buffer.md) |

## Integration requirements

| ID | Required behavior |
|---|---|
| `R-PX4-MAGNETIC-AIDING` | [PX4 module magnetic aiding](requirements/integration/px4/magnetic-aiding.md) |
| `R-PX4-MAGNETIC-MODEL` | [PX4 module geographic magnetic reference](requirements/integration/px4/magnetic-model.md) |
| `R-REPLAY-MAGNETIC-FIELD` | [Replay magnetic field checks](requirements/integration/replay/magnetic-field.md) |
| `R-REPLAY-MAGNETIC-LIFECYCLE` | [Replay magnetic lifecycle](requirements/integration/replay/magnetic-lifecycle.md) |
| `R-REPLAY-MAGNETIC-MODEL` | [Replay geographic magnetic model](requirements/integration/replay/magnetic-model.md) |
| `R-REPLAY-MAGNETIC-OBSERVATION` | [Replay magnetic observations](requirements/integration/replay/magnetic-observation.md) |

## Additional contracts

The [requirement catalogue](verification-requirements.md) and its subject links
own requirements not defined in the entries above. A forwarding entry contains
no second definition.

## Architecture

- [Numerical Architecture](architecture/numerics.md).
