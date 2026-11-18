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
| `SO3-SMALL-ANGLE` | [Small-angle behavior](requirements/math/so3/small-angle.md) |

## Core requirements

| ID | Required behavior |
|---|---|
| `E-PRED-ATTITUDE` | [Attitude prediction](requirements/core/attitude-prediction.md) |
| `E-PRED` | [Nominal prediction](requirements/core/prediction.md) |
| `E-STATE` | [State, frames and perturbation convention](requirements/core/state.md) |
| `E-PRED-INS` | [INS translation prediction](requirements/core/translation-prediction.md) |

## Runtime requirements

| ID | Required behavior |
|---|---|
| `R-RING-BUFFER` | [Fixed-capacity timestamped history](requirements/runtime/ring-buffer.md) |

## Additional contracts

The [requirement catalogue](verification-requirements.md) and its subject links
own requirements not defined in the entries above. A forwarding entry contains
no second definition.

## Architecture

- [Numerical Architecture](architecture/numerics.md).
