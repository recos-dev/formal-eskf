# E-OBS: Observation conventions and model ownership

**ID:** `E-OBS`

**Applies to:** Shared core sensor models and correction wrappers; caller bindings are separate.

## Required behavior

Observation residuals shall use measurement minus prediction, r=z-h(x).
Jacobians shall differentiate h with respect to the local error coordinates of
[E-STATE](state.md), not differentiate the residual or the four unconstrained
quaternion coefficients. Every observation shall use the same prior/time/frame
as its covariance and linearization.

| Model | Contract | Configurations |
|---|---|---|
| Horizontal/vertical position | [E-OBS-POSITION](position-observation.md) | INS |
| Horizontal/3D velocity | [E-OBS-VELOCITY](velocity-observation.md) | INS |
| Full magnetic vector | [E-OBS-MAGNETIC](magnetic-observation.md) | INS/AHRS |
| Gravity vector | [E-OBS-ACCEL](gravity-observation.md) | INS/AHRS |

The caller owns calibration, units, sensor/body alignment, physical reference
point, timestamp alignment, trustworthy references and model applicability.
These premises are not established by algebraic success. Checked wrappers use
[E-CORRECT](correction.md), preserve full noise covariance and INS bias permissions,
and publish only on full success. Each state/P output may alias its corresponding
whole input independently. Unchecked Jacobian functions do not validate inputs.

Runtime gating, health/lifecycle, source resets, coordinate conversion and delayed
fusion have separate requirement owners. They shall not be inferred from a core
observation proof. Fusing separate groups also does not account for correlations
between those groups or permit double-counting a sample.

## Acceptance and verification

Acceptance requires each applicable child model, its local Jacobian/residual and wrapper transaction, with the shared correction assumptions. Test nonidentity attitudes, nonzero cross-covariances, correlated within-group noise and invalid data. Exact model proofs do not establish physical sensor accuracy or observability of every state.
