# SW-CORE: Portable ESKF estimation operations

**ID:** `SW-CORE`

**Status:** Draft

**Applies to:** Caller-driven INS and rotation-only AHRS operations in the shared core; each child's configuration and input domain apply.

**Uses:** [E-STATE](../core/state.md), [C-LAYER-BOUNDARY](../common/layer-boundaries.md), [C-NUMERICAL-PROFILE](../common/numerical-profile.md), [C-INPUT-DOMAIN](../common/input-domain.md), [C-FAILURE](../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../common/numerical-acceptance.md).

## Basis

This requirement composes the existing numerical contracts below. Their sources,
equations and approximation choices remain authoritative. It does not introduce
a scheduler, sensor lifecycle or new estimator state.

## Required behavior

The shared core shall provide prediction and measurement correction in the state
and local-error coordinates of E-STATE. Nominal prediction, error covariance and
observation linearization shall use the priors, frames and time intervals required
by their respective contracts. Process noise shall be converted and injected once.

Correction shall use the selected effective gain consistently for error and
covariance updates, inject the error and express the resulting covariance in the
new local coordinates. Public composed operations shall publish at the transaction
boundaries of E-STEP; a caller composing separate operations owns any wider
transaction. Checked failure behavior does not turn unchecked model helpers into
validated entry points.

## Decomposition

| Child requirement | Allocated obligation | Applicability |
|---|---|---|
| [E-PRED](../core/prediction.md) | Predict the nominal state using the selected attitude and translation models. | INS/AHRS; translation is INS-only. |
| [E-NOISE](../core/process-noise.md) | Convert the declared sampled-noise and bias-density inputs into step covariance. | INS/AHRS use their respective source blocks. |
| [E-PCOV](../core/covariance-prediction.md) | Propagate the error covariance and map source noise into error coordinates. | INS/AHRS; selected attitude mode. |
| [E-OBS](../core/observations.md) | Define supported sensor residuals, local Jacobians and observation wrappers. | Per model; position/velocity are INS-only. |
| [E-INNOV](../core/innovation.md) | Evaluate the componentwise innovation decision when requested by a caller. | Explicit gate operation; not implicit in every correction. |
| [E-CORRECT](../core/correction.md) | Solve for the gain and apply the Joseph covariance update. | INS/AHRS. |
| [E-BIAS-UPDATE](../core/bias-update.md) | Apply the supplied INS bias-learning permissions. | INS only; AHRS has no bias states. |
| [E-MAG-TILT](../core/magnetic-tilt.md) | Apply the selected magnetic attitude-gain protection. | Magnetic correction; both enabled and disabled build modes have defined behavior. |
| [E-INJECT](../core/injection.md) | Inject the estimated local error into the nominal state. | INS/AHRS. |
| [E-RESET](../core/covariance-reset.md) | Transport covariance after error injection. | INS/AHRS; selected reset mode. |
| [E-STEP](../core/composed-steps.md) | Enforce complete-operation order, failure propagation and publication. | Public composed operations, not an arbitrary caller sequence. |

## Acceptance and verification

- Satisfy each applicable child and its dependencies for the declared INS/AHRS,
  scalar, backend and approximation profile. Existing E-PRED, E-OBS and E-STEP
  subrequirements retain their own scope.
- Establish composition across the shared prior, noise scaling, final gain,
  injection/reset and transaction boundaries. Separate-operation tests or proofs
  alone do not establish correct argument wiring and joint publication.
- Preserve the distinction between exact-model proofs, scoped C++ checks and
  numerical/target evidence. No long-run estimation accuracy, physical sensor
  validity or complete deployed-backend proof follows from this decomposition.
