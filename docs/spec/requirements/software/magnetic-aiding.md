# SW-MAGNETIC: Qualified fixed-field magnetic aiding

**ID:** `SW-MAGNETIC`

**Status:** Draft

**Applies to:** Portable INS/AHRS magnetic policies and caller composition using a fixed navigation-frame observation reference and the selected tilt-protection mode.

**Uses:** [E-OBS-MAGNETIC](../core/magnetic-observation.md), [E-MAG-TILT](../core/magnetic-tilt.md), [R-MEASUREMENT-FUSION](../runtime/measurement-fusion.md).

## Basis

The child contracts provide magnetic qualification, reference handling,
correction and heading recovery without owning the platform sensor source or
estimator. Geographic lookup and caller-specific startup remain separate.

## Required behavior

Magnetic aiding shall distinguish the roles of the observation reference and the
optional health reference. A caller may explicitly assign the same vector to both
roles under R-MAGNETIC-FIELD; an observation reference alone does not establish
independent health-reference availability. Field and heading diagnostics, lifecycle readiness,
innovation acceptance and successful numerical correction shall retain their
separate meanings. A caller shall use consistent source, reference, state and
time context when passing results between these stages.

Lifecycle permission shall authorize only the matching observation's operation.
Correction and heading reset shall report their actual outcomes before progress
is acknowledged. Lifecycle-controlled heading recovery shall use the reset
measurement required by its caller binding and preserve the child-defined tilt,
covariance and unrelated-state behavior. Reference and source/calibration changes
shall follow their distinct invalidation rules; they
shall not silently authorize fusion or manufacture successful progress.

## Decomposition

| Child requirement | Allocated obligation |
|---|---|
| [R-MAGNETIC-REFERENCE](../runtime/magnetic/reference.md) | Validate and adopt an externally supplied geographic reference when available. |
| [R-MAGNETIC-FIELD](../runtime/magnetic/field.md) | Check field geometry and selected plausibility conditions. |
| [R-MAGNETIC-HEADING](../runtime/magnetic/heading.md) | Evaluate aided heading consistency under its observability conditions. |
| [R-MAGNETOMETER-LIFECYCLE](../runtime/magnetic/lifecycle.md) | Qualify/start/stop aiding and manage single-use correction/reset permissions. |
| [R-MAGNETOMETER-FUSION](../runtime/magnetic/fusion.md) | Compose field checking, innovation gating and protected correction. |
| [R-MAGNETOMETER-FILTER](../runtime/magnetic/reset-filter.md) | Prepare filtered magnetic measurements for heading reset. |
| [R-MAGNETOMETER-RESET](../runtime/magnetic/heading-reset.md) | Apply the checked heading/covariance reset. |

## Acceptance and verification

- Establish source/reference/result correspondence across the caller sequence;
  check qualification, rejection, timeout, permitted reset and failed feedback.
- Exercise independent-reference presence/absence and enabled/disabled checks,
  preserving observation-reference selection and the configured tilt behavior.
  Reference absence shall follow the child policy, not an invented universal ban
  or permission.
- Verify source/calibration and reference-only transitions separately. Component
  evidence does not prove geographic lookup validity, physical magnetic health,
  heading observability or the correctness of PX4/replay wiring.
