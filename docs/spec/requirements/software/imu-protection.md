# SW-IMU: IMU health and bias-learning control

**ID:** `SW-IMU`

**Status:** Draft

**Applies to:** Portable INS runtime policies and their caller bindings; configuration, health provenance and timing are caller inputs.

**Uses:** [E-BIAS-UPDATE](../core/bias-update.md), [E-NOISE](../core/process-noise.md), [E-STATE](../core/state.md).

## Basis

These children control different effects: bias-gain permission, sampled IMU
variance, bias driving noise, explicit bias-value bounds and vertical-health
history. They are not interchangeable forms of one health flag.

## Required behavior

INS callers shall bind IMU health and bias controls to the actual sensor intervals,
state prior and configuration required by each child. Sample-noise selection,
bias-noise inhibition and correction permissions shall affect only their assigned
parts of the estimation operation. A disabled bias-learning permission shall not
be interpreted as clearing an estimated bias or changing all process noise.

Callers shall preserve the distinction between held fault indicators and ordered
last-good/update timestamps, and shall not manufacture fresh health from missing
or held measurements. Bias limiting shall remain an explicit state operation;
it shall not be presented as covariance correction or evidence of physical bias
accuracy. Caller-specific ordering and recovery remain owned by the integration
requirements.

## Decomposition

| Child requirement | Allocated obligation |
|---|---|
| [R-IMU-BIAS-INHIBITION](../runtime/imu/bias-inhibition.md) | Maintain motion envelopes and select per-axis bias-learning permissions. |
| [R-VERTICAL-ACCELERATION](../runtime/imu/vertical-health.md) | Track clipping/apparent-falling health and its distinct timing signals. |
| [R-IMU-PROCESS-NOISE](../runtime/imu/process-noise.md) | Select fault-dependent sampled IMU variance. |
| [R-BIAS-PROCESS-NOISE](../runtime/imu/bias-process-noise.md) | Select bias driving noise from the prior variance. |
| [R-BIAS-LIMITS](../runtime/imu/bias-limits.md) | Apply finite per-axis bias bounds with failure preservation. |

## Acceptance and verification

- Verify each policy's equations/transitions and unchanged fields, then verify
  that caller prediction/correction consumes the intended result once with the
  specified prior, timing and units.
- Exercise enabled/disabled learning, clipping and held-fault transitions,
  unavailable health, invalid data and configuration changes. Check that one
  control is not substituted for another and that numerical failures retain the
  appropriate operation's publication boundary.
- Thresholds and health heuristics do not establish sensor-fault detection
  completeness, bias observability or tuned estimation accuracy.
