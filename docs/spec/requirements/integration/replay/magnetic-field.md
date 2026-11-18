# R-REPLAY-MAGNETIC-FIELD: Replay magnetic field checks

**ID:** `R-REPLAY-MAGNETIC-FIELD`

**Status:** Draft

**Applies to:** PX4 replay host preparation/execution.

**Parent:** [SW-REPLAY](../../software/replay-aiding.md).

**Uses:** [R-MAGNETIC-FIELD](../../runtime/magnetic/field.md).

## Required behavior

Preparation reads `EKF2_MAG_CHECK`, `EKF2_MAG_CHK_STR` (gauss), and
`EKF2_MAG_CHK_INC` (degrees). Missing values use explicit PX4 v1.16.2 defaults
`1`, `0.2`, and `20`; metadata identifies every default. Reject non-finite,
nonintegral/unknown mask bits, values outside the reference parameter ranges,
and changes during the prepared interval. Convert tolerance units once to
microtesla/radians and write `magnetic_field_parameters.csv` with exactly one
`check_mask,strength_tolerance_ut,inclination_tolerance_rad` row.

- Mask bits 0/1/2 select strength, inclination and required independent reference.
  Only an initialized geographic provider establishes reference availability in
  this path. The measured seed remains the correction reference when no model
  is supplied; it cannot satisfy the required-reference flag.
- Feed the same selection/tolerances to lifecycle field qualification and gated
  correction. Missing required reference is a health rejection, not numerical
  failure; it cannot trigger a correction or heading reset. Log missing reference,
  each performed test, strength/reference strength and inclination residual.
- Older prepared inputs without this file retain their supplied-reference checks
  and state results. Their summary marks the parameters unconfigured. Newly
  prepared logs always select explicitly; defaults are not recovered historical
  configuration or evidence of physical suitability.

The selection, fallback range and reference requirement follow
[PX4 v1.16.2 field checks](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp#L496)
and [parameter definitions](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/params_magnetometer.yaml#L101).
Mask zero disables the health-duration qualification, including reset qualification.
Other masks retain the configured duration, including force-only mask 4.
Our weak-horizontal/numerical guards, sample count, recency, readiness and heading
policy remain active even for mask zero.
Computed rotated strength/atan2 inclination and grouped correction also preclude
claims of bitwise equivalence or identical estimator outcomes. Observation noise,
innovation gate and heading consistency thresholds are separate parameters.
