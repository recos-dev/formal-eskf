# SW-GRAVITY: Conditional gravity aiding

**ID:** `SW-GRAVITY`

**Status:** Draft

**Applies to:** Portable INS/AHRS gravity-aiding components and their composition using the specified 10000 us sampler profile.

**Uses:** [E-OBS-ACCEL](../core/gravity-observation.md), [E-BIAS-UPDATE](../core/bias-update.md).

## Basis

The existing children separate observation preparation, magnitude diagnostics,
eligibility and the numerical transaction. Caller scheduling and tuning remain
in the PX4/replay bindings.

## Required behavior

Gravity aiding shall form non-overlapping observations from real IMU intervals,
evaluate the corresponding magnitude diagnostic and fuse only when the declared
conditions permit it. Correction shall consume the emitted interval observation,
not the filtered diagnostic vector. Caller-provided clipping, rest, alignment and
horizontal-aiding facts shall correspond to that observation's fusion horizon.

Partial samples, held predictions, unavailable clipping and failed magnitude
evaluation shall not become successful gravity corrections. Gaps and source/frame
changes shall reset the sampling history as specified by the children. Numerical
publication remains limited to the successful correction transaction; a band check
or eligible flag alone is not success.

## Decomposition

| Child requirement | Allocated obligation |
|---|---|
| [R-ACCELEROMETER-SAMPLING](../runtime/gravity/sampling.md) | Accumulate rotation-aware intervals, clipping and actual observation durations. |
| [R-ACCELEROMETER-MAGNITUDE](../runtime/gravity/magnitude.md) | Maintain the vector LPF and gravity-band diagnostic. |
| [R-ACCELEROMETER-CONDITIONS](../runtime/gravity/conditions.md) | Evaluate sample eligibility using current diagnostics and caller context. |
| [R-ACCELEROMETER-FUSION](../runtime/gravity/fusion.md) | Apply the conditional correction with state/covariance failure preservation. |

## Acceptance and verification

- Verify the same emitted observation/diagnostic reaches the condition and
  correction stages, with no duplicate magnitude update or substitution of the
  LPF vector for the measurement.
- Exercise partial intervals, clipping availability, gap/source resets and every
  eligibility/failure outcome. Verify actual caller cadence and context separately
  from component arithmetic and control-flow evidence.
- Passing the band test does not establish physical gravity-only motion or
  independence from prediction noise. Innovation gating remains deferred under
  the existing child contracts; this parent does not add it.
