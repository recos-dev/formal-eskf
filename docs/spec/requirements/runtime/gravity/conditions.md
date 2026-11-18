# R-ACCELEROMETER-CONDITIONS: gravity-fusion prerequisites

**ID:** `R-ACCELEROMETER-CONDITIONS`

**Status:** Draft

**Applies to:** Portable runtime policy; integrations supply timestamps, metadata and deployment parameters.

**Uses:** [R-ACCELEROMETER-MAGNITUDE](magnitude.md).

## Required behavior

`runtime::check_accelerometer_fusion_conditions<Linalg>` consumes caller-supplied
enable, tilt-alignment, horizontal-aiding, rest and per-axis clipping flags,
together with the actual current `AccelerometerMagnitude::update` result.
All inputs describe the same sample/fusion horizon. Availability of a GNSS fix
is not the same as active horizontal aiding. The caller owns those facts; this
function detects neither rest nor alignment and owns no history.

- On a successful magnitude evaluation, `active = enabled AND (within_gravity_band
  OR at_rest) AND NOT horizontal_aiding AND tilt_aligned`.
- `sample_eligible = active AND NOT any_axis_clipped`. Clipping blocks this
  sample without clearing the otherwise active mode. Rest bypasses only the
  band check, not disable, missing alignment, horizontal aiding or clipping.
- A failed magnitude evaluation propagates its status and returns both flags
  false, even at rest. This is the project's checked-input behavior, not a claim
  that PX4 handles invalid arithmetic identically. Do not substitute a previous
  successful result after the current update fails.
- The function is stateless and leaves every input unchanged. Neither flag
  means fusion succeeded, nor establishes a physically valid gravity observation.
  `sample_eligible` is a pre-innovation result; no innovation gate, correction,
  state/P update, timer, sensor selection or recovery logic is introduced.

The separation of mode activation and sample clipping follows
[PX4 e1c28e2 gravity fusion](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/aid_sources/gravity/gravity_fusion.cpp#L56).
Its additional innovation rejection remains a separate deferred task here.
The same predicate supports INS and AHRS; any configuration without horizontal
aiding supplies false for that flag. Native tests enumerate all 256 boolean
combinations with all declared statuses, and exercise real magnitude-result
consumption in both scalar formats. ESBMC additionally checks the actual
stateless predicate for all eight flags, every status and arbitrary unused IEEE
diagnostics, including input purity, in both formats. Rest cannot bypass failed
magnitude evaluation or clipping; horizontal aiding prevents activation. No
separate Lean state-machine theorem is needed for this predicate. Actual
rest/aiding producers and sample/horizon correspondence retain their own
obligations; the portable fusion composition is specified by [R-ACCELEROMETER-FUSION](fusion.md).

`TiltAlignment` has its own initialized inductive proof: finite nonnegative
stored variance and a monotone alignment latch, with both fields preserved on
failure. ESBMC checks the actual source in both formats, consuming the exact-type
rotation and 3x3 product frame contracts. Observers bind `R_nb * P_theta_b * R_nbᵀ`,
all operands and call order; the real transpose and all finite/negative checks
remain inline. Success stores precisely the N/E diagonal sum and applies the
strict encoded `(pi/60)^2` threshold. Numerical outputs may be arbitrary IEEE
values, so overflow/nonfinite/negative-result paths are retained. Lean separately
proves the real tilt projection's nonnegativity, trace decomposition/bound and
identity-frame formula under PSD covariance. It does not establish the truth
of covariance-based attitude confidence, floating-point PSD, or target libraries.

## Caller bindings

- Replay condition sources (host demo): [R-REPLAY-GRAVITY-CONDITIONS](../../integration/replay/gravity-conditions.md).
