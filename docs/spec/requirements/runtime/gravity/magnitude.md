# R-ACCELEROMETER-MAGNITUDE: filtered gravity-band diagnostic

**ID:** `R-ACCELEROMETER-MAGNITUDE`

**Applies to:** Portable runtime policy; integrations supply timestamps, metadata and deployment parameters.

## Required behavior

`runtime::AccelerometerMagnitude<Linalg>` filters body-frame specific force in
m/s^2 before estimator-bias subtraction. The caller supplies
`delta_vel / delta_vel_dt`, not an integrated increment. It owns only one filtered vector and
its initialization flag; no timestamps, estimator state or covariance.

- The first valid sample seeds the vector directly. Later samples use
  `filtered += alpha * (sample - filtered)`, then compute `squared_norm(filtered)`.
  Filter the vector, not its magnitude or a peak envelope.
- Keep alpha fixed at `0.010 / (0.090 + 0.010)`, evaluated in the selected scalar
  format. Do not recompute it from each sample's dt. The nominal 90 ms time
  constant assumes 10 ms updates; actual cadence belongs to the caller. This
  component does not resample, enforce that cadence or promise 90 ms at 250 Hz.
- With `g = 9.80665 m/s^2`, report `within_gravity_band` only when the computed
  squared norm is strictly between `(0.9*g)^2` and `(1.1*g)^2`. Equality fails.
  A finite out-of-band sample still updates filter history successfully.
- Non-finite input returns `non_finite_input`; a non-finite filtered vector or
  squared norm returns `non_finite_result`. Failure preserves initialization and
  filter history, with a fresh false band flag and invalid zero diagnostic.
  Successful zero input is not an error. `reset()` clears history and requires
  the next valid sample to seed again, including after source/frame/calibration
  changes. The filtered-vector accessor is meaningful only after initialization.
- This result is not fusion permission, sensor health or evidence of rest.
  Persistent horizontal acceleration can pass; body rotation can reduce the
  filtered magnitude. No bias subtraction, normalization, correction, motion
  policy, clipping logic, at-rest override or innovation gate is added here.

The valid-input recurrence, initialization, fixed coefficient and strict band
follow PX4 revision `e1c28e27550114e85ddb8760ab3355eaa7a89d8a`:
[coefficient settings](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/ekf.h#L650),
[initial nominal interval](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/estimator_interface.h#L358),
[sample updates](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/ekf.cpp#L162),
[magnitude check](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/aid_sources/gravity/gravity_fusion.cpp#L50).
Checked failure behavior follows this project's scalar contract. This is not
alignment with the older v1.16.2 gravity-fusion peak envelope, a claim of complete
PX4 fusion equivalence, or a backend-independent bitwise equivalence claim.

`tests/test_accelerometer_magnitude.cpp` checks both scalar formats, recurrence,
startup/reset, strict band boundaries, vector-before-magnitude behavior and
failure preservation. Separate Lean theorems prove the real affine recurrence,
constant-input preservation, convex bounds, one-step residual contraction and
exact residual after any number of constant-input updates. These do not prove
that the signal is gravity or bound floating-point error.

ESBMC checks the actual constructor/reset, exact scalar-format filter/norm/band
expressions, failure statuses and complete history/input frames in binary32 and
binary64 with the fixed-array proof backend. Finite history is established by
construction/reset and preserved inductively by every public update, including
failures; successful diagnostics are finite and nonnegative. Accessor identity
is checked. Target backend arithmetic, cadence and physical interpretation remain
outside this source claim.
