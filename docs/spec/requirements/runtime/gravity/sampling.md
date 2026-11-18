# R-ACCELEROMETER-SAMPLING: Rotation-aware gravity observation sampling

**ID:** `R-ACCELEROMETER-SAMPLING`

**Status:** Draft

**Applies to:** Portable 10000 us sampler profile; replay and PX4 callers share the algorithm.

**Parent:** [SW-GRAVITY](../../software/gravity-aiding.md).

**Uses:** [R-ACCELEROMETER-MAGNITUDE](magnitude.md), [SO3-EXP](../../math/so3/exp.md).

## Basis

Follow the rotation-aware endpoint and interval-selection structure of the
[PX4 IMU downsampler](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/imu_down_sampler/imu_down_sampler.cpp).
This is the gravity-observation sampler, not main-filter downsampling.

## Required behavior

`runtime::AccelerometerSample<Linalg, 10000>` shall prepare non-overlapping
observations from real IMU intervals. Callers own admission, gaps and source/frame
changes. It owns partial increments and magnitude-filter history, not estimator
state, covariance or a delayed-fusion buffer:

- Consume finite body-frame angular rate and specific force, an integer period
  in [1, 50000] microseconds, and a clipping mask of -1 (unknown) or 0..7.
  Check non-finite IMU values before period/mask domain errors. Pass the supplied
  normalization threshold to SO3-EXP and propagate its failures. A successful
  partial interval has `ready=false`; it is not an emitted observation.
- For each real sample, form `dtheta = omega_m * dt`, `dv = f_m * dt` and
  `D = R(Exp(dtheta))^T`. Accumulate in the latest body frame as
  `dv_sum_new = D * dv_sum_old + (dv + D * dv)/2`. This is an endpoint rotation
  approximation, not an exact continuous-time/sculling integral. No estimator
  bias is subtracted here; INS correction subtracts its prior bias once.
- Track mean input period `T` starting at 5 ms, using `T += 0.1*(dt-T)`.
  At each interval start set `N = max(1, round(10 ms/T))`, target `N*T` and
  minimum `max((N-1)*T, T/2)`. Emit once the sample count reaches N and duration
  exceeds the minimum, or duration exceeds the target. All decisions use the
  actual integration periods; no interpolated, repeated or overlapping samples.
  Timing rules follow the referenced downsampler, not bitwise PX4 equivalence.
- On emission, divide accumulated dv by the actual interval duration and update
  `AccelerometerMagnitude` exactly once, even when fusion is unavailable/rejected.
  Correction consumes that current interval average, not the LPF vector. Partial
  final intervals are not flushed as shorter observations. Non-finite input,
  invalid periods/masks or arithmetic failure preserves sampler/LPF history without
  publishing an observation; the caller decides how to handle the failure.
- OR all constituent clipping masks; any unknown makes interval clipping unknown.
  The caller supplies rest/alignment/aiding conditions at the emission tick. It shall reset partial
  increments and LPF history on gaps or source/frame changes and resume with real
  samples. Prediction holds are never accumulated as measurements.

## Acceptance and verification

Sampling proof scope: Lean proves the discrete endpoint rule's identity-rotation,
zero-increment, zero-history and transported-sum identities. ESBMC checks actual
10 ms sampler construction/reset and its integration helper in both scalar
formats: exact dt scaling, Exp threshold/status, two inverse-action arguments,
trapezoidal expression, finite-result check, output atomicity and all history/input
frames. Same-type Exp and inverse-action pure frames allow arbitrary numerical
returns; this does not prove rotation accuracy or exact continuous integration.
The inverse input frame is independently checked on its actual implementation.
The 10 ms public sampling owner now has same-type compositional proofs of all
result/history fields, strict readiness, unknown-clipping propagation, child-call
ordering and failure atomicity. Initialization establishes a reachable-state
superset; every update preserves finite vector history, average period in
[1, 50000] us, bounded adaptive thresholds and counters with no integer wrap.
The actual clear body verifies round/cast/count and target/minimum equations;
a separate double-precision producer verifies nearest-integer rounding including
positive half ties. Vector division is composed from the actual same-backend
F-LINALG producer; arbitrary IEEE quotient replies include non-finite failures.
Integration and magnitude producers supply the remaining same-type contracts.
The proof is for `IntervalUs=10000`; it does not certify other template periods,
exact 100 Hz cadence, target libm accuracy or module/replay sequencing. See the
[verification scope and evidence rules](../../../../guides/verification.md).

## Caller bindings

- [R-REPLAY-GRAVITY-SAMPLING](../../integration/replay/gravity-sampling.md).
- [R-PX4-GRAVITY](../../integration/px4/gravity-aiding.md).
