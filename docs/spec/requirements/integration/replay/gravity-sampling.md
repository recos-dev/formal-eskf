# R-REPLAY-GRAVITY-SAMPLING: Replay sampling and tuning

**ID:** `R-REPLAY-GRAVITY-SAMPLING`

**Applies to:** Host replay input preparation and same-horizon execution.

**Uses:** [R-ACCELEROMETER-FUSION](../../runtime/gravity/fusion.md).

## Required behavior

| Setting | Selected replay baseline |
|---|---|
| Gravity magnitude | `g = 9.80665 m/s^2` |
| Effective per-axis observation standard deviation | `sigma = g` |
| Observation covariance | `V = g^2 * I_3 = 96.1703842225 * I_3 m^2/s^4` |
| Nominal observation/LPF interval | `10,000 us`; actual intervals must be reported |

PX4 `e1c28e2` uses normalized observations, defaults
[`EKF2_GRAV_NOISE`](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/params_gravity.yaml)
to `1.0`, and squares it for observation variance. Multiplying that variance by
`g^2` matches the local tangential information scale near gravity-sized specific
force, not the complete normalized nonlinear update. This is an effective
pseudo-observation weight, not measured hardware noise or validated flight tuning.
It is separate from prediction's `ACCEL_VAR`; do not multiply V by `dt` or `dt^2`.
The MATLAB reference's enabled `accel_correct1` uses normalized observations and
`V_accel = 0.7 * I_3`, corresponding to `67.31926895575 * I_3` on this same scale;
its cross-product preprocessing is not part of the current model.

The PX4 [nominal EKF period](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/module.yaml#L15)
is 10 ms. Its [IMU downsampler](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/imu_down_sampler/imu_down_sampler.cpp)
accumulates rotation-aware increments over whole input samples; it does not just
select the latest raw sample every 10 ms. The selected target is not a promise of
exactly 100 Hz or an instruction to fuse each roughly 250 Hz raw input. Fixed LPF
alpha remains `0.1`; its effective time constant depends on actual update intervals.

`runtime::AccelerometerSample` prepares non-overlapping observations; replay and
the PX4 adapter use it directly. It does not change prediction/navigation timing
or add a delayed fusion buffer:

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
  invalid periods/masks or arithmetic failure preserves sampler/LPF history and
  ends this fail-fast replay, without publishing an observation.
- OR all constituent clipping masks; any unknown makes interval clipping unknown.
  Rest and alignment/aiding conditions use the emission tick. Gaps discard partial
  increments and LPF history; resume with real samples after the gap. Prediction
  holds are never accumulated as measurements. A new replay also resets history.
- After that tick's navigation corrections, call `runtime::fuse_accelerometer`
  only when interval clipping is known; its conditions decide whether to correct.
  This uses the agreed V and the current replay bias-learning permissions. A failed
  correction ends replay. Record each observation, actual duration, aggregated
  clipping, LPF diagnostic and outcome (`fused`, `rejected`, `metadata_unavailable`)
  in `accelerometer_updates.csv`; report counts and actual interval statistics.

Missing clipping still blocks correction, unknown rest disables only its override,
and innovation gating remains deferred. The existing legacy ULog and KITTI inputs
lack clipping and therefore do not exercise successful accel correction. These
starting settings and native tests are not calibrated flight parameters or new
Lean/ESBMC evidence. Prediction/measurement reuse correlation, interval-averaging
model error and full PX4 estimator equivalence remain outside the claim.

The PX4 module shares this sampler and `runtime::TiltAlignment` with replay.
`FormalEskf::fuse_gravity` runs after navigation corrections and before magnetic aiding.
It receives actual gyro/accel integration periods: sampler angular rate is scaled
by their ratio, preserving measured delta-angle while averaging force over the
accel period. Fresh `vehicle_land_detected.at_rest` (at most 2 s old, not future)
provides the rest override; missing/stale rest is false. Horizontal aiding is the
actual GNSS position/velocity lifecycle activity, not merely a valid fix.
Tilt alignment latches when the NED roll/pitch variance sum is below (3°)²;
the estimator owns this latch independently of the gravity sampler and enablement,
and module alignment flags expose it. Clipping is ORed across intervals.
Rejected IMU samples and small gaps clear partial samples and LPF history;
synthetic holds never enter the sampler. Existing restart-required identity,
calibration and large-gap faults terminate the session before further sampling.

`FESKF_GRAV_EN` defaults to 1 and is read only at startup. It requests the existing
eligibility/correction path, not unconditional fusion. Disabled sampling still
updates the LPF. The current-horizon module uses the same g² I weight and no
innovation gate. Each successful correction applies the shared bias limits;
rejection preserves state/P. Status distinguishes emitted samples from successful
corrections and publishes the latest emission interval and gravity mode. Mode is
not a successful-fusion count. The pinned PX4 gravity controller supplies the
condition reference; its normalized sequential/gated update remains a declared
difference. Native tests, flight A/B evidence and formal-proof gaps remain distinct.

The adapter consumes tilt history before sampling, and a completed interval/LPF
update before correction. A later failure does not roll back those earlier
stages. `reset_samples()` clears partial increments, LPF history and active mode,
but preserves the tilt-alignment latch. After successful sampling, disabled,
horizontally aided or unaligned contexts clear active mode even on partial
intervals; otherwise it is held until an emission. The module admits only known
0–7 clipping masks and valid integration periods before this private phase;
unsupported metadata rejects the IMU and resets observation history. The shared
sampler still supports unknown metadata for replay, where it blocks correction.
Known clipping preserves an otherwise active mode while rejecting the sample.
These module history boundaries are checked through production gravity and
`Run()` paths in `integrations/px4/tests/ModuleTest.cpp`. Sampler cadence,
unknown metadata and failure preservation remain checked across native backends
in `tests/test_accelerometer_sample.cpp`. Neither establishes a whole-tick
transaction or closes the pending runtime Lean/ESBMC obligations.

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
