# R-PX4-GRAVITY: Delayed gravity-aiding binding

**ID:** `R-PX4-GRAVITY`

**Status:** Draft

**Applies to:** PX4 INS module at the delayed IMU/fusion horizon.

**Parent:** [SW-PX4](../../software/px4-estimation.md).

**Uses:** [R-ACCELEROMETER-SAMPLING](../../runtime/gravity/sampling.md), [R-ACCELEROMETER-FUSION](../../runtime/gravity/fusion.md), [PX4-CONFIG](configuration.md), [R-PX4-DELAYED-FUSION](delayed-fusion.md).

## Required behavior

The PX4 module uses `AccelerometerSample` and `runtime::TiltAlignment`.
`FormalEskf::fuse_gravity` runs after navigation corrections and before magnetic aiding.
It receives actual gyro/accel integration periods: sampler angular rate is scaled
by their ratio, preserving measured delta-angle while averaging force over the
accel period. Historical `vehicle_land_detected.at_rest` (strictly less than 2 s
old at the delayed horizon, known at its retained IMU context and not future)
provides the rest override; missing/stale rest is false. Horizontal aiding is the
actual GNSS position/velocity lifecycle activity, not merely a valid fix.
Tilt alignment latches when the NED roll/pitch variance sum is below (3°)²;
the estimator owns this latch independently of the gravity sampler and enablement,
and module alignment flags expose it. Clipping is ORed across intervals.
Rejected IMU samples and small gaps clear partial samples and LPF history;
synthetic holds never enter the sampler. Existing restart-required identity,
calibration and large-gap faults terminate the session before further sampling.

`FESKF_GRAV_EN` defaults to 1 and is read only at startup. It requests the existing
eligibility/correction path, not unconditional fusion. When gravity aiding is
compiled in, disabling this startup parameter still updates the sampler/LPF.
Build exclusion under PX4-CONFIG omits sampling and correction but retains tilt
alignment. Use g=9.80665 m/s² and V=g² I in raw force units at the delayed horizon,
without an innovation gate. Each successful correction applies the shared bias limits;
rejection preserves state/P. Status distinguishes emitted samples from successful
corrections and publishes the latest emission interval and gravity mode. Mode is
not a successful-fusion count. The pinned PX4 gravity controller supplies the
condition reference; its normalized sequential/gated update remains a declared
difference. Native scenarios and formal source claims remain distinct.

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

## Acceptance and verification

These module history boundaries are checked through production gravity and
`Run()` paths in `integrations/px4/tests/ModuleTest.cpp`. Sampler cadence,
unknown metadata and failure preservation remain checked across native backends
in `tests/runtime/gravity/test_sampling.cpp`. Neither establishes a whole-tick
transaction, module source proof or physical model validity. The portable
sampler/fusion contracts own their scoped Lean/ESBMC obligations; their proofs
do not establish this adapter's context and scheduling.
