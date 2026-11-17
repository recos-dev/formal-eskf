# Sensor positions and body-point output validation

F-05.1 prepared for review on 2026-11-17, based on `e742f7e`.
Reference/build: PX4 v1.16.2, `54f0455ffcd755534539a7cf33a09a20bf71d29d`.
[R-PX4-SENSOR-POSITIONS](spec/verification-requirements.md#r-px4-sensor-positions-sensor-and-output-reference-points)
owns the equations, installation assumptions and evidence boundary.

## Scope and alignment

The module accepts six restart-only FRD installation coordinates, all zero by
default. GNSS antenna position, altitude and rotational velocity are converted
to the delayed IMU point before initialization, gating, fusion and resets.
Current navigation output is converted from IMU to body origin. An attitude
reset includes the resulting body-point position/velocity reset deltas.

Pinned PX4 uses the same nominal point transforms: antenna minus IMU offset
for GNSS, and IMU relative to body origin for output. Both retain receiver noise
without modeling installation uncertainty. This module preserves its fixed
Cartesian frame, ordered latest-prior corrections and existing `z_deriv = vz`
mapping. It freezes installation parameters and checks a per-axis ±10 m profile
bound. Its reset bookkeeping synchronously describes the body-point change;
identical EKF2 reset-counter behavior is not claimed.

No ESKF core equation, state dimension, runtime policy, simulator source or
acceptance threshold changed. No new framework, dynamic container or steady-state
allocation was introduced. Fixed history storage still passes its 64 KiB compile
assertion; this is not a measured target stack/WCET claim.

## Results

Local evidence: `output/px4-sensor-positions-20261117/`. The manifest binds source,
configuration, binaries, commands and logs by SHA-256. Flight provenance retains
the package used for each attempt. Only the test fixture was strengthened after
the flights; the production package and flight binary are unchanged.

| Check | Result / scope |
|---|---|
| Pinned PX4 build | Module fixture and SITL binary build with the target's warning/error flags |
| Native module | 107 scenarios, zero failures; 102 prior scenarios plus five installation fixtures |
| Baseline comparison | Build/run `e742f7e` separately: 102 scenarios pass; its 35,089,827-byte comparison trace is identical to the current trace |
| Module ASan/UBSan | 107 scenarios, zero failures; four production units and fixture instrumented, external PX4 libraries uninstrumented; leak checks disabled |
| Output predictor | 2/2 native and 2/2 ASan/UBSan pass, Eigen and PX4 Matrix; same leak-check limitation |
| Cppcheck | unix32/unix64 pass in configured core/test/component scope; live module translation units are outside that configuration |
| Package / formatting / references | 8/8 package tests, check-only formatting, whitespace and 1,298 traceability references pass |
| SIH attempt 1 | Flight completed; ground checks stopped at 13 pass / 1 fail due to cross-topic recovery sampling, retained below; ULog analyzer correctly refused acceptance |
| SIH attempt 2 | Unchanged harness: 20/20 ground checks and 18/18 independent ULog checks pass |

The oracles are in [ModuleTest.cpp](../integrations/px4/module/tests/ModuleTest.cpp)
(`sensor_position_parameters`, `rotating_sensor_geometry`, `body_reference_output`)
and [test_px4_output_predictor.cpp](../tests/test_px4_output_predictor.cpp).

The comparison trace covers the fixture's existing recorded state, covariance,
policy/reset history and topic payloads, not every possible input. New scenarios
use independent oracles rather than entering the old trace. Rotating-rig point
positions are differentiated numerically using PX4 Matrix rotations, independent
of the production cross-product expression. Tests cover tilted three-axis
rotation and vertical velocity, gyro bias, raw-data/noise preservation, initial
state, first-start resets, ordinary correction and coincident sensors away from
the body origin. Publication uses a current attitude distinct from delayed state.
Reset tests check simultaneous translation and orientation-induced point changes.
Parameter cases use actual PX4 bindings, all six coordinates, finite/range limits
and restart-only ownership. Existing zero-offset cases retain their trace.

SIH uses zero installation offsets, matching its coincident simulated sensor
points. It is a regression check, **not** physical nonzero-installation validation.
Attempt 2 position/velocity/attitude RMSE is 0.237 m / 0.339 m/s / 1.666°;
airborne estimate validity is 100%, final truth height is -0.0033 m, and ULog
dropouts are zero. This pass does not resolve the previously retained stock-SIH
contact-condition repeatability issue in F-07.

## Retained cross-topic sampling failure

Attempt 1 fails `gnss_after_mag_restart`. Its local/global navigation messages
already report reset counter 4, while the 10 Hz status/flags still describe the
preceding pre-recovery publication:

| Captured output | Publication time (µs) | Recovery indication |
|---|---:|---|
| Local position | 161768000 | Position/height/velocity counters 4; navigation valid |
| Estimator status / flags | 161700000 | Corresponding counters 3; GNSS inactive, gravity active |
| Global position | 161780000 | Height counter 4; navigation valid |

The subsequent module snapshot records qualified active GNSS, counters 4 and
gravity inactive, consistent with navigation recovery. The harness compares
different publication epochs; `flight-01/recovery-snapshot-analysis.json` retains
the observations. The failure remains a failure. Neither test assertions nor
module publication rates were changed. F-07 should make cross-topic checks wait
for comparable post-event evidence without bypassing recovery criteria.

## Review conclusion and remaining limits

Self-review checked FRD/NED and altitude signs, delayed/current rate ownership,
initial geographic reference, all GNSS correction/reset inputs, shared publication
point, reset accumulation and unchanged default behavior. The nominal geometry
implementation is ready for code review.

Installation uncertainty and attitude/gyro-error correlations remain unmodeled.
Published variances retain delayed IMU-state proxies; acceleration remains at
the IMU point. Their interpretation is recorded for F-06.1. Physical installation
calibration, target resource acceptance and module source formal proof remain
open. Native tests are not new Lean/ESBMC proofs, and no full agent audit is claimed.
