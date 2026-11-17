# PX4 confidence and failure reporting — 2026-11-17

F-06.1 corrects the existing module's seven published topic families without
changing ESKF equations, measurement acceptance, noise settings or the restart
policy. Contract: [R-PX4-STATUS](spec/verification-requirements.md#r-px4-status-confidence-and-failure-reporting).
Parent: `7079a4f`; reference: PX4 v1.16.2
`54f0455ffcd755534539a7cf33a09a20bf71d29d`. Evidence applies to the dirty source
and binaries identified in `output/px4-confidence-20261117/identity.json`.

## Topic audit

| Topic | Decision and changes | Remaining limits |
| --- | --- | --- |
| `vehicle_attitude` | Current output quaternion, sample epoch and reset delta/counter retained; unhealthy output stops | No independently propagated output covariance |
| `vehicle_local_position` | Separate horizontal/vertical capabilities; PX4-compatible shared local z/vz permission; horizontal uncertainty uses sum of N/E variances; generation-aware dead reckoning | Delayed IMU-state covariance and IMU-point acceleration accompany current body-point output; no lever-arm covariance transport or PX4 dead-reckoning residual inflation |
| `vehicle_global_position` | Latitude/longitude validity follows horizontal aiding; absolute-altitude validity requires height even when vertical velocity remains usable | Fixed-origin geography/geoid assumptions from F-05.2 remain |
| `vehicle_odometry` | Body-local attitude covariance retained; quality stays 0 rather than invented 100% | Diagonal covariance proxies, not calibrated quality |
| `estimator_status` | Independent solution bits, shared packed control/fault mapping, filtered/amplitude innovation summaries, separate horizontal/vertical preflight warnings; unavailable ratios, time-slip and magnetic telemetry are NaN | Our magnetic/velocity models and double arithmetic differ from PX4; no numerical-error wire bit is invented |
| `estimator_status_flags` | Same named control/fault producer; clipping and actual rejection reported separately from gate exceedance and estimator failure | Counters observe 10 Hz publication snapshots, not every transition; unsupported modes remain false |
| `estimator_sensor_bias` | Healthy/device/finite/nonnegative variance checks and PX4's variance-vector norm threshold; learning inhibition does not erase stored validity | Stable flags remain false without a convergence monitor; threshold is a reporting convention, not an accuracy proof |

Pinned PX4 `common.h` and EKF2 publication use fault bits **10/11** for vertical
acceleration/clipping; the old `EstimatorStatus.msg` comment says 16/17. The
actual producer/consumer layout controls this mapping. Our general numerical or
history fault does not mean bad acceleration: existing module fault/error
diagnostics report it and solution/bias permissions are withdrawn. The shell
label is now `estimator_errors`, because the counter includes history failures.
The corresponding SIH parser string changed; its acceptance criteria did not.

## Innovation reporting

`InnovationRatio` is a fixed-size, module-local scalar diagnostic filter shared
by GNSS, magnetic and height owners. Each measurement updates each observed axis
once using its effective timestamp and prediction-minus-observation sign. It
implements the pinned PX4 0.5 s signed-ratio filter, dt limits, outlier limit,
unsigned seeding and zero-on-full-source-reset behavior. Down-only velocity
rescue retains that attempt's diagnostics. Source changes discard the history;
repeated publication does not advance it. Raw gates and actual fusion outcomes
remain available separately. In particular, a deliberately bounded vertical
innovation may exceed the raw gate yet be successfully fused.

This is reporting infrastructure, not an LPF on the estimator's measurements or
state. Missing/invalid diagnostics explicitly become NaN. Core/runtime numerical
code, interfaces, Lean and ESBMC files are unchanged. No PX4 implementation is
vendored into the portable core.

The numeric heading ratio remains a residual diagnostic, including NaN when
unavailable. Its preflight boolean additionally retains the module's magnetic
readiness veto and checks tilt alignment. Pinned Commander does not inspect our
tilt/yaw capability flags; copying EKF2's ratio-only preflight expression without
its GNSS alignment prerequisite was unsafe. This explicit adapter contract keeps
the existing sensor/source/age/variance/health protection and leaves GNSS fusion
conditions unchanged.

## Evidence and failed attempts

Results below are filled from the final validation artifacts. Native/sanitizer
checks are empirical, not formal source proofs. ASan/UBSan instrument the four
module translation units and fixture; linked PX4 archives remain uninstrumented,
and leak detection is disabled for host singleton lifetime.

- `build-01.log` retained a test-only nonexistent tilt-reset method; the fixture
  was corrected. `module-01.log` retained three failed fault-fixture assertions:
  the sample interval did not satisfy the monitor's existing input domain. The
  final fixture uses real 4 ms clipping samples and two independent innovations.
- `flight-01` failed normal arming on heading stability after correcting only
  ratio scaling. Investigation found the missing signed, measurement-time LPF.
  The filter above fixes the reporting defect; neither the 0.5 threshold nor
  arming checks were relaxed, and force arming was not used.
- `flight-02` passed normal arming, the flight sequence and **20/20** ground-fault
  checks. Independent ULog analysis passed **17/18**: truth landing height was
  **−0.663 m**, failing `landed_near_ground`. Position/velocity/attitude RMSE were
  **0.237 m / 0.338 m/s / 1.662°**. This remains a failed accuracy acceptance run,
  consistent with the separately tracked F-07.1 landing anomaly, not a pass.
- `flight-03` validates the pre-review source after removing obsolete ratio caches,
  clearing same-ID source-generation history and marking unsupported telemetry
  unavailable. It passed 20/20 ground-fault checks but retained the same 17/18
  ULog failure (−0.610 m landing truth). All earlier attempts remain.
- Independent review found the ratio-only heading preflight expression dropped
  the previous unavailable-heading veto. Real on-ground magnetic source and
  calibration replacements can invalidate heading while GNSS remains usable;
  Commander does not supply the missing tilt/yaw check. Added real uORB/Run/
  buffer/lifecycle regressions: **126 scenarios, 5 failures** before repair
  (`build-08-regression.log`, `module-06-regression.log`), covering both replacement
  paths plus no-tilt/stale/fault cases. Restoring full magnetic readiness and
  adding explicit tilt readiness gives **126/0** (`build-09.log`, `module-07.log`).
  Independent re-review confirmed the repair and found no additional blocker.
  This is why the final SIH run must use the repaired binary, not flight-03.

## Final validation

| Check | Final result | Artifact |
| --- | --- | --- |
| Pinned PX4 module/flight binary build | Pass | `build-09.log` |
| Native production-module fixtures | 126 scenarios, 0 failures | `module-07.log` |
| ASan/UBSan production-module fixtures | 126 scenarios, 0 failures | `module-asan-review/module.log` |
| Package tests / installed identity | 8 tests pass; 63 installed payloads match | `package-review.log`, `identity.json` |
| Formatting / traceability references | Pass; 1,319 source/spec references resolve | `format-review.log`, `identity.json` |
| Final SIH live sequence | Normal arm, waypoints, land, automatic disarm; 20/20 ground-fault checks pass | `flight-04/flight.json` |
| Final independent ULog checks | **17/18; fail** on truth landing height **−0.616 m** | `flight-04/analysis.json` |

Final airborne position/velocity/attitude RMSE: **0.236 m / 0.339 m/s / 1.734°**;
maximum attitude error **3.466°**; valid-estimate fraction **100%**; no in-air
failsafe or ULog dropout. These sampled SIH metrics do not validate tuning on a
real vehicle. The unresolved landing criterion is retained under F-07.1, without
changing the threshold or rerunning until a favorable sample appears.

## Landing failure disposition

The subsequent [independent SIH investigation](debug-sih-landing-20261117.md)
reproduced approximately 0.603 m of ground penetration using the unchanged pinned
SIH dynamics **without an estimator, controller or land detector**. The two ULogs
show the same missing-contact condition, fixed reference altitude and penetration
before land detection. This establishes an external simulator defect as a
sufficient cause; it does not prove all estimator/controller behavior correct or
attribute every historical landing failure to that defect.

F-06.1 can therefore be reviewed as a reporting correction separately from the
SIH contact repair. Its original flight acceptance remains **17/18**, not waived
or relabeled as passing. No simulator patch, threshold relaxation, noise tuning
or ESKF core change is part of this change. Full simulator/landing acceptance
remains open under F-07.1.

The final repaired-source run, `flight-04`, has the same observed contact defect:
first below ground at 105.808 s with 0.804 m/s downward speed, 43/43 samples with
upward acceleration during 0.86 s of continued penetration, then stop at
−0.616058 m before land detection. The reference altitude remains fixed.
`flight-04/landing-diagnosis.json` records that check using the same inspector.
Original analyses and the pinned simulator source remain unchanged.

## Self-review and scope

Capability fixtures independently exercise height-only, horizontal-only,
velocity-without-height, changed source, lost aiding, unaligned, stale and fault
states. Bias tests cover zero, threshold, next representable above threshold,
aggregate norm, negative/non-finite variance, learning inhibition and unhealthy
output. Filter oracles cover timestamp boundaries, signed cancellation, clipping,
invalid/recovered diagnostics and resets; production tests cover source changes,
reset integration and publication without re-filtering. Packed/named flags and
counter changes are checked against the same scenarios. Existing real numerical,
IMU-identity and timeline fault scenarios still exercise restart-required behavior.

Review found no reason to change core formulas or sensor acceptance. It also did
not establish uncertainty calibration, long-range validity, whole-module formal
proof, lossless fault-event delivery, real-time WCET or hardware-flight safety.
F-05.2's geographic budget and F-07's disturbance/resource/hardware work remain
open. The live comparison is behaviorally aligned to the supported pinned PX4
rules; it does not claim EKF2 feature or numerical equivalence.

## Commit review disposition

Self-review and independent review found no remaining blocker in this scoped
reporting change after the heading-readiness regression was repaired and retested.
The before/after regression result is retained rather than hidden by the final
pass. Installed payloads and the final flight binary match the reviewed source;
the original pre-review identity is retained as `identity-before-review.json`.

The proposed change comprises the module reporting/filter implementation, its
package entry, native fixtures, the shell-label parser update, requirement/map
updates and these validation/debug records. Existing unrelated ROADMAP drafts,
other FEATURE_TODO hunks, datasets and `eskf_codegen` are outside this change.
No SIH physics patch, new fusion policy, numerical core change or formal-proof
claim is included. The unresolved simulator acceptance is explicitly separate;
this disposition is permission to review a scoped commit, not flight certification.
