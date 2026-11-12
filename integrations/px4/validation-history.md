# PX4 integration validation history

These records were moved from the integration README without rerunning or
reclassifying the checks. Each result applies to its recorded source, settings,
installed package and binary; later code changes require impact review. Failed
attempts and limitations remain part of the record.

Evidence paths such as `output/...` are relative to the formal-eskf repository
root and refer to local artifacts, not files bundled with this document. If an
artifact is unavailable, the summary does not establish reproducible acceptance.
See the [integration guide](README.md) for current setup and behavior,
[FEATURE_TODO.md](../../FEATURE_TODO.md) for the functional backlog, and the
[review criteria](../../docs/spec/formal-verification-traceability-and-review-criteria.md)
for requirement/evidence acceptance.

## Local validation: 2026-11-06

Built and flown using the installed current headers and the pinned PX4 host.
Normal arming, takeoff, square trajectory, yaw change, landing and automatic
disarming passed. Ground fault checks distinguish 1 s dead reckoning, 5 s
horizontal validity loss and 7 s session stop; recovery increments position
and velocity reset counters once each.

| Check | Result |
| --- | --- |
| Package installation/export | 8 passed |
| GNSS quality/lifecycle/reset and fusion native suites, PX4 matrix | 4 passed |
| Independent ULog acceptance checks | 18 passed |
| Ground sensor failure/recovery scenarios | 10 passed |
| Position / velocity RMSE against SIH truth | 0.381 m / 0.223 m/s |
| Attitude RMSE / maximum error | 1.434 / 3.915 degrees |
| Valid estimates in flight | 100% |
| Numerical faults / airborne failsafes / ULog dropouts | 0 / 0 / 0 |

Local evidence is in `output/px4-gnss-sitl-final/`: source manifest, binary hash,
flight/fault records, ULog, `analysis.json` and `flight.png`. The earlier
`output/px4-gnss-sitl-run/` failed its fixed 4 s recovery expectation after a
stationary-speed check transient; it is retained and is not counted as a pass.
The test now waits for actual quality recovery with a bounded additional 20 s
window, checking that resets do not occur before recovery. Quality thresholds
and health timing were not relaxed.

## Magnetic integration validation: 2026-11-11

The pinned host build, normal flight and independent ULog analysis passed.
The 14 ground scenarios include magnetic outages with/without GNSS aiding,
loss of unaided yaw alignment and one real heading reset on recovery.
Attitude, local-position and estimator-status reset counters agreed; the
recorded reset was -0.02268 rad. This is a control-flow test, not heading-accuracy
ground truth for a physical compass disturbance.

| Check | Result |
| --- | --- |
| Magnetic native suites, float/double | 5 Eigen + 5 PX4 matrix passed |
| Adapter/lifecycle ASan + UBSan, both backends | 4 passed, leak detection enabled |
| Package installation/export | 8 passed |
| Ground failure/recovery scenarios | 14 passed |
| Independent ULog acceptance checks | 18 passed |
| Position / velocity RMSE against SIH truth | 0.404 m / 0.258 m/s |
| Attitude RMSE / maximum error | 1.643 / 3.555 degrees |
| Valid estimates in flight | 100% |
| Numerical faults / airborne failsafes / ULog dropouts | 0 / 0 / 0 |

Evidence is in `output/px4-mag-sitl-02/`, including the source manifest, binary
hash, ULog, fault records, `analysis.json` and `flight.png`. Native tests also
exercise field rejection, heading inconsistency, calibration/source changes,
stale/future/duplicate samples and reset failure. They do not establish aircraft
behavior under in-flight magnetic disturbances. The wrapper/lifecycle additions
have no new Lean or ESBMC proof in this change.

An additional Cppcheck 2.21 compilation-database scan reported no diagnostics
in the module/core, but 334 in upstream/generated PX4 headers. This is not a
clean whole-PX4 static-analysis result or a substitute for formal evidence.

## Magnetic reference validation: 2026-11-11

Two closed-loop runs exercised geographic heading (mask 3) and stored heading
(mask 2, 3.4056056°, the host model's declination at the SIH origin). Both passed
all 18 ULog acceptance checks and 14 ground failure/recovery scenarios.

| Check | Geographic | Stored |
| --- | --- | --- |
| WMM lookup count, startup to landing | 1 → 10 | 1 → 10 |
| Selected heading source | Geographic | Stored |
| Observation reference revision, startup to landing | 1 → 1 | 1 → 1 |
| Position / velocity RMSE | 0.346 m / 0.259 m/s | 0.404 m / 0.256 m/s |
| Attitude RMSE | 1.519° | 1.649° |

Evidence is in `output/px4-wmm-geographic/` and `output/px4-wmm-saved/`, with
source manifests and binary hashes. The stored run used the final build after
splitting the declination status line to avoid console truncation. These short
flights did not cross model-adoption thresholds. Native tests separately exercise
changed models, real heading reset, health-only rejection without heading reset,
all four source masks with/without WMM, failed lookups and frame conversion.
Three native suites passed on each backend (Eigen/PX4 matrix, float/double);
the four adapter suites also passed ASan/UBSan with leak detection enabled.
The pinned host build, all eight package tests, repository Cppcheck profiles
(unix32/unix64) and formatting checks passed. This change adds no Lean/ESBMC
proof or evidence of long-distance/in-flight disturbance behavior.

## Height integration validation: 2026-11-12

The final pinned-host build passed normal arming, the flight trajectory, landing,
automatic disarming and all 20 ground fault/recovery scenarios. The six added
scenarios exercise barometer outage/re-entry, GNSS-to-barometer fallback, loss of
both height sources and recovery. Local/global/status height reset counters agree;
global altitude deltas have the opposite sign to local Down deltas.

| Check | Result |
| --- | --- |
| Height, magnetic and GNSS regression suites, float/double | 5 Eigen + 5 PX4 matrix passed |
| Height ASan + UBSan, leak detection enabled | 2 backend suites passed |
| Package installation/export | 8 passed; 65 installed files verified |
| Cppcheck unix32/unix64; formatting | Passed |
| Final independent ULog acceptance / ground scenarios | 18/18; 20/20 passed |
| Position / velocity RMSE against SIH truth | 0.165 m / 0.262 m/s |
| Attitude RMSE / maximum error | 1.654° / 3.662° |
| Valid estimates / numerical faults / airborne failsafes / dropouts | 100% / 0 / 0 / 0 |

Final evidence is in `output/px4-height-sitl-02/`, including the source manifest,
binary hash, ULog and reports. Native tests additionally cover bias drift, rejected
sources, calibration changes, timeout boundaries, simultaneous resets, Down-only
velocity recovery and numerical rollback. They do not establish in-flight outage
recovery on real hardware. This change adds no Lean/ESBMC proof.

The earlier `output/px4-height-sitl-01/` passed all 20 ground scenarios but only
17/18 flight checks: SIH stopped at -0.547 m true height, outside the fixed ±0.5 m
landing limit. Truth crossed the ground plane and continued descending for 0.74 s;
this is consistent with SIH's ground contact requiring both negative altitude and
downward net force, without a position clamp. The failure remains recorded. The
final run stopped at -0.0026 m; neither simulator nor acceptance threshold changed.

## IMU/bias integration validation: 2026-11-12

The final pinned-host build passed normal arming, trajectory, landing and automatic
disarming. All 20 existing ground sensor failure/recovery scenarios passed; their
status snapshots reported accel/gyro learning masks 7/7, no high dynamics and an
available vertical-health verdict without a fault.

| Check | Result |
| --- | --- |
| Bias policy, vertical health and height/magnetic adapter suites | 4 Eigen + 4 PX4 matrix passed, float/double |
| Same suites under ASan + UBSan, leak detection enabled | 8 passed |
| Existing replay bias scenarios | 8 passed |
| Package installation/export | 8 passed; 66 installed files verified |
| Cppcheck unix32/unix64; formatting | Passed |
| Independent ULog checks / ground scenarios | 18/18; 20/20 passed |
| Position / velocity RMSE against SIH truth | 0.163 m / 0.260 m/s |
| Attitude RMSE / maximum error | 1.710° / 3.697° |
| Valid estimates / numerical faults / airborne failsafes / dropouts | 100% / 0 / 0 / 0 |

Evidence is in `output/px4-imu-bias-sitl-01/`, with the source manifest and binary
hash. Native tests exercise independent integration periods, peak decay/failure
rollback, fault-driven inhibition, three-second recovery, real height/magnetic
corrections with inhibited accel bias, and reset-safe health diagnostics. This
normal-flight run does not validate physical high-dynamic/clipping disturbances
or demonstrate an accuracy improvement. No Lean/ESBMC proof was added.

## IMU noise and clipping validation: 2026-11-12

F-01.1 connects the existing noise selector to the module's production prediction
path. Native tests use that same path with Eigen and PX4 matrix, float/double:
per-axis clipping, preceding vertical faults, rotated `dt²` covariance, retained
measured motion, restoration, unchanged gyro/bias noise and failure rollback.
Admission tests retain startup/gyro-clipping rejection; component sequences
exercise clipping counters and bias inhibition. These tests do not execute the
entire uORB scheduler under all possible event sequences.

| Check | Result |
| --- | --- |
| Module IMU prediction, noise selector, vertical health and replay bias | 4 Eigen + 4 PX4 matrix suites passed, float/double |
| Same suites under ASan + UBSan | 8 passed, leak detection enabled on the host |
| Pinned PX4 build; package installation/export | Passed; 8 package tests, 67 installed files verified |
| Cppcheck unix32/unix64; formatting | Passed |
| Independent ULog flight checks / ground fault scenarios | 18/18; 20/20 passed |
| Position / velocity / attitude RMSE against SIH truth | 0.164 m / 0.260 m/s / 1.696° |
| Valid estimates / numerical faults / airborne failsafes / log dropouts | 100% / 0 / 0 / 0 |

Flight evidence is in `output/px4-imu-noise-sitl-02/`, including installed source
hashes and binary identity. This flight is a regression check, not a physical
clipping experiment or an accuracy-improvement claim. Sandbox attempts could
not open SITL's local UDP socket or complete LeakSanitizer; the successful runs
used the host environment with leak detection retained. No Lean/ESBMC proof
was added; those gaps remain explicit in the traceability map.

## Bias driving-noise integration validation: 2026-11-12

F-01.2 connects the existing per-axis variance policy after IMU noise selection,
including gap holds. Module/policy suites passed on both backends (4 native and
4 ASan/UBSan suites, float/double, leak detection enabled). Threshold equality,
selected clipping variance, release and failure rollback are covered. Formatting,
unix32/unix64 Cppcheck and the pinned PX4 build passed.

`output/px4-bias-noise-sitl-01/` passed all 18 independent flight checks and all
20 ground scenarios: position/velocity/attitude RMSE 0.163 m / 0.257 m/s / 1.669°;
100% valid estimates, no numerical faults, airborne failsafes or ULog dropouts.
This is integration regression evidence; no Lean/ESBMC proof was added.

## Bias-limit integration

F-01.3 applies the existing per-axis limiter at initialization and after each
successful GNSS, height or magnetic correction group, with fixed 0.4 m/s² and
0.15 rad/s bounds. Each height source is limited before the next source runs.
Rejected updates and resets do not clamp; covariance is unchanged by limiting.
PX4 limits after scalar fusions, so grouped saturation trajectories can differ.
Published bias limits use those same constants. Native and ASan/UBSan suites
passed on both backends (6 each, float/double; leak detection enabled), together
with the pinned PX4 build, 8 package tests, formatting and unix32/unix64 Cppcheck.
All existing correction call sites were reviewed. This commit adds no formal
proof or dedicated flight experiment; integration flight regression continues
with F-02.1.

## Gravity integration validation: 2026-11-12

F-02.1 reuses the existing sampler/tilt logic without changing numerical-core
models. Eight native and eight ASan/UBSan suites cover both backends and scalar
formats, including disabled/aided/unaligned states, rest override, clipping,
unknown metadata, independent integration periods, actual tilt correction,
bias limiting and history reset. Leak detection remains enabled. Pinned PX4
build, 8 package tests (71 installed files), formatting and unix32/unix64
Cppcheck passed. A test-only unread-result warning was corrected and rechecked.

| Same binary, startup gravity setting | Enabled | Disabled |
| --- | --- | --- |
| Independent ULog flight checks | 18/18 | 18/18 |
| Ground failure/recovery scenarios | 20/20 | 20/20 |
| Gravity corrections through final ground scenario | 3160 | 0 |
| Position / velocity RMSE | 0.186 m / 0.220 m/s | 0.163 m / 0.256 m/s |
| Attitude RMSE | 1.063° | 1.642° |

`output/px4-gravity-{on,off}-sitl-01/` records settings, source/binary hashes and
ULog evidence. Enabled gravity starts after horizontal aiding stops and stops
on GNSS recovery. The 250 Hz input emitted 8 ms observations in these runs;
10 ms is a nominal whole-sample target, not an exact-rate promise. Both flights
had 100% valid estimates and no numerical faults, airborne failsafes or dropouts.
Independent runs/ground outages are not a paired accuracy-improvement experiment
or physical acceleration-fault validation. Lean/ESBMC adapter evidence is pending.

## Vertical recovery integration validation: 2026-11-12

F-03.1 connects the existing bounded-innovation path and extracts the existing
last-good reset predicate for use by both GNSS lifecycle and module height
coordination. Core equations and reset primitives are unchanged. Tests cover
both height sources, strict 500 ms timing, held-fault versus reset independence,
quality rejection, invalid health time, raw/selected diagnostics and real reset
counters, including successful limited fusion followed by a required reset.

Eight native and eight ASan/UBSan suites passed (both backends, float/double,
leak detection enabled). All 43 selected replay-related checks passed; two
initially missing native test executables were built and rerun separately.
Pinned PX4 compilation, 8 package tests, 71 installed-file checks, formatting
and unix32/unix64 Cppcheck passed. No Lean/ESBMC proof was added.

The final `output/px4-vertical-recovery-sitl-01/` passed all 18 independent flight
checks and all 20 ground fault/recovery scenarios. Position/velocity/attitude
RMSE were 0.163 m / 0.257 m/s / 1.659°; valid estimates were 100%, with no
numerical faults, airborne failsafes or log dropouts. These are integration
regressions, not physical in-flight acceleration-fault experiments; bounded
fault branches and reset boundaries are exercised by native component tests.
