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

## Navigation ownership refactor validation: 2026-11-14

A-01 batch 4 uses `ec85c36` as its baseline. GNSS history is grouped in one
plain record; quality conversion, motion context, velocity diagnostics and
velocity/position execution are named operations. `HeightAiding` retains its
source arbitration, datum/offset history and transactional state/P update.
Admission, barometer preparation and recovery have separate responsibilities;
its sole-use `HeightBias` type is consolidated into that header. Applied height
reset deltas/counts return to the estimator, which owns published reset history.
Fixed settings move to `DeploymentProfile` without changing values or arithmetic.

| Check | Result |
| --- | --- |
| Original/current/installed production module | 78 scenarios passed; 30,382,040 trace bytes identical |
| Native CTest, three backends | 119 passed |
| Module and fixture ASan/UBSan | 78 scenarios passed |
| Package tests; installed identity | 8 passed; 58 files verified |
| Repository Cppcheck unix32/unix64; formatting; pinned PX4 build | Passed |
| Independent ULog checks; ground fault/recovery checks | 18/18; 20/20 passed |
| SIH position / velocity / attitude RMSE | 0.164 m / 0.257 m/s / 1.678 degrees |

The 25 additional production scenarios cover GNSS quality/admission, receiver
changes, context precedence, independent rejection and bias permissions, coupled
resets, degraded-quality health evidence, height fallback/re-entry, source and
calibration changes, combined height/Down-velocity resets and numerical failures.
The same assertions run against baseline/current sources; only private-member
access changes. Traces include complete nominal state/covariance, navigation and
magnetic history/diagnostics/reset data and seven uORB payloads. Native height
tests retain scalar/backend coverage and now account for returned reset results
at the caller, as the module does.

GNSS operations retain partial success if a later group fails; the two-source
height update still commits only after all operations succeed. Observation bias
is frozen before offset learning, barometer precedes GNSS, and velocity consumes
preceding height/health evidence. GNSS admission includes the 500 ms boundary;
height admission rejects its 200/500 ms boundaries. An initial new fixture
incorrectly expected the GNSS boundary to expire; its assertion/input was corrected
against the unchanged baseline before comparison. A native build also caught a
narrowing counter accumulation, fixed with an explicit modulo-256 conversion.
Neither required changing estimator behavior or loosening numerical assertions.

Evidence is in `output/px4-navigation-refactor-20261114/summary.json`. Stack
reports contain only static or bounded frames with unchanged largest frames;
this does not bound the full target call chain. Advisory analysis of the four
module units retains the existing CLI `constParameter` finding and dependency
findings. Sanitizers cover the module/fixture, not all PX4 archives; leak detection
is disabled. Shared core/runtime, replay and Lean/ESBMC bodies are unchanged.
These checks do not establish hardware, concurrency, WCET, delayed-fusion or
complete PX4-RT acceptance.

## Magnetic ownership refactor validation: 2026-11-14

A-01 batch 3 uses `5c781f7`, including the separately committed replay fix,
as its baseline. The module owns magnetic history, estimator heading alignment
and quaternion reset bookkeeping directly. Named operations handle admission,
source/field preparation, correction feedback and reset; `MagneticAiding` is
retired without a compatibility wrapper. Shared core/runtime and replay code,
numerical models, thresholds and caller horizons are unchanged.

| Check | Result |
| --- | --- |
| Original/current/installed production module | 53 scenarios passed; 22,532,722 trace bytes identical |
| Native CTest, three backends | 119 passed |
| Module and fixture ASan/UBSan | 53 scenarios passed |
| Package tests; installed identity | 8 passed; 59 files verified |
| Repository Cppcheck unix32/unix64; formatting; pinned PX4 build | Passed |
| Independent ULog checks; ground fault/recovery checks | 18/18; 20/20 passed |
| SIH position / velocity / attitude RMSE | 0.164 m / 0.258 m/s / 1.640 degrees |

The production fixture grows from 30 to 53 scenarios. It covers strict health
startup, aided/unaided data outages, stale/future/duplicate/invalid input,
source/calibration and flight-context precedence, bias permissions/limits,
field/innovation timeouts, heading consistency, numerical correction/reset
failure, actual host WMM lookup and independent observation/health revisions.
The same assertions run against both versions; only private-member access changes.
Traces include state/P, lifecycle/progress/reset data and seven uORB outputs.

The retired helper's three backend test instances and model/controller integration
cases move to the production module fixture. Its claim is the deployed binary64
PX4-matrix profile, not the retired generic controller's scalar/backend surface.
Portable numerical/runtime and geographic-model tests retain their own coverage.
The single arrival watermark replaces two equivalent healthy-path watermarks;
lifecycle receipt and successful-fusion/reset clocks remain separate. Model lookup
still follows admission, and a later failure retains preceding mutations and
prevents normal continuation. The two alignment-stop checks remain at their
distinct pre-filter and post-feedback phases.

Evidence is in `output/px4-magnetic-refactor-20261114/summary.json`, including
source/package/binary identity and compiler stack reports. Reported frames remain
static or bounded, with the same largest numerical frames; this is not a complete
target call-chain bound. The advisory four-unit module Cppcheck scan retains the
pre-existing CLI `constParameter` finding and upstream/generated-header findings;
it is not a clean module-analysis claim. Sanitizers instrument the module and
fixture, not all PX4 archives, with leak detection disabled. Controlled-time tests
do not prove work-queue concurrency or shutdown. No new Lean/ESBMC proof, hardware
qualification, delayed-horizon equivalence or completion of PX4-RT is claimed.

## Replay magnetic alignment fix validation: 2026-11-14

A-01 batch 3's prerequisite corrects replay wiring against baseline `2c8815d`:
forward owned heading alignment and clear it when an active session stops
without horizontal aiding. Actual reset feedback restores alignment. Core,
runtime and PX4 module code, numerical models and settings are unchanged;
magnetic structural refactoring remains separate.

| Regression | Baseline reset count | Corrected reset count |
| --- | --- | --- |
| Unaided data outage and return | 0 | 1 |
| Horizontally aided data outage and return | 0 | 0 |
| Field-rejection stop, then GNSS recovery before magnetic recovery | 0 | 1 |
| Rejected-fusion stop, then GNSS recovery before health qualification | 0 | 1 |

The three missing-reset assertions fail against the saved original executable.
The corrected magnetic replay suite passes 26/26, as does ASan/UBSan; the
lifecycle, heading and module-aiding suites pass 9/9 across three backends.
Formatting and repository Cppcheck unix32/unix64 pass. Existing assertions and
thresholds are retained. The 14 existing `R-MAGNETOMETER-LIFECYCLE` references
still resolve; no mapping or proof-body change is required.

Evidence and source/binary identity are in
`output/replay-magnetic-alignment-20261114/summary.json`. The initial sanitizer
attempt failed because LeakSanitizer cannot run under the sandbox's ptrace;
ASan/UBSan were rerun with leak detection disabled. These are host replay and
component checks, not new Lean/ESBMC proofs, SITL or target real-time acceptance.

## Gravity ownership refactor validation: 2026-11-14

A-01 batch 2 uses `95c22cf` as its behavior baseline on the same pinned PX4
host. The module owns tilt alignment and a concrete gravity history/reporting
record, calling the existing sampler/fusion operations directly. The exclusive
`GravityAiding` wrapper is retired. Shared core/runtime and replay source, models,
settings and evaluation horizons are unchanged; magnetic admission is unchanged.

| Check | Result |
| --- | --- |
| Original/current production gravity phase and `Run()` | 30 scenarios passed; 5,870,099 trace bytes identical |
| Native CTest, three backends | 122 passed |
| Replay CLI regressions: bias, vertical health and magnetic reset | 46 passed |
| Module and fixture ASan/UBSan | 30 scenarios passed |
| Package installation/export; installed identity | 8 tests passed; 60 files verified |
| Repository Cppcheck unix32/unix64; formatting; pinned PX4 build | Passed |
| Independent ULog checks; ground fault/recovery scenarios | 18/18; 20/20 passed |
| SIH position / velocity / attitude RMSE | 0.166 m / 0.259 m/s / 1.670 degrees |

The three backend instances of the retired wrapper test are replaced by expanded
production-module tests (14 to 30 scenarios). Portable sampler, tilt and fusion
tests retain their existing backend/scalar coverage; module evidence applies to
its declared binary64/PX4 matrix profile. New cases cover actual tilt correction,
enablement, aided/unaided conditions, rest freshness, clipping, separate periods,
LPF retention/reset, bias inhibition/limiting and alignment/sample/correction
failures. Real `Run()` sequences exercise GNSS loss and metadata rejection.

The private gravity phase relies on `accept_imu()` for valid periods and known
0–7 masks. The retired helper's duplicate period check and unknown-mask branch
are not needed on this call path; invalid metadata still rejects before the
phase and clears observation history. Generic sampler/replay unknown-metadata
behavior is unchanged. The same expanded fixture runs against the original
and refactor, with only private-member accesses migrated; its state/covariance,
gravity diagnostics and seven uORB topic traces match byte-for-byte.

Evidence is in `output/px4-gravity-refactor-20261114/`: `summary.json` records
source/package/binary identity, comparison and check logs; `sitl/` retains the
flight and analysis. The additional four-unit module Cppcheck scan remains
advisory: upstream/generated-header findings and the pre-existing CLI
`constParameter` finding persist; no finding identifies the changed gravity path.
The fixture uses controlled time without a running work queue. Sanitizers cover
the four module translation units and fixture, not linked PX4 platform archives;
leak detection is disabled. No new Lean/ESBMC, concurrency, target resource or
physical-flight claim is made. Existing runtime/module formal gaps remain open.

## Module input boundary validation: 2026-11-14

A-01 batch 1 follow-up against the architecture spec at `4a7478c` keeps the
previous uncommitted refactor and completes its IMU input boundary. `Run()`
decodes the PX4 message once; admission precedes rate division, and downstream
operations consume explicit internal input. Queue settings are named without
changing their values. Core/runtime numerical code and proof bodies are unchanged.

| Check | Result |
| --- | --- |
| Original module versus current production `Run()` | 14 scenarios passed; 3,222,747 trace bytes identical |
| Native CTest, three backends | 125 passed |
| Module and fixture ASan/UBSan | 14 scenarios passed |
| Package installation/export; installed source identity | 8 tests passed; 61 files verified |
| Formatting; pinned PX4 module/executable build | Passed |
| Independent ULog checks; ground fault/recovery scenarios | 18/18; 20/20 passed |
| SIH position / velocity / attitude RMSE | 0.163 m / 0.256 m/s / 1.552 degrees |

The comparison runs the expanded fixture against the actual original module at
`acca7a49c40313abe1dad7f344ce5babe8d5b10d` and the refactor, on the same pinned PX4
host as below. It includes all six FRD components, independent measured periods,
inclusive period limits, rejection ordering and source/calibration changes.
State, covariance, counters and seven uORB topic payloads match byte-for-byte.

Evidence is in `output/px4-module-entry-spec-20261114/`; `summary.json` binds the
source hashes, comparison, build/test logs and `sitl-local/` flight provenance.
The initial build failure from missed output identity-field renames, package-test
CLI error and sandbox UDP restriction are retained alongside successful reruns.
These are bounded regression checks, not exhaustive equivalence or real-time
acceptance. The fixture uses controlled time and does not start the work queue;
ASan/UBSan instruments the module and fixture, not linked PX4 platform archives,
and disables leak detection. No new Lean/ESBMC proof or Cppcheck run is claimed.
The existing mutex, immediate fusion and publication order remain for later work.

## Module entry refactor validation: 2026-11-14

A-01 batch 1 compares the module at `acca7a49c40313abe1dad7f344ce5babe8d5b10d`
with the uncommitted refactor, using PX4
`54f0455ffcd755534539a7cf33a09a20bf71d29d`. The actual original module was
retained before editing; the baseline fixture adds only friend access, with no
behavior/layout change. Both versions execute the same fixture with the same
platform libraries. Core/runtime numerical headers and proof bodies are unchanged.

| Check | Result |
| --- | --- |
| Controlled production module scenarios | 13 passed; original/refactored traces byte-identical |
| Compared trace | State, covariance, status counters and seven uORB topic payloads; 2,954,099 bytes |
| Retained navigation/attitude/output methods | 16 bodies matched after explicit member/settings renames |
| Native CTest, three backends | 125 passed |
| ASan/UBSan, affected helper suites | 6 passed |
| ASan/UBSan, all four module translation units and fixture | 13 scenarios passed |
| Package installation/export | 8 passed |
| Offline agent-review tool regressions | 100 passed; no AI audit |
| Existing Cppcheck unix32/unix64 configuration | Passed |
| C++ formatting | Passed |
| Pinned PX4 module and executable build | Passed |
| Final SITL ground failure/recovery scenarios | 20 passed |
| Final independent ULog acceptance checks | 18 passed |
| Final SIH position / velocity RMSE | 0.173 m / 0.244 m/s |
| Final SIH attitude RMSE / maximum error | 1.426 / 3.674 degrees |

The fixture covers startup parameter validation and accumulation, IMU rejection,
gap/overlap boundaries, independent integration periods, held previous rates,
source/calibration changes, fault latching and watchdog output. It uses real
uORB and production `Run()` with controlled time/timer dispatch. It deliberately
does not start the work queue; expected queue/fault messages are retained.
This comparison is bounded scenario evidence, not scheduler/concurrency proof
or a claim about every possible input. The sanitizer build instruments the
module and fixture, not the linked PX4 platform archives; leak detection was
disabled. No new Lean/ESBMC proof is claimed or executed for this change.

An additional Cppcheck scan of the four module translation units is advisory,
not a clean whole-module result. It reports upstream/generated-header findings
and one existing unused CLI-argument `constParameter` finding in
`FormalEskf::custom_command`. The initial scan could not parse glibc declaration
macros; the retained rerun supplies their actual C++ expansions. It does not
suppress the findings or extend the scope of the passing repository check.

Evidence is in `output/px4-module-entry-20261114/`: `summary.json` records source
hashes and evidence links; `comparison.json`, baseline/final traces, build/test
logs and `retained-bodies.json` retain the controlled comparison. The final
`sitl-final/` contains source/package provenance, binary hash, flight/fault
records, ULog and analysis. Its production package files and binary match the
final refactor; only the test fixture gained additional nonzero-rate cases
after that flight, then passed comparison and sanitizers again.
The first sandboxed SITL attempt could not bind localhost UDP and is retained
in `sitl-run.log`. A successful intermediate flight is in `sitl-local/`; final
acceptance uses `sitl-final/`, which includes the final production changes.
SIH results do not establish physical-aircraft accuracy or real-time budgets.

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
