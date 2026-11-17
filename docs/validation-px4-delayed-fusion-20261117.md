# PX4 delayed-fusion integration validation

F-04.2 implementation prepared for review on 2026-11-17, on top of
`6e72a92` (`Add observation and context histories for delayed fusion`).
PX4 reference/build: v1.16.2, `54f0455ffcd755534539a7cf33a09a20bf71d29d`.
The [timing profile](spec/px4-timing-profile.md#px4-time-live-module-binding)
and `R-PX4-DELAYED-FUSION` own the contracts; this is execution evidence,
not a new specification or proof of PX4.

## Implemented scope

- Bind restart-only timing parameters to the three fixed-capacity owners.
  At nominal 250 Hz, 54 real records give a 212 ms delayed horizon.
- Propagate/publish current attitude before delayed work; predict/fuse one
  delayed interval, apply matching output feedback, then publish current
  navigation. Backup callbacks do not advance estimator time.
- Select GNSS once for velocity, position and height; use historical flight/rest
  context for delayed policies. Source generations invalidate qualification and
  height-offset history even across an A-to-B-to-A switch before selection.
- Apply position, velocity and heading resets to current output and retained
  output history before reporting reset counters. Expose both epochs and loss
  diagnostics. Current navigation uncertainty remains a delayed-covariance proxy.

No portable core/runtime equations, observation models, gate thresholds or
simulator acceptance thresholds changed. This is a module timing behavior change,
not a claim of bit-identical output to the immediate-fusion baseline.

## Verification results

Evidence is retained locally under `output/px4-delayed-fusion-20261117/`.
`evidence-manifest.json` identifies source, binaries, configurations and logs by
SHA-256. Each flight also retains its package manifest and binary hash in
`provenance.json`. Generated evidence is not committed.

| Check | Result and boundary |
|---|---|
| PX4 build and module tests | 100 scenarios, zero failures; actual module translation units and PX4 types, strict GCC build |
| Module ASan/UBSan | Same 100 scenarios pass; four module translation units and tests instrumented, external PX4 libraries uninstrumented; shutdown leak checking disabled |
| Component CTest | 8/8 pass: IMU history, output predictor, observation/context history and height manager, each on Eigen and PX4 Matrix |
| Component ASan/UBSan | Same 8/8 pass |
| Height ESBMC | 32/32 existing profiles pass after adding generation to admission/frame assertions; 67.24 s, peak RSS 384,296 KiB (375.3 MiB), zero swaps |
| Cppcheck | unix32/unix64 pass; 173 active checkers, 13 intentionally inactive; configured core/test/component scope, not the live PX4 translation units |
| Packaging | 8/8 tests pass; installed package matches repository inputs |
| Formatting / references | Check-only formatting and whitespace checks pass; 1,276 traceability references resolve |

Native cases include the exact warmup boundary, historical permissions before
prediction, shared GNSS selection, nonzero configured delay, backup/stale output,
delayed gap holds, current-versus-main publication values, actual GNSS/heading
reset propagation, post-attitude prediction failure, source changes and invalid
startup periods/settings. A later test-only extension exercised retained gap
holds at the delayed horizon; the final native and sanitizer runs include it.

The ESBMC result covers the existing scoped height producers/admission/queries,
including the new source-generation field. It does not prove the whole height
transaction or the new live timeline. No Lean theorem changed or was rerun;
no full ESBMC sweep or AI audit is claimed.

## Closed-loop SIH results and retained failure

The unchanged flight harness replaces EKF2, takes off to 5 m, flies an 8 m square
with a yaw change, lands and checks 20 ground fault/recovery cases. Independent
ULog analysis uses simulator truth and the existing 18 acceptance checks.

| Run | Independent checks | Ground cases | Position / velocity / attitude RMSE | Final truth height |
|---|---|---|---|---|
| `flight-02`, intermediate source | 18/18 pass | 20/20 pass | 0.231 m / 0.337 m/s / 1.672° | -0.002 m |
| `flight-final`, intermediate source | **17/18; landing check failed** | 20/20 pass | 0.237 m / 0.279 m/s / 1.064° | **-0.565 m** |
| `flight-accepted`, final production source | 18/18 pass | 20/20 pass | 0.233 m / 0.338 m/s / 1.710° | -0.004 m |

The final flight's production-file hashes match the reviewed source; only the
test-only gap extension followed that flight build. Final status reports a
212,000 us lag, 54 IMU entries, ready output, zero numerical faults and zero
history overflow. Airborne output validity is 100% and ULog has no dropouts.

The failed run is **not resolved by the later pass**. Its true vehicle crossed
the simulated ground plane while still descending at about 0.883 m/s, then
stopped below it. The truth origin remained constant. Pinned SIH ground contact
requires both negative height and downward net force; it cancels velocity but
does not project position back to the plane (`sih.cpp`, `equations_of_motion`).
The logged crossing has upward acceleration, consistent with that contact rule
allowing penetration. This is a plausible explanation, not an isolated causal
proof that estimator/control behavior is irrelevant. `landing-comparison.json`
retains the crossing and settling data. Keep this acceptance anomaly open for
repeatability/ground-contact investigation under F-07. No oracle was relaxed and
no failing log was discarded.

`flight-01` could not start because the sandbox denied its local UDP socket;
the approved retry is `flight-02`. It is not counted as flight evidence. Earlier
native development failures remain in `module-attempt-*.log`; final results use
the final binaries rather than those attempts.

## Review and remaining limits

Self-review checked ownership, admission/selection order, historical context,
reset transforms, publication epochs, failure suppression and source-generation
invalidation against the timing contract. No known implementation defect remains
from those checks. This does not establish exhaustive correctness.

The three history owners reserve 44,952 bytes including fixed backing storage,
metadata and output staging, enforced below the 64 KiB profile budget. Other
module memory and stack are additional. Phase perf counters use PX4 time and
read zero in lockstep; they are not wall-time WCET measurements. Target stack,
worst-case execution time, deadline/load testing and hardware acceptance remain
F-07.2 work. The SIH landing anomaly remains F-07 validation work.

Source formal proofs of IMU/observation histories, output tracking and the live
composition remain open under their existing requirement IDs. Native tests,
sanitizers and the scoped height proofs do not close those obligations or prove
complete EKF2 parity.

## Reproduction

Use the existing developer instructions in [the integration README](../integrations/px4/README.md#developer-checks)
to install/build the pinned PX4 module and run its native test executable.
Run `./scripts/verify_esbmc.sh height` for the scoped proof suite. Component CTest
uses `build/px4-imu-history`, with the ASan configuration recorded in the evidence
manifest. Exact module sanitizer compile/link commands are retained in
`module-asan/commands.json` under the evidence directory.

```bash
python3 integrations/px4/sitl_flight.py --px4 ../px4-test/PX4-Autopilot \
    --output output/px4-delayed-fusion-review --check-faults --startup-timeout 180
python3 integrations/px4/analyze_flight.py output/px4-delayed-fusion-review
```

Use a new output directory to preserve existing attempts and provenance.
