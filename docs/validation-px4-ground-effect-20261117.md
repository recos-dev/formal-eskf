# Barometer ground-effect protection validation

F-03.2 implementation prepared for review on 2026-11-17, based on `65f8dc0`.
PX4 source/build: v1.16.2, `54f0455ffcd755534539a7cf33a09a20bf71d29d`.
The [ground-effect contract](spec/verification-requirements.md#px4-module-barometer-ground-effect)
owns behavior and differences from the reference. Implementation checks pass.
Follow-up isolation identifies SIH's contact condition as the penetration
mechanism; **repeatable landing acceptance on unmodified SIH remains open**.

## Scope and review

The module now retains ground-effect and fixed-wing flags in delayed context,
updates the no-terrain activation/10 s hold policy before height fusion, and
reports protection through PX4 status flags. Height fusion supplies a fixed 4 m
deadzone to barometer correction only. The runtime applies it after raw gating
and the acceleration-fault clamp, preserving raw diagnostics and rejection.
GNSS and barometer offset learning are unchanged. The feature changes no ESKF
core equation, state dimension, simulator source, acceptance threshold or
verification script. Temporary simulator experiments are distinguished below.

Self-review checked sign conversion, gate/limit/deadzone order, covariance update
at zero residual, source isolation, historical activation, unchanged-context
handling, strict timeout equality, landing/fixed-wing exits, missing metadata
and output status. The implementation follows PX4's no-terrain branch while
requiring fresh vehicle status rather than assuming a false fixed-wing flag
when unavailable. It does not substitute local altitude for unknown HAGL.

## Results

Evidence is local under `output/px4-ground-effect-20261117/`. Its
`evidence-manifest.json` binds source, configuration, binaries and logs by SHA-256;
the flight retains its own installed-package and binary provenance.

| Check | Result / scope |
|---|---|
| PX4 build and native module | 102 scenarios, zero failures; actual module/uORB path with controlled time |
| Module ASan/UBSan | 102 scenarios, zero failures; four module translation units and tests instrumented, external PX4 libraries uninstrumented; leak checking disabled |
| Native components | 6/6 pass: navigation measurement, height management and observation history, each on Eigen and PX4 Matrix |
| Component ASan/UBSan | Same 6/6 pass |
| ESBMC `navigation-recovery` | 28/28 profiles pass; binary32/64 height callers now include symbolic zero/positive/invalid deadzones and all four state/P alias modes |
| ESBMC cost | 664.25 s elapsed, peak RSS 1,075,632 KiB (1.03 GiB), zero swaps; sequential execution |
| Cppcheck | unix32/unix64 pass; 173 active checkers, 13 intentionally inactive; configured core/test/component scope, not live PX4 translation units |
| Packaging / formatting / references | 8/8 package tests, check-only formatting and whitespace checks pass; 1,278 references resolve |
| SIH live flight / ground cases | Route, landing/disarm sequence and 20/20 ground fault cases pass |
| Independent SIH truth analysis | **17/18; `landed_near_ground` fails** |

Numeric tests independently check both residual signs, exact 4 m boundaries,
raw rejection that stays rejected even after reduction, acceleration limiting
before the deadzone, unchanged gain/Joseph covariance, invalid configuration
rollback and GNSS/bias isolation. Module tests exercise delayed activation,
production pressure-transient correction, both status publications, 10 s equality
and expiry, no refresh from repeated snapshots, fixed-wing/landing and stale-status
exit. Existing disabled/default-zero cases remain in the suites.

The symbolic caller proof retains its existing gate/limiter/correction contracts;
it is not an end-to-end proof of the module, backend internals, clock/context
history or physical rotor wash. No new Lean theorem or full AI audit is claimed.

## Retained landing failure

The unchanged harness flew the 5 m / 8 m square / yaw-change route and logged
210 status samples with ground-effect protection active (1110 total, nominal
10 Hz status logging). This confirms activation, not physical mitigation quality.

| Independent metric | Result |
|---|---:|
| Position 3D RMSE | 0.235 m |
| Velocity 3D RMSE | 0.338 m/s |
| Attitude RMSE | 1.667° |
| Valid airborne output | 100% |
| ULog dropouts | 0 |
| Final truth height | **-0.660 m**, outside the unchanged ±0.5 m limit |

`flight-01/landing-investigation.json` shows the true vehicle crossing the ground
plane at 105.752 s with 0.808 m/s downward velocity and upward acceleration.
It continues below the plane until 106.652 s, where simulated contact cancels
velocity and leaves it 0.660 m below ground. The truth reference altitude is
constant. Pinned SIH's `equations_of_motion` requires both negative height and
downward net force for contact; it does not project position back to the ground.
The trace is consistent with that rule delaying contact during upward braking.

The same failure category was already retained before this feature in
[F-04 validation](validation-px4-delayed-fusion-20261117.md#closed-loop-sih-results-and-retained-failure).
That prior result alone did not establish causality. The original failed run is
retained, with its unmodified SIH and oracle. The follow-up below isolates the
contact mechanism without replacing that result. Physical tuning and hardware
acceptance also remain open.

## Follow-up isolation

On 2026-11-17, four additional stock-SIH flights used the same harness, board,
environment and mission, in ON/OFF/OFF/ON order, with fresh rootfs directories.
The OFF export changes only `ground_effect_deadzone_m` from 4 to 0; activation
policy and status reporting remain present. Package hashes confirm that single
source difference. These are matched-scenario closed-loop runs, not identical
sensor-input replay; wall-clock phase timing and scheduling can differ.

| Run | Deadzone | Truth landing height | Unchanged checks |
|---|---:|---:|---|
| `on-01` | 4 m | -0.6191 m | 17/18; landing height fails |
| `off-01` | 0 m | -0.5914 m | 17/18; landing height fails |
| `off-02` | 0 m | -0.5896 m | 17/18; landing height fails |
| `on-02` | 4 m | -0.0028 m | 18/18 |

All four retain a constant truth altitude reference, zero landing-period
height/vertical-velocity resets and zero ULog dropouts. In the three failures,
the vehicle crosses the ground while braking upward and continues below it for
0.82–0.86 s before stopping. In the successful run, acceleration immediately
before contact is downward. This agrees with the original force-sign condition.
The deadzone is therefore not necessary for penetration to occur. It still
changes estimator/controller trajectories; this small sample establishes neither
zero influence nor a quantitative protection benefit.

Two causal checks supplement the flight comparison:

- The exact SIH contact/translational-integration block was compiled in an
  isolated aligned-frame point-mass harness, without an estimator or controller.
  At 0.8 m/s downward and 4 ms steps, a 1 kg body under -0.1 N net Down force
  receives no contact response for one second and reaches -0.7490 m. With
  +0.1 N it stops at -0.0038 m. Reversing force afterward stops the penetrated
  body without restoring its height. This is a contact-rule counterexample,
  not a proof of equivalence to the complete ECEF simulator.
- One diagnostic SIH build additionally permits contact when below ground,
  previously airborne and descending: `!_grounded && _v_N(2) > 0`. The production
  estimator remains unchanged at 4 m. The vehicle approaches at 0.8063 m/s while
  braking upward, stops near the plane, and passes 18/18 at **-0.0034 m**.
  A preceding, broader velocity-only intervention prevented normal takeoff
  (4.469 m tracking error); that aborted attempt is retained and excluded from
  acceptance evidence. Neither intervention is adopted as a simulator fix.

The SIH source and production binary were restored byte-for-byte, with matching
SHA-256 hashes. Installation verification and all 102 native module scenarios
pass after restoration. No production source, numerical model or acceptance
threshold changed during this investigation.

**Review disposition:** the ground-effect implementation is suitable for a
scoped commit; the simulator contact defect is independently reproducible.
F-07 still owns resolution of repeatable landing acceptance on the pinned
simulator. The diagnostic 18/18 result does not close that acceptance gap.
The default barometer simulator has no rotor-wash pressure model, so these
flights also do not establish physical mitigation effectiveness or tuning.

Evidence is retained under the original evidence root's `isolation/` directory:
`protocol.json`, per-flight provenance/logs, `comparison.json`,
`landing-comparison.png`, the extracted contact kernel, both intervention
patches/command records, and `manifest.json`. The manifest binds these artifacts
and the reviewed source; prior evidence is retained separately.

## Reproduction

Use the existing [module developer checks](../integrations/px4/README.md#production-module-tests)
and `./scripts/verify_esbmc.sh navigation-recovery`. Component runs use
`build/px4-imu-history` and `build/px4-imu-history-asan` with CTest regex
`(navigation_measurement|px4_height_aiding|px4_observation_history)`.
Module sanitizer compile/link commands are retained in `module-asan/commands.json`
inside the evidence directory. Flight commands are unchanged:

```bash
python3 integrations/px4/sitl_flight.py --px4 ../px4-test/PX4-Autopilot \
    --output output/px4-ground-effect-review --check-faults --startup-timeout 180
python3 integrations/px4/analyze_flight.py output/px4-ground-effect-review
```
