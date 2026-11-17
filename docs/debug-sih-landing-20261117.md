# SIH landing-height failure — 2026-11-17

**The two inspected failures expose ground penetration in pinned PX4 SIH's
contact model.** The existing 0.5 m check correctly rejects those simulated
landings. Neither that threshold nor the original reports has been changed.
No estimator, PX4 checkout or installation was modified; the candidate below is
an isolated experiment requiring review before adoption.

Scope: F-07.1 investigation of the failed landing check recorded by
[F-06.1](validation-px4-confidence-20261117.md). Reference: PX4 v1.16.2
`54f0455ffcd755534539a7cf33a09a20bf71d29d`; formal-eskf parent `7079a4f`, with
F-06.1's existing uncommitted source. Evidence and reproduction scripts are under
`output/sih-landing-debug-20261117/`; `identity.json` records their hashes.

## Existing-log evidence

`analyze_flight.py` checks the final **simulator groundtruth** height `-z`, not
an estimator-relative height. In both ULogs the simulator reference altitude
remains exactly 489.399994 m. The vehicle penetrates that ground plane before
land detection, then stays at the penetrated position. The fault-injection phase
does not explain the failure; these logged flight files end after disarming.

Times below are simulator sample times, not the runner's wall-clock event times.
Groundtruth is logged at 50 Hz, so contact timing has that sampling resolution.

| Existing run | First below-ground sample: time / downward speed | First stopped sample | Height after stopping | Continued descent | Upward-acceleration samples during penetration |
| --- | --- | --- | --- | --- | --- |
| `px4-confidence-20261117/flight-02` | 105.768 s / 0.810 m/s | 106.668 s | −0.663422 m | 0.90 s | 45/45 |
| `px4-confidence-20261117/flight-03` | 105.828 s / 0.801 m/s | 106.648 s | −0.609741 m | 0.82 s | 41/41 |

During penetration, Down acceleration is negative: the vehicle is slowing its
descent but still moving downward. Both runs report ground contact at 107.376 s,
maybe-landed at 107.712 s and landed at 108.048 s, after the simulator has already
stopped below ground. This ordering does not support blaming early land-detector
disarm for the penetration. It does not certify the estimator's entire landing
performance or every historical failure as having this same cause.

## Source and causal reproduction

Pinned [SIH contact handling](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/simulation/simulator_sih/sih.cpp#L427)
applies its ground reaction only when **altitude is below ground AND net force
is downward**. A downward-moving vehicle with an upward net force therefore
continues through the ground. When the force finally turns downward, the model
cancels velocity in one time step; it never projects the penetrated position
back to the ground. These are two separate contact-model omissions.

The isolated probe extracts these four production method bodies unchanged:
`computeGravity`, `equations_of_motion`, `ecefToNed`, `computeRotEcefToNed`.
It verifies the source against the pinned Git object, uses the same PX4 matrix
headers and built geodesy libraries, and supplies a concrete state holder and
controlled external force. It excludes the estimator, controller, scheduler,
aerodynamic force generator, sensors and land detector. Thus it tests the actual
contact/integration statements without relying on another closed-loop flight.

Starting 1 cm below ground with downward speed 0.8 m/s and upward acceleration
0.15 m/s², using 4 ms steps:

| Probe phase | Unmodified SIH dynamics |
| --- | --- |
| After 0.8 s of upward acceleration | Height −0.602020 m; still descending at 0.679995 m/s |
| Next step, net acceleration changed downward to 0.01 m/s² | Stops at height −0.603363 m |
| Independent free-motion displacement | 0.592 m; computed SIH height agrees within 0.1 mm |

This reproduces approximately the observed penetration with **no ESKF present**.
It establishes the contact-model defect as a sufficient cause. Estimator/control
behavior can affect the exact descent trajectory and penetration depth; this
investigation does not claim those paths are otherwise error-free.

## Smallest reviewed repair direction

Keep this outside formal-eskf core and module. For the supported SIH multicopter:

1. Treat incoming downward velocity at/below the ground as contact even while
   the free-flight net force points upward; include the ground boundary itself.
2. Enforce the position constraint at contact in the simulator's geodetic/ECEF
   state, rather than leaving the vehicle below the floor.
3. Enforce the stopped contact velocity explicitly. Preserve the existing
   multicopter full-stop/friction assumption and allow upward force to lift off
   again once no incoming velocity remains.

The isolated candidate passes **10 checks across seven controlled scenarios**:
causal reproduction/free-motion oracle, incoming motion with upward/zero/downward
force, unchanged above-ground motion, unchanged free takeoff, sustained support
and takeoff after a collision. Above-ground and free-takeoff reported height/vertical-velocity values are
identical to the baseline. This is a small host test, not whole-simulator or
real-vehicle qualification.

Self-review rejected a position-only candidate: approximately `8.4e-9 m/s` of
residual downward velocity repeatedly retriggered contact and prevented a later
upward command from taking off. Explicitly satisfying the velocity constraint
fixes that case. Merely adding `velocity_down > 0` to the existing `if` is therefore
insufficient. The initial probe also had a fixture member-initialization-order
error; its failed output is retained and excluded from the final findings.

## Next acceptance boundary

The minimal proposal changes **external SIH physics**, not estimator tuning or
analysis tolerance. Before applying it, review the contact semantics and whether
to carry an explicit host patch or submit an upstream fix. Fixed-wing/VTOL ground
behavior is outside this multicopter candidate. Further acceptance needs contact
boundary/timestep/oblique-motion cases and a clean, versioned SIH flight comparison
with unchanged arming, trajectory and 0.5 m threshold. No full SIH rerun or patch
installation was performed in this investigation.

Local reproduction (existing pinned PX4 build and Python integration dependencies
required; these ignored evidence scripts are not installed tooling):

```bash
python3 output/sih-landing-debug-20261117/inspect_logs.py
python3 output/sih-landing-debug-20261117/probe.py
```

`log-analysis.json` retains the ULog hashes and extracted times;
`probe-results.json` retains commands, source hashes, results and all check values;
`candidate.patch` is a diff between isolated probe sources only. Original
`flight-02/analysis.json` and `flight-03/analysis.json` remain **17/18 failures**.
No code, gate, requirement, backlog, traceability map or existing validation
record was changed, and nothing was staged or committed.
