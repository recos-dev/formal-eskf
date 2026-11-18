# PX4 SIH 250 Hz Deployment Profile

## Summary

Bind the 250 Hz SIH delayed-fusion profile under
[PX4-TIME, PX4-OUT and PX4-RT](../architecture/px4-module.md). This design
baseline is implemented by the live SIH module. Native/SIH evidence is configuration-specific;
source formal proof and target timing/resource acceptance remain separate.

Use the same reference as the architecture: **PX4 v1.16.2,
`54f0455ffcd755534539a7cf33a09a20bf71d29d`**. The first profile targets the existing
250 Hz SIH setup, INS/binary64 and `Px4MatrixBackend`. Hardware and other sensor
rates need their own profile; these values are not universal flight tuning.


Numeric tuning in the module-binding requirements denotes the default deployment
profile unless explicitly identified as an invariant or bound. Only settings
exposed under [PX4-CONFIG](../requirements/integration/px4/configuration.md) may
be selected at build/startup; use the packaged configuration and
[configuration guide](../../guides/configuration.md) to identify effective values.
Changing a tunable does not waive validation, temporal ordering, atomicity or
resource limits, and does not extend proof evidence to the new configuration.

## Startup parameters and fixed storage

Make timing configurable through the module's own startup parameters. Read them
once, derive the history lengths, and freeze the resulting configuration for the
session. The backing capacities remain compile-time constants; changing an
active length never allocates memory or changes the reserved object size.

| Setting | Configuration rule |
|---|---|
| Requested main-filter interval | Startup parameter, initially 4000 us; accept only an implemented interval-realization mode with its numerical and timing evidence |
| Requested minimum fusion lag | Startup parameter, initially 200 ms; derive the IMU/output active length from this value and the selected mode's minimum endpoint spacing |
| Sensor residual delays | Startup settings with explicit units and producer bindings; zero in the initial SIH profile |
| Maximum lag, admitted rates and history capacities | Deployment-profile bounds; parameters must fit these bounds and the memory/response budget |
| Active history lengths | Derived once from the timing/rate bounds; not independent user knobs |

Validate the complete configuration before starting estimation: units and
overflow-safe conversions, supported interval mode, sensor-delay/arrival
coverage, lag bounds, all required lengths and resource limits. IMU and output
histories use the same derived length. Use `RingBuffer::allocate(length)` only
during startup/restart, after validation. Reject unsupported values or required
lengths exceeding capacity; do not silently clamp them, shorten history or fall
back to another period. Running parameter changes take effect only after restart.

For the initial mode, require the minimum fusion lag to cover the largest
`source arrival-age limit + residual delay + maximum half-period offset`.
The half-period bound is 2.1 ms. This puts admitted observations no earlier than
the delayed horizon at admission; later selection still applies its age window.
Together with the 64-slot capacity, integer-millisecond fusion-delay settings
with zero residual delays can range from 153 through 239 ms. This is configuration
admission, not a timing/accuracy qualification of every setting.

The values below describe the **default configuration**, not fixed lengths for
every configuration. A 10 ms setting needs rotation-aware accumulation and
equivalent-noise support before it can be accepted. Merely exposing a parameter
does not implement that mode; the first one-interval mode retains the 4 ms profile.
The module binds `FESKF_DT_US`, `FESKF_DELAY_MS`, `FESKF_GPS_DLY`,
`FESKF_MAG_DLY` and `FESKF_BARO_DLY` once in `load_startup_parameters`.
The first setting is in microseconds; the others are integer milliseconds.
All three owners validate the same settings before callback registration.

## Initial main-filter interval and horizon

The first integration and its acceptance runs use SIH's nominal **250 Hz (4 ms)**
IMU input. The main filter processes one real `vehicle_imu` interval per step;
the output predictor advances on every admitted interval. Support for 400/500 Hz
profiles is deferred, while startup configuration and derived lengths remain
part of the design.
Use a separate current-time output state and delayed main state even though their
nominal update rates are equal. PX4 permits this prediction period; its default
is 10 ms. Do not introduce IMU downsampling in this first integration.

This preserves the existing per-sample `E-PRED`/`E-NOISE`/`E-PCOV` contract. PX4's
downsampler rotates accumulated delta velocity into the updated body frame;
our prediction uses the prior attitude. A later longer-period profile must
establish the frame conversion, rotation composition and equivalent noise before
using accumulated increments. The gravity sampler's approximately 10 ms interval
is independent of the main-filter period and remains unchanged.

| Profile setting | Default value / rule |
|---|---|
| Real IMU integration periods | Angle and velocity periods each 3.8–4.2 ms; retain both measured values |
| Consecutive admitted IMU endpoint spacing | 3.8–50 ms; strictly increasing |
| Integration/endpoint discrepancy | Retain the existing 200 us tolerance and bounded held-rate gap policy |
| Delayed horizon lag | At least 200 ms, at most 300 ms, checked using actual timestamps |
| IMU/output active history | Derived: 54 endpoint records each by default; 64 fixed backing slots each |
| Observation selection age | Strictly less than 100 ms behind the fusion horizon |
| Residual sensor delays | Zero for this SIH profile; explicit per-source values in any later profile |

The 3.8 ms minimum bounds storage demand; it is a deliberately narrower
deployment envelope than the core's accepted time-step domain. Out-of-envelope
IMU data must not be clamped into the profile. The nominal 4 ms period is supported
by the existing SIH configuration, but the full envelope still requires testing.

For the selected mode's minimum endpoint spacing `T_min` and requested minimum
lag `D_min`, derive `L_imu = max(2, 1 + ceil(D_min / T_min))`. `T_min` is an
enforced bound on completed IMU-history endpoints, not simply the requested or
average filter period. The default configuration gives **54** records. The oldest
is nominally **212 ms** behind the newest, and at least **201.4 ms** behind under
the minimum spacing. This is a fixed history length, not exactly 200 ms on every
tick. For example, requesting 230 ms with the same 3.8 ms minimum needs 62 slots;
240 ms needs 65 and must be rejected by this 64-slot build. These counts alone
do not establish acceptance of the other startup checks. Jitter and permitted
gaps must still keep the actual span within the profile's 300 ms upper bound;
exceeding it faults the session rather than skipping unprocessed history.

## Source timestamps and admission

All times are unsigned microseconds in PX4's monotonic boot-time domain. Retain
the producer's raw sample time, effective selection time and arrival time
separately. Reject zero timestamps, arithmetic underflow, duplicate/out-of-order
source samples and sample times later than the arrival clock. Maintain source
watermarks across an empty observation buffer.

For sensor delay `d` and interval-centering offset `h`, use
`t_observation = t_sample - d - h`. Following PX4, `h` is half the main-filter
average real integration period, rounded down to integer microseconds. Initialize
that average from the selected mode (4 ms by default); update it with the existing
period averaging weights using real intervals only. Freeze `d` and `h` with each
admitted observation; never retime records already buffered. The first profile
therefore has `h` near 2 ms.

The half-period shift is an explicit timing approximation: a selected observation
can physically refer to up to `h` after the filter endpoint. Selection and delayed
policy clocks use `t_observation`; diagnostics retain `t_sample`. Do not describe
these as identical physical epochs. Future buffered observations are future
relative to the fusion horizon, not permission for a future boot-clock timestamp.

| Input | Sample epoch / interval | Residual delay | Arrival-age limit | Minimum admitted spacing |
|---|---|---|---|---|
| IMU | `vehicle_imu.timestamp_sample`, interval endpoint; separate `delta_angle_dt` and `delta_velocity_dt` | None | Existing output-age limit: 100 ms | 3.8 ms between endpoints |
| GNSS | `vehicle_gps_position.timestamp_sample`; retain position/velocity from the same message | 0 | 150 ms | 50 ms (at most 20 Hz) |
| Magnetometer | `vehicle_magnetometer.timestamp_sample`; producer's averaged raw-sample epoch | 0 | 100 ms | 10 ms (at most 100 Hz) |
| Barometer | `vehicle_air_data.timestamp_sample`; producer's averaged raw-sample epoch | 0 | 100 ms | 5 ms (at most 200 Hz) |
| Flight/rest context | Original land/status `timestamp`, plus the IMU endpoint at which it was known | No half-period shift | Existing source-specific freshness limits | One context snapshot per admitted real IMU |

Observation rate limits apply to **effective** timestamps, after offsetting.
They are enforced admission limits, not assumptions about uORB publication rates.
Read at most one available update per source per callback; count generation loss,
rate rejection, stale arrival and buffer overflow separately. Identity/calibration
changes and explicit source-invalid indications must be handled even when a
payload is rejected by the rate limiter.

**GNSS differs from pinned EKF2:** its GNSS adapter uses `timestamp`, then subtracts
`EKF2_GPS_DELAY` (default 110 ms) and half a filter interval. Our existing adapter
uses `timestamp_sample`; SIH supplies a sample epoch, and its airframe sets GNSS
delay to zero. Do not subtract 110 ms again or silently fall back to `timestamp`.
A producer without a trustworthy sample epoch needs a reviewed binding and delay.
Receiver UTC is never a replacement for boot time.

The proposed arrival limits fit inside the minimum retained lag, leaving margin
for admission and interval quantization. They are testable acceptance limits,
not measured physical sensor delays. Do not infer callback rates from decimated
ULog rows or treat publication-minus-sample age as calibrated physical latency.
Logs/producers without these sample fields require a separate timestamp binding.

## Histories and ownership

Use the existing [runtime ring buffer contract](../requirements/runtime/ring-buffer.md).
Store internal values and provenance, not whole uORB messages. Each observation
carries source/calibration identity and the timestamps needed above. No new
generic sensor registry or buffer wrapper is required.

| History | Default active / fixed backing slots | Retained content and sizing basis |
|---|---|---|
| IMU | 54 / 64 | One record per real interval, including any preceding permitted held-rate segment |
| Output | 54 / 64 | Current output state captured at each matching real IMU endpoint |
| Context | 128 / 128 | Flight/rest/ground-effect and vehicle-type values, known flags and original context timestamps, sampled at real IMU endpoints |
| GNSS | 16 / 16 | One event shared by velocity, horizontal-position and height consumers |
| Magnetometer | 64 / 64 | Field measurement and source/calibration metadata |
| Barometer | 128 / 128 | Height measurement and source/calibration metadata |

For each aiding source, reserve at least
`ceil((D_max + selection_age + output_age_limit) / minimum_spacing) + 2` slots.
The output-age allowance covers source admission while IMU delivery pauses:
a source timestamp can be ahead of the latest admitted IMU, while still no later
than wall time. With a 500 ms retention allowance, GNSS/magnetic/barometer need
**12/52/102** slots respectively; the selected capacities exceed these counts.
The default active lengths use those full capacities. Other configurations must
derive sufficient active lengths within the same backing limits and recheck
context coverage; changing the IMU history alone is insufficient.
This allows oldest/superseded entries to remain until the next bounded selection.
It does not authorize
processing a backlog after an unsupported IMU outage. Once the latest IMU's age reaches
the 100 ms output-age limit, stop queueing fusion observations; arrival/status
monitoring continues. A source-selection miss must not erase its admission
watermark.

Context needs `D_max + selection_age + maximum_endpoint_spacing = 450 ms`.
Its 127 endpoint intervals cover at least 482.6 ms. Seed a context snapshot at
initialization; thereafter retain only information known at each endpoint,
carrying the original message timestamp so repeated snapshots do not refresh
freshness. A context message newer than the endpoint cannot enter that snapshot.
Lookup the newest context at/before the requested effective epoch; missing or
stale context is unknown, not a positive fusion permission.

Arrival validation runs immediately. Source qualification/drift history and
lifecycle run once in effective-time order at the delayed horizon, using the
selected event's historical context. Filter-dependent eligibility uses the
delayed prior; current safety conditions may only veto it. Preserve the existing
quality/gate rules. Superseded observations are not counted as qualified or
fused. At startup, transfer qualification state valid at the initialization
epoch; never replay alignment samples as new corrections.

Budget **64 KiB maximum for these histories, metadata and their retained staging
copies** in this first SITL profile. The module enforces the sum of the three
owner `sizeof` values at compile time; count full backing storage, not active length. Objects belong to the
session, not callback-local stack storage. Core matrices, module objects and the
existing stack reservation are additional memory, measured under F-07.2.

## Initialization and normal advance

1. Validate the profile and prepare fixed storage at startup. Existing alignment
   establishes the main state, covariance, policies and output seed at one epoch
   `t0`. Retire observations with effective time at/before `t0`, while retaining
   admission watermarks.
2. Propagate the output seed with each subsequent real IMU interval and store its
   endpoint alongside the IMU record. Warm up the configured active IMU/output
   length (54 by default).
   Do not publish a valid estimate until alignment, matched history and the lag
   checks pass. Never fill missing history with repeated seed samples.
3. Once ready, predict the main state from `t0` through the oldest IMU record,
   then perform at most one selected event per source. Subsequent callbacks
   advance through exactly the next oldest record, once. Before a full-buffer
   push, verify that the record to be overwritten has already been processed.
4. Preserve the fusion order in PX4-EXEC. Pop GNSS once and pass that event to its
   velocity, horizontal-position and height consumers; no second height pop.
   Advance gravity sampling using real delayed IMU intervals and matching context.
   Check magnetic model/reference validity for that event; source/calibration
   changes cannot reuse pending data from the previous generation.
5. Find output history at **exactly** the completed fusion timestamp before
   constructing feedback. Equal buffer lengths/indices are insufficient. Apply
   tracking correction and updated bias feedback, then publish current navigation
   output. The fast attitude phase precedes this step and uses previous feedback.

For the default configuration and nominal intervals, the first ready tick has
output at `t0 + 216 ms` and the first completed main estimate at `t0 + 4 ms`;
the next tick advances both by 4 ms.
No interpolation, duplicate prediction or loop that drains all pending IMU is
needed in this profile. Output-tracking equations must be checked independently
with our quaternion/frame conventions under R-PX4-OUTPUT-PREDICTOR; timing agreement alone does not
establish their numerical correctness.

## Loss, timeout and reset

| Event | Required action |
|---|---|
| Permitted IMU gap | Carry the previous-rate hold and real interval in one history record. Apply the same segment sequence to output and main prediction, at most two prediction calls each. The hold creates no fresh gravity, clipping or motion-health evidence. |
| Unsupported gap, invalid IMU or lag outside the configured bounds after warmup (default 200–300 ms) | Latch restart-required fault; stop valid publication. Do not skip delayed intervals to recover the requested lag. |
| Required IMU/output record lost or timestamp mismatch | Fault before using a mismatched prior/history. Include owner overwrite checks; `push()` itself does not report loss. |
| Aiding overflow, expiry or ordering rejection | Retire/drop with a reason; preserve other sources. Do not refresh successful-fusion time or reset the whole estimator merely for source loss. |
| No new IMU | Backup callback maintains wall-time arrival/output expiry and status; no synthetic propagation and no advance of delayed policy time. Evaluate delayed lifecycle at the next real delayed step. |
| Sensor/calibration switch | Clear that source's pending observations and requalify; retain ordering/provenance rules for the new generation. IMU identity/calibration change remains restart-required. |
| Successful state reset | Apply the same reset transform to affected current/output-history payloads, tracking accumulators and any vertical auxiliary state before the next affected publication. Preserve timestamps. Report the actual consumer-visible reset delta/counter exactly once. |
| Ordinary fusion/tracking correction | Update tracking/bias feedback; do not increment reset counters. |
| Session restart | Reset histories, processed cursors, timing averages and session policies together; repeat alignment/warmup. No automatic numerical-fault recovery is introduced. |

The callback is not a multi-sensor transaction: earlier successful stages and an
already-published attitude retain their recorded meaning after a later failure.
No subsequent output may claim that failed work succeeded.

## Output epochs and execution budget

Current attitude, position and velocity use the latest admitted IMU epoch;
publication `timestamp` uses wall time. Estimator state/covariance, innovations,
bias and delayed-policy status identify the fusion epoch. Diagnostics retain both
horizons and their lag. Current arrival failures are separate from delayed source
status; they must not retimestamp a delayed estimate.

As in EKF2, navigation uncertainty fields can carry delayed-filter covariance
proxies alongside current output. Document this approximation in the topic
mapping and expose the covariance epoch/lag in diagnostics; they are **not** a
propagated current-time covariance or a certified current-error bound. Rotate
body-local attitude covariance with the matching delayed attitude under
[R-PX4-FRAME-OUTPUT](../requirements/integration/px4/output-frames.md#r-px4-frame-output-geographic-and-rotation-variance-output-frames).
With a nonzero
IMU installation offset these remain IMU-state proxies; the module's body-point
position/velocity transform does not transport their covariance. Suppress valid
output on stale/faulted history; never make delayed uncertainty look current by
changing its diagnostic epoch. An output predictor does not own a second EKF.

Bound each callback to one input per subscribed source, one main-filter step,
one observation per aiding source, and the fixed history scans above. Held gaps
allow at most two prediction segments; they do not permit repeated fusion.
Context scans visit at most 128 entries per lookup; output feedback/reset visits
at most the configured active length (54 by default, at most 64 in this build).
Include bounded source-history clearing on switches.

Retain the existing navigation publication interval (10 ms), maximum output age
(100 ms) and backup interval (20 ms). The proposed response deadline is **3.8 ms**
from IMU callback trigger through completed processing, including queue wait;
the fast attitude phase must finish within this budget before the slow phase.
Record both phase latencies. The deadline follows the minimum admitted interval,
but does not prove that the current work queue/CPU can meet it. F-07.2 measures
simultaneous aiding, reset, gap and fault paths using wall time, not lockstep
simulation time. A missed budget is a failed profile, not permission to silently
drop work or change the main-filter period.

## Review and implementation acceptance

Acceptance of this profile requires evidence for:

- Startup parameter conversion and derived lengths, exact-capacity/over-capacity
  cases, unsupported period modes and restart-only configuration changes.
- Exact warmup/once-only advance, matching output epochs and held-gap replay.
- Timestamp/offset boundaries, future retention, rate limits, late/out-of-order
  events, overflow and unknown/stale historical context.
- One GNSS event feeding all three consumers; gravity/magnetic eligibility and
  source changes using the correct delayed context without duplicate evidence.
- Independent output propagation/tracking and reset cases, including output
  before later-stage failure; correct state, covariance and publication epochs.
- Actual storage accounting and production-path bounded-work checks, followed by
  wall-time/stack/dependency acceptance for the declared target under F-07.2.

The `R-RING-BUFFER` evidence covers its declared sample type, capacities
and inductive operation invariants. It does not prove these larger capacities, the module's time
arithmetic, temporal ownership, output tracking or scheduling. Extend evidence
to the instantiated paths and their assumptions; do not claim profile acceptance from
component tests/proofs alone.

## Pinned reference points

- [EKF2 timestamps and publication](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp): IMU at line 603, GPS at 2402, barometer at 2143, magnetometer at 2481; odometry uncertainty at 1684.
- [Input timing and buffer lengths](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/estimator_interface.cpp#L79): source delay/half-period adjustment and initialization at line 539.
- [IMU downsampling](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/imu_down_sampler/imu_down_sampler.cpp#L13) and [output history/feedback](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/output_predictor/output_predictor.cpp#L257). Our exact-epoch check is stronger than the latter's equal-length assumption.
- [Prediction/delay parameters](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/module.yaml#L15), [GNSS delay](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/params_gnss.yaml#L20), and [SIH airframe override](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/ROMFS/px4fmu_common/init.d-posix/airframes/10040_sihsim_quadx#L37).
- Producer epochs: [SIH GNSS](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/simulation/sensor_gps_sim/SensorGpsSim.cpp#L153), [averaged magnetic sample](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/sensors/vehicle_magnetometer/VehicleMagnetometer.cpp#L566), and [averaged barometer sample](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/sensors/vehicle_air_data/VehicleAirData.cpp#L268).
