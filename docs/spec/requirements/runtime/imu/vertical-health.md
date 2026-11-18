# R-VERTICAL-ACCELERATION: apparent falling and clipping detection

**ID:** `R-VERTICAL-ACCELERATION`

**Applies to:** Portable runtime policy; integrations supply timestamps, metadata and deployment parameters.

## Required behavior

`runtime::VerticalAccelerationHealth<Linalg>` is a standalone monitor. It owns
clipping counters, falling likelihood and fault timing, not estimator state/P,
noise, bias permissions, fusion decisions or resets.

| Input | Meaning |
|---|---|
| IMU time | Nonzero, strictly increasing estimator-horizon timestamp in microseconds |
| Average interval | Caller-supplied average EKF interval in seconds, not observation cadence |
| Clipping | Known per-body-axis flags accumulated over this IMU interval |
| Six observation slots | Active flag, signed innovation and innovation variance S; pressure height, GNSS height/Down velocity, range height, vision height/Down velocity |

All innovations use prediction minus observation with positive meaning apparent
falling. Negate our core's observation-minus-prediction residual; range callers
must also convert the opposite distance-to-ground direction. The caller supplies
the latest cached diagnostics from active aiding sources, including gate-rejected
observations. Lifecycle owns source activity and data expiry; this monitor adds
no independent freshness timeout and does not treat rejection as deactivation.

- An active, finite, nonnegative variance below **FLT_EPSILON** is ignored. This
  fixed reference floor is used for float and double, not replaced by DBL_EPSILON.
  Otherwise form `innovation / sqrt(S)`, without a gate-sigma divisor or absolute
  value. Any source strictly above 3 gives medium likelihood. A >3 source plus
  a >1 source of a different reference type gives high likelihood; otherwise low.
  Types are pressure, GNSS, and ground: GNSS height/velocity share GNSS, and range
  plus both vision slots share ground. Equality at 1 or 3 does not exceed a test.
- Set `N = trunc(1 / average_interval_s)`. Per axis, increment if clipped and
  count < N; otherwise decrement if count > 0. Preserve the decrement even when
  clipping continues at N, and when a changed N is below the stored count. This
  is not a sliding window or a saturating counter. Warning means count >= N/2
  with integer division; critical means count >= N. `bad_clipping` means all
  three axes warn or any axis is critical. Frequent clipping means body Z warns
  or `bad_clipping` is set.
- `bad_sample` means high likelihood, or medium likelihood with frequent
  clipping. Update last-bad time on bad samples, otherwise last-good time.
  `bad_vertical_acceleration` holds until elapsed time since last bad is >=3 s.
  Every bad sample refreshes the hold. Use ordered timestamp subtraction to
  avoid overflow. Low likelihood/no active sources is absence of evidence, not
  proof of a healthy IMU. Last-good time describes the instantaneous predicate,
  even while the fault is held; it supplies the independent 500 ms reset policy
  in `R-GNSS-FUSION`.
- Validate time first (`out_of_range`), then a finite positive interval
  (`non_finite_input`/`domain_error`) and known clipping (`domain_error` if absent).
  A non-finite reciprocal returns `non_finite_result`; require reciprocal in
  [2,65536) before uint16 conversion (`out_of_range` otherwise). This keeps the
  conversion defined and the half-limit nonzero. For each active observation,
  require finite innovation/variance and nonnegative variance, even below the
  floor. Non-finite input returns `non_finite_input`, negative variance returns
  `domain_error`, and non-finite computed ratio returns `non_finite_result`.
  Inactive observation coefficients are not consumed. Scalar-root failures
  propagate. Source validation follows slot order: barometer height, GNSS height,
  GNSS Down velocity, range height, vision height, vision Down velocity. Return
  the first failure; already sufficient falling evidence must not skip later
  active sources. Validate all sources before publishing counters or timestamps.
- Failure preserves every stored field. The stored state then belongs to the
  previous successful timestamp and is not a verdict for the failed sample.
  Unknown clipping must not be converted to clear. Reset clears all history;
  callers restart on IMU gaps, unknown-metadata interruptions or source/calibration/
  frame changes. After reset, last-update zero means no evaluation yet.

Reference: PX4 v1.16.2 [height health monitor](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/height_control.cpp#L122)
and its [source-combination tests](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/test/test_EKF_accelerometer.cpp#L111).
Alignment covers valid inputs and finite arithmetic. Explicit invalid-input
rejection, missing-metadata handling and overflow-safe time subtraction are our
API safeguards; no matching invalid-data behavior or bitwise float/double parity
is claimed. The caller must match filter cadence, averaged interval and source
diagnostic ordering for matching timelines. PX4 evaluates after GNSS velocity
and before height fusion; bias/noise prediction consumes the preceding fault.
Native tests cover source combinations, signed residual conversion, counters,
recovery and failure atomicity. The `vertical-health` ESBMC suite additionally
checks initialization/reset, the actual likelihood producer's enum/failure frame,
and the actual public update under that frame, in binary32/binary64. It establishes
ordered sample validation, clipping-count recurrence, verdict consumption, atomic
history and inductive timestamp/three-second-hold invariants. Arbitrary samples,
including failures and timestamp boundaries, remain admitted; the prior-history
premise is established at initialization and preserved by update/reset.

The actual likelihood producer also has binary32/binary64 source-equation checks:
ordered source/scalar arguments, the fixed FLT_EPSILON floor, signed IEEE division,
strict thresholds, distinct-reference grouping, first-failure priority and atomic
publication. Every IEEE root return remains admitted independently, including
negative, zero and non-finite values. The observer preserves scalar returns and
production state. A separate protocol proof establishes its prefix invariants
without assumptions; the classification and ratio profiles reuse exactly those
proved invariants. They do not assume their numerical conclusions, successful
inputs or successful results. Default safety and complete-unwinding checks remain
enabled in all profiles; the ratio profile uses non-incremental multi-property
solving to bound memory.

Scalar-root accuracy, physical detection validity and module/replay sequencing
remain **open**. No new Lean algebra is needed for the scoped state-transition
properties. See the [verification scope and evidence rules](../../../../guides/verification.md).

Replay integration:

- Evaluate once per real IMU prediction, including ticks without GNSS messages.
  Require the GNSS quality/lifecycle profile and known clipping. Legacy inputs
  without that profile report unavailable; do not infer source activity from a
  correction count. Barometer/range/vision slots remain inactive. GNSS height
  and velocity alone can only give medium likelihood, requiring clipping to
  trigger a fault.
- Use actual GNSS correction diagnostics, including gate-rejected results:
  negate the selected `fusion_innovation` and retain S. A permitted limit is
  reflected in the monitor cache, as in the reference's aid-source status;
  raw residuals/ratios remain in the fusion log. While velocity aiding remains
  active after a quality rejection, evaluate its velocity gate/limit for the
  cache without correcting the estimator.
  Invalid receiver velocity is not a new received sample; unavailable quality
  metadata invalidates the velocity cache. Numerical failures abort replay.
- Take the diagnostic snapshot after current velocity processing and before
  current height processing. It contains the current velocity cache and the
  preceding height cache. Height activity is from before height control; velocity
  activity reflects current horizontal control. Inactive sources are excluded;
  caches are cleared after lifecycle stop. Successful full velocity/height resets
  set that cache's innovation to zero and S to observation variance, as in the
  reference's aid-status reset. A Down-only reset retains velocity diagnostics.
- Evaluate the snapshot before height fusion, so height can consume the current
  fault. Prediction, bias permissions and every velocity event at this horizon
  consume the preceding horizon's fault. For multiple due GNSS messages,
  re-evaluate each pre-height snapshot from the same saved preceding monitor
  state and retain the final result. Clipping therefore advances once, while
  every height attempt sees its current pre-height evidence. This batching
  remains a replay approximation, not EKF2's receiver-buffer schedule.
- Initialize nominal/average interval from the seed IMU integration period.
  For each actual sample update `average = 0.99*average +
  0.01*clamp(dt, 0.5*nominal, 2*nominal)`. Accel/gyro periods are synchronized in
  replay. The averaging rule follows the reference; replay's logged prediction
  cadence replaces EKF2's configured downsampled cadence. Do not use GNSS dt.
  Missing intervals/unknown clipping clear monitor, diagnostic caches and average
  history before consuming a prior verdict. Held gap predictions never increment
  clipping counters. Restart from the next actual sample with known metadata.
- `vertical_acceleration_health.csv` reports availability, interval, counters,
  likelihood (0/1/2 = low/medium/high), current fault/timers and the signed input
  diagnostics with source timestamps. `bias_learning.csv` reports the preceding
  verdict used for permissions. Summary counts separate evaluated, unavailable,
  fault-held and fault-inhibited ticks. An available low verdict with no active
  diagnostics still does not prove health.

The replay fault now feeds accel bias-learning permissions and the sampled
noise selection in `R-IMU-PROCESS-NOISE`, plus the GNSS fusion exceptions in
`R-GNSS-VERTICAL-RECOVERY`. Its last-good time separately feeds the 500 ms
reset criterion in `R-GNSS-FUSION`, shared by the module height coordinator.
Native replay tests exercise rejection, input ordering, actual bias preservation/
recovery and
metadata interruptions;
this is not a formal proof or flight-accuracy result.
