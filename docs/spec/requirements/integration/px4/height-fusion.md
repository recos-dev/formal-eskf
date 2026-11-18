# R-HEIGHT-FUSION: PX4 GNSS/barometer height management

**ID:** `R-HEIGHT-FUSION`

**Applies to:** Single-owner PX4 INS module; active deployment profile and stated dependencies apply.

## Required behavior

`integrations/px4/module/HeightAiding.hpp` owns the two-source deployment policy;
its `HeightBias` type holds separate scalar sensor offsets, outside the 15-state INS.
The module supplies calibrated MSL altitude (GNSS compensated to the IMU under
`R-PX4-SENSOR-POSITIONS`), source identity, qualified GNSS,
ordered IMU time and velocity-aiding activity. The helper returns applied reset
counts/deltas; the module owns and publishes accumulated reset history only
after the height transaction succeeds. uORB, clocks and estimator ownership
remain in the module. No numerical-core equations or state dimensions change.
The caller retains the core's finite-state, unit-quaternion and symmetric-PSD
covariance preconditions; this helper does not prove or recheck positive semidefiniteness.

- Use the pinned v1.16.2 public defaults: GNSS is preferred height reference,
  barometer is enabled, barometer standard deviation 3.5 m, GNSS standard deviation
  `max(epv, 0.75 m)` capped at 10 m when barometer is active, and 5-sigma height gates.
  Process barometer before GNSS, after horizontal/velocity operations. These are
  fixed deployment settings, not live `EKF2_*` parameter bindings.
- With altitude `h = origin_altitude - p_D`, define each offset as `b = z_h - h`.
  The core observes `p_D = origin_altitude - z_h + b` with variance `R + P_b`.
  Freeze the current observation at the prior offset before training the offset
  for subsequent samples. Only active non-reference sources predict/learn their
  offset; the current reference's offset and variance remain frozen.
- Offset estimation uses a scalar Kalman update with a strict 3-sigma gate,
  0.13 m/sqrt(s) process noise density; prediction and accepted updates constrain
  variance between `1e-8` and `max(R, 1e-4)` m². Persistent signed innovations increase process variance by
  1000: signed-ratio LPF magnitude >0.2 or a one-sided sequence >10 s, with a 10 s
  LPF time constant. Prediction and sequence monitoring use the estimator tick
  interval; their cadence is not a claim of bit-identical EKF2 bias evolution.
- Start GNSS only with caller-qualified, finite data; start barometer after more
  than 20 finite samples, with a reset-only altitude LPF (alpha 0.1). These fixed
  warmup/LPF values follow the reference's 200 ms / 10 ms / 90 ms defaults;
  warmup counts selected events, not arrivals or elapsed wall time. Queue admission
  and selection follow `PX4-TIME-LIVE`. Source, calibration or generation changes
  stop that session and clear its offset history; barometer also clears its fault
  latch and repeats warmup.
  Altitude must lie strictly between -12000 and 100000 m, with finite positive
  measurement variance. An invalid initial GNSS altitude delays module startup
  instead of latching a numerical-core fault.
- GNSS start/re-entry resets height/P to its raw qualified altitude, selects GNSS
  as reference and zeroes its offset. Secondary barometer start aligns its offset
  to the current estimate without resetting navigation. Track session startup
  grace separately from actual successful fusion/reset progress. An invalid or
  unqualified observation stops only that source; it cannot claim fusion success.
- Missing-data deadlines are 400 ms for barometer and 1 s for GNSS, equality
  expired. The helper retains its sample-age checks (200/500 ms); the live adapter
  applies the stricter `PX4-TIME-LIVE` admission and selection limits first.
  Future buffered samples wait; duplicate/stale samples cannot refresh progress. When the reference
  stops, select an active GNSS source, otherwise active barometer, otherwise none.
  Fallback selection alone changes no state/P or reset counter.
- After the current height attempt, a source whose progress is >5 s old stops
  if it cannot reset. A failed barometer is fault-latched when another height
  source still has recent progress; ordinary good samples do not clear that latch.
  A reset is required if all height progress is >5 s old OR available health has
  no last-good acceleration or that last-good time is strictly >500 ms old.
  This predicate is shared with `GnssFusion::VerticalAccelerationTimes`; it is
  independent of the 3 s held fault. Height consumes current pre-height health;
  velocity consumes preceding health. Successful bounded fusion may refresh
  progress but cannot cancel the independent acceleration reset. Unavailable
  health alone cannot request reset. Future/inconsistent health times are rejected.
  Only the current reference with a fresh, qualified sample may reset height:
  GNSS uses raw altitude; barometer uses its filtered altitude minus offset. An outage without data cannot authorize a reset.
- A real height reset preserves origin coordinates, decorrelates Down position
  using the existing checked reset primitive and transports every sensor offset
  by the applied Down delta. Barometer also resets only Down velocity to zero,
  variance 10 (m/s)², when no velocity-aiding session is active. With active GNSS
  velocity, the module uses preceding health and shared height progress for its
  checked Down-only reset after the normal velocity attempt. A Down-only reset
  retains that attempt's diagnostics; a full velocity reset clears its innovation.
  Horizontal values and reset counters are unaffected by either Down-only operation.
- Commit the height tick's state/P, history and counters only if all numerical
  operations succeed. Rejection can update diagnostics/lifecycle but cannot act
  as successful correction. Preserve raw gate evidence after a timeout reset;
  reset metadata is separate. Publish applied Down/altitude deltas with opposite
  signs and matching local/global/status counters, plus aggregate odometry reset
  count. Accumulate multiple reset deltas within a tick. Height validity lasts
  at most 5 s after actual height fusion/reset, separately from session activity.

This profile covers GNSS and barometer with a fixed GNSS preference, including
the bounded IMU-fault innovation exception and independent reset timing.
[R-PX4-BARO-GROUND-EFFECT](barometer-ground-effect.md) owns the ground-effect extension; delayed/current timeline binding
belongs to `R-PX4-DELAYED-FUSION`. GNSS lever-arm compensation belongs to the
module input boundary, not this height manager. Dynamic-pressure correction,
barometer lever arms, range/vision height and configurable reference selection
remain outside this profile. Native/backend tests and closed-loop
SITL are empirical evidence. `HeightBias.lean` proves the real scalar gain,
unclamped posterior covariance (including Joseph equivalence), residual
contraction and offset transport before source-specific re-alignment. It does
not assert that the C++ variance floor always decreases variance.

The `height` ESBMC suite checks actual scalar-offset equations and field frames,
explicit defaults, finite classification, both sources' admission/stop behavior
and read-only timeout/innovation queries. The actual barometer filter/warmup
producer preserves its full field frame and saturates the counter at 21; a
separate IEEE lemma proves the admitted LPF remains within ±131072 m from a
prior within that bound. This bound is a verification invariant, not a new
operational altitude limit. Source branches and both scalar formats
are covered separately. Prediction's internal finite/nonnegative variance,
ceiling and interval premises, and admission's ordered watermark premises must
still be established by the complete public transition. These private producer
checks alone are not an inductive proof of `HeightAiding::update`.

The scoped standard-library models preserve min/max reference selection,
bounded `any_of`, and the two sessions' value construction/copy/index/iteration
semantics. ESBMC 8.4.0's member-name and container initialization/copy defects
are recorded with native controls; no production body is rewritten to obtain
a pass. Model controls do not prove target STL layout, ABI or implementation.
The complete height transaction, history induction, numerical callee composition,
actual module/replay ordering/publication and floating-point error bounds remain
separate obligations. Bias estimates are not independent altitude truth or a
proof of physical sensor health.





## Caller bindings

- PX4 module barometer ground effect: [barometer-ground-effect.md](barometer-ground-effect.md).
