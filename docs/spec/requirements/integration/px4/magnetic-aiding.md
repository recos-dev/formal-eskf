# R-PX4-MAGNETIC-AIDING: PX4 module magnetic aiding

**ID:** `R-PX4-MAGNETIC-AIDING`

**Applies to:** PX4 module magnetic caller binding; portable policies retain their own contracts.

**Uses:** [R-MAGNETOMETER-FUSION](../../runtime/magnetic/fusion.md), [R-MAGNETOMETER-LIFECYCLE](../../runtime/magnetic/lifecycle.md), [E-MAG-TILT](../../core/magnetic-tilt.md).

## Required behavior

`FormalEskfAttitude.cpp` connects the existing field check, heading monitor,
lifecycle, gated correction, reset LPF and heading reset for the fixed-field INS.
The module's `Magnetic` record owns sensor history and diagnostics; estimator
members own heading alignment and quaternion reset bookkeeping. Named operations
separate admission, source/field preparation, correction feedback and reset,
using build-selected settings from `DeploymentProfile.hpp`. State/P, clocks, uORB and
serialized execution retain the same module owner. Native tests and SITL are
evidence, not Lean/ESBMC proof of this wiring; numerical core equations are unchanged.

- Read calibrated FRD `vehicle_magnetometer` data, convert gauss to microtesla,
  and bind `device_id`/`calibration_count`. Startup accumulation cannot mix sources.
  Reject future boot-clock timestamps; observations newer than the delayed horizon
  wait in history. Duplicate samples cannot fuse twice. Apply the arrival/selection
  limits in `PX4-TIME-LIVE` without refreshing lifecycle progress on rejection.
  Missing data still advances the lifecycle on each delayed estimator tick.
- The default deployment profile uses the pinned PX4 v1.16.2 settings: 5 microtesla
  measurement standard deviation, 3-sigma componentwise gate, strength-only
  check with 20 microtesla tolerance, healthy duration strictly greater than 1 s,
  at least four samples, no-data stop at 1 s and unsuccessful-fusion timeout
  strictly greater than 7 s. Heading consistency uses 0.3 rad and 0.5 m/s²;
  reset yaw variance is 0.3² rad². The explicit 5 microtesla minimum horizontal
  field is our geometry guard. These are build-time defaults, configurable through Kconfig; they are not live EKF2 parameters.
- Supply horizontal-aiding **session activity**, not output-validity grace flags.
  Fresh land-detector state supplies flight/rest context; fresh arming state is
  the flight-only fallback. Missing context is unknown, never assumed at rest.
  Acceleration history uses actual prediction intervals and is cleared on IMU gaps.
- Initialize the heading-consistency latch only after the existing real startup
  alignment. This creates no successful fusion timestamp. Non-finite/zero external
  field samples stop aiding without pretending the estimator has a numerical
  failure. Numerical failures from correction/reset still latch the module fault.
- Inhibit navigation correction on field rejection or aided heading inconsistency.
  Source/calibration changes reseed the reset LPF; ground changes require a
  qualified heading reset. A stop without horizontal aiding clears yaw alignment;
  returning qualified data must actually reset it before ordinary correction.
  Record success/rejection from the actual transaction, and never correct twice
  on a reset sample. Failed resets cannot change state/P or reset counters.
- Publish separate session (`cs_mag`), heading-aiding (`cs_mag_hdg`), alignment,
  consistency and disturbed-field flags. Keep `cs_mag_3d=false` for this protected
  heading mode. Publish the applied quaternion/heading delta and increment the
  attitude, local heading and estimator-status reset counters only on real reset;
  also increment the aggregate odometry reset counter. Heading output validity
  additionally requires recent progress and the existing variance check.

The observation remains a three-component fixed-field model with compile-time
`ESKF_MAG_TILT=1`; it is not PX4's magnetic-state estimator or an identical scalar
yaw update. Geographic reference updates and heading-source selection follow [R-PX4-MAGNETIC-MODEL](magnetic-model.md). Magnetic-bias learning, AUTO/3D switching and terrain-based
in-flight realignment are not integrated here. Magnetic projection does not claim
that all non-attitude state gains vanish. `R-PX4-DELAYED-FUSION` owns the live
timeline integration evidence; this magnetic contract does not establish hardware
flight performance or prove the complete delayed system.
