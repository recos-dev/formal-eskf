# R-PX4-BARO-GROUND-EFFECT: PX4 module barometer ground effect

**ID:** `R-PX4-BARO-GROUND-EFFECT`

**Status:** Draft

**Applies to:** PX4 module barometric residual policy; compile-time configuration applies.

**Uses:** [R-GNSS-VERTICAL-RECOVERY](../../runtime/gnss/vertical-recovery.md), [PX4-CONFIG](configuration.md).

## Required behavior

Follow the pinned v1.16.2 no-terrain branch when ground-effect protection is
enabled in the build. The module has no terrain or
height-above-ground estimate; local Down position is not a substitute for HAGL.

- Retain `vehicle_land_detected.in_ground_effect` and vehicle fixed-wing status
  in known-at context history. Apply the policy at the delayed main IMU epoch,
  before height fusion, using original message timestamps with strict 3 s
  freshness. Fresh land state overrides armed-state flight fallback.
- A new land/status message carrying a known positive ground-effect indication
  activates protection only in flight and with fresh non-fixed-wing vehicle
  status. Hold until strictly more than the configured duration (default 10 s)
  after its last delayed detection;
  a negative request alone does not immediately clear it. Repeated snapshots of
  unchanged source messages do not renew detection. Landing, fixed-wing state or
  unavailable vehicle/flight context clears the latch; an old detection time alone
  cannot reactivate it. Missing vehicle status is an explicit stricter guard than
  the reference wrapper's default false fixed-wing flag.
- Supply the configured deadzone (default **4 m**, following public
  `EKF2_GND_EFF_DZ`) only to barometer fusion while protected. The internal PX4 parameter
  struct initializes 5 m, but the wrapper binds the public default 4 m. This
  module binds build-time settings, not live `EKF2_*` parameters. A disabled build
  or an inactive protection latch supplies zero deadzone.
- Our NED residual is observed-minus-predicted, opposite to PX4's innovation.
  A transient pressure increase lowers indicated altitude and produces a positive
  residual here. Apply `r_used = max(r_selected - deadzone, 0)` only when positive,
  after the existing bad-acceleration clamp. Negative residuals are unchanged.
  Preserve raw innovation, variance and gate ratio; the deadzone alone never
  authorizes fusion of a rejected observation. A zero selected residual still
  executes covariance correction, matching the reference behavior.
- GNSS, barometer scalar-bias learning, reference arbitration, timeout resets and
  reset counters keep their existing rules. Vertical-health evidence consumes
  the selected residual with the existing sign conversion. Publish protection
  via `CS_GND_EFFECT` / `cs_gnd_effect`, gated by healthy current output; this flag
  is not proof of actual ground proximity or successful barometer fusion.

Reference: [barometer control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/barometer/baro_height_control.cpp#L90),
[mode/timeout](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L945)
and [public parameters](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/params_barometer.yaml#L42).
Native tests cover the actual module path and finite-state timing rules;
`navigation-recovery` covers the scoped residual/correction composition. Physical
rotor-wash response, tuned deadzone, terrain-based activation and whole-module
formal proof remain separate obligations.
