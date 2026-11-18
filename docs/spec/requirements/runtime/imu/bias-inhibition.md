# R-IMU-BIAS-INHIBITION: Decaying-peak bias-learning permissions

**ID:** `R-IMU-BIAS-INHIBITION`

**Status:** Draft

**Applies to:** Portable INS runtime policy; caller-established mode/fault conditions.

## Basis

Behavioral reference: PX4 v1.16.2
[IMU bias inhibition](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L971),
[learning limits](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/params_accel_bias.yaml),
and [legacy parameter semantics](https://docs.px4.io/v1.13/en/advanced_config/parameter_reference#EKF2_AID_MASK).

## Required behavior

`runtime::ImuBiasInhibition` owns the two decaying envelopes and returns six
INS correction permissions. The caller supplies the prior unit attitude, biases,
actual mode/fault inputs and separate positive angular/force integration periods.
This policy does not select process noise or detect the truth of those inputs.

- For each sensor interval dt, use decay=0 when dt >= tau; otherwise
  decay=1-dt/tau. Update its peak to max(norm(measurement-prior_bias), decay*old_peak).
  Use angular_interval for angular rate and force_interval for specific force;
  synchronized periods are a specialization. Specific force retains gravity.
  Initial peaks are zero. A peak strictly greater than its limit inhibits learning;
  equality does not. No startup wait or separate continuous-good timer applies.
- Gyro permissions follow the gyro-bias enable switch. Accel permissions require
  its enable switch, no high dynamics, no bad-vertical-acceleration fault, and
  no clipping on that axis. Rest overrides only the observability restriction;
  otherwise fake height inhibits all axes, and fake position permits only axes
  with `abs(R_nb(2,axis)) > 0.966`. These are actual mode/fault inputs, never
  substitutes inferred from GNSS age or sample availability.

- Validate finite IMU/bias data, intervals and parameters, positive intervals,
  limits and tau, and clipping bits 0..7. Invalid input or non-finite norm/rotation
  arithmetic returns disabled permissions and preserves both peaks. The unit
  attitude premise is supplied by the caller. Successful updates publish both
  peaks together. Missing intervals do not invent held samples or erase history.

## Acceptance and verification

Scoped binary32/binary64 ESBMC checks establish the shared
policy's constructor, exact peak decay, finite/nonnegative peak invariant,
permissions, failure atomicity and input purity. The actual norm/rotation producers
are included, with arbitrary scalar-root returns (not an accurate-libm claim).
The caller uses the separately proved peak invariant and checks every callee
premise, including distinct integration periods. ESBMC's incorrect `std::max`
tie model is replaced locally by an explicit standard-library contract model;
this does not prove a deployed STL. Caller metadata parsing, same-horizon
sequencing and forwarding are separate obligations. No Lean-to-C++ refinement or physical
observability claim is made.
