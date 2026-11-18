# R-BIAS-PROCESS-NOISE: variance-dependent bias driving noise

**ID:** `R-BIAS-PROCESS-NOISE`

**Status:** Draft

**Applies to:** Portable runtime policy; integrations supply timestamps, metadata and deployment parameters.

**Uses:** [E-NOISE](../../core/process-noise.md).

## Required behavior

`runtime::inhibit_bias_process_noise(P, step_noise, output_noise)`
selects INS process noise for one prediction. Its rule follows PX4 v1.16.2
[covariance prediction](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/covariance.cpp#L145):

| Body axis i | Keep the configured intensity only when |
|---|---|
| Accelerometer bias | `P(9+i,9+i) < specific_force_variance(i)` |
| Gyroscope bias | `P(12+i,12+i) < angular_rate_variance(i)` |

- Otherwise set only that bias intensity to zero. Preserve both IMU measurement
  variance vectors and every other bias axis. No new threshold parameter is
  introduced. Zero variance/intensity is valid; equality inhibits the increment.
- The INS bias transition rows are identity and sampled measurement noise has
  no bias rows. Thus prior bias diagonals equal their propagated values before
  bias noise, including with nonzero cross-covariances. Reading the prior avoids
  a second prediction. This is scoped to this INS model, not arbitrary dynamics.
- Validate all twelve noise coefficients and six consumed bias diagonals as
  finite/nonnegative, even on inhibited axes. Non-finite input returns
  `non_finite_input`; negative input returns `domain_error`. No universal
  precedence is promised for multiple independent invalid inputs. Other entries
  of P are not read here; valid symmetric PSD covariance remains the prediction
  caller's premise. Failure preserves the whole output, including an alias of
  configured noise; success permits that alias. State/P are never changed here.
- Start every step from configured noise, apply `R-IMU-PROCESS-NOISE`
  when fault/clipping metadata is available, then apply this selector. A correction
  that reduces variance below the selected threshold releases inhibition immediately. This rule takes
  no learning permissions, motion/clipping or enable flags directly; fault and
  clipping influence its thresholds through the supplied IMU variances.
- This is a pre-increment check, not a clamp: one permitted increment may cross
  the threshold. The next step inhibits further bias noise while variance stays
  above it; an already-high covariance is not reduced. No hard covariance bound,
  physical sensor-health claim or bias-value limit follows from this rule.

The threshold quantities have matching units: m²/s⁴ for accelerometer bias and
sampled specific force, rad²/s² for gyro bias and sampled angular rate. Preserve
`E-NOISE`: the selected continuous driving-noise intensity is multiplied by `dt`.
PX4's bias-driving parameters instead have units m/s³ and rad/s² and contribute
`(sigma * dt)^2`. At a fixed dt, matching that contribution would require
`intensity = sigma^2 * dt`; there is no constant conversion across variable dt.
This feature aligns the threshold decision, not parameter values or discretization.
Do not copy PX4's bias-noise numbers into our intensity fields unchanged.

Native backend, threshold, rollback,
correction/recovery and replay tests support the rule. The `imu-policies` ESBMC
suite verifies the actual portable selector, both scalar formats/aliases, every
consumed coefficient, equality behavior, unchanged P and failure publication.
Unused P entries remain arbitrary, not assumed finite or PSD by the harness.
Lean proves nonnegative selected noise, exact identity bias transition rows and
exact PSD preservation under the existing prediction premises. These results
do not prove a hard covariance bound, IEEE PSD preservation or caller sequencing.
Full PX4 covariance constraints remain separate work.

## Caller bindings

- [R-REPLAY-BIAS](../../integration/replay/imu-bias.md).
- [R-PX4-IMU-BIAS](../../integration/px4/imu-bias.md).
