# R-REPLAY-GRAVITY-SAMPLING: Replay sampling and tuning

**ID:** `R-REPLAY-GRAVITY-SAMPLING`

**Status:** Draft

**Applies to:** Host replay input preparation and same-horizon execution.

**Uses:** [R-ACCELEROMETER-SAMPLING](../../runtime/gravity/sampling.md), [R-ACCELEROMETER-FUSION](../../runtime/gravity/fusion.md).

## Required behavior

| Setting | Selected replay baseline |
|---|---|
| Gravity magnitude | `g = 9.80665 m/s^2` |
| Effective per-axis observation standard deviation | `sigma = g` |
| Observation covariance | `V = g^2 * I_3 = 96.1703842225 * I_3 m^2/s^4` |
| Nominal observation/LPF interval | `10,000 us`; actual intervals must be reported |

PX4 `e1c28e2` uses normalized observations, defaults
[`EKF2_GRAV_NOISE`](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/params_gravity.yaml)
to `1.0`, and squares it for observation variance. Multiplying that variance by
`g^2` matches the local tangential information scale near gravity-sized specific
force, not the complete normalized nonlinear update. This is an effective
pseudo-observation weight, not measured hardware noise or validated flight tuning.
It is separate from prediction's `ACCEL_VAR`; do not multiply V by `dt` or `dt^2`.
The MATLAB reference's enabled `accel_correct1` uses normalized observations and
`V_accel = 0.7 * I_3`, corresponding to `67.31926895575 * I_3` on this same scale;
its cross-product preprocessing is not part of the current model.

The PX4 [nominal EKF period](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/module.yaml#L15)
is 10 ms. Its [IMU downsampler](https://github.com/PX4/PX4-Autopilot/blob/e1c28e27550114e85ddb8760ab3355eaa7a89d8a/src/modules/ekf2/EKF/imu_down_sampler/imu_down_sampler.cpp)
accumulates rotation-aware increments over whole input samples; it does not just
select the latest raw sample every 10 ms. The selected target is not a promise of
exactly 100 Hz or an instruction to fuse each roughly 250 Hz raw input. Fixed LPF
alpha remains `0.1`; its effective time constant depends on actual update intervals.

Use [R-ACCELEROMETER-SAMPLING](../../runtime/gravity/sampling.md) for every real
IMU interval. Reset it on gaps and new sessions; do not supply held predictions.
A sampling failure ends replay. Record actual emitted durations rather than
assuming exactly 100 Hz.

- After that tick's navigation corrections, call `runtime::fuse_accelerometer`
  only when interval clipping is known; its conditions decide whether to correct.
  This uses the agreed V and the current replay bias-learning permissions. A failed
  correction ends replay. Record each observation, actual duration, aggregated
  clipping, LPF diagnostic and outcome (`fused`, `rejected`, `metadata_unavailable`)
  in `accelerometer_updates.csv`; report counts and actual interval statistics.

Missing clipping still blocks correction, unknown rest disables only its override,
and innovation gating remains deferred. The existing legacy ULog and KITTI inputs
lack clipping and therefore do not exercise successful accel correction. These
starting settings and native tests are not calibrated flight parameters or new
Lean/ESBMC evidence. Prediction/measurement reuse correlation, interval-averaging
model error and full PX4 estimator equivalence remain outside the claim.
