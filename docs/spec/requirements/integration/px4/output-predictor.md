# R-PX4-OUTPUT-PREDICTOR: current-time output tracking

**ID:** `R-PX4-OUTPUT-PREDICTOR`

**Status:** Draft

**Applies to:** Single-owner PX4 INS module; active deployment profile and stated dependencies apply.

## Required behavior

`px4::OutputPredictor` owns current nominal INS output and matching history under
[the deployment profile](../../../profiles/px4-sih-250hz.md).
It owns neither covariance nor sensor admission, clock, fusion or publication.

- Configure fixed storage and seed a finite state with a valid unit quaternion
  at a nonzero epoch; reject invalid/repeated seed or live reconfiguration without
  mutation. Use the IMU owner's configuration and seed epoch; no synthetic history.
- Consume each admitted IMU interval exactly once, in order. Propagate held and
  real segments using the declared core model and feedback biases; commit the
  endpoint only if both succeed. Never overwrite output awaiting feedback.
- Retain the body angular rate used for the latest real propagation, including
  its bias and tracking correction. Later feedback must not retroactively change
  this rate; failed propagation preserves it and session reset clears it.
- Enforce actual lag bounds and exact delayed/start-epoch pairing. After main
  success, apply the specified attitude-rate feedback, velocity/position PI
  history translations and bias feedback. No interpolation or ordinary-feedback
  reset counters. Main success and valid IMU admission are caller premises.
- Apply a main reset's common left quaternion delta and additive NED deltas to
  all occupied history and current output, preserving epochs and tracking sums.
  A reset before the first interval transforms the seed without creating history.
- Validate all history candidates before committing a correction/reset. On an
  operational fault, preserve current output, retained history, feedback and
  completion cursor, latch the cause, and block use until explicit session reset.
  A session reset clears temporal state and feedback, retaining configuration.
- Use bounded work and fixed persistent storage. Readiness indicates completed
  feedback and no fault; it does not certify freshness, accuracy or safe publication.

Current evidence is native binary64 testing with Eigen and PX4 matrix, including
analytic motion, variable periods, repeated wraps, feedback, reset and numerical
rollback. Lean/ESBMC source evidence, nonlinear stability/error bounds and target
resource bounds remain open. Live binding is owned by [R-PX4-DELAYED-FUSION](delayed-fusion.md).

The tracking structure follows the pinned PX4 output predictor. Let subscript
`d` denote the matching delayed output, `m` the completed main state, and
`lag = (t_current - t_d)` in seconds. Use the shortest-sign, normalized difference:

\[
q_e = q_m^{-1} \otimes q_d,\qquad
e_\theta = -2\,\operatorname{sign}_+(q_{e,0})\,q_{e,v},\qquad
\delta\omega = \frac{0.5}{\mathrm{lag}}e_\theta,
\]

where `sign_+(0) = +1`. This is PX4's small-angle tracking error, not quaternion
Log; the exactly-180-degree choice is ambiguous. Add `delta_omega` to the next
prediction's body angular rate. Do not rewrite past attitudes during ordinary
feedback. For velocity and position, independently:

\[
\bar{\Delta t}_k = 0.8\bar{\Delta t}_{k-1} + 0.2\Delta t_{d,k},\quad
g_x = \bar{\Delta t}_k / \tau_x,\quad
e = x_m-x_d,\quad s_k=s_{k-1}+e,\quad
\delta x = g_x e + 0.1g_x^2s_k.
\]

Initialize the average to 4 ms and both sums to zero. Apply each translation to
every retained output and the current output; commit only after all candidates
are finite. Bias feedback takes effect on the next prediction. Constants live in
`profile::output`; tau_x is the selected velocity or position time constant,
respectively. Both default to 0.25 s and may be configured independently under
[PX4-CONFIG](configuration.md). These PX4 reference defaults are not flight tuning
validated for this module.

`apply_reset` applies the same left quaternion delta and additive NED velocity /
position deltas as the main reset, to the current output and every retained
record. Preserve epochs. Common transforms leave the delayed tracking residuals
and their sums unchanged; preserve them. The caller owns reset counters and must
stop on failure. This operation is not a coordinate-frame rotation of NED vectors.
`reset()` instead clears all temporal/tracking state and fault for a new session.

The intended differences from PX4 are explicit:

- Reuse this project's prior-attitude, constant-acceleration prediction; PX4's
  output predictor rotates its delta velocity with the updated attitude.
- Average the matching real angular periods, not callback wall time. Apply
  attitude feedback as a rate across held and real segments; at a fixed period
  it equals PX4's `0.5 * dt / lag` angle correction. Permitted gaps do not count
  as additional observations or feedback updates.
- Require exact epochs and failure-atomic history updates. Keep local Cartesian
  position; no global-reference recentering, separate vertical-position-derivative
  filter or unaided-yaw output is implemented here. The predictor retains the
  IMU-point state and latest propagated body rate; the module converts to the
  body output point under `R-PX4-SENSOR-POSITIONS`.

History-wide translation/reset uses persistent candidate storage, not a callback-local array.

Borrowed history views are valid only until the next owner mutation; callers shall consume them synchronously.
