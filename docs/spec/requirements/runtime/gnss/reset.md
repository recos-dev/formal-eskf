# R-GNSS-RESET: checked GNSS position and velocity reset

**ID:** `R-GNSS-RESET`

**Status:** Draft

**Applies to:** Portable runtime policy; integrations supply timestamps, metadata and deployment parameters.

**Parent:** [SW-GNSS](../../software/gnss-aiding.md).

**Uses:** [E-STATE](../../core/state.md).

## Required behavior

These caller-invoked INS operations replace navigation estimates; they are not
Kalman corrections or `E-RESET`'s post-injection change of error coordinates.
The navigation reset operations and GNSS horizontal-variance policy belong to
`runtime/navigation.hpp`, not to the generic numerical correction. Inputs are already aligned to the prior's
local NED frame, body/IMU origin and time; quality, alignment and reset authority
remain caller responsibilities. No clock, lifecycle or automatic retry is owned.

Behavior reference: PX4 v1.16.2, revision
`54f0455ffcd755534539a7cf33a09a20bf71d29d`:
[GNSS reset dispatch](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp),
[position reset](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp),
[altitude reset](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/position_fusion.cpp),
[GNSS height control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gnss_height_control.cpp),
and [velocity reset](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/velocity_fusion.cpp).

| Operation | Nominal state | Covariance |
|---|---|---|
| `reset_gnss_horizontal_position` | Set `p_N,p_E` to the supplied 2-vector; leave `p_D` and all other state fields unchanged | Clear rows/columns 0 and 1, then set both diagonals to `max(variance_N + variance_E, 0.01^2)` in m^2 |
| `reset_vertical_position` | Set only `p_D` to the supplied scalar; leave velocity, horizontal position, attitude and biases unchanged | Clear off-diagonal entries in row/column 2; set `P(2,2) = max(variance_D, 0.01^2)` in m^2; preserve every other entry |
| `reset_vertical_velocity` | Set only `v_D` to the supplied scalar in m/s; preserve all other nominal fields | Clear off-diagonal entries in rows/columns 2 and 5; retain `P(2,2)`; set `P(5,5) = max(variance_D, 0.01^2)` in m^2/s^2; preserve every other entry |
| `reset_velocity` | Set all three components of `v_n`; leave position, attitude and biases unchanged | Clear off-diagonal entries in rows/columns 0..5. Preserve position diagonals 0..2; set velocity diagonals 3..5 to `max(variance_i, 0.01^2)` in m^2/s^2 |

Vertical position and velocity resets are source-independent numerical operations.
The GNSS prefix is retained only for horizontal position, whose variance-sum
policy is receiver-specific. Source permissions and acknowledgment stay in callers.

The horizontal variance sum follows `resetHorizontalPositionToGnss` calling
`resetLatLonTo`; it is not the per-axis policy of PX4's other position-reset
overload. Velocity reset also decorrelates position, even though position values
and variances do not change. Untouched covariance entries are copied exactly.
The input variances describe diagonal observation noise, not standard
deviations or a full correlated measurement covariance. Noise preprocessing,
including any vertical-velocity scaling, happens before this call.
The Down-only velocity reset follows `resetVerticalVelocityTo`: decorrelate
height as well as vertical velocity, while retaining height value/variance and
both horizontal velocity values/variances. It reports `new v_D - old v_D`.

Vertical position is already converted to the existing local NED origin. For
an altitude-relative Down coordinate with a fixed, common datum,
`p_D = h_origin - h_GNSS`, so `p_D_new - p_D_old` agrees with PX4's
`-(altitude_new - altitude_old)`: climbing gives a negative Down delta.
An ECEF-to-NED adapter must instead supply its projected Down coordinate
consistently; it is not generally identical to altitude difference over distance.
This API does not accept absolute altitude or initialize an origin.
Its covariance rule follows `resetAltitudeTo`, which
does not change velocity. PX4's velocity controller can separately reset `v_D`
when height recovery is required; that policy is not part of this operation.
For GNSS-reference height reset, PX4 uses the prepared GNSS measurement variance,
not the height-bias variance added to its ordinary correction diagnostics. This
API neither estimates nor changes sensor-height biases, terrain or output history.

- All nominal coefficients, P, observation and variances must be finite; otherwise
  return `non_finite_input` before arithmetic. P must be exactly symmetric with
  nonnegative diagonal, and input variances nonnegative; violations return
  `domain_error`. Zero/tiny nonnegative variances receive the explicit floor.
  Symmetric PSD P and a unit prior quaternion remain caller premises; the checks
  are not a PSD test, attitude repair or arbitrary-corruption recovery.
- Compute reset delta as new minus old, in NED meters or m/s. A non-finite delta
  or horizontal variance sum from finite inputs returns `non_finite_result`.
  Check all results before publishing state, covariance or delta.
- On success publish all three outputs. On failure preserve all three, including
  distinct sentinel outputs. Complete state and covariance outputs may each
  independently alias their respective prior. All other input/output storage is
  disjoint. No quaternion normalization, injection, solve or allocation occurs.
- Unlike PX4's internal reset helpers, this checked boundary rejects negative
  or non-finite variances instead of clamping negatives or using NaN to retain
  covariance. The finite valid-input reset behavior above is the aligned scope.
- Return a delta per successful call, not a fusion success, clock update or
  counter increment. The caller must separately acknowledge completed recovery
  and aggregate/publish reset notifications. `R-GNSS-FUSION` supplies
  reset-on-start/timeout orchestration separately. These operations do not perform
  origin changes, output-predictor adjustment, height-reference/bias management,
  automatic height-failure decisions or yaw reset.

Lean expresses navigation covariance reset as `D P Dᵀ` plus nonnegative
replacement diagonal terms. It proves PSD preservation, untouched/decoupled
coefficients, each operation's variance policy and retained nominal attitude/biases.
The real model uses exact decimal `0.01²`; the C++ proof checks each scalar format's
encoded constant and operation order. These are separate evidence layers.

ESBMC verifies the actual input validator for observation sizes 1/2/3, a scalar
lemma connecting finite zero subtraction residuals to exact equality, and all four
reset callers in binary32/binary64 with every independent whole-prior state/P alias.
The height owner additionally has exact `solve_proof::Backend` profiles for its
size-one validator and in-place Down-position/Down-velocity resets; this avoids
assuming equivalence with the original opaque-math backend.
The same-type validation summary is input-pure and allows every returned status;
callers check its exact arguments and invocation. State/delta behavior, complete
covariance decorrelation/floors, overflow errors, failure atomicity and preserved
borrowed inputs are checked. An arbitrary bounded row/column quantifies every
covariance output, not selected sample entries. Comparisons retain values, signed
zero and NaN classification; NaN payload preservation is not claimed. Public invalid
inputs are not assumed away; source/frame correctness, PSD and unit attitude retain their specified
caller ownership. No target-backend proof, floating-point error bound or machine-
checked Lean-to-C++ refinement is claimed. Native tests and mutation controls
complement these proofs. Actual caller permission, acknowledgment and publication
remain separate; `E-RESET` covers a different operation.
