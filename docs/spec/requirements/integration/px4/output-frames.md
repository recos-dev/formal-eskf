# R-PX4-FRAME-OUTPUT: geographic and rotation-variance output frames

**ID:** `R-PX4-FRAME-OUTPUT`

**Status:** Draft

**Applies to:** Single-owner PX4 INS module; active deployment profile and stated dependencies apply.

## Required behavior

The module retains a fixed local origin and its current small-area Cartesian
model. Projection accuracy, model fidelity and estimator accuracy are distinct;
F-05.2 in the [functional backlog](../../../../../FEATURE_TODO.md#f-05--sensor-location-and-frames)
tracks geographic-domain validation and the remaining operating-envelope decisions.

- The core's attitude error is body-local. For the delayed attitude/covariance
  pair compute `P_theta_n = R(q_nb) P_theta_b R(q_nb)^T`, retaining all attitude
  cross terms. Publish its Down diagonal as `heading_var` and the sum of its
  North/East diagonals as `tilt_var`. The heading-validity threshold uses the
  same Down variance. Do not combine current predictor attitude with delayed P.
- These fields follow PX4's navigation-frame rotation-variance proxies, not an
  exact Euler-yaw variance or a current-epoch covariance. At large pitch, the
  Euler-yaw Jacobian has a separate singularity; this change does not certify
  heading near that singularity. Odometry orientation variances retain the
  body-local covariance diagonals, as in PX4's `getRotVarBody()` mapping.
- Normalize computed global longitude and the estimated longitude used for WMM
  lookup to `[-180,180)`. This changes the representation only: retain the local
  origin, state, covariance, reference timestamp and reset history. Raw malformed
  receiver coordinates still follow the existing rejection policy.
- The module's horizontal coordinates use PX4 spherical azimuthal-equidistant
  projection, its Down coordinate uses MSL altitude difference, and receiver
  velocity remains local geographic NED. Replay uses WGS84 ECEF projected to
  fixed-origin NED, with ellipsoidal height and transported velocity. These are
  different approximations; do not claim identical coordinates or reuse one
  path's validation as proof of the other.
- No distance-based invalidation, automatic rebasing, ECEF core, gravity model
  or navigation-frame transport dynamics is introduced. Existing freshness,
  health and fault conditions continue to govern output validity outside the
  assessment grid. Those flags do not certify geographic model fidelity. A
  flight-qualified range/latitude/height envelope requires an accepted error
  budget and validation; it is not inferred from a successful map round trip.

Native evidence must cover tilted and correlated body covariance, a separate
current output attitude, heading permission, unchanged body odometry variances,
and both antimeridian crossing directions through publication and real WMM lookup.
Geographic analysis must call the host projection and compare it with independent
geodesy; record source/library identities, grid and numerical tolerances. Finite
grid checks, sanitizer checks and SIH regression are not global bounds, physical
flight qualification or Lean/ESBMC proof.

Reference: pinned PX4 [rotation variance](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L920),
[odometry mapping](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L1689)
and [longitude arithmetic](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/lib/lat_lon_alt/lat_lon_alt.cpp#L117).
