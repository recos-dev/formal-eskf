# PX4 fixed-frame assessment — 2026-11-17

F-05.2 establishes the geographic differences in the current implementation and
corrects two output-frame defects. It does **not** establish a flight-qualified
radius. Requirements are in
[R-PX4-FRAME-OUTPUT](spec/verification-requirements.md#r-px4-frame-output-geographic-and-rotation-variance-output-frames).
Parent: formal-eskf `9618d18`; reference: PX4 v1.16.2
`54f0455ffcd755534539a7cf33a09a20bf71d29d`. All results below apply to this dirty
source revision identified by the evidence hashes, not every later checkout.

## Actual models

| Quantity | PX4 module | PX4 log replay |
| --- | --- | --- |
| Horizontal position | PX4 spherical azimuthal-equidistant projection, radius 6,371,000 m | WGS84 ECEF difference rotated into fixed-origin NED |
| Height | MSL altitude difference; a separate origin ellipsoidal altitude supplies global ellipsoidal output | Ellipsoidal heights in the ECEF conversion |
| GNSS velocity | Receiver's local geographic NED, without geographic transport to the origin | Rotated from receiver-local NED into origin NED |
| Main model | Fixed Cartesian navigation frame, constant gravity | Fixed Cartesian navigation frame, constant gravity |

These paths do not have interchangeable coordinates. The module's two global
altitudes retain the origin's geoid separation; geoid variation is not modeled.
Equal ellipsoidal heights also have a nonzero Down displacement in an origin
tangent plane. Neither that geometric sag nor a different projection coordinate
is by itself an estimator height/position error.

Pinned EKF2 also uses [MapProjection](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/lib/geo/geo.cpp)
for local coordinates, but its [nominal prediction](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf.cpp#L233)
maintains geographic position and includes Earth rotation, Coriolis and transport
terms. Sharing its projection does not make our fixed Cartesian core equivalent
to EKF2 over long distances or long unaided durations.

## Independent geographic comparison

Run [analyze_frame.py](../integrations/px4/analyze_frame.py) after installing the
integration requirements and building the pinned PX4 SITL target. It compiles a
probe against the actual PX4 `libgeo.a`; it does not replace MapProjection with a
second implementation of our own.

- GeographicLib 2.1 generates WGS84 destinations and measures inverse geodesic
  round trips. PROJ 9.5.1 through pyproj 3.7.1 supplies independent spherical
  [AEQD](https://proj.org/en/stable/operations/projections/aeqd.html) and WGS84
  [topocentric](https://proj.org/en/stable/operations/conversions/topocentric.html)
  conversions. NumPy 2.2.6 supplies matrix calculations.
- Origins: latitudes −80, −60, −45, −25, 0, 25, 45, 60, 80, 89.9 degrees;
  longitudes 0, 120, 179.999 degrees. Distances: 0, 100, 500 m, 1, 2, 5, 10,
  20, 50, 100 km. Eight bearings per nonzero distance: **2,190 points**.
- Replay comparisons use origin ellipsoidal height 0 m and destination heights
  −100, 0, 500, 3,000 m: **8,760 comparisons**. This samples conversion geometry,
  not atmospheric height, geoid errors or a deployment altitude envelope.
- Projection acceptance allows half the Euclidean float-ULP quantization plus
  2 micrometres. Replay acceptance is 0.1 micrometre. All cases passed:
  projection/oracle maximum **4.618 mm**, round-trip maximum **4.610 mm**,
  replay/oracle maximum **0.938 nm**. The tight replay agreement checks coordinate
  implementation, not physical sensor accuracy.

The following maxima are over the sampled origins with absolute latitude at
most 60 degrees, at equal zero ellipsoidal heights. They are sampled model
differences, not continuous-domain bounds or observed flight errors.

| Radius | Sphere vs WGS84 tangent horizontal coordinates | Tangent Down sag | Full local-NED/origin-NED rotation | Unrotated velocity difference at 20 m/s | Gravity-direction difference |
| --- | ---: | ---: | ---: | ---: | ---: |
| 100 m | 0.561 m | 0.00079 m | 0.00179° | 0.000626 m/s | 0.000155 m/s² |
| 1 km | 5.613 m | 0.0789 m | 0.01792° | 0.00626 m/s | 0.00155 m/s² |
| 5 km | 28.065 m | 1.973 m | 0.08961° | 0.0313 m/s | 0.00774 m/s² |
| 10 km | 56.134 m | 7.892 m | 0.17921° | 0.0626 m/s | 0.01548 m/s² |
| 100 km | 565.363 m | 789.195 m | 1.79172° | 0.6254 m/s | 0.15479 m/s² |

The velocity column is the maximum over vector direction at norm 20 m/s of
`(T - I)v`, where `T` maps the receiver's geographic NED into origin NED.
Gravity compares the Down directions with magnitude 9.80665 m/s². These do not
include gravity-magnitude variation or unmodeled inertial dynamics.

There is also a position/velocity metric mismatch. A central ±1 m geographic
difference estimates the spherical projection Jacobian `J`. Even at the origin,
`20 ||J - I||₂` reaches **0.1123 m/s** in this grid: the sphere/ellipsoid scale
difference does not disappear merely by staying close to home. At latitude 25°,
1 km north, the corresponding value is **0.0762 m/s**. This is a model-consistency
indicator, not the velocity residual of a flight. Differentiation uses double
PROJ coordinates to avoid differentiating PX4 float quantization.

Latitude matters independently of distance. At 89.9° latitude, 1 km east rotates
the complete geographic NED frame by **5.116°**; at 20 m/s the maximum untransported
velocity difference is **1.785 m/s**. A distance-only policy cannot qualify polar
operation. Antipodal/global navigation is outside this analysis.

## Corrected output behavior

1. **Rotation variance:** the core's right-local error covariance is in body
   coordinates. Heading and tilt now use `R P_theta_b Rᵀ` with the matching
   delayed attitude. Heading validity uses the same Down diagonal. Odometry
   retains body-coordinate variance. This matches the frame semantics of PX4's
   [rotation-variance accessors](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L920).
   It is still a rotation-variance proxy, not exact Euler-yaw covariance. For
   example, `P_b = diag(0.01, 0.04, 0.001)` at roll 90°, pitch 0° gives Down
   variance 0.04 rather than the former body-Z value 0.001. At pitch 60° the
   same rolled attitude has Down variance 0.0175 but Euler-yaw variance 0.16.
2. **Longitude:** MapProjection reproject does not normalize longitude; 263 grid
   cases exceeded the conventional interval. Computed global longitude and the
   estimated WMM-query longitude now wrap to `[-180,180)`, consistent with PX4
   geographic arithmetic. No public-origin reset, state change or relaxation of
   raw GNSS validation is introduced.

Native fixtures cover three tilted/correlated covariance cases, a current output
attitude distinct from delayed state, the heading threshold, unchanged odometry
body variances, and both dateline directions through actual WMM and publication.
Core equations and covariance propagation are unchanged. No new dynamic storage,
OS operation or unbounded scan is added to the module tick; target execution-time
qualification remains F-07.2.

## Verification and retained attempts

Evidence root: `output/px4-frame-envelope-20261117/`. Geographic `manifest.json`
records source, actual geo archive, probe, library versions and result hashes.
The final `verification-manifest.json` binds module/package/binary/test evidence.
These are local artifacts; missing artifacts must not be treated as acceptance.

| Check | Result / scope |
| --- | --- |
| PX4 module build and native production fixture | 112 scenarios, zero failures |
| Module/fixture ASan and UBSan | 112 scenarios, zero failures; PX4 archives uninstrumented, leak detection disabled |
| Independent geographic comparisons | 2,190 projection and 8,760 ECEF/NED cases passed |
| Package tests and installed-source check | 8 tests passed; 62 installed files and host patch matched |
| Formatting, Python syntax and traceability references | Passed; 1,305 existing/new source references resolved |
| SIH regression, flight 03 | 20/20 ground-fault and 18/18 independent ULog checks passed; not a geographic-range test |

The first launch failed before simulation because the sandbox prohibited a UDP
socket (`flight-run.log`). Flight 02 completed the commanded flight and landing,
but passed only 19/20 ground-fault checks. At GNSS height recovery, local output
timestamp 176868000 had reset counters 5; the status snapshot timestamp 176800000
still had counters 4 and gravity active. Internal snapshot 176876000 already
reported GNSS height active and counters 5. This is evidence of cross-topic
sampling skew; the failed result remains failed. Its independent analyzer refused
the failed live-run result. F-07.1 retains the test-harness synchronization work.

Flight 03 repeated the unchanged configuration and assertions. All ground checks
and the independent analyzer passed: position RMSE **0.236 m**, velocity RMSE
**0.339 m/s**, attitude RMSE **1.643°** over 73.872 s airborne; no airborne
failsafe, numerical fault flags or ULog dropout. These are simulation results
with zero installation offsets, not nonzero-installation or geographic-range
qualification. A passing repeat does not erase the earlier sampling failure.

The initial package-test invocation omitted required `--px4`; the corrected
invocation passed all eight tests. No acceptance assertion, production threshold
or noise setting was relaxed. No Lean, ESBMC, target WCET or physical-flight
qualification is claimed by this task.

## Operating-envelope recommendation

Keep the current model explicitly scoped to local experimental navigation.
For the next validation campaign, use a declared latitude, maximum distance,
speed, height datum and unaided duration, and accept budgets for position/velocity
consistency and attitude/vertical error before selecting a limit. The table lets
that decision use measured scales rather than an arbitrary radius.

The grid does not justify calling even 1 km universally safe. Error budgets may
require reconciling projection derivatives and receiver velocities, spatial
gravity/geoid effects, or D-07's geographic-state design. Do not silently retrofit
these into the shared ESKF equations. A smaller radius alone cannot eliminate the
sphere/ellipsoid scale mismatch; a radius alone also cannot bound time-dependent
inertial drift.

Current behavior beyond the sampled domain is unchanged: existing health,
freshness and fault rules govern validity, with no range guard or automatic
rebasing. Those flags do not certify model fidelity. **F-05.2 remains open for
the accepted operating budget and out-of-envelope policy.** F-06.1 can continue
auditing diagnostics without implying that this flight-envelope decision is done.
