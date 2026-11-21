# F-GEO-ECEF: WGS84 geodetic and ECEF position conversion

**ID:** `F-GEO-ECEF`

**Status:** Draft

**Applies to:** Proposed portable geodesy component, IEEE-754 binary64 only.
This is not an ECEF estimator-state model or a flight operating envelope.

**Uses:** [C-FAILURE](../../common/failure.md),
[C-NUMERICAL-PROFILE](../../common/numerical-profile.md),
[C-NUMERICAL-ACCEPTANCE](../../common/numerical-acceptance.md),
[C-CPP-SUBSET](../../common/cpp-subset.md).

## Basis

Use the WGS84 ellipsoid and geodetic latitude, following GeographicLib v2.5
[constants](https://github.com/geographiclib/geographiclib/blob/v2.5/include/GeographicLib/Constants.hpp)
and [Geocentric semantics](https://github.com/geographiclib/geographiclib/blob/v2.5/include/GeographicLib/Geocentric.hpp).
The restricted domain, checked failure behavior and API are project decisions;
GeographicLib is a host verification reference, not a required runtime dependency.

## Required behavior

### Coordinates and domain

| Quantity | Meaning and proposed V1 domain |
|---|---|
| `latitude_deg` | Geodetic latitude in degrees, inclusive `[-90, 90]`; not geocentric latitude. |
| `longitude_deg` | Longitude in degrees, inclusive `[-180, 180]` on input; canonical `[-180, 180)` on reverse output. |
| `height_ellipsoid_m` | Metres along the ellipsoid normal, inclusive `[-12000, 100000]`. |
| ECEF `[x, y, z]` | Metres; origin at Earth's centre, x through latitude/longitude `(0, 0)`, y through `(0, 90)`, z through the North Pole. |

The component shall use `a = 6378137 m`, `f = 1/298.257223563`,
`e2 = f(2-f)` and `b = a(1-f)`. These are ellipsoid definitions, not tunable
estimator parameters. Coordinate storage and calculations shall remain binary64;
a binary32 instantiation shall be rejected at compile time.

The caller shall supply ellipsoidal height. MSL/orthometric height `H` is a
different datum: converting it requires an externally established geoid
undulation `N_g`, with `h = H + N_g`. This component shall not infer `N_g`, use
a fixed offset silently, or accept a datum selector that treats the values as
interchangeable. Geoid lookup and datum/epoch transformations are outside V1.

The proposed height limits bound the numerical implementation domain while
covering negative terrain elevations and airborne use. They do not certify an
aircraft, sensor, atmospheric model or PX4 deployment up to those limits.

### Forward and reverse maps

For angles `phi` and `lambda` converted to radians, the forward map shall be:

~~~text
N(phi) = a / sqrt(1 - e2 sin(phi)^2)
x = (N(phi) + h) cos(phi) cos(lambda)
y = (N(phi) + h) cos(phi) sin(lambda)
z = (N(phi)(1-e2) + h) sin(phi)
~~~

At an exactly represented input latitude of `+90` or `-90` degrees, the forward
map shall produce zero x/y and `z = sign(latitude_deg) * (b+h)`. Both input
representations of the antimeridian denote the same physical position.

The reverse map shall return the geodetic latitude and normal height corresponding
to the ECEF position in the supported near-surface domain. Longitude shall use
the `atan2(y, x)` quadrant and the canonical interval above. For a nonzero point
on the z axis, latitude shall be the corresponding pole, longitude shall be zero,
and height shall be `abs(z)-b`. No meaningful longitude is recovered at a pole.

The Earth centre shall be rejected. Points outside the proposed height domain
shall not be converted into an in-domain substitute. The checked height limit
applies to the supplied forward height and computed reverse height, respectively;
a forward/reverse round trip within the inverse error budget of a limit is not
guaranteed to retain the same admission decision. No boundary clamping is implied.

### Failure and execution

The following precedence shall apply. Later rows apply only if earlier checks
have not failed; reverse height validation follows candidate computation.

| Condition | Required response |
|---|---|
| Any supplied coordinate is NaN or infinity | `non_finite_input`. |
| Forward latitude/longitude outside the stated ranges, or reverse input at Earth's centre | `domain_error`. |
| Forward height outside the stated range, or a reverse point established to be outside that domain | `out_of_range`. |
| A required intermediate or candidate is nonfinite | `non_finite_result`. |
| Finite reverse candidate height outside the stated range | `out_of_range`. |
| Otherwise | `success`; publish the complete candidate. |

Failure shall preserve every output coefficient/field, including pre-existing
nonfinite values and signed zeros. Inputs shall remain unchanged. There shall be
no allocation, exception, platform call, shared mutable state or unbounded
convergence loop. The implementation shall document a fixed worst-case work bound
and establish its numerical adequacy over this domain before acceptance.

## Acceptance and verification

- Check independent axis/equator/pole fixtures, negative heights, longitude
  quadrants, both antimeridian representations and limits plus adjacent values.
  Check mixed-invalid precedence and sentinel preservation on every failure.
- Compare forward/reverse results with the pinned independent reference, not
  only with each other. Test inverse height admission at analytically known
  axis points. A shared error in two project functions must not pass a round trip.
- Proposed V1 binary64 acceptance budget: forward Cartesian error at most
  `1e-6 m` in Euclidean norm; reverse height error at most `1e-6 m`, and at most
  `1e-6 m` Cartesian error when the returned geodetic result is reconstructed
  independently. Check longitude range/quadrants separately; do not require
  equality of polar longitude or use a uniform longitude tolerance near poles.
- This micrometre budget is a proposed numerical allocation, not GNSS accuracy.
  The reference documents nanometre-scale errors in this domain; implementation,
  libm, compiler and oracle uncertainties still need explicit evidence. A finite
  grid establishes only its sampled scope, not a uniform floating-point bound.
- Separate exact-model properties, source/status/atomicity checks and numerical
  comparison evidence. No proof or implementation is currently bound to this Draft.

## Open decisions

Owner review is required for the proposed height limits and numerical budget.
Inverse algorithm selection and its bounded-work/error justification remain
implementation work; acceptance cannot rely solely on matching GeographicLib.
