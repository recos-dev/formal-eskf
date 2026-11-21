# F-GEO-NED: Fixed-origin ECEF and NED transformations

**ID:** `F-GEO-NED`

**Status:** Draft

**Applies to:** Proposed portable geodesy component; fixed-origin Cartesian
positions and changes of vector basis, using IEEE-754 binary64.

**Uses:** [F-GEO-ECEF](geodetic-ecef.md),
[F-LINALG](../linalg/operations.md),
[C-FAILURE](../../common/failure.md),
[C-NUMERICAL-ACCEPTANCE](../../common/numerical-acceptance.md).

## Basis

GeographicLib v2.5 [LocalCartesian](https://github.com/geographiclib/geographiclib/blob/v2.5/include/GeographicLib/LocalCartesian.hpp)
defines a local tangent frame through geocentric coordinates. This contract
uses North/East/Down instead of its East/North/Up order. It defines a rigid
Cartesian transformation, not a spherical map projection or a moving NED frame.

## Required behavior

### Origin and basis

An origin shall contain one geodetic position admitted by F-GEO-ECEF. Let its
ECEF position be `r0_e`, geodetic latitude `phi0` and longitude `lambda0` in radians.
The component shall construct the column-vector rotation from ECEF to origin NED:

~~~text
       [ -sin(phi0) cos(lambda0)  -sin(phi0) sin(lambda0)   cos(phi0) ]
C_ne = [ -sin(lambda0)             cos(lambda0)             0         ]
       [ -cos(phi0) cos(lambda0)  -cos(phi0) sin(lambda0)  -sin(phi0) ]
~~~

NED axes shall be North, East and Down in that order. Down is opposite the
ellipsoid normal at the origin; it is not the ECEF z axis. At a polar origin,
the caller's supplied longitude shall define the horizontal basis. The component
shall retain that convention rather than reconstruct longitude from `r0_e`.

### Position, vector and covariance semantics

For a fixed origin, the position operations shall compute:

~~~text
p_n = C_ne (r_e - r0_e)
r_e = r0_e + transpose(C_ne) p_n
~~~

The origin subtraction shall occur in binary64 before any caller converts the
local result to binary32. Local coordinates are metres in one fixed tangent
frame, not geodesic arc lengths. Equal ellipsoidal heights at different locations
do not generally give equal local Down coordinates. `p_n(2)` shall not be replaced
by an altitude difference.

The exposed basis shall support the following compositions through existing
linalg operations; separate velocity, magnetic-field and covariance wrappers
are not required:

~~~text
v_n = C_ne v_e                       // no position/origin translation
P_n = C_ne P_e transpose(C_ne)

T_0s = C_0e transpose(C_se)           // sample-local NED to origin NED
v_0 = T_0s v_s
P_0 = T_0s P_s transpose(T_0s)
~~~

The caller shall identify the source frame and units and establish covariance
symmetry/PSD as required by the consuming operation. The component shall not
silently treat receiver-local NED velocity as origin NED velocity. Position
uncertainty expressed in latitude/longitude/height is not an ECEF metric
covariance: it needs a separate geodetic Jacobian, outside this V1 API.

These covariance identities assume deterministic frame/origin parameters.
Origin uncertainty, attitude transport, full ESKF covariance rebasing and
position/velocity cross-covariance transport are caller/composition obligations,
not established by a successful 3x3 basis conversion.

### Frame state, failure and ownership

- A default frame shall be uninitialized. `set_origin` shall validate and build
  the ECEF origin and rotation as one candidate, then publish both together.
  Failure shall preserve all previous frame data and initialization state.
- After successful initialization, transformations and basis retrieval shall
  preserve the frame. Copying a frame shall copy its values without shared state.
  Read-only calls may share an initialized frame; concurrent origin mutation is
  a caller synchronization violation, not an internal locking responsibility.
- An operation with a nonfinite explicit position input shall return
  `non_finite_input` before checking initialization. Otherwise, use of an
  uninitialized frame shall return `domain_error`. A nonfinite computed result
  shall return `non_finite_result`. Failure shall preserve all output coefficients.
  Position input and output may alias; publication shall follow full validation.
- Affine position operations shall check finiteness, not perform a second
  geodetic inversion or sensor-quality test. Their numerical acceptance domain
  is specified below; finite success alone does not certify geographic range.
- The frame shall own fixed-size storage and cache its basis at `set_origin`.
  ECEF/NED position operations shall perform bounded arithmetic without repeated
  trigonometry, allocation, exceptions, locks or OS calls.
- `set_origin` may explicitly replace an existing origin, but shall not transform
  estimator state, history, covariance or outputs. A caller holding coordinates
  in the old frame shall not reinterpret them in the new one. Live origin
  rebasing requires a separate coordinated contract and is outside V1.

## Acceptance and verification

- Establish exact-real orthogonality and determinant `+1`, both position inverse
  identities, vector-length preservation and covariance PSD preservation under
  the stated premises. These identities alone do not prove IEEE-754 accuracy.
- Test independent axes: at origin latitude/longitude `(0,0)`, ECEF +z is North,
  +y is East, and +x is Down-negative. Test nontrivial origins, polar longitude
  conventions, antimeridian neighbours, negative heights and correlated 3x3
  covariances. Different sample/origin bases must expose an omitted rotation.
- Compare position conversions with GeographicLib after explicitly permuting
  ENU to `[north, east, -up]`. For origins and points within F-GEO-ECEF's height
  domain, the proposed V1 Euclidean position-error budget is `1e-6 m` per affine
  conversion relative to the exact transform of the represented Cartesian input
  and geodetic origin. This includes origin/basis construction errors. An
  additional conversion of the point under F-GEO-ECEF has its own error, which
  shall be propagated rather than hidden in a production-only round trip.
- Compare `C_ne` with an independent high-precision basis; propose maximum
  coefficient error `1e-14` and `max_abs(C_ne C_ne^T - I) <= 3e-14` for binary64.
  These dimensionless allocations are deliberately above one unit-scale rounding
  error; trig/backend evidence must justify them. Vector and covariance error
  budgets shall account for input magnitude and matrix arithmetic separately.
- Exercise default use, successful origin replacement, rejected replacement,
  invalid input precedence, output/input aliasing and unchanged failure outputs.
  Inspect bounded storage/work. Host timing is not a target WCET certificate.

## Open decisions

The proposed numerical allocations require owner review and implementation
evidence. The [roadmap](../../../../roadmap.md#f-05--sensor-location-and-frames)
separately tracks the operating envelope (F-05.2), origin rebasing (F-05.4) and
long-range navigation model (D-07). This conversion contract does not establish
their acceptance.
