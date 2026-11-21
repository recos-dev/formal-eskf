# Geodesy Architecture and Minimal API

**Status:** Draft

## Scope and requirements

[F-GEO-ECEF](../requirements/math/geodesy/geodetic-ecef.md) owns geodetic/ECEF
positions, datum, domain and conversion behavior.
[F-GEO-NED](../requirements/math/geodesy/local-ned.md) owns fixed-origin NED
positions, basis orientation and origin lifecycle. They are shared mathematical
requirements, not new children of the ESKF state model.

This proposal introduces a portable numerical utility used by adapters. It does
not change [E-STATE](../requirements/core/state.md), prediction/correction,
sensor admission, published frames or current replay/PX4 behavior. It defines
proposed operations and data semantics for review, not an implemented interface.

## Responsibilities and interfaces

The planned capability has two layers, tracked under
[F-05](../../roadmap.md#f-05--sensor-location-and-frames):

| Layer | Responsibility |
|---|---|
| Coordinate conversion | Convert geodetic positions through ECEF into local NED at a caller-selected origin. This document covers this layer. |
| Origin rebasing | Runtime/application decides when to select a new local NED frame and coordinates the estimator and dependent data transition. Its trigger and transaction contracts are separate follow-up work. |

~~~text
GNSS / dataset adapter
    |  named geodetic fields with an established height datum
    v
geodetic -> ECEF -> fixed-origin local NED -> existing ESKF position input
                                                    |
global output <- geodetic <- ECEF <- local NED position
~~~

Proposed placement: `include/formal_eskf/geodesy/ecef.hpp` and `local_ned.hpp`,
namespace `formal_eskf::geodesy`. The first owns the geodetic value type and
stateless conversions; the second owns one origin and its cached basis. There
is no new projection backend, virtual interface, policy hierarchy or precision
selector. Reuse `Status`, scalar math and fixed-size linalg; `Linalg` is the one
existing backend parameter and its `value_type` shall be binary64 `double`.

Cartesian positions reuse the existing vector type. Parameter and function names
identify their frames; no new vector arithmetic or frame-tag template system is
introduced. `GeodeticPosition` uses the named fields `latitude_deg`,
`longitude_deg` and `height_ellipsoid_m` defined by F-GEO-ECEF, rather than placing
mixed units in an anonymous vector.

| Operation | Input | Output or state effect | Contract owner |
|---|---|---|---|
| `geodetic_to_ecef` | Geodetic position | ECEF position. | F-GEO-ECEF |
| `ecef_to_geodetic` | ECEF position | Geodetic position. | F-GEO-ECEF |
| `set_origin` | Geodetic origin | Atomically publish the cached ECEF origin, NED basis and initialized state. | F-GEO-NED |
| `initialized` | Local frame | Boolean initialization state; preserve the frame. | F-GEO-NED |
| `ecef_to_ned` | ECEF position and local frame | Position in the frame's fixed-origin NED coordinates. | F-GEO-NED |
| `ned_to_ecef` | Fixed-origin NED position and local frame | ECEF position. | F-GEO-NED |
| `rotation_from_ecef` | Local frame | Copy the cached 3x3 ECEF-to-NED rotation into caller-owned output. | F-GEO-NED |

Except for the boolean initialization query, each operation reports `Status`.
The owning requirements define domains, failure precedence, preservation and
alias behavior; this table does not replace those contracts.

The caller composes two checked operations for geodetic/NED conversion, checking
each status and keeping the ECEF intermediate private. Direct geodetic/NED
wrappers are not needed initially. Vector and 3x3 covariance transformations use
the returned basis and existing linalg. A second frame initialized at the sample
location supplies `C_se` when receiver-local NED must be transported to origin NED.

## Execution and state

Each `LocalNed` owns its cached ECEF origin, rotation and initialized flag. No
estimator state is borrowed or modified. The adapter owns the geographic origin's
identity/lifetime and its relation to pending observations and published outputs.
`set_origin` is an explicit local-frame operation, not an estimator reset API.

For a binary32 estimator, the adapter selects a binary64 geodesy backend, performs
the origin subtraction in binary64, then checks the conversion to its estimator
scalar. Template variation stays at this numerical boundary. Datum lookup,
sensor timing and quality remain adapter/runtime responsibilities.

## Design decisions

- Preserve the existing PX4 projection. Its pinned
  [MapProjection](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/lib/geo/geo.cpp)
  is spherical azimuthal-equidistant, with module Down supplied by MSL height
  difference. F-GEO-NED is an ellipsoidal-origin Cartesian transform. Sharing
  the letters NED does not make their coordinates, derivatives or datums equal.
- The replay [existing ECEF/basis helpers](../../../examples/replay_common.py)
  are migration candidates, not implementations of these new checked contracts.
  Adopting this API in replay or PX4 needs a separate caller/data-flow review;
  this Draft authorizes no replacement or automatic mode selection.
- GeographicLib v2.5 supplies an independent host oracle with explicit ENU/NED
  adaptation. It is not vendored or linked into the flight component by this
  proposal. Selection of a production inverse algorithm remains separate from
  selection of an oracle.
- This conversion proposal adds no geoid database, datum/epoch conversion,
  geodetic covariance Jacobian, live frame rebasing, navigation-frame transport
  or Earth-rotation dynamics. Origin rebasing is the next layer described above.
  The current fixed-gravity ESKF does not become a long-range INS because its
  input coordinates pass through ECEF.

## Verification boundary

The two requirement IDs provide the initial review units. Their acceptance
sections separate exact geometry, checked source behavior and numerical error.
There is no implementation, registered test suite or proof evidence for these
Drafts yet. The current executable review-map denominator remains unchanged;
adoption must register source/test/proof bindings and explicitly retain gaps.

Independent-oracle and round-trip tests alone cannot establish a uniform error
bound. Component acceptance also cannot discharge the module's
[geographic operating-envelope obligation](../requirements/integration/px4/output-frames.md).
PX4 integration, target execution time/stack and full-estimator origin changes
retain separate acceptance scopes.
