# Geodesy Tests

**Status:** Draft
**Scope:** [F-GEO-ECEF](../../../requirements/math/geodesy/geodetic-ecef.md) and
[F-GEO-NED](../../../requirements/math/geodesy/local-ned.md).
**Level / purpose:** Unit, numerical regression and backend conformance.
**Profiles:** Binary64 only, all available linalg adapters; GCC/Clang, Debug/Release,
strict floating point. Binary32 is a negative compile check. No estimator macros.
**Setup:** GeographicLib v2.5 WGS84 fixtures, analytic axes and independent
long-double normal/tangent geometry. Use NaN/signed-zero sentinels for preservation.
**Regression:** Geodesy, scalar/libm, linalg, compiler or floating-point profile changes.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-F-GEO-ECEF-FORMAT | Reject binary32 geographic coordinates | F-GEO-ECEF, coordinates/domain | Compile each conversion and local frame with double, then float. | Double compiles; float fails with the binary64 requirement diagnostic. | Native: [test_compile_contracts.py](../../../../../tests/common/test_compile_contracts.py) |
| TC-F-GEO-ECEF-AXES | Geodetic axes and longitude convention | F-GEO-ECEF, forward/reverse | Equator, cardinal longitudes, both poles with nonzero longitude, negative height. | Analytic exact ECEF axes; inverse pole longitude zero and antimeridian -180. | Native: [test_geodetic_ecef.cpp](../../../../../tests/math/test_geodetic_ecef.cpp) |
| TC-F-GEO-ECEF-REFERENCE | Independent geodetic conversion fixtures | F-GEO-ECEF, numerical acceptance | Nontrivial hemispheres, near poles, antimeridian neighbours and height extremes. Feed Cartesian fixtures directly to inverse. | GeographicLib forward values; inverse height and independent Cartesian reconstruction within 1 micrometre. Boundary admission tested separately. | Partial: [test_geodetic_ecef.cpp](../../../../../tests/math/test_geodetic_ecef.cpp); sampled accuracy, no uniform libm/error proof |
| TC-F-GEO-ECEF-GRID | Geodetic grid numerical regression | F-GEO-ECEF, numerical acceptance | 11 latitudes × 9 longitudes × 5 heights, including near-pole and near-limit cases. | Independent long-double forward/reconstruction, 1 micrometre Cartesian/height budgets; canonical longitude. | Partial: [test_geodetic_ecef.cpp](../../../../../tests/math/test_geodetic_ecef.cpp); finite grid and host oracle precision |
| TC-F-GEO-ECEF-FORWARD-FAILURE | Geodetic input admission and preservation | F-GEO-ECEF, failure | NaN/Inf, mixed-invalid inputs, adjacent angular/height limits and accepted endpoints. | Ordered status; every output/input field preserved bitwise on failure. | Native: [test_geodetic_ecef.cpp](../../../../../tests/math/test_geodetic_ecef.cpp) |
| TC-F-GEO-ECEF-REVERSE-FAILURE | ECEF input admission and preservation | F-GEO-ECEF, failure | NaN/Inf, signed-zero centre, interior/remote points, exact axis height limits and adjacent outside points. | Ordered status without overflow; inverse height admission; unchanged outputs and input. | Native: [test_geodetic_ecef.cpp](../../../../../tests/math/test_geodetic_ecef.cpp) |
| TC-F-GEO-NED-AXES | Local NED axes and aliased outputs | F-GEO-NED, origin/position | Equatorial signed permutation, both polar origins with supplied longitude, separate and aliased position outputs. | Analytic axis/direction/translation results and inverse; polar basis retains longitude. | Native: [test_local_ned.cpp](../../../../../tests/math/test_local_ned.cpp) |
| TC-F-GEO-NED-GEOMETRY | Local NED independent geometry | F-GEO-NED, numerical acceptance | Nontrivial/near-polar origins, negative heights, represented near-Earth Cartesian points and their local positions. | Long-double normal/tangent basis and affine maps; coefficient error ≤1e-14, orthogonality ≤3e-14, determinant +1, position error ≤1 micrometre. | Partial: [test_local_ned.cpp](../../../../../tests/math/test_local_ned.cpp); sampled scope, not exact identities or uniform bounds |
| TC-F-GEO-NED-TRANSPORT | Tangent-frame curvature and vector transport | F-GEO-NED, vector/covariance | Different origins at equal height; signed-permutation sample-to-origin basis, vector and correlated SPD covariance. | Nonzero Down despite equal ellipsoidal height; hand-calculated vector and covariance rotation, without position translation. | Native: [test_local_ned.cpp](../../../../../tests/math/test_local_ned.cpp) |
| TC-F-GEO-NED-LIFECYCLE | Local origin lifecycle and value ownership | F-GEO-NED, frame state | Default use, invalid first origin, initialization, invalid replacements, successful replacement after copying. | Status precedence, preserved frame/output on rejection; owned copy retains original origin and basis. | Native: [test_local_ned.cpp](../../../../../tests/math/test_local_ned.cpp) |
| TC-F-GEO-NED-FAILURE | Local transform invalid and overflowing inputs | F-GEO-NED, failure | NaN/±Inf in each axis; finite max-magnitude vectors overflowing forward/reverse rotation; aliased failure. | Correct input/result status, preserved outputs and inputs. | Native: [test_local_ned.cpp](../../../../../tests/math/test_local_ned.cpp) |

## Source allocation

| Source | API group → cases above |
|---|---|
| `geodesy/ecef.hpp` | Value type, scalar restriction, forward/inverse conversions → F-GEO-ECEF cases. |
| `geodesy/local_ned.hpp` | Origin transaction, initialization query, cached basis, affine transforms/copy → F-GEO-NED cases and format rejection. |
| `tests/common/test_compile_contracts.py` | Public-header independence additionally checks geodesy has no core/runtime/platform dependency. |

## Oracle and remaining gaps

The ten Cartesian fixtures in `test_geodetic_ecef.cpp` come from GeographicLib
v2.5 `CartConvert -p 9`, with WGS84 defaults. Each row's geodetic fields reproduce
the input; output precision is 1 nm. No GeographicLib runtime/build dependency
is introduced. Long-double geometry provides a second sampled reference; its
precision/libm properties belong to the host profile, not target qualification.

Inverse and forward paths are checked against independent data, not just each
other. The geometry test evaluates each affine direction against the represented
input separately, so input conversion error is not hidden in a round trip.
The requirement's boundary-admission caveat remains explicit.

Exact-real identities, a uniform floating-point/libm error bound, source proofs
and target WCET/stack remain open. Native tests do not inject a failing math
backend into geodetic/origin construction; nonfinite-result branches there need
separate source/seam evidence. Local-position arithmetic overflow is exercised
with the actual backend. Runtime origin rebasing and PX4/replay adoption are
outside this component suite. Passing rows do not approve the Draft contracts.
