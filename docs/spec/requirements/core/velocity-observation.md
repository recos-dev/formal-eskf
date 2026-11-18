# E-OBS-VELOCITY: Navigation-frame velocity observation

**ID:** `E-OBS-VELOCITY`

**Status:** Draft

**Applies to:** INS horizontal and three-dimensional velocity correction.

**Parent:** [E-OBS](observations.md).

## Required behavior

For INS error order [delta_p,delta_v,delta_theta,delta_ba,delta_bg], 3D velocity
uses h=v_n and H=[0,I_3,0,0,0]. Horizontal velocity selects only v_N,v_E and
columns 3,4 of the 15-coordinate error (zero-based indexing). All other entries
are zero. Jacobians are constant and exact; residual is z-h.

Measurements are linear velocity in NED, m/s, at the same body/IMU point, frame
and time as the prior; V is respectively 3x3 or 2x2 in m^2/s^2, with correlations
retained. This is not body velocity, scalar speed or integrated displacement.
The full state can change through cross-covariance. Choose horizontal **or** 3D
fusion for a given sample; using both double-counts its horizontal components.

Reject non-finite prior velocity vectors or measurements as `non_finite_input`,
and non-finite residuals as `non_finite_result`. Then use [E-CORRECT](correction.md)
with unchanged V and bias permissions. Conversion, lever arms and timing remain
caller obligations; no direct absolute-position observation is introduced.

## Acceptance and verification

Verify selectors, residuals, full covariance/control forwarding and failure/alias frames for both dimensions. A 3D API does not assert that all sensors provide independent same-time vertical velocity.
