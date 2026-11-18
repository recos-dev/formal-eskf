# E-OBS-POSITION: Navigation-frame position observation

**ID:** `E-OBS-POSITION`

**Status:** Draft

**Applies to:** INS horizontal and vertical position correction.

**Parent:** [E-OBS](observations.md).

## Required behavior

For INS error order [delta_p,delta_v,delta_theta,delta_ba,delta_bg], horizontal
position uses h=[p_N,p_E]^T and H=[I_2,0_(2x13)]; vertical position uses h=p_D
and the 1x15 H with only column 2 equal to one (zero-based indexing). Every
other entry is zero. Both Jacobians are constant and exact; residual is z-h.

Measurements use meters and covariance m^2 at the same NED origin, body/IMU
point and time as the prior. Vertical position is positive Down, not positive-up
altitude or range to terrain. No vertical velocity is required or differenced.
Horizontal uses a full 2x2 V; vertical uses a finite positive variance, not a
standard deviation. The complete state can change through cross-covariance.

Before residual evaluation, reject a non-finite prior position vector or measured
position as `non_finite_input`; the vertical wrapper also checks its scalar
variance at this stage. A non-finite computed residual is `non_finite_result`.
Then use [E-CORRECT](correction.md), forwarding V and bias permissions unchanged.
No gate or datum/lever-arm conversion is performed here.

## Acceptance and verification

Verify exact selector matrices and residuals, absence of unintended axes, full-state correction forwarding, statuses and failure/alias frames. Separate updates do not encode cross-source measurement correlations.
