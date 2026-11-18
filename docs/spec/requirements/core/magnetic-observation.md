# E-OBS-MAGNETIC: Fixed-reference magnetic observation

**ID:** `E-OBS-MAGNETIC`

**Status:** Draft

**Applies to:** INS/AHRS full-vector magnetic correction with either tilt build mode.

**Parent:** [E-OBS](observations.md).

## Required behavior

For calibrated body field m_b and a fixed trustworthy NED reference m_n,

\[
h=R(q_{nb})^Tm_n,\quad r=m_b-h,\quad
H_{AHRS}=[h]_\times,\quad H_{INS}=[0,0,[h]_\times,0,0].
\]

Use microtesla for both fields and microtesla^2 for the full 3x3 V. Neither field
shall be normalized. The positive hat sign follows the local right perturbation;
no quaternion factor of two is applied to a rotation-vector error. The reference
is not an estimated state; no magnetic-bias state is introduced.

Reject non-finite quaternion/field coefficients as `non_finite_input`; either
exactly zero field vector is `domain_error`. Reject non-finite generated geometry
or residual as `non_finite_result`. Field strength, inclination, disturbances,
weak horizontal heading and source health are runtime concerns, not established
by this nonzero check. A unit prior is a caller premise, not repaired here.

Use the full vector H,r,V in correction with [E-MAG-TILT](magnetic-tilt.md) and
INS [E-BIAS-UPDATE](bias-update.md). Tilt projection constrains the gain, not the
observation Jacobian. Preserve [E-CORRECT](correction.md) failure/alias behavior.
Calibration, reference uncertainty and prior/noise independence remain caller
responsibilities. A single field direction does not observe all attitude DOFs.

## Acceptance and verification

Verify the right-perturbation Jacobian and unit/rotation conventions, checked preparation, same-prior policy forwarding and atomic correction. This is not a scalar compass observation or evidence of WMM accuracy.
