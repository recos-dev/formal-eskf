# F-JR: SO(3) right Jacobian

**ID:** `F-JR`

**Applies to:** Checked SO(3) operation used by exact covariance reset; binary32/binary64.

**Uses:** [C-NUMERICAL-PROFILE](../../common/numerical-profile.md), [C-INPUT-DOMAIN](../../common/input-domain.md), [C-FAILURE](../../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../../common/numerical-acceptance.md).

## Basis

[Sola, Quaternion kinematics for the error-state Kalman filter, v1](https://arxiv.org/pdf/1711.02508v1), (182)-(183).

## Required behavior

For finite rotation vector phi, theta=||phi|| and W=[phi]_x, the exact target is

\[
J_r(\phi)=I-\frac{1-\cos\theta}{\theta^2}W
             +\frac{\theta-\sin\theta}{\theta^3}W^2,\qquad J_r(0)=I.
\]

It maps a perturbation of the rotation vector into a local right perturbation:
Exp(phi+d)=Exp(phi) Exp(J_r(phi)d)+O(||d||^2). It is neither the inverse right
Jacobian nor a rotation matrix; there is no principal-angle restriction.

The implementation shall remove the zero singularity. For t=theta^2 <= 1/256,
use A=1/2-t/24+t^2/720-t^3/40320 and
B=1/6-t/120+t^2/5040-t^3/362880 in
J=(1-t B)I+B phi phi^T-A W. At larger t, use the equivalent half-angle form
J=c I+(1-c)u u^T-a[u]_x, where u=phi/theta,
c=2 sin(theta/2)cos(theta/2)/theta and a=2 sin(theta/2)^2/theta.
These evaluation branches do not change the exact mathematical target.

Reject non-finite input as `non_finite_input`, non-finite squared norm or
result as `non_finite_result`, and an invalid negative computed squared norm as
`domain_error`. Propagate checked scalar failures. Publish a complete finite
matrix only on success; failure preserves the output. This operation has no
mode macro and does not itself transform covariance.

## Acceptance and verification

Prove the exact differential, zero limit and selected Taylor coefficients/remainders; check branch boundaries, ordered IEEE calculation and failure publication. Taylor truncation, rounding and math-library accuracy are distinct obligations; finiteness alone does not discharge an error bound.
