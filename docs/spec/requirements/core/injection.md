# E-INJECT: Nominal error injection

**ID:** `E-INJECT`

**Applies to:** Shared INS/AHRS core; binary32/binary64 under the selected build configuration.

## Basis

[Sola, Quaternion kinematics for the error-state Kalman filter, v1](https://arxiv.org/pdf/1711.02508v1), (283a)-(283e); gravity is a fixed parameter, not an injected state.

## Required behavior

For the local right-multiplicative convention of [E-STATE](state.md), inject the
supplied estimated error using

\[
p^+=p+\delta p,\quad v^+=v+\delta v,\quad
q^+=\operatorname{normalize}(q\otimes\operatorname{Exp}(\delta\theta_b)),\quad
b_a^+=b_a+\delta b_a,\quad b_g^+=b_g+\delta b_g.
\]

AHRS injects attitude only. Exp is always used; `ESKF_QUAT_APPROX` affects
prediction, not injection. The [Exp](../math/so3/exp.md) and
[normalized composition](../math/quaternion/composition.md) contracts own the
nonlinear calculation and normalization threshold. The prior is unit under the
model premise; injection checks finiteness and normalization feasibility, not
prediction's residual tolerance.

Reject non-finite consumed state/error/threshold values as `non_finite_input`,
and a threshold outside (0,1] as `domain_error`. Propagate Exp/normalization
failure; non-finite internally generated coefficients or additive outputs are
`non_finite_result`. Publish the full nominal candidate only on success; preserve
the output on every failure, including when it aliases the complete prior state.
Standalone injection shall not alter covariance or zero the caller's error.

## Acceptance and verification

Verify the exact additive/right-multiplicative relation, both configurations, nonlinear dependencies and full failure/alias frame. Normalization success is not a certified floating-point unit-norm error bound.
