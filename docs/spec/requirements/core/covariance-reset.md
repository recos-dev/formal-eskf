# E-RESET: Post-injection covariance coordinates

**ID:** `E-RESET`

**Status:** Draft

**Applies to:** Shared INS/AHRS core; binary32/binary64 under the selected build configuration.

**Uses:** [C-NUMERICAL-PROFILE](../common/numerical-profile.md), [C-INPUT-DOMAIN](../common/input-domain.md), [C-FAILURE](../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../common/numerical-acceptance.md).

## Basis

[Sola, Quaternion kinematics for the error-state Kalman filter, v1](https://arxiv.org/pdf/1711.02508v1), (183), reset section 5.4 and first-order (294).

## Required behavior

Use the estimated delta_theta **before zeroing it**. Define G as identity on all
Euclidean error coordinates and J on the attitude block (offset 0 for AHRS,
offset 6 for INS):

\[
P^+=GPG^T,\qquad
J=\begin{cases}J_r(\delta\theta),&\texttt{ESKF_RESET_APPROX=0},\\
I-\tfrac12[\delta\theta]_\times,&\texttt{ESKF_RESET_APPROX=1}.
\end{cases}
\]

Default to the exact [right Jacobian](../math/so3/right-jacobian.md). Exact here
means the differential of the reset map at the injected estimate; covariance
transport remains a linearized uncertainty model. Transform the **entire** P,
including attitude cross-covariances. The two approximation macros are independent.

The caller establishes symmetric PSD P. Check P and the consumed attitude error
for finiteness; unused Euclidean error coordinates shall not affect reset.
Propagate right-Jacobian failure, or reject non-finite approximate-Jacobian or
sandwich arithmetic as `non_finite_result`. Check the computed covariance for finiteness, then restore symmetry by
averaging off-diagonal pairs as 0.5*a + 0.5*b before publishing. Failure preserves output; complete P
input/output aliasing is supported. Cleanup is not a floating-point PSD proof.

`reset_covariance` only changes covariance. [E-STEP-INJECT-RESET](injection-reset-step.md)
owns publication of zero in every estimated error-mean coordinate after successful
injection and covariance reset. This zero is the filter's new error estimate,
not a claim of zero physical estimation error or zero uncertainty.

## Acceptance and verification

Verify zero-increment identity, selected Jacobian, full covariance/cross-covariance transformation and conditional real PSD. Check success/failure publication and aliases for both reset modes and scalar formats.
