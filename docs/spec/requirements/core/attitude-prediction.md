# E-PRED-ATTITUDE: Attitude prediction

**ID:** `E-PRED-ATTITUDE`

**Applies to:** INS and AHRS; the selected ESKF_QUAT_APPROX configuration.

**Parent:** [E-PRED](prediction.md). Its input validation, prior snapshot, status and atomicity rules apply.

## Basis

The prediction sources and interpretation are defined by [E-PRED](prediction.md#sources-and-interpretation).

## Required behavior

Let `q_old = q_nb_old` and let `omega_b` be the INS bias-corrected rate `imu.angular_rate_b - b_g_old`, or the supplied AHRS rate, held constant over the step. Use scalar-first coefficients and Hamilton multiplication:

~~~text
theta = omega_b * dt

ESKF_QUAT_APPROX=0 (default):
    q_next = q_old ⊗ Exp_q(theta)

ESKF_QUAT_APPROX=1:
    c      = q_old ⊗ [1, theta_x/2, theta_y/2, theta_z/2]
    q_next = c / norm(c)
~~~

The default exact-real map solves `q_dot = 0.5 * q ⊗ [0, omega_b]` for a constant body rate. The approximation is normalized Euler; it must have the same first-order derivative at `dt = 0`, not be claimed equal to Exp for a finite nonzero step. Both preserve unit norm in the exact model for unit `q_old`.

The C++ Exp path includes its existing small-angle branch and checked normalization of the increment and product. The Euler path normalizes its candidate without first normalizing the increment. Their finite-precision correspondence must be checked separately; the model equations above are not assertions of bitwise equality to real arithmetic.

## Acceptance and verification

Establish the equations and limiting behavior under the parent input domain.
Keep exact-real identities separate from coefficient/status checks and numerical
residual bounds; use the parent verification boundary and matching configurations.
