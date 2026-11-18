# E-PRED-INS: INS translation prediction

**ID:** `E-PRED-INS`

**Status:** Draft

**Applies to:** INS nominal prediction; body-frame specific force, NED position/velocity.

**Parent:** [E-PRED](prediction.md). Its input validation, prior snapshot, status and atomicity rules apply.

## Basis

The prediction sources and interpretation are defined by [E-PRED](prediction.md#sources-and-interpretation).

## Required behavior

For the prior INS state, define the exact-real step by SICE (11), equivalently Solà (260a)–(260e):

~~~text
f_b      = imu.specific_force_b - b_a_old
omega_b  = imu.angular_rate_b - b_g_old
a_n      = R(q_nb_old) * f_b + gravity_n
p_next   = p_old + v_old * dt + 0.5 * a_n * dt^2
v_next   = v_old + a_n * dt
b_a_next = b_a_old
b_g_next = b_g_old
~~~

The integration freezes `a_n` over this step. It is not an exact solution of the fully coupled dynamics under arbitrary changing attitude or acceleration. In particular, the position increment must retain the factor `0.5` and use `v_old`, not `v_next`.

## Acceptance and verification

Establish the equations and limiting behavior under the parent input domain.
Keep exact-real identities separate from coefficient/status checks and numerical
residual bounds; use the parent verification boundary and matching configurations.
