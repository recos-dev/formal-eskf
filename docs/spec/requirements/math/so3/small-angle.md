# SO3-SMALL-ANGLE: Small-angle behavior

**ID:** `SO3-SMALL-ANGLE`

**Applies to:** Exact quaternion/SO(3) behavior; ESKF frame usage is defined by E-STATE.

## Basis

Hamilton, scalar-first active rotations on column vectors. Use the [coefficient convention](../quaternion/multiplication.md).

## Required behavior

As `theta = norm(phi)` approaches zero:

~~~text
Exp_q(phi)
  = [
      1 - theta²/8 + O(theta⁴),
      (1/2 - theta²/48 + O(theta⁴)) * phi
    ]

Exp_R(phi)
  = I + hat(phi) + 1/2 * hat(phi)² + O(theta³)
~~~

Therefore, to first order:

~~~text
Exp_q(phi) ≈ [1, phi/2]
Exp_R(phi) ≈ I + hat(phi)
~~~

The exact behavior at zero is defined by continuity:

~~~text
sin(theta/2)/theta  -> 1/2
sin(theta)/theta    -> 1
(1-cos(theta))/theta² -> 1/2
~~~

For the principal quaternion logarithm, let `r = norm(qv)` after selecting
`q0 >= 0`. When `q0 > 0` and `r` approaches zero, the vector scale is:

~~~text
2 * atan(r/q0) / r
  = 2/q0 * (1 - r²/(3*q0²) + O(r⁴))
~~~

The truncated expressions are approximations, not alternative definitions of
`Exp_q`, `Log_q`, or `Exp_R`.

## Acceptance and verification

Establish the stated exact identities on the declared real/unit-quaternion
domain. Checked C++ behavior, scalar assumptions and numerical residual bounds
remain separate obligations; a finite output does not prove a rotation invariant. Bound truncation and rounding separately before asserting numerical accuracy.
