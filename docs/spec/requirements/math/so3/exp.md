# SO3-EXP: Exponential map

**ID:** `SO3-EXP`

**Applies to:** Exact quaternion/SO(3) behavior; ESKF frame usage is defined by E-STATE.

## Basis

Hamilton, scalar-first active rotations on column vectors. Use the [coefficient convention](../quaternion/multiplication.md).

## Required behavior

For a rotation vector `phi in R³`, let `theta = norm(phi)`.

~~~text
Exp_q(phi) = [
  cos(theta/2),
  (sin(theta/2)/theta) * phi
]
~~~

At zero:

~~~text
Exp_q(0) = identity
~~~

The equivalent rotation matrix is:

~~~text
Exp_R(phi) = I
           + (sin(theta)/theta) * hat(phi)
           + ((1-cos(theta))/theta²) * hat(phi)²
~~~

## Acceptance and verification

Establish the stated exact identities on the declared real/unit-quaternion
domain. Checked C++ behavior, scalar assumptions and numerical residual bounds
remain separate obligations; a finite output does not prove a rotation invariant.
