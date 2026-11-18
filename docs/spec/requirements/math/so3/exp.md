# SO3-EXP: Exponential map

**ID:** `SO3-EXP`

**Status:** Draft

**Applies to:** Real rotation vectors in radians; checked C++ evaluation under the selected scalar/backend profile.

**Uses:** [Q-NORMALIZE](../quaternion/normalization.md), [SO3-SMALL-ANGLE](small-angle.md).

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

The quotient formulas apply for theta > 0; their continuous extension at zero
gives identity. No principal-angle restriction is imposed on the input.

The checked C++ map shall reject non-finite input/threshold values, then a
nonpositive normalization threshold. Check the computed squared angle before
selecting the small-angle or trigonometric branch; propagate checked scalar
failures and pass the candidate through Q-NORMALIZE. Publish only on success
and preserve output on failure. Its Taylor approximation and floating-point
normalization are separate from the exact map; neither the type name nor success
proves exact unit norm.

## Acceptance and verification

Establish the exact map, unit norm and zero limit for real rotation vectors.
Check both C++ branches, scalar/normalization status use and failure preservation;
truncation and rounding bounds remain separate numerical obligations.
