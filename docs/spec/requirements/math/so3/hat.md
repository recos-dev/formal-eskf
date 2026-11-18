# SO3-HAT: Hat operator

**ID:** `SO3-HAT`

**Status:** Draft

**Applies to:** Real three-vectors; no unit-quaternion premise.

## Basis

Hamilton, scalar-first active rotations on column vectors. Use the [coefficient convention](../quaternion/multiplication.md).

## Required behavior

For `a = [ax, ay, az]`:

~~~text
hat(a) = [
   0,  -az,  ay,
   az,   0, -ax,
  -ay,  ax,   0
]
~~~

The defining behavior is:

~~~text
hat(a) * b = cross(a, b)
transpose(hat(a)) = -hat(a)
~~~

## Acceptance and verification

Establish the coefficient, cross-product and skew-symmetry identities for arbitrary
real vectors. The C++ operation is unchecked; its coefficient correspondence and
the caller's finite-input/result obligations are separate from exact algebra.
