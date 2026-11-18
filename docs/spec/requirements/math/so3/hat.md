# SO3-HAT: Hat operator

**ID:** `SO3-HAT`

**Applies to:** Exact quaternion/SO(3) behavior; ESKF frame usage is defined by E-STATE.

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

Establish the stated exact identities on the declared real/unit-quaternion
domain. Checked C++ behavior, scalar assumptions and numerical residual bounds
remain separate obligations; a finite output does not prove a rotation invariant.
