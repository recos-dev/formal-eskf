# Q-INVERSE: Inverse

**ID:** `Q-INVERSE`

**Status:** Draft

**Applies to:** Exact quaternion/SO(3) behavior; ESKF frame usage is defined by E-STATE.

## Basis

Hamilton, scalar-first active rotations on column vectors. Use the [coefficient convention](multiplication.md).

## Required behavior

For a unit quaternion:

~~~text
inverse(q) = conjugate(q) = [q0, -q1, -q2, -q3]
~~~

Its behavior is:

~~~text
q ⊗ inverse(q) = identity
inverse(q) ⊗ q = identity
R(inverse(q)) = transpose(R(q))
~~~

## Acceptance and verification

Establish the stated exact identities on the declared real/unit-quaternion
domain. Checked C++ behavior, scalar assumptions and numerical residual bounds
remain separate obligations; a finite output does not prove a rotation invariant.
