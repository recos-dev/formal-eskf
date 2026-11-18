# Q-IDENTITY: Identity

**ID:** `Q-IDENTITY`

**Status:** Draft

**Applies to:** Exact quaternion/SO(3) behavior; ESKF frame usage is defined by E-STATE.

## Basis

Hamilton, scalar-first active rotations on column vectors. Use the [coefficient convention](multiplication.md).

## Required behavior

~~~text
identity() = [1, 0, 0, 0]
~~~

For every unit quaternion `q`:

~~~text
identity ⊗ q = q
q ⊗ identity = q
~~~

## Acceptance and verification

Establish the stated exact identities on the declared real/unit-quaternion
domain. Checked C++ behavior, scalar assumptions and numerical residual bounds
remain separate obligations; a finite output does not prove a rotation invariant.
