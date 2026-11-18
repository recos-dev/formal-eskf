# Q-SIGN: Quaternion sign equivalence

**ID:** `Q-SIGN`

**Status:** Draft

**Applies to:** Exact quaternion/SO(3) behavior; ESKF frame usage is defined by E-STATE.

## Basis

Hamilton, scalar-first active rotations on column vectors. Use the [coefficient convention](multiplication.md).

## Required behavior

Unit quaternions form a double cover of `SO(3)`:

~~~text
q ≡ -q
R(q) = R(-q)
rotate(q, v) = rotate(-q, v)
~~~

Coefficient equality and rotation equality are different behaviors:

~~~text
same_coefficients(q1, q2)
same_rotation(q1, q2)
~~~

No operation implicitly forces `q0 >= 0`. Sign selection is performed only when a unique local representation is required, such as the principal `Log_q`.

## Acceptance and verification

Establish the stated exact identities on the declared real/unit-quaternion
domain. Checked C++ behavior, scalar assumptions and numerical residual bounds
remain separate obligations; a finite output does not prove a rotation invariant.
