# Q-ROT-COMPOSE: Rotation composition

**ID:** `Q-ROT-COMPOSE`

**Applies to:** Exact quaternion/SO(3) behavior; ESKF frame usage is defined by E-STATE.

## Basis

Hamilton, scalar-first active rotations on column vectors. Use the [coefficient convention](multiplication.md).

## Required behavior

Composition follows the frame order:

~~~text
q_ac = q_ab ⊗ q_bc
R(q_ac) = R(q_ab) * R(q_bc)
~~~

The rightmost rotation is applied first.

## Acceptance and verification

Establish the stated exact identities on the declared real/unit-quaternion
domain. Checked C++ behavior, scalar assumptions and numerical residual bounds
remain separate obligations; a finite output does not prove a rotation invariant.
