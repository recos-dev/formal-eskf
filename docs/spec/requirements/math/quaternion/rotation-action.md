# Q-ROTATE: Rotation action

**ID:** `Q-ROTATE`

**Applies to:** Exact quaternion/SO(3) behavior; ESKF frame usage is defined by E-STATE.

## Basis

Hamilton, scalar-first active rotations on column vectors. Use the [coefficient convention](multiplication.md).

## Required behavior

~~~text
rotate(q_ab, v_b) = R(q_ab) * v_b
inverse_rotate(q_ab, v_a) = transpose(R(q_ab)) * v_a
~~~

Rotation preserves vector length:

~~~text
norm(rotate(q, v)) = norm(v)
~~~

Applying the inverse rotation recovers the original vector:

~~~text
inverse_rotate(q, rotate(q, v)) = v
~~~

## Acceptance and verification

Establish the stated exact identities on the declared real/unit-quaternion
domain. Checked C++ behavior, scalar assumptions and numerical residual bounds
remain separate obligations; a finite output does not prove a rotation invariant.
