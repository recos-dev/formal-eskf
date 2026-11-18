# Q-ROT-MATRIX: Quaternion rotation matrix

**ID:** `Q-ROT-MATRIX`

**Status:** Draft

**Applies to:** Exact quaternion/SO(3) behavior; ESKF frame usage is defined by E-STATE.

## Basis

Hamilton, scalar-first active rotations on column vectors. Use the [coefficient convention](multiplication.md).

## Required behavior

For a unit quaternion `q = [q0, q1, q2, q3]`:

~~~text
R(q) = [
  q0²+q1²-q2²-q3²,  2(q1*q2-q0*q3),      2(q1*q3+q0*q2),
  2(q1*q2+q0*q3),   q0²-q1²+q2²-q3²,    2(q2*q3-q0*q1),
  2(q1*q3-q0*q2),   2(q2*q3+q0*q1),      q0²-q1²-q2²+q3²
]
~~~

## Acceptance and verification

Establish the stated exact identities on the declared real/unit-quaternion
domain. Checked C++ behavior, scalar assumptions and numerical residual bounds
remain separate obligations; a finite output does not prove a rotation invariant.
