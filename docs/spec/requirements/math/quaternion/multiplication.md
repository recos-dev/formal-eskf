# Q-MUL: Hamilton multiplication

**ID:** `Q-MUL`

**Applies to:** Exact quaternion/SO(3) behavior; ESKF frame usage is defined by E-STATE.

## Basis

Hamilton, scalar-first active rotations on column vectors. The coefficient and multiplication convention is defined here.

## Required behavior

A quaternion uses scalar-first ordering:

~~~text
q = [q0, q1, q2, q3] = [q0, qv]
~~~

Unit quaternions satisfy:

~~~text
q0² + q1² + q2² + q3² = 1
~~~

Quaternion multiplication uses the Hamilton convention. Rotations are active rotations acting on column vectors.

For `q_a = [a0, a1, a2, a3]` and `q_b = [b0, b1, b2, b3]`:

~~~text
q_a ⊗ q_b = [
  a0*b0 - a1*b1 - a2*b2 - a3*b3,
  a0*b1 + a1*b0 + a2*b3 - a3*b2,
  a0*b2 - a1*b3 + a2*b0 + a3*b1,
  a0*b3 + a1*b2 - a2*b1 + a3*b0
]
~~~

## Acceptance and verification

Establish the stated exact identities on the declared real/unit-quaternion
domain. Checked C++ behavior, scalar assumptions and numerical residual bounds
remain separate obligations; a finite output does not prove a rotation invariant.

## Scope

These requirements cover rotations, not a general SE(3) abstraction. ESKF position
and velocity remain additive Euclidean states. [E-STATE](../../core/state.md) owns
the body-to-NED convention, units and perturbation side. API/domain failures and
numerical tolerances follow the shared numerical contracts.
