# Q-MUL: Hamilton multiplication

**ID:** `Q-MUL`

**Status:** Draft

**Applies to:** Hamilton products of real quaternion coefficients; rotation interpretation additionally requires unit inputs.

## Basis

Hamilton, scalar-first active rotations on column vectors, following
[Solà, Quaternion kinematics for the error-state Kalman filter, v1](https://arxiv.org/pdf/1711.02508v1),
equation (12). The coefficient and multiplication convention is defined here.

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

Establish each Hamilton coefficient for arbitrary real inputs and unit closure
under unit-input premises. The raw C++ operator is unchecked and does not
normalize. Its ordered coefficient computations and numerical residual bounds
are separate obligations; checked publication belongs to
[Q-ROT-COMPOSE](composition.md).

## Scope

These requirements cover rotations, not a general SE(3) abstraction. ESKF position
and velocity remain additive Euclidean states. [E-STATE](../../core/state.md) owns
the body-to-NED convention, units and perturbation side. API/domain failures and
numerical tolerances follow the shared numerical contracts.
