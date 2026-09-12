## Summary

Define the mathematical behavior and conventions of the `SO(3)` and unit-quaternion operations used by the ESKF.

## Scope

This specification covers rotations only. Position and velocity remain additive Euclidean states, so a general `SE(3)` abstraction is not required.

The definitions below describe exact mathematical behavior. API design, invalid-input handling, floating-point tolerances, implementation details, and proof obligations are specified separately.

## Conventions

### Frames

The navigation frame is NED:

~~~text
n.x = North
n.y = East
n.z = Down
~~~

The IMU measurement frame is the body frame `b`.

`R_nb` and `q_nb` map body-frame coordinates into navigation-frame coordinates:

~~~text
v_n = R_nb * v_b
v_n = rotate(q_nb, v_b)
~~~

### Quaternion representation

A quaternion uses scalar-first ordering:

~~~text
q = [q0, q1, q2, q3] = [q0, qv]
~~~

Unit quaternions satisfy:

~~~text
q0² + q1² + q2² + q3² = 1
~~~

Quaternion multiplication uses the Hamilton convention. Rotations are active rotations acting on column vectors.

For a unit quaternion `q = [q0, q1, q2, q3]`:

~~~text
R(q) = [
  q0²+q1²-q2²-q3²,  2(q1*q2-q0*q3),      2(q1*q3+q0*q2),
  2(q1*q2+q0*q3),   q0²-q1²+q2²-q3²,    2(q2*q3-q0*q1),
  2(q1*q3-q0*q2),   2(q2*q3+q0*q1),      q0²-q1²-q2²+q3²
]
~~~

## Required behavior

### Identity

~~~text
identity() = [1, 0, 0, 0]
~~~

For every unit quaternion `q`:

~~~text
identity ⊗ q = q
q ⊗ identity = q
~~~

### Normalization

For a nonzero quaternion `q`:

~~~text
normalize(q) = q / norm(q)
norm(normalize(q)) = 1
~~~

Normalization does not select a preferred sign. Both `normalize(q)` and `-normalize(q)` are valid representations of the same rotation.

### Multiplication and composition

For `q_a = [a0, a1, a2, a3]` and `q_b = [b0, b1, b2, b3]`:

~~~text
q_a ⊗ q_b = [
  a0*b0 - a1*b1 - a2*b2 - a3*b3,
  a0*b1 + a1*b0 + a2*b3 - a3*b2,
  a0*b2 - a1*b3 + a2*b0 + a3*b1,
  a0*b3 + a1*b2 - a2*b1 + a3*b0
]
~~~

Composition follows the frame order:

~~~text
q_ac = q_ab ⊗ q_bc
R(q_ac) = R(q_ab) * R(q_bc)
~~~

The rightmost rotation is applied first.

### Inverse

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

### Rotation action

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

### Hat operator

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

### Exponential map

For a rotation vector `phi in R³`, let `theta = norm(phi)`.

~~~text
Exp_q(phi) = [
  cos(theta/2),
  (sin(theta/2)/theta) * phi
]
~~~

At zero:

~~~text
Exp_q(0) = identity
~~~

The equivalent rotation matrix is:

~~~text
Exp_R(phi) = I
           + (sin(theta)/theta) * hat(phi)
           + ((1-cos(theta))/theta²) * hat(phi)²
~~~

### Logarithmic map

`Log_q` returns the principal rotation vector.

Because `q` and `-q` represent the same rotation, first choose a representative with `q0 >= 0`. This produces the same representative away from `q0 = 0`. At `q0 = 0`, both opposite axis signs remain valid principal results for the rotation at `pi`. For `q = [q0, qv]`:

~~~text
theta = 2 * atan2(norm(qv), q0)

Log_q(q) = theta * qv / norm(qv),  when norm(qv) > 0
Log_q(q) = [0, 0, 0],              when norm(qv) = 0
~~~

The principal result satisfies:

~~~text
norm(Log_q(q)) <= pi
~~~

At exactly `pi`, the rotation axis has two equivalent signs. Local inverse behavior is therefore specified on the open domain:

~~~text
Log_q(Exp_q(phi)) = phi,  when norm(phi) < pi
~~~

### Quaternion sign equivalence

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

### Small-angle behavior

As `theta = norm(phi)` approaches zero:

~~~text
Exp_q(phi)
  = [
      1 - theta²/8 + O(theta⁴),
      (1/2 - theta²/48 + O(theta⁴)) * phi
    ]

Exp_R(phi)
  = I + hat(phi) + 1/2 * hat(phi)² + O(theta³)
~~~

Therefore, to first order:

~~~text
Exp_q(phi) ≈ [1, phi/2]
Exp_R(phi) ≈ I + hat(phi)
~~~

The exact behavior at zero is defined by continuity:

~~~text
sin(theta/2)/theta  -> 1/2
sin(theta)/theta    -> 1
(1-cos(theta))/theta² -> 1/2
~~~

For the principal quaternion logarithm, let `r = norm(qv)` after selecting
`q0 >= 0`. When `q0 > 0` and `r` approaches zero, the vector scale is:

~~~text
2 * atan(r/q0) / r
  = 2/q0 * (1 - r²/(3*q0²) + O(r⁴))
~~~

The truncated expressions are approximations, not alternative definitions of
`Exp_q`, `Log_q`, or `Exp_R`.
