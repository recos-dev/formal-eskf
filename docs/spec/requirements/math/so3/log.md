# SO3-LOG: Logarithmic map

**ID:** `SO3-LOG`

**Applies to:** Exact quaternion/SO(3) behavior; ESKF frame usage is defined by E-STATE.

## Basis

Hamilton, scalar-first active rotations on column vectors. Use the [coefficient convention](../quaternion/multiplication.md).

## Required behavior

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

## Acceptance and verification

Establish the stated exact identities on the declared real/unit-quaternion
domain. Checked C++ behavior, scalar assumptions and numerical residual bounds
remain separate obligations; a finite output does not prove a rotation invariant.
