# Q-NORMALIZE: Normalization

**ID:** `Q-NORMALIZE`

**Applies to:** Exact quaternion/SO(3) behavior; ESKF frame usage is defined by E-STATE.

## Basis

Hamilton, scalar-first active rotations on column vectors. Use the [coefficient convention](multiplication.md).

## Required behavior

For a nonzero quaternion `q`:

~~~text
normalize(q) = q / norm(q)
norm(normalize(q)) = 1
~~~

Normalization does not select a preferred sign. Both `normalize(q)` and `-normalize(q)` are valid representations of the same rotation.

## Acceptance and verification

Establish the stated exact identities on the declared real/unit-quaternion
domain. Checked C++ behavior, scalar assumptions and numerical residual bounds
remain separate obligations; a finite output does not prove a rotation invariant. Normalization does not impose a preferred quaternion sign.
