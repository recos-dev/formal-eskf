# SO3-SMALL-ANGLE: Small-angle behavior

**ID:** `SO3-SMALL-ANGLE`

**Status:** Draft

**Applies to:** Rotation vectors near zero and principal unit quaternions near identity, with q0 > 0 for the Log expansion.

## Basis

Hamilton, scalar-first active rotations on column vectors. Use the [coefficient convention](../quaternion/multiplication.md).

## Required behavior

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

### C++ branch binding

Let epsilon be the selected scalar format's machine epsilon. For Exp, the Taylor
branch applies when the computed t=theta² is <= epsilon. Its candidate uses
`q0 = 1 - t/8 + t²/384` and `qv = (1/2 - t/48 + t²/3840) * phi`, followed by
[Q-NORMALIZE](../quaternion/normalization.md). Above the threshold use checked
square root and half-angle sine/cosine.

For Log, computed r²=0 returns zero first. For 0 < r² <= epsilon, require
computed q0² > 0, then use `2/q0 * (1 - r²/(3*q0²))`; an unsafe divisor returns
`zero_or_unsafe_divisor`. Above the threshold use checked square root and atan2.
Both maps retain their finite-result/status/publication checks. These branches
do not supply a universal numerical error tolerance or principal-range proof.

## Acceptance and verification

Establish the limiting/Taylor relations in the stated near-zero domains. Check
zero, threshold equality and both executable branches separately. Bound
truncation, rounding and branch transitions before asserting numerical accuracy;
a finite result does not establish these bounds.
