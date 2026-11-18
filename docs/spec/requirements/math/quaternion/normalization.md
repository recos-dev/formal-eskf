# Q-NORMALIZE: Normalization

**ID:** `Q-NORMALIZE`

**Status:** Draft

**Applies to:** Nonzero real quaternion coefficients; checked construction and normalization in the selected scalar/backend profile.

**Uses:** [C-NUMERICAL-PROFILE](../../common/numerical-profile.md), [C-FAILURE](../../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../../common/numerical-acceptance.md).

## Basis

Hamilton, scalar-first active rotations on column vectors. Use the [coefficient convention](multiplication.md).

## Required behavior

For a nonzero quaternion `q`:

~~~text
normalize(q) = q / norm(q)
norm(normalize(q)) = 1
~~~

Normalization does not select a preferred sign. Both `normalize(q)` and `-normalize(q)` are valid representations of the same rotation.

### Checked construction and normalization

`UnitQuaternion::from_coefficients` and `so3::normalize` shall first reject a
non-finite `minimum_norm` as `non_finite_input`, then a nonpositive threshold as
`domain_error`. There is no upper bound of one at this layer; ESKF callers impose
their own narrower threshold domain.

Reject non-finite coefficients as `non_finite_input`. Compute their norm; a
non-finite norm is `non_finite_result`, and a finite norm below the threshold is
`invalid_quaternion_norm`. Equality at the threshold is accepted. Divide every
coefficient by that computed norm and require every result to be finite before
publishing; otherwise return `non_finite_result`. Failure preserves the complete
output. In-place quaternion normalization is supported. Do not replace invalid
input with identity or apply a preferred sign.

## Acceptance and verification

Establish the exact identities for every nonzero real quaternion, without assuming
the input is already unit. Check threshold/error order, coefficient division and
failure/alias preservation separately. Finite normalized coefficients do not
establish a floating-point unit-norm residual bound.
