# Q-ROT-COMPOSE: Rotation composition

**ID:** `Q-ROT-COMPOSE`

**Status:** Draft

**Applies to:** Unit real quaternions and checked finite-precision rotation composition.

**Uses:** [Q-MUL](multiplication.md), [Q-NORMALIZE](normalization.md).

## Basis

Hamilton, scalar-first active rotations on column vectors. Use the [coefficient convention](multiplication.md).

## Required behavior

Composition follows the frame order:

~~~text
q_ac = q_ab ⊗ q_bc
R(q_ac) = R(q_ab) * R(q_bc)
~~~

The rightmost rotation is applied first.

`compose_normalized` shall compute the Hamilton product, then apply
[Q-NORMALIZE](normalization.md) with the supplied threshold. Propagate that
normalization status unchanged. Publish only on success; failure preserves output,
including an output alias to either complete input. The raw multiplication
operator does not normalize or return a status. ESKF callers own any remapping
of an invalid intermediate product to a caller-facing failure category.

## Acceptance and verification

Establish the rotation-order identity for unit real inputs. Check the product to
normalization handoff, unchanged threshold/status, publication and aliases in
the selected scalar/backend profile. Neither a finite output nor normalization
success proves a floating-point rotation invariant.
