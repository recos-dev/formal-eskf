# F-SOLVE: Symmetric positive-definite linear solve

**ID:** `F-SOLVE`

**Applies to:** Fixed-size linalg solves, supported dimensions and selected scalar/backend profile.

**Uses:** [C-NUMERICAL-PROFILE](../../common/numerical-profile.md), [C-INPUT-DOMAIN](../../common/input-domain.md), [C-FAILURE](../../common/failure.md), [C-NUMERICAL-ACCEPTANCE](../../common/numerical-acceptance.md).

## Basis

The linalg solve surface replaces inversion in Kalman-gain calculation.
The [public solve contract](../../../../../include/formal_eskf/linalg/solve.hpp)
defines checks and aliasing. Use the [shared domain](../../common/input-domain.md)
and [numerical acceptance](../../common/numerical-acceptance.md) obligations.

## Required behavior

For a square symmetric positive-definite system S and matching right-hand side B,
solve S X = B using Cholesky LLT and triangular solves. Reuse one factorization
for all right-hand-side columns; do not form an explicit inverse.

The exact model is S = L L^T with a lower-triangular L and positive diagonal,
L Y = B, then L^T X = Y. The right solve X S = B uses
S X^T = B^T under the same symmetry premise.

| Condition | Required response |
|---|---|
| Non-finite coefficient in S or B | `non_finite_input` before factorization |
| S not exactly symmetric in the selected scalar representation | `not_positive_definite` |
| Failed positive-definite factorization | `not_positive_definite` |
| Non-finite factor or solution | `non_finite_result` |
| Successful factorization and finite solution | `success`, publishing the complete solution |

Propagate backend failure statuses. Preserve output on every failure. Output may
alias either complete input when dimensions match; otherwise storage is disjoint.
The right solve has the same domain/publication contract. The caller owns cleanup
of roundoff asymmetry before calling; the solve shall not silently symmetrize S.

No condition-number cutoff is imposed by this API. Mathematical SPD does not
make a floating-point factorization infallible. Success establishes the stated
checked computation, not a certified condition number, exact residual or
forward-error bound. Deployment conditioning and residual limits remain separate
obligations under the shared numerical acceptance contract.

## Acceptance and verification

Check factor/triangular-solve coefficient correspondence, status propagation,
non-finite and non-SPD cases, multiple right-hand sides, and all permitted whole
object aliases. Exact solve theorems, bounded production-source checks and backend
conformance establish different claims; accepted numerical error needs explicit
input/conditioning domains and residual bounds.
