# F-LINALG: Fixed-size linear algebra operations

**ID:** `F-LINALG`

**Status:** Draft

**Applies to:** Compile-time dimensions, supported scalar/backend profiles and the shapes below.

**Uses:** [C-FAILURE](../../common/failure.md),
[C-NUMERICAL-PROFILE](../../common/numerical-profile.md),
[C-NUMERICAL-ACCEPTANCE](../../common/numerical-acceptance.md).

## Basis

The backend boundary and shared-operation allocation are defined by [numerical architecture](../../../architecture/numerics.md).

## Required behavior

### Summary

This specification defines the minimum fixed-size linear-algebra surface required by the ESKF. It separates backend-dependent primitives from shared derived operations and ESKF-specific domain logic, and records the vector and matrix dimensions required by the current algorithm.

### Required operations

The shared interface exposes these operations. Backend-dependent primitives
are defined by the [minimal backend surface](../../../architecture/numerics.md#minimal-backend-surface).

#### 1. Fixed-size types

~~~text
Scalar
Vec<N>
Mat<R,C>
~~~

Only compile-time-fixed dimensions are needed.

The C++ value types are `linalg::Matrix<Backend, Rows, Columns>` and
`linalg::Vector<Backend, Size>`. `Vector` is an alias for an owning `Size x 1`
matrix, with identical storage and operators. These are the sole public type
entry points; backends supply storage and primitive operations, not additional
`matrix_type` or `vector_type` aliases. Bind short mathematical type names at
configuration boundaries; do not introduce another traits or dispatch layer.
Include the frontend header and the selected backend header explicitly.

#### 2. Construction

~~~text
zero<R,C>()
identity<N>()
from_coefficients<R,C>(...)
~~~

#### 3. Coefficient and block access

~~~text
A(row, column)
A(row, column) = value
v(index)
v(index) = value
get_block<BR,BC>(A, row, column)
set_block<BR,BC>(A, row, column, B)
get_segment<N>(v, offset)
set_segment<N>(v, offset, x)
~~~

Block and segment operations are included initially because the current ESKF constructs transition and observation matrices from fixed-size blocks.

Vector indexing and segments use the corresponding matrix indexing and block
operations. Extraction returns an owning value, not a view. Block/segment bounds
are checked at compile time without relying on potentially overflowing index
addition. Coefficient reads and writes use `operator()`; there is no separate
`set()` or backend `set_coefficient()` interface. Shared free operations use
internal `MatrixAccess` to reach backend storage without making it part of the
public matrix API.

#### 4. Basic arithmetic

~~~text
add(A, B)
subtract(A, B)
negate(A)
scale(scalar, A)
divide(A, scalar)
~~~

Restricted operator overloads such as +, -, and scalar * express these same operations. The list defines the semantic surface, not a required spelling for every C++ function.

#### 5. Products

~~~text
multiply(Mat<R,K>, Mat<K,C>) -> Mat<R,C>
multiply(Mat<R,C>, Vec<C>)   -> Vec<R>
dot(Vec<N>, Vec<N>)          -> Scalar
~~~

#### 6. Transpose

~~~text
transpose(Mat<R,C>) -> Mat<C,R>
~~~

#### 7. Vector magnitude

~~~text
squared_norm(Vec<N>) -> Scalar
norm(Vec<N>)         -> Scalar
~~~

#### 8. Scalar-math dependency

~~~text
scalar_math_type
~~~

Each linalg backend identifies the scalar-math implementation used for
finite-value inspection and elementary functions. `all_finite(Vec<N>)` and
`all_finite(Mat<R,C>)` are shared bounded loops over `is_finite(Scalar)`; they
are not separate linalg-backend primitives.

#### 9. Symmetric positive-definite solve

Use [F-SOLVE](spd-solve.md).

### Shared derived operations

These operations are required by the ESKF, but should be implemented once on top of the backend primitives rather than independently in every backend.

~~~text
normalize(v)
cross(a, b)
trace(A)
diagonal(A)
max_abs(A)
symmetrize(A)
is_symmetric(A, tolerance)
sandwich(A, P) = A * P * transpose(A)
right_solve_spd(B, S)          // X*S = B
~~~

Keeping these shared prevents the backends from developing different mathematical behavior. The hat/skew rotation convention is defined by [SO3-HAT](../so3/hat.md), not by a second linalg definition.

#### Checked normalization

`normalize(input, minimum_norm, output)` shall normalize a vector using the
selected backend's computed norm. `minimum_norm` is an absolute magnitude in
the input's units, not a unit-norm residual tolerance. The exact-real reference
is `input / sqrt(dot(input, input))` for nonzero input.

The following rows define failure precedence; each later row applies only when
the preceding rows did not reject. In particular, a nonfinite computed norm
precedes rejection of a finite nonpositive threshold. This preserves the
existing checked API's observable status ordering.

| Condition | Required response |
|---|---|
| Any input coefficient or `minimum_norm` is NaN or infinity | Return `non_finite_input`. |
| The computed norm is NaN or infinity | Return `non_finite_result`. |
| `minimum_norm <= 0` or computed norm `< minimum_norm` | Return `zero_or_unsafe_divisor`; do not divide. |
| Any coefficient of the candidate `input / computed_norm` is nonfinite | Return `non_finite_result`. |
| Otherwise | Return `success` and publish every candidate coefficient. |

Equality with a positive threshold shall be accepted. Positive and negative
zero thresholds shall be rejected. Any failure shall leave every output
coefficient unchanged, including signed zeros and existing nonfinite values.
Input and output may be the same vector; the complete candidate shall be
validated before publication. A disjoint input shall remain unchanged.

Success establishes a finite candidate and the specified divisor checks. It
does not establish an exact unit norm or a general residual bound; backend norm
and division errors, including underflow, require the numerical acceptance
profile below. No rescaling or fallback vector is implied.

#### Finite inspection and maximum magnitude

`all_finite(A)` shall be true exactly when every coefficient is finite.
`max_abs(A)` shall return the largest absolute coefficient for finite inputs;
an all-zero matrix shall return positive zero. These operations shall preserve
the input.

`max_abs` shall not hide a nonfinite coefficient behind a finite result. With
NaNs and no infinities it shall return NaN; with infinities and no NaNs it shall
return positive infinity. With both present, either NaN or positive infinity
is permitted. NaN payload and which nonfinite value is encountered first are
not API guarantees. Callers needing a validity decision shall use finite
classification, not rely on a comparison with the returned nonfinite value.

#### Symmetry

`symmetrize(A)` is unchecked arithmetic for `(A + transpose(A)) / 2`, evaluated
by addition followed by scaling by one half. It shall return an owning matrix
and preserve the input. It shall not promise finite output when the addition
overflows, repair nonfinite data, or establish positive semidefiniteness.

`is_symmetric(A, tolerance)` shall return false if any matrix coefficient or
the tolerance is nonfinite, or if the tolerance is negative. For finite inputs
and a nonnegative tolerance, form the scalar residuals `abs(A(i,j) - A(j,i))`.
Any nonfinite residual shall return false, even for a finite large tolerance.
Otherwise the result shall be true exactly when the maximum residual is
`<= tolerance`. The tolerance is absolute, in matrix-coefficient units; equality
is accepted and either signed zero is a valid zero tolerance. The operation
shall preserve its inputs. It checks symmetry only, not covariance validity.

### ESKF dimensions currently required

The abstraction must support INS (16 nominal coefficients, 15 error coordinates) and AHRS (4 nominal coefficients, 3 error coordinates). For error dimension D and measurement dimension M, this includes MxD observation Jacobians, DxM gains and MxM innovation systems.

Required compile-time dimensions remain:

~~~text
Vectors: 1, 2, 3, 4, 6, 15, 16

Matrices:
  1x1, 1x3, 1x15, 3x1, 15x1
  2x3, 3x2, 6x3, 3x6
  2x2, 2x15, 2x16, 15x2
  3x3, 3x15, 3x16, 15x3
  4x3, 4x4, 4x15, 4x16, 15x4
  6x6, 6x15, 6x16, 15x6
  12x12, 12x15, 15x12
  15x15
  16x15
~~~

This is an abstraction-support requirement, not a claim that every shape has
a production observation API or a completed proof profile.

Current corrections use measurement sizes 1, 2 and 3; generic matrix operations
remain dimension templates. INS propagation uses a 15x15 state covariance,
12x12 source-noise covariance and 15x12 noise injection; AHRS uses 3x3 attitude
error covariance. Quaternion coefficients use four-element storage.

The 16 nominal INS coefficients do not make the correction operate on a 16x16
covariance. Current observation code derives Mx15 error-state Jacobians directly.
An Mx16 nominal Jacobian and 16x15 perturbation lift may be used for derivation,
but are not required C++ correction interfaces or extra state dimensions.

## Acceptance and verification

For each supported backend/profile, check dimensions, initialization, access,
block/segment value semantics and arithmetic correspondence. Use symbolic source
checks only for their registered shapes/operations; native conformance does not
prove backend internals or all template instantiations. Raw arithmetic is distinct
from checked normalization and solve; preserve their own domain/failure contracts.

For the derived clauses above, cover threshold equality and adjacent represented
values, overlapping rejection conditions, nonfinite/overflow results and
normalization with disjoint and identical input/output. Check exact fixtures and
the specified failure frame independently of general numerical accuracy.
Controlled norm/division producers can establish the wrapper's result handling;
they do not establish the actual backend's norm or division conformance.

## Open decisions

General normalization/reduction accuracy still requires bounded input domains,
scalar/backend/compiler profiles and independently justified error budgets under
C-NUMERICAL-ACCEPTANCE. In particular, finite normalized coefficients alone do
not discharge a unit-norm residual bound. The
[test design](../../../verification/tests/math/scalar-linalg.md) identifies
bounded fixtures and remaining evidence obligations; passing them does not
approve this Draft or establish all-input numerical acceptance.
