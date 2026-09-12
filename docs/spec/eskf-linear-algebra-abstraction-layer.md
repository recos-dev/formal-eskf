## Summary

This specification defines the minimum fixed-size linear-algebra surface required by the ESKF. It separates backend-dependent primitives from shared derived operations and ESKF-specific domain logic, and records the vector and matrix dimensions required by the current algorithm.

## Proposed boundary

The ESKF algorithm should depend on a compile-time backend:

~~~text
EskfCore<Linalg>
    |
    +-- EigenBackend<float>
    +-- EigenBackend<double>
    +-- Px4MatrixBackend<Scalar>
    +-- VerificationLinalg<Scalar>
~~~

The ESKF equations are written once in EskfCore. Only generic linear-algebra operations are backend-dependent. The names above are schematic; backend selection is independent of the INS/AHRS configuration.

## Backend primitives

These operations need an abstract backend because their implementation differs between Eigen and a verifier-friendly fixed-array implementation.

### 1. Fixed-size types

~~~text
Scalar
Vec<N>
Mat<R,C>
~~~

Only compile-time-fixed dimensions are needed.

### 2. Construction

~~~text
zero<R,C>()
identity<N>()
from_coefficients<R,C>(...)
~~~

### 3. Coefficient and block access

~~~text
get(A, row, column)
set(A, row, column, value)
get_block<BR,BC>(A, row, column)
set_block<BR,BC>(A, row, column, B)
get_segment<N>(v, offset)
set_segment<N>(v, offset, x)
~~~

Block and segment operations are included initially because the current ESKF constructs transition and observation matrices from fixed-size blocks.

### 4. Basic arithmetic

~~~text
add(A, B)
subtract(A, B)
negate(A)
scale(scalar, A)
divide(A, scalar)
~~~

These may later use restricted operator overloads such as +, -, and scalar *, but they remain part of the abstract semantic surface.

### 5. Products

~~~text
multiply(Mat<R,K>, Mat<K,C>) -> Mat<R,C>
multiply(Mat<R,C>, Vec<C>)   -> Vec<R>
dot(Vec<N>, Vec<N>)          -> Scalar
~~~

### 6. Transpose

~~~text
transpose(Mat<R,C>) -> Mat<C,R>
~~~

### 7. Vector magnitude

~~~text
squared_norm(Vec<N>) -> Scalar
norm(Vec<N>)         -> Scalar
~~~

### 8. Scalar-math dependency

~~~text
scalar_math_type
~~~

Each linalg backend identifies the scalar-math implementation used for
finite-value inspection and elementary functions. `all_finite(Vec<N>)` and
`all_finite(Mat<R,C>)` are shared bounded loops over `is_finite(Scalar)`; they
are not separate linalg-backend primitives.

### 9. Symmetric positive-definite solve

~~~text
solve_spd(
    Mat<N,N> S,
    Mat<N,M> B,
    Mat<N,M> X) -> Status
~~~

This replaces direct matrix inversion and covers the LLT-based solve used by the Kalman gain calculation.

## Shared derived operations

These operations are required by the ESKF, but should be implemented once on top of the backend primitives rather than independently in every backend.

~~~text
try_normalize(v)
cross(a, b)
trace(A)
diagonal(A)
max_abs(A)
symmetrize(A)
is_symmetric(A, tolerance)
sandwich(A, P) = A * P * transpose(A)
right_solve_spd(B, S)          // X*S = B
~~~

Keeping these shared prevents the backends from developing different mathematical behavior. The hat/skew rotation convention is defined by [SO(3) and Unit Quaternion](https://github.com/shengwen-tw/formal-eskf/issues/3), not by a second linalg definition.

## ESKF dimensions currently required

The abstraction must support INS (16 nominal coefficients, 15 error coordinates) and AHRS (4 nominal coefficients, 3 error coordinates). For error dimension D and measurement dimension M, this includes MxD observation Jacobians, DxM gains and MxM innovation systems.

Required compile-time dimensions include:

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

The `Mx16` forms are INS nominal-state observation Jacobians. A model that uses this representation obtains an `Mx15` error-state Jacobian through the `16x15` perturbation mapping. Models may instead derive the error-state Jacobian directly; this chain is not a mandatory correction API.

## Minimal backend surface

The proposed minimum is:

~~~text
types
zero / identity / coefficient construction
coefficient / block / segment access
add / subtract / negate / scale / divide
matrix-matrix multiply
matrix-vector multiply
dot / squared_norm / norm
transpose
SPD solve
~~~

Everything else should be shared code above this surface.
