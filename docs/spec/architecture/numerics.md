# Numerical Architecture

## Scope and requirements

The numerical core composes scalar, fixed-size linalg, SO(3) and ESKF operations.
It owns no clock, sensor qualification or publication. Follow the layer contract.

## Backend boundary

The ESKF algorithm depends on a compile-time backend:

~~~text
EskfCore<Linalg>
    |
    +-- EigenBackend<float>
    +-- EigenBackend<double>
    +-- Px4MatrixBackend<Scalar>
    +-- VerificationLinalg<Scalar>
~~~

ESKF equations use shared operations above the backend. The names above are schematic, not a required `EskfCore` class or an exhaustive backend list; backend selection is independent of the INS/AHRS configuration. The actual ownership and source layout are in the [architecture specification](overview.md#current-implementation-and-ownership).

## Minimal backend surface

The required minimum is:

~~~text
storage and scalar types
zero initialization / coefficient access
add / subtract / negate / scale / divide
matrix-matrix multiply
matrix-vector multiply
dot / squared_norm / norm
transpose
SPD solve
~~~

Identity, row-major construction, blocks and segments are shared operations
implemented through these primitives. Everything else should be shared code
above this surface.

## Verification boundary

Backend conformance and differential tests do not prove external backend internals.
Source proofs with the fixed-array backend remain conditional on the matched scalar
and linalg contracts. INS/AHRS and numerical modes are explicit configurations.
