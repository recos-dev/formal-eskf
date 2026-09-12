## Summary

Define the C++ language and runtime profile for the shared ESKF core within the boundary established by [Formal Verification Strategy](https://github.com/shengwen-tw/formal-eskf/issues/1). This specification owns source-language and portability constraints.

## Language baseline

The shared core uses C++20 language mode with a verifier-approved subset.

The standard version alone does not authorize every C++20 feature. Each used construct must be accepted by the selected verifier and all supported target compilers.

Allowed language features are:

~~~text
fixed-size class and function templates
integer non-type template parameters
concepts and requires clauses
simple constexpr functions and constants
aggregate and value initialization
enum class
bounded value types
restricted eager arithmetic operators
explicit Result/Status return values
~~~

An operator overload must have the same eager behavior as its named operation. Expression-template behavior is confined to a backend adapter.

## Disallowed core features

The shared ESKF core does not use:

~~~text
dynamic allocation
exceptions
RTTI or dynamic_cast
virtual dispatch
recursion
variable-length arrays
unbounded loops or loops without a compile-time upper bound
mutable global or thread-local state
threads, mutexes, atomics, or volatile device access
filesystem, logging, clocks, or operating-system APIs
compiler-specific vector intrinsics
ranges, coroutines, or format
~~~

Platform adapters may use OS facilities outside the shared core.

## Memory and execution

All matrix dimensions and maximum loop bounds are compile-time constants. Bounded loops may exit early; their maximum work must remain statically bounded.

Estimator state, covariance, and numerical workspace have statically bounded sizes. Large temporary storage is visible in a workspace definition or stack-usage report.

Every object is initialized before use. The implementation does not depend on:

~~~text
out-of-bounds access
invalid lifetime or dangling references
strict-aliasing violations
signed integer overflow
unsequenced side effects
unspecified evaluation order
reinterpretation of object representation
~~~

The core is deterministic for the same input and numerical profile.

## Error handling

Expected numerical failures use explicit return values:

~~~text
Result<T, Error>
Status
~~~

The core does not throw, abort, log, retry, or select a platform-specific fallback. Public transitions commit state only after all required operations succeed.

## Dependency boundary

The shared core may depend only on:

~~~text
project scalar-math operations
project fixed-size linalg abstraction
project SO(3)/UnitQuaternion layer
verifier-approved basic C++ headers
~~~

Eigen types and expressions do not appear in shared-core state, interfaces, or equations. They remain inside `EigenLinalg`.

Standard mathematical functions are isolated behind the scalar-math boundary. Use of standard-library containers requires confirmation by the verifier spike.

## Configuration and reproducibility

Algorithm variants are explicit types, policies, or separate functions. Preprocessor conditionals do not silently change state layout or ESKF equations.

Any conditional compilation that changes verified behavior creates a separate verification target.

A reproducible build record contains:

~~~text
compiler name and exact version
target triple and ABI
C++ standard mode
preprocessor definitions
optimization level
exception and RTTI settings
linker and runtime-library versions
reference to the selected numerical profile
~~~

Floating-point behavior and flags are owned by the scalar/IEEE-754 issue rather than repeated here.

## Trusted computing base

Source-level verification does not by itself verify the compiler, assembler, linker, runtime libraries, operating system, generated machine code, or processor.
