## Summary

Define a layered formal-verification strategy for a portable ESKF core.

The intended deployment environments include embedded Linux, FreeRTOS and NuttX. The estimator uses fixed-size linear algebra, nonlinear quaternion operations and IEEE-754 arithmetic.

This specification owns the verification claim boundary. Mathematical behavior belongs to [linear algebra](https://github.com/shengwen-tw/formal-eskf/issues/2), [SO(3)](https://github.com/shengwen-tw/formal-eskf/issues/3) and [state conventions](https://github.com/shengwen-tw/formal-eskf/issues/4); numerical and language constraints belong to [scalar semantics](https://github.com/shengwen-tw/formal-eskf/issues/5) and [C++ portability](https://github.com/shengwen-tw/formal-eskf/issues/6).


## Proposed verification boundary

~~~text
┌────────────────────────────────────────────────────────────┐
│                 Mathematical Specifications                │
│          Linear Algebra · SO(3) · ESKF Transitions         │
└─────────────────────────────┬──────────────────────────────┘
                              │
                              ▼
┌────────────────────────────────────────────────────────────┐
│          Shared, Platform-Independent C++ ESKF Core        │
└─────────────────────────────┬──────────────────────────────┘
                              │
                    ┌─────────┴─────────┐
                    │                   │
                    ▼                   ▼
┌─────────────────────────────┐   ┌─────────────────────────────┐
│ Verified Fixed-Array Backend│   │ Pluggable Matrix Backends   │
├─────────────────────────────┤   ├─────────────────────────────┤
│ • Proof harnesses           │   │ • EigenBackend              │
│ • Symbolic inputs           │   │ • Px4MatrixBackend          │
│ • Formal verification       │   │ • Conformance tests         │
│                             │   │ • Differential tests        │
└─────────────────────────────┘   └─────────────────────────────┘
~~~

## Evidence responsibilities

| Evidence | What it establishes | What it does not establish |
|---|---|---|
| Lean 4 | Exact properties of explicitly defined mathematical or abstract transition models | Every IEEE-754 execution of C++ |
| ESBMC | Stated properties of the analyzed C++ paths under recorded assumptions, bounds and profiles | Untested dimensions, macro variants, backends or target environments |
| Agent review | Reviewed correspondence of specifications, Lean definitions, C++ and harness assumptions | A machine-checked Lean-to-C++ refinement theorem |
| Native tests and static analysis | Regression, backend conformance and implementation diagnostics | Formal proof of all inputs or dependencies |

Use `Lean 4` and `ESBMC 8.4` for the current proof workflow. Neither tool becomes a runtime dependency of the estimator. A separately rewritten C++ model is not a substitute for checking the production implementation.

## Formal proof claim

The formal proof applies directly to the shared ESKF core instantiated with the verified fixed-array backend. Each claim is scoped by its specified properties, assumptions and verification configurations.

If a target instead deploys Eigen or PX4 matrix, the claim is limited to:

> The shared ESKF algorithm satisfies the proved properties under the required linear-algebra and scalar-math contracts.

This does not constitute formal verification of Eigen or PX4 matrix internals, the platform math library, or all executions of those dependencies. Conformance and differential tests provide supporting evidence only.

## Scope and completion

Each claimed property must record its specification, actual source, scalar and backend profile, dimensions, macro settings, assumptions, tool versions and successful execution results. A missing or stale dependency leaves a gap; timeouts and incomplete unwinding are not passing results.

Exact algebra, bounded implementation checks, inductive runtime safety and target numerical guarantees must be reported separately. Source verification does not verify the compiler, generated machine code, operating system or CPU.
