## Summary

Verification follows explicit requirements through mathematical models, production C++ and retained execution evidence. Each claim states which behavior, assumptions and configuration it covers. Runtime and deployment claims require evidence for their own boundaries in addition to the numerical core.

The intended deployment environments include embedded Linux, FreeRTOS and NuttX. The estimator uses fixed-size linear algebra, nonlinear quaternion operations and IEEE-754 arithmetic.

This specification owns the verification claim boundary. Mathematical behavior belongs to [linear algebra](eskf-linear-algebra-abstraction-layer.md), [SO(3)](so3-and-unit-quaternion-abstraction-layer.md) and [state conventions](eskf-state-conventions.md); numerical and language constraints belong to [scalar semantics](scalar-ieee754-and-failure-semantics.md) and [C++ portability](cpp-portability-and-constraints.md).

The [requirements](verification-requirements.md) define acceptance obligations. The [traceability and review criteria](formal-verification-traceability-and-review-criteria.md) define record ownership, evidence acceptance and change-impact review. This requirement-led assurance approach supports future ISO certification work; it does not make tool success a certification claim.

## Numerical-core verification boundary

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
│ Fixed-Array Proof Backend   │   │ Pluggable Matrix Backends   │
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

Source-level formal claims cover the analyzed shared-core paths instantiated with the fixed-array proof backend, subject to the accepted evidence for their dependencies. Each claim is scoped by its specified properties, assumptions and verification configurations; the diagram does not assert that every path or backend operation has been proved.

If a target instead deploys an external matrix backend such as Eigen or PX4 matrix, the claim is limited to:

> The shared ESKF algorithm satisfies the proved properties under the required linear-algebra and scalar-math contracts.

This does not constitute formal verification of external backend internals, the platform math library, or all executions of those dependencies. Conformance and differential tests provide supporting evidence only. Runtime scheduling, sensor lifecycle and adapter publication require their own [software obligations](eskf-software-architecture-and-verification-boundaries.md#software-proof-obligations).

## Scope and completion

Each claimed property must record its specification, actual source, scalar and backend profile, dimensions, macro settings, assumptions, tool versions and successful execution results. A missing or stale dependency leaves a gap; timeouts and incomplete unwinding are not passing results.

Exact algebra, bounded implementation checks, inductive runtime safety and target numerical guarantees must be reported separately. Source verification does not verify the compiler, generated machine code, operating system or CPU.
