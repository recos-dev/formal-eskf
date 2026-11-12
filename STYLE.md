# Coding Style Guide

This lightweight C++ style guide is derived from [solvcon](https://github.com/solvcon/solvcon/blob/875cc3a8e731d3e6474745dee6b7b256983ae99a/STYLE.md) and adapted for formal-eskf.

For estimator design, the primary reference is ArduPilot's actual same-layer practice: concrete mathematical types and direct calls in [its EKF](https://github.com/ArduPilot/ardupilot/blob/546cf152a8c868acbac82dd326f9649da66d6ba1/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp#L625), alongside its [style guide](https://ardupilot.org/dev/docs/style-guide.html). The [design baseline and template boundaries](docs/spec/cpp-portability-and-constraints.md#design-baseline) define how to apply that reference while retaining C++20. ArduPilot's allocator, floating-point flags and platform conventions are not adopted by this reference.

The [C++ portability and constraints specification](docs/spec/cpp-portability-and-constraints.md#language-subset) owns the permitted language subset and examples. This guide owns naming, readability and organization. `.clang-format` owns formatting.

## Readability

- Express estimator steps with ordinary types, named functions and direct control flow. A reader should be able to follow the numerical or fusion sequence without tracing a type-dispatch framework.
- Keep templates within the specified numerical and configuration boundaries. Resolve related types together and use short, meaningful aliases throughout the implementation.
- Use explicit overloads or named functions for distinct INS/AHRS behaviors instead of state-type detection inside a generic dispatcher. Share common mathematics; accept small duplication when removing it would require more type detection or forwarding layers.
- Give each helper a numerical, contract or state-management responsibility. Avoid wrappers that only rename or forward an existing operation without providing a needed interface boundary.
- Group related data when it has a common meaning or lifetime. Do not shorten argument lists by hiding unrelated dependencies in a catch-all context, global object or service framework.
- Keep ownership and update order visible. Preserve numerical checks, failure atomicity, aliasing rules and the distinction between rejection and failure when simplifying code.

## Naming

- Use `CamelCase` for classes, structs, enums and template parameters: `UnitQuaternion`, `Linalg`, `Rows`.
- Use `snake_case` for functions, variables and ordinary constants: `try_normalize`, `minimum_norm`, `row_count`. Reserve `UPPER_CASE` for macros.
- Use descriptive type aliases ending in `_type` or `_t`, such as `value_type` and `matrix_type`.
- Prefix encapsulated data members with `m_`. Plain state and parameter structs keep unprefixed fields.
- Preserve established mathematical notation when its meaning is clear from the surrounding model, such as `q_nb`, `R`, `H` and `K`. Otherwise, prefer descriptive names.

## Layout and Organization

- Use UTF-8 and LF line endings. Let `.clang-format` control indentation, spacing and wrapping.
- Always brace control-flow bodies. Keep trivial accessors compact; do not place multiple executable statements on one line.
- Separate logical groups with a blank line, without padding short functions.
- Use `#pragma once` in headers and include public project headers as `<formal_eskf/...>`.
- Keep library declarations in the `formal_eskf` namespace. Do not introduce namespace-wide `using namespace` directives.
- Keep nontrivial member definitions outside the class declaration when this makes the interface easier to scan. Template definitions can remain in the same header.
- Use closing comments for long class and namespace blocks, following the existing `/* end class Matrix */` convention.

Check formatting without changing files:

```bash
./scripts/format_cpp.sh --check
```

## Types and Interfaces

- Write references to const objects as `Type const &`.
- Use `auto` when the type is apparent or spelling it out would obscure an expression. Keep important interface types explicit.
- Use classes for encapsulated invariants; plain state and parameter aggregates do not need accessor wrappers.
- Required borrowed inputs and outputs normally use references. Use non-owning pointers when absence or another explicit interface requirement makes them appropriate; document lifetime and null behavior.
- Group explicitly defaulted or deleted special members. Do not add declarations solely for visual uniformity.

## Comments

- Explain intent, assumptions and non-obvious numerical choices, rather than narrating statements.
- Use Doxygen comments for useful API documentation. Reference the relevant equation or specification when it explains the implementation; do not duplicate entire specifications.
- Keep essential preconditions, units/frames and failure behavior visible at the API. Put extended derivations and shared contract details in the linked specification.

## Documentation

- Lead with the behavior or decision, then its rationale and assumptions. State required success/failure behavior and link the applicable requirement and evidence boundary.
- Keep requirements and contracts in `docs/spec/`. Use stable requirement IDs; describe sources, units, frames, configuration scope and exclusions where relevant. Distinguish adopted rules from proposals and existing implementation.
- Preserve the requirement-to-design/source-to-evidence chain defined by the [review criteria](docs/spec/formal-verification-traceability-and-review-criteria.md#record-ownership). Simplifying prose must not remove acceptance conditions, assumptions or unresolved gaps.
- Keep current functional work in `FEATURE_TODO.md` and architecture/verification plans in `ROADMAP.md`. Reference their existing IDs instead of creating another task list.
- Keep README files focused on entry points, operation and current limitations. Put dated validation summaries separately, retaining source/configuration identity, evidence paths, failed attempts and claim limits; never present historical results as current verification.
- Link to the authoritative definition instead of copying changing counts or full contracts. When a requirement, implementation boundary or evidence source changes, review affected links and summaries as well as the traceability mapping.

## Applying This Guide

Apply these rules to new code and changes within the current task. Existing code may predate them; its presence is not permission to extend an unnecessarily complex pattern. Record out-of-scope cleanup for review rather than rewriting unrelated code. This guide does not authorize an architecture refactor, public API change or numerical behavior change by itself.
