# Coding Style Guide

This guide defines the C++ naming, readability and organization rules for formal-eskf.

ArduPilot's actual same-layer practice is the primary estimator-design reference.
The [design baseline](../spec/requirements/common/cpp-subset.md#design-baseline)
pins the reference and defines its application while retaining C++20; it does not
import ArduPilot's allocation, floating-point or platform conventions.

The [C++ subset](../spec/requirements/common/cpp-subset.md#language-subset) owns
language permissions and template boundaries. This guide owns naming, readability
and organization; [.clang-format](../../.clang-format) owns formatting.

## Readability

- Express estimator steps with ordinary types, named functions and direct control flow. A reader should be able to follow the numerical or fusion sequence without tracing a type-dispatch framework.
- Use short, meaningful aliases for the types resolved at the configuration boundary.
- Share common mathematics; accept small duplication when removing it would require more dispatch or forwarding layers. Follow the subset's INS/AHRS and template rules.
- Give each helper a numerical, contract or state-management responsibility. Avoid wrappers that only rename or forward an existing operation without providing a needed interface boundary.
- Group related data when it has a common meaning or lifetime. Do not shorten argument lists by hiding unrelated dependencies in a catch-all context, global object or service framework.
- Keep ownership and update order visible. Preserve numerical checks, failure atomicity, aliasing rules and the distinction between rejection and failure when simplifying code.

## Naming

- Use `CamelCase` for classes, structs, enums and template parameters: `UnitQuaternion`, `Linalg`, `Rows`.
- Use `snake_case` for functions, variables and ordinary constants: `normalize`, `minimum_norm`, `row_count`. Reserve `UPPER_CASE` for macros.
- Use descriptive type aliases ending in `_type` or `_t`, such as `value_type` and `matrix_type`.
- Name public operations and internal helpers by their action, without a `try_` prefix. Return types and documented contracts describe failure. Keep `correct_*` for numerical correction and `fuse_*` for operations that also apply fusion policy.
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

From the repository root, check formatting without changing files:

```bash
./scripts/format_cpp.sh --check
```

## Types and Interfaces

- Write references to const objects as `Type const &`.
- Follow the subset's rules for type deduction, classes/aggregates and borrowing. Keep important interface types and ownership visible; do not add accessor wrappers to plain data.
- Group explicitly defaulted or deleted special members. Do not add declarations solely for visual uniformity.

## Comments

- Explain intent, assumptions and non-obvious numerical choices, rather than narrating statements.
- Use Doxygen comments for useful API documentation. Reference the relevant equation or specification when it explains the implementation; do not duplicate entire specifications.
- Keep essential preconditions, units/frames and failure behavior visible at the API. Put extended derivations and shared contract details in the linked specification.

## Documentation

- Write project documentation in English, including specifications, requirements, guides and analysis drafts.
- Use the [specification template](../spec/TEMPLATE.md) for requirement writing and approval, and the [documentation index](../README.md#evidence-and-maintenance) for record ownership and maintenance.
- Lead with the behavior or decision, then its rationale and assumptions. Link the authoritative contract instead of copying it. Keep README files focused on entry points, operation and current limitations.

## Applying This Guide

Apply these rules to new code and changes within the current task. Existing code may predate them; its presence is not permission to extend an unnecessarily complex pattern. Record out-of-scope cleanup for review rather than rewriting unrelated code. This guide does not authorize an architecture refactor, public API change or numerical behavior change by itself.
