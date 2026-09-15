# Coding Style Guide

This lightweight C++ style guide is derived from [solvcon](https://github.com/solvcon/solvcon/blob/875cc3a8e731d3e6474745dee6b7b256983ae99a/STYLE.md) and adapted for formal-eskf.

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
- Group explicitly defaulted or deleted special members. Do not add declarations solely for visual uniformity.

## Comments

- Explain intent, assumptions and non-obvious numerical choices, rather than narrating statements.
- Use Doxygen comments for useful API documentation. Reference the relevant equation or specification when it explains the implementation; do not duplicate entire specifications.
