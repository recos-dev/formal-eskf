## Summary

Generate a source-derived inventory of project C++ functions, cross-reference their verification requirements and evidence, and produce a deterministic queue for agent-assisted review. Newly added or unmapped code must remain visible even when every currently registered requirement passes. The claim boundary follows [Formal Verification Strategy](https://github.com/shengwen-tw/formal-eskf/issues/1).

## Methodology

Use Clang's source AST as the primary inventory, then connect discovered functions to verification requirements and existing Lean/ESBMC evidence. Extend the existing traceability and agent-review workflow rather than creating a separate proof registry. [Clang AST](https://clang.llvm.org/docs/IntroductionToTheClangAST.html).

## Inventory boundary and identity

Cover project-authored functions, including internal helpers, within the declared source and configuration scope. External libraries remain explicit dependencies, not implicitly verified project code.

Distinguish function identity, configuration and required property. One function may need several proofs, and one proof may cover several functions; matching function, Lean theorem and ESBMC harness counts is not required.

## Configuration completeness

Track evidence against the supported estimator configurations, backends and numerical profiles. Missing configurations must remain visible; evidence for one profile must not silently cover another.

## Review pipeline

```text
┌────────────────────────────────────────────────────────────┐
│          Source/Header Manifest · Build Profiles           │
└─────────────────────────────┬──────────────────────────────┘
                              │
                              ▼
┌────────────────────────────────────────────────────────────┐
│                Deterministic AST Inventory                 │
└─────────────────────────────┬──────────────────────────────┘
                              │
          ┌───────────────────┴───────────────────┐
          │ Requirements · Lean/ESBMC Evidence    │
          └───────────────────┬───────────────────┘
                              │
                              ▼
┌────────────────────────────────────────────────────────────┐
│                   Mechanical Gap Checks                    │
└─────────────────────────────┬──────────────────────────────┘
                              │
                              ▼
┌────────────────────────────────────────────────────────────┐
│                       AI Review Queue                      │
└─────────────────────────────┬──────────────────────────────┘
                              │
                              ▼
┌────────────────────────────────────────────────────────────┐
│              Reviewed Mapping · Open Findings              │
└────────────────────────────────────────────────────────────┘
```

Missing mappings, incomplete evidence and unresolved inconsistencies enter the review queue. Changes to code, dependencies, configurations or proofs return affected items for review.

## Agent-assisted consistency audit

The agent compares implementation behavior, specifications and proof assumptions, including conventions, input domains, equations and failure handling. Record findings using the `PASS`/`GAP`/`NUMERICAL`/`MISMATCH` classifications.

The review must not weaken requirements, exclude missing evidence or modify production behavior merely to obtain a passing result.
