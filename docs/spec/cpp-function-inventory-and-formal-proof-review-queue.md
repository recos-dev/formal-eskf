## Summary

This specification defines a source-derived inventory of project C++ functions and its intended connection to requirements, evidence and agent-assisted review. Newly added or unmapped code must remain visible even when every currently registered requirement passes. The claim boundary follows [Formal Verification Strategy](formal-verification-strategy.md).

## Implementation boundary

[inventory_cpp.py](../../scripts/inventory_cpp.py) already extracts declarations using Clang AST, the configured source roots and supplied compilation databases. It records input hashes and supports snapshot freshness checks. Its output is an inventory of declarations within that scope, not a proof of functions or all template executions.

A reviewed symbol-to-requirement baseline and mandatory per-function review completeness are still planned. The current [agent-review runner](../../scripts/run_agent_review.sh) audits registered requirements; it does not enforce the complete AST pipeline below. Track that integration in [ROADMAP.md](../../ROADMAP.md), without treating this design as completed coverage.

## Methodology

Use Clang's source AST as the primary inventory, then connect discovered functions to verification requirements and existing Lean/ESBMC evidence. Extend the existing traceability and agent-review workflow rather than creating a separate proof registry. [Clang AST](https://clang.llvm.org/docs/IntroductionToTheClangAST.html).

## Inventory boundary and identity

Cover project-authored functions, including internal helpers, within the declared source and configuration scope. External libraries remain explicit dependencies, not implicitly verified project code.

Distinguish function identity, configuration and required property. One function may need several proofs, and one proof may cover several functions; matching function, Lean theorem and ESBMC harness counts is not required.

## Configuration completeness

Track evidence against the supported estimator configurations, backends and numerical profiles. Missing configurations must remain visible; evidence for one profile must not silently cover another.

## Review pipeline

The intended pipeline includes the implemented inventory and the planned mapping/completeness stages:

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
