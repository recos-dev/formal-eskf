# Agent Guidelines

## Project

formal-eskf is a portable C++20 error-state Kalman filter with INS and rotation-only AHRS configurations. Keep the numerical core independent of deployment platforms and linear-algebra backends.

## Specifications

Read the relevant specifications in `docs/spec/` before changing behavior. Keep detailed contracts and equations there, not in this entry file:

- Core/runtime/adapter boundaries: `eskf-software-architecture-and-verification-boundaries.md`.
- Language and runtime constraints: `cpp-portability-and-constraints.md`.
- State conventions: `eskf-state-conventions.md`.
- Linear algebra: `eskf-linear-algebra-abstraction-layer.md`.
- SO(3) and quaternions: `so3-and-unit-quaternion-abstraction-layer.md`.
- Numerical and failure semantics: `scalar-ieee754-and-failure-semantics.md`.
- Static analysis: `cpp-static-analysis-policy.md`.
- Requirements and evidence: `verification-requirements.md`, `formal-verification-strategy.md` and `formal-verification-traceability-and-review-criteria.md`.
- Function inventory and review queue: `cpp-function-inventory-and-formal-proof-review-queue.md`.

Verify paper symbols and equations against the actual reference before citing or implementing them. Report conflicts between specifications, papers and code instead of silently choosing a different model.

## Implementation

- Use fixed-size types and bounded operations. Keep dynamic allocation, exceptions, RTTI and OS facilities outside the shared core.
- Use the existing abstractions; backend types and platform effects must not enter shared-core interfaces or equations.
- Preserve specified behavior, defaults and failure/aliasing contracts unless a change is explicitly in scope.
- Follow [STYLE.md](STYLE.md) and `.clang-format` for C++. Use uppercase script variables and split scripts into readable functions. Avoid unnecessary wrappers, helpers and new scripts.
- Keep platform integration and replay conveniences outside the core. Do not vendor external platform code without approval.

## Verification

- Keep Lean model proofs, bounded ESBMC checks, native tests, static analysis and agent review distinct. They do not constitute a machine-checked Lean-to-C++ refinement.
- Scope claims to the analyzed production paths, assumptions and configurations. Record outstanding abstraction dependencies; do not extend claims to unverified targets.
- Do not weaken requirements or assertions, assume away failures or replace production operations merely to obtain a passing result.
- Maintain affected entries in `formal/agent-review/traceability-map.json`. Distinguish requirements, theorems, harnesses and execution profiles when reporting coverage; do not present their counts as interchangeable.
- Missing or stale evidence, timeouts and incomplete unwinding are gaps, not passes.

Use `README.md` for build setup and `.github/workflows/ci.yml` for CI checks. Reuse the existing `scripts/` entry points and consult their `--help` for options. Run checks appropriate to the change; prefer targeted proof suites over rerunning every proof for every edit.

For verification-tool changes, run `tests/test_esbmc.sh` and/or `tests/test_agent_review.sh` as applicable. Their default modes are offline regression tests, not proof execution or an AI audit.

Use `scripts/format_cpp.sh --check` for read-only formatting checks; its default mode edits files. Cppcheck checks without fixing source. ESBMC `--list` inventories profiles without proving them.

## Collaboration

- Inspect the worktree first; preserve unrelated edits and staged changes. Do not interfere with other ongoing work or verification runs.
- Keep changes scoped to the request. Separate proof, production-code, tooling and documentation changes when that helps review; do not bundle unrelated cleanup.
- Self-review changes before handing them back. Report the result, checks performed and remaining gaps without claiming unrun checks passed.
- Stage, commit, push or rewrite commit metadata only when explicitly requested. When staging, select exact task-related files and review the staged diff; exclude temporary notes and generated outputs.
- Keep documentation concise and avoid duplicating specifications or hardcoding changing proof statistics here.
